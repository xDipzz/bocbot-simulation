#!/usr/bin/env python3

import os
import select
import sys
import termios
import threading
import time
import traceback
import tty

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None

MSG = """
BOCBOT - CINEMATIC ROVER CONTROLLER
-------------------------------------------
W/S : Smooth Acceleration (Forward/Reverse)
A/D : Smooth Steering (Left/Right)
Q/E : Pivot in Place (Left/Right)
R   : Reset Position
Ctrl-C : Quit
-------------------------------------------
"""

SUPPORTED_KEYS = ('w', 's', 'a', 'd', 'q', 'e', 'r', '\x03')
KEY_ORDER = ('w', 's', 'a', 'd', 'q', 'e', 'r', '\x03')
LINEAR_KEYS = {'w', 's'}
STEER_KEYS = {'a', 'd'}
PIVOT_KEYS = {'q', 'e'}


def key_label(key):
    labels = {
        'w': 'W(fwd)',
        's': 'S(rev)',
        'a': 'A(left)',
        'd': 'D(right)',
        'q': 'Q(pivot-L)',
        'e': 'E(pivot-R)',
        'r': 'R(reset)',
        '\x03': 'Ctrl-C',
    }
    return labels.get(key, repr(key))


def ordered_key_labels(keys):
    return [key_label(key) for key in KEY_ORDER if key in keys]


def normalize_key_char(char):
    if char is None:
        return None
    if not isinstance(char, str) or len(char) != 1:
        return None
    normalized = char.lower()
    if normalized in SUPPORTED_KEYS:
        return normalized
    return None


def set_cbreak_noecho(fd, settings):
    attrs = list(settings)
    attrs[3] &= ~(termios.ICANON | termios.ECHO)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
    return settings


class KeyboardBackend:
    name = 'unknown'
    supports_true_hold = False

    def start(self):
        return None

    def get_pressed_keys(self):
        return set()

    def shutdown(self):
        return None


class X11KeyboardBackend(KeyboardBackend):
    name = 'x11-key-listener'
    supports_true_hold = True

    def __init__(self):
        self._lock = threading.Lock()
        self._pressed_keys = set()
        self._listener = pynput_keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )

    def _normalize_key(self, key):
        normalized = normalize_key_char(getattr(key, 'char', None))
        return normalized

    def _on_press(self, key):
        normalized = self._normalize_key(key)
        if normalized is None:
            return
        with self._lock:
            self._pressed_keys.add(normalized)

    def _on_release(self, key):
        normalized = self._normalize_key(key)
        if normalized is None:
            return
        with self._lock:
            self._pressed_keys.discard(normalized)

    def start(self):
        self._listener.start()

    def get_pressed_keys(self):
        with self._lock:
            return set(self._pressed_keys)

    def shutdown(self):
        self._listener.stop()


class TerminalRepeatKeyboardBackend(KeyboardBackend):
    name = 'tty-repeat-fallback'
    supports_true_hold = False

    def __init__(self, settings, hold_timeout=0.18, read_timeout=0.02):
        self.settings = settings
        self.hold_timeout = hold_timeout
        self.read_timeout = read_timeout
        self._last_seen = {}

    def _read_events(self):
        tty.setraw(sys.stdin.fileno())
        events = []
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], self.read_timeout)
            if rlist:
                events.append(sys.stdin.read(1))
                while True:
                    rlist_inner, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if not rlist_inner:
                        break
                    events.append(sys.stdin.read(1))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return events

    def _clear_conflicts(self, key):
        if key in LINEAR_KEYS:
            conflict_keys = LINEAR_KEYS - {key}
        elif key in STEER_KEYS:
            conflict_keys = STEER_KEYS - {key}
        elif key in PIVOT_KEYS:
            conflict_keys = PIVOT_KEYS - {key}
        else:
            conflict_keys = set()

        for conflict_key in conflict_keys:
            self._last_seen.pop(conflict_key, None)

    def get_pressed_keys(self):
        now = time.monotonic()
        for event in self._read_events():
            normalized = normalize_key_char(event)
            if normalized is None:
                continue
            self._clear_conflicts(normalized)
            self._last_seen[normalized] = now

        active_keys = set()
        stale_keys = []
        for key, last_seen in self._last_seen.items():
            if now - last_seen <= self.hold_timeout:
                active_keys.add(key)
            else:
                stale_keys.append(key)

        for key in stale_keys:
            self._last_seen.pop(key, None)

        return active_keys


def create_keyboard_backend(log, settings):
    if pynput_keyboard is not None and os.environ.get('DISPLAY'):
        try:
            backend = X11KeyboardBackend()
            backend.start()
            set_cbreak_noecho(sys.stdin.fileno(), settings)
            print('[STARTUP] Input backend: X11 key listener (real key hold + multi-key combos)')
            log.info('Input backend: X11 key listener (real key hold + multi-key combos)')
            return backend
        except Exception as exc:
            print('[STARTUP] X11 key listener unavailable, falling back to terminal repeat mode')
            log.warn(f'X11 key listener unavailable: {exc}')

    backend = TerminalRepeatKeyboardBackend(settings)
    print('[STARTUP] Input backend: terminal repeat fallback (limited multi-key hold)')
    log.warn('Input backend: terminal repeat fallback - simultaneous key hold is limited')
    return backend


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.log = self.get_logger()

        self.log.info('==== TELEOP NODE STARTING ====')
        self.log.info('Node name: teleop_wasd')
        self.log.info('Publish topic: /cmd_vel (geometry_msgs/Twist)')
        self.log.info('Reset service: /gazebo/set_entity_state')

    def check_subscribers(self):
        return self.publisher_.get_subscription_count()

    def wait_for_cmd_vel_subscribers(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        highest_count = 0

        while time.monotonic() < deadline:
            sub_count = self.check_subscribers()
            highest_count = max(highest_count, sub_count)
            if sub_count > 0:
                return sub_count
            rclpy.spin_once(self, timeout_sec=0.1)

        return highest_count

    def reset_robot(self):
        if not self.set_state_client.service_is_ready():
            self.log.warn('RESET FAILED: /gazebo/set_entity_state service NOT available')
            self.log.warn('Ensure libgazebo_ros_state.so is loaded in launch file')
            print('[WARN] Reset service not available - is libgazebo_ros_state.so loaded?')
            return False

        req = SetEntityState.Request()
        req.state.name = 'bocbot'
        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.8
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'

        self.log.info('RESET: Requesting teleport to (0.0, 0.0, 0.8)')
        future = self.set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.log.error('RESET FAILED: service call timed out')
            print('[ERROR] Reset request timed out')
            return False

        if future.exception() is not None:
            self.log.error(f'RESET FAILED: service exception: {future.exception()}')
            print(f'[ERROR] Reset service exception: {future.exception()}')
            return False

        response = future.result()
        if response is None:
            self.log.error('RESET FAILED: empty service response')
            print('[ERROR] Reset failed: empty service response')
            return False

        if not response.success:
            status_message = getattr(response, 'status_message', '') or 'no additional detail'
            self.log.error(f'RESET FAILED: Gazebo rejected teleport: {status_message}')
            print(f'[ERROR] Reset failed: {status_message}')
            return False

        self.log.info('RESET: Teleported bocbot to (0.0, 0.0, 0.8)')
        print('[RESET] Robot teleported to origin')
        return True


def main(args=None):
    rclpy.init(args=args)

    settings = None
    keyboard_backend = None
    frame_count = 0
    node = Teleop()
    log = node.log

    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)

    # --- TUNING PARAMETERS ---
    max_speed = 4.0
    max_turn = 3.5
    lin_accel = 3.0
    lin_decel = 4.0
    ang_accel = 5.0
    ang_decel = 8.0
    turn_scale_at_max = 0.4

    log.info('==== TUNING PARAMETERS ====')
    log.info(f'  MAX_SPEED         = {max_speed} m/s')
    log.info(f'  MAX_TURN          = {max_turn} rad/s')
    log.info(f'  LIN_ACCEL         = {lin_accel} m/s^2')
    log.info(f'  LIN_DECEL         = {lin_decel} m/s^2')
    log.info(f'  ANG_ACCEL         = {ang_accel} rad/s^2')
    log.info(f'  ANG_DECEL         = {ang_decel} rad/s^2')
    log.info(f'  TURN_SCALE_AT_MAX = {turn_scale_at_max}')
    log.info('===========================')

    # --- STATE ---
    current_speed = 0.0
    current_turn = 0.0
    last_time = time.monotonic()
    last_status_time = 0.0
    last_keys = set()
    sub_warned = False

    try:
        if settings is None:
            raise RuntimeError('stdin is not a tty; teleop requires an interactive terminal')

        keyboard_backend = create_keyboard_backend(log, settings)

        print('[STARTUP] Checking connections...')
        print('[STARTUP] Waiting up to 5s for /cmd_vel subscriber...')
        log.info('Waiting for /cmd_vel subscriber (5s timeout)...')
        sub_count = node.wait_for_cmd_vel_subscribers(timeout_sec=5.0)
        if sub_count > 0:
            print(f'[STARTUP] /cmd_vel subscribers: {sub_count} (diff_drive plugin connected)')
            log.info(f'/cmd_vel has {sub_count} subscriber(s) - OK')
        else:
            print('[STARTUP] /cmd_vel subscribers: 0 (robot will not move)')
            print('[STARTUP] Check that the diff_drive plugin loaded and the robot spawned cleanly.')
            log.error('No subscribers on /cmd_vel after 5s wait')

        print('[STARTUP] Waiting up to 5s for reset service...')
        log.info('Waiting for /gazebo/set_entity_state (5s timeout)...')
        if node.set_state_client.wait_for_service(timeout_sec=5.0):
            print('[STARTUP] Reset service: READY')
            log.info('Reset service: READY')
        else:
            print('[STARTUP] Reset service: NOT AVAILABLE (R key will not work)')
            log.warn('Reset service not found after 5s - R key disabled')

        print(MSG)
        print('[RUNNING] Control loop active. Logs print every 0.5s.\n')
        log.info('==== CONTROL LOOP STARTED ====')

        while True:
            now = time.monotonic()
            dt = min(now - last_time, 0.1)
            last_time = now
            frame_count += 1

            keys = keyboard_backend.get_pressed_keys()
            new_keys = keys - last_keys

            if keys != last_keys:
                if keys:
                    labels = ordered_key_labels(keys)
                    log.info(f'KEY INPUT: {" + ".join(labels)}')
                elif last_keys:
                    log.info('KEY INPUT: [all released]')

            if '\x03' in keys:
                log.info('Ctrl-C detected -> shutting down')
                print('\n[EXIT] Ctrl-C pressed, stopping robot...')
                break

            if 'r' in new_keys:
                if node.reset_robot():
                    current_speed = 0.0
                    current_turn = 0.0
                    zero_twist = Twist()
                    node.publisher_.publish(zero_twist)

            target_speed = 0.0
            if 'w' in keys:
                target_speed = max_speed
            elif 's' in keys:
                target_speed = -max_speed

            target_turn = 0.0
            pivoting = False

            if 'q' in keys:
                target_turn = max_turn
                target_speed = 0.0
                pivoting = True
            elif 'e' in keys:
                target_turn = -max_turn
                target_speed = 0.0
                pivoting = True
            elif 'a' in keys:
                target_turn = max_turn
            elif 'd' in keys:
                target_turn = -max_turn

            if not pivoting and abs(current_speed) > 0.1:
                speed_ratio = min(abs(current_speed) / max_speed, 1.0)
                turn_scale = 1.0 - speed_ratio * (1.0 - turn_scale_at_max)
                target_turn *= turn_scale

            if pivoting:
                current_speed = 0.0
            elif current_speed < target_speed:
                linear_rate = lin_accel if target_speed > 0 else lin_decel
                current_speed = min(target_speed, current_speed + linear_rate * dt)
            elif current_speed > target_speed:
                linear_rate = lin_accel if target_speed < 0 else lin_decel
                current_speed = max(target_speed, current_speed - linear_rate * dt)

            if current_turn < target_turn:
                angular_rate = ang_accel if abs(target_turn) >= abs(current_turn) else ang_decel
                current_turn = min(target_turn, current_turn + angular_rate * dt)
            elif current_turn > target_turn:
                angular_rate = ang_accel if abs(target_turn) >= abs(current_turn) else ang_decel
                current_turn = max(target_turn, current_turn - angular_rate * dt)

            twist = Twist()
            twist.linear.x = current_speed
            twist.angular.z = current_turn
            node.publisher_.publish(twist)

            if now - last_status_time >= 0.5:
                last_status_time = now
                sub_count = node.check_subscribers()

                if sub_count == 0 and not sub_warned:
                    print('[WARN] /cmd_vel has 0 subscribers! Robot will not move.')
                    log.warn('/cmd_vel lost all subscribers')
                    sub_warned = True
                elif sub_count > 0 and sub_warned:
                    print('[OK] /cmd_vel subscriber reconnected')
                    log.info('/cmd_vel subscriber reconnected')
                    sub_warned = False

                moving = abs(current_speed) > 0.01 or abs(current_turn) > 0.01
                state = 'MOVING' if moving else 'IDLE'
                pivot_suffix = ' PIVOT' if pivoting else ''
                active_keys = ordered_key_labels(keys)
                key_text = 'none' if not active_keys else '+'.join(active_keys)

                print(
                    f'  [{state}{pivot_suffix}] '
                    f'keys={key_text} '
                    f'spd={current_speed:+.2f}/{target_speed:+.2f} '
                    f'trn={current_turn:+.2f}/{target_turn:+.2f} '
                    f'dt={dt:.3f} '
                    f'subs={sub_count} '
                    f'f={frame_count}'
                )

                log.info(
                    f'STATUS: {state}{pivot_suffix} | '
                    f'keys={key_text} | '
                    f'speed={current_speed:+.2f} (target={target_speed:+.2f}) | '
                    f'turn={current_turn:+.2f} (target={target_turn:+.2f}) | '
                    f'twist=[{twist.linear.x:.2f}, {twist.angular.z:.2f}] | '
                    f'backend={keyboard_backend.name} | '
                    f'subs={sub_count} | dt={dt:.4f} | frame={frame_count}'
                )

            rclpy.spin_once(node, timeout_sec=0)
            last_keys = keys.copy()

            # Cap loop at ~50 Hz to prevent CPU-heavy busy spinning
            time.sleep(0.02)

    except KeyboardInterrupt:
        log.info('KeyboardInterrupt received -> shutting down')
        print('\n[EXIT] Ctrl-C pressed, stopping robot...')
    except Exception as exc:
        log.error(f'EXCEPTION in control loop: {type(exc).__name__}: {exc}')
        print(f'\n[ERROR] {type(exc).__name__}: {exc}')
        log.error(traceback.format_exc())
    finally:
        stop_twist = Twist()
        node.publisher_.publish(stop_twist)
        log.info(f'SHUTDOWN: Sent zero twist. Total frames: {frame_count}')
        print(f'[SHUTDOWN] Sent stop command. Frames processed: {frame_count}')

        if keyboard_backend is not None:
            keyboard_backend.shutdown()

        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState

msg = """
BOCBOT - CINEMATIC ROVER CONTROLLER
-------------------------------------------
W/S : Smooth Acceleration (Forward/Reverse)
A/D : Smooth Steering (Left/Right)
Q/E : Pivot in Place (Left/Right)
R   : Reset Position
Ctrl-C : Quit
-------------------------------------------
"""

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
        """Check if anything is listening to /cmd_vel."""
        count = self.publisher_.get_subscription_count()
        return count

    def reset_robot(self):
        if not self.set_state_client.service_is_ready():
            self.log.warn('RESET FAILED: /gazebo/set_entity_state service NOT available')
            self.log.warn('Ensure libgazebo_ros_state.so is loaded in launch file')
            print('[WARN] Reset service not available - is libgazebo_ros_state.so loaded?')
            return False
        req = SetEntityState.Request()
        req.state.name = 'bocbot'
        req.state.pose.position.z = 0.8
        req.state.pose.orientation.w = 1.0
        self.set_state_client.call_async(req)
        self.log.info('RESET: Teleported bocbot to (0, 0, 0.8)')
        print('[RESET] Robot teleported to origin')
        return True

def getKeys(settings):
    """Read all currently buffered key presses as a set."""
    tty.setraw(sys.stdin.fileno())
    keys = set()
    rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
    if rlist:
        keys.add(sys.stdin.read(1))
        while True:
            rlist_inner, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist_inner:
                keys.add(sys.stdin.read(1))
            else:
                break
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return keys

def key_label(k):
    """Human-readable label for a key."""
    labels = {
        'w': 'W(fwd)', 's': 'S(rev)', 'a': 'A(left)', 'd': 'D(right)',
        'q': 'Q(pivot-L)', 'e': 'E(pivot-R)', 'r': 'R(reset)', '\x03': 'Ctrl-C',
    }
    return labels.get(k, repr(k))

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = Teleop()
    log = node.log

    # --- TUNING PARAMETERS ---
    MAX_SPEED = 4.0
    MAX_TURN  = 3.5
    LIN_ACCEL = 3.0
    LIN_DECEL = 4.0
    ANG_ACCEL = 5.0
    ANG_DECEL = 8.0
    TURN_SCALE_AT_MAX = 0.4

    log.info('==== TUNING PARAMETERS ====')
    log.info(f'  MAX_SPEED      = {MAX_SPEED} m/s')
    log.info(f'  MAX_TURN       = {MAX_TURN} rad/s')
    log.info(f'  LIN_ACCEL      = {LIN_ACCEL} m/s^2')
    log.info(f'  LIN_DECEL      = {LIN_DECEL} m/s^2')
    log.info(f'  ANG_ACCEL      = {ANG_ACCEL} rad/s^2')
    log.info(f'  ANG_DECEL      = {ANG_DECEL} rad/s^2')
    log.info(f'  TURN_SCALE_AT_MAX = {TURN_SCALE_AT_MAX}')
    log.info('===========================')

    # --- STATE ---
    current_speed = 0.0
    current_turn  = 0.0
    last_time = time.monotonic()
    frame_count = 0
    last_status_time = 0.0
    last_keys = set()
    sub_warned = False

    # --- STARTUP CHECKS ---
    print('[STARTUP] Checking connections...')

    # Check /cmd_vel subscribers
    sub_count = node.check_subscribers()
    if sub_count > 0:
        print(f'[STARTUP] /cmd_vel subscribers: {sub_count} (diff_drive plugin connected)')
        log.info(f'/cmd_vel has {sub_count} subscriber(s) - OK')
    else:
        print('[STARTUP] /cmd_vel subscribers: 0 (WARNING: nobody listening, robot wont move!)')
        log.warn('/cmd_vel has 0 subscribers - diff_drive plugin may not be loaded')

    # Check reset service
    print('[STARTUP] Waiting up to 5s for reset service...')
    log.info('Waiting for /gazebo/set_entity_state (5s timeout)...')
    if node.set_state_client.wait_for_service(timeout_sec=5.0):
        print('[STARTUP] Reset service: READY')
        log.info('Reset service: READY')
    else:
        print('[STARTUP] Reset service: NOT AVAILABLE (R key will not work)')
        log.warn('Reset service not found after 5s - R key disabled')

    # Re-check subscribers after waiting (plugin may have started)
    sub_count = node.check_subscribers()
    if sub_count == 0:
        print('[STARTUP] WARNING: Still 0 subscribers on /cmd_vel!')
        print('[STARTUP] The robot will NOT respond to commands.')
        print('[STARTUP] Check that the URDF diff_drive plugin loaded correctly.')
        log.error('No subscribers on /cmd_vel after startup checks')
    else:
        log.info(f'/cmd_vel now has {sub_count} subscriber(s)')

    try:
        print(msg)
        print('[RUNNING] Control loop active. Logs print every 0.5s.\n')
        log.info('==== CONTROL LOOP STARTED ====')

        while True:
            now = time.monotonic()
            dt = min(now - last_time, 0.1)
            last_time = now
            frame_count += 1

            keys = getKeys(settings)

            # --- LOG KEY CHANGES ---
            if keys != last_keys:
                if keys:
                    labels = [key_label(k) for k in keys]
                    log.info(f'KEY INPUT: {" + ".join(labels)}')
                elif last_keys:
                    log.info('KEY INPUT: [all released]')
                last_keys = keys.copy()

            if '\x03' in keys:
                log.info('Ctrl-C detected -> shutting down')
                print('\n[EXIT] Ctrl-C pressed, stopping robot...')
                break

            if 'r' in keys:
                node.reset_robot()

            # --- LINEAR TARGET ---
            target_speed = 0.0
            if 'w' in keys:
                target_speed = MAX_SPEED
            elif 's' in keys:
                target_speed = -MAX_SPEED

            # --- ANGULAR TARGET ---
            target_turn = 0.0
            pivoting = False

            if 'q' in keys:
                target_turn = MAX_TURN
                target_speed = 0.0
                pivoting = True
            elif 'e' in keys:
                target_turn = -MAX_TURN
                target_speed = 0.0
                pivoting = True
            elif 'a' in keys:
                target_turn = MAX_TURN
            elif 'd' in keys:
                target_turn = -MAX_TURN

            # Speed-dependent turn reduction
            if not pivoting and abs(current_speed) > 0.1:
                speed_ratio = abs(current_speed) / MAX_SPEED
                turn_scale = 1.0 - speed_ratio * (1.0 - TURN_SCALE_AT_MAX)
                target_turn *= turn_scale

            # --- SMOOTH LINEAR RAMP ---
            if pivoting:
                current_speed = 0.0
            elif current_speed < target_speed:
                rate = LIN_ACCEL if target_speed > 0 else LIN_DECEL
                current_speed = min(target_speed, current_speed + rate * dt)
            elif current_speed > target_speed:
                rate = LIN_ACCEL if target_speed < 0 else LIN_DECEL
                current_speed = max(target_speed, current_speed - rate * dt)

            # --- SMOOTH ANGULAR RAMP ---
            if current_turn < target_turn:
                ang_rate = ANG_ACCEL if abs(target_turn) >= abs(current_turn) else ANG_DECEL
                current_turn = min(target_turn, current_turn + ang_rate * dt)
            elif current_turn > target_turn:
                ang_rate = ANG_ACCEL if abs(target_turn) >= abs(current_turn) else ANG_DECEL
                current_turn = max(target_turn, current_turn - ang_rate * dt)

            # --- PUBLISH ---
            twist = Twist()
            twist.linear.x = current_speed
            twist.angular.z = current_turn
            node.publisher_.publish(twist)

            # --- PERIODIC STATUS LOG (every 0.5s) ---
            if now - last_status_time >= 0.5:
                last_status_time = now

                # Check subscriber health periodically
                sub_count = node.check_subscribers()
                if sub_count == 0 and not sub_warned:
                    print('[WARN] /cmd_vel has 0 subscribers! Robot will not move.')
                    log.warn('/cmd_vel lost all subscribers')
                    sub_warned = True
                elif sub_count > 0 and sub_warned:
                    print('[OK] /cmd_vel subscriber reconnected')
                    log.info('/cmd_vel subscriber reconnected')
                    sub_warned = False

                # Build status for xterm
                moving = abs(current_speed) > 0.01 or abs(current_turn) > 0.01
                state = 'MOVING' if moving else 'IDLE'
                piv = ' PIVOT' if pivoting else ''

                print(
                    f'  [{state}{piv}] '
                    f'spd={current_speed:+.2f}/{target_speed:+.2f} '
                    f'trn={current_turn:+.2f}/{target_turn:+.2f} '
                    f'dt={dt:.3f} '
                    f'subs={sub_count} '
                    f'f={frame_count}'
                )

                # Full ROS log
                log.info(
                    f'STATUS: {state}{piv} | '
                    f'speed={current_speed:+.2f} (target={target_speed:+.2f}) | '
                    f'turn={current_turn:+.2f} (target={target_turn:+.2f}) | '
                    f'twist=[{twist.linear.x:.2f}, {twist.angular.z:.2f}] | '
                    f'subs={sub_count} | dt={dt:.4f} | frame={frame_count}'
                )

            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        log.error(f'EXCEPTION in control loop: {type(e).__name__}: {e}')
        print(f'\n[ERROR] {type(e).__name__}: {e}')
        import traceback
        log.error(traceback.format_exc())
    finally:
        stop_twist = Twist()
        node.publisher_.publish(stop_twist)
        log.info(f'SHUTDOWN: Sent zero twist. Total frames: {frame_count}')
        print(f'[SHUTDOWN] Sent stop command. Frames processed: {frame_count}')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

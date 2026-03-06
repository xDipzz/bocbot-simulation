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
        self.get_logger().info('Teleop node initialized')
        self.get_logger().info('Publishing Twist on: /cmd_vel')
        self.get_logger().info('Reset service: /gazebo/set_entity_state')

    def reset_robot(self):
        if not self.set_state_client.service_is_ready():
            self.get_logger().warn('RESET FAILED: /gazebo/set_entity_state service NOT available')
            print('[WARN] Reset service not available - is libgazebo_ros_state.so loaded?')
            return False
        req = SetEntityState.Request()
        req.state.name = 'bocbot'
        req.state.pose.position.z = 0.8
        req.state.pose.orientation.w = 1.0
        self.set_state_client.call_async(req)
        self.get_logger().info('RESET: Teleported bocbot to (0, 0, 0.8)')
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

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = Teleop()

    # --- TUNING PARAMETERS ---
    MAX_SPEED = 4.0          # m/s maximum linear velocity
    MAX_TURN  = 3.5          # rad/s maximum angular velocity

    LIN_ACCEL = 3.0          # How fast the rover speeds up (m/s per second)
    LIN_DECEL = 4.0          # How fast the rover brakes (m/s per second)
    ANG_ACCEL = 5.0          # How fast steering ramps up (rad/s per second)
    ANG_DECEL = 8.0          # How fast steering centers back (rad/s per second)

    TURN_SCALE_AT_MAX = 0.4  # Turn power fraction at max speed

    # Log all tuning parameters at startup
    node.get_logger().info('=== TUNING PARAMETERS ===')
    node.get_logger().info(f'  MAX_SPEED={MAX_SPEED} m/s, MAX_TURN={MAX_TURN} rad/s')
    node.get_logger().info(f'  LIN_ACCEL={LIN_ACCEL}, LIN_DECEL={LIN_DECEL}')
    node.get_logger().info(f'  ANG_ACCEL={ANG_ACCEL}, ANG_DECEL={ANG_DECEL}')
    node.get_logger().info(f'  TURN_SCALE_AT_MAX={TURN_SCALE_AT_MAX}')
    node.get_logger().info('=========================')

    # --- STATE ---
    current_speed = 0.0
    current_turn  = 0.0
    last_time = time.monotonic()
    frame_count = 0
    last_status_time = 0.0
    last_keys = set()

    # Check if reset service is available at startup
    print('[STARTUP] Waiting up to 5s for reset service...')
    node.get_logger().info('Waiting for /gazebo/set_entity_state service...')
    if node.set_state_client.wait_for_service(timeout_sec=5.0):
        print('[STARTUP] Reset service: READY')
        node.get_logger().info('Reset service: READY')
    else:
        print('[STARTUP] Reset service: NOT AVAILABLE (R key will not work)')
        node.get_logger().warn('Reset service NOT available after 5s timeout')

    try:
        print(msg)
        print('[RUNNING] Listening for key input...\n')
        node.get_logger().info('Teleop control loop started')

        while True:
            now = time.monotonic()
            dt = min(now - last_time, 0.1)
            last_time = now
            frame_count += 1

            keys = getKeys(settings)

            # Log key changes
            if keys != last_keys:
                if keys:
                    key_names = []
                    for k in keys:
                        if k == '\x03': key_names.append('Ctrl-C')
                        else: key_names.append(k.upper())
                    node.get_logger().info(f'Keys pressed: {", ".join(key_names)}')
                last_keys = keys.copy()

            if '\x03' in keys:
                node.get_logger().info('Ctrl-C detected, shutting down')
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

                # Build compact status line for xterm
                status_parts = []
                if abs(current_speed) > 0.01:
                    status_parts.append(f'speed={current_speed:+.2f}')
                if abs(current_turn) > 0.01:
                    status_parts.append(f'turn={current_turn:+.2f}')
                if pivoting:
                    status_parts.append('PIVOT')

                if status_parts:
                    status = ' | '.join(status_parts)
                    print(f'  [{status}]  dt={dt:.3f}s  frame={frame_count}')

                # Detailed ROS log
                node.get_logger().debug(
                    f'frame={frame_count} dt={dt:.4f} '
                    f'target=[spd={target_speed:.2f} trn={target_turn:.2f}] '
                    f'current=[spd={current_speed:.2f} trn={current_turn:.2f}] '
                    f'twist=[lin.x={twist.linear.x:.2f} ang.z={twist.angular.z:.2f}]'
                )

            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        node.get_logger().error(f'Exception in control loop: {e}')
        print(f'\n[ERROR] {e}')
    finally:
        # Send zero velocity
        stop_twist = Twist()
        node.publisher_.publish(stop_twist)
        node.get_logger().info(f'Sent zero twist. Total frames: {frame_count}')
        print(f'[SHUTDOWN] Sent stop command. Total frames processed: {frame_count}')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

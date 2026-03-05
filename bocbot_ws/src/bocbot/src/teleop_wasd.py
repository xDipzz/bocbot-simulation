#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

msg = """
🎮 BUNCH OF CODERS ROBOT - RC CONTROLLER 🎮
-------------------------------------------
Smooth RC Car driving mode!

        W (Forward)
A (Left)    S (Reverse)    D (Right)

Hold W+A or W+D to drive and turn smoothly!

Other keys:
R : Reset robot to starting position
Ctrl-C : Quit
"""

forward = 1.0
backward = -1.0
left = 1.0
right = -1.0

def getKey(settings, timeout=0.03):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.set_state_client.wait_for_service(timeout_sec=2.0)

    def reset_robot(self):
        if not self.set_state_client.service_is_ready():
            self.get_logger().warn('Reset service /gazebo/set_entity_state not available')
            return
            
        self.get_logger().info('Resetting robot to default position...')
        req = SetEntityState.Request()
        req.state.name = 'bocbot'
        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.1
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
        self.set_state_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = Teleop()

    speed = 1.2  # Max linear speed
    turn = 2.0   # Max angular speed
    
    current_x = 0.0
    current_th = 0.0
    target_x = 0.0
    target_th = 0.0
    
    # Acceleration factors (how fast it reaches target speed)
    accel_x = 0.08
    accel_th = 0.15
    decel_x = 0.1
    decel_th = 0.2

    # Timers to detect when keys are released
    last_x_time = time.time()
    last_th_time = time.time()
    key_timeout = 0.12  # If a key isn't seen for this long, consider it released

    try:
        print(msg)
        while True:
            key = getKey(settings, timeout=0.02)
            now = time.time()
            
            # Process incoming key
            if key == 'w':
                target_x = forward
                last_x_time = now
            elif key == 's':
                target_x = backward
                last_x_time = now
            elif key == 'a':
                target_th = left
                last_th_time = now
            elif key == 'd':
                target_th = right
                last_th_time = now
            elif key == 'r':
                node.reset_robot()
                current_x = 0.0
                current_th = 0.0
                target_x = 0.0
                target_th = 0.0
            elif key == '\x03': # ctrl-c
                break

            # If haven't seen a linear key in a bit, set target to 0
            if now - last_x_time > key_timeout:
                target_x = 0.0
                
            # If haven't seen an angular key in a bit, set target to 0
            if now - last_th_time > key_timeout:
                target_th = 0.0

            # Smoothly interpolate current linear speed towards target
            if target_x != 0.0:
                # Accelerating
                if current_x < target_x:
                    current_x = min(target_x, current_x + accel_x)
                elif current_x > target_x:
                    current_x = max(target_x, current_x - accel_x)
            else:
                # Decelerating
                if current_x > 0.0:
                    current_x = max(0.0, current_x - decel_x)
                elif current_x < 0.0:
                    current_x = min(0.0, current_x + decel_x)

            # Smoothly interpolate current angular speed towards target
            if target_th != 0.0:
                # Accelerating
                if current_th < target_th:
                    current_th = min(target_th, current_th + accel_th)
                elif current_th > target_th:
                    current_th = max(target_th, current_th - accel_th)
            else:
                # Decelerating
                if current_th > 0.0:
                    current_th = max(0.0, current_th - decel_th)
                elif current_th < 0.0:
                    current_th = min(0.0, current_th + decel_th)

            # Apply limits to fix floating point math drift near 0
            if abs(current_x) < 0.001: current_x = 0.0
            if abs(current_th) < 0.001: current_th = 0.0

            # Publish
            twist = Twist()
            twist.linear.x = current_x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = current_th * turn
            node.publisher_.publish(twist)
            
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.publisher_.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

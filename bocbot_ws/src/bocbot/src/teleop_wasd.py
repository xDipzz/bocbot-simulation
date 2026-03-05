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
🎮 BUNCH OF CODERS ROBOT - PERFORMANCE CONTROLLER 🎮
-------------------------------------------
W/S : Smooth Acceleration (Forward/Reverse)
A/D : Instant Steering (Left/Right)
R   : Reset Position
Ctrl-C : Quit
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

    def reset_robot(self):
        if self.set_state_client.service_is_ready():
            req = SetEntityState.Request()
            req.state.name = 'bocbot'
            req.state.pose.position.z = 0.8
            req.state.pose.orientation.w = 1.0
            self.set_state_client.call_async(req)

def getKeys(settings):
    tty.setraw(sys.stdin.fileno())
    keys = []
    rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
    if rlist:
        keys.append(sys.stdin.read(1))
        while True:
            rlist_inner, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist_inner:
                keys.append(sys.stdin.read(1))
            else:
                break
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return keys

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = Teleop()

    # TANK/ROVER TUNING
    MAX_SPEED = 4.0
    MAX_TURN = 3.5 # Boosted turning speed
    
    current_speed = 0.0
    
    # Speed Ramps
    ACCEL = 0.4
    DECEL = 0.5

    try:
        print(msg)
        while True:
            keys = getKeys(settings)
            
            if '\x03' in keys: break # Ctrl-C
            if 'r' in keys: node.reset_robot()

            # --- LINEAR LOGIC (Smooth) ---
            target_speed = 0.0
            if 'w' in keys: target_speed = MAX_SPEED
            elif 's' in keys: target_speed = -MAX_SPEED

            if current_speed < target_speed:
                current_speed = min(target_speed, current_speed + ACCEL)
            elif current_speed > target_speed:
                current_speed = max(target_speed, current_speed - DECEL)

            # --- ANGULAR LOGIC (Instant/Digital) ---
            # Removing the ramp from angular velocity makes steering feel much more direct
            turn_vel = 0.0
            if 'a' in keys: turn_vel = MAX_TURN
            elif 'd' in keys: turn_vel = -MAX_TURN

            # Publish
            twist = Twist()
            twist.linear.x = current_speed
            twist.angular.z = turn_vel
            node.publisher_.publish(twist)
            
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)
    finally:
        node.publisher_.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

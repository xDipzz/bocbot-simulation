#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_sensor_data

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class AutonomousTourNode(Node):
    def __init__(self):
        super().__init__('autonomous_tour')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Waypoints for the 10-room tour
        self.waypoints = [
            ("Room 1 (Reception)", -8.0, 4.0),
            ("Room 2 (Office 1)", -10.0, 12.0),
            ("Room 3 (Conf Room)", -10.0, 20.0),
            ("Room 4 (Office 2)", -10.0, 28.0),
            ("Room 5 (Lounge)", -10.0, 35.0),
            ("Bridge Approach", 0.0, 10.0),
            ("Bridge Platform (Ascending)", 0.0, 20.0),
            ("Bridge Descend", 0.0, 30.0),
            ("Room 10 (Lab)", 10.0, 35.0),
            ("Room 9 (Office 4)", 10.0, 28.0),
            ("Room 8 (Server Room)", 10.0, 20.0),
            ("Room 7 (Office 3)", 10.0, 12.0),
            ("Room 6 (Workspace)", 8.0, 4.0),
            ("Entrance / Finish", 0.0, 0.0)
        ]
        
        self.current_wp_index = 0
        self.state = "NAVIGATING" # "NAVIGATING" or "WAITING"
        self.wait_start_time = 0.0
        self.wait_duration = 3.0 # seconds to pause at each room
        
        self.get_logger().info('Autonomous Tour Node Started.')
        self.get_logger().info(f'Next destination: {self.waypoints[self.current_wp_index][0]}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            self.get_logger().info('Waiting for /odom messages...', throttle_duration_sec=2.0)
            return

        if self.current_wp_index >= len(self.waypoints):
            # Tour complete
            msg = Twist()
            self.publisher_.publish(msg)
            self.get_logger().info('Tour Complete!')
            rclpy.shutdown()
            return

        name, tx, ty = self.waypoints[self.current_wp_index]
        
        if self.state == "WAITING":
            msg = Twist()
            self.publisher_.publish(msg)
            
            if time.time() - self.wait_start_time >= self.wait_duration:
                self.current_wp_index += 1
                if self.current_wp_index < len(self.waypoints):
                    self.state = "NAVIGATING"
                    next_name = self.waypoints[self.current_wp_index][0]
                    self.get_logger().info(f'Moving to: {next_name}')
            return

        # Calculate distance and angle to target
        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.hypot(dx, dy)
        
        target_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(target_angle - self.current_yaw), math.cos(target_angle - self.current_yaw))

        msg = Twist()

        if distance < 0.6:
            # Reached waypoint
            self.get_logger().info(f'Arrived at {name}. Pausing for {self.wait_duration}s.')
            self.state = "WAITING"
            self.wait_start_time = time.time()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            # Navigation control logic
            # If we are facing the wrong way, pivot first
            if abs(angle_error) > 0.4:
                msg.linear.x = 0.0
                # Proportional turn, clamped
                msg.angular.z = max(min(angle_error * 2.0, 1.5), -1.5)
            else:
                # Move forward while adjusting angle
                speed = max(min(distance, 1.2), 0.2) # min speed 0.2, max 1.2
                
                # If climbing bridge, apply more speed to overcome stairs
                if "Bridge" in name:
                    speed = 2.0
                    
                msg.linear.x = speed
                msg.angular.z = angle_error * 2.5

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousTourNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Stop robot on exit
        msg = Twist()
        node.publisher_.publish(msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

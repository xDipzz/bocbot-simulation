#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import rclpy


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutonomousTourNode(Node):
    NAVIGATE = "NAVIGATING"
    WAITING = "WAITING"
    BACKUP = "BACKUP_RECOVERY"
    TURN = "TURN_RECOVERY"
    DONE = "DONE"

    def __init__(self):
        super().__init__('autonomous_tour')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/bocbot/local_map', 1)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/bocbot/scan',
            self._scan_callback,
            qos_profile_sensor_data
        )

        self.control_timer = self.create_timer(0.08, self._control_loop)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.scan = None
        self.scan_received = False
        self.scan_time = None

        self.waypoints = [
            ("Room 1 (Reception)", -8.0, 4.0, True),
            ("Room 2 (Office 1)", -10.0, 12.0, True),
            ("Room 3 (Conf Room)", -10.0, 20.0, True),
            ("Room 4 (Office 2)", -10.0, 28.0, True),
            ("Room 5 (Lounge)", -10.0, 35.0, True),
            ("Bridge Approach", 0.0, 10.0, False),
            ("Bridge Platform (Ascending)", 0.0, 20.0, False),
            ("Bridge Descend", 0.0, 30.0, False),
            ("Room 10 (Lab)", 10.0, 35.0, True),
            ("Room 9 (Office 4)", 10.0, 28.0, True),
            ("Room 8 (Server Room)", 10.0, 20.0, True),
            ("Room 7 (Office 3)", 10.0, 12.0, True),
            ("Room 6 (Workspace)", 8.0, 4.0, True),
            ("Entrance / Finish", 0.0, 0.0, True),
        ]

        self.current_wp_index = 0
        self.state = self.NAVIGATE
        self.wait_start = 0.0
        self.wait_duration = 3.0

        self.waypoint_tolerance = 0.55
        self.max_linear_speed = 1.0
        self.bridge_speed = 1.1
        self.min_linear_speed = 0.08
        self.max_angular_speed = 1.6
        self.align_threshold = math.radians(14.0)
        self.align_scale = 2.0
        self.drive_scale = 1.2

        self.front_limit_stop = 0.55
        self.front_limit_slow = 1.05
        self.side_limit_warn = 0.85
        self.side_limit_critical = 0.55
        self.avoid_gain = 2.4
        self.obstacle_turn_gain = 1.1

        self.stuck_progress_window = 1.6
        self.stuck_motion_threshold = 0.035
        self.no_progress_timeout = 1.6
        self.backup_speed = -0.30
        self.backup_duration = 0.9
        self.turn_speed = 0.9
        self.turn_duration = 1.0
        self.recovery_start = 0.0
        self.recovery_turn = 1.0

        self.map_resolution = 0.15
        self.map_width = int(30.0 / self.map_resolution)
        self.map_height = int(40.0 / self.map_resolution)
        self.map_origin_x = -15.0
        self.map_origin_y = -0.5
        self.map_data = [-1] * (self.map_width * self.map_height)
        self.last_map_publish = time.monotonic() - 1.0

        self.last_progress_time = time.monotonic()
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0
        self.no_progress_time = 0.0
        self.last_status = 0.0
        self.start_time = time.monotonic()

        self.get_logger().info('Autonomous Tour node started (with obstacle avoidance + map).')
        self.get_logger().info(f'Waypoints configured: {len(self.waypoints)}')

    def _odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_received = True

    def _scan_callback(self, msg):
        self.scan = msg
        self.scan_received = True
        self.scan_time = time.monotonic()
        self._update_local_map(msg)

    def _control_loop(self):
        now = time.monotonic()

        if not self.odom_received:
            if now - self.last_status >= 2.0:
                self.get_logger().info('Waiting for /odom...')
                self.last_status = now
            return

        if now - self.last_map_publish >= 0.8:
            self._publish_local_map()
            self.last_map_publish = now

        if self.current_wp_index >= len(self.waypoints):
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info('Tour complete.')
            self.state = self.DONE
            rclpy.shutdown()
            return

        if self.scan is None and now - self.start_time > 2.5:
            if now - self.last_status >= 2.0:
                self.get_logger().warn(
                    'Lidar not ready on /bocbot/scan. '
                    'Following waypoints in fallback mode until scan arrives.'
                )
                self.last_status = now

        if self.state == self.WAITING:
            self._publish_cmd(0.0, 0.0)
            if now - self.wait_start >= self.wait_duration:
                self.current_wp_index += 1
                if self.current_wp_index < len(self.waypoints):
                    self.state = self.NAVIGATE
                    waypoint_name = self.waypoints[self.current_wp_index][0]
                    self.get_logger().info(f'Next waypoint: {waypoint_name}')
                    self.no_progress_time = 0.0
                    self.last_progress_time = now
                    self.last_progress_x = self.current_x
                    self.last_progress_y = self.current_y
            return

        if self.state in (self.BACKUP, self.TURN):
            self._execute_recovery(now)
            return

        name, tx, ty, should_wait = self.waypoints[self.current_wp_index]

        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - self.current_yaw)

        if distance <= self.waypoint_tolerance:
            self._publish_cmd(0.0, 0.0)
            if should_wait:
                self.get_logger().info(f'Arrived at {name} | hold {self.wait_duration:.1f}s')
                self.state = self.WAITING
                self.wait_start = now
                self.no_progress_time = 0.0
            else:
                self.current_wp_index += 1
                self.no_progress_time = 0.0
                self.last_progress_time = now
                self.last_progress_x = self.current_x
                self.last_progress_y = self.current_y
            return

        front_clear = self._scan_sector_min(-0.28, 0.28)
        left_clear = self._scan_sector_min(0.28, 1.22)
        right_clear = self._scan_sector_min(-1.22, -0.28)

        if front_clear < self.front_limit_stop:
            self._start_recovery(front_clear, left_clear, right_clear)
            return

        if front_clear == float('inf'):
            front_clear = 10.0
        if left_clear == float('inf'):
            left_clear = 10.0
        if right_clear == float('inf'):
            right_clear = 10.0

        on_bridge = "Bridge" in name
        speed = self._compute_speed(distance, on_bridge)
        angular = self._goal_turn(angle_error, on_bridge)
        angular += self._avoid_turn(left_clear, right_clear, front_clear)

        if front_clear <= self.front_limit_slow:
            speed *= 0.25
            angular *= 1.1
        elif front_clear < 1.5:
            clearance_scale = (front_clear - self.front_limit_stop) / (1.5 - self.front_limit_stop)
            speed *= clamp(clearance_scale, 0.35, 1.0)

        if distance <= 2.5:
            speed *= clamp(0.7 + 0.3 * (distance / 2.5), 0.25, 1.0)

        if abs(angle_error) > self.align_threshold and distance > 0.8:
            speed = 0.0

        angular = clamp(angular, -self.max_angular_speed, self.max_angular_speed)
        self._publish_cmd(speed, angular)
        self._check_progress(now, speed, angular, left_clear, right_clear)

        if now - self.last_status >= 1.0:
            self.get_logger().info(
                f'{name}: dist={distance:.2f} angle={math.degrees(angle_error):.1f}deg '
                f'cmd=({speed:.2f},{angular:.2f}) '
                f'front={front_clear:.2f} left={left_clear:.2f} right={right_clear:.2f}'
            )
            self.last_status = now

    def _compute_speed(self, distance, on_bridge):
        max_speed = self.bridge_speed if on_bridge else self.max_linear_speed
        return clamp(0.8 * distance, self.min_linear_speed, max_speed)

    def _goal_turn(self, angle_error, on_bridge):
        scale = self.align_scale if not on_bridge else 1.4
        return angle_error * scale * self.drive_scale

    def _avoid_turn(self, left_clear, right_clear, front_clear):
        if self.scan is None:
            return 0.0

        left_blocked = left_clear < self.side_limit_warn
        right_blocked = right_clear < self.side_limit_warn
        if not left_blocked and not right_blocked:
            return 0.0

        diff = right_clear - left_clear
        lateral = clamp(diff / max(self.side_limit_warn, 0.01), -1.0, 1.0)
        if min(left_clear, right_clear) < self.side_limit_critical:
            turn = self.avoid_gain * (1.0 if diff >= 0 else -1.0)
        else:
            turn = self.obstacle_turn_gain * (-lateral)

        if front_clear < 1.1 and not (left_blocked and right_blocked):
            turn *= 1.4

        return clamp(turn, -self.max_angular_speed, self.max_angular_speed)

    def _start_recovery(self, front_clear, left_clear, right_clear):
        self.state = self.BACKUP
        self.recovery_start = time.monotonic()
        self.no_progress_time = 0.0
        self.last_progress_time = self.recovery_start
        self.last_progress_x = self.current_x
        self.last_progress_y = self.current_y
        self.recovery_turn = -1.0 if left_clear > right_clear else 1.0
        self.get_logger().warn(
            f'Recovery triggered: front={front_clear:.2f}, left={left_clear:.2f}, right={right_clear:.2f}'
        )

    def _execute_recovery(self, now):
        elapsed = now - self.recovery_start
        if elapsed <= self.backup_duration:
            self._publish_cmd(self.backup_speed, 0.0)
            return
        if elapsed <= self.backup_duration + self.turn_duration:
            self._publish_cmd(0.0, self.turn_speed * self.recovery_turn)
            return
        self.get_logger().info('Recovery finished, resuming NAVIGATE.')
        self.state = self.NAVIGATE
        self.no_progress_time = 0.0
        self.last_progress_time = now
        self.last_progress_x = self.current_x
        self.last_progress_y = self.current_y

    def _check_progress(self, now, speed, angular, left_clear, right_clear):
        dt = now - self.last_progress_time
        if dt <= 0.0:
            return

        moved = math.hypot(self.current_x - self.last_progress_x, self.current_y - self.last_progress_y)
        moving_intent = abs(speed) > 0.08

        if moving_intent and moved < self.stuck_motion_threshold:
            self.no_progress_time += dt
            if self.no_progress_time > self.no_progress_timeout:
                self._start_recovery(front_clear=10.0, left_clear=left_clear, right_clear=right_clear)
                return
        else:
            self.no_progress_time = 0.0

        if dt >= self.stuck_progress_window:
            self.last_progress_time = now
            self.last_progress_x = self.current_x
            self.last_progress_y = self.current_y

    def _publish_cmd(self, linear, angular):
        msg = Twist()
        max_linear = max(self.max_linear_speed, self.bridge_speed)
        msg.linear.x = float(clamp(linear, -0.35, max_linear))
        msg.angular.z = float(clamp(angular, -self.max_angular_speed, self.max_angular_speed))
        self.cmd_pub.publish(msg)

    def _publish_local_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(self.map_data)
        self.map_pub.publish(msg)

    def _scan_sector_min(self, sector_start, sector_end):
        if not self.scan_received or self.scan is None:
            return float('inf')

        scan = self.scan
        if scan.angle_increment <= 0.0:
            return float('inf')

        start = max(sector_start, scan.angle_min)
        end = min(sector_end, scan.angle_max)
        if start >= end:
            return float('inf')

        i_start = int((start - scan.angle_min) / scan.angle_increment)
        i_end = int((end - scan.angle_min) / scan.angle_increment)
        i_start = max(0, min(i_start, len(scan.ranges) - 1))
        i_end = max(0, min(i_end, len(scan.ranges) - 1))
        if i_start > i_end:
            i_start, i_end = i_end, i_start

        best = float('inf')
        for i in range(i_start, i_end + 1):
            range_m = scan.ranges[i]
            if not math.isfinite(range_m):
                continue
            if range_m < scan.range_min or range_m > scan.range_max:
                continue
            best = min(best, range_m)
        return best

    def _update_local_map(self, scan):
        if not self.odom_received:
            return

        for i, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m):
                continue
            if range_m < scan.range_min or range_m > min(scan.range_max, 30.0):
                continue

            a = scan.angle_min + i * scan.angle_increment
            local_x = range_m * math.cos(a)
            local_y = range_m * math.sin(a)
            world_x = self.current_x + local_x * math.cos(self.current_yaw) - local_y * math.sin(self.current_yaw)
            world_y = self.current_y + local_x * math.sin(self.current_yaw) + local_y * math.cos(self.current_yaw)

            mx = int((world_x - self.map_origin_x) / self.map_resolution)
            my = int((world_y - self.map_origin_y) / self.map_resolution)
            if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                self._set_map_cell(mx, my, 100)
                for y in (my - 1, my, my + 1):
                    for x in (mx - 1, mx, mx + 1):
                        self._set_map_cell(x, y, 100)

        robot_mx = int((self.current_x - self.map_origin_x) / self.map_resolution)
        robot_my = int((self.current_y - self.map_origin_y) / self.map_resolution)
        if 0 <= robot_mx < self.map_width and 0 <= robot_my < self.map_height:
            self._set_map_cell(robot_mx, robot_my, 0)
            for y in (robot_my - 1, robot_my, robot_my + 1):
                for x in (robot_mx - 1, robot_mx, robot_mx + 1):
                    self._set_map_cell(x, y, 0)

    def _set_map_cell(self, mx, my, value):
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            idx = my * self.map_width + mx
            self.map_data[idx] = value


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousTourNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt (SIGINT)')
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

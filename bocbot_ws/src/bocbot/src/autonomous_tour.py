#!/usr/bin/env python3
"""
autonomous_tour.py — PID-controlled autonomous 10-room office tour for bocbot.

Architecture (cascaded control):
  Outer loop : waypoint sequencer  (position -> desired heading & speed)
  Inner loop : dual PID controllers (linear on distance, angular on heading error)
  Output stage: trapezoidal velocity profiler (jerk-free acceleration/deceleration)

Bridge handling:
  - Gain scheduling: higher integral + Kp on ascent for sustained torque
  - Heavy damping + low speed cap on descent for active braking
  - Tight heading tolerance (<4 deg) to keep robot centred on 3m-wide bridge

State machine:  ALIGN -> DRIVE -> ARRIVE -> WAIT -> next waypoint -> ...
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# =============================================================================
#  PID CONTROLLER
# =============================================================================

class PID:
    """
    Discrete PID with anti-windup integral clamping and low-pass filtered
    derivative.  Callable: pid(error, dt) -> command.
    """

    def __init__(self, kp, ki, kd, lo, hi, imax=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.lo, self.hi = lo, hi
        self.imax = imax if imax is not None else abs(hi) / max(ki, 1e-9)
        self._i = 0.0
        self._ep = None
        self._df = 0.0

    def reset(self):
        self._i = 0.0
        self._ep = None
        self._df = 0.0

    def __call__(self, err, dt):
        if dt < 1e-9:
            return 0.0
        p = self.kp * err
        self._i = max(-self.imax, min(self.imax, self._i + err * dt))
        i = self.ki * self._i
        if self._ep is not None:
            self._df = 0.3 * ((err - self._ep) / dt) + 0.7 * self._df
        self._ep = err
        d = self.kd * self._df
        return max(self.lo, min(self.hi, p + i + d))

    def set_gains(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd


# =============================================================================
#  VELOCITY PROFILER
# =============================================================================

class Profiler:
    """Trapezoidal rate-limiter for smooth, jerk-limited velocity transitions."""

    def __init__(self, lin_acc=0.8, lin_dec=1.2, ang_acc=2.5):
        self.la, self.ld, self.aa = lin_acc, lin_dec, ang_acc
        self.v = 0.0
        self.w = 0.0

    def __call__(self, tv, tw, dt):
        if dt < 1e-9:
            return self.v, self.w
        dv = tv - self.v
        lim = (self.la if dv > 0 else self.ld) * dt
        self.v += max(-lim, min(lim, dv))
        dw = tw - self.w
        alim = self.aa * dt
        self.w += max(-alim, min(alim, dw))
        return self.v, self.w

    def stop(self, dt):
        return self(0.0, 0.0, dt)

    def reset(self):
        self.v = self.w = 0.0


# =============================================================================
#  UTILITIES
# =============================================================================

def wrap(a):
    """Normalise angle to (-pi, pi] — always picks shortest rotation."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_q(q):
    """Extract yaw from geometry_msgs Quaternion (ZYX Euler)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def smoothstep(x):
    """Hermite smoothstep [0,1] with zero derivative at endpoints."""
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


# =============================================================================
#  WAYPOINT TAGS
# =============================================================================

ROOM    = "room"       # pause and announce arrival
PASS    = "pass"       # transit waypoint, no pause
BR_APP  = "br_app"     # bridge approach (slow down, align)
BR_UP   = "br_up"      # bridge ascending stairs
BR_FLAT = "br_flat"    # bridge platform
BR_DN   = "br_down"    # bridge descending stairs
BR_EXIT = "br_exit"    # bridge exit

BRIDGE_TAGS = {BR_APP, BR_UP, BR_FLAT, BR_DN, BR_EXIT}


# =============================================================================
#  TOUR WAYPOINTS
# =============================================================================

def build_tour():
    """
    Complete 10-room tour through the 30x40m office complex.

    Layout:
      Corridor    x in [-2.5, 2.5], y = 0..40
      West wing   x in [-15, -2.5]   rooms 1-5 (south to north)
      East wing   x in [2.5, 15]     rooms 6-10 (south to north)
      Bridge      3m wide at x=0, stairs y=12..16 & y=24..28,
                  platform y=16..24, peak z=0.80m (10 steps x 0.08m)

    Corridor-wall doorways (4m gaps): y ~ 4, 12, 20, 28, 36
    Cross-wall doorways (4m gaps):
      West x in [-10.75, -6.75]  |  East x in [6.75, 10.75]
      at y = 8, 16, 24, 32

    Route: west rooms 1-5 via internal doors -> corridor -> bridge south ->
           east rooms 6-10 via internal doors -> corridor -> bridge south ->
           entrance.
    """
    return [
        # -- Start --
        ("Start",                   0.0,   1.0,  PASS),

        # -- West wing (enter at Room 1, traverse through internal doorways) --
        ("Corr to R1 door",       -1.0,   4.0,  PASS),
        ("R1 entry",              -4.0,   4.0,  PASS),
        ("Room 1 - Reception",    -8.0,   5.0,  ROOM),

        ("R1-R2 doorway",         -8.0,   8.0,  PASS),
        ("Room 2 - Office 1",    -7.0,  11.0,  ROOM),

        ("R2-R3 doorway",         -8.0,  16.0,  PASS),
        ("Room 3 - Conference",   -7.0,  19.0,  ROOM),

        ("R3-R4 doorway",         -8.0,  24.0,  PASS),
        ("Room 4 - Office 2",    -9.0,  27.0,  ROOM),

        ("R4-R5 doorway",         -8.0,  32.0,  PASS),
        ("Room 5 - Lounge",      -8.0,  36.0,  ROOM),

        # -- Exit west wing to corridor (north end) --
        ("R5 to corridor",        -4.0,  36.0,  PASS),
        ("Corridor north",         0.0,  36.0,  PASS),

        # -- Bridge crossing #1 (south-bound) --
        # Robot climbs "descending" stairs (north side y=27.8..24.2, rising to 0.80m),
        # crosses platform (y=24..16, flat at 0.80m),
        # descends "ascending" stairs (south side y=15.8..12.2, falling to ground).
        ("Pre-bridge N",           0.0,  29.0,  BR_APP),
        ("Bridge climb mid",       0.0,  27.0,  BR_UP),
        ("Bridge climb top",       0.0,  24.5,  BR_UP),
        ("Platform N",             0.0,  23.0,  BR_FLAT),
        ("Platform center",        0.0,  20.0,  BR_FLAT),
        ("Platform S",             0.0,  17.0,  BR_FLAT),
        ("Bridge descend top",     0.0,  15.5,  BR_DN),
        ("Bridge descend mid",     0.0,  13.0,  BR_DN),
        ("Post-bridge S",          0.0,  11.0,  BR_EXIT),

        # -- East wing (enter at Room 6, traverse north) --
        ("Corr to R6 door",        1.0,   4.0,  PASS),
        ("R6 entry",               4.0,   4.0,  PASS),
        ("Room 6 - Workspace",     8.0,   5.0,  ROOM),

        ("R6-R7 doorway",          8.0,   8.0,  PASS),
        ("Room 7 - Office 3",     8.0,  11.0,  ROOM),

        ("R7-R8 doorway",          8.0,  16.0,  PASS),
        ("Room 8 - Server Room",   6.0,  19.0,  ROOM),

        ("R8-R9 doorway",          8.0,  24.0,  PASS),
        ("Room 9 - Office 4",     9.0,  27.0,  ROOM),

        ("R9-R10 doorway",         8.0,  32.0,  PASS),
        ("Room 10 - Lab",          8.0,  36.0,  ROOM),

        # -- Exit east wing, bridge crossing #2 --
        ("R10 to corridor",        4.0,  36.0,  PASS),
        ("Corridor north 2",       0.0,  36.0,  PASS),

        ("Pre-bridge N2",          0.0,  29.0,  BR_APP),
        ("Bridge climb mid 2",    0.0,  27.0,  BR_UP),
        ("Bridge climb top 2",    0.0,  24.5,  BR_UP),
        ("Platform N2",            0.0,  23.0,  BR_FLAT),
        ("Platform center 2",     0.0,  20.0,  BR_FLAT),
        ("Platform S2",            0.0,  17.0,  BR_FLAT),
        ("Bridge descend top 2",  0.0,  15.5,  BR_DN),
        ("Bridge descend mid 2",  0.0,  13.0,  BR_DN),
        ("Post-bridge S2",         0.0,  11.0,  BR_EXIT),

        # -- Return to entrance --
        ("Corridor south",         0.0,   5.0,  PASS),
        ("Entrance - Finish",      0.0,   1.0,  ROOM),
    ]


# =============================================================================
#  NAVIGATION NODE
# =============================================================================

class TourNode(Node):
    """
    State machine:
      ALIGN  - rotate in place until heading is within tolerance of target bearing
      DRIVE  - translate toward target with dual-PID (linear + angular) control
      ARRIVE - ramp velocity to zero (stabilisation hold)
      WAIT   - hold position at ROOM waypoints for ROOM_PAUSE seconds
      DONE   - tour complete, robot stopped
    """

    # -- Control rate --
    HZ = 20

    # -- Waypoint tolerances (m) --
    TOL       = 0.30
    TOL_BR    = 0.45     # relaxed on bridge (avoid overshoot on stairs)
    TOL_FINAL = 0.20

    # -- Heading thresholds (rad) --
    ALIGN_OK    = 0.12   # ~7 deg — exit ALIGN
    ALIGN_OK_BR = 0.06   # ~3.5 deg — tight for bridge centering
    REALIGN     = 0.6    # ~35 deg drift triggers re-align from DRIVE

    # -- Speed limits (m/s) --
    V_MAX      = 1.0
    V_MIN      = 0.08
    V_BR_UP    = 0.35    # sustained climb speed
    V_BR_FLAT  = 0.45    # platform speed
    V_BR_DN    = 0.20    # slow controlled descent
    V_BR_SLOW  = 0.50    # approach / exit
    W_MAX      = 1.2     # rad/s

    # -- Deceleration --
    DECEL_R = 2.0        # start smoothstep deceleration at this distance

    # -- PID gains: normal navigation --
    N_LP = 0.8;  N_LI = 0.02;  N_LD = 0.15
    N_AP = 3.0;  N_AI = 0.05;  N_AD = 0.4

    # -- PID gains: bridge ascend (more push, tighter heading) --
    U_LP = 1.2;  U_LI = 0.08;  U_LD = 0.10
    U_AP = 4.5;  U_AI = 0.10;  U_AD = 0.5

    # -- PID gains: bridge descend / platform (gentle, heavy damping) --
    D_LP = 0.5;  D_LI = 0.01;  D_LD = 0.30
    D_AP = 4.5;  D_AI = 0.10;  D_AD = 0.5

    # -- Timing --
    ROOM_PAUSE = 3.0     # seconds to hold at each room
    STAB_TIME  = 0.4     # stabilisation hold in ARRIVE before advancing
    SCAN_TIMEOUT = 0.75   # seconds
    MAP_PERIOD = 0.8      # seconds between local map publishes

    # -- Stuck detection --
    STUCK_TIME = 6.0     # seconds without meaningful movement
    STUCK_DIST = 0.12    # minimum displacement to reset stuck timer
    STUCK_SPEED_MIN = 0.18

    # -- Obstacle handling --
    FRONT_STOP = 0.55
    FRONT_SLOW = 1.05
    SIDE_WARN = 0.85
    SIDE_CRITICAL = 0.55
    AVOID_GAIN = 2.4
    OBSTACLE_TURN_GAIN = 1.1
    RECOVERY_BACKUP = -0.30
    RECOVERY_BACKUP_TIME = 0.9
    RECOVERY_TURN_TIME = 1.0
    RECOVERY_TURN_SPEED = 0.9

    def __init__(self):
        super().__init__('autonomous_tour')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/bocbot/local_map', 1)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/bocbot/scan', self._scan_cb_bocbot, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb_scan, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/bocbot/gazebo_ros_head_hokuyo_controller/out',
            self._scan_cb_plugin_namespaced, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/gazebo_ros_head_hokuyo_controller/out',
            self._scan_cb_plugin_global, qos_profile_sensor_data)

        # Odometry state
        self.x = self.y = self.yaw = 0.0
        self.odom_ok = False
        self._odom_t = 0.0
        self.scan = None
        self.scan_received = False
        self.scan_time = None
        self.scan_topic = None
        self._last_scan_topic_log = 0.0

        # Local occupancy map
        self.map_resolution = 0.15
        self.map_width = int(30.0 / self.map_resolution)
        self.map_height = int(40.0 / self.map_resolution)
        self.map_origin_x = -15.0
        self.map_origin_y = -0.5
        self.map_data = [-1] * (self.map_width * self.map_height)
        self.last_map_publish = time.monotonic() - self.MAP_PERIOD

        # Movement watchdog
        self.last_progress_time = time.monotonic()
        self.last_progress_x = self.x
        self.last_progress_y = self.y
        self.no_progress_time = 0.0
        self._last_status = 0.0
        self._last_scan_status = 0.0
        self._recovery_start = 0.0
        self._recovery_turn = 1.0

        # Waypoint list
        self.wps = build_tour()
        self.wi = 0

        # Controllers
        self.lpid = PID(self.N_LP, self.N_LI, self.N_LD, -0.15, self.V_MAX)
        self.apid = PID(self.N_AP, self.N_AI, self.N_AD, -self.W_MAX, self.W_MAX)
        self.prof = Profiler(lin_acc=0.8, lin_dec=1.2, ang_acc=2.5)

        # State
        self.state = 'ALIGN'
        self.st_t = time.monotonic()
        self.last_t = time.monotonic()
        self.last_log_t = 0.0
        self.tour_start = 0.0   # set when first odom arrives

        # Bridge section tracking for log transitions
        self._in_bridge = False

        self.create_timer(1.0 / self.HZ, self._tick)

        self.get_logger().info('=== Autonomous Tour Node Started ===')
        self.get_logger().info(
            f'{len(self.wps)} waypoints  |  {self.HZ} Hz control  |  '
            f'{sum(1 for w in self.wps if w[3] == ROOM)} rooms')
        self._log_wp()

    # -- Odom callback --

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_q(msg.pose.pose.orientation)
        self._odom_t = time.monotonic()
        if not self.odom_ok:
            self.odom_ok = True
            self.tour_start = time.monotonic()
            self.last_progress_time = self._odom_t
            self.last_progress_x = self.x
            self.last_progress_y = self.y
            self.get_logger().info(
                f'Odom locked: ({self.x:.2f}, {self.y:.2f}) '
                f'yaw={math.degrees(self.yaw):.1f} deg')

    # -- Main control loop --

    def _scan_cb(self, msg, topic):
        if self.scan_topic != topic and time.monotonic() - self._last_scan_topic_log >= 1.0:
            if self.scan_topic is None:
                self.get_logger().info(f'Using lidar source: {topic}')
            else:
                self.get_logger().info(
                    f'Switching lidar source: {self.scan_topic} -> {topic}')
            self.scan_topic = topic
            self._last_scan_topic_log = time.monotonic()

        self.scan = msg
        self.scan_received = True
        self.scan_time = time.monotonic()
        self._update_local_map(msg)

    def _scan_cb_scan(self, msg):
        self._scan_cb(msg, '/scan')

    def _scan_cb_bocbot(self, msg):
        self._scan_cb(msg, '/bocbot/scan')

    def _scan_cb_plugin_namespaced(self, msg):
        self._scan_cb(msg, '/bocbot/gazebo_ros_head_hokuyo_controller/out')

    def _scan_cb_plugin_global(self, msg):
        self._scan_cb(msg, '/gazebo_ros_head_hokuyo_controller/out')

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_t
        self.last_t = now
        scan_fresh = self._scan_fresh(now)

        if not self.odom_ok:
            self.get_logger().info(
                'Waiting for /odom ...', throttle_duration_sec=2.0)
            return

        if not scan_fresh and now - self._last_scan_status >= 2.0:
            self.get_logger().warn(
                'Lidar stale / unavailable: holding linear speed at 0 m/s while waiting for a live scan topic.')
            self._last_scan_status = now

        # Odom watchdog
        if now - self._odom_t > 1.0:
            self.get_logger().warn(
                'Odom stale (>1s) — holding', throttle_duration_sec=2.0)
            self._stop(dt)
            return

        if now - self.last_map_publish >= self.MAP_PERIOD:
            self._publish_local_map()
            self.last_map_publish = now

        # Tour finished?
        if self.wi >= len(self.wps):
            self._stop(dt)
            return

        name, tx, ty, tag = self.wps[self.wi]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        bear = math.atan2(dy, dx)
        herr = wrap(bear - self.yaw)

        # Per-tag parameters
        tol = self._tol(tag)
        atol = self.ALIGN_OK_BR if tag in BRIDGE_TAGS else self.ALIGN_OK
        vmax = self._vmax(tag)
        self._apply_gains(tag)
        if scan_fresh:
            front_clear = self._scan_sector_min(-0.28, 0.28)
            left_clear = self._scan_sector_min(0.28, 1.22)
            right_clear = self._scan_sector_min(-1.22, -0.28)
        else:
            front_clear = float('inf')
            left_clear = float('inf')
            right_clear = float('inf')

        # Bridge transition logging
        on_bridge = tag in BRIDGE_TAGS
        if on_bridge and not self._in_bridge:
            self._in_bridge = True
            self.get_logger().info('--- ENTERING BRIDGE SECTION ---')
        elif not on_bridge and self._in_bridge:
            self._in_bridge = False
            self.get_logger().info('--- EXITED BRIDGE SECTION ---')

        if self.state == 'BACKUP':
            self._execute_recovery(now)
            return

        if self.state in ('ALIGN', 'DRIVE') and scan_fresh and front_clear < self.FRONT_STOP:
            if now - self._last_status >= 2.0:
                self.get_logger().warn(
                    'Obstacle too close ahead; attempting recovery')
                self._last_status = now
            self._start_recovery(front_clear, left_clear, right_clear)
            return

        # ---- State machine ----

        if self.state == 'ALIGN':
            if dist < tol:
                self._go('ARRIVE')
            elif abs(herr) < atol:
                self.get_logger().info(
                    f'  Aligned ({math.degrees(herr):.1f} deg) -> DRIVE')
                self._go('DRIVE')
            else:
                w_cmd = self.apid(herr, dt)
                w_cmd += self._avoid_turn(left_clear, right_clear, front_clear)
                _, wo = self.prof(0.0, w_cmd, dt)
                self._cmd(0.0, wo)

        elif self.state == 'DRIVE':
            if dist < tol:
                self._go('ARRIVE')
                return

            if not scan_fresh:
                rw = self.apid(herr, dt)
                vo, wo = self.prof(0.0, rw, dt)
                self._cmd(0.0, wo)
                if now - self.last_log_t > 2.0:
                    self.last_log_t = now
                    elapsed = now - self.tour_start
                    self.get_logger().info(
                        f'  [{self.wi+1}/{len(self.wps)}] {name} (no lidar): '
                        f'd={dist:.2f}m h={math.degrees(herr):.1f}deg '
                        f'v={vo:.2f} w={wo:.2f}  T+{elapsed:.0f}s')
                self._check_progress(now, vo, left_clear, right_clear)
                return

            # Re-align on large heading drift (skip on bridge stairs)
            if abs(herr) > self.REALIGN and tag not in {BR_UP, BR_DN, BR_FLAT}:
                self.get_logger().info(
                    f'  Heading drift {math.degrees(herr):.0f} deg -> re-align')
                self._go('ALIGN')
                return

            # Linear PID on distance
            rv = self.lpid(dist, dt)
            # Smoothstep deceleration near target
            if dist < self.DECEL_R:
                rv *= smoothstep(dist / self.DECEL_R)
            rv = max(self.V_MIN, min(vmax, rv))
            if front_clear <= self.FRONT_SLOW:
                rv *= 0.25
            elif front_clear < 1.5:
                rv *= clamp((front_clear - self.FRONT_STOP) / (1.5 - self.FRONT_STOP), 0.35, 1.0)

            # Angular PID on heading error
            rw = self.apid(herr, dt)
            rw += self._avoid_turn(left_clear, right_clear, front_clear)

            if dist <= 2.5:
                rv *= clamp(0.7 + 0.3 * (dist / 2.5), 0.25, 1.0)

            # Profiled output
            vo, wo = self.prof(rv, rw, dt)
            self._cmd(vo, wo)
            self._check_progress(now, vo, left_clear, right_clear)

            # Periodic status log
            if now - self.last_log_t > 2.0:
                self.last_log_t = now
                elapsed = now - self.tour_start
                self.get_logger().info(
                    f'  [{self.wi+1}/{len(self.wps)}] {name}: '
                    f'd={dist:.2f}m h={math.degrees(herr):.1f}deg '
                    f'v={vo:.2f} w={wo:.2f} front={front_clear:.2f} '
                    f'left={left_clear:.2f} right={right_clear:.2f}  '
                    f'T+{elapsed:.0f}s')

        elif self.state == 'ARRIVE':
            vo, wo = self.prof.stop(dt)
            self._cmd(vo, wo)
            if now - self.st_t >= self.STAB_TIME:
                if tag == ROOM:
                    elapsed = now - self.tour_start
                    self.get_logger().info(
                        f'>> ARRIVED: {name}  '
                        f'({self.x:.2f},{self.y:.2f})  '
                        f'pausing {self.ROOM_PAUSE:.0f}s  T+{elapsed:.0f}s')
                    self._go('WAIT')
                else:
                    self._next()

        elif self.state == 'WAIT':
            self._stop(dt)
            if now - self.st_t >= self.ROOM_PAUSE:
                self._next()

        elif self.state == 'DONE':
            self._stop(dt)

    # -- State helpers --

    def _go(self, s):
        self.state = s
        self.st_t = time.monotonic()
        if s == 'ARRIVE':
            self.lpid.reset()
            self.apid.reset()

    def _next(self):
        self.wi += 1
        self.lpid.reset()
        self.apid.reset()
        self.prof.reset()
        self.no_progress_time = 0.0
        self.last_progress_time = time.monotonic()
        self.last_progress_x = self.x
        self.last_progress_y = self.y
        if self.wi >= len(self.wps):
            elapsed = time.monotonic() - self.tour_start
            self.get_logger().info('=' * 50)
            self.get_logger().info(
                f'  TOUR COMPLETE  |  {elapsed:.0f}s elapsed')
            self.get_logger().info('=' * 50)
            self._go('DONE')
        else:
            self._go('ALIGN')
            self._log_wp()

    def _log_wp(self):
        if self.wi < len(self.wps):
            n, x, y, t = self.wps[self.wi]
            self.get_logger().info(
                f'[{self.wi+1}/{len(self.wps)}] Next: {n}  '
                f'({x:.1f},{y:.1f})  [{t}]')

    # -- Tag-based parameter helpers --

    def _tol(self, tag):
        if tag in BRIDGE_TAGS:
            return self.TOL_BR
        if self.wi == len(self.wps) - 1:
            return self.TOL_FINAL
        return self.TOL

    def _vmax(self, tag):
        return {
            BR_UP:   self.V_BR_UP,
            BR_FLAT: self.V_BR_FLAT,
            BR_DN:   self.V_BR_DN,
            BR_APP:  self.V_BR_SLOW,
            BR_EXIT: self.V_BR_SLOW,
        }.get(tag, self.V_MAX)

    def _apply_gains(self, tag):
        if tag in (BR_UP, BR_APP):
            self.lpid.set_gains(self.U_LP, self.U_LI, self.U_LD)
            self.apid.set_gains(self.U_AP, self.U_AI, self.U_AD)
        elif tag in (BR_DN, BR_FLAT, BR_EXIT):
            self.lpid.set_gains(self.D_LP, self.D_LI, self.D_LD)
            self.apid.set_gains(self.D_AP, self.D_AI, self.D_AD)
        else:
            self.lpid.set_gains(self.N_LP, self.N_LI, self.N_LD)
            self.apid.set_gains(self.N_AP, self.N_AI, self.N_AD)

    # -- Publishing --

    def _cmd(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def _stop(self, dt):
        v, w = self.prof.stop(dt)
        if abs(v) < 0.005 and abs(w) < 0.005:
            v = w = 0.0
        self._cmd(v, w)

    # -- Perception and safety helpers --

    def _scan_fresh(self, now):
        return (
            self.scan_received
            and self.scan is not None
            and self.scan_time is not None
            and now - self.scan_time <= self.SCAN_TIMEOUT
        )

    def _scan_sector_min(self, sector_start, sector_end):
        if self.scan is None:
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

    def _avoid_turn(self, left_clear, right_clear, front_clear):
        if self.scan is None or not self._scan_fresh(time.monotonic()):
            return 0.0

        left_blocked = left_clear < self.SIDE_WARN
        right_blocked = right_clear < self.SIDE_WARN
        if not left_blocked and not right_blocked:
            return 0.0

        diff = right_clear - left_clear
        lateral = clamp(diff / max(self.SIDE_WARN, 0.01), -1.0, 1.0)
        if min(left_clear, right_clear) < self.SIDE_CRITICAL:
            turn = self.AVOID_GAIN * (1.0 if diff >= 0 else -1.0)
        else:
            turn = self.OBSTACLE_TURN_GAIN * (-lateral)

        if front_clear < 1.1 and not (left_blocked and right_blocked):
            turn *= 1.4

        return clamp(turn, -self.W_MAX, self.W_MAX)

    def _start_recovery(self, front_clear, left_clear, right_clear):
        self.state = 'BACKUP'
        self._recovery_start = time.monotonic()
        self.no_progress_time = 0.0
        self.last_progress_time = self._recovery_start
        self.last_progress_x = self.x
        self.last_progress_y = self.y
        self._recovery_turn = 1.0 if left_clear > right_clear else -1.0
        self.prof.reset()
        self.get_logger().warn(
            f'Recovery triggered: front={front_clear:.2f}, left={left_clear:.2f}, right={right_clear:.2f}'
        )

    def _execute_recovery(self, now):
        elapsed = now - self._recovery_start
        if elapsed <= self.RECOVERY_BACKUP_TIME:
            self._cmd(self.RECOVERY_BACKUP, 0.0)
            return
        if elapsed <= self.RECOVERY_BACKUP_TIME + self.RECOVERY_TURN_TIME:
            self._cmd(0.0, self.RECOVERY_TURN_SPEED * self._recovery_turn)
            return

        self.get_logger().info('Recovery finished, resuming alignment.')
        self._go('ALIGN')
        self.no_progress_time = 0.0
        self.last_progress_time = now
        self.last_progress_x = self.x
        self.last_progress_y = self.y

    def _check_progress(self, now, speed, left_clear, right_clear):
        dt = now - self.last_progress_time
        if dt <= 0.0:
            return

        moved = math.hypot(self.x - self.last_progress_x, self.y - self.last_progress_y)
        moving_intent = abs(speed) > self.STUCK_SPEED_MIN

        if moved >= self.STUCK_DIST:
            self.no_progress_time = 0.0
            self.last_progress_x = self.x
            self.last_progress_y = self.y

        elif moving_intent:
            self.no_progress_time += dt
            if self.no_progress_time > self.STUCK_TIME:
                self._start_recovery(front_clear=10.0, left_clear=left_clear, right_clear=right_clear)
                return
        else:
            self.no_progress_time = 0.0
            self.last_progress_x = self.x
            self.last_progress_y = self.y

        self.last_progress_time = now

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

    def _update_local_map(self, scan):
        if not self.odom_ok:
            return

        for i, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m):
                continue
            if range_m < scan.range_min or range_m > min(scan.range_max, 30.0):
                continue

            a = scan.angle_min + i * scan.angle_increment
            local_x = range_m * math.cos(a)
            local_y = range_m * math.sin(a)
            world_x = (self.x + local_x * math.cos(self.yaw) - local_y * math.sin(self.yaw))
            world_y = (self.y + local_x * math.sin(self.yaw) + local_y * math.cos(self.yaw))

            mx = int((world_x - self.map_origin_x) / self.map_resolution)
            my = int((world_y - self.map_origin_y) / self.map_resolution)
            if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                self._set_map_cell(mx, my, 100)
                for y in (my - 1, my, my + 1):
                    for x in (mx - 1, mx, mx + 1):
                        self._set_map_cell(x, y, 100)

        robot_mx = int((self.x - self.map_origin_x) / self.map_resolution)
        robot_my = int((self.y - self.map_origin_y) / self.map_resolution)
        if 0 <= robot_mx < self.map_width and 0 <= robot_my < self.map_height:
            self._set_map_cell(robot_mx, robot_my, 0)
            for y in (robot_my - 1, robot_my, robot_my + 1):
                for x in (robot_mx - 1, robot_mx, robot_mx + 1):
                    self._set_map_cell(x, y, 0)

    def _set_map_cell(self, mx, my, value):
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            idx = my * self.map_width + mx
            self.map_data[idx] = value


# =============================================================================
#  ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TourNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Tour interrupted.')
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

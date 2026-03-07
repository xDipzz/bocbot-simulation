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
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


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

    # -- Stuck detection --
    STUCK_TIME = 8.0     # seconds without meaningful movement
    STUCK_DIST = 0.15    # minimum displacement to reset stuck timer

    def __init__(self):
        super().__init__('autonomous_tour')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos_profile=qos_profile_sensor_data)

        # Odometry state
        self.x = self.y = self.yaw = 0.0
        self.odom_ok = False
        self._odom_t = 0.0

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

        # Stuck detector
        self._sk_x = self._sk_y = 0.0
        self._sk_t = time.monotonic()

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
            self.get_logger().info(
                f'Odom locked: ({self.x:.2f}, {self.y:.2f}) '
                f'yaw={math.degrees(self.yaw):.1f} deg')

    # -- Main control loop --

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_t
        self.last_t = now

        if not self.odom_ok:
            self.get_logger().info(
                'Waiting for /odom ...', throttle_duration_sec=2.0)
            return

        # Odom watchdog
        if now - self._odom_t > 1.0:
            self.get_logger().warn(
                'Odom stale (>1s) — holding', throttle_duration_sec=2.0)
            self._stop(dt)
            return

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

        # Bridge transition logging
        on_bridge = tag in BRIDGE_TAGS
        if on_bridge and not self._in_bridge:
            self._in_bridge = True
            self.get_logger().info('--- ENTERING BRIDGE SECTION ---')
        elif not on_bridge and self._in_bridge:
            self._in_bridge = False
            self.get_logger().info('--- EXITED BRIDGE SECTION ---')

        # Stuck detection (ALIGN / DRIVE only)
        if self.state in ('ALIGN', 'DRIVE'):
            moved = math.hypot(self.x - self._sk_x, self.y - self._sk_y)
            if moved > self.STUCK_DIST:
                self._sk_x, self._sk_y = self.x, self.y
                self._sk_t = now
            elif now - self._sk_t > self.STUCK_TIME:
                self.get_logger().warn(
                    f'STUCK {self.STUCK_TIME:.0f}s at ({self.x:.1f},{self.y:.1f}) '
                    f'target={name} d={dist:.2f}m')
                self._sk_t = now

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
                _, wo = self.prof(0.0, w_cmd, dt)
                self._cmd(0.0, wo)

        elif self.state == 'DRIVE':
            if dist < tol:
                self._go('ARRIVE')
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

            # Angular PID on heading error
            rw = self.apid(herr, dt)

            # Profiled output
            vo, wo = self.prof(rv, rw, dt)
            self._cmd(vo, wo)

            # Periodic status log
            if now - self.last_log_t > 2.0:
                self.last_log_t = now
                elapsed = now - self.tour_start
                self.get_logger().info(
                    f'  [{self.wi+1}/{len(self.wps)}] {name}: '
                    f'd={dist:.2f}m h={math.degrees(herr):.1f}deg '
                    f'v={vo:.2f} w={wo:.2f}  T+{elapsed:.0f}s')

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
        self._sk_x, self._sk_y = self.x, self.y
        self._sk_t = time.monotonic()
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

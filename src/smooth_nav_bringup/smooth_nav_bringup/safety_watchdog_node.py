#!/usr/bin/env python3
# Copyright 2026 smooth_nav Authors
# SPDX-License-Identifier: Apache-2.0

"""
safety_watchdog_node.py — Real-world safety layer for smooth_nav.

Monitors the robot's state and enforces safety constraints:

1. **cmd_vel watchdog**: If no velocity command is received within timeout,
   publishes zero velocity (emergency stop). Prevents runaway robots when
   nodes crash.

2. **Velocity limiting**: Enforces hardware limits regardless of what the
   controller outputs. Essential when tuning PID gains.

3. **Acceleration limiting**: Smooths out sudden velocity jumps that could
   damage hardware or slip wheels.

4. **Obstacle proximity stop** (optional): Subscribes to /scan (LaserScan)
   and stops the robot if any reading is below a safety distance.

Subscribes:
    /cmd_vel_raw (geometry_msgs/Twist)  — raw controller output
    /scan (sensor_msgs/LaserScan)       — optional laser scan

Publishes:
    /cmd_vel (geometry_msgs/Twist)      — safe, rate-limited velocity
    /safety_status (std_msgs/String)    — current safety state

Parameters:
    cmd_timeout: float (default 0.5) — seconds before emergency stop
    max_linear_vel: float (default 0.22) — m/s hardware limit
    max_angular_vel: float (default 2.84) — rad/s hardware limit
    max_linear_accel: float (default 1.0) — m/s² acceleration limit
    max_angular_accel: float (default 5.0) — rad/s² angular acceleration limit
    obstacle_stop_distance: float (default 0.15) — meters (0 to disable)
    safety_rate: float (default 20.0) — Hz watchdog check rate
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# LaserScan is optional (might not be available in sim-only setups)
try:
    from sensor_msgs.msg import LaserScan
    HAS_LASER = True
except ImportError:
    HAS_LASER = False


class SafetyWatchdogNode(Node):
    """Safety layer that sits between the controller and the motor driver."""

    def __init__(self):
        super().__init__('safety_watchdog_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 2.84)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 5.0)
        self.declare_parameter('obstacle_stop_distance', 0.15)
        self.declare_parameter('safety_rate', 20.0)

        # ── State ───────────────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()
        self._last_cmd = Twist()
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._obstacle_too_close = False
        self._safety_state = 'NOMINAL'

        # ── Subscribers ─────────────────────────────────────────────
        # Controller publishes to /cmd_vel_raw; we publish safe /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self._cmd_callback, 10)

        if HAS_LASER:
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self._scan_callback, 10)

        # ── Publishers ──────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/safety_status', 10)

        # ── Watchdog timer ──────────────────────────────────────────
        rate = self.get_parameter('safety_rate').value
        self._timer = self.create_timer(1.0 / rate, self._watchdog_tick)

        self.get_logger().info(
            f'SafetyWatchdog active: '
            f'v_max={self.get_parameter("max_linear_vel").value} m/s, '
            f'ω_max={self.get_parameter("max_angular_vel").value} rad/s, '
            f'timeout={self.get_parameter("cmd_timeout").value} s')

    def _cmd_callback(self, msg: Twist):
        """Receive raw velocity command from controller."""
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _scan_callback(self, msg):
        """Check laser scan for nearby obstacles."""
        min_dist = self.get_parameter('obstacle_stop_distance').value
        if min_dist <= 0.0:
            return

        closest = float('inf')
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                closest = min(closest, r)

        self._obstacle_too_close = closest < min_dist
        if self._obstacle_too_close:
            self.get_logger().warn(
                f'Obstacle at {closest:.2f} m (< {min_dist} m) — STOPPING',
                throttle_duration_sec=1.0)

    def _watchdog_tick(self):
        """
        Main safety loop — runs at safety_rate Hz.

        Checks timeout, applies limits, publishes safe cmd_vel.
        """
        now = self.get_clock().now()
        timeout = self.get_parameter('cmd_timeout').value
        dt = 1.0 / self.get_parameter('safety_rate').value

        elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9
        timed_out = elapsed > timeout

        # Determine desired velocity
        if timed_out or self._obstacle_too_close:
            desired_linear = 0.0
            desired_angular = 0.0
            if timed_out and self._safety_state != 'TIMEOUT_STOP':
                self._safety_state = 'TIMEOUT_STOP'
                self.get_logger().warn(
                    f'No cmd_vel for {elapsed:.2f}s — emergency stop')
            elif self._obstacle_too_close:
                self._safety_state = 'OBSTACLE_STOP'
        else:
            desired_linear = self._last_cmd.linear.x
            desired_angular = self._last_cmd.angular.z
            self._safety_state = 'NOMINAL'

        # ── Velocity clamping ───────────────────────────────────────
        v_max = self.get_parameter('max_linear_vel').value
        w_max = self.get_parameter('max_angular_vel').value
        desired_linear = max(-v_max, min(desired_linear, v_max))
        desired_angular = max(-w_max, min(desired_angular, w_max))

        # ── Acceleration limiting ───────────────────────────────────
        a_lin_max = self.get_parameter('max_linear_accel').value
        a_ang_max = self.get_parameter('max_angular_accel').value

        dv = desired_linear - self._current_linear
        dw = desired_angular - self._current_angular

        max_dv = a_lin_max * dt
        max_dw = a_ang_max * dt

        if abs(dv) > max_dv:
            self._current_linear += math.copysign(max_dv, dv)
        else:
            self._current_linear = desired_linear

        if abs(dw) > max_dw:
            self._current_angular += math.copysign(max_dw, dw)
        else:
            self._current_angular = desired_angular

        # ── Publish safe command ────────────────────────────────────
        safe_cmd = Twist()
        safe_cmd.linear.x = self._current_linear
        safe_cmd.angular.z = self._current_angular
        self.cmd_pub.publish(safe_cmd)

        # ── Publish status ──────────────────────────────────────────
        status = String()
        status.data = self._safety_state
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

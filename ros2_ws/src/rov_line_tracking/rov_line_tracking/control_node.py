#!/usr/bin/env python3
"""
==============================================================================
FAZ 4: Closed-Loop PID and Autonomous Task Node (control_node)
==============================================================================
Task Requirements:
- T-4.1: 10 Hz asynchronous main task loop (create_timer).
- T-4.2: Depth Stabilization (Heave) PID controller (Target: 2.0m, Precision: ±5 cm, Max Force: 7 N).
- T-4.3: Yaw PID controller from vision line error.
- T-4.4: Surge (Forward) & Sway (Lateral) force limits (Max Force: 10 N).
- T-4.5: Output Wrench/setpoints to Pixhawk via Micro-XRCE-DDS bridge.
==============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget
from rov_line_tracking.fail_safe import FailSafeManager, SystemState
import math


class PIDController:
    """Discrete PID controller with anti-windup clamping and output saturation."""
    def __init__(self, kp: float, ki: float, kd: float, max_output: float, dt: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.dt = dt

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, target: float, current: float) -> float:
        error = target - current
        self.integral += error * self.dt
        
        # Anti-windup clamping for integral term
        max_i = self.max_output * 0.5
        self.integral = max(-max_i, min(self.integral, max_i))

        derivative = (error - self.previous_error) / self.dt if self.dt > 0 else 0.0
        self.previous_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Output saturation limit
        return max(-self.max_output, min(output, self.max_output))

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Declare parameters
        self.declare_parameter('loop_frequency', 10.0)
        self.declare_parameter('target_depth', 2.0)
        self.declare_parameter('depth_tolerance', 0.05)

        self.declare_parameter('heave_kp', 4.0)
        self.declare_parameter('heave_ki', 0.2)
        self.declare_parameter('heave_kd', 1.5)
        self.declare_parameter('max_heave_force', 7.0)

        self.declare_parameter('yaw_kp', 0.05)
        self.declare_parameter('yaw_ki', 0.001)
        self.declare_parameter('yaw_kd', 0.01)
        self.declare_parameter('max_yaw_torque', 3.0)

        self.declare_parameter('max_surge_force', 10.0)
        self.declare_parameter('max_sway_force', 10.0)
        self.declare_parameter('default_surge_speed', 3.0)

        self.declare_parameter('lost_line_timeout_count', 30)
        self.declare_parameter('emergency_ascent_force', 5.0)

        # Get dynamic parameters
        loop_freq = self.get_parameter('loop_frequency').get_parameter_value().double_value
        dt = 1.0 / loop_freq if loop_freq > 0 else 0.1

        self.target_depth = self.get_parameter('target_depth').get_parameter_value().double_value
        self.depth_tolerance = self.get_parameter('depth_tolerance').get_parameter_value().double_value

        heave_kp = self.get_parameter('heave_kp').get_parameter_value().double_value
        heave_ki = self.get_parameter('heave_ki').get_parameter_value().double_value
        heave_kd = self.get_parameter('heave_kd').get_parameter_value().double_value
        self.max_heave_force = self.get_parameter('max_heave_force').get_parameter_value().double_value

        yaw_kp = self.get_parameter('yaw_kp').get_parameter_value().double_value
        yaw_ki = self.get_parameter('yaw_ki').get_parameter_value().double_value
        yaw_kd = self.get_parameter('yaw_kd').get_parameter_value().double_value
        self.max_yaw_torque = self.get_parameter('max_yaw_torque').get_parameter_value().double_value

        self.max_surge_force = self.get_parameter('max_surge_force').get_parameter_value().double_value
        self.max_sway_force = self.get_parameter('max_sway_force').get_parameter_value().double_value
        self.default_surge = self.get_parameter('default_surge_speed').get_parameter_value().double_value

        timeout_count = self.get_parameter('lost_line_timeout_count').get_parameter_value().integer_value
        ascent_force = self.get_parameter('emergency_ascent_force').get_parameter_value().double_value

        # T-4.2: Heave (Depth) PID setup
        self.heave_pid = PIDController(heave_kp, heave_ki, heave_kd, self.max_heave_force, dt=dt)

        # T-4.3: Yaw PID setup
        self.yaw_pid = PIDController(yaw_kp, yaw_ki, yaw_kd, self.max_yaw_torque, dt=dt)

        # T-5.1 & T-5.2: Fail Safe Manager setup
        self.fail_safe = FailSafeManager(timeout_count=timeout_count, emergency_ascent_force=ascent_force)

        # State Variables
        self.current_depth = 0.0
        self.line_error = float('nan')

        # Subscriptions
        # Subscribes to PX4 vehicle odometry (Pixhawk TELEM2 via MAVROS)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            10
        )

        # Subscribes to line error from vision_node
        self.line_error_sub = self.create_subscription(
            Float64,
            '/vision/line_error',
            self.line_error_callback,
            10
        )

        # Publishers
        # Generic ROS Wrench publisher
        self.wrench_pub = self.create_publisher(Wrench, '/control/wrench_command', 10)

        # PX4 MAVROS Publishers (T-4.5)
        self.attitude_target_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        # T-4.1: Asynchronous timer operating at 10 Hz
        timer_period = 1.0 / loop_freq
        self.timer = self.create_timer(timer_period, self.control_loop_callback)

        self.get_logger().info(f"Control Node running at {loop_freq} Hz. Target Depth: {self.target_depth}m (±{self.depth_tolerance*100:.0f}cm)")

    def odom_callback(self, msg: Odometry):
        """Processes depth/odometry feedback from Pixhawk PX4."""
        # Odometry position z (depth is usually negative Z in ENU frame in MAVROS)
        self.current_depth = abs(float(msg.pose.pose.position.z))

    def line_error_callback(self, msg: Float64):
        """Processes centroid deviation error from vision_node."""
        self.line_error = msg.data

    def publish_offboard_heartbeat(self):
        """MAVROS handles offboard heartbeat automatically when setpoints are published."""
        pass

    def control_loop_callback(self):
        """T-4.1: 10 Hz Async Timer Main Control Loop."""
        self.publish_offboard_heartbeat()

        # Update Fail-Safe State Machine
        system_state = self.fail_safe.update(self.line_error)

        surge_force = 0.0
        sway_force = 0.0
        yaw_torque = 0.0
        heave_force = 0.0

        # T-4.2: Depth (Heave) PID Computation (Target: 2.0m, Max Force: 7 N)
        depth_error = self.target_depth - self.current_depth
        if abs(depth_error) <= self.depth_tolerance:
            # Within ±5 cm target precision window
            heave_force = 0.0
        else:
            heave_force = self.heave_pid.compute(self.target_depth, self.current_depth)
            # Enforce max heave force limit (7 N)
            heave_force = max(-self.max_heave_force, min(heave_force, self.max_heave_force))

        # Handle Horizontal Axes & State Machine Logic
        if system_state == SystemState.LINE_FOLLOWING:
            # T-4.3: Yaw PID computation from line error (target error = 0)
            yaw_torque = self.yaw_pid.compute(0.0, self.line_error)
            surge_force = self.default_surge
            sway_force = 0.0

        elif system_state == SystemState.SEARCH_MODE:
            # T-5.1: Dead Reckoning Search Pattern
            self.get_logger().warn_throttle(2.0, "State: SEARCH MODE (Dead Reckoning)")
            surge_force, sway_force, yaw_torque = self.fail_safe.get_search_pattern_wrench(self.default_surge)

        elif system_state == SystemState.EMERGENCY_SURFACE:
            # T-5.2: Emergency surface procedure
            self.get_logger().error_throttle(1.0, "State: EMERGENCY SURFACE ASCENT! Motors disarmed, surfacing...")
            surge_force, sway_force, yaw_torque, heave_force = self.fail_safe.get_emergency_surface_wrench()

        # T-4.4: Enforce Surge / Sway limits (Max 10 N)
        surge_force = max(-self.max_surge_force, min(surge_force, self.max_surge_force))
        sway_force = max(-self.max_sway_force, min(sway_force, self.max_sway_force))

        # T-4.5: Publish calculated forces/torques (Wrench) & PX4 rates setpoint
        wrench_msg = Wrench()
        wrench_msg.force.x = surge_force
        wrench_msg.force.y = sway_force
        wrench_msg.force.z = heave_force
        wrench_msg.torque.z = yaw_torque
        self.wrench_pub.publish(wrench_msg)

        rates_msg = AttitudeTarget()
        rates_msg.header.stamp = self.get_clock().now().to_msg()
        rates_msg.type_mask = 7  # Ignore attitude (1+2+4), use body rates
        rates_msg.body_rate.x = 0.0
        rates_msg.body_rate.y = 0.0
        rates_msg.body_rate.z = float(yaw_torque)
        # MAVROS AttitudeTarget supports 1D thrust. For 3D thrust, additional MAVROS plugins or manual control might be needed.
        # Here we map the heave force to the main throttle/thrust axis.
        rates_msg.thrust = float(-heave_force / self.max_heave_force)
        self.attitude_target_pub.publish(rates_msg)

        self.get_logger().debug(
            f"State: {system_state.name} | Depth: {self.current_depth:.2f}m (Heave F: {heave_force:.1f}N) | "
            f"Surge: {surge_force:.1f}N | Yaw T: {yaw_torque:.2f}Nm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

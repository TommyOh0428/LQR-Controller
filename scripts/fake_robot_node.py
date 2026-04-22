#!/usr/bin/env python3
"""Fake robot node for headless Nav2 benchmarking without Gazebo.

Replaces Gazebo with a simple velocity integrator that publishes:
  /clock, /odom, /scan, /amcl_pose, and TF frames.

Subscribes to /cmd_vel and integrates unicycle kinematics to update pose.

Usage:
    python3 fake_robot_node.py [--ros-args -p start_x:=-2.0 -p start_y:=-0.5]
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class FakeRobotNode(Node):
    """Lightweight robot simulator: integrates cmd_vel → odom/TF/scan."""

    def __init__(self):
        super().__init__('fake_robot')

        # Declare parameters
        self.declare_parameter('start_x', -2.0)
        self.declare_parameter('start_y', -0.5)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('noise_v', 0.0)
        self.declare_parameter('noise_omega', 0.0)

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        rate = self.get_parameter('update_rate').value
        self.noise_v = self.get_parameter('noise_v').value
        self.noise_omega = self.get_parameter('noise_omega').value
        self.dt = 1.0 / rate
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        # Publishers
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', amcl_qos)

        # TF broadcasters
        self.tf_br = TransformBroadcaster(self)
        self.static_tf_br = StaticTransformBroadcaster(self)

        # Subscribe to cmd_vel_nav (raw controller output, before velocity_smoother).
        # navigation_launch.py remaps controller_server's cmd_vel → cmd_vel_nav and
        # velocity_smoother outputs cmd_vel_smoothed → cmd_vel.  Subscribing to
        # cmd_vel_nav bypasses the smoother zeroing the topic on abort.
        # Also subscribe to /cmd_vel for recovery behavior commands (backup/spin)
        # which behavior_server publishes directly without going through the smoother.
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._recovery_cmd_vel_cb, 10)
        self._last_nav_stamp = self.get_clock().now()

        # Publish static TFs once
        self._publish_static_tfs()

        # Main loop timer
        self.create_timer(self.dt, self._update)
        self.get_logger().info(
            f'Fake robot at ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f}), '
            f'rate={rate:.0f} Hz')

    # ── static TF ────────────────────────────────────────────────────

    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        # map → odom  (perfect localization — identity)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # base_footprint → base_link
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        t2.transform.rotation.w = 1.0
        tfs.append(t2)

        # base_link → base_scan  (LiDAR mount offset on TurtleBot3 Burger)
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'base_scan'
        t3.transform.translation.x = -0.032
        t3.transform.translation.z = 0.172
        t3.transform.rotation.w = 1.0
        tfs.append(t3)

        self.static_tf_br.sendTransform(tfs)

    # ── callbacks ────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        """Primary: raw controller output on /cmd_vel_nav."""
        self.v_cmd = msg.linear.x
        self.omega_cmd = msg.angular.z
        self._last_nav_stamp = self.get_clock().now()

    def _recovery_cmd_vel_cb(self, msg: Twist):
        """Fallback: behavior_server recovery commands on /cmd_vel.

        Only apply if no cmd_vel_nav has arrived in the last 0.2 s
        (i.e. the controller is not currently active).
        """
        age = (self.get_clock().now() - self._last_nav_stamp).nanoseconds / 1e9
        if age > 0.2:
            self.v_cmd = msg.linear.x
            self.omega_cmd = msg.angular.z

    def _update(self):
        # Integrate with optional noise
        v = self.v_cmd + np.random.normal(0, self.noise_v)
        omega = self.omega_cmd + np.random.normal(0, self.noise_omega)

        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw += omega * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        now_msg = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # 1. /clock  (drives use_sim_time for all other nodes)
        clock = Clock()
        clock.clock = now_msg
        self.clock_pub.publish(clock)

        # 2. TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp = now_msg
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

        # 3. /odom
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v_cmd
        odom.twist.twist.angular.z = self.omega_cmd
        self.odom_pub.publish(odom)

        # 4. /amcl_pose  (ground-truth pose, since we skip real AMCL)
        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp = now_msg
        amcl.header.frame_id = 'map'
        amcl.pose.pose.position.x = self.x
        amcl.pose.pose.position.y = self.y
        amcl.pose.pose.orientation.z = qz
        amcl.pose.pose.orientation.w = qw
        self.amcl_pub.publish(amcl)

        # 5. /scan  (360° max-range readings — no dynamic obstacles)
        scan = LaserScan()
        scan.header.stamp = now_msg
        scan.header.frame_id = 'base_scan'
        num_rays = 360
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi * (1.0 - 1.0 / num_rays)
        scan.angle_increment = 2.0 * math.pi / num_rays
        scan.time_increment = 0.0
        scan.scan_time = self.dt
        scan.range_min = 0.12
        scan.range_max = 3.5
        scan.ranges = [3.5] * num_rays
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = FakeRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

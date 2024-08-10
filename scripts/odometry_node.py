#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.subscriber_left_encoder = self.create_subscription(
            Float32,
            'left_wheel_encoder',
            self.left_encoder_callback,
            10)
        self.subscriber_right_encoder = self.create_subscription(
            Float32,
            'right_wheel_encoder',
            self.right_encoder_callback,
            10)
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_base = 0.13  # meters
        self.wheel_radius = 0.3  # meters
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

    def left_encoder_callback(self, msg):
        self.left_wheel_pos = msg.data
        self.update_odometry()

    def right_encoder_callback(self, msg):
        self.right_wheel_pos = msg.data
        self.update_odometry()

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate velocities
        v_left = (self.left_wheel_pos * self.wheel_radius) / dt
        v_right = (self.right_wheel_pos * self.wheel_radius) / dt
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.wheel_base

        # Update position and orientation
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt

        # Create quaternion from yaw angle
        quat = self.quaternion_from_euler(0, 0, self.theta)

        # Create and publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        self.publisher_.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

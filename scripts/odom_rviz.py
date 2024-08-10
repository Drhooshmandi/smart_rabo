#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
from geometry_msgs.msg import Twist

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Subscriptions
        self.subscription_left = self.create_subscription(
            Int32,
            '/left_motor_ticks',
            self.listener_callback_left,
            10
        )
        self.subscription_right = self.create_subscription(
            Int32,
            '/right_motor_ticks',
            self.listener_callback_right,
            10
        )
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer=self.create_timer(0.1,self.update_odometry)



        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_base = 0.134  # meters
        self.wheel_radius = 0.03  # meters
        self.pulses_per_rotation = 1550
        self.left_wheel_pos = 0
        self.right_wheel_pos = 0
        self.last_left_pulse_count = 0
        self.last_right_pulse_count = 0

        

    def listener_callback_left(self, msg1):
        self.left_wheel_pos = msg1.data
        #self.get_logger().info(f'Left Received message: {msg1.data}')
        #self.update_odometry()

    def listener_callback_right(self, msg2):
        self.right_wheel_pos = msg2.data
        #self.get_logger().info(f'Right Received message: {msg2.data}')
       # self.update_odometry()


   
    def publish_twist(self, movement):
        twist = Twist()
        twist.linear.x = movement[0] * 0.1
        twist.angular.z = movement[1] * 0.5
        self.publisher_.publish(twist)

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate the difference in encoder pulses
        delta_left_pulses = self.left_wheel_pos - self.last_left_pulse_count
        delta_right_pulses = self.right_wheel_pos - self.last_right_pulse_count

        self.last_left_pulse_count = self.left_wheel_pos
        self.last_right_pulse_count = self.right_wheel_pos

        # Calculate the distance each wheel has traveled
        left_distance = (delta_left_pulses / self.pulses_per_rotation) * (2 * math.pi * self.wheel_radius)
        right_distance = (delta_right_pulses / self.pulses_per_rotation) * (2 * math.pi * self.wheel_radius)

        # Calculate velocities
        v_left = left_distance / dt
        v_right = right_distance / dt
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.wheel_base

        # Update position and orientation
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt

        # Normalize theta
        self.theta = self.normalize_angle(self.theta)

        # here, we print the results for inspection
        print("x,y,theta: (%s,%s,%s)"%(self.x,self.y,self.theta))
        #print("Current value left encoder %s"%self.currentValueLeftEncoder)
       # print("Time difference %s"%(dt))
        # Create quaternion from yaw angle
        qx, qy, qz, qw = self.quaternion_from_euler(0, 0, self.theta)

        # Create and publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set the orientation
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        if self.theta<= 1000*3.14/2:
            self.publish_twist([1,1])
        else:
            self.publish_twist([0,0])
       # self.publisher_.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

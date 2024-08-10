#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import math

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.left = self.create_subscription(
            Int32,
            '/left_motor_ticks',
            self.Left_listener_callback,
            10)
        
        self.right = self.create_subscription(
            Int32,
            '/right_motor_ticks',
            self.Rigth_listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot parameters (example values, adjust as per your robot)
        self.wheel_radius = 0.03 # meters
        self.wheel_base = 0.134  # meters

        # Control parameters
        self.circle_radius = 0.25# meters
        self.linear_velocity = 0.1 # meters per second (adjust as needed)

        self.left  # prevent unused variable warning
        self.right


    def Left_listener_callback(self, msg):
        pass
        #self.get_logger().info(f'Left Received message: {msg.data}'
        
    def Rigth_listener_callback(self, msg):
        pass
        #self.get_logger().info(f'Right Received message: {msg.data}')
    
    
    def calculate_cmd_vel(self):
        # Calculate angular velocity to achieve circular motion
        angular_velocity = self.linear_velocity / self.circle_radius

        # Calculate linear velocity of each wheel
        v_left = self.linear_velocity - angular_velocity * self.wheel_base / 2
        v_right = self.linear_velocity + angular_velocity * self.wheel_base / 2

        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = angular_velocity

        # Publish Twist message
        self.publisher_.publish(twist_msg)

        self.get_logger().info(f"Sending cmd_vel: linear={twist_msg.linear.x}, angular={twist_msg.angular.z}")


'''def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()'''

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        node.calculate_cmd_vel()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot when shutting down
        stop_twist = Twist()
        node.publisher_.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class RoboController(Node):
    def __init__(self):
        super().__init__('keyBoard_node')
       # self.subscription = self.create_subscription(Joy,'/joy',self.key_callback, 10)
        self.leds_publisher = self.create_publisher(Int8, '/LEDs', 10)
        self.servo_publisher = self.create_publisher(Int8, '/servo', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.1#0.05  # linear speed
        self.turn = 1.0   # angular speed

        self.key_mapping = {
            'w': [1, 0],
            'x': [-1, 0],
            'a': [0, 1],
            'd': [0, -1],
            's': [0, 0]
        }
      

        self.i = 0
        self.servo_position = 0
        self.get_logger().info('Code started.')

 
    
    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key in self.key_mapping:
                    self.publish_twist(self.key_mapping[key])
                elif key in ['1', '2', '3', '4']:
                    msg = Int8()
                    msg.data = int(key)-1
                    self.leds_publisher.publish(msg)
                    self.get_logger().info(f'Publishing: {msg.data}')
                elif key in ['5', '6']:
                    if int(key)== 5:
                        self.servo_position = 50
                    elif int(key)== 6:
                        self.servo_position = 0
                    servo_msg = Int8()
                    servo_msg.data = self.servo_position
                    self.servo_publisher.publish(servo_msg)
                    self.get_logger().info(f'Publishing: {servo_msg.data}')
                elif key == '\x03':  # Ctrl+C
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


  

    def publish_twist(self, movement):
        twist = Twist()
        twist.linear.x = movement[0] * self.speed
        twist.angular.z = movement[1] * self.turn
        self.publisher_.publish(twist)
        self.get_logger().info(f'Published twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

    def timer_callback(self):
        self.get_logger().info(f'Publishing:')
          

   

def main(args=None):
    rclpy.init(args=args)
    node = RoboController()
    try:
       #rclpy.spin(node)
       node.run()
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

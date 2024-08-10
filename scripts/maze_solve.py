#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import math
import time

class MazeNode(Node):

    def __init__(self):
        super().__init__('maze_solve_node')

        self.front = self.create_subscription(
            Int32,
            '/distance_front',
            self.Front_listener_callback,
            10)
        
        self.right= self.create_subscription(
            Int32,
            '/distance_right',
            self.Rigth_listener_callback,
            10)
        
        self.left = self.create_subscription(
            Int32,
            '/distance_left',
            self.Left_listener_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize encoder values
        self.sonar_left = 0
        self.sonar_right = 0
        self.sonar_front=0
        # Robot parameters (example values, adjust as per your robot)
        self.wheel_radius = 0.027  # meters
        self.wheel_base = 0.13  # meters

        # Control parameters
        self.circle_radius =0.0 #0.2# meters
        self.linear_velocity =0.0# 0.1 # meters per second (adjust as needed)

        self.left  # prevent unused variable warning
        self.right
        #Parameter Values
        self.baseSpeed=0.1*1000
        self.oldErrorP=0.0
        self.errorI=0.0
        self.P=4.0
        self.I=0.0
        self.D=0.1

        self.init_R =True
        self.init_L =True
        self.init_F =True




    def Left_listener_callback(self, msg):
        self.sonar_left = msg.data
        self.get_logger().info(f'Left Received Distance: {msg.data}')


    def Rigth_listener_callback(self, msg):
        self.sonar_right = msg.data
        self.get_logger().info(f'Right Received Distance: {msg.data}')


    def Front_listener_callback(self, msg):
        self.sonar_front= msg.data
        self.get_logger().info(f'Front Received Distance: {msg.data}')
    
    
    def timer_callback(self):
       # Create Twist message
        twist_msg = Twist()


        #twist_msg.linear.x = 0.0
        #twist_msg.angular.z =0.0
        #self.publisher_.publish(twist_msg)
        if self.sonar_front<10:

            '''if self.sonar_right>15:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z =1.0
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 1.0'''
            
            twist_msg.linear.x = 0.0
            twist_msg.angular.z =0.0
            self.publisher_.publish(twist_msg)
            time.sleep(1.0)

            self.init_R =True
            self.init_F =True

        else:
            if self.sonar_right>15:
                twist_msg.linear.x = 0.1
                twist_msg.angular.z =0.0
                self.init_R =True
            
            elif self.sonar_left>15:
                twist_msg.linear.x = 0.1
                twist_msg.angular.z =0.0 
                self.init_R =True
                

            else:
               
               if(self.init_R ):
                    self.oldErrorP=0
                
               self.init_R =False
               
               errorP = self.sonar_left - self.sonar_right
               errorD = errorP- self.oldErrorP
               self.errorI = (2.0 / 3.0) * self.errorI +errorP 
               PID_value = int( self.P * errorP+ self.D * errorD + self.I * self.errorI )
               self.oldErrorP = errorP
                
               wR = self.baseSpeed + 1*PID_value
               wL = self.baseSpeed - 1*PID_value
               
                
               twist_msg.linear.x = (wR+wL)/2000
               twist_msg.angular.z =2*(wR-wL)/(10000*0.13)
               
           
            self.get_logger().info(f"Sending cmd_vel: linear={twist_msg.linear.x}, angular={twist_msg.angular.z}")
            self.publisher_.publish(twist_msg)

  


        # Publish Twist message
        

       # self.get_logger().info(f"Sending cmd_vel: linear={twist_msg.linear.x}, angular={twist_msg.angular.z}")


'''def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()'
    
    
     if(self.init_F ):
                    self.oldErrorP=0
                
                self.init_R =True
                self.init_F =False
                errorP = self.sonar_right-8
                errorD = errorP- self.oldErrorP
                self.errorI = (2.0 / 3.0) * self.errorI +errorP 
                PID_value = int( self.P * errorP+ self.D * errorD + self.I * self.errorI )
                self.oldErrorP = errorP
                
                wR = self.baseSpeed - 1*PID_value
                wL = self.baseSpeed  + 1*PID_value
                if wR<0:
                   wR=0
                
                if wL<0:
                   L=0'''

def main(args=None):
    rclpy.init(args=args)
    node = MazeNode()
    try:
        
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

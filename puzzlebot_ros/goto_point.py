import rclpy
from rclpy.node import Node
from rclpy import qos

import signal, os, time

import math

from .my_math import wrap_to_pi
from .my_math import euler_from_quaternion

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np

from timeit import default_timer as timer
  
class GotoPoint(Node):

    def __init__(self):
        super().__init__('goto_point')
        
        # Velocity setpoint topics (control_input=2 on the robot)
        self.pub_cmdR = self.create_publisher(Float32, 'VelocitySetR', 10)
        self.pub_cmdL = self.create_publisher(Float32, 'VelocitySetL', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.sub_odom = self.create_subscription(Odometry,'/odom',self.odom_callback,qos.qos_profile_sensor_data)
                                
        self.goto_dt = 0.1  # seconds
        self.timer_goto = self.create_timer(self.goto_dt, self.goto_loop)
                
        self.velocityR = 0.0
        self.velocityL = 0.0
        
        self.w_setR = 0.0
        self.w_setL = 0.0
                        
        # Target points        
        self.target_x = [1,  1,  0,  0]
        self.target_y = [0, -1, -1,  0]
        
        # Target gazebo simulation points    
        self.target_x = [2,  2,  1,  1]
        self.target_y = [0, -1, -1,  0]
        
        self.current_point = 0
        
        self.Dmin = 0.05
        self.Kd = 1.0
        self.Kt = 1.0
        
        self.v_max = 0.2
        self.w_max = 2
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        
        self.Sig = np.array([[0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0]])

        
        self.vc = 0
        self.wc = 0
        
        self.wheel_radius = 0.05
        self.robot_width = 0.173
                
        self.enc_received = False
        
        
    def encR_callback(self, msg):
        self.velocityR = msg.data
        self.enc_received = True

        
    def encL_callback(self, msg):
        self.velocityL = msg.data


    def laser_callback(self, msg):
        self.laser_distance = msg.data

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.pose_theta = yaw
                                     
        self.Sig[0,0] = msg.pose.covariance[0]
        self.Sig[0,1] = msg.pose.covariance[1]
        self.Sig[1,0] = msg.pose.covariance[6]
        self.Sig[1,1] = msg.pose.covariance[7]
        self.Sig[2,2] = msg.pose.covariance[35]
        

    def goto_loop(self):    
                    
        if self.current_point<len(self.target_x):
        
            err_x = self.target_x[self.current_point] - self.pose_x
            err_y = self.target_y[self.current_point] - self.pose_y
            err_d = math.sqrt(err_x**2+err_y**2)
        
            err_theta = math.atan2(err_y,err_x)-self.pose_theta
            err_theta = wrap_to_pi(err_theta)
            
            self.vc = self.Kd*err_d
            self.wc = self.Kt*err_theta
            
            if self.vc>self.v_max:
                self.vc = self.v_max
                              
            if abs(self.wc)>self.w_max:
                self.wc = np.sign(self.wc)*self.w_max
                
            if abs(err_theta)>0.2:
                self.vc = 0
                
            self.w_setR = (self.vc + self.wc*self.robot_width/2) / self.wheel_radius
            self.w_setL = (self.vc - self.wc*self.robot_width/2) / self.wheel_radius
            
            if err_d<self.Dmin:
                self.current_point = self.current_point + 1                
        else:
            self.vc = 0.0
            self.wc = 0.0
            self.w_setR = 0.0                       
            self.w_setL = 0.0 
             
        msg_cmdR = Float32()
        msg_cmdL = Float32()
        msg_cmdR.data = self.w_setR    
        msg_cmdL.data = self.w_setL    
        self.pub_cmdR.publish(msg_cmdR)
        self.pub_cmdL.publish(msg_cmdL)

    
    def stop_handler(self,signum, frame):
        msg = Float32()                       
        msg.data = 0.0                     
        self.pub_cmdR.publish(msg)
        self.pub_cmdL.publish(msg)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    goto_point = GotoPoint()

    signal.signal(signal.SIGINT, goto_point.stop_handler)
        
    rclpy.spin(goto_point)
    
    goto_point.destroy_node()
        

if __name__ == '__main__':
    main()

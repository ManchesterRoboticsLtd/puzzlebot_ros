import rclpy
from rclpy.node import Node
from rclpy import qos

import signal, os, time

import math
from .my_math import euler_from_quaternion
from .my_math import quaternion_from_euler
from .my_math import wrap_to_pi

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np

from timeit import default_timer as timer
  
class Deadreckoning(Node):

    def __init__(self):
        super().__init__('dead_reckoning')
                
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.sub_robot_vel = self.create_subscription(TwistStamped,'robot_vel',self.robot_vel_callback,qos.qos_profile_sensor_data)
                                                        
        self.tf_broadcaster = TransformBroadcaster(self)
                                                
        self.run_dt = 0.02  # seconds
        self.timer_run = self.create_timer(self.run_dt, self.run_loop)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
                
        self.velocityR = 0.0
        self.velocityL = 0.0
        
        self.Vr = 0.0
        self.Wr = 0.0
                
        self.sigma_squared = 0.1
        
        self.Sig = np.array([[0,0,0],
                             [0,0,0],
                             [0,0,0]])
                        
        self.wheel_radius = 0.05
        self.robot_width = 0.1875
        
        self.total_time = 0;
        
        self.first_stamp = True
        
        
    def encR_callback(self, msg):
        self.velocityR = msg.data

        
    def encL_callback(self, msg):
        self.velocityL = msg.data


    def robot_vel_callback(self, msg):
        if self.first_stamp == True:
            self.first_stamp = False
            self.prev_stamp = msg.header.stamp
        
        if self.first_stamp == False:
            self.dt = msg.header.stamp.sec - self.prev_stamp.sec + (msg.header.stamp.nanosec - self.prev_stamp.nanosec)*1e-9

        self.prev_stamp = msg.header.stamp
        
        R = self.wheel_radius
        L = self.robot_width
        
        self.Vr = msg.twist.linear.x
        self.Wr = msg.twist.angular.z
                        
        self.pose_x = self.pose_x + self.dt*self.Vr*math.cos(self.pose_theta+self.dt*self.Wr/2)
        self.pose_y = self.pose_y + self.dt*self.Vr*math.sin(self.pose_theta+self.dt*self.Wr/2)
        self.pose_theta = wrap_to_pi(self.pose_theta + self.dt*self.Wr)

        H = np.array([[1, 0, -self.dt*self.Vr*math.sin(self.pose_theta)],
                      [0, 1, self.dt*self.Vr*math.cos(self.pose_theta)],
                      [0, 0, 1]])
             
        dH = np.array([[0.5*self.dt*R*math.cos(self.pose_theta), 0.5*self.dt*R*math.cos(self.pose_theta)],
                       [0.5*self.dt*R*math.sin(self.pose_theta), 0.5*self.dt*R*math.sin(self.pose_theta)],
                       [self.dt*R/L, -self.dt*R/L]])
                       
        K = np.array([[self.sigma_squared*abs(self.velocityR),   0                                     ],
                      [0,                                        self.sigma_squared*abs(self.velocityL)]])
                      
        Q = dH @ K @ dH.T                  # Q = dH*K*dH'
        
        self.Sig = H @ self.Sig @ H.T + Q  # Sig = H*Sig*H' + Q

    
    def run_loop(self):
        
        odom = Odometry()
        
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_euler(0.0,0.0,self.pose_theta)
        odom.pose.covariance[0] = self.Sig[0][0]
        odom.pose.covariance[1] = self.Sig[0][1]
        odom.pose.covariance[6] = self.Sig[1][0]
        odom.pose.covariance[7] = self.Sig[1][1]
        odom.pose.covariance[35] = self.Sig[2][2]
        
        self.pub_odom.publish(odom)
               
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
                    

    def stop(self):
        msg_cmdR = Float32()
        msg_cmdL = Float32()                        
        msg_cmdR.data = 0.0
        msg_cmdL.data = 0.0                       
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
    
    dead_reckon = Deadreckoning()

    signal.signal(signal.SIGINT, dead_reckon.stop_handler)
        
    rclpy.spin(dead_reckon)
    
    dead_reckon.destroy_node()


if __name__ == '__main__':
    main()

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
from nav_msgs.msg import Odometry
from aruco_msgs.msg import MarkerArray

import numpy as np

from timeit import default_timer as timer
  
class Kalman(Node):

    def __init__(self):
        super().__init__('kalman')
                
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.sub_aruco = self.create_subscription(MarkerArray,'/marker_publisher/markers',self.aruco_callback,qos.qos_profile_sensor_data)
                                                
        self.run_dt = 0.02  # seconds
        self.timer_run = self.create_timer(self.run_dt, self.run_loop)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
                
        self.velocityR = 0.0
        self.velocityL = 0.0
        
        self.Vr = 0.0
        self.Wr = 0.0
        
        self.wheel_radius = 0.05
        self.robot_width = 0.1875
        
        self.sigma_squared = 0.1
        
        self.Sig = np.array([[0,0,0],
                             [0,0,0],
                             [0,0,0]])
                              
        self.map = [[0,   3,   1,   0],
                    [1,   0,  -1,   3.1415]]
                
        self.total_time = 0;
        
        self.timer_run = timer()
        
        
    def encR_callback(self, msg):
        self.velocityR = msg.data

        
    def encL_callback(self, msg):
        self.velocityL = msg.data


    def aruco_callback(self, msg):
        for aruco_marker in msg.markers:
            marker = [0, 0, 0, 0]
            roll, pitch, yaw = euler_from_quaternion(aruco_marker.pose.pose.orientation)
            marker[0] = aruco_marker.id
            marker[1] = aruco_marker.pose.pose.position.z
            marker[2] = -aruco_marker.pose.pose.position.x
            marker[3] = wrap_to_pi(-pitch)
            self.kalman_correction(marker,0.1)

    
    def run_loop(self):
            
        self.kalman_prediction()
        
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
               
        
    def kalman_prediction(self):
        dt = timer()-self.timer_run
        self.timer_run = timer()
        
        R = self.wheel_radius
        L = self.robot_width
    
        self.Vr = (self.velocityR+self.velocityL)*self.wheel_radius/2
        self.Wr = (self.velocityR-self.velocityL)*self.wheel_radius/(self.robot_width)
                
        self.pose_x = self.pose_x + dt*self.Vr*math.cos(self.pose_theta+dt*self.Wr/2)
        self.pose_y = self.pose_y + dt*self.Vr*math.sin(self.pose_theta+dt*self.Wr/2)
        self.pose_theta = wrap_to_pi(self.pose_theta + dt*self.Wr)
        
        H = np.array([[1, 0, -dt*self.Vr*math.sin(self.pose_theta)],
                      [0, 1, dt*self.Vr*math.cos(self.pose_theta)],
                      [0, 0, 1]])
             
        dH = np.array([[0.5*dt*R*math.cos(self.pose_theta), 0.5*dt*R*math.cos(self.pose_theta)],
                       [0.5*dt*R*math.sin(self.pose_theta), 0.5*dt*R*math.sin(self.pose_theta)],
                       [dt*R/L, -dt*R/L]])
                       
        K = np.array([[self.sigma_squared*abs(self.velocityR),   0                                     ],
                      [0,                                        self.sigma_squared*abs(self.velocityL)]])
                      
        Q = dH @ K @ dH.T                  # Q = dH*K*dH'
        
        self.Sig = H @ self.Sig @ H.T + Q  # Sig = H*Sig*H' + Q
        
        #print(self.Sig)

    def kalman_correction(self, marker, cov):
        #   Sensor model:
        #      z_x =  (m_x-s_x)*cos(s_theta) + (m_y-s_y)*sin(s_theta)
        #      z_y = -(m_x-s_x)*sin(s_theta) + (m_y-s_y)*cos(s_theta)
        #      z_theta = m_thteta - s_theta
        #
        #   Jacobian:
        #      -cos(s_theta)    -sin(s_theta)    -(m_x-s_x)*sin(s_theta) + (m_y-s_y)*cos(s_theta)
        #       sin(s_theta)    -cos(s_theta)    -(m_x-s_x)*cos(s_theta) - (m_y-s_y)*sin(s_theta)
        #       0                0               -1                          

        S = [0,0,0]
        
        S[0] = self.pose_x
        S[1] = self.pose_y
        S[2] = self.pose_theta
        
        found, M = self.get_landmark(marker[0])
        
        if found == 1:
            
            Z_hat = np.array([ (M[0]-S[0])*math.cos(S[2]) + (M[1]-S[1])*math.sin(S[2]),
                              -(M[0]-S[0])*math.sin(S[2]) + (M[1]-S[1])*math.cos(S[2]),
                                wrap_to_pi(M[2]-S[2])])
        
            G = np.array([[-math.cos(S[2]),  -math.sin(S[2]), -(M[0]-S[0])*math.sin(S[2]) + (M[1]-S[1])*math.cos(S[2])],
                          [ math.sin(S[2]),  -math.cos(S[2]), -(M[0]-S[0])*math.cos(S[2]) - (M[1]-S[1])*math.sin(S[2])],
                          [         0,           0,                                             -1]])
                      
            R = np.array([[cov,   0,     0],
                          [  0, cov,     0],
                          [  0,   0,   cov]])
                          
            diff = marker[1:4] - Z_hat
            marker[3] = wrap_to_pi(marker[3])
                      
            Z = G @ self.Sig @ G.T + R               # Z = G*Sig*G' + R
        
            K = self.Sig @ G.T @ np.linalg.inv(Z)    # Kalman gain:  K = Sig*G'*inv(Z)
        
            S = S + K @ diff                         # Update pose mean:  s = s + K*(Z-Z_hat)
        
            self.Sig = (np.eye(3)-K @ G) @ self.Sig  # Update covariance:  Sig = (I-K*G)*Sig
            
        self.pose_x = S[0]
        self.pose_y = S[1]
        self.pose_theta = S[2]


    def get_landmark(self, id):
        for mark in self.map:
            if mark[0] == id:
                return  1, mark[1:4]
                
        return 0, 0, 0, 0
            

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
    
    kalman = Kalman()

    signal.signal(signal.SIGINT, kalman.stop_handler)
        
    rclpy.spin(kalman)
    
    kalman.destroy_node()


if __name__ == '__main__':
    main()

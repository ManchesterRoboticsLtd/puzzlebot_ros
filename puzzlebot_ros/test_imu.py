import rclpy
from rclpy.node import Node
from rclpy import qos

import math

import signal, os, time

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

import matplotlib.pyplot as plt
import numpy as np


class ImuTest(Node):

    def __init__(self):
        super().__init__('pwm_control')
        
        #self.pub_linear = self.create_publisher(Float32, 'VelocityLinearX', 10)
        #self.pub_angular = self.create_publisher(Float32, 'VelocityAngular', 10)
        self.pub_velR = self.create_publisher(Float32, 'VelocitySetR', 10)
        self.pub_velL = self.create_publisher(Float32, 'VelocitySetL', 10)
        
        self.pub_imu_vel = self.create_publisher(Vector3, 'velocity_set', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.sub_imu_velocity = self.create_subscription(Vector3,'velocity',self.imu_velocity_callback,qos.qos_profile_sensor_data)
        self.sub_imu_acc = self.create_subscription(Vector3,'acc',self.imu_acc_callback,qos.qos_profile_sensor_data)
        
        self.dt = 0.01  # seconds
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.start_step = 300
        self.N = 640+self.start_step
        
        self.velocityR = 0.0
        self.velocityL = 0.0
        
        self.velocityR_all = [];
        self.velocityL_all = [];
        
        self.velocity_imu = [];
        self.acc_x_all = [];
        self.acc_y_all = [];
        self.acc_z_all = [];
        
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        
        msg_velocity_set = Vector3()
        msg_velocity_set.x = 0.0
        msg_velocity_set.y = 0.0
        msg_velocity_set.z = 0.0
        self.pub_imu_vel.publish(msg_velocity_set)
        
        self.i = 0;
        self.F = 0.3
        self.Fa = 0.3
        
    def encR_callback(self, msg):
        self.velocityR = self.velocityR*self.F + (1-self.F)*msg.data
        
    def encL_callback(self, msg):
        self.velocityL = self.velocityL*self.F + (1-self.F)*msg.data

    def imu_velocity_callback(self, msg):
        self.velocity_x = msg.x
        self.velocity_y = msg.y
        self.velocity_z = msg.z
        
    def imu_acc_callback(self, msg):
        self.acc_x = self.acc_x*self.Fa + (1-self.Fa)*msg.x
        self.acc_y = self.acc_y*self.Fa + (1-self.Fa)*msg.y
        self.acc_z = self.acc_z*self.Fa + (1-self.Fa)*msg.z


    def control_loop(self):
    
        msg_setR = Float32()
        msg_setL = Float32()
        
        msg_setR.data = 0.0       
        msg_setL.data = 0.0
            
        if self.i < self.N:
            
            self.velocityR_all.append(self.velocityR*0.0505)
            #self.velocity_imu.append(-np.sign(self.velocity_y)*math.sqrt(self.velocity_x**2+self.velocity_y**2+self.velocity_z**2))
            self.velocity_imu.append(self.velocity_z)
            
            self.acc_x_all.append(self.acc_x)
            self.acc_y_all.append(self.acc_y)
            self.acc_z_all.append(self.acc_z)
            
            if self.i >= self.start_step:
                msg_setR.data = 4.0*math.sin((self.i-self.start_step)*self.dt*0.5)     
                msg_setL.data = 4.0*math.sin((self.i-self.start_step)*self.dt*0.5)
            
               
        if self.i == self.N:
            msg_setR.data = 0.0       
            msg_setL.data = 0.0
            self.pub_velR.publish(msg_setR)
            self.pub_velL.publish(msg_setL)
            
            t = np.arange(0., self.N*self.dt, self.dt)
            
            plt.figure(1)
            plt.plot(t,self.velocityR_all)
            plt.plot(t,self.velocity_imu)
            plt.grid()
            plt.figure(2)
            #plt.plot(t,self.acc_x_all)
            #plt.plot(t,self.acc_y_all)
            plt.plot(t,self.acc_z_all)
            plt.show()
            
        self.pub_velR.publish(msg_setR)
        self.pub_velL.publish(msg_setL)
        
        self.i += 1

    def stop_handler(self,signum, frame):
        msg = Float32()                       
        msg.data = 0.0                     
        self.pub_velR.publish(msg)
        self.pub_velL.publish(msg)
        rclpy.shutdown()
        
        
def main(args=None):
    rclpy.init(args=args)

    imu_test = ImuTest()

    signal.signal(signal.SIGINT, imu_test.stop_handler)
    
    rclpy.spin(imu_test)

    imu_test.destroy_node()


if __name__ == '__main__':
    main()

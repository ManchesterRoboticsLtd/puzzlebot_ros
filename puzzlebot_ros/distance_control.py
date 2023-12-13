import rclpy
from rclpy.node import Node
from rclpy import qos

import signal, os, time

from std_msgs.msg import Float32

from .pid_controller import PidController

import matplotlib.pyplot as plt
import numpy as np

    
class DistanceControl(Node):

    def __init__(self):
        super().__init__('distance_control')
        
        self.pub_cmdR = self.create_publisher(Float32, 'ControlR', 10)
        self.pub_cmdL = self.create_publisher(Float32, 'ControlL', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.sub_laser = self.create_subscription(Float32,'LaserDistance',self.laser_callback,qos.qos_profile_sensor_data)
        
        self.pid_dt = 0.02  # seconds
        self.timer_pid = self.create_timer(self.pid_dt, self.velocity_loop)
        
        self.distance_dt = 0.1  # seconds
        self.timer_distance = self.create_timer(self.distance_dt, self.distance_loop)
                
        self.velocityR = 0.0
        self.velocityL = 0.0
        
        self.setR = 0.0
        self.setL = 0.0
        
        self.laser_distance = 0.0
        
        self.pidR = PidController()
        self.pidL = PidController()
        
        self.setD = 0.25
        self.Kd = 10.0
        

        
    def encR_callback(self, msg):
        self.velocityR = msg.data

        
    def encL_callback(self, msg):
        self.velocityL = msg.data


    def laser_callback(self, msg):
        self.laser_distance = msg.data


    def distance_loop(self):
        errD = self.laser_distance - self.setD
        
        if errD < 0.5:
            self.setR = self.Kd*errD
            self.setL = self.Kd*errD
        else:
            self.setR = 0.0
            self.setL = 0.0

    
    def velocity_loop(self):
    
        msg_cmdR = Float32()
        msg_cmdL = Float32()            
            
        msg_cmdR.data = self.pidR.GetControl(self.setR,self.velocityR,self.pid_dt)
        msg_cmdL.data = self.pidL.GetControl(self.setL,self.velocityL,self.pid_dt)
                          
        self.pub_cmdR.publish(msg_cmdR)
        self.pub_cmdL.publish(msg_cmdL)

        
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
    
    distance_control = DistanceControl()

    signal.signal(signal.SIGINT, distance_control.stop_handler)
    
    rclpy.spin(distance_control)
    
    distance_control.destroy_node()


if __name__ == '__main__':
    main()

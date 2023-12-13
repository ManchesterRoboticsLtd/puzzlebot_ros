import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import Float32

from .pid_controller import PidController

import matplotlib.pyplot as plt
import numpy as np

    
class VelocityControl(Node):

    def __init__(self):
        super().__init__('pwm_control')
        
        self.pub_cmdR = self.create_publisher(Float32, 'ControlR', 10)
        self.pub_cmdL = self.create_publisher(Float32, 'ControlL', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.dt = 0.02  # seconds
        self.timer = self.create_timer(self.dt, self.pid_loop)
        
        self.start_step = 20
        self.N = 100
        
        self.velocityR = 0.0
        self.velocityL = 0.0
        
        self.pidR = PidController()
        self.pidL = PidController()
        
        self.velocityR_all = []
        self.velocityL_all = []
        self.setR_all = []
        self.setL_all = []
        
        self.i = 0;
        
    def encR_callback(self, msg):
        self.velocityR = msg.data
        
    def encL_callback(self, msg):
        self.velocityL = msg.data


    def pid_loop(self):
    
        msg_cmdR = Float32()
        msg_cmdL = Float32()
        
        setR = 0.0       
        setL = 0.0
            
        if self.i < self.N:
            
            self.velocityR_all.append(self.velocityR)
            self.velocityL_all.append(self.velocityL)
            
            if self.i >= self.start_step:
                setR = 10      
                setL = 7                               
            
            msg_cmdR.data = self.pidR.GetControl(setR,self.velocityR,self.dt)
            msg_cmdL.data = self.pidL.GetControl(setL,self.velocityL,self.dt)
            
            self.setR_all.append(setR)
            self.setL_all.append(setL)
            
               
        self.pub_cmdR.publish(msg_cmdR)
        self.pub_cmdL.publish(msg_cmdL)
        
        if self.i == self.N:
            t = np.arange(0., self.N*self.dt, self.dt)
            plt.plot(t,self.velocityR_all)
            plt.plot(t,self.setR_all)
            plt.plot(t,self.velocityL_all)
            plt.plot(t,self.setL_all)
            plt.show()
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    velocity_control = VelocityControl()

    rclpy.spin(velocity_control)

    velocity_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

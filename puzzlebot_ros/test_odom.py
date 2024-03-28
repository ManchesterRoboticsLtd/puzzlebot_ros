import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

from .pid_controller import PidController
import math
from .my_math import wrap_to_pi
  
class VelocityControl(Node):

    def __init__(self):
        super().__init__('pwm_control')
        
        self.pub_cmdR = self.create_publisher(Float32, 'VelocitySetR', 10)
        self.pub_cmdL = self.create_publisher(Float32, 'VelocitySetL', 10)
        
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        self.sub_robot_vel = self.create_subscription(TwistStamped,'robot_vel',self.robot_vel_callback,qos.qos_profile_sensor_data)
        
        self.dt = 0.01  # seconds
        self.timer = self.create_timer(self.dt, self.loop)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        
        self.velocityR = 0.0
        self.velocityL = 0.0
                                
        self.first_stamp = True
        
        self.R = 0.0503
        self.L = 0.17
        
        self.target_theta = 6.15
        self.target_x = 1.0
        
        
    def encR_callback(self, msg):
        self.velocityR = msg.data
        
    def encL_callback(self, msg):
        self.velocityL = msg.data


    def robot_vel_callback(self, msg):
        if self.first_stamp == True:
            self.first_stamp = False
            self.prev_stamp = msg.header.stamp
        
        self.dt = msg.header.stamp.sec - self.prev_stamp.sec + (msg.header.stamp.nanosec - self.prev_stamp.nanosec)*1e-9

        self.prev_stamp = msg.header.stamp
        
        self.Vr = msg.twist.linear.x
        self.Wr = msg.twist.angular.z

        self.pose_x = self.pose_x + self.dt*self.Vr*math.cos(self.pose_theta+self.dt*self.Wr/2)
        self.pose_y = self.pose_y + self.dt*self.Vr*math.sin(self.pose_theta+self.dt*self.Wr/2)
        #self.pose_theta = wrap_to_pi(self.pose_theta + self.dt*self.Wr)
        self.pose_theta = (self.pose_theta + self.dt*self.Wr)
        

    def loop(self):
    
        self.dt = 0.01
        
        #self.Vr = (self.velocityR + self.velocityL)*self.R/2
        #self.Wr = (self.velocityR - self.velocityL)*self.R/self.L
        
        #self.pose_x = self.pose_x + self.dt*self.Vr*math.cos(self.pose_theta+self.dt*self.Wr/2)
        #self.pose_y = self.pose_y + self.dt*self.Vr*math.sin(self.pose_theta+self.dt*self.Wr/2)
        #self.pose_theta = (self.pose_theta + self.dt*self.Wr)
        
        msg_cmdR = Float32()
        msg_cmdL = Float32()
        
        msg_cmdR.data = 0.0
        msg_cmdL.data = 0.0
        
        if self.pose_theta < self.target_theta:
            msg_cmdR.data = 3.1415
            msg_cmdL.data = -3.1415

                  
        self.pub_cmdR.publish(msg_cmdR)
        self.pub_cmdL.publish(msg_cmdL)
        
        print(self.pose_x,"  ",self.pose_y,"  ",self.pose_theta)
        


def main(args=None):
    rclpy.init(args=args)

    velocity_control = VelocityControl()

    rclpy.spin(velocity_control)

    velocity_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

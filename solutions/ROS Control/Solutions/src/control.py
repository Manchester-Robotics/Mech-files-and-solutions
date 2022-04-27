#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D,Twist

import numpy as np

class robot_controller:
    def __init__(self):
        self.wr = 0.0
        self.wl = 0.0
        
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        self.l = 0.19
        self.r = 0.05
        
        self.current_time = rospy.get_time()
        self.previous_time = rospy.get_time()

        self.wr_sub = rospy.Subscriber("/wr",Float32,self.wr_callback)
        self.wl_sub = rospy.Subscriber("/wl",Float32,self.wl_callback)

        self.pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)

        self.e_d = Float32()
        self.e_theta = Float32()

        self.e_d.data = 0.0
        self.e_theta.data = 0.0

        self.e_d_pub = rospy.Publisher("/e_d",Float32,queue_size=1)
        self.e_theta_pub = rospy.Publisher("/e_theta",Float32,queue_size=1)

        self.goal_x = 0.0
        self.goal_y = -1.0

        self.control = Twist()
        self.control.linear.x = 0.0
        self.control.linear.y = 0.0
        self.control.linear.z = 0.0
        self.control.angular.x = 0.0
        self.control.angular.y = 0.0
        self.control.angular.z = 0.0

        self.control_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        
        self.K_v = 0.2
        self.K_w = 0.4
        self.I_w = 0.1

        self.integral = 0.0

    def wr_callback(self,msg):
        self.wr = msg.data
    
    def wl_callback(self,msg):
        self.wl = msg.data

    def run(self):
        self.current_time = rospy.get_time()
        dt = (self.current_time - self.previous_time)
        self.previous_time = self.current_time

        self.pose.theta = self.wrap_to_Pi(self.pose.theta + dt * self.r * ((self.wr - self.wl) / self.l))
        self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
        self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)

        self.pose_pub.publish(self.pose)

        self.e_d.data = np.sqrt(np.power(self.goal_x - self.pose.x,2) + np.power(self.goal_y - self.pose.y,2))
        self.e_theta.data = self.wrap_to_Pi(np.arctan2(self.goal_y,self.goal_x) - self.pose.theta)

        self.e_d_pub.publish(self.e_d)
        self.e_theta_pub.publish(self.e_theta)

        self.control.linear.x = self.K_v * self.e_d.data
        self.integral += self.I_w * dt * self.e_theta.data
        self.control.angular.z = self.K_w * self.e_theta.data + self.integral

        if(self.e_d.data < 0.05):
            self.control.linear.x = 0
            self.control.angular.z = 0
        self.control_pub.publish(self.control)

    def stop(self):
        self.control.linear.x = 0
        self.control.angular.z = 0
        self.control_pub.publish(self.control)

    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi


if __name__ == "__main__":
    rospy.init_node("controller")
    controller = robot_controller()
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(controller.stop)
    
    try:
        while not rospy.is_shutdown():
            controller.run()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass


    

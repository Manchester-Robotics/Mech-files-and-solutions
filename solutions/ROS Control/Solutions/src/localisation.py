#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

import numpy as np

class robot_controller:
    def __init__(self):
        # Variables for the storage of wheel speeds
        self.wr = 0.0
        self.wl = 0.0

        # Pose2D variable for the storage of the robot position
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        # Robot Constants
        self.l = 0.19
        self.r = 0.05
        
        # Time values for the computation of dt
        self.current_time = rospy.get_time()
        self.previous_time = rospy.get_time()

        # ROS Susbcribers and Publishers
        self.wr_sub = rospy.Subscriber("/wr",Float32,self.wr_callback)
        self.wl_sub = rospy.Subscriber("/wl",Float32,self.wl_callback)

        self.pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)

    # Callbacks for wheel speeds
    def wr_callback(self,msg):
        self.wr = msg.data
    
    def wl_callback(self,msg):
        self.wl = msg.data

    # Main Loop function
    def run(self):
        # Compute time since last main loop
        self.current_time = rospy.get_time()
        dt = (self.current_time - self.previous_time)
        self.previous_time = self.current_time

        # Update pose using equations from slides
        self.pose.theta = self.wrap_to_Pi(self.pose.theta + dt * self.r * ((self.wr - self.wl) / self.l))
        self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
        self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)

        # Publish pose
        self.pose_pub.publish(self.pose)

    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2*np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi

        
if __name__ == "__main__":
    #Initialse node and controller instances
    rospy.init_node("controller")
    controller = robot_controller()
    #run controller every 100 ms
    loop_rate = rospy.Rate(10)
    
    #Run main loop, handling exceptions if required
    try:
        while not rospy.is_shutdown():
            controller.run()
            loop_rate.sleep()
    except rospy.ROSInterruptException:
        pass


    

#!/usr/bin/env python

# ROS Tools:
import rospy
from std_msgs.msg import Float32

# Tools:
import pygame

class PSControl:

    def __init__ (self):

        # Init PyGame & JoySticks
        pygame.init()
        pygame.joystick.init()

        # If the number of Joysticks connected is superior to 1
        # block the system from running as a security measure.
        if pygame.joystick.get_count() > 1:
            rospy.logerr("PS Controller: More than one controller connected, please use only one of them.")
            return 0

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.axes = self.joystick.get_numaxes()
        self.states = [1] * self.axes

        # For each axis:
        for i in range(self.axes):

            # ROS Publisher
            pub = rospy.Publisher("/command/{}".format(i), Float32, queue_size = 1)
            self.states[i] = {"pub": pub, "state": 0}


    def read_axes (self):

        pygame.event.get()
        
        for i in range(self.axes):        
            axis = self.joystick.get_axis(i)
            self.states[i]["state"] = axis


    def run (self):

        self.read_axes()
        self.publish_commands()


    def publish_commands(self):

        for i in range(self.axes):
            self.states[i]["pub"].publish(self.states[i]["state"])


if __name__ == "__main__":
    
    # Init Node
    rospy.init_node("ps_control")

    # Init Publisher Class
    ps_control = PSControl()
    
    while not rospy.is_shutdown():
        ps_control.run()

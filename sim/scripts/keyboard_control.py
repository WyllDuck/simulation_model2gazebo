#!/usr/bin/env python

# ROS Tools:
import rospy
from std_msgs.msg import Float32

# Tools:
import pygame

class KeyboardControl:

    def __init__ (self):

        # Parameters
        self.change_rate = 0.05
        self.saturation = 1
        
        self.key_strokes = dict()

        # Publishers
        pub = rospy.Publisher("/command/2", Float32, queue_size = 1)
        self.key_strokes["thrust"] = {"pub": pub, "down": [False, False], "up": [True, True], "cmd": [0, 0], "key_codes": [pygame.K_LSHIFT, pygame.K_LCTRL]}

        pub = rospy.Publisher("/command/1", Float32, queue_size = 1)        
        self.key_strokes["roll"] = {"pub": pub, "down": [False, False], "up": [True, True], "cmd": [0, 0], "key_codes": [pygame.K_d, pygame.K_a]}

        pub = rospy.Publisher("/command/6", Float32, queue_size = 1)
        self.key_strokes["pitch"] = {"pub": pub, "down": [False, False], "up": [True, True], "cmd": [0, 0], "key_codes": [pygame.K_w, pygame.K_s]}

        pub = rospy.Publisher("/command/4", Float32, queue_size = 1)
        self.key_strokes["yaw"] = {"pub": pub, "down": [False, False], "up": [True, True], "cmd": [0, 0], "key_codes": [pygame.K_q, pygame.K_e]}


    def read_keyboard (self):

        # Update Key States (UP | DOWN)
        for event in pygame.event.get():

            if event.type == pygame.KEYDOWN:
                for key in self.key_strokes.values():
                    for i in range(2):
                        if key["key_codes"][i] == event.key:
                            key["down"][i] = True
                            key["up"][i] = False

            elif event.type == pygame.KEYUP:
                for key in self.key_strokes.values():
                    for i in range(2):
                        if key["key_codes"][i] == event.key:
                            key["down"][i] = False
                            key["up"][i] = True

        # Update Command Values
        for key in self.key_strokes.values():

            for i in range(2):
                if key["down"][i]:
                    if key["cmd"][i] < abs(self.saturation - self.change_rate):
                        key["cmd"][i] += self.change_rate
                    else:
                        key["cmd"][i] = self.saturation

                elif key["up"][i]:
                    key["cmd"][i] = 0


    def run (self):

        self.read_keyboard()
        self.publish_commands()


    def publish_commands(self):

        for key in self.key_strokes.values():
            key["pub"].publish(key["cmd"][0] - key["cmd"][1])


if __name__ == "__main__":
    
    # Init Node
    rospy.init_node("keyboard_control")

    # Init PyGame
    pygame.init()
    screen = pygame.display.set_mode((300, 300))

    # Init Publisher Class
    keyboard_control = KeyboardControl()
    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():

        keyboard_control.run()
        rate.sleep()
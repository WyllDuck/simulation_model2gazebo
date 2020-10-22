#!/usr/bin/env python

# ROS Tools:
import rospy
from std_msgs.msg import Float32

# Tools:
import pygame

class KeyboardControl:

    def __init__ (self):

        # Parameters
        self.change_rate = 0.08
        self.saturation = 1
        
        self.key_strokes = dict()

        # Publishers
        # Arrows Set Keyboard (+ rePage & avPage)
        pub = rospy.Publisher("/command/1", Float32, queue_size = 1)
        self.key_strokes["UP"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_UP}

        pub = rospy.Publisher("/command/2", Float32, queue_size = 1)        
        self.key_strokes["DOWN"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_DOWN}

        pub = rospy.Publisher("/command/3", Float32, queue_size = 1)
        self.key_strokes["LEFT"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_LEFT}
        
        pub = rospy.Publisher("/command/4", Float32, queue_size = 1)
        self.key_strokes["RIGHT"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_RIGHT}
        
        pub = rospy.Publisher("/command/5", Float32, queue_size = 1)
        self.key_strokes["REPAG"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_PAGEUP}

        pub = rospy.Publisher("/command/6", Float32, queue_size = 1)
        self.key_strokes["AVPAG"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_PAGEDOWN}

        # Letters Set Keyboard (ASDW + EQ)
        pub = rospy.Publisher("/command/7", Float32, queue_size = 1)
        self.key_strokes["A"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_a}

        pub = rospy.Publisher("/command/8", Float32, queue_size = 1)
        self.key_strokes["S"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_s}

        pub = rospy.Publisher("/command/9", Float32, queue_size = 1)
        self.key_strokes["D"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_d}

        pub = rospy.Publisher("/command/10", Float32, queue_size = 1)
        self.key_strokes["W"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_w}

        pub = rospy.Publisher("/command/11", Float32, queue_size = 1)
        self.key_strokes["E"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_e}

        pub = rospy.Publisher("/command/12", Float32, queue_size = 1)
        self.key_strokes["Q"] = {"pub": pub, "down": False, "up": True, "cmd": 0, "key_code": pygame.K_q}

        
    def read_keyboard (self):

        # Update Key States (UP | DOWN)
        for event in pygame.event.get():

            if event.type == pygame.KEYDOWN:
                for key in self.key_strokes.values():
                    if key["key_code"] == event.key:
                        key["down"] = True
                        key["up"] = False

            elif event.type == pygame.KEYUP:
                for key in self.key_strokes.values():
                    if key["key_code"] == event.key:
                        key["down"] = False
                        key["up"] = True

        # Update Command Values
        for key in self.key_strokes.values():

            if key["down"]:
                if key["cmd"] < abs(self.saturation - self.change_rate):
                    key["cmd"] += self.change_rate
                else:
                    key["cmd"] = self.saturation

            elif key["up"]:
                key["cmd"] = 0


    def run (self):

        self.read_keyboard()
        self.publish_commands()


    def publish_commands(self):

        for key in self.key_strokes.values():
            key["pub"].publish(key["cmd"])


if __name__ == "__main__":
    
    # Init Node
    rospy.init_node("keyboard_control")

    # Init PyGame
    pygame.init()
    screen = pygame.display.set_mode((300, 300))

    # Init Publisher Class
    keyboard_control = KeyboardControl()
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():

        keyboard_control.run()
        rate.sleep()
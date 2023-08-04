#!/usr/bin/env python

# launch this script with sudo to control joystick's leds!

import subprocess
import time
import os
import signal
import io
#import pygame

import rospy
from sensor_msgs.msg import Joy
from ur5_info.msg import MoveUR5WithGripper, JointPositions
from gripper_move.msg import GripperAngle


class JoyStickLeds:
    def __init__(self):
        self.red_file = io.FileIO("/sys/class/leds/0005:054C:09CC.0001:red/brightness", "w")
        self.green_file = io.FileIO("/sys/class/leds/0005:054C:09CC.0001:green/brightness", "w")
        self.blue_file = io.FileIO("/sys/class/leds/0005:054C:09CC.0001:blue/brightness", "w")

    def to_bytes(self, value):
        value = max(min(int(value), 255), 0)
        value = bytes(value)
        return value

    def set_red(self, value):
        self.red_file.write(self.to_bytes(value))

    def set_green(self, value):
        self.green_file.write(self.to_bytes(value))

    def set_blue(self, value):
        self.blue_file.write(self.to_bytes(value))

    def set_color(self, r, g, b):
        self.set_red(r)
        self.set_green(g)
        self.set_blue(b)


class ManipulatorHandler:
    def __init__(self):
        self.joint_state_publisher = rospy.Publisher('/move_robot_delay_gripper', MoveUR5WithGripper, latch=True, queue_size=100)
        self.gripper_angle_publisher = rospy.Publisher('/gripper_angle', GripperAngle, latch=True, queue_size=100)
        self.raised = False
        self.folded = False
        self.grasped = False
        # create joystickleds object
        self.joystickleds = JoyStickLeds()
        # sound notification init
        # pygame.mixer.init()
        # self.record_sound = pygame.mixer.Sound("sounds/record.mp3")
        # self.stop_sound = pygame.mixer.Sound("sounds/stop.mp3")

    def raise_hand(self):
        if self.raised:
            return
        self.raised = True
        self.folded = False
        print('Raise arm')
        msg = MoveUR5WithGripper()
        msg.positions.append(JointPositions())
        msg.positions[0].position = [1.594929575920105, -1.9816530386554163, 1.6976728439331055, 
        -2.832118336354391, -1.5817907492267054, 0.0]
        msg.delay = [0, 0, 0, 0, 0, 20]
        msg.gripperAngle = 0.
        self.joystickleds.set_color(0, 255, 0) # green
        self.joint_state_publisher.publish(msg)
        
    def fold(self):
        if self.folded:
            return
        self.folded = True
        self.raised = False
        print('Fold arm')
        msg = MoveUR5WithGripper()
        msg.positions.append(JointPositions())
        msg.positions[0].position = [1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]
        msg.delay = [0, 0, 0, 0, 0, 20]
        msg.gripperAngle = 0.
        self.joystickleds.set_color(0, 0, 64) # blue
        self.joint_state_publisher.publish(msg)

    def grasp(self):
        if self.grasped:
            return
        self.grasped = True
        print('Grasp')
        msg = GripperAngle()
        msg.angle = 0.05
        self.joystickleds.set_color(255, 255, 0) # yellow
        self.gripper_angle_publisher.publish(msg)

    def ungrasp(self):
        if not self.grasped:
            return
        self.grasped = False
        print('Ungrasp')
        msg = GripperAngle()
        msg.angle = 0.085
        self.joystickleds.set_color(0, 0, 64) # blue
        self.gripper_angle_publisher.publish(msg)
        
    def health_check(self, event):
        if self.process is not None:
            exit_code = self.process.poll()
            if self.recording and exit_code is not None:
                print "Health check failure, exit code", exit_code
                self.recording = False
                self.joystickleds.set_color(255, 0, 0)
                print "Stopped"


def joy_callback(msg):
    global manipulator_handler
    share_button = msg.buttons[8]
    options_button = msg.buttons[9]
    x_button = msg.buttons[0]
    triangle_button = msg.buttons[2]
    if share_button:
        manipulator_handler.grasp()
    elif options_button:
        manipulator_handler.ungrasp()
    elif x_button:
        manipulator_handler.raise_hand()
    elif triangle_button:
        manipulator_handler.fold()


def reset_color(joystickleds):
    joystickleds.set_color(0, 0, 64)


if __name__ == "__main__":
    manipulator_handler = ManipulatorHandler()
    rospy.init_node("manipulator_handler", anonymous=True)
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_callback)
    #rospy.Timer(rospy.Duration(1), manipulator_handler.health_check)
    rospy.on_shutdown(lambda: reset_color(manipulator_handler.joystickleds))
    rospy.spin()


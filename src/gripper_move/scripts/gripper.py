#!/usr/bin/python
import rospy
import actionlib
import roslib; roslib.load_manifest('ur_driver')
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
import time

from gripper_move.srv import *
from gripper_move.msg import *


class Gripper(object):
    def __init__(self, angle = 0, speed = 0.10): # speed = 0.10 = 5 sm/sec
        # params
        self.pos_goal = angle
        self.pos_max = 0.085
        self.pos_min = 0.0
        self.speed = speed
        self.force = 0          # 25 Nytons (minimal force)

        self.rate = rospy.Rate(15)

        if self.pos_goal < self.pos_min:
            self.pos_goal = self.pos_min
        if self.pos_goal > self.pos_max:
            self.pos_goal = self.pos_max

        # client
        self.robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
        self.robotiq_client.wait_for_server()
        rospy.loginfo("Connected to the gripper server")

        self.connect_pub = rospy.Publisher('/get_gripper_state', GripperAngle, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.sleep(1)

    def move(self):
        print("Open start,  angle = %f, min_angle = %f, max_angle = %f, speed = %f", self.pos_goal, self.pos_min, self.pos_max, self.speed)
        print("----------")
        print(self.pos_goal)
        Robotiq.goto(self.robotiq_client, pos=self.pos_goal, speed=self.speed, force=self.force, block=False)
        print("Open finish")

    def state(self):
        r = Robotiq()
        return r.get_current_gripper_status().requested_position

    def spin(self):
        while not rospy.is_shutdown():
            # msg = GripperAngle()
            # msg.angle = Robotiq.get_current_joint_position()
            # self.connect_pub.publish(msg)
            self.rate.sleep()


def handle_gripper_move_srv(req):
    global gripper
    gripper = Gripper(req.angle, req.speed)
    gripper.move()
    return GripperMoveRobotResponse(True)

def handle_gripper_move_sub(req):
    print('============')
    print(req.name)
    print(req.angle)

    global gripper
    gripper = Gripper(req.angle)
    gripper.move()

def handle_gripper_state_srv(req):
    global gripper
    gripper = Gripper()
    return GetGripperStateResponse(gripper.state)

def handle_gripper_move_angle_sub(req):
    global gripper
    gripper = Gripper(req.angle)
    gripper.move()


def get_gripper_state(req):
    if len(req.position) != 1:
        return
    
    gripper_state_pub = rospy.Publisher('/gripper_state_robot', GripperInfo, queue_size=10)
    msg = GripperInfo()
    msg.id = 0
    msg.name = "robotiq"
    msg.angle = req.position[0]
    gripper_state_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("gripper_controller")
    s = rospy.Service('gripper_move_robot', GripperMoveRobot, handle_gripper_move_srv)
    state = rospy.Service('gripper_state_robot', GetGripperState, handle_gripper_state_srv)
    rospy.Subscriber("gripper_state", GripperInfo, handle_gripper_move_sub) # get message from UI
    rospy.Subscriber("gripper_angle", GripperAngle, handle_gripper_move_angle_sub)
    rospy.Subscriber("/joint_states", JointState, get_gripper_state)

    gripper_state = Gripper()
    gripper_state.spin()

    rospy.spin()

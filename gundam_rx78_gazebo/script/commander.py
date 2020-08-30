#!/usr/bin/env python
#coding:utf-8

import rospy
import time
import numpy as np
from math import *
from std_msgs.msg import String, Float32, Float32MultiArray
from settings import GAIT_COMMANDER_CIPHER


#######################################cvs##########################################

import actionlib

import argparse
import sys
import csv

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
#######################################cvs##########################################
def main(filename):
    print("Initializing node... ")
    #rospy.init_node("joint_trajectory_client_csv_example")
    rospy.sleep(1)
    print("Running. Ctrl-c to quit")

    # init goal
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1)

    with open(filename) as f:
        reader = csv.reader(f, skipinitialspace=True)
        first_row = True
        for row in reader:
            if first_row:
                goal.trajectory.joint_names = row[1:]
                first_row = False
            else:
                point = JointTrajectoryPoint()
                point.positions = [float(n) for n in row[1:]]
                point.time_from_start = rospy.Duration(float(row[0]))
                goal.trajectory.points.append(point)
    client = actionlib.SimpleActionClient(
        '/fullbody_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
    )

    if not client.wait_for_server(timeout=rospy.Duration(10)):
        rospy.logerr("Timed out waiting for Action Server")
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)

    # send goal
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    print("waiting...")
    if not client.wait_for_result(timeout=rospy.Duration(60)):
        rospy.logerr("Timed out waiting for JTA")
    rospy.loginfo("Exitting...")

#######################################cvs##########################################


def brazo():
    print("Initializing node... ")
    #rospy.init_node("joint_trajectory_client_example")
    rospy.sleep(1)
    print("Running. Ctrl-c to quit")
    positions = {
        'larm_shoulder_p':-1.55,
        'larm_shoulder_r':1.55,
        'larm_elbow_p':-1.0,
        'larm_gripper':0.0,
    }
    client = actionlib.SimpleActionClient(
        '/fullbody_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
    )

    if not client.wait_for_server(timeout=rospy.Duration(10)):
        rospy.logerr("Timed out waiting for Action Server")
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)

    # init goal
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1)
    goal.trajectory.joint_names = positions.keys()

    # points
    point = JointTrajectoryPoint()
    goal.trajectory.joint_names = positions.keys()
    point.positions = positions.values()
    point.time_from_start = rospy.Duration(10)
    goal.trajectory.points.append(point)

    

    # send goal
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    print(goal)
    print("waiting...")
    if not client.wait_for_result(timeout=rospy.Duration(20)):
        rospy.logerr("Timed out waiting for JTA")
    rospy.loginfo("Exitting...")

class Commander():
    def __init__(self):
        time.sleep(2.0)
        rospy.init_node("commander","joint_trajectory_client_csv_example","joint_trajectory_client_example")
        self.command_pubber = rospy.Publisher("state_change", String, queue_size = 10)
        rospy.Subscriber("imu_data", Float32MultiArray, self.imu_callback)
        rospy.Subscriber("dune_distance", Float32, self.dune_distance_callback)
        self.imu_data = [0., 0., 0.]
        self.dune_distance = 0.

    def main_loop(self):
        while not rospy.is_shutdown():
            print("Commander:")
            print("w\t: Adelante")
            print("s\t: Atras")
            print("a\t: Izquierda")
            print("d\t: Derecha")
            print("st\t: Stop")
            print("br\t: Brazo")
            print("q\t: exit")
            command = raw_input()
            if command == "w":
                #adelante
                
                #args = parser.parse_args('`rospack find gundam_rx78_control`/sample/csv/turn-left.csv'[1:])
                main("/home/david/catkin_ws_apps/src/gundam_robot/gundam_rx78_control/sample/csv/walk-forward.csv")
            elif command == "s":
                #atras
                main("/home/david/catkin_ws_apps/src/gundam_robot/gundam_rx78_control/sample/csv/walk-backward.csv")
            elif command == "a":
                #izquierda
                main("/home/david/catkin_ws_apps/src/gundam_robot/gundam_rx78_control/sample/csv/turn-left.csv")
            elif command == "d":
                #derecha
                main("/home/david/catkin_ws_apps/src/gundam_robot/gundam_rx78_control/sample/csv/turn-right.csv")
            elif command == "st":
                #stop
                main("/home/david/catkin_ws_apps/src/gundam_robot/gundam_rx78_control/sample/csv/step.csv")
            elif command == "br":
                #Brazo
                brazo()
            elif command == 'q':
                exit()

    def Pub_command(self, command, T):
        self.command_pubber.publish(GAIT_COMMANDER_CIPHER[command])
        time.sleep(T)

    def imu_callback(self, msg):
        self.imu_data = np.array(msg.data)/pi*180

    def dune_distance_callback(self, msg):
        self.dune_distance = msg.data


if __name__ == "__main__":
    
    mCommander = Commander()
    mCommander.main_loop()

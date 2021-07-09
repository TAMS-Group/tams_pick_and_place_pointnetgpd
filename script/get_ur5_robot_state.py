#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang
# E-mail     : liang@informatik.uni-hamburg.de
# Description:
# Date       : 08/09/2018 12:00 AM
# File Name  : get_ur5_robot_state.py
from __future__ import print_function, division
import rospy
import numpy as np
import moveit_commander


def get_robot_state_moveit():
    try:
        current_joint_values = np.array(group.get_current_joint_values())
        diff = abs(current_joint_values - home_joint_values) * 180 / np.pi
        if np.sum(diff < 2) == 6:  # if current joint - home position < 2 degree, we think it is at home
            return 1  # robot at home
        else:
            return 2  # robot is moving
    except:
        print("Get robot state failed", end="\r")
        return 3  # robot state unknown


if __name__ == "__main__":
    rospy.init_node("ur5_state_checker_if_it_at_home", anonymous=True)
    rate = rospy.Rate(10)
    group = moveit_commander.MoveGroupCommander("arm")
    home_joint_values = np.array([0, -1.5708, 0, -1.5708, 0, 0])
    while not rospy.is_shutdown():
        at_home = get_robot_state_moveit()
        if at_home == 1:
            rospy.set_param("/robot_at_home", True)
            print("Robot is at home position", end="\r")
        elif at_home == 2:
            rospy.set_param("/robot_at_home", False)
            print("Robot is not at home position", end="\r")
        elif at_home == 3:
            rospy.set_param("/robot_at_home", False)
            print("Get robot state failed", end="\r")
        rate.sleep()

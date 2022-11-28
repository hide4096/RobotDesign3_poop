#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import actionlib
import geometry_msgs.msg
import rosnode
from tf import transformations
from tf.transformations import quaternion_from_euler
import math
from geometry_msgs.msg import Point,Pose
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    rospy.sleep(1.0)

    while True:
        gripper_goal.command.position = 1.2
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

        arm.set_named_target("home")
        arm.go()

        rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node("Move_arm_example")
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import traceback
import moveit_commander
import actionlib
import geometry_msgs.msg
import rosnode
import tf
from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
from geometry_msgs.msg import Point,Pose
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

def yaw_of(object_orientation):
    eular = euler_from_quaternion(
        (object_orientation.x,object_orientation.y,object_orientation.z,
        object_orientation.w)
    )

    return eular[2]

def deg2rad(degree):
    return degree * (math.pi/180.0)




def sweep(arm,tf_listener,ar_pos,ar_rot,marker_name):
    for i in range(0,180,45):

        target_pose = Pose()
        target_pose.position.x = 0.1 * math.sin(deg2rad(i))
        target_pose.position.y = -0.1 * math.cos(deg2rad(i))
        target_pose.position.z = 0.2

        q = quaternion_from_euler(deg2rad(140),0.0,deg2rad(i))
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        arm.set_pose_target(target_pose)

        if arm.go() is False:
            print("Failed")
            while True:
                rospy.sleep(1.0)
            
        rospy.sleep(0.1)

        for name in marker_name:
            try:
                (ar_pos[name],ar_rot[name]) = tf_listener.lookupTransform('/base_link',name,rospy.Time(0))
                print(name)
            except:
                #traceback.print_exc()
                continue

        rospy.sleep(0.5)
    
    return

def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    tf_listener = tf.TransformListener()

    # マーカー名生成
    marker_name = []
    for i in range(0,7):
        marker_name.append("/ar_marker_" + str(i))

    rospy.sleep(1.0)

    while True:
        # ハンドを開く
        gripper_goal.command.position = 1.2
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

        # ホーム姿勢にする
        arm.set_named_target("home")
        arm.go()
        rospy.sleep(1.0)

        print("Start")

        ar_pos = {}
        ar_rot = {}

        sweep(arm,tf_listener,ar_pos,ar_rot,marker_name)
"""
        try:
            target_pose = Pose()
            target_pose.position.x = ar_pos[0]
            target_pose.position.y = ar_pos[1]
            target_pose.position.z = ar_pos[2]+1.0
            ar_yaw = euler_from_quaternion((ar_rot[0],ar_rot[1],ar_rot[2],ar_rot[3]))[2]
            q = quaternion_from_euler(-math.pi,0.0,ar_yaw)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

            arm.set_pose_target(target_pose)
            arm.go()
            rospy.sleep(1.0)
            gripper_goal.command.position = 0.1
            gripper.send_goal(gripper_goal)
            gripper.wait_for_result(rospy.Duration(1.0))
            rospy.sleep(2.0)

        except:
            import traceback
            traceback.print_exc()
            continue
"""

if __name__ == '__main__':
    rospy.init_node("Move_arm_example")
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
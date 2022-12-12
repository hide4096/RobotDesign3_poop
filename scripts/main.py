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
import hanoi
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
    for i in range(90,180,45):

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

def move(arm,gripper,gripper_goal,tf_listener,ar_pos,ar_rot,marker_name,frm,to):

    target_pose = Pose()
    target_pose.position.x = ar_pos[frm][0]
    target_pose.position.y = ar_pos[frm][1]
    target_pose.position.z = ar_pos[frm][2]+1.0
    ar_yaw = euler_from_quaternion((ar_rot[frm][0],ar_rot[frm][1],ar_rot[frm][2],ar_rot[frm][3]))[2]
    q = quaternion_from_euler(-math.pi,0.0,ar_yaw)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    arm.set_pose_target(target_pose)

    if arm.go() is False:
        print("Failed")
        while True:
            rospy.sleep(1.0)

    rospy.sleep(1.0)

    gripper_goal.command.position = 0.1
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    rospy.sleep(1.0)

    target_pose = Pose()
    target_pose.position.x = ar_pos[to][0]
    target_pose.position.y = ar_pos[to][1]
    target_pose.position.z = ar_pos[to][2]+1.0
    ar_yaw = euler_from_quaternion((ar_rot[to][0],ar_rot[to][1],ar_rot[to][2],ar_rot[to][3]))[2]
    q = quaternion_from_euler(-math.pi,0.0,ar_yaw)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    arm.set_pose_target(target_pose)

    if arm.go() is False:
        print("Failed")
        while True:
            rospy.sleep(1.0)

    rospy.sleep(1.0)

    gripper_goal.command.position = 1.2
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    rospy.sleep(1.0)

def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(0.1)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    tf_listener = tf.TransformListener()

    # マーカー名生成
    marker_name = []
    for i in range(0,7):
        marker_name.append("/ar_marker_" + str(i))
    
    marker_ground = [marker_name[0],marker_name[1],marker_name[2]]
    marker_poop = [marker_name[3],marker_name[4],marker_name[5]]


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

        mn = marker_name[6]
        if mn in ar_pos:

            target_pose = Pose()
            target_pose.position.x = ar_pos[mn][0]
            target_pose.position.y = ar_pos[mn][1]
            target_pose.position.z = ar_pos[mn][2]+0.09
            ar_yaw = euler_from_quaternion((ar_rot[mn][0],ar_rot[mn][1],ar_rot[mn][2],ar_rot[mn][3]))[2]
            q = quaternion_from_euler(-math.pi,0.0,ar_yaw)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

            arm.set_pose_target(target_pose)
            if arm.go() is False:
                print("Failed")
                continue

            rospy.sleep(1.0)
            gripper_goal.command.position = 0.1
            gripper.send_goal(gripper_goal)
            gripper.wait_for_result(rospy.Duration(1.0))
            rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node("Move_arm_example")
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
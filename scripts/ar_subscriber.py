#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

class ARPoseSubscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_alvmarker)

    def callback_alvmarker(self, markers):

        for m in markers.markers:
            marker_id = m.id
            if marker_id == 1:
                marker_pose = m.pose.pose
                pos = marker_pose.position
                ori = marker_pose.orientation
                rospy.loginfo("marker[%d] position (x,y,z) = (%3.3f: %3.3f: %3.3f), orientation (x,y,z,w) = (%3.3f: %3.3f: %3.3f: %3.3f)" 
                              % (marker_id, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w))

        return

if __name__ == '__main__':
    rospy.init_node('ar_pose_subscriber')
    ARPoseSubscriber()

    rospy.spin()

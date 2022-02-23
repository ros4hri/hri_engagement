#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import absolute_import
import rospy
from geometry_msgs.msg import PointStamped
from hri_msgs.msg import IdsList


from hri_msgs.msg import Expression
from expressive_eyes.face_manager import FaceManager
from expressive_eyes import expressions as exp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import actionlib
import hri_msgs.msg
import hri_actions_msgs.msg
import tf


class EngagementNode(object):
    def __init__(self):
        self.br = CvBridge()
        """ Initialize the goal publisher node."""
        self.look_at_pub = rospy.Publisher('/look_at', PointStamped,
                                          queue_size=10)


        self.faces_list_sub = rospy.Subscriber("/humans/faces/tracked", IdsList, self.face_list_cb)
        self.listener = tf.TransformListener()

        self.face_detected = []

        self.loop_rate = rospy.Rate(30)



    def face_list_cb(self, msg):
        rospy.logdebug("Message: %s" % msg)
        
        pass


    def check_for_egagement(self):


        #Pose
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/gaze"+self.active_id
        point.point.x = 0
        point.point.y = 0
        point.point.z = 0

        self.look_at_pub.publish(point)

    def run(self):

        elapsed_time = rospy.Duration()

        while not rospy.is_shutdown():
            begin_time = rospy.Time.now()
            #execute our code here

            self.loop_rate.sleep()

            elapsed_time = rospy.Time.now() - begin_time




if __name__ == "__main__":
    rospy.init_node("expressive_eyes_node")
    node = EngagementNode()
    node.run()

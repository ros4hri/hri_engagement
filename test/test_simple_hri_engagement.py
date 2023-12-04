#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from hri_engagement.engagement_node import BUFFER_DURATION, REFERENCE_FRAME, NODE_RATE, VISUAL_SOCIAL_ENGAGEMENT_THR, EngagementNode
import os
import json
import tf2_ros
import rospy
import unittest
import rosbag
from hri_msgs.msg import EngagementLevel
import geometry_msgs.msg
import sys
import pyhri

sys.path.insert(0, os.path.abspath(os.path.join
                                   (os.path.dirname(__file__), '..')))

PKG = 'test_hri_engagement'


class TestHRIEngagement(unittest.TestCase):

    def setUp(self):
        self.directory = os.path.dirname(
            os.path.realpath(__file__)) + "/simple_bags/"
        self.expected_result = None
        self.engagement_value = EngagementLevel.UNKNOWN
        self.person_tracked = dict()
        self.person_id = dict()
        self.face_id = dict()
        self.reference_frame = "camera_link"
        REFERENCE_FRAME = self.reference_frame
        engagement_history_size = NODE_RATE * BUFFER_DURATION
        self.frame_to_skip = engagement_history_size + 10
        self.visual_social_engagement_thr = VISUAL_SOCIAL_ENGAGEMENT_THR
        self.rosbag_files = list()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # static tf
        t_camera = geometry_msgs.msg.TransformStamped()
        t_camera.header.stamp = rospy.Time.now()
        t_camera.header.frame_id = "camera_link"
        t_camera.child_frame_id = "camera_color_optical_frame"
        t_camera.transform.translation.x = 0
        t_camera.transform.translation.y = 0
        t_camera.transform.translation.z = 0
        t_camera.transform.rotation.x = -0.5
        t_camera.transform.rotation.y = 0.5
        t_camera.transform.rotation.z = -0.5
        t_camera.transform.rotation.w = 0.5
        self.publish_static_tf(t_camera.header.frame_id,
                               t_camera.child_frame_id, t_camera)

    def tearDown(self):
        self.tfBuffer = None
        self.listener = None

    @staticmethod
    def publish_static_tf(source_frame, target_frame, transform):
        """
        method that publishes the tf from reference_frame to child_frame
        """
        br = tf2_ros.StaticTransformBroadcaster()
        print(f'static transformation from {source_frame} to {target_frame}')
        br.sendTransform(transform)

    @staticmethod
    def publish_tf(source_frame, target_frame, transform):
        """
        it publishes the transform from the source_frame to target_frame
        """
        br = tf2_ros.TransformBroadcaster()
        print(f' transformation from {source_frame} to {target_frame}')
        br.sendTransform(transform)

    def test_engagement_one_person(self):
        """
        it tests the engagement of one person
        """
        f = open(self.directory + 'list_of_bags.json')
        data = json.load(f)
        engaged_data = data['single_person']['engaged']

        # extract person_id, rosbag_name, and expected result
        for person in engaged_data:
            if person == "person_id":
                self.persons_id = engaged_data[person]
            if person == "rosbag":
                self.rosbag_files = engaged_data[person]
            if person == 'expected_result':
                self.expected_result = \
                    [(int(engaged_data[person]))][0]

        assert len(self.persons_id) == len(self.rosbag_files), \
            f'n_persons need to be equal to n_rosbag_files'

        person_instance = pyhri.Person("",
                                       self.tfBuffer,
                                       self.reference_frame)
        face_instance = pyhri.Face("",
                                   self.tfBuffer,
                                   self.reference_frame)

        face_id = ""
        tf_rcamera_to_hface = None
        tf_hface_to_hgaze = None

        # run the unittest for one rosbag
        for i in range(len(self.rosbag_files)):
            engagement_node = \
                EngagementNode(self.visual_social_engagement_thr)
            person_instance.id = (self.persons_id[i])
            rospy.loginfo(f'Processing file ...'
                          f'{self.directory + self.rosbag_files[i]}')
            bag = rosbag.Bag(self.directory + self.rosbag_files[i],
                             'r', allow_unindexed=True)
            iterator = bag.read_messages(topics=["/humans/persons/tracked",
                                                 "/humans/persons/"
                                                 + self.persons_id[i] +
                                                 "/face_id", "/tf",
                                                 "/tf_static"])
            self.tf_rcamera_to_hface = None
            self.tf_hface_to_hgaze = None

            # we need to skip the first NODE_RATE * BUFFER_DURATION frames as
            # the algorithm is still buffering plus the addional 4 frame to give it
            # the possibility to swich from UNKOWN to ENGAGED.
            frame_counter = 0

            for (topic, msg, t) in iterator:
                # get the tracked persons from the topic
                if topic == "/humans/persons/tracked":
                    self.person_tracked = msg
                    for id in self.person_tracked.ids:
                        engagement_node.hri_listener._tracked_persons[id] = \
                            person_instance

                # for each tracked person get the face_id
                # (we need that to compute the engagement)
                if topic == "/humans/persons/" + self.persons_id[i] \
                        + "/face_id":
                    face_id = msg
                    face_instance.id = face_id.data
                    face_instance.frame = "face_" + face_id.data
                    face_instance.gaze_frame = "gaze_" + face_id.data
                    person_instance.face = face_instance

                # get the tfs and republish them
                if topic == "/tf":
                    if msg.transforms[0].header.frame_id == \
                            "camera_color_optical_frame" and \
                            msg.transforms[0].child_frame_id == "face_" \
                            + face_id.data:
                        # compute the transformation
                        tf_rcamera_to_hface = msg.transforms[0]
                    if msg.transforms[0].header.frame_id == "face_" \
                            + face_id.data and \
                            msg.transforms[0].child_frame_id == "gaze_" \
                            + face_id.data:
                        # compute the transformation
                        tf_hface_to_hgaze = msg.transforms[0]

                    if tf_rcamera_to_hface is not None and \
                            tf_hface_to_hgaze is not None:
                        tf_rcamera_to_hface.header.stamp = rospy.Time.now()
                        self.publish_tf(tf_rcamera_to_hface.header.frame_id,
                                        tf_rcamera_to_hface.child_frame_id,
                                        tf_rcamera_to_hface)
                        tf_hface_to_hgaze.header.stamp = rospy.Time.now()
                        self.publish_tf(tf_hface_to_hgaze.header.frame_id,
                                        tf_hface_to_hgaze.child_frame_id,
                                        tf_hface_to_hgaze)

                        rospy.sleep(rospy.Duration(
                            nsecs=(1e9/script.engagement_node.NODE_RATE)))

                        try:
                            engagement_node.get_tracked_humans()
                            if frame_counter < self.frame_to_skip and engagement_node.tracked_persons_in_the_scene:
                                frame_counter += 1
                            else:
                                self.assertEqual(self.expected_result,
                                                 engagement_node.active_persons
                                                 [self.persons_id[i]].
                                                 person_current_engagement_level)

                        except (tf2_ros.LookupException,
                                tf2_ros.ConnectivityException,
                                tf2_ros.ExtrapolationException) as e:
                            rospy.logwarn(f'Exception in tf2 {e}')

            bag.close()

    def test_disengagement_one_person(self):
        """
        it tests the disengagement of one person
        """
        f = open(self.directory + 'list_of_bags.json')
        data = json.load(f)
        disengaged_data = data['single_person']['disengaged']

        # extract person_id, rosbag_name, and expected result
        for person in disengaged_data:
            if person == "person_id":
                self.persons_id = disengaged_data[person]
            if person == "rosbag":
                self.rosbag_files = disengaged_data[person]
            if person == 'expected_result':
                self.expected_result = \
                    [(int(disengaged_data[person]))][0]

        assert len(self.persons_id) == len(self.rosbag_files), \
            f'n_persons need to be equal to n_rosbag_files'

        person_instance = pyhri.Person("",
                                       self.tfBuffer,
                                       self.reference_frame)
        face_instance = pyhri.Face("",
                                   self.tfBuffer,
                                   self.reference_frame)

        face_id = ""
        tf_rcamera_to_hface = None
        tf_hface_to_hgaze = None

        # run the unittest for one rosbag
        for i in range(len(self.rosbag_files)):
            engagement_node = \
                EngagementNode(self.visual_social_engagement_thr)
            person_instance.id = (self.persons_id[i])
            rospy.loginfo(f'Processing file ...'
                          f'{self.directory + self.rosbag_files[i]}')
            bag = rosbag.Bag(self.directory + self.rosbag_files[i],
                             'r', allow_unindexed=True)
            iterator = bag.read_messages(topics=["/humans/persons/tracked",
                                                 "/humans/persons/"
                                                 + self.persons_id[i] +
                                                 "/face_id", "/tf",
                                                 "/tf_static"])

            # we need to skip the first NODE_RATE * BUFFER_DURATION frames as
            # the algorithm is still buffering plus the addional 4 frame to give it
            # the possibility to swich from UNKOWN to ENGAGED.
            frame_counter = 0

            for (topic, msg, t) in iterator:
                # get the tracked persons from the topic
                if topic == "/humans/persons/tracked":
                    self.person_tracked = msg
                    for id in self.person_tracked.ids:
                        engagement_node.hri_listener._tracked_persons[id] = \
                            person_instance

                # for each tracked person get the face_id
                # (we need that to compute the engagement)
                if topic == "/humans/persons/" + self.persons_id[i] \
                        + "/face_id":
                    face_id = msg
                    face_instance.id = face_id.data
                    face_instance.frame = "face_" + face_id.data
                    face_instance.gaze_frame = "gaze_" + face_id.data
                    person_instance.face = face_instance

                # get the tfs and republish them
                if topic == "/tf":
                    if msg.transforms[0].header.frame_id == \
                            "camera_color_optical_frame" and \
                            msg.transforms[0].child_frame_id == "face_" \
                            + face_id.data:
                        # compute the transformation
                        tf_rcamera_to_hface = msg.transforms[0]
                    if msg.transforms[0].header.frame_id == "face_" \
                            + face_id.data and \
                            msg.transforms[0].child_frame_id == "gaze_" \
                            + face_id.data:
                        # compute the transformation
                        tf_hface_to_hgaze = msg.transforms[0]

                    if tf_rcamera_to_hface is not None and \
                            tf_hface_to_hgaze is not None:
                        tf_rcamera_to_hface.header.stamp = rospy.Time.now()
                        self.publish_tf(tf_rcamera_to_hface.header.frame_id,
                                        tf_rcamera_to_hface.child_frame_id,
                                        tf_rcamera_to_hface)
                        tf_hface_to_hgaze.header.stamp = rospy.Time.now()
                        self.publish_tf(tf_hface_to_hgaze.header.frame_id,
                                        tf_hface_to_hgaze.child_frame_id,
                                        tf_hface_to_hgaze)

                        try:
                            engagement_node.get_tracked_humans()
                            if frame_counter < self.frame_to_skip:
                                frame_counter += 1
                            else:
                                self.assertEqual(self.expected_result,
                                                 engagement_node.active_persons
                                                 [self.persons_id[i]].
                                                 person_current_engagement_level)

                        except (tf2_ros.LookupException,
                                tf2_ros.ConnectivityException,
                                tf2_ros.ExtrapolationException) as e:
                            rospy.logwarn(f'Exception in tf2 {e}')

            bag.close()


if __name__ == "__main__":
    import rostest

    rospy.init_node("hri_engagement_test")
    rostest.rosrun(PKG, "test_hri_engagement", TestHRIEngagement)

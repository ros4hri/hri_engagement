#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import absolute_import
import rospy
from hri_msgs.msg import IdsList
from hri_msgs.msg import EngagementLevel
from std_msgs.msg import String
import tf2_ros

import numpy as np
import math
import enum

# time window to store the engagement status of the person (secs)
BUFFER_DURATION = 2
# rate of the main node
NODE_RATE = 30


class EngagementStatus(enum.Enum):
    """
    Auxiliary class to print in a more intuitive way the engagement status
    """

    # unknown: no information is provided about the engagement level
    UNKNOWN = 0
    # disengaged: the person has not looked in the direction of the robot
    DISENGAGED = 1
    # engaging: the person has started to look in the direction of the robot
    ENGAGING = 2
    # engaged: the person is fully engaged with the robot
    ENGAGED = 3
    # disengaging: the person has started to look away from the robot
    DISENGAGING = 4


class Person(object):
    """
    Auxiliary class that given a Person identified with their person_id,
    it publishes their engagement status on the topic:
    /humans/persons/<human_id>/engagement_status.
    To compute the engagement, their gaze direction is estimated with
    respect to the robot.
    The engagement statuses in which a person can be, are the following:
    # UNKNOWN: no information is provided about the engagement level
    # DISENGAGED: the person has not looked in the direction of the robot
    # ENGAGING: the person has started to look in the direction of the robot
    # ENGAGED: the person is fully engaged with the robot
    # DISENGAGING: the person has started to look away from the robot
    """

    def __init__(self, person_id=None, robot_gaze_frame="sellion_link"):
        """
        :param person_id:-> str
        person identifier flag to check whether the person pub
        is still registered
        :param robot_gaze_frame:-> str
        reference frame for the robot

        """
        self.person_id = person_id
        # face id of the person
        self.face_id = None
        # flag
        self.is_registered = True
        # buffer and listener for the tf2 transformation
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        # to change on the robot
        self.reference_frame = robot_gaze_frame
        # distance threshold for considering faces (mt)
        self.distance_thr = 1.5
        # scale value to define the field of view of the human (mt)
        self.fov_scale_thr = 0.3
        # time vars for computing the tf transform availability
        self.current_time_from_tf = rospy.Time.now()
        self.start_time_from_tf = rospy.Time.now()
        # timeout after which the engagement status is set to unknown
        self.timeout_tf = 10

        # publisher for the engagement status of the person
        try:
            self.engagement_status_pub = rospy.Publisher(
                "/humans/persons/" + self.person_id + "/engagement_status",
                EngagementLevel,
                queue_size=10,
            )
        except AttributeError:
            rospy.logerr(
                "cannot create a pub as "
                "the value of self.person_id is ".format(self.person_id)
            )

        # subscriber for getting the face_id assigned of the person
        self.face_id_sub = rospy.Subscriber(
            "/humans/persons/" + self.person_id + "/face_id",
            String,
            self.face_id_cb,
        )

        # number of samples used to infer the user's engagement
        self.engagement_history_size = NODE_RATE * BUFFER_DURATION

        # list in which it is stored the engagement level
        # of dim equals to self.engagement_history_size.
        self.person_engagement_history = list()

        # current engagement level
        self.person_current_engagement_level = EngagementLevel.UNKNOWN
        # publish the engagement status as soon as the Person is created
        self.publish_engagement_status()

    def unregister(self):
        """
        method that unregister the Person engagement_status_pub
        and the face_id_sub
        """
        self.engagement_status_pub.unregister()
        self.face_id_sub.unregister()
        self.is_registered = False

    def face_id_cb(self, msg):
        """
        Callback function to get the face_id of the person.
        Note, that the face_id is not persistent, and it can change as soon as a
        face is lost. The face_id is needed to compute the distance of the robot
        from the person as well as their engagement with respect to the robot.
        """
        self.face_id = msg

    def assess_engagement(self):
        """
        it first computes the distance between the robot and the person
        if distance is less than self.distance_thr then the person is active,
        and we can compute the gaze direction of the person with respect to the
        robot. Note, this method assesses the current engagement of the person as follows:
        if the person is looking at the robot we add +1 in the engagement_history,
        if not we add a -1. The vector will be then used by the Person class to estimate
        the human engagement over the BUFFER_DURATION
        """

        try:
            # transform from base link to gaze_[face_id]
            gaze_origin_trans = self.buffer.lookup_transform(
                self.reference_frame,
                "gaze_" + self.face_id.data,
                rospy.Time(0))

            face_distance = np.sqrt(gaze_origin_trans.transform.translation.x ** 2
                                    + gaze_origin_trans.transform.translation.y ** 2
                                    + gaze_origin_trans.transform.translation.z ** 2)

            # if the distance is less than self.distance_thr
            # we can consider that person as active and therefore
            # compute their engagement status
            if face_distance < self.distance_thr:
                # transform from gaze_[human_id] to reference_frame
                gaze_origin_inverse_trans = self.buffer.lookup_transform(
                    "gaze_" + self.face_id.data,
                    self.reference_frame,
                    rospy.Time(0))
                # check on the same vector expressed in gaze_<id> frame,
                # evaluating the length of its projection
                # on the XY plane.
                gaze_distance = np.sqrt(gaze_origin_inverse_trans.
                                        transform.translation.x ** 2
                                        + gaze_origin_inverse_trans.
                                        transform.translation.y ** 2)
                # the person is currently engaged
                if gaze_distance < \
                        (self.fov_scale_thr * gaze_origin_inverse_trans.
                                transform.translation.z):
                    self.person_engagement_history.append(1)
                else:
                    # the person is currently disengaged
                    self.person_engagement_history.append(-1)      
                # time of the last successful tf
                self.start_time_from_tf = rospy.Time.now()

        except tf2_ros.TransformException:
            rospy.logwarn("Unable to find a transform"
                          " between frame {} and frame {} "
                          "for person_id {}".
                          format(self.reference_frame,
                                 self.face_id.data, self.person_id))
            self.current_time_from_tf = (rospy.Time.now()
                                         - self.start_time_from_tf).to_sec()
            # if the tf is not available for more than timeout_tf secs
            # then set the engagement status of that person to UNKNOWN
            if self.current_time_from_tf >= self.timeout_tf:
                rospy.logwarn("Timeout. Set the EngagementLevel for person"
                              " {} to UNKNOWN".format(self.person_id))
                self.person_current_engagement_level \
                    = EngagementLevel.UNKNOWN
                self.publish_engagement_status()
                self.start_time_from_tf = rospy.Time.now()

    def compute_engagement(self):
        """
        Status can be "unknown", "disengaged", "engaging",
        "engaged", "disengaging".
        It computes the engagement level of the person averaging the values
        stored in the person_engagement_history that has size:
        engagement_history_size.
        At the beginning the person status is set to "unknown"
        """

        # waiting a few frames before assessing whether a person is engaged or not
        if len(self.person_engagement_history) \
                < self.engagement_history_size:
            rospy.logdebug("building buffer for person {}".format(self.person_id))
            return
        else:
            # clean up the person engagement history
            self.person_engagement_history = \
                self.person_engagement_history[-self.engagement_history_size:]

            # compute the average engagement value
            engagement_value = sum(self.person_engagement_history
                                   [-self.engagement_history_size:]) \
                               / self.engagement_history_size

            if self.person_current_engagement_level \
                    == EngagementLevel.UNKNOWN:
                self.person_current_engagement_level \
                    = EngagementLevel.DISENGAGED

            elif self.person_current_engagement_level \
                    == EngagementLevel.DISENGAGED:
                if 0 <= engagement_value < 1:
                    self.person_current_engagement_level \
                        = EngagementLevel.ENGAGING

            elif self.person_current_engagement_level \
                    == EngagementLevel.ENGAGING:
                if -1 <= engagement_value < -0.5:
                    self.person_current_engagement_level \
                        = EngagementLevel.DISENGAGED
                elif 0.0 < engagement_value <= 1.0:
                    self.person_current_engagement_level \
                        = EngagementLevel.ENGAGED

            elif self.person_current_engagement_level \
                    == EngagementLevel.ENGAGED:
                if engagement_value <= 0.0:
                    self.person_current_engagement_level \
                        = EngagementLevel.DISENGAGING

            elif self.person_current_engagement_level \
                    == EngagementLevel.DISENGAGING:
                if engagement_value > 0.0:
                    self.person_current_engagement_level \
                        = EngagementLevel.ENGAGED
                elif engagement_value <= 0.0:
                    self.person_current_engagement_level \
                        = EngagementLevel.DISENGAGED

    def publish_engagement_status(self):
        """
        method that publishes the engagement_status of the person
        """
        # we check whether the person id exists
        if self.is_registered:
            engagement_msg = EngagementLevel()
            engagement_msg.level = self.person_current_engagement_level
            self.engagement_status_pub.publish(engagement_msg)
            # publish the message only if it is different
            # or if is the same after 10 secs
            rospy.loginfo_throttle_identical(
                10,
                "Engagement status for {} is: {}".format(
                    self.person_id,
                    EngagementStatus(self.person_current_engagement_level),
                ),
            )

    def run(self):
        """
        it calls the engaged_person method that computes the
        engagement status and the callback that publishes the
        status on the topic /humans/persons/<human_id>/engagement_status
        """

        # if we do not have the face id of the person we just return
        if not self.face_id:
            rospy.loginfo_throttle_identical(
                1, "there is no face_id for the person {}".format(self.person_id)
            )
            return
        else:
            self.assess_engagement()
            self.compute_engagement()
            self.publish_engagement_status()


class EngagementNode(object):
    """
    This node detects the persons who are in the field of view of the
    robot's camera (tracked persons).
    Among those persons, it selects those who are active, that is, those who
    are currently in the engagement zone (distance from the robot <
    zone distance_thr).
    For each of the active persons, it creates a Person object from which
    the engagement status is computed and published in a topic.
    """

    def __init__(self,
                 robot_gaze_frame,
                 distance_thr=1.5,
                 fov_scale_thr=0.5):
        """
        :param robot_gaze_frame: -> str
        reference frame to compute if a human is in the field of view
        of the robot
        :param distance_thr: -> float
        max distance used by the robot to look for active users
        :param fov_scale_thr: -> float
        value to scale up the FoV of the humans expressed in meters
        """
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        # to change on the robot
        self.reference_frame = robot_gaze_frame

        # distance threshold for considering faces
        self.distance_thr = distance_thr
        # scale value to define the field of view of the human (meters)
        self.fov_scale_thr = fov_scale_thr

        # those persons who are actively detected
        self.tracked_persons_in_the_scene = IdsList()
        # those humans who are actively detected and at a distance
        # less than self.distance_thr, person_id:Person_instance
        self.active_persons = dict()
        # for the current version in order to compute engagement
        # we need to get the transformation from the gaze_id published
        # as tf by the hri_face_detection
        self.face_id = ""

        try:
            self.humans_tracked_id_sub = \
                rospy.Subscriber("/humans/persons/tracked",
                                 IdsList,
                                 self.get_tracked_humans_cb,
                                 queue_size=10)
        except AttributeError:
            rospy.logerr("No tracked persons")
            return

        # frame rate of the node (hz)
        self.loop_rate = rospy.Rate(NODE_RATE)

        # Person object
        self.person = None

    def get_tracked_humans_cb(self, msg):
        """
        Callback that updates the list of the detected persons
        :param msg: -> IdList
        ids and header of the tracked persons
        """
        self.tracked_persons_in_the_scene = msg

    def get_tracked_humans(self):

        # check if the current active persons are
        # still active otherwise: unregister them and remove from the dict
        if self.active_persons:
            for active_human in list(self.active_persons.keys()):
                if active_human not in self.tracked_persons_in_the_scene.ids:
                    self.active_persons[
                        active_human
                    ].person_current_engagement_level = EngagementLevel.UNKNOWN
                    self.active_persons[active_human].publish_engagement_status()
                    self.active_persons[active_human].unregister()
                    del self.active_persons[active_human]
        else:
            rospy.loginfo_throttle(1, "There are no active people around")

        # check whether the active persons are new
        # if so create a new instance of a Person
        for person_id in self.tracked_persons_in_the_scene.ids:
            active_persons_id = list(self.active_persons.keys())

            if person_id in active_persons_id:
                self.active_persons[person_id].run()
            else:
                self.person = Person(
                    person_id=person_id,
                    robot_gaze_frame=self.reference_frame,
                    visual_social_engagement_thr=self.visual_social_engagement_thr,
                )
                self.active_persons[person_id] = self.person
                self.person.run()

    def run(self):
        """Timer callback to loop for the active humans and compute
        their engagement status according to timer set in the constructor"""
        while not rospy.is_shutdown():
            self.get_tracked_humans()
            self.loop_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("engagement_node")
    robot_gaze_frame = rospy.get_param("~robot_gaze_frame", default="sellion_link")
    node = EngagementNode(robot_gaze_frame)
    node.run()

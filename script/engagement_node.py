#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import absolute_import
import rospy
from hri_msgs.msg import IdsList
from hri_msgs.msg import EngagementLevel
from std_msgs.msg import String
import tf2_ros
from tf import transformations

import numpy as np
import math
import enum

# time window to store the engagement status of the person (secs)
BUFFER_DURATION = 4  # secs
# rate of the main node
NODE_RATE = 20

# field of 'attention' of a person
FOV = 60.0 * math.pi / 180

# threshold of 'visual social engagement' to consider engagement. See
# Person.assess_engagement for detailed explanation.
VISUAL_SOCIAL_ENGAGEMENT_THR = 0.5


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

    def __init__(
        self, person_id, reference_frame: str, visual_social_engagement_thr: float
    ):
        """
        :param person_id:-> str
        person identifier flag to check whether the person pub
        is still registered
        :param reference_frame:-> str
        reference frame for the robot
        :param visual_social_engagement_thr: -> float
        visual social engagement threshold to be considered as 'engaging' with the robot

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
        self.reference_frame = reference_frame

        self.visual_social_engagement_thr = visual_social_engagement_thr

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
        self.face_id = msg.data

    def distance(self, t):
        return math.sqrt(
            t.translation.x ** 2 + t.translation.y ** 2 + t.translation.z ** 2
        )

    def assess_engagement(self):
        """
        Computes the current 'visual social engagement' metric as defined in
        "Measuring Visual Social Engagement from Proxemics and Gaze" (by Webb
        and Lemaignan).

        If the person's engagement metric is above
        visual_social_engagement_thr, we add +1 in the engagement_history, if
        not we add a -1. The vector will be then used by the Person class to
        estimate the human engagement over the BUFFER_DURATION

        """

        gaze_frame = "gaze_" + self.face_id

        try:
            # compute the robot 'viewed' from the person's gaze (gaze_[face_id])
            origin_gaze_trans = self.buffer.lookup_transform(
                gaze_frame, self.reference_frame, rospy.Time(0)
            )

        except tf2_ros.TransformException:
            rospy.logwarn(
                "Unable to find a transform"
                " between frame {} and frame {} "
                "for person_id {}".format(
                    gaze_frame, self.reference_frame, self.person_id
                )
            )
            self.current_time_from_tf = (
                rospy.Time.now() - self.start_time_from_tf
            ).to_sec()

            # if the tf is not available for more than timeout_tf secs
            # then set the engagement status of that person to UNKNOWN
            if self.current_time_from_tf >= self.timeout_tf:
                rospy.logwarn(
                    "Timeout. Set the EngagementLevel for person"
                    " {} to UNKNOWN".format(self.person_id)
                )
                self.person_current_engagement_level = EngagementLevel.UNKNOWN
                self.publish_engagement_status()
                self.start_time_from_tf = rospy.Time.now()

            return

        # computation of Visual Social Engagement, following "Measuring Visual
        # Social Engagement from Proxemics and Gaze" by Webb et Lemaignan

        # first, compute the inverse transformation
        trans = [
            origin_gaze_trans.transform.translation.x,
            origin_gaze_trans.transform.translation.y,
            origin_gaze_trans.transform.translation.z,
        ]

        rot = [
            origin_gaze_trans.transform.rotation.x,
            origin_gaze_trans.transform.rotation.y,
            origin_gaze_trans.transform.rotation.z,
            origin_gaze_trans.transform.rotation.w,
        ]
        transform = transformations.concatenate_matrices(
            transformations.translation_matrix(trans),
            transformations.quaternion_matrix(rot),
        )
        inversed_transform = transformations.inverse_matrix(transform)
        inverse_translation = transformations.translation_from_matrix(
            inversed_transform
        )

        # next, compute the measure of visual social engagement

        # [in the following, A denotes the person and B denotes the robot]

        d_AB = self.distance(origin_gaze_trans.transform)

        ## gaze_AB
        # gaze_AB measures how 'close' B is from the optical axis of A

        # frame conventions:
        #  - the *gaze* is using the 'optical frame' convention: +Z forward, +Y down
        #  - the *reference frame* (normally, the sellion link) is using the +Z up, +X forward orientation
        #
        # -> change coordinates to match those used in
        # paper above
        xB = origin_gaze_trans.transform.translation.z
        yB = origin_gaze_trans.transform.translation.x
        zB = origin_gaze_trans.transform.translation.y

        gaze_AB = 0.0
        if xB > 0:
            gaze_AB = max(0, 1 - (math.sqrt(yB ** 2 + zB ** 2) / (math.tan(FOV) * xB)))

        # gaze_BA measures how 'close' A is from the optical axis of B
        xA = inverse_translation[0]
        yA = inverse_translation[1]
        zA = inverse_translation[2]

        gaze_BA = 0.0
        if xA > 0:
            gaze_BA = max(0, 1 - (math.sqrt(yA ** 2 + zA ** 2) / (math.tan(FOV) * xA)))

        M_AB = gaze_AB * gaze_BA

        S_AB = min(1, M_AB / d_AB)
        rospy.logdebug("S_AB: %s" % S_AB)

        if S_AB > self.visual_social_engagement_thr:
            self.person_engagement_history.append(1)
        else:
            # the person is currently disengaged
            self.person_engagement_history.append(-1)

        # time of the last successful tf
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
        if len(self.person_engagement_history) < self.engagement_history_size:
            rospy.logdebug("building buffer for person {}".format(self.person_id))
            return
        else:
            # clean up the person engagement history
            self.person_engagement_history = self.person_engagement_history[
                -self.engagement_history_size :
            ]

            # compute the average engagement value
            engagement_value = (
                sum(self.person_engagement_history) / self.engagement_history_size
            )

            rospy.logdebug("History: %s" % self.person_engagement_history)
            rospy.logdebug("Mean: %s" % engagement_value)

            if self.person_current_engagement_level == EngagementLevel.UNKNOWN:
                self.person_current_engagement_level = EngagementLevel.DISENGAGED

            elif self.person_current_engagement_level == EngagementLevel.DISENGAGED:
                if 0 <= engagement_value < 1:
                    self.person_current_engagement_level = EngagementLevel.ENGAGING

            elif self.person_current_engagement_level == EngagementLevel.ENGAGING:
                if -1 <= engagement_value < -0.5:
                    self.person_current_engagement_level = EngagementLevel.DISENGAGED
                elif 0.5 < engagement_value <= 1.0:
                    self.person_current_engagement_level = EngagementLevel.ENGAGED

            elif self.person_current_engagement_level == EngagementLevel.ENGAGED:
                if engagement_value <= 0.5:
                    self.person_current_engagement_level = EngagementLevel.DISENGAGING

            elif self.person_current_engagement_level == EngagementLevel.DISENGAGING:
                if engagement_value > 0.5:
                    self.person_current_engagement_level = EngagementLevel.ENGAGED
                elif engagement_value <= 0.0:
                    self.person_current_engagement_level = EngagementLevel.DISENGAGED

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
    Among those persons, it selects those who are active, that is, those whose
    visual social engagement metric (as defined in "Measuring Visual Social
    Engagement from Proxemics and Gaze" by Webb and Lemaignan) is above 0.5.

    For each of the active persons, it creates a Person object from which
    the engagement status is computed and published in a topic.
    """

    def __init__(
        self,
        reference_frame: str,
        visual_social_engagement_thr: float = VISUAL_SOCIAL_ENGAGEMENT_THR,
    ):
        """
        :param reference_frame: -> str
        reference frame to compute if a human is in the field of view
        of the robot
        :param visual_social_engagement_thr: -> float
        visual social engagement threshold to be considered as 'engaging' with the robot
        """
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        # to change on the robot
        self.reference_frame = reference_frame

        # visual social engagement threshold
        self.visual_social_engagement_thr = visual_social_engagement_thr

        # those persons who are actively detected
        self.tracked_persons_in_the_scene = IdsList()
        # those humans who are actively detected and are considered as 'engaged'
        self.active_persons = dict()

        try:
            self.humans_tracked_id_sub = rospy.Subscriber(
                "/humans/persons/tracked",
                IdsList,
                self.get_tracked_humans_cb,
                queue_size=10,
            )
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
                    reference_frame=self.reference_frame,
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
    reference_frame = rospy.get_param("~reference_frame", default="sellion_link")
    FOV = rospy.get_param("~field_of_view", default=FOV)
    VISUAL_SOCIAL_ENGAGEMENT_THR = rospy.get_param(
        "~engagement_threshold", default=VISUAL_SOCIAL_ENGAGEMENT_THR
    )

    node = EngagementNode(reference_frame)
    node.run()

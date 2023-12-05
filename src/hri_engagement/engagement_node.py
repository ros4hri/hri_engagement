# -*- coding: utf-8 -*-

import rospy
from pyhri import HRIListener
from pyhri.person import Person
from hri_msgs.msg import EngagementLevel
from hri_actions_msgs.msg import Intent
import tf2_ros
from tf import transformations

import math
import json

# time window to store the engagement status of the person (secs)
BUFFER_DURATION = 10  # secs
# rate of the main node
NODE_RATE = 10

# people further away than MAX_DISTANCE have their engagement level set to DISENGAGED
MAX_DISTANCE = 4  # m

# field of 'attention' of a person
FOV = 60.0 * math.pi / 180

# threshold of 'visual social engagement' to consider engagement. See
# Person.assess_engagement for detailed explanation.
VISUAL_SOCIAL_ENGAGEMENT_THR = 0.3

# reference frame of the robot
REFERENCE_FRAME = "sellion_link"

EngagementStatus = {
    # unknown: no information is provided about the engagement level
    EngagementLevel.UNKNOWN: "UNKNOWN",
    # disengaged: the person has not looked in the direction of the robot
    EngagementLevel.DISENGAGED: "DISENGAGED",
    # engaging: the person has started to look in the direction of the robot
    EngagementLevel.ENGAGING: "ENGAGING",
    # engaged: the person is fully engaged with the robot
    EngagementLevel.ENGAGED: "ENGAGED",
    # disengaging: the person has started to look away from the robot
    EngagementLevel.DISENGAGING: "DISENGAGING"
}


class PersonEngagement(object):
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

    def __init__(self, person: Person, reference_frame=REFERENCE_FRAME):
        self.person = person

        self.reference_frame = reference_frame
        # flag
        self.is_registered = True

        # time vars for computing the tf transform availability
        self.current_time_from_tf = rospy.Time.now()
        self.start_time_from_tf = rospy.Time.now()
        # timeout after which the engagement status is set to unknown
        self.timeout_tf = 10

        # publisher for the engagement status of the person
        try:
            self.engagement_status_pub = \
                rospy.Publisher(
                    self.person.ns +
                    "/engagement_status",
                    EngagementLevel,
                    queue_size=10,
                )
            self.intent_pub = \
                rospy.Publisher("/intents",
                                Intent,
                                queue_size=10)

        except AttributeError:
            rospy.logwarn(
                "cannot create a pub as "
                "the value of self.person_id is ".format(self.person.id)
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
        self.person_current_engagement_level = EngagementLevel.UNKNOWN
        self.publish_engagement_status()
        self.engagement_status_pub.unregister()

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

        if not self.person.face:
            rospy.logdebug("No face detected, can not compute engagement")
            return

        # compute the person's position 'viewed' from the robot's 'gaze'
        person_from_robot = self.person.face.gaze_transform(
            from_frame=self.reference_frame)

        if person_from_robot == tf2_ros.TransformStamped():
            rospy.logdebug(
                "null transform published for person's face %s" %
                self.person.face
            )
            self.current_time_from_tf = (
                rospy.Time.now() - self.start_time_from_tf
            ).to_sec()

            # if the tf is not available for more than timeout_tf secs
            # then set the engagement status of that person to UNKNOWN
            if self.current_time_from_tf >= self.timeout_tf:
                rospy.logdebug(
                    "Timeout. Set the EngagementLevel for person"
                    " {} to UNKNOWN".format(self.person.id)
                )
                self.person_current_engagement_level = EngagementLevel.UNKNOWN
                self.publish_engagement_status()
                self.start_time_from_tf = rospy.Time.now()

            return

        ######################################################################
        # computation of Visual Social Engagement, following "Measuring Visual
        # Social Engagement from Proxemics and Gaze" by Webb et Lemaignan

        # first, compute the inverse transformation
        trans_p_from_r = [
            person_from_robot.transform.translation.x,
            person_from_robot.transform.translation.y,
            person_from_robot.transform.translation.z,
        ]
        rot_p_from_r = [
            person_from_robot.transform.rotation.x,
            person_from_robot.transform.rotation.y,
            person_from_robot.transform.rotation.z,
            person_from_robot.transform.rotation.w,
        ]
        transform_p_from_r = transformations.concatenate_matrices(
            transformations.translation_matrix(trans_p_from_r),
            transformations.quaternion_matrix(rot_p_from_r),
        )

        robot_from_person = transformations.inverse_matrix(transform_p_from_r)

        # next, compute the measure of visual social engagement
        # [in the following, A denotes the person and B denotes the robot]

        tx, ty, tz = transformations.translation_from_matrix(robot_from_person)
        d_ab = math.sqrt(tx ** 2 + ty ** 2 + tz ** 2)

        if d_ab > MAX_DISTANCE:
            self.person_engagement_history.append(-1)
            rospy.logdebug(
                f"dAB: {d_ab:.2f}: person is too far to be engaged (max distance: {MAX_DISTANCE}m)")

            return

        # gaze_AB
        # gaze_AB measures how 'close' B is from the optical axis of A

        # frame conventions:
        #  - the *gaze* is using the 'optical frame' convention: +Z forward, +Y down
        #  - the *reference frame* (normally, the sellion link) is using the +Z up,
        #    +X forward orientation
        #
        # -> change coordinates to match those used in the paper above
        xb = tz
        yb = tx
        zb = ty

        gaze_ab = 0.0
        if xb > 0:
            gaze_ab = max(0, 1 - (math.sqrt(yb ** 2 + zb ** 2) /
                                  (math.tan(FOV) * xb)))

        # gaze_BA measures how 'close' A is from the optical axis of B
        t_ba = person_from_robot.transform.translation
        xa = t_ba.x
        ya = t_ba.y
        za = t_ba.z

        gaze_ba = 0.0
        log_d_ab = 0.0
        if xa > 0:
            gaze_ba = max(0, 1 - (math.sqrt(ya ** 2 + za ** 2) /
                                  (math.tan(FOV) * xa)))

            # transform the distance into a factor that:
            #  - starts at 1 when distance = 0
            #  - 'gently' goes down to 0
            #  - reaches 0 at MAX_DISTANCE
            log_d_ab = math.log(-d_ab + MAX_DISTANCE + 1) / \
                math.log(MAX_DISTANCE + 1)

        m_ab = gaze_ab * gaze_ba
        s_ab = min(1, m_ab * log_d_ab)
        rospy.logdebug(
            f"dAB: {d_ab:.2f}, log(d_ab): {log_d_ab:.2f}, gazeAB: {gaze_ab:.2f}, gazeBA: {gaze_ba:.2f},  M_AB: {m_ab:.2f}, S_AB: {s_ab:.2f}")

        if s_ab > VISUAL_SOCIAL_ENGAGEMENT_THR:
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

        # start computed engaged when reaching half the nominal buffer size
        if len(self.person_engagement_history) < BUFFER_DURATION * NODE_RATE * 0.5:
            return

        # clean up the person engagement history
        self.person_engagement_history = self.person_engagement_history[
            -self.engagement_history_size:]

        # compute the average engagement value
        engagement_value = (
            sum(self.person_engagement_history) /
            len(self.person_engagement_history)
        )

        rospy.logdebug("History: %s" % self.person_engagement_history)
        rospy.logdebug("Mean: %s" % engagement_value)

        next_level = EngagementLevel.UNKNOWN

        if self.person_current_engagement_level == EngagementLevel.UNKNOWN:
            next_level = EngagementLevel.DISENGAGED

        elif self.person_current_engagement_level == EngagementLevel.DISENGAGED:
            if engagement_value > -0.4:
                next_level = EngagementLevel.ENGAGING

        elif self.person_current_engagement_level == EngagementLevel.ENGAGING:
            if engagement_value < -0.6:
                next_level = EngagementLevel.DISENGAGED
            elif engagement_value > 0.5:
                next_level = EngagementLevel.ENGAGED

        elif self.person_current_engagement_level == EngagementLevel.ENGAGED:
            if engagement_value < 0.4:
                next_level = EngagementLevel.DISENGAGING

        elif self.person_current_engagement_level == EngagementLevel.DISENGAGING:
            if engagement_value > 0.6:
                next_level = EngagementLevel.ENGAGED
            elif engagement_value < -0.5:
                next_level = EngagementLevel.DISENGAGED

        if next_level != EngagementLevel.UNKNOWN and next_level != self.person_current_engagement_level:
            self.person_current_engagement_level = next_level
            rospy.loginfo("Engagement status for {} is: {}".format(
                self.person.id,
                EngagementStatus[self.person_current_engagement_level],
            ),
            )

            # if we just became engaged, publish an ENGAGE_WITH intent
            if self.person_current_engagement_level == EngagementLevel.ENGAGED:
                intent_msg = Intent()
                intent_msg.intent = intent_msg.ENGAGE_WITH
                intent_msg.data = json.dumps({"recipient": self.person.id})
                self.intent_pub.publish(intent_msg)

    def publish_engagement_status(self):
        """
        method that publishes the engagement_status of the person
        """

        engagement_msg = EngagementLevel()
        engagement_msg.header.stamp = rospy.Time.now()
        engagement_msg.level = self.person_current_engagement_level
        self.engagement_status_pub.publish(engagement_msg)

    def run(self):
        """
        it calls the engaged_person method that computes the
        engagement status and the callback that publishes the
        status on the topic /humans/persons/<human_id>/engagement_status
        """

        # if we do not have the face id of the person we just return
        if not self.person.id:
            rospy.logdebug_throttle_identical(
                1, "there is no face_id for the person {}".format(
                    self.person.id)
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
            visual_social_engagement_thr: float = VISUAL_SOCIAL_ENGAGEMENT_THR,
            reference_frame: str = REFERENCE_FRAME
    ):
        """
        :param visual_social_engagement_thr: -> float
        visual social engagement threshold to be considered as 'engaging' with 
        the robot
        """

        # visual social engagement threshold
        self.visual_social_engagement_thr = visual_social_engagement_thr
        self.reference_frame = reference_frame
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.hri_listener = HRIListener()

        # get the list of IDs of the currently visible persons
        self.tracked_persons_in_the_scene = None

        # those humans who are actively detected and are considered as 'engaged'
        self.active_persons = dict()

        # frame rate of the node (hz)
        self.loop_rate = rospy.Rate(NODE_RATE)

    def get_tracked_humans(self):
        """
        updater of self.active_persons, the dictionary of tracked humans
        PersonEngagement
        """

        self.tracked_persons_in_the_scene = self.hri_listener.tracked_persons

        # check if the current active persons are
        # still active otherwise: unregister them and remove from the dict
        if self.active_persons:
            for active_human in list(self.active_persons.keys()):
                if active_human not in self.tracked_persons_in_the_scene.keys():
                    self.active_persons[
                        active_human
                    ].person_current_engagement_level = EngagementLevel.UNKNOWN
                    self.active_persons[active_human].publish_engagement_status(
                    )
                    self.active_persons[active_human].unregister()
                    del self.active_persons[active_human]
        else:
            rospy.loginfo_throttle(1, "There are no active people around")

        # check whether the active persons are new
        # if so create a new instance of a Person
        for person_id, person_instance in self.tracked_persons_in_the_scene.items():
            active_persons_id = list(self.active_persons.keys())

            if person_id in active_persons_id:
                self.active_persons[person_id].run()
            else:
                self.active_persons[person_id] = PersonEngagement(
                    person_instance, self.reference_frame)
                self.active_persons[person_id].run()

    def run(self):
        """Timer callback to loop for the active humans and compute
        their engagement status according to timer set in the constructor"""
        while not rospy.is_shutdown():
            self.get_tracked_humans()
            self.loop_rate.sleep()
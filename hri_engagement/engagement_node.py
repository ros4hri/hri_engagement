# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import json

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from hri import HRIListener, Person
from hri_msgs.msg import EngagementLevel
from hri_actions_msgs.msg import Intent
import tf_transformations as transformations
from geometry_msgs.msg import TransformStamped

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

# diagnostic message publish rate in Hertz
DIAG_PUB_RATE = 1


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

    def __init__(self,
                 node: Node,
                 person: Person,
                 reference_frame: str,
                 max_distance: float,
                 field_of_view: float,
                 engagement_threshold: float,
                 node_rate: float,
                 buffer_duration: float,
                 ):

        self.node = node

        self.person = person

        self.reference_frame = reference_frame
        self.max_distance = max_distance
        self.fov = field_of_view
        self.visual_social_engagement_thr = engagement_threshold

        # number of samples used to infer the user's engagement
        self.engagement_history_size = node_rate * buffer_duration

        self.is_registered = True

        # time vars for computing the tf transform availability
        self.current_time_from_tf = self.node.get_clock().now()
        self.start_time_from_tf = self.node.get_clock().now()
        # timeout after which the engagement status is set to unknown
        self.timeout_tf = 10

        # publisher for the engagement status of the person
        try:
            self.engagement_status_pub = self.node.create_publisher(
                EngagementLevel,
                self.person.ns +
                "/engagement_status",
                10,
            )
            self.intent_pub = self.node.create_publisher(
                Intent,
                "/intents",
                10)

        except AttributeError:
            self.get_logger().warn(
                "cannot create a pub as "
                "the value of self.person_id is ".format(self.person.id)
            )

        # list in which it is stored the engagement level
        # of dim equals to self.engagement_history_size.
        self.person_engagement_history = list()

        # current engagement level
        self.person_current_engagement_level = EngagementLevel.UNKNOWN
        # publish the engagement status as soon as the Person is created
        self.publish_engagement_status()

    def get_logger(self):
        return self.node.get_logger()

    def unregister(self):
        """
        method that unregister the Person engagement_status_pub
        and the face_id_sub
        """
        self.person_current_engagement_level = EngagementLevel.UNKNOWN
        self.publish_engagement_status()
        self.engagement_status_pub.destroy()
        self.intent_pub.destroy()

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
            self.get_logger().debug("No face detected, can not compute engagement")
            return

        if not self.person.face.gaze_transform:
            self.get_logger().debug(
                "Face detected, but can not compute gaze direction. Can not compute engagement")
            return

        # compute the person's position 'viewed' from the robot's 'gaze'
        person_from_robot = self.person.face.gaze_transform(
            from_frame=self.reference_frame)

        if person_from_robot == TransformStamped():
            self.get_logger().debug(
                "null transform published for person's face %s" %
                self.person.face
            )
            self.current_time_from_tf = (
                self.node.get_clock().now() - self.start_time_from_tf
            ).to_sec()

            # if the tf is not available for more than timeout_tf secs
            # then set the engagement status of that person to UNKNOWN
            if self.current_time_from_tf >= self.timeout_tf:
                self.get_logger().debug(
                    "Timeout. Set the EngagementLevel for person"
                    " {} to UNKNOWN".format(self.person.id)
                )
                self.person_current_engagement_level = EngagementLevel.UNKNOWN
                self.publish_engagement_status()
                self.start_time_from_tf = self.node.get_clock().now()

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

        if d_ab > self.max_distance:
            self.person_engagement_history.append(-1)
            self.get_logger().debug(
                f"dAB: {d_ab:.2f}: person is too far to be engaged (max distance: {self.max_distance}m)")

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
                                  (math.tan(self.fov) * xb)))

        # gaze_BA measures how 'close' A is from the optical axis of B
        t_ba = person_from_robot.transform.translation
        xa = t_ba.x
        ya = t_ba.y
        za = t_ba.z

        gaze_ba = 0.0
        log_d_ab = 0.0
        if xa > 0:
            gaze_ba = max(0, 1 - (math.sqrt(ya ** 2 + za ** 2) /
                                  (math.tan(self.fov) * xa)))

            # transform the distance into a factor that:
            #  - starts at 1 when distance = 0
            #  - 'gently' goes down to 0
            #  - reaches 0 at max_distance
            log_d_ab = math.log(-d_ab + self.max_distance + 1) / \
                math.log(self.max_distance + 1)

        m_ab = gaze_ab * gaze_ba
        s_ab = min(1, m_ab * log_d_ab)
        self.get_logger().debug(
            f"dAB: {d_ab:.2f}, log(d_ab): {log_d_ab:.2f}, gazeAB: {gaze_ab:.2f}, gazeBA: {gaze_ba:.2f},  M_AB: {m_ab:.2f}, S_AB: {s_ab:.2f}")

        if s_ab > self.visual_social_engagement_thr:
            self.person_engagement_history.append(1)
        else:
            # the person is currently disengaged
            self.person_engagement_history.append(-1)
        # time of the last successful tf
        self.start_time_from_tf = self.node.get_clock().now()

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
        if len(self.person_engagement_history) < self.engagement_history_size * 0.5:
            return

        # clean up the person engagement history
        self.person_engagement_history = self.person_engagement_history[
            -self.engagement_history_size:]

        # compute the average engagement value
        engagement_value = (
            sum(self.person_engagement_history) /
            len(self.person_engagement_history)
        )

        self.get_logger().debug("History: %s" % self.person_engagement_history)
        self.get_logger().debug("Mean: %s" % engagement_value)

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
            self.get_logger().info("Engagement status for {} is: {}".format(
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
        engagement_msg.header.stamp = self.node.get_clock().now().to_msg()
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
            self.get_logger().debug(
                "there is no face_id for the person {}".format(
                    self.person.id), throttle_duration_sec=1
            )
            return
        else:
            self.assess_engagement()
            self.compute_engagement()
            self.publish_engagement_status()


class EngagementNode(Node):
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
    ):
        """
        :param visual_social_engagement_thr: -> float
        visual social engagement threshold to be considered as 'engaging' with 
        the robot
        """
        super().__init__('hri_engagement')

        self.declare_parameter(
            'reference_frame', 'sellion_link', ParameterDescriptor(
                description="Robot's reference point, used to compute the distance and mutual gaze."))

        self.declare_parameter(
            'max_distance', 4.0, ParameterDescriptor(
                description="People further away than this distance (in meters) are considered as disengaged."))

        self.declare_parameter(
            'field_of_view', 60., ParameterDescriptor(
                description="Field of view (in degrees) of both humans and robot. Use to compute mutual gaze between the robot and the people."))

        self.declare_parameter(
            'engagement_threshold', 0.5, ParameterDescriptor(
                description="Threshold to start considering someone as 'engaging'. Higher values will make the engagement status more conservative."))

        self.declare_parameter(
            'observation_window', 10., ParameterDescriptor(
                description="The time window (in sec.) used to compute the engagement level of a person."))

        self.declare_parameter(
            'rate', 10., ParameterDescriptor(
                description="Engagement level computation and publication rate (in Hz)."))

        # get the list of IDs of the currently visible persons
        self.tracked_persons_in_the_scene = None

        # those humans who are actively detected and are considered as 'engaged'
        self.active_persons = dict()

        self.get_logger().info('State: Unconfigured.')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:

        self.max_distance = self.get_parameter("max_distance").value
        self.reference_frame = self.get_parameter("reference_frame").value
        self.fov = self.get_parameter("field_of_view").value * math.pi / 180
        self.visual_social_engagement_thr = self.get_parameter(
            "engagement_threshold").value
        self.buffer_duration = self.get_parameter("observation_window").value
        self.node_rate = self.get_parameter("rate").value

        self.get_logger().info('State: Inactive.')
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:

        self.hri_listener = HRIListener("hri_engagement_listener")

        self.proc_timer = self.create_timer(
            1/self.node_rate, self.get_tracked_humans)

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 1)
        self.diag_timer = self.create_timer(
            1/DIAG_PUB_RATE, self.do_diagnostics)

        self.get_logger().info('State: Active.')
        return super().on_activate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('State: Unconfigured.')
        return super().on_cleanup(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        del self.hri_listener
        self.destroy_ros_interfaces()
        self.get_logger().info('State: Inactive.')
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        if state.id == State.PRIMARY_STATE_ACTIVE:
            self.destroy_ros_interfaces()
        self.get_logger().info('State: Finalized.')
        return super().on_shutdown(state)

    def destroy_ros_interfaces(self):
        self.destroy_timer(self.diag_timer)
        self.destroy_publisher(self.diag_pub)

        self.destroy_timer(self.proc_timer)

        for _, person in self.active_persons.items():
            person.unregister()

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
            self.get_logger().info("There are no active people around", throttle_duration_sec=1)

        # check whether the active persons are new
        # if so create a new instance of a Person
        for person_id, person_instance in self.tracked_persons_in_the_scene.items():
            active_persons_id = list(self.active_persons.keys())

            if person_id in active_persons_id:
                self.active_persons[person_id].run()
            else:
                self.active_persons[person_id] = PersonEngagement(
                    self,
                    person_instance,
                    self.reference_frame,
                    self.max_distance,
                    self.fov,
                    self.visual_social_engagement_thr,
                    self.node_rate,
                    self.buffer_duration,
                )

                self.active_persons[person_id].run()

    def do_diagnostics(self):
        now = self.get_clock().now()
        arr = DiagnosticArray(header=Header(stamp=now.to_msg()))
        msg = DiagnosticStatus(
            name='Social perception: Engagement estimation', hardware_id='none')

        msg.level = DiagnosticStatus.OK

        msg.values = [
            KeyValue(key='Package name', value='hri_engagement'),
            KeyValue(key='Current engagement levels:',
                     value=str({k: EngagementStatus[v.person_current_engagement_level] for k, v in self.active_persons.items()})),
        ]

        arr.status = [msg]
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = EngagementNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()


if __name__ == '__main__':
    main()

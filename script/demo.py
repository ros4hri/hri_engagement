#!/usr/bin/env python


from __future__ import absolute_import

# ros import
import rospy
from hri_msgs.msg import IdsList
from hri_msgs.msg import EngagementLevel
from std_msgs.msg import String
import tf

from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import actionlib
# import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal

# python import
import numpy as np
import time

''' The robot detects the engaged humans in its field of view 
    - It subscribes to the topic /humans/persons/tracked
        - for any human in tracked 
            -   detect their engagement status subscribing to /humans/persons/<person_id>/engagement_status
            -   decode their engagement status as follows:
                # unknown: no information is provided about the engagement level 
                uint8 UNKNOWN=0
                # disengaged: the human has not looked in the direction of the robot
                uint8 DISENGAGED=1
                # engaging: the human has started to look in the direction of the robot
                uint8 ENGAGING=2
                # engaged: the human is fully engaged with the robot
                uint8 ENGAGED=3
                # disengaging: the human has started to look away from the robot
                uint8 DISENGAGING=4
            -   if they are engaged then:
                Say "HI" and play a motion
            
'''


class RobotNode:

    def __init__(self, language):
        self.client_motion = SimpleActionClient('/play_motion', PlayMotionAction)
        self.client_voice = SimpleActionClient('/tts', TtsAction)
        self.language = language
        self.reproduction_has_ended = False
        self.tracked_humans_in_the_scene = IdsList()
        self.engaged_status = EngagementLevel()
        self.humans_engagement_history = {}

       # this dict stores the info related to the human for engaging them
       # every n sec human_id:[engagament_time, elapsed_time, robot_engaged_action]

        self.engagement_info = {}
        self.timeout = 10


        # frame rate of the node
        self.frame_rate = 2
        # time for engagement history
        self.buffer_time = 5
        # number of samples used to infer the user's engagement
        self.engagement_history_size = self.frame_rate * self.buffer_time

        self.loop_rate = rospy.Rate(self.frame_rate)


    def wait_for_valid_time(self, timeout):
        """Wait for a valid time (non-zero), this is important
        when using a simulated clock"""
        # Loop until:
        # * ros master shutdowns
        # * control+C is pressed (handled in is_shutdown())
        # * timeout is achieved
        # * time is valid
        start_time = time.time()
        while not rospy.is_shutdown():
            if not rospy.Time.now().is_zero():
                return
            if time.time() - start_time > timeout:
                rospy.logerr("Timed-out waiting for valid time.")
                exit(0)
            time.sleep(0.1)
        # If control+C is pressed the loop breaks, we can exit
        exit(0)

    def tracked_human_list_cb(self, msg):
        """ Callback that updates the detected persons
             msg: IdList that consists of ids and header """
        self.tracked_humans_in_the_scene = msg

    def engaged_human_cb(self, msg):
        self.engaged_status = msg

    def get_tracked_humans(self):
        while (not self.tracked_humans_in_the_scene.ids):
            # read the current tracked humans
            rospy.Subscriber(
                "/humans/persons/tracked",
                IdsList,
                self.tracked_human_list_cb)
            continue

        for human_id in self.tracked_humans_in_the_scene.ids:
            # for each active human we initialise the engagement history vector
            if human_id not in self.humans_engagement_history:
                self.humans_engagement_history[human_id] = [0]
                self.engagement_info[human_id] = [rospy.Time.now(), 0.0, False]


    def get_engaged_status(self, human_id):

        # waiting a few frames before assessing whether a person is engaged or not
        if len(self.humans_engagement_history[human_id]) < self.engagement_history_size:
            rospy.loginfo("waiting {} for human {} to engage with the robot".
                          format(self.humans_engagement_history[human_id], human_id))
        else:
            # clean up the buffer
            self.humans_engagement_history[human_id] = \
                self.humans_engagement_history[human_id][-self.engagement_history_size:]

        self.get_tracked_humans()

        # if there is at least one human in the scene
        if self.tracked_humans_in_the_scene.ids:
            # check their engagement status
            for human_id in self.tracked_humans_in_the_scene.ids:
                rospy.Subscriber(
                    "/humans/persons/" + human_id + "/engagement_status",
                    EngagementLevel,
                    self.engaged_human_cb)
                print("human id {} engagement status {}".format(human_id, self.engaged_status))
                self.humans_engagement_history[human_id].append(self.engaged_status)


    def reproduce_text(self, text, locked=False):
        rospy.loginfo("Waiting for Server")
        self.client_voice.wait_for_server()
        rospy.loginfo("Reached Server")
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = self.language
        self.client_voice.send_goal(goal)

    def reproduce_motion(self, action_name):
        """ Reproduce the motion passed as input
        Parameters
        -----------
        :action_name: str
        The name of the action in the yaml file
        Returns
        -------
        None
        """
        rospy.loginfo("Starting run_motion application ...")
        self.wait_for_valid_time(10.0)
        rospy.loginfo("Waiting for Action Server ...")
        self.client_motion.wait_for_server()
        # call play_motion
        goal = PlayMotionGoal()
        goal.motion_name = action_name
        goal.skip_planning = False
        goal.priority = 0  # Optional

        rospy.loginfo("Sending goal with motion: " + str(action_name))
        self.send_goal(goal)
        rospy.loginfo("Execute action without waiting for result...")

    def check_time(self, human_id):
        last_engagement = self.engagement_info[human_id][0]
        elapsed_time = self.engagement_info[human_id][1]

        elapsed_time += rospy.Time.now().to_sec()-last_engagement.to_sec()
        print(elapsed_time)
        if elapsed_time < self.timeout:
            return True
        else:
            self.engagement_info[human_id][0] = rospy.Time.now()
            self.engagement_info[human_id][1] = 0
            self.engagement_info[human_id][2] = False
            return False

    def run(self):
        while not rospy.is_shutdown():
            self.get_tracked_humans()
            if self.tracked_humans_in_the_scene:
                for human_id in self.tracked_humans_in_the_scene.ids:
                    self.get_engaged_status(human_id)
                    # compute the average engagement status
                    if self.check_time(human_id) and not self.engagement_info[human_id][2]:
                        # implement here the speech and motion
                        rospy.loginfo("HUMAN IS ENGAGED")
                        self.engagement_info[human_id][2] = True
                    else:
                        print("do nothing")

            self.loop_rate.sleep()

# robot_behaviour = rospy.get_param("/modality")
# gesture = "hello_provocative_robot"
# audio = "You...Again! I'm so annoying to see you around"
#
# robot.reproduce_motion(gesture)
# rospy.sleep(2)
# robot.reproduce_text(audio)

if __name__ == '__main__':
    rospy.init_node('engagement_demo', anonymous=True)
    node = RobotNode("en_GB")
    node.run()

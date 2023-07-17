#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import asdict, dataclass
from geometry_msgs.msg import TransformStamped
from hri_actions_msgs.msg import Intent
from hri_msgs.msg import EngagementLevel, IdsList
import json
from math import cos, sin
from math import pi as PI
import os
import pandas as pd
import rospy
import rostest
from std_msgs.msg import String
import sys
import unittest
from tf.transformations import quaternion_from_euler
import tf2_ros
from typing import Dict, List

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from script.engagement_node import BUFFER_DURATION, FOV, REFERENCE_FRAME, NODE_RATE

PKG = "hri_engagement"


@dataclass(frozen=True)
class Position:
    x: float = 0
    y: float = 0
    z: float = 0


@dataclass(frozen=True)
class Rotation:
    roll: float = 0
    pitch: float = 0
    yaw: float = 0


# human head positions and rotations relative to the REFERENCE_FRAME (robot sellion link)
HEAD_POS_FRONT = Position(x=1)
HEAD_POS_LEFT = Position(x=cos(FOV), y=sin(FOV))
HEAD_POS_RIGHT = Position(x=cos(FOV), y=sin(-FOV))
HEAD_POS_SLIGHTLY_LEFT = Position(x=cos(FOV / 4), y=sin(FOV / 4))
HEAD_POS_SLIGHTLY_RIGHT = Position(x=cos(FOV / 4), y=sin(-FOV / 4))
HEAD_ROT_FRONT = Rotation(yaw=PI)
HEAD_ROT_LEFT = Rotation(yaw=PI - FOV)
HEAD_ROT_RIGHT = Rotation(yaw=PI + FOV)
HEAD_ROT_SLIGHTLY_LEFT = Rotation(yaw=PI - FOV / 4)
HEAD_ROT_SLIGHTLY_RIGHT = Rotation(yaw=PI + FOV / 4)

# human gaze rotations relative to the human head frame
GAZE_ROT_FRONT = Rotation(roll=-PI / 2, yaw=-PI / 2)

# allowed engagement level transitions (previous level, next level)
ENG_LVL_ALLOWED_TRANS = [
    (EngagementLevel.UNKNOWN, EngagementLevel.UNKNOWN),
    (EngagementLevel.UNKNOWN, EngagementLevel.DISENGAGED),
    (EngagementLevel.DISENGAGED, EngagementLevel.DISENGAGED),
    (EngagementLevel.DISENGAGED, EngagementLevel.ENGAGING),
    (EngagementLevel.ENGAGING, EngagementLevel.ENGAGING),
    (EngagementLevel.ENGAGING, EngagementLevel.DISENGAGED),
    (EngagementLevel.ENGAGING, EngagementLevel.ENGAGED),
    (EngagementLevel.ENGAGED, EngagementLevel.ENGAGED),
    (EngagementLevel.ENGAGED, EngagementLevel.DISENGAGING),
    (EngagementLevel.DISENGAGING, EngagementLevel.DISENGAGING),
    (EngagementLevel.DISENGAGING, EngagementLevel.ENGAGED),
    (EngagementLevel.DISENGAGING, EngagementLevel.DISENGAGED),
]


@dataclass(frozen=True)
class PersonState:
    engaged: bool = True
    head_pos: Position = HEAD_POS_FRONT
    head_rot: Rotation = HEAD_ROT_FRONT
    gaze_rot: Rotation = GAZE_ROT_FRONT
    duration: float = 1.5 * BUFFER_DURATION
    movement: str = "static"


PersonData = List[PersonState]


@dataclass(frozen=True)
class TestData:
    name: str
    params: Dict


class SynthDataPublisher:
    def __init__(self, df: pd.DataFrame):
        self.run_count = 0
        self.df = df
        self.persons_ids = df.columns.get_level_values(0).unique()
        self.persons_tracked_pub = rospy.Publisher(
            "/humans/persons/tracked", IdsList, queue_size=1, latch=True
        )
        self.faces_tracked_pub = rospy.Publisher(
            "/humans/faces/tracked", IdsList, queue_size=1, latch=True
        )
        self.intents_gt_pub = rospy.Publisher(
            "/intents/gt", Intent, queue_size=len(self.persons_ids), latch=True
        )
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.face_id_pub = {}
        self.engagement_status_gt_pub = {}
        for id in self.persons_ids:
            self.face_id_pub[id] = rospy.Publisher(
                f"/humans/persons/{id}/face_id", String, queue_size=1, latch=True
            )
            self.engagement_status_gt_pub[id] = rospy.Publisher(
                f"/humans/persons/{id}/engagement_status/gt",
                EngagementLevel,
                queue_size=1,
                latch=True,
            )

    def run(self):
        df_curr = self.df.iloc[self.run_count]
        ids_tracked = [
            id for id in self.persons_ids if (df_curr[id, "tracked"] == True)
        ]
        ids_tested = [id for id in self.persons_ids if (df_curr[id, "tested"] == True)]
        ids_tracked_mgs = IdsList(ids=[str(id) for id in ids_tracked])
        self.persons_tracked_pub.publish(ids_tracked_mgs)
        self.faces_tracked_pub.publish(ids_tracked_mgs)
        for id in ids_tracked:
            df_person_state = df_curr[id]

            self.face_id_pub[id].publish(String(data=str(id)))
            tf = TransformStamped()
            time = rospy.Time.now()

            tf.header.stamp = time
            tf.header.frame_id = REFERENCE_FRAME
            tf.child_frame_id = f"face_{id}"
            tf.transform.translation.x = df_person_state["head_pos.x"]
            tf.transform.translation.y = df_person_state["head_pos.y"]
            tf.transform.translation.z = df_person_state["head_pos.z"]
            q = quaternion_from_euler(
                df_person_state["head_rot.roll"],
                df_person_state["head_rot.pitch"],
                df_person_state["head_rot.yaw"],
            )
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_br.sendTransform(tf)

            tf.header.stamp = time
            tf.header.frame_id = f"face_{id}"
            tf.child_frame_id = f"gaze_{id}"
            tf.transform.translation.x = 0.0
            tf.transform.translation.y = 0.0
            tf.transform.translation.z = 0.0
            q = quaternion_from_euler(
                df_person_state["gaze_rot.roll"],
                df_person_state["gaze_rot.pitch"],
                df_person_state["gaze_rot.yaw"],
            )
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_br.sendTransform(tf)

        for id in ids_tested:
            df_person_state = df_curr[id]
            if df_person_state["engaged"] == True:
                engagement_msg = EngagementLevel()
                engagement_msg.header.stamp = rospy.Time.now()
                engagement_msg.level = EngagementLevel.ENGAGED
                self.engagement_status_gt_pub[id].publish(engagement_msg)

                intent_msg = Intent()
                intent_msg.intent = intent_msg.ENGAGE_WITH
                intent_msg.data = f'{{"recipient":"{id}"}}'
                self.intents_gt_pub.publish(intent_msg)
            else:
                engagement_msg = EngagementLevel()
                engagement_msg.header.stamp = rospy.Time.now()
                engagement_msg.level = EngagementLevel.DISENGAGED
                self.engagement_status_gt_pub[id].publish(engagement_msg)

        self.run_count += 1
        data_exhausted = self.run_count >= len(self.df.index)
        return data_exhausted

    def close(self):
        self.persons_tracked_pub.unregister()
        self.faces_tracked_pub.unregister()
        self.intents_gt_pub.unregister()
        del self.tf_br
        for publisher in self.face_id_pub.values():
            publisher.unregister()
        for publisher in self.engagement_status_gt_pub.values():
            publisher.unregister()


class PersonUnderTest:
    def __init__(self, id: String, test_case: unittest.TestCase):
        self.id = id
        self.test_case = test_case
        self.engagement_level = EngagementLevel.UNKNOWN
        self.engagement_level_gt = None
        self.is_intent_engaged = False
        self.is_intent_engaged_gt = None
        self.engagement_status_sub = rospy.Subscriber(
            f"/humans/persons/{id}/engagement_status",
            EngagementLevel,
            lambda msg: self._on_engagement_level(msg),
        )
        self.engagement_status_gt_sub = rospy.Subscriber(
            f"/humans/persons/{id}/engagement_status/gt",
            EngagementLevel,
            lambda msg: self._on_engagement_level_gt(msg),
        )
        self.intents_sub = rospy.Subscriber(
            "/intents", Intent, lambda msg: self._on_intents(msg)
        )
        self.intents_gt_sub = rospy.Subscriber(
            "/intents/gt", Intent, lambda msg: self._on_intents_gt(msg)
        )

    def close(self):
        self.engagement_status_sub.unregister()
        self.engagement_status_gt_sub.unregister()
        self.intents_sub.unregister()
        self.intents_gt_sub.unregister()

    def _on_engagement_level(self, msg: EngagementLevel):
        self.test_case.assertIn(
            (self.engagement_level, msg.level), ENG_LVL_ALLOWED_TRANS
        )
        self.engagement_level = msg.level

    def _on_engagement_level_gt(self, msg: EngagementLevel):
        self.engagement_level_gt = msg.level

    def _on_intents(self, msg: Intent):
        data = json.loads(msg.data)
        if msg.intent == Intent.ENGAGE_WITH and data["recipient"] == str(self.id):
            self.is_intent_engaged = True

    def _on_intents_gt(self, msg: Intent):
        data = json.loads(msg.data)
        if msg.intent == Intent.ENGAGE_WITH and data["recipient"] == str(self.id):
            self.is_intent_engaged_gt = True


class EngagementTester:
    def __init__(self, test_case: unittest.TestCase):
        self.test_case = test_case
        self.persons_under_test = {}
        self.persons_tracked_sub = rospy.Subscriber(
            "/humans/persons/tracked",
            IdsList,
            lambda msg: self._on_persons_tracked(msg),
        )

    def run(self):
        for person in self.persons_under_test.values():
            if person.engagement_level_gt:
                self.test_case.assertEqual(
                    person.engagement_level, person.engagement_level_gt
                )
                person.engagement_level_gt = None
            if person.is_intent_engaged_gt:
                self.test_case.assertEqual(
                    person.is_intent_engaged, person.is_intent_engaged_gt
                )
                person.is_intent_engaged_gt = None

    def close(self):
        for person in self.persons_under_test.values():
            person.close()
        self.persons_tracked_sub.unregister()

    def _on_persons_tracked(self, msg: IdsList):
        ids_tracked_new = set(msg.ids)
        ids_tracked = self.persons_under_test.keys()
        ids_to_remove = ids_tracked - ids_tracked_new
        ids_to_add = ids_tracked_new - ids_tracked

        for id in ids_to_remove:
            self.persons_under_test[id].close()
            del self.persons_under_test[id]

        for id in ids_to_add:
            self.persons_under_test[id] = PersonUnderTest(id, self.test_case)


class GenericTestSequence(unittest.TestCase):
    def setUp(self) -> None:
        return super().setUp()

    def tearDown(self) -> None:
        persons_tracked_pub = rospy.Publisher(
            "/humans/persons/tracked", IdsList, queue_size=1, latch=True
        )
        faces_tracked_pub = rospy.Publisher(
            "/humans/faces/tracked", IdsList, queue_size=1, latch=True
        )
        persons_tracked_pub.publish(IdsList())
        faces_tracked_pub.publish(IdsList())
        rospy.sleep(rospy.Duration(2 / NODE_RATE))
        return super().tearDown()

    def _test(self, name, data):
        rospy.logdebug(f"Running test_{name}")
        synth_pub = SynthDataPublisher(self.buildPersonsData(data))
        tester = EngagementTester(self)
        data_exhausted = False
        while not (rospy.is_shutdown() or data_exhausted):
            print("Time: " + str(rospy.Time.now().to_sec()))
            data_exhausted = synth_pub.run()
            rospy.sleep(rospy.Duration(1 / NODE_RATE))
            tester.run()
        synth_pub.close()
        tester.close()

    def buildPersonsData(self, data: List[PersonData]) -> pd.DataFrame:
        df = pd.DataFrame()
        df_persons = []
        for person_data in data:
            curr_timestamp = pd.Timestamp(0)
            df_states = []
            for state in person_data:
                end_timestamp = curr_timestamp + pd.Timedelta(seconds=state.duration)
                df_state = pd.io.json.json_normalize(asdict(state)).drop(
                    columns=["duration"]
                )
                df_state.index = [end_timestamp]
                if state.movement == "static":
                    index = pd.date_range(
                        start=curr_timestamp,
                        end=end_timestamp,
                        freq=pd.DateOffset(seconds=1 / NODE_RATE),
                    )
                    df_state = df_state.reindex(index=index, method="backfill")
                # TODO: add other kind of movement, for instance adding noise and/or interpolating towards the next PersonState
                df_state["tracked"] = [True] * df_state.index.size
                start_check_timestamp = curr_timestamp + pd.Timedelta(
                    seconds=BUFFER_DURATION * 1.1
                )
                df_state["tested"] = [
                    timestamp > start_check_timestamp for timestamp in df_state.index
                ]
                df_states.append(df_state)
                curr_timestamp = end_timestamp
            df_persons.append(pd.concat(df_states))
        df = pd.concat(df_persons, axis=1, keys=range(len(data)))
        return df


class TestSequenceMeta(type):
    def __new__(mcs, name, bases, dct):
        def gen_test(name, params):
            def test(self):
                self._test(name, **params)

            return test

        test_sequence_data = dct["test_sequence_data"]
        for test in test_sequence_data:
            dct[f"test_{test.name}"] = gen_test(test.name, test.params)
        return type.__new__(mcs, name, bases, dct)


class TestSynthData(GenericTestSequence, metaclass=TestSequenceMeta):
    test_sequence_data = [
        TestData(name="one_person_in_front", params={"data": [[PersonState()]]}),
        TestData(
            name="one_person_look_slightly_left",
            params={"data": [[PersonState(head_rot=HEAD_ROT_SLIGHTLY_LEFT)]]},
        ),
        TestData(
            name="one_person_look_slightly_right",
            params={"data": [[PersonState(head_rot=HEAD_ROT_SLIGHTLY_RIGHT)]]},
        ),
        TestData(
            name="one_person_look_left",
            params={"data": [[PersonState(engaged=False, head_rot=HEAD_ROT_LEFT)]]},
        ),
        TestData(
            name="one_person_look_right",
            params={"data": [[PersonState(engaged=False, head_rot=HEAD_ROT_RIGHT)]]},
        ),
        TestData(
            name="one_person_out_fov_left",
            params={
                "data": [
                    [
                        PersonState(
                            engaged=False,
                            head_pos=HEAD_POS_LEFT,
                            head_rot=HEAD_ROT_RIGHT,
                        )
                    ]
                ]
            },
        ),
        TestData(
            name="one_person_out_fov_right",
            params={
                "data": [
                    [
                        PersonState(
                            engaged=False,
                            head_pos=HEAD_POS_RIGHT,
                            head_rot=HEAD_ROT_LEFT,
                        )
                    ]
                ]
            },
        ),
        TestData(
            name="two_people_in_front",
            params={
                "data": [
                    [
                        PersonState(
                            head_pos=HEAD_POS_SLIGHTLY_LEFT,
                            head_rot=HEAD_ROT_SLIGHTLY_RIGHT,
                        )
                    ],
                    [
                        PersonState(
                            head_pos=HEAD_POS_SLIGHTLY_RIGHT,
                            head_rot=HEAD_ROT_SLIGHTLY_LEFT,
                            duration=8,
                        )
                    ],
                ]
            },
        ),
        TestData(
            name="two_people_one_out_fov",
            params={
                "data": [
                    [PersonState()],
                    [
                        PersonState(
                            engaged=False,
                            head_pos=HEAD_POS_LEFT,
                            head_rot=HEAD_ROT_RIGHT,
                        )
                    ],
                ]
            },
        ),
        TestData(
            name="two_people_one_look_away",
            params={
                "data": [
                    [PersonState(engaged=False, head_rot=HEAD_ROT_LEFT)],
                    [
                        PersonState(
                            head_pos=HEAD_POS_SLIGHTLY_LEFT,
                            head_rot=HEAD_ROT_SLIGHTLY_RIGHT,
                        )
                    ],
                ]
            },
        ),
        TestData(
            name="one_person_swipe_look",
            params={
                "data": [
                    [
                        PersonState(engaged=False, head_rot=HEAD_ROT_LEFT),
                        PersonState(),
                        PersonState(engaged=False, head_rot=HEAD_ROT_RIGHT),
                    ]
                ]
            },
        ),
        TestData(
            name="one_person_move_across",
            params={
                "data": [
                    [
                        PersonState(
                            engaged=False,
                            head_pos=HEAD_POS_LEFT,
                            head_rot=HEAD_ROT_RIGHT,
                        ),
                        PersonState(),
                        PersonState(
                            engaged=False,
                            head_pos=HEAD_POS_RIGHT,
                            head_rot=HEAD_ROT_LEFT,
                        ),
                    ]
                ]
            },
        ),
    ]


if __name__ == "__main__":
    rospy.init_node("tester_hri_engagement")
    rostest.rosrun(PKG, "synth_data", TestSynthData)

hri_engagement
==============


Overview
--------

`hri_engagement` is a [ROS4HRI](https://wiki.ros.org/hri)-compatible 
engagement detector node.

The node estimates the engagement status of persons who are in the field 
of view of the camera.
Among those persons who are *tracked* using our [hri_face_detection](https://gitlab/ros4hri/hri_face_detect), 
it selects those who are *active*, that is, persons who are currently 
in the engagement zone (distance from the camera less than  a threshold).
For any of the active persons, it creates a Person object from which
the engagement status is computed and published on the topic: `humans/persons/<person_id>/engagement_status`.

The engagement status is defined according to the `EngagementLevel.msg` defined in our [hri_mgs](https://gitlab/ros4hri/hri_msgs/-/blob/master/msg/). Here a quick overview of the different states:
- uint8 UNKNOWN=0 # unknown: no information is provided about the engagement level
- uint8 DISENGAGED=1 # disengaged: the human has not looked in the direction of the robot
- uint8 ENGAGING=2 # engaging: the human has started to look in the direction of the robot
- uint8 ENGAGED=3 # engaged: the human is fully engaged with the robot
- uint8 DISENGAGING=4 # disengaging: the human has started to look away from the robot


Launch
------

`roslaunch hri_engagement hri_engagement.launch <parameters>`


ROS API
-------

#### Node parameters:
- `~robot_gaze_frame` (default: `/sellion_link`): set it according to your camera frame. 

### Topics

`hri_engagement` follows the ROS4HRI conventions ([REP-155](https://github.com/severin-lemaignan/rep/blob/master/rep-0155.rst)). 
In particular, it refers to the REP to know the list of the people tracked, their face_id, 

#### Subscribed topics

- `/humans/persons/tracked`
  ([hri_msgs/IdList](https://gitlab/ros4hri/hri_msgs/-/blob/master/msg/IdsList.msg)):
  the IdList of the tracked people 
- `/humans/persons/<person_id>/face_id`
  ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html)):
  the face_id corresponding to person <person_id>

#### Published topics

- `/humans/persons/<person_id>/engagement_status`
  ([hri_msgs/EngagementLevel](https://gitlab/ros4hri/hri_msgs/-/blob/master/msg/EngagementLevel.msg)):
  the IdList of the tracked people 
- `/intents`([hri_actions_msgs/Intent](https://gitlab/ros4hri/hri_actions_msgs/-/blob/main/msg/Intent.msg)): 
  the Intent ENGAGE_WITH (user is engaged with the robot)

Usage
------
Before launching the engagement detector, you need to get the following nodes compiled and running:
- `hri_face_detection` https://gitlab/ros4hri/hri_face_detect
- `hri_face_identification` https://gitlab/ros4hri/hri_face_identification
- `hri_person_manager` https://gitlab/ros4hri/hri_person_manager

Test
------
There are two tests, `test_hri_engagement_synth_data` using synthetic data and `test_hri_engagement` using
rosbags derived from live recordings.

The synthetic data are simple programmatically generated inputs and expected ground truth outputs.
Currently are generated sequences of static positions of up to two persons simultaneously.

There are two live recordings tests, one called `test_simple_hri_engagement` and the other called
`test_complex_hri_engagement`.
The first one runs a test on a few simple bags of a person in front of the camera. Those are supposed to all pass.
THe second one runs a test on more complex bags of three single persons and three multi persons (two of them in front 
of the camera). At the moment those tests do not pass and is not run by default.
 
First, we need to build the package 
- `catkin build hri_engagement` 
And then:
- `catkin test hri_engagement` 

N.B: The folder `hri_engagement_bags` contains the bags recorded in the PRISCA Lab in Naples. Each bag is a short
recorded of 7-15 sec of a person (1p) or more than one (2p) in front of the camera looking at it (eng_1 and eng_2)
or not (dis_1 and dis_2). The information regarding the bags are stored in the bag.json file.



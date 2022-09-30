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


Usage
------
Before launching the engagement detector, you need to get the following nodes compiled and running:
- `hri_face_detection` https://gitlab/ros4hri/hri_face_detect
- `hri_face_identification` https://gitlab/ros4hri/hri_face_identification
- `hri_person_manager` https://gitlab/ros4hri/hri_person_manager



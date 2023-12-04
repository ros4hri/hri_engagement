hri_engagement
==============


Overview
--------

`hri_engagement` is a [ROS4HRI](https://wiki.ros.org/hri)-compatible 
engagement detector node.

The node estimates the engagement status of persons who are in the field of view
of the camera, using the 'visual social engagement' metric defined in
["Measuring Visual Social Engagement from Proxemics and
Gaze"](https://doi.org/10.1109/RO-MAN53752.2022.9900801) (by Webb and
Lemaignan).  This metric combines the distance between the robot and the person,
with a measure of mutual gazing (whether the person and the robot are looking at
each other).


For each of the whose faces are detected by the robot (ie, in
`/humans/faces/tracked`), it performs temporal filtering of this metric
(over a 2sec window) and then publishes the resulting level of engagement on the
topic: `humans/persons/<person_id>/engagement_status`.

The engagement status is defined according to the `EngagementLevel.msg` defined
in
[hri_mgs](https://github.com/ros4hri/hri_msgs/blob/master/msg/EngagementLevel.msg).
Here a quick overview of the different states:

- `UNKNOWN`: no information is provided about the engagement level
- `DISENGAGED`: the human has not looked in the direction of the robot
- `ENGAGING`: the human has started to look in the direction of the robot
- `ENGAGED`: the human is fully engaged with the robot
- `DISENGAGING`: the human has started to look away from the robot


Launch
------

`roslaunch hri_engagement monitor.launch <parameters>`


ROS API
-------

#### Node parameters:

- `~reference_frame` (default: `/sellion_link`): the robot's reference point,
  used to compute the distance and mutual gaze. Should be aligned with the
  robot's gaze direction
- `~max_distance` (default: `4`m): people further away than `max_distance` are
  automatically set to be `DISENGAGED`
- `~field_of_view` (default: `60` degrees): field of 'attention' of both the
  humans and the robot, in degrees. If eg the robot is outside of the field of
  attention of the human, this human will be considered as dis-engaged, as it
  does not look at the robot.
- `~engagement_threshold` (default: `0.55`): used to decide 'when' a person is
  considered as engaging (or disengaging). Increase this value to make it
  'harder' for the robot to consider someone as 'engaging', decrease it for the
  opposite effect.

### Topics

`hri_engagement` follows the ROS4HRI conventions ([REP-155](https://www.ros.org/reps/rep-0155.html)). 
In particular, it uses the conventions (topics, naming) defined in the REP to access the list of the people tracked.

#### Subscribed topics

- `/humans/persons/tracked`
  ([hri_msgs/IdList](https://gitlab/ros4hri/hri_msgs/-/blob/master/msg/IdsList.msg)):
  the IdList of the tracked people 
- `/humans/persons/<person_id>/face_id`
  ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html)):
  the face_id corresponding to person <person_id>

#### Published topics

- `/humans/persons/<person_id>/engagement_status`
  ([hri_msgs/EngagementLevel](https://github.com/ros4hri/hri_msgs/blob/master/msg/EngagementLevel.msg)):
  the level of engagement of each tracked person
- `/intents`([hri_actions_msgs/Intent](https://github.com/ros4hri/hri_actions_msgs/blob/main/msg/Intent.msg)): 
  publishes an `ENGAGE_WITH` intent when a user is engaged with the robot.

Usage
------

Before launching the engagement detector, you need to get the following nodes
compiled and running:

- [`hri_face_detect`](https://github.com/ros4hri/hri_face_detect/)
- [`hri_face_identification`](https://gitlab/ros4hri/hri_face_identification)
- [`hri_person_manager`](https://github.com/ros4hri/hri_person_manager)

Test
------

### Testing with a webcam

You can test `hri_engagement` with your webcam:

1. start `usb_cam`: `rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/image`
2. By default, `usb_cam` publish its images in the `head_camera` optical frame,
   with `Z` forward. `hri_engagement` expect a reference frame (by default named
   `sellion_link` with `X` forward: `rosrun tf2_ros static_transform_publisher 0
   0 0 0 1.52 0 sellion_link head_camera`
3. start `hri_face_detect`: `rosrun hri_face_detect detect`
4. start `hri_person_manager`: `rosrun hri_person_manager hri_person_manager`
5. finally, start `hri_engagement`: `rosrun hri_engagement engagement_monitor
   _reference_frame:=sellion_link`

If you configure the log level of `hri_engagement` to `DEBUG` (using eg
`rqt_console`), you can see the exact values being computed.

### Unit-tests

There are two tests, `test_hri_engagement_synth_data` using synthetic data and
`test_hri_engagement` using rosbags derived from live recordings.

The synthetic data are simple programmatically generated inputs and expected
ground truth outputs.  Currently are generated sequences of static positions of
up to two persons simultaneously.

There are two live recordings tests, one called `test_simple_hri_engagement` and
the other called `test_complex_hri_engagement`.  The first one runs a test on a
few simple bags of a person in front of the camera. Those are supposed to all
pass.  THe second one runs a test on more complex bags of three single persons
and three multi persons (two of them in front of the camera). At the moment
those tests do not pass and is not run by default.
 
First, we need to build the package 
- `catkin build hri_engagement` And then:
- `catkin test hri_engagement` 

N.B: The folder `hri_engagement_bags` contains the bags recorded in the PRISCA
Lab in Naples. Each bag is a short recorded of 7-15 sec of a person (1p) or more
than one (2p) in front of the camera looking at it (eng_1 and eng_2) or not
(dis_1 and dis_2). The information regarding the bags are stored in the bag.json
file.



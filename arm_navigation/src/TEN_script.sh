#!/bin/bash

rostopic pub -1 /mac_arm/rail/cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['rail']
position: [0.0]
velocity: [0]
effort: [0]"

# copy first letter to where parser wants it
cp ../config/desired_structures/{t,scripted}.txt
# run parser - reads (scripted).txt and creates (center_coords).yaml
python core.py
# run navigation # assumes block positions are in the world frame. reads from yaml.
rosrun arm_navigation navigator.py
#move rail to next position
rostopic pub -1 /mac_arm/rail/cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['rail']
position: [0.2]
velocity: [0]
effort: [0]"

# copy second letter to where the parser wants it
# run parser
# run navigation
#move rail to last position
rostopic pub -1 /mac_arm/rail/cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['rail']
position: [0.4]
velocity: [0]
effort: [0]"

# copy last letter to where the parser expects it
# run parser
# run navigation
#move rail to home position
rostopic pub -1 /mac_arm/rail/cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['rail']
position: [0.0]
velocity: [0]
effort: [0]"

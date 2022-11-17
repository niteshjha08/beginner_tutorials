# ROS2 beginner tutorials
This repository contains [beginner tutorials](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html) for publishers, subscribers, services, launch files and logging using ROS2 Humble Hawksbill

## Dependencies
ROS2 Humble Hawskbill
Ubuntu 20.04

## Steps to compile
- Create ROS workspace as mentioned [here](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- cd to ros2_ws/src
- git clone git@github.com:niteshjha08/beginner_tutorials.git
- cd ..
- colcon build --packages-select cpp_pubsub
- source ros2_ws/install/local_setup.bash

## Run publisher and subscriber
`ros2 run cpp_pubsub talker` : publisher

`ros2 run cpp_pubsub listener` : subscriber

## Services and Logging
1) Run publisher and subscriber separately as before 
`ros2 run cpp_pubsub talker` : publisher

`ros2 run cpp_pubsub listener` : subscriber

2) OR, use launch file to run them: 
`ros2 launch cpp_pubsub comms_launch.yaml count:=1` where 1 is the starting value of the publish message.

3) Call service to change the text being published with:
`ros2 service call /modify_string cpp_pubsub/srv/ModifyString "{a: new_string}" `
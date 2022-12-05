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

## TF2, ROS bag record
1) Launch talker, listener, and optionally the ros2 bag recorded as follows (use record:=1 to record all topics/ record:=0 to not record):
`ros2 launch ros2 launch cpp_pubsub start_comms.py record:=1`

#### Results of tf2 testing:
a) `ros2 run tf2_ros tf2_echo world talk`

![tf2_echo]()

b) `ros2 run tf2_tools view_frames`

![view_frame]()

## Testing 
### Using colcon test
`colcon test --event-handlers console_direct+ --packages-select cpp_pubsub`

![colcon_test]()
`ros2 run cpp_pubsub pub_test`

![pub_test]()
## Results
### RQT Console
![logs](https://github.com/niteshjha08/beginner_tutorials/blob/Week10_HW/media/logs.png)

### CLI logs
![cli_logs](https://github.com/niteshjha08/beginner_tutorials/blob/Week10_HW/media/cli_log.png)

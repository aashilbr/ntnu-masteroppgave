#!/bin/bash

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash
catkin build

roslaunch --wait gazebo_ros empty_world.launch

sleep 1d
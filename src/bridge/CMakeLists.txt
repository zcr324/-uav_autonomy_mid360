cmake_minimum_required(VERSION 3.0.2)
project(bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  quadrotor_msgs
)

catkin_package()

# pure-python package: scripts live in scripts/

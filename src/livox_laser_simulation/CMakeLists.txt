cmake_minimum_required(VERSION 3.0.2)
project(livox_laser_simulation)

## Compile as C++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  pcl_conversions
  tf
  livox_ros_driver2
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    pcl_conversions
    tf
    livox_ros_driver2
  DEPENDS
    GAZEBO
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions(
  ${GAZEBO_CXX_FLAGS}
  ${PCL_DEFINITIONS}
)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${PCL_LIBRARIES}
  RayPlugin
)

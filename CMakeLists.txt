cmake_minimum_required(VERSION 2.8.3)
project(robot_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  vision_msgs
  message_generation
)

add_message_files(
  FILES
  Sensors.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  vision_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frontier_detector
#  CATKIN_DEPENDS cvbridge geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime)



cmake_minimum_required(VERSION 2.8.3)
project(autonet18p_Pi_sos)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries 
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf2
    tf
    tf2_ros
    nav_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs geometry_msgs sensor_msgs std_msgs 
  #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


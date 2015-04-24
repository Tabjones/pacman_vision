cmake_minimum_required(VERSION 2.8.3)
project(pacman_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  asus_scanner_models
  calibration
  dynamic_reconfigure
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  tf
  pacman_vision_comm
)

find_package(Boost REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
## Add C++11 
add_definitions(-std=c++11)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vision_node
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs pcl_conversions pcl_ros roscpp roslib sensor_msgs tf pacman_vision_comm
  DEPENDS pcl boost
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pcl_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${boost_LIBRARY_DIRS}
  )
add_definitions(${PCL_DEFINITIONS})

add_executable(pacman_vision src/vision_node.cpp 
                             src/scene_processing.cpp)

add_dependencies(pacman_vision pacman_vision_comm_generate_messages_cpp)

target_link_libraries(pacman_vision
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${boost_LIBRARIES}
)

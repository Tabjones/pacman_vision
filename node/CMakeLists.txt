cmake_minimum_required(VERSION 2.8.3)
project(pacman_vision)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(freenect2 REQUIRED)

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

find_package(PCL 1.7.2 REQUIRED)
find_package(HDF5 REQUIRED)

include_directories(BEFORE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find pel in system
find_package(pel 0.6 REQUIRED)

######################
## catkin generated ##
######################
generate_dynamic_reconfigure_options( cfg/pacman_vision.cfg )

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
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs pcl_conversions pcl_ros roscpp roslib sensor_msgs tf pacman_vision_comm message_runtime
  DEPENDS PCL Boost pel freenect2
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${pel_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${pel_LIBRARY_DIRS}
  )

add_executable(pacman_vision src/vision_node.cpp
                             src/kinect2_processor.cpp
                             src/estimator.cpp
                             src/broadcaster.cpp
                             src/tracker.cpp
                             src/utility.cpp
                             src/vito_listener.cpp
                             src/storage.cpp
                             src/supervoxels.cpp)

add_dependencies(pacman_vision
  pacman_vision_comm_generate_messages_cpp
  pacman_vision_gencfg
  )

target_link_libraries(pacman_vision
   ${freenect2_LIBRARY}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${pel_LIBRARIES}
)

##POST BUILD COMMAND to fetch database
set(wget_arg "-OpelDB.tar.gz")
set(wget_url "https://www.dropbox.com/s/79vy38432vv80gu/PELdatabase.tar.gz?dl=0")
set(tar_arg "-zxvf")
set(tar_tgt "pelDB.tar.gz")
set(rm_comm "pelDB.tar.gz")
set(rm_arg "-rf")
set(rm_db "database")
add_custom_command(TARGET pacman_vision
  POST_BUILD
  COMMAND rm ${rm_db} ${rm_arg}
  COMMAND wget ${wget_arg} ${wget_url}
  COMMAND tar ${tar_arg} ${tar_tgt}
  COMMAND rm ${rm_comm}
  WORKING_DIRECTORY ${pacman_vision_SOURCE_DIR}
  )


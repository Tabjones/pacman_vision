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
find_package(HDF5 REQUIRED)

include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(-std=c++11 ${PCL_DEFINITIONS})

set(PEL_LW_NAME "pel")
set(PEL_BUILD "${CMAKE_BINARY_DIR}/${PEL_LW_NAME}")
set(PEL_DIR ${PEL_BUILD})

#find PEL in system
find_package(PEL 0.6)

if (NOT PEL_FOUND)
  include(ExternalProject)
  set(PEL_SRC "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/${PEL_LW_NAME}")

  MESSAGE(STATUS "Pose Estimation Library (PEL) is NOT FOUND in system, building it...")
  MESSAGE(STATUS ${LIBRARY_OUTPUT_PATH})

  ExternalProject_Add(
    Pel
    SOURCE_DIR ${PEL_SRC}
    BINARY_DIR ${PEL_BUILD}
    BUILD_IN_SOURCE 0
    CONFIGURE_COMMAND ${CMAKE_COMMAND} "${PEL_SRC}/ --prefix=${CATKIN_DEVEL_PREFIX}" TODO
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    BUILD_COMMAND make
    INSTALL_DIR ${CATKIN_DEVEL_PREFIX}
    INSTALL_COMMAND make install
    )

  add_custom_command(TARGET Pel
      POST_BUILD
      COMMAND find_package(PEL 0.6))
endif(NOT PEL_FOUND)

#find_package(PEL 0.6 REQUIRED)

######################
## catkin generated ##
######################
generate_dynamic_reconfigure_options( cfg/pacman_vision.cfg )

##generate_messages(
##  dynamic_reconfigure
##  )

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
  DEPENDS PCL Boost
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${PEL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${boost_LIBRARY_DIRS}
  ${CATKIN_DEVEL_PREFIX}/lib
  ${PEL_LIBRARY_DIRS}
  )

add_executable(pacman_vision src/vision_node.cpp
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
  Pel)

target_link_libraries(pacman_vision
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${boost_LIBRARIES}
   ${PEL_LIBRARIES}
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


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

set(SOURCES
  src/vision_node.cpp
  src/broadcaster.cpp
  src/utility.cpp
  src/vito_listener.cpp
  src/storage.cpp
  src/supervoxels.cpp
  )

set(EXT_DEPS
  PCL
  )

  ###################
## Find libfreenect2 ##
find_package(freenect2)
IF(freenect2_FOUND)
  set(PACMAN_VISION_WITH_KINECT2_SUPPORT 1)
  include_directories(${freenect2_INCLUDE_DIRS})
  LIST(APPEND SOURCES
    src/kinect2_processor.cpp
    )
  LIST(APPEND LIBRARIES
    ${freenect2_LIBRARY}
    )
  LIST(APPEND EXT_DEPS
    freenect2)
ELSE(freenect2_FOUND)
  message(STATUS "=======================================================")
  message(STATUS "== Libreenect2 NOT found, disabling kinect2 support. ==")
  message(STATUS "=======================================================")
ENDIF(freenect2_FOUND)

  ###################
## Find pel in system ##
find_package(pel 0.6)
IF(pel_FOUND)
  set(PACMAN_VISION_WITH_PEL_SUPPORT 1)
  include_directories(${pel_INCLUDE_DIRS})
  LIST(APPEND SOURCES
    src/estimator.cpp
    src/tracker.cpp
    )
  LIST(APPEND LIBRARIES
    ${pel_LIBRARIES}
    )
  LIST(APPEND LINK_DIR
    ${pel_LIBRARY_DIRS}
    )
  LIST(APPEND EXT_DEPS
    pel)
ELSE(pel_FOUND)
  message(STATUS "=============================================================")
  message(STATUS "== Pel NOT found, disabling estimator and tracker modules. ==")
  message(STATUS "=============================================================")
ENDIF(pel_FOUND)

find_package(PCL 1.7.2 REQUIRED)
find_package(HDF5 REQUIRED)

include_directories(BEFORE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

####### Config File
configure_file("${PROJECT_SOURCE_DIR}/cmake/config_h.cmake.in"
  "${CATKIN_DEVEL_PREFIX}/include/pacman_vision/config.h")
include_directories(${CATKIN_DEVEL_PREFIX}/include)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
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

##TODO##Add cmake created cfg for ros dynamic reconfigure?
###Add same thing for package.xml

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
  CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  tf
  pacman_vision_comm
  message_runtime
  DEPENDS ${EXT_DEPS}
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${LINK_DIR}
  )

add_executable(pacman_vision ${SOURCES})

add_dependencies(pacman_vision
  pacman_vision_comm_generate_messages_cpp
  pacman_vision_gencfg
  )

target_link_libraries(pacman_vision
   ${LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
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


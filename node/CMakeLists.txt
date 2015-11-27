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

##Include a FindVCG if needed
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

###########################
##Control build behaviour##
################################################

## This will build estimator and tracker modules
set (PACMAN_VISION_BUILD_RECOGNITION        1)
## This will build vito listener module
set (PACMAN_VISION_BUILD_LISTENER           1)
## This will build supervoxels module
set (PACMAN_VISION_BUILD_SUPERVOXELS        1)
## This will build in hand modeler module
set (PACMAN_VISION_BUILD_MODELER            1)
## This will build Kinect2 Processor
set (PACMAN_VISION_BUILD_KINECT2_PROCESSOR  1)

################################################
## Set default sources
set(SOURCES
  src/vision_node.cpp
  src/node_utils.cpp
  src/utility.cpp
  src/storage.cpp
  src/broadcaster.cpp
  )


IF(PACMAN_VISION_BUILD_MODELER)
    LIST(APPEND SOURCES
        src/in_hand_modeler.cpp
    )
ENDIF(PACMAN_VISION_BUILD_MODELER)

IF(PACMAN_VISION_BUILD_LISTENER)
    LIST(APPEND SOURCES
        src/vito_listener.cpp
    )
ENDIF(PACMAN_VISION_BUILD_LISTENER)

IF(PACMAN_VISION_BUILD_SUPERVOXELS)
    LIST(APPEND SOURCES
        src/supervoxels.cpp
    )
ENDIF(PACMAN_VISION_BUILD_SUPERVOXELS)

##Check extra deps
set(EXT_DEPS
  PCL
  )

IF(PACMAN_VISION_BUILD_KINECT2_PROCESSOR)
    #######################
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
        unset(PACMAN_VISION_WITH_KINECT2_SUPPORT)
        unset(PACMAN_VISION_BUILD_KINECT2_PROCESSOR)
        message(WARNING "=========================================================")
        message(WARNING "== Libreenect2 NOT found, disabling kinect2 processor. ==")
        message(WARNING "=========================================================")
    ENDIF(freenect2_FOUND)
ENDIF(PACMAN_VISION_BUILD_KINECT2_PROCESSOR)

IF(PACMAN_VISION_BUILD_RECOGNITION)
    ########################
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
        unset(PACMAN_VISION_WITH_PEL_SUPPORT)
        unset(PACMAN_VISION_BUILD_RECOGNITION)
        message(WARNING "=============================================================")
        message(WARNING "== Pel NOT found, disabling estimator and tracker modules. ==")
        message(WARNING "=============================================================")
    ENDIF(pel_FOUND)
ENDIF(PACMAN_VISION_BUILD_RECOGNITION)

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

#############################################
## catkin generated for Dynamic Reconfigure##
#############################################
IF(PACMAN_VISION_WITH_KINECT2_SUPPORT)
  IF(PACMAN_VISION_WITH_PEL_SUPPORT)
    generate_dynamic_reconfigure_options( cfg/pacman_vision.complete.cfg )
  ELSE(PACMAN_VISION_WITH_PEL_SUPPORT)
    generate_dynamic_reconfigure_options( cfg/pacman_vision.nopel.cfg )
  ENDIF(PACMAN_VISION_WITH_PEL_SUPPORT)
ELSE(PACMAN_VISION_WITH_KINECT2_SUPPORT)
  IF(PACMAN_VISION_WITH_PEL_SUPPORT)
    generate_dynamic_reconfigure_options( cfg/pacman_vision.nokinect2.cfg )
  ELSE(PACMAN_VISION_WITH_PEL_SUPPORT)
    generate_dynamic_reconfigure_options( cfg/pacman_vision.barebone.cfg )
  ENDIF(PACMAN_VISION_WITH_PEL_SUPPORT)
ENDIF(PACMAN_VISION_WITH_KINECT2_SUPPORT)

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

IF(PACMAN_VISION_BUILD_RECOGNITION)
    ##POST BUILD COMMAND to fetch PEL database
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
ENDIF(PACMAN_VISION_BUILD_RECOGNITION)

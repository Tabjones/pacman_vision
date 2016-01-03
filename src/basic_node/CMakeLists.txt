################################################################################
######### Set the node sources #################################################
################################################################################
set(SRC
    basic_node.cpp
    sensor_processor.cpp
    ../../include/basic_node/basic_node_config.hpp
    ../../include/basic_node/sensor_config.hpp
  )
################################################################################
###### If told to build Kinect2 Processor, Check if we can #####################
################################################################################
IF(PACV_BUILD_KINECT2)
    ## Find libfreenect2 ##
    find_package(freenect2)
    IF(freenect2_FOUND)
        #Yes we can!
        set(PACV_KINECT2_SUPPORT 1 PARENT_SCOPE)
        LIST(APPEND SRC
            kinect2_processor.cpp)
        LIST(APPEND FNECT2
            ${MORE_LIBRARIES}
            ${freenect2_LIBRARY})
        LIST(APPEND FNECT2_INC
            ${MORE_INCLUDE_DIRS}
            ${freenect2_INCLUDE_DIRS})
        #propagate these vars upstream
        set(MORE_LIBRARIES ${FNECT2} PARENT_SCOPE)
        set(MORE_INCLUDE_DIRS ${FNECT2_INC} PARENT_SCOPE)
    ELSE(freenect2_FOUND)
        set(PACV_KINECT2_SUPPORT 0 PARENT_SCOPE)
        message(WARNING "==============================================================")
        message(WARNING "== Libreenect2 NOT found, disabling kinect2 processor.      ==")
        message(WARNING "== set PACV_BUILD_KINECT2 to false, or install Libreenect2! ==")
        message(WARNING "==============================================================")
    ENDIF(freenect2_FOUND)
ENDIF(PACV_BUILD_KINECT2)

#set library name
set(PACV_BASICNODE_LIB pacv_libbasicnode PARENT_SCOPE)

add_library(pacv_libbasicnode SHARED
    ${SRC})

add_dependencies(pacv_libbasicnode
  pacman_vision_comm_generate_messages_cpp
  )

target_link_libraries(pacv_libbasicnode
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PACV_COMMON_LIB}
    ${Boost_LIBRARIES})

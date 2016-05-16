# Software License Agreement (BSD License)
#
#   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
#   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
#   All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder(s) nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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


install(TARGETS pacv_libbasicnode
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)

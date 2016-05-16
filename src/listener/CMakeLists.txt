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
######### Set sources #################################################
################################################################################
set(SRC
    listener.cpp
    ../../include/listener/listener_config.hpp
    # ../../include/listener/vito_geometry.h
  )

#set library name
set(PACV_LISTENER_LIB pacv_liblistener PARENT_SCOPE)

set(PACV_LISTENER_SUPPORT 1 PARENT_SCOPE)

    add_library(pacv_liblistener SHARED
        ${SRC})

    add_dependencies(pacv_liblistener
        pacman_vision_comm_generate_messages_cpp
    )

target_link_libraries(pacv_liblistener
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PACV_COMMON_LIB}
    )

install(TARGETS pacv_liblistener
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)

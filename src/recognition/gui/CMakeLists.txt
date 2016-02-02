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
######### Set the gui sources #################################################
################################################################################
set(SRC
    estimator_gui.cpp
    tracker_gui.cpp
  )

QT4_ADD_RESOURCES(QSRC ../../../resources/resources.qrc)
QT4_WRAP_UI(QSRC estimator_gui.ui)
QT4_WRAP_UI(QSRC tracker_gui.ui)

#set library name
set(PACV_RECOGNITIONGUI_LIB pacv_librecognitiongui PARENT_SCOPE)

IF(PACV_RECOGNITION_SUPPORT)
add_library(pacv_librecognitiongui SHARED
    ${SRC}
    ${QSRC}
    )

target_link_libraries(pacv_librecognitiongui
    # ${catkin_LIBRARIES}
    # ${PACV_COMMON_LIB}
    ${PACV_RECOGNITION_LIB}
    Qt4::QtGui
    Qt4::QtXml
    )
ENDIF(PACV_RECOGNITION_SUPPORT)

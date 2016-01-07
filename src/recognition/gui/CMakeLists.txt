################################################################################
######### Set the gui sources #################################################
################################################################################
set(SRC
    estimator_gui.cpp
    #tracker_gui.cpp
  )

QT4_ADD_RESOURCES(QSRC ../../../resources/resources.qrc)
QT4_WRAP_UI(QSRC estimator_gui.ui)
# QT4_WRAP_UI(QSRC tracker_gui.ui)

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

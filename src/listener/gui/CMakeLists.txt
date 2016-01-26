################################################################################
######### Set the gui sources #################################################
################################################################################
set(SRC
    listener_gui.cpp
  )

QT4_ADD_RESOURCES(QSRC ../../../resources/resources.qrc)
QT4_WRAP_UI(QSRC listener_gui.ui)

#set library name
set(PACV_LISTENERGUI_LIB pacv_liblistenergui PARENT_SCOPE)

IF(PACV_LISTENER_SUPPORT)
add_library(pacv_liblistenergui SHARED
    ${SRC}
    ${QSRC}
    )

target_link_libraries(pacv_liblistenergui
    # ${catkin_LIBRARIES}
    # ${PACV_COMMON_LIB}
    ${PACV_LISTENER_LIB}
    Qt4::QtGui
    Qt4::QtXml
    )
ENDIF(PACV_LISTENER_SUPPORT)

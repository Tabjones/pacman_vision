################################################################################
######### Set the gui sources #################################################
################################################################################
set(SRC
    basic_node_gui.cpp
  )

QT4_ADD_RESOURCES(QSRC ../../../resources/resources.qrc)
QT4_WRAP_UI(QSRC basic_node_gui.ui)

#set library name
set(PACV_BASICGUI_LIB pacv_libbasicgui PARENT_SCOPE)

add_library(pacv_libbasicgui SHARED
    ${SRC}
    ${QSRC}
    )

target_link_libraries(pacv_libbasicgui
    ${catkin_LIBRARIES}
    ${PACV_COMMON_LIB}
    ${PACV_BASICNODE_LIB}
    Qt4::QtGui
    Qt4::QtXml
    )

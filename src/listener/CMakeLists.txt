################################################################################
######### Set sources #################################################
################################################################################
set(SRC
    listener.cpp
    ../../include/listener/listener_config.hpp
  )

#set library name
set(PACV_LISTENER_LIB pacv_liblistener PARENT_SCOPE)

IF(PACV_LISTENER_SUPPORT)

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

ENDIF(PACV_LISTENER_SUPPORT)

################################################################################
######### Set sources #################################################
################################################################################
set(SRC
    estimator.cpp
    ../../include/recognition/estimator_config.hpp
  )
########################
## Find pel in system ##
find_package(pel 0.6)
IF(pel_FOUND)
    set(PACV_RECOGNITION_SUPPORT 1 PARENT_SCOPE)
    set(PACV_RECOGNITION_SUPPORT 1 )
    include_directories(${pel_INCLUDE_DIRS})
    link_directories(${pel_LIBRARY_DIRS})
    LIST(APPEND PELLIB
        ${pel_LIBRARIES}
        ${MORE_LIBRARIES}
        )
    LIST(APPEND PELINC
        ${MORE_INCLUDE_DIRS}
        ${pel_INCLUDE_DIRS}
        )
    LIST(APPEND PELDIR
        ${MORE_LINK_DIRS}
        ${pel_LIBRARY_DIRS}
        )
    set(MORE_LIBRARIES ${PELLIB} PARENT_SCOPE)
    set(MORE_INCLUDE_DIRS ${PELINC} PARENT_SCOPE)
    set(MORE_LINK_DIRS ${PELDIR} PARENT_SCOPE)
ELSE(pel_FOUND)
    set(PACV_RECOGNITION_SUPPORT 0 PARENT_SCOPE)
    set(PACV_RECOGNITION_SUPPORT 0 )
    message(WARNING "=============================================================")
    message(WARNING "== Pel NOT found, disabling estimator and tracker modules. ==")
    message(WARNING "=============================================================")
ENDIF(pel_FOUND)

#set library name
set(PACV_RECOGNITION_LIB pacv_librecognition PARENT_SCOPE)

IF(PACV_RECOGNITION_SUPPORT)

    add_library(pacv_librecognition SHARED
        ${SRC})

    add_dependencies(pacv_librecognition
        pacman_vision_comm_generate_messages_cpp
    )

target_link_libraries(pacv_librecognition
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_SEARCH_LIBRARIES}
    ${PACV_COMMON_LIB}
    ${Boost_LIBRARIES}
    ${PEL_LIBRARIES})

     ##POST BUILD COMMAND to fetch PEL database
     set(wget_arg "-OpelDB.tar.gz")
     set(wget_url "https://www.dropbox.com/s/79vy38432vv80gu/PELdatabase.tar.gz?dl=0")
     set(tar_arg "-zxvf")
     set(tar_tgt "pelDB.tar.gz")
     set(rm_comm "pelDB.tar.gz")
     set(rm_arg "-rf")
     set(rm_db "database")
     add_custom_command(TARGET pacv_librecognition
         POST_BUILD
         COMMAND rm ${rm_db} ${rm_arg}
         COMMAND wget ${wget_arg} ${wget_url}
         COMMAND tar ${tar_arg} ${tar_tgt}
         COMMAND rm ${rm_comm}
         WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
         )
 ENDIF(PACV_RECOGNITION_SUPPORT)

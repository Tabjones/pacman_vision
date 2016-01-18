################################################
## Set common sources
set(SRC
  common_ros.cpp
  common_pcl.cpp
  storage.cpp
  box.cpp
  ../../include/common/dynamic_modules.hpp
  ../../include/common/modules_config.hpp
  ../../include/common/random_generation.hpp
  )

#set library name
set(PACV_COMMON_LIB pacv_libcommon PARENT_SCOPE)

add_library(pacv_libcommon SHARED
    ${SRC})

target_link_libraries(pacv_libcommon
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES})

######## Find the VCG Library
find_path(VCG_DIR "vcg/complex/complex.h"
        HINTS "${VCG_ROOT}" "$ENV{VCG_ROOT}"
        PATHS "$ENV{HOME}" "$ENV{HOME}/Code" "/usr" "/usr/local" "/usr/share" "/usr/local/share"
        PATH_SUFFIXES "vcg" "vcglib" "include"
        DOC "Root directory of VCG library")

####### Report Found or not
if(EXISTS "${VCG_DIR}" AND NOT "${VCG_DIR}" STREQUAL "")
  set(VCG_FOUND TRUE)
  set(VCG_INCLUDE_DIRS ${VCG_DIR})
  set(VCG_INCLUDR_DIR ${VCG_DIR})
  message (STATUS "VCG found on: ${VCG_INCLUDE_DIRS}")
else()
  package_report_not_found(VCG "Please specify VCG directory with VCG_ROOT env. variable")
endif()

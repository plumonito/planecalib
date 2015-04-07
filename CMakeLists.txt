cmake_minimum_required(VERSION 3.0)

project(planecalib CXX C)

#add_subdirectory(3rdparty)

if(MSVC)
    set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release)
    set(CMAKE_MAP_IMPORTED_CONFIG Release)
endif()

find_package(freeglut REQUIRED QUIET PATHS "${EP_PREFIX}")
find_package(glew REQUIRED QUIET PATHS "${EP_PREFIX}")
find_package(OpenCV REQUIRED QUIET PATHS "${EP_PREFIX}")
find_package(gflags REQUIRED PATHS "${EP_PREFIX}")
find_package(glog REQUIRED PATHS "${EP_PREFIX}")

if(WIN32)
    set(SuiteSparse_USE_LAPACK_BLAS ON)
endif()
find_package(SuiteSparse REQUIRED PATHS "${EP_PREFIX}")

find_package(Ceres REQUIRED QUIET PATHS "${EP_PREFIX}")

add_definitions(-DGLEW_STATIC=1 -DFREEGLUT_STATIC=1
    -DFREEGLUT_LIB_PRAGMAS=0 -DGLOG_NO_ABBREVIATED_SEVERITIES=1)

#Eigen
add_library(eigen INTERFACE)
#set(EIGEN_INCLUDE_DIR ${EP_PREFIX}/include/eigen3)
target_include_directories(eigen INTERFACE ${EIGEN_INCLUDE_DIR})

#  matio -------------------------------------------------------------------
find_library(MATIO_LIB matio)
find_library(MATIO_DLL libmatio.dll)

find_path(MATIO_INCLUDE_DIRS1 NAMES matio.h)
find_path(MATIO_INCLUDE_DIRS2 NAMES matio_pubconf.h)

set(MATIO_INCLUDE_DIRS ${MATIO_INCLUDE_DIRS1})
list(APPEND MATIO_INCLUDE_DIRS ${MATIO_INCLUDE_DIRS2})

add_library(matio SHARED IMPORTED GLOBAL)

set_property(TARGET matio APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
  "${MATIO_INCLUDE_DIRS}"
  )

set_property(TARGET matio APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(matio PROPERTIES
  IMPORTED_IMPLIB_DEBUG ${MATIO_LIB}
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG ""
  IMPORTED_LOCATION_DEBUG ${MATIO_DLL}
  )
set_property(TARGET matio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(matio PROPERTIES
  IMPORTED_IMPLIB_RELEASE ${MATIO_LIB}
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE ${MATIO_DLL}
  )

list(APPEND RELEASE_DLL_GLOB_PATTERNS ${MATIO_DLL})
list(APPEND DEBUG_DLL_GLOB_PATTERNS ${MATIO_DLL})

#Ceres
include_directories("${Ceres_DIR}/../include")
	
add_subdirectory(code)

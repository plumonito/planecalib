cmake_minimum_required(VERSION 3.0)

project(planecalib CXX C)

add_subdirectory(3rdparty)

if(MSVC)
    set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release)
    set(CMAKE_MAP_IMPORTED_CONFIG Release)
endif()

find_package(freeglut REQUIRED QUIET PATHS "${EP_PREFIX}")
find_package(glew REQUIRED QUIET PATHS "${EP_PREFIX}")
find_package(OpenCV REQUIRED QUIET PATHS "${EP_PREFIX}")
# find_package(gflags REQUIRED PATHS "${EP_PREFIX}")
find_package(glog REQUIRED PATHS "${EP_PREFIX}")

if(WIN32)
    set(SuiteSparse_USE_LAPACK_BLAS ON)
endif()
find_package(SuiteSparse REQUIRED PATHS "${EP_PREFIX}")
if(WIN32)
	include(${USE_SuiteSparse})
endif()

find_package(Ceres REQUIRED QUIET PATHS "${EP_PREFIX}")

#add_definitions(-DGLEW_STATIC=1 -DFREEGLUT_STATIC=1
#   -DFREEGLUT_LIB_PRAGMAS=0 -DGLOG_NO_ABBREVIATED_SEVERITIES=1)
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES=1)

#Eigen
#add_library(eigen INTERFACE)
#set(EIGEN_INCLUDE_DIR ${EP_PREFIX}/include/eigen3)
#target_include_directories(eigen INTERFACE ${EIGEN_INCLUDE_DIR})

find_package(matio PATHS "3rdparty_new/matio_cmake")
if(matio_FOUND)
    list(APPEND RELEASE_DLL_GLOB_PATTERNS ${MATIO_DLL})
    list(APPEND DEBUG_DLL_GLOB_PATTERNS ${MATIO_DLL})
    add_definitions(-DHAS_MATIO)
else()
    message(STATUS "matio not found. Disabling matlab interaction.")
endif()

#Ceres
#include_directories("${Ceres_DIR}/../include")
	
if(WIN32)
	add_definitions( "/wd4251" )
endif()

add_subdirectory(code)

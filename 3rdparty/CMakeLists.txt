cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

include(config_external_project)

set(FREEGLUT_ROOT_DIR freeglut-2.8.1)
set(GLEW_ROOT_DIR glew-1.11.0)
set(EIGEN_ROOT_DIR eigen-3.2.2)
set(GFLAGS_ROOT_DIR gflags-2.1.1)
set(GLOG_ROOT_DIR glog-0.3.3)
set(SUITESPARSE_METIS_ROOT_DIR suitesparse-metis-for-windows-1.2.1)
set(CERES_SOLVER_ROOT_DIR ceres-solver-1.9.0)
set(OPENCV_ROOT_DIR opencv-git)

# freeglut --------------------------------------------------
ConfigExternalProject(freeglut
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/freeglut_cmake
    CMAKE_ARGS -DFREEGLUT_ROOT_DIR:STRING=${FREEGLUT_ROOT_DIR})

# glew --------------------------------------------------
ConfigExternalProject(glew
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glew_cmake
    CMAKE_ARGS -DGLEW_ROOT_DIR:STRING=${GLEW_ROOT_DIR})

# opencv ----------------------------------------------------
ConfigExternalProject(OpenCV
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${OPENCV_ROOT_DIR}
    CMAKE_ARGS -DBUILD_SHARED_LIBS:BOOLEAN=FALSE
    CMAKE_CACHE_ARGS -DWITH_CUDA:BOOLEAN=OFF
                     -DBUILD_PERF_TESTS:BOOLEAN=OFF
                     -DBUILD_TESTS:BOOLEAN=OFF
                     -DBUILD_EXAMPLES:BOOLEAN=OFF
                     -DBUILD_DOCS:BOOLEAN=OFF
                     -DBUILD_ANDROID_EXAMPLES:BOOLEAN=OFF
                     -DBUILD_WITH_STATIC_CRT:BOOLEAN=OFF
                     -DWITH_TESTS:BOOLEAN=OFF)
set(OpenCV_STATIC ON CACHE BOOLEAN "")

# gflags -----------------------------------------------------------------
ConfigExternalProject(gflags
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${GFLAGS_ROOT_DIR}
    CMAKE_CACHE_ARGS -DGFLAGS_NAMESPACE:STRING=google)

# we must fix the gflags-config, the gflags target is gflags-static!
if(WIN32)
    set(gflags_DIR "${EP_PREFIX}/CMake")
else()
    set(gflags_DIR "${EP_PREFIX}/lib/cmake/gflags")
endif()
file(STRINGS "${gflags_DIR}/gflags-config.cmake" NVMARK REGEX "NVMARK")
if(NOT NVMARK AND EXISTS "${gflags_DIR}/gflags-config.cmake")
    file(APPEND "${gflags_DIR}/gflags-config.cmake"
                "\n# NVMARK\nset(gflags_LIBRARIES gflags-static)\n")
endif()


# glog --------------------------------------------------------------------
ConfigExternalProject(glog
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glog_cmake
    CMAKE_ARGS -DGLOG_ROOT_DIR:STRING=${GLOG_ROOT_DIR}
    CMAKE_CACHE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING= )

# suitesparse-metis -------------------------------------------------------
set(suitesparse_dir "${CMAKE_CURRENT_SOURCE_DIR}/${SUITESPARSE_METIS_ROOT_DIR}")
set(CFLAGS -Didxtype=idx_t)
if(WIN32)
    set(SuiteSparse_USE_LAPACK_BLAS ON)
    set(WIN32_COMPLEX_FIX -DHAVE_COMPLEX_H:BOOLEAN=OFF)
endif()

ConfigExternalProject(SuiteSparse
    SOURCE_DIR ${suitesparse_dir}
    CMAKE_ARGS -DCMAKE_C_FLAGS:STRING=${CFLAGS}
    CMAKE_CACHE_ARGS -DGKLIB_PATH:PATH=${suitesparse_dir}/metis/GKlib
                     -DSUITESPARSE_INSTALL_PREFIX:PATH=${EP_PREFIX}
                     ${WIN32_COMPLEX_FIX}
                     -DCMAKE_DEBUG_POSTFIX:STRING=)
include(${USE_SuiteSparse})

# eigen -------------------------------------------------------
ConfigExternalProject(Eigen
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${EIGEN_ROOT_DIR})
add_library(eigen INTERFACE)
set(EIGEN_INCLUDE_DIR ${EP_PREFIX}/include/eigen3)
target_include_directories(eigen INTERFACE ${EIGEN_INCLUDE_DIR})

#  ceres-solver ----------------------------------------------
if(NOT MSVC)
    set(CXXFLAGS -Wno-error=unused-local-typedefs)
else()
    set(CXXFLAGS "/DGOOGLE_GLOG_DLL_DECL= /MP")
endif()

ConfigExternalProject(Ceres
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${CERES_SOLVER_ROOT_DIR}
    CMAKE_CACHE_ARGS
               -DCXSPARSE_LIBRARY:PATH=
               -DCMAKE_CXX_FLAGS:STRING=${CXXFLAGS}
               -DBUILD_TESTING:BOOLEAN=OFF
               -DBUILD_EXAMPLES:BOOLEAN=OFF
               -DEIGEN_INCLUDE_DIR_HINTS:PATH=${EIGEN_INCLUDE_DIR}
               -DSUITESPARSE_INCLUDE_DIR_HINTS:PATH=${EP_PREFIX}/include/suitesparse
               -DCXSPARSE_CHECK_INCLUDE_DIRS:PATH=${EP_PREFIX}/include/suitesparse
               "-DSUITESPARSE_LIBRARY_DIR_HINTS:STRING=${EP_PREFIX}/lib\\\;${EP_PREFIX}/lib64"
               "-DCXSPARSE_CHECK_LIBRARY_DIRS:STRING=${EP_PREFIX}/lib\\\;${EP_PREFIX}/lib64")

# ceres is such a pain... it insists in using its own Find*, we need to use
# ours...
if(WIN32)
    set(Ceres_DIR "${EP_PREFIX}/CMake")
else()
    set(Ceres_DIR "${EP_PREFIX}/share/Ceres")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindGlog.cmake" "${Ceres_DIR}")

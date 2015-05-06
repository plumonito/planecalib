cmake_minimum_required(VERSION 3.0)

if(WIN32)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install directory")
endif()
SET(CMAKE_DEBUG_POSTFIX "d")

project(glog CXX)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 3)
set(VERSION_PATCH 3)
set(GLOG_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

set(BUILD_UNITTEST OFF CACHE BOOL "")

# sources
find_path(GLOG_ROOT_DIR src/logging.cc HINTS "../glog-0.3.3" "../glog")

if(WIN32)
	set(INCLUDE_DIR ${GLOG_ROOT_DIR}/src/windows)
	
	set(SRCS
		${GLOG_ROOT_DIR}/src/logging.cc
		${GLOG_ROOT_DIR}/src/raw_logging.cc
		${GLOG_ROOT_DIR}/src/utilities.cc
		${GLOG_ROOT_DIR}/src/vlog_is_on.cc
		${GLOG_ROOT_DIR}/src/windows/port.cc
	)

	set(INTERNAL_HDRS
		${GLOG_ROOT_DIR}/src/utilities.h
		${GLOG_ROOT_DIR}/src/base/commandlineflags.h
		${GLOG_ROOT_DIR}/src/base/googleinit.h
		${GLOG_ROOT_DIR}/src/base/mutex.h
		${GLOG_ROOT_DIR}/src/windows/port.h
		${GLOG_ROOT_DIR}/src/windows/config.h
	)

	set(PUBLIC_HDRS
		${GLOG_ROOT_DIR}/src/windows/glog/log_severity.h
		${GLOG_ROOT_DIR}/src/windows/glog/logging.h
		${GLOG_ROOT_DIR}/src/windows/glog/raw_logging.h
		${GLOG_ROOT_DIR}/src/windows/glog/stl_logging.h
		${GLOG_ROOT_DIR}/src/windows/glog/vlog_is_on.h
	)
	
	set(UNITTEST_SRCS
		${GLOG_ROOT_DIR}/src/logging_unittest.cc
		${GLOG_ROOT_DIR}/src/googletest.h
	)
endif()

#Install
# Absolute install paths
set(INSTALL_LIB_DIR "lib")
set(INSTALL_BIN_DIR "bin")
set(INSTALL_INCLUDE_DIR "include")
set(INSTALL_CMAKE_DIR "CMake")
 
# Create the glogConfig and glogConfigVersion files
set(TARGETS_FILE glogTargets.cmake)
configure_file(config.cmake.in
  "${PROJECT_BINARY_DIR}/glogConfig.cmake" @ONLY)
configure_file(version.cmake.in
  "${PROJECT_BINARY_DIR}/glogConfigVersion.cmake" @ONLY)
 
# Install the glogConfig.cmake and glogConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}/glogConfig.cmake"
  "${PROJECT_BINARY_DIR}/glogConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

###################################
#Create targets

#glog
add_library(glog SHARED ${SRCS} ${INTERNAL_HDRS} ${PUBLIC_HDRS})
target_compile_definitions(glog PRIVATE LIBGLOG_EXPORTS)
target_include_directories(glog PRIVATE ${INCLUDE_DIR} 
	INTERFACE $<BUILD_INTERFACE:${INCLUDE_DIR}> $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)
set_target_properties(glog PROPERTIES PUBLIC_HEADER ${PUBLIC_HDRS})
install(TARGETS glog
			EXPORT glogTargets
			RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
			ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
			LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
			#PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/glog" COMPONENT dev
)
 
add_library(glog-static STATIC ${SRCS} ${INTERNAL_HDRS} ${PUBLIC_HDRS})
target_compile_definitions(glog-static PUBLIC GOOGLE_GLOG_DLL_DECL=)
target_include_directories(glog-static PRIVATE ${INCLUDE_DIR} 
	INTERFACE $<BUILD_INTERFACE:${INCLUDE_DIR}> $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)
set_target_properties(glog-static PROPERTIES PUBLIC_HEADER ${PUBLIC_HDRS})
install(TARGETS glog-static
			EXPORT glogTargets
			ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
			LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
			#PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/glog" COMPONENT dev
)

#Install headers
install(FILES ${PUBLIC_HDRS} DESTINATION "${INSTALL_INCLUDE_DIR}/glog")

						
if(BUILD_UNITTEST)
	add_executable(logging_unittest ${UNITTEST_SRCS})
	target_link_libraries(logging_unittest glog)
	target_include_directories(logging_unittest PRIVATE ${INCLUDE_DIR})
	install(TARGETS logging_unittest
				EXPORT glogUnitTestTargets
				RUNTIME DESTINATION ${INSTALL_BIN_DIR})

	add_executable(logging_unittest_static ${UNITTEST_SRCS})
	target_link_libraries(logging_unittest_static glog-static)
	target_include_directories(logging_unittest_static PRIVATE ${INCLUDE_DIR})
	install(TARGETS logging_unittest_static
				EXPORT glogUnitTestTargets
				RUNTIME DESTINATION ${INSTALL_BIN_DIR})
endif()
		
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE glog)

#Write export set (glogTargets.cmake) for use with the build-tree
export(EXPORT glogTargets
	FILE "${CMAKE_BINARY_DIR}/glogTargets.cmake")
	
# Install the export set for use with the install-tree
install(EXPORT glogTargets 
  DESTINATION "${INSTALL_CMAKE_DIR}" 
  COMPONENT dev)

##################################
# planecalib_desktop application
##################################

#GLUT should already be included

##################################
# Sources

#Add all files
#file(GLOB_RECURSE sources_cpp *.cpp)
#file(GLOB_RECURSE sources_h *.h)
set(sources_cpp)
set(sources_h)
list(APPEND sources_cpp "main.cpp")
#list(APPEND sources_h "${PROJECT_SOURCE_DIR}/planecalib_ui/PlaneCalibApp.h") #Important to add the MainDriver.h file so that the project recompiles when it changes.

##################################
# Target

add_executable(planecalib_desktop ${sources_cpp} ${sources_h})
target_link_libraries (planecalib_desktop planecalib_ui planecalib gflags-static freeglut) 

#INSTALL_TARGET_WITH_DEPS(planecalib_desktop ${3RD_PARTY_DIR}/bin)
#install(DIRECTORY ${CMAKE_SOURCE_DIR}/planecalib/assets DESTINATION .)
#install(DIRECTORY ${CMAKE_SOURCE_DIR}/planecalib_ui/assets DESTINATION .)

add_custom_command(TARGET planecalib_desktop POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/planecalib/assets $<TARGET_FILE_DIR:planecalib_desktop>/assets)
add_custom_command(TARGET planecalib_desktop POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/planecalib_ui/assets $<TARGET_FILE_DIR:planecalib_desktop>/assets)
	

if(WIN32)
    # assets must be copied also to current_binary_dir in MSVC so that it can be run
    # by the IDE
    if(MSVC)
        add_custom_command(TARGET planecalib_desktop POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/planecalib/assets ${CMAKE_CURRENT_BINARY_DIR}/assets)

        add_custom_command(TARGET planecalib_desktop POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/planecalib_ui/assets ${CMAKE_CURRENT_BINARY_DIR}/assets)
    endif()

    make_directory(${CMAKE_CURRENT_BINARY_DIR}/Debug)
    make_directory(${CMAKE_CURRENT_BINARY_DIR}/Release)

	foreach(import_target glog ceres blas lapack ${OpenCV_LIBRARIES})
		foreach(config RELEASE DEBUG)
			get_property(libname_set TARGET ${import_target} PROPERTY IMPORTED_LOCATION_${config} SET)
			if(${libname_set})
				get_property(libname TARGET ${import_target} PROPERTY IMPORTED_LOCATION_${config})
			else()
				get_property(libname TARGET ${import_target} PROPERTY IMPORTED_LOCATION)
			endif()
			if(${libname})
    			get_filename_component(libname_ext ${libname} EXT)
    			string(TOLOWER ${libname_ext} ${libname_ext})
    			if(${libname_ext} STREQUAL ".dll")
    				message(STATUS "Copying dll ${libname}...")
    				execute_process(
    					COMMAND ${CMAKE_COMMAND} -E copy ${libname} ${CMAKE_CURRENT_BINARY_DIR}/${config})
    			endif()
            endif()
		endforeach()
	endforeach()
endif()

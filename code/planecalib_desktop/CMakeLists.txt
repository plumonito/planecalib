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
target_link_libraries (planecalib_desktop planecalib_ui planecalib freeglut) 

#INSTALL_TARGET_WITH_DEPS(planecalib_desktop ${3RD_PARTY_DIR}/bin)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/planecalib/assets DESTINATION .)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/planecalib_ui/assets DESTINATION .)

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

    foreach(dll ${DEBUG_DLL_LIBRARIES})
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E copy ${dll} ${CMAKE_CURRENT_BINARY_DIR}/Debug)
    endforeach()

    foreach(dll ${RELEASE_DLL_LIBRARIES})
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E copy ${dll} ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endforeach()
endif()

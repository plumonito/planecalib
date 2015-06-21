##################################
# planecalib_ui lib
##################################

##################################
# Sources

#Add all files
file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_h *.h)
file(GLOB_RECURSE sources_frag *.frag)
file(GLOB_RECURSE sources_vert *.vert)
GroupSources(planecalib_ui)

##################################
# Target

add_library(planecalib_ui ${sources_cpp} ${sources_h} ${sources_frag} ${sources_vert})
target_link_libraries (planecalib_ui planecalib glew freeglut matio)
target_include_directories(planecalib_ui PUBLIC ${OpenCV_INCLUDE_DIRS} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)

##################################
# planecalib lib
##################################

##################################
# Sources

#Add all files
file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_h *.h)
file(GLOB_RECURSE sources_hpp *.hpp)
GroupSources(planecalib)

configure_file(ProjectConfig.h.in ProjectConfig.h)

##################################
# Target

add_library(planecalib ${sources_cpp} ${sources_h} ${sources_hpp})
target_link_libraries(planecalib PUBLIC ceres ${glog_LIBRARIES}  gflags-shared eigen ${OpenCV_LIBRARIES})
include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(planecalib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/.. PUBLIC ${CMAKE_CURRENT_BINARY_DIR})


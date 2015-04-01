##################################
# planecalib_ui lib
##################################

##################################
# Sources
include_directories(C:/code/matio/visual_studio/Release/include)


# matio
add_library(matio SHARED IMPORTED)
set_property(TARGET matio APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(matio PROPERTIES
  IMPORTED_IMPLIB_DEBUG "C:/code/matio/visual_studio/Release/libmatio.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG ""
  IMPORTED_LOCATION_DEBUG "C:/code/matio/visual_studio/Release/libmatio.dll"
  )
set_property(TARGET matio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(matio PROPERTIES
  IMPORTED_IMPLIB_RELEASE "C:/code/matio/visual_studio/Release/libmatio.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE "C:/code/matio/visual_studio/Release/libmatio.dll"
  )

list(APPEND DEBUG_DLL_LIBRARIES C:/code/matio/visual_studio/Release/libmatio.dll)
list(APPEND RELEASE_DLL_LIBRARIES C:/code/matio/visual_studio/Release/libmatio.dll)
set(RELEASE_DLL_LIBRARIES "${RELEASE_DLL_LIBRARIES}" CACHE STRING "" FORCE)
set(DEBUG_DLL_LIBRARIES "${DEBUG_DLL_LIBRARIES}" CACHE STRING "" FORCE)

#Add all files
file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_h *.h)
GroupSources(planecalib_ui)

##################################
# Target

add_library(planecalib_ui ${sources_cpp} ${sources_h})
target_link_libraries (planecalib_ui glew planecalib ${OpenCV_LIBRARIES} freeglut matio)
target_include_directories(planecalib_ui PUBLIC ${OpenCV_INCLUDE_DIRS} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
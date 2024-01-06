set(SUBSYS_NAME examples)
set(SUBSYS_DESC "PCL examples")
set(SUBSYS_DEPS common io features search kdtree octree filters keypoints segmentation sample_consensus outofcore stereo geometry surface)

set(DEFAULT FALSE)
set(REASON "Code examples are disabled by default.")
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if(NOT build)
  return()
endif()

# this variable will filled with all targets added with pcl_add_example
set(PCL_EXAMPLES_ALL_TARGETS)

include_directories(${PCL_INCLUDE_DIRS})
# This looks for all examples/XXX/CMakeLists.txt
file (GLOB examples_sbudirs */CMakeLists.txt)
# Extract only relevant XXX and append it to PCL_EXAMPLES_SUBDIRS
foreach(subdir ${examples_sbudirs})
  # PATH of get_filename_component has problem dealing with windows shared folders "//xxx.xxx.xxx.xxx/folder"
  # Check here for the discussion: http://www.pcl-developers.org/Small-hack-to-get-filename-component-td5706691.html
  # Modified by Yangyan Li(yangyan.lee@gmail.com)
  # get_filename_component (dir ${subdir} PATH)
  get_filename_component (absolute ${subdir} ABSOLUTE)
  get_filename_component (name ${subdir} NAME)
  STRING(REGEX REPLACE ${name} "" dir ${absolute})
  list(APPEND PCL_EXAMPLES_SUBDIRS ${dir})
endforeach()

# Add the subdirectories
foreach(subdir ${PCL_EXAMPLES_SUBDIRS})
  add_subdirectory(${subdir})
endforeach()

if(CMAKE_GENERATOR_IS_IDE)
  set(SUBSYS_TARGET_NAME EXAMPLES_BUILD)
else()
  set(SUBSYS_TARGET_NAME ${SUBSYS_NAME})
endif()
add_custom_target(${SUBSYS_TARGET_NAME} DEPENDS ${PCL_EXAMPLES_ALL_TARGETS})
set_target_properties(${SUBSYS_TARGET_NAME} PROPERTIES FOLDER "Examples")


set(SUBSYS_NAME examples)
set(SUBSYS_DESC "PCL examples")
set(SUBSYS_DEPS common io features search kdtree octree filters keypoints segmentation sample_consensus outofcore stereo geometry)

set(DEFAULT FALSE)
set(REASON "Code examples are disabled by default.")
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if (build)
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
    endforeach(subdir)
    # Display the found subdirectories
    message (STATUS "PCL_EXAMPLES_SUBDIRS ${PCL_EXAMPLES_SUBDIRS}")
    # Add the subdirectories
    foreach(subdir ${PCL_EXAMPLES_SUBDIRS})
    add_subdirectory(${subdir})
    endforeach(subdir)
endif (build)

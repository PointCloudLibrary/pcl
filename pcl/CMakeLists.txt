cmake_minimum_required (VERSION 2.4.6)

if (WIN32)
    include(cmake/windows.cmake)
else (WIN32)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
#add_definitions(-Wall -funroll-loops -momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -DNDEBUG -fgcse-lm -fgcse-sm -fsched-spec-load)
#add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)
add_definitions (-Wall -DEIGEN_NO_DEBUG -DEIGEN_USE_NEW_STDVECTOR -DEIGEN2_SUPPORT -fno-strict-aliasing)# -fvisibility=hidden -fvisibility-inlines-hidden)
rosbuild_check_for_sse ()

rosbuild_genmsg ()
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (src)
include_directories (include)

# ---[ Point Cloud Library
#add_subdirectory (src/pcl)
#add_subdirectory (src/tools)
#add_subdirectory (test)

# Option to generate a standalone tarball
add_custom_target (tarball ${PROJECT_SOURCE_DIR}/make_tarball WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} VERBATIM)

endif (WIN32)



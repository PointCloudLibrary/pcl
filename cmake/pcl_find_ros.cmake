# Look for ROS.
# This is a temporary and ugly hack.

macro(link_ros_libs _target)
    if(USE_ROS)
        target_link_libraries(${_target} sensor_msgs roscpp_serialization rosconsole)
    endif(USE_ROS)
endmacro(link_ros_libs)


macro(get_ros_inc_path _dest _pkg)
    execute_process(COMMAND rospack find ${_pkg}
        RESULT_VARIABLE _res
        OUTPUT_VARIABLE ${_dest}
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _res) #unix return code 0 means good...
        message(STATUS "Found ROS package ${_pkg} path: ${${_dest}}")
    endif()
endmacro(get_ros_inc_path)

message(STATUS "ROS_ROOT $ENV{ROS_ROOT}")
set(ROS_ROOT $ENV{ROS_ROOT})
if(ROS_ROOT)
    option(USE_ROS "Integrate with ROS rather than using native files" OFF)
    message(STATUS "Found ROS; USE_ROS is ${USE_ROS}")
    if(USE_ROS)
        set(_ros_pkgs std_msgs sensor_msgs roscpp_serialization cpp_common rostime
            roscpp_traits roscpp rosconsole std_msgs rosbag topic_tools pcl)
        set(_ros_paths)
        foreach(_pkg ${_ros_pkgs})
            get_ros_inc_path(_path ${_pkg})
            list(APPEND _ros_paths ${_path})
        endforeach(_pkg)
        foreach(_path ${_ros_paths})
            include_directories(${_path}/include
                                ${_path}/msg_gen/cpp/include
                                )
            link_directories(${_path}/lib)
        endforeach(_path)
        # use, i.e. don't skip the full RPATH for the build tree
        SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

        # when building, don't use the install RPATH already
        # (but later on when installing)
        SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

        # the RPATH to be used when installing
        SET(CMAKE_INSTALL_RPATH "${CMAKE_SOURCE_DIR}/../../lib64")
        # don't add the automatically determined parts of the RPATH
        # which point to directories outside the build tree to the install RPATH
        SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    endif(USE_ROS)
endif(ROS_ROOT)


# Look for ROS.
# This is a temporary and ugly hack.

macro(link_ros_libs _target)
    if(USE_ROS)
        target_link_libraries(${_target} sensor_msgs rosbag topic_tools ros
            boost_thread-mt boost_signals-mt roscpp_serialization XmlRpc
            rosconsole boost_thread-mt log4cxx rostime cpp_common roslib
            rospack rosstack)
    endif(USE_ROS)
endmacro(link_ros_libs)


macro(get_ros_inc_path _dest _pkg)
    execute_process(COMMAND rospack find ${_pkg}
        RESULT_VARIABLE _res
        OUTPUT_VARIABLE ${_dest}
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(RESULT_VARIABLE)
        message(STATUS "Found ROS package ${_pkg} path: ${${_dest}}")
    endif(RESULT_VARIABLE)
endmacro(get_ros_inc_path)


if(ENV{ROS_ROOT})
    option(USE_ROS "Integrate with ROS rather than using native files" ON)
    message(STATUS "Found ROS; USE_ROS is ${USE_ROS}")
    if(USE_ROS)
        set(_ros_pkgs std_msgs sensor_msgs roscpp_serialization cpp_common rostime
            roscpp_traits roscpp rosconsole std_msgs)
        set(_ros_paths)
        foreach(_pkg ${_ros_pkgs)
            get_ros_path(_path ${_pkg})
            list(APPEND _ros_paths ${_path})
        endforeach(_pkg)
        foreach(_path ${_ros_paths})
            include_directories(${_path}/include)
            link_directories(${_path}/lib)
        endforeach(_path)
    endif(USE_ROS)
endif(ENV{ROS_ROOT})


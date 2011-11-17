if(NOT EXISTS "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/install_manifest.txt")
    message(FATAL_ERROR "Cannot find install manifest: \"/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/install_manifest.txt\"")
endif(NOT EXISTS "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/install_manifest.txt")

file(READ "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
    message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
    message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
    if(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        exec_program("/usr/local/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
            OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
        if(NOT "${rm_retval}" STREQUAL 0)
            message(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
        endif(NOT "${rm_retval}" STREQUAL 0)
    else(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
    endif(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
endforeach(file)

# remove pcl directory in include (removes all files in it!)
message(STATUS "Uninstalling \"/usr/local/include/pcl-1.1\"")
if(EXISTS "/usr/local/include/pcl-1.1")
    exec_program("/usr/local/bin/cmake"
        ARGS "-E remove_directory \"/usr/local/include/pcl-1.1\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"/usr/local/include/pcl-1.1\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "/usr/local/include/pcl-1.1")
    message(STATUS
        "Directory \"/usr/local/include/pcl-1.1\" does not exist.")
endif(EXISTS "/usr/local/include/pcl-1.1")

# remove pcl directory in lib (removes all files in it!)
# created by cmake/pcl_targets.cmake for PCLDepends.cmake
message(STATUS "Uninstalling \"/usr/local/lib/pcl\"")
if(EXISTS "/usr/local/lib/pcl")
    exec_program("/usr/local/bin/cmake"
        ARGS "-E remove_directory \"/usr/local/lib/pcl\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"/usr/local/lib/pcl\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "/usr/local/lib/pcl")
    message(STATUS
        "Directory \"/usr/local/lib/pcl\" does not exist.")
endif(EXISTS "/usr/local/lib/pcl")

# remove pcl directory in share (removes all files in it!)
# created by CMakeLists.txt for PCLConfig.cmake
message(STATUS "Uninstalling \"/usr/local/share/pcl-1.1\"")
if(EXISTS "/usr/local/share/pcl-1.1")
    exec_program("/usr/local/bin/cmake"
        ARGS "-E remove_directory \"/usr/local/share/pcl-1.1\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"/usr/local/share/pcl-1.1\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "/usr/local/share/pcl-1.1")
    message(STATUS
        "Directory \"/usr/local/share/pcl-1.1\" does not exist.")
endif(EXISTS "/usr/local/share/pcl-1.1")

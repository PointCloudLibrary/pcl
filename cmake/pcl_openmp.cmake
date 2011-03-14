# Find the correct flags to use for OpenMP.
# Taken from rosbuild and edited a little.

include(${PROJECT_SOURCE_DIR}/cmake/pcl_targets.cmake)

find_package(OpenMP)

###############################################################################
# Add the OpenMP flags to a target.
# _name The name of the target to add the flags to.
macro(PCL_ADD_OPENMP_FLAGS _name)
    if(OPENMP_FOUND)
        PCL_ADD_CFLAGS(${_name} ${OpenMP_CXX_FLAGS})
    endif(OPENMP_FOUND)
endmacro(PCL_ADD_OPENMP_FLAGS)


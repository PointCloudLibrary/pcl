# Find the correct flags to use for OpenMP.
# Taken from rosbuild and edited a little.

include(${PROJECT_SOURCE_DIR}/cmake/pcl_targets.cmake)

###############################################################################
# Search for the correct flags for OpenMP.
# The variable OPENMP_FLAGS_FOUND will be set to true if the correct flags were
# found.
# The flags are stored in OPENMP_FLAGS.
macro(PCL_FIND_OPENMP_FLAGS)
	include(FindOpenMP)
	if(OPENMP_FOUND)
		set(OPENMP_FLAGS_FOUND FALSE)
	else(OPENMP_FOUND)
		set(OPENMP_FLAGS_FOUND true)
	endif(OPENMP_FOUND)
endmacro(PCL_FIND_OPENMP_FLAGS)


###############################################################################
# Add the OpenMP flags to a target.
# _name The name of the target to add the flags to.
macro(PCL_ADD_OPENMP_FLAGS _name)
    if(OPENMP_FLAGS_FOUND)
        PCL_ADD_CFLAGS(${_name} ${OPENMP_FLAGS})
        PCL_ADD_LINKFLAGS(${_name} ${OPENMP_FLAGS})
    endif(OPENMP_FLAGS_FOUND)
endmacro(PCL_ADD_OPENMP_FLAGS)


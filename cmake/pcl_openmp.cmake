# Find the correct flags to use for OpenMP.
# Taken from rosbuild and edited a little.

include(${PROJECT_SOURCE_DIR}/cmake/pcl_targets.cmake)

###############################################################################
# Search for the correct flags for OpenMP.
# The variable OPENMP_FLAGS_FOUND will be set to true if the correct flags were
# found.
# The flags are stored in OPENMP_FLAGS.
macro(PCL_FIND_OPENMP_FLAGS)
    include(CheckFunctionExists)
    set(_possible_flags "-fopenmp" # GCC
        "-openmp" # ICC
        "-mp" #SGI and PGI
        "-xopenmp" # Sun
        "-omp" # Tru64
        "-qsmp=omp" #AIX
        )

    # Have not found the flag yet
    set(OPENMP_FLAGS_FOUND FALSE)
    set(OPENMP_FLAGS)

    foreach(_flag ${_possible_flags})
        if(NOT OPENMP_FLAGS_FOUND)
            set(CMAKE_REQUIRED_FLAGS ${_flag})
            check_function_exists(omp_set_num_threads OPENMP_FLAGS_${_flag})
            if(OPENMP_FLAGS_${_flag})
                set(OPENMP_FLAGS ${_flag})
                message(STATUS "Found OpenMP flags: ${OPENMP_FLAGS}")
								set(OPENMP_FLAGS_FOUND true)
            endif(OPENMP_FLAGS_${_flag})
        endif(NOT OPENMP_FLAGS_FOUND)
    endforeach(_flag)

    if(NOT OPENMP_FLAGS_FOUND)
        message(STATUS "OpenMP flags were not found.")
    endif(NOT OPENMP_FLAGS_FOUND)
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


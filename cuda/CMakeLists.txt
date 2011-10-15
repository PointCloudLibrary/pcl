set(SUBSYS_NAME cuda)
set(SUBSYS_DESC "Point cloud CUDA libraries")
set(SUBSYS_DEPS )

# pcl_cuda depends on PCL, CUDA, OpenNI, VTK, maybe: OpenCV

# Find CUDA
find_package(CUDA)
if(NOT CUDA_FOUND)
    set(DEFAULT FALSE)
    set(REASON "CUDA was not found.")
else(NOT CUDA_FOUND)
    if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" 
                        OUTPUT_VARIABLE GCC_VERSION_STRING)
        if(GCC_VERSION_STRING VERSION_LESS 4.4)
            set(DEFAULT FALSE)
            set(REASON "GCC version is less than 4.4.")
        else(GCC_VERSION_STRING VERSION_LESS 4.4)
            set(DEFAULT TRUE)
            set(REASON)
            message(STATUS "CUDA found (include: ${CUDA_INCLUDE_DIRS}, lib: ${CUDA_LIBRARIES})")
        endif(GCC_VERSION_STRING VERSION_LESS 4.4)
    else(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        set(DEFAULT TRUE)
        set(REASON)
        message(STATUS "CUDA found (include: ${CUDA_INCLUDE_DIRS}, lib: ${CUDA_LIBRARIES})")
    endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)    
endif(NOT CUDA_FOUND)

OPTION(BUILD_CUDA "Build the CUDA-related subsystems" ON)

if(BUILD_CUDA)
    if(CUDA_FOUND)      
        set(CUDA_ARCH_BIN "20" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
        set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              
        
        # Guess this macros will be included in cmake distributive
        include(${PCL_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)                  
        APPEND_TARGET_ARCH_FLAGS()
        
        collect_subproject_directory_names(${CMAKE_CURRENT_SOURCE_DIR} "CMakeLists.txt" PCL_CUDA_MODULES_NAMES PCL_CUDA_MODULES_DIRS)
        set(PCL_CUDA_MODULES_NAMES_UNSORTED ${PCL_CUDA_MODULES_NAMES})
        topological_sort(PCL_CUDA_MODULES_NAMES PCL_ _DEPENDS)
        sort_relative(PCL_CUDA_MODULES_NAMES_UNSORTED PCL_CUDA_MODULES_NAMES PCL_CUDA_MODULES_DIRS)
        foreach(subdir ${PCL_CUDA_MODULES_DIRS})
            add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
        endforeach(subdir)
    endif(CUDA_FOUND)
endif(BUILD_CUDA)


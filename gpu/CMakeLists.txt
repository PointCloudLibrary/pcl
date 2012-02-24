set(SUBSYS_NAME gpu)
set(SUBSYS_DESC "Point cloud GPU libraries")
set(SUBSYS_DEPS )

# pcl_cuda depends on PCL, CUDA, OpenNI, VTK, maybe: OpenCV

# Find CUDA
set(CUDA_FIND_QUIETLY TRUE)
find_package(CUDA)
if(NOT CUDA_FOUND)
    set(DEFAULT FALSE)
    set(REASON "CUDA was not found.")
endif(NOT CUDA_FOUND)

OPTION(BUILD_GPU "Build the GPU-related subsystems" ${DEFAULT})

if(BUILD_GPU)
    if(CUDA_FOUND)      
        set(CUDA_ARCH_BIN "20" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
        set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              
        
        # Guess this macros will be included in cmake distributive
        include(${PCL_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)                  
        APPEND_TARGET_ARCH_FLAGS()
		
		string(REPLACE "-std=gnu++0x" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
		string(REPLACE "-std=gnu++0x" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
		string(REPLACE "-std=gnu++0x" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
		string(REPLACE "-std=gnu++0x" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
						 
        collect_subproject_directory_names(${CMAKE_CURRENT_SOURCE_DIR} "CMakeLists.txt" PCL_GPU_MODULES_NAMES PCL_GPU_MODULES_DIRS)
        set(PCL_GPU_MODULES_NAMES_UNSORTED ${PCL_GPU_MODULES_NAMES})
        topological_sort(PCL_GPU_MODULES_NAMES PCL_ _DEPENDS)
        sort_relative(PCL_GPU_MODULES_NAMES_UNSORTED PCL_GPU_MODULES_NAMES PCL_GPU_MODULES_DIRS)
        foreach(subdir ${PCL_GPU_MODULES_DIRS})
            add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
        endforeach(subdir)
    endif(CUDA_FOUND)
endif(BUILD_GPU)


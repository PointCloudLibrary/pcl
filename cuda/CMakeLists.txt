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
    set(DEFAULT TRUE)
    set(REASON)
    message(STATUS "CUDA found (include: ${CUDA_INCLUDE_DIRS}, lib: ${CUDA_LIBRARIES})")
endif(NOT CUDA_FOUND)

OPTION(BUILD_CUDA "Build the CUDA-related subsystems" ON)

if(BUILD_CUDA)
    if(CUDA_FOUND)      
        set(CUDA_ARCH_BIN "20" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
        set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              
        
        # Guess this macros will be included in cmake distributive
        include(${PCL_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)                  
        APPEND_TARGET_ARCH_FLAGS()
        
        add_subdirectory (common)
        add_subdirectory (io)
        add_subdirectory (features)
        add_subdirectory (segmentation)
        add_subdirectory (sample_consensus)
        add_subdirectory (apps)
            
        add_subdirectory (containers)
        #add_subdirectory (gpu_features)
        add_subdirectory (utils)    
        add_subdirectory (octree)	
    endif(CUDA_FOUND)
endif(BUILD_CUDA)


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

if(build)
    if(CUDA_FOUND)
        include(CudaComputeTargetFlags.cmake)
        APPEND_TARGET_ARCH_FLAGS()
        if(PCL_SHARED_LIBS)
            # to overcome a limitation in cuda_add_library, we add manually PCLAPI_EXPORTS macro
            set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-DPCLAPI_EXPORTS")
        endif(PCL_SHARED_LIBS)
        add_subdirectory (common)
        add_subdirectory (io)
        add_subdirectory (features)
        add_subdirectory (segmentation)
        add_subdirectory (sample_consensus)
        add_subdirectory (apps)
    endif(CUDA_FOUND)
endif(build)


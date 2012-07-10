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

if(CMAKE_COMPILER_IS_GNUCXX)
  string(REPLACE "-Wold-style-cast" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "-Wno-invalid-offsetof" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function")
endif()

if(BUILD_GPU)
    if(CUDA_FOUND)
        # Find a complete list for CUDA compute capabilities at http://developer.nvidia.com/cuda-gpus
		
		# CUDA_ARCH_BIN is a space separated list of versions to include in output so-file. So you can set CUDA_ARCH_BIN = 10 11 12 13 20
        # Also user can specify virtual arch in parenthesis to limit instructions  set, 
		# for example CUDA_ARCH_BIN = 11(11) 12(11) 13(11) 20(11) 21(11) -> forces using only sm_11 instructions.
        # The CMake scripts interpret XX as XX (XX). This allows user to omit parenthesis. 
		# Arch 21 is an exceptional case since it doesn't have own sm_21 instructions set. 
		# So 21 = 21(21) is an invalid configuration and user has to explicitly force previous sm_20 instruction set via 21(20).
        # CUDA_ARCH_BIN adds support of only listed GPUs. As alternative CMake scripts also parse 'CUDA_ARCH_PTX' variable,
        # which is a list of intermediate PTX codes to include in final so-file. The PTX code can/will be JIT compiled for any current or future GPU. 
		# To add support of older GPU for kinfu, I would embed PTX 11 and 12 into so-file. GPU with sm_13  will run PTX 12 code (no difference for kinfu)

        if(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
            set(CUDA_ARCH_BIN "3.0" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
        else()
            set(CUDA_ARCH_BIN "2.0 2.1(2.0)" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
        endif()

        set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")
		#set(CUDA_ARCH_PTX "1.1 1.2" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

        # Guess this macros will be included in cmake distributive
        include(${PCL_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)
        APPEND_TARGET_ARCH_FLAGS()

        collect_subproject_directory_names(${CMAKE_CURRENT_SOURCE_DIR} "CMakeLists.txt" PCL_GPU_MODULES_NAMES PCL_GPU_MODULES_DIRS)
        set(PCL_GPU_MODULES_NAMES_UNSORTED ${PCL_GPU_MODULES_NAMES})
        topological_sort(PCL_GPU_MODULES_NAMES PCL_ _DEPENDS)
        sort_relative(PCL_GPU_MODULES_NAMES_UNSORTED PCL_GPU_MODULES_NAMES PCL_GPU_MODULES_DIRS)
        foreach(subdir ${PCL_GPU_MODULES_DIRS})
            add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
        endforeach(subdir)
    endif(CUDA_FOUND)
endif(BUILD_GPU)


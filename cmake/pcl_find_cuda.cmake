# Find CUDA
set(CUDA_FIND_QUIETLY TRUE)
find_package(CUDA 4)

if(CUDA_FOUND)
	message(STATUS "Found CUDA Toolkit v${CUDA_VERSION_STRING}")
	
	# CUDA_ARCH_BIN is a space separated list of versions to include in output so-file. So you can set CUDA_ARCH_BIN = 10 11 12 13 20
	# Also user can specify virtual arch in parenthesis to limit instructions  set, 
	# for example CUDA_ARCH_BIN = 11(11) 12(11) 13(11) 20(11) 21(11) -> forces using only sm_11 instructions.
	# The CMake scripts interpret XX as XX (XX). This allows user to omit parenthesis. 
	# Arch 21 is an exceptional case since it doesn't have own sm_21 instructions set. 
	# So 21 = 21(21) is an invalid configuration and user has to explicitly force previous sm_20 instruction set via 21(20).
	# CUDA_ARCH_BIN adds support of only listed GPUs. As alternative CMake scripts also parse 'CUDA_ARCH_PTX' variable,
	# which is a list of intermediate PTX codes to include in final so-file. The PTX code can/will be JIT compiled for any current or future GPU. 
	# To add support of older GPU for kinfu, I would embed PTX 11 and 12 into so-file. GPU with sm_13  will run PTX 12 code (no difference for kinfu)
	
	# Find a complete list for CUDA compute capabilities at http://developer.nvidia.com/cuda-gpus

	if(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
		set(CUDA_ARCH_BIN "2.0 2.1(2.0) 3.0" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
	else()
		set(CUDA_ARCH_BIN "2.0 2.1(2.0)" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
	endif()

	set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")
	#set(CUDA_ARCH_PTX "1.1 1.2" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

	# Guess this macros will be included in cmake distributive
	include(${PCL_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)
	APPEND_TARGET_ARCH_FLAGS()
    
endif()

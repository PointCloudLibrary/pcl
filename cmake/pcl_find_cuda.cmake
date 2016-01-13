# Find CUDA



if(MSVC)
	# Setting this to true brakes Visual Studio builds.
	set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF CACHE BOOL "CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE")
endif()

set(CUDA_FIND_QUIETLY TRUE)
find_package(CUDA 4)

if(CUDA_FOUND)
	message(STATUS "Found CUDA Toolkit v${CUDA_VERSION_STRING}")
	
	if(${CUDA_VERSION_STRING} VERSION_LESS "7.5")
	  # Recent versions of cmake set CUDA_HOST_COMPILER to CMAKE_C_COMPILER which
	  # on OSX defaults to clang (/usr/bin/cc), but this is not a supported cuda
	  # compiler.  So, here we will preemptively set CUDA_HOST_COMPILER to gcc if
	  # that compiler exists in /usr/bin.  This will not override an existing cache
	  # value if the user has passed CUDA_HOST_COMPILER on the command line.
	  if (NOT DEFINED CUDA_HOST_COMPILER AND CMAKE_C_COMPILER_ID STREQUAL "Clang" AND EXISTS /usr/bin/gcc)
	    set(CUDA_HOST_COMPILER /usr/bin/gcc CACHE FILEPATH "Host side compiler used by NVCC")
	    message(STATUS "Setting CMAKE_HOST_COMPILER to /usr/bin/gcc instead of ${CMAKE_C_COMPILER}.  See http://dev.pointclouds.org/issues/979")
	  endif()

	  # Send a warning if CUDA_HOST_COMPILER is set to a compiler that is known
	  # to be unsupported.
	  if (CUDA_HOST_COMPILER STREQUAL CMAKE_C_COMPILER AND CMAKE_C_COMPILER_ID STREQUAL "Clang")
	    message(WARNING "CUDA_HOST_COMPILER is set to an unsupported compiler: ${CMAKE_C_COMPILER}.  See http://dev.pointclouds.org/issues/979")
	  endif()
	endif()

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

        if(NOT ${CUDA_VERSION_STRING} VERSION_LESS "6.5")
                set(__cuda_arch_bin "2.0 2.1(2.0) 3.0 3.5 5.0 5.2")
        elseif(NOT ${CUDA_VERSION_STRING} VERSION_LESS "6.0")
                set(__cuda_arch_bin "2.0 2.1(2.0) 3.0 3.5 5.0")
        elseif(NOT ${CUDA_VERSION_STRING} VERSION_LESS "5.0")
                set(__cuda_arch_bin "2.0 2.1(2.0) 3.0 3.5")
        elseif(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
                set(__cuda_arch_bin "2.0 2.1(2.0) 3.0")
        else()
                set(__cuda_arch_bin "2.0 2.1(2.0)")
        endif()

        set(CUDA_ARCH_BIN ${__cuda_arch_bin} CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")

	set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")
	#set(CUDA_ARCH_PTX "1.1 1.2" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

	# Guess this macros will be included in cmake distributive
	include(${PCL_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)
	APPEND_TARGET_ARCH_FLAGS()

endif()

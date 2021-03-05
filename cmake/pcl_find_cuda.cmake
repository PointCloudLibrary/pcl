# Find CUDA
if(MSVC)
  # Setting this to true brakes Visual Studio builds.
  set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF CACHE BOOL "CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE")
endif()

set(CUDA_FIND_QUIETLY TRUE)
find_package(CUDA 9.0)

if(CUDA_FOUND)
  message(STATUS "Found CUDA Toolkit v${CUDA_VERSION_STRING}")
  
  enable_language(CUDA)
  set(HAVE_CUDA TRUE)

  # Find a complete list for CUDA compute capabilities at http://developer.nvidia.com/cuda-gpus
  
  # For a list showing CUDA toolkit version support for compute capabilities see: https://en.wikipedia.org/wiki/CUDA
  # or the nvidia release notes ie: 
  # https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-general-new-features
  # or
  # https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#deprecated-features
  
  if(NOT ${CUDA_VERSION_STRING} VERSION_LESS "11.0")
    set(__cuda_arch_bin "35;37;50;52;53;60;61;62;70;72;75;80;86")
  elseif(NOT ${CUDA_VERSION_STRING} VERSION_LESS "10.0")
    set(__cuda_arch_bin "30;32;35;37;50;52;53;60;61;62;70;72;75")
  elseif(NOT ${CUDA_VERSION_STRING} VERSION_LESS "9.0")
    set(__cuda_arch_bin "30;32;35;37;50;52;53;60;61;62;70;72")
  endif()

  set(CUDA_ARCH_BIN ${__cuda_arch_bin} CACHE STRING "Specify 'real' GPU architectures to build binaries for")
  
  if(POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
    foreach(ver ${CUDA_ARCH_BIN})
      set(CMAKE_CUDA_ARCHITECTURES "${ver}-real;${ver}-virtual;${CMAKE_CUDA_ARCHITECTURES}")
    endforeach()
    set(CMAKE_CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
    message(STATUS "CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}")
  else()
    # Generate SASS
    foreach(ver ${CUDA_ARCH_BIN})
      set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_${ver},code=sm_${ver}")
    endforeach()
    # Generate PTX for last architecture
    list(GET CUDA_ARCH_BIN -1 ver)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_${ver},code=compute_${ver}")
    message(STATUS "CUDA GEN_CODE: ${ver}")
  endif ()
endif()

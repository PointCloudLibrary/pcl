# Find CUDA
if(MSVC)
  # Setting this to true brakes Visual Studio builds.
  set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF CACHE BOOL "CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE")
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.11)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.17)
      find_package(CUDAToolkit QUIET)
      set(CUDA_TOOLKIT_INCLUDE ${CUDAToolkit_INCLUDE_DIRS})
    else()
      set(CUDA_FIND_QUIETLY TRUE)
      find_package(CUDA 9.0)
    endif()

    set(CUDA_FOUND TRUE)
    set(CUDA_VERSION_STRING ${CMAKE_CUDA_COMPILER_VERSION})
  else()
    message(STATUS "No CUDA compiler found")
  endif()
else()
  set(CUDA_FIND_QUIETLY TRUE)
  find_package(CUDA 9.0)
endif()

if(CUDA_FOUND)
  message(STATUS "Found CUDA Toolkit v${CUDA_VERSION_STRING}")
  
  set(HAVE_CUDA TRUE)

  if (CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA")
    if(${CUDA_VERSION_STRING} VERSION_GREATER_EQUAL "11.1")
      execute_process(COMMAND ${CMAKE_CUDA_COMPILER} --list-gpu-code RESULT_VARIABLE EXIT_CODE OUTPUT_VARIABLE OUTPUT_VAL)
      if(EXIT_CODE EQUAL 0)
        #Remove sm_
        string(REPLACE "sm_" "" OUTPUT_VAL ${OUTPUT_VAL})
        #Convert to list
        string(REPLACE "\n" ";" __CUDA_ARCH_BIN ${OUTPUT_VAL})
        #Remove last empty entry
        list(REMOVE_AT __CUDA_ARCH_BIN -1)
      else()
        message(FATAL_ERROR "Failed to run NVCC to get list of GPU codes: ${EXIT_CODE}")
      endif()
    elseif(${CUDA_VERSION_STRING} VERSION_GREATER_EQUAL "11.0")
      set(__CUDA_ARCH_BIN "35;37;50;52;53;60;61;62;70;72;75;80")
    elseif(${CUDA_VERSION_STRING} VERSION_GREATER_EQUAL "10.0")
      set(__CUDA_ARCH_BIN "30;32;35;37;50;52;53;60;61;62;70;72;75")
    elseif(${CUDA_VERSION_STRING} VERSION_GREATER_EQUAL "9.1")
      set(__CUDA_ARCH_BIN "30;32;35;37;50;52;53;60;61;62;70;72")
    else()
      set(__CUDA_ARCH_BIN "30;32;35;37;50;52;53;60;61;62;70")
    endif()
  else()
    message(FATAL_ERROR "Unsupported CUDA compiler ${CMAKE_CUDA_COMPILER_ID}.")
  endif()

  set(CUDA_ARCH_BIN ${__CUDA_ARCH_BIN} CACHE STRING "Specify 'real' GPU architectures to build binaries for")
  
  if(POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
    set(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH_BIN})
    message(STATUS "CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}")
  else()
    # Generate SASS
    set(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH_BIN})
    # Generate PTX for last architecture
    list(GET CUDA_ARCH_BIN -1 ver)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_${ver},code=compute_${ver}")
    message(STATUS "CMAKE_CUDA_FLAGS: ${CMAKE_CUDA_FLAGS}")   
  endif ()
else()
  message(STATUS "CUDA was not found.")
endif()

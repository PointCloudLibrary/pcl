
# If user has specified compiler options/definitions/features/link options, do not override them
if(PCL_PRIVATE_COMPILER_OPTIONS OR PCL_PUBLIC_COMPILER_OPTIONS OR
   PCL_PRIVATE_COMPILER_DEFINITIONS OR PCL_PUBLIC_COMPILER_DEFINITIONS OR
   PCL_PRIVATE_COMPILER_FEATURES OR PCL_PUBLIC_COMPILER_FEATURES OR
   PCL_PRIVATE_LINK_OPTIONS)
  return()
endif()

set(PCL_PRIVATE_COMPILER_OPTIONS "")
set(PCL_PUBLIC_COMPILER_OPTIONS "")

set(PCL_PRIVATE_COMPILER_DEFINITIONS "")
set(PCL_PUBLIC_COMPILER_DEFINITIONS "")

set(PCL_PRIVATE_COMPILER_FEATURES "")
set(PCL_PUBLIC_COMPILER_FEATURES "")

set(PCL_PRIVATE_LINK_OPTIONS "")

# Check for unsupported compiler versions
if(MSVC AND ("${MSVC_VERSION}" LESS 1910))
  message(FATAL_ERROR "The compiler versions prior to Visual Studio version 2017 are not supported. Please upgrade to a newer version or another compiler!")
endif()

# Compiler identification
# Define a variable CMAKE_COMPILER_IS_X where X is the compiler short name.
# Note: CMake automatically defines one for GNUCXX, nothing to do in this case.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_COMPILER_IS_CLANG 1)
elseif(__COMPILER_PATHSCALE)
  set(CMAKE_COMPILER_IS_PATHSCALE 1)
elseif(MSVC)
  set(CMAKE_COMPILER_IS_MSVC 1)
elseif(MINGW)
  set(CMAKE_COMPILER_IS_MINGW 1)
endif()

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 17 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if("${CMAKE_CXX_STANDARD}" GREATER_EQUAL 17)
  set(PCL_PRIVATE_COMPILER_FEATURES cxx_std_17)
  set(PCL__cplusplus 201703L)
  set(PCL_REQUIRES_MSC_VER 1912)
elseif("${CMAKE_CXX_STANDARD}" EQUAL 14)
  set(PCL_PRIVATE_COMPILER_FEATURES cxx_std_14)
  set(PCL__cplusplus 201402L)
  set(PCL_REQUIRES_MSC_VER 1900)
else()
  message(FATAL_ERROR "Unknown or unsupported C++ standard specified")
endif()

set(CMAKE_CUDA_STANDARD 17 CACHE STRING "The target CUDA/C++ standard. PCL requires CUDA/C++ 14 or higher.")
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# check for allocation functions that return aligned memory
include("${PCL_SOURCE_DIR}/cmake/pcl_alignment.cmake")
PCL_CHECK_FOR_ALIGNMENT()

# check for SSE flags
if(PCL_ENABLE_SSE)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake")
  PCL_CHECK_FOR_SSE()
  list(APPEND PCL_PUBLIC_COMPILER_OPTIONS ${SSE_FLAGS})
endif()

# check for AVX flags
if(PCL_ENABLE_AVX)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_avx.cmake")
  PCL_CHECK_FOR_AVX()
  list(APPEND PCL_PUBLIC_COMPILER_OPTIONS ${AVX_FLAGS})
endif()

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
      list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-Wabi=18")
    else()
      list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-Wabi")
    endif()
    list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-Wall" "-Wextra" "-fno-strict-aliasing")
  endif()

  if(PCL_WARNINGS_ARE_ERRORS)
    list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-Werror" "-fno-strict-aliasing")
  endif()

  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "" AND NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    list(APPEND PCL_PRIVATE_LINK_OPTIONS "-Wl,--as-needed")
  endif()
endif()

if(CMAKE_COMPILER_IS_MSVC)
  if(PCL_SHARED_LIBS)
    list(APPEND PCL_PRIVATE_LINK_OPTIONS "-Wl,--export-all-symbols" "-Wl,--enable-auto-import")
  else()
    list(APPEND PCL_PRIVATE_DEFINITIONS "BOOST_LIB_DIAGNOSTIC" "BOOST_THREAD_USE_LIB")
  endif()

  list(APPEND PCL_PRIVATE_DEFINITIONS "BOOST_ALL_NO_LIB" "_SCL_SECURE_NO_WARNINGS" "_CRT_SECURE_NO_WARNINGS" "NOMINMAX")

  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/fp:precise")
    
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/bigobj")

  set(PCL_USE_GLOBAL_OPTIMIZATION TRUE)
  if(CUDA_FOUND)
    if(${CUDA_VERSION_STRING} VERSION_GREATER_EQUAL "10.0" AND ${CUDA_VERSION_STRING} VERSION_LESS "12.0")
      set(PCL_USE_GLOBAL_OPTIMIZATION FALSE)
      message("Global optimizations /GL has been turned off, as it doesn't work with nvcc/thrust in CUDA 10 and 11.")
    endif()
  endif()

  # Add extra code generation/link optimizations
  if(CMAKE_MSVC_CODE_LINK_OPTIMIZATION AND PCL_USE_GLOBAL_OPTIMIZATION AND ${CMAKE_BUILD_TYPE} EQUAL Release)
    list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/GL")
    list(APPEND PCL_PRIVATE_LINK_OPTIONS "/LTCG" "/OPT:REF")
  endif()
  # /MANIFEST:NO") # please, don't disable manifest generation, otherwise crash at start for vs2008

  # Disable some warnings
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/wd4800" "/wd4521" "/wd4251" "/wd4275" "/wd4305" "/wd4355")

  # Enable warnings, which are disabled by default (see https://learn.microsoft.com/de-de/cpp/preprocessor/compiler-warnings-that-are-off-by-default)
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/w34265")

  if(PCL_WARNINGS_ARE_ERRORS)
    # MSVC supports external includes only since Visual Studio 2019 version 16.10.0.
    # CMake supports external includes since 3.22.0 using the Ninja generator or NMake files (see https://gitlab.kitware.com/cmake/cmake/-/merge_requests/4766)
    # CMake supports external includes for Visual Studio also since 3.24.0 (see https://gitlab.kitware.com/cmake/cmake/-/merge_requests/7238)
    if(CMAKE_C_COMPILER_VERSION VERSION_LESS "19.29.30036.3" OR CMAKE_VERSION VERSION_LESS 3.22.0 OR (CMAKE_VERSION VERSION_LESS 3.24.0 AND CMAKE_GENERATOR MATCHES "Visual Studio"))
    message(WARNING "With the used combination of compiler and CMake version it is not recommended to activate PCL_WARNINGS_ARE_ERRORS, "
                    "because also warnings from 3rd party components are marked as errors. It is recommended to upgrade to "
                    "Visual Studio 2019 version 16.10.0 and CMake 3.24.0 (or CMake 3.22.0 if using Ninja or NMake files).")
    endif()
    list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/WX")
  endif()

  include(ProcessorCount)
  ProcessorCount(CPUCores)
  set(MSVC_MP ${CPUCores} CACHE STRING "Number of simultaneously running compilers (0 = automatic detection by MSVC). See documentation of /MP flag.")

  if(MSVC_MP EQUAL 0)
    # MSVC_MP is 0 in case the information cannot be determined by ProcessorCount => fallback
    # Generator expression is necessary to limit /MP flag to C/CXX, so flag will be not set to e.g. CUDA (see https://gitlab.kitware.com/cmake/cmake/issues/17535)
    add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP>)
  elseif(MSVC_MP GREATER 1)
    add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP${MSVC_MP}>)
  endif()

  if(CMAKE_GENERATOR STREQUAL "Ninja")
    list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "/FS")
  endif()
endif()

if(CMAKE_COMPILER_IS_PATHSCALE)
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-Wno-uninitialized" "-zerouv" "-mp")
  list(APPEND PCL_PRIVATE_LINK_OPTIONS "-mp")
endif()

if(CMAKE_COMPILER_IS_CLANG)
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-Wall" "-Wextra")
  # Unfortunately older Clang versions do not have this: -Wno-unnamed-type-template-args
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-ftemplate-depth=1024" "-Wno-invalid-offsetof")
  if(APPLE AND WITH_CUDA AND CUDA_FOUND)
    list(APPEND PCL_PRIVATE_COMPILER_OPTIONS "-stdlib=libstdc++")
  endif()
  set(CLANG_LIBRARIES "stdc++")
endif()

if(CMAKE_COMPILER_IS_MINGW)
    list(APPEND PCL_PRIVATE_DEFINITIONS "BOOST_THREAD_USE_LIB")
    list(APPEND PCL_PRIVATE_LINK_OPTIONS "-Wl,--allow-multiple-definition")
endif()

# OpenMP (optional)
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)
option(USE_HOMEBREW_FALLBACK "(macOS-only) also look in 'brew --prefix' for libraries (e.g. OpenMP)" TRUE)
if(WITH_OPENMP)
  find_package(OpenMP COMPONENTS C CXX)
  if(APPLE AND NOT OpenMP_FOUND)
    if(USE_HOMEBREW_FALLBACK)
      # libomp 15.0+ from brew is keg-only, so have to search in other locations.
      # See https://github.com/Homebrew/homebrew-core/issues/112107#issuecomment-1278042927.
      execute_process(COMMAND brew --prefix libomp
                      OUTPUT_VARIABLE HOMEBREW_LIBOMP_PREFIX
                      OUTPUT_STRIP_TRAILING_WHITESPACE)
      set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I${HOMEBREW_LIBOMP_PREFIX}/include")
      set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I${HOMEBREW_LIBOMP_PREFIX}/include")
      set(OpenMP_C_LIB_NAMES omp)
      set(OpenMP_CXX_LIB_NAMES omp)
      set(OpenMP_omp_LIBRARY ${HOMEBREW_LIBOMP_PREFIX}/lib/libomp.dylib)
      find_package(OpenMP COMPONENTS C CXX)
    endif()
  endif()
endif()
if(OpenMP_FOUND)
  list(APPEND PCL_PRIVATE_COMPILER_OPTIONS ${OpenMP_CXX_FLAGS})

  # We could use OpenMP_CXX_VERSION starting from CMake 3.9, but this value is only available on first run of CMake (see https://gitlab.kitware.com/cmake/cmake/issues/19150),
  # so we use always OpenMP_CXX_SPEC_DATE, which is available since CMake 3.7.
  message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
  if((MSVC_VERSION EQUAL 1900) OR (MSVC_VERSION MATCHES "^191[0-9]$"))
    if(${CMAKE_BUILD_TYPE} EQUAL Debug)
      list(APPEND PCL_PRIVATE_DEFINITIONS "/DELAYLOAD:VCOMP140D.dll")
    else()
      list(APPEND PCL_PRIVATE_DEFINITIONS "/DELAYLOAD:VCOMP140.dll")
    endif()
  endif()
else()
  message(STATUS "Not found OpenMP")
endif()

if(${PCL_ENABLE_CCACHE})
  include (UseCompilerCache)
  UseCompilerCache(ccache REQUIRED)
endif()


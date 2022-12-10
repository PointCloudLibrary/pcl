### ---[ PCL global CMake
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

if(POLICY CMP0074)
  # 1. Remove with 3.12.4.
  # 2. Remove search paths with *_ROOT since they will be automatically checked
  cmake_policy(SET CMP0074 NEW)
endif()

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(PCL_CXX_COMPILE_FEATURES cxx_std_14)

set(CMAKE_CUDA_STANDARD 14 CACHE STRING "The target CUDA/C++ standard. PCL requires CUDA/C++ 14 or higher.")
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif()

project(PCL VERSION 1.13.0)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# ---[ Include pkgconfig
include(FindPkgConfig)

if(WIN32 AND NOT MINGW)
  set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Add postfix to target for Debug build.")
  set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "Add postfix to target for Release build.")
  set(CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "Add postfix to target for RelWithDebInfo build.")
  set(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "Add postfix to target for MinSizeRel build")
endif()

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)

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

# https://github.com/fish-shell/fish-shell/issues/5865
include(CheckCXXSourceCompiles)
CHECK_CXX_SOURCE_COMPILES("
#include <atomic>
struct big { int foo[64]; };
std::atomic<big> x;
int main() {
   return x.load().foo[13];
}"
LIBATOMIC_NOT_NEEDED)
IF (NOT LIBATOMIC_NOT_NEEDED)
    SET(ATOMIC_LIBRARY "atomic")
ENDIF()

# Create a variable with expected default CXX flags
# This will be used further down the road to check if the user explicitly provided CXX flags
if(CMAKE_COMPILER_IS_MSVC)
  set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")
else()
  set(CMAKE_CXX_FLAGS_DEFAULT "")
endif()

include("${PCL_SOURCE_DIR}/cmake/pcl_verbosity.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_options.cmake")
include("${PCL_SOURCE_DIR}/cmake/clang-format.cmake")

if(${PCL_ENABLE_CCACHE})
  include (UseCompilerCache)
  UseCompilerCache(ccache REQUIRED)
endif()

# Enable verbose timing display?
if(CMAKE_TIMING_VERBOSE AND UNIX)
  set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_SOURCE_DIR}/cmake/custom_output.sh")
endif()

# check for SSE flags
include("${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake")
if(PCL_ENABLE_SSE AND "${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
  PCL_CHECK_FOR_SSE()
endif()

# check for AVX flags
if(PCL_ENABLE_AVX AND "${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_avx.cmake")
  PCL_CHECK_FOR_AVX()
endif()

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
      string(APPEND CMAKE_CXX_FLAGS " -Wabi=11")
    else()
      string(APPEND CMAKE_CXX_FLAGS " -Wabi")
    endif()
    string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion ${SSE_FLAGS} ${AVX_FLAGS}")
  endif()

  if(PCL_WARNINGS_ARE_ERRORS)
    string(APPEND CMAKE_CXX_FLAGS " -Werror -fno-strict-aliasing")
  endif()

  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "" AND NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed")
  endif()

  if(WIN32)
    if(PCL_SHARED_LIBS)
      string(APPEND CMAKE_SHARED_LINKER_FLAGS " -Wl,--export-all-symbols -Wl,--enable-auto-import")
      if(MINGW)
        add_definitions("-DBOOST_THREAD_USE_LIB")
        string(APPEND CMAKE_SHARED_LINKER_FLAGS " -Wl,--allow-multiple-definition")
      endif()
    else()
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif()
  endif()
endif()

if(CMAKE_COMPILER_IS_MSVC)
  add_definitions("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -DPCL_ONLY_CORE_POINT_TYPES ${SSE_DEFINITIONS}")
  
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    string(APPEND CMAKE_CXX_FLAGS " /fp:precise ${SSE_FLAGS} ${AVX_FLAGS} /bigobj")

    # Add extra code generation/link optimizations
    if(CMAKE_MSVC_CODE_LINK_OPTIMIZATION AND (NOT BUILD_CUDA) AND (NOT BUILD_GPU))
      string(APPEND CMAKE_CXX_FLAGS_RELEASE " /GL")
      string(APPEND CMAKE_SHARED_LINKER_FLAGS_RELEASE " /LTCG /OPT:REF")
      string(APPEND CMAKE_EXE_LINKER_FLAGS_RELEASE " /LTCG")
    else()
      set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/bigobj")
      
      message("Global optimizations /GL has been turned off, as it doesn't work with nvcc/thrust")
    endif()
    # /MANIFEST:NO") # please, don't disable manifest generation, otherwise crash at start for vs2008

    # Disable some warnings
    string(APPEND CMAKE_CXX_FLAGS " /wd4800 /wd4521 /wd4251 /wd4275 /wd4305 /wd4355")

    # Enable warnings, which are disabled by default (see https://learn.microsoft.com/de-de/cpp/preprocessor/compiler-warnings-that-are-off-by-default)
    string(APPEND CMAKE_CXX_FLAGS " /w34265")

    if(PCL_WARNINGS_ARE_ERRORS)
      # MSVC supports external includes only since Visual Studio 2019 version 16.10.0.
      # CMake supports external includes since 3.22.0 using the Ninja generator or NMake files (see https://gitlab.kitware.com/cmake/cmake/-/merge_requests/4766)
      # CMake supports external includes for Visual Studio also since 3.24.0 (see https://gitlab.kitware.com/cmake/cmake/-/merge_requests/7238)
      if(CMAKE_C_COMPILER_VERSION VERSION_LESS "19.29.30036.3" OR CMAKE_VERSION VERSION_LESS 3.22.0 OR (CMAKE_VERSION VERSION_LESS 3.24.0 AND CMAKE_GENERATOR MATCHES "Visual Studio"))
        message(WARNING "With the used combination of compiler and CMake version it is not recommended to activate PCL_WARNINGS_ARE_ERRORS, "
                        "because also warnings from 3rd party components are marked as errors. It is recommended to upgrade to "
                        "Visual Studio 2019 version 16.10.0 and CMake 3.24.0 (or CMake 3.22.0 if using Ninja or NMake files).")
      endif()
      string(APPEND CMAKE_CXX_FLAGS " /WX")
    endif()

    include(ProcessorCount)
    ProcessorCount(CPUCores)
    set(MSVC_MP ${CPUCores} CACHE STRING "Number of simultaneously running compilers (0 = automatic detection by MSVC). See documentation of /MP flag.")
    if (CMAKE_VERSION VERSION_LESS 3.11.0)
      # Usage of COMPILE_LANGUAGE generator expression for MSVC in add_compile_options requires at least CMake 3.11, see https://gitlab.kitware.com/cmake/cmake/issues/17435
      if(MSVC_MP EQUAL 0)
        # MSVC_MP is 0 in case the information cannot be determined by ProcessorCount => fallback
        string(APPEND CMAKE_C_FLAGS " /MP")
        string(APPEND CMAKE_CXX_FLAGS " /MP")
      elseif(MSVC_MP GREATER 1)
        string(APPEND CMAKE_C_FLAGS " /MP${MSVC_MP}")
        string(APPEND CMAKE_CXX_FLAGS " /MP${MSVC_MP}")
      endif()      
    else()      
      if(MSVC_MP EQUAL 0)
        # MSVC_MP is 0 in case the information cannot be determined by ProcessorCount => fallback
        # Generator expression is necessary to limit /MP flag to C/CXX, so flag will be not set to e.g. CUDA (see https://gitlab.kitware.com/cmake/cmake/issues/17535)
        add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP>)
      elseif(MSVC_MP GREATER 1)
        add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP${MSVC_MP}>)
      endif()
    endif()
  endif()

  if(CMAKE_GENERATOR STREQUAL "Ninja")
    string(APPEND CMAKE_C_FLAGS " /FS")
    string(APPEND CMAKE_CXX_FLAGS " /FS")
  endif()
endif()

if(CMAKE_COMPILER_IS_PATHSCALE)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    set(CMAKE_CXX_FLAGS "-Wno-uninitialized -zerouv -mp")
  endif()
  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "")
    set(CMAKE_SHARED_LINKER_FLAGS "-mp")
  endif()
endif()

if(CMAKE_COMPILER_IS_CLANG)
  if("${CMAKE_C_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    set(CMAKE_C_FLAGS "-Qunused-arguments")
  endif()
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    set(CMAKE_CXX_FLAGS "-ftemplate-depth=1024 -Qunused-arguments -Wno-invalid-offsetof ${SSE_FLAGS} ${AVX_FLAGS}") # Unfortunately older Clang versions do not have this: -Wno-unnamed-type-template-args
    if(APPLE AND WITH_CUDA AND CUDA_FOUND)
      string(APPEND CMAKE_CXX_FLAGS " -stdlib=libstdc++")
    endif()
  endif()
  set(CLANG_LIBRARIES "stdc++")
endif()

if(CMAKE_COMPILER_IS_MINGW)
  add_definitions(-DPCL_ONLY_CORE_POINT_TYPES)
endif()

include("${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

if(WIN32)
  set(PCL_RESOURCES_DIR "${PCL_SOURCE_DIR}/resources")
  set(PCL_POINTCLOUDS_DIR "${PCL_RESOURCES_DIR}/pointclouds")
endif()

set(PCL_OUTPUT_LIB_DIR "${PCL_BINARY_DIR}/${LIB_INSTALL_DIR}")
set(PCL_OUTPUT_BIN_DIR "${PCL_BINARY_DIR}/${BIN_INSTALL_DIR}")
make_directory("${PCL_OUTPUT_LIB_DIR}")
make_directory("${PCL_OUTPUT_BIN_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PCL_OUTPUT_LIB_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PCL_OUTPUT_BIN_DIR}")
if(WIN32)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PCL_OUTPUT_BIN_DIR}")
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${PCL_OUTPUT_LIB_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${PCL_OUTPUT_BIN_DIR}")
    # ---[ Windows requires DLLs (shared libraries) to be installed in the same directory as executables
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${PCL_OUTPUT_BIN_DIR}")
  endforeach()
else()
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PCL_OUTPUT_LIB_DIR}")
endif()

# Check if the current generator is an IDE.
check_if_ide(CMAKE_GENERATOR_IS_IDE)

# Add an "uninstall" target
if(CMAKE_GENERATOR_IS_IDE)
  set(UNINSTALL_TARGET_NAME UNINSTALL)
else()
  set(UNINSTALL_TARGET_NAME uninstall)
endif()
configure_file("${PCL_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
               "${PCL_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(${UNINSTALL_TARGET_NAME} "${CMAKE_COMMAND}" -P
                  "${PCL_BINARY_DIR}/uninstall_target.cmake")

###
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
###
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" is_system_dir)
if("${is_system_dir}" STREQUAL "-1")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
endif()

### ---[ Find universal dependencies

# OpenMP (optional)
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)
if(WITH_OPENMP)
  find_package(OpenMP COMPONENTS C CXX)
endif()
if(OpenMP_FOUND)
  string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
  string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")

  # We could use OpenMP_CXX_VERSION starting from CMake 3.9, but this value is only available on first run of CMake (see https://gitlab.kitware.com/cmake/cmake/issues/19150),
  # so we use always OpenMP_CXX_SPEC_DATE, which is available since CMake 3.7.
  message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")

  if((MSVC_VERSION EQUAL 1900) OR (MSVC_VERSION MATCHES "^191[0-9]$"))
    string(APPEND CMAKE_SHARED_LINKER_FLAGS_DEBUG " /DELAYLOAD:VCOMP140D.dll")
    string(APPEND CMAKE_SHARED_LINKER_FLAGS_RELEASE " /DELAYLOAD:VCOMP140.dll")
  endif()
else()
  message(STATUS "Not found OpenMP")
endif()

# Threads (required)
find_package(Threads REQUIRED)

# Eigen (required)
find_package(Eigen 3.3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

# FLANN (required)
find_package(FLANN 1.9.1 REQUIRED)
if(NOT (${FLANN_LIBRARY_TYPE} MATCHES ${PCL_FLANN_REQUIRED_TYPE}) AND NOT (${PCL_FLANN_REQUIRED_TYPE} MATCHES "DONTCARE"))
  message(FATAL_ERROR "Flann was selected with ${PCL_FLANN_REQUIRED_TYPE} but found as ${FLANN_LIBRARY_TYPE}")
endif()

# libusb
option(WITH_LIBUSB "Build USB RGBD-Camera drivers" TRUE)
if(WITH_LIBUSB)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_libusb.cmake")
endif()

# Dependencies for different grabbers
PCL_ADD_GRABBER_DEPENDENCY("OpenNI" "OpenNI grabber support")
PCL_ADD_GRABBER_DEPENDENCY("OpenNI2" "OpenNI2 grabber support")
PCL_ADD_GRABBER_DEPENDENCY("Ensenso" "IDS-Imaging Ensenso camera support")
PCL_ADD_GRABBER_DEPENDENCY("davidSDK" "David Vision Systems SDK support")
PCL_ADD_GRABBER_DEPENDENCY("DSSDK" "DepthSense SDK support")
PCL_ADD_GRABBER_DEPENDENCY("RSSDK" "RealSense SDK support")
PCL_ADD_GRABBER_DEPENDENCY("RSSDK2" "RealSense SDK 2.0 (librealsense) support")

# metslib
if(PKG_CONFIG_FOUND)
  pkg_check_modules(METSLIB metslib)
  if(METSLIB_FOUND)
    set(HAVE_METSLIB ON)
    include_directories(SYSTEM ${METSLIB_INCLUDE_DIRS})
  else()
    include_directories(SYSTEM "${PCL_SOURCE_DIR}/recognition/include/pcl/recognition/3rdparty/")
  endif()
else()
    include_directories(SYSTEM ${PCL_SOURCE_DIR}/recognition/include/pcl/recognition/3rdparty/)
endif()

# LibPNG
option(WITH_PNG "PNG file support" TRUE)
if(WITH_PNG)
  find_package(PNG)
  if(PNG_FOUND)
    set(HAVE_PNG ON)
    include_directories(SYSTEM "${PNG_INCLUDE_DIR}")
  endif()
endif()

# Qhull
option(WITH_QHULL "Include convex-hull operations" TRUE)
if(WITH_QHULL)
  find_package(Qhull)
  if(NOT (${QHULL_LIBRARY_TYPE} MATCHES ${PCL_QHULL_REQUIRED_TYPE}) AND NOT (${PCL_QHULL_REQUIRED_TYPE} MATCHES "DONTCARE"))
    message(FATAL_ERROR "Qhull was selected with ${PCL_QHULL_REQUIRED_TYPE} but found as ${QHULL_LIBRARY_TYPE}")
  endif()
endif()

# Cuda
option(WITH_CUDA "Build NVIDIA-CUDA support" TRUE)
if(WITH_CUDA)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_cuda.cmake")
endif()


# Reset VTK_FOUND to off
set(VTK_FOUND OFF)
# Find VTK - VTK has to be found before Qt, otherwise it can overwrite Qt variables
option(WITH_VTK "Build VTK-Visualizations" TRUE)
if(WITH_VTK)
  if(ANDROID)
    message(WARNING "VTK is not supported on Android.")
  else()
    include("${PCL_SOURCE_DIR}/cmake/pcl_find_vtk.cmake")
  endif()
endif()

# VTK can depend on Qt and search for its required version, so search after so we can overwrite Qt5_FOUND/Qt6_FOUND if the version we require is not found.
set(WITH_QT "AUTO" CACHE STRING "Build QT Front-End (AUTO|YES|QT6|QT5|NO)")
set_property(CACHE WITH_QT PROPERTY STRINGS "AUTO" "YES" "QT6" "QT5" "NO")
if(WITH_QT)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_qt.cmake")
endif()

#Find PCAP
option(WITH_PCAP "pcap file capabilities in Velodyne HDL driver" TRUE)
if(WITH_PCAP)
  find_package(Pcap)
endif()

# OpenGL and GLUT
option(WITH_OPENGL "Support for OpenGL" TRUE)
if(WITH_OPENGL)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_gl.cmake")
endif()

# Boost (required)
include("${PCL_SOURCE_DIR}/cmake/pcl_find_boost.cmake")

### ---[ Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file("${pcl_config_h_in}" "${pcl_config_h}")
PCL_ADD_INCLUDES(common "" "${pcl_config_h}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

collect_subproject_directory_names("${PCL_SOURCE_DIR}" "CMakeLists.txt" PCL_MODULES_NAMES PCL_MODULES_DIRS doc)
set(PCL_MODULES_NAMES_UNSORTED ${PCL_MODULES_NAMES})
topological_sort(PCL_MODULES_NAMES PCL_ _DEPENDS)
sort_relative(PCL_MODULES_NAMES_UNSORTED PCL_MODULES_NAMES PCL_MODULES_DIRS)
foreach(subdir ${PCL_MODULES_DIRS})
  add_subdirectory("${PCL_SOURCE_DIR}/${subdir}")
endforeach()

### ---[ Documentation
add_subdirectory(doc)

### ---[ Configure PCLConfig.cmake
include("${PCL_SOURCE_DIR}/cmake/pcl_pclconfig.cmake")

### ---[ Package creation
include("${PCL_SOURCE_DIR}/cmake/pcl_all_in_one_installer.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_cpack.cmake")

if(CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")
  PCL_MAKE_CPACK_INPUT()
  set(CPACK_PROJECT_CONFIG_FILE "${PCL_CPACK_CFG_FILE}")
  include(CPack)
endif()
### ---[ Make a pretty picture of the dependency graph
include("${PCL_SOURCE_DIR}/cmake/dep_graph.cmake")
MAKE_DEP_GRAPH()

### ---[ Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

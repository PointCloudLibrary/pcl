### ---[ PCL global CMake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

if(POLICY CMP0074)
  # 1. Remove with 3.12.4.
  # 2. Remove search paths with *_ROOT since they will be automatically checked
  cmake_policy(SET CMP0074 NEW)
endif()

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_VERSION VERSION_LESS 3.8)
  # CMake did not have cxx_std_14 compile feature prior to 3.8
  # We use cxx_attribute_deprecated as a proxy because this feature is a part of c++14 standard
  set(PCL_CXX_COMPILE_FEATURES cxx_attribute_deprecated)
else()
  set(PCL_CXX_COMPILE_FEATURES cxx_std_14)
endif()

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif()

project(PCL VERSION 1.11.0.99)
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
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_CLANG 1)
elseif(__COMPILER_PATHSCALE)
  set(CMAKE_COMPILER_IS_PATHSCALE 1)
elseif(MSVC)
  set(CMAKE_COMPILER_IS_MSVC 1)
elseif(MINGW)
  set(CMAKE_COMPILER_IS_MINGW 1)
endif()

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

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi=11")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion ${SSE_FLAGS_STR}")
    if(PCL_WARNINGS_ARE_ERRORS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
    endif()
  endif()

  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "" AND NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed")
  endif()

  if(WIN32)
    if(PCL_SHARED_LIBS)
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-all-symbols -Wl,--enable-auto-import")
      if(MINGW)
        add_definitions("-DBOOST_THREAD_USE_LIB")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--allow-multiple-definition")
      endif()
    else()
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif()
  endif()
endif()

if(CMAKE_COMPILER_IS_MSVC)
  add_definitions("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -DPCL_ONLY_CORE_POINT_TYPES ${SSE_DEFINITIONS}")
  
  add_compile_options(/bigobj)
  
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise /wd4800 /wd4521 /wd4251 /wd4275 /wd4305 /wd4355 ${SSE_FLAGS_STR}")

    # Add extra code generation/link optimizations
    if(CMAKE_MSVC_CODE_LINK_OPTIMIZATION)
      set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL")
      set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG /OPT:REF")
      set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    endif()
    # /MANIFEST:NO") # please, don't disable manifest generation, otherwise crash at start for vs2008

    if(PCL_WARNINGS_ARE_ERRORS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
    endif()

    include(ProcessorCount)
    ProcessorCount(CPUCores)
    set(MSVC_MP ${CPUCores} CACHE STRING "Number of simultaneously running compilers (0 = automatic detection by MSVC). See documentation of /MP flag.")
    if (CMAKE_VERSION VERSION_LESS 3.11.0)
      # Usage of COMPILE_LANGUAGE generator expression for MSVC in add_compile_options requires at least CMake 3.11, see https://gitlab.kitware.com/cmake/cmake/issues/17435	  
      if(MSVC_MP EQUAL 0)
        # MSVC_MP is 0 in case the information cannot be determined by ProcessorCount => fallback
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
      elseif(MSVC_MP GREATER 1)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP${MSVC_MP}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${MSVC_MP}")
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
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /FS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FS")
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
    set(CMAKE_CXX_FLAGS "-ftemplate-depth=1024 -Qunused-arguments -Wno-invalid-offsetof ${SSE_FLAGS_STR}") # Unfortunately older Clang versions do not have this: -Wno-unnamed-type-template-args
    if(APPLE AND WITH_CUDA AND CUDA_FOUND)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
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
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  if(${CMAKE_VERSION} VERSION_LESS "3.7")
    message(STATUS "Found OpenMP")
  else()
    # We could use OpenMP_CXX_VERSION starting from CMake 3.9, but this value is only available on first run of CMake (see https://gitlab.kitware.com/cmake/cmake/issues/19150),
    # so we use always OpenMP_CXX_SPEC_DATE, which is available since CMake 3.7.
    message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
  endif()
  if(MSVC)
    if(MSVC_VERSION EQUAL 1900)
      set(OPENMP_DLL VCOMP140)
    elseif(MSVC_VERSION MATCHES "^191[0-9]$")
      set(OPENMP_DLL VCOMP140)
    elseif(MSVC_VERSION MATCHES "^192[0-9]$")
      set(OPENMP_DLL VCOMP140)
    endif()
    if(OPENMP_DLL)
      set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
      set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
    else()
      message(WARNING "Delay loading flag for OpenMP DLL is invalid.")
    endif()
  endif()
else()
  message(STATUS "Not found OpenMP")
endif()

# Threads (required)
find_package(Threads REQUIRED)

# Eigen (required)
find_package(Eigen 3.1 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

# FLANN (required)
if(NOT PCL_SHARED_LIBS OR ((WIN32 AND NOT MINGW) AND NOT PCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32))
  set(FLANN_USE_STATIC ON)
endif()
find_package(FLANN 1.7.0 REQUIRED)

# libusb-1.0
option(WITH_LIBUSB "Build USB RGBD-Camera drivers" TRUE)
if(WITH_LIBUSB)
  find_package(libusb-1.0)
  if(LIBUSB_1_FOUND)
    include_directories(SYSTEM "${LIBUSB_1_INCLUDE_DIR}")
  endif()
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
  if(NOT PCL_SHARED_LIBS OR ((WIN32 AND NOT MINGW) AND NOT PCL_BUILD_WITH_QHULL_DYNAMIC_LINKING_WIN32))
    set(QHULL_USE_STATIC ON)
  endif()
  find_package(Qhull)
  if(QHULL_FOUND)
    include_directories(SYSTEM ${QHULL_INCLUDE_DIRS})
  endif()
endif()

# Cuda
option(WITH_CUDA "Build NVIDIA-CUDA support" TRUE)
if(WITH_CUDA)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_cuda.cmake")
endif()

option(WITH_QT "Build QT Front-End" TRUE)
if(WITH_QT)
  find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets QUIET)
endif()

# Find VTK
option(WITH_VTK "Build VTK-Visualizations" TRUE)
if(WITH_VTK AND NOT ANDROID)
  set(PCL_VTK_COMPONENTS
    vtkChartsCore
    vtkCommonCore
    vtkCommonDataModel
    vtkCommonExecutionModel
    vtkFiltersCore
    vtkFiltersExtraction
    vtkFiltersModeling
    vtkImagingCore
    vtkImagingSources
    vtkInteractionStyle
    vtkInteractionWidgets
    vtkIOCore
    vtkIOGeometry
    vtkIOImage
    vtkIOLegacy
    vtkIOPLY
    vtkRenderingAnnotation
    vtkRenderingLOD
    vtkViewsContext2D
  )
  find_package(VTK COMPONENTS ${PCL_VTK_COMPONENTS})
  if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 6.2))
    message(WARNING "The minimum required version of VTK is 6.2, but found ${VTK_VERSION}")
    set(VTK_FOUND FALSE)
  endif()

  if(VTK_FOUND)
    if(NOT DEFINED VTK_RENDERING_BACKEND)
      # On old VTK versions this variable does not exist. In this case it is
      # safe to assume OpenGL backend
      set(VTK_RENDERING_BACKEND "OpenGL")
    endif()
    list(APPEND PCL_VTK_COMPONENTS vtkRenderingContext${VTK_RENDERING_BACKEND})

    if(WITH_QT)
      if(";${VTK_MODULES_ENABLED};" MATCHES ";vtkGUISupportQt;" AND ";${VTK_MODULES_ENABLED};" MATCHES ";vtkRenderingQt;")
        set(QVTK_FOUND ON)
        list(APPEND PCL_VTK_COMPONENTS vtkRenderingQt vtkGUISupportQt)
      else()
        unset(QVTK_FOUND)
      endif()
    endif()

    find_package(VTK COMPONENTS ${PCL_VTK_COMPONENTS})

    message(STATUS "VTK_MAJOR_VERSION ${VTK_MAJOR_VERSION}, rendering backend: ${VTK_RENDERING_BACKEND}")
    if(PCL_SHARED_LIBS OR (NOT (PCL_SHARED_LIBS) AND NOT (VTK_BUILD_SHARED_LIBS)))
      if(VTK_USE_FILE)
        include(${VTK_USE_FILE})
      endif()
      message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, libs: ${VTK_LIBRARIES}")
      if(APPLE)
        option(VTK_USE_COCOA "Use Cocoa for VTK render windows" ON)
        mark_as_advanced(VTK_USE_COCOA)
      endif()
      if(${VTK_RENDERING_BACKEND} STREQUAL "OpenGL")
        set(VTK_RENDERING_BACKEND_OPENGL_VERSION "1")
        message(DEPRECATION "The rendering backend OpenGL is deprecated and not available anymore since VTK 8.2."
                            "Please switch to the OpenGL2 backend instead, which is available since VTK 6.2."
                            "Support of the deprecated backend will be dropped with PCL 1.13.")
      elseif(${VTK_RENDERING_BACKEND} STREQUAL "OpenGL2")
        set(VTK_RENDERING_BACKEND_OPENGL_VERSION "2")
      endif()
    else()
      set(VTK_FOUND OFF)
      message("Warning: You are to build PCL in STATIC but VTK is SHARED!")
      message("Warning: VTK disabled!")
    endif()
  endif()
else()
  set(VTK_FOUND OFF)
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

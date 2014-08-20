### ---[ PCL global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD) # do not use VERSION option in project() command
endif()

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# ---[ Include pkgconfig
include (FindPkgConfig)

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()
if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "_debug")
  endif()
  if(NOT DEFINED CMAKE_RELEASE_POSTFIX)
    set(CMAKE_RELEASE_POSTFIX "_release")
  endif()
  if(NOT DEFINED CMAKE_RELWITHDEBINFO_POSTFIX)
    set(CMAKE_RELWITHDEBINFO_POSTFIX "_release")
  endif()
  if(NOT DEFINED CMAKE_MINSIZEREL_POSTFIX)
    set(CMAKE_MINSIZEREL_POSTFIX "_release")
  endif()
endif()

# ---[ special maintainer mode
SET(CMAKE_CXX_FLAGS_MAINTAINER "-pedantic -Wno-variadic-macros -Weffc++ -Wno-long-long" CACHE STRING
    "Flags used by the C++ compiler during maintainer builds."
    FORCE)
SET(CMAKE_C_FLAGS_MAINTAINER "-pedantic -Wno-variadic-macros -Weffc++ -Wno-long-long" CACHE STRING
    "Flags used by the C compiler during maintainer builds."
    FORCE)
SET(CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used for linking binaries during maintainer builds."
    FORCE)
SET(CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used by the shared libraries linker during maintainer builds."
    FORCE)
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_MAINTAINER
    CMAKE_C_FLAGS_MAINTAINER
    CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    CMAKE_SHARED_LINKER_FLAGS_MAINTAINER )
# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

# ---[ Android check
if (ANDROID)
  set (PCL_SHARED_LIBS OFF)
  message ("PCL shared libs on Android must be: ${PCL_SHARED_LIBS}")
endif()

include("${PCL_SOURCE_DIR}/cmake/pcl_verbosity.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_options.cmake")

# Enable verbose timing display?
if(CMAKE_TIMING_VERBOSE AND UNIX)
  set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_SOURCE_DIR}/cmake/custom_output.sh")
endif(CMAKE_TIMING_VERBOSE AND UNIX)

# check for SSE flags
include("${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake")
if (PCL_ENABLE_SSE)
  PCL_CHECK_FOR_SSE()
endif (PCL_ENABLE_SSE)

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion ${SSE_FLAGS}")

    # Enable -Wabi for GCC > 4.3, and -Wno-deprecated for GCC < 4.3
    # to disable a lot of warnings which are not fixable
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (GCC_VERSION VERSION_GREATER 4.3)
      message(STATUS "-- GCC > 4.3 found, enabling -Wabi")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    else()
      message(STATUS "-- GCC < 4.3 found, enabling -Wno-deprecated")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
    endif ()
  endif()

  if(NOT ANDROID)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  endif(NOT ANDROID)

  if(WIN32)
    if(PCL_SHARED_LIBS)
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-all-symbols -Wl,--enable-auto-import")
      if (MINGW)
        add_definitions("-DBOOST_THREAD_USE_LIB")
      endif()
    else(PCL_SHARED_LIBS)
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif(PCL_SHARED_LIBS)
  endif()
endif()

if(MSVC)
  SET(CMAKE_COMPILER_IS_MSVC 1)
  add_definitions ("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX /bigobj")
  if("${CMAKE_CXX_FLAGS}" STREQUAL " /DWIN32 /D_WINDOWS /W3 /GR /EHsc")	# Check against default flags
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /EHsc /fp:precise /wd4800 /wd4521 /wd4251 /wd4275 /wd4305 /wd4355 ${SSE_FLAGS}")

    # Add extra code generation/link optimizations
    if(CMAKE_MSVC_CODE_LINK_OPTIMIZATION)
      SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL")
      SET(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
      SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    endif(CMAKE_MSVC_CODE_LINK_OPTIMIZATION)
    # /MANIFEST:NO") # please, don't disable manifest generation, otherwise crash at start for vs2008

    if( MSVC_VERSION GREATER 1500 AND ${CMAKE_VERSION} VERSION_GREATER "2.8.6")
      include(ProcessorCount)
      ProcessorCount(N)
      if(NOT N EQUAL 0)
        SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP${N} ")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${N} ")
      endif()
    endif()
  endif()
endif()

if (__COMPILER_PATHSCALE)
  SET(CMAKE_COMPILER_IS_PATHSCALE 1)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    SET(CMAKE_CXX_FLAGS "-Wno-uninitialized -zerouv -pthread -mp")
  endif()
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET(CMAKE_COMPILER_IS_CLANG 1)
  if("${CMAKE_C_FLAGS}" STREQUAL "")
    SET(CMAKE_C_FLAGS "-Qunused-arguments")
  endif()
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    SET(CMAKE_CXX_FLAGS "-ftemplate-depth=1024 -Qunused-arguments -Wno-invalid-offsetof ${SSE_FLAGS}") # Unfortunately older Clang versions do not have this: -Wno-unnamed-type-template-args
    if(APPLE AND WITH_CUDA AND CUDA_FOUND)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
    endif()
  endif()
  SET(CLANG_LIBRARIES "stdc++")
endif()

include("${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake")
set(PCL_VERSION 1.7.2 CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

if(WIN32)
  set(PCL_RESOURCES_DIR "${PCL_SOURCE_DIR}/resources")
  set(PCL_POINTCLOUDS_DIR "${PCL_RESOURCES_DIR}/pointclouds")
endif(WIN32)

set(PCL_OUTPUT_LIB_DIR "${PCL_BINARY_DIR}/${LIB_INSTALL_DIR}")
set(PCL_OUTPUT_BIN_DIR "${PCL_BINARY_DIR}/${BIN_INSTALL_DIR}")
make_directory("${PCL_OUTPUT_LIB_DIR}")
make_directory("${PCL_OUTPUT_BIN_DIR}")
if(WIN32)
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${PCL_OUTPUT_LIB_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${PCL_OUTPUT_BIN_DIR}")
    # ---[ Windows requires DLLs (shared libraries) to be installed in the same directory as executables
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${PCL_OUTPUT_BIN_DIR}")
  endforeach(config)
else(WIN32)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PCL_OUTPUT_LIB_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PCL_OUTPUT_BIN_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PCL_OUTPUT_LIB_DIR}")
endif(WIN32)

# Add an "uninstall" target
configure_file("${PCL_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
               "${PCL_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${PCL_BINARY_DIR}/uninstall_target.cmake")

###
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
###
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
ENDIF("${is_system_dir}" STREQUAL "-1")

### ---[ Find universal dependencies
# the gcc-4.2.1 coming with MacOS X is not compatible with the OpenMP pragmas we use, so disabling OpenMP for it
if((NOT APPLE) OR (NOT CMAKE_COMPILER_IS_GNUCXX) OR (GCC_VERSION VERSION_GREATER 4.2.1) OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
  find_package(OpenMP)
endif()
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
  if(MSVC90 OR MSVC10)
    if(MSVC90)
      set(OPENMP_DLL VCOMP90)
    elseif(MSVC10)
      set(OPENMP_DLL VCOMP100)
    endif(MSVC90)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
  endif(MSVC90 OR MSVC10)
else(OPENMP_FOUND)
  message (STATUS "Not found OpenMP")
endif()
# Boost (required)
include("${PCL_SOURCE_DIR}/cmake/pcl_find_boost.cmake")
# Eigen (required)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
# FLANN (required)
if(NOT PCL_SHARED_LIBS OR (WIN32 AND NOT MINGW))
  set(FLANN_USE_STATIC ON)
endif(NOT PCL_SHARED_LIBS OR (WIN32 AND NOT MINGW))
find_package(FLANN 1.7.0 REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})

# libusb-1.0
option(WITH_LIBUSB "Build USB RGBD-Camera drivers" TRUE)
if(WITH_LIBUSB)
  find_package(libusb-1.0)
  if(LIBUSB_1_FOUND)
    include_directories("${LIBUSB_1_INCLUDE_DIR}")
  endif(LIBUSB_1_FOUND)
endif(WITH_LIBUSB)

# OpenNI
option(WITH_OPENNI "OpenNI driver support" TRUE)
if(WITH_OPENNI)
  find_package(OpenNI)
  if (OPENNI_FOUND)
    set(HAVE_OPENNI ON)
    include_directories(SYSTEM ${OPENNI_INCLUDE_DIRS})
  endif(OPENNI_FOUND)
endif(WITH_OPENNI)

# OpenNI 2
option(WITH_OPENNI2 "OpenNI 2 driver support" TRUE)
if(WITH_OPENNI2)
  find_package(OpenNI2)
  if (OPENNI2_FOUND)
    set(HAVE_OPENNI2 ON)
    include_directories(SYSTEM ${OPENNI2_INCLUDE_DIRS})
  endif(OPENNI2_FOUND)
endif(WITH_OPENNI2)

# Fotonic (FZ_API)
option(WITH_FZAPI "Build Fotonic Camera support" TRUE)
if(WITH_FZAPI)
  find_package(FZAPI)
  if (FZAPI_FOUND)
    set(HAVE_FZAPI ON)
    include_directories(SYSTEM "${FZAPI_INCLUDE_DIR}")
  endif(FZAPI_FOUND)
endif(WITH_FZAPI)

# Intel Perceptional Computing Interface (PXCAPI)
option(WITH_PXCAPI "Build PXC Device support" TRUE)
if(WITH_PXCAPI)
  find_package(PXCAPI)
  if (PXCAPI_FOUND)
    set(HAVE_PXCAPI ON)
    include_directories(SYSTEM ${PXCAPI_INCLUDE_DIRS})
  endif(PXCAPI_FOUND)
endif(WITH_PXCAPI)

# metslib
if (PKG_CONFIG_FOUND)
  pkg_check_modules(METSLIB metslib)
  if (METSLIB_FOUND)
    set (HAVE_METSLIB ON)
    include_directories(${METSLIB_INCLUDE_DIRS})
  else()
    include_directories("${PCL_SOURCE_DIR}/recognition/include/pcl/recognition/3rdparty/")
  endif()
else()
    include_directories(${PCL_SOURCE_DIR}/recognition/include/pcl/recognition/3rdparty/)
endif()

# LibPNG
option(WITH_PNG "PNG file support" TRUE)
if(WITH_PNG)
  find_package(PNG)
  if (PNG_FOUND)
    set (HAVE_PNG ON)
    include_directories("${PNG_INCLUDE_DIR}")
  endif(PNG_FOUND)
endif(WITH_PNG)

# Qhull
option(WITH_QHULL "Include convex-hull operations" TRUE)
if(WITH_QHULL)
  if(NOT PCL_SHARED_LIBS OR WIN32)
    set(QHULL_USE_STATIC ON)
  endif(NOT PCL_SHARED_LIBS OR WIN32)
  find_package(Qhull)
endif(WITH_QHULL)

option(WITH_QT "Build QT Front-End" TRUE)
if(WITH_QT)
  # Find Qt4
  find_package(Qt4)
  if (QT4_FOUND)
    include("${QT_USE_FILE}")
  endif (QT4_FOUND)

  # Find QT5
  if(NOT QT4_FOUND)
      include(cmake/pcl_find_qt5.cmake)
  endif(NOT QT4_FOUND)
endif(WITH_QT)

# Find VTK
option(WITH_VTK "Build VTK-Visualizations" TRUE)
if(WITH_VTK AND NOT ANDROID)
  find_package(VTK)
  if(VTK_FOUND)
    message(STATUS "VTK_MAJOR_VERSION ${VTK_MAJOR_VERSION}")
    if (PCL_SHARED_LIBS OR
        (NOT (PCL_SHARED_LIBS) AND NOT (VTK_BUILD_SHARED_LIBS)))
      set(VTK_FOUND TRUE)
      find_package (QVTK)
      if (${VTK_MAJOR_VERSION} VERSION_LESS "6.0")
         message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
         link_directories(${VTK_LIBRARY_DIRS})
      else(${VTK_MAJOR_VERSION} VERSION_LESS "6.0")
         include (${VTK_USE_FILE})
         message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARIES}")
      endif (${VTK_MAJOR_VERSION} VERSION_LESS "6.0")
      if (APPLE)
          option (VTK_USE_COCOA "Use Cocoa for VTK render windows" ON)
          MARK_AS_ADVANCED (VTK_USE_COCOA)
      endif (APPLE)
      set(HAVE_VTK ON)
    else ()
      set(VTK_FOUND OFF)
      set(HAVE_VTK OFF)
      message ("Warning: You are to build PCL in STATIC but VTK is SHARED!")
      message ("Warning: VTK disabled!")
    endif ()
  endif(VTK_FOUND)
else(WITH_VTK AND NOT ANDROID)
  set(VTK_FOUND OFF)
  set(HAVE_VTK OFF)
endif(WITH_VTK AND NOT ANDROID)


#Find PCAP
option(WITH_PCAP "pcap file capabilities in Velodyne HDL driver" TRUE)
if(WITH_PCAP)
  find_package(Pcap)
endif(WITH_PCAP)

# OpenGL and GLUT
include("${PCL_SOURCE_DIR}/cmake/pcl_find_gl.cmake")

### ---[ Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file("${pcl_config_h_in}" "${pcl_config_h}")
PCL_ADD_INCLUDES(common "" "${pcl_config_h}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

### ---[ Set up for tests
enable_testing()

### ---[ Set up for examples
#include("${PCL_SOURCE_DIR}/cmake/pcl_examples.cmake")

### ---[ Add the libraries subdirectories
include("${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake")

collect_subproject_directory_names("${PCL_SOURCE_DIR}" "CMakeLists.txt" PCL_MODULES_NAMES PCL_MODULES_DIRS doc)
set(PCL_MODULES_NAMES_UNSORTED ${PCL_MODULES_NAMES})
topological_sort(PCL_MODULES_NAMES PCL_ _DEPENDS)
sort_relative(PCL_MODULES_NAMES_UNSORTED PCL_MODULES_NAMES PCL_MODULES_DIRS)
foreach(subdir ${PCL_MODULES_DIRS})
  add_subdirectory("${PCL_SOURCE_DIR}/${subdir}")
endforeach(subdir)

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
endif(CPACK_GENERATOR)
### ---[ Make a pretty picture of the dependency graph
include("${PCL_SOURCE_DIR}/cmake/dep_graph.cmake")
MAKE_DEP_GRAPH()

### ---[ Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

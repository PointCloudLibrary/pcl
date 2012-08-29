### ---[ PCL global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)
# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()
if(WIN32)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "_debug")
  endif()
  if(NOT DEFINED CMAKE_RELEASE_POSTFIX)
    set(CMAKE_RELEASE_POSTFIX "_release")
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

include(${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake)
include(${PCL_SOURCE_DIR}/cmake/pcl_options.cmake)

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -Wconversion -Wold-style-cast -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof")

  # Enable -Wabi for GCC > 4.3, and -Wno-deprecated for GCC < 4.3 
  # to disable a lot of warnings which are not fixable
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.3)
    message("-- GCC > 4.3 found, enabling -Wabi")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
  else()
    message("-- GCC < 4.3 found, enabling -Wno-deprecated")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
  endif ()


  if(NOT ANDROID)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  endif(NOT ANDROID)
  if(WIN32)
    if(PCL_SHARED_LIBS)
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-all-symbols -Wl,--enable-auto-import")
    else(PCL_SHARED_LIBS)
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif(PCL_SHARED_LIBS)
  endif()
endif()

if(MSVC)
  add_definitions ("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /EHsc /fp:fast /wd4800 /wd4521 /wd4251 /wd4275")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL /wd4800")
  SET(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
  SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
  # /MANIFEST:NO") # please, don't disable manifest generation, otherwise crash at start for vs2008
  
  if(MSVC90 OR MSVC10 AND NOT ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.8 AND NOT ${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} LESS 8.6)
    include(ProcessorCount)
    ProcessorCount(N)
    if(NOT N EQUAL 0)
      SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP${N} ")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${N} ")
    endif()
  endif()
endif()

# check for SSE flags
include(${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()

if (__COMPILER_PATHSCALE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-uninitialized -zerouv -pthread -mp")
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-unnamed-type-template-args")
  SET(CLANG_LIBRARIES "stdc++")
endif()

# ---[ Project folders
option(USE_PROJECT_FOLDERS "Use folders to organize PCL projects in an IDE." OFF)
mark_as_advanced(USE_PROJECT_FOLDERS)
if(USE_PROJECT_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif(USE_PROJECT_FOLDERS)

include(${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake)
set(PCL_VERSION 1.7.0 CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

if(WIN32)
  set(PCL_RESOURCES_DIR ${PCL_SOURCE_DIR}/resources)
  set(PCL_POINTCLOUDS_DIR ${PCL_RESOURCES_DIR}/pointclouds)
endif(WIN32)

set(PCL_OUTPUT_LIB_DIR ${PCL_BINARY_DIR}/${LIB_INSTALL_DIR})
set(PCL_OUTPUT_BIN_DIR ${PCL_BINARY_DIR}/${BIN_INSTALL_DIR})
make_directory(${PCL_OUTPUT_LIB_DIR})
make_directory(${PCL_OUTPUT_BIN_DIR})
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
if((NOT APPLE) OR (NOT CMAKE_COMPILER_IS_GNUCXX) OR (GCC_VERSION VERSION_GREATER 4.2.1))
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
include(${PCL_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
# Eigen (required)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
# FLANN (required)
if(NOT PCL_SHARED_LIBS OR WIN32)
  set(FLANN_USE_STATIC ON)
endif(NOT PCL_SHARED_LIBS OR WIN32)
find_package(FLANN 1.7.0 REQUIRED)
include_directories(SYSTEM ${FLANN_INCLUDE_DIRS})

# libusb-1.0
find_package(libusb-1.0)
if(LIBUSB_1_FOUND)
  include_directories(${LIBUSB_1_INCLUDE_DIR})
endif(LIBUSB_1_FOUND)

# OpenNI
find_package(OpenNI)
if (OPENNI_FOUND)
  set(HAVE_OPENNI ON)
  include_directories(SYSTEM ${OPENNI_INCLUDE_DIRS})
endif()
# LibPNG
find_package(PNG)
if (PNG_FOUND)
  set (HAVE_PNG ON)
  include_directories(${PNG_INCLUDE_DIR})
endif(PNG_FOUND)

# Qhull
if(NOT PCL_SHARED_LIBS OR WIN32)
  set(QHULL_USE_STATIC ON)
endif(NOT PCL_SHARED_LIBS OR WIN32)
find_package(Qhull)

# Cuda
include(${PCL_SOURCE_DIR}/cmake/pcl_find_cuda.cmake)

# Find ROS
include(${PCL_SOURCE_DIR}/cmake/pcl_find_ros.cmake)
# Find QT4
find_package(Qt4)
if (QT4_FOUND)
  include(${QT_USE_FILE})
endif (QT4_FOUND)
# Find VTK
find_package(VTK)
if(VTK_FOUND)
  if (PCL_SHARED_LIBS OR 
      (NOT (PCL_SHARED_LIBS) AND NOT (VTK_BUILD_SHARED_LIBS)))
    set(VTK_FOUND TRUE)
    find_package (QVTK)
    message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
    link_directories(${VTK_LIBRARY_DIRS})
    set(HAVE_VTK ON)
  else ()
    set(VTK_FOUND OFF)
    set(HAVE_VTK OFF)
    message ("Warning: You are to build PCL in STATIC but VTK is SHARED!")
    message ("Warning: VTK disabled!")
  endif ()
endif(VTK_FOUND)
# Find MPI
if (WITH_MPI) # this script searches for MPI 10 sec under windows, annoying especially if you do this often
  find_package(MPI)
  if(MPI_CXX_FOUND)
    include_directories(SYSTEM ${MPI_INCLUDE_PATH})
  endif(MPI_CXX_FOUND)
endif()
#Find Doxygen and html help compiler if any
find_package(Doxygen)
if(DOXYGEN_FOUND)
  find_package(HTMLHelp)
endif(DOXYGEN_FOUND)

### ---[ Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file(${pcl_config_h_in} ${pcl_config_h})
PCL_ADD_INCLUDES(common "" ${pcl_config_h})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

### ---[ Set up for tests
include(${PCL_SOURCE_DIR}/cmake/pcl_tests.cmake)

### ---[ Set up for examples
#include(${PCL_SOURCE_DIR}/cmake/pcl_examples.cmake)

### ---[ Add the libraries subdirectories
include(${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake)

collect_subproject_directory_names(${PCL_SOURCE_DIR} "CMakeLists.txt" PCL_MODULES_NAMES PCL_MODULES_DIRS doc)
set(PCL_MODULES_NAMES_UNSORTED ${PCL_MODULES_NAMES})
topological_sort(PCL_MODULES_NAMES PCL_ _DEPENDS)
sort_relative(PCL_MODULES_NAMES_UNSORTED PCL_MODULES_NAMES PCL_MODULES_DIRS)
foreach(subdir ${PCL_MODULES_DIRS})
  add_subdirectory(${PCL_SOURCE_DIR}/${subdir})  
endforeach(subdir)

### ---[ Documentation
add_subdirectory(doc)

### ---[ Configure PCLConfig.cmake
include(${PCL_SOURCE_DIR}/cmake/pcl_pclconfig.cmake)

### ---[ Package creation
include(${PCL_SOURCE_DIR}/cmake/pcl_all_in_one_installer.cmake)
include(${PCL_SOURCE_DIR}/cmake/pcl_cpack.cmake)

if(CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")
  PCL_MAKE_CPACK_INPUT()
  set(CPACK_PROJECT_CONFIG_FILE "${PCL_CPACK_CFG_FILE}")
  include(CPack)
endif(CPACK_GENERATOR)
### ---[ Make a pretty picture of the dependency graph
include(${PCL_SOURCE_DIR}/cmake/dep_graph.cmake)
MAKE_DEP_GRAPH()

### ---[ Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

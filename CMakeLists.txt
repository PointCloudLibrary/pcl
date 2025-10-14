### ---[ PCL global CMake
cmake_minimum_required(VERSION 3.16.3 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif()

project(PCL VERSION 1.15.1.99)
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

# Ensure try_compile sets the include paths (e.g. for Eigen3)
set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})

# Check if we need to link against libatomic (armel)
include("${PCL_SOURCE_DIR}/cmake/pcl_check_atomic.cmake")

include("${PCL_SOURCE_DIR}/cmake/pcl_verbosity.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_options.cmake")
include("${PCL_SOURCE_DIR}/cmake/clang-format.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

# Check/Set compiler settings
include("cmake/pcl_set_compiler_settings.cmake")
message(STATUS "PCL public Options: ${PCL_PUBLIC_COMPILER_OPTIONS}")
message(STATUS "PCL private Options: ${PCL_PRIVATE_COMPILER_OPTIONS}")
message(STATUS "PCL public Definitions: ${PCL_PUBLIC_COMPILER_DEFINITIONS}")
message(STATUS "PCL private Definitions: ${PCL_PRIVATE_COMPILER_DEFINITIONS}")
message(STATUS "PCL public Compile Features: ${PCL_PUBLIC_COMPILER_FEATURES}")
message(STATUS "PCL private Compile Features: ${PCL_PRIVATE_COMPILER_FEATURES}")
message(STATUS "PCL private Link Options: ${PCL_PRIVATE_LINK_OPTIONS}")

# Enable verbose timing display?
if(CMAKE_TIMING_VERBOSE AND UNIX)
  set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_SOURCE_DIR}/cmake/custom_output.sh")
endif()

# Cuda
option(WITH_CUDA "Build NVIDIA-CUDA support" TRUE)
if(WITH_CUDA)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_cuda.cmake")
endif()

if(WIN32)
  set(PCL_RESOURCES_DIR "${PCL_SOURCE_DIR}/resources")
  set(PCL_POINTCLOUDS_DIR "${PCL_RESOURCES_DIR}/pointclouds")
endif()

#Set up output directory for builds
set(PCL_OUTPUT_LIB_DIR "${PCL_BINARY_DIR}/${LIB_INSTALL_DIR}")
set(PCL_OUTPUT_BIN_DIR "${PCL_BINARY_DIR}/${BIN_INSTALL_DIR}")
file(MAKE_DIRECTORY "${PCL_OUTPUT_LIB_DIR}")
file(MAKE_DIRECTORY "${PCL_OUTPUT_BIN_DIR}")
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
# Threads (required)
find_package(Threads REQUIRED)

# Eigen3 (required)
find_package(Eigen3 REQUIRED NO_MODULE)
if (Eigen3_VERSION VERSION_LESS 3.3)
  message(FATAL_ERROR "Eigen version>=3.3 is required, but found ${Eigen3_VERSION}")
endif()
if(NOT EIGEN3_FOUND AND Eigen3_FOUND)
    set(EIGEN3_FOUND ${Eigen3_FOUND})
endif()

# FLANN
find_package(FLANN 1.9.1)
if(NOT FLANN_FOUND)
  message(WARNING "Flann was not found, so many PCL modules will not be built!")
else()
  set(PCL_HAS_FLANN 1)
  if(NOT (${FLANN_LIBRARY_TYPE} MATCHES ${PCL_FLANN_REQUIRED_TYPE}) AND NOT (${PCL_FLANN_REQUIRED_TYPE} MATCHES "DONTCARE"))
    message(FATAL_ERROR "Flann was selected with ${PCL_FLANN_REQUIRED_TYPE} but found as ${FLANN_LIBRARY_TYPE}")
  endif()
endif()

find_package(nanoflann 1.4.2 QUIET)
if(TARGET nanoflann::nanoflann)
  set(PCL_HAS_NANOFLANN 1)
  message(STATUS "Found nanoflann")
else()
  message(STATUS "nanoflann was not found, so some PCL classes will not be available.")
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

# LibPNG
option(WITH_PNG "PNG file support" TRUE)
if(WITH_PNG)
  find_package(PNG)
  if(PNG_FOUND)
    set(HAVE_PNG ON)
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

# find GLEW before VTK as it uses custom findGLEW that doesn't work with cmakes findGLEW.
option(WITH_GLEW "Support for GLEW" TRUE)
if(WITH_GLEW)
  find_package(GLEW QUIET)
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

# System zlib (for nurbs on surface)
option(WITH_SYSTEM_ZLIB "Use system zlib" TRUE)
if(WITH_SYSTEM_ZLIB)
  find_package(ZLIB)
  if(ZLIB_FOUND)
    set(HAVE_ZLIB ON)
  endif()
else()
  message(WARNING "CMake will use a ZLIB version bundled with the PCL source code. However, that is an older version which may pose a risk and may be removed in a future PCL release. It is recommended to install an up-to-date version of ZLIB on your system and set WITH_SYSTEM_ZLIB=TRUE.")
endif()

option(WITH_SYSTEM_CJSON "Use system cJSON" TRUE)
if(WITH_SYSTEM_CJSON)
  find_package(cJSON)
  if(cJSON_FOUND)
    set(HAVE_CJSON ON)
  else()
    message(WARNING "It is recommended to install an up-to-date version of cJSON on your system. CMake did not find one, and will instead use a cJSON version bundled with the PCL source code. However, that is an older version which may pose a risk and may be removed in a future PCL release.")
  endif()
endif()

set(CMAKE_REQUIRED_LIBRARIES Eigen3::Eigen) # so that Eigen/Core is found below
string(REPLACE ";" " " PCL_PUBLIC_COMPILE_OPTIONS_STR "${PCL_PUBLIC_COMPILER_OPTIONS}")
set(CMAKE_REQUIRED_FLAGS ${PCL_PUBLIC_COMPILE_OPTIONS_STR})
CHECK_CXX_SOURCE_COMPILES("
#include <Eigen/Core>
#if (EIGEN_DEFAULT_ALIGN_BYTES==0) || EIGEN_MALLOC_ALREADY_ALIGNED
#error Eigen will not use handmade_aligned_malloc (which is fine, we just throw an error here to make this compilation fail)
#endif
int main() { return 0; }"
PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC)
if (PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC) # CHECK_CXX_SOURCE_COMPILES does not necessarily set this to 0 or 1
  set(PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC 1)
else()
  set(PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC 0)
endif()
unset(CMAKE_REQUIRED_LIBRARIES)
unset(CMAKE_REQUIRED_FLAGS)

### --[ Set variables for pcl_config.h
if (BUILD_visualization)
  set(visualization_AVAILABLE TRUE)
endif()

### ---[ Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file("${pcl_config_h_in}" "${pcl_config_h}")
PCL_ADD_INCLUDES(common "" "${pcl_config_h}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

## ---[ PCL modules
collect_subproject_directory_names("${PCL_SOURCE_DIR}" "CMakeLists.txt" PCL_MODULES_NAMES PCL_MODULES_DIRS doc)
set(PCL_MODULES_NAMES_UNSORTED ${PCL_MODULES_NAMES})
topological_sort(PCL_MODULES_NAMES PCL_ _DEPENDS)
sort_relative(PCL_MODULES_NAMES_UNSORTED PCL_MODULES_NAMES PCL_MODULES_DIRS)
foreach(subdir ${PCL_MODULES_DIRS})
  add_subdirectory("${PCL_SOURCE_DIR}/${subdir}")
endforeach()


install(
  EXPORT PCLTargets
  NAMESPACE PCL::
  DESTINATION cmake
)  

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

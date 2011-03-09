cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

include(${PROJECT_SOURCE_DIR}/cmake/pcl_options.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/pcl_utils.cmake)
set(PROJECT_VERSION 2.0.0 CACHE STRING "PROJECT version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

# Add an "uninstall" target
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

# Find universal dependencies
# TODO: Write proper Find___.cmake modules for these
# Boost
include(${PROJECT_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
# SSE flags
include(${PROJECT_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()
# Eigen
GET_PKG_CONFIG_INFO(eigen3 REQUIRED)
APPLY_PKG_CONFIG_DIRS(eigen3)
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_USE_NEW_STDVECTOR
    -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
# FLANN
GET_PKG_CONFIG_INFO(flann REQUIRED)
APPLY_PKG_CONFIG_DIRS(flann)
# CMinpack
GET_PKG_CONFIG_INFO(cminpack REQUIRED)
APPLY_PKG_CONFIG_DIRS(cminpack)
# OpenMP
include(${PROJECT_SOURCE_DIR}/cmake/pcl_openmp.cmake)
PCL_FIND_OPENMP_FLAGS()

# Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file(${pcl_config_h_in} ${pcl_config_h})
PCL_ADD_INCLUDES(common "" ${pcl_config_h})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

# Add the library subdirectories
include(${PROJECT_SOURCE_DIR}/cmake/pcl_targets.cmake)
add_subdirectory(common)
add_subdirectory(io)
add_subdirectory(kdtree)
add_subdirectory(range_image)
add_subdirectory(features)
add_subdirectory(sample_consensus)
add_subdirectory(keypoints)
add_subdirectory(filters)
add_subdirectory(registration)
add_subdirectory(segmentation)
add_subdirectory(surface)
add_subdirectory(visualization)

# Package creation
include(InstallRequiredSystemLibraries)
configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    "${PROJECT_BINARY_DIR}/cpack_options.cmake" @ONLY)
set(CPACK_PROJECT_CONFIG_FILE "${PROJECT_BINARY_DIR}/cpack_options.cmake")
include(CPack)

# Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

set(PROJECT_DEPENDENCY_GRAPHVIZ_FILE "${PROJECT_BINARY_DIR}/pcl.dot")
set(pcl_subprojects common io kdtree range_image features sample_consensus keypoints filters registration segmentation surface visualization)
file(WRITE ${PROJECT_DEPENDENCY_GRAPHVIZ_FILE} "digraph pcl {\n")
foreach(SUBDIR ${pcl_subprojects})
  set(BUILD_SUBDIR "BUILD_${SUBDIR}")
  if (${BUILD_SUBDIR})
    file(APPEND ${PROJECT_DEPENDENCY_GRAPHVIZ_FILE} "  \"${SUBDIR}\" [style=\"filled\" fillcolor=\"#A3A27C\" shape=\"box\"];\n ")
  endif (${BUILD_SUBDIR})
  string(TOUPPER "PCL_${SUBDIR}_DEPENDS" DEPENDS_SUBDIR)
  if(DEFINED ${DEPENDS_SUBDIR})
    foreach(DEP ${${DEPENDS_SUBDIR}})
      file(APPEND ${PROJECT_DEPENDENCY_GRAPHVIZ_FILE} 
        "  \"${SUBDIR}\" -> \"${DEP}\";\n")
    endforeach()
  endif()
endforeach()
file(APPEND ${PROJECT_DEPENDENCY_GRAPHVIZ_FILE} "  \"test\" [style=\"filled\" fillcolor=\"#A3A27C\" shape=\"box\"];\n ")
file(APPEND ${PROJECT_DEPENDENCY_GRAPHVIZ_FILE} "}\n")

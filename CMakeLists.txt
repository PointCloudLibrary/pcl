cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

include(${PROJECT_SOURCE_DIR}/cmake/pcl_utils.cmake)
set(PCL_VERSION 2.0.0 CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

# Add an "uninstall" target
CONFIGURE_FILE ("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET (uninstall "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

# Add the library subdirectories
add_subdirectory(pcl)
add_subdirectory(visualization)

# Package creation
include(InstallRequiredSystemLibraries)
configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    "${PROJECT_BINARY_DIR}/cpack_options.cmake" @ONLY)
set(CPACK_PROJECT_CONFIG_FILE "${PROJECT_BINARY_DIR}/cpack_options.cmake")
include(CPack)


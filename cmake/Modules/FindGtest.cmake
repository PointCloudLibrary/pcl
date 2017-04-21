###############################################################################
# Find GTest
#
# This sets the following variables:
# GTEST_FOUND - True if GTest was found.
# GTEST_INCLUDE_DIRS - Directories containing the GTest include files.
# GTEST_SRC_DIR - Directories containing the GTest source files.

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr /usr/local)
endif(CMAKE_SYSTEM_NAME STREQUAL Linux)
if(APPLE)
  list(APPEND CMAKE_INCLUDE_PATH /opt/local)
  set(CMAKE_FIND_FRAMEWORK NEVER)
endif()

find_path(GTEST_INCLUDE_DIR gtest/gtest.h
    HINTS "${GTEST_ROOT}" "$ENV{GTEST_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/gtest" "$ENV{PROGRAMW6432}/gtest"
    PATHS "$ENV{PROGRAMFILES}/gtest-1.7.0" "$ENV{PROGRAMW6432}/gtest-1.7.0"
    PATH_SUFFIXES gtest include/gtest include)

find_path(GTEST_SRC_DIR src/gtest-all.cc
    HINTS "${GTEST_ROOT}" "$ENV{GTEST_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/gtest" "$ENV{PROGRAMW6432}/gtest"
    PATHS "$ENV{PROGRAMFILES}/gtest-1.7.0" "$ENV{PROGRAMW6432}/gtest-1.7.0"
    PATH /usr/src/gtest
    PATH_SUFFIXES gtest usr/src/gtest)

set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIR})
set(CMAKE_FIND_FRAMEWORK)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gtest DEFAULT_MSG GTEST_INCLUDE_DIR GTEST_SRC_DIR)

mark_as_advanced(GTEST_INCLUDE_DIR GTEST_SRC_DIR)

if(GTEST_FOUND)
  message(STATUS "GTest found (include: ${GTEST_INCLUDE_DIRS}, src: ${GTEST_SRC_DIR})")
endif(GTEST_FOUND)

set(SUBSYS_NAME global_tests)
set(SUBSYS_DESC "Point cloud library global unit tests")

set(DEFAULT OFF)
set(build TRUE)
set(REASON "Disabled by default")
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

find_package(GTestSource REQUIRED)
include_directories(SYSTEM ${GTEST_INCLUDE_DIRS} ${GTEST_SRC_DIR})

add_library(pcl_gtest STATIC ${GTEST_SRC_DIR}/src/gtest-all.cc)

enable_testing()

# VS needs -C config to run correct
if(MSVC)
  set(PCL_CTEST_ARGUMENTS ${PCL_CTEST_ARGUMENTS} -C $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>)
endif()

# Enables you to disable visualization tests. Used on CI.
if(PCL_DISABLE_VISUALIZATION_TESTS)
  list(APPEND EXCLUDE_TESTS visualization)
endif()

# Enables you to disable GPU tests. Used on CI as it has no access to GPU hardware
if(PCL_DISABLE_GPU_TESTS)
  list(APPEND EXCLUDE_TESTS gpu)
endif()

#Check if there are any tests to exclude
if(EXCLUDE_TESTS)
  message(STATUS "Tests excluded: ${EXCLUDE_TESTS}")
  string(REPLACE ";" "|" EXCLUDE_TESTS_REGEX "${EXCLUDE_TESTS}")
  set(PCL_CTEST_ARGUMENTS ${PCL_CTEST_ARGUMENTS} -E "(${EXCLUDE_TESTS_REGEX})")
endif()

add_custom_target(tests "${CMAKE_CTEST_COMMAND}" ${PCL_CTEST_ARGUMENTS} -V -T Test VERBATIM)

set_target_properties(tests PROPERTIES FOLDER "Tests")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory(2d)
add_subdirectory(common)
add_subdirectory(features)
add_subdirectory(filters)
add_subdirectory(geometry)
add_subdirectory(gpu)
add_subdirectory(io)
add_subdirectory(kdtree)
add_subdirectory(keypoints)
add_subdirectory(ml)
add_subdirectory(people)
add_subdirectory(octree)
add_subdirectory(outofcore)
add_subdirectory(recognition)
add_subdirectory(registration)
add_subdirectory(search)
add_subdirectory(surface)
add_subdirectory(segmentation)
add_subdirectory(sample_consensus)
add_subdirectory(visualization)

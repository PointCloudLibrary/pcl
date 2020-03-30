set(SUBSYS_NAME global_tests)
set(SUBSYS_DESC "Point cloud library global unit tests")

set(DEFAULT OFF)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

find_package(GTestSource REQUIRED)
include_directories(SYSTEM ${GTEST_INCLUDE_DIRS} ${GTEST_SRC_DIR})

add_library(pcl_gtest STATIC ${GTEST_SRC_DIR}/src/gtest-all.cc)

enable_testing()

if(MSVC)
  # VS needs -C config to run correct
  add_custom_target(tests "${CMAKE_CTEST_COMMAND}" -C $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release> -V -T Test VERBATIM)
else()
  add_custom_target(tests "${CMAKE_CTEST_COMMAND}" -V -T Test VERBATIM)
endif()  

set_target_properties(tests PROPERTIES FOLDER "Tests")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory(2d)
add_subdirectory(common)
add_subdirectory(features)
add_subdirectory(filters)
add_subdirectory(geometry)
add_subdirectory(io)
add_subdirectory(kdtree)
add_subdirectory(keypoints)
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

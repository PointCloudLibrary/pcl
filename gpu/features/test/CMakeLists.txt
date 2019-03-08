if(NOT BUILD_TESTS)
  return()
endif()

include_directories(
  "${PCL_SOURCE_DIR}/test/gtest-1.6.0/include"
  "${PCL_SOURCE_DIR}/test/gtest-1.6.0"
)

set(pcl_gtest_sources "${PCL_SOURCE_DIR}/test/gtest-1.6.0/src/gtest-all.cc")

set(the_test_target test_gpu_features)

get_filename_component(DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(INC1 "${DIR}/../../../io/include" REALPATH)
get_filename_component(INC2 "${DIR}/../../../features/include" REALPATH)
get_filename_component(INC3 "${DIR}/../../../kdtree/include" REALPATH)
get_filename_component(INC4 "${DIR}/../../../visualization/include" REALPATH)
get_filename_component(INC5 "${DIR}/../../../common/include" REALPATH)
get_filename_component(INC6 "${DIR}/../../../search/include" REALPATH)
get_filename_component(INC7 "${DIR}/../../../octree/include" REALPATH)


file(GLOB test_src *.cpp *.hpp)
list(APPEND test_src ${pcl_gtest_sources})
include_directories("${INC1}" "${INC2}" "${INC3}" "${INC4}" "${INC5}" "${INC6}" "${INC7}")
include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

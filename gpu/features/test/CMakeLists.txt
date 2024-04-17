if(NOT BUILD_TESTS)
  return()
endif()

set(the_test_target test_gpu_features)

file(GLOB test_src *.cpp *.hpp)
list(APPEND test_src ${pcl_gtest_sources})


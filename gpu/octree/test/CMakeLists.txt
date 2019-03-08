if(NOT BUILD_TESTS)
  return()
endif()

set(the_test_target test_gpu_octree)

get_filename_component(DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(INC "${DIR}/../../../octree/include" REALPATH)
include_directories("${INC}")

file(GLOB test_src *.cpp *.hpp)

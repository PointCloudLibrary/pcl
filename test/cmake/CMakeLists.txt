set(SUBSYS_NAME tests_cmake)
set(SUBSYS_DESC "Point cloud library CMake config tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS surface)
set(OPT_DEPS kdtree search)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

# Regression test for #6522: find_package(PCL COMPONENTS surface) must
# expose QHULL::QHULL and FLANN::FLANN on the component link interfaces,
# otherwise $<TARGET_RUNTIME_DLLS:app> omits qhull_r.dll / flann_cpp.dll
# and a deployed app exits 0xC0000135 (STATUS_DLL_NOT_FOUND). The consumer
# project is configured against this build tree's PCLConfig.cmake and fails
# at configure time if either target is missing from the link interface.
# Only meaningful when both optional deps are enabled in this build.
#
# Two build-tree quirks are worked around here (both pre-existing and
# unrelated to #6522):
#  - the build tree has no Modules/ dir next to PCLConfig.cmake (that only
#    exists in an install tree), so the source-tree find modules are passed
#    explicitly via CMAKE_MODULE_PATH;
#  - the build-tree PCLConfig.cmake cannot locate the component headers on
#    its own (it only looks under the build include dir), so the component
#    include dirs are pre-seeded from the source tree. find_path() honours
#    a pre-set cache variable and skips its search.
if(WITH_QHULL AND WITH_FLANN)
  # Forward the parent's Eigen3 location to the consumer configure: the
  # consumer's PCLConfig.cmake calls find_package(Eigen3 REQUIRED NO_MODULE),
  # which fails when Eigen3 lives outside the default search paths.
  set(PCL_CONSUMER_DEP_HINTS "")
  if(Eigen3_DIR)
    list(APPEND PCL_CONSUMER_DEP_HINTS "-DEigen3_DIR=${Eigen3_DIR}")
  endif()
  add_test(NAME cmake_pcl_config_surface_external_deps
           COMMAND ${CMAKE_COMMAND}
                   -S ${CMAKE_CURRENT_SOURCE_DIR}/test_pcl_config_external_deps
                   -B ${CMAKE_CURRENT_BINARY_DIR}/test_pcl_config_external_deps
                   -DPCL_DIR=${PCL_BINARY_DIR}
                   ${PCL_CONSUMER_DEP_HINTS}
                   -DCMAKE_MODULE_PATH=${PCL_SOURCE_DIR}/cmake/Modules
                   -DPCL_COMMON_INCLUDE_DIR=${PCL_SOURCE_DIR}/common/include
                   -DPCL_KDTREE_INCLUDE_DIR=${PCL_SOURCE_DIR}/kdtree/include
                   -DPCL_OCTREE_INCLUDE_DIR=${PCL_SOURCE_DIR}/octree/include
                   -DPCL_SEARCH_INCLUDE_DIR=${PCL_SOURCE_DIR}/search/include
                   -DPCL_SURFACE_INCLUDE_DIR=${PCL_SOURCE_DIR}/surface/include)
endif()

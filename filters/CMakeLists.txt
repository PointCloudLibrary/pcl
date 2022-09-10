set(SUBSYS_NAME filters)
set(SUBSYS_DESC "Point cloud filters library")
set(SUBSYS_DEPS common sample_consensus search kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(srcs
  src/conditional_removal.cpp
  src/convolution.cpp
  src/crop_box.cpp
  src/extract_indices.cpp
  src/filter.cpp
  src/filter_indices.cpp
  src/passthrough.cpp
  src/shadowpoints.cpp
  src/project_inliers.cpp
  src/pyramid.cpp
  src/radius_outlier_removal.cpp
  src/random_sample.cpp
  src/normal_space.cpp
  src/sampling_surface_normal.cpp
  src/statistical_outlier_removal.cpp
  src/voxel_grid.cpp
  src/approximate_voxel_grid.cpp
  src/bilateral.cpp
  src/fast_bilateral.cpp
  src/fast_bilateral_omp.cpp
  src/crop_hull.cpp
  src/voxel_grid_covariance.cpp
  src/voxel_grid_label.cpp
  src/frustum_culling.cpp
  src/covariance_sampling.cpp
  src/median_filter.cpp
  src/uniform_sampling.cpp
  src/voxel_grid_occlusion_estimation.cpp
  src/normal_refinement.cpp
  src/grid_minimum.cpp
  src/morphological_filter.cpp
  src/local_maximum.cpp
  src/model_outlier_removal.cpp
  src/farthest_point_sampling.cpp
)

set(incs
  "include/pcl/${SUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/conditional_removal.h"
  "include/pcl/${SUBSYS_NAME}/crop_box.h"
  "include/pcl/${SUBSYS_NAME}/clipper3D.h"
  "include/pcl/${SUBSYS_NAME}/plane_clipper3D.h"
  "include/pcl/${SUBSYS_NAME}/box_clipper3D.h"
  "include/pcl/${SUBSYS_NAME}/crop_hull.h"
  "include/pcl/${SUBSYS_NAME}/extract_indices.h"
  "include/pcl/${SUBSYS_NAME}/filter.h"
  "include/pcl/${SUBSYS_NAME}/filter_indices.h"
  "include/pcl/${SUBSYS_NAME}/passthrough.h"
  "include/pcl/${SUBSYS_NAME}/shadowpoints.h"
  "include/pcl/${SUBSYS_NAME}/project_inliers.h"
  "include/pcl/${SUBSYS_NAME}/radius_outlier_removal.h"
  "include/pcl/${SUBSYS_NAME}/random_sample.h"
  "include/pcl/${SUBSYS_NAME}/normal_space.h"
  "include/pcl/${SUBSYS_NAME}/sampling_surface_normal.h"
  "include/pcl/${SUBSYS_NAME}/statistical_outlier_removal.h"
  "include/pcl/${SUBSYS_NAME}/voxel_grid.h"
  "include/pcl/${SUBSYS_NAME}/approximate_voxel_grid.h"
  "include/pcl/${SUBSYS_NAME}/bilateral.h"
  "include/pcl/${SUBSYS_NAME}/fast_bilateral.h"
  "include/pcl/${SUBSYS_NAME}/fast_bilateral_omp.h"
  "include/pcl/${SUBSYS_NAME}/voxel_grid_covariance.h"
  "include/pcl/${SUBSYS_NAME}/convolution.h"
  "include/pcl/${SUBSYS_NAME}/convolution_3d.h"
  "include/pcl/${SUBSYS_NAME}/voxel_grid_label.h"
  "include/pcl/${SUBSYS_NAME}/voxel_grid_occlusion_estimation.h"
  "include/pcl/${SUBSYS_NAME}/frustum_culling.h"
  "include/pcl/${SUBSYS_NAME}/covariance_sampling.h"
  "include/pcl/${SUBSYS_NAME}/median_filter.h"
  "include/pcl/${SUBSYS_NAME}/uniform_sampling.h"
  "include/pcl/${SUBSYS_NAME}/normal_refinement.h"
  "include/pcl/${SUBSYS_NAME}/grid_minimum.h"
  "include/pcl/${SUBSYS_NAME}/morphological_filter.h"
  "include/pcl/${SUBSYS_NAME}/local_maximum.h"
  "include/pcl/${SUBSYS_NAME}/model_outlier_removal.h"
  "include/pcl/${SUBSYS_NAME}/farthest_point_sampling.h"
)
set(experimental_incs
  "include/pcl/${SUBSYS_NAME}/experimental/functor_filter.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/crop_box.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/crop_hull.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/plane_clipper3D.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/box_clipper3D.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/extract_indices.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/filter.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/filter_indices.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/passthrough.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/shadowpoints.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/project_inliers.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/radius_outlier_removal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/random_sample.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/normal_space.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/sampling_surface_normal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/statistical_outlier_removal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/voxel_grid.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/approximate_voxel_grid.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/bilateral.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/fast_bilateral.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/fast_bilateral_omp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/voxel_grid_covariance.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/convolution.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/convolution_3d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/voxel_grid_occlusion_estimation.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/frustum_culling.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/covariance_sampling.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/median_filter.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/uniform_sampling.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/normal_refinement.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/grid_minimum.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/morphological_filter.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/local_maximum.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/model_outlier_removal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/farthest_point_sampling.hpp"
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries("${LIB_NAME}" pcl_common pcl_sample_consensus pcl_search pcl_kdtree pcl_octree)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/experimental" ${experimental_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

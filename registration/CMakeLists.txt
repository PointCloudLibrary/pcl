set(SUBSYS_NAME registration)
set(SUBSYS_DESC "Point cloud registration library")
set(SUBSYS_DEPS common octree kdtree search sample_consensus features filters)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(incs
  "include/pcl/${SUBSYS_NAME}/eigen.h"
  "include/pcl/${SUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/boost_graph.h"
  "include/pcl/${SUBSYS_NAME}/convergence_criteria.h"
  "include/pcl/${SUBSYS_NAME}/default_convergence_criteria.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_estimation.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_estimation_normal_shooting.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_estimation_backprojection.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_estimation_organized_projection.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_distance.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_median_distance.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_surface_normal.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_features.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_one_to_one.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_poly.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_sample_consensus.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_sample_consensus_2d.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_trimmed.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_var_trimmed.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_rejection_organized_boundary.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_sorting.h"
  "include/pcl/${SUBSYS_NAME}/correspondence_types.h"
  "include/pcl/${SUBSYS_NAME}/ia_ransac.h"
  "include/pcl/${SUBSYS_NAME}/icp.h"
  "include/pcl/${SUBSYS_NAME}/joint_icp.h"
  "include/pcl/${SUBSYS_NAME}/incremental_registration.h"
  "include/pcl/${SUBSYS_NAME}/icp_nl.h"
  "include/pcl/${SUBSYS_NAME}/lum.h"
  "include/pcl/${SUBSYS_NAME}/elch.h"
  "include/pcl/${SUBSYS_NAME}/meta_registration.h"
  "include/pcl/${SUBSYS_NAME}/ndt.h"
  "include/pcl/${SUBSYS_NAME}/ndt_2d.h"
  "include/pcl/${SUBSYS_NAME}/ppf_registration.h"

  "include/pcl/${SUBSYS_NAME}/impl/pairwise_graph_registration.hpp"

  "include/pcl/${SUBSYS_NAME}/pyramid_feature_matching.h"
  "include/pcl/${SUBSYS_NAME}/registration.h"
  "include/pcl/${SUBSYS_NAME}/transforms.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_2D.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_svd.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_svd_scale.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_dual_quaternion.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_lm.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_point_to_plane.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_point_to_plane_weighted.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_point_to_plane_lls.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_point_to_plane_lls_weighted.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_symmetric_point_to_plane_lls.h"
  "include/pcl/${SUBSYS_NAME}/transformation_validation.h"
  "include/pcl/${SUBSYS_NAME}/transformation_validation_euclidean.h"
  "include/pcl/${SUBSYS_NAME}/gicp.h"
  "include/pcl/${SUBSYS_NAME}/gicp6d.h"
  "include/pcl/${SUBSYS_NAME}/bfgs.h"
  "include/pcl/${SUBSYS_NAME}/warp_point_rigid.h"
  "include/pcl/${SUBSYS_NAME}/warp_point_rigid_6d.h"
  "include/pcl/${SUBSYS_NAME}/warp_point_rigid_3d.h"
  "include/pcl/${SUBSYS_NAME}/distances.h"
  "include/pcl/${SUBSYS_NAME}/exceptions.h"
  "include/pcl/${SUBSYS_NAME}/sample_consensus_prerejective.h"
  "include/pcl/${SUBSYS_NAME}/ia_fpcs.h"
  "include/pcl/${SUBSYS_NAME}/ia_kfpcs.h"
  "include/pcl/${SUBSYS_NAME}/matching_candidate.h"
  "include/pcl/${SUBSYS_NAME}/transformation_estimation_3point.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/default_convergence_criteria.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_estimation.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_estimation_normal_shooting.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_estimation_backprojection.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_estimation_organized_projection.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_distance.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_median_distance.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_surface_normal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_features.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_one_to_one.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_poly.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_sample_consensus.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_sample_consensus_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_trimmed.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_var_trimmed.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_organized_boundary.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/correspondence_types.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ia_ransac.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/icp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/joint_icp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/incremental_registration.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/icp_nl.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/elch.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/lum.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/meta_registration.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ndt.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ndt_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ppf_registration.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/pyramid_feature_matching.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/registration.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_2D.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_svd.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_svd_scale.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_dual_quaternion.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_lm.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_point_to_plane_lls.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_point_to_plane_lls_weighted.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_point_to_plane_weighted.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_symmetric_point_to_plane_lls.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_validation_euclidean.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/gicp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/sample_consensus_prerejective.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ia_fpcs.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ia_kfpcs.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_3point.hpp"
)

set(srcs
  src/registration.cpp
  src/correspondence_estimation.cpp
  src/correspondence_estimation_normal_shooting.cpp
  src/correspondence_estimation_backprojection.cpp
  src/correspondence_estimation_organized_projection.cpp
  src/correspondence_rejection_distance.cpp
  src/correspondence_rejection_median_distance.cpp
  src/correspondence_rejection_surface_normal.cpp
  src/correspondence_rejection_features.cpp
  src/correspondence_rejection_one_to_one.cpp
  src/correspondence_rejection_poly.cpp
  src/correspondence_types.cpp
  src/correspondence_rejection_sample_consensus.cpp
  src/correspondence_rejection_sample_consensus_2d.cpp
  src/correspondence_rejection_trimmed.cpp
  src/correspondence_rejection_var_trimmed.cpp
  src/correspondence_rejection_organized_boundary.cpp
  src/ppf_registration.cpp
  src/pyramid_feature_matching.cpp
  #src/pairwise_graph_registration.cpp
  src/ia_ransac.cpp
  src/icp.cpp
  src/joint_icp.cpp
  src/gicp.cpp
  src/gicp6d.cpp
  src/icp_nl.cpp
  src/elch.cpp
  src/lum.cpp
  src/ndt.cpp
  src/ndt_2d.cpp
  src/transformation_estimation_2D.cpp
  src/transformation_estimation_svd.cpp
  src/transformation_estimation_svd_scale.cpp
  src/transformation_estimation_dual_quaternion.cpp
  src/transformation_estimation_lm.cpp
  src/transformation_estimation_point_to_plane_weighted.cpp
  src/transformation_estimation_point_to_plane_lls.cpp
  src/transformation_estimation_point_to_plane_lls_weighted.cpp
  src/transformation_validation_euclidean.cpp
  src/sample_consensus_prerejective.cpp
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries("${LIB_NAME}" pcl_kdtree pcl_search pcl_sample_consensus pcl_features pcl_filters)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})
# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

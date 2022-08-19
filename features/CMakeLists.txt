set(SUBSYS_NAME features)
set(SUBSYS_DESC "Point cloud features library")
set(SUBSYS_DEPS common search kdtree octree filters 2d)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

set(incs
  "include/pcl/${SUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/eigen.h"
  "include/pcl/${SUBSYS_NAME}/board.h"
  "include/pcl/${SUBSYS_NAME}/flare.h"
  "include/pcl/${SUBSYS_NAME}/brisk_2d.h"
  "include/pcl/${SUBSYS_NAME}/cppf.h"
  "include/pcl/${SUBSYS_NAME}/cvfh.h"
  "include/pcl/${SUBSYS_NAME}/our_cvfh.h"
  "include/pcl/${SUBSYS_NAME}/crh.h"
  "include/pcl/${SUBSYS_NAME}/don.h"
  "include/pcl/${SUBSYS_NAME}/feature.h"
  "include/pcl/${SUBSYS_NAME}/fpfh.h"
  "include/pcl/${SUBSYS_NAME}/fpfh_omp.h"
  "include/pcl/${SUBSYS_NAME}/from_meshes.h"
  "include/pcl/${SUBSYS_NAME}/gasd.h"
  "include/pcl/${SUBSYS_NAME}/gfpfh.h"
  "include/pcl/${SUBSYS_NAME}/integral_image2D.h"
  "include/pcl/${SUBSYS_NAME}/integral_image_normal.h"
  "include/pcl/${SUBSYS_NAME}/intensity_gradient.h"
  "include/pcl/${SUBSYS_NAME}/intensity_spin.h"
  "include/pcl/${SUBSYS_NAME}/linear_least_squares_normal.h"
  "include/pcl/${SUBSYS_NAME}/moment_invariants.h"
  "include/pcl/${SUBSYS_NAME}/moment_of_inertia_estimation.h"
  "include/pcl/${SUBSYS_NAME}/multiscale_feature_persistence.h"
  "include/pcl/${SUBSYS_NAME}/narf.h"
  "include/pcl/${SUBSYS_NAME}/narf_descriptor.h"
  "include/pcl/${SUBSYS_NAME}/normal_3d.h"
  "include/pcl/${SUBSYS_NAME}/normal_3d_omp.h"
  "include/pcl/${SUBSYS_NAME}/normal_based_signature.h"
  "include/pcl/${SUBSYS_NAME}/organized_edge_detection.h"
  "include/pcl/${SUBSYS_NAME}/pfh.h"
  "include/pcl/${SUBSYS_NAME}/pfh_tools.h"
  "include/pcl/${SUBSYS_NAME}/pfhrgb.h"
  "include/pcl/${SUBSYS_NAME}/ppf.h"
  "include/pcl/${SUBSYS_NAME}/ppfrgb.h"
  "include/pcl/${SUBSYS_NAME}/shot.h"
  "include/pcl/${SUBSYS_NAME}/shot_lrf.h"
  "include/pcl/${SUBSYS_NAME}/shot_lrf_omp.h"
  "include/pcl/${SUBSYS_NAME}/shot_omp.h"
  "include/pcl/${SUBSYS_NAME}/spin_image.h"
  "include/pcl/${SUBSYS_NAME}/principal_curvatures.h"
  "include/pcl/${SUBSYS_NAME}/rift.h"
  "include/pcl/${SUBSYS_NAME}/rops_estimation.h"
  "include/pcl/${SUBSYS_NAME}/rsd.h"
  "include/pcl/${SUBSYS_NAME}/grsd.h"
  "include/pcl/${SUBSYS_NAME}/statistical_multiscale_interest_region_extraction.h"
  "include/pcl/${SUBSYS_NAME}/vfh.h"
  "include/pcl/${SUBSYS_NAME}/esf.h"
  "include/pcl/${SUBSYS_NAME}/3dsc.h"
  "include/pcl/${SUBSYS_NAME}/usc.h"
  "include/pcl/${SUBSYS_NAME}/boundary.h"
  "include/pcl/${SUBSYS_NAME}/range_image_border_extractor.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/board.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/flare.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/brisk_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/cppf.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/cvfh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/our_cvfh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/crh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/don.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/feature.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/fpfh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/fpfh_omp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/gasd.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/gfpfh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/integral_image2D.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/integral_image_normal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/intensity_gradient.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/intensity_spin.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/linear_least_squares_normal.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/moment_invariants.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/moment_of_inertia_estimation.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/multiscale_feature_persistence.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/narf.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/normal_3d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/normal_3d_omp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/normal_based_signature.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/organized_edge_detection.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/pfh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/pfhrgb.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ppf.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ppfrgb.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/shot.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/shot_lrf.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/shot_lrf_omp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/shot_omp.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/spin_image.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/principal_curvatures.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/rift.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/rops_estimation.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/rsd.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/grsd.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/statistical_multiscale_interest_region_extraction.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/vfh.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/esf.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/3dsc.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/usc.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/boundary.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/range_image_border_extractor.hpp"
)

set(srcs
  src/board.cpp
  src/flare.cpp
  src/brisk_2d.cpp
  src/boundary.cpp
  src/cppf.cpp
  src/cvfh.cpp
  src/our_cvfh.cpp
  src/crh.cpp
  src/don.cpp
  src/fpfh.cpp
  src/from_meshes.cpp
  src/gasd.cpp
  src/gfpfh.cpp
  src/integral_image_normal.cpp
  src/intensity_gradient.cpp
  src/intensity_spin.cpp
  src/linear_least_squares_normal.cpp
  src/moment_invariants.cpp
  src/moment_of_inertia_estimation.cpp
  src/multiscale_feature_persistence.cpp
  src/narf.cpp
  src/normal_3d.cpp
  src/normal_based_signature.cpp
  src/organized_edge_detection.cpp
  src/pfh.cpp
  src/ppf.cpp
  src/shot.cpp
  src/shot_lrf.cpp
  src/spin_image.cpp
  src/principal_curvatures.cpp
  src/rift.cpp
  src/rops_estimation.cpp
  src/rsd.cpp
  src/grsd.cpp
  src/statistical_multiscale_interest_region_extraction.cpp
  src/vfh.cpp
  src/esf.cpp
  src/3dsc.cpp
  src/usc.cpp
  src/range_image_border_extractor.cpp
)

if(MSVC)
  # Workaround to avoid hitting the MSVC 4GB linker memory limit when building pcl_features.
  # Disable whole program optimization (/GL) and link-time code generation (/LTCG).
  string(REPLACE "/GL" "" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
  string(REPLACE "/LTCG" "" CMAKE_SHARED_LINKER_FLAGS_RELEASE ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
endif()

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries("${LIB_NAME}" pcl_common pcl_search pcl_kdtree pcl_octree pcl_filters)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})
# Install headers
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

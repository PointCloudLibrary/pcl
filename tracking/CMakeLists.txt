set(SUBSYS_NAME tracking)
set(SUBSYS_DESC "Point cloud tracking library")
set(SUBSYS_DEPS common search kdtree filters octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs 
        src/tracking.cpp
        src/particle_filter.cpp
        src/kld_adaptive_particle_filter.cpp
        src/coherence.cpp
        )

    set(incs 
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/tracking.h"
        "include/pcl/${SUBSYS_NAME}/tracker.h"
        "include/pcl/${SUBSYS_NAME}/coherence.h"
        "include/pcl/${SUBSYS_NAME}/nearest_pair_point_cloud_coherence.h"
        "include/pcl/${SUBSYS_NAME}/approx_nearest_pair_point_cloud_coherence.h"
        "include/pcl/${SUBSYS_NAME}/distance_coherence.h"
        "include/pcl/${SUBSYS_NAME}/hsv_color_coherence.h"
        "include/pcl/${SUBSYS_NAME}/normal_coherence.h"
        "include/pcl/${SUBSYS_NAME}/particle_filter.h"
        "include/pcl/${SUBSYS_NAME}/particle_filter_omp.h"
        "include/pcl/${SUBSYS_NAME}/kld_adaptive_particle_filter.h"
        "include/pcl/${SUBSYS_NAME}/kld_adaptive_particle_filter_omp.h"
        "include/pcl/${SUBSYS_NAME}/pyramidal_klt.h"
        )

    set(impl_incs 
        "include/pcl/${SUBSYS_NAME}/impl/tracking.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/tracker.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/coherence.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/nearest_pair_point_cloud_coherence.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/approx_nearest_pair_point_cloud_coherence.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/distance_coherence.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/hsv_color_coherence.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/normal_coherence.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/particle_filter.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/particle_filter_omp.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/kld_adaptive_particle_filter.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/kld_adaptive_particle_filter_omp.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/pyramidal_klt.hpp"
        )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    target_link_libraries("${LIB_NAME}" pcl_common pcl_kdtree pcl_search pcl_filters pcl_octree)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
endif(build)

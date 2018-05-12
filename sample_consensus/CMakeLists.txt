set(SUBSYS_NAME sample_consensus)
set(SUBSYS_DESC "Point cloud sample consensus library")
set(SUBSYS_DEPS common search)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
   set(srcs 
        src/sac.cpp
        src/sac_model_circle.cpp
        src/sac_model_circle3d.cpp
        src/sac_model_cylinder.cpp
        src/sac_model_cone.cpp
        src/sac_model_line.cpp
        src/sac_model_parallel_line.cpp
        src/sac_model_stick.cpp
        src/sac_model_normal_parallel_plane.cpp
        src/sac_model_normal_plane.cpp
        src/sac_model_normal_sphere.cpp
        src/sac_model_plane.cpp
        src/sac_model_registration.cpp
        src/sac_model_sphere.cpp
        )

    set(incs 
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/eigen.h"
        "include/pcl/${SUBSYS_NAME}/lmeds.h"
        "include/pcl/${SUBSYS_NAME}/method_types.h"
        "include/pcl/${SUBSYS_NAME}/mlesac.h"
        "include/pcl/${SUBSYS_NAME}/model_types.h"
        "include/pcl/${SUBSYS_NAME}/msac.h"
        "include/pcl/${SUBSYS_NAME}/ransac.h"
        "include/pcl/${SUBSYS_NAME}/rmsac.h"
        "include/pcl/${SUBSYS_NAME}/rransac.h"
        "include/pcl/${SUBSYS_NAME}/prosac.h"
        "include/pcl/${SUBSYS_NAME}/sac.h"
        "include/pcl/${SUBSYS_NAME}/sac_model.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_circle.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_circle3d.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_cylinder.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_cone.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_line.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_stick.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_normal_parallel_plane.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_normal_plane.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_normal_sphere.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_parallel_line.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_parallel_plane.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_perpendicular_plane.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_plane.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_registration.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_registration_2d.h"
        "include/pcl/${SUBSYS_NAME}/sac_model_sphere.h"
        )

    set(impl_incs 
        "include/pcl/${SUBSYS_NAME}/impl/lmeds.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/mlesac.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/msac.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/ransac.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/rmsac.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/rransac.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/prosac.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_circle.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_circle3d.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_cylinder.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_cone.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_line.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_stick.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_normal_parallel_plane.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_normal_plane.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_normal_sphere.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_parallel_line.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_parallel_plane.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_perpendicular_plane.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_plane.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_registration.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_registration_2d.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/sac_model_sphere.hpp"
        )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    target_link_libraries("${LIB_NAME}" pcl_common)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
endif(build)


set(SUBSYS_NAME recognition)
set(SUBSYS_DESC "Point cloud recognition library")
set(SUBSYS_DEPS common io search kdtree octree features filters registration sample_consensus ml)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(LINEMOD_INCLUDES
        "include/pcl/${SUBSYS_NAME}/linemod/line_rgbd.h"
        )
    set(LINEMOD_IMPLS
        "include/pcl/${SUBSYS_NAME}/impl/linemod/line_rgbd.hpp"
        )

    set(incs 
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/color_gradient_dot_modality.h"
        "include/pcl/${SUBSYS_NAME}/color_gradient_modality.h"
        "include/pcl/${SUBSYS_NAME}/color_modality.h"
        "include/pcl/${SUBSYS_NAME}/crh_alignment.h"
        "include/pcl/${SUBSYS_NAME}/linemod.h"
        "include/pcl/${SUBSYS_NAME}/dotmod.h"
        "include/pcl/${SUBSYS_NAME}/quantizable_modality.h"
        "include/pcl/${SUBSYS_NAME}/quantized_map.h"
        "include/pcl/${SUBSYS_NAME}/dot_modality.h"
        "include/pcl/${SUBSYS_NAME}/region_xy.h"
        "include/pcl/${SUBSYS_NAME}/mask_map.h"
        "include/pcl/${SUBSYS_NAME}/point_types.h"
        "include/pcl/${SUBSYS_NAME}/distance_map.h"
        "include/pcl/${SUBSYS_NAME}/dense_quantized_multi_mod_template.h"
        "include/pcl/${SUBSYS_NAME}/sparse_quantized_multi_mod_template.h"
        "include/pcl/${SUBSYS_NAME}/surface_normal_modality.h"
        "include/pcl/${SUBSYS_NAME}/linemod/line_rgbd.h"
        "include/pcl/${SUBSYS_NAME}/implicit_shape_model.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/auxiliary.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/hypothesis.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/model_library.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/rigid_transform_space.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/obj_rec_ransac.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/orr_graph.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/orr_octree_zprojection.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/trimmed_icp.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/orr_octree.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/simple_octree.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/voxel_structure.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/bvh.h"
        )

    set(ransac_based_incs 
        "include/pcl/${SUBSYS_NAME}/ransac_based/auxiliary.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/hypothesis.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/model_library.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/rigid_transform_space.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/obj_rec_ransac.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/orr_graph.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/orr_octree_zprojection.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/trimmed_icp.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/orr_octree.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/simple_octree.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/voxel_structure.h"
        "include/pcl/${SUBSYS_NAME}/ransac_based/bvh.h"
	)

    set(hv_incs
        "include/pcl/${SUBSYS_NAME}/hv/occlusion_reasoning.h"
        "include/pcl/${SUBSYS_NAME}/hv/hypotheses_verification.h"
        "include/pcl/${SUBSYS_NAME}/hv/hv_papazov.h"
        "include/pcl/${SUBSYS_NAME}/hv/hv_go.h"
        "include/pcl/${SUBSYS_NAME}/hv/greedy_verification.h"
        )

    set(face_detection_incs
        "include/pcl/${SUBSYS_NAME}/face_detection/face_common.h"
        "include/pcl/${SUBSYS_NAME}/face_detection/face_detector_data_provider.h"
        "include/pcl/${SUBSYS_NAME}/face_detection/rf_face_detector_trainer.h"
        "include/pcl/${SUBSYS_NAME}/face_detection/rf_face_utils.h"
        ) 

    set(cg_incs
        "include/pcl/${SUBSYS_NAME}/cg/correspondence_grouping.h"
        "include/pcl/${SUBSYS_NAME}/cg/hough_3d.h"
        "include/pcl/${SUBSYS_NAME}/cg/geometric_consistency.h"
        )

    set(impl_incs 
        "include/pcl/${SUBSYS_NAME}/impl/linemod/line_rgbd.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/ransac_based/simple_octree.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/ransac_based/voxel_structure.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/implicit_shape_model.hpp"
        )

    set(ransac_based_impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/ransac_based/simple_octree.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/ransac_based/voxel_structure.hpp"
	)

    set(hv_impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/hv/occlusion_reasoning.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/hv/hv_papazov.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/hv/greedy_verification.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/hv/hv_go.hpp"
        )

    set(cg_impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/cg/correspondence_grouping.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/cg/hough_3d.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/cg/geometric_consistency.hpp"
        )

    set(srcs
        src/linemod.cpp
        src/quantizable_modality.cpp
        src/dotmod.cpp
        src/mask_map.cpp
        src/hv/occlusion_reasoning.cpp
        src/hv/hv_papazov.cpp
        src/hv/hv_go.cpp
        src/hv/greedy_verification.cpp
        src/cg/hough_3d.cpp
        src/cg/geometric_consistency.cpp
        src/ransac_based/obj_rec_ransac.cpp
        src/ransac_based/model_library.cpp
        src/ransac_based/orr_octree.cpp
        src/ransac_based/orr_octree_zprojection.cpp
        src/face_detection/face_detector_data_provider.cpp
        src/face_detection/rf_face_detector_trainer.cpp
        src/implicit_shape_model.cpp
        )
        
    if (HAVE_METSLIB)
        set(metslib_incs "")
    else(HAVE_METSLIB)
        set(metslib_incs
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/abstract-search.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/local-search.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/mets.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/metslib_config.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/model.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/observer.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/simulated-annealing.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/tabu-search.hh"
            "include/pcl/${SUBSYS_NAME}/3rdparty/metslib/termination-criteria.hh"
            )
    endif(HAVE_METSLIB)

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs} ${face_detection_incs} ${ransac_based_incs} ${ransac_based_impl_incs} ${hv_incs} ${hv_impl_incs} ${cg_incs} ${cg_impl_incs} ${metslib_incs})
    target_link_libraries("${LIB_NAME}" pcl_common pcl_kdtree pcl_octree pcl_search pcl_features pcl_registration pcl_sample_consensus pcl_filters pcl_ml pcl_io)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/ransac_based" ${ransac_based_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/hv" ${hv_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/cg" ${cg_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/face_detection" ${face_detection_incs})
    
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/ransac_based" ${ransac_based_impl_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/hv" ${hv_impl_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/cg" ${cg_impl_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/linemod" ${LINEMOD_INCLUDES})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/linemod" ${LINEMOD_IMPLS})

endif(build)

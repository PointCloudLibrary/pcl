set(SUBSYS_NAME gpu_segmentation)
set(SUBSYS_PATH gpu/segmentation)
set(SUBSYS_DESC "Point cloud GPU segmentation library")
set(SUBSYS_DEPS common gpu_containers gpu_utils gpu_octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
mark_as_advanced("BUILD_${SUBSYS_NAME}")

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs src/extract_clusters.cpp
        )

    set(incs  include/pcl/gpu/segmentation/gpu_extract_clusters.h
              include/pcl/gpu/segmentation/gpu_extract_labeled_clusters.h
              include/pcl/gpu/segmentation/gpu_seeded_hue_segmentation.h
        )

    set(impl_incs include/pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp
                  include/pcl/gpu/segmentation/impl/gpu_extract_labeled_clusters.hpp
                  include/pcl/gpu/segmentation/impl/gpu_seeded_hue_segmentation.hpp
        )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    target_link_libraries("${LIB_NAME}" pcl_gpu_octree pcl_gpu_utils pcl_gpu_containers)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/impl" ${impl_incs})
endif(build)



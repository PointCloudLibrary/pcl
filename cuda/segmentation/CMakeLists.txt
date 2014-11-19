set(SUBSYS_NAME cuda_segmentation)
set(SUBSYS_PATH cuda/segmentation)
set(SUBSYS_DESC "Point cloud CUDA Segmentation library")
set(SUBSYS_DEPS cuda_common io common)

# ---[ Point Cloud Library - pcl/cuda/io

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")


if (build)
    set(incs
        include/pcl/cuda/segmentation/connected_components.h
        )

    set(srcs
        src/connected_components.cu
        )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_CUDA_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs})

    set(EXT_DEPS "")
    #set(EXT_DEPS CUDA)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})

endif()

set(SUBSYS_NAME cuda_common)
set(SUBSYS_PATH cuda/common)
set(SUBSYS_DESC "Point cloud CUDA common library")
set(SUBSYS_DEPS)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})

## ---[ Point Cloud Library - pcl/cuda/common
set(incs
    include/pcl/cuda/cutil.h
    include/pcl/cuda/cutil_math.h
    include/pcl/cuda/point_cloud.h
    include/pcl/cuda/point_types.h
    include/pcl/cuda/thrust.h
    include/pcl/cuda/time_gpu.h
    include/pcl/cuda/time_cpu.h
    include/pcl/cuda/cutil_inline.h
    include/pcl/cuda/cutil_inline_bankchecker.h
    include/pcl/cuda/cutil_inline_drvapi.h
    include/pcl/cuda/cutil_inline_runtime.h
    )
set(common_incs
    include/pcl/cuda/common/eigen.h
    include/pcl/cuda/common/point_type_rgb.h
    )

include_directories(./include)
set(LIB_NAME "pcl_${SUBSYS_NAME}")
set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC "${SUBSYS_DESC}"
    PCL_DEPS "${SUBSYS_DEPS}" EXT_DEPS "" HEADER_ONLY)

# Install include files
PCL_ADD_INCLUDES(${SUBSYS_NAME} "cuda" ${incs})
PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${common_incs})

set(SUBSYS_NAME cuda_filters)
set(SUBSYS_PATH cuda/filters)
set(SUBSYS_DESC "Point cloud CUDA filter library")
set(SUBSYS_DEPS cuda_common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})

## ---[ Point Cloud Library - pcl/cuda/filters
set(incs
    include/pcl/cuda/filters/filter.h
    include/pcl/cuda/filters/passthrough.h
    include/pcl/cuda/filters/voxel_grid.h
    )

include_directories(./include)

set(EXT_DEPS CUDA)

# Install include files
PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs})

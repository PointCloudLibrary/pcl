set(SUBSYS_NAME gpu_containers)
set(SUBSYS_PATH cuda/containers)
set(SUBSYS_DESC "Containers with reference counting for GPU memory. This module can be compiled without Cuda Toolkit.")
set(SUBSYS_DEPS common gpu_utils)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if (build)
    FILE(GLOB srcs src/*.cpp src/*.hpp)
    FILE(GLOB incs include/pcl/gpu/containers/*.hpp)
    FILE(GLOB incs_impl include/pcl/gpu/containers/impl/*.hpp)

    source_group("Header Files\\impl" FILES ${incs_impl})
    LIST(APPEND incs ${incs_impl})

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_CUDA_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_gpu_utils)

    set(EXT_DEPS "")
    #set(EXT_DEPS CUDA)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs})
endif()
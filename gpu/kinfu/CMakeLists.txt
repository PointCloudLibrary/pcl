set(SUBSYS_NAME gpu_kinfu)
set(SUBSYS_PATH gpu/kinfu)
set(SUBSYS_DESC "Kinect Fusion implementation")
set(SUBSYS_DEPS common io gpu_containers geometry search visualization)

set(build FALSE)

# OpenNI found?
if(NOT WITH_OPENNI)
    set(DEFAULT FALSE)
    set(REASON "OpenNI was not found or was disabled by the user.")
else()
    set(DEFAULT TRUE)
    set(REASON)
endif()

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

REMOVE_VTK_DEFINITIONS()
FILE(GLOB incs include/pcl/gpu/kinfu/*.h*)
file(GLOB srcs src/*.cpp src/*.h*)
file(GLOB cuda src/cuda/*.cu src/cuda/*.h*)

source_group("Source Files\\cuda" FILES ${cuda})
source_group("Source Files" FILES ${srcs})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})

if(UNIX OR APPLE)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
endif()

if(NOT UNIX OR APPLE)
  add_definitions(-DPCLAPI_EXPORTS)
endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")
CUDA_COMPILE(cuda_objs ${cuda})

PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${cuda} ${cuda_objs})
target_link_libraries("${LIB_NAME}" pcl_gpu_containers)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})

if(BUILD_visualization)
  add_subdirectory(test)
  add_subdirectory(tools)
endif()

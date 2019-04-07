set(SUBSYS_NAME simulation)
set(SUBSYS_DESC "Point Cloud Library Simulation")
set(SUBSYS_DEPS common io surface kdtree features search octree visualization filters geometry)

set(build FALSE)
find_package(OpenGL)
find_package(GLEW)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" OFF)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS opengl glew)

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(srcs
   src/camera.cpp
   src/model.cpp
   src/range_likelihood.cpp
   src/scene.cpp
   src/glsl_shader.cpp
   src/sum_reduce.cpp
)

set(incs
  "include/pcl/${SUBSYS_NAME}/camera.h"
  "include/pcl/${SUBSYS_NAME}/model.h"
  "include/pcl/${SUBSYS_NAME}/range_likelihood.h"
  "include/pcl/${SUBSYS_NAME}/scene.h"
  "include/pcl/${SUBSYS_NAME}/glsl_shader.h"
  "include/pcl/${SUBSYS_NAME}/sum_reduce.h"
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")

PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}")
target_sources("${LIB_NAME}" PRIVATE ${srcs} ${incs})


set(EXT_DEPS eigen3)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})

add_subdirectory(tools)

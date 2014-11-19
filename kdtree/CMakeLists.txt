set(SUBSYS_NAME kdtree)
set(SUBSYS_DESC "Point cloud kd-tree library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS flann)

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs 
        src/kdtree_flann.cpp
        )

    set(incs 
        "include/pcl/${SUBSYS_NAME}/kdtree.h"
        "include/pcl/${SUBSYS_NAME}/io.h"
        "include/pcl/${SUBSYS_NAME}/flann.h"
        "include/pcl/${SUBSYS_NAME}/kdtree_flann.h"
        )

    set(impl_incs 
        "include/pcl/${SUBSYS_NAME}/impl/io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/kdtree_flann.hpp"
        )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    target_link_libraries("${LIB_NAME}" pcl_common ${FLANN_LIBRARIES})
    set(EXT_DEPS flann)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
                       "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

endif(build)


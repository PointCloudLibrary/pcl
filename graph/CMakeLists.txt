set(SUBSYS_NAME graph)
set(SUBSYS_DESC "Point cloud graph library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

# Boost >= 1.53 is required
if(build AND (Boost_MAJOR_VERSION GREATER 1 OR Boost_MINOR_VERSION GREATER 52))

    set(incs
        "include/pcl/${SUBSYS_NAME}/point_cloud_graph.h"
        "include/pcl/${SUBSYS_NAME}/point_cloud_graph_concept.h"
        )

    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

endif()

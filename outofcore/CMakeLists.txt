set(SUBSYS_NAME outofcore)
set(SUBSYS_DESC "Point cloud outofcore library")
set(SUBSYS_DEPS common io filters octree visualization)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs
        src/cJSON.cpp
        src/outofcore_node_data.cpp
        src/outofcore_base_data.cpp
        )

    set(incs
        "include/pcl/${SUBSYS_NAME}/metadata.h"
        "include/pcl/${SUBSYS_NAME}/outofcore_base_data.h"
        "include/pcl/${SUBSYS_NAME}/outofcore_node_data.h"
        "include/pcl/${SUBSYS_NAME}/outofcore_iterator_base.h"
        "include/pcl/${SUBSYS_NAME}/outofcore_breadth_first_iterator.h"
        "include/pcl/${SUBSYS_NAME}/outofcore_depth_first_iterator.h"
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/cJSON.h"
        "include/pcl/${SUBSYS_NAME}/octree_base.h"
        "include/pcl/${SUBSYS_NAME}/octree_base_node.h"
        "include/pcl/${SUBSYS_NAME}/octree_abstract_node_container.h"
        "include/pcl/${SUBSYS_NAME}/octree_disk_container.h"
        "include/pcl/${SUBSYS_NAME}/octree_ram_container.h"
        "include/pcl/${SUBSYS_NAME}/outofcore.h"
        "include/pcl/${SUBSYS_NAME}/outofcore_impl.h"
        )

    set(impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/outofcore_breadth_first_iterator.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/outofcore_depth_first_iterator.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/octree_base.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/octree_base_node.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/octree_disk_container.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/octree_ram_container.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/monitor_queue.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/lru_cache.hpp"
        )
    set(visualization_incs
        "include/pcl/${SUBSYS_NAME}/visualization/axes.h"
        "include/pcl/${SUBSYS_NAME}/visualization/camera.h"
        "include/pcl/${SUBSYS_NAME}/visualization/common.h"
        "include/pcl/${SUBSYS_NAME}/visualization/geometry.h"
        "include/pcl/${SUBSYS_NAME}/visualization/grid.h"
        "include/pcl/${SUBSYS_NAME}/visualization/object.h"
        "include/pcl/${SUBSYS_NAME}/visualization/outofcore_cloud.h"
        "include/pcl/${SUBSYS_NAME}/visualization/scene.h"
        "include/pcl/${SUBSYS_NAME}/visualization/viewport.h"
        )
    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs} ${visualization_incs})
    #PCL_ADD_SSE_FLAGS("${LIB_NAME}")
    target_link_libraries("${LIB_NAME}" pcl_common pcl_visualization ${Boost_SYSTEM_LIBRARY})
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/visualization" ${visualization_incs})

    if(BUILD_tools)
        add_subdirectory(tools)
    endif(BUILD_tools)
    
endif(build)

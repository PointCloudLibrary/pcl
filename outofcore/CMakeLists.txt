set(SUBSYS_NAME outofcore)
set(SUBSYS_DESC "Point cloud outofcore library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(srcs
        src/cJSON.cpp
        )

    set(incs
        include/pcl/${SUBSYS_NAME}/octree_base.h
        include/pcl/${SUBSYS_NAME}/octree_base_node.h
        include/pcl/${SUBSYS_NAME}/octree_exceptions.h
        include/pcl/${SUBSYS_NAME}/pointCloudTools.h
        include/pcl/${SUBSYS_NAME}/cJSON.h
        )

    set(impl_incs
        include/pcl/${SUBSYS_NAME}/impl/octree_base.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree_base_node.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree_disk_container.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree_ram_container.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    #PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    if(BUILD_TESTS)
        add_subdirectory(test)
    endif(BUILD_TESTS)
    
endif(build)

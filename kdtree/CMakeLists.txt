set(SUBSYS_NAME kdtree)
set(SUBSYS_DESC "Point cloud kd-tree library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/kdtree_flann.cpp
        src/organized_data.cpp
        src/tree_types.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} ${FLANN_LIBRARIES} ${Boost_LIBRARIES})
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    list(APPEND EXT_DEPS flann)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/kdtree.h
        include/pcl/${SUBSYS_NAME}/kdtree_flann.h
        include/pcl/${SUBSYS_NAME}/organized_data.h
        include/pcl/${SUBSYS_NAME}/tree_types.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/kdtree_flann.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_data.hpp
        include/pcl/${SUBSYS_NAME}/impl/tree_types.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(test)
endif(build)


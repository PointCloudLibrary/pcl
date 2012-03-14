set(SUBSYS_NAME people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common search kdtree sample_consensus)

set(build FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(srcs src/conversions/conversions.cpp
        )

    set(incs include/pcl/${SUBSYS_NAME}/conversions/conversions.h
             include/pcl/${SUBSYS_NAME}/colormap.h
        )

    #set(impl_incs include/pcl/${SUBSYS_NAME}/conversions/conversions.hpp
    #    )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
    target_link_libraries(${LIB_NAME} pcl_search pcl_sample_consensus)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    #PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)

#########################################################
#  * Copyright (c) 2013, Intelligent Robotics Lab, DLUT
#  * Author: Qinghua Li, Yan Zhuang
#  * Created on: July 07, 2013
#########################################################

set(SUBSYS_NAME place_recognition)
set(SUBSYS_DESC "Point cloud place recognition library")
set(SUBSYS_DEPS common io features registration kdtree search)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS opencv)

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(incs
        include/pcl/${SUBSYS_NAME}/bearing_angle.h
        include/pcl/${SUBSYS_NAME}/scene_cognition.h
        )

    set(srcs
        src/bearing_angle.cpp
        src/scene_cognition.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
    target_link_libraries(${LIB_NAME} pcl_common pcl_io opencv_core opencv_imgproc)
    set(EXT_DEPS opencv)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

endif(build)

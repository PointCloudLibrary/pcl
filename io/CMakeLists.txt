set(SUBSYS_NAME io)
set(SUBSYS_DESC "Point cloud IO library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    GET_PKG_CONFIG_INFO(openni-dev "")
    if(openni-dev_FOUND)
        set(KINECT_GRABBER_INCLUDES include/pcl/io/openni_depth_image.h
            include/pcl/io/openni_device_kinect.h
            include/pcl/io/openni_driver.h
            include/pcl/io/openni_image_bayer_grbg.h
            include/pcl/io/openni_image_yuv_422.h
            include/pcl/io/openni_device.h
            include/pcl/io/openni_device_primesense.h
            include/pcl/io/openni_exception.h
            include/pcl/io/openni_image.h
            include/pcl/io/kinect_grabber.h)
        set(KINECT_GRABBER_SOURCES src/openni_camera/openni_device.cpp
            src/openni_camera/openni_device_primesense.cpp
            src/openni_camera/openni_image_bayer_grbg.cpp
            src/openni_camera/openni_depth_image.cpp
            src/openni_camera/openni_device_kinect.cpp
            src/openni_camera/openni_driver.cpp
            src/openni_camera/openni_exception.cpp
            src/openni_camera/openni_image_yuv_422.cpp
            src/kinect_grabber.cpp)
        APPLY_PKG_CONFIG_DIRS(openni-dev)
    endif(openni-dev_FOUND)

    set(srcs src/io.cpp
        src/pcd_io.cpp
        src/vtk_io.cpp
        ${KINECT_GRABBER_SOURCES}
        )
    APPLY_PKG_CONFIG_TO_SRCS(openni-dev ${srcs})

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} ${Boost_LIBRARIES})
    APPLY_PKG_CONFIG_TO_TGTS(${LIB_NAME} openni-dev)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    list(APPEND EXT_DEPS ${Boost_LIBRARIES})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/io.h
        include/pcl/${SUBSYS_NAME}/pcd_io.h
        include/pcl/${SUBSYS_NAME}/vtk_io.h
        ${KINECT_GRABBER_INCLUDES}
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/io.hpp)
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)


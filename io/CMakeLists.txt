set(SUBSYS_NAME io)
set(SUBSYS_DESC "Point cloud IO library")
set(SUBSYS_DEPS common octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
if(WIN32)
    PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS openni openni2 pcap png vtk)
else(WIN32)
    PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS openni openni2 pcap png vtk libusb-1.0)
endif(WIN32)

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
  set(IMAGE_INCLUDES
    include/pcl/io/image_metadata_wrapper.h
    include/pcl/io/image.h
    include/pcl/io/image_rgb24.h
    include/pcl/io/image_yuv422.h
    include/pcl/io/image_ir.h
    include/pcl/io/image_depth.h
    include/pcl/io/io_exception.h
  )

  set(IMAGE_SOURCES
    src/image_rgb24.cpp
    src/image_yuv422.cpp
    src/image_ir.cpp
    src/image_depth.cpp
    src/io_exception.cpp
  )

    ## OpenNI 2.x
    OPTION(BUILD_OPENNI2 "Build the OpenNI 2 Grabber." OFF)
    MARK_AS_ADVANCED(BUILD_OPENNI2)
    if(NOT BUILD_OPENNI2)
      # Set OPENNI2_FOUND to false locally to avoid building anything OpenNI 2 related
      # Remember that other modules (libraries) need to check explicitly for BUILD_OPENNI2
      set(OPENNI2_FOUND FALSE)
    endif()
    if(OPENNI2_FOUND)
        set(OPENNI2_GRABBER_INCLUDES
            include/pcl/io/openni2_grabber.h
            )
        set(OPENNI2_INCLUDES
            include/pcl/io/openni2/openni.h
            include/pcl/io/openni2/openni2_metadata_wrapper.h
            include/pcl/io/openni2/openni2_frame_listener.h
            include/pcl/io/openni2/openni2_timer_filter.h
            include/pcl/io/openni2/openni2_video_mode.h
            include/pcl/io/openni2/openni2_convert.h
            include/pcl/io/openni2/openni2_device.h
            include/pcl/io/openni2/openni2_device_info.h
            include/pcl/io/openni2/openni2_device_manager.h
        )

        set(OPENNI2_GRABBER_SOURCES
            src/openni2_grabber.cpp
            src/openni2/openni2_timer_filter.cpp
            src/openni2/openni2_video_mode.cpp
            src/openni2/openni2_convert.cpp
            src/openni2/openni2_device.cpp
            src/openni2/openni2_device_info.cpp
            src/openni2/openni2_device_manager.cpp
        )

      source_group("OpenNI 2\\Header Files" FILES ${OPENNI2_GRABBER_INCLUDES} ${OPENNI2_INCLUDES} ${IMAGE_INCLUDES})
      source_group("OpenNI 2\\Source Files" FILES ${OPENNI2_GRABBER_SOURCES} ${IMAGE_SOURCES})

        # Copy OpenNI2 redist directory to bin. Needed for driver modules. Only tested on Windows.
        if(MSVC)
            file(COPY ${OPENNI2_REDIST_DIR} DESTINATION ${CMAKE_BINARY_DIR}/bin PATTERN *.*)
        endif(MSVC)
    endif(OPENNI2_FOUND)

    ## OpenNI 1.x
    OPTION(BUILD_OPENNI "Build the OpenNI Grabber." ON)
    MARK_AS_ADVANCED(BUILD_OPENNI)
    if(NOT BUILD_OPENNI)
      # Set OPENNI_FOUND to false locally to avoid building anything OpenNI related
      # Remember that other modules (libraries) need to check explicitly for BUILD_OPENNI
      set(OPENNI_FOUND FALSE)
    endif()
    if(OPENNI_FOUND)
        set(OPENNI_GRABBER_INCLUDES
            include/pcl/io/openni_grabber.h
            include/pcl/io/oni_grabber.h
            )
        set(OPENNI_INCLUDES
            include/pcl/io/openni_camera/openni_shift_to_depth_conversion.h
            include/pcl/io/openni_camera/openni.h
            include/pcl/io/openni_camera/openni_depth_image.h
            include/pcl/io/openni_camera/openni_device.h
            include/pcl/io/openni_camera/openni_device_kinect.h
            include/pcl/io/openni_camera/openni_device_primesense.h
            include/pcl/io/openni_camera/openni_device_xtion.h
            include/pcl/io/openni_camera/openni_device_oni.h
            include/pcl/io/openni_camera/openni_driver.h
            include/pcl/io/openni_camera/openni_exception.h
            include/pcl/io/openni_camera/openni_image.h
            include/pcl/io/openni_camera/openni_image_bayer_grbg.h
            include/pcl/io/openni_camera/openni_image_yuv_422.h
            include/pcl/io/openni_camera/openni_image_rgb24.h
            include/pcl/io/openni_camera/openni_ir_image.h
           ${IMAGE_INCLUDES}
        )
        set(OPENNI_GRABBER_SOURCES
            src/openni_camera/openni_device.cpp
            src/openni_camera/openni_device_primesense.cpp
            src/openni_camera/openni_image_bayer_grbg.cpp
            src/openni_camera/openni_depth_image.cpp
            src/openni_camera/openni_ir_image.cpp
            src/openni_camera/openni_device_kinect.cpp
            src/openni_camera/openni_device_xtion.cpp
            src/openni_camera/openni_device_oni.cpp
            src/openni_camera/openni_driver.cpp
            src/openni_camera/openni_exception.cpp
            src/openni_camera/openni_image_yuv_422.cpp
            src/openni_camera/openni_image_rgb24.cpp
            src/openni_grabber.cpp
            src/oni_grabber.cpp
            ${IMAGE_SOURCES}
        )
    endif(OPENNI_FOUND)

    source_group("Image Headers" FILES ${IMAGE_INCLUDES})
    source_group("Image Sources" FILES ${IMAGE_SOURCES})

    if(FZAPI_FOUND)
        set(FZAPI_GRABBER_INCLUDES
            include/pcl/io/fotonic_grabber.h
            )
#        set(FZAPI_INCLUDES
#            include/pcl/io/openni_camera/openni.h
#            )
        set(FZAPI_GRABBER_SOURCES
            src/fotonic_grabber.cpp
            )
    endif(FZAPI_FOUND)

	if(PXCAPI_FOUND)
		set(PXC_GRABBER_INCLUDES
			include/pcl/io/pxc_grabber.h
			)
		set(PXC_GRABBER_SOURCES
			src/pxc_grabber.cpp
			)
	endif(PXCAPI_FOUND)

    if(LIBUSB_1_FOUND)
        set(DINAST_GRABBER_INCLUDES
            include/pcl/io/dinast_grabber.h
           )
        set(DINAST_GRABBER_SOURCES
            src/dinast_grabber.cpp
           )
    endif(LIBUSB_1_FOUND)

    if (VTK_FOUND AND NOT ANDROID)
        set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
        include("${VTK_USE_FILE}")
        set(VTK_IO_INCLUDES
            "include/pcl/${SUBSYS_NAME}/vtk_lib_io.h"
            "include/pcl/${SUBSYS_NAME}/png_io.h"
           )
        set(VTK_IO_INCLUDES_IMPL
            "include/pcl/${SUBSYS_NAME}/impl/vtk_lib_io.hpp"
           )
        set(VTK_IO_SOURCE
            src/vtk_lib_io.cpp
            src/png_io.cpp
           )
        set(VTK_IO_TARGET_LINK_LIBRARIES vtkCommon vtkWidgets vtkIO vtkImaging vtkHybrid vtkGraphics vtkRendering vtkFiltering vtkVolumeRendering)
        # Indicates that we can rely on VTK to be present
        set(VTK_DEFINES -DPCL_BUILT_WITH_VTK)
    endif ()

    set(PLY_SOURCES src/ply/ply_parser.cpp)
    set(PLY_INCLUDES
        "include/pcl/${SUBSYS_NAME}/ply/byte_order.h"
        "include/pcl/${SUBSYS_NAME}/ply/io_operators.h"
        "include/pcl/${SUBSYS_NAME}/ply/ply.h"
        "include/pcl/${SUBSYS_NAME}/ply/ply_parser.h"
       )
    PCL_ADD_LIBRARY(pcl_io_ply "${SUBSYS_NAME}" ${PLY_SOURCES} ${PLY_INCLUDES})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/ply" ${PLY_INCLUDES})

    set(srcs
        src/debayer.cpp
        src/pcd_grabber.cpp
        src/pcd_io.cpp
        src/vtk_io.cpp
        src/ply_io.cpp
        src/ascii_io.cpp
        src/compression.cpp
        src/lzf.cpp
        src/lzf_image_io.cpp
        src/obj_io.cpp
        src/ifs_io.cpp
        src/image_grabber.cpp
        src/hdl_grabber.cpp
        src/robot_eye_grabber.cpp
        ${VTK_IO_SOURCE}
        ${OPENNI_GRABBER_SOURCES}
        ${OPENNI2_GRABBER_SOURCES}
        ${IMAGE_SOURCES}
        ${DINAST_GRABBER_SOURCES}
        ${FZAPI_GRABBER_SOURCES}
        ${PXC_GRABBER_SOURCES}
        src/file_io.cpp
        )
    if(PNG_FOUND)
      list(APPEND srcs
          src/libpng_wrapper.cpp
          )
    endif(PNG_FOUND)

	if(PCAP_FOUND)
	  set(PCAP_DEFINES -DHAVE_PCAP)
          include_directories(${PCAP_INCLUDE_DIRS})
	  add_definitions(${PCAP_DEFINES})
	endif(PCAP_FOUND)

    set(incs
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/eigen.h"
        "include/pcl/${SUBSYS_NAME}/debayer.h"
        "include/pcl/${SUBSYS_NAME}/file_io.h"
        "include/pcl/${SUBSYS_NAME}/lzf.h"
        "include/pcl/${SUBSYS_NAME}/lzf_image_io.h"
        "include/pcl/${SUBSYS_NAME}/io.h"
        "include/pcl/${SUBSYS_NAME}/grabber.h"
        "include/pcl/${SUBSYS_NAME}/file_grabber.h"
        "include/pcl/${SUBSYS_NAME}/pcd_grabber.h"
        "include/pcl/${SUBSYS_NAME}/pcd_io.h"
        "include/pcl/${SUBSYS_NAME}/vtk_io.h"
        "include/pcl/${SUBSYS_NAME}/ply_io.h"
        "include/pcl/${SUBSYS_NAME}/tar.h"
        "include/pcl/${SUBSYS_NAME}/obj_io.h"
        "include/pcl/${SUBSYS_NAME}/ascii_io.h"
        "include/pcl/${SUBSYS_NAME}/ifs_io.h"
        "include/pcl/${SUBSYS_NAME}/image_grabber.h"
        "include/pcl/${SUBSYS_NAME}/hdl_grabber.h"
        "include/pcl/${SUBSYS_NAME}/robot_eye_grabber.h"
        "include/pcl/${SUBSYS_NAME}/point_cloud_image_extractors.h"
        ${VTK_IO_INCLUDES}
        ${OPENNI_GRABBER_INCLUDES}
        ${OPENNI2_GRABBER_INCLUDES}
        ${IMAGE_INCLUDES}
        ${DINAST_GRABBER_INCLUDES}
        ${FZAPI_GRABBER_INCLUDES}
        ${PXC_GRABBER_INCLUDES}
        )

    set(compression_incs
        include/pcl/compression/octree_pointcloud_compression.h
        include/pcl/compression/color_coding.h
        include/pcl/compression/compression_profiles.h
        include/pcl/compression/entropy_range_coder.h
        include/pcl/compression/point_coding.h
       )
    if(PNG_FOUND)
      list(APPEND compression_incs
          include/pcl/compression/organized_pointcloud_conversion.h
          include/pcl/compression/libpng_wrapper.h
          )
      if(OPENNI_FOUND OR OPENNI2_FOUND)
        list(APPEND compression_incs
          include/pcl/compression/organized_pointcloud_compression.h
          )
      endif(OPENNI_FOUND OR OPENNI2_FOUND)
    endif(PNG_FOUND)

    set(impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/pcd_io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/lzf_image_io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/synchronized_queue.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/point_cloud_image_extractors.hpp"
        include/pcl/compression/impl/entropy_range_coder.hpp
        include/pcl/compression/impl/octree_pointcloud_compression.hpp
        ${VTK_IO_INCLUDES_IMPL}
       )
    if(PNG_FOUND AND (OPENNI_FOUND OR OPENNI2_FOUND) )
     list(APPEND impl_incs
         include/pcl/compression/impl/organized_pointcloud_compression.hpp
        )
    endif(PNG_FOUND AND (OPENNI_FOUND OR OPENNI2_FOUND) )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")

    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" ${VTK_INCLUDE_DIRECTORIES})
    add_definitions(${VTK_DEFINES})
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${compression_incs} ${impl_incs} ${OPENNI_INCLUDES} ${OPENNI2_INCLUDES})
    link_directories(${VTK_LINK_DIRECTORIES})
    target_link_libraries("${LIB_NAME}" pcl_common pcl_io_ply ${VTK_LIBRARIES} )
    if(PNG_FOUND)
      target_link_libraries("${LIB_NAME}" "${PNG_LIBRARY}")
    endif(PNG_FOUND)

    if(LIBUSB_1_FOUND)
      target_link_libraries("${LIB_NAME}" ${LIBUSB_1_LIBRARIES})
    endif(LIBUSB_1_FOUND)

    if(OPENNI2_FOUND)
      target_link_libraries(${LIB_NAME} ${OPENNI2_LIBRARIES})
    endif(OPENNI2_FOUND)

    if(OPENNI_FOUND)
      target_link_libraries("${LIB_NAME}" ${OPENNI_LIBRARIES})
    endif(OPENNI_FOUND)

    if(FZAPI_FOUND)
      target_link_libraries("${LIB_NAME}" ${FZAPI_LIBS})
	  if(WIN32)
		target_link_libraries("${LIB_NAME}" Version.lib)
  	  endif(WIN32)
    endif(FZAPI_FOUND)

	if(PXCAPI_FOUND)
	  link_directories(${PXCAPI_LIB_DIRS})
	  target_link_libraries("${LIB_NAME}" ${PXCAPI_LIBS})
	endif(PXCAPI_FOUND)

    if (PCAP_FOUND)
      target_link_libraries("${LIB_NAME}" ${PCAP_LIBRARIES})
    endif(PCAP_FOUND)

    set(EXT_DEPS eigen3)

    if(OPENNI_FOUND)
      list(APPEND EXT_DEPS openni-dev)
    endif(OPENNI_FOUND)
    if(OPENNI2_FOUND)
      list(APPEND EXT_DEPS openni2-dev)
    endif(OPENNI2_FOUND)

    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" compression ${compression_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/openni_camera" ${OPENNI_INCLUDES})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/openni2" ${OPENNI2_INCLUDES})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

    add_subdirectory(tools)

endif(build)

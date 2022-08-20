set(SUBSYS_NAME io)
set(SUBSYS_DESC "Point cloud IO library")
set(SUBSYS_DEPS common octree)
set(SUBSYS_EXT_DEPS boost eigen)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
if(WIN32)
  PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS openni openni2 ensenso davidSDK dssdk rssdk rssdk2 pcap png vtk EXT_DEPS ${SUBSYS_EXT_DEPS})
else()
  PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS openni openni2 ensenso davidSDK dssdk pcap png vtk libusb EXT_DEPS ${SUBSYS_EXT_DEPS})
endif()

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT WIN32)
  option(PCL_IO_ENABLE_MAND_LOCKING "Enables the use of mandatory locking on filesystems mounted using the mand option." ON)
  mark_as_advanced(PCL_IO_ENABLE_MAND_LOCKING)
  if(NOT PCL_IO_ENABLE_MAND_LOCKING)
    add_definitions(-DNO_MANDATORY_LOCKING)
  endif()
endif()

if(NOT build)
  return()
endif()

if(WITH_OPENNI2)
  set(IMAGE_INCLUDES
    include/pcl/io/image_metadata_wrapper.h
    include/pcl/io/image.h
    include/pcl/io/image_rgb24.h
    include/pcl/io/image_yuv422.h
    include/pcl/io/image_ir.h
    include/pcl/io/image_depth.h
  )

  set(IMAGE_SOURCES
    src/image_rgb24.cpp
    src/image_yuv422.cpp
    src/image_ir.cpp
    src/image_depth.cpp
  )

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
  endif()
endif()

if(WITH_OPENNI)
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
endif()

source_group("Image Headers" FILES ${IMAGE_INCLUDES})
source_group("Image Sources" FILES ${IMAGE_SOURCES})

if(WITH_ENSENSO)
  set(ENSENSO_GRABBER_INCLUDES
    include/pcl/io/ensenso_grabber.h
  )
  set(ENSENSO_GRABBER_SOURCES
    src/ensenso_grabber.cpp
  )
endif()

if(WITH_DAVIDSDK AND VTK_FOUND) # VTK is mandatory for OBJ/PLY/STL input/output
  set(DAVIDSDK_GRABBER_INCLUDES
    include/pcl/io/davidsdk_grabber.h
  )
  set(DAVIDSDK_GRABBER_SOURCES
    src/davidsdk_grabber.cpp
  )
endif()

if(WITH_DSSDK)
  set(DSSDK_GRABBER_INCLUDES
    include/pcl/io/depth_sense_grabber.h
  )
  set(DSSDK_GRABBER_SOURCES
    src/depth_sense_grabber.cpp
    src/depth_sense/depth_sense_grabber_impl.cpp
    src/depth_sense/depth_sense_device_manager.cpp
  )
endif()

if(WITH_RSSDK)
  set(RSSDK_GRABBER_INCLUDES
    include/pcl/io/real_sense_grabber.h
  )
  set(RSSDK_GRABBER_SOURCES
    src/real_sense_grabber.cpp
    src/real_sense/real_sense_device_manager.cpp
  )
endif()

if(WITH_RSSDK2)
  set(RSSDK2_GRABBER_INCLUDES
      include/pcl/io/real_sense_2_grabber.h
  )
  set(RSSDK2_GRABBER_SOURCES
      src/real_sense_2_grabber.cpp
  )
endif()

if(LIBUSB_FOUND)
  set(DINAST_GRABBER_INCLUDES
    include/pcl/io/dinast_grabber.h
  )
  set(DINAST_GRABBER_SOURCES
    src/dinast_grabber.cpp
  )
endif()

if(VTK_FOUND AND NOT ANDROID)
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

  # Indicates that we can rely on VTK to be present
  set(VTK_DEFINES -DPCL_BUILT_WITH_VTK)
endif()

set(PLY_SOURCES src/ply/ply_parser.cpp)
set(PLY_INCLUDES
  "include/pcl/${SUBSYS_NAME}/ply/byte_order.h"
  "include/pcl/${SUBSYS_NAME}/ply/io_operators.h"
  "include/pcl/${SUBSYS_NAME}/ply/ply.h"
  "include/pcl/${SUBSYS_NAME}/ply/ply_parser.h"
)

PCL_ADD_LIBRARY(pcl_io_ply COMPONENT ${SUBSYS_NAME} SOURCES ${PLY_SOURCES} ${PLY_INCLUDES})
if(MINGW)
  # libws2_32 isn't added by default for MinGW
  target_link_libraries(pcl_io_ply ws2_32)
endif()
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/ply" ${PLY_INCLUDES})
target_include_directories(pcl_io_ply PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(pcl_io_ply Boost::boost)

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
  src/vlp_grabber.cpp
  src/robot_eye_grabber.cpp
  src/auto_io.cpp
  src/io_exception.cpp
  src/tim_grabber.cpp
  ${VTK_IO_SOURCE}
  ${OPENNI_GRABBER_SOURCES}
  ${OPENNI2_GRABBER_SOURCES}
  ${IMAGE_SOURCES}
  ${DINAST_GRABBER_SOURCES}
  ${ENSENSO_GRABBER_SOURCES}
  ${DAVIDSDK_GRABBER_SOURCES}
  ${DSSDK_GRABBER_SOURCES}
  ${RSSDK_GRABBER_SOURCES}
  ${RSSDK2_GRABBER_SOURCES}
)

if(PNG_FOUND)
  list(APPEND srcs
    src/libpng_wrapper.cpp
  )
endif()

if(PCAP_FOUND)
  set(PCAP_DEFINES -DHAVE_PCAP)
  include_directories(SYSTEM ${PCAP_INCLUDE_DIRS})
  add_definitions(${PCAP_DEFINES})
endif()

set(incs
  "include/pcl/${SUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/eigen.h"
  "include/pcl/${SUBSYS_NAME}/debayer.h"
  "include/pcl/${SUBSYS_NAME}/fotonic_grabber.h"
  "include/pcl/${SUBSYS_NAME}/file_io.h"
  "include/pcl/${SUBSYS_NAME}/auto_io.h"
  "include/pcl/${SUBSYS_NAME}/low_level_io.h"
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
  "include/pcl/${SUBSYS_NAME}/vlp_grabber.h"
  "include/pcl/${SUBSYS_NAME}/robot_eye_grabber.h"
  "include/pcl/${SUBSYS_NAME}/point_cloud_image_extractors.h"
  "include/pcl/${SUBSYS_NAME}/io_exception.h"
  "include/pcl/${SUBSYS_NAME}/tim_grabber.h"
  ${VTK_IO_INCLUDES}
  ${OPENNI_GRABBER_INCLUDES}
  ${OPENNI2_GRABBER_INCLUDES}
  ${IMAGE_INCLUDES}
  ${DINAST_GRABBER_INCLUDES}
  ${ENSENSO_GRABBER_INCLUDES}
  ${DAVIDSDK_GRABBER_INCLUDES}
  ${DSSDK_GRABBER_INCLUDES}
  ${RSSDK_GRABBER_INCLUDES}
  ${RSSDK2_GRABBER_INCLUDES}
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
  if(WITH_OPENNI OR WITH_OPENNI2)
  list(APPEND compression_incs
    include/pcl/compression/organized_pointcloud_compression.h
  )
  endif()
endif()

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/ascii_io.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/pcd_io.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/auto_io.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/lzf_image_io.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/synchronized_queue.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/point_cloud_image_extractors.hpp"
  include/pcl/compression/impl/entropy_range_coder.hpp
  include/pcl/compression/impl/octree_pointcloud_compression.hpp
  ${VTK_IO_INCLUDES_IMPL}
)

if(PNG_FOUND AND (WITH_OPENNI OR WITH_OPENNI2))
  list(APPEND impl_incs
    include/pcl/compression/impl/organized_pointcloud_compression.hpp
  )
endif()

set(LIB_NAME "pcl_${SUBSYS_NAME}")

add_definitions(${VTK_DEFINES})

PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${compression_incs} ${impl_incs} ${OPENNI_INCLUDES} ${OPENNI2_INCLUDES})

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries("${LIB_NAME}" Boost::boost Boost::filesystem Boost::iostreams pcl_common pcl_io_ply)

if(VTK_FOUND)
  if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0)
    target_link_libraries("${LIB_NAME}" 
                          VTK::IOImage
                          VTK::IOGeometry
                          VTK::IOPLY)
  else()
    link_directories(${VTK_LINK_DIRECTORIES})
    target_link_libraries("${LIB_NAME}"
                          vtkIOImage
                          vtkIOGeometry
                          vtkIOPLY
                          )
    if(${VTK_VERSION} VERSION_LESS 8.0)
      target_link_libraries("${LIB_NAME}" vtkIOMPIImage vtkIOMPIParallel)
      if(${VTK_VERSION} VERSION_GREATER_EQUAL 7.0)
        target_link_libraries("${LIB_NAME}" vtkFiltersParallelDIY2)
      endif()
    else()
      target_link_libraries("${LIB_NAME}" vtkImagingCore)
    endif()
  endif()
endif()

if(PNG_FOUND)
  target_link_libraries("${LIB_NAME}" ${PNG_LIBRARIES})
endif()

if(LIBUSB_FOUND)
  target_link_libraries("${LIB_NAME}" libusb::libusb)
endif()

if(WITH_OPENNI2)
  target_link_libraries(${LIB_NAME} ${OPENNI2_LIBRARIES})
endif()

if(WITH_OPENNI)
  target_link_libraries("${LIB_NAME}" ${OPENNI_LIBRARIES})
  if(VTK_FOUND AND (NOT ${VTK_VERSION} VERSION_LESS 9.0))
    target_link_libraries("${LIB_NAME}" VTK::FiltersCore VTK::FiltersGeneral)
  endif()
endif()

if(WITH_FZAPI)
  target_link_libraries("${LIB_NAME}" ${FZAPI_LIBS})
  if(WIN32)
  target_link_libraries("${LIB_NAME}" Version.lib)
  endif()
endif()

if(WITH_ENSENSO)
  target_link_libraries(${LIB_NAME} ${ENSENSO_LIBRARIES})
endif()

if(WITH_DAVIDSDK)
  target_link_libraries(${LIB_NAME} ${DAVIDSDK_LIBRARIES})
endif()

if(WITH_DSSDK)
  target_link_libraries(${LIB_NAME} ${DSSDK_LIBRARIES})
endif()

if(WITH_RSSDK)
  target_link_libraries(${LIB_NAME} ${RSSDK_LIBRARIES})
endif()

if(WITH_RSSDK2)
  target_link_libraries(${LIB_NAME} ${RSSDK2_LIBRARIES})
endif()

if(PCAP_FOUND)
  target_link_libraries("${LIB_NAME}" ${PCAP_LIBRARIES})
endif()

set(EXT_DEPS eigen3) # Although this depends on boost, that cannot be specified here because there is no boost.pc

if(WITH_OPENNI)
  list(APPEND EXT_DEPS libopenni)
endif()
if(WITH_OPENNI2)
  list(APPEND EXT_DEPS libopenni2)
endif()
if(WITH_ENSENSO)
  list(APPEND EXT_DEPS ensenso)
endif()

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" compression ${compression_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/openni_camera" ${OPENNI_INCLUDES})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/openni2" ${OPENNI2_INCLUDES})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

if(BUILD_tools)
  add_subdirectory(tools)
endif()

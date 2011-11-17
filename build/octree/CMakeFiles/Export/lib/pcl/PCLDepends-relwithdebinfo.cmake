#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
SET(CMAKE_IMPORT_FILE_VERSION 1)

# Compute the installation prefix relative to this file.
GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)

# Import target "pcl_common" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_common PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_common.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_common.so.1.1"
  )

# Import target "pcl_octree" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_octree APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_octree PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_octree.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_octree.so.1.1"
  )

# Import target "pcl_io" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_io APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_io PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_common;/usr/lib/libOpenNI.so"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_io.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_io.so.1.1"
  )

# Import target "pcl_kdtree" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_kdtree APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_kdtree PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_common;/usr/lib/libflann_cpp.so"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_kdtree.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_kdtree.so.1.1"
  )

# Import target "pcl_range_image" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_range_image APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_range_image PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;gomp;pcl_common"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_range_image.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_range_image.so.1.1"
  )

# Import target "pcl_sample_consensus" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_sample_consensus APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_sample_consensus PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_common;/usr/lib/libcminpack.so"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_sample_consensus.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_sample_consensus.so.1.1"
  )

# Import target "pcl_filters" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_filters APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_filters PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_common;pcl_sample_consensus;pcl_kdtree"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_filters.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_filters.so.1.1"
  )

# Import target "pcl_features" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_features APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_features PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;gomp;pcl_range_image;pcl_kdtree"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_features.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_features.so.1.1"
  )

# Import target "pcl_range_image_border_extractor" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_range_image_border_extractor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_range_image_border_extractor PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_range_image;pcl_kdtree"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_range_image_border_extractor.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_range_image_border_extractor.so.1.1"
  )

# Import target "pcl_keypoints" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_keypoints APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_keypoints PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_range_image_border_extractor"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_keypoints.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_keypoints.so.1.1"
  )

# Import target "pcl_registration" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_registration APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_registration PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_kdtree;pcl_sample_consensus;pcl_features"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_registration.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_registration.so.1.1"
  )

# Import target "pcl_segmentation" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_segmentation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_segmentation PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_kdtree;pcl_sample_consensus"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_segmentation.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_segmentation.so.1.1"
  )

# Import target "pcl_surface" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_surface APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_surface PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_common;pcl_kdtree;/usr/lib/libqhull.so"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_surface.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_surface.so.1.1"
  )

# Import target "pcl_visualization" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET pcl_visualization APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(pcl_visualization PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;pcl_io;pcl_kdtree;pcl_range_image;-pthread;-Wl,-Bsymbolic-functions;-lwx_gtk2u_richtext-2.8;-lwx_gtk2u_aui-2.8;-lwx_gtk2u_xrc-2.8;-lwx_gtk2u_qa-2.8;-lwx_gtk2u_html-2.8;-lwx_gtk2u_adv-2.8;-lwx_gtk2u_core-2.8;-lwx_baseu_xml-2.8;-lwx_baseu_net-2.8;-lwx_baseu-2.8;vtkCommon;vtkWidgets"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpcl_visualization.so.1.1.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libpcl_visualization.so.1.1"
  )

# Cleanup temporary variables.
SET(_IMPORT_PREFIX)

# Commands beyond this point should not need to know the version.
SET(CMAKE_IMPORT_FILE_VERSION)

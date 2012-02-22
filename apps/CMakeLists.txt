set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search)

# Find VTK 
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    include (${VTK_USE_FILE})
endif(NOT VTK_FOUND)

# OpenNI found?
if(NOT OPENNI_FOUND)
    set(DEFAULT FALSE)
    set(REASON "OpenNI was not found.")
else(NOT OPENNI_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
endif(NOT OPENNI_FOUND)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS openni vtk)

if(build)

  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

  PCL_ADD_EXECUTABLE(test_search_speed ${SUBSYS_NAME} src/test_search.cpp)
  target_link_libraries(test_search_speed pcl_common pcl_io pcl_search pcl_kdtree)
  
  PCL_ADD_EXECUTABLE(nn_classification_example ${SUBSYS_NAME} src/nn_classification_example.cpp)
  target_link_libraries(nn_classification_example pcl_common pcl_io pcl_features pcl_kdtree)
  
  PCL_ADD_EXECUTABLE(pyramid_surface_matching ${SUBSYS_NAME} src/pyramid_surface_matching.cpp)
  target_link_libraries(pyramid_surface_matching pcl_common pcl_io pcl_features pcl_registration pcl_filters)
  
  PCL_ADD_EXECUTABLE(statistical_multiscale_interest_region_extraction_example ${SUBSYS_NAME} src/statistical_multiscale_interest_region_extraction_example.cpp)
  target_link_libraries(statistical_multiscale_interest_region_extraction_example pcl_common pcl_io pcl_features pcl_filters)

  if (VTK_FOUND)
    PCL_ADD_EXECUTABLE(ppf_object_recognition ${SUBSYS_NAME} src/ppf_object_recognition.cpp)
    target_link_libraries(ppf_object_recognition pcl_common pcl_io pcl_filters pcl_features pcl_registration pcl_visualization pcl_sample_consensus pcl_segmentation)  
    
    PCL_ADD_EXECUTABLE(multiscale_feature_persistence_example ${SUBSYS_NAME} src/multiscale_feature_persistence_example.cpp)
    target_link_libraries(multiscale_feature_persistence_example pcl_common pcl_io pcl_filters pcl_features pcl_visualization)
    
    PCL_ADD_EXECUTABLE(surfel_smoothing_test ${SUBSYS_NAME} src/surfel_smoothing_test.cpp)
    target_link_libraries(surfel_smoothing_test pcl_common pcl_io pcl_surface pcl_filters pcl_features pcl_visualization)
    
    PCL_ADD_EXECUTABLE(feature_matching ${SUBSYS_NAME} src/feature_matching.cpp)
    target_link_libraries(feature_matching pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation)
 
    if (OPENNI_FOUND AND BUILD_OPENNI)  
      PCL_ADD_EXECUTABLE(openni_fast_mesh ${SUBSYS_NAME} src/openni_fast_mesh.cpp)
      target_link_libraries(openni_fast_mesh pcl_common pcl_io pcl_visualization pcl_surface)
      
      PCL_ADD_EXECUTABLE(openni_voxel_grid ${SUBSYS_NAME} src/openni_voxel_grid.cpp)
      target_link_libraries(openni_voxel_grid pcl_common pcl_io pcl_filters pcl_visualization)

      PCL_ADD_EXECUTABLE(openni_passthrough ${SUBSYS_NAME} src/openni_passthrough.cpp)
      target_link_libraries(openni_passthrough pcl_common pcl_io pcl_filters pcl_visualization)

      PCL_ADD_EXECUTABLE(openni_stream_compression ${SUBSYS_NAME} src/openni_stream_compression.cpp)
      target_link_libraries(openni_stream_compression pcl_common pcl_io pcl_filters pcl_visualization pcl_octree)
      
      PCL_ADD_EXECUTABLE(openni_planar_segmentation ${SUBSYS_NAME} src/openni_planar_segmentation.cpp)
      target_link_libraries(openni_planar_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus)

      PCL_ADD_EXECUTABLE(openni_ii_normal_estimation ${SUBSYS_NAME} src/openni_ii_normal_estimation.cpp)
      target_link_libraries(openni_ii_normal_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)
      
      PCL_ADD_EXECUTABLE(openni_feature_persistence ${SUBSYS_NAME} src/openni_feature_persistence.cpp)
      target_link_libraries(openni_feature_persistence pcl_common pcl_io pcl_filters pcl_visualization pcl_features)
      
      PCL_ADD_EXECUTABLE(openni_mls_smoothing ${SUBSYS_NAME} src/openni_mls_smoothing.cpp)
      target_link_libraries(openni_mls_smoothing pcl_common pcl_io pcl_surface pcl_visualization)
      
      PCL_ADD_EXECUTABLE(openni_change_viewer ${SUBSYS_NAME} src/openni_change_viewer.cpp)
      target_link_libraries(openni_change_viewer pcl_common pcl_io pcl_kdtree pcl_octree pcl_visualization pcl_filters)
      
      PCL_ADD_EXECUTABLE(openni_uniform_sampling ${SUBSYS_NAME} src/openni_uniform_sampling.cpp)
      target_link_libraries(openni_uniform_sampling pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface pcl_keypoints)
      
      PCL_ADD_EXECUTABLE(openni_boundary_estimation ${SUBSYS_NAME} src/openni_boundary_estimation.cpp)
      target_link_libraries(openni_boundary_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)
  
      if (QHULL_FOUND)
        PCL_ADD_EXECUTABLE(openni_3d_convex_hull ${SUBSYS_NAME} src/openni_3d_convex_hull.cpp)
        target_link_libraries(openni_3d_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)
       
        PCL_ADD_EXECUTABLE(openni_3d_concave_hull ${SUBSYS_NAME} src/openni_3d_concave_hull.cpp)
        target_link_libraries(openni_3d_concave_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)
      
        PCL_ADD_EXECUTABLE(openni_tracking ${SUBSYS_NAME} src/openni_tracking.cpp)
        target_link_libraries(openni_tracking pcl_common pcl_io pcl_surface pcl_visualization pcl_filters pcl_features pcl_segmentation pcl_tracking pcl_search)
                
        set(incs include/pcl/${SUBSYS_NAME}/dominant_plane_segmentation.h)  
        set(impl_incs include/pcl/${SUBSYS_NAME}/impl/dominant_plane_segmentation.hpp)
        set(srcs src/dominant_plane_segmentation.cpp)
         
        # Install include files
        PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
        PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
          
        set(LIB_NAME pcl_${SUBSYS_NAME})
        PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${impl_incs} ${incs})
        target_link_libraries(${LIB_NAME} pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search)
            
        PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")
           
      endif() # QHULL_FOUND
      
      PCL_ADD_EXECUTABLE(openni_planar_convex_hull ${SUBSYS_NAME} src/openni_planar_convex_hull.cpp)
      target_link_libraries(openni_planar_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface)
    
    endif() # OPENNI_FOUND + BUILD_OPENNI
    
  endif() # VTK_FOUND

endif(build)

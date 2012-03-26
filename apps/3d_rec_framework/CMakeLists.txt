set(SUBSYS_NAME 3d_rec_framework)
set(SUBSYS_DESC "3D recognition framework")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition)

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

set(build OFF)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS vtk openni)

if(build)

  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(incs include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/normal_estimator.h
           include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/global/cvfh_estimator.h 
           include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/global/vfh_estimator.h
           include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/global/esf_estimator.h
           include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/local/local_estimator.h
           include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/local/fpfh_local_estimator.h
           include/pcl/apps/${SUBSYS_NAME}/feature_wrapper/local/shot_local_estimator.h
           include/pcl/apps/${SUBSYS_NAME}/pc_source/source.h
           include/pcl/apps/${SUBSYS_NAME}/pc_source/mesh_source.h
           include/pcl/apps/${SUBSYS_NAME}/tools/openni_frame_source.h
           include/pcl/apps/${SUBSYS_NAME}/pipeline/global_nn_classifier.h 
           include/pcl/apps/${SUBSYS_NAME}/pipeline/local_recognizer.h
           include/pcl/apps/${SUBSYS_NAME}/utils/metrics.h
           include/pcl/apps/${SUBSYS_NAME}/utils/persistence_utils.h
           include/pcl/apps/${SUBSYS_NAME}/utils/vtk_model_sampling.h)
             
  set(srcs src/pipeline/global_nn_classifier.cpp
           src/pipeline/local_recognizer.cpp 
           src/tools/openni_frame_source.cpp)
           
  set(impl_incs include/pcl/apps/${SUBSYS_NAME}/pipeline/impl/global_nn_classifier.hpp
                include/pcl/apps/${SUBSYS_NAME}/pipeline/impl/local_recognizer.hpp)
      
  # Install include files
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    
  set(LIB_NAME pcl_${SUBSYS_NAME})
  PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${impl_incs} ${incs})
  target_link_libraries(${LIB_NAME} pcl_apps pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search)
      
  PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")
  
  add_subdirectory(tools)

endif(build)

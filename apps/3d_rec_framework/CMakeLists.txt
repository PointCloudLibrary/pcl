set(SUBSUBSYS_NAME 3d_rec_framework)
set(SUBSUBSYS_DESC "3D recognition framework")
set(SUBSUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition ml)

# Find VTK 
if(NOT VTK_FOUND)
    set(DEFAULT AUTO_OFF)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    include("${VTK_USE_FILE}")
endif(NOT VTK_FOUND)

# OpenNI found?
if(NOT WITH_OPENNI)
    set(DEFAULT AUTO_OFF)
    set(REASON "OpenNI was not found or was disabled by the user.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
    set(DEFAULT TRUE)
    set(REASON)
endif()

# Default to not building for now
if (${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSUBSYS_DEPS} EXT_DEPS vtk openni)

if(build)

  include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

  set(incs_fw "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/normal_estimator.h")
  set(incs_fw_global "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global/cvfh_estimator.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global/vfh_estimator.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global/esf_estimator.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global/crh_estimator.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global/global_estimator.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global/ourcvfh_estimator.h")
                     
  set(incs_fw_local "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local/local_estimator.h"
                    "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local/fpfh_local_estimator.h"
                    "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local/fpfh_local_estimator_omp.h"
                    "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local/shot_local_estimator.h"
                    "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local/colorshot_local_estimator.h"
                    "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local/shot_local_estimator_omp.h")
  set(incs_pc_source "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pc_source/source.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pc_source/mesh_source.h"
                     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pc_source/registered_views_source.h")
                     
  set(incs_pipelines "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/global_nn_classifier.h"
           "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/global_nn_recognizer_crh.h"
           "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/global_nn_recognizer_cvfh.h"
           "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/local_recognizer.h")
           
  set(incc_tools_framework "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/openni_frame_source.h")
  
  set(incs_utils "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/utils/metrics.h"
           "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/utils/persistence_utils.h"
           "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/utils/vtk_model_sampling.h")
           
  set(srcs src/pipeline/global_nn_classifier.cpp
           src/pipeline/global_nn_recognizer_crh.cpp
           src/pipeline/global_nn_recognizer_cvfh.cpp
           src/pipeline/local_recognizer.cpp 
           src/tools/openni_frame_source.cpp)
           
  set(impl_incs_pipeline "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/impl/global_nn_classifier.hpp"
                "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/impl/global_nn_recognizer_crh.hpp"
                "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/impl/global_nn_recognizer_cvfh.hpp"
                "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/impl/local_recognizer.hpp")
      
  # Install include files
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper" ${incs_fw})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/global" ${incs_fw_global})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/feature_wrapper/local" ${incs_fw_local})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools" ${incc_tools_framework})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline" ${incs_pipelines})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pc_source" ${incs_pc_source})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/utils" ${incs_utils})
  
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/pipeline/impl" ${impl_incs_pipeline})
       
  set(LIB_NAME "pcl_${SUBSUBSYS_NAME}")
  PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSUBSYS_NAME}" ${srcs} ${impl_incs_pipeline} ${incs_utils} ${incs_fw} ${incs_fw_global} ${incs_fw_local} ${incc_tools_framework} ${incs_pipelines} ${incs_pc_source})
  target_link_libraries("${LIB_NAME}" pcl_apps pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search)
  
  if(WITH_OPENNI)
    target_link_libraries("${LIB_NAME}" ${OPENNI_LIBRARIES})
    if(NOT WIN32)
      find_package(libusb-1.0 REQUIRED)
      target_link_libraries("${LIB_NAME}" ${LIBUSB_1_LIBRARIES})
    endif()
  endif()
      
  PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" "" "" "" "" "")
  
  add_subdirectory(tools)

endif(build)

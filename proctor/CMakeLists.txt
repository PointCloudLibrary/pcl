set(SUBSYS_NAME proctor)
set(SUBSYS_DESC "Proctor: 3D feature benchmarking")
set(SUBSYS_DEPS common io filters sample_consensus segmentation search kdtree features surface octree search registration visualization keypoints)
set(DEFAULT OFF)
set(REASON "")

if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif(NOT VTK_FOUND)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS vtk)

if (ANDROID_NDK)
  set (build FALSE)
endif ()

if(build)
  include_directories(include/pcl/${SUBSYS_NAME})
  include_directories(include/pcl/)

  set(incs
      include/pcl/${SUBSYS_NAME}/config.h
      include/pcl/${SUBSYS_NAME}/detector.h
      include/pcl/${SUBSYS_NAME}/ia_ransac_sub.h
      include/pcl/${SUBSYS_NAME}/proctor.h
      include/pcl/${SUBSYS_NAME}/scanner.h
      include/pcl/${SUBSYS_NAME}/timer.h
      include/pcl/${SUBSYS_NAME}/vtkLidarPoint.h
      include/pcl/${SUBSYS_NAME}/vtkLidarScanner.h
      include/pcl/${SUBSYS_NAME}/vtkRay.h
      )

  set(impl_incs
      include/pcl/${SUBSYS_NAME}/impl/ia_ransac_sub.hpp
      include/pcl/${SUBSYS_NAME}/impl/timer.hpp
      )

  PCL_ADD_EXECUTABLE(proctor 
                     ${SUBSYS_NAME} 
                     ${incs}
                     ${impl_incs}
                     src/main.cpp 
                     src/detector.cpp 
                     src/proctor.cpp 
                     src/scanner.cpp
                    )
  target_link_libraries(proctor 
                        ${FLANN_LIBRARIES} 
                        pcl_common pcl_features pcl_keypoints pcl_io pcl_search pcl_visualization
                        vtkCommon vtkFiltering vtkGraphics vtkIO)
endif(build)


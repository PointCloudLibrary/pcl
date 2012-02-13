set(SUBSYS_NAME proctor)
set(SUBSYS_DESC "Proctor: 3D feature benchmarking")
set(SUBSYS_DEPS common io filters sample_consensus segmentation search kdtree features surface octree search registration visualization keypoints)
set(DEFAULT OFF)
set(REASON "")

if(VTK_FOUND AND VTK_USE_QVTK AND ${Boost_MPI_FOUND} AND ${Boost_SERIALIZATION_FOUND})
    set(DEFAULT TRUE)
    set(REASON)
    set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
elseif(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
elseif(NOT VTK_USE_QVTK)
    set(DEFAULT FALSE)
    set(REASON "VTK was not built with Qt support.")
elseif(NOT Boost_MPI_FOUND OR NOT Boost_SERIALIZATION_FOUND)
    set(DEFAULT FALSE)
    set(REASON "Boost_MPI or Boost_serialization was not found.")
endif(VTK_FOUND AND VTK_USE_QVTK AND ${Boost_MPI_FOUND} AND ${Boost_SERIALIZATION_FOUND})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include (${VTK_USE_FILE})

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS vtk)

if (ANDROID_NDK)
  set (build FALSE)
endif ()

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
  include_directories(include/pcl/${SUBSYS_NAME})
  include_directories(include/pcl/)

  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(incs
      include/pcl/${SUBSYS_NAME}/config.h
      include/pcl/${SUBSYS_NAME}/confusion_matrix.h
      include/pcl/${SUBSYS_NAME}/database_entry.h
      include/pcl/${SUBSYS_NAME}/detector.h

      include/pcl/${SUBSYS_NAME}/model_source.h
      include/pcl/${SUBSYS_NAME}/scanning_model_source.h
      include/pcl/${SUBSYS_NAME}/primitive_model_source.h

      include/pcl/${SUBSYS_NAME}/proctor.h
      include/pcl/${SUBSYS_NAME}/proctor_mpi.h
      include/pcl/${SUBSYS_NAME}/scanner.h
      include/pcl/${SUBSYS_NAME}/timer.h
      include/pcl/${SUBSYS_NAME}/vtkLidarPoint.h
      include/pcl/${SUBSYS_NAME}/vtkLidarScanner.h
      include/pcl/${SUBSYS_NAME}/vtkRay.h
      include/pcl/${SUBSYS_NAME}/detector_visualizer.h

      include/pcl/${SUBSYS_NAME}/proposer.h
      include/pcl/${SUBSYS_NAME}/basic_proposer.h
      include/pcl/${SUBSYS_NAME}/inverse_bag_proposer.h
      include/pcl/${SUBSYS_NAME}/registration_proposer.h
      include/pcl/${SUBSYS_NAME}/hough_proposer.h
      include/pcl/${SUBSYS_NAME}/soft_hough_proposer.h

      include/pcl/${SUBSYS_NAME}/feature_wrapper.h
      include/pcl/${SUBSYS_NAME}/fpfh_wrapper.h
      include/pcl/${SUBSYS_NAME}/shot_wrapper.h
      include/pcl/${SUBSYS_NAME}/si_wrapper.h
      include/pcl/${SUBSYS_NAME}/3dsc_wrapper.h

      include/pcl/${SUBSYS_NAME}/keypoint_wrapper.h
      include/pcl/${SUBSYS_NAME}/uniform_sampling_wrapper.h
      include/pcl/${SUBSYS_NAME}/harris_wrapper.h
      )

  set(impl_incs
      include/pcl/${SUBSYS_NAME}/impl/timer.hpp
      )

  # TODO Get it to build without a visualizer when QT4 does not exist
  if (QT4_FOUND AND VTK_USE_QVTK)
    QT4_WRAP_UI(detector_visualizer_ui src/detector_visualizer.ui)
    QT4_WRAP_CPP(detector_visualizer_moc include/pcl/proctor/detector_visualizer.h src/detector_visualizer.ui)

    PCL_ADD_EXECUTABLE(proctor 
                      ${SUBSYS_NAME} 
                      ${detector_visualizer_ui}
                      ${detector_visualizer_moc}
                      src/detector_visualizer.cpp
                      ${incs}
                      ${impl_incs}
                      src/confusion_matrix.cpp
                      src/main.cpp
                      src/model_source.cpp
                      src/detector.cpp
                      src/proctor.cpp
                      #src/proctor_mpi.cpp
                      src/proposer.cpp
                      src/scanning_model_source.cpp
                      src/primitive_model_source.cpp
                      src/scanner.cpp

                      src/basic_proposer.cpp
                      src/inverse_bag_proposer.cpp
                      src/registration_proposer.cpp
                      src/hough_proposer.cpp
                      src/soft_hough_proposer.cpp

                      src/fpfh_wrapper.cpp
                      src/shot_wrapper.cpp
                      src/si_wrapper.cpp
                      src/3dsc_wrapper.cpp

                      src/keypoint_wrapper.cpp
                      src/uniform_sampling_wrapper.cpp
                      src/harris_wrapper.cpp
                      )

    target_link_libraries(proctor 
                          ${FLANN_LIBRARIES} 
                          pcl_common pcl_features pcl_keypoints pcl_io pcl_search pcl_visualization
                          vtkCommon vtkFiltering vtkGraphics vtkIO ${Boost_MPI_LIBRARY} ${MPI_CXX_LIBRARIES} ${Boost_SERIALIZATION_LIBRARY}
                          pcl_filters QVTK ${QT_LIBRARIES})
  endif ()
#boost_mpi
endif(build)


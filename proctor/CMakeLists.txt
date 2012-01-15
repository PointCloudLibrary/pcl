set(SUBSYS_NAME proctor)
set(SUBSYS_DESC "Proctor: 3D feature benchmarking")
set(SUBSYS_DEPS common io filters sample_consensus segmentation search kdtree features surface octree search registration visualization keypoints)
set(DEFAULT OFF)
set(REASON "")

find_package(Boost 1.40.0 COMPONENTS mpi)

if(NOT VTK_FOUND OR NOT Boost_MPI_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK or MPI was not found.")
    MESSAGE(${REASON})
else(NOT VTK_FOUND OR NOT Boost_MPI_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
    include (${VTK_USE_FILE})
endif(NOT VTK_FOUND OR NOT Boost_MPI_FOUND)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

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
      include/pcl/${SUBSYS_NAME}/basic_proposer.h
      include/pcl/${SUBSYS_NAME}/config.h
      include/pcl/${SUBSYS_NAME}/confusion_matrix.h
      include/pcl/${SUBSYS_NAME}/database_entry.h
      include/pcl/${SUBSYS_NAME}/detector.h
      include/pcl/${SUBSYS_NAME}/model_source.h
      include/pcl/${SUBSYS_NAME}/scanning_model_source.h
      include/pcl/${SUBSYS_NAME}/proctor.h
      include/pcl/${SUBSYS_NAME}/proctor_mpi.h
      include/pcl/${SUBSYS_NAME}/proposer.h
      include/pcl/${SUBSYS_NAME}/registration_proposer.h
      include/pcl/${SUBSYS_NAME}/scanner.h
      include/pcl/${SUBSYS_NAME}/timer.h
      include/pcl/${SUBSYS_NAME}/vtkLidarPoint.h
      include/pcl/${SUBSYS_NAME}/vtkLidarScanner.h
      include/pcl/${SUBSYS_NAME}/vtkRay.h
      include/pcl/${SUBSYS_NAME}/detector_visualizer.h
      )

  set(impl_incs
      include/pcl/${SUBSYS_NAME}/impl/timer.hpp
      )

  # TODO Get it to build without a visualizer when QT4 does not exist
  if (QT4_FOUND)
    QT4_WRAP_UI(detector_visualizer_ui src/detector_visualizer.ui)
    QT4_WRAP_CPP(detector_visualizer_moc include/pcl/proctor/detector_visualizer.h src/detector_visualizer.ui)

    PCL_ADD_EXECUTABLE(proctor 
                      ${SUBSYS_NAME} 
                      ${detector_visualizer_ui}
                      ${detector_visualizer_moc}
                      src/detector_visualizer.cpp
                      ${incs}
                      ${impl_incs}
                      src/basic_proposer.cpp
                      src/main.cpp
                      src/model_source.cpp
                      src/scanning_model_source.cpp
                      src/confusion_matrix.cpp
                      src/detector.cpp
                      src/proctor.cpp
                      src/proposer.cpp
                      src/registration_proposer.cpp
                      #src/proctor_mpi.cpp
                      src/scanner.cpp
                      )

    PCL_ADD_OPENMP_FLAGS(proctor)
    PCL_LINK_OPENMP(proctor)

    target_link_libraries(proctor 
                          ${FLANN_LIBRARIES} 
                          pcl_common pcl_features pcl_keypoints pcl_io pcl_search pcl_visualization
                          vtkCommon vtkFiltering vtkGraphics vtkIO ${Boost_mpi_LIBRARY} ${MPI_CXX_LIBRARIES} boost_serialization
                          pcl_filters QVTK ${QT_LIBRARIES})
  endif ()
#boost_mpi
endif(build)


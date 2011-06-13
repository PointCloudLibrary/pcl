cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# we need FindFLANN.cmake 
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

project(vfh_cluster_classifier)

find_package(PCL 1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(HDF5)
if(HDF5_FOUND)

  find_package(Boost 1.40.0 COMPONENTS system filesystem thread date_time)
  include_directories(${Boost_INCLUDE_DIRS})

  find_package(Flann)
  include_directories(${FLANN_INCLUDE_DIRS})

  find_package(VTK REQUIRED)
  include (${VTK_USE_FILE})
  link_directories(${VTK_LIBRARY_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(build_tree build_tree.cpp)
  target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(nearest_neighbors nearest_neighbors.cpp)
  target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} 
                                        ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} 
                                        vtkCommon vtkRendering)
endif(HDF5_FOUND)

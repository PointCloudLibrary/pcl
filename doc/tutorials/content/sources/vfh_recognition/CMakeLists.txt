cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(vfh_cluster_classifier)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(HDF5 REQUIRED)
find_package(FLANN REQUIRED)

include_directories(SYSTEM
  ${HDF5_INCLUDE_DIR}
)

add_executable(build_tree build_tree.cpp)
target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 FLANN::FLANN ${HDF5_LIBRARIES})

add_executable(nearest_neighbors nearest_neighbors.cpp)
target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} ${Boost_LIBRARIES} FLANN::FLANN ${HDF5_LIBRARIES})

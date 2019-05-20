if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
  include("${VTK_USE_FILE}")
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

set(the_target people_tracking)

include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

#PCL_ADD_EXECUTABLE(${the_target} "${SUBSYS_NAME}" people_tracking.cpp)
#target_link_libraries("${the_target}" pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization)

if(HAVE_OPENNI)
  PCL_ADD_EXECUTABLE(pcl_people_app COMPONENT ${SUBSYS_NAME} SOURCES people_app.cpp)
  target_link_libraries (pcl_people_app pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization ${Boost_LIBRARIES})
endif()

PCL_ADD_EXECUTABLE(pcl_people_pcd_prob COMPONENT ${SUBSYS_NAME} SOURCES people_pcd_prob.cpp)
target_link_libraries (pcl_people_pcd_prob pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization ${Boost_LIBRARIES})

#PCL_ADD_EXECUTABLE(people_pcd_folder COMPONENT ${SUBSYS_NAME} SOURCES people_pcd_folder.cpp)
#target_link_libraries (people_pcd_folder pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization ${Boost_LIBRARIES})

#PCL_ADD_EXECUTABLE(people_pcd_person COMPONENT ${SUBSYS_NAME} SOURCES people_pcd_person.cpp)
#target_link_libraries (people_pcd_person pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization)

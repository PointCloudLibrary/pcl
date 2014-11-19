if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
  include("${VTK_USE_FILE}")
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT VTK_FOUND)

set(the_target people_tracking)

include_directories(${VTK_INCLUDE_DIRS})

#PCL_ADD_EXECUTABLE("${the_target}" "${SUBSYS_NAME}" people_tracking.cpp)
#target_link_libraries("${the_target}" pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization)

if (HAVE_OPENNI)
  PCL_ADD_EXECUTABLE(pcl_people_app "${SUBSYS_NAME}" people_app.cpp)
  target_link_libraries (pcl_people_app pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization ${Boost_LIBRARIES})
endif (HAVE_OPENNI)

PCL_ADD_EXECUTABLE(pcl_people_pcd_prob "${SUBSYS_NAME}" people_pcd_prob.cpp)
target_link_libraries (pcl_people_pcd_prob pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization ${Boost_LIBRARIES})

#PCL_ADD_EXECUTABLE(people_pcd_folder "${SUBSYS_NAME}" people_pcd_folder.cpp)
#target_link_libraries (people_pcd_folder pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization ${Boost_LIBRARIES})

#PCL_ADD_EXECUTABLE(people_pcd_person "${SUBSYS_NAME}" people_pcd_person.cpp)
#target_link_libraries (people_pcd_person pcl_common pcl_filters pcl_kdtree pcl_segmentation pcl_kdtree pcl_gpu_people pcl_filters pcl_io pcl_visualization)

#if(VTK_FOUND)
#  if(QT4_FOUND AND VTK_USE_QVTK)
#    QT4_WRAP_UI(people_proc_ui people_proc/people_proc.ui)
#    QT4_WRAP_CPP(people_proc_moc people_proc/people_proc.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
#    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_people_proc "${SUBSYS_NAME}" ${people_proc_ui} ${people_proc_moc} people_proc/people_proc.cpp)
#    target_link_libraries(pcl_people_proc pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface pcl_gpu_people QVTK ${QT_LIBRARIES})
#
#  endif()
#endif()

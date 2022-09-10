if(NOT (BUILD_surface_on_nurbs AND BUILD_visualization))
  return()
endif()

PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS visualization geometry surface)

## Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
endif()

PCL_ADD_EXAMPLE(pcl_test_nurbs_fitting_surface
                FILES test_nurbs_fitting_surface.cpp
                LINK_WITH pcl_common pcl_io pcl_surface pcl_visualization)

PCL_ADD_EXAMPLE(pcl_example_nurbs_fitting_surface
                FILES example_nurbs_fitting_surface.cpp
                LINK_WITH pcl_common pcl_io pcl_surface pcl_visualization)

PCL_ADD_EXAMPLE(pcl_example_nurbs_viewer_surface
                FILES example_nurbs_viewer_surface.cpp
                LINK_WITH pcl_common pcl_io pcl_surface pcl_visualization)

PCL_ADD_EXAMPLE(pcl_example_nurbs_fitting_closed_curve
                FILES example_nurbs_fitting_closed_curve.cpp
                LINK_WITH pcl_common pcl_io pcl_surface pcl_visualization)

PCL_ADD_EXAMPLE(pcl_example_nurbs_fitting_curve2d
                FILES example_nurbs_fitting_curve2d.cpp
                LINK_WITH pcl_common pcl_io pcl_surface pcl_visualization)

PCL_ADD_EXAMPLE(pcl_example_nurbs_fitting_closed_curve3d
                FILES example_nurbs_fitting_closed_curve3d.cpp
                LINK_WITH pcl_common pcl_io pcl_surface pcl_visualization)

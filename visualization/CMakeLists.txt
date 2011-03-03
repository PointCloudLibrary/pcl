set(LIB_NAME visualization)
set(LIB_DESC "Point cloud visualization classes")
set(LIB_DEPS pcl)

set(build TRUE)
PCL_LIB_OPTION(build LIB ${LIB_NAME} ON ${LIB_DEPS})

if(build)
    # Find Boost
    # find_package(wxWidgets REQUIRED)
    # include(${wxWidgets_USE_FILE})
    # include_directories(${wxWidgets_INCLUDE_DIRS})
    # find_package (VTK REQUIRED)
    # include (${VTK_USE_FILE})

    set(viz_srcs src/libpcl_visualization/interactor.cpp
        src/libpcl_visualization/interactor_style.cpp
        src/libpcl_visualization/point_cloud_handlers.cpp
        src/libpcl_visualization/pcl_visualizer.cpp
        src/libpcl_visualization/histogram_visualizer.cpp
        src/libpcl_visualization/common/common.cpp
        src/libpcl_visualization/common/io.cpp
        src/libpcl_visualization/common/shapes.cpp
        src/libpcl_visualization/common/float_image_utils.cpp
        src/libpcl_visualization/common/image_widget_wx.cpp
        src/libpcl_visualization/range_image_visualizer.cpp
        src/libpcl_visualization/cloud_viewer.cpp
        )

    # link to vtkCommon, vtkWIdgets, vtkHybrid, ${wxWidgets_LIBRARIES}

    # executable: test_pcl_vis src/test.cpp
    # executable: test_geometry src/test_geometry.cpp
    # executable: test_shapes src/test_shapes.cpp
    # share data: http://pr.willowgarage.com/data/pcl/partial_cup_model_new.pcd
    # subdir: src/examples
    # subdir: src/tools
endif(build)


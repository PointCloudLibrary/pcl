set(LIB_NAME visualization)
set(LIB_DESC "Point cloud visualization classes")
set(LIB_DEPS pcl)

set(build TRUE)
PCL_LIB_OPTION(build LIB ${LIB_NAME} ON ${LIB_DEPS})

if(build)
    # Find wxWidgets
    find_package(wxWidgets REQUIRED)
    include(${wxWidgets_USE_FILE})
    include_directories(${wxWidgets_INCLUDE_DIRS})

    # Find VTK
    find_package (VTK REQUIRED)
    include (${VTK_USE_FILE})

    set(VIS_COMPONENT visualization)

    set(viz_srcs src/interactor.cpp
        src/interactor_style.cpp
        src/point_cloud_handlers.cpp
        src/pcl_visualizer.cpp
        src/histogram_visualizer.cpp
        src/common/common.cpp
        src/common/io.cpp
        src/common/shapes.cpp
        src/common/float_image_utils.cpp
        src/common/image_widget_wx.cpp
        src/range_image_visualizer.cpp
        src/cloud_viewer.cpp
        )

    PCL_ADD_LIBRARY(pcl_visualization ${viz_srcs} ${VIS_COMPONENT})
    target_link_libraries(pcl_visualization ${wxWidgets_LIBRARIES}
        vtkCommon vtkWidgets vtkHybrid)
    # Link boost
    PCL_MAKE_PKGCONFIG(pcl_visualization visualization
        "Point cloud visualisation library" "" "" "" "")

    add_subdirectory(examples)
    add_subdirectory(test)
    add_subdirectory(tools)

    # share data: http://pr.willowgarage.com/data/pcl/partial_cup_model_new.pcd
endif(build)


set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud visualization library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    # Find wxWidgets
    find_package(wxWidgets REQUIRED)
    include(${wxWidgets_USE_FILE})
    include_directories(${wxWidgets_INCLUDE_DIRS})

    # Find VTK
    find_package (VTK REQUIRED)
    include (${VTK_USE_FILE})

    set(srcs src/interactor.cpp
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

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/common/include)
    PCL_ADD_LIBRARY(pcl_visualization ${SUBSYS_NAME} ${srcs})
    target_link_libraries(pcl_visualization ${wxWidgets_LIBRARIES}
        vtkCommon vtkWidgets vtkHybrid ${Boost_LIBRARIES})
    PCL_MAKE_PKGCONFIG(pcl_visualization ${SUBSYS_NAME} "${SUBSYS_DESC}" "" ""
        "" "")

    # Install include files
    set(incs include/visualization/cloud_viewer.h
        include/visualization/histogram_visualizer.h
        include/visualization/interactor.h
        include/visualization/interactor_style.h
        include/visualization/pcl_visualizer.h
        include/visualization/point_cloud_handlers.h
        include/visualization/range_image_visualizer.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} visualization ${incs})
    set(common_incs include/visualization/common/actor_map.h
        include/visualization/common/common.h
        include/visualization/common/float_image_utils.h
        include/visualization/common/image_widget_wx.h
        include/visualization/common/io.h
        include/visualization/common/ren_win_interact_map.h
        include/visualization/common/shapes.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} visualization/common ${common_incs})

    add_subdirectory(examples)
    add_subdirectory(test)
    add_subdirectory(tools)

    # share data: http://pr.willowgarage.com/data/pcl/partial_cup_model_new.pcd
endif(build)


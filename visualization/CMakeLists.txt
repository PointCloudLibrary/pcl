set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud visualization library")
set(SUBSYS_DEPS common features filters io kdtree range_image)

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

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    target_link_libraries(${LIB_NAME} ${wxWidgets_LIBRARIES}
        vtkCommon vtkWidgets vtkHybrid ${Boost_LIBRARIES})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${SUBSYS_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/cloud_viewer.h
        include/pcl/${SUBSYS_NAME}/histogram_visualizer.h
        include/pcl/${SUBSYS_NAME}/interactor.h
        include/pcl/${SUBSYS_NAME}/interactor_style.h
        include/pcl/${SUBSYS_NAME}/pcl_visualizer.h
        include/pcl/${SUBSYS_NAME}/point_cloud_handlers.h
        include/pcl/${SUBSYS_NAME}/range_image_visualizer.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(common_incs include/pcl/${SUBSYS_NAME}/common/actor_map.h
        include/pcl/${SUBSYS_NAME}/common/common.h
        include/pcl/${SUBSYS_NAME}/common/float_image_utils.h
        include/pcl/${SUBSYS_NAME}/common/image_widget_wx.h
        include/pcl/${SUBSYS_NAME}/common/io.h
        include/pcl/${SUBSYS_NAME}/common/ren_win_interact_map.h
        include/pcl/${SUBSYS_NAME}/common/shapes.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/common ${common_incs})

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/cloud_viewer.hpp
        include/pcl/${SUBSYS_NAME}/impl/histogram_visualizer.hpp
        include/pcl/${SUBSYS_NAME}/impl/pcl_visualizer.hpp
        include/pcl/${SUBSYS_NAME}/impl/point_cloud_handlers.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    set(commom_impl_incs include/pcl/${SUBSYS_NAME}/common/impl/shapes.hpp)
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/common/impl
        ${common_impl_incs})

    #add_subdirectory(examples)
    add_subdirectory(test)
    add_subdirectory(tools)

    # share data: http://pr.willowgarage.com/data/pcl/partial_cup_model_new.pcd
endif(build)


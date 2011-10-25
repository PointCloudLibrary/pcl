set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud visualization library")
set(SUBSYS_DEPS common io kdtree range_image)

if(wxWidgets_FOUND)
  include(${wxWidgets_USE_FILE})
  include_directories(${wxWidgets_INCLUDE_DIRS})
  set(wxWidgets_USE_FILE ${wxWidgets_USE_FILE} CACHE INTERNAL "wxWidgets_USE_FILE")
else(wxWidgets_FOUND)
  set(DEFAULT TRUE)   # wxWidgets is optional
  set(REASON "wxWidgets was not found.")
endif(wxWidgets_FOUND)

if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif(NOT VTK_FOUND)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS vtk OPT_DEPS wxwidgets openni)

if(build)
  if(wxWidgets_FOUND)
    set(WXWIDGETS_INCLUDES
        include/pcl/${SUBSYS_NAME}/range_image_visualizer.h
       )
    set(WXWIDGETS_COMMON_INCLUDES
        include/pcl/${SUBSYS_NAME}/common/float_image_utils.h
        include/pcl/${SUBSYS_NAME}/common/image_widget_wx.h
        )
    set(WXWIDGETS_SOURCES
        src/common/image_widget_wx.cpp
        src/range_image_visualizer.cpp
        src/common/float_image_utils.cpp
       )
  endif()

    set(srcs
        src/point_picking_event.cpp
        src/interactor_style.cpp
        src/point_cloud_handlers.cpp
        src/pcl_visualizer.cpp
        src/histogram_visualizer.cpp
        src/common/common.cpp
        src/common/io.cpp
        src/common/shapes.cpp
        src/cloud_viewer.cpp
        src/image_viewer.cpp
        src/window.cpp
        ${WXWIDGETS_SOURCES}
        )
    if("${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}" VERSION_LESS "5.6")
        # A custom interactor is necessary on VTK 5.4 and below
        set(srcs ${srcs} src/interactor.cpp)
    endif("${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}" VERSION_LESS "5.6")

    set(incs 
        include/pcl/${SUBSYS_NAME}/cloud_viewer.h
        include/pcl/${SUBSYS_NAME}/histogram_visualizer.h
        include/pcl/${SUBSYS_NAME}/image_viewer.h
        include/pcl/${SUBSYS_NAME}/interactor_style.h
        include/pcl/${SUBSYS_NAME}/pcl_visualizer.h
        include/pcl/${SUBSYS_NAME}/registration_visualizer.h
        include/pcl/${SUBSYS_NAME}/point_cloud_handlers.h
        include/pcl/${SUBSYS_NAME}/keyboard_event.h
        include/pcl/${SUBSYS_NAME}/point_picking_event.h
        include/pcl/${SUBSYS_NAME}/mouse_event.h
        include/pcl/${SUBSYS_NAME}/window.h
        ${WXWIDGETS_INCLUDES}
        )

    set(common_incs 
        include/pcl/${SUBSYS_NAME}/common/actor_map.h
        include/pcl/${SUBSYS_NAME}/common/common.h
        include/pcl/${SUBSYS_NAME}/common/io.h
        include/pcl/${SUBSYS_NAME}/common/ren_win_interact_map.h
        include/pcl/${SUBSYS_NAME}/common/shapes.h
        ${WXWIDGETS_COMMON_INCLUDES}
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/histogram_visualizer.hpp
        include/pcl/${SUBSYS_NAME}/impl/pcl_visualizer.hpp
        include/pcl/${SUBSYS_NAME}/impl/registration_visualizer.hpp
        include/pcl/${SUBSYS_NAME}/impl/point_cloud_handlers.hpp
        )

    set(common_impl_incs include/pcl/${SUBSYS_NAME}/common/impl/shapes.hpp)

    set(LIB_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${common_incs} ${impl_incs} ${common_impl_incs})
     target_link_libraries(${LIB_NAME} pcl_io pcl_kdtree pcl_range_image 
                                       ${wxWidgets_LIBRARIES} 
                                       vtkCommon vtkWidgets vtkHybrid)
    set(EXT_DEPS "")
    if(OPENNI_FOUND)
      list(APPEND EXT_DEPS openni-dev)
    endif(OPENNI_FOUND)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/common ${common_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/common/impl
        ${common_impl_incs})

    #add_subdirectory(examples)
    add_subdirectory(test)
    add_subdirectory(tools)

    # share data: http://pr.willowgarage.com/data/pcl/partial_cup_model_new.pcd
endif(build)

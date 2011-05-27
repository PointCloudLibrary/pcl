set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud visualization library")
set(SUBSYS_DEPS common io kdtree octree range_image octree filters)

if(wxWidgets_FOUND)
  set(HAVE_WXWIDGETS ON)
  include(${wxWidgets_USE_FILE})
  include_directories(${wxWidgets_INCLUDE_DIRS})
endif()

if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
elseif(NOT wxWidgets_FOUND)
    set(DEFAULT TRUE)   # wxWidgets is optional
    set(REASON "wxWidgets was not found.")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
		set(VTK_INCLUDE_DIRS ${VTK_INCLUDE_DIRS} CACHE INTERNAL "VTK include dirs")
    message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif(NOT VTK_FOUND)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

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

    set(srcs src/interactor.cpp
        src/interactor_style.cpp
        src/point_cloud_handlers.cpp
        src/pcl_visualizer.cpp
        src/histogram_visualizer.cpp
        src/common/common.cpp
        src/common/io.cpp
        src/common/shapes.cpp
        src/cloud_viewer.cpp
        ${WXWIDGETS_SOURCES}
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/cloud_viewer.h
        include/pcl/${SUBSYS_NAME}/histogram_visualizer.h
        include/pcl/${SUBSYS_NAME}/interactor.h
        include/pcl/${SUBSYS_NAME}/interactor_style.h
        include/pcl/${SUBSYS_NAME}/pcl_visualizer.h
        include/pcl/${SUBSYS_NAME}/point_cloud_handlers.h
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
        include/pcl/${SUBSYS_NAME}/impl/point_cloud_handlers.hpp
        )

    set(common_impl_incs include/pcl/${SUBSYS_NAME}/common/impl/shapes.hpp)

    set(LIB_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${common_incs} ${impl_incs} ${common_impl_incs})
#    if(MSVC)
#       set(VTK_COMMON_LIBRARIES optimized vtkCommon debug vtkCommon-gd)
#       set(VTK_WIDGETS_LIBRARIES optimized vtkWidgets debug vtkWidgets-gd)
#       set(VTK_HYBRID_LIBRARIES optimized vtkHybrid debug vtkHybrid-gd)
#       target_link_libraries(${LIB_NAME} pcl_io pcl_kdtree pcl_range_image 
#                                        ${wxWidgets_LIBRARIES} 
#                                        ${VTK_COMMON_LIBRARIES} 
#                                        ${VTK_WIDGETS_LIBRARIES}
#                                        ${VTK_HYBRID_LIBRARIES})
#    else(MSVC)
       target_link_libraries(${LIB_NAME} pcl_io pcl_kdtree pcl_range_image 
                                        ${wxWidgets_LIBRARIES} 
                                        vtkCommon vtkWidgets vtkHybrid)
#    endif(MSVC)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    if(OPENNI_FOUND)
      list(APPEND EXT_DEPS openni-dev)
    endif(OPENNI_FOUND)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${EXT_DEPS}" "" "" "")

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

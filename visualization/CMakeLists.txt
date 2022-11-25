set(SUBSYS_NAME visualization)
set(SUBSYS_DESC "Point cloud visualization library")
set(SUBSYS_DEPS common io kdtree geometry search octree)

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS vtk OPT_DEPS openni openni2 ensenso davidSDK dssdk rssdk)

if(ANDROID)
  set(build FALSE)
  message("VTK was found, but cannot be compiled for Android. Please use VES instead.")
endif()

if(OPENGL_FOUND)
  if(OPENGL_INCLUDE_DIR)
    include_directories(SYSTEM "${OPENGL_INCLUDE_DIR}")
  endif()
  if(OPENGL_DEFINITIONS)
    add_definitions("${OPENGL_DEFINITIONS}")
  endif()
endif()

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
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
  src/common/ren_win_interact_map.cpp
  src/cloud_viewer.cpp
  src/image_viewer.cpp
  src/window.cpp
  src/range_image_visualizer.cpp
  src/common/float_image_utils.cpp
  src/vtk/pcl_image_canvas_source_2d.cpp
  src/vtk/pcl_context_item.cpp
  src/vtk/vtkRenderWindowInteractorFix.cpp
  src/pcl_painter2D.cpp
  src/pcl_plotter.cpp
)

if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0 AND UNIX AND NOT ANDROID AND NOT APPLE AND NOT APPLE_IOS)
  list(APPEND srcs "src/vtk/vtkFixedXRenderWindowInteractor.cxx")
endif()

if(NOT (${VTK_VERSION} VERSION_LESS 9.0))
  if(NOT (";${VTK_AVAILABLE_COMPONENTS};" MATCHES ";RenderingContextOpenGL2;"))
    list(REMOVE_ITEM srcs
      src/pcl_painter2D.cpp
    )
  endif()  
endif()

set(incs
  "include/pcl/${SUBSYS_NAME}/eigen.h"
  "include/pcl/${SUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/cloud_viewer.h"
  "include/pcl/${SUBSYS_NAME}/histogram_visualizer.h"
  "include/pcl/${SUBSYS_NAME}/image_viewer.h"
  "include/pcl/${SUBSYS_NAME}/interactor_style.h"
  "include/pcl/${SUBSYS_NAME}/pcl_visualizer.h"
  "include/pcl/${SUBSYS_NAME}/pcl_painter2D.h"
  "include/pcl/${SUBSYS_NAME}/registration_visualizer.h"
  "include/pcl/${SUBSYS_NAME}/point_cloud_handlers.h"
  "include/pcl/${SUBSYS_NAME}/point_cloud_color_handlers.h"
  "include/pcl/${SUBSYS_NAME}/point_cloud_geometry_handlers.h"
  "include/pcl/${SUBSYS_NAME}/keyboard_event.h"
  "include/pcl/${SUBSYS_NAME}/point_picking_event.h"
  "include/pcl/${SUBSYS_NAME}/area_picking_event.h"
  "include/pcl/${SUBSYS_NAME}/mouse_event.h"
  "include/pcl/${SUBSYS_NAME}/window.h"
  "include/pcl/${SUBSYS_NAME}/range_image_visualizer.h"
  "include/pcl/${SUBSYS_NAME}/vtk.h"
  "include/pcl/${SUBSYS_NAME}/simple_buffer_visualizer.h"
  "include/pcl/${SUBSYS_NAME}/pcl_plotter.h"
  "include/pcl/${SUBSYS_NAME}/qvtk_compatibility.h"
)

set(common_incs
  "include/pcl/${SUBSYS_NAME}/common/actor_map.h"
  "include/pcl/${SUBSYS_NAME}/common/common.h"
  "include/pcl/${SUBSYS_NAME}/common/io.h"
  "include/pcl/${SUBSYS_NAME}/common/ren_win_interact_map.h"
  "include/pcl/${SUBSYS_NAME}/common/shapes.h"
  "include/pcl/${SUBSYS_NAME}/common/float_image_utils.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/histogram_visualizer.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/pcl_visualizer.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/image_viewer.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/registration_visualizer.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/point_cloud_handlers.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/point_cloud_color_handlers.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/point_cloud_geometry_handlers.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/pcl_plotter.hpp"
)

set(common_impl_incs
  "include/pcl/${SUBSYS_NAME}/common/impl/shapes.hpp"
  "include/pcl/${SUBSYS_NAME}/common/impl/common.hpp"
)

set(vtk_incs
  "include/pcl/${SUBSYS_NAME}/vtk/pcl_image_canvas_source_2d.h"
  "include/pcl/${SUBSYS_NAME}/vtk/pcl_context_item.h"
  "include/pcl/${SUBSYS_NAME}/vtk/vtkRenderWindowInteractorFix.h"
  "include/pcl/${SUBSYS_NAME}/vtk/pcl_vtk_compatibility.h"
)

if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0 AND UNIX AND NOT ANDROID AND NOT APPLE AND NOT APPLE_IOS)
  list(APPEND vtk_incs "include/pcl/${SUBSYS_NAME}/vtk/vtkFixedXRenderWindowInteractor.h")
endif()

if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0)
  if(NOT (";${VTK_AVAILABLE_COMPONENTS};" MATCHES ";RenderingContextOpenGL2;"))
    
    list(REMOVE_ITEM incs
      "include/pcl/${SUBSYS_NAME}/pcl_painter2D.h"
    )
  endif()
endif()

# on apple, a workaround is used for the cocoa render window interactor
if(APPLE)
  list(APPEND srcs
  include/pcl/visualization/vtk/vtkRenderWindowInteractorFix.mm)
endif()

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${common_incs} ${impl_incs} ${common_impl_incs} ${vtk_incs})

# apple workaround (continued)
if(APPLE)
  target_link_libraries("${LIB_NAME}" "-framework Cocoa")
endif()

target_link_libraries("${LIB_NAME}" pcl_common pcl_io pcl_kdtree ${OPENGL_LIBRARIES})

if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0)
  #Some libs are referenced through depending on IO
  target_link_libraries("${LIB_NAME}"
                        VTK::ChartsCore
                        VTK::CommonColor
                        VTK::CommonComputationalGeometry
                        VTK::CommonDataModel
                        VTK::FiltersExtraction
                        VTK::FiltersGeometry
                        VTK::FiltersGeneral
                        VTK::FiltersModeling
                        VTK::FiltersSources
                        VTK::IOImage
                        VTK::IOPLY
                        VTK::ImagingSources
                        VTK::InteractionImage
                        VTK::InteractionStyle
                        VTK::RenderingAnnotation
                        VTK::RenderingContext2D
                        VTK::RenderingFreeType
                        VTK::RenderingLOD
                        VTK::RenderingOpenGL2
                        VTK::ViewsContext2D)
                        
  if("RenderingContextOpenGL2" IN_LIST VTK_AVAILABLE_COMPONENTS)
    target_link_libraries("${LIB_NAME}" VTK::RenderingContextOpenGL2)
  endif()
else()
  target_include_directories("${LIB_NAME}" SYSTEM PUBLIC ${VTK_INCLUDE_DIRS})
  target_link_libraries("${LIB_NAME}"
                        vtkChartsCore
                        vtkFiltersExtraction
                        vtkFiltersGeometry
                        vtkFiltersModeling
                        vtkImagingCore
                        vtkImagingSources
                        vtkInteractionStyle
                        vtkRenderingAnnotation
                        vtkRenderingContext2D
                        vtkRenderingContext${VTK_RENDERING_BACKEND}
                        vtkRenderingFreeType
                        vtkRenderingLOD
                        vtkRendering${VTK_RENDERING_BACKEND}
                        vtkViewsContext2D)
  if(${VTK_VERSION} VERSION_LESS 7.0)
    target_link_libraries("${LIB_NAME}" vtkRenderingVolume${VTK_RENDERING_BACKEND})  	
  endif()
  if(${VTK_VERSION} VERSION_GREATER_EQUAL 7.0 AND ${VTK_VERSION} VERSION_LESS 9.0)
    target_link_libraries("${LIB_NAME}" vtkRenderingGL2PS${VTK_RENDERING_BACKEND})
  endif()
  if(${VTK_VERSION} VERSION_GREATER_EQUAL 8.0)
    target_link_libraries("${LIB_NAME}" vtkCommonColor vtkCommonComputationalGeometry)
  endif()
  
  # These two libraries are present and required in ubuntu 18.04 VTK 6, but not present in later versions of ubuntu using VTK 6.
  # They are also present on later versions of VTK on later versions of ubuntu.
  if("vtkRenderingFreeTypeFontConfig" IN_LIST VTK_MODULES_ENABLED)
    target_link_libraries("${LIB_NAME}" vtkRenderingFreeTypeFontConfig)
  endif()
  if("vtkRenderingMatplotlib" IN_LIST VTK_MODULES_ENABLED)
    target_link_libraries("${LIB_NAME}" vtkRenderingMatplotlib)
  endif()
endif()

if(HAVE_QVTK)
  if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0)
    target_link_libraries("${LIB_NAME}" VTK::GUISupportQt)
  else()
    target_link_libraries("${LIB_NAME}" vtkGUISupportQt)
  endif()
endif()

set(EXT_DEPS "")
if(WITH_OPENNI)
  list(APPEND EXT_DEPS libopenni)
endif()
if(WITH_OPENNI2)
  list(APPEND EXT_DEPS libopenni2)
endif()
if(WITH_ENSENSO)
  list(APPEND EXT_DEPS ensenso)
endif()
if(WITH_DAVIDSDK)
  list(APPEND EXT_DEPS davidSDK)
endif()
if(WITH_DSSDK)
  list(APPEND EXT_DEPS dssdk)
endif()
if(WITH_RSSDK)
  list(APPEND EXT_DEPS rssdk)
endif()

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/common" ${common_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/common/impl" ${common_impl_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/vtk" ${vtk_incs})

#TODO: Update when CMAKE 3.10 is available
if(NOT (${VTK_VERSION} VERSION_LESS 9.0))
  vtk_module_autoinit(TARGETS "${LIB_NAME}" 
                      MODULES VTK::RenderingContextOpenGL2
                              VTK::RenderingOpenGL2
                              VTK::RenderingFreeType)
endif()

if(BUILD_TESTS)
  add_subdirectory(test)
endif()

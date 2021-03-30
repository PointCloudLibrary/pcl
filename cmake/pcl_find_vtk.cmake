function(checkVTKComponents)
  cmake_parse_arguments(PARAM "" "MISSING_COMPONENTS" "COMPONENTS" ${ARGN})

  set(vtkMissingComponents)
  
  foreach(vtkComponent ${PARAM_COMPONENTS})
    if (VTK_VERSION VERSION_LESS 9.0)
      if (NOT TARGET ${vtkComponent})
        list(APPEND vtkMissingComponents ${vtkComponent})
      endif()
    else()
      if (NOT TARGET VTK::${vtkComponent})
        list(APPEND vtkMissingComponents ${vtkComponent})
      endif()
    endif()
  endforeach()
  
  set(${PARAM_MISSING_COMPONENTS} ${vtkMissingComponents} PARENT_SCOPE)
endfunction()

# Start with a generic call to find any VTK version we are supporting, so we retrieve
# the version of VTK. As the module names were changed from VTK 8.2 to 9.0, we don't
# search explicitly for modules. Furthermore we don't pass required minimum version 6.2
# to find_package because then it only accept versions with same major version.
find_package(VTK)

if(NOT VTK_FOUND)
    return()
endif()

if(VTK_FOUND AND (VTK_VERSION VERSION_LESS 6.2))
  message(WARNING "The minimum required version of VTK is 6.2, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
  return()
endif()

set(NON_PREFIX_PCL_VTK_COMPONENTS
  ChartsCore
  CommonColor
  CommonCore
  CommonDataModel
  CommonExecutionModel
  CommonMath
  CommonMisc
  CommonTransforms
  FiltersCore
  FiltersExtraction
  FiltersGeneral
  FiltersGeometry
  FiltersModeling
  FiltersSources
  ImagingCore
  ImagingSources
  InteractionStyle
  InteractionWidgets
  IOCore
  IOGeometry
  IOImage
  IOLegacy
  IOPLY
  RenderingAnnotation
  RenderingCore
  RenderingContext2D
  RenderingLOD
  RenderingFreeType
  ViewsCore
  ViewsContext2D
)

#If VTK version 6 use OpenGL
if(VTK_VERSION VERSION_LESS 7.0)
  set(VTK_RENDERING_BACKEND "OpenGL")
  set(VTK_RENDERING_BACKEND_OPENGL_VERSION "1")
  message(DEPRECATION "The rendering backend OpenGL is deprecated and not available anymore since VTK 8.2."
					  "Please switch to the OpenGL2 backend instead, which is available since VTK 6.2."
					  "Support of the deprecated backend will be dropped with PCL 1.13.")

#If VTK version 7,8 or 9 use OpenGL2
else()
  set(VTK_RENDERING_BACKEND "OpenGL2")
  set(VTK_RENDERING_BACKEND_OPENGL_VERSION "2")
endif()

list(APPEND NON_PREFIX_PCL_VTK_COMPONENTS Rendering${VTK_RENDERING_BACKEND})

#Append vtk to components if version is <9.0
if(VTK_VERSION VERSION_LESS 9.0)
  foreach(vtkComponent ${NON_PREFIX_PCL_VTK_COMPONENTS})
    set(vtkComponent "vtk${vtkComponent}")
    list(APPEND PCL_VTK_COMPONENTS ${vtkComponent})
  endforeach()
else()
  set(PCL_VTK_COMPONENTS ${NON_PREFIX_PCL_VTK_COMPONENTS})
endif()

# Check if requested modules are available
checkVTKComponents(COMPONENTS ${PCL_VTK_COMPONENTS} MISSING_COMPONENTS vtkMissingComponents)

if (vtkMissingComponents)
  set(VTK_FOUND FALSE)
  message(WARNING "Missing vtk modules: ${vtkMissingComponents}")
endif()

if(WITH_QT)
  if(VTK_VERSION VERSION_LESS 9.0)
    if(";${VTK_MODULES_ENABLED};" MATCHES ";vtkGUISupportQt;" AND ";${VTK_MODULES_ENABLED};" MATCHES ";vtkRenderingQt;")
      set(HAVE_QVTK TRUE)
      #PCL_VTK_COMPONENTS is used in the PCLConfig.cmake to refind the required modules.
      #Pre vtk 9.0, all vtk libraries are linked into pcl_visualizer.
      #Subprojects can link against pcl_visualizer and directly use VTK-QT libraries.
      list(APPEND PCL_VTK_COMPONENTS vtkRenderingQt vtkGUISupportQt)
    else()
      unset(HAVE_QVTK)
    endif()
  else()
	if(";${VTK_AVAILABLE_COMPONENTS};" MATCHES ";GUISupportQt;" AND ";${VTK_AVAILABLE_COMPONENTS};" MATCHES ";RenderingQt;")
      set(HAVE_QVTK TRUE)
      #PCL_VTK_COMPONENTS is used in the PCLConfig.cmake to refind the required modules.
      #Post vtk 9.0, only required libraries are linked against pcl_visualizer.
      #Subprojects need to manually link to VTK-QT libraries.
      list(APPEND PCL_VTK_COMPONENTS RenderingQt GUISupportQt)
    else()
      unset(HAVE_QVTK)
    endif()
  endif()
endif()

if(PCL_SHARED_LIBS OR (NOT (PCL_SHARED_LIBS) AND NOT (VTK_BUILD_SHARED_LIBS)))
  if(VTK_VERSION VERSION_LESS 9.0)
    if(VTK_USE_FILE)
      include(${VTK_USE_FILE})
    endif()
  endif()
  
  if(APPLE)
    option(VTK_USE_COCOA "Use Cocoa for VTK render windows" ON)
    mark_as_advanced(VTK_USE_COCOA)
  endif()
else()
  set(VTK_FOUND OFF)
  message("Warning: You are to build PCL in STATIC but VTK is SHARED!")
  message("Warning: VTK disabled!")
endif()

message(STATUS "VTK version: ${VTK_VERSION}")
message(STATUS "VTK rendering backend: ${VTK_RENDERING_BACKEND}")

if(WITH_QT)
  if(HAVE_QVTK)
    message(STATUS "VTK Qt support: YES")
  else()
    message(STATUS "VTK Qt support: NOTFOUND")
  endif()
else()
  message(STATUS "VTK Qt support: NO")
endif()

if(VTK_INCLUDE_DIRS)
  message(STATUS "VTK include: ${VTK_INCLUDE_DIRS}")
ENDIF()

if(VTK_LIBRARIES)
  message(STATUS "VTK libs: ${VTK_LIBRARIES}")
endif()

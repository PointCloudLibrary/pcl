#Disable some warnings which are all over Qt headers and make finding error messages hell
#if(${CMAKE_CXX_FLAGS} MATCHES "-Wconversion") 
#  string(REGEX REPLACE "-Wconversion(.+)" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
#endif()

set(SUBSYS_NAME cloud_composer)
set(SUBSYS_DESC "Cloud Composer - Application for Manipulating Point Clouds")
set(SUBSYS_DEPS common io visualization)

# Find VTK 
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
  include (${VTK_USE_FILE})
endif(NOT VTK_FOUND)

# QT4 Found?
if(NOT QT4_FOUND)
  set(DEFAULT FALSE)
  set(REASON "Qt4 was not found.")
else(NOT QT4_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
endif(NOT QT4_FOUND)

# QVTK?
if(NOT VTK_USE_QVTK)
  set(DEFAULT FALSE)
  set(REASON "Cloud composer requires QVTK")
else(NOT VTK_USE_QVTK)
  set(DEFAULT TRUE)
  set(REASON)
endif(NOT VTK_USE_QVTK)

#Default to not building for now
set(DEFAULT FALSE)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)

  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  
  if (VTK_FOUND AND QT4_FOUND AND VTK_USE_QVTK)
    #Qt Sources, Headers, and Resources set
    set(incs include/pcl/apps/${SUBSYS_NAME}/cloud_composer.h
             include/pcl/apps/${SUBSYS_NAME}/project_model.h
             include/pcl/apps/${SUBSYS_NAME}/cloud_viewer.h
             include/pcl/apps/${SUBSYS_NAME}/cloud_view.h
             include/pcl/apps/${SUBSYS_NAME}/item_inspector.h)
    set(srcs src/main.cpp 
             src/cloud_composer.cpp
             src/project_model.cpp
             src/cloud_viewer.cpp
             src/cloud_view.cpp
             src/item_inspector.cpp)
    set(uis src/cloud_composer_main_window.ui)
    set(resources resources/resources.qrc)

    QT4_WRAP_UI(cloud_composer_ui ${uis})
    QT4_WRAP_CPP(cloud_composer_moc ${incs})
    QT4_ADD_RESOURCES(resource_srcs ${resources})
    
    set(EXE_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_EXECUTABLE(${EXE_NAME} ${SUBSYS_NAME} ${cloud_composer_ui} ${cloud_composer_moc} ${srcs} ${resource_srcs})
    target_link_libraries(${EXE_NAME} pcl_common pcl_io pcl_visualization QVTK ${QT_LIBRARIES})

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    
    PCL_MAKE_PKGCONFIG(${EXE_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")
    
  endif ()  #VTK_FOUND AND QT4_FOUND AND VTK_USE_QVTK


endif(build)

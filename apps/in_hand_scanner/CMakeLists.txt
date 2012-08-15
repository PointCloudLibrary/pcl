set(SUBSYS_NAME in_hand_scanner)
set(SUBSYS_DESC "Kinect in-hand scanner for small objects")
set(SUBSYS_DEPS     common     geometry     io     kdtree     registration     visualization)
set(SUBSYS_LIBS pcl_common pcl_geometry pcl_io pcl_kdtree pcl_registration pcl_visualization)

################################################################################

# Qt files
set(UI_FILES main_window.ui)
set(MOC_FILES include/pcl/apps/${SUBSYS_NAME}/main_window.h)
set(RESOURCE_FILES "")

# Header files
set(INCS ${MOC_FILES}
         include/pcl/apps/${SUBSYS_NAME}/in_hand_scanner.h
   )

set(IMPL_INCS "")

# Source files
set(SRCS src/main.cpp
         src/in_hand_scanner.cpp
         src/main_window.cpp
   )

################################################################################

set(EXTERNAL_LIBS_FOUND TRUE)
set(REASON "")
set(DEFAULT TRUE)

# OpenNI
if(NOT OPENNI_FOUND OR NOT BUILD_OPENNI)
  set(DEFAULT FALSE)
  set(REASON ${REASON} "OpenNI was not found or was disabled by the user. ")
  set(EXTERNAL_LIBS_FOUND FALSE)
endif()

# VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON ${REASON} "VTK was not found. ")
  set(EXTERNAL_LIBS_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

# QT4
if(NOT QT4_FOUND)
  set(DEFAULT FALSE)
  set(REASON ${REASON} "Qt4 was not found. ")
  set(EXTERNAL_LIBS_FOUND FALSE)
else()
  set(SUBSYS_LIBS ${SUBSYS_LIBS} ${QT_LIBRARIES})
endif()

# QVTK
if(NOT VTK_USE_QVTK)
  set(DEFAULT FALSE)
  set(REASON ${REASON} "QVTK was not found. ")
  set(EXTERNAL_LIBS_FOUND FALSE)
else()
  set(SUBSYS_LIBS ${SUBSYS_LIBS} QVTK)
endif()

# Default to not building for now
set(DEFAULT FALSE)

# PCL
pcl_subsys_option(BUILD ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
pcl_subsys_depend(BUILD ${SUBSYS_NAME} ${SUBSYS_DEPS})
pcl_add_doc(${SUBSYS_NAME})

################################################################################

if(BUILD AND EXTERNAL_LIBS_FOUND)

  # Process the Qt files
  qt4_wrap_ui(QT_UI_FILES ${UI_FILES})
  qt4_wrap_cpp(QT_MOC_FILES ${MOC_FILES})
  qt4_add_resources(QT_RESOURCE_FILES ${RESOURCE_FILES})

  set(SRCS ${SRCS} ${QT_UI_FILES} ${QT_MOC_FILES} ${QT_RESOURCE_FILES})

  # Put everything together
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(EXE_NAME pcl_${SUBSYS_NAME})
  pcl_add_executable(${EXE_NAME} ${SUBSYS_NAME} ${SRCS} ${INCS} ${IMPL_INCS})
  target_link_libraries(${EXE_NAME} ${SUBSYS_LIBS})

  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME} ${INCS})
  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${IMPL_INCS})

  pcl_make_pkgconfig(${EXE_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

endif()


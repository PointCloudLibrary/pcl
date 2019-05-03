set(SUBSUBSYS_NAME point_cloud_editor)
set(SUBSUBSYS_DESC "Point Cloud Editor - Simple editor for 3D point clouds")
set(SUBSUBSYS_DEPS common filters io apps)

# QT5 Found?
if(NOT Qt5_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "Qt5 was not found.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

# Find OpenGL
if(NOT OPENGL_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "OpenGL was not found.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()


# Default to not building for now
if(${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${SUBSYS_DEPS})
PCL_ADD_DOC(${SUBSUBSYS_NAME})

if(NOT build)
  return()
endif()

set(MOC_INCS
  "include/pcl/apps/${SUBSUBSYS_NAME}/cloudEditorWidget.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/mainWindow.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/denoiseParameterForm.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/statisticsDialog.h"
)

set(RSRC
  resources/pceditor_resources.qrc
)

set(INCS ${MOC_INCS}
  "include/pcl/apps/${SUBSUBSYS_NAME}/cloud.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/cloudTransformTool.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/command.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/commandQueue.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/common.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/copyBuffer.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/copyCommand.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/cutCommand.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/deleteCommand.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/denoiseCommand.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/localTypes.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/pasteCommand.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/select1DTool.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/select2DTool.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/selection.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/selectionTransformTool.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/statistics.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/toolInterface.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/trackball.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/transformCommand.h"
)

set(SRCS
  src/main.cpp
  src/mainWindow.cpp
  src/commandQueue.cpp
  src/selection.cpp
  src/copyBuffer.cpp
  src/deleteCommand.cpp
  src/cutCommand.cpp
  src/pasteCommand.cpp
  src/cloud.cpp
  src/cloudEditorWidget.cpp
  src/cloudTransformTool.cpp
  src/select1DTool.cpp
  src/select2DTool.cpp
  src/selectionTransformTool.cpp
  src/transformCommand.cpp
  src/common.cpp
  src/denoiseCommand.cpp
  src/statistics.cpp
  src/statisticsDialog.cpp
  src/trackball.cpp
  src/denoiseParameterForm.cpp
)

qt5_wrap_cpp(MOC_SRCS ${MOC_INCS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
qt5_add_resources(RESOURCES_SRCS ${RSRC})

include_directories(
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
PCL_ADD_EXECUTABLE(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${SRCS} ${RESOURCES_SRCS} ${MOC_SRCS} ${INCS})

target_link_libraries("${EXE_NAME}" Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES} ${BOOST_LIBRARIES} pcl_common pcl_io pcl_filters)

PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}" ${INCS})
PCL_MAKE_PKGCONFIG(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} DESC ${SUBSUBSYS_DESC})

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${EXE_NAME})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

set(SUBSUBSYS_NAME point_cloud_editor)
set(SUBSUBSYS_DESC "Point Cloud Editor - Simple editor for 3D point clouds")
set(SUBSUBSYS_DEPS common filters io apps)
set(SUBSUBSYS_EXT_DEPS vtk ${QTX} OpenGL)

# Default to not building for now
if(${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSUBSYS_NAME} PARENT_NAME ${SUBSYS_NAME} DEPS ${SUBSUBSYS_DEPS} EXT_DEPS ${SUBSUBSYS_EXT_DEPS})
PCL_ADD_DOC(${SUBSUBSYS_NAME})

if(NOT build)
  return()
endif()

set(RSRC
  resources/pceditor_resources.qrc
)

set(INCS
  "include/pcl/apps/${SUBSUBSYS_NAME}/cloudEditorWidget.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/mainWindow.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/denoiseParameterForm.h"
  "include/pcl/apps/${SUBSUBSYS_NAME}/statisticsDialog.h"
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

include_directories(
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
PCL_ADD_EXECUTABLE(
  ${EXE_NAME}
  COMPONENT
    ${SUBSUBSYS_NAME}
  SOURCES
    ${SRCS}
    ${RSRC}
    ${INCS})

target_link_libraries("${EXE_NAME}" ${QTX}::Widgets ${QTX}::OpenGL ${OPENGL_LIBRARIES} ${BOOST_LIBRARIES} pcl_common pcl_io pcl_filters)

PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}" ${INCS})
PCL_MAKE_PKGCONFIG(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} DESC ${SUBSUBSYS_DESC})

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${EXE_NAME})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

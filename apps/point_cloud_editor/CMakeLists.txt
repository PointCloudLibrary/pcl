set(SUBSUBSYS_NAME point_cloud_editor)
set(SUBSUBSYS_DESC "Point Cloud Editor - Simple editor for 3D point clouds")
set(SUBSUBSYS_DEPS common filters io apps)

set(MOC_INCS "include/pcl/apps/${SUBSUBSYS_NAME}/cloudEditorWidget.h"
             "include/pcl/apps/${SUBSUBSYS_NAME}/mainWindow.h"
             "include/pcl/apps/${SUBSUBSYS_NAME}/denoiseParameterForm.h"
             "include/pcl/apps/${SUBSUBSYS_NAME}/statisticsDialog.h"
  )

set(RSRC resources/pceditor_resources.qrc)

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

set(SRCS src/main.cpp
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

if(NOT QT4_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "Qt4 was not found.")
else()
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

PCL_SUBSUBSYS_OPTION(BUILD "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(BUILD "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${SUBSYS_DEPS})
PCL_ADD_DOC(${SUBSUBSYS_NAME})

if(BUILD)
  include("${QT_USE_FILE}")
  set(QT_USE_QTOPENGL TRUE)
  #QT4_WRAP_CPP(MOC_SRCS ${MOC_INCS})
  QT4_WRAP_CPP(MOC_SRCS ${MOC_INCS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
  QT4_ADD_RESOURCES(RESOURCES_SRCS ${RSRC})

  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}"
                      "${CMAKE_CURRENT_SOURCE_DIR}/include"
                      "${QT_QTOPENGL_INCLUDE_DIR}"
		      )

  set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
  PCL_ADD_EXECUTABLE("${EXE_NAME}"
                     "${SUBSUBSYS_NAME}"
		     ${SRCS}
		     ${RESOURCES_SRCS}
		     ${MOC_SRCS}
		     ${INCS}
		     )

  TARGET_LINK_LIBRARIES("${EXE_NAME}"
                        ${QT_LIBRARIES}
                        ${QT_QTOPENGL_LIBRARY}
                        ${OPENGL_LIBRARIES}
                        ${BOOST_LIBRARIES}
                        pcl_common
                        pcl_io
                        pcl_filters
			)

  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}" ${INCS})
  PCL_MAKE_PKGCONFIG("${EXE_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" "" "" "" "" "")
endif()

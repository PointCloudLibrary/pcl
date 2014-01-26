SET(SUBSUBSYS_NAME point_cloud_editor)
SET(SUBSUBSYS_DESC "Point Cloud Editor - Simple editor for 3D point clouds")
SET(SUBSUBSYS_DEPS common filters io apps)

SET(MOC_INCS "include/pcl/apps/${SUBSUBSYS_NAME}/cloudEditorWidget.h"
             "include/pcl/apps/${SUBSUBSYS_NAME}/mainWindow.h"
             "include/pcl/apps/${SUBSUBSYS_NAME}/denoiseParameterForm.h"
             "include/pcl/apps/${SUBSUBSYS_NAME}/statisticsDialog.h"
  )

SET(RSRC resources/pceditor_resources.qrc)

SET(INCS ${MOC_INCS}
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

SET(SRCS src/main.cpp
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

IF(NOT QT4_FOUND)
  SET(DEFAULT AUTO_OFF)
  SET(REASON "Qt4 was not found.")
ELSE(NOT QT4_FOUND)
  SET(DEFAULT TRUE)
  SET(REASON)
ENDIF(NOT QT4_FOUND)

# Find OpenGL
IF(NOT OPENGL_FOUND)
  SET(DEFAULT AUTO_OFF)
  SET(REASON "OpenGL was not found.")
ELSEIF(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  SET(DEFAULT TRUE)
  SET(REASON)
ENDIF(NOT OPENGL_FOUND)


# Default to not building for now
if (${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(BUILD "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(BUILD "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${SUBSYS_DEPS})
PCL_ADD_DOC(${SUBSUBSYS_NAME})

IF(BUILD)
  INCLUDE("${QT_USE_FILE}")
  SET(QT_USE_QTOPENGL TRUE)
  #QT4_WRAP_CPP(MOC_SRCS ${MOC_INCS})
  QT4_WRAP_CPP(MOC_SRCS ${MOC_INCS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
  QT4_ADD_RESOURCES(RESOURCES_SRCS ${RSRC})

  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}"
                      "${CMAKE_CURRENT_SOURCE_DIR}/include"
                      "${QT_QTOPENGL_INCLUDE_DIR}"
		      )

  SET(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
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
ENDIF(BUILD)

set(SUBSUBSYS_NAME in_hand_scanner)
set(SUBSUBSYS_DESC "In-hand scanner for small objects")
set(SUBSUBSYS_DEPS     common     features     io     kdtree apps)
set(SUBSUBSYS_LIBS pcl_common pcl_features pcl_io pcl_kdtree)
set(SUBSUBSYS_EXT_DEPS ${QTX} OpenGL OpenGL_GLU openni)
set(REASON "")
set(DEFAULT OFF)

# Have Qt?
if("${QTX}" STREQUAL "")
  set(REASON "Cloud composer requires Qt.")
endif()

pcl_subsubsys_option(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSUBSYS_NAME} PARENT_NAME ${SUBSYS_NAME} DEPS ${SUBSUBSYS_DEPS} EXT_DEPS ${SUBSUBSYS_EXT_DEPS})

pcl_add_doc("${SUBSUBSYS_NAME}")

################################################################################

set(INCS
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/common_types.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/icp.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/input_data_processing.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/integration.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/mesh_processing.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/utils.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/visibility_confidence.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/main_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/help_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/opengl_viewer.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/in_hand_scanner.h"
)

set(IMPL_INCS
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/common_types.hpp"
)

set(SRCS
  src/help_window.cpp
  src/main.cpp
  src/main_window.cpp
  src/icp.cpp
  src/in_hand_scanner.cpp
  src/input_data_processing.cpp
  src/integration.cpp
  src/mesh_processing.cpp
  src/opengl_viewer.cpp
  src/visibility_confidence.cpp
)

set(UI
  src/main_window.ui
  src/help_window.ui)

# Offline integration
set(OI_INCS
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/integration.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/visibility_confidence.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/opengl_viewer.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/offline_integration.h"
)

set(OI_SRCS
  src/main_offline_integration.cpp
  src/integration.cpp
  src/offline_integration.cpp
  src/opengl_viewer.cpp
  src/visibility_confidence.cpp
)

################################################################################

if(NOT build)
  return()
endif()

set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")

PCL_ADD_EXECUTABLE(
  ${EXE_NAME}
  COMPONENT
    ${SUBSUBSYS_NAME}
  SOURCES
    ${SRCS}
    ${INCS}
    ${IMPL_INCS}
    ${UI}
  BUNDLE)

target_link_libraries("${EXE_NAME}" ${SUBSUBSYS_LIBS} ${OPENGL_LIBRARIES} ${QTX}::Concurrent ${QTX}::Widgets ${QTX}::OpenGL)
target_include_directories(${EXE_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
if (${QTX} MATCHES "Qt6")
  target_link_libraries("${EXE_NAME}" ${QTX}::OpenGLWidgets)
endif()

pcl_add_includes("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${INCS})
pcl_add_includes("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}/impl" ${IMPL_INCS})

PCL_MAKE_PKGCONFIG(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} DESC ${SUBSUBSYS_DESC})

PCL_ADD_EXECUTABLE(
  pcl_offline_integration
  COMPONENT
    ${SUBSUBSYS_NAME}
  SOURCES
    ${OI_SRCS}
    ${OI_INCS}
  BUNDLE)

target_link_libraries(pcl_offline_integration ${SUBSUBSYS_LIBS} ${OPENGL_LIBRARIES} ${QTX}::Concurrent ${QTX}::Widgets ${QTX}::OpenGL)
target_include_directories(pcl_offline_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
if (${QTX} MATCHES "Qt6")
  target_link_libraries(pcl_offline_integration ${QTX}::OpenGLWidgets)
endif()

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${PCL_IN_HAND_SCANNER_ALL_TARGETS})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

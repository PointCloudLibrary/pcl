set(SUBSUBSYS_NAME in_hand_scanner)
set(SUBSUBSYS_DESC "In-hand scanner for small objects")
set(SUBSUBSYS_DEPS     common     features     io     kdtree apps)
set(SUBSUBSYS_LIBS pcl_common pcl_features pcl_io pcl_kdtree)

################################################################################
# QT5 Found?
if(NOT Qt5_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "Qt5 was not found.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

# OpenGL
if(NOT OPENGL_FOUND AND NOT OPENGL_GLU_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "OpenGL & GLU are required for the in_hand_scanner app!")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

#OpenNI
if(NOT WITH_OPENNI)
  set(DEFAULT AUTO_OFF)
  set(REASON "OpenNI was not found or was disabled by the user.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

# Default to not building for now
if(${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

pcl_subsubsys_option(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
pcl_subsubsys_depend(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS Qt5 OpenGL OpenGL_GLU openni)

pcl_add_doc("${SUBSUBSYS_NAME}")

################################################################################

set(INCS
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/common_types.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/eigen.h"
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

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

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
target_link_libraries("${EXE_NAME}" ${SUBSUBSYS_LIBS} ${OPENGL_LIBRARIES} Qt5::Concurrent Qt5::Widgets Qt5::OpenGL)

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

target_link_libraries(pcl_offline_integration ${SUBSUBSYS_LIBS} ${OPENGL_LIBRARIES} Qt5::Concurrent Qt5::Widgets Qt5::OpenGL)

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${PCL_IN_HAND_SCANNER_ALL_TARGETS})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

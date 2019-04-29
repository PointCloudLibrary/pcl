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

# Qt
set(MOC_IN_HAND_SCANNER_INC     "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/in_hand_scanner.h")
set(MOC_OPENGL_VIEWER_INC       "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/opengl_viewer.h")
set(MOC_OFFLINE_INTEGRATION_INC "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/offline_integration.h")

set(MOC_MAIN_WINDOW_INC "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/main_window.h")
set(MOC_HELP_WINDOW_INC "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/help_window.h")
set(UI_MAIN_WINDOW      src/main_window.ui)
set(UI_HELP_WINDOW      src/help_window.ui)

# Offline integration
set(OI_INCS
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/integration.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/visibility_confidence.h"
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

qt5_wrap_cpp(MOC_IN_HAND_SCANNER_SRC     "${MOC_IN_HAND_SCANNER_INC}")
qt5_wrap_cpp(MOC_OPENGL_VIEWER_SRC       "${MOC_OPENGL_VIEWER_INC}" OPTIONS -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
qt5_wrap_cpp(MOC_OFFLINE_INTEGRATION_SRC "${MOC_OFFLINE_INTEGRATION_INC}" OPTIONS -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)

qt5_wrap_cpp(MOC_MAIN_WINDOW_SRC "${MOC_MAIN_WINDOW_INC}")
qt5_wrap_cpp(MOC_HELP_WINDOW_SRC "${MOC_HELP_WINDOW_INC}")
qt5_wrap_ui(UI_MAIN_WINDOW_INC   "${UI_MAIN_WINDOW}")
qt5_wrap_ui(UI_HELP_WINDOW_INC   "${UI_HELP_WINDOW}")

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

# In-hand scanner
list(APPEND INCS "${MOC_IN_HAND_SCANNER_INC}" "${MOC_OPENGL_VIEWER_INC}" "${MOC_MAIN_WINDOW_INC}" "${MOC_HELP_WINDOW_INC}" "${UI_MAIN_WINDOW_INC}" "${UI_HELP_WINDOW_INC}")
list(APPEND SRCS "${MOC_IN_HAND_SCANNER_SRC}" "${MOC_OPENGL_VIEWER_SRC}" "${MOC_MAIN_WINDOW_SRC}" "${MOC_HELP_WINDOW_SRC}")

set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
PCL_ADD_EXECUTABLE(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${SRCS} ${INCS} ${IMPL_INCS} BUNDLE)
target_link_libraries("${EXE_NAME}" ${SUBSUBSYS_LIBS} ${OPENGL_LIBRARIES} Qt5::Concurrent Qt5::Widgets Qt5::OpenGL)

pcl_add_includes("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${INCS})
pcl_add_includes("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}/impl" ${IMPL_INCS})

PCL_MAKE_PKGCONFIG(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} DESC ${SUBSUBSYS_DESC})

# Offline integration
list(APPEND OI_INCS "${MOC_OPENGL_VIEWER_INC}" "${MOC_OFFLINE_INTEGRATION_INC}")
list(APPEND OI_SRCS "${MOC_OPENGL_VIEWER_SRC}" "${MOC_OFFLINE_INTEGRATION_SRC}")

PCL_ADD_EXECUTABLE(pcl_offline_integration COMPONENT ${SUBSUBSYS_NAME} SOURCES ${OI_SRCS} ${OI_INCS} BUNDLE)
target_link_libraries(pcl_offline_integration ${SUBSUBSYS_LIBS} ${OPENGL_LIBRARIES} Qt5::Concurrent Qt5::Widgets Qt5::OpenGL)

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${PCL_IN_HAND_SCANNER_ALL_TARGETS})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

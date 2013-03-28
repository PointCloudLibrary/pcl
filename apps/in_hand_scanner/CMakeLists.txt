set(SUBSYS_NAME in_hand_scanner)
set(SUBSYS_DESC "In-hand scanner for small objects")
set(SUBSYS_DEPS     common     features     io     kdtree apps)
set(SUBSYS_LIBS pcl_common pcl_features pcl_io pcl_kdtree)

################################################################################

set(INCS
    include/pcl/apps/${SUBSYS_NAME}/boost.h
    include/pcl/apps/${SUBSYS_NAME}/common_types.h
    include/pcl/apps/${SUBSYS_NAME}/eigen.h
    include/pcl/apps/${SUBSYS_NAME}/icp.h
    include/pcl/apps/${SUBSYS_NAME}/input_data_processing.h
    include/pcl/apps/${SUBSYS_NAME}/integration.h
    include/pcl/apps/${SUBSYS_NAME}/mesh_processing.h
    include/pcl/apps/${SUBSYS_NAME}/utils.h
    include/pcl/apps/${SUBSYS_NAME}/visibility_confidence.h
)

set(IMPL_INCS
    include/pcl/apps/${SUBSYS_NAME}/impl/common_types.hpp
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
set(MOC_IN_HAND_SCANNER_INC     include/pcl/apps/${SUBSYS_NAME}/in_hand_scanner.h)
set(MOC_OPENGL_VIEWER_INC       include/pcl/apps/${SUBSYS_NAME}/opengl_viewer.h)
set(MOC_OFFLINE_INTEGRATION_INC include/pcl/apps/${SUBSYS_NAME}/offline_integration.h)

set(MOC_MAIN_WINDOW_INC include/pcl/apps/${SUBSYS_NAME}/main_window.h)
set(MOC_HELP_WINDOW_INC include/pcl/apps/${SUBSYS_NAME}/help_window.h)
set(UI_MAIN_WINDOW      src/main_window.ui)
set(UI_HELP_WINDOW      src/help_window.ui)

# Offline integration
set(OI_INCS
    include/pcl/apps/${SUBSYS_NAME}/integration.h
    include/pcl/apps/${SUBSYS_NAME}/visibility_confidence.h
)

set(OI_SRCS
    src/main_offline_integration.cpp
    src/integration.cpp
    src/offline_integration.cpp
    src/opengl_viewer.cpp
    src/visibility_confidence.cpp
)

################################################################################

# Default to not building for now
set(DEFAULT FALSE)

pcl_subsys_option(BUILD app_${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
pcl_subsys_depend(BUILD app_${SUBSYS_NAME} ${SUBSYS_DEPS})
pcl_add_doc(${SUBSYS_NAME})

set(ADDITIONAL_LIBS "")
if(BUILD)
  # Qt
  if(NOT QT4_FOUND)
    message(WARNING "Qt4 is needed for the in_hand_scanner app! It will not be built!")
    set(BUILD FALSE)
  endif()

  # OpenGL
  find_package(OpenGL)
  if(OPENGL_FOUND AND OPENGL_GLU_FOUND)
    list(APPEND ADDITIONAL_LIBS ${OPENGL_LIBRARIES})
  else()
    message(WARNING "OpenGL & GLU are needed for the in_hand_scanner app! It will not be built!")
    set(BUILD FALSE)
  endif()
endif()

################################################################################

if(BUILD)

  # Qt
  # http://qtnode.net/wiki/Qt4_with_cmake
  # http://qt-project.org/quarterly/view/using_cmake_to_build_qt_projects
  set(QT_USE_QTOPENGL TRUE)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_IN_HAND_SCANNER_SRC     ${MOC_IN_HAND_SCANNER_INC})
  qt4_wrap_cpp(MOC_OPENGL_VIEWER_SRC       ${MOC_OPENGL_VIEWER_INC} OPTIONS -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
  qt4_wrap_cpp(MOC_OFFLINE_INTEGRATION_SRC ${MOC_OFFLINE_INTEGRATION_INC} OPTIONS -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)

  qt4_wrap_cpp(MOC_MAIN_WINDOW_SRC ${MOC_MAIN_WINDOW_INC})
  qt4_wrap_cpp(MOC_HELP_WINDOW_SRC ${MOC_HELP_WINDOW_INC})
  qt4_wrap_ui(UI_MAIN_WINDOW_INC   ${UI_MAIN_WINDOW})
  qt4_wrap_ui(UI_HELP_WINDOW_INC   ${UI_HELP_WINDOW})

  list(APPEND ADDITIONAL_LIBS ${QT_LIBRARIES})
  include_directories(${CMAKE_CURRENT_BINARY_DIR}) # For the ui files

  # In-hand scanner
  list(APPEND INCS ${MOC_IN_HAND_SCANNER_INC} ${MOC_OPENGL_VIEWER_INC} ${MOC_MAIN_WINDOW_INC} ${MOC_HELP_WINDOW_INC} ${UI_MAIN_WINDOW_INC} ${UI_HELP_WINDOW_INC})
  list(APPEND SRCS ${MOC_IN_HAND_SCANNER_SRC} ${MOC_OPENGL_VIEWER_SRC} ${MOC_MAIN_WINDOW_SRC} ${MOC_HELP_WINDOW_SRC})

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(EXE_NAME pcl_${SUBSYS_NAME})
  pcl_add_executable_opt_bundle(${EXE_NAME} ${SUBSYS_NAME} ${SRCS} ${INCS} ${IMPL_INCS})
  target_link_libraries(${EXE_NAME} ${SUBSYS_LIBS} ${ADDITIONAL_LIBS})

  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME} ${INCS})
  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${IMPL_INCS})

  pcl_make_pkgconfig(${EXE_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

  # Offline integration
  list(APPEND OI_INCS ${MOC_OPENGL_VIEWER_INC} ${MOC_OFFLINE_INTEGRATION_INC})
  list(APPEND OI_SRCS ${MOC_OPENGL_VIEWER_SRC} ${MOC_OFFLINE_INTEGRATION_SRC})

  pcl_add_executable_opt_bundle(pcl_offline_integration ${SUBSYS_NAME} ${OI_SRCS} ${OI_INCS})
  target_link_libraries(pcl_offline_integration ${SUBSYS_LIBS} ${ADDITIONAL_LIBS})

endif()

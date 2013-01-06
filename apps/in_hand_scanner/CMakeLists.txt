set(SUBSYS_NAME in_hand_scanner)
set(SUBSYS_DESC "In-hand scanner for small objects")
set(SUBSYS_DEPS     common     features     io     kdtree      visualization apps)
set(SUBSYS_LIBS pcl_common pcl_features pcl_io pcl_kdtree  pcl_visualization)

################################################################################

set(INCS
    include/pcl/apps/${SUBSYS_NAME}/boost.h
    include/pcl/apps/${SUBSYS_NAME}/common_types.h
    include/pcl/apps/${SUBSYS_NAME}/eigen.h
    include/pcl/apps/${SUBSYS_NAME}/icp.h
    include/pcl/apps/${SUBSYS_NAME}/input_data_processing.h
    include/pcl/apps/${SUBSYS_NAME}/integration.h
    include/pcl/apps/${SUBSYS_NAME}/visibility_confidence.h
)

set(QT_MOC_INCS
    include/pcl/apps/${SUBSYS_NAME}/in_hand_scanner.h
    include/pcl/apps/${SUBSYS_NAME}/opengl_viewer.h
)

set(IMPL_INCS
    include/pcl/apps/${SUBSYS_NAME}/impl/common_types.hpp
)

set(SRCS
    src/main.cpp
    src/icp.cpp
    src/in_hand_scanner.cpp
    src/input_data_processing.cpp
    src/integration.cpp
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

  # Qt http://qtnode.net/wiki/Qt4_with_cmake
  set(QT_USE_QTOPENGL TRUE)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(QT_MOC_SRCS ${QT_MOC_INCS})
  list(APPEND SRCS ${QT_MOC_SRCS})
  list(APPEND ADDITIONAL_LIBS ${QT_LIBRARIES})

  # Put everything together
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(EXE_NAME pcl_${SUBSYS_NAME})
  pcl_add_executable_opt_bundle(${EXE_NAME} ${SUBSYS_NAME} ${SRCS} ${INCS} ${IMPL_INCS} ${QT_MOC_INCS})
  target_link_libraries(${EXE_NAME} ${SUBSYS_LIBS} ${ADDITIONAL_LIBS})

  pcl_add_executable_opt_bundle(pcl_test_integration ${SUBSYS_NAME} src/test_integration.cpp src/integration.cpp)
  target_link_libraries(pcl_test_integration pcl_common pcl_io pcl_visualization)

  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME} ${INCS} ${QT_MOC_INCS})
  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${IMPL_INCS})

  pcl_make_pkgconfig(${EXE_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

endif()

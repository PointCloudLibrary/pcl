set(SUBSYS_NAME in_hand_scanner)
set(SUBSYS_DESC "In-hand scanner for small objects")
set(SUBSYS_DEPS     common     geometry     io     kdtree     registration     visualization)
set(SUBSYS_LIBS pcl_common pcl_geometry pcl_io pcl_kdtree pcl_registration pcl_visualization)

################################################################################

# Header files
set(INCS include/pcl/apps/${SUBSYS_NAME}/common_types.h
         include/pcl/apps/${SUBSYS_NAME}/custom_interactor_style.h
         include/pcl/apps/${SUBSYS_NAME}/eigen.h
         include/pcl/apps/${SUBSYS_NAME}/icp.h
         include/pcl/apps/${SUBSYS_NAME}/in_hand_scanner.h
         include/pcl/apps/${SUBSYS_NAME}/input_data_processing.h
         include/pcl/apps/${SUBSYS_NAME}/integration.h
   )

set(IMPL_INCS "")

# Source files
set(SRCS src/main.cpp
         src/custom_interactor_style.cpp
         src/icp.cpp
         src/in_hand_scanner.cpp
         src/input_data_processing.cpp
         src/integration.cpp
   )

################################################################################

# Default to not building for now
set(DEFAULT FALSE)

pcl_subsys_option(BUILD ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
pcl_subsys_depend(BUILD ${SUBSYS_NAME} ${SUBSYS_DEPS})
pcl_add_doc(${SUBSYS_NAME})

################################################################################

if(BUILD)

  # Put everything together
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(EXE_NAME pcl_${SUBSYS_NAME})
  pcl_add_executable_opt_bundle(${EXE_NAME} ${SUBSYS_NAME} ${SRCS} ${INCS} ${IMPL_INCS})
  target_link_libraries(${EXE_NAME} ${SUBSYS_LIBS})

  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME} ${INCS})
  pcl_add_includes(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${IMPL_INCS})

  pcl_make_pkgconfig(${EXE_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

endif()

set(SUBSYS_NAME documentation)
set(SUBSYS_DESC "Point clouds library documentation")
set(SUBSYS_DEPS)

set(build FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
  add_subdirectory(doxygen)
  add_subdirectory(tutorials)
endif(build)

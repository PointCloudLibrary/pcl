option(BUILD_documentation "Point clouds library documentation" OFF)

if(BUILD_documentation)
  add_subdirectory(doxygen)
  add_subdirectory(tutorials)
endif(BUILD_documentation)

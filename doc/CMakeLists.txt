option(BUILD_documentation "Build the documentation (Doxygen)." OFF)
option(BUILD_tutorials "Build the tutorials." OFF)
mark_as_advanced(BUILD_documentation BUILD_tutorials)

add_subdirectory(doxygen)
add_subdirectory(tutorials)

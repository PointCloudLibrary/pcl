find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in ${doxyfile})
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${doxyfile})
endif(DOXYGEN_FOUND)


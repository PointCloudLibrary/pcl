if(NOT SPHINX_FOUND)
  return()
endif()

add_custom_target(tutorials ALL
                  COMMAND "${SPHINX_EXECUTABLE}"
	                  -b html
	                  -d "${SPHINX_CACHE_DIR}"
	                  -Dversion="${PCL_VERSION_PRETTY}"
	                  -Drelease="${PCL_VERSION_PRETTY}"
	                  "${CMAKE_CURRENT_SOURCE_DIR}/content" html)
add_dependencies(tutorials doc)
set_target_properties(tutorials PROPERTIES FOLDER "Documentation (Tutorials)")
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
        DESTINATION "${DOC_INSTALL_DIR}/tutorials"
        COMPONENT doc)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/content/sources"
        DESTINATION "${DOC_INSTALL_DIR}/tutorials"
        COMPONENT doc)

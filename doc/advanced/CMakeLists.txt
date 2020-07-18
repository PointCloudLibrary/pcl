if(NOT SPHINX_FOUND)
  return()
endif()

add_custom_target(advanced ALL
                  COMMAND "${SPHINX_EXECUTABLE}"
                    -b html
                    -d "${SPHINX_CACHE_DIR}"
                    -Dversion="${PCL_VERSION_PRETTY}"
                    -Drelease="${PCL_VERSION_PRETTY}"
                    "${CMAKE_CURRENT_SOURCE_DIR}/content" html)
add_dependencies(advanced doc)
set_target_properties(advanced PROPERTIES FOLDER "Documentation (Advanced)")
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
        DESTINATION "${DOC_INSTALL_DIR}/advanced"
        COMPONENT doc)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/content/files"
        DESTINATION "${DOC_INSTALL_DIR}/advanced"
        COMPONENT doc)

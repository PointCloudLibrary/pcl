if(SPHINX_FOUND)
  add_custom_target(tutorials ALL
                    COMMAND "${SPHINX_EXECUTABLE}" -b html -a -d "${SPHINX_CACHE_DIR}" -D html_file_suffix=".${SPHINX_HTML_FILE_SUFFIX}" "${CMAKE_CURRENT_SOURCE_DIR}/content" html)
  add_dependencies(tutorials doc)
  if(USE_PROJECT_FOLDERS)
    set_target_properties(tutorials PROPERTIES FOLDER "Documentation (Tutorials)")
  endif(USE_PROJECT_FOLDERS)
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
          DESTINATION "${DOC_INSTALL_DIR}/tutorials"
          COMPONENT doc)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/content/sources"
          DESTINATION "${DOC_INSTALL_DIR}/tutorials"
          COMPONENT doc)
endif(SPHINX_FOUND)

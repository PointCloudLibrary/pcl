if(SPHINX_FOUND)
  add_custom_target(advanced ALL
                    COMMAND "${SPHINX_EXECUTABLE}" -b html -a -d "${SPHINX_CACHE_DIR}" -D html_file_suffix=".${SPHINX_HTML_FILE_SUFFIX}" "${CMAKE_CURRENT_SOURCE_DIR}/content" html)
  add_dependencies(advanced doc)
  if(USE_PROJECT_FOLDERS)
    set_target_properties(advanced PROPERTIES FOLDER "Documentation (Advanced)")
  endif(USE_PROJECT_FOLDERS)
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
          DESTINATION "${DOC_INSTALL_DIR}/advanced"
          COMPONENT doc)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/content/files"
          DESTINATION "${DOC_INSTALL_DIR}/advanced"
          COMPONENT doc)
endif(SPHINX_FOUND)

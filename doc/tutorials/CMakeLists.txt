option(WITH_TUTORIALS "Build tutorials (requires Sphinx)" TRUE)
if(WITH_TUTORIALS)
  find_package(Sphinx)
endif(WITH_TUTORIALS)

if(SPHINX_FOUND)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
  if(WIN32)
    set(TMPDIR "$ENV{TEMP}")
  else()
    set(TMPDIR "/tmp")
  endif()
  file(TO_CMAKE_PATH "${TMPDIR}" TMPDIR)
  add_custom_target(tutorials
                    COMMAND "${CMAKE_COMMAND}" -E remove_directory "${TMPDIR}/doctrees"
                    COMMAND "${SPHINX_EXECUTABLE}" -b html -a -d "${TMPDIR}/doctrees" "${CMAKE_CURRENT_SOURCE_DIR}/content" html
                    COMMAND "${SPHINX_EXECUTABLE}" -b html -a -d "${TMPDIR}/doctrees" "${CMAKE_CURRENT_SOURCE_DIR}/../advanced/content" ../advanced/html
                    )
  if(USE_PROJECT_FOLDERS)
    set_target_properties(tutorials PROPERTIES FOLDER "Documentation")
  endif(USE_PROJECT_FOLDERS)
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
          DESTINATION "${DOC_INSTALL_DIR}/tutorials"
          COMPONENT doc)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/content/sources"
          DESTINATION "${DOC_INSTALL_DIR}/tutorials"
          COMPONENT doc
          PATTERN ".svn" EXCLUDE)
  if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/../advanced/html")
    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../advanced/html"
            DESTINATION "${DOC_INSTALL_DIR}/advanced"
            COMPONENT doc)
  endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../advanced/content/files")
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../advanced/content/files"
            DESTINATION "${DOC_INSTALL_DIR}/advanced"
            COMPONENT doc
            PATTERN ".svn" EXCLUDE)
  endif()
endif(SPHINX_FOUND)

if(BUILD_tutorials)
  find_package(Sphinx)
  if(SPHINX_FOUND)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
    if(WIN32)
      set(TMPDIR "$ENV{TEMP}")
    else()
      set(TMPDIR "/tmp")
    endif()
    file(TO_CMAKE_PATH "${TMPDIR}" TMPDIR)
    add_custom_target(Tutorials 
                      COMMAND ${CMAKE_COMMAND} -E remove_directory "${TMPDIR}/doctrees"
                      COMMAND ${SPHINX_EXECUTABLE} -b html -a -d "${TMPDIR}/doctrees" "${CMAKE_CURRENT_SOURCE_DIR}/content" html
                      )
    if(USE_PROJECT_FOLDERS)
      set_target_properties(Tutorials PROPERTIES FOLDER "Documentation")
    endif(USE_PROJECT_FOLDERS)
    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
            DESTINATION share/doc/pcl/tutorials
            COMPONENT pcl_docs)
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/content/sources"
            DESTINATION share/doc/pcl/tutorials
            COMPONENT pcl_docs
            PATTERN ".svn" EXCLUDE)
  endif(SPHINX_FOUND)
endif(BUILD_tutorials)

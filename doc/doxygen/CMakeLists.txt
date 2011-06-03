if(BUILD_documentation)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    find_package(HTMLHelp)
    if(HTML_HELP_COMPILER)
      set(DOCUMENTATION_HTML_HELP YES)
    else(HTML_HELP_COMPILER)
      set(DOCUMENTATION_HTML_HELP NO)
    endif(HTML_HELP_COMPILER)

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
    set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in" ${doxyfile})
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${doxyfile})
    if(USE_PROJECT_FOLDERS)
      set_target_properties(doc PROPERTIES FOLDER "Documentation")
    endif(USE_PROJECT_FOLDERS)

    if(DOCUMENTATION_HTML_HELP STREQUAL YES)
      install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
              DESTINATION share/doc/pcl
              COMPONENT pcl_docs
              FILES_MATCHING PATTERN *.chm
              )
    else(DOCUMENTATION_HTML_HELP STREQUAL YES)
      install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html" 
              DESTINATION share/doc/pcl
              PATTERN "*.md5" EXCLUDE
              PATTERN "*.map" EXCLUDE
              PATTERN "*.chm" EXCLUDE
              COMPONENT pcl_docs)
    endif(DOCUMENTATION_HTML_HELP STREQUAL YES)
  endif(DOXYGEN_FOUND)
endif(BUILD_documentation)

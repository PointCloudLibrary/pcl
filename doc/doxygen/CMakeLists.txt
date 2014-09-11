if(WITH_DOCS)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    find_package(HTMLHelp)
  endif(DOXYGEN_FOUND)
endif(WITH_DOCS)

message(STATUS "DOXYGEN_FOUND ${DOXYGEN_FOUND}")
message(STATUS "HTML_HELP_COMPILER ${HTML_HELP_COMPILER}")

if(DOXYGEN_FOUND)
  if(HTML_HELP_COMPILER)
    set(DOCUMENTATION_HTML_HELP YES)
  else(HTML_HELP_COMPILER)
    set(DOCUMENTATION_HTML_HELP NO)
  endif(HTML_HELP_COMPILER)
  if(DOXYGEN_DOT_EXECUTABLE)
    set(HAVE_DOT YES)
  else(DOXYGEN_DOT_EXECUTABLE)
    set(HAVE_DOT NO)
  endif(DOXYGEN_DOT_EXECUTABLE)

  option(DOXYGEN_USE_SHORT_NAMES "Generate shorter (but less readable) file names" OFF)
  if(DOXYGEN_USE_SHORT_NAMES)
    set(SHORT_NAMES YES)
  else(DOXYGEN_USE_SHORT_NAMES)
    set(SHORT_NAMES NO)
  endif(DOXYGEN_USE_SHORT_NAMES)

  set(STRIPPED_HEADERS "${PCL_SOURCE_DIR}/${PCL_MODULES_NAMES}/include")
  string(REPLACE ";" "/include \\\n\t\t\t\t\t\t\t\t\t\t\t\t ${PCL_SOURCE_DIR}/" STRIPPED_HEADERS "${STRIPPED_HEADERS}")
  set(DOC_SOURCE_DIR "\"${PCL_SOURCE_DIR}\"\\")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
  set(doxy_html_index_file "${CMAKE_CURRENT_BINARY_DIR}/html/index.html")
  set(doxyfile_in "${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in")
  set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
  configure_file("${doxyfile_in}" "${doxyfile}")

  add_custom_command(OUTPUT "${doxy_html_index_file}"
                     COMMAND "${DOXYGEN_EXECUTABLE}" "${doxyfile}"
                     # The following should be ${doxyfile} only but it
                     # will break the dependency.
                     # The optimal solution would be creating a
                     # custom_command for ${doxyfile} generation
                     # but I still have to figure out how...
                     MAIN_DEPENDENCY "${doxyfile}" "${doxyfile_in}"
                     DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/pcl.doxy"
                     COMMENT "Generating HTML documentation")
  add_custom_target(doc ALL DEPENDS "${doxy_html_index_file}")

  if(USE_PROJECT_FOLDERS)
    set_target_properties(doc PROPERTIES FOLDER "Documentation (Doxygen)")
  endif(USE_PROJECT_FOLDERS)
  if(DOCUMENTATION_HTML_HELP STREQUAL YES)
    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
            DESTINATION "${DOC_INSTALL_DIR}"
            COMPONENT doc
            FILES_MATCHING PATTERN *.chm
            )
  else(DOCUMENTATION_HTML_HELP STREQUAL YES)
    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html" 
            DESTINATION "${DOC_INSTALL_DIR}"
            COMPONENT doc
            PATTERN "*.md5" EXCLUDE
            PATTERN "*.map" EXCLUDE
            PATTERN "*.chm" EXCLUDE)
  endif(DOCUMENTATION_HTML_HELP STREQUAL YES)
endif(DOXYGEN_FOUND)

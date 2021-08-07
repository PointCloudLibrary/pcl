if(WITH_DOCS)
  find_package(Doxygen REQUIRED)
  if(DOXYGEN_FOUND)
    find_package(HTMLHelp)
  endif()
endif()

message(STATUS "DOXYGEN_FOUND ${DOXYGEN_FOUND}")
message(STATUS "HTML_HELP_COMPILER ${HTML_HELP_COMPILER}")

if(NOT DOXYGEN_FOUND)
  return()
endif()

if(HTML_HELP_COMPILER)
  set(DOCUMENTATION_HTML_HELP YES)
else()
  set(DOCUMENTATION_HTML_HELP NO)
endif()
if(DOCUMENTATION_HTML_HELP)
  set(DOCUMENTATION_SEARCHENGINE NO)
else()
  set(DOCUMENTATION_SEARCHENGINE YES)
endif()
if(DOXYGEN_DOT_EXECUTABLE)
  set(HAVE_DOT YES)
else()
  set(HAVE_DOT NO)
endif()

option(DOXYGEN_USE_SHORT_NAMES "Generate shorter (but less readable) file names" OFF)
if(DOXYGEN_USE_SHORT_NAMES)
  set(SHORT_NAMES YES)
else()
  set(SHORT_NAMES NO)
endif()

set(STRIPPED_HEADERS "${PCL_SOURCE_DIR}/${PCL_MODULES_NAMES}/include")
string(REPLACE ";" "/include \\\n                         ${PCL_SOURCE_DIR}/" STRIPPED_HEADERS "${STRIPPED_HEADERS}")
set(DOC_SOURCE_DIR "${PCL_SOURCE_DIR}")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in" "${doxyfile}")

# The following process to use a stamp file is customized from the CMake 3.16.5 `doxygen_add_docs` command
set(STAMP_FILE "${CMAKE_CURRENT_BINARY_DIR}/doc.stamp")
add_custom_command(
    VERBATIM
    OUTPUT ${STAMP_FILE}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND "${DOXYGEN_EXECUTABLE}" "${doxyfile}"
    COMMAND ${CMAKE_COMMAND} -E touch ${STAMP_FILE}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS "${doxyfile}" ${DOC_SOURCE_DIR}
    COMMENT "Generate stamp file for target doc"
)
add_custom_target(doc ALL
    DEPENDS ${STAMP_FILE}
    SOURCES ${DOC_SOURCE_DIR}
)

set_target_properties(doc PROPERTIES FOLDER "Documentation (Doxygen)")
if(DOCUMENTATION_HTML_HELP STREQUAL YES)
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
          DESTINATION "${DOC_INSTALL_DIR}"
          COMPONENT doc
          FILES_MATCHING PATTERN *.chm
          )
else()
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
          DESTINATION "${DOC_INSTALL_DIR}"
          COMPONENT doc
          PATTERN "*.md5" EXCLUDE
          PATTERN "*.map" EXCLUDE
          PATTERN "*.chm" EXCLUDE)
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/pointcloudlibrary_logo.png"
          DESTINATION "${DOC_INSTALL_DIR}/html"
          COMPONENT doc)
endif()

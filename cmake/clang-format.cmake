find_package(ClangFormat 10)
# search for version number in clang-format without version number
if(ClangFormat_FOUND)
  message(STATUS "Adding target 'format'")
  add_custom_target(
  format
  COMMAND sh
    ${PCL_SOURCE_DIR}/.dev/format.sh
    ${ClangFormat_EXECUTABLE}
    ${PCL_SOURCE_DIR}
  )
endif()

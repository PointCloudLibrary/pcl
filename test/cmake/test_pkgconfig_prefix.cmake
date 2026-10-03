if(IS_ABSOLUTE "${PKGCFG_INSTALL_DIR}")
  set(expected_prefix "prefix=${CMAKE_INSTALL_PREFIX}")
else()
  file(RELATIVE_PATH pkgconfig_to_prefix
       "${CMAKE_INSTALL_PREFIX}/${PKGCFG_INSTALL_DIR}"
       "${CMAKE_INSTALL_PREFIX}")
  set(expected_prefix "prefix=\${pcfiledir}/${pkgconfig_to_prefix}")
endif()

file(GLOB_RECURSE pkgconfig_files "${PCL_BINARY_DIR}/pcl_*.pc")
if(NOT pkgconfig_files)
  message(FATAL_ERROR "No generated PCL pkg-config files found")
endif()

foreach(pkgconfig_file IN LISTS pkgconfig_files)
  file(STRINGS "${pkgconfig_file}" prefix_line REGEX "^prefix=")
  if(NOT prefix_line STREQUAL expected_prefix)
    message(FATAL_ERROR
            "Unexpected prefix in ${pkgconfig_file}:\n"
            "  expected: ${expected_prefix}\n"
            "  actual:   ${prefix_line}")
  endif()
endforeach()

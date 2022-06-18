
if(WIN32)
  option(BUILD_all_in_one_installer "Build an all-in-one NSIS installer" OFF)
endif()

if(NOT BUILD_all_in_one_installer)
  return()
endif()

# get root directory of each dependency libraries to be copied to PCL/3rdParty
get_filename_component(BOOST_ROOT "${Boost_INCLUDE_DIR}" PATH)  # ../Boost/include/boost-x_x/ -> ../Boost/include/
get_filename_component(BOOST_ROOT "${BOOST_ROOT}" PATH)         # ../Boost/include/           -> ../Boost/
get_filename_component(EIGEN_ROOT "${EIGEN_INCLUDE_DIRS}" PATH) # ../Eigen3/include/          -> ../Eigen3/
get_filename_component(QHULL_ROOT "${Qhull_DIR}" PATH)          # ../qhull/lib/cmake/Qhull/   -> ../qhull/lib/cmake
get_filename_component(QHULL_ROOT "${QHULL_ROOT}" PATH)         # ../qhull/lib/cmake/         -> ../qhull/lib/
get_filename_component(QHULL_ROOT "${QHULL_ROOT}" PATH)         # ../qhull/lib/               -> ../qhull/
get_filename_component(VTK_ROOT "${VTK_DIR}" PATH)              # ../VTK/lib/cmake/vtk-x.x/   -> ../VTK/lib/cmake/
get_filename_component(VTK_ROOT "${VTK_ROOT}" PATH)             # ../VTK/lib/cmake/           -> ../VTK/lib/
get_filename_component(VTK_ROOT "${VTK_ROOT}" PATH)             # ../VTK/lib/                 -> ../VTK/

set(PCL_3RDPARTY_COMPONENTS)
foreach(dep Eigen Boost Qhull FLANN VTK)
  string(TOUPPER ${dep} DEP)
  install(
    DIRECTORY "${${DEP}_ROOT}/"
    DESTINATION 3rdParty/${dep}
    COMPONENT ${dep}
    PATTERN "*/Uninstall.exe" EXCLUDE
  )
  list(APPEND PCL_3RDPARTY_COMPONENTS ${dep})
endforeach()

if(WITH_RSSDK2)
  get_filename_component(RSSDK2_ROOT "${RSSDK2_INCLUDE_DIRS}" PATH)
  install(
          DIRECTORY "${RSSDK2_ROOT}/"
          DESTINATION 3rdParty/librealsense2
          COMPONENT rssdk2
          PATTERN "*/Uninstall.exe" EXCLUDE
      )
  list(APPEND PCL_3RDPARTY_COMPONENTS rssdk2)
endif()

if(WITH_OPENNI)
  if(CMAKE_CL_64)
    set(OPENNI_PACKAGE "OpenNI-Win64-1.5.4-Dev.msi")
    set(OPENNI_URL "http://sourceforge.net/projects/pointclouds/files/dependencies/${OPENNI_PACKAGE}")
    set(OPENNI_MD5 c8f9cbe8447a16d32572a4e2c2d00af0)
    set(OPENNI_SENSOR_PACKAGE "Sensor-Win-OpenSource64-5.1.0.msi")
    set(OPENNI_SENSOR_URL "http://sourceforge.net/projects/pointclouds/files/dependencies/${OPENNI_SENSOR_PACKAGE}")
    set(OPENNI_SENSOR_MD5 badb880116436870943b1b7c447dfa22)
  else()
    set(OPENNI_PACKAGE "OpenNI-Win32-1.5.4-Dev.msi")
    set(OPENNI_URL "http://sourceforge.net/projects/pointclouds/files/dependencies/${OPENNI_PACKAGE}")
    set(OPENNI_MD5 996d48f447b41a5501b7d22af27ab251)
    set(OPENNI_SENSOR_PACKAGE "Sensor-Win-OpenSource32-5.1.0.msi")
    set(OPENNI_SENSOR_URL "http://sourceforge.net/projects/pointclouds/files/dependencies/${OPENNI_SENSOR_PACKAGE}")
    set(OPENNI_SENSOR_MD5 55da1f7541d7c9c98772bddf801c7e1c)
  endif()

  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "  IntCmp $OpenNI_selected 0 noinstall_openni_packages\n")

  file(DOWNLOAD ${OPENNI_URL} "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_PACKAGE}"
    STATUS _openni_download_status LOG _openni_download_log
    EXPECTED_MD5 ${OPENNI_MD5}
     )
  list(GET _openni_download_status 0 _error_code)
  list(GET _openni_download_status 1 _error_message)
  if(_error_code EQUAL 0)
    install(
      FILES "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_PACKAGE}"
      DESTINATION 3rdParty/OpenNI
      COMPONENT OpenNI
    )
    list(APPEND PCL_3RDPARTY_COMPONENTS OpenNI)
    string(APPEND CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\n    ExecWait 'msiexec /i \\\"$INSTDIR\\\\3rdParty\\\\OpenNI\\\\${OPENNI_PACKAGE}\\\" /quiet '")
  else()
    message("WARNING : Could not download ${OPENNI_URL}, error code : ${_error_code}, error message : ${_error_message}")
  endif()

  file(DOWNLOAD ${OPENNI_SENSOR_URL} "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_SENSOR_PACKAGE}"
    STATUS _openni_download_status LOG _openni_download_log
    EXPECTED_MD5 ${OPENNI_SENSOR_MD5}
     )
  list(GET _openni_download_status 0 _error_code)
  list(GET _openni_download_status 1 _error_message)
  if(_error_code EQUAL 0)
    install(
      FILES "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_SENSOR_PACKAGE}"
      DESTINATION 3rdParty/OpenNI
      COMPONENT OpenNI
    )
    list(APPEND PCL_3RDPARTY_COMPONENTS OpenNI)
    string(APPEND CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\n    ExecWait 'msiexec /i \\\"$INSTDIR\\\\3rdParty\\\\OpenNI\\\\${OPENNI_SENSOR_PACKAGE}\\\" /quiet '")
  else()
    message("WARNING : Could not download ${OPENNI_SENSOR_URL}, error code : ${_error_code}, error message : ${_error_message}")
  endif()
  list(REMOVE_DUPLICATES PCL_3RDPARTY_COMPONENTS)
  string(APPEND CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\n  noinstall_openni_packages:\n")
endif()

if(WITH_OPENNI2)
  if(CMAKE_CL_64)
    set(OPENNI2_PACKAGE "OpenNI-Windows-x64-2.2.msi")
    set(OPENNI2_ZIP "OpenNI-Windows-x64-2.2.0.33.zip")
    set(OPENNI2_URL "http://com.occipital.openni.s3.amazonaws.com/${OPENNI2_ZIP}")
    set(OPENNI2_MD5 d187f1dd0b091e27cebd03216b1bfff5)
  else()
    set(OPENNI2_PACKAGE "OpenNI-Windows-x86-2.2.msi")
    set(OPENNI2_ZIP "OpenNI-Windows-x86-2.2.0.33.zip")
    set(OPENNI2_URL "http://com.occipital.openni.s3.amazonaws.com/${OPENNI2_ZIP}")
    set(OPENNI2_MD5 59b38e23d951d59917a35f7f89efaf22)
  endif()

  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "  IntCmp $OpenNI2_selected 0 noinstall_openni2_packages\n")

  file(DOWNLOAD ${OPENNI2_URL} "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI2_ZIP}"
    STATUS _openni2_download_status LOG _openni2_download_log
    EXPECTED_MD5 ${OPENNI2_MD5}
     )
  list(GET _openni2_download_status 0 _error_code)
  list(GET _openni2_download_status 1 _error_message)
  if(_error_code EQUAL 0)
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar -xzf ${OPENNI2_ZIP}
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      RESULT_VARIABLE _error_code
      ERROR_VARIABLE _error_message
    )
    if(_error_code EQUAL 0)
      install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI2_PACKAGE}"
        DESTINATION 3rdParty/OpenNI2
        COMPONENT OpenNI2
      )
      list(APPEND PCL_3RDPARTY_COMPONENTS OpenNI2)
      string(APPEND CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\n    ExecWait 'msiexec /i \\\"$INSTDIR\\\\3rdParty\\\\OpenNI2\\\\${OPENNI2_PACKAGE}\\\" /quiet '")
    else()
      message("WARNING : Could not unzip ${OPENNI2_ZIP}, error code : ${_error_code}, error message : ${_error_message}")
    endif()
  else()
    message("WARNING : Could not download ${OPENNI2_ZIP_URL}, error code : ${_error_code}, error message : ${_error_message}")
  endif()
  list(REMOVE_DUPLICATES PCL_3RDPARTY_COMPONENTS)
  string(APPEND CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\n  noinstall_openni2_packages:\n")
endif()

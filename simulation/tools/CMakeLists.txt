PCL_ADD_EXECUTABLE(range-test ${SUBSYS_NAME} range_test.cpp)
target_link_libraries(range-test pcl_common pcl_simulation pcl_io glut)


#add_executable(range-test-drive-pcl-version range_test_drive_pcl_version.cpp)
#pods_use_pkg_config_packages(range-test-drive-pcl-version 
#      pcl_simulation-1.4 pcl_io-1.4  )
#target_link_libraries (range-test-drive-pcl-version  glut    )
#pods_install_executables(range-test-drive-pcl-version)

                 

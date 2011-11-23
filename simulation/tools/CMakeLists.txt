PCL_ADD_EXECUTABLE(range-test ${SUBSYS_NAME} range_test.cpp)
target_link_libraries(range-test pcl_common pcl_simulation pcl_io glut)


include_directories(${VTK_INCLUDE_DIRS})
PCL_ADD_EXECUTABLE(range-test-v2 ${SUBSYS_NAME} range_test_v2.cpp)
target_link_libraries (range-test-v2  ${VTK_IO_TARGET_LINK_LIBRARIES}  glut
  pcl_simulation   pcl_common  pcl_io pcl_range_image pcl_visualization)
#pcl_surface pcl_features



#add_executable(range-test-drive-pcl-version range_test_drive_pcl_version.cpp)
#pods_use_pkg_config_packages(range-test-drive-pcl-version 
#      pcl_simulation-1.4 pcl_io-1.4  )
#target_link_libraries (range-test-drive-pcl-version  glut    )
#pods_install_executables(range-test-drive-pcl-version)

                 


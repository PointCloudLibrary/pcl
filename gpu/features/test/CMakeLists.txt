if(BUILD_TESTS)

    include_directories("${PCL_SOURCE_DIR}/test/gtest-1.6.0/include"
                        "${PCL_SOURCE_DIR}/test/gtest-1.6.0")
    
    set(pcl_gtest_sources "${PCL_SOURCE_DIR}/test/gtest-1.6.0/src/gtest-all.cc")

    set(the_test_target test_gpu_features)
    
    get_filename_component(DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
    get_filename_component(INC1 "${DIR}/../../../io/include" REALPATH)
    get_filename_component(INC2 "${DIR}/../../../features/include" REALPATH)
    get_filename_component(INC3 "${DIR}/../../../kdtree/include" REALPATH)
    get_filename_component(INC4 "${DIR}/../../../visualization/include" REALPATH)
    get_filename_component(INC5 "${DIR}/../../../common/include" REALPATH)
    get_filename_component(INC6 "${DIR}/../../../search/include" REALPATH)
    get_filename_component(INC7 "${DIR}/../../../octree/include" REALPATH)
    
    
    #find_library(VTK)
    
    #set(INC6 "d:/svn/pcl_3rdparty/vtk-5.6.1/Common/" "d:/svn/pcl_3rdparty/vtk-5.6.1/bin/" "d:/svn/pcl_3rdparty/vtk-5.6.1/Rendering/")
    
    FILE(GLOB test_src *.cpp *.hpp)
    list(APPEND test_src ${pcl_gtest_sources})
    include_directories("${INC1}" "${INC2}" "${INC3}" "${INC4}" "${INC5}" "${INC6}" "${INC7}" ${VTK_INCLUDE_DIRS})
    #PCL_ADD_TEST(a_gpu_features_test ${the_test_target} FILES ${test_src} LINK_WITH pcl_gpu_containers pcl_gpu_octree pcl_gpu_features pcl_io pcl_features pcl_visualization pcl_common)
    
    #add_dependencies(${the_test_target} pcl_gpu_octree pcl_gpu_containes pcl_gpu_features pcl_io pcl_features pcl_visualization pcl_common)    
endif(BUILD_TESTS)

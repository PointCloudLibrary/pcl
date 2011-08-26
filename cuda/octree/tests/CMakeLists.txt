if(BUILD_TESTS)
    find_package(OpenCV)
    if (OpenCV_FOUND)

        get_filename_component(DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
        get_filename_component(INC ${DIR}/../../../octree/include REALPATH)
        include_directories(${INC} ${OpenCV_INCLUDE_DIRS})

        FILE(GLOB test_src *.cpp *.hpp)
        PCL_ADD_TEST(a_gpu_octree_test test_gpu_octree FILES ${test_src} LINK_WITH pcl_octree pcl_gpu_containers pcl_gpu_octree)

        target_link_libraries(test_gpu_octree ${OpenCV_LIBS})
        add_dependencies(test_gpu_octree pcl_gpu_octree pcl_octree pcl_gpu_containes)
    endif()
endif(BUILD_TESTS)

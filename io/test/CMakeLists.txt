if(BUILD_TESTS)
    if(OPENNI_FOUND)
        PCL_ADD_TEST(io_io test_io
                     FILES test_io.cpp
                     LINK_WITH pcl_io)
    endif(OPENNI_FOUND)

    PCL_ADD_TEST(io_iterators test_iterators 
                 FILES test_iterators.cpp
                 LINK_WITH pcl_io)
endif(BUILD_TESTS)


if(BUILD_TESTS)
    if(OPENNI_FOUND)
        add_executable(test_io test_io.cpp)
        target_link_libraries(test_io pcl_io ${GTEST_BOTH_LIBRARIES})
        add_test(io_io test_io)
    endif(OPENNI_FOUND)

    add_executable(test_iterators test_iterators.cpp)
    target_link_libraries(test_iterators pcl_io ${GTEST_BOTH_LIBRARIES})
    add_test(io_iterators test_iterators)
endif(BUILD_TESTS)


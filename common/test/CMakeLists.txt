if(BUILD_TESTS)
    add_executable(test_vector_average test_vector_average.cpp)
    PCL_ADD_OPENMP_FLAGS(test_vector_average)
    target_link_libraries(test_vector_average ${GTEST_BOTH_LIBRARIES}
        ${Boost_SYSTEM_LIBRARY})
    PCL_LINK_OPENMP(test_vector_average)
    add_test(common_vector_average test_vector_average)

    add_executable(test_common test_common.cpp)
    PCL_ADD_OPENMP_FLAGS(test_common)
    target_link_libraries(test_common ${GTEST_BOTH_LIBRARIES}
        ${Boost_THREAD_LIBRARY})
    PCL_LINK_OPENMP(test_common)
    add_test(common_common test_common)
endif(BUILD_TESTS)


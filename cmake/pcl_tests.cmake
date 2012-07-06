# Test management

# Need GTest to build the tests
#find_package(GTest)
#if(GTEST_FOUND)
#    set(REASON)
#    set(DEFAULT ON)
#else(GTEST_FOUND)
#    set(REASON "GTest was not found.")
#    set(DEFAULT OFF)
#endif(GTEST_FOUND)

# Add an option so the user can turn tests off
set(DEFAULT ON)
option(BUILD_TESTS "Build the library tests" ${DEFAULT})

# Print a suitable status message
if(NOT ${DEFAULT} AND NOT ${BUILD_TESTS})
    if(REASON)
        message(STATUS "Tests will not be built: ${REASON}")
    else(REASON)
        message(STATUS "Tests will not be built: Disabled manually")
    endif(REASON)
elseif(NOT ${BUILD_TESTS})
    message(STATUS "Tests will not be built: Disabled manually")
    #else(NOT ${DEFAULT} AND NOT ${BUILD_TESTS})
#    if(GTEST_FOUND)
#        message(STATUS "Tests will be built")
#    else(GTEST_FOUND)
#        set (BUILD_TESTS OFF CACHE BOOL "Tests can not be built!" FORCE)
#    endif(GTEST_FOUND)
endif(NOT ${DEFAULT} AND NOT ${BUILD_TESTS})

# Set up for testing if tests are enabled
if(BUILD_TESTS)
    enable_testing()
#    include_directories(${GTEST_INCLUDE_DIRS})
endif(BUILD_TESTS)



# If we would like to compile against a dynamically linked Boost
if(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32 AND WIN32)
    set(Boost_USE_STATIC_LIBS OFF)
    set(Boost_USE_STATIC OFF)
    set(Boost_USE_MULTITHREAD ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
else()
    if(NOT PCL_SHARED_LIBS OR WIN32)
        set(Boost_USE_STATIC_LIBS ON)
        set(Boost_USE_STATIC ON)
    endif()
endif()

# use static Boost in Windows
if(WIN32)
    set(Boost_USE_STATIC_LIBS @Boost_USE_STATIC_LIBS@)
    set(Boost_USE_STATIC @Boost_USE_STATIC@)
    set(Boost_USE_MULTITHREAD @Boost_USE_MULTITHREAD@)
endif()

set(Boost_ADDITIONAL_VERSIONS
        "1.71.0" "1.71" "1.70.0" "1.70"
        "1.69.0" "1.69" "1.68.0" "1.68" "1.67.0" "1.67" "1.66.0" "1.66" "1.65.1" "1.65.0" "1.65"
        "1.64.0" "1.64" "1.63.0" "1.63" "1.62.0" "1.62" "1.61.0" "1.61" "1.60.0" "1.60"
        "1.59.0" "1.59" "1.58.0" "1.58" "1.57.0" "1.57" "1.56.0" "1.56" "1.55.0" "1.55")

# Disable the config mode of find_package(Boost)
set(Boost_NO_BOOST_CMAKE ON)

# Required boost modules
set(BOOST_OPTIONAL_MODULES serialization mpi) # MPI needed?
set(BOOST_REQUIRED_MODULES filesystem date_time iostreams system)

# Temporarily clean out CMAKE_MODULE_PATH, so that we can pick up the built-in Find-module from CMake:
set( CMAKE_MODULE_PATH_BAK ${CMAKE_MODULE_PATH} )
set( CMAKE_MODULE_PATH )

# Call CMake's own Find-module
find_package(Boost 1.55.0  REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES} OPTIONAL_COMPONENTS ${BOOST_OPTIONAL_MODULES})

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH_BAK} )

if(Boost_SERIALIZATION_FOUND)
    set(BOOST_SERIALIZATION_FOUND TRUE)
endif()

if(Boost_FOUND)
    set(BOOST_FOUND TRUE)
    # Obtain diagnostic information about Boost's automatic linking outputted
    # during compilation time.
    add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
    include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()

set(BOOST_INCLUDE_DIRS "${Boost_INCLUDE_DIR}")
set(BOOST_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}")
set(BOOST_LIBRARIES ${Boost_LIBRARIES})
if(WIN32 AND NOT MINGW)
    set(BOOST_DEFINITIONS ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
endif()

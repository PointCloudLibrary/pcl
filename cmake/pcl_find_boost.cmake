# Find and set Boost flags

# If we would like to compile against a dynamically linked Boost
if(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32 AND WIN32)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_STATIC OFF)
  set(Boost_USE_MULTITHREAD ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
else(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32 AND WIN32)
  if(NOT PCL_SHARED_LIBS OR WIN32)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC ON)
  endif(NOT PCL_SHARED_LIBS OR WIN32)
endif(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32 AND WIN32)

if(${CMAKE_VERSION} VERSION_LESS 2.8.5)
  SET(Boost_ADDITIONAL_VERSIONS "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46.1" "1.46.0" "1.46" "1.47" "1.47.0")
else(${CMAKE_VERSION} VERSION_LESS 2.8.5)
  SET(Boost_ADDITIONAL_VERSIONS "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0")
endif(${CMAKE_VERSION} VERSION_LESS 2.8.5)

# Disable the config mode of find_package(Boost)
set(Boost_NO_BOOST_CMAKE ON)

# Optional boost modules
find_package(Boost 1.47.0 QUIET COMPONENTS serialization mpi)
if(Boost_MPI_FOUND)
  set(BOOST_MPI_FOUND TRUE)
endif(Boost_MPI_FOUND)
if(Boost_SERIALIZATION_FOUND)
  set(BOOST_SERIALIZATION_FOUND TRUE)
endif(Boost_SERIALIZATION_FOUND)

# Required boost modules
set(BOOST_REQUIRED_MODULES system filesystem thread date_time iostreams chrono)

find_package(Boost 1.47.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})

if(Boost_FOUND)
  set(BOOST_FOUND TRUE)
  # Obtain diagnostic information about Boost's automatic linking outputted 
  # during compilation time.
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

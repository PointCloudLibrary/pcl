# Find and set Boost flags

if(NOT PCL_SHARED_LIBS OR WIN32)
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_STATIC ON)
endif(NOT PCL_SHARED_LIBS OR WIN32)

if(${CMAKE_VERSION} VERSION_LESS 2.8.5)
  SET(Boost_ADDITIONAL_VERSIONS "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46.1" "1.46.0" "1.46" "1.47" "1.47.0")
else(${CMAKE_VERSION} VERSION_LESS 2.8.5)
  SET(Boost_ADDITIONAL_VERSIONS "1.47" "1.47.0")
endif(${CMAKE_VERSION} VERSION_LESS 2.8.5)

# Disable the config mode of find_package(Boost)
set(Boost_NO_BOOST_CMAKE ON)

# Required boost modules
find_package(Boost 1.40.0 REQUIRED COMPONENTS system filesystem thread date_time iostreams)

if(Boost_FOUND)
  set(BOOST_FOUND TRUE)
  # Obtain diagnostic information about Boost's automatic linking outputted 
  # during compilation time.
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

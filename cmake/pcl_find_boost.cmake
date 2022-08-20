# Find and set Boost flags

# If we would like to compile against a dynamically linked Boost
if(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32 AND WIN32)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_STATIC OFF)
  set(Boost_USE_MULTITHREAD ON)
  string(APPEND CMAKE_CXX_FLAGS " -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
else()
  if(NOT PCL_SHARED_LIBS OR WIN32)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC ON)
  endif()
endif()

set(Boost_ADDITIONAL_VERSIONS
  "1.80.0" "1.80"
  "1.79.0" "1.79" "1.78.0" "1.78" "1.77.0" "1.77" "1.76.0" "1.76" "1.75.0" "1.75" 
  "1.74.0" "1.74" "1.73.0" "1.73" "1.72.0" "1.72" "1.71.0" "1.71" "1.70.0" "1.70"
  "1.69.0" "1.69" "1.68.0" "1.68" "1.67.0" "1.67" "1.66.0" "1.66" "1.65.1" "1.65.0" "1.65")

# Optional boost modules
find_package(Boost 1.65.0 QUIET COMPONENTS serialization mpi)
if(Boost_SERIALIZATION_FOUND)
  set(BOOST_SERIALIZATION_FOUND TRUE)
endif()

# Required boost modules
set(BOOST_REQUIRED_MODULES filesystem date_time iostreams system)
find_package(Boost 1.65.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})

if(Boost_FOUND)
  set(BOOST_FOUND TRUE)
endif()

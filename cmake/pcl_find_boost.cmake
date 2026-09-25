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

# Optional boost modules (filesystem no longer needed - using std::filesystem with C++17)
set(BOOST_OPTIONAL_MODULES serialization mpi)
# Required boost modules
set(BOOST_REQUIRED_MODULES iostreams)

find_package(Boost 1.71.0 QUIET COMPONENTS ${BOOST_OPTIONAL_MODULES} CONFIG)
find_package(Boost 1.71.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES} CONFIG)

if(Boost_FOUND)
  set(BOOST_FOUND TRUE)
endif()

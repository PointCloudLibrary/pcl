# Find and set Boost flags

if(NOT PCL_SHARED_LIBS OR WIN32)
  set(Boost_USE_STATIC_LIBS ON)
endif(NOT PCL_SHARED_LIBS OR WIN32)

find_package(Boost 1.40.0 REQUIRED COMPONENTS system filesystem thread date_time)

if(Boost_FOUND)
  # Obtain diagnostic information about Boost's automatic linking outputted 
  # during compilation time.
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
  
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

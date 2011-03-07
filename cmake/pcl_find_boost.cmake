# Find and set Boost flags

if(NOT PCL_SHARED_LIBS)
    set(Boost_USE_STATIC_LIBS ON)
endif(NOT PCL_SHARED_LIBS)

find_package(Boost 1.36.0 COMPONENTS system filesystem)
# TODO: What should the minimum version number be?
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


# Find and set Boost flags

if(NOT PCL_SHARED_LIBS)
    set(Boost_USE_STATIC_LIBS ON)
endif(NOT PCL_SHARED_LIBS)

find_package(Boost 1.44.0 COMPONENTS system filesystem thread)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

foreach(boost_component ${Boost_LIBRARIES})
  string(REGEX MATCH "boost_.*" component "${boost_component}")
  if(NOT "${component}" EQUAL "")
    string(REGEX REPLACE "(boost_.*)\\..*" "\\1" component "${component}")
    list(APPEND boost_components ${component})
  endif(NOT "${component}" EQUAL "")
endforeach(boost_component)

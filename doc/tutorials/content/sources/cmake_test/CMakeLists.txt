cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# This is not really a tutorial, but instead tests the behaviour of find_package(PCL) when built together with the other tutorials
project(cmake_test)

set(BOOST_LIBRARIES "boost_dont_overwrite")
set(Boost_LIBRARIES "boost_dont_overwrite")
find_package(PCL REQUIRED)

if(NOT "${BOOST_LIBRARIES}" STREQUAL "boost_dont_overwrite")
  message(FATAL_ERROR "find_package(PCL) changed the value of BOOST_LIBRARIES")
endif()
if(NOT "${Boost_LIBRARIES}" STREQUAL "boost_dont_overwrite")
  message(FATAL_ERROR "find_package(PCL) changed the value of Boost_LIBRARIES")
endif()

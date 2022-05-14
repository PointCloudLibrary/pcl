# This file is not processed if WITH_QT evaluates to a CMake false constant.
#
# First we convert WITH_QT to WITH_QT_STR with
# - CMake true constants to string YES (except numbers unequal 1)
# - Other values to upper case string
#
# Background: WITH_QT was previously boolean and we want to be backwards compatible.
#
# This if condition matches all CMake true constants (`${WITH_QT}`) except numbers other than 1 (`NOT (WITH_QT LESS 1 OR WITH_QT GREATER 1)`).
#
# This way we prevent things like -DWITH_QT=5 to be handled as YES instead of QT5.
# Setting -DWITH_QT=5 will error and inform you to use -DWITH_QT=QT5 instead.
# (This breaks backward compatibility, but hopefully no one will use values not equal to 1 if they want to express true.)
#
# Note: "NOT (WITH_QT LESS 1 OR WITH_QT GREATER 1)" is not the same as
#       "WITH_QT EQUAL 1" because "LESS/GREATER/EQUAL" all pre-check if WITH_QT is a valid number.
if(${WITH_QT} AND NOT (WITH_QT LESS 1 OR WITH_QT GREATER 1))
  set(WITH_QT_STR "YES")
else()
  string(TOUPPER ${WITH_QT} WITH_QT_STR)
endif()

if(NOT WITH_QT_STR MATCHES "^(AUTO|YES|QT6|QT5)$")
  message(FATAL_ERROR "Option WITH_QT must be one of AUTO|YES|QT6|QT5|NO but is '${WITH_QT}'")
endif()

if(WITH_QT_STR MATCHES "^(AUTO|YES|QT6)$")
  find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets)
  set(QT6_FOUND ${Qt6_FOUND})
  set(QT_FOUND ${QT6_FOUND})
  if (QT6_FOUND)
    set(QTX Qt6)
  endif()
endif()

if(WITH_QT_STR MATCHES "^(AUTO|YES|QT5)$" AND NOT QT6_FOUND)
  find_package(Qt5 5.9.5 QUIET COMPONENTS Concurrent OpenGL Widgets)
  set(QT5_FOUND ${Qt5_FOUND})
  set(QT_FOUND ${QT5_FOUND})
  if(QT5_FOUND)
    set(QTX Qt5)
  endif()
endif()

if(NOT WITH_QT_STR MATCHES "^(AUTO)$" AND NOT QT_FOUND)
  message(FATAL_ERROR "Can not find Qt required by WITH_QT=${WITH_QT}.")
endif()

if(NOT QT_FOUND)
  message(STATUS "Qt is not found.")
  return()
endif()

set(QT_VERSION ${${QTX}_VERSION})
message(STATUS "Qt version: ${QT_VERSION}")

set(QT_DISABLE_PRECATED_BEFORE_VAL "0x050900")

#Set Cmake Auto features to skip .hh files
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 OLD)
endif()

#If building CUDA required libraries
#Change ${QTX}::Core fixed -fPIC flags to conditionally only CXX
#TODO: To be removed when QT is >5.14.1
if(BUILD_CUDA OR BUILD_GPU)
  if(${QTX}Widgets_VERSION VERSION_LESS 5.14.1)
    get_property(core_options TARGET ${QTX}::Core PROPERTY INTERFACE_COMPILE_OPTIONS)
    string(REPLACE "-fPIC" "$<IF:$<COMPILE_LANGUAGE:CXX>,-fPIC,>"  new_core_options ${core_options})
    set_property(TARGET ${QTX}::Core PROPERTY INTERFACE_COMPILE_OPTIONS ${new_core_options})
  endif()
endif()

get_property(core_def TARGET ${QTX}::Core PROPERTY INTERFACE_COMPILE_DEFINITIONS)
list(APPEND core_def "QT_DISABLE_DEPRECATED_BEFORE=${QT_DISABLE_PRECATED_BEFORE_VAL}")
set_property(TARGET ${QTX}::Core PROPERTY INTERFACE_COMPILE_DEFINITIONS ${core_def})

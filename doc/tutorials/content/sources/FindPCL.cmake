# Try to find PCL components
#
# The follwoing variables are optionally searched for defaults
#  PCL_ROOT:             Base directory of PCL tree to use.
#  $ENV{PCL_ROOT}:       Base directory of PCL tree to use.
#
# The following are set after configuration is done: 
#  PCL_FOUND
#  PCL_INCLUDE_DIRS
#  PCL_LIBRARIES
#  PCL_LIBRARY_DIRS
#  PCL_VERSION
#  PCL_DEFINITIONS
#
# In addition for each component these are set:
#  PCL_COMPONENT_INCLUDE_DIR
#  PCL_COMPONENT_LIBRARY
#  PCL_COMPONENT_LIBRARY_DEBUG
#  PCL_COMPONENT_LIBRARIES
#  PCL_COMPONENT_DEFINITIONS if available
# 
# To use PCL from within you code
# find_package(PCL [VERSION] [QUIET] [REQUIRED] [COMPONENTS module1 module2 ...])
# if(PCL_FOUND)
#   include_directories(${PCL_INCLUDE_DIRS})
#   list(APPEND LIBS ${PCL_LBRARIES})
# endif(PCL_FOUND)
# Or if you want to link against a particular module
# target_link_libraries(my_fabulous_target ${PCL_XXX_LIBRARIES}) where XXX is 
# module from COMPONENTS
#
# Tested with:
# -PCL 1.0
#
# www.pointclouds.org
# --------------------------------

include(FindPackageHandleStandardArgs)
#set a suffix based on project name and version
set(PCL_SUFFIX pcl-1.0)
#set a suffix for debug libraries
set(PCL_DEBUG_SUFFIX "-gd")

#set all pcl component and their account into variables
set(pcl_all_components io common kdtree keypoints filters range_image registration sample_consensus segmentation features surface octree visualization )
list(LENGTH pcl_all_components PCL_NB_COMPONENTS)

#list each component dependencies IN PCL
set(pcl_features_int_dep common kdtree range_image)
set(pcl_filters_int_dep common sample_consensus io kdtree)
set(pcl_io_int_dep common octree)
set(pcl_kdtree_int_dep common)
set(pcl_keypoints_int_dep common features io kdtree range_image)
set(pcl_range_image_int_dep common)
set(pcl_range_image_border_extractor_int_dep common kdtree range_image)
set(pcl_registration_int_dep common io kdtree sample_consensus)
set(pcl_sample_consensus_int_dep common)
set(pcl_segmentation_int_dep common kdtree sample_consensus)
set(pcl_surface_int_dep common io features kdtree registration)
set(pcl_octree_int_dep common)
set(pcl_visualization_int_dep common io kdtree octree range_image octree filters)
#list each component external dependencies (ext means mandatory and opt means optional)
set(pcl_common_ext_dep eigen)
set(pcl_features_ext_dep eigen)
set(pcl_io_opt_dep openni-dev)
set(pcl_kdtree_ext_dep flann)
set(pcl_range_image_ext_dep eigen)
set(pcl_range_image_border_extractor_ext_dep eigen)
set(pcl_registration_ext_dep eigen)
set(pcl_sample_consensus_ext_dep cminpack)
set(pcl_surface_opt_dep qhull eigen)
set(pcl_visualization_ext_dep VTK eigen)
set(pcl_visualization_opt_dep wxWidgets)

#if the component doesn't lie in a directory with the same name
set(range_image_border_extractor_parent features)

set(PCL_INCLUDE_DIRS)
set(PCL_LIBRARY_DIRS)
set(PCL_LIBRARIES)
set(PCL_DEFINITIONS)

set(PCL_FOUND_LIBRARIES)

if(PCL_FIND_QUIETLY)
  set(QUIET_ QUIET)
else(PCL_FIND_QUIETLY)
  set(QUIET_)
endif(PCL_FIND_QUIETLY)

macro(pcl_report_not_found _reason)
  unset(PCL_FOUND)
  unset(PCL_LIBRARIES)
  unset(PCL_INCLUDE_DIRS)
  unset(PCL_LIBRARY_DIRS)
  unset(PCL_DEFINITONS)
  if(PCL_FIND_REQUIRED)
    message(FATAL_ERROR ${_reason})
  elseif(NOT PCL_FIND_QUIETLY)
    message(SEND_ERROR ${_reason})
  endif()
  return()
endmacro(pcl_report_not_found)

macro(pcl_message)
  if(NOT PCL_FIND_QUIETLY)
    message(${ARGN})
  endif(NOT PCL_FIND_QUIETLY)
endmacro(pcl_message)

#remove this as soon as eigen is shipped with FindEigen.cmake
macro(find_eigen)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_EIGEN eigen3)
  endif(PkgConfig_FOUND)
  find_path(EIGEN_INCLUDE_DIRS Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS} 
          "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/Eigen 3.0.0" "$ENV{PROGRAMW6432}/Eigen 3.0.0"
          "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"
    PATH_SUFFIXES eigen3 include/eigen3 include)
  find_package_handle_standard_args(eigen DEFAULT_MSG EIGEN_INCLUDE_DIRS)
endmacro(find_eigen)

#remove this as soon as cminpack is shipped with FindCminpack.cmake
macro(find_cminpack)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_CMINPACK cminpack)
  endif(PkgConfig_FOUND)
  find_path(CMINPACK_INCLUDE_DIRS cminpack.h
    HINTS ${PC_CMINPACK_INCLUDEDIR} ${PC_CMINPACK_INCLUDE_DIRS} 
          "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}" 
          "$ENV{PROGRAMFILES}/CMINPACK 1.1.3" "$ENV{PROGRAMW6432}/CMINPACK 1.1.3" 
          "$ENV{PROGRAMFILES}/CMINPACK" "$ENV{PROGRAMW6432}/CMINPACK" 
    PATH_SUFFIXES include/cminpack-1)

  # Most likely we are on windows so prefer static libraries over shared ones
  find_library(CMINPACK_LIBRARY 
    NAMES cminpack_s cminpack
    HINTS ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS} 
          "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/CMINPACK 1.1.3" "$ENV{PROGRAMW6432}/CMINPACK 1.1.3" 
          "$ENV{PROGRAMFILES}/CMINPACK" "$ENV{PROGRAMW6432}/CMINPACK"
    PATH_SUFFIXES lib)
  
  find_library(CMINPACK_LIBRARY_DEBUG 
    NAMES cminpack_s-gd cminpack-gd cminpack_s cminpack
    HINTS ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS} 
          "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/CMINPACK 1.1.3" "$ENV{PROGRAMW6432}/CMINPACK 1.1.3" 
          "$ENV{PROGRAMFILES}/CMINPACK" "$ENV{PROGRAMW6432}/CMINPACK"
    PATH_SUFFIXES lib)

  if(NOT CMINPACK_LIBRARY_DEBUG)
    set(CMINPACK_LIBRARY_DEBUG ${CMINPACK_LIBRARY})
  endif(NOT CMINPACK_LIBRARY_DEBUG)

  set(CMINPACK_INCLUDE_DIRS ${CMINPACK_INCLUDE_DIR})
  set(CMINPACK_LIBRARIES optimized ${CMINPACK_LIBRARY} debug ${CMINPACK_LIBRARY_DEBUG})
  
  find_package_handle_standard_args(cminpack DEFAULT_MSG CMINPACK_LIBRARY CMINPACK_INCLUDE_DIRS)
  if(CMINPACK_FOUND)
    get_filename_component(cminpack_lib ${CMINPACK_LIBRARY} NAME_WE)
    if("${cminpack_lib}" STREQUAL "cminpack_s")
      set(IS_CMINPACK_STATIC ON)
    endif("${cminpack_lib}" STREQUAL "cminpack_s")
    get_filename_component(CMINPACK_LIBRARY_PATH ${CMINPACK_LIBRARY} PATH)
    get_filename_component(CMINPACK_LIBRARY_DEBUG_PATH ${CMINPACK_LIBRARY_DEBUG} PATH)
    set(CMINPACK_LIBRARY_DIRS ${CMINPACK_LIBRARY_PATH} ${CMINPACK_LIBRARY_DEBUG_PATH})
  endif(CMINPACK_FOUND)
endmacro(find_cminpack)

#remove this as soon as qhull is shipped with FindQhull.cmake
macro(find_qhull)
  set(QHULL_MAJOR_VERSION 6)

  find_path(QHULL_INCLUDE_DIRS
    NAMES libqhull/libqhull.h qhull.h
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          "$ENV{PROGRAMFILES}/qhull" "$ENV{PROGRAMW6432}/qhull" 
    PATH_SUFFIXES qhull src/libqhull libqhull include)

  # Most likely we are on windows so prefer static libraries over shared ones (Mourad's recommend)
  find_library(QHULL_LIBRARY 
    NAMES qhullstatic qhull qhull${QHULL_MAJOR_VERSION}
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          "$ENV{PROGRAMFILES}/qhull" "$ENV{PROGRAMW6432}/qhull" 
    PATH_SUFFIXES project build bin lib)
  
  find_library(QHULL_LIBRARY_DEBUG 
    NAMES qhullstatic_d qhull_d qhull_d${QHULL_MAJOR_VERSION} 
          qhull qhull${QHULL_MAJOR_VERSION}
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          "$ENV{PROGRAMFILES}/qhull" "$ENV{PROGRAMW6432}/qhull" 
    PATH_SUFFIXES project build bin lib)
  
  if(NOT QHULL_LIBRARY_DEBUG)
    set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
  endif(NOT QHULL_LIBRARY_DEBUG)

  set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})

  find_package_handle_standard_args(qhull DEFAULT_MSG QHULL_LIBRARY QHULL_INCLUDE_DIRS)

  if(QHULL_FOUND)
    get_filename_component(QHULL_LIBRARY_PATH ${QHULL_LIBRARY} PATH)
    get_filename_component(QHULL_LIBRARY_DEBUG_PATH ${QHULL_LIBRARY_DEBUG} PATH)
    set(QHULL_LIBRARY_DIRS ${QHULL_LIBRARY_PATH} ${QHULL_LIBRARY_DEBUG_PATH}) 
  endif(QHULL_FOUND)
endmacro(find_qhull)

#remove this as soon as openni-dev is shipped with FindOpenni.cmake
macro(find_openni)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_OPENNI-DEV openni-dev)
  endif(PkgConfig_FOUND)
  find_path(OPENNI-DEV_INCLUDE_DIRS XnStatus.h
    HINTS ${PC_OPENNI-DEV_INCLUDEDIR} ${PC_OPENNI-DEV_INCLUDE_DIRS} 
          "${OPENNI-DEV_ROOT}" "$ENV{OPENNI-DEV_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/OpenNI/Include" "$ENV{PROGRAMW6432}/OpenNI/Include"
    PATH_SUFFIXES include/openni Include)
  #add a hint so that it can find it without the pkg-config
  find_library(OPENNI-DEV_LIBRARY 
    NAMES OpenNI64 OpenNI 
    HINTS ${PC_OPENNI-DEV_LIBDIR} ${PC_OPENNI-DEV_LIBRARY_DIRS} 
          "${OPENNI-DEV_ROOT}" "$ENV{OPENNI-DEV_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/OpenNI/Lib" "$ENV{PROGRAMW6432}/OpenNI/Lib64"
    PATH_SUFFIXES lib Lib Lib64)

  find_package_handle_standard_args(openni-dev DEFAULT_MSG OPENNI-DEV_LIBRARY OPENNI-DEV_INCLUDE_DIRS)

  if(OPENNI-DEV_FOUND)
    get_filename_component(OPENNI-DEV_LIBRARY_PATH ${OPENNI-DEV_LIBRARY} PATH)
    set(OPENNI-DEV_LIBRARY_DIRS ${OPENNI-DEV_LIBRARY_PATH}) 
  endif(OPENNI-DEV_FOUND)    
endmacro(find_openni)

#remove this as soon as flann is shipped with FindFlann.cmake
macro(find_flann)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_FLANN flann)
  endif(PkgConfig_FOUND)

  find_path(FLANN_INCLUDE_DIRS flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS} 
          "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9"
          "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES include)

  find_library(FLANN_LIBRARY
    NAMES flann_cpp_s flann_cpp
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
          "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES lib)

  find_library(FLANN_LIBRARY_DEBUG 
    NAMES flann_cpp_s-gd flann_cpp-gd flann_cpp_s flann_cpp
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT} $ENV{FLANN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
          "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES lib)
  if(NOT FLANN_LIBRARY_DEBUG)
    set(FLANN_LIBRARY_DEBUG ${FLANN_LIBRARY})
  endif(NOT FLANN_LIBRARY_DEBUG)

  find_package_handle_standard_args(Flann DEFAULT_MSG FLANN_LIBRARY FLANN_INCLUDE_DIRS)
  if(FLANN_FOUND)
    get_filename_component(flann_lib ${FLANN_LIBRARY} NAME_WE)
    if("${flann_lib}" STREQUAL "flann_cpp_s")
      set(IS_FLANN_STATIC ON)
    endif("${flann_lib}" STREQUAL "flann_cpp_s")
    get_filename_component(FLANN_LIBRARY_PATH ${FLANN_LIBRARY} PATH)
    get_filename_component(FLANN_LIBRARY_DEBUG_PATH ${FLANN_LIBRARY_DEBUG} PATH)
    set(FLANN_LIBRARY_DIRS ${FLANN_LIBRARY_PATH} ${FLANN_LIBRARY_DEBUG_PATH}) 
  endif(FLANN_FOUND)
endmacro(find_flann)

macro(find_wxWidgets)
  find_package(wxWidgets ${QUIET_})
endmacro(find_wxWidgets)

macro(find_VTK)
  find_package(VTK ${QUIET_})
endmacro(find_VTK)

# Finds each component external libraries if any
# The functioning is as following
# try to find _lib
# |--> _lib found ==> include the headers,
#                     link to its library directories or include _lib_USE_FILE
# ---> _lib not found
#                   |--> _lib is optional ==> disable it (thanks to the guardians) 
#                   |                         and warn
#                   ---> _lib is required
#                                       |--> component is required explicitely ==> error
#                                       ---> component is induced ==> warn and remove it
#                                                                     from the list

macro(find_external_library _component _lib _is_optional)
  if("${_lib}" STREQUAL "eigen")
    find_eigen()
  elseif("${_lib}" STREQUAL "flann")
    find_flann()
  elseif("${_lib}" STREQUAL "qhull")
    find_qhull()
  elseif("${_lib}" STREQUAL "openni-dev")
    find_openni()
  elseif("${_lib}" STREQUAL "cminpack")
    find_cminpack()
  elseif("${_lib}" STREQUAL "VTK")
    find_VTK()
  elseif("${_lib}" STREQUAL "wxWidgets")
    find_wxWidgets()
  endif("${_lib}" STREQUAL "eigen")

  string(TOUPPER "${_lib}" LIB)
  if(${LIB}_FOUND)
    list(APPEND PCL_INCLUDE_DIRS "${${LIB}_INCLUDE_DIRS}")
    if(${_lib}_USE_FILE)
      include(${${_lib}_USE_FILE})
    else(${_lib}_USE_FILE)
      list(APPEND PCL_LIBRARY_DIRS "${${LIB}_LIBRARY_DIRS}")
    endif(${_lib}_USE_FILE)
  else(${LIB}_FOUND)
    if("${_is_optional}" STREQUAL "OPTIONAL")
      add_definitions("-DDISABLE_${LIB}")
      pcl_message("** WARNING ** ${_component} features related to ${_lib} will be disabled")
    elseif("${_is_optional}" STREQUAL "REQUIRED")
      if((NOT PCL_FIND_ALL) OR (PCL_FIND_ALL EQUAL 1))
        pcl_report_not_found("${_component} is required but ${_lib} was not found")
      elseif(PCL_FIND_ALL EQUAL 0)
        # raise error and remove _component from PCL_TO_FIND_COMPONENTS
        string(TOUPPER "${_component}" COMPONENT)
        pcl_message("** WARNING ** ${_component} will be disabled cause ${_lib} was not found")
        list(REMOVE_ITEM PCL_TO_FIND_COMPONENTS ${_component})
        # set(PCL_${COMPONENT}_LIBRARIES PCL_${COMPONENT}_LIBRARIES-NOTFOUND)
        # set(PCL_${COMPONENT}_LIBRARY PCL_${COMPONENT}_LIBRARY-NOTFOUND)
        # set(PCL_${COMPONENT}_LIBRARY_DEBUG PCL_${COMPONENT}_LIBRARY_DEBUG-NOTFOUND)
        # set(PCL_${COMPONENT}_INCLUDE_DIR PCL_${COMPONENT}_INCLUDE_DIR-NOTFOUND)
        # unset(PCL_${COMPONENT}_FOUND)
      endif((NOT PCL_FIND_ALL) OR (PCL_FIND_ALL EQUAL 1))
    endif("${_is_optional}" STREQUAL "OPTIONAL")
  endif(${LIB}_FOUND)
endmacro(find_external_library)

macro(pcl_check_external_dependency _component)
endmacro(pcl_check_external_dependency)

#flatten dependencies recursivity is great \o/
macro(compute_dependencies TO_FIND_COMPONENTS)
  foreach(component ${${TO_FIND_COMPONENTS}})
    set(pcl_component pcl_${component})
    if(${pcl_component}_int_dep AND (NOT PCL_FIND_ALL))
      foreach(dependency ${${pcl_component}_int_dep})
        list(FIND ${TO_FIND_COMPONENTS} ${component} pos)
        list(FIND ${TO_FIND_COMPONENTS} ${dependency} found)
        if(found EQUAL -1)
          set(pcl_dependency pcl_${dependency})
          if(${pcl_dependency}_int_dep)
            list(INSERT ${TO_FIND_COMPONENTS} ${pos} ${dependency})
            if(pcl_${dependency}_ext_dep)
              list(APPEND pcl_${component}_ext_dep ${pcl_${dependency}_ext_dep})
            endif(pcl_${dependency}_ext_dep)
            if(pcl_${dependency}_opt_dep)
              list(APPEND pcl_${component}_opt_dep ${pcl_${dependency}_opt_dep})
            endif(pcl_${dependency}_opt_dep)
            compute_dependencies(${TO_FIND_COMPONENTS})
          else(${pcl_dependency}_int_dep)
            list(INSERT ${TO_FIND_COMPONENTS} 0 ${dependency})
          endif(${pcl_dependency}_int_dep)
        endif(found EQUAL -1)
      endforeach(dependency)
    endif(${pcl_component}_int_dep AND (NOT PCL_FIND_ALL))
  endforeach(component)
endmacro(compute_dependencies)

#check if user provided a list of components
#if no components at all or full list is given set PCL_FIND_ALL
if(PCL_FIND_COMPONENTS)
  list(LENGTH PCL_FIND_COMPONENTS PCL_FIND_COMPONENTS_LENGTH)
  if(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
    set(PCL_TO_FIND_COMPONENTS ${pcl_all_components})
    set(PCL_FIND_ALL 1)
  else(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
    set(PCL_TO_FIND_COMPONENTS ${PCL_FIND_COMPONENTS})    
  endif(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
else(PCL_FIND_COMPONENTS)
  set(PCL_TO_FIND_COMPONENTS ${pcl_all_components})
  set(PCL_FIND_ALL 1)
endif(PCL_FIND_COMPONENTS)

#if boost is unavailable just give up
find_package(Boost 1.40.0 ${QUIET_} COMPONENTS system filesystem thread date_time)
if(Boost_FOUND)
  list(APPEND PCL_INCLUDE_DIRS "${Boost_INCLUDE_DIRS}")
  list(APPEND PCL_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}")
else(Boost_FOUND)
  pcl_report_not_found("Boost was not found, PCL won't work properly")
endif(Boost_FOUND)

find_file(PCL_CONFIG_H pcl/pcl_config.h
  HINTS /usr /usr/local /opt/local /opt
  "$ENV{PROGRAMFILES}/PCL" "$ENV{PROGRAMW6432}/PCL" "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}"
  ${${PC_PCL_COMPONENT}_INCLUDEDIR} ${${PC_PCL_COMPONENT}_INCLUDE_DIRS} 
  ${PCL_ROOT} $ENV{PCL_ROOT}
  PATH_SUFFIXES include/${PCL_SUFFIX}
  DOC "path to pcl_config.h header")

if(NOT EXISTS ${PCL_CONFIG_H})
  pcl_report_not_found("PCL can not be found on this machine")
endif(NOT EXISTS ${PCL_CONFIG_H})

# we did find pcl_config.h
# extract PCL_VERSION
file(STRINGS ${PCL_CONFIG_H} PCL_FOUND_VERSION REGEX "PCL_VERSION *.*")
if("${PCL_FOUND_VERSION}" STREQUAL "")
  pcl_report_not_found("File ${PCL_CONFIG_H} is corrupt")
else("${PCL_FOUND_VERSION}" STREQUAL "")
  string(REGEX REPLACE "#.*define.*PCL_VERSION " "" PCL_FOUND_VERSION "${PCL_FOUND_VERSION}")
  string(REPLACE "\"" "" PCL_FOUND_VERSION "${PCL_FOUND_VERSION}")
  if(${PCL_FOUND_VERSION} VERSION_GREATER 0.9)
    get_filename_component(PCL_DIR ${PCL_CONFIG_H} PATH)
    get_filename_component(PCL_DIR ${PCL_DIR} PATH)
  else(${PCL_FOUND_VERSION} VERSION_GREATER 0.9)
    report_not_found("Found unappropriate version ${PCL_FOUND_VERSION}")
  endif(${PCL_FOUND_VERSION} VERSION_GREATER 0.9)
endif("${PCL_FOUND_VERSION}" STREQUAL "")

#require pkgconfig if available
#find_package(PkgConfig)

compute_dependencies(PCL_TO_FIND_COMPONENTS)

# compute external dependencies per component
foreach(component ${PCL_TO_FIND_COMPONENTS})
  if(NOT PkgConfig_FOUND)
    foreach(opt ${pcl_${component}_opt_dep})
      find_external_library(${component} ${opt} OPTIONAL)
    endforeach(opt)
    foreach(ext ${pcl_${component}_ext_dep})
      find_external_library(${component} ${ext} REQUIRED)
    endforeach(ext) 
  endif(NOT PkgConfig_FOUND)
endforeach(component)

foreach(component ${PCL_TO_FIND_COMPONENTS})
  set(pcl_component pcl_${component})
  string(TOUPPER "pcl_${component}" PCL_COMPONENT)
  string(TOUPPER "PC_${pcl_component}" PC_PCL_COMPONENT)
  string(TOUPPER "${pcl_component}_DEFINITIONS" PCL_COMPONENT_DEFINITIONS)
  string(TOUPPER "${pcl_component}_INCLUDE_DIR" PCL_COMPONENT_INCLUDE_DIR)
  string(TOUPPER "${pcl_component}_LIBRARY" PCL_COMPONENT_LIBRARY)

  pcl_message(STATUS "looking for ${PCL_COMPONENT}")

  if(PkgConfig_FOUND)
    pkg_check_modules(${PC_PCL_COMPONENT} QUIET ${pcl_component})
    set(${PCL_COMPONENT_DEFINITIONS} ${${PC_PCL_COMPONENT}_CFLAGS_OTHER})
    mark_as_advanced(${PCL_COMPONENT_DEFINITIONS})
  endif(PkgConfig_FOUND)
  if(${PC_PCL_COMPONENT}_VERSION)
    if(NOT PCL_FOUND_VERSION VERSION_EQUAL ${PC_PCL_COMPONENT}_VERSION)
      pcl_report_not_found("PCL component: ${component} should be at version ${PCL_FOUND_VERSION}")
    endif(NOT PCL_FOUND_VERSION VERSION_EQUAL ${PC_PCL_COMPONENT}_VERSION)
  endif(${PC_PCL_COMPONENT}_VERSION)

  find_path(${PCL_COMPONENT_INCLUDE_DIR}
    NAMES pcl/${component} pcl/${${component}_parent}
    HINTS ${PCL_DIR}
    PATHS /usr /usr/local
    "$ENV{PROGRAMFILES}/PCL" "$ENV{PROGRAMW6432}/PCL"
    ${${PC_PCL_COMPONENT}_INCLUDEDIR} ${${PC_PCL_COMPONENT}_INCLUDE_DIRS}
    PATH_SUFFIXES include/${PCL_SUFFIX}
    DOC "path to ${component} headers")

  get_filename_component(component_header_path ${${PCL_COMPONENT_INCLUDE_DIR}} PATH)

  #check that the found path is same as found PCL_DIR 
  if(NOT "${${PCL_COMPONENT_INCLUDE_DIR}}" STREQUAL "${PCL_DIR}")
    pcl_report_not_found("${${PCL_COMPONENT_INCLUDE_DIR}} is not a valid include directory")
  endif(NOT "${${PCL_COMPONENT_INCLUDE_DIR}}" STREQUAL "${PCL_DIR}")

  find_library(${PCL_COMPONENT_LIBRARY} ${pcl_component}
    HINTS ${PCL_DIR} "$ENV{PROGRAMFILES}/PCL" "$ENV{PROGRAMW6432}/PCL"
    ${${PC_PCL_COMPONENT}_LIBDIR} ${${PC_PCL_COMPONENT}_LIBRARY_DIRS}
    "${PCL_ROOT}" "$ENV{PCL_ROOT}"
    PATH_SUFFIXES lib bin
    DOC "path to ${pcl_component} library")
  get_filename_component(${component}_library_path 
    ${${PCL_COMPONENT_LIBRARY}}
    PATH)

  find_library(${PCL_COMPONENT_LIBRARY}_DEBUG ${pcl_component}${PCL_DEBUG_SUFFIX}
    HINTS ${${component}_library_path} /usr/local /opt /opt/local
    "$ENV{PROGRAMFILES}/PCL" "$ENV{PROGRAMW6432}/PCL"
    ${${PC_PCL_COMPONENT}_LIBDIR} ${${PC_PCL_COMPONENT}_LIBRARY_DIRS}
    "${PCL_ROOT}" "$ENV{PCL_ROOT}"
    PATH_SUFFIXES lib bin
    DOC "path to ${pcl_component} library debug")
  # if only release found let release be debug too
  # NOTE: this is only for non Makefile sake
  if(NOT ${PCL_COMPONENT_LIBRARY}_DEBUG AND ${PCL_COMPONENT_LIBRARY})
    set(${PCL_COMPONENT_LIBRARY}_DEBUG ${${PCL_COMPONENT_LIBRARY}})
  endif(NOT ${PCL_COMPONENT_LIBRARY}_DEBUG AND ${PCL_COMPONENT_LIBRARY})

  get_filename_component(${component}_library_path_debug 
    ${${PCL_COMPONENT_LIBRARY}_DEBUG}
    PATH)
  
  find_package_handle_standard_args(${PCL_COMPONENT} DEFAULT_MSG
    ${PCL_COMPONENT_LIBRARY} ${PCL_COMPONENT_INCLUDE_DIR})

  if(${PCL_COMPONENT}_FOUND)
    set(${PCL_COMPONENT}_LIBRARIES optimized ${${PCL_COMPONENT_LIBRARY}} debug ${${PCL_COMPONENT_LIBRARY}_DEBUG})
    list(APPEND PCL_INCLUDE_DIRS ${${PCL_COMPONENT_INCLUDE_DIR}})
    if(${PC_PCL_COMPONENT}_INCLUDE_DIRS)
      list(APPEND PCL_INCLUDE_DIRS ${${PC_PCL_COMPONENT}_INCLUDE_DIRS})
    endif(${PC_PCL_COMPONENT}_INCLUDE_DIRS)
    mark_as_advanced(${PCL_COMPONENT_INCLUDE_DIR})
    list(APPEND PCL_LIBRARIES ${${PCL_COMPONENT}_LIBRARIES})
    list(APPEND PCL_LIBRARY_DIRS ${component_library_path})
    list(APPEND PCL_LIBRARY_DIRS ${component_library_path_debug})
    mark_as_advanced(${PCL_COMPONENT_LIBRARY} ${PCL_COMPONENT_LIBRARY}_DEBUG)
  endif(${PCL_COMPONENT}_FOUND)
endforeach(component)

if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_INCLUDE_DIRS)
endif(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")

if(NOT "${PCL_LIBRARY_DIRS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_LIBRARY_DIRS)
endif(NOT "${PCL_LIBRARY_DIRS}" STREQUAL "")

find_package_handle_standard_args(PCL DEFAULT_MSG PCL_LIBRARIES PCL_INCLUDE_DIRS)
mark_as_advanced(PCL_LIBRARIES PCL_INCLUDE_DIRS PCL_LIBRARY_DIRS)

if(PCL_FOUND)
  set(PCL_VERSION ${PCL_FOUND_VERSION})
  list(APPEND PCL_DEFINITIONS -DEIGEN_USE_NEW_STDVECTOR) 
  list(APPEND PCL_DEFINITIONS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
  if(WIN32)
    list(APPEND PCL_DEFINITIONS -DBOOST_ALL_NO_LIB)
    if(IS_FLANN_STATIC)
      list(APPEND PCL_DEFINITIONS -DFLANN_STATIC)
    endif(IS_FLANN_STATIC)
    if(IS_CMINPACK_STATIC)
      list(APPEND PCL_DEFINITIONS -DCMINPACK_NO_DLL)
    endif(IS_CMINPACK_STATIC)
  endif(WIN32)
endif(PCL_FOUND)

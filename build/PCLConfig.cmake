# ------------------------------------------------------------------------------------
# Helper to use PCL from outside project
#
# target_link_libraries(my_fabulous_target pcl_xxx) where xxx is in
# - common
# - cuda
# - features
# - filters
# - io
# - kdtree
# - keypoints
# - octree
# - range_image
# - range_image_border_extractor
# - registration
# - sample_consensus
# - segmentation
# - surface
# - visualization
#
# For compatibility reason we also provide PCL_XXX_LIBRARY, PCL_XXX_LIBRARY_DEBUG
# and PCL_XXX_LIBRARIES where XXX is upper cased xxx from the components above.
#
# PCL_INCLUDE_DIRS is filled with PCL and available 3rdparty headers
# PCL_LIBRARY_DIRS is filled with PCL components libraries install directory and
# 3rdparty libraries paths
# 
# Tested with:
# - PCL 1.0
# - PCL 1.0.1
#
#                                   www.pointclouds.org
#------------------------------------------------------------------------------------

set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.1")
set(PCL_LIBRARY_DIRS "/usr/local/lib")
include("/usr/local/lib/pcl/PCLDepends.cmake")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS common)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS io)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS filters)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS features)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS kdtree)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS octree)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS range_image)
  list(APPEND PCL_COMPONENTS range_image_border_extractor)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS surface)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS sample_consensus)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS registration)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS segmentation)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS visualization)
endif("TRUE" STREQUAL "TRUE")

if("TRUE" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS keypoints)
endif("TRUE" STREQUAL "TRUE")

if("" STREQUAL "TRUE")
  list(APPEND PCL_COMPONENTS cuda)
endif("" STREQUAL "TRUE")

if(PCL_FIND_COMPONENTS)
  foreach(comp ${PCL_FIND_COMPONENTS})
    list(FIND PCL_COMPONENTS ${comp} found)
    if(found EQUAL -1)
      message(FATAL_ERROR "requested components ${comp} not found")
    else(found EQUAL -1)
      string(TOUPPER "${comp}" COMP)
      set(PCL_${COMP}_LIBRARY pcl_${comp})
      set(PCL_${COMP}_LIBRARY_DEBUG pcl_${comp})
      get_target_property(CONFIGURATIONS pcl_${comp} IMPORTED_CONFIGURATIONS)
      list(FIND CONFIGURATIONS DEBUG debug_found)
      if(debug_found EQUAL -1 AND WIN32)
        message("${comp} not installed in Debug configuration, using default!")
      endif(debug_found EQUAL -1 AND WIN32)
      set(PCL_${COMP}_LIBRARIES pcl_${comp})
      list(APPEND PCL_LIBRARIES ${PCL_${COMP}_LIBRARIES})
    endif(found EQUAL -1)
  endforeach(comp)
else(PCL_FIND_COMPONENTS)
  foreach(comp ${PCL_COMPONENTS})
    string(TOUPPER "${comp}" COMP)
    set(PCL_${COMP}_LIBRARY pcl_${comp})
    set(PCL_${COMP}_LIBRARY_DEBUG pcl_${comp})
    get_target_property(CONFIGURATIONS pcl_${comp} IMPORTED_CONFIGURATIONS)
    list(FIND CONFIGURATIONS DEBUG debug_found)
    if(debug_found EQUAL -1 AND WIN32)
      message("${comp} not installed in Debug configuration, using default!")
    endif(debug_found EQUAL -1 AND WIN32)
    set(PCL_${COMP}_LIBRARIES pcl_${comp})
    list(APPEND PCL_LIBRARIES ${PCL_${COMP}_LIBRARIES})
  endforeach(comp)
endif(PCL_FIND_COMPONENTS)

##
# append 3rd party stuff
##
list(APPEND PCL_INCLUDE_DIRS "/usr/include")

list(APPEND PCL_INCLUDE_DIRS "/usr/include")

list(APPEND PCL_INCLUDE_DIRS "/usr/include/cminpack")

list(APPEND PCL_INCLUDE_DIRS "/usr/include/eigen3")

list(APPEND PCL_INCLUDE_DIRS "/usr/include/cminpack")

if("ON" STREQUAL "ON")
  list(APPEND PCL_INCLUDE_DIRS "/usr/include/openni")
endif("ON" STREQUAL "ON")

if("ON" STREQUAL "ON")
  list(APPEND PCL_INCLUDE_DIRS "/usr/include/qhull")
endif("ON" STREQUAL "ON")

if("ON" STREQUAL "ON")
  find_package(wxWidgets REQUIRED)
  include("/usr/local/share/cmake-2.8/Modules/UsewxWidgets.cmake")
  list(APPEND PCL_INCLUDE_DIRS "/usr/lib/wx/include/gtk2-unicode-release-2.8;/usr/include/wx-2.8")
  list(APPEND PCL_LIBRARY_DIRS "")
endif ("ON" STREQUAL "ON")

if("ON" STREQUAL "ON")
  set(VTK_DIR "/usr/lib/vtk-5.2")
  find_package(VTK REQUIRED)
  include ("${VTK_USE_FILE}")
  list(APPEND PCL_INCLUDE_DIRS "${VTK_INCLUDE_DIRS}")
  list(APPEND PCL_LIBRARY_DIRS "${VTK_LIBRARY_DIRS}")
endif("ON" STREQUAL "ON")

list(SORT PCL_INCLUDE_DIRS)
list(REMOVE_DUPLICATES PCL_INCLUDE_DIRS)

##
# Add some definitions for eigen sake
##
add_definitions(-DEIGEN_USE_NEW_STDVECTOR 
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

##
# those are for windows sake
##
if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
  if("/usr/lib/libflann_cpp.so" MATCHES "flann_cpp_s")
    add_definitions(-DFLANN_STATIC)
  endif("/usr/lib/libflann_cpp.so" MATCHES "flann_cpp_s")
  if("/usr/lib/libcminpack.so" MATCHES "cminpack_s")
    add_definitions(-DCMINPACK_NO_DLL)
  endif("/usr/lib/libcminpack.so" MATCHES "cminpack_s")
endif(MSVC)

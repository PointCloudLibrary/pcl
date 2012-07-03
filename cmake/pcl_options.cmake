# Options for building PCL.

# Build shared libraries by default.
option(PCL_SHARED_LIBS "Build shared libraries." ON)
if(PCL_SHARED_LIBS)
  set(PCL_LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
  set(PCL_LIB_SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
  set(PCL_LIB_TYPE "SHARED")
#  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_SHARED_LIBRARY_SUFFIX})
  if(WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_IMPORT_LIBRARY_SUFFIX})
  endif(WIN32)
else(PCL_SHARED_LIBS)
  set(PCL_LIB_PREFIX ${CMAKE_STATIC_LIBRARY_PREFIX})
  set(PCL_LIB_SUFFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
  set(PCL_LIB_TYPE "STATIC")
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
endif(PCL_SHARED_LIBS)
mark_as_advanced(PCL_SHARED_LIBS)

# Precompile for a minimal set of point types instead of all.
option(PCL_ONLY_CORE_POINT_TYPES "Compile explicitly only for a small subset of point types (e.g., pcl::PointXYZ instead of PCL_XYZ_POINT_TYPES)." OFF)
mark_as_advanced(PCL_ONLY_CORE_POINT_TYPES)


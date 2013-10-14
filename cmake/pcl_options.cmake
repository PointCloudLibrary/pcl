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

# Build with dynamic linking for Boost (advanced users)
option(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32 "Build against a dynamically linked Boost on Win32 platforms." OFF)
mark_as_advanced(PCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32)

# Precompile for a minimal set of point types instead of all.
option(PCL_ONLY_CORE_POINT_TYPES "Compile explicitly only for a small subset of point types (e.g., pcl::PointXYZ instead of PCL_XYZ_POINT_TYPES)." OFF)
mark_as_advanced(PCL_ONLY_CORE_POINT_TYPES)

# Precompile for a minimal set of point types instead of all.
option(PCL_NO_PRECOMPILE "Do not precompile PCL code for any point types at all." OFF)
mark_as_advanced(PCL_NO_PRECOMPILE)

# Enable or Disable the check for SSE optimizations
option(PCL_ENABLE_SSE "Enable or Disable SSE optimizations." ON)
mark_as_advanced(PCL_ENABLE_SSE)

# Display timing information for each compiler instance on screen
option(CMAKE_TIMING_VERBOSE "Enable the display of timing information for each compiler instance." OFF)
mark_as_advanced(CMAKE_TIMING_VERBOSE)

# MSVC extra optimization options. Might lead to increasingly larger compile/link times.
option(CMAKE_MSVC_CODE_LINK_OPTIMIZATION "Enable the /GL and /LTCG code and link optimization options for MSVC. Enabled by default." ON)
mark_as_advanced(CMAKE_MSVC_CODE_LINK_OPTIMIZATION)


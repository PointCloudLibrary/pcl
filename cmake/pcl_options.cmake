# Options for building PCL.

# Build shared libraries by default.
if(WIN32)
	option(PCL_SHARED_LIBS "Build shared libraries." OFF)
else(WIN32)
	option(PCL_SHARED_LIBS "Build shared libraries." ON)
endif(WIN32)
mark_as_advanced(PCL_SHARED_LIBS)

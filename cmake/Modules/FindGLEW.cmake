# Copyright (c) 2009 Boudewijn Rempt <boud@valdyas.org>                                                                                          
#                                                                                                                                                
# Redistribution and use is allowed according to the terms of the BSD license.                                                                   
# For details see the accompanying COPYING-CMAKE-SCRIPTS file. 
# 
# - try to find glew library and include files
#  GLEW_INCLUDE_DIR, where to find GL/glew.h, etc.
#  GLEW_LIBRARIES, the libraries to link against
#  GLEW_FOUND, If false, do not try to use GLEW.
# Also defined, but not for general use are:
#  GLEW_GLEW_LIBRARY = the full path to the glew library.

if(WIN32)

  if(CYGWIN)

    find_path( GLEW_INCLUDE_DIR GL/glew.h)

    find_library( GLEW_GLEW_LIBRARY glew32
      ${OPENGL_LIBRARY_DIR}
      /usr/lib/w32api
      /usr/X11R6/lib
    )


  else()
  
    find_path( GLEW_INCLUDE_DIR GL/glew.h
      $ENV{GLEW_ROOT}/include
    )

    find_library( GLEW_GLEW_LIBRARY
      NAMES glew glew32 glew32s
      PATHS
      $ENV{GLEW_ROOT}/lib
      ${OPENGL_LIBRARY_DIR}
    )

  endif()

else()

  if(APPLE)
# These values for Apple could probably do with improvement.
  if(${CMAKE_SYSTEM_VERSION} VERSION_LESS "13.0.0")
    find_path( GLEW_INCLUDE_DIR glew.h
      /System/Library/Frameworks/GLEW.framework/Versions/A/Headers
      ${OPENGL_LIBRARY_DIR}
      )
    set(GLEW_GLEW_LIBRARY "-framework GLEW" CACHE STRING "GLEW library for OSX")
  else()
    find_package(PkgConfig)
    pkg_check_modules(GLEW glew)
    set(GLEW_GLEW_LIBRARY ${GLEW_LIBRARIES} CACHE STRING "GLEW library for OSX")
    set(GLEW_INCLUDE_DIR ${GLEW_INCLUDEDIR} CACHE STRING "GLEW include dir for OSX")
  endif()
    set(GLEW_cocoa_LIBRARY "-framework Cocoa" CACHE STRING "Cocoa framework for OSX")
  else()

    find_path( GLEW_INCLUDE_DIR GL/glew.h
      /usr/include/GL
      /usr/openwin/share/include
      /usr/openwin/include
      /usr/X11R6/include
      /usr/include/X11
      /opt/graphics/OpenGL/include
      /opt/graphics/OpenGL/contrib/libglew
    )

    find_library( GLEW_GLEW_LIBRARY GLEW
      /usr/openwin/lib
      /usr/X11R6/lib
    )

  endif()

endif()

set( GLEW_FOUND FALSE )
if(GLEW_INCLUDE_DIR)
  if(GLEW_GLEW_LIBRARY)
    # Is -lXi and -lXmu required on all platforms that have it?
    # If not, we need some way to figure out what platform we are on.
    set( GLEW_LIBRARIES
      ${GLEW_GLEW_LIBRARY}
      ${GLEW_cocoa_LIBRARY}
    )
    set( GLEW_FOUND TRUE )

#The following deprecated settings are for backwards compatibility with CMake1.4
    set(GLEW_LIBRARY ${GLEW_LIBRARIES})
    set(GLEW_INCLUDE_PATH ${GLEW_INCLUDE_DIR})

  endif()
endif()

if(GLEW_FOUND)
  if(NOT GLEW_FIND_QUIETLY)
    message(STATUS "Found Glew: ${GLEW_LIBRARIES}")
  endif()
  if(GLEW_GLEW_LIBRARY MATCHES glew32s)
    add_definitions(-DGLEW_STATIC)
  endif()
else()
  if(GLEW_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find Glew")
  endif()
endif()

mark_as_advanced(
  GLEW_INCLUDE_DIR
  GLEW_GLEW_LIBRARY
  GLEW_Xmu_LIBRARY
  GLEW_Xi_LIBRARY
)

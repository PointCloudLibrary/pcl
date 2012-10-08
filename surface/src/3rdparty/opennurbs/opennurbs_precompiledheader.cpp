/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

// Delete evertyhing this comment and everything below it except
// the "#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"" for the public version of 
// opennurbs source code.


// CHECK SETTINGS BEFORE ANYTHING IS INCLUDED

#if defined(WIN32) && defined(WIN64)
#error WIN32 and WIN64 defined.  This is wrong!
#endif

#if defined(WIN64)

#if !defined(_WIN32)
#error Microsoft defines _WIN32 for all Windows builds
#endif

#if defined(_M_IX86) || defined(_M_IA64)
#error Incorrect _M_... setting for x64 build
#endif

#if !defined(_M_X64)
// This should be automatically defined by the compiler
#error _M_X64 should be defined for x64 builds
#endif

// All opennurbs code uses the "offical" _M_X64. Unfortunately, 
// some Microsoft VC 2005 header files, like float.h do not.
// The Microsoft compiler should automatically defined both
// _M_X64 and _M_AMD64 for the WIN64 platform.  If it doesn't,
// then we have a serious problem because some system header
// files will not be correctly preprocessed.
#if !defined(_M_AMD64)
// This should be automatically defined by the compiler
#error _M_AMD64 should be defined for x64 builds
#endif

#endif


#if defined(WIN32)

#if !defined(_WIN32)
#error Microsoft defines _WIN32 for all Windows builds
#endif

//#if defined(_M_IA64) || defined(_M_X64) || defined(_M_AMD64)
//#error Incorrect _M_... setting for 32 bit Windows build.
//#endif

//#if !defined(_M_IX86)
// This should be automatically defined by the compiler
//#error _M_IX86 should be defined for 32 bit Windows builds.
//#endif

#endif



#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"



// CHECK SETTINGS AFTER EVERTHING IS INCLUDED

//#if defined(WIN32) && defined(WIN64)
//#error WIN32 and WIN64 defined.  This is wrong!
//#endif

#if defined(WIN64)

#if !defined(_WIN32)
#error Microsoft defines _WIN32 for all Windows builds
#endif

#if defined(_M_IX86) || defined(_M_IA64)
#error Incorrect _M_... setting for x64 build
#endif

#if !defined(_M_X64)
// This should be automatically defined by the compiler
#error _M_X64 should be defined for x64 builds
#endif

// All opennurbs code uses the "offical" _M_X64. Unfortunately, 
// some Microsoft VC 2005 header files, like float.h do not.
// The Microsoft compiler should automatically defined both
// _M_X64 and _M_AMD64 for the WIN64 platform.  If it doesn't,
// then we have a serious problem because some system header
// files will not be correctly preprocessed.
#if !defined(_M_AMD64)
// This should be automatically defined by the compiler
#error _M_AMD64 should be defined for x64 builds
#endif

#endif


#if defined(WIN32)

#if !defined(_WIN32)
#error Microsoft defines _WIN32 for all Windows builds
#endif

//#if defined(_M_IA64) || defined(_M_X64) || defined(_M_AMD64)
//#error Incorrect _M_... setting for 32 bit Windows build.
//#endif

//#if !defined(_M_IX86)
// This should be automatically defined by the compiler
//#error _M_IX86 should be defined for 32 bit Windows builds.
//#endif

#endif


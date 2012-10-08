/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
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

#if !defined(OPENNURBS_VERSION_DEFINITION)
#error Do NOT include opennurbs_version.h in your code.  Use ON::Version() instead.
#endif

// OpenNURBS users:
//   Do not change OPENNURBS_VERSION or the OpenNURBS code
//   that reads 3DM files not work correctly.

// The YYYYMMDD portion of the Debug and Release
// version numbers is always the same.  
// The last digit of a debug build version number is 9. 
// The last digit of a V4 release build version number is 4.
// The last digit of a V5 release build version number is 5.
#if defined(_DEBUG)
#define OPENNURBS_VERSION 201111229
#else
#define OPENNURBS_VERSION 201111225
#endif

// Subversion revision used to build opennurbs.
// The build process updates this number; it should be zero for developers.
// If the build process commits a number other than 0 it's a bug.
#define OPENNURBS_SVN_REVISION "0"

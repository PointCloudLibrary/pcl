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

#include "pcl/surface/3rdparty/opennurbs/opennurbs_system.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_defines.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_memory.h"

// Memory utilities used by OpenNURBS.  If you are using
// custom memory managment, NOTHING in this file needs to
// be changed.


void* onmemdup( const void* src, size_t sz )
{
  void* p;
  if ( src && sz>0 ) 
  {
    p = onmalloc(sz);
    if (p)
      memcpy(p,src,sz);
  }
  else {
    p = 0;
  }
  return p;
}


char* onstrdup( const char* src )
{
  char* p;
  size_t sz;
  if ( src ) 
  {
    for ( sz=0;*src++;sz++)
      ; /* empty for body */
    sz++;
    p = (char*)onmemdup( src-sz, sz*sizeof(*src) );
  }
  else 
  {
    p = 0;
  }
  return p;
}


unsigned char* onmbsdup( const unsigned char* src )
{
  unsigned char* p;
  size_t sz; /* sz = number of bytes to dup (>=_mbclen(scr)) */
  if ( src ) 
  {
    for ( sz=0;*src++;sz++)
      ; /* empty for body */
    sz++;
    p = (unsigned char*)onmemdup( src-sz, sz*sizeof(*src) );
  }
  else 
  {
    p = 0;
  }
  return p;
}

#if defined(_WCHAR_T_DEFINED)

wchar_t* onwcsdup( const wchar_t* src )
{
  wchar_t* p;
  size_t sz;
  if ( src ) 
  {
    for ( sz=0;*src++;sz++)
      ; /* empty for body */
    sz++;
    p = (wchar_t*)onmemdup( src-sz, sz*sizeof(*src) );
  }
  else 
  {
    p = 0;
  }
  return p;
}
#endif

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

#include <pcl/surface/openNURBS/opennurbs_system.h>
#include <pcl/surface/openNURBS/opennurbs_defines.h>
#include <pcl/surface/openNURBS/opennurbs_memory.h>
#include <pcl/surface/openNURBS/opennurbs_error.h>

#if defined(ON_DLL_IMPORTS)
/*
// If you use OpenNURBS as a windows DLL, then define ON_DLL_IMPORTS 
// in applications that use OpenNURBS and they will get onmalloc(), etc.,
// from the DLL.
//
// If you use OpenNURBS as a static library, do not define ON_DLL_IMPORTS.
*/
#error opennurbs_memory.c must not be compiled with ON_DLL_IMPORTS defined.
#endif


#if defined(_MSC_VER)
#if _MSC_VER == 1200
/*
//  (_MSC_VER is defined as 1200 for Microsoft Visual C++ 6.0)
//
//   NOTE WELL: Microsoft's VC 6.0 realloc() contains a bug that can cause
//              crashes and should be avoided.  See MSDN Knowledge Base
//              article ID Q225099 for more information.
*/
#define ON_REALLOC_BROKEN
#endif
#endif


void* onmalloc( size_t sz )
{
  return (sz > 0) ? malloc(sz) : 0;
}

void* oncalloc( size_t num, size_t sz )
{
  return (num > 0 && sz > 0) ? calloc(num,sz) : 0;
}

void onfree( void* memblock )
{
  if ( memblock )
    free( memblock );
}

void* onrealloc( void* memblock, size_t sz )
{
  if ( 0 == memblock )
  {
    return onmalloc(sz);
  }

  if ( 0 == sz )
  {
    onfree(memblock);
    return 0;
  }

#if defined(ON_REALLOC_BROKEN)
  /* use malloc() and memcpy() instead of buggy realloc() */
  void* p;
  const size_t memblocksz = _msize(memblock);
  if ( sz <= memblocksz ) {
    /* shrink */
    if ( memblocksz <= 28 || 8*sz >= 7*memblocksz ) 
    {
      /* don't bother reallocating */
      p = memblock;
    }
    else {
      /* allocate smaller block */
      p = malloc(sz);
      if ( p ) 
      {
        memcpy( p, memblock, sz );
        free(memblock);
      }
    }
  }
  else if ( sz > memblocksz ) {
    /* grow */
    p = malloc(sz);
    if ( p ) {
      memcpy( p, memblock, memblocksz );
      free(memblock);
    }
  }
  return p;
#else
  return realloc( memblock, sz );
#endif
}

size_t onmsize( const void* memblock )
{
  size_t sz =
#if defined(ON_OS_WINDOWS)
  (0 != memblock) ? _msize((void*)memblock) : 0
#else
  // OS doesn't support _msize().
  0
#endif
  ;
  
  return sz;
}

void ON_MemoryManagerBegin(void)
{
}

void ON_MemoryManagerEnd(void)
{
}


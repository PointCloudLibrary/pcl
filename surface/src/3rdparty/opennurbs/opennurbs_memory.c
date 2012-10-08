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
#include "pcl/surface/3rdparty/opennurbs/opennurbs_error.h"

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

ON_MEMORY_POOL* ON_MainMemoryPool(void)
{
  return 0;
}

ON_MEMORY_POOL* ON_WorkerMemoryPool(void)
{
  return 0;
}

void* onmalloc_from_pool( ON_MEMORY_POOL* pool, size_t sz )
{
  void* p;
  p = (sz > 0) ? malloc(sz) : 0;
  return p;
}

void* onmalloc( size_t sz )
{
  return onmalloc_from_pool( 0, sz );
}

void* oncalloc_from_pool( ON_MEMORY_POOL* pool, size_t num, size_t sz )
{
  void* p;
  p = (num > 0 && sz > 0) ? calloc(num,sz) : 0;
  return p;
}

void* oncalloc( size_t num, size_t sz )
{
  return oncalloc_from_pool( 0, num, sz );
}

void onfree( void* memblock )
{
  if ( 0 != memblock )
    free(memblock);
}

void* onrealloc( void* memblock, size_t sz )
{
  return onrealloc_from_pool( 0, memblock, sz );
}

void* onrealloc_from_pool( ON_MEMORY_POOL* pool, void* memblock, size_t sz )
{
  void* p;
  
  if ( sz <= 0 ) 
  {
    onfree(memblock);
    return 0;
  }
  
  if ( !memblock ) 
  {
    return onmalloc_from_pool( pool, sz);
  }

  p = realloc(memblock,sz);

  return p;
}

size_t onmsize( const void* memblock )
{
  size_t sz = 0;

  if (memblock) 
  {
#if defined(ON_COMPILER_MSC)
    sz = _msize( (void*)memblock );
#elif defined(ON_COMPILER_XCODE)
    sz = malloc_size( (void*)memblock );
#else
    // No predictable function exists and
    // nothing in core opennurbs code uses
    // onmsize().  If you find a portable
    // way to support another compiler or 
    // platform, then report it to the support
    // contact on http://opennurbs.org and
    // the code will be added in the next release.
    //ON_ERROR("onmsize not implemented on this compiler or platform.");
    sz = 0;
#endif
  }

  return sz;
}


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

#if !defined(OPENNURBS_MEMORY_INC_)
#define OPENNURBS_MEMORY_INC_

#if defined (cplusplus) || defined(_cplusplus) || defined(__cplusplus)
extern "C" {
#endif

ON_DECL
size_t ON_MemoryPageSize();


ON_DECL
void ON_MemoryManagerBegin(void);

ON_DECL
void ON_MemoryManagerEnd(void);

/*
/////////////////////////////////////////////////////////////////////////////
//
// ALL memory managment in the openNURBS toolkit is done through calls to
//    onmalloc(), oncalloc(), onrealloc(), onfree(), 
//    onmsize(), onmemdup(), onstrdup(), ..., and the
//    new and delete operators
// The on*() functions are all declared in opennurbs_memory.h and defined
// in opennurbs_memory.c.  The new and delete operators that use onmalloc()
// and onfree() are defined in opennurbs_memory_new.cpp.
//
// You may OPTIONALLY provide your own memory managment functions.  See
// opennurbs_memory.c for details.
//
/////////////////////////////////////////////////////////////////////////////
//
// Details:
//
// void* onmalloc( site_t sz );
//
//   If sz is zero, NULL is returned.
//   If sz is positive and there is not enough memory to satify
//   the allocation request, the ON_memory_error_handler(0) is called
//   and NULL is returned.  
//
//   If you have implemented multiple memory pools in a custom manager,
//   the request is sent to the current pool.
//        
//
// void* oncalloc( size_t num, size_t sz );
//
//   If sz or num is zero, NULL is returned.
//   If sz and num are positive and there is not enough memory to satify
//   the allocation request, the ON_memory_error_handler(0) is called
//   and NULL is returned.
//
//   If you have implemented multiple memory pools in a custom manager,
//   the request is sent to the current pool.
//               
//               
// void* onrealloc( void* p, size_t sz );
//
//   If p is NULL, then onmalloc(sz) is called.
//   If sz is zero, then onfree(p) is called and
//   NULL is returned.
//   If p is not NULL, sz is positive, and there is not
//   enough memory to to satify the allocation request,
//   the ON_memory_error_handler(0) is called and NULL is returned.
//   If p is not NULL and is known to be invalid, then
//   ON_memory_error_handler(1) is called.
//
//   If you have implemented multiple memory pools in a custom manager,
//   the request is sent to the current pool.
//
//   NOTE WELL: Microsoft's VC 6.0 realloc() contains a bug that can cause
//              crashes and should be avoided.  See MSDN Knowledge Base
//              article ID Q225099 for more information.
//
//
// void onfree( void* p );
//
//   NULL p is tolerated but considered poor style.
//   If p is not NULL and is known to be invalid, then
//   ON_memory_error_handler(2) is called.
//
//
// size_t onmsize( void*  p );
//
//   If p is NULL, then zero is returned.  Otherwise the
//   the size in bytes of the memory block allocated by onmalloc(),
//   oncalloc(), or onrealloc() is returned.
//               
//
// void* onmemdup( const void* src, size_t sz );
//
//   If src is not NULL and sz is positive, then onmalloc( sz )
//   is called to get memory, sz bytes of src are copied into this
//   memory, and the pointer to this memory is returned.
//   If onmalloc() returns NULL, then NULL is returned.
//   If src is NULL or sz is zero, then NULL is returned.
//               
//
// char* onstrdup( const char* src );
//
//   If src is not NULL, then onmemdup( sc, (strlen(src)+1)*sizeof(src[0]) )
//   is called.  If onmemdup() returns NULL, then NULL is returned.
//   If src is NULL, then NULL is returned.
//               
//
*/

ON_DECL
void*  onmalloc( size_t );

ON_DECL
void*  oncalloc( size_t, size_t );

ON_DECL
void   onfree( void* );

ON_DECL
void*  onrealloc( void*, size_t );

ON_DECL
size_t onmsize( const void* );

ON_DECL
void*  onmemdup( const void*, size_t );

ON_DECL
char*  onstrdup( const char* );

#if defined(_WCHAR_T_DEFINED)
ON_DECL
wchar_t* onwcsdup( const wchar_t* );
#endif

ON_DECL
unsigned char* onmbsdup( const unsigned char* );

/* define to handle _TCHAR* ontcsdup( const _TCHAR* ) */
#if defined(_UNICODE)
#define ontcsdup onwcsdup
#elif defined(_MBCS)
#define ontcsdup onmbsdup
#else
#define ontcsdup onstrdup
#endif

#if defined (cplusplus) || defined(_cplusplus) || defined(__cplusplus)
}
#endif

#endif

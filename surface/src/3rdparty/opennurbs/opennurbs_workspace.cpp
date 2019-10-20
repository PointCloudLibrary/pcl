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

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

ON_Workspace::ON_Workspace() 
: m_pFileBlk(0)
, m_pMemBlk(0)
{}

ON_Workspace::~ON_Workspace()
{
  Destroy();
}

struct ON_Workspace_FBLK 
{
  struct ON_Workspace_FBLK* pNext;
  FILE* pFile;
} * m_pFileBlk;

struct ON_Workspace_MBLK 
{
  struct ON_Workspace_MBLK* pNext;
  void* pMem;
} * m_pMemBlk;

void ON_Workspace::Destroy()
{
  struct ON_Workspace_FBLK* pFileBlk = m_pFileBlk;
  while ( pFileBlk ) {
    if ( pFileBlk->pFile )
      fclose( pFileBlk->pFile );
    pFileBlk = pFileBlk->pNext;
  }
  m_pFileBlk = 0;

  struct ON_Workspace_MBLK* pNext = m_pMemBlk;
  struct ON_Workspace_MBLK* p = NULL;
  while ( pNext ) {
    p = pNext;
    pNext = pNext->pNext;
    if ( p->pMem ) {
      onfree(p->pMem);
      p->pMem = NULL;
    }
    onfree( p );
  }
  m_pMemBlk = 0;
}

void* ON_Workspace::GetMemory( std::size_t size )
{
  void* p = NULL;
  if ( size > 0 ) 
  {
    struct ON_Workspace_MBLK* pBlk = (struct ON_Workspace_MBLK*)onmalloc(sizeof(*pBlk));
    if ( pBlk ) 
    {
      pBlk->pMem = p = onmalloc(size);
      pBlk->pNext = m_pMemBlk;
      m_pMemBlk = pBlk;
    }
  }
  return p;
}

void* ON_Workspace::GrowMemory( void* p, std::size_t size )
{
  void* newp = NULL;
  if ( !p ) {
    newp = GetMemory(size);
  }
  else {
    struct ON_Workspace_MBLK* pBlk = m_pMemBlk;
    while ( pBlk ) {
      if ( pBlk->pMem == p ) {
        if ( size > 0 ) {
          newp = onrealloc(p,size);
        }
        else {
          newp = p;
        }
        pBlk->pMem = newp;
        break;
      }
      pBlk = pBlk->pNext;
    }
  }
  return newp;
}

void ON_Workspace::KeepAllMemory()
{
  struct ON_Workspace_MBLK* p;
  struct ON_Workspace_MBLK* pNext = m_pMemBlk;
  m_pMemBlk = 0;
  while ( pNext )
  {
    p = pNext;
    pNext = pNext->pNext;
    p->pMem = 0; // caller want to manage this heap
    onfree( p );
  }
}

int ON_Workspace::KeepMemory( void* p )
{
  int rc = false;
  if ( p ) {
    struct ON_Workspace_MBLK* pPrevBlk = NULL;
    struct ON_Workspace_MBLK* pBlk = m_pMemBlk;
    while ( pBlk ) {
      if ( pBlk->pMem == p ) {
        // Remove pBlk from list so ~ON_Workspace() won't onfree() its memory
        // and any future GrowMemory...() or KeepMemory() calls won't have
        // to search past it.
        pBlk->pMem = NULL;
        if ( pPrevBlk ) {
          pPrevBlk->pNext = pBlk->pNext;
        }
        else {
          m_pMemBlk = pBlk->pNext;
        }
        onfree( pBlk );
        rc = true;
        break;
      }
      pPrevBlk = pBlk;
      pBlk = pBlk->pNext;
    }
  }
  return rc;
}

int* ON_Workspace::GetIntMemory( std::size_t size )
{
  int* pi = (int*)(GetMemory(size*sizeof(*pi)));
  return pi;
}

double* ON_Workspace::GetDoubleMemory( std::size_t size )
{
  double* pd = (double*)(GetMemory(size*sizeof(*pd)));
  return pd;
}

ON_3dPoint* ON_Workspace::GetPointMemory( std::size_t size )
{
  ON_3dPoint* p3d = (ON_3dPoint*)(GetMemory(size*sizeof(*p3d)));
  return p3d;
}

ON_3dVector* ON_Workspace::GetVectorMemory( std::size_t size )
{
  ON_3dVector* v3d = (ON_3dVector*)(GetMemory(size*sizeof(*v3d)));
  return v3d;
}

int** ON_Workspace::GetIntMemory( std::size_t row_count, std::size_t col_count )
{
  int** p = 0;
  std::size_t i;
  if ( row_count > 0 && col_count > 0 )
  {
    p = (int**)GetMemory(row_count*(sizeof(*p) + col_count*sizeof(**p)));
    if ( p )
    {
      p[0] = (int*)(p+row_count);
      for( i = 1; i < row_count; i++ )
      {
        p[i] = p[i-1] + col_count;
      }
    }
  }
  return p;
}

double** ON_Workspace::GetDoubleMemory( std::size_t row_count, std::size_t col_count )
{
  double** p = 0;
  std::size_t i;
  if ( row_count > 0 && col_count > 0 )
  {
    // i keeps doubles aligned
    i = (sizeof(*p) < sizeof(**p))
      ? (row_count + (row_count%2))
      : row_count;
    p = (double**)GetMemory(i*sizeof(*p) + row_count*col_count*sizeof(**p));
    if ( p )
    {
      p[0] = (double*)(p+i);
      for( i = 1; i < row_count; i++ )
      {
        p[i] = p[i-1] + col_count;
      }
    }
  }
  return p;
}


int* ON_Workspace::GrowIntMemory( int* p, std::size_t size )
{
  int* pi = (int*)(GrowMemory(p,size*sizeof(*pi)));
  return pi;
}

double* ON_Workspace::GrowDoubleMemory( double* p, std::size_t size )
{
  double* pd = (double*)(GrowMemory(p,size*sizeof(*pd)));
  return pd;
}

ON_3dPoint* ON_Workspace::GrowPointMemory( ON_3dPoint* p, std::size_t size )
{
  ON_3dPoint* p3d = (ON_3dPoint*)(GrowMemory(p,size*sizeof(*p3d)));
  return p3d;
}

ON_3dVector* ON_Workspace::GrowVectorMemory( ON_3dVector* p, std::size_t size )
{
  ON_3dVector* v3d = (ON_3dVector*)(GrowMemory(p,size*sizeof(*v3d)));
  return v3d;
}


FILE* ON_Workspace::OpenFile( const char* sFileName, const char* sMode ) 
{
  FILE* pFile = ON::OpenFile( sFileName, sMode );
  if ( pFile ) 
  {
    struct ON_Workspace_FBLK* pFileBlk = (struct ON_Workspace_FBLK*)GetMemory( sizeof(*pFileBlk) );
    pFileBlk->pNext = m_pFileBlk;
    pFileBlk->pFile = pFile;
    m_pFileBlk = pFileBlk;
  }
  return pFile;
}

FILE* ON_Workspace::OpenFile( const wchar_t* sFileName, const wchar_t* sMode ) 
{
  FILE* pFile = ON::OpenFile( sFileName, sMode );
  if ( pFile ) 
  {
    struct ON_Workspace_FBLK* pFileBlk = (struct ON_Workspace_FBLK*)GetMemory( sizeof(*pFileBlk) );
    pFileBlk->pNext = m_pFileBlk;
    pFileBlk->pFile = pFile;
    m_pFileBlk = pFileBlk;
  }
  return pFile;
}

int ON_Workspace::KeepFile( FILE* pFile )
{
  int rc = false;
  if ( pFile ) {
    struct ON_Workspace_FBLK* pFileBlk = m_pFileBlk;
    while ( pFileBlk ) {
      if ( pFileBlk->pFile == pFile ) {
        pFileBlk->pFile = NULL;
        rc = true;
        break;
      }
      pFileBlk = pFileBlk->pNext;
    }
  }
  return rc;
}

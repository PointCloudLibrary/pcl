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

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"   // openNURBS declarations of functions that perform simple calculations

ON_2dPointArray::ON_2dPointArray() 
{}

ON_2dPointArray::ON_2dPointArray(int c) 
                : ON_SimpleArray<ON_2dPoint>(c) 
{}

ON_2dPointArray::ON_2dPointArray(const ON_2dPointArray& src) 
                : ON_SimpleArray<ON_2dPoint>(src)
{}

bool ON_2dPointArray::GetBBox( // returns true if successful
       double boxmin[2],
       double boxmax[2],
       int bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_2dPointArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_2dPointArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 2, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_2dPointArray& ON_2dPointArray::operator=(const ON_2dPointArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_2dPoint>::operator=(src);
  }
  return *this;
}

ON_3dPointArray::ON_3dPointArray() 
{}

ON_3dPointArray::ON_3dPointArray(int c) : ON_SimpleArray<ON_3dPoint>(c) 
{}

ON_3dPointArray::ON_3dPointArray(const ON_SimpleArray<ON_3dPoint>& src) 
                : ON_SimpleArray<ON_3dPoint>(src)
{}

ON_3dPointArray::ON_3dPointArray(const ON_SimpleArray<ON_3fPoint>& src) 
{
  *this = src;
}

ON_BoundingBox ON_3dPointArray::BoundingBox() const
{
  ON_BoundingBox bbox;
  GetBoundingBox(bbox);
  return bbox;
}

bool ON_3dPointArray::GetBoundingBox( 
  ON_BoundingBox& bbox,
  int bGrowBox
  ) const
{
  return GetBBox( &bbox.m_min.x, &bbox.m_max.x, bGrowBox );
}

bool ON_3dPointArray::GetBBox( // returns true if successful
       double boxmin[3],
       double boxmax[3],
       int bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_3dPointArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_3dPointArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 3, (m_a) ? &m_a[0].x : 0, i, j );
}


bool ON_3dPointArray::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis_of_rotation,
      const ON_3dPoint& center_of_rotation
      )
{
  const int count = m_count;
  ON_Xform rot;
  rot.Rotation( sin_angle, cos_angle, axis_of_rotation, center_of_rotation );
  ON_SimpleArray<int> fix_index(128);
  int i;
  for ( i = 0; i < count; i++ ) {
    if ( m_a[i] == center_of_rotation )
      fix_index.Append(i);
  }
  const bool rc = Transform( rot );
  for ( i = 0; i < fix_index.Count(); i++ ) {
    m_a[fix_index[i]] = center_of_rotation;
  }
  return rc;
}

bool ON_3dPointArray::Rotate(
      double angle_in_radians,
      const ON_3dVector& axis_of_rotation,
      const ON_3dPoint& center_of_rotation
      )
{
  return Rotate( sin(angle_in_radians), cos(angle_in_radians), axis_of_rotation, center_of_rotation );
}

bool ON_3dPointArray::Translate(
      const ON_3dVector& delta
      )
{
  int i;
  for (i=0;i<m_count;i++)
    m_a[i]+=delta;
  return (m_count>0)?true:false;
}


ON_3dPointArray& ON_3dPointArray::operator=(const ON_3dPointArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_3dPoint>::operator=(src);
  }
  return *this;
}


ON_3dPointArray& ON_3dPointArray::operator=(const ON_SimpleArray<ON_3fPoint>& src)
{
  Create( 3, false, src.Count(), 3, (const float*)src.Array() );
  return *this;
}


bool ON_3dPointArray::Create( 
  int point_dimension,
  int bRational,
  int point_count,
  int point_stride,
  const double* points
  )
{
  bool rc = false;
  if (     point_dimension >= 2 && point_dimension <= 3 
        && point_count>0 && points 
        && point_stride >= bRational?(point_dimension+1):point_dimension )
  {
    rc = true;
    int i;
    ON_3dPoint q(0.0,0.0,0.0);
    ON_4dPoint h(0.0,0.0,0.0,1.0);
    m_count = 0;
    SetCapacity(point_count);
    SetCount(point_count);
    if ( bRational )
    {
      for ( i = 0; i < point_count; i++ )
      {
        h.x = points[0];
        h.y = points[1];
        if ( point_dimension == 3 )
          h.z = points[2];
        h.w = points[point_dimension];
        m_a[i] = h;
        points += point_stride;
      }
    }
    else
    {
      for ( i = 0; i < point_count; i++ )
      {
        q.x = points[0];
        q.y = points[1];
        if ( point_dimension == 3 )
          q.z = points[2];
        m_a[i] = q;
        points += point_stride;
      }
    }
  }
  else
    Destroy();
  return rc;
}


bool ON_3dPointArray::Create( 
  int point_dimension,
  int bRational,
  int point_count,
  int point_stride,
  const float* points
  )
{
  bool rc = false;
  if (     point_dimension >= 2 && point_dimension <= 3 
        && point_count>0 && points 
        && point_stride >= bRational?(point_dimension+1):point_dimension )
  {
    rc = true;
    int i;
    ON_3dPoint q(0.0,0.0,0.0);
    ON_4dPoint h(0.0,0.0,0.0,1.0);
    m_count = 0;
    SetCapacity(point_count);
    SetCount(point_count);
    if ( bRational )
    {
      for ( i = 0; i < point_count; i++ )
      {
        h.x = points[0];
        h.y = points[1];
        if ( point_dimension == 3 )
          h.z = points[2];
        h.w = points[point_dimension];
        m_a[i] = h;
        points += point_stride;
      }
    }
    else
    {
      for ( i = 0; i < point_count; i++ )
      {
        q.x = points[0];
        q.y = points[1];
        if ( point_dimension == 3 )
          q.z = points[2];
        m_a[i] = q;
        points += point_stride;
      }
    }
  }
  else
    Destroy();
  return rc;
}



ON_4dPointArray::ON_4dPointArray() 
{}

ON_4dPointArray::ON_4dPointArray(int c) : ON_SimpleArray<ON_4dPoint>(c) 
{}

ON_4dPointArray::ON_4dPointArray(const ON_4dPointArray& src) : ON_SimpleArray<ON_4dPoint>(src)
{}

bool ON_4dPointArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 3, true, Count(), 4, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_4dPointArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 4, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_4dPointArray& ON_4dPointArray::operator=(const ON_4dPointArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_4dPoint>::operator=(src);
  }
  return *this;
}

ON_2dVectorArray::ON_2dVectorArray() 
{}

ON_2dVectorArray::ON_2dVectorArray(int c) : ON_SimpleArray<ON_2dVector>(c) 
{}

ON_2dVectorArray::ON_2dVectorArray(const ON_2dVectorArray& src) : ON_SimpleArray<ON_2dVector>(src)
{}

bool ON_2dVectorArray::GetBBox( // returns true if successful
       double boxmin[2],
       double boxmax[2],
       int bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_2dVectorArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_2dVectorArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 2, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_2dVectorArray& ON_2dVectorArray::operator=(const ON_2dVectorArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_2dVector>::operator=(src);
  }
  return *this;
}

ON_3dVectorArray::ON_3dVectorArray() 
{}

ON_3dVectorArray::ON_3dVectorArray(int c) : ON_SimpleArray<ON_3dVector>(c) 
{}

ON_3dVectorArray::ON_3dVectorArray(const ON_3dVectorArray& src) : ON_SimpleArray<ON_3dVector>(src)
{}

bool ON_3dVectorArray::GetBBox(
       double boxmin[3],
       double boxmax[3],
       bool bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_3dVectorArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_3dVectorArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 3, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_3dVectorArray& ON_3dVectorArray::operator=(const ON_3dVectorArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_3dVector>::operator=(src);
  }
  return *this;
}

////
////
////
ON_2fPointArray::ON_2fPointArray() 
{}

ON_2fPointArray::ON_2fPointArray(int c) 
                : ON_SimpleArray<ON_2fPoint>(c) 
{}

ON_2fPointArray::ON_2fPointArray(const ON_2fPointArray& src) 
                : ON_SimpleArray<ON_2fPoint>(src)
{}

bool ON_2fPointArray::GetBBox( // returns true if successful
       float boxmin[2],
       float boxmax[2],
       int bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_2fPointArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_2fPointArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 2, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_2fPointArray& ON_2fPointArray::operator=(const ON_2fPointArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_2fPoint>::operator=(src);
  }
  return *this;
}


ON_3fPointArray::ON_3fPointArray() 
{}

ON_3fPointArray::ON_3fPointArray(int c) : ON_SimpleArray<ON_3fPoint>(c)
{}

ON_3fPointArray::ON_3fPointArray(const ON_3fPointArray& src) : ON_SimpleArray<ON_3fPoint>(src)
{}

bool ON_3fPointArray::GetBBox( // returns true if successful
       float boxmin[3],
       float boxmax[3],
       int bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_3fPointArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_3fPointArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 3, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_3fPointArray& ON_3fPointArray::operator=(const ON_3fPointArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_3fPoint>::operator=(src);
  }
  return *this;
}

ON_4fPointArray::ON_4fPointArray() 
{}

ON_4fPointArray::ON_4fPointArray(int c) : ON_SimpleArray<ON_4fPoint>(c) 
{}

ON_4fPointArray::ON_4fPointArray(const ON_4fPointArray& src) : ON_SimpleArray<ON_4fPoint>(src)
{}

bool ON_4fPointArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 3, true, Count(), 4, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_4fPointArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 4, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_4fPointArray& ON_4fPointArray::operator=(const ON_4fPointArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_4fPoint>::operator=(src);
  }
  return *this;
}

ON_2fVectorArray::ON_2fVectorArray() 
{}

ON_2fVectorArray::ON_2fVectorArray(int c) : ON_SimpleArray<ON_2fVector>(c) 
{}

ON_2fVectorArray::ON_2fVectorArray(const ON_2fVectorArray& src) : ON_SimpleArray<ON_2fVector>(src)
{}

bool ON_2fVectorArray::GetBBox(
       float boxmin[2],
       float boxmax[2],
       bool bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_2fVectorArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 2, false, Count(), 2, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_2fVectorArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 2, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_2fVectorArray& ON_2fVectorArray::operator=(const ON_2fVectorArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_2fVector>::operator=(src);
  }
  return *this;
}

ON_3fVectorArray::ON_3fVectorArray() 
{}

ON_3fVectorArray::ON_3fVectorArray(int c) : ON_SimpleArray<ON_3fVector>(c) 
{}

ON_3fVectorArray::ON_3fVectorArray(const ON_3fVectorArray& src) : ON_SimpleArray<ON_3fVector>(src)
{}

bool ON_3fVectorArray::GetBBox( // returns true if successful
       float boxmin[3],
       float boxmax[3],
       int bGrowBox
       ) const
{
  return ON_GetPointListBoundingBox( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, boxmin, boxmax, bGrowBox );
}

bool ON_3fVectorArray::Transform( const ON_Xform& xform )
{
  return ON_TransformPointList( 3, false, Count(), 3, (m_a) ? &m_a[0].x : 0, xform );
}

bool ON_3fVectorArray::SwapCoordinates( int i, int j )
{
  return ON_SwapPointListCoordinates( Count(), 3, (m_a) ? &m_a[0].x : 0, i, j );
}

ON_3fVectorArray& ON_3fVectorArray::operator=(const ON_3fVectorArray& src)
{
  if ( this != &src ) {
    ON_SimpleArray<ON_3fVector>::operator=(src);
  }
  return *this;
}

ON_UuidPair::ON_UuidPair()
{
  memset(this,0,sizeof(*this));
}

int ON_UuidPair::CompareFirstUuid(const class ON_UuidPair* a,const class ON_UuidPair* b)
{
  if (!a)
  {
    return (b) ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }
  return ON_UuidCompare(a->m_uuid[0],b->m_uuid[0]);
}

int ON_UuidPair::CompareSecondUuid(const class ON_UuidPair* a,const class ON_UuidPair* b)
{
  if (!a)
  {
    return (b) ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }
  return ON_UuidCompare(a->m_uuid[1],b->m_uuid[1]);
}

int ON_UuidPair::Compare(const class ON_UuidPair* a,const class ON_UuidPair* b)
{
  int i;
  if (!a)
  {
    return (b) ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }
  if ( 0 == (i = ON_UuidCompare(a->m_uuid[0],b->m_uuid[0])) )
  {
    i = ON_UuidCompare(a->m_uuid[1],b->m_uuid[1]);
  }
  return i;
}

ON_UuidList::ON_UuidList() 
                     : ON_SimpleArray<ON_UUID>(32),
                       m_sorted_count(0),
                       m_removed_count(0)
{
}

ON_UuidList::ON_UuidList(int capacity) 
                     : ON_SimpleArray<ON_UUID>(capacity),
                       m_sorted_count(0),
                       m_removed_count(0)
{
}

ON_UuidList::~ON_UuidList()
{
  m_sorted_count = 0;
  m_removed_count = 0;
}

ON_UuidList::ON_UuidList(const ON_UuidList& src) 
                     : ON_SimpleArray<ON_UUID>(src),
                       m_sorted_count(src.m_sorted_count),
                       m_removed_count(src.m_removed_count)
{
}

ON_UuidList& ON_UuidList::operator=(const ON_UuidList& src)
{
  if ( this != &src)
  {
    ON_SimpleArray<ON_UUID>::operator=(src);
    m_sorted_count = src.m_sorted_count;
    m_removed_count = src.m_removed_count;
  }
  return *this;
}


bool ON_UuidList::AddUuid(ON_UUID uuid, bool bCheckForDupicates)
{
  bool rc = bCheckForDupicates ? !FindUuid(uuid) : true;
  if (rc)
  {
    Append(uuid);
  }
  return rc;
}

int ON_UuidList::Count() const
{
  return m_count - m_removed_count;
}

int ON_UuidList::CompareUuid(const ON_UUID* a, const ON_UUID* b)
{
  return memcmp(a,b,sizeof(*a));
}

void ON_UuidList::SortHelper()
{
  if ( m_sorted_count < m_count || m_removed_count > 0 )
  {
    // clean up array
    QuickSort(ON_UuidList::CompareUuid);
    while ( m_count > 0 && ON_max_uuid == m_a[m_count-1] )
    {
      m_count--;
    }
    m_removed_count = 0;
    m_sorted_count = m_count;
  }
}
const ON_UUID* ON_UuidList::Array() const
{
  const ON_UUID* array = 0;
  if ( m_count > m_removed_count )
  {
    const_cast<ON_UuidList*>(this)->SortHelper();
    if (m_count > 0 && m_sorted_count == m_count && 0 == m_removed_count )
      array = m_a;
  }
  return array;
}

void ON_UuidList::Empty()
{
  m_count = 0;
  m_sorted_count = 0;
  m_removed_count = 0;
}

void ON_UuidList::Destroy()
{
  ON_SimpleArray<ON_UUID>::Destroy();
  m_count = 0;
  m_sorted_count = 0;
  m_removed_count = 0;
}

void ON_UuidList::Reserve(int capacity)
{
  ON_SimpleArray<ON_UUID>::Reserve(capacity);
}

bool ON_UuidList::RemoveUuid(ON_UUID uuid)
{
  ON_UUID* p = SearchHelper(&uuid);
  if ( 0 != p )
  {
    *p = ON_max_uuid;
    m_removed_count++;
  }
  return (0!=p);
}

void ON_UuidList::Compact()
{
  SortHelper();
  SetCapacity(m_count);
}

bool ON_UuidList::Write( class ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
  if (rc)
  {
    const_cast<ON_UuidList*>(this)->SortHelper();
    rc = archive.WriteArray( *this );
    if ( !archive.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_UuidList::Read( class ON_BinaryArchive& archive )
{
  m_count = 0;
  m_removed_count = 0;
  m_sorted_count = 0;
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, 
                                       &major_version, 
                                       &minor_version );
  if (rc)
  {
    if ( 1 != major_version )
      rc = false;

    if (rc)
      rc = archive.ReadArray( *this );

    if ( !archive.EndRead3dmChunk() )
      rc = false;
  }

  SortHelper();

  return rc;
}


bool ON_UuidList::FindUuid(ON_UUID uuid) const
{
  return (0!=SearchHelper(&uuid));
}

ON_UUID* ON_UuidList::SearchHelper(const ON_UUID* uuid) const
{
  if ( m_count - m_sorted_count > 8 || m_removed_count > 0 )
  {
    // time to resort the array so that the speedy
    // bsearch() can be used to find uuids
    const_cast<ON_UuidList*>(this)->SortHelper();
  }

  ON_UUID* p = (m_sorted_count > 0 )
             ? (ON_UUID*)bsearch( uuid, m_a, m_sorted_count, sizeof(m_a[0]), 
                                  (int(*)(const void*,const void*))ON_UuidList::CompareUuid ) 
             : 0;

  if (0 == p)
  {
    // do a slow search on the last m_count-m_sort_count elements
    // in the array.
    int i;
    for ( i = m_sorted_count; i < m_count; i++ )
    {
      if ( 0 == ON_UuidList::CompareUuid(uuid,m_a+i) )
      {
        p = m_a+i;
        break;
      }
    }
  }

  return p;
}



ON_UuidIndexList::ON_UuidIndexList() 
                     : ON_SimpleArray<ON_UuidIndex>(32),
                       m_sorted_count(0),
                       m_removed_count(0)
{
}

ON_UuidIndexList::ON_UuidIndexList(int capacity) 
                     : ON_SimpleArray<ON_UuidIndex>(capacity>32?capacity:32),
                       m_sorted_count(0),
                       m_removed_count(0)
{
}

ON_UuidIndexList::~ON_UuidIndexList()
{
  m_sorted_count = 0;
  m_removed_count = 0;
}

ON_UuidIndexList::ON_UuidIndexList(const ON_UuidIndexList& src) 
                     : ON_SimpleArray<ON_UuidIndex>(src),
                       m_sorted_count(src.m_sorted_count),
                       m_removed_count(src.m_removed_count)
{
}

ON_UuidIndexList& ON_UuidIndexList::operator=(const ON_UuidIndexList& src)
{
  if ( this != &src)
  {
    ON_SimpleArray<ON_UuidIndex>::operator=(src);
    m_sorted_count = src.m_sorted_count;
    m_removed_count = src.m_removed_count;
  }
  return *this;
}

bool ON_UuidIndexList::AddUuidIndex(ON_UUID uuid, int index, bool bCheckForDupicates)
{
  bool rc = bCheckForDupicates ? !FindUuid(uuid,NULL) : true;
  if (rc)
  {
    if ( ON_max_uuid == uuid )
      rc = 0;
    else
    {
      ON_UuidIndex& ui = AppendNew();
      ui.m_id = uuid;
      ui.m_i = index;
    }
  }
  return rc;
}

int ON_UuidIndexList::Count() const
{
  return m_count - m_removed_count;
}

void ON_UuidIndexList::Empty()
{
  m_count = 0;
  m_sorted_count = 0;
  m_removed_count = 0;
}

void ON_UuidIndexList::Reserve( int capacity )
{
  if( m_capacity < capacity )
    SetCapacity( capacity );
}

bool ON_UuidIndexList::RemoveUuid(ON_UUID uuid)
{
  ON_UuidIndex* p = SearchHelper(&uuid);
  if ( 0 != p )
  {
    p->m_id = ON_max_uuid;
    m_removed_count++;
    unsigned int i = (unsigned int)(p - m_a);
    if ( i < m_sorted_count )
      m_sorted_count = i;
  }
  return (0!=p);
}

static
int compar_uuidindex_uuid(const ON_UuidIndex* a, const ON_UuidIndex* b)
{
  return ON_UuidList::CompareUuid(&a->m_id,&b->m_id);
}

bool ON_UuidIndexList::FindUuid(ON_UUID uuid, int* index) const
{
  const ON_UuidIndex* ui = SearchHelper(&uuid);
  if (ui && index)
  {
    *index = ui->m_i;
  }
  return (0!=ui);
}

bool ON_UuidIndexList::FindUuidIndex(ON_UUID uuid, int index) const
{
  const ON_UuidIndex* ui = SearchHelper(&uuid);
  if (ui && index != ui->m_i)
  {
    ui = 0;
  }
  return (0!=ui);
}





ON_UuidPairList::ON_UuidPairList() 
: ON_SimpleArray<ON_UuidPair>(32)
, m_sorted_count(0)
, m_removed_count(0)
{
}

ON_UuidPairList::ON_UuidPairList(int capacity) 
: ON_SimpleArray<ON_UuidPair>(capacity>32?capacity:32)
, m_sorted_count(0)
, m_removed_count(0)
{
}

ON_UuidPairList::~ON_UuidPairList()
{
  m_sorted_count = 0;
  m_removed_count = 0;
}

ON_UuidPairList::ON_UuidPairList(const ON_UuidPairList& src) 
: ON_SimpleArray<ON_UuidPair>(src)
, m_sorted_count(src.m_sorted_count)
, m_removed_count(src.m_removed_count)
{
}

ON_UuidPairList& ON_UuidPairList::operator=(const ON_UuidPairList& src)
{
  if ( this != &src)
  {
    ON_SimpleArray<ON_UuidPair>::operator=(src);
    m_sorted_count = src.m_sorted_count;
    m_removed_count = src.m_removed_count;
  }
  return *this;
}

bool ON_UuidPairList::AddPair(ON_UUID id1, ON_UUID id2, bool bCheckForDupicates)
{
  bool rc = bCheckForDupicates ? !FindId1(id1,0) : true;
  if (rc)
  {
    if ( ON_max_uuid == id1 && ON_max_uuid == id2 )
    {
      // The value pair (ON_max_uuid,ON_max_uuid) is used
      // to mark removed elements.
      rc = false;
    }
    else
    {
      ON_UuidPair& ui = AppendNew();
      ui.m_uuid[0] = id1;
      ui.m_uuid[1] = id2;
    }
  }
  return rc;
}

int ON_UuidPairList::Count() const
{
  return m_count - m_removed_count;
}

void ON_UuidPairList::Empty()
{
  m_count = 0;
  m_sorted_count = 0;
  m_removed_count = 0;
}


void ON_UuidPairList::Reserve( int capacity )
{
  if( m_capacity < capacity )
    SetCapacity( capacity );
}

bool ON_UuidPairList::RemovePair(ON_UUID id1)
{
  ON_UuidPair* p = SearchHelper(&id1);
  if ( 0 != p )
  {
    p->m_uuid[0] = ON_max_uuid;
    p->m_uuid[1] = ON_max_uuid;
    m_removed_count++;
    unsigned int i = (unsigned int)(p - m_a);
    if ( i < m_sorted_count )
      m_sorted_count = i;
  }
  return (0!=p);
}

bool ON_UuidPairList::RemovePair(ON_UUID id1, ON_UUID id2)
{
  ON_UuidPair* p = SearchHelper(&id1);
  if ( 0 != p && p->m_uuid[1] == id2)
  {
    p->m_uuid[0] = ON_max_uuid;
    p->m_uuid[1] = ON_max_uuid;
    m_removed_count++;
    unsigned int i = (unsigned int)(p - m_a);
    if ( i < m_sorted_count )
      m_sorted_count = i;
  }
  return (0!=p);
}

static
int compar_uuidpair_id1(const ON_UuidPair* a, const ON_UuidPair* b)
{
  return ON_UuidList::CompareUuid(&a->m_uuid[0],&b->m_uuid[0]);
}

static
int compar_uuidpair_id1id2(const ON_UuidPair* a, const ON_UuidPair* b)
{
  int i;
  if ( 0 == (i = ON_UuidList::CompareUuid(&a->m_uuid[0],&b->m_uuid[0])) )
    i = ON_UuidList::CompareUuid(&a->m_uuid[1],&b->m_uuid[1]);
  return i;
}

bool ON_UuidPairList::FindId1(ON_UUID id1, ON_UUID* id2) const
{
  const ON_UuidPair* ui = SearchHelper(&id1);
  if (ui && id2)
  {
    *id2 = ui->m_uuid[1];
  }
  return (0!=ui);
}

bool ON_UuidPairList::FindPair(ON_UUID id1, ON_UUID id2) const
{
  const ON_UuidPair* ui = SearchHelper(&id1);
  if (ui && id2 != ui->m_uuid[1])
  {
    ui = 0;
  }
  return (0!=ui);
}

int ON_UuidPairList::GetId1s(
    ON_SimpleArray<ON_UUID>& uuid_list
    ) const
{
  const int count0 = uuid_list.Count();
  int i;
  uuid_list.Reserve(uuid_list.Count() + m_count - m_removed_count);
  for ( i = 0; i < m_count; i++ )
  {
    if ( ON_max_uuid == m_a[i].m_uuid[0] && ON_max_uuid == m_a[i].m_uuid[1] )
      continue;
    uuid_list.Append(m_a[i].m_uuid[0]);
  }
  return uuid_list.Count() - count0;
}

void ON_UuidPairList::ImproveSearchSpeed()
{
  if ( ((unsigned int)m_count) > m_sorted_count )
  {
    QuickSort(compar_uuidpair_id1id2);
    if ( m_removed_count > 0 )
    {
      // cull removed items.  These get sorted to the
      // end because the removed uuid is the largest
      // possible uuid.
      ON_UuidPair removed_uuid;
      removed_uuid.m_uuid[0] = ON_max_uuid;
      removed_uuid.m_uuid[1] = ON_max_uuid;
      while ( m_count > 0 && !compar_uuidpair_id1id2(&removed_uuid,m_a+(m_count-1)))
      {
        m_count--;
      }
      m_removed_count = 0;
    }
    m_sorted_count = m_count;
  }
}

ON_UuidPair* ON_UuidPairList::SearchHelper(const ON_UUID* id1) const
{
  if ( m_count - m_sorted_count > 8 || m_removed_count > 0 )
  {
    // time to resort the array so that the speedy
    // bsearch() can be used to find uuids
    const_cast<ON_UuidPairList*>(this)->ImproveSearchSpeed();
  }

  // NOTE: 
  //   The pair (ON_max_uuid,ON_max_uuid) never appears in the sorted portion of the array.
  ON_UuidPair* p = (m_sorted_count > 0 )
                   ? (ON_UuidPair*)bsearch( id1, m_a, m_sorted_count, 
                                        sizeof(m_a[0]), 
                                        (int(*)(const void*,const void*))compar_uuidpair_id1 ) 
                   : 0;

  if (0 == p)
  {
    // do a slow search on the last m_count-m_sort_count elements
    // in the array.
    int i;
    for ( i = m_sorted_count; i < m_count; i++ )
    {
      if (    0 == ON_UuidList::CompareUuid(id1,&m_a[i].m_uuid[0]) 
           && (ON_max_uuid != m_a[i].m_uuid[0] || ON_max_uuid != m_a[i].m_uuid[1])
         )
      {
        p = m_a+i;
        break;
      }
    }
  }

  return p;
}

void ON_UuidList::RemapUuids( const ON_SimpleArray<ON_UuidPair>& uuid_remap )
{
  if( m_count > 0 && uuid_remap.Count() > 0 )
  {
    bool bRemapped = false;
    int i, j;
    for ( i = 0; i < m_count; i++ )
    {
      j = uuid_remap.BinarySearch( (const ON_UuidPair*)&m_a[i], ON_UuidPair::CompareFirstUuid );
      if ( j >= 0 )
      {
        if ( ON_max_uuid == m_a[i] )
          continue;
        m_sorted_count = 0;
        bRemapped = true;
        m_a[i] = uuid_remap[j].m_uuid[1];
        if ( ON_max_uuid == m_a[i] )
          m_removed_count++;
      }
    }

    if ( bRemapped )
    {
      m_sorted_count = 0;
      SortHelper();
      for ( i = m_count-1; i > 0; i-- )
      {
        if ( m_a[i] == m_a[i-1] )
        {
          Remove(i);
          m_sorted_count--;
        }
      }
    }
  }
}

int ON_UuidList::GetUuids(
    ON_SimpleArray<ON_UUID>& uuid_list
    ) const
{
  const int count0 = uuid_list.Count();
  int i;
  uuid_list.Reserve(uuid_list.Count() + (m_count-m_removed_count));
  for ( i = 0; i < m_count; i++ )
  {
    if ( ON_max_uuid == m_a[i] )
      continue;
    uuid_list.Append(m_a[i]);
  }
  return uuid_list.Count() - count0;
}

int ON_UuidIndexList::GetUuids(
    ON_SimpleArray<ON_UUID>& uuid_list
    ) const
{
  const int count0 = uuid_list.Count();
  int i;
  uuid_list.Reserve(uuid_list.Count() + m_count);
  for ( i = 0; i < m_count; i++ )
  {
    if ( ON_max_uuid == m_a[i].m_id )
      continue;
    uuid_list.Append(m_a[i].m_id);
  }
  return uuid_list.Count() - count0;
}

void ON_UuidIndexList::ImproveSearchSpeed()
{
  if ( ((unsigned int)m_count) > m_sorted_count )
  {
    QuickSort(compar_uuidindex_uuid);
    if ( m_removed_count > 0 )
    {
      // cull removed items.  These get sorted to the
      // end because the removed uuid is the largest
      // possible uuid.
      ON_UuidIndex removed_uuid;
      removed_uuid.m_id = ON_max_uuid;
      removed_uuid.m_i = 0;
      while ( m_count > 0 && !compar_uuidindex_uuid(&removed_uuid,m_a+(m_count-1)))
      {
        m_count--;
      }
      m_removed_count = 0;
    }
    m_sorted_count = m_count;
  }
}

ON_UuidIndex* ON_UuidIndexList::SearchHelper(const ON_UUID* uuid) const
{
  if ( m_count - m_sorted_count > 8 || m_removed_count > 0 )
  {
    // time to resort the array so that the speedy
    // bsearch() can be used to find uuids
    const_cast<ON_UuidIndexList*>(this)->ImproveSearchSpeed();
  }

  ON_UuidIndex* p = (m_sorted_count > 0 )
                   ? (ON_UuidIndex*)bsearch( uuid, m_a, m_sorted_count, 
                                        sizeof(m_a[0]), 
                                        (int(*)(const void*,const void*))compar_uuidindex_uuid ) 
                   : 0;
  if (0 == p)
  {
    // do a slow search on the last m_count-m_sort_count elements
    // in the array.
    int i;
    for ( i = m_sorted_count; i < m_count; i++ )
    {
      if ( 0 == ON_UuidList::CompareUuid(uuid,&m_a[i].m_id) )
      {
        p = m_a+i;
        break;
      }
    }
  }

  return p;
}

ON_2dexMap::ON_2dexMap() : m_bSorted(0)
{}

ON_2dexMap::ON_2dexMap(int capacity) 
            : ON_SimpleArray<ON_2dex>(capacity), 
            m_bSorted(0)
{}

ON_2dexMap::~ON_2dexMap()
{}

int ON_2dexMap::Count() const
{
  return m_count;
}

const ON_2dex* ON_2dexMap::Array() const
{
  return m_a;
}

void ON_2dexMap::Reserve(int capacity )
{
  ON_SimpleArray<ON_2dex>::Reserve(capacity);
}

ON_2dex ON_2dexMap::operator[](int i) const
{
  return m_a[i];
}

void ON_2dexMap::Create(int count, int i0, int j)
{
  if ( count <= 0 )
  {
    m_count = 0;
  }
  else
  {
    ON_SimpleArray<ON_2dex>::Reserve(count);
    m_count = count;
    ON_2dex* a = m_a;
    ON_2dex d;
    d.j = j;
    count += i0;
    for ( d.i = i0; d.i < count; d.i++ )
    {
      *a++ = d;
    }
  }
  m_bSorted = true;
}


const ON_2dex* ON_BinarySearch2dexArray( int key_i, const ON_2dex* base, size_t nel )
{
  if (nel > 0 && base )
  {
    size_t i;
    int base_i;

    // The end tests are not necessary, but they
    // seem to provide overall speed improvement
    // for the types of searches that call this
    // function.

    // 26 January 2012 Dale Lear
    //   Part of the fix for http://dev.mcneel.com/bugtrack/?q=97693
    //   When the values of key_i and base[].i are large,
    //   key_i - base[].i suffers from integer overflow
    //   and the difference can not be use to compare
    //   magnitudes.

    base_i = base[0].i;
    if ( key_i < base_i )
      return 0;
    if ( key_i == base_i )
      return base;

    base_i = base[nel-1].i;
    if ( key_i > base_i )
      return 0;
    if ( key_i == base_i )
      return (base + (nel-1));

    while ( nel > 0 )
    {
      i = nel/2;
      base_i = base[i].i;
      if ( key_i < base_i )
      {
        nel = i;
      }
      else if ( key_i > base_i )
      {
        i++;
        base += i;
        nel -= i;
      }
      else
      {
        return base+i;
      }
    }
  }
  return 0;
}

static
int compare_2dex_i(const void* a, const void* b)
{
  const int ai = *((const int*)a);
  const int bi = *((const int*)b);
  if ( ai < bi )
    return -1;
  if ( ai > bi )
    return 1;
  return 0;
}

const ON_2dex* ON_2dexMap::Find2dex(int i) const
{
  const ON_2dex* e = 0;
  if ( m_count > 0 )
  {
    if ( !m_bSorted )
    {
      ON_qsort(m_a,m_count,sizeof(m_a[0]),compare_2dex_i);
      const_cast<ON_2dexMap*>(this)->m_bSorted = true;
    }
    e = ON_BinarySearch2dexArray(i,m_a,m_count);
  }
  return e;
}

int ON_2dexMap::FindIndex( int i, int not_found_rc) const
{
  const ON_2dex* e = Find2dex(i);
  return e ? e->j : not_found_rc;
}

bool ON_2dexMap::AddIndex(  int i, int j )
{
  bool rc = (0 == Find2dex(i));
  if ( rc )
  {
    ON_2dex& d = AppendNew();
    d.i = i;
    d.j = j;
    m_bSorted = ( m_count < 2 || (m_bSorted && m_a[m_count-2].i < i) );
  }
  return rc;
}

bool ON_2dexMap::SetIndex( int i, int j )
{
  ON_2dex* e = const_cast<ON_2dex*>(Find2dex(i));
  if ( e )
  {
    e->j = j;
  }
  return (0!=e);
}

void ON_2dexMap::SetOrAddIndex( int i, int j )
{
  ON_2dex* e = const_cast<ON_2dex*>(Find2dex(i));
  if ( e )
  {
    e->j = j;
  }
  else
  {
    ON_2dex& d = AppendNew();
    d.i = i;
    d.j = j;
    m_bSorted = ( m_count < 2 || (m_bSorted && m_a[m_count-2].i < i) );
  }
}

bool ON_2dexMap::RemoveIndex( int i )
{
  const ON_2dex* e = Find2dex(i);
  if (e)
  {
    int n = (int)(m_a-e);
    for( m_count--; n < m_count; n++ )
      m_a[n] = m_a[n+1];
  }
  return (0 != e);
}


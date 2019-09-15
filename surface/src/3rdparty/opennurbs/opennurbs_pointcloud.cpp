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

ON_OBJECT_IMPLEMENT(ON_PointCloud, ON_Geometry, "2488F347-F8FA-11d3-BFEC-0010830122F0");


ON_3dPoint& ON_PointCloud::operator[](int i)
{
  return m_P[i];
}

const ON_3dPoint& ON_PointCloud::operator[](int i) const
{
  return m_P[i];
}

ON_3dPoint ON_PointCloud::Point( ON_COMPONENT_INDEX ci ) const
{
  return (ON_COMPONENT_INDEX::pointcloud_point == ci.m_type && ci.m_index >= 0 && ci.m_index < m_P.Count() )
    ? m_P[ci.m_index]
    : ON_UNSET_POINT;
}

ON_PointCloud::ON_PointCloud() : m_flags(0)
{
  m_hidden_count=0;
}

ON_PointCloud::ON_PointCloud( int capacity ) : m_P(capacity), m_flags(0)
{
  m_hidden_count=0;
}

ON_PointCloud::ON_PointCloud( const ON_PointCloud& src ) : ON_Geometry(src)
{
  *this = src;
}

ON_PointCloud& ON_PointCloud::operator=( const ON_PointCloud& src )
{
  if ( this != &src ) {
    Destroy();
    ON_Geometry::operator=(src);
    m_P = src.m_P;
    m_H = src.m_H;
    m_C = src.m_C;
    m_N = src.m_N;
    m_hidden_count = src.m_hidden_count;

    m_plane = src.m_plane;
    m_bbox = src.m_bbox;
    m_flags = src.m_flags;
  }
  return *this;
}

ON_PointCloud::~ON_PointCloud()
{
  Destroy();
}

void ON_PointCloud::Destroy()
{
  m_H.Destroy();
  m_C.Destroy();
  m_N.Destroy();
  m_P.Destroy();
  m_hidden_count=0;
  m_flags = 0;
  m_bbox.Destroy();
}

void ON_PointCloud::EmergencyDestroy()
{
  m_P.EmergencyDestroy();
  m_C.EmergencyDestroy();
  m_H.EmergencyDestroy();
  m_N.EmergencyDestroy();
  m_hidden_count=0;
  m_flags = 0;
  m_bbox.Destroy();
}

ON_BOOL32 ON_PointCloud::IsValid( ON_TextLog* ) const
{
  return ( m_P.Count() > 0 ) ? true : false;
}

void ON_PointCloud::Dump( ON_TextLog& dump ) const
{
  int i;
  const bool bHasNormals = HasPointNormals();
  const bool bHasHiddenPoints = (HiddenPointCount() > 0);
  const int point_count = m_P.Count();
  dump.Print("ON_PointCloud: %d points\n",point_count);
  dump.PushIndent();
  for ( i = 0; i < point_count; i++ ) {
    dump.Print("point[%2d]: ",i);
    dump.Print( m_P[i] );
    if ( bHasNormals )
    {
      dump.Print(", normal = ");
      dump.Print(m_N[i]);
    }
    if ( bHasHiddenPoints && m_H[i])
    {
      dump.Print(" (hidden)");
    }
    dump.Print("\n");
  }
  dump.PopIndent();
}

ON_BOOL32 ON_PointCloud::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.Write3dmChunkVersion(1,1);

  if (rc) rc = file.WriteArray( m_P );
  if (rc) rc = file.WritePlane( m_plane );
  if (rc) rc = file.WriteBoundingBox( m_bbox );
  if (rc) rc = file.WriteInt( m_flags);

  // added for 1.1  (7 December 2005)
  if (rc) rc = file.WriteArray(m_N);
  if (rc) rc = file.WriteArray(m_C);

  return rc;
}

ON_BOOL32 ON_PointCloud::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version == 1 ) 
  {
    if (rc) rc = file.ReadArray( m_P );
    if (rc) rc = file.ReadPlane( m_plane );
    if (rc) rc = file.ReadBoundingBox( m_bbox );
    if (rc) rc = file.ReadInt( &m_flags);

    if (rc && minor_version >= 1 )
    {
      if (rc) rc = file.ReadArray( m_N );
      if (rc) rc = file.ReadArray( m_C );
    }
  }
  return rc;
}

ON::object_type ON_PointCloud::ObjectType() const
{
  return ON::pointset_object;
}


int ON_PointCloud::Dimension() const
{
  return 3;
}

ON_BOOL32 ON_PointCloud::GetBBox( // returns true if successful
       double* boxmin,    // minimum
       double* boxmax,    // maximum
       ON_BOOL32 bGrowBox  // true means grow box
       ) const
{
  if ( !m_bbox.IsValid() ) {
    m_P.GetBBox( (double*)&m_bbox.m_min.x, (double*)&m_bbox.m_max.x, false );
  }
  ON_BOOL32 rc = m_bbox.IsValid();
  if (rc) {
    if ( bGrowBox ) {
      if ( boxmin ) {
        if ( boxmin[0] > m_bbox.m_min.x ) boxmin[0] = m_bbox.m_min.x;
        if ( boxmin[1] > m_bbox.m_min.y ) boxmin[1] = m_bbox.m_min.y;
        if ( boxmin[2] > m_bbox.m_min.z ) boxmin[2] = m_bbox.m_min.z;
      }
      if ( boxmax ) {
        if ( boxmax[0] < m_bbox.m_max.x ) boxmax[0] = m_bbox.m_max.x;
        if ( boxmax[1] < m_bbox.m_max.y ) boxmax[1] = m_bbox.m_max.y;
        if ( boxmax[2] < m_bbox.m_max.z ) boxmax[2] = m_bbox.m_max.z;
      }
    }
    else {
      if ( boxmin ) {
        boxmin[0] = m_bbox.m_min.x;
        boxmin[1] = m_bbox.m_min.y;
        boxmin[2] = m_bbox.m_min.z;
      }
      if ( boxmax ) {
        boxmax[0] = m_bbox.m_max.x;
        boxmax[1] = m_bbox.m_max.y;
        boxmax[2] = m_bbox.m_max.z;
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_PointCloud::Transform( 
       const ON_Xform& xform
       )
{
  TransformUserData(xform);
  ON_BOOL32 rc = m_P.Transform(xform);
  if (rc && HasPlane() )
    rc = m_plane.Transform(xform);
  m_bbox.Destroy();
  return rc;
}

bool ON_PointCloud::IsDeformable() const
{
  return true;
}

bool ON_PointCloud::MakeDeformable()
{
  return true;
}

ON_BOOL32 ON_PointCloud::SwapCoordinates(
      int i, int j        // indices of coords to swap
      )
{
  ON_BOOL32 rc = m_P.SwapCoordinates(i,j);
  if ( rc && HasPlane() ) {
    rc = m_plane.SwapCoordinates(i,j);
  }
  if ( rc && m_bbox.IsValid() ) {
    rc = m_bbox.SwapCoordinates(i,j);
  }
  return rc;
}

int ON_PointCloud::PointCount() const
{
  return m_P.Count();
}

void ON_PointCloud::AppendPoint( const ON_3dPoint& pt )
{
  m_P.Append(pt);
}

void ON_PointCloud::InvalidateBoundingBox()
{
  m_bbox.Destroy();
}

void ON_PointCloud::SetOrdered(bool b)
{
  if ( b ) {
    m_flags |= 1;
  }
  else {
    m_flags &= 0xFFFFFFFE;
  }
}

bool ON_PointCloud::IsOrdered() const
{
  return (m_flags & 1) ? true : false;
}

bool ON_PointCloud::HasPlane() const
{
  return ( m_flags&2) ? true : false;
}

void ON_PointCloud::SetPlane( const ON_Plane& plane )
{
  m_plane = plane;
  if ( m_plane.IsValid() ) {
    m_flags |= 2;
  }
  else {
    m_flags &= 0xFFFFFFFD;
  }
}

const ON_Plane& ON_PointCloud::Plane()
{
  return m_plane;
}





double ON_PointCloud::Height(int i)
{
  return (m_P[i] - m_plane.origin)*m_plane.zaxis;
}

bool ON_GetClosestPointInPointList( 
          int point_count,
          const ON_3dPoint* point_list,
          ON_3dPoint P,
          int* closest_point_index
          )
{
  bool rc = false;
  if ( point_count>0 && 0 != point_list && closest_point_index )
  {
    double d = 1.0e300;
    double d2 = 1.0e300;
    double x,e;
    int i;
    int best_i = -1;
    //const double* pl = &point_list[0].x;
    for ( i = point_count; i--; point_list++ )
    {
      e = d2;
      x = point_list->x - P.x;
      e = x*x;
      if ( e >= d2 ) continue;
      x = point_list->y - P.y;
      e += x*x;
      if ( e >= d2 ) continue;
      x = point_list->z - P.z;
      e += x*x;
      if ( e >= d2 ) continue;
      d2 = (1.0+ON_SQRT_EPSILON)*e;
      e = P.DistanceTo(*point_list);
      if ( e < d )
      {
        d = e;
        best_i = point_count-i-1;
      }
    }
    if ( best_i >= 0 )
    {
      if ( closest_point_index )
        *closest_point_index = best_i;
      rc = true;
    }
  }
  return rc;
}

bool ON_3dPointArray::GetClosestPoint( 
          ON_3dPoint P,
          int* closest_point_index,
          double maximum_distance
          ) const
{
  int i;

  bool rc = ON_GetClosestPointInPointList( m_count, m_a , P, &i );

  if (rc)
  {
    if ( maximum_distance > 0.0 && P.DistanceTo(m_a[i]) > maximum_distance )
    {
      rc = false;
    }
    else if ( closest_point_index )
    {
      *closest_point_index = i;
    }
  }

  return rc;
}

bool ON_PointCloud::HasPointColors() const
{
  const int point_count = m_P.Count();
  return (point_count > 0 && point_count == m_C.Count());
}

bool ON_PointCloud::HasPointNormals() const
{
  const int point_count = m_P.Count();
  return (point_count > 0 && point_count == m_N.Count());
}

bool ON_PointCloud::GetClosestPoint(
                ON_3dPoint P,
                int* closest_point_index,
                double maximum_distance 
                ) const
{
  if ( maximum_distance > 0.0 && m_bbox.IsValid() )
  {
    // check bounding box
    if ( m_bbox.MinimumDistanceTo(P) > maximum_distance )
      return false;
  }
  return m_P.GetClosestPoint( P, closest_point_index, maximum_distance );
}

int ON_PointCloud::HiddenPointCount() const
{
  int point_count;
  return (    m_hidden_count > 0 
           && (point_count = m_P.Count()) > 0
           && m_hidden_count < point_count 
           && m_H.Count() == point_count 
           )
           ? m_hidden_count
           : 0;
}

void ON_PointCloud::DestroyHiddenPointArray()
{
  m_hidden_count = 0;
  m_H.Destroy();
}

const bool* ON_PointCloud::HiddenPointArray() const
{
  return (m_hidden_count > 0 && m_H.Count() == m_P.Count()) 
         ? m_H.Array() 
         : 0;
}

void ON_PointCloud::SetHiddenPointFlag( int point_index, bool bHidden )
{
  const int point_count = m_P.Count();
  if ( point_index >= 0 && point_index < point_count )
  {
    if ( bHidden )
    {
      if ( point_count != m_H.Count() )
      {
        m_H.SetCapacity(point_count);
        m_H.SetCount(point_count);
        m_H.Zero();
        m_H[point_index] = true;
        m_hidden_count = 1;
      }
      else if ( false == m_H[point_index] )
      {
        m_H[point_index] = true;
        m_hidden_count++;
      }
    }
    else
    {
      // show this vertex
      if ( m_hidden_count > 0 && point_count == m_H.Count() )
      {
        if  ( m_H[point_index] )
        {
          m_H[point_index] = false;
          m_hidden_count--;
          if ( 0 == m_hidden_count )
          {
            DestroyHiddenPointArray();
          }
        }
      }
      else if ( m_hidden_count > 0 || m_H.Capacity() > 0 )
      {
        // if m_H exists, it is bogus.
        DestroyHiddenPointArray();
      }
    }
  }
}

bool ON_PointCloud::PointIsHidden( int point_index ) const
{
  int point_count;
  return (    point_index >= 0
           && point_index < (point_count = m_P.Count())
           && m_H.Count() == point_count )
           ? m_H[point_index]
           : false;
}


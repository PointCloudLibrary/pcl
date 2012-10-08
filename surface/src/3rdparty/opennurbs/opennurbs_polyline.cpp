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

ON_Polyline::ON_Polyline()
{
}

ON_Polyline::ON_Polyline(const ON_3dPointArray& src) : ON_3dPointArray(src)
{
}


bool ON_Polyline::IsValid( double tolerance ) const
{
  bool rc = (m_count>=2)?true:false;
  int i;
  if ( tolerance > 0.0 )
  {
    for ( i = 1; rc && i < m_count; i++ )
    {
      if ( m_a[i].DistanceTo(m_a[i-1]) <= tolerance )
        rc = false;
    }
    if ( rc && m_count < 4 && m_a[0].DistanceTo(m_a[m_count-1]) <= tolerance )
      rc = false;
  }
  else {
    for ( i = 1; rc && i < m_count && rc; i++ )
    {
      if ( m_a[i] == m_a[i-1] )
        rc = false;
    }
    if ( rc && m_count < 4 && m_a[0] == m_a[m_count-1] )
      rc = false;
  }
  return rc;
}

int ON_Polyline::Clean( double tolerance )
{
  // 14 January 2005 Dale Lear
  //     Fixed this cleaner so that it did not modify
  //     the start and end point.
  int count0 = m_count;

  if ( m_count > 2 )
  {
    int i,j;
    j = 0;
    for ( i = 1; i < m_count-1; i++ )
    {
       if ( m_a[j].DistanceTo(m_a[i]) <= tolerance )
         continue;
       j++;
       if ( i > j )
         m_a[j] = m_a[i];
    }

    if ( m_count > j+2 )
    {
      m_a[j+1] = m_a[m_count-1];
      m_count = j+2;
    }

    while ( m_count > 2 && m_a[m_count-2].DistanceTo(m_a[m_count-1]) <= tolerance )
    {
      m_a[m_count-2] = m_a[m_count-1];
      m_count--;
    }
  }

  return count0-m_count;
}

ON_Polyline& ON_Polyline::operator=(const ON_3dPointArray& src)
{
  ON_3dPointArray::operator=(src);
  return *this;
}

ON_Polyline::~ON_Polyline()
{
}

int ON_Polyline::PointCount() const
{
  return m_count;
}

int ON_Polyline::SegmentCount() const
{
  int i = m_count-1;
  if (i < 0 )
    i = 0;
  return i;
}


bool ON_Polyline::IsClosed( double tolerance ) const
{
  bool rc = false;
  const int count = m_count-1;
  int i;
  if ( count >= 3 )
  {
    if ( tolerance > 0.0 )
    {
      if ( m_a[0].DistanceTo(m_a[count]) <= tolerance ) {
        for ( i = 1; i < count; i++ ) {
          if (   m_a[i].DistanceTo(m_a[0]) > tolerance 
              && m_a[i].DistanceTo(m_a[count]) > tolerance ) 
          {
             rc = true;
             break;
          }
        }
      }
    }
    else
    {
      if ( ON_PointsAreCoincident(3,false,&m_a[0].x,&m_a[count].x) ) 
      {
        for ( i = 1; i < count; i++ ) {
          if (    !ON_PointsAreCoincident(3,false,&m_a[i].x,&m_a[0].x) 
               && !ON_PointsAreCoincident(3,false,&m_a[i].x,&m_a[count].x) 
             )
          {
            rc = true;
            break;
          }
        }
      }
    }
  }
  return rc;
}


double ON_Polyline::Length() const 
{
  const int count = m_count;
  double d = 0;
  int i;
  for ( i = 1; i < count; i++ ) 
  {
    d += m_a[i].DistanceTo(m_a[i-1]);
  }
  return d;
}

ON_3dVector ON_Polyline::SegmentDirection( int segment_index ) const
{
  ON_3dVector v;
  if ( segment_index >= 0 && segment_index < m_count-1 ) {
    v = m_a[segment_index+1] - m_a[segment_index];
  }
  else {
    v.Zero();
  }
  return v;
}

ON_3dVector ON_Polyline::SegmentTangent( int segment_index ) const
{
  ON_3dVector v = SegmentDirection(segment_index);
  v.Unitize();
  return v;
}

ON_3dPoint ON_Polyline::PointAt( double t ) const
{
  const int count = m_count;
  int segment_index = 0;
  if ( count < 0 ) {
    return ON_origin;
  }
  else if (count == 1 ) {
    return m_a[0];
  }
  else {
    segment_index = (int)floor(t);
    if ( segment_index < 0 ) {
      segment_index = 0;
      t = 0.0;
    }
    else if ( segment_index >= count-1 ) {
      segment_index = count-1;
      t = 1.0;
    }
    else {
      t -= ((double)segment_index);
    }
  }

  return (1-t)*m_a[segment_index] + t*m_a[segment_index+1];
}

ON_3dVector ON_Polyline::DerivativeAt( double t ) const
{
  const int count = m_count;
  int segment_index = 0;
  if ( count < 2 )
    return ON_origin;
  else {
    segment_index = (int)floor(t);
    if ( segment_index < 0 )
      segment_index = 0;
    else if ( segment_index >= count-1 )
      segment_index = count-1;
  }
  return m_a[segment_index+1]-m_a[segment_index];
}

ON_3dVector ON_Polyline::TangentAt( double t ) const
{
  ON_3dVector v = DerivativeAt(t);
  v.Unitize();
  return v;
}

bool ON_Polyline::ClosestPointTo( const ON_3dPoint& point, double *t, int segment_index0, int segment_index1 ) const
{
  bool rc = false;
  int segment_index;
  double segment_t, segment_d, best_t, best_d;
  best_t = 0.0; // to keep lint quiet
  best_d = 0.0; // to keep lint quiet
  if ( t ) {
    if ( segment_index0 < 0 )
      segment_index0 = 0;
    if ( segment_index1 > SegmentCount() )
      segment_index1 = SegmentCount();
    for ( segment_index = segment_index0; segment_index < segment_index1; segment_index++ ) {
      double seg_length = m_a[segment_index].DistanceTo(m_a[segment_index + 1]);
      if (seg_length < ON_EPSILON)
        segment_t = 0.0;
      else {
        const ON_3dVector D = SegmentTangent(segment_index);
        const int i = ( point.DistanceTo(m_a[segment_index]) <= point.DistanceTo(m_a[segment_index+1]) ) ? 0 : 1;
        segment_t = (point - m_a[segment_index+i])*D/seg_length;
        if ( i ) {
          segment_t = 1.0 + segment_t;
        }
        if ( segment_t < 0.0 )
          segment_t = 0.0;
        else if (segment_t > 1.0 )
          segment_t = 1.0;
      }
      segment_d = point.DistanceTo((1-segment_t)*m_a[segment_index] + segment_t*m_a[segment_index+1]);
      if ( !rc || segment_d < best_d ) 
      {
        best_t = segment_t + ((double)segment_index);
        best_d = segment_d;
      }
      rc = true;
    }
  }
  if (rc)
    *t = best_t;
  return rc;
}

bool ON_Polyline::ClosestPointTo( const ON_3dPoint& point, double *t ) const
{
  return ClosestPointTo( point, t, 0, SegmentCount() );
}

ON_3dPoint ON_Polyline::ClosestPointTo( const ON_3dPoint& point ) const
{
  double t;
  ON_BOOL32 rc = ClosestPointTo( point, &t );
  if ( !rc )
    t = 0.0;
  return PointAt(t);
}

bool ON_Polyline::CreateInscribedPolygon(
  const ON_Circle& circle,
  int side_count
  )
{
  bool rc = ( circle.IsValid() && side_count >= 3 ) ? true : false;
  if ( rc )
  {
    SetCapacity(side_count+1);
    SetCount(side_count+1);
    double a = 2.0*ON_PI/side_count;
    int i;
    for ( i = 0; i < side_count; i++ )
    {
      m_a[i] = circle.PointAt(a*i);
    }
    m_a[side_count] = m_a[0];
  }
  else
    Destroy();
  return rc;
}

bool ON_Polyline::CreateCircumscribedPolygon(
            const ON_Circle& circle,
            int side_count
            )
{
  bool rc = ( circle.IsValid() && side_count >= 3 ) ? true : false;
  if ( rc )
  {
    SetCapacity(side_count+1);
    SetCount(side_count+1);
    double half_a = ON_PI/side_count;
    int i;
    ON_Circle c = circle;
    c.radius = circle.radius/cos(half_a);
    for ( i = 0; i < side_count; i++ )
    {
      m_a[i] = c.PointAt(half_a*(1+2*i));
    }
    m_a[side_count] = m_a[0];
  }
  else
    Destroy();
  return rc;
}

bool ON_Polyline::CreateStarPolygon(
            const ON_Circle& circle,
            double other_radius,
            int side_count
            )
{
  bool rc = ( circle.IsValid() && side_count >= 3 && other_radius >= 0.0 ) 
          ? true 
          : false;
  if ( rc )
  {
    SetCapacity(2*side_count+1);
    SetCount(2*side_count+1);
    double half_a = ON_PI/side_count;
    int i;
    ON_Circle other_circle = circle;
    other_circle.radius = other_radius;
    for ( i = 0; i < side_count; i++ )
    {
      m_a[i*2]   = circle.PointAt(half_a*2*i);
      m_a[i*2+1] = other_circle.PointAt(half_a*(1+2*i));
    }
    m_a[side_count*2] = m_a[0];
  }
  else
    Destroy();
  return rc;
}


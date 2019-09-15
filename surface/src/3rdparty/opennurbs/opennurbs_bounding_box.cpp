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

ON_BoundingBox::ON_BoundingBox()
                : m_min(1.0,0.0,0.0), 
                  m_max(-1.0,0.0,0.0)
{}

const ON_BoundingBox ON_BoundingBox::EmptyBoundingBox;

ON_BoundingBox::ON_BoundingBox( const ON_3dPoint& min_pt, const ON_3dPoint& max_pt )
                : m_min( min_pt ), 
                  m_max( max_pt )
{}

ON_BoundingBox::~ON_BoundingBox()
{}

void ON_BoundingBox::Destroy()
{
  m_min.Zero();
  m_max.Zero();
  m_min.x = 1.0;
  m_max.x = -1.0;
}

//////////
// ON_BoundingBox::Transform() updates the bounding box
// to be the smallest axis aligned bounding box that contains
// the transform of the eight corner points of the input
// bounding box.
bool ON_BoundingBox::Transform( const ON_Xform& xform )
{
  ON_3dPointArray corners;
  bool rc =  GetCorners( corners );
  if (rc) {
    rc = corners.Transform(xform);
    if (rc)
      rc = Set(corners);
  }
  return rc;
}

double ON_BoundingBox::Tolerance() const
{
  // rough guess at a tolerance to use for comparing
  return ON_BoundingBoxTolerance( 3, m_min, m_max );
}

ON_3dPoint& ON_BoundingBox::operator[](int i)
{
  return (i>0) ? m_max : m_min;
}

const ON_3dPoint& ON_BoundingBox::operator[](int i) const
{
  return (i>0) ? m_max : m_min;
}

ON_3dPoint ON_BoundingBox::Min() const 
{
	return m_min;
}

ON_3dPoint ON_BoundingBox::Max() const 
{
	return m_max;
}

ON_3dVector ON_BoundingBox::Diagonal() const
{
  return m_max - m_min;
}

ON_3dPoint ON_BoundingBox::Center() const
{
  return 0.5*(m_max+m_min);
}

ON_3dPoint ON_BoundingBox::Corner( int x_index, int y_index, int z_index ) const
{
  // 8 corners of box
  // x_index   0 = Min().x, 1 = Max().x
  // y_index   0 = Min().y, 1 = Max().y
  // z_index   0 = Min().z, 1 = Max().z
  ON_3dPoint corner;
  corner.x = (x_index>0) ? m_max.x : m_min.x;
  corner.y = (y_index>0) ? m_max.y : m_min.y;
  corner.z = (z_index>0) ? m_max.z : m_min.z;
  return corner;
}

bool
ON_BoundingBox::GetCorners( 
  ON_3dPoint corners[8]// returns list of 8 corner points
  ) const
{
  int n = 0;
  if ( IsValid() ) 
  {
    ON_3dPoint P;
    int i,j,k;
    for( i = 0; i < 2; i++ ) 
    {
      P.x = (i) ? m_max.x : m_min.x;
      for ( j = 0; j < 2; j++ )
      {
        P.y = (j) ? m_max.y : m_min.y;
        for ( k = 0; k < 2; k++ )
        {
          P.z = (k) ? m_max.z : m_min.z;
          corners[n++] = P;
        }
      }
    }
  }
  return (8==n);
}

bool
ON_BoundingBox::GetCorners( 
  ON_3dPointArray& corners// returns list of 8 corner points
  ) const
{
  ON_3dPoint c[8];
  corners.Empty();
  bool rc = GetCorners(c);
  if ( rc )
    corners.Append(8,c);
  return rc;
}

ON_ClippingRegion::ON_ClippingRegion()
{
  memset(this,0,sizeof(*this));
}

void ON_ClippingRegion::SetClipPlaneTolerance( double clip_plane_tolerance )
{
  if ( clip_plane_tolerance > 0.0 && clip_plane_tolerance < 3.402823466e+38 )
    m_clip_plane_tolerance = (float)clip_plane_tolerance;
  else
    m_clip_plane_tolerance = 0.0;
}

double ON_ClippingRegion::ClipPlaneTolerance() const
{
  return (float)m_clip_plane_tolerance;
}

int ON_ClippingRegion::InViewFrustum( 
  ON_3dPoint P
  ) const
{
  return InViewFrustum(1,&P);
}

int ON_ClippingRegion::InViewFrustum( 
  const ON_BoundingBox& bbox
  ) const
{
  if (    !ON_IsValid(bbox.m_min.x) 
       || !ON_IsValid(bbox.m_max.x) 
       || bbox.m_min.x > bbox.m_max.x
       )
  {
    return 0;
  }

  ON_3dPoint P[8];
  P[0] = bbox.m_min;
  P[1] = bbox.m_max;
  P[2].x = bbox.m_min.x; P[2].y = bbox.m_min.y; P[2].z = bbox.m_max.z;
  P[3].x = bbox.m_min.x; P[3].y = bbox.m_max.y; P[3].z = bbox.m_min.z;
  P[4].x = bbox.m_min.x; P[4].y = bbox.m_max.y; P[4].z = bbox.m_max.z;
  P[5].x = bbox.m_max.x; P[5].y = bbox.m_min.y; P[5].z = bbox.m_min.z;
  P[6].x = bbox.m_max.x; P[6].y = bbox.m_min.y; P[6].z = bbox.m_max.z;
  P[7].x = bbox.m_max.x; P[7].y = bbox.m_max.y; P[7].z = bbox.m_min.z;

  return InViewFrustum(8,P);
}

int ON_ClippingRegion::InViewFrustum( 
  int count, 
  const ON_3fPoint* p
  ) const
{
  const double* xform;
  const float* cv;
  double x, w;
  unsigned int out, all_out, some_out;
  int i;

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  for ( i = count; i--; cv += 3 )
  {
    out = 0;
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3];
    if (x < -w) out  = 0x01; else if (x > w) out  = 0x02;
    x = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7];
    if (x < -w) out |= 0x04; else if (x > w) out |= 0x08;
    x = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11];
    if (x < -w) out |= 0x10; else if (x > w) out |= 0x20;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::InViewFrustum( 
  int count, 
  const ON_3dPoint* p
  ) const
{
  const double* xform;
  const double* cv;
  double x, w;
  unsigned int out, all_out, some_out;
  int i;

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  for ( i = count; i--; cv += 3 )
  {
    out = 0;
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3];
    if (x < -w) out  = 0x01; else if (x > w) out  = 0x02;
    x = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7];
    if (x < -w) out |= 0x04; else if (x > w) out |= 0x08;
    x = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11];
    if (x < -w) out |= 0x10; else if (x > w) out |= 0x20;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::InViewFrustum( 
  int count, 
  const ON_4dPoint* p
  ) const
{
  const double* xform;
  const double* cv;
  double x, w;
  unsigned int out, all_out, some_out;
  int i;

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  for ( i = count; i--; cv += 4 )
  {
    out = 0;
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15]*cv[3];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3]*cv[3];
    if (x < -w) out  = 0x01; else if (x > w) out  = 0x02;
    x = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7]*cv[3];
    if (x < -w) out |= 0x04; else if (x > w) out |= 0x08;
    x = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11]*cv[3];
    if (x < -w) out |= 0x10; else if (x > w) out |= 0x20;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::InClipPlaneRegion( 
  ON_3dPoint P
  ) const
{
  return InClipPlaneRegion(1,&P);
}

int ON_ClippingRegion::InClipPlaneRegion( 
  const ON_BoundingBox& bbox
  ) const
{
  if (    !ON_IsValid(bbox.m_min.x) 
       || !ON_IsValid(bbox.m_max.x) 
       || bbox.m_min.x > bbox.m_max.x
       )
  {
    return 0;
  }

  if ( m_clip_plane_count <= 0 )
  {
    return 2;
  }

  ON_3dPoint P[8];
  P[0] = bbox.m_min;
  P[1] = bbox.m_max;
  P[2].x = bbox.m_min.x; P[2].y = bbox.m_min.y; P[2].z = bbox.m_max.z;
  P[3].x = bbox.m_min.x; P[3].y = bbox.m_max.y; P[3].z = bbox.m_min.z;
  P[4].x = bbox.m_min.x; P[4].y = bbox.m_max.y; P[4].z = bbox.m_max.z;
  P[5].x = bbox.m_max.x; P[5].y = bbox.m_min.y; P[5].z = bbox.m_min.z;
  P[6].x = bbox.m_max.x; P[6].y = bbox.m_min.y; P[6].z = bbox.m_max.z;
  P[7].x = bbox.m_max.x; P[7].y = bbox.m_max.y; P[7].z = bbox.m_min.z;

  return InClipPlaneRegion(8,P);
}

int ON_ClippingRegion::InClipPlaneRegion( 
  int count, 
  const ON_3fPoint* p
  ) const
{
  const ON_PlaneEquation* cpeqn;
  const float* cv;
  double x;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  if ( count <= 0 || !p )
    return 0;

  if ( m_clip_plane_count <= 0 )
    return 2;

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  cv = &p[0].x;
  for ( i = count; i--; cv += 3 )
  {
    out = 0;
    //if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d;
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::InClipPlaneRegion( 
  int count, 
  const ON_3dPoint* p
  ) const
{
  const ON_PlaneEquation* cpeqn;
  const double* cv;
  double x;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  if ( count <= 0 || !p )
    return 0;

  if ( m_clip_plane_count <= 0 )
    return 2;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  cv = &p[0].x;
  for ( i = count; i--; cv += 3 )
  {
    out = 0;
    //if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d;
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::InClipPlaneRegion( 
  int count, 
  const ON_4dPoint* p
  ) const
{
  const ON_PlaneEquation* cpeqn;
  const double* cv;
  double x;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  if ( count <= 0 || !p )
    return 0;

  if ( m_clip_plane_count <= 0 )
    return 2;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  cv = &p[0].x;
  for ( i = count; i--; cv += 4 )
  {
    out = 0;
    //if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d*cv[3];
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::IsVisible( ON_3dPoint P ) const
{
  return IsVisible(1,&P);
}

int ON_ClippingRegion::IsVisible( const ON_BoundingBox& bbox ) const
{
  if (    !ON_IsValid(bbox.m_min.x) 
       || !ON_IsValid(bbox.m_max.x) 
       || bbox.m_min.x > bbox.m_max.x
       )
  {
    return 0;
  }

  ON_3dPoint P[8];
  P[0] = bbox.m_min;
  P[1] = bbox.m_max;
  P[2].x = bbox.m_min.x; P[2].y = bbox.m_min.y; P[2].z = bbox.m_max.z;
  P[3].x = bbox.m_min.x; P[3].y = bbox.m_max.y; P[3].z = bbox.m_min.z;
  P[4].x = bbox.m_min.x; P[4].y = bbox.m_max.y; P[4].z = bbox.m_max.z;
  P[5].x = bbox.m_max.x; P[5].y = bbox.m_min.y; P[5].z = bbox.m_min.z;
  P[6].x = bbox.m_max.x; P[6].y = bbox.m_min.y; P[6].z = bbox.m_max.z;
  P[7].x = bbox.m_max.x; P[7].y = bbox.m_max.y; P[7].z = bbox.m_min.z;

  return IsVisible(8,P);
}


int ON_ClippingRegion::IsVisible( int count, const ON_3fPoint* p ) const
{
  const double* xform;
  const ON_PlaneEquation* cpeqn;
  const float* cv;
  double x, w;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  for ( i = count; i--; cv += 3 )
  {
    out = 0;
    if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d;
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3];
    if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
    x = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7];
    if (x < -w) out |= 0x04; else if (x > w) out |= 0x08;
    x = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11];
    if (x < -w) out |= 0x10; else if (x > w) out |= 0x20;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::IsVisible( int count, const ON_3dPoint* p ) const
{
  const double* xform;
  const ON_PlaneEquation* cpeqn;
  const double* cv;
  double x, w;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  for ( i = count; i--; cv += 3 )
  {
    out = 0;
    if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d;
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3];
    if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
    x = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7];
    if (x < -w) out |= 0x04; else if (x > w) out |= 0x08;
    x = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11];
    if (x < -w) out |= 0x10; else if (x > w) out |= 0x20;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}


int ON_ClippingRegion::IsVisible( int count, const ON_4dPoint* p ) const
{
  const double* xform;
  const ON_PlaneEquation* cpeqn;
  const double* cv;
  double x, w;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  for ( i = count; i--; cv += 4 )
  {
    out = 0;
    if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d*cv[3];
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;
      }
    }
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15]*cv[3];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3]*cv[3];
    if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
    x = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7]*cv[3];
    if (x < -w) out |= 0x04; else if (x > w) out |= 0x08;
    x = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11]*cv[3];
    if (x < -w) out |= 0x10; else if (x > w) out |= 0x20;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

unsigned int ON_ClippingRegion::TransformPoint(
                     const ON_4dPoint& P, 
                     ON_4dPoint& Q
                     ) const
{
  unsigned int out, cpbit;
  const double* xform = &m_xform.m_xform[0][0];
  const double cv[4] = {P.x,P.y,P.z,P.w};
  const ON_PlaneEquation* cpeqn;
  int j;
  double x,y,z,w;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  out = 0;
  if ( m_clip_plane_count )
  {
    cpbit = 0x40;
    cpeqn = m_clip_plane;
    j = m_clip_plane_count;
    while (j--)
    {
      x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d*cv[3];
      if ( x < -clip_plane_tolerance )
        out |= cpbit;
      cpbit <<= 1;
      cpeqn++;
    }
  }
  w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15]*cv[3];
  x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3]*cv[3];
  if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
  y = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7]*cv[3];
  if (y < -w) out |= 0x04; else if (y > w) out |= 0x08;
  z = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11]*cv[3];
  if (z < -w) out |= 0x10; else if (z > w) out |= 0x20;
  if ( w <= 0.0 )
    out = 0x80000000;
  Q.x = x; Q.y = y; Q.z = z; Q.w = w;
  return out;
}

unsigned int ON_ClippingRegion::TransformPoint(
                     const ON_3dPoint& P, 
                     ON_3dPoint& Q
                     ) const
{
  unsigned int out, cpbit;
  const double* xform = &m_xform.m_xform[0][0];
  const double cv[3] = {P.x,P.y,P.z};
  const ON_PlaneEquation* cpeqn;
  int j;
  double x,y,z,w;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  out = 0;
  if ( m_clip_plane_count )
  {
    cpbit = 0x40;
    cpeqn = m_clip_plane;
    j = m_clip_plane_count;
    while (j--)
    {
      x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d;
      if ( x < -clip_plane_tolerance )
        out |= cpbit;
      cpbit <<= 1;
      cpeqn++;
    }
  }
  w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15];
  x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3];
  if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
  y = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7];
  if (y < -w) out |= 0x04; else if (y > w) out |= 0x08;
  z = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11];
  if (z < -w) out |= 0x10; else if (z > w) out |= 0x20;
  if ( w <= 0.0 )
  {
    w = (0.0==w) ? 1.0 : 1.0/w;
    out |= 0x80000000;
  }
  else
  {
    w = 1.0/w;
  }
  Q.x = x*w; Q.y = y*w; Q.z = z*w;
  return out;
}

unsigned int ON_ClippingRegion::TransformPoint(
                     const ON_3fPoint& P, 
                     ON_3dPoint& Q
                     ) const
{
  ON_3dPoint PP(P.x,P.y,P.z);
  return TransformPoint(PP,Q);
}

int ON_ClippingRegion::TransformPoints( int count, ON_4dPoint* p, unsigned int* pflags ) const
{
  // transforms cv's to pick coordinates
  const double* xform;
  const ON_PlaneEquation* cpeqn;
  double* cv;
  double x, y, z, w;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;

  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  i = count;
  while (i--) 
  {
    out = 0;
    if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d*cv[3];
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15]*cv[3];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3]*cv[3];
    if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
    y = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7]*cv[3];
    if (y < -w) out |= 0x04; else if (y > w) out |= 0x08;
    z = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11]*cv[3];
    if (z < -w) out |= 0x10; else if (z > w) out |= 0x20;
    if ( w <= 0.0 )
      out |= 0x80000000;
    some_out |= out;
    all_out  &= out;
    *pflags++ = out;
    *cv++ = x; *cv++ = y; *cv++ = z; *cv++ = w;
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}

int ON_ClippingRegion::TransformPoints( int count, ON_4dPoint* p ) const
{
  // transforms cv's to pick coordinates
  const double* xform;
  const ON_PlaneEquation* cpeqn;
  double* cv;
  double x, y, z, w;
  unsigned int out, all_out, some_out, cpbit;
  int i, j;
  
  // 14 May 2012 Dale Lear
  //   Fix http://dev.mcneel.com/bugtrack/?q=102481
  //   Picking hatches that are coplanar with clipping planes.
  //   The "fix" was to set clipping_plane_tolerance = same
  //   tolerance the display code uses.  Before the fix,
  //   0.0 was used as the clipping_plane_tolerance.
  const double clip_plane_tolerance = ClipPlaneTolerance();

  some_out = 0;
  all_out  = 0xFFFFFFFF;
  xform = &m_xform.m_xform[0][0];
  cv = &p[0].x;
  i = count;
  while (i--) 
  {
    out = 0;
    if ( m_clip_plane_count )
    {
      cpbit = 0x40;
      cpeqn = m_clip_plane;
      j = m_clip_plane_count;
      while (j--)
      {
        x = cpeqn->x*cv[0] + cpeqn->y*cv[1] + cpeqn->z*cv[2] + cpeqn->d*cv[3];
        if ( x < -clip_plane_tolerance )
          out |= cpbit;
        cpbit <<= 1;
        cpeqn++;;
      }
    }
    w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15]*cv[3];
    x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3]*cv[3];
    if (x < -w) out |= 0x01; else if (x > w) out |= 0x02;
    y = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7]*cv[3];
    if (y < -w) out |= 0x04; else if (y > w) out |= 0x08;
    z = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11]*cv[3];
    if (z < -w) out |= 0x10; else if (z > w) out |= 0x20;
    *cv++ = x; *cv++ = y; *cv++ = z; *cv++ = w;
    some_out |= out;
    all_out  &= out;
    if ( some_out && !all_out )
    {
      //  no further "out" checking is necessary
      while (i--)
      {
        x = xform[0]*cv[0] + xform[1]*cv[1] + xform[2]*cv[2] + xform[3]*cv[3];
        y = xform[4]*cv[0] + xform[5]*cv[1] + xform[6]*cv[2] + xform[7]*cv[3];
        z = xform[8]*cv[0] + xform[9]*cv[1] + xform[10]*cv[2] + xform[11]*cv[3];
        w = xform[12]*cv[0] + xform[13]*cv[1] + xform[14]*cv[2] + xform[15]*cv[3];
        *cv++ = x; *cv++ = y; *cv++ = z; *cv++ = w;
      }
      break;
    }
  }

  if ( all_out )
    i = 0;
  else if ( some_out )
    i = 1;
  else
    i = 2;

  return i;
}


bool ON_ClippingRegion::GetLineClipPlaneParamters( 
       ON_4dPoint P0, 
       ON_4dPoint P1, 
       double* t0, 
       double* t1 
       ) const
{
  double s0, s1, x0, x1, s;
  const ON_PlaneEquation* eqn;
  int i;
  if ( m_clip_plane_count )
  {
    s0 = 0.0;
    s1 = 1.0;
    eqn = m_clip_plane;
    const double clip_plane_tolerance = ClipPlaneTolerance();
    for ( i = 0; i < m_clip_plane_count; i++, eqn++ )
    {
      x0 = eqn->x*P0.x + eqn->y*P0.y + eqn->z*P0.z + eqn->d*P0.w;
      x1 = eqn->x*P1.x + eqn->y*P1.y + eqn->z*P1.z + eqn->d*P1.w;
      if ( x0 < 0.0)
      {
        if ( x1 <= 0.0 )
        {
          if ( x0 < -clip_plane_tolerance && x1 <= - clip_plane_tolerance )
            return false;
        }
        if ( x0 != x1 )
        {
          s = x0/(x0-x1);
          if ( s > s0 )
          {
            s0 = s;
            if ( s0 >= s1 )
              return false;
          }
        }
      }
      else if ( x1 < 0.0 )
      {
        if ( x0 <= 0.0 )
        {
          if ( x1 < -clip_plane_tolerance && x0 <= - clip_plane_tolerance )
            return false;
        }
        if ( x0 != x1 )
        {
          s = x1/(x1-x0);
          if ( s < s1 )
          {
            s1 = s;
            if ( s0 >= s1 )
              return false;
          }
        }
      }
    }
    *t0 = s0;
    *t1 = s1;
  }
  else
  {
    *t0 = 0.0;
    *t1 = 1.0;
  }
  return true;
}


int ON_BoundingBox::IsVisible( 
  const ON_Xform& bbox2c
  ) const
{
  int i,j,k;
  unsigned int all_out, some_out, out;
  double bx,by,bz,x,w;
  const double* p;

  if ( !ON_IsValid(m_min.x) || !ON_IsValid(m_max.x) || m_min.x > m_max.x)
    return 0;

  some_out = 0;    // will be != 0 if some portion of box is outside visible region
  all_out  = 0xFFFFFFFF; // will be == 0 if some portion of box is inside visible region
  p = &bbox2c.m_xform[0][0];
  i = 2; bx = m_min.x;
  while(i--)
  {
    j = 2; by = m_min.y;
    while(j--)
    {
      k = 2; bz = m_min.z;
      while(k--)
      {
        w = bx*p[12] + by*p[13] + bz*p[14] + p[15];
        x = bx*p[ 0] + by*p[ 1] + bz*p[ 2] + p[ 3];
        if ( x < -w) out  = 0x01; else if (x > w) out  = 0x02; else out = 0;
        x = bx*p[ 4] + by*p[ 5] + bz*p[ 6] + p[ 7];
        if ( x < -w) out |= 0x04; else if (x > w) out |= 0x08;
        x = bx*p[ 8] + by*p[ 9] + bz*p[10] + p[11];
        if ( x < -w) out |= 0x10; else if (x > w) out |= 0x20;
        some_out |= out;
        all_out  &= out;
        if ( some_out && !all_out )
        {
          // box intersects visble region but is not completely inside it.
          return  1;
        }
        bz = m_max.z;
      }
      by = m_max.y;
    }
    bx = m_max.x;
  }

  return ( all_out ? 0 : 2 );
}

bool ON_BoundingBox::IsPointIn( const ON_3dPoint& p, int bStrictlyIn ) const
{
  bool bIn = false;
  if ( bStrictlyIn ) {
	  bIn = m_min.x<p.x && p.x<m_max.x &&
					m_min.y<p.y && p.y<m_max.y &&
					m_min.z<p.z && p.z<m_max.z;
  }
  else {
	  bIn = m_min.x<=p.x && p.x<=m_max.x &&
					m_min.y<=p.y && p.y<=m_max.y &&
					m_min.z<=p.z && p.z<=m_max.z;
  }
  return bIn;
};

ON_3dPoint ON_BoundingBox::ClosestPoint( 
  const ON_3dPoint& test_point
  ) const
{
  ON_3dPoint near_point = test_point;
	// GBA  30 March 04.  For performance reasons in closest point to surface
	//this function	no longer validates the bounding box. 
  if ( test_point.x < m_min.x )
    near_point.x = m_min.x;
  else if ( test_point.x > m_max.x )
    near_point.x = m_max.x;
  if ( test_point.y < m_min.y )
    near_point.y = m_min.y;
  else if ( test_point.y > m_max.y )
    near_point.y = m_max.y;
  if ( test_point.z < m_min.z )
    near_point.z = m_min.z;
  else if ( test_point.z > m_max.z )
    near_point.z = m_max.z;
  return near_point;
}



int ON_BoundingBox::GetClosestPoint( 
  const ON_Line& line, ON_3dPoint& box_point, double* t0, double* t1
  ) const
{
  if(!IsValid() || !line.IsValid()) 
		return 0;			
	
	ON_3dPoint closest;
	
	if(line.Direction().Length()<=ON_SQRT_EPSILON){
		ON_3dPoint center = line.PointAt(.5);
		if(t0) *t0 = 0.0;
		if(t1) *t1 = 1.0;
		box_point = ClosestPoint(center);
		return IsPointIn( center )? 3 : 1;
	}

	ON_Interval over[3];			//parameter overlap for each direction

	for(int j=0; j<3; j++){
		ON_Interval pl( line[0][j], line[1][j]);
		if( pl[0]!=pl[1])
			over[j]= ON_Interval(	pl.NormalizedParameterAt(Min()[j]),
														pl.NormalizedParameterAt(Max()[j])  );
		else 
			if( Min()[j]<=pl[0] && pl[0]<=Max()[j] )
				over[j]=ON_Interval(-ON_DBL_MAX, ON_DBL_MAX);
			else 
				over[j]=ON_Interval(ON_UNSET_VALUE, ON_UNSET_VALUE);
	}



	// Step 1.  Check for an intersection of the infinte line with the box
	ON_Interval overlap(-ON_DBL_MAX, ON_DBL_MAX);	
	bool nonempty=true;
  int i;
	for( i=0;i<3 && nonempty;i++)
		nonempty = overlap.Intersection(over[i]);
	
	if(nonempty){	// infinte line intersects box
		if( overlap.Intersection( ON_Interval(0,1) ) ){
			// Box & Line segment  intersect
			if(t0) *t0 = overlap[0];
			if(t1) *t1 = overlap[1];
			box_point = line.PointAt(overlap[0]);
			return (overlap.Length()>0)? 3 : 2;
		}
		// closest point is at end of line segment
		double EndInd=(overlap[1]<0)? 0.0: 1.0;
		if(t0) *t0 = EndInd;
		if(t1) *t1 = EndInd;
		return 1;
	}


	//  Step 2.  Check for closest point on box edge and line segment interior
	//   In this case when we project orthogonal to the box edge we get a line 
	//   in the plane that doesn't intersect the 2d-box in the plane.  The projection
  //   of the 3d closest point is the closest point of this 2d closest point problem.
	int k[3];
	for(i=0; i<3; i++)
  {
		// Project box and line onto coord plane with normal Unit(i).

		if(!overlap.Intersection( over[(i+1)%3], over[(i+2)%3] )){	
			// Projected line doesnt intersect the projexted box.  
			// Find the closest  vertex of the projected box.  
			ON_3dVector StdUnit(0,0,0);
			StdUnit[i]=1.0;
			ON_3dVector n = ON_CrossProduct(line.Direction(), StdUnit);
			if(n.Length()==0)
				continue;
			n.Unitize();
			int ilo[3]={0,0,0};
			int ihi[3]={1,1,1};
      int imin[3]={-1,-1,-1};
			double amin=0.0;
			ihi[i]=ilo[i];
			for(k[0]=ilo[0];  k[0]<=ihi[0]; k[0]++)
				for(k[1]=ilo[1];  k[1]<=ihi[1]; k[1]++)
					for(k[2]=ilo[2];  k[2]<=ihi[2]; k[2]++){
						double a = n*(Corner(k[0],k[1],k[2]) - line.from);
						if(amin == 0.0 || fabs(a)<fabs(amin))
            {
							amin= a;
							imin[0]=k[0]; imin[1]=k[1]; imin[2]=k[2];
						}
			}
      if ( imin[0] < 0 )
      {
        return 0;
      }
			ON_3dPoint vertex = Corner(imin[0],imin[1],imin[2]);
			vertex[i] = line.from[i];
			// Solve for 2d-closest point between closest corner and projected line 
			ON_3dVector ProjDir = line.Direction();
			ProjDir[i]=0.0;
			double t = ( vertex - line.from)*ProjDir / ProjDir.LengthSquared();
			ON_3dPoint cp = line.PointAt(t);
			if( 0<=t && t<=1 && m_min[i]<=cp[i] &&cp[i]<= m_max[i] ){
				if(t0) *t0 = t;		// found the  closest point 
				if(t1) *t1 = t;
				vertex[i] =  cp[i];
				box_point = vertex;
				return 1;
			}
		}
	}

	//Step 3. Check each Corner of the box for closest points
	for( k[0]=0; k[0]<2; k[0]++)
		for( k[1]=0; k[1]<2; k[1]++)
			for( k[2]=0; k[2]<2; k[2]++){
				ON_3dPoint corner = Corner(k[0],k[1],k[2]);
				double tstar;
				line.ClosestPointTo( corner, &tstar );
				ON_3dPoint cp = line.PointAt( tstar);
				ON_3dVector disp = cp - corner;
				bool InNCone=true;
				for(int j=0;j<2 && InNCone;j++){
					InNCone = InNCone && ( k[j] )? disp[j]>=0 : disp[j]<=0  ;
				}
				if(InNCone){
					if(t0) *t0 = tstar;
					if(t1) *t1 = tstar;
					box_point = corner;
					return 1;
				} 
	}

	//Step 4. Closest point is at a line end
	for(i=0; i<2; i++){
		closest = ClosestPoint(line[i]);
		double dot = (closest - line[i]) * line.Direction();
		if( (i==0 && dot<= 0) || (i==1 && dot>=0) )
    {
			if(t0) *t0 = i;
			if(t1) *t1 = i;
			box_point = closest;
			return 1;
		}
	}

	ON_ASSERT(false);		//Should never get here
	return 0;  
}				



ON_3dPoint ON_BoundingBox::FarPoint( 
  const ON_3dPoint& test_point
  ) const
{
  ON_3dPoint far_point = test_point;
 // if ( IsValid() ) {
    far_point.x = ( fabs(m_min.x-test_point.x) >= fabs(m_max.x-test_point.x) )
                ? m_min.x : m_max.x;
    far_point.y = ( fabs(m_min.y-test_point.y) >= fabs(m_max.y-test_point.y) )
                ? m_min.y : m_max.y;
    far_point.z = ( fabs(m_min.z-test_point.z) >= fabs(m_max.z-test_point.z) )
                ? m_min.z : m_max.z;
//  }
  return far_point;
}

//TODO:  Replace this static function with an ON_Interval member function.
//  Add to the ON_Interval class in ON_Interval.h
//
// returns true if the intersection is non-empty and sets AB to the intersection
//  bool   GetIntersection(ON_Interval B, ON_Interval& AB);


static bool Intersect( ON_Interval A, ON_Interval B, ON_Interval& AB);

bool Intersect( ON_Interval A, ON_Interval B, ON_Interval& AB){
	if(A.IsDecreasing()) A.Swap();
	if(B.IsDecreasing()) B.Swap();

	bool NotEmpty=true;
	if( A.m_t[0] <= B.m_t[0] && B.m_t[0]<=A.m_t[1] &&  A.m_t[1]<= B.m_t[1]){
		AB.Set(B.m_t[0], A.m_t[1]);
	} else if( B.m_t[0] <= A.m_t[0] && A.m_t[0]<=B.m_t[1] && B.m_t[1]<=A.m_t[1]){
		AB.Set(A.m_t[0], B.m_t[1]);
	} else if( A.m_t[0] <=  B.m_t[0] && B.m_t[0] <= B.m_t[1] && B.m_t[1]<= A.m_t[1]){
		AB.Set(B.m_t[0], B.m_t[1]);
	} else if( B.m_t[0] <= A.m_t[0] && A.m_t[0] <= A.m_t[1] && A.m_t[1]<= B.m_t[1]){
		AB.Set(A.m_t[0], A.m_t[1]);
	} else if( B.m_t[0] <= A.m_t[0]  && A.m_t[0] <= A.m_t[1]  && A.m_t[1]<= B.m_t[1]){
		AB.Set(A.m_t[0], A.m_t[1]);
	} else if(A.m_t[1] < B.m_t[0] || B.m_t[1] < A.m_t[0] ){
		AB.Destroy();
		NotEmpty = false;
	}
	return NotEmpty;
}

bool ON_BoundingBox::GetClosestPoint( 
       const ON_BoundingBox& other_box, // "other" bounding box
       ON_3dPoint& this_point, // point on "this" box that is closest to "other" box
       ON_3dPoint& other_point // point on "other" box that is closest to "this" box
       )  const
{
  ON_BoundingBox b;
	if ( !IsValid() || !other_box.IsValid() )
		return false;

	for (int i=0; i<3; i++ )
  {
		ON_Interval It(m_min[i],m_max[i]);
		ON_Interval Io(other_box.m_min[i],other_box.m_max[i]);
		ON_Interval intersect;
		bool NotEmpty = Intersect(It,Io,intersect);
		if(NotEmpty)
    {
			this_point[i] = other_point[i] = intersect.Mid();
		} 
    else {
			if(m_max[i]< other_box.m_min[i] )
      {
				this_point[i] = m_max[i];
				other_point[i] = other_box.m_min[i];
			} 
      else {
				this_point[i] = m_min[i];
				other_point[i] = other_box.m_max[i];
			}
		}
	}
	return true;
}

//////////
// Get points on bounding boxes that are farthest from each other.
bool ON_BoundingBox::GetFarPoint( 
       const ON_BoundingBox& other_box, // "other" bounding box
       ON_3dPoint& this_point, // point on "this" box that is farthest from "other" box point
       ON_3dPoint& other_point // point on "other" box that is farthest from "this" box point
       )  const
{
	if(!IsValid() || !other_box.IsValid())
		return false;
	for(int i=0; i<3; i++){
		ON_Interval It(m_min[i], m_max[i]);
		ON_Interval Io(other_box.m_min[i], other_box.m_max[i]);
		if( It.Includes(Io) || Io.Includes(It)){
			if( m_max[i] - other_box.m_min[i] > other_box.m_max[i] - m_min[i]){
				this_point[i] = m_max[i];
				other_point[i] = other_box.m_min[i];
			} else {
				this_point[i] = m_min[i];
				other_point[i] = other_box.m_max[i];
			}
		} else {
			if( m_min[i]< other_box.m_min[i]){
				this_point[i]=m_min[i];
			} else {
				other_point[i] = other_box.m_min[i];
			}
			if( m_max[i]> other_box.m_max[i]){
				this_point[i]=m_max[i];
			} else {
				other_point[i] = other_box.m_max[i];
			}
		}
	}
	return true;
}

bool ON_BoundingBox::SwapCoordinates( int i, int j )
{
  bool rc = false;
  if ( IsValid() && 0 <= i && i < 3 && 0 <= j && j < 3 ) {
    rc = true;
    if ( i != j ) {
      double t = m_min[i]; m_min[i] = m_min[j]; m_min[j] = t;
      t = m_max[i]; m_max[i] = m_max[j]; m_max[j] = t;
    }
  }
  return rc;
}

bool ON_BoundingBox::IsDisjoint( const ON_BoundingBox& other_bbox ) const
{
  if ( m_min.x > m_max.x || other_bbox.m_min.x > other_bbox.m_max.x 
       || m_min.x > other_bbox.m_max.x 
       || m_max.x < other_bbox.m_min.x )
  {
    return true;
  }
  if ( m_min.y > m_max.y || other_bbox.m_min.y > other_bbox.m_max.y 
       || m_min.y > other_bbox.m_max.y 
       || m_max.y < other_bbox.m_min.y )
  {
    return true;
  }
  if ( m_min.z > m_max.z || other_bbox.m_min.z > other_bbox.m_max.z 
       || m_min.z > other_bbox.m_max.z 
       || m_max.z < other_bbox.m_min.z )
  {
    return true;
  }
  return false;
}

bool ON_BoundingBox::Intersection(
          const ON_BoundingBox& a
          )
{
  if ( IsValid() && a.IsValid() ) {
    if ( a.m_min.x > m_min.x )
      m_min.x = a.m_min.x;
    if ( a.m_min.y > m_min.y )
      m_min.y = a.m_min.y;
    if ( a.m_min.z > m_min.z )
      m_min.z = a.m_min.z;
    if ( a.m_max.x < m_max.x )
      m_max.x = a.m_max.x;
    if ( a.m_max.y < m_max.y )
      m_max.y = a.m_max.y;
    if ( a.m_max.z < m_max.z )
      m_max.z = a.m_max.z;
  }
  else {
    Destroy();
  }
  return IsValid();
}

bool ON_BoundingBox::Intersection(				//Returns true when intersect is non-empty. 
				 const ON_Line& line,		//Infinite Line segment to intersect with 
				 double* t0 ,			// t0  parameter of first intersection point
				 double* t1       // t1  parameter of last intersection point (t0<=t1)   
				 ) const
{		 
	ON_Interval t(-ON_DBL_MAX, ON_DBL_MAX), ti, Li;
  const double* boxmin = &m_min.x;
  const double* boxmax = &m_max.x;
  const double* from   = &line.from.x;
  const double* to     = &line.to.x;
	for(int i=0; i<3; i++)
  {
		if( from[i] == to[i] )
    {
			if( from[i] < boxmin[i] || from[i] > boxmax[i] )
				return false;
		} 
    else 
    {
      Li.m_t[0] = from[i];
      Li.m_t[1] = to[i];
			ti.m_t[0] = Li.NormalizedParameterAt( boxmin[i]); 
			ti.m_t[1] = Li.NormalizedParameterAt( boxmax[i]);
			if ( !t.Intersection(ti) )
        return false;
		}
	}	

	if(t0)
		*t0 = t.Min();
	if(t1)
		*t1 = t.Max();
	return true;
}		 

bool ON_BoundingBox::Union(
          const ON_BoundingBox& a
          )
{
  if ( IsValid() ) {
    if ( a.IsValid() ) {
      if ( a.m_min.x < m_min.x )
        m_min.x = a.m_min.x;
      if ( a.m_min.y < m_min.y )
        m_min.y = a.m_min.y;
      if ( a.m_min.z < m_min.z )
        m_min.z = a.m_min.z;
      if ( a.m_max.x > m_max.x )
        m_max.x = a.m_max.x;
      if ( a.m_max.y > m_max.y )
        m_max.y = a.m_max.y;
      if ( a.m_max.z > m_max.z )
        m_max.z = a.m_max.z;
    }
  }
  else if ( a.IsValid() ) {
    *this = a;
  }
  else {
    Destroy();
  }
  return IsValid();
}

bool ON_BoundingBox::Intersection(
          const ON_BoundingBox& a,
          const ON_BoundingBox& b
          )
{
  if ( a.IsValid() && b.IsValid() ) {
    m_min.x = (a.m_min.x >= b.m_min.x) ? a.m_min.x : b.m_min.x;
    m_min.y = (a.m_min.y >= b.m_min.y) ? a.m_min.y : b.m_min.y;
    m_min.z = (a.m_min.z >= b.m_min.z) ? a.m_min.z : b.m_min.z;
    m_max.x = (a.m_max.x <= b.m_max.x) ? a.m_max.x : b.m_max.x;
    m_max.y = (a.m_max.y <= b.m_max.y) ? a.m_max.y : b.m_max.y;
    m_max.z = (a.m_max.z <= b.m_max.z) ? a.m_max.z : b.m_max.z;
  }
  else {
    Destroy();
  }
  return IsValid();
}

bool ON_BoundingBox::Includes( 
    const ON_BoundingBox& other,
    bool bProperSubSet) const
{
	bool rc = true;
	bool proper = false;
	for(int i=0; i<3 && rc ; i++)
  {
		ON_Interval thisI( m_min[i], m_max[i]);
		ON_Interval otherI( other.m_min[i], other.m_max[i]);
		rc = thisI.Includes( otherI );
		if(bProperSubSet && !proper)
    {
			proper = (other.m_min[i] > m_min[i]) || (other.m_max[i] < m_max[i]);
    }
	}
  // 9 December 2004 Dale Lear
  //    fixed bug by changing if(proper) to if(bProperSubSet)
	if(bProperSubSet)
		rc = rc && proper;
	return rc;
}


bool ON_BoundingBox::Union(
          const ON_BoundingBox& a,
          const ON_BoundingBox& b
          )
{
  if ( a.IsValid() ) {
    if ( b.IsValid() ) {
      m_min.x = (a.m_min.x <= b.m_min.x) ? a.m_min.x : b.m_min.x;
      m_min.y = (a.m_min.y <= b.m_min.y) ? a.m_min.y : b.m_min.y;
      m_min.z = (a.m_min.z <= b.m_min.z) ? a.m_min.z : b.m_min.z;
      m_max.x = (a.m_max.x >= b.m_max.x) ? a.m_max.x : b.m_max.x;
      m_max.y = (a.m_max.y >= b.m_max.y) ? a.m_max.y : b.m_max.y;
      m_max.z = (a.m_max.z >= b.m_max.z) ? a.m_max.z : b.m_max.z;
    }
    else {
      *this = a;
    }
  }
  else if ( b.IsValid() ) {
    *this = b;
  }
  else {
    Destroy();
  }
  return IsValid();
}

double ON_BoundingBox::Volume() const
{
  double dx = m_max.x - m_min.x;
  double dy = m_max.y - m_min.y;
  double dz = m_max.z - m_min.z;
  return (dx > 0.0 && dy > 0.0 && dz > 0.0) ? dx*dy*dz : 0.0;
}

double ON_BoundingBox::Area() const
{
  double dx = m_max.x - m_min.x;
  double dy = m_max.y - m_min.y;
  double dz = m_max.z - m_min.z;
  return (dx >= 0.0 && dy >= 0.0 && dz >= 0.0) ? 2.0*(dx*dy + dy*dz + dz*dx) : 0.0;
}

bool ON_BoundingBox::Set(     
    int dim, int is_rat, int count, int stride, 
    const double* points, 
    int bGrowBox
  )
{
  return ON_GetPointListBoundingBox(dim, is_rat, count, stride, points, *this, bGrowBox!=0, 0 );
}

bool ON_BoundingBox::Set ( const ON_3dPoint& P, int bGrowBox )
{
  if ( !bGrowBox || !IsValid() )
  {
    m_min = P;
    m_max = P;    
  }
  else
  {
    if ( P.x < m_min.x ) m_min.x = P.x; else if ( m_max.x < P.x ) m_max.x = P.x;
    if ( P.y < m_min.y ) m_min.y = P.y; else if ( m_max.y < P.y ) m_max.y = P.y;
    if ( P.z < m_min.z ) m_min.z = P.z; else if ( m_max.z < P.z ) m_max.z = P.z;
  }
  return true;
}


bool ON_BoundingBox::Set( const ON_SimpleArray<ON_4dPoint>& a, int bGrowBox )
{
  const int count = a.Count();
  const double* p = (count>0) ? &a.Array()->x : 0;
  return ON_GetPointListBoundingBox(3, 1, count, 4, p, *this, bGrowBox!=0, 0 );
}

bool ON_BoundingBox::Set( const ON_SimpleArray<ON_3dPoint>& a, int bGrowBox )
{
  const int count = a.Count();
  const double* p = (count>0) ? &a.Array()->x : 0;
  return ON_GetPointListBoundingBox(3, 0, count, 3, p, *this, bGrowBox!=0, 0 );
}

bool ON_BoundingBox::Set( const ON_SimpleArray<ON_2dPoint>& a, int bGrowBox )
{
  const int count = a.Count();
  const double* p = (count>0) ? &a.Array()->x : 0;
  return ON_GetPointListBoundingBox(2, 0, count, 2, p, *this, bGrowBox!=0, 0 );
}

ON_BoundingBox ON_PointListBoundingBox(
    int dim, int is_rat, int count, int stride, const double* points
    )
{
  ON_BoundingBox bbox;
  ON_GetPointListBoundingBox( dim, is_rat, count, stride, points, bbox, false, 0 );
  return bbox;
}

bool ON_GetPointListBoundingBox( 
    int dim, int is_rat, int count, int stride, const double* points, 
    ON_BoundingBox& tight_bbox,
    int bGrowBox,
    const ON_Xform* xform
    )
{
  // bounding box workhorse
  bool rc = false;
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }
  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }
  if ( is_rat )
  {
    is_rat = 1;
  }

  if ( count > 0 && dim > 0 && points && (count == 1 || stride >= dim+is_rat) ) 
  {
    ON_BoundingBox bbox;
    ON_3dPoint P(0.0,0.0,0.0);
    double w;
    int i, wi;

    if ( xform && xform->IsIdentity() )
    {
      xform = 0;
    }
    wi = dim;
    if ( dim > 3 )
    {
      dim = 3;
    }

    rc = true;
    if ( is_rat ) 
    {
      // skip bogus starting points
      while ( count > 0 && points[wi] == 0.0 ) 
      {
        count--;
        points += stride;
        rc = false;
      }
      if ( count <= 0 )
        return false;
    }

    memcpy( &bbox.m_min.x, points, dim*sizeof(bbox.m_min.x) );
    if ( is_rat )
    {
      w = 1.0/points[wi];
      bbox.m_min.x *= w; bbox.m_min.y *= w; bbox.m_min.z *= w;
    }
    if ( xform )
    {
      bbox.m_min.Transform(*xform);
    }
    bbox.m_max = bbox.m_min;
    points += stride;
    count--;

    if ( count > 0 ) 
    {
      if ( is_rat )
      {
        // homogeneous rational points
        if ( xform )
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            if ( 0.0 == (w = points[wi]) ) 
            {
              rc = false;
              continue;
            }
            memcpy( &P.x, points, dim*sizeof(P.x) );
            w = 1.0/w;
            P.x *= w; P.y *= w; P.z *= w;
            P.Transform(*xform);
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
          if ( dim < 3 )
          {
            for ( i = dim; i < 3; i++)
            {
              bbox.m_min[i] = 0.0;
              bbox.m_max[i] = 0.0;
            }
          }
        }
        else
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            if ( 0.0 == (w = points[wi]) ) 
            {
              rc = false;
              continue;
            }
            memcpy( &P.x, points, dim*sizeof(P.x) );
            w = 1.0/w;
            P.x *= w; P.y *= w; P.z *= w;
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
        }
      }
      else 
      {
        // bounding box of non-rational points
        if ( xform )
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            memcpy( &P.x, points, dim*sizeof(P.x) );
            P.Transform(*xform);
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
          if ( dim < 3 )
          {
            for ( i = dim; i < 3; i++)
            {
              bbox.m_min[i] = 0.0;
              bbox.m_max[i] = 0.0;
            }
          }
        }
        else
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            memcpy( &P.x, points, dim*sizeof(P.x) );
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
        }
      }
    }

    tight_bbox.Union(bbox);
  }
  else if ( bGrowBox ) 
  {
    // result is still valid if no points are added to a valid input box
    rc = (0 == count);
  }

  return rc;
}

bool ON_GetPointListBoundingBox( 
    int dim, int is_rat, int count, int stride, const float* points, 
    ON_BoundingBox& tight_bbox,
    int bGrowBox,
    const ON_Xform* xform
    )
{
  // bounding box workhorse
  ON_BoundingBox bbox;
  ON_3dPoint P(0.0,0.0,0.0);
  ON_3fPoint Q(0.0,0.0,0.0);
  double w;
  int i, wi;
  bool rc = false;
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }
  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }
  if ( is_rat )
  {
    is_rat = 1;
  }

  if ( count > 0 && dim > 0 && points && (count == 1 || stride >= dim+is_rat) ) 
  {
    if ( xform && xform->IsIdentity() )
    {
      xform = 0;
    }
    wi = dim;
    if ( dim > 3 )
    {
      dim = 3;
    }

    rc = true;
    if ( is_rat ) 
    {
      // skip bogus starting points
      while ( count > 0 && points[wi] == 0.0f ) 
      {
        count--;
        points += stride;
        rc = false;
      }
      if ( count <= 0 )
        return false;
    }

    if ( !bGrowBox  )
    {
      memcpy( &Q.x, points, dim*sizeof(Q.x) );
      bbox.m_min = Q;
      if ( is_rat )
      {
        w = 1.0/points[wi];
        bbox.m_min.x *= w; bbox.m_min.y *= w; bbox.m_min.z *= w;
      }
      if ( xform )
      {
        bbox.m_min.Transform(*xform);
      }
      bbox.m_max = bbox.m_min;
      points += stride;
      count--;
      bGrowBox = true;
    }

    if ( count > 0 ) 
    {
      if ( is_rat )
      {
        // homogeneous rational points
        if ( xform )
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            if ( 0.0 == (w = points[wi]) ) 
            {
              rc = false;
              continue;
            }
            memcpy( &Q.x, points, dim*sizeof(Q.x) );
            w = 1.0/w;
            P.x = w*Q.x; P.y = w*Q.y; P.z = w*Q.z;
            P.Transform(*xform);
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
          if ( dim < 3 )
          {
            for ( i = dim; i < 3; i++)
            {
              bbox.m_min[i] = 0.0;
              bbox.m_max[i] = 0.0;
            }
          }
        }
        else
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            if ( 0.0 == (w = points[wi]) ) 
            {
              rc = false;
              continue;
            }
            memcpy( &Q.x, points, dim*sizeof(Q.x) );
            w = 1.0/w;
            P.x = w*Q.x; P.y = w*Q.y; P.z = w*Q.z;
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
        }
      }
      else 
      {
        // bounding box of non-rational points
        if ( xform )
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            memcpy( &Q.x, points, dim*sizeof(Q.x) );
            P.x = Q.x; P.y = Q.y; P.z = Q.z;
            P.Transform(*xform);
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
          if ( dim < 3 )
          {
            for ( i = dim; i < 3; i++)
            {
              bbox.m_min[i] = 0.0;
              bbox.m_max[i] = 0.0;
            }
          }
        }
        else
        {
          for ( /*empty*/; count--; points += stride ) 
          {
            memcpy( &Q.x, points, dim*sizeof(Q.x) );
            P.x = Q.x; P.y = Q.y; P.z = Q.z;
            if ( bbox.m_min.x > P.x ) bbox.m_min.x = P.x; else if ( bbox.m_max.x < P.x ) bbox.m_max.x = P.x;
            if ( bbox.m_min.y > P.y ) bbox.m_min.y = P.y; else if ( bbox.m_max.y < P.y ) bbox.m_max.y = P.y;
            if ( bbox.m_min.z > P.z ) bbox.m_min.z = P.z; else if ( bbox.m_max.z < P.z ) bbox.m_max.z = P.z;
          }
        }
      }
    }

    tight_bbox.Union(bbox);
  }
  else if ( bGrowBox ) 
  {
    // result is still valid if no points are added to a valid input box
    rc = (0 == count);
  }

  return rc;
}


bool ON_GetPointListBoundingBox( 
    int dim, int is_rat, int count, int stride, const double* points, 
    double* boxmin, double* boxmax,
    int bGrowBox
    )
/*****************************************************************************
Bounding Box of a set of points
 
INPUT:
  dim           ( >= 1 ) dimension of each point
  is_rat        ( true if points are rational )
  count         ( >= 1 ) number of points
  stride        ( >= (is_rat)?(dim+1):dim )
  points        array of dim*count doubles
  boxmin, boxmax      unused arrays of dim doubles
  bGrowBox       true if input box should be enlarged to contain points
										boxmin[i]>boxmax[i] for some i, represents an empty initial box
                 false if input box should be ignored bounding box of points
                       is returned
OUTPUT:
  boxmin, boxmax      diagonal corners of bounding box
*****************************************************************************/
{
  // OBSOLETE
  // bounding box workhorse
  double x, w;
  int j;
  bool rc = false;
  for ( j = 0; j < dim && bGrowBox; j++ )
  {
    if ( boxmin[j] > boxmax[j] )
      bGrowBox = false;
  }

  if ( count > 0 ) 
  {
    if ( is_rat )
    {
      is_rat = 1;
    }
    if ( points && dim > 0 && (count == 1 || stride >= dim+is_rat) ) 
    {
      // input is valid list of a least 1 point

      if ( is_rat ) 
      {
        // bounding box of homogeneous rational points
        rc = true;
        while ( count > 0 && points[dim] == 0.0 ) 
        {
          count--;
          points += stride;
          rc = false;
        }
        if ( count > 0 ) 
        {
          if ( !bGrowBox  )
          {
            ON_ArrayScale( dim, 1.0/points[dim], points, boxmin );
            memcpy( boxmax, boxmin, dim*sizeof(*boxmax) );
            points += stride;
            count--;
            bGrowBox = true;
          }
          if ( count > 0 ) 
          {
            for ( /*empty*/; count--; points += stride ) 
            {
              if ( points[dim] == 0.0 ) {
                rc = false;
                continue;
              }
              w = 1.0/points[dim];
              for ( j = 0; j < dim; j++ ) 
              {
                x = w*points[j];
                if (boxmin[j] > x) 
                  boxmin[j] = x; 
                else if (boxmax[j] < x) 
                  boxmax[j] = x;
              }
            }
          }
        }
      }
      else 
      {
        // bounding box of non-rational points
        rc = true;
        if ( !bGrowBox ) 
        {
          // use first point to initialize box 
          memcpy( boxmin, points, dim*sizeof(*boxmin) );
          memcpy( boxmax, boxmin, dim*sizeof(*boxmax) );
          points += stride;
          count--;
          bGrowBox = true;
        }
        if ( count ) 
        {
          // grow box to contain the rest of the points
          for ( /*empty*/; count--; points += stride ) 
          {
            for ( j = 0; j < dim; j++ ) 
            {
              x = points[j];
              if (boxmin[j] > x) 
                boxmin[j] = x; 
              else if (boxmax[j] < x) 
                boxmax[j] = x;
            }
          }
        }
      }
    }
  }
  else if ( bGrowBox ) 
  {
    // result is still valid if no points are added to a valid input box
    rc = true;
  }
  return rc;
}


ON_BoundingBox ON_PointListBoundingBox(
    int dim, int is_rat, int count, int stride, const float* points
    )
{
  ON_BoundingBox bbox;
  ON_GetPointListBoundingBox( dim, is_rat, count, stride, points, bbox, false, 0 );
  return bbox;
}


bool ON_GetPointListBoundingBox( 
    int dim, int is_rat, int count, int stride, const float* points, 
    float* boxmin, float* boxmax,
    int bGrowBox
    )
/*****************************************************************************
Bounding Box of a set of points
 
INPUT:
  dim           ( >= 1 ) dimension of each point
  is_rat        ( true if points are rational )
  count         ( >= 1 ) number of points
  stride        ( >= (is_rat)?(dim+1):dim )
  points        array of dim*count floats
  boxmin, boxmax      unused arrays of dim floats
  bGrowBox       true if input box should be enlarged to contain points
                 false if input box should be ignored bounding box of points
                       is returned
OUTPUT:
  boxmin, boxmax      diagonal corners of bounding box
*****************************************************************************/
{
  // OBSOLETE
  // bounding box workhorse
  float x;
  double w;
  int j;
  bool rc = false;
  for ( j = 0; j < dim && bGrowBox; j++ )
  {
    if ( boxmin[j] > boxmax[j] )
      bGrowBox = false;
  }
  if ( count > 0 ) 
  {
    if ( is_rat )
      is_rat = 1;
    if ( points && dim > 0 && (count == 1 || stride >= dim+is_rat) ) 
    {
      if ( is_rat ) {
        rc = true;
        while ( count > 0 && points[dim] == 0.0 ) {
          count--;
          points += stride;
          rc = false;
        }
        if ( count > 0 ) {
          if ( !bGrowBox ) 
          {
            ON_ArrayScale( dim, 1.0f/points[dim], points, boxmin );
            memcpy( boxmax, boxmin, dim*sizeof(*boxmax) );
            points += stride;
            count--;
            bGrowBox = true;
          }
          for ( /*empty*/; count--; points += stride ) 
          {
            if ( points[dim] == 0.0 )
              continue;
            w = 1.0/points[dim];
            for ( j = 0; j < dim; j++ ) {
              x = (float)(w*points[j]);
              if (boxmin[j] > x) 
                boxmin[j] = x; 
              else if (boxmax[j] < x) 
                boxmax[j] = x;
            }
          }
        }
      }
      else
      {
        rc = true;
        if ( !bGrowBox ) {
          memcpy( boxmin, points, dim*sizeof(*boxmin) );
          memcpy( boxmax, boxmin, dim*sizeof(*boxmax) );
          points += stride;
          count--;
          bGrowBox = true;
        }
        for ( /*empty*/; count--; points += stride ) 
        {
          for ( j = 0; j < dim; j++ ) {
            x = points[j];
            if (boxmin[j] > x) 
              boxmin[j] = x; 
            else if (boxmax[j] < x) 
              boxmax[j] = x;
          }
        }
      }
    }
  }
  else if ( bGrowBox ) 
  {
    rc = true;
  }
  return rc;
}


ON_BoundingBox ON_PointGridBoundingBox(
        int dim,
        ON_BOOL32 is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        const double* p
    )
{
  ON_BoundingBox bbox;
  if ( dim > 3 )
  {
    // strides control stepping - no need to waste time on coordinates we don't return
    dim = 3;
  }
  ON_GetPointGridBoundingBox( dim, is_rat, 
                              point_count0, point_count1, 
                              point_stride0, point_stride1, p, 
                              &bbox.m_min.x, &bbox.m_max.x, false );
  return bbox;
}


bool ON_GetPointGridBoundingBox(
        int dim,
        int is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        const double* p,
        double* boxmin, double* boxmax,
        int bGrowBox
        )
{
  int i;
  for ( i = 0; i < dim && bGrowBox; i++ )
  {
    if ( boxmin[i] > boxmax[i] )
      bGrowBox = false;
  }
  bool rc = bGrowBox ? true : false;
  for ( i = 0; i < point_count0; i++ ) 
  {
    if ( !ON_GetPointListBoundingBox( dim, is_rat, point_count1, point_stride1, p + i*point_stride0, boxmin, boxmax, bGrowBox ) ) {
      rc = false;
      break;
    }
    else 
    {
      bGrowBox = true;
      if (!i)
        rc = true;
    }
  }
  return rc;
}

bool ON_BeyondSinglePrecision( const ON_BoundingBox& bbox, ON_Xform* xform )
{
  bool rc = false;

  if ( bbox.IsValid() )
  {
    // 31 March 2011:
    //   The values of too_far = 262144.0 and too_big = 1048576.0
    //   are first guesses.  If you changes these values,
    //   you must append a comment containing your name,
    //   the date, the values your are using, a bug number
    //   of a bug report containing a file that demonstrates
    //   why you changed the number.  You must retest all 
    //   previous bugs before committing your changes.
    //
    //   DATE: 31 March 2011
    //   NAME: Dale Lear
    //   COMMENT: First guess at values for too_far and too
    //   VALUES:  too_far = 262144.0 from tests with simple mesh sphere
    //            too_big = 1048576.0
    //   BUG: http://dev.mcneel.com/bugtrack/?q=83437
    const double too_far = 262144.0;  // should be a power of 2
    const double too_big = 1048576.0; // MUST be a power of 2
    bool bTooFar = (    bbox.m_min.x >=  too_far 
                     || bbox.m_min.y >=  too_far 
                     || bbox.m_min.x >=  too_far 
                     || bbox.m_max.x <= -too_far
                     || bbox.m_max.y <= -too_far
                     || bbox.m_max.x <= -too_far
                   );
    bool bTooBig = (   bbox.m_min.x <= -too_big 
                     || bbox.m_min.y <= -too_big 
                     || bbox.m_min.x <= -too_big 
                     || bbox.m_max.x >=  too_big
                     || bbox.m_max.y >=  too_big
                     || bbox.m_max.x >=  too_big
                     );
    if ( bTooFar || bTooBig )
    {
      rc = true;
      if ( 0 != xform )
      {
        ON_3dVector C = bbox.Center();
        // Any modification of coordinates contributes to 
        // less precision in calculations.  These tests 
        // remove small components of translations that 
        // do not help matters and may add more fuzz to 
        // calculations.
        if ( fabs(C.x) <= 100.0 )
          C.x = 0.0;
        if ( fabs(C.y) <= 100.0 )
          C.y = 0.0;
        if ( fabs(C.z) <= 100.0 )
          C.z = 0.0;
        double r = 0.5*bbox.m_max.DistanceTo(bbox.m_min);
        // T = translate center of bbox to origin
        ON_Xform T;
        T.Translation(-C);

        // S = scale to shrink things that are too big 
        //     to have a maximum coordinate of 1024.
        //     The scale is a power of 2 to preserve as much 
        //     precision as possible.
        double s = 1.0;
        if ( r > too_big/16.0 )
        {
          // also apply a power of 2 scale to shrink large 
          // object so its coordinates are <= 1024.0
          s = too_big;
          while ( r > s*1024.0 )
            s *= 2.0;
          s = 1.0/s;
        }
        ON_Xform S(s);

        // xform positions bbox in a region of space
        // where single precision coordinates should
        // work for most calculations.
        *xform = S*T;
      }
    }
  }

  if (!rc && 0 != xform )
    xform->Identity();

  return rc;
}


double ON_BoundingBoxTolerance(
        int dim,
        const double* bboxmin,
        const double* bboxmax
        )
{
  int i;
  double x, tolerance=0.0;

#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// generated by the do {...} while(0) in the ON_ASSERT_OR_RETURN macro.
#pragma warning( disable : 4127 )
#endif

  ON_ASSERT_OR_RETURN( dim > 0 && bboxmin != NULL && bboxmax != NULL,tolerance);
  for ( i = 0; i < dim; i++ ) {
    ON_ASSERT_OR_RETURN(bboxmin[i] <= bboxmax[i],tolerance);
  }

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif

  tolerance = ON_ArrayDistance(dim,bboxmin,bboxmax)*ON_EPSILON;
  for ( i = 0; i < dim; i++ ) {
    x = (bboxmax[i] - bboxmin[i])*ON_SQRT_EPSILON;
    if ( x > tolerance )
      tolerance = x;
    x = (fabs(bboxmax[i]) - fabs(bboxmin[i]))*ON_EPSILON;
    if ( x > tolerance )
      tolerance = x;
  }
  if ( tolerance > 0.0 && tolerance < ON_ZERO_TOLERANCE )
    tolerance = ON_ZERO_TOLERANCE;
  return tolerance;
}

int ON_BoundingBox::IsDegenerate( double tolerance ) const
{
  ON_3dVector diag = Diagonal();
  if ( tolerance < 0.0 )
  {
    // compute scale invarient tolerance
    tolerance = diag.MaximumCoordinate()*ON_SQRT_EPSILON;
  }
  int rc = 0;
  if ( diag.x < 0.0 )
    return 4;
  if ( diag.x <=  tolerance )
    rc++;
  if ( diag.y < 0.0 )
    return 4;
  if ( diag.y <=  tolerance )
    rc++;
  if ( diag.z < 0.0 )
    return 4;
  if ( diag.z <=  tolerance )
    rc++;
  return rc;
}

double ON_BoundingBox::MinimumDistanceTo( const ON_3dPoint& P ) const
{
  // 8 Feb 2005 - new function - not tested yet

  // this function must be fast
  // If Q = any point in box, then
  // P.DistanceTo(Q) >= MinimumDistanceTo(P).
  ON_3dVector V;

  if ( P.x < m_min.x )
    V.x = m_min.x - P.x;
  else if ( P.x > m_max.x )
    V.x = P.x - m_max.x;
  else
    V.x = 0.0;

  if ( P.y < m_min.y )
    V.y = m_min.y - P.y;
  else if ( P.y > m_max.y )
    V.y = P.y - m_max.y;
  else
    V.y = 0.0;

  if ( P.z < m_min.z )
    V.z = m_min.z - P.z;
  else if ( P.z > m_max.z )
    V.z = P.z - m_max.z;
  else
    V.z = 0.0;

  return V.Length();
}

double ON_BoundingBox::MaximumDistanceTo( const ON_3dPoint& P ) const
{
  // this function must be fast
  // If Q = any point in box, then
  // P.DistanceTo(Q) <= MaximumDistanceTo(P).
  ON_3dVector V;

  V.x = ( (P.x < 0.5*(m_min.x+m_max.x)) ? m_max.x : m_min.x) - P.x;
  V.y = ( (P.y < 0.5*(m_min.y+m_max.y)) ? m_max.y : m_min.y) - P.y;
  V.z = ( (P.z < 0.5*(m_min.z+m_max.z)) ? m_max.z : m_min.z) - P.z;

  return V.Length();
}

static double ON_BBoxMinimumDistanceToHelper( const ON_BoundingBox& bbox, ON_Line line )
{
  // 8 Feb 2005 - new function - not tested yet

  // this function must be fast

  // returns 0.0 if the line intersects the box and 
  // returns != 0.0 if the line does not intersect 
  //   returns d > 0.0 if the line misses the box and the minimum dist is >= d.
  //   returns ON_UNSET_VALUE if the line misses the box but the minimum distance
  //   is not easily bounded away from zero.


  double d, t;
  bool bTrimmed;


  // quick check for line.from inside box
  if ( bbox.m_min.x <= line.from.x && line.from.x <= bbox.m_max.x )
  {
    if ( bbox.m_min.y <= line.from.y && line.from.y <= bbox.m_max.y )
    {
      if ( bbox.m_min.z <= line.from.z && line.from.z <= bbox.m_max.z )
      {
        return 0.0;
      }
    }
  }
   
  // quick check for line.to inside box
  if ( bbox.m_min.x <= line.to.x && line.to.x <= bbox.m_max.x )
  {
    if ( bbox.m_min.y <= line.to.y && line.to.y <= bbox.m_max.y )
    {
      if ( bbox.m_min.z <= line.to.z && line.to.z <= bbox.m_max.z )
      {
        return 0.0;
      }
    }
  }

  ON_BoundingBox line_bbox;
  line_bbox.Set(3,false,2,3,&line.from.x,false);

  d = bbox.MinimumDistanceTo(line_bbox);
  if ( d > 0.0 )
    return d;

  if ( bbox.m_min.x <= line_bbox.m_min.x && line_bbox.m_max.x <= bbox.m_max.x )
  {
    if ( bbox.m_min.y <= line_bbox.m_min.y && line_bbox.m_max.y <= bbox.m_max.y )
    {
      // The fact that MinimumDistanceTo(line_bbox) == 0.0 implies
      // that the z-extents of the line intersects this bounding box.
      return 0.0;
    }
    else if ( bbox.m_min.z <= line_bbox.m_min.z && line_bbox.m_max.z <= bbox.m_max.z )
    {
      // The fact that MinimumDistanceTo(line_bbox) == 0.0 implies
      // that the y-extents of the line intersects this bounding box.
      return 0.0;
    }
  }
  else if ( bbox.m_min.y <= line_bbox.m_min.y && line_bbox.m_max.y <= bbox.m_max.y 
            && bbox.m_min.z <= line_bbox.m_min.z && line_bbox.m_max.z <= bbox.m_max.z )
  {
    // The fact that MinimumDistanceTo(line_bbox) == 0.0 implies
    // that the x-extents of the line intersects this bounding box.
    return 0.0;
  }

  d = line.to.x - line.from.x;
  bTrimmed = false;
  if ( d != 0.0 )
  {
    if ( d < 0.0 )
    {
      line.Reverse();
      d = -d;
    }
    d = 1.0/d;
    t = (bbox.m_min.x - line.from.x)*d;
    if( 0.0 < t && t < 1.0 )
    {
      line.from = line.PointAt(t);
      line.from.x = bbox.m_min.x;
      d = line.to.x - line.from.x;
      if ( d != 0.0 )
        d = 1.0/d;
      bTrimmed = true;
    }
    t = (bbox.m_max.x - line.from.x)*d;
    if( 0.0 < t && t < 1.0 )
    {
      line.to = line.PointAt(t);
      line.to.x = bbox.m_max.x;
      bTrimmed = true;
    }
  }

  d = line.to.y - line.from.y;
  if ( d < 0.0 )
  {
    line.Reverse();
    d = -d;
  }

  if ( bTrimmed )
  {
    if ( line.to.y < bbox.m_min.y || line.from.y > bbox.m_max.y )
      return ON_UNSET_VALUE;
    if ( line.from.z < bbox.m_min.z && line.to.z < bbox.m_min.z )
      return ON_UNSET_VALUE;
    if ( line.from.z > bbox.m_max.z && line.to.z > bbox.m_max.z )
      return ON_UNSET_VALUE;
  }

  if ( d > 0.0 )
  {
    d = 1.0/d;
    t = (bbox.m_min.y - line.from.y)*d;
    if( 0.0 < t && t < 1.0 )
    {
      line.from = line.PointAt(t);
      line.from.y = bbox.m_min.y;
      d = line.to.y - line.from.y;
      if ( d != 0.0 )
        d = 1.0/d;
    }
    t = (bbox.m_max.y - line.from.y)*d;
    if( 0.0 < t && t < 1.0 )
    {
      line.to = line.PointAt(t);
      line.to.y = bbox.m_max.y;
    }
  }

  if ( line.from.z < bbox.m_min.z && line.to.z < bbox.m_min.z )
    return ON_UNSET_VALUE;

  if ( line.from.z > bbox.m_max.z && line.to.z > bbox.m_max.z )
    return ON_UNSET_VALUE;

  return 0.0; // some portion of the line hits the box
}

double ON_BoundingBox::MinimumDistanceTo( const ON_Plane& plane ) const
{
  ON_PlaneEquation e;
  e.Create(plane.origin,plane.zaxis);
  return MinimumDistanceTo(e);
}

double ON_BoundingBox::MinimumDistanceTo( const ON_PlaneEquation& e ) const
{
  double t, t0, t1;
  ON_3dPoint P(m_min); // min, min, min
  t0 = t1 = e.ValueAt(P);
  
  P.z = m_max.z; // min, min, max
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  else if (t > t1) 
  {
    t1 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  
  P.y = m_max.y; // min, max, max
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  else if (t > t1) 
  {
    t1 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  
  P.z = m_min.z; // min, max, min
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  else if (t > t1) 
  {
    t1 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }

  P.x = m_max.x; // max, max, min
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  else if (t > t1) 
  {
    t1 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }

  P.y = m_min.y; // max, min, min
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  else if (t > t1) 
  {
    t1 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }

  P.z = m_max.z; // max, min, max
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }
  else if (t > t1) 
  {
    t1 = t;  if ( t0 <= 0.0 && t1 >= 0.0 ) return 0.0;
  }

  P.y = m_max.y; // max, max, max
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t;
  }
  else if (t > t1) 
  {
    t1 = t;
  }

  if ( t0 >= 0.0 ) return t0;
  if ( t1 <= 0.0 ) return -t1;
  return 0.0;
}


double ON_BoundingBox::MaximumDistanceTo( const ON_Plane& plane ) const
{
  ON_PlaneEquation e;
  e.Create(plane.origin,plane.zaxis);
  return MinimumDistanceTo(e);
}

double ON_BoundingBox::MaximumDistanceTo( const ON_PlaneEquation& e ) const
{
  double t, t0;
  ON_3dPoint P(m_min); // min, min, min
  t0 = fabs(e.ValueAt(P));
  
  P.z = m_max.z; // min, min, max
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;
  
  P.y = m_max.y; // min, max, max
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;
  
  P.z = m_min.z; // min, max, min
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;

  P.x = m_max.x; // max, max, min
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;

  P.y = m_min.y; // max, min, min
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;

  P.z = m_max.z; // max, min, max
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;

  P.y = m_max.y; // max, max, max
  t = fabs(e.ValueAt(P));  if (t > t0) t0 = t;

  return t0;
}


bool ON_BoundingBox::IsFartherThan( double d, const ON_Plane& plane ) const
{
  ON_PlaneEquation e;
  e.Create(plane.origin,plane.zaxis);
  return IsFartherThan(d,e);
}

bool ON_BoundingBox::IsFartherThan( double d, const ON_PlaneEquation& e ) const
{
  double t, t0, t1;
  ON_3dPoint P(m_min); // min, min, min
  t0 = t1 = e.ValueAt(P);
  if ( t0 <= d && t1 >= -d ) return false;
  
  P.z = m_max.z; // min, min, max
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  
  P.y = m_max.y; // min, max, max
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  
  P.z = m_min.z; // min, max, min
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }

  P.x = m_max.x; // max, max, min
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }

  P.y = m_min.y; // max, min, min
  t = e.ValueAt(P); 
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }

  P.z = m_max.z; // max, min, max
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }

  P.y = m_max.y; // max, max, max
  if (t < t0) 
  {
    t0 = t; if ( t0 <= d && t1 >= -d ) return false;
  }
  else if (t > t1) 
  {
    t1 = t; if ( t0 <= d && t1 >= -d ) return false;
  }

  return true;
}


double ON_BoundingBox::MinimumDistanceTo( const ON_Line& line ) const
{
  double d = ON_BBoxMinimumDistanceToHelper( *this, line );
  if ( d < 0.0 )
  {
    // At this point we know the line does not intersect the box.
    // To get a lower bound on the shortest distance between the
    // line and the box, we need to compare the line to the
    // edges of the box.

    const ON_BoundingBox line_bbox(line.BoundingBox());
    ON_Line edge;
    double e,t;
    int i,j;

    edge.from.z = m_min.z;
    edge.to.z = m_max.z;
    for ( i = 0; i < 2; i++ )
    {    
      edge.from.x = i?m_min.x:m_max.x;
      if ( d > 0.0 )
      {
        if ( line_bbox.m_min.x - edge.from.x > d )
          continue;
        if ( edge.from.x - line_bbox.m_max.x > d )
          continue;
      }
      edge.to.x = edge.from.x;
      for ( j = 0; j < 2; j++ )
      {
        edge.from.y = j?m_min.y:m_max.y;
        if ( d > 0.0 )
        {
          if ( line_bbox.m_min.y - edge.from.y > d )
            continue;
          if ( edge.from.y - line_bbox.m_max.y > d )
            continue;
        }
        edge.to.y = edge.from.y;
        if ( ON_Intersect(edge,line,&e,&t) )
        {
          if ( e < 0.0 ) e = 0.0; else if (e > 1.0) e = 1.0;
          if ( t < 0.0 ) t = 0.0; else if (t > 1.0) t = 1.0;
          e = edge.PointAt(e).DistanceTo(line.PointAt(t));
          if ( d < 0.0 || e < d )
            d = e;
        }
      }
    }

    edge.from.y = m_min.y;
    edge.to.y = m_max.y;
    for ( i = 0; i < 2; i++ )
    {    
      edge.from.z = i?m_min.z:m_max.z;
      edge.to.z = edge.from.z;
      if ( d > 0.0 )
      {
        if ( line_bbox.m_min.z - edge.from.z > d )
          continue;
        if ( edge.from.z - line_bbox.m_max.z > d )
          continue;
      }
      for ( j = 0; j < 2; j++ )
      {
        edge.from.x = j?m_min.x:m_max.x;
        if ( d > 0.0 )
        {
          if ( line_bbox.m_min.x - edge.from.x > d )
            continue;
          if ( edge.from.x - line_bbox.m_max.x > d )
            continue;
        }
        edge.to.x = edge.from.x;
        if ( ON_Intersect(edge,line,&e,&t) )
        {
          if ( e < 0.0 ) e = 0.0; else if (e > 1.0) e = 1.0;
          if ( t < 0.0 ) t = 0.0; else if (t > 1.0) t = 1.0;
          e = edge.PointAt(e).DistanceTo(line.PointAt(t));
          if ( d < 0.0 || e < d )
            d = e;
        }
      }
    }

    edge.from.x = m_min.x;
    edge.to.x = m_max.x;
    for ( i = 0; i < 2; i++ )
    {    
      edge.from.y = i?m_min.y:m_max.y;
      edge.to.y = edge.from.y;
      if ( d > 0.0 )
      {
        if ( line_bbox.m_min.y - edge.from.y > d )
          continue;
        if ( edge.from.y - line_bbox.m_max.y > d )
          continue;
      }
      for ( j = 0; j < 2; j++ )
      {
        edge.from.z = j?m_min.z:m_max.z;
        edge.to.z = edge.from.z;
        if ( d > 0.0 )
        {
          if ( line_bbox.m_min.z - edge.from.z > d )
            continue;
          if ( edge.from.z - line_bbox.m_max.z > d )
            continue;
        }
        if ( ON_Intersect(edge,line,&e,&t) )
        {
          if ( e < 0.0 ) e = 0.0; else if (e > 1.0) e = 1.0;
          if ( t < 0.0 ) t = 0.0; else if (t > 1.0) t = 1.0;
          e = edge.PointAt(e).DistanceTo(line.PointAt(t));
          if ( d < 0.0 || e < d )
            d = e;
        }
      }
    }

    if ( d < 0.0 )
      d = 0.0;

  }

  return d;
}

double ON_BoundingBox::MaximumDistanceTo( const ON_Line& line ) const
{
  // 8 Feb 2005 - new function - not tested yet

  // this function must be fast
  // If Q = any point on the line and 
  // P = any point in box, then
  // P.DistanceTo(Q) <= MaximumDistanceTo(line).

  double d,dx,dy,dz;
  const double* a;
  int i,j,k;

  d = 0.0;
  a = &line.from.x;
  for ( i = 0; i < 2; i++ )
  {
    dx = fabs(a[0] - (i?m_max.x:m_min.x));
    dx = dx*dx;
    if ( dx <= d )
      continue;
    for ( j = 0; j < 2; j++ )
    {
      dy = fabs(a[1] - (j?m_max.y:m_min.y));
      dy = dx + dy*dy;
      if ( dy <= d )
        continue;
      for ( k = 0; k < 2; k++ )
      {
        dz = fabs(a[2] - (k?m_max.z:m_min.z));          
        dz = dz*dz + dy;
        if ( dz > d )
          d = dz;
      }
    }
  }

  a = &line.to.x;
  for ( i = 0; i < 2; i++ )
  {
    dx = fabs(a[0] - (i?m_max.x:m_min.x));
    dx = dx*dx;
    if ( dx <= d )
      continue;
    for ( j = 0; j < 2; j++ )
    {
      dy = fabs(a[1] - (j?m_max.y:m_min.y));
      dy = dx + dy*dy;
      if ( dy <= d )
        continue;
      for ( k = 0; k < 2; k++ )
      {
        dz = fabs(a[2] - (k?m_max.z:m_min.z));          
        dz = dz*dz + dy;
        if ( dz > d )
          d = dz;
      }
    }
  }

  return sqrt(d);
}

double ON_BoundingBox::MinimumDistanceTo( const ON_BoundingBox& other ) const
{
  // this must be fast
  ON_3dVector V;

  if ( m_min.x > other.m_max.x )
    V.x = m_min.x - other.m_max.x;
  else if ( m_max.x < other.m_min.x )
    V.x = other.m_min.x - m_max.x;
  else
    V.x = 0.0;

  if ( m_min.y > other.m_max.y )
    V.y = m_min.y - other.m_max.y;
  else if ( m_max.y < other.m_min.y )
    V.y = other.m_min.y - m_max.y;
  else
    V.y = 0.0;

  if ( m_min.z > other.m_max.z )
    V.z = m_min.z - other.m_max.z;
  else if ( m_max.z < other.m_min.z )
    V.z = other.m_min.z - m_max.z;
  else
    V.z = 0.0;

  return V.Length();
}


double ON_BoundingBox::MaximumDistanceTo( const ON_BoundingBox& other ) const
{
  // this must be fast
  ON_3dVector V;
  double d;

  V.x = fabs(m_min.x - other.m_max.x);
  d = fabs(m_max.x - other.m_min.x);
  if ( d > V.x )
    V.x = d;

  V.y = fabs(m_min.y - other.m_max.y);
  d = fabs(m_max.y - other.m_min.y);
  if ( d > V.y )
    V.y = d;

  V.z = fabs(m_min.z - other.m_max.z);
  d = fabs(m_max.z - other.m_min.z);
  if ( d > V.z )
    V.z = d;

  return V.Length();
}


bool ON_BoundingBox::IsFartherThan( double d, const ON_3dPoint& P ) const
{
  return (d < MinimumDistanceTo(P));
}

bool ON_BoundingBox::IsFartherThan( double d, const ON_Line& line ) const
{
  ON_BoundingBox bbox = *this;
  bbox.m_min.x -= d;
  bbox.m_min.y -= d;
  bbox.m_min.z -= d;
  bbox.m_max.x += d;
  bbox.m_max.y += d;
  bbox.m_max.z += d;
  d = ON_BBoxMinimumDistanceToHelper( bbox, line );
  // d != 0.0 if and only if line misses the enlarged box
  return (d != 0.0);
}

bool ON_BoundingBox::IsFartherThan( double d, const ON_BoundingBox& other ) const
{
  return (d < MinimumDistanceTo(other));
}




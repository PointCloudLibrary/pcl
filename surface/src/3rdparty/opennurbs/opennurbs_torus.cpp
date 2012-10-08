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

ON_Torus::ON_Torus() : major_radius(0.0), minor_radius(0.0)
{}

ON_Torus::ON_Torus( const ON_Plane& major_plane, double major__radius, double minor__radius )
{
  Create(major_plane,major__radius,minor__radius);
}

ON_Torus::ON_Torus( const ON_Circle& major__circle, double minor__radius )
{
  Create(major__circle,minor__radius);
}

ON_Torus::~ON_Torus()
{}

ON_BOOL32 ON_Torus::IsValid( ON_TextLog* text_log ) const
{
  bool rc=false;
  if ( minor_radius <= 0.0 )
  {
    if ( text_log )
      text_log->Print("ON_Torus.minor_radius = %g (should be > 0)\n",minor_radius);
  }
  else if ( major_radius <= minor_radius)
  {
    if ( text_log )
      text_log->Print("ON_Torus.major_radius = %g (should be > minor_radius=%g)\n",major_radius,minor_radius);
  }
  else if ( !plane.IsValid() )
  {
    if ( text_log )
      text_log->Print("ON_Torus.plane is not valid.\n");
  }
  else
    rc = true;
  return rc;
}

ON_BOOL32 ON_Torus::Create( const ON_Plane& major_plane, double major__radius, double minor__radius )
{
  plane = major_plane;
  major_radius = major__radius;
  minor_radius = minor__radius;
  return IsValid();
}

ON_BOOL32 ON_Torus::Create( const ON_Circle& major__circle, double minor__radius )
{
  return Create( major__circle.plane, major__circle.radius, minor__radius );
}

#define EVAL_SETUP_MINOR \
  const double sin_ma = sin(minor_angle_radians);\
  const double cos_ma = cos(minor_angle_radians)

#define EVAL_SETUP_MAJOR \
  const double sin_MA = sin(major_angle_radians);\
  const double cos_MA = cos(major_angle_radians);\
  const ON_3dVector raxis = cos_MA*plane.xaxis + sin_MA*plane.yaxis

ON_3dPoint ON_Torus::PointAt(double major_angle_radians, double minor_angle_radians) const
{
  EVAL_SETUP_MINOR;
  EVAL_SETUP_MAJOR;
  return ((major_radius + cos_ma*minor_radius)*raxis + sin_ma*minor_radius*plane.zaxis + plane.origin);
}

ON_3dVector ON_Torus::NormalAt(double major_angle_radians, double minor_angle_radians) const
{
  EVAL_SETUP_MINOR;
  EVAL_SETUP_MAJOR;
  return (cos_ma*raxis + sin_ma*plane.zaxis);
}

ON_Circle ON_Torus::MajorCircleRadians(double minor_angle_radians ) const
{
  EVAL_SETUP_MINOR;
  ON_Circle c(plane,major_radius);
  c.radius = major_radius + cos_ma*minor_radius;
  c.plane.origin += sin_ma*minor_radius*plane.zaxis;
  c.plane.UpdateEquation();
  return c;
}

ON_Circle ON_Torus::MajorCircleDegrees(double minor_angle_degrees ) const
{
  return MajorCircleRadians(minor_angle_degrees*ON_PI/180.0);
}

ON_Circle ON_Torus::MinorCircleRadians(double major_angle_radians ) const
{
  EVAL_SETUP_MAJOR;
  ON_Circle c;
  c.plane.xaxis = raxis;
  c.plane.yaxis = plane.zaxis;
  c.plane.zaxis = ON_CrossProduct( c.plane.xaxis, c.plane.yaxis );
  c.plane.origin = plane.origin + major_radius*raxis;
  c.plane.UpdateEquation();
  c.radius = minor_radius;
  return c;
}

ON_Circle ON_Torus::MinorCircleDegrees(double minor_angle_degrees ) const
{
  return MinorCircleRadians(minor_angle_degrees*ON_PI/180.0);
}

ON_3dPoint ON_Torus::Center() const
{
  return plane.origin;
}

ON_3dVector ON_Torus::Axis() const
{
  return plane.zaxis;
}

double ON_Torus::MajorRadius() const
{
  return major_radius;
}

double ON_Torus::MinorRadius() const
{
  return minor_radius;
}

ON_BOOL32 ON_Torus::ClosestPointTo( 
         ON_3dPoint test_point, 
         double* major__angle_radians, 
         double* minor__angle_radians
       ) const
{
  double major_angle_radians, minor_angle_radians;
  const ON_Circle major_circle(plane,major_radius);
  ON_BOOL32 rc = major_circle.ClosestPointTo( test_point, &major_angle_radians );
  if ( rc && minor__angle_radians )
  {
    EVAL_SETUP_MAJOR;
    ON_3dVector v = test_point - major_radius*raxis;
    rc = v.Unitize();
    if ( rc )
    {
      double sma = v*plane.zaxis;
      double cma = v*raxis;
      minor_angle_radians = atan2(sma,cma);
      if ( minor_angle_radians < 0.0 )
        minor_angle_radians += 2.0*ON_PI;
    }
    else
      minor_angle_radians = 0.0;
    *minor__angle_radians = minor_angle_radians;
  }
  if ( major__angle_radians )
    *major__angle_radians = major_angle_radians;
  return rc;
}

ON_3dPoint ON_Torus::ClosestPointTo( ON_3dPoint test_point ) const
{
  const ON_Circle major_circle(plane,major_radius);
  ON_3dPoint C = major_circle.ClosestPointTo( test_point );
  ON_3dVector v = test_point - C;
  if ( !v.Unitize() )
  {
    v = C - plane.origin;
    v.Unitize();
  }
  return C + minor_radius*v;
}


// rotate cylinder about its origin
ON_BOOL32 ON_Torus::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis // axis of rotation
      )
{
  return Rotate(sin_angle, cos_angle, axis, plane.origin );
}

ON_BOOL32 ON_Torus::Rotate(
      double angle,            // angle in radians
      const ON_3dVector& axis // axis of rotation
      )
{
  return Rotate(sin(angle), cos(angle), axis, plane.origin );
}

ON_BOOL32 ON_Torus::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint&  point // center of rotation
      )
{
  return plane.Rotate( sin_angle, cos_angle, axis, point );
}

ON_BOOL32 ON_Torus::Rotate(
      double angle,             // angle in radians
      const ON_3dVector& axis,  // axis of rotation
      const ON_3dPoint&  point  // center of rotation
      )
{
  return Rotate(sin(angle),cos(angle),axis,point);
}

ON_BOOL32 ON_Torus::Translate( const ON_3dVector& delta )
{
  return plane.Translate(delta);
}

ON_BOOL32 ON_Torus::Transform( const ON_Xform& xform )
{
  ON_Circle major_c(plane,major_radius);
  ON_BOOL32 rc = major_c.Transform(xform);
  if (rc)
  {
    double s = (0.0==major_radius) ? 1.0 : major_c.radius/major_radius;
    plane = major_c.plane;
    major_radius = major_c.radius;
    minor_radius *= s;
  }
  return rc;
}


int ON_Torus::GetNurbForm( ON_NurbsSurface& s ) const
{
  int rc = 0;
  ON_RevSurface revsrf;
  if ( RevSurfaceForm(&revsrf) )
  {
    rc = revsrf.GetNurbForm(s);
  }
  else
    s.Destroy();
  return rc;
}

ON_RevSurface* ON_Torus::RevSurfaceForm( ON_RevSurface* srf ) const
{
  if ( srf )
    srf->Destroy();
  ON_RevSurface* pRevSurface = NULL;
  if ( IsValid() )
  {
    ON_Circle circle = MinorCircleRadians(0.0);
    ON_ArcCurve* circle_crv = new ON_ArcCurve(circle);
    if ( srf )
      pRevSurface = srf;
    else
      pRevSurface = new ON_RevSurface();
    pRevSurface->m_angle.Set(0.0,2.0*ON_PI);
    pRevSurface->m_t = pRevSurface->m_angle;
    pRevSurface->m_curve = circle_crv;
    pRevSurface->m_axis.from = plane.origin;
    pRevSurface->m_axis.to = plane.origin + plane.zaxis;
    pRevSurface->m_bTransposed = false;
    double r[2], h[2];
    h[0] = fabs(minor_radius);
    h[1] = -h[0];
    r[0] = fabs(major_radius) + h[0];
    r[1] = -r[0];
    int i, j, k, n=0;
    ON_3dPoint pt[8];
    for (i=0;i<2;i++)
    {
      for (j=0;j<2;j++)
      {
        for (k=0;k<2;k++)
        {
          pt[n++] = plane.PointAt( r[i], r[j], h[k] );
        }
      }
    }
    pRevSurface->m_bbox.Set( 3, 0, 8, 3, &pt[0].x );
  }
  return pRevSurface;
}

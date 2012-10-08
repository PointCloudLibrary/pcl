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

ON_Sphere::ON_Sphere() : radius(0.0)
{}

ON_Sphere::ON_Sphere( const ON_3dPoint& center, double r)
{
  Create(center,r);
}

ON_Sphere::~ON_Sphere()
{}

bool ON_Sphere::IsValid() const
{
  return ( ON_IsValid(radius) && radius > 0.0 && plane.IsValid() ) ? true : false;
}

bool ON_Sphere::Create( const ON_3dPoint& center, double r )
{
  plane = ON_xy_plane;
  plane.origin = center;
  plane.UpdateEquation();
  radius = r;
  return (r > 0.0) ? true : false;
}

ON_3dPoint ON_Sphere::PointAt(double longitude, double latitude) const
{
  return radius*NormalAt(longitude,latitude) + plane.origin;
}

ON_3dVector ON_Sphere::NormalAt(double longitude, double latitude) const
{
  return cos(latitude)*(cos(longitude)*plane.xaxis + sin(longitude)*plane.yaxis) + sin(latitude)*plane.zaxis;
}

ON_Circle ON_Sphere::LatitudeRadians(double a) const
{
  return ON_Circle(PointAt(0.0,a),PointAt(0.5*ON_PI,a),PointAt(ON_PI,a));
}

ON_Circle ON_Sphere::LatitudeDegrees(double a) const
{
  return LatitudeRadians(a*ON_PI/180.0);
}

ON_Circle ON_Sphere::LongitudeRadians(double a) const
{
  return ON_Circle(PointAt(a,0.0),NorthPole(),PointAt(a+ON_PI,0.0));
}

ON_Circle ON_Sphere::LongitudeDegrees(double a) const
{
  return LongitudeRadians(a*ON_PI/180.0);
}

ON_3dPoint ON_Sphere::Center() const
{
  return plane.origin;
}

ON_3dPoint ON_Sphere::NorthPole() const
{
  return PointAt(0.0, 0.5*ON_PI);
}

ON_3dPoint ON_Sphere::SouthPole() const
{
  return PointAt(0.0, -0.5*ON_PI);
}

double ON_Sphere::Radius() const
{
  return radius;
}

double ON_Sphere::Diameter() const
{
  return 2.0*radius;
}

bool ON_Sphere::ClosestPointTo( 
       ON_3dPoint point, 
       double* longitude,
       double* latitude
       ) const
{
  bool rc = true;
  ON_3dVector v = point - plane.origin;
  double h = v*plane.zaxis;
  double x = v*plane.xaxis;
  double y = v*plane.yaxis;
  double r = 1.0;
  if ( x == 0.0 && y == 0.0 ) {
    if ( longitude )
      *longitude = 0.0;
    if ( latitude )
      *latitude = (h>=0.0) ? 0.5*ON_PI : -0.5*ON_PI;
    if ( h == 0.0 )
      rc = false;
  }
  else {
    if ( fabs(x) >= fabs(y) ) {
      r = y/x;
      r = fabs(x)*sqrt(1.0+r*r);
    }
    else {
      r = x/y;
      r = fabs(y)*sqrt(1.0+r*r);
    }
    if ( longitude ) {
      *longitude = atan2(y,x);
      if ( *longitude < 0.0 )
        *longitude += 2.0*ON_PI;
      if ( *longitude < 0.0 || *longitude >= 2.0*ON_PI)
        *longitude = 0.0;
    }
    if ( latitude )
      *latitude = atan(h/r);
  }
  return rc;
}

ON_BoundingBox ON_Sphere::BoundingBox() const
{
  ON_BoundingBox bbox;
  double r = Radius();
  bbox.m_min = Center();
  bbox.m_max = bbox.m_min;
  bbox.m_min.x -= r;
  bbox.m_min.y -= r;
  bbox.m_min.z -= r;
  bbox.m_max.x += r;
  bbox.m_max.y += r;
  bbox.m_max.z += r;
  return bbox;
}

// returns point on cylinder that is closest to given point
ON_3dPoint ON_Sphere::ClosestPointTo( ON_3dPoint point ) const
{
  ON_3dVector v = point - plane.origin;
  v.Unitize();
  return plane.origin + radius*v;
}


// rotate cylinder about its origin
bool ON_Sphere::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis // axis of rotation
      )
{
  return Rotate(sin_angle, cos_angle, axis, plane.origin );
}

bool ON_Sphere::Rotate(
      double angle,            // angle in radians
      const ON_3dVector& axis // axis of rotation
      )
{
  return Rotate(sin(angle), cos(angle), axis, plane.origin );
}

// rotate cylinder about a point and axis
bool ON_Sphere::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint&  point // center of rotation
      )
{
  return plane.Rotate( sin_angle, cos_angle, axis, point );
}

bool ON_Sphere::Rotate(
      double angle,             // angle in radians
      const ON_3dVector& axis,  // axis of rotation
      const ON_3dPoint&  point  // center of rotation
      )
{
  return Rotate(sin(angle),cos(angle),axis,point);
}

bool ON_Sphere::Translate(
      const ON_3dVector& delta
      )
{
  return plane.Translate(delta);
}


bool ON_Sphere::Transform( const ON_Xform& xform )
{
  ON_Circle xc(plane,radius);
  bool rc = xc.Transform(xform);
  if (rc)
  {
    plane = xc.plane;
    radius = xc.radius;
  }
  return rc;
}

int ON_Sphere::GetNurbForm( ON_NurbsSurface& s ) const
{
  int rc = 0;
  if ( IsValid() ) {
    s.Create(3,true,3,3,9,5);
    s.m_knot[0][0] = s.m_knot[0][1] = 0.0;
    s.m_knot[0][2] = s.m_knot[0][3] = 0.5*ON_PI;
    s.m_knot[0][4] = s.m_knot[0][5] = ON_PI;
    s.m_knot[0][6] = s.m_knot[0][7] = 1.5*ON_PI;
    s.m_knot[0][8] = s.m_knot[0][9] = 2.0*ON_PI;

    s.m_knot[1][0] = s.m_knot[1][1] = -0.5*ON_PI;
    s.m_knot[1][2] = s.m_knot[1][3] = 0.0;
    s.m_knot[1][4] = s.m_knot[1][5] = 0.5*ON_PI;

    ON_4dPoint* CV = (ON_4dPoint*)s.m_cv;
    const ON_3dVector x = radius*plane.xaxis;
    const ON_3dVector y = radius*plane.yaxis;
    const ON_3dVector z = radius*plane.zaxis;

    ON_3dPoint p[9] = {plane.origin+x,
                       plane.origin+x+y,
                       plane.origin+y,
                       plane.origin-x+y,
                       plane.origin-x,
                       plane.origin-x-y,
                       plane.origin-y,
                       plane.origin+x-y,
                       plane.origin+x};

    const double w = 1.0/sqrt(2.0);
    double w13;
    int i;
    ON_4dPoint southpole = plane.origin - z;
    ON_4dPoint northpole = plane.origin + z;
    for ( i = 0; i < 8; i++ ) {
      CV[5*i  ] = southpole;
      CV[5*i+1] = p[i] - z;
      CV[5*i+2] = p[i];
      CV[5*i+3] = p[i] + z;
      CV[5*i+4] = northpole;

      if ( i%2) {
        CV[5*i  ].x *= w;
        CV[5*i  ].y *= w;
        CV[5*i  ].z *= w;
        CV[5*i  ].w = w;
        CV[5*i+2].x *= w;
        CV[5*i+2].y *= w;
        CV[5*i+2].z *= w;
        CV[5*i+2].w = w;
        CV[5*i+4].x *= w;
        CV[5*i+4].y *= w;
        CV[5*i+4].z *= w;
        CV[5*i+4].w = w;
        w13 = 0.5;
      }
      else {
        w13 = w;
      }
      CV[5*i+1].x *= w13;
      CV[5*i+1].y *= w13;
      CV[5*i+1].z *= w13;
      CV[5*i+1].w  = w13;

      CV[5*i+3].x *= w13;
      CV[5*i+3].y *= w13;
      CV[5*i+3].z *= w13;
      CV[5*i+3].w  = w13;
    }
    CV[40] = CV[0];
    CV[41] = CV[1];
    CV[42] = CV[2];
    CV[43] = CV[3];
    CV[44] = CV[4];
    rc = 2;
  }
  return rc;
}

ON_RevSurface* ON_Sphere::RevSurfaceForm( ON_RevSurface* srf ) const
{
  return RevSurfaceForm(false,srf);
}

ON_RevSurface* ON_Sphere::RevSurfaceForm( 
  bool bArcLengthParameterization,
  ON_RevSurface* srf 
  ) const
{
  if ( srf )
    srf->Destroy();
  ON_RevSurface* pRevSurface = NULL;
  if ( IsValid() )
  {
    ON_Arc arc;
    arc.plane.origin = plane.origin;
    arc.plane.xaxis = -plane.zaxis;
    arc.plane.yaxis =  plane.xaxis;
    arc.plane.zaxis = -plane.yaxis;
    arc.plane.UpdateEquation();
    arc.radius = radius;
    arc.SetAngleRadians(ON_PI);
    ON_ArcCurve* arc_curve = new ON_ArcCurve( arc, -0.5*ON_PI, 0.5*ON_PI );
    if ( srf )
      pRevSurface = srf;
    else
      pRevSurface = new ON_RevSurface();
    pRevSurface->m_angle.Set(0.0,2.0*ON_PI);
    pRevSurface->m_t = pRevSurface->m_angle;
    pRevSurface->m_curve = arc_curve;
    pRevSurface->m_axis.from = plane.origin;
    pRevSurface->m_axis.to = plane.origin + plane.zaxis;
    pRevSurface->m_bTransposed = false;
    pRevSurface->m_bbox.m_min = plane.origin;
    pRevSurface->m_bbox.m_min.x -= radius;
    pRevSurface->m_bbox.m_min.y -= radius;
    pRevSurface->m_bbox.m_min.z -= radius;
    pRevSurface->m_bbox.m_max = plane.origin;
    pRevSurface->m_bbox.m_max.x += radius;
    pRevSurface->m_bbox.m_max.y += radius;
    pRevSurface->m_bbox.m_max.z += radius;
    if ( bArcLengthParameterization )
    {
      double r = fabs(radius);
      if ( !(r > ON_SQRT_EPSILON) )
        r = 1.0;
      r *= ON_PI;
      pRevSurface->SetDomain(0,0.0,2.0*r);
      pRevSurface->SetDomain(1,-0.5*r,0.5*r);
    }
  }
  return pRevSurface;
}



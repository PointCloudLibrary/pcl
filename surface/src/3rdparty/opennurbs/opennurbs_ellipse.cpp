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

ON_Ellipse::ON_Ellipse() 
{
  radius[0] = radius[1] = 0.0;
}


ON_Ellipse::ON_Ellipse(
    const ON_Plane& p,
    double rx, double ry
    )
{
  Create(p,rx,ry);
}


ON_Ellipse::ON_Ellipse(
    const ON_Circle& c
    )
{
  Create(c);
}

ON_Ellipse::~ON_Ellipse()
{}

ON_Ellipse& ON_Ellipse::operator=(const ON_Circle& c)
{
  Create( c );
  return *this;
}

ON_BOOL32 ON_Ellipse::Create( const ON_Plane& p, double rx, double ry )
{
  plane = p;
  radius[0] = rx;
  radius[1] = ry;
  return IsValid();
}

ON_BOOL32 ON_Ellipse::Create( const ON_Circle& c )
{
  return Create( c.Plane(), c.Radius(), c.Radius() );
}

ON_BOOL32 ON_Ellipse::IsValid() const
{
  return (plane.IsValid() && radius[0] > ON_ZERO_TOLERANCE && radius[1] > ON_ZERO_TOLERANCE) ? true : false;
}

ON_BOOL32 ON_Ellipse::IsCircle() const
{
  double r0 = radius[0];
  return ( ON_IsValid(r0) && fabs(r0-radius[1]) <= fabs(r0)*ON_ZERO_TOLERANCE && IsValid() ) ? true : false;
}

double ON_Ellipse::Radius( int i ) const
{
  return radius[(i)?1:0];
}

const ON_3dPoint& ON_Ellipse::Center() const
{
  return plane.origin;
}

double ON_Ellipse::FocalDistance() const
{
  int i = (fabs(radius[0]) >= fabs(radius[1])) ? 0 : 1;
  const double a = fabs(radius[i]);
  const double b = a > 0.0 ? fabs(radius[1-i])/a : 0.0;
  return a*sqrt(1.0 - b*b);
}

bool ON_Ellipse::GetFoci( ON_3dPoint& F1, ON_3dPoint& F2 ) const
{
  const double f = FocalDistance();
  const ON_3dVector& majorAxis = (radius[0] >= radius[1]) ? plane.xaxis : plane.yaxis;
  F1 = plane.origin + f*majorAxis;
  F2 = plane.origin - f*majorAxis;
  return true;
}

const ON_3dVector& ON_Ellipse::Normal() const
{
  return plane.zaxis;
}

const ON_Plane& ON_Ellipse::Plane() const
{
  return plane;
}

ON_3dPoint ON_Ellipse::PointAt( double t ) const
{
  return plane.PointAt( cos(t)*radius[0], sin(t)*radius[1] );
}

ON_3dVector ON_Ellipse::DerivativeAt( 
                 int d, // desired derivative ( >= 0 )
                 double t // parameter
                 ) const
{
  double r0 = radius[0];
  double r1 = radius[1];
  switch (abs(d)%4) {
  case 0:
    r0 *=  cos(t);
    r1 *=  sin(t);
    break;
  case 1:
    r0 *= -sin(t);
    r1 *=  cos(t);
    break;
  case 2:
    r0 *= -cos(t);
    r1 *= -sin(t);
    break;
  case 3:
    r0 *=  sin(t);
    r1 *= -cos(t);
    break;
  }
  return ( r0*plane.xaxis + r1*plane.yaxis );
}

ON_3dVector ON_Ellipse::TangentAt( 
                 double t // parameter
                 ) const
{
  ON_3dVector T = DerivativeAt( 1, t );
  T.Unitize();
  return T;
}

ON_3dVector ON_Ellipse::CurvatureAt( 
                 double t // parameter
                 ) const
{
  ON_3dVector T, K;
  ON_EvCurvature(DerivativeAt( 1, t ),DerivativeAt( 2, t ),T,K);
  return K;
}

static int distSqToEllipse(void* p, double t, double* f, double* df )
{
  // used in call to TL_NRdbrent().
  double dx, dy, st, ct;
  const double* a = (const double*)p;
  // a[0], a[1] = x/y radii of 2d ellipse
  // (a[2],a[3]) = 2d point
  // f(t) = distance squared from ellipse(t) to 2d point
  ct = cos(t);
  st = sin(t);
  dx = ct*a[0] - a[2];
  dy = st*a[1] - a[3];
  if ( f ) {
    *f = dx*dx + dy*dy;
  }
  if ( df ) {
    *df = 2.0*(dy*a[1]*ct - dx*a[0]*st);
  }
  return 0;
}

#if defined(ON_COMPILER_MSC)
// Disable the MSC /W4 warning
//   C4127: conditional expression is constant
// on the line
//   for(...; true; ... )
//
// This source code is used on many compilers and
// I do not trust all of them to get for(..;;...)
// right.
#pragma warning( push )
#pragma warning( disable : 4127 ) 
#endif

ON_BOOL32 ON_Ellipse::ClosestPointTo( const ON_3dPoint& point, double* t ) const
{
  ON_BOOL32 rc = true;
  if ( t ) {
    ON_2dPoint uv;
    rc = plane.ClosestPointTo( point, &uv.x, &uv.y );
    if ( uv.x == 0.0 ) {
      if ( uv.y == 0.0 ) {
        *t = (radius[0] <= radius[1]) ? 0.0 : 0.5*ON_PI;
        return true;
      }
      if ( uv.y >= radius[1] ) {
        *t = 0.5*ON_PI;
        return true;
      }
      if ( uv.y <= -radius[1] ) {
        *t = 1.5*ON_PI;
        return true;
      }
    }
    else if ( uv.y == 0.0 ) {
      if ( uv.x >= radius[0] ) {
        *t = 0.0;
        return true;
      }
      if ( uv.x <= -radius[0] ) {
        *t = ON_PI;
        return true;
      }
    }
    {
      // use circluar approximation to get a seed value
      double t0, t1;
      *t = atan2( uv.y, uv.x );
      if ( *t < 0.0 )
      {
        *t += 2.0*ON_PI;
        if ( 2.0*ON_PI <= *t)
        {
          // == happens when atan2() <= 0.5*ON_EPSILON*2.0*PI
          *t = 0.0;
        }
      }
      if ( radius[0] != radius[1] ) {
        // set limits for search
        if ( uv.x >= 0.0 ) {
          if ( uv.y >= 0.0 ) {
            // search quadrant I
            t0 = 0.0;
            t1 = 0.5*ON_PI;
          }
          else {
            // search quadrant IV
            t0 = 1.5*ON_PI;
            t1 = 2.0*ON_PI;
          }
        }
        else {
          if ( uv.y >= 0.0 ) {
            // search quadrant II
            t0 = 0.5*ON_PI;
            t1 = ON_PI;
          }
          else {
            // search quadrant III
            t0 = ON_PI;
            t1 = 1.5*ON_PI;
          }
        }

        // solve for closest point using Brent's algorithm
        {
          // 6 October 2003 Dale Lear:
          //    Fixed several serious bugs here.
          // get seed value appropriate for Brent
          double p[4], et, d0, d1, dt;
          int i;
          p[0] = radius[0];
          p[1] = radius[1];
          p[2] = uv.x;
          p[3] = uv.y;
          et = *t;
          if ( et <= t0 )
            et = 0.9*t0 + 0.1*t1;
          else if ( et >= t1 )
            et = 0.9*t1 + 0.1*t0;
          distSqToEllipse( p, t0, &d0, NULL );
          distSqToEllipse( p, t1, &d1, NULL );
          if ( d0 == 0.0 ) {
            *t = (t0 == 2.0*ON_PI) ? 0.0 : t0;
            return true;
          }
          if ( d1 == 0.0 ) {
            *t = (t1 == 2.0*ON_PI) ? 0.0 : t1;
            return true;
          }
          if ( d0 > d1 ) {
            dt = t0; t0 = t1; t1 = dt;
            dt = d0; d0 = d1; d1 = dt;
          }
          *t = (t0 == 2.0*ON_PI) ? 0.0 : t0;
          for ( i = 0; true; i++ ) {
            distSqToEllipse( p, et, &dt, NULL );
            if ( dt < d0 )
            {
              *t = (et >= 2.0*ON_PI) ? 0.0 : et;
              break;
            }
            if ( i >= 100 ) 
            {
              ON_3dPoint E0 = PointAt(t0);
              if (    sqrt(d0) <= ON_ZERO_TOLERANCE 
                   || sqrt(d0) <= ON_SQRT_EPSILON*E0.DistanceTo(Center()) 
                   )
              {
                // Could not find a seed value for dbrent, 
                // but t0 is pretty close.
                return true;
              }
              ON_3dVector T = TangentAt(t0);
              ON_3dVector V = E0 - point;
              if ( V.Unitize() )
              {
                // Could not find a seed value for dbrent, 
                // but V and T are orthoganal, so t0 is 
                // pretty close.
                if ( fabs(V*T) <= 0.087155742747658173558064270837474 )
                  return true;
              }
              return false; // can't get valid seed - bail out
            }
            et = (i) ? (0.5*(t0+et)) : 0.5*(t0+t1);
            if ( et == t0 )
            {
              return true;
            }
          }

          rc = ON_FindLocalMinimum( distSqToEllipse, p,
                            t0, et, t1, 
                            ON_EPSILON,  ON_SQRT_EPSILON, 100,
                            &et );
          rc = (rc > 0) ? true : false;
          if ( rc )
            *t = (et >= 2.0*ON_PI) ? 0.0 : et;
        }
      }
    }
  }
  return rc;
}

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif

ON_3dPoint ON_Ellipse::ClosestPointTo( const ON_3dPoint& point ) const
{
  double t;
  ClosestPointTo( point, &t );
  return PointAt( t );
}

double ON_Ellipse::EquationAt( 
                 const ON_2dPoint& p // coordinates in plane
                 ) const
{
  double e, x, y;
  if ( radius[0] != 0.0 && radius[1] != 0.0 ) {
    x = p.x/radius[0];
    y = p.y/radius[1];
    e = x*x + y*y - 1.0;
  }
  else {
    e = 0.0;
  }
  return e;
}

ON_2dVector ON_Ellipse::GradientAt( 
                 const ON_2dPoint& p // coordinates in plane
                 ) const
{
  ON_2dVector g;
  if ( radius[0] != 0.0 && radius[1] != 0.0 ) {
    g.x = 2.0*p.x/(radius[0]*radius[0]);
    g.y = 2.0*p.y/(radius[1]*radius[1]);
  }
  else {
    g.Zero();
  }
  return g;
}

ON_BOOL32 ON_Ellipse::Rotate( 
                          double sin_angle, double cos_angle, 
                          const ON_3dVector& axis
                          )
{
  return plane.Rotate( sin_angle, cos_angle, axis );
}

ON_BOOL32 ON_Ellipse::Rotate( 
                          double angle, 
                          const ON_3dVector& axis
                          )
{
  return plane.Rotate( angle, axis );
}

ON_BOOL32 ON_Ellipse::Rotate( 
                          double sin_angle, double cos_angle, 
                          const ON_3dVector& axis,
                          const ON_3dPoint& point
                          )
{
  return plane.Rotate( sin_angle, cos_angle, axis, point );
}

ON_BOOL32 ON_Ellipse::Rotate( 
                          double angle, 
                          const ON_3dVector& axis,
                          const ON_3dPoint& point
                          )
{
  return plane.Rotate( angle, axis, point );
}


ON_BOOL32 ON_Ellipse::Translate(
                          const ON_3dVector& delta
                            )
{
  return plane.Translate( delta );
}

ON_BOOL32 ON_Ellipse::GetNurbForm( ON_NurbsCurve& nurbscurve ) const
{
  int rc = 0;
  if ( IsValid() ) {
    nurbscurve.Create( 3, true, 3, 9 );
    nurbscurve.m_knot[0] = nurbscurve.m_knot[1] = 0.0;
    nurbscurve.m_knot[2] = nurbscurve.m_knot[3] = 0.5*ON_PI;
    nurbscurve.m_knot[4] = nurbscurve.m_knot[5] = ON_PI;
    nurbscurve.m_knot[6] = nurbscurve.m_knot[7] = 1.5*ON_PI;
    nurbscurve.m_knot[8] = nurbscurve.m_knot[9] = 2.0*ON_PI;
    ON_4dPoint* CV = (ON_4dPoint*)nurbscurve.m_cv;

    CV[0] = plane.PointAt( radius[0],        0.0);
    CV[1] = plane.PointAt( radius[0],  radius[1]);
    CV[2] = plane.PointAt(       0.0,  radius[1]);
    CV[3] = plane.PointAt(-radius[0],  radius[1]);
    CV[4] = plane.PointAt(-radius[0],        0.0);
    CV[5] = plane.PointAt(-radius[0], -radius[1]);
    CV[6] = plane.PointAt(       0.0, -radius[1]);
    CV[7] = plane.PointAt( radius[0], -radius[1]);
    CV[8] = CV[0];
    
    const double w = 1.0/sqrt(2.0);
    int i;
    for ( i = 1; i < 8; i += 2 ) {
      CV[i].x *= w;
      CV[i].y *= w;
      CV[i].z *= w;
      CV[i].w = w;
    }
    rc = 2;
  }
  return rc;
}

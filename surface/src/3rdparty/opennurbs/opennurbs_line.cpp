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

ON_Line::ON_Line()
{
  from.Zero();
  to.Zero();
}

ON_Line::ON_Line( const ON_3dPoint& from_pt, const ON_3dPoint& to_pt )
: from(from_pt), to(to_pt)
{}

ON_Line::~ON_Line()
{
}

ON_3dPoint& ON_Line::operator[](int i)
{
  return (i<=0) ? from : to;
}

const ON_3dPoint& ON_Line::operator[](int i) const
{
  return (i<=0) ? from : to;
}

bool ON_Line::Create( const ON_3dPoint& from_pt, const ON_3dPoint& to_pt )
{
  from = from_pt;
  to = to_pt;
  return IsValid();
}


bool ON_Line::IsValid() const
{
  return (from != to && from.IsValid() && to.IsValid());
}

double ON_Line::Length() const
{
  return from.DistanceTo(to);
}

ON_3dVector ON_Line::Direction() const
{
  return (to-from);
}

ON_3dVector ON_Line::Tangent() const
{
  ON_3dVector V = Direction();
  V.Unitize();
  return V;
}

ON_3dPoint ON_Line::PointAt( double t ) const
{
  // 26 Feb 2003 Dale Lear
  //     Changed 
  //          return (1-t)*from + t*to;
  //     to the following so that axis aligned lines will
  //     return exact answers for large values of t.  
  //     See RR 9683.
  const double s = 1.0-t;
  return  ON_3dPoint( (from.x == to.x) ? from.x : s*from.x + t*to.x,
                      (from.y == to.y) ? from.y : s*from.y + t*to.y,
                      (from.z == to.z) ? from.z : s*from.z + t*to.z );
}

void ON_Line::Reverse()
{
  ON_3dPoint tmp = from;
  from = to;
  to = tmp;
}

bool ON_Line::ClosestPointTo( const ON_3dPoint& point, double *t ) const
{
  bool rc = false;
  if ( t ) {
    const ON_3dVector D = Direction();
    const double DoD = D.LengthSquared();
    if ( DoD > 0.0 ) {
      if ( point.DistanceTo(from) <= point.DistanceTo(to) ) {
        *t = ((point - from)*D)/DoD;
      }
      else {
        *t = 1.0 + ((point - to)*D)/DoD;
      }
      rc = true;
    }
    else {
      *t = 0.0;
    }
  }
  return rc;
}

ON_3dPoint ON_Line::ClosestPointTo( const ON_3dPoint& point ) const
{
  double t;
  ClosestPointTo( point, &t );
  return PointAt(t);
}

double ON_Line::DistanceTo( ON_3dPoint test_point ) const
{
  return test_point.DistanceTo(ClosestPointTo(test_point));
}


bool ON_Line::Transform( const ON_Xform& tr )
{
  from = tr*from;
  to = tr*to;
  // 5 June 2003 Dale Lear RR 10493
  //    Always return true.
  //return (from != to) ? true : false;
  return true;
}

// rotate line about a point and axis
bool ON_Line::Rotate(
      double sin_angle,                  // sin(angle)
      double cos_angle,                  // cos(angle)
      const ON_3dVector& axis,  // axis of rotation
      const ON_3dPoint& center  // center of rotation
      )
{
  ON_Xform rot;
  rot.Rotation( sin_angle, cos_angle, axis, center );
  const bool bFixP0 = (from==center);
  const bool bFixP1 = (to==center);
  const bool rc = Transform( rot );
  if ( bFixP0 )
    from = center;
  if ( bFixP1 )
    to = center;
  return rc;
}

bool ON_Line::Rotate(
      double a,                 // angle in radians
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint& center // center of rotation
      )
{
  return Rotate( sin(a), cos(a), axis, center );
}

bool ON_Line::Translate(
      const ON_3dVector& delta
      )
{
  ON_Xform tr;
  tr.Translation( delta );
  return Transform( tr );
}

int ON_ArePointsOnLine( // returns 0=no, 1 = yes, 2 = pointset is (to tolerance) a single point on the line
        int dim,     // 2 or 3
        int is_rat,
        int count, 
        int stride, const double* point,
        const ON_BoundingBox& bbox, // if needed, use ON_GetBoundingBox(dim,is_rat,count,stride,point)
        const ON_Line& line,  // line to test
        double tolerance
        )
{
  double w;
  int i, j, k;

  if ( count < 1 )
    return 0;

  if ( !line.IsValid() )
  {
    ON_ERROR("line parameter not valid");
    return 0;
  }
  if ( !bbox.IsValid() )
  {
    ON_ERROR("bbox parameter not valid");
    return 0;
  }
  if ( !ON_IsValid(tolerance) || tolerance < 0.0 )
  {
    ON_ERROR("tolerance parameter not valid");
    return 0;
  }
  if ( dim < 2 || dim > 3 )
  {
    ON_ERROR("dim parameter not valid");
    return 0;
  }
  if ( 0 == point )
  {
    ON_ERROR("point parameter not valid");
    return 0;
  }
  if ( stride < (is_rat?(dim+1):dim) )
  {
    ON_ERROR("stride parameter not valid");
    return 0;
  }

  int rc = 0;

  if ( tolerance == 0.0 ) {
    tolerance = bbox.Tolerance();
  }

  ON_3dPoint Q;

  // test bounding box to quickly detect the common coordinate axis cases
  rc = (count == 1 || bbox.Diagonal().Length() <= tolerance) ? 2 : 1;
  for ( i = 0; rc && i < 2; i++ ) {
    Q.x = bbox[i].x;
    for ( j = 0; rc && j < 2; j++) {
      Q.y = bbox[j].y;
      for ( k = 0; rc && k < 2; k++) {
        Q.z = bbox[k].z;
        if ( Q.DistanceTo( line.ClosestPointTo( Q ) ) > tolerance )
          rc = 0;
      }
    }
  }

  if ( !rc ) {
    // test points one by one
    Q.Zero();
    rc = (count == 1 || bbox.Diagonal().Length() <= tolerance) ? 2 : 1;
    if ( is_rat ) {
      for ( i = 0; i < count; i++ ) {
        w = point[dim];
        if ( w == 0.0 ) {
          ON_ERROR("rational point has zero weight");
          return 0;
        }
        ON_ArrayScale( dim, 1.0/w, point, &Q.x );
        if ( Q.DistanceTo( line.ClosestPointTo( Q ) ) > tolerance ) {
          rc = 0;
          break;
        }
        point += stride;
      }
    }
    else {
      for ( i = 0; i < count; i++ ) {
        memcpy( &Q.x, point, dim*sizeof(Q.x) );
        if ( Q.DistanceTo( line.ClosestPointTo( Q ) ) > tolerance ) {
          rc = 0;
          break;
        }
        point += stride;
      }
    }
  }

  return rc;
}


ON_BoundingBox ON_Line::BoundingBox() const
{
  ON_BoundingBox bbox;
  bbox.Set( 3, false, 2, 3, &from.x, false );
  return bbox;
}

bool ON_Line::GetBoundingBox(
       ON_BoundingBox& bbox,
       int bGrowBox
       ) const
{
  bbox.Set( 3, false, 2, 3, &from.x, bGrowBox );
  return true;
}

bool ON_Line::InPlane( ON_Plane& plane, double tolerance ) const
{
  const ON_3dVector v = to-from;
  const bool bTinyX = fabs(v.x) <= tolerance;
  const bool bTinyY = fabs(v.y) <= tolerance;
  const bool bTinyZ = fabs(v.z) <= tolerance;
  bool rc = true;
  ON_3dVector X;
  ON_3dVector Y;
  if ( bTinyZ && ( !bTinyX || !bTinyY ) )
  {
    X = ON_xaxis;
    Y = ON_yaxis;
  }
  else if ( bTinyX && ( !bTinyY || !bTinyZ ) )
  {
    X = ON_yaxis;
    Y = ON_zaxis;
  }
  else if ( bTinyY && ( !bTinyZ || !bTinyX ) )
  {
    X = ON_zaxis;
    Y = ON_xaxis;
  }
  else
  {
    X = v;
    X.Unitize();
    Y.PerpendicularTo(X);
    if ( bTinyX && bTinyY && bTinyZ )
    {
      rc = false;
      if ( X.IsZero() )
      {
        X = ON_xaxis;
        Y = ON_yaxis;
      }
    }
  }
  plane.CreateFromFrame( from, X, Y );
  return rc;
}

double ON_Line::MinimumDistanceTo( const ON_3dPoint& P ) const
{
  double d, t;
  if (ClosestPointTo(P,&t))
  {
    if ( t < 0.0 ) t = 0.0; else if (t > 1.0) t = 1.0;
    d = PointAt(t).DistanceTo(P);
  }
  else
  {
    // degenerate line
    d = from.DistanceTo(P);
    t = to.DistanceTo(P);
    if ( t < d )
      d = t;
  }
  return d;
}

double ON_Line::MinimumDistanceTo( const ON_Line& L ) const
{
  ON_3dPoint A, B;
  double a, b, t, x, d;
  bool bCheckA, bCheckB;

  bool bGoodX = ON_Intersect(*this,L,&a,&b);

  bCheckA = true;
  if ( a < 0.0) a = 0.0; else if (a > 1.0) a = 1.0; else bCheckA=!bGoodX;
  bCheckB = true;
  if ( b < 0.0) b = 0.0; else if (b > 1.0) b = 1.0; else bCheckB=!bGoodX;

  A = PointAt(a);
  B = L.PointAt(b);
  d = A.DistanceTo(B);

  if ( bCheckA )
  {
    L.ClosestPointTo(A,&t);
    if (t<0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    x = L.PointAt(t).DistanceTo(A);
    if ( x < d )
      d = x;
  }

  if ( bCheckB )
  {
    ClosestPointTo(B,&t);
    if (t<0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    x = PointAt(t).DistanceTo(B);
    if (x < d )
      d = x;
  }
 
  return d;
}

double ON_Line::MaximumDistanceTo( const ON_3dPoint& P ) const
{
  double a, b;
  a = from.DistanceTo(P);
  b = to.DistanceTo(P);
  return ((a<b)?b:a);
}

double ON_Line::MaximumDistanceTo( const ON_Line& L ) const
{
  double a, b;
  a = MaximumDistanceTo(L.from);
  b = MaximumDistanceTo(L.to);
  return ((a<b)?b:a);
}

bool ON_Line::IsFartherThan( double d, const ON_3dPoint& P ) const
{
  if ( P.x > to.x+d && P.x > from.x+d )
  {
    return true;
  }
  if ( P.x < to.x-d && P.x < from.x-d )
  {
    return true;
  }
  if ( P.y > to.y+d && P.y > from.y+d )
  {
    return true;
  }
  if ( P.y < to.y-d && P.y < from.y-d )
  {
    return true;
  }
  if ( P.z > to.z+d && P.z > from.z+d )
  {
    return true;
  }
  if ( P.z < to.z-d && P.z < from.z-d )
  {
    return true;
  }
  return (MinimumDistanceTo(P) > d);
}


bool ON_Line::IsFartherThan( double d, const ON_Line& L ) const
{
  ON_3dPoint A, B;
  double a, b, t, x;
  bool bCheckA, bCheckB;

  a = from.x; if (to.x < a) {b=a; a = to.x;} else b = to.x;
  if ( b+d < L.from.x && b+d < L.to.x )
    return true;
  if ( a-d > L.from.x && a-d > L.to.x )
    return true;

  a = from.y; if (to.y < a) {b=a; a = to.y;} else b = to.y;
  if ( b+d < L.from.y && b+d < L.to.y )
    return true;
  if ( a-d > L.from.y && a-d > L.to.y )
    return true;

  a = from.z; if (to.z < a) {b=a; a = to.z;} else b = to.z;
  if ( b+d < L.from.z && b+d < L.to.z )
    return true;
  if ( a-d > L.from.z && a-d > L.to.z )
    return true;

  if ( !ON_Intersect(*this,L,&a,&b) )
  {
    // lines are parallel or anti parallel
    if ( Direction()*L.Direction() >= 0.0 )
    {
      // lines are parallel
      a = 0.0;
      L.ClosestPointTo(from,&b);
      // If ( b >= 0.0), then this->from and L(b) are a pair of closest points.
      if ( b < 0.0 )
      {
        // Othersise L.from and this(a) are a pair of closest points.
        b = 0.0;
        ClosestPointTo(L.from,&a);
      }
    }
    else
    {
      // lines are anti parallel
      a = 1.0;
      L.ClosestPointTo(to,&b);
      // If ( b >= 0.0), then this->to and L(b) are a pair of closest points.
      if ( b < 0.0 )
      {
        // Othersise L.to and this(a) are a pair of closest points.
        b = 0.0;
        ClosestPointTo(L.from,&a);
      }
    }
  }

  A = PointAt(a);
  B = L.PointAt(b);
  x = A.DistanceTo(B);
  if (x > d)
    return true;

  bCheckA = true;
  if ( a < 0.0) a = 0.0; else if (a > 1.0) a = 1.0; else bCheckA=false;
  if (bCheckA )
  {
    A = PointAt(a);
    L.ClosestPointTo(A,&t);
    if (t<0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    x = L.PointAt(t).DistanceTo(A);
  }

  bCheckB = true;
  if ( b < 0.0) b = 0.0; else if (b > 1.0) b = 1.0; else bCheckB=false;
  if ( bCheckB )
  {
    B = L.PointAt(b);
    ClosestPointTo(B,&t);
    if (t<0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    t = PointAt(t).DistanceTo(B);
    if ( bCheckA )
    {
      if ( t<x ) x = t;
    }
    else
    {
      x = t;
    }
  }
 
  return (x > d);
}


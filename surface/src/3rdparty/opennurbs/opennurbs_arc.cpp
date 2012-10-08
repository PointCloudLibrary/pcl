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

ON_Arc::ON_Arc() : m_angle(0.0,2.0*ON_PI)
{
  radius=1.0;
}

ON_Arc::ON_Arc( const ON_Circle& c, double angle_in_radians ) 
       : m_angle(0.0,2.0*ON_PI)
{
  Create( c, angle_in_radians );
}

ON_Arc::ON_Arc( const ON_Circle& c, ON_Interval angle_interval_in_radians ) 
       : m_angle(0.0,2.0*ON_PI)
{
  Create( c, angle_interval_in_radians );
}

ON_Arc::ON_Arc( const ON_Plane& p, double r, double angle_in_radians )
       : m_angle(0.0,2.0*ON_PI)
{
  Create( p, r, angle_in_radians );
}

ON_Arc::ON_Arc( const ON_3dPoint& C, double r, double angle_in_radians )
       : m_angle(0.0,2.0*ON_PI)
{
  Create( C, r, angle_in_radians );
}

ON_Arc::ON_Arc( const ON_Plane& pln, const ON_3dPoint& C, double r, double angle_in_radians )
       : m_angle(0.0,2.0*ON_PI)
{
  Create( pln, C, r, angle_in_radians );
}

ON_Arc::ON_Arc( const ON_2dPoint& P, const ON_2dPoint& Q, const ON_2dPoint& R ) 
       : m_angle(0.0,2.0*ON_PI)
{
  Create( P, Q, R );
}

ON_Arc::ON_Arc( const ON_3dPoint& P, const ON_3dPoint& Q, const ON_3dPoint& R )
       : m_angle(0.0,2.0*ON_PI)
{
  Create( P, Q, R );
}

ON_Arc& ON_Arc::operator=( const ON_Circle& c )
{

#if defined(ON_COMPILER_IRIX)
  plane = c.plane;
  radius = c.radius;
#else
  ON_Circle::operator=(c);
#endif

  m_angle.Set(0.0,2.0*ON_PI);
  return *this;
}


bool ON_Arc::Create(
  const ON_Circle& circle,
  double angle_radians          // angle in radians
  )
{
  return Create( circle, ON_Interval( 0.0, angle_radians ) );
}

bool ON_Arc::Create(
  const ON_Circle& circle,
  ON_Interval angle_interval_in_radians
  )
{
  bool rc = true;
  plane = circle.plane;
  plane.UpdateEquation();
  radius = circle.radius;
  m_angle = angle_interval_in_radians;
  if ( m_angle.IsDecreasing() )
  {
    rc = false; // bogus input
    // m_angle must never be decreasing
    m_angle.Swap();
    Reverse();
  }
  if ( m_angle.Length() > 2.0*ON_PI )
  {
    rc = false; // bogus input
    m_angle.m_t[1] = m_angle.m_t[0] + 2.0*ON_PI;
  }
  if ( rc )
    rc = IsValid();
  return rc;
}


bool ON_Arc::Create(
  const ON_Plane& pl, // circle is in this plane with center at m_origin
  double r,         // radius
  double angle_radians   // angle in radians
  )
{
  return Create( ON_Circle(pl,r), ON_Interval( 0.0, angle_radians ) );
}

bool ON_Arc::Create( // arc is parallel to XY plane
  const ON_3dPoint& center, // center
  double r,            // radius
  double angle_radians // angle in radians
  )
{
  ON_Plane p;
  p.CreateFromNormal( center, ON_zaxis );
  return Create( ON_Circle(p,r), ON_Interval( 0.0, angle_radians ) );
}

bool ON_Arc::Create(        // arc parallel to a plane
  const ON_Plane& pl,       // circle will be parallel to this plane
  const ON_3dPoint& center, // center
  double r,                 // radius
  double angle_radians      // angle in radians
  )
{
  ON_Plane p = pl;
  p.origin = center;
  p.UpdateEquation();
  return Create( ON_Circle( p, r), ON_Interval( 0.0, angle_radians ) );
}

bool ON_Arc::Create( // arc through 3 2d points
  const ON_2dPoint& P, // point P
  const ON_2dPoint& Q, // point Q
  const ON_2dPoint& R // point R
  )
{
  ON_Circle c(P,Q,R);
  double a = 0.0;
  c.ClosestPointTo( R, &a );
  return Create( c, ON_Interval(0.0,a) );
}

bool ON_Arc::Create( // arc through 3 3d points
  const ON_3dPoint& P, // point P
  const ON_3dPoint& Q, // point Q
  const ON_3dPoint& R  // point R
  )
{
  ON_Circle c;
  double a = 0.0;

  for (;;)
  {

    if ( !c.Create(P,Q,R) )
      break;

    if ( !c.ClosestPointTo( R, &a ) )
      break;

    if ( !(a > 0.0) )
      break;
    
    if ( !Create( c, ON_Interval(0.0,a) ) )
      break;

    return true;
  }

  plane = ON_Plane::World_xy;
  radius = 0.0;
  m_angle.Set(0.0,0.0);

  return false;
}

//////////
// Create an arc from a 2d start point, 2d start direction, and 2d end point.
bool ON_Arc::Create(
  const ON_2dPoint& P, // [IN] start point
  const ON_2dVector& Pdir,  // [IN] arc direction at start
  const ON_2dPoint&  Q   // [IN] end point
  )
{
  return Create( ON_3dPoint(P), ON_3dVector(Pdir), ON_3dPoint(Q) );
}

//////////
// Create an arc from a 3d start point, 3d start direction, and 3d end point.
bool ON_Arc::Create(
  const ON_3dPoint& P, // [IN] start point
  const ON_3dVector& Pdir,  // [IN] arc direction at start
  const ON_3dPoint&  Q   // [IN] end point
  )
{
  double a=0.0;
  bool rc = ON_Circle::Create(P,Pdir,Q);
  if ( rc ) {
    m_angle.m_t[0] = 0.0;
    rc = ON_Circle::ClosestPointTo(Q,&a);
    m_angle.m_t[1] = a;
    if (a <= ON_ZERO_TOLERANCE || a >= 2.0*ON_PI-ON_ZERO_TOLERANCE )
      rc = false;
  }
  return rc;
}


ON_Arc::~ON_Arc()
{}

void ON_Arc::Dump( ON_TextLog& dump ) const
{
  dump.Print("Arc: normal = ");
  dump.Print(plane.zaxis);
  dump.Print(" center = ");
  dump.Print(plane.origin);
  dump.Print(" start = ");
  dump.Print( StartPoint() );
  dump.Print(" end = ");
  dump.Print( EndPoint() );
  dump.Print(" radius = ");
  dump.Print(Radius());
  dump.Print(" angle = [");
  dump.Print(m_angle[0]);
  dump.Print(",");
  dump.Print(m_angle[1]);
  dump.Print("]\n");
}

ON_3dPoint ON_Arc::StartPoint() const
{
  return PointAt(m_angle[0]);
}

ON_3dPoint ON_Arc::MidPoint() const
{
  return PointAt(m_angle.Mid());
}

ON_3dPoint ON_Arc::EndPoint() const
{
  return PointAt(m_angle[1]);
}

bool ON_Arc::IsValid() const
{
  return (    ON_Circle::IsValid() 
           && m_angle.IsValid()
           && AngleRadians() > ON_ZERO_TOLERANCE 
           && AngleRadians() <= 2.0*ON_PI+ON_ZERO_TOLERANCE) 
         ? true : false;
}

ON_BoundingBox ON_Arc::BoundingBox() const
{
  // TODO - compute tight arc bounding box

  // Using these knot[] and cv[] arrays makes this function
  // not use any heap memory.
  double knot[10];
  ON_4dPoint cv[9];
  ON_NurbsCurve c;
  c.m_knot = knot;
  c.m_cv = &cv[0].x;
  if ( GetNurbForm(c) )
    return c.BoundingBox();
  return ON_Circle::BoundingBox();
}

bool ON_Arc::GetBoundingBox(
       ON_BoundingBox& bbox,
       int bGrowBox
       ) const
{
  if (bGrowBox)
  {
    ON_BoundingBox arc_bbox = BoundingBox();
    bbox.Union(arc_bbox);
  }
  else
    bbox = BoundingBox();
  return bbox.IsValid();
}

bool ON_Arc::IsCircle() const
{
  return (fabs(fabs(AngleRadians()) - 2.0*ON_PI) <= ON_ZERO_TOLERANCE) 
         ? true : false;
}

double ON_Arc::AngleRadians() const
{
  return m_angle[1]-m_angle[0];
}

double ON_Arc::AngleDegrees() const
{
  return (AngleRadians()/ON_PI)*180.0;
}

ON_Interval ON_Arc::Domain() const
{
  return m_angle;
}

ON_Interval ON_Arc::DomainRadians() const
{
  return m_angle;
}

ON_Interval ON_Arc::DomainDegrees() const
{
  const double rtd = 180.0/ON_PI;
  ON_Interval ad = m_angle;
  ad.m_t[0] *= rtd;
  ad.m_t[1] *= rtd;
  return ad;
}

bool ON_Arc::SetAngleRadians( double a )
{
  if ( a < 0.0 ) 
  {
    double a0 = m_angle.m_t[0];
    m_angle.Set(a0+a,a0);
    Reverse();
  }
  else
  {
    m_angle.m_t[1] = m_angle.m_t[0] + a;
  }
  return ( fabs(m_angle.Length()) <= 2.0*ON_PI ) ? true : false;
}

bool ON_Arc::SetAngleIntervalRadians( ON_Interval angle_in_radians )
{
  bool rc = angle_in_radians.IsIncreasing() 
            && angle_in_radians.Length() < (1.0+ON_SQRT_EPSILON)*2.0*ON_PI;
  if (rc)
  {
    m_angle = angle_in_radians;
  }
  return rc;
}

bool ON_Arc::SetAngleDegrees( double a )
{
  return SetAngleRadians( (a/180.0)*ON_PI );
}

bool ON_Arc::Trim( ON_Interval domain)
{
  bool ok = false;

  if(domain[0]<domain[1] && domain[1]-domain[0]<=2.0 * ON_PI+ON_ZERO_TOLERANCE){
		m_angle = domain;
    if (m_angle.Length() > 2.0*ON_PI) m_angle[1] = m_angle[0] + 2.0*ON_PI;
    ok = true;
  }
	return ok;
}

bool ON_ArcCurve::IsContinuous(
    ON::continuity c,
    double t, 
    int*,   // hint                - formal parameter intentionally ignored in this virtual function
    double, // point_tolerance     - formal parameter intentionally ignored in this virtual function
    double, // d1_tolerance        - formal parameter intentionally ignored in this virtual function
    double, // d2_tolerance        - formal parameter intentionally ignored in this virtual function
    double, // cos_angle_tolerance - formal parameter intentionally ignored in this virtual function
    double  // curvature_tolerance - formal parameter intentionally ignored in this virtual function
    ) const
{
  // 20 March 2003 Dale Lear
  //      Added this override of IsContinuous() to
  //      speed queries and support the
  //      locus favors of ON::continuity.

  bool rc = true;

  if ( !IsClosed() )
  {
    switch(c)
    {
    case ON::unknown_continuity:
    case ON::C0_continuous:
    case ON::C1_continuous:
    case ON::C2_continuous:
    case ON::G1_continuous:
    case ON::G2_continuous:
    case ON::Cinfinity_continuous:
    case ON::Gsmooth_continuous:
      // rc = true;
      break;

    case ON::C0_locus_continuous:
    case ON::C1_locus_continuous:
    case ON::C2_locus_continuous:
    case ON::G1_locus_continuous:
    case ON::G2_locus_continuous:
      // open arc is locus discontinuous at end parameter.
      // By convention (see ON::continuity comments) it
      // is locus continuous at start parameter.
      if ( t >= Domain()[1] )
        rc = false;
      break;
    }
  }

  return rc;
}



bool ON_Arc::Reverse()
{
  m_angle.Reverse();
  plane.yaxis = -plane.yaxis;
  plane.zaxis = -plane.zaxis;
  plane.UpdateEquation();
  return true;
}

double ON_Arc::Length() const
{
  return fabs(AngleRadians()*radius);
}

double ON_Arc::SectorArea() const
{
  return fabs(0.5*AngleRadians()*radius*radius);
}

ON_3dPoint ON_Arc::SectorAreaCentroid() const
{
  double a = 0.5*fabs(AngleRadians());
  double d = (a > 0.0) ? sin(a)/a : 0.0;
  d *= 2.0*radius/3.0;
  a = 0.5*(m_angle[1]+m_angle[0]);
  return plane.PointAt(d*cos(a),d*sin(a));
}

double ON_Arc::SegmentArea() const
{
  double a = fabs(AngleRadians());
  return (0.5*(a - sin(a))*radius*radius);
}

ON_3dPoint ON_Arc::SegmentAreaCentroid() const
{
  double a = fabs(AngleRadians());
  double sin_halfa = sin(0.5*a);
  double d = 3.0*(a - sin(a));
  if ( d > 0.0 )
    d = (sin_halfa*sin_halfa*sin_halfa)/d;
  d *= 4.0*radius;
  a = 0.5*(m_angle[1]+m_angle[0]);
  return plane.PointAt(d*cos(a),d*sin(a));
}


/* moved to opennurbs_arccurve.cpp


int ON_Arc::GetNurbForm( ON_NurbsCurve& nurbscurve ) const
{
  int rc = 0;
  if ( IsValid() ) {
    if ( IsCircle() )
      rc = ON_Circle::GetNurbForm( nurbscurve );
    else {
      double a, b, c, t, dt, angle;
      int span_count, i;
      angle = m_angle.Length();
      if (angle <= 0.5*ON_PI + ON_ZERO_TOLERANCE) {
		    span_count = 1;
        dt = 0.5;
      }
      else if (angle <= ON_PI + ON_ZERO_TOLERANCE) {
		    span_count = 2;
  		  angle *= 0.5;
        dt = 0.25;
      }
      else if (angle <= 1.5*ON_PI + ON_ZERO_TOLERANCE) {
		    span_count = 3;
  		  angle /= 3.0;
        dt = 1.0/6.0;
      }
      else {
		    span_count = 4;
  		  angle *= 0.25;
        dt = 0.125;
      }
      nurbscurve.Create( 3, true, 3, 2*span_count+1 );
      ON_4dPoint* CV = (ON_4dPoint*)nurbscurve.m_cv;
      t = m_angle[0];
      for ( i = 0; i < span_count; i++ ) {
        nurbscurve.m_knot[2*i] = t;
        nurbscurve.m_knot[2*i+1] = t;
        CV[2*i] = PointAt(m_angle.ParameterAt(t));
        t += dt;
        CV[2*i+1] = PointAt(m_angle.ParameterAt(t));
        t += dt;
      }

      span_count *= 2;
      t = m_angle[1];
      CV[span_count] = PointAt(t);
      nurbscurve.m_knot[span_count] = t;
      nurbscurve.m_knot[span_count+1] = t;

	    a = cos(0.5*angle);
	    b = a - 1.0;
	    c = radius*angle;

	    for (i = 1; i < span_count; i += 2) {
		    CV[i].x +=  b * plane.origin.x;
		    CV[i].y +=  b * plane.origin.y;
		    CV[i].z +=  b * plane.origin.z;
		    CV[i].w = a;
	    }
      
      //for ( i = 1; i < span_count; i += 2 ) {
      //  t = CV[i].w;
      //  c = 1.0/t;
      //  a = CV[i].x*c; b = ArcDeFuzz(a); if ( a != b ) CV[i].x = b*t;
      //  a = CV[i].y*c; b = ArcDeFuzz(a); if ( a != b ) CV[i].y = b*t;
      //  a = CV[i].z*c; b = ArcDeFuzz(a); if ( a != b ) CV[i].z = b*t;
      //}
    }
    rc = 2;
  }
	return rc;
}
*/

// returns parameters of point on arc that is closest to given point
bool ON_Arc::ClosestPointTo( 
       const ON_3dPoint& pt, 
       double* t
       ) const
{
  /*
  double tt, a;
  if ( !t )
    t =&tt;
  ON_BOOL32 rc = ON_Circle::ClosestPointTo(pt,t);
  if (rc) {
    if ( *t < m_angle[0] ) {
      a = 0.5*(m_angle[0] + m_angle[1] - 2.0*ON_PI);
      if ( *t < a )
        *t = m_angle[1];
      else 
        *t = m_angle[0];
    }
    else if ( *t > m_angle[1] ) {
      a = 0.5*(m_angle[0] + m_angle[1] + 2.0*ON_PI);
      if ( *t > a )
        *t = m_angle[0];
      else 
        *t = m_angle[1];
    }
  }
  */
  double s;
  double twopi = 2.0*ON_PI;
  bool rc = ON_Circle::ClosestPointTo(pt,&s);
  if (rc){
    s -= m_angle[0];
    while (s < 0.0) s += twopi;

		// Greg Arden April 14 2003. Changed test from ">" to ">=" this ensures that
		// closest point to a circle at the seam will return the least parameter value.
    while (s >= twopi) s -= twopi;

    double s1 = m_angle.Length();

    if (s < 0.0) s = 0.0;//shouldn't happen
    if (s > s1){
      if (s > 0.5*s1 + ON_PI)
        s = 0.0;
      else
        s = s1;
    }

    if (t)
      *t = m_angle[0] + s;
  }

      
  return rc;
}

// returns point on circle that is arc to given point
ON_3dPoint ON_Arc::ClosestPointTo( 
       const ON_3dPoint& pt
       ) const
{
  double t = m_angle[0];
  ClosestPointTo( pt, &t );
  return PointAt(t);
}


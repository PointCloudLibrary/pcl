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

ON_Cone::ON_Cone() 
{
  height = 0.0;
}

ON_Cone::ON_Cone(
    const ON_Plane& p,
    double h,
    double r
    )
{
  Create(p,h,r);
}

ON_Cone::~ON_Cone()
{}


ON_BOOL32 ON_Cone::Create(
    const ON_Plane& p,
    double h,
    double r
    )
{
  plane = p;
  height = h;
  radius = r;
  return IsValid();
}

ON_BOOL32 ON_Cone::IsValid() const
{
  return (plane.IsValid() && height != 0.0 && radius != 0.0);
}


const ON_3dVector& ON_Cone::Axis() const
{
  return plane.zaxis;
}

const ON_3dPoint& ON_Cone::ApexPoint() const
{
  return plane.origin;
}

ON_3dPoint ON_Cone::BasePoint() const
{
  return plane.origin + height*plane.zaxis;
}

double ON_Cone::AngleInRadians() const
{
  return height == 0.0 ? (radius!=0.0?ON_PI:0.0) : atan(radius/height);
}

double ON_Cone::AngleInDegrees() const
{
  return 180.0*AngleInRadians()/ON_PI;
}

ON_Circle ON_Cone::CircleAt( 
      double height_parameter
      ) const
{
  ON_Circle c(plane,radius);
  c.Translate(height_parameter*plane.zaxis);
  if ( height != 0.0 )
    c.radius *= height_parameter/height;
  else if (height_parameter==0.0)
    c.radius = 0.0;
  return c;
}

ON_Line ON_Cone::LineAt( 
      double radial_parameter
      ) const
{
  return ON_Line(PointAt(radial_parameter,height),ApexPoint());
}


ON_3dPoint ON_Cone::PointAt( double radial_parameter, double height_parameter ) const
{
  double r;
  if ( height != 0.0 )
    r = (radius/height)*height_parameter;
  else
    r = (height_parameter == 0.0)?0.0:radius;
  return plane.PointAt(r*cos(radial_parameter),r*sin(radial_parameter)) + height_parameter*plane.zaxis;
}

ON_3dVector ON_Cone::NormalAt( double radial_parameter, double height_parameter ) const
{
  double s = sin(radial_parameter);
  double c = cos(radial_parameter);
  if ( radius<0.) {
    c = -c;
    s = -s;
  }
  ON_3dVector ds = c*plane.yaxis - s*plane.xaxis;
  ON_3dVector N = ON_CrossProduct( ((radius<0.0)?-ds:ds),
                                   plane.PointAt(radius*c,radius*s,height) - plane.origin
                                   );
  N.Unitize();
  return N;
}

ON_BOOL32 ON_Cone::Transform( const ON_Xform& xform )
{
  ON_Circle xc(plane,radius);
  ON_BOOL32 rc = xc.Transform(xform);
  if (rc)
  {
    ON_3dPoint xH = xform*(plane.origin + height*plane.zaxis);
    double xh = (xH-xc.plane.origin)*xc.plane.zaxis;
    plane = xc.plane;
    radius = xc.radius;
    height = xh;
  }
  return rc;
}

bool ON_Cone::ClosestPointTo( 
          ON_3dPoint point, 
          double* radial_parameter,
          double* height_parameter
       ) const
{
  // untested code

  bool rc = false;

  ON_3dVector v = (point-plane.origin);
  double x = v*plane.xaxis;
  double y = v*plane.yaxis;
  double z = v*plane.zaxis;

  if ( radial_parameter )
  {
    double a = ( 0.0 == y && 0.0 == x ) ? 0.0 : atan2(y,x);

    if (a > 2.0*ON_PI )
    {
      a -= 2.0*ON_PI;
    }
    
    if (a < 0.0 )
    {
      a += 2.0*ON_PI;
    }

    *radial_parameter = a;
  }

  if (height_parameter)
  {
    point.x -= plane.origin.x;
    point.y -= plane.origin.y;
    point.z -= plane.origin.z;
    v.x = x;
    v.y = y;
    v.z = 0.0;
    v.Unitize();
    v.x *= radius;
    v.y *= radius;
    ON_Line line(ON_origin, v.x*plane.xaxis + v.y*plane.yaxis + height*plane.zaxis );
    rc = line.ClosestPointTo(point,&z);
    if (rc)
    {
      *height_parameter = z*height;
    }
  }

  return rc;
}

// returns point on cylinder that is closest to given point
ON_3dPoint ON_Cone::ClosestPointTo( 
       ON_3dPoint point
       ) const
{
  // untested code

  ON_3dVector v = (point-plane.origin);
  double x = v*plane.xaxis;
  double y = v*plane.yaxis;
  //double z = v*plane.zaxis;

  point.x -= plane.origin.x;
  point.y -= plane.origin.y;
  point.z -= plane.origin.z;
  v.x = x;
  v.y = y;
  v.z = 0.0;
  v.Unitize();
  v.x *= radius;
  v.y *= radius;
  ON_Line line(ON_origin, v.x*plane.xaxis + v.y*plane.yaxis + height*plane.zaxis );
  return line.ClosestPointTo(point);
}

ON_BOOL32 ON_Cone::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis_of_rotation
      )
{
  return Rotate( sin_angle, cos_angle, axis_of_rotation, plane.origin );
}

ON_BOOL32 ON_Cone::Rotate(
      double angle,
      const ON_3dVector& axis_of_rotation
      )
{
  return Rotate( sin(angle), cos(angle), axis_of_rotation, plane.origin );
}

// rotate plane about a point and axis
ON_BOOL32 ON_Cone::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis_of_rotation,
      const ON_3dPoint& center_of_rotation
      )
{
  return plane.Rotate( sin_angle, cos_angle, axis_of_rotation, center_of_rotation );
}

ON_BOOL32 ON_Cone::Rotate(
      double angle,              // angle in radians
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint&  point  // center of rotation
      )
{
  return Rotate( sin(angle), cos(angle), axis, point );
}

ON_BOOL32 ON_Cone::Translate(
      const ON_3dVector& delta
      )
{
  return plane.Translate( delta );
}

int ON_Cone::GetNurbForm( ON_NurbsSurface& s ) const
{
  int rc = 0;
  if ( IsValid() ) {
    ON_Circle c = CircleAt(height);
    ON_NurbsCurve n;
    c.GetNurbForm(n);
    ON_3dPoint apex = ApexPoint();
    ON_4dPoint cv;
    int i, j0, j1;

    s.Create(3,true,3,2,9,2);
    for ( i = 0; i < 10; i++ )
      s.m_knot[0][i] = n.m_knot[i];

    if ( height >= 0.0 ) {
      s.m_knot[1][0] = 0.0;
      s.m_knot[1][1] = height; 
      j0 = 0;
      j1 = 1;
    }
    else {
      s.m_knot[1][0] = height;
      s.m_knot[1][1] = 0.0;
      j0 = 1;
      j1 = 0;
    }

    for ( i = 0; i < 9; i++ ) {
      cv = n.CV(i);
      s.SetCV(i, j1, ON::homogeneous_rational, &cv.x );
      cv.x = apex.x*cv.w;
      cv.y = apex.y*cv.w;
      cv.z = apex.z*cv.w;
      s.SetCV(i, j0, cv);
    }
    rc = 2;
  }
  return rc;
}

ON_RevSurface* ON_Cone::RevSurfaceForm( ON_RevSurface* srf ) const
{
  if ( srf )
    srf->Destroy();
  ON_RevSurface* pRevSurface = NULL;
  if ( IsValid() )
  {
    ON_Line line;
    ON_Interval line_domain;
    if ( height >= 0.0 )
      line_domain.Set(0.0,height);
    else
      line_domain.Set(height,0.0);
    line.from = PointAt(0.0,line_domain[0]);
    line.to = PointAt(0.0,line_domain[1]);
    ON_LineCurve* line_curve = new ON_LineCurve( line, line_domain[0], line_domain[1] );
    if ( srf )
      pRevSurface = srf;
    else
      pRevSurface = new ON_RevSurface();
    pRevSurface->m_angle.Set(0.0,2.0*ON_PI);
    pRevSurface->m_t = pRevSurface->m_angle;
    pRevSurface->m_curve = line_curve;
    pRevSurface->m_axis.from = plane.origin;
    pRevSurface->m_axis.to = plane.origin + plane.zaxis;
    pRevSurface->m_bTransposed = false;
    pRevSurface->m_bbox.m_min = plane.origin;
    pRevSurface->m_bbox.m_max = plane.origin;
    pRevSurface->m_bbox.Union(CircleAt(height).BoundingBox());
  }
  return pRevSurface;
}

/*
// obsolete use ON_BrepCone
ON_Brep* ON_Cone::BrepForm( ON_Brep* brep ) const
{
  ON_Brep* pBrep = 0;
  if ( brep )
    brep->Destroy();
  ON_RevSurface* pRevSurface = RevSurfaceForm();
  if ( pRevSurface )
  {
    if ( brep )
      pBrep = brep;
    else
      pBrep = new ON_Brep();
    if ( !pBrep->Create(pRevSurface) ) 
    {
      if ( !brep )
        delete pBrep;
      pBrep = 0;
      if ( !pRevSurface )
      {
        delete pRevSurface;
        pRevSurface = 0;
      }
    }
    else 
    {
      // add cap
      ON_Circle circle = CircleAt(height);
      ON_NurbsSurface* pCapSurface = ON_NurbsSurfaceQuadrilateral( 
        circle.plane.PointAt(-radius,-radius),
        circle.plane.PointAt(+radius,-radius),
        circle.plane.PointAt(+radius,+radius),
        circle.plane.PointAt(-radius,+radius)
        );
      pCapSurface->m_knot[0][0] = -fabs(radius);
      pCapSurface->m_knot[0][1] =  fabs(radius);
      pCapSurface->m_knot[1][0] = pCapSurface->m_knot[0][0];
      pCapSurface->m_knot[1][1] = pCapSurface->m_knot[0][1];
      circle.Create( ON_xy_plane, ON_origin, radius );
      ON_NurbsCurve* c2 = new ON_NurbsCurve();
      circle.GetNurbForm(*c2);
      c2->ChangeDimension(2);

      pBrep->m_S.Append(pCapSurface);
      pBrep->m_C2.Append(c2);
      ON_BrepFace& cap = pBrep->NewFace( pBrep->m_S.Count()-1 );
      ON_BrepLoop& loop = pBrep->NewLoop( ON_BrepLoop::outer, cap );
      ON_BrepEdge& edge = pBrep->m_E[1];
      ON_BrepTrim& trim = pBrep->NewTrim( edge, true, loop, pBrep->m_C2.Count()-1 );
      trim.m_tolerance[0] = 0.0;
      trim.m_tolerance[1] = 0.0;
      trim.m_pbox.m_min.x = -radius;
      trim.m_pbox.m_min.y = -radius;
      trim.m_pbox.m_min.z = 0.0;
      trim.m_pbox.m_max.x = radius;
      trim.m_pbox.m_max.y = radius;
      trim.m_pbox.m_max.z = 0.0;
      loop.m_pbox = trim.m_pbox;
      pBrep->SetTrimIsoFlags(trim);
      for ( int eti = 0; eti < edge.m_ti.Count(); eti++ )
        pBrep->m_T[ edge.m_ti[eti] ].m_type = ON_BrepTrim::mated;
      if ( !pBrep->IsValid() )
      {
        if (brep)
          brep->Destroy();
        else
          delete pBrep;
        pBrep = 0;
      }
    }
  }
  return pBrep;
}
*/

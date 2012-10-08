#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

ON_Box::ON_Box()
{}

ON_Box::ON_Box( const ON_BoundingBox& bbox )
{
  Create(bbox);
}

ON_Box::~ON_Box()
{}

void ON_Box::Destroy()
{
  plane = ON_xy_plane;
  dx.Destroy();
  dy.Destroy();
  dz.Destroy();
}

bool ON_Box::IsValid() const
{
  return (    dx.IsIncreasing() 
           && dy.IsIncreasing() 
           && dz.IsIncreasing() 
           && plane.IsValid() 
         );
}

bool ON_Box::Create( const ON_BoundingBox& bbox )
{
  plane = ON_xy_plane;
  dx.Set(bbox.m_min.x,bbox.m_max.x);
  dy.Set(bbox.m_min.y,bbox.m_max.y);
  dz.Set(bbox.m_min.z,bbox.m_max.z);
  return (dx.IsValid() && dy.IsValid() && dz.IsValid());
}

int ON_Box::IsDegenerate( double tolerance ) const
{
  int rc = 0;
  // 0     box is not degenerate
  // 1     box is a rectangle (degenerate in one direction)
  // 2     box is a line (degenerate in two directions)
  // 3     box is a point (degenerate in three directions)
  // 4     box is not valid
  if ( !dx.IsIncreasing() || !dy.IsIncreasing() || !dz.IsIncreasing() )
  {
    rc = 4;
  }
  else
  {
    const ON_3dVector diag(dx.Length(),dy.Length(),dz.Length());
    if ( !ON_IsValid(tolerance) || tolerance < 0.0 )
    {
      // compute scale invarient tolerance
      tolerance = diag.MaximumCoordinate()*ON_SQRT_EPSILON;
    }
    if ( diag.x <= tolerance )
      rc++;
    if ( diag.y <= tolerance )
      rc++;
    if ( diag.z <= tolerance )
      rc++;
  }
  return rc;
}

ON_3dPoint ON_Box::Center() const
{
  return plane.PointAt(dz.Mid(),dy.Mid(),dz.Mid());
}

bool ON_Box::GetCorners( ON_3dPoint* corners ) const
{
  int i,j,k,n=0;
  double r,s,t;
  for( i= 0; i < 2; i++ )
  {
    r = dx.m_t[i];
    for ( j = 0; j < 2; j++ )
    {
      s = dy.m_t[j];
      for ( k = 0; k < 2; k++ )
      {
        t = dz.m_t[k];
        corners[n++] = plane.PointAt(r,s,t);
      }
    }
  }
  return true;
}

bool ON_Box::GetCorners( ON_SimpleArray<ON_3dPoint>& corners ) const
{
  corners.Empty();
  corners.Reserve(8);
  bool rc = GetCorners(corners.Array());
  if (rc)
    corners.SetCount(8);
  return rc;
}

ON_BoundingBox ON_Box::BoundingBox() const
{
  ON_BoundingBox bbox;
  ON_3dPoint corners[8];
  if ( GetCorners(corners) )
    bbox.Set(3,0,8,3,&corners[0].x,false);
  return bbox;
}

ON_3dPoint ON_Box::PointAt( 
        double r, 
        double s, 
        double t 
        ) const
{
  // Do not validate - it is too slow.
  return plane.PointAt(r,s,t);
}

// returns point on cylinder that is closest to given point
bool ON_Box::ClosestPointTo( ON_3dPoint point, double* r, double* s, double* t ) const
{
  // Do not validate box - it is too slow.
  const ON_3dVector v = point - plane.origin;

  *r = v*plane.xaxis;
  if ( *r < dx.m_t[0] )
    *r = dx.m_t[0];
  else if ( *r > dx.m_t[1] )
    *r = dx.m_t[1];

  *s = v*plane.yaxis;
  if ( *s < dy.m_t[0] )
    *s = dy.m_t[0];
  else if ( *s > dy.m_t[1] )
    *s = dy.m_t[1];

  *t = v*plane.zaxis;
  if ( *t < dz.m_t[0] )
    *t = dz.m_t[0];
  else if ( *t > dz.m_t[1] )
    *t = dz.m_t[1];

  return true;
}

ON_3dPoint ON_Box::ClosestPointTo( ON_3dPoint point ) const
{
  // Do not validate - it is too slow.
  double r,s,t;
  ClosestPointTo(point,&r,&s,&t);
  return PointAt(r,s,t);
}

// rotate box about its center
bool ON_Box::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis // axis of rotation
      )
{
  return Rotate(sin_angle, cos_angle, axis, Center() );
}

bool ON_Box::Rotate(
      double angle,           // angle in radians
      const ON_3dVector& axis // axis of rotation
      )
{
  return Rotate(sin(angle), cos(angle), axis, plane.origin );
}

// rotate box about a point and axis
bool ON_Box::Rotate(
      double sin_angle,
      double cos_angle,
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint&  point // center of rotation
      )
{
  return plane.Rotate( sin_angle, cos_angle, axis, point );
}

bool ON_Box::Rotate(
      double angle,             // angle in radians
      const ON_3dVector& axis,  // axis of rotation
      const ON_3dPoint&  point  // center of rotation
      )
{
  return Rotate(sin(angle),cos(angle),axis,point);
}

bool ON_Box::Translate(
      const ON_3dVector& delta
      )
{
  return plane.Translate(delta);
}


bool ON_Box::Transform( const ON_Xform& xform )
{
  ON_3dPoint corners[8];
  bool rc = GetCorners(corners);
  if ( rc )
  {
    ON_Plane xplane(plane);
    rc = xplane.Transform(xform);
    if ( rc )
    {
      int i;
      for ( i = 0; i < 8; i++ )
      {
        corners[i] = xform*corners[i];
      }
      double x0,x1,x,y0,y1,y,z0,z1,z;
      ON_3dVector v = corners[7] - plane.origin;
      x0 = x1 = v*plane.xaxis;
      y0 = y1 = v*plane.yaxis;
      z0 = z1 = v*plane.zaxis;
      for ( i = 0; i < 7; i++ )
      {
        v = corners[i] - plane.origin;
        x = v*plane.xaxis;
        if ( x < x0 ) x0 = x; else if (x > x1 ) x1 = x;
        y = v*plane.yaxis;
        if ( y < y0 ) y0 = y; else if (y > y1 ) y1 = y;
        z = v*plane.zaxis;
        if ( z < z0 ) z0 = z; else if (z > z1 ) z1 = z;
      }
      double tol = ON_SQRT_EPSILON;
      if ( fabs(dx.ParameterAt(x0)) > tol || fabs(dx.ParameterAt(x1)-1.0) > tol )
        dx.Set(x0,x1);
      if ( fabs(dy.ParameterAt(y0)) > tol || fabs(dy.ParameterAt(y1)-1.0) > tol )
        dy.Set(y0,y1);
      if ( fabs(dz.ParameterAt(z0)) > tol || fabs(dz.ParameterAt(z1)-1.0) > tol )
        dz.Set(z0,z1);
    }
  }
  return rc;
}

double ON_Box::Volume() const
{
  return dx.Length()*dy.Length()*dz.Length();
}

double ON_Box::Area() const
{
  double a = dx.Length();
  double b = dy.Length();
  double c = dz.Length();
  return 2.0*(a*b + b*c + c*a);
}

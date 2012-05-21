/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
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

#include <pcl/surface/openNURBS/opennurbs.h>

ON_Quaternion ON_CrossProduct( const ON_Quaternion& p, const ON_Quaternion& q)
{
  return ON_Quaternion(0.0, p.c*q.d - p.d*q.c, p.d*q.b - p.b*q.d, p.b*q.c - p.c*q.d);
}

ON_Quaternion ON_QuaternionProduct( const ON_Quaternion& p, const ON_Quaternion& q)
{
  return ON_Quaternion( p.a*q.a - p.b*q.b - p.c*q.c - p.d*q.d,
                        p.a*q.b + p.b*q.a + p.c*q.d - p.d*q.c,
                        p.a*q.c - p.b*q.d + p.c*q.a + p.d*q.b,
                        p.a*q.d + p.b*q.c - p.c*q.b + p.d*q.a);
}


/////////////////////////////////////

const ON_Quaternion ON_Quaternion::Zero(0.0,0.0,0.0,0.0);
const ON_Quaternion ON_Quaternion::Identity(1.0,0.0,0.0,0.0);
const ON_Quaternion ON_Quaternion::I(0.0,1.0,0.0,0.0);
const ON_Quaternion ON_Quaternion::J(0.0,0.0,1.0,0.0);
const ON_Quaternion ON_Quaternion::K(0.0,0.0,0.0,1.0);

void ON_Quaternion::SetRotation(double angle, const ON_3dVector& axis)
{
  double s = axis.Length();
  s = (s > 0.0) ? sin(0.5*angle)/s : 0.0;
  a = cos(0.5*angle);
  b = s*axis.x;
  c = s*axis.y;
  d = s*axis.z;
}

ON_Quaternion ON_Quaternion::Rotation(double angle, const ON_3dVector& axis)
{
  double s = axis.Length();
  s = (s > 0.0) ? sin(0.5*angle)/s : 0.0;
  return ON_Quaternion(cos(0.5*angle),s*axis.x,s*axis.y,s*axis.z);
}

ON_Quaternion ON_Quaternion::Exp(ON_Quaternion q)
{
  // Added 8 Jan 2010 - not tested yet
  double v = ((const ON_3dVector*)&q.b)->Length();
  if ( !(v > ON_DBL_MIN) )
    v = 0.0;
  double ea = exp(q.a);
  double z = (v > 0.0) ? ea*sin(v)/v : 0.0;
  return ON_Quaternion( ea*cos(v), z*q.b, z*q.c, z*q.d );
}

ON_Quaternion ON_Quaternion::Log(ON_Quaternion q)
{
  // Added 8 Jan 2010 - not tested yet
  double lenq = q.Length();
  double v = ((const ON_3dVector*)&q.b)->Length();
  if ( !(v > ON_DBL_MIN) )
    v = 0.0;
  double z = (v > 0.0) ? acos(q.a/lenq)/v : 0.0;
  return ON_Quaternion(log(lenq), z*q.b, z*q.c, z*q.d );
}

ON_Quaternion ON_Quaternion::Pow(ON_Quaternion q,double t)
{
  // Added 8 Jan 2010 - not tested yet
  return ON_Quaternion::Exp( t * ON_Quaternion::Log(q) );
}

ON_Quaternion ON_Quaternion::Slerp(ON_Quaternion q0, ON_Quaternion q1, double t)
{
  // Added 8 Jan 2010 - not tested yet
  ON_Quaternion q;
  if ( t <= 0.5 )
  {
    q = q0.Inverse()*q1;
    q = q0*ON_Quaternion::Pow(q,t);
  }
  else
  {
    q = q1.Inverse()*q0;
    q = q1*ON_Quaternion::Pow(q,1.0-t);
  }
  return q;
}


void ON_Quaternion::Set(double qa, double qb, double qc, double qd)
{
  a = qa;
  b = qb;
  c = qc;
  d = qd;
}

void ON_Quaternion::SetRotation(const ON_Plane& plane0, const ON_Plane& plane1 )
{
  double m[3][3], r, s;
  double* q;
  int i,j,k;

  // set m[][] = rotation matrix (acting on the left)
  m[0][0] = plane1.xaxis.x*plane0.xaxis.x
          + plane1.yaxis.x*plane0.yaxis.x
          + plane1.zaxis.x*plane0.zaxis.x;
  m[0][1] = plane1.xaxis.x*plane0.xaxis.y
          + plane1.yaxis.x*plane0.yaxis.y
          + plane1.zaxis.x*plane0.zaxis.y;
  m[0][2] = plane1.xaxis.x*plane0.xaxis.z
          + plane1.yaxis.x*plane0.yaxis.z
          + plane1.zaxis.x*plane0.zaxis.z;
  m[1][0] = plane1.xaxis.y*plane0.xaxis.x
          + plane1.yaxis.y*plane0.yaxis.x
          + plane1.zaxis.y*plane0.zaxis.x;
  m[1][1] = plane1.xaxis.y*plane0.xaxis.y
          + plane1.yaxis.y*plane0.yaxis.y
          + plane1.zaxis.y*plane0.zaxis.y;
  m[1][2] = plane1.xaxis.y*plane0.xaxis.z
          + plane1.yaxis.y*plane0.yaxis.z
          + plane1.zaxis.y*plane0.zaxis.z;
  m[2][0] = plane1.xaxis.z*plane0.xaxis.x
          + plane1.yaxis.z*plane0.yaxis.x
          + plane1.zaxis.z*plane0.zaxis.x;
  m[2][1] = plane1.xaxis.z*plane0.xaxis.y
          + plane1.yaxis.z*plane0.yaxis.y
          + plane1.zaxis.z*plane0.zaxis.y;
  m[2][2] = plane1.xaxis.z*plane0.xaxis.z
          + plane1.yaxis.z*plane0.yaxis.z
          + plane1.zaxis.z*plane0.zaxis.z;

  k = 1;
  s = ON_SQRT_EPSILON;
  for ( i = 0; i < 3 && k; i++ ) for ( j = 0; j < 3; j++ )
  {
    if ( i == j )
    {
      if (fabs(m[i][i]-1.0) > s )
      {
        k = 0;
        break;
      }
    }
    else
    {
      if (fabs(m[i][j]) > s )
      {
        k = 0;
        break;
      }
    }
  }

  if ( k )
  {
    // m[][] is the identity matrix
    a = 1.0;
    b = c = d = 0.0;
    return;
  }

  i = (m[0][0] >= m[1][1]) 
    ? ((m[0][0] >= m[2][2])?0:2) 
    : ((m[1][1] >= m[2][2])?1:2);
  j = (i+1)%3;
  k = (i+2)%3;

  // Note: 
  //   For any rotation matrix, the diagonal is
  //     x^2(1-cos(t)) + cos(t), y^2(1-cos(t)) + cos(t), z^2(1-cos(t)) + cos(t),
  //   where (x,y,z) is the unit vector axis of rotation and "t" is the angle.
  //   So the trace = 1 + 2cos(t).
  //
  //   When cos(t) >= 0, m[i][i] corresponds to the axis component that has
  //   the largest absolute value.
  //
  //   
  //
  //   Observe that 
  //     s = 1 + m[i][i] - m[j][j] - m[k][k]
  //       = 1 + 2*m[i][i] - m[i][i] - m[j][j] - m[k][k]
  //       = 1 + 2*m[i][i] - trace
  //       = 2*(m[i][i] - cos(t))
  //       = 2*(w^2(1-cos(t)^2) + cos(t) - cos(t))
  //       = 2*w*w*(sin(t)^2)
  //
  //    When cos(t) >= 0, m[i][i] corresponds to the coordinate of
  //    the rotation axis with largest absolute value.


  s = 1.0 + m[i][i] - m[j][j] - m[k][k];
  if ( s > ON_DBL_MIN )
  {
    r = sqrt(s);
    s = 0.5/r;
    a = s*(m[k][j] - m[j][k]);
    q = &b;
    q[i] = 0.5*r;
    q[j] = s*(m[i][j] + m[j][i]);
    q[k] = s*(m[k][i] + m[i][k]);
  }
  else
  {
    if ( s < -1.0e-14 )
      ON_ERROR("noisy rotation matrix");
    a = 1.0;
    b = c = d = 0.0;
  }
}

ON_Quaternion ON_Quaternion::Rotation(const ON_Plane& plane0, const ON_Plane& plane1)
{
  ON_Quaternion q;
  q.SetRotation(plane0,plane1);
  return q;
}

bool ON_Quaternion::GetRotation(ON_Xform& xform) const
{
  bool rc;
  ON_Quaternion q(*this);
  if ( q.Unitize() )
  {
    if (    fabs(q.a-a) <= ON_ZERO_TOLERANCE
         && fabs(q.b-b) <= ON_ZERO_TOLERANCE
         && fabs(q.c-c) <= ON_ZERO_TOLERANCE
         && fabs(q.d-d) <= ON_ZERO_TOLERANCE
         )
    {
      // "this" was already unitized - don't tweak bits
      q = *this;
    }
    xform[1][0] = 2.0*(q.b*q.c + q.a*q.d);
    xform[2][0] = 2.0*(q.b*q.d - q.a*q.c);
    xform[3][0] = 0.0;

    xform[0][1] = 2.0*(q.b*q.c - q.a*q.d);
    xform[2][1] = 2.0*(q.c*q.d + q.a*q.b);
    xform[3][1] = 0.0;

    xform[0][2] = 2.0*(q.b*q.d + q.a*q.c);
    xform[1][2] = 2.0*(q.c*q.d - q.a*q.b);
    xform[3][2] = 0.0;

    q.b = q.b*q.b;
    q.c = q.c*q.c;
    q.d = q.d*q.d;
    xform[0][0] = 1.0 - 2.0*(q.c + q.d);
    xform[1][1] = 1.0 - 2.0*(q.b + q.d);
    xform[2][2] = 1.0 - 2.0*(q.b + q.c);

    xform[0][3] = xform[1][3] = xform[2][3] = 0.0;
    xform[3][3] = 1.0;
    rc = true;
  }
  else if ( IsZero() )
  {
    xform.Zero();
    rc = false;
  }
  else
  {
    // something is seriously wrong
    ON_ERROR("ON_Quaternion::GetRotation(ON_Xform) quaternion is invalid");
    xform.Identity();
    rc = false;
  }

  return rc;
}

bool ON_Quaternion::GetRotation(ON_Plane& plane) const
{
  plane.xaxis.x = a*a + b*b - c*c - d*d;
  plane.xaxis.y = 2.0*(a*d + b*c);
  plane.xaxis.z = 2.0*(b*d - a*c);

  plane.yaxis.x = 2.0*(b*c - a*d);
  plane.yaxis.y = a*a - b*b + c*c - d*d;
  plane.yaxis.z = 2.0*(a*b + c*d);

  plane.zaxis.x = 2.0*(a*c + b*d);
  plane.zaxis.y = 2.0*(c*d - a*b);
  plane.zaxis.z = a*a - b*b - c*c + d*d;

  plane.xaxis.Unitize();
  plane.yaxis.Unitize();
  plane.zaxis.Unitize();
  plane.origin.Set(0.0,0.0,0.0);
  plane.UpdateEquation();

  return plane.IsValid();
}

bool ON_Quaternion::GetRotation(double& angle, ON_3dVector& axis) const
{
  const double s = Length();
  angle = (s > ON_DBL_MIN) ? 2.0*acos(a/s) : 0.0;
  axis.x = b;
  axis.y = c;
  axis.z = d;
  return (axis.Unitize() && s > ON_DBL_MIN);
}

ON_3dVector ON_Quaternion::Vector() const
{
  return ON_3dVector(b,c,d);
}

double ON_Quaternion::Scalar() const
{
  return a;
}

bool ON_Quaternion::IsZero() const
{
  return (0.0 == a && 0.0 == b && 0.0 == c && 0.0 == d);
}

bool ON_Quaternion::IsNotZero() const
{
  return ( (0.0 != a || 0.0 != b || 0.0 != c || 0.0 != d) 
           && ON_IsValid(a)
           && ON_IsValid(b)
           && ON_IsValid(c) 
           && ON_IsValid(d)
         );
}

bool ON_Quaternion::IsScalar() const
{
  return (0.0 == b && 0.0 == c && 0.0 == d);
}

bool ON_Quaternion::IsVector() const
{
  return (0.0 == a && (0.0 != b || 0.0 != c || 0.0 != d));
}

bool ON_Quaternion::IsValid() const
{
  return ((ON_IS_VALID(a) && ON_IS_VALID(b) && ON_IS_VALID(c) && ON_IS_VALID(d)) ? true : false);
}

ON_Quaternion ON_Quaternion::Conjugate() const
{
  return ON_Quaternion(a,-b,-c,-d);
}

ON_Quaternion ON_Quaternion::Inverse() const
{
  double x = a*a+b*b+c*c+d*d;
  x = ( x > ON_DBL_MIN ) ? 1.0/x : 0.0;
  return ON_Quaternion(a*x,-b*x,-c*x,-d*x);
}

bool ON_Quaternion::Invert()
{
  double x = a*a+b*b+c*c+d*d;
  if ( x <= ON_DBL_MIN )
    return false;
  x = 1.0/x;
  a *= x;
  x = -x;
  b *= x;
  c *= x;
  d *= x;
  return true;
}

ON_3dVector ON_Quaternion::Rotate(ON_3dVector v) const
{
  // returns q*(0,v.x,v.y,v.z)*Inverse(q)
  double x = a*a + b*b + c*c + d*d;
  x = ( x > ON_DBL_MIN ) ? 1.0/x : 0.0;
  const ON_Quaternion qinv(a*x,-b*x,-c*x,-d*x);

  const ON_Quaternion qv( -b*v.x - c*v.y - d*v.z,
                           a*v.x + c*v.z - d*v.y,
                           a*v.y - b*v.z + d*v.x,
                           a*v.z + b*v.y - c*v.x);

  v.x = qv.a*qinv.b + qv.b*qinv.a + qv.c*qinv.d - qv.d*qinv.c;
  v.y = qv.a*qinv.c - qv.b*qinv.d + qv.c*qinv.a + qv.d*qinv.b;
  v.z = qv.a*qinv.d + qv.b*qinv.c - qv.c*qinv.b + qv.d*qinv.a;
  return v;
}

double ON_Quaternion::DistanceTo(const ON_Quaternion& q) const
{
  const ON_Quaternion pq(q.a-a,q.b-b,q.c-c,q.d-d);
  return pq.Length();
}


double ON_Quaternion::Distance(const ON_Quaternion& p, const ON_Quaternion& q)
{
  const ON_Quaternion pq(q.a-p.a,q.b-p.b,q.c-p.c,q.d-p.d);
  return pq.Length();
}


double ON_Quaternion::Length() const
{
  double len;
  double fa = fabs(a);
  double fb = fabs(b);
  double fc = fabs(c);
  double fd = fabs(d);
  if ( fb >= fa && fb >= fc && fb >= fd) 
  {
    len = fa; fa = fb; fb = len;
  }
  else if ( fc >= fa && fc >= fb && fc >= fd) 
  {
    len = fa; fa = fc; fc = len;
  }
  else if ( fd >= fa && fd >= fb && fd >= fc) 
  {
    len = fa; fa = fd; fd = len;
  }

  // 15 September 2003 Dale Lear
  //     For small denormalized doubles (positive but smaller
  //     than DBL_MIN), some compilers/FPUs set 1.0/fa to +INF.
  //     Without the ON_DBL_MIN test we end up with
  //     microscopic quaternions that have infinte norm!
  //
  //     This code is absolutely necessary.  It is a critical
  //     part of the bug fix for RR 11217.
  if ( fa > ON_DBL_MIN ) 
  {
    len = 1.0/fa;
    fb *= len;
    fc *= len;
    fd *= len;
    len = fa*sqrt(1.0 + fb*fb + fc*fc + fd*fd);
  }
  else if ( fa > 0.0 && ON_IS_FINITE(fa) )
    len = fa;
  else
    len = 0.0;

  return len;
}


double ON_Quaternion::LengthSquared() const
{
  return (a*a + b*b + c*c + d*d);
}


ON_Xform ON_Quaternion::MatrixForm() const
{
  double m[4][4];
  m[0][0] =  a; m[0][1] =  b; m[0][2] =  c; m[0][3] =  d; 
  m[1][0] = -b; m[1][1] =  a; m[1][2] = -d; m[1][3] =  c; 
  m[2][0] = -c; m[2][1] =  d; m[2][2] =  a; m[2][3] = -b; 
  m[3][0] = -d; m[3][1] = -c; m[3][2] =  b; m[3][3] =  a; 
  return ON_Xform(&m[0][0]);
}

bool ON_Quaternion::Unitize()
{
  double x = Length();

  if (x > ON_DBL_MIN)
  {
    x = 1.0/x;
    a *= x; b *= x; c *= x; d *= x;
  }
  else if ( x > 0.0 )
  {
    ON_Quaternion q(a*1.0e300,b*1.0e300,c*1.0e300,c*1.0e300);
    if ( !q.Unitize() )
      return false;
    a = q.a; 
    b = q.b;
    c = q.c;
    d = q.d;
  }
  else
  {
    return false;
  }

  return true;
}


ON_Quaternion::ON_Quaternion(double qa, double qb, double qc, double qd) : a(qa),b(qb),c(qc),d(qd) {}

ON_Quaternion::ON_Quaternion(const ON_3dVector& v) : a(0.0),b(v.x),c(v.y),d(v.z) {}

ON_Quaternion& ON_Quaternion::operator=(const ON_3dVector& v)
{
  a = 0.0;
  b = v.x;
  c = v.y;
  d = v.z;
  return *this;
}

ON_Quaternion ON_Quaternion::operator*(int x) const
{
  return ON_Quaternion(a*x,b*x,c*x,d*x);
}

ON_Quaternion ON_Quaternion::operator*(float x) const
{
  return ON_Quaternion(a*x,b*x,c*x,d*x);
}

ON_Quaternion ON_Quaternion::operator*(double x) const
{
  return ON_Quaternion(a*x,b*x,c*x,d*x);
}

ON_Quaternion ON_Quaternion::operator/(int y) const
{
  double x = (0!=y) ? 1.0/((double)y) : 0.0;
  return ON_Quaternion(a*x,b*x,c*x,d*x);
}

ON_Quaternion ON_Quaternion::operator/(float y) const
{
  double x = (0.0f!=y) ? 1.0/((double)y) : 0.0;
  return ON_Quaternion(a*x,b*x,c*x,d*x);
}

ON_Quaternion ON_Quaternion::operator/(double y) const
{
  double x = (0.0!=y) ? 1.0/((double)y) : 0.0;
  return ON_Quaternion(a*x,b*x,c*x,d*x);
}

ON_Quaternion  ON_Quaternion::operator+(const ON_Quaternion& q) const
{
  return ON_Quaternion(a+q.a,b+q.b,c+q.c,d+q.d);
}

ON_Quaternion  ON_Quaternion::operator-(const ON_Quaternion& q) const
{
  return ON_Quaternion(a-q.a,b-q.b,c-q.c,d-q.d);
}

ON_Quaternion  ON_Quaternion::operator*(const ON_Quaternion& q) const
{
  return ON_Quaternion(a*q.a - b*q.b - c*q.c - d*q.d,
                       a*q.b + b*q.a + c*q.d - d*q.c,
                       a*q.c - b*q.d + c*q.a + d*q.b,
                       a*q.d + b*q.c - c*q.b + d*q.a);
}

ON_Quaternion operator*(int x, const ON_Quaternion& q)
{
  return ON_Quaternion(x*q.a,x*q.b,x*q.c,x*q.d);
}

ON_Quaternion operator*(float x, const ON_Quaternion& q)
{
  return ON_Quaternion(x*q.a,x*q.b,x*q.c,x*q.d);
}

ON_Quaternion operator*(double x, const ON_Quaternion& q)
{
  return ON_Quaternion(x*q.a,x*q.b,x*q.c,x*q.d);
}


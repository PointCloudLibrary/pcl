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

ON_OBJECT_IMPLEMENT(ON_OffsetSurface,ON_SurfaceProxy,"00C61749-D430-4ecc-83A8-29130A20CF9C");

ON_OffsetSurface::ON_OffsetSurface()
                 : m__pSrf(0)
{
}

ON_OffsetSurface::~ON_OffsetSurface()
{
  m_offset_function.SetBaseSurface( 0 );
  if ( 0 != m__pSrf && this != m__pSrf )
    delete m__pSrf;
  m__pSrf = 0;
  
}

ON_OffsetSurface::ON_OffsetSurface( const ON_OffsetSurface& src)
                 : ON_SurfaceProxy(src),
                   m__pSrf(0),
                   m_offset_function(src.m_offset_function)
{
  if ( 0 != src.m__pSrf )
  {
    m__pSrf = src.ON_SurfaceProxy::DuplicateSurface();
    SetProxySurface(m__pSrf);
  }
  m_offset_function.SetBaseSurface( BaseSurface() );
}

ON_OffsetSurface& ON_OffsetSurface::operator=(const ON_OffsetSurface& src)
{
  if ( this != &src )
  {
    if ( 0 != m__pSrf && this != m__pSrf )
      delete m__pSrf;
    m__pSrf = 0;
    if ( 0 != src.m__pSrf )
    {
      m__pSrf = src.ON_SurfaceProxy::DuplicateSurface();
      SetProxySurface(m__pSrf);
    }
    else
    {
      ON_SurfaceProxy::operator=(src);
    }
    m_offset_function = src.m_offset_function;
    m_offset_function.SetBaseSurface( BaseSurface() );
  }
  return *this;
}

ON_OffsetSurfaceFunction& ON_OffsetSurface::OffsetFunction()
{
  return m_offset_function;
}

const ON_OffsetSurfaceFunction& ON_OffsetSurface::OffsetFunction() const
{
  return m_offset_function;
}


bool ON_OffsetSurface::SetBaseSurface(
  const ON_Surface* base_surface
  )
{
  bool rc = false;
  if ( this != base_surface )
  {
    rc = true;
    if ( 0 == base_surface )
    {
      if ( 0 != m__pSrf && this != m__pSrf )
        delete m__pSrf;
      m__pSrf = 0;
      ON_SurfaceProxy::SetProxySurface(0);
      m_offset_function.SetBaseSurface(0);
    }
    else if ( BaseSurface() != base_surface )
    {
      if ( 0 != m__pSrf && this != m__pSrf )
        delete m__pSrf;
      m__pSrf = 0;
      ON_SurfaceProxy::SetProxySurface(base_surface);
    }
    m_offset_function.SetBaseSurface( BaseSurface() );
  }
  return rc;
}

bool ON_OffsetSurface::SetBaseSurface(
      ON_Surface* base_surface, 
      bool bManage
      )
{
  bool rc = SetBaseSurface(base_surface);
  if (rc && bManage )
    m__pSrf = base_surface;
  return rc;
}

const ON_Surface* ON_OffsetSurface::BaseSurface() const
{
  return ProxySurface();
}

ON_BOOL32 ON_OffsetSurface::GetBBox(
       double* bbox_min,
       double* bbox_max,
       ON_BOOL32 bGrowBox
       ) const
{
  ON_BOOL32 rc = ON_SurfaceProxy::GetBBox(bbox_min,bbox_max);
  if ( rc )
  {
    double d, distance = 0.0;
    int i, count = m_offset_function.m_offset_value.Count();
    for ( i = 0; i < count; i++ )
    {
      d = fabs(m_offset_function.m_offset_value[i].m_distance);
      if ( distance < d)
        distance = d;
    }
    distance *= 2;
    if ( 0 != bbox_min )
    {
      bbox_min[0] -= distance;
      bbox_min[1] -= distance;
      bbox_min[2] -= distance;
    }
    if ( 0 != bbox_max )
    {
      bbox_max[0] += distance;
      bbox_max[1] += distance;
      bbox_max[2] += distance;
    }
  }
  return rc;
}


ON_BOOL32 ON_OffsetSurface::Evaluate(
       double s, double t,
       int der_count,
       int v_stride,
       double* v,
       int side,
       int* hint
       ) const
{
  int vv_stride = v_stride;
  double* vv = v;
  ON_3dVector srf_value[6];//, normal_value[3];
  if ( der_count < 2 )
  {
    vv = &srf_value[0].x;
    vv_stride = 3;
  }

  ON_BOOL32 rc = ON_SurfaceProxy::Evaluate(s,t,(der_count>2?der_count:2),vv_stride,vv,side,hint);

  if ( v != vv )
  {
    // save answer in v[] array
    v[0] = srf_value[0].x;
    v[1] = srf_value[0].y;
    v[2] = srf_value[0].z;
    if ( der_count > 0 )
    {
      v[v_stride]   = srf_value[1].x;
      v[v_stride+1] = srf_value[1].y;
      v[v_stride+2] = srf_value[1].z;
      v[2*v_stride]   = srf_value[2].x;
      v[2*v_stride+1] = srf_value[2].y;
      v[2*v_stride+2] = srf_value[2].z;
    }
  }
  else
  {
    srf_value[0] = v;
    srf_value[1] = v+v_stride;
    srf_value[2] = v+2*v_stride;
    srf_value[3] = v+3*v_stride;
    srf_value[4] = v+4*v_stride;
    srf_value[5] = v+5*v_stride;
  }

  if (rc)
  {
    double darray[21]; // 21 = ((5+1)*(5+2)/2) = enough room for der_count <= 5
    double* d = (der_count>5)
              ? (double*)onmalloc(((der_count+1)*(der_count+2))/2*sizeof(d[0]))
              : darray;
    rc = m_offset_function.EvaluateDistance(s,t,der_count,d);
    if (rc)
    {
      ON_3dVector N;
      ON_EvNormal(side,
                  srf_value[1], srf_value[2],
                  srf_value[3], srf_value[4], srf_value[5],
                  N);
      v[0] += d[0]*N.x;
      v[1] += d[0]*N.y;
      v[2] += d[0]*N.z;

      if ( der_count > 0 )
      {
        ON_3dVector Ns, Nt;
        ON_EvNormalPartials(srf_value[1], srf_value[2],
                  srf_value[3], srf_value[4], srf_value[5],
                  Ns, Nt);
        v[v_stride]   += d[0]*Ns.x + d[1]*N.x;
        v[v_stride+1] += d[0]*Ns.y + d[1]*N.y;
        v[v_stride+2] += d[0]*Ns.z + d[1]*N.z;

        v[2*v_stride]   += d[0]*Nt.x + d[2]*N.x;
        v[2*v_stride+1] += d[0]*Nt.y + d[2]*N.y;
        v[2*v_stride+2] += d[0]*Nt.z + d[2]*N.z;
      }
    }
    if ( d != darray )
      onfree(d);
  }

  return rc;
}


void ON_BumpFunction::EvaluateHelperLinearBump(double t, double dt, int der_count, double* value) const
{
  value[0] = t;
  if (der_count>0)
  {
    value[1] = dt;
    if ( der_count > 1 )
    {
      der_count--;
      value += 2;
      while(der_count--)
        *value++ = 0.0;
    }
  }
}

void ON_BumpFunction::EvaluateHelperQuinticBump(double t, double dt, int der_count, double* value) const
{
  // c(t) = (1-t)^3 * (1 + 3t + 6t^2)
  //bool neg = (t<0.0);
  //t = fabs(t);
  if ( fabs(t) < 1.0)
  {
    double a2 = (1-t);
    double a1 = a2*a2;
    double a = a1*a2;
    double b = 1.0 + t*(3.0 + 6.0*t);
    value[0] = a*b;
    if (der_count>0)
    {
      a1 *= -3.0;
      double b1 = 3.0 + 12.0*t;
      value[1] = dt*(a1*b + b1*a);
      //if ( neg )
      //  value[1] = - value[1];
      if ( der_count > 1 )
      {
        value[2] = dt*dt*(6.0*a2*b + 12.0*a + 2.0*a1*b1);
        if ( der_count > 2 )
        {
          der_count-=2;
          value += 3;
          while ( der_count-- )
            *value++ = 0.0;
        }
      }
    }
  }
  else
  {
    while ( der_count-- >= 0 )
      *value++ = 0.0;
  }
}

/*
double EvaluateCubicBump(double t, double* d, double* dd) const
{
  // c(t) = 1 - 3t^2 + 2t^3
  // CubicBump(t) = c(t)  if 0 <= t < 1
  //                c(-t) if -1 < t <= 0
  //                0 otherwise
  bool neg = (t<0.0);
  t = fabs(t);
  if ( t < 1.0)
  {
    // f(t) = 1 - 3t^2 + 2t^3
    if (d)
    {
      // f'(t) = -6t + 6t^2
      *d = (neg) ? (6.0*t*(1.0-t)) : (6.0*t*(t-1.0));
    }
    if (dd)
    {
      // f"(t) = -6 + 12t
      *dd = -6.0 + 12.0*t;
    }
    t = 1.0+t*(t*(2.0*t-3.0));
  }
  else
  {
    t = 0.0;
    if ( d )
      *d = 0.0
    if ( *dd )
      *dd = 0.0
  }
  return t
}
*/

void ON_BumpFunction::Evaluate(double s, double t, int der_count, double* value) const
{
  double tmp[20];
  double* xvalue;
  double* yvalue;
  xvalue = ( der_count > 9 ) 
         ? ((double*)onmalloc((der_count+1)*2*sizeof(xvalue[0])))
         : &tmp[0];
  yvalue = xvalue + (der_count+1);
  double x = s-m_x0;
  const double dx = m_sx[x >= 0.0 ? 1 : 0];
  x *= dx;
  double y = t-m_y0;
  const double dy = m_sy[y >= 0.0 ? 1 : 0];
  y *= dy;

  if ( 5 == m_type[0] )
  {
    EvaluateHelperQuinticBump(x,dx,der_count,xvalue);
  }
  else
  {
    EvaluateHelperLinearBump(x,dx,der_count,xvalue);
  }
  if ( 5 == m_type[1] )
  {
    EvaluateHelperQuinticBump(y,dy,der_count,yvalue);
  }
  else
  {
    EvaluateHelperLinearBump(y,dy,der_count,yvalue);
  }

  int n, i, j;
  for ( n = 0; n <= der_count; n++ )
  {
    for ( i = n, j = 0; j <= n; i--, j++ )
    {
      *value++ = m_a*xvalue[i]*yvalue[j]; // d^nf/(ds^i dt^j)
    }
  }
}


bool ON_OffsetSurfaceFunction::SetBaseSurface( const ON_Surface* srf )
{
  bool rc = false;
  Destroy();
  m_srf = srf;
  if ( 0 != m_srf )
  {
    m_domain[0] = m_srf->Domain(0);
    m_domain[1] = m_srf->Domain(1);
    rc = m_domain[0].IsIncreasing() && m_domain[1].IsIncreasing();
    if ( !rc )
      Destroy();
  }
  return rc;
}

bool ON_OffsetSurfaceFunction::SetSideTangency(
  int side,
  bool bEnable
  )
{
  bool rc = false;
  if ( 0 <= side && side < 4 )
  {
    m_bZeroSideDerivative[side] = bEnable?true:false;
    m_bValid = false;
    rc = true;
  }
  return rc;
}

bool ON_OffsetSurfaceFunction::SideTangency(int side) const
{
  bool rc = ( 0 <= side && side < 4 ) 
          ? m_bZeroSideDerivative[side] 
          : false;
  return rc;
}

int ON_OffsetSurfaceFunction::OffsetPointCount() const
{
  return (0 != m_srf) ? m_offset_value.Count() : 0;
}


double ON_OffsetSurfaceFunction::DistanceAt(
  double s,
  double t
  ) const
{
  double d = 0.0;
  EvaluateDistance( s, t, 0, &d );
  return d;
}

bool ON_OffsetSurfaceFunction::EvaluateDistance(
      double s,
      double t,
      int num_der,
      double* value
      ) const
{
  const int vcnt = ((num_der+1)*(num_der+2))/2;
  int vi;
  for ( vi = 0; vi < vcnt; vi++ )
  {
    value[vi] = 0;
  }

  bool rc = const_cast<ON_OffsetSurfaceFunction*>(this)->Initialize();

  if (rc)
  {
    double barray[21];
    double* bump_value = (vcnt > 21)
                       ? (double*)onmalloc(vcnt*sizeof(bump_value[0]))
                       : barray;
    const int bump_count = m_bumps.Count();
    int bump_index, vi;
    for ( bump_index = 0; bump_index < bump_count; bump_index++ )
    {
      m_bumps[bump_index].Evaluate( s, t, num_der, bump_value );
      for ( vi = 0; vi < vcnt; vi++ )
      {
        value[vi] += bump_value[vi];
      }
    }
    if ( bump_value != barray )
      onfree(bump_value);
  }
  return rc;
}


ON_3dPoint ON_OffsetSurfaceFunction::PointAt(
  double s,
  double t
  ) const
{
  //double d = 0.0;
  ON_3dPoint P;
  ON_3dVector N;
  if ( 0 != m_srf )
  {
    if ( m_srf->EvNormal(s,t,P,N) )
    {
      P = P + DistanceAt(s,t)*N;
    }
  }
  return P;
}

ON_2dPoint ON_OffsetSurfaceFunction::OffsetSurfaceParameter(int i) const
{
  ON_2dPoint p(ON_UNSET_VALUE,ON_UNSET_VALUE);
  if ( 0 != m_srf && i >= 0 && i < m_offset_value.Count() )
  {
    p.x = m_offset_value[i].m_s;
    p.y = m_offset_value[i].m_t;
  }
  return p;
}

const ON_Surface* ON_OffsetSurfaceFunction::BaseSurface() const
{
  return m_srf;
}

double ON_OffsetSurfaceFunction::OffsetDistance(int i) const
{
  double d = ON_UNSET_VALUE;
  if ( 0 != m_srf && i >= 0 && i < m_offset_value.Count() )
  {
    d = m_offset_value[i].m_distance;
  }
  return d;
}

bool ON_OffsetSurfaceFunction::SetOffsetPoint(
  double s,
  double t,
  double distance,
  double radius
  )
{
  bool rc = false;
  if ( ON_IsValid(s) && ON_IsValid(t) && ON_IsValid(distance) && ON_IsValid(radius) )
  {
    double u = m_domain[0].NormalizedParameterAt(s);

    // 14 Jan 2008, Mikko, TRR 29861:
    // Changing the clamping to happen when the 
    // point is outside or nearly outside the domain.
    const double dTol = ON_SQRT_EPSILON; // tiny border around untrimmed edges

    if ( u < dTol)
    {
      s = m_domain[0][0];
      u = 0.0;
    }
    if ( u > 1.0-dTol)
    {
      s = m_domain[0][1];
      u = 1.0;
    }

    double v = m_domain[1].NormalizedParameterAt(t);
    if ( v < dTol)
    {
      t = m_domain[1][0];
      v = 0.0;
    }
    if ( v > 1.0-dTol)
    {
      t = m_domain[1][1];
      v = 1.0;
    }

    if ( u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0 )
    {
      ON_OffsetSurfaceValue offset_value;
      offset_value.m_s = s;
      offset_value.m_t = t;
      offset_value.m_distance = distance;
      offset_value.m_radius = (radius > 0.0) ? radius : 0.0;
      offset_value.m_index = (int)((u + v*4096.0)*4096.0);
      int i;
      for ( i = 0; i < m_offset_value.Count(); i++ )
      {
        if ( m_offset_value[i].m_index == offset_value.m_index )
        {
          m_offset_value[i] = offset_value;
          break;
        }
      }
      if (i == m_offset_value.Count())
      {
        m_offset_value.Append(offset_value);
        m_bumps.SetCount(0);
        m_bValid = false;
      }
      rc = true;
    }
  }
  return rc;
}


bool ON_OffsetSurfaceFunction::SetDistance( int index, double distance)
{
  int count = m_offset_value.Count();
  if( index < 0 || index > count-1)
    return false;

  m_offset_value[index].m_distance = distance;
  m_bValid = false;

  return true;
}


bool ON_OffsetSurfaceFunction::SetPoint( int index, double s, double t)
{
  int count = m_offset_value.Count();
  if( index < 0 || index > count-1)
    return false;

  m_offset_value[index].m_s = s;
  m_offset_value[index].m_t = t;
  m_bValid = false;

  return true;
}


void ON_OffsetSurfaceFunction::Destroy()
{
  m_srf = 0;
  m_bZeroSideDerivative[0] = false;
  m_bZeroSideDerivative[1] = false;
  m_bZeroSideDerivative[2] = false;
  m_bZeroSideDerivative[3] = false;

  m_domain[0].Destroy();
  m_domain[1].Destroy();
  m_bumps.SetCount(0);
  m_bValid = false;
}



ON_OffsetSurfaceFunction::ON_OffsetSurfaceFunction()
{
  Destroy();
}

ON_OffsetSurfaceFunction::~ON_OffsetSurfaceFunction()
{
  Destroy();
}

bool ON_OffsetSurfaceFunction::Initialize()
{
  const int count = m_offset_value.Count();
  if ( !m_bValid && 0 != m_srf && count > 0)
  {
    ON_Workspace ws;
    m_bumps.SetCount(0);
    m_bumps.Reserve(count);
    int i;
    double a,b,ds,dt;
    
    for (i = 0; i < count; i++ )
    {
      ON_BumpFunction& bump = m_bumps.AppendNew();
      ON_OffsetSurfaceValue offset_value = m_offset_value[i];
      double ds0 = offset_value.m_s - m_domain[0][0];
      double ds1 = m_domain[0][1] - offset_value.m_s;
      if ( 0.0 == ds0 )
        ds0 = -ds1;
      else if ( 0.0 == ds1 )
        ds1 = -ds0;
      double dt0 = offset_value.m_t - m_domain[1][0];
      double dt1 = m_domain[1][1] - offset_value.m_t;
      if ( 0.0 == dt0 )
        dt0 = -dt1;
      else if ( 0.0 == dt1 )
        dt1 = -dt0;

      // default is a large cubic bump
      bump.m_point.x = offset_value.m_s;
      bump.m_point.y = offset_value.m_t;
      bump.m_x0 = bump.m_point.x;
      bump.m_y0 = bump.m_point.y;

      bump.m_sx[0] = -1.0/ds0;
      bump.m_sx[1] =  1.0/ds1;
      bump.m_sy[0] = -1.0/dt0;
      bump.m_sy[1] =  1.0/dt1;
      bump.m_type[0] = 5;
      bump.m_type[1] = 5;
      bump.m_a = 1.0;

      if ( offset_value.m_radius > 0.0 )
      {
        // user specified cubic bump size
        ON_3dPoint Pt;
        ON_3dVector Ds, Dt;
        if ( m_srf->Ev1Der(offset_value.m_s,offset_value.m_t,Pt,Ds,Dt) )
        {
          ds = (ds0>ds1) ? ds0 : ds1;
          dt = (dt0>dt1) ? dt0 : dt1;
          a = Ds.Length();
          if ( a > ON_ZERO_TOLERANCE )
          {
            b = offset_value.m_radius/a;
            if ( b < ds )
              ds = b;
          }
          a = Dt.Length();
          if ( a > ON_ZERO_TOLERANCE )
          {
            b = offset_value.m_radius/a;
            if ( b < dt )
              dt = b;
          }
          if ( !m_bZeroSideDerivative[0] || dt < dt0 )
            dt0 = dt;
          if ( !m_bZeroSideDerivative[1] || ds < ds1 )
            ds1 = ds;
          if ( !m_bZeroSideDerivative[2] || dt < dt1 )
            dt1 = dt;
          if ( !m_bZeroSideDerivative[3] || ds < ds0 )
            ds0 = ds;
          bump.m_sx[0] = -1.0/ds0;
          bump.m_sx[1] =  1.0/ds1;
          bump.m_sy[0] = -1.0/dt0;
          bump.m_sy[1] =  1.0/dt1;
        }
      }
      else if ( bump.m_point.x == m_domain[0][0] &&  bump.m_point.y == m_domain[1][0] )
      {
        // SW corner bilinear bump
        if ( !m_bZeroSideDerivative[1] && !m_bZeroSideDerivative[3] )
        {
          bump.m_type[0] = 1;
          bump.m_x0 = m_domain[0][1];
          bump.m_sx[0] = -1.0/m_domain[0].Length();
          bump.m_sx[1] = -1.0/m_domain[0].Length();
        }
        if ( !m_bZeroSideDerivative[0] && !m_bZeroSideDerivative[2] )
        {
          bump.m_type[1] = 1;
          bump.m_y0 = m_domain[1][1];
          bump.m_sy[0] = -1.0/m_domain[1].Length();
          bump.m_sy[1] = -1.0/m_domain[1].Length();
        }
      }
      else if ( bump.m_point.x == m_domain[0][1] &&  bump.m_point.y == m_domain[1][0] )
      {
        // SE corner bilinear bump
        if ( !m_bZeroSideDerivative[1] && !m_bZeroSideDerivative[3] )
        {
          bump.m_type[0] = 1;
          bump.m_x0 = m_domain[0][0];
          bump.m_sx[0] = 1.0/m_domain[0].Length();
          bump.m_sx[1] = 1.0/m_domain[0].Length();
        }
        if ( !m_bZeroSideDerivative[0] && !m_bZeroSideDerivative[2] )
        {
          bump.m_type[1] = 1;
          bump.m_y0 = m_domain[1][1];
          bump.m_sy[0] = -1.0/m_domain[1].Length();
          bump.m_sy[1] = -1.0/m_domain[1].Length();
        }
      }
      else if ( bump.m_point.x == m_domain[0][1] &&  bump.m_point.y == m_domain[1][1] )
      {
        // NE corner bilinear bump
        if ( !m_bZeroSideDerivative[1] && !m_bZeroSideDerivative[3] )
        {
          bump.m_type[0] = 1;
          bump.m_x0 = m_domain[0][0];
          bump.m_sx[0] = 1.0/m_domain[0].Length();
          bump.m_sx[1] = 1.0/m_domain[0].Length();
        }
        if ( !m_bZeroSideDerivative[0] && !m_bZeroSideDerivative[2] )
        {
          bump.m_type[1] = 1;
          bump.m_y0 = m_domain[1][0];
          bump.m_sy[0] = 1.0/m_domain[1].Length();
          bump.m_sy[1] = 1.0/m_domain[1].Length();
        }
      }
      else if ( bump.m_point.x == m_domain[0][0] &&  bump.m_point.y == m_domain[1][1] )
      {
        // NW corner bilinear bump
        if ( !m_bZeroSideDerivative[1] && !m_bZeroSideDerivative[3] )
        {
          bump.m_x0 = m_domain[0][1];
          bump.m_sx[0] = -1.0/m_domain[0].Length();
          bump.m_sx[1] = -1.0/m_domain[0].Length();
          bump.m_type[0] = 1;
        }
        if ( !m_bZeroSideDerivative[0] && !m_bZeroSideDerivative[2] )
        {
          bump.m_y0 = m_domain[1][0];
          bump.m_sy[0] = 1.0/m_domain[1].Length();
          bump.m_sy[1] = 1.0/m_domain[1].Length();
          bump.m_type[1] = 1;
        }
      }
    }


    ON_Matrix M(count,count);
    double* B = (double*)onmalloc(2*count*sizeof(*B));
    double* X = B + count;
    int j;
    for ( i = 0; i < count; i++ ) 
    {
      ON_2dPoint p = m_bumps[i].m_point;
      B[i] = m_offset_value[i].m_distance;
      for ( j = 0; j < count; j++ )
      {
        M[i][j] = m_bumps[j].ValueAt(p.x,p.y);
      }
    }
    int rank = M.RowReduce(ON_ZERO_TOLERANCE,B);
    if ( count == rank )
    {
      if ( M.BackSolve(ON_ZERO_TOLERANCE,count,B,X) )
      {
        m_bValid = true;
      }
    }

    if ( !m_bValid )
    {
#if 0 //defined(TL2_MATRIX_INC_)
      // Have access to SVD - try it
      for ( i = 0; i < count; i++ ) 
      {
        ON_2dPoint p = m_bumps[i].m_point;
        B[i] = m_offset_value[i].m_distance;
        for ( j = 0; j < count; j++ )
        {
          M[i][j] = m_bumps[j].ValueAt(p.x,p.y);
        }
      }
      ON_Matrix U, V;
      double* diagonal = (double*)onmalloc(2*count*sizeof(*diagonal));
      double* inverted_diagonal = diagonal + count;
      if ( TL2_MatrixSVD(M,U,diagonal,V,30) )
      {
        rank = TL2_MatrixSVDInvertDiagonal(count,diagonal,inverted_diagonal);
        if ( rank > 0 )
        {
          if ( TL2_MatrixSVDSolve(U,inverted_diagonal,V,1,B,1,X) )
          {
            m_bValid = true;
          }
        }
      }
#endif
    }

    if ( m_bValid )
    {
      for ( i = 0; i < count; i++ )
      {
        m_bumps[i].m_a = X[i];
      }
    }

    onfree(B);
  }
  return m_bValid;
}

ON_BumpFunction::ON_BumpFunction()
{
  m_point.x = ON_UNSET_VALUE;
  m_point.y = ON_UNSET_VALUE;
  m_type[0] = 0;
  m_type[1] = 0;
  m_x0 = 0.0;
  m_y0 = 0.0;
  m_sx[0] = 0.0;
  m_sx[1] = 0.0;
  m_sy[0] = 0.0;
  m_sy[1] = 0.0;
  m_a = 0.0;
}

bool ON_BumpFunction::operator==(const ON_BumpFunction& other) const
{
  return ( m_point.x == other.m_point.x && m_point.y == other.m_point.y );
}

bool ON_BumpFunction::operator<(const ON_BumpFunction& other) const
{
  return ( (m_point.x < other.m_point.x) || (m_point.x == other.m_point.x && m_point.y < other.m_point.y) );
}

bool ON_BumpFunction::operator>(const ON_BumpFunction& other) const
{
  return ( (m_point.x > other.m_point.x) || (m_point.x == other.m_point.x && m_point.y > other.m_point.y) );
}

double ON_BumpFunction::ValueAt(
  double s,
  double t
  ) const
{
  double v;
  Evaluate(s,t,0,&v);
  return v;
}


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

ON_OBJECT_IMPLEMENT(ON_CurveProxy,ON_Curve,"4ED7D4D9-E947-11d3-BFE5-0010830122F0");

ON_CurveProxy::ON_CurveProxy() : m_real_curve(0), m_bReversed(0)
{}

ON_CurveProxy::ON_CurveProxy( const ON_CurveProxy& src ) 
             : ON_Curve(src), m_real_curve(0), m_bReversed(0)
{
  *this = src;
}

ON_CurveProxy::ON_CurveProxy( const ON_Curve* c ) 
              : m_real_curve(c), m_bReversed(0)
{
  if ( m_real_curve )
    m_real_curve_domain =m_this_domain = m_real_curve->Domain();
}

ON_CurveProxy::ON_CurveProxy( const ON_Curve* c, ON_Interval domain ) 
             : m_real_curve(c), 
               m_bReversed(0),
               m_real_curve_domain(domain),
               m_this_domain(domain)
{
}

unsigned int ON_CurveProxy::SizeOf() const
{
  unsigned int sz = ON_Curve::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Curve));
  // Do not add in size of m_real_curve - its memory is not
  // managed by this class.
  return sz;
}

ON__UINT32 ON_CurveProxy::DataCRC(ON__UINT32 current_remainder) const
{
  if ( m_real_curve )
    current_remainder = m_real_curve->DataCRC(current_remainder);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bReversed),&m_bReversed);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_real_curve_domain),&m_real_curve_domain);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_this_domain),&m_this_domain);
  return current_remainder;
}

ON_CurveProxy& ON_CurveProxy::operator=( const ON_CurveProxy& src )
{
  if ( this != &src ) 
  {
    ON_Curve::operator=(src);
    m_real_curve = src.m_real_curve;
    m_bReversed = src.m_bReversed;
    m_real_curve_domain = src.m_real_curve_domain;
    m_this_domain = src.m_this_domain;
  }
  return *this;
}

ON_CurveProxy::~ON_CurveProxy()
{
  m_real_curve = 0;
}

double ON_CurveProxy::RealCurveParameter( double t ) const
{
  // change a "this" curve parameter into an m_real_curve parameter
  double s;
  if ( m_bReversed || m_real_curve_domain != m_this_domain )
  {
    s = m_this_domain.NormalizedParameterAt(t);
    if (m_bReversed)
      s = 1.0 - s;
    t = m_real_curve_domain.ParameterAt(s);
  }
  return t;
}

double ON_CurveProxy::ThisCurveParameter( double real_curve_parameter ) const
{
  // change an m_real_curve curve parameter into a "this" parameter
  double s;
  double t = real_curve_parameter;
  if ( m_bReversed || m_real_curve_domain != m_this_domain )
  {
    s = m_real_curve_domain.NormalizedParameterAt(real_curve_parameter);
    if (m_bReversed)
      s = 1.0 - s;
    t = m_this_domain.ParameterAt(s);
  }
  return t;
}

ON_Interval ON_CurveProxy::RealCurveInterval( const ON_Interval* sub_domain ) const
{
  if ( !sub_domain )
    return m_real_curve_domain;
  ON_Interval d = m_this_domain;
  d.Intersection(*sub_domain);
  double t0 = RealCurveParameter( d[m_bReversed?1:0] );
  double t1 = RealCurveParameter( d[m_bReversed?0:1] );
  return ON_Interval(t0,t1);
}


bool ON_CurveProxy::ProxyCurveIsReversed() const
{
  return m_bReversed;
}

void ON_CurveProxy::SetProxyCurve( const ON_Curve* real_curve )
{
  // setting m_real_curve=0 prevents crashes if user has deleted
  // the "real" curve before calling SetProxyCurve().
  m_real_curve = 0;

  if ( real_curve )
    SetProxyCurve( real_curve, real_curve->Domain() );
  else
  {
    DestroyCurveTree();
    m_real_curve_domain.Destroy();
    m_this_domain.Destroy();
    m_bReversed = false;
  }
}

void ON_CurveProxy::SetProxyCurve( const ON_Curve* real_curve, 
                                   ON_Interval real_curve_subdomain)
{
  if ( real_curve != this )
  {
    // setting m_real_curve=0 prevents crashes if user has deleted
    // the "real" curve before calling SetProxyCurve().
    m_real_curve = 0;
    DestroyCurveTree();
    m_real_curve_domain.Destroy();
    m_this_domain.Destroy();
    m_bReversed = false;
  }
  else
  {
    // If you are debugging and end up here, there is a 99% chance
    // that you passed the wrong pointer to SetProxyCurve().
    // However, I will assume you really meant to use a fancy self
    // reference to adjust domains.
    if ( IsValid() && m_this_domain.Includes(real_curve_subdomain) )
    {
      real_curve = m_real_curve;
      // because input real_curve_subdomain was with respect to "this".
      double r0 = RealCurveParameter(real_curve_subdomain[0]);
      double r1 = RealCurveParameter(real_curve_subdomain[1]);
      real_curve_subdomain.Set(r0,r1);
    }
    else
    {
      real_curve = 0;
    }

    // setting m_real_curve=0 prevents crashes if user has deleted
    // the "real" curve before calling SetProxyCurve().
    m_real_curve = 0;
    DestroyCurveTree();
  }

  m_real_curve = real_curve;
  if ( m_real_curve )
  {
    SetProxyCurveDomain( real_curve_subdomain );
  }
  else
  {
    m_real_curve_domain = real_curve_subdomain;
  }
  m_this_domain = m_real_curve_domain;
}

const ON_Curve* ON_CurveProxy::ProxyCurve() const
{
  return m_real_curve;
}

bool ON_CurveProxy::SetProxyCurveDomain( ON_Interval proxy_curve_subdomain )
{
  DestroyCurveTree();
  bool rc = proxy_curve_subdomain.IsIncreasing();
  if ( rc )
  {
    if ( m_real_curve )
    {
      ON_Interval cdom = m_real_curve->Domain();
      cdom.Intersection( proxy_curve_subdomain );
      rc = cdom.IsIncreasing();
      if (rc )
        m_real_curve_domain = cdom;
    }
    else
    {
      m_real_curve_domain = proxy_curve_subdomain;
    }
  }
  return rc;
}

ON_Interval ON_CurveProxy::ProxyCurveDomain() const
{
  return m_real_curve_domain;
}

ON_Curve* ON_CurveProxy::DuplicateCurve() const
{
  // duplicate underlying curve
  ON_Curve* dup_crv = 0;
  if ( m_real_curve && m_real_curve != this )
  {
    dup_crv = m_real_curve->DuplicateCurve();
    if ( dup_crv )
    {
      dup_crv->Trim(m_real_curve_domain);
      if( m_bReversed )
        dup_crv->Reverse();
      dup_crv->SetDomain( m_this_domain );
    }
  }
  return dup_crv;
}


ON_BOOL32
ON_CurveProxy::IsValid( ON_TextLog* text_log ) const
{
  ON_BOOL32 rc = ( m_real_curve ) ? m_real_curve->IsValid(text_log) : false;

  if ( rc && !m_real_curve_domain.IsIncreasing() )
  {
    rc = false;
    if ( text_log) 
      text_log->Print("ON_CurveProxy.m_real_curve_domain is not increasing.\n");
  }

  if ( rc && !m_real_curve->Domain().Includes( m_real_curve_domain ) )
  {
    rc = false;
    if ( text_log) 
      text_log->Print("ON_CurveProxy.m_real_curve_domain is not included m_real_curve->Domain().\n");
  }

  if ( rc && !m_this_domain.IsIncreasing() )
  {
    rc = false;
    if ( text_log) 
      text_log->Print("ON_CurveProxy.m_this_domain is not increasing.\n");
  }

  return rc;
}

void
ON_CurveProxy::Dump( ON_TextLog& dump ) const
{
  dump.Print("ON_CurveProxy uses %x on [%g,%g]\n",m_real_curve,m_real_curve_domain[0],m_real_curve_domain[1]);
}

ON_BOOL32 
ON_CurveProxy::Write(
       ON_BinaryArchive&  // open binary file
     ) const
{
  return false;
}

ON_BOOL32 
ON_CurveProxy::Read(
       ON_BinaryArchive&  // open binary file
     )
{
  return false;
}

int 
ON_CurveProxy::Dimension() const
{
  return ( m_real_curve ) ? m_real_curve->Dimension() : 0;
}

ON_BOOL32 
ON_CurveProxy::GetBBox( // returns true if successful
         double* boxmin,    // minimum
         double* boxmax,    // maximum
         ON_BOOL32 bGrowBox
         ) const
{
  return ( m_real_curve ) ? m_real_curve->GetBBox(boxmin,boxmax,bGrowBox) : false;
}

ON_BOOL32
ON_CurveProxy::Transform( const ON_Xform& )
{
  return false; // cannot modify proxy objects
}

ON_Interval ON_CurveProxy::Domain() const
{
  return m_this_domain;
}

ON_BOOL32 ON_CurveProxy::SetDomain( double t0, double t1 )
{
  ON_BOOL32 rc = false;
  if (t0 < t1)
  {
		DestroyCurveTree();
    m_this_domain.Set(t0, t1);
    rc = true;
  }
  return rc;
}


bool ON_CurveProxy::SetDomain( ON_Interval domain )
{
  return SetDomain( domain[0], domain[1] ) ? true : false;
}

//Do not change SpanCount() without making sure it gives the correct result for use in GetSpanVector()
int ON_CurveProxy::SpanCount() const
{
  if (!m_real_curve) return 0;
  int rsc = m_real_curve->SpanCount();
  ON_Interval domain = m_real_curve->Domain();
  if (m_real_curve_domain == domain) 
    return rsc;
  double* rsv = (double*)onmalloc((rsc+1)*sizeof(double));
  if (!rsv) return 0;
  if (!m_real_curve->GetSpanVector(rsv)){
    onfree((void*)rsv);
    return 0;
  }

  int i=0;
  int sc = 0;
  
  while (i <= rsc && rsv[i] <= m_real_curve_domain[0]) i++;
  while (i <= rsc && rsv[i] < m_real_curve_domain[1]){
    sc++;
    i++;
  }
  sc++;
  onfree((void*)rsv);

  return sc;
}


//Do not change GetSpanVector() without making sure it is consistent with SpanCount()
ON_BOOL32 ON_CurveProxy::GetSpanVector( double* d ) const
{

#if 0
  ON_BOOL32 rc = m_real_curve ? m_real_curve->GetSpanVector(d) : false;
  if (rc && (m_bReversed || m_this_domain != m_real_curve_domain) ) 
  {
    double x;
    int i, j, count = SpanCount();
    for ( i = 0; i <= count; i++ )
    {
      x = m_real_curve_domain.NormalizedParameterAt(d[i]);
      d[i] = x;
    }
    if ( m_bReversed )
    {
      for ( i = 0, j = count; i <= j; i++, j-- ) 
      {
        x = d[i];
        d[i] = 1.0-d[j];
        d[j] = 1.0-x;
      }
    }
    for ( i = 0; i <= count; i++ )
    {
      d[i] = m_this_domain.ParameterAt(d[i]);
    }
  }
  return rc;
#endif

  if (!m_real_curve) return false;
  int rsc = m_real_curve->SpanCount();
  if (rsc < 1) return false;
  double* rsv = (double*)onmalloc((rsc+1)*sizeof(double));
  if (!rsv || !m_real_curve->GetSpanVector(rsv)) return false;
  ON_Interval domain = m_real_curve->Domain();

  if (m_real_curve_domain == m_this_domain && m_real_curve_domain == domain){
    int i;
    for (i=0; i <= rsc; i++) d[i] = rsv[i];
    onfree((void*)rsv);
    return true;
  }

  if (m_real_curve_domain[1] <= domain[0] || m_real_curve_domain[0] >= domain[1]){
    onfree((void*)rsv);
    return false;
  }

  int i=0;
  int sc = 0;
  d[0] = m_real_curve_domain[0];
  
  while (i<= rsc && rsv[i] <= d[0]) i++;
  while (i <= rsc && rsv[i] < m_real_curve_domain[1]){
    sc++;
    d[sc] = rsv[i];
    i++;
  }
  sc++;
  d[sc] = m_real_curve_domain[1];

  onfree((void*)rsv);

  if (m_bReversed || m_real_curve_domain != m_this_domain){
    double x;
    int j;
    for ( i = 0; i <= sc; i++ )
    {
      x = m_real_curve_domain.NormalizedParameterAt(d[i]);
      d[i] = x;
    }
    if ( m_bReversed )
    {
      for ( i = 0, j = sc; i <= j; i++, j-- ) 
      {
        x = d[i];
        d[i] = 1.0-d[j];
        d[j] = 1.0-x;
      }
    }
    for ( i = 0; i <= sc; i++ )
    {
      d[i] = m_this_domain.ParameterAt(d[i]);
    }
  }

  return true;
}

int ON_CurveProxy::Degree() const
{
  return m_real_curve ? m_real_curve->Degree() : 0;
}

ON_BOOL32 
ON_CurveProxy::GetParameterTolerance(
         double t,  // t = parameter in domain
         double* tminus, // tminus
         double* tplus   // tplus
         ) const
{
  ON_BOOL32 rc = ( m_real_curve ) 
          ? m_real_curve->GetParameterTolerance( RealCurveParameter(t),tminus,tplus) 
          : false;
  if (rc)
  {
    if ( tminus )
      *tminus = ThisCurveParameter(*tminus);
    if ( tplus )
      *tplus = ThisCurveParameter(*tplus);
  }
  return rc;
}


ON_BOOL32
ON_CurveProxy::IsLinear( // true if curve locus is a line segment
      double tolerance // tolerance to use when checking linearity
      ) const
{
  // 12 December 2003 Dale Lear - fixed bug so this works
  //     when a proxy is restricted to using a linear portion
  //     of a non-linear real curve.
  bool rc = false;
  if ( 0 != m_real_curve )
  {
    ON_Interval cdom = m_real_curve->Domain();
    if ( cdom == m_real_curve_domain )
    {
      rc = m_real_curve->IsLinear(tolerance) ? true : false;
    }
    else
    {
      // The ON_CurveProxy::DuplicateCurve() scope is critical
      // because there are situation where people derive a 
      // class from ON_CurveProxy, and override the virtual
      // DuplicateCurve().  In this situation I rely on getting
      // the result returned by ON_CurveProxy::DuplicateCurve().
      ON_Curve* temp_curve = ON_CurveProxy::DuplicateCurve();
      if ( 0 != temp_curve )
      {
        rc = temp_curve->IsLinear(tolerance) ? true : false;
        delete temp_curve;
      }
    }
  }
  return rc;
}

int ON_CurveProxy::IsPolyline(
      ON_SimpleArray<ON_3dPoint>* pline_points,
      ON_SimpleArray<double>* pline_t
      ) const
{
  ON_SimpleArray<double> tmp_t;

  if ( pline_points )
    pline_points->SetCount(0);
  if ( pline_t )
    pline_t->SetCount(0);
  if ( !m_real_curve_domain.IsIncreasing() )
    return 0;
  if ( !m_real_curve )
    return 0;
  const ON_Interval cdom = m_real_curve->Domain();
  if ( !cdom.Includes(m_real_curve_domain) )
    return 0;

  // See if the "real" curve is a polyline
  int rc = 0;
  if ( m_real_curve_domain == cdom )
  {
    // proxy uses entire curve
    rc = m_real_curve->IsPolyline(pline_points,pline_t);
    if ( rc < 2 )
      rc = 0;

    if ( pline_points && pline_points->Count() != rc)
    {
      // The pline_points info is bogus, clear everything and
      // return 0.
      pline_points->SetCount(0);
      if ( pline_t )
        pline_t->SetCount(0);
      rc = 0;
    }

    if ( pline_t && pline_t->Count() != rc)
    {
      // The pline_t info is bogus, clear everything and
      // return 0.
      pline_t->SetCount(0);
      if ( pline_points )
        pline_points->SetCount(0);
      rc = 0;
    }

    if ( rc )
    {
      if ( m_bReversed )
      {
        if ( pline_points )
          pline_points->Reverse();
        if ( pline_t )
          pline_t->Reverse();
      }

      if ( pline_points && IsClosed() && pline_points->Count() > 3 )
      {
        // 27 February 2003 Dale Lear
        //     If proxy curve says it's closed, then
        //     make sure end point of returned polyline
        //     is exactly equal to start point.
        *pline_points->Last() = *pline_points->First();
      }
      
      if ( pline_t && (m_bReversed || m_real_curve_domain != m_this_domain) )
      {
        int i;
        for ( i = 0; i < rc; i++ )
        {
          (*pline_t)[i] = ThisCurveParameter( (*pline_t)[i] );
        }
      }
    }
  }
  else
  {
    // 12 December 2003 Dale Lear
    //   We have to extract a sub curve for the test, because
    //   the region in question may be a polyline and the unused
    //   portion may not be a polyline.  This tends to happen
    //   when the "real" curve is a polycurve that contains 
    //   a polyline and the polycurve has been parametrically
    //   trimmed during a brep operation (like a boolean).
    //
    // The ON_CurveProxy::DuplicateCurve() scope is critical
    // because there are situation where people derive a 
    // class from ON_CurveProxy, and override the virtual
    // DuplicateCurve().  In this situation I rely on getting
    // the result returned by ON_CurveProxy::DuplicateCurve().
    ON_Curve* temp_curve = ON_CurveProxy::DuplicateCurve();
    if ( temp_curve )
    {
      rc = temp_curve->IsPolyline(pline_points,pline_t);
      delete temp_curve;
    }
  }

  return rc;
}



ON_BOOL32
ON_CurveProxy::IsArc( // true if curve locus in an arc or circle
      const ON_Plane* plane, // if not NULL, test is performed in this plane
      ON_Arc* arc,         // if not NULL and true is returned, then arc
                              // arc parameters are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = false;
  const ON_Interval cdom = m_real_curve->Domain();
  if ( cdom == m_real_curve_domain )
  {
    rc = m_real_curve->IsArc(plane,arc,tolerance) ? true : false;
    if ( rc && arc  && m_bReversed )
      arc->Reverse();
  }
  else
  {
    // The ON_CurveProxy::DuplicateCurve() scope is critical
    // because there are situation where people derive a 
    // class from ON_CurveProxy, and override the virtual
    // DuplicateCurve().  In this situation I rely on getting
    // the result returned by ON_CurveProxy::DuplicateCurve().
    ON_Curve* temp_curve = ON_CurveProxy::DuplicateCurve();
    if ( 0 != temp_curve )
    {
      rc = temp_curve->IsArc(plane,arc,tolerance) ? true : false;
      delete temp_curve;
    }
  }
  return rc;
}

ON_BOOL32
ON_CurveProxy::IsPlanar(
      ON_Plane* plane, // if not NULL and true is returned, then plane parameters
                         // are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  return ( m_real_curve ) ? m_real_curve->IsPlanar(plane,tolerance) : false;
}

ON_BOOL32
ON_CurveProxy::IsInPlane(
      const ON_Plane& plane, // plane to test
      double tolerance // tolerance to use when checking linearity
      ) const
{
  return ( m_real_curve ) ? m_real_curve->IsInPlane(plane,tolerance) : false;
}

ON_BOOL32 
ON_CurveProxy::IsClosed() const
{
  ON_BOOL32 rc = false;
  if ( m_real_curve && m_real_curve->Domain() == m_real_curve_domain )
  {
    rc = m_real_curve->IsClosed();
  }
  return rc;
}

ON_BOOL32 
ON_CurveProxy::IsPeriodic() const
{
  ON_BOOL32 rc = false;
  if ( m_real_curve && m_real_curve->Domain() == m_real_curve_domain )
  {
    rc = m_real_curve->IsPeriodic();
  }
  return rc;
}

bool ON_CurveProxy::GetNextDiscontinuity( 
                ON::continuity c,
                double t0,
                double t1,
                double* t,
                int* hint,
                int* dtype,
                double cos_angle_tolerance,
                double curvature_tolerance
                ) const
{
  bool rc = false;
  if ( 0 != dtype )
    *dtype = 0;

  if ( 0 != m_real_curve )
  {
    double s;

    // convert to "real" curve parameters
    double s0 = RealCurveParameter( t0 );
    double s1 = RealCurveParameter( t1 );

    // 21 October 2005 Dale Lear:
    //
    // NOTE: If m_bReversed is true, then RealCurveParameter
    //       will reverse the direction of the search.
    //       The commented out code below just messed things up.
    //
    //if ( m_bReversed )
    //{
    //  // NOTE: GetNextDiscontinuity search begins at 
    //  //       "t0" and goes towards "t1" so it is ok if s0 > s1.
    //  s = s0; s0 = s1; s1 = s;
    //}

    ON::continuity parametric_c = ON::ParametricContinuity(c);

    int realcrv_dtype = 0;
    bool realcrv_rc = m_real_curve->GetNextDiscontinuity(parametric_c,s0,s1,&s,hint,&realcrv_dtype,cos_angle_tolerance,curvature_tolerance);

    if ( realcrv_rc ) 
    {
      double thiscrv_t = ThisCurveParameter(s);
      if ( !(t0 < thiscrv_t && thiscrv_t < t1) && !(t1 < thiscrv_t && thiscrv_t < t0) )
      {
        realcrv_rc = false;
        realcrv_dtype = 0;
        // Sometimes proxy domain adjustments kill all the precision.
        // To avoid infinite loops, it is critical that *t != t0
        double s2 = ON_SQRT_EPSILON*s1 + (1.0 - ON_SQRT_EPSILON)*s0;
        if ( (s0 < s2 && s2 < s1) || (s1 < s2 && s2 < s0) )
        {
          realcrv_rc = m_real_curve->GetNextDiscontinuity(parametric_c,s2,s1,&s,hint,&realcrv_dtype,cos_angle_tolerance,curvature_tolerance);
          if ( realcrv_rc )
            thiscrv_t = ThisCurveParameter(s);
        }
      }
      if ( realcrv_rc )
      {
        if ( (t0 < thiscrv_t && thiscrv_t < t1) || (t1 < thiscrv_t && thiscrv_t < t0) )
        {
          *t = thiscrv_t;
          if ( dtype )
            *dtype = realcrv_dtype;
          rc = true;
        }
      }
    }

    if ( !rc && parametric_c != c )
    {
      // 20 March 2003 Dale Lear:
      //   Let base class test decide locus continuity questions at ends 
      rc = ON_Curve::GetNextDiscontinuity( c, t0, t1, t, hint, dtype, cos_angle_tolerance, curvature_tolerance );
    }
  }

  return rc;
}


bool ON_CurveProxy::IsContinuous(
    ON::continuity desired_continuity,
    double t, 
    int* hint, // default = NULL,
    double point_tolerance, // default=ON_ZERO_TOLERANCE
    double d1_tolerance, // default==ON_ZERO_TOLERANCE
    double d2_tolerance, // default==ON_ZERO_TOLERANCE
    double cos_angle_tolerance, // default==ON_DEFAULT_ANGLE_TOLERANCE_COSINE
    double curvature_tolerance  // default==ON_SQRT_EPSILON
    ) const
{
  bool rc = true;
  if ( m_real_curve )
  {
    if ( m_real_curve_domain != m_real_curve->Domain() )
    {
      // 20 March 2003 Dale Lear
      //      Added this code to correctly handle the new locus
      //      flavors of ON::continuity.
      switch(desired_continuity)
      {
      case ON::unknown_continuity:
      case ON::C0_continuous:
      case ON::C1_continuous:
      case ON::C2_continuous:
      case ON::G1_continuous:
      case ON::G2_continuous:
      case ON::Cinfinity_continuous:
      case ON::Gsmooth_continuous:
        break;

      case ON::C0_locus_continuous:
      case ON::C1_locus_continuous:
      case ON::C2_locus_continuous:
      case ON::G1_locus_continuous:
      case ON::G2_locus_continuous:
        if ( t >= Domain()[1] )
        {
          // Since the proxy curve is using a subset of the real curve,
          // the proxy can't be closed.  So if the query parameter
          // is >= domain max, the curve cannot be locus continuous.
          rc = false;
        }
        else
        {
          // otherwise we want the answer for a non-locus test
          desired_continuity = ON::ParametricContinuity(desired_continuity);
        }
        break;
      }
    }
    if (rc)
      rc = m_real_curve->IsContinuous( desired_continuity, 
                           RealCurveParameter(t), hint, 
                           point_tolerance, d1_tolerance, d2_tolerance, 
                           cos_angle_tolerance, curvature_tolerance );
  }
  return rc;
}


ON_BOOL32
ON_CurveProxy::Reverse()
{
  if ( m_this_domain.IsIncreasing() )
  {
    m_bReversed = (m_bReversed) ? false : true;
	  DestroyCurveTree();
    m_this_domain.Reverse();
  }
  return true;
}

ON_BOOL32 
ON_CurveProxy::Evaluate( // returns false if unable to evaluate
       double t,       // evaluation parameter
       int der_count,  // number of derivatives (>=0)
       int v_stride,   // v[] array stride (>=Dimension())
       double* v,      // v[] array of length stride*(ndir+1)
       int side,       // optional - determines which side to evaluate from
                       //         0 = default
                       //      <  0 to evaluate from below, 
                       //      >  0 to evaluate from above
       int* hint       // optional - evaluation hint (int) used to speed
                       //            repeated evaluations
       ) const
{
  // When the proxy domain is a proper subdomain of the 
  // real curve and we are evaluating at the end, we
  // need to be certain we are getting the values
  // from the active part of the curve.
  double normt = m_this_domain.NormalizedParameterAt(t);
  if( fabs( normt )<ON_ZERO_TOLERANCE)
    side = (abs(side) <= 1) ? 1 : 2;
  else if( fabs(1.0 - normt)<ON_ZERO_TOLERANCE)
    side = (abs(side) <= 1) ? -1 : -2;
  
  if ( 0 != side )
  {
    if ( m_bReversed )
    {
      side = -side;
    }
    if (m_bReversed || m_real_curve_domain != m_this_domain )
    {
      // 9 November 2010 Dale Lear - ON_TuneupEvaluationParameter fix
      //  If we have to adjust the evaluation parameter for the 
      //  real curve and the evaluation side was specified, then
      //  set side to +2 or -2 to trigger the use of
      //  ON_TuneupEvaluationParameter() when it matters.
      if ( -1 == side )
        side = -2;
      else if ( 1 == side )
        side = 2;
    }
  }

  double r = RealCurveParameter(t);
  ON_BOOL32 rc = ( m_real_curve ) 
          ? m_real_curve->Evaluate( r,der_count,v_stride,v,side,hint) 
          : false;
  if ( rc && m_bReversed ) 
  {
    // negate odd derivatives
    const int dim = m_real_curve->Dimension();
    int di, i;
    for ( di = 1; di <= der_count; di+=2 ) 
    {
      v += v_stride;
      for ( i = 0; i < dim; i++ ) 
      {
        v[i] = -v[i];
      }
      v += v_stride;
    }
  }
  return rc;
}

ON_BOOL32 ON_CurveProxy::Trim(
  const ON_Interval& domain
  )
{
  bool rc = false;
  if ( m_this_domain.IsIncreasing() && m_real_curve_domain.IsIncreasing() )
  {
    ON_Interval trim_dom = m_this_domain;
    trim_dom.Intersection(domain);
    if ( trim_dom.IsIncreasing() )
    {
      ON_Interval real_dom = RealCurveInterval( &trim_dom );
      if ( real_dom.IsIncreasing() )
      {
        DestroyCurveTree();
        m_real_curve_domain = real_dom;
        m_this_domain = trim_dom;
        rc = true;
      }
    }
  }
  return rc;
}


// override of virtual ON_Curve::Split
ON_BOOL32 ON_CurveProxy::Split(
    double t,
    ON_Curve*& left_side,
    ON_Curve*& right_side
  ) const
{
  ON_BOOL32 rc = false;
  if ( m_this_domain.IsIncreasing() && m_real_curve_domain.IsIncreasing() && m_this_domain.Includes(t,true) )
  {
    double crv_t = RealCurveParameter(t);
    if ( m_real_curve_domain.Includes(crv_t,true) )
    {
      ON_CurveProxy* left_proxy = 0;
      ON_CurveProxy* right_proxy = 0;
      if ( left_side )
      {
        left_proxy = ON_CurveProxy::Cast(left_side);
        if ( !left_proxy )
          return false;
      }
      if ( right_side )
      {
        right_proxy = ON_CurveProxy::Cast(right_side);
        if ( !right_proxy )
          return false;
        if ( right_side == left_side )
          return false;
      }

      bool bRev = m_bReversed;

      ON_Interval left_real_dom, right_real_dom;
      if ( bRev )
      {
        left_real_dom.Set(crv_t,m_real_curve_domain[1]);
        right_real_dom.Set(m_real_curve_domain[0],crv_t);
      }
      else
      {
        left_real_dom.Set(m_real_curve_domain[0],crv_t);
        right_real_dom.Set(crv_t,m_real_curve_domain[1]);
      }

      ON_Interval left_this_dom(m_this_domain[0],t);
      ON_Interval right_this_dom(t,m_this_domain[1]);

      if (    left_real_dom.IsIncreasing() 
           && right_real_dom.IsIncreasing()
           && left_this_dom.IsIncreasing()
           && right_this_dom.IsIncreasing()
           )
      {
        // note well that left_proxy or right_proxy might also be this
        const ON_Curve* real_crv = m_real_curve;
        if ( real_crv )
        {
          ON_Interval d = real_crv->Domain();
          if ( !d.Includes(left_real_dom) )
            return false;
          if ( !d.Includes(right_real_dom) )
            return false;
        }

        if ( !left_proxy )
          left_proxy = new ON_CurveProxy();
        if ( !right_proxy )
          right_proxy = new ON_CurveProxy();

        left_proxy->SetProxyCurve( real_crv, left_real_dom );
        right_proxy->SetProxyCurve( real_crv, right_real_dom );

        if ( bRev )
        {
          left_proxy->Reverse();
          right_proxy->Reverse();
        }

        left_proxy->SetDomain(left_this_dom[0],left_this_dom[1]);
        right_proxy->SetDomain(right_this_dom[0],right_this_dom[1]);

        if (!left_side) left_side = left_proxy;
        if (!right_side) right_side = right_proxy;

        rc = true;
      }
    }
  }
  return rc;
}

int 
ON_CurveProxy::GetNurbForm( // returns 0: unable to create NURBS representation
                 //            with desired accuracy.
                 //         1: success - returned NURBS parameterization
                 //            matches the curve's to wthe desired accuracy
                 //         2: success - returned NURBS point locus matches
                 //            the curve's to the desired accuracy but, on
                 //            the interior of the curve's domain, the 
                 //            curve's parameterization and the NURBS
                 //            parameterization may not match to the 
                 //            desired accuracy.
      ON_NurbsCurve& nurbs,
      double tolerance,  // (>=0)
      const ON_Interval* sub_domain  // OPTIONAL subdomain of ON::ProxyCurve::Domain()
      ) const
{
  ON_BOOL32 rc = false;
  if ( m_real_curve ) 
  {
    ON_Interval scratch_domain = RealCurveInterval( sub_domain );
    rc = m_real_curve->GetNurbForm(nurbs,tolerance,&scratch_domain);
    if ( rc )
    {
      if ( m_bReversed )
        nurbs.Reverse();
      ON_Interval d = m_this_domain;
      if ( sub_domain )
        d.Intersection( *sub_domain );
      nurbs.SetDomain( d[0], d[1] );

      if ( nurbs.m_dim <= 3 && nurbs.m_dim >= 1 )
      {
        double t0 = Domain()[0];
        double t1 = Domain()[1];
        if ( 0 != sub_domain )
        {
          if ( t0 < sub_domain->Min() )
            t0 = sub_domain->Min();
          if ( sub_domain->Max() < t1 )
            t1 = sub_domain->Max();
        }
        // set ends of NURBS curve to be exactly on ends of proxy curve
        ON_3dPoint P0 = PointAt(t0);
        ON_3dPoint P1 = PointAt(t1);
        ON_3dPoint N0 = nurbs.PointAtStart();
        ON_3dPoint N1 = nurbs.PointAtEnd();
				
				// 22 September 2003, GBA.  The end tuning code below should only  be applied
				//					to clamped nurbs curves.  In particular it should not be used on
				//					periodic nurbs curves.  Fixes TRR#11502.
				ON_BOOL32 clamped = nurbs.IsClamped(2);
        if ( clamped && (P0 != N0 || P1 != N1) )
        {
          if ( 0==nurbs.m_is_rat )
          {
            nurbs.SetCV(0,P0);
            nurbs.SetCV(nurbs.m_cv_count-1,P1);
          }
          else
          {
            ON_4dPoint H0, H1;
            H0 = P0;
            H0.w = nurbs.Weight(0);
            H0.x *= H0.w;
            H0.y *= H0.w;
            H0.z *= H0.w;
            nurbs.SetCV(0,H0);

            H1 = P1;
            H1.w = nurbs.Weight(nurbs.m_cv_count-1);
            H1.x *= H1.w;
            H1.y *= H1.w;
            H1.z *= H1.w;
            nurbs.SetCV(nurbs.m_cv_count-1,H1);
          }
        }
      }
    }
  }
  return rc;
}

int 
ON_CurveProxy::HasNurbForm( // returns 0: unable to create NURBS representation
                 //            with desired accuracy.
                 //         1: success - returned NURBS parameterization
                 //            matches the curve's to wthe desired accuracy
                 //         2: success - returned NURBS point locus matches
                 //            the curve's to the desired accuracy but, on
                 //            the interior of the curve's domain, the 
                 //            curve's parameterization and the NURBS
                 //            parameterization may not match to the 
                 //            desired accuracy.
      ) const

{
  if (!m_real_curve)
    return 0;
  return m_real_curve->HasNurbForm();
}

ON_BOOL32 ON_CurveProxy::GetCurveParameterFromNurbFormParameter(
      double nurbs_t,
      double* curve_t
      ) const
{
  ON_BOOL32 rc = false;
  if ( m_real_curve ) 
  {
    // 18 June 2003 Dale Lear and Chuck
    //     Fixing joining bug in STEP TEST 2 caused by error
    //     in converting NURBS parameter to arc parameter.
    const ON_Curve* real_crv = m_real_curve;

    ON_Curve* tmp_real_crv = 0;
    if ( m_real_curve_domain != m_real_curve->Domain() )
    {
      const ON_ArcCurve* arc_curve = ON_ArcCurve::Cast(m_real_curve);
      if ( 0 != arc_curve )
      {
        tmp_real_crv = arc_curve->DuplicateCurve();
        if ( 0 != tmp_real_crv )
        {
          if ( tmp_real_crv->Trim(m_real_curve_domain) )
          {
            real_crv = tmp_real_crv;
          }
        }
      }
    }

    rc = real_crv->GetCurveParameterFromNurbFormParameter( RealCurveParameter(nurbs_t),curve_t);
    if ( rc )
      *curve_t = ThisCurveParameter(*curve_t);

    if ( 0 != tmp_real_crv )
      delete tmp_real_crv;
  }
  return rc;
}

ON_BOOL32 ON_CurveProxy::GetNurbFormParameterFromCurveParameter(
      double curve_t,
      double* nurbs_t
      ) const
{
  ON_BOOL32 rc = false;
  if ( m_real_curve ) 
  {
    // 18 June 2003 Dale Lear and Chuck
    //     Fixing joining bug in STEP TEST 2 caused by error
    //     in converting NURBS parameter to arc parameter.
    const ON_Curve* real_crv = m_real_curve;

    ON_Curve* tmp_real_crv = 0;
    if ( m_real_curve_domain != m_real_curve->Domain() )
    {
      const ON_ArcCurve* arc_curve = ON_ArcCurve::Cast(m_real_curve);
      if ( 0 != arc_curve )
      {
        tmp_real_crv = arc_curve->DuplicateCurve();
        if ( 0 != tmp_real_crv )
        {
          if ( tmp_real_crv->Trim(m_real_curve_domain) )
          {
            real_crv = tmp_real_crv;
          }
        }
      }
    }

    rc = real_crv->GetNurbFormParameterFromCurveParameter( RealCurveParameter(curve_t),nurbs_t);
    if ( rc )
      *nurbs_t = ThisCurveParameter(*nurbs_t);

    if ( 0 != tmp_real_crv )
      delete tmp_real_crv;
  }
  return rc;
}

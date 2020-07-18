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

ON_OBJECT_IMPLEMENT(ON_PolylineCurve,ON_Curve,"4ED7D4E6-E947-11d3-BFE5-0010830122F0");

ON_PolylineCurve::ON_PolylineCurve()
{
  m_dim = 3;
}

ON_PolylineCurve::ON_PolylineCurve( const ON_PolylineCurve& L ) : ON_Curve(L)
{
  *this = L;
}

ON_PolylineCurve::ON_PolylineCurve( const ON_3dPointArray& L )
{
  *this = L;
}

ON_PolylineCurve::~ON_PolylineCurve()
{
}

unsigned int ON_PolylineCurve::SizeOf() const
{
  unsigned int sz = ON_Curve::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Curve));
  sz += m_pline.SizeOfArray();
  sz += m_t.SizeOfArray();
  return sz;
}

ON__UINT32 ON_PolylineCurve::DataCRC(ON__UINT32 current_remainder) const
{
  current_remainder = m_pline.DataCRC(current_remainder);
  current_remainder = m_t.DataCRC(current_remainder);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_dim),&m_dim);
  return current_remainder;
}

void ON_PolylineCurve::EmergencyDestroy()
{
  m_pline.EmergencyDestroy();
  m_t.EmergencyDestroy();
}

ON_PolylineCurve& ON_PolylineCurve::operator=( const ON_PolylineCurve& src )
{
  if ( this != &src ) {
    ON_Curve::operator=(src);
    m_pline = src.m_pline;
    m_t     = src.m_t;
    m_dim   = src.m_dim;
  }
  return *this;
}

ON_PolylineCurve& ON_PolylineCurve::operator=( const ON_3dPointArray& src )
{
  m_pline = src;
  m_dim   = 3;
  const int count = src.Count();
  m_t.Reserve(count);
  m_t.SetCount(count);
  int i;
  for (i = 0; i < count; i++) {
    m_t[i] = (double)i;
  }
  return *this;
}

int ON_PolylineCurve::Dimension() const
{
  return m_dim;
}

ON_BOOL32 
ON_PolylineCurve::GetBBox( // returns true if successful
         double* boxmin,    // minimum
         double* boxmax,    // maximum
         ON_BOOL32 bGrowBox
         ) const
{
  return ON_GetPointListBoundingBox( m_dim, false, PointCount(), 3, m_pline[0], 
                        boxmin, boxmax, bGrowBox?true:false
                        );
}


ON_BOOL32
ON_PolylineCurve::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
	DestroyCurveTree();
  return m_pline.Transform( xform );
}



ON_BOOL32
ON_PolylineCurve::SwapCoordinates( int i, int j )
{
	DestroyCurveTree();
  return m_pline.SwapCoordinates(i,j);
}

ON_BOOL32 ON_PolylineCurve::IsValid( ON_TextLog* text_log ) const
{
  const int count = PointCount();
  if ( count >= 2 && count == m_t.Count() ) 
  {
    if ( !m_pline.IsValid() )
    {
      if ( 0 != text_log )
      {
        text_log->Print("PolylineCurve m_pline[] is not valid.\n");
      }
      return ON_IsNotValid();
    }
    int i;
    for ( i = 1; i < count; i++ ) 
    {
      if ( m_t[i] <= m_t[i-1] )
      {
        if ( 0 != text_log )
        {
          text_log->Print("PolylineCurve m_t[%d]=%g should be less than m_t[%d]=(%g).\n",
                           i-1,m_t[i-1],i,m_t[i]);
        }
        return ON_IsNotValid();
      }
    }

    if (m_dim < 2 || m_dim > 3 )
    {
      if (0 != text_log )
        text_log->Print("PolylineCurve m_dim = %d (should be 2 or 3).\n",m_dim);
      return ON_IsNotValid();
    }
  }
  else if ( 0 != text_log )
  {
    if ( count < 2 )
      text_log->Print("PolylineCurve has %d points (should be >= 2)\n",count);
    else
      text_log->Print("PolylineCurve m_t.Count() = %d and PointCount() = %d (should be equal)\n",
                      m_t.Count(),count);
    return ON_IsNotValid();
  }

  return true;
}

void ON_PolylineCurve::Dump( ON_TextLog& dump ) const
{
  ON_Interval d = Domain();
  dump.Print( "ON_PolylineCurve:  domain = [%g,%g]\n",d[0],d[1]);
  for ( int i = 0; i < PointCount(); i++ ) {
    dump.Print( "  point[%2d] = ",i);
    dump.Print( m_pline[i] );
    dump.Print( ", %g\n",m_t[i]);
  }
}

ON_BOOL32 ON_PolylineCurve::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if (rc) {
    if (rc) rc = file.WriteArray( m_pline );
    if (rc) rc = file.WriteArray( m_t );
    if (rc) rc = file.WriteInt(m_dim);
  }
  return rc;
}

ON_BOOL32 ON_PolylineCurve::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1) {
    // common to all 1.x versions
    if (rc) rc = file.ReadArray( m_pline );
    if (rc) rc = file.ReadArray( m_t );
    if (rc) rc = file.ReadInt(&m_dim);
  }
  return rc;
}

ON_Interval ON_PolylineCurve::Domain() const
{
  ON_Interval d;
  //ON_BOOL32 rc = false;
  const int count = PointCount();
  if ( count >= 2 && m_t[0] < m_t[count-1] ) {
    d.Set(m_t[0],m_t[count-1]);
  }
  return d;
}

ON_BOOL32 ON_PolylineCurve::SetDomain( double t0, double t1 )
{
  ON_BOOL32 rc = false;
  const int count = m_t.Count()-1;
  if ( count >= 1 )
  {
    if ( t0 == m_t[0] && t1 == m_t[count] )
      rc = true;
    else if ( t0 < t1 ) 
    {
      const ON_Interval old_domain = Domain();
      const ON_Interval new_domain(t0,t1);
      m_t[0] = t0;
      m_t[count] = t1;
      for ( int i = 1; i < count; i++ )
      {
        m_t[i] = new_domain.ParameterAt( old_domain.NormalizedParameterAt(m_t[i]) );
      }      
			rc=true;
    }
  }
	DestroyCurveTree();
  return rc;  
}

bool ON_PolylineCurve::ChangeDimension( int desired_dimension )
{
  bool rc = (desired_dimension>=2 && desired_dimension<=3);

  if ( rc && m_dim != desired_dimension )
  {
  	DestroyCurveTree();
    int i, count = m_pline.Count();
    if ( 2 == desired_dimension )
    {
      if ( count > 0 )
      {
        // 7 April 2003 Dale Lear:
        //   If x coord of first point is set, then
        //   zero all z coords.  
        if ( ON_UNSET_VALUE != m_pline[0].x )
        {
          for ( i = 0; i < count; i++ )
            m_pline[i].z = 0.0;
        }
      }
      m_dim = 2;
    }
    else 
    {
      if ( count > 0 )
      {
        // 7 April 2003 Dale Lear:
        //   If first point x coord is set and z is unset, then
        //   zero all z coords.  
        if ( ON_UNSET_VALUE != m_pline[0].x && ON_UNSET_VALUE == m_pline[0].z )
        {
          for ( i = 0; i < count; i++ )
            m_pline[i].z = 0.0;
        }
      }
      m_dim = 3;
    }
  }

  return rc;
}


ON_BOOL32 ON_PolylineCurve::ChangeClosedCurveSeam( double t )
{
  const ON_Interval old_dom = Domain();
  ON_BOOL32 rc = IsClosed();
  if ( rc )
  {
    double k = t;
    if ( !old_dom.Includes(t) )
    {
      double s = old_dom.NormalizedParameterAt(t);
      s = fmod(s,1.0);
      if ( s < 0.0 )
        s += 1.0;
      k = old_dom.ParameterAt(s);
    }
    if ( old_dom.Includes(k,true) )
    {
      int old_count = PointCount();
      int i = ON_NurbsSpanIndex(2,old_count,m_t.Array(),k,0,0);
      if ( k < m_t[i] )
        return false;
      if ( k >= m_t[i+1] )
        return false;
      int new_count = (k==m_t[i]) ? old_count : old_count+1;
      ON_SimpleArray<ON_3dPoint> new_pt(new_count);
      ON_SimpleArray<double> new_t(new_count);
      ON_3dPoint new_start = (k==m_t[i]) ? m_pline[i] : PointAt(k);
      new_pt.Append( new_start );
      new_t.Append(k);
      int n = old_count-i-1;
      new_pt.Append( n, m_pline.Array() + i+1 );
      new_t.Append( n, m_t.Array() + i+1 );
      
      int j = new_t.Count();

      n = new_count-old_count+i-1;
      new_pt.Append( n, m_pline.Array() + 1 );
      new_t.Append(  n, m_t.Array() + 1 );

      new_pt.Append( new_start );
      new_t.Append(k);

      double d = old_dom.Length();
      while ( j  < new_t.Count() )
      {
        new_t[j] += d;
        j++;
      }

      m_pline = new_pt;
      m_t = new_t;
    }
    else
    {
      // k already at start end of this curve
      rc = true;
    }
    if ( rc )
      SetDomain( t, t + old_dom.Length() );
  }
  return rc;
}

int ON_PolylineCurve::SpanCount() const
{
  return m_pline.SegmentCount();
}

ON_BOOL32 ON_PolylineCurve::GetSpanVector( // span "knots" 
       double* s // array of length SpanCount() + 1 
       ) const
{
  ON_BOOL32 rc = false;
  const int count = PointCount();
  if ( count >= 1 ) 
  {
    memcpy( s, m_t.Array(), count*sizeof(*s) );
    rc = true;
  }
  return rc;
}

int ON_PolylineCurve::Degree() const
{
  return 1;
}


ON_BOOL32
ON_PolylineCurve::IsLinear( // true if curve locus is a line segment
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = false;
  ON_NurbsCurve nurbs_curve;
  nurbs_curve.m_dim = m_dim;
  nurbs_curve.m_is_rat = 0;
  nurbs_curve.m_order = 2;
  nurbs_curve.m_cv_count = m_pline.Count();
  if ( nurbs_curve.m_cv_count >= 2 )
  {
    nurbs_curve.m_cv = const_cast<double*>(&m_pline[0].x);
    nurbs_curve.m_cv_stride = (int)(&m_pline[1].x - nurbs_curve.m_cv); // the int converts 64 bit std::size_t
    nurbs_curve.m_knot = const_cast<double*>(m_t.Array());
    // using ptr to make sure we go through vtable
    const ON_Curve* ptr = &nurbs_curve;
    rc = ptr->IsLinear(tolerance);
    nurbs_curve.m_cv = 0;
    nurbs_curve.m_knot = 0;
  }
  return rc;
}

int ON_PolylineCurve::IsPolyline(
      ON_SimpleArray<ON_3dPoint>* pline_points,
      ON_SimpleArray<double>* pline_t
      ) const
{
  if ( pline_points )
    pline_points->SetCount(0);
  if ( pline_t )
    pline_t->SetCount(0);
  int rc = this->PointCount();
  if ( rc >= 2 )
  {
    if ( pline_points )
      pline_points->operator=(m_pline);
    if ( pline_t )
      pline_t->operator=(m_t);
  }
  else
    rc = 0;
  return rc;
}

ON_BOOL32
ON_PolylineCurve::IsArc( // true if curve locus in an arc or circle
      const ON_Plane*, // if not NULL, test is performed in this plane
      ON_Arc*,         // if not NULL and true is returned, then arc
                              // arc parameters are filled in
      double // tolerance to use when checking linearity
      ) const
{
  return false;
}


ON_BOOL32
ON_PolylineCurve::IsPlanar(
      ON_Plane* plane, // if not NULL and true is returned, then plane parameters
                         // are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = false;
  ON_NurbsCurve nurbs_curve;
  nurbs_curve.m_dim = m_dim;
  nurbs_curve.m_is_rat = 0;
  nurbs_curve.m_order = 2;
  nurbs_curve.m_cv_count = m_pline.Count();
  if ( nurbs_curve.m_cv_count >= 2 )
  {
    if (m_dim == 2 )
    {
      rc = ON_Curve::IsPlanar(plane,tolerance);
    }
    else
    {
      nurbs_curve.m_cv = const_cast<double*>(&m_pline[0].x);
      nurbs_curve.m_cv_stride = (int)(&m_pline[1].x - nurbs_curve.m_cv); // the (int) converts 64 bit std::size_t
      nurbs_curve.m_knot = const_cast<double*>(m_t.Array());
      // using ptr to make sure we go through vtable
      const ON_Curve* ptr = &nurbs_curve;
      rc = ptr->IsPlanar(plane,tolerance);
      nurbs_curve.m_cv = 0;
      nurbs_curve.m_knot = 0;
    }
  }
  return rc;
}

ON_BOOL32
ON_PolylineCurve::IsInPlane(
      const ON_Plane& plane, // plane to test
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = false;
  ON_NurbsCurve nurbs_curve;
  nurbs_curve.m_dim = m_dim;
  nurbs_curve.m_is_rat = 0;
  nurbs_curve.m_order = 2;
  nurbs_curve.m_cv_count = m_pline.Count();
  if ( nurbs_curve.m_cv_count >= 2 )
  {
    nurbs_curve.m_cv = const_cast<double*>(&m_pline[0].x);
    nurbs_curve.m_cv_stride = (int)(&m_pline[1].x - nurbs_curve.m_cv);
    nurbs_curve.m_knot = const_cast<double*>(m_t.Array());
    rc = nurbs_curve.IsInPlane(plane,tolerance);
    nurbs_curve.m_cv = 0;
    nurbs_curve.m_knot = 0;
  }
  return rc;
}

ON_BOOL32 
ON_PolylineCurve::IsClosed() const
{
  return m_pline.IsClosed(0.0);
}

ON_BOOL32 
ON_PolylineCurve::IsPeriodic() const
{
  return false;
}

bool ON_PolylineCurve::GetNextDiscontinuity( 
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
  
  const int segment_count = m_pline.SegmentCount();

  if ( segment_count > 0 && t0 != t1 )
  {
    ON_Interval domain = Domain();

    if ( t0 < t1 )
    {
      if ( t0 < domain[0] )
        t0 = domain[0];
      if ( t1 > domain[1] )
        t1 = domain[1];
      if ( t0 >= t1 )
        return false;
    }
    else if ( t0 > t1 )
    {
      if ( t1 < domain[0] )
        t1 = domain[0];
      if ( t0 > domain[1] )
        t0 = domain[1];
      if ( t1 >= t0 )
        return false;
    }

    if ( t0 != t1 )
    {
      ON_3dPoint Pm, Pp;
      ON_3dVector D1m, D1p, Tm, Tp;

      if ( dtype )
        *dtype = 0;
      c = ON::PolylineContinuity(c);
      ON::continuity parametric_c = ON::ParametricContinuity(c);
      if ( segment_count >= 2 && parametric_c != ON::C0_continuous ) 
      {
        int i = 0;
        int delta_i = 1;
        double s0 = t0;
        double s1 = t1;
        i = ON_NurbsSpanIndex(2,PointCount(),m_t,t0,0,(hint)?*hint:0);
        double segtol = (fabs(m_t[i]) + fabs(m_t[i+1]) + fabs(m_t[i+1]-m_t[i]))*ON_SQRT_EPSILON;
        if ( t0 < t1 )
        {
          if ( t0 < m_t[i+1] && t1 > m_t[i+1] && (m_t[i+1]-t0) <= segtol && i+1 < PointCount() )
          {
            t0 = m_t[i+1];
            i = ON_NurbsSpanIndex(2,PointCount(),m_t,t0,0,(hint)?*hint:0);
          }
          if ( hint )
            *hint = i;
          i++; // start checking at first m_t[i] > t0
        }
        else if ( t0 > t1 )
        {
          // Check backwards (have to handle this case so 
          // ON_CurveProxy::GetNextDiscontinuity() works on
          // reversed proxy curves.
          if ( t0 > m_t[i] && t1 < m_t[i] && (t0-m_t[i]) <= segtol && i > 0 )
          {
            t0 = m_t[i];
            i = ON_NurbsSpanIndex(2,PointCount(),m_t,t0,0,(hint)?*hint:0);
          }
          if ( hint )
            *hint = i;
          if ( t0 == m_t[i] )
            i--;
          delta_i = -1;
          s0 = t1;
          s1 = t0;
        }
        for ( /*empty*/; !rc && 0 < i && i < segment_count && s0 < m_t[i] && m_t[i] < s1; i += delta_i )
        {
          Ev1Der(m_t[i], Pm, D1m, -1, hint );
          Ev1Der(m_t[i], Pp, D1p, +1, hint );
          if ( parametric_c == ON::C1_continuous || parametric_c == ON::C2_continuous )
          {
            if ( !(D1m-D1p).IsTiny(D1m.MaximumCoordinate()*ON_SQRT_EPSILON) )
              rc = true;
          }
          else if ( parametric_c == ON::G1_continuous || parametric_c == ON::G2_continuous || parametric_c == ON::Gsmooth_continuous )
          {
            Tm = D1m;
            Tp = D1p;
            Tm.Unitize();
            Tp.Unitize();
            if ( Tm*Tp < cos_angle_tolerance )
              rc = true;
          }
          if ( rc )
          {
            if ( dtype )
              *dtype = 1;
            if ( t )
              *t = m_t[i];
            break;
          }
        }
      }

      if ( !rc && segment_count > 0 && parametric_c != c )
      {
        // 20 March 2003 Dale Lear:
        //   Let base class test for locus continuities at start/end.
        rc = ON_Curve::GetNextDiscontinuity( c, t0, t1, t, hint, dtype, cos_angle_tolerance, curvature_tolerance );
      }
    }
  }

  return rc;
}


bool ON_PolylineCurve::IsContinuous(
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
  const int segment_count = m_pline.SegmentCount();

  if ( segment_count >= 1 )
  {
    bool bPerformTest = false;
    desired_continuity = ON::PolylineContinuity(desired_continuity);

    if ( t <= m_t[0] || t >= m_t[segment_count] )
    {
      // 20 March 2003 Dale Lear
      //     Consistently handles locus case and out of domain case.
      switch(desired_continuity)
      {
      case ON::C0_locus_continuous: 
      case ON::C1_locus_continuous: 
      case ON::G1_locus_continuous: 
        bPerformTest = true;
        break;
      default:
        // intentionally ignoring other ON::continuity enum values
        break;
      }
    }
    else
    {
      if ( segment_count >= 2 && desired_continuity != ON::C0_continuous ) 
      {
        int i = ON_NurbsSpanIndex(2,PointCount(),m_t,t,0,(hint)?*hint:0);
        
        {
          // 20 March 2003 Dale Lear:
          //     If t is very near interior m_t[] value, see if it
          //     should be set to that value.  A bit or two of 
          //     precision sometimes gets lost in proxy
          //     domain to real curve domain conversions on the interior
          //     of a curve domain.
          double segtol = (fabs(m_t[i]) + fabs(m_t[i+1]) + fabs(m_t[i+1]-m_t[i]))*ON_SQRT_EPSILON;
          if ( m_t[i]+segtol < m_t[i+1]-segtol )
          {
            if ( fabs(t-m_t[i]) <= segtol && i > 0 )
            {
              t = m_t[i];
            }
            else if ( fabs(t-m_t[i+1]) <= segtol && i+1 < PointCount() )
            {
              t = m_t[i+1];
              i = ON_NurbsSpanIndex(2,PointCount(),m_t,t,0,(hint)?*hint:0);
            }
          }
        }
        
        if ( hint )
          *hint = i;
        if ( i > 0 && i < segment_count && t == m_t[i] )
        {
          // "locus" and "parametric" tests are the same at this point.
          desired_continuity = ON::ParametricContinuity(desired_continuity);
          bPerformTest = true;
        }
      }
    }

    if ( bPerformTest )
    {
      // need to evaluate and test
      rc = ON_Curve::IsContinuous( desired_continuity, t, hint,
                                   point_tolerance, d1_tolerance, d2_tolerance,
                                   cos_angle_tolerance, curvature_tolerance );
    }
  }

  return rc;
}

ON_BOOL32
ON_PolylineCurve::Reverse()
{
  ON_BOOL32 rc = false;
  const int count = PointCount();
  if ( count >= 2 ) {
    m_pline.Reverse();
    m_t.Reverse();
    double* t = m_t.Array();
    for ( int i = 0; i < count; i++ ) {
      t[i] = -t[i];
    }
    rc = true;
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 ON_PolylineCurve::SetStartPoint(
        ON_3dPoint start_point
        )
{
  // 10 March 2009 Dale Lear
  //    I'm using exact compare instead of the fuzzy IsClosed()
  //    check to permit setting the start point.  This fixes
  //    a bug Mikko reported that prevented making polylines
  //    exactly closed when the end points were almost exactly
  //    equal.  At this point, I don't remember why we don't allow
  //    SetStartPoint() the start point of a closed curve.
  bool rc = false;
  int count = m_pline.Count();
  if (    count >= 2 
    && ( !m_pline[0].IsValid()
         || m_pline[count-1].x != m_pline[0].x // used to call IsClosed()
         || m_pline[count-1].y != m_pline[0].y
         || m_pline[count-1].z != m_pline[0].z
       )
     )
  {
    m_pline[0] = start_point;
    rc = true;
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 ON_PolylineCurve::SetEndPoint(
        ON_3dPoint end_point
        )
{
  // 10 March 2009 Dale Lear
  //    I'm using exact compare instead of the fuzzy IsClosed()
  //    check to permit setting the start point.  This fixes
  //    a bug Mikko reported that prevented making polylines
  //    exactly closed when the end points were almost exactly
  //    equal.  At this point, I don't remember why we don't allow
  //    SetEndPoint() the end point of a closed curve.
  bool rc = false;
  int count = m_pline.Count();
  if (    count >= 2 
    && ( !m_pline[count-1].IsValid()
         || m_pline[count-1].x != m_pline[0].x // used to call IsClosed()
         || m_pline[count-1].y != m_pline[0].y
         || m_pline[count-1].z != m_pline[0].z
       )
     )
  {
    m_pline[count-1] = end_point;
    rc = true;
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 
ON_PolylineCurve::Evaluate( // returns false if unable to evaluate
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
  ON_BOOL32 rc = false;
  const int count = PointCount();
  if ( count >= 2 ) 
  {
    int segment_index = ON_NurbsSpanIndex(2,count,m_t,t,side,(hint)?*hint:0);

    if ( -2 == side || 2 == side )
    {
      // 9 November 2010 Dale Lear - ON_TuneupEvaluationParameter fix
      //   When evluation passes through ON_CurveProxy or ON_PolyCurve reparamterization
      //   and the original side parameter was -1 or +1, it is changed to -2 or +2
      //   to indicate that if t is numerically closed to an end paramter, then
      //   it should be tuned up to be at the end paramter.
      double a = t;
      if ( ON_TuneupEvaluationParameter( side, m_t[segment_index], m_t[segment_index+1], &a) )
      {
        // recalculate segment index
        t = a;
        segment_index = ON_NurbsSpanIndex(2,count,m_t,t,side,segment_index);
      }
    }

    const double t0 = m_t[segment_index];
    const double t1 = m_t[segment_index+1];
    double s = (t == t1) ? 1.0 : (t-t0)/(t1-t0);
    const ON_3dPoint p = (1.0-s)*m_pline[segment_index] + s*m_pline[segment_index+1];
    v[0] = p.x;
    v[1] = p.y;
    if ( m_dim == 3 )
      v[2] = p.z;
    if ( der_count >= 1 ) {
      v += v_stride;
      ON_3dVector d = 1.0/(t1-t0)*(m_pline[segment_index+1] - m_pline[segment_index]);
      v[0] = d.x;
      v[1] = d.y;
      if ( m_dim == 3 )
        v[2] = d.z;
    }
    for ( int di = 2; di <= der_count; di++ ) {
      v += v_stride;
      v[0] = 0.0;
      v[1] = 0.0;
      if ( m_dim == 3 )
        v[2] = 0.0;
    }
    if ( hint )
      *hint = segment_index;
    rc = true;
  }
  return rc;
}

ON_BOOL32 
ON_PolylineCurve::PointCount() const
{
  return m_pline.PointCount();
}

bool ON_PolylineCurve::Append( const ON_PolylineCurve& c )
{

  if ( PointCount() == 0 ) {
    *this = c;
    return IsValid() ? true : false;
  }

  if (!IsValid() || !c.IsValid())
    return false;

  if ( c.Dimension() == 3 &&  Dimension() == 2) 
    m_dim = 3;

  m_pline.Remove();
  m_pline.Append(c.m_pline.Count(), c.m_pline.Array());
  m_t.Reserve(m_t.Count()+c.m_t.Count()-1);
  double del = *m_t.Last() - c.m_t[0];
  int i;
  for (i=1; i<c.m_t.Count(); i++)
    m_t.Append(c.m_t[i] + del);

  return true;
}

// returns true if t is sufficiently close to m_t[index]
bool ON_PolylineCurve::ParameterSearch(double t, int& index, bool bEnableSnap) const{
	return ON_Curve::ParameterSearch( t, index,bEnableSnap, m_t, ON_SQRT_EPSILON);
}


ON_BOOL32 ON_PolylineCurve::Trim( const ON_Interval& domain )
{
  int segment_count = m_t.Count()-1;

	if ( segment_count < 1 || m_t.Count() != m_pline.Count() || !domain.IsIncreasing() )
    return false;

  const ON_Interval original_polyline_domain = Domain();
  if ( !original_polyline_domain.IsIncreasing() )
    return false;
  
  ON_Interval output_domain = domain;
  if ( !output_domain.Intersection(original_polyline_domain) )
    return false;
	if(!output_domain.IsIncreasing())
		return false;

  ON_Interval actual_trim_domain = output_domain;

  int i, j;
  int s0 = -2; // s0 gets set to index of first segment we keep
  int s1 = -3; // s1 gets set to index of last segment we keep
  
	if ( ParameterSearch(output_domain[0], s0, true ) )
  {
    // ParameterSearch says domain[0] is within "microtol" of
    // m_t[s0].  So we will actually trim at m_t[s0].
    if (s0 >= 0 && s0 <= segment_count)
    {
      actual_trim_domain[0]=m_t[s0];
    }
  }

	if ( ParameterSearch(output_domain[1], s1, true ) )
  {
    if (s1 >= 0 && s1 <= segment_count )
    {
      // ParameterSearch says domain[1] is within "microtol" of
      // m_t[s1].  So we will actually trim at m_t[s1].
      actual_trim_domain[1]=m_t[s1];
      s1--;
    }
  }

  if ( !actual_trim_domain.IsIncreasing() )
  {
    // After microtol snapping, there is not enough curve left to trim.
    return false;
  }

  if ( s0 < 0 || s0 > s1 || s1 >= segment_count )
  {
    // Because output_domain is a subinterval of original_polyline_domain,
    // the only way that (s0 < 0 || s0 > s1 || s1 >= segment_count) can be true
    // is if something is seriously wrong with the m_t[] values.
    return false;
  }

  // we will begin modifying the polyline
  DestroyCurveTree();

  if ( actual_trim_domain == original_polyline_domain )
  {
    // ParameterSearch says that the ends of output_domain
    // were microtol away from being the entire curve.  
    // Set the domain and return.
    m_t[0] = output_domain[0];
    m_t[segment_count] = output_domain[1];
    return true;
  }

  if ( s1 < segment_count-1 )
  {
    m_t.SetCount(s1+2);
    m_pline.SetCount(s1+2);
    segment_count = s1+1;
  }

  if ( s0 > 0 )
  {
    double* tmp_t = m_t.Array();
    ON_3dPoint* tmp_P = m_pline.Array();
    for ( i = 0, j = s0; j <= segment_count; i++, j++ )
    {
      tmp_t[i] = tmp_t[j];
      tmp_P[i] = tmp_P[j];
    }
    s1 -= s0;
    s0 = 0;
    m_t.SetCount(s1+2);
    m_pline.SetCount(s1+2);
    segment_count = s1+1;
  }

  bool bTrimFirstSegment = ( m_t[0] < actual_trim_domain[0] || (0 == s1 && actual_trim_domain[1] < m_t[1]) );
  bool bTrimLastSegment = (s1>s0 && m_t[s1] < actual_trim_domain[1] && actual_trim_domain[1] < m_t[s1+1]);

  if ( bTrimFirstSegment )
  {
    ON_Interval seg_domain(m_t[0],m_t[1]);
    ON_3dPoint Q0 = m_pline[0];
    ON_3dPoint Q1 = m_pline[1];
    ON_Line seg_chord(Q0,Q1);
    double np0 = 0.0;
    double np1 = 1.0;
    bool bSet0 = false;
    bool bSet1 = false;
    if ( m_t[0] < actual_trim_domain[0] && actual_trim_domain[0] < m_t[1] )
    {
      np0 = seg_domain.NormalizedParameterAt(actual_trim_domain[0]);
      Q0 = seg_chord.PointAt( np0 );
      bSet0 = true;
    }
    if ( 0 == s1 && m_t[0] < actual_trim_domain[1] && actual_trim_domain[1] < m_t[1] )
    {
      np1 = seg_domain.NormalizedParameterAt(actual_trim_domain[1]);
      Q1 = seg_chord.PointAt( np1 );
      bSet1 = true;
    }
    
    if ( np0 >= np1 )
      return false; // trim is not viable

    if ( bSet0 )
    {
      if ( np0 >= 1.0-ON_SQRT_EPSILON && Q0.DistanceTo(Q1) <= ON_ZERO_TOLERANCE && s1>0 && m_t[1] < actual_trim_domain[1] )
      {
        // trim will leave a micro segment at the start - just remove the first segment
        m_t.Remove(0);
        m_pline.Remove(0);
        s1--;
        segment_count--;
        actual_trim_domain[0] = m_t[0];
      }
      m_t[0] = actual_trim_domain[0];
      m_pline[0] = Q0;
    }
    if ( bSet1 )
    {
      m_t[1] = actual_trim_domain[1];
      m_pline[1] = Q1;
    }
  }

  if ( bTrimLastSegment )
  {
    ON_Interval seg_domain(m_t[s1],m_t[s1+1]);
    ON_3dPoint Q0 = m_pline[s1];
    ON_3dPoint Q1 = m_pline[s1+1];
    ON_Line seg_chord(Q0,Q1);
    double np = seg_domain.NormalizedParameterAt(actual_trim_domain[1]);
    Q1 = seg_chord.PointAt(np);
    if ( np <= ON_SQRT_EPSILON && Q1.DistanceTo(Q0) <= ON_ZERO_TOLERANCE && s1 > 0 )
    {
        // trim will leave a micro segment at the end - just remove the last segment
      m_pline.SetCount(s1+1);
      m_t.SetCount(s1+1);
      s1--;
      segment_count--;
      actual_trim_domain[1] = m_t[s1+1];
    }
    m_t[s1+1] = actual_trim_domain[1];
    m_pline[s1+1] = Q1;
  }

  // If we get this far, trims were is successful.
  // The following makes potential tiny adjustments
  // that need to happen when trims get snapped to
  // input m_t[] values that are within fuzz of the
  // output_domain[] values.
	m_t[0] = output_domain[0];
  m_t[m_t.Count()-1] = output_domain[1];

  return true;
}

bool ON_PolylineCurve::Extend(
  const ON_Interval& domain
  )

{
  if (IsClosed()) 
    return false;
  if (PointCount() < 2) 
    return false;
  if ( !domain.IsIncreasing() )
    return false;
  bool changed = false;
  if ( domain == Domain() )
    return true;
  
  if (domain[0] < m_t[0]){
    changed = true;
    double len = m_t[1] - m_t[0];
    if ( len <= 0.0 )
      return false;
    ON_3dVector V = m_pline[1] - m_pline[0];
    ON_3dPoint Q0 = m_pline[0];
    Q0 += (domain[0]-m_t[0])/len*V;
    m_t[0] = domain[0];
    m_pline[0] = Q0;
  }

  int last = PointCount()-1;
  if (domain[1] > m_t[last]){
    changed = true;
    double len = m_t[last] - m_t[last-1];
    if ( len <= 0.0 )
      return false;
    ON_3dVector V = m_pline[last] - m_pline[last-1];
    ON_3dPoint Q1 = m_pline[last];
    Q1 += (domain[1]-m_t[last])/len*V;
    m_t[last] = domain[1];
    m_pline[last] = Q1;
  }

  if (changed){
    DestroyCurveTree();
  }
  return changed;
}



ON_BOOL32 ON_PolylineCurve::Split(
    double t,
    ON_Curve*& left_side,
    ON_Curve*& right_side
  ) const
{
  bool rc = false;
  ON_PolylineCurve* left_pl=0;
  ON_PolylineCurve* right_pl=0;
  if ( left_side ) 
  {
    left_pl = ON_PolylineCurve::Cast(left_side);
    if (!left_pl)
      return false;
  }
  if ( right_side )
  {
    right_pl = ON_PolylineCurve::Cast(right_side);
    if (!right_pl)
      return false;
  }

  // count = number of polyline segments
  const int count = m_t.Count()-1;
  if ( count >= 1 && m_t[0] < t && t < m_t[count] )
	{
    // March 26 2003 Greg Arden
    //   Use new function ParameterSearch() to snap parameter value
	  //	 when close to break point.
		int segment_index;
		bool split_at_break = ParameterSearch(t, segment_index, true);

    // 22 August 2008 Dale Lear
    //   Added segment_index checks to fix bug when
    //   t = m_t[count]-epsilon and segment_index is
    //   returned as count.  In this case right_point_count
    //   go set to 1 and an invalid polyline was returned.
    //   The || in the first expression prevents splitting
    //   when (t=m_t[0]+epsilon and epsilon is small enough
    //   that parameter search considers t to be nearly equal
    //   to m_t[0].
    if (    ( segment_index >= 1 || (false==split_at_break && 0 == segment_index) )
         && segment_index < count 
         && m_t[0] < t && t < m_t[count] 
       )
    {
      int left_point_count = (split_at_break) 
                           ? segment_index+1
                           : segment_index+2;	
      int right_point_count = m_t.Count() - segment_index;

      if ( left_pl != this )
      {
        if ( !left_pl )
          left_pl = new ON_PolylineCurve();
        left_pl->m_t.Reserve(left_point_count);
        left_pl->m_t.SetCount(left_point_count);
        left_pl->m_pline.Reserve(left_point_count);
        left_pl->m_pline.SetCount(left_point_count);
        memcpy( left_pl->m_t.Array(), m_t.Array(), left_point_count*sizeof(double) );
        memcpy( left_pl->m_pline.Array(), m_pline.Array(), left_point_count*sizeof(ON_3dPoint) );
				if(split_at_break)
        {
					// reparameterize the last segment 
					*left_pl->m_t.Last()= t;
				}
        left_pl->m_dim = m_dim;
      }
      if ( right_pl != this )
      {
        if ( !right_pl )
          right_pl = new ON_PolylineCurve();
        right_pl->m_t.Reserve(right_point_count);
        right_pl->m_t.SetCount(right_point_count);
        right_pl->m_pline.Reserve(right_point_count);
        right_pl->m_pline.SetCount(right_point_count);
        memcpy( right_pl->m_t.Array(), 
							  m_t.Array() + m_t.Count() - right_point_count, 
							  right_point_count*sizeof(double) );
        memcpy( right_pl->m_pline.Array(), 
							  m_pline.Array() + m_pline.Count() - right_point_count,
							  right_point_count*sizeof(ON_3dPoint) );
				if( split_at_break)
        {
					// Reparameterize the first segment
					right_pl->m_t[0] = t;
        }
        right_pl->m_dim = m_dim;
      }
      left_pl->Trim( ON_Interval( left_pl->m_t[0], t ) );
      right_pl->Trim( ON_Interval( t, *right_pl->m_t.Last() ) );  
		  rc = true;  
    }

	}


	left_side = left_pl;
	right_side = right_pl;
  return rc;
}


int ON_PolylineCurve::GetNurbForm( 
                                  ON_NurbsCurve& nurb, 
                                  double,
                                  const ON_Interval* subdomain  // OPTIONAL subdomain of ON::ProxyCurve::Domain()
                                  ) const
{
  int rc = 0;
  const int count = PointCount();
  if ( count < 2 )
    nurb.Destroy();
  else  if ( nurb.Create( Dimension(), false, 2, count) ) {
    int i;
    for ( i = 0; i < count; i++ ) {
      nurb.SetKnot( i, m_t[i] );
      nurb.SetCV( i, m_pline[i] );
    }
    if ( subdomain && *subdomain != Domain() )
      nurb.Trim(*subdomain);
    if ( nurb.IsValid() )
      rc = 1;
  }
  return rc;
}

int ON_PolylineCurve::HasNurbForm() const
{
  if (PointCount() < 2)
    return 0;
  if (!IsValid())
    return 0;
  return 1;
}

ON_BOOL32 ON_PolylineCurve::GetCurveParameterFromNurbFormParameter(
      double nurbs_t,
      double* curve_t
      ) const
{
  *curve_t = nurbs_t;
  return true;
}

ON_BOOL32 ON_PolylineCurve::GetNurbFormParameterFromCurveParameter(
      double curve_t,
      double* nurbs_t
      ) const
{
  *nurbs_t = curve_t;
  return true;
}

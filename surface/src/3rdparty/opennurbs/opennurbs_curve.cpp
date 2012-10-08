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

ON_VIRTUAL_OBJECT_IMPLEMENT(ON_Curve,ON_Geometry,"4ED7D4D7-E947-11d3-BFE5-0010830122F0");

ON_Curve::ON_Curve()
: ON_Geometry()
{}

ON_Curve::ON_Curve(const ON_Curve& src)
: ON_Geometry(src)
{}

ON_Curve& ON_Curve::operator=(const ON_Curve& src)
{
  if ( this != &src ) {
    DestroyCurveTree();
    ON_Geometry::operator=(src);
  }
  return *this;
}

ON_Curve::~ON_Curve()
{
  // Do not call the (virtual) DestroyRuntimeCache or 
  // DestroyCurveTree (which calls DestroyRuntimeCache()
  // because it opens the potential for crashes in a
  // "dirty" destructors of classes derived from ON_Curve
  // that to not use DestroyRuntimeCache() in their
  // destructors and to not set deleted pointers to zero.
}

unsigned int ON_Curve::SizeOf() const
{
  unsigned int sz = ON_Geometry::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Geometry));
  // Currently, the size of m_ctree is not included
  // because this is cached runtime information.
  // Applications that care about object size are 
  // typically storing "inactive" objects for potential
  // future use and should call DestroyRuntimeCache(true)
  // to remove any runtime cache information.
  return sz;
}


ON_Curve* ON_Curve::DuplicateCurve() const
{
  // ON_CurveProxy overrides this virtual function.
  return Duplicate();
}


ON::object_type ON_Curve::ObjectType() const
{
  return ON::curve_object;
}


ON_BOOL32 ON_Curve::GetDomain(double* s0,double* s1) const
{
  ON_BOOL32 rc = false;
  ON_Interval d = Domain();
  if ( d.IsIncreasing() ) {
    if(s0) *s0 = d.Min();
    if (s1) *s1 = d.Max();
    rc = true;
  }
  return rc;
}

void ON_Curve::DestroyCurveTree()
{
  DestroyRuntimeCache(true);
}

bool ON_Curve::GetTightBoundingBox( 
		ON_BoundingBox& tight_bbox, 
    int bGrowBox,
		const ON_Xform* xform
    ) const
{
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }

  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }

  // In general, putting start and end point in the box lets me avoid
  // testing lots of nodes.
  ON_3dPoint P = PointAtStart();
  if ( xform )
    P = (*xform)*P;
  tight_bbox.Set( P, bGrowBox );
  bGrowBox = true;

  P = PointAtEnd();
  if ( xform )
    P = (*xform)*P;
  tight_bbox.Set( P, bGrowBox );

  ON_BoundingBox curve_bbox = BoundingBox();
  if ( ON_WorldBBoxIsInTightBBox( tight_bbox, curve_bbox, xform ) )
  {
    // Curve is inside tight_bbox
    return true;
  }

  ON_NurbsCurve N;
  if ( 0 == GetNurbForm(N) )
    return false;
  if ( N.m_order < 2 || N.m_cv_count < N.m_order )
    return false;

  ON_BezierCurve B;
  for ( int span_index = 0; span_index <= N.m_cv_count - N.m_order; span_index++ )
  {
    if ( !(N.m_knot[span_index + N.m_order-2] < N.m_knot[span_index + N.m_order-1]) )
      continue;
    if ( !N.ConvertSpanToBezier( span_index, B ) )
      continue;
    if ( !B.GetTightBoundingBox(tight_bbox,bGrowBox,xform) )
      continue;
    bGrowBox = true;
  }

  return (0!=bGrowBox);
}

bool ON_Curve::SetDomain( ON_Interval domain )
{
  return ( domain.IsIncreasing() && SetDomain( domain[0], domain[1] )) ? true : false;
}


ON_BOOL32 ON_Curve::SetDomain( double, double )
{
  // this virtual function is overridden by curves that can change their domain.
  return false;
}

ON_BOOL32 ON_Curve::ChangeClosedCurveSeam( double t )
{
  // this virtual function is overridden by curves that can be closed
  return false;
}

//virtual
bool ON_Curve::ChangeDimension( int desired_dimension )
{
  return (desired_dimension > 0 && desired_dimension == Dimension() );
}


//virtual
ON_BOOL32 ON_Curve::GetSpanVectorIndex(
       double t,                // [IN] t = evaluation parameter
       int side,                // [IN] side 0 = default, -1 = from below, +1 = from above
       int* span_vector_i,      // [OUT] span vector index
       ON_Interval* span_domain // [OUT] domain of the span containing "t"
       ) const
{
  ON_BOOL32 rc = false;
  int i;
  int span_count = SpanCount();
  if ( span_count > 0 ) {
    double* span_vector = (double*)onmalloc((span_count+1)*sizeof(span_vector[0]));
    rc = GetSpanVector( span_vector );
    if (rc) {
      i = ON_NurbsSpanIndex( 2, span_count+1, span_vector, t, side, 0 );
      if ( i >= 0 && i <= span_count ) {
        if ( span_vector_i )
          *span_vector_i = i;
        if ( span_domain )
          span_domain->Set( span_vector[i], span_vector[i+1] );
      }
      else
        rc = false;
    }
    onfree(span_vector);
  }
  return rc;
}


ON_BOOL32 ON_Curve::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
       double t,       // t = parameter in domain
       double* tminus, // tminus
       double* tplus   // tplus
       ) const
{
  ON_BOOL32 rc = false;
  ON_Interval d = Domain();
  if ( d.IsIncreasing() )
    rc = ON_GetParameterTolerance( d[0], d[1], t, tminus, tplus );
  return rc;
}

int ON_Curve::IsPolyline(
      ON_SimpleArray<ON_3dPoint>* pline_points, // default = NULL
      ON_SimpleArray<double>* pline_t // default = NULL
      ) const
{
  // virtual function that is overridden
  return 0;
}


ON_BOOL32 ON_Curve::IsLinear( double tolerance ) const
{
  ON_BOOL32 rc = false;
  if ( Dimension() == 2 || Dimension() == 3 ) {
    const int span_count = SpanCount();
    const int span_degree = Degree();
    if ( span_count > 0 ) {
      ON_SimpleArray<double> s(span_count+1);
      s.SetCount(span_count+1);
      if ( GetSpanVector( s.Array() ) ) {
        if ( tolerance == 0.0 )
          tolerance = ON_ZERO_TOLERANCE;
        ON_Line line( PointAtStart(), PointAtEnd() );
        if ( line.Length() > tolerance ) {
          double t, t0, d, delta;
          ON_Interval sp;
          int n, i, span_index;
          rc = true;
          t0 = 0;              //  Domain()[0];
          ON_3dPoint P;

          for ( span_index = 0; span_index < span_count; span_index++ ) {
            sp.Set( s[span_index], s[span_index+1] );
            n = 2*span_degree+1;
            delta = 1.0/n;
            for ( i = (span_index)?0:1; i < n; i++ ) {
              P = PointAt( sp.ParameterAt(i*delta) );
              if ( !line.ClosestPointTo( P, &t ) )
                rc = false;
              else if ( t < t0 )
                rc = false;
							else if (t > 1.0 + ON_SQRT_EPSILON)
								rc = false;
              d = P.DistanceTo( line.PointAt(t) );
              if ( d > tolerance )
                rc = false;
              t0 = t;
            }
          }
        }
      }
    }
  }
  return rc;
}

bool ON_Curve::IsEllipse(
    const ON_Plane* plane,
    ON_Ellipse* ellipse,
    double tolerance
    ) const
{
  // virtual function
  ON_Arc arc;
  bool rc = IsArc(plane,&arc,tolerance)?true:false;
  if (rc && ellipse)
  {
    ellipse->plane = arc.plane;
    ellipse->radius[0] = arc.radius;
    ellipse->radius[1] = arc.radius;
  }
  return rc;
}

bool ON_Curve::IsArcAt( 
  double t, 
  const ON_Plane* plane,
  ON_Arc* arc,
  double tolerance,
  double* t0, 
  double* t1
  ) const
{
  double k, k0, k1;
  int hint;
  if ( !GetDomain(&k0,&k1) )
    return false;
  if ( 0 != t0 )
    *t0 = k0;
  if ( 0 != t1 )
    *t1 = k1;
  if ( !ON_IsValid(t) )
    return false;
  if ( ! (t <= k1) )
    return false;

  if ( IsArc(plane,arc,tolerance) )
    return true; // entire curve is an arc

  // check sub-segments
  hint = 0;
  for ( k = k0; k0 <= t && GetNextDiscontinuity(ON::G2_locus_continuous, k0, k1, &k, &hint); k0 = k )
  {
    if ( !(k > k0) )
      break; // sanity check to prevent infinite loops
    if( t <= k )
    {
      if ( 0 != t0 )
        *t0 = k0;
      if ( 0 != t1 )
        *t1 = k1;
      ON_CurveProxy subcrv(this,ON_Interval(k0,k));
      if ( subcrv.IsArc(plane,arc,tolerance) )
        return true;

      // NOTE WELL: 
      //   When t == k, we need to check the next segment as well
      //   (happens when t is the parameter between a line and arc segment.)
      //   The "k0 <= t" test is in the for() condition will
      //   terminate the loop when t < k
    }
  }

  return false;
}


ON_BOOL32 ON_Curve::IsArc( const ON_Plane* plane, ON_Arc* arc, double tolerance ) const
{
  ON_BOOL32 rc = false;
  double c0, c, t, delta;
  int n, i, span_index;
  ON_Plane pln;
  ON_Arc a;
  ON_3dPoint P, C;

  if ( !plane ) {
    if ( !IsPlanar(&pln,tolerance) )
      return false;
    plane = &pln;
  }
  if ( !arc )
    arc = &a;
  const int span_count = SpanCount();
  const int span_degree = Degree();
  if ( span_count < 1 )
    return false;
  ON_SimpleArray<double> d(span_count+1);
  d.SetCount(span_count+1);
  if ( !GetSpanVector(d.Array()) )
    return false;

  const ON_BOOL32 bIsClosed = IsClosed();

  ON_3dPoint P0 = PointAt( d[0] );
  t = bIsClosed ? 0.5*d[0] + 0.5*d[span_count] : d[span_count];
  ON_3dPoint P1 = PointAt( 0.5*d[0] + 0.5*t );
  ON_3dPoint P2 = PointAt( t );

  if ( !arc->Create(P0,P1,P2) )
    return false;

  if ( bIsClosed )
    arc->SetAngleRadians(2.0*ON_PI);

  ON_Interval arc_domain = arc->Domain();
  ON_3dPoint A0 = arc->PointAt(arc_domain[0]);
  ON_3dPoint A1 = arc->PointAt(arc_domain[1]);
  ON_3dPoint C0 = PointAtStart();
  ON_3dPoint C1 = PointAtEnd();
  if (    false == ON_PointsAreCoincident(3,0,&A0.x,&C0.x) 
       || false == ON_PointsAreCoincident(3,0,&A1.x,&C1.x) 
     )
  {
    return false;
  }


  if ( tolerance == 0.0 )
    tolerance = ON_ZERO_TOLERANCE;
  rc = true;
  c0 = 0.0;
  for ( span_index = 0; rc && span_index < span_count; span_index++ ) {
    n = 2*span_degree+1;
    if ( n < 4 )
      n = 4;
    delta = 1.0/n;
    for ( i = 0; i < n; i++ ) {
      t = i*delta;
      P = PointAt( (1.0-t)*d[span_index] + t*d[span_index+1] );
      if ( !arc->ClosestPointTo(P,&c) ) {
        rc = false;
        break;        
      }
      if ( c < c0 ) {
        rc = false;
        break;
      }
      C = arc->PointAt(c);        
      if ( C.DistanceTo(P) > tolerance ) {
        rc = 0;
        break;
      }
      c0 = c;
    }
  }

  return rc;
}

ON_BOOL32 ON_Curve::IsPlanar( ON_Plane* plane, double tolerance ) const
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  if ( dim == 2 ) 
  {
    // all 2d curves use this code to set the plane
    // so that there is consistent behavior.
    rc = true;
    if ( plane )
    {
      *plane = ON_xy_plane;
      //plane->CreateFromFrame( PointAtStart(), ON_xaxis, ON_yaxis );
    }
  }
  else if ( IsLinear(tolerance) )
  {
    rc = true;
    if ( plane )
    {
      ON_Line line( PointAtStart(), PointAtEnd() );
      if ( !line.InPlane( *plane, tolerance ) )
        line.InPlane( *plane, 0.0 );
    }
  }
  else if ( dim == 3 ) 
  {
    const int span_count = SpanCount();
    if ( span_count < 1 )
      return false;
    const int span_degree = Degree();
    if ( span_degree < 1 )
      return false;
    ON_SimpleArray<double> s(span_count+1);
    s.SetCount(span_count+1);
    if ( !GetSpanVector(s.Array()) )
      return false;
    ON_Interval d = Domain();

    // use initial point, tangent, and evaluated spans to guess a plane
    ON_3dPoint pt = PointAt(d.ParameterAt(0.0));
    ON_3dVector x = TangentAt(d.ParameterAt(0.0));
    if ( x.Length() < 0.95 ) {
      return false;
    }
    int n = (span_degree > 1) ? span_degree+1 : span_degree;
    double delta = 1.0/n;
    int i, span_index, hint = 0;
    ON_3dPoint q;
    ON_3dVector y;
    ON_BOOL32 bNeedY = true;
    for ( span_index = 0; span_index < span_count && bNeedY; span_index++ ) {
      d.Set(s[span_index],s[span_index+1]);
      for ( i = span_index ? 0 : 1; i < n && bNeedY; i++ ) {
        if ( !EvPoint( d.ParameterAt(i*delta), q, 0, &hint ) )
          return false;
        y = q-pt;
        y = y - (y*x)*x;
        bNeedY = ( y.Length()  <= 1.0e-6 );
      }
    }
    if ( bNeedY )
      y.PerpendicularTo(x);
    ON_Plane pln( pt, x, y );
    if ( plane )
      *plane = pln;

    // test
    rc = true;
    n = 2*span_degree + 1;
    delta = 1.0/n;
    double h = pln.plane_equation.ValueAt(PointAtEnd());
    if ( fabs(h) > tolerance )
      rc = false;
    hint = 0;
    for ( span_index = 0; rc && span_index < span_count; span_index++ ) {
      d.Set(s[span_index],s[span_index+1]);
      for ( i = 0; rc && i < n; i++ ) {
        if ( !EvPoint( d.ParameterAt(i*delta), q, 0, &hint ) )
          rc = false;
        else {
          h = pln.plane_equation.ValueAt(q);
          if ( fabs(h) > tolerance )
            rc = false;
        }
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_Curve::IsClosed() const
{
  bool rc = false;
  double *a, *b, *c, *p, w[12];
  const int dim = Dimension();
  a = 0;
  if ( dim > 1 ) 
  {
    ON_Interval d = Domain();
    a = (dim>3) ? (double*)onmalloc(dim*4*sizeof(*a)) : w;
    b = a+dim;
    c = b+dim;
    p = c+dim;
    if (    Evaluate( d.ParameterAt(0.0), 0, dim, a, 1 )
         && Evaluate( d.ParameterAt(1.0), 0, dim, p,-1 ) 
       )
    {
      // Note:  The point compare test should be the same
      //        as the one used in ON_PolyCurve::HasGap().
      //
      if ( ON_PointsAreCoincident( dim, false, a, p ) ) 
      {
        if (    Evaluate( d.ParameterAt(1.0/3.0), 0, dim, b, 0 ) 
             && Evaluate( d.ParameterAt(2.0/3.0), 0, dim, c, 0 )
           )
        {
          if (    false == ON_PointsAreCoincident( dim, false, a, b ) 
               && false == ON_PointsAreCoincident( dim, false, p, c ) 
               && false == ON_PointsAreCoincident( dim, false, p, b ) 
               && false == ON_PointsAreCoincident( dim, false, p, c ) 
             )
          {
            rc = true;
          }
        }
      }
    }
    if ( dim > 3 && 0 != a )
      onfree(a);
  }

  return rc;
}


ON_BOOL32 ON_Curve::IsPeriodic() const
{
  // curve types that may be periodic override this virtual function
  return false;
}

bool ON_Curve::GetNextDiscontinuity( 
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
  // this function must be overridden by curve objects that
  // can have parametric discontinuities on the interior of the curve.

  bool rc = false;

  if ( dtype )
    *dtype = 0;
  
  if ( t0 != t1 )
  {
    bool bTestC0 = false;
    bool bTestD1 = false;
    bool bTestD2 = false;
    bool bTestT = false;
    bool bTestK = false;
    switch(c)
    {
    case ON::C0_locus_continuous:
      bTestC0 = true;
      break;
    case ON::C1_locus_continuous:
      bTestC0 = true;
      bTestD1 = true;
      break;
    case ON::C2_locus_continuous:
      bTestC0 = true;
      bTestD1 = true;
      bTestD2 = true;
      break;
    case ON::G1_locus_continuous:
      bTestC0 = true;
      bTestT  = true;
      break;
    case ON::G2_locus_continuous:
      bTestC0 = true;
      bTestT  = true;
      bTestK  = true;
      break;
    default:
      // other values ignored on purpose.
      break;
    }

    if ( bTestC0 )
    {
      // 20 March 2003 Dale Lear:
      //   Have to look for locus discontinuities at ends.
      //   Must test both ends becuase t0 > t1 is valid input.
      //   In particular, for ON_CurveProxy::GetNextDiscontinuity() 
      //   to work correctly on reversed "real" curves, the 
      //   t0 > t1 must work right.
      ON_Interval domain = Domain();

      if ( t0 < domain[1] && t1 >= domain[1] )
        t1 = domain[1];
      else if ( t0 > domain[0] && t1 <= domain[0] )
        t1 = domain[0];

      if ( (t0 < domain[1] && t1 >= domain[1]) || (t0 > domain[0] && t1 <= domain[0]) )
      {
        if ( IsClosed() )
        {
          if ( bTestD1 || bTestT )
          {
            // need to check locus continuity at start/end of closed curve.
            ON_3dPoint Pa, Pb;
            ON_3dVector D1a, D1b, D2a, D2b;
            if (    Ev2Der(domain[0],Pa,D1a,D2a,1,NULL) 
                 && Ev2Der(domain[1],Pb,D1b,D2b,-1,NULL) )
            {
              Pb = Pa; // IsClosed() = true means assume Pa=Pb;
              if ( bTestD1 )
              {
                if ( !(D1a-D1b).IsTiny(D1b.MaximumCoordinate()*ON_SQRT_EPSILON ) )
                {
                  if ( dtype )
                    *dtype = 1;
                  *t = t1;
                  rc = true;
                }
                else if ( bTestD2 && !(D2a-D2b).IsTiny(D2b.MaximumCoordinate()*ON_SQRT_EPSILON) )
                {
                  if ( dtype )
                    *dtype = 2;
                  *t = t1;
                  rc = true;
                }

              }
              else if ( bTestT )
              {
                ON_3dVector Ta, Tb, Ka, Kb;
                ON_EvCurvature( D1a, D2a, Ta, Ka );
                ON_EvCurvature( D1b, D2b, Tb, Kb );
                if ( Ta*Tb < cos_angle_tolerance )
                {
                  if ( dtype )
                    *dtype = 1;
                  *t = t1;
                  rc = true;
                }
                else if ( bTestK )
                {
                  // NOTE: 
                  //  This test must exactly match the one
                  //  used in ON_NurbsCurve::GetNextDiscontinuity()
                  if ( !ON_IsG2CurvatureContinuous( Ka, Kb,
                                                    cos_angle_tolerance,
                                                    curvature_tolerance
                                                    )
                     )
                  {
                    if ( dtype )
                      *dtype = 2;
                    *t = t1;
                    rc = true;
                  }
                }
              }
            }
          }
        }
        else
        {
          // open curves are not locus continuous at ends.
          if (dtype )
            *dtype = 0; // locus C0 discontinuity
          *t = t1;
          rc = true;
        }
      }
    }
  }

  return rc;
}



bool ON_Curve::IsContinuous(
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
  ON_Interval domain = Domain();
  if ( !domain.IsIncreasing() )
  {
    return true;
  }

  ON_3dPoint Pm, Pp;
  ON_3dVector D1m, D1p, D2m, D2p, Tm, Tp, Km, Kp;

  bool bIsClosed = false;

  // 20 March 2003 Dale Lear
  //     I added this preable to handle the new
  //     locus continuity values.
  double t0 = t;
  double t1 = t;
  switch(desired_continuity)
  {
  case ON::C0_locus_continuous:
  case ON::C1_locus_continuous:
  case ON::C2_locus_continuous:
  case ON::G1_locus_continuous:
  case ON::G2_locus_continuous:
    if ( t <= domain[0] )
    {
      // By convention - see comments by ON::continuity enum.
      return true;
    }
    if ( t == domain[1] )
    {
      if ( !IsClosed() )
      {
        // open curves are not locus continuous at the end parameter
        // see comments by ON::continuity enum
        return false;
      }
      else 
      {
        if ( ON::C0_locus_continuous == desired_continuity )
        {
          return true;
        }
        bIsClosed = true;
      }

      t0 = domain[0];
      t1 = domain[1];
    }
    break;

  case ON::unknown_continuity:
  case ON::C0_continuous:
  case ON::C1_continuous:
  case ON::C2_continuous:
  case ON::G1_continuous:
  case ON::G2_continuous:
  case ON::Cinfinity_continuous:
  case ON::Gsmooth_continuous:
  default:
    // does not change pre 20 March behavior - just skips the out
    // of domain evaluation on parametric queries.
    if ( t <= domain[0] || t >= domain[1] )
      return true;
    break;
  }

  // at this point, no difference between parametric and locus tests.
  desired_continuity = ON::ParametricContinuity(desired_continuity);


  // this is slow and uses evaluation
  // virtual overrides on curve classes that can have multiple spans
  // are much faster because the avoid evaluation
  switch ( desired_continuity )
  {
  case ON::unknown_continuity:
    break;

  case ON::C0_continuous:  
    if ( !EvPoint( t1, Pm, -1, hint ) )
      return false;
    if ( !EvPoint( t0, Pp,  1, hint ) )
      return false;
    if ( bIsClosed )
      Pm = Pp;
    if ( !(Pm-Pp).IsTiny(point_tolerance) )
      return false;
    break;

  case ON::C1_continuous:
    if ( !Ev1Der( t1, Pm, D1m, -1, hint ) )
      return false;
    if ( !Ev1Der( t0, Pp, D1p,  1, hint ) )
      return false;
    if ( bIsClosed )
      Pm = Pp;
    if ( !(Pm-Pp).IsTiny(point_tolerance) || !(D1m-D1p).IsTiny(d1_tolerance) )
      return false;
    break;

  case ON::G1_continuous:
    if ( !EvTangent( t1, Pm, Tm, -1, hint ) )
      return false;
    if ( !EvTangent( t0, Pp, Tp,  1, hint ) )
      return false;
    if ( bIsClosed )
      Pm = Pp;
    if ( !(Pm-Pp).IsTiny(point_tolerance) || Tm*Tp < cos_angle_tolerance )
      return false;
    break;

  case ON::C2_continuous:
    if ( !Ev2Der( t1, Pm, D1m, D2m, -1, hint ) )
      return false;
    if ( !Ev2Der( t0, Pp, D1p, D2p,  1, hint ) )
      return false;
    if ( bIsClosed )
      Pm = Pp;
    if ( !(Pm-Pp).IsTiny(point_tolerance) || !(D1m-D1p).IsTiny(d1_tolerance) || !(D2m-D2p).IsTiny(d2_tolerance) )
      return false;
    break;

  case ON::G2_continuous:
  case ON::Gsmooth_continuous:
    if ( !EvCurvature( t1, Pm, Tm, Km, -1, hint ) )
      return false;
    if ( !EvCurvature( t0, Pp, Tp, Kp,  1, hint ) )
      return false;
    if ( !bIsClosed && !(Pm-Pp).IsTiny(point_tolerance) )
      return false;
    if ( Tm*Tp < cos_angle_tolerance )
      return false; // tangent discontinuity

    if ( desired_continuity == ON::Gsmooth_continuous )
    {
      if ( !ON_IsGsmoothCurvatureContinuous(Km,Kp,cos_angle_tolerance,curvature_tolerance) )
        return false;
    }
    else
    {
      if ( !ON_IsG2CurvatureContinuous(Km,Kp,cos_angle_tolerance,curvature_tolerance) )
        return false;
    }
    break;

  case ON::C0_locus_continuous:
  case ON::C1_locus_continuous:
  case ON::C2_locus_continuous:
  case ON::G1_locus_continuous:
  case ON::G2_locus_continuous:
  case ON::Cinfinity_continuous:
    break;
  }

  return true;
}


ON_3dPoint ON_Curve::PointAt( double t ) const
{
  ON_3dPoint p(0.0,0.0,0.0);
  if ( !EvPoint(t,p) )
    p = ON_UNSET_POINT;
  return p;
}

ON_3dPoint ON_Curve::PointAtStart() const
{
  return PointAt(Domain().Min());
}

ON_3dPoint ON_Curve::PointAtEnd() const
{
  return PointAt(Domain().Max());
}


ON_BOOL32 ON_Curve::SetStartPoint(ON_3dPoint start_point)
{
  return false;
}

ON_BOOL32 ON_Curve::SetEndPoint(ON_3dPoint end_point)
{
  return false;
}

ON_3dVector ON_Curve::DerivativeAt( double t ) const
{
  ON_3dPoint  p(0.0,0.0,0.0);
  ON_3dVector d(0.0,0.0,0.0);
  Ev1Der(t,p,d);
  return d;
}

ON_3dVector ON_Curve::TangentAt( double t ) const
{
  ON_3dPoint point;
  ON_3dVector tangent;
  EvTangent( t, point, tangent );
  return tangent;
}

ON_3dVector ON_Curve::CurvatureAt( double t ) const
{
  ON_3dPoint point;
  ON_3dVector tangent, kappa;
  EvCurvature( t, point, tangent, kappa );
  return kappa;
}

ON_BOOL32 ON_Curve::EvTangent(
       double t,
       ON_3dPoint& point,
       ON_3dVector& tangent,
       int side,
       int* hint
       ) const
{
  ON_3dVector D1, D2;//, K;
  tangent.Zero();
  int rc = Ev1Der( t, point, tangent, side, hint );
  if ( rc && !tangent.Unitize() ) 
  {
    if ( Ev2Der( t, point, D1, D2, side, hint ) )
    {
      // Use l'Hopital's rule to show that if the unit tanget
      // exists, the 1rst derivative is zero, and the 2nd
      // derivative is nonzero, then the unit tangent is equal
      // to +/-the unitized 2nd derivative.  The sign is equal
      // to the sign of D1(s) o D2(s) as s approaches the 
      // evaluation parameter.
      tangent = D2;
      rc = tangent.Unitize();
      if ( rc )
      {
        ON_Interval domain = Domain();
        double tminus = 0.0;
        double tplus = 0.0;
        if ( domain.IsIncreasing() && GetParameterTolerance( t, &tminus, &tplus ) )
        {
          ON_3dPoint p;
          ON_3dVector d1, d2;
          double eps = 0.0;
          double d1od2tol = 0.0; //1.0e-10; // 1e-5 is too big
          double d1od2;
          double tt = t;
          //double dt = 0.0;

          if ( (t < domain[1] && side >= 0) || (t == domain[0]) )
          {
            eps = tplus-t;
            if ( eps <= 0.0 || t+eps > domain.ParameterAt(0.1) )
              return rc;
          }
          else if ( (t > domain[0] && side < 0) || (t == domain[1]) )
          {
            eps = tminus - t;
            if ( eps >= 0.0 || t+eps < domain.ParameterAt(0.9) )
              return rc;
          }

          int i, negative_count=0, zero_count=0;
          int test_count = 3;
          for ( i = 0; i < test_count; i++, eps *= 0.5 )
          {
            tt = t + eps;
            if ( tt == t )
              break;
            if (!Ev2Der( tt, p, d1, d2, side, 0 ))
              break;
            d1od2 = d1*d2;
            if ( d1od2 > d1od2tol )
              break;
            if ( d1od2 < d1od2tol )
              negative_count++;
            else
              zero_count++;
          }
          if ( negative_count > 0 && test_count == negative_count+zero_count )
          {
            // all sampled d1od2 values were <= 0 
            // and at least one was strictly < 0.
            tangent.Reverse();
          }
        }
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_Curve::EvCurvature(
       double t,
       ON_3dPoint& point,
       ON_3dVector& tangent,
       ON_3dVector& kappa,
       int side,
       int* hint
       ) const
{
  ON_3dVector d1, d2;
  int rc = Ev2Der( t, point, d1, d2, side, hint );
  if ( rc )
    rc = ON_EvCurvature( d1, d2, tangent, kappa );
  return rc;
}



ON_BOOL32 ON_Curve::FrameAt( double t, ON_Plane& plane) const
{
  ON_BOOL32 rc = false;
  ON_Interval domain = Domain();
  if( t < domain[0] - ON_EPSILON || t > domain[1] + ON_EPSILON)
    return false;

  ON_3dPoint pt;
  ON_3dVector d1, d2, T, K;
  rc = Ev2Der( t, pt, d1, d2 );
  if (rc)
    rc = ON_EvCurvature( d1, d2, T, K );
  if (rc)
  {
    if ( !K.Unitize() )
      K.PerpendicularTo(T);
    K.Unitize();
    plane.origin = pt;
    plane.xaxis = T;
    plane.yaxis = K;
    plane.zaxis = ON_CrossProduct(plane.xaxis,plane.yaxis);
    plane.UpdateEquation();
  }
  return rc;
}

ON_BOOL32 ON_Curve::EvPoint( // returns false if unable to evaluate
       double t,         // evaluation parameter
       ON_3dPoint& point,   // returns value of curve
       int side,        // optional - determines which side to evaluate from
                       //         0 = default
                       //      <  0 to evaluate from below, 
                       //      >  0 to evaluate from above
       int* hint       // optional - evaluation hint used to speed repeated
                       //            evaluations
       ) const
{
  ON_BOOL32 rc = false;
  double ws[128];
  double* v;
  if ( Dimension() <= 3 ) {
    v = &point.x;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
  }
  else if ( Dimension() <= 128 ) {
    v = ws;
  }
  else {
    v = (double*)onmalloc(Dimension()*sizeof(*v));
  }
  rc = Evaluate( t, 0, Dimension(), v, side, hint );
  if ( Dimension() > 3 ) {
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    if ( Dimension() > 128 )
      onfree(v);
  }
  return rc;
}

bool ON_Brep::EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const
{
  // TODO
  return false;
}

bool ON_Surface::EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const
{
  // TODO
  return false;
}

bool ON_PolyCurve::EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const
{
  // TODO
  return false;
}

bool ON_Curve::EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const
{
  // virtual function default
  bool rc = false;

  ON_3dPoint Q = ON_UNSET_POINT;
  if ( 1 == objref.m_evp.m_t_type )
  {
    if ( !EvPoint(objref.m_evp.m_t[0],Q) )
      Q = ON_UNSET_POINT;
  }

  switch( objref.m_osnap_mode )
  {
  case ON::os_center:
    {
      ON_Ellipse ellipse;
      if ( IsEllipse(0,&ellipse) )
      {
        P = ellipse.plane.origin;
        rc = true;
      }
      else
      {
        ON_SimpleArray<ON_3dPoint> pline;
        if ( IsClosed() && IsPolyline(&pline) && pline.Count() >= 4 )
        {
          P = pline[0];
          int i;
          for ( i = pline.Count()-2; i > 0; i-- )
          {
            Q = pline[i];
            P.x += Q.x; P.y += Q.y; P.z += Q.z;
          }
          double s = 1.0/(pline.Count()-1.0);
          P.x *= s; P.y *= s; P.z *= s;
          rc = true;
        }
        else if ( Q.IsValid() )
        {
          ON_3dVector T, K;
          if ( EvCurvature(objref.m_evp.m_t[0],Q,T,K) )
          {
            double k = K.Length();
            if ( k > 0.0 )
            {
              P = Q + (1.0/(k*k))*K;
              rc = true;
            }
          }
        }
      }
    }
    break;

  case ON::os_focus:
    {
      ON_Ellipse ellipse;
      if ( IsEllipse(0,&ellipse) )
      {
        ON_3dPoint F1, F2;
        if ( ellipse.GetFoci(F1,F2) )
        {
          P = ( F1.DistanceTo(Q) <= F1.DistanceTo(Q)) ? F1 : F2;
          rc = true;
        }
      }
    }
    break;

  case ON::os_midpoint:
    {
    }
    break;

  case ON::os_end:
    {
      ON_SimpleArray<ON_3dPoint> pline;
      if ( IsPolyline(&pline)  )
      {
        P = pline[0];
        double d = P.DistanceTo(Q);
        int i;
        for ( i = 1; i < pline.Count(); i++ )
        {
          double d1 = pline[i].DistanceTo(Q);
          if ( d1 < d )
          {
            d = d1;
            P = pline[i];
            rc = true;
          }
        }
      }
      else
      {
        P = PointAtStart();
        rc = true;
        if ( !IsClosed() )
        {
          ON_3dPoint P1 = PointAtEnd();
          if ( P.DistanceTo(Q) > P1.DistanceTo(Q) )
          {
            P = P1;
          }
        }
      }      
    }
    break;

  default:
    if ( Q.IsValid() )
    {
      P = Q;
      rc = true;
    }
    break;
  }

  return rc;
}

ON_BOOL32 ON_Curve::Ev1Der( // returns false if unable to evaluate
       double t,         // evaluation parameter
       ON_3dPoint& point,
       ON_3dVector& derivative,
       int side,        // optional - determines which side to evaluate from
                       //      <= 0 to evaluate from below, 
                       //      >  0 to evaluate from above
       int* hint       // optional - evaluation hint used to speed repeated
                       //            evaluations
       ) const
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  double ws[2*64];
  double* v;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  derivative.x = 0.0;
  derivative.y = 0.0;
  derivative.z = 0.0;
  if ( dim <= 64 ) {
    v = ws;
  }
  else {
    v = (double*)onmalloc(2*dim*sizeof(*v));
  }
  rc = Evaluate( t, 1, dim, v, side, hint );
  point.x = v[0];
  derivative.x = v[dim];
  if ( dim > 1 ) {
    point.y = v[1];
    derivative.y = v[dim+1];
    if ( dim > 2 ) {
      point.z = v[2];
      derivative.z = v[dim+2];
      if ( dim > 64 )
        onfree(v);
    }
  }

  return rc;
}

ON_BOOL32 ON_Curve::Ev2Der( // returns false if unable to evaluate
       double t,         // evaluation parameter
       ON_3dPoint& point,
       ON_3dVector& firstDervative,
       ON_3dVector& secondDervative,
       int side,        // optional - determines which side to evaluate from
                       //      <= 0 to evaluate from below, 
                       //      >  0 to evaluate from above
       int* hint       // optional - evaluation hint used to speed repeated
                       //            evaluations
       ) const
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  double ws[3*64];
  double* v;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  firstDervative.x = 0.0;
  firstDervative.y = 0.0;
  firstDervative.z = 0.0;
  secondDervative.x = 0.0;
  secondDervative.y = 0.0;
  secondDervative.z = 0.0;
  if ( dim <= 64 ) {
    v = ws;
  }
  else {
    v = (double*)onmalloc(3*dim*sizeof(*v));
  }
  rc = Evaluate( t, 2, dim, v, side, hint );
  point.x = v[0];
  firstDervative.x = v[dim];
  secondDervative.x = v[2*dim];
  if ( dim > 1 ) {
    point.y = v[1];
    firstDervative.y = v[dim+1];
    secondDervative.y = v[2*dim+1];
    if ( dim > 2 ) {
      point.z = v[2];
      firstDervative.z = v[dim+2];
      secondDervative.z = v[2*dim+2];
      if ( dim > 64 )
        onfree(v);
    }
  }

  return rc;
}

static
ON::eCurveType ON_CurveType( const ON_Curve* curve )
{
  const ON_ClassId* curve_id = &ON_Curve::m_ON_Curve_class_id;
  const ON_ClassId* id = curve->ClassId();

  // "fake virtual" handling of fast/easy special cases
  while (0 != id && curve_id != id )
  {
    if ( &ON_ArcCurve::m_ON_ArcCurve_class_id == id )
      return ON::ctArc;
    if ( &ON_LineCurve::m_ON_LineCurve_class_id == id )
      return ON::ctLine;
    if ( &ON_PolylineCurve::m_ON_PolylineCurve_class_id == id )
      return ON::ctPolyline;
    if ( &ON_CurveProxy::m_ON_CurveProxy_class_id == id )
      return ON::ctProxy;
    if ( &ON_PolyCurve::m_ON_PolyCurve_class_id == id )
      return ON::ctPolycurve;
    if ( &ON_NurbsCurve::m_ON_NurbsCurve_class_id == id )
      return ON::ctNurbs;
    if ( &ON_CurveOnSurface::m_ON_CurveOnSurface_class_id == id )
      return ON::ctOnsurface;
    id = id->BaseClass();
  }
  
  return ON::ctCurve;
}

/*
Description:
  Carefully match curve ends.
Parameters:
  curve0 - [in]
  end0 - [in] 0=match start of curve0, 1 = match end of curve0
  curve1 - [in]
  end1 - [in] 0=match start of curve1, 1 = match end of curve1
  gap_tolerance - [in]
    The match is not performed if the initial gap is <= gap_tolerance.
    If gap_tolerance < 0, then ON_ComparePoint() is used to
    compare the points.
Returns:
  True if ends of curves are matched to requested gap_tolerance.
*/
bool ON_MatchCurveEnds( ON_Curve* curve0,
                        int end0,
                        ON_Curve* curve1,
                        int end1,
                        double gap_tolerance = 0.0
                        );

bool ON_MatchCurveEnds( ON_Curve* curve0,
                        int end0,
                        ON_Curve* curve1,
                        int end1,
                        double gap_tolerance )
{
  ON_BOOL32 rc = false;
  if ( 0 != curve0 && 0 != curve1 
       && end0 >= 0 && end0 <= 1 
       && end1 >= 0 && end1 <= 1 )
  {
    ON_3dPoint P0 = end0 ? curve0->PointAtEnd() : curve0->PointAtStart();
    ON_3dPoint P1 = end1 ? curve1->PointAtEnd() : curve1->PointAtStart();
    rc = ( gap_tolerance < 0.0 )
       ? (0==ON_ComparePoint( 3, false, &P0.x, &P1.x ) )
       : (P0.DistanceTo(P1) <= gap_tolerance);
    if ( !rc )
    {
      // try to close the gap
      ON_Curve* seg[2] = {0,0};
      int fix[2] = {0,0};
      ON_3dPoint fixPoint[2];
      fixPoint[0] = ON_UNSET_POINT;
      fixPoint[1] = ON_UNSET_POINT;

      // hurestic for deciding which point gets moved
      int i;
      for ( i = 0; i <= 1; i++ )
      {
        ON_3dPoint Q0 = ON_UNSET_POINT;
        ON_3dPoint Q1 = ON_UNSET_POINT;
        ON_Curve* c = i ? curve1 : curve0;
        ON::eCurveType ct = ON_CurveType(c);
        int e = i ? end1 : end0;
        while ( ON::ctPolycurve == ct )
        {
          c->DestroyRuntimeCache();
          ON_PolyCurve* polycurve = ON_PolyCurve::Cast(c);
          if ( 0 == polycurve )
            break;
          c = polycurve->SegmentCurve(e?(polycurve->Count()-1):0);
          if( 0 == c )
            return false;
          ct = ON_CurveType(c);
        }
        seg[i] = c;
        switch(ct)
        {
        case ON::ctArc: // arc
          if ( c->IsClosed() )
            fix[i] = 200;
          else
          {
            fix[i] = 100;
          }
          Q0 = c->PointAtStart();
          Q1 = c->PointAtEnd();
          break;
        case ON::ctLine: // line
          fix[i] = 20;
          Q0 = c->PointAtStart();
          Q1 = c->PointAtEnd();
          break;
        case ON::ctPolyline: // polyline
          fix[i] = 10;
          if ( c->SpanCount() == 1 )
          {
            // same as a line
            fix[i] = 20;
            Q0 = c->PointAtStart();
            Q1 = c->PointAtEnd();
          }
          else
            fix[i] = 10;
          break;
        case ON::ctProxy: // proxy
          fix[i] = 1000; // cannot change
          break;
        case ON::ctPolycurve: // polycurve
          fix[i] = 1000; // if this happens, something is bad
          break;
        case ON::ctNurbs: // nurbs
          {
            if ( c->Degree() == 1 )
            {
              if ( c->SpanCount() == 1 )
              {
                // same as a line
                fix[i] = 20; 
                Q0 = c->PointAtStart();
                Q1 = c->PointAtEnd();
              }
              else
              {
                // same as a polyline
                fix[i] = 10;
              }
            }
            else
              fix[i] = 0;
          }
          break;
        case ON::ctOnsurface: // curve on surface
          fix[i] = 1000; // cannot change
          break;
        default: // ???
          fix[i] = 50;
          break;
        }

        int j = 0;
        if ( ON_UNSET_VALUE != Q0.x && Q0.x == Q1.x )
        {
          fixPoint[i].x = Q0.x;
          j++;
        }
        if ( ON_UNSET_VALUE != Q0.y && Q0.y == Q1.y )
        {
          fixPoint[i].y = Q0.y;
          j++;
        }
        if ( ON_UNSET_VALUE != Q0.z && Q0.z == Q1.z )
        {
          fixPoint[i].z = Q0.z;
          j++;
        }
        if ( 2 == j )
          fix[i] += 9;
        else if ( 1 == j )
          fix[i] += 1;
      }

      if ( fix[0] >= 1000 || fix[1] < fix[0] )
      {
        if ( fix[1] >= 1000 )
          return false;
        rc =  end1
           ? curve1->SetEndPoint(P0)
           : curve1->SetStartPoint(P0);
      }
      else if ( fix[1] >= 1000 || fix[0] < fix[1] )
      {
        rc =  end0
           ? curve0->SetEndPoint(P1)
           : curve0->SetStartPoint(P1);
      }
      else 
      {
        ON_3dPoint P = 0.5*(P0+P1);
        if ( P0.x == P1.x )
          P.x = P0.x;
        else if ( fixPoint[0].x != ON_UNSET_VALUE && fixPoint[1].x == ON_UNSET_VALUE )
          P.x = fixPoint[0].x;
        else if ( fixPoint[0].x == ON_UNSET_VALUE && fixPoint[1].x != ON_UNSET_VALUE )
          P.x = fixPoint[1].x;
        if ( P0.y == P1.y )
          P.y = P0.y;
        else if ( fixPoint[0].y != ON_UNSET_VALUE && fixPoint[1].y == ON_UNSET_VALUE )
          P.y = fixPoint[0].y;
        else if ( fixPoint[0].y == ON_UNSET_VALUE && fixPoint[1].y != ON_UNSET_VALUE )
          P.y = fixPoint[1].y;
        if ( P0.z == P1.z )
          P.z = P0.z;
        else if ( fixPoint[0].z != ON_UNSET_VALUE && fixPoint[1].z == ON_UNSET_VALUE )
          P.z = fixPoint[0].z;
        else if ( fixPoint[0].z == ON_UNSET_VALUE && fixPoint[1].z != ON_UNSET_VALUE )
          P.z = fixPoint[1].z;   
        ON_BOOL32 rc0 =  end0
           ? curve0->SetEndPoint(P)
           : curve0->SetStartPoint(P);
        ON_BOOL32 rc1 =  end1
           ? curve1->SetEndPoint(P)
           : curve1->SetStartPoint(P);
        rc = (rc0 && rc1);
      }
      if ( rc )
      {
        P0 = end0 ? curve0->PointAtEnd() : curve0->PointAtStart();
        P1 = end1 ? curve1->PointAtEnd() : curve1->PointAtStart();
        double d = P0.DistanceTo(P1);
        rc = ( gap_tolerance <= 0.0 ) // <= is correct here
           ? (0==ON_ComparePoint( 3, false, &P0.x, &P1.x ) )
           : (d <= gap_tolerance);
      }
    }
  }
  return rc?true:false;
}

static bool ForceMatchArcs(ON_ArcCurve& Arc0, int end0, ON_ArcCurve& Arc1, int end1)

{
  ON_3dPoint P0 = (end0) ? Arc0.PointAtEnd() : Arc0.PointAtStart();
  ON_3dPoint P1 = (end1) ? Arc1.PointAtEnd() : Arc1.PointAtStart();
  ON_3dPoint P = 0.5*(P0+P1);

  bool bMove[2] = {true,true};
  bool rc = true;

  if (bMove[0]){
    ON_BOOL32 brc = (end0) ? Arc0.SetEndPoint(P) : Arc0.SetStartPoint(P);
    if (!brc)
      rc = false;
  }
  if (bMove[1]){
    ON_BOOL32 brc = (end1) ? Arc1.SetEndPoint(P) : Arc1.SetStartPoint(P);
    if (!brc)
      rc = false;
  }

  return rc;


}

bool ON_ForceMatchCurveEnds(ON_Curve& Crv0, int end0, ON_Curve& Crv1, int end1)

{

  if (end0)
    end0 = 1;
  if (end1)
    end1 = 1;

  int i;
  ON::eCurveType ct[2];
  ON_Curve* seg[2];
  bool bIsArc[2];
  for ( i = 0; i<2; i++ )
  {
    ON_Curve* c = i ? &Crv1 : &Crv0;
    ct[i] = ON_CurveType(c);
    int e = i ? end1 : end0;
    while ( ON::ctPolycurve == ct[i] )
    {
      c->DestroyRuntimeCache();
      ON_PolyCurve* polycurve = ON_PolyCurve::Cast(c);
      if ( 0 == polycurve )
        break;
      c = polycurve->SegmentCurve(e?(polycurve->Count()-1):0);
      if( 0 == c )
        return false;
      ct[i] = ON_CurveType(c);
    }
    if (c->IsClosed())
      return false;
    if (ct[i] == ON::ctArc)
      bIsArc[i] = true;
    else {
      if (ct[i] == ON::ctLine || ct[i] == ON::ctNurbs || ct[i] == ON::ctPolyline)
        bIsArc[i] = false;
      else
        return false;
    }
    seg[i] = c;
  }

  ON_3dPoint P;
  bool bMove[2] = {true,true};
  if (bIsArc[0]){
    if (!bIsArc[1]){
      P = (end0) ? seg[0]->PointAtEnd() : seg[0]->PointAtStart();
      bMove[0] = false;
    }
    else {
      ON_ArcCurve* pArc0 = ON_ArcCurve::Cast(seg[0]);
      if (!pArc0)
        return false;
      ON_ArcCurve* pArc1 = ON_ArcCurve::Cast(seg[1]);
      if (!pArc1)
        return false;
      return ForceMatchArcs(*pArc0, end0, *pArc1, end1);
    }
  }
  else {
    if (bIsArc[1]){
      P = (end1) ? seg[1]->PointAtEnd() : seg[1]->PointAtStart();
      bMove[1] = false;
    }
    else {
      ON_3dPoint P0 = (end0) ? seg[0]->PointAtEnd() : seg[0]->PointAtStart();
      ON_3dPoint P1 = (end1) ? seg[1]->PointAtEnd() : seg[1]->PointAtStart();
      P = 0.5*(P0+P1);
    }
  }

  bool rc = true;
  if (bMove[0]){
    ON_BOOL32 brc = (end0) ? seg[0]->SetEndPoint(P) : seg[0]->SetStartPoint(P);
    if (!brc)
      rc = false;
  }
  if (bMove[1]){
    ON_BOOL32 brc = (end1) ? seg[1]->SetEndPoint(P) : seg[1]->SetStartPoint(P);
    if (!brc)
      rc = false;
  }

  return rc;
}

bool ON_NurbsCurve::RepairBadKnots( double knot_tolerance, bool bRepair )
{
  bool rc = false;
  if ( m_order >= 2 && m_cv_count > m_order
       && 0 != m_cv && 0 != m_knot 
       && m_dim > 0
       && m_cv_stride >= (m_is_rat)?(m_dim+1):m_dim
       && 0 != m_cv
       && 0 != m_knot
       && m_knot[m_cv_count-1] - m_knot[m_order-2] > knot_tolerance
       )
  {
    // save domain so it does not change
    ON_Interval domain = Domain();
    //const int cv_count0 = m_cv_count;

    const int sizeof_cv = CVSize()*sizeof(*m_cv);
    //int knot_count = KnotCount();
    int i, j0, j1;

    ON_BOOL32 bIsPeriodic = IsPeriodic();

    if ( !bIsPeriodic )
    {
      if ( m_knot[0] != m_knot[m_order-2] || m_knot[m_cv_count-1] != m_knot[m_cv_count+m_order-3] )
      {
        rc = true;
        if ( bRepair )
          ClampEnd(2);
        else
          return rc;
      }
    }

    // make sure last span has m_knot[m_cv_count-1] - m_knot[m_cv_count-2] > knot_tolerance
    for ( i = m_cv_count-2; i > m_order-2; i-- )
    {
      if ( m_knot[m_cv_count-1] - m_knot[i] > knot_tolerance )
      {
        if ( i < m_cv_count-2 )
        {
          rc = true;
          if ( bRepair )
          {
            // remove extra knots but do not change end point location
            DestroyRuntimeCache();
            double* cv = (double*)onmalloc(sizeof_cv);
            ClampEnd(2);
            memcpy( cv, CV(m_cv_count-1), sizeof_cv );
            m_cv_count = i+2;
            ClampEnd(2);
            memcpy( CV(m_cv_count-1), cv, sizeof_cv );
            for ( i = m_cv_count-1; i < m_cv_count+m_order-2; i++ )
              m_knot[i] = domain[1];
            onfree(cv);
            cv = 0;
          }
          else
            return rc;
        }
        break; // there at least one valid span
      }
    }

    // make sure first span has m_knot[m_order-1] - m_knot[m_order-2] > knot_tolerance
    for ( i = m_order-1; i < m_cv_count-1; i++ )
    {
      if ( m_knot[i] - m_knot[m_order-2] > knot_tolerance )
      {
        if ( i > m_order-1 )
        {
          rc = true;
          if ( bRepair )
          {
            // remove extra knots but do not change end point location
            DestroyRuntimeCache();
            i -= (m_order-1);
            double* cv = (double*)onmalloc(sizeof_cv);
            ClampEnd(2);
            memcpy(cv,CV(0),sizeof_cv);
            for ( j0 = 0, j1 = i; j1 < m_cv_count; j0++, j1++ )
              memcpy(CV(j0),CV(j1),sizeof_cv);
            for ( j0 = 0, j1 = i; j1 < m_cv_count+m_order-2; j0++, j1++ )
              m_knot[j0] = m_knot[j1];
            m_cv_count -= i;
            ClampEnd(2);
            memcpy( CV(0), cv, sizeof_cv );
            for ( i = 0; i <= m_order-2; i++ )
              m_knot[i] = domain[0];
            onfree(cv);
            cv = 0;
          }
          else
            return rc;
        }
        break; // there at least one valid span
      }
    }


    if (    m_knot[m_order-1]-m_knot[m_order-2] > knot_tolerance 
         && m_knot[m_cv_count-1]-m_knot[m_cv_count-2] > knot_tolerance )
    {
      // Remove interior knots with mulitiplicity >= m_order
      for ( i = m_cv_count-m_order-1; i >= m_order; i-- )
      {
        if ( m_knot[i+m_order-1] - m_knot[i] <= knot_tolerance )
        {
          rc = true;
          if ( bRepair )
          {
            // empty evaluation span - remove CV and knot
            DestroyRuntimeCache();
            for ( j0 = i, j1 = i+1; j1 < m_cv_count; j0++, j1++ )
              memcpy( CV(j0), CV(j1), sizeof_cv );
            for ( j0 = i, j1 = i+1; j1 < m_cv_count+m_order-2; j0++, j1++ )
              m_knot[j0] = m_knot[j1];
            m_cv_count--;
          }
          else
          {
            // query mode
            return rc;
          }
        }
      }
    }

    if ( bRepair && bIsPeriodic && rc )
    {
      if ( !IsPeriodic() )
        ClampEnd(2);
    }
  }
  return rc;
}



bool ON_Curve::FirstSpanIsLinear( 
  double min_length,
  double tolerance
  ) const
{
  return FirstSpanIsLinear(min_length,tolerance,0);
}

bool ON_Curve::FirstSpanIsLinear( 
  double min_length,
  double tolerance,
  ON_Line* span_line
  ) const
{
  const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(this);
  if ( 0 != nurbs_curve )
  {
    return nurbs_curve->SpanIsLinear(0,min_length,tolerance,span_line);
  }

  const ON_PolylineCurve* polyline_curve = ON_PolylineCurve::Cast(this);
  if ( 0 != polyline_curve )
  {
    bool rc = (polyline_curve->PointCount() >= 2);
    if (rc && 0 != span_line )
    {
      span_line->from = polyline_curve->m_pline[0];
      span_line->to = polyline_curve->m_pline[1];
    }
    return rc;
  }

  const ON_LineCurve* line_curve = ON_LineCurve::Cast(this);
  if ( 0 != line_curve )
  {
    if ( span_line )
      *span_line = line_curve->m_line;
    return true;
  }

  const ON_PolyCurve* poly_curve = ON_PolyCurve::Cast(this);
  if ( 0 != poly_curve )
  {
    const ON_Curve* segment = poly_curve->SegmentCurve(0);
    return (0 != segment) ? segment->FirstSpanIsLinear(min_length,tolerance,span_line) : false;
  }

  const ON_CurveProxy* proxy_curve = ON_CurveProxy::Cast(this);
  if ( 0 != proxy_curve )
  {
    const ON_Curve* curve = proxy_curve->ProxyCurve();
    if ( 0 == curve )
      return false;
    bool bProxyCurveIsReversed = proxy_curve->ProxyCurveIsReversed();
    bool rc = bProxyCurveIsReversed
            ? curve->FirstSpanIsLinear(min_length,tolerance,span_line) 
            : curve->LastSpanIsLinear(min_length,tolerance,span_line);
    if ( rc && bProxyCurveIsReversed && 0 != span_line )
      span_line->Reverse();
    return rc;
  }

  return false;
}



bool ON_Curve::LastSpanIsLinear( 
  double min_length,
  double tolerance
  ) const
{
  return LastSpanIsLinear(min_length,tolerance,0);
}

bool ON_Curve::LastSpanIsLinear( 
  double min_length,
  double tolerance,
  ON_Line* span_line
  ) const
{
  const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(this);
  if ( 0 != nurbs_curve )
  {
    return nurbs_curve->SpanIsLinear(nurbs_curve->m_cv_count-nurbs_curve->m_order,min_length,tolerance,span_line);
  }

  const ON_PolylineCurve* polyline_curve = ON_PolylineCurve::Cast(this);
  if ( 0 != polyline_curve )
  {
    int count = polyline_curve->PointCount();
    if ( count >= 2 && 0 != span_line )
    {
      span_line->from = polyline_curve->m_pline[count-2];
      span_line->to = polyline_curve->m_pline[count-1];
    }
    return ( count >= 2);
  }

  const ON_LineCurve* line_curve = ON_LineCurve::Cast(this);
  if ( 0 != line_curve )
  {
    if ( span_line )
      *span_line = line_curve->m_line;
    return true;
  }

  const ON_PolyCurve* poly_curve = ON_PolyCurve::Cast(this);
  if ( 0 != poly_curve )
  {
    const ON_Curve* segment = poly_curve->SegmentCurve(poly_curve->Count()-1);
    return (0 != segment) ? segment->LastSpanIsLinear(min_length,tolerance,span_line) : false;
  }

  const ON_CurveProxy* proxy_curve = ON_CurveProxy::Cast(this);
  if ( 0 != proxy_curve )
  {
    const ON_Curve* curve = proxy_curve->ProxyCurve();
    if ( 0 == curve )
      return false;
    bool bProxyCurveIsReversed = proxy_curve->ProxyCurveIsReversed();
    bool rc = bProxyCurveIsReversed
           ? curve->LastSpanIsLinear(min_length,tolerance,span_line) 
           : curve->FirstSpanIsLinear(min_length,tolerance,span_line);
    if ( rc && bProxyCurveIsReversed && 0 != span_line )
      span_line->Reverse();
    return rc;
  }

  return false;
}



bool ON_NurbsCurve::SpanIsLinear(
    int span_index, 
    double min_length,
    double tolerance
    ) const
{
  return SpanIsLinear(span_index,min_length,tolerance,0);
}

bool ON_NurbsCurve::SpanIsLinear(
    int span_index, 
    double min_length,
    double tolerance,
    ON_Line* span_line
    ) const
{
  if ( m_dim < 2 || m_dim > 3 )
    return false;

  if ( -1 == span_index && m_cv_count-m_order+1+span_index >= 0 )
  {
    // negative span indices work from the back
    span_index += m_cv_count-m_order+1;
  }
  else if ( span_index < 0 || span_index > m_cv_count-m_order )
  {
    ON_ERROR("span_index out of range.");
    return false;
  }

  if ( !(m_knot[span_index+m_order-2] < m_knot[span_index+m_order-1]) )
  {
    // empty span
    ON_ERROR("empty span.");
    return false;
  }

  if (    m_knot[span_index] == m_knot[span_index+m_order-2]
       && m_knot[span_index+m_order-1] == m_knot[span_index+2*m_order-3]
     )
  {
    ON_3dPoint P, Q;
    ON_Line line;
    const int i1 = span_index+m_order-1;
    if ( !GetCV(span_index,line.from) )
      return false;
    if ( !GetCV(i1,line.to) )
      return false;
    if ( !(line.Length() >= min_length) ) 
      return false;
    double t0, t, d;
    t0 = t = 0.0;
    for ( int i = span_index+1; i < i1; i++ )
    {
      if ( !GetCV(i,P) )
        return false;
      if ( !line.ClosestPointTo( P, &t ) )
        return false;
      if ( !(t0 < t) )
        return false;
			if ( !(t <= 1.0 + ON_SQRT_EPSILON) )
				return false;
      Q = line.PointAt(t);
      if ( false == ON_PointsAreCoincident(3,0,&P.x,&Q.x) )
      {
        d = P.DistanceTo( line.PointAt(t) );
        if ( !(d <= tolerance) )
          return false;
      }
      t0 = t;
    }
    if ( span_line )
      *span_line = line;
    return true;
  }

  return false;
}

ON_BOOL32 ON_Curve::Trim( const ON_Interval& in )
{
  // TODO - make this pure virtual
  return false;
}


bool ON_Curve::Extend(
  const ON_Interval& domain
  )

{
  return false;
}


ON_Curve* ON_TrimCurve( 
            const ON_Curve& curve,
            ON_Interval trim_parameters
            )
{
  ON_Curve* trimmed_curve = 0;

  const ON_Interval curve_domain = curve.Domain();
  ON_BOOL32 bDecreasing = trim_parameters.IsDecreasing();
  trim_parameters.Intersection( curve_domain ); // trim_parameters will be increasing or empty
  if ( bDecreasing )
  {
    trim_parameters.Swap();
    if ( trim_parameters[0] == curve_domain[1] )
    {
      if ( trim_parameters[1] == curve_domain[0] )
        return 0;
      trim_parameters[0] = curve_domain[0];
    }
    else if ( trim_parameters[1] == curve_domain[0] )
      trim_parameters[1] = curve_domain[1];
    else if ( !trim_parameters.IsDecreasing() )
      return 0;
  }

  if ( trim_parameters.IsDecreasing() && curve.IsClosed() )
  {
    ON_Curve* left_crv = curve.DuplicateCurve();
    if ( !left_crv->Trim(ON_Interval(trim_parameters[0],curve_domain[1])) )
    {
      delete left_crv;
      return 0;
    }
    ON_Curve* right_crv = curve.DuplicateCurve();
    if ( !right_crv->Trim(ON_Interval(curve_domain[0],trim_parameters[1])) )
    {
      delete left_crv;
      delete right_crv;
      return 0;
    }
    ON_PolyCurve* polycurve = ON_PolyCurve::Cast(left_crv);
    if ( polycurve == NULL )
    {
      polycurve = new ON_PolyCurve();      
      polycurve->Append( left_crv );
    }

    ON_PolyCurve* ptmp = ON_PolyCurve::Cast(right_crv);
    if ( ptmp )
    {
      int i;
      for ( i = 0; i < ptmp->Count(); i++ )
      {
        ON_Interval sdom = ptmp->SegmentDomain(i);
        ON_Curve* segment = ptmp->HarvestSegment(i);
        segment->SetDomain(sdom[0],sdom[1]); // to keep relative parameterization unchanged
        polycurve->Append( segment );
      }
      delete right_crv;
      ptmp = 0;
      right_crv = 0;
    }
    else
    {
      polycurve->Append( right_crv );
    }

    polycurve->SetDomain( trim_parameters[0], trim_parameters[1] + curve_domain.Length() );

    trimmed_curve = polycurve;
  }
  else if ( trim_parameters.IsIncreasing() )
  {
    trimmed_curve = curve.DuplicateCurve();
    if( !trimmed_curve->Trim(trim_parameters) )
    {
      delete trimmed_curve;
      trimmed_curve = 0;
    }
  }

  return trimmed_curve;
}

ON_BOOL32 ON_Curve::Split(
    double, // t = curve parameter to split curve at
    ON_Curve*&, // left portion returned here (can pass "this" as the pointer)
    ON_Curve*&  // right portion returned here (can pass "this" as the pointer)
  ) const
{
  // override if curve can split itself
  return false;
}

// virtual
int ON_Curve::GetNurbForm(
      ON_NurbsCurve& nurbs_curve,
      double tolerance,
      const ON_Interval* subdomain
      ) const
{
  return 0;
}

// virtual
int ON_Curve::HasNurbForm() const
{
  return 0;
}

ON_NurbsCurve* ON_Curve::NurbsCurve(
      ON_NurbsCurve* pNurbsCurve,
      double tolerance,
      const ON_Interval* subdomain
      ) const
{
  ON_NurbsCurve* nurbs_curve = pNurbsCurve;
  if ( !nurbs_curve )
    nurbs_curve = new ON_NurbsCurve();
  int rc = GetNurbForm( *nurbs_curve, tolerance, subdomain );
  if ( !rc )
  {
    if (!pNurbsCurve)
      delete nurbs_curve;
    nurbs_curve = NULL;
  }
  return nurbs_curve;
}

ON_BOOL32 ON_Curve::GetCurveParameterFromNurbFormParameter(
      double nurbs_t,
      double* curve_t
      ) const
{
  *curve_t = nurbs_t;
  return false;
}

ON_BOOL32 ON_Curve::GetNurbFormParameterFromCurveParameter(
      double curve_t,
      double* nurbs_t
      ) const
{
  *nurbs_t = curve_t;
  return false;
}

ON_CurveArray::ON_CurveArray( int initial_capacity ) 
              : ON_SimpleArray<ON_Curve*>(initial_capacity)
{}

ON_CurveArray::~ON_CurveArray()
{
  Destroy();
}

void ON_CurveArray::Destroy()
{
  int i = m_capacity;
  while ( i-- > 0 ) {
    if ( m_a[i] ) {
      delete m_a[i];
      m_a[i] = NULL;
    }
  }
  Empty();
}

bool ON_CurveArray::Duplicate( ON_CurveArray& dst ) const
{
  dst.Destroy();
  dst.SetCapacity( Capacity() );

  const int count = Count();
  int i;
  ON_Curve* curve;
  for ( i = 0; i < count; i++ ) 
  {
    curve = 0;
    if ( m_a[i] ) 
    {
      curve = m_a[i]->Duplicate();
    }
    dst.Append(curve);      
  }
  return true;
}

bool ON_CurveArray::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 0 );
  if (rc) rc = file.Write3dmChunkVersion(1,0);
  if (rc ) {
    int i;
    rc = file.WriteInt( Count() );
    for ( i = 0; rc && i < Count(); i++ ) {
      if ( m_a[i] ) {
        rc = file.WriteInt(1);
        if ( rc ) 
          rc = file.WriteObject( *m_a[i] ); // polymorphic curves
      }
      else {
        // NULL curve
        rc = file.WriteInt(0);
      }
    }
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}


bool ON_CurveArray::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  int flag;
  Destroy();
  bool rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
  if (rc) 
  {
    rc = ( tcode == TCODE_ANONYMOUS_CHUNK );
    if (rc) rc = file.Read3dmChunkVersion(&major_version,&minor_version);
    if (rc && major_version == 1) 
    {
      ON_Object* p;
      int count;
      rc = file.ReadInt( &count );
      if (rc) 
      {
        SetCapacity(count);
        SetCount(count);
        Zero();
        int i;
        for ( i = 0; rc && i < count && rc; i++ ) 
        {
          flag = 0;
          rc = file.ReadInt(&flag);
          if (rc && flag==1) 
          {
            p = 0;
            rc = file.ReadObject( &p ) ? true : false; // polymorphic curves
            m_a[i] = ON_Curve::Cast(p);
            if ( !m_a[i] )
              delete p;
          }
        }
      }
    }
    else 
    {
      rc = false;
    }
    if ( !file.EndRead3dmChunk() )
    {
      rc = false;
    }
  }
  return rc;
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////// utilities for curve joining ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


struct CurveJoinSeg {
  int id;
  bool bRev;
};

//distance from curve[id[0] end[end[0]] to curve[id[1]] end[end[1]] is dist.
struct CurveJoinEndData {
  int id[2];  //index into array of curves
  int end[2]; //0 for start, 1 for end
  double dist;
};


static int CompareEndData(const CurveJoinEndData* a, const CurveJoinEndData* b)

{
  if (a->dist < b->dist) return -1;
  if (a->dist > b->dist) return 1;
  return 0;
}

static void ReverseSegs(ON_SimpleArray<CurveJoinSeg>& SArray)

{
  int i;
  for (i=0; i<SArray.Count(); i++)
    SArray[i].bRev = !SArray[i].bRev;
  SArray.Reverse();
  return;
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////// end of utilities for curve joining ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


bool ON_Curve::IsClosable(double tolerance, 
                          double min_abs_size, //0.0
                          double min_rel_size  //10.0
                          ) const

{
  if (IsClosed()) return true;
  if (Degree() + SpanCount() < 4) return false;

  ON_3dPoint P[6];

  P[0] = PointAtStart();
  P[5] = PointAtEnd();

  double gap = P[0].DistanceTo(P[5]);
  if (gap > tolerance) return false;

  bool abs_ok = (min_abs_size < 0.0) ? true : false;
  bool rel_ok = (min_rel_size <= 1.0) ? true : false;
  bool ok = abs_ok && rel_ok;

  if (!ok){

    //make sure curve is long enough to close off.
    int i;
    double len = 0.0;

    for (i=1; i<6; i++){
      if (i!=5)
        P[i] = PointAt(Domain().ParameterAt(0.2*i));
      if (!abs_ok && P[i].DistanceTo(P[0]) > min_abs_size)
        abs_ok = true;
      if (!rel_ok){
        len += P[i-1].DistanceTo(P[i]);
        if (len >= min_rel_size*gap)
          rel_ok = true;
      }
      ok = abs_ok && rel_ok;
      if (ok) break;
    }
  }

  return ok;
}

//Makes continuous ON_PolyCurves and ON_Polycurves. Appends results to OutCurves.
//returns number of curves appended.



int 
ON_JoinCurves(const ON_SimpleArray<const ON_Curve*>& InCurves,
                      ON_SimpleArray<ON_Curve*>& OutCurves,
                      double join_tol,
                      bool bPreserveDirection, // = false
                      ON_SimpleArray<int>* key //=0
                      )

{

  int i, count = OutCurves.Count();
  if (InCurves.Count() < 1)
    return 0;

  int dim = InCurves[0]->Dimension();
  for (i=1; i<InCurves.Count(); i++){
    if (InCurves[i]->Dimension() != dim) return 0;
  }

  if (key) {
    key->Reserve(InCurves.Count());
    for (i=0; i<InCurves.Count(); i++) key->Append(-1);
  }

  //Copy curves, take out closed curves. 
  OutCurves.Reserve(InCurves.Count());
  ON_SimpleArray<ON_Curve*> IC(InCurves.Count());
  ON_SimpleArray<int> cmap(InCurves.Count());
  for (i=0; i<InCurves.Count(); i++){
    ON_Curve* C = InCurves[i]->DuplicateCurve();
    if (!C) continue;
    if (C->IsClosed()) {
      if (key) (*key)[i] = OutCurves.Count();
      OutCurves.Append(C);
    }
    else {
      cmap.Append(i);
      IC.Append(C);
    }
  }

  //IC is a list of copies of all open curves.  match endpoints and join into polycurves.
  //copy curves that are not joined.
  ON_3dPointArray Start(IC.Count());
  Start.SetCount(IC.Count());
  ON_3dPointArray End(IC.Count());
  End.SetCount(IC.Count());
  for (i=0; i<IC.Count(); i++){
    Start[i] = IC[i]->PointAtStart();
    End[i] = IC[i]->PointAtEnd();
  }

  //get a list of all possible joins
  ON_SimpleArray<CurveJoinEndData> EData(IC.Count());
  for (i=0; i<IC.Count(); i++){
    int j;
    for (j=0; j<IC.Count(); j++){
      if (i==j) continue;
      double dist = Start[i].DistanceTo(End[j]);
      if (dist <= join_tol){
        CurveJoinEndData& ED = EData.AppendNew();
        ED.id[0] = i;
        ED.end[0] = 0;
        ED.id[1] = j;
        ED.end[1] = 1;
        ED.dist = dist;
      }
      if (!bPreserveDirection && i<j){
        dist = Start[i].DistanceTo(Start[j]);
        if (dist <= join_tol){
          CurveJoinEndData& ED = EData.AppendNew();
          ED.id[0] = i;
          ED.end[0] = 0;
          ED.id[1] = j;
          ED.end[1] = 0;
          ED.dist = dist;
        }
        dist = End[i].DistanceTo(End[j]);
        if (dist <= join_tol){
          CurveJoinEndData& ED = EData.AppendNew();
          ED.id[0] = i;
          ED.end[0] = 1;
          ED.id[1] = j;
          ED.end[1] = 1;
          ED.dist = dist;
        }
      }
    }
  }

  //sort possiblities by distance
  EData.QuickSort(CompareEndData);

  int* endspace = (int*)onmalloc(2*sizeof(int)*IC.Count());
  memset(endspace, 0, 2*sizeof(int)*IC.Count());
  int** endarray = (int**)onmalloc(sizeof(int*)*IC.Count());

  //endarray[i] is an int[2].  if endarray[i][0] > 0, then IC[i] is part of
  //polycurve endarray[i][0] - 1, and the start of IC[i] is interior to the polycurve.
  //if endarray[i][1] > 0, then the end of IC[i] is interior.  if both endarray[i][0] > 0
  //and endarray[i][1] > 0, then they are equal.

  for (i=0; i<IC.Count(); i++)
    endarray[i] = &endspace[2*i];

  ON_ClassArray<ON_SimpleArray<CurveJoinSeg> > SegsArray(IC.Count());

  for (i=0; i<EData.Count(); i++){
    const CurveJoinEndData& ED = EData[i];
    if (endarray[ED.id[0]][ED.end[0]] > 0 || endarray[ED.id[1]][ED.end[1]] > 0)
      continue; //one of these endpoints has already been join to a closer end
    if (endarray[ED.id[0]][1 - ED.end[0]] == 0){
      if (endarray[ED.id[1]][1 - ED.end[1]] == 0){//new curve
        endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = SegsArray.Count()+1;
        ON_SimpleArray<CurveJoinSeg>& SArray = SegsArray.AppendNew();
        SArray.Reserve(4);
        CurveJoinSeg& Seg0 = SArray.AppendNew();
        CurveJoinSeg& Seg1 = SArray.AppendNew();
        if (ED.end[0]) {
          Seg0.id = ED.id[0];
          Seg0.bRev = false;
          Seg1.id = ED.id[1];
          Seg1.bRev = (ED.end[1]) ? true : false;
        }
        else {
          Seg1.id = ED.id[0];
          Seg1.bRev = false;
          Seg0.id = ED.id[1];
          Seg0.bRev = (ED.end[1]) ? false : true;
        }
      }

      else {

        //second curve is part of an existing sequence. Insert or append first curve.
        ON_SimpleArray<CurveJoinSeg>& SArray = SegsArray[endarray[ED.id[1]][1 - ED.end[1]] - 1];
        endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = 
          endarray[ED.id[1]][1 - ED.end[1]];

        if (SArray[0].id == ED.id[1]){
          CurveJoinSeg Seg;
          Seg.id = ED.id[0];
          Seg.bRev = (ED.end[0]) ? false : true;
          SArray.Insert(0, Seg);
        }
        else {
          CurveJoinSeg& Seg = SArray.AppendNew();
          Seg.id = ED.id[0];
          Seg.bRev = (ED.end[0]) ? true : false;
        }
      }
    }
    else if (endarray[ED.id[1]][1 - ED.end[1]] == 0){
    //first curve is part of an existing sequence. Insert or append second curve.
      ON_SimpleArray<CurveJoinSeg>& SArray = SegsArray[endarray[ED.id[0]][1 - ED.end[0]] - 1];
      endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = 
        endarray[ED.id[0]][1 - ED.end[0]];

      if (SArray[0].id == ED.id[0]){
        CurveJoinSeg Seg;
        Seg.id = ED.id[1];
        Seg.bRev = (ED.end[1]) ? false : true;
        SArray.Insert(0, Seg);
      }
      else {
        CurveJoinSeg& Seg = SArray.AppendNew();
        Seg.id = ED.id[1];
        Seg.bRev = (ED.end[1]) ? true : false;
      }
    }
    else {
      //both are in existing sequences.  join the sequences.
      if (endarray[ED.id[0]][1 - ED.end[0]] == endarray[ED.id[1]][1 - ED.end[1]])
        //closes off this curve
        endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = 
          endarray[ED.id[0]][1 - ED.end[0]];
      else {
        int segid0 = endarray[ED.id[0]][1 - ED.end[0]];
        int segid1 = endarray[ED.id[1]][1 - ED.end[1]];
        ON_SimpleArray<CurveJoinSeg>& SArray0 = SegsArray[segid0 - 1];
        ON_SimpleArray<CurveJoinSeg>& SArray1 = SegsArray[segid1 - 1];
        if (SArray0[0].id == ED.id[0]){
          if (SArray1[0].id == ED.id[1]){
            ReverseSegs(SArray0);
            int j;
            for (j=0; j<SArray1.Count(); j++){
              if (endarray[SArray1[j].id][0] > 0) endarray[SArray1[j].id][0] = segid0;
              if (endarray[SArray1[j].id][1] > 0) endarray[SArray1[j].id][1] = segid0;
              SArray0.Append(SArray1[j]);
            }
            endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = segid0;
            SArray1.SetCount(0);
          }
          else {
            int j;
            for (j=0; j<SArray0.Count(); j++){
              if (endarray[SArray0[j].id][0] > 0) endarray[SArray0[j].id][0] = segid1;
              if (endarray[SArray0[j].id][1] > 0) endarray[SArray0[j].id][1] = segid1;
              SArray1.Append(SArray0[j]);
            }
            endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = segid1;
            SArray0.SetCount(0);
          }
        }
        else if (SArray1[0].id == ED.id[1]){
          int j;
          for (j=0; j<SArray1.Count(); j++){
            if (endarray[SArray1[j].id][0] > 0) endarray[SArray1[j].id][0] = segid0;
            if (endarray[SArray1[j].id][1] > 0) endarray[SArray1[j].id][1] = segid0;
            SArray0.Append(SArray1[j]);
          }
          endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = segid0;
          SArray1.SetCount(0);
        }
        else {
          ReverseSegs(SArray1);
          int j;
          for (j=0; j<SArray1.Count(); j++) {
            if (endarray[SArray1[j].id][0] > 0) endarray[SArray1[j].id][0] = segid0;
            if (endarray[SArray1[j].id][1] > 0) endarray[SArray1[j].id][1] = segid0;
            SArray0.Append(SArray1[j]);
          }
          endarray[ED.id[0]][ED.end[0]] = endarray[ED.id[1]][ED.end[1]] = segid0;
          SArray1.SetCount(0);
        }
      }
    }
  }

  //make polycurves out of sequences

  for (i=0; i<SegsArray.Count(); i++){
    ON_SimpleArray<CurveJoinSeg>& SArray = SegsArray[i];
    if (SArray.Count() < 2) continue;
    if (!bPreserveDirection){//if number of reversed segs is more than half, reverse.
      int count= 0;
      int j;
      for (j=0; j<SArray.Count(); j++) {
        if (SArray[j].bRev)
          count++;
      }
      if (2*count > SArray.Count())
        ReverseSegs(SArray);
    }
    ON_PolyCurve* PC = new ON_PolyCurve(SArray.Count());
    bool pc_added = false;
    int j;
    int min_seg = 0;
    int min_id = -1;
    for (j=0; j<SArray.Count(); j++){
      if (key) 
        (*key)[cmap[SArray[j].id]] = OutCurves.Count();
      ON_Curve* C = IC[SArray[j].id];
      if (min_id < 0 || SArray[j].id < min_id){
        min_id = SArray[j].id;
        min_seg = j;
      }
      if (SArray[j].bRev) C->Reverse();
      if (PC->Count()){
        bool bSet = true;
        if (!ON_ForceMatchCurveEnds(*PC, 1, *C, 0)) {
          ON_3dPoint P = PC->PointAtEnd();
          ON_3dPoint Q = C->PointAtStart();
          P = 0.5*(P+Q);
          if (!PC->SetEndPoint(P) || !C->SetStartPoint(P)) {
            ON_NurbsCurve* NC = C->NurbsCurve();
            if (NC && NC->SetStartPoint(P)){
              delete C;
              C = NC;
            }
            else {
              bSet = false;
              delete NC;
              if (PC->Count()) {
                pc_added = true;
                OutCurves.Append(PC);
              }
              if (key)
                (*key)[cmap[SArray[j].id]]++;
              OutCurves.Append(C);
              int k;
              for (k=j+1; k<SArray.Count(); k++){
                if (key)
                  (*key)[cmap[SArray[k].id]] = OutCurves.Count();
                OutCurves.Append(IC[SArray[k].id]);
              }
              break;
            }
          }
        }
      }
      ON_PolyCurve* pPoly = ON_PolyCurve::Cast(C);
      if( pPoly){
        int si;
        for (si=0; si<pPoly->Count(); si++){
          const ON_Curve* SC = pPoly->SegmentCurve(si);
          ON_Curve* SCCopy = SC->DuplicateCurve();
          if (SCCopy) PC->Append(SCCopy);
        }
        delete pPoly;
      }
      else PC->Append(C);
    }
    if (!PC->Count()) delete PC;
    else if (!pc_added) {
      if (!PC->IsClosed() && PC->IsClosable(join_tol)){
        if (!ON_ForceMatchCurveEnds(*PC, 0, *PC, 1))
          PC->SetEndPoint(PC->PointAtStart());
      }
      if (PC->IsClosed() && min_id >= 0){
        //int sc = PC->SpanCount();
        double t = PC->SegmentDomain(min_seg)[0];
        PC->ChangeClosedCurveSeam(t);
      }

      // 28 October 2010 Dale Lear
      //    I added the RemoveNesting() and SynchronizeSegmentDomains()
      //    lines so Rhino will create higher quality geometry.
      PC->RemoveNesting();
      PC->SynchronizeSegmentDomains();

      OutCurves.Append(PC);
    }
  }

  //add in singletons
  for (i=0; i<IC.Count(); i++){
    if (endarray[i][0] == 0 && endarray[i][1] == 0){
      if (key) (*key)[cmap[i]] = OutCurves.Count();
      OutCurves.Append(IC[i]);
    }
  }

	/* This was added by greg to fix big curves that are nearly, but not quite, closed.
     It causes problems when the curve is tiny.
	for(i=0; i<OutCurves.Count(); i++){
		if(!OutCurves[i]->IsClosed()){
			ON_3dPoint s= OutCurves[i]->PointAtStart();
			ON_3dPoint e = OutCurves[i]->PointAtEnd();
			if(s.DistanceTo(e)<join_tol)
				OutCurves[i]->SetEndPoint( s );			
		}
	}
  */

  //Chuck added this, 1/16/03.
  for(i=0; i<OutCurves.Count(); i++){
    ON_Curve* C = OutCurves[i];
    if (!C || C->IsClosed()) continue;
    if (C->IsClosable(join_tol))
      C->SetEndPoint(C->PointAtStart());
  }
    
  onfree((void*)endarray);
  onfree((void*)endspace);

  return OutCurves.Count() - count;
}

// returns true if t is sufficiently close to m_t[index]
// -1 <= index <= m_t.Count()
bool ON_Curve::ParameterSearch(double t, int& index, bool bEnableSnap,
																const ON_SimpleArray<double>& m_t, double RelTol) const{

  // 24 October 2003 Dale Lear - added comments and fixed bugs when t < m_t[0]
  //    If you make changes to this code, please discuss them with Dale Lear.

	bool rc = false;
	int count = m_t.Count();
	ON_Interval c_dom = Domain();
	index = -1;
	if(count>1 && ON_IsValid(t))
  {
		
    index = ON_SearchMonotoneArray(m_t, count, t);
    // index < 0       : means t < m_t[0]
    // index == count-1: means t == m_t[count-1]
    // index == count  : means t > m_t[count-1]

		rc  = (index>=0 && index<=count-1  && t == m_t[index]);
		if( bEnableSnap && !rc)
    {
      // see if t is within "ktol" of a value in m_t[]
			double ktol = RelTol*( ON_Max(fabs(c_dom[0]) ,fabs(c_dom[1])));

      if (index >= 0 && index < count-1 )
      {
        // If we get here, then m_t[index] < t < m_t[index+1]
        double middle_t = 0.5*(m_t[index] + m_t[index+1]);
			  if( t < middle_t && t - m_t[index] <= ktol)
        {
          // t is a hair bigger than m_t[index]
					rc = true;
			  } 
        else if( t > middle_t && m_t[index+1]-t <= ktol)
        {
          // t is a hair smaller than m_t[index+1]
					rc = true;
					index ++;
			  }	
      }
      else if (index == count)
      {
        // If we get here, then t > m_t[count-1]
				if( t-m_t[count-1]<=ktol)
        {
          // t is a hair bigger than m_t[count-1]
					rc = true;
					index = count-1;
				}
			}
      else if (index<0)
      {
        // 22 October 2003 Dale Lear - added this case to match the index==count case above.
        // If we get here, then t < m_t[0]
				if( m_t[0]-t <= ktol)
        {
          // t is a hair smaller than m_t[count-1]
					rc = true;
					index = 0;
				}
			}
		}
	}
	return rc;
}

bool ON_SortLines( 
        int line_count, 
        const ON_Line* line_list, 
        int* index, 
        bool* bReverse 
        )
{
  ON_3dPoint StartP, EndP, Q;
  double d, startd, endd;
  int Ni, start_i, start_end, end_i, end_end, i, end;

  if ( index ) 
  {
    for ( i = 0; i < line_count; i++ ) 
      index[i] = i;
  }
  if ( bReverse  )
  {
    for ( i = 0; i < line_count; i++ ) 
      bReverse[i] = false;
  }
  if ( line_count < 1 || 0 == line_list || 0 == index || 0 == bReverse )
  {
    ON_ERROR("ON_SortLines - illegal input");
    return false;
  }
  if ( 1 == line_count )
  {
    return true;
  }

  // sort lines
  for ( Ni = 1; Ni < line_count; Ni++ ) 
  {
    /* index[] = some permutation of {0,...,line_count-1}
    // N[index[0]], ..., N[index[Ni-1]] are in order
    // N[index[j]] needs to be reversed if bReverse[j] is true
    */
    start_i = end_i = Ni;
    start_end = end_end = 0;
    StartP = line_list[ index[0]    ][(bReverse[0])    ? 1 : 0];
    EndP   = line_list[ index[Ni-1] ][(bReverse[Ni-1]) ? 0 : 1];
    startd = StartP.DistanceTo( line_list[index[start_i]].from ); // "from" is correct here
    endd   = EndP.DistanceTo( line_list[index[end_i]].from );     // "from" is correct here

    for ( i = Ni; i < line_count; i++ ) 
    {
      Q = line_list[index[i]].from;
      for ( end = 0; end < 2; end++ ) 
      {
        d = StartP.DistanceTo( Q );
        if ( d < startd ) 
        {
          start_i = i;
          start_end = end;
          startd = d;
        }

        d = EndP.DistanceTo( Q );
        if ( d < endd ) 
        {
          end_i = i;
          end_end = end;
          endd = d;
        }

        Q = line_list[index[i]].to;
      }
    }

    if ( startd < endd ) 
    {
      // N[index[start_i]] will be first in list
      i = index[Ni];
      index[Ni] = index[start_i];
      index[start_i] = i;
      start_i = index[Ni];
      for ( i = Ni; i > 0; i-- ) 
      {
        index[i] = index[i-1];
        bReverse[i] = bReverse[i-1];
      }
      index[0] = start_i;
      bReverse[0] = (start_end != 1);
    }
    else 
    {
      // N[index[end_i]] will be next in the list
      i = index[Ni];
      index[Ni] = index[end_i];
      index[end_i] = i;
      bReverse[Ni] = (end_end == 1);
    }
  }

  return true;
}



bool ON_SortLines( 
        const ON_SimpleArray<ON_Line>& line_list,
        int* index, 
        bool* bReverse 
        )
{
  return ON_SortLines(line_list.Count(),line_list.Array(),index,bReverse);
}

bool ON_SortCurves( int curve_count, const ON_Curve* const* curve_list, int* index, bool* bReverse )
{
  int i;

  if ( curve_count < 1 || 0 == curve_list || 0 == curve_list[0] || 0 == index || 0 == bReverse )
  {
    if ( index ) 
    {
      for ( i = 0; i < curve_count; i++ ) 
        index[i] = i;
    }
    if ( bReverse  )
    {
      for ( i = 0; i < curve_count; i++ ) 
        bReverse[i] = false;
    }
    ON_ERROR("ON_SortCurves - illegal input");
    return false;
  }

  if ( 1 == curve_count )
  {
    index[0] = 0;
    bReverse[0] = false;
    return true;
  }

  // get start and end points
  ON_SimpleArray< ON_Line > line_list(curve_count);
  ON_Interval d;
  bool rc = true;
  for ( i = 0; i < curve_count; i++ ) 
  {
    index[i] = i;
    bReverse[0] = false;
    if ( !rc )
      continue;
    const ON_Curve* curve = curve_list[i];
    if ( !curve )
    {
      rc = false;
      continue;
    }
    d = curve->Domain();
    if ( !d.IsIncreasing() )
    {
      rc = false;
      continue;
    }
    ON_Line& line = line_list.AppendNew();
    if ( !curve->EvPoint(d[0],line.from,1) || !curve->EvPoint(d[1],line.to,-1) )
    {
      rc = false;
    }
  }

  if ( !rc )
  {
    ON_ERROR("ON_SortCurves - illegal input curve");
  }
  else
  {
    rc = ON_SortLines( curve_count, line_list, index, bReverse );
  }
  return rc;
}


bool ON_SortCurves( const ON_SimpleArray<const ON_Curve*>& curves, ON_SimpleArray<int>& index, ON_SimpleArray<bool>& bReverse )
{
  const int curve_count = curves.Count();
  index.Reserve(curve_count);
  index.SetCount(curve_count);
  bReverse.Reserve(curve_count);
  bReverse.SetCount(curve_count);
  return ON_SortCurves( curve_count,curves.Array(),index.Array(),bReverse.Array());
}

bool ON_SortCurves( const ON_SimpleArray<ON_Curve*>& curves, ON_SimpleArray<int>& index, ON_SimpleArray<bool>& bReverse )
{
  const int curve_count = curves.Count();
  index.Reserve(curve_count);
  index.SetCount(curve_count);
  bReverse.Reserve(curve_count);
  bReverse.SetCount(curve_count);
  return ON_SortCurves( curve_count,curves.Array(),index.Array(),bReverse.Array());
}


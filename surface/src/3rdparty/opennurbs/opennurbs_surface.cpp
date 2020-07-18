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

ON_VIRTUAL_OBJECT_IMPLEMENT(ON_Surface,ON_Geometry,"4ED7D4E1-E947-11d3-BFE5-0010830122F0");

ON_Surface::ON_Surface()
: ON_Geometry()
{}

ON_Surface::ON_Surface(const ON_Surface& src)
: ON_Geometry(src)
{}

unsigned int ON_Surface::SizeOf() const
{
  unsigned int sz = ON_Geometry::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Geometry));
  // Currently, the size of m_stree is not included
  // because this is cached runtime information.
  // Applications that care about object size are 
  // typically storing "inactive" objects for potential
  // future use and should call DestroyRuntimeCache(true)
  // to remove any runtime cache information.
  return sz;
}

ON_Surface& ON_Surface::operator=(const ON_Surface& src)
{
  DestroySurfaceTree();
  ON_Geometry::operator=(src);
  return *this;
}

ON_Surface::~ON_Surface()
{
  // Do not call the (virtual) DestroyRuntimeCache or 
  // DestroySurfaceTree (which calls DestroyRuntimeCache()
  // because it opens the potential for crashes in a
  // "dirty" destructors of classes derived from ON_Surface
  // that to not use DestroyRuntimeCache() in their
  // destructors and to not set deleted pointers to zero.
}

ON_Surface* ON_Surface::DuplicateSurface() const
{
  return Duplicate();
}

ON::object_type ON_Surface::ObjectType() const
{
  return ON::surface_object;
}

ON_BOOL32 ON_Surface::GetDomain( int dir, double* t0, double* t1 ) const
{
  ON_Interval d = Domain(dir);
  if ( t0 ) *t0 = d[0];
  if ( t1 ) *t1 = d[1];
  return d.IsIncreasing();
}

ON_BOOL32 ON_Surface::GetSurfaceSize( 
    double* width, 
    double* height 
    ) const
{
  if ( width )
    *width = 0.0;
  if ( height )
    *height = 0.0;
  return false;
}

bool ON_Surface::SetDomain( int dir, ON_Interval domain )
{
  return ( dir >= 0 
           && dir <= 1 
           && domain.IsIncreasing() 
           && SetDomain( dir, domain[0], domain[1] )) ? true : false;
}

ON_BOOL32 ON_Surface::SetDomain( 
  int, // 0 sets first parameter's domain, 1 gets second parameter's domain
  double, double 
  )
{
  // TODO make this pure virutual when all the source code is available
  return false;
}

//////////
// If t is in the domain of the surface, GetSpanVectorIndex() returns the 
// span vector index "i" such that span_vector[i] <= t <= span_vector[i+1].
// The "side" parameter determines which span is selected when t is at the
// end of a span.
//
//virtual
ON_BOOL32 ON_Surface::GetSpanVectorIndex(
      int dir, // 0 gets first parameter's domain, 1 gets second parameter's domain
      double t,      // [IN] t = evaluation parameter
      int side,         // [IN] side 0 = default, -1 = from below, +1 = from above
      int* span_vector_i,        // [OUT] span vector index
      ON_Interval* span_domain // [OUT] domain of the span containing "t"
      ) const
{
  ON_BOOL32 rc = false;
  int i;
  int span_count = SpanCount(dir);
  if ( span_count > 0 ) {
    double* span_vector = (double*)onmalloc((span_count+1)*sizeof(span_vector[0]));
    rc = GetSpanVector( dir, span_vector );
    if (rc) {
      i = ON_NurbsSpanIndex( 2, span_count, span_vector, t, side, 0 );
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

ON_BOOL32 ON_Surface::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
       int dir,
       double t,       // t = parameter in domain
       double* tminus, // tminus
       double* tplus   // tplus
       ) const
{
  ON_BOOL32 rc = false;
  ON_Interval d = Domain( dir );
  if ( d.IsIncreasing() )
    rc = ON_GetParameterTolerance( d.Min(), d.Max(), t, tminus, tplus );
  return rc;
}

ON_Surface::ISO
ON_Surface::IsIsoparametric( const ON_Curve& curve, const ON_Interval* subdomain ) const
{
  ISO iso = not_iso;

  if ( subdomain )
  {
    ON_Interval cdom = curve.Domain();
    double t0 = cdom.NormalizedParameterAt(subdomain->Min());
    double t1 = cdom.NormalizedParameterAt(subdomain->Max());
    if ( t0 < t1-ON_SQRT_EPSILON )
    {
      if ( (t0 > ON_SQRT_EPSILON && t0 < 1.0-ON_SQRT_EPSILON) || (t1 > ON_SQRT_EPSILON && t1 < 1.0-ON_SQRT_EPSILON) )
      {
        cdom.Intersection(*subdomain);
        if ( cdom.IsIncreasing() )
        {
          ON_NurbsCurve nurbs_curve;
          if ( curve.GetNurbForm( nurbs_curve, 0.0,&cdom) )
          {
            return IsIsoparametric( nurbs_curve, 0 );
          }
        }
      }
    }
  }


  ON_BoundingBox bbox;
  double tolerance = 0.0;
  const int dim = curve.Dimension();
  if ( (dim == 2 || dim==3) && curve.GetBoundingBox(bbox) ) 
  {
    iso = IsIsoparametric( bbox );
    switch (iso) {
    case x_iso:
    case W_iso:
    case E_iso:
      // make sure curve is a (nearly) vertical line
      // and weed out vertical scribbles
      tolerance = bbox.m_max.x - bbox.m_min.x;
      if ( tolerance < ON_ZERO_TOLERANCE && ON_ZERO_TOLERANCE*1024.0 <= (bbox.m_max.y-bbox.m_min.y) )
      {
        // 26 March 2007 Dale Lear
        //    If tolerance is tiny, then use ON_ZERO_TOLERANCE
        //    This fixes cases where iso curves where not getting
        //    the correct flag because tol=1e-16 and the closest
        //    point to line had calculation errors of 1e-15.
        tolerance = ON_ZERO_TOLERANCE;
      }
      if ( !curve.IsLinear( tolerance ) )
        iso = not_iso;
      break;
    case y_iso:
    case S_iso:
    case N_iso:
      // make sure curve is a (nearly) horizontal line
      // and weed out horizontal scribbles
      tolerance = bbox.m_max.y - bbox.m_min.y;
      if ( tolerance < ON_ZERO_TOLERANCE && ON_ZERO_TOLERANCE*1024.0 <= (bbox.m_max.x-bbox.m_min.x) )
      {
        // 26 March 2007 Dale Lear
        //    If tolerance is tiny, then use ON_ZERO_TOLERANCE
        //    This fixes cases where iso curves where not getting
        //    the correct flag because tol=1e-16 and the closest
        //    point to line had calculation errors of 1e-15.
        tolerance = ON_ZERO_TOLERANCE;
      }
      if ( !curve.IsLinear( tolerance ) )
        iso = not_iso;
      break;
    default:
      // nothing here
      break;
    }
  }
  return iso;
}

ON_Surface::ISO 
ON_Surface::IsIsoparametric( const ON_BoundingBox& bbox ) const
{
  ISO iso = not_iso;
  if ( bbox.m_min.z == bbox.m_max.z ) {
    const double ds = bbox.m_max.x - bbox.m_min.x;
    const double dt = bbox.m_max.y - bbox.m_min.y;
    double a, b, s0, s1, t0, t1;
    ON_Interval d = Domain(0);
    s0 = d.Min();
    s1 = d.Max();
    d = Domain(1);
    t0 = d.Min();
    t1 = d.Max();
    double stol = (s1-s0)/32.0;
    double ttol = (t1-t0)/32.0;
    if ( s0 < s1 && t0 < t1 && ( ds <= stol || dt <= ttol) ) 
    {
      if ( ds*(t1-t0) <= dt*(s1-s0) ) 
      {
        // check for s = constant iso
        if ( bbox.m_max.x <= s0+stol ) 
        {
          // check for west side iso
          GetParameterTolerance( 0, s0, &a, &b);
          if ( a <= bbox.m_min.x && bbox.m_max.x <= b  ) 
            iso = W_iso;
        }
        else if ( bbox.m_min.x >= s1-stol ) 
        {
          // check for east side iso
          GetParameterTolerance( 0, s1, &a, &b);
          if ( a <= bbox.m_min.x && bbox.m_max.x <= b  )
            iso = E_iso;
        }

        if ( iso == not_iso && (s0 < bbox.m_max.x || bbox.m_min.x < s1) )
        {
          // check for interior "u = constant" iso
          GetParameterTolerance( 0, 0.5*(bbox.m_min.x+bbox.m_max.x), &a, &b);
          if ( a <= bbox.m_min.x && bbox.m_max.x <= b  )
            iso = x_iso;
        }
      }
      else
      {
        // check for t = constant iso
        if ( bbox.m_max.y <= t0+ttol ) 
        {
          // check for south side iso
          GetParameterTolerance( 1, t0, &a, &b);
          if ( a < bbox.m_min.y && bbox.m_max.y <= b  )
            iso = S_iso;
        }
        else if ( bbox.m_min.y >= t1-ttol ) 
        {
          // check for north side iso
          GetParameterTolerance( 1, t1, &a, &b);
          if ( a < bbox.m_min.y && bbox.m_max.y <= b  )
            iso = N_iso;
        }

        if ( iso == not_iso && (t0 < bbox.m_max.x || bbox.m_min.x < t1) )
        {
          // check for interior "t = constant" iso
          GetParameterTolerance( 1, 0.5*(bbox.m_min.y+bbox.m_max.y), &a, &b);
          if ( a < bbox.m_min.y && bbox.m_max.y <= b  )
            iso = y_iso;
        }
      }
    }
  }
  return iso;
}


ON_BOOL32 ON_Surface::IsPlanar( ON_Plane*, double ) const
{
  return false;
}

ON_BOOL32 
ON_Surface::IsClosed(int dir) const
{
  ON_Interval d = Domain(dir);
  if ( d.IsIncreasing() && Dimension() <= 3 ) {
    const int span_count = SpanCount(dir?0:1);
    const int span_degree = Degree(dir?0:1);
    if ( span_count > 0 && span_degree > 0 )
    {
      ON_SimpleArray<double> s(span_count+1);
      s.SetCount(span_count+1);
      int n = 2*span_degree+1;
      double delta = 1.0/n;
      ON_3dPoint P, Q;
      P.Zero();
      Q.Zero();
      int hintP[2] = {0,0};
      int hintQ[2] = {0,0};
      double *u0, *u1, *v0, *v1;
      double t;
      ON_Interval sp;
      if ( dir ) {
        v0 = &d.m_t[0];
        v1 = &d.m_t[1];
        u0 = &t;
        u1 = &t;
      }
      else {
        u0 = &d.m_t[0];
        u1 = &d.m_t[1];
        v0 = &t;
        v1 = &t;
      }
      if ( GetSpanVector( dir?0:1, s.Array() ) )
      {
        int span_index, i;
        for ( span_index = 0; span_index < span_count; span_index++ ) {
          sp.Set(s[span_index],s[span_index+1]);
          for ( i = 0; i < n; i++ ) {
            t = sp.ParameterAt(i*delta);
            if ( !Evaluate( *u0, *v0, 1, 3, P, 0, hintP ) )
              return false;
            if ( !Evaluate( *u1, *v1, 2, 3, Q, 0, hintQ ) )
              return false;
            if ( false == ON_PointsAreCoincident( 3, 0, &P.x, &Q.x ) )
              return false;
          }
        }
        return true;
      }
    }
  }
  return false;
}

ON_BOOL32 ON_Surface::IsPeriodic(int) const
{
  return false;
}

bool ON_Surface::GetNextDiscontinuity( 
                int dir,
                ON::continuity c,
                double t0,
                double t1,
                double* t,
                int*,
                int* dtype,
                double cos_angle_tolerance,
                double curvature_tolerance
                ) const
{
  // 28 Jan 2005 - untested code

  // this function must be overridden by surface objects that
  // can have parametric discontinuities on the interior of the curve.

  bool rc = false;

  // using tmp_dtype means I don't have to keep checking for a null dtype;
  int tmp_dtype = 0;
  if ( !dtype )
    dtype = &tmp_dtype;
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
      int hinta[2], hintb[2], span_index, j;
      ON_Interval domain = Domain(dir);
      ON_Interval span_domain;
      ON_2dPoint st0, st1;
      ON_3dVector Da[6], Db[6];
      ON_3dVector& D1a = Da[1+dir];
      ON_3dVector& D1b = Db[1+dir];
      ON_3dVector& D2a = Da[3+2*dir];
      ON_3dVector& D2b = Db[3+2*dir];

      if ( t0 < domain[1] && t1 >= domain[1] )
        t1 = domain[1];
      else if ( t0 > domain[0] && t1 <= domain[0] )
        t1 = domain[0];

      if ( (t0 < domain[1] && t1 >= domain[1]) || (t0 > domain[0] && t1 <= domain[0]) )
      {
        if ( IsClosed(dir) )
        {
          int span_count = SpanCount(1-dir);
          double* span_vector = (span_count>0) ? ((double*)onmalloc((span_count+1)*sizeof(*span_vector))) : 0;
          if (!GetSpanVector(1-dir,span_vector))
            span_count = 0;
          st0[dir] = domain[0];
          st1[dir] = domain[1];

          for ( span_index = 0; span_index < span_count && 1 != *dtype; span_index++ )
          {
            span_domain.Set(span_vector[span_index],span_vector[span_index+1]);
            for ( j = (span_index?1:0); j <= 2 && 1 != *dtype; j++ )
            {
              st0[1-dir] = span_domain.ParameterAt(0.5*j);
              st1[1-dir] = st0[1-dir];
              if ( bTestD1 || bTestT )
              {
                // need to check locus continuity at start/end of closed surface.
                if (    Evaluate(st0.x,st0.y,2,3,&Da[0].x,1,hinta) 
                     && Evaluate(st1.x,st1.y,2,3,&Db[0].x,2,hintb) )
                {
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
                    else if ( bTestK && (Ka-Kb).Length() > curvature_tolerance )
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

          if ( span_vector)
          {
            onfree(span_vector);
          }
        }
        else
        {
          // open curves are not locus continuous at ends.
          *dtype = 0; // locus C0 discontinuity
          *t = t1;
          rc = true;
        }
      }
    }
  }

  return rc;
}

static 
bool PrincipalCurvaturesAreContinuous( 
    bool bSmoothTest, 
    double k1a, double k2a, // side "a" principal curvatures
    double k1b, double k2b, // side "b" principal curvatures 
    double curvature_tolerance 
    )
{
  ON_3dVector K[2];
  K[0].x = k1a;
  K[0].y = 0.0;
  K[0].z = 0.0;
  K[1].x = k1b;
  K[1].y = 0.0;
  K[1].z = 0.0;
  // compare the first principal curvatures
  bool rc = ( bSmoothTest )
            ? ON_IsGsmoothCurvatureContinuous(K[0],K[1],0.0,curvature_tolerance)
            : ON_IsG2CurvatureContinuous(K[0],K[1],0.0,curvature_tolerance);
  if ( rc )
  {
    // compare the second principal curvatures
    K[0].x = k2a;
    K[1].x = k2b;
    rc = ( bSmoothTest )
       ? ON_IsGsmoothCurvatureContinuous(K[0],K[1],0.0,curvature_tolerance)
       : ON_IsG2CurvatureContinuous(K[0],K[1],0.0,curvature_tolerance);
  }
  return rc;
}

bool ON_Surface::IsContinuous(
    ON::continuity desired_continuity,
    double s, 
    double t, 
    int* hint, // default = NULL,
    double point_tolerance, // default=ON_ZERO_TOLERANCE
    double d1_tolerance, // default==ON_ZERO_TOLERANCE
    double d2_tolerance, // default==ON_ZERO_TOLERANCE
    double cos_angle_tolerance, // default==ON_DEFAULT_ANGLE_TOLERANCE_COSINE
    double curvature_tolerance  // default==ON_SQRT_EPSILON
    ) const
{
  int qi, span_count[2];
  span_count[0] = SpanCount(0);
  span_count[1] = SpanCount(1);
  if ( span_count[0] <= 1 && span_count[1] <= 1 )
    return true;

  ON_3dPoint P[4];
  ON_3dVector Ds[4], Dt[4], Dss[4], Dst[4], Dtt[4], N[4], K1[4], K2[4];
  double gauss[4], mean[4], kappa1[4], kappa2[4], sq[4], tq[4];


  switch ( desired_continuity )
  {
  case ON::C0_locus_continuous:
  case ON::C1_locus_continuous:
  case ON::C2_locus_continuous:
  case ON::G1_locus_continuous:
  case ON::G2_locus_continuous:
    {
      // 7 April 2005 Dale Lear
      //    This locus continuity test was added.  Prior to
      //    this time, this function ignored the word "locus".
      //    The reason for the change is that Chuck's filleting code
      //    needs to query the continuity at the seams of closed surfaces.

      // See ON::continuity comments.  The different sq[] values
      // must NOT be used when s == Domain(0)[0] and must always
      // be used when s == Domain(0)[1].  In particular, if a surface
      // is not closed in the "s" direction and s == Domain(1)[1], then
      // the answer to any locus query is false.
      ON_Interval d = Domain(0);
      if ( s == d[1] )
      {
        sq[0] = sq[1] = d[0];
        sq[2] = sq[3] = d[1];
      }
      else
      {
       sq[0] = sq[1] = sq[2] = sq[3] = s;
      }

      d = Domain(1);
      // See ON::continuity comments.  The different tq[] values
      // must NOT be used when t == Domain(1)[0] and must always
      // be used when t == Domain(1)[1].  In particular, if a surface
      // is not closed in the "t" direction and t == Domain(1)[1], then
      // the answer to any locus query is false.
      if ( t == d[1] )
      {
        tq[0] = tq[3] = d[0];
        tq[1] = tq[2] = d[1];
      }
      else
      {
       tq[0] = tq[1] = tq[2] = tq[3] = t;
      }
    }
    break;

  default:
    sq[0] = sq[1] = sq[2] = sq[3] = s;
    tq[0] = tq[1] = tq[2] = tq[3] = t;
    break;
  }

  desired_continuity = ON::ParametricContinuity(desired_continuity);

  // this is slow and uses evaluation
  // virtual overrides on curve classes that can have multiple spans
  // are much faster because the avoid evaluation
  switch ( desired_continuity )
  {

  case ON::C0_continuous:  
    for ( qi = 0; qi < 4; qi++ )
    {
      if ( !EvPoint( sq[qi], tq[qi], P[qi], qi+1 ) )
        return false;
      if ( qi )
      {
        if ( !(P[qi]-P[qi-1]).IsTiny(point_tolerance) )
          return false;
      }
    }
    if ( !(P[3]-P[0]).IsTiny(point_tolerance) )
      return false;
    break;

  case ON::C1_continuous:
    for ( qi = 0; qi < 4; qi++ )
    {
      if ( !Ev1Der( sq[qi], tq[qi], P[qi], Ds[qi], Dt[qi], qi+1, hint ) )
        return false;
      if ( qi )
      {
        if ( !(P[qi]-P[qi-1]).IsTiny(point_tolerance) )
          return false;
        if ( !(Ds[qi]-Ds[qi-1]).IsTiny(d1_tolerance) )
          return false;
        if ( !(Dt[qi]-Dt[qi-1]).IsTiny(d1_tolerance) )
          return false;
      }
    }
    if ( !(P[3]-P[0]).IsTiny(point_tolerance) )
      return false;
    if ( !(Ds[3]-Ds[0]).IsTiny(d1_tolerance) )
      return false;
    if ( !(Dt[3]-Dt[0]).IsTiny(d1_tolerance) )
      return false;
    break;

  case ON::C2_continuous:
    for ( qi = 0; qi < 4; qi++ )
    {
      if ( !Ev2Der( sq[qi], tq[qi], P[qi], Ds[qi], Dt[qi], 
                    Dss[qi], Dst[qi], Dtt[qi], 
                    qi+1, hint ) )
        return false;
      if ( qi )
      {
        if ( !(P[qi]-P[qi-1]).IsTiny(point_tolerance) )
          return false;
        if ( !(Ds[qi]-Ds[qi-1]).IsTiny(d1_tolerance) )
          return false;
        if ( !(Dt[qi]-Dt[qi-1]).IsTiny(d1_tolerance) )
          return false;
        if ( !(Dss[qi]-Dss[qi-1]).IsTiny(d2_tolerance) )
          return false;
        if ( !(Dst[qi]-Dst[qi-1]).IsTiny(d2_tolerance) )
          return false;
        if ( !(Dtt[qi]-Dtt[qi-1]).IsTiny(d2_tolerance) )
          return false;
      }
    }
    if ( !(P[3]-P[0]).IsTiny(point_tolerance) )
      return false;
    if ( !(Ds[3]-Ds[0]).IsTiny(d1_tolerance) )
      return false;
    if ( !(Dt[3]-Dt[0]).IsTiny(d1_tolerance) )
      return false;
    if ( !(Dss[3]-Dss[0]).IsTiny(d2_tolerance) )
      return false;
    if ( !(Dst[3]-Dst[0]).IsTiny(d2_tolerance) )
      return false;
    if ( !(Dtt[3]-Dtt[0]).IsTiny(d2_tolerance) )
      return false;
    break;

  case ON::G1_continuous:
    for ( qi = 0; qi < 4; qi++ )
    {
      if ( !EvNormal( sq[qi], tq[qi], P[qi], N[qi], qi+1 ) )
        return false;
      if ( qi )
      {
        if ( !(P[qi]-P[qi-1]).IsTiny(point_tolerance) )
          return false;
        if ( N[qi]*N[qi-1] < cos_angle_tolerance )
          return false;
      }
    }
    if ( !(P[3]-P[0]).IsTiny(point_tolerance) )
      return false;
    if ( N[3]*N[0] < cos_angle_tolerance )
      return false;
    break;

  case ON::G2_continuous:
  case ON::Gsmooth_continuous:
    {
      bool bSmoothCon = (ON::Gsmooth_continuous == desired_continuity);
      for ( qi = 0; qi < 4; qi++ )
      {
        if ( !Ev2Der( sq[qi], tq[qi], P[qi], Ds[qi], Dt[qi], 
                      Dss[qi], Dst[qi], Dtt[qi], 
                      qi+1, hint ) )
          return false;
        ON_EvPrincipalCurvatures( Ds[qi], Dt[qi], Dss[qi], Dst[qi], Dtt[qi], N[qi],
                                  &gauss[qi], &mean[qi], &kappa1[qi], &kappa2[qi], 
                                  K1[qi], K2[qi] );
        if ( qi )
        {
          if ( !(P[qi]-P[qi-1]).IsTiny(point_tolerance) )
            return false;
          if ( N[qi]*N[qi-1] < cos_angle_tolerance )
            return false;
          if ( !PrincipalCurvaturesAreContinuous(bSmoothCon,kappa1[qi],kappa2[qi],kappa1[qi-1],kappa2[qi-1],curvature_tolerance) )
            return false;
        }
        if ( !(P[3]-P[0]).IsTiny(point_tolerance) )
          return false;
        if ( N[3]*N[0] < cos_angle_tolerance )
          return false;
        if ( !PrincipalCurvaturesAreContinuous(bSmoothCon,kappa1[3],kappa2[3],kappa1[0],kappa2[0],curvature_tolerance) )
          return false;
      }
    }
    break;

  default:
    // intentionally ignoring other ON::continuity enum values
    break;

  }

  return true;
}

ON_BOOL32 
ON_Surface::IsSingular(int) const
{
  return false;
}

bool ON_Surface::IsSolid() const
{
  const bool bIsClosed0 = ( IsClosed(0) || ( IsSingular(1) && IsSingular(3) ) );
  const bool bIsClosed1 = ( IsClosed(1) || ( IsSingular(0) && IsSingular(2) ) );

  if ( bIsClosed0 && bIsClosed1 )
    return true;

  const ON_Extrusion* extrusion = ON_Extrusion::Cast(this);
  if ( 0 != extrusion && extrusion->IsSolid() )
    return true;

  return false;
}


bool 
ON_Surface::IsAtSingularity(double s, double t,
                            bool bExact //true by default
                            ) const

{
  if (bExact){
    if (s == Domain(0)[0]){
      if (IsSingular(3))
        return true;
    }
    else if (s == Domain(0)[1]){
      if (IsSingular(1))
        return true;
    }
    if (t == Domain(1)[0]){
      if (IsSingular(0))
        return true;
    }
    else if (t == Domain(1)[1]){
      if (IsSingular(2))
        return true;
    }
    return false;
  }

  if (IsAtSingularity(s, t, true))
    return true;

  bool bCheckPartials[2] = {false, false};
  int i;

  double m[2];
  for (i=0; i<2; i++)
    m[i] = Domain(i).Mid();
  if (s < m[0]){
    if (IsSingular(3))
      bCheckPartials[1] = true;
  }
  else {
    if (IsSingular(1))
      bCheckPartials[1] = true;
  }
  if (!bCheckPartials[0] && !bCheckPartials[1]){
    if (t < m[1]){
      if (IsSingular(0))
        bCheckPartials[0] = true;
    }
    else {
      if (IsSingular(2))
        bCheckPartials[0] = true;
    }
  }

  if (!bCheckPartials[0] && !bCheckPartials[1])
    return false;

  ON_3dPoint P;
  ON_3dVector M[2], S[2];
  if (!Ev1Der(s, t, P, S[0], S[1]))
    return false;
  if (!Ev1Der(m[0], m[1], P, M[0], M[1]))
    return false;

  for (i=0; i<2; i++){
    if (!bCheckPartials[i])
      continue;
    if (S[i].Length() < 1.0e-6 * M[i].Length())
      return true;
  }

  return false;

}

int ON_Surface::IsAtSeam(double s, double t) const

{
  int rc = 0;
  int i;
  for (i=0; i<2; i++){
    if (!IsClosed(i))
      continue;
    double p = (i) ? t : s;
    if (p == Domain(i)[0] || p == Domain(i)[1])
      rc += (i+1);
  }

  return rc;
}


ON_3dPoint
ON_Surface::PointAt( double s, double t ) const
{
  ON_3dPoint p(0.0,0.0,0.0);
  EvPoint( s, t, p );
  return p;
}

ON_3dVector
ON_Surface::NormalAt( double s, double t ) const
{
  ON_3dVector N(0.0,0.0,0.0);
  EvNormal( s, t, N );
  return N;
}

ON_BOOL32 ON_Surface::FrameAt( double u, double v, ON_Plane& frame) const
{
  ON_BOOL32 rc = false;
  ON_3dPoint origin;
  ON_3dVector udir, vdir, normal;
  if( EvNormal( u, v, origin, udir, vdir, normal))
  {
    if ( udir.Unitize() )
      vdir = ON_CrossProduct( normal, udir);
    else if ( vdir.Unitize() )
      udir = ON_CrossProduct( vdir, normal);
    frame.CreateFromFrame( origin, udir, vdir);
    rc = frame.IsValid();
  }
  return rc;
}



ON_BOOL32 
ON_Surface::EvPoint( // returns false if unable to evaluate
       double s, double t, // evaluation parameters
       ON_3dPoint& point,
       int side,        // optional - determines which side to evaluate from
                       //         0 = default
                       //         1 from NE quadrant
                       //         2 from NW quadrant
                       //         3 from SW quadrant
                       //         4 from SE quadrant
       int* hint       // optional - evaluation hint (int[2]) used to speed
                       //            repeated evaluations
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
  rc = Evaluate( s, t, 0, Dimension(), v, side, hint );
  if ( Dimension() > 3 ) {
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    if ( Dimension() > 128 )
      onfree(v);
  }
  return rc;
}

ON_BOOL32
ON_Surface::Ev1Der( // returns false if unable to evaluate
       double s, double t, // evaluation parameters
       ON_3dPoint& point,
       ON_3dVector& ds,
       ON_3dVector& dt,
       int side,        // optional - determines which side to evaluate from
                       //         0 = default
                       //         1 from NE quadrant
                       //         2 from NW quadrant
                       //         3 from SW quadrant
                       //         4 from SE quadrant
       int* hint       // optional - evaluation hint (int[2]) used to speed
                       //            repeated evaluations
       ) const
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  double ws[3*32];
  double* v;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  ds.x = 0.0;
  ds.y = 0.0;
  ds.z = 0.0;
  dt.x = 0.0;
  dt.y = 0.0;
  dt.z = 0.0;
  if ( dim <= 32 ) {
    v = ws;
  }
  else {
    v = (double*)onmalloc(3*dim*sizeof(*v));
  }
  rc = Evaluate( s, t, 1, dim, v, side, hint );
  point.x = v[0];
  ds.x = v[dim];
  dt.x = v[2*dim];
  if ( dim > 1 ) {
    point.y = v[1];
    ds.y = v[dim+1];
    dt.y = v[2*dim+1];
    if ( dim > 2 ) {
      point.z = v[2];
      ds.z = v[dim+2];
      dt.z = v[2*dim+2];
      if ( dim > 32 )
        onfree(v);
    }
  }

  return rc;
}

ON_BOOL32 
ON_Surface::Ev2Der( // returns false if unable to evaluate
       double s, double t, // evaluation parameters
       ON_3dPoint& point,
       ON_3dVector& ds,
       ON_3dVector& dt,
       ON_3dVector& dss,
       ON_3dVector& dst,
       ON_3dVector& dtt,
       int side,        // optional - determines which side to evaluate from
                       //         0 = default
                       //         1 from NE quadrant
                       //         2 from NW quadrant
                       //         3 from SW quadrant
                       //         4 from SE quadrant
       int* hint       // optional - evaluation hint (int[2]) used to speed
                       //            repeated evaluations
       ) const
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  double ws[6*16];
  double* v;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  ds.x = 0.0;
  ds.y = 0.0;
  ds.z = 0.0;
  dt.x = 0.0;
  dt.y = 0.0;
  dt.z = 0.0;
  dss.x = 0.0;
  dss.y = 0.0;
  dss.z = 0.0;
  dst.x = 0.0;
  dst.y = 0.0;
  dst.z = 0.0;
  dtt.x = 0.0;
  dtt.y = 0.0;
  dtt.z = 0.0;
  if ( dim <= 16 ) {
    v = ws;
  }
  else {
    v = (double*)onmalloc(6*dim*sizeof(*v));
  }
  rc = Evaluate( s, t, 2, dim, v, side, hint );
  point.x = v[0];
  ds.x = v[dim];
  dt.x = v[2*dim];
  dss.x = v[3*dim];
  dst.x = v[4*dim];
  dtt.x = v[5*dim];
  if ( dim > 1 ) {
    point.y = v[1];
    ds.y = v[dim+1];
    dt.y = v[2*dim+1];
    dss.y = v[3*dim+1];
    dst.y = v[4*dim+1];
    dtt.y = v[5*dim+1];
    if ( dim > 2 ) {
      point.z = v[2];
      ds.z = v[dim+2];
      dt.z = v[2*dim+2];
      dss.z = v[3*dim+2];
      dst.z = v[4*dim+2];
      dtt.z = v[5*dim+2];
      if ( dim > 16 )
        onfree(v);
    }
  }

  return rc;
}


ON_BOOL32
ON_Surface::EvNormal( // returns false if unable to evaluate
         double s, double t, // evaluation parameters (s,t)
         ON_3dVector& normal, // unit normal
         int side,       // optional - determines which side to evaluate from
                         //         0 = default
                         //         1 from NE quadrant
                         //         2 from NW quadrant
                         //         3 from SW quadrant
                         //         4 from SE quadrant
         int* hint       // optional - evaluation hint (int[2]) used to speed
                         //            repeated evaluations
         ) const
{
  ON_3dPoint point;
  ON_3dVector ds, dt;
  return EvNormal( s, t, point, ds, dt, normal, side, hint );
}

ON_BOOL32
ON_Surface::EvNormal( // returns false if unable to evaluate
         double s, double t, // evaluation parameters (s,t)
         ON_3dPoint& point,  // returns value of surface
         ON_3dVector& normal, // unit normal
         int side,       // optional - determines which side to evaluate from
                         //         0 = default
                         //         1 from NE quadrant
                         //         2 from NW quadrant
                         //         3 from SW quadrant
                         //         4 from SE quadrant
         int* hint       // optional - evaluation hint (int[2]) used to speed
                         //            repeated evaluations
         ) const
{
  ON_3dVector ds, dt;
  return EvNormal( s, t, point, ds, dt, normal, side, hint );
}

ON_BOOL32
ON_Surface::EvNormal( // returns false if unable to evaluate
         double s, double t, // evaluation parameters (s,t)
         ON_3dPoint& point,  // returns value of surface
         ON_3dVector& ds, // first partial derivatives (Ds)
         ON_3dVector& dt, // (Dt)
         ON_3dVector& normal, // unit normal
         int side,       // optional - determines which side to evaluate from
                         //         0 = default
                         //         1 from NE quadrant
                         //         2 from NW quadrant
                         //         3 from SW quadrant
                         //         4 from SE quadrant
         int* hint       // optional - evaluation hint (int[2]) used to speed
                         //            repeated evaluations
         ) const
{
  // simple cross product normal - override to support singular surfaces
  ON_BOOL32 rc = Ev1Der( s, t, point, ds, dt, side, hint );
  if ( rc ) {
    const double len_ds = ds.Length();
    const double len_dt = dt.Length();

    // do not reduce the tolerance used here - there is a retry in the code
    // below.
    if ( len_ds >  ON_SQRT_EPSILON*len_dt && len_dt >  ON_SQRT_EPSILON*len_ds ) 
    {
      ON_3dVector a = ds/len_ds;
      ON_3dVector b = dt/len_dt;
      normal = ON_CrossProduct( a, b );
      rc = normal.Unitize();
    }
    else 
    {
      // see if we have a singular point 
      double v[6][3];
      int normal_side = side;
      ON_BOOL32 bOnSide = false;
      ON_Interval sdom = Domain(0);
      ON_Interval tdom = Domain(1);
		  if (s == sdom.Min()) {
			  normal_side = (normal_side >= 3) ? 4 : 1;
        bOnSide = true;
		  }
		  else if (s == sdom.Max()) {
			  normal_side = (normal_side >= 3) ? 3 : 2;
        bOnSide = true;
		  }
		  if (t == tdom.Min()) {
			  normal_side = (normal_side == 2 || normal_side == 3) ? 2 : 1;
        bOnSide = true;
		  }
		  else if (t == tdom.Max()) {
			  normal_side = (normal_side == 2 || normal_side == 3) ? 3 : 4;
        bOnSide = true;
		  }
      if ( !bOnSide )
      {
        // 2004 November 11 Dale Lear 
        //  Added a retry again with a more generous tolerance
        if ( len_ds >  ON_EPSILON*len_dt && len_dt >  ON_EPSILON*len_ds ) 
        {
          ON_3dVector a = ds/len_ds;
          ON_3dVector b = dt/len_dt;
          normal = ON_CrossProduct( a, b );
          rc = normal.Unitize();
        }
        else
        {
          rc = false;
        }
      }
      else {
        rc = Evaluate( s, t, 2, 3, &v[0][0], normal_side, hint );
        if ( rc ) {
	        rc = ON_EvNormal( normal_side, v[1], v[2], v[3], v[4], v[5], normal);
        }
      }
    }
  }
  if ( !rc ) {
    normal.Zero();
  }
  return rc;
}

//virtual
ON_Curve* ON_Surface::IsoCurve(
       int    ,    // 0 first parameter varies and second parameter is constant
                   //   e.g., point on IsoCurve(0,c) at t is srf(t,c) - Horizontal
                   // 1 first parameter is constant and second parameter varies
                   //   e.g., point on IsoCurve(1,c) at t is srf(c,t) - Vertical
       double      // value of constant parameter
       ) const
{
  return NULL;
}

//virtual
ON_BOOL32 ON_Surface::Trim(
       int,
       const ON_Interval&
       )
{
  return false;
}

//virtual
bool ON_Surface::Extend(
      int,
      const ON_Interval&
      )
{
  return false;
}

//virtual
ON_BOOL32 ON_Surface::Split(
       int,
       double,
       ON_Surface*&,
       ON_Surface*&
       ) const
{
  return false;
}

// virtual
int ON_Surface::GetNurbForm(
      ON_NurbsSurface&,
      double
      ) const
{
  return 0;
}

// virtual
int ON_Surface::HasNurbForm( ) const
{
  return 0;
}

bool ON_Surface::GetSurfaceParameterFromNurbFormParameter(
      double nurbs_s, double nurbs_t,
      double* surface_s, double* surface_t
      ) const
{
  // NOTE: ON_SumSurface and ON_RevSurface override this virtual function
  *surface_s = nurbs_s;
  *surface_t = nurbs_t;
  return true;
}

bool ON_Surface::GetNurbFormParameterFromSurfaceParameter(
      double surface_s, double surface_t,
      double* nurbs_s,  double* nurbs_t
      ) const
{
  // NOTE: ON_SumSurface and ON_RevSurface override this virtual function
  *nurbs_s = surface_s;
  *nurbs_t = surface_t;
  return true;
}


ON_NurbsSurface* ON_Surface::NurbsSurface(
      ON_NurbsSurface* pNurbsSurface,
      double tolerance,
      const ON_Interval*,
      const ON_Interval*
      ) const
{
  ON_NurbsSurface* nurbs_surface = pNurbsSurface;
  if ( !nurbs_surface )
    nurbs_surface = new ON_NurbsSurface();
  int rc = GetNurbForm( *nurbs_surface, tolerance );
  if ( !rc )
  {
    if (!pNurbsSurface)
      delete nurbs_surface;
    nurbs_surface = NULL;
  }
  return nurbs_surface;
}



ON_SurfaceArray::ON_SurfaceArray( int initial_capacity )
                   : ON_SimpleArray<ON_Surface*>(initial_capacity)
{}

ON_SurfaceArray::~ON_SurfaceArray()
{
  Destroy();
}

void ON_SurfaceArray::Destroy()
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

ON_BOOL32 ON_SurfaceArray::Duplicate( ON_SurfaceArray& dst ) const
{
  dst.Destroy();
  dst.SetCapacity( Capacity() );

  const int count = Count();
  int i;
  ON_Surface* surface;
  for ( i = 0; i < count; i++ ) 
  {
    surface = 0;
    if ( m_a[i] ) 
    {
      surface = m_a[i]->Duplicate();
    }
    dst.Append(surface);      
  }
  return true;
}

ON_BOOL32 ON_SurfaceArray::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 0 );
  if (rc) rc = file.Write3dmChunkVersion(1,0);
  if (rc ) 
  {
    int i;
    rc = file.WriteInt( Count() );
    for ( i = 0; rc && i < Count(); i++ ) {
      if ( m_a[i] ) 
      {
        rc = file.WriteInt(1);
        if ( rc ) 
          rc = file.WriteObject( *m_a[i] ); // polymorphic surfaces
      }
      else 
      {
        // NULL surface
        rc = file.WriteInt(0);
      }
    }
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}


ON_BOOL32 ON_SurfaceArray::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  int flag;
  Destroy();
  ON_BOOL32 rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
  if (rc) 
  {
    rc = ( tcode == TCODE_ANONYMOUS_CHUNK );
    if (rc) rc = file.Read3dmChunkVersion(&major_version,&minor_version);
    if (rc && major_version == 1) {
      ON_Object* p;
      int count;
      ON_BOOL32 rc = file.ReadInt( &count );
      if (rc) {
        SetCapacity(count);
        SetCount(count);
        Zero();
        int i;
        for ( i = 0; rc && i < count && rc; i++ ) {
          flag = 0;
          rc = file.ReadInt(&flag);
          if (rc && flag==1) {
            p = 0;
            rc = file.ReadObject( &p ); // polymorphic surfaces
            m_a[i] = ON_Surface::Cast(p);
            if ( !m_a[i] )
              delete p;
          }
        }
      }
    }
    else {
      rc = false;
    }
    if ( !file.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}

ON_BOOL32 ON_Surface::HasBrepForm() const
{
  return true;
}

ON_Brep* ON_Surface::BrepForm( ON_Brep* brep ) const
{
  ON_Brep* pBrep = NULL;
  if ( brep )
    brep->Destroy();
  // 26 August 2008 Dale Lear - fixed bug
  //    When this function was called on an ON_SurfaceProxy
  //ON_Surface* pSurface = Duplicate();
  ON_Surface* pSurface = DuplicateSurface();
  if ( pSurface )
  {
    if ( brep )
      pBrep = brep;
    else
      pBrep = new ON_Brep();
    if ( !pBrep->Create(pSurface) )
    {
      if ( pSurface )
      {
        delete pSurface;
        pSurface = NULL;
      }
      if ( !brep )
        delete pBrep;
      pBrep = NULL;
    }
  }
  return pBrep;
}

void ON_Surface::DestroySurfaceTree()
{
  DestroyRuntimeCache(true);
}

ON_SurfaceProperties::ON_SurfaceProperties()
{
  memset(this,0,sizeof(*this));
}


void ON_SurfaceProperties::Set( const ON_Surface* surface )
{
  m_surface = surface;

  if ( 0 == m_surface )
  {
    m_bIsSet = false;

    m_bHasSingularity = false;
    m_bIsSingular[0] = false;
    m_bIsSingular[1] = false;
    m_bIsSingular[2] = false;
    m_bIsSingular[3] = false;

    m_bHasSeam = false;
    m_bIsClosed[0] = false;
    m_bIsClosed[1] = false;

    m_domain[0].Set(0.0,0.0);
    m_domain[1].Set(0.0,0.0);
  }
  else
  {
    m_bIsSet = true;

    m_bIsSingular[0] = m_surface->IsSingular(0)?true:false;
    m_bIsSingular[1] = m_surface->IsSingular(1)?true:false;
    m_bIsSingular[2] = m_surface->IsSingular(2)?true:false;
    m_bIsSingular[3] = m_surface->IsSingular(3)?true:false;
    m_bHasSingularity = (m_bIsSingular[0] || m_bIsSingular[1] || m_bIsSingular[2] || m_bIsSingular[3]);

    m_bIsClosed[0] = m_surface->IsClosed(0)?true:false;
    m_bIsClosed[1] = m_surface->IsClosed(1)?true:false;
    m_bHasSeam = (m_bIsClosed[0] || m_bIsClosed[1]);

    m_domain[0] = m_surface->Domain(0);
    m_domain[1] = m_surface->Domain(1);
  }

}

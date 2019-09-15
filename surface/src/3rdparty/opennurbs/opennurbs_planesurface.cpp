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

ON_OBJECT_IMPLEMENT(ON_PlaneSurface,ON_Surface,"4ED7D4DF-E947-11d3-BFE5-0010830122F0");
ON_OBJECT_IMPLEMENT(ON_ClippingPlaneSurface,ON_PlaneSurface,"DBC5A584-CE3F-4170-98A8-497069CA5C36");


ON_PlaneSurface::ON_PlaneSurface()
{}

ON_PlaneSurface::ON_PlaneSurface( const ON_PlaneSurface& src ) : ON_Surface(src)
{
  *this = src;
}

ON_PlaneSurface& ON_PlaneSurface::operator=( const ON_PlaneSurface& src )
{
  if ( this != &src ) {
    ON_Surface::operator=(src);
    m_plane = src.m_plane;
    m_domain[0] = src.m_domain[0];
    m_domain[1] = src.m_domain[1];
    m_extents[0] = src.m_extents[0];
    m_extents[1] = src.m_extents[1];
  }
  return *this;
}

ON_PlaneSurface::ON_PlaneSurface( const ON_Plane& src )
{
  *this = src;
}


unsigned int ON_PlaneSurface::SizeOf() const
{
  unsigned int sz = ON_Surface::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Surface));
  return sz;
}

ON__UINT32 ON_PlaneSurface::DataCRC(ON__UINT32 current_remainder) const
{
  current_remainder = ON_CRC32(current_remainder,sizeof(m_plane),&m_plane);
  current_remainder = ON_CRC32(current_remainder,2*sizeof(m_domain[0]),&m_domain[0]);
  current_remainder = ON_CRC32(current_remainder,2*sizeof(m_extents[0]),&m_extents[0]);
  return current_remainder;
}


ON_PlaneSurface& ON_PlaneSurface::operator=( const ON_Plane& src )
{
  m_plane = src;
  m_domain[0].Set(0.0,1.0);
  m_domain[1].Set(0.0,1.0);
  m_extents[0] = m_domain[0];
  m_extents[1] = m_domain[1];
  return *this;
}

ON_PlaneSurface::~ON_PlaneSurface()
{}

ON_BOOL32
ON_PlaneSurface::IsValid( ON_TextLog* ) const
{
  return (   m_plane.IsValid() 
           && m_domain[0].IsIncreasing() && m_domain[1].IsIncreasing() 
           && m_extents[0].IsIncreasing() && m_extents[1].IsIncreasing() 
           ) ? true : false;
}

void
ON_PlaneSurface::Dump( ON_TextLog& dump ) const
{
  dump.Print("ON_PlaneSurface\n");
}

ON_BOOL32 
ON_PlaneSurface::Write(
       ON_BinaryArchive& file  // open binary file
     ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,1);

  // version 1.0 chunks
  if (rc)
    rc = file.WritePlane( m_plane );
  if (rc)
    rc = file.WriteInterval( m_domain[0] );
  if (rc)
    rc = file.WriteInterval( m_domain[1] );
  
  // added to version 1.1 chunks
  if (rc)
    rc = file.WriteInterval( m_extents[0] );
  if (rc)
    rc = file.WriteInterval( m_extents[1] );
  return rc;
}

ON_BOOL32 
ON_PlaneSurface::Read(
       ON_BinaryArchive& file // open binary file
     )
{
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version == 1) {
    // common to all 1.x formats
    if (rc)
      rc = file.ReadPlane( m_plane );
    if (rc)
      rc = file.ReadInterval( m_domain[0] );
    if (rc)
      rc = file.ReadInterval( m_domain[1] );
    m_extents[0] = m_domain[0];
    m_extents[1] = m_domain[1];
    if ( minor_version >= 1 )
    {
      if (rc)
        rc = file.ReadInterval( m_extents[0] );
      if (rc)
        rc = file.ReadInterval( m_extents[1] );
    }
  }
  return rc;
}

int 
ON_PlaneSurface::Dimension() const
{
  return 3;
}

ON_BOOL32 
ON_PlaneSurface::GetBBox( // returns true if successful
         double* boxmin,    // minimum
         double* boxmax,    // maximum
         ON_BOOL32 bGrowBox
         ) const
{
  int i,j,k=0;
  ON_3dPoint corner[4];
  for ( i = 0; i < 2; i++ ) for ( j = 0; j < 2; j++ ) {
    corner[k++] = PointAt( m_domain[0].m_t[i], m_domain[1].m_t[j] );
  }
  return ON_GetPointListBoundingBox( 3, 0, 4, 3, 
                                     &corner[0].x, 
                                     boxmin, 
                                     boxmax, bGrowBox?true:false );
}

ON_BOOL32
ON_PlaneSurface::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
  ON_3dPoint p = m_plane.origin + m_extents[0][0]*m_plane.xaxis + m_extents[1][0]*m_plane.yaxis;
  ON_3dPoint q = m_plane.origin + m_extents[0][1]*m_plane.xaxis + m_extents[1][1]*m_plane.yaxis;
  bool rc = m_plane.Transform( xform )?true:false;
  if (rc && fabs(fabs(xform.Determinant())-1.0) > ON_SQRT_EPSILON )
  {
    p = xform*p;
    q = xform*q;
    double x0, x1, y0, y1;
    rc = false;
    if ( m_plane.ClosestPointTo(p,&x0,&y0) && m_plane.ClosestPointTo(q,&x1,&y1) )
    {
      if ( x0 < x1 && y0 < y1 )
      {
        m_extents[0].Set(x0,x1);
        m_extents[1].Set(y0,y1);
        rc = true;
      }
    }
  }
  return rc;
}

ON_Interval ON_PlaneSurface::Domain( int dir ) const
{
  // evaluation domain - do not confuse with m_extents
  return dir ? m_domain[1] : m_domain[0];
}

int ON_PlaneSurface::SpanCount( int ) const
{
  return 1;
}

ON_BOOL32 ON_PlaneSurface::GetSurfaceSize( 
    double* width, 
    double* height 
    ) const
{
  if ( width ) 
    *width = Extents(0).Length();
  if ( height ) 
    *height = Extents(1).Length();
  return true;
}


ON_BOOL32 ON_PlaneSurface::GetSpanVector( int dir, double* s ) const
{
  ON_Interval d = Domain(dir);
  s[0] = d.Min();
  s[1] = d.Max();
  return d.IsIncreasing();
}

int ON_PlaneSurface::Degree( int ) const
{
  return 1;
}

ON_BOOL32 
ON_PlaneSurface::GetParameterTolerance(
         int dir,
         double t,  // t = parameter in domain
         double* tminus, // tminus
         double* tplus   // tplus
         ) const
{
  dir = (dir)?1:0;
  return ON_GetParameterTolerance( m_domain[dir][0], m_domain[dir][1], t, tminus, tplus );
}

ON_BOOL32 ON_PlaneSurface::IsPlanar( ON_Plane* plane, double ) const
{
  if ( plane )
    *plane = this->m_plane;
  return true;
}

ON_BOOL32 
ON_PlaneSurface::IsClosed( int ) const
{
  return false;
}

ON_BOOL32 
ON_PlaneSurface::IsPeriodic( int ) const
{
  return false;
}

ON_BOOL32 
ON_PlaneSurface::IsSingular( int ) const
{
  return false;
}

bool ON_PlaneSurface::GetNextDiscontinuity( 
                int dir,
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
  return ON_Surface::GetNextDiscontinuity(dir,c,t0,t1,t,hint,dtype,cos_angle_tolerance,curvature_tolerance);
}

ON_BOOL32
ON_PlaneSurface::Reverse( int dir )
{
  if ( dir < 0 || dir > 1 )
    return false;
  m_extents[dir].Reverse();
  m_domain[dir].Reverse();
  if ( dir )
    m_plane.yaxis.Reverse();
  else
    m_plane.xaxis.Reverse();
  m_plane.zaxis.Reverse();
  m_plane.UpdateEquation();
  return true;
}

bool ON_PlaneSurface::IsContinuous(
    ON::continuity,
    double,
    double,
    int*,   // default = NULL,
    double, // default=ON_ZERO_TOLERANCE
    double, // default==ON_ZERO_TOLERANCE
    double, // default==ON_ZERO_TOLERANCE
    double, // default==ON_DEFAULT_ANGLE_TOLERANCE_COSINE
    double  // default==ON_SQRT_EPSILON
    ) const
{
  return true;
}

ON_BOOL32
ON_PlaneSurface::Transpose()
{
  // swaps x and y axes and reverses zaxis
  m_plane.Flip();

  ON_Interval i = m_domain[0];
  m_domain[0] = m_domain[1];
  m_domain[1] = i;

  i = m_extents[0];
  m_extents[0] = m_extents[1];
  m_extents[1] = i;

  return true;
}

ON_BOOL32 
ON_PlaneSurface::Evaluate( // returns false if unable to evaluate
       double s, double t, // evaluation parameters
       int der_count,  // number of derivatives (>=0)
       int v_stride,   // v[] array stride (>=Dimension())
       double* v,      // v[] array of length stride*(ndir+1)
       int     ,       // optional - determines which side to evaluate from
                       //         0 = default
                       //      <  0 to evaluate from below, 
                       //      >  0 to evaluate from above
       int*            // optional - evaluation hint (int) used to speed
                       //            repeated evaluations
       ) const
{
  double ds = 1.0;
  double dt = 1.0;
  if ( m_extents[0] != m_domain[0] )
  {
    s = m_extents[0].ParameterAt( m_domain[0].NormalizedParameterAt(s) );
    ds = m_extents[0].Length()/m_domain[0].Length();
  }
  if ( m_extents[1] != m_domain[1] )
  {
    t = m_extents[1].ParameterAt( m_domain[1].NormalizedParameterAt(t) );
    dt = m_extents[1].Length()/m_domain[1].Length();
  }
  ON_3dPoint P = m_plane.PointAt( s, t );
  v[0] = P.x;
  v[1] = P.y;
  v[2] = P.z;
  v += v_stride;
  if ( der_count >= 1 ) 
  {
    v[0] = ds*m_plane.xaxis.x;
    v[1] = ds*m_plane.xaxis.y;
    v[2] = ds*m_plane.xaxis.z;
    v += v_stride;

    v[0] = dt*m_plane.yaxis.x;
    v[1] = dt*m_plane.yaxis.y;
    v[2] = dt*m_plane.yaxis.z;
    v += v_stride;

    if ( der_count > 1 ) 
    {
      // zero higher partials
      memset( v, 0, (((der_count+1)*(der_count+2)/2-4)*v_stride+3)*sizeof(*v) );
    }
  }
  return true;
}

ON_Curve* ON_PlaneSurface::IsoCurve( int dir, double c ) const
{
  ON_LineCurve* line_curve = 0;
  if ( (dir == 0 || dir == 1) && IsValid() ) 
  {
    ON_Line line;
    ON_Interval domain = Domain(dir);
    if ( dir == 1 )
    {
      line.from = PointAt( c, domain[0] );
      line.to = PointAt( c, domain[1] );
    }
    else
    {
      line.from = PointAt( domain[0], c );
      line.to = PointAt( domain[1], c );
    }
    line_curve = new ON_LineCurve(line);
    line_curve->m_dim = 3;
    line_curve->m_t = domain;
  }
  return line_curve;
}

ON_BOOL32 ON_PlaneSurface::Trim(
       int dir,
       const ON_Interval& domain
       )
{
  if ( dir < 0 || dir > 1 )
    return false;
  ON_Interval current_domain = Domain(dir);
  if ( current_domain[0] == ON_UNSET_VALUE && current_domain[1] == ON_UNSET_VALUE )
    current_domain = domain;
  ON_Interval trim_domain, trim_extents = m_extents[dir];
  trim_domain.Intersection(domain, Domain(dir) );
  if ( !trim_domain.IsIncreasing() )
    return false;
  if ( m_domain[dir] == m_extents[dir] )
    trim_extents = trim_domain;
  else
  {
    double x0 = m_extents[dir].ParameterAt( m_domain[dir].NormalizedParameterAt( trim_domain[0] ) );
    double x1 = m_extents[dir].ParameterAt( m_domain[dir].NormalizedParameterAt( trim_domain[1] ) );
    trim_extents.Set(x0,x1);
  }
  if ( !trim_extents.IsIncreasing() )
    return false;
  m_extents[dir] = trim_extents;
  m_domain[dir] = trim_domain;
  return true;
}

bool ON_PlaneSurface::Extend(
      int dir,
      const ON_Interval& domain
      )
{
  if ( dir < 0 || dir > 1 ) return false;
  bool changed = false;
  ON_Interval tdom = Domain(dir);
  ON_Interval xdom = m_extents[dir];

  if (domain[0] < Domain(dir)[0]){
    changed = true;
    tdom[0] = domain[0];
    xdom[0] = m_extents[dir].ParameterAt( m_domain[dir].NormalizedParameterAt(domain[0]));
  }
  if (domain[1] > Domain(dir)[1]){
    changed = true;
    tdom[1] = domain[1];
    xdom[1] = m_extents[dir].ParameterAt( m_domain[dir].NormalizedParameterAt(domain[1]));
  }
  if (!changed) return false;
  DestroySurfaceTree();

  m_domain[dir] = tdom;
  m_extents[dir] = xdom;
  return true;
}

ON_BOOL32 ON_PlaneSurface::Split(
       int dir,
       double c,
       ON_Surface*& west_or_south_side,
       ON_Surface*& east_or_north_side
       ) const
{
  ON_PlaneSurface* ws_side = 0;
  ON_PlaneSurface* en_side = 0;

  if ( dir < 0 || dir > 1 )
    return false;
  if ( !Domain(dir).Includes(c,true) )
    return false;

  double t;
  if ( Domain(dir) == Extents(dir) )
    t = c;
  else
  {
    t = Extents(dir).ParameterAt( Domain(dir).NormalizedParameterAt(c) );
    if ( !Extents(dir).Includes(t,true) )
      return false;
  }

  if ( west_or_south_side )
  {
    if ( west_or_south_side == east_or_north_side )
      return false;
    ws_side = ON_PlaneSurface::Cast(west_or_south_side);
    if ( !ws_side )
      return false;
  }

  if ( east_or_north_side )
  {
    en_side = ON_PlaneSurface::Cast(east_or_north_side);
    if ( !en_side )
      return false;
  }

  if ( !ws_side )
    ws_side = new ON_PlaneSurface();
  if ( !en_side )
    en_side = new ON_PlaneSurface();

  *ws_side = *this;
  *en_side = *this;
  ws_side->m_domain[dir].m_t[1] = c;
  en_side->m_domain[dir].m_t[0] = c;
  ws_side->m_extents[dir].m_t[1] = t;
  en_side->m_extents[dir].m_t[0] = t;

  west_or_south_side = ws_side;
  east_or_north_side = en_side;

  return true;
}

bool ON_PlaneSurface::GetClosestPoint( const ON_3dPoint& test_point,
        double* s,double* t,  // parameters of local closest point returned here
        double maximum_distance,
        const ON_Interval* sdomain, // first parameter sub_domain
        const ON_Interval* tdomain  // second parameter sub_domain
        ) const
{
  double u = 0.0, v=0.0;

	ON_Interval sdom = Domain(0);
	ON_Interval tdom = Domain(1);
	if(sdomain==NULL)
		sdomain = &sdom;
	if(tdomain==NULL)
		tdomain = &tdom;

  bool rc = m_plane.ClosestPointTo( test_point, &u, &v );
  if ( rc ) 
  {
    // convert m_plane coordinates to ON_Surface coordinates
    if ( m_domain[0] != m_extents[0] )
    {
      u = m_domain[0].ParameterAt( m_extents[0].NormalizedParameterAt(u) );
    }
    if ( m_domain[1] != m_extents[1] )
    {
      v = m_domain[1].ParameterAt( m_extents[1].NormalizedParameterAt(v) );
    }

    if ( u < sdomain->Min() )
      u = sdomain->Min();
    else if ( u > sdomain->Max() )
      u = sdomain->Max();

    if ( v < tdomain->Min() )
      v = tdomain->Min();
    else if ( v > tdomain->Max() )
      v = tdomain->Max();

    if ( s )
      *s = u;
    if ( t )
      *t = v;
    if (maximum_distance > 0.0) 
    {
      ON_3dPoint pt = PointAt(u,v);
      if ( test_point.DistanceTo(pt) > maximum_distance )
        rc = false;
    }
  }
  return rc;
}

//////////
// Find parameters of the point on a surface that is locally closest to 
// the test_point.  The search for a local close point starts at 
// seed parameters. If a sub_domain parameter is not NULL, then
// the search is restricted to the specified portion of the surface.
//
// true if returned if the search is successful.  false is returned if
// the search fails.
ON_BOOL32 ON_PlaneSurface::GetLocalClosestPoint( const ON_3dPoint& test_point, // test_point
        double, double,     // seed_parameters
        double* s,double* t,   // parameters of local closest point returned here
        const ON_Interval* sdomain, // first parameter sub_domain
        const ON_Interval* tdomain  // second parameter sub_domain
        ) const
{
  // for planes, global serach is fast and returns same answer as local search
  return GetClosestPoint(test_point,s,t,0.0,sdomain,tdomain);
}


ON_Surface* ON_PlaneSurface::Offset(
      double offset_distance, 
      double,
      double* max_deviation
      ) const
{
  if ( max_deviation )
    *max_deviation = 0.0;
  ON_PlaneSurface* offset_srf = new ON_PlaneSurface(*this);
  ON_3dVector delta = offset_srf->m_plane.zaxis;
  double d = delta.Length();
  if ( fabs(1.0-d) <= ON_SQRT_EPSILON )
    d = 1.0;
  d = offset_distance/d;
  offset_srf->m_plane.origin = offset_srf->m_plane.origin + (d*delta);
  offset_srf->m_plane.UpdateEquation();
  return offset_srf;
}


int 
ON_PlaneSurface::GetNurbForm( // returns 0: unable to create NURBS representation
                   //            with desired accuracy.
                   //         1: success - returned NURBS parameterization
                   //            matches the surface's to wthe desired accuracy
                   //         2: success - returned NURBS point locus matches
                   //            the surfaces's to the desired accuracy but, on
                   //            the interior of the surface's domain, the 
                   //            surface's parameterization and the NURBS
                   //            parameterization may not match to the 
                   //            desired accuracy.
        ON_NurbsSurface& nurbs,
        double
        ) const
{
  ON_BOOL32 rc = IsValid();

  if( !rc )
  {
    if (    m_plane.origin.x != ON_UNSET_VALUE 
         && m_plane.xaxis.x != ON_UNSET_VALUE 
         && m_plane.yaxis.x != ON_UNSET_VALUE
         && m_domain[0].IsIncreasing() && m_domain[1].IsIncreasing()
         && m_extents[0].Length() > 0.0 && m_extents[1].Length() > 0.0
         )
    {
      ON_3dVector N = ON_CrossProduct(m_plane.xaxis,m_plane.yaxis);
      if ( N.Length() <= 1.0e-4 )
      {
        ON_WARNING("ON_PlaneSurface::GetNurbForm - using invalid surface.");
        rc = true;
      }
    }
  }

  if ( rc ) 
  {
    nurbs.m_dim = 3;
    nurbs.m_is_rat = 0;
    nurbs.m_order[0] = nurbs.m_order[1] = 2;
    nurbs.m_cv_count[0] = nurbs.m_cv_count[1] = 2;
    nurbs.m_cv_stride[1] = nurbs.m_dim;
    nurbs.m_cv_stride[0] = nurbs.m_cv_stride[1]*nurbs.m_cv_count[1];
    nurbs.ReserveCVCapacity(12);
    nurbs.ReserveKnotCapacity(0,2);
    nurbs.ReserveKnotCapacity(1,2);
    nurbs.m_knot[0][0] = m_domain[0][0];
    nurbs.m_knot[0][1] = m_domain[0][1];
    nurbs.m_knot[1][0] = m_domain[1][0];
    nurbs.m_knot[1][1] = m_domain[1][1];
    nurbs.SetCV( 0, 0, PointAt( m_domain[0][0], m_domain[1][0] ));
    nurbs.SetCV( 0, 1, PointAt( m_domain[0][0], m_domain[1][1] ));
    nurbs.SetCV( 1, 0, PointAt( m_domain[0][1], m_domain[1][0] ));
    nurbs.SetCV( 1, 1, PointAt( m_domain[0][1], m_domain[1][1] ));
  }

  return rc;
}

int 
ON_PlaneSurface::HasNurbForm( // returns 0: unable to create NURBS representation
                   //            with desired accuracy.
                   //         1: success - returned NURBS parameterization
                   //            matches the surface's to wthe desired accuracy
                   //         2: success - returned NURBS point locus matches
                   //            the surfaces's to the desired accuracy but, on
                   //            the interior of the surface's domain, the 
                   //            surface's parameterization and the NURBS
                   //            parameterization may not match to the 
                   //            desired accuracy.
        ) const

{
  if (!IsValid())
    return 0;
  return 1;
}

bool ON_PlaneSurface::SetExtents( 
       int dir,
       ON_Interval extents,
       bool bSyncDomain
       )
{
  if ( dir < 0 || dir > 1 || !extents.IsIncreasing() )
    return false;
  m_extents[dir] = extents;
  if ( bSyncDomain )
    m_domain[dir] = m_extents[dir];
  return true;
}

ON_Interval ON_PlaneSurface::Extents(
       int dir
       ) const
{
  // rectangle extents - do not confuse with m_domain
  return dir ? m_extents[1] : m_extents[0];
}

bool ON_PlaneSurface::CreatePseudoInfinitePlane( 
        ON_PlaneEquation plane_equation,
        const ON_BoundingBox& bbox,
        double padding
        )
{
  ON_Plane plane(&plane_equation.x);
  return CreatePseudoInfinitePlane(plane,bbox,padding);
}

bool ON_PlaneSurface::CreatePseudoInfinitePlane( 
        const ON_Plane& plane,
        const ON_BoundingBox& bbox,
        double padding
        )
{
  ON_3dPoint bbox_corners[8];
  if ( !bbox.GetCorners(bbox_corners) )
    return false;
  return CreatePseudoInfinitePlane(plane,8,bbox_corners,padding);
}

bool ON_PlaneSurface::CreatePseudoInfinitePlane( 
        const ON_Plane& plane,
        int point_count,
        const ON_3dPoint* point_list,
        double padding
        )
{
  if ( !plane.IsValid() )
    return false;
  if ( point_count < 1 )
    return false;
  if ( 0 == point_list )
    return false;
  if ( !ON_IsValid(padding) || padding < 0.0 )
    return false;

  ON_Interval plane_domain[2];
  double s, t;
  s = ON_UNSET_VALUE;
  t = ON_UNSET_VALUE;
  if ( !plane.ClosestPointTo( point_list[0], &s, &t ) || !ON_IsValid(s) || !ON_IsValid(t) )
    return 0;
  plane_domain[0].m_t[1] = plane_domain[0].m_t[0] = s;
  plane_domain[1].m_t[1] = plane_domain[1].m_t[0] = t;
  
  for ( int i = 1; i < point_count; i++ )
  {
    s = ON_UNSET_VALUE;
    t = ON_UNSET_VALUE;
    if ( !plane.ClosestPointTo( point_list[i], &s, &t ) || !ON_IsValid(s) || !ON_IsValid(t) )
      return 0;
    if ( s < plane_domain[0].m_t[0] ) plane_domain[0].m_t[0] = s; else if ( s > plane_domain[0].m_t[1] ) plane_domain[0].m_t[1] = s;
    if ( t < plane_domain[1].m_t[0] ) plane_domain[1].m_t[0] = t; else if ( t > plane_domain[1].m_t[1] ) plane_domain[1].m_t[1] = t;
  }

  s = padding*plane_domain[0].Length() + padding;
  if ( !(s > 0.0) && !plane_domain[0].IsIncreasing() )
    s = 1.0;
  plane_domain[0].m_t[0] -= s;
  plane_domain[0].m_t[1] += s;

  t = padding*plane_domain[1].Length() + padding;
  if ( !(t > 0.0) && !plane_domain[1].IsIncreasing() )
    t = 1.0;
  plane_domain[1].m_t[0] -= t;
  plane_domain[1].m_t[1] += t;

  m_plane = plane;
  m_domain[0] = plane_domain[0];
  m_domain[1] = plane_domain[1];
  m_extents[0] = plane_domain[0];
  m_extents[1] = plane_domain[1];

  return IsValid()?true:false;
}



ON_BOOL32 ON_PlaneSurface::SetDomain( 
  int dir, 
  double t0, 
  double t1
  )
{
  bool rc = false;
  if ( dir >= 0 && dir <= 1 && t0 < t1 )
  {
    rc = true;
    m_domain[dir].Set(t0,t1);
    DestroySurfaceTree();
  }
  return rc;
}

void ON_ClippingPlaneInfo::Default()
{
  memset(this,0,sizeof(*this));
}

bool ON_ClippingPlaneInfo::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;
  
  for(;;)
  {
    rc = file.WritePlaneEquation(m_plane_equation);
    if (!rc) break;

    rc = file.WriteUuid(m_plane_id);
    if (!rc) break;

    rc = file.WriteBool(m_bEnabled);
    if (!rc) break;

    break;
  }

  if ( !file.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

bool ON_ClippingPlaneInfo::Read( ON_BinaryArchive& file )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;
  
  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = file.ReadPlaneEquation(m_plane_equation);
    if (!rc) break;

    rc = file.ReadUuid(m_plane_id);
    if (!rc) break;

    rc = file.ReadBool(&m_bEnabled);
    if (!rc) break;

    break;
  }

  if ( !file.EndRead3dmChunk() )
    rc = false;

  return rc;
}


void ON_ClippingPlane::Default()
{
  m_plane = ON_xy_plane;
  m_viewport_ids.Empty();
  m_plane_id = ON_nil_uuid;
  m_bEnabled = true;
}

ON_ClippingPlane::ON_ClippingPlane()
{
  Default();
}

ON_ClippingPlane::~ON_ClippingPlane()
{
}

ON_ClippingPlaneInfo ON_ClippingPlane::ClippingPlaneInfo() const
{
  ON_ClippingPlaneInfo info;
  info.m_plane_equation = m_plane.plane_equation;
  info.m_plane_id = m_plane_id;
  info.m_bEnabled = m_bEnabled;
  return info;
}

bool ON_ClippingPlane::Read( ON_BinaryArchive& file )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    ON_UUID viewport_id;
    rc = file.ReadUuid(viewport_id);
    if(!rc) break;

    if( 0 == minor_version )
      m_viewport_ids.AddUuid( viewport_id );

    rc = file.ReadUuid(m_plane_id);
    if (!rc) break;

    rc = file.ReadPlane(m_plane);
    if (!rc) break;

    rc = file.ReadBool(&m_bEnabled);
    if (!rc) break;

    if( minor_version > 0 )
    {
      rc = m_viewport_ids.Read(file);
      if (!rc) break;
    }

    break;
  }

  if ( !file.EndRead3dmChunk() )
    rc = false;

  return rc;
}

bool ON_ClippingPlane::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if (!rc)
    return false;

  for(;;)
  {
    //version 1.1 - write list of viewport uuids instead of single uuid
    ON_UUID viewport_id = ::ON_nil_uuid;
    if( m_viewport_ids.Count() > 0 )
      viewport_id = *(m_viewport_ids.Array());
    rc = file.WriteUuid(viewport_id);
    if (!rc) break;

    rc = file.WriteUuid(m_plane_id);
    if (!rc) break;

    rc = file.WritePlane(m_plane);
    if (!rc) break;

    rc = file.WriteBool(m_bEnabled);
    if (!rc) break;

    //version 1.1 - write list of viewport uuids instead of single uuid
    rc = m_viewport_ids.Write(file);
    if (!rc) break;

    break;
  }

  if ( !file.EndWrite3dmChunk() )
    rc = false;

  return rc;
}


void ON_ClippingPlaneSurface::Default()
{
  m_clipping_plane.Default();
  m_plane = m_clipping_plane.m_plane;
  m_domain[0].Set(0.0,1.0);
  m_domain[1].Set(0.0,1.0);
  m_extents[0].Set(-1.0,1.0);
  m_extents[1].Set(-1.0,1.0);
}


ON::object_type ON_ClippingPlaneSurface::ObjectType() const
{
  return ON::clipplane_object;
}

ON_ClippingPlaneSurface::ON_ClippingPlaneSurface()
{
  Default();
}

ON_ClippingPlaneSurface::~ON_ClippingPlaneSurface()
{
}

ON_ClippingPlaneSurface::ON_ClippingPlaneSurface(const ON_PlaneSurface& src)
                        : ON_PlaneSurface(src)
{
  m_clipping_plane.m_plane = m_plane;
}

ON_ClippingPlaneSurface::ON_ClippingPlaneSurface(const ON_Plane& src)
                        : ON_PlaneSurface(src)
{
  m_clipping_plane.m_plane = m_plane;
}

ON_ClippingPlaneSurface& ON_ClippingPlaneSurface::operator=(const ON_Plane& src)
{
  m_plane = src;
  m_clipping_plane.m_plane = m_plane;
  return *this;
}

ON_ClippingPlaneSurface& ON_ClippingPlaneSurface::operator=(const ON_PlaneSurface& src)
{
  if ( this != &src )
  {
    ON_PlaneSurface::operator=(src);
    m_clipping_plane.m_plane = m_plane;
  }
  return *this;
}

unsigned int ON_ClippingPlaneSurface::SizeOf() const
{
  return ON_PlaneSurface::SizeOf() + sizeof(m_clipping_plane);
}

ON__UINT32 ON_ClippingPlaneSurface::DataCRC(ON__UINT32 current_remainder) const
{
  ON__UINT32 crc = ON_PlaneSurface::DataCRC(current_remainder);
  crc = ON_CRC32(crc,sizeof(m_clipping_plane),&m_clipping_plane);
  return crc;
}

void ON_ClippingPlaneSurface::Dump( ON_TextLog& text_log ) const
{
  text_log.Print("Clipping plane surface\n");
  text_log.PushIndent();  
  text_log.Print("Enabled = %d",m_clipping_plane.m_bEnabled);
  text_log.Print("View IDs =\n");
  {
    text_log.PushIndent();
    ON_SimpleArray<ON_UUID> uuid_list;
    m_clipping_plane.m_viewport_ids.GetUuids(uuid_list);
    for( int i=0; i<uuid_list.Count(); i++ )
    {
      text_log.Print( uuid_list[i] );
      text_log.Print("\n");
    }
    text_log.PopIndent();
  }
  text_log.Print("Plane ID = ");
  text_log.Print(m_clipping_plane.m_plane_id);
  text_log.Print("\n");  

  text_log.Print("Plane surface\n");
  text_log.PushIndent();  
  ON_PlaneSurface::Dump(text_log);
  text_log.PopIndent();  
  text_log.PopIndent();  
}

ON_BOOL32 ON_ClippingPlaneSurface::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;

  for(;;)
  {
    rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,0);
    if (rc)
    {
      rc = ON_PlaneSurface::Write(file)?true:false;
      if (!file.EndWrite3dmChunk())
        rc = false;
    }
    if (!rc) break;

    rc = m_clipping_plane.Write(file);
    if (rc) break;

    break;
  }

  if (!file.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

ON_BOOL32 ON_ClippingPlaneSurface::Read( ON_BinaryArchive& file )
{
  Default();

  int major_version = 0;
  int minor_version = 0;

  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    rc =  ( 1 == major_version );
    if (!rc) break;

    ON__UINT32 tcode = 0;
    ON__INT64 big_value = 0;

    rc = file.BeginRead3dmBigChunk(&tcode,&big_value)?true:false;
    if (rc)
    {
      rc = (TCODE_ANONYMOUS_CHUNK == tcode);
      if (rc)
        rc = (ON_PlaneSurface::Read(file)?true:false);
      if (!file.EndRead3dmChunk())
        rc = false;
    }
    if (!rc) break;

    rc = m_clipping_plane.Read(file);
    if (rc) break;

    break;
  }

  if (!file.EndRead3dmChunk() )
    rc = false;

  return rc;
}



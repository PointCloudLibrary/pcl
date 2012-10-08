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

bool ON_Brep::IsValidForV2( const ON_BrepTrim& trim ) const
{
  int ti = trim.m_trim_index;
  if ( ti < 0 || ti >= m_T.Count() )
    return false;
  if ( &trim != &m_T[ti] )
    return false;
  if ( trim.ProxyCurveIsReversed() )
    return false;
  if ( trim.Domain() != trim.ProxyCurveDomain() )
    return false;
  const ON_Curve * curve = trim.TrimCurveOf();
  if ( curve != trim.ProxyCurve() )
    return false;
  const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(curve);
  if ( 0 == nurbs_curve )
    return false;
  if ( !nurbs_curve->IsClamped(2) )
    return false;
  if ( nurbs_curve->m_dim != 2 )
    return false;
  if ( nurbs_curve->m_is_rat )
  {
    // 2 June 2003 Dale Lear - RR 8809 fix
    //    V2 likes end weights to be 1.0
    if ( nurbs_curve->m_cv[2] != 1.0 || nurbs_curve->CV(nurbs_curve->m_cv_count-1)[2] != 1.0 )
    {
      return false;
    }
  }

  if (    nurbs_curve->m_cv_count >= 4 
       && 0 == ON_ComparePoint( nurbs_curve->m_dim, nurbs_curve->m_is_rat, nurbs_curve->m_cv, nurbs_curve->CV(nurbs_curve->m_cv_count-1) ) 
       )
  {
    // 14 April 2003 Dale Lear
    //     RR 8843 - V2 wants ends of this trim farther apart
    if ( trim.m_vi[0] != trim.m_vi[1] )
    {
      const ON_BrepLoop* loop = Loop(trim.m_li);
      if ( 0 != loop && loop->m_ti.Count() > 1 )
        return false;
    }
  }
  
  if ( curve->Domain() != trim.Domain() )
    return false;

  return true;
}

bool ON_Brep::IsValidForV2( const ON_BrepEdge& edge ) const
{
  int ei = edge.m_edge_index;
  if ( ei < 0 || ei >= m_E.Count() )
    return false;
  if ( &edge != &m_E[ei] )
    return false;
  if ( edge.ProxyCurveIsReversed() )
    return false;
  if ( edge.Domain() != edge.ProxyCurveDomain() )
    return false;
  const ON_Curve * curve = edge.EdgeCurveOf();
  if ( curve != edge.ProxyCurve() )
    return false;
  const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(curve);
  if ( 0 == nurbs_curve )
    return false;
  if ( !nurbs_curve->IsClamped(2) )
    return false;
  if ( nurbs_curve->m_dim != 3 )
    return false;
  if ( nurbs_curve->m_is_rat )
  {
    // 2 June 2003 Dale Lear - RR 8809 fix
    //    V2 likes end weights to be 1.0
    if ( nurbs_curve->m_cv[3] != 1.0 || nurbs_curve->CV(nurbs_curve->m_cv_count-1)[3] != 1.0 )
    {
      return false;
    }
  }

  if ( curve->Domain() != edge.Domain() )
    return false;

  // 14 April 2003 Dale Lear
  //     RR 8808 - V2 requires edges to be strictly closed/open
  if (    nurbs_curve->m_cv_count >= 4 
       && 0 == ON_ComparePoint( nurbs_curve->m_dim, nurbs_curve->m_is_rat, nurbs_curve->m_cv, nurbs_curve->CV(nurbs_curve->m_cv_count-1) ) 
       )
  {
    if ( edge.m_vi[0] != edge.m_vi[1] )
      return false;
  }
  else if (edge.m_vi[0] == edge.m_vi[1] )
  {
    return false;
  }

  return true;
}

bool ON_Brep::IsValidForV2() const
{
  bool rc = IsValidTopology()?true:false;
  if ( rc )
  {
    int c2i, c3i, si, ti, li, ei, vi, fi, next_ti, lti, next_lti, loop_trim_count;
    ON_3dPoint P0, P1;

    const int c2_count = m_C2.Count();
    const int c3_count = m_C3.Count();
    const int s_count  = m_S.Count();
    const int vertex_count = m_V.Count();
    const int edge_count = m_E.Count();
    const int face_count = m_F.Count();
    const int loop_count = m_L.Count();
    const int trim_count = m_T.Count();

    for ( c2i = 0; c2i < c2_count; c2i++ )
    {
      // v2 3dm files expect NURBS curves
      if ( !ON_NurbsCurve::Cast(m_C2[c2i]) )
        return false;
    }

    for ( c3i = 0; c3i < c3_count; c3i++ )
    {
      // v2 3dm files expect NURBS curves
      if ( !ON_NurbsCurve::Cast(m_C3[c3i]) )
        return false;
    }

    for ( si = 0; si < s_count; si++ )
    {
      // v2 3dm files expect NURBS surfaces
      if ( !ON_NurbsSurface::Cast(m_S[si]) )
        return false;
    }

    for ( vi = 0; vi < vertex_count; vi++ )
    {
      const ON_BrepVertex& vertex = m_V[vi];
      if ( vertex.m_vertex_index != vi )
        return false;
    }

    for ( fi = 0; fi < face_count; fi++ )
    {
      const ON_BrepFace& face = m_F[fi];
      if ( face.m_face_index != fi )
        return false;
    }

    for ( ti = 0; ti < trim_count; ti++ )
    {
      if ( !IsValidForV2( m_T[ti] ) )
        return false;
    }

    for ( ei = 0; ei < edge_count; ei++ )
    {
      if ( !IsValidForV2(m_E[ei]) )
        return false;
    }

    for ( li = 0; li < loop_count; li++ )
    {
      const ON_BrepLoop& loop = m_L[li];
      if ( loop.m_loop_index == -1 )
        return false;
      loop_trim_count = loop.m_ti.Count();
      for ( lti = 0; lti < loop_trim_count; lti++ )
      {
        next_lti = (lti+1)%loop_trim_count;
        ti = loop.m_ti[lti];
        next_ti = loop.m_ti[next_lti];
        if ( ti < 0 || ti >= trim_count )
          return false;
        if ( next_ti < 0 || next_ti >= trim_count )
          return false;
        P0 = m_T[ti].PointAtEnd();
        P1 = m_T[next_ti].PointAtStart();
        if ( P0.DistanceTo(P1) > ON_ZERO_TOLERANCE )
          return false;
      }
    }
  }

  return rc;
}


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

#if !defined(ON_POLYEDGECURVE_INC_)
#define ON_POLYEDGECURVE_INC_

class ON_PolyEdgeSegment;

class ON_CLASS ON_PolyEdgeCurve : public ON_PolyCurve
{
  ON_OBJECT_DECLARE(ON_PolyEdgeCurve);

public:
  ON_PolyEdgeCurve();
  ~ON_PolyEdgeCurve();
  // default copy constructor and operator= are fine.

  // virtual ON_Object::DestroyRuntimeCache override
  void DestroyRuntimeCache( bool bDelete = true );
};

class ON_CLASS ON_PolyEdgeSegment : public ON_CurveProxy
{
  ON_OBJECT_DECLARE(ON_PolyEdgeSegment);
public:
  // construction

  ON_PolyEdgeSegment();
  ~ON_PolyEdgeSegment();
  // default copy constructor and operator= are fine.

  // virtual ON_Object::DestroyRuntimeCache override
  void DestroyRuntimeCache( bool bDelete = true );

  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  ON_BOOL32 Read( ON_BinaryArchive& );

  // m_object_id = id of a brep or curve object in Rhino
  ON_UUID m_object_id; 
  // When the Rhino object is a brep, m_component_index
  // refers to either an edge or a trim.
  ON_COMPONENT_INDEX m_component_index;
  // corresponding domain of the edge - see note below
  ON_Interval m_edge_domain;  
  // corresponding domain of the trim - see note below
  ON_Interval m_trim_domain;   


  // When m_component_index refers to an ON_BrepTrim, there
  // are four domains and 4 classes derived from ON_Curve
  // that play a role in the polyedge segment.  It is possible
  // for all 4 of these domains to be different.
  //
  // "this" ON_PolyEdgeSegment is an ON_ProxyCurve.  The
  // evaluation domain of "this" is 
  //   = this->Domain()
  //   = ON_ProxyCurve::m_this_domain
  //
  // ON_ProxyCurve::m_real_curve points to the curve in the
  // c3 = ON_Brep::m_C3[edge.m_c3i].  "this" is a proxy for some 
  // sub-interval of c3.
  //   = this->ProxyCurveDomain()
  //   = ON_ProxyCurve::m_real_curve_domain
  //
  // The edge, an ON_BrepEdge, is also a proxy based on c3,
  // and the edge's evaluation domain is edge.m_this_domain.  
  // ON_PolyEdgeSegment::m_edge_domain records the increasing
  // subinterval of edge.m_this_domain that corresponds
  // to the portion of c3 "this" is using.
  // 
  // The trim, an ON_BrepTrim, is a proxy based on a curve
  // in ON_Brep::m_C2[].  Some portion of the trim corresponds
  // to the portion of the edge we are using.  m_trim_domain
  // is an increasing, possible subinterval, of the trim's domain
  // ON_BrepTrim::m_this_domain.

  // Runtime helpers
  const void* m_parent_object; // CRhinoBrepObject or CRhinoCurveObject
  const ON_Brep* m_brep;
  const ON_BrepTrim* m_trim;  // 2d trim in m_brep
  const ON_BrepEdge* m_edge;  // 3d edge in m_brep
  const ON_BrepFace* m_face;
  const ON_Surface* m_surface;

private:
  friend class ON_PolyEdgeCurve;
  void ClearEvalCacheHelper();

  // parameter evaluation cache
  double m_t;
  double m_edge_t;
  double m_trim_t;
  double m_srf_uv[2];
  int m_trim_hint;
  int m_edge_hint;

  // surface evaluation cache
  int m_evsrf_hint[2];
  double m_evsrf_uv[2];
  ON_3dPoint m_evsrf_pt;
  ON_3dVector m_evsrf_du;
  ON_3dVector m_evsrf_dv;
  ON_3dVector m_evsrf_duu;
  ON_3dVector m_evsrf_duv;
  ON_3dVector m_evsrf_dvv;
  ON_3dVector m_evsrf_tan;

  void Init();
};

#endif

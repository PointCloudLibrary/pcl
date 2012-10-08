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

  // virtual ON_Curve::IsClosed override
  ON_BOOL32 IsClosed() const;

  // virtual ON_Curve::DuplicateCurve override
  ON_Curve* DuplicateCurve() const;

  /*
  Description:
    Create a one segment ON_PolyEdgeCurve curve that uses a 
    single edge.
  */
  bool Create(
           const ON_BrepTrim* trim,
           const ON_UUID& object_id
           );

  /*
  Description:
    Create a one segment ON_PolyEdgeCurve curve that uses a 
    single curve.
  */
  bool Create(
           const ON_Curve* curve,
           const ON_UUID& object_id
           );

  int SegmentCount() const;

  ON_PolyEdgeSegment* SegmentCurve(
    int segment_index
    ) const;

  ON_PolyEdgeSegment* operator[](int) const;

  ON_BOOL32 Prepend( ON_PolyEdgeSegment* new_segment ); // Prepend curve.
  ON_BOOL32 Append( ON_PolyEdgeSegment* new_segment );  // Append curve.
  ON_BOOL32 Insert( 
           int segment_index,
           ON_PolyEdgeSegment* new_segment
           );

  // if the segment is an edge, the following
  // return non-NULL pointers.
  const ON_BrepEdge* EdgeAt(double t) const;
  const ON_BrepTrim* TrimAt(double t) const;
  const ON_Brep*     BrepAt(double t) const;
  const ON_BrepFace* FaceAt(double t) const;
  const ON_Surface*  SurfaceAt(double t) const;
  ON_Surface::ISO    IsoType( double t) const;

  double EdgeParameter(double t) const;

  // Test if there are any surface edges in the polyedge
  bool ContainsAnyEdges() const;
  // Test if all segments of the polyedge are surface edges
  bool ContainsAllEdges() const;
  
  /*
  Description:
    See if this polyedge has an edge as one of its segments
  Parameters:
    edge - [in] the edge to look for
  Returns:
     -1: edge is not in the polyedge
    >=0: index of first segment that uses the edge
  */
  int FindEdge( const ON_BrepEdge* edge) const;

  /*
  Description:
    See if this polyedge has a trim as one of its segments
  Parameters:
    trim - [in] the trim to look for
  Returns:
     -1: trim is not in the polyedge
    >=0: index of first segment that uses the trim
  */
  int FindTrim( const ON_BrepTrim* trim) const;

  /*
  Description:
    See if this polyedge has a wire curve as one of its segments
  Parameters:
    curve - [in] the curve to look for
  Returns:
     -1: trim is not in the polyedge
    >=0: index of first segment that uses the curve
  */
  int FindCurve( const ON_Curve* curve) const;


  // OBSOLETE and unreliable.  Use FindCurve, FindEdge, or FindTrim
  //bool Contains( const ON_Curve* curve) const;

  // virtual ON_Curve overrides do nothing
  // to prevent changing edge
  ON_BOOL32 SetStartPoint(ON_3dPoint start_point);
  ON_BOOL32 SetEndPoint(ON_3dPoint end_point);
  ON_BOOL32 ChangeClosedCurveSeam( double t );
  ON_BOOL32 PrependAndMatch(ON_Curve*);
  ON_BOOL32 AppendAndMatch(ON_Curve*);

  // 7-1-03 lw added override to unset cached closed flag
  // when a segment is removed
  ON_BOOL32 Remove(); // remove last
  ON_BOOL32 Remove( int index);
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

  // virtual ON_Curve::IsClosed override
  ON_BOOL32 IsClosed() const;


  // virtual ON_Curve::DuplicateCurve override
  ON_Curve* DuplicateCurve() const;

  /*
  Description:
    Creates a polyedge segment that uses the entire edge
    and has the same domain as the edge.
  Parameters:
    trim - [in] 
  Returns:
    true if successful (edge was valid and trim_index was valid)
  Remarks:
    Use ON_Curve::SetDomain, ON_Curve::Trim, ON_Curve::Reverse,
    etc., to tweak the domain, support, direction etc.
  */
  bool Create( 
          const ON_BrepTrim* trim,
          const ON_UUID& object_id
          );

  /*
  Description:
    Creates a polyedge segment that uses the entire curve
    and has the same domain as the curve.
  Parameters:
    curve - [in] 
  Remarks:
    Use ON_Curve::SetDomain, ON_Curve::Trim, ON_Curve::Reverse,
    etc., to tweak the domain, support, direction etc.
  */
  bool Create( 
          const ON_Curve* curve,
          const ON_UUID& object_id
          );

  const ON_BrepEdge* Edge() const;
  const ON_BrepTrim* Trim() const;
  const ON_Brep*     Brep() const;
  const ON_BrepFace* Face() const;
  const ON_Surface*  Surface() const;
  ON_Surface::ISO    IsoType() const;

  double EdgeParameter(double t) const;

  /*
  Returns:
    True if this segment has an ON_BrepEdge and the direction of
    the ON_BrepEdge is the reverse of the direction of the segment.
  */
  bool ReversedEdgeDir() const;

  /*
  Returns:
    True if this segment has an ON_BrepTrim and the direction of
    the ON_BrepTrime is the reverse of the direction of the segment.
  */
  bool ReversedTrimDir() const;

  /*
  Returns:
    subdomain of the edge that this segment uses.  This can
    be different than the domain returned by this->Domain().
  */
  ON_Interval EdgeDomain() const;

  /*
  Returns:
    subdomain of the trim that this segment uses.  This can
    be different than the domain returned by this->Domain().
  */
  ON_Interval TrimDomain() const;

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

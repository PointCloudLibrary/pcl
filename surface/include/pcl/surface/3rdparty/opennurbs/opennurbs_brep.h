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

////////////////////////////////////////////////////////////////
//
//   Definition of b-rep and its parts
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_BREP_INC_)
#define OPENNURBS_BREP_INC_

class ON_BrepTrim;
class ON_BrepEdge;
class ON_BrepLoop;
class ON_BrepFace;


// TEMPORARY DEFINES SO I DON'T BREAK THE BUILD
#define m_vertex_user_i m_vertex_user.i
#define m_trim_user_i m_trim_user.i
#define m_edge_user_i m_edge_user.i
#define m_loop_user_i m_loop_user.i
#define m_face_user_i m_face_user.i

// Description:
//   Brep vertex information is stored in ON_BrepVertex classes.
//   ON_Brep.m_V[] is an array of all the vertices in the brep.
//
//   If a vertex is a point on a face, then brep.m_E[m_ei]
//   will be an edge with no 3d curve.  This edge will have
//   a single trim with type ON_BrepTrim::ptonsrf.  There
//   will be a loop containing this single trim.
//   Use ON_Brep::NewPointOnFace() to create vertices that are
//   points on faces. 
class ON_CLASS ON_BrepVertex : public ON_Point
{
  ON_OBJECT_DECLARE(ON_BrepVertex);

public:
  // Union available for application use.
  // The constructor zeros m_vertex_user.
  // The value is of m_vertex_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_vertex_user; 

  // index of the vertex in the ON_Brep.m_V[] array
  int m_vertex_index;

  /////////////////////////////////////////////////////////////////
  // Construction
  //
  // In general, you should not directly create ON_BrepVertex classes.
  // Use ON_Brep::NewVertex instead.
  ON_BrepVertex();
  ON_BrepVertex(
    int // vertex index
    );
  ON_BrepVertex& operator=(const ON_BrepVertex&);

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // virtual ON_Object::Dump() override
  void Dump( ON_TextLog& ) const; // for debugging

  // virtual ON_Object::Write() override
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  // virtual ON_Object::Read() override
  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Geometry::ComponentIndex() override
  ON_COMPONENT_INDEX ComponentIndex() const;

  /////////////////////////////////////////////////////////////////
  // Interface

  // Description:
  //   Set vertex location.
  // Parameters:
  //   point - [in] 3d vertex location
  bool SetPoint( 
          const ON_3dPoint& // point
          );

  // Returns:
  //   Vertex location.
  ON_3dPoint Point() const;

  // Returns:
  //   value of ON_BrepVertex::m_tolerance
  // Remarks:
  //   Use ON_Brep::SetVertexTolerance( ON_BrepVertex& ) to set tolerances.
  double Tolerance() const;

  // Returns:
  //   number of edges that begin or end at this vertex.
  int EdgeCount() const;

  /////////////////////////////////////////////////////////////////
  // Implementation

  // indices of edges starting/ending at this vertex
  //
  // For closed edges, edge.m_vi[0] = edge.m_vi[1] and 
  // edge.m_edge_index appears twice in the m_ei[] array.
  // The first occurance of edge.m_edge_index in m_ei[]
  // is for the closed edge starting the vertex.
  // The second occurance of edge,m_edge_index in m_ei[]
  // is for the closed edge ending at the vertex.
  // C.f. ON_Brep::Next/PrevEdge().
  ON_SimpleArray<int> m_ei;

  // accuracy of vertex point (>=0.0 or ON_UNSET_VALUE)
  //
  // A value of ON_UNSET_VALUE indicates that the
  // tolerance should be computed.
  //
  // A value of 0.0 indicates that the distance
  // from the vertex to any applicable edge or trim
  // end is <=  ON_ZERO_TOLERANCE
  //
  // If an edge begins or ends at this vertex,
  // then the distance from the vertex's 
  // 3d point to the appropriate end of the
  // edge's 3d curve must be <= this tolerance.
  //
  // If a trim begins or ends at this vertex,
  // then the distance from the vertex's 3d point
  // to the 3d point on the surface obtained by
  // evaluating the surface at the appropriate
  // end of the trimming curve must be <= this
  // tolerance.
  double m_tolerance;

private:
  ON_BrepVertex( const ON_BrepVertex& ); // no implementation

};

/*
Description:
  Brep edge information is stored in ON_BrepEdge classes.
  ON_Brep.m_E[] is an array of all the edges in the brep.

  An ON_BrepEdge is derived from ON_CurveProxy so the the
  edge can supply easy to use evaluation tools via 
  the ON_Curve virtual member functions.

  Note well that the domains and orientations of the curve
  m_C3[edge.m_c3i] and the edge as a curve may not
  agree.
*/
class ON_CLASS ON_BrepEdge : public  ON_CurveProxy
{
  ON_OBJECT_DECLARE(ON_BrepEdge);
public:

  // Union available for application use.
  // The constructor zeros m_edge_user.
  // The value is of m_edge_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_edge_user;

  // index of edge in ON_Brep.m_E[] array
  int m_edge_index;    


  // virtual ON_Curve::IsClosed override
  ON_BOOL32 IsClosed() const;

  /////////////////////////////////////////////////////////////////
  // Construction
  //
  // In general, you should not directly create ON_BrepEdge classes.
  // Use ON_Brep::NewVertex instead.
  ON_BrepEdge();
  ON_BrepEdge(int); // edge index
  ON_BrepEdge& operator=(const ON_BrepEdge&);

  // virtual ON_Object function
  // The ON_BrepEdge override returns ON::curve_object.
  ON::object_type ObjectType() const;

  /*
  Returns:
    Brep this edge belongs to.
  */
  ON_Brep* Brep() const;


  /*
  Parameters:
    eti - [in] index into the edge's m_ti[] array.
  Returns:
    The trim brep.m_T[edge.m_ti[eti]];
  */
  ON_BrepTrim* Trim( int eti ) const;

  /*
  Returns:
    Number of trims attached to this edge.
  */
  int TrimCount() const;

  /*
  Parameters:
    evi - [in] 0 or 1
  Returns:
    Brep vertex at specified end of the edge.
  */
  ON_BrepVertex* Vertex(int evi) const;

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // virtual ON_Object::Dump() override
  void Dump( ON_TextLog& ) const; // for debugging

  // virtual ON_Object::Write() override
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  // virtual ON_Object::Read() override
  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Geometry::ComponentIndex() override
  ON_COMPONENT_INDEX ComponentIndex() const;

  // virtual ON_Curve::Reverse override
  ON_BOOL32 Reverse();

  // virtual ON_Curve::SetStartPoint override
  ON_BOOL32 SetStartPoint(
          ON_3dPoint start_point
          );

  // virtual ON_Curve::SetEndPoint override
  ON_BOOL32 SetEndPoint(
          ON_3dPoint end_point
          );

  /////////////////////////////////////////////////////////////////
  // Implementation

  /*
  Returns:
    brep.m_C3[] index of the 3d curve geometry used by this edge 
    or -1.
  */
  int EdgeCurveIndexOf() const;

  /*
  Returns:
    3d curve geometry used by this edge or NULL.
  */
  const ON_Curve* EdgeCurveOf() const;

  /*
  Description:
    Expert user tool that replaces the 3d curve geometry
    of an edge
  Parameters;
    c3i - [in] brep 3d curve index of new curve
  Returns:
    True if successful.
  Example:

            ON_Curve* pCurve = ...;
            int c3i = brep.AddEdgeCurve(pCurve);
            edge.ChangeEdgeCurve(c3i);

  Remarks:
    Sets m_c3i, calls SetProxyCurve, cleans runtime caches.
  */
  bool ChangeEdgeCurve(
    int c3i 
    );

  /*
  Description:
    When an edge is modified, the m_pline[].e values need
    to be set to ON_UNSET_VALUE by calling UnsetPlineEdgeParameters().
  */
  void UnsetPlineEdgeParameters();

  // index of 3d curve in m_C3[] array
  // (edge.m_curve also points to m_C3[m_c3i])
  int m_c3i;

  // indices of starting/ending vertex
  //
  // For closed edges, m_vi[0] = m_vi[1] and m_edge_index
  // appears twice in the m_V[m_vi[0]].m_ei[] array.
  // The first occurance of m_edge_index in m_V[m_vi[0]].m_ei[]
  // is for the closed edge starting the vertex.  The second
  // occurance of m_edge_index in m_V[m_vi[0]].m_ei[]
  // is for the closed edge edge ending at the vertex.
  // C.f. ON_Brep::Next/PrevEdge().
  int m_vi[2];

  // indices of Trims that use this edge
  ON_SimpleArray<int> m_ti;

  // accuracy of edge curve (>=0.0 or ON_UNSET_VALUE)
  //
  // A value of ON_UNSET_VALUE indicates that the
  // tolerance should be computed.
  //
  // The maximum distance from the edge's 3d curve
  // to any surface of a face that has this edge as
  // a portion of its boundary must be <= this
  // tolerance.
  double m_tolerance;

private:
  friend class ON_Brep;
  ON_Brep* m_brep; // so isolated edge class edge can get at it's 3d curve
  ON_BrepEdge( const ON_BrepEdge& ); // no implementation
};

struct ON_BrepTrimPoint
{
  ON_2dPoint p; // 2d surface parameter space point
  double t;     // corresponding trim curve parameter
  double e;     // corresponding edge curve parameter (ON_UNSET_VALUE if unknown)
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_BrepTrimPoint>;
#pragma warning( pop )
#endif


/*
Description:
  Brep trim information is stored in ON_BrepTrim classes.
  ON_Brep.m_T[] is an array of all the trim in the brep.

  An ON_BrepTrim is derived from ON_CurveProxy so the the
  trim can supply easy to use evaluation tools via 
  the ON_Curve virtual member functions.

  Note well that the domains and orientations of the curve
  m_C2[trim.m_c2i] and the trin as a curve may not
  agree.
*/
class ON_CLASS ON_BrepTrim : public  ON_CurveProxy
{
  ON_OBJECT_DECLARE(ON_BrepTrim);

public:
  void DestroyRuntimeCache( bool bDelete = true );

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // Union available for application use.
  // The constructor zeros m_trim_user.
  // The value is of m_trim_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_trim_user;

  int m_trim_index;  // index of trim in ON_Brep.m_T[] array

  // types of trim - access through m_type member.  Also see m_iso and ON_Surface::ISO
  enum TYPE 
  {
    unknown  = 0,
    boundary = 1,       // trim is connected to an edge, is part of an outer, 
                        // inner or slit loop, and is the only trim connected
                        // to the edge.
    mated    = 2,       // trim is connected to an edge, is part of an outer,
                        // inner or slit loop, no other trim from the same 
                        // loop is connected to the edge, and at least one 
                        // trim from a different loop is connected to the edge.
    seam     = 3,       // trim is connected to an edge, is part of an outer, 
                        // inner or slit loop, and one other trim from the 
                        // same loop is connected to the edge.
                        // (There can be other mated trims that are also
                        // connected to the edge.  For example, the non-mainfold
                        // edge that results when a surface edge lies in the
                        // middle of another surface.)  Non-mainfold "cuts"
                        // have seam trims too.
    singular = 4,       // trim is part of an outer loop, the trim's 2d curve
                        // runs along the singular side of a surface, and the
                        // trim is NOT connected to an edge. (There is no 3d
                        // edge because the surface side is singular.)
    crvonsrf = 5,       // trim is connected to an edge, is the only trim in
                        // a crfonsrf loop, and is the only trim connected to
                        // the edge.
    ptonsrf  = 6,       // trim is a point on a surface, trim.m_pbox is records
                        // surface parameters, and is the only trim
                        // in a ptonsrf loop.  This trim is not connected
                        // to an edge and has no 2d curve.
    slit     = 7,       // 17 Nov 2006 - reserved for future use
                        //   currently an invalid value
    trim_type_count = 8,
    force_32_bit_trim_type = 0xFFFFFFFF
  };

  /////////////////////////////////////////////////////////////////
  // Construction
  //
  // In general, you should not directly create ON_BrepTrim classes.
  // Use ON_Brep::NewTrim instead.
  ON_BrepTrim();
  ON_BrepTrim(int); // trim index
  ON_BrepTrim& operator=(const ON_BrepTrim&);

  /*
  Returns:
    Brep that this trim belongs to.
  */
  ON_Brep* Brep() const;

  /*
  Returns:
    Brep loop that this trim belongs to.
  */
  ON_BrepLoop* Loop() const;

  /*
  Returns:
    Brep face this trim belongs to.
  */
  ON_BrepFace* Face() const;

  /*
  Returns:
    Brep edge this trim uses or belongs to.  This will
    be NULL for singular trims.
  */
  ON_BrepEdge* Edge() const;

  /*
  Parameters:
    tvi - [in] 0 or 1
  Returns:
    Brep vertex at specified end of the trim.
  */
  ON_BrepVertex* Vertex(int tvi) const;

  /////////////////////////////////////////////////////////////////
  // ON_Object overrides
  //
  // (Trims are purely topologicial - geometry queries should be 
  //  directed at the trim's 2d curve or the trim's edge's 3d curve.)

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const; // for debugging

  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Geometry::ComponentIndex() override
  ON_COMPONENT_INDEX ComponentIndex() const;

  // virtual ON_Curve::Reverse override
  // Reverses curve - caller must make sure trim's m_bRev3d
  // flags are properly updated.  Use
  // ON_Brep::FlipTrim to reverse and trim and update all
  // m_bRev3d informtion.
  ON_BOOL32 Reverse();

  // virtual ON_Curve::SetStartPoint override
  ON_BOOL32 SetStartPoint(
          ON_3dPoint start_point
          );

  // virtual ON_Curve::SetEndPoint override
  ON_BOOL32 SetEndPoint(
          ON_3dPoint end_point
          );

  /////////////////////////////////////////////////////////////////
  // Interface
  
  /*
  Description:
    Expert user tool that replaces the 2d curve geometry
    of a trim
  Parameters;
    c2i - [in] brep 2d curve index of new curve
  Returns:
    True if successful.
  Example:

            ON_Curve* pCurve = ...;
            int c2i = brep.AddTrimCurve(pCurve);
            trim.ChangeTrimCurve(c2i);

  Remarks:
    Sets m_c2i, calls SetProxyCurve, cleans runtime caches,
    and updates m_pbox.
  */
  bool ChangeTrimCurve( int c2i );

  /*
  Description:
    Destroy parameter space information.
    Currently, this involves destroying m_pline
    and m_pbox. Parameter space information should
    be destroyed when the location of a trim
    curve is changed.
  */
  void DestroyPspaceInformation();
  
  /*
  Description:
    Expert user function.
    Removes a trim from an edge.
  Parameters:
    bRemoveFromStartVertex - [in] if true, the trim
      is removed from its start vertex by setting
      m_vi[0] to -1.
    bRemoveFromEndVertex - [in] if true, the trim
      is removed from its start vertex by setting
      m_vi[1] to -1.
  Remarks:
    If the trim is attached to an edge (m_ei>=0), then
    the trim is removed from the edge and the edge's
    m_ti[] list.  The trim's m_bRev3d and tolerance values
    are not changed.
  */
  bool RemoveFromEdge( 
        bool bRemoveFromStartVertex,
        bool bRemoveFromEndVertex
        );

  /*
  Description:
    Expert user function.
    Attaches a trim to an edge.
  Parameters:
    edge_index - [in] index of an edge.
    bRev3d - [in] value for trim's m_bRev3d field.
  Remarks:
    If the trim is attached to an edge (m_ei>=0), then
    the trim is removed from the edge and the edge's
    m_ti[] list.  The trim's tolerance values are not
    changed.
  */
  bool AttachToEdge(
        int edge_index,
        bool bRev3d
        );

  /*
  Returns:
    2d curve geometry used by this trim or NULL
  */
  const ON_Curve* TrimCurveOf() const;

  /*
  Returns:
    3d curve geometry used by this trim or NULL.
  */
  const ON_Curve* EdgeCurveOf() const;

  /*
  Returns:
    3d surface geometry used by this trim or NULL
  */
  const ON_Surface* SurfaceOf() const;

  /*
  Returns:
    brep.m_C2[] 2d curve index of the 2d curve geometry used by 
    this trim or -1.
  */
  int TrimCurveIndexOf() const;

  /*
  Returns:
    brep.m_C3[] 3d curve index of the 3d curve geometry used by 
    this trim or -1.
  */
  int EdgeCurveIndexOf() const;

  /*
  Returns:
    brep.m_S[] surface index of the 3d surface geometry used by 
    this trim or -1.
  */
  int SurfaceIndexOf() const;

  /*
  Returns:
    brep.m_F[] face index of the face used by this trim or -1.
  */
  int FaceIndexOf() const;

  /*
  Returns:
    True if the trim satisfies these four criteria.
      1) is part of a loop
      2) is connected to a 3d edge
      3) one other trim from the same loop is connected to the edge
      4) The 2d trim curve for the other trim is the reverse
         of the 2d trim curve for this trim.
  Remarks:
    In order for IsSlit() to work correctly, the m_type and m_iso
    fields must be set correctly.  In V4 SR1, this function will
    be removed and ON_BrepTrim::slit will be added as a type.
  */
  bool IsSlit() const;

  /*
  Returns:
    True if the trim satisfies these four criteria.
      1) is part of a loop
      2) is connected to a 3d edge
      3) one other trim from the same loop is connected to the edge
      4) the 2d trim curve for this trim lies along the side of 
         the face's parameter space and the 2d curve for the other
         trim lies on the opposite side of the face's parameter
         space.
  Remarks:
    In order for IsSeam() to work correctly, the m_type and m_iso
    fields must be set correctly.  In V4 SR1, this function will
    be removed and ON_BrepTrim::slit will be added as a type.
  */
  bool IsSeam() const;

  /*
  Description:
    Expert user tool that tranforms all the parameter space (2d)
    trimming curves in this loop.  Only 2d curve geometry is
    changed.  The caller is responsible for reversing loops,
    toggle m_bRev, flags, etc.
  Parameters:
    xform - [in] Transformation applied to 2d curve geometry.
  Returns
    True if successful.  If false is returned, the brep
    may be invalid.
  */
  bool TransformTrim( const ON_Xform& xform );

  // index of the 2d parameter space trimming curve
  int m_c2i;

  // index of 3d edge (-1 if ON_BrepTrim is singular)
  int m_ei;

  // Indices of start/end vertices.  Trims along singular
  // sides and trims that correspond to closed 3d edges
  // have m_vi[0] = m_vi[1].  Note that singular trims
  // and trims on the closed edge of a closed surface can
  // have an open 2d trimming curve and still have 
  // m_vi[0] = m_vi[1].
  int m_vi[2];
        
  // true if the 2d trim and 3d edge have opposite orientations.
  bool   m_bRev3d;

  TYPE   m_type;
  ON_Surface::ISO  m_iso;

  // index of loop that uses this trim
  int    m_li;

  // The values in m_tolerance[] record the accuracy of
  // the parameter space trimming curves.
  //
  // Remarks:
  //   m_tolerance[0] = accuracy of parameter space curve
  //   in first ( "u" ) parameter
  //
  //   m_tolerance[1] = accuracy of parameter space curve
  //   in second ( "v" ) parameter
  //
  //   A value of ON_UNSET_VALUE indicates that the 
  //   tolerance should be computed. If the value >= 0.0, 
  //   then the tolerance is set.  If the value is 
  //   ON_UNSET_VALUE, then the tolrance needs to be
  //   computed.
  //
  //   If the trim is not singular, then the trim must
  //   have an edge.  If P is a 3d point on the edge's
  //   curve and surface(u,v) = Q is the point on the 
  //   surface that is closest to P, then there must
  //   be a parameter t in the interval [m_t[0], m_t[1]]
  //   such that
  //
  //   |u - curve2d(t)[0]| <= m_tolerance[0]
  //
  //   and 
  //
  //   |v - curve2d(t)[1]| <= m_tolerance[1]
  //
  //   If P is the 3d point for the vertex brep.m_V[m_vi[k]]
  //   and (uk,vk) is the corresponding end of the trim's
  //   parameter space curve, then there must be a surface
  //   parameter (u,v) such that:
  //
  //   *  the distance from the 3d point surface(u,v) to P
  //      is <= brep.m_V[m_vi[k]].m_tolerance,
  //   *  |u-uk| <= m_tolerance[0].
  //   *  |v-vk| <= m_tolerance[1].
  double m_tolerance[2]; 

  // Runtime polyline approximation of trimming curve.
  // This information is not saved in 3DM archives.
  ON_SimpleArray<ON_BrepTrimPoint> m_pline;

  /*
  Description:
    When an edge is modified, the m_pline[].e values need
    to be set to ON_UNSET_VALUE by calling UnsetPlineEdgeParameters().
  */
  void UnsetPlineEdgeParameters();

  // Runtime parameter space trimming curve bounding box.
  // This information is not saved in 3DM archives.
  ON_BoundingBox m_pbox;

public:
  // values stored in legacy file formats - ignore

  void m__legacy_flags_Set(int,int);   // used internally - ignore
  bool m__legacy_flags_Get(int*,int*) const; // used internally - ignore
  double m__legacy_2d_tol; // used internally - ignore
  double m__legacy_3d_tol; // used internally - ignore
  int    m__legacy_flags;  // used internally - ignore

private:
  friend class ON_Brep;
  ON_Brep* m_brep; // so isolated edge class edge can get at it's 3d curve
  ON_BrepTrim( const ON_BrepTrim& ); // no implementation
};

class ON_CLASS ON_BrepLoop : public  ON_Geometry
{
  ON_OBJECT_DECLARE(ON_BrepLoop);

public:
  void DestroyRuntimeCache( bool bDelete = true );

  // virtual ON_Geometry overrides
  // A loop is derived from ON_Geometry so that is can 
  // be passed around to things that expect ON_Geometry
  // pointers.  It is not a very useful stand-alone object.

  /*
  Description:
    virtual ON_Geometry::Dimension() override.
  Returns:
    2
  */
  int Dimension() const;

  // virtual ON_Geometry::GetBBox() override.
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;

  // virtual ON_Geometry::Transform() override.
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );
public:
  /*
  Returns:
   Brep that the loop belongs to.
  */
  ON_Brep* Brep() const;

  /*
  Returns:
    Brep face this loop belongs to.
  */
  ON_BrepFace* Face() const;

  /*
  Parameters:
    lti - [in] index into the loop's m_ti[] array.
  Returns:
    The trim brep.m_T[loop.m_ti[lti]];
  */
  ON_BrepTrim* Trim( int lti ) const;

  /*
  Returns:
    Number of trims in this loop.
  */
  int TrimCount() const;

  // Union available for application use.
  // The constructor zeros m_loop_user.
  // The value is of m_loop_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_loop_user;

  int m_loop_index;  // index of loop in ON_Brep.m_L[] array

  enum TYPE {
    unknown  = 0,
    outer    = 1,  // 2d loop curves form a simple closed curve with a counterclockwise orientation
    inner    = 2,  // 2d loop curves form a simple closed curve with a clockwise orientation
    slit     = 3,  // always closed - used internally during splitting operations
    crvonsrf = 4,  // "loop" is a curveonsrf made from a single 
                   // (open or closed) trim that is has type ON_BrepTrim::crvonsrf.
    ptonsrf = 5,   // "loop" is a ptonsrf made from a single 
                   // trim that is has type ON_BrepTrim::ptonsrf.
    type_count = 6
  };

  ON_BrepLoop();
  ON_BrepLoop(int); // loop index
  ON_BrepLoop& operator=(const ON_BrepLoop&);

  /////////////////////////////////////////////////////////////////
  // ON_Object overrides
  //
  // (Loops and trims are purely topologicial - geometry queries should be 
  // directed at the trim's 2d curve or the trim's edge's 3d curve.)

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const; // for debugging

  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Geometry::ComponentIndex() override
  ON_COMPONENT_INDEX ComponentIndex() const;

  /////////////////////////////////////////////////////////////////
  // Interface

  //////////
  // Returns the index i such that loop.m_ti[i] = trim.m_trim_index.
  // Returns -1 if the trim is not in this loop
  int IndexOfTrim( const ON_BrepTrim& ) const;

  /*
  Returns:
    brep.m_S[] surface index of the 3d surface geometry used by 
    this loop or -1.
  */
  int SurfaceIndexOf() const;

  /*
  Returns:
    Pointer to the surface geometry used by the loop.   
  */
  const ON_Surface* SurfaceOf() const;

  /*
  Description:
    Expert user tool that tranforms all the parameter space (2d)
    trimming curves in this loop.  Only 2d curve geometry is
    changed.  The caller is responsible for reversing loops,
    toggle m_bRev, flags, etc.
  Parameters:
    xform - [in] Transformation applied to 2d curve geometry.
  Returns
    True if successful.  If false is returned, the brep
    may be invalid.
  */
  bool TransformTrim( const ON_Xform& xform );

  ON_SimpleArray<int> m_ti;   // trim indices
  TYPE         m_type;
  int          m_fi;   // index of face that uses this loop

  //////////
  // parameter space trimming loop bounding box
  // runtime information - not saved
  ON_BoundingBox m_pbox;
private:
  friend class ON_Brep;
  ON_Brep* m_brep;
  ON_BrepLoop(const ON_BrepLoop&); // no implementation
};

class ON_CLASS ON_BrepFace : public ON_SurfaceProxy
{
  ON_OBJECT_DECLARE(ON_BrepFace);

public:
  void DestroyRuntimeCache( bool bDelete = true );

  // Union available for application use.
  // The constructor zeros m_face_user.
  // The value is of m_face_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_face_user;

  int m_face_index;  // index of face in ON_Brep.m_F[] array

  ON_BrepFace();
  ~ON_BrepFace();
  ON_BrepFace(int);
  ON_BrepFace& operator=(const ON_BrepFace&);

  /*
  Returns:
   Brep that the face belongs to.
  */
  ON_Brep* Brep() const;

  /*
  Parameters:
    fli - [in] index into the face's m_li[] array.
  Returns:
    The loop brep.m_L[face.m_li[fli]];
  */
  ON_BrepLoop* Loop( int fli ) const;

  /*
  Returns:
    Number of loops in this face.
  */
  int LoopCount() const;

  /*
  Returns:
    Outer boundary loop for this face.
  */
  ON_BrepLoop* OuterLoop() const;

  /*
  Parameters:
    dir
       1: side with underlying surface normal
         pointing into the topology region
      -1: side with underlying surface normal
          pointing out of the topology region
  Returns:
    Brep region topology face side.  If the region
    topology has not be created by calling
    ON_Brep::RegionToplogy(), then NULL is returned.
  */
  class ON_BrepFaceSide* FaceSide(int dir) const;


  /////////////////////////////////////////////////////////////////
  // ON_Object overrides
  //
  // (Faces are purely topologicial - geometry queries should be 
  //  directed at the face's 3d surface.)

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const; // for debugging

  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Geometry::ComponentIndex() override
  ON_COMPONENT_INDEX ComponentIndex() const;

  // virtual ON_Geometry::ClearBoundingBox() override
  void ClearBoundingBox();

  // virtual ON_Geometry::GetBBox() override
  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // minimum
         double*,    // maximum
         ON_BOOL32 = false  // true means grow box
         ) const;

  /*
  Description:
    This is an override of the virtual ON_Surface::Reverse
    function.  It toggles the face's m_bRev flag so the abstract
    orientation of the face does not change.
  Parameters:
    dir - [in] 0 = reverse "s" parameter, 1 = reverse "t" parameter
         The domain changes from [a,b] to [-a,-b]
  Returns:
    True if successful.
  Remarks:
    The range of the face's trimming curves and the orientation direction
    of then loops are changed so that the resulting face is still valid.
  */
  ON_BOOL32 Reverse(
    int dir
    );

  /*
  Description:
    This is an override of the virtual ON_Surface::Transpose
    function.  It toggles the face's m_bRev flag so the abstract
    orientation of the face does not change.
  Returns:
    True if successful.
  Remarks:
    The range of the face's trimming curves and the orientation direction
    of then loops are changed so that the resulting face is still valid.
  */
  ON_BOOL32 Transpose();

  /*
  Description:
    This is an override of the virtual ON_Surface::SetDomain
    function.
  Parameters:
    dir - [in] 0 = set "u" domain, 1 = set "v" domain.
    t0 - [in]
    t1 - [in] t0 < t1  The new domain is the interval (t0,t1)
  Returns:
    True if successful.
  */
  ON_BOOL32 SetDomain(
    int dir,
    double t0,
    double t1
    );

  /*
  //////////
  // Change the domain of a face
  // This changes the parameterization of the face's surface and transforms
  // the "u" and "v" coordinates of all the face's parameter space trimming
  // curves.  The locus of the face is not changed.
  */
  bool SetDomain(
         ON_Interval udom,
         ON_Interval vdom
         );

  /////////////////////////////////////////////////////////////////
  // Rendering Interface
  //int MaterialIndex() const; // if -1, use parent's material definition
  //void SetMaterialIndex(int);

  // If true is returne, then ~ON_BrepFace will delete mesh.
  bool SetMesh( ON::mesh_type, ON_Mesh* mesh );

  const ON_Mesh* Mesh( ON::mesh_type mesh_type ) const;

  /*
  Description:
    Destroy meshes used to render and analyze surface and polysrf
    objects.
  Parameters:
    mesh_type - [in] type of mesh to destroy
    bDeleteMesh - [in] if true, cached mesh is deleted.
      If false, pointer to cached mesh is just set to NULL.
  See Also:
    CRhinoObject::GetMeshes
    CRhinoObject::MeshCount
    CRhinoObject::IsMeshable
  */
  void DestroyMesh( ON::mesh_type mesh_type, bool bDeleteMesh = true );

  /////////////////////////////////////////////////////////////////
  // "Expert" Interface

  /*
  Description:
    Expert user tool that tranforms all the parameter space (2d)
    trimming curves on this face.  Only 2d curve geometry is
    changed.  The caller is responsible for reversing loops,
    toggle m_bRev, flags, etc.
  Parameters:
    xform - [in] Transformation applied to 2d curve geometry.
  Returns
    True if successful.  If false is returned, the brep
    may be invalid.
  */
  bool TransformTrim( const ON_Xform& xform );

  /*
  Description:
    Expert user tool that replaces the 3d surface geometry
    use by the face.
  Parameters;
    si - [in] brep surface index of new surface
    bTransformTrimCurves - [in]
      If unsure, then pass true.
      If the surface's domain has changed and you are certain
      its parameterization still jibes with the trim curve
      locations, then pass false.
  Returns:
    True if successful.
  Example:

            ON_Surface* pSurface = ...;
            int si = brep.AddSurface(pSurface);
            face.ChangeSurface(si);

  Remarks:
    If the face had a surface and new surface has a different
    shape, then you probably want to call something like
    ON_Brep::RebuildEdges() to move the 3d edge curves so they
    will lie on the new surface. This doesn't delete the old 
    surface; call ON_Brep::CullUnusedSurfaces() or ON_Brep::Compact
    to remove unused surfaces.
  See Also:
    ON_Brep::RebuildEdges
    ON_Brep::CullUnusedSurfaces
  */
  bool ChangeSurface(
    int si
    );
  bool ChangeSurface(
    int si,
    bool bTransformTrimCurves
    );

  /*
  Returns:
    brep.m_S[] surface index of the 3d surface geometry used by 
    this face or -1.
  */
  int SurfaceIndexOf() const;

  /*
  Returns:
    Pointer to the surface geometry used by the face.   
  */
  const ON_Surface* SurfaceOf() const;

  ON_SimpleArray<int> m_li; // loop indices (outer loop is m_li[0])
  int m_si;            // index of surface in b-rep m_S[] array
  bool m_bRev;         // true if face orientation is opposite
                       //      of natural surface orientation

  // m_face_material_channel provides a way to have individual
  // brep faces use a rendering material that is different
  // from the rendering material used by the parent brep.
  // If m_face_material_channel is zero 
  // channel and m_face_material_channel.m_j is the back face
  // materal. The default is (0,0) which indicates the face
  // should use the parent brep's material.
  // If "mat" is the brep's rendering material and
  // 0 < m_material_channel.m_i < mat.m_material_channel.Count(),
  // then this face should use the material with id
  // mat.m_material_channel[face.m_material_channel.m_i-1].m_id.
  // If m_material_channel.m_i or the id is invalid in any way,
  // then the default should be used.
  int m_face_material_channel;

  // Persistent id for this face.  Default is ON_nil_uuid.
  ON_UUID m_face_uuid;
private:
  ON_BoundingBox m_bbox;      // 3d bounding box
  ON_Interval    m_domain[2]; // rectangular bounds of 2d curves
  ON_Mesh* m_render_mesh;
  ON_Mesh* m_analysis_mesh;
  ON_Mesh* m_preview_mesh;
  //int m_material_index; // if 0 (default), ON_Brep's object attributes
  //                      // determine material.
private:
  friend class ON_Brep;
  ON_Brep* m_brep;
  ON_BrepFace( const ON_BrepFace& );
};

class ON_CLASS ON_BrepFaceSide : public ON_Object
{
  ON_OBJECT_DECLARE(ON_BrepFaceSide);
public:
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // Union available for application use.
  // The constructor zeros m_faceside_user.
  // The value is of m_faceside_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_faceside_user;

  // index of face side in ON_BrepRegionTopology.m_FS[] array
  int m_faceside_index;  

  ON_BrepFaceSide();
  ~ON_BrepFaceSide();
  ON_BrepFaceSide& operator=(const ON_BrepFaceSide&);

  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);


  /*
  Returns:
   Brep this face side belongs to.
  */
  ON_Brep* Brep() const;

  /*
  Returns:
    Region topology this face side belongs to.
  */
  class ON_BrepRegionTopology* RegionTopology() const;

  /*
  Returns:
   Region the face side belongs to.
  */
  class ON_BrepRegion* Region() const;

  /*
  Returns:
   Face this side belongs to.
  */
  class ON_BrepFace* Face() const;

  /*
  Returns:
   +1: underlying geometric surface normal points
       into region.
   -1: underlying geometric surface normal points
       out of region.
  */
  int SurfaceNormalDirection() const;

public:
  int m_ri; // region index 
            // m_ri = -1 indicates this faceside overlaps
            // another faceside. Generally this is a flaw
            // in an ON_Brep.
  int m_fi; // face index
  int m_srf_dir; //  1 ON_BrepFace's surface normal points into region
                 // -1 ON_BrepFace's surface normal points out of region

private:
  friend class ON_Brep;
  friend class ON_BrepRegionTopology;
  ON_BrepRegionTopology* m_rtop;
  ON_BrepFaceSide( const ON_BrepFaceSide& );
};

class ON_CLASS ON_BrepRegion : public ON_Object
{
  ON_OBJECT_DECLARE(ON_BrepRegion);
public:
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // Union available for application use.
  // The constructor zeros m_region_user.
  // The value is of m_region_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_region_user;

  // index of region in ON_BrepRegionTopology.m_R[] array
  int m_region_index;

  ON_BrepRegion();
  ~ON_BrepRegion();
  ON_BrepRegion& operator=(const ON_BrepRegion&);

  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);

  /*
  Returns:
   Brep this region belongs to.
  */
  ON_Brep* Brep() const;

  /*
  Returns:
    Region topology this region belongs to.
  */
  class ON_BrepRegionTopology* RegionTopology() const;

  /*
  Parameter:
    rfsi - [in] index into the region's m_fsi[] array.
  Returns:
    The face side in rtop.m_FS[m_fsi[rsi]], where
    rtop is the ON_BrepRegionTopology class this
    region belongs to.
  */
  ON_BrepFaceSide* FaceSide(int rfsi) const;

  /*
  Returns:
    True if the region is finite.
  */
  bool IsFinite() const;

  /*
  Returns:
   Region bounding box.
  */
  const ON_BoundingBox& BoundingBox() const;

  ON_SimpleArray<int> m_fsi; // indices of face sides
  int m_type; // 0 = infinte, 1 = bounded
  ON_BoundingBox m_bbox;

  /*
  Description:
    Get the boundary of a region as a brep object.  
    If the region is finite, the boundary will be a closed
    manifold brep.  The boundary may have more than one
    connected component.
  Parameters:
    brep - [in] if not NULL, the brep form is put into
                this brep.
  Returns: the region boundary as a brep or NULL if the
           calculation fails.
  */
  ON_Brep* RegionBoundaryBrep( ON_Brep* brep = NULL ) const;

private:
  friend class ON_Brep;
  friend class ON_BrepRegionTopology;
  ON_BrepRegionTopology* m_rtop;
  ON_BrepRegion( const ON_BrepRegion& );
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_BrepVertex>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepVertex>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_BrepEdge>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepEdge>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_BrepTrim>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepTrim>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_BrepLoop>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepLoop>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_BrepFace>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepFace>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepFaceSide>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_BrepRegion>;
#pragma warning( pop )
#endif

class ON_CLASS ON_BrepVertexArray : public ON_ObjectArray<ON_BrepVertex>
{
public:
  ON_BrepVertexArray();
  ~ON_BrepVertexArray();

  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepEdgeArray   : public  ON_ObjectArray<ON_BrepEdge>
{
public:
  ON_BrepEdgeArray();
  ~ON_BrepEdgeArray();
  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepTrimArray   : public  ON_ObjectArray<ON_BrepTrim>
{
public:
  ON_BrepTrimArray();
  ~ON_BrepTrimArray();
  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepLoopArray   : public  ON_ObjectArray<ON_BrepLoop>
{
public:
  ON_BrepLoopArray();
  ~ON_BrepLoopArray();
  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepFaceArray   : public  ON_ObjectArray<ON_BrepFace>
{
public:
  ON_BrepFaceArray();
  ~ON_BrepFaceArray();
  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepFaceSideArray : public ON_ObjectArray<ON_BrepFaceSide>
{
public:
  ON_BrepFaceSideArray();
  ~ON_BrepFaceSideArray();

  bool Read( ON_BinaryArchive& );
  bool Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepRegionArray : public ON_ObjectArray<ON_BrepRegion>
{
public:
  ON_BrepRegionArray();
  ~ON_BrepRegionArray();

  bool Read( ON_BinaryArchive& );
  bool Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;
};

class ON_CLASS ON_BrepRegionTopology
{
public:
  ON_BrepRegionTopology();
  ON_BrepRegionTopology(const ON_BrepRegionTopology& src);
  ~ON_BrepRegionTopology();
  ON_BrepRegionTopology& operator=(const ON_BrepRegionTopology&);

  ON_BrepFaceSideArray m_FS;
  ON_BrepRegionArray m_R;

  ON_Brep* Brep() const;
  bool IsValid( ON_TextLog* text_log = 0 ) const;
  bool Read( ON_BinaryArchive& );
  bool Write( ON_BinaryArchive& ) const;

  unsigned int SizeOf() const;

private:
  friend class ON_BrepRegionTopologyUserData;
  friend class ON_Brep;
  ON_Brep* m_brep;
};

class ON_CLASS ON_Brep : public ON_Geometry 
{
  ON_OBJECT_DECLARE(ON_Brep);

public:
  // virtual ON_Object::DestroyRuntimeCache override
  void DestroyRuntimeCache( bool bDelete = true );

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  // virtual ON_Geometry override
  bool EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const;

public:


  /*
  Description:
    Use ON_Brep::New() instead of new ON_Brep() when writing
    Rhino plug-ins (or when openNURBS is used as a Microsoft 
    DLL and you need to create a new ON_Brep in a different 
    .EXE or .DLL).
  Example:

              // bad - ON_Brep* pBrep = new ON_Brep();
              ON_Brep* pBrep = ON_Brep::New(); // good
              ...
              delete pBrep;
              pBrep = NULL;

  Returns:
    Pointer to an ON_Brep.  Destroy by calling delete.
  Remarks:
    When openNURBS is used as a Microsoft DLL, the CL.EXE
    compiler uses local vtables for classes that are new-ed
    in other executables but uses the ordinary vtable for
    for classes that are allocated in functions like
    ON_BrepCylinder(), ON_NurbsSurfaceQuadrilateral(),
    ON_Cylinder::RevSurfaceForm(NULL), etc.
    Using static New() functions like ON_Brep::New() insures
    that identical classes has the same vtable and makes
    all code run identically.
  */
  static ON_Brep* New();

  /*
  Description:
    Use ON_Brep::New(const ON_Brep& src) instead 
    of new ON_Brep(const ON_Brep& src).
  Returns:
    Pointer to an ON_Brep.  Destroy by calling delete.
  Remarks:
    See static ON_Brep* ON_Brep::New() for details.
  */
  static ON_Brep* New(const ON_Brep&);

	// Construction
  ON_Brep();
	~ON_Brep();		
  ON_Brep(const ON_Brep&);
  ON_Brep& operator=(const ON_Brep&);

  // Override of virtual ON_Object::MemoryRelocate
  void MemoryRelocate();


  /*
  Description:
    See if this and other are same brep geometry.
  Parameters:
    other - [in] other brep
    tolerance - [in] tolerance to use when comparing
                     control points.
  Returns:
    true if breps are the same
  */
  bool IsDuplicate( 
          const ON_Brep& other, 
          double tolerance = ON_ZERO_TOLERANCE 
          ) const;

  /////////////////////////////////////////////////////////////////
  // construction/destruction helpers

  // returns Brep to state it has after default construction
  void Destroy(); 

  // call if memory pool used by b-rep members becomes invalid
  void EmergencyDestroy(); 

  /*
  Description:
    Calculates polygon mesh approximation of the brep
    and appends one mesh for each face to the mesh_list[]
    array.
  Parameters:
    mp - [in] meshing parameters
    mesh_list - [out] meshes are appended to this array.
  Returns:
    Number of meshes appended to mesh_list[] array.
  */
  int CreateMesh( 
    const ON_MeshParameters& mp,
    ON_SimpleArray<ON_Mesh*>& mesh_list
    ) const;

  /*
  Description:
    Destroy meshes used to render and analyze brep.
  Parameters:
    mesh_type - [in] type of mesh to destroy
    bDeleteMesh - [in] if true, cached meshes are deleted.
      If false, pointers to cached meshes are just set to NULL.
  See Also:
    ON_Brep::GetMesh
    ON_BrepFace::DestroyMesh
    ON_BrepFace::Mesh
    ON_BrepFace::SetMesh
  */
  void DestroyMesh( ON::mesh_type mesh_type, bool bDeleteMesh = true );

  /*
  Description:
    Get cached meshes used to render and analyze brep.
  Parameters:
    mesh_type - [in] type of mesh to get
    meshes - [out] meshes are appended to this array.  The ON_Brep
      owns these meshes so they cannot be modified.
  Returns:
    Number of meshes added to array. (Same as m_F.Count())
  See Also:
    ON_Brep::DestroyMesh
    ON_BrepFace::DestroyMesh
    ON_BrepFace::Mesh
    ON_BrepFace::SetMesh
  */
  int GetMesh( ON::mesh_type mesh_type, ON_SimpleArray< const ON_Mesh* >& meshes ) const;

  /*
  Description:
    Create a brep from a surface.  The resulting surface has an outer
    boundary made from four trims.  The trims are ordered so that
    they run along the south, east, north, and then west side of the
    surface's parameter space.
  Parameters:
    pSurface - [in] pointer to a surface.  The brep will manage this
       pointer and delete it in ~ON_Brep.
  Returns:
    @untitled table
    true     successful
      When true is returned, the pSurface pointer is added to the
      brep's m_S[] array and it will be deleted by the brep's
      destructor.
    false
      brep cannot be created from this surface.
      When false is returned, then the caller is responsible
      for deleting pSurface unless it was previously added
      to the brep's m_S[] array.     
  Remarks:
    The surface class must be created with new so that the
    delete in ~ON_Brep will not cause a crash.
  */
  bool Create( 
          ON_Surface*& pSurface
          );

  bool Create( 
          ON_NurbsSurface*& pNurbsSurface
          );

  bool Create( 
          ON_PlaneSurface*& pPlaneSurface
          );

  bool Create( 
          ON_RevSurface*& pRevSurface
          );

  bool Create( 
          ON_SumSurface*& pSumSurface
          );

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  See Also:
    ON_Brep::SetTolerancesAndFlags
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  /*
  Description:
    Tests the brep to see if its topology information is
    valid.
  Parameters:
    text_log - [in] if the brep topology is not valid and 
        text_log is not NULL, then a brief english 
        description of the problem is appended to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     brep topology is valid
    false    brep topology is not valid
  Remarks:
    ON_Brep::IsValidTopology can be called at any time.
  See Also:
    ON_Brep::IsValid
    ON_Brep::IsValidGeometry
    ON_Brep::IsValidTolerancesAndFlags
  */
  bool IsValidTopology( ON_TextLog* text_log = NULL ) const;


  /*
  Description:
    Expert user function that tests the brep to see if its
    geometry information is valid.  The value of 
    brep.IsValidTopology() must be true before 
    brep.IsValidGeometry() can be safely called.
  Parameters:
    text_log - [in] if the brep geometry is not valid and 
        text_log is not NULL, then a brief english 
        description of the problem is appended to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     brep geometry is valid
    false    brep geometry is not valid
  Remarks:
    ON_Brep::IsValidTopology must be true before you can
    safely call ON_Brep::IsValidGeometry.
  See Also:
    ON_Brep::IsValid
    ON_Brep::IsValidTopology
    ON_Brep::IsValidTolerancesAndFlags
  */
  bool IsValidGeometry( ON_TextLog* text_log = NULL ) const;

  /*
  Description:
    Expert user function that tests the brep to see if its
    tolerances and flags are valid.  The values of 
    brep.IsValidTopology() and brep.IsValidGeometry() must
    be true before brep.IsValidTolerancesAndFlags() can 
    be safely called.
  Parameters:
    text_log - [in] if the brep tolerance or flags are not
        valid and text_log is not NULL, then a brief english 
        description of the problem is appended to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     brep tolerance and flags are valid
    false    brep tolerance and flags are not valid
  Remarks:
    ON_Brep::IsValidTopology and ON_Brep::IsValidGeometry
    must be true before you can safely call
    ON_Brep::IsValidTolerancesAndFlags.
  See Also:
    ON_Brep::IsValid
    ON_Brep::IsValidTopology
    ON_Brep::IsValidGeometry
  */
  bool IsValidTolerancesAndFlags( ON_TextLog* text_log = NULL ) const;

  // Description:
  //   Tests brep to see if it is valid for 
  //   saving in V2 3DM archives.
  // Returns:
  //   true if brep is valid for V2 3DM archives.
  // Remarks:
  //   V2 breps could not have dangling curves.
  bool IsValidForV2() const;
  bool IsValidForV2( const ON_BrepTrim& ) const;
  bool IsValidForV2( const ON_BrepEdge& ) const;

  // virtual ON_Objet::Dump() override
  void Dump( ON_TextLog& ) const; // for debugging

  // virtual ON_Objet::Write() override
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  // virtual ON_Objet::Read() override
  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Objet::ObjectType() override
  ON::object_type ObjectType() const;

  // virtual ON_Geometry::Dimension() override
  int Dimension() const;

  // virtual ON_Geometry::ClearBoundingBox() override
  void ClearBoundingBox();

  // virtual ON_Geometry::GetBBox() override
  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // minimum
         double*,    // maximum
         ON_BOOL32 = false  // true means grow box
         ) const;

  // virtual ON_Geometry::Transform() override
  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  // virtual ON_Geometry::SwapCoordinates() override
  ON_BOOL32 SwapCoordinates(
        int, int        // indices of coords to swap
        );

  // virtual ON_Geometry::HasBrepForm() override
  ON_BOOL32 HasBrepForm() const; // returns true

  /*
  Description:
    If possible, BrepForm() creates a brep form of the
    ON_Geometry. 
  Parameters:
    brep - [in] if not NULL, brep is used to store the brep
        form of the geometry.
  Result:
    If brep is not NULL, then brep = this, otherwise
    a duplicate of this is returned.
  Remarks:
    Override of virtual ON_Geometry::BrepForm
  */
  ON_Brep* BrepForm( ON_Brep* brep = NULL ) const;

  /////////////////////////////////////////////////////////////////
  // Creation Interface

  // These add a new geometry piece to the b-rep and return the
  // index that should be used to reference the geometry.
  // -1 is returned if the input is not acceptable.
  // ~ON_Brep() will delete the geometry.
  int AddTrimCurve( ON_Curve* ); // 2d curve used by ON_BrepTrim
  int AddEdgeCurve( ON_Curve* ); // 3d curve used by ON_BrepEdge
  int AddSurface( ON_Surface* ); // 3d surface used by ON_BrepFace

  // Description:
  //   Set 3d curve geometry used by a b-rep edge.
  // Parameters:
  //   edge - [in]
  //   c3_index - [in] index of 3d curve in m_C3[] array
  //   sub_domain - [in] if not NULL, sub_domain is an increasing
  //      sub interval of m_C3[c3_index]->Domain().
  // Returns:
  //   true if successful.
  bool SetEdgeCurve( 
    ON_BrepEdge& edge,
    int c3_index,
    const ON_Interval* sub_domain = NULL
    );

  // Description:
  //   Set 2d curve geometry used by a b-rep trim.
  // Parameters:
  //   trim - [in]
  //   c2_index - [in] index of 2d curve in m_C2[] array
  //   sub_domain - [in] if not NULL, sub_domain is an increasing
  //      sub interval of m_C2[c2_index]->Domain().
  // Returns:
  //   true if successful.
  bool SetTrimCurve( 
    ON_BrepTrim& trim,
    int c2_index,
    const ON_Interval* sub_domain = NULL
    );

  // These add a new topology piece to the b-rep and return a 
  // reference that is intended to be used for initialization.
  ON_BrepVertex& NewVertex();
  ON_BrepVertex& NewVertex( 
    ON_3dPoint vertex_point,
    double vertex_tolerance = ON_UNSET_VALUE
    );

  ON_BrepEdge& NewEdge(
                  int = -1              // 3d curve index
                  );
  ON_BrepEdge& NewEdge( 
                  ON_BrepVertex&, // start vertex
                  ON_BrepVertex&, // end vertex
                  int = -1,       // 3d curve index
                  const ON_Interval* = NULL, // sub_domain
                  double edge_tolerance = ON_UNSET_VALUE
                  );

  /*
  Description:
    Add a new face to a brep.  An incomplete face is added.
    The caller must create and fill in the loops used by
    the face.
  Parameters:
    si - [in] index of surface in brep's m_S[] array
  Returns:
    Reference to new face.
  Remarks:
    Adding a new face may grow the dynamic m_F array.  When
    this happens pointers and references to memory in the
    previous m_F[] array may become invalid.  Use face indices
    if this is an issue.
  Example:
    See ON_BrepBox and ON_BrepSphere source code.
  See Also:
    ON_Brep::AddSurface
  */
  ON_BrepFace& NewFace(
                  int si = -1
                  );

  /*
  Description:
    Add a new face to a brep.  This creates a complete face with
    new vertices at the surface corners, new edges along the surface
    boundary, etc.  The loop of the returned face has four trims that
    correspond to the south, east, north, and west side of the 
    surface in that order.  If you use this version of NewFace to
    add an exiting brep, then you are responsible for using a tool
    like ON_Brep::JoinEdges() to hook the new face to its
    neighbors.
  Parameters:
    surface - [in] surface is copied.
  Returns:
    Pointer to new face.
  Remarks:
    Adding a new face may grow the dynamic arrays used to store
    vertices, edges, faces, loops, and trims.  When these dyamic
    arrays are grown, any pointers and references to memory in
    the previous arrays may become invalid.  Use indices
    if this is an issue.
  See Also:
    ON_Brep::JoinEdges
    ON_Brep::AddSurface
  */
  ON_BrepFace* NewFace( 
    const ON_Surface& surface 
    );

  /*
  Description:
    Add a new face to brep.  This version is for expert users.
  Parameters:
    pSurface - [in] the returned face will have an outer loop
                    that goes around the edges of the surface.
    vid - [in/out] four vertex indices that specify the vertices at
                   the (sw,se,nw,ne) corners.  If the input value
                   of a vertex index is -1, then the vertex will be 
                   created.
    eid - [in/out] four edge indices that specify the edges for
                   the (south,east,north,west) sides.  If the input value
                   of an edge index is -1, then the edge will be created.
    bRev3d - [in/out] four values of the trim m_bRev3d flags of
                   the (south,east,north,west) sides.
  Returns:
    Pointer to the new face or NULL if input is not valid.
    If null is returned, then the caller must delete pSurace
    unless it was previously added to the brep's m_S[] array.
  Remarks:
    Adding a new face may grow the dynamic m_F array.  When
    this happens pointers and references to memory in the
    previous m_F[] array may become invalid.  Use face indices
    if this is an issue.
  Example:
    See ON_BrepBox and ON_BrepSphere source code.
  See Also:
    ON_Brep::AddSurface
    ON_Brep::AddFace( int si )
    ON_Brep::Create( ON_Surface*& )
  */
  ON_BrepFace* NewFace(
       ON_Surface* pSurface,
       int vid[4],
       int eid[4],
       ON_BOOL32 bRev3d[4]
       );

  /*
  Description:
    Add a new face to the brep whose surface geometry is a 
    ruled surface between two edges.
  Parameters:
    edgeA - [in] The south side of the face's surface will
          run along edgeA.
    bRevEdgeA - [in] true if the new face's outer boundary
          orientation along edgeA is opposite the orientation
          of edgeA.
    edgeB - [in] The north side of the face's surface will
          run along edgeA.
    bRevEdgeB - [in] true if the new face's outer boundary
          orientation along edgeB is opposite the orientation
          of edgeB.
  Returns:
    A pointer to the new face or a NULL if the new face could
    not be created.
  */
  ON_BrepFace* NewRuledFace(
        const ON_BrepEdge& edgeA,
        bool bRevEdgeA,
        const ON_BrepEdge& edgeB, 
        bool bRevEdgeB
        );

  /*
  Description:
    Add a new face to the brep whose surface geometry is a 
    ruled cone with the edge as the base and the vertex as
    the apex point.
  Parameters:
    vertex - [in] The apex of the cone will be at this vertex.
                   The north side of the surface's parameter
                   space will be a singular point at the vertex.
    edge - [in] The south side of the face's surface will
          run along this edge.
    bRevEdge - [in] true if the new face's outer boundary
          orientation along the edge is opposite the 
          orientation of edge.
  Returns:
    A pointer to the new face or a NULL if the new face could
    not be created.
  */
  ON_BrepFace* NewConeFace(
        const ON_BrepVertex& vertex,
        const ON_BrepEdge& edge,
        bool bRevEdge
        );

  /*
  Description:
    Create a new outer boundary loop that runs along the edges
    of the underlying surface.
  Returns:
    New outer boundary loop.
  */
  ON_BrepLoop& NewLoop( ON_BrepLoop::TYPE );

  /*
  Description:
    Create a new boundary loop on a face.  After you get this
    ON_BrepLoop, you still need to create the vertices, edges, 
    and trims that define the loop.
  Returns:
    New loop that needs to be filled in.
  */
  ON_BrepLoop& NewLoop( ON_BrepLoop::TYPE loop_type, ON_BrepFace& face );

  /*
  Description:
    Create a new outer boundary loop that runs along the sides
    of the face's surface.  All the necessary trims, edges,
    and vertices are created and added to the brep.
  Parameters:
    face_index - [in] index of face that needs an outer boundary
                      that runs along the sides of its surface.
  Returns:
    New outer boundary loop that is complete.
  */
  ON_BrepLoop* NewOuterLoop( int face_index );

  /*
  Description:
    Add a new face to brep.  This version is for expert users.
  Parameters:
    face_index - [in] index of face that will get a new outer
                   loop running around the sides of the face's
                   underlying surface.
    vid - [in/out] four vertex indices that specify the vertices at
                   the (sw,se,nw,ne) corners.  If the input value
                   of a vertex index is -1, then the vertex will be 
                   created.
    eid - [in/out] four edge indices that specify the edges for
                   the (south,east,north,west) sides.  If the input value
                   of an edge index is -1, then the edge will be created.
    bRev3d - [in/out] four values of the trim m_bRev3d flags of
                   the (south,east,north,west) sides.
  Returns:
    Pointer to the new loop or NULL if input is not valid.
  Remarks:
    Adding a new loop may grow the dynamic m_L array.  When
    this happens pointers and references to memory in the
    previous m_L[] array may become invalid.  Use face indices
    if this is an issue.
  See Also:
    ON_Brep::NewFace
  */
  ON_BrepLoop* NewOuterLoop(
         int face_index,
         int vid[4],
         int eid[4],
         ON_BOOL32 bRev3d[4]
         );

  /*
  Description:
    Add a planar trimming loop to a planar face.
  Parameters:
    face_index - [in] index of planar face.  The underlying
        suface must be an ON_PlaneSurface.
    loop_type - [in] type of loop to add.  If loop_type is
        ON_BrepLoop::unknown, then the loop direction is tested
        and the the new loops type will be set to 
        ON_BrepLoop::outer or ON_BrepLoop::inner.  If the loop_type
        is ON_BrepLoop::outer, then the direction of the new loop
        is tested and flipped if it is clockwise. If the loop_type
        is ON_BrepLoop::inner, then the direction of the new loop
        is tested and flipped if it is counter-clockwise.
    boundary - [in] a list of 3d curves that form a simple (no self
        intersections) closed curve.  These curves define the 3d
        edge geometry and should be near the planar surface.
    bDuplicateCurves - [in] If true, then duplicates of the curves 
        in the boundary array are added to the brep.  If false, the
        curves in the boundary array are added to the brep and will
        be deleted by ON_Brep::~ON_Brep.
    Returns:
      true if successful.  The new loop will be brep.m_L.Last().
  */
  bool NewPlanarFaceLoop(
        int face_index,
        ON_BrepLoop::TYPE loop_type,
        ON_SimpleArray<ON_Curve*>& boundary,
        ON_BOOL32 bDuplicateCurves = true
        );


  /*
  Description:
    Add a new trim that will be part of an inner, outer, or slit loop
    to the brep.
  Parameters:
    c2i - [in] index of 2d trimming curve
  Returns:
    new trim
  Example:
    int c2i = brep->AddTrimCurve( p2dCurve );
    ON_BrepTrim& trim = NewTrim( edge, bRev3d, loop, c2i );
    trim.m_ei = ...;
    trim.m_li = ...;
    trim.m_tolerance[0] = ...;
    trim.m_tolerance[1] = ...;
    trim.m_type = ...;
    trim.m_iso = ...;
  Remarks:
    You should set the trim's ON_BrepTrim::m_tolerance, ON_BrepTrim::m_type,
    ON_BrepTrim::m_iso, ON_BrepTrim::m_li, and ON_BrepTrim::m_ei values.
        In general, you should try to use the
    ON_BrepTrim::NewTrim( edge, bRev3d, loop, c2i ) version of NewTrim.
    If you want to add a singular trim, use ON_Brep::NewSingularTrim.
    If you want to add a crvonsrf trim, use ON_Brep::NewCurveOnFace.
    If you want to add a ptonsrf trim, use ON_Brep::NewPointOnFace.
  See Also:
    ON_Brep::SetTrimTypeFlags
    ON_Brep::SetTrimIsoFlags
    ON_Brep::NewSingularTrim
    ON_Brep::NewPointOnFace
    ON_Brep::NewCurveOnFace
  */
  ON_BrepTrim& NewTrim(
                  int c2i = -1
                  );

  /*
  Description:
    Add a new trim that will be part of an inner, outer, or slit loop
    to the brep.
  Parameters:
    bRev3d - [in] ON_BrepTrim::m_bRev3d value.  true if the
       edge and trim have opposite directions.
    loop - [in] trim is appended to this loop
    c2i - [in] index of 2d trimming curve
  Returns:
    new trim
  Example:
    int c2i = brep->AddTrimCurve( p2dCurve );
    ON_BrepTrim& trim = NewTrim( edge, bRev3d, loop, c2i );
    trim.m_ei = ...;
    trim.m_tolerance[0] = ...;
    trim.m_tolerance[1] = ...;
    trim.m_type = ...;
    trim.m_iso = ...;
  Remarks:
    You should set the trim's ON_BrepTrim::m_tolerance, ON_BrepTrim::m_type,
    ON_BrepTrim::m_iso, and ON_BrepTrim::m_ei values.
        In general, you should try to use the
    ON_BrepTrim::NewTrim( edge, bRev3d, loop, c2i ) version of NewTrim.
    If you want to add a singular trim, use ON_Brep::NewSingularTrim.
    If you want to add a crvonsrf trim, use ON_Brep::NewCurveOnFace.
    If you want to add a ptonsrf trim, use ON_Brep::NewPointOnFace.
  See Also:
    ON_Brep::SetTrimTypeFlags
    ON_Brep::SetTrimIsoFlags
    ON_Brep::NewSingularTrim
    ON_Brep::NewPointOnFace
    ON_Brep::NewCurveOnFace
  */
  ON_BrepTrim& NewTrim(
                  ON_BOOL32 bRev3d,
                  ON_BrepLoop& loop,
                  int c2i = -1
                  );

  /*
  Description:
    Add a new trim that will be part of an inner, outer, or slit loop
    to the brep.
  Parameters:
    edge - [in] 3d edge associated with this trim
    bRev3d - [in] ON_BrepTrim::m_bRev3d value.  true if the
       edge and trim have opposite directions.
    c2i - [in] index of 2d trimming curve
  Returns:
    new trim
  Example:
    int c2i = brep->AddTrimCurve( p2dCurve );
    ON_BrepTrim& trim = NewTrim( edge, bRev3d, c2i );
    trim.m_li = ...;
    trim.m_tolerance[0] = ...;
    trim.m_tolerance[1] = ...;
    trim.m_type = ...;
    trim.m_iso = ...;
  Remarks:
    You should set the trim's ON_BrepTrim::m_tolerance, 
    ON_BrepTrim::m_type, ON_BrepTrim::m_iso, 
    and ON_BrepTrim::m_li values.
        In general, you should try to use the
    ON_BrepTrim::NewTrim( edge, bRev3d, loop, c2i ) version of NewTrim.
    If you want to add a singular trim, use ON_Brep::NewSingularTrim.
    If you want to add a crvonsrf trim, use ON_Brep::NewCurveOnFace.
    If you want to add a ptonsrf trim, use ON_Brep::NewPointOnFace.
  See Also:
    ON_Brep::SetTrimTypeFlags
    ON_Brep::SetTrimIsoFlags
    ON_Brep::NewSingularTrim
    ON_Brep::NewPointOnFace
    ON_Brep::NewCurveOnFace
  */
  ON_BrepTrim& NewTrim(
                  ON_BrepEdge& edge,
                  ON_BOOL32 bRev3d,
                  int c2i = -1
                  );

  /*
  Description:
    Add a new trim that will be part of an inner, outer, or slit loop
    to the brep.
  Parameters:
    edge - [in] 3d edge associated with this trim
    bRev3d - [in] ON_BrepTrim::m_bRev3d value.  true if the
       edge and trim have opposite directions.
    loop - [in] trim is appended to this loop
    c2i - [in] index of 2d trimming curve
  Returns:
    new trim
  Example:
    int c2i = brep->AddTrimCurve( p2dCurve );
    ON_BrepTrim& trim = brep->NewTrim( edge, bRev3d, loop, c2i );
    trim.m_tolerance[0] = ...;
    trim.m_tolerance[1] = ...;
  Remarks:
    You should set the trim's ON_BrepTrim::m_tolerance values.
    If c2i is -1, you must set the trim's ON_BrepTrim::m_iso values.
    This version of NewTrim sets the trim.m_type value.  If the
    input edge or loop are not currently valid, then you may
    need to adjust the trim.m_type value.
    If you want to add a singular trim, use ON_Brep::NewSingularTrim.
    If you want to add a crvonsrf trim, use ON_Brep::NewCurveOnFace.
    If you want to add a ptonsrf trim, use ON_Brep::NewPointOnFace.
  See Also:
    ON_Brep::SetTrimTypeFlags
    ON_Brep::SetTrimIsoFlags
    ON_Brep::NewSingularTrim
    ON_Brep::NewPointOnFace
    ON_Brep::NewCurveOnFace
  */
  ON_BrepTrim& NewTrim(
                  ON_BrepEdge& edge,
                  ON_BOOL32 bRev3d,
                  ON_BrepLoop& loop,
                  int c2i = -1
                  );

  /*
  Description:
    Add a new singular trim to the brep.
  Parameters:
    vertex - [in] vertex along collapsed surface edge
    loop - [in] trim is appended to this loop
    iso - [in] one of ON_Surface::S_iso, ON_Surface::E_iso, 
               ON_Surface::N_iso, or ON_Surface::W_iso.
    c2i - [in] index of 2d trimming curve
  Returns:
    new trim
  See Also:
    ON_Brep::NewTrim
  */
  ON_BrepTrim& NewSingularTrim(
                  const ON_BrepVertex& vertex,
                  ON_BrepLoop& loop,
                  ON_Surface::ISO iso,
                  int c2i = -1
                  );

  /*
  Description:
    Adds a new point on face to the brep.
  Parameters:
    face - [in] face that vertex lies on
    s,t - [in] surface parameters
  Returns:
    new vertex that represents the point on face.
  Remarks:
    If a vertex is a point on a face, then brep.m_E[m_ei]
    will be an edge with no 3d curve.  This edge will have
    a single trim with type ON_BrepTrim::ptonsrf.  There
    will be a loop containing this single trim.
  */
  ON_BrepVertex& NewPointOnFace( 
    ON_BrepFace& face,
    double s,
    double t
    );

  /*
  Description:
    Add a new curve on face to the brep.
  Parameters:
    face - [in] face that curve lies on
    edge - [in] 3d edge associated with this curve on surface
    bRev3d - [in] true if the 3d edge and the 2d parameter space 
                  curve have opposite directions.
    c2i - [in] index of 2d curve in face's parameter space
  Returns:
    new trim that represents the curve on surface
  Remarks:
    You should set the trim's ON_BrepTrim::m_tolerance and
    ON_BrepTrim::m_iso values.
  */
  ON_BrepTrim& NewCurveOnFace(
                  ON_BrepFace& face,
                  ON_BrepEdge& edge,
                  ON_BOOL32 bRev3d = false,
                  int c2i = -1
                  );

  // appends a copy of brep to this and updates
  // indices of appended brep parts.  Duplicates are not removed.
  void Append( 
    const ON_Brep& // brep
    ); 

  // This function can be used to compute vertex information for a
  // b-rep when everything but the m_V array is properly filled in.
  // It is intended to be used when creating a ON_Brep from a 
  // definition that does not include explicit vertex information.
  void SetVertices(void);

  // This function can be used to set the ON_BrepTrim::m_iso
  // flag. It is intended to be used when creating a ON_Brep from
  // a definition that does not include compatible parameter space
  // type information.
  // See Also: ON_BrepSetFlagsAndTolerances
  bool SetTrimIsoFlags();    // sets all trim iso flags
  bool SetTrimIsoFlags( ON_BrepFace& );
  bool SetTrimIsoFlags( ON_BrepLoop& );
  bool SetTrimIsoFlags( ON_BrepTrim& );


  /*
  Description:
    Calculate the type (singular, mated, boundary, etc.) of
    an ON_BrepTrim object.
  Parameters:
    trim - [in]
    bLazy - [in] if true and trim.m_type is set to something other
       than ON_BrepTrim::unknown, then no calculation is
       performed and the value of trim.m_type is returned.
       If false, the value of trim.m_type is ignored and is caluculated.
  Returns:
    Type of trim.
  Remarks:
    The trim must be connected to a valid loop.
  See Also:
    ON_Brep::SetTrimTypeFlags
  */
  ON_BrepTrim::TYPE TrimType( 
    const ON_BrepTrim& trim, 
    ON_BOOL32 bLazy = true
    ) const;

  // This function can be used to set the ON_BrepTrim::m_type
  // flag.  If the optional bLazy argument is true, then only
  // trims with m_type = unknown are set.
  // See Also: ON_BrepSetFlagsAndTolerances
  bool SetTrimTypeFlags( ON_BOOL32 bLazy = false );    // sets all trim iso flags
  bool SetTrimTypeFlags( ON_BrepFace&, ON_BOOL32 bLazy = false );
  bool SetTrimTypeFlags( ON_BrepLoop&, ON_BOOL32 bLazy = false );
  bool SetTrimTypeFlags( ON_BrepTrim&, ON_BOOL32 bLazy = false );

  // GetTrim2dStart() evaluates the start of the
  // parameter space (2d) trim curve.
  bool GetTrim2dStart( 
          int trim_index,         // index of ON_BrepTrim in m_T[] array
          ON_2dPoint& 
          ) const;

  // GetTrim2dEnd() evaluates end of the
  // parameter space (2d) trim curve.
  bool GetTrim2dEnd(
          int,         // index of ON_BrepTrim in m_T[] array
          ON_2dPoint& 
          ) const;

  // GetTrim3dStart() evaluates the 3d surface at the start of the
  // parameter space (2d) trim curve.
  bool GetTrim3dStart( 
          int,         // index of ON_BrepTrim in m_T[] array
          ON_3dPoint& 
          ) const;

  // GetTrim3dEnd() evaluates the 3d surface at the end of the
  // parameter space (2d) trim curve.
  bool GetTrim3dEnd(
          int,         // index of ON_BrepTrim in m_T[] array
          ON_3dPoint& 
          ) const;

  // This function examines the 2d parameter space curves and returns
  // the loop's type based on their orientation.  Use this function for
  // debugging loop orientation problems.
  ON_BrepLoop::TYPE ComputeLoopType( const ON_BrepLoop& ) const;

  // These set the various tolerances.  The optional ON_BOOL32 argument
  // is called bLazy.  If bLazy is false, the tolerance is recomputed
  // from its definition.  If bLazy is true, the tolerance is computed
  // only if its current value is negative.
  bool SetVertexTolerance( ON_BrepVertex& vertex, ON_BOOL32 bLazy = false ) const;
  virtual
  bool SetTrimTolerance( ON_BrepTrim& trim, ON_BOOL32 bLazy = false ) const;
  virtual
  bool SetEdgeTolerance( ON_BrepEdge& edge, ON_BOOL32 bLazy = false ) const;

  /*
  Description:
    Set the brep's vertex tolerances.
  Parameters:
    bLazy - [in] if true, only vertex tolerances with the value
       ON_UNSET_VALUE will be set.  If false, the vertex tolerance
       is recomputed from the geometry in the brep.
  Returns:
    true if successful.
  See Also:
    ON_Brep::SetVertexTolerance
    ON_Brep::SetTrimTolerance
    ON_Brep::SetEdgeTolerance
    ON_Brep::SetVertexTolerances
    ON_Brep::SetTrimTolerances
    ON_Brep::SetEdgeTolerances
    ON_Brep::SetTolerancesAndFlags
  */
  bool SetVertexTolerances( ON_BOOL32 bLazy = false );

  /*
  Description:
    Set the brep's trim tolerances.
  Parameters:
    bLazy - [in] if true, only trim tolerances with the value
       ON_UNSET_VALUE will be set.  If false, the trim tolerance
       is recomputed from the geometry in the brep.
  Returns:
    true if successful.
  See Also:
    ON_Brep::SetVertexTolerance
    ON_Brep::SetTrimTolerance
    ON_Brep::SetEdgeTolerance
    ON_Brep::SetVertexTolerances
    ON_Brep::SetTrimTolerances
    ON_Brep::SetEdgeTolerances
    ON_Brep::SetTolerancesAndFlags
  */
  bool SetTrimTolerances( ON_BOOL32 bLazy = false );

  /*
  Description:
    Set the brep's edge tolerances.
  Parameters:
    bLazy - [in] if true, only edge tolerances with the value
       ON_UNSET_VALUE will be set.  If false, the edge tolerance
       is recomputed from the geometry in the brep.
  Returns:
    true if successful.
  See Also:
    ON_Brep::SetVertexTolerance
    ON_Brep::SetTrimTolerance
    ON_Brep::SetEdgeTolerance
    ON_Brep::SetVertexTolerances
    ON_Brep::SetTrimTolerances
    ON_Brep::SetEdgeTolerances
    ON_Brep::SetTolerancesAndFlags
  */
  bool SetEdgeTolerances( ON_BOOL32 bLazy = false );


  /*
  Description:
    Set the trim parameter space bounding box (trim.m_pbox).
  Parameters:
    trim - [in]
    bLazy - [in] if true and trim.m_pbox is valid, then
       the box is not set.
  Returns:
    true if trim ends up with a valid bounding box.
  */
  virtual
  bool SetTrimBoundingBox( ON_BrepTrim& trim, ON_BOOL32 bLazy=false );

  /*
  Description:
    Set the loop parameter space bounding box (loop.m_pbox).
  Parameters:
    loop - [in]
    bLazy - [in] if true and loop trim trim.m_pbox is valid, 
       then that trim.m_pbox is not recalculated.
  Returns:
    true if loop ends up with a valid bounding box.
  */
  virtual
  bool SetTrimBoundingBoxes( ON_BrepLoop& loop, ON_BOOL32 bLazy=false );


  /*
  Description:
    Set the loop and trim parameter space bounding boxes
    for every loop and trim in the face 
  Parameters:
    face - [in]
    bLazy - [in] if true and trim trim.m_pbox is valid, 
       then that trim.m_pbox is not recalculated.
  Returns:
    true if all the face's loop and trim parameter space bounding 
    boxes are valid.
  */
  virtual
  bool SetTrimBoundingBoxes( ON_BrepFace& face, ON_BOOL32 bLazy=false );

  /*
  Description:
    Set the loop and trim parameter space bounding boxes
    for every loop and trim in the brep.
  Parameters:
    bLazy - [in] if true and trim trim.m_pbox is valid, 
       then that trim.m_pbox is not recalculated.
  Returns:
    true if all the loop and trim parameter space bounding boxes
    are valid.
  */
  virtual
  bool SetTrimBoundingBoxes( ON_BOOL32 bLazy=false );

  /*
  Description:
    Set tolerances and flags in a brep
  Parameters:
    bLazy - [in] if true, only flags and tolerances that are not
       set will be calculated.
    bSetVertexTolerances - [in] true to compute vertex.m_tolerance values
    bSetEdgeTolerances - [in] true to compute edge.m_tolerance values
    bSetTrimTolerances - [in] true to compute trim.m_tolerance[0,1] values
    bSetTrimIsoFlags - [in] true to compute trim.m_iso values
    bSetTrimTypeFlags - [in] true to compute trim.m_type values
    bSetLoopTypeFlags - [in] true to compute loop.m_type values
    bSetTrimBoxes - [in] true to compute trim.m_pbox values
  See Also:
    ON_Brep::SetVertexTolerance
    ON_Brep::SetEdgeTolerance
    ON_Brep::SetTrimTolerance
    ON_Brep::SetTrimTypeFlags
    ON_Brep::SetTrimIsoFlags
    ON_Brep::ComputeLoopType
    ON_Brep::SetTrimBoundingBox
    ON_Brep::SetTrimBoundingBoxes
  */
  void SetTolerancesBoxesAndFlags(
       ON_BOOL32 bLazy = false,
       ON_BOOL32 bSetVertexTolerances = true,
       ON_BOOL32 bSetEdgeTolerances = true,
       ON_BOOL32 bSetTrimTolerances = true,
       ON_BOOL32 bSetTrimIsoFlags = true,
       ON_BOOL32 bSetTrimTypeFlags = true,
       ON_BOOL32 bSetLoopTypeFlags = true,
       ON_BOOL32 bSetTrimBoxes = true
       );


  /////////////////////////////////////////////////////////////////
  // Query Interface

  /*
  Description:
    Determine how many brep faces reference m_S[surface_index].
  Parameters:
    surface_index - [in] index of the surface in m_S[] array
    max_count - [in] counting stops if max_count > 0 and
                     at least max_count faces use the surface.
  Returns:
    Number of brep faces that reference the surface.
  */
  int SurfaceUseCount( 
              int surface_index,
              int max_count=0 ) 
              const;
  /*
  Description:
    Determine how many brep edges reference m_C3[c3_index].
  Parameters:
    c3_index - [in] index of the 3d curve in m_C3[] array
    max_count - [in] counting stops if max_count > 0 and
                     at least max_count edges use the 3d curve.
  Returns:
    Number of brep edges that reference the 3d curve.
  */
  int EdgeCurveUseCount( 
              int c3_index,
              int max_count=0 ) 
              const;

  /*
  Description:
    Determine how many brep trims reference m_C2[c2_index].
  Parameters:
    c2_index - [in] index of the 2d curve in m_C2[] array
    max_count - [in] counting stops if max_count > 0 and
                     at least max_count trims use the 2d curve.
  Returns:
    Number of brep trims that reference the 2d curve.
  */
  int TrimCurveUseCount( 
              int c2_index,
              int max_count=0 ) 
              const;

  /*
  Description:
    Get a single 3d curve that traces the entire loop
  Parameters:
    loop - [in] loop whose 3d curve should be duplicated
    bRevCurveIfFaceRevIsTrue - [in] If false, the returned
       3d curve has an orientation compatible with the
       2d curve returned by Loop2dCurve().
       If true and the m_bRev flag of the loop's face
       is true, then the returned curve is reversed.
  Returns:
    A pointer to a 3d ON_Curve.  The caller must delete
    this curve.
  */
  ON_Curve* Loop3dCurve( 
    const ON_BrepLoop& loop,
    ON_BOOL32 bRevCurveIfFaceRevIsTrue = false
    ) const;

  /*
  Description:
    Get a list of 3d curves that trace the non-seam edge
    portions of an entire loop
  Parameters:
    loop - [in] loop whose 3d curve should be duplicated
    curve_list - [out] 3d curves are appended to this list
    bRevCurveIfFaceRevIsTrue - [in] If false, the returned
       3d curves have an orientation compatible with the
       2d curve returned by Loop2dCurve().
       If true and the m_bRev flag of the loop's face
       is true, then the returned curves are reversed.
  Returns:
    Number of curves appended to curve_list.
  */
  int Loop3dCurve( 
    const ON_BrepLoop& loop,
    ON_SimpleArray<ON_Curve*>& curve_list,
    ON_BOOL32 bRevCurveIfFaceRevIsTrue = false
    ) const;


  /*
  Description:
    Get a 3d curve that traces the entire loop
  Parameters:
    loop - [in] loop whose 2d curve should be duplicated
  Returns:
    A pointer to a 2d ON_Curve.  The caller must delete
    this curve.
  */
  ON_Curve* Loop2dCurve( const ON_BrepLoop& loop ) const;

  /*
  Description:
    Determine orientation of a brep.
  Returns:
    @untitle table
    +2     brep is a solid but orientation cannot be computed
    +1     brep is a solid with outward facing normals
    -1     brep is a solid with inward facing normals
     0     brep is not a solid
  Remarks:
    The base class implementation returns 2 or 0.  This
    function is overridden in the Rhino SDK and returns
    +1, -1, or 0.
  See Also:
    ON_Brep::IsSolid
  */
  virtual
  int SolidOrientation() const;

  /*
  Description:
    Test brep to see if it is a solid.  (A "solid" is
    a closed oriented manifold.)
  Returns:
    @untitled table
    true       brep is a solid
    fals       brep is not a solid
  See Also:
    ON_Brep::SolidOrientation
    ON_Brep::IsManifold
  */
  bool IsSolid() const;
  
  /*
  Description:
    Test brep to see if it is an oriented manifold.
  Parameters:
    pbIsOriented - [in]  if not null, *pbIsOriented is set
        to true if b-rep is an oriented manifold and false
        if brep is not an oriented manifold.
    pbHasBoundary - [in]  if not null, *pbHasBoundary is set
        to true if b-rep has a boundary edge and false if
        brep does not have a boundary edge.
  Returns:
    true       brep is a manifold
    fals       brep is not a manifold
  See Also:
    ON_Brep::IsSolid
  */
  bool IsManifold( // returns true if b-rep is an oriented manifold
    ON_BOOL32* pbIsOriented = NULL,
    ON_BOOL32* pbHasBoundary = NULL
    ) const;

  /*
  Description: 
    Determine if P is inside Brep.  This question only makes sense
    when the brep is a closed manifold.  This function does not
    not check for closed or manifold, so result is not valid in
    those cases.  Intersects a line through P with brep, finds
    the intersection point Q closest to P, and looks at face 
    normal at Q.  If the point Q is on an edge or the intersection
    is not transverse at Q, then another line is used.
  Parameters:
    P - [in] 3d point
    tolerance - [in] 3d distance tolerance used for intersection
      and determining strict inclusion.
    bStrictlInside - [in] If bStrictlInside is true, then this
      function will return false if the distance from P is within
      tolerance of a brep face.
  Returns:
    True if P is in, false if not. See parameter bStrictlyIn.
  */
  bool IsPointInside(
          ON_3dPoint P, 
          double tolerance,
          bool bStrictlyInside
          ) const;


  bool IsSurface() const;      // returns true if the b-rep has a single face
                               // and that face is geometrically the same
                               // as the underlying surface.  I.e., the face
                               // has trivial trimming.  In this case, the
                               // surface is m_S[0].
                               // The flag m_F[0].m_bRev records
                               // the correspondence between the surface's
                               // natural parametric orientation and the
                               // orientation of the b-rep.


  bool FaceIsSurface(          // returns true if the face has a single
         int // index of face  // outer boundary and that boundary runs
         ) const;              // along the edges of the underlying surface.
                               // In this case the geometry of the surface
                               // is the same as the geometry of the face.
                               // If FaceIsSurface() is true, then
                               // m_S[m_F[face_index].m_si] is the surface.
                               // The flag m_F[face_index].m_bRev records
                               // the correspondence between the surface's
                               // natural parametric orientation and the
                               // orientation of face in the b-rep.

  bool LoopIsSurfaceBoundary(  // returns true if the loop's trims all run
         int // index of loop  // along the edge's of the underlying surface's
         ) const;              // parameter space.

  /////////////////////////////////////////////////////////////////
  // Modification Interface

  //////////
  // Clears all ON_BrepFace.m_bRev flags by ON_BrepFace::Transpose
  // on each face with a true m_bRev.
  bool FlipReversedSurfaces();

  //////////
  // Change the domain of a trim's 2d curve.  This changes only the
  // parameterization of the 2d trimming curve; the locus of the 
  // 2d trimming curve is not changed.
  bool SetTrimDomain(
         int, // index of trim in m_T[] array
         const ON_Interval&
         );

  //////////
  // Change the domain of an edge.  This changes only the
  // parameterization of the 3d edge curve; the locus of the 
  // 3d edge curve is not changed.
  bool SetEdgeDomain(
         int, // index of edge in m_E[] array
         const ON_Interval&
         );

  // Reverses entire brep orientation of all faces by toggling 
  // value of all face's ON_BrepFace::m_bRev flag.
  void Flip();

  // reverses orientation of a face by toggling ON_BrepFace::m_bRev
  void FlipFace(ON_BrepFace&);

  // Reverses orientation of trimming loop. 
  // This function is intended to be used by brep experts and does
  // does NOT modify ON_BrepLoop::m_type.  You should make sure 
  // ON_BrepLoop::m_type jibes with the loop's direction.  (Outer loops
  // should be counter-clockwise and inner loops should be clockwise.)
  // You can use ON_Brep::LoopDirection() to determine the direction of
  // a loop.
  void FlipLoop(ON_BrepLoop&); // reverses orientation of trimming loop

  // LoopDirection() examines the 2d trimming curve geometry that defines
  // the loop and returns
  //
  //   @untitled table
  //   +1    the loop is a counter-clockwise loop.
  //   -1    the loop is a clockwise loop.
  //    0    the loop is not a continuous closed loop.
  //
  // Since LoopDirection() calculates its result based on the 2d trimming
  // curve geometry, it can be use to set ON_BrepLoop::m_type to outer/inner
  // when translating from data definition where this distinction is murky.
  int LoopDirection( const ON_BrepLoop& ) const;


  /*
  Description:
    Sort the face.m_li[] array by loop type 
    (outer, inner, slit, crvonsrf, ptonsrf)
  Parameters:
    face - [in/out] face whose m_li[] array should be sorted.
  Returns:
    @untitled table
    true      success
    false     failure - no loops or loops with unset loop.m_type
  See Also:
    ON_Brep::ComputeLoopType
    ON_Brep::LoopDirection
  */
  bool SortFaceLoops( ON_BrepFace& face ) const;

  /*
  Description:
    Expert user function.
  See Also:
    ON_Brep::JoinEdges
  */
  bool CombineCoincidentVertices(ON_BrepVertex&, ON_BrepVertex&); // moves information to first vertex and deletes second

  /*
  Description:
    Expert user function.
  See Also:
    ON_Brep::JoinEdges
  */
  bool CombineCoincidentEdges(ON_BrepEdge&, ON_BrepEdge&); // moves information to first edge and deletes second

  /*
  Description:
    Expert user function.
    Combines contiguous edges into a single edge.  The edges
    must share a common vertex, then angle between the edge
    tangents are the common vertex must be less than or
    equal to angle_tolerance_radians, and any associated
    trims must be contiguous in there respective boundaries.
  Parameters;
    edge_index0 - [in]
    edge_index1 - [in]
    angle_tolerance_radians - [in]
  Returns:
    Pointer to the new edge or NULL if the edges cannot
    be combined into a single edge.
  Remarks:
    The input edges are deleted but are still in the
    brep's m_E[] arrays.  Use ON_Brep::Compact to remove 
    the unused edges.
  */
  ON_BrepEdge* CombineContiguousEdges( 
    int edge_index0, 
    int edge_iindex1, 
    double angle_tolerance_radians = ON_PI/180.0
    );

  // These remove a topology piece from a b-rep but do not
  // rearrange the arrays that hold the brep objects.  The
  // deleted objects have their indices set to -1.  Deleting
  // an object that is connected to other objects will 
  // modify thos objects.
  void DeleteVertex(ON_BrepVertex& vertex);
  void DeleteEdge(ON_BrepEdge& edge, ON_BOOL32 bDeleteEdgeVertices); // pass true to delete vertices used only by edge
  void DeleteTrim(ON_BrepTrim& trim, ON_BOOL32 bDeleteTrimEdges); // pass true to delete edges and vertices used only by trim
  void DeleteLoop(ON_BrepLoop& loop, ON_BOOL32 bDeleteLoopEdges); // pass true to delete edges and vertices used only by trim
  void DeleteFace(ON_BrepFace& face, ON_BOOL32 bDeleteFaceEdges); // pass true to delete edges and vertices used only by face
  void DeleteSurface(int s_index);
  void Delete2dCurve(int c2_index);
  void Delete3dCurve(int c3_index);

  // Description:
  //   Set m_vertex_user.i, m_edge_user.i, m_face_user.i, m_loop_user.i,
  //   and m_trim_user.i values of faces of component including 
  //   m_F[face_index] to label. Numbering starts at 1.
  // Parameters:
  //   face_index - [in] index of face in component
  //   label - [in] value for m_*_user.i
  // Returns:
  // Remarks:
  //   Chases through trim lists of face edges to find adjacent faces.
  //   Does NOT check for vertex-vertex connections
  void LabelConnectedComponent(
    int face_index,
    int label
    );

  /*
  Description:
    Set  m_vertex_user.i, m_edge_user.i, m_face_user.i, m_loop_user.i,
    and m_trim_user.i values values to distinguish connected components.
  Parameters:
  Returns:
    number of connected components
  Remarks:
    For each face in the ith component, sets m_face_user.i to i>0.
    Chases through trim lists of face edges to find adjacent faces.
	  Numbering starts at 1. Does NOT check for vertex-vertex connections.
  See Also:
    ON_Brep::GetConnectedComponents
  */
  int LabelConnectedComponents();

  /*
  Description:
    If this brep has two or more connected components, 
    then duplicates of the connected components are appended
    to the components[] array.
  Parameters:
    components - [in] connected components are appended to this array.
    bDuplicateMeshes - [in] if true, any meshes on this brep are copied
         to the output breps.
  Returns:
    Number of connected components appended to components[] or zero
    if this brep has only one connected component.
  See Also:
    ON_Brep::GetConnectedComponents
  */
  int GetConnectedComponents( 
          ON_SimpleArray< ON_Brep* >& components,
          bool bDuplicateMeshes
          ) const;

  /*
  Description:
    Copy a subset of this brep.
  Parameters:
    subfi_count - [in] length of sub_fi[] array.
    sub_fi - [in] array of face indices in this
      brep to copy. (If any values inf sub_fi[]
      are out of range or if sub_fi[] contains
      duplicates, this function will return null.)
    sub_brep - [in] if this pointer is not null,
      then the subbrep will be created in this
      class.
  Returns:
    If the input is valid, a pointer to the
    subbrep is returned.  If the input is not
    valid, null is returned.  The faces in
    in the subbrep's m_F array are in the same
    order as they were specified in sub_fi[].
  */
  ON_Brep* SubBrep( 
    int subfi_count, 
    const int* sub_fi, 
    ON_Brep* sub_brep = 0 
    ) const;

  ///////////////////////////////////////////////////////////////////////
  //
  // region topology
  //
  bool HasRegionTopology() const;

  /*
  Description:
    Get region topology information:
    In order to keep the ON_Brep class efficient, rarely used
    region topology information is not maintained.  If you 
    require this information, call RegionTopology().
  */
  const ON_BrepRegionTopology& RegionTopology() const;

  /*
  Description:
    Get region topology information:
    In order to keep the ON_Brep class efficient, rarely used
    region topology information is not maintained.  If you 
    require this information, call RegionTopology().
  */
  void DestroyRegionTopology();
  // Description:
  //   Duplicate a single brep face.
  // Parameters:
  //   face_index - [in] index of face to duplicate
  //   bDuplicateMeshes - [in] if true, any attached meshes are duplicated
  // Returns:
  //   Single face brep.
  // Remarks:
  //   The m_vertex_user.i, m_edge_user.i, m_face_user.i, m_loop_user.i,
  //   and m_trim_user.i values of the returned brep are are set to the 
  //   indices of the objects they duplicate.
  // See Also:
  //   ON_Brep::DeleteFace, ON_Brep::ExtractFace
  ON_Brep* DuplicateFace(
    int face_index,
    ON_BOOL32 bDuplicateMeshes
    ) const;

  // Description:
  //   Duplicate a a subset of a brep
  // Parameters:
  //   face_count - [in] length of face_index[] array
  //   face_index - [in] array of face indices
  //   bDuplicateMeshes - [in] if true, any attached meshes are duplicated
  // Returns:
  //   A brep made by duplicating the faces listed in the face_index[] array.
  // Remarks:
  //   The m_vertex_user.i, m_edge_user.i, m_face_user.i, m_loop_user.i,
  //   and m_trim_user.i values of the returned brep are are set to the 
  //   indices of the objects they duplicate.
  // See Also:
  //   ON_Brep::DuplicateFace
  ON_Brep* DuplicateFaces(
    int face_count,
    const int* face_index,
    ON_BOOL32 bDuplicateMeshes
    ) const;

  // Description:
  //   Extract a face from a brep.
  // Parameters:
  //   face_index - [in] index of face to extract
  // Returns:
  //   Single face brep.
  // See Also:
  //   ON_Brep::DeleteFace, ON_Brep::DuplicateFace
  ON_Brep* ExtractFace(
    int face_index
    );


  /*
  Description:
    Standardizes the relationship between an ON_BrepEdge
    and the 3d curve it uses.  When done, the edge will
    be the only edge that references its 3d curve, the 
    domains of the edge and 3d curve will be the same, 
    and the edge will use the entire locus of the 3d curve.
  Parameters:
    edge_index - [in] index of edge to standardize.
    bAdjustEnds - [in] if true, move edge curve endpoints to vertices
  See Also:
    ON_Brep::StandardizeEdgeCurves
    ON_Brep::Standardize
  */
  bool StandardizeEdgeCurve( int edge_index, bool bAdjustEnds );


  /*
  Description:
    Expert user only.  Same as above, but to be used when the edge
    curve use count is known for the edge.
    Standardizes the relationship between an ON_BrepEdge
    and the 3d curve it uses.  When done, the edge will
    be the only edge that references its 3d curve, the 
    domains of the edge and 3d curve will be the same, 
    and the edge will use the entire locus of the 3d curve.
  Parameters:
    edge_index - [in] index of edge to standardize.
    bAdjustEnds - [in] if true, move edge curve endpoints to vertices
    EdgeCurveUse - [in] if > 1, then the edge curve for this edge is used by more than one
        edge.  if 1, then the edge curve is used only for this edge. 
        If <= 0, then use count is unknown.
  See Also:
    ON_Brep::StandardizeEdgeCurves
    ON_Brep::Standardize
  */
  bool StandardizeEdgeCurve( int edge_index, bool bAdjustEnds, int EdgeCurveUse );


  /*
  Description:
    Standardize all edges in the brep.
  Parameters:
    bAdjustEnds - [in] if true, move edge curve endpoints to vertices
  See Also:
    ON_Brep::StandardizeEdgeCurve
    ON_Brep::Standardize
  */
  void StandardizeEdgeCurves( bool bAdjustEnds );

  /*
  Description:
    Standardizes the relationship between an ON_BrepTrim
    and the 2d curve it uses.  When done, the trim will
    be the only trim that references its 2d curve, the 
    domains of the trim and 2d curve will be the same, 
    and the trim will use the entire locus of the 2d curve.
  Parameters:
    trim_index - [in] index of trim to standardize.
  See Also:
    ON_Brep::StandardizeTrimCurves
    ON_Brep::Standardize
  */
  bool StandardizeTrimCurve( int trim_index );

  /*
  Description:
    Standardize all trims in the brep.
  See Also:
    ON_Brep::StandardizeTrimCurve
    ON_Brep::Standardize
  */
  void StandardizeTrimCurves();

  /*
  Description:
    Standardizes the relationship between an ON_BrepFace
    and the 3d surface it uses.  When done, the face will
    be the only face that references its 3d surface, and
    the orientations of the face and 3d surface will be 
    the same. 
  Parameters:
    face_index - [in] index of face to standardize.
  See Also:
    ON_Brep::StardardizeFaceSurfaces
    ON_Brep::Standardize
  */
  bool StandardizeFaceSurface( int face_index );

  /*
  Description:
    Standardize all faces in the brep.
  See Also:
    ON_Brep::StandardizeFaceSurface
    ON_Brep::Standardize
  */
  void StandardizeFaceSurfaces();

  // misspelled function name is obsolete
  ON_DEPRECATED void StardardizeFaceSurfaces();

  /*
  Description:
    Standardize all trims, edges, and faces in the brep.
  Remarks:
    After standardizing, there may be unused curves and surfaces
    in the brep.  Call ON_Brep::Compact to remove these unused
    curves and surfaces.
  See Also:
    ON_Brep::StandardizeTrimCurves
    ON_Brep::StandardizeEdgeCurves
    ON_Brep::StandardizeFaceSurface
    ON_Brep::Compact
  */
  void Standardize();
  

  /*
  Description:
    Sometimes the ON_Surface used by a face extends far
    beyond the face's outer boundary.  ShrinkSurface uses
    ON_Surface::Trim to remove portions of the surface that
    extend beyond the face's outer boundary loop.
  Parameters:
    face - [in] face to test and whose surface should be shrunk.
    DisableSide - [in] This is a bit field.  A set bit indicates not to shrink
                the surface on a given side.  The default of 0 enables shrinking 
                on all four sides.
      @table  
      value       meaning
      0x0001     Dont shrink on the west side of domain.
      0x0002     Dont shrink on the south side of domain.
      0x0004     Dont shrink on the east side of domain.
      0x0008     Dont shrink on the north side of domain.
  Returns:
    @untitled table
    true        successful
    false       failure
  Remarks:
    If a surface needs to be shrunk it is copied.  After shrinking,
    you may want to call ON_Brep::CullUnusedSurfaces to remove
    any unused surfaces.
  See Also:
    ON_Brep::ShrinkSurfaces
    ON_Brep::CullUnusedSurfaces
  */
  bool ShrinkSurface( ON_BrepFace& face, int DisableSide=0 );

  /*
  Description:
    Sometimes the ON_Surface used by a face extends far
    beyond the face's outer boundary.  ShrinkSurfaces calls
    ON_Shrink::ShrinkSurface on each face to remove portions
    of surfaces that extend beyond their face's outer boundary
    loop.
  Returns:
    @untitled table
    true        successful
    false       failure
  Remarks:
    If a surface needs to be shrunk it is copied.  After shrinking,
    you may want to call ON_Brep::CullUnusedSurfaces to remove
    any unused surfaces.
  See Also:
    ON_Brep::ShrinkSurface
    ON_Brep::CullUnusedSurfaces
  */
  bool ShrinkSurfaces();

  /*
  Description:
    Uses the CullUnused*() members to delete any unreferenced
    objects from arrays, reindexes as needed, and shrinks
    arrays to minimum required size.
  See Also:
    ON_Brep::CullUnusedFaces
    ON_Brep::CullUnusedLoops
    ON_Brep::CullUnusedTrims
    ON_Brep::CullUnusedEdges
    ON_Brep::CullUnusedVertices
    ON_Brep::CullUnused3dCurves
    ON_Brep::CullUnused2dCurves
    ON_Brep::CullUnusedSurfaces
  */
  bool Compact();

  bool CullUnusedFaces(); // culls faces with m_face_index == -1
  bool CullUnusedLoops(); // culls loops with m_loop_index == -1
  bool CullUnusedTrims(); // culls trims with m_trim_index == -1
  bool CullUnusedEdges(); // culls edges with m_edge_index == -1
  bool CullUnusedVertices(); // culls vertices with m_vertex_index == -1
  bool CullUnused3dCurves(); // culls 2d curves not referenced by a trim
  bool CullUnused2dCurves(); // culls 3d curves not referenced by an edge
  bool CullUnusedSurfaces(); // culls surfaces not referenced by a face

  /////////////////////////////////////////////////////////////////
  // Navigation Interface

  // for moving around loops - returns trim index of prev/next trim in loop
  int PrevTrim(
        int // index of current trim (m_trim_index)
        ) const;
  int NextTrim(
        int // index of current trim (m_trim_index)
        ) const;

  /*
  Description:
    This is a simple tool for getting running through the edges
    that begin and end at a vertex.
  Parameters:
    current_edge_index - [in]
    endi - [in] 0 = use the edge start vertex, 1 = use the edge end vertex
    prev_endi - [out] 0 if previous edge begins at the vertex, 
                      1 if previous edge ends at the vertex
  Returns:
    edge index of the previous edge or -1 if there is only one edge
    that begins or ends at the vertex.
  Remarks:
    This is a tool that simplifies searching through the
    ON_BrepVertex.m_ei[] array.
    The edges are in no particular order.
  See Also:
    ON_Brep::NextEdge
  */
  int PrevEdge(
        int current_edge_index,
        int endi,
        int* prev_endi = NULL
        ) const;

  /*
  Description:
    This is a simple tool for getting running through the edges
    that begin and end at a vertex.
  Parameters:
    current_edge_index - [in]
    endi - [in] 0 = use the edge start vertex, 1 = use the edge end vertex
    next_endi - [out] 0 if next edge begins at the vertex, 
                      1 if next edge ends at the vertex
  Returns:
    edge index of the next edge or -1 if there is only one edge
    that begins or ends at the vertex.
  Remarks:
    This is a tool that simplifies searching through the
    ON_BrepVertex.m_ei[] array.  
    The edges are in no particular order.
  See Also:
    ON_Brep::NextEdge
  */
  int NextEdge(
        int current_edge_index,
        int endi,
        int* next_endi = NULL
        ) const;

  /*
  Description:
    Get a brep component from its index.
  Parameters:
    component_index - [in] 
  Returns:
    A const pointer to the component.  Do not delete
    the returned object.  It points to an object managed
    by this brep.
  See Also:
    ON_Brep::Face
    ON_Brep::Edge
    ON_Brep::Loop
    ON_Brep::Trim
    ON_Brep::Vertex
  */
  const ON_Geometry* BrepComponent( 
    ON_COMPONENT_INDEX ci
    ) const;

  /*
  Description:
    Get vertex from trim index or component index.
  Parameters:
    vertex_index - [in] either an index into m_V[] or a component index
                      of type brep_vertex.
  Returns:
    If the index is a valid vertex index or a valid vertex component
    index, then a pointer to the ON_BrepVertex is returned.  Otherwise
    NULL is returned.
  See Also
    ON_Brep::Component( const ON_BrepVertex& )
  */
  ON_BrepVertex* Vertex( int vertex_index ) const;
  ON_BrepVertex* Vertex( ON_COMPONENT_INDEX vertex_index ) const;

  /*
  Description:
    Get edge from edge index or component index.
  Parameters:
    edge_index - [in] either an index into m_E[] or a component index
                      of type brep_edge.
  Returns:
    If the index is a valid edge index or a valid edge component
    index, then a pointer to the ON_BrepEdge is returned.  Otherwise
    NULL is returned.
  See Also
    ON_Brep::Component( const ON_BrepEdge& )
  */
  ON_BrepEdge* Edge( int edge_index ) const;
  ON_BrepEdge* Edge( ON_COMPONENT_INDEX edge_index ) const;

  /*
  Description:
    Get trim from trim index or component index.
  Parameters:
    trim_index - [in] either an index into m_T[] or a component index
                      of type brep_trim.
  Returns:
    If the index is a valid trim index or a valid trim component
    index, then a pointer to the ON_BrepTrim is returned.  Otherwise
    NULL is returned.
  See Also
    ON_Brep::Component( const ON_BrepTrim& )
  */
  ON_BrepTrim* Trim( int trim_index ) const;
  ON_BrepTrim* Trim( ON_COMPONENT_INDEX trim_index ) const;

  /*
  Description:
    Get loop from loop index or component index.
  Parameters:
    loop_index - [in] either an index into m_L[] or a component index
                      of type brep_loop.
  Returns:
    If the index is a valid loop index or a valid loop component
    index, then a pointer to the ON_BrepLoop is returned.  Otherwise
    NULL is returned.
  See Also
    ON_Brep::Component( const ON_BrepLoop& )
  */
  ON_BrepLoop* Loop( int loop_index ) const;
  ON_BrepLoop* Loop( ON_COMPONENT_INDEX loop_index ) const;

  /*
  Description:
    Get face from face index or component index.
  Parameters:
    face_index - [in] either an index into m_F[] or a component index
                      of type brep_face.
  Returns:
    If the index is a valid face index or a valid face component
    index, then a pointer to the ON_BrepFace is returned.  Otherwise
    NULL is returned.
  See Also
    ON_Brep::Component( const ON_BrepFace& )
  */
  ON_BrepFace* Face( int face_index ) const;
  ON_BrepFace* Face( ON_COMPONENT_INDEX face_index ) const;

  /*
  Description:
    remove slit trims and slit boundaries from each face.
  Returns:
    true if any slits were removed
  Remarks:
    Caller should call Compact() afterwards.
  */
  bool RemoveSlits();

  /*
  Description:
    remove slit trims and slit boundaries from a face.
  Parameters:
    F - [in] brep face
  Returns:
    true if any slits were removed
  Remarks:
    Caller should call Compact() when done.
  */
  bool RemoveSlits(ON_BrepFace& F);

  /*
  Description:
    If fid0 != fid1 and m_F[fid0] and m_F[fid1] have the same surface (m_si is identical),
    and they are joined along a set of edges that do not have any other faces, then this will
    combine the two faces into one.
  Parameters:
    fid0, fid1 - [in] indices into m_F of faces to be merged.
  Returns:
    id of merged face if faces were successfully merged. -1 if not merged.
  Remarks:
    Caller should call Compact() when done.
  */
  int MergeFaces(int fid0, int fid1);

  /*
  Description:
    Merge all possible faces that have the same m_si
  Returns:
    true if any faces were successfully merged.
  Remarks:
    Caller should call Compact() when done.
  */
  bool MergeFaces();

  /*
  Description:
    Removes nested polycurves from the m_C2[] and m_C3[] arrays.
  Parameters:
    bExtractSingleSegments - [in] if true, polycurves with a
      single segment are replaced with the segment curve.
    bEdges - [in] if true, the m_C3[] array is processed
    bTrimCurves - [in] if true, the m_C2[] array is processed.  
  Returns:
    True if any nesting was removed and false if no nesting
    was removed.
  */
  bool RemoveNesting(
          bool bExtractSingleSegments,
          bool bEdges = true, 
          bool bTrimCurves = true
          );

  /*
  Description:
    Expert user tool to collapse a "short" edge to a vertex.
    The edge is removed and the topology is repaired
    so that everything that used to connect to the edge
    connects the specified vertex.
  Parameters:
    edge_index - [in] index of edge to remove
    bCloseTrimGap - [in] if true and the removal of the
       edge creates a gap in the parameter space trimming
       loop, then the 2d trim curves will be adjusted to
       close the gap.
    vertex_index - [in] if >= 0, this the edge is collapsed
       to this vertex.  Otherwise a vertex is automatically
       selected or created.
  Returns:
    True if edge was successfully collapsed.
  Remarks:
    After you finish cleaning up the brep, you need
    to call ON_Brep::Compact() to remove unused edge,
    trim, and vertex information from the brep's m_E[], 
    m_V[], m_T[], m_C2[], and m_C3[] arrays.
  */
  bool CollapseEdge(
    int edge_index,
    bool bCloseTrimGap = true,
    int vertex_index = -1
    );

  /*
  Description:
    Expert user tool to move trims and edges from
    one vertex to another.
  Parameters:
    old_vi - [in] index of old vertex
    new_vi - [in] index of new vertex
    bClearTolerances - [in] if true, then tolerances of
       edges and trims that are connected ot the old
       vertex are set to ON_UNSET_VALUE.
    vertex_index - [in] if >= 0, this the edge is collapsed
       to this vertex.  Otherwise a vertex is automatically
       selected or created.
  Returns:
    True if successful.
  Remarks:
    After you finish cleaning up the brep, you need
    to call ON_Brep::Compact() to remove unused edge,
    trim, and vertex information from the brep's m_E[], 
    m_V[], m_T[], m_C2[], and m_C3[] arrays.
  */
  bool ChangeVertex( 
    int old_vi, 
    int new_vi, 
    bool bClearTolerances 
    );

  /*
  Description:
    Expert user tool to remove any gap between adjacent trims.
  Parameters:
    trim0 - [in]
    trim1 - [in]
  Returns:
    True if successful.
  Remarks:
    The trims must be in the same trimming loop.  The vertex
    at the end of trim0 must be the same as the vertex at
    the start of trim1.  The trim's m_iso and m_type flags
    need to be correctly set.
  */
  bool CloseTrimGap( 
    ON_BrepTrim& trim0, 
    ON_BrepTrim& trim1 
    );

  /*
  Description:
    Remove edges that are not connected to a face.
  Parameters:
    bDeleteVertices - [in] if true, then the vertices
      at the ends of the wire edges are deleted if 
      they are not connected to face trimming edges.
  Returns:
    Number of edges that were removed.
  Remarks:
    After you finish cleaning up the brep, you need
    to call ON_Brep::Compact() to remove unused edge,
    trim, and vertex information from the brep's m_E[], 
    m_V[], m_T[], m_C2[], and m_C3[] arrays.

    If you want to remove wire edges and wiere
    After you finish cleaning up the brep, you need
    to call ON_Brep::Compact() to remove deleted vertices
    from the m_V[] array.
  See Also:
    ON_Brep::RemoveWireVertices
  */
  int RemoveWireEdges( bool bDeleteVertices = true );

  /*
  Description:
    Remove vertices that are not connected to an edge.
  Returns:
    Number of vertices that were deleted.
  Remarks:
    After you finish cleaning up the brep, you need
    to call ON_Brep::Compact() to remove deleted 
    vertices from the m_V[] array.
  See Also:
    ON_Brep::RemoveWireEdges
  */
  int RemoveWireVertices();

  /////////////////////////////////////////////////////////////////
  // "Expert" Interface

  void Set_user(ON_U u); // set every brep m_*_user value to u
  void Clear_vertex_user_i(); // zero all brep's m_vertex_user values
  void Clear_edge_user_i(int);   // zero all brep's m_edge_user values
  void Clear_edge_user_i();   // zero all brep's m_edge_user values
  void Clear_trim_user_i();   // zero all brep's m_trim_user values
  void Clear_loop_user_i();   // zero all brep's m_loop_user values
  void Clear_face_user_i();   // zero all brep's m_face_user values
  void Clear_user_i();        // zero all brep's m_*_user values

  // Union available for application use.
  // The constructor zeros m_brep_user.
  // The value is of m_brep_user is not saved in 3DM
  // archives and may be changed by some computations.
  ON_U m_brep_user; 

  // geometry 
  // (all geometry is deleted by ~ON_Brep().  Pointers can be NULL
  // or not referenced.  Use Compact() to remove unreferenced geometry.
  ON_CurveArray   m_C2;  // Pointers to parameter space trimming curves
                         // (used by trims).
  ON_CurveArray   m_C3;  // Pointers to 3d curves (used by edges).
  ON_SurfaceArray m_S;   // Pointers to parametric surfaces (used by faces)

  // topology
  // (all topology is deleted by ~ON_Brep().  Objects can be unreferenced.
  // Use Compact() to to remove unreferenced geometry.
  ON_BrepVertexArray  m_V;   // vertices
  ON_BrepEdgeArray    m_E;   // edges
  ON_BrepTrimArray    m_T;   // trims
  ON_BrepLoopArray    m_L;   // loops
  ON_BrepFaceArray    m_F;   // faces

protected:	
  friend class ON_BrepFace;
  friend class ON_BrepRegion;
  friend class ON_BrepFaceSide;
  ON_BoundingBox m_bbox;

  // Never directly set m_is_solid, use calls to IsSolid() and/or 
  // SolidOrientation() when you need to know the answer to this
  // question.
  // 0 = unset
  // 1 = solid with normals pointing out
  // 2 = solid with normals pointing in
  // 3 = not solid
  int m_is_solid;

  // These are friends so legacy tol values stored in v1 3dm files
  // can be used to set brep edge and trimming tolerances with a call
  // to ON_Brep::SetTolsFromLegacyValues().
  friend bool ON_BinaryArchive::ReadV1_TCODE_LEGACY_FAC(ON_Object**,ON_3dmObjectAttributes*);
  friend bool ON_BinaryArchive::ReadV1_TCODE_LEGACY_SHL(ON_Object**,ON_3dmObjectAttributes*);
  void Initialize();

  // helpers to set ON_BrepTrim::m_iso flag
  void SetTrimIsoFlag(int,double[6]);
  void SetTrimIsoFlag(int);

  // helpers to create and set vertices
  bool SetEdgeVertex(const int, const int, const int );
  bool HopAcrossEdge( int&, int& ) const;
  bool SetTrimStartVertex( const int, const int);
  void SetLoopVertices(const int);
  void ClearTrimVertices();
  void ClearEdgeVertices();

  // helpers for SwapFaceParameters()
  bool SwapLoopParameters(
        int // index of loop
        );
  bool SwapTrimParameters(
        int // index of trim
        );

  // helpers for validation checking
  bool IsValidTrim(int trim_index,ON_TextLog* text_log) const;
  bool IsValidTrimTopology(int trim_index,ON_TextLog* text_log) const;
  bool IsValidTrimGeometry(int trim_index,ON_TextLog* text_log) const;
  bool IsValidTrimTolerancesAndFlags(int trim_index,ON_TextLog* text_log) const;

  bool IsValidLoop(int loop_index,ON_TextLog* text_log) const;
  bool IsValidLoopTopology(int loop_index,ON_TextLog* text_log) const;
  bool IsValidLoopGeometry(int loop_index,ON_TextLog* text_log) const;
  bool IsValidLoopTolerancesAndFlags(int loop_index,ON_TextLog* text_log) const;

  bool IsValidFace(int face_index,ON_TextLog* text_log) const;
  bool IsValidFaceTopology(int face_index,ON_TextLog* text_log) const;
  bool IsValidFaceGeometry(int face_index,ON_TextLog* text_log) const;
  bool IsValidFaceTolerancesAndFlags(int face_index,ON_TextLog* text_log) const;
  
  bool IsValidEdge(int edge_index,ON_TextLog* text_log) const;
  bool IsValidEdgeTopology(int edge_index,ON_TextLog* text_log) const;
  bool IsValidEdgeGeometry(int edge_index,ON_TextLog* text_log) const;
  bool IsValidEdgeTolerancesAndFlags(int edge_index,ON_TextLog* text_log) const;

  bool IsValidVertex(int vertex_index,ON_TextLog* text_log) const;
  bool IsValidVertexTopology(int vertex_index,ON_TextLog* text_log) const;
  bool IsValidVertexGeometry(int vertex_index,ON_TextLog* text_log) const;
  bool IsValidVertexTolerancesAndFlags(int vertex_index,ON_TextLog* text_log) const;

  void SetTolsFromLegacyValues();

  // read helpers to support various versions
  bool ReadOld100( ON_BinaryArchive& ); // reads legacy old RhinoIO toolkit b-rep
  bool ReadOld101( ON_BinaryArchive& ); // reads legacy Rhino 1.1 b-rep
  bool ReadOld200( ON_BinaryArchive&, int ); // reads legacy trimmed surface
  ON_Curve* Read100_BrepCurve( ON_BinaryArchive& ) const;
  ON_Surface* Read100_BrepSurface( ON_BinaryArchive& ) const;

  // helpers for reading legacy v1 trimmed surfaces and breps
  bool ReadV1_LegacyTrimStuff( ON_BinaryArchive&, ON_BrepFace&, ON_BrepLoop& );
  bool ReadV1_LegacyTrim( ON_BinaryArchive&, ON_BrepFace&, ON_BrepLoop& );
  bool ReadV1_LegacyLoopStuff( ON_BinaryArchive&, ON_BrepFace& );
  bool ReadV1_LegacyLoop( ON_BinaryArchive&, ON_BrepFace& );
  bool ReadV1_LegacyFaceStuff( ON_BinaryArchive& );
  bool ReadV1_LegacyShellStuff( ON_BinaryArchive& );
};

///////////////////////////////////////////////////////////////////////////////
//
// brep construction tools
// 

/*
Description:
  Create a brep representation of a mesh.
Parameters:
  mesh_topology - [in]
  bTrimmedTriangles - [in] if true, triangles in the mesh
     will be represented by trimmed planes in the brep.
     If false, triangles in the mesh will be represented by
     untrimmed singular bilinear NURBS surfaces in the brep.
  pBrep - [in] If not NULL, this the mesh representation will
     be put into this brep.
Example:

          ON_Mesh mesh = ...;
          ON_Brep* pBrep = ON_BrepFromMesh( mesh.Topology() );
          ...
          delete pBrep;

See Also
  ON_BrepFromMesh( const ON_Mesh& mesh, ... );
*/
ON_DECL
ON_Brep* ON_BrepFromMesh( 
          const ON_MeshTopology& mesh_topology, 
          ON_BOOL32 bTrimmedTriangles = true,
          ON_Brep* pBrep = NULL 
          );

/*
Description:
  Get an ON_Brep definition of a box.
Parameters:
  box_corners - [in] 8 points defining the box corners
     arranged as the vN lables indicate.

          v7_______e6_____v6
           |\             |\
           | e7           | e5
           |  \ ______e4_____\ 
          e11  v4         |   v5
           |   |        e10   |
           |   |          |   |
          v3---|---e2----v2   e9
           \   e8         \   |
            e3 |           e1 |
             \ |            \ |
              \v0_____e0_____\v1

  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the box with topology

   edge              vertices
    m_E[ 0]           m_V[0], m_V[1]
    m_E[ 1]           m_V[1], m_V[2]
    m_E[ 2]           m_V[2], m_V[3]
    m_E[ 3]           m_V[3], m_V[0]
    m_E[ 4]           m_V[4], m_V[5]
    m_E[ 5]           m_V[5], m_V[6]
    m_E[ 6]           m_V[6], m_V[7]
    m_E[ 7]           m_V[7], m_V[4]
    m_E[ 8]           m_V[0], m_V[4]
    m_E[ 9]           m_V[1], m_V[5]
    m_E[10]           m_V[2], m_V[6]
    m_E[11]           m_V[3], m_V[7]

   face              boundary edges
    m_F[0]            +m_E[0] +m_E[9]  -m_E[4] -m_E[8]
    m_F[1]            +m_E[1] +m_E[10] -m_E[5] -m_E[9]
    m_F[2]            +m_E[2] +m_E[11] -m_E[6] -m_E[10]
    m_F[3]            +m_E[3] +m_E[8]  -m_E[7] -m_E[11]
    m_F[4]            -m_E[3] -m_E[2]  -m_E[1] -m_E[0]
//     m_F[5]            +m_E[4] +m_E[5]  +m_E[6] +m_E[7]
*/
ON_DECL
ON_Brep* ON_BrepBox( const ON_3dPoint* box_corners, ON_Brep* pBrep = NULL );

/*
Description:
  Get an ON_Brep definition of a wedge.
Parameters:
  corners - [in] 6 points defining the box corners
     arranged as the vN lables indicate.

                     /v5    
                    /|\       
                   / | \     
                  e5 |  e4   
                 /   e8  \     
                /__e3_____\  
              v3|    |    |v4     
                |    |    |       
                |    /v2  |   
                e6  / \   e7   
                |  /   \  |   
                | e2    e1|   
                |/       \|     
                /____e0___\  
              v0           v1

  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the wedge with topology

  edge              vertices
    m_E[ 0]           m_V[0], m_V[1]
    m_E[ 1]           m_V[1], m_V[2]
    m_E[ 2]           m_V[2], m_V[0]
    m_E[ 3]           m_V[3], m_V[4]
    m_E[ 4]           m_V[4], m_V[5]
    m_E[ 5]           m_V[5], m_V[0]
    m_E[ 6]           m_V[0], m_V[3]
    m_E[ 7]           m_V[1], m_V[4]
    m_E[ 8]           m_V[2], m_V[5]

  face              boundary edges
    m_F[0]            +m_E[0] +m_E[7]  -m_E[3] -m_E[6]
    m_F[1]            +m_E[1] +m_E[8]  -m_E[4] -m_E[7]
    m_F[2]            +m_E[2] +m_E[6]  -m_E[5] -m_E[8]
    m_F[3]            +m_E[3] +m_E[8]  -m_E[7] -m_E[11]
    m_F[4]            -m_E[2]  -m_E[1] -m_E[0]
    m_F[5]            +m_E[3] +m_E[4]  +m_E[5]
*/
ON_DECL
ON_Brep* ON_BrepWedge( const ON_3dPoint* corners, ON_Brep* pBrep = NULL );

/*
Description:
  Get an ON_Brep definition of a sphere.
Parameters:
  sphere - [in]
  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the sphere with a single face,
  a single edge along the seam, and vertices at the north
  and south poles.
*/
ON_DECL
ON_Brep* ON_BrepSphere( const ON_Sphere& sphere, ON_Brep* pBrep = NULL );

/*
Description:
  Get an ON_Brep definition of a torus.
Parameters:
  torus - [in]
  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the torus with a single face
  a two edges along the seams.
*/
ON_DECL
ON_Brep* ON_BrepTorus( const ON_Torus& torus, ON_Brep* pBrep = NULL );

/*
Description:
  Get an ON_Brep definition of a cylinder.
Parameters:
  cylinder - [in] cylinder.IsFinite() must be true
  bCapBottom - [in] if true end at cylinder.m_height[0] should be capped
  bCapTop - [in] if true end at cylinder.m_height[1] should be capped
  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the cylinder with a single
  face for the cylinder, an edge along the cylinder seam, 
  and vertices at the bottom and top ends of this seam edge.
  The optional bottom/top caps are single faces with one
  circular edge starting and ending at the bottom/top vertex.
*/
ON_DECL
ON_Brep* ON_BrepCylinder( const ON_Cylinder& cylinder, 
                          ON_BOOL32 bCapBottom,
                          ON_BOOL32 bCapTop,
                          ON_Brep* pBrep = NULL );

/*
Description:
  Get an ON_Brep definition of a cone.
Parameters:
  cylinder - [in] cylinder.IsFinite() must be true
  bCapBase - [in] if true the base of the cone should be capped.
  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the cone with a single
  face for the cone, an edge along the cone seam, 
  and vertices at the base and apex ends of this seam edge.
  The optional cap is asingle face with one circular edge 
  starting and ending at the base vertex.
*/
ON_DECL
ON_Brep* ON_BrepCone( 
          const ON_Cone& cone, 
          ON_BOOL32 bCapBottom,
          ON_Brep* pBrep = NULL 
          );

/*
Description:
  Get an ON_Brep form of a surface of revolution.
Parameters:
  pRevSurface - [in] pointer to a surface of revolution.
     The brep will manage this pointer and delete it in ~ON_Brep.
  bCapStart - [in] if true, the start of the revolute is
     not on the axis of revolution, and the surface of revolution
     is closed, then a circular cap will be added to close
     of the hole at the start of the revolute.
  bCapEnd - [in] if true, the end of the revolute is
     not on the axis of revolution, and the surface of revolution
     is closed, then a circular cap will be added to close
     of the hole at the end of the revolute.
  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  @untitled table
  true     successful
  false    brep cannot be created from this surface.
Remarks:
  The surface class must be created with new because
  it will be destroyed with the delete operator
  in ~ON_Brep.
*/
ON_DECL
ON_Brep* ON_BrepRevSurface( 
          ON_RevSurface*& pRevSurface,
          ON_BOOL32 bCapStart,
          ON_BOOL32 bCapEnd,
          ON_Brep* pBrep = NULL 
          );


          
/*
Description:
  Create an ON_Brep trimmed plane.
Parameters:
  plane - [in] plane that will be trimmed.
  boundary - [in] a simple (no self intersections) closed
      curve that defines the outer boundary of the trimmed
      plane.  This curve is copied for use in the brep.
  pBrep - [in] if not NULL, this brep will be used and returned.
Returns:
  An ON_Brep representation of the trimmed plane with a single face.
See Also:
  ON_Brep::NewPlanarFaceLoop()
*/
ON_DECL
ON_Brep* ON_BrepTrimmedPlane( 
            const ON_Plane& plane, 
            const ON_Curve& boundary,
            ON_Brep* pBrep = NULL );

/*
Description:
  Get an ON_Brep definition of a trimmed plane.
Parameters:
  plane - [in] plane that will be trimmed.
  boundary - [in] a list of 3d curves that form a simple 
      (no self intersections) closed curve that defines the
      outer boundary of the trimmed plane.
  bDuplicateCurves - [in] if true, duplicates of the
       curves in the boundary array are used in the brep.  If false
       the curves in the boundary array are used in the brep
       and the brep's destructor will delete the curves.
  pBrep - [in] if not NULL, this brep will be used and
               returned.
Returns:
  An ON_Brep representation of the trimmed plane with a singe face.
See Also:
  ON_Brep::NewPlanarFaceLoop()
*/
ON_DECL
ON_Brep* ON_BrepTrimmedPlane( 
            const ON_Plane& plane, 
            ON_SimpleArray<ON_Curve*>& boundary,
            ON_BOOL32 bDuplicateCurves = true,
            ON_Brep* pBrep = NULL );


/*
Description:
  Extrude a brep
Parameters:
  brep - [in/out]
  path_curve - [in] path to extrude along.
  bCap - [in] if true, the extusion is capped with a translation
              of the input brep.
Returns:
  True if successful.
See Also:
  ON_BrepExtrudeFace
  ON_BrepExtrudeLoop
  ON_BrepExtrudeEdge
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new faces are appended to brep.m_F[]. It is the caller's
  responsibility to insure the result does not self intersect.
*/
ON_DECL
bool ON_BrepExtrude( 
          ON_Brep& brep,
          const ON_Curve& path_curve,
          bool bCap = true
          );

/*
Description:
  Extrude a face in a brep.
Parameters:
  brep - [in/out]
  face_index - [in] index of face to extrude.
  path_curve - [in] path to extrude along.
  bCap - [in] if true, the extusion is capped with a translation
              of the face being extruded.
Example:
  Extrude a face along a vector.

          ON_Brep brep = ...;
          int face_index = ...;
          ON_3dVector v = ...;
          ON_LineCurve line_curve( ON_Line( ON_origin, vector ) );
          ON_BrepExtrudeFace( brep, face_index, line_curve, true );

Returns:
  @untitled table
  0    failure
  1    successful - no cap added
  2    successful - cap added as last face
See Also:
  ON_BrepExtrude
  ON_BrepExtrudeLoop
  ON_BrepExtrudeEdge
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new faces are appended to brep.m_F[].  If a cap is requested
  it is the last face in the returned brep.m_F[]
*/
ON_DECL
int ON_BrepExtrudeFace( 
          ON_Brep& brep,
          int face_index,
          const ON_Curve& path_curve,
          bool bCap = true
          );

/*
Description:
  Extrude a loop in a brep.
Parameters:
  brep - [in/out]
  loop_index - [in] index of face to extrude.
  path_curve - [in] path to extrude along.
  bCap - [in] if true and the loop is closed, the extusion
              is capped.
Returns:
  @untitled table
  0    failure
  1    successful - no cap added
  2    successful - cap added as last face
See Also:
  ON_BrepExtrude
  ON_BrepExtrudeFace
  ON_BrepExtrudeEdge
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new faces are appended to brep.m_F[].  If a cap is requested
  it is the last face in the returned brep.m_F[]
*/
ON_DECL
int ON_BrepExtrudeLoop( 
          ON_Brep& brep,
          int loop_index,
          const ON_Curve& path_curve,
          bool bCap = true
          );

/*
Description:
  Extrude an edge in a brep.
Parameters:
  brep - [in/out]
  edge_index - [in] index of face to extrude.
  path_curve - [in] path to extrude along.
Returns:
  @untitled table
  0    failure
  1    successful
See Also:
  ON_BrepExtrude
  ON_BrepExtrudeFace
  ON_BrepExtrudeLoop
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new face is appended to brep.m_F[].
*/
ON_DECL
int ON_BrepExtrudeEdge( 
          ON_Brep& brep,
          int edge_index,
          const ON_Curve& path_curve
          );


/*
Description:
  Extrude a vertex in a brep.
Parameters:
  brep - [in/out]
  vertex_index - [in] index of vertex to extrude.
  path_curve - [in] path to extrude along.
Returns:
  @untitled table
  0    failure
  1    successful
See Also:
  ON_BrepExtrude
  ON_BrepExtrudeFace
  ON_BrepExtrudeLoop
  ON_BrepExtrudeEdge
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new vertex is appended to brep.m_V[] and
  the new edge is appended to brep.m_E[].
*/
ON_DECL
int ON_BrepExtrudeVertex( 
          ON_Brep& brep,
          int vertex_index,
          const ON_Curve& path_curve
          );


/*
Description:
  Cone a face in a brep.
Parameters:
  brep - [in/out]
  face_index - [in] index of face to extrude.
  apex_point - [in] apex of cone.
Returns:
  @untitled table
  0    failure
  1    successful
See Also:
  ON_BrepExtrudeFace
  ON_BrepExtrudeLoop
  ON_BrepExtrudeEdge
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new faces are appended to brep.m_F[].
*/
ON_DECL
int ON_BrepConeFace( 
          ON_Brep& brep,
          int face_index,
          ON_3dPoint apex_point
          );

/*
Description:
  Cone a loop in a brep.
Parameters:
  brep - [in/out]
  loop_index - [in] index of face to extrude.
  apex_point - [in] apex of cone.
Returns:
  @untitled table
  0    failure
  1    successful
See Also:
  ON_BrepExtrudeFace
  ON_BrepExtrudeLoop
  ON_BrepExtrudeEdge
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new faces are appended to brep.m_F[].
*/
ON_DECL
bool ON_BrepConeLoop( 
          ON_Brep& brep,
          int loop_index,
          ON_3dPoint apex_point
          );

/*
Description:
  Cone an edge in a brep.
Parameters:
  brep - [in/out]
  edge_index - [in] index of face to extrude.
  apex_point - [in] apex of cone.
Returns:
  @untitled table
  0    failure
  1    successful
See Also:
  ON_BrepExtrudeFace
  ON_BrepExtrudeLoop
  ON_BrepExtrudeEdge
  ON_BrepExtrudeVertex
  ON_BrepConeFace
  ON_BrepConeLoop
  ON_BrepConeEdge
Remarks:
  The new face is appended to brep.m_F[].
*/
ON_DECL
int ON_BrepConeEdge( 
          ON_Brep& brep,
          int edge_index,
          ON_3dPoint apex_point
          );

//These merge adjacent faces that have the same underlying surface.
ON_DECL
int ON_BrepMergeFaces(ON_Brep& B, int fid0, int fid1);

ON_DECL
bool ON_BrepMergeFaces(ON_Brep& B);

//This removes all slit trims  from F that are not joined to another face.
//Unlike ON_Brep::RemoveSlits(), this will remove slit pairs from a loop in cases 
//that will result in the creation of more loops. Caller is responsible for calling 
//ON_Brep::Compact() to get rid of deleted trims and loops.

ON_DECL
bool ON_BrepRemoveSlits(ON_BrepFace& F);

//Merges all possible edges
ON_DECL
void ON_BrepMergeAllEdges(ON_Brep& B);

/*
Description:
  Merges two breps into a single brep.  The
  result may be non-manifold or have multiple
  connected components.
Parameters:
  brep0 - [in]
  brep1 - [in]
  tolerance - [in]
Returns:
  Merged brep or NULL if calculation failed.
*/
ON_DECL
ON_Brep* ON_MergeBreps(
          const ON_Brep& brep0,
          const ON_Brep& brep1,
          double tolerance
          );

#endif

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

#if !defined(OPENNURBS_MESH_INC_)
#define OPENNURBS_MESH_INC_

///////////////////////////////////////////////////////////////////////////////
//
// Class  ON_Mesh
//
class ON_CLASS ON_MeshParameters
{
  // surface meshing perameters
public:

  enum MESH_STYLE
  {
    // All of these enum values must be in the range 0-255 because
    // unsigned chars are use for storage in some locations.
    unset_mesh_style      =   0,
    render_mesh_fast      =   1, // Use ON_MeshParameters::FastRenderMesh
    render_mesh_quality   =   2, // Use ON_MeshParameters::QualityRenderMesh
    // 3 - 8 reserved for future predefined render mesh styles
    render_mesh_custom    =   9,// Use ON_3dmSettings::m_CustomRenderMeshSettings
    render_mesh_per_object = 10 // Use ON_Object::GetMeshParameters().
  };

  /*
  Description:
    Parameters that create render meshes where meshing
    speed is prefered over mesh quality.
  */
  static 
  const ON_MeshParameters FastRenderMesh;

  /*
  Description:
    Parameters that create render meshes where mesh quality
    is prefered over meshing speed.
  */
  static 
  const ON_MeshParameters QualityRenderMesh;

  /*
  Description:
    Get a value to use for tolerance based on the relative_tolerance
    and actual size.
  Parameters:
    relative_tolerance - [in] 
      See m_relative_tolerance field
    actual_size - [in]
      Diagonal ov object bounding box or some similar measure of
      an object's 3d size.
  Returns:
    A value that can be used for m_tolerance if no
    user specified value is available.
  */
  static
  double Tolerance( double relative_tolerance, double actual_size );

  /*
  Description:
    Get a value to use for minimum edge length base on max_edge_length
    and tolerance settings.
  Parameters:
    max_edge_length - [in] 
      3d maximum edge length used to create mesh.
    tolerance - [in]
      3d distance tolerance used to create mesh.
  Returns:
    A value that can be used for m_min_edge_length if no
    user specified value is available.
  */
  static
  double MinEdgeLength( double max_edge_length, double tolerance );

  ON_MeshParameters();
  ~ON_MeshParameters();
  // C++ default works fine // ON_MeshParameters(const ON_MeshParameters& );
  // C++ default works fine // ON_MeshParameters& operator=(const ON_MeshParameters&);
 
  bool operator!=(const ON_MeshParameters&) const;
  bool operator==(const ON_MeshParameters&) const;

  // compares with mesh's mesh parameters
  bool operator==(const ON_Mesh&) const;
  bool operator!=(const ON_Mesh&) const;
  
  void Dump( ON_TextLog& test_log ) const;

  void Default(); 

  /*
  Description:
    Tool for provding a simple slider interface.
  Parameters:
    density - [in] 0.0 <= density <= 1.0
      0 quickly creates coarse meshes.
      1 creates accurate meshes but takes lots of time.
  */
  void Set(
    double density,
    double min_edge_length = 0.0001
    );

  /*
  Description:
    Sets the meshing parameters to ON_MeshParameters::FastRenderMesh.
  */
  ON_DEPRECATED
  void JaggedAndFasterMeshParameters();

  /*
  Description:
    Sets the meshing parameters to ON_MeshParameters::QualityRenderMesh.
  */
  ON_DEPRECATED
  void SmoothAndSlowerMeshParameters();

  /*
  Description:
    Sets the meshing parameters to create the default
    analysis mesh.
  */
  void DefaultAnalysisMeshParameters();

  // Compare() ignores weld and curvature settings
  // Ignores m_min_tolerance setting.
  int Compare( const ON_MeshParameters& ) const;

  /*
  Description:
    Compares all meshing parameters that control mesh geometry.
    Does not compare m_bCustomSettings, m_bComputeCurvature, 
    m_bDoublePrecision, m_min_tolerance, and m_texture_range.
  */
  int CompareGeometrySettings( const ON_MeshParameters& ) const;


  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );
  ON__UINT32 DataCRC(ON__UINT32) const;


  // Meshing happens in two stages.  The first stage creates a
  // rectangular grid.  The second stage refines the grid until
  // the mesh meets all meshing requirements.  The third stage
  // combines coincident vertices if the resulting mesh is a composite.
  
  bool m_bCustomSettings;    // false - if these settings were used to create
                             //         a mesh and the app settings don't match,
                             //         then remesh the object using the app
                             //         settings.
                             // true  - these settings are customized for a
                             //         particular object - ignore app mesh
                             //         settings.

  bool m_bComputeCurvature;  // false - (default) - ON_Mesh::m_K[] not computed
                             // true  - ON_Mesh::m_K[] computed

  bool m_bSimplePlanes;      // false - (default) planar surfaces are meshed
                             //          using the controls below.
                             // true   - planar surfaces are meshed using
                             //          minimal number of triangles and
                             //          aspect/edge controls are ignored.

  bool m_bRefine;            // false - skip stage 2
                             // true  - (default) do stage 2

  bool m_bJaggedSeams;       // false - (default) edges of meshes of joined 
                             //          b-rep faces match with no gaps or
                             //          "T" joints.
                             // true   - faces in b-reps are meshed independently.
                             //          This is faster but results in gaps and
                             //          "T" joints along seams between faces.

  bool m_bDoublePrecision;   // false - (default) the mesh vertices will be 
                             //         float precision values in the m_V[] array.
                             // true -  The mesh vertices will be double precision
                             //         values in the DoublePrecisionVertices()
                             //         array.  Float precision values will also
                             //         be returned in the m_V[] array.
  bool m_bCustomSettingsEnabled; // false - if these settings should be ignored
                             //         when used as per object custom render mesh 
                             //         settings.
                             //  true - ignore these settings.
  unsigned char m_mesher;    // 0 = slow mesher, 1 = fast mesher
    
  int m_texture_range;       // 1: normalized
                             //
                             //          each face has a normalized texture range 
                             //          [0,1]x[0,1].
                             //
                             // 2: packed normalized (default)
                             //
                             //          each face in a polysurface is assigned
                             //          a texture range that is a subrectangle 
                             //          of [0,1]x[0,1].  The subrectangles are 
                             //          mutually disjoint and packed into
                             //          into [0,1]x[0,1] in a way that minimizes
                             //          distortion and maximizes the coverage
                             //          of [0,1]x[0,1].  (This texture style 
                             //          is suitable for creating texture maps 
                             //          with popular 3D painting programs.)

private:
  unsigned int m_reserved2;
public:
                           
  // These controls are used in both stages

  double m_tolerance; // maximum distance from center of edge to surface

        
  double m_relative_tolerance; // If 0 < m_relative_tolerance < 1, 
  double m_min_tolerance;      // then the maximum distance from the
                               // center of an edge to the surface will
                               // be <= T, where T is the larger of
                               // (m_min_tolerance,d*m_relative_tolerance), 
                               // where d is an esimate of the size of the
                               // object being meshed.


  double m_min_edge_length; // edges shorter than m_min_edge_length will
                            // not be split even if the do not meet other
                            // meshing requirements

  double m_max_edge_length; // edges longer than m_max_edge_length will
                            // be split even when they meet all other
                            // meshing requirements

  // These controls are used during stage 1 to generate the grid
  double m_grid_aspect_ratio;  // desired aspect ratio of quads in grid
                               // 0.0 = any aspect ratio is acceptable
                               // values >0 and < sqrt(2) are treated as sqrt(2)
  int    m_grid_min_count;     // minimum number of quads in initial grid
  int    m_grid_max_count;     // desired masimum number of quads in initial grid
  double m_grid_angle;         // (in radians) maximum angle between surface
                               // normal evaluated at adjacent vertices.
                               // 0.0 is treated as pi.
  double m_grid_amplification; // The parameters above generate a grid.
                               // If you want fewer quads, set m_grid_amplification
                               // to a value < 1.  If you want more quads,
                               // set m_grid_amplification to a value > 1.
                               // default = 1 and values <= 0 are treated as 1.

  // These controls are used during stage 2 to refine the grid
  double m_refine_angle;       // (in radians) maximum angle in radians between
                               // surface normal evaluated at adjacent vertices.

  // These controls are used during stage 3
  int     m_face_type;         // 0 = mixed triangle and quads
                               // 1 = all triangles
                               // 2 = all quads
private:
  unsigned int m_reserved3;
};

class ON_CLASS ON_MeshCurvatureStats
{
public:
  ON_MeshCurvatureStats();
  ~ON_MeshCurvatureStats();
  ON_MeshCurvatureStats(const ON_MeshCurvatureStats& );
  ON_MeshCurvatureStats& operator=(const ON_MeshCurvatureStats&);

  void Destroy();
  void EmergencyDestroy();
  
  bool Set( ON::curvature_style,
            int,           // Kcount,
            const ON_SurfaceCurvature*, // K[]
            const ON_3fVector*, // N[] surface normals needed for normal sectional curvatures
            double = 0.0   // if > 0, value is used for "infinity"
            );

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  ON::curvature_style m_style;
  
  double m_infinity; // curvature values >= this are considered infinite
                     // and not used to compute the m_average or m_adev
  int    m_count_infinite; // number of "infinte" values
  int    m_count;    // count of "finite" values
  double m_mode;     // mode of "finite" values
  double m_average;  // average of "finite" values
  double m_adev;     // average deviation of "finite" values

  ON_Interval m_range;
};

///////////////////////////////////////////////////////////////////////////////
//
// Class  ON_MeshTopology
//

struct ON_MeshTopologyVertex
{
  // m_tope_count = number of topological edges that begin or 
  // end at this topological vertex.
  int m_tope_count;

  // m_topei[] is an array of length m_tope_count with the indices 
  // of the topological edges that begin or end at this topological
  // vertex.  Generally, these edges are listed in no particular
  // order.  If you want the edges listed "radially", then call
  // ON_MeshTopology::SortVertexEdges.
  const int* m_topei;

  // m_v_count = number of ON_Mesh vertices that correspond to 
  // this topological vertex.
  int m_v_count;

  // m_vi[] is an array of length m_v_count with the indices of the
  // ON_Mesh vertices that correspond to this topological vertex.
  const int* m_vi;
};

struct ON_MeshTopologyEdge
{
  // m_topvi[] = indices of the topological verteices where the 
  // edge begins and ends.
  int m_topvi[2];

  // m_topf_count = number of topological faces tat share this topological edge
  int m_topf_count;

  // m_topfi[] is an array of length m_topf_count with the indices of the
  // topological faces that share this topological edge.
  const int* m_topfi;
};

struct ON_CLASS ON_MeshTopologyFace
{
  /*
    m_topei[] = indices of the topological edges that bound the face.
    If m_topei[2] = m_topei[3], then the face is a triangle, otherwise
    the face is a quad.
 
    NOTE WELL:
      The topological edge with index m_topei[k] ENDS at the
      vertex corresponding to ON_MeshFace.vi[k]. So, ...

      If the face is a quad, (ON_MeshFace.vi[2]!=ON_MeshFace.vi[3]),
      the topological edge with index m_topei[0] STARTS at
      ON_MeshFace.vi[3] and ENDS at ON_MeshFace.vi[0],
      the topological edge with index m_topei[1] STARTS at
      ON_MeshFace.vi[0] and ENDS at ON_MeshFace.vi[1],
      the topological edge with index m_topei[2] STARTS at
      ON_MeshFace.vi[1] and ENDS at ON_MeshFace.vi[2], and
      the topological edge with index m_topei[3] STARTS at
      ON_MeshFace.vi[0] and ENDS at ON_MeshFace.vi[1],
      
      If the face is a triangle, (ON_MeshFace.vi[2]==ON_MeshFace.vi[3]),
      the topological edge with index m_topei[0] STARTS at
      ON_MeshFace.vi[2] and ENDS at ON_MeshFace.vi[0],
      the topological edge with index m_topei[1] STARTS at
      ON_MeshFace.vi[0] and ENDS at ON_MeshFace.vi[1],
      the topological edge with index m_topei[2] STARTS at
      ON_MeshFace.vi[1] and ENDS at ON_MeshFace.vi[2].
  */
  int m_topei[4];

  /*
    If m_reve[i] is 0, then the orientation of the edge matches the
    orientation of the face.  If m_reve[i] is 1, then the orientation
    of the edge is opposite that of the face.
  */
  char m_reve[4];

  /*
  Description:
    A topological mesh face is a valid triangle if m_topei[0], 
    m_topei[1], m_topei[2] are distinct edges and 
    m_topei[3]=m_topei[2].
  Returns:
    True if face is a triangle.
  */
  bool IsTriangle() const;

  /*
  Description:
    A topological mesh face is a valid quad if m_topei[0], 
    m_topei[1], m_topei[2], and m_topei[3] are distinct edges.
  Returns:
    True if face is a quad.
  */
  bool IsQuad() const;

  /*
  Description:
    A topological mesh face is valid if m_topei[0], m_topei[1], 
    and m_topei[2] are mutually distinct, and m_topei[3] is 
    either equal to m_topei[2] or mutually distinct from the
    first three indices.
  Returns:
    True if face is valid.
  */
  bool IsValid( ) const;
};

class ON_CLASS ON_MeshFace
{
public:
  int vi[4]; // vertex index - vi[2]==vi[3] for tirangles

  /*
  Returns:
    True if vi[2] == vi[3];
  Remarks:
    Assumes the face is valid.
  */
  bool IsTriangle() const;

  /*
  Returns:
    True if vi[2] != vi[3];
  Remarks:
    Assumes the face is valid.
  */
  bool IsQuad() const;

  /*
  Description:
    Determine if a face is valid by checking that the vertices
    are distinct.
  Parameters:
    mesh_vertex_count - [in]
      number of vertices in the mesh
    V - [in]
      optional array of mesh_vertex_count vertex locations.
  Returns:
    true
      The face is valid.
    false
      The face is not valid. It may be possible to repair the
      face by calling ON_MeshFace::Repair().
  */
  bool IsValid( 
        int mesh_vertex_count
        ) const;
  bool IsValid(
        int mesh_vertex_count,
        const ON_3fPoint* V
        ) const;
  bool IsValid(
        int mesh_vertex_count,
        const ON_3dPoint* V
        ) const;

  /*
  Description:
    Reverses the order of the vertices in v[].
    vi[0] is not changed.
  */
  void Flip();

  /*
  Description:
    If IsValid() returns false, then you can use Repair()
    to attempt to create a valid triangle. 
  Parameters:
    mesh_vertex_count - [in]
      number of vertices in the mesh
    V - [in]
      optional array of mesh_vertex_count vertex locations.
  Returns:
    true
     repair was successful and v[0], v[1], vi[2] have distinct valid
     values and v[2] == v[3].
    false
     this face's vi[] values cannot be repaired    
  */
  bool Repair(
        int mesh_vertex_count
        );
  bool Repair(
        int mesh_vertex_count,
        const ON_3fPoint* V
        );
  bool Repair(
        int mesh_vertex_count,
        const ON_3dPoint* V
        );

  /*
  Description:
    Compute the face normal
  Parameters:
    dV - [in] double precision vertex array for the mesh
    fV - [in] float precision vertex array for the mesh
    FN - [out] face normal
  Returns:
    true if FN is valid.
  */
  bool ComputeFaceNormal( const ON_3dPoint* dV, ON_3dVector& FN ) const;
  bool ComputeFaceNormal( const ON_3fPoint* fV, ON_3dVector& FN ) const;
};

struct ON_MeshFaceSide
{
  int vi[2]; // vertex indices
  int fi;    // mesh m_F[] array face index
  unsigned char  side;  // edge connects mesh m_V[m_F[fi].vi[side]] and m_V[m_F[fi].vi[(side+1)%4]]
  unsigned char  dir;   // 0 = counterclockwise, 1 = clockwise (reversed)
  unsigned short value; // Set to zero by ON_Mesh::GetFaceSideList(). Can be used as needed.
};


/*
Description:
  Sort the sides[] array of ON_MeshFaceSide structs in dictionary
  order by "vi[0]", "vi[1]", "fi", and "side" values.
Paramters:
  sides_count - [in]
    number of elements in the sides[] array.
  sides - [in/out]
Remarks:
  The function is thread safe.
*/
ON_DECL
void ON_SortMeshFaceSidesByVertexIndex( 
        int sides_count, 
        struct ON_MeshFaceSide* sides 
        );

struct ON_MeshPart
{
  // ON_Mesh faces with indices fi[0] <= i < fi[1] reference
  // vertices with indices vi[0] <= j < vi[1].
  int vi[2]; // subinterval of mesh m_V[] array
  int fi[2]; // subinterval of mesh m_F[] array
  int vertex_count;   // = vi[1] - vi[0];
  int triangle_count; // tris + 2*quads >= fi[1] - fi[0]
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_MeshFace>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_MeshTopologyVertex>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_MeshTopologyEdge>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_MeshTopologyFace>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<struct ON_MeshPart>;
#pragma warning( pop )
#endif

class ON_CLASS ON_MeshTopology
{
  // A mesh topology class is always associated with an ON_Mesh
  // and can be retrieved by calling ON_Mesh::Topology()
public:
  ON_MeshTopology();
  ~ON_MeshTopology();

  bool IsValid() const;

  void Dump( ON_TextLog& ) const;

  //////////
  // The parent ON_Mesh geometry used to compute this mesh topology.
  const ON_Mesh* m_mesh;

  //////////
  // number of topoligical vertices (<= m_mesh.VertexCount())
  int TopVertexCount() const;

  //////////
  // number of topoligical edges
  int TopEdgeCount() const;
  
  //////////
  // number of topoligical faces (same as m_mesh.FaceCount())
  int TopFaceCount() const;

  /*
  Description:
    Get a vertex reference to a mesh vertex index.
  Parameters:
    ci - [in] component index with type mesh_vertex or meshtop_vertex.
  Returns:
    a reference to the vertex
  */
  class ON_MeshVertexRef VertexRef(ON_COMPONENT_INDEX ci) const;

  class ON_MeshVertexRef VertexRef(int topv_index) const;

  /*
  Description:
    Get an edge reference.
  Parameters:
    ci - [in] component index with type meshtop_edge.
  Returns:
    a reference to the edge
  */
  class ON_MeshEdgeRef EdgeRef(ON_COMPONENT_INDEX ci) const;

  class ON_MeshEdgeRef EdgeRef(int tope_index) const;

  /*
  Description:
    Get a face reference from a mesh face index.
  Parameters:
    ci - [in] component index with type mesh_face.
  Returns:
    a reference to the face.
  Remarks:
    The OM_Mesh.m_F[] and ON_MeshTopology.m_topf[] arrays
    are parallel arrays; corresponding faces have identical
    indices.
  */
  class ON_MeshFaceRef FaceRef(ON_COMPONENT_INDEX ci) const;

  class ON_MeshFaceRef FaceRef(int topf_index) const;


  /*
  Description:
    Get the 3d point location of a vertex.
  Parameters:
    topv_index - [in];
  Returns:
    Location of vertex.
  */
  ON_3fPoint TopVertexPoint(
    int topv_index
    ) const;

  /*
  Description:
    Get the 3d line along an edge.
  Parameters:
    tope_index - [in];
  Returns:
    Line along edge.  If input is not valid,
    the line.from and to are ON_UNSET_POINT
  */
  ON_Line TopEdgeLine(
    int tope_index
    ) const;

  ////////
  // returns index of edge that connects topological vertices
  // returns -1 if no edge is found.
  int TopEdge(
    int vtopi0,
    int vtopi1 // ON_MeshTopology vertex topology indices
    ) const;

  ////////
  // returns ON_MeshTopology vertex topology index of a face
  // corner.  The face is triangle iv TopFaceVertex(2) = TopFaceVertex(3)
  bool GetTopFaceVertices(
    int topfi,    // ON_MeshTopology face topology index (= ON_Mesh face index)
    int topvi[4]  // ON_MeshTopology vertex indices returned here
    ) const;

  /*
  Description:
    Sort the m_topei[] list of a mesh topology vertex so that
    the edges are in radial order.  The "const" is a white
    lie to make this function easier to call.
  Parameter:
    topvi - [in] index of vertex in m_topv[] array.
  Remarks:
    A nonmanifold edge is treated as a boundary edge with respect
    to sorting.  If any boundary or nonmanifold edges end at the
    vertex, then the first edge will be a boundary or nonmanifold
    edge.
  */
  bool SortVertexEdges( int topvi ) const;

  /*
  Description:
    Sort the m_topei[] list of every mesh topology vertex so 
    that the edges are in radial order.  The "const" is a white
    lie to make this function easier to call.
  Remarks:
    Same as
    for ( int topvi = 0; topvi < m_topv.Count(); topvi++ )
      SortVertexEdges(topvi);
  */
  bool SortVertexEdges() const;

  /*
  Description:
    Returns true if the topological vertex is hidden. 
  Parameters:
    topvi - [in] mesh topology vertex index.
  Returns:
    True if mesh topology vertex is hidden.
  Remarks:
    The mesh topology vertex is hidden if and only if
    all the ON_Mesh vertices it represents is hidden.
  */
  bool TopVertexIsHidden( int topvi ) const;

  /*
  Description:
    Returns true if the topological edge is hidden. 
  Parameters:
    topei - [in] mesh topology edge index.
  Returns:
    True if mesh topology edge is hidden.
  Remarks:
    The mesh topology edge is hidden if and only if
    either of its mesh topology vertices is hidden.
  */
  bool TopEdgeIsHidden( int topei ) const;

  /*
  Description:
    Returns true if the topological face is hidden. 
  Parameters:
    topfi - [in] mesh topology face index.
  Returns:
    True if mesh topology face is hidden.
  Remarks:
    The mesh topology face is hidden if and only if
    any of its mesh topology edges are hidden.
  */
  bool TopFaceIsHidden( int topfi ) const;

  //////////
  // m_topv_map[] has length m_mesh.VertexCount() and 
  // m_topv[m_topv_map[vi]] is the topological mesh vertex that is assocated
  // the with the mesh vertex m_mesh.m_V[vi].
  ON_SimpleArray<int> m_topv_map;

  ////////////
  // Array of topological mesh vertices.  See the comments in the definition
  // of ON_MeshTopologyVertex for details.
  ON_SimpleArray<ON_MeshTopologyVertex> m_topv;

  ////////////
  // Array of topological mesh edges.  See the comments in the definition
  // of ON_MeshTopologyEdge for details.
  ON_SimpleArray<ON_MeshTopologyEdge> m_tope;

  ////////////
  // Array of topological mesh faces.  The topological face
  // m_topf[fi] corresponds to the mesh face ON_Mesh.m_F[fi].
  // See the comments in the definition of ON_MeshTopologyFace
  // for details. To get the indices of the mesh topology 
  // vertices at the face corners use 
  // topvi = m_topv_map[m_mesh.m_F[fi].vi[n]]
  ON_SimpleArray<ON_MeshTopologyFace> m_topf;

  /*
  Description:
    Expert user function for efficiently getting the
    integer arrays used by the ON_MeshTopologyVertex
    and ON_MeshTopologyEdge classes.
  Parameters:
    count - [in] number of integers in array
  Returns:
    pointer to integer array.  The array memory
    will be freed by ~ON_MeshTopology()
  */
  int* GetIntArray(int count);

private:
  friend class ON_Mesh;

  bool Create();
  void Destroy();
  void EmergencyDestroy();

  // efficient workspaces for
  struct memchunk
  {
    struct memchunk* next;
  } *m_memchunk;

  // NOTE: this field is a bool with valid values of 0 and 1.
  volatile int m_b32IsValid; // sizeof(m_bIsValid) must be 4 - it is used in sleep locks.
                    //    0: Not Valid
                    //    1: Valid
                    //   -1: Sleep locked - ON_Mesh::Topology() calculation is in progress
  int WaitUntilReady(int sleep_value) const; // waits until m_b32IsValid >= 0

private:
  // no implementation
  ON_MeshTopology(const ON_MeshTopology&);
  ON_MeshTopology& operator=(const ON_MeshTopology&);
};

struct ON_MeshNgon
{
  // Number of N-gon corners (N >= 3)
  int N;

  // N-gon vertex indices
  // An array of N indices into the mesh's m_V[] vertex array.
  // If the ON_MeshNgon is returned by the ON_MeshNgonList::AddNgon()
  // function, then the memory for vi is managed by the ON_MeshNgonList
  // class.  
  int* vi;

  // N-gon face indices
  // An array of N indices into the mesh's m_F[] face array.
  // Often, only N-2 indices are used. Unused indices are set to -1.
  // If the ON_MeshNgon is returned by the ON_MeshNgonList::AddNgon()
  // function, then the memory for fi is managed by the ON_MeshNgonList
  // class.  
  int* fi;
};

class ON_CLASS ON_MeshNgonList
{
public:
  ON_MeshNgonList();
  ~ON_MeshNgonList();
  ON_MeshNgonList(const ON_MeshNgonList&);
  ON_MeshNgonList& operator=(const ON_MeshNgonList&);


  /*
  Description:
    Add an N-gon to the list
  Parameters:
    N - [in] number of vertices ( >= 5)
    vi - [in] array of N vertex indices into the mesh's m_V[] array.
    fi - [in] array of N face indices into the mesh's m_F[] array.
              Unused indices are set to -1.  In many cases
              there are N-2 valid indices and these are triangles.
  Remarks:
    Adding an N-gon may invalidate any pointers previously
    returned by Ngon.
  */
  bool AddNgon(int N, const int* vi, const int* fi);
  struct ON_MeshNgon* AddNgon(int N);

  /*
  Returns:
    Number of Ngons
  */
  int NgonCount() const;

  /*
  Parameters:
    Ngon_index - [in] zero based index
  Returns:
    NULL or a pointer to the Ngon
  */
  ON_MeshNgon* Ngon(int Ngon_index) const;

  /*
  Description:
    If you know about how many ngons you will need,
    then use the function to reserve space for them.
  */
  bool ReserveNgonCapacity(int capacity);

  /*
  Description:
    Destroy N-gon list
  */
  void Destroy();

  /*
  Returns:
    Approximate number of bytes used by this class.
  */
  unsigned int SizeOf() const;

private:
  int m_ngons_count;
  int m_ngons_capacity;
  ON_MeshNgon* m_ngons;
  struct ON_NGON_MEMBLK* m_memblk_list;
};

class ON_CLASS ON_MeshPartition
{
public:
  ON_MeshPartition();
  ~ON_MeshPartition();

  // maximum number of vertices in a partition
  int m_partition_max_vertex_count;
  // maximum number of triangles in a partition (quads count as 2 triangles)
  int m_partition_max_triangle_count;

  // Partition i uses 
  // vertices m_V[j] where 
  //
  //   m_part[i].vi[0] <= j < m_part[i].vi[1] 
  //
  // and uses faces m_F[k] where
  //
  //    m_part[i].fi[0] <= k < m_part[i].fi[1]
  ON_SimpleArray<struct ON_MeshPart> m_part;
};



class ON_CLASS ON_MappingTag
{
public:
  ON_MappingTag();
  void Default();
  bool Write(ON_BinaryArchive&) const;
  bool Read(ON_BinaryArchive&);
  void Dump( ON_TextLog& ) const;
  void Transform( const ON_Xform& xform );
  void Set(const ON_TextureMapping& mapping);

  /*
  Description:
    Sets the tag to the value the meshes have that
    come out of ON_Brep::CreateMesh().
  */
  void SetDefaultSurfaceParameterMappingTag();

  int Compare( const ON_MappingTag& other,
               bool bCompareId = true,
               bool bCompareCRC = true,
               bool bCompareXform = true
               ) const;

  /*
  Returns:
    True if the mapping tag is set.
  */
  bool IsSet() const;

  /*
  Returns:
    True if the mapping tag is for a mapping with
    type ON_TextureMapping::srfp_mapping with
    m_uvw = identity.
  */
  bool IsDefaultSurfaceParameterMapping() const;

  // Identifies the mapping used to create the texture 
  // coordinates and records transformations applied 
  // to the mesh after the texture coordinates were
  // calculated.  If the texture mapping does not
  // change when the mesh is transformed, then set 
  // m_mesh_xform to zero so that compares will work right.
  //
  // 
  ON_UUID                 m_mapping_id;   // ON_TextureMapping::m_mapping_id
  ON_TextureMapping::TYPE m_mapping_type; // ON_TextureMapping::m_type
  ON__UINT32              m_mapping_crc;  // ON_TextureMapping::MappingCRC()
  ON_Xform                m_mesh_xform;
};

class ON_CLASS ON_TextureCoordinates
{
public:
  ON_TextureCoordinates();

  ON_MappingTag   m_tag;
  int                        m_dim; // 1, 2, or 3
  ON_SimpleArray<ON_3fPoint> m_T;   // texture coordinates
};


#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_MappingTag>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_TextureCoordinates>;
#pragma warning( pop )
#endif

class ON_CLASS ON_Mesh : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_Mesh);
public:
  ON_Mesh();
  ON_Mesh(
    int   initial_face_array_capacity,   // initial face array capacity
    int   initial_vertex_array_capacity, // initial vertex array capacity
    bool  has_vertex_normals,            // true if mesh has vertex normals
    bool  has_texture_coordinates        // true if mesh has texture coordinates
    );
  ON_Mesh( const ON_Mesh& );
  ON_Mesh& operator=( const ON_Mesh& );
  ~ON_Mesh();

  // Override of virtual ON_Object::MemoryRelocate
  void MemoryRelocate();

  // virtual ON_Object::DestroyRuntimeCache override
  void DestroyRuntimeCache( bool bDelete = true );

  void Destroy();
  void EmergencyDestroy(); // Call only when memory used by this class's
                           // members will soon become invalid for reasons 
                           // beyond your control. EmergencyDestroy() zeros
                           // anything that could possibly cause
                           // ~ON_Mesh() to crash.  Calling
                           // EmergencyDestroy() under normal conditions 
                           // will result in ~ON_Mesh() leaking
                           // memory.

  void DestroyTree( bool bDeleteTree = true );

  /////////////////////////////////////////////////////////////////
  // ON_Object overrides

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

  ON::object_type ObjectType() const;

  /////////////////////////////////////////////////////////////////
  // ON_Geometry overrides

  int Dimension() const;

  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // minimum
         double*,    // maximum
         ON_BOOL32 = false  // true means grow box
         ) const;

  /*
	Description:
    Get tight bounding box of the mesh.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      mesh's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      mesh is calculated.  The mesh is not modified.
	Returns:
    True if the returned tight_bbox is set to a valid 
    bounding box.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  ON_BOOL32 SwapCoordinates(
        int, int        // indices of coords to swap
        );

  // virtual ON_Geometry override
  bool EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const;


  /////////////////////////////////////////////////////////////////
  // Interface
  // 

  // creation
  bool SetVertex(
         int,              // vertex index
         const ON_3dPoint& // vertex location
         );
  bool SetVertex(
         int,              // vertex index
         const ON_3fPoint& // vertex location
         );
  bool SetVertexNormal(
         int,               // vertex index
         const ON_3dVector& // unit normal
         );
  bool SetVertexNormal(
         int,               // vertex index
         const ON_3fVector& // unit normal
         );
  bool SetTextureCoord(
         int,               // vertex index
         double, double     // texture coordinates
         );
  bool SetTriangle(
         int, // face index
         int,int,int // vertex indices
         );
  bool SetQuad(
         int, // face index
         int,int,int,int // vertex indices
         );

  /*
  Description:
    Get a vertex reference to a mesh vertex index.
  Parameters:
    ci - [in] component index with type mesh_vertex or meshtop_vertex.
  Returns:
    a reference to the vertex
  */
  ON_MeshVertexRef VertexRef(ON_COMPONENT_INDEX ci) const;

  ON_MeshVertexRef VertexRef(int mesh_V_index) const;

  /*
  Description:
    Get an edge reference from a mesh topology edge index.
  Parameters:
    ci - [in] component index with type meshtop_edge
  Returns:
    a reference to the edge
  */
  ON_MeshEdgeRef EdgeRef(ON_COMPONENT_INDEX ci) const;

  ON_MeshEdgeRef EdgeRef(int tope_index) const;

  /*
  Description:
    Get a face reference from a mesh face index.
  Parameters:
    ci - [in] component index with type mesh_face.
  Returns:
    a reference to the face
  */
  ON_MeshFaceRef FaceRef(ON_COMPONENT_INDEX ci) const;

  ON_MeshFaceRef FaceRef(int mesh_F_index) const;

  /*
  Parameters:
   ci - [in] a component index with type mesh_vertex, meshtop_vertex,
             meshtop_edge, or mesh_face.
  Returns:
    A pointer to an ON_MeshVertexRef, ON_MeshEdgeRef, or ON_MeshFaceRef.
    The caller must delete the returned object when it is no longer
    needed.
  See Also:
    ON_Mesh::VertexRef
    ON_Mesh::EdgeRef
    ON_Mesh::FaceRef
  */
  ON_Geometry* MeshComponent( 
      ON_COMPONENT_INDEX ci
      ) const;

  // query
  int VertexCount() const;
  int FaceCount() const;
  int QuadCount() const; // number of faces that are quads
  int TriangleCount() const; // number of faces that are triangles
  int InvalidFaceCount() const; // number of face that have invalid m_vi[] values.
  bool HasVertexNormals() const; // normals at vertices
  bool HasFaceNormals() const;
  bool HasTextureCoordinates() const;
  bool HasSurfaceParameters() const;
  bool HasPrincipalCurvatures() const;
  bool HasVertexColors() const;

  /*
  Returns:
    Number of vertices that are hidden.
  */
  int HiddenVertexCount() const;

  bool GetCurvatureStats( 
         ON::curvature_style, 
         ON_MeshCurvatureStats& 
         ) const;

  void InvalidateVertexBoundingBox(); // Call if defining geometry is changed by 
                             // directly manipulating the m_V[] array.
  void InvalidateVertexNormalBoundingBox(); // Call if defining geometry is changed by 
                             // directly manipulating the m_N[] array.
  void InvalidateTextureCoordinateBoundingBox(); // Call if defining geometry is changed by 
                             // directly manipulating the m_T[] array.
  void InvalidateCurvatureStats(); // Call if defining geometry is changed by 
                             // directly manipulating the m_T[] array.
  void InvalidateBoundingBoxes(); // Invalidates all cached bounding box information.


  void Flip(); // reverses face orientations and flips vertex and face normals

  void FlipVertexNormals(); // reverses vertex normals
  void FlipFaceNormals(); // reverses face normals
  void FlipFaceOrientation(); // reverses face orientation (does nothing to normals)

  void SetMeshParameters( const ON_MeshParameters& );
  const ON_MeshParameters* MeshParameters() const;
  void DeleteMeshParameters();

  
  bool UnitizeVertexNormals();
  bool UnitizeFaceNormals();
  bool CountQuads();

  /*
  Description:
    Splits all quads along the short diagonal.
  */
  bool ConvertQuadsToTriangles();

  /*
  Description:
    Joins adjacent triangles into quads if the resulting quad
    is nice.
  Parameters:
    angle_tol_radians - [in] Used to compare adjacent
      triangles' face normals.  For two triangles to be considered,
      the angle between their face normals has to be <= angle_tol_radians.
      When in doubt use ON_PI/90.0 (2 degrees).
    min_diagonal_length_ratio - [in] ( <= 1.0) For two triangles to be
       considered the ratio of the resulting quad's diagonals
       (length of the shortest diagonal)/(length of longest diagonal).
       has to be >= min_diagonal_length_ratio.
       When in doubt us .875.
  */
  bool ConvertTrianglesToQuads(
    double angle_tol_radians,
    double min_diagonal_length_ratio
    );

  bool ComputeFaceNormals();   // compute face normals for all faces
  bool ComputeFaceNormal(int); // computes face normal of indexed face

  /*
  Description:
    Get a list of pairs of faces that clash.
  Parameters:
    max_pair_count - [in]
      If max_pair_count > 0, then at most this many pairs
      will be appended to the clashing_pairs[] array.
      If max_pair_count <= 0, then all clashing pairs
      will be appended to the clashing_pairs[] array.
    clashing_pairs - [out]
      The faces indices of clashing pairs are appended
      to this array. 
  Returns:
    Number of pairs appended to clashing_pairs[].
  */
  int GetClashingFacePairs( 
    int max_pair_count,
    ON_SimpleArray< ON_2dex >& clashing_pairs
    ) const;

  /*
  Description:
    Cull clashing faces from the mesh.
  Parameters:
    what_to_cull - [in]
      0: when a pair of faces clash, cull both faces
      1: when a pair of faces clash, leave the face with the
         longest edge.
      2: when a pair of faces clash, cull the face with the
         longest edge.
      3: when a pair of faces clash, leave the face with
         the largest area.
      4: when a pair of faces clash, cull the face with
         the largest area.
  Returns:
    Number of faces culled from the mesh.
  Remarks:
    If a large face clashes with many small faces, the large
    face and one small face will be removed.  When a degenerate
    face is encountered, it is also culled.
  */
  int CullClashingFaces( int what_to_cull );

  int CullDegenerateFaces(); // returns number of degenerate faces

  int CullUnusedVertices(); // returns number of culled vertices

  // Description:
  //   Removes any unreferenced objects from arrays, reindexes as needed,
  //   and shrinks arrays to minimum required size.
  bool Compact();

  bool ComputeVertexNormals();    // uses face normals to cook up a vertex normal
  
  //////////
  // Scales textures so the texture domains are [0,1] and
  // eliminates any texture rotations.
  bool NormalizeTextureCoordinates();

	/////////
	// Description:
	//		Transposes the texture coordinates
	//  Returns
	//			true  -  success
	bool TransposeTextureCoordinates();
	bool TransposeSurfaceParameters();
 
	/////////
	// Description:
	//		Reverse one coordinate direction of the texture coordinates, within texture domain m_tex_domain
	//	Parameters:
	//		dir  -[in]	-   dir=0  first texture coordinate is reversed
	//									  dir=1 second texture coordinate is reversed
	//  Returns
	//			true  -  success
	bool ReverseTextureCoordinates( int dir );
	bool ReverseSurfaceParameters( int dir );
 


  /*
  Description:
    Use a texture mapping function to set the m_T[] values.
  Parameters:
    mapping - [in]
    mesh_xform - [in]
      If not NULL, the mapping calculation is performed as
      if the mesh were transformed by mesh_xform; the
      location of the mesh is not changed.
    bLazy - [in]
      If true and the m_T[] values were set using the same
      mapping parameters, then no calculation is performed.
  Returns:
    True if successful.
  See Also:
    ON_TextureMapping::GetTextureCoordinates
  */
  bool SetTextureCoordinates( 
          const class ON_TextureMapping& mapping,
					const class ON_Xform* mesh_xform = 0,
          bool bLazy = true
          );

  bool HasCachedTextureCoordinates() const;

  const ON_TextureCoordinates* CachedTextureCoordinates( 
          const ON_UUID& mapping_id 
          ) const;

  const ON_TextureCoordinates* SetCachedTextureCoordinates( 
          const class ON_TextureMapping& mapping,
					const class ON_Xform* mesh_xform = 0,
          bool bLazy = true
          );

  bool EvaluateMeshGeometry( const ON_Surface& ); // evaluate surface at tcoords
                                                  // to set mesh geometry

  // finds all coincident vertices and merges them if break angle is small enough
  bool CombineCoincidentVertices( 
          ON_3fVector, // coordinate tols for considering vertices
                       // to be coincident
          double  // cosine normal angle tolerance in radians
                  // if vertices are coincident, then they are combined
                  // if NormalA o NormalB >= this value
          );

  /*
  Description:
    Combines identical vertices.
  Parameters:
    bIgnoreVertexNormals - [in] If true, then vertex normals
      are ignored when comparing vertices.
    bIgnoreTextureCoordinates - [in] If true, then vertex
      texture coordinates, colors, and principal curvatures
      are ignored when comparing vertices.
  Returns:
    True if the mesh is changed, in which case the returned
    mesh will have fewer vertices than the input mesh.
  */
  bool CombineIdenticalVertices(
          bool bIgnoreVertexNormals = false,
          bool bIgnoreTextureCoordinates = false
          );

  void Append( const ON_Mesh& ); // appends a copy of mesh to this and updates
                                 // indices of appended mesh parts

  /*
  Description:
    Append a list of meshes. This function is much more efficient
    than making repeated calls to ON_Mesh::Append(const ON_Mesh&)
    when lots of meshes are being joined into a single large mesh.
  Parameters:
    count - [in]
      length of meshes[] array.
    meshes - [in]
      array of meshes to append.
  */
  void Append( int count, const ON_Mesh* const* meshes );
  
  /*
  Description:
    Expert user function to set m_is_closed member.  
    Setting this value correctly after a mesh is constructed 
    can save time when IsClosed() is called.
    This function sets the private member variable m_is_closed.
  Paramters:
    closed - [in]
      0: The mesh is not closed.  There is at least one face with an 
         edge that is geometrically distinct (as an unoriented line segment)
         from all other edges.
      1: The mesh is closed.  Every geometrically distict edge is used
         by two or more faces.
  */
  void SetClosed(int closed);

  /*
  Returns:
    True if every mesh "edge" has two or more faces.
  */
  bool IsClosed() const;

  /*
  Returns:
    True if every mesh "edge" has at most two faces.
  */
  bool IsManifold() const;

  /*
  Returns:
    True if the mesh is manifold and every pair of faces
    that share an "edge" have compatible orientations.
  */
  bool IsOriented() const;

  /*
  Description:
    Determine if the mesh is a manifold.
  Parameters:
    bTopologicalTest - [in]
      If true, the query treats coincident vertices as
      the same.
    pbIsOriented - [out]
      If the input pointer is not NULL, then the returned
      value of *pbIsOriented will be true if the mesh
      is a manifold and adjacent faces have compatible
      face normals.
    pbHasBoundary - [out]
      If the input pointer is not NULL, then the returned
      value of *pbHasBoundary will be true if the mesh
      is a manifold and there is at least one "edge"
      with no adjacent faces have compatible
      face normals.
  Returns:
    True if every mesh "edge" has at most two adjacent faces.
  */
  bool IsManifold(
    bool bTopologicalTest,
    bool* pbIsOriented = NULL,
    bool* pbHasBoundary = NULL
    ) const;

  /*
  Description:
    Expert user function to set m_is_solid member.  
    Setting this value correctly after a mesh is constructed 
    can save time when IsSolid() is called.
    This function sets the private member variable m_is_solid.
    If solid is nonzero, it will set m_is_closed to 1.
  Paramters:
    solid - [in]
      0: The mesh is not an oriented manifold solid mesh. Either
         the mesh is not closed, not manifold, or the faces are
         not oriented compatibly.
      1: The mesh is an oriented manifold solid whose face normals
         point outwards.
     -1: The mesh is an oriented manifold solid whose face normals
         point inwards.
  */
  void SetSolidOrientation(int solid_orientation);

  /*
  Description:
    Determine orientation of a mesh.
  Returns:
    +1     mesh is a solid with outward facing normals
    -1     mesh is a solid with inward facing normals
     0     mesh is not a solid
  See Also:
    ON_Mesh::IsSolid
  */
  int SolidOrientation() const;

  /*
  Description:
    Test mesh to see if it is a solid.  (A "solid" is
    a closed oriented manifold.)
  Returns:
    true       mesh is a solid
    fals       mesh is not a solid
  See Also:
    ON_Mesh::SolidOrientation
    ON_Mesh::IsManifold
  */
  bool IsSolid() const;

  /*
  Description:
    Appends a list of mesh edges that begin or end at the specified
    vertices to the edges[] array.
  Parameters:
    vcount - [in]
      number of vertices
    vertex_index - [in]
      array of vertex indices
    bNoDuplicates - [in]
      If true, then only one edges[] is added for each edge,
      the first vertex index will alwasy be less than the
      second, and the returned elements are sorted in dictionary
      order.
      If false and an edge is shared by multiple faces, then
      there will be an edges[] element added for each face and the
      order of the vertex indicies will indicate the orientation
      of the edge with respect to the face.  No sorting is performed
      in this case.
    edges - [out]
      Edges that begin or end at one of the specified vertices are
      appended to this array.  Each ON_2dex records the start and
      end vertex index.
  Returns:
    Number of ON_2dex values appended to the edges[] array.
  */
  int GetVertexEdges( 
    int vcount,
    const int* vertex_index, 
    bool bNoDuplicates,
    ON_SimpleArray<ON_2dex>& edges
    ) const;


  /*
  Description:
    Appends a list of mesh edges to the edges[] array.
  Parameters:
    edges - [out]
      Each edges[] element is a pair of vertex indices.  There
      is at least one face in the mesh with an edge running between
      the indicies.
  Returns:
    Number of ON_2dex values appended to the edges[] array.
  */
  int GetMeshEdges( 
    ON_SimpleArray<ON_2dex>& edges
    ) const;

  /*
  Description:
    Assign a unique id to each vertex location.  Coincident vertices
    get the same id.
  Parameters:
    first_vid - [in]
      Initial vertex id.  Typically 1 or 0.
    Vid - [out]
      If not null, then Vid[] sould be an array of length VertexCount().
      and the vertex ids will be stored in this array.  If null,
      the array will be allocated by calling onmalloc().  The returned
      array Vid[i] is the id of the vertex m_V[i].  If m_V[i] and
      m_V[j] are the same 3d point, then Vid[i] and Vid[j] will have
      the same value.
    Vindex - [out] (can be null)
      If Vindex is not null, then it must have length at least m_V.Count()
      and the returned array will be a permutation of (0,1,...,m_V.Count()-1)
      such (Vid[Vindex[0]], Vid[Vindex[1]], ..., Vid[Vindex[m_V.Count()-1]])
      is an increasing list of value.
  Returns:
    null if the mesh has no vertices.
    An array of length VertexCount(). If vertices m_V[i] and m_V[j]
    are coincident, then Vid[i] = Vid[j].  The id values begin at first_vid.
    The maximum vertex id is Vid[Vindex[m_V.Count()-1]].  The number of
    unique vertex locations is (Vid[Vindex[m_V.Count()-1]] - first_vid + 1).
  */
  int* GetVertexLocationIds( 
    int first_vid, 
    int* Vid, 
    int* Vindex
    ) const;

  /*
  Description:
    Get a list of the sides of every face.
  Parameters:
    Vid - [in] (can be null)
      If Vid is null, then the mesh m_V[] index values are used to set
      the ON_MeshFaceSide::vi[] values.
      If Vid is not null, then it must be an array of length VertexCount().
      The value Vid[mesh m_V[] index] will be used to set the
      ON_MeshFaceSide::vi[] values.
    sides - [out]
      If the input value of sides is not null, then sides[] must be long 
      enough to hold the returned side list.  The maximum posssible length
      is 4*FaceCount() for a mesh contining FaceCount() nondegenerate quads.
      If the input value of sides is null, memory will be allocated using
      onmalloc() and the caller is responsible for calling onfree() at an
      appropriate time.  This function fills in the sides[] array
      with face side information.  The returned list is sorted by sides[].fi
      and the sides[].side and each element has vi[0] <= vi[1].  
      The function ON_SortMeshFaceSidesByVertexIndex() can be used to sort the 
      list by the sides[].vi[] values.
  Returns:
    Number of elements added to sides[].
  Remarks:
    Faces with out of range ON_MeshFace.vi[] values are skipped. 
    Degenerate faces are processed, but degenerate sides (equal vertex indices)
    are not added to the list.
  */
  int GetMeshFaceSideList( 
      const int* Vid,
      struct ON_MeshFaceSide*& sides
      ) const;

  /*
  Description:
    Get a list of the geometrically uniqued edges in a mesh.
  Parameters:
    edge_list - [out]
      The edge list for this mesh is appended to edge_list[].  
      The ON_2dex i and j values are mesh->m_V[] array indices.
      There is exactly one element in edge_list[] for each
      unoriented 3d line segment in the mesh. The edges are 
      oriented the same way the corresponding ON_MeshTopology
      edge is oriented.
    ci_meshtop_edge_map - [out]
      If you call the verson of GetMeshEdgeList() with the ci_meshtop_edge_map[],
      parameter, then the edge in edge_list[i] cooresponds to the edge
      in ON_MeshTopology.m_tope[ci_meshtop_edge_map[i]]. The value
      ci_meshtop_edge_map[i] is useful if you need to convert an edge_list[]
      index into an ON_COMPONENT_INDEX with type meshtop_edge.
    ci_meshtop_vertex_map - [out]
      If you call the verson of GetMeshEdgeList() with the ci_meshtop_vertex_map[],
      parameter, then the vertex m_V[i] cooresponds to the vertex
      in ON_MeshTopology.m_topv[ci_meshtop_vertex_map[i]]. The value
      ci_meshtop_vertex_map[i] is useful if you need to convert an m_V[]
      index into an ON_COMPONENT_INDEX with type meshtop_vertex.
    edge_list_partition - [out] (can be null)
      The edge_list[] is always ordered so that edge_types
      are partitioned into contiguous regions. The edge_list_partition[5]
      values report the edge type regions.
      * If edge_type_partition[0] <= ei < edge_type_partition[1], then
        edge_list[ei] is an edge of exactly two faces and the vertices
        used by the faces are identical.  These are also called
        "manifold edges".
      * If edge_type_partition[1] <= ei < edge_type_partition[2], then
        edge_list[ei] is an edge of exactly two faces, but at least
        one of the vertices is duplicated.  These are also called
        "crease edges".
      * If edge_type_partition[2] <= ei < edge_type_partition[3], then
        edge_list[ei] is an edge of 3 or more faces. These are also called
        "nonmanifold edges".
      * If edge_type_partition[3] <= ei < edge_type_partition[4], 
        then edge_list[ei] is a boundary edge of exactly one mesh face.
        These are also called "naked edges".
  Returns:
    Number of edges added to edge_list[].
  Remarks:
    This calculation also sets m_closed.  If you modify the mesh's
    m_V or m_F information after calling this function, be sure to
    clear m_is_closed.
  */
  int GetMeshEdgeList( 
      ON_SimpleArray<ON_2dex>& edge_list, 
      int edge_type_partition[5] 
      ) const;

  int GetMeshEdgeList( 
      ON_SimpleArray<ON_2dex>& edge_list, 
      ON_SimpleArray<int>& ci_meshtop_edge_map,
      int edge_type_partition[5] 
      ) const;

  int GetMeshEdgeList( 
      ON_SimpleArray<ON_2dex>& edge_list, 
      ON_SimpleArray<int>& ci_meshtop_edge_map,
      ON_SimpleArray<int>& ci_meshtop_vertex_map,
      int edge_type_partition[5] 
      ) const;

  ///////////////////////////////////////////////////////////////////////
  //
  // mesh editing
  //

  /*
  Description:
    Replace a mesh edge with a vertex at its center and update
    adjacent faces as needed.
  Parameters:
    topei - [in] index of edge in MeshTopology().m_tope[] array
  Returns:
    true if successful.
  */
  bool CollapseEdge( int topei );

  /*
  Description:
    Tests a mesh edge to see if it is valid as input to
    ON_Mesh::SwapMeshEdge.
  Parameters:
    topei - [in] index of edge in MeshTopology().m_tope[] array
  Returns:
    true if edge can be swapped by ON_Mesh::SwapMeshEdge.
  See Also:
    ON_Mesh::SwapEdge
  */
  bool IsSwappableEdge( int topei );


  /*
  Description:
    If the edge is shared by two triangular face, then
    the edge is "swapped".
  Parameters:
    topei - [in] index of edge in MeshTopology().m_tope[] array
  Returns:
    true if successful
  See Also:
    ON_Mesh::IsSwappableEdge
  */
  bool SwapEdge( int topei );

  /*
  Description:
    Removes a face from a mesh and does not alter the
    geometry of the remaining mesh.
  Parameters:
    meshfi - [in] index of face in ON_Mesh.m_F[] array
  Remarks:
    This function calls DestroyTopology() and DestroyPartition().
    The caller is responsible for calling Compact() if that step
    is required.
  Returns:
    true if successful
  */
  bool DeleteFace( int meshfi );

  /*
  Description:
    Destroys the m_H[] array and sets m_hidden_count=0.
  */
  void DestroyHiddenVertexArray();

  /*
  Returns:
    If the mesh has some hidden vertices, then an array
    of length VertexCount() is returned and the i-th
    element is true if the i-th vertex is hidden.
    If no vertices are hidden, NULL is returned.
  */
  const bool* HiddenVertexArray() const;

  /*
  Description:
    Set the runtime vertex hidden flag.
  Parameters:
    meshvi - [in] mesh vertex index
    bHidden - [in] true to hide vertex
  */
  void SetVertexHiddenFlag( int meshvi, bool bHidden );

  /*
  Description:
    Returns true if the mesh vertex is hidden.  This is a runtime
    setting that is not saved in 3dm files.
  Parameters:
    meshvi - [in] mesh vertex index.
  Returns:
    True if mesh vertex is hidden.
  */
  bool VertexIsHidden( int meshvi ) const;

  /*
  Description:
    Returns true if the mesh face is hidden.  This is a runtime
    setting that is not saved in 3dm files.
  Parameters:
    meshfi - [in] mesh face index.
  Returns:
    True if mesh face is hidden.
  Remarks:
    A face is hidden if, and only if, at least one of its
    vertices is hidden.
  */
  bool FaceIsHidden( int meshvi ) const;


  ///////////////////////////////////////////////////////////////////////
  //
  // mesh topology
  //
  // In order to keep the mesh facet definition simple and make the mesh
  // definition easily used in common rendering application, if two facets
  // share a vertex location but have different normals, curvatures, 
  // textures, etc., at that common vertex location, then the vertex is
  // duplicated.  When the topology of the mesh needs to be known,
  // use Topology() to get a class that provides complete topological
  // information about the mesh.
  const ON_MeshTopology& Topology() const;

  ///////////////////////////////////////////////////////////////////////
  // If you modify the mesh in any way that may change its topology,
  // then call DestroyTopology().  Specifically if you add or remove
  // vertices or face, change vertex locations, or change the face m_vi[]
  // values, then you must call DestroyTopology().
  void DestroyTopology();

  /*
  Returns:
    This is an expert user function that returns true if the topology
    information is already calculated and cached.  It can be used to
    to avoid calling the Topology() function when the expensive creation
    step will be performed.
  */
  bool TopologyExists() const;


  ///////////////////////////////////////////////////////////////////////
  //
  // mesh partitions
  //
  // In ancient times, some rendering engines were only able to process
  // small batches of triangles and th CreatePartition() function was
  // provided to partition the mesh into subsets of vertices and faces
  // that those renering engines could handle.
  //
  const ON_MeshPartition* CreatePartition( 
                int, // maximum number of vertices in a partition
                int  // maximum number of triangles in a partition
                );
  const ON_MeshPartition* Partition() const;
  void DestroyPartition();

  /*
  Description:
    Extract the portion of this mesh defined by mesh_part.
  Parameters:
    mesh_part - [in]
      defines portion of the mesh to extract.
    mesh - [in] (can be null, cannot be = "this).
      If mesh is no null, the extracted mesh will be put into
      this mesh.  If mesh is null, the extracted mesh will
      be created in a mesh allocated on the heap using the
      new operator.
  Returns:
    A pointer to the submesh.  If the input mesh parameter is null,
    then the caller must delete this mesh when it is no longer needed.
    If the input is invalid, then null is returned.
  */
  ON_Mesh* MeshPart( 
    const ON_MeshPart& mesh_part,
    ON_Mesh* mesh 
    ) const;

  /*
  Description:
    Create a mesh that is a single face of this mesh.
  Parameters:
  Returns:
    A pointer to the submesh.  If the input mesh parameter is null,
    then the caller must delete this mesh when it is no longer needed.
    If the input is invalid, then null is returned.
  */
  ON_Mesh* DuplicateFace( 
    int face_index,
    ON_Mesh* mesh 
    ) const;

  ///////////////////////////////////////////////////////////////////////
  //
  // mesh N-gon lists.  
  //   ON_Mesh objects support faces that are triangle or quads.
  //   When a mesh is created from a format that supports N-gons
  //   for N larger than 4, an optional N-gon list can be added 
  //   that specifies the vertices and faces that make up the N-gon.
  //

  /*
  Description:
    If the mesh has an N-gon list, return a pointer to it.
  Returns:
    A pointer to the current N-gon list or NULL.
  */
  const class ON_MeshNgonList* NgonList() const;

  /*
  Description:
    If an N-gon list exists, it is returned and can be modified.
    If no N-gon list exists, a new empty list is returned and
    it can be modified.
  Returns:
    A pointer to the N-gon list that can be modified.
  */
  class ON_MeshNgonList* ModifyNgonList();

  /*
  Description:
    Destroy any existing N-gon list.
  */
  void DestroyNgonList();

  ///////////////////////////////////////////////////////////////////////
  //
  // mesh components
  //   ON_Mesh objects can consist of sets of faces that are isolated
  //   from any other sets of faces.  The following 2 functions will
  //   dissect a mesh into these sets, called components.  Not to be 
  //   confused with ON_COMPONENT_INDEX.

  /*
    Description:
      Calculates the components of a mesh and sets a label for each face in
      the facet_component_labels array.
    Parameters:
      bUseVertexConnections- [in]
        If this parameter is true, then facets that share a common vertex
        are considered connected.
        If this parameter is false, then facets must share an edge to
        be considered connected.
      bUseTopologicalConnections - [in]
        If this parameter is true, then geometric location is used
        to determine if facets are connected. 
        If this parameter is false, then facets must share the same vertex 
        or vertices to be considered connected.
      facet_component_labels- [out]
        facet_component_labels[] will be an array with the same size
        as ON_Mesh.m_F.Count() and facet_component_labels[i]
        is the component id m_F[i] belongs to.  The component id
        will be 1 to the number of compoents.
    Returns:
      Number of components on success, 0 on failure 
  */

  int GetConnectedComponents( bool bUseVertexConnections, 
                              bool bTopologicalConnections, 
                              ON_SimpleArray<int>& facet_component_labels
                            ) const;

  /*
    Description:
      Calculates the components of a mesh and sets a label for each face in
      the facet_component_labels array.
    Parameters:
      bUseVertexConnections- [in]
        If this parameter is true, then facets that share a common vertex
        are considered connected.
        If this parameter is false, then facets must share an edge to
        be considered connected.
      bUseTopologicalConnections - [in]
        If this parameter is true, then geometric location is used
        to determine if facets are connected. 
        If this parameter is false, then facets must share the same vertex 
        or vertices to be considered connected.
      components   - [out]
        New components are appended to this array
        if this parameter is null, then the components are just counted.
    Returns:
      Number of components on success, 0 on failure 
  */

  int GetConnectedComponents( bool bUseVertexConnections, 
                              bool bTopologicalConnections, 
                              ON_SimpleArray<ON_Mesh*>* components
                            ) const;


  /////////////////////////////////////////////////////////////////
  // 
  // Double precision vertex support
  // 

  /*
  Returns:
    True if the mesh has single and double precision
    vertices, and the values of the two sets are synchronized.
  */
  bool HasSynchronizedDoubleAndSinglePrecisionVertices() const;

  /*
  Returns:
    True if the mesh has double precision vertices.
  Remarks:
    This function returns true if a mesh has double
    precision vertex information, even if it is not
    updated. 
    
    Use ON_Mesh::DoublePrecisionVerticesAreValid()
    and ON_Mesh::SinglePrecisionVerticesAreValid() to 
    check the validity.  
    
    Use ON_Mesh::UpdateDoublePrecisionVertices()
    or ON_Mesh::UpdateSinglePrecisionVertices() to synchronize
    values of single and double precision vertices.
  */
  bool HasDoublePrecisionVertices() const;

  /*
  Parameters:
    bEnableDoublePrecisionVertices - [in]
      True to enable use of double precision vertices.
      False to destroy any existing precision vertices.
  */
  void EnableDoublePrecisionVertices(bool bEnableDoublePrecisionVertices);

  /*
  Description:
    If you modify the values of double precision vertices,
    then you must call UpdateSinglePrecisonVertices().
  Remarks:
    If double precision vertices are not present, this function
    does nothing.
  */
  void UpdateSinglePrecisionVertices();

  /*
  Description:
    If you modify the values of the single precision vertices
    in m_V[], then you must call UpdateDoublePrecisionVertices().
  Remarks:
    If double precision vertices are not present, this function
    does nothing.
  */
  void UpdateDoublePrecisionVertices();

  /*
  Description:
    If you have modified the single precision vertices
    and are certain they are valid, then call this 
    function to update crc information.
  Remarks:
    If double precision vertices are not present, this function
    does nothing.
  */
  void SetSinglePrecisionVerticesAsValid();

  /*
  Description:
    If you have modified the double precision vertices
    and are certain they are valid, then call this 
    function to update crc information.
  Remarks:
    If double precision vertices are not present, this function
    does nothing.
  */
  void SetDoublePrecisionVerticesAsValid();

  /*
  Description:
    The functions UpdateSinglePrecisionVertices(), 
    UpdateDoublePrecisionVertices(), and 
    SetSinglePrecisionVerticesAsValid() save
    the count and crc of the single precision vertex
    array. True is returned if there are no
    double precision vertices or the current
    count and crc of the single precision
    vertex array match the saved values.
  Remarks:
    If double precision vertices are not present, this function
    does nothing and returns true.
  */
  bool SinglePrecisionVerticesAreValid() const;

  /*
  Description:
    The functions UpdateSinglePrecisionVertices(), 
    UpdateDoublePrecisionVertices(), and 
    SetDoublePrecisionVerticesAsValid() save
    the count and crc of the double precision vertex
    array. True is returned if the current
    count and crc of the double precision
    vertex array match the saved values.
  Remarks:
    If double precision vertices are not present, this function
    does nothing and returns true.
  */
  bool DoublePrecisionVerticesAreValid() const;

  /*
  Description:
    The function removes all double precision vertex information.
  */
  void DestroyDoublePrecisionVertices();


  /////////////////////////////////////////////////////////////////
  // Implementation - mesh geometry

  // Vertex locations
  //   In a case where adjacent facets share a vertex
  //   location but have distinct normals or texture
  //   coordinates at that location, the vertex must
  //   be duplicated.

  /*
  Description:
    Get double precision vertices.  If they do not exist,
    they will be created and match the existing single
    precision vertices.
  Returns:
    Array of double precision vertices.  If you modify the
    values in this array, you must make the same modifications
    to the single precision vertices, or call 
    UpdateSinglePrecisonVertices().
  Example:

          // add a bunch of double precision information
          ON_3dPointArray& dv = mesh.DoublePrecisionVertices();
          for ( i = 0; i < lots; i++ )
          {
            dv[i] = ...
          }
          // This call updates the single precison values
          // in m_V[] and sets all the counts and CRCs that
          // are used in validity checking.
          mesh.UpdateSinglePrecisonVertices();
    
  Remarks:
    Avoid mulitple calls to DoublePrecisionVertices().
    It is most efficient to make one call, save a local 
    reference, and use the local reference as needed.
  */
  ON_3dPointArray& DoublePrecisionVertices();
  const ON_3dPointArray& DoublePrecisionVertices() const;

  /*
  Description:
    Get single precision vertices.
  Returns:
    Array of float precision vertices.  If you modify the
    values in this array, you must make the same modifications
    to the double precision vertices, or call 
    UpdateSinglePrecisonVertices().
  */
  ON_3fPointArray& SinglePrecisionVertices();
  const ON_3fPointArray& SinglePrecisionVertices() const;

  /*
  Description:
    In general,use one of
    ON_Mesh::SinglePrecisionVertices()
    or
    ON_Mesh::DoublePrecisionVertices()
    to get the array of vertex locations.  If you modify
    m_V[] directly and HasDoublePrecisionVertices() is true,
    then you must make the same modifications to the array
    returned by DoublePrecisionVertices().
  */
  ON_3fPointArray m_V;

  /*
  Returns:
    Location of the vertex.  If double precision vertices
    are present, the double precision vertex location is
    returned.  If vertex_index is out of range,
    ON_UNSET_VALUE is returned.
  */
  ON_3dPoint Vertex(int vertex_index) const;

  // m_F[] facets (triangles or quads)
  ON_SimpleArray<ON_MeshFace> m_F;

  // m_N[] OPTIONAL vertex unit normals
  // If m_N[] is empty or m_N.Count() != m_V.Count(), 
  // Either m_N[] has zero count or it m_N[j] is the
  // the unit vertex normal at m_V[j].
  ON_3fVectorArray m_N;

  // m_FN[] OPTIONAL face unit normals
  // If m_FN[] is empty or m_FN.Count() != m_F.Count(), 
  // then m_FN is ignored.  Otherwise m_FN[j] is the
  // unit normal for the facet m_F[j].
  ON_3fVectorArray m_FN;

  /////////////////////////////////////////////////////////////////
  // Implementation - texture coordinates
  //
  // OPTIONAL texture coordinates for each vertex

  // It would be nice if this were an ON_TextureCoordinates,
  // but that breaks lots of checked out code that assumes
  // m_T is an array of ON_2fPoints.
  ON_MappingTag m_Ttag; // OPTIONAL tag for values in m_T[]
  ON_2fPointArray m_T;  // OPTIONAL texture coordinates for each vertex

  // RUNTIME ONLY
  //   This array is used to cache texture coordinates used by
  //   rendering applications that require 1d texture coordinates,
  //   3d texture coordinates, or multiple sets of texture 
  //   coordinates (e.g. blended textures with different mappings).
  //   Users are responsible for verifying 
  //   m_TC[i].m_T.Count() = m_V.Count()
  ON_ClassArray<ON_TextureCoordinates> m_TC;  

  // If m_T.Count() == m_V.Count(), then the mesh has texture coordinates
  // and m_T[j] is the texture coordinate for vertex m_V[j].
  //
  // When opennurbs or Rhino meshes an ON_Surface or ON_Brep, the texture
  // coordinates have a "canonical" linear relationship with the surface 
  // parameters that is described in the next section.  However, various 
  // mappings, spherical, planar, cylindrical, etc., can be applied that 
  // change the values of the texture coordinates.
  //
  // If a texture mapping function was used to set the m_T[] values, 
  // then the id and serial number of the mapping function is saved
  // in m_mapping_id and m_mapping_sn. The intended use of these fields
  // is to make it easy to avoid unnecessary recalculation.  
  // If a mesh is modified, then m_mapping_id should be set to nil 
  // and m_mapping_crc should be set to 0.
  //
  /////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////
  // Implementation - surface parameters and packed texture 
  // information
  //
  // If m_S.Count() == m_V.Count(), then the mesh is a tesselation
  // of a parameteric surface and m_S[j] is the surface parameter at
  // m_V[j].  Storing values in m_S[] is OPTIONAL.
  //
  // If m_srf_scale[] has positive values, then they report
  // the world coordinate size of a rectangle that would 
  // minimize texture distortion if it were mapped to the
  // mesh using normalized surface evaluation parameters.
  // This information is used to calculate high quality 
  // packed texture coordinates.  
  ON_2dPointArray m_S;
  ON_Interval m_srf_domain[2]; // surface evaluation domain.
  double m_srf_scale[2];


  // Packed texture information.
  //
  // If either of the m_packed_tex_domain[] intervals is a 
  // proper subinterval of (0,1), then a texture packing 
  // calculation assigned this subrectangle to this mesh.

  ON_Interval m_packed_tex_domain[2];

  // The m_packed_tex_rotate setting is valid only when
  // m_S, m_srf_domain, m_packed_scale[] and 
  // m_packed_tex_domain[] are all valid and the texture
  // coordinates are based on surface evaluation parameters.
  // In this special situation, this boolean records the 
  // correspondence between the the surface parameters, (u,v),
  // and the packed texture coordinates, (s,t),
  //
  //   m_packed_tex_rotate = false:
  //     a = m_srf_domain[0].NormalizedParameterAt(u);
  //     b = m_srf_domain[1].NormalizedParameterAt(v);
  //     s = m_packed_tex_domain[0].ParameterAt(a);
  //     t = m_packed_tex_domain[1].ParameterAt(b);
  //
  //     x = m_packed_tex_domain[0].NormalizedParameterAt(s);
  //     y = m_packed_tex_domain[1].NormalizedParameterAt(t);
  //     u = m_srf_domain[0].ParameterAt(x);
  //     v = m_srf_domain[1].ParameterAt(y);
  //
  //   m_packed_tex_rotate = true:
  //     a = m_srf_domain[0].NormalizedParameterAt(u);
  //     b = m_srf_domain[1].NormalizedParameterAt(v);
  //     s = m_packed_tex_domain[0].ParameterAt(a);
  //     t = m_packed_tex_domain[1].ParameterAt(1.0-b);
  //
  //     x = m_packed_tex_domain[0].NormalizedParameterAt(s);
  //     y = m_packed_tex_domain[1].NormalizedParameterAt(t);
  //     u = m_srf_domain[0].ParameterAt(y);
  //     v = m_srf_domain[1].ParameterAt(1.0 - x);
  bool m_packed_tex_rotate;

  /*
  Returns:
    True if the m_srf_scale[] values are positive and
    the m_packed_tex_domain[] intervals are set to values
    that describe a proper subrectangle of (0,1)x(0,1).
    True does not necessarily mean the current values in
    m_T[] are packed texture coordinates.
  */
  bool HasPackedTextureRegion() const;

  /////////////////////////////////////////////////////////////////
  // Implementation - curvature

  ON_SimpleArray<ON_SurfaceCurvature> m_K;  // OPTIONAL surface curvatures
                                            // Either m_K[] has zero count or it has the same
                                            // count as m_V[], in which case m_K[j] reports
                                            // the surface curvatures at m_V[j].

  /////////////////////////////////////////////////////////////////
  // Implementation - false color
  ON_MappingTag m_Ctag; // OPTIONAL tag for values in m_C[]
  ON_SimpleArray<ON_Color> m_C;  // OPTIONAL vertex color
                                 // Either m_C[] has zero count or it has the same
                                 // count as m_V[], in which case m_C[j] reports
                                 // the color assigned to m_V[j].

  /////////////////////////////////////////////////////////////////
  // Implementation - runtime vertex visibility - not saved in 3dm files.
  ON_SimpleArray<bool> m_H; // OPTIONAL vertex visibility.
                            // If m_H.Count() = m_V.Count(), then
                            // m_H[vi] is true if the vertex m_V[vi] 
                            // is hidden.  Otherwise, all vertices are visible.
  int m_hidden_count;       // number of vertices that are hidden
                            // = number of true values in m_H[] array.

  /////////////////////////////////////////////////////////////////
  // Implementation - runtime UI information
  const ON_Object* m_parent; // runtime parent geometry (use ...::Cast() to get it)

protected:
  friend class ON_MeshVertexRef;
  friend class ON_MeshEdgeRef;
  friend class ON_MeshFaceRef;


  /////////////////////////////////////////////////////////////////
  // Implementation - mesh topology
  ON_MeshTopology m_top;

  ON_MeshParameters* m_mesh_parameters; // If mesh was created from a parametric surface,
                                        // these parameters were used to create the mesh.
  int                         m_invalid_count;
  int                         m_quad_count;
  int                         m_triangle_count;

private:
  char m_mesh_is_closed;   // 0 = unset, 1 = all edges have 2 or more faces, 2 = at least one boundary edge 
  char m_mesh_is_manifold; // 0 = unset, 1 = all edges have 1 or 2 faces, 2 = not manifold
  char m_mesh_is_oriented; // 0 = unset, 1 = faces normals agree across all edges that have 2 faces, 2 = not oriented
  char m_mesh_is_solid;    // 0 = unset, 1 = solid with outward face normals, 2 = solid with inward face normals, 3 = not solid

protected:
  // The bounding boxes are valid if m_?box[0][0] <= m_?box[0][1];
  float m_vbox[2][3]; // 3d bounding box of all referenced vertices
  float m_nbox[2][3]; // 3d bounding box of all referenced unit normals 
                      // (for estimation of Gauss map bounds)
  float m_tbox[2][2]; // 2d bounding box of all referenced texture coordinates
  ON_MeshCurvatureStats* m_kstat[4]; // gaussian,mean,min,max,sectionx,sectiony,sectionz

  // sub-mesh information rendering large meshes
  ON_MeshPartition* m_partition;

private:
  bool Write_1( ON_BinaryArchive& ) const; // uncompressed 1.x format
  bool Write_2( int, ON_BinaryArchive& ) const; // compressed 2.x format
  bool Read_1( ON_BinaryArchive& );
  bool Read_2( int, ON_BinaryArchive& );
  bool WriteFaceArray( int, int, ON_BinaryArchive& ) const;
  bool ReadFaceArray( int, int, ON_BinaryArchive& );
  bool SwapEdge_Helper( int, bool );
};

class ON_CLASS ON_MeshVertexRef : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_MeshVertexRef);
public:
  ON_MeshVertexRef();
  ~ON_MeshVertexRef();
  ON_MeshVertexRef& operator=(const ON_MeshVertexRef&);


  // parent mesh
  const ON_Mesh* m_mesh;
  
  // m_mesh->m_V[] index
  // (can be -1 when m_top_vi references a shared vertex location)
  int m_mesh_vi; 
  
  // m_mesh->m_top.m_tope[] index
  int m_top_vi; 


  /*
  Description:
    Override of the virtual ON_Geometry::ComponentIndex().
  Returns:
    A component index for the vertex.  The type of the returned
    component index can be 
    ON_COMPONENT_INDEX::mesh_vertex, 
    ON_COMPONENT_INDEX::meshtop_vertex, or
    ON_COMPONENT_INDEX::invalid_type.
  */
  ON_COMPONENT_INDEX ComponentIndex() const;

  /*
  Returns:
    The mesh topology associated with this 
    mesh vertex reference or NULL if it doesn't
    exist.
  */
  const ON_MeshTopology* MeshTopology() const;

  /*
  Returns:
    The 3d location of the mesh vertex.  Returns
    ON_UNSET_POINT is this ON_MeshVertexRef is not 
    valid.
  */
  ON_3dPoint Point() const;

  /*
  Returns:
    The mesh topology vertex associated with this 
    mesh vertex reference.
  */
  const ON_MeshTopologyVertex* MeshTopologyVertex() const;

  // overrides of virtual ON_Object functions
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const;
  unsigned int SizeOf() const;
  ON::object_type ObjectType() const;

  // overrides of virtual ON_Geometry functions
  int Dimension() const;
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );
};

class ON_CLASS ON_MeshEdgeRef : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_MeshEdgeRef);
public:
  ON_MeshEdgeRef();
  ~ON_MeshEdgeRef();
  ON_MeshEdgeRef& operator=(const ON_MeshEdgeRef&);

  // parent mesh
  const ON_Mesh* m_mesh;
  
  // m_mesh->m_top.m_tope[] index
  int m_top_ei; 

  /*
  Description:
    Override of the virtual ON_Geometry::ComponentIndex().
  Returns:
    A mesh component index for the edge.  The type is
    ON_COMPONENT_INDEX::meshtop_edge and the index is the
    index into the ON_MeshTopology.m_tope[] array.
  */
  ON_COMPONENT_INDEX ComponentIndex() const;

  /*
  Returns:
    The mesh topology associated with this 
    mesh edge reference or NULL if it doesn't
    exist.
  */

  const ON_MeshTopology* MeshTopology() const;
  /*
  Returns:
    The 3d location of the mesh edge.  Returns
    ON_UNSET_POINT,ON_UNSET_POINT, is this ON_MeshEdgeRef
    is not valid.
  */
  ON_Line Line() const;

  /*
  Returns:
    The mesh topology edge associated with this 
    mesh edge reference.
  */
  const ON_MeshTopologyEdge* MeshTopologyEdge() const;

  // overrides of virtual ON_Object functions
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const;
  unsigned int SizeOf() const;
  ON::object_type ObjectType() const;

  // overrides of virtual ON_Geometry functions
  int Dimension() const;
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );
};

class ON_CLASS ON_MeshFaceRef : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_MeshFaceRef);
public:
  ON_MeshFaceRef();
  ~ON_MeshFaceRef();
  ON_MeshFaceRef& operator=(const ON_MeshFaceRef&);

  // parent mesh
  const ON_Mesh* m_mesh;

  // m_mesh->m_F[] and m_mesh->m_top.m_tope[] index.
  int m_mesh_fi; 

  /*
  Description:
    Override of the virtual ON_Geometry::ComponentIndex().
  Returns:
    A mesh component index for the face.  The type is
    ON_COMPONENT_INDEX::mesh_face and the index is the
    index into the ON_Mesh.m_F[] array.
  */
  ON_COMPONENT_INDEX ComponentIndex() const;

  /*
  Returns:
    The mesh topology associated with this 
    mesh face reference or NULL if it doesn't
    exist.
  */
  const ON_MeshTopology* MeshTopology() const;

  /*
  Returns:
    The mesh face associated with this mesh face reference.
  */
  const ON_MeshFace* MeshFace() const;

  /*
  Returns:
    The mesh topology face associated with this 
    mesh face reference.
  */
  const ON_MeshTopologyFace* MeshTopologyFace() const;

  // overrides of virtual ON_Object functions
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const;
  unsigned int SizeOf() const;
  ON::object_type ObjectType() const;

  // overrides of virtual ON_Geometry functions
  int Dimension() const;
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );
};

/*
Description:
  Calculate a mesh representation of the NURBS surface's control polygon.
Parameters:
  nurbs_surface - [in]
  bCleanMesh - [in] If true, then degenerate quads are cleaned
                    up to be triangles. Surfaces with singular
                    sides are a common source of degenerate qauds.
  input_mesh - [in] If NULL, then the returned mesh is created
       by a class to new ON_Mesh().  If not null, then this 
       mesh will be used to store the conrol polygon.
Returns:
  If successful, a pointer to a mesh.
*/
ON_DECL
ON_Mesh* ON_ControlPolygonMesh( 
          const ON_NurbsSurface& nurbs_surface, 
          bool bCleanMesh,
          ON_Mesh* input_mesh = NULL
          );

/*
Description:
  Finds the unit normal to the triangle
Parameters:
  A - [in] triangle corner
  B - [in] triangle corner
  C - [in] triangle corner
Returns:
  Unit normal
*/
ON_DECL
ON_3dVector ON_TriangleNormal(
        const ON_3dPoint& A,
        const ON_3dPoint& B,
        const ON_3dPoint& C
        );


/*
Description:
  Finds the unit normal to the triangle
Parameters:
  A - [in] triangle corner
  B - [in] triangle corner
  C - [in] triangle corner
  a - [out] must not be null
  b - [out] must not be null
  c - [out] must not be null
  d - [out] must not be null
    The equation of the plane is a*x + b*y + c*z + d = 0
  ev_tol - [out]
    If ev_tol is not null, then it is the maximum absolute
    value of the plane equation evaluated at A,B,C.  Mathematically,
    ev_tol is zero.  Since these computations are performed with
    finite precision doubles, ev_tol is generally not zero.
Returns:
  Unit normal
*/
ON_DECL
bool ON_GetTrianglePlaneEquation(
        const ON_3dPoint& A,
        const ON_3dPoint& B,
        const ON_3dPoint& C,
        double* a,
        double* b,
        double* c,
        double* d,
        double* evaluation_tol
        );

#endif

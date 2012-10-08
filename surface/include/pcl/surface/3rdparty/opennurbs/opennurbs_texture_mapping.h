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
//   defines ON_TextureMapping
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_TEXTURE_MAPPING_INC_)
#define OPENNURBS_TEXTURE_MAPPING_INC_

///////////////////////////////////////////////////////////////////////////////
//
// Class ON_TextureMapping
//
class ON_Line;
class ON_BrepFace;
class ON_3dPoint;

typedef int  ( *TEXMAP_INTERSECT_LINE_SURFACE )( const ON_Line*, const ON_BrepFace*, ON_SimpleArray<ON_X_EVENT>& );
typedef bool ( *TEXMAP_BREP_FACE_CLOSEST_POINT )( const ON_BrepFace*, const ON_3dPoint*, ON_3dPoint& );

class ON_CLASS ON_TextureMapping : public ON_Object
{
public:
	ON_OBJECT_DECLARE(ON_TextureMapping);

	ON_TextureMapping();
	~ON_TextureMapping();

	// The copy constructor and operator= overrides are needed
  // to ensure m_geometry is properly copied.
	ON_TextureMapping(const ON_TextureMapping& src);
	ON_TextureMapping& operator=(const ON_TextureMapping& src);

  // overrides virtual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // overrides virtual ON_Object::Dump
  void Dump( ON_TextLog& ) const;

  // overrides virtual ON_Object::SizeOf
  unsigned int SizeOf() const;

  // overrides virtual ON_Object::Write
  ON_BOOL32 Write(
         ON_BinaryArchive& binary_archive
       ) const;

  // overrides virtual ON_Object::Read
  ON_BOOL32 Read(
         ON_BinaryArchive& binary_archive
       );

  void Default();

  virtual
  ON_UUID ModelObjectId() const;

	/*
	Determines whether the mapping, as currently set up, requires vertex normals to be present on the
	mesh in order to evaluate the mapping correctly.
		*/
	bool RequiresVertexNormals() const;
	bool IsPeriodic(void) const;

  /*
  Description:
	  Create a mapping that will convert surface parameters into 
    normalized (0,1)x(0,1) texture coordinates.
	*/
	bool SetSurfaceParameterMapping(void);

  /*
  Description:
    Create a planar projection texture mapping.
  Parameters:
    plane - [in]
    dx - [in]  portion of the plane's x axis that is mapped to [0,1]
               (can be a decreasing interval)               
    dy - [in]  portion of the plane's x axis that is mapped to [0,1]
               (can be a decreasing interval)               
    dz - [in]  portion of the plane's x axis that is mapped to [0,1]
               (can be a decreasing interval)       
    projection_method - [in] 
        1: Closest point mapping.
          A target point P is mapped to the point on the plane
          that is closest to P.  The target normal is ignored.
        2: Target line mapping.  A target point-vector pair
          (P, N), are mapped to the point on the plane
          where the line through P, parallel to N, intersects
          the plane.  If the line is parallel to the plane,
          the closest point mapping is used.
  Example:
    Create a mapping that maps the world axis aligned rectangle in
    the world yz plane with corners at (0,3,5) and (0,7,19) to the
    texture coordinate unit square.

          ON_3dVector plane_xaxis(0.0,1.0,0.0);
          ON_3dVector plane_yaxis(0.0,0,0,1.0);
          ON_3dPoint plane_origin(0.0,2.0,4.0);
          ON_Plane plane(plane_origin,plane_xaxis,plane_yaxis);
          ON_Interval dx( 0.0, 7.0 - 3.0);
          ON_Interval dy( 0.0, 19.0 - 5.0);
          ON_Interval dz( 0.0, 1.0 );
          ON_TextureMapping mapping;
          mapping.CreatePlaneMapping(plane,dx,dy,dz);

  Returns:
    True if input is valid.
  */
  bool SetPlaneMapping(
            const ON_Plane& plane,
            const ON_Interval& dx,
            const ON_Interval& dy,
            const ON_Interval& dz
            );

  /*
  Description:
    Create a cylindrical projection texture mapping.
  Parameters:
    cylinder - [in]  
        cylinder in world space used to define a cylindrical
        coordinate system.  The angular parameter maps (0,2pi)
        to texture "u" (0,1), The height parameter maps 
        (height[0],height[1]) to texture "v" (0,1), and 
        the radial parameter maps (0,r) to texture "w" (0,1).
    bIsCapped - [in]
        If true, the cylinder is treated as a finite
        capped cylinder.          
  Returns:
    True if input is valid.
  Remarks:
    When the cylinder is capped and m_texture_space = divided, 
    the cylinder is mapped to texture space as follows:
      The side is mapped to 0 <= "u" <= 2/3.
      The bottom is mapped to 2/3 <= "u" <= 5/6.
      The top is mapped to 5/6 <= "u" <= 5/6.
    This is the same convention box mapping uses.
  */
	bool SetCylinderMapping( 
		 const ON_Cylinder& cylinder,
		 bool bIsCapped
	);

  /*
  Description:
    Create a spherical projection texture mapping.
  Parameters:
    sphere - [in]  
        sphere in world space used to define a spherical
        coordinate system. The longitude parameter maps
        (0,2pi) to texture "u" (0,1).  The latitude paramter
        maps (-pi/2,+pi/2) to texture "v" (0,1).
        The radial parameter maps (0,r) to texture "w" (0,1).
  Returns:
    True if input is valid.
  */
	bool SetSphereMapping( 
		 const ON_Sphere& sphere
	);

  /*
  Description:
    Create a box projection texture mapping.
  Parameters:
    plane - [in]  
        The sides of the box the box are parallel to the 
        plane's coordinate planes.  The dx, dy, dz intervals
        determine the location of the sides.
    dx - [in]
       Determines the location of the front and back planes.
       The vector plane.xaxis is perpendicular to these planes
       and they pass through plane.PointAt(dx[0],0,0) and
       plane.PointAt(dx[1],0,0), respectivly.
    dy - [in]
       Determines the location of the left and right planes.
       The vector plane.yaxis is perpendicular to these planes
       and they pass through plane.PointAt(0,dy[0],0) and
       plane.PointAt(0,dy[1],0), respectivly.
    dz - [in] 
       Determines the location of the top and bottom planes.
       The vector plane.zaxis is perpendicular to these planes
       and they pass through plane.PointAt(0,0,dz[0]) and
       plane.PointAt(0,0,dz[1]), respectivly.
    bIsCapped - [in]
        If true, the box is treated as a finite
        capped box.          
  Returns:
    True if input is valid.
  Remarks:
    When m_texture_space = divided, the box is mapped to texture 
    space as follows:

    If the box is not capped, then each side maps to 1/4 of the texture map.

          v=1+---------+---------+---------+---------+
             | x=dx[1] | y=dy[1] | x=dx[0] | y=dy[0] |
             | Front   | Right   | Back    | Left    |
             | --y->   | <-x--   | <-y--   | --x->   |
          v=0+---------+---------+---------+---------+
            0/4 <=u<= 1/4 <=u<= 2/4 <=u<= 3/4 <=u<= 4/4

    If the box is capped, then each side and cap gets 1/6 of the texture map.

          v=1+---------+---------+---------+---------+---------+---------+
             | x=dx[1] | y=dy[1] | x=dx[0] | y=dy[0] | z=dx[1] | z=dz[0] |
             | Front   | Right   | Back    | Left    | Top     |  Bottom |
             | --y->   | <-x--   | <-y--   | --x->   | --x->   | --x->   |
          v=0+---------+---------+---------+---------+---------+---------+
            0/6 <=u<= 1/6 <=u<= 2/6 <=u<= 3/6 <=u<= 4/6 <=u<= 5/6 <=u<= 6/6 
  */
	bool SetBoxMapping( 
		 const ON_Plane& plane,
		 ON_Interval dx,
		 ON_Interval dy,
		 ON_Interval dz,
     bool bIsCapped
	);

	/*
  Description:
    Get plane mapping parameters from this texture mapping.
  Parameters:
    plane - [out]
    dx - [out]
      Portion of the plane's x axis that is mapped to [0,1]
    dy - [out]
      Portion of the plane's y axis that is mapped to [0,1]
    dz - [out]
      Portion of the plane's z axis that is mapped to [0,1]
  Returns:
	  True if valid plane mapping parameters were returned.
  Remarks:
    NOTE WELL:
      Generally, GetMappingPlane will not return the same
      parameters passed to SetPlaneMapping.  However, the
      location of the plane will be the same.
	*/
	bool GetMappingPlane(
		 ON_Plane& plane,
		 ON_Interval& dx,
		 ON_Interval& dy,
		 ON_Interval& dz
	   ) const;

	/*
  Description:
	  Get a cylindrical projection parameters from this texture mapping.
	Parameters:
	  cylinder - [out]  
  Returns:
	  True if a valid cylinder is returned.
  Remarks:
    Generally, GetMappingCylinder will not return the same
    parameters passed to SetCylinderMapping.  However, the
    location of the cylinder will be the same.  
    If this mapping is not cylindrical, the cylinder will
    approximate the actual mapping primitive.
	*/
	bool GetMappingCylinder( 
		 ON_Cylinder& cylinder
	) const;

	/*
  Description:
	  Get a spherical projection parameters from this texture mapping.
	Parameters:
	  sphere - [out]  
  Returns:
	  True if a valid sphere is returned.
  Remarks:
    Generally, GetMappingShere will not return the same
    parameters passed to SetSphereMapping.  However, the
    location of the sphere will be the same.
    If this mapping is not cylindrical, the cylinder will
    approximate the actual mapping primitive.
	*/
	bool GetMappingSphere( 
		 ON_Sphere& sphere
	) const;

	/*
	Get a box projection from the texture mapping.
	Parameters:
	plane - [out]  
		The center of the box is at plane.origin and the sides
		of the box are parallel to the plane's coordinate planes.
	dx - [out]
	   The "front" and "back" sides of the box are in spanned
	   by the vectors plane.yaxis and plane.zaxis.  The back
	   plane contains the point plane.PointAt(dx[0],0,0) and
	   the front plane contains the point plane.PointAt(dx[1],0,0).
	dy - [out]
	   The "left" and "right" sides of the box are in spanned
	   by the vectors plane.zaxis and plane.xaxis.  The left
	   plane contains the point plane.PointAt(0,dx[0],0) and
	   the back plane contains the point plane.PointAt(0,dy[1],0).
	dz - [out] 
	   The "top" and "bottom" sides of the box are in spanned
	   by the vectors plane.xaxis and plane.yaxis.  The bottom
	   plane contains the point plane.PointAt(0,0,dz[0]) and
	   the top plane contains the point plane.PointAt(0,0,dz[1]).
  Returns:
	  True if a valid box is returned.
  Remarks:
    Generally, GetMappingBox will not return the same
    parameters passed to SetBoxMapping.  However, the
    location of the box will be the same.
	*/
	bool GetMappingBox( 
		 ON_Plane& plane,
		 ON_Interval& dx,
		 ON_Interval& dy,
		 ON_Interval& dz
	) const;


  /*
  Description:
    Reverses the texture in the specified direction.
  Parameters:
    dir - [in] 0 = reverse "u", 1 = reverse "v", 2 = reverse "w".
  Remarks:
    Modies m_uvw so that the spedified direction transforms
    the texture coordinate t to 1-t.
  Returns:
    True if input is valid.
  */
  bool ReverseTextureCoordinate( int dir );

  /*
  Description:
    Swaps the specified texture coordinates.
  Parameters:
    i - [in]
    j - [in]
  Remarks:
    Modifies m_uvw so that the specified texture coordinates are swapped.
  Returns:
    True if input is valid.
  */
  bool SwapTextureCoordinate( int i, int j );

  /*
  Description:
    Tiles the specified texture coordinates.
  Parameters:
    dir - [in] 0 =  "u", 1 = "v", 2 = "w".
    count - [in] number of tiles
    offset - [in] offset of the tile
  Remarks:
    Modies m_uvw so that the specified texture coordinate is
    tiled.
  Returns:
    True if input is valid.
  */
  bool TileTextureCoordinate( int dir, double count, double offset );

  /*
  Description:
    Evaluate the mapping to get a texture coordinate.
  Parameters:
    P - [in] Vertex location
    N - [in] If the mapping projection is ray_projection,
             then this is the vertex unit normal.  Otherwise
             N is ignored.
    T - [out] Texture coordinate (u,v,w)

    P_xform -[in] 
      Transformation to be applied to P before performing
      the mapping calculation.
    N_xform - [in] 
      Transformation to be applied to N before performing
      the mapping calculation.  One way to calculate N_xform
      is to use the call P_xform::GetVectorTransform(N_xform).

  Returns:
    Nonzero if evaluation is successful.  When the mapping
    is a box or capped cylinder mapping, the value indicates 
    which side was evaluated.

      Cylinder mapping:
        1 = cylinder wall, 2 = bottom cap, 3 = top cap
      Box mapping:
        1 = front
        2 = right
        3 = back
        4 = left
        5 = bottom
        6 = top        

  See Also:
    ON_TextureMapping::GetTextureCoordinates
    ON_Mesh::SetTextureCoordinates
  */
  virtual
  int Evaluate( 
    const ON_3dPoint& P,
    const ON_3dVector& N,
    ON_3dPoint* T
    ) const;

  virtual
  int Evaluate( 
    const ON_3dPoint& P,
    const ON_3dVector& N,
    ON_3dPoint* T,
	  const ON_Xform& P_xform,
    const ON_Xform& N_xform
    ) const;

  int EvaluatePlaneMapping( 
    const ON_3dPoint& P,
    const ON_3dVector& N,
    ON_3dPoint* T
    ) const;

  int EvaluateSphereMapping( 
    const ON_3dPoint& P,
    const ON_3dVector& N,
    ON_3dPoint* T
    ) const;

  int EvaluateCylinderMapping( 
    const ON_3dPoint& P,
    const ON_3dVector& N,
    ON_3dPoint* T
    ) const;

  int EvaluateBoxMapping( 
    const ON_3dPoint& P,
    const ON_3dVector& N,
    ON_3dPoint* T
    ) const;

  /*
  Description:
    Quickly check to see if a mesh or tag has texture coordinates
    set by this mapping.
  Parameters:
    mesh - [in]
    tag - [in]
    object_xform - [in] (optional)
      If this transform is not NULL, then true will be
      returned only if the mapping function is the same and
      the tag's m_mesh_xform field is the same as mesh_xform.
      This parameter is typically NULL or the value of 
      ON_MappingRef::m_object_xform.
  Returns:
    True if the meshes texture coordinates were set by this
    mapping.
  */
  bool HasMatchingTextureCoordinates( 
         const ON_Mesh& mesh,
         const ON_Xform* object_xform = 0
         ) const; 
  bool HasMatchingTextureCoordinates( 
         const class ON_MappingTag& tag,
         const ON_Xform* object_xform = 0
         ) const; 

  /*
  Description:
    Get texture coordinates.  This calculation is
    expensive.  When possible, use a MappingMatch()
    query to avoid unnecessary calculations.
  Parameters:
    mesh - [in]
    T - [out] Texture coordinates returned here.
    mesh_xform - [in] (optional)
      If the mesh has been transformed since the texture mapping was set 
      up, pass the transformation here.  Typically this is the value
      of ON_Mesh::m_mapping_xform or ON_MappingRef::m_object_xform
    bLazy - [in]
      If true and the mesh.m_T[] values were calculated using
      this mapping, they are simply copied to the T[] array
      and no calculations are performed.  If you are calling
      the 3d point version and you care about the z-coordinate,
      then do not use the lazy option (meshes only store
      2d texture coordinates).
    Tside - [out]
      In the case of divided textures, side information is returned
      here if a lazy mapping is not done.  Otherwise Tside->Count()
      will be zero.
      Cylinder mapping:
        1 = cylinder wall, 2 = bottom cap, 3 = top cap
      Box mapping:
        1 = front
        2 = right
        3 = back
        4 = left
        5 = bottom
        6 = top        
  Example:
    
          ON_TextureMapping mapping = ...;
          const ON_Mesh* mesh = ...;
          bool bLazy = true;
          ON_SimpleArray<ON_3dPoint> T(mesh->VertexCount());
          T.SetCount(mesh->m_VertexCount());
          if ( !mapping.GetTextureCoordinates(mesh,3,3,&T[0].x,bLazy) )
            T.SetCount(0).

  Returns:
    True if successful.
  */
  bool GetTextureCoordinates( 
    const ON_Mesh& mesh, 
    ON_SimpleArray<ON_3fPoint>& T,
		const ON_Xform* mesh_xform = 0,
    bool bLazy = false,
    ON_SimpleArray<int>* Tside = 0
    ) const;

  bool GetTextureCoordinates( 
    const ON_Mesh& mesh, 
    ON_SimpleArray<ON_2fPoint>& T,
		const ON_Xform* mesh_xform = 0,
    bool bLazy = false,
    ON_SimpleArray<int>* Tside = 0
    ) const;

public:
  // The only reliable and persistent way to reference texture 
  // mappings is by the mapping_id.  If the mapping id is
  // set to m_srfp_mapping_id, then all other mapping settings
  // are ignored.
  ON_UUID m_mapping_id;

  // Runtime texture mapping table index. 
  // This value is NOT SAVED IN 3DM FILES.
  // This value is constant for each runtime instance of Rhino,
  // but can change each time a model is loaded or saved.  
  // Once a texture mapping is in the CRhinoDoc material table,
  // its id and index never change in that instance of Rhino.
  int m_mapping_index;

  // The texture mapping name is for UI and user comfort. 
  // Duplicates are permitted.
  ON_wString m_mapping_name;

  //////////////////////////////////////////////////////////
  //
  // Mapping types:
  //
  //   You can either calculate texture coordinates based on
  //   the parameterization of the surface used to create a mesh,
  //   or project the natural parameterization from a mapping
  //   primitive, like a plane, sphere, box, or cylinder.
  //
	// Do not change TYPE enum values - they are saved in 3dm files.
  //
  enum TYPE
  {
    no_mapping       = 0,

    srfp_mapping     = 1, // u,v = linear transform of surface params,w = 0
    plane_mapping    = 2, // u,v,w = 3d coordinates wrt frame
    cylinder_mapping = 3, // u,v,w = logitude, height, radius
    sphere_mapping   = 4, // (u,v,w) = longitude,latitude,radius
    box_mapping      = 5,
    mesh_mapping_primitive = 6, // m_mapping_primitive is an ON_Mesh 
    srf_mapping_primitive  = 7, // m_mapping_primitive is an ON_Surface
    brep_mapping_primitive = 8, // m_mapping_primitive is an ON_Brep

    force_32bit_mapping_type = 0xFFFFFFFF
  };

	TYPE m_type;

  //////////////////////////////////////////////////////////
  //
  // Projection:
  //
  //   When a mapping primitive, like a plane, sphere, box,
  //   or cylinder, is used, there are two projection options.
  //
  //  clspt_projection: world xyz maps to the point on the 
  //                    mapping primitive that is closest to xyz.
  //                    In this case, ON_TextureMapping::Evaluate
  //                    ignores the vector argument.
  //
  //  ray_projection:   world xyz + world vector defines a world line.
  //                    The world line is intersected with the mapping 
  //                    primitive and the intersection point that is
  //                    closest to the world xyz point is used to
  //                    calculate the mapping parameters.
  //
  //  The value of m_projection can be changed as needed.
  //
  //  If m_type = srfp_mapping, then m_projection is ignored.
  //
  enum PROJECTION
  {
    no_projection    = 0,
    clspt_projection = 1,
    ray_projection   = 2,
    force_32bit_mapping_projection = 0xFFFFFFFF
  };

  PROJECTION m_projection;

  //////////////////////////////////////////////////////////
  //
  // Texture space
  //
  //   When a mapping primitive is a box or a capped cylinder,
  //   there are two options for the mapping.  Either the sides
  //   all map to (0,1)x(0,1) (so the either texture map appears 
  //   on each side, or the sides map to distinct regions of the
  //   texture space.  
  //   
  enum TEXTURE_SPACE
  {
    single  = 0, // sides and caps map to same texture space
    divided = 1, // sides and caps map to distinct vertical
                 // regions of texture space.
                 // (0, 1/4, 2/4, 3/4, 1) for uncapped boxes.
                 // (0, 1/6, 2/6, 3/6, 4/6, 5/6, 1) for capped boxes.
                 // (0, 4/6, 5/6, 1) for capped cylinders.
    force_32bit_texture_space = 0xFFFFFFFF
  };
  
  TEXTURE_SPACE m_texture_space;

  // The m_bCapped applies to planar, cylinder and box mappings.
  // If m_bCapped is false, the cylinder or box is "infinite", if m_bCapped is true, they are finite.
  // In planar mappings, m_bCapped=false means "the Z texture coordinate will always be 0.0"
  // this is now the default behaviour in Rhino 5.0 - it's what users expect apparently.
  bool m_bCapped;

  //////////////////////////////////////////////////////////
  //
  // For primitive based mappings, these transformations are
  // used to map the world coordinate (x,y,z) point P and 
  // surface normal N before it is projected to the normalized 
  // mapping primitive. The surface normal transformation,
  // m_Nxyz, is always calculated from m_Pxyz.  It is a 
  // runtime setting that is not saved in 3dm files. 
  // If m_type is srfp_mapping, then m_Pxyz and m_Nxyz are
  // ignored.
  ON_Xform m_Pxyz;
  ON_Xform m_Nxyz;

  // Transform applied to mapping coordinate (u,v,w) to 
  // convert it into a texture coordinate.
  ON_Xform m_uvw;

  // Custom mapping primitive.
  ON_Object* m_mapping_primitive;

  static TYPE TypeFromInt( int i );
  static PROJECTION ProjectionFromInt( int i );
  static TEXTURE_SPACE TextureSpaceFromInt( int i);

  ON__UINT32 MappingCRC() const;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_TextureMapping>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_TextureMapping>;
#pragma warning( pop )
#endif


#endif


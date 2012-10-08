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
//   defines ON_Xform (4 x 4 transformation matrix)
//
////////////////////////////////////////////////////////////////

#if !defined(ON_XFORM_INC_)
#define ON_XFORM_INC_

class ON_Matrix;

class ON_CLASS ON_Xform
{
public:
  double m_xform[4][4]; // [i][j] = row i, column j.  I.e., 
                        //
                        //           [0][0] [0][1] [0][2] [0][3]
                        //           [1][0] [1][1] [1][2] [1][3]
                        //           [2][0] [2][1] [2][2] [2][3]
                        //           [3][0] [3][1] [3][2] [3][3]

  // use implicit destructor, copy constructor
  ON_Xform();                     // zero matrix

  ON_Xform( int );                // diagonal matrix (d,d,d,1)
  ON_Xform( double );             // diagonal matrix (d,d,d,1)

#if defined(ON_COMPILER_MSC)
  // Microsoft's compiler won't pass double m[4][4] as a const double[4][4] arg.
  // Gnu's compiler handles this.
  ON_Xform( double[4][4] );       // from standard double m[4][4]
  ON_Xform( float[4][4] );        // from standard float m[4][4]
#endif
  
  ON_Xform( const double[4][4] ); // from standard double m[4][4]
  ON_Xform( const float[4][4] );  // from standard float m[4][4]
  
  ON_Xform( const double* );      // from array of 16 doubles (row0,row1,row2,row3)
  ON_Xform( const float* );       // from array of 16 floats (row0,row1,row2,row3)
  
  ON_Xform( const ON_Matrix& ); // from upper left 4x4 of an
                                    // arbitrary matrix.  Any missing
                                    // rows/columns are set to identity. 
	ON_Xform(const ON_3dPoint& P,	// as a frame. 
						const ON_3dVector& X,	
						const ON_3dVector& Y,	
						const ON_3dVector& Z); 

  // use implicit operator=(const ON_3dVector&), operator==
  
  double* operator[](int);
  const double* operator[](int) const;

  // xform = scalar results in a diagonal 3x3 with bottom row = 0,0,0,1
  ON_Xform& operator=( int );
  ON_Xform& operator=( float );
  ON_Xform& operator=( double );
  ON_Xform& operator=( const ON_Matrix& ); // from upper left 4x4 of an
                                               // arbitrary matrix.  Any missing
                                               // rows/columns are set to identity.

  // All non-commutative operations have "this" as left hand side and
  // argument as right hand side.
  ON_2dPoint operator*( const ON_2dPoint& ) const;
  ON_3dPoint operator*( const ON_3dPoint& ) const;
  ON_4dPoint operator*( const ON_4dPoint& ) const;
  
  ON_2dVector operator*( const ON_2dVector& ) const;
  ON_3dVector operator*( const ON_3dVector& ) const;
  
  ON_Xform operator*( const ON_Xform& /*rhs*/ ) const;
  ON_Xform operator+( const ON_Xform& ) const;
  ON_Xform operator-( const ON_Xform& /*rhs*/ ) const;

  /*
  Description:
    Test the entries of the transformation matrix
    to see if they are valid number.
  Returns:
    True if ON_IsValid() is true for every number
    in the transformation matrix.
  */
  bool IsValid() const;

  /*
  Returns:
    true if matrix is the identity transformation

          1 0 0 0
          0 1 0 0
          0 0 1 0
          0 0 0 1
  Remarks:
    An element of the matrix is "zero" if fabs(x) <= zero_tolerance.
    An element of the matrix is "one" if fabs(1.0-x) <= zero_tolerance.
    If the matrix contains a nan, false is returned.
  */
  bool IsIdentity( double zero_tolerance = 0.0) const;
  
  /*
  Returns:
    true if the matrix is valid and is not the identity transformation
  Remarks:
    An element of the matrix is "zero" if fabs(x) <= zero_tolerance.
    An element of the matrix is "one" if fabs(1.0-x) <= zero_tolerance.
    If the matrix contains a nan, false is returned.
  */
  bool IsNotIdentity( double zero_tolerance = 0.0) const;
  
  /*
  Returns:
    true if matrix is a pure translation

          1 0 0 dx
          0 1 0 dy
          0 0 1 dz
          0 0 0 1
  Remarks:
    The test for zero is fabs(x) <= zero_tolerance.
    The test for one is fabs(x-1) <= zero_tolerance.
  */
  bool IsTranslation( double zero_tolerance = 0.0) const;
  
  /*
  Returns:
    true if matrix is the zero transformation

          0 0 0 0
          0 0 0 0
          0 0 0 0
          0 0 0 *
  */
  bool IsZero() const;

  /*
  Description:
    A similarity transformation can be broken into a sequence
    of dialations, translations, rotations, and reflections.
  Returns:
    +1: This transformation is an orientation preserving similarity.
    -1: This transformation is an orientation reversing similarity.
     0: This transformation is not a similarity.
  */
  int IsSimilarity() const;


  int Compare( const ON_Xform& other ) const;

  
  // matrix operations
  void Transpose(); // transposes 4x4 matrix

  int 
  Rank( // returns 0 to 4
    double* = NULL // If not NULL, returns minimum pivot
  ) const;

  double
  Determinant( // returns determinant of 4x4 matrix
    double* = NULL // If not NULL, returns minimum pivot
  ) const;

  bool
  Invert( // If matrix is non-singular, returns true,
          // otherwise returns false and sets matrix to 
          // pseudo inverse.
    double* = NULL // If not NULL, returns minimum pivot
  );

  ON_Xform
  Inverse(  // If matrix is non-singular, returns inverse,
            // otherwise returns pseudo inverse.
    double* = NULL // If not NULL, returns minimum pivot
  ) const;

  /*
  Description:
    When transforming 3d point and surface or mesh normals
    two different transforms must be used.
    If P_xform transforms the point, then the inverse
    transpose of P_xform must be used to tranform normal
    vectors.
  Parameters:
    N_xform - [out]
  Returns:
    The determinant of the transformation.
    If non-zero, "this" is invertable and N_xform can be calculated.
    False if "this" is not invertable, in which case
    the returned N_xform = this with the right hand column
    and bottom row zeroed out.
  */
  double GetSurfaceNormalXform( ON_Xform& N_xform ) const;

  /*
  Description:
    If a texture mapping is applied to an object, the object
    is subsequently transformed by T, and the texture mapping
    needs to be recalculated, then two transforms are required
    to recalcalculate the texture mapping.
  Parameters:
    P_xform - [out] 
      Transform to apply to points before applying the
      texture mapping transformation.
    N_xform - [out] 
      Transform to apply to surface normals before applying
      the texture mapping transformation.
  Returns:
    The determinant of the "this" transformation.
    If non-zero, "this" is invertable and P_xform and N_xform
    were calculated.
    False if "this" is not invertable, in which case
    the returned P_xform and N_xform are the identity.
  */
  double GetMappingXforms( ON_Xform& P_xform, ON_Xform& N_xform ) const;

  // Description:
  //   Computes matrix * transpose([x,y,z,w]).
  //
  // Parameters:
  //   x - [in]
  //   y - [in]
  //   z - [in]
  //   z - [in]
  //   ans - [out] = matrix * transpose([x,y,z,w])
  void ActOnLeft(
         double, // x
         double, // y
         double, // z
         double, // w
         double[4] // ans
         ) const;

  // Description:
  //   Computes [x,y,z,w] * matrix.
  //
  // Parameters:
  //   x - [in]
  //   y - [in]
  //   z - [in]
  //   z - [in]
  //   ans - [out] = [x,y,z,w] * matrix
  void ActOnRight(
         double, // x
         double, // y
         double, // z
         double, // w
         double[4] // ans
         ) const;

  ////////////////////////////////////////////////////////////////
  // standard transformations

  // All zeros including the bottom row.
  void Zero();

  // diagonal is (1,1,1,1)
  void Identity();

  // diagonal 3x3 with bottom row = 0,0,0,1
  void Diagonal(double); 

  /*
  Description:
    Create non-uniform scale transformation with the origin as
    a fixed point.
  Parameters:
    fixed_point - [in]
    x_scale_factor - [in]
    y_scale_factor - [in]
    z_scale_factor - [in]
  Remarks:
    The diagonal is (x_scale_factor, y_scale_factor, z_scale_factor, 1)
  */
  void Scale( 
    double x_scale_factor,
    double y_scale_factor,
    double z_scale_factor
    );

  /*
  Description:
    Create non-uniform scale transformation with the origin as
    a fixed point.
  Parameters:
    fixed_point - [in]
    scale_vector - [in]
  Remarks:
    The diagonal is (scale_vector.x, scale_vector.y, scale_vector.z, 1)
  */
  void Scale( 
    const ON_3dVector& scale_vector
    );

  /*
  Description:
    Create uniform scale transformation with a specified
    fixed point.
  Parameters:
    fixed_point - [in]
    scale_factor - [in]
  */
  void Scale
    (
    ON_3dPoint fixed_point,
    double scale_factor
    );

  /*
  Description:
    Create non-uniform scale transformation with a specified
    fixed point.
  Parameters:
    plane - [in] plane.origin is the fixed point
    x_scale_factor - [in] plane.xaxis scale factor
    y_scale_factor - [in] plane.yaxis scale factor
    z_scale_factor - [in] plane.zaxis scale factor
  */
  void Scale
    (
    const ON_Plane& plane,
    double x_scale_factor,
    double y_scale_factor,
    double z_scale_factor
    );

  /*
  Description:
    Create shear transformation.
  Parameters:
    plane - [in] plane.origin is the fixed point
    x1 - [in] plane.xaxis scale factor
    y1 - [in] plane.yaxis scale factor
    z1 - [in] plane.zaxis scale factor
  */
  void Shear
    (
    const ON_Plane& plane,
    const ON_3dVector& x1,
    const ON_3dVector& y1,
    const ON_3dVector& z1
    );

  // Right column is (d.x, d.y,d.z, 1).
  void Translation( 
    const ON_3dVector& // d
    );

  // Right column is (dx, dy, dz, 1).
  void Translation( 
    double, // dx
    double, // dy
    double  // dz
    );

  // Description:
  //   Get transformation that projects to a plane
  // Parameters:
  //   plane - [in] plane to project to
  // Remarks:
  //   This transformaton maps a 3d point P to the
  //   point plane.ClosestPointTo(Q).
  void PlanarProjection(
    const ON_Plane& plane
    );

  // Description: 
  //   The Rotation() function is overloaded and provides several
  //   ways to compute a rotation transformation.  A positive
  //   rotation angle indicates a counter-clockwise (right hand rule)
  //   rotation about the axis of rotation.
  //
  // Parameters:
  //   sin_angle - sin(rotation angle)
  //   cos_angle - cos(rotation angle)
  //   rotation_axis - 3d unit axis of rotation
  //   rotation_center - 3d center of rotation
  //
  // Remarks: 
  //   In the overloads that take frames, the frames should 
  //   be right hand orthonormal frames 
  //   (unit vectors with Z = X x Y).  
  //   The resulting rotation fixes
  //   the origin (0,0,0), maps initial X to 
  //   final X, initial Y to final Y, and initial Z to final Z.
  //  
  //   In the overload that takes frames with center points, 
  //   if the initial and final center are equal, then that 
  //   center point is the fixed point of the rotation.  If 
  //   the initial and final point differ, then the resulting
  //   transform is the composition of a rotation fixing P0
  //   and translation from P0 to P1.  The resulting 
  //   transformation maps P0 to P1, P0+X0 to P1+X1, ...
  //
  //   The rotation transformations that map frames to frames
  //   are not the same as the change of basis transformations
  //   for those frames.  See ON_Xform::ChangeBasis().
  //   
  void Rotation(
    double sin_angle,
    double cos_angle,
    ON_3dVector rotation_axis,
    ON_3dPoint rotation_center
    );

  // Parameters:
  //   angle - rotation angle in radians
  //   rotation_axis - 3d unit axis of rotation
  //   rotation_center - 3d center of rotation
  void Rotation(
    double angle_radians,
    ON_3dVector rotation_axis,
    ON_3dPoint rotation_center
    );

  /*
  Description:
    Calculate the minimal transformation that rotates
    start_dir to end_dir while fixing rotation_center.    
  */
  void Rotation(
    ON_3dVector start_dir,
    ON_3dVector end_dir,
    ON_3dPoint rotation_center
    );

  // Parameters:
  //   X0 - initial frame X
  //   Y0 - initial frame Y
  //   Z0 - initial frame Z
  //   X1 - final frame X
  //   Y1 - final frame Y
  //   Z1 - final frame Z
  //
  void Rotation( 
    const ON_3dVector& X0,
    const ON_3dVector& Y0,
    const ON_3dVector& Z0,
    const ON_3dVector& X1,
    const ON_3dVector& Y1,
    const ON_3dVector& Z1
    );

  // Parameters:
  //   P0 - initial frame center
  //   X0 - initial frame X
  //   Y0 - initial frame Y
  //   Z0 - initial frame Z
  //   P1 - initial frame center
  //   X1 - final frame X
  //   Y1 - final frame Y
  //   Z1 - final frame Z
  void Rotation( 
    const ON_3dPoint& P0,
    const ON_3dVector& X0,
    const ON_3dVector& Y0,
    const ON_3dVector& Z0,
    const ON_3dPoint& P1,
    const ON_3dVector& X1,
    const ON_3dVector& Y1,
    const ON_3dVector& Z1
    );

  /*
  Description:
    Create rotation transformation that maps plane0 to plane1.
  Parameters:
    plane0 - [in]
    plane1 - [in]
  */
  void Rotation( 
    const ON_Plane& plane0,
    const ON_Plane& plane1
    );

  /*
  Description:
    Create mirror transformation matrix.
  Parameters:
    point_on_mirror_plane - [in] point on mirror plane
    normal_to_mirror_plane - [in] normal to mirror plane
  Remarks:
    The mirror transform maps a point Q to
    Q - (2*(Q-P)oN)*N, where
    P = point_on_mirror_plane and N = normal_to_mirror_plane.
  */
  void Mirror(
    ON_3dPoint point_on_mirror_plane,
    ON_3dVector normal_to_mirror_plane
    );

  // Description: The ChangeBasis() function is overloaded 
  //   and provides several
  //   ways to compute a change of basis transformation.
  //
  // Parameters:
  //   plane0 - inital plane
  //   plane1 - final plane
  //
  // Returns:
  //   @untitled table
  //   true    success
  //   false   vectors for initial frame are not a basis
  //
  // Remarks: 
  //   If you have points defined with respect to planes, the
  //   version of ChangeBasis() that takes two planes computes
  //   the transformation to change coordinates from one plane to 
  //   another.  The predefined world plane ON_world_plane can
  //   be used as an argument.
  //
  //   If P = plane0.Evaluate( a0,b0,c0 ) and 
  //
  //   (a1,b1,c1) = ChangeBasis(plane0,plane1)*ON_3dPoint(a0,b0,c0),
  //
  //   then P = plane1.Evaluate( a1, b1, c1 )
  //          
  //   The version of ChangeBasis() that takes six vectors
  //   maps (a0,b0,c0) to (a1,b1,c1) where
  //   a0*X0 + b0*Y0 + c0*Z0 = a1*X1 + b1*Y1 + c1*Z1
  //
  //   The version of ChangeBasis() that takes six vectors
  //   with center points
  //   maps (a0,b0,c0) to (a1,b1,c1) where
  //   P0 + a0*X0 + b0*Y0 + c0*Z0 = P1 + a1*X1 + b1*Y1 + c1*Z1
  //
  //   The change of basis transformation is not the same as
  //   the rotation transformation that rotates one orthonormal
  //   frame to another.  See ON_Xform::Rotation().
  bool ChangeBasis( 
    const ON_Plane& plane0,
    const ON_Plane& plane1
    );

  // Description:
  //   Get a change of basis transformation.
  // Parameters:
  //   X0 - initial basis X (X0,Y0,Z0 can be any 3d basis)
  //   Y0 - initial basis Y
  //   Z0 - initial basis Z
  //   X1 - final basis X (X1,Y1,Z1 can be any 3d basis)
  //   Y1 - final basis Y
  //   Z1 - final basis Z
  // Remarks:
  //   Change of basis transformations and rotation transformations
  //   are often confused.  This is a change of basis transformation.
  //   If Q = a0*X0 + b0*Y0 + c0*Z0 = a1*X1 + b1*Y1 + c1*Z1
  //   then this transform will map the point (a0,b0,c0) to (a1,b1,c1)
  bool ChangeBasis( 
    const ON_3dVector& X0,
    const ON_3dVector& Y0,
    const ON_3dVector& Z0,
    const ON_3dVector& X1,
    const ON_3dVector& Y1,
    const ON_3dVector& Z1
    );

  // Parameters:
  //   P0 - initial center
  //   X0 - initial basis X (X0,Y0,Z0 can be any 3d basis)
  //   Y0 - initial basis Y
  //   Z0 - initial basis Z
  //   P1 - final center
  //   X1 - final basis X (X1,Y1,Z1 can be any 3d basis)
  //   Y1 - final basis Y
  //   Z1 - final basis Z
  // Remarks:
  //   Change of basis transformations and rotation transformations
  //   are often confused.  This is a change of basis transformation.
  //   If Q = P0 + a0*X0 + b0*Y0 + c0*Z0 = P1 + a1*X1 + b1*Y1 + c1*Z1
  //   then this transform will map the point (a0,b0,c0) to (a1,b1,c1)
  bool ChangeBasis( 
    const ON_3dPoint& P0,
    const ON_3dVector& X0,
    const ON_3dVector& Y0,
    const ON_3dVector& Z0,
    const ON_3dPoint& P1,
    const ON_3dVector& X1,
    const ON_3dVector& Y1,
    const ON_3dVector& Z1
    );

  // standard viewing transformations
  void WorldToCamera( 
         const ON_3dPoint&,  // CameraLocation
         const ON_3dVector&, // unit CameraX vector (right)
         const ON_3dVector&, // unit CameraY vector (up)
         const ON_3dVector&  // unit CameraZ vector (from screen to camera)
         );
  void CameraToWorld( 
         const ON_3dPoint&,  // CameraLocation
         const ON_3dVector&, // unit CameraX vector (right)
         const ON_3dVector&, // unit CameraY vector (up)
         const ON_3dVector&  // unit CameraZ vector (from screen to camera)
         );
  bool CameraToClip( // maps viewport frustum to -1 <= x,y,z <= 1 box
      ON_BOOL32, // true for perspective, false for orthographic
      double, double, // left != right (usually left < right )
      double, double, // bottom != top (usually bottom < top )
      double, double  // near != far (usually 0 < near < far )
      );

  // maps -1 <= x,y,z <= 1 box to viewport frustum
  bool ClipToCamera( 
      int, // true for perspective, false for orthographic
      double, double, // left != right (usually left < right )
      double, double, // bottom != top (usually bottom < top )
      double, double  // near != far an bot are non-zero (usually 0 < near < far )
      );

  // Computes transform that maps the clipping box 
  //
  //           -1<x<1,-1<y<1,-1<z<1 
  //
  // to the screen box
  //
  //          (left,right) X (bottom,top) X (near,far)
  bool ClipToScreen(                           
      double, // left
      double, // right
      double, // bottom
      double, // top
      double, // near_z
      double  // far_z
      );

  // Computes transform that maps the screen box
  //
  //          (left,right) X (bottom,top) X (near,far)
  //  
  // to the clipping box 
  //
  //           -1<x<1,-1<y<1,-1<z<1 
  bool ScreenToClip(
      double, // left
      double, // right
      double, // bottom
      double, // top
      double, // near_z
      double  // far_z
      );

  // Description: Computes homogeneous point clipping flags and
  //   returns an int with bits set to indicate if the point
  //   is outside of the clipping box.
  //
  // Parameters:
  //   point - [in] 4d homogeneous clipping coordinate point
  //
  // Returns:  
  //  @table  
  //   bit      point location
  //   1        x/w < -1
  //   2        x/w > +1
  //   4        y/w < -1
  //   8        y/w > +1
  //   16       z/w < -1
  //   32       z/w > +1
  //
  int ClipFlag4d(
    const double* // point
    ) const;

  // Parameters:
  //   count - [in] number of 4d points
  //   stride - [in] (>=4)
  //   points - [in] 4d clipping coordinate points 
  //            (array of stride*count doubles)
  //   bTestZ - [in] (default=true) if false, do not test "z" coordinate
  //
  int ClipFlag4d(
    int, // count
    int, // stride
    const double*, // points
    ON_BOOL32 = true // bTeztZ
    ) const;

  // Description: 
  //   Computes 3d point clipping flags and
  //   returns an int with bits set to indicate if the point
  //   is outside of the clipping box.
  //
  // Parameters:
  //   point - [in] 3d clipping coordinate point
  //
  // Returns:  
  //  @table  
  //   bit      point location
  //   1        x < -1
  //   2        x > +1
  //   4        y < -1
  //   8        y > +1
  //   16       z < -1
  //   32       z > +1
  int ClipFlag3d(
    const double* // point
    ) const;

  // Parameters:
  //   count - [in] number of 3d points
  //   stride - [in] (>=3)
  //   points - [in] 3d clipping coordinate points (array of stride*count doubles)
  //   bTestZ - [in] (default=true) if false, do not test "z" coordinate
  //
  int ClipFlag3d(
    int, // count
    int, // stride 
    const double*, // points
    ON_BOOL32 = true // bTestZ
    ) const;

  // Description: Computes 3d clipping flags for a 3d bounding
  //   box and returns an int with bits set to indicate if
  //   the bounding box is outside of the clipping box.
  //
  // Parameters:
  //   boxmin - [in] 3d boxmin corner
  //   boxmax - [in] 3d boxmax corner
  //
  // Returns:  
  //  @table  
  //   bit      box location
  //   1        boxmax x < -1
  //   2        boxmin x > +1
  //   4        boxmax y < -1
  //   8        boxmin y > +1
  //   16       boxmax z < -1
  //   32       boxmin z > +1
  int ClipFlag3dBox(
    const double*, // boxmin
    const double*  // boxmax
    ) const;


  /*
  Description:
    Calculates the transformation that linearly maps
    old_interval to new_interval.
  Parameters:
    dir - [in] 0 = x, 1 = y, 2= z;
    old_interval - [in]
    new_interval - [in]
  */
  bool IntervalChange(
    int dir,
    ON_Interval old_interval,
    ON_Interval new_interval
    );
};

class ON_CLASS ON_ClippingRegion
{
public:
  ON_ClippingRegion();

  // The transformation m_xform transforms the view frustum,
  // in object coordinates to the (-1,+1)^3 clipping 
  // coordinate box.
  ON_Xform m_xform;

  /*
  Parameters:
    clip_plane_tolerance - [in]  
      3d world coordinates tolerance to use when testing 
      objects to see if the planes in m_clip_plane[] hide
      the objects.      
  Remarks:
    The constructor sets this value to zero.  Rhino uses
    values around 1e-5.
  */
  void SetClipPlaneTolerance( double clip_plane_tolerance );

  /*
  Returns:
    3d world coordinates tolerance to use when testing 
    objects to see if the planes in m_clip_plane[] hide
    the objects.      
  Remarks:
    The constructor sets this value to zero.  Rhino uses
    values around 1e-5.
  */
  double ClipPlaneTolerance() const;

  enum
  {
    max_clip_plane_count = 16, // must be <= 25
    frustum_bitmask      = 0x0000003F,
    near_plane_bitmask   = 0x00000020,
    far_plane_bitmask    = 0x00000010,
    clip_plane_bitmask   = 0x7FFFFFC0,
    negw_bitmask         = 0x80000000
  };

  // Up to 25 additional clipping planes in object coordinates.
  // The convex region that is the intersection of the positive 
  // side of these planes is the active region.
  int m_clip_plane_count; // (0 <= m_clip_plane_count <= max_clip_plane_count)

private:
  // The "float" should be a double, but that can't happen
  // until V6 because it will brake the SDK.  Use the
  // SetClipPlaneTolerance() and ClipPlaneTolerance() 
  // functions to set and get this value.
  float m_clip_plane_tolerance;

public:
  ON_PlaneEquation m_clip_plane[max_clip_plane_count];

  /*
  Description:
    The "view frustum" is the frustum the m_xform transformation
    maps to clipping coordinate box (-1,+1)^3.  These functions
    determine if some portion of the convex hull of the test points
    is inside the view frustum.
  Parameters:
    P - [in] point
    box - [in] bounding box
    count - [in] number of points
    p - [in] array of points
    bEnableClippingPlanes - [in]
      If true, then the additional clipping planes are tested.
      If false, then the additional clipping planes are ignored.
  Returns:
    0 = No part of the of the convex hull of the tested points
        is in the view frustum.
    1 = A portion of the convex hull of the otested points may
        be in the view frustum.
    2 = The entire convex hull of the tested points is in the
        view frustum.
  */
  int InViewFrustum( 
    ON_3dPoint P
    ) const;
  int InViewFrustum( 
    const ON_BoundingBox& bbox
    ) const;
  int InViewFrustum( 
    int count, 
    const ON_3fPoint* p
    ) const;
  int InViewFrustum( 
    int count, 
    const ON_3dPoint* p
    ) const;
  int InViewFrustum( 
    int count, 
    const ON_4dPoint* p
    ) const;

  /*
  Description:
    The "clip plane region" is the convex hull of the planes in
    the m_clip_plane[] array.  These functions determine if
    some portion of the convex hull of the test points is inside
    the clip plane region.
  Parameters:
    P - [in] point
    box - [in] bounding box
    count - [in] number of points
    p - [in] array of points
    bEnableClippingPlanes - [in]
      If true, then the additional clipping planes are tested.
      If false, then the additional clipping planes are ignored.
  Returns:
    0 = No part of the of the convex hull of the tested points
        is in the clip plane region.
    1 = A portion of the convex hull of the tested points may
        be in the clip plane region.
    2 = The entire convex hull of the tested points is in the
        clip plane region.
  */
  int InClipPlaneRegion( 
    ON_3dPoint P
    ) const;
  int InClipPlaneRegion( 
    const ON_BoundingBox& bbox
    ) const;
  int InClipPlaneRegion( 
    int count, 
    const ON_3fPoint* p
    ) const;
  int InClipPlaneRegion( 
    int count, 
    const ON_3dPoint* p
    ) const;
  int InClipPlaneRegion( 
    int count, 
    const ON_4dPoint* p
    ) const;


  /*
  Description:
    The "visible area" is the intersection of the view frustum,
    defined by m_xform, and the clipping region, defined by the
    m_clip_plane[] array.  These functions determing if some
    portion of the convex hull of the test points is visible.
  Parameters:
    P - [in] point
    box - [in] bounding box
    count - [in] number of points
    p - [in] array of points
  Returns:
    0 = no part of the object is in the region.
    1 = a portion of the object is in the region
    2 = entire object is in clipping region
  */
  int IsVisible( 
    ON_3dPoint P
    ) const;
  int IsVisible( 
    const ON_BoundingBox& bbox
    ) const;
  int IsVisible( 
    int count, 
    const ON_3fPoint* p
    ) const;
  int IsVisible( 
    int count, 
    const ON_3dPoint* p
    ) const;
  int IsVisible( 
    int count, 
    const ON_4dPoint* p
    ) const;

  /*
  Description:
    Transform a list of 4d homogenous points while testing
    for visibility.
  Parameters:
    count - [in] number of points
    p - [in/out] array of points to test and transform
          If 0 is returned, some of the points may not
          be transformed.  In all other cases, the output
          points are transformed by m_xform.
    pflags - [out]
          0 when the point is in the visible region.  
          Otherwise the bits are set to indicate which planes clip the
          intput point.
          0x01 left of the view frusturm
          0x02 right of the view frustum
          0x04 below the view frustum
          0x08 above the view frustum
          0x10 behind the view frustum (too far)
          0x20 in front of the view frustum (too near)

          0x10 below m_clip_plane[0]
          0x20 below m_clip_plane[1]
          ...
          0x40000000 below m_clip_plane[24]

          0x80000000 transformation created a non-positive weight
  Returns:
    0 = convex hull of the points is not in the region.
        The m_cull_bits field reports which plane or planes
        culled the point set.
    1 = a portion of the convex hull is in the region.
        The m_cull_bits field reports which plane or planes
        culled the point set.
    2 = all points are in the region.
        The m_cull_bits field will be zero.
  */
  int TransformPoints( int count, ON_4dPoint* p ) const;
  int TransformPoints( int count, ON_4dPoint* p, unsigned int* pflags ) const;


  /*
  Description:
    Transform a pont and return the clipping information.
  Parameters:
    P - [in] point ot transform
    Q - [out] transformed point
  Returns:
    0 when the point is in the visible region.  
    Otherwise the bits are set to indicate which planes clip the
    intput point.
    0x01 left of the view frusturm
    0x02 right of the view frustum
    0x04 below the view frustum
    0x08 above the view frustum
    0x10 behind the view frustum (too far)
    0x20 in front of the view frustum (too near)

    0x10 below m_clip_plane[0]
    0x20 below m_clip_plane[1]
    ...
    0x40000000 below m_clip_plane[24]

    0x80000000 transformation created a non-positive weight
  */
  unsigned int TransformPoint(
                     const ON_4dPoint& P, 
                     ON_4dPoint& Q
                     ) const;
  unsigned int TransformPoint(
                     const ON_3dPoint& P, 
                     ON_3dPoint& Q
                     ) const;
  unsigned int TransformPoint(
                     const ON_3fPoint& P, 
                     ON_3dPoint& Q
                     ) const;

  /*
  Description:
    Calculate the interval for the segment of a line that
    is in the clip plane region.
  Parameters:
    P0 - [in] start point
    P1 - [in] end point
    t0 - [out] start parameter
    t1 - [out] end parameter
  Returns:
    True if some portion of the line is visible and
    0.0 <= *t0 <= *t1 <= 1.0.
  */
  bool GetLineClipPlaneParamters( 
         ON_4dPoint P0, 
         ON_4dPoint P1, 
         double* t0, 
         double* t1 
         ) const;

};


class ON_CLASS ON_Localizer
{
public:
  ON_Localizer();
  ~ON_Localizer();

  ON_Localizer(const ON_Localizer&);
  ON_Localizer& operator=(const ON_Localizer&);

  void Destroy();
  bool Read(ON_BinaryArchive&);
  bool Write(ON_BinaryArchive&) const;

  /*
  Descrption:
    Creates a cylindrical localizer.
    If d = distance from the point to the line, 
    then the localizer has the following behavior:

    point distance                localizer value
    d <= r0 < r1 or d >= r0 > r1      0
    d >= r1 > r0 or d <= r1 < r0      1

    For values of d between r0 and r1, the localizer
    smoothly transitions between 0 to 1.

  Parameters:
    P - [in] cylinder axis point
    D - [in] cylinder axis direction
    r0 - [in]
    r1 - [in]
      r0 and r1 are radii that control where the localizer is nonzero.  
      Both r0 and r1 must be postive and the cannot be equal.  
      If 0 < r0 < r1, then the localizer is zero for points 
      inside the cylinder of radius r0 and one for points outside
      the cylinder of radius r1.
      If 0 < r1 < r0, then the localizer is one for points 
      inside the cylinder of radius r1 and zero for points outside
      the cylinder of radius r0.      

  Returns:
    True if the input is value and the localizer is initialized.
  */
  bool CreateCylinderLocalizer( ON_3dPoint P, ON_3dVector D, double r0, double r1 );

  /*
  Descrption:
    Creates a planar localizer.
    If d = signed distance from the point to the plane,
    then the localizer has the following behavior:

    point distance                localizer value
    d <= h0 < h1 or d >= h0 > h1      0
    d >= h1 > h0 or d <= h1 < h0      1

    For values of d between h0 and h1, the localizer
    smoothly transitions between 0 to 1.

  Parameters:
    P - [in] point on plane
    N - [in] normal to plane
    h0 - [in]
    h1 - [in]
      h0 and h1 are signed distances that control where the 
      localizer is nonzero.

  Returns:
    True if the input is value and the localizer is initialized.
  */
  bool CreatePlaneLocalizer( ON_3dPoint P, ON_3dVector N, double h0, double h1 );

  /*
  Descrption:
    Creates a spherical localizer.
    If d = distance from the point to the center of the sphere, 
    then the localizer has the following behavior:

    point distance                localizer value
    d <= r0 < r1 or d >= r0 > r1      0
    d >= r1 > r0 or d <= r1 < r0      1

    For values of d between r0 and r1, the localizer
    smoothly transitions between 0 to 1.

  Parameters:
    P - [in] center of sphere
    r0 - [in]
    r1 - [in]
      r0 and r1 are radii that control where the localizer is nonzero.  
      Both r0 and r1 must be postive and the cannot be equal.  
      If 0 < r0 < r1, then the localizer is zero for points 
      inside the cylinder of radius r0 and one for points outside
      the cylinder of radius r1.
      If 0 < r1 < r0, then the localizer is one for points 
      inside the cylinder of radius r1 and zero for points outside
      the cylinder of radius r0.      

  Returns:
    True if the input is value and the localizer is initialized.
  */
  bool CreateSphereLocalizer( ON_3dPoint P, double r0, double r1 );

  /*
  Description:
    Evaluators.
  Parameters:
    P - [in]
      Evaluation point
    distance - [in]
      Evaluation distance
  Returns:
    Value of the localizer.
  */
  double Value(ON_3dPoint P) const;
  double Value(double distance) const;

  /*
  Parameters:
    bbox - [in]
  Returns:
    True if localizer is identically zero inside bbox.
  */
  bool IsZero( const ON_BoundingBox& bbox ) const;

  enum TYPE
  {
    no_type       = 0,
    sphere_type   = 1,
    plane_type    = 2,
    cylinder_type = 3,
    curve_type    = 4,
    surface_type  = 5,
    distance_type = 6,
    force_32bit_localizer_type = 0xFFFFFFFF
  };

  TYPE m_type;

  ON_Interval      m_d;
  ON_3dPoint       m_P;
  ON_3dVector      m_V;
  class ON_NurbsCurve*   m_nurbs_curve;
  class ON_NurbsSurface* m_nurbs_surface;
};


class ON_CLASS ON_SpaceMorph
{
public:
  ON_SpaceMorph();
  virtual ~ON_SpaceMorph();

  /*
  Description:
    Provides a quick way to determine if a morph function
    is the identity (doesn't move the points) on a region
    of space.
  Parameters:
    bbox - [in] region of space to test.
  Returns:
    The default always returns false.  If you override
    this function, then return true when every point
    in the bounding box is fixed by the morph.  
  */
  virtual
  bool IsIdentity( const ON_BoundingBox& bbox ) const;

  /*
  Description:
    Returns the desired accuracy of the morph.
    This value is primarily used for deforming
    surfaces and breps.
  Returns:
    3d fitting tolerance.
  Remarks:
    The default is 0.0 and any value <= 0.0 is 
    ignored by morphing functions.
    The value returned by Tolerance() does not
    affect the way meshes and points are morphed.
  */
  double Tolerance() const;

  /*
  Description:
    Set the 3d fitting tolerance used when morphing
    surfaces and breps.
  Parameters:
    tolerance - [in] values < 0.0 are treated as 0.0.                     
  */
  void SetTolerance(
          double tolerance
          );

  /*
  Returns:
    True if the morph should be done as quickly as
    possible because the result is being used for
    some type of dynamic preview.  If QuickPreview
    is true, the tolerance may be ignored.
  Remarks:
    The value returned by QuickPreview() does not
    affect the way meshes and points are morphed.
    The default is false.
  */
  bool QuickPreview() const;

  /*
  Description:
    Set the quick preview value.
  Parameters:
    bQuickPreview - [in]
  */
  void SetQuickPreview( 
          bool bQuickPreview 
          );

  /*
  Returns:
    True if the morph should be done in a way that
    preserves the structure of the geometry.  
    In particular, for NURBS objects, true
    means that only the control points are moved.
  Remarks:
    The value returned by PreserveStructure() does not
    affect the way meshes and points are morphed.
    The default is false.
  */
  bool PreserveStructure() const;

  /*
  Description:
    Set the preserve structure value.
  Parameters:
    bPreserveStructure - [in]
  */
  void SetPreserveStructure( 
          bool bPreserveStructure
          );

private:
  double m_tolerance;
  bool m_bQuickPreview;
  bool m_bPreserveStructure;
};

#if defined(ON_DLL_TEMPLATE)

// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_Xform>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Localizer>;
#pragma warning( pop )
#endif

#endif

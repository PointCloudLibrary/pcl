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
//   defines ON_Viewport
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_VIEWPORT_INC_)
#define OPENNURBS_VIEWPORT_INC_

///////////////////////////////////////////////////////////////////////////////
// Class  ON_Viewport
//
//	This object represents a viewing frustum
///////////////////////////////////////////////////////////////////////////////
class ON_CLASS ON_Viewport : public ON_Geometry 
{
	ON_OBJECT_DECLARE( ON_Viewport );
public:

  // Default z=up perspective camera direction
  static const ON_3dVector Default3dCameraDirection;

  // Construction
	ON_Viewport();
  ~ON_Viewport();
	ON_Viewport& operator=( const ON_Viewport& );

  bool IsValidCamera() const;
  bool IsValidFrustum() const;

  // ON_Object overrides //////////////////////////////////////////////////////
  //

  /*
  Description:
    Test for a valid camera, frustum, and screen port.
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
    true     camera, frustum, and screen port are valid.
    false    camera, frustum, or screen port is invalid.
  Remarks:
    Overrides virtual ON_Object::IsValid
  See Also:
    IsValidCamera, IsValidFrustum
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // Description:
  //   Dumps debugging text description to a text log.
  //
  // Parameters:
  //   dump_target - [in] text log
  //
  // Remarks:
  //   This overrides the virtual ON_Object::Dump() function.
  void Dump( 
    ON_TextLog& // dump_target
    ) const;

  // Description:
  //   Writes ON_Viewport defintion from a binary archive.
  //
  // Parameters:
  //   binary_archive - [in] open binary archive
  //
  // Returns:
  //   true if successful.
  //
  // Remarks:
  //   This overrides the virtual ON_Object::Write() function.
  ON_BOOL32 Write(
         ON_BinaryArchive&  // binary_archive
       ) const;


  // Description:
  //   Reads ON_Viewport defintion from a binary archive.
  //
  // Parameters:
  //   binary_archive - [in] open binary archive
  //
  // Returns:
  //   true if successful.
  //
  // Remarks:
  //   This overrides the virtual ON_Object::Read() function.
  ON_BOOL32 Read(
         ON_BinaryArchive&  // binary_archive
       );


  // ON_Geometry overrides //////////////////////////////////////////////////////
  //

  // Description:
  //   The dimension of a camera view frustum is 3.
  //
  // Returns:
  //   3
  //
  // Remarks:
  //   This is virtual ON_Geometry function.
  int Dimension() const;

  // Description:
  //   Gets bounding box of viewing frustum.
  //
  // Parameters:
  //   boxmin - [in/out] array of Dimension() doubles
  //   boxmax - [in/out] array of Dimension() doubles
  //   bGrowBox - [in] (default=false) 
  //     If true, then the union of the input bbox and the 
  //     object's bounding box is returned in bbox.  
  //     If false, the object's bounding box is returned in bbox.
  //
  // Returns:
  //   @untitled table
  //   true     Valid frustum and bounding box returned.
  //   false    Invalid camera or frustum. No bounding box returned.
  //
  // Remarks:
  //   This overrides the virtual ON_Geometry::GetBBox() function.
  ON_BOOL32 GetBBox( // returns true if successful
         double*, // boxmin
         double*, // boxmax
         ON_BOOL32 = false // bGrowBox
         ) const;

  // Description:
  //   Transforms the view camera location, direction, and up.
  //
  // Parameters:
  //   xform - [in] transformation to apply to camera.
  //
  // Returns:
  //   @untitled table
  //   true     Valid camera was transformed.
  //   false    Invalid camera, frustum, or transformation.
  //
  // Remarks:
  //   This overrides the virtual ON_Geometry::Transform() function.
  ON_BOOL32 Transform( 
         const ON_Xform& // xform
         );

  // Interface /////////////////////////////////////////////////////////////////
  //
  void Initialize();

  ON::view_projection Projection() const;

  /*
  Description:
    Unconditionally set the projection.
  Parameters:
    projection - [in]
  See Also:
    ON_Viewport::SetParallelProjection
    ON_Viewport::SetPerpectiveProjection
    ON_Viewport::SetTwoPointPerspectiveProjection    
  */
  bool SetProjection( ON::view_projection projection );

  /*
  Description:
    Use this function to change projections of valid viewports
    from persective to parallel.  It will make common additional
    adjustments to the frustum so the resulting views are similar.
    The camera location and direction will not be changed.
  Parameters:
    bSymmetricFrustum - [in]
      True if you want the resulting frustum to be symmetric.
  Remarks:
    If the current projection is parallel and bSymmetricFrustum,
    FrustumIsLeftRightSymmetric() and FrustumIsTopBottomSymmetric()
    are all equal, then no changes are made and true is returned.
  */
  bool ChangeToParallelProjection( bool bSymmetricFrustum );

  /*
  Description:
    Use this function to change projections of valid viewports
    from parallel to perspective.  It will make common additional
    adjustments to the frustum and camera location so the resulting
    views are similar.  The camera direction and target point are
    not be changed.
  Parameters:
    target_distance - [in]
      If ON_UNSET_VALUE this parameter is ignored.  Otherwise
      it must be > 0 and indicates which plane in the current 
      view frustum should be perserved.
    bSymmetricFrustum - [in]
      True if you want the resulting frustum to be symmetric.
    lens_length - [in] (pass 50.0 when in doubt)
      35 mm lens length to use when changing from parallel
      to perspective projections. If the current projection
      is perspective or lens_length is <= 0.0,
      then this parameter is ignored.
  Remarks:
    If the current projection is perspective and bSymmetricFrustum,
    FrustumIsLeftRightSymmetric() and FrustumIsTopBottomSymmetric()
    are all equal, then no changes are made and true is returned.
  */
  bool ChangeToPerspectiveProjection( 
          double target_distance,
          bool bSymmetricFrustum,
          double lens_length
          );

  /*
  Description:
    Use this function to change projections of valid viewports
    to a two point perspective.  It will make common additional
    adjustments to the frustum and camera location and direction
    so the resulting views are similar.
  Parameters:
    target_distance - [in]
      If ON_UNSET_VALUE this parameter is ignored.  Otherwise
      it must be > 0 and indicates which plane in the current 
      view frustum should be perserved.
    up - [in]
      This direction will be the locked up direction.  Pass 
      ON_3dVector::ZeroVector if you want to use the world axis
      direction that is closest to the current up direction.
      Pass CameraY() if you want to preserve the current up direction.
    lens_length - [in] (pass 50.0 when in doubt)
      35 mm lens length to use when changing from parallel
      to perspective projections. If the current projection
      is perspective or lens_length is <= 0.0,
      then this parameter is ignored.
  Remarks:
    If the current projection is perspective and 
    FrustumIsLeftRightSymmetric() is true and
    FrustumIsTopBottomSymmetric() is false, then no changes are
    made and true is returned.
  */
  bool ChangeToTwoPointPerspectiveProjection( 
          double target_distance,
          ON_3dVector up,
          double lens_length
          );

  /*
  Returns:
    True if the projection is ON::perspective_view.
  */
  bool IsPerspectiveProjection() const;

  /*
  Returns
    IsPerspectiveProjection() 
    && CameraUpIsLocked()
    && FrustumIsLeftRightSymmetric
    && !FrustumIsTopBottomSymmetric
  */
  bool IsTwoPointPerspectiveProjection() const;

  /*
  Returns:
    True if the projection is ON::parallel_view.
  */
  bool IsParallelProjection() const;

  // These return true if the current direction and up are not zero and not
  // parallel so the camera position is well defined.
  bool SetCameraLocation( const ON_3dPoint& );
  bool SetCameraDirection( const ON_3dVector& );
  bool SetCameraUp( const ON_3dVector& );

  ON_3dPoint CameraLocation() const;
  ON_3dVector CameraDirection() const;
  ON_3dVector CameraUp() const;

  bool CameraLocationIsLocked() const;
  bool CameraDirectionIsLocked() const;
  bool CameraUpIsLocked() const;
  bool FrustumIsLeftRightSymmetric() const;
  bool FrustumIsTopBottomSymmetric() const;

  void SetCameraLocationLock( bool bLockCameraLocation );
  void SetCameraDirectionLock( bool bLockCameraDirection ) ;
  void SetCameraUpLock( bool bLockCameraUp );
  void SetFrustumLeftRightSymmetry( bool bForceLeftRightSymmetry );
  void SetFrustumTopBottomSymmetry( bool bForceTopBottomSymmetry );
  void UnlockCamera(); // sets all camera locks to false
  void UnlockFrustumSymmetry(); // sets all frustum symmetry locks to false

  // returns true if current camera orientation is valid
  bool GetCameraFrame(
      double*, // CameraLocation[3]
      double*, // CameraX[3]
      double*, // CameraY[3]
      double*  // CameraZ[3]
      ) const;

  // these do not check for a valid camera orientation
  ON_3dVector CameraX() const; // unit to right vector
  ON_3dVector CameraY() const; // unit up vector
  ON_3dVector CameraZ() const; // unit vector in -CameraDirection

  
  bool IsCameraFrameWorldPlan( 
      // Returns true if the camera direction = some world axis.
      // The indices report which axes are used.  For a "twisted"
      // plan view it is possible to have zero x and y indices.
      // This function returns true if and only if the "z" index
      // is non-zero.
      //
      // Indices are +/-1 = world +/-x, +/-2 = world +/-y, +/-3 = world +/-z,
      int*, // if true and plan is axis aligned, view x index, else 0
      int*, // if true and plan is axis aligned, view y index, else 0
      int*  // if true, view z index, else 0
      );

  bool GetCameraExtents( 
      // returns bounding box in camera coordinates - this is useful information
      // for setting view frustrums to include the point list
      int,           // count = number of 3d points
      int,           // stride = number of doubles to skip between points (>=3)
      const double*, // 3d points in world coordinates
      ON_BoundingBox& cambbox, // bounding box in camera coordinates
      int bGrowBox = false   // set to true if you want to enlarge an existing camera coordinate box
      ) const;

  bool GetCameraExtents( 
      // returns bounding box in camera coordinates - this is useful information
      // for setting view frustrums to include the point list
      const ON_BoundingBox&, // world coordinate bounding box
      ON_BoundingBox& cambbox, // bounding box in camera coordinates
      int bGrowBox = false   // set to true if you want to enlarge an existing camera coordinate box
      ) const;

  bool GetCameraExtents( 
      // returns bounding box in camera coordinates - this is useful information
      // for setting view frustrums to include the point list
      ON_3dPoint&,     // world coordinate bounding sphere center
      double,          // world coordinate bounding sphere radius
      ON_BoundingBox& cambox, // bounding box in camera coordinates
      int bGrowBox = false     // set to true if you want to enlarge an existing camera coordinate box
      ) const;

  /*
  Description:
    Set the view frustum.  If FrustumSymmetryIsLocked() is true
    and left != -right or bottom != -top, then they will be
    adjusted so the resulting frustum is symmetric.
  */
  bool SetFrustum(
        double left,   // 
        double right,  //   ( left < right )
        double bottom, // 
        double top,    //   ( bottom < top )
        double near_dist,   // 
        double far_dist     //   ( 0 < near_dist < far_dist ) // ignored by Rhino version 1.0
        );
  bool GetFrustum(
        double* left,        // 
        double* right,       // (left < right)
        double* bottom,      // 
        double* top,         // (bottom < top)
        double* near_dist = NULL, // 
        double* far_dist = NULL   // (0 < near_dist < far_dist)
        ) const;

  // SetFrustumAspect() changes the larger of the frustum's widht/height
  // so that the resulting value of width/height matches the requested
  // aspect.  The camera angle is not changed.  If you change the shape
  // of the view port with a call SetScreenPort(), then you generally 
  // want to call SetFrustumAspect() with the value returned by 
  // GetScreenPortAspect().
  bool SetFrustumAspect( double );

  // Returns frustum's width/height
  bool GetFrustumAspect( double& ) const;

  // Returns world coordinates of frustum's center
  bool GetFrustumCenter( double* ) const;

  // The near clipping plane stored in the Rhino 1.0 file is frequently very
  // small and useless for high quality z-buffer based rendering.  The far
  // clipping value is not stored in the file.  Use these functions to set
  // the frustum's near and far clipping planes to appropriate values.
  double FrustumLeft() const;
  double FrustumRight() const;
  double FrustumBottom() const;
  double FrustumTop() const;
  double FrustumNear() const;
  double FrustumFar() const;

  /*
  Returns:
    frustum right - frustum left
  */
  double FrustumWidth() const; // right - left

  /*
  Returns:
    frustum right - frustum left
  */
  double FrustumHeight() const; // top - bottom

  /*
  Returns:
    Minimum of fabs(FrustumWidth()) and fabs(FrustumHeight())
  */
  double FrustumMinimumDiameter() const;

  /*
  Returns:
    Maximum of fabs(FrustumWidth()) and fabs(FrustumHeight())
  */
  double FrustumMaximumDiameter() const;


  bool SetFrustumNearFar(       
         const double* bboxmin,  // 3d bounding box min
         const double* bboxmax   // 3d bounding box max
         );
  bool SetFrustumNearFar( 
         const double* center,  // 3d bounding sphere center
         double radius         // 3d bounding sphere radius
         );
  bool SetFrustumNearFar( 
         double near_dist, // ( > 0 )
         double far_dist   // 
         );

  /*
  Description:
    If needed, adjust the current frustum so it has the 
    specified symmetries and adjust the camera location
    so the target plane remains visible.
  Parameters:
    bLeftRightSymmetric - [in]
      If true, the frustum will be adjusted so left = -right.
    bTopBottomSymmetric - [in]
      If true, the frustum will be adjusted so top = -bottom.
    target_distance - [in]
      If projection is not perspective or target_distance 
      is ON_UNSET_VALUE, this this parameter is ignored. 
      If the projection is perspective and target_distance 
      is not ON_UNSET_VALUE, then it must be > 0.0 and
      it is used to determine which plane in the old
      frustum will appear unchanged in the new frustum.
  bool
    Returns true if the returned viewport has a frustum
    with the specified symmetries.
  */
  bool ChangeToSymmetricFrustum( 
    bool bLeftRightSymmetric, 
    bool bTopBottomSymmetric,
    double target_distance
    );

  /*
  Description:
    Get near and far clipping distances of a point
  Parameters:
    point - [in] 
    near_dist - [out] 
      near distance of the point (can be < 0)
    far_dist - [out] 
      far distance of the point (can be equal to near_dist)
    bGrowNearFar - [in]
      If true and input values of near_dist and far_dist
      are not ON_UNSET_VALUE, the near_dist and far_dist
      are enlarged to include bbox.
  Returns:
    True if the point is ing the view frustum and
    near_dist/far_dist were set.
    False if the bounding box does not intesect the
    view frustum.
  */
  bool GetPointDepth(       
         ON_3dPoint point,
         double* near_dist,
         double* far_dist,
         bool bGrowNearFar=false
         ) const;

  /*
  Description:
    Get the view plane depth of a point
  Parameters:
    point - [in] 
    view_plane_depth - [out] 
      positive values are in front of the camera and negative
      values are behind the camera.
      If 0 <= point_depth < FrustumNear(), the point's view
      plane is between the camera and the frustum's near plane.
      If point_depth > FrustumFar(), the point's view
      plane is farther from the camera and the frustum's far plane.
  Returns:
    True if the point is ing the view frustum and
    near_dist/far_dist were set.
    False if the bounding box does not intesect the
    view frustum.
  */
  bool GetPointDepth(       
       ON_3dPoint point,
       double* view_plane_depth
       ) const;

  /*
  Description:
    Get near and far clipping distances of a bounding box.
  Parameters:
    bbox - [in] 
      bounding box
    near_dist - [out] 
      near distance of the box
      This value can be zero or negative when the camera
      location is inside bbox.
    far_dist - [out] 
      far distance of the box
      This value can be equal to near_dist, zero or negative 
      when the camera location is in front of the bounding box.
    bGrowNearFar - [in]
      If true and input values of near_dist and far_dist
      are not ON_UNSET_VALUE, the near_dist and far_dist
      are enlarged to include bbox.
  Returns:
    True if the bounding box intersects the view frustum and
    near_dist/far_dist were set.
    False if the bounding box does not intesect the view frustum.
  Remarks:
    This function ignores the current value of the viewport's 
    near and far settings. If the viewport is a perspective
    projection, the it intersects the semi infinite frustum
    volume with the bounding box and returns the near and far
    distances of the intersection.  If the viewport is a parallel
    projection, it instersects the infinte view region with the
    bounding box and returns the near and far distances of the
    projection.
  */
  bool GetBoundingBoxDepth(       
         ON_BoundingBox bbox,
         double* near_dist,
         double* far_dist,
         bool bGrowNearFar=false
         ) const;

  /*
  Description:
    Get near and far clipping distances of a bounding sphere.
  Parameters:
    sphere - [in] 
      bounding sphere
    near_dist - [out] 
      near distance of the sphere (can be < 0)
    far_dist - [out] 
      far distance of the sphere (can be equal to near_dist)
    bGrowNearFar - [in]
      If true and input values of near_dist and far_dist
      are not ON_UNSET_VALUE, the near_dist and far_dist
      are enlarged to include bbox.
  Returns:
    True if the sphere intersects the view frustum and
    near_dist/far_dist were set.
    False if the sphere does not intesect the view frustum.
  */
  bool GetSphereDepth( 
         ON_Sphere sphere,
         double* near_dist,
         double* far_dist,
         bool bGrowNearFar=false
         ) const;

  /*
  Description:
    Set near and far clipping distance subject to constraints.
  Parameters:
    near_dist - [in] (>0) desired near clipping distance
    far_dist - [in] (>near_dist) desired near clipping distance
    min_near_dist - [in] 
      If min_near_dist <= 0.0, it is ignored.
      If min_near_dist > 0 and near_dist < min_near_dist, 
      then the frustum's near_dist will be increased to 
      min_near_dist.
    min_near_over_far - [in] 
      If min_near_over_far <= 0.0, it is ignored.
      If near_dist < far_dist*min_near_over_far, then
      near_dist is increased and/or far_dist is decreased
      so that near_dist = far_dist*min_near_over_far.
      If near_dist < target_dist < far_dist, then near_dist
      near_dist is increased and far_dist is decreased so that
      projection precision will be good at target_dist.
      Otherwise, near_dist is simply set to 
      far_dist*min_near_over_far.
    target_dist - [in]  
      If target_dist <= 0.0, it is ignored.
      If target_dist > 0, it is used as described in the
      description of the min_near_over_far parameter.
    relative_depth_bias - [in]
      If relative_depth_bias <= 0.0, it is ignored.
      If relative_depth_bias > 0, it is assumed that
      the requested near_dist and far_dist were calculated
      assuming no depth bias and the values will be
      appropriately adjusted to ensure the frustum's 
      near and far clipping planes will not clip biased
      objects.
  */
  bool SetFrustumNearFar( 
         double near_dist,
         double far_dist,
         double min_near_dist,
         double min_near_over_far,
         double target_dist
         );

  bool SetFrustumNearFar( 
         double near_dist,
         double far_dist,
         double min_near_dist,
         double min_near_over_far,
         double target_dist,
         double relative_depth_bias
         );

  // Description:
  //   Get near clipping plane.
  //
  //  near_plane - [out] near clipping plane if camera and frustum
  //      are valid.  The plane's frame is the same as the camera's
  //      frame.  The origin is located at the intersection of the
  //      camera direction ray and the near clipping plane. The plane's
  //      normal points out of the frustum towards the camera
  //      location.
  //
  // Returns:
  //   true if camera and frustum are valid.
  bool GetNearPlane( 
    ON_Plane& near_plane 
    ) const;

  bool GetNearPlaneEquation( 
    ON_PlaneEquation& near_plane_equation 
    ) const;

  // Description:
  //   Get far clipping plane.
  //
  //  far_plane - [out] far clipping plane if camera and frustum
  //      are valid.  The plane's frame is the same as the camera's
  //      frame.  The origin is located at the intersection of the
  //      camera direction ray and the far clipping plane. The plane's
  //      normal points into the frustum towards the camera location.
  //
  // Returns:
  //   true if camera and frustum are valid.
  bool GetFarPlane( 
    ON_Plane& far_plane 
    ) const;

  bool GetFarPlaneEquation( 
    ON_PlaneEquation& far_plane_equation 
    ) const;

  /*
  Description:
    Get the plane that is a specified distance from the camera.
    This plane is parallel to the frustum's near and far planes.
  Parameters:
    view_plane_depth - [in]
      The distance from the camera location to the view plane. 
      Positive distances are in front of the camera and
      negative distances are behind the camera.
      A value of FrustumNear() will return the frustum's
      near plane and a valud of FrustumFar() will return
      the frustum's far plane.
    view_plane - [out]
      View plane
    view_plane_equation - [out]
      Equation of the view plane.
  Returns:
    True if the camera and frustum are valid and view_plane
    was calculated.  False otherwise.
  */
  bool GetViewPlane( 
    double view_plane_depth,
    ON_Plane& view_plane 
    ) const;

  bool GetViewPlaneEquation( 
    double view_plane_depth,
    ON_PlaneEquation& view_plane_equation 
    ) const;

  /*
  Description:
  Get left world frustum clipping plane.
  Parameters:
    left_plane - [out] 
      frustum left side clipping plane.  The normal points
      into the visible region of the frustum.  If the projection
      is perspective, the origin is at the camera location,
      otherwise the origin isthe point on the plane that is
      closest to the camera location.
  Returns:
    True if camera and frustum are valid and plane was set.
  */
  bool GetFrustumLeftPlane( 
    ON_Plane& left_plane 
    ) const;

  bool GetFrustumLeftPlaneEquation( 
    ON_PlaneEquation& left_plane_equation 
    ) const;

  /*
  Description:
  Get right world frustum clipping plane.
  Parameters:
    right_plane - [out] 
      frustum right side clipping plane.  The normal points
      into the visible region of the frustum.  If the projection
      is perspective, the origin is at the camera location,
      otherwise the origin isthe point on the plane that is
      closest to the camera location.
  Returns:
    True if camera and frustum are valid and plane was set.
  */
  bool GetFrustumRightPlane( 
    ON_Plane& right_plane 
    ) const;

  bool GetFrustumRightPlaneEquation( 
    ON_PlaneEquation& right_plane_equation 
    ) const;

  /*
  Description:
  Get right world frustum clipping plane.
  Parameters:
    right_plane - [out] 
      frustum bottom side clipping plane.  The normal points
      into the visible region of the frustum.  If the projection
      is perspective, the origin is at the camera location,
      otherwise the origin isthe point on the plane that is
      closest to the camera location.
  Returns:
    True if camera and frustum are valid and plane was set.
  */
  bool GetFrustumBottomPlane( 
    ON_Plane& bottom_plane 
    ) const;

  bool GetFrustumBottomPlaneEquation( 
    ON_PlaneEquation& bottom_plane_equation 
    ) const;
  /*
  Description:
  Get right world frustum clipping plane.
  Parameters:
    top_plane - [out] 
      frustum top side clipping plane.  The normal points
      into the visible region of the frustum.  If the projection
      is perspective, the origin is at the camera location,
      otherwise the origin isthe point on the plane that is
      closest to the camera location.
  Returns:
    True if camera and frustum are valid and plane was set.
  */
  bool GetFrustumTopPlane( 
    ON_Plane& top_plane 
    ) const;

  bool GetFrustumTopPlaneEquation( 
    ON_PlaneEquation& top_plane_equation 
    ) const;

  // Description:
  //   Get corners of near clipping plane rectangle.
  //
  // Parameters:
  //   left_bottom - [out] 
  //   right_bottom - [out]
  //   left_top - [out]
  //   right_top - [out]
  //
  // Returns:
  //   true if camera and frustum are valid.
  bool GetNearRect( 
          ON_3dPoint& left_bottom,
          ON_3dPoint& right_bottom,
          ON_3dPoint& left_top,
          ON_3dPoint& right_top
          ) const;

  // Description:
  //   Get corners of far clipping plane rectangle.
  //
  // Parameters:
  //   left_bottom - [out] 
  //   right_bottom - [out]
  //   left_top - [out]
  //   right_top - [out]
  //
  // Returns:
  //   true if camera and frustum are valid.
  bool GetFarRect( 
          ON_3dPoint& left_bottom,
          ON_3dPoint& right_bottom,
          ON_3dPoint& left_top,
          ON_3dPoint& right_top
          ) const;

  /*
  Description:
    Get the world coordinate corners of the rectangle of
    a view plane that is a specified distance from the camera.
    This rectangle is parallel to the frustum's near and far planes.
  Parameters:
    view_plane_depth - [in]
      The distance from the camera location to the view plane. 
      Positive distances are in front of the camera and
      negative distances are behind the camera.
      A value of FrustumNear() will return the frustum's
      near rectangle and a valud of FrustumFar() will return
      the frustum's far rectangle.
    left_bottom - [out]
    right_bottom - [out]
    left_top - [out]
    right_top - [out]
  Returns:
    True if the camera and frustum are valid and view_plane
    was calculated.  False otherwise.
  */
  bool GetViewPlaneRect(
          double view_plane_depth,
          ON_3dPoint& left_bottom,
          ON_3dPoint& right_bottom,
          ON_3dPoint& left_top,
          ON_3dPoint& right_top
          ) const;


  /*
  Description:
    Location of viewport in pixels.
    These are provided so you can set the port you are using
    and get the appropriate transformations to and from
    screen space.
  Parameters:
    port_left - [in]
    port_right - [in] (port_left != port_right)
    port_bottom - [in]
    port_top - [in] (port_top != port_bottom)
    port_near - [in]
    port_far - [in]
  Example:

          // For a Windows window
          int width = width of window client area in pixels;
          int height = height of window client area in pixels;
          port_left = 0;
          port_right = width;
          port_top = 0;
          port_bottom = height;
          port_near = 0;
          port_far = 1;
          SetScreenPort( port_left, port_right, 
                         port_bottom, port_top, 
                         port_near, port_far );

  Returns:
    true if input is valid.
  See Also:
    ON_Viewport::GetScreenPort
  */
  bool SetScreenPort(
        int port_left,
        int port_right,
        int port_bottom,
        int port_top,
        int port_near = 0,
        int port_far = 0
        );

  bool GetScreenPort(
        int* left,
        int* right,         //( port_left != port_right )
        int* port_bottom,
        int* port_top,      //( port_bottom != port_top)
        int* port_near=NULL,  
        int* port_far=NULL   
        ) const;

  /* 
  Returns:
    abs(port_right - port_left)
  */
  int ScreenPortWidth() const;

  /* 
  Returns:
    abs(port_bottom - port_top)
  */
  int ScreenPortHeight() const;

  bool GetScreenPortAspect( double& ) const; // port's |width/height|

  bool GetCameraAngle( 
          double* half_diagonal_angle, // 1/2 of diagonal subtended angle
          double* half_vertical_angle, // 1/2 of vertical subtended angle
          double* half_horizontal_angle // 1/2 of horizontal subtended angle
          ) const;
  bool GetCameraAngle( 
          double* half_smallest_angle  // 1/2 of smallest subtended view angle
          ) const;
  bool SetCameraAngle( 
          double half_smallest_angle // 1/2 of smallest subtended view angle
                  // 0 < angle < pi/2
          );

  // These functions assume the camera is horizontal and crop the
  // film rather than the image when the aspect of the frustum
  // is not 36/24.  (35mm film is 36mm wide and 24mm high.)
  //
  // The SetCamera35mmLensLength() preserves camera location,
  // changes the frustum, but maintains the frsutrum's aspect.
  bool GetCamera35mmLensLength( 
    double* lens_length 
    ) const;
  bool SetCamera35mmLensLength( 
    double lens_length 
    );

  // Same as GetCamera35mmLensLength() with "lens" misspelled.
  bool GetCamera35mmLenseLength( 
    double* lens_length 
    ) const;

  // Same as SetCamera35mmLensLength() with "lens" misspelled.
  bool SetCamera35mmLenseLength( 
    double lens_length 
    );

  bool GetXform( 
         ON::coordinate_system srcCS,
         ON::coordinate_system destCS,
         ON_Xform& matrix      // 4x4 transformation matrix (acts on the left)
         ) const;

  /*
  Description:
    Get the world coordinate line in the view frustum
    that projects to a point on the screen.
  Parameters:
    screenx - [in]
    screeny - [in] (screenx,screeny) = screen location
    world_line - [out] 3d world coordinate line segment
           starting on the near clipping plane and ending 
           on the far clipping plane.
  Returns:
    true if successful. 
    false if view projection or frustum is invalid.
  */
  bool GetFrustumLine( 
            double screenx, 
            double screeny, 
            ON_Line& world_line
            ) const;

  // display tools
  bool GetWorldToScreenScale( 
    const ON_3dPoint& point_in_frustum, // [in]  point in viewing frustum.
    double* pixels_per_unit             // [out] scale = number of pixels per world unit at the 3d point
    ) const;

  bool GetCoordinateSprite(
         int,        // size in pixels of coordinate sprite axes
         int, int,   // screen (x,y) for sprite origin
         int[3],     // returns depth order for axes
         double [3][2]  // screen coords for axes ends
         ) const;

  // Use Extents() as a quick way to set a viewport to so that bounding
  // volume is inside of a viewports frusmtrum.
  // The view angle is used to determine the position of the camera.
  bool Extents( 
         double half_view_angle,        // 1/2 smallest subtended view angle
                        // (0 < angle < pi/2)
         const ON_BoundingBox& world_bbox// 3d world coordinate bounding box
         );
  bool Extents( 
         double half_view_angle,        // 1/2 smallest subtended view angle
                        // (0 < angle < pi/2)
         const ON_3dPoint& center, // 3d world coordinate bounding sphere center
         double radius        // 3d sphere radius
         );

  ////////////////////////////////////////////////////////////////////////
  // View changing from screen input points.  Handy for
  // using a mouse to manipulate a view.
  //

  //////////
  // ZoomToScreenRect() may change camera and frustum settings
  bool ZoomToScreenRect(
         int screen_x0, 
         int screen_y0,  // (x,y) screen coords of a rectangle corner
         int screen_x1, 
         int screen_y1   // (x,y) screen coords of opposite rectangle corner
         );

  //////////
  // DollyCamera() does not update the frustum's clipping planes.
  // To update the frustum's clipping planes call DollyFrustum(d)
  // with d = dollyVector o cameraFrameZ.  To convert screen locations
  // into a dolly vector, use GetDollyCameraVector().
  bool DollyCamera( // Does not update frustum.  To update frustum use 
                    // DollyFrustum(d) with d = dollyVector o cameraFrameZ
          const ON_3dVector& dolly_vector // dolly vector in world coordinates
          );

  //////////
  // Gets a world coordinate dolly vector that can be passed to
  // DollyCamera().
  bool GetDollyCameraVector(
         int screen_x0, 
         int screen_y0,  // (x,y) screen coords of start point
         int screen_x1, 
         int screen_y1,  // (x,y) screen coords of end point
         double proj_plane_dist,      // distance of projection plane from camera.
                      // When in doubt, use 0.5*(frus_near+frus_far).
         ON_3dVector& dolly_vector // world coordinate dolly vector returned here
         ) const;

  //////////
  // Moves frustum's clipping planes
  bool DollyFrustum(
          double dolly_distance // distance to move in camera direction
          );

  /*
  Description:
    Apply scaling factors to parallel projection clipping coordinates
    by setting the m_clip_mod transformation.  
  Parameters:
    x - [in] x > 0
    y - [in] y > 0
  Example:
    If you want to compress the view projection across the viewing
    plane, then set x = 0.5, y = 1.0, and z = 1.0.
  Returns:
    True if successful.
    False if input is invalid or the view is a perspective view.
  */
  bool SetViewScale( double x, double y );
  void GetViewScale( double* x, double* y ) const;

  /*
  Description:
    Gets the m_clip_mod transformation;
  Returns:
    value of the m_clip_mod transformation.
  */
  ON_Xform ClipModXform() const;

  /*
  Description:
    Gets the m_clip_mod_inverse transformation;
  Returns:
    value of the m_clip_mod_inverse transformation.
  */
  ON_Xform ClipModInverseXform() const;

  /*
  Returns:
    True if clip mod xform is identity.
  */
  bool ClipModXformIsIdentity() const;

  /*
  Description:
    Return a point on the central axis of the view frustum.
    This point is a good choice for a general purpose target point.
  Parameters:
    target_distance - [in]
      If target_distance > 0.0, then the distance from the returned
      point to the camera plane will be target_distance. Note that
      if the frustum is not symmetric, the distance from the
      returned point to the camera location will be larger than
      target_distanct.
      If target_distance == ON_UNSET_VALUE and the frustum
      is valid with near > 0.0, then 0.5*(near + far) will be used
      as the target_distance.
  Returns:
    A point on the frustum's central axis.  If the viewport or input
    is not valid, then ON_3dPoint::UnsetPoint is returned.
  */
  ON_3dPoint FrustumCenterPoint( double target_distance ) const;

  /*
  Returns:
    The current value of the target point.  This point does not play
    a role in the view projection calculations.  It can be used as a 
    fixed point when changing the camera so the visible regions of the
    before and after frustums both contain the region of interest.
  Remarks:
    The default constructor sets this point on ON_3dPoint::UnsetPoint.
    You must explicitly call one SetTargetPoint() functions to set
    the target point.
  */
  ON_3dPoint TargetPoint() const;

  /*
  Description:
    Set the target point.
  Parameters:
    target_point - [in]
      When in doubt, the point returned by FrustumCenterPoint(ON_UNSET_VALUE)
      is a good choice.
  Remarks:
    The default constructor sets this point on ON_3dPoint::UnsetPoint.
    You must explicitly call one SetTargetPoint() functions to set
    the target point.
  */
  bool SetTargetPoint( ON_3dPoint target_point );

  /*
  Description:
    Get the distance from the target point to the camera plane.
    Note that if the frustum is not symmetric, then this distance
    is shorter than the distance from the target to the camera location.
  Parameters:
    bUseFrustumCenterFallback - [in]
      If bUseFrustumCenterFallback is false and the target point is
      not valid, then ON_UNSET_VALUE is returned.
      If bUseFrustumCenterFallback is true and the frustum is valid
      and current target point is not valid or is behind the camera,
      then 0.5*(near + far) is returned.
  Returns:
    Shortest signed distance from camera plane to target point.
    If the target point is on the visible side of the camera,
    a positive value is returned.  ON_UNSET_VALUE is returned
    when the input of view is not valid.
  */
  double TargetDistance( bool bUseFrustumCenterFallback ) const;

  /*
  Description:    
    Get suggested values for setting the perspective minimum
    near distance and minimum near/far ratio.
  Parameters:      
    camera_location - [in]
    depth_buffer_bit_depth - [in]
      typically 32, 24, 16 or 8, but any positive value can be 
      passed in.
    min_near_dist - [out]
      Suggest value for passing to SetPerspectiveMinNearDist().     
    min_near_over_far - [out]
      Suggest value for passing to SetPerspectiveMinNearOverFar().     
  */
  static void GetPerspectiveClippingPlaneConstraints( 
        ON_3dPoint camera_location,
        unsigned int depth_buffer_bit_depth,
        double* min_near_dist,
        double* min_near_over_far
        );

  /*
  Description:
    Calculate the value to add to homogeneous "z" clipping coordinate
    that corresponds to moving the corresponding euclidean camera
    coordinate by relative_depth_bias*(far - near).
  Parameters:
    relative_depth_bias - [in]
      signed relative bias. 
      = 0: no bias, 
      > 0: bias towards frustum's near clipping plane
      < 0: bias towards frustum's far clipping plane
      When you have curves and points that are "on" shaded objects,
      values around 1/256 work well to move the wire objects
      in front of or behind shaded objects.
    clip_z [-in]
    clip_w [-in]
      clip_z and clip_w are the homogeneous "w" and "w" coordinates
      of a homogeneous clipping coordinate point.
  Returns:
    The clipping coordinate depth bias to add to the z-clipping
    coordinate that corresponds to adding cam_depth_bias
    to the z camera coordinate.
  Remarks:
    For perspective views, this bias is largest in the vicinity
    of the frustum's near clipping plane and smallest in the
    vicinity of the frustum's far clipping plane.
    For orthographic projectsions, this bias is constant.
  */
  double ClipCoordDepthBias(
    double relative_depth_bias,
    double clip_z, 
    double clip_w
    ) const;

  /*
  Description:
    Calculate a transformation to apply to clipping coordinates to
    bias their depth.

  Parameters:
    relative_depth_bias - [in]
      signed relative bias. 
      = 0: no bias, 
      > 0: bias towards frustum's near clipping plane
      < 0: bias towards frustum's far clipping plane
      When you have curves and points that are "on" shaded objects,
      values around 1/512 work well to move the wire objects
      in front of or behind shaded objects.

    clip_bias - [out]
      clip_bias = cam2clip * delta * clip2cam,
      where delta = 1 0 0 0 
                    0 1 0 0
                    0 0 1 D
                    0 0 0 1
      and D = relative_depth_bias*(far-near).

  Returns:
    True if the function worked.  False if the frustum settings
    are not valild, in which cate the identity matrix is returned.

  Remarks:
    The inverse of the transformations returned by 
    GetClipCoordDepthBiasXform(+r,...) is the transformation
    returned by GetClipCoordDepthBiasXform(-r,...).
  */
  bool GetClipCoordDepthBiasXform( 
    double relative_depth_bias,
    ON_Xform& clip_bias
    ) const;

  /*
  Description:
    Set suggested the perspective minimum near distance and
    minimum near/far ratio to the suggested values returned
    by GetPerspectiveClippingPlaneConstraints().
  Parameters:
    depth_buffer_bit_depth - [in]
      typically 32, 24, 16 or 8, but any positive value can be 
      passed in.
  */
  void SetPerspectiveClippingPlaneConstraints(
        unsigned int depth_buffer_bit_depth
        );

  /*
  Description:
    Expert user function to control the minimum
    ratio of near/far when perspective projections
    are begin used.
  Parameters:
    min_near_over_far - [in]
  Remarks:
    This is a runtime setting and is not saved in 3dm files.
  */
  void SetPerspectiveMinNearOverFar(double min_near_over_far);

  /*
  Description:
    Expert user function to get the minimum runtime
    value of near/far when perspective projections
    are begin used.
  Returns:
    The minimum permitted value of near/far when perspective 
    projections are begin used.
  Remarks:
    This is a runtime setting and is not saved in 3dm files.
  */
  double PerspectiveMinNearOverFar() const;

  /*
  Description:
    Expert user function to control the minimum
    value of near when perspective projections
    are begin used.
  Parameters:
    min_near_dist - [in]
  Remarks:
    This is a runtime setting and is not saved in 3dm files.
  */
  void SetPerspectiveMinNearDist(double min_near_dist);

  /*
  Description:
    Expert user function to get the minimum
    value of near when perspective projections
    are begin used.
  Returns:
    The minimum permitted value of near when perspective 
    projections are begin used.
  Remarks:
    This is a runtime setting and is not saved in 3dm files.
  */
  double PerspectiveMinNearDist() const;
  
  /*
  Description:
    Sets the viewport's id to the value used to 
    uniquely identify this viewport.
  Parameters:
    viewport_id - [in]    
  Returns:
    True if the viewport's id was successfully set
    and false otherwise (ie. the viewport uuid has
    already been set).
  Remarks:
    There is no approved way to change the viewport 
    id once it is set in order to maintain consistency
    across multiple viewports and those routines that 
    manage them.
  */
  bool  SetViewportId(const ON_UUID& viewport_id );

  ON_UUID ViewportId(void) const;

  /*
  Description:
    EXPERT USER function to change the viewport's id.
    If you change the id, you risk damaging display
    and visibility relationships in the model.
  Parameters:
    viewport_id - [in]    
  */
  void ChangeViewportId(const ON_UUID& viewport_id);
  
protected:

  // These boolean status flags are set to true when
  // the associated fields contain valid values.
  bool m_bValidCamera;
  bool m_bValidFrustum;
  bool m_bValidPort;
  unsigned char m_reserved1;

  // Camera Settings: ///////////////////////////////////////////////

  // perspective or parallel projection
  ON::view_projection m_projection;

  //   Camera location, direction and orientation (in world coordinates).
  //   These values are used to set the camera frame vectors CamX, CamY,
  //   CamZ.  If bValidCamera is true, then the CamX, CamY and CamZ
  //   vectors are properly initialized and should be used
  //   instead of CamDir[] and CamUp[].  The frame vectors CamX, CamY, CamZ
  //   are always a right handed orthonormal frame.  The CamDir
  //   and CamUp vectors contain the values passed to SetViewCamera().

  // If true and the camera is valid, then the corresponding camera
  // parameter will not be changed by view editing functions. This
  // permits user interface to easily preserve important camera
  // features without having to perform excessive calculations.
  bool m_bLockCamUp;
  bool m_bLockCamDir;
  bool m_bLockCamLoc;
  unsigned char m_frustum_symmetry_flags; // 0 != (flags & 1) top/bottom symmetry enforced
                                          // 0 != (flags & 2) left/right symmetry enforced.
  ON_3dPoint m_CamLoc;  // camera location
  ON_3dVector m_CamDir; // from camera towards view (nonzero and not parallel to m_CamUp)
  ON_3dVector m_CamUp;  // (nonzero and not parallel to m_CamDir)

  // The camera frame vectors are properly initialized by SetCamera()
  ON_3dVector m_CamX;
  ON_3dVector m_CamY;
  ON_3dVector m_CamZ;

  // View Frustum Settings: ///////////////////////////////////////
  //   left, right are camera X coords on near clipping plane
  //   bottom, top are camera Y coords on near clipping plane
  //   near = distance from camera to near clipping plane
  //   far = distance from camera to far clipping plane
  double m_frus_left,   m_frus_right; // frus_left < frus_right 
  double m_frus_bottom, m_frus_top;   // frus_bottom < frus_top 
  double m_frus_near,   m_frus_far;   // frus_near < frus_far 
                                      // in perspective, 0 < frus_near
  

  // Device View Port Box Settings: ( in display device coordinates ) ////
  //   The point (left,bottom,-near), in camera coordinates, of the view
  //   frustum is mapped to pixel coordinate (port_left,port_bottom,port_near).
  //   The point (right,top,-far), in camera coordinates, of the view frustum 
  //   is mapped to pixel coordinate (port_right,port_top,port_far).
  int m_port_left,   m_port_right; // port_left != port_right
  int m_port_bottom, m_port_top;   // port_bottom != port_top  
                                   // In many situations including Windows,
                                   // port_left = 0,
                                   // port_right = viewport width-1,
                                   // port_top = 0,
                                   // port_bottom = viewport height-1.
  int m_port_near,   m_port_far;   // (If you want an 8 bit z-buffer with 
                                   // z=255 being "in front of" z=0, then
                                   // set port_near = 255 and port_far = 0.)


  // The location of this point has no impact on the 
  // view projection. It is simply a suggestion for a 
  // fixed point when views are rotated or the isometric 
  // depth when perpsective views are dollied.  The default
  // is ON_UNSET_POINT.
  ON_3dPoint m_target_point;

private:
  // When this id matches the viewport id saved in an ON_DisplayMaterialRef
  // list in ON_3dmObjectAttributes, then the the display material is used
  // for that object in this view.
  ON_UUID m_viewport_id;

  bool SetCameraFrame(); // used to set m_CamX, m_CamY, m_CamZ

  // This transform is used to tweak the clipping 
  // coordinates.  The default is the identity.  
  // Modify this transformation when you need to do
  // things like z-buffer bias, non-uniform viewplane
  // scaling, and so on.

  /*
  Description:
    Sets the m_clip_mod transformation;
  Parameters:
    clip_mod_xform - [in] invertable transformation
  */
  bool SetClipModXform( ON_Xform clip_mod_xform );
  ON_Xform m_clip_mods;
  ON_Xform m_clip_mods_inverse;

  // Runtime values that depend on the graphics hardware being used.
  // These values are not saved in 3dm files.
  double m__MIN_NEAR_DIST;
  double m__MIN_NEAR_OVER_FAR;

public:
  static const double DefaultNearDist;        // 0.005
  static const double DefaultFarDist;         // 1000.0
  static const double DefaultMinNearDist;     // 0.0001
  static const double DefaultMinNearOverFar;  // 0.0001
};

ON_DECL
bool 
ON_GetViewportRotationAngles( 
    const ON_3dVector&, // X, // X,Y,Z must be a right handed orthonormal basis
    const ON_3dVector&, // Y, 
    const ON_3dVector&, // Z,
    double*, // angle1, // returns rotation about world Z
    double*, // angle2, // returns rotation about world X ( 0 <= a2 <= pi )
    double*  // angle3  // returns rotation about world Z
    );

ON_DECL
bool
ON_ViewportFromRhinoView( // create ON_Viewport from legacy Rhino projection info
        ON::view_projection, // projection,
        const ON_3dPoint&, // rhvp_target, // 3d point
        double, // rhvp_angle1 in radians
        double, // rhvp_angle2 in radians
        double, // rhvp_angle3 in radians
        double, // rhvp_viewsize,     // > 0
        double, // rhvp_cameradist,   // > 0
        int, // screen_width, 
        int, // screen_height,
        ON_Viewport&
        );

/*
Description:
  Calculate the corners of the polygon that is the
  intersection of a view frustum with and infinte plane.
Parameters:
  vp - [in] defines view frustum
  plane_equation - [in] defined infinte plane
  points  - [out] corners of the polygon.
    If true is returned and points.Count() is zero, then
    the plane missed the frustum.  Note that the start/end
    point is not duplicated in the list.
Returns:
  True if input was valid, false otherwise.  Note that
  even when true is returned, the returned points.Count()
  may be zero if the plane and frustum do not intersect.
*/
ON_DECL
bool
ON_IntersectViewFrustumPlane(
          const ON_Viewport& vp,
          const ON_PlaneEquation& plane_equation, 
          ON_SimpleArray<ON_3dPoint>& points 
          );

#endif


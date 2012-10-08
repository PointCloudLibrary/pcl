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

#if !defined(ON_BOUNDING_BOX_INC_)
#define ON_BOUNDING_BOX_INC_

////////////////////////////////////////////////////////////////
//
//   ON_BoundingBox  - axis aligned bounding box
//

class ON_CLASS ON_BoundingBox
{
public:
  static const ON_BoundingBox EmptyBoundingBox; // ((1.0,0,0,0,0),(-1.0,0.0,0.0))

  ON_BoundingBox(); // creates EmptyBox

	ON_BoundingBox(
    const ON_3dPoint&, // min corner of axis aligned bounding box
    const ON_3dPoint&  // max corner of axis aligned bounding box
    );
  ~ON_BoundingBox();


  // temporary - use ON_ClippingRegion - this function will be removed soon.
  int IsVisible( 
    const ON_Xform& bbox2c
    ) const;

  void Destroy(); // invalidates bounding box

  // operator[] returns min if index <= 0 and max if indes >= 1
  ON_3dPoint& operator[](int);
  const ON_3dPoint& operator[](int) const;

  ON_3dPoint Min() const;
  ON_3dPoint Max() const;
  ON_3dVector Diagonal() const; // max corner - min corner
  ON_3dPoint Center() const;
  ON_3dPoint Corner( // 8 corners of box
    int, // x_index   0 = Min().x, 1 = Max().x
    int, // y_index   0 = Min().y, 1 = Max().y
    int  // z_index   0 = Min().z, 1 = Max().z
    ) const;
  bool GetCorners( 
    ON_3dPointArray& box_corners // returns list of 8 corner points
    ) const;
  bool GetCorners( 
    ON_3dPoint box_corners[8] // returns list of 8 corner points
    ) const;

  bool IsValid() const; // empty boxes are not valid
  
  void Dump(class ON_TextLog&) const;
  
  /*
  Description:
    Test a bounding box to see if it is degenerate (flat)
    in one or more directions.
  Parameters:
    tolerance - [in] Distances <= tolerance will be considered
        to be zero.  If tolerance is negative (default), then
        a scale invarient tolerance is used.
  Returns:
    @untitled table
    0     box is not degenerate
    1     box is a rectangle (degenerate in one direction)
    2     box is a line (degenerate in two directions)
    3     box is a point (degenerate in three directions)
    4     box is not valid
  */
  int IsDegenerate( 
    double tolerance = ON_UNSET_VALUE
    ) const;


  //////////
  // ON_BoundingBox::Transform() updates the bounding box
  // to be the smallest axis aligned bounding box that contains
  // the transform of the eight corner points of the input
  // bounding box.
  bool Transform( const ON_Xform& );

  double Tolerance() const; // rough guess at a tolerance to use for comparing
                            // objects in this bounding box


  // All of these Set() functions set or expand a box to enclose the points in the arguments
  // If bGrowBox is true, the existing box is expanded, otherwise it is only set to the current point list
  bool Set(     
    int dim,
    int is_rat,
    int count,
    int stride,
    const double* point_array,
    int bGrowBox = false
    );

  bool Set(
    const ON_3dPoint& point,
    int bGrowBox = false
    );

  bool Set(     
    const ON_SimpleArray<ON_4dPoint>& point_array,
    int bGrowBox = false
    );

  bool Set(     
    const ON_SimpleArray<ON_3dPoint>& point_array,
    int bGrowBox = false
    );

  bool Set(     
    const ON_SimpleArray<ON_2dPoint>& point_array,
    int bGrowBox = false
    );

	bool IsPointIn(
         const ON_3dPoint& test_point, // point to test
         int bStrictlyIn = false 
                   // true to test for strict ( min < point < max )
                   // false to test for (min <= point <= max)
                   //       
         ) const;

  //////////
  // Point on or in the box that is closest to test_point.
  // If test_point is in or on the box, the test_point is returned.
  ON_3dPoint ClosestPoint( 
    const ON_3dPoint& test_point
    ) const;


  /*
  Description:
    Quickly find a lower bound on the distance 
    between the point and this bounding box.
  Parameters:
    P - [in]
  Returns:
    A distance that is less than or equal to the shortest
    distance from the line to this bounding box.
    Put another way, if Q is any point in this bounding box,
    then P.DistanceTo(Q) >= MinimumDistanceTo(bbox).
  */
  double MinimumDistanceTo( const ON_3dPoint& P ) const;

  /*
  Description:
    Quickly find an upper bound on the distance 
    between the point and this bounding box.
  Parameters:
    P - [in]
  Returns:
    A distance that is greater than or equal to the 
    longest distance from the point P to this bounding box.
    Put another way, if Q is any point in this bounding box,
    then P.DistanceTo(Q) <= MaximumDistanceTo(bbox).
  */
  double MaximumDistanceTo( const ON_3dPoint& P ) const;


  /*
  Description:
    Quickly find a lower bound on the distance 
    between this and the other bounding box.
  Parameters:
    other - [in]
  Returns:
    A distance that is less than or equal to the shortest
    distance between the bounding boxes.
    Put another way, if Q is any point in this bounding box
    and P is any point in the other bounding box,
    then P.DistanceTo(Q) >= MinimumDistanceTo(bbox).
  */
  double MinimumDistanceTo( const ON_BoundingBox& other ) const;

  /*
  Description:
    Quickly find an upper bound on the distance 
    between this and the other bounding box.
  Parameters:
    other - [in]
  Returns:
    A distance that is greater than or equal to the longest
    distance between the bounding boxes.
    Put another way, if Q is any point in this bounding box
    and P is any point in the other bounding box,
    then P.DistanceTo(Q) <= MaximumDistanceTo(bbox).
  */
  double MaximumDistanceTo( const ON_BoundingBox& other ) const;

  /*
  Description:
    Quickly find a lower bound on the distance 
    between the line segment and this bounding box.
  Parameters:
    line - [in]
  Returns:
    A distance that is less than or equal to the shortest
    distance from the line to this bounding box.
    Put another way, if Q is any point on line
    and P is any point in this bounding box, then
    P.DistanceTo(Q) >= MinimumDistanceTo(bbox).
  */
  double MinimumDistanceTo( const ON_Line& line ) const;

  /*
  Description:
    Quickly find a tight lower bound on the distance 
    between the plane and this bounding box.
  Parameters:
    plane - [in]
  Returns:
    The minimum distance between a point on the plane
    and a point on the bounding box.
  See Also:
    ON_PlaneEquation::MimimumValueAt
    ON_PlaneEquation::MaximumValueAt
  */
  double MinimumDistanceTo( const ON_Plane& plane ) const;
  double MinimumDistanceTo( const ON_PlaneEquation& plane_equation ) const;

  /*
  Description:
    Quickly find an upper bound on the distance 
    between the line segment and this bounding box.
  Parameters:
    line - [in]
  Returns:
    A distance that is greater than or equal to the 
    longest distance from the line to this bounding box.
    Put another way, if Q is any point on the line
    and P is any point in this bounding box, then
    P.DistanceTo(Q) <= MaximumDistanceTo(bbox).
  */
  double MaximumDistanceTo( const ON_Line& line ) const;

  /*
  Description:
    Quickly find a tight upper bound on the distance 
    between the plane and this bounding box.
  Parameters:
    plane - [in]
  Returns:
    A distance that is equal to the longest distance from
    the plane to this bounding box.  Put another way, 
    if Q is any point on the plane and P is any point 
    in this bounding box, then 
    P.DistanceTo(Q) <= MaximumDistanceTo(bbox) and there
    is at least one point on the bounding box where the
    distance is equal to the returned value.
  See Also:
    ON_PlaneEquation::MaximumValueAt
  */
  double MaximumDistanceTo( const ON_Plane& plane ) const;
  double MaximumDistanceTo( const ON_PlaneEquation& plane_equation ) const;


  /*
  Description:
    Quickly determine if the shortest distance from
    the point P to the bounding box is greater than d.
  Parameters:
    d - [in] distance (> 0.0)
    P - [in] 
  Returns:
    True if if the shortest distance from the point P
    to the bounding box is greater than d. 
  */
  bool IsFartherThan( double d, const ON_3dPoint& P ) const;

  /*
  Description:
    Quickly determine if the shortest distance from the line
    to the bounding box is greater than d.
  Parameters:
    d - [in] distance (> 0.0)
    line - [in] 
  Returns:
    True if the shortest distance from the line
    to the bounding box is greater than d. It is not the
    case that false means that the shortest distance
    is less than or equal to d.
  */
  bool IsFartherThan( double d, const ON_Line& line ) const;

  /*
  Description:
    Quickly determine if the shortest distance from the plane
    to the bounding box is greater than d.
  Parameters:
    d - [in] distance (> 0.0)
    plane - [in] 
  Returns:
    True if the shortest distance from the plane
    to the bounding box is greater than d, and false
    if the shortest distance is less than or equal to d.
  */
  bool IsFartherThan( double d, const ON_Plane& plane ) const;

  /*
  Description:
    Quickly determine if the shortest distance from the plane
    to the bounding box is greater than d.
  Parameters:
    d - [in] distance (> 0.0)
    plane_equation - [in] (the first three coefficients 
                           are assumed to be a unit vector.
                           If not, adjust your d accordingly.)
  Returns:
    True if the shortest distance from the plane
    to the bounding box is greater than d, and false
    if the shortest distance is less than or equal to d.
  */
  bool IsFartherThan( double d, const ON_PlaneEquation& plane_equation ) const;

  /*
  Description:
    Quickly determine if the shortest distance this bounding
    box to another bounding box is greater than d.
  Parameters:
    d - [in] distance (> 0.0)
    other - [in] other bounding box
  Returns:
    True if if the shortest distance from this bounding
    box to the other bounding box is greater than d. 
  */
  bool IsFartherThan( double d, const ON_BoundingBox& other ) const;


  // Description:
  //   Get point in a bounding box that is closest to a line
  //   segment.
  // Parameters:
  //   line - [in] line segment
  //   box_point - [out] point in box that is closest to line
  //       segment point at t0.
  //   t0 - [out] parameter of point on line that is closest to
  //       the box.
  //   t1 - [out] parameter of point on line that is closest to
  //       the box.
  // Returns:
  //   3 success - line segments intersects box in a segment
  //               from line(t0) to line(t1) (t0 < t1)
  //   2 success - line segments intersects box in a single point
  //               at line(t0) (t0==t1)
  //   1 success - line segment does not intersect box.  Closest
  //               point on the line is at line(t0) (t0==t1)
  //   0 failure - box is invalid.
  // Remarks:
  //   The box is treated as a solid box.  If the intersection
  //   of the line segment, then 3 is returned.
  int GetClosestPoint( 
    const ON_Line&, // line
    ON_3dPoint&,    // box_point
    double*,        // t0
    double*         // t1
    ) const;

  //////////
  // Get points on bounding boxes that are closest to each other.
  // If the boxes intersect, then the point at the centroid of the
  // intersection is returned for both points.
  bool GetClosestPoint( 
         const ON_BoundingBox&, // "other" bounding box
         ON_3dPoint&, // point on "this" box that is closest to "other" box
         ON_3dPoint&  // point on "other" box that is closest to "this" box
         )  const;

  //////////
  // Point on the box that is farthest from the test_point.
  ON_3dPoint FarPoint( 
    const ON_3dPoint& // test_point
    ) const;

  //////////
  // Get points on bounding boxes that are farthest from each other.
  bool GetFarPoint( 
         const ON_BoundingBox&, // "other" bounding box
         ON_3dPoint&, // point on "this" box that is farthest from "other" box
         ON_3dPoint&  // point on "other" box that is farthest from "this" box
         )  const;

  /* 
  Description:
    Intersect this with other_bbox and save intersection in this.
  Parameters:
    other_bbox - [in]
  Returns:
    True if this-intesect-other_bbox is a non-empty valid bounding box
    and this is set.  False if the intersection is empty, in which case
    "this" is set to an invalid bounding box.
  Remarks:
    If "this" or other_bbox is invalid, they are treated as
    the empty set, and false is returned.
  */
  bool Intersection(
         const ON_BoundingBox& other_bbox
         );

  /* 
  Description:
    Set "this" to the intersection of bbox_A and bbox_B.
  Parameters:
    bbox_A - [in] 
    bbox_B - [in]
  Returns:
    True if the "this" is a non-empty valid bounding box.
    False if the intersection is empty, in which case
    "this" is set to an invalid bounding box.
  Remarks:
    If bbox_A or bbox_B is invalid, they are treated as
    the empty set, and false is returned.
  */
  bool Intersection( // this = intersection of two args
         const ON_BoundingBox& bbox_A, 
         const ON_BoundingBox& bbox_B
         );

	bool Intersection(				//Returns true when intersect is non-empty. 
				 const ON_Line&,		//Infinite Line segment to intersect with 
				 double* =NULL ,			// t0  parameter of first intersection point
				 double* =NULL       // t1  parameter of last intersection point (t0<=t1)   
				 ) const;			 

  /* 
  Description:
    Test a box to see if it is contained in this box.
  Parameters:
    other - [in] box to test
    bProperSubSet - [in] if true, then the test is for a proper inclusion.
  Returns:
    If bProperSubSet is false, then the result is true when
			this->m_min[i] <= other.m_min[i] and other.m_max[i] <= this->m_max[i].
			for i=0,1 and 2.
    If bProperSubSet is true, then the result is true when
			the above condition is true and at least one of the inequalities is strict.
  */
  bool Includes( 
    const ON_BoundingBox& other,
    bool bProperSubSet = false
    ) const;

	double Volume() const;

  double Area() const;

  // Union() returns true if union is not empty.
  // Invalid boxes are treated as the empty set.
  bool Union( // this = this union arg
         const ON_BoundingBox&
         );

  bool Union( // this = union of two args
         const ON_BoundingBox&, 
         const ON_BoundingBox&
         );
                  
  /* 
  Description:
    Test to see if "this" and other_bbox are disjoint (do not intersect).
  Parameters:
    other_bbox - [in]
  Returns:
    True if "this" and other_bbox are disjoint.
  Remarks:
    If "this" or other_bbox is invalid, then true is returned.
  */
  bool IsDisjoint(
    const ON_BoundingBox& other_bbox
    ) const;

  bool SwapCoordinates( int, int );

  ON_3dPoint m_min;
  ON_3dPoint m_max;
};

#if defined(ON_DLL_TEMPLATE)

// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_BoundingBox>;
#pragma warning( pop )

#endif

/*
Description:
  Get a tight bounding box that contains the points.
Parameters:
 dim - [in] (>=1)
 is_rat - [in] true if points are rational
 count - [in] number of points
 stride - [in] stride between points
 point_list - [in]
 bbox - [in/out]
 bGrowBox - [in] (default = false)
   If the input bbox is valid and bGrowBox is true,
   then the output bbox is the union of the input
   bbox and the bounding box of the point list.
 xform - [in] (default = NULL)
   If not null, the bounding box of the transformed
   points is calculated.  The points are not modified.
Returns:
  True if the output bbox is valid.
*/
ON_DECL
bool ON_GetPointListBoundingBox(
    int dim,
    int is_rat,
    int count,
    int stride,
    const double* point_list,
    ON_BoundingBox& bbox,
    int bGrowBox = false,
    const ON_Xform* xform = 0
    );

ON_DECL
bool ON_GetPointListBoundingBox(
    int dim,
    int is_rat,
    int count,
    int stride,
    const float* point_list,
    ON_BoundingBox& bbox,
    int bGrowBox = false,
    const ON_Xform* xform = 0
    );

ON_DECL
bool ON_GetPointListBoundingBox(
    int dim,
    int is_rat,
    int count,
    int stride,
    const double* point_list,
    double* boxmin,       // min[dim]
    double* boxmax,       // max[dim]
    int bGrowBox
    );

ON_DECL
ON_BoundingBox ON_PointListBoundingBox(
    int dim,
    int is_rat,
    int count,
    int stride,
    const double* point_list
    );

ON_DECL
bool ON_GetPointListBoundingBox(
    int dim,
    int is_rat,
    int count,
    int stride,
    const float* point_list,
    float* boxmin,       // min[dim]
    float* boxmax,       // max[dim]
    int bGrowBox
    );

ON_DECL
ON_BoundingBox ON_PointListBoundingBox( // low level workhorse function
    int dim,
    int is_rat,
    int count,
    int stride,
    const float* point_list
    );

ON_DECL
bool ON_GetPointGridBoundingBox(
        int dim,
        int is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        const double* point_grid,
        double* boxmin,       // min[dim]
        double* boxmax,       // max[dim]
        int bGrowBox
    );

ON_DECL
ON_BoundingBox ON_PointGridBoundingBox(
        int dim,
        int is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        const double* point_grid
    );

ON_DECL
double ON_BoundingBoxTolerance(
        int dim,
        const double* bboxmin,
        const double* bboxmax
        );

/*
Description:
  Determine if an object is too large or too far 
  from the origin for single precision coordinates
  to be useful.
Parameters:
  bbox - [in]
    Bounding box of an object with single precision
    coordinates.  An ON_Mesh is an example of an
    object with single precision coordinates.
  xform - [out]
    If this function returns false and xform is not
    null, then the identity transform is returned.
    If this function returns true and xform is not
    null, then the transform moves the region
    contained in bbox to a location where single 
    precision coordinates will have enough
    information for the object to be useful.
Returns:
  true:
    The region contained in bbox is too large
    or too far from the origin for single 
    precision coordinates to be useful.
  false:
    A single precision object contained in bbox
    will be satisfactory for common calculations.
*/
ON_DECL
bool ON_BeyondSinglePrecision( const ON_BoundingBox& bbox, ON_Xform* xform );

ON_DECL
bool ON_WorldBBoxIsInTightBBox( 
          const ON_BoundingBox& tight_bbox, 
          const ON_BoundingBox& world_bbox,
          const ON_Xform* xform
          );

#endif

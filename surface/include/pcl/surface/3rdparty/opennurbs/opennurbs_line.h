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

#if !defined(ON_LINE_INC_)
#define ON_LINE_INC_

class ON_CLASS ON_Line
{
public:

  ON_Line();
  ON_Line( const ON_3dPoint& start, const ON_3dPoint& end );
  ~ON_Line();

  /*
  Returns:
    True if from != to.
  */
  bool IsValid() const;

  // line[0] = start point line[1] = end point
  ON_3dPoint& operator[](int);
  const ON_3dPoint& operator[](int) const;


  // Description:
  //   Create a line from two points.
  // Parameters:
  //   start - [in] point at start of line segment
  //   end - [in] point at end of line segment
  // Returns:
  //   true if start and end are distinct points.
  bool Create( 
    const ON_3dPoint& start, 
    const ON_3dPoint& end
    );

  /*
  Description: 
    Get line's 3d axis aligned bounding box.
  Returns:
    3d bounding box.
  */
  ON_BoundingBox BoundingBox() const;

  /*
  Description:
    Get line's 3d axis aligned bounding box or the
    union of the input box with the object's bounding box.
  Parameters:
    bbox - [in/out] 3d axis aligned bounding box
    bGrowBox - [in] (default=false) 
      If true, then the union of the input bbox and the 
      object's bounding box is returned in bbox.  
      If false, the object's bounding box is returned in bbox.
  Returns:
    true if object has bounding box and calculation was successful.
  */
  bool GetBoundingBox(
         ON_BoundingBox& bbox,
         int bGrowBox = false
         ) const;

  /*
	Description:
    Get tight bounding box.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      line's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      line is calculated.  The line is not modified.
	Returns:
    True if a valid tight_bbox is returned.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
  Description:
    Get a plane that contains the line.
  Parameters:
    plane - [out] a plane that contains the line.  The orgin
       of the plane is at the start of the line.  The distance
       from the end of the line to the plane is <= tolerance.
       If possible a plane parallel to the world xy, yz or zx
       plane is returned.
    tolerance - [in]
  Returns:
    true if a coordinate of the line's direction vector is
    larger than tolerance.
  */
  bool InPlane( ON_Plane& plane, double tolerance = 0.0 ) const;

  // Returns:
  //   Length of line
  double Length() const;

  // Returns:
  //   direction vector = line.to - line.from
  // See Also:
  //   ON_Line::Tangent
  ON_3dVector Direction() const;

  // Returns:
  //   Unit tangent vector.
  // See Also:
  //   ON_Line::Direction
  ON_3dVector Tangent() const;

  /*
  Description:
    Evaluate point on (infinite) line.
  Parameters:
    t - [in] evaluation parameter. t=0 returns line.from
             and t=1 returns line.to.
  Returns:
    (1-t)*line.from + t*line.to.
  See Also:
    ON_Line::Direction
    ON_Line::Tangent
  */
  ON_3dPoint PointAt( 
    double t 
    ) const;

  /*
  Description:
    Find the point on the (infinite) line that is
    closest to the test_point.
  Parameters:
    test_point - [in]
    t - [out] line.PointAt(*t) is the point on the line 
              that is closest to test_point.
  Returns:
    true if successful.
  */
  bool ClosestPointTo( 
    const ON_3dPoint& test_point, 
    double* t
    ) const;

  /*
  Description:
    Find the point on the (infinite) line that is
    closest to the test_point.
  Parameters:
    test_point - [in]
  Returns:
    The point on the line that is closest to test_point.
  */
  ON_3dPoint ClosestPointTo( 
    const ON_3dPoint& test_point
    ) const;

  /*
  Description:
    Find the point on the (infinite) line that is
    closest to the test_point.
  Parameters:
    test_point - [in]
  Returns:
    distance from the point on the line that is closest
    to test_point.
  See Also:
    ON_3dPoint::DistanceTo
    ON_Line::ClosestPointTo
  */
  double DistanceTo( ON_3dPoint test_point ) const;


  /*
  Description:
    Finds the shortest distance between the line as a finite
    chord and the other object.
  Parameters:
    P - [in]
    L - [in] (another finite chord)
  Returns:
    A value d such that if Q is any point on 
    this line and P is any point on the other object, 
    then d <= Q.DistanceTo(P).
  */
  double MinimumDistanceTo( const ON_3dPoint& P ) const;
  double MinimumDistanceTo( const ON_Line& L ) const;

  /*
  Description:
    Finds the longest distance between the line as a finite
    chord and the other object.
  Parameters:
    P - [in]
    L - [in] (another finite chord)
  Returns:
    A value d such that if Q is any point on this line and P is any
    point on the other object, then d >= Q.DistanceTo(P).
  */
  double MaximumDistanceTo( const ON_3dPoint& P ) const;
  double MaximumDistanceTo( const ON_Line& other ) const;


  /*
  Description:
    Quickly determine if the shortest distance from
    this line to the other object is greater than d.
  Parameters:
    d - [in] distance (> 0.0)
    P - [in] 
    L - [in] 
  Returns:
    True if if the shortest distance from this line
    to the other object is greater than d.
  */
  bool IsFartherThan( double d, const ON_3dPoint& P ) const;
  bool IsFartherThan( double d, const ON_Line& L ) const;


  // For intersections see ON_Intersect();

  // Description:
  //   Reverse line by swapping from and to.
  void Reverse();

  bool Transform( 
    const ON_Xform& xform
    );

  // rotate line about a point and axis
  bool Rotate(
        double sin_angle,
        double cos_angle,
        const ON_3dVector& axis_of_rotation,
        const ON_3dPoint& center_of_rotation
        );

  bool Rotate(
        double angle_in_radians,
        const ON_3dVector& axis_of_rotation,
        const ON_3dPoint& center_of_rotation
        );

  bool Translate(
        const ON_3dVector& delta
        );

public:
  ON_3dPoint from; // start point
  ON_3dPoint to;   // end point
};

#endif

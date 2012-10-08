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

#if !defined(ON_BOX_INC_)
#define ON_BOX_INC_

class ON_CLASS ON_Box
{
public:
  ON_Plane plane; 
  // intervals are finite and increasing when the box is valid
  ON_Interval dx;
  ON_Interval dy;
  ON_Interval dz;

  ON_Box();
  ON_Box( const ON_BoundingBox& bbox );
  ~ON_Box();

  bool IsValid() const;

  bool Create( const ON_BoundingBox& bbox );

  void Destroy();

  ON_3dPoint Center() const;
  bool GetCorners( ON_3dPoint* corners ) const;
  bool GetCorners( ON_SimpleArray<ON_3dPoint>& corners ) const;

  ON_BoundingBox BoundingBox() const;

  ON_3dPoint PointAt( 
          double r, 
          double s, 
          double t 
          ) const;

  bool ClosestPointTo( 
          ON_3dPoint point, 
          double* r, 
          double* s, 
          double* t 
          ) const;

  // returns point on box that is closest to given point
  ON_3dPoint ClosestPointTo( 
         ON_3dPoint test_point
         ) const;

  // rotate sphere about its origin
  bool Rotate(
        double sin_angle,               // sin(angle)
        double cos_angle,               // cos(angle)
        const ON_3dVector& axis_of_rotation // axis of rotation
        );

  bool Rotate(
        double angle_radians,               // angle in radians
        const ON_3dVector& axis_of_rotation // axis of rotation
        );

  // rotate sphere about a point and axis
  bool Rotate(
        double sin_angle,               // sin(angle)
        double cos_angle,               // cos(angle)
        const ON_3dVector& axis_of_rotation, // axis of rotation
        const ON_3dPoint& center_of_rotation  // center of rotation
        );

  bool Rotate(
        double angle_radians,               // angle in radians
        const ON_3dVector& axis_of_rotation, // axis of rotation
        const ON_3dPoint& center_of_rotation  // center of rotation
        );

  bool Translate(
        const ON_3dVector&
        );

  bool Transform( const ON_Xform& );

  /*
  Description:
    Test the box to see if it is degenerate (flat)
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

  double Volume() const;

  double Area() const;
};

#endif

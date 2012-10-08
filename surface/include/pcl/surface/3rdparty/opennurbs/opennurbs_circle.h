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

#if !defined(ON_CIRCLE_INC_)
#define ON_CIRCLE_INC_

class ON_NurbsCurve;

/*
Description:
	ON_Circle is a circle in 3d.  The cirle is represented by a radius and an 
	orthonormal frame	of the plane containing the circle, with origin at the center.

	An Is_Valid() circle has positive radius and an Is_ Valid() plane defining the frame.
	
	The circle is parameterized by radians from 0 to 2 Pi given by 
     t -> center + cos(t)*radius*xaxis + sin(t)*radius*yaxis	
	where center, xaxis and yaxis define the orthonormal frame of the circle's plane.  
*/
class ON_CLASS ON_Circle
{
public:
  // Creates a radius one circle with center (0,0,0)
  // in the world XY plane
  ON_Circle();

  // Creates a circle in the plane with center at
  // plane.origin.
  ON_Circle(
    const ON_Plane& plane,
    double radius
    );

  // Creates a circle parallel to the world XY plane
  // with given center and radius
  ON_Circle(
    const ON_3dPoint& center,
    double radius
    );

  // Creates a circle parallel to the plane
  // with given center and radius.
  ON_Circle(
    const ON_Plane& plane,
    const ON_3dPoint& center,
    double radius
    );

  // Create a circle through three 2d points.
  // The start/end of the circle is at point P.
  ON_Circle( // circle through 3 2d points
    const ON_2dPoint& P,
    const ON_2dPoint& Q,
    const ON_2dPoint& R
    );

  // Create a circle through three 3d points.
  // The start/end of the circle is at point P.
  ON_Circle(
    const ON_3dPoint& P,
    const ON_3dPoint& Q,
    const ON_3dPoint& R
    );

  ~ON_Circle();

  // Creates a circle in the plane with center at
  // plane.origin.
  bool Create(
    const ON_Plane& plane,
    double radius
    );

  // Creates a circle parallel to the world XY plane
  // with given center and radius
  bool Create(
    const ON_3dPoint& center,
    double radius
    );

  // Creates a circle parallel to the plane
  // with given centr and radius.
  bool Create(
    const ON_Plane& plane,
    const ON_3dPoint& center,
    double radius
    );

  // Create a circle through three 2d points.
  // The start/end of the circle is at point P.
  bool Create( // circle through 3 2d points
    const ON_2dPoint& P,
    const ON_2dPoint& Q,
    const ON_2dPoint& R
    );

  // Create a circle through three 3d points.
  // The start/end of the circle is at point P.
  bool Create(
    const ON_3dPoint& P,
    const ON_3dPoint& Q,
    const ON_3dPoint& R
    );

  // Create a circle from two 2d points and a 
  // tangent at the first point.
  // The start/end of the circle is at point P.
  bool Create(
    const ON_2dPoint& P,
    const ON_2dVector& tangent_at_P,
    const ON_2dPoint& Q
    );

  // Create a circle from two 3d points and a 
  // tangent at the first point.
  // The start/end of the circle is at point P.
  bool Create(
    const ON_3dPoint& P,
    const ON_3dVector& tangent_at_P,
    const ON_3dPoint& Q
    );

	//	A Valid circle has m_radius>0 and m_plane.IsValid().
  bool IsValid() const;

  //bool UpdatePoints();  // sets m_point[] to have valid points

  bool IsInPlane( const ON_Plane&, double = ON_ZERO_TOLERANCE ) const;

  double Radius() const;
  double Diameter() const;
  double Circumference() const;
  const ON_3dPoint& Center() const;
  const ON_3dVector& Normal() const;
  const ON_Plane& Plane() const; // plane containing circle

  ON_BoundingBox BoundingBox() const;

  /*
	Description:
    Get tight bounding box.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      arc's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      arc is calculated.  The arc is not modified.
	Returns:
    True if a valid tight_bbox is returned.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  bool Transform( const ON_Xform& );

  // Circles use trigonometric parameterization
  // t -> center + cos(t)*radius*xaxis + sin(t)*radius*yaxis
  ON_3dPoint PointAt( 
    double // evaluation parameter
    ) const;
  ON_3dVector DerivativeAt( 
    int,   // derivative (>=0)
    double // evaluation parameter
    ) const;

  ON_3dVector TangentAt(double) const;

  // returns parameters of point on circle that is closest to given point
  bool ClosestPointTo( 
         const ON_3dPoint& point, 
         double* t
         ) const;

  // returns point on circle that is closest to given point
  ON_3dPoint ClosestPointTo( 
         const ON_3dPoint& point
         ) const;

  // evaluate circle's implicit equation in plane
  double EquationAt( const ON_2dPoint& plane_point ) const;

  ON_2dVector GradientAt( const ON_2dPoint& plane_point ) const;

  // rotate circle about its center
  bool Rotate(
        double sin_angle,
        double cos_angle,
        const ON_3dVector& axis_of_rotation
        );

  bool Rotate(
        double angle_in_radians,
        const ON_3dVector&  axis_of_rotation
        );

  // rotate circle about a point and axis
  bool Rotate(
        double sin_angle,
        double cos_angle,
        const ON_3dVector& axis_of_rotation,
        const ON_3dPoint& center_of_rotation
        );

  bool Rotate(
        double angle_in_radians,
        const ON_3dVector&  axis_of_rotation,
        const ON_3dPoint& center_of_rotation
        );

  bool Translate(
        const ON_3dVector& delta
        );

  bool Reverse();

  // Description:
  //   Get a four span rational degree 2 NURBS circle representation
  //   of the circle.
  // Returns:
  //   2 for success, 0 for failure
  // Remarks:
  //   Note that the parameterization of NURBS curve
  //   does not match  circle's transcendental paramaterization.  
  //   Use ON_Circle::GetRadianFromNurbFormParameter() and
  //   ON_Circle::GetParameterFromRadian() to convert between 
  //   the NURBS curve parameter and the transcendental parameter.
  int GetNurbForm(
        ON_NurbsCurve& nurbs_curve
        ) const; 

  /*
  Description:
    Convert a NURBS curve circle parameter to a circle radians parameter.
  Parameters:
    nurbs_parameter - [in]
    circle_radians_parameter - [out]
  Example:

          ON_Circle circle = ...;
          double nurbs_t = 1.2345; // some number in interval (0,2.0*ON_PI).
          double circle_t;
          circle.GetRadianFromNurbFormParameter( nurbs_t, &circle_t );

          ON_NurbsCurve nurbs_curve;
          circle.GetNurbsForm( nurbs_curve );
          circle_pt = circle.PointAt(circle_t);
          nurbs_pt = nurbs_curve.PointAt(nurbs_t);
          // circle_pt and nurbs_pt will be the same

  Remarks:
    The NURBS curve parameter is with respect to the NURBS curve
    created by ON_Circle::GetNurbForm.  At nurbs parameter values of 
    0.0, 0.5*ON_PI, ON_PI, 1.5*ON_PI, and 2.0*ON_PI, the nurbs
    parameter and radian parameter are the same.  At all other
    values the nurbs and radian parameter values are different.
  See Also:
    ON_Circle::GetNurbFormParameterFromRadian
  */
  bool GetRadianFromNurbFormParameter(
        double nurbs_parameter,
        double* circle_radians_parameter
        ) const;

  /*
  Description:
    Convert a circle radians parameter to a NURBS curve circle parameter.
  Parameters:
    circle_radians_parameter - [in] 0.0 to 2.0*ON_PI
    nurbs_parameter - [out]
  Example:

          ON_Circle circle = ...;
          double circle_t = 1.2345; // some number in interval (0,2.0*ON_PI).
          double nurbs_t;
          circle.GetNurbFormParameterFromRadian( circle_t, &nurbs_t );

          ON_NurbsCurve nurbs_curve;
          circle.GetNurbsForm( nurbs_curve );
          circle_pt = circle.PointAt(circle_t);
          nurbs_pt = nurbs_curve.PointAt(nurbs_t);
          // circle_pt and nurbs_pt will be the same

  Remarks:
    The NURBS curve parameter is with respect to the NURBS curve
    created by ON_Circle::GetNurbForm.  At radian values of 
    0.0, 0.5*ON_PI, ON_PI, 1.5*ON_PI, and 2.0*ON_PI, the nurbs
    parameter and radian parameter are the same.  At all other
    values the nurbs and radian parameter values are different.
  See Also:
    ON_Circle::GetNurbFormParameterFromRadian
  */
  bool GetNurbFormParameterFromRadian(
        double circle_radians_parameter,
        double* nurbs_parameter
        ) const;

public:
  // circle is in the plane with center at plane.m_origin.
  ON_Plane   plane;  
  double     radius;   // radius
  //ON_3dPoint m_point[3]; // 3 points on the circle
};


#endif


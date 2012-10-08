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

#if !defined(OPENNURBS_ELLIPSE_INC_)
#define OPENNURBS_ELLIPSE_INC_

class ON_Ellipse;
class ON_Plane;

class ON_CLASS ON_Ellipse
{
public:
  ON_Ellipse(); // zeros all fields - plane is invalid

  ON_Ellipse(
    const ON_Plane&,
    double, double     // radii for x and y vectors
    );

  ON_Ellipse(
    const ON_Circle&
    );

  ~ON_Ellipse();

  ON_Ellipse& operator=(const ON_Circle&);

  ON_BOOL32 Create(
    const ON_Plane&,  // point on the plane
    double, double     // radii for x and y vectors
    );

  ON_BOOL32 Create(
    const ON_Circle&
    );

  ON_BOOL32 IsValid() const; // returns true if all fields contain reasonable
                        // information and equation jibes with point and Z.

  ON_BOOL32 IsCircle() const; // returns true is ellipse is a circle

  double Radius( 
    int // 0 = x axis radius, 1 = y axis radius
    ) const; 
  const ON_3dPoint& Center() const;
  const ON_3dVector& Normal() const;
  const ON_Plane& Plane() const; // plane containing ellipse

  /*
  Returns:
    Distance from the center to a focus, commonly called "c".
  */
  double FocalDistance() const;

  bool GetFoci( ON_3dPoint& F1, ON_3dPoint& F2 ) const;

  // Evaluation uses the trigonometrix parameterization
  // t -> plane.origin + cos(t)*radius[0]*plane.xaxis + sin(t)*radius[1]*plane.yaxis
  // evaluate parameters and return point
  ON_3dPoint  PointAt( double ) const;
  ON_3dVector DerivativeAt( 
                 int, // desired derivative ( >= 0 )
                 double // parameter
                 ) const;

  ON_3dVector TangentAt( double ) const;  // returns unit tangent
  ON_3dVector CurvatureAt( double ) const;  // returns curvature vector

  // returns parameters of point on ellipse that is closest to given point
  ON_BOOL32 ClosestPointTo( 
         const ON_3dPoint&, 
         double*
         ) const;
  // returns point on ellipse that is closest to given point
  ON_3dPoint ClosestPointTo( 
         const ON_3dPoint& 
         ) const;

  // evaluate ellipse's implicit equation in plane
  double EquationAt( const ON_2dPoint& ) const;
  ON_2dVector GradientAt( const ON_2dPoint& ) const;

  // rotate ellipse about its center
  ON_BOOL32 Rotate(
        double,              // sin(angle)
        double,              // cos(angle)
        const ON_3dVector&  // axis of rotation
        );
  ON_BOOL32 Rotate(
        double,              // angle in radians
        const ON_3dVector&  // axis of rotation
        );

  // rotate ellipse about a point and axis
  ON_BOOL32 Rotate(
        double,              // sin(angle)
        double,              // cos(angle)
        const ON_3dVector&, // axis of rotation
        const ON_3dPoint&   // center of rotation
        );
  ON_BOOL32 Rotate(
        double,              // angle in radians
        const ON_3dVector&, // axis of rotation
        const ON_3dPoint&   // center of rotation
        );

  ON_BOOL32 Translate(
        const ON_3dVector&
        );

  // parameterization of NURBS curve does not match ellipse's transcendental paramaterization
  int GetNurbForm( ON_NurbsCurve& ) const; // returns 0=failure, 2=success

public: // members left public
  // The center of the ellipse is at the plane's origin.  The axes of the
  // ellipse are the plane's x and y axes. The equation of the ellipse 
  // with respect to the plane is (x/m_r[0])^2 + (y/m_r[1])^2 = 1;
  ON_Plane plane;
  double radius[2]; // radii for x and y axes (both must be > 0)
};

#endif

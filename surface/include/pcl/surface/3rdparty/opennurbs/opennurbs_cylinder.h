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

#if !defined(OPENNURBS_CYLINDER_INC_)
#define OPENNURBS_CYLINDER_INC_

class ON_NurbsSurface;
class ON_RevSurface;
class ON_Brep;

/*
Description:
  ON_Cylinder is a right circular cylinder.
*/
class ON_CLASS ON_Cylinder
{
public:
  ON_Cylinder(); // zeros all fields - cylinder is invalid

  ON_Cylinder( // infinte cylinder
    const ON_Circle&  // point on the bottom plane
    );

  ON_Cylinder( // infinte cylinder
    const ON_Circle&,  // point on the bottom plane
    double             // height
    );

  ~ON_Cylinder();

  bool Create(
    const ON_Circle&  // point on the bottom plane
    );

  bool Create(
    const ON_Circle&,  // point on the bottom plane
    double               // height
    );

  bool IsValid() const; // returns true if all fields contain reasonable
                        // information and equation jibes with point and Z.

  bool IsFinite() const; // returns true if the cylinder is finite
                         // (height[0] != height[1]) and false if the
                         // cylinder is infinite.

  const ON_3dVector& Axis() const;
  const ON_3dPoint& Center() const;
  double Height() const; // returns 0 for infinite cylinder
  ON_Circle CircleAt( 
        double // linear parameter
        ) const;
  ON_Line LineAt( 
        double // angular parameter
        ) const;

  // evaluate parameters and return point
  ON_3dPoint PointAt(
    double, // angular parameter [0,2pi]
    double  // linear parameter (height from base circle's plane)
    ) const;
  ON_3dPoint NormalAt(
    double, // angular parameter [0,2pi]
    double  // linear parameter (height from base circle's plane)
    ) const;

  // returns parameters of point on cylinder that is closest to given point
  bool ClosestPointTo( 
         ON_3dPoint, 
         double*, // angular parameter [0,2pi]
         double*  // linear parameter (height from base circle's plane)
         ) const;
  // returns point on cylinder that is closest to given point
  ON_3dPoint ClosestPointTo( 
         ON_3dPoint 
         ) const;

  // For intersections see ON_Intersect();

  // rotate cylinder about its origin
  bool Rotate(
        double,               // sin(angle)
        double,               // cos(angle)
        const ON_3dVector&  // axis of rotation
        );
  bool Rotate(
        double,               // angle in radians
        const ON_3dVector&  // axis of rotation
        );

  // rotate cylinder about a point and axis
  bool Rotate(
        double,               // sin(angle)
        double,               // cos(angle)
        const ON_3dVector&, // axis of rotation
        const ON_3dPoint&   // center of rotation
        );
  bool Rotate(
        double,              // angle in radians
        const ON_3dVector&, // axis of rotation
        const ON_3dPoint&   // center of rotation
        );

  bool Translate(
        const ON_3dVector&
        );

  // parameterization of NURBS surface does not match cylinder's transcendental paramaterization
  int GetNurbForm( ON_NurbsSurface& ) const; // returns 0=failure, 2=success

  /*
  Description:
    Creates a surface of revolution definition of the cylinder.
  Parameters:
    srf - [in] if not NULL, then this srf is used.
  Result:
    A surface of revolution or NULL if the cylinder is not 
    valid or is infinite.
  */
  ON_RevSurface* RevSurfaceForm( ON_RevSurface* srf = NULL ) const;

public: // members left public
  // base circle
  ON_Circle  circle;

  
  // If height[0] = height[1], the cylinder is infinite,
  // Otherwise, height[0] < height[1] and the center of
  // the "bottom" cap is 
  //
  //          circle.plane.origin + height[0]*circle.plane.zaxis,
  //
  // and the center of the top cap is 
  //
  //          circle.plane.origin + height[1]*circle.plane.zaxis.
  double height[2];
};

#endif

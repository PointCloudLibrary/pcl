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

#if !defined(ON_SPHERE_INC_)
#define ON_SPHERE_INC_

class ON_RevSurface;

class ON_CLASS ON_Sphere
{
public:
  
  ON_Plane plane; // equitorial plane
  double radius;  // > 0

  ON_Sphere();
  ON_Sphere( const ON_3dPoint& center, double radius ); // center, radius
  ~ON_Sphere();

  bool IsValid() const;

  bool Create( const ON_3dPoint& center, double radius); // center radius

  ON_Circle LatitudeRadians(double latitude_radians ) const;
  ON_Circle LatitudeDegrees(double latitude_degrees) const;
  ON_Circle LongitudeRadians(double longitude_radians) const;
  ON_Circle LongitudeDegrees(double longitude_degrees) const;

  ON_3dPoint Center() const;
  ON_3dPoint NorthPole() const;
  ON_3dPoint SouthPole() const;
  double Diameter() const;
  double Radius() const;

  ON_3dPoint PointAt(
    double longitude_radians, 
    double latitude_radians
    ) const;   // longitude [0,2pi], latitude [-pi/2,pi/2] in radians

  ON_3dVector NormalAt(
    double longitude_radians, 
    double latitude_radians
    ) const;   // longitude [0,2pi], latitude [-pi/2,pi/2] in radians

  ON_BoundingBox BoundingBox() const;

  // returns parameters of point on sphere that is closest to given point
  bool ClosestPointTo( 
         ON_3dPoint test_point, 
         double* longitude_radians, // longitude  [0,2pi)
         double* latitude_radians // latitude   [-pi/2,pi/2]
         ) const;

  // returns point on sphere that is closest to given point
  ON_3dPoint ClosestPointTo( 
         ON_3dPoint test_point
         ) const;

  // For intersections see ON_Intersect();

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

  // parameterization of NURBS surface does not match sphere's transcendental paramaterization
  int GetNurbForm( ON_NurbsSurface& ) const; // returns 0=failure, 2=success

  /*
  Description:
    Creates a surface of revolution definition of the sphere.
  Parameters:
    bArcLengthParameterization - [in]
      true: 
        The domain will be set to (0,radius*2*pi)x(-radius*pi/2,radius*pi/2)
      false: 
        The domain will be set to (0,2*pi)x(-pi/2,pi/2)
    srf - [in]
      if not NULL, then this srf is used.
  Result:
    A surface of revolution or NULL if the sphere is not valid.
  */
  ON_RevSurface* RevSurfaceForm( bool bArcLengthParameterization, ON_RevSurface* srf = NULL ) const;
  ON_DEPRECATED ON_RevSurface* RevSurfaceForm( ON_RevSurface* srf = NULL ) const;
};

#endif

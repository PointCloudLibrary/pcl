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

#if !defined(ON_CONE_INC_)
#define ON_CONE_INC_

class ON_NurbsSurface;
class ON_Brep;

// Description:
//   Lightweight right circular cone.  Use ON_ConeSurface if
//   you need ON_Cone geometry as a virtual ON_Surface.
class ON_CLASS ON_Cone
{
public:

  // Creates a cone with world XY plane as the base plane,
  // center = (0,0,0), radius = 0.0, height = 0.0.
  ON_Cone();

  // See ON_Cone::Create.
  ON_Cone(
    const ON_Plane& plane,
    double height,
    double radius
    );

  ~ON_Cone();

  // Description:
  //   Creates a right circular cone from a plane, height,
  //   and radius.
  //  plane - [in] The apex of cone is at plane.origin and
  //      the axis of the cone is plane.zaxis.
  //  height - [in] The center of the base is height*plane.zaxis.
  //  radius - [in] tan(cone angle) = radius/height
  ON_BOOL32 Create(
    const ON_Plane& plane,
    double height,
    double radius
    );

  // Returns true if plane is valid, height is not zero, and
  // radius is not zero.
  ON_BOOL32 IsValid() const;

  // Returns:
  //   Center of base circle.
  // Remarks:
  //   The base point is plane.origin + height*plane.zaxis.
  ON_3dPoint BasePoint() const;

  // Returns:
  //   Point at the tip of the cone.
  // Remarks:
  //   The apex point is plane.origin.
  const ON_3dPoint& ApexPoint() const;

  // Returns:
  //   Unit vector axis of cone.
  const ON_3dVector& Axis() const;

  // Returns:
  //   The angle (in radians) between the axis and the 
  //   side of the cone.
  //   The angle and the height have the same sign.
  double AngleInRadians() const;

  // Returns:
  //   The angle Iin degrees) between the axis and the side.
  //   The angle and the height have the same sign.
  double AngleInDegrees() const;           

  // evaluate parameters and return point
  // Parameters:
  //   radial_parameter - [in] 0.0 to 2.0*ON_PI
  //   height_parameter - [in] 0 = apex, height = base
  ON_3dPoint PointAt(
    double radial_parameter,
    double height_parameter
    ) const;

  // Parameters:
  //   radial_parameter - [in] (in radians) 0.0 to 2.0*ON_PI
  //   height_parameter - [in] 0 = apex, height = base
  // Remarks:
  //   If radius>0 and height>0, then the normal points "out"
  //   when height_parameter >= 0.
  ON_3dVector NormalAt(
    double radial_parameter,
    double height_parameter
    ) const;

  // Description:
  //   Get iso curve circle at a specified height.
  // Parameters:
  //   height_parameter - [in] 0 = apex, height = base
  ON_Circle CircleAt( 
    double height_parameter
    ) const;

  // Description:
  //   Get iso curve line segment at a specified angle.
  // Parameters:
  //   radial_parameter - [in] (in radians) 0.0 to 2.0*ON_PI
  ON_Line LineAt( 
    double radial_parameter 
    ) const;

  // returns parameters of point on cone that is closest to given point
  bool ClosestPointTo( 
          ON_3dPoint point, 
          double* radial_parameter,
          double* height_parameter
         ) const;

  // returns point on cone that is closest to given point
  ON_3dPoint ClosestPointTo( 
         ON_3dPoint 
         ) const;

  ON_BOOL32 Transform( const ON_Xform& );

  // rotate cone about its origin
  ON_BOOL32 Rotate(
        double sin_angle,
        double cos_angle,
        const ON_3dVector& axis_of_rotation
        );

  ON_BOOL32 Rotate(
        double angle_in_radians,
        const ON_3dVector& axis_of_rotation
        );

  // rotate cone about a point and axis
  ON_BOOL32 Rotate(
        double sin_angle,
        double cos_angle,
        const ON_3dVector& axis_of_rotation,
        const ON_3dPoint& center_of_rotation
        );
  ON_BOOL32 Rotate(
        double angle_in_radians,
        const ON_3dVector& axis_of_rotation,
        const ON_3dPoint& center_of_rotation
        );

  ON_BOOL32 Translate(
        const ON_3dVector& delta
        );

  ON_BOOL32 GetNurbForm( ON_NurbsSurface& ) const;

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

public:
  ON_Plane plane; // apex = plane.origin, axis = plane.zaxis
  double   height; // not zero
  double   radius; // not zero
};

#endif

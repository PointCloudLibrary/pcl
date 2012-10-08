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

#if !defined(ON_INTERSECT_INC_)
#define ON_INTERSECT_INC_

// These simple intersectors are fast and detect transverse intersections.
// If the intersection is not a simple transverse case, then they
// return false and you will have to use one of the slower but fancier
// models.


/*
Description:
  Intersect two lines.
Parameters:
  lineA - [in]
  lineB - [in]
  double* a - [out]
  double* b - [out] The shortest distance between the lines is the
                   chord from lineA.PointAt(*a) to lineB.PointAt(*b).
  tolerance - [in] If > 0.0, then an intersection is reported only
                   if the distance between the points is <= tolerance.
                   If <= 0.0, then the closest point between the lines
                   is reported.
  bIntersectSegments - [in] if true, the input lines are treated
                           as finite segments.  If false, the
                           input lines are treated as infinite lines.
Returns:
  True if a closest point can be calculated and the result passes
  the tolerance parameter test.
See Also:
  ON_Intersect( const ON_Line& lineA, const ON_Line& line B)
Remarks:
  If the lines are exactly parallel, meaning the system of equations
  used to find a and b has no numerical solution, then false is returned.
  If the lines are nearly parallel, which is often numerically true
  even if you think the lines look exactly parallel, then the
  closest points are found and true is returned.  So, if you
  care about weeding out "parallel" lines, then you need to
  do something like the following.

          bool rc = ON_IntersectLineLine(lineA,lineB,
                                         &a,&b,
                                         tolerance,
                                         bIntersectSegments);
          if (rc)
          {
            double angle_tolerance_radians = 0.5*ON_PI/180.0; // or whatever
            double parallel_tol = cos(angle_tolerance_radians);
            if ( fabs(lineA.Tangent()*lineB.Tangent()) >= parallel_tol )
            {
              ... do whatever you think is appropriate
            }
          }
*/
ON_DECL
bool ON_IntersectLineLine(
          const ON_Line& lineA, 
          const ON_Line& lineB, 
          double* a,
          double* b,
          double tolerance,
          bool bIntersectSegments
          );

/*
Description:
  Find the closest point between two infinte lines.
Parameters:
  lineA - [in]
  lineB - [in]
  double* a - [out]
  double* b - [out] The shortest distance between the lines is the
                   chord from lineA.PointAt(*a) to lineB.PointAt(*b).
Returns:
  True if points are found and false if the lines are numerically parallel.
  Numerically parallel means the 2x2 matrix

            AoA  -AoB
           -AoB   BoB

 is numerically singluar, where A = lineA.to-lineA.from 
 and B = lineB.to-lineB.from.
See Also:
  ON_IntersectLineLine
*/
ON_DECL
bool ON_Intersect(
          const ON_Line& lineA, 
          const ON_Line& lineB, 
          double* a,
          double* b
          );

ON_DECL
bool ON_Intersect( // Returns false unless intersection is a single point
                   // If returned parameter is < 0 or > 1, then the line
                   // segment between line.m_point[0] and line.m_point[1]
                   // does not intersect the plane
          const ON_Line&, 
          const ON_Plane&, 
          double* // parameter on line
          );

ON_DECL
bool ON_Intersect( const ON_Plane&, 
                  const ON_Plane&, 
                   ON_Line& // intersection line is returned here
                   );

ON_DECL
bool ON_Intersect( const ON_Plane&, 
                  const ON_Plane&, 
                  const ON_Plane&,
                  ON_3dPoint& // intersection point is returned here
                  );

/*
Description:
  Intersect a plane and a sphere.
Parameters:
  plane - [in]
  sphere - [in]
  circle - [out]
Returns:
  0: no intersection
    circle radius = 0 and circle origin = point on the plane
    closest to the sphere.
  1: intersection is a single point
    circle radius = 0;
  2: intersection is a circle
    circle radius > 0.
*/
ON_DECL
int ON_Intersect(
          const ON_Plane& plane, 
          const ON_Sphere& sphere,
          ON_Circle& circle
          );

ON_DECL
int ON_Intersect( // returns 0 = no intersections, 
                  // 1 = one intersection, 
                  // 2 = 2 intersections
                  // If 0 is returned, first point is point 
                  // on line closest to sphere and 2nd point is the point
                  // on the sphere closest to the line.
                  // If 1 is returned, first point is obtained by evaluating
                  // the line and the second point is obtained by evaluating
                  // the sphere.
                 const ON_Line&, const ON_Sphere&,
                  ON_3dPoint&, ON_3dPoint& // intersection point(s) returned here
                  );

ON_DECL
int ON_Intersect( // returns 0 = no intersections, 
                  // 1 = one intersection, 
                  // 2 = 2 intersections
                  // 3 = line lies on cylinder
                  // If 0 is returned, first point is point 
                  // on line closest to cylinder and 2nd point is the point
                  // on the sphere closest to the line.
                  // If 1 is returned, first point is obtained by evaluating
                  // the line and the second point is obtained by evaluating
                  // the sphere.
                  const ON_Line&, const ON_Cylinder&,
                  ON_3dPoint&, ON_3dPoint& // intersection point(s) returned here
                  );

/*
Description:
  Intersect an infinite line and an axis aligned bounding box.
Parameters:
  bbox - [in]
  line - [in]
  tolerance - [in]  If tolerance > 0.0, then the intersection is
                   performed against a box that has each side
                   moved out by tolerance.
  line_parameters - [out] 
    Pass null if you do not need the parameters.
    If true is returned and line.from != line.to,
    then the chord from line.PointAt(line_parameters[0])
    to line.PointAt(line_parameters[1]) is the intersection.
    If true is returned and line.from = line.to, then line.from
    is in the box and the interval (0.0,0.0) is returned.
    If false is returned, the input value of line_parameters
    is not changed.
Returns:
  True if the line intersects the box and false otherwise.
*/
ON_DECL
bool ON_Intersect( const ON_BoundingBox& bbox, 
                   const ON_Line& line, 
                   double tolerance,
                   ON_Interval* line_parameters
                   );

/*
Description:
  Intersect two spheres using exact calculations.
Parameters:
  sphere0 - [in]
  sphere1 - [in]
  circle - [out] If intersection is a point, then that point will be the center, radius 0.
Returns:
  0 if no intersection,
  1 if a single point,
  2 if a circle,
  3 if the spheres are the same.
*/
ON_DECL
int ON_Intersect( const ON_Sphere& sphere0, 
                  const ON_Sphere& sphere1, 
                  ON_Circle& circle
                 );
#endif

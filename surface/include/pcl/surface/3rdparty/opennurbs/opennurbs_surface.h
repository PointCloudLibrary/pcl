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
//   Definition of virtual parametric surface
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_SURFACE_INC_)
#define OPENNURBS_SURFACE_INC_

class ON_Curve;
class ON_NurbsSurface;
class ON_SurfaceTree;

////////////////////////////////////////////////////////////////
//
//   Definition of virtual parametric surface
//
////////////////////////////////////////////////////////////////

class ON_Mesh;
class ON_MeshParameters;
class ON_PolyCurve;
class ON_CurveProxy;
class ON_Surface;

class ON_CLASS ON_Surface : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_Surface);

public:
  // virtual ON_Object::DestroyRuntimeCache override
  void DestroyRuntimeCache( bool bDelete = true );

  // pure virtual class for surface objects
public:

  // flags for isoparametric curves
  // note: odd values are all "x" = constant
  // and even values > 0 are all "y" = constant
  // ON_BrepTrim::m_iso uses these flags
  enum ISO 
  {
    not_iso = 0, // curve is not an isoparameteric curve
    x_iso   = 1, // curve is a "x" = constant (vertical) isoparametric
                 // curve in the interior of the surface's domain
    y_iso   = 2, // curve is a "y" = constant (horizontal) isoparametric
                 // curve in the interior of the surface's domain
    W_iso   = 3, // curve is a "x" = constant isoparametric curve
                 // along the west side of the surface's domain
    S_iso   = 4, // curve is a "y" = constant isoparametric curve
                 // along the south side of the surface's domain
    E_iso   = 5, // curve is a "x" = constant isoparametric curve
                 // along the east side of the surface's domain
    N_iso   = 6, // curve is a "y" = constant isoparametric curve
                 // along the north side of the surface's domain
    iso_count = 7
  };

public:
  ON_Surface();
  ON_Surface(const ON_Surface&);
  ON_Surface& operator=(const ON_Surface&);
  virtual ~ON_Surface();

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Geometry override
  bool EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const;

  /*
  Description:
    Get a duplicate of the surface.
  Returns:
    A duplicate of the surface.  
  Remarks:
    The caller must delete the returned surface.
    For non-ON_SurfaceProxy objects, this simply duplicates the surface using
    ON_Object::Duplicate.
    For ON_SurfaceProxy objects, this duplicates the actual proxy surface 
    geometry and, if necessary, transposes the result to that
    the returned surfaces's parameterization and locus match the proxy surface's.
  */
  virtual
  ON_Surface* DuplicateSurface() const;

  //////////
  // override ON_Object::ObjectType() - returns ON::surface_object
  ON::object_type ObjectType() const;


  /////////////////////////////
  //
  // virtual ON_Geometry functions
  //

  /*
  Description:
    Overrides virtual ON_Geometry::HasBrepForm and returns true.
  Result:
    Returns true.
  See Also:
    ON_Brep::Create( ON_Surface&* )
  */
  ON_BOOL32 HasBrepForm() const;

  /*
  Description:
    Overrides virtual ON_Geometry::HasBrepForm.  
    Uses ON_Brep::Create( ON_Surface&* ) to create a brep
    form.  The surface is copied for use in the returned
    brep.
  Parameters:
    brep - [in] if not NULL, brep is used to store the brep
        form of the surface.
  Result:
    Returns a pointer to on ON_Brep or NULL.  If the brep
    parameter is not NULL, then brep is returned if the
    surface has a brep form and NULL is returned if the
    geometry does not have a brep form.
  Remarks:
    The caller is responsible for managing the brep memory.
  */
  ON_Brep* BrepForm( ON_Brep* brep = NULL ) const;

  ////////////////////////////////////////////////////////////////////
  // surface interface

  ON_BOOL32 GetDomain( 
         int dir,              // 0 gets first parameter, 1 gets second parameter
         double* t0,
         double* t1
         ) const;

  bool SetDomain( 
    int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
    ON_Interval domain
    );

  virtual
  ON_BOOL32 SetDomain( 
    int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
    double t0, 
    double t1
    );

  virtual
  ON_Interval Domain(
    int dir // 0 gets first parameter's domain, 1 gets second parameter's domain
    ) const = 0;

  /*
  Description:
    Get an estimate of the size of the rectangle that would
    be created if the 3d surface where flattened into a rectangle.
  Parameters:
    width - [out]  (corresponds to the first surface parameter)
    height - [out] (corresponds to the first surface parameter)
  Example:

          // Reparameterize a surface to minimize distortion 
          // in the map from parameter space to 3d.
          ON_Surface* surf = ...;
          double width, height;
          if ( surf->GetSurfaceSize( &width, &height ) )
          {
            srf->SetDomain( 0, ON_Interval( 0.0, width ) );
            srf->SetDomain( 1, ON_Interval( 0.0, height ) );
          }

  Returns:
    true if successful.
  */
  virtual
  ON_BOOL32 GetSurfaceSize( 
      double* width, 
      double* height 
      ) const;


  virtual 
  int SpanCount(
    int dir // 0 gets first parameter's domain, 1 gets second parameter's domain
    ) const = 0; // number of smooth nonempty spans in the parameter direction

  virtual
  ON_BOOL32 GetSpanVector( // span "knots" 
        int dir, // 0 gets first parameter's domain, 1 gets second parameter's domain
        double* span_vector // array of length SpanCount() + 1 
        ) const = 0; // 

  //////////
  // If t is in the domain of the surface, GetSpanVectorIndex() returns the 
  // span vector index "i" such that span_vector[i] <= t <= span_vector[i+1].
  // The "side" parameter determines which span is selected when t is at the
  // end of a span.
  virtual
  ON_BOOL32 GetSpanVectorIndex(
        int dir , // 0 gets first parameter's domain, 1 gets second parameter's domain
        double t,      // [IN] t = evaluation parameter
        int side,         // [IN] side 0 = default, -1 = from below, +1 = from above
        int* span_vector_index,        // [OUT] span vector index
        ON_Interval* span_interval // [OUT] domain of the span containing "t"
        ) const;

  virtual 
  int Degree( // returns maximum algebraic degree of any span 
                  // ( or a good estimate if curve spans are not algebraic )
    int dir // 0 gets first parameter's domain, 1 gets second parameter's domain
    ) const = 0; 

  virtual ON_BOOL32 GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
         int dir,        // 0 gets first parameter, 1 gets second parameter
         double t,       // t = parameter in domain
         double* tminus, // tminus
         double* tplus   // tplus
         ) const;

  /*
  Description:
    Test a 2d curve to see if it is iso parameteric in the surface's
    parameter space.
  Parameters:
    curve - [in] curve to test
    curve_domain = [in] optional sub domain of the curve
  Returns:
    Isoparametric status of the curve.
  Remarks:
    Because it may transpose domains, ON_SurfaceProxy overrides
    this function.  All other surface classes just use
    the base class implementation.
  */
  virtual
  ISO IsIsoparametric(
        const ON_Curve& curve,
        const ON_Interval* curve_domain = NULL
        ) const;

  /*
  Description:
    Test a 2d bounding box to see if it is iso parameteric in the surface's
    parameter space.
  Parameters:
    bbox - [in] bounding box to test
  Returns:
    Isoparametric status of the bounding box.
  Remarks:
    Because it may transpose domains, ON_SurfaceProxy overrides
    this function.  All other surface classes just use
    the base class implementation.
  */
  virtual
  ISO IsIsoparametric(
        const ON_BoundingBox& bbox
        ) const;

  /*
  Description:
    Test a surface to see if it is planar.
  Parameters:
    plane - [out] if not NULL and true is returned,
                  the plane parameters are filled in.
    tolerance - [in] tolerance to use when checking
  Returns:
    true if there is a plane such that the maximum distance from
    the surface to the plane is <= tolerance.
  */
  virtual
  ON_BOOL32 IsPlanar(
        ON_Plane* plane = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  /*
  Description:
    Determine if the surface is a portion of a sphere.
  Parameters:
    sphere - [out] if not NULL and true is returned,
      then the sphere definition is returned.
    tolerance - [in]
      tolerance to use when checking
  Returns:
    True if the surface is a portion of a sphere.                   
  */
  bool IsSphere(
        ON_Sphere* sphere = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  /*
  Description:
    Determine if the surface is a portion of a cylinder.
  Parameters:
    cylinder - [out] if not NULL and true is returned, 
      then the cylinder definition is returned.
    tolerance - [in]
      tolerance to use when checking
  Returns:
    True if the surface is a portion of a cylinder.                   
  */
  bool IsCylinder(
        ON_Cylinder* cylinder = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  /*
  Description:
    Determine if the surface is a portion of a cone.
  Parameters:
    cone - [out] if not NULL and true is returned, 
      then the cone definition is returned.
    tolerance - [in]
      tolerance to use when checking
  Returns:
    True if the surface is a portion of a cone.                   
  */
  bool IsCone(
        ON_Cone* cone = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  /*
  Description:
    Determine if the surface is a portion of a torus.
  Parameters:
    torus - [out] if not NULL and true is returned,
      then the torus definition is returned.
    tolerance - [in]
      tolerance to use when checking
  Returns:
    True if the surface is a portion of a torus.                   
  */
  bool IsTorus(
        ON_Torus* torus = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  virtual 
  ON_BOOL32 IsClosed(   // true if surface is closed in direction
        int        // dir  0 = "s", 1 = "t"
        ) const;

  virtual 
  ON_BOOL32 IsPeriodic( // true if surface is periodic in direction (default is false)
        int        // dir  0 = "s", 1 = "t"
        ) const;

  virtual
  ON_BOOL32 IsSingular( // true if surface side is collapsed to a point
        int        // side of parameter space to test
                   // 0 = south, 1 = east, 2 = north, 3 = west
        ) const;

  /*
  Returns:
    True if the surface defines a solid, like a sphere or torus.
    False if the surface does not define a solid, like a plane or cone.
  */
  bool IsSolid() const;

  /*
  Description:
    Test if a surface parameter value is at a singularity.
  Parameters:
    s - [in] surface parameter to test
    t - [in] surface parameter to test
    bExact - [in] if true, test if s,t is exactly at a singularity
      if false, test if close enough to cause numerical problems.
  Returns:
    true if surface is singular at (s,t)
  */
  bool IsAtSingularity(
    double s, 
    double t, 
    bool bExact = true
    ) const;

  /*
  Description:
    Test if a surface parameter value is at a seam.
  Parameters:
    s - [in] surface parameter to test
    t - [in] surface parameter to test
  Returns:
    0 if not a seam,
    1 if s == Domain(0)[i] and srf(s, t) == srf(Domain(0)[1-i], t)
    2 if t == Domain(1)[i] and srf(s, t) == srf(s, Domain(1)[1-i])
    3 if 1 and 2 are true.
  */
  int IsAtSeam(
    double s,
    double t
    ) const;
  
  /*
  Description:
    Search for a derivatitive, tangent, or curvature 
    discontinuity.
  Parameters:
    dir - [in] If 0, then "u" parameter is checked.  If 1, then
               the "v" parameter is checked.
    c - [in] type of continity to test for.
    t0 - [in] Search begins at t0. If there is a discontinuity
              at t0, it will be ignored.  This makes it 
              possible to repeatedly call GetNextDiscontinuity
              and step through the discontinuities.
    t1 - [in] (t0 != t1)  If there is a discontinuity at t1 is 
              will be ingored unless c is a locus discontinuity
              type and t1 is at the start or end of the curve.
    t - [out] if a discontinuity is found, then *t reports the
          parameter at the discontinuity.
    hint - [in/out] if GetNextDiscontinuity will be called 
       repeatedly, passing a "hint" with initial value *hint=0
       will increase the speed of the search.       
    dtype - [out] if not NULL, *dtype reports the kind of 
        discontinuity found at *t.  A value of 1 means the first 
        derivative or unit tangent was discontinuous.  A value 
        of 2 means the second derivative or curvature was 
        discontinuous.  A value of 0 means teh curve is not
        closed, a locus discontinuity test was applied, and
        t1 is at the start of end of the curve.
    cos_angle_tolerance - [in] default = cos(1 degree) Used only
        when c is ON::G1_continuous or ON::G2_continuous.  If the
        cosine of the angle between two tangent vectors is 
        <= cos_angle_tolerance, then a G1 discontinuity is reported.
    curvature_tolerance - [in] (default = ON_SQRT_EPSILON) Used 
        only when c is ON::G2_continuous.  If K0 and K1 are 
        curvatures evaluated from above and below and 
        |K0 - K1| > curvature_tolerance, then a curvature 
        discontinuity is reported.
  Returns:
    Parametric continuity tests c = (C0_continuous, ..., G2_continuous):

      true if a parametric discontinuity was found strictly 
      between t0 and t1. Note well that all curves are 
      parametrically continuous at the ends of their domains.

    Locus continuity tests c = (C0_locus_continuous, ...,G2_locus_continuous):

      true if a locus discontinuity was found strictly between
      t0 and t1 or at t1 is the at the end of a curve.
      Note well that all open curves (IsClosed()=false) are locus
      discontinuous at the ends of their domains.  All closed 
      curves (IsClosed()=true) are at least C0_locus_continuous at 
      the ends of their domains.
  */
  virtual
  bool GetNextDiscontinuity( 
                  int dir,
                  ON::continuity c,
                  double t0,
                  double t1,
                  double* t,
                  int* hint=NULL,
                  int* dtype=NULL,
                  double cos_angle_tolerance=ON_DEFAULT_ANGLE_TOLERANCE_COSINE,
                  double curvature_tolerance=ON_SQRT_EPSILON
                  ) const;

  /*
  Description:
    Test continuity at a surface parameter value.
  Parameters:
    c - [in] continuity to test for
    s - [in] surface parameter to test
    t - [in] surface parameter to test
    hint - [in] evaluation hint
    point_tolerance - [in] if the distance between two points is
        greater than point_tolerance, then the surface is not C0.
    d1_tolerance - [in] if the difference between two first derivatives is
        greater than d1_tolerance, then the surface is not C1.
    d2_tolerance - [in] if the difference between two second derivatives is
        greater than d2_tolerance, then the surface is not C2.
    cos_angle_tolerance - [in] default = cos(1 degree) Used only when
        c is ON::G1_continuous or ON::G2_continuous.  If the cosine
        of the angle between two normal vectors 
        is <= cos_angle_tolerance, then a G1 discontinuity is reported.
    curvature_tolerance - [in] (default = ON_SQRT_EPSILON) Used only when
        c is ON::G2_continuous.  If K0 and K1 are curvatures evaluated
        from above and below and |K0 - K1| > curvature_tolerance,
        then a curvature discontinuity is reported.
  Returns:
    true if the surface has at least the c type continuity at the parameter t.
  */
  virtual
  bool IsContinuous(
    ON::continuity c,
    double s, 
    double t, 
    int* hint = NULL,
    double point_tolerance=ON_ZERO_TOLERANCE,
    double d1_tolerance=ON_ZERO_TOLERANCE,
    double d2_tolerance=ON_ZERO_TOLERANCE,
    double cos_angle_tolerance=ON_DEFAULT_ANGLE_TOLERANCE_COSINE,
    double curvature_tolerance=ON_SQRT_EPSILON
    ) const;

  virtual 
  ON_BOOL32 Reverse(  // reverse parameterizatrion, Domain changes from [a,b] to [-b,-a]
    int // dir  0 = "s", 1 = "t"
    ) = 0;

  virtual 
  ON_BOOL32 Transpose() = 0; // transpose surface parameterization (swap "s" and "t")

  // simple evaluation interface - no error handling
  ON_3dPoint  PointAt( double, double ) const;
  ON_3dVector NormalAt( double, double ) const;
  ON_BOOL32 FrameAt( double u, double v, ON_Plane& frame) const;

  ON_BOOL32 EvPoint( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters
         ON_3dPoint& point,    // returns value of surface
         int quadrant = 0,     // optional - determines which side to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* hint = 0         // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const;

  ON_BOOL32 Ev1Der( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters (s,t)
         ON_3dPoint& point,    // returns value of surface
         ON_3dVector& du,      // first partial derivatives (Ds)
         ON_3dVector& dv,      // (Dt)
         int quadrant = 0,     // optional - determines which side to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* hint = 0         // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const;

  ON_BOOL32 Ev2Der( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters (s,t)
         ON_3dPoint& point,    // returns value of surface
         ON_3dVector& du,      // first partial derivatives (Ds)
         ON_3dVector& dv,      // (Dt)
         ON_3dVector& duu,     // second partial derivatives (Dss)
         ON_3dVector& duv,     // (Dst)
         ON_3dVector& dvv,     // (Dtt)
         int quadrant= 0,      // optional - determines which side to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* hint = 0         // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const;

  ON_BOOL32 EvNormal( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters (s,t)
         ON_3dPoint& point,    // returns value of surface
         ON_3dVector& normal,  // unit normal
         int quadrant = 0,     // optional - determines which side to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* hint = 0         // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const;

  ON_BOOL32 EvNormal( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters (s,t)
         ON_3dVector& normal,  // unit normal
         int quadrant = 0,     // optional - determines which side to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* hint = 0         // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const;

  ON_BOOL32 EvNormal( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters (s,t)
         ON_3dPoint& point,    // returns value of surface
         ON_3dVector& du,      // first partial derivatives (Ds)
         ON_3dVector& dv,      // (Dt)
         ON_3dVector& normal,  // unit normal
         int = 0,              // optional - determines which side to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* = 0              // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const;

  // work horse evaluator
  virtual 
  ON_BOOL32 Evaluate( // returns false if unable to evaluate
         double u, double v,   // evaluation parameters
         int num_der,          // number of derivatives (>=0)
         int array_stride,     // array stride (>=Dimension())
         double* der_array,    // array of length stride*(ndir+1)*(ndir+2)/2
         int quadrant = 0,     // optional - determines which quadrant to evaluate from
                               //         0 = default
                               //         1 from NE quadrant
                               //         2 from NW quadrant
                               //         3 from SW quadrant
                               //         4 from SE quadrant
         int* hint = 0         // optional - evaluation hint (int[2]) used to speed
                               //            repeated evaluations
         ) const = 0;

  /*
  Description:
    Get isoparametric curve.
  Parameters:
    dir - [in] 0 first parameter varies and second parameter is constant
                 e.g., point on IsoCurve(0,c) at t is srf(t,c)
                 This is a horizontal line from left to right
               1 first parameter is constant and second parameter varies
                 e.g., point on IsoCurve(1,c) at t is srf(c,t
                 This is a vertical line from bottom to top

    c - [in] value of constant parameter 
  Returns:
    Isoparametric curve.
  Remarks:
    In this function "dir" indicates which direction the resulting
    curve runs.  0: horizontal, 1: vertical
    In the other ON_Surface functions that take a "dir"
    argument, "dir" indicates if "c" is a "u" or "v" parameter.
  */
  virtual
  ON_Curve* IsoCurve(
         int dir,
         double c
         ) const;

  /*
  Description:
    Removes the portions of the surface outside of the specified interval.

  Parameters:
    dir - [in] 0  The domain specifies an sub-interval of Domain(0)
                  (the first surface parameter).
               1  The domain specifies an sub-interval of Domain(1)
                  (the second surface parameter).
    domain - [in] interval of the surface to keep. If dir is 0, then
        the portions of the surface with parameters (s,t) satisfying
        s < Domain(0).Min() or s > Domain(0).Max() are trimmed away.
        If dir is 1, then the portions of the surface with parameters
        (s,t) satisfying t < Domain(1).Min() or t > Domain(1).Max() 
        are trimmed away.
  */
  virtual
  ON_BOOL32 Trim(
         int dir,
         const ON_Interval& domain
         );

  /*
   Description:
     Pure virtual function. Default returns false.
     Where possible, analytically extends surface to include domain.
   Parameters:
     dir - [in] 0  new Domain(0) will include domain.
                   (the first surface parameter).
                1  new Domain(1) will include domain.
                   (the second surface parameter).
     domain - [in] if domain is not included in surface domain, 
     surface will be extended so that its domain includes domain.  
     Will not work if surface is closed in direction dir. 
     Original surface is identical to the restriction of the
     resulting surface to the original surface domain, 
   Returns:
     true if successful.
     */
  virtual
  bool Extend(
    int dir,
    const ON_Interval& domain
    );


  /*
  Description:
    Splits (divides) the surface into two parts at the 
    specified parameter.

  Parameters:
    dir - [in] 0  The surface is split vertically.  The "west" side
                  is returned in "west_or_south_side" and the "east"
                  side is returned in "east_or_north_side".
               1  The surface is split horizontally.  The "south" side
                  is returned in "west_or_south_side" and the "north"
                  side is returned in "east_or_north_side".
    c - [in] value of constant parameter in interval returned
               by Domain(dir)
    west_or_south_side - [out] west/south portion of surface returned here
    east_or_north_side - [out] east/north portion of surface returned here

  Example:

          ON_NurbsSurface srf = ...;
          int dir = 1;
          ON_NurbsSurface* south_side = 0;
          ON_NurbsSurface* north_side = 0;
          srf.Split( dir, srf.Domain(dir).Mid() south_side, north_side );

  */
  virtual
  ON_BOOL32 Split(
         int dir,
         double c,
         ON_Surface*& west_or_south_side,
         ON_Surface*& east_or_north_side
         ) const;

  /*
  Description:
    Get a NURBS surface representation of this surface.
  Parameters:
    nurbs_surface - [out] NURBS representation returned here
    tolerance - [in] tolerance to use when creating NURBS
        representation.
    s_subdomain - [in] if not NULL, then the NURBS representation
        for this portion of the surface is returned.
    t_subdomain - [in] if not NULL, then the NURBS representation
        for this portion of the surface is returned.
  Returns:
    0   unable to create NURBS representation
        with desired accuracy.
    1   success - returned NURBS parameterization
        matches the surface's to wthe desired accuracy
    2   success - returned NURBS point locus matches
        the surface's to the desired accuracy and the
        domain of the NURBS surface is correct.  On
        However, This surface's parameterization and
        the NURBS surface parameterization may not 
        match to the desired accuracy.  This situation
        happens when getting NURBS representations of
        surfaces that have a transendental parameterization
        like spheres, cylinders, and cones.
  Remarks:
    This is a low-level virtual function.  If you do not need
    the parameterization information provided by the return code,
    then ON_Surface::NurbsSurface may be easier to use.
  See Also:
    ON_Surface::NurbsSurface
  */
  virtual
  int GetNurbForm(
        ON_NurbsSurface& nurbs_surface,
        double tolerance = 0.0
        ) const;


  /*
  Description:
    Is there a NURBS surface representation of this surface.
  Parameters:
  Returns:
    0   unable to create NURBS representation
        with desired accuracy.
    1   success - NURBS parameterization
        matches the surface's
    2   success - NURBS point locus matches
        the surface's and the
        domain of the NURBS surface is correct.  
        However, This surface's parameterization and
        the NURBS surface parameterization may not 
        match.  This situation
        happens when getting NURBS representations of
        surfaces that have a transendental parameterization
        like spheres, cylinders, and cones.
  Remarks:
    This is a low-level virtual function. 
  See Also:
    ON_Surface::GetNurbForm
    ON_Surface::NurbsSurface
  */
  virtual
  int HasNurbForm() const;

  // Description:
  //   Get a NURBS surface representation of this surface.
  // Parameters:
  //   pNurbsSurface - [in/out] if not NULL, this pNurbsSurface
  //   will be used to store the NURBS representation
  //   of the surface and will be returned.
  //   tolerance - [in] tolerance to use when creating NURBS
  //       surface representation.
  //   s_subdomain - [in] if not NULL, then the NURBS representation
  //       for this portion of the surface is returned.
  //   t_subdomain - [in] if not NULL, then the NURBS representation
  //       for this portion of the surface is returned.
  // Returns:
  //   NULL or a NURBS representation of the surface.
  // Remarks:
  //   See ON_Surface::GetNurbForm for important details about
  //   the NURBS surface parameterization.
  // See Also:
  //   ON_Surface::GetNurbForm
  ON_NurbsSurface* NurbsSurface(
        ON_NurbsSurface* pNurbsSurface = NULL,
        double tolerance = 0.0,
        const ON_Interval* s_subdomain = NULL,
        const ON_Interval* t_subdomain = NULL
        ) const;

  virtual
  bool GetSurfaceParameterFromNurbFormParameter(
        double nurbs_s, double nurbs_t,
        double* surface_s, double* surface_t
        ) const;

  virtual
  bool GetNurbFormParameterFromSurfaceParameter(
        double surface_s, double surface_t,
        double* nurbs_s,  double* nurbs_t
        ) const;


  // If the geometry surface is modified in any way, then
  // call DestroySurfaceTree().
  void DestroySurfaceTree();
};

class ON_CLASS ON_SurfaceProperties
{
  // Surface properties
public:
  // The constructor sets all fields to zero.
  ON_SurfaceProperties();

  /*
  Parameters:
    surface - [in]
      If surface is not null, then it is used to set the surface properties.
      If surface is null, then all surface properties are set to to zero.
  Remarks:
    Does not modify the value of m_tag.
  */
  void Set( const ON_Surface* surface );

  bool m_bIsSet;           // True if Set() has been callled with a non-null surface.

  bool m_bHasSingularity;  // true if at least one m_bSingular[] setting is true.
  bool m_bIsSingular[4];   // m_bSingular[i] = ON_Surface::IsSingular(i)

  bool m_bHasSeam;         // true if at least one m_bClosed[] setting is true.
  bool m_bIsClosed[2];     // m_bClosed[i] = ON_Surface::IsClosed(i)

private:
  bool m_bReserved[7];

public:
  ON_Interval m_domain[2]; // m_domain[i] = ON_Surface.Domain(i)

private:
  unsigned char m_reserved[16];

public:
  // Last pointer passed to ON_SurfaceProperties::Set().
  const ON_Surface* m_surface;

  // The constructor sets this value to zero.
  // Nothing in opennurbs modifies or uses this value.
  ON__INT_PTR m_tag;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_Surface*>;
#pragma warning( pop )
#endif

class ON_CLASS ON_SurfaceArray : public ON_SimpleArray<ON_Surface*>
{
public:
  ON_SurfaceArray( int = 0 );
  ~ON_SurfaceArray();

  ON_BOOL32 Write( ON_BinaryArchive& ) const;
  ON_BOOL32 Read( ON_BinaryArchive& );

  void Destroy(); // deletes surfaces in array and sets count to 0

  ON_BOOL32 Duplicate( ON_SurfaceArray& ) const; // operator= copies the pointer values
                                     // duplicate copies the surfaces themselves
};


#endif

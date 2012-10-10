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
//   Definition of NURBS surface
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_NURBSSURFACE_INC_)
#define OPENNURBS_NURBSSURFACE_INC_

#include <pcl/pcl_exports.h>

class ON_CLASS ON_TensorProduct
{
  // Pure virtual tensor passed to ON_NurbsSurface::TensorProduct()
public:
  ON_TensorProduct();

  virtual
  ~ON_TensorProduct();

  // Evaluate() must define a function T:R^dimA X R^dimB -> R^Dimension()
  // such that
  //
  //          T(a*A0 + (1-a)*A1, B) = a*T(A0,B) + (1-a)*T(A1,B) and
  //          T(A, b*B0 + (1-b)*B1) = b*T(A,B0) + (1-b)*T(A,B1).
  virtual
  int DimensionA() const = 0; // dimension of A space

  virtual
  int DimensionB() const = 0; // dimension of B space

  virtual
  int DimensionC() const = 0; // dimension of range space

  virtual
  bool Evaluate( double,        // a
                 const double*, // A
                 double,        // b
                 const double*, // B
                 double*        // C
                ) = 0;

};

class ON_Brep;
class ON_NurbsSurface;

class PCL_EXPORTS ON_CLASS ON_NurbsSurface : public ON_Surface
{
  ON_OBJECT_DECLARE(ON_NurbsSurface);

public:
  /*
  Description:
    Use ON_NurbsSurface::New(...) instead of new ON_NurbsSurface(...)
  Returns:
    Pointer to an ON_NurbsSurface.  Destroy by calling delete.
  Remarks:
    See static ON_Brep* ON_Brep::New() for details.
  */
  static ON_NurbsSurface* New();
  static ON_NurbsSurface* New(
          const ON_NurbsSurface& nurbs_surface
          );
  static ON_NurbsSurface* New(
            const ON_BezierSurface& bezier_surface 
            );
  static ON_NurbsSurface* New(
          int dimension,
          ON_BOOL32 bIsRational,
          int order0,
          int order1,
          int cv_count0,
          int cv_count1
          );

  ON_NurbsSurface();
  ON_NurbsSurface(const ON_NurbsSurface& nurbs_surface);
  ON_NurbsSurface(const ON_BezierSurface& bezier_surface);
  ON_NurbsSurface(
          int dimension,     // dimension (>= 1)
          ON_BOOL32 bIsRational, // true to make a rational NURBS
          int order0,       // order0 (>= 2)
          int order1,       // order1 (>= 2)
          int cv_count0,    // cv count0 (>= order0)
          int cv_count1     // cv count1 (>= order1)
          );

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  /*
  Description:
    See if this and other are same NURBS geometry.
  Parameters:
    other - [in] other NURBS surface
    bIgnoreParameterization - [in] if true, parameterization
             and orientaion are ignored.
    tolerance - [in] tolerance to use when comparing
                     control points.
  Returns:
    true if curves are tne same.
  */
  bool IsDuplicate( 
          const ON_NurbsSurface& other, 
          bool bIgnoreParameterization,
          double tolerance = ON_ZERO_TOLERANCE 
          ) const;

  void Initialize(void);  // zeros all fields

  ON_BOOL32 Create( 
          int dim,  // dimension (>= 1)
          ON_BOOL32 is_rat, // true to make a rational NURBS
          int order0,  // order0 (>= 2)
          int order1,  // order1 (>= 2)
          int cv_count0,  // cv count0 (>= order0)
          int cv_count1  // cv count1 (>= order1)
          );

  /*
  Description:
    Create a ruled surface from two curves.
  Parameters:
    curveA - [in] (must have same NURBS form knots as curveB)
    curveB - [in] (must have same NURBS form knots as curveA)
    curveA_domain - [in] if not NULL, then this is a subdomain
       of curveA to use for the ruled surface.
    curveB_domain - [in] if not NULL, then this is a subdomain
       of curveA to use for the ruled surface.
  Returns:
    @untitled table
    0      failure
    1      success - parameterization is exact
    2      success - parameterization is not exact
  Remarks:
    The ruling parameter is the second surface parameter and
    it is in the interval [0,1].
    The true ruled surface has parameterization 
    srf(s,t) = (1.0-t)*curveA(s) + t*curveB(s).
    The returned NURBS surface has parameterization
    srf(s,t) = (1.0-t)*nurbs_curveA(s) + t*nurbs_curveB(s),
    where nurbs_curveX is the NURBS form of curveX.  If the
    parameterization of nurbs_curveX does not match the 
    parameterization of curveX, then 2 is returned.
  */
  virtual
  int CreateRuledSurface(
         const ON_Curve& curveA,
         const ON_Curve& curveB,
         const ON_Interval* curveA_domain = NULL,
         const ON_Interval* curveB_domain = NULL
         );

  /*
  Description:
    Create a cone surface from a curve to a point.
  Parameters:
    apex_point - [in]
    curve - [in]
    curve_domain - [in] if not NULL, then this is a subdomain
       of curve to use for the ruled surface.
  Returns:
    @untitled table
    0      failure
    1      success - parameterization is exact
    2      success - parameterization is not exact
  Remarks:
    The ruling parameter is the second surface parameter and
    it is in the interval [0,1].
    The true cone surface has parameterization 
    srf(s,t) = (1.0-t)*curve(s) + t*apex_point.
    The returned NURBS surface has parameterization
    srf(s,t) = (1.0-t)*nurbs_curve(s) + t*apex_point,
    where nurbs_curve is the NURBS form of curve.  If the
    parameterization of nurbs_curve does not match the 
    parameterization of curve, then 2 is returned.
  */
  int CreateConeSurface(
         ON_3dPoint apex_point,
         const ON_Curve& curve,
         const ON_Interval* curve_domain = NULL
         );

  /*
  Description:
    Collapse the side of a NURBS surface to a single point.
  Parameters:
    side - [in] 0 = south west, 
                1 = south east, 
                2 = north east,
                3 = north west
    point - [in] point to collapse to.  If point is ON_unset_point,
                the the current location of the start of the side
                is used.
  Returns:
    True if successful.
  Remarks:
    If the surface is rational, the weights of the side control
    points must be set before calling CollapseSide.
  */
  bool CollapseSide(
    int side,
    ON_3dPoint point = ON_unset_point
    );

  void Destroy();

  virtual ~ON_NurbsSurface();

  void EmergencyDestroy(); // call if memory used by this class becomes invalid

  ON_NurbsSurface& operator=(const ON_NurbsSurface&);
  
  /*
  Description:
    Set NURBS surface equal to bezier surface with domain [0,1]x[0,1].
  Parameters:
    bezier_surface - [in]
  */
	ON_NurbsSurface& operator=(
    const ON_BezierSurface& bezier_surface
    );

  /////////////////////////////////////////////////////////////////
  // ON_Object overrides

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const; // for debugging

  ON_BOOL32 Write(
         ON_BinaryArchive&  // open binary file
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // open binary file
       );

  /////////////////////////////////////////////////////////////////
  // ON_Geometry overrides

  int Dimension() const;

  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // minimum
         double*,    // maximum
         ON_BOOL32 = false  // true means grow box
         ) const;

  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  ON_BOOL32 SwapCoordinates(
        int, int        // indices of coords to swap
        );

  /////////////////////////////////////////////////////////////////
  // ON_Surface overrides

  ON_BOOL32 SetDomain( 
    int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
    double t0, 
    double t1
    );

  ON_Interval Domain(
    int // 0 gets first parameter's domain, 1 gets second parameter's domain
    ) const;


  /*
  Description:
    Get an estimate of the size of the rectangle that would
    be created if the 3d surface where flattened into a rectangle.
  Parameters:
    width - [out]  (corresponds to the first surface parameter)
    height - [out] (corresponds to the first surface parameter)
  Remarks:
    overrides virtual ON_Surface::GetSurfaceSize
  Returns:
    true if successful.
  */
  ON_BOOL32 GetSurfaceSize( 
      double* width, 
      double* height 
      ) const;

  int SpanCount(
    int // 0 gets first parameter's domain, 1 gets second parameter's domain
    ) const; // number of smooth spans in curve

  ON_BOOL32 GetSpanVector( // span "knots" 
    int, // 0 gets first parameter's domain, 1 gets second parameter's domain
    double* // array of length SpanCount() + 1 
    ) const; // 

  int Degree( // returns maximum algebraic degree of any span 
                  // ( or a good estimate if curve spans are not algebraic )
    int // 0 gets first parameter's domain, 1 gets second parameter's domain
    ) const; 

  ON_BOOL32 GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
         int,     // 0 gets first parameter, 1 gets second parameter
         double,  // t = parameter in domain
         double*, // tminus
         double*  // tplus
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
  Remarks:
    Overrides virtual ON_Surface::IsPlanar.
  */
  ON_BOOL32 IsPlanar(
        ON_Plane* plane = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  ON_BOOL32 IsClosed(   // true if NURBS surface is closed (either surface has
        int // dir // clamped end knots and euclidean location of start
        ) const;   // CV = euclidean location of end CV, or surface is
                   // periodic.)

  ON_BOOL32 IsPeriodic( // true if NURBS surface is periodic (degree > 1,
        int // dir // periodic knot vector, last degree many CVs 
        ) const;   // are duplicates of first degree many CVs.)
  
  ON_BOOL32 IsSingular( // true if surface side is collapsed to a point
        int        // side of parameter space to test
                   // 0 = south, 1 = east, 2 = north, 3 = west
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
  Remarks:
    Overrides virtual ON_Surface::IsContinuous
  */
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

  ON_BOOL32 Reverse(  // reverse parameterizatrion, Domain changes from [a,b] to [-b,-a]
    int // dir  0 = "s", 1 = "t"
    );

  ON_BOOL32 Transpose(); // transpose surface parameterization (swap "s" and "t")

  ON_BOOL32 Evaluate( // returns false if unable to evaluate
         double, double, // evaluation parameter
         int,            // number of derivatives (>=0)
         int,            // array stride (>=Dimension())
         double*,        // array of length stride*(ndir+1)*(ndir+2)/2
         int = 0,        // optional - determines which quadrant to evaluate from
                         //         0 = default
                         //         1 from NE quadrant
                         //         2 from NW quadrant
                         //         3 from SW quadrant
                         //         4 from SE quadrant
         int* = 0        // optional - evaluation hint (int[2]) used to speed
                         //            repeated evaluations
         ) const;

  /*
  Description:
    Get isoparametric curve.
    Overrides virtual ON_Surface::IsoCurve.
  Parameters:
    dir - [in] 0 first parameter varies and second parameter is constant
                 e.g., point on IsoCurve(0,c) at t is srf(t,c)
               1 first parameter is constant and second parameter varies
                 e.g., point on IsoCurve(1,c) at t is srf(c,t)

    c - [in] value of constant parameter 
  Returns:
    Isoparametric curve.
  */
  ON_Curve* IsoCurve(
         int dir,
         double c
         ) const;

  /*
  Description:
    Removes the portions of the surface outside of the specified interval.
    Overrides virtual ON_Surface::Trim.

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
  ON_BOOL32 Trim(
         int dir,
         const ON_Interval& domain
         );

  /*
   Description:
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
  bool Extend(
    int dir,
    const ON_Interval& domain
    );


  /*
  Description:
    Splits (divides) the surface into two parts at the 
    specified parameter.
    Overrides virtual ON_Surface::Split.

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
  ON_BOOL32 Split(
         int dir,
         double c,
         ON_Surface*& west_or_south_side,
         ON_Surface*& east_or_north_side
         ) const;

  /*
  Description:
    Offset surface.
  Parameters:
    offset_distance - [in] offset distance
    tolerance - [in] Some surfaces do not have an exact offset that
      can be represented using the same class of surface definition.
      In that case, the tolerance specifies the desired accuracy.
    max_deviation - [out] If this parameter is not NULL, the maximum
      deviation from the returned offset to the true offset is returned
      here.  This deviation is zero except for cases where an exact
      offset cannot be computed using the same class of surface definition.
  Returns:
    Offset surface.
  */
  ON_Surface* Offset(
        double offset_distance, 
        double tolerance, 
        double* max_deviation = NULL
        ) const;

  int GetNurbForm( // returns 0: unable to create NURBS representation
                   //            with desired accuracy.
                   //         1: success - returned NURBS parameterization
                   //            matches the surface's to wthe desired accuracy
                   //         2: success - returned NURBS point locus matches
                   //            the surfaces's to the desired accuracy but, on
                   //            the interior of the surface's domain, the 
                   //            surface's parameterization and the NURBS
                   //            parameterization may not match to the 
                   //            desired accuracy.
        ON_NurbsSurface&,
        double = 0.0 // tolerance
        ) const;

  /////////////////////////////////////////////////////////////////
  // Interface

  /*
  Description:
    Get the maximum length of a nurb surface's control polygon
    rows and/or columns
  Parameters:
    dir - [in] 0 to get "u" direction length, 1 to get "v" 
               direction length
    length - [out] maximum length of a polygon "row" in the 
                   specified direction
  Returns:
    true if successful.
  */
  double ControlPolygonLength( int dir ) const;


  bool IsRational(  // true if NURBS surface is rational
        void
        ) const;
  
  int CVSize(       // number of doubles per control vertex 
        void        // = IsRational() ? Dim()+1 : Dim()
        ) const;
  
  int Order(        // order = degree + 1
        int         // dir 0 = "s", 1 = "t"
        ) const;
	
  int CVCount(      // number of control vertices
        int         // dir 0 = "s", 1 = "t"
        ) const;

  int CVCount(      // total number of control vertices
        void
        ) const;

  int KnotCount(    // total number of knots in knot vector
        int dir         // dir 0 = "s", 1 = "t"
        ) const;
  
  /*
  Description:
    Expert user function to get a pointer to control vertex
    memory.  If you are not an expert user, please use
    ON_NurbsSurface::GetCV( ON_3dPoint& ) or 
    ON_NurbsSurface::GetCV( ON_4dPoint& ).
  Parameters:
    i - [in] (0 <= i < m_cv_count[0])
    j - [in] (0 <= j < m_cv_count[1])
  Returns:
    Pointer to control vertex.
  Remarks:
    If the NURBS surface is rational, the format of the 
    returned array is a homogeneos rational point with
    length m_dim+1.  If the NURBS surface is not rational, 
    the format of the returned array is a nonrational 
    euclidean point with length m_dim.
  See Also
    ON_NurbsSurface::CVStyle
    ON_NurbsSurface::GetCV
    ON_NurbsSurface::Weight
  */
  double* CV(
        int i,
        int j
        ) const;

  /*
  Description:
    Returns the style of control vertices in the m_cv array.
  Returns:
    @untitled table
    ON::not_rational                m_is_rat is false
    ON::homogeneous_rational        m_is_rat is true
  */
  ON::point_style CVStyle() const;

  double Weight(        // get value of control vertex weight
        int i, int j   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        ) const;

  ON_BOOL32 SetWeight(      // get value of control vertex weight
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        double weight
        );

  ON_BOOL32 SetCV(              // set a single control vertex
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        ON::point_style, // style of input point
        const double* cv    // value of control vertex
        );

  ON_BOOL32 SetCV(               // set a single control vertex
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        const ON_3dPoint& cv// value of control vertex
                           // If NURBS is rational, weight
                           // will be set to 1.
        );

  ON_BOOL32 SetCV(              // set a single control vertex
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        const ON_4dPoint& cv// value of control vertex
        );

  ON_BOOL32 SetCVRow(          // Sets CV( *, row_index ) 
       int row_index,               // row_index >= 0 and < m_cv_count[1]
       const ON_3dPoint& cv // value of control vertex
                          // If NURBS is rational, weight
                          // will be set to 1.
       );

  ON_BOOL32 SetCVRow(          // Sets CV( *, row_index ) 
       int row_index,               // row_index >= 0 and < m_cv_count[1]
       int v_stride,               // v stride
       const double* v     // v[] = values (same dim and is_rat as surface)
       );

  ON_BOOL32 SetCVColumn(       // Sets CV( col_index, * ) 
       int col_index,               // col_index >= 0 and < m_cv_count[0]
       const ON_3dPoint& cv // value of control vertex
                          // If NURBS is rational, weight
                          // will be set to 1.
       );

  ON_BOOL32 SetCVColumn(       // Sets CV( col_index, * ) 
       int col_index,               // col_index >= 0 and < m_cv_count[0]
       int v_stride,               // v stride
       const double* v     // v[] = values (same dim and is_rat as surface)
       );

  ON_BOOL32 GetCV(              // get a single control vertex
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        ON::point_style, // style to use for output point
        double* cv          // array of length >= CVSize()
        ) const;

  ON_BOOL32 GetCV(              // get a single control vertex
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        ON_3dPoint& cv     // gets euclidean cv when NURBS is rational
        ) const;

  ON_BOOL32 GetCV(              // get a single control vertex
        int i, int j,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        ON_4dPoint& cv     // gets homogeneous cv
        ) const;

  int SetKnot(
        int dir,    // dir 0 = "s", 1 = "t"
        int knot_index,            // knot index ( 0 to KnotCount - 1 )
        double knot_value         // value for knot
        );

  double Knot(
        int dir,    // dir 0 = "s", 1 = "t"
        int knot_index  // knot index ( >= 0 and < Order + CV_count - 2 )
        ) const;

  int KnotMultiplicity(
        int dir,    // dir 0 = "s", 1 = "t"
        int knot_index            // knot index ( >= 0 and < Order + CV_count - 2 )
        ) const;

  const double* Knot(   // knot[] array
        int dir    // dir 0 = "s", 1 = "t"
        ) const;

  // Description:
  //   Make knot vector a clamped uniform knot vector
  //   based on the current values of m_order and m_cv_count.
  //   Does not change values of control vertices.
  // Parameters:
  //   dir - [in] 0 = u knots, 1 = v knots
  //   delta - [in] (>0.0) knot spacing.
  // Returns:
  //   true if successful.
  // Remarks:
  //   Allocates m_knot[] if it is not big enough.
  // See Also:
  //   ON_MakeClampedUniformKnotVector
  bool MakeClampedUniformKnotVector( 
    int dir,
    double delta = 1.0 
    );

  // Description:
  //   Make knot vector a periodic uniform knot vector
  //   based on the current values of m_order and m_cv_count.
  //   Does not change values of control vertices.
  // Parameters:
  //   dir - [in] 0 = u knots, 1 = v knots
  //   delta - [in] (>0.0) knot spacing.
  // Returns:
  //   true if successful.
  // Remarks:
  //   Allocates m_knot[] if it is not big enough.
  // See Also:
  //   ON_MakePeriodicUniformKnotVector
  bool MakePeriodicUniformKnotVector( 
    int dir,
    double delta = 1.0 
    );


  bool IsClamped( // determine if knot vector is clamped
        int dir,    // dir 0 = "s", 1 = "t"
        int end = 2 // end to check: 0 = start, 1 = end, 2 = start and end
        ) const;
  
  double SuperfluousKnot(
           int dir,    // dir 0 = "s", 1 = "t"
           int end  // 0 = start, 1 = end
           ) const;

  double GrevilleAbcissa(
           int dir,  // dir
           int cv_index  // index (0 <= index < CVCount(dir)
           ) const;

  bool GetGrevilleAbcissae( // see ON_GetGrevilleAbcissa() for details
           int dir,      // dir
           double* g   // g[cv count]
           ) const;

  bool SetClampedGrevilleKnotVector(
           int dir,          // dir
           int g_stride,          // g_stride
           const double* g // g[], CVCount(dir) many Greville abcissa
           );

  bool SetPeriodicGrevilleKnotVector(
           int dir,          // dir
           int g_stride,          // g_stride
           const double* g // g[], Greville abcissa
           );

  bool ZeroCVs(); // zeros all CVs (any weights set to 1);

  bool ClampEnd(
            int dir,         // dir 0 = "s", 1 = "t"
            int end // 0 = clamp start, 1 = clamp end, 2 = clamp start and end
            );

  bool InsertKnot(
           int dir,         // dir 0 = "s", 1 = "t"
           double knot_value, // value of knot
           int knot_multiplicity=1   // multiplicity of knot ( >= 1 and <= degree )
           );

  bool MakeRational();

  bool MakeNonRational();

  bool IncreaseDegree(
           int dir,  // dir 0 = "s", 1 = "t"
           int desired_degree  //  desired_degree
           );

  bool ChangeDimension(
           int desired_dimension  //  desired_dimension
           );

  /*
  Description:
    If the surface is closed in direction dir, then modify it so that
    the seam is at parameter t in the dir direction.
  Parameters:
		dir - [in] must be 0 or 1
    t -		[in] dir parameter of seam, must have Domain(dir).Includes(t).
               The resulting surface domain in the dir direction will start at t.
  Returns:
    true if successful.
  */
  ON_BOOL32 ChangeSurfaceSeam( 
						int dir,
            double t 
            );


  // Creates a tensor product nurbs surface with srf(s,t) = T(A(s),B(t));
  ON_BOOL32 TensorProduct(
        const ON_NurbsCurve&, // A
        const ON_NurbsCurve&, // B
        ON_TensorProduct&     // T
        );

  /////////////////////////////////////////////////////////////////
  // Tools for managing CV and knot memory
  ON_BOOL32 ReserveKnotCapacity( // returns false if allocation fails
                    // does not change m_order or m_cv_count
    int dir, // dir 0 = "s", 1 = "t"
    int knot_array_capacity // minimum capacity of m_knot[] array
    );
  ON_BOOL32 ReserveCVCapacity(  // returns false if allocation fails
                    // does not change m_order or m_cv_count
    int cv_array_capacity // minimum capacity of m_cv[] array
    );

  /*
  Description:
    Convert a NURBS surface bispan into a bezier surface.
  Parameters:
    span_index0 - [in] Specifies the "u" span and must satisfy
         0 <= span_index0 <= m_cv_count[0]-m_order[0]
         m_knot[0][span_index0+m_order[0]-2] < m_knot[0][span_index0+m_order[0]-1]
    span_index1 - [in] Specifies the "v" span and must satisfy
         0 <= span_index1 <= m_cv_count[1]-m_order[1]
         m_knot[1][span_index1+m_order[1]-2] < m_knot[1][span_index1+m_order[1]-1]
    bezier_surface - [out] bezier surface returned here
  Returns:
    true if successful
    false if input is not valid
  */
  ON_BOOL32 ConvertSpanToBezier(
      int span_index0,
      int span_index1, 
      ON_BezierSurface& bezier_surface
      ) const;

  /////////////////////////////////////////////////////////////////
  // Implementation
public:
  // NOTE: These members are left "public" so that expert users may efficiently
  //       create NURBS curves using the default constructor and borrow the
  //       knot and CV arrays from their native NURBS representation.
  //       No technical support will be provided for users who access these
  //       members directly.  If you can't get your stuff to work, then use
  //       the constructor with the arguments and the SetKnot() and SetCV()
  //       functions to fill in the arrays.

  int     m_dim;            // (>=1)

  int     m_is_rat;         // 1 for rational B-splines. (Control vertices
                            // use homogeneous form.)
                            // 0 for non-rational B-splines. (Control
                            // verticies do not have a weight coordinate.)

  int     m_order[2];       // order = degree+1 (>=2)

  int     m_cv_count[2];    // number of control vertices ( >= order )

  // knot vector memory

  int     m_knot_capacity[2]; // If m_knot_capacity > 0, then m_knot[]
                              // is an array of at least m_knot_capacity
                              // doubles whose memory is managed by the
                              // ON_NurbsSurface class using rhmalloc(),
                              // onrealloc(), and rhfree().
                              // If m_knot_capacity is 0 and m_knot is
                              // not NULL, then  m_knot[] is assumed to
                              // be big enough for any requested operation
                              // and m_knot[] is not deleted by the
                              // destructor.

  double* m_knot[2];        // Knot vector. ( The knot vector has length
                            // m_order+m_cv_count-2. )
  
  // control vertex net memory

  int     m_cv_stride[2];   // The pointer to start of "CV[i]" is
                            //   m_cv + i*m_cv_stride.

  int     m_cv_capacity;    // If m_cv_capacity > 0, then m_cv[] is an array
                            // of at least m_cv_capacity doubles whose
                            // memory is managed by the ON_NurbsSurface
                            // class using rhmalloc(), onrealloc(), and rhfree().
                            // If m_cv_capacity is 0 and m_cv is not
                            // NULL, then m_cv[] is assumed to be big enough
                            // for any requested operation and m_cv[] is not
                            // deleted by the destructor.

  double* m_cv;             // Control points.
                            // If m_is_rat is false, then control point is
                            //
                            //          ( CV(i)[0], ..., CV(i)[m_dim-1] ).
                            //
                            // If m_is_rat is true, then the control point
                            // is stored in HOMOGENEOUS form and is
                            //
                            //         [ CV(i)[0], ..., CV(i)[m_dim] ].
                            // 
};


class PCL_EXPORTS ON_CLASS ON_NurbsCage : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_NurbsCage);

public:
  ON_NurbsCage();

  ON_NurbsCage(
    int dim,
    bool is_rat,
    int order0,
    int order1,
    int order2,
    int cv_count0,
    int cv_count1,
    int cv_count2
    );

  ON_NurbsCage( 
    const ON_BoundingBox& bbox,
    int order0,
    int order1,
    int order2,
    int cv_count0,
    int cv_count1,
    int cv_count2
    );

  ON_NurbsCage( 
    const ON_3dPoint* box_corners, // array of 8 3d points
    int order0,
    int order1,
    int order2,
    int cv_count0,
    int cv_count1,
    int cv_count2
    );

  ON_NurbsCage( const ON_BezierCage& src );

  ~ON_NurbsCage();

  ON_NurbsCage(const ON_NurbsCage& src);

  ON_NurbsCage& operator=(const ON_NurbsCage& src);

  ON_NurbsCage& operator=(const ON_BezierCage& src);


  /*
  Description:
    Overrides the pure virtual ON_Object::IsValid function.
  Parameters:
    text_log - [in] If not null and the object is invalid,
                    a brief description of the problem
                    suitable for debugging C++ code
                    is printed in this log.
  Returns:
    True if the orders are at least two, dimension is positive,
    knot vectors are valid, and the other fields are valid
    for the specified orders and dimension.
  */
  ON_BOOL32 IsValid( 
          ON_TextLog* text_log = NULL 
          ) const;

  /*
  Description:
    Overrides the pure virtual ON_Object::Dump function.
  Parameters:
    text_log - [in] A listing of the values of the members.
  */
  void Dump( ON_TextLog& text_log) const;

  /*
  Description:
    Overrides the pure virtual ON_Object::SizeOf function.
  Returns:
    An estimate of the amount of memory used by the class 
    and its members.
  */
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  /*
  Description:
    Overrides the pure virtual ON_Object::Read function.
    Reads the definition of this class from an
    archive previously saved by ON_BezierVolue::Write.
  Parameters:
    archive - [in] target archive
  Returns:
    True if successful.
  */
  ON_BOOL32 Read(
    ON_BinaryArchive& archive
    );

  /*
  Description:
    Overrides the pure virtual ON_Object::Write function.
    Saves the definition of this class in serial binary
    form that can be read by ON_BezierVolue::Read.
  Parameters:
    archive - [in] target archive
  Returns:
    True if successful.
  */
  ON_BOOL32 Write(
    ON_BinaryArchive& archive
    ) const;

  /*
  Description:
    Overrides the pure virtual ON_Object::ObjectType function.
    Saves the definition of this class in serial binary
    form that can be read by ON_BezierVolue::Read.
  Parameters:
    archive - [in] target archive
  Returns:
    True if successful.
  */
  ON::object_type ObjectType() const;

  /*
  Description:
    Overrides the pure virtual ON_Object::DestroyRuntimeCache function.
    Saves the definition of this class in serial binary
    form that can be read by ON_BezierVolue::Read.
  Parameters:
    bDelete - [in] if true, the cache is deleted.  If false, the
       pointers to the cache are set to zero; this is done when
       the cache memory was allocated from a pool that has
       been destroyed and an attempt to free the memory would
       result in a crash.
  Returns:
    True if successful.
  */
  void DestroyRuntimeCache( 
    bool bDelete = true 
    );


  /*
  Description:
    Overrides virtual ON_Geometry::Dimension function.
    Gets a tight bounding box with respect to the coordinate
    system specified by the frame parameter.
  Parameters:
    bbox - [in/out]
    bGrowBox - [in] If true, the input bbox is grown to include
        this object's bounding box.
    frame - [in] if not null, this specifies the coordinate system
                frame.
  Returns:
    True if successful.
  */
  int Dimension() const;

  /*
  Description:
    Overrides virtual ON_Geometry::GetBBox function.
    Gets the world axis aligned bounding box that contains
    the NURBS volume's control points.  The NURBS volume
    maps the unit cube into this box.
  Parameters:
    boxmin - [in] array of Dimension() doubles
    boxmax - [in] array of Dimension() doubles
    bGrowBox =  [in] if true and the input is a valid box
                          then the input box is grown to
                          include this object's bounding box.
  Returns:
    true if successful.
  */
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
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
      surface's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      surface is calculated.  The surface is not modified.
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
    Overrides virtual ON_Geometry::Transform function.
    Transforms NURBS volume.
  Parameters:
    xform - [in]
  Returns:
    true if successful.
  */
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );

  /*
  Description:
    Overrides virtual ON_Geometry::IsDeformable function.
  Returns:
    True because a NURBS volume can be accuratly modified 
    with "squishy" transformations like projections, 
    shears, an non-uniform scaling.
  */
  bool IsDeformable() const;

  /*
  Description:
    Overrides virtual ON_Geometry::MakeDeformable function.
  Returns:
    True because NURBS volumes are deformable.
  */
  bool MakeDeformable();

  /*
  Returns:
    True if the cage is a parallelogram within the tolerance.
    This means the cage can be used as a starting point
    for cage deformations.
  */
  bool IsParallelogram(double tolerance) const;

  bool Create(
    int dim,
    bool is_rat,
    int order0,
    int order1,
    int order2,
    int cv_count0,
    int cv_count1,
    int cv_count2
    );

  /*
  Description:
    Create a Nurbs volume with corners defined by a bounding box.
  Parameters:
    box_corners - [in] 8 points that define corners of the volume

            7______________6
            |\             |\
            | \            | \
            |  \ _____________\
            |   4          |   5
            |   |          |   |
            |   |          |   |
            3---|----------2   |
            \   |          \   |
             \  |z          \  |
            y \ |            \ |
               \0_____________\1
                       x

  */
  bool Create(
    const ON_BoundingBox& bbox,
    int order0,
    int order1,
    int order2,
    int cv_count0,
    int cv_count1,
    int cv_count2
    );

  /*
  Description:
    Create a nurbs volume from a 3d box
  Parameters:
    box_corners - [in] 8 points that define corners of the volume

            7______________6
            |\             |\
            | \            | \
            |  \ _____________\
            |   4          |   5
            |   |          |   |
            |   |          |   |
            3---|----------2   |
            \   |          \   |
             \  |t          \  |
            s \ |            \ |
               \0_____________\1
                       r

  */
  bool Create(
    const ON_3dPoint* box_corners,
    int order0,
    int order1,
    int order2,
    int cv_count0,
    int cv_count1,
    int cv_count2
    );

  void Destroy();

  void EmergencyDestroy(); // call if memory used by ON_NurbsCage becomes invalid

  ON_Interval Domain(
    int // dir 0 = "r", 1 = "s", 2 = "t"
    ) const;

  bool Reverse( 
    int dir // dir 0 = "r", 1 = "s", 2 = "t"
    );
  
  bool Transpose(
    int dir0,
    int dir1
    );

  bool ClampEnd(
            int dir,         // dir 0 = "r", 1 = "s", 2 = "t"
            int end // 0 = clamp start, 1 = clamp end, 2 = clamp start and end
            );

  bool InsertKnot(
           int dir,         // dir 0 = "r", 1 = "s", 2 = "t"
           double knot_value, // value of knot
           int knot_multiplicity=1   // multiplicity of knot ( >= 1 and <= degree )
           );

  ON_BOOL32 IncreaseDegree(
           int dir,  // dir 0 = "r", 1 = "s", 2 = "t"
           int desired_degree  //  desired_degree
           );

  ON_BOOL32 ChangeDimension(
           int desired_dimension  //  desired_dimension
           );

  /*
  Description:
    Evaluate the NURBS cage
  Parameters:
    r - [in]
    s - [in]
    t - [in] (r,s,t) = evaluation parameters
    der_count - [in]  (>= 0)
    v_stride - [in] (>= m_dim)
    v - [out] An array of length v_stride*(der_count+1)(der_count+2)*(der_count+3)/6.
              The evaluation results are stored in this array.

                P = v[0],...,v[m_dim-1]
                Dr = v[v_stride],...
                Ds = v[2*v_stride],...
                Dt = v[3*v_stride],...

              In general, Dr^i Ds^j Dt^k is returned in v[n],...,v[n+m_dim-1], where

               d = (i+j+k)
               n = v_stride*( d*(d+1)*(d+2)/6 + (j+k)*(j+k+1)/2 + k) 

    side - [in] specifies the span to use for the evaluation
                when r, s, or t is at a knot value.
            0 = default
            1 = from upper NE quadrant
            2 = from upper NW quadrant
            3 = from upper SW quadrant
            4 = from upper SE quadrant
            5 = from lower NE quadrant
            6 = from lower NW quadrant
            7 = from lower SW quadrant
            8 = from lower SE quadrant  
    hint - [in/out] If a bunch of evaluations will be performed that
                    tend to occur in the same region, then
                    hint[3] can be used to speed the search for
                    the evaluation span.  The input value is
                    used as a search hint and the output value
                    records the span used for that evaluation.
  Example:

          int der_count = 2;
          int v_stride = dim;
          double v[v_stride*(der_count+1)*(der_count+2)*(der_count+3)/6];
          int side = 0;
          int hint[3]; hint[0] = 0; hint[1] = 0; hint[2] = 0;
          bool rc = cage.Evaluate(r,s,t,der_count,v_stride,v,side,hint);

          ON_3dPoint P = v;

          // first order partial derivatives
          ON_3dVector Dr = v + v_stride;
          ON_3dVector Ds = v + 2*v_stride;
          ON_3dVector Dt = v + 3*v_stride;

          // second order partial derivatives
          ON_3dVector Drr = v + 4*v_stride;
          ON_3dVector Drs = v + 5*v_stride;
          ON_3dVector Drt = v + 6*v_stride;
          ON_3dVector Dss = v + 7*v_stride;
          ON_3dVector Dst = v + 8*v_stride;
          ON_3dVector Dtt = v + 8*v_stride;

  Returns:
    True if successful
  See Also:
    ON_NurbsCage::PointAt
  */
  bool Evaluate( 
         double r, 
         double s, 
         double t,
         int der_count,
         int v_stride,
         double* v,
         int side=0,
         int* hint=0
         ) const;

  /*
  Description:
    Evaluates bezer volume map.
  Parameters:
    rst - [in]
  Returns:
    Value of the nurbs volume map at (r,s,t).
  */
  ON_3dPoint PointAt(
         double r, 
         double s, 
         double t
         ) const;

  ON_NurbsSurface* IsoSurface(
         int dir,
         double c,
         ON_NurbsSurface* srf = 0
         ) const;

  bool Trim(
         int dir,
         const ON_Interval& domain
         );

  bool Extend(
    int dir,
    const ON_Interval& domain
    );

  /*
  Description:
    Evaluates bezer volume map.
  Parameters:
    rst - [in]
  Returns:
    Value of the nurbs volume map at (rst.x,rst.y,rst.z).
  */
  ON_3dPoint PointAt(
         ON_3dPoint rst
         ) const;

  bool IsRational() const;
  
  int CVSize() const;
  
  int Order(
        int dir     // dir 0 = "r", 1 = "s", 2 = "t"
        ) const;
	
  int CVCount(      // number of control vertices
        int         // dir 0 = "r", 1 = "s", 2 = "t"
        ) const;

  int CVCount(      // total number of control vertices
        void
        ) const;

  int KnotCount(    // total number of knots in knot vector
        int dir     // dir 0 = "r", 1 = "s", 2 = "t"
        ) const;

  int Degree(
        int dir
        ) const;


  int SpanCount(
    int dir         // dir 0 = "r", 1 = "s", 2 = "t"
    ) const;

  bool GetSpanVector(
    int dir,        // dir 0 = "r", 1 = "s", 2 = "t"
    double* span_vector
    ) const;

  /*
  Description:
    Expert user function to get a pointer to control vertex
    memory.  If you are not an expert user, please use
    ON_NurbsCage::GetCV( ON_3dPoint& ) or 
    ON_NurbsCage::GetCV( ON_4dPoint& ).
  Parameters:
    cv_index0 - [in] (0 <= cv_index0 < m_order[0])
    cv_index1 - [in] (0 <= cv_index1 < m_order[1])
  Returns:
    Pointer to control vertex.
  Remarks:
    If the Nurbs surface is rational, the format of the 
    returned array is a homogeneos rational point with
    length m_dim+1.  If the Nurbs surface is not rational, 
    the format of the returned array is a nonrational 
    euclidean point with length m_dim.
  See Also
    ON_NurbsCage::CVStyle
    ON_NurbsCage::GetCV
    ON_NurbsCage::Weight
  */
  double* CV(
        int i,
        int j,
        int k
        ) const;

  /*
  Description:
    Returns the style of control vertices in the m_cv array.
  Returns:
    @untitled table
    ON::not_rational                m_is_rat is false
    ON::homogeneous_rational        m_is_rat is true
  */
  ON::point_style CVStyle() const;

  double Weight(        // get value of control vertex weight
        int i,
        int j,
        int k
        ) const;

  bool SetWeight(      // get value of control vertex weight
        int i,
        int j,
        int k,
        double w
        );

  bool SetCV(              // set a single control vertex
        int i,
        int j,
        int k,
        ON::point_style, // style of input point
        const double*     // value of control vertex
        );

  // set a single control vertex
  // If NURBS is rational, weight
  // will be set to 1.
  bool SetCV(
        int i,
        int j,
        int k,
        const ON_3dPoint& point
        );

  // set a single control vertex
  // value of control vertex
  // If NURBS is not rational, euclidean
  // location of homogeneous point will
  // be used.
  bool SetCV(
        int i,
        int j,
        int k,
        const ON_4dPoint& hpoint
        );

  bool GetCV(              // get a single control vertex
        int i,
        int j,
        int k,
        ON::point_style, // style to use for output point
        double*           // array of length >= CVSize()
        ) const;

  bool GetCV(              // get a single control vertex
        int i,
        int j,
        int k,
        ON_3dPoint&      // gets euclidean cv when NURBS is rational
        ) const;

  bool GetCV(              // get a single control vertex
        int i,
        int j,
        int k,
        ON_4dPoint&      // gets homogeneous cv
        ) const;

  /*
  Parameters:
    dir - [in] 0 = "r", 1 = "s", 2 = "t"
    knot_index - [in] 0 <= knot_index < KnotCount(dir)
    knot_value - [in]
  Returns:
    True if dir and knot_index parameters were valid and knot value
    was set.
  */
  bool SetKnot(
        int dir,
        int knot_index,
        double knot_value
        );

  /*
  Parameters:
    dir - [in] 0 = "r", 1 = "s", 2 = "t"
    knot_index - [in] 0 <= knot_index < KnotCount(dir)
  Returns:
    Value of knot or ON_UNSET_VALUE if input parameters are not valid.
  */
  double Knot(
        int dir,
        int knot_index
        ) const;

  bool ZeroCVs(); // zeros control vertices and, if rational, sets weights to 1

  bool MakeRational();

  bool MakeNonRational();

  bool IsClosed(   // true if NURBS cage is closed (either cage has
        int // dir // clamped end knots and euclidean location of start
        ) const;   // CV = euclidean location of end CV, or cage is
                   // periodic.)

  bool IsPeriodic( // true if NURBS cage is periodic (degree > 1,
        int // dir // periodic knot vector, last degree many CVs 
        ) const;   // are duplicates of first degree many CVs.)

  bool IsSingular( // true if cage side is collapsed to a point
        int        // side of parameter space to test
                   // 0 = south, 1 = east, 2 = north, 3 = west
        ) const;

  double GrevilleAbcissa(
          int dir,    // dir
          int gindex  // index (0 <= index < CVCount(dir)
          ) const;

  /////////////////////////////////////////////////////////////////
  // Tools for managing CV and knot memory

  /*
  Description:
    cv_capacity - [in] number of doubles to reserve
  */
  bool ReserveCVCapacity(
    int cv_capacity
    );

  bool ReserveKnotCapacity(
    int dir,
    int cv_capacity
    );

  /////////////////////////////////////////////////////////////////
  // Implementation
public:
  // NOTE: These members are left "public" so that expert users may efficiently
  //       create nurbs curves using the default constructor and borrow the
  //       knot and CV arrays from their native NURBS representation.
  //       No technical support will be provided for users who access these
  //       members directly.  If you can't get your stuff to work, then use
  //       the constructor with the arguments and the SetKnot() and SetCV()
  //       functions to fill in the arrays.


  int     m_dim;
  bool    m_is_rat;
  int     m_order[3];
  int     m_cv_count[3];
  int     m_knot_capacity[3];
  double* m_knot[3];
  int     m_cv_stride[3];
  int     m_cv_capacity;
  double* m_cv;
};

ON_DECL
bool ON_GetCageXform( 
          const ON_NurbsCage& cage, 
          ON_Xform& cage_xform 
          );


class PCL_EXPORTS ON_CLASS ON_MorphControl : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_MorphControl);

public:
  ON_MorphControl();
  ~ON_MorphControl();
  // C++ default copy construction and operator= work fine.


  void Destroy();
  

  /////////////////////////////////////////////////////////
  //
  // ON_Object virtual functions
  //

  void MemoryRelocate();

  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const;

  unsigned int SizeOf() const;

  ON_BOOL32 Write(
    ON_BinaryArchive& archive
    ) const;

  ON_BOOL32 Read(
    ON_BinaryArchive& archive
    );

  ON::object_type ObjectType() const;

  void DestroyRuntimeCache( bool bDelete = true );

  /////////////////////////////////////////////////////////
  //
  // ON_Geometry virtual functions
  //

  int Dimension() const;

  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;

	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  void ClearBoundingBox();

  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );

  ON_BOOL32 HasBrepForm() const;

  ON_Brep* BrepForm( ON_Brep* brep = NULL ) const;


  /*
  Returns:
    True if the target NURBS object is rational
  */
  bool IsRational() const;

  /*
  Description:
    Makes the target NURBS object rational.
  */
  bool MakeRational();

  /*
  Description:
    Makes the target NURBS object non-rational.
  */
  bool MakeNonRational();

  /*
  Returns:
    Number of control points in the target NURBS object.
  */
  int CVCount() const;

  int CVCount(int dir) const;
  int Order(int dir) const;
  const double* Knot(int dir) const;
  ON_3dex MaxCVIndex() const;
  const double* CV(ON_3dex) const;
  double Weight(ON_3dex) const;

  /////////////////////////////////////////////////////////
  //
  // Localizers
  //

  /*
  Description:
    Adds localizer with support near the controling NURBS object.
  Parameters:
    support_distance - [in] >= 0
      If the distance a point to the controls NURBS 
      curve/surface/cage is less than or equal to support_distance,
      then MorphPoint() deformation has 100% effect.
      
    falloff_distance - [in] > 0
      If the distance a point to the controls NURBS 
      curve/surface/cage is more than support_distance+falloff_distance,
      then MorphPoint() deformation does not move the point.
      As the distance varies from support_distance to 
      support_distance+falloff_distance the deformation attenuates
      from 100% to 0%.
  */
  bool AddControlLocalizer(
    double support_distance, 
    double falloff_distance
    );

  bool AddSphereLocalizer(
    ON_3dPoint center,
    double support_distance, 
    double falloff_distance
    );

  bool AddCylinderLocalizer(
    ON_Line axis,
    double support_distance, 
    double falloff_distance
    );

  bool AddBoxLocalizer(
    ON_BoundingBox bbox,
    double support_distance, 
    double falloff_distance
    );

  bool AddPlaneLocalizer(
    const ON_Plane& plane,
    double support_distance, 
    double falloff_distance
    );

  bool AddConvexPolygonLocalizer(
    const ON_SimpleArray<ON_Plane>& planes,
    double support_distance, 
    double falloff_distance
    );

  /////////////////////////////////////////////////////////
  //
  //

  // Get a cage_morph that can be passed to Morph functions
  bool GetCageMorph( class ON_CageMorph& cage_morph ) const;

  bool IsIdentity( const ON_BoundingBox& bbox ) const;

  int m_varient; // 1= curve, 2 = surface, 3 = cage

  // The value of m_varient determines which nurbs object
  // controls the cage
  ON_NurbsCurve   m_nurbs_curve0;
  ON_NurbsCurve   m_nurbs_curve;
  ON_Interval     m_nurbs_curve_domain;

  ON_NurbsSurface m_nurbs_surface0;
  ON_NurbsSurface m_nurbs_surface;
  ON_Interval     m_nurbs_surface_domain[2];

  ON_Xform        m_nurbs_cage0;
  ON_NurbsCage    m_nurbs_cage;

  // Rhino captive object ids
  ON_UuidList m_captive_id;

  // Use ON_GetCageXform to set m_cage_xform.

  // Used to localize the deformation
  ON_ClassArray<ON_Localizer> m_localizers;

  // ON_SpaceMorphOptions
  double m_sporh_tolerance;
  bool   m_sporh_bQuickPreview;
  bool   m_sporh_bPreserveStructure;
};


class PCL_EXPORTS ON_CLASS ON_CageMorph : public ON_SpaceMorph
{
public:
  ON_CageMorph();
  ~ON_CageMorph();

  bool IsIdentity( const ON_BoundingBox& bbox ) const;

  const ON_MorphControl* m_control;
};


// Description:
//   Get an ON_NurbsSurface definition of a quadrilateral.
// Parameters:
//   P - [in]
//   Q - [in]
//   R - [in]
//   S - [in] corners in counter clockwise layer
//   nurbs_surface - [in] if this pointer is not NULL,
//       then this ON_NurbsSurface is used to return
//       the quadrilateral.
// Returns:
//   An ON_NurbsSurface representation of the quadrilateral.
ON_DECL
ON_NurbsSurface* ON_NurbsSurfaceQuadrilateral( 
             const ON_3dPoint& P, 
             const ON_3dPoint& Q, 
             const ON_3dPoint& R, 
             const ON_3dPoint& S,
             ON_NurbsSurface* nurbs_surface = NULL
             );

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_NurbsCurve>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_NurbsCurve>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_NurbsCurve*>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_NurbsSurface>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_NurbsSurface>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_NurbsSurface*>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_NurbsCage>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_NurbsCage>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_NurbsCage*>;
#pragma warning( pop )
#endif

#endif

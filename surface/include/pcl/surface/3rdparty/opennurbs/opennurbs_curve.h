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
//   Definition of virtual parametric curve
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_CURVE_INC_)
#define OPENNURBS_CURVE_INC_

class ON_Curve;
class ON_Plane;
class ON_Arc;
class ON_NurbsCurve;
class ON_CurveTree;


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////


class ON_CLASS ON_MeshCurveParameters
{
public:
  ON_MeshCurveParameters();

  // If main_seg_count <= 0, then both these parameters are ignored.
  // If main_seg_count > 0, then sub_seg_count must be >= 1.  In this
  // case the curve will be broken into main_seg_count equally spaced
  // chords. If needed, each of these chords can be split into as many
  // sub_seg_count sub-parts if the subdivision is necessary for the
  // mesh to meet the other meshing constraints.  In particular, if
  // sub_seg_count = 0, then the curve is broken into main_seg_count
  // pieces and no further testing is performed.
  int m_main_seg_count; 
  int m_sub_seg_count;

  int m_reserved1;
  int m_reserved2;

  // Maximum angle (in radians) between unit tangents at adjacent
  // vertices.
  double m_max_ang_radians;

  // Maximum permitted value of 
  // distance chord midpoint to curve) / (length of chord)
  double m_max_chr;

  // If max_aspect < 1.0, the parameter is ignored. 
  // If 1 <= max_aspect < sqrt(2), it is treated as if 
  // max_aspect = sqrt(2).
  // This parameter controls the maximum permitted value of
  // (length of longest chord) / (length of shortest chord)
  double m_max_aspect;

  // If tolerance = 0, the parameter is ignored.
  // This parameter controls the maximum permitted value of the
  // distance from the curve to the mesh.  
  double m_tolerance;

  // If m_min_edge_length = 0, the parameter is ignored.
  // This parameter controls the minimum permitted edge length.
  double m_min_edge_length;
  
  // If max_edge_length = 0, the parameter is ignored.
  // This parameter controls the maximum permitted edge length.
  double m_max_edge_length;

  double m_reserved3;
  double m_reserved4;
};

class ON_CLASS ON_Curve : public ON_Geometry
{
  // pure virtual class for curve objects

  // Any object derived from ON_Curve should have a
  //   ON_OBJECT_DECLARE(ON_...);
  // as the last line of its class definition and a
  //   ON_OBJECT_IMPLEMENT( ON_..., ON_baseclass );
  // in a .cpp file.
  //
  // See the definition of ON_Object for details.
  ON_OBJECT_DECLARE(ON_Curve);

public:
  // virtual ON_Object::DestroyRuntimeCache override
  void DestroyRuntimeCache( bool bDelete = true );

public:
  ON_Curve();
  ON_Curve(const ON_Curve&);
  ON_Curve& operator=(const ON_Curve&);
  virtual ~ON_Curve();

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Geometry override
  bool EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const;

  /*
  Description:
    Get a duplicate of the curve.
  Returns:
    A duplicate of the curve.  
  Remarks:
    The caller must delete the returned curve.
    For non-ON_CurveProxy objects, this simply duplicates the curve using
    ON_Object::Duplicate.
    For ON_CurveProxy objects, this duplicates the actual proxy curve 
    geometry and, if necessary, trims and reverse the result to that
    the returned curve's parameterization and locus match the proxy curve's.
  */
  virtual
  ON_Curve* DuplicateCurve() const;

  // Description:
  //   overrides virtual ON_Object::ObjectType.
  // Returns:
  //   ON::curve_object
  ON::object_type ObjectType() const;

  /*
	Description:
    Get tight bounding box of the curve.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      curve's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      curve is calculated.  The curve is not modified.
	Returns:
    True if the returned tight_bbox is set to a valid 
    bounding box.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  ////////////////////////////////////////////////////////////////////
  // curve interface

  // Description:
  //   Gets domain of the curve
  // Parameters:
  //   t0 - [out]
  //   t1 - [out] domain is [*t0, *t1]
  // Returns:
  //   true if successful.
  ON_BOOL32 GetDomain( double* t0, double* t1 ) const;

  // Returns:
  //   domain of the curve.
  virtual 
  ON_Interval Domain() const = 0;

  /*
  Description:
    Set the domain of the curve.
  Parameters:
    domain - [in] increasing interval
  Returns:
    true if successful.
  */
  bool SetDomain( ON_Interval domain );

  // Description:
  //   Set the domain of the curve
  // Parameters:
  //   t0 - [in]
  //   t1 - [in] new domain will be [t0,t1]
  // Returns:
  //   true if successful.
  virtual
  ON_BOOL32 SetDomain( 
        double t0, 
        double t1 
        );


  /*
  Description:
    If this curve is closed, then modify it so that
    the start/end point is at curve parameter t.
  Parameters:
    t - [in] curve parameter of new start/end point.  The
             returned curves domain will start at t.
  Returns:
    true if successful.
  */
  virtual 
  ON_BOOL32 ChangeClosedCurveSeam( 
            double t 
            );

  /*
  Description:
    Change the dimension of a curve.
  Parameters:
    desired_dimension - [in]
  Returns:
    true if the curve's dimension was already desired_dimension
    or if the curve's dimension was successfully changed to
    desired_dimension.
  */
  virtual
  bool ChangeDimension(
          int desired_dimension
          );


  // Description:
  //   Get number of nonempty smooth (c-infinity) spans in curve
  // Returns:
  //   Number of nonempty smooth (c-infinity) spans.
  virtual 
  int SpanCount() const = 0;

  // Description:
  //   Get number of parameters of "knots".
  // Parameters:
  //   knots - [out] an array of length SpanCount()+1 is filled in
  //       with the parameters where the curve is not smooth (C-infinity).
  // Returns:
  //   true if successful
  virtual
  ON_BOOL32 GetSpanVector(
        double* knots
        ) const = 0; // 

  //////////
  // If t is in the domain of the curve, GetSpanVectorIndex() returns the 
  // span vector index "i" such that span_vector[i] <= t <= span_vector[i+1].
  // The "side" parameter determines which span is selected when t is at the
  // end of a span.
  virtual
  ON_BOOL32 GetSpanVectorIndex(
        double t ,               // [IN] t = evaluation parameter
        int side,                // [IN] side 0 = default, -1 = from below, +1 = from above
        int* span_vector_index,  // [OUT] span vector index
        ON_Interval* span_domain // [OUT] domain of the span containing "t"
        ) const;

  // Description:
  //   Returns maximum algebraic degree of any span
  //   or a good estimate if curve spans are not algebraic.
  // Returns:
  //   degree
  virtual 
  int Degree() const = 0; 

  // Description:
  //   Returns maximum algebraic degree of any span
  //   or a good estimate if curve spans are not algebraic.
  // Returns:
  //   degree
  virtual 
  ON_BOOL32 GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
         double t,       // [IN] t = parameter in domain
         double* tminus, // [OUT] tminus
         double* tplus   // [OUT] tplus
         ) const;

  // Description:
  //   Test a curve to see if the locus if its points is a line segment.
  // Parameters:
  //   tolerance - [in] // tolerance to use when checking linearity
  // Returns:
  //   true if the ends of the curve are farther than tolerance apart
  //   and the maximum distance from any point on the curve to
  //   the line segment connecting the curve's ends is <= tolerance.
  virtual
  ON_BOOL32 IsLinear(
        double tolerance = ON_ZERO_TOLERANCE 
        ) const;

  /*
  Description:
    Several types of ON_Curve can have the form of a polyline including
    a degree 1 ON_NurbsCurve, an ON_PolylineCurve, and an ON_PolyCurve
    all of whose segments are some form of polyline.  IsPolyline tests
    a curve to see if it can be represented as a polyline.
  Parameters:
    pline_points - [out] if not NULL and true is returned, then the
        points of the polyline form are returned here.
    t - [out] if not NULL and true is returned, then the parameters of
        the polyline points are returned here.
  Returns:
    @untitled table
    0        curve is not some form of a polyline
    >=2      number of points in polyline form
  */
  virtual
  int IsPolyline(
        ON_SimpleArray<ON_3dPoint>* pline_points = NULL,
        ON_SimpleArray<double>* pline_t = NULL
        ) const;

  // Description:
  //   Test a curve to see if the locus if its points is an arc or circle.
  // Parameters:
  //   plane - [in] if not NULL, test is performed in this plane
  //   arc - [out] if not NULL and true is returned, then arc parameters
  //               are filled in
  //   tolerance - [in] tolerance to use when checking
  // Returns:
  //   ON_Arc.m_angle > 0 if curve locus is an arc between
  //   specified points.  If ON_Arc.m_angle is 2.0*ON_PI, then the curve
  //   is a circle.
  virtual
  ON_BOOL32 IsArc(
        const ON_Plane* plane = NULL,
        ON_Arc* arc = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  /*
  Description:
  Parameters:
    t - [in] curve parameter
    plane - [in]
      if not NULL, test is performed in this plane
    arc - [out]
      if not NULL and true is returned, then arc parameters
       are filled in
    tolerance - [in]
      tolerance to use when checking
    t0 - [out]
      if not NULL, and then *t0 is set to the parameter
      at the start of the G2 curve segment that was
      tested.
    t1 - [out]
      if not NULL, and then *t0 is set to the parameter
      at the start of the G2 curve segment that was
      tested.
  Returns:
    True if the paramter t is on a arc segment of the curve.
  */
  bool IsArcAt( 
    double t, 
    const ON_Plane* plane = 0,
    ON_Arc* arc = 0,
    double tolerance = ON_ZERO_TOLERANCE,
    double* t0 = 0, 
    double* t1 = 0
    ) const;

  virtual
  bool IsEllipse(
      const ON_Plane* plane = NULL,
      ON_Ellipse* ellipse = NULL,
      double tolerance = ON_ZERO_TOLERANCE
      ) const;

  // Description:
  //   Test a curve to see if it is planar.
  // Parameters:
  //   plane - [out] if not NULL and true is returned,
  //                 the plane parameters are filled in.
  //   tolerance - [in] tolerance to use when checking
  // Returns:
  //   true if there is a plane such that the maximum distance from
  //   the curve to the plane is <= tolerance.
  virtual
  ON_BOOL32 IsPlanar(
        ON_Plane* plane = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  // Description:
  //   Test a curve to see if it lies in a specific plane.
  // Parameters:
  //   test_plane - [in]
  //   tolerance - [in] tolerance to use when checking
  // Returns:
  //   true if the maximum distance from the curve to the
  //   test_plane is <= tolerance.
  virtual
  ON_BOOL32 IsInPlane(
        const ON_Plane& test_plane,
        double tolerance = ON_ZERO_TOLERANCE
        ) const = 0;

  /*
  Description:
    Decide if it makes sense to close off this curve by moving 
    the endpoint to the start based on start-end gap size and length
    of curve as approximated by chord defined by 6 points.
  Parameters:
    tolerance - [in] maximum allowable distance between start and end.
                     if start - end gap is greater than tolerance, returns false
    min_abs_size - [in] if greater than 0.0 and none of the interior sampled
                     points are at least min_abs_size from start, returns false.
    min_rel_size - [in] if greater than 1.0 and chord length is less than 
                     min_rel_size*gap, returns false.
  Returns:
    true if start and end points are close enough based on above conditions.
  */

  bool IsClosable(
        double tolerance,
        double min_abs_size = 0.0,
        double min_rel_size = 10.0
        ) const;

  // Description:
  //   Test a curve to see if it is closed.
  // Returns:
  //   true if the curve is closed.
  virtual 
  ON_BOOL32 IsClosed() const;

  // Description:
  //   Test a curve to see if it is periodic.
  // Returns:
  //   true if the curve is closed and at least C2 at the start/end.
  virtual 
  ON_BOOL32 IsPeriodic() const;

  /*
  Description:
    Search for a derivatitive, tangent, or curvature 
    discontinuity.
  Parameters:
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
        If 'c', the type of continuity to test for 
        is ON::Gsmooth_continuous and the curvature changes 
        from curved to 0 or 0 to curved and there is no 
        tangency kink dtype is returns 3
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
    Test continuity at a curve parameter value.
  Parameters:
    c - [in] type of continuity to test for. Read ON::continuity
             comments for details.
    t - [in] parameter to test
    hint - [in] evaluation hint
    point_tolerance - [in] if the distance between two points is
        greater than point_tolerance, then the curve is not C0.
    d1_tolerance - [in] if the difference between two first derivatives is
        greater than d1_tolerance, then the curve is not C1.
    d2_tolerance - [in] if the difference between two second derivatives is
        greater than d2_tolerance, then the curve is not C2.
    cos_angle_tolerance - [in] default = cos(1 degree) Used only when
        c is ON::G1_continuous or ON::G2_continuous.  If the cosine
        of the angle between two tangent vectors 
        is <= cos_angle_tolerance, then a G1 discontinuity is reported.
    curvature_tolerance - [in] (default = ON_SQRT_EPSILON) Used only when
        c is ON::G2_continuous or ON::Gsmooth_continuous.  
        ON::G2_continuous:
          If K0 and K1 are curvatures evaluated
          from above and below and |K0 - K1| > curvature_tolerance,
          then a curvature discontinuity is reported.
        ON::Gsmooth_continuous:
          If K0 and K1 are curvatures evaluated from above and below
          and the angle between K0 and K1 is at least twice angle tolerance
          or ||K0| - |K1|| > (max(|K0|,|K1|) > curvature_tolerance,
          then a curvature discontinuity is reported.
  Returns:
    true if the curve has at least the c type continuity at 
    the parameter t.
  */
  virtual
  bool IsContinuous(
    ON::continuity c,
    double t, 
    int* hint = NULL,
    double point_tolerance=ON_ZERO_TOLERANCE,
    double d1_tolerance=ON_ZERO_TOLERANCE,
    double d2_tolerance=ON_ZERO_TOLERANCE,
    double cos_angle_tolerance=ON_DEFAULT_ANGLE_TOLERANCE_COSINE,
    double curvature_tolerance=ON_SQRT_EPSILON
    ) const;


  // Description:
  //   Reverse the direction of the curve.
  // Returns:
  //   true if curve was reversed.
  // Remarks:
  //   If reveresed, the domain changes from [a,b] to [-b,-a]
  virtual 
  ON_BOOL32 Reverse()=0;


  /*
  Description:
    Force the curve to start at a specified point.
  Parameters:
    start_point - [in]
  Returns:
    true if successful.
  Remarks:
    Some end points cannot be moved.  Be sure to check return
    code.
  See Also:
    ON_Curve::SetEndPoint
    ON_Curve::PointAtStart
    ON_Curve::PointAtEnd
  */
  virtual
  ON_BOOL32 SetStartPoint(
          ON_3dPoint start_point
          );

  /*
  Description:
    Force the curve to end at a specified point.
  Parameters:
    end_point - [in]
  Returns:
    true if successful.
  Remarks:
    Some end points cannot be moved.  Be sure to check return
    code.
  See Also:
    ON_Curve::SetStartPoint
    ON_Curve::PointAtStart
    ON_Curve::PointAtEnd
  */
  virtual
  ON_BOOL32 SetEndPoint(
          ON_3dPoint end_point
          );
  
  // Description:
  //   Evaluate point at a parameter.
  // Parameters:
  //   t - [in] evaluation parameter
  // Returns:
  //   Point (location of curve at the parameter t).
  // Remarks:
  //   No error handling.
  // See Also:
  //   ON_Curve::EvPoint
  //   ON_Curve::PointAtStart
  //   ON_Curve::PointAtEnd
  ON_3dPoint  PointAt( 
                double t 
                ) const;

  // Description:
  //   Evaluate point at the start of the curve.
  // Parameters:
  //   t - [in] evaluation parameter
  // Returns:
  //   Point (location of the start of the curve.)
  // Remarks:
  //   No error handling.
  // See Also:
  //   ON_Curve::PointAt
  ON_3dPoint  PointAtStart() const;

  // Description:
  //   Evaluate point at the end of the curve.
  // Parameters:
  //   t - [in] evaluation parameter
  // Returns:
  //   Point (location of the end of the curve.)
  // Remarks:
  //   No error handling.
  // See Also:
  //   ON_Curve::PointAt
  ON_3dPoint  PointAtEnd() const;

  // Description:
  //   Evaluate first derivative at a parameter.
  // Parameters:
  //   t - [in] evaluation parameter
  // Returns:
  //   First derivative of the curve at the parameter t.
  // Remarks:
  //   No error handling.
  // See Also:
  //   ON_Curve::Ev1Der
  ON_3dVector DerivativeAt(
                double t 
                ) const;

  // Description:
  //   Evaluate unit tangent vector at a parameter.
  // Parameters:
  //   t - [in] evaluation parameter
  // Returns:
  //   Unit tangent vector of the curve at the parameter t.
  // Remarks:
  //   No error handling.
  // See Also:
  //   ON_Curve::EvTangent
  ON_3dVector TangentAt(
                double t 
                ) const;

  // Description:
  //   Evaluate the curvature vector at a parameter.
  // Parameters:
  //   t - [in] evaluation parameter
  // Returns:
  //   curvature vector of the curve at the parameter t.
  // Remarks:
  //   No error handling.
  // See Also:
  //   ON_Curve::EvCurvature
  ON_3dVector CurvatureAt(
                double t
                ) const;

  // Description:
  //   Return a 3d frame at a parameter.
  // Parameters:
  //   t - [in] evaluation parameter
  //   plane - [out] the frame is returned here
  // Returns:
  //   true if successful
  // See Also:
  //   ON_Curve::PointAt, ON_Curve::TangentAt,
  //   ON_Curve::Ev1Der, Ev2Der
  ON_BOOL32 FrameAt( double t, ON_Plane& plane) const;

  // Description:
  //   Evaluate point at a parameter with error checking.
  // Parameters:
  //   t - [in] evaluation parameter
  //   point - [out] value of curve at t
  //   side - [in] optional - determines which side to evaluate from
  //               =0   default
  //               <0   to evaluate from below, 
  //               >0   to evaluate from above
  //   hint - [in/out] optional evaluation hint used to speed repeated evaluations
  // Returns:
  //   false if unable to evaluate.
  // See Also:
  //   ON_Curve::PointAt
  //   ON_Curve::EvTangent
  //   ON_Curve::Evaluate
  ON_BOOL32 EvPoint(
         double t,
         ON_3dPoint& point, 
         int side = 0,
         int* hint = 0
         ) const;

  // Description:
  //   Evaluate first derivative at a parameter with error checking.
  // Parameters:
  //   t - [in] evaluation parameter
  //   point - [out] value of curve at t
  //   first_derivative - [out] value of first derivative at t
  //   side - [in] optional - determines which side to evaluate from
  //               =0   default
  //               <0   to evaluate from below, 
  //               >0   to evaluate from above
  //   hint - [in/out] optional evaluation hint used to speed repeated evaluations
  // Returns:
  //   false if unable to evaluate.
  // See Also:
  //   ON_Curve::EvPoint
  //   ON_Curve::Ev2Der
  //   ON_Curve::EvTangent
  //   ON_Curve::Evaluate
  ON_BOOL32 Ev1Der(
         double t,
         ON_3dPoint& point,
         ON_3dVector& first_derivative,
         int side = 0,
         int* hint = 0
         ) const;

  // Description:
  //   Evaluate second derivative at a parameter with error checking.
  // Parameters:
  //   t - [in] evaluation parameter
  //   point - [out] value of curve at t
  //   first_derivative - [out] value of first derivative at t
  //   second_derivative - [out] value of second derivative at t
  //   side - [in] optional - determines which side to evaluate from
  //               =0   default
  //               <0   to evaluate from below, 
  //               >0   to evaluate from above
  //   hint - [in/out] optional evaluation hint used to speed repeated evaluations
  // Returns:
  //   false if unable to evaluate.
  // See Also:
  //   ON_Curve::Ev1Der
  //   ON_Curve::EvCurvature
  //   ON_Curve::Evaluate
  ON_BOOL32 Ev2Der(
         double t,
         ON_3dPoint& point,
         ON_3dVector& first_derivative,
         ON_3dVector& second_derivative,
         int side = 0,
         int* hint = 0
         ) const;

  /*
  Description:
    Evaluate unit tangent at a parameter with error checking.
  Parameters:
    t - [in] evaluation parameter
    point - [out] value of curve at t
    tangent - [out] value of unit tangent
    side - [in] optional - determines which side to evaluate from
                =0   default
                <0   to evaluate from below, 
                >0   to evaluate from above
    hint - [in/out] optional evaluation hint used to speed repeated evaluations
  Returns:
    false if unable to evaluate.
  See Also:
    ON_Curve::TangentAt
    ON_Curve::Ev1Der
  */
  ON_BOOL32 EvTangent(
         double t,
         ON_3dPoint& point,
         ON_3dVector& tangent,
         int side = 0,
         int* hint = 0
         ) const;

  /*
  Description:
    Evaluate unit tangent and curvature at a parameter with error checking.
  Parameters:
    t - [in] evaluation parameter
    point - [out] value of curve at t
    tangent - [out] value of unit tangent
    kappa - [out] value of curvature vector
    side - [in] optional - determines which side to evaluate from
                =0   default
                <0   to evaluate from below, 
                >0   to evaluate from above
    hint - [in/out] optional evaluation hint used to speed repeated evaluations
  Returns:
    false if unable to evaluate.
  See Also:
    ON_Curve::CurvatureAt
    ON_Curve::Ev2Der
    ON_EvCurvature
  */
  ON_BOOL32 EvCurvature(
         double t,
         ON_3dPoint& point,
         ON_3dVector& tangent,
         ON_3dVector& kappa,
         int side = 0,
         int* hint = 0
         ) const;

  /*
  Description:
    This evaluator actually does all the work.  The other ON_Curve
    evaluation tools call this virtual function.
  Parameters:
    t - [in] evaluation parameter ( usually in Domain() ).
    der_count - [in] (>=0) number of derivatives to evaluate
    v_stride - [in] (>=Dimension()) stride to use for the v[] array
    v - [out] array of length (der_count+1)*v_stride
        curve(t) is returned in (v[0],...,v[m_dim-1]),
        curve'(t) is retuned in (v[v_stride],...,v[v_stride+m_dim-1]),
        curve"(t) is retuned in (v[2*v_stride],...,v[2*v_stride+m_dim-1]),
        etc.
    side - [in] optional - determines which side to evaluate from
                =0   default
                <0   to evaluate from below, 
                >0   to evaluate from above
    hint - [in/out] optional evaluation hint used to speed repeated evaluations
  Returns:
    false if unable to evaluate.
  See Also:
    ON_Curve::EvPoint
    ON_Curve::Ev1Der
    ON_Curve::Ev2Der
  */
  virtual 
  ON_BOOL32 Evaluate(
         double t,
         int der_count,
         int v_stride,
         double* v,
         int side = 0,
         int* hint = 0
         ) const = 0;

  
  /*
  Parameters:
    min_length -[in]
      minimum length of a linear span
    tolerance -[in]
      distance tolerance to use when checking linearity.
  Returns 
    true if the span is a non-degenrate line.  This means:
    - dimension = 2 or 3
    - The length of the the line segment from the span's initial 
      point to the span's control point is >= min_length.
    - The maximum distance from the line segment to the span
    is <= tolerance and the span increases monotonically
    in the direction of the line segment.
  */
  bool FirstSpanIsLinear( 
    double min_length,
    double tolerance
    ) const;

  bool LastSpanIsLinear( 
    double min_length,
    double tolerance
    ) const;

  bool FirstSpanIsLinear( 
    double min_length,
    double tolerance,
    ON_Line* span_line
    ) const;

  bool LastSpanIsLinear( 
    double min_length,
    double tolerance,
    ON_Line* span_line
    ) const;

  // Description:
  //   Removes portions of the curve outside the specified interval.
  // Parameters:
  //   domain - [in] interval of the curve to keep.  Portions of the
  //      curve before curve(domain[0]) and after curve(domain[1]) are
  //      removed.
  // Returns:
  //   true if successful.
  virtual
  ON_BOOL32 Trim(
    const ON_Interval& domain
    );

  // Description:
  //   Pure virtual function. Default returns false.
  //   Where possible, analytically extends curve to include domain.
  // Parameters:
  //   domain - [in] if domain is not included in curve domain, 
  //   curve will be extended so that its domain includes domain.  
  //   Will not work if curve is closed. Original curve is identical
  //   to the restriction of the resulting curve to the original curve domain, 
  // Returns:
  //   true if successful.
  virtual
  bool Extend(
    const ON_Interval& domain
    );

  /*
  Description:
    Splits (divides) the curve at the specified parameter.  
    The parameter must be in the interior of the curve's domain.
    The pointers passed to Split must either be NULL or point to
    an ON_Curve object of the same type.  If the pointer is NULL,
    then a curve will be created in Split().  You may pass "this"
    as left_side or right_side.
  Parameters:
    t - [in] parameter to split the curve at in the
             interval returned by Domain().
    left_side - [out] left portion of curve returned here
    right_side - [out] right portion of curve returned here
	Returns:
		true	- The curve was split into two pieces.  
		false - The curve could not be split.  For example if the parameter is
						too close to an endpoint.

  Example:
    For example, if crv were an ON_NurbsCurve, then

          ON_NurbsCurve right_side;
          crv.Split( crv.Domain().Mid() &crv, &right_side );

    would split crv at the parametric midpoint, put the left side
    in crv, and return the right side in right_side.
  */
  virtual
  ON_BOOL32 Split(
      double t,
      ON_Curve*& left_side,
      ON_Curve*& right_side
    ) const;

  /*
  Description:
    Get a NURBS curve representation of this curve.
  Parameters:
    nurbs_curve - [out] NURBS representation returned here
    tolerance - [in] tolerance to use when creating NURBS
        representation.
    subdomain - [in] if not NULL, then the NURBS representation
        for this portion of the curve is returned.
  Returns:
    0   unable to create NURBS representation
        with desired accuracy.
    1   success - returned NURBS parameterization
        matches the curve's to wthe desired accuracy
    2   success - returned NURBS point locus matches
        the curve's to the desired accuracy and the
        domain of the NURBS curve is correct.  On
        However, This curve's parameterization and
        the NURBS curve parameterization may not 
        match to the desired accuracy.  This situation
        happens when getting NURBS representations of
        curves that have a transendental parameterization
        like circles
  Remarks:
    This is a low-level virtual function.  If you do not need
    the parameterization information provided by the return code,
    then ON_Curve::NurbsCurve may be easier to use.
  See Also:
    ON_Curve::NurbsCurve
  */
  virtual
  int GetNurbForm(
        ON_NurbsCurve& nurbs_curve,
        double tolerance = 0.0,
        const ON_Interval* subdomain = NULL
        ) const;
  /*
  Description:
    Does a NURBS curve representation of this curve.
  Parameters:
  Returns:
    0   unable to create NURBS representation
        with desired accuracy.
    1   success - NURBS parameterization
        matches the curve's to wthe desired accuracy
    2   success - NURBS point locus matches
        the curve's and the
        domain of the NURBS curve is correct.  
        However, This curve's parameterization and
        the NURBS curve parameterization may not 
        match.  This situation
        happens when getting NURBS representations of
        curves that have a transendental parameterization
        like circles
  Remarks:
    This is a low-level virtual function.  
  See Also:
    ON_Curve::GetNurbForm
    ON_Curve::NurbsCurve
  */
  virtual
  int HasNurbForm() const;

  /*
  Description:
    Get a NURBS curve representation of this curve.
  Parameters:
    pNurbsCurve - [in/out] if not NULL, this ON_NurbsCurve
    will be used to store the NURBS representation
    of the curve will be returned.
    tolerance - [in] tolerance to use when creating NURBS
        representation.
    subdomain - [in] if not NULL, then the NURBS representation
        for this portion of the curve is returned.
  Returns:
    NULL or a NURBS representation of the curve.
  Remarks:
    See ON_Surface::GetNurbForm for important details about
    the NURBS surface parameterization.
  See Also:
    ON_Curve::GetNurbForm
  */
  ON_NurbsCurve* NurbsCurve(
        ON_NurbsCurve* pNurbsCurve = NULL,
        double tolerance = 0.0,
        const ON_Interval* subdomain = NULL
        ) const;

  // Description:
  //   Convert a NURBS curve parameter to a curve parameter
  //
  // Parameters:
  //   nurbs_t - [in] nurbs form parameter
  //   curve_t - [out] curve parameter
  //
  // Remarks:
  //   If GetNurbForm returns 2, this function converts the curve
  //   parameter to the NURBS curve parameter.
  //
  // See Also:
  //   ON_Curve::GetNurbForm, ON_Curve::GetNurbFormParameterFromCurveParameter
  virtual
  ON_BOOL32 GetCurveParameterFromNurbFormParameter(
        double nurbs_t,
        double* curve_t
        ) const;

  // Description:
  //   Convert a curve parameter to a NURBS curve parameter.
  //
  // Parameters:
  //   curve_t - [in] curve parameter
  //   nurbs_t - [out] nurbs form parameter
  //
  // Remarks:
  //   If GetNurbForm returns 2, this function converts the curve
  //   parameter to the NURBS curve parameter.
  //
  // See Also:
  //   ON_Curve::GetNurbForm, ON_Curve::GetCurveParameterFromNurbFormParameter
  virtual
  ON_BOOL32 GetNurbFormParameterFromCurveParameter(
        double curve_t,
        double* nurbs_t
        ) const;


  // Description:
  //   Destroys the runtime curve tree used to speed closest
  //   point and intersection calcuations.
  // Remarks:
  //   If the geometry of the curve is modified in any way,
  //   then call DestroyCurveTree();  The curve tree is 
  //   created as needed.
  void DestroyCurveTree();

  /*
	Description:
		Lookup a parameter in the m_t array, optionally using a built in snap tolerance to 
		snap a parameter value to an element of m_t.
		This function is used by some types derived from ON_Curve to snap parameter values
	Parameters:
		t			- [in]	parameter
		index -[out]	index into m_t such that
					  			if function returns false then
								   
									 @table  
									 value                  condition
						  			-1									 t<m_t[0] or m_t is empty				
										0<=i<=m_t.Count()-2		m_t[i] < t < m_t[i+1]			
										m_t.Count()-1					t>m_t[ m_t.Count()-1]			 

									if the function returns true then t is equal to, or is closest to and 
									within  tolerance of m_t[index]. 
									
		bEnableSnap-[in] enable snapping 
		m_t				-[in]	Array of parameter values to snap to
		RelTol		-[in] tolerance used in snapping
	
	Returns:		
		true if the t is exactly equal to (bEnableSnap==false), or within tolerance of
		(bEnableSnap==true) m_t[index]. 
  */
protected:
  bool ParameterSearch( double t, int& index, bool bEnableSnap, const ON_SimpleArray<double>& m_t, 
															double RelTol=ON_SQRT_EPSILON) const;

private:
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_Curve*>;
#pragma warning( pop )
#endif

class ON_CLASS ON_CurveArray : public ON_SimpleArray<ON_Curve*>
{
public:
  ON_CurveArray( int = 0 );
  ~ON_CurveArray(); // deletes any non-NULL curves

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  void Destroy(); // deletes curves, sets pointers to NULL, sets count to zero

  bool Duplicate( ON_CurveArray& ) const; // operator= copies the pointer values
                                          // duplicate copies the curves themselves

  /*
	Description:
    Get tight bounding box of the bezier.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      tight bounding box of the bezier curve.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      bezier is calculated.  The bezier curve is not modified.
	Returns:
    True if the returned tight_bbox is set to a valid 
    bounding box.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;
};

/*
Description:
  Trim a curve.
Parameters:
  curve - [in] curve to trim (not modified)
  trim_parameters - [in] trimming parameters
    If curve is open, then  trim_parameters must be an increasing 
    interval.If curve is closed, and trim_parameters ins a 
    decreasing interval, then the portion of the curve across the
    start/end is returned.
Returns:
  trimmed curve or NULL if input is invalid.
*/
ON_DECL
ON_Curve* ON_TrimCurve( 
            const ON_Curve& curve,
            ON_Interval trim_parameters
            );

/*
Description:
  Move ends of curves to a common point. Neither curve can be closed or an ON_CurveProxy.
  If one is an arc or polycurve with arc at end to change, and the other is not, 
  then the arc is left unchanged and the other curve is moved to the arc endpoint. 
  Otherwise, both are moved to the midpoint of the segment between the ends.
Parameters:
  Crv0 - [in] first curve to modify.
         [out] with one endpoint possibly changed.
  end0 - [in] if 0, change start of Crv0.  Otherwise change end.
  Crv1 - [in] second curve to modify.
         [out] with one endpoint possibly changed.
  end1 - [in] if 0, change start of Crv1.  Otherwise change end.
Returns:
  true if the endpoints match. Falsse otherwise,
*/
ON_DECL
bool ON_ForceMatchCurveEnds(
                            ON_Curve& Crv0, 
                            int end0, 
                            ON_Curve& Crv1, 
                            int end1
                            );

/*
Description:
  Join all contiguous curves of an array of ON_Curves.
Parameters:
  InCurves - [in] Array of curves to be joined (not modified)
  OutCurves - [out] Resulting joined curves and copies of curves that were not joined to anything
                    are appended.
  join_tol - [in] Distance tolerance used to decide if endpoints are close enough
  bPreserveDirection - [in] If true, curve endpoints will be compared to curve startpoints.
                            If false, all start and endpoints will be compared, and copies of input 
                            curves may be reversed in output.
  key     -  [out] if key is not null, InCurves[i] was joined into OutCurves[key[i]].
Returns:
  Number of curves added to Outcurves
Remarks:
  Closed curves are copied to OutCurves. 
  Curves that cannot be joined to others are copied to OutCurves.  When curves are joined, the results
  are ON_PolyCurves. All members of InCurves must have same dimension, at most 3.
  */
ON_DECL
int ON_JoinCurves(const ON_SimpleArray<const ON_Curve*>& InCurves,
                  ON_SimpleArray<ON_Curve*>& OutCurves,
                  double join_tol,
                  bool bPreserveDirection = false,
                  ON_SimpleArray<int>* key = 0
                 );


/*
Description:
  Sort a list of lines so they are geometrically continuous.
Parameters:
  line_count - [in] number of lines
  line_list  - [in] array of lines
  index       - [out] The input index[] is an array of line_count unused integers.
                      The returned index[] is a permutation of {0,1,...,line_count-1}
                      so that the list of lines is in end-to-end order.
  bReverse    - [out] The input bReverse[] is an array of line_count unused bools.
                      If the returned value of bReverse[j] is true, then
                      line_list[index[j]] needs to be reversed.
Returns:
  True if successful, false if not.
*/
ON_DECL
bool ON_SortLines( 
        int line_count, 
        const ON_Line* line_list, 
        int* index, 
        bool* bReverse 
        );

/*
Description:
  Sort a list of lines so they are geometrically continuous.
Parameters:
  line_list  - [in] array of lines
  index       - [out] The input index[] is an array of line_count unused integers.
                      The returned index[] is a permutation of {0,1,...,line_count-1}
                      so that the list of lines is in end-to-end order.
  bReverse    - [out] The input bReverse[] is an array of line_count unused bools.
                      If the returned value of bReverse[j] is true, then
                      line_list[index[j]] needs to be reversed.
Returns:
  True if successful, false if not.
*/
ON_DECL
bool ON_SortLines( 
        const ON_SimpleArray<ON_Line>& line_list,
        int* index, 
        bool* bReverse 
        );

/*
Description:
  Sort a list of open curves so end of a curve matches the start of the next curve.
Parameters:
  curve_count - [in] number of curves
  curve_list  - [in] array of curve pointers
  index       - [out] The input index[] is an array of curve_count unused integers.
                      The returned index[] is a permutation of {0,1,...,curve_count-1}
                      so that the list of curves is in end-to-end order.
  bReverse    - [out] The input bReverse[] is an array of curve_count unused bools.
                      If the returned value of bReverse[j] is true, then
                      curve_list[index[j]] needs to be reversed.
Returns:
  True if successful, false if not.
*/
ON_DECL
bool ON_SortCurves(
          int curve_count,
          const ON_Curve* const* curve_list, 
          int* index,
          bool* bReverse
          );

/*
Description:
  Sort a list of curves so end of a curve matches the start of the next curve.
Parameters:
  curve       - [in] array of curves to sort.  The curves themselves are not modified.
  index       - [out] The input index[] is an array of curve_count unused integers.
                      The returned index[] is a permutation of {0,1,...,curve_count-1}
                      so that the list of curves is in end-to-end order.
  bReverse    - [out] The input bReverse[] is an array of curve_count unused bools.
                      If the returned value of bReverse[j] is true, then
                      curve[index[j]] needs to be reversed.
Returns:
  True if successful, false if not.
*/
ON_DECL
bool ON_SortCurves( 
                   const ON_SimpleArray<const ON_Curve*>& curves, 
                   ON_SimpleArray<int>& index, 
                   ON_SimpleArray<bool>& bReverse 
                   );

/*
Description:
  Sort a list of curves so end of a curve matches the start of the next curve.
Parameters:
  curve_count - [in] number of curves
  curve       - [in] array of curve pointers
  index       - [out] The input index[] is an array of curve_count unused integers.
                      The returned index[] is a permutation of {0,1,...,curve_count-1}
                      so that the list of curves is in end-to-end order.
  bReverse    - [out] The input bReverse[] is an array of curve_count unused bools.
                      If the returned value of bReverse[j] is true, then
                      curve[index[j]] needs to be reversed.
Returns:
  True if successful, false if not.
*/
ON_DECL
bool ON_SortCurves( 
          const ON_SimpleArray<ON_Curve*>& curves, 
          ON_SimpleArray<int>& index, 
          ON_SimpleArray<bool>& bReverse 
          );

/*
Description:
  Determine the orientaion (counterclockwise or clockwise) of a closed
  planar curve.
Paramters:
  curve - [in] simple (no self intersections) closed planar curve
  xform - [in] Transformation to map the curve to the xy plane. If the
               curve is parallel to the xy plane, you may pass NULL.
Returns:
  +1: The curve's orientation is counter clockwise in the xy plane.
  -1: The curve's orientation is clockwise in the xy plane.
   0: Unable to compute the curve's orientation.
*/
ON_DECL
int ON_ClosedCurveOrientation( const ON_Curve& curve, const ON_Xform* xform );

#endif

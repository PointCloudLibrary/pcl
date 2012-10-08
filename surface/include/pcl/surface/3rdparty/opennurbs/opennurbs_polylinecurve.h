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

#if !defined(OPENNURBS_CURVE_POLYLINE_INC_)
#define OPENNURBS_CURVE_POLYLINE_INC_

class ON_PolylineCurve;
class ON_CLASS ON_PolylineCurve : public ON_Curve
{
  ON_OBJECT_DECLARE(ON_PolylineCurve);

public:
  ON_PolylineCurve();
  ON_PolylineCurve(const ON_3dPointArray&);
  ON_PolylineCurve(const ON_PolylineCurve&);
	ON_PolylineCurve& operator=(const ON_PolylineCurve&);
	ON_PolylineCurve& operator=(const ON_3dPointArray&);

  virtual ~ON_PolylineCurve();

  // Description:
  //   Call if memory used by ON_PolylineCurve becomes invalid.
  void EmergencyDestroy(); 

  
  /////////////////////////////////////////////////////////////////
  // ON_Object overrides

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

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

  // Description:
  //   virtual ON_Object::Dump override
  void Dump( 
    ON_TextLog& dump
    ) const;

  // Description:
  //   virtual ON_Object::Write override
  ON_BOOL32 Write(
         ON_BinaryArchive& binary_archive
       ) const;

  // Description:
  //   virtual ON_Object::Read override
  ON_BOOL32 Read(
         ON_BinaryArchive& binary_archive
       );

  /////////////////////////////////////////////////////////////////
  // ON_Geometry overrides

  // Description:
  //   virtual ON_Geometry::Dimension override
  // Returns:
  //   value of m_dim
  int Dimension() const;

  // Description:
  //   virtual ON_Geometry::GetBBox override
  //   Calculates axis aligned bounding box.
  // Parameters:
  //   boxmin - [in/out] array of Dimension() doubles
  //   boxmax - [in/out] array of Dimension() doubles
  //   bGrowBox - [in] (default=false) 
  //     If true, then the union of the input bbox and the 
  //     object's bounding box is returned in bbox.  
  //     If false, the object's bounding box is returned in bbox.
  // Returns:
  //   true if object has bounding box and calculation was successful
  ON_BOOL32 GetBBox( // returns true if successful
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
      polyline's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      polyline is calculated.  The polyline is not modified.
	Returns:
    True if a valid tight_bbox is returned.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  // Description:
  //   virtual ON_Geometry::Transform override.
  //   Transforms the NURBS curve.
  //
  // Parameters:
  //   xform - [in] transformation to apply to object.
  //
  // Remarks:
  //   When overriding this function, be sure to include a call
  //   to ON_Object::TransformUserData() which takes care of 
  //   transforming any ON_UserData that may be attached to 
  //   the object.
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  // Description:
  //   virtual ON_Geometry::SwapCoordinates override.
  //   Swaps control point coordinate values with indices i and j.
  // Parameters:
  //   i - [in] coordinate index
  //   j - [in] coordinate index
  ON_BOOL32 SwapCoordinates(
        int i, 
        int j
        );

  /////////////////////////////////////////////////////////////////
  // ON_Curve overrides

  // Description:
  //   virtual ON_Curve::Domain override.
  // Returns:
  //   domain of the polyline curve.
  ON_Interval Domain() const;

  // Description:
  //   virtual ON_Curve::SetDomain override.
  //   Set the domain of the curve
  // Parameters:
  //   t0 - [in]
  //   t1 - [in] new domain will be [t0,t1]
  // Returns:
  //   true if successful.
  ON_BOOL32 SetDomain(
        double t0, 
        double t1 
        );

  bool ChangeDimension(
          int desired_dimension
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
  Remarks:
    Overrides virtual ON_Curve::ChangeClosedCurveSeam
  */
  ON_BOOL32 ChangeClosedCurveSeam( 
            double t 
            );

  // Description:
  //   virtual ON_Curve::SpanCount override.
  //   Get number of segments in polyline.
  // Returns:
  //   Number of segments in polyline.
  int SpanCount() const;

  // Description:
  //   virtual ON_Curve::GetSpanVector override.
  //   Get list of parameters at polyline points.
  // Parameters:
  //   knot_values - [out] an array of length SpanCount()+1 is 
  //       filled in with the parameter values.  knot_values[i]
  //       is the parameter for the point m_pline[i].
  // Returns:
  //   true if successful
  ON_BOOL32 GetSpanVector(
         double* knot_values
         ) const;

  // Description:
  //   virtual ON_Curve::Degree override.
  // Returns:
  //   1
  int Degree() const; 

  // Description:
  //   virtual ON_Curve::IsLinear override.
  // Returns:
  //   true if all the polyline points are within tolerance
  //   of the line segment connecting the ends of the polyline.
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
  int IsPolyline(
        ON_SimpleArray<ON_3dPoint>* pline_points = NULL,
        ON_SimpleArray<double>* pline_t = NULL
        ) const;

  // Description:
  //   virtual ON_Curve::IsArc override.
  // Returns:
  //   false for all polylines.
  ON_BOOL32 IsArc(
        const ON_Plane* plane = NULL,
        ON_Arc* arc = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  // Description:
  //   virtual ON_Curve::IsPlanar override.
  // Returns:
  //   true if the polyline is planar.
  ON_BOOL32 IsPlanar(
        ON_Plane* plane = NULL,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  // Description:
  //   virtual ON_Curve::IsInPlane override.
  // Returns:
  //   true if every point in the polyline is within 
  //   tolerance of the test_plane.
  ON_BOOL32 IsInPlane(
        const ON_Plane& test_plane,
        double tolerance = ON_ZERO_TOLERANCE
        ) const;

  // Description:
  //   virtual ON_Curve::IsClosed override.
  // Returns:
  //   true if the polyline has 4 or more point, the
  //   first point and the last point are equal, and
  //   some other point is distinct from the first and
  //   last point.
  ON_BOOL32 IsClosed() const;

  // Description:
  //   virtual ON_Curve::IsPeriodic override.
  // Returns:
  //   false for all polylines.
  ON_BOOL32 IsPeriodic(  // true if curve is a single periodic segment
        void 
        ) const;
  
  /*
  Description:
    Search for a derivatitive, tangent, or curvature discontinuity.
  Parameters:
    c - [in] type of continity to test for.  If ON::C1_continuous
    t0 - [in] search begins at t0
    t1 - [in] (t0 < t1) search ends at t1
    t - [out] if a discontinuity is found, the *t reports the
          parameter at the discontinuity.
    hint - [in/out] if GetNextDiscontinuity will be called repeatedly,
       passing a "hint" with initial value *hint=0 will increase the speed
       of the search.       
    dtype - [out] if not NULL, *dtype reports the kind of discontinuity
        found at *t.  A value of 1 means the first derivative or unit tangent
        was discontinuous.  A value of 2 means the second derivative or
        curvature was discontinuous.
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
    true if a discontinuity was found on the interior of the interval (t0,t1).
  Remarks:
    Overrides ON_Curve::GetNextDiscontinuity.
  */
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
    c - [in] continuity to test for
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
    true if the curve has at least the c type continuity at the parameter t.
  Remarks:
    Overrides ON_Curve::IsContinuous.
  */
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
  //   virtual ON_Curve::Reverse override.
  //   Reverse parameterizatrion by negating all m_t values
  //   and reversing the order of the m_pline points.
  // Remarks:
  //   Domain changes from [a,b] to [-b,-a]
  ON_BOOL32 Reverse();

  /*
  Description:
    Force the curve to start at a specified point.
  Parameters:
    start_point - [in]
  Returns:
    true if successful.
  Remarks:
    Some start points cannot be moved.  Be sure to check return
    code.
  See Also:
    ON_Curve::SetEndPoint
    ON_Curve::PointAtStart
    ON_Curve::PointAtEnd
  */
  // virtual
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
  //virtual
  ON_BOOL32 SetEndPoint(
          ON_3dPoint end_point
          );

  ON_BOOL32 Evaluate( // returns false if unable to evaluate
         double,         // evaluation parameter
         int,            // number of derivatives (>=0)
         int,            // array stride (>=Dimension())
         double*,        // array of length stride*(ndir+1)
         int = 0,        // optional - determines which side to evaluate from
                         //         0 = default
                         //      <  0 to evaluate from below, 
                         //      >  0 to evaluate from above
         int* = 0        // optional - evaluation hint (int) used to speed
                         //            repeated evaluations
         ) const;

  // Description:
  //   virtual ON_Curve::Trim override.
  ON_BOOL32 Trim( const ON_Interval& );

  // Description:
  //   Where possible, analytically extends curve to include domain.
  // Parameters:
  //   domain - [in] if domain is not included in curve domain, 
  //   curve will be extended so that its domain includes domain.  
  //   Will not work if curve is closed. Original curve is identical
  //   to the restriction of the resulting curve to the original curve domain, 
  // Returns:
  //   true if successful.
  bool Extend(
    const ON_Interval& domain
    );

  // Description:
  //   virtual ON_Curve::Split override.
  //
  // Split() divides the polyline at the specified parameter.  The parameter
  // must be in the interior of the curve's domain.  The pointers passed
  // to ON_NurbsCurve::Split must either be NULL or point to an ON_NurbsCurve.
  // If the pointer is NULL, then a curve will be created
  // in Split().  You may pass "this" as one of the pointers to Split().
  // For example, 
  //
  //   ON_NurbsCurve right_side;
  //   crv.Split( crv.Domain().Mid() &crv, &right_side );
  //
  // would split crv at the parametric midpoint, put the left side in crv,
  // and return the right side in right_side.
  ON_BOOL32 Split(
      double,    // t = curve parameter to split curve at
      ON_Curve*&, // left portion returned here (must be an ON_NurbsCurve)
      ON_Curve*&  // right portion returned here (must be an ON_NurbsCurve)
    ) const;

  int GetNurbForm( // returns 0: unable to create NURBS representation
                   //            with desired accuracy.
                   //         1: success - returned NURBS parameterization
                   //            matches the curve's to wthe desired accuracy
                   //         2: success - returned NURBS point locus matches
                   //            the curve's to the desired accuracy but, on
                   //            the interior of the curve's domain, the 
                   //            curve's parameterization and the NURBS
                   //            parameterization may not match to the 
                   //            desired accuracy.
        ON_NurbsCurve&,
        double = 0.0,
        const ON_Interval* = NULL     // OPTIONAL subdomain of polyline
        ) const;

  int HasNurbForm( // returns 0: unable to create NURBS representation
                   //            with desired accuracy.
                   //         1: success - returned NURBS parameterization
                   //            matches the curve's to wthe desired accuracy
                   //         2: success - returned NURBS point locus matches
                   //            the curve's to the desired accuracy but, on
                   //            the interior of the curve's domain, the 
                   //            curve's parameterization and the NURBS
                   //            parameterization may not match to the 
                   //            desired accuracy.
        ) const;

  // virtual ON_Curve::GetCurveParameterFromNurbFormParameter override
  ON_BOOL32 GetCurveParameterFromNurbFormParameter(
        double, // nurbs_t
        double* // curve_t
        ) const;

  // virtual ON_Curve::GetNurbFormParameterFromCurveParameter override
  ON_BOOL32 GetNurbFormParameterFromCurveParameter(
        double, // curve_t
        double* // nurbs_t
        ) const;
/*
	Description:
		Lookup a parameter in the m_t array, optionally using a built in snap tolerance to 
		snap a parameter value to an element of m_t.
	Parameters:
		t - [in]	  	parameter
		index -[out]	index into m_t such that
					  			if function returns false then value of index is
								   
									 @table  
									 value of index              condition
						  			  -1									  t<m_t[0] or m_t is empty				
										  0<=i<=m_t.Count()-2		m_t[i] < t < m_t[i+1]			
										  m_t.Count()-1					t>m_t[ m_t.Count()-1]			 

									if the function returns true then t is equal to, or is closest to and 
									within  tolerance of m_t[index]. 
									
		bEnableSnap-[in] enable snapping 	
	Returns:		
		true if the t is exactly equal to, or within tolerance of
		(only if bEnableSnap==true) m_t[index]. 
*/ 
	bool ParameterSearch(double t, int& index, bool bEnableSnap) const;

  bool Append( const ON_PolylineCurve& );

  /////////////////////////////////////////////////////////////////
  // Interface
	public:
  int PointCount() const; // number of points in polyline

  ON_Polyline            m_pline;
  ON_SimpleArray<double> m_t;    // parameters
  int                    m_dim;  // 2 or 3 (2 so ON_PolylineCurve can be uses as a trimming curve)
};


#endif

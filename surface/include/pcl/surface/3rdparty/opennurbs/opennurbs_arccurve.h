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

#if !defined(ON_GEOMETRY_CURVE_ARC_INC_)
#define ON_GEOMETRY_CURVE_ARC_INC_


/*
Description:
  ON_ArcCurve is used to represent arcs and circles.
  ON_ArcCurve.IsCircle() returns true if the curve
  is a complete circle.
Details:
	an ON_ArcCurve is a subcurve of a circle, with a 
	constant speed parameterization. The parameterization is
	an affine linear reparameterzation of the underlying arc
	m_arc onto the domain m_t.
	
	A valid ON_ArcCurve has Radius()>0 and  0<AngleRadians()<=2*PI
	and a strictly increasing Domain(). 
*/
class ON_CLASS ON_ArcCurve : public ON_Curve
{
  ON_OBJECT_DECLARE(ON_ArcCurve);

public:
  ON_ArcCurve();
  ON_ArcCurve(const ON_ArcCurve&);
  virtual ~ON_ArcCurve();

  // virtual ON_Object::SizeOf override
  unsigned int SizeOf() const;

  // virtual ON_Object::DataCRC override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;

  /*
  Description:
    Create an arc curve with domain (0,arc.Length()).
  */
  ON_ArcCurve( 
      const ON_Arc& arc 
      );

  /*
  Description:
    Create an arc curve with domain (t0,t1)
  */
  ON_ArcCurve(
      const ON_Arc& arc,
      double t0,
      double t1
      );

  /*
  Description:
    Creates a curve that is a complete circle with 
    domain (0,circle.Length()).
  */
  ON_ArcCurve(
      const ON_Circle& circle
      );

  /*
  Description:
    Creates a curve that is a complete circle with domain (t0,t1).
  */
  ON_ArcCurve(
      const ON_Circle& circle,
      double t0,
      double t1
      );


	ON_ArcCurve& operator=(const ON_ArcCurve&);

  /*
  Description:
    Create an arc curve with domain (0,arc.Length()).
  */
	ON_ArcCurve& operator=(const ON_Arc& arc);

  /*
  Description:
    Creates a curve that is a complete circle with 
    domain (0,circle.Length()).
  */
	ON_ArcCurve& operator=(const ON_Circle& circle);
  
  /////////////////////////////////////////////////////////////////
  // ON_Object overrides

  /*
  Description:
		A valid ON_ArcCurve has Radius()>0 and  0<AngleRadians()<=2*PI
		and a strictly increasing Domain(). 
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

  void Dump( ON_TextLog& ) const;

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

  /*
	Description:
    Get tight bounding box of the arc.
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
    True if the returned tight_bbox is set to a valid 
    bounding box.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;


  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  /////////////////////////////////////////////////////////////////
  // ON_Curve overrides

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

  ON_Interval Domain() const;

  bool ChangeDimension(
          int desired_dimension
          );

  ON_BOOL32 ChangeClosedCurveSeam( 
            double t 
            );

  int SpanCount() const; // number of smooth spans in curve

  ON_BOOL32 GetSpanVector( // span "knots" 
         double* // array of length SpanCount() + 1 
         ) const; // 

  int Degree( // returns maximum algebraic degree of any span 
                  // ( or a good estimate if curve spans are not algebraic )
    ) const; 

  ON_BOOL32 IsLinear( // true if curve locus is a line segment between
                 // between specified points
        double = ON_ZERO_TOLERANCE // tolerance to use when checking linearity
        ) const;

  ON_BOOL32 IsArc( // ON_Arc.m_angle > 0 if curve locus is an arc between
              // specified points
        const ON_Plane* = NULL, // if not NULL, test is performed in this plane
        ON_Arc* = NULL, // if not NULL and true is returned, then arc parameters
                         // are filled in
        double = 0.0    // tolerance to use when checking
        ) const;

  ON_BOOL32 IsPlanar(
        ON_Plane* = NULL, // if not NULL and true is returned, then plane parameters
                           // are filled in
        double = 0.0    // tolerance to use when checking
        ) const;

  ON_BOOL32 IsInPlane(
        const ON_Plane&, // plane to test
        double = 0.0    // tolerance to use when checking
        ) const;

  ON_BOOL32 IsClosed(  // true if curve is closed (either curve has
        void      // clamped end knots and euclidean location of start
        ) const;  // CV = euclidean location of end CV, or curve is
                  // periodic.)

  ON_BOOL32 IsPeriodic(  // true if curve is a single periodic segment
        void 
        ) const;
  
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

  ON_BOOL32 Reverse();       // reverse parameterizatrion
                        // Domain changes from [a,b] to [-b,-a]

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

  /*
  Description:
    Splits (divides) the arc at the specified parameter.  
    The parameter must be in the interior of the arc's domain.
    The ON_Curve pointers passed to ON_ArcCurve::Split must 
    either be NULL or point to ON_ArcCurve objects.
    If a pointer is NULL, then an ON_ArcCurve will be created 
    in Split().  You may pass "this" as left_side or right_side.
  Parameters:
    t - [in] parameter to split the curve at in the
             interval returned by Domain().
    left_side - [out] left portion of curve returned here.
       If not NULL, left_side must point to an ON_ArcCuve.
    right_side - [out] right portion of curve returned here
       If not NULL, right_side must point to an ON_ArcCuve.
  Remarks:
    Overrides virtual ON_Curve::Split.
  */
  virtual
  ON_BOOL32 Split(
      double t,
      ON_Curve*& left_side,
      ON_Curve*& right_side
    ) const;

  // virtual ON_Curve::GetNurbForm override
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
        const ON_Interval* = NULL     // OPTIONAL subdomain of arc curve
        ) const;

  // virtual ON_Curve::HasNurbForm override
  int HasNurbForm( // returns 0: unable to create NURBS representation
                   //            with desired accuracy.
                   //         1: success - NURBS parameterization
                   //            matches the curve's
                   //         2: success - returned NURBS point locus matches
                   //            the curve'sbut, on
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
    Returns true if this arc curve is a complete circle.
  */
  bool IsCircle() const;

  // Returns:
  //   The arc's radius.
	double Radius() const;

  // Returns:
  //   The arc's subtended angle in radians.
  double AngleRadians() const;

  // Returns:
  //   The arc's subtended angle in degrees.
  double AngleDegrees() const;


  /////////////////////////////////////////////////////////////////

  ON_Arc   m_arc;
  
  // evaluation domain (always increasing)
  // ( m_t[i] corresponds to m_arc.m_angle[i] )
  ON_Interval m_t;  

  // The dimension of a arc curve can be 2 or 3.
  // (2 so ON_ArcCurve can be used as a trimming curve)
  int m_dim;

};


#endif

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

#if !defined(ON_GEOMETRY_CURVE_LINE_INC_)
#define ON_GEOMETRY_CURVE_LINE_INC_

class ON_LineCurve;
class ON_CLASS ON_LineCurve : public ON_Curve
{
  ON_OBJECT_DECLARE(ON_LineCurve);

public:
  ON_LineCurve();
  ON_LineCurve(const ON_2dPoint&,const ON_2dPoint&); // creates a 2d line curve
  ON_LineCurve(const ON_3dPoint&,const ON_3dPoint&); // creates a 3d line curve
  ON_LineCurve(const ON_Line&);
  ON_LineCurve(const ON_Line&,
                double,double    // domain
                );
  ON_LineCurve(const ON_LineCurve&);

  virtual ~ON_LineCurve();

	ON_LineCurve& operator=(const ON_LineCurve&);
	ON_LineCurve& operator=(const ON_Line&);
  
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

  /*
	Description:
    Get tight bounding box of the line.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      line's tight bounding box.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      line is calculated.  The line is not modified.
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

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  ON_BOOL32 SwapCoordinates(
        int, int        // indices of coords to swap
        );

  /////////////////////////////////////////////////////////////////
  // ON_Curve overrides

  ON_Interval Domain() const;

  // Description:
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

  ON_BOOL32 IsArc( // ON_Arc.m_angle > 0 if curve locus is an arc between
              // specified points
        const ON_Plane* = NULL, // if not NULL, test is performed in this plane
        ON_Arc* = NULL, // if not NULL and true is returned, then arc parameters
                         // are filled in
        double = ON_ZERO_TOLERANCE    // tolerance to use when checking
        ) const;

  ON_BOOL32 IsPlanar(
        ON_Plane* = NULL, // if not NULL and true is returned, then plane parameters
                           // are filled in
        double = ON_ZERO_TOLERANCE    // tolerance to use when checking
        ) const;

  ON_BOOL32 IsInPlane(
        const ON_Plane&, // plane to test
        double = ON_ZERO_TOLERANCE    // tolerance to use when checking
        ) const;

  ON_BOOL32 IsClosed(  // true if curve is closed (either curve has
        void      // clamped end knots and euclidean location of start
        ) const;  // CV = euclidean location of end CV, or curve is
                  // periodic.)

  ON_BOOL32 IsPeriodic(  // true if curve is a single periodic segment
        void 
        ) const;
  
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

  ON_BOOL32 Reverse();       // reverse parameterizatrion
                        // Domain changes from [a,b] to [-b,-a]

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
  //   Removes portions of the curve outside the specified interval.
  // Parameters:
  //   domain - [in] interval of the curve to keep.  Portions of the
  //      curve before curve(domain[0]) and after curve(domain[1]) are
  //      removed.
  // Returns:
  //   true if successful.
  ON_BOOL32 Trim(
    const ON_Interval& domain
    );

  // Description:
  //   Where possible, analytically extends curve to include domain.
  // Parameters:
  //   domain - [in] if domain is not included in curve domain, 
  //   curve will be extended so that its domain includes domain.  
  //   Original curve is identical
  //   to the restriction of the resulting curve to the original curve domain, 
  // Returns:
  //   true if successful.
  bool Extend(
    const ON_Interval& domain
    );

  // Description:
  //   virtual ON_Curve::Split override.
  //   Divide the curve at the specified parameter.  The parameter
  //   must be in the interior of the curve's domain.  The pointers
  //   passed to Split must either be NULL or point to an ON_Curve
  //   object of the same of the same type.  If the pointer is NULL,
  //   then a curve will be created in Split().  You may pass "this"
  //   as one of the pointers to Split().
  // Parameters:
  //   t - [in] parameter in interval Domain().
  //   left_side - [out] left portion of curve
  //   right_side - [out] right portion of curve
  // Example:
  //   For example, if crv were an ON_NurbsCurve, then
  //
  //     ON_NurbsCurve right_side;
  //     crv.Split( crv.Domain().Mid() &crv, &right_side );
  //
  //   would split crv at the parametric midpoint, put the left side
  //   in crv, and return the right side in right_side.
  ON_BOOL32 Split(
      double t,    // t = curve parameter to split curve at
      ON_Curve*& left_side, // left portion returned here
      ON_Curve*& right_side // right portion returned here
    ) const;

  // Description:
  //   virtual ON_Curve::GetNurbForm override.
  //   Get a NURBS curve representation of this curve.
  // Parameters:
  //   nurbs_curve - [out] NURBS representation returned here
  //   tolerance - [in] tolerance to use when creating NURBS
  //       representation.
  //   subdomain - [in] if not NULL, then the NURBS representation
  //       for this portion of the curve is returned.
  // Returns:
  //   0   unable to create NURBS representation
  //       with desired accuracy.
  //   1   success - returned NURBS parameterization
  //       matches the curve's to wthe desired accuracy
  //   2   success - returned NURBS point locus matches
  //       the curve's to the desired accuracy but, on
  //       the interior of the curve's domain, the 
  //       curve's parameterization and the NURBS
  //       parameterization may not match to the 
  //       desired accuracy.
  int GetNurbForm(
        ON_NurbsCurve&,
        double = 0.0,
        const ON_Interval* = NULL
        ) const;

  // Description:
  //   virtual ON_Curve::HasNurbForm override.
  //   Does a NURBS curve representation of this curve exist.
  // Parameters:
  // Returns:
  //   0   unable to create NURBS representation
  //       with desired accuracy.
  //   1   success - returned NURBS parameterization
  //       matches the curve's to wthe desired accuracy
  //   2   success - returned NURBS point locus matches
  //       the curve's to the desired accuracy but, on
  //       the interior of the curve's domain, the 
  //       curve's parameterization and the NURBS
  //       parameterization may not match to the 
  //       desired accuracy.
  int HasNurbForm(
        ) const;

  // Description:
  //   virtual ON_Curve::GetCurveParameterFromNurbFormParameter override.
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
  //   virtual ON_Curve::GetNurbFormParameterFromCurveParameter override.
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

  /////////////////////////////////////////////////////////////////
  // Interface

  ON_Line m_line;
  ON_Interval m_t;  // domain
  int      m_dim;   // 2 or 3 (2 so ON_LineCurve can be uses as a trimming curve)
};


#endif

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

#pragma once

/*
  class ON_HatchLoop
  /////////////////////////////////////////////////////////////////
  Represents a 3d boundary loop curve
*/
class ON_CLASS ON_HatchLoop
{
public:
#if defined(ON_DLL_EXPORTS) || defined(ON_DLL_IMPORTS)
  // When the Microsoft CRT(s) is/are used, this is the best
  // way to prevent crashes that happen when a hatch loop is
  // allocated with new in one DLL and deallocated with
  // delete in another DLL.

  // new/delete
  void* operator new(std::size_t);
  void  operator delete(void*);

  // array new/delete
  void* operator new[] (std::size_t);
  void  operator delete[] (void*);

  // in place new/delete
  void* operator new(std::size_t,void*);
  void  operator delete(void*,void*);
#endif

  enum eLoopType
  {
    ltOuter = 0,
    ltInner = 1,
  };

  ON_HatchLoop();
  ON_HatchLoop( ON_Curve* pCurve2d, eLoopType type = ltOuter);
  ON_HatchLoop( const ON_HatchLoop& src);
  ~ON_HatchLoop();

  ON_HatchLoop& operator=( const ON_HatchLoop& src);

  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const; // for debugging
  ON_BOOL32 Write( ON_BinaryArchive&) const;
  ON_BOOL32 Read( ON_BinaryArchive&);

  // Interface
  /////////////////////////////////////////////////////////////////

  /*
  Description:
    Get a closed 2d curve boundary loop
  Parameters:
  Return:
    Pointer to loop's 2d curve
  */
  const ON_Curve* Curve() const;
 
  /*
  Description:
    Specify the 2d loop curve in the hatch's plane coordinates
  Parameters:
    curve - [in] 2d input curve
  Return:
    true: success, false, curve couldn't be duplicated
  Remarks:
    The curve is copied
  */
  bool SetCurve( const ON_Curve& curve);

  /*
  Description:
    Get the type flag of the loop
  Returns:
    eLoopType::ltInner or eLoopType::ltOuter
  */
  eLoopType Type() const;

  /*
  Description:
    Specify the type flag of the loop
  Parameters:
    type - [in] ltInner or ltOuter
  */
  void SetType( eLoopType type);

protected:
  friend class ON_Hatch;
  eLoopType m_type;         // loop type flag - inner or outer
  ON_Curve* m_p2dCurve;     // 2d closed curve bounding the hatch
                            // This is really a 3d curve with z coordinates = 0
};


/*
  class ON_HatchLine
  /////////////////////////////////////////////////////////////////
  Represents one line of a hatch pattern
  Similar to AutoCAD's .pat file definition
  ON_HatchLine's are used by ON_HatchPattern
    to specify the dashes and offset patterns of the lines.

  Each line has the following information:
  Angle is the direction of the line CCW from the x axis
  The first line origin is at base
  Each line repetition is offset by offset from the previous line
    offset.x is parallel to the line and 
    offset.y is perpendicular to the line
  The base and offset values are rotated by the line's angle to 
    produce a location in the hatch pattern's coordinate system
  There can be gaps and dashes specified for drawing the line

  If there are no dashes, the line is solid
  Negative length dashes are gaps
  Positive length dashes are drawn as line segments
*/

class ON_CLASS ON_HatchLine
{
public:
  ON_HatchLine();
  // C++ default copy construction and operator= work fine.

  ON_HatchLine( 
    double angle, 
    const ON_2dPoint& base, 
    const ON_2dVector& offset,
    const ON_SimpleArray<double> dashes);

  bool operator==( const ON_HatchLine&) const;
  bool operator!=( const ON_HatchLine&) const;

  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const; // for debugging
  ON_BOOL32 Write( ON_BinaryArchive&) const;  // serialize definition to binary archive
  ON_BOOL32 Read( ON_BinaryArchive&);  // restore definition from binary archive

  // Interface
  /////////////////////////////////////////////////////////////////

  /*
  Description:
    Get angle of the hatch line.
    CCW from x-axis
  Parameters:
  Return:
    The angle in radians
  */
  double Angle() const;

  /*
  Description:
    Set angle of the hatch line.
    CCW from x-axis
  Parameters:
    angle - [in] angle in radians
  Return:
  */
  void SetAngle( double angle);
  
  /*
  Description:
    Get this line's 2d basepoint
  Parameters:
  Return:
    the base point
  */
  ON_2dPoint Base() const;
  /*
  Description:
    Set this line's 2d basepoint
  Parameters:
    base - [in] the basepoint
  Return:
  */
  void SetBase( const ON_2dPoint& base);
  
  /*
  Description:
    Get this line's 2d offset for line repetitions
    Offset().x is shift parallel to line
    Offset().y is spacing perpendicular to line
  Parameters:
  Return:
    the offset
  */
  ON_2dVector Offset() const;

  /*
  Description:
    Get this line's 2d offset for line repetitions
    Offset().x is shift parallel to line
    Offset().y is spacing perpendicular to line
  Parameters:
    offset - [in] the shift,spacing for repeated lines
  Return:
  */
  void SetOffset( const ON_2dVector& offset);

  /*
  Description:
    Get the number of gaps + dashes in the line
  Parameters:
  Return:
    nummber of dashes in the line
  */
  int DashCount() const;

  /*
  Description:
    Get the dash length at index
  Parameters:
    index - [in] the dash to get
  Return:
    the length of the dash ( gap if negative)
  */
  double Dash( int) const;

  /*
  Description:
    Add a dash to the pattern
  Parameters:
    dash - [in] length to append - < 0 for a gap
  */
  void AppendDash( double dash);

  /*
  Description:
    Specify a new dash array
  Parameters:
    dashes - [in] array of dash lengths
  */
  void SetPattern( const ON_SimpleArray<double>& dashes);

  /*
  Description:
    Get the line's angle, base, offset and dashes 
    in one function call
  Parameters:
    angle  - [out] angle in radians CCW from x-axis
    base   - [out] origin of the master line
    offset - [out] offset for line replications
    dashes - [out] the dash array for the line
  Return:
  */
  void GetLineData(
    double& angle, 
    ON_2dPoint& base, 
    ON_2dVector& offset, 
    ON_SimpleArray<double>& dashes) const;

  /*
  Description:
    Get the total length of a pattern repeat
  Parameters:
  Return:
    Pattern length
  */
  double GetPatternLength() const;

public:
  double m_angle;
  ON_2dPoint m_base;
  ON_2dVector m_offset;
  ON_SimpleArray< double> m_dashes;
};




#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_HatchLoop*>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_HatchLine>;
#pragma warning( pop )
#endif


/*
  class ON_HatchPattern
  /////////////////////////////////////////////////////////////////
  Fill definition for a hatch

  The hatch  will be one of 
    ON_Hatch::ftLines     - pat file style definition
    ON_Hatch::ftGradient  - uses a color function
    ON_Hatch::ftSolid     - uses entity color

*/
class ON_CLASS ON_HatchPattern : public ON_Object
{
  ON_OBJECT_DECLARE( ON_HatchPattern);

public:

  enum eFillType
  {
    ftSolid    = 0,  // uses entity color
    ftLines    = 1,  // pat file definition
    ftGradient = 2,  // uses a fill color function
    ftLast     = 3
  };

  ON_HatchPattern();
  ~ON_HatchPattern();
  // C++ default copy construction and operator= work fine.

 // ON_Object overrides
  /////////////////////////////////////////////////////////////////
   ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const; // for debugging
  ON_BOOL32 Write( ON_BinaryArchive&) const;
  ON_BOOL32 Read( ON_BinaryArchive&);

  // virtual
  ON_UUID ModelObjectId() const;


  //////////////////////////////////////////////////////////////////////
  // Interface

  /*
  Description:
    Return the pattern's fill type
  Parameters:
  */
  eFillType FillType() const;

  /*
  Description:
    Set the pattern's fill type
  Parameters:
    type - [in] the new filltype
  */
  void SetFillType( eFillType type);

  /*
  Description:
    Set the name of the pattern
  Parameters:
    pName - [in] the new name
  Returns:
  */
  void SetName( const wchar_t* pName);
  void SetName( const char* pName);
  
  /*
  Description:
    Get the name of the pattern
  Parameters:
    string - [out] The name is returned here
  */
  void GetName( ON_wString& string) const;

  /*
  Description:
    Get the name of the pattern
  Returns:
    The name string
  */
  const wchar_t* Name() const;

  /*
  Description:
    Set the name of the pattern
  Parameters:
    pDescription - [in] the new description
  Returns:
  */
  void SetDescription( const wchar_t* pDescription);
  void SetDescription( const char* pDescription);
  
  /*
  Description:
    Get a short description of the pattern
  Parameters:
    string - [out] The string is returned here
  */
  void GetDescription( ON_wString& string) const;

  /*
  Description:
    Return a short text description of the pattern type
  Parameters:
  Returns:
    The description string
  */
  const wchar_t* Description() const;

  /*
  Description:
    Set the table index of the pattern
  Parameters:
    index - [in] the new index
  Returns:
  */
  void SetIndex( int index);

  /*
  Description:
    Return the table index of the pattern
  Parameters:
  Returns:
    The table index
  */
  int Index() const;

  // Interface functions for line hatches
  /////////////////////////////////////////////////////////////////
  /*
  Description:
    Get the number of ON_HatchLines in the pattern
  Parameters:
  Return:
    number of lines
  */
  int HatchLineCount() const;

  /*
  Description:
    Add an ON_HatchLine to the pattern
  Parameters:
    line - [in] the line to add
  Return:
    >= 0 index of the new line
    -1 on failure
  */
  int AddHatchLine( const ON_HatchLine& line);

  /*
  Description:
    Get the ON_HatchLine at index
  Parameters:
    index - [in] Index of the line to get
  Return:
    the hatch line
    NULL if index is out of range
  */
  const ON_HatchLine* HatchLine( int index) const;

  /*
  Description:
    Remove a hatch line from the pattern
  Parameters:
    index - [in] Index of the line to remove
  Return:
    true - success
    false - index out of range
  */
  bool RemoveHatchLine( int index);

  /*
  Description:
    Remove all of the hatch line from the pattern
  Parameters:

  Return:
    true - success
    false - index out of range
  */
  void RemoveAllHatchLines();

  /*
  Description:
    Set all of the hatch lines at once. 
    Existing hatchlines are deleted.
  Parameters:
    lines - [in] Array of lines to add.  Lines are copied
  Return:
    number of lines added
  */
  int SetHatchLines( const ON_ClassArray<ON_HatchLine> lines);

public:
  int m_hatchpattern_index;         // Index in the hatch pattern table
  ON_wString m_hatchpattern_name;   // String name of the pattern
  ON_UUID m_hatchpattern_id;
  
  eFillType m_type;
  
  ON_wString m_description;  // String description of the pattern

  // Represents a collection of ON_HatchLine's to make a complete pattern
  // This is the definition of a hatch pattern.
  // Simple solid line hatches with fixed angle and spacing are also 
  // represented with this type of hatch
  ON_ClassArray<ON_HatchLine> m_lines; // used by line hatches
};

/*
  class ON_Hatch
  /////////////////////////////////////////////////////////////////
  Represents a hatch in planar boundary loop or loops 
  This is a 2d entity with a plane defining a local coordinate system
  The loops, patterns, angles, etc are all in this local coordinate system

  The ON_Hatch object manages the plane and loop array
  Fill definitions are in the ON_HatchPattern or class derived from ON_HatchPattern
  ON_Hatch has an index to get the pattern definition from the pattern table

*/
class ON_CLASS ON_Hatch : public ON_Geometry
{
  ON_OBJECT_DECLARE( ON_Hatch);

public:
  // Default constructor
  ON_Hatch();
  ON_Hatch( const ON_Hatch&);
  ON_Hatch& operator=(const ON_Hatch&);
  ~ON_Hatch();

  virtual ON_Hatch* DuplicateHatch() const;

  // ON_Object overrides
  /////////////////////////////////////////////////////////////////
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  void Dump( ON_TextLog& ) const; // for debugging
  ON_BOOL32 Write( ON_BinaryArchive&) const;
  ON_BOOL32 Read( ON_BinaryArchive&);
  ON::object_type ObjectType() const;

  // ON_Geometry overrides
  /////////////////////////////////////////////////////////////////
  /*
    Returns the geometric dimension of the object ( usually 3)
  */
  int Dimension() const;

  /*
    Description:
      Get a bounding 3d WCS box of the object
      This is a bounding box of the boundary loops
    Parameters:
      [in/out] double* boxmin - pointer to dim doubles for min box corner
      [in/out] double* boxmax - pointer to dim doubles for max box corner
      [in] ON_BOOL32 growbox   - true to grow the existing box,
                            false ( the default) to reset the box
    Returns:
      true = Success
      false = Failure
    Remarks:
  */
  ON_BOOL32 GetBBox( double*, double*, ON_BOOL32 = false) const;

  /*
	Description:
    Get tight bounding box of the hatch.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      tight bounding box of the hatch.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      hatch is calculated.  The hatch is not modified.
	Returns:
    True if the returned tight_bbox is set to a valid 
    bounding box.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;


  /*
    Description:
      Transform the object by a 4x4 xform matrix

    Parameters:
      [in] xform  - An ON_Xform with the transformation information
    Returns:
      true = Success
      false = Failure
    Remarks:
      The object has been transformed when the function returns.
  */
  ON_BOOL32 Transform( const ON_Xform&);

  // Interface
  /////////////////////////////////////////////////////////////////

  /*
  Description:
    Create a hatch from input geometry and parameters
  Parameters:
    plane [I] - ON_Plane to make the hatch on
    loops [I] - Array of boundary loops with the outer one first
    pattern_index [I] - Index into the hatch table
    pattern_rotation [I] - ccw in radians about plane origin
    pattern_scale [I] - Scale factor for pattern definition
  Returns:
    true = success, false = failure
  */
  bool Create( const ON_Plane& plane,
               const ON_SimpleArray<const ON_Curve*> loops, 
               int pattern_index, 
               double pattern_rotation, 
               double pattern_scale);

  /*
  Description:
    Get the plane defining the hatch's coordinate system
  Parameters:
  Returns:
    the plane
  */
  const ON_Plane& Plane() const;

  /*
  Description:
    Set the plane defining the hatch's coordinate system
  Parameters:
    plane - [in] the plane to set
  Returns:
  */
  void SetPlane( const ON_Plane& plane);
  
  /*
  Description:
    Gets the rotation applied to the hatch pattern 
    when it is mapped to the hatch's plane
  Returns:
    The rotation in radians
  Remarks:
    The pattern is rotated counter-clockwise around
    the hatch's plane origin by this value
  */
  double PatternRotation() const;

/*
  Description:
    Sets the rotation applied to the hatch pattern 
    when it is mapped to the hatch's plane
  Parameters:
    rotation - [in] The rotation in radians
  Remarks:
    The pattern is rotated counter-clockwise around
    the hatch's plane origin by this value
  */
  void SetPatternRotation( double rotation);
  
  /*
  Description:
    Gets the scale applied to the hatch pattern 
    when it is mapped to the hatch's plane
  Returns:
    The scale
  Remarks:
    The pattern is scaled around
    the hatch's plane origin by this value
  */
  double PatternScale() const;

/*
  Description:
    Sets the scale applied to the hatch pattern 
    when it is mapped to the hatch's plane
  Parameters:
    scale - [in] The scale
  Remarks:
    The pattern is scaled around
    the hatch's plane origin by this value
  */
  void SetPatternScale( double scale);
  
  /*
  Description:
    Get the number of loops used by this hatch
  Parameters:
  Returns:
    the number of loops
  */
  int LoopCount() const;

  /*
  Description:
    Add a loop to the hatch
  Parameters:
    loop - [in] the loop to add. Memory management for the loop is managed
           by this class.
  Returns:
  */
  void AddLoop( ON_HatchLoop* loop);

  /*
  Description:
    Insert a loop to the hatch at the specified index
  Parameters:
    index - [in] zero based index of the position where insert the loop to.
    loop - [in] the loop to insert. Memory management for the loop is managed
                by this class on success.
  Returns:
    true if success
	  false if index is lower than 0 or greater than current loop count.
  */
  bool InsertLoop( int index,
                   ON_HatchLoop* loop);

  /*
  Description:
    Remove a loop in the hatch
  Parameters:
    loop - [in] zero based index of the loop to remove.
  Returns:
    true if success
  */
  bool RemoveLoop( int index);

  /*
  Description:
    Get the loop at index
  Parameters:
    index - [in] which loop to get
  Returns:
    pointer to loop at index
    NULL if index is out of range
  */
  const ON_HatchLoop* Loop( int index) const;

  /*
  Description:
    Get the 3d curve corresponding to loop[index]
  Parameters:
    index - [in] which loop to get
  Returns:
    pointer to 3d curve of loop at index
    NULL if index is out of range or curve can't be made
    Caller deletes the returned curve
  */
  ON_Curve* LoopCurve3d( int index) const;

  /*
  Description:
    Get the index of the hatch's pattern
  Parameters:
  Returns:
    index of the pattern
  */
  int PatternIndex() const;

/*
  Description:
    Set the index of the hatch's pattern
  Parameters:
    index - [in] pattern index to set
  Returns:
  */
  void SetPatternIndex( int index);

  // Basepoint functions added March 23, 2008 -LW
  /*
  Description:
    Set 2d Base point for hatch pattern alignment.
  Parameters:
    basepoint - 2d point in hatch's ECS
  */
  void SetBasePoint(ON_2dPoint basepoint);

  /*
  Description:
    Set 3d Base point for hatch pattern alignment.
  Parameters:
    point - 3d WCS point
  Remarks:
    Projects point to hatch's plane and sets 2d point
  */
  void SetBasePoint(ON_3dPoint point);

  /*
  Description:
    Return 3d WCS point that lies on hatch's plane used for pattern origin.
  */
  ON_3dPoint BasePoint() const;

  /*
  Description:
    Return 2d ECS point used for pattern origin.
  */
  ON_2dPoint BasePoint2d() const;

  /*
  Function added June 12 2008 LW
  Description:
    Remove all of the loops on the hatch and add the curves in 'loops' as new loops
  Parameters:
    loops - [in] An array of pointers to 2d or 3d curves
                 If the curves are 2d, add them to the hatch directly
                 If they are 3d, project them to the hatch's plane first
  Returns:
    true  - success
    false - no loops in input array or an error adding them
  */
  bool ReplaceLoops(ON_SimpleArray<const ON_Curve*> loops);

protected:
  ON_Plane m_plane;
  double m_pattern_scale;
  double m_pattern_rotation;
  ON_SimpleArray<ON_HatchLoop*> m_loops;
  int m_pattern_index;

    // This function is temporary and will be removed next time the SDK can be modified.
  class ON_HatchExtra* HatchExtension();

};

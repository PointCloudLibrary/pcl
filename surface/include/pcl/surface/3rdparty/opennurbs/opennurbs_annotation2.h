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

#if defined(ON_OS_WINDOWS_GDI)

#define ON_RECT RECT

#else

typedef struct tagON_RECT
{
  int left;
  int top;
  int right;
  int bottom;
} ON_RECT;

#endif


class ON_CLASS ON_Annotation2Text : public ON_wString
{
public:
  ON_Annotation2Text();
  ~ON_Annotation2Text();

  // 24 Sep 2010 Dale Lear
  //    None of these were implmented and they don't make any sense.
  //    ON_Annotation2Text is derived from ON_wString, not ON_Object.
  //    I'm commenting out these functions and it doesn't break the
  //    SDK because linking would fail for anybody trying to use
  //    these functions.

  //////void SetDefaults();
  //////  // override virtual ON_Object::Dump function
  //////void Dump( ON_TextLog& text_log ) const;
  //////// override virtual ON_Object::Dump function
  //////unsigned int SizeOf() const;
  //////// override virtual ON_Object::Write function
  //////ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  //////// override virtual ON_Object::Read function
  //////ON_BOOL32 Read(ON_BinaryArchive& binary_archive);
  //////// override virtual ON_UserData::GetDescription function
  //////ON_BOOL32 GetDescription( ON_wString& description );
  //////// override virtual ON_UserData::Archive function
  //////ON_BOOL32 Archive() const; 



  ON_Annotation2Text& operator=(const char*);
  ON_Annotation2Text& operator=(const wchar_t*);

  void SetText( const char* s );
  void SetText( const wchar_t* s );

  // m_rect is a Windows gdi RECT that bounds text 
  // ("x" increases to the right and "y" increases downwards).
  // If all fields are 0, then m_rect is not set.
  // If left < right and top < bottom, then the rect bounds 
  // the text when it is drawn with its font's 
  // lfHeight=ON_Font::normal_font_height and (0,0) left baseline
  // point of the leftmost character on the first line
  // of text. If (x,y) is a point on the drawn text, then
  // left <= x < right and top <= y < bottom.
  ON_RECT m_rect;
};

// Extension to ON_TextEntity added 12/10/2009 for Text background drawing
class ON_CLASS ON_TextExtra : public ON_UserData
{
  ON_OBJECT_DECLARE(ON_TextExtra);
public:

  ON_TextExtra();
  ~ON_TextExtra();

  static
  ON_TextExtra* TextExtension(class ON_TextEntity2* pDim, bool bCreate);
  static const 
  ON_TextExtra* TextExtension(const class ON_TextEntity2* pDim, bool bCreate);

  void SetDefaults();

  // override virtual ON_Object::Dump function
  void Dump( ON_TextLog& text_log ) const;

  // override virtual ON_Object::Dump function
  unsigned int SizeOf() const;

  // override virtual ON_Object::Write function
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;

  // override virtual ON_Object::Read function
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);

  // override virtual ON_UserData::GetDescription function
  ON_BOOL32 GetDescription( ON_wString& description );

  // override virtual ON_UserData::Archive function
  ON_BOOL32 Archive() const; 

  ON_UUID ParentUUID() const;
  void SetParentUUID( ON_UUID parent_uuid);

  bool DrawTextMask() const;
  void SetDrawTextMask(bool bDraw);

  int MaskColorSource() const;
  void SetMaskColorSource(int source);

  ON_Color MaskColor() const;  // Only works right if MaskColorSource returns 2.
                               // Does not return viewport background color
  void SetMaskColor(ON_Color color);

  double MaskOffsetFactor() const;
  void SetMaskOffsetFactor(double offset);

  ON_UUID  m_parent_uuid;    // uuid of the text using this extension

  bool     m_bDrawMask;      // do or don't draw a mask

  int      m_color_source;   // 0: Use background color from viewport
                             // 1: Use specific color from m_mask_color

  ON_Color m_mask_color;     // Color to use for mask if m_color_source is 2

  double   m_border_offset;  // Offset for the border around text to the rectangle used to draw the mask
                             // This number * HeightOfI for the text is the offset on each side of the 
                             // tight rectangle around the text characters to the mask rectangle.
};


class ON_CLASS ON_DimensionExtra : public ON_UserData
{
  ON_OBJECT_DECLARE(ON_DimensionExtra);
public:

  ON_DimensionExtra();
  ~ON_DimensionExtra();

  static
  ON_DimensionExtra* DimensionExtension(class ON_LinearDimension2* pDim, bool bCreate);
  static const 
  ON_DimensionExtra* DimensionExtension(const class ON_LinearDimension2* pDim, bool bCreate);
  static
  ON_DimensionExtra* DimensionExtension(class ON_RadialDimension2* pDim, bool bCreate);
  static const 
  ON_DimensionExtra* DimensionExtension(const class ON_RadialDimension2* pDim, bool bCreate);
  static
  ON_DimensionExtra* DimensionExtension(class ON_OrdinateDimension2* pDim, bool bCreate);
  static const 
  ON_DimensionExtra* DimensionExtension(const class ON_OrdinateDimension2* pDim, bool bCreate);

  void SetDefaults();

  // override virtual ON_Object::Dump function
  void Dump( ON_TextLog& text_log ) const;

  // override virtual ON_Object::Dump function
  unsigned int SizeOf() const;

  // override virtual ON_Object::Write function
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;

  // override virtual ON_Object::Read function
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);

  // override virtual ON_UserData::GetDescription function
  ON_BOOL32 GetDescription( ON_wString& description );

  // override virtual ON_UserData::Archive function
  ON_BOOL32 Archive() const; 

  ON_UUID ParentUUID() const;
  void SetParentUUID( ON_UUID parent_uuid);

  //  0: default position
  //  1: force inside
  // -1: force outside
  int ArrowPosition() const;
  void SetArrowPosition( int position);

  // For a dimension in page space that measures between points in model space
  // of a detail view, this is the ratio of the page distance / model distance.
  // When the dimension text is displayed, the distance measured in model space
  // is multiplied by this number to get the value to display.
  double DistanceScale() const;
  void SetDistanceScale(double s);

  // Basepont in modelspace coordinates for ordinate dimensions
  void SetModelSpaceBasePoint(ON_3dPoint basepoint);
  ON_3dPoint ModelSpaceBasePoint() const;

  //const wchar_t* ToleranceUpperString() const;
  //ON_wString& ToleranceUpperString();
  //void SetToleranceUpperString( const wchar_t* upper_string);
  //void SetToleranceUpperString( ON_wString& upper_string);

  //const wchar_t* ToleranceLowerString() const;
  //ON_wString& ToleranceLowerString();
  //void SetToleranceLowerString( const wchar_t* lower_string);
  //void SetToleranceLowerString( ON_wString& lower_string);

  //const wchar_t* AlternateString() const;
  //ON_wString& AlternateString();
  //void SetAlternateString( const wchar_t* alt_string);
  //void SetAlternateString( ON_wString& alt_string);

  //const wchar_t* AlternateToleranceUpperString() const;
  //ON_wString& AlternateToleranceUpperString();
  //void SetAlternateToleranceUpperString( const wchar_t* upper_string);
  //void SetAlternateToleranceUpperString( ON_wString& upper_string);

  //const wchar_t* AlternateToleranceLowerString() const;
  //ON_wString& AlternateToleranceLowerString();
  //void SetAlternateToleranceLowerString( const wchar_t* lower_string);
  //void SetAlternateToleranceLowerString( ON_wString& lower_string);

  ON_UUID m_partent_uuid;  // the dimension using this extension

  int m_arrow_position;

  // This is either NULL or an array of GDI rects for the substrings 
  // that make up the dimension string.
  // If the dimension text is all on the same line, there is just one
  // rectangle needed to bound the text and that is the same as the
  // m_rect on the ON_Annotation2Text.
  // If the dimension has tolerances or for some other reason has more
  // than one line of text, m_text_rects is an array of 7 rects, one
  // each for the substrings that might be needed to display the dimension.
  // If some of the rects aren't used, they are empty at 0,0
  // The strings that correspond to these rectangles are generated from
  // info in the dimstyle
  ON_RECT* m_text_rects;

  double m_distance_scale;
  ON_3dPoint m_modelspace_basepoint;
};


/*
  class ON_Annotation2

    Description:
      Used to serialize definitions of annotation objects (dimensions, text, leaders, etc.).
      Virtual base class for annotation objects
      Replaces ON_Annotation
*/
class ON_CLASS ON_Annotation2 : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_Annotation2);

  // UNICODE symbol code to use for degrees, radius, diameter and plus/minus in dimensions
  enum SYMBOLS
  {
    degreesym = 176,
    radiussym = L'R',
    diametersym = 216,
    plusminussym = 177,
  };

public:
  ON_Annotation2();
  ~ON_Annotation2();
  // C++ automatically provides the correct copy constructor and operator= .
  //ON_Annotation2(const ON_Annotation2&);
  //ON_Annotation2& operator=(const ON_Annotation2&);

  // convert from old style annotation
  ON_Annotation2(const ON_Annotation&);
  ON_Annotation2& operator=(const ON_Annotation&);

  // Description:
  //   Sets initial defaults
  void Create();

  void Destroy();

  void EmergencyDestroy();

  /////////////////////////////////////////////////////////////////
  //
  // ON_Object overrides
  //

  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;


  /*
    Description: Writes the object to a file

    Returns:
      @untitled Table
      true     Success
      false    Failure
  */
  ON_BOOL32 Write(
         ON_BinaryArchive&
       ) const;

  /*
    Description: Reads the object from a file

    Returns:
      @untitled Table
      true     Success
      false    Failure
  */
  ON_BOOL32 Read(
         ON_BinaryArchive&
       );

  /*
    Returns: The Object Type of this object
  */
  ON::object_type ObjectType() const;

  /////////////////////////////////////////////////////////////////
  //
  // ON_Geometry overrides
  //

  /*
    Returns the geometric dimension of the object ( usually 3)
  */
  int Dimension() const;

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  // virtual ON_Geometry override
  bool EvaluatePoint( const class ON_ObjRef& objref, ON_3dPoint& P ) const;

  /////////////////////////////////////////////////////////////////
  //
  // ON_Annotation2 interface
  //

  // Definitions of text justification
  // Not implemented on all annotation objects
  enum eTextJustification
  {
    tjUndefined = 0,
    tjLeft   = 1<<0,
    tjCenter = 1<<1,
    tjRight  = 1<<2,
    tjBottom = 1<<16,
    tjMiddle = 1<<17,
    tjTop    = 1<<18,
    tjBottomLeft   = tjBottom | tjLeft,
    tjBottomCenter = tjBottom | tjCenter,
    tjBottomRight  = tjBottom | tjRight,
    tjMiddleLeft   = tjMiddle | tjLeft,
    tjMiddleCenter = tjMiddle | tjCenter,
    tjMiddleRight  = tjMiddle | tjRight,
    tjTopLeft      = tjTop    | tjLeft,
    tjTopCenter    = tjTop    | tjCenter,
    tjTopRight     = tjTop    | tjRight,
  };

  /*
    Description:
      Query if the annotation object is a text object
    Parameters:
      none
    Returns:
      @untitled table
      true    It is text
      false   Its not text
  */
  bool IsText() const;

  /*
    Description:
      Query if the annotation object is a leader
    Parameters:
      none
    Returns:
      @untitled table
      true    It is a leader
      false   Its not a leader
  */
  bool IsLeader() const;

  /*
    Description:
      Query if the annotation object is a dimension
    Parameters:
      none
    Returns:
      @untitled table
      true    It is a dimension
      false   Its not a dimension
  */
  bool IsDimension() const;

  /*
    Description:
      Set or get the index in the appropriate table for either the font or
      dimstyle of this object
    Parameters:
      [in] int  the new index (Set)
    Returns:
      int -  The index (Get)
    Remarks:
      If the object is a text object the index is of object's font in the Font Table
      If the object is anything else, the index is of the object's dimstyle in the DimStyle Table
      Derived objects can use FontIndex() and StyleIndex() to set/get these same values.
  */
  int Index() const;
  void SetIndex( int);

  /*
  Returns:
    Dimension type
    Linear dim:  distance between arrow tips
    Radial dim:  radius or diameter depending on m_type value
    Angular dim: angle in degrees
    Leader:      ON_UNSET_VALUE
    Text:        ON_UNSET_VALUE
  */
  virtual 
  double NumericValue() const;

  /*
    Description:
      Set  or Get the height of the text in this annotation
    Parameters:
      [in] double new text height to set
    Returns:
      double Height of the text
    Remarks:
      Height is in model units
  */
  void SetHeight( double);
  double Height() const;

  /*
    Description:
      Sets or gets the object type member to a specific annotation type:
           dtDimLinear, dtDimAligned, dtDimAngular, etc.
    Parameters:
      [in] ON::eAnnotationType type - dtDimLinear, dtDimAligned, dtDimAngular, etc.
    Returns:
      ON::eAnnotationType of the object
  */
  void SetType( ON::eAnnotationType);
  ON::eAnnotationType Type() const;

  /*
    Description:
      Set or get the plane for the object's ECS
    Parameters:
      [in] ON_Plane& plane in WCS
    Returns:
      const ON_Plane& - the object's ECS plane in WCS coords
  */
  void SetPlane( const ON_Plane&);
  const ON_Plane& Plane() const;

  /*
    Description:
      Returns the number of definition points this object has
    Parameters:
      none
    Returns:
      @untitled table
      int   the object's point count
  */
  int PointCount() const;
  void SetPointCount( int count);

  /*
    Description:
      Set or get the object's whole points array at once
    Parameters:
      [in] ON_2dPointArray& pts
    Returns:
      const ON_2dPointArray& - ref to the object's point array
  */
  void SetPoints( const ON_2dPointArray&);
  const ON_2dPointArray& Points() const;

  /*
    Description:
      Set individual definition points for the annotation
    Parameters:
      @untitled table
      [in] int index               index of the point to set in ECS 2d coordinates
      [in] const ON_2dPoint& pt    the new point value
    Returns:
      ON_2dPoint   the point coordinates in ECS
  */
  void SetPoint( int, const ON_2dPoint&);
  ON_2dPoint Point( int) const;

  /*
    Description:
      
      Set or get the string value of the user text, with no substitution for "<>"
    Parameters:
      [in] const wchar_t* string   the new value for UserText
    Returns:
      const ON_wString&    The object's UserText
    Remarks:
      UserText is the string that gets printed when the dimensoin is drawn.
      If it contains the token "<>", that token is replaced with the measured
      value for the dimension, formatted according to the DimStyle settings.
      "<>" is the default for linear dimensions.
      Other dimensions include "<>" in their default string
  */

  // OBSOLETE - call SetTextValue( text_value );
  ON_DEPRECATED void SetUserText( const wchar_t* text_value );

  // OBSOLETE - call TextValue( text_value );
  ON_DEPRECATED const ON_wString& UserText() const;


  /*
  Description:
    Gets the value of the annotation text.
  Returns:
    Value of the annotation text.
  See Also:
    ON_Annotation2Text::SetTextValue()
    ON_Annotation2Text::SetTextFormula()
    ON_Annotation2Text::TextFormula()    
  Remarks:
    This gets the literal value of the text, there is no
    substitution for any "<>" substrings.  When a dimension
    is drawn, any occurance of "<>" will be replaced
    with the measured value for the dimension and formatted
    according to the DimStyle settings.

    Annotation text values can be constant or the result 
    of evaluating text formula containing %<...>% 
    expressions. The ...TextValue() functions set
    and get the text's value.  The ...TextFormula()
    functions get and set the text's formula.
  */
  const wchar_t* TextValue() const;

  /*
  Description:
    Sets the value of the annotation text.  No changes
    are made to the text_value string.
  Parameters:
    text_value - [in]
  Returns:
    Value of the annotation text.
  See Also:
    ON_Annotation2Text::SetTextFormula()
    ON_Annotation2Text::TextValue()    
    ON_Annotation2Text::TextFormula()    
  Remarks:
    Annotation text values can be constant or the result 
    of evaluating text formula containing %<...>% 
    expressions. The ...TextValue() functions set
    and get the text's value.  The ...TextFormula()
    functions get and set the text's formula.
  */
  void SetTextValue( const wchar_t* text_value );

  /*
  Description:
    Gets the formula for the annotation text.
  Parameters:
    text_value - [in]
  Returns:
    Value of the annotation text.
  See Also:
    ON_Annotation2Text::SetTextValue()
    ON_Annotation2Text::TextValue()    
    ON_Annotation2Text::TextFormula()    
  Remarks:
    Annotation text values can be constant or the result 
    of evaluating text formula containing %<...>% 
    expressions. The ...TextValue() functions set
    and get the text's value.  The ...TextFormula()
    functions get and set the text's formula.
  */
  const wchar_t* TextFormula() const;

  /*
  Description:
    Sets the formula for the annotation text.
  Parameters:
    text_value - [in]
  Returns:
    Value of the annotation text.
  See Also:
    ON_Annotation2Text::SetTextValue()
    ON_Annotation2Text::Value()    
    ON_Annotation2Text::Formula()    
  Remarks:
    Annotation text values can be constant or the result 
    of evaluating text formula containing %<...>% 
    expressions. The ...TextValue() functions set
    and get the text's value.  The ...TextFormula()
    functions get and set the text's formula.
  */
  void SetTextFormula( const wchar_t* s );

  /*
    Description:
      Set or get a flag indication that the dimension text has been moved
      from the default location.
    Parameters:
      bUserPositionedText - [in] 
               true to indicate that the text has been placed by the user.
               false to indicate that it hasn't
    Returns:
      @untitled table
      true    The text has been moved
      false   The text is in the default location
    Remarks:
      If the text is in the default location, it should be repositioned
      automatically when the dimension is adjusted.
      If it has been moved, it should not be automatically positioned.
  */
  void SetUserPositionedText( int bUserPositionedText );
  bool UserPositionedText() const;

  /*
    Description:
      Set or get the text display mode for the annotation
    Parameters:
      [in] ON::eTextDisplayMode mode - new mode to set
    Returns:
      ON::eTextDisplayMode  - current mode
    Remarks:
      This is the way the text is oriented with respect to the dimension line or screen:
      Above line, In LIne, Horizontal
  */
  void SetTextDisplayMode( ON::eTextDisplayMode);
  ON::eTextDisplayMode TextDisplayMode() const;


  /*
    Description:
      Gets a transform matrix to change from the object's 2d ECS to 3d WCS
    Parameters:
      [out] xform   set to produce the ECS to WCS transform
    Returns:
      @untitled table
      true    Success
      false   Failure
  */
  ON_BOOL32 GetECStoWCSXform( ON_Xform&) const;

  /*
    Description:
      Gets a transform matrix to change from to 3d WCS to the object's 2d ECS
    Parameters:
      [out] xform - set to produce the WCS to ECS transform
    Returns:
      @untitled table
      true    Success
      false   Failure
  */
  ON_BOOL32 GetWCStoECSXform( ON_Xform& xform) const;

  /*
    Description:
      Set the object's point array to a specified length
    Parameters:
      [in] length - the new size of the array
    Returns:
      void
  */
  void ReservePoints( int);


  /*
    Description:
      static function to provide the default UserText string for the object
    Returns:
      const wchar_t* - the default string to use
  */
  static const wchar_t* DefaultText();

  /*
    Description:
      Convert back to the version of ON_Annotation used in Rhino 2
    Parameters:
      target [out] the old-style object
    Returns:
      @untitled table
      true     Success
      False    Failure
    See Also:  ON_AngularDimension::ConvertBack()
  */
  virtual 
  void ConvertBack( ON_Annotation& target);

  /*
    Description:
      Set or Get the text justification
    Parameters:
      justification [in] See enum eJustification for meanings
    Returns:
      The justification for the text in this object
    Comments:
      This is not implemented on all annotation objects.
      The default SetJustification() does nothing
      The default Justification() always returns 0

  */
  virtual
  void SetJustification( unsigned int justification);

  virtual 
  unsigned int Justification();

  /*
    Description:
      Get the transformation that maps the annotation's
      text to world coordinates.
      Added Oct 30, 07 LW
    Parameters:
      gdi_text_rect - [in] 
              Windows gdi rect of text when it is drawn with
              LOGFONT lfHeight = ON_Font::normal_font_height.
      gdi_height_of_I - [in]
         Value returned by ON_Font::HeightOfI().
      dimstyle_textheight - [in]
         Height of text in world units.  If the annotation is
         an ON_TextEntity2, this is the m_textheight value.  
         If the annotation is not an ON_TextEntity2, pass in 
         the value returned by the dimension style's 
         ON_DimStyle::TextHeight() 
      dimstyle_textgap - [in]
         The value of the annotation's dimension style's 
         ON_DimStyle::TextGap().
      dimstyle_textalignment - [in]
         ON::TextDisplayMode(ON_DimStyle::TextAlignment()).
      dimscale - [in]
         Global dimension scaling value.  If you are using the
         Rhino SDK, this value is returned by
         CRhinoDoc::Properties().AnnotationSettings().DimScale().
         If you are using the OpenNURBS IO toolkit, this value
         is on ON_3dmSettings::m_AnnotationSettings.m_dimscale.
      cameraX - [in]
         zero or the view's unit camera right vector
      cameraY - [in]
         zero or the view's unit camera up vector
      model_xform - [in] transforms the text's parent entity 
         to world coordinates in case its instance geometry
         NULL == Identity
      text_xform - [out]
    Returns:
      True if text_xform is set.
  */
  bool GetTextXform( 
        ON_RECT gdi_text_rect,
        int gdi_height_of_I,
        double dimstyle_textheight,
        double dimstyle_textgap,
        ON::eTextDisplayMode dimstyle_textalignment,
        double dimscale,
        ON_3dVector cameraX,
        ON_3dVector cameraY,
        const ON_Xform* model_xform,
        ON_Xform& text_xform // output
        ) const;

  /*
    Description:

    This function has been replaced with a version that
    takes a model transform to transform block instance 
    geometry to world coordinates  Oct 30, 07 LW

      Get the transformation that maps the annotation's
      text to world coordinates.
    Parameters:
      gdi_text_rect - [in] 
              Windows gdi rect of text when it is drawn with
              LOGFONT lfHeight = ON_Font::normal_font_height.
      gdi_height_of_I - [in]
         Value returned by ON_Font::HeightOfI().
      dimstyle_textheight - [in]
         Height of text in world units.  If the annotation is
         an ON_TextEntity2, this is the m_textheight value.  
         If the annotation is not an ON_TextEntity2, pass in 
         the value returned by the dimension style's 
         ON_DimStyle::TextHeight() 
      dimstyle_textgap - [in]
         The value of the annotation's dimension style's 
         ON_DimStyle::TextGap().
      dimstyle_textalignment - [in]
         ON::TextDisplayMode(ON_DimStyle::TextAlignment()).
      dimscale - [in]
         Global dimension scaling value.  If you are using the
         Rhino SDK, this value is returned by
         CRhinoDoc::Properties().AnnotationSettings().DimScale().
         If you are using the OpenNURBS IO toolkit, this value
         is on ON_3dmSettings::m_AnnotationSettings.m_dimscale.
      cameraX - [in]
         zero or the view's unit camera right vector
      cameraY - [in]
         zero or the view's unit camera up vector
      xform - [out]
    Returns:
      True if xform is set.
  */
  bool GetTextXform( 
        ON_RECT gdi_text_rect,
        int gdi_height_of_I,
        double dimstyle_textheight,
        double dimstyle_textgap,
        ON::eTextDisplayMode dimstyle_textalignment,
        double dimscale,
        ON_3dVector cameraX,
        ON_3dVector cameraY,
        ON_Xform& xform
        ) const;

  /*
    Description:
      Get the transformation that maps the annotation's
      text to world coordinates.
      Oct 30, 07 LW
    Parameters:
      gdi_text_rect - [in] 
              Windows gdi rect of text when it is drawn with
              LOGFONT lfHeight = ON_Font::normal_font_height.
      font - [in]
      dimstyle - [in]
      dimscale - [in]
         Global dimension scaling value.  If you are using the
         Rhino SDK, this value is returned by
         CRhinoDoc::Properties().AnnotationSettings().DimScale().
         If you are using the OpenNURBS IO toolkit, this value
         is on ON_3dmSettings::m_AnnotationSettings.m_dimscale.
      vp - [in]
      model_xform - [in] transforms the text's parent entity 
         to world coordinates in case its instance geometry
         NULL == Identity
      text_xform - [out]
    Returns:
      True if text_xform is set.
  */
  //bool GetTextXform( 
  //    const ON_RECT gdi_text_rect,
  //    const ON_Font& font,
  //    const ON_DimStyle& dimstyle,
  //    double dimscale,
  //    const ON_Viewport* vp,
  //    const ON_Xform* model_xform,
  //    ON_Xform& text_xform  // output
  //    ) const;
  bool GetTextXform( 
      const ON_RECT gdi_text_rect,
      const ON_Font& font,
      const ON_DimStyle* dimstyle,
      double dimscale,
      const ON_Viewport* vp,
      const ON_Xform* model_xform,
      ON_Xform& text_xform  // output
      ) const;

  /*
    Description:

    This function has been replaced with a version that
    takes a model transform because the viewport doesn't 
    contain block instance transform info  Oct 30, 07 LW

      Get the transformation that maps the annotation's
      text to world coordinates.
    Parameters:
      gdi_text_rect - [in] 
              Windows gdi rect of text when it is drawn with
              LOGFONT lfHeight = ON_Font::normal_font_height.
      font - [in]
      dimstyle - [in]
      dimscale - [in]
         Global dimension scaling value.  If you are using the
         Rhino SDK, this value is returned by
         CRhinoDoc::Properties().AnnotationSettings().DimScale().
         If you are using the OpenNURBS IO toolkit, this value
         is on ON_3dmSettings::m_AnnotationSettings.m_dimscale.
      vp - [in]
      xform - [out]
    Returns:
      True if xform is set.
  */
  bool GetTextXform( 
      ON_RECT gdi_text_rect,
      const ON_Font& font,
      const ON_DimStyle& dimstyle,
      double dimscale,
      const ON_Viewport* vp,
      ON_Xform& xform
      ) const;

  /*
  Description:
    Get the annotation plane coordinates (ECS) of the point
    that is used to position the text.  The relative position
    of the text to this points depends on the type of
    annotation, the dimstyle's text alignment flag, and the
    view projection.
    This point is not  the same as the base point of the text.
  Parameters:
    text_point - [out];
  Returns:
    True if text_point is set.
  */
  bool GetTextPoint( ON_2dPoint& text_2d_point ) const;

  // enum for tyoe of annotation DimLinear, DimRadius, etc.
  ON::eAnnotationType m_type;

  // m_textdisplaymode controls the orientation
  // of the text.
  // If m_textdisplaymode = dtHorizontal, then
  // the text is always horizontal and in the
  // view plane.  Otherwise it lies in m_plane.
  ON::eTextDisplayMode m_textdisplaymode;

  // m_plane is the plane containing the annotation.
  // All parts of the annotation that are not
  // text lie in this plane. If
  // m_textdisplaymode != dtHorizontal, then
  // the text lies in the plane too.  
  // (ECS reference plane in WCS coordinates.)
  ON_Plane m_plane;

  // Definition points for the dimension.
  // These are 2d coordinates in m_plane.
  // The location of these points depends on the
  // type of annotation class.  There is a comment
  // at the start of the definions for
  // ON_LinearDimension2, ON_RadialDimension2,
  // ON_AngularDimension2, ON_TextEntity2, and
  // ON_Leader2 that explains how the points are used.
  ON_2dPointArray m_points;

  // With the addition of tolerances and therefore multi-line
  // text, the ON_wString in m_usertext will hold multiple 
  // strings with NULLs between them.  
  // The strings will be in this order:
  // Result of expanding "<>", or user override
  // Alternate dimension
  // Tolerance upper
  // Tolerance lower
  // Alt tolerance upper
  // Alt tolerance lower
  // Prefix
  // Suffix
  // Alt prefix
  // Alt suffix
  // 
  ON_Annotation2Text m_usertext;

  // true: User has positioned text
  // false: use default location
  bool m_userpositionedtext;
  // Added 13 Aug, 2010 - Lowell
  // This determines whether the object will be scaled according to detail
  // scale factor or by 1.0 in paperspace rather than by 
  // dimscale or text scale.
  // For the first try this will only be used on text and its
  // here on the base class because it would fit and in case 
  // its needed later on dimensions.
  bool m_annotative_scale;
private:
  bool m_reserved_b1;
  bool m_reserved_b2;
public:

  // For dimensions, this is the ON_DimStyle index
  // For text, its the ON_Font index
  int m_index;

  // Text height in model units
  // This is used by text, but not by dimensions
  // Dimensions get their height from dimension styles
  double m_textheight;

  // Left, Center, Right / Bottom, Middle, Top text justification
  // See eTextJustification above
  unsigned int m_justification;
};


// Subclass of ON_Annotation2 to provide linear dimensions
class ON_CLASS ON_LinearDimension2 : public ON_Annotation2
{
  ON_OBJECT_DECLARE(ON_LinearDimension2);

public:

  /*
    The annotation's dimstyle controls the position of TEXT,
    the size of the arrowheads, and the amount the ends of 
    linear dimension's extension lines extend beyond the 
    dimension lines.

    In the picture below, [n] means ON_Annotation2::m_points[n].

                                                     [2]
                                                      |
        |                                             |
       [1]-------------------------------------------[3]
        |                                             |
        |                       TEXT
        |                       [4]
       [0]

      The "x" and "y" coordinates of [0] must be (0.0, 0.0).

      The "x" coordinate of [1] = "x" of [0]
      The "y" coordinate of [1] can be any value.

      The "x" and "y" coordinates of [2] can be any value.

      The "x" coordinate of [3] = "x" coordinate of [2].
      The "y" coordinate of [3] = "y" coordinate of [1].
  */

  enum POINT_INDEX
  {
    // Do not change these enum values.  They are saved in files as the 
    // ON_COMPONENT_INDEX.m_index value.
    //
    // Indices of linear dimension definition points in 
    // the m_points[] array
    ext0_pt_index    = 0, // end of first extension line
    arrow0_pt_index  = 1, // arrowhead tip on first extension line
    ext1_pt_index    = 2, // end of second extension line
    arrow1_pt_index  = 3, // arrowhead tip on second extension line
    userpositionedtext_pt_index = 4,
    dim_pt_count     = 5, // number of m_points[] in an angular dim

    // Points calculated from values in m_points[]
    text_pivot_pt = 10000, // center of dimension text
    dim_mid_pt    = 10001  // midpoint of dimension line
  };

  ON_LinearDimension2();
  ~ON_LinearDimension2();
  // C++ automatically provides the correct copy constructor and operator= .
  //ON_LinearDimension2( const ON_LinearDimension2& );
  //ON_LinearDimension2& operator=(const ON_LinearDimension2&);

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  /*
  Description:
    Checks the linear dimension and repairs any point locations or flags
    that are not set correctly.
  Returns:
    0:  linear dimension is damaged beyond repair
    1:  linear dimension was perfect and nothing needed to be repaired.
    2:  linear dimension had flaws that were repaired.
  */
  int Repair();

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_2dPoint Dim2dPoint(
       int point_index
       ) const;

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_3dPoint Dim3dPoint(
       int point_index
       ) const;

  // overrides virual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = 0 ) const;

  // overrides virual ON_Object::Write
  ON_BOOL32 Write(ON_BinaryArchive&) const;

  // overrides virual ON_Object::Read
  ON_BOOL32 Read(ON_BinaryArchive&);

  // overrides virual ON_Geometry::GetBBox
  ON_BOOL32 GetBBox(
         double*,
         double*,
         ON_BOOL32 = false
         ) const;

  // overrides virual ON_Geometry::GetTightBoundingBox
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
  Description:
    Overrides virtual ON_Annotation2::NumericValue();
  Returns:
    distance between arrow tips
  */
  double NumericValue() const;

  /*
    Description:
      Get or set the DimStyle index in the dimstyle table for the dimension
    Parameters:
      [in] int  the new index (Set)
    Returns:
      int -  The current index (Get)
  */
  int StyleIndex() const;
  void SetStyleIndex( int);

  /*
    Description:
      static function to provide the default UserText string for the object
    Returns:
      const wchar_t* - the default string to use
  */
  static const wchar_t* DefaultText();


// 6-23-03 lw Added v2 file writing of annotation
  void GetV2Form( ON_LinearDimension& dim);

  bool CreateFromV2( 
      const ON_Annotation& v2_ann,
      const ON_3dmAnnotationSettings& settings,
      int dimstyle_index
      );

  /*
  Description:
    Get the annotation plane x coordinates of the dimension
    line. The y coordinate of the dimension line is m_ponts[1].y.
  Parameters:
    gdi_text_rect - [in] 
       Windows rect (left < right, top < bottom) that bounds text.
       The baseline of the text should be at y=0 in the rect coordinates.
    gdi_height_of_I - [in] 
       Height of an I in the text in the same.
    gdi_to_world - [in] 
       transform returned by ON_Annotation2::GetTextXform().
    dimstyle - [in]
      dimscale - [in]
    vp - [in]
    x - [out] plane x coordinates of the dimension line.
              The y coordinate = m_points[arrow0_pt_index].y
    bInside - [out] true if arrowheads go inside extension lines, 
                    false if they go outside
  Returns:
    0: the input or class is not valid
    1: A single line from x[0] to x[1] with arrow heads at both ends.
        Arrowtips at x[4] & x[5]
    2: Two lines from x[0] to x[1] and from x[1] to x[2].  The
        Arrowtips at x[4] & x[5]
       
  */
  int GetDimensionLineSegments(
      ON_RECT gdi_text_rect,
      int gdi_height_of_I,
      ON_Xform gdi_to_world,
      const ON_DimStyle& dimstyle,
      double dimscale,
      const ON_Viewport* vp,
      double a[6],
      bool& bInside
      ) const;


  // Added for V5. 4/24/07 LW
  // Get the userdata extension for this dimension
  ON_DimensionExtra* DimensionExtension();
  const ON_DimensionExtra* DimensionExtension() const;




};

//////////
// class ON_RadialDimension2
class ON_CLASS ON_RadialDimension2 : public ON_Annotation2
{
  ON_OBJECT_DECLARE(ON_RadialDimension2);

public:

  /*
    The annotation's dimstyle controls the position of TEXT,
    and the size of the arrowheads.

    In the picture below, [n] means ON_Annotation2::m_points[n].

    Radial dimensions do not permit user positioned text


           knee
            [3]--------[2] TEXT
            /         (tail)
           /
          /
        [1] (arrow head here)


    + [0] = (usually at (0,0) = center of circle)
  */

  enum POINT_INDEX
  {
    // Do not change these enum values.  They are saved in files as the 
    // ON_COMPONENT_INDEX.m_index value.
    //
    // Indices of radial dimension definition points in 
    // the m_points[] array
    center_pt_index = 0, // location of + (usually at center of circle)
    arrow_pt_index  = 1, // arrow tip
    tail_pt_index   = 2, // end of radial dimension
    knee_pt_index   = 3, // number of m_points[] in a radial dim
    dim_pt_count    = 4, // number of m_points[] in a radial dim

    // Points calculated from values in m_points[]
    text_pivot_pt = 10000, // start/end of dimension text at tail
  };

  ON_RadialDimension2();
  ~ON_RadialDimension2();
  // C++ automatically provides the correct copy constructor and operator= .
  //ON_RadialDimension2(const ON_RadialDimension2&);
  //ON_RadialDimension2& operator=(const ON_RadialDimension2&);

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_2dPoint Dim2dPoint(
       int point_index
       ) const;

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_3dPoint Dim3dPoint(
       int point_index
       ) const;


  // overrides virual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = 0 ) const;

  // overrides virual ON_Object::Write
  ON_BOOL32 Write(ON_BinaryArchive&) const;

  // overrides virual ON_Object::Read
  ON_BOOL32 Read(ON_BinaryArchive&);

  // overrides virual ON_Geometry::GetBBox
  ON_BOOL32 GetBBox(
         double*,
         double*,
         ON_BOOL32 = false
         ) const;

  // overrides virual ON_Geometry::GetTightBoundingBox
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
    Description:
      Set the plane and definition points from WCS 3d input
    Parameters:
      center - [in] center of circle
      arrowtip - [in] 3d point on the circle at the dimension arrow tip
      xaxis - [in] x axis of the dimension's plane
      normal - [in] normal to the dimension's plane
      offset_distance - [in] distance from arrow tip to knee point
    Returns:
      @untitled table
      true     Success
      false    Failure
  */
  bool CreateFromPoints( 
          ON_3dPoint center, 
          ON_3dPoint arrowtip, 
          ON_3dVector xaxis, 
          ON_3dVector normal,
          double offset_distance
          );

  /*
  Description:
    Overrides virtual ON_Annotation2::NumericValue();
  Returns:
    If m_type is ON::dtDimDiameter, then the diameter
    is returned, othewise the radius is returned.
  */
  double NumericValue() const;

  /*
    Description:
      Get or set the DimStyle index in the dimstyle table for the dimension
    Parameters:
      [in] int  the new index (Set)
    Returns:
      int -  The current index (Get)
  */
  int StyleIndex() const;
  void SetStyleIndex( int);

  /*
    Description:
      static function to provide the default UserText string for the object
    Returns:
      const wchar_t* - the default string to use
  */
  static const wchar_t* DefaultDiameterText();
  static const wchar_t* DefaultRadiusText();

// 6-23-03 lw Added v2 file writing of annotation
  void GetV2Form( ON_RadialDimension& dim);

  bool CreateFromV2( 
      const ON_Annotation& v2_ann,
      const ON_3dmAnnotationSettings& settings,
      int dimstyle_index
      );

  bool GetArrowHeadDirection( ON_2dVector& arrowhead_dir ) const;
  bool GetArrowHeadTip( ON_2dPoint& arrowhead_tip ) const;
};


//////////
// class ON_AngularDimension2
class ON_CLASS ON_AngularDimension2 : public ON_Annotation2
{
  ON_OBJECT_DECLARE(ON_AngularDimension2);

public:

  /*
    The annotation's dimstyle controls the position of TEXT,
    the size of the arrowheads, and the amount the ends of 
    linear dimension's extension lines extend beyond the 
    dimension lines.

    In the picture below, [n] means ON_Annotation2::m_points[n].

    [0] = if m_userpositionedtext=true, this is the center of text.
          If m_userpositionedtext=false, this point is not used and
          the center of the text is at the arc's midpoint.

    Always counter clockwise arc in m_plane with center = (0,0)
    [1] = a point somewhere on the line from the center through the start point.
          The distance from center to [1] can be any value.
    [2] = a point somewhere on the line from the center through the end point.
          The distance from center to [2] can be any value.
    [3] = a point on the interior of the arc.  The distance 
          from (0,0) to [3] is the radius of the arc.


                  /
                [2]
                /
               /         [0]TEXT
              /
             /    [3]
     -----(0,0)----------[1]---
           /
          /
         /

  */

  enum POINT_INDEX
  {
    // Do not change these enum values.  They are saved in files as the 
    // ON_COMPONENT_INDEX.m_index value.
    //
    // Indices of angular dimension definition points in 
    // the m_points[] array
    userpositionedtext_pt_index  = 0, // 
    start_pt_index = 1, // point on the start ray (not necessarily on arc)
    end_pt_index   = 2, // point on the end ray (not necessarily on arc)
    arc_pt_index   = 3, // point on the interior of dimension arc
    dim_pt_count   = 4, // number of m_points[] in an angular dim

    // Points calculated from values in m_points[]
    text_pivot_pt = 10000, // center of dimension text
    arcstart_pt   = 10001,
    arcend_pt     = 10002,
    arcmid_pt     = 10003,
    arccenter_pt  = 10004, // center of circle arc lies on  
    extension0_pt = 10005, // point where first extension line starts
    extension1_pt = 10006  // point where second extension line starts
  };

  ON_AngularDimension2();
  ~ON_AngularDimension2();
  // C++ copy constructor and operator= work fine.
  //ON_AngularDimension2(const ON_AngularDimension2&);
  //ON_AngularDimension2& operator=(const ON_AngularDimension2&);

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_2dPoint Dim2dPoint(
       int point_index
       ) const;

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_3dPoint Dim3dPoint(
       int point_index
       ) const;


  // overrides virual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = 0 ) const;

  // overrides virual ON_Geometry::GetBBox
  ON_BOOL32 GetBBox(
         double*,
         double*,
         ON_BOOL32 = false
         ) const;

  // overrides virual ON_Geometry::GetTightBoundingBox
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
    Description:
      Read from or write to a file
    Returns:
      @untitled Table
      true     Success
      false    Failure
  */
  ON_BOOL32 Write( ON_BinaryArchive& file ) const;
  ON_BOOL32 Read( ON_BinaryArchive& file );

  /*
    Description:
      Set the plane and definition points from 3d points
      in world coordinates.
    Parameters:
      apex - [in] 3d apex of the dimension
                  (center of arc)
      p0 - [in] 3d point on first line
      p1 - [in] 3d point on second line
      arcpt - [in] 3d point on dimension arc 
                   (determines radius of arc)
      Normal - [in] normal of the plane on which to make the dimension
                    (must be perpendicular to p0-apex and p1-apex) 
    Returns:
      @untitled table
      true     Success
      false    Failure
  */
  bool CreateFromPoints( 
    const ON_3dPoint& apex, 
    const ON_3dPoint& p0, 
    const ON_3dPoint& p1, 
    ON_3dPoint& arcpt, 
    ON_3dVector& Normal
    );

  /*
    Description:
      Set the plane and definition points from a 3d arc.
    Parameters:
      arc - [in]
    Returns:
      @untitled table
      true     Success
      false    Failure
  */
  bool CreateFromArc( 
    const ON_Arc& arc
    );

  bool CreateFromV2( 
      const ON_Annotation& v2_ann,
      const ON_3dmAnnotationSettings& settings,
      int dimstyle_index
      );

  bool GetArc( ON_Arc& arc ) const;

  bool GetExtensionLines(ON_Line extensions[2]) const;

  // Set or get the measured angle in radians
  void SetAngle( double angle);
  double Angle() const;
  void SetRadius( double radius);
  double Radius() const;

  /*
  Description:
    Overrides virtual ON_Annotation2::NumericValue();
  Returns:
    Angle in degrees
  */
  double NumericValue() const;

  /*
    Description:
      Get or set the DimStyle index in the dimstyle table for the dimension
    Parameters:
      [in] int  the new index (Set)
    Returns:
      int -  The current index (Get)
  */
  int StyleIndex() const;
  void SetStyleIndex( int);

  /*
    Description:
      static function to provide the default UserText string for the object
    Returns:
      const wchar_t* - the default string to use
  */
  static const wchar_t* DefaultText();


  /*
    Description:
      Convert back to the version of ON_Annotation used in Rhino 2
    Parameters:
      target [out] the old-style object
    Returns:
      @untitled table
      true     Success
      False    Failure
    See Also:  ON_AnnotationObject::ConvertBack()
  */
  void ConvertBack( ON_AngularDimension2& target);

// 6-23-03 lw Added v2 file writing of annotation
  void GetV2Form( ON_AngularDimension& dim);

  double m_angle;      // angle being dimensioned
  double m_radius;     // radius for dimension arc

  /*
  Description:
    Get the annotation plane angles of the dimension arc.
  Parameters:
    gdi_text_rect - [in] Windows rect (left < right, top < bottom)
       that bounds text.
    gdi_height_of_I - [in] 
       Height of an I in the text.
    gdi_to_world - [in] 
       transform returned by ON_Annotation2::GetTextXform().
    dimstyle - [in]
      dimscale - [in]
    vp - [in]
    a - [out]
      angles at the ends of the arc segment(s) and the arrow tips
    bInside - [out] true if arrowheads go inside, false if they go outside
  Returns:
    number of arc segments to draw
    0: the input or class is not valid
    1: A single arc from a[0] to a[1] with arrow heads at a[4] & a[5].
    2: Two arcs from a[0] to a[1] & from a[2] to a[3].
       Arrowheads are at a[4] & a[5].
  */
  int GetDimensionArcSegments(
      ON_RECT gdi_text_rect,
      int gdi_height_of_I,
      ON_Xform gdi_to_world,
      const ON_DimStyle& dimstyle,
      double dimscale,
      const ON_Viewport* vp,
      double a[6],
      bool& bInside
      ) const;

  
  /*
  Description:
    Get distance from dimension apex to extension line offset points
  Parameters:
    index - [in]  which distance to get
  Returns:
    Distance to offset point [index]
  */
  double DimpointOffset(
    int index) const;

  /*
  Description:
    Set distance from dimension apex to extension line offset points
  Parameters:
    index  - [in]  which distance to set
    offset - [in] Value to set
  */
  void SetDimpointOffset(
    int index, 
    double offset);
};



/*
  class ON_LinearDimension2

  Description:
    Override od ON_Annotation2 to provide linear dimensions
*/
class ON_CLASS ON_OrdinateDimension2 : public ON_Annotation2
{
  ON_OBJECT_DECLARE(ON_OrdinateDimension2);

public:

  /*
    In the picture below, [n] means ON_Annotation2::m_points[n].

    Measures in X direction

                       [1]
                        |
                        |
                        |
                        |
                        |
                       [0]
       +
 [plane origin]                                      [plane origin]
                                                           +

      or - Measures in Y direction                                                   *---[1]       
                                                                                    /
                                                                                   /
                   [0]--------------------[1]                   [0]---------------*


                                                                              * = calculated, not stored


       +     
 [plane origin]


      The reference point of for the dimension is at the entity plane origin
      The "x" and "y" coordinates of [1] can be any value.
      The "x" and "y" coordinates of [2] can be any value.
      If Direction is "x", the dimension measures along the "x" axis
      If Direction is "y", the dimension measures along the "y" axis
      If Direction is "x" and [1][x] <> [0][x], an offset segment is drawn
      If Direction is "y" and [1][y] <> [0][y], an offset segment is drawn
      The dimension lines are always drawn in the X or Y directions of the entity plane
      The distance represented by the dimension is measured from the 
        plane origin to point [0], parallel to the appropriate axis.
      The points of the offset segment are calculated rather than stored
  */

  enum POINT_INDEX
  {
    // Do not change these enum values.  They are saved in files as the 
    // ON_COMPONENT_INDEX.m_index value.
    //
    // Indices of linear dimension definition points in 
    // the m_points[] array
    definition_pt_index    = 0, // First end of the dimension line
    leader_end_pt_index    = 1, // Other end of the leader (near the text)
    dim_pt_count           = 2, // Number of m_points[] in an ordinate dim

    // Points calculated from values in m_points[]
    text_pivot_pt = 10000, // Center of dimension text
    offset_pt_0   = 10001, // First offset point  (nearest text)
    offset_pt_1   = 10002  // Second offset point
  };

  enum DIRECTION
  {
    x = 0,  // measures horizontally
    y = 1,  // measures vertically
  };

  ON_OrdinateDimension2();
  ~ON_OrdinateDimension2();

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
    default_offset [in] - kink offset to use if m_kink_offset_0
                          or m_kink_offset_1 are ON_UNSET_VALUE
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_2dPoint Dim2dPoint(
       int point_index,
       double default_offset = 1.0
       ) const;

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
    default_offset [in] - kink offset to use if m_kink_offset_0
                          or m_kink_offset_1 are ON_UNSET_VALUE
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_3dPoint Dim3dPoint(
       int point_index,
       double default_offset = 1.0
       ) const;

  // overrides virual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = 0 ) const;

  // overrides virual ON_Geometry::GetBBox
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         ON_BOOL32 bGrowBox = false
         ) const;

  // overrides virual ON_Geometry::GetTightBoundingBox
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
    Description:
      Read from or write to a file
    Returns:
      @untitled Table
      true     Success
      false    Failure
  */
  ON_BOOL32 Write( ON_BinaryArchive& file ) const;
  ON_BOOL32 Read( ON_BinaryArchive& file );

  /*
  Description:
    Overrides virtual ON_Annotation2::NumericValue();
  Returns:
    If Direction is 'X', x coordinate of point[1]
    If Direction is 'Y', y coordinate of point[1]
  */
  double NumericValue() const;

  /*
    Description:
      Get or set the DimStyle index in the dimstyle table for the dimension
    Parameters:
      [in] int  the new index (Set)
    Returns:
      int -  The current index (Get)
  */
  int StyleIndex() const;
  void SetStyleIndex( int);

  /*
    Description:
      Gets the direction ( X or Y) that the ordinate dimension measures
      based on the relative location of the defining point and leader endpoint
    Returns:
      0: measures parallel to the entity plane x axis
      1: measures parallel to the entity plane y axis
    Remarks:
      This does not consider the dimension's explicit Direction setting 
  */
  int ImpliedDirection() const;

  /*
    Description:
      Gets or sets the direction ( X or Y) that the ordinate dimension measures
    Returns:
     -1: direction determined by dim point and leader point
      0: measures parallel to the entity plane x axis
      1: measures parallel to the entity plane y axis
  */
  int Direction() const;
  void SetDirection( int direction);

  /*
    Description:
      Get the height of the text in this dimension
      by asking the dimension's dimstyle
    Returns:
      double Height of the text
    Remarks:
      Height is in model units
  double Height() const;
  */

  /*
    Description:
      static function to provide the default UserText string for the object
    Returns:
      const wchar_t* - the default string to use
  */
  static const wchar_t* DefaultText();

  /*
    Description:
      Returns or sets the offset distance parallel to the dimension 
      line direction of from the text end of the dimension line to 
      the offset point 
      If the offset point hasn't been explicitly defined, returns 
      ON_UNSET_VALUE and a default should be used to find the point.
    Parameters:
      index [in] - which offset distance to return 
                   (0 is closer to the text)
      offset [in] - the offset distance to set
  */
  double KinkOffset( int index) const;
  void SetKinkOffset( int index, double offset);


  int m_direction;   // -1 == underermined
                     //  0 == x direction
                     //  1 == y direction

  // kink offsets added 2-4-06 - LW
  double m_kink_offset_0;  // from leader_end_point to first break point
  double m_kink_offset_1;  // from first break point to second break point

  /*
    Description:
      Calculates the 2d point locations of the dimension line kinks

    Parameters:
      p0, p1 [in] - End points of the dimension line
      direction [in] - orientation of the dimension
      default_offset [in] - Use this if offsets are ON_UNSET_VALUE
      k0, k1 [out] - The kink points
    Remarks:
      The offsets must be set to the right values before calling this, or
      If they are ON_UNSET_VALUE, they will be set to the defaults
  */
  void CalcKinkPoints( ON_2dPoint p0, ON_2dPoint p1, 
                       int direction, double default_offset,
                       ON_2dPoint& k0, ON_2dPoint& k1) const;

};



//////////
// class ON_TextEntity2
class ON_CLASS ON_TextEntity2 : public ON_Annotation2
{
  ON_OBJECT_DECLARE(ON_TextEntity2);

public:
  ON_TextEntity2();
  ~ON_TextEntity2();

  // overrides virual ON_Object::IsValid
  // Text entities with strings that contain no "printable" characters
  // are considered to be NOT valid.
  ON_BOOL32 IsValid( ON_TextLog* text_log = 0 ) const;

  // overrides virual ON_Object::Write
  ON_BOOL32 Write(ON_BinaryArchive&) const;

  // overrides virual ON_Object::Read
  ON_BOOL32 Read(ON_BinaryArchive&);

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  // overrides virual ON_Geometry::GetBBox
  // This just adds the text base point to the box
  // There is no calculation of the size of the text or its bounds
  ON_BOOL32 GetBBox(
         double*,
         double*,
         ON_BOOL32 = false
         ) const;

  // overrides virual ON_Geometry::GetTightBoundingBox
  // This just adds the text base point to the box
  // There is no calculation of the size of the text or its bounds
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
    Description:
      Get or set the Font index in the Font Table for the text

    Parameters:
      [in] int  the new index (Set)

    Returns:
      int -  The current index (Get)
  */
  int FontIndex() const;
  void SetFontIndex( int);

// 6-23-03 lw Added v2 file writing of annotation
  void GetV2Form( ON_TextEntity& text);

  void SetJustification( unsigned int justification);

  unsigned int Justification();

  // Determines whether or not to draw a Text Mask
  bool DrawTextMask() const;
  void SetDrawTextMask(bool bDraw);

  // Determines where to get the color to draw a Text Mask
  // 0: Use background color of the viewport.  Initially, gradient backgrounds will not be supported
  // 1: Use the ON_Color returned by MaskColor()
  int MaskColorSource() const;
  void SetMaskColorSource(int source);

  ON_Color MaskColor() const;  // Only works right if MaskColorSource returns 1.
                               // Does not return viewport background color
  void SetMaskColor(ON_Color color);

  // Offset for the border around text to the rectangle used to draw the mask
  // This number * CRhinoAnnotation::TextHeight() for the text is the offset 
  // on each side of the tight rectangle around the text characters to the mask rectangle.
  double MaskOffsetFactor() const;
  void SetMaskOffsetFactor(double offset);

  // Scale annotation according to detail scale factor in paperspace
  // or by 1.0 in paperspace and not in a detail
  // Otherwise, dimscale or text scale is used
  bool AnnotativeScaling() const;
  void SetAnnotativeScaling(bool b);
};

//////////
// class ON_Leader2
class ON_CLASS ON_Leader2 : public ON_Annotation2
{
  ON_OBJECT_DECLARE(ON_Leader2);

public:

  /*
    The annotation's dimstyle controls the position of TEXT,
    the size of the arrowheads, and the amount the ends of 
    linear dimension's extension lines extend beyond the 
    dimension lines.

    Leaders:

      Polyline with N=m_points.Count() points (N >= 2).

                      [N-2] ----- [N-1] TEXT
                        /         (tail)
                       /
                      /
            [1]------[2]
            /
           /
          /
        [0] (arrow)

      Leaders ignore the m_userpositionedtext setting.  If the
      default leader text handling is not adequate, then use
      a leader with no text and an ON_TextEntity2.
  */

  enum POINT_INDEX
  {
    // Do not change these enum values.  They are saved in files as the 
    // ON_COMPONENT_INDEX.m_index value.
    //
    // Indices of leader definition points in 
    // the m_points[] array
    arrow_pt_index  = 0, // arrow tip

    // Points calculated from values in m_points[]
    text_pivot_pt = 10000, // start/end of dimension text at tail
    tail_pt       = 10001
  };

  // Constructors
  ON_Leader2();
  ~ON_Leader2();
  // C++ automatically provides the correct copy constructor and operator= .
  //ON_Leader2(const ON_Leader2&);
  //ON_Leader2& operator=(const ON_Leader2&);

  // overrides virtual ON_Geometry::Transform()
  ON_BOOL32 Transform( const ON_Xform& xform );

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_2dPoint Dim2dPoint(
       int point_index
       ) const;

  /*
  Description:
    Get the m_plane coordinates of the dimension point.
  Parameters:
    point_index - [in] One of the POINT_INDEX enum values
  Returns:
    2d point or ON_UNSET_POINT if point_index or m_points[]
    array is not valid.
  */
  ON_3dPoint Dim3dPoint(
       int point_index
       ) const;

  // overrides virual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = 0 ) const;

  // overrides virual ON_Object::Write
  ON_BOOL32 Write(ON_BinaryArchive&) const;

  // overrides virual ON_Object::Read
  ON_BOOL32 Read(ON_BinaryArchive&);

  // overrides virual ON_Geometry::GetBBox
  ON_BOOL32 GetBBox(
         double*,
         double*,
         ON_BOOL32 = false
         ) const;

  // overrides virual ON_Geometry::GetTightBoundingBox
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  /*
    Description:
      Add or delete points to the leader
    Parameters:
      index [in] the point to delete
      point [in]  The point to add
    Returns:
      @untitled table
      true     Success
      False    Failure
  */
  void AddPoint( const ON_2dPoint& point);
  bool RemovePoint( int index = -1);

  /*
    Description:
      Converts an ON_Leader2 to the v2 form ON_Leader
    Parameters:
      leader [out] - the result of the conversion
  */
  void GetV2Form( ON_Leader& leader);
  bool CreateFromV2( 
      const ON_Annotation& v2_ann,
      const ON_3dmAnnotationSettings& settings,
      int dimstyle_index
      );

// April 22, 2010 Lowell - Added to support right justified text on left pointing leader tails rr64292
  bool GetTextDirection( ON_2dVector& text_dir ) const;
  bool GetArrowHeadDirection( ON_2dVector& arrowhead_dir ) const;
  bool GetArrowHeadTip( ON_2dPoint& arrowhead_tip ) const;
};


/*
  A simple dot with text that doesn't rotate witn the world axes
*/
class ON_CLASS ON_TextDot : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_TextDot);

public:
  ON_TextDot();
  ~ON_TextDot();
  // C++ automatically provides the correct copy constructor and operator= .
  //ON_TextDot( const ON_TextDot& src);
  //ON_TextDot& operator=( const ON_TextDot& src);

  void EmergencyDestroy();

  //---------------------------
  // ON_Object overrides

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Paramters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief english description of the
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

  /*
    Description: Write data values to a text file for debugging
  */
  void Dump( ON_TextLog& log) const;

  /*
    Description: Writes the object to a file

    Returns:
      @untitled Table
      true     Success
      false    Failure
  */
  ON_BOOL32 Write( ON_BinaryArchive& ar) const;

  /*
    Description: Reads the object from a file

    Returns:
      @untitled Table
      true     Success
      false    Failure
  */
  ON_BOOL32 Read( ON_BinaryArchive& ar);

  /*
    Returns: The Object Type of this object
  */
  ON::object_type ObjectType() const;

  //---------------------------
  // ON_Geometry overrides

  /*
    Returns the geometric dimension of the object ( usually 3)
  */
  int Dimension() const;

  /*
    Description:
      Get a bounding 3d WCS box of the object
    Parameters:
      [in/out] double* boxmin - pointer to dim doubles for min box corner
      [in/out] double* boxmax - pointer to dim doubles for max box corner
      [in] ON_BOOL32 growbox   - true to grow the existing box,
                            false ( the default) to reset the box
    Returns:
      true = Success
      false = Failure
    Remarks:
      Since the bounding box of this entity changes size at different
      zoom levels, the bounding box is a point at the definition point
  */
  ON_BOOL32 GetBBox( double* box_min, double* box_max, ON_BOOL32 grow_box = false) const;

  /*
    Description:
      Transform the object by a 4x4 xform matrix
    Parameters:
      [in] xform  - An ON_Xform with the transformation information
    Returns:
      true = Success
      false = Failure
    Remarks:
      The object has been transformed when the function returns
  */
  ON_BOOL32 Transform( const ON_Xform& xform);

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  const ON_3dPoint& Point() const;
  void SetPoint( const ON_3dPoint& point);

  int Height() const;
  void SetHeight( int);

  const wchar_t* TextString() const;
  void SetTextString( const wchar_t* string);

  const wchar_t* FontFace() const;
  void SetFontFace( const wchar_t* face);

  
  /*
    Description:
      Get or Set whether the dot is drawn "On Top" of other geometry
    Parameters:
      [in] bTop  bool - It is or isn't on top
    Returns:
      @untitled table
      true - on top
      false - not on top
  */
  void SetAlwaysOnTop(bool bTop);
  bool AlwaysOnTop() const;

  /*
    Description:
      Get or Set whether the dot is drawn with a transparent background
    Parameters:
      [in] bTransparent  bool - It is or isn't transparent
    Returns:
      @untitled table
      true - transparent
      false - not transparent
  */
  void SetTransparent(bool bTransparent);
  bool Transparent() const;

  /*
    Description:
      Get or Set whether the dot is drawn with Bold text
    Parameters:
      [in] bBold  bool - It is or isn't Bold
    Returns:
      @untitled table
      true - Bold
      false - not Bold
  */
  void SetBold(bool bBold);
  bool Bold() const;

  /*
    Description:
      Get or Set whether the dot is drawn with Italic text
    Parameters:
      [in] bItalic  bool - It is or isn't Italic
    Returns:
      @untitled table
      true - Italic
      false - not Italic
  */
  void SetItalic(bool bItalic);
  bool Italic() const;


  ON_3dPoint m_point;
  int m_height;        // in points
  ON_wString m_text;
  ON_wString m_fontface;
  int m_display;       // some future display flags - 
};

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

#if !defined(OPENNURBS_DIMSTYLE_INC_)
#define OPENNURBS_DIMSTYLE_INC_

class ON_CLASS ON_DimStyle : public ON_Object
{
  ON_OBJECT_DECLARE(ON_DimStyle);

public:
  enum eArrowType
  {
    solidtriangle = 0,    // 2:1
    dot = 1,
    tick = 2,
    shorttriangle = 3,    // 1:1
    arrow = 4,
    rectangle = 5,
    longtriangle = 6,     // 4:1
    longertriangle = 7,   // 6:1
  };

  ON_DimStyle();
  ~ON_DimStyle();
  // C++ default copy construction and operator= work fine.-

  ON_DimStyle& operator=( const ON_3dmAnnotationSettings& src);

  //////////////////////////////////////////////////////////////////////
  //
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

  // virtual
  void Dump( ON_TextLog& ) const; // for debugging

  // virtual
  ON_BOOL32 Write(
         ON_BinaryArchive&  // serialize definition to binary archive
       ) const;

  // virtual
  ON_BOOL32 Read(
         ON_BinaryArchive&  // restore definition from binary archive
       );

  void EmergencyDestroy();

  // virtual
  ON_UUID ModelObjectId() const;


  //////////////////////////////////////////////////////////////////////
  //
  // Interface

  void SetName( const wchar_t* );
  void SetName( const char* );

  void GetName( ON_wString& ) const;
  const wchar_t* Name() const;

  void SetIndex(int);
  int Index() const;

  void SetDefaults();
  void SetDefaultsNoExtension();


  double ExtExtension() const;
  void SetExtExtension( const double);

  double ExtOffset() const;
  void SetExtOffset( const double);

  double ArrowSize() const;
  void SetArrowSize( const double);

  double LeaderArrowSize() const;
  void SetLeaderArrowSize( const double);

  double CenterMark() const;
  void SetCenterMark( const double);

  int TextAlignment() const;
  void SetTextAlignment( ON::eTextDisplayMode);

  int ArrowType() const;
  void SetArrowType( eArrowType);

  int LeaderArrowType() const;
  void SetLeaderArrowType( eArrowType);

  int AngularUnits() const;
  void SetAngularUnits( int);

  int LengthFormat() const;
  void SetLengthFormat( int);

  int AngleFormat() const;
  void SetAngleFormat( int);

  int LengthResolution() const;
  void SetLengthResolution( int);

  int AngleResolution() const;
  void SetAngleResolution( int);

  int FontIndex() const;
  virtual void SetFontIndex( int index);

  double TextGap() const;
  void SetTextGap( double gap);

  double TextHeight() const;
  void SetTextHeight( double height);

  // added at ver 1.3
  double LengthFactor() const;
  ON_DEPRECATED void SetLengthactor( double);
  void SetLengthFactor( double); // added 6/24/07 because of typo

  bool Alternate() const;
  void SetAlternate( bool);

  double AlternateLengthFactor() const;
  ON_DEPRECATED void SetAlternateLengthactor( double);
  void SetAlternateLengthFactor( double); // added 6/24/07 because of typo

  int AlternateLengthFormat() const;
  void SetAlternateLengthFormat( int);

  int AlternateLengthResolution() const;
  void SetAlternateLengthResolution( int);

  int AlternateAngleFormat() const;
  void SetAlternateAngleFormat( int);

  int AlternateAngleResolution() const;
  void SetAlternateAngleResolution( int);

  void GetPrefix( ON_wString& ) const;
  const wchar_t* Prefix() const;
  void SetPrefix( const wchar_t*);
  void SetPrefix( wchar_t*);

  void GetSuffix( ON_wString& ) const;
  const wchar_t* Suffix() const;
  void SetSuffix( const wchar_t*);
  void SetSuffix( wchar_t*);

  void GetAlternatePrefix( ON_wString& ) const;
  const wchar_t* AlternatePrefix() const;
  void SetAlternatePrefix( const wchar_t*);
  void SetAlternatePrefix( wchar_t*);

  void GetAlternateSuffix( ON_wString& ) const;
  const wchar_t* AlternateSuffix() const;
  void SetAlternateSuffix( const wchar_t*);
  void SetAlternateSuffix( wchar_t*);

  bool SuppressExtension1() const;
  void SetSuppressExtension1( bool);

  bool SuppressExtension2() const;
  void SetSuppressExtension2( bool);

  // obsolete
  ON_DEPRECATED void Composite( const ON_DimStyle& override);

  // Don't change these enum values
  // They are used in file reading & writing
  enum eField
  {
    fn_name                        = 0,
    fn_index                       = 1,
    fn_extextension                = 2,
    fn_extoffset                   = 3,
    fn_arrowsize                   = 4,
    fn_centermark                  = 5,
    fn_textgap                     = 6,
    fn_textheight                  = 7,
    fn_textalign                   = 8,
    fn_arrowtype                   = 9,
    fn_angularunits                = 10,
    fn_lengthformat                = 11,
    fn_angleformat                 = 12,
    fn_angleresolution             = 13,
    fn_lengthresolution            = 14,
    fn_fontindex                   = 15,
    fn_lengthfactor                = 16,
    fn_bAlternate                  = 17,
    fn_alternate_lengthfactor      = 18,
    fn_alternate_lengthformat      = 19, 
    fn_alternate_lengthresolution  = 20,
    fn_alternate_angleformat       = 21, 
    fn_alternate_angleresolution   = 22,
    fn_prefix                      = 23,
    fn_suffix                      = 24,
    fn_alternate_prefix            = 25,
    fn_alternate_suffix            = 26,
    fn_dimextension                = 27,
    fn_leaderarrowsize             = 28,
    fn_leaderarrowtype             = 29,
    fn_suppressextension1          = 30,
    fn_suppressextension2          = 31,
    fn_last                        = 32, // not used - left here for sdk
                   
  // Added for v5 - 5/01/07 LW    
  // version 1.6
    fn_overall_scale               = 33,
    fn_ext_line_color_source       = 34,
    fn_dim_line_color_source       = 35,
    fn_arrow_color_source          = 36,
    fn_text_color_source           = 37,
    fn_ext_line_color              = 38,
    fn_dim_line_color              = 39,
    fn_arrow_color                 = 40,
    fn_text_color                  = 41,
    fn_ext_line_plot_color_source  = 42,
    fn_dim_line_plot_color_source  = 43,
    fn_arrow_plot_color_source     = 44,
    fn_text_plot_color_source      = 45,
    fn_ext_line_plot_color         = 46,
    fn_dim_line_plot_color         = 47,
    fn_arrow_plot_color            = 48,
    fn_text_plot_color             = 49,
    fn_ext_line_plot_weight_source = 50,
    fn_dim_line_plot_weight_source = 51,
    fn_ext_line_plot_weight_mm     = 52,
    fn_dim_line_plot_weight_mm     = 53,
    fn_tolerance_style             = 54,
    fn_tolerance_resolution        = 55,
    fn_tolerance_upper_value       = 56,
    fn_tolerance_lower_value       = 57,
    fn_tolerance_height_scale      = 58,
    fn_baseline_spacing            = 59,

  // Added for v5 - 12/15/09 LW    
  // version 1.7
    fn_draw_mask                   = 60,
    fn_mask_color_source           = 61,
    fn_mask_color                  = 62,
    fn_mask_border                 = 63,

  // Added for v5 - 12/17/09 LW    
  // version 1.8
    fn_dimscale                    = 64,
    fn_dimscale_source             = 65,

    //When fields are added to ON_DimStyleExtra,
    //   enum { eFieldCount = 64 }; in opennurbs_dimstyle.cpp
    // needs to be changed.
    fn_really_last                 = 0xFFFF
  };

  // These are obsolete - don't use
  // 5/01/07 - LW
  ON_DEPRECATED void InvalidateField( eField field);
  ON_DEPRECATED void InvalidateAllFields();
  ON_DEPRECATED void ValidateField( eField field);
  ON_DEPRECATED bool IsFieldValid( eField) const;

  // added version 1.3
  double DimExtension() const;
  void SetDimExtension( const double);

  // This section Added for v5 - 4-24-07 LW
  // version 1.6

  // Test if a specific field has been set in this dimstyle
  // and not inherited from its parent.
  bool IsFieldOverride( eField field_id) const;
  // Set a field to be overridden or not
  // Fields that aren't overrides inherit from their parent dimstyle
  void SetFieldOverride(  ON_DimStyle::eField field_id, bool bOverride);

  // Test if the dimstyle has any field override flags set
  bool HasOverrides() const;

  // Change the fields in this dimstyle to match the fields of the 
  // source dimstyle for all of the fields that are marked overridden in the source
  // and to match the parent for all of the fields not marked overriden.
  // Returns true if any overrides were set.
  bool OverrideFields( const ON_DimStyle& source, const ON_DimStyle& parent);

  // 
  // Change the fields in this dimstyle to match the fields of the 
  // parent dimstyle for all of the fields that are not marked overridden in the 
  // target dimstyle.
  // This is the complement of OverrideFields()
  bool InheritFields( const ON_DimStyle& parent);

  // Test if this dimstyle is the child of any other dimstyle
  bool IsChildDimstyle() const;

  // Test if this dimstyle is the child of a given dimstyle
  // A dimstyle may have several child dimstyles, but only one parent
  bool IsChildOf( const ON_UUID& parent_uuid) const;
  bool IsChildOf( ON_UUID& parent_uuid) const; // decl error - const forgotten

  ON_UUID ParentId() const;

  // Set the parent of this dimstyle
  void SetParentId( ON_UUID parent_uuid);
  ON_DEPRECATED void SetParent( ON_UUID& parent_uuid); // use set parent id

  // Tolerances
  // Tolerance style
  //  0: None
  //  1: Symmetrical
  //  2: Deviation
  //  3: Limits
  //  4: Basic
  int  ToleranceStyle() const;
  int  ToleranceResolution() const;
  double ToleranceUpperValue() const;
  double ToleranceLowerValue() const;
  double ToleranceHeightScale() const;

  double BaselineSpacing() const;

  void SetToleranceStyle( int style);
  void SetToleranceResolution( int resolution);
  void SetToleranceUpperValue( double upper_value);
  void SetToleranceLowerValue( double lower_value);
  void SetToleranceHeightScale( double scale);
  
  void SetBaselineSpacing( double spacing = false);

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

  // Per DimStyle DimScale
  void SetDimScaleSource(int source);
  int DimScaleSource() const;          // 0: Global DimScale, 1: DimStyle DimScale
  void SetDimScale(double scale);
  double DimScale() const;

  // Offset for the border around text to the rectangle used to draw the mask
  // This number * CRhinoAnnotation::TextHeight() for the text is the offset 
  // on each side of the tight rectangle around the text characters to the mask rectangle.
  double MaskOffsetFactor() const;

  void Scale( double scale);

  // UUID of the dimstyle this was originally copied from
  // so Restore Defaults has some place to look
  void SetSourceDimstyle(ON_UUID source_uuid);
  ON_UUID SourceDimstyle() const;

  // Defaults for values stored in Userdata extension
  static int      DefaultToleranceStyle();
  static int      DefaultToleranceResolution();
  static double   DefaultToleranceUpperValue();
  static double   DefaultToleranceLowerValue();
  static double   DefaultToleranceHeightScale();
  static double   DefaultBaselineSpacing();
  static bool     DefaultDrawTextMask(); // false
  static int      DefaultMaskColorSource(); // 0;
  static ON_Color DefaultMaskColor(); // .SetRGB(255,255,255);
  static double   DefaultDimScale(); // 1.0;
  static int      DefaultDimScaleSource(); // 0;

  bool CompareFields(const ON_DimStyle& other) const;

public:
  ON_wString m_dimstyle_name;   // String name of the style
  int m_dimstyle_index;         // Index in the dimstyle table
  ON_UUID m_dimstyle_id;

  double m_extextension; // extension line extension
  double m_extoffset;    // extension line offset
  double m_arrowsize;  // length of an arrow - may mean different things to different arrows
  double m_centermark; // size of the + at circle centers
  double m_textgap;    // gap around the text for clipping dim line
  double m_textheight; // model unit height of dimension text before applying dimscale
  int m_textalign;     // text alignment relative to the dimension line
  int m_arrowtype;     // 0: filled narrow triangular arrow
  int m_angularunits;  // 0: degrees, 1: radians
  int m_lengthformat;  // 0: decimal, 1: feet, 2: feet & inches
  int m_angleformat;   // 0: decimal degrees, ...
  int m_angleresolution;    // for decimal degrees, digits past decimal
  int m_lengthresolution;   // depends on m_lengthformat
                            // for decimal, digits past the decimal point
  int m_fontindex;     // index of the ON_Font used by this dimstyle

  // added fields version 1.2, Jan 13, 05
  double m_lengthfactor;  // (dimlfac) model units multiplier for length display
  bool m_bAlternate;      // (dimalt) display alternate dimension string (or not)
                          // using m_alternate_xxx values
  double m_alternate_lengthfactor;  // (dimaltf) model units multiplier for alternate length display
  int m_alternate_lengthformat;     // 0: decimal, 1: feet, 2: feet & inches
  int m_alternate_lengthresolution; // depends on m_lengthformat
                                    // for decimal, digits past the decimal point
  int m_alternate_angleformat;      // 0: decimal degrees, ...
  int m_alternate_angleresolution;  // for decimal degrees, digits past decimal
  ON_wString m_prefix;              // string preceding dimension value string
  ON_wString m_suffix;              // string following dimension value string
  ON_wString m_alternate_prefix;    // string preceding alternate value string
  ON_wString m_alternate_suffix;    // string following alternate value string

private:
  unsigned int m_valid;        // Obsolete deprecated field to be removed - Do not use
public:

  // field added version 1.4, Dec 28, 05
  double m_dimextension;  // (dimdle) dimension line extension past the "tip" location

  // fields added version 1.5 Mar 23 06
  double m_leaderarrowsize;       // Like dimension arrow size but applies to leaders
  int    m_leaderarrowtype;       // Like dimension arrow type but applies to leaders
  bool   m_bSuppressExtension1;   // flag to not draw extension lines
  bool   m_bSuppressExtension2;   // flag to not draw extension lines


  // Added March 23, 2008 -LW
  // This function is temporary and will be removed next time the SDK can be modified.
  class ON_DimStyleExtra* DimStyleExtension(); // can return null
  const class ON_DimStyleExtra* DimStyleExtension() const; // can return null
};

#endif


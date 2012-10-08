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

#if !defined(OPENNURBS_FONT_INC_)
#define OPENNURBS_FONT_INC_

class ON_CLASS ON_Font : public ON_Object
{
  ON_OBJECT_DECLARE(ON_Font);
public:
  ON_Font();
  ~ON_Font();
  // C++ default copy construction and operator= work fine.

  /*
  Description:
    Create a font with a specified facename and properties.
  Parameters:
    face_name - [in]
      If face_name is null or empty, then "Arial" is used.
    bBold - [in]
      True for a bold version of the font.
    bItalic - [in]
      True for an italic version of the font.
  Returns:
    True if the font was created.  The name of this font is
    the face name with " Bold", " Italic" or " Bold Italic"
    appended as
 */
  bool CreateFontFromFaceName( 
    const wchar_t* face_name,
    bool bBold,
    bool bItalic 
    );

#if defined(ON_OS_WINDOWS_GDI)
  ON_Font( const LOGFONT& logfont );
  ON_Font& operator=( const LOGFONT& logfont );
#endif

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

  // virtual
  ON_UUID ModelObjectId() const;

  //////////////////////////////////////////////////////////////////////
  //
  // Interface

  enum 
  { 

#if defined(ON_OS_WINDOWS_GDI)

    // Windows GDI facename length

    // 13 November 2008 - Dale Lear
    // Because:
    //   * Prior to this date the above "ON_OS_WINDOWS_GDI" 
    //     was misspelled and this code did not get compiled.
    //   * The Windows headers defines LF_FACESIZE = 32
    //   * ON_Font has a member wchar_t m_facename[face_name_size] array
    //   * We cannot break the SDK by changing the size of ON_Font
    //
    //  we cannot define face_name_size = LF_FACESIZE+1.  So, I'm
    //  using the same "65" we use below.  It is critical that
    //  face_name_size >= LF_FACESIZE+1
    //
    //face_name_size = LF_FACESIZE+1, // <- prior to 13 Nov but never used
    face_name_size = 65,

    // Windows GDI font weights
    bold_weight   = FW_BOLD,
    medium_weight = FW_MEDIUM,
    normal_weight = FW_NORMAL,
    light_weight  = FW_LIGHT,

    // Windows GDI character sets
    default_charset = DEFAULT_CHARSET,
    symbol_charset  = SYMBOL_CHARSET,

#else

    face_name_size = 65, // must be >= 33

    bold_weight   = 700,
    medium_weight = 500,
    normal_weight = 400,
    light_weight  = 300,

    default_charset = 1,
    symbol_charset  = 2,

#endif

    normal_font_height = 256
  }; 

  // Ratio of linefeed to character height (1.6)
  static 
  const double m_default_linefeed_ratio;

  static
  const int m_metrics_char; // ASCII code of character to used
                            // to get runtime "default" glyph
                            // metrics. (Currently an "I").

  /*
  Returns:
    True if the font's character set should be SYMBOL_CHARSET;
  */
  static
  bool IsSymbolFontFaceName( 
          const wchar_t* facename
          );

  void SetFontName( const wchar_t* );
  void SetFontName( const char* );
  
  void GetFontName( ON_wString& ) const;
  const wchar_t* FontName() const;

  void SetFontIndex(int);
  int FontIndex() const;

  /*
  Returns:
    The ratio (height of linefeed)/(height of I).
  */
  double LinefeedRatio() const;

  void SetLinefeedRatio( double linefeed_ratio );

  bool SetFontFaceName( const wchar_t* );
  bool SetFontFaceName( const char* );
  
  void GetFontFaceName( ON_wString& ) const;
  const wchar_t* FontFaceName() const;

  int FontWeight() const;
  void SetFontWeight( int);

  bool IsItalic() const;
  void SetIsItalic( bool );
  void SetItalic( bool );

  bool IsBold() const;
  void SetBold( bool );

  // Added 7/12/07 LW
  bool IsUnderlined() const;
  void SetUnderlined( bool );

  void Defaults();

  /*
  Returns:
    Height of the 'I' character when the font is drawn 
    with m_logfont.lfHeight = ON_Font::normal_font_height.
  */
  int HeightOfI() const;

  /*
  Returns:
    Height of a linefeed when the font is drawn 
    with m_logfont.lfHeight = ON_Font::normal_font_height.
  */
  int HeightOfLinefeed() const;

  /*
    Description:
      Returns the ratio of the height of a typical upper case letter 
      to the height of a whole character cell.

  Parameters:
    none

  Returns:
    double - ratio of Windows Font Height / m_HeightOfH
  */
  double AscentRatio() const;

  /*
    Description:
      Compare the visible characteristics to another font

    Parameters:
      font_to_compare - [in] The cont to compare this one to
      bCompareName    - [in] if this is set, test if the names match
                             otherwise don't compare the names

    Returns:
      true if font_to_compare matches this one
      false if font_to_match doesn't match this one

    Added for v5 - 5/20/07
  */
  bool CompareFontCharacteristics( ON_Font& font_to_compare, bool bCompareName) const;

#if defined(ON_OS_WINDOWS_GDI)
  bool SetLogFont( const LOGFONT& logfont );
  const LOGFONT& LogFont() const;
#endif

public:
  ON_wString m_font_name;      // Name of this font in the Rhino UI
  int        m_font_weight;    // Same as m_logfont.lfWeight
  bool       m_font_italic;    // Same as m_logfont.lfItalic
  bool       m_font_underlined;// Same as m_logfont.lfUnderlined (Added 7/12/07 LW)
  double     m_linefeed_ratio; // defaults to static s_linefeed_ratio.
  int        m_font_index;     // font index in Rhino font table
  ON_UUID    m_font_id;
  wchar_t    m_facename[face_name_size]; // same as m_logfont.lfFaceName ( 

public:

  /*  
  Description:
    Insures the settings in the OS specific information, like 
    the Windows m_logfont field, match the persistent m_font_* values 
    above that are used for all OSs and used in UI code.
  */
  void UpdateImplementationSettings();
#if defined(ON_OS_WINDOWS_GDI)
  // Windows specific settins
  LOGFONT m_logfont;
#endif

private:
  // volitile - can be changed by ON_Font::HeightOfI() const.
  int m_I_height; // height of the 'I' character when the font is drawn 
                  // with m_logfont.lfHeight = 256.
};

#endif

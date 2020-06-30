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

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

#include <pcl/common/utils.h> // pcl::utils::ignore


ON_OBJECT_IMPLEMENT( ON_Font, ON_Object, "4F0F51FB-35D0-4865-9998-6D2C6A99721D" );

ON_Font::ON_Font()
{
  Defaults();
}

ON_Font::~ON_Font()
{
}

bool ON_Font::CreateFontFromFaceName( 
  const wchar_t* facename,
  bool bBold,
  bool bItalic 
  )
{
  PurgeUserData();
  Defaults();

  if ( 0 == facename || 0 == facename[0] )
    facename = L"Arial";

  bool rc = SetFontFaceName(facename);

  HeightOfI();

  m_font_name = facename;
  if ( bBold )
  {
    SetBold(true);
    m_font_name += "L Bold";
  }
  if ( bItalic )
  {
    SetItalic(true);
    m_font_name += "L Italic";
  }

  return rc;
}


void ON_Font::Defaults()
{
  m_font_name.Empty();
  m_font_weight = ON_Font::normal_weight;
  m_font_italic = false;
  m_font_underlined = false;
  m_linefeed_ratio = m_default_linefeed_ratio;
  m_font_index = -1;
  memset(&m_font_id,0,sizeof(m_font_id));
  memset( &m_facename, 0, sizeof( m_facename));
  m_I_height = 0;
#if defined(ON_OS_WINDOWS_GDI)
  memset(&m_logfont,0,sizeof(m_logfont));
  m_logfont.lfHeight = normal_font_height;
  m_logfont.lfCharSet = default_charset;
#endif
  SetFontFaceName(L"Arial");
}

//////////////////////////////////////////////////////////////////////
//
// ON_Object overrides

ON_BOOL32 ON_Font::IsValid( ON_TextLog* ) const
{
  return ( m_font_name.Length() > 0 
           && m_font_index >= 0 
           && m_facename[0] > 32 
           && m_facename[64] == 0
           );
}

void ON_Font::Dump( ON_TextLog& dump ) const
{
  const wchar_t* name = FontName();
  if ( !name )
    name = L"";
  dump.Print("font index = %d\n",m_font_index);
  dump.Print("font name = \"%ls\"\n",name);
  dump.Print("font face name = \"%ls\"\n",m_facename);
  dump.Print("font weight = \"%d\"\n",m_font_weight);
  dump.Print("font is italic = \"%d\"\n",m_font_italic);
  dump.Print("font is underlined = \"%d\"\n",m_font_underlined);
  dump.Print("font linefeed ratio = \"%g\"\n", m_linefeed_ratio);
}

ON_BOOL32 ON_Font::Write(
       ON_BinaryArchive& file // serialize definition to binary archive
     ) const
{
  bool rc = file.Write3dmChunkVersion(1,2);
  while(rc)
  {
    rc = file.WriteInt(m_font_index);
    if  (!rc) break;
    rc = file.WriteString(m_font_name);
    if  (!rc) break;
    {
      // 18 October 2002 Dale Lear:
      //   Lowell, wchar_t has different sizes on different OSs.
      //   When writing a wchar_t string, you should use one
      //   of the WriteString functions.  This function must continue
      //   to use WriteShort(64,...) so old files will remain valid.
      unsigned short sh[64];
      memset(sh,0,sizeof(sh));
      int i;
      for ( i = 0; i < 64 && i < face_name_size-1; i++ )
        sh[i] = m_facename[i];
      rc = file.WriteShort(64, sh);
      if  (!rc) break;
    }

    // 1.1 additions
    rc = file.WriteInt( m_font_weight);
    if  (!rc) break;
    rc = file.WriteInt( m_font_italic);
    if  (!rc) break;
    rc = file.WriteDouble( m_linefeed_ratio);
    if  (!rc) break;

    // 1.2 addition
    rc = file.WriteUuid( m_font_id);
    if (!rc) break;

    // 1.3 addition
//    rc = file.WriteInt( m_font_underlined);
//    if  (!rc) break;

    break;
  }

  return rc;
}

ON_BOOL32 ON_Font::Read(
       ON_BinaryArchive& file // restore definition from binary archive
     )
{
  Defaults();
  m_font_index = -1;
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc && major_version == 1 ) 
  {
    int i;
    for(;;)
    {
      rc = file.ReadInt( &m_font_index );
      if  (!rc) break;
      rc = file.ReadString( m_font_name );
      if  (!rc) break;

      {
        // 18 October 2002 Dale Lear:
        //   Lowell, wchar_t has different sizes on different OSs.
        //   When writing a wchar_t string, you should use one
        //   of the WriteString functions.  This function must continue
        //   to use ReadShort(64,...) so old files will remain valid.
        unsigned short sh[64];
        rc = file.ReadShort(64, sh);
        if (!rc) break;

        wchar_t facename[65];
        for ( i = 0; i < 64; i++ )
        {
          facename[i] = sh[i];
        }
        facename[64] = 0;
        SetFontFaceName(facename);
      }

      if( minor_version >= 1 )
      {
        rc = file.ReadInt( &i );
        if (!rc) break;
        SetFontWeight(i);

        rc = file.ReadInt( &i);
        if (!rc) break;
        SetIsItalic(i?true:false);

        rc = file.ReadDouble( &m_linefeed_ratio );
        if (!rc) break;

        if ( minor_version >= 2 )
        {
          rc = file.ReadUuid( m_font_id );
          if (!rc) break;
        }
        //if ( minor_version >= 3 )
        //{
        //  rc = file.ReadInt( &i);
        //  if (!rc) break;
        //  SetUnderlined(i?true:false);
        //}
      }

      break;
    }
  }
  else
  {
    ON_ERROR("ON_Font::Read - get newer version of opennurbs");
    rc = false;
  }

  return rc;
}

// Ratio of linefeed to character height
const double ON_Font::m_default_linefeed_ratio = 1.6;

// This must be an 'I' or 'H', but we have not tested 'H'.
// There are problems with any other upper case character.
// In particular, the standard 'M' does not work.
const int ON_Font::m_metrics_char = 'I';

//////////////////////////////////////////////////////////////////////
//
// Interface
void ON_Font::SetFontName( const wchar_t* s )
{
  m_font_name = s;
}

void ON_Font::SetFontName( const char* s )
{
  m_font_name = s;
}

void ON_Font::GetFontName( ON_wString& s ) const
{
  s = m_font_name;
}

const wchar_t* ON_Font::FontName() const
{
  const wchar_t* s = m_font_name;
  return s;
}

#if defined(ON_OS_WINDOWS_GDI)
static 
int CALLBACK ON__IsSymbolFontFaceNameHelper( ENUMLOGFONTEX*, NEWTEXTMETRICEX*, DWORD, LPARAM)
{
  // If the fontname in the logfont structure has
  // a corresponding symbol font on the system, 
  // set the  lfCharSet member to SYMBOL_CHARSET, 
  // otherwise DEFAULT_CHARSET
  // The input logfont structure may be modified.
  return 7;
}
#endif

bool ON_Font::IsSymbolFontFaceName( const wchar_t* s)
{
  bool rc = false;
  pcl::utils::ignore(s);
#if defined(ON_OS_WINDOWS_GDI)
  if( s && s[0])
  {
    HDC hdc = ::GetDC( NULL);    
    if( hdc)
    {      
      LOGFONT logfont;
      memset( &logfont, 0, sizeof( logfont));
      int i;
      for ( i = 0; i < LF_FACESIZE && s[i]; i++ )
      {
        logfont.lfFaceName[i] = s[i];
      }
      logfont.lfCharSet = ON_Font::symbol_charset;
      if( 7 == ::EnumFontFamiliesEx( hdc, &logfont, (FONTENUMPROC)ON__IsSymbolFontFaceNameHelper, 0, 0))
      {
        rc = true;
      }    
      ::ReleaseDC( NULL, hdc);
    }
  }
#endif

  return rc;
}

bool ON_Font::SetFontFaceName( const wchar_t* s )
{
  int i;
  memset( &m_facename, 0, sizeof(m_facename) );
  if ( s)
  {
    for ( i = 0; i < face_name_size-1 && s[i]; i++ )
    {
      m_facename[i] = s[i];
    }
  }

#if defined(ON_OS_WINDOWS_GDI)
  memset( &m_logfont.lfFaceName, 0, sizeof(m_logfont.lfFaceName) );
#endif

  m_I_height = 0;

  UpdateImplementationSettings();

  return( m_facename[0] ? true : false);
}

bool ON_Font::SetFontFaceName( const char* s )
{
  ON_wString wstr(s);
  const wchar_t* w = wstr;
  return SetFontFaceName(w);
}

double ON_Font::AscentRatio() const
{
  return ((double)normal_font_height) / ((double)HeightOfI());
}

void ON_Font::GetFontFaceName( ON_wString& s ) const
{
  s = m_facename;
}

const wchar_t* ON_Font::FontFaceName() const
{
  const wchar_t* s = m_facename;
  return s;
}

void ON_Font::SetFontIndex(int i)
{
  m_font_index = i;
}

int ON_Font::FontIndex() const
{
  return m_font_index;
}

double ON_Font::LinefeedRatio() const
{
  return m_linefeed_ratio;
}

void ON_Font::SetLinefeedRatio( double d)
{
  m_linefeed_ratio = d;
}

int ON_Font::FontWeight() const
{
  return m_font_weight;
}

void ON_Font::SetFontWeight( int w)
{
  if ( w != m_font_weight )
  {
    if ( w < 0 )
      w = 0;
    m_font_weight = w;
    m_I_height = 0;
    UpdateImplementationSettings();
  }
}

bool ON_Font::IsItalic() const
{
  return m_font_italic;
}

void ON_Font::SetIsItalic( bool b)
{
  SetItalic( b);
}


void ON_Font::SetItalic( bool b)
{
  if ( m_font_italic != b )
  {
    m_font_italic = b?true:false;
    m_I_height = 0;
    UpdateImplementationSettings();
  }
}

bool ON_Font::IsBold() const
{
  return (FontWeight() >= bold_weight);
}


void ON_Font::SetBold( bool bBold )
{
  SetFontWeight( bBold ? bold_weight : normal_weight);
}


bool ON_Font::IsUnderlined() const
{
  return m_font_underlined;
}


void ON_Font::SetUnderlined( bool b)
{
  if ( m_font_underlined != b )
  {
    m_font_underlined = b?true:false;
    UpdateImplementationSettings();
  }
}

void ON_Font::UpdateImplementationSettings()
{
#if defined(ON_OS_WINDOWS_GDI) 
  BYTE b;
  LONG w;
  std::size_t cap0, cap1, cap, i;

  w = m_font_weight;
  if ( w < 0 )
    w = 0;
  if ( w != m_logfont.lfWeight )
  {
    m_logfont.lfWeight = w;
    m_I_height = 0;
  }

  b = m_font_italic ? 1 : 0;
  if ( b != m_logfont.lfItalic )
  {
    m_logfont.lfItalic = b;
    m_I_height = 0;
  }

  b = m_font_underlined ? 1 : 0;
  if ( b != m_logfont.lfUnderline )
  {
    m_logfont.lfUnderline = b;
    m_I_height = 0;
  }

  b = 0;
  cap0 = sizeof(m_facename)/sizeof(m_facename[0]);
  cap1 = sizeof(m_logfont.lfFaceName)/sizeof(m_logfont.lfFaceName[0]);
  cap = cap0 < cap1 ? cap0 : cap1;
  for ( i = 0; i < cap; i++ )
  {
    if ( m_logfont.lfFaceName[i] != m_facename[i] )
    {
      m_logfont.lfFaceName[i] = m_facename[i];
      b = 1;
    }
  }
  if ( b )
  {
    for ( i = cap; i < cap1; i++ )
      m_logfont.lfFaceName[i] = 0;

    m_logfont.lfCharSet = ON_Font::IsSymbolFontFaceName( m_logfont.lfFaceName )
                      ? ((unsigned char)ON_Font::symbol_charset)
                      : ((unsigned char)ON_Font::default_charset);

    m_I_height = 0;
  }
  
#endif
}

/*
Returns:
  Height of the 'I' character when the font is drawn 
  with m_logfont.lfHeight = 256.
*/
int ON_Font::HeightOfI() const
{
  if ( m_I_height  <= 0 )
  {
    // Default is height of Arial 'I'.  If we are running
    // on Windows, then we calculate the actual height of
    // an 'I' in the font.
    //   The ..ON_Font::normal_font_height/256 is here 
    //   so this code will continue to work correctly 
    //   if somebody changes ON_Font::normal_font_height.
    int I_height = (166*ON_Font::normal_font_height)/256;

#if defined(ON_OS_WINDOWS_GDI)
    if ( m_logfont.lfFaceName[0] )
    {
      // Get the height of an 'I'
      HDC hdc = ::GetDC( NULL);
      if (hdc)
      {
        LOGFONT logfont = m_logfont;
        logfont.lfHeight = normal_font_height;
        HFONT font = ::CreateFontIndirect( &logfont);
        if ( font )
        {
          wchar_t str[2];
          str[0] = ON_Font::m_metrics_char;
          str[1] = 0;
          HFONT oldfont = (HFONT)::SelectObject( hdc, font);
          ::SetBkMode( hdc, TRANSPARENT);
          ::BeginPath(hdc);
          ::ExtTextOut( hdc, 0, 0, 0, NULL, str, 1, NULL);
          ::EndPath( hdc);
          int numPoints = ::GetPath( hdc, NULL, NULL, 0);

          if( numPoints > 2)
          {
            // Allocate room for the points & point types
            LPPOINT pPoints = (LPPOINT)onmalloc( numPoints * sizeof(*pPoints) );
            LPBYTE pTypes = (LPBYTE)onmalloc( numPoints * sizeof(*pTypes) );
            if ( pTypes && pPoints)
            {
              // Get the points and types from the current path
              numPoints = ::GetPath( hdc, pPoints, pTypes, numPoints);
              if( numPoints > 2)
              {
                int ymin = pPoints[0].y;
                int ymax = ymin;
                int k;
                for( k = 1; k < numPoints; k++)
                {
                  if( pPoints[k].y < ymin)
                    ymin = pPoints[k].y;
                  else if( pPoints[k].y > ymax)
                    ymax = pPoints[k].y;
                }
                I_height = ymax - ymin + 1;
              }
            }
            onfree( pPoints);
            onfree( pTypes);
          }
          ::SelectObject( hdc, oldfont);
          ::DeleteObject( font);
        }
      }
      ::ReleaseDC( NULL, hdc);
    }
#endif
    const_cast<ON_Font*>(this)->m_I_height = I_height;
  }
  return m_I_height;
}


int ON_Font::HeightOfLinefeed() const
{
  return ( (int)( ceil(m_linefeed_ratio*HeightOfI()) ) );
}

#if defined(ON_OS_WINDOWS_GDI)

#pragma message( " --- OpenNURBS including Windows LOGFONT support in ON_Font" )

bool ON_Font::SetLogFont( const LOGFONT& logfont )
{
  if ( &m_logfont != &logfont )
  {
    memcpy(&m_logfont,&logfont,sizeof(m_logfont));
  }
    
  // synch persistent fields
  m_font_weight = m_logfont.lfWeight;
  m_font_italic = (m_logfont.lfItalic?true:false);
  m_font_underlined = (m_logfont.lfUnderline?true:false);
  memset(&m_facename[0],0,sizeof(m_facename));
  int i;
  for ( i = 0; i < face_name_size && i < LF_FACESIZE; i++ )
  {
    m_facename[i] = (wchar_t)m_logfont.lfFaceName[i]; 
  }
  m_facename[face_name_size-1] = 0;


  m_I_height = 0;

  return true;
}

const LOGFONT& ON_Font::LogFont() const
{
  return m_logfont;
}

ON_Font::ON_Font( const LOGFONT& logfont )
{
  Defaults();
  SetLogFont(logfont);
}

ON_Font& ON_Font::operator=( const LOGFONT& logfont )
{
  if ( &m_logfont == &logfont )
  {
    LOGFONT lf = logfont;
    SetLogFont(lf);
  }
  else
  {
    SetLogFont(logfont);
  }
  return *this;
}

bool ON_Font::CompareFontCharacteristics( ON_Font& other_font, bool bCompareName) const
{
  if( bCompareName && m_font_name.CompareNoCase( other_font.m_font_name))
    return false;

  if( m_font_weight != other_font.m_font_weight)
    return false;
  
  if( m_font_italic != other_font.m_font_italic)
    return false;
  
  if( m_font_underlined != other_font.m_font_underlined)
    return false;
  
  if( m_linefeed_ratio != other_font.m_linefeed_ratio)
    return false;
  
  if( _wcsicmp( m_facename, other_font.m_facename))
    return false;

  return true;
}


#endif


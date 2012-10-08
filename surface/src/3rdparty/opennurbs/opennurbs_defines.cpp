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

// {EA2EFFD2-C9A9-4cb1-BE15-D2F46290F1A1}
//const ON_UUID ON_MaterialRef::material_from_layer = 
//{ 0xea2effd2, 0xc9a9, 0x4cb1, { 0xbe, 0x15, 0xd2, 0xf4, 0x62, 0x90, 0xf1, 0xa1 } };



// {86EDFDE4-8AAF-4bcd-AB7C-F7111978D7FE}
//const ON_UUID ON_MaterialRef::material_from_parent = 
//{ 0x86edfde4, 0x8aaf, 0x4bcd, { 0xab, 0x7c, 0xf7, 0x11, 0x19, 0x78, 0xd7, 0xfe } };


// on_stricmp() is a wrapper for case insensitive string compare
// and calls one of _stricmp(), stricmp(), or strcasecmp()
// depending on OS.
int on_stricmp(const char * s1, const char * s2)
{
#if defined(ON_OS_WINDOWS)
  //return stricmp(s1,s2);
  return _stricmp(s1,s2);
#else
  return strcasecmp(s1,s2);
#endif
}

// on_strupr() calls _strupr() or strupr() depending on OS
char* on_strupr(char* s)
{
#if defined(ON_OS_WINDOWS)
  return _strupr(s); // ANSI name
#else
  if (s) {
    while (*s) {
      *s = toupper(*s);
      s++;
    }
  }
  return s;
#endif
}

// on_strlwr() calls _strlwr() or strlwr() depending on OS
char* on_strlwr(char* s)
{
#if defined(ON_OS_WINDOWS)
  return _strlwr(s); // ANSI name
#else
  if (s) {
    while (*s) {
      *s = tolower(*s);
      s++;
    }
  }
  return s;
#endif
}

// on_strrev() calls _strrev() or strrev() depending on OS
char* on_strrev(char* s)
{
#if defined(ON_OS_WINDOWS)
  return _strrev(s); // ANSI name
#else
  int i, j;
  char c;
  for ( i = 0, j = ((int)strlen(s))-1; i < j; i++, j-- ) {
    c = s[i];
    s[i] = s[j];
    s[j] = c;
  }
  return s;
#endif
}

// Windows code page support 
//   Only ON_SetStringConversionWindowsLocaleID,
//   ON_GetStringConversionWindowsLocaleID, and 
//   on_wcsicmp should look at g_s__windows_locale_id
//   and g_s__windows_locale_os.
static unsigned int g_s__windows_locale_id = 0;
static unsigned int g_s__windows_locale_os = 0; // 0 = Win 95/98/ME...
                                              // 1 = Win NT/2000/XP...

unsigned int ON_SetStringConversionWindowsLocaleID(unsigned int locale_id, ON_BOOL32 bWin9X)
{
  unsigned int rc = g_s__windows_locale_id;
  g_s__windows_locale_os = bWin9X ? 0 : 1;
  g_s__windows_locale_id = locale_id?true:false;
  return rc;
}

unsigned int ON_GetStringConversionWindowsLocaleID()
{
  return g_s__windows_locale_id;
}


static int on__hack__tolower(int c)
{
  // This tolower is a simple "hack" to provide something that
  // sort of works when OpenNURBS is used with a compiler that
  // fails to provide functional localization tools.


  // TODO: 
  //   Expand these switch statments as users provide support
  //   for symbols.  This is not the correct way to solve this
  //   problem, but it will work in some simple cases.
  //   If you are using Microsoft Developer studio in Windows,
  //   then this code is never called.
  //  
  // Before you get too carried away, study 
  //
  //  http://www.microsoft.com/globaldev/wrguide/WRG_sort.asp
  //
  // and make sure what you are attempting to do is worth the
  // trouble.  In short, this approach is doomed to fail
  // and is not good enough for robust applications that want
  // to work well with most languages.
  //
  // That said, please post your additions to the OpenNURBS
  // newsgroup so they can be included in future
  // distrubutions.

  int i;
  if ( c <= 0 )
  {
    i = c;
  }
  else if ( c <= 127 )
  {
    // ASCII character
    i = tolower(c);
  }
  else if ( c <= 255 )
  {
    // "extended" ASCII character
    switch(c)
    {
      case 192:  // UNICODE Latin capital letter A with grave  (A`)
        i = 224; // UNICODE Latin small letter A with grave    (a`)
        break;

      case 193:  // UNICODE Latin capital letter A with acute  (A')
        i = 225; // UNICODE Latin small letter A with acute    (a')
        break;

      case 194:  // UNICODE Latin capital letter A with circumflex (A^)
        i = 226; // UNICODE Latin small letter A with circumflex   (a^)
        break;

      case 195:  // UNICODE Latin capital letter A with tilde (A~)
        i = 227; // UNICODE Latin small letter A with tilde   (a~)
        break;

      case 196:  // UNICODE Latin capital letter A with diaeresis (A")
        i = 228; // UNICODE Latin small letter A with diaeresis   (a")
        break;

      case 197:  // UNICODE Latin capital letter A with ring above (A ring above)
        i = 229; // UNICODE Latin small letter A with ring above   (a ring above)
        break;

      case 198:  // UNICODE Latin capital letter Ae
        i = 230; // UNICODE Latin small letter Ae
        break;

      case 199:  // UNICODE Latin capital letter C with cedilla (C,)
        i = 231; // UNICODE Latin small letter C with cedilla   (c,)
        break;

      case 200:  // UNICODE Latin capital letter E with grave (E`)
        i = 232; // UNICODE Latin small letter E with grave   (e`)
        break;

      case 201:  // UNICODE Latin capital letter E with acute (E')
        i = 233; // UNICODE Latin small letter E with acute   (e')
        break;

      case 202:  // UNICODE Latin capital letter E with circumflex (E^)
        i = 234; // UNICODE Latin small letter E with circumflex   (e^)
        break;

      case 203:  // UNICODE Latin capital letter E with diaeresis (E")
        i = 235; // UNICODE Latin small letter E with diaeresis   (e")
        break;

      case 204:  // UNICODE Latin capital letter I with grave (I`)
        i = 236; // UNICODE Latin small letter I with grave   (i`)
        break;

      case 205:  // UNICODE Latin capital letter I with acute (I')
        i = 237; // UNICODE Latin small letter I with acute   (i')
        break;

      case 206:  // UNICODE Latin capital letter I with circumflex (I^)
        i = 238; // UNICODE Latin small letter I with circumflex   (i^)
        break;

      case 207:  // UNICODE Latin capital letter I with diaeresis (I")
        i = 239; // UNICODE Latin small letter I with diaeresis   (i")
        break;

      case 208:  // UNICODE Latin capital letter Eth (ED)
        i = 240; // UNICODE Latin small letter Eth (ed)
        break;

      case 209:  // UNICODE Latin capital letter N with tilde (N~)
        i = 241; // UNICODE Latin small letter n with tilde (n~)
        break;

      case 210:  // UNICODE Latin capital letter O with grave (O`)
        i = 242; // UNICODE Latin small letter O with grave   (o`)
        break;

      case 211:  // UNICODE Latin capital letter O with acute (O')
        i = 243; // UNICODE Latin small letter O with acute   (o')
        break;

      case 212:  // UNICODE Latin capital letter O with circumflex (O^)
        i = 244; // UNICODE Latin small letter O with circumflex   (o^)
        break;

      case 213:  // UNICODE Latin capital letter O with tilde (O~)
        i = 245; // UNICODE Latin small letter O with tilde   (o~)
        break;

      case 214:  // UNICODE Latin capital letter O with diaeresis (O")
        i = 246; // UNICODE Latin small letter O with diaeresis   (o")
        break;

      case 216:  // UNICODE Latin capital letter O with stroke (O/)
        i = 248; // UNICODE Latin small letter O with stroke   (o/)
        break;

      case 217:  // UNICODE Latin capital letter U with grave (U`)
        i = 249; // UNICODE Latin small letter U with grave   (u`)
        break;

      case 218:  // UNICODE Latin capital letter U with acute (U')
        i = 250; // UNICODE Latin small letter U with acute   (u')
        break;

      case 219:  // UNICODE Latin capital letter U with circumflex (U^)
        i = 251; // UNICODE Latin small letter U with circumflex   (u^)
        break;

      case 220:  // UNICODE Latin capital letter U with diaeresis (U")
        i = 252; // UNICODE Latin small letter U with diaeresis   (u")
        break;

      case 221:  // UNICODE Latin capital letter Y with acute (Y')
        i = 253; // UNICODE Latin small letter Y with acute   (y')
        break;

      case 222:  // UNICODE Latin capital letter Thorn (P|)
        i = 254; // UNICODE Latin small letter Thorn   (p|)
        break;

      default:
        i = c;
        break;
    }
  }
  else if ( c <= 0x0177 )
  {
    if ( 0 == (c % 2) )
      i = c+1;
    else
      i = c;
  }
  else if ( c <= 0x0192 )
  {
    switch( c)
    {
    case 0x0178:  // UNICODE Latin capital letter Y with diaeresis (Y")
      i = 0x00FF; // UNICODE Latin small letter Y with diaeresis   (y")
      break;

    case 0x0179:  // UNICODE Latin capital letter Z with acute (Z')
      i = 0x017A; // UNICODE Latin small letter Z with acute   (z')
      break;

    case 0x017B:  // UNICODE Latin capital letter Z with dot above
      i = 0x017C; // UNICODE Latin small letter Z with dot above
      break;

    case 0x017D:  // UNICODE Latin capital letter Z with caron
      i = 0x017E; // UNICODE Latin small letter Z with caron
      break;

    case 0x018F:  // UNICODE Latin capital letter Schwa
      i = 0x0259; // UNICODE Latin small letter Schwa
      break;

    default:
      i = c;
      break;
    }
  }
  else if ( c <= 0x01FF )
  {
    if ( 0 == (c % 2) )
      i = c+1;
    else
      i = c;
  }
  else
  {
    // My apologies to those of you whose languages use these symbols.
    // I am too ignorant to make good guesses at what to do here.
    // Please fill in as needed and post your changes so I can include
    // them in future versions of OpenNURBS.
    switch (c)
    {
      // example
      case 0x0391:  // UNICODE Greek capital letter alpha
        i = 0x03B1; // UNICODE Greek small letter alpha
        break;

      default:
        i = c;
    }
  }

  return i;
}

static
int on__hack__wcsicmp( const wchar_t* s1, const wchar_t* s2)
{
  // This "hack" case insensitive wchar_t compare tool is used
  // when OpenNURBS is compiled in a development environment
  // that does not provide proper localization support.

  // handle NULL strings consistently and without crashing.
  if ( !s1 ) 
  {
    return s2 ? -1 : 0;
  }
  else if ( !s2 ) 
  {
    return 1;
  }

  int rc, c1, c2;
  
  do
  {
    c1 = on__hack__tolower(*s1++);
    c2 = on__hack__tolower(*s2++);
    rc = c1-c2;
  }
  while( 0 == rc && c1 && c2 );

  return rc;
}

#if defined(ON_COMPILER_MSC)
// Disable the MSC /W4 unreachable code warning for the call to on__hack__wcsicmp()
#pragma warning( push )
#pragma warning( disable : 4702 )
#endif

int on_wcsicmp( const wchar_t* s1, const wchar_t* s2)
{
  // handle NULL strings consistently and without crashing.
  if ( !s1 ) 
  {
    return s2 ? -1 : 0;
  }
  else if ( !s2 ) 
  {
    return 1;
  }

#if defined(ON_OS_WINDOWS)

#if defined(ON_COMPILER_BORLAND)
  // Borland's compiler / C library
  return wcscmpi(s1,s2); 
#else
  // Microsoft compiler

  if ( 0 != g_s__windows_locale_id )
  {
    if ( 0 == g_s__windows_locale_os )
    {
      // On Win 95/98/ME, CompareStringW() doesn't work
      // and CompareStringA() is glacial.  So we test 
      // strings and use wcsicmp() whenever it will return
      // the right answer.  
      {
        const wchar_t* c1 = s1;
        const wchar_t* c2 = s2;
        while ( *c1 > 0 && *c1 < 128 && *c2 > 0 && *c2 < 128 )
        {
          c1++;
          c2++;
        }      
        if ( 0 == *c1 || 0 == *c2 )
        {
#if defined(ON_COMPILER_MSC1400)
          return _wcsicmp(s1,s2);
#else
          return wcsicmp(s1,s2);
#endif
        }
      }

      // These convert UNICODE to wide character strings
      ON_String a(s1);
      ON_String b(s2);

      // Wide char conversion
      int rc = ::CompareStringA(g_s__windows_locale_id,
                           NORM_IGNORECASE | NORM_IGNOREWIDTH,
                           a.Array(),
                           -1,
                           b.Array(),
                           -1);
      if (rc == CSTR_LESS_THAN)
        return -1;
      if (rc == CSTR_EQUAL)
        return 0;
      if (rc == CSTR_GREATER_THAN)
        return 1;
    }
    else
    {
      // a version of Windows with working UNICODE support
      int rc = ::CompareStringW(g_s__windows_locale_id,
                           NORM_IGNORECASE | NORM_IGNOREWIDTH,
                           s1,
                           -1,
                           s2,
                           -1);

      if (rc == CSTR_LESS_THAN)
        return -1;
      if (rc == CSTR_EQUAL)
        return 0;
      if (rc == CSTR_GREATER_THAN)
        return 1;
    }
  }

  // Microsoft's wcsicmp() doesn't work right for
  // upper/lower case accented latin characters, 
  // upper/lower case cyrillic, upper/lower case Greek,
  // Asian characters, etc.  
  //
  // Basically, if the character code >= 127 or you are
  // using a language other than US english, then 
  // Microsoft's wcsicmp() blows it.
  //
#if defined(ON_COMPILER_MSC1400)
  return _wcsicmp(s1,s2); // Microsoft's compiler / C library
#else
  return wcsicmp(s1,s2); // Microsoft's compiler / C library
#endif

#endif


#endif

  // If your compiler does not have a way to perform
  // a case insensitive compare of UNICODE strings,
  // then use the "hack" version below.
  return on__hack__wcsicmp(s1,s2);
}

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif

wchar_t* on_wcsupr(wchar_t* s)
{
#if defined(ON_OS_WINDOWS)
#if defined(ON_COMPILER_BORLAND)
  // Borland's compiler / C library
  return _wcsupr(s);
#else
  // Microsoft compiler
  return _wcsupr(s);
#endif
#else
  if (s) 
  {
    wchar_t c;
    while (*s) 
    {
      if ( 0 != (c = toupper(*s)) )
        *s = c;
      s++;
    }
  }
  return s;
#endif
}

// on_wcslwr() calls _wcslwr() or wcslwr() depending on OS
wchar_t* on_wcslwr(wchar_t* s)
{
#if defined(ON_OS_WINDOWS)
#if defined(ON_COMPILER_BORLAND)
  // Borland's compiler / C library
  return _wcslwr(s);
#else
  // Microsoft compiler
  return _wcslwr(s);
#endif
#else
  if (s) 
  {
    wchar_t c;
    while (*s) 
    {
      if ( 0 != (c = tolower(*s)) )
        *s = c;
      s++;
    }
  }
  return s;
#endif
}

void ON_wString::MakeUpper()
{
  if ( !IsEmpty() ) 
  {
#if defined(ON_OS_WINDOWS)
    if ( 0 != g_s__windows_locale_id )
    {
      if ( 0 == g_s__windows_locale_os )
      {
        // On Win 95/98/ME, LCMapStringW() doesn't work.
        // (I hope you don't need the right answer in a hurry on Win9X.)

        // These convert UNICODE to wide character strings
        ON_String in(*this);
        int len_in = in.Length();
        int max_len_out = 2*len_in+16; // if 2x for wide char expansion
        ON_String out;
        out.ReserveArray(max_len_out+1);
        out.SetLength(max_len_out+1);

        // Wide char conversion
        int rc = ::LCMapStringA(g_s__windows_locale_id, 
                              LCMAP_UPPERCASE, 
                              in.Array(), 
                              len_in,
                              out.Array(), 
                              max_len_out);
        if (rc > 0 && rc <= max_len_out)
        {
          out.SetLength(rc);
          operator=(out); // multi-byte to wchar conversion
          return;
        }
      }
      else
      {
        // a version of Windows with working UNICODE support
        int len_in = Length();
        int max_len_out = len_in+16;
        ON_wString out;
        out.ReserveArray(max_len_out+1);
        out.SetLength(max_len_out+1);

        // Wide char conversion
        int rc = ::LCMapStringW(g_s__windows_locale_id, 
                              LCMAP_UPPERCASE, 
                              Array(), 
                              len_in,
                              out.Array(), 
                              max_len_out);
        if (rc > 0 && rc <= max_len_out)
        {
          out.SetLength(rc);
          operator=(out); // very fast - simply changes reference count
          return;
        }
      }
    }
#endif

    // If ::LCMapStringA() or ::LCMapStringW() failed or we are
    // running on a non-Windows OS, then we fall through to the
    // wcslwr() function which will handle most most characters
    // in most Western European languages but is not adequate for
    // commercial quality software.
  	CopyArray();
    on_wcsupr(m_s);
  }
}

void ON_wString::MakeLower()
{
  if ( !IsEmpty() ) 
  {
#if defined(ON_OS_WINDOWS)
    if ( 0 != g_s__windows_locale_id )
    {
      if ( 0 == g_s__windows_locale_os )
      {
        // On Win 95/98/ME, LCMapStringW() doesn't work.
        // (I hope you don't need the right answer in a hurry on Win9X.)

        // These convert UNICODE to wide character strings
        ON_String in(*this);
        int len_in = in.Length();
        int max_len_out = 2*len_in+16; // if 2x for wide char expansion
        ON_String out;
        out.ReserveArray(max_len_out+1);
        out.SetLength(max_len_out+1);

        // Wide char conversion to multi-byte lower case string
        int rc = ::LCMapStringA(g_s__windows_locale_id, 
                              LCMAP_LOWERCASE, 
                              in.Array(), 
                              len_in,
                              out.Array(), 
                              max_len_out);
        if (rc > 0 && rc <= max_len_out)
        {
          out.SetLength(rc);
          operator=(out); // multi-byte to wchar conversion
          return;
        }
      }
      else
      {
        // a version of Windows with working UNICODE support
        int len_in = Length();
        int max_len_out = len_in+16;

        // ReserveArray(max_len_out+1) allocates max_len_out+2
        // wchars (room for the NULL terminator is allocatcated).
        // The +1 is just in case LCMapStringW() has a bug and
        // writes an extra wchar or puts a NULL terminator
        // in s[max_len_out]. This is a lot of paranoia, but
        // the memory cost is negligable and it will prevent
        // difficult to diagnose crashes if MS releases a buggy
        // version of LCMapStringW().
        ON_wString out;
        out.ReserveArray(max_len_out+1);
        out.SetLength(max_len_out+1);
        
        // Wide char conversion to lower case.
        // Note that changing to lower case in some languages
        // can change the string length.
        int rc = ::LCMapStringW(g_s__windows_locale_id, 
                              LCMAP_LOWERCASE, 
                              Array(), 
                              len_in,
                              out.Array(), 
                              max_len_out);
        if (rc > 0 && rc <= max_len_out)
        {
          out.SetLength(rc);
          operator=(out); // very fast - simply changes reference count
          return;
        }
      }
    }
#endif

    // If ::LCMapStringA() or ::LCMapStringW() failed or we are
    // running on a non-Windows OS, then we fall through to the
    // wcslwr() function which will handle most most characters
    // in most Western European languages but is not adequate for
    // commercial quality software.
    CopyArray();
    on_wcslwr(m_s);
  }
}

wchar_t* on_wcsrev(wchar_t* s)
{
  if ( !s )
    return 0;
  int i, j;
  wchar_t w;
  for ( j = 0; 0 != s[j]; j++ )
  {
    // empty for body
  }

  for ( i = 0, j--; i < j; i++, j-- ) 
  {
    w = s[i];
    if ( w >= 0xD800 && w <= 0xDBFF && s[i+1] >= 0xDC00 && s[i+1] <= 0xDFFF )
    {
      // UTF-16 surrogate pair
      if ( i+1 < j-1 )
      {
        s[i] = s[j-1];
        s[j-1] = w;
        w = s[i+1];
        s[i+1] = s[j];
        s[j] = w;
      }
      i++;
      j--;
    }
    else
    {
      s[i] = s[j];
      s[j] = w;
    }
  }
  return s;
}

int on_WideCharToMultiByte(
    const wchar_t* lpWideCharStr,
    int cchWideChar,
    char* lpMultiByteStr,
    int cchMultiByte
    )
{
  // 14 March 2011 Dale Lear
  //   It turns out that Windows WideCharToMultiByte does correctly
  //   convert UTF-16 to UTF-8 in Windows 7 when the code page 
  //   is CP_ACP and calls with CP_UTF8 sometimes fail to do
  //   any conversion.  So, I wrote ON_ConvertWideCharToUTF8()
  //   and opennurbs will use ON_ConvertWideCharToUTF8 to get 
  //   consistent results on all platforms.
  unsigned int error_status = 0;
  unsigned int error_mask = 0xFFFFFFFF;
  ON__UINT32 error_code_point = 0xFFFD;
  const wchar_t* p1 = 0;
  int count = ON_ConvertWideCharToUTF8(false,lpWideCharStr,cchWideChar,lpMultiByteStr,cchMultiByte,
                                       &error_status,error_mask,error_code_point,&p1);
  if ( 0 != error_status )
  {
    ON_ERROR("Error converting UTF-16 encoded wchar_t string to UTF-8 encoded char string.");
  }
  return count;
}

int on_MultiByteToWideChar(
    const char* lpMultiByteStr,
    int cchMultiByte,
    wchar_t* lpWideCharStr,
    int cchWideChar
    )
{
  // 14 March 2011 Dale Lear
  //   It turns out that Windows WideCharToMultiByte does correctly
  //   convert UTF-16 to UTF-8 in Windows 7 when the code page 
  //   is CP_ACP and calls with CP_UTF8 sometimes fail to do
  //   any conversion.  So, I wrote ON_ConvertUTF8ToWideChar()
  //   and opennurbs will use ON_ConvertUTF8ToWideChar to get 
  //   consistent results on all platforms.
  unsigned int error_status = 0;
  unsigned int error_mask = 0xFFFFFFFF;
  ON__UINT32 error_code_point = 0xFFFD;
  const char* p1 = 0;
  int count = ON_ConvertUTF8ToWideChar(lpMultiByteStr,cchMultiByte,lpWideCharStr,cchWideChar,
                                       &error_status,error_mask,error_code_point,&p1);
  if ( 0 != error_status )
  {
    ON_ERROR("Error converting UTF-8 encoded char string to UTF-16 encoded wchar_t string.");
  }
  return count;
}

int on_vsnprintf( char *buffer, size_t count, const char *format, va_list argptr )
{
#if defined(ON_OS_WINDOWS)

#if defined(ON_COMPILER_BORLAND)
  return vsprintf( buffer, format, argptr );
#else
  return _vsnprintf( buffer, count, format, argptr );
#endif

#else
  return vsnprintf( buffer, count, format, argptr );
#endif
}

int on_vsnwprintf( wchar_t *buffer, size_t count, const wchar_t *format, va_list argptr )
{
#if defined(ON_OS_WINDOWS)

#if defined(ON_COMPILER_BORLAND)
  return vswprintf( buffer, format, argptr );
#else
  return _vsnwprintf( buffer, count, format, argptr );
#endif

#else
  // if an OS doesn't support vsnwprintf(), use ASCII and hope for the best

  ON_String aformat = format; // convert format from UNICODE to ASCII

  // format an ASCII buffer
  char* abuffer = (char*)onmalloc(4*count*sizeof(*abuffer));
  int rc = on_vsnprintf( abuffer, 4*count, aformat.Array(), argptr );

  // convert formatted ASCII buffer to UNICODE
  on_MultiByteToWideChar( abuffer, (int)strlen(abuffer), buffer, (int)count );
  onfree(abuffer);  
  return rc;
#endif
}

void on_splitpath(
  const char* path,
  const char** drive,
  const char** dir,
  const char** fname,
  const char** ext
  )
{
  // The "const char* path" parameter is a UTF-8 encoded string. 
  // Since the unicode code point values for the characters we 
  // are searching for ( '/' '\' '.' ':' A-Z a-z) are all > 0 
  // and < 128, we can simply check for an array element having
  // the character value and not have to worry about dealing
  // with UTF-8 continuation values (>= 128).

  const char slash1 = '/';
  const char slash2 = '\\'; // do this even with the os is unix because
                            // we might be parsing a file name saved
                            // in Windows.

  const char* f;
  const char* e;
  const char* s;
  const char* s1;

  if ( 0 != drive )
    *drive = 0;
  if ( 0 != dir )
    *dir = 0;
  if ( 0 != fname )
    *fname = 0;
  if ( 0 != ext )
    *ext = 0;

  if ( 0 != path && 0 != *path )
  {
    // deal with Windows' drive letter (even when the os is unix)
    if ( ':' == path[1] )
    {
      if ( (path[0] >= 'A' && path[0] <= 'Z') || (path[0] >= 'a' && path[0] <= 'z') )
      {
        if ( drive )
          *drive = path;
        path += 2;
        if ( 0 == *path )
          return;
      }
    }
  }

  if ( 0 != path && 0 != *path )
  {
    e = 0;
    f = 0;
    s1 = path;
    while ( 0 != *s1 )
      s1++;
    s = (s1 > path) ? s1 - 1 : path;
  
    while ( s > path && '.' != *s && slash1 != *s && slash2 != *s )
      s--;

    if ( '.' == *s && 0 != s[1] )
    {
      // extensions must have something after the dot.
      e = s;
      s1 = e;
      s--;
    }

    while ( s > path && slash1 != *s && slash2 != *s )
      s--;

    if ( s >= path && s < s1 )
    {
      if (slash1 == *s || slash2 == *s ) 
      {
        if ( s+1 < s1 )
          f = s+1;
      }
      else if ( s == path )
      {
        f = s;
      }
    }

    if ( 0 == f )
    {
      // must have a non-empty filename in order to have and "extension"
      f = e;
      e = 0;
    }

    if ( 0 != dir && (0 == f || path < f) )
      *dir = path;

    if ( 0 != f && 0 != fname )
      *fname = f;

    if ( 0 != e && 0 != ext )
      *ext = e;
  }

}

void on_wsplitpath(
  const wchar_t* path,
  const wchar_t** drive,
  const wchar_t** dir,
  const wchar_t** fname,
  const wchar_t** ext
  )
{
  // The "const wchar_t* path" parameter is a UTF-8, UTF-16 or UTF-32
  // encoded string. Since the unicode code point values for the 
  // characters we are searching for ( '/' '\' '.' ':' A-Z a-z) are
  // all > 0 and < 128, we can simply check for an array element 
  // having the character value and not have to worry about dealing
  // with UTF-16 surrogate pair values (0xD800-0xDBFF and DC00-DFFF)
  // and UTF-8 continuation values (>= 128).

  const wchar_t slash1 = '/';
  const wchar_t slash2 = '\\'; // do this even with the os is unix because
                               // we might be parsing a file name saved
                               // in Windows.

  const wchar_t* f;
  const wchar_t* e;
  const wchar_t* s;
  const wchar_t* s1;

  if ( 0 != drive )
    *drive = 0;
  if ( 0 != dir )
    *dir = 0;
  if ( 0 != fname )
    *fname = 0;
  if ( 0 != ext )
    *ext = 0;

  if ( 0 != path && 0 != *path )
  {
    // deal with Windows' drive letter (even when the os is unix)
    if ( ':' == path[1] )
    {
      if ( (path[0] >= 'A' && path[0] <= 'Z') || (path[0] >= 'a' && path[0] <= 'z') )
      {
        if ( drive )
          *drive = path;
        path += 2;
        if ( 0 == *path )
          return;
      }
    }
  }

  if ( 0 != path && 0 != *path )
  {
    e = 0;
    f = 0;
    s1 = path;
    while ( 0 != *s1 )
      s1++;
    s = (s1 > path) ? s1 - 1 : path;
  
    while ( s > path && '.' != *s && slash1 != *s && slash2 != *s )
      s--;

    if ( '.' == *s && 0 != s[1] )
    {
      // extensions must have something after the dot.
      e = s;
      s1 = e;
      s--;
    }

    while ( s > path && slash1 != *s && slash2 != *s )
      s--;

    if ( s >= path && s < s1 )
    {
      if (slash1 == *s || slash2 == *s ) 
      {
        if ( s+1 < s1 )
          f = s+1;
      }
      else if ( s == path )
      {
        f = s;
      }
    }

    if ( 0 == f )
    {
      // must have a non-empty filename in order to have and "extension"
      f = e;
      e = 0;
    }

    if ( 0 != dir && (0 == f || path < f) )
      *dir = path;

    if ( 0 != f && 0 != fname )
      *fname = f;

    if ( 0 != e && 0 != ext )
      *ext = e;
  }

}



int ON::Version()
{
#define OPENNURBS_VERSION_DEFINITION
#include "pcl/surface/3rdparty/opennurbs/opennurbs_version.h"
  return OPENNURBS_VERSION;
#undef OPENNURBS_VERSION
#undef OPENNURBS_VERSION_DEFINITION
}

const char* ON::SourceRevision()
{
  return OPENNURBS_SRC_SVN_REVISION;
}

const char* ON::SourceBranch()
{
  return OPENNURBS_SRC_SVN_BRANCH;
}

const char* ON::DocumentationRevision()
{
  return OPENNURBS_DOC_SVN_REVISION;
}

const char* ON::DocumentationBranch()
{
  return OPENNURBS_DOC_SVN_BRANCH;
}

FILE* ON::OpenFile( // like fopen() - needed when OpenNURBS is used as a DLL
        const char* filename, // file name
        const char* filemode // file mode
        )
{
  return ON_FileStream::Open(filename,filemode);
}

FILE* ON::OpenFile( // like fopen() - needed when OpenNURBS is used as a DLL
        const wchar_t* filename, // file name
        const wchar_t* filemode // file mode
        )
{
  return ON_FileStream::Open(filename,filemode);
}

int ON::CloseFile( // like fclose() - needed when OpenNURBS is used as a DLL
        FILE* fp // pointer returned by OpenFile()
        )
{
  return ON_FileStream::Close(fp);
}

int ON::CloseAllFiles()
{
  // returns number of files closed or EOF for error
#if defined(ON_OS_WINDOWS)
  return _fcloseall(); // ANSI C name
#else
  // I can't find an fcloseall() or _fcloseall() in
  // gcc version egcs-2.91.66 19990314/Linux (egcs-1.1.2 release)
  return EOF;
#endif
}



ON::active_space ON::ActiveSpace(int i)
{
  ON::active_space as;

  switch(i)
  {
  case no_space:    as = no_space;    break;
  case model_space: as = model_space; break;
  case page_space:  as = page_space;  break;
  default:          as = no_space;    break;
  }

  return as;
}


ON::unit_system ON::UnitSystem(int i)
{
  unit_system us = no_unit_system;
  switch(i) {
  case no_unit_system: us = no_unit_system; break;

  case angstroms: us = angstroms; break;

  case nanometers: us = nanometers; break;
  case microns: us = microns; break;
  case millimeters: us = millimeters; break;
  case centimeters: us = centimeters; break;
  case decimeters: us = decimeters; break;
  case meters: us = meters; break;
  case dekameters: us = dekameters; break;
  case hectometers: us = hectometers; break;
  case kilometers: us = kilometers; break;
  case megameters: us = megameters; break;
  case gigameters: us = gigameters; break;
  case microinches: us = microinches; break;
  case mils: us = mils; break;
  case inches: us = inches; break;
  case feet: us = feet; break;
  case yards: us = yards; break;
  case miles: us = miles; break;
  case printer_point: us = printer_point; break;
  case printer_pica: us = printer_pica; break;
  case nautical_mile: us = nautical_mile; break;
  case astronomical: us = astronomical; break;
  case lightyears: us = lightyears; break;
  case parsecs: us = parsecs; break;
  case custom_unit_system: us = custom_unit_system; break;
  default: us = no_unit_system; break;
  }
  return us;
}

double ON::UnitScale(
                     const class ON_3dmUnitsAndTolerances& u_and_t_from, 
                     const class ON_3dmUnitsAndTolerances& u_and_t_to
                     )
{
  return ON::UnitScale( u_and_t_from.m_unit_system, u_and_t_to.m_unit_system );
}

double ON::UnitScale(
    ON::unit_system us_from,
    const class ON_UnitSystem& us_to
    )
{
  double scale = 1.0;
  ON::unit_system us1 = us_to.m_unit_system;
  if ( ON::custom_unit_system == us1 )
  {
    if ( us_to.m_custom_unit_scale > 0.0 && ON_IsValid(us_to.m_custom_unit_scale) )
    {
      scale *= us_to.m_custom_unit_scale;
      us1 = ON::meters;
    }
  }
  return scale*ON::UnitScale(us_from,us1);
}

double ON::UnitScale(
    const class ON_UnitSystem& us_from, 
    ON::unit_system us_to
    )
{
  double scale = 1.0;
  ON::unit_system us0 = us_from.m_unit_system;
  if ( ON::custom_unit_system == us0 )
  {
    if ( us_from.m_custom_unit_scale > 0.0 && ON_IsValid(us_from.m_custom_unit_scale) )
    {
      scale /= us_from.m_custom_unit_scale;
      us0 = ON::meters;
    }
  }
  return scale*ON::UnitScale(us0,us_to);
}

double ON::UnitScale(
                     const class ON_UnitSystem& u_and_t_from, 
                     const class ON_UnitSystem& u_and_t_to
                     )
{
  double scale = 1.0;
  ON::unit_system us_from = u_and_t_from.m_unit_system;
  ON::unit_system us_to   = u_and_t_to.m_unit_system;

  if ( ON::no_unit_system != us_from && ON::no_unit_system != us_to )
  {
    if ( ON::custom_unit_system == us_from 
         && ON_IsValid(u_and_t_from.m_custom_unit_scale) 
         && u_and_t_from.m_custom_unit_scale > 0.0 )
    {
      scale /= u_and_t_from.m_custom_unit_scale;
      us_from = ON::meters;
    }

    if ( ON::custom_unit_system == us_to 
         && ON_IsValid(u_and_t_to.m_custom_unit_scale) 
         && u_and_t_to.m_custom_unit_scale > 0.0 )
    {
      scale *= u_and_t_to.m_custom_unit_scale;
      us_to = ON::meters;
    }

    scale *= ON::UnitScale( us_from, us_to );
  }

  return scale;
}

static bool IsEnglishUnit( ON::unit_system us )
{
  return (
          ON::microinches == us
          || ON::mils == us
          || ON::inches == us
          || ON::feet == us
          || ON::yards == us
          || ON::miles == us
          || ON::printer_point == us
          || ON::printer_pica == us
         );
}

double ON::UnitScale(
            ON::unit_system u0, // from
            ON::unit_system u1  // to
            )
{
  // Scale factor for changing unit systems
  // Examples 
  //   100.0  = UnitScale( ON::meters, ON::centimeters ) 
  //     2.54 = UnitScale( ON::inches, ON::centimeters ) 
  //    12.0  = UnitScale( ON::feet, ON::inches ) 

  // the default cases are here to keep lint quiet
  double scale = 1.0;
  
  if (  u0 != u1
        && u1 != ON::custom_unit_system
        && ((int)u1) > 0 && ((int)u1) < 26
        // switch weeds out bogus values of u0
      ) 
  switch( u0 ) 
  {
  case ON::angstroms:
    scale = UnitScale( meters, u1)*1.0e-10;
    break;

  case ON::nanometers:
    scale = UnitScale( meters, u1)*1.0e-9;
    break;

  case ON::microns:
    scale = UnitScale( meters, u1)*1.0e-6;
    break;

  case ON::millimeters:
    switch( u1 ) 
    {
    case ON::meters:      scale = 1.0e-3; break;
    case ON::microns:     scale = 1.0e+3; break;
    case ON::centimeters: scale = 1.0e-1; break;

    default:
      scale = IsEnglishUnit(u1)
            ? UnitScale( inches, u1 )/25.4
            : UnitScale( meters, u1 )*1.0e-3;
      break;
    }
    break;

  case ON::centimeters:
    switch( u1 ) 
    {
    case ON::meters:      scale = 1.0e-2; break;
    case ON::millimeters: scale = 1.0e+1; break;

    default:
      scale = IsEnglishUnit(u1)
            ? UnitScale( inches, u1 )/2.54
            : UnitScale( meters, u1 )*1.0e-2;
      break;
    }
    break;

  case ON::decimeters:
    scale = IsEnglishUnit(u1)
          ? UnitScale( inches, u1 )/0.254
          : UnitScale( meters, u1 )*1.0e-1;
    break;

  case ON::meters:
    switch( u1 ) 
    {
    case ON::angstroms:      scale = 1.0e+10; break;
    case ON::nanometers:     scale = 1.0e+9;  break;
    case ON::microns:        scale = 1.0e+6;  break;
    case ON::millimeters:    scale = 1.0e+3;  break;
    case ON::centimeters:    scale = 1.0e+2;  break;
    case ON::decimeters:     scale = 1.0e1;   break;
    case ON::meters:         scale = 1.0;     break;
    case ON::dekameters:     scale = 1.0e-1;  break;
    case ON::hectometers:    scale = 1.0e-2;  break;
    case ON::kilometers:     scale = 1.0e-3;  break;
    case ON::megameters:     scale = 1.0e-6;  break;
    case ON::gigameters:     scale = 1.0e-9;  break;

    case ON::nautical_mile:  scale = 1.0/1852.0; break;
    case ON::astronomical:   scale = 1.0/1.4959787e+11; break;
    case ON::lightyears:     scale = 1.0/9.4607304725808e+15; break;
    case ON::parsecs:        scale = 1.0/3.08567758e+16; break;

    default:
      if ( IsEnglishUnit(u1) )
        scale = UnitScale( inches, u1 )/0.0254;
      break;
    }
    break;

  case ON::dekameters:
    scale = UnitScale( meters, u1 )*10.0;
    break;

  case ON::hectometers:
    scale = UnitScale( meters, u1 )*100.0;
    break;

  case ON::kilometers:
    scale = IsEnglishUnit(u1)
          ? UnitScale( inches, u1 )/0.0000254
          : UnitScale( meters, u1 )*1000.0;
    break;

  case ON::megameters:
    scale = UnitScale( meters, u1 )*1.0e6;
    break;

  case ON::gigameters:
    scale = UnitScale( meters, u1 )*1.0e9;
    break;

  case ON::microinches:
    scale = UnitScale( inches, u1 )*1.0e-6;
    break;

  case ON::mils:
    scale = UnitScale( inches, u1 )*1.0e-3;
    break;

  case ON::inches:
    switch( u1 ) 
    {
    case ON::angstroms:       scale = 2.54e+8; break;
    case ON::nanometers:      scale = 2.54e+7; break;
    case ON::microns:         scale = 2.54e+4; break;
    case ON::millimeters:     scale = 25.4; break;
    case ON::centimeters:     scale = 2.54; break;
    case ON::decimeters:      scale = 2.54e-1; break;
    case ON::meters:          scale = 2.54e-2; break;
    case ON::dekameters:      scale = 2.54e-3; break;
    case ON::hectometers:     scale = 2.54e-4; break;
    case ON::kilometers:      scale = 2.54e-5; break;
    case ON::megameters:      scale = 2.54e-8; break;
    case ON::gigameters:      scale = 2.54e-11; break;

    case ON::printer_point: scale = 72.0;            break;
    case ON::printer_pica:  scale = 6.0;             break;
    case ON::microinches: scale = 1.0e+6;            break;
    case ON::mils:        scale = 1.0e+3;            break;
    case ON::inches:      scale = 1.0;               break;
    case ON::feet:        scale = 1.0/12.0;          break;
    case ON::yards:       scale = 1.0/36.0;          break;
    case ON::miles:       scale = 1.0/(12.0*5280.0); break;

    default:
      scale = UnitScale( meters, u1 )*2.54e-2;
      break;
    }
    break;

  case ON::feet:
    switch( u1 ) 
    {      
    case ON::yards:       scale = 1.0/3.0; break;
    case ON::miles:       scale = 1.0/5280.0; break;
    default:
      scale = UnitScale( inches, u1 )*12.0;
      break;
    }
    break;

  case ON::yards:
    switch( u1 ) 
    {      
    case ON::feet:        scale = 3.0; break;
    case ON::miles:       scale = 1.0/1760.0; break;
    default:
      scale = UnitScale( inches, u1 )*36.0;
      break;
    }
    break;

  case ON::miles:
    if ( ON::feet == u1 )
    {
      scale = 5280.0;
    }
    else
    {
      scale = IsEnglishUnit(u1)
            ? UnitScale( inches, u1 )*12.0*5280.0
            : UnitScale( meters, u1 )*1609.344;
    }
    break;

  case ON::printer_point:
    scale = UnitScale( inches, u1 )/72.0;
    break;

  case ON::printer_pica:
    scale = UnitScale( inches, u1 )/6.0;
    break;

  case ON::nautical_mile:
    scale = UnitScale( meters, u1 )*1852.0;
    break;

  case ON::astronomical:
    // 1.4959787e+11  http://en.wikipedia.org/wiki/Astronomical_unit
    // 1.495979e+11   http://units.nist.gov/Pubs/SP811/appenB9.htm  
    //    An astronomical unit (au) is the mean distance from the 
    //    center of the earth to the center of the sun.
    scale = UnitScale( meters, u1 )*1.4959787e+11;
    break;

  case ON::lightyears:
    // 9.4607304725808e+15 http://en.wikipedia.org/wiki/Light_year
    // 9.46073e+15 meters  http://units.nist.gov/Pubs/SP811/appenB9.htm
    //    A light year is the distance light travels in one Julian year.
    //    The speed of light is exactly 299792458 meters/second.
    //    A Julian year is exactly 365.25 * 86400 seconds and is 
    //    approximately the time it takes for one earth orbit.
    scale = UnitScale( meters, u1 )*9.4607304725808e+15;
    break;

  case ON::parsecs:
    // 3.08567758e+16  // http://en.wikipedia.org/wiki/Parsec
    // 3.085678e+16    // http://units.nist.gov/Pubs/SP811/appenB9.htm  
    scale = UnitScale( meters, u1 )*3.08567758e+16;
    break;

  case ON::custom_unit_system:
  case ON::no_unit_system:
    // nothing to do here
    break;
  }

  return scale;
}


//// distance_display_mode ///////////////////////////////////
enum distance_display_mode
{
  decimal     = 0, 
  fractional  = 1,
  feet_inches = 2
};

ON::distance_display_mode ON::DistanceDisplayMode(int i)
{
  distance_display_mode dm = decimal;
  switch (i) 
  {
  case decimal:
    dm = decimal;
    break;
  case fractional:
    dm = fractional;
    break;
  case feet_inches:
    dm = feet_inches;
    break;
  }
  return dm;
}


ON::point_style ON::PointStyle(int i)
{
  //convertintegertopoint_styleenum
  point_style ps = unknown_point_style;
  switch (i) {
  case not_rational: ps = not_rational; break;
  case homogeneous_rational: ps = homogeneous_rational; break;
  case euclidean_rational: ps = euclidean_rational; break;
  default: ps = unknown_point_style; break;
  }
  return ps;
}


ON::knot_style ON::KnotStyle(int i)
{
  //convertintegertoknot_styleenum
  knot_style ks = unknown_knot_style;
  switch (i) {
  case uniform_knots: ks = uniform_knots; break;
  case quasi_uniform_knots: ks = quasi_uniform_knots; break;
  case piecewise_bezier_knots: ks = piecewise_bezier_knots; break;
  case clamped_end_knots: ks = clamped_end_knots; break;
  case non_uniform_knots: ks = non_uniform_knots; break;
  default: ks = unknown_knot_style; break;
  }
  return ks;
}

ON::continuity ON::Continuity(int i)
{
  continuity c = unknown_continuity;

  switch(i)
  {
  case unknown_continuity: c = unknown_continuity; break;
  case C0_continuous: c = C0_continuous; break;
  case C1_continuous: c = C1_continuous; break;
  case C2_continuous: c = C2_continuous; break;
  case G1_continuous: c = G1_continuous; break;
  case G2_continuous: c = G2_continuous; break;
  
  // 30 March 2003 Dale Lear added these
  case C0_locus_continuous: c = C0_locus_continuous; break;
  case C1_locus_continuous: c = C1_locus_continuous; break;
  case C2_locus_continuous: c = C2_locus_continuous; break;
  case G1_locus_continuous: c = G1_locus_continuous; break;
  case G2_locus_continuous: c = G2_locus_continuous; break;

  case Cinfinity_continuous: c = Cinfinity_continuous; break;

  case Gsmooth_continuous: c = Gsmooth_continuous; break;
  };

  return c;
}

ON::continuity ON::ParametricContinuity(int i)
{
  // "erase" the locus setting.
  continuity c = unknown_continuity;

  switch(i)
  {
  case unknown_continuity: c = unknown_continuity; break;
  case C0_continuous: c = C0_continuous; break;
  case C1_continuous: c = C1_continuous; break;
  case C2_continuous: c = C2_continuous; break;
  case G1_continuous: c = G1_continuous; break;
  case G2_continuous: c = G2_continuous; break;
  case C0_locus_continuous: c = C0_continuous; break;
  case C1_locus_continuous: c = C1_continuous; break;
  case C2_locus_continuous: c = C2_continuous; break;
  case G1_locus_continuous: c = G1_continuous; break;
  case G2_locus_continuous: c = G2_continuous; break;
  case Cinfinity_continuous: c = Cinfinity_continuous; break;
  case Gsmooth_continuous: c = Gsmooth_continuous; break;
  };

  return c;
}


ON::continuity ON::PolylineContinuity(int i)
{
  continuity c = unknown_continuity;

  switch(i)
  {
  case unknown_continuity: c = unknown_continuity; break;
  case C0_continuous: c = C0_continuous; break;
  case C1_continuous: c = C1_continuous; break;
  case C2_continuous: c = C1_continuous; break;
  case G1_continuous: c = G1_continuous; break;
  case G2_continuous: c = G1_continuous; break;
  case C0_locus_continuous: c = C0_locus_continuous; break;
  case C1_locus_continuous: c = C1_locus_continuous; break;
  case C2_locus_continuous: c = C1_locus_continuous; break;
  case G1_locus_continuous: c = G1_locus_continuous; break;
  case G2_locus_continuous: c = G1_locus_continuous; break;
  case Cinfinity_continuous: c = C1_continuous; break;
  case Gsmooth_continuous: c = G1_continuous; break;
  };

  return c;
}


ON::curve_style ON::CurveStyle(int i)
{
  //convertintegertocurve_styleenum
  curve_style cs = unknown_curve_style;
  switch (i) {
  case line: cs = line; break;
  case circle: cs = circle; break;
  case ellipse: cs = ellipse; break;
  case parabola: cs = parabola; break;
  case hyperbola: cs = hyperbola; break;
  case planar_polyline: cs = planar_polyline; break;
  case polyline: cs = polyline; break;
  case planar_freeform_curve: cs = planar_freeform_curve; break;
  case freeform_curve: cs = freeform_curve; break;
  default: cs = unknown_curve_style; break;
  }
  return cs;
}

ON::surface_style ON::SurfaceStyle(int i)
{
  //convertintegertosurface_styleenum
  surface_style ss = unknown_surface_style;
  
  switch (i) {
  case plane: ss = plane; break;
  case circular_cylinder: ss = circular_cylinder; break;
  case elliptical_cylinder: ss = elliptical_cylinder; break;
  case circular_cone: ss = circular_cone; break;
  case elliptical_cone: ss = elliptical_cone; break;
  case sphere: ss = sphere; break;
  case torus: ss = torus; break;
  case surface_of_revolution: ss = surface_of_revolution; break;
  case ruled_surface: ss = ruled_surface; break;
  case freeform_surface: ss = freeform_surface; break;
  default: ss = unknown_surface_style; break;
  }
  return ss;
}

ON::sort_algorithm ON::SortAlgorithm(int i)
{
  sort_algorithm sa = ON::quick_sort;
  
  switch (i) {
  case ON::heap_sort: sa = ON::heap_sort; break;
  case ON::quick_sort: sa = ON::quick_sort; break;
  default: sa = ON::quick_sort; break;
  }
  return sa;
}

ON::endian ON::Endian(int i)
{ // convert integer to endian enum
  endian e = (i<=0) ? little_endian : big_endian;
  return e;
}

ON::endian ON::Endian()
{
  // returns endian-ness of cpu.
  union {
    int i;
    unsigned char b[sizeof(int)];
  } u;
  u.i = 1;
  return (u.b[0] == 1) ? little_endian : big_endian;
}

ON::archive_mode ON::ArchiveMode(int i)
{
  // convert integer to endian enum
  archive_mode a = read;
  switch(i) {
  case read:      a = read; break;
  case write:     a = write; break;
  case readwrite: a = readwrite; break;
  case read3dm:   a = read3dm; break;
  case write3dm:  a = write3dm; break;
  }
  return a;
}

ON::view_projection ON::ViewProjection(int i)
{
  // convert integer to view_projection enum
  view_projection v = ON::unknown_view;
  switch(i) 
  {
  case ON::parallel_view:          v = ON::parallel_view;          break;
  case ON::perspective_view:       v = ON::perspective_view;       break;
  }
  return v;
}

bool ON::IsParallelProjection( ON::view_projection proj )
{
  return ON::parallel_view == proj;
}

bool ON::IsPerspectiveProjection( ON::view_projection proj )
{
  return ( ON::perspective_view == proj );
}

ON::coordinate_system ON::CoordinateSystem(int i)
{
  // convert integer to coordinate_system enum
  coordinate_system cs = world_cs;
  switch(i) {
  case world_cs:  cs = world_cs; break;
  case camera_cs: cs = camera_cs; break;
  case clip_cs:   cs = clip_cs; break;
  case screen_cs: cs = screen_cs; break;
  }
  return cs;
}

ON::exception_type ON::ExceptionType(int i)
{
  // convert integer to exception_type enum
  ON::exception_type e = unknown_exception;
  switch(i) {
  case out_of_memory:               e = out_of_memory; break;
  case unable_to_write_archive:     e = unable_to_write_archive; break;
  case unable_to_read_archive:      e = unable_to_read_archive; break;
  case unable_to_seek_archive:      e = unable_to_seek_archive; break;
  case unexpected_end_of_archive:   e = unexpected_end_of_archive; break;
  case unexpected_value_in_archive: e = unexpected_value_in_archive; break;
  };
  return e;
}

ON::layer_mode ON::LayerMode(int i)
{
  ON::layer_mode m = normal_layer;
	switch(i)
  {
    case normal_layer:        m = normal_layer;       break;
    case hidden_layer:        m = hidden_layer;       break;
    case locked_layer:        m = locked_layer;       break;
  }
  return m;
}

ON::object_mode ON::ObjectMode(int i)
{
  ON::object_mode m = normal_object;
	switch(i)
  {
    case normal_object:  m = normal_object;  break;
    case hidden_object:  m = hidden_object;  break;
    case locked_object:  m = locked_object;  break;
    case idef_object:    m = idef_object;    break;
  }
  return m;
}

ON::object_color_source ON::ObjectColorSource(int i)
{
  // convert integer to object_mode enum
  ON::object_color_source cs = color_from_layer;
  switch (i) 
  {
  case color_from_layer: // use color assigned to layer
    cs = color_from_layer;
    break;
  case color_from_object: // use color assigned to object
    cs = color_from_object;
    break;
  case color_from_material:  // use diffuse render material color
    cs = color_from_material;
    break;
  case color_from_parent:
    cs = color_from_parent;
    break;
  }
  return cs;
}

ON::plot_color_source ON::PlotColorSource(int i)
{
  // convert integer to object_mode enum
  ON::plot_color_source cs = plot_color_from_layer;
  switch (i) 
  {
  case plot_color_from_layer:
    cs = plot_color_from_layer;
    break;
  case plot_color_from_object:
    cs = plot_color_from_object;
    break;
  case plot_color_from_display: 
    cs = plot_color_from_display;
    break;
  case plot_color_from_parent:
    cs = plot_color_from_parent;
    break;
  }
  return cs;
}

ON::plot_weight_source ON::PlotWeightSource(int pw)
{
  switch(pw)
  {
  case plot_weight_from_layer:  return plot_weight_from_layer;  break;
  case plot_weight_from_object: return plot_weight_from_object; break;
  case plot_weight_from_parent: return plot_weight_from_parent; break;
  }
  return plot_weight_from_layer;
}


ON::object_linetype_source ON::ObjectLinetypeSource(int i)
{
  // convert integer to object_mode enum
  ON::object_linetype_source ls = linetype_from_layer;
  switch (i) {
  case linetype_from_layer: // use linetype assigned to layer
    ls = linetype_from_layer;
    break;
  case linetype_from_object: // use linetype assigned to object
    ls = linetype_from_object;
    break;
  case linetype_from_parent:
    ls = linetype_from_parent;
    break;
  }
  return ls;
}

ON::object_material_source ON::ObjectMaterialSource(int i)
{
  ON::object_material_source ms = material_from_layer;
  switch(i) {
  case material_from_layer:
    ms = material_from_layer;
    break;
  case material_from_object:
    ms = material_from_object;
    break;
  case material_from_parent:
    ms = material_from_parent;
    break;
  }
  return ms;
}

ON::light_style ON::LightStyle(int i)
{
  // convert integer to light_style enum
  light_style ls = unknown_light_style;
  switch(i)
  {
    case unknown_light_style: ls = unknown_light_style; break;
    //case view_directional_light: ls = view_directional_light; break;
    //case view_point_light: ls = view_point_light; break;
    //case view_spot_light: ls = view_spot_light; break;
    case camera_directional_light: ls = camera_directional_light; break;
    case camera_point_light: ls = camera_point_light; break;
    case camera_spot_light: ls = camera_spot_light; break;
    case world_directional_light: ls = world_directional_light; break;
    case world_point_light: ls = world_point_light; break;
    case world_spot_light: ls = world_spot_light; break;
    case ambient_light: ls = ambient_light; break;
    case world_linear_light: ls = world_linear_light; break;
    case world_rectangular_light: ls = world_rectangular_light; break;
  }
  return ls;
}

ON::curvature_style ON::CurvatureStyle(int i)
{
  // convert integer to light_style enum
  ON::curvature_style cs = unknown_curvature_style;
  switch(i) {
  case gaussian_curvature:
    cs = gaussian_curvature;
    break;
  case mean_curvature:
    cs = mean_curvature;
    break;
  case min_curvature: 
    // minimum unsigned radius of curvature
    cs = min_curvature;
    break;
  case max_curvature: 
    // maximum unsigned radius of curvature
    cs = max_curvature;
    break;
  //case section_curvature_x:
    // unsigned normal curvature with respect to sections cut perp to world x axis
    //cs = section_curvature_x;
    //break;
  //case section_curvature_y:
    // unsigned normal curvature with respect to sections cut perp to world y axis
    //cs = section_curvature_y;
    //break;
  //case section_curvature_z:
    // unsigned normal curvature with respect to sections cut perp to world z axis
    //cs = section_curvature_z;
    //break;
  }
  return cs;
}

/*enum view_type // This is already in the header. I commented it out to see if it would compile and it does. John Croudy.
{
  model_view_type     = 0,
  plot_page_view_type = 1,
  nested_view_type    = 2 
};*/

ON::view_type ON::ViewType(int vt)
{
  switch(vt)
  {
  case model_view_type:  return (model_view_type);  break;
  case page_view_type:   return (page_view_type);   break;
  case nested_view_type: return (nested_view_type); break;
  }

  return (model_view_type);
}


ON::display_mode ON::DisplayMode(int i)
{
  // convert integer to light_style enum
  ON::display_mode dm = default_display;
  switch(i) {
  case default_display:
    dm = default_display;
    break;
  case wireframe_display:
    dm = wireframe_display;
    break;
  case shaded_display:
    dm = shaded_display;
    break;
  case renderpreview_display: 
    dm = renderpreview_display;
    break;
  }
  return dm;
}

ON::texture_mode ON::TextureMode(int i)
{
  // convert integer to texture_mode enum
  ON::texture_mode tm;
  switch (i) {
  case no_texture:
    tm = no_texture;
    break;
  case modulate_texture:
    tm = modulate_texture;
    break;
  case decal_texture:
    tm = decal_texture;
    break;
  default:
    tm = no_texture;
    break;
  }
  return tm;
}

ON::object_type ON::ObjectType(int i)
{
  // convert integer to object_type enum
  object_type ot = unknown_object_type;
  switch(i) 
  {
  case unknown_object_type:  ot = unknown_object_type; break;

  case point_object:         ot = point_object; break;
  case pointset_object:      ot = pointset_object; break;
  case curve_object:         ot = curve_object; break;
  case surface_object:       ot = surface_object; break;
  case brep_object:          ot = brep_object; break;
  case mesh_object:          ot = mesh_object; break;
  case layer_object:         ot = layer_object; break;
  case material_object:      ot = material_object; break;
  case light_object:         ot = light_object; break;
  case annotation_object:    ot = annotation_object; break;
  case userdata_object:      ot = userdata_object; break;
  case instance_definition:  ot = instance_definition; break;
  case instance_reference:   ot = instance_reference; break;
  case text_dot:             ot = text_dot; break;
  case grip_object:          ot = grip_object; break;
  case detail_object:        ot = detail_object; break;
  case hatch_object:         ot = hatch_object; break;
  case morph_control_object: ot = morph_control_object; break;
  case loop_object:          ot = loop_object; break;
  case polysrf_filter:       ot = polysrf_filter; break;
  case edge_filter:          ot = edge_filter; break;
  case polyedge_filter:      ot = polyedge_filter; break;
  case meshvertex_object:    ot = meshvertex_object; break;
  case meshedge_object:      ot = meshedge_object; break;
  case meshface_object:      ot = meshface_object; break;
  case cage_object:          ot = cage_object; break;
  case phantom_object:       ot = phantom_object; break;
  case extrusion_object:     ot = extrusion_object; break;

  default: ot = unknown_object_type; break;
  }

  return ot;
}

ON::bitmap_type ON::BitmapType(int i)
{
  // convert integer to object_type enum
  bitmap_type bt = unknown_bitmap_type;
  switch(i) {
  case unknown_bitmap_type: bt = unknown_bitmap_type; break;
  case windows_bitmap:      bt = windows_bitmap; break;
  case opengl_bitmap:       bt = opengl_bitmap; break;
  case png_bitmap:          bt = png_bitmap; break;
  default: bt = unknown_bitmap_type; break;
  }
  return bt;
}

ON::object_decoration ON::ObjectDecoration(int i)
{
  ON::object_decoration d;
  switch(i)
  {
  case no_object_decoration: d = no_object_decoration; break;
  case start_arrowhead:      d = start_arrowhead; break;
  case end_arrowhead:        d = end_arrowhead;   break;
  case both_arrowhead:       d = both_arrowhead;  break;
  default:                   d = no_object_decoration; break;
  }
  return d;
}


ON::osnap_mode ON::OSnapMode(int i)
{
  ON::osnap_mode osm;
  switch((unsigned int)i)
  {
  case os_none:          osm = os_none; break;
  case os_near:          osm = os_near; break;
  case os_focus:         osm = os_focus; break;
  case os_center:        osm = os_center; break;
  case os_vertex:        osm = os_vertex; break;
  case os_knot:          osm = os_knot; break;
  case os_quadrant:      osm = os_quadrant; break;
  case os_midpoint:      osm = os_midpoint; break;
  case os_intersection:  osm = os_intersection; break;
  case os_end:           osm = os_end; break;
  case os_perpendicular: osm = os_perpendicular; break;
  case os_tangent:       osm = os_tangent; break;
  case os_point:         osm = os_point; break;
  case os_all_snaps:     osm = os_all_snaps; break;
  default:
    osm = os_none;
    break;
  };
  return osm;
}


ON::cubic_loft_end_condition ON::CubicLoftEndCondition(int i)
{
  ON::cubic_loft_end_condition e;
  switch(i)
  {
  case cubic_loft_ec_quadratic:
    e = ON::cubic_loft_ec_quadratic;
    break;
  case cubic_loft_ec_linear:
    e = ON::cubic_loft_ec_linear;
    break;
  case cubic_loft_ec_cubic:
    e = ON::cubic_loft_ec_cubic;
    break;
  case cubic_loft_ec_natural:
    e = ON::cubic_loft_ec_natural;
    break;
  case cubic_loft_ec_unit_tangent:
    e = ON::cubic_loft_ec_unit_tangent;
    break;
  case cubic_loft_ec_1st_derivative:
    e = ON::cubic_loft_ec_1st_derivative;
    break;
  case cubic_loft_ec_2nd_derivative:
    e = ON::cubic_loft_ec_2nd_derivative;
    break;
  case cubic_loft_ec_free_cv:
    e = ON::cubic_loft_ec_free_cv;
    break;
  default:
    ON_ERROR("ON::CubicLoftEndCondition(i) value of i is not valid.");
    e = ON::cubic_loft_ec_quadratic;
    break;
  }
  return e;
}


ON::mesh_type ON::MeshType(int i)
{
  mesh_type mt = default_mesh;
  switch(i)
  {
  case default_mesh:  mt = default_mesh;  break;
  case render_mesh:   mt = render_mesh;   break;
  case analysis_mesh: mt = analysis_mesh; break;
  case preview_mesh:  mt = preview_mesh; break;
  case any_mesh:      mt = any_mesh;      break;
  default:            mt = default_mesh;  break;
  }
  return mt;
}


ON::eAnnotationType ON::AnnotationType(int i)
{
  // convert integer to eAnnotationType enum
  eAnnotationType at = dtNothing;
  switch(i) {
  case dtNothing:
    at = dtNothing;
    break;
  case dtDimLinear:
    at = dtDimLinear;
    break;
  case dtDimAligned:
    at = dtDimAligned;
    break;
  case dtDimAngular:
    at = dtDimAngular;
    break;
  case dtDimDiameter:
    at = dtDimDiameter;
    break;
  case dtDimRadius:
    at = dtDimRadius;
    break;
  case dtLeader:
    at = dtLeader;
    break;
  case dtTextBlock:
    at = dtTextBlock;
    break;
  case dtDimOrdinate:
    at = dtDimOrdinate;
    break;
  }
  return at;
}

ON::eTextDisplayMode ON::TextDisplayMode( int i)
{
  eTextDisplayMode m = dtAboveLine;
  switch( i)
  {
  case dtHorizontal:
    m = dtHorizontal;
    break;
  case dtAboveLine:
    m = dtAboveLine;
    break;
  case dtInLine:
    m = dtInLine;
    break;
  }
  return m;
}


// Windows code page support 
//   Only ON_SetStringConversionWindowsCodePage
//   and ON_GetStringConversionWindowsCodePage 
//   should look at g_s__windows_code_page.
static unsigned int g_s__windows_code_page = 0;

unsigned int ON_SetStringConversionWindowsCodePage( unsigned int code_page )
{
  unsigned int prev_cp = g_s__windows_code_page;
  g_s__windows_code_page = code_page;
  return prev_cp;
}

unsigned int ON_GetStringConversionWindowsCodePage()
{
  return g_s__windows_code_page;
}

/*
ON_TimeLimit::ON_TimeLimit()
{
  m_time_limit[0] = 0;
  m_time_limit[1] = 0;
}

ON_TimeLimit::ON_TimeLimit(ON__UINT64 time_limit_seconds)
{
  SetTimeLimit(time_limit_seconds);
}

void ON_TimeLimit::SetTimeLimit(ON__UINT64 time_limit_seconds)
{
  m_time_limit[0] = 0;
  m_time_limit[1] = 0;
  if ( time_limit_seconds > 0 )
  {
    // This is a crude implementation that works
    // unless clock() is close to wrapping around
    // or time_limit_seconds is unreasonably large.
    clock_t max_ticks = (clock_t)(time_limit_seconds*((double)CLOCKS_PER_SEC));
    if ( max_ticks > 0 )
    {
      clock_t now_clock = ::clock();
      clock_t max_clock = max_ticks + now_clock;
      time_t ::time()
      if ( now_clock < max_clock )
      {
        *((clock_t*)(&m_time_limit[0])) = max_clock;
      }
    }
  }
}

bool ON_TimeLimit::Continue() const
{
  clock_t max_clock = *((clock_t*)&m_time_limit[0]);
  return ( max_clock <= 0 || ::clock() <= max_clock );
}

bool ON_TimeLimit::IsSet() const
{
  clock_t max_clock = *((clock_t*)&m_time_limit[0]);
  return ( max_clock > 0 );
}
*/

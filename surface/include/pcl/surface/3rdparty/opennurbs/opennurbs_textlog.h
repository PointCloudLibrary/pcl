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

#if !defined(ON_TEXTLOG_INC_)
#define ON_TEXTLOG_INC_

#include <pcl/pcl_exports.h>

class PCL_EXPORTS ON_CLASS ON_TextLog
{
public:
  /*
  Description:
    Create a text log that dumps to the virtual function
    void ON_TextLog::AppendText().
  */
  ON_TextLog();

  /*
  Description:
    Create a text log that dumps to an ASCII file.
  Parameters:
    fp - [in] Pointer to an open ASCII text file.  The file
              pointer must remain valid as long as the text
              log is in use.
  */
  ON_TextLog( FILE* fp); // dump to open ASCII text file

  /*
  Description:
    Create a text log that dumps to a string.
  Parameters:
    s - [in] String that must exist as long as
             the text log is in use.
  */
  ON_TextLog( ON_wString& s );

  virtual ~ON_TextLog();

  void SetDoubleFormat( const char* ); // default is %g
  void GetDoubleFormat( ON_String& ) const;

  void SetFloatFormat( const char* ); // default is %g
  void GetFloatFormat( ON_String& ) const;

  void PushIndent();
  void PopIndent();
  int IndentSize() const; //  0: one tab per indent
                          // >0: number of spaces per indent
  void SetIndentSize(int);
  
  void PrintWrappedText( const char*, int = 60 );    // last arg is maximum line length
  void PrintWrappedText( const wchar_t*, int = 60 ); // last arg is maximum line length

  /*
  Description:
    Print a formatted ASCII string of up to 2000 characters.
  Parameters:
    format - [in] NULL terminated format control string 
  Remarks:
    To print strings longer than 2000 characters, you must
    use ON_TextLog::PrintString.
  See Also:
    ON_TextLog::PrintString
  */
  void Print( const char* format, ... );

  /*
  Description:
    Print a formatted INICODE string of up to 2000 characters.
  Parameters:
    format - [in] NULL terminated format control string 
  Remarks:
    To print strings longer than 2000 characters, you must
    use ON_TextLog::PrintString.
  See Also:
    ON_TextLog::PrintString
  */
  void Print( const wchar_t* format, ... );

  void Print( float );
  void Print( double );
  void Print( const ON_2dPoint& );
  void Print( const ON_3dPoint& );
  void Print( const ON_4dPoint& );
  void Print( const ON_2dVector& );
  void Print( const ON_3dVector& );
  void Print( const ON_Xform& );
  void Print( const ON_UUID& );
  void Print( const ON_COMPONENT_INDEX& );

  /*
  Description:
    Print an unformatted UNICODE string of any length.
  Parameters:
    string - [in]
  */
  void Print( const ON_wString& string );

  /*
  Description:
    Print an unformatted ASCII string of any length.
  Parameters:
    string - [in]
  */
  void Print( const ON_String& string );

  void Print( const ON_3dPointArray&, const char* = NULL );
  void Print( 
         const ON_Matrix&, 
         const char* = NULL, // optional preamble
         int = 0             // optional number precision
    );

  // printing utilities
  /*
  Description:
    Same as calling Print("\n");
  */
  void PrintNewLine();

  /*
  Description:
    Print an unformatted ASCII string of any length.
  Parameters:
    s - [in] NULL terminated ASCII string.
  */
  void PrintString( const char* s );

  /*
  Description:
    Print an unformatted UNICODE string of any length.
  Parameters:
    s - [in] NULL terminated UNICODE string.
  */
  void PrintString( const wchar_t* s );

  void PrintRGB( const ON_Color& );

  void PrintTime( const struct tm& );

  void PrintPointList( 
    int,               // dim
    ON_BOOL32,              // true for rational points
    int,               // count
    int,               // stride
    const double*,     // point[] array
    const char* = NULL // optional preabmle
    );

  void PrintPointGrid( 
    int,               // dim
    ON_BOOL32,              // true for rational points
    int, int,          // point_count0, point_count1
    int, int,          // point_stride0, point_stride1
    const double*,     // point[] array
    const char* = NULL // optional preabmle
    );
    
  void PrintKnotVector( 
    int,             // order
    int,             // cv_count
    const double*    // knot[] array
    );

  ON_TextLog& operator<<( const char* );
  ON_TextLog& operator<<( char );
  ON_TextLog& operator<<( short );
  ON_TextLog& operator<<( int );
  ON_TextLog& operator<<( float );
  ON_TextLog& operator<<( double );
  ON_TextLog& operator<<( const ON_2dPoint& );
  ON_TextLog& operator<<( const ON_3dPoint& );
  ON_TextLog& operator<<( const ON_4dPoint& );
  ON_TextLog& operator<<( const ON_2dVector& );
  ON_TextLog& operator<<( const ON_3dVector& );
  ON_TextLog& operator<<( const ON_Xform& );

protected:
  FILE* m_pFile;
  ON_wString* m_pString;

  
  /*
  Description:
    If the ON_TextLog(ON_wString& wstr) constructor was used, the
    default appends s to wstr.  If the ON_TextLog(FILE* fp) 
    constructor was used, the default calls fputs( fp, s).
    In all other cases, the default calls printf("%s",s).
  Parameters:
    s - [in];
  */
  virtual
  void AppendText(
        const char* s
        );

  /*
  Description:
    If the ON_TextLog(ON_wString& wstr) constructor was used, the
    default appends s to wstr.  In all other cases, the default 
    converts the string to an ON_String and calls the ASCII
    version AppendText(const char*).
  Parameters:
    s - [in];
  */
  virtual
  void AppendText(
        const wchar_t* s
        );
                  
private:
  ON_String m_indent;
  ON_String m_double_format;
  ON_String m_double2_format;
  ON_String m_double3_format;
  ON_String m_double4_format;
  ON_String m_float_format;
  ON_String m_float2_format;
  ON_String m_float3_format;
  ON_String m_float4_format;

  ON_String m_line;

  int m_beginning_of_line; // 0
  int m_indent_size;       // 0 use tabs, > 0 = number of spaces per indent level

private:
  // no implementation
  ON_TextLog( const ON_TextLog& );
  ON_TextLog& operator=( const ON_TextLog& );

};


#endif

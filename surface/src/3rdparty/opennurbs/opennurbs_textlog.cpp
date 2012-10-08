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

//////////////////////////////////////////////////////////////////////////////

ON_TextLog::ON_TextLog() : m_pFile(0), m_pString(0), m_indent(""), m_beginning_of_line(1), m_indent_size(0)
{
  SetFloatFormat("%g");
  SetDoubleFormat("%.17g");
}

ON_TextLog::ON_TextLog( FILE* pFile ) : m_pFile(pFile), m_pString(0), m_indent(""), m_beginning_of_line(1), m_indent_size(0)
{
  SetFloatFormat("%g");
  SetDoubleFormat("%.17g");
}

ON_TextLog::ON_TextLog( ON_wString& wstr ) : m_pFile(0), m_pString(&wstr), m_indent(""), m_beginning_of_line(1), m_indent_size(0)
{
  SetFloatFormat("%g");
  SetDoubleFormat("%.17g");
}

ON_TextLog::~ON_TextLog()
{
}

void ON_TextLog::SetDoubleFormat(const char* sFormat)
{
  m_double_format = sFormat;
  m_double2_format = m_double_format + ", " + m_double_format;
  m_double3_format = m_double2_format + ", " + m_double_format;
  m_double4_format = m_double3_format + ", " + m_double_format;
}

void ON_TextLog::GetDoubleFormat( ON_String& s ) const
{
  s = m_double_format;
}

void ON_TextLog::SetFloatFormat(const char* sFormat)
{
  m_float_format = sFormat;
  m_float2_format = m_float_format + ", " + m_float_format;
  m_float3_format = m_float2_format + ", " + m_float_format;
  m_float4_format = m_float3_format + ", " + m_float_format;
}

void ON_TextLog::GetFloatFormat( ON_String& s ) const
{
  s = m_float_format;
}

void ON_TextLog::PushIndent()
{
  if ( m_indent_size > 0 ) {
    int i;
    for ( i = 0; i < m_indent_size; i++ ) {
      m_indent += ' ';
    }
  }
  else {
    m_indent += "\t";
  }
}

void ON_TextLog::PopIndent()
{
  const int length = m_indent.Length();
  const int indent_lenth = m_indent_size>0 ? m_indent_size : 1;
  if ( length >= indent_lenth ) {
    m_indent.SetLength(length-indent_lenth);
  }
  else {
    m_indent.Destroy();
  }
}

int ON_TextLog::IndentSize() const
{
  //  0: one tab per indent
  // >0: number of spaces per indent
  return m_indent_size;
}

void ON_TextLog::SetIndentSize(int s)
{
  m_indent_size = (s>0) ? s : 0;
}

void ON_TextLog::Print( const char* format, ... )
{
  // format message and append it to the log
  const int MAX_MSG_LENGTH = 2047;
  char s[MAX_MSG_LENGTH+1];
  va_list args;

  s[0] = 0;
  if (format) 
  {
    va_start(args, format);
    on_vsnprintf( s, MAX_MSG_LENGTH-1, format, args);
    va_end(args);
    s[MAX_MSG_LENGTH] = 0;
  }
  if ( *s ) 
  {
    char* s0 = s;
    char* s1 = s;
    for ( s1 = s0; *s1; s1++) {
      if ( *s1 == '\n' ) {
        *s1 = 0;
        if ( m_beginning_of_line && m_indent && m_indent[0] )
          AppendText( m_indent );
        if (*s0) 
          AppendText(s0);
        AppendText("\n");
        m_beginning_of_line = 1;
        s0 = s1+1;
      }
    }
    if (*s0) {
      if ( m_beginning_of_line && m_indent && m_indent[0] )
        AppendText( m_indent );
      AppendText(s0);
      m_beginning_of_line = 0;
    }
  }
}

void ON_TextLog::Print( const wchar_t* wformat, ... )
{
  // format message and append it to the log
  const int MAX_MSG_LENGTH = 2047;
  wchar_t s[MAX_MSG_LENGTH+1];
  va_list args;

  s[0] = 0;
  if (wformat) 
  {
    va_start(args, wformat);
    on_vsnwprintf( s, MAX_MSG_LENGTH-1, wformat, args);
    va_end(args);
    s[MAX_MSG_LENGTH] = 0;
  }
  if ( *s ) 
  {
    wchar_t* s0 = s;
    wchar_t* s1 = s;
    for ( s1 = s0; *s1; s1++) {
      if ( *s1 == '\n' ) {
        *s1 = 0;
        if ( m_beginning_of_line && m_indent && m_indent[0] )
          AppendText( m_indent );
        if (*s0) 
          AppendText(s0);
        AppendText("\n");
        m_beginning_of_line = 1;
        s0 = s1+1;
      }
    }
    if (*s0) {
      if ( m_beginning_of_line && m_indent && m_indent[0] )
        AppendText( m_indent );
      AppendText(s0);
      m_beginning_of_line = 0;
    }
  }
}


void ON_TextLog::AppendText( const char* s )
{
  // This is a virtual function 
  if ( s && *s ) 
  {
    if ( m_pString )
    {
      (*m_pString) += s;
    }
    else if ( m_pFile ) 
    {
      fputs( s, m_pFile );
    }
    else
    {
      printf("%s",s);
    }
  }
}

void ON_TextLog::AppendText( const wchar_t* s )
{
  // This is a virtual function 
  if ( m_pString )
  {
    (*m_pString) += s;
  }
  else
  {
    // If sizeof(wchar_t) = 2, str = s performs
    // performs UTF-16 to UTF-8 conversion.
    // If sizeof(wchar_t) = 4, str = s performs
    // performs UTF-32 to UTF-8 conversion.
    ON_String str = s;
    AppendText(str.Array());
  }
}

void ON_TextLog::Print( float x )
{
  if ( ON_UNSET_FLOAT == x )
    Print("ON_UNSET_FLOAT");
  else
    Print(m_float_format,x);
}

void ON_TextLog::Print( double x )
{
  if ( ON_UNSET_VALUE == x )
    Print("ON_UNSET_VALUE");
  else
    Print(m_double_format,x);
}

void ON_TextLog::Print( const ON_2dPoint& p )
{
  Print("(");
  Print(m_double2_format, p.x, p.y);
  Print(")");
}

void ON_TextLog::Print( const ON_3dPoint& p )
{
  Print("(");
  if ( ON_3dPoint::UnsetPoint == p )
    Print("UnsetPoint");
  else
    Print(m_double3_format, p.x, p.y, p.z );
  Print(")");
}

void ON_TextLog::Print( const ON_4dPoint& p )
{
  Print("[");
  Print(m_double4_format, p.x, p.y, p.z, p.w );
  Print("]");
}

void ON_TextLog::Print( const ON_2dVector& p )
{
  Print("<");
  Print(m_double2_format, p.x, p.y);
  Print(">");
}

void ON_TextLog::Print( const ON_3dVector& p )
{
  Print("<");
  if ( ON_3dVector::UnsetVector == p )
    Print("UnsetVector");
  else
    Print(m_double3_format, p.x, p.y, p.z);
  Print(">");
}

void ON_TextLog::Print( const ON_Xform& xform )
{
  if ( xform.IsIdentity() )
  {
    Print("identity transformation\n");
  }
  else if ( xform.IsZero() )
  {
    Print("zero transformation\n");
  }
  else
  {
    Print(m_double4_format,xform[0][0],xform[0][1],xform[0][2],xform[0][3]);
    Print("\n");
    Print(m_double4_format,xform[1][0],xform[1][1],xform[1][2],xform[1][3]);
    Print("\n");
    Print(m_double4_format,xform[2][0],xform[2][1],xform[2][2],xform[2][3]);
    Print("\n");
    Print(m_double4_format,xform[3][0],xform[3][1],xform[3][2],xform[3][3]);
    Print("\n");
  }
}

void ON_TextLog::Print( const ON_UUID& uuid )
{
  Print("%08X-%04X-%04x-%02X%02X-%02X%02X%02X%02X%02X%02X", 
        uuid.Data1, uuid.Data2, uuid.Data3,
        uuid.Data4[0], uuid.Data4[1], uuid.Data4[2], uuid.Data4[3],
        uuid.Data4[4], uuid.Data4[5], uuid.Data4[6], uuid.Data4[7]
        );
}

void ON_TextLog::Print( const ON_COMPONENT_INDEX& ci )
{
  switch( ci.m_type )
  {
    case ON_COMPONENT_INDEX::invalid_type:
      Print("invalid_type(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::brep_vertex:
      Print("brep_vertex(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::brep_edge:
      Print("brep_edge(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::brep_face:
      Print("brep_face(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::brep_trim:
      Print("brep_trim(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::brep_loop:
      Print("brep_loop(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::mesh_vertex:
      Print("mesh_vertex(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::meshtop_vertex:
      Print("meshtop_vertex(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::meshtop_edge:
      Print("meshtop_edge(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::mesh_face:
      Print("mesh_face(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::idef_part:
      Print("idef_part(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::polycurve_segment:
      Print("polycurve_segment(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::pointcloud_point:
      Print("pointcloud_point(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::group_member:
      Print("group_member(%d)",ci.m_index);
      break;
    case ON_COMPONENT_INDEX::no_type:
      Print("no_type(%d)",ci.m_index);
      break;
    default:
      Print("ON_COMPONENT_INDEX(%d,%d)",ci.m_type,ci.m_index);
      break;
  }
}

void ON_TextLog::Print( const ON_wString& string )
{
  const wchar_t* s = string;
  if ( s && *s )
    AppendText(s);
}

void ON_TextLog::Print( const ON_String& string )
{
  const char* s = string;
  if ( s && *s )
    AppendText(s);
}

void ON_TextLog::PrintString( const char* s )
{
  if ( s && *s )
    AppendText(s);
}

void ON_TextLog::PrintNewLine()
{
  Print("\n");
}


void ON_TextLog::PrintString( const wchar_t* s )
{
  if ( s && *s )
    AppendText(s);
}

void ON_TextLog::PrintRGB( const ON_Color& color )
{
  if ( color == ON_UNSET_COLOR )
    Print("ON_UNSET_COLOR");
  else
    Print("%d %d %d",color.Red(),color.Green(),color.Blue());
}

void ON_TextLog::PrintTime( const struct tm& t )
{
  if (   0 != t.tm_sec
      || 0 != t.tm_min
      || 0 != t.tm_hour
      || 0 != t.tm_mday
      || 0 != t.tm_mon
      || 0 != t.tm_year
      || 0 != t.tm_wday
    )
  {
    const char* sDayName[8] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday","<invalid day>"};
    const char* sMonName[13] = {"January","February","March","April","May","June",
                               "July","August","September","October","November","December","<invalid month>"};
    int wday = t.tm_wday;
    if ( wday < 0 || wday > 6 )
      wday = 7;
    int mon = t.tm_mon;
    if ( mon < 0 || mon > 11 )
      mon = 12;

    Print("%s %s %02d %02d:%02d:%02d %4d",
                sDayName[wday],
                sMonName[mon],
                t.tm_mday,
                t.tm_hour,
                t.tm_min,
                t.tm_sec,
                t.tm_year+1900);
  }
}


void ON_TextLog::PrintPointList( int dim, int is_rat, int count, int stride, const double* P,
                                const char* sPreamble )
{
  double w, x;
  int i, j, cvdim;

  ON_String preamble = "";
  if ( sPreamble && *sPreamble )
    preamble += sPreamble;
  cvdim = (is_rat) ? dim+1 : dim;

  if ( count == 0 ) {
    Print( "%sEMPTY point list\n", preamble.Array() );
  }
  else if ( !P ) {
    Print( "%sNULL point list\n", preamble.Array() );
  }

  for ( i = 0; i < count; i++ ) {
    Print( "%s[%2d] %c", preamble.Array(), i, (is_rat) ? '[' : '(' );
    Print( m_double_format, P[0] );
    for ( j = 1; j < cvdim; j++ ) {
      Print( ", ");
      Print(m_double_format, P[j] );
    }
    Print("%c", (is_rat) ? ']' : ')' );
    if ( is_rat ) 
    {
      w = P[dim];
      if ( w != 0.0 ) 
      {
        // print euclidean coordinates
        w = 1.0/w;
        x = w*P[0];
        Print( " = (");
        Print( m_double_format, x );
        for ( j = 1; j < dim; j++ ) 
        {
          x = w*P[j];
          Print( ", ");
          Print( m_double_format, x );
        }
        Print(")");
      }
    }
    Print("\n");
    P += stride;
  }
}

void ON_TextLog::PrintPointGrid( int dim, int is_rat, 
                                int point_count0, int point_count1, 
                                int point_stride0, int point_stride1,
                                const double* P,
                                const char* sPreamble )
{
  char s[1024];
  int i;
  if (!sPreamble || !sPreamble[0])
    sPreamble = "point";
  for ( i = 0; i < point_count0; i++ ) {
    sprintf( s,  "%s[%2d]", sPreamble, i );
    PrintPointList( dim, is_rat, point_count1, point_stride1, P + i*point_stride0, s );
  }
}

void ON_TextLog::PrintKnotVector( int order, int cv_count, const double* knot )
{
  int i, i0, mult, knot_count;
  if ( !knot )
    Print("NULL knot vector\n");
  if ( order < 2 )
    Print("knot vector order < 2\n");
  if ( cv_count < order )
    Print("knot vector cv_count < order\n");
  if ( order >= 2 && cv_count >= order && knot ) {
    knot_count = ON_KnotCount( order, cv_count );
    i = i0 = 0;
    Print("index                     value  mult       delta\n");
    while ( i < knot_count ) {
      mult = 1;
      while ( i+mult < knot_count && knot[i] == knot[i+mult] )
        mult++;
      if ( i == 0 ) {
        Print( "%5d  %23.17g  %4d\n", i, knot[i], mult );
      }
      else {
        Print( "%5d  %23.17g  %4d  %10.4g\n", i, knot[i], mult, knot[i]-knot[i0] );
      }
      i0 = i;
      i += mult;
    }
  }
}

void ON_TextLog::Print( const ON_3dPointArray& a, const char* sPreamble )
{
  const double* p = (a.Array() ? &a.Array()[0].x : NULL );
  PrintPointList( 3, false, a.Count(), 3, p, sPreamble );
}

void ON_TextLog::Print( const ON_Matrix& M, const char* sPreamble, int precision )
{
  double x;
  char digit[10] = {'0','1','2','3','4','5','6','7','8','9'};
  char* sRow;
  char* sIJ;
  int xi, row_count, column_count, row_index, column_index;
  
  row_count = M.RowCount();
  column_count = M.ColCount();

  sRow = (char*)alloca( (5*column_count + 2 + 64)*sizeof(*sRow) );

  if ( !sPreamble )
    sPreamble = "Matrix";

  Print("%s (%d rows %d columns)\n",sPreamble,row_count,column_count);
  for ( row_index = 0; row_index < row_count; row_index++ ) {
    sIJ = sRow;
    Print("%5d:",row_index);
    if ( precision > 3 ) {
      for ( column_index = 0; column_index < column_count; column_index++ ) {
        x = M.m[row_index][column_index];
        Print( " %8f",x);
      }
      Print("\n");
    }
    else {
      for ( column_index = 0; column_index < column_count; column_index++ ) {
        x = M.m[row_index][column_index];
        if ( x == 0.0 ) {
          strcpy( sIJ, "  0   " );
          sIJ += 4;
        }
        else {
          *sIJ++ = ' ';
          *sIJ++ = ( x >0.0 ) ? '+' : '-';
          x = fabs( x );
          if      ( x >= 10.0 ) {
            *sIJ++ = '*';
            *sIJ++ = ' ';
            *sIJ++ = ' ';
          }
          else if ( x <= ON_SQRT_EPSILON) {
            *sIJ++ = '0';
            *sIJ++ = ' ';
            *sIJ++ = ' ';
          }
          else if ( x < 0.1) {
            *sIJ++ = '~';
            *sIJ++ = ' ';
            *sIJ++ = ' ';
          }
          else if ( x < .95 ) {
            *sIJ++ = '.';
            xi = (int)floor(x*10.0);
            if ( xi > 9 )
              xi = 9;
            else if (xi < 1)
              xi = 1;
            *sIJ++ = digit[xi];
            *sIJ++ = '~';
          }
          else {
            xi = (int)floor(x);
            if ( xi < 1 )
              xi = 1;
            else if (xi > 9)
              xi = 9;
            *sIJ++ = digit[xi];
            if ( x == floor(x) ) {
              *sIJ++ = ' ';
              *sIJ++ = ' ';
            }
            else {
              *sIJ++ = '.';
              *sIJ++ = '~';
            }
          }
        }
      }
      *sIJ = 0;
      Print("%s\n",sRow);
    }
  }
}

ON_TextLog& ON_TextLog::operator<<(const char* s)
{
  Print( "%s", s );
  return *this;
}

ON_TextLog& ON_TextLog::operator<<(char c)
{
  Print( "%c", c );
  return *this;
}

ON_TextLog& ON_TextLog::operator<<(short i)
{
  int ii = (int)i;
  Print("%d", ii );
  return *this;
}

ON_TextLog& ON_TextLog::operator<<(int i)
{
  Print("%d",i);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<(float x)
{
  Print(m_float_format,x);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<(double x)  
{
  Print(m_double_format,x);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<( const ON_2dPoint& p )
{
  Print(p);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<( const ON_3dPoint& p )
{
  Print(p);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<( const ON_4dPoint& p )
{
  Print(p);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<( const ON_2dVector& p )
{
  Print(p);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<( const ON_3dVector& p )
{
  Print(p);
  return *this;
}

ON_TextLog& ON_TextLog::operator<<( const ON_Xform& xform )
{
  Print(xform);
  return *this;
}

void ON_TextLog::PrintWrappedText( const char* s, int line_length )
{
  ON_wString ws = s;
  PrintWrappedText(ws,line_length);
}

static void wsncpy(wchar_t* dst, const wchar_t* src, int n)
{
  // can't use _wcsncpy() because this has to compile on UNIX boxes
  if ( dst && n > 0 ) {
    if ( src ) {
      while ( 0 != (*dst++ = *src++) && n-- > 0 );
    }
    else
      *dst = 0;
  }
}


void ON_TextLog::PrintWrappedText( const wchar_t* s, int line_length )
{
  ON_Workspace ws;
  if ( s && *s && line_length > 0 ) {
    const int max_line_length = line_length+255;
    wchar_t* sLine = (wchar_t*)ws.GetMemory((max_line_length+1)*sizeof(*sLine));
    const int wrap_length = line_length;
    int i  = 0;
    int i1 = 0;
    int isp = 0;
    ON_BOOL32 bPrintLine = false;
    while ( s[i] ) {
      i1 = i;
      if ( s[i] == 10 || s[i] == 13 ) {
        // hard break at CR or LF
        i++;
        if ( s[i] == 10 && s[i-1] == 13 ) {
          // it's a CR+LF hard end of line - skip LF too
          i++;
        }      
        bPrintLine = true;
      }
      else if ( i && s[i] == 32 ) {
        if ( !isp ) {
          isp = i++;
        }
        if ( i < wrap_length ) {
          isp = i++;
        }
        else {
          bPrintLine = true;
          if ( isp ) {
            i1 = i = isp;
            while ( s[i] == 32 )
              i++;
          }
          else {
            i++;
          }
        }
      }
      else {
        i++;
      }
      if ( bPrintLine ) {
        if ( i1 >= max_line_length )
          i1 = max_line_length-1;
        if ( i1 > 0 ) {
          wsncpy( sLine, s, i1 );
          sLine[i1] = 0;
          Print( "%ls\n", sLine );
        }
        else {
          Print("\n");
        }

        s += i;
        i = i1 = isp = 0;
        bPrintLine = false;
      }
    }
    if ( s[0] ) {
      Print( "%ls", s );
    }
  }
}


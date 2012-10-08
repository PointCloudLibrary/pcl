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

// wide char (unicode) <-> char (ascii) converter
static int w2c_size( int, const wchar_t* ); // gets minimum "c_count" arg for w2c().
static int w2c( int,            // w_count = number of wide chars to convert
                const wchar_t*, // source wide char string
                int,            // c_count, 
                char*           // array of at least c_count+1 characters
                );
static int c2w( int,           // c_count = number of chars to convert
                const char*,   // source byte char string
                int,           // w_count, 
                wchar_t*       // array of at least c_count+1 wide characters
                );
static wchar_t c2w( char );

static int w2c_size( int w_count, const wchar_t* w )
{
  // returns number of bytes used in wide conversion.  Does not
  // include NULL terminator.
  int rc = 0;
  if ( w ) {
    unsigned int error_status = 0;
    rc = ON_ConvertWideCharToUTF8(false,w,w_count,0,0,&error_status,0,0,0);
    if ( error_status )
    {
      ON_ERROR("Wide char string is not valid.");
    }
    if ( rc < 0 )
      rc = 0;
  }
  return rc;
}

static int w2c( int w_count, 
                const wchar_t* w, 
                int c_count, 
                char* c // array of at least c_count+1 characters
                )
{
  // convert UTF-16 string to UTF-8 string
  int rc = 0;
  if ( c ) 
    c[0] = 0;
  // returns length of converted c[]
  if ( c_count > 0 && c )
  {
    c[0] = 0;
    if ( w ) 
    {
      unsigned int error_status = 0;
      unsigned int error_mask = 0xFFFFFFFF;
      ON__UINT32 error_code_point = 0xFFFD;
      const wchar_t* p1 = 0;
      rc = ON_ConvertWideCharToUTF8(false,w,w_count,c, c_count, &error_status,error_mask,error_code_point,&p1);
      if ( error_status )
      {
        ON_ERROR("Error converting UTF-16 encoded wchar_t string to UTF-8 encoded char string.");
      }
      if ( rc > 0 && rc <= c_count )
        c[rc] = 0;
      else 
      {
        c[c_count] = 0;
        rc = 0;
      }
    }
  }
	return rc;
}

static wchar_t c2w( char c )
{
  // NOTE: 
  //   Single character conversion of char values 0x80 to 0xFF 
  //   get mapped to unicode code points U+0080 U+00FF
  //   In particular, this is NOT UTF-8 conversion. 
  wchar_t w = ((unsigned char)c);
  return w;
}

static int c2w( int c_count, 
                const char* c, 
                int w_count, 
                wchar_t* w // array of at least w_count+1 wide characters
                )
{
  // convert UTF-8 string to UTF-16 string
  int rc = 0;
  if ( w ) 
    w[0] = 0;
  // returns length of converted c[]
  if ( w_count > 0 && w && c_count > 0 && c && c[0] ) {
    w[0] = 0;
    if ( c ) 
    {
      unsigned int error_status = 0;
      unsigned int error_mask = 0xFFFFFFFF;
      ON__UINT32 error_code_point = 0xFFFD;
      const char* p1 = 0;
      rc = ON_ConvertUTF8ToWideChar(c,c_count,w,w_count,&error_status,error_mask,error_code_point,&p1);
      if ( rc > 0 && rc <= w_count )
        w[rc] = 0;
      else {
        w[w_count] = 0;
        rc = 0;
      }
      if ( 0 != error_status )
      {
        ON_ERROR("Error converting UTF-8 encoded char string to UTF-16 encoded wchar_t string.");
      }
    }
  }
	return rc;
}


void ON_String::CopyToArray( int w_count, const wchar_t* w )
{
  // if sizeof(wchar_t) is 2, this converts a UTF-16 string to UTF-8 string
  // if sizeof(wchar_t) is 4, this converts a UTF-32 string to UTF-8 string
  int c_count = w2c_size( w_count, w );
  char* c = (char*)onmalloc(c_count+1);
  memset( c, 0, c_count+1 );
  const int c_length = w2c( w_count, w, c_count, c );
  c[c_length] = 0;
  CopyToArray( c_count, c );
  onfree(c);
}



/////////////////////////////////////////////////////////////////////////////
// Empty strings point at empty_wstring

struct ON_wStringHeader
{
	int    ref_count;       // reference count (>=0 or -1 for empty string)
	int    string_length;   // does not include any terminators
	int    string_capacity; // does not include any terminators
	wchar_t* string_array() {return (wchar_t*)(this+1);}
};

static struct {
  ON_wStringHeader header;
  wchar_t           s;  
} empty_wstring = { {-1, 0, 0}, 0 }; // ref_count=-1, length=0, capacity=0, s=0 
static ON_wStringHeader* pEmptyStringHeader = &empty_wstring.header;
static const wchar_t* pEmptywString = &empty_wstring.s;

//////////////////////////////////////////////////////////////////////////////
// protected helpers

void ON_wString::Create()
{
  m_s = (wchar_t*)pEmptywString;
}

ON_wStringHeader* ON_wString::Header() const
{
  ON_wStringHeader* p = (ON_wStringHeader*)m_s;
  if (p)
    p--;
  else
    p = pEmptyStringHeader;
  return p;
}

void ON_wString::CreateArray( int capacity )
{
  Destroy();
  if ( capacity > 0 ) {
		ON_wStringHeader* p =
			(ON_wStringHeader*)onmalloc( sizeof(ON_wStringHeader) + (capacity+1)*sizeof(*m_s) );
		p->ref_count = 1;
		p->string_length = 0;
		p->string_capacity = capacity;
		m_s = p->string_array();
    memset( m_s, 0, (capacity+1)*sizeof(*m_s) );
  }
}

void ON_wString::Destroy()
{
  ON_wStringHeader* p = Header();
  if ( p != pEmptyStringHeader && p->ref_count > 0 ) {
    p->ref_count--;
		if ( p->ref_count == 0 )
			onfree(p);
  }
	Create();
}

void ON_wString::Empty()
{
  ON_wStringHeader* p = Header();
  if ( p != pEmptyStringHeader ) {
    if ( p->ref_count > 1 ) {
      // string memory is shared
      p->ref_count--;
	    Create();
    }
    else if ( p->ref_count == 1 ) {
      // string memory is not shared - reuse it
      if (m_s && p->string_capacity>0)
        *m_s = 0;
      p->string_length = 0;
    }
    else {
      // should not happen
      ON_ERROR("ON_wString::Empty() encountered invalid header - fixed.");
      Create();
    }
  }
  else {
    // initialized again
	  Create();
  }
}

void ON_wString::EmergencyDestroy()
{
	Create();
}

void ON_wString::EnableReferenceCounting( bool bEnable )
{
  // TODO fill this in;
}

bool ON_wString::IsReferenceCounted() const
{
  return true;
}


void ON_wString::CopyArray()
{
  // If 2 or more string are using array, it is duplicated.
  // Call CopyArray() before modifying array contents.
  ON_wStringHeader* p = Header();
  if ( p != pEmptyStringHeader && p && p->ref_count > 1 ) 
  {
    const wchar_t* s = m_s;
    // p and s remain valid after Destroy() because
    // will simply be decremented and no deallocation
    // will happen.
    Destroy();
    Create();
    CopyToArray( p->string_capacity, s );
    if ( p->string_length < p->string_capacity )
    {
      Header()->string_length = p->string_length;
    }
  }
}

void ON_wString::ReserveArray( size_t array_capacity )
{
  ON_wStringHeader* p = Header();
  const int capacity = (int)array_capacity; // for 64 bit compiler
  if ( p == pEmptyStringHeader ) 
  {
		CreateArray(capacity);
  }
  else if ( p->ref_count > 1 ) 
  {
		CreateArray(capacity);
    ON_wStringHeader* p1 = Header();
    const int size = (capacity < p->string_length) ? capacity : p->string_length;
    if ( size > 0 ) 
    {
      memcpy( p1->string_array(), p->string_array(), size*sizeof(*m_s) );
      p1->string_length = size;
    }
  }
	else if ( capacity > p->string_capacity ) 
  {
		p = (ON_wStringHeader*)onrealloc( p, sizeof(ON_wStringHeader) + (capacity+1)*sizeof(*m_s) );
    m_s = p->string_array();
    memset( &m_s[p->string_capacity], 0, (1+capacity-p->string_capacity)*sizeof(*m_s) );
    p->string_capacity = capacity;
	}
}

void ON_wString::ShrinkArray()
{
  ON_wStringHeader* p = Header();
  if ( p != pEmptyStringHeader ) {
    if ( p->string_length < 1 ) {
      Destroy();
    }
    else if ( p->ref_count > 1 ) {
      // shared string
      CreateArray(p->string_length);
		  ON_wStringHeader* p1 = Header();
      memcpy( m_s, p->string_array(), p->string_length*sizeof(*m_s));
      p1->string_length = p->string_length;
      m_s[p1->string_length] = 0;
    }
	  else if ( p->string_length < p->string_capacity ) {
      // onrealloc string
		  p = (ON_wStringHeader*)onrealloc( p, sizeof(ON_wStringHeader) + (p->string_length+1)*sizeof(*m_s) );
      p->string_capacity = p->string_length;
      m_s = p->string_array();
      m_s[p->string_length] = 0;
	  }
  }
}

void ON_wString::CopyToArray( const ON_wString& s )
{
  CopyToArray( s.Length(), s.Array() );
}

void ON_wString::CopyToArray( int size, const char* s )
{
  if ( size > 0 && s && s[0] ) {
	  ReserveArray(size);
    Header()->string_length = c2w( size, s, Header()->string_capacity, m_s );
    m_s[Header()->string_length] = 0;
  }
  else {
    if ( Header()->ref_count > 1 )
      Destroy();
    else {
      Header()->string_length = 0;
      m_s[0] = 0;
    }
  }
}

void ON_wString::CopyToArray( int size, const unsigned char* s )
{
  CopyToArray( size, ((char*)s) );
}

void ON_wString::CopyToArray( int size, const wchar_t* s )
{
  if ( size > 0 && s && s[0] ) {
	  ReserveArray(size);
	  memcpy(m_s, s, size*sizeof(*m_s));
	  Header()->string_length = size;
    m_s[Header()->string_length] = 0;
  }
  else {
    if ( Header()->ref_count != 1 )
      Destroy();
    else {
      Header()->string_length = 0;
      m_s[0] = 0;
    }
  }
}

void ON_wString::AppendToArray( const ON_wString& s )
{
  AppendToArray( s.Length(), s.Array() );
}

void ON_wString::AppendToArray( int size, const char* s )
{
  if ( size > 0 && s && s[0] ) {
	  ReserveArray(size + Header()->string_length );
    Header()->string_length += c2w( size, s, Header()->string_capacity-Header()->string_length, &m_s[Header()->string_length] );
    m_s[Header()->string_length] = 0;
  }
}

void ON_wString::AppendToArray( int size, const unsigned char* s )
{
  AppendToArray( size, ((char*)s) );
}

void ON_wString::AppendToArray( int size, const wchar_t* s )
{
  if ( size > 0 && s && s[0] ) {
	  ReserveArray(size + Header()->string_length );
	  memcpy(&m_s[Header()->string_length], s, size*sizeof(*m_s));
	  Header()->string_length += size;
    m_s[Header()->string_length] = 0;
  }
}


int ON_wString::Length( const char* s )
{
  size_t slen = s ? strlen(s) : 0;
  int n = ((0 < slen && slen <= 2147483645) ?((int)slen) : 0); // the (int) cast is for 64 bit size_t conversion
  return n;
}

int ON_wString::Length( const unsigned char* s )
{
  return ON_wString::Length((const char*)s);
}

int ON_wString::Length( const wchar_t* s )
{
  size_t slen =  s ? wcslen(s) : 0;
  int n = ((0 < slen && slen <= 2147483645) ?((int)slen) : 0); // the (int) cast is for 64 bit size_t conversion
  return n;
}

//////////////////////////////////////////////////////////////////////////////
// Construction/Destruction

ON_wString::ON_wString()
{
	Create();
}

ON_wString::~ON_wString()
{
  Destroy();
}

ON_wString::ON_wString(const ON_wString& src)
{
	if (    src.Header()->ref_count > 0 
       && 0 == ON_WorkerMemoryPool()
       )	
  {
		m_s = src.m_s;
    src.Header()->ref_count++;
	}
	else 
  {
		Create();
		*this = src.m_s; // use operator=(const wchar_t*) to copy
	}
}

ON_wString::ON_wString(const ON_String& src)
{
	Create();
	*this = src;
}

ON_wString::ON_wString( const char* s )
{
	Create();
  if ( s && s[0] ) 
  {
    CopyToArray( (int)strlen(s), s ); // the (int) is for 64 bit size_t conversion
  }
}

ON_wString::ON_wString( const char* s, int length )
{
	Create();
  if ( s && length > 0 ) {
    CopyToArray(length,s);
	}
}

ON_wString::ON_wString( char c, int repeat_count )
{
  Create();
  if ( repeat_count > 0 ) {
    char* s = (char*)onmalloc((repeat_count+1)*sizeof(*s));
    s[repeat_count] = 0;
    memset( s, c, repeat_count*sizeof(*s) );
    CopyToArray( repeat_count, s );
    onfree(s);
    m_s[repeat_count] = 0;
    Header()->string_length = repeat_count;
  }
}

ON_wString::ON_wString( const unsigned char* s )
{
	Create();
  if ( s && s[0] ) {
    CopyToArray( (int)strlen((const char*)s), (const char*)s ); // the (int) is for 64 bit size_t conversion
  }
}

ON_wString::ON_wString( const unsigned char* s, int length )
{
	Create();
  if ( s && length > 0 ) {
    CopyToArray(length,s);
	}
}

ON_wString::ON_wString( unsigned char c, int repeat_count )
{
  Create();
  if ( repeat_count > 0 ) {
    char* s = (char*)onmalloc((repeat_count+1)*sizeof(*s));
    s[repeat_count] = 0;
    memset( s, c, repeat_count*sizeof(*s) );
    CopyToArray( repeat_count, s );
    onfree(s);
    m_s[repeat_count] = 0;
    Header()->string_length = repeat_count;
  }
}


ON_wString::ON_wString( const wchar_t* s )
{
	Create();
  if ( s && s[0] ) {
    CopyToArray( (int)wcslen(s), s ); // the (int) is for 64 bit size_t conversion
  }
}

ON_wString::ON_wString( const wchar_t* s, int length )
{
	Create();
  if ( s && length > 0 ) {
    CopyToArray( length, s );
	}
}

ON_wString::ON_wString( wchar_t c, int repeat_count )
{
  int i;
  Create();
  if ( repeat_count > 0 ) {
    ReserveArray(repeat_count);
    for (i=0;i<repeat_count;i++)
      m_s[i] = c;
    m_s[repeat_count] = 0;
    Header()->string_length = repeat_count;
  }
}

#if defined(ON_OS_WINDOWS)
bool ON_wString::LoadResourceString(HINSTANCE instance, UINT id )
{
  bool rc = false;
  wchar_t s[2048]; // room for 2047 characters
  int length;

  Destroy();
  length = ::LoadStringW( instance, id, s, 2047 );
  if ( length > 0 && length < 2048 ) {
    CopyToArray( length, s );
    rc = true;
  }
  return rc;
}
#endif

int ON_wString::Length() const
{
  return Header()->string_length;
}

wchar_t& ON_wString::operator[](int i)
{
  CopyArray();
  return m_s[i];
}

wchar_t ON_wString::operator[](int i) const
{
  return m_s[i];
}

bool ON_wString::IsEmpty() const
{
  return (Header()->string_length <= 0 ) ? true : false;
}

const ON_wString& ON_wString::operator=(const ON_wString& src)
{
	if (m_s != src.m_s)	
  {
    if ( src.IsEmpty() ) 
    {
      Destroy();
      Create();
    }
    else if (    src.Header()->ref_count > 0 
              && 0 == ON_WorkerMemoryPool()
            ) 
    {
      Destroy();
      src.Header()->ref_count++;
      m_s = src.m_s;
    }
    else 
    {
      ReserveArray(src.Length());
      memcpy( m_s, src.Array(), src.Length()*sizeof(*m_s));
      Header()->string_length = src.Length();
    }
  }
	return *this;
}

const ON_wString& ON_wString::operator=(const ON_String& src)
{
  *this = src.Array();
  return *this;
}

const ON_wString& ON_wString::operator=( char c )
{
	CopyToArray( 1, &c );
	return *this;
}

const ON_wString& ON_wString::operator=( const char* s )
{
  if ( (void*)s != (void*)m_s )
	  CopyToArray( Length(s), s);
	return *this;
}

const ON_wString& ON_wString::operator=( unsigned char c )
{
	CopyToArray( 1, &c );
	return *this;
}

const ON_wString& ON_wString::operator=( const unsigned char* s )
{
  if ( (void*)s != (void*)m_s )
	  CopyToArray( Length(s), s);
	return *this;
}

const ON_wString& ON_wString::operator=( wchar_t c )
{
	CopyToArray( 1, &c );
	return *this;
}

const ON_wString& ON_wString::operator=( const wchar_t* s )
{
  if ( (void*)s != (void*)m_s )
	  CopyToArray( Length(s), s);
	return *this;
}

ON_wString ON_wString::operator+(const ON_wString& s2) const
{
	ON_wString s(*this);
  s.AppendToArray( s2 );
	return s;
}

ON_wString ON_wString::operator+(const ON_String& s2) const
{
	ON_wString s(*this);
  s.AppendToArray( s2.Length(), s2.Array() );
	return s;
}

ON_wString ON_wString::operator+(char s2 ) const
{
	ON_wString s(*this);
  s.AppendToArray( 1, &s2 );
	return s;
}

ON_wString ON_wString::operator+(unsigned char s2 ) const
{
	ON_wString s(*this);
  s.AppendToArray( 1, &s2 );
	return s;
}

ON_wString ON_wString::operator+( wchar_t s2 ) const
{
	ON_wString s(*this);
  s.AppendToArray( 1, &s2 );
	return s;
}

ON_wString ON_wString::operator+(const char* s2) const
{
	ON_wString s(*this);
  s.AppendToArray( ON_wString::Length(s2), s2 );
	return s;
}

ON_wString ON_wString::operator+(const unsigned char* s2) const
{
	ON_wString s(*this);
  s.AppendToArray( ON_wString::Length(s2), s2 );
	return s;
}

ON_wString ON_wString::operator+(const wchar_t* s2) const
{
	ON_wString s(*this);
  s.AppendToArray( ON_wString::Length(s2), s2 );
	return s;
}

//////////////////////////////////////////////////////////////////////////////
// operator+=()

void ON_wString::Append( const char* s , int count )
{
  // append specified number of characters
  if ( s && count > 0 )
    AppendToArray(count,s);
}

void ON_wString::Append( const unsigned char* s , int count )
{
  // append specified number of characters
  if ( s && count > 0 )
    AppendToArray(count,s);
}


void ON_wString::Append( const wchar_t* s, int count )
{
  // append specified number of characters
  if ( s && count > 0 )
    AppendToArray(count,s);
}

const ON_wString& ON_wString::operator+=(const ON_wString& s)
{
  AppendToArray(s);
	return *this;
}

const ON_wString& ON_wString::operator+=(const ON_String& s)
{
  AppendToArray( s.Length(), s.Array() );
	return *this;
}

const ON_wString& ON_wString::operator+=( char s )
{
  AppendToArray(1,&s);
	return *this;
}

const ON_wString& ON_wString::operator+=( unsigned char s )
{
  AppendToArray(1,&s);
	return *this;
}

const ON_wString& ON_wString::operator+=( wchar_t s )
{
  AppendToArray(1,&s);
	return *this;
}

const ON_wString& ON_wString::operator+=( const char* s )
{
  AppendToArray(Length(s),s);
	return *this;
}

const ON_wString& ON_wString::operator+=( const unsigned char* s )
{
  AppendToArray(Length(s),s);
	return *this;
}

const ON_wString& ON_wString::operator+=( const wchar_t* s )
{
  AppendToArray(Length(s),s);
	return *this;
}

void ON_wString::SetLength(size_t string_length)
{
  int length = (int)string_length; // for 64 bit compilers
  if ( length >= Header()->string_capacity ) {
    ReserveArray(length);
  }
  if ( length >= 0 && length <= Header()->string_capacity ) {
    CopyArray();
    Header()->string_length = length;
    m_s[length] = 0;
  }
}

wchar_t* ON_wString::Array()
{
  CopyArray();
  return ( Header()->string_capacity > 0 ) ? m_s : 0;
}

const wchar_t* ON_wString::Array() const
{
  return ( Header()->string_capacity > 0 ) ? m_s : 0;
}

/*
Returns:
  Total number of bytes of memory used by this class.
  (For use in ON_Object::SizeOf() overrides.
*/
unsigned int ON_wString::SizeOf() const
{
  size_t sz = sizeof(*this);
  if ( ((const void*)m_s) != ((const void*)pEmptywString) )
    sz += (sizeof(ON_wStringHeader) + sizeof(wchar_t)*(Header()->string_capacity+1));
  return ((unsigned int)sz);
}

ON__UINT32 ON_wString::DataCRC(ON__UINT32 current_remainder) const
{
  int string_length = Header()->string_length;
  if ( string_length > 0 )
  {
    current_remainder = ON_CRC32(current_remainder,string_length*sizeof(*m_s),m_s);
  }
  return current_remainder;
}

ON__UINT32 ON_wString::DataCRCLower(ON__UINT32 current_remainder) const
{
  int string_length = Header()->string_length;
  if ( string_length > 0 )
  {
    ON_wString s(*this);
    s.MakeLower();
    current_remainder = s.DataCRC(current_remainder);
  }
  return current_remainder;
}


int ON_wString::Compare( const char* s ) const
{
  int rc = 0;
  if ( s && s[0] ) {
    if ( IsEmpty() ) {
      rc = -1;
    }
    else {
      int c_count = w2c_size( Length(m_s), m_s );
      char* c = (char*)onmalloc((c_count+1)*sizeof(*c));
      w2c( Length(m_s), m_s, c_count, c );
      c[c_count] = 0;
      rc = strcmp( c, s );
      onfree(c);
    }
  }
  else {
    rc = IsEmpty() ? 0 : 1;
  }
  return rc;
}

int ON_wString::Compare( const unsigned char* s) const
{
  return ON_wString::Compare((const char*)s);
}

int ON_wString::Compare( const wchar_t* s ) const
{
  int rc = 0;
  if ( s && s[0] ) {
    if ( IsEmpty() ) {
      rc = -1;
    }
    else {
      rc = wcscmp( m_s, s );
    }
  }
  else {
    rc = IsEmpty() ? 0 : 1;
  }
  return rc;
}

int ON_wString::CompareNoCase( const char* s) const
{
  int rc = 0;
  if ( s && s[0] ) {
    if ( IsEmpty() ) {
      rc = -1;
    }
    else {
      int c_count = w2c_size( Length(m_s), m_s );
      char* c = (char*)onmalloc((c_count+1)*sizeof(*c));
      w2c( Length(m_s), m_s, c_count, c );
      c[c_count] = 0;
      rc = on_stricmp( c, s );
      onfree(c);
    }
  }
  else {
    rc = IsEmpty() ? 0 : 1;
  }
  return rc;
}

int ON_wString::CompareNoCase( const unsigned char* s) const
{
  return ON_wString::CompareNoCase((const char*)s);
}

int ON_wString::CompareNoCase( const wchar_t* s) const
{
  int rc = 0;
  if ( s && s[0] ) {
    if ( IsEmpty() ) {
      rc = -1;
    }
    else {
      rc = on_wcsicmp( m_s, s );
    }
  }
  else {
    rc = IsEmpty() ? 0 : 1;
  }
  return rc;
}


bool ON_WildCardMatch(const wchar_t* s, const wchar_t* pattern)
{
  if ( !pattern || !pattern[0] ) {
    return ( !s || !s[0] ) ? true : false;
  }

  if ( *pattern == '*' ) {
    pattern++;
    while ( *pattern == '*' )
      pattern++;
    
    if ( !pattern[0] )
      return true;

    while (*s) {
      if ( ON_WildCardMatch(s,pattern) )
        return true;
      s++;
    }

    return false;
  }

  while ( *pattern != '*' )
  {
    if ( *pattern == '?' ) {
      if ( *s) {
        pattern++;
        s++;
        continue;
      }
      return false;
    }
    
    if ( *pattern == '\\' ) {
      switch( pattern[1] )
      {
      case '*':
      case '?':
        pattern++;
        break;
      }
    }
    if ( *pattern != *s ) {
      return false;
    }

    if ( *s == 0 )
      return true;

    pattern++;
    s++;
  }
  
  return ON_WildCardMatch(s,pattern);
}


bool ON_WildCardMatchNoCase(const wchar_t* s, const wchar_t* pattern)
{
  if ( !pattern || !pattern[0] ) {
    return ( !s || !s[0] ) ? true : false;
  }

  if ( *pattern == '*' ) 
  {
    pattern++;
    while ( *pattern == '*' )
      pattern++;
    
    if ( !pattern[0] )
      return true;

    while (*s) {
      if ( ON_WildCardMatchNoCase(s,pattern) )
        return true;
      s++;
    }

    return false;
  }

  while ( *pattern != '*' )
  {
    if ( *pattern == '?' )
    {
      if ( *s) {
        pattern++;
        s++;
        continue;
      }
      return false;
    }
    
    if ( *pattern == '\\' )
    {
      switch( pattern[1] )
      {
      case '*':
      case '?':
        pattern++;
        break;
      }
    }
    if ( towupper(*pattern) != towupper(*s) )
    {
      return false;
    }

    if ( *s == 0 )
      return true;

    pattern++;
    s++;
  }
  
  return ON_WildCardMatchNoCase(s,pattern);
}

bool ON_wString::WildCardMatch( const wchar_t* pattern ) const
{
  return ON_WildCardMatch(m_s,pattern);
}


bool ON_wString::WildCardMatchNoCase( const wchar_t* pattern ) const
{
  return ON_WildCardMatchNoCase(m_s,pattern);
}

/*
static TestReplace( ON_TextLog* text_log )
{
  int len, len1, len2, i, count, gap, k, i0, repcount, replen;
  ON_wString str;

  bool bRepeat = false;

  wchar_t ws[1024], wsToken1[1024], wsToken2[1024];

  memset(ws,     0,sizeof(ws));
  memset(wsToken1,0,sizeof(wsToken1));
  memset(wsToken2,0,sizeof(wsToken2));

	for ( len = 1; len < 32; len++ )
  {
    for ( len1 = 1; len1 < len+1; len1++ )
    {
      if ( len1 > 0 )
        wsToken1[0] = '<';
      for ( i = 1; i < len1-1; i++ )
        wsToken1[i] = '-';
      if ( len1 > 1 )
        wsToken1[len1-1] = '>';
      wsToken1[len1] = 0;

      for ( len2 = 1; len2 < len1+5; len2++ )
      {
        if ( len2 > 0 )
          wsToken2[0] = '+';
        for ( i = 1; i < len2-1; i++ )
          wsToken2[i] = '=';
        if ( len2 > 1 )
          wsToken2[len2-1] = '*';
        wsToken2[len2] = 0;

        for ( k = 1; k*len1 <= len+1; k++ )
        {
          gap = (len/k) - len1;
          if (0 == len1 && gap < 1 )
            gap = 1;
          else if ( gap < 0 )
            gap = 0;
          bRepeat = false;
          for ( i0 = 0; i0 < 2*len1 + gap; i0++ )
          {
            for ( i = 0; i < len; i++ )
            {
              ws[i] = (wchar_t)('a' + (i%26));
            }
            ws[len] = 0;
            count = 0;
            for ( i = i0; i+len1 <= len; i += (gap+len1) )
            {
              memcpy(&ws[i],wsToken1,len1*sizeof(ws[0]));
              count++;
            }
            str = ws;
            repcount = str.Replace(wsToken1,wsToken2);
            replen = str.Length();
            if ( repcount != count || replen != len + count*(len2-len1) )
            {
              if ( text_log )
              {
                text_log->Print("%ls -> %ls failed\n",wsToken1,wsToken2);
                text_log->Print("%ls (%d tokens, %d chars)\n",ws,count,len);
                text_log->Print("%ls (%d tokens, %d chars)\n",str.Array(),repcount,replen);
              }
              if ( bRepeat )
              {
                bRepeat = false;
              }
              else
              {
                bRepeat = true;
                i0--;
              }
            }
          }
          bRepeat = false;
        }
      }
    }
  }
}
*/

int ON_wString::Replace( const wchar_t* token1, const wchar_t* token2 )
{
  int count = 0;

  if ( 0 != token1 && 0 != token1[0] )
  {
    if ( 0 == token2 )
      token2 = L"";
    const int len1 = (int)wcslen(token1);
    if ( len1 > 0 )
    {
      const int len2 = (int)wcslen(token2);
      int len = Length();
      if ( len >= len1 )
      {
        // in-place
        ON_SimpleArray<int> n(32);
        const wchar_t* s = m_s;
        int i;
        for ( i = 0; i <= len-len1; /*empty*/ )
        {
          if ( wcsncmp(s,token1,len1) )
          {
            s++;
            i++;
          }
          else
          {
            n.Append(i);
            i += len1;
            s += len1;
          }
        }

        count = n.Count();

        // reserve array space - must be done even when len2 <= len1
        // so that shared arrays are not corrupted.
        const int newlen = len + (count*(len2-len1));
        if ( 0 == newlen )
        {
          Destroy();
          return count;
        }

        CopyArray();

        // 24 August 2006 Dale Lear
        //    This used to say
        //       ReserveArray(newlen);
        //    but when newlen < len and the string had multiple
        //    references, the ReserveArray(newlen) call truncated
        //    the input array.  
        ReserveArray( (newlen<len) ? len : newlen );

        int i0, i1, ni, j;

        if ( len2 > len1 )
        {
          // copy from back to front
          i1 = newlen;
          i0 = len;
          for ( ni =0; ni < count; ni++ )
            n[ni] = n[ni] + len1;
          for ( ni = count-1; ni >= 0; ni-- )
          {
            j = n[ni];
            while ( i0 > j )
            {
              i0--;
              i1--;
              m_s[i1] = m_s[i0];
            }
            i1 -= len2;
            i0 -= len1;
            memcpy(&m_s[i1],token2,len2*sizeof(m_s[0]));
          }
        }
        else
        {
          // copy from front to back
          i0 = i1 = n[0];
          n.Append(len);
          for ( ni = 0; ni < count; ni++ )
          {
            if ( len2 > 0 )
            {
              memcpy(&m_s[i1],token2,len2*sizeof(m_s[0]));
              i1 += len2;
            }
            i0 += len1;
            j = n[ni+1];
            while ( i0 < j )
            {
              m_s[i1++] = m_s[i0++];
            }
          }
        }
        Header()->string_length = newlen;
        m_s[newlen] = 0;
      }
    }
  }

  return count;
}

int ON_wString::Replace( wchar_t token1, wchar_t token2 )
{
  int count = 0;
  int i = Length();
  while (i--)
  {
    if ( token1 == m_s[i] )
    {
      if ( 0 == count )
        CopyArray();
      m_s[i] = token2;
      count++;
    }
  }
  return count;
}


void ON_wString::UrlEncode()
{
  wchar_t c, c0, c1;
  wchar_t* buffer = 0;
  wchar_t* s1 = 0;
  const wchar_t* s = Array();
  const int count = Length();
  int i;

  for ( i = 0; i < count; i++ )
  {
    c = *s++;
    if ( 0 == c )
      break;
    if ('0' <= c && c <= '9')
    {
      if ( s1 )
        *s1++ = c;
      continue;
    }
    if ('a' <= c && c <= 'z')
    {
      if ( s1 )
        *s1++ = c;
      continue;
    }
    if ('A' <= c && c <= 'Z')
    {
      if ( s1 )
        *s1++ = c;
      continue;
    }
    if (c >= 256)
    {
      if ( s1 )
        *s1++ = c;
      continue;
    }

    // convert this character to %xx
    if ( !s1 )
    {
      buffer = (wchar_t*)onmalloc((count*3 + 1)*sizeof(buffer[0]));
      if ( i > 0 )
        memcpy(buffer,Array(),i*sizeof(buffer[0]));
      s1 = buffer+i;
    }
    c0 = ((c/16)%16) + '0';
    if ( c0 > '9' )
      c0 += ('A'-'9'-1);
    c1 = (c%16) + '0';
    if ( c1 > '9' )
      c1 += ('A'-'9'-1);
    *s1++ = '%';
    *s1++ = c0;
    *s1++ = c1;
  }
  if ( s1 )
  {
    *s1 = 0;
    *this = buffer;
    onfree(buffer);
  }
}

static bool UrlDecodeHelper( wchar_t* s)
{
  // if s[0] and s[1] are hex digits, then s[1] is
  // set to the wchar_t with that hex value.
  if ( !s )
    return false;

  wchar_t c0 = *s++;
  if ( c0 >= '0' && c0 <= '9' )
    c0 -= '0';
  else if ( c0 >= 'A' && c0 <= 'F' )
    c0 -= 'A' - 0x0A;
  else if ( c0 >= 'a' && c0 <= 'f' )
    c0 -= 'a' - 0x0A;
  else
    return false;

  wchar_t c1 = *s;
  if ( c1 >= '0' && c1 <= '9' )
    c1 -= '0';
  else if ( c1 >= 'A' && c1 <= 'F' )
    c1 -= 'A' - 0x0A;
  else if ( c1 >= 'a' && c1 <= 'f' )
    c1 -= 'a' - 0x0A;
  else
    return false;

  *s = c0*0x10 + c1;
  return true;
}

static bool IsValidUrlChar(wchar_t c)
{
  if ( c >= '0' && c <= '9' )
    return true;
  if ( c >= 'A' && c <= 'Z' )
    return true;
  if ( c >= 'A' && c <= 'z' )
    return true;

  // ON_wString::UrlEncode() encodes assumes the following
  // characters are literal and encodes them.  However,
  // it is permitted for these characters to appear in
  // a URL.
  switch(c)
  {
  case '$':
  case '-':
  case '_':
  case '.':
  case '+':
  case '!':
  case '*':
  case '\'':
  case '(':
  case ')':
    // RFC 1738 character
    return true;
  case '&':
  case ',':
  case '/':
  case ':':
  case ';':
  case '=':
  case '?':
  case '@':
    // permitted URL syntax character
    return true;
  case '#':
    //  URL bookmark character
    return true;
  }

  return false;
}

bool ON_wString::UrlDecode()
{
  CopyArray();

  bool rc = true;
  wchar_t c;
  wchar_t* s0 = Array();
  if ( !s0 )
    return true;
  wchar_t* s1 = s0;
  //const wchar_t* debg = s1;
  int i;
  for (i = Length(); i > 0; i-- )
  {
    c = *s0++;
    if (0==c)
      break;
    if (i >= 3 && '%' == c && UrlDecodeHelper(s0) )
    {
      s0++;
      *s1++ = *s0++;
      i -= 2;
    }
    else
    {
      *s1++ = c;
      if (rc)
        rc = IsValidUrlChar(c);
    }
  }
  *s1 = 0;
  SetLength(s1 - Array());
  return rc;
}



static bool IsWhiteSpaceHelper( wchar_t c, const wchar_t* whitespace )
{
  while ( *whitespace )
  {
    if ( c == *whitespace++ )
      return true;
  }
  return false;
}

int ON_wString::ReplaceWhiteSpace( wchar_t token, const wchar_t* whitespace )
{
  wchar_t* s0;
  wchar_t* s1;
  int n;
  wchar_t c;

  if ( 0 == (s0 = m_s) )
    return 0;
  s1 = s0 + Length();
  if ( whitespace && *whitespace )
  {
    while( s0 < s1 )
    {
      if (IsWhiteSpaceHelper(*s0++,whitespace))
      {
        // need to modify this string
        n = ((int)(s0 - m_s)); // keep win64 happy with (int) cast
        CopyArray(); // may change m_s if string has multiple refs
        s0 = m_s + n;
        s1 = m_s + Length();
        s0[-1] = token;
        n = 1;
        while ( s0 < s1 )
        {
          if ( IsWhiteSpaceHelper(*s0++,whitespace) )
          {
            s0[-1] = token;
            n++;
          }
        }
        return n;
      }
    }
  }
  else
  {
    while( s0 < s1 )
    {
      c = *s0++;
      if ( (1 <= c && c <= 32) || 127 == c )
      {
        // need to modify this string
        n = ((int)(s0 - m_s)); // keep win64 happy with (int) cast
        CopyArray(); // may change m_s if string has multiple refs
        s0 = m_s + n;
        s1 = m_s + Length();
        s0[-1] = token;
        n = 1;
        while ( s0 < s1 )
        {
          c = *s0++;
          if ( (1 <= c && c <= 32) || 127 == c )
          {
            s0[-1] = token;
            n++;
          }
        }
        return n;
      }
    }
  }
  return 0;
}

int ON_wString::RemoveWhiteSpace( const wchar_t* whitespace )
{
  wchar_t* s0;
  wchar_t* s1;
  wchar_t* s;
  int n;
  wchar_t c;

  if ( 0 == (s0 = m_s) )
    return 0;
  s1 = s0 + Length();
  if ( whitespace && *whitespace )
  {
    while( s0 < s1 )
    {
      if (IsWhiteSpaceHelper(*s0++,whitespace))
      {
        // need to modify this string
        n = ((int)(s0 - m_s)); // keep win64 happy with (int) cast
        CopyArray(); // may change m_s if string has multiple refs
        s0 = m_s + n;
        s = s0-1;
        s1 = m_s + Length();
        while ( s0 < s1 )
        {
          if ( !IsWhiteSpaceHelper(*s0,whitespace) )
          {
            *s++ = *s0;
          }
          s0++;
        }
        *s = 0;
        n = ((int)(s1 - s)); // keep win64 happy with (int) cast
        Header()->string_length -= n;
        return n;
      }
    }
  }
  else
  {
    while( s0 < s1 )
    {
      c = *s0++;
      if ( (1 <= c && c <= 32) || 127 == c )
      {
        // need to modify this string
        n = ((int)(s0 - m_s)); // keep win64 happy with (int) cast
        CopyArray(); // may change m_s if string has multiple refs
        s0 = m_s + n;
        s = s0-1;
        s1 = m_s + Length();
        while ( s0 < s1 )
        {
          c = *s0;
          if ( c < 1 || (c > 32 && 127 != c) )
          {
            *s++ = *s0;
          }
          s0++;
        }
        *s = 0;
        n = ((int)(s1 - s)); // keep win64 happy with (int) cast
        Header()->string_length -= n;
        return n;
      }
    }
  }
  return 0;
}


///////////////////////////////////////////////////////////////////////////////

ON_wString::operator const wchar_t*() const
{
  return ( m_s == pEmptywString ) ? NULL : m_s;
}

int ON_wString::Find( char c ) const
{
	// find first single character
  char s[2];
  s[0] = c;
  s[1] = 0;
  return Find( s );
}

int ON_wString::Find( unsigned char c ) const
{
  return Find( (char)c );
}

int ON_wString::Find( wchar_t c ) const
{
	// find first single character
  wchar_t s[2];
  s[0] = c;
  s[1] = 0;
  return Find( s );
}

int ON_wString::ReverseFind( char c ) const
{
  wchar_t w = c2w(c);
  return ReverseFind( w );
}

int ON_wString::ReverseFind( unsigned char c ) const
{
  wchar_t w = c2w((char)c);
  return ReverseFind( w );
}

int ON_wString::ReverseFind( wchar_t c ) const
{
	// find first single character
  if ( IsEmpty() )
    return -1;
  int i;
  const int length = Length();
  for ( i = length-1; i >= 0; i-- ) {
    if ( c == m_s[i] )
      return i;
  }
  return -1;
}

int ON_wString::Find( const char* s ) const
{
  int rc = -1;
  if ( s && s[0] && !IsEmpty() ) {
    const int s_count = (int)strlen(s); // the (int) is for 64 bit size_t conversion
    wchar_t* w = (wchar_t*)onmalloc( (s_count+2)*sizeof(w[0]) );
    c2w( s_count, s, s_count+1, w );
    const wchar_t* p;
    p = wcsstr( m_s, w );
    if ( p )
    {
      rc = ((int)(p-m_s)); // the (int) cast is for 64 bit compilers
    }
    onfree( w );
  }
  return rc;
}

int ON_wString::Find( const unsigned char* s ) const
{
  return Find( (const char*)s );
}

int ON_wString::Find( const wchar_t* s ) const
{
  int rc = -1;
  if ( s && s[0] && !IsEmpty() ) {
    const wchar_t* p;
    p = wcsstr( m_s, s );
    if ( p )
    {
      rc = ((int)(p-m_s)); // the (int) is for 64 bit size_t conversion
    }
  }
  return rc;
}

void ON_wString::MakeReverse()
{
  if ( !IsEmpty() ) {
  	CopyArray();
    on_wcsrev(m_s);
  }
}

void ON_wString::TrimLeft(const wchar_t* s)
{
  wchar_t c;
  const wchar_t* sc;
  wchar_t* dc;
  int i;
  if ( !IsEmpty() ) {
    if ( !s )
      s = L" \t\n";
    for ( i = 0; 0 != (c=m_s[i]); i++ )
    {
      for (sc = s;*sc;sc++) {
        if ( *sc == c )
          break;
      }
      if (!(*sc))
        break;
    }
    if ( i > 0 ) {
      if ( m_s[i] ) {
        CopyArray();
        dc = m_s;
        sc = m_s+i;
        while( 0 != (*dc++ = *sc++) );
        Header()->string_length -= i;
      }
      else
        Destroy();
    }
  }
}

void ON_wString::TrimRight(const wchar_t* s)
{
  wchar_t c;
  const wchar_t* sc;
  int i = Header()->string_length;
  if ( i > 0 ) {
    if ( !s )
      s = L" \t\n";
    for (i--; i >= 0 && 0 != (c=m_s[i]); i-- )
    {
      for (sc = s;*sc;sc++) {
        if ( *sc == c )
          break;
      }
      if (!(*sc))
        break;
    }
    if ( i < 0 )
      Destroy();
    else if ( m_s[i+1] ) {
      CopyArray();
      m_s[i+1] = 0;
      Header()->string_length = i+1;
    }
  }
}

void ON_wString::TrimLeftAndRight(const wchar_t* s)
{
  TrimRight(s);
  TrimLeft(s);
}


int ON_wString::Remove( wchar_t c)
{
  wchar_t* s0;
  wchar_t* s1;
  wchar_t* s;
  int n;

  if ( 0 == (s0 = m_s) )
    return 0;
  s1 = s0 + Length();
  while( s0 < s1 )
  {
    if (c == *s0++)
    {
      // need to modify this string
      n = ((int)(s0 - m_s)); // keep win64 happy with (int) cast
      CopyArray(); // may change m_s if string has multiple refs
      s0 = m_s + n;
      s = s0-1;
      s1 = m_s + Length();
      while ( s0 < s1 )
      {
        if ( c != *s0 )
        {
          *s++ = *s0;
        }
        s0++;
      }
      *s = 0;
      n = ((int)(s1 - s)); // keep win64 happy with (int) cast
      Header()->string_length -= n;
      return n;
    }
  }
  return 0;
}

wchar_t ON_wString::GetAt( int i ) const
{
  return m_s[i];
}


void ON_wString::SetAt( int i, char c )
{
  if ( i >= 0 && i < Header()->string_length ) {
	  CopyArray();
	  m_s[i] = c2w(c);
  }
}

void ON_wString::SetAt( int i, unsigned char c )
{
  SetAt( i, (char)c );
}

void ON_wString::SetAt( int i, wchar_t c )
{
  if ( i >= 0 && i < Header()->string_length ) {
	  CopyArray();
	  m_s[i] = c;
  }
}

ON_wString ON_wString::Mid(int i, int count) const
{
  ON_wString(s);
  if ( i >= 0 && i < Length() && count > 0 ) {
    if ( count > Length() - i )
      count = Length() - i;
    s.CopyToArray( count, &m_s[i] );
  }
  return s;
}

ON_wString ON_wString::Mid(int i) const
{
  return Mid( i, Length() - i );
}

ON_wString ON_wString::Left(int count) const
{
  ON_wString s;
  if ( count > Length() )
    count = Length();
  if ( count > 0 ) {
    s.CopyToArray( count, m_s );
  }
  return s;
}

ON_wString ON_wString::Right(int count) const
{
  ON_wString s;
  if ( count > Length() )
    count = Length();
  if ( count > 0 ) {
    s.CopyToArray( count, &m_s[Length()-count] );
  }
  return s;
}

void ON_MSC_CDECL ON_wString::Format( const char* sFormat, ...)
{
#define MAX_MSG_LENGTH 2048
  char sMessage[MAX_MSG_LENGTH];
  va_list args;

  /* put formatted message in sMessage */
  memset(sMessage,0,sizeof(sMessage));
  if (sFormat) {
    va_start(args, sFormat);
    on_vsnprintf(sMessage, MAX_MSG_LENGTH-1, sFormat, args);
    sMessage[MAX_MSG_LENGTH-1] = 0;
    va_end(args);
  }
  const int len = Length(sMessage);
  if ( len < 1 ) {
    Destroy();
    Create();
  }
  else {
    ReserveArray( len );
    CopyToArray( len, sMessage );
  }
}

void ON_MSC_CDECL ON_wString::Format( const unsigned char* sFormat, ...)
{
#define MAX_MSG_LENGTH 2048
  char sMessage[MAX_MSG_LENGTH];
  va_list args;

  /* put formatted message in sMessage */
  memset(sMessage,0,sizeof(sMessage));
  if (sFormat) {
    va_start(args, sFormat);
    on_vsnprintf(sMessage, MAX_MSG_LENGTH-1, (const char*)sFormat, args);
    sMessage[MAX_MSG_LENGTH-1] = 0;
    va_end(args);
  }
  const int len = Length(sMessage);
  if ( len < 1 ) {
    Destroy();
    Create();
  }
  else {
    ReserveArray( len );
    CopyToArray( len, sMessage );
  }
}

void ON_MSC_CDECL ON_wString::Format( const wchar_t* sFormat, ...)
{
#define MAX_MSG_LENGTH 2048
  wchar_t sMessage[MAX_MSG_LENGTH];
  va_list args;

  /* put formatted message in sMessage */
  memset(sMessage,0,sizeof(sMessage));
  if (sFormat) {
    va_start(args, sFormat);
    on_vsnwprintf(sMessage, MAX_MSG_LENGTH-1, sFormat, args);
    sMessage[MAX_MSG_LENGTH-1] = 0;
    va_end(args);
  }
  const int len = Length(sMessage);
  if ( len < 1 ) {
    Destroy();
    Create();
  }
  else {
    ReserveArray( len );
    CopyToArray( len, sMessage );
  }
}

///////////////////////////////////////////////////////////////////////////////

bool ON_wString::operator==(const ON_wString& s2) const
{
  return (Compare(s2) == 0) ? true : false;
}

bool ON_wString::operator==(const wchar_t* s2) const
{
  return (Compare(s2) == 0) ? true : false;
}

bool ON_wString::operator!=(const ON_wString& s2) const
{
  return (Compare(s2) != 0) ? true : false;
}

bool ON_wString::operator!=(const wchar_t* s2) const
{
  return (Compare(s2) != 0) ? true : false;
}

bool ON_wString::operator<(const ON_wString& s2) const
{
  return (Compare(s2) < 0) ? true : false;
}

bool ON_wString::operator<(const wchar_t* s2) const
{
  return (Compare(s2) < 0) ? true : false;
}

bool ON_wString::operator>(const ON_wString& s2) const
{
  return (Compare(s2) > 0) ? true : false;
}

bool ON_wString::operator>(const wchar_t* s2) const
{
  return (Compare(s2) > 0) ? true : false;
}

bool ON_wString::operator<=(const ON_wString& s2) const
{
  return (Compare(s2) <= 0) ? true : false;
}

bool ON_wString::operator<=(const wchar_t* s2) const
{
  return (Compare(s2) <= 0) ? true : false;
}

bool ON_wString::operator>=(const ON_wString& s2) const
{
  return (Compare(s2) >= 0) ? true : false;
}

bool ON_wString::operator>=(const wchar_t* s2) const
{
  return (Compare(s2) >= 0) ? true : false;
}


void ON_String::SplitPath( 
    const char* path,
    ON_String* drive,
    ON_String* dir,
    ON_String* fname,
    ON_String* ext
    )
{
  const char* dr = 0;
  const char* d = 0;
  const char* f = 0;
  const char* e = 0;

  on_splitpath(path,&dr,&d,&f,&e);

  if ( 0 != drive )
  {
    if ( 0 != dr )
    {
      if ( 0 != d )
        drive->CopyToArray((int)(d-dr),dr);
      else if ( 0 != f )
        drive->CopyToArray((int)(f-dr),dr);
      else if ( 0 != e )
        drive->CopyToArray((int)(e-dr),dr);
      else
        *drive = dr;
    }
    else
      drive->Empty();
  }

  if ( 0 != dir )
  {
    if ( 0 != d )
    {
      if ( 0 != f )
        dir->CopyToArray((int)(f-d),d);
      else if ( 0 != e )
        dir->CopyToArray((int)(e-d),d);
      else
        *dir = d;
    }
    else
      dir->Empty();
  }

  if ( 0 != fname )
  {
    if ( 0 != f )
    {
      if ( 0 != e )
        fname->CopyToArray((int)(e-f),f);
      else
        *fname = f;
    }
    else
      fname->Empty();
  }

  if ( 0 != ext )
  {
    *ext = e;
  }
}

void ON_wString::SplitPath( 
    const char* path,
    ON_wString* drive,
    ON_wString* dir,
    ON_wString* fname,
    ON_wString* ext
    )
{
  const char* dr = 0;
  const char* d = 0;
  const char* f = 0;
  const char* e = 0;

  on_splitpath(path,&dr,&d,&f,&e);

  if ( 0 != drive )
  {
    if ( 0 != dr )
    {
      if ( 0 != d )
        drive->CopyToArray((int)(d-dr),dr);
      else if ( 0 != f )
        drive->CopyToArray((int)(f-dr),dr);
      else if ( 0 != e )
        drive->CopyToArray((int)(e-dr),dr);
      else
        *drive = dr;
    }
    else
      drive->Empty();
  }

  if ( 0 != dir )
  {
    if ( 0 != d )
    {
      if ( 0 != f )
        dir->CopyToArray((int)(f-d),d);
      else if ( 0 != e )
        dir->CopyToArray((int)(e-d),d);
      else
        *dir = d;
    }
    else
      dir->Empty();
  }

  if ( 0 != fname )
  {
    if ( 0 != f )
    {
      if ( 0 != e )
        fname->CopyToArray((int)(e-f),f);
      else
        *fname = f;
    }
    else
      fname->Empty();
  }

  if ( 0 != ext )
  {
    *ext = e;
  }
}

void ON_wString::SplitPath( 
    const wchar_t* path,
    ON_wString* drive,
    ON_wString* dir,
    ON_wString* fname,
    ON_wString* ext
    )
{
  const wchar_t* dr = 0;
  const wchar_t* d = 0;
  const wchar_t* f = 0;
  const wchar_t* e = 0;

  on_wsplitpath(path,&dr,&d,&f,&e);

  if ( 0 != drive )
  {
    if ( 0 != dr )
    {
      if ( 0 != d )
        drive->CopyToArray((int)(d-dr),dr);
      else if ( 0 != f )
        drive->CopyToArray((int)(f-dr),dr);
      else if ( 0 != e )
        drive->CopyToArray((int)(e-dr),dr);
      else
        *drive = dr;
    }
    else
      drive->Empty();
  }

  if ( 0 != dir )
  {
    if ( 0 != d )
    {
      if ( 0 != f )
        dir->CopyToArray((int)(f-d),d);
      else if ( 0 != e )
        dir->CopyToArray((int)(e-d),d);
      else
        *dir = d;
    }
    else
      dir->Empty();
  }

  if ( 0 != fname )
  {
    if ( 0 != f )
    {
      if ( 0 != e )
        fname->CopyToArray((int)(e-f),f);
      else
        *fname = f;
    }
    else
      fname->Empty();
  }

  if ( 0 != ext )
  {
    *ext = e;
  }
}


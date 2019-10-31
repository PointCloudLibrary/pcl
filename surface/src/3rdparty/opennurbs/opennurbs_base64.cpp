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

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_DecodeBase64
// 

ON_DecodeBase64::ON_DecodeBase64()
{
  Begin();
}

ON_DecodeBase64::~ON_DecodeBase64()
{
  Begin();
}

void ON_DecodeBase64::Begin()
{
  m_decode_count = 0;
  m_output_count = 0;
  memset(m_output,0,512);
  m_status = 0;
  m_cache_count = 0;
  m_cache[0] = 0;
  m_cache[1] = 0;
  m_cache[2] = 0;
  m_cache[3] = 0;  
}

bool ON_DecodeBase64::End()
{
  if ( 0 != m_status )
  {
    if ( 3 == m_status || 4 == m_status )
    {
      if ( 0 != m_output_count )
        SetError(); // all output should have been flushed
      else
        m_status = 5; // finished
    }
    else if ( 1 != m_status )
      SetError();
  }
  else
  {
    if ( m_output_count > 0 )
    {
      Output();
      m_output_count = 0;
    }
    m_status = 5; // finished
  }
  m_output_count = 0;
  memset(m_output,0,512);
  return ( 1 != m_status );
}

void ON_DecodeBase64::SetError()
{
  // unrecoverable error
  ON_ERROR("ON_DecodeBase64::Decode - error");
  m_status = 1;
}

bool ON_DecodeBase64::Error() const
{
  return (1 == m_status);
}

void ON_DecodeBase64::DecodeHelper1()
{
  // Send the last byte of decoded output when the
  // last 4 bytes of the base64 encoded string are "**=="
  // This function is called at most one time to
  // decode the last base 64 quartet into the final
  // byte of output.

  union
  {
    ON__INT32 i;
    unsigned char b[4];
  } u;
  m_status = 0;
  if ( m_output_count >= 512 )
  {
    Output();
    m_output_count = 0;
  }
  u.i = 4*m_cache[0] + m_cache[1]/16;
  m_output[m_output_count++] = u.b[0];
  Output();
  m_output_count = 0;
}

void ON_DecodeBase64::DecodeHelper2()
{
  // Send the last 2 bytes of decoded output when the
  // last 4 bytes of base64 encoded string are "***=".
  // This function is called at most one time to
  // decode the last base 64 quartet into the final
  // two bytes of output.

  union
  {
    ON__INT32 i;
    unsigned char b[4];
  } u;
  m_status = 0;
  if ( m_output_count >= 511 )
  {
    Output();
    m_output_count = 0;
  }

  u.i = 1024*m_cache[0] + 16*m_cache[1] + m_cache[2]/4;
  m_output[m_output_count++] = u.b[1];
  m_output[m_output_count++] = u.b[0];
  Output();
  m_output_count = 0;
}

const char* ON_DecodeBase64::Decode(const char* base64str)
{
  union
  {
    ON__INT32 i;
    unsigned char b[4];
  } u;
  ON__INT32 i;
  unsigned char* outbuf;

  //#if defined(_DEBUG)
  //  if (    m_cache_count < 0
  //       || m_cache_count >= 4
  //       || m_cache_count != (m_decode_count % 4) 
  //     )
  //  {
  //    // algorithm error
  //    SetError();
  //    return 0;
  //  }
  //#endif

  if ( m_status )
  {
    // rarely executed code
    if ( 1 == m_status )
    {
      return 0;
    }
    if ( base64str )
    {
      i = *base64str;
      if      (i >= 65 && i <=  90) i =  1;
      else if (i >= 97 && i <= 122) i =  1;
      else if (i >= 48 && i <=  57) i =  1;
      else if ('+' == i)            i =  1; 
      else if ('/' == i)            i =  1;
      else if ('=' == i)            i = -1;
      else
      {
        return 0;
      }

      if ( 2 != m_status || -1 != m_cache[2] )
      {
        SetError();
        return 0;
      }
      if ( -1 != i )
      {
        // base64 encoded strings are parsed in groups of 4 characters.
        // When we enter this part of the Decode function, m_status is
        // either 1 (error occured earlier) or 2.  
        // A 2 means the previous character we parsed was the 3rd
        // of the group and it was an equal sign.  In this case, the
        // the 4th character in the group must be an equal sign and
        // the group encodes a single byte.
        SetError();
        return 0;
      }
    }
  }

  if (!base64str)
    return 0;

  outbuf = m_output+m_output_count;

  for(;;)
  {
    while ( m_cache_count < 4 )
    {
      // base 64 encodes 3 bytes as a 4 digit base 64 number.
      // The base 64 "digits" are A-z,a-z,0-9,+ and /.  The
      // values of these "digits" are listed below.
      // 'A' ->  0
      // ...
      // 'Z' -> 25
      // 'a' -> 26
      // ...
      // 'z' -> 51
      // '0' -> 52
      // ...
      // '9' -> 61
      // '+' -> 62
      // '/' -> 63
      // '=' padding used to encode the last one or two bytes
      //     If the 3rd and 4th characters in the quartet are
      //     equal signs, the quartet represents a single byte
      //     as a 2 digit base 64 number.  
      //     If the 4th character in the quartet is an equal sign,
      //     the quartet represents two bytes as a 3 digit 
      //     base 64 number.  
      i = *base64str++;
      if      (i >= 65 && i <=  90) i -= 65;
      else if (i >= 97 && i <= 122) i -= 71;
      else if (i >= 48 && i <=  57) i +=  4;
      else if ('+' == i) i = 62; 
      else if ('/' == i) i = 63;
      else if ('=' == i) 
      {
        if ( m_cache_count < 2 )
        {
          // An equal sign cannot be the 1rst or 2nd character
          // in a 4 character block
          SetError();
          return 0;
        }
        if ( 2 == m_cache_count )
        {
          // This equal sign is the 3rd character.  The next 
          // character must also be an = sign or the input is
          // not valid.
          m_status = 2;
        }
        else // 3 == m_cache_count
        {
          // This equal sign is the 4th character.
          // This must be the last encoded character.
          if ( -1 == m_cache[2] )
          {
            // block ends with 2 equal signs
            // and will decode into a single byte
            m_status = 3;
            m_cache[m_cache_count++] = -1;
            m_decode_count++;
            DecodeHelper1();
            return base64str;
          }
          else
          {
            // block ends with 1 equal sign and will
            // decode into 2 bytes.
            m_status = 4;
            m_cache[m_cache_count++] = -1;
            m_decode_count++;
            DecodeHelper2();
            return base64str;
          }
        }
        i = -1;
      }
      else
      {
        // end of valid portion of this base64str 
        return (base64str-1);
      }
      m_cache[m_cache_count++] = i;
      m_decode_count++;
    }

    m_cache_count = 0;

    // 3 bytes of output
    if ( m_output_count >= 510 )
    {
      Output();
      m_output_count = 0;
      outbuf = m_output;
    }
    u.i = m_cache[3] + 64*(m_cache[2] + 64*(m_cache[1] + 64*m_cache[0]));
    *outbuf++ = u.b[2];
    *outbuf++ = u.b[1];
    *outbuf++ = u.b[0];
    m_output_count += 3;
  }

  //return 0;
}

const char* ON_DecodeBase64::Decode(const char* base64str, std::size_t base64str_count)
{
  char* sEnd;
  const char* p;
  char s[1025];
  if ( 0 == base64str )
    return 0;
  sEnd = s + 1024;
  *sEnd = 0;
  while ( base64str_count >= 1024 )
  {
    memcpy(s,base64str,1024);
    p = Decode(s);
    if ( 0 == p )
      return 0;
    if ( p != sEnd )
    {
      return base64str + (p - s);
    }
    base64str += 1024;
    base64str_count -= 1024;
  }
  if ( base64str_count > 0 )
  {
    memcpy(s,base64str,base64str_count);
    s[base64str_count]=0;
    p = Decode(s);
    if ( 0 == p )
      return 0;
    base64str += (p - s);
  }
  return base64str;
}

const wchar_t* ON_DecodeBase64::Decode(const wchar_t* base64str)
{
  const wchar_t* p;
  wchar_t w;
  if ( 0 == base64str )
    return 0;
  p = base64str;
  for(;;)
  {
    w = *p++;
    if ( w < 32 || w > 122 )
      break;
  }
  return Decode(base64str,p-base64str);
}

const wchar_t* ON_DecodeBase64::Decode(const wchar_t* base64str, std::size_t base64str_count)
{
  char* sEnd;
  const char* p;
  char s[1025];
  std::size_t i;
  wchar_t w;
  if ( 0 == base64str )
    return 0;
  sEnd = s + 1024;
  *sEnd = 0;
  while ( base64str_count >= 1024 )
  {
    for(i=0;i<1024;i++)
    {
      w = base64str[i];
      if ( w < 32 || w > 122 )
      {
        s[i] = 0;
        break;
      }
      s[i] = (char)w;
    }
    p = Decode(s);
    if ( 0 == p )
      return 0;
    if ( p != sEnd )
    {
      return base64str + (p - s);
    }
    base64str += 1024;
    base64str_count -= 1024;
  }
  if ( base64str_count > 0 )
  {
    for(i=0;i<base64str_count;i++)
    {
      w = base64str[i];
      if ( w < 32 || w > 122 )
      {
        s[i] = 0;
        break;
      }
      s[i] = (char)w;
    }
    s[i] = 0;
    p = Decode(s);
    if ( 0 == p )
      return 0;
    base64str += (p - s);
  }
  return base64str;
}

// virtual 
void ON_DecodeBase64::Output()
{
  // default does nothing.
}

/*
//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_EncodeBase64
// 

ON_EncodeBase64::ON_EncodeBase64()
{
  Begin();
}

ON_EncodeBase64::~ON_EncodeBase64()
{
  Begin();
}

//vitrual
void ON_EncodeBase64::Output()
{
  // The default does nothing - override this function
  // if you want to do something useful with the output.
}

void ON_EncodeBase64::Begin()
{
  m_encode_count = 0;
  m_output_count = 0;
  memset(m_output,0,80);
  m_unused2 = 0;
  m_input_count = 0;
  memset(m_input,0,64);
}

void ON_EncodeBase64::EncodeHelper1(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode the final byte of input into 4 bytes of outbuf.
  unsigned char c;
  
  c = (*inbuf >> 2);

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (*inbuf & 3) << 4;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  *outbuf++ = '=';
  *outbuf   = '=';
}

void ON_EncodeBase64::EncodeHelper2(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode the final 2 bytes of input into 4 bytes of outbuf.
  unsigned char b, c;
  
  b = *inbuf++;
  c = (b >> 2);

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 3) << 4;
  b = *inbuf;
  c |= (b & 0xF0) >> 4;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 0x0F) << 2;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  *outbuf  = '=';
}


void ON_EncodeBase64::EncodeHelper3(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode 3 bytes from inbuf into 4 bytes of outbuf.
  unsigned char b, c;
  
  b = *inbuf++;
  c = (b >> 2);

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 3) << 4;
  b = *inbuf++;
  c |= (b & 0xF0) >> 4;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 0x0F) << 2;
  b = *inbuf++;
  c |= (b&0xC0) >> 6;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  b &= 0x3F;
  if ( b < 26 ) b += 65; else if ( b < 52 ) b += 71; 
  else if ( b < 62 ) b -= 4; else b = (b&1) ? '/' : '+';
  *outbuf++ = b;
}

void ON_EncodeBase64::EncodeHelper57(const unsigned char* inbuf )
{
  // base64 encode 57 bytes from inbuf and put results in m_output[]
  //
  // Encoding 57 input bytes creates 76 output bytes and 76
  // is the maximum line length for base64 encoding.
  char* outbuf = m_output;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeHelper3(inbuf,outbuf); 
  memset(outbuf+4,0,4);
  m_encode_count += 57;
}

void ON_EncodeBase64::Encode( const void* buffer, std::size_t sizeof_buffer )
{
  // code is designed for speed
  unsigned int sz;
  const unsigned char* inbuf;
  if ( sizeof_buffer <= 0 || !buffer )
    return;

  if ( m_input_count )
  {
    // residual input left from the previous call to Encode()
    sz = 57 - m_input_count;
    if ( sizeof_buffer < sz )
    {
      // still don't have 57 bytes of input
      memcpy(m_input+m_input_count,buffer,sizeof_buffer);
      m_input_count += ((int)sizeof_buffer); // safe cast - sizeof_buffer < 57
      return;
    }

    // m_input[] has 57 bytes - encode it
    memcpy(m_input+m_input_count,buffer,sz);
    EncodeHelper57(m_input);
    m_output_count = 76;
    m_input_count = 0;
    Output();
    if ( sizeof_buffer == sz )
    {
      // no input left
      m_output_count = 0;
      *m_output = 0;
      return;
    }

    // deal with remaining input
    buffer = ((const unsigned char*)buffer + sz);
    sizeof_buffer -= sz;
  }

  m_output_count = 76;
  inbuf = (const unsigned char*)buffer;
  while(sizeof_buffer >= 57 )
  {
    // encode 57 bytes in m_input_count[]
    EncodeHelper57(inbuf);
    Output();  
    inbuf += 57;
    sizeof_buffer -= 57;
  }

  if ( sizeof_buffer )
  {
    // store what's left of the input in m_input[]
    memcpy(m_input,inbuf,sizeof_buffer);
    m_input_count = (int)sizeof_buffer; // safe cast - sizeof_buffer < 57
  }

  m_output_count = 0;
  *m_output = 0;
}

void ON_EncodeBase64::End()
{
  const unsigned char* inbuf;
  char* outbuf;
  m_output_count = 0;
  if ( m_input_count > 0 )
  {
    // flush rest of input
    inbuf = m_input;
    outbuf = m_output;
    while ( m_input_count >= 3 )
    {
      EncodeHelper3(inbuf,outbuf);
      inbuf += 3;
      outbuf += 4;
      m_input_count -= 3;
      m_output_count += 4;
      m_encode_count += 3;
    }
    if ( 1 == m_input_count )
    {
      // 1 byte of input left
      EncodeHelper1(inbuf,outbuf);
      outbuf += 4;
      m_output_count += 4;
      m_encode_count++;
    }
    else if ( 2 == m_input_count )
    {
      // 2 bytes of input left
      EncodeHelper2(inbuf,outbuf);
      outbuf += 4;
      m_output_count += 4;
      m_encode_count += 2;
    }
    memset(outbuf,0,80-m_output_count);
    m_input_count = 0;
    Output();
    m_output_count = 0;
  }
  *m_output = 0;
}

*/

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


ON_Base64EncodeStream::ON_Base64EncodeStream()
: m_out_callback_function(0)
, m_out_callback_context(0)
, m_in_size(0)
, m_out_size(0)
, m_in_crc(0)
, m_out_crc(0)
, m_implementation(0)
, m_reserved(0)
{}

ON_Base64EncodeStream::~ON_Base64EncodeStream()
{

  if ( 0 != m_implementation )
  {
    onfree(m_implementation);
    m_implementation = 0;
  }
}

void ON_Base64EncodeStream::ErrorHandler()
{
  // place holder for error handing
  ON_ERROR("ON_UncompressStream error");
}

class ON_Base64EncodeImplementation
{
public:
  // input waiting to be encoded
  // At most 56 bytes can be waiting to be processed in m_in_buffer[].
  // m_in_buffer[] is full when it has 57 bytes.
  ON__UINT32 m_in_buffer_size;
  unsigned char m_in_buffer[64];

  // When the output stream handler is called, m_out_buffer[]
  // is a null terminated string with 4 to 76 characters of 
  // base64 encoded output.
  char m_out_buffer[80];
};


bool ON_Base64EncodeStream::Begin()
{
  if ( 0 != m_implementation )
  {
    onfree(m_implementation);
    m_implementation = 0;
  }

  // zero these because the same instance of an 
  // ON_UncompressStream class may be used multiple times.
  m_in_size = 0;
  m_out_size = 0;
  m_in_crc = 0;
  m_out_crc = 0;

  ON_Base64EncodeImplementation* imp = (ON_Base64EncodeImplementation*)onmalloc(sizeof(*imp));
  memset(imp,0,sizeof(*imp));
  m_implementation = imp;

  return true;
}

static void EncodeBase64Helper1(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode the final byte of input into 4 bytes of outbuf.
  unsigned char c;
  
  c = (*inbuf >> 2);

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (*inbuf & 3) << 4;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  *outbuf++ = '=';
  *outbuf   = '=';
}

static void EncodeBase64Helper2(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode the final 2 bytes of input into 4 bytes of outbuf.
  unsigned char b, c;
  
  b = *inbuf++;
  c = (b >> 2);

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 3) << 4;
  b = *inbuf;
  c |= (b & 0xF0) >> 4;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 0x0F) << 2;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  *outbuf  = '=';
}


static void EncodeBase64Helper3(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode 3 bytes from inbuf into 4 bytes of outbuf.
  unsigned char b, c;
  
  b = *inbuf++;
  c = (b >> 2);

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 3) << 4;
  b = *inbuf++;
  c |= (b & 0xF0) >> 4;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  c = (b & 0x0F) << 2;
  b = *inbuf++;
  c |= (b&0xC0) >> 6;

  if ( c < 26 ) c += 65; else if ( c < 52 ) c += 71; 
  else if ( c < 62 ) c -= 4; else c = (c&1) ? '/' : '+';
  *outbuf++ = c;

  b &= 0x3F;
  if ( b < 26 ) b += 65; else if ( b < 52 ) b += 71; 
  else if ( b < 62 ) b -= 4; else b = (b&1) ? '/' : '+';
  *outbuf++ = b;
}

static void EncodeBase64Helper57(const unsigned char* inbuf, char* outbuf )
{
  // base64 encode 57 bytes from inbuf and put results in m_output[]
  //
  // Encoding 57 input bytes creates 76 output bytes and 76
  // is the maximum line length for base64 encoding.
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  inbuf += 3; outbuf += 4;
  EncodeBase64Helper3(inbuf,outbuf); 
  outbuf[4] = 0;
}


bool ON_Base64EncodeStream::In(
    ON__UINT64 in_buffer_size,
    const void* in_buffer
    )
{
  if ( in_buffer_size <= 0 )
    return true;

  if ( 0 == m_implementation )
  {
    ErrorHandler();
    return false;
  }

  if ( 0 == in_buffer )
  {
    ErrorHandler();
    return false;
  }
  
  bool rc = false;
  ON__UINT32 crc1;
  ON__UINT32 sz;

  ON_Base64EncodeImplementation* imp = (ON_Base64EncodeImplementation*)m_implementation;
  if ( imp->m_in_buffer_size > 0 )
  {
    // residual input left from the previous call to In()
    sz = 57 - imp->m_in_buffer_size;
    if ( in_buffer_size < sz )
    {
      // still don't have sizeof(imp->m_in_buffer_size) bytes of input
      sz = (ON__UINT32)in_buffer_size; // cast is safe because in_buffer_size < sizeof(imp->m_in_buffer_size) <= 8192
      memcpy( imp->m_in_buffer + imp->m_in_buffer_size, in_buffer, sz );
      imp->m_in_buffer_size += sz;
      return true;
    }

    memcpy( imp->m_in_buffer + imp->m_in_buffer_size, in_buffer, sz );
    in_buffer = ((const unsigned char*)in_buffer) + sz;
    in_buffer_size -= sz;

    EncodeBase64Helper57(imp->m_in_buffer,imp->m_out_buffer);
    imp->m_in_buffer_size = 0;

    crc1 = ON_CRC32(m_out_crc,76,imp->m_out_buffer);
    rc = ( 0 != m_out_callback_function )
        ? m_out_callback_function(m_out_callback_context,76,imp->m_out_buffer)
        : Out(m_out_callback_context,76,imp->m_out_buffer);
    if ( !rc )
    {
      onfree(m_implementation);
      m_implementation = 0;
      return false;
    }
    m_in_crc = ON_CRC32(m_in_crc,57,imp->m_in_buffer);
    m_in_size += 57;
    m_out_crc = crc1;
    m_out_size += 76;
  }

  while(in_buffer_size >= 57 )
  {
    // base 64 encode 57 input bytes to create 76 output bytes
    EncodeBase64Helper57((const unsigned char*)in_buffer,imp->m_out_buffer);
    crc1 = ON_CRC32(m_out_crc,76,imp->m_out_buffer);
    rc = ( 0 != m_out_callback_function )
        ? m_out_callback_function(m_out_callback_context,76,imp->m_out_buffer)
        : Out(m_out_callback_context,76,imp->m_out_buffer);
    if ( !rc )
    {
      onfree(m_implementation);
      m_implementation = 0;
      return false;
    }
    m_in_crc = ON_CRC32(m_in_crc,57,in_buffer);
    m_in_size += 57;
    m_out_crc = crc1;
    m_out_size += 76;
    in_buffer = ((const unsigned char*)in_buffer) + 57;
    in_buffer_size -= 57;
  }

  if ( in_buffer_size > 0 )
  {
    // store what's left of the input in imp->m_in_buffer[]
    memcpy(imp->m_in_buffer,in_buffer,(std::size_t)in_buffer_size);
  }
  imp->m_in_buffer_size = (ON__UINT32)in_buffer_size; // safe cast - sizeof_buffer < 57

  return true;
}

bool ON_Base64EncodeStream::End()
{
  if ( 0 == m_implementation )
  {
    ErrorHandler();
    return false;
  }
  
  bool rc = true;
  ON_Base64EncodeImplementation* imp = (ON_Base64EncodeImplementation*)m_implementation;
  if ( imp->m_in_buffer_size > 0 )
  {
    // residual input left from the final call to In()
    const unsigned char* in_buffer = imp->m_in_buffer;
    ON__UINT32 in_buffer_size = imp->m_in_buffer_size;
    ON__UINT32 out_buffer_size = 0;
    while ( in_buffer_size >= 3 )
    {
      EncodeBase64Helper3( in_buffer, &imp->m_out_buffer[out_buffer_size] );
      in_buffer += 3;
      out_buffer_size += 4;
      in_buffer_size -= 3;
    }
    if ( 1 == in_buffer_size )
    {
      // 1 byte of input left
      EncodeBase64Helper1(in_buffer,&imp->m_out_buffer[out_buffer_size]);
      out_buffer_size += 4;
    }
    else if ( 2 == in_buffer_size )
    {
      // 2 bytes of input left
      EncodeBase64Helper2(in_buffer,&imp->m_out_buffer[out_buffer_size]);
      out_buffer_size += 4;
    }
    imp->m_out_buffer[out_buffer_size] = 0;

    ON__UINT32 crc1 = ON_CRC32(m_out_crc,out_buffer_size,imp->m_out_buffer);
    rc = ( 0 != m_out_callback_function )
        ? m_out_callback_function(m_out_callback_context,out_buffer_size,imp->m_out_buffer)
        : Out(m_out_callback_context,out_buffer_size,imp->m_out_buffer);
    if ( rc )
    {
      m_in_crc = ON_CRC32(m_in_crc,imp->m_in_buffer_size,imp->m_in_buffer);
      m_in_size += imp->m_in_buffer_size;
      m_out_crc = crc1;
      m_out_size += out_buffer_size;
    }
  }

  onfree(m_implementation);
  m_implementation = 0;

  return rc;
}

bool ON_Base64EncodeStream::Out( void*, ON__UINT32, const char* )
{
  // default base 64 encoded stream handler does nothing.
  return true;
}

bool ON_Base64EncodeStream::SetCallback( 
    ON_StreamCallbackFunction out_callback_function,
    void* out_callback_context
    )
{
  m_out_callback_function = out_callback_function;
  m_out_callback_context = out_callback_context;
  return true;
}

ON_StreamCallbackFunction ON_Base64EncodeStream::CallbackFunction() const
{
  return m_out_callback_function;
}

void* ON_Base64EncodeStream::CallbackContext() const
{
  return m_out_callback_context;
}

ON__UINT64 ON_Base64EncodeStream::InSize() const
{
  return m_in_size;
}

ON__UINT64 ON_Base64EncodeStream::OutSize() const
{
  return m_out_size;
}

ON__UINT32 ON_Base64EncodeStream::InCRC() const
{
  return m_in_crc;
}

ON__UINT32 ON_Base64EncodeStream::OutCRC() const
{
  return m_out_crc;
}



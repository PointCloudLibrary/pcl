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

struct ON_ZlibImplementation
{
  z_stream m_strm;
  unsigned char m_zlib_out_buffer[16384];
};

ON_CompressStream::ON_CompressStream()
: m_out_callback_function(0)
, m_out_callback_context(0)
, m_in_size(0)
, m_out_size(0)
, m_in_crc(0)
, m_out_crc(0)
, m_implementation(0)
, m_reserved(0)
{}


ON_CompressStream::~ON_CompressStream()
{

  if ( 0 != m_implementation )
  {
    onfree(m_implementation);
    m_implementation = 0;
  }
}

void ON_CompressStream::ErrorHandler()
{
  // place holder for error handing
  ON_ERROR("ON_CompressStream error");
}

bool ON_CompressStream::Begin()
{
  if ( 0 != m_implementation )
  {
    onfree(m_implementation);
    m_implementation = 0;
  }

  // zero these because the same instance of an 
  // ON_CompressStream class may be used multiple times.
  m_in_size = 0;
  m_out_size = 0;
  m_in_crc = 0;
  m_out_crc = 0;

  struct ON_ZlibImplementation* imp = (struct ON_ZlibImplementation*)onmalloc(sizeof(*imp));
  memset(&imp->m_strm,0,sizeof(imp->m_strm));

  if ( Z_OK != deflateInit( &imp->m_strm, Z_BEST_COMPRESSION ) )
  {
    onfree(imp);
    return false;
  }

  m_implementation = imp;

  return true;
}


bool ON_CompressStream::In( ON__UINT64 size, const void* uncompressed_buffer )
{
  if ( size <= 0 )
    return true;

  if ( 0 == m_implementation )
  {
    ErrorHandler();
    return false;
  }

  if ( 0 == uncompressed_buffer )
  {
    ErrorHandler();
    return false;
  }
  
  struct ON_ZlibImplementation* imp = (struct ON_ZlibImplementation*)m_implementation;
  z_stream& strm = imp->m_strm;
  if ( 0 != strm.avail_in || 0 != strm.next_in )
  {
    // strm.avail_in is always zero when we leave an ON_CompressStream function.
    ErrorHandler();
    return false;
  }

  const ON__UINT32 sizeof_out_buffer = (ON__UINT32)(sizeof(imp->m_zlib_out_buffer));
  void* out_buffer = imp->m_zlib_out_buffer;
  int zrc = Z_OK;
  const ON__UINT64 max_sz = 0x7FFFFFF0;
  bool rc = false;
  ON__UINT32 deflate_output_count;

  // counter prevents infinte loops if there is a bug in zlib return codes.
  for( int counter = 512; counter > 0; counter-- )
  {
    // Call zlib's deflate function.  It can either process
    // more input from m_zlib.strm.next_in[], create more
    // compressed output in m_zlib.strm.next_out[], or do both.

    // provide storage for compressed stream output
    strm.next_out  = (z_Bytef*)out_buffer;
    strm.avail_out = sizeof_out_buffer;

    if ( strm.avail_in <= 0 )
    {
      if ( size <= 0 )
      {
        // finshed with uncompressed input
        break;
      }
      // submit a portion of uncompressed_buffer to zlib
      ON__UINT64 sz = (size > max_sz) ? max_sz : size;
      m_in_size += sz;
      m_in_crc = ON_CRC32(m_in_crc,(size_t)sz,uncompressed_buffer); // (size_t) cast is safe because sz <= max_sz = 0x7FFFFFF0
      strm.next_in = (z_Bytef*)uncompressed_buffer;
      strm.avail_in = (ON__UINT32)sz;
      uncompressed_buffer = ((const unsigned char*)uncompressed_buffer) + sz;
      size -= sz;
      counter = 512; // added input - reset the counter that detects stalls
    }

    // calculate compression
    ON__UINT32 avail_in0 = strm.avail_in;
    ON__UINT32 avail_out0 = strm.avail_out;
    zrc = z_deflate( &strm, Z_NO_FLUSH ); 
    if ( zrc < 0 ) 
    {
      // Something went haywire - bail out.
      ErrorHandler();
      rc = false;
      break;
    }
    if ( strm.avail_in < avail_in0 || strm.avail_out > avail_out0 )
    {
      // zlib did something
      rc = true; 
    }    

    deflate_output_count = sizeof_out_buffer - strm.avail_out;
    if ( deflate_output_count > 0 ) 
    {
      // The last call to deflate created compressed output.  
      // Send the output to compressed stream handler.

      // Calculate the updated crc and size before we call
      // the output handler because someday sombody will
      // decide it's a good idea to modify the values
      // in the buffer argument.
      ON__UINT32 out_crc1 = ON_CRC32( m_out_crc, deflate_output_count, out_buffer);
      ON__UINT64 out_size1 = m_out_size + deflate_output_count;
      
      rc = (0 != m_out_callback_function)
          ? m_out_callback_function( m_out_callback_context, deflate_output_count, out_buffer )
          : Out( m_out_callback_context, deflate_output_count, out_buffer );
      if ( !rc )
        break;

      // Update compressed stream crc and size
      m_out_crc = out_crc1;
      m_out_size = out_size1;
      counter = 512; // created output - reset counter that detects stalls
    }

    if ( size <= 0 && strm.avail_in <= 0 )
    {
      // no input left
      break;
    }
  }

  strm.avail_in = 0;
  strm.next_in = 0;
  strm.next_out  = 0;
  strm.avail_out = 0;

  return rc;
}

bool ON_CompressStream::End()
{
  if ( 0 == m_implementation )
  {
    ErrorHandler();
    return false;
  }
  
  struct ON_ZlibImplementation* imp = (struct ON_ZlibImplementation*)m_implementation;
  z_stream& strm = imp->m_strm;
  if ( 0 != strm.avail_in || 0 != strm.next_in )
  {
    // strm.avail_in is always zero when we leave an ON_CompressStream function.
    ErrorHandler();
    return false;
  }

  const ON__UINT32 sizeof_out_buffer = (ON__UINT32)(sizeof(imp->m_zlib_out_buffer));
  void* out_buffer = imp->m_zlib_out_buffer;
  int zrc = Z_OK;
  bool rc = false;
  ON__UINT32 deflate_output_count;

  // counter prevents infinte loops if there is a bug in zlib return codes.
  for( int counter = 512; counter > 0; counter-- )
  {
    // provide storage for compressed stream output
    strm.avail_in = 0;
    strm.next_in = 0;
    strm.next_out  = (z_Bytef*)out_buffer;
    strm.avail_out = sizeof_out_buffer;

    // finish compression calculation
    zrc = z_deflate( &strm, Z_FINISH ); 
    if ( zrc < 0 ) 
    {
      // Something went haywire - bail out.
      ErrorHandler();
      rc = false;
      break;
    }

    deflate_output_count = sizeof_out_buffer - strm.avail_out;
    if ( deflate_output_count > 0 ) 
    {
      // The last call to deflate created compressed output.  
      // Send the output to compressed stream handler.

      // Calculate the updated crc and size before we call
      // the output handler because someday sombody will
      // decide it's a good idea to modify the values
      // in the buffer argument.
      ON__UINT32 compressed_crc1 = ON_CRC32( m_out_crc, deflate_output_count, out_buffer);
      ON__UINT64 compressed_size1 = m_out_size + ((ON__UINT64)deflate_output_count);
      
      rc = (0 != m_out_callback_function)
          ? m_out_callback_function( m_out_callback_context, deflate_output_count, out_buffer )
          : Out( m_out_callback_context, deflate_output_count, out_buffer );
      if ( !rc )
        break;

      // Update compressed stream crc and size
      m_out_crc = compressed_crc1;
      m_out_size = compressed_size1;
      counter = 512; // created output - reset counter that detects stalls
    }

    if ( Z_STREAM_END == zrc )
    {
      // no input left, all pending compressing is finished,
      // and all compressed output has been returned.
      rc = true;
      break;
    }
  }

  strm.avail_in = 0;
  strm.next_in = 0;
  strm.next_out  = 0;
  strm.avail_out = 0;

  deflateEnd(&strm);

  onfree(m_implementation);
  m_implementation = 0;

  return rc;
}

bool ON_CompressStream::Out( void*, ON__UINT32, const void* )
{
  // default compressed stream handler does nothing.
  return true;
}

bool ON_CompressStream::SetCallback( 
    ON_StreamCallbackFunction out_callback_function,
    void* out_callback_context
    )
{
  m_out_callback_function = out_callback_function;
  m_out_callback_context = out_callback_context;
  return true;
}

ON_StreamCallbackFunction ON_CompressStream::CallbackFunction() const
{
  return m_out_callback_function;
}

void* ON_CompressStream::CallbackContext() const
{
  return m_out_callback_context;
}


ON__UINT64 ON_CompressStream::InSize() const
{
  return m_in_size;
}

ON__UINT64 ON_CompressStream::OutSize() const
{
  return m_out_size;
}

ON__UINT32 ON_CompressStream::InCRC() const
{
  return m_in_crc;
}

ON__UINT32 ON_CompressStream::OutCRC() const
{
  return m_out_crc;
}


///////////////////////////////////////////////////////////////////////////////////////////


ON_UncompressStream::ON_UncompressStream()
: m_out_callback_function(0)
, m_out_callback_context(0)
, m_in_size(0)
, m_out_size(0)
, m_in_crc(0)
, m_out_crc(0)
, m_implementation(0)
, m_reserved(0)
{}


ON_UncompressStream::~ON_UncompressStream()
{

  if ( 0 != m_implementation )
  {
    onfree(m_implementation);
    m_implementation = 0;
  }
}

void ON_UncompressStream::ErrorHandler()
{
  // place holder for error handing
  ON_ERROR("ON_UncompressStream error");
}

bool ON_UncompressStream::Begin()
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

  struct ON_ZlibImplementation* imp = (struct ON_ZlibImplementation*)onmalloc(sizeof(*imp));
  memset(&imp->m_strm,0,sizeof(imp->m_strm));

  if ( Z_OK != inflateInit( &imp->m_strm ) )
  {
    onfree(imp);
    return false;
  }

  m_implementation = imp;


  return true;
}


bool ON_UncompressStream::In( ON__UINT64 size, const void* compressed_buffer )
{
  if ( size <= 0 )
    return true;

  if ( 0 == m_implementation )
  {
    ErrorHandler();
    return false;
  }

  if ( 0 == compressed_buffer )
  {
    ErrorHandler();
    return false;
  }
  
  struct ON_ZlibImplementation* imp = (struct ON_ZlibImplementation*)m_implementation;
  z_stream& strm = imp->m_strm;
  if ( 0 != strm.avail_in || 0 != strm.next_in )
  {
    // strm.avail_in is always zero when we leave an ON_UncompressStream function.
    ErrorHandler();
    return false;
  }

  const ON__UINT32 sizeof_out_buffer = (ON__UINT32)(sizeof(imp->m_zlib_out_buffer));
  void* out_buffer = imp->m_zlib_out_buffer;
  int zrc = Z_OK;
  const ON__UINT64 max_sz = 0x7FFFFFF0;
  bool rc = false;
  ON__UINT32 inflate_output_count;

  // counter prevents infinte loops if there is a bug in zlib return codes.
  for( int counter = 512; counter > 0; counter-- )
  {
    // Call zlib's inflate function.  It can process
    // more compressed input from strm.next_in[], create more
    // uncompressed output in strm.next_out[], or do both.

    // provide storage for uncompressed stream output
    strm.next_out  = (z_Bytef*)out_buffer;
    strm.avail_out = sizeof_out_buffer;

    if ( strm.avail_in <= 0 )
    {
      if ( size <= 0 )
      {
        // finshed with compressed input
        break;
      }
      // submit a portion of compressed_buffer to zlib
      ON__UINT64 sz = (size > max_sz) ? max_sz : size;
      m_in_size += sz;
      m_in_crc = ON_CRC32(m_in_crc,(size_t)sz,compressed_buffer); // (size_t) cast is safe because sz <= max_sz = 0x7FFFFFF0
      strm.next_in = (z_Bytef*)compressed_buffer;
      strm.avail_in = (ON__UINT32)sz;
      compressed_buffer = ((const unsigned char*)compressed_buffer) + sz;
      size -= sz;
      counter = 512; // added input - reset the counter that detects stalls
    }

    // calculate compression
    ON__UINT32 avail_in0 = strm.avail_in;
    ON__UINT32 avail_out0 = strm.avail_out;
    zrc = z_inflate( &strm, Z_NO_FLUSH ); 
    if ( zrc < 0 ) 
    {
      // Something went haywire - bail out.
      ErrorHandler();
      rc = false;
      break;
    }
    if ( strm.avail_in < avail_in0 || strm.avail_out > avail_out0 )
    {
      // zlib did something
      rc = true; 
    }    

    inflate_output_count = sizeof_out_buffer - strm.avail_out;
    if ( inflate_output_count > 0 ) 
    {
      // The last call to inflate created uncompressed output.  
      // Send the output to the uncompressed stream handler.

      // Calculate the updated crc and size before we call
      // the output handler because someday sombody will
      // decide it's a good idea to modify the values
      // in the buffer argument.
      ON__UINT32 out_crc1 = ON_CRC32( m_out_crc, inflate_output_count, out_buffer);
      ON__UINT64 out_size1 = m_out_size + inflate_output_count;
      
      rc = (0 != m_out_callback_function)
          ? m_out_callback_function( m_out_callback_context, inflate_output_count, out_buffer )
          : Out( m_out_callback_context, inflate_output_count, out_buffer );
      if ( !rc )
        break;

      // Update compressed stream crc and size
      m_out_crc = out_crc1;
      m_out_size = out_size1;
      counter = 512; // created output - reset counter that detects stalls
    }

    if ( size <= 0 && strm.avail_in <= 0 )
    {
      // no input left
      break;
    }
  }

  strm.avail_in = 0;
  strm.next_in = 0;
  strm.next_out  = 0;
  strm.avail_out = 0;

  return rc;
}

bool ON_UncompressStream::End()
{
  if ( 0 == m_implementation )
  {
    ErrorHandler();
    return false;
  }
  
  struct ON_ZlibImplementation* imp = (struct ON_ZlibImplementation*)m_implementation;
  z_stream& strm = imp->m_strm;
  if ( 0 != strm.avail_in || 0 != strm.next_in )
  {
    // strm.avail_in is always zero when we leave an ON_UncompressStream function.
    ErrorHandler();
    return false;
  }

  const ON__UINT32 sizeof_out_buffer = (ON__UINT32)(sizeof(imp->m_zlib_out_buffer));
  void* out_buffer = imp->m_zlib_out_buffer;
  int zrc = Z_OK;
  bool rc = false;
  ON__UINT32 inflate_output_count;

  // counter prevents infinte loops if there is a bug in zlib return codes.
  for( int counter = 512; counter > 0; counter-- )
  {
    // provide storage for compressed stream output
    strm.avail_in = 0;
    strm.next_in = 0;
    strm.next_out  = (z_Bytef*)out_buffer;
    strm.avail_out = sizeof_out_buffer;

    // finish compression calculation
    zrc = z_inflate( &strm, Z_FINISH ); 
    if ( zrc < 0 ) 
    {
      // Something went haywire - bail out.
      ErrorHandler();
      rc = false;
      break;
    }

    inflate_output_count = sizeof_out_buffer - strm.avail_out;
    if ( inflate_output_count > 0 ) 
    {
      // The last call to inflate created uncompressed output.  
      // Send the output to the uncompressed stream handler.

      // Calculate the updated crc and size before we call
      // the output handler because someday sombody will
      // decide it's a good idea to modify the values
      // in the buffer argument.
      ON__UINT32 out_crc1 = ON_CRC32( m_out_crc, inflate_output_count, out_buffer);
      ON__UINT64 out_size1 = m_out_size + inflate_output_count;
      
      rc = (0 != m_out_callback_function)
          ? m_out_callback_function( m_out_callback_context, inflate_output_count, out_buffer )
          : Out( m_out_callback_context, inflate_output_count, out_buffer );
      if ( !rc )
        break;

      // Update compressed stream crc and size
      m_out_crc = out_crc1;
      m_out_size = out_size1;
      counter = 512; // created output - reset counter that detects stalls
    }

    if ( Z_STREAM_END == zrc )
    {
      // no input left, all pending compressing is finished,
      // and all compressed output has been returned.
      rc = true;
      break;
    }
  }

  strm.avail_in = 0;
  strm.next_in = 0;
  strm.next_out  = 0;
  strm.avail_out = 0;

  inflateEnd(&strm);

  onfree(m_implementation);
  m_implementation = 0;

  return rc;
}

bool ON_UncompressStream::Out( void*, ON__UINT32, const void* )
{
  // default uncompressed stream handler does nothing.
  return true;
}

bool ON_UncompressStream::SetCallback( 
    ON_StreamCallbackFunction out_callback_function,
    void* out_callback_context
    )
{
  m_out_callback_function = out_callback_function;
  m_out_callback_context = out_callback_context;
  return true;
}

ON_StreamCallbackFunction ON_UncompressStream::CallbackFunction() const
{
  return m_out_callback_function;
}

void* ON_UncompressStream::CallbackContext() const
{
  return m_out_callback_context;
}

ON__UINT64 ON_UncompressStream::InSize() const
{
  return m_in_size;
}

ON__UINT64 ON_UncompressStream::OutSize() const
{
  return m_out_size;
}

ON__UINT32 ON_UncompressStream::InCRC() const
{
  return m_in_crc;
}

ON__UINT32 ON_UncompressStream::OutCRC() const
{
  return m_out_crc;
}


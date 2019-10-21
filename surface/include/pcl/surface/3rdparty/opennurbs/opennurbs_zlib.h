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

#if !defined(OPENNURBS_ZLIB_INC_)
#define OPENNURBS_ZLIB_INC_

// If you are using opennurbs as a statically linked library, then
// you may make calls to the same zlib that opennurbs uses.  This
// zlib is compiled with z_ symbol projectection.  All the necessary
// header files are included by opennurbs.h.
// 
// If you are using opennurbs as a DLL or writing a Rhino plug-in
// and you want to use the same zlib that opennurbs uses, then
// compile opennurbs_zlib_memory.cpp into your application
// and statically link with the zlib library. All the necessary
// header files are included by opennurbs.h.


#if !defined(Z_PREFIX)
/* decorates zlib functions with a "z_" prefix to prevent symbol collision. */
#define Z_PREFIX
#endif

#if !defined(MY_ZCALLOC)
/* have zlib use oncalloc() and onfree() for memory managment*/
#define MY_ZCALLOC
#endif

#include "zlib.h"

ON_BEGIN_EXTERNC
voidpf zcalloc (voidpf, unsigned, unsigned);
void  zcfree (voidpf, voidpf);
ON_END_EXTERNC

class ON_CLASS ON_CompressedBuffer
{
public:
  ON_CompressedBuffer();
  ~ON_CompressedBuffer();
  ON_CompressedBuffer(const ON_CompressedBuffer& src);
  ON_CompressedBuffer& operator=(const ON_CompressedBuffer& src);

  /*
  Description:
    Compress inbuffer.
  Parameters:
    sizeof__inbuffer - [in]
       Number of bytes in inbuffer.
    inbuffer - [in]
       Uncompressed information.
    sizeof_element - [out]
       This parameter only matters if the buffer will be compressed,
       and decompressed on CPUs with different endianness.  If this
       is the case, then the types in the buffer need to have the
       same size (2,4, or 8).  
  Returns:
    True if inbuffer is successfully compressed.
  */
  bool Compress(
          std::size_t sizeof__inbuffer,  // sizeof uncompressed input data
          const void* inbuffer,     // uncompressed input data
          int sizeof_element
          );

  /*
  Returns:
    Number of bytes in the uncompressed information.
  */
  std::size_t SizeOfUncompressedBuffer() const;

  /*
  Description:
    Uncompress the contents of this ON_CompressedBuffer.
  Parameters:
    outbuffer - [in/out]
       This buffer must have at least SizeOfUncompressedBuffer() bytes.
       If the function returns true, then the uncopressed information
       is stored in this buffer.
    bFailedCRC - [out]
       If not null, then this boolean is set to true if the CRC 
       of the uncompressed information has changed.
  Returns:
    True if uncompressed information is returned in outbuffer.
  */
  bool Uncompress( // read and uncompress
          void* outbuffer,           // uncompressed output data returned here
          int* bFailedCRC
          ) const;

  /*
  Description:
    Destroy the current informtion in the ON_CompressedBuffer 
    so the class can be reused.
  */
  void Destroy();

  bool Write( ON_BinaryArchive& binary_archive ) const;
  bool Read( ON_BinaryArchive& binary_archive );

  /////////////////////////////////////////////////
  //
  // Implementation
  //
  bool CompressionInit( struct ON_CompressedBufferHelper* ) const;
  bool CompressionEnd( struct ON_CompressedBufferHelper* ) const;
  std::size_t DeflateHelper( // returns number of bytes written
        struct ON_CompressedBufferHelper*,
        std::size_t sizeof___inbuffer,  // sizeof uncompressed input data ( > 0 )
        const void* in___buffer     // uncompressed input data ( != NULL )
        );
  bool InflateHelper(
        struct ON_CompressedBufferHelper*,
        std::size_t sizeof___outbuffer,  // sizeof uncompressed data
        void* out___buffer          // buffer for uncompressed data
        ) const;
  bool WriteChar( 
        std::size_t count, 
        const void* buffer 
        );

  std::size_t     m_sizeof_uncompressed;
  std::size_t     m_sizeof_compressed;
  ON__UINT32 m_crc_uncompressed;
  ON__UINT32 m_crc_compressed;
  int        m_method; // 0 = copied, 1 = compressed
  int        m_sizeof_element;
  std::size_t     m_buffer_compressed_capacity;
  void*      m_buffer_compressed;
};

#endif

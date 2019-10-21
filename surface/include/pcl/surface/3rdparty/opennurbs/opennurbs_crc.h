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

#if !defined(OPENNURBS_CRC_INC_)
#define OPENNURBS_CRC_INC_

ON_BEGIN_EXTERNC

/*
Description:
  Continues 16 bit CRC calulation to include the buffer.

Parameters:
  current_remainder - [in]
  sizeof_buffer - [in]  number of bytes in buffer
  buffer - [in] 

Example:
  16 bit CRC calculations are typically done something like this:

          const ON__UINT16 crc_seed = 0; // or 1, or your favorite starting value

          // Compute CRC on "good" data
          unsigned ON__UINT16 first_crc = crc_seed;
          first_crc = ON_CRC16( first_crc, size1, buffer1 );
          ...
          first_crc = ON_CRC16( first_crc, sizeN, bufferN );
          unsigned char two_zero_bytes[2] = (0,0);
          first_crc = ON_CRC16( first_crc, 2, two_zero_bytes );

          // make sure 16 bit CRC calculation is valid
          ON__UINT16 check_crc_calculation = ON_CRC16( first_crc, 2, &first_crc );
          if ( check_crc_calculation != 0 ) 
          {
             printf("ON_CRC16() calculated a bogus 16 bit CRC\n");
          }

          // Do something that may potentially change the values in
          // the buffers (like storing them on a faulty disk).

          // Compute CRC on "suspect" data
          ON__UINT16 second_crc = crc_seed;
          second_crc = ON_CRC16( second_crc, size1, buffer1 );
          ...
          second_crc = ON_CRC16( second_crc, sizeN, bufferN );
          if ( 0 != ON_CRC16( second_crc, 2, &first_crc ) ) 
          {
            printf( "The value of at least one byte has changed.\n" );
          }
*/
ON_DECL
ON__UINT16 ON_CRC16(
         ON__UINT16 current_remainder,
         std::size_t sizeof_buffer,
         const void* buffer
         );

/*
Description:
  Continues 32 bit CRC calulation to include the buffer

  ON_CRC32() is a slightly altered version of zlib 1.3.3's crc32()
  and the zlib "legal stuff" is reproduced below.

  ON_CRC32() and zlib's crc32() compute the same values.  ON_CRC32() 
  was renamed so it wouldn't clash with the other crc32()'s that are 
  out there and the argument order was switched to match that used by
  the legacy ON_CRC16().

Parameters:
  current_remainder - [in]
  sizeof_buffer - [in]  number of bytes in buffer
  buffer - [in] 

Example:
  32 bit CRC calculations are typically done something like this:

          const ON__UINT32 crc_seed = 0; // or 1, or your favorite starting value

          //Compute CRC on "good" data
          ON__UINT32 first_crc = crc_seed;
          first_crc = ON_CRC32( first_crc, size1, buffer1 );
          ...
          first_crc = ON_CRC32( first_crc, sizeN, bufferN );

          // Do something that may potentially change the values in
          // the buffers (like storing them on a faulty disk).

          // Compute CRC on "suspect" data
          ON__UINT32 second_crc = crc_seed;
          second_crc = ON_CRC32( second_crc, size1, buffer1 );
          ...
          second_crc = ON_CRC32( second_crc, sizeN, bufferN );
          if ( second_crc != first_crc ) 
          {
            printf( "The value of at least one byte has changed.\n" );
          }
*/
ON_DECL
ON__UINT32 ON_CRC32(
         ON__UINT32 current_remainder,
         std::size_t sizeof_buffer,
         const void* buffer
         );

/*
zlib.h -- interface of the 'zlib' general purpose compression library
version 1.1.3, July 9th, 1998

Copyright (C) 1995-1998 Jean-loup Gailly and Mark Adler

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Jean-loup Gailly        Mark Adler
jloup@gzip.org          madler@alumni.caltech.edu

The data format used by the zlib library is described by RFCs (Request for
Comments) 1950 to 1952 in the files ftp://ds.internic.net/rfc/rfc1950.txt
(zlib format), rfc1951.txt (deflate format) and rfc1952.txt (gzip format).

*/

ON_END_EXTERNC

#endif

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

ON__UINT16 ON_CRC16( ON__UINT16 current_remainder, std::size_t count, const void* p )
{
  // 16 bit cyclic redundancy check using CCITT generator polynomial

  // CRC calculations are typically done something like this:
  // 
  // const short crc_seed = 0; // or 1, or your favorite starting value
  //
  // // Compute CRC on "good" data
  // short first_crc = crc_seed;
  // first_crc = ON_CRC16( first_crc, size1, buffer1 );
  // ...
  // first_crc = ON_CRC16( first_crc, sizeN, bufferN );
  //
  //    Do something that may potentially change the values in
  //    the buffers (like storing them on a faulty disk).
  //
  // // Compute CRC on "suspect" data
  // short second_crc = crc_seed;
  // second_crc = ON_CRC16( second_crc, size1, buffer1 );
  // ...
  // second_crc = ON_CRC16( second_crc, sizeN, bufferN );


  // if ( ON_CRC16( second_crc, 2, &first_crc ) ) {
  //   printf( "The value of at least one byte has changed.\n" );
  // }

  static ON__UINT16 ON_CRC16_CCITT_TABLE[256] =
   {0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0};

  const unsigned char* b = (const unsigned char*)p;
  if ( count > 0 && b ) 
  {
    ON__UINT16 r1;
	  // update crc remainder
	  while (count >= 8) 
    {
      // while() loop unrolled for speed
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
      count -= 8;
	  }
	  while (count--) 
    {
		  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
		  current_remainder = (current_remainder << 8) ^ (*b++);
		  current_remainder ^= r1;  
	  }
  }

  return current_remainder;
}

ON__UINT32 ON_CRC32( ON__UINT32 current_remainder, std::size_t count, const void* p )
{
  /*
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  // ON_CRC32() is a slightly altered version of zlib 1.3.3's crc32()
  // and the zlib "legal stuff" is reproduced below.
  //
  // ON_CRC32() and crc32() compute the same values.  ON_CRC32() was renamed
  // so it wouldn't clash with the other crc32()'s that are out there and the
  // argument order was switched to match that used by the legacy ON_CRC16().
  //
  //////////////////////////////////////////////////////////////////////////////////////////

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


  /*
  ON_CRC32_ZLIB_TABLE[] is a table for a byte-wise 32-bit CRC calculation 
  using the generator polynomial:
  x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1.
  */
  static ON__UINT32 ON_CRC32_ZLIB_TABLE[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419,
    0x706af48f, 0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4,
    0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07,
    0x90bf1d91, 0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de,
    0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856,
    0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4,
    0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3,
    0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac, 0x51de003a,
    0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599,
    0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190,
    0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f,
    0x9fbfe4a5, 0xe8b8d433, 0x7807c9a2, 0x0f00f934, 0x9609a88e,
    0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed,
    0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3,
    0xfbd44c65, 0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2,
    0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a,
    0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5,
    0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 0xbe0b1010,
    0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17,
    0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6,
    0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615,
    0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
    0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1, 0xf00f9344,
    0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a,
    0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1,
    0xa6bc5767, 0x3fb506dd, 0x48b2364b, 0xd80d2bda, 0xaf0a1b4c,
    0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef,
    0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe,
    0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31,
    0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c,
    0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b,
    0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1,
    0x18b74777, 0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c,
    0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45, 0xa00ae278,
    0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7,
    0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 0x40df0b66,
    0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605,
    0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8,
    0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b,
    0x2d02ef8d
  };

  if ( count > 0 && p ) 
  {
    const unsigned char* b = (const unsigned char*)p;

    // The trailing L was needed long ago when "int" was often a 16 bit integers.
    // Today it is more common for "int" to be a 32 bit integer and
    // L to be a 32 or 64 bit integer.  So, the L is causing more
    // problems that it is fixing.
    // current_remainder ^= 0xffffffffL; 
    current_remainder ^= 0xffffffff;
    while (count >= 8)
    {
      // while() loop unrolled for speed
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
      count -= 8;
    }
    while(count--) 
    {
      current_remainder = ON_CRC32_ZLIB_TABLE[((int)current_remainder ^ (*b++)) & 0xff] ^ (current_remainder >> 8);
    }
    // The trailing L was needed long ago when "int" was often a 16 bit integers.
    // Today it is more common for "int" to be a 32 bit integer and
    // L to be a 32 or 64 bit integer.  So, the L is causing more
    // problems than it is fixing.
    //current_remainder ^= 0xffffffffL;
    current_remainder ^= 0xffffffff;
  }

  return current_remainder;
}




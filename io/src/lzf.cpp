/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2000-2010 Marc Alexander Lehmann <schmorp@schmorp.de>
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/io/lzf.h>
#include <cstring>
#include <climits>
#include <pcl/console/print.h>
#include <errno.h>

/*
 * Size of hashtable is (1 << HLOG) * sizeof (char *)
 * decompression is independent of the hash table size
 * the difference between 15 and 14 is very small
 * for small blocks (and 14 is usually a bit faster).
 * For a low-memory/faster configuration, use HLOG == 13;
 * For best compression, use 15 or 16 (or more, up to 22).
 */
#define HLOG 13

typedef unsigned int LZF_HSLOT;
typedef unsigned int LZF_STATE[1 << (HLOG)];

#define STRICT_ALIGN !(defined(__i386) || defined (__amd64))
#if !STRICT_ALIGN
/* for unaligned accesses we need a 16 bit datatype. */
# if USHRT_MAX == 65535
    typedef unsigned short u16;
# elif UINT_MAX == 65535
    typedef unsigned int u16;
# else
#  undef STRICT_ALIGN
#  define STRICT_ALIGN 1
# endif
#endif

// IDX works because it is very similar to a multiplicative hash, e.g.
// ((h * 57321 >> (3*8 - HLOG)) & ((1 << (HLOG)) - 1))
#define IDX(h) ((( h >> (3*8 - HLOG)) - h  ) & ((1 << (HLOG)) - 1))

///////////////////////////////////////////////////////////////////////////////////////////
//
// compressed format
//
// 000LLLLL <L+1>    ; literal, L+1=1..33 octets
// LLLooooo oooooooo ; backref L+1=1..7 octets, o+1=1..4096 offset
// 111ooooo LLLLLLLL oooooooo ; backref L+8 octets, o+1=1..4096 offset
//
//
unsigned int
pcl::lzfCompress (const void *const in_data, unsigned int in_len,
                  void *out_data, unsigned int out_len)
{
  LZF_STATE htab;
  const unsigned char *ip = static_cast<const unsigned char *> (in_data);
        unsigned char *op = static_cast<unsigned char *> (out_data);
  const unsigned char *in_end  = ip + in_len;
        unsigned char *out_end = op + out_len;
  const unsigned char *ref;

  // off requires a type wide enough to hold a general pointer difference.
  // ISO C doesn't have that (size_t might not be enough and ptrdiff_t only
  // works for differences within a single object). We also assume that no
  // no bit pattern traps. Since the only platform that is both non-POSIX
  // and fails to support both assumptions is windows 64 bit, we make a
  // special workaround for it.
#if defined (WIN32) && defined (_M_X64) && defined (_MSC_VER)
  // workaround for missing POSIX compliance
  unsigned _int64 off; 
#else
  unsigned long off;
#endif
  unsigned int hval;
  int lit;

  if (!in_len || !out_len)
  {
    PCL_WARN ("[pcl::lzf_compress] Input or output has 0 size!\n");
    return (0);
  }

  // Initialize the htab
  memset (htab, 0, sizeof (htab));

  // Start run
  lit = 0; op++;

  hval = (ip[0] << 8) | ip[1];
  while (ip < in_end - 2)
  {
    unsigned int *hslot;

    hval = (hval << 8) | ip[2];
    hslot = htab + IDX (hval);
    ref = *hslot + (static_cast<const unsigned char*> (in_data)); 
    *hslot = static_cast<unsigned int> (ip - (static_cast<const unsigned char*> (in_data)));

    if (
        // The next test will actually take care of this, but this is faster if htab is initialized
        ref < ip 
        && (off = ip - ref - 1) < (1 << 13)
        && ref > static_cast<const unsigned char *> (in_data)
        && ref[2] == ip[2]
#if STRICT_ALIGN
        && ((ref[1] << 8) | ref[0]) == ((ip[1] << 8) | ip[0])
#else
        && *reinterpret_cast<const u16 *> (ref) == *reinterpret_cast<const u16 *> (ip)
#endif
      )
    {
      // Match found at *ref++
      unsigned int len = 2;
      ptrdiff_t maxlen = in_end - ip - len;
      maxlen = maxlen > ((1 << 8) + (1 << 3)) ? ((1 << 8) + (1 << 3)) : maxlen;

      // First a faster conservative test
      if (op + 3 + 1 >= out_end)
      {
        // Second the exact but rare test
        if (op - !lit + 3 + 1 >= out_end)
        {
          PCL_WARN ("[pcl::lzf_compress] Attempting to write data outside the output buffer!\n");
          return (0);
        }
      }

      // Stop run
      op [- lit - 1] = static_cast<unsigned char>(lit - 1);
      // Undo run if length is zero
      op -= !lit;

      while (true)
      {
        if (maxlen > 16)
        {
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;

          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;

          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;

          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
          len++; if (ref [len] != ip [len]) break;
        }

        do
          len++;
        while (len < static_cast<unsigned int> (maxlen) && ref[len] == ip[len]);

        break;
      }

      // Len is now #octets - 1
      len -= 2;
      ip++;

      if (len < 7)
      {
        *op++ = static_cast<unsigned char> ((off >> 8) + (len << 5));
      }
      else
      {
        *op++ = static_cast<unsigned char> ((off >> 8) + (  7 << 5));
        *op++ = static_cast<unsigned char> (len - 7);
      }

      *op++ = static_cast<unsigned char> (off);

      // Start run
      lit = 0; op++;

      ip += len + 1;

      if (ip >= in_end - 2)
        break;

      --ip;
      hval = (ip[0] << 8) | ip[1];

      hval = (hval << 8) | ip[2];
      htab[IDX (hval)] = static_cast<unsigned int> (ip - (static_cast<const unsigned char *> (in_data)));
      ip++;
    }
    else
    {
      // One more literal byte we must copy
      if (op >= out_end)
      {
        PCL_WARN ("[pcl::lzf_compress] Attempting to copy data outside the output buffer!\n");
        return (0);
      }

      lit++; *op++ = *ip++;

      if (lit == (1 <<  5))
      {
        // Stop run
        op [- lit - 1] = static_cast<unsigned char> (lit - 1);
        // Start run
        lit = 0; op++;
      }
    }
  }

  // At most 3 bytes can be missing here 
  if (op + 3 > out_end)
    return (0);

  while (ip < in_end)
  {
    lit++; *op++ = *ip++;

    if (lit == (1 <<  5))
    {
      // Stop run
      op [- lit - 1] = static_cast<unsigned char> (lit - 1);
      // Start run
      lit = 0; op++;
    }
  }

  // End run
  op [- lit - 1] = static_cast<unsigned char> (lit - 1);
  // Undo run if length is zero
  op -= !lit;

  return (static_cast<unsigned int> (op - static_cast<unsigned char *> (out_data)));
}

///////////////////////////////////////////////////////////////////////////////////////////
unsigned int 
pcl::lzfDecompress (const void *const in_data,  unsigned int in_len,
                    void             *out_data, unsigned int out_len)
{
  unsigned char const *ip = static_cast<const unsigned char *> (in_data);
  unsigned char       *op = static_cast<unsigned char *> (out_data);
  unsigned char const *const in_end  = ip + in_len;
  unsigned char       *const out_end = op + out_len;

  do
  {
    unsigned int ctrl = *ip++;

    // Literal run
    if (ctrl < (1 << 5))
    {
      ctrl++;

      if (op + ctrl > out_end)
      {
        errno = E2BIG;
        return (0);
      }

      // Check for overflow
      if (ip + ctrl > in_end)
      {
        errno = EINVAL;
        return (0);
      }
      switch (ctrl)
      {
        case 32: *op++ = *ip++; case 31: *op++ = *ip++; case 30: *op++ = *ip++; case 29: *op++ = *ip++;
        case 28: *op++ = *ip++; case 27: *op++ = *ip++; case 26: *op++ = *ip++; case 25: *op++ = *ip++;
        case 24: *op++ = *ip++; case 23: *op++ = *ip++; case 22: *op++ = *ip++; case 21: *op++ = *ip++;
        case 20: *op++ = *ip++; case 19: *op++ = *ip++; case 18: *op++ = *ip++; case 17: *op++ = *ip++;
        case 16: *op++ = *ip++; case 15: *op++ = *ip++; case 14: *op++ = *ip++; case 13: *op++ = *ip++;
        case 12: *op++ = *ip++; case 11: *op++ = *ip++; case 10: *op++ = *ip++; case  9: *op++ = *ip++;
        case  8: *op++ = *ip++; case  7: *op++ = *ip++; case  6: *op++ = *ip++; case  5: *op++ = *ip++;
        case  4: *op++ = *ip++; case  3: *op++ = *ip++; case  2: *op++ = *ip++; case  1: *op++ = *ip++;
      }
    }
    // Back reference
    else
    {
      unsigned int len = ctrl >> 5;

      unsigned char *ref = op - ((ctrl & 0x1f) << 8) - 1;

      // Check for overflow
      if (ip >= in_end)
      {
        errno = EINVAL;
        return (0);
      }
      if (len == 7)
      {
        len += *ip++;
        // Check for overflow
        if (ip >= in_end)
        {
          errno = EINVAL;
          return (0);
        }
      }
      ref -= *ip++;

      if (op + len + 2 > out_end)
      {
        errno = E2BIG;
        return (0);
      }

      if (ref < static_cast<unsigned char *> (out_data))
      {
        errno = EINVAL;
        return (0);
      }

      switch (len)
      {
        default:
        {
          len += 2;

          if (op >= ref + len)
          {
            // Disjunct areas
            memcpy (op, ref, len);
            op += len;
          }
          else
          {
            // Overlapping, use byte by byte copying
            do
              *op++ = *ref++;
            while (--len);
          }

          break;
        }
        case 9: *op++ = *ref++;
        case 8: *op++ = *ref++;
        case 7: *op++ = *ref++;
        case 6: *op++ = *ref++;
        case 5: *op++ = *ref++;
        case 4: *op++ = *ref++;
        case 3: *op++ = *ref++;
        case 2: *op++ = *ref++;
        case 1: *op++ = *ref++;
        case 0: *op++ = *ref++; // two octets more
                *op++ = *ref++;
      }
    }
  }
  while (ip < in_end);

  return (static_cast<unsigned int> (op - static_cast<unsigned char*> (out_data)));
}


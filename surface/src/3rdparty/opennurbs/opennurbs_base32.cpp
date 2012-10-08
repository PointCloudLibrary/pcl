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

bool ON_Base32ToString( const ON_SimpleArray<unsigned char>& base32_digits, ON_String& sBase32 )
{
  int digit_count = base32_digits.Count();
  sBase32.ReserveArray(digit_count);
  sBase32.SetLength(digit_count);
  bool rc = ON_Base32ToString( base32_digits, digit_count, sBase32.Array() );
  if (!rc)
    sBase32.SetLength(0);
  return rc;
}

bool ON_Base32ToString( const ON_SimpleArray<unsigned char>& base32_digits, ON_wString& sBase32 )
{
  ON_String s;
  bool rc = ON_Base32ToString( base32_digits, s );
  if (rc)
    sBase32 = s;
  return rc;
}

bool ON_Base32ToString( const unsigned char* base32_digits, int base32_digit_count, char* sBase32 )
{
  const char* base32_digit_symbol = "0123456789ABCDEFGHJKMNPQRTUVWXYZ";
  const char error_symbol = '#';
  unsigned char d;
  bool rc = false;

  if ( 0 == sBase32 )
    return false;

  if ( 0 == base32_digits || base32_digit_count <= 0 )
  {
    *sBase32++ = error_symbol;
  }
  else
  {
    rc = true;
    while(base32_digit_count--)
    {
      d = *base32_digits++;
      if ( d < 32 )
      {
        *sBase32++ = base32_digit_symbol[d];
      }
      else
      {
        rc = false;
        *sBase32++ = error_symbol;
      }
    }
  }
  *sBase32 = 0; // NULL terminate string

  return rc;
}

int ON_CorrectBase32StringTypos( const char* sBase32, ON_String& sBase32clean )
{
  char* sClean = 0;
  if ( sBase32 == sBase32clean.Array() )
    sClean = sBase32clean.Array();
  else
  {
    sBase32clean.SetLength(0);
    sBase32clean.ReserveArray(strlen(sBase32));
    sClean = sBase32clean.Array();
  }
  int length = ON_CorrectBase32StringTypos( sBase32, sClean );
  sBase32clean.SetLength(length);
  return length;
}

int ON_CorrectBase32StringTypos( const wchar_t* sBase32, ON_wString& sBase32clean )
{
  if ( 0 == sBase32 || 0 == sBase32[0] )
    return 0;
  ON_String s = sBase32;
  int length = ON_CorrectBase32StringTypos(s.Array(),s.Array());
  if ( length > 0 )
    sBase32clean = s;
  else
    sBase32clean.SetLength(0);
  return length;
}

int ON_CorrectBase32StringTypos( const char* sBase32, char* sBase32clean )
{
  char c;
  int length = 0;
  if ( 0 == sBase32clean )
    return 0;
  if (0 != sBase32 )
  {
    while ( 0 != (c = *sBase32++) )
    {
      if ( c >= '0' && c <= '9' )
      {
        sBase32clean[length++] = c;
      }
      else
      {
        if ( c >= 'a' && c < 'z' )
          c -= 'a'-'A';

        if ( 'I' == c || 'L' == c )
          c = '1';
        else if ('O' == c )
          c = '0';
        else if ( 'S' == c )
          c = '5';
        else if ( c < 'A' || c > 'Z' )
        {
          length = 0;
          break;
        }

        sBase32clean[length++] = c;
      }
    }
  }
  sBase32clean[length] = 0;
  return length;
}

int ON_StringToBase32(const ON_wString& sBase32, ON_SimpleArray<unsigned char>& base32_digits )
{
  ON_String s(sBase32);
  return ON_StringToBase32(s,base32_digits);
}

int ON_StringToBase32(const ON_String& sBase32, ON_SimpleArray<unsigned char>& base32_digits )
{
  const char* s = sBase32;
  if ( 0 == s || 0 == s[0] )
    return 0;
  base32_digits.Reserve(sBase32.Length());
  int digit_count = ON_StringToBase32(sBase32,base32_digits.Array());
  base32_digits.SetCount(digit_count);
  return digit_count;
}

int ON_StringToBase32(const char* sBase32, unsigned char* base32_digits )
{
  char c;
  int digit_count = 0;
  if ( 0 == base32_digits )
    return 0;
  if ( 0 != sBase32 )
  {
    while ( 0 != (c = *sBase32++) )
    {
      if (c >= '0' && c <= '9' )
        base32_digits[digit_count++] = c - '0';
      else if ( c >= 'A' && c <= 'H' )
        base32_digits[digit_count++] = 10 + c - 'A';
      else if ( c >= 'J' && c <= 'K' )
        base32_digits[digit_count++] = 9 + c - 'A';
      else if ( c >= 'M' && c <= 'N' )
        base32_digits[digit_count++] = 8 + c - 'A';
      else if ( c >= 'P' && c <= 'R' )
        base32_digits[digit_count++] = 7 + c - 'A';
      else if ( c >= 'T' && c <= 'Z' )
        base32_digits[digit_count++] = 6 + c - 'A';
      else
      {
        digit_count = 0;
        break;
      }
    }
  }  
  return digit_count;
}

int ON_GetBase32Digits( const ON_SimpleArray<unsigned char>& x, ON_SimpleArray<unsigned char>& base32_digits )
{
  int x_count = x.Count();
  int bit_count = 8*x_count;
  int base32_digit_count = (bit_count/5) + ((bit_count%5)?1:0);
  base32_digits.Reserve(base32_digit_count);
  base32_digit_count = ON_GetBase32Digits( x.Array(), x_count, base32_digits.Array() );
  base32_digits.SetCount(base32_digit_count);
  return base32_digit_count;  
}

int ON_GetBase32Digits( const unsigned char* x, int x_count, unsigned char* base32_digits )
{
  int x_bit_count = 8*x_count;

  unsigned char mask, c;
  unsigned char bits[5] = {0,0,0,0,0};
  unsigned int bits_count = 0;
  unsigned int base32_digit_count = 0;
  int i;

  if ( 0 == base32_digits || 0 == x || x_count <= 0 )
    return 0;

  if ( x == base32_digits )
  {
    unsigned char* tmp = (unsigned char*)onmalloc(x_count*sizeof(x[0]));
    if ( 0 == tmp )
      return 0;
    memcpy(tmp,x,x_count*sizeof(x[0]));
    i = ON_GetBase32Digits(tmp,x_count,base32_digits);    
    onfree(tmp);
    return i;
  }

  i = x_bit_count % 5;
  if ( i )
    bits_count = 5-i;

  for ( i = 0; i < x_count; i++)
  {
    c = x[i];
    for (mask = 128; 0 != mask; mask /= 2 )
    {
      bits[bits_count++] = (0 != (c & mask)) ? 1 : 0;
      if ( 5 == bits_count )
      {
        base32_digits[base32_digit_count++] = 16*bits[0] + 8*bits[1] + 4*bits[2] + 2*bits[3] + bits[4];
        bits_count = 0;
      }
    }
  }

  return base32_digit_count;
}


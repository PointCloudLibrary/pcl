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

#if !defined(ON_BASE32_INC_)
#define ON_BASE32_INC_


/*
Description:
  Convert a number into base32 digits.
Parameters:
  x - [in]
  x_count - [in]
    x[] is an array of length x_count and represents the value
    x[0]*2^(8*(x_count-1)) + ... + x[x_count-2]*256 + x[x_count-1].
  base32_digits - [out]
    When base32_digits is not a dynamic array, base32_digits[] 
    must a be an array of length at least
    ((8*x_count)/5) + (((8*x_count)%5)?1:0) or 1, 
    whichever is greater.

    The base32_digits[] array will be filled in with base32 digit 
    values (0 to 31) so that the value
    b[0]*32^(b_count-1) + ... + b[b_count-2]*32 + b[b_count-1]
    is the same as that defined by the x[] array.
Returns
  The number of base 32 digits in the base32_digits[] array.
  If 0 is returned, the input is not valid.
*/
ON_DECL
int ON_GetBase32Digits( const ON_SimpleArray<unsigned char>& x, ON_SimpleArray<unsigned char>& base32_digits );
ON_DECL
int ON_GetBase32Digits( const unsigned char* x, int x_count, unsigned char* base32_digits );


/*
Description:
  Convert a list of base32 digits into a string form.
Parameters:
  base32_digits - [in]
  base32_digit_count - [in]
    base32_digits[] is an array of length base32_digit_count. 
    Each element is in the range 0 to 31.
  sBase32 - [out]
    sBase32[] must be an array of length base32_digit_count+1 or 2,
    whichever is greater. The string representation of the base 32
    number will be put in this string. A hash mark symbol (#) is
    used to indicate an error in the input value.  The returned
    string is null terminated.
Returns
  True if the input is valid.  False if the input is not valid,
  in which case hash marks indicate the invalid entries.
*/
ON_DECL
bool ON_Base32ToString( const ON_SimpleArray<unsigned char>& base32_digits, ON_String& sBase32 );
ON_DECL
bool ON_Base32ToString( const ON_SimpleArray<unsigned char>& base32_digits, ON_wString& sBase32 );
ON_DECL
bool ON_Base32ToString( const unsigned char* base32_digits, int base32_digit_count, char* sBase32 );


/*
Description:
  Fixt a common typos in sBase32 string.  Lower case letters are
  converted to upper case. The letters 'I', 'L', 'O' and 'S' are
  converted to '1' (one), '1' (one) '0' zero and '5' (five).
Parameters:
  sBase32 - [in]
  sBase32clean - [out]
    (can be the same string as sBase32)
Returns:
  If the input is valid, the length of the converted string is returned.
  If the input is not valid, 0 is returned.
*/
ON_DECL
int ON_CorrectBase32StringTypos( const wchar_t* sBase32, ON_wString& sBase32clean );
ON_DECL
int ON_CorrectBase32StringTypos( const char* sBase32, ON_String& sBase32clean );
ON_DECL
int ON_CorrectBase32StringTypos( const char* sBase32, char* sBase32clean );


/*
Description:
  Convert a null terminate string containing the 32 symbols 

  0 1 2 3 4 5 6 7 8 9 A B C D E F G H J K M N P Q R T U V W X Y Z

  (I,L,O and S are missing) into a list of base 32 digits.
Parameters:
  sBase32 - [in]
    String with base 32 digits
  base32_digits - [out]
    base32_digits[] is an array of length strlen(sBase32). 
    The returned array, element will be in the range 0 to 31.
    sBase32[] must be an array of length base32_digit_count+1 or 2,
    whichever is greater. The string representation of the base 32
    number will be put in this string. A hash mark symbol (#) is
    used to indicate an error in the input value.  The returned
    string is null terminated.
Returns
  True if the input is valid.  False if the input is not valid,
  in which case hash marks indicate the invalid entries.
*/
ON_DECL
int ON_StringToBase32(const ON_wString& sBase32, ON_SimpleArray<unsigned char>& base32_digits );
ON_DECL
int ON_StringToBase32(const ON_String& sBase32, ON_SimpleArray<unsigned char>& base32_digits );
ON_DECL
int ON_StringToBase32(const char* sBase32, unsigned char* base32_digits );


#endif

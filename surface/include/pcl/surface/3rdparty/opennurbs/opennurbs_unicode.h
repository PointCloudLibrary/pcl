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

#if !defined(OPENNURBS_UNICODE_INC_)
#define OPENNURBS_UNICODE_INC_

ON_BEGIN_EXTERNC

struct ON_UnicodeErrorParameters
{
  /*
  If an error occurs, then bits of error_status are
  set to indicate what type of error occured.

  Error types:
    1: The input parameters were invalid. 
       This error cannot be masked.

    2: The output buffer was not large enough to hold the converted
       string. As much conversion as possible is performed in this
       case and the error cannot be masked.

    4: When parsing a UTF-8 or UTF-32 string, the values of two
       consecutive encoding sequences formed a valid UTF-16 
       surrogate pair. 
          
       This error is masked if 0 != (4 & m_error_mask).
       If the error is masked, then the surrogate pair is
       decoded, the value of the resulting unicode code point
       is used, and parsing continues.

    8: An overlong UTF-8 encoding sequence was encountered and 
       the value of the overlong sUTF-8 equence was a valid unicode
       code point. 
          
       This error is masked if 0 != (8 & m_error_mask).
       If the error is masked, then the unicode code point is 
       used and parsing continues.

   16: An illegal UTF-8 encoding sequence occured or an invalid
       unicode code point value resulted from decoding a
       UTF-8 sequence. 

       This error is masked if 0 != (16 & m_error_mask).
       If the error is masked and the value of m_error_code_point is
       a valid unicode code point, then m_error_code_point is used
       and parsing continues.
  */
  unsigned int m_error_status;

  /*
  If 0 != (error_mask & 4), then type 4 errors are masked.
  If 0 != (error_mask & 8), then type 8 errors are masked.
  If 0 != (error_mask & 16) and m_error_code_point is a valid unicode
  code point value, then type 16 errors are masked.
  */
  unsigned int m_error_mask;

  /*
  Unicode code point value to use in when masking type 16 errors.
  If 0 == (error_mask & 16), then this parameter is ignored.
  0xFFFD is a popular choice for the m_error_code_point value.
  */
  ON__UINT32 m_error_code_point;
};


/*
Description:
  Test a value to determine if it is a valid unicode code point value.
Parameters:
  u - [in] value to test
Returns:
  true: u is a valid unicode code point
  false: u is not a valid unicode code point
Remarks:
  Valid unicode code points are 
  (0 <= u && u <= 0xD7FF) || (0xE000 <= u && u <= 0x10FFFF)
*/
ON_DECL
int ON_IsValidUnicodeCodePoint( ON__UINT32 u );

/*
Description:
  Convert an integer to its UTF-8 form.
Parameters:
  u - [in]
    Interger in the CPU's native byte order that can be
    converted to UTF-8 form.
    Valid values are in the interval [0,2147483647].
  sUTF8 - [out]
    sUTF8 is a buffer of 6 ON__UINT8 elements and the UTF-8 form
    is returned in sUTF8[]. The returned value specifies how 
    many elements of sUTF8[] are set.
Returns:
  0: u is too large (>=2^31) to be encode as a UTF-8 string.
     No changes are made to the sUTF8[] values.
  1: the UTF-8 form of u is 1 byte returned in sUTF8[0].
  2: the UTF-8 form of u is 2 byts returned in sUTF8[0],sUTF8[1].
  3: the UTF-8 form of u is 3 bytes returned in sUTF8[0],sUTF8[1],sUTF8[2].
  4: the UTF-8 form of u is 4 bytes returned in sUTF8[0],sUTF8[1],sUTF8[2],sUTF8[3].
  5: the UTF-8 form of u is 5 bytes returned in sUTF8[0],sUTF8[1],sUTF8[2],sUTF8[3],sUTF8[4].
  6: the UTF-8 form of u is 6 bytes returned in sUTF8[0],sUTF8[1],sUTF8[2],sUTF8[3],sUTF8[4],sUTF8[5].
  For return values requiring less than 6 bytes, no changes
  are made to the unused bytes in sUTF8[].
Remarks:
  Any integer in the range 0 to 2^31 - 1 can be encoded as a UTF-8 string.
  When a unicode string is being encoded take steps to ensure that
  u is a valid unicode code point value.  The function ON_IsValidUnicodeCodePoint()
  can be used to determine if u is a valid unicode code point value.
*/
ON_DECL
int ON_EncodeUTF8( ON__UINT32 u, ON__UINT8 sUTF8[6] );

/*
Description:
  Decode a UTF-8 encode string to get a single unicode code point.
Parameters:
  sUTF8 - [in]
    UTF-8 string to convert.

  sUTF8_count - [in]
    number of ON__UINT8 elements in sUTF8[].

   e - [in/out] 
    If e is null, errors are not masked and parsing is performed
    to the point where the first error occurs.
    If e is not null, all errors are reported by setting the appropriate
    e->m_error_status bits and errors are handled as described in the
    definition of the ON_UnicodeErrorParameters struct.

  unicode_code_point - [out]
    The unicode_code_point pointer must not be null.
    If a nonzero value is returned, then *unicode_code_point is
    a valid unicode code point value.
Returns:
  Number of elements of sUTF8 that were parsed.
  0 indicates failure.
*/
ON_DECL
int ON_DecodeUTF8(
    const ON__UINT8* sUTF8,
    int sUTF8_count,
    struct ON_UnicodeErrorParameters* e,
    ON__UINT32* unicode_code_point
    );

/*
Description:
  Convert a 4 byte unicode code point value to its UTF-16 form.
Parameters:
  unicode_code_point - [in]
    4 byte unicode code point value in the CPU's native byte order.
    Valid values are in the interval [0,0xD7FF] or the 
    interval [0xE000,0x10FFFF].
  sUTF16 - [out]
    sUTF16 is buffer of 2 ON__UINT16 elements. If the UTF-16 form
    is a single value, it is returned in sUTF16[0]. If the UTF-16
    is a surrogate pair, the first code unit (high surrogate) 
    is returned sUTF16[0] and the second unit (low surrogate) is
    returned in sUTF16[1].  The returned values are in
    the CPU's native byte order.
Returns:
  0: u is not a valid Unicode code point. No changes are
     made to the w[] values.
  1: u is a valie Unicode code point with a UTF-16 form 
     consisting of the single value returned in w[0].
  2: u is a valid Unicode code point with a UTF-16 form 
     consisting of a surrogate pair returned in w[0] and w[1].
*/
ON_DECL
int ON_EncodeUTF16( ON__UINT32 unicode_code_point, ON__UINT16 sUTF16[2] );

/*
Description:
  Decode a UTF-16 string to get a single unicode code point.
Parameters:
  sUTF16 - [in]
    UTF-16 string to convert.

  sUTF16_count - [in]
    number of ON__UINT16 elements in sUTF16[].

  e - [in/out] 
    If e is null, errors are not masked and parsing is performed
    to the point where the first error occurs.
    If e is not null, all errors are reported by setting the appropriate
    e->m_error_status bits and errors are handled as described in the
    definition of the ON_UnicodeErrorParameters struct.

  unicode_code_point - [out]
    The unicode_code_point pointer must not be null.
    If a nonzero value is returned, then *unicode_code_point is
    a valid unicode code point value in the CPU's native byte order.
Returns:
  Number of elements of sUTF16 that were parsed.
  0 indicates failure.
*/
ON_DECL
int ON_DecodeUTF16(
    const ON__UINT16* sUTF16,
    int sUTF16_count,
    struct ON_UnicodeErrorParameters* e,
    ON__UINT32* unicode_code_point
    );

/*
Description:
  Decode a UTF-16 encode string whose elements have byte order
  opposite the native CPU's to get a single unicode code point.
Parameters:
  sUTF16 - [in]
    UTF-16 string to convert with byte order opposite the
    CPU's native byte order.

  sUTF16_count - [in]
    number of ON__UINT16 elements in sUTF16[].

  e - [in/out] 
    If e is null, errors are not masked and parsing is performed
    to the point where the first error occurs.
    If e is not null, all errors are reported by setting the appropriate
    e->m_error_status bits and errors are handled as described in the
    definition of the ON_UnicodeErrorParameters struct.

  unicode_code_point - [out]
    The unicode_code_point pointer must not be null.
    If a nonzero value is returned, then *unicode_code_point is
    a valid unicode code point value in the CPU's native byte order.
Returns:
  Number of elements of sUTF16 that were parsed.
  0 indicates failure.
*/
ON_DECL
int ON_DecodeSwapByteUTF16(
    const ON__UINT16* sUTF16,
    int sUTF16_count,
    struct ON_UnicodeErrorParameters* e,
    ON__UINT32* unicode_code_point
    );

/*
Description:
  Convert a unicode string from a UTF-8 encoded ON__UINT8 array
  into a UTF-16 encoded ON__UINT16 array.

Parameters:
  sUTF8 - [in]
    UTF-8 string to convert.

  sUTF8_count - [in]
    If sUTF8_count >= 0, then it specifies the number of
    ON__UINT8 elements in sUTF8[] to convert.

    If sUTF8_count == -1, then sUTF8 must be a null terminated
    string and all the elements up to the first null element are
    converted.

  sUTF16 - [out]
    If sUTF16 is not null and sUTF16_count > 0, then the UTF-16
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF16_count - [in]
    If sUTF16_count > 0, then it specifies the number of available
    ON__UINT16 elements in the sUTF16[] buffer.
    
    If sUTF16_count == 0, then the sUTF16 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF16 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
       4: The values of two UTF-8 encoding sequences formed a valid
          UTF-16 surrogate pair. This error can be masked.  If the
          error is masked, then the surrogate pair is added
          to the UTF-16 output string and parsing continues.
       8: An overlong UTF-8 encoding sequence was encountered. 
          The value of the overlong sequence was a valid unicode
          code point. This error can be masked. If the error is masked,
          then the unicode code point is encoded and added to the
          UTF-16 output string and parsing continues.
      16: An illegal UTF-8 encoding sequence occured or an invalid
          unicode code point value resulted from decoding a
          UTF-8 sequence. This error can be masked. If the error is
          masked and error_code_point is a valid unicode code point,
          then its UTF-16 encoding is added to the UTF-16 output
          string and parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 4), then type 4 errors are masked.
    If 0 != (error_mask & 8), then type 8 errors are masked.
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUTF8 - [out]
    If sNextUTF8 is not null, then *sNextUTF8 points to the first
    element in the input sUTF8[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextUTF8 points to
    the element of sUTF8[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextUTF8 points to
    sUTF8 + sUTF8_count.

Returns:
  If sUTF16_count > 0, the return value is the number of ON__UINT16
  elements written to sUTF16[].  When the return value < sUTF16_count,
  a null terminator is written to sUTF16[return value].

  If sUTF16_count == 0, the return value is the minimum number of
  ON__UINT16 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF8ToUTF16(
    const ON__UINT8* sUTF8,
    int sUTF8_count,
    ON__UINT16* sUTF16,
    int sUTF16_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const ON__UINT8** sNextUTF8
    );

/*
Description:
  Convert a unicode string from a UTF-8 encoded ON__UINT8 array
  into a UTF-32 encoded ON__UINT32 array.

Parameters:
  sUTF8 - [in]
    UTF-8 string to convert.

  sUTF8_count - [in]
    If sUTF8_count >= 0, then it specifies the number of
    ON__UINT8 elements in sUTF8[] to convert.

    If sUTF8_count == -1, then sUTF8 must be a null terminated
    string and all the elements up to the first null element are
    converted.

  sUTF32 - [out]
    If sUTF32 is not null and sUTF32_count > 0, then the UTF-32
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF32_count - [in]
    If sUTF32_count > 0, then it specifies the number of available
    ON__UINT32 elements in the sUTF32[] buffer.
    
    If sUTF32_count == 0, then the sUTF32 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF32 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
       4: The values of two UTF-8 encoding sequences formed a valid
          UTF-16 surrogate pair. This error can be masked.  If the
          error is masked, then the surrogate pair is decoded,
          the code point value is added to the UTF-32 output 
          string and parsing continues.
       8: An overlong UTF-8 encoding sequence was encountered. 
          The value of the overlong sequence was a valid unicode
          code point. This error can be masked. If the error is masked,
          then the unicode code point is added to the UTF-32
          output string and parsing continues.
      16: An illegal UTF-8 encoding sequence occured or an invalid
          unicode code point value resulted from decoding a
          UTF-8 sequence. This error can be masked. If the error is
          masked and error_code_point is a valid unicode code point,
          then its value is added to the UTF-32 output string and 
          parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 4), then type 4 errors are masked.
    If 0 != (error_mask & 8), then type 8 errors are masked.
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUTF8 - [out]
    If sNextUTF8 is not null, then *sNextUTF8 points to the first
    element in the input sUTF8[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextUTF8 points to
    the element of sUTF8[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextUTF8 points to
    sUTF8 + sUTF8_count.

Returns:
  If sUTF32_count > 0, the return value is the number of ON__UINT32
  elements written to sUTF32[].  When the return value < sUTF32_count,
  a null terminator is written to sUTF32[return value].

  If sUTF32_count == 0, the return value is the minimum number of
  ON__UINT32 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF8ToUTF32(
    const ON__UINT8* sUTF8,
    int sUTF8_count,
    ON__UINT32* sUTF32,
    int sUTF32_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const ON__UINT8** sNextUTF8
    );

/*
Description:
  Convert a unicode string from a UTF-16 encoded ON__UINT16 array
  into a UTF-8 encoded ON__UINT8 array.

Parameters:
  bTestByteOrder - [in]
    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFEFF, then this element is ignored.

    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFFFE, then this element is ignored and the subsequent
    elements of sUTF16[] have their bytes swapped before the 
    conversion is calculated.

    In all other cases the first element of sUTF16[] is 
    converted and no byte swapping is performed.

  sUTF16 - [in]
    UTF-16 string to convert.  
    
    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFEFF, then this element is skipped and it is assumed 
    that sUTF16[] is in the CPU's native byte order.
    
    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFFFE, then this element is skipped and it is assumed 
    that sUTF16[] is not in the CPU's native byte order and bytes
    are swapped before characters are converted.

    If bTestByteOrder is false or the first character of sUTF16[]
    is neither 0xFEFF nor 0xFFFE, then the sUTF16 string must match
    the CPU's byte order.

  sUTF16_count - [in]
    If sUTF16_count >= 0, then it specifies the number of
    ON__UINT16 elements in sUTF16[] to convert.

    If sUTF16_count == -1, then sUTF16 must be a null terminated
    string and all the elements up to the first null element are
    converted.
    
  sUTF8 - [out]
    If sUTF8 is not null and sUTF8_count > 0, then the UTF-8
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF8_count - [in]
    If sUTF8_count > 0, then it specifies the number of available
    ON__UINT8 elements in the sUTF8[] buffer.
    
    If sUTF8_count == 0, then the sUTF8 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF8 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
      16: An illegal UTF-16 encoding sequence occured or an invalid
          unicode code point value resulted from decoding a
          UTF-16 sequence. This error can be masked. If the error is
          masked and error_code_point is a valid unicode code point,
          then its UTF-8 encoding is added to the UTF-8 output
          string and parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUTF16 - [out]
    If sNextUTF16 is not null, then *sNextUTF16 points to the first
    element in the input sUTF16[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextUTF16 points to
    the element of sUTF16[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextUTF16 points to
    sUTF16 + sUTF16_count.

  If sUTF8_count > 0, the return value is the number of ON__UINT8
  elements written to sUTF8[].  When the return value < sUTF8_count,
  a null terminator is written to sUTF8[return value].

  If sUTF8_count == 0, the return value is the minimum number of
  ON__UINT8 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF16ToUTF8(
    int bTestByteOrder,
    const ON__UINT16* sUTF16,
    int sUTF16_count,
    ON__UINT8* sUTF8,
    int sUTF8_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const ON__UINT16** sNextUTF16
    );

/*
Description:
  Convert a unicode string from a UTF-16 encoded ON__UINT16 array
  into a UTF-32 encoded ON__UINT32 array.

Parameters:
  bTestByteOrder - [in]
    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFEFF, then this element is ignored.

    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFFFE, then this element is ignored and the subsequent
    elements of sUTF16[] have their bytes swapped before the 
    conversion is calculated.

    In all other cases the first element of sUTF16[] is 
    converted and no byte swapping is performed.

  sUTF16 - [in]
    UTF-16 string to convert.  
    
    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFEFF, then this element is skipped and it is assumed 
    that sUTF16[] is in the CPU's native byte order.
    
    If bTestByteOrder is true and the first element of sUTF16[]
    is 0xFFFE, then this element is skipped and it is assumed 
    that sUTF16[] is not in the CPU's native byte order and bytes
    are swapped before characters are converted.

    If bTestByteOrder is false or the first character of sUTF16[]
    is neither 0xFEFF nor 0xFFFE, then the sUTF16 string must match
    the CPU's byte order.

  sUTF16_count - [in]
    If sUTF16_count >= 0, then it specifies the number of
    ON__UINT16 elements in sUTF16[] to convert.

    If sUTF16_count == -1, then sUTF16 must be a null terminated
    string and all the elements up to the first null element are
    converted.

  sUTF32 - [out]
    If sUTF32 is not null and sUTF32_count > 0, then the UTF-32
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF32_count - [in]
    If sUTF32_count > 0, then it specifies the number of available
    ON__UINT32 elements in the sUTF32[] buffer.
    
    If sUTF32_count == 0, then the sUTF32 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF32 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
      16: An illegal UTF-16 encoding sequence occured or an invalid
          unicode code point value resulted from decoding a
          UTF-16 sequence. This error can be masked. If the error is
          masked and error_code_point is a valid unicode code point,
          then its value is added to the UTF-32 output string and 
          parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUTF16 - [out]
    If sNextUTF16 is not null, then *sNextUTF16 points to the first
    element in the input sUTF16[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextUTF16 points to
    the element of sUTF16[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextUTF16 points to
    sUTF16 + sUTF16_count.

Returns:
  If sUTF32_count > 0, the return value is the number of ON__UINT32
  elements written to sUTF32[].  When the return value < sUTF32_count,
  a null terminator is written to sUTF32[return value].

  If sUTF32_count == 0, the return value is the minimum number of
  ON__UINT32 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF16ToUTF32(
    int bTestByteOrder,
    const ON__UINT16* sUTF16,
    int sUTF16_count,
    unsigned int* sUTF32,
    int sUTF32_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const ON__UINT16** sNextUTF16
    );

/*
Description:
  Convert a unicode string from a UTF-32 encoded ON__UINT32 array
  into a UTF-8 encoded ON__UINT8 array.

Parameters:
  bTestByteOrder - [in]
    If bTestByteOrder is true and the first element of sUTF32[]
    is 0x0000FEFF, then this element is ignored.

    If bTestByteOrder is true and the first element of sUTF32[]
    is 0xFFFE0000, then this element is ignored and the subsequent
    elements of sUTF32[] have their bytes swapped before the 
    conversion is calculated.

    In all other cases the first element of sUTF32[] is 
    converted and no byte swapping is performed.

  sUTF32 - [in]
    UTF-32 string to convert.  
    
    If bTestByteOrder is true and the first element of sUTF32[]
    is 0x0000FEFF, then this element is skipped and it is assumed 
    that sUTF32[] is in the CPU's native byte order.
    
    If bTestByteOrder is true and the first element of sUTF32[]
    is 0xFFFE0000, then this element is skipped and it is assumed 
    that sUTF32[] is not in the CPU's native byte order and bytes
    are swapped before characters are converted.

    If bTestByteOrder is false or the first character of sUTF32[]
    is neither 0x0000FEFF nor 0xFFFE0000, then the sUTF32 string 
    must match the CPU's byte order.

  sUTF32_count - [in]
    If sUTF32_count >= 0, then it specifies the number of
    ON__UINT32 elements in sUTF32[] to convert.

    If sUTF32_count == -1, then sUTF32 must be a null terminated
    string and all the elements up to the first null element are
    converted.
    
  sUTF8 - [out]
    If sUTF8 is not null and sUTF8_count > 0, then the UTF-8
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF8_count - [in]
    If sUTF8_count > 0, then it specifies the number of available
    ON__UINT8 elements in the sUTF8[] buffer.
    
    If sUTF8_count == 0, then the sUTF8 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF8 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
       4: The values of two UTF-32 elements form a valid
          UTF-16 surrogate pair. This error can be masked. If the
          error is masked, then the surrogate pair is converted
          to a valid unicode code point, its UTF-8 encoding is
          added to the UTF-8 output string and parsing continues.
      16: An invalid unicode code point occured in sUTF32[].
          This error can be masked. If the error is masked and
          error_code_point is a valid unicode code point,
          then its UTF-8 encoding is added to the UTF-8 output
          string and parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 4), then type 4 errors are masked.
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUTF32 - [out]
    If sNextUTF32 is not null, then *sNextUTF32 points to the first
    element in the input sUTF32[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextUTF32 points to
    the element of sUTF32[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextUTF32 points to
    sUTF32 + sUTF32_count.

Returns:
  If sUTF8_count > 0, the return value is the number of ON__UINT8
  elements written to sUTF8[].  When the return value < sUTF8_count,
  a null terminator is written to sUTF8[return value].

  If sUTF8_count == 0, the return value is the minimum number of
  ON__UINT8 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF32ToUTF8(
    int bTestByteOrder,
    const ON__UINT32* sUTF32,
    int sUTF32_count,
    ON__UINT8* sUTF8,
    int sUTF8_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const ON__UINT32** sNextUTF32
    );

/*
Description:
  Convert a unicode string from a UTF-32 encoded ON__UINT32 array
  into a UTF-16 encoded ON__UINT16 array.

Parameters:
  bTestByteOrder - [in]
    If bTestByteOrder is true and the first element of sUTF32[]
    is 0x0000FEFF, then this element is ignored.

    If bTestByteOrder is true and the first element of sUTF32[]
    is 0xFFFE0000, then this element is ignored and the subsequent
    elements of sUTF32[] have their bytes swapped before the 
    conversion is calculated.

    In all other cases the first element of sUTF32[] is 
    converted and no byte swapping is performed.

  sUTF32 - [in]
    UTF-32 string to convert.  
    
    If bTestByteOrder is true and the first element of sUTF32[]
    is 0x0000FEFF, then this element is skipped and it is assumed 
    that sUTF32[] is in the CPU's native byte order.
    
    If bTestByteOrder is true and the first element of sUTF32[]
    is 0xFFFE0000, then this element is skipped and it is assumed 
    that sUTF32[] is not in the CPU's native byte order and bytes
    are swapped before characters are converted.

    If bTestByteOrder is false or the first character of sUTF32[]
    is neither 0x0000FEFF nor 0xFFFE0000, then the sUTF32 string 
    must match the CPU's byte order.

  sUTF32_count - [in]
    If sUTF32_count >= 0, then it specifies the number of
    ON__UINT32 elements in sUTF32[] to convert.

    If sUTF32_count == -1, then sUTF32 must be a null terminated
    string and all the elements up to the first null element are
    converted.

  sUTF16 - [out]
    If sUTF16 is not null and sUTF16_count > 0, then the UTF-16
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF16_count - [in]
    If sUTF16_count > 0, then it specifies the number of available
    ON__UINT16 elements in the sUTF16[] buffer.
    
    If sUTF16_count == 0, then the sUTF16 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF16 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
       4: The values of two UTF-32 elements form a valid
          UTF-16 surrogate pair. This error can be masked. If the
          error is masked, then the surrogate pair is added to
          the UTF-16 output string and parsing continues.
      16: An invalid unicode code point occured in sUTF32[].
          This error can be masked. If the error is masked and
          error_code_point is a valid unicode code point,
          then its UTF-16 encoding is added to the UTF-16 output
          string and parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 4), then type 4 errors are masked.
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUnicode - [out]
    If sNextUnicode is not null, then *sNextUnicode points to the first
    byte in the input sNextUnicode[] buffer that was not converted. 

    If an error occurs and is not masked, then this unsigned int
    will be an illegal unicode code point value.

    If an error does not occur, then (*sNextUnicode - sUnicode) 
    is the number of values converted.

Returns:
  If sUTF16_count > 0, the return value is the number of ON__UINT16
  elements written to sUTF16[].  When the return value < sUTF16_count,
  a null terminator is written to sUTF16[return value].

  If sUTF16_count == 0, the return value is the minimum number of
  ON__UINT16 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF32ToUTF16(
    int bTestByteOrder,
    const ON__UINT32* sUTF32,
    int sUTF32_count,
    ON__UINT16* sUTF16,
    int sUTF16_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const ON__UINT32** sNextUTF32
    );

/*
Description:
  Convert a wchar_t string using the native platform's most common
  encoding into a unicode string encoded as a UTF-8 char array.

  If 2 = sizeof(wchar_t), then the wchar_t array is assumed to be
  a UTF-16 encoded string. This is the case with current versions
  of Microsoft Windows.

  If 4 = sizeof(wchar)t), then the wchar_t array is assumed to be
  a UTF-32 encoded string. This is the case with current versions
  of Apple OSX.

Parameters:
  bTestByteOrder - [in]
    If bTestByteOrder is true and the first element of sWideChar[]
    is 0xFEFF, then this element is ignored.

    If bTestByteOrder is true and the first element of sWideChar[]
    is 0xFFFE, then this element is ignored and the subsequent
    elements of sWideChar[] have their bytes swapped before the 
    conversion is calculated.

    In all other cases the first element of sWideChar[] is 
    converted and no byte swapping is performed.

  sWideChar - [in]
    wchar_t string to convert.  
    
    If bTestByteOrder is true and the first element of sWideChar[]
    is 0xFEFF, then this element is skipped and it is assumed 
    that sWideChar[] is in the CPU's native byte order.
    
    If bTestByteOrder is true and the first element of sWideChar[]
    is 0xFFFE, then this element is skipped and it is assumed 
    that sWideChar[] is not in the CPU's native byte order and bytes
    are swapped before characters are converted.

    If bTestByteOrder is false or the first character of sWideChar[]
    is neither 0xFEFF nor 0xFFFE, then the sWideChar string must match
    the CPU's byte order.

  sWideChar_count - [in]
    If sWideChar_count >= 0, then it specifies the number of
    wchar_t elements in sWideChar[] to convert.

    If sWideChar_count == -1, then sWideChar must be a null terminated
    string and all the elements up to the first null element are
    converted.
    
  sUTF8 - [out]
    If sUTF8 is not null and sUTF8_count > 0, then the UTF-8
    encoded string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sUTF8_count - [in]
    If sUTF8_count > 0, then it specifies the number of available
    ON__UINT8 elements in the sUTF8[] buffer.
    
    If sUTF8_count == 0, then the sUTF8 parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sUTF8 output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
      16: An illegal wchar_t encoding sequence occured or an invalid
          unicode code point value resulted from decoding a
          wchar_t sequence. This error can be masked. If the error is
          masked and error_code_point is a valid unicode code point,
          then its UTF-8 encoding is added to the UTF-8 output
          string and parsing continues.

  error_mask - [in]
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextWideChar - [out]
    If sNextWideChar is not null, then *sNextWideChar points to the first
    element in the input sWideChar[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextWideChar points to
    the element of sWideChar[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextWideChar points to
    sWideChar + sWideChar_count.

  If sUTF8_count > 0, the return value is the number of ON__UINT8
  elements written to sUTF8[].  When the return value < sUTF8_count,
  a null terminator is written to sUTF8[return value].

  If sUTF8_count == 0, the return value is the minimum number of
  ON__UINT8 elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertWideCharToUTF8(
    int bTestByteOrder,
    const wchar_t* sWideChar,
    int sWideChar_count,
    char* sUTF8,
    int sUTF8_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const wchar_t** sNextWideChar
    );

/*
Description:
  Convert a UTF-8 encoded char string to wchar_t string using
  the native platform's most common encoding.

  If 2 = sizeof(wchar_t), then UTF-16 encoding is used for the
  output string. This is the case with current versions of
  Microsoft Windows.

  If 4 = sizeof(wchar_t), then UTF-32 encoding is used for the
  output string. This is the case with current versions of
  Apple OSX.

Parameters:
  sUTF8 - [in]
    UTF-8 string to convert.

  sUTF8_count - [in]
    If sUTF8_count >= 0, then it specifies the number of
    ON__UINT8 elements in sUTF8[] to convert.

    If sUTF8_count == -1, then sUTF8 must be a null terminated
    string and all the elements up to the first null element are
    converted.

  sWideChar - [out]
    If sWideChar is not null and sWideChar_count > 0, then the
    output string is returned in this buffer. If there is room
    for the null terminator, the converted string will be null
    terminated. The null terminator is never included in the count 
    of returned by this function. The converted string is in the 
    CPU's native byte order. No byte order mark is prepended.

  sWideChar_count - [in]
    If sWideChar_count > 0, then it specifies the number of available
    wchar_t elements in the sWideChar[] buffer.
    
    If sWideChar_count == 0, then the sWideChar parameter is ignored.

  error_status - [out]
    If error_status is not null, then bits of *error_status are
    set to indicate the success or failure of the conversion.  
    When the error_mask parameter is used to used to mask some
    conversion errors, multiple bits may be set.
       0: Successful conversion with no errors.
       1: Invalid input parameters. This error cannot be masked.
       2: The sWideChar output buffer was not large enough to hold 
          the converted string. This error cannot be masked.
       4: The values of two UTF-8 encoding sequences formed a valid
          UTF-16 surrogate pair. This error can be masked.  If the
          error is masked, then the surrogate pair is added
          to the UTF-16 output string and parsing continues.
       8: An overlong UTF-8 encoding sequence was encountered. 
          The value of the overlong sequence was a valid unicode
          code point. This error can be masked. If the error is masked,
          then the unicode code point is encoded and added to the
          UTF-16 output string and parsing continues.
      16: An illegal UTF-8 encoding sequence occured or an invalid
          unicode code point value resulted from decoding a
          UTF-8 sequence. This error can be masked. If the error is
          masked and error_code_point is a valid unicode code point,
          then its encoding is added to the output string and parsing
          continues.

  error_mask - [in]
    If 0 != (error_mask & 4), then type 4 errors are masked.
    If 0 != (error_mask & 8), then type 8 errors are masked.
    If 0 != (error_mask & 16) and error_code_point is a valid unicode
    code point value, then type 16 errors are masked.

  error_code_point - [in]
    Unicode code point value to use in when masking type 16 errors.
    If 0 == (error_mask & 16), then this parameter is ignored.
    0xFFFD is a popular choice for the error_code_point value.

  sNextUTF8 - [out]
    If sNextUTF8 is not null, then *sNextUTF8 points to the first
    element in the input sUTF8[] buffer that was not converted. 

    If an error occurs and is not masked, then *sNextUTF8 points to
    the element of sUTF8[] where the conversion failed.  If no errors
    occur or all errors are masked, then *sNextUTF8 points to
    sUTF8 + sUTF8_count.

Returns:
  If sWideChar_count > 0, the return value is the number of wchar_t
  elements written to sWideChar[].  When the return value < sWideChar_count,
  a null terminator is written to sWideChar[return value].

  If sWideChar_count == 0, the return value is the minimum number of
  wchar_t elements that are needed to hold the converted string.
  The return value does not include room for a null terminator.  
  Increment the return value by one if you want to have an element
  to use for a null terminator.
*/
ON_DECL
int ON_ConvertUTF8ToWideChar(
    const char* sUTF8,
    int sUTF8_count,
    wchar_t* sWideChar,
    int sWideChar_count,
    unsigned int* error_status,
    unsigned int error_mask,
    ON__UINT32 error_code_point,
    const char** sNextUTF8
    );

ON_END_EXTERNC

#endif

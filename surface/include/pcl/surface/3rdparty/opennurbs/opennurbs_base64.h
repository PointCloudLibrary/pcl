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

#if !defined(OPENNURBS_BASE64_INC_)
#define OPENNURBS_BASE64_INC_

//////////////////////////////////////////////////////////////////////////////////////////

class ON_CLASS ON_Base64EncodeStream
{
public:
  ON_Base64EncodeStream();
  virtual ~ON_Base64EncodeStream();

  /*
  Description:
    ON_Base64EncodeStream delivers the base64 encoded stream by
    calling a base64 encoded stream output handler function. 
    There are two options for specifying the base64 encoded stream
    output handler function.
      1. Overriding the virtual Out() function.
      2. Providing a callback function.
    SetCallback() is used to specify a callback function to handle
    the base64 encoded stream and to specify a context pointer to be 
    passed to either option of the handler.
  Parameters:
    callback_function - [in]
      Function to handle sections of the base64 encoded stream.
      If callback_function is null, then the virtual Out() 
      function will be called. When callback_function 
      is specified, it must return true if the base64 encoding 
      calculation should continue and false to cancel the 
      base64 encoding calculation.
    callback_context - [in]
      This value is passed as the first argument when calling 
      callback_function or the virutal Out() function.
  Returns:
    True if successful.
  Remarks:
    Once base64 encoding has started, it would be unusual to
    intentionally change the base64 encoded stream output handler,
    but you can do this if you need to.    
  */
  bool SetCallback( 
    ON_StreamCallbackFunction callback_function,
    void* callback_context
    );

  /*
  Returns:
    Current value of the callback function for handling
    the base64 encoded stream.  If the callback function is
    null, the the virtual Out() function is used to
    handle the output stream.
  */
  ON_StreamCallbackFunction CallbackFunction() const;

  /*
  Returns:
    Current value of the context pointer passed as the first
    argument to the base64 encoded stream output handler function.
  */
  void* CallbackContext() const;
  
  /*
  Description:
    Call Begin() one time to initialize the base64 encoding
    calculation.  Then call In() one or more times 
    to submit the unencoded stream to the base64 encoding
    calculation. When you reach the end of the unencoded
    stream, call End().
  Returns:
    true if successful, false if an error occured.
  */
  bool Begin();


  /*
  Description:
    Call In() one or more times to base64 encode a stream of bytes.
    After the last call to In(), call End().  Calling In() will
    result in at least in_buffer_size/57 and at most 
    (in_buffer_size+56)/57 calls to to the output stream handler.
  Parameters:
    in_buffer_size - [in]
      number of bytes in in_buffer
    in_buffer - [in]
  Returns:
    true if successful, false if an error occured.
  */
  bool In(
    ON__UINT64 in_buffer_size,
    const void* in_buffer
    );

  /*
  Description:
    If an explicit base 64 encoded stream output handler is not
    specified ( CallbackFunction() returns null ), then the 
    virtual Out() function is called to handle the base 64 encoded
    output stream.  As the input stream is encoded, one or more
    calls to Out() will occur.

    With a possible exception of the last call to Out(), when Out()
    is called, 57 input bytes have been encoded into 76 output 
    characters with ASCII codes A-Z, a-z, 0-9, +, /.
  Parameters:
    callback_context - [in]
      context pointer set by calling SetCallback().  Typically
      the context pointer is not used by a virtual override
      because the context can be added as member variables
      of the derived class, but it is available if needed.
    out_buffer_size - [in]
      number of non-null characters in out_buffer.
    out_buffer - [in]
      A null terminated ASCII string that is a base 64 encoding.
      out_buffer[0...(out_buffer_size-1)] are ASCII characters with
      values characters with ASCII codes A-Z, a-z, 0-9, +, /
      and out_buffer[out_buffer_size] = 0.    
  Returns:
    True to continue base 64 encodeing and false to cancel the
    encoding calculation.
  */
  virtual bool Out( 
    void* callback_context, 
    ON__UINT32 out_buffer_size, 
    const char* out_buffer 
    );

  /*
  Description:
    After the last call to In(), call End().  Calling End() may
    generate one call to the output stream handler with the value
    of out_buffer_size = 4 to 76.
  Returns:
    true if successful, false if an error occured.
  */
  bool End();
  
  /*
  Returns:
    Then the returned value is the total number bytes in the input
    stream. The size is updated every time In() is called before 
    any calls are made to the output stream handler.  If the 
    calculation is finished ( End() has been called ), then the
    returned value is the total number of bytes in the entire 
    input stream.
  */
  ON__UINT64 InSize() const;

  /*
  Returns:
    Then the returned value is the total number characters in the
    output stream. The size is incremented immediately after each
    call to the output stream handler.  If the base64 encoding
    calculation is finished ( End() has been called ), then the
    returned value is the total number of bytes in the entire 
    output stream.
  */
  ON__UINT64 OutSize() const;

  /*
  Returns:
    Then the returned value is the 32-bit crc of the input stream.
    The crc is updated every time In() is called before any calls
    are made to the output stream handler.  If the base64 encoding 
    calculation is finished ( End() has been called ), then the
    returned value is the 32-bit crc of the entire input stream.
  */
  ON__UINT32 InCRC() const;

  /*
  Returns:
    Then the returned value is the 32bit crc of the output stream.
    The crc is updated immediately after each call to the output
    stream handler.  If the calculation is finished ( End() has 
    been called ), then the returned value is the 32-bit crc of
    the entire output stream.
  */
  ON__UINT32 OutCRC() const;
    
private:
  ON_StreamCallbackFunction m_out_callback_function;
  void* m_out_callback_context;
  ON__UINT64 m_in_size;
  ON__UINT64 m_out_size;
  ON__UINT32 m_in_crc;
  ON__UINT32 m_out_crc;
  void* m_implementation;
  void* m_reserved;

  void ErrorHandler();
  
private:
  // prohibit use - no implementation
  ON_Base64EncodeStream(const ON_Base64EncodeStream&);
  ON_Base64EncodeStream& operator=(const ON_Base64EncodeStream&);
};

//////////////////////////////////////////////////////////////////////////////////////////

class ON_CLASS ON_DecodeBase64
{
public:
  ON_DecodeBase64();
  virtual ~ON_DecodeBase64();

  void Begin();

  // Decode will generate zero or more callbacks to the
  // virtual Output() function.  If the base 64 encoded information
  // is in pieces, you can call Decode() for each piece.  For example,
  // if your encoded information is in a text file, you might call
  // Decode() for every line in the file.  Decode() returns 0 if
  // there is nothing in base64str to decode or if it detects an
  // error that prevents any further decoding.  The function Error()
  // can be used to determine if an error occured.  Otherwise,
  // Decode() returns a pointer to the location in the string where
  // it stopped decoding because it detected a character, like a null
  // terminator, an end of line character, or any other character
  // that could not be part of the base 64 encoded information.
  const char* Decode(const char* base64str);
  const char* Decode(const char* base64str, std::size_t base64str_count);
  const wchar_t* Decode(const wchar_t* base64str);
  const wchar_t* Decode(const wchar_t* base64str, std::size_t base64str_count);

  // You must call End() when Decode() returns 0 or when you have
  // reached the end of your encoded information.  End() may
  // callback to Output() zero or one time.  If all the information
  // passed to Decode() was successfully decoded, then End()
  // returns true.  If something was not decoded, then End()
  // returns false.
  bool End();

  // Override the virtual Output() callback function to process the 
  // decoded output.  Each time Output() is called there are m_output_count
  // bytes in the m_output[] array.
  // Every call to Decode() can result in zero, one, or many callbacks
  // to Output().  Calling End() may result in zero or one callbacks
  // to Output().
  virtual void Output();

  // m_decode_count = total number of input base64 characters
  // that Decode() has decoded.
  unsigned int m_decode_count; 

  int  m_output_count; // 0 to 512
  unsigned char m_output[512];

  // Call if your Output() function detects an error and
  // wants to stop further decoding.
  void SetError();

  // Returns true if an error occured during decoding because
  // invalid input was passed to Decode().
  bool Error() const;

private:
  int m_status; // 1: error - decoding stopped
                // 2: '=' encountered as 3rd char in Decode()
                // 3: successfully parsed "**=="
                // 4: successfully parsed "***="
                // 5: End() successfully called.

  // cached encoded input from previous call to Decode() 
  int m_cache_count;
  int m_cache[4];

  void DecodeHelper1(); // decodes "**==" quartet into 1 byte
  void DecodeHelper2(); // decodes "***=" quartet into 2 bytes
};


/////////////////////////////////////////////////////////////////////


/*
class ON_CLASS ON_EncodeBase64
{
public:
  ON_EncodeBase64();
  virtual ~ON_EncodeBase64();

  void Begin();

  // Calling Encode will generate at least
  // sizeof_buffer/57 and at most (sizeof_buffer+56)/57
  // calls to Output().  Every callback to Output() will
  // have m_output_count = 76.
  void Encode(const void* buffer, std::size_t sizeof_buffer);

  // Calling End may generate a single call to Output()
  // If it does generate a single call to Output(),
  // then m_output_count will be between 1 and 76.
  void End(); // may generate a single call to Output().

  // With a single exception, when Output() is called,
  // 57 input bytes have been encoded into 76 output
  // characters with ASCII codes A-Z, a-z, 0-9, +, /.
  // m_output_count will be 76
  // m_output[0...(m_output_count-1)] will be the base 64
  // encoding.
  // m_output[m_output_count] = 0.
  // The Output() function can modify the values of m_output[]
  // and m_output_count anyway it wants.
  virtual void Output();

  // Total number of bytes passed to Encode().
  int m_encode_count;

  // When the virtual Output() is called, there are m_output_count (1 to 76)
  // characters of base64 encoded output in m_output[].  The remainder of 
  // the m_output[] array is zero.  The Output function may modify the
  // contents of m_output[] any way it sees fit.
  int  m_output_count;
  char m_output[80];
  
private:
  // input waiting to be encoded
  // At most 56 bytes can be waiting to be processed in m_input[].
  unsigned int  m_unused2; // Here for alignment purposes. Never used by opennurbs.
  unsigned int  m_input_count;
  unsigned char m_input[64];

  void EncodeHelper1(const unsigned char*, char*);
  void EncodeHelper2(const unsigned char*, char*);
  void EncodeHelper3(const unsigned char*, char*);
  void EncodeHelper57(const unsigned char*);
};
*/

#endif

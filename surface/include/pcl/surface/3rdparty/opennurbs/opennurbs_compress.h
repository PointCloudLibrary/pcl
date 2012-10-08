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

#if !defined(OPENNURBS_COMPRESS_INC_)
#define OPENNURBS_COMPRESS_INC_

typedef bool (*ON_StreamCallbackFunction)( void* context, ON__UINT32 size, const void* buffer );

class ON_CLASS ON_CompressStream
{
public:
  ON_CompressStream();
  virtual ~ON_CompressStream();

  /*
  Description:
    ON_CompressStream delivers the compressed stream by calling
    a compressed stream output handler function.  There are two
    options for specifying the compressed stream output handler
    function.
      1. Overriding the virtual Out() function.
      2. Providing a callback function.
    SetCallback() is used to specify a callback function to handle
    the compressed stream and to specify a context pointer to be 
    passed to either option of the handler.
  Parameters:
    callback_function - [in]
      Function to call with sections of the compressed stream.
      If callback_function is null, then the virtual Out() 
      function will be called. When callback_function 
      is specified, it must return true if the compression 
      calculation should continue and false to cancel the 
      compression calculation.
    callback_context - [in]
      This value is passed as the first argument when calling 
      callback_function or the virutal Out() function.
  Returns:
    True if successful.
  Remarks:
    Once compression has started, it would be unusual to
    intentionally change the compressed stream output handler,
    but you can do this if you need to.    
  */
  bool SetCallback( 
    ON_StreamCallbackFunction callback_function,
    void* callback_context
    );

  /*
  Returns:
    Current value of the callback function for handling
    the compressed stream.  If the callback function is
    null, the the virtual Out() function is used to
    handle
  */
  ON_StreamCallbackFunction CallbackFunction() const;

  /*
  Returns:
    Current value of the context pointer passed as the first
    argument to the compressed stream output handler function.
  */
  void* CallbackContext() const;
  
  /*
  Description:
    Call Begin() one time to initialize the compression
    calculation.  Then call In() one or more times 
    to submit the uncompressed stream to the compression calculation.  
    When you reach the end of the uncompressed stream, call 
    End().
  Returns:
    true if successful, false if an error occured.
  */
  bool Begin();

  /*
  Description:
    Call In() one or more times to compress a stream of uncompressed
    bytes.  After the last call to In(), call End().  Calling In()
    may generate zero or more calls to the output stream handler.
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
    If an explicit compressed stream output handler is not specified
    ( CallbackFunction() returns null ), then the virtual Out() 
    function is called to handle the compressed output stream.
    As the input stream is compressed, one or more calls to Out()
    will occur.
  Returns:
    True to continue compressing and false to cancel the compression
    calculation.
  Remarks:
    In general, it is probably going to be easier to test and debug
    your code if you ignore the callback_context parameter and add 
    a member variable to your derived class to make additional
    information accessable to your Out function.
  */
  virtual bool Out( 
    void* callback_context, 
    ON__UINT32 out_buffer_size, 
    const void* out_buffer 
    );

  /*
  Description:
    After the last call to In(), call End().  
    Calling End() may generate zero or more 
    calls to the output stream handler.
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
    Then the returned value is the total number bytes in the output
    stream. The size is incremented immediately after each call to
    the output stream handler.  If the compression calculation is 
    finished ( End() has been called ), then the returned value is
    the total number of bytes in the entire output stream.
  */
  ON__UINT64 OutSize() const;

  /*
  Returns:
    Then the returned value is the 32-bit crc of the input stream.
    The crc is updated every time In() is called before any calls
    are made to the output stream handler.  If the compression 
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
  ON_CompressStream(const ON_CompressStream&);
  ON_CompressStream& operator=(const ON_CompressStream&);
};


class ON_CLASS ON_UncompressStream
{
public:
  ON_UncompressStream();
  virtual ~ON_UncompressStream();

  /*
  Description:
    ON_UncompressStream delivers the uncompressed stream by calling
    an uncompressed stream output handler function.  There are two
    options for specifying the uncompressed stream output handler
    function.
      1. Overriding the virtual Out() function.
      2. Providing a callback function.
    SetCallback() is used to specify a callback function to handle
    the uncompressed stream and to specify a context pointer to be 
    passed to either option of the handler.
  Parameters:
    callback_function - [in]
      Function to call with sections of the uncompressed stream.
      If callback_function is null, then the virtual Out() 
      function will be called. When callback_function 
      is specified, it must return true if the uncompression 
      calculation should continue and false to cancel the 
      uncompression calculation.
    callback_context - [in]
      This value is passed as the first argument when calling 
      callback_function or the virutal Out() function.
  Returns:
    True if successful.
  Remarks:
    Once uncompression has started, it would be unusual to
    intentionally change the uncompressed stream output handler,
    but you can do this if you need to.    
  */
  bool SetCallback( 
    ON_StreamCallbackFunction callback_function,
    void* callback_context
    );

  /*
  Returns:
    Current value of the callback function for handling
    the uncompressed stream.  If the callback function is
    null, the the virtual UncompressedStreamOut() function
    is used.
  */
  ON_StreamCallbackFunction CallbackFunction() const;

  /*
  Returns:
    Current value of the context pointer passed as the first
    argument to the uncompressed stream output handler function.
  */
  void* CallbackContext() const;
  
  /*
  Description:
    Call BeginUnompressStream() one time to initialize the compression
    calculation.  Then call In() one or more times 
    to submit the compressed stream to the uncompression calculation.  
    When you reach the end of the compressed stream, call 
    End().
  Returns:
    true if successful, false if an error occured.
  */
  bool Begin();

  /*
  Description:
    Call In() one or more times to uncompress a stream of compressed
    bytes.  After the last call to In(), call End(). Calling End()
    may generate zero or more calls to the output stream handler.
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
    If an explicit uncompressed stream handler is not specified
    ( CallbackFunction() returns null ), then the virtual Out() 
    function is called to handle the uncompressed output stream.
    As the input stream is uncompressed, one or more calls to Out()
    will occur.
  Returns:
    True to continue uncompressing and false to cancel the 
    uncompression calculation.
  Remarks:
    In general, it is probably going to be easier to test and debug
    your code if you ignore the callback_context parameter and add 
    a member variable to your derived class to make additional
    information accessable to your Out function.
  */
  virtual bool Out( 
    void* callback_context, 
    ON__UINT32 out_buffer_size, 
    const void* out_buffer 
    );

  /*
  Description:
    After the last call to In(), call End().  
    Calling End() may generate zero or more 
    calls to the output stream handler.
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
    Then the returned value is the total number bytes in the output
    stream. The size is incremented immediately after each call to
    the output stream handler.  If the compression calculation is 
    finished ( End() has been called ), then the returned value is
    the total number of bytes in the entire output stream.
  */
  ON__UINT64 OutSize() const;

  /*
  Returns:
    Then the returned value is the 32-bit crc of the input stream.
    The crc is updated every time In() is called before any calls
    are made to the output stream handler.  If the compression 
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
  ON_UncompressStream(const ON_UncompressStream&);
  ON_UncompressStream& operator=(const ON_UncompressStream&);
};

#endif

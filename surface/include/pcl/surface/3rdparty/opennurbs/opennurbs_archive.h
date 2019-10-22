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

#if !defined(ON_ARCHIVE_INC_)
#define ON_ARCHIVE_INC_

class ON_CLASS ON_FileStream
{
public:
  /*
  Description:
    Portable wrapper for C runtime fopen().
  Parameters:
    filename - [in]
    mode - [in]
  Remarks:
    Use the ON_FileStream static functions for reading, writing, 
    seeking, position finding with the FILE pointer returned
    by this function.
  */
  static FILE* Open( const wchar_t* filename, const wchar_t* mode );

  /*
  Description:
    Portable wrapper for C runtime fopen().
  Parameters:
    filename - [in]
    mode - [in]
  Remarks:
    Use the ON_FileStream static functions for reading, writing, 
    seeking, position finding with the FILE pointer returned
    by this function.
  */
  static FILE* Open( const char* filename, const char* mode );
  
  /*
  Description:
    Portable wrapper for C runtime fclose().
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
  Returns:
       0: successful
      -1: null fp parameter
    != 0: fclose() failure code
  */
  static int Close( FILE* fp );

  /*
  Description:
    Portable wrapper for C runtime ftell().
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
  Returns:
    >= 0: current file position
      -1: an error occured
  */
  static ON__INT64 CurrentPosition( FILE* fp );

  /*
  Description:
    Portable wrapper for C runtime fseek(fp,offset,SEEK_CUR).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
    offset - [in]
  */
  static bool SeekFromCurrentPosition( FILE* fp, ON__INT64 offset );

  /*
  Description:
    Portable wrapper for C runtime fseek(fp,offset,SEEK_SET).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
    offset - [in]
  */
  static bool SeekFromStart( FILE* fp, ON__INT64 offset );

  /*
  Description:
    Portable wrapper for C runtime fseek(fp,offset,SEEK_END).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
    offset - [in]
  */
  static bool SeekFromEnd( FILE* fp, ON__INT64 offset );

  /*
  Description:
    Portable wrapper for C runtime fseek(fp,offset,origin).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
    offset - [in]
    origin - [in]
      SEEK_SET (0): seek from beginning of file.  
      SEEK_CUR (1): seek from current position of file pointer.
      SEEK_END (2): seek from end of file.
  */
  static bool Seek( FILE* fp, ON__INT64 offset, int orgin );

  /*
  Description:
    Portable wrapper for C runtime fread(buffer,1,count,fp).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open()
    count - [in]
      number of bytes to read.
    buffer - [out]
      read bytes are stored in this buffer
  Returns:
    number of bytes read
  */
  static ON__UINT64 Read( FILE* fp, ON__UINT64 count, void* buffer );

  /*
  Description:
    Portable wrapper for C runtime fwrite(buffer,1,count,fp).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open()
    count - [in]
      number of bytes to write
    buffer - [in]
      data to be written
  Returns:
    number of bytes written.
  */
  static ON__UINT64 Write( FILE* fp, ON__UINT64 count, const void* buffer );

  /*
  Description:
    Portable wrapper for C runtime fflush(fp).
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
  Returns:
    true if flush was successful.  False if an error occured.
  */
  static bool Flush( FILE* fp );

  /*
  Description:
    Portable wrapper for C runtime fstat().
  Parameters:
    fp - [in]
      FILE pointer returned by ON_FileStream::Open().
    file_size - [out]
      If file_size is not null, the the size of the file
      in bytes returned here
    file_creation_time - [out]
      If file_creation_time is not null, then the time the file 
      was created is returned here as the number of seconds since
      midnight January 1, 1970.
    file_last_modified_time - [out]
      If file_last_modified_time is not null, then the time the file
      was last modified is returned here as the number of seconds
      since midnight January 1, 1970.
  Returns:
    true if the query was successful.  False if an error occured.
  */
  static bool GetFileInformation( 
    FILE* fp,
    ON__UINT64* file_size,
    ON__UINT64* file_create_time,
    ON__UINT64* file_last_modified_time
    );
};

class ON_CLASS ON_FileIterator
{
public:
  ON_FileIterator();
  ~ON_FileIterator();
  void Destroy();

  /*
  Description:
    Find the first matching file in the directory.
  Parameters:
    directory_name - [in]
      The directory to look in.
    file_name_filter - [in]
      If this paramter is null, then the iteration
      includes all names in the directory.
      The file name to search for. This parameter can 
      include wildcard characters, such as an
      asterisk (*) or a question mark (?). For example,
      "\rootdir\subdir\*.*"  will iterate all files in
      the \rootdir\subdir\ directory.

  Example:
          // Iterate through the files in a directory named "\rootdir\subdir"
          FILE* fp = 0;
          ON_FileIterator fit;
          const char* directory = "\\rootdir\\subdir";
          for ( const wchar_t* filename = fit.FirstFile( directory, "*.3dm" );
                0 != filename;
                filename = fit.NextFile()
              )
          {
            if ( fit.CurrentFileIsDirectory() )
              continue;
            ON_String fullpath = directory;
            fullpath += '\\';
            fullpath += filename;
            FILE* fp = ON_FileStream::Open(fullpath,"rb");
            if ( 0 == fp )
            {
              continue;
            }
            ...
            ON_FileStream::Close(fp);
            fp = 0;
          }
        }

  Returns:
    NULL if no matching files are present in the directory.
  */
  const wchar_t* FirstFile( 
    const wchar_t* directory_name, 
    const wchar_t* file_name_filter
    );

  const wchar_t* FirstFile( 
    const char* directory_name, 
    const char* file_name_filter
    );

  /*
  Description:
    Find the next matching file in the directory.
  Returns:
    NULL if no more matching files are present in the directory.
  */
  const wchar_t* NextFile();

  const wchar_t* CurrentFileName() const;

  ON__UINT64 CurrentFileSize() const;

  /*
  Returns 
    true if the current "file" is a directory.
  */
  bool CurrentFileIsDirectory() const;

  /*
  Returns 
    true if the current file or directory is hidden.
    This means its name begins with a '.' or it's
    Windows hidden attribute is true.
  */
  bool CurrentFileIsHidden() const;

  bool GetCurrentFullPathFileName( ON_wString& filename ) const;

  /*
  Returns:
    File creation time in seconds since January 1, 1970
  */
  ON__UINT64 CurrentFileCreateTime() const;

  /*
  Returns:
    File last modified time in seconds since January 1, 1970
  */
  ON__UINT64 CurrentFileLastModifiedTime() const;

  /*
  Returns:
    File last access time in seconds since January 1, 1970
  */
  ON__UINT64 CurrentFileLastAccessTime() const;

  /*
  Returns:
    Number of matching files returned so far.
  */
  ON__UINT64 Count() const;

private:
  // Used by Windows ::Find
  ON__UINT64 m_count;
  ON_wString m_directory;

#if defined(ON_COMPILER_MSC)
  ON__UINT32 m_file_attributes_mask;
  HANDLE m_h;
  WIN32_FIND_DATA m_fd;
#else
  ON_wString m_ws_file_name_filter;
  ON_String m_utf8_file_name_filter;
  DIR* m_dir;
  struct dirent m_dirent;
  char m_dirent_name_buffer[NAME_MAX+1]; // < this field provide storage for m_dirent.d_name[]

  // information about the current file
  wchar_t m_current_name[1024];
  ON__UINT64 m_current_file_attributes; // 1 = regular file, 2 = directory
  ON__UINT64 m_current_file_size;
  ON__UINT64 m_current_file_create_time;
  ON__UINT64 m_current_last_modified_time;
  ON__UINT64 m_current_last_access_time;
#endif
};


/////////////////////////////////////////////////////////////////////
//
// ON_Buffer 
//

typedef void (*ON_Buffer_ErrorHandler)(class ON_Buffer*);

class ON_CLASS ON_Buffer
{
public:
  ON_Buffer();
  ~ON_Buffer();

  ON_Buffer(const ON_Buffer& src);
  ON_Buffer& operator=(const ON_Buffer& src);

  /*
  Description:
    Compare contents of buffers.
  Paramters:
    a - [in]
    b - [in]
  Returns:
    -1: a < b
     0: a == b
     1: a > b
  */
  static int Compare( const ON_Buffer& a, const ON_Buffer& b );

  void Destroy();
  void EmergencyDestroy();

  /*
  Returns:
    True if Size() == CurrentPosition().
  Remarks:
    It is possible to seek beyond the end of the buffer.
    In this case, the current position will be past the end
    of the buffer and AtEnd() will return false.
  */
  bool AtEnd() const;

  /*
  Returns:
    Number of bytes currently in the buffer.
  Remarks:
    It is possible to seek beyond the end of the buffer.
    In this case, the current position will be past the end
    of the buffer and CurrentPosition() will be greater than
    Size().
  */
  ON__UINT64 Size() const;

  /*
  Returns:
    32-bit CRC of the buffer contents.
  Remarks:
    
  */
  ON__UINT32 CRC32( ON__UINT32 current_remainder ) const;


  /*
  Returns:
    Current position in the buffer.
  Remarks:
    It is possible to seek beyond the end of the buffer.
    In this case, the current position will be past the end
    of the buffer and CurrentPosition() will be greater than
    Size().
  */
  ON__UINT64 CurrentPosition() const;
  
  /*
  Parameters:
    size - [in]
      number of bytes to write.
    buffer - [in]
      values to write.
  Returns:
    Number of bytes written buffer.
  */
  ON__UINT64 Write( ON__UINT64 size, const void* buffer );

  /*
  Parameters:
    size - [in]
      number of bytes to read.
    buffer - [out]
      read values are returned in buffer.
  Returns:
    Number of bytes read into buffer. For example, 
    if CurrentPosition() <= Size() and 
    size > (Size() - CurrentPosition()) and
    buffer is not null, then the value
    (Size() - CurrentPosition()) is returned.
  Remarks:
    If the size parameter is zero, then nothing is done.
    When CurrentPosition() <= Size(), attempts to read more 
    than (Size() - CurrentPosition()) bytes do not generate 
    an error. When CurrentPosition() > Size(), any attempt
    to read generates an error.
  */
  ON__UINT64 Read( ON__UINT64 size, void* buffer );

  enum
  {
    seek_from_beginning_of_file = 0,
    seek_from_current_position = 1,
    seek_from_end_of_file = 2
  };

  /*
  Parameters:
    offset - [in]
      number of bytes to seek from origin
    origin - [in]
      initial position.
        0 (SEEK_SET) Seek from beginning of file.
        1 (SEEK_CUR) Seek from current position.
        2 (SEEK_END) Seek from end of file.
  Returns:
    True if successful.
    False if the seek would result in a file position
    before the beginning of the file. If false is
    returned, the current position is not changed.
  Remarks:
    Seeking beyond the end of the buffer is succeeds.
    Seeking before the beginning of the buffer fails.
  */
  bool Seek( 
    ON__INT64 offset, 
    int origin 
    );

  /*
  Parameters:
    offset - [in] (>= 0)
      number of bytes to seek from the start of the buffer.
  Returns:
    True if successful.
    False if the seek would result in a file position
    before the beginning of the file. If false is
    returned, the current position is not changed.
  Remarks:
    Seeking beyond the end of the buffer is succeeds.
    Seeking before the beginning of the buffer fails.
  */
  bool SeekFromStart( ON__INT64 offset ); 

  /*
  Parameters:
    offset - [in]
      number of bytes to seek from the current position.
  Returns:
    True if successful.
    False if the seek would result in a file position
    before the beginning of the file. If false is
    returned, the current position is not changed.
  Remarks:
    Seeking beyond the end of the buffer is succeeds.
    Seeking before the beginning of the buffer fails.
  */
  bool SeekFromCurrentPosition( ON__INT64 offset ); 

  /*
  Parameters:
    offset - [in]
      number of bytes to seek from the end fo the buffer.
  Returns:
    True if successful.
    False if the seek would result in a file position
    before the beginning of the file. If false is
    returned, the current position is not changed.
  Remarks:
    Seeking beyond the end of the buffer is succeeds.
    Seeking before the beginning of the buffer fails.
  */
  bool SeekFromEnd( ON__INT64 offset ); 

  /*
  Parameters:
    buffer_size - [in]
      new size of buffer.
  Returns:
    True if successful.    
  Remarks:
    The current position is not changed and may be beyond the
    end of the file. Use Seek to set the current position after
    calling ChangeSize().
  */
  bool ChangeSize( ON__UINT64 buffer_size );

  /*
  Description:
    Return unused memory to heap.
  Remarks:
    Call this function after creating an ON_Buffer that will persist for
    and extended amount of time. There are never more than 16 pages of
    unsued memory (16*4096 bytes on most computers) in an ON_Buffer.
    Compact() can be called at any time, but calling Compact() the then
    writing at the end of the buffer is not an efficient use of time
    or memory.
  */
  bool Compact();

  /*
  Returns
    True if the ON_Buffer is valid.
  */
  bool IsValid( const ON_TextLog* text_log ) const;

  /*
  Returns:
    Value that identifies most recent error.
    0: no error
    1: attempt to seek to a negative position
  */
  ON__UINT32 LastError() const;
  
  void ClearLastError();

  ON_Buffer_ErrorHandler ErrorHandler() const;
  
  void SetErrorHandler(ON_Buffer_ErrorHandler error_handler);

  /*
  Description:
    Use WriteToBinaryArchive() to save an entire ON_Buffer inside
    a binary archive.  Use ReadFromBinaryArchive() to retrieve
    the ON_Buffer from the ON_BinaryArchive.
  */
  bool WriteToBinaryArchive( ON_BinaryArchive& ) const;

  /*
  Description:
    Use ReadFromBinaryArchive() to retrieve an entire ON_Buffer
    that was written using WriteToBinaryArchive().
  */
  bool ReadFromBinaryArchive( ON_BinaryArchive& );

  /*
  Description:
    Compress this buffer

  Parameters:
    compressed_buffer - [out]
      (The reference can be *this)
  
  Example:

        // compress a buffer in place
        ON_Buffer buffer;
        buffer = ...;
        if ( !buffer.Compress(buffer) )
        {
           // compression failed
        }
        else
        {
           // buffer is now compressed
        }

  Returns:
    True if successful.  False if failed.
  */
  bool Compress( ON_Buffer& compressed_buffer ) const;

  /*
  Description:
    Uncompress this buffer which must have been compressed using
    ON_Buffer::Compress().

  Parameters:
    uncompressed_buffer - [out]
      (The reference can be *this)

  Example:
        // silly example that compresses and then uncompresses a buffer in place
        // to show how to call the functions.
        ON_Buffer buffer;
        buffer = ...; // buffer is in it uncompressed form
        if ( buffer.Compress(buffer) )
        {
           // buffer is now compressed
           if ( buffer.Uncompress(buffer) )
           {
              // buffer is uncompressed again.
           }
        }

  Returns:
    True if successful.  False if failed.
  */
  bool Uncompress( ON_Buffer& uncompressed_buffer ) const;

private:

  ON__UINT64 m_buffer_size; // total number of bytes in the buffer
  ON__UINT64 m_current_position;

  struct ON_BUFFER_SEGMENT* m_first_segment;
  struct ON_BUFFER_SEGMENT* m_last_segment;
  struct ON_BUFFER_SEGMENT* m_current_segment;
  bool SetCurrentSegment(bool);
  void Copy( const ON_Buffer& );

  ON_MEMORY_POOL* m_heap;
  ON_Buffer_ErrorHandler m_error_handler;

  ON__UINT32 m_last_error;
  unsigned char m_reserved[12];
};

/////////////////////////////////////////////////////////////////////
//
// ON_BinaryArchive 
//      virtual class for CPU independent serialization
//
// ON_BinaryFile
//      simple class for CPU independent binary file I/O
//      includes optional CRC support
//

class ON_Object;
class ON_Group;
class ON_Font;
class ON_DimStyle;
class ON_Arc;
class ON_ObjectAttributes;
class ON_InstanceDefinition;
class ON_HatchPattern;
class ON_Linetype;

struct ON_3DM_CHUNK
{
  std::size_t m_offset; // In read or write_using_fseek mode, this is the
                   // file position of first byte after chunk's length.
                   // In write_using_buffer mode, this of the m_buffer[]
                   // position of first byte after chunk's length.
  unsigned int m_typecode;
  int m_value;
  int m_do_length; // true if chunk is a long chunk with length
  ON__UINT16 m_do_crc16; // 16 bit CRC using CCITT polynomial
  ON__UINT16 m_crc16;
  ON__UINT32 m_do_crc32; // 32 bit CRC
  ON__UINT32 m_crc32;
};

struct ON_3DM_BIG_CHUNK
{
  ON__UINT64 m_big_offset; // In read or write_using_fseek mode, this is the
                           // file position of first byte after chunk's length.
                           // In write_using_buffer mode, this of the m_buffer[]
                           // position of first byte after chunk's length.

  ON__UINT64 Length() const; // 0 for short chunks

  ON__INT64 m_big_value;
  ON__UINT32 m_typecode;

  ON__UINT8 m_bLongChunk; // true if chunk is a long chunk and m_big_value is a length.
  ON__UINT8 m_reserved1;
  ON__UINT8 m_reserved2;
  ON__UINT8 m_reserved3;

  // CRC settings
  ON__UINT8 m_do_crc16; // true (1) if we are calculating 16 bit CRC
  ON__UINT8 m_do_crc32; // true (1) if we are calculating 32 bit CRC
  ON__UINT16 m_crc16; // current 16 bit CRC value
  ON__UINT32 m_crc32; // current 32 bit CRC value
};

bool ON_IsLongChunkTypecode(ON__UINT32 typecode);

bool ON_IsShortChunkTypecode(ON__UINT32 typecode);

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_3DM_CHUNK>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_3DM_BIG_CHUNK>;
#pragma warning( pop )
#endif

class ON_Light;
class ON_Bitmap;
class ON_TextureMapping;
class ON_Material;
class ON_Layer;
class ON_3dmProperties;
class ON_3dmSettings;
class ON_3dmObjectAttributes;
class ON_3dmGoo;

class ON_BinaryArchive;

// Used int ON_3dmProperties::Read() to set ON_BinaryArchive.m_3dm_opennurbs_version
// Do not call directly. 
void ON_SetBinaryArchiveOpenNURBSVersion(ON_BinaryArchive&,int);

class ON_CLASS ON_BinaryArchive // use for generic serialization of binary data
{
public:
  ON_BinaryArchive( ON::archive_mode );
  virtual ~ON_BinaryArchive();

  virtual 
  std::size_t CurrentPosition( // current offset (in bytes) into archive ( like ftell() )
                ) const = 0; 
  virtual 
  bool SeekFromCurrentPosition( // seek from current position ( like fseek( ,SEEK_CUR) )
                int // byte offset ( >= -CurrentPostion() )
                ) = 0; 
  virtual 
  bool SeekFromStart(  // seek from current position ( like fseek( ,SEEK_SET) )
                std::size_t // byte offset ( >= 0 )
                ) = 0;
  virtual 
  bool AtEnd() const = 0; // true if at end of file

  bool BigSeekFromStart( ON__UINT64 offset );
  bool BigSeekForward( ON__UINT64 offset );
  bool BigSeekBackward( ON__UINT64 offset );
  bool BigSeekFromCurrentPosition( ON__INT64 offset );

  /*
  Description:
    Tool for swapping bytes when doing I/O on
    using big endian CPUs.
  Remarks:
    3dm files are always saved with little endian byte order.
  See Also:
    ON_BinaryArchive::Endian
  */
  static
  bool ToggleByteOrder(
    int, // number of elements
    int, // size of element (2,4, or 8)
    const void*,  // source buffer
    void*         // destination buffer (can be same a source buffer)
    );

  static
  const char* TypecodeName( unsigned int tcode );

  static
  char* ON_TypecodeParse( unsigned int tcode, char* typecode_name, std::size_t max_length );

  bool ReadMode() const;  // true if reading is permitted
  bool WriteMode() const; // true if writing is permitted
  
  /*
  Returns:
     Endian-ness of the cpu reading this file.
  Remarks:
    3dm files are alwasy saved with little endian byte order.
  */
  ON::endian Endian() const; // endian-ness of cpu

  int BadCRCCount() const; // number of chunks read with bad CRC 

  bool ReadByte( std::size_t, void* ); // must fail if mode is not read or readwrite

  bool WriteByte( std::size_t, const void* ); // must fail if mode is not write or readwrite

  /*
  Description:
    Expert user function that uses Read() to load a buffer.
  Paramters:
    sizeof_buffer - [in] number of bytes to attempt to read.
    buffer - [out] read bytes are stored in this buffer
  Returns:
    Number of bytes actually read, which may be less than
    sizeof_buffer if the end of file is encountered.
  */
  ON__UINT64 ReadBuffer( ON__UINT64 sizeof_buffer, void* buffer );

  /*
  Description:
    Expert user function to control CRC calculation while reading and writing.
    Typically this is used when seeking around and reading/writing information
    in non-serial order.
  Parameters:
    bEnable  - [in]
  Returns:
    Current state of CRC calculation.  Use the returned value to restore the
    CRC calculation setting after you are finished doing your fancy pants
    expert IO.
  */
  bool EnableCRCCalculation( bool bEnable );

  // ReadCompressedBuffer()/WriteCompressedBuffer() use zlib 1.1.3
  // to inflate/deflate the data buffer.
  // Care must be used to get an endian independent file.  
  // See ON_Mesh::Read()/ON_Mesh::Write() for an example of an endian
  // independent use of compression. See also ToggleByteOrder() and Endian().
  //
  // To read data archived by WriteCompressedBuffer( sizeof_buffer, buffer )
  // do something like:
  //
  //   std::size_t sizeof_buffer = 0;
  //   ReadCompressedBufferSize(&sizeof_buffer);
  //   buffer = something with sizeof_buffer bytes.
  //   int bFailedCRC = false;
  //   bool ok = ReadCompressedBuffer( sizeof_buffer, buffer, &bFailedCRC );
  //


  /*
  Description:
    Red the size of a compressed buffer.
  Parameters:
    sizeof__outbuffer - [out] size of the uncompressed buffer in bytes
  Returns:
    True if read was successful.
  */
  bool ReadCompressedBufferSize( std::size_t* sizeof__outbuffer );

  /*
  Description:
    Read compressed information from an archive and uncompress it.
  Parameters:
    sizeof__outbuffer - [in] size of the uncompressed buffer in bytes
    outbuffer - [out] uncompressed buffer returned here
    bFailedCRC - [out] true if cyclic redundancy check fails
                      on uncompressed buffer

  Example:

          std::size_t sizeof_buffer = 0;
          ReadCompressedBufferSize(&sizeof_buffer);
          buffer = ...; // something with sizeof_buffer bytes.
          int bFailedCRC = false;
          bool ok = ReadCompressedBuffer( sizeof_buffer, buffer, &bFailedCRC );

  Returns:
    True if read was successful.  You need to check the value
    of bFailedCRC to see if the information that was read is valid.
  */
  bool ReadCompressedBuffer(
          std::size_t sizeof__outbuffer,
          void* outbuffer,
          int* bFailedCRC
          );

  /*
  Description:
    Compress buffer and write the compressed information to the archive.
  Parameters:
    sizeof__inbuffer - [in] size of the uncompressed buffer in bytes
    inbuffer - [in] uncompressed buffer
  Returns:
    True if write was successful.
  */
  bool WriteCompressedBuffer(
    std::size_t sizeof__inbuffer,
    const void* inbuffer
    );

  bool ReadBool( bool* );

	bool ReadChar(    // Read an array of 8 bit chars
			std::size_t,       // number of chars to read
			char*    
			);  
	bool ReadChar(    // Read an array of 8 bit unsigned chars
			std::size_t,       // number of unsigned chars to read
			unsigned char*    
			);  
	bool ReadChar(    // Read a single 8 bit char
			char*    
			);  
	bool ReadChar(    // Read a single 8 bit unsigned char
			unsigned char*    
			);  

	bool ReadShort(   // Read an array of 16 bit shorts
			std::size_t,       // number of shorts to read
			short*    
			);  
	bool ReadShort(   // Read an array of 16 bit unsigned shorts
			std::size_t,       // number of shorts to read
			unsigned short*    
			);  
	bool ReadShort(   // Read a single 16 bit short
			short*    
			);  
	bool ReadShort(   // Read a single 16 bit unsigned short
			unsigned short*    
			);  

	bool ReadInt( // Read an array of 32 bit integers
			std::size_t,	      // number of ints to read
			int*      
			); 
	bool ReadInt( // Read an array of 32 bit integers
			std::size_t,	      // number of ints to read
			unsigned int*      
			); 
	bool ReadInt( // Read a single 32 bit integer
			int*      
			); 
	bool ReadInt( // Read a single 32 bit unsigned integer
			unsigned int*      
			); 

	bool ReadBigInt( // Read an array of 64 bit integers
			std::size_t,	      // number of ints to read
			ON__INT64*      
			); 
	bool ReadBigInt( // Read an array of 64 bit integers
			std::size_t,	      // number of ints to read
			ON__UINT64*      
			); 
	bool ReadBigInt( // Read a single 64 bit integer
			ON__INT64*      
			); 
	bool ReadBigInt( // Read a single 64 bit unsigned integer
			ON__UINT64*      
			); 

	bool ReadLong( // Read an array of 32 bit integers
			std::size_t,	      // number of ints to read
			long*      
			); 
	bool ReadLong( // Read an array of 32 bit integers
			std::size_t,	      // number of ints to read
			unsigned long*      
			); 
	bool ReadLong( // Read a single 32 bit integer
			long*      
			); 
	bool ReadLong( // Read a single 32 bit unsigned integer
			unsigned long*      
			); 
	bool ReadSize( // Read a single std::size_t
			std::size_t*
			); 

  bool ReadBigSize( std::size_t* ); // 64 bits
  
  bool ReadBigTime( time_t* ); // UCT seconds since 1 January 1970 (64 bits)


	bool ReadFloat(   // Read an array of floats
			std::size_t,       // number of floats
			float*
			);
	bool ReadFloat(   // Read a single float
			float*
			);
	bool ReadDouble(  // Read an array of IEEE doubles
			std::size_t,       // number of doubles
			double*
			);
	bool ReadDouble(  // Read a single double
			double*
			);

  bool ReadColor(
    ON_Color&
    );

  bool ReadPoint (
    ON_2dPoint&
    );
  bool ReadPoint (
    ON_3dPoint&
    );
  bool ReadPoint (
    ON_4dPoint&
    );
  bool ReadVector (
    ON_2dVector&
    );
  bool ReadVector (
    ON_3dVector&
    );

  bool ReadBoundingBox(ON_BoundingBox&);

  bool ReadXform(ON_Xform&);

  bool ReadPlaneEquation(ON_PlaneEquation&);

  bool ReadPlane(ON_Plane&);

  bool ReadLine(ON_Line&);

  bool ReadArc(ON_Arc&);

  bool ReadCircle(ON_Circle&);

  bool ReadInterval( ON_Interval& );

  bool ReadUuid( ON_UUID& );

  bool ReadDisplayMaterialRef( ON_DisplayMaterialRef& );

  bool ReadLinetypeSegment( ON_LinetypeSegment& );

  // All times are stored in coordinated universal time
  // ( a.k.a GMT, UTC ).  Use ANSI C time() and gmtime() calls.
  bool ReadTime( struct tm& );

  /*
  Parameters:
    str_array_count - [out]
      Number of elements in the string array. All ON_BinaryArchive string
      WriteString() functions write a null terminator to the file and
      the null terminator is included in the count. This means that
      if a string has a non-zero element, then str_array_count >= 2.
  Remarks:
    Modify your code to use ReadStringUTF8ElementCount() when reading
    UTF-8 encoded strings and ReadStringUTF16ElementCount()
    when reading UTF-16 encoded strings.
  */
  ON_DEPRECATED bool ReadStringSize(
      std::size_t* str_array_count
      );

  /*
  Parameters:
    string_utf8_element_count - [out]
      Number of bytes in the string array. All ON_BinaryArchive string
      WriteString() functions write a null terminator to the file and
      the null terminator is included in string_element_count. This means
      that if opennurbs wrote the string, either string_element_count = 0
      or string_element_count >= 2.
  */
  bool ReadStringUTF8ElementCount(
    std::size_t* string_utf8_element_count
    );

  /*
  Parameters:
    string_utf16_element_count - [out]
      Number of elements in the string array. All ON_BinaryArchive string
      WriteString() functions write a null terminator to the file and
      the null terminator is included in string_element_count. This means
      that if opennurbs wrote the string, either string_element_count = 0
      or string_element_count >= 2.
  */
  bool ReadStringUTF16ElementCount(
    std::size_t* string_utf16_element_count
    );


  /*
  Parameters:
    str_array_count - [in]
      Number of char elements in str_array[], including the null
      terminator.  The value of str_array_count is returned by
      ReadCharStringElementCount().
    str_array - [in/out]
      Pass in an array with at least str_array_count elements.
      If true is returned and str_array_count > 0,
      then str_array[str_array_count-1] = 0. All strings with
      char elements written by Rhino are UTF-8 encoded
      unicode strings.
  */
  bool ReadString(
      std::size_t str_array_count,
      char* str_array
      );

  /*
  Parameters:
    str_array_count - [in]
      Number of unsignd char elements in str_array[], including
      the null terminator. The value of str_array_count is returned
      by ReadCharStringElementCount().
    str_array - [in/out]
      Pass in an array with at least str_array_count elements.
      If true is returned and str_array_count > 0,
      then str_array[str_array_count-1] = 0. All strings with
      unsigned char elements written by Rhino are UTF-8 encoded 
      unicode strings.
  */
  bool ReadString(
      std::size_t str_array_count,
      unsigned char* str_array
      );

  /*
  Parameters:
    str_array_count - [in]
      Number of unsigned short elements in str_array[],
      including the null terminator. The value of 
      str_array_count is returned by ReadWideCharStringElementCount().
    str_array - [in/out]
      Pass in an array with at least str_array_count elements.
      If true is returned and str_array_count > 0,
      then str_array[str_array_count-1] = 0. All strings with
      unsigned short elements written by Rhino are UTF-16 encoded
      unicode strings.
  */
  bool ReadString(
      std::size_t str_array_count,
      unsigned short*  str_array
      );

  bool ReadString( ON_String& sUTF8 );

  bool ReadString( ON_wString& s );

  bool ReadComponentIndex( ON_COMPONENT_INDEX& );

  bool ReadArray( ON_SimpleArray<bool>& );
  bool ReadArray( ON_SimpleArray<char>& );
  bool ReadArray( ON_SimpleArray<short>& );
  bool ReadArray( ON_SimpleArray<int>& );
  bool ReadArray( ON_SimpleArray<float>& );
  bool ReadArray( ON_SimpleArray<double>& );
  bool ReadArray( ON_SimpleArray<ON_Color>& );
  bool ReadArray( ON_SimpleArray<ON_2dPoint>& );
  bool ReadArray( ON_SimpleArray<ON_3dPoint>& );
  bool ReadArray( ON_SimpleArray<ON_4dPoint>& );
  bool ReadArray( ON_SimpleArray<ON_2dVector>& );
  bool ReadArray( ON_SimpleArray<ON_3dVector>& );
  bool ReadArray( ON_SimpleArray<ON_Xform>& );
  bool ReadArray( ON_SimpleArray<ON_2fPoint>& );
  bool ReadArray( ON_SimpleArray<ON_3fPoint>& );
  bool ReadArray( ON_SimpleArray<ON_4fPoint>& );
  bool ReadArray( ON_SimpleArray<ON_2fVector>& );
  bool ReadArray( ON_SimpleArray<ON_3fVector>& );
  bool ReadArray( ON_SimpleArray<ON_UUID>& );
  bool ReadArray( ON_SimpleArray<ON_UuidIndex>& );
  bool ReadArray( ON_SimpleArray<ON_SurfaceCurvature>& );
  bool ReadArray( ON_ClassArray<ON_String>& );
  bool ReadArray( ON_ClassArray<ON_wString>& );
  bool ReadArray( ON_SimpleArray<ON_DisplayMaterialRef>& );
  bool ReadArray( ON_SimpleArray<ON_LinetypeSegment>& );  
  bool ReadArray( ON_SimpleArray<ON_MappingChannel>& );
  bool ReadArray( ON_ClassArray<ON_MaterialRef>& );
  bool ReadArray( ON_ClassArray<ON_MappingRef>& );
  bool ReadArray( ON_ClassArray<class ON_ObjRef>& );
  bool ReadArray( ON_SimpleArray<class ON_ObjRef_IRefID>& );
  bool ReadArray( ON_SimpleArray<class ON_ClippingPlaneInfo>& );
  bool ReadArray( ON_ObjectArray<class ON_Layer>& );
  bool ReadArray( ON_SimpleArray<class ON_Layer*>& );

  bool WriteBool( bool );

  bool WriteChar(    // Write an array of 8 bit chars
			std::size_t,       // number of chars to write
			const char*    
			);  
	bool WriteChar(    // Write an array of 8 bit unsigned chars
			std::size_t,       // number of unsigned chars to write
			const unsigned char*    
			);  
	bool WriteChar(    // Write a single 8 bit char
			char
			);  
	bool WriteChar(    // Write a single 8 bit unsigned char
			unsigned char
			);  

	bool WriteShort(   // Write an array of 16 bit shorts
			std::size_t,       // number of shorts to write
			const short*    
			);  
	bool WriteShort(   // Write an array of 16 bit unsigned shorts
			std::size_t,       // number of shorts to write
			const unsigned short*    
			);  
	bool WriteShort(   // Write a single 16 bit short
			short
			);  
	bool WriteShort(   // Write a single 16 bit unsigned short
			unsigned short
			);  

	bool WriteInt( // Write an array of 32 bit integers
			std::size_t,	      // number of ints to write
			const int*      
			); 
	bool WriteInt( // Write an array of 32 bit integers
			std::size_t,	      // number of ints to write
			const unsigned int*      
			); 
	bool WriteInt( // Write a single 32 bit integer
			int    
			); 
	bool WriteInt( // Write a single 32 bit unsigned integer
			unsigned int
			); 

	bool WriteBigInt( // Write an array of 64 bit integers
			std::size_t,	      // number of ints to write
			const ON__INT64*      
			); 
	bool WriteBigInt( // Write an array of 64 bit integers
			std::size_t,	      // number of ints to write
			const ON__UINT64*      
			); 
	bool WriteBigInt( // Write a single 64 bit integer
			ON__INT64    
			); 
	bool WriteBigInt( // Write a single 64 bit unsigned integer
			ON__UINT64
			); 

	bool WriteLong( // Write an array of 32 bit integers
			std::size_t,	      // number of ints to write
			const long*      
			); 
	bool WriteLong( // Write an array of 32 bit integers
			std::size_t,	      // number of ints to write
			const unsigned long*      
			); 
	bool WriteLong( // Write a single 32 bit integer
			long    
			); 
	bool WriteLong( // Write a single 32 bit unsigned integer
			unsigned long
			); 
	bool WriteSize( // Write a single std::size_t
			std::size_t
			); 

  bool WriteBigSize( std::size_t ); // 64 bits 
  
  bool WriteBigTime( time_t ); // UCT seconds since 1 January 1970 (64 bits)

	bool WriteFloat(   // Write a number of IEEE floats
			std::size_t,       // number of doubles
			const float*
			);
	bool WriteFloat(   // Write a single float
			float
			);
	bool WriteDouble(  // Write a single double
      std::size_t,
			const double*
			);
	bool WriteDouble(  // Write a single double
			double
			);

  bool WriteColor (
    const ON_Color&
    );

  bool WritePoint (
    const ON_2dPoint&
    );
  bool WritePoint (
    const ON_3dPoint&
    );
  bool WritePoint (
    const ON_4dPoint&
    );
  bool WriteVector (
    const ON_2dVector&
    );
  bool WriteVector (
    const ON_3dVector&
    );

  bool WriteBoundingBox(const ON_BoundingBox&);

  bool WriteXform(const ON_Xform&);

  bool WritePlaneEquation(const ON_PlaneEquation&);

  bool WritePlane(const ON_Plane&);

  bool WriteLine(const ON_Line&);

  bool WriteArc(const ON_Arc&);

  bool WriteCircle(const ON_Circle&);

  bool WriteInterval( const ON_Interval& );

  bool WriteUuid( const ON_UUID& );

  bool WriteDisplayMaterialRef( const ON_DisplayMaterialRef& );

  bool WriteLinetypeSegment( const ON_LinetypeSegment& );

  // All times are stored in universal coordinated time
  // ( a.k.a GMT, UCT ).  Use ANSI C time() and gmtime() calls.
  bool WriteTime( const struct tm& );

  /*
  Parameters:
    sUTF8 - [in]
      A null terminated UTF-8 encoded unicode string.
  Remarks:
    To read a string written with WriteString(const char*),
    call ReadStringUTF8ElementCount(&string_utf8_element_count)
    to get the number of char elements written in the file,
    obtain a buffer with at least string_utf8_element_count
    char elements and then call 
    ReadString(string_utf8_element_count,buffer) to read the
    char elements.

    If 0 == sUTF8 or 0 == SUTF8[0], a 4 byte int with
    value = 0 is written, otherwise a 4 byte int with
    value = strlen + 1 is written, followed by the string,
    followed by the null terminator.
  */
  bool WriteString(
      const char* sUTF8         
      );

  /*
  Parameters:
    sUTF8 - [in]
      A null terminated UTF-8 encoded unicode string.
  Remarks:
    To read a string written with WriteString(const unsigned char*),
    call ReadStringUTF8ElementCount(&string_utf8_element_count) to
    get the number of unsigned char elements written in the file,
    obtain a buffer with at least string_utf8_element_count
    unsigned char elements and then call 
    ReadString(string_utf8_element_count,buffer) to read the 
    unsigned charelements.

    If 0 == sUTF8 or 0 == SUTF8[0], a 4 byte int with
    value = 0 is written, otherwise a 4 byte int with
    value = strlen + 1 is written, followed by the string,
    followed by the null terminator.
  */
  bool WriteString(
      const unsigned char* sUTF8
      );

  /*
  Parameters:
    sUTF16 - [in]
      A null terminated UTF-16 encoded unicode string.
  Remarks:
    To read a string written with WriteString(const unsigned short*),
    call ReadStringUTF16ElementCount(&string_utf16_element_count) to
    get the number of unsigned short elements written in the file,
    obtain a buffer with at least string_utf16_element_count
    unsigned short elements and then call 
    ReadString(string_utf16_element_count,buffer) to read the
    unsigned short elements.

    If 0 == sUTF8 or 0 == SUTF8[0], a 4 byte int with
    value = 0 is written, otherwise a 4 byte int with
    value = strlen + 1 is written, followed by the string,
    followed by the null terminator.
  */
  bool WriteString(
      const unsigned short* sUTF16
      );
  
  bool WriteString( const ON_String& sUTF8 );

  bool WriteString( const ON_wString& s);

  bool WriteComponentIndex( const ON_COMPONENT_INDEX& );

  bool WriteArray( const ON_SimpleArray<bool>& );
  bool WriteArray( const ON_SimpleArray<char>& );
  bool WriteArray( const ON_SimpleArray<short>& );
  bool WriteArray( const ON_SimpleArray<int>& );
  bool WriteArray( const ON_SimpleArray<float>& );
  bool WriteArray( const ON_SimpleArray<double>& );

  bool WriteArray( const ON_SimpleArray<ON_Color>& );

  bool WriteArray( const ON_SimpleArray<ON_2dPoint>& );
  bool WriteArray( const ON_SimpleArray<ON_3dPoint>& );
  bool WriteArray( const ON_SimpleArray<ON_4dPoint>& );
  bool WriteArray( const ON_SimpleArray<ON_2dVector>& );
  bool WriteArray( const ON_SimpleArray<ON_3dVector>& );

  bool WriteArray( const ON_SimpleArray<ON_2fPoint>& );
  bool WriteArray( const ON_SimpleArray<ON_3fPoint>& );
  bool WriteArray( const ON_SimpleArray<ON_4fPoint>& );
  bool WriteArray( const ON_SimpleArray<ON_2fVector>& );
  bool WriteArray( const ON_SimpleArray<ON_3fVector>& );
  bool WriteArray( const ON_SimpleArray<ON_Xform>& );
  bool WriteArray( const ON_SimpleArray<ON_UUID>& );
  bool WriteArray( const ON_SimpleArray<ON_UuidIndex>& );
  bool WriteArray( const ON_SimpleArray<ON_SurfaceCurvature>& );
  bool WriteArray( const ON_ClassArray<ON_String>& );
  bool WriteArray( const ON_ClassArray<ON_wString>& );
  bool WriteArray( const ON_SimpleArray<ON_DisplayMaterialRef>& );
  bool WriteArray( const ON_SimpleArray<ON_LinetypeSegment>& );  
  bool WriteArray( const ON_SimpleArray<ON_MappingChannel>& );
  bool WriteArray( const ON_ClassArray<ON_MaterialRef>& );
  bool WriteArray( const ON_ClassArray<ON_MappingRef>& );
  bool WriteArray( const ON_ClassArray<class ON_ObjRef>& );
  bool WriteArray( const ON_SimpleArray<class ON_ObjRef_IRefID>& );
  bool WriteArray( const ON_SimpleArray<class ON_ClippingPlaneInfo>& );
  bool WriteArray( int count, const class ON_Layer* );
  bool WriteArray( int count, const class ON_Layer*const* );

  /////////////////////////////////////////////////////
  //
  // Read/Write classes derived from ON_Object
  //

  /*
  Description:
    Reads and object from a 3dm archive;
  Parameters:
    ppObject - [out]  object is allocated and a pointer to the
                      allocated object is returned as *ppObject;
  Returns:
    0: failure - unable to read object because of file IO problems
    1: success
    3: unable to read object because it's UUID is not registered
       this could happen in cases where old code is attempting to read
       new objects.
  */
  int ReadObject( 
         ON_Object** ppObject
         );


  /*
  Description:
    Reads and object from a 3dm archive.
  Parameters:
    object - [in] The value of object.ON_ClassId()->Uuid() must
                  exactly match the class uuid in of the next
                  object in the archive.
  Returns:
    0: failure - unable to read object because of file IO problems.
    1: success
    2: unable to read object because the class id in the archive
       did not match pObject->ClassId.
  */
  int ReadObject( 
         ON_Object& object
         );

  bool WriteObject( const ON_Object* ); // writes object definition
  bool WriteObject( const ON_Object& ); // writes object definition


  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  //
  // 3DM Interface - ignore if not reading/writing a 3DM file
  //                 this is here so that the infrastructure
  //                 for writing 3dm archives is available for
  //                 any type of serialization device.
  //
  bool EnableSave3dmRenderMeshes( ON_BOOL32 = true ); // returns previous state
  bool Save3dmRenderMeshes() const;

  bool EnableSave3dmAnalysisMeshes( ON_BOOL32 = true ); // returns previous state
  bool Save3dmAnalysisMeshes() const;
  
  bool EnableSaveUserData( ON_BOOL32 = true ); // returns previous state
  bool SaveUserData() const;
  
  /*
  Returns:
    50 (The Rhino 5.0 opennurbs file version.)
    This is the value of version to pass to ON_BinaryArchive
    functions like Write3dmStartSection() when you want to use the 
    the current opennurbs version number and you do not want to have
    to update your code when this version number changes.    
  */
  static int CurrentArchiveVersion();

  ///////////////////////////////////////////////////////////////////
  // Step 1: REQUIRED - Write/Read Start Section
  //

  /*
  Parameters:
    version - [in]
       0, 2, 3, 4, 5 or 50 (5 is treated as 50)
       
       If version is 0, then the value of ON_BinaryArchive::CurrentArchiveVersion()
       is used.

       Use either 0 or the value of ON_BinaryArchive::CurrentArchiveVersion()
       for the version parameter when you want your code to write the most 
       up to date file version. 

    sStartSectionComment - [in]
      NULL or ASCII string with application name, et cetera.
      This information is primarily used when debugging files
      that contain problems.  McNeel and Associates stores
      application name, application version, compile date, 
      and the OS in use when file was written.
  */
  bool Write3dmStartSection( 
        int version,
        const char* sStartSectionComment
        );

  /*
  Parameters:
    version - [out]
       .3dm file version (2, 3, 4, 5 or 50)
    sStartSectionComment - [out]
      string passed to Write3dmStartSection()
  */
  bool Read3dmStartSection( 
        int* version,
        ON_String& sStartSectionComment
        );

  ///////////////////////////////////////////////////////////////////
  // Step 2: REQUIRED - Write/Read properties table
  //
  bool Write3dmProperties(
        const ON_3dmProperties&
        );
  bool Read3dmProperties(
        ON_3dmProperties&
        );

  ///////////////////////////////////////////////////////////////////
  // Step 3: REQUIRED - Write/Read settings table
  //
  bool Write3dmSettings(
        const ON_3dmSettings&
        );
  bool Read3dmSettings(
        ON_3dmSettings&
        );

  ///////////////////////////////////////////////////////////////////
  // Step 4: REQUIRED - Write/Read bitmap table (it can be empty)
  //
  bool BeginWrite3dmBitmapTable();
  bool Write3dmBitmap( const ON_Bitmap& );
  bool EndWrite3dmBitmapTable();

  bool BeginRead3dmBitmapTable();
  int  Read3dmBitmap(   // returns 0 at end of light table
                        //         1 bitmap successfully read
            ON_Bitmap** // bitmap returned here
            );
  bool EndRead3dmBitmapTable();

  ///////////////////////////////////////////////////////////////////
  // Step 5: REQUIRED - Write/Read render material table (it can be empty)
  //
  bool BeginWrite3dmTextureMappingTable();
  bool Write3dmTextureMapping( const ON_TextureMapping& );
  bool EndWrite3dmTextureMappingTable();

  bool BeginRead3dmTextureMappingTable();
  int  Read3dmTextureMapping( // returns 0 at end of table
            ON_TextureMapping** // layer returned here
            );
  bool EndRead3dmTextureMappingTable();

  ///////////////////////////////////////////////////////////////////
  // Step 6: REQUIRED - Write/Read render material table (it can be empty)
  //
  bool BeginWrite3dmMaterialTable();
  bool Write3dmMaterial( const ON_Material& );
  bool EndWrite3dmMaterialTable();

  bool BeginRead3dmMaterialTable();
  int  Read3dmMaterial( // returns 0 at end of table
            ON_Material** // layer returned here
            );
  bool EndRead3dmMaterialTable();

  ///////////////////////////////////////////////////////////////////
  // Step 7: REQUIRED - Write/Read linetype table (it can be empty)
  //
  bool BeginWrite3dmLinetypeTable();
  bool Write3dmLinetype( const ON_Linetype&);
  bool EndWrite3dmLinetypeTable();

  bool BeginRead3dmLinetypeTable();
  int  Read3dmLinetype(ON_Linetype**);
  bool EndRead3dmLinetypeTable();

  ///////////////////////////////////////////////////////////////////
  // Step 8: REQUIRED - Write/Read layer table (it can be empty)
  //
  bool BeginWrite3dmLayerTable();
  bool Write3dmLayer( const ON_Layer& );
  bool EndWrite3dmLayerTable();

  bool BeginRead3dmLayerTable();
  int  Read3dmLayer( // returns 0 at end of table
            ON_Layer** // layer returned here
            );
  bool EndRead3dmLayerTable();

  ///////////////////////////////////////////////////////////////////
  // Step 9: REQUIRED - Write/Read group table (it can be empty)
  //
  bool BeginWrite3dmGroupTable();
  bool Write3dmGroup( const ON_Group& );
  bool EndWrite3dmGroupTable();

  bool BeginRead3dmGroupTable();

  // Description:
  //   Reads groups from group table.  If the group definition is
  //   read, a group is created by calling new ON_Group(),
  //   initialized with values stored in the archive, and 
  //   returned.
  //
  // Parameters:
  //   ppGroup - If the group definition is
  //   read, a group is created by calling new ON_Group(),
  //   initialized with values stored in the archive, and 
  //   a pointer to the new group is returned in *ppGroup.
  //
  // Returns:
  //
  //   @untitled table
  //   0     at the end of the group table
  //   1     group definition was successfully read
  //   -1    archive is corrupt at this point
  //
  // Example:
  //   Calls to Read3dmGroup need to be bracketed by calls
  //   to BeginRead3dmGroupTable() / EndRead3dmGroupTable().
  //
  //           archive.BeginRead3dmGroupTable();
  //           ON_Group* pGroup;
  //           int rc = 1;
  //           while(rc==1)
  //           { //
  //             pGroup = 0;
  //             archive.Read3dmGroup(&pGroup);
  //             if ( pGroup )
  //               do something with pGroup
  //           } //
  //           archive.EndRead3dmGroupTable();
  //      
  int  Read3dmGroup(
            ON_Group** // ppGroup
            );

  bool EndRead3dmGroupTable();


  ///////////////////////////////////////////////////////////////////
  // Step 10: REQUIRED - Write/Read font table (it can be empty)
  //
  bool BeginWrite3dmFontTable();
  bool Write3dmFont( const ON_Font& );
  bool EndWrite3dmFontTable();

  bool BeginRead3dmFontTable();

  // Description:
  //   Reads fonts from font table.  If the font definition is
  //   read, a font is created by calling new ON_Font(),
  //   initialized with values stored in the archive, and 
  //   returned.
  //
  // Parameters:
  //   ppFont - If the font definition is
  //   read, a font is created by calling new ON_Font(),
  //   initialized with values stored in the archive, and 
  //   a pointer to the new font is returned in *ppFont.
  //
  // Returns:
  //
  //   @untitled table
  //   0     at the end of the font table
  //   1     font definition was successfully read
  //   -1    archive is corrupt at this point
  //
  // Example:
  //   Calls to Read3dmFont need to be bracketed by calls
  //   to BeginRead3dmFontTable() / EndRead3dmFontTable().
  //
  //           archive.BeginRead3dmFontTable();
  //           int rc = 1;
  //           ON_Font* pFont;
  //           while(rc==1)
  //           { //
  //             pFont = 0;
  //             archive.Read3dmFont(&pFont);
  //             if ( pFont )
  //               do something with pFont
  //           } //
  //           archive.EndRead3dmFontTable();
  //      
  int Read3dmFont(
            ON_Font** // ppFont
            );

  bool EndRead3dmFontTable();


  ///////////////////////////////////////////////////////////////////
  // Step 11: REQUIRED - Write/Read dimstyle table (it can be empty)
  //
  bool BeginWrite3dmDimStyleTable();
  bool Write3dmDimStyle( const ON_DimStyle& );
  bool EndWrite3dmDimStyleTable();

  bool BeginRead3dmDimStyleTable();

  // Description:
  //   Reads annotation dimension styles from dimension style table.
  //   If the dimension style definition is read, 
  //   a dimension style is created by calling new ON_DimStyle(),
  //   initialized with values stored in the archive, and 
  //   returned.
  //
  // Parameters:
  //   ppDimStyle - If the dimstyle definition is
  //   read, a dimstyle is created by calling new ON_DimStyle(),
  //   initialized with values stored in the archive, and 
  //   a pointer to the new dimstyle is returned in *ppDimStyle.
  //
  // Returns:
  //
  //   @untitled table
  //   0     at the end of the dimension style table
  //   1     dimension style definition was successfully read
  //   -1    archive is corrupt at this point
  //
  // Example:
  //   Calls to Read3dmDimStyle need to be bracketed by calls
  //   to BeginRead3dmDimStyleTable() / EndRead3dmDimStyleTable().
  //
  //           archive.BeginRead3dmDimStyleTable();
  //           int rc = 1;
  //           ON_DimStyle* pDimStyle;
  //           while(rc==1)
  //           { //
  //             pDimStyle = 0;
  //             archive.Read3dmDimStyle(&pDimStyle);
  //             if ( pDimStyle )
  //               do something with pDimStyle
  //           } //
  //           archive.EndRead3dmDimStyleTable();
  //      
  int Read3dmDimStyle(
            ON_DimStyle** // ppDimStyle
            );

  bool EndRead3dmDimStyleTable();


  ///////////////////////////////////////////////////////////////////
  // Step 12: REQUIRED - Write/Read render light table (it can be empty)
  //
  bool BeginWrite3dmLightTable();
  bool Write3dmLight( const ON_Light&,
         const ON_3dmObjectAttributes* // optional
         );
  bool EndWrite3dmLightTable();

  bool BeginRead3dmLightTable();
  int  Read3dmLight(  // returns 0 at end of light table
                      //         1 light successfully read
                      //        -1 if file is corrupt
            ON_Light**, // light returned here
            ON_3dmObjectAttributes* // optional - if NOT NULL, object attributes are
                                    //            returned here
            );
  bool EndRead3dmLightTable();


  ///////////////////////////////////////////////////////////////////
  // Step 13: REQUIRED - Write/Read hatch pattern table (it can be empty)
  //
  bool BeginWrite3dmHatchPatternTable();
  bool Write3dmHatchPattern( const ON_HatchPattern&);
  bool EndWrite3dmHatchPatternTable();

  bool BeginRead3dmHatchPatternTable();
  int  Read3dmHatchPattern(ON_HatchPattern**);
  bool EndRead3dmHatchPatternTable();

  ///////////////////////////////////////////////////////////////////
  // Step 14: REQUIRED - Write/Read instance definition table (it can be empty)
  //
  bool BeginWrite3dmInstanceDefinitionTable();
  bool Write3dmInstanceDefinition( const ON_InstanceDefinition& );
  bool EndWrite3dmInstanceDefinitionTable();

  bool BeginRead3dmInstanceDefinitionTable();

  /*
   Description:
     Reads instance definitions from instance defintion table.
  
   Parameters:
     ppInstanceDefinition - If an instance defintion is
     read, an instance defintion is created by calling new 
     ON_InstanceDefinition(), initialized with values stored
     in the archive, and a pointer to the new instance defintion
     is returned in *ppInstanceDefinition.
  
   Returns:
  
     @untitled table
     0     at the end of the instance defintion table
     1     instance defintion was successfully read
     -1    archive is corrupt at this point
  
   Example:
     Calls to Read3dmInstanceDefinition need to be bracketed by calls
     to BeginRead3dmInstanceDefinitionTable() / EndRead3dmInstanceDefinitionTable().
  
             archive.BeginRead3dmInstanceDefinitionTable();
             int rc = 1;
             ON_InstanceDefinition* pInstanceDefinition;
             while(rc==1)
             { 
               pInstanceDefinition = 0;
               archive.Read3dmInstanceDefinition(&pInstanceDefinition);
               if ( pInstanceDefinition )
                 do something with pInstanceDefinition
             } 
             archive.EndRead3dmInstanceDefinitionTable();
  */      
  int Read3dmInstanceDefinition(
            ON_InstanceDefinition** // ppInstanceDefinition
            );

  bool EndRead3dmInstanceDefinitionTable();

  ///////////////////////////////////////////////////////////////////
  // Step 15: REQUIRED - Write/Read geometry and annotation table (it can be empty)
  //
  bool BeginWrite3dmObjectTable();
  bool Write3dmObject( 
         const ON_Object&,
         const ON_3dmObjectAttributes* // optional
         );
  bool EndWrite3dmObjectTable();

  bool BeginRead3dmObjectTable();
  int  Read3dmObject( // returns 0 at end of object table
                      //         1 if object is read
                      //         2 if object is skipped because it does not match filter
                      //        -1 if file is corrupt
          ON_Object**, // object returned here (NULL if skipped)
          ON_3dmObjectAttributes*, // optional - if NOT NULL, object attributes are
                                   //            returned here
          unsigned int = 0 // optional filter made by setting ON::object_type bits
          );  // returns NULL at end of object table
  bool EndRead3dmObjectTable();

  ///////////////////////////////////////////////////////////////////
  // Step 16: REQUIRED - Write/Read history record table (it can be empty)
  //
  bool BeginWrite3dmHistoryRecordTable();
  bool Write3dmHistoryRecord( 
         const class ON_HistoryRecord&
         );
  bool EndWrite3dmHistoryRecordTable();

  bool BeginRead3dmHistoryRecordTable();

  /*
  Returns:
           0 at end of object table
           1 if object is read
          -1 if file is corrupt
  */
  int  Read3dmHistoryRecord(
          class ON_HistoryRecord*&
          );
  bool EndRead3dmHistoryRecordTable();

  ///////////////////////////////////////////////////////////////////
  // Step 17: OPTIONAL - Write/Read 0 or more user tables
  //

  /*
  Description:
    Write the user table header information that must precede
    the user table information written by a plug-in.
  Parameters:
    plugin_id - [in]
    bSavingGoo - [in]
      Set to false if a plug-in will be used to write
      the user table.  Set to true if a user table written by
      a missing plug-in is being resaved. In this case,
      goo_3dm_version and goo_opennurbs_version must also be
      set.  In practice, you should use Write3dmAnonymousUserTableRecord()
      to handle writing "goo" and use this function only when
      the plug-in in present.
    goo_3dm_version - [in]
      If bSavingGoo is false, this parameter must be zero and
      ON_BinaryArchive::Archive3dmVersion() will be used.
      If bSavingGoo is true, this parameter must be the version of 
      the 3dm archive (1,2,3,4,5,50,...) the plug-in code used to 
      write the user table.
    goo_opennurbs_version - [in]
      If bSavingGoo is false, this parameter must be zero and
      ON_BinaryArchive::ArchiveOpenNURBSVersion() will be used.
      If bSavingGoo is true, this parameter must be the version
      of the opennurbs (YYYYMMDDN) the plug-in code used to 
      write the user table.
  Returns:
    True if the the user information can be written.
    False if user informtion should not be written.
  */
  bool BeginWrite3dmUserTable(
    const ON_UUID& plugin_id,
    bool bSavingGoo,
    int goo_3dm_version,
    int goo_opennurbs_version
    );

  bool EndWrite3dmUserTable();

  /*
  Description:
    If Read3dmAnaonymousUserTable() was used to read ON_3dmGoo because a 
    plug-in was not present, then use Write3dmAnonymousUserTableRecord()
    to put than information back into the archive.
    Write3dmAnonymousUserTableRecord() writes the entire record.
    Do NOT call BeginWrite3dmUserTable() / EndWrite3dmUserTable() when
    using Write3dmAnonymousUserTableRecord().
  Parameters:
    plugin_id - [in]
    goo_version - [in]
      The version of the archive (1,2,3,4,5,50,...) that was used when
      the plug-in wrote the user table.
    goo_opennurbs_version - [in]
      The version of opennurbs ( YYYMMDDN ) that was used when the 
      plug-in wrote the user table.
    goo - [in]
  Returns:
    True if the goo was written or skipped because it could not be robustly
    saved.  False if a catastrophic IO error occured.
  */
  bool Write3dmAnonymousUserTableRecord( 
    const ON_UUID& plugin_id,
    int goo_3dm_version,
    int goo_opennurbs_version,
    const ON_3dmGoo& goo
    );

  // OBSOLETE - use BeginWrite3dmUserTable(plugin_id, bSavingGoo, 3dm_version, opennurbs_version )
  ON_DEPRECATED bool BeginWrite3dmUserTable( const ON_UUID& );

  // OBSOLETE - use Write3dmAnonymousUserTableRecord(plugin_id, ..., goo)
  ON_DEPRECATED bool Write3dmAnonymousUserTable( const ON_3dmGoo& );

  /*
  Parameters:
    plugin_id - [out] 
      id of plug-in that wrote the user table
    bLastSavedAsGoo - [out] 
      True if this table was saved into this archive as goo because
      the plug-in was not present at the time of the save.
    archive_3dm_version - [out]
      Version of the archive the plug-in wrote to.  When bLastSavedAsGoo
      is true, this number can be different from Archive3dmVersion().
    archive_opennurbs_version - [out]
      Version of opennurbs the plug-in used to write the archive.  
      When bLastSavedAsGoo is true, this number can be different 
      from ArchiveOpenNURBSVersion().     
  Returns:
    False when there are no more user tables or an IO error occurs.
  */
  bool BeginRead3dmUserTable(
    ON_UUID& plugin_id,
    bool* bLastSavedAsGoo,
    int* archive_3dm_version,
    int* archive_opennurbs_version
    );

  /*
  Description:
    If the plug-in that wrote the user table is not present and you need
    to read and resave the user table, then use Read3dmAnonymousUserTable()
    to load the information into "goo".
    If you do not need to resave the information, then simply call EndRead3dmUserTable()
    to skip over this table.
  */
  bool Read3dmAnonymousUserTable( 
    int archive_3dm_version,
    int archive_opennurbs_version,
    ON_3dmGoo& goo
    );

  bool EndRead3dmUserTable();

  // OBSOLETE - use BeginRead3dmUserTable( plugin_id, bLastSavedAsGoo, archive_3dm_version, ... )
  ON_DEPRECATED bool BeginRead3dmUserTable(
    ON_UUID&
    );

  // OBSOLETE - use Read3dmAnonymousUserTable( archive_3dm_version, archive_opennurbs_version, goo )
  ON_DEPRECATED bool Read3dmAnonymousUserTable( ON_3dmGoo& );




  ///////////////////////////////////////////////////////////////////
  // Step 18: REQUIRED when writing / OPTIONAL when reading
  //         Write end of file marker.  This information is primarily
  //         used when debugging files to make sure the end of the file
  //         hasn't been cut off.
  //

  // Description:
  //   Writes a TCODE_ENDOFFILE chunk that contains the number
  //   of bytes in the archive.
  //
  // Returns:
  //   true if successful, false if unable to write to archive.
  bool Write3dmEndMark();

  // Description:
  //   Checks for a TCODE_ENDOFFILE chunk at the current position.
  //   If it finds one, it reads it and returns the number
  //   of bytes in the archive.  Comparing this number with
  //   the current file position can help detect files that
  //   have been damaged by loosing sections.
  //
  // Parameters:
  //   sizeof_archive - [out] number of bytes written to archive
  //
  // Returns:
  //   true if successful, false if unable to find or read
  //   a TCODE_ENDOFFILE chunk.
  bool Read3dmEndMark( 
           std::size_t* // sizeof_archive
           );

  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  // Low level tools to  Write/Read chunks. See opennurbs_3dm.h for details
  // about the structure of chunks.  Every chunk must begin with a
  // call to BeginWrite/ReadChunk().
  // If BeginWriteChunk()/BeginReadChunk() returns true, then
  // you must call EndWrite/ReadChunk() or cease using the archive.

  // Description:
  //   Writes a chunk header containing 4 byte typecode and value.
  //
  // Parameters:
  //   typecode - [in] a TCODE_* number from opennurbs_3dm.h
  //   value    - [in] if (typecode&TCODE_SHORT) is nonzero, then
  //              this is the value to be saved.  Othewise, pass
  //              a zero and the EndWrite3dmChunk() call will
  //              store the length of the chunk.
  //
  // Returns:
  //   true if write was successful.
  bool BeginWrite3dmChunk(
        unsigned int, // typecode
        int // value
        );

  bool BeginWrite3dmBigChunk(
        ON__UINT32 typecode,
        ON__INT64 value
        );

  /*
  Description:
    Begins writing a chunk.
  Parameters:
    tcode - [in] chunk's typecode from opennurbs_3dm.h.  This cannot be a short tcode.
    major_version - [in] ( >= 1)
    minor_version - [in] ( >= 0 )
  Returns:
    True if input was valid and chunk was started.  In this case
    You must call EndWrite3dmChunk(), even if something goes wrong
    while you attempt to write the contents of the chunk.
    False if input was not valid or the write failed.
  */
  bool BeginWrite3dmChunk(
        unsigned int tcode,
        int major_version,
        int minor_version
        );


  // updates length in chunk header
  bool EndWrite3dmChunk();

  bool Write3dmGoo( const ON_3dmGoo& ); // call to write "goo"

  // OBSOLETE - Use BeginRead3dmBigChunk()
  ON_DEPRECATED bool BeginRead3dmChunk(
        unsigned int*,   // typecode from opennurbs_3dm.h
        int*             // value
        );

  // When the end of the 3dm file is reached, BeginReadChunk() will
  // return true with a typecode of TCODE_ENDOFFILE.
  bool BeginRead3dmBigChunk(
        unsigned int*,   // typecode from opennurbs_3dm.h
        ON__INT64*       // value
        );
  /*
  Description:
    Begins reading a chunk that must be in the archive at this location.
  Parameters:
    expected_tcode - [in] chunk's typecode from opennurbs_3dm.h
    major_version - [out] 
    minor_version - [out] 
  Returns:
    True if beginning of the chunk was read.  In this case
    You must call EndRead3dmChunk(), even if something goes wrong
    while you attempt to read the interior of the chunk.
    False if the chunk did not exist at the current location in the file.
  */
  bool BeginRead3dmChunk(
        unsigned int expected_tcode,
        int* major_version,
        int* minor_version
        );

  /*
  Description:
    Calling this will skip rest of stuff in chunk if it was only partially read.
  Parameters:
    bSupressPartiallyReadChunkWarning - [in]
      Generally, a call to ON_WARNING is made when a chunk is partially
      read.  If bSupressPartiallyReadChunkWarning is true, then
      no warning is issued for partially read chunks.
  */
  bool EndRead3dmChunk(); 
  bool EndRead3dmChunk(bool bSupressPartiallyReadChunkWarning); 


  ///////////////////////////////////////////////////////////////////
  //
  // Tools for dictionary IO (used in .NET)
  //

  /*
  Description:
    Begins writing a dictionary.
  Parameters:
    dictionary_id - [in]
    version - [in]
      It is suggested that you use YYYYMMDD as the version number.
    dictionary_name - [in]
      You may pass NULL.
  Remarks:
    Begins a new chunk with tcode TCODE_DICTIONARY and then writes
    a TCODE_DICTIONARY_ID chunk containing the id, version and name.
    After calling this function, you may either write entries by
    calling
      BeginWriteDictionaryEntry(); 
      write entry definition...
      EndWriteDictionaryEntry();
    or you may finish writing the dictionay by calling
      EndWriteDictionary();
  */
  bool BeginWriteDictionary(
          ON_UUID dictionary_id,
          unsigned int version,
          const wchar_t* dictionary_name
          );
  /*
  Description:
    Begins writing a dictionary entry.
  Parameters:
    de_type - [in]
    entry_name - [in]
  Returns:
    true 
      Entry header was written and you must call EndWriteDictionary()
      after writing the entry data.
    false 
      Failed to write entry header.  Do not call EndWriteDictionary().
  Remarks:
    Begins a new chunk with tcode TCODE_DICTIONARY_ENTRY,
    then writes the int, and then writes the string.
  */
  bool EndWriteDictionary();

  /*
  Description:
    Begins writing a dictionary entry.
  Parameters:
    de_type - [in]
    entry_name - [in]
  Returns:
    true 
      Entry header was written and you must call EndWriteDictionary()
      after writing the entry data.
    false 
      Failed to write entry header.  Do not call EndWriteDictionary().
  Remarks:
    Begins a new chunk with tcode TCODE_DICTIONARY_ENTRY,
    then writes the int, and then writes the string.
  */
  bool BeginWriteDictionaryEntry(
          int de_type, 
          const wchar_t* entry_name
          );
  bool EndWriteDictionaryEntry();

  bool BeginReadDictionary(
          ON_UUID* dictionary_id,
          unsigned int* version,
          ON_wString& dictionary_name
          );
  bool EndReadDictionary();

  /*
  Description:
    Begin reading a dictionary entry.
  Parameters:
    de_type - [out]
    entry_name - [out]
  Returns:
    0: serious IO error
    1: success
        read information and then call EndReadDictionaryEntry()
    2: at end of dictionary
  */
  int BeginReadDictionaryEntry(
          int* de_type, 
          ON_wString& entry_name
          );
  bool EndReadDictionaryEntry();

  bool Read3dmGoo( ON_3dmGoo& ); // Call to read "goo"

  // OBSOLETE - Use PeekAt3dmBigChunkType()
  ON_DEPRECATED bool PeekAt3dmChunkType( // does not change file position
        unsigned int*,   // typecode from opennurbs_3dm.h
        int*             // value
        );

  bool PeekAt3dmBigChunkType( // does not change file position
        ON__UINT32* typecode,
        ON__INT64* big_value
        );

  bool Seek3dmChunkFromStart( 
        // beginning at the start of the active chunk, search portion of
        // archive included in active chunk for the start of a subchunk 
        // with the specified type.
        // if true is returned, then the position is set so the next call to
        // BeginRead3dmChunk() will read a chunk with the specified typecode
        unsigned int    // typecode from opennurbs_3dm.h
        );
  bool Seek3dmChunkFromCurrentPosition( 
        // beginning at the current position, search portion of archive
        // included in active chunk for the start of a subchunk with the
        // specified type.
        // if true is returned, then the position is set so the next call to
        // BeginRead3dmChunk() will read a chunk with the specified typecode
        unsigned int    // typecode from opennurbs_3dm.h
        );

  // A chunk version is a single byte that encodes a major.minor 
  // version number.  Useful when creating I/O code for 3dm chunks
  // that may change in the future.  Increment the minor version 
  // number if new information is added to the end of the chunk. 
  // Increment the major version if the format of the chunk changes
  // in some other way.
  bool Write3dmChunkVersion(
    int, // major // 0 to 15
    int  // minor // 0 to 16
    );
  bool Read3dmChunkVersion(
    int*, // major // 0 to 15
    int*  // minor // 0 to 16
    );

  /*
  Description:
    Low level tool to writes user data attached to the 
    object.  This function should never be called
    directly.
  Parameters:
    object - [in]
  Returns:
    True if successful.
  */
  bool WriteObjectUserData( const ON_Object& object );

  /*
  Description:
    Low level tool to read user data and attach it to
    the object.  This function should never be called
    directly.
  Parameters:
    object - [in/out]
  Returns:
    True if successful.
  */
  bool ReadObjectUserData( ON_Object& object );

  /*
  Description:
    If a 3dm archive is being read or written, then this is the
    version of the 3dm archive format (1, 2, 3, 4 or 5).
  Returns:
    @untitle table
    0     a 3dm archive is not being read/written
    1     a version 1 3dm archive is being read/written
    2     a version 2 3dm archive is being read/written
    3     a version 3 3dm archive is being read/written
    4     a version 4 3dm archive is being read/written
    5     an old version 5 3dm archive is being read
    50    a version 5 3dm archive is being read/written
  See Also:
    ON_BinaryArchive::ArchiveOpenNURBSVersion
  */
  int Archive3dmVersion() const;

  /*
  Description:
    If a 3dm archive is being read, then this is the version
    of openNURBS that was used to write the archive.  This value
    is only available after ON_BinaryArchive::Read3dmProperties
    is called.
  See Also:
    ON_BinaryArchive::Archive3dmVersion
    ON_BinaryArchive::Read3dmProperties
  Returns:
    Version of openNURBS used to write the archive.  The openNURBS
    version is the value returned by ON::Version.
  See Also:
    ON::Version
    ON_BinaryArchive::Read3dmProperties
    ON_BinaryArchive::Archive3dmVersion
  Remarks:
    This value is rarely needed.  You probably want to
    use ON_BinaryArchive::Archive3dmVersion.
  */
  int ArchiveOpenNURBSVersion() const;

  /*
  Description:
    When a 3dm archive is saved from an MFC application that
    supports Windows linking/embedding, the first 5kb to 1mb
    of the file contains information that is put there by MFC.
    ArchiveStartOffset() returns the offset into the file where
    the 3dm archive actually begins. The call to 
    ON_BinaryArchive::Read3dmStartSection() calculates this
    offset and stores the value in m_3dm_start_section_offset.
  Returns:
    Offset into the binary "file" where the actual 3dm archive
    begins.
  Remarks:
    Generally, this value can be ignored. This function is
    a diagnostice tool that is used to analyzed damaged files.
  */
  std::size_t ArchiveStartOffset() const;

  enum table_type
  {
    no_active_table = 0,
    properties_table,
    settings_table,
    bitmap_table,
    texture_mapping_table,
    material_table,
    linetype_table,
    layer_table,
    light_table,
    object_table,
    group_table,
    font_table,
    dimstyle_table,
    hatchpattern_table,
    instance_definition_table,
    historyrecord_table,
    user_table
  };

  /*
  Description:
    Expert user function for reading damaged files.
  Parameters:
    chunk - [out] current chunk.
  Returns:
    Level of the chunk or 0 if there is no current
    chunk.
  */
  int GetCurrentChunk(ON_3DM_CHUNK& chunk) const;
  int GetCurrentChunk(ON_3DM_BIG_CHUNK& big_chunk) const;

  /*
  Description:
    Expert user function for reading damaged files.  The search starts
    at the beginning of the file.
  Parameters:
    tcode_table - [in] typecode of the table
    tcode_record - [in] typecode of the record
    class_uuid - [in] id of the opennurbs class in the record
    min_length_data - [in] minimum size of the opennurbs class data
  Returns:
    True if the table start is found.  In this case the current
    position of the archive is at the start of the table and
    the standared BeginRead3dm...Table() function can be used.
    False if the table start is not found.
  */
  bool FindTableInDamagedArchive(
          unsigned int tcode_table,
          unsigned int tcode_record,
          ON_UUID class_uuid,
          int min_length_data
          );

  /*
  Description:
    Expert user function for studying contents of a file.
    The primary use is as an aid to help dig through files
    that have been damaged (bad disks, transmission errors, etc.)
    If an error is found, a line that begins with the word
    "ERROR" is printed.
  Parameters:
    text_log - [in] place to print informtion
    recursion_depth - [in] simply a counter
        to aid in debugging.
  Returns:
    0 if something went wrong, otherwise the typecode
    of the chunk that was just studied.
  */
  unsigned int 
  Dump3dmChunk(
        ON_TextLog& text_log, 
        int recursion_depth = 0
        );

protected:

  /*
  Description:
    Works like the C runtrim fread().
  Returns:
    actual number of bytes read (like fread())
  */
  virtual
  std::size_t Read( std::size_t, void* ) = 0; 

  /*
  Description:
    Works like the C runtrim fwrite().
  Returns:
    actual number of bytes written (like fwrite())
  */
  virtual
  std::size_t Write( std::size_t, const void* ) = 0;

  /*
  Description:
    Force Write() to flush any buffered data to physical archive.
  Returns:
    True if successful or if there is nothing to flush.  False if
    information could not be flushed.
  */
  virtual
  bool Flush() = 0;

  /*
  Applications (like Rhino) override this function to load plug-ins
  Description:
    When ON_BinaryArchive::ReadObject() encounters userdata and
    the user data class id is not present,  LoadUserDataApplication
    is called to load the application that created user data.
  Returns:
    0 - could not load the application
    1 - successfully loaded the application
    2 - the application was already loaded
  */
  virtual
  int LoadUserDataApplication(
    ON_UUID /*application_id*/
    )
  {
    return 0;
  }

  bool SetArchive3dmVersion(int);

private:
  // 16 bit integer IO
  bool WriteInt8( std::size_t, const ON__INT8* );
  bool ReadInt8( std::size_t, ON__INT8* );

  // 16 bit integer IO
  bool WriteInt16( std::size_t, const ON__INT16* );
  bool ReadInt16( std::size_t, ON__INT16* );

  // 32 bit integer IO
  bool WriteInt32( std::size_t, const ON__INT32* );
  bool ReadInt32( std::size_t, ON__INT32* );

  // 64 bit integer IO
  bool WriteInt64( std::size_t, const ON__INT64* );
  bool ReadInt64(  std::size_t, ON__INT64* );

  bool BeginWrite3dmTable( 
    unsigned int // tcode
    );
  bool EndWrite3dmTable( 
    unsigned int // tcode
    );
  bool BeginRead3dmTable( 
    unsigned int // tcode
    );
  bool EndRead3dmTable( 
    unsigned int // tcode
    );
  
  bool Read3dmV1Layer( ON_Layer*& );
  int  Read3dmV1Light(  // returns 0 at end of light table
                      //         1 light successfully read
                      //        -1 if file is corrupt
            ON_Light**, // light returned here
            ON_3dmObjectAttributes* // optional - if NOT NULL, object attributes are
                                    //            returned here
            );
  int Read3dmV1Material( ON_Material** );
  int  Read3dmV1Object( // returns 0 at end of object table
                      //         1 if object is read
                      //         2 if object is skipped because it does not match filter
                      //        -1 if file is corrupt
          ON_Object**, // object returned here (NULL if skipped)
          ON_3dmObjectAttributes*, // optional - if NOT NULL, object attributes are
                                   //            returned here
          unsigned int = 0 // optional filter made by setting ON::object_type bits
          );  // returns NULL at end of object table

  bool Read3dmV1AttributesOrMaterial( 
            ON_3dmObjectAttributes*,    // attributes,
            ON_Material*,      // material,
            ON_BOOL32&,             // bHaveMat
            unsigned int,      // end_mark_tcode 
            class ON__3dmV1_XDATA* = 0 // v1 "xdata"
            );
  bool Read3dmV1String( ON_String& );
  int  Read3dmV1LayerIndex( const char* ) const;

public:
  // helpers for reading V1 objects
  bool ReadV1_TCODE_RH_POINT(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_MESH_OBJECT(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_LEGACY_CRV(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_LEGACY_FAC(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_LEGACY_SHL(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_RHINOIO_OBJECT_NURBS_CURVE(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_RHINOIO_OBJECT_NURBS_SURFACE(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_RHINOIO_OBJECT_BREP(ON_Object**,ON_3dmObjectAttributes*);
  bool ReadV1_TCODE_ANNOTATION(unsigned int,ON_Object**,ON_3dmObjectAttributes*);

private:
  ON::archive_mode Mode() const; // current read/write mode
  void UpdateCRC( std::size_t, const void* );
  int ReadObjectHelper(ON_Object**);

  int m_3dm_version;
  int m_3dm_v1_layer_index;
  int m_3dm_v1_material_index;

  // The bits in m_error_message_mask are used to mask errors
  // when we know we are doing something that may generate an
  // error.
  //
  // bit 0x00000001
  //   V1 files do not have a table structure and are read using
  //   multiple passes and there are valid situations where a 
  //   4 byte read is attempted at the end of a file.
  //
  // bit 0x00000002
  //   Some v1 files do not have an end mark.  When reading
  //   these v1 files bit 0x02 is set.
  //
  // bit 0x00000004
  //   Requested read may go beyond end of file.
  //   One situation where this happens is when a table is not at the 
  //   expected location in a file, 

  unsigned int m_error_message_mask;
protected:
  unsigned int ErrorMessageMask() const;
  /*
  Paramters:
    sizeof_request - [in] 
      value of count parameter passed to virtual Read() function.
    sizeof_read - [in]
      number of bytes actually read by the virtual Read() function.
  Returns:
    True if a call to Read() is permitted to ask for more bytes
    than are left in the file.  This value varies as the file
    is read and must be checked at each failure.
  */
  bool MaskReadError( ON__UINT64 sizeof_request, ON__UINT64 sizeof_read ) const;
private:


  // When a 3DM archive is read, m_3dm_opennurbs_version records the version of
  // OpenNURBS used to create the archive.  Otherwise, m_3dm_opennurbs_version
  // is zero.
  //
  // Read3dmProperties() sets this to the version of OpenNURBS that was
  // used to write file file.  If the file was created using a version
  // of OpenNURBS before 200012210, this number will be zero.
  //
  // Write3dmProperties() stores the value returned by ON::Version() in
  // the archive's properties table.
  friend void ON_SetBinaryArchiveOpenNURBSVersion(ON_BinaryArchive&,int);
  int m_3dm_opennurbs_version;

  // When a 3dm archive is saved from an MFC application that supports
  // Windows linking/embedding, the first 5kb to 1mb of the file contains
  // information that is put there by MFC.  m_3dm_start_section_offset
  // records the offset into the file where the 3dm archive actually begins.
  std::size_t m_3dm_start_section_offset;

  table_type m_active_table;

  table_type TableTypeFromTypecode( unsigned int ); // table type from tcode

  ON_SimpleArray<ON_3DM_BIG_CHUNK> m_chunk;

  // stack of chunks
  bool PushBigChunk( ON__UINT32 typecode, ON__INT64 value );

  bool WriteChunkTypecode( ON__UINT32 );
  bool ReadChunkTypecode( ON__UINT32* );
  bool WriteChunkValue( ON__UINT32 typecode, ON__INT64 );
  bool WriteChunkLength( ON__UINT64 );
  bool ReadChunkValue( ON__UINT32 typecode, ON__INT64* value64 );
  bool FindMisplacedTable( 
        ON__UINT64 filelength,
        const ON__UINT32 table_tocde,
        const ON__UINT32 table_record_record,
        const ON_UUID class_uuid,
        const ON__UINT64 min_length_data
        );

  bool ReadObjectUserDataAnonymousChunk(
          const ON__UINT64 length_TCODE_ANONYMOUS_CHUNK,
          const int archive_3dm_version,
          const int archive_opennurbs_version,
          class ON_UserData* ud );

public:
  std::size_t SizeofChunkLength() const;

private:
  bool WriteEOFSizeOfFile( ON__UINT64 );
  bool ReadEOFSizeOfFile( ON__UINT64* );

  bool m_bDoChunkCRC; // true if active chunk crc status should be checked
                      // and updated.
  int m_bad_CRC_count; // number of chunks that have a bad crc


private:
  // compressed buffer I/O uses zlib 1.1.3 inflate()/deflate()
  struct
  {
    ON::archive_mode mode; // ON::read = read and inflate,  ON::write = deflate and write
    enum
    {
      sizeof_x_buffer = 16384
    };
    unsigned char    buffer[sizeof_x_buffer];
    z_stream         strm;
  } m_zlib;

  // returns number of bytes written
  std::size_t WriteDeflate(
        std::size_t,         // sizeof uncompressed input data
        const void*  // uncompressed input data
        );
  bool ReadInflate(
        std::size_t,  // sizeof uncompressed input data
        void* // buffer to hold uncompressed data
        );
  bool CompressionInit();
  void CompressionEnd();

private:
  // endian-ness of the cpu reading this file.
  // 3dm files are alwasy saved with little endian byte order.
  ON::endian m_endian;

  ON::archive_mode m_mode;

  // 3dm write options
  bool m_bSaveUserData; // true to save user data (increases file size)
  bool m_bSavePreviewImage;    // true to save 200x200 preview bitmap (increases file size)
  bool m_bEmbedTextureBitmaps; // true to embed texture, bump, trace, and wallpaper bitmaps (increases file size)
  bool m_bSaveRenderMeshes;    // true to save meshes used to render B-rep objects (increases file size)
  bool m_bSaveAnalysisMeshes;  // true to save meshes used in surface analysis (increases file size)

  // ids of plug-ins that support saving older (V3) versions
  // of user data.  This information is filled in from the
  // list of plug-ins passed in whenteh settings are saved.
  ON_SimpleArray< ON_UUID > m_V3_plugin_id_list;

  struct ON__3dmV1LayerIndex* m_V1_layer_list;

  // prohibit default construction, copy construction, and operator=
  ON_BinaryArchive();
  ON_BinaryArchive( const ON_BinaryArchive& ); // no implementation
  ON_BinaryArchive& operator=( const ON_BinaryArchive& ); // no implementation

};

class ON_CLASS ON_3dmGoo
{
  // used to store goo
public:
  ON_3dmGoo();
  ~ON_3dmGoo();
  ON_3dmGoo( const ON_3dmGoo& );
  ON_3dmGoo& operator=( const ON_3dmGoo& );

  void Dump(ON_TextLog&) const;

  unsigned int m_typecode;
  int m_value;
  unsigned char* m_goo;
  ON_3dmGoo* m_next_goo;
  ON_3dmGoo* m_prev_goo;
};


class ON_CLASS ON_BinaryFile : public ON_BinaryArchive
{
public:
  ON_BinaryFile( ON::archive_mode );

  /*
  Description:
    Create an ON_BinaryArchive that reads/writes from an ordinary file.
  Parameters:
    mode - [in]
    fp - [in]
      If a file is being read, fp is the pointer returned 
      from ON_FileStream::Open(...,"rb").
      If a file is being written, fp is the pointer returned 
      from ON_FileStream::Open(...,"wb").
  */
  ON_BinaryFile( ON::archive_mode, FILE* fp );

  virtual ~ON_BinaryFile();

  // ON_BinaryArchive overrides
  std::size_t CurrentPosition() const; 
  bool SeekFromCurrentPosition(int);
  bool SeekFromStart(std::size_t);
  bool AtEnd() const;

  // fseek from end (since the file has an end)
  bool SeekFromEnd( int ); 

  //////////
  // To use custom memory buffering instead of relying
  // on fread()/fwrite()'s build in buffering, call
  // EnableMemoryBuffer() with the buffer size immediately
  // after constructing the ON_BinaryFile.  There appear
  // to be enough bugs in existing Windows NT/2000 NETWORK
  // I/O that using this hack will speed up I/O by factors
  // of 10 to 100.
  void EnableMemoryBuffer(
         int=16384 // capacity of memory buffer
         );

protected:
  std::size_t Read( std::size_t, void* );
  std::size_t Write( std::size_t, const void* );
  bool Flush();

private:
  // Implementation
  FILE* m_fp;

  // if m_memory_buffer_capacity is zero, then Write() uses
  // fwrite() directly.  If m_memory_buffer_capacity is
  // greater than zero, then Write() buffers its results
  // into m_memory_buffer.  This is provided to work around
  // bugs in some networks that result in extremely slow
  // performance when seeking is used.
  std::size_t m_memory_buffer_capacity;
  std::size_t m_memory_buffer_size;
  std::size_t m_memory_buffer_ptr;
  unsigned char* m_memory_buffer;

private:
  // prohibit default construction, copy construction, and operator=
  ON_BinaryFile( ); // no implementation
  ON_BinaryFile( const ON_BinaryFile& ); // no implementation
  ON_BinaryFile& operator=( const ON_BinaryFile& ); // no implementation
};

class ON_CLASS ON_BinaryArchiveBuffer : public ON_BinaryArchive
{
public:
  /*
  Description:
    Create an ON_BinaryArchive that reads/writes from an ON_Buffer.
  Parameters:
    mode - [in]
    buffer - [in]
  Remarks:
    If a non-null buffer is specifed, then do not call SetBuffer()
  */
  ON_BinaryArchiveBuffer( ON::archive_mode, ON_Buffer* buffer );

  virtual ~ON_BinaryArchiveBuffer();

  /*
  Description:
    If the ON_BinaryArchiveBuffer class is created with the constructor
    that has a single "mode" parameter, then use SetBuffer()
    to specify the buffer to read/write from before using
    the ON_BinaryArchiveBuffer.
  Parameters:
    buffer - [in]
  Returns:
    True if the buffer is set.  Once the buffer is set it
    cannot be changed.
  */
  bool SetBuffer( ON_Buffer* buffer );

  /*
  Returns:
    Buffer being read/written. 
  */
  ON_Buffer* Buffer() const;

  // virtual ON_BinaryArchive overrides
  std::size_t CurrentPosition() const; 
  bool SeekFromCurrentPosition(int);
  bool SeekFromStart(std::size_t);
  bool AtEnd() const;

  bool SeekFromEnd( ON__INT64 ); 

protected:
  std::size_t Read( std::size_t, void* );
  std::size_t Write( std::size_t, const void* );
  bool Flush();

private:
  // Buffer being read/written.
  ON_Buffer* m_buffer;

private:
  // prohibit use - you should specify a buffer.
  ON_BinaryArchiveBuffer( ON::archive_mode );
private:
  // prohibit default construction, copy construction, and operator=
  ON_BinaryArchiveBuffer( ); // no implementation
  ON_BinaryArchiveBuffer( const ON_BinaryArchiveBuffer& ); // no implementation
  ON_BinaryArchiveBuffer& operator=( const ON_BinaryArchiveBuffer& ); // no implementation
};


class ON_CLASS ON_Read3dmBufferArchive : public ON_BinaryArchive
{
public:

  /*
  Description:
    Construct an ON_BinaryArchive for reading information from a memory buffer.
  Parameters:
    sizeof_buffer - [in] size of buffer in bytes (>0)
    buffer - [in] memory buffer containing binary archive
    bCopyBuffer - [in]
      true - copy the input buffer.  
          Useful when the buffer may be destroyed while this class is still in use.
      false - Do not copy the input buffer.  
          In this case you are responsible for making certain the input buffer 
          is valid while this class is in use.
    archive_3dm_version  - [in] (1,2,3,4 or 5)
    archive_opennurbs_version - [in] YYYYMMDDn
  */
  ON_Read3dmBufferArchive( 
    std::size_t sizeof_buffer, 
    const void* buffer,
    bool bCopyBuffer,
    int archive_3dm_version,
    int archive_opennurbs_version
    );

  ~ON_Read3dmBufferArchive();

  /*
  Returns: 
     value of m_sizeof_buffer
  */
  std::size_t SizeOfBuffer() const;

  /*
  Returns: 
     value of m_buffer
  */
  const void* Buffer() const;

  // ON_BinaryArchive overrides
  std::size_t CurrentPosition() const; 
  bool SeekFromCurrentPosition(int); 
  bool SeekFromStart(std::size_t);
  bool AtEnd() const;

protected:
  // ON_BinaryArchive overrides
  std::size_t Read( std::size_t, void* ); // return actual number of bytes read (like fread())
  std::size_t Write( std::size_t, const void* );
  bool Flush();

private:
  void* m_p;
  const unsigned char* m_buffer;
  std::size_t m_sizeof_buffer;
  std::size_t m_buffer_position;
  ON__INT_PTR m_reserved1;
  ON__INT_PTR m_reserved2;
  ON__INT_PTR m_reserved3;
  ON__INT_PTR m_reserved4;

private:
  // prohibit use - no implementation
  ON_Read3dmBufferArchive(); 
  ON_Read3dmBufferArchive( const ON_Read3dmBufferArchive& );
  ON_Read3dmBufferArchive& operator=(const ON_Read3dmBufferArchive&);
};

class ON_CLASS ON_Write3dmBufferArchive : public ON_BinaryArchive
{
public:

  /*
  Description:
    Construct an ON_BinaryArchive for writing information to a memory buffer.
  Parameters:
    initial_sizeof_buffer - [in] 
      initial size of buffer in bytes (>=0)
      If you are unable to estimate the size you will need, pass in zero.
    max_sizeof_buffer - [in] 
      maximum size of buffer in bytes (>=0)
      If max_sizeof_buffer > 0 and the amount of information saved 
      requires a buffer larger than this size, then writing fails. 
      If max_sizeof_buffer <= 0, then no buffer size limits are enforced.
    archive_3dm_version  - [in] (0, ,2,3,4 or 50)
      Pass 0 or ON_BinaryArchive::CurrentArchiveVersion() to write the
      version of opennurbs archives used by lastest version of Rhino.
    archive_opennurbs_version - [in] YYYYMMDDn
  */
  ON_Write3dmBufferArchive( 
    std::size_t initial_sizeof_buffer, 
    std::size_t max_sizeof_buffer, 
    int archive_3dm_version,
    int archive_opennurbs_version
    );

  ~ON_Write3dmBufferArchive();

  /*
  Returns: 
     Size of the archive in bytes.
  */
  std::size_t SizeOfArchive() const;

  /*
  Returns: 
     value of m_sizeof_buffer
  */
  std::size_t SizeOfBuffer() const;

  /*
  Returns: 
     value of m_buffer.
     SizeOfArchive() reports the number of bytes
     written to this buffer.
     SizeOfBuffer() reports the number of bytes
     allocated in this buffer.
     
  */
  const void* Buffer() const;

  /*
  Returns:
    The pointer to the buffer and sets all 
    members on this archive back to zero.
    The caller is responsible for calling onfree() on
    the pointer when finished with the buffer.
  */
  void* HarvestBuffer();

  // ON_BinaryArchive overrides
  std::size_t CurrentPosition() const; 
  bool SeekFromCurrentPosition(int); 
  bool SeekFromStart(std::size_t);
  bool AtEnd() const;

protected:
  // ON_BinaryArchive overrides
  std::size_t Read( std::size_t, void* ); 
  std::size_t Write( std::size_t, const void* ); // return actual number of bytes written (like fwrite())
  bool Flush();

private:
  void AllocBuffer(std::size_t);
  void* m_p;
  unsigned char* m_buffer;
  std::size_t m_sizeof_buffer;
  const std::size_t m_max_sizeof_buffer;
  std::size_t m_sizeof_archive;
  std::size_t m_buffer_position;
  ON__INT_PTR m_reserved1;
  ON__INT_PTR m_reserved2;
  ON__INT_PTR m_reserved3;
  ON__INT_PTR m_reserved4;

private:
  // prohibit use - no implementation
  ON_Write3dmBufferArchive(); 
  ON_Write3dmBufferArchive( const ON_Write3dmBufferArchive& );
  ON_Write3dmBufferArchive& operator=(const ON_Write3dmBufferArchive&);
};

/*
Description:
  Create a simple archive that contains a single geometric object.
Parameters:
  archive - [in] destination archive.
  version - [in] (0, 2, 3, 4, or 50) format version.archive version number.
      Version 2 format can be read by Rhino 2 and Rhino 3.  Version
      3 format can be read by Rhino 3.
      Pass 0 or ON_BinaryArchive::CurrentArchiveVersion() to write
      the latest version of archives supported by Rhino.
  object - [in] object to be saved in the archive's object table.
      This is typically some type of ON_Curve, ON_Surface, ON_Mesh,
      or ON_Brep.
Returns:
  @untitled table
  true     archive successfully written.
  false    archive not successfully written.
Example:

          const char* filename = "myfile.3dm";
          FILE* fp = ON::OpenFile( filename, "wb" );
          ON_BinaryFile file( fp, ON::write3dm );
          ON_BOOL32 ok = ON_WriteArchive( archive, geometry );
          ON::CloseFile( fp );

Remarks:
  The object table in the archive will contain a single
  object.
*/
ON_DECL
bool ON_WriteOneObjectArchive( 
          ON_BinaryArchive& archive,
          int version,
          const ON_Object& object
          );

#endif


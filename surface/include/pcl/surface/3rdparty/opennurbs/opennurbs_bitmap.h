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

////////////////////////////////////////////////////////////////
//
//  Defines ON_WindowsBITMAPINFO class that is used to provide OS independent
//  serialization of Windows device independent bitmaps (BITMAPINFO) used
//  to store preview images.
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_BITMAP_INC_)
#define OPENNURBS_BITMAP_INC_

class ON_CLASS ON_Bitmap : public ON_Object
{
  // virtual base class for bitmap objects
  ON_OBJECT_DECLARE(ON_Bitmap);
public:
  ON_Bitmap();
  ~ON_Bitmap();

  // C++ defaults work fine
  //ON_Bitmap(const ON_Bitmap&);
  //ON_Bitmap& operator=(const ON_Bitmap&);

  // virtual
  ON_UUID ModelObjectId() const;


  void Dump( ON_TextLog& ) const; // for debugging

  void EmergencyDestroy();
  void Destroy();

  void Defaults();

  virtual
  int Width() const = 0;
  virtual
  int Height() const = 0; // >0 means it's a bottom-up bitmap with origin at lower right
                          // <0 means it's a top-down bitmap with origin at upper left
  virtual
  int BitsPerPixel() const = 0; // bits per pixel
  virtual
  int SizeofScan() const = 0;  // number of bytes per scan line
  virtual
  int SizeofImage() const = 0; // size of current map in bytes

  virtual
  unsigned char* Bits(
    int // index of scan line 
    ) = 0;
  virtual
  const unsigned char* Bits(
    int // index of scan line 
    ) const = 0;

  ON_UUID    m_bitmap_id;
  int        m_bitmap_index;
  ON_wString m_bitmap_name;     // descriptive name
  ON_wString m_bitmap_filename; // full path to file
};

/*
  ON_EmbeddedFile is derived from ON_Bitmap so it can be stored 
  in the Rhino document's CRhinoDoc::m_bitmap_table[] array.
  The ON_EmbeddedFile class is used to embed any type of file
  in a Rhino document.
*/
class ON_CLASS ON_EmbeddedFile : public ON_Object
{
  ON_OBJECT_DECLARE(ON_EmbeddedFile);
public:
  ON_EmbeddedFile();
  ON_EmbeddedFile(const ON_EmbeddedFile& src);
  ON_EmbeddedFile& operator=(const ON_EmbeddedFile& src);

  virtual ~ON_EmbeddedFile();
  void EmergencyDestroy();
  void Destroy();
  void DestroyBuffer();

  /*
  Description:
    Store the specified file in an ON_EmbeddedFile class.
  Parameters:
    filename - [in]
      full path to the file.
    bCompress - [in]
      true if the image of the file should be compressed.
      (Pass false if the file is already in a compressed
      format, like jpg, png, zip files.)
  Returns:
    true if successful.  When true is returned m_id is set to
    a new unique id, m_full_file_name is set to filename,
    and m_relative_file_name is empty.
  */
  bool Create( 
    const wchar_t* filename, 
    bool bCompress
    );

  /*
  Description:
    Store the specified file in an ON_EmbeddedFile class.
  Parameters:
    fp - [in]
      Result of calling ON::OpenFile( ..., "rb" )
    bCompress - [in]
      true if the image of the file should be compressed.
      (Pass false if the file is already in a compressed
      format, like jpg, png, zip files.)
  */
  bool Create( 
    FILE* fp,
    bool bCompress 
    );
  
  /*
  Description:
    Store the specified buffer in an ON_EmbeddedFile class.
  Parameters:
    source_buffer - [in]
    source_buffer_size - [in]
      number of bytes in source_buffer.
    bCompress - [in]
      true if the source_buffer should be compressed.
      (Pass false if source_buffer is already in a compressed format.)
  */
  bool Create( 
    const void* source_buffer,
    ON__UINT64 sizeof_source_buffer,
    bool bCompress 
    );

  bool Extract( 
    const wchar_t* destination_filename
    ) const;

  bool Extract( 
    FILE* fp
    ) const;

  /*
  Description:
    Extracts the file into a buffer.
  Parameters:
    buffer - [out]
      buffer must point to FileSize() bytes of memory.
      The extracted file will be copied to this buffer.
  Returns:
    True if successful.
    False if not successful.
  */
  bool Extract( 
    void* buffer
    ) const;
        
  /*
  Returns
    full path file name
  */
  const wchar_t* FullFileName() const;
    
  /*
  Returns
    Relative file name.  Usually relative to the directory
    where the archive containing this embedded file was last
    saved.
  */
  const wchar_t* RelativeFileName() const;

  ON_UUID Id() const;

  void SetId( ON_UUID id );

  void SetFullFileName( const wchar_t* full_file_name );


  void SetRelativeFileName( const wchar_t* relative_file_name );

  ON__UINT64 FileSize() const;
  ON__UINT64 FileLastModifiedTime() const;
  ON__UINT32 FileCRC() const;
  
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  ON_BOOL32 Write( ON_BinaryArchive& ) const;
  ON_BOOL32 Read( ON_BinaryArchive& );

  // The relative path is typically set when the .3dm file is
  // saved and is the path to the file relative to the location
  // of the saved file.
  // (The full path to the file is in ON_Bitmap::m_bitmap_filename.)
  ON_UUID    m_id;
  ON_wString m_full_file_name; // full path file name
  ON_wString m_relative_file_name; // relative path when the archive was last saved.

private:
  void* m_reserved;

public:
  ON__UINT64 m_file_size;
  ON__UINT64 m_file_time;  // last modified time returned by ON::GetFileStats()
  ON__UINT32 m_file_crc;   // 32 bit crc of the file from ON_CRC32

public:
  ON__UINT32 m_buffer_crc; // will be different from m_file_crc if the buffer is compressed.
  ON_Buffer m_buffer;
  unsigned char m_bCompressedBuffer; // true if m_buffer is compressed.

private:
  unsigned char m_reserved3[7];
};


#if !defined(ON_OS_WINDOWS_GDI)

// These are the values of the Windows defines mentioned
// in the comment below.  If you're running on Windows,
// they get defined by Windows system header files.
// If you aren't running on Windows, then you don't
// need them.
//#define BI_RGB        0L
//#define BI_RLE8       1L
//#define BI_RLE4       2L
//#define BI_BITFIELDS  3L

// Mimics Windows BITMAPINFOHEADER structure.
// For details searh for "BITMAPINFOHEADER" at http://msdn.microsoft.com/default.asp 
struct ON_WindowsBITMAPINFOHEADER
{
  unsigned int   biSize;          // DWORD = sizeof(BITMAPINFOHEADER)
  int            biWidth;         // LONG  = width (in pixels) of (decompressed) bitmap
  int            biHeight;        // LONG  = height (in pixels) of (decompressed) bitmap
                                  //         >0 means it's a bottom-up bitmap with origin
                                  //            in the lower left corner.
                                  //         <0 means it's a top-down bitmap with origin
                                  //            in the upper left corner.
  unsigned short biPlanes;        // WORD  = number of planes 
                                  //         (always 1 in current Windows versions)
  unsigned short biBitCount;      // WORD  = bits per pixel (0,1,4,8,16,24,32 are valid)
                                  //         1 See http://msdn.microsoft.com/default.asp  
                                  //         4 See http://msdn.microsoft.com/default.asp  
                                  //         8 The bitmap has a maximum of 256 colors, 
                                  //           and the bmiColors member contains up 
                                  //           to 256 entries. In this case, each byte
                                  //           in the array represents a single pixel. 
                                  //        16 See http://msdn.microsoft.com/default.asp  
                                  //        24 If biClrUsed=0 and biCompression=BI_RGB(0),
                                  //           then each 3-byte triplet in the bitmap 
                                  //           array represents the relative intensities
                                  //           of blue, green, and red, respectively, for
                                  //           a pixel. For other possibilities, see
                                  //           http://msdn.microsoft.com/default.asp  
                                  //        32 If biClrUsed=0 and biCompression=BI_RGB(0),
                                  //           then each 4-byte DWORD in the bitmap 
                                  //           array represents the relative intensities
                                  //           of blue, green, and red, respectively, for
                                  //           a pixel. The high byte in each DWORD is not
                                  //           used.  
                                  //           If biClrUsed=3, biCompression=BITFIELDS(3),
                                  //           biColors[0] = red mask (0x00FF0000), 
                                  //           biColors[1] = green mask (0x0000FF00), and
                                  //           biColors[2] = blue mask (0x000000FF),
                                  //           then tese masks are used with each 4-byte
                                  //           DWORD in the bitmap array to determine
                                  //           the pixel's relative intensities.                                 //           
                                  //           For other possibilities, see
                                  //           http://msdn.microsoft.com/default.asp  
  unsigned int   biCompression;   // DWORD   Currently, Windows defines the following
                                  //         types of compression.
                                  //         =0  BI_RGB (no compression)
                                  //         =1  BI_RLE8 (run length encoded used for 8 bpp)
                                  //         =2  BI_RLE4 (run length encoded used for 4 bpp)
                                  //         =3  BI_BITFIELDS  Specifies that the bitmap is
                                  //             not compressed and that the color table 
                                  //             consists of three DWORD color masks that
                                  //             specify the red, green, and blue components,
                                  //             respectively, of each pixel. This is valid
                                  //             when used with 16- and 32-bit-per-pixel
                                  //             bitmaps.
                                  //         =4  BI_JPEG (not supported in Win 95/NT4)
                                  //
  unsigned int   biSizeImage;     // DWORD = bytes in image
  int            biXPelsPerMeter; // LONG
  int            biYPelsPerMeter; // LONG
  unsigned int   biClrUsed;       // DWORD = 0 or true length of bmiColors[] array.  If 0,
                                  //           then the value of biBitCount determines the
                                  //           length of the bmiColors[] array.
  unsigned int   biClrImportant;  // DWORD
};

struct ON_WindowsRGBQUAD {
  // Mimics Windows RGBQUAD structure.
  // For details searh for "RGBQUAD" at http://msdn.microsoft.com/default.asp 
  unsigned char rgbBlue;      // BYTE
  unsigned char rgbGreen;     // BYTE
  unsigned char rgbRed;       // BYTE
  unsigned char rgbReserved;  // BYTE
};

struct ON_WindowsBITMAPINFO
{
  // Mimics Windows BITMAPINFO structure.
  // For details searh for "BITMAPINFO" at http://msdn.microsoft.com/default.asp 
  ON_WindowsBITMAPINFOHEADER bmiHeader;
  ON_WindowsRGBQUAD bmiColors[1]; // The "[1]" is for the compiler.  In
                                  // practice this array commonly has
                                  // length 0, 3, or 256 and a BITMAPINFO*
                                  // points to a contiguous piece of memory
                                  // that contains
                                  //
                                  //          BITMAPINFOHEADER
                                  //          RGBQUAD[length determined by flags]
                                  //          unsigned char[biSizeImage]
                                  //
                                  // See the ON_WindowsBITMAPINFOHEADER comments
                                  // and http://msdn.microsoft.com/default.asp
                                  // for more details.
};

#endif

// OBSOLETE // class ON_OpenGLBitmap;

class ON_CLASS ON_WindowsBitmap : public ON_Bitmap
{
  ON_OBJECT_DECLARE(ON_WindowsBitmap);
  // Uncompressed 8 bpp, 24 bpp, or 32 bpp Windows device 
  // independent bitmaps (DIB)
public:

  ON_WindowsBitmap();
  ON_WindowsBitmap( const ON_WindowsBitmap& );
  ~ON_WindowsBitmap();

  ON_WindowsBitmap& operator=( const ON_WindowsBitmap& );

  void EmergencyDestroy();
  void Destroy();

  bool Create( 
         int, // width
         int, // height
         int  // bits per pixel ( 1, 2, 4, 8, 16, 24, or 32 )
         );

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  ON_BOOL32 Write( ON_BinaryArchive& ) const; // writes compressed image
  ON_BOOL32 Read( ON_BinaryArchive& );        // reads compressed image
  bool WriteCompressed( ON_BinaryArchive& ) const;
  bool ReadCompressed( ON_BinaryArchive& );
  bool WriteUncompressed( ON_BinaryArchive& ) const;
  bool ReadUncompressed( ON_BinaryArchive& );

  int Width() const;
  int Height() const; // >0 means it's a bottom-up bitmap with origin at lower right
                      // <0 means it's a top-down bitmap with origin at upper left

  int PaletteColorCount() const; // number of colors in palette
  int SizeofPalette() const;     // number of bytes in palette
  int BitsPerPixel() const;
  //int SizeofPixel() const;       // number of bytes per pixel
  int SizeofScan() const;        // number of bytes per scan line
  int SizeofImage() const;       // number of bytes in image

  unsigned char* Bits(
    int // index of scan line 
    );
  const unsigned char* Bits(
    int // index of scan line 
    ) const;

  //int PaletteIndex( ON_Color ) const; // for 8bpp bitmaps

  ON_Color Pixel( 
    int, // 0 <= i < width
    int  // 0 <= j < height
    ) const;
  ON_Color Pixel( 
    int,  // 0 <= i < width
    const unsigned char* // value of Bits( j )
    ) const;

  //ON_BOOL32 SetColor( // sets entire map to specified color 
  //       ON_Color
  //       );

#if defined(ON_OS_WINDOWS_GDI)

  /*
  Description:
    Create an ON_WindowsBitmap from a contiguous bitmap.
    Copies src.
  Parameters:
    src - [in] contiguous Windows device independent bitmap.
  Remarks:
    If the current Windows BITMAPINFO is identical to ON_WindowsBITMAPINFO,
    then the result of this call is identical to

         int color_count = number of colors in bitmap's palette;
         ON_WindowsBitmap::Create( &src, &src.bmiColors[color_count], true ).

  See Also:
    ON_WindowsBitmap::Create    
  */
  ON_WindowsBitmap( const BITMAPINFO& src );

  /*
  Description:
    Create an ON_WindowsBitmap from a contiguous bitmap.
    Shares bitmap memory with src.
  Parameters:
    src - [in] contiguous Windows device independent bitmap.
  See Also:
    ON_WindowsBitmap::Create    
  Remarks:
    ~ON_WindowsBitmap will not delete src.
  */
  ON_WindowsBitmap( const BITMAPINFO* src );

  /*
  Description:
    Create an ON_WindowsBitmap from a contiguous bitmap.
    Copies src.
  Parameters:
    src - [in] contiguous Windows device independent bitmap.
  See Also:
    ON_WindowsBitmap::Create    
  */
  ON_WindowsBitmap& operator=( const BITMAPINFO& src );

  /*
  Description:
    Create and ON_WindowsBitmap from a Windows BITMAPINFO pointer
    and a pointer to the bits.

    This is intended to make it easy to write compressed bimaps.
    For ON_WindowsBitmap classes created with ON_WindowsBitmap::Share,
    ON_WindowsBitmap::Destroy and ~ON_WindowsBitmap will
    not free the bmi and bits memory.

  Parameters:
    bmi  - [in] valid BITMAPINFO
    bits - [in] bits for BITMAPINFO
    bCopy - [in] If true, the bmi and bits are copied into a contiguous
                 bitmap that will be deleted by ~ON_WindowsBitmap.
                 If false, the m_bmi and m_bits pointers on this class
                 are simply set to bmi and bits.  In this case,
                 ~ON_WindowsBitmap will not free the bmi or bits
                 memory.

  Example:

          ON_BinaryArchive archive = ...;
          BITMAPINFO* bmi = 0;
          unsigned char* bits = 0;
          int color_count = ...; // number of colors in palette

          int sizeof_palette = sizeof(bmi->bmiColors[0]) * color_count;

          BITMAPINFO* bmi = (LPBITMAPINFO)calloc( 1, sizeof(*bmi) + sizeof_palette );

          bmi->bmiHeader.biSize          = sizeof(bmi->bmiHeader);
          bmi->bmiHeader.biWidth         = width;
          bmi->bmiHeader.biHeight        = height;
          bmi->bmiHeader.biPlanes        = 1;
          bmi->bmiHeader.biBitCount      = (USHORT)color_depth;
          bmi->bmiHeader.biCompression   = BI_RGB;                  
          bmi->bmiHeader.biXPelsPerMeter = 0;
          bmi->bmiHeader.biYPelsPerMeter = 0;
          bmi->bmiHeader.biClrUsed       = 0;
          bmi->bmiHeader.biClrImportant  = 0;
          bmi->bmiHeader.biSizeImage     = GetStorageSize();

          // initialize palette
          ...

          HBITMAP hbm = ::CreateDIBSection( NULL, bmi, ..., (LPVOID*)&bits, NULL, 0);

          {
            // Use ON_WindowsBitmap to write a compressed bitmap to 
            // archive.  Does not modify bmi or bits.
            ON_WindowsBitmap onbm;
            onbm.Create(bmi,bit,false);
            onbm.Write( arcive );
          }

  */
  bool Create( const BITMAPINFO* bmi, 
               const unsigned char* bits,
               bool bCopy
             );

#endif

  /*
  Returns:
    True if m_bmi and m_bits are in a single contiguous 
    block of memory.
    False if m_bmi and m_bits are in two blocks of memory.    
  */
  bool IsContiguous() const;

#if defined(ON_OS_WINDOWS_GDI)
  BITMAPINFO*                  m_bmi;
#else
  struct ON_WindowsBITMAPINFO* m_bmi;
#endif

  unsigned char*               m_bits;

private:
  int m_bFreeBMI; // 0 m_bmi and m_bits are not freed by ON_WindowsBitmap::Destroy
                  // 1 m_bmi  memory is freed by ON_WindowsBitmap::Destroy
                  // 2 m_bits memory is freed by ON_WindowsBitmap::Destroy
                  // 3 m_bmi and m_bits memory is freed by ON_WindowsBitmap::Destroy                    
};

/*
Description:
  ON_WindowsBitmapEx is identical to ON_WindowsBitmap except that
  it's Read/Write functions save bitmap names.
*/
class ON_CLASS ON_WindowsBitmapEx : public ON_WindowsBitmap
{
  ON_OBJECT_DECLARE(ON_WindowsBitmapEx);
public:
  ON_WindowsBitmapEx();
  ~ON_WindowsBitmapEx();
  ON_BOOL32 Write( ON_BinaryArchive& ) const; // writes compressed image
  ON_BOOL32 Read( ON_BinaryArchive& );        // reads compressed image
};

class ON_CLASS ON_EmbeddedBitmap : public ON_Bitmap
{
  ON_OBJECT_DECLARE(ON_EmbeddedBitmap);
public:
  ON_EmbeddedBitmap();
  ~ON_EmbeddedBitmap();
  void EmergencyDestroy();
  void Destroy();
  void Create( int sizeof_buffer );

  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  ON_BOOL32 Write( ON_BinaryArchive& ) const;
  ON_BOOL32 Read( ON_BinaryArchive& );

  int Width() const;
  int Height() const;
  int BitsPerPixel() const;
  int SizeofScan() const;
  int SizeofImage() const;
  unsigned char* Bits(int);
  const unsigned char* Bits(int) const;

  void* m_buffer;
  size_t m_sizeof_buffer;
  int m_free_buffer; // 1 = ~ON_EmbeddedBitmap will onfree m_buffer.
  ON__UINT32 m_biffer_crc32; // 32 bit crc from ON_CRC32
};


#endif

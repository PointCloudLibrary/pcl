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

FILE* ON_FileStream::Open( const wchar_t* filename, const wchar_t* mode )
{
  FILE* fp = 0;

  if ( 0 == filename || 0 == filename[0] || 0 == mode || 0 == mode[0] )
    return fp;

#if defined(ON_OS_WINDOWS)
  errno_t e = _wfopen_s(&fp,filename,mode); 
  if ( 0 != e && 0 == fp )
    fp = 0; // reference e to keep lint quiet.
#else
  // I can't find an wfopen() or _wfopen() in
  // gcc version egcs-2.91.66 19990314/Linux (egcs-1.1.2 release)
  ON_String fnameUTF8(filename);
  ON_String modeUTF8(mode);
  fp = fopen(fnameUTF8,modeUTF8);
#endif

  return fp;
}

FILE* ON_FileStream::Open( const char* filename, const char* mode )
{
  FILE* fp = 0;

  if ( 0 == filename || 0 == filename[0] || 0 == mode || 0 == mode[0] )
    return fp;

#if defined(ON_OS_WINDOWS)
  errno_t e = fopen_s(&fp,filename,mode); 
  if ( 0 != e && 0 == fp )
    fp = 0; // reference e to keep lint quiet.
#else
  fp = fopen(filename,mode);
#endif

  return fp;
}

int ON_FileStream::Close( FILE* fp )
{
  return ( ( 0 != fp ) ? fclose(fp) : -1 );
}

ON__INT64 ON_FileStream::CurrentPosition( FILE* fp )
{
  if ( 0 == fp )
    return -1;
#if defined(ON_OS_WINDOWS)
  return _ftelli64(fp);
#else
  return ftell(fp);
#endif
}

bool ON_FileStream::SeekFromCurrentPosition( FILE* fp, ON__INT64 offset )
{
  return ON_FileStream::Seek(fp,offset,SEEK_CUR);
}

bool ON_FileStream::SeekFromStart( FILE* fp, ON__INT64 offset )
{
  return ON_FileStream::Seek(fp,offset,SEEK_SET);
}

bool ON_FileStream::SeekFromEnd( FILE* fp, ON__INT64 offset )
{
  return ON_FileStream::Seek(fp,offset,SEEK_END);
}

bool ON_FileStream::Seek( FILE* fp, ON__INT64 offset, int origin )
{
  if ( 0 == fp )
    return false;

  if ( origin < 0 || origin > 2 )
    return false;

  if ( 0 == offset )
    return true;
  
#if defined(ON_OS_WINDOWS)

  if ( 0 != _fseeki64(fp,offset,origin) )
    return false;

#else

  const int i = 2147483646;
  const ON__INT64 i64 = i;
  while ( offset > i64 )
  {
    if ( 0 != fseek( fp, i, origin ) )
      return false;
    offset -= i64;
  }
  while ( offset < -i64 )
  {
    if ( 0 != fseek( fp, -i, origin ) )
      return false;
    offset += i64;
  }
  if ( 0 != offset )
  {
    int ioffset = (int)offset;
    if ( 0 != fseek( fp, ioffset, origin ) )
      return false;
  }

#endif

  return true;
}

ON__UINT64 ON_FileStream::Read( FILE* fp, ON__UINT64 count, void* buffer )
{
  ON__UINT64 rc = 0;
  if ( 0 == fp || count <= 0 || 0 == buffer )
    return rc;

  if ( count <= ON_MAX_SIZE_T )
  {
    rc = (ON__UINT64)fread(buffer,1,(size_t)count,fp);
  }
  else
  {
    size_t sz, szread;
    while ( count > 0 )
    {
      sz = ( count > ON_MAX_SIZE_T ) ? ON_MAX_SIZE_T : ((size_t)count);
      szread = fread(buffer,1,sz,fp);
      rc += szread;
      if ( szread != sz )
        break;
      count -= sz;
      buffer = ((unsigned char*)buffer) + sz;
    }
  }

 return rc;
}

ON__UINT64 ON_FileStream::Write( FILE* fp, ON__UINT64 count, const void* buffer )
{
  ON__UINT64 rc = 0;
  if ( 0 == fp || count <= 0 || 0 == buffer )
    return rc;

  if ( count <= ON_MAX_SIZE_T )
  {
    rc = fwrite(buffer,1,(size_t)count,fp);
  }
  else
  {
    size_t sz, szwrite;
    while ( count > 0 )
    {
      sz = ( count > ON_MAX_SIZE_T ) ? ON_MAX_SIZE_T : ((size_t)count);
      szwrite = fwrite(buffer,1,sz,fp);
      rc += szwrite;
      if ( szwrite != sz )
        break;
      count -= sz;
      buffer = ((unsigned char*)buffer) + sz;
    }
  }

 return rc;
}

bool ON_FileStream::Flush( FILE* fp )
{
  if ( 0 == fp )
    return false;
  if ( 0 != fflush(fp) )
    return false;
  return true;
}


bool ON_FileStream::GetFileInformation( 
    FILE* fp,
    ON__UINT64* file_size,
    ON__UINT64* file_create_time,
    ON__UINT64* file_last_modified_time
    )
{
  bool rc = false;

  if (file_size)
    *file_size = 0;
  if (file_create_time)
    *file_create_time = 0;
  if (file_last_modified_time)
    *file_last_modified_time = 0;

  if ( fp )
  {

#if defined(ON_COMPILER_MSC)

    // Microsoft compilers
    int fd = _fileno(fp);    
#if (_MSC_VER >= 1400)
    // VC 8 (2005) 
    // works for file sizes > 4GB 
    // when size_t is a 64 bit integer
    struct _stat64 sb;
    memset(&sb,0,sizeof(sb));
    int fstat_rc = _fstat64(fd, &sb);
#else
    // VC6 compiler
    // works on most compilers
    struct _stat sb;
    memset(&sb,0,sizeof(sb));
    int fstat_rc = _fstat(fd, &sb);
#endif

#else
    // works on most compilers
    int fd = fileno(fp);
    struct stat sb;
    memset(&sb,0,sizeof(sb));
    int fstat_rc = fstat(fd, &sb);
#endif


    if (0 == fstat_rc)
    {
      if (file_size)
        *file_size = (ON__UINT64)sb.st_size;
      if (file_create_time)
        *file_create_time = (ON__UINT64)sb.st_ctime;
      if (file_last_modified_time)
        *file_last_modified_time = (ON__UINT64)sb.st_mtime;
      rc = true;
    }
  }

  return rc;
}


class ON_ReadChunkHelper
{
public:
  ON_ReadChunkHelper(ON_BinaryArchive&);
  ~ON_ReadChunkHelper();

  ON_BinaryArchive& m_binary_archive;
  bool m_bReadSuccess;
  ON__UINT32 m_chunk_tcode;
  ON__INT64 m_chunk_value;

private:
  bool m_bCallEndRead3dmChunk;
  // prohibit use - no implementation
  ON_ReadChunkHelper();
  ON_ReadChunkHelper(const ON_ReadChunkHelper&);
  ON_ReadChunkHelper& operator=(const ON_ReadChunkHelper&);
};

ON_ReadChunkHelper::ON_ReadChunkHelper(ON_BinaryArchive& binary_archive)
: m_binary_archive(binary_archive)
, m_bReadSuccess(0)
, m_chunk_tcode(0)
, m_chunk_value(0)
, m_bCallEndRead3dmChunk(0)
{
  m_bReadSuccess = m_binary_archive.BeginRead3dmBigChunk(&m_chunk_tcode,&m_chunk_value);
  if ( m_bReadSuccess )
    m_bCallEndRead3dmChunk = true;
}

ON_ReadChunkHelper::~ON_ReadChunkHelper()
{
  if ( m_bReadSuccess && !m_binary_archive.EndRead3dmChunk() )
    m_bReadSuccess = false;
}

bool ON_IsUnsignedChunkTypecode( ON__UINT32 typecode )
{
  // returns tru if the chunk value should be treated as an unsigned int.
  return ( 0 == (TCODE_SHORT & typecode)
           || TCODE_RGB == typecode 
           || TCODE_RGBDISPLAY == typecode
           || TCODE_PROPERTIES_OPENNURBS_VERSION == typecode
           || TCODE_OBJECT_RECORD_TYPE == typecode
         );
}

bool ON_IsLongChunkTypecode(ON__UINT32 typecode)
{
  // NOTE: RenderXXXX plug-in used zero as a typecode in material userdata, sigh ...
  //return (0 != typecode && 0 == (TCODE_SHORT & typecode));
  return (0 == (TCODE_SHORT & typecode));
}

bool ON_IsShorChunkTypecode(ON__UINT32 typecode)
{
  return (0 != (TCODE_SHORT & typecode));
}

static
bool DownSizeINT( ON__INT64 i64, ON__INT32* i32 )
{
  const static ON__INT64 i32max = 2147483647;
  if ( i64 <= i32max && i64 >= (-i32max - 1) )
  {
    *i32 = (ON__INT32)i64;
    return true;
  }

  ON_ERROR("i64 too big to convert to 4 byte signed int");
  *i32 = 0;
  return false;
}

static
bool DownSizeUINT( ON__UINT64 u64, ON__UINT32* u32 )
{
  if ( u64 <= 0xFFFFFFFF )
  {
    *u32 = (ON__UINT32)u64;
    return true;
  }

  ON_ERROR("u64 too big to convert to 4 byte unsigned int");
  *u32 = 0;
  return false;
}

static
bool DownSizeChunkValue( ON__UINT32 typecode, ON__INT64 v64, ON__INT32* v32 )
{
  if ( 0 == v32 )
    return true;
  return ( ON_IsLongChunkTypecode(typecode) )
         ? DownSizeUINT( (ON__UINT64)v64, (ON__UINT32*)v32 )
         : DownSizeINT( v64, v32 );
}



struct ON__3dmV1LayerIndex
{
  int m_layer_index;
  int m_layer_name_length;
  char* m_layer_name;
  struct ON__3dmV1LayerIndex* m_next;
};

ON_BinaryArchive::ON_BinaryArchive( ON::archive_mode mode ) 
                 : m_3dm_version(0), 
                   m_3dm_v1_layer_index(0), m_3dm_v1_material_index(0),
                   m_error_message_mask(0),
                   m_3dm_opennurbs_version(0),
                   m_3dm_start_section_offset(0),
                   m_active_table(ON_BinaryArchive::no_active_table),
                   m_bDoChunkCRC(0), m_bad_CRC_count(0),
                   m_endian(ON::Endian()), 
                   m_mode(mode)
{
  // Sparc, MIPS, ... CPUs have big endian byte order
  // ON_BinaryArchives use little endian byte order

  m_bSaveUserData = true; // true to save user data (increases file size)
  m_bSavePreviewImage    = false; // true to save 200x200 preview bitmap (increases file size)
  m_bEmbedTextureBitmaps = false; // true to embed texture, bump, trace, and wallpaper bitmaps (increases file size)
  m_bSaveRenderMeshes    = false; // true to save meshes used to render B-rep objects (increases file size)
  m_bSaveAnalysisMeshes  = false; // true to save meshes used in surface analysis (increases file size)

  m_zlib.mode = ON::unknown_archive_mode;
  memset( &m_zlib.strm, 0, sizeof(m_zlib.strm) );

  m_V1_layer_list = 0;
}

ON_BinaryArchive::~ON_BinaryArchive()
{
  if ( 0 != m_V1_layer_list )
  {
    struct ON__3dmV1LayerIndex* next = m_V1_layer_list;
    m_V1_layer_list = 0;
    for ( int i = 0; 0 != next && i < 1000; i++ )
    {
      struct ON__3dmV1LayerIndex* p = next;
      next = p->m_next;
      onfree(p);
    }
  }

  CompressionEnd();
}


bool ON_BinaryArchive::ToggleByteOrder(
  int count,          // number of elements
  int sizeof_element, // size of element (2,4, or 8)
  const void* src,    // source buffer
  void* dst           // destination buffer (can be same as source buffer)
  )
{
  unsigned char c[32];
  const unsigned char* a = (const unsigned char*)src;
  unsigned char* b = (unsigned char*)dst;

  bool rc = (count==0 || (count>0&&src&&dst));
  if ( rc ) 
  {
    // loops are unrolled and a switch is used
    // to speed things up a bit.
    switch(sizeof_element) 
    {
    case 2:
      while(count--) 
      {
        c[0] = *a++;
        c[1] = *a++;
        *b++ = c[1];
        *b++ = c[0];
      }
      break;

    case 4:
      while(count--) 
      {
        c[0] = *a++;
        c[1] = *a++;
        c[2] = *a++;
        c[3] = *a++;
        *b++ = c[3];
        *b++ = c[2];
        *b++ = c[1];
        *b++ = c[0];
      }
      break;

    case 8:
      while(count--) 
      {
        c[0] = *a++;
        c[1] = *a++;
        c[2] = *a++;
        c[3] = *a++;
        c[4] = *a++;
        c[5] = *a++;
        c[6] = *a++;
        c[7] = *a++;
        *b++ = c[7];
        *b++ = c[6];
        *b++ = c[5];
        *b++ = c[4];
        *b++ = c[3];
        *b++ = c[2];
        *b++ = c[1];
        *b++ = c[0];
      }
      break;

    default:
      if ( sizeof_element > 0 && sizeof_element < 32 )
      {
        // As of 2 May 2003, this case is never used
        // by core opennurbs objects.
        //
        // This is here so that future code will work
        // if and when 128 bit "ints"/"doubles" become common
        // enough that they can be stored in 3dm files.
        // It may also happen that third party applications
        // on specialized CPUs need to toggle byte order
        // for 128 bit ints/doubles stored as user data.
        int i;
        while(count--)
        {
          for (i = 0; i < sizeof_element; i++)
            c[i] = *a++;
          while(i--)
            *b++ = c[i];
        }
      }
      else
      {
        rc = false;
      }
      break;
    }
  }
  return rc;
}

bool ON_BinaryArchive::BigSeekFromStart( ON__UINT64 offset )
{
  // SeekFromStart() is a virutal function that
  // any developer can implement.  Some implementations
  // may use signed 4 byte int in critical places.
  // BigSeekFromStart() will work correctly in
  // this worst case situation.
  return ( offset > 2147483632 )
    ? ( SeekFromStart(2147483632) && BigSeekForward(offset - 2147483632) )
    : SeekFromStart((size_t)offset);
}



bool ON_BinaryArchive::BigSeekForward( ON__UINT64 offset )
{
  // SeekFromCurrentPosition() is a virutal function that
  // uses a signed 4 byte int in critical places.
  // BigSeekForward() will work correctly when
  // offset is larger than the maximum value of a signed
  // 4 byte int.
  while ( offset > 2147483632 )
  {
    if ( !SeekFromCurrentPosition(2147483632) )
      return false;
    offset -= 2147483632;
  }
  if ( offset > 0 )
  {
    int ioffset32 = (int)((ON__INT64)offset);
    return SeekFromCurrentPosition(ioffset32);
  }
  return true;
}


bool ON_BinaryArchive::BigSeekBackward( ON__UINT64 offset )
{
  // SeekFromCurrentPosition() is a virutal function that
  // uses a signed 4 byte int in critical places.
  // BigSeekBackward() will work correctly when
  // offset is larger than the maximum value of a signed
  // 4 byte int.
  while ( offset > 2147483632 )
  {
    if ( !SeekFromCurrentPosition(-2147483632) )
      return false;
    offset -= 2147483632;
  }
  if ( offset > 0 )
  {
    int ioffset32 = (int)((ON__INT64)offset);
    return SeekFromCurrentPosition(-ioffset32);
  }
  return true;
}


bool ON_BinaryArchive::BigSeekFromCurrentPosition( ON__INT64 offset )
{
  // SeekFromCurrentPosition() is a virutal function that
  // uses a signed 4 byte int in critical places.
  // BigSeekFromCurrentPosition() will work correctly when
  // offset is smaller than the minimum value of a signed
  // 4 byte int or the maximum value of a signed 4 byte int.
  return ( offset >= 0 ) 
         ? BigSeekForward((ON__UINT64)offset)
         : BigSeekBackward((ON__UINT64)(-offset));
}

bool
ON_BinaryArchive::ReadMode() const
{
  return (m_mode & ON::read) ? true : false;
}

bool
ON_BinaryArchive::WriteMode() const
{
  return (m_mode & ON::write) ? true : false;
}

bool
ON_BinaryArchive::ReadChar(    // Read an array of 8 bit chars
		size_t count,       // number of chars to read
		char*  p  
		)
{
  return ReadByte( count, p );
}

bool
ON_BinaryArchive::ReadChar(    // Read an array of 8 bit unsigned chars
		size_t count,       // number of unsigned chars to read
		unsigned char* p   
		)
{
  return ReadByte( count, p );
}

bool
ON_BinaryArchive::ReadChar(    // Read a single 8 bit char
		char* p
		)
{
  return ReadByte( 1, p );
}

bool
ON_BinaryArchive::ReadChar(    // Read a single 8 bit unsigned char
		unsigned char* p
		)
{
  return ReadByte( 1, p );
}

bool
ON_BinaryArchive::ReadInt16( // Read an array of 16 bit integers
		size_t count,            // number of unsigned integers to read
		ON__INT16* p
		)
{
  bool rc = ReadByte( count<<1, p );
  if ( rc && m_endian == ON::big_endian )
  {
    // reverse byte order
		unsigned char* b= (unsigned char*) (p);
		unsigned char  c;
		while(count--) {
			c = b[0]; b[0] = b[1]; b[1] = c;
			b += 2;
		}
  }
  return rc;
}

bool
ON_BinaryArchive::ReadShort(   // Read an array of 16 bit shorts
		size_t count,       // number of unsigned chars to read
		short* p
		)
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 2 == sizeof(*p).  Since this code has to run on machines
// where sizeof(*p) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  bool rc = true;

  if ( 2 == sizeof(*p) )
  {
    rc = ReadInt16( count, (ON__INT16*)p );
  }
  else
  {
    size_t j;
    ON__INT16 i16;
    for ( j = 0; j < count && rc; j++ )
    {
      rc = ReadInt16( 1, &i16 );
      *p++ = (short)i16;
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::ReadShort(   // Read an array of 16 bit unsigned shorts
		size_t count,       // number of unsigned chars to read
		unsigned short* p
		)
{
  return ReadShort( count, (short*)p );
}

bool
ON_BinaryArchive::ReadShort(   // Read a single 16 bit short
		short* p
		)
{
  return ReadShort( 1, p );
}

bool
ON_BinaryArchive::ReadShort(   // Read a single 16 bit unsigned short
		unsigned short* p
		)
{
  return ReadShort( 1, p );
}

bool
ON_BinaryArchive::ReadInt32( // Read an array of 32 bit integers
		size_t count,            // number of 32 bit integers to read
		ON__INT32* p
		)
{
  bool rc = ReadByte( count<<2, p );
  if ( rc && m_endian == ON::big_endian ) 
  {
		unsigned char* b= (unsigned char*)p;
		unsigned char  c;
		while(count--) {
			c = b[0]; b[0] = b[3]; b[3] = c;
			c = b[1]; b[1] = b[2]; b[2] = c;
			b += 4;
		}
  }
  return rc;
}

bool
ON_BinaryArchive::ReadInt( // Read an array of integers
		size_t count,          // number of unsigned chars to read
		int* p
		)
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 4 == sizeof(*p).  Since this code has to run on machines
// where sizeof(*p) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  bool rc;
  if ( 4 == sizeof(*p) )
  {
    rc = ReadInt32( count, (ON__INT32*)p );
  }
  else
  {
    rc = true;
    ON__INT32 i32;
    size_t j;
    for ( j = 0; j < count && rc; j++ )
    {
      rc = ReadInt32(1,&i32);
      if (rc)
        *p++ = (int)i32;
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::ReadInt( // Read an array of 32 bit integers
		size_t count,       // number of unsigned chars to read
		unsigned int* p
		)
{
  return ReadInt( count, (int*)p );
}

bool
ON_BinaryArchive::ReadInt( // Read a single 32 bit integer
		int* p
		)
{
  return ReadInt( 1, p );
}

bool
ON_BinaryArchive::ReadInt( // Read a single 32 bit unsigned integer
		unsigned int* p
		)
{
  return ReadInt( 1, p );
}

bool ON_BinaryArchive::ReadBigInt( // Read an array of 64 bit integers
		size_t count,
		ON__INT64* p 
		)
{
  return ReadInt64(1,p);
}

bool ON_BinaryArchive::ReadBigInt( // Read an array of 64 bit integers
		size_t count,
		ON__UINT64* p
		)
{
  return ReadInt64(1,(ON__INT64*)p);
}

bool ON_BinaryArchive::ReadBigInt( // Read a single 64 bit integer
		ON__INT64* p
		)
{
  return ReadInt64(1,p);
}

bool ON_BinaryArchive::ReadBigInt( // Read a single 64 bit unsigned integer
		ON__UINT64* p
		)
{
  return ReadInt64(1,(ON__INT64*)p);
}



bool
ON_BinaryArchive::ReadLong( // Read an array of 32 bit integers
		size_t count,       // number of unsigned chars to read
		long* p
		)
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 4 == sizeof(*p).  Since this code has to run on machines
// where sizeof(*p) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  bool rc;
  if ( 4 == sizeof(*p) )
  {
    rc = ReadInt32( count, (ON__INT32*)p );
  }
  else
  {
    rc = true;
    ON__INT32 i32;
    size_t j;
    for ( j = 0; j < count && rc; j++ )
    {
      rc = ReadInt32(1,&i32);
      if (rc)
        *p++ = (long)i32;
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::ReadLong( // Read an array of 32 bit integers
		size_t count,       // number of unsigned chars to read
		unsigned long* p
		)
{
  return ReadLong( count, (long*)p );
}

bool
ON_BinaryArchive::ReadLong( // Read a single 32 bit integer
		long* p
		)
{
  return ReadLong( 1, (long*)p );
}

bool
ON_BinaryArchive::ReadLong( // Read a single 32 bit unsigned integer
		unsigned long* p
		)
{
  return ReadLong( 1, (long*)p );
}

bool
ON_BinaryArchive::ReadFloat(   // Read an array of floats
		size_t count,       // number of unsigned chars to read
		float* p
		)
{
  // 32 bit floats and 32 bit integers have same size and endian issues
  return ReadInt32( count, (ON__INT32*)p );
}

bool
ON_BinaryArchive::ReadFloat(   // Read a single float
		float* p
		)
{
  return ReadFloat( 1, p );
}

bool
ON_BinaryArchive::ReadDouble(  // Read an array of IEEE 64 bit doubles
		size_t count,       // number of unsigned chars to read
		double* p
		)
{
  bool rc = ReadByte( count<<3, p );
  if ( rc && m_endian == ON::big_endian ) 
  {
		unsigned char* b=(unsigned char*)p;
		unsigned char  c;
		while(count--) {
			c = b[0]; b[0] = b[7]; b[7] = c;
			c = b[1]; b[1] = b[6]; b[6] = c;
			c = b[2]; b[2] = b[5]; b[5] = c;
			c = b[3]; b[3] = b[4]; b[4] = c;
			b += 8;
		}
  }
  return rc;
}

bool
ON_BinaryArchive::ReadDouble(  // Read a single double
		double* p
		)
{
  return ReadDouble( 1, p );
}

bool
ON_BinaryArchive::ReadColor( ON_Color& color )
{
  unsigned int colorref = 0;
  bool rc = ReadInt( &colorref );
  color = colorref;
  return rc;
}

bool
ON_BinaryArchive::ReadPoint (
  ON_2dPoint& p
  )
{
  return ReadDouble( 2, &p.x );
}

bool
ON_BinaryArchive::ReadPoint (
  ON_3dPoint& p
  )
{
  return ReadDouble( 3, &p.x );
}

bool
ON_BinaryArchive::ReadPoint (
  ON_4dPoint& p
  )
{
  return ReadDouble( 4, &p.x );
}

bool
ON_BinaryArchive::ReadVector (
  ON_2dVector& v
  )
{
  return ReadDouble( 2, &v.x );
}

bool
ON_BinaryArchive::ReadVector (
  ON_3dVector& v
  )
{
  return ReadDouble( 3, &v.x );
}

bool ON_BinaryArchive::WriteBoundingBox(const ON_BoundingBox& bbox)
{
  bool rc = WritePoint( bbox.m_min );
  if (rc) rc = WritePoint( bbox.m_max );
  return rc;
}

bool ON_BinaryArchive::ReadBoundingBox(ON_BoundingBox& bbox)
{
  bool rc = ReadPoint( bbox.m_min );
  if (rc) rc = ReadPoint( bbox.m_max );
  return rc;
}

bool 
ON_BinaryArchive::WriteXform( const ON_Xform& x )
{
  return WriteDouble( 16, &x.m_xform[0][0] );
}

bool 
ON_BinaryArchive::ReadXform( ON_Xform& x )
{
  return ReadDouble( 16, &x.m_xform[0][0] );
}
bool 
ON_BinaryArchive::WritePlaneEquation( const ON_PlaneEquation& plane_equation )
{
  bool rc = WriteDouble( 4, &plane_equation.x );
  return rc;
}

bool 
ON_BinaryArchive::ReadPlaneEquation( ON_PlaneEquation& plane_equation )
{
  bool rc = ReadDouble( 4, &plane_equation.x );
  return rc;
}

bool 
ON_BinaryArchive::WritePlane( const ON_Plane& plane )
{
  bool rc = WritePoint( plane.origin );
  if (rc) rc = WriteVector( plane.xaxis );
  if (rc) rc = WriteVector( plane.yaxis );
  if (rc) rc = WriteVector( plane.zaxis );
  if (rc) rc = WriteDouble( 4, &plane.plane_equation.x );
  return rc;
}

bool 
ON_BinaryArchive::ReadPlane( ON_Plane& plane )
{
  bool rc = ReadPoint( plane.origin );
  if (rc) rc = ReadVector( plane.xaxis );
  if (rc) rc = ReadVector( plane.yaxis );
  if (rc) rc = ReadVector( plane.zaxis );
  if (rc) rc = ReadDouble( 4, &plane.plane_equation.x );
  return rc;
}

bool 
ON_BinaryArchive::WriteLine( const ON_Line& line )
{
  bool rc = WritePoint( line.from );
  if (rc) rc = WritePoint( line.to );
  return rc;
}

bool 
ON_BinaryArchive::ReadLine( ON_Line& line )
{
  bool rc = ReadPoint( line.from );
  if (rc) rc = ReadPoint( line.to );
  return rc;
}

bool 
ON_BinaryArchive::WriteArc(const ON_Arc& arc )
{
  bool rc = WriteCircle(arc);
  if (rc)
    rc = WriteInterval(arc.m_angle);
  return rc;
}

bool 
ON_BinaryArchive::ReadArc( ON_Arc& arc )
{
  bool rc = ReadCircle(arc);
  if (rc)
    rc = ReadInterval(arc.m_angle);
  return rc;
}

bool 
ON_BinaryArchive::WriteCircle(const ON_Circle& circle)
{
  bool rc = WritePlane( circle.plane );
  if (rc)
    rc = WriteDouble( circle.radius );
  // m_point[] removed 2001, November, 7
  if (rc)
    rc = WritePoint( circle.PointAt(0.0) );
  if (rc)
    rc = WritePoint( circle.PointAt(0.5*ON_PI) );
  if (rc)
    rc = WritePoint( circle.PointAt(ON_PI) );
  /*
  if (rc)
    rc = WritePoint( circle.m_point[0] );
  if (rc)
    rc = WritePoint( circle.m_point[1] );
  if (rc)
    rc = WritePoint( circle.m_point[2] );
  */
  return rc;
}

bool 
ON_BinaryArchive::ReadCircle(ON_Circle& circle)
{
  ON_3dPoint scratch;
  bool rc = ReadPlane( circle.plane );
  if (rc)
    rc = ReadDouble( &circle.radius );
  // m_point[] removed 2001, November, 7
  if (rc)
    rc = ReadPoint( scratch );
  if (rc)
    rc = ReadPoint( scratch );
  if (rc)
    rc = ReadPoint( scratch );
  /*
  if (rc)
    rc = ReadPoint( circle.m_point[0] );
  if (rc)
    rc = ReadPoint( circle.m_point[1] );
  if (rc)
    rc = ReadPoint( circle.m_point[2] );
  */
  return rc;
}


bool 
ON_BinaryArchive::WriteInterval( const ON_Interval& t )
{
  return WriteDouble( 2, t.m_t );
}

bool 
ON_BinaryArchive::ReadInterval( ON_Interval& t )
{
  return ReadDouble( 2, t.m_t );
}

bool
ON_BinaryArchive::ReadUuid( ON_UUID& uuid )
{
  bool    rc = ReadInt32( 1, (ON__INT32*)(&uuid.Data1) );
  if (rc) rc = ReadInt16( 1, (ON__INT16*)(&uuid.Data2) );
  if (rc) rc = ReadInt16( 1, (ON__INT16*)(&uuid.Data3) );
  if (rc) rc = ReadByte( 8, uuid.Data4 );
  return rc;
}

bool ON_BinaryArchive::ReadDisplayMaterialRef( ON_DisplayMaterialRef& dmr )
{
  bool rc = ReadUuid( dmr.m_viewport_id );
  if (rc) 
    rc = ReadUuid( dmr.m_display_material_id );
  return rc;
}

bool
ON_BinaryArchive::ReadTime( struct tm& utc )
{
  // utc = coordinated universal time ( a.k.a GMT, UTC )
  // (From ANSI C time() and gmtime().)
  bool rc = ReadInt( &utc.tm_sec );
  if ( rc )
    rc = ReadInt( &utc.tm_min );
  if ( rc )
    rc = ReadInt( &utc.tm_hour );
  if ( rc )
    rc = ReadInt( &utc.tm_mday );
  if ( rc )
    rc = ReadInt( &utc.tm_mon );
  if ( rc )
    rc = ReadInt( &utc.tm_year );
  if ( rc )
    rc = ReadInt( &utc.tm_wday );
  if ( rc )
    rc = ReadInt( &utc.tm_yday );
  if ( rc ) {
    if ( utc.tm_sec < 0 || utc.tm_sec > 60 )
      rc = false;
    if ( utc.tm_min < 0 || utc.tm_min > 60 )
      rc = false;
    if ( utc.tm_hour < 0 || utc.tm_hour > 24 )
      rc = false;
    if ( utc.tm_mday < 0 || utc.tm_mday > 31 )
      rc = false;
    if ( utc.tm_mon < 0 || utc.tm_mon > 12 )
      rc = false;
    // no year restrictions because dates are used in archeological userdata
    if ( utc.tm_wday < 0 || utc.tm_wday > 7 )
      rc = false;
    if ( utc.tm_yday < 0 || utc.tm_yday > 366 )
      rc = false;
    if ( !rc ) {
      ON_ERROR("ON_BinaryArchive::ReadTime() - bad time in archive");
    }
  }
  return rc;
}

bool
ON_BinaryArchive::ReadStringSize( // Read size of NULL terminated string
    size_t* sizeof_string          // (returned size includes NULL terminator)
    )
{
  ON__UINT32 ui32 = 0;
  bool rc = ReadInt32(1,(ON__INT32*)&ui32);
  // Note that ui32 = number of elements in the string array, including
  // the null terminator.  So ui32 should either be 0 or be >= 2.
  // The string array elements can be chars or unsigned shorts;
  // therefore the number of bytes in the string cannot be determined
  // at this point because we don't know what type of string is
  // being read.
  if (rc)
  {
    // 8 October 2004 Dale Lear
    //    Added the sanity checks on string size to avoid attempts
    //    to allocate huge amounts of memory when the value
    //    comes from a damaged file.
    if ( 0 != (0xF000000 & ui32) )
    {
      // 268 million chars oughta be plenty
      ON_ERROR("string element count is impossibly large");
      rc = false;
    }
    else if ( ui32 > 0 )
    {
      // make sure this is possible
      const ON_3DM_BIG_CHUNK* curchunk = m_chunk.Last();
      if ( 0 != curchunk && 0 == (TCODE_SHORT & curchunk->m_typecode) )
      {
        if (    curchunk->m_big_value < 0 
             || ((ON__INT64)ui32) > curchunk->m_big_value 
           )
        {
          ON_ERROR("string element count exceeds current chunk size");
          rc = false;
        }
      }
    }

    if (rc)
    {
      *sizeof_string = (size_t)ui32;
    }
  }
  return rc;
}


bool
ON_BinaryArchive::ReadStringUTF8ElementCount(
    size_t* string_utf8_element_count
    )
{
  ON__UINT32 ui32 = 0;
  bool rc = ReadInt32(1,(ON__INT32*)&ui32);
  // Note that ui32 = number of elements in the string array, including
  // the null terminator.  So ui32 should either be 0 or be >= 2.
  // The string array elements can be chars or unsigned shorts;
  // therefore the number of bytes in the string cannot be determined
  // at this point because we don't know what type of string is
  // being read.
  if (rc)
  {
    // 8 October 2004 Dale Lear
    //    Added the sanity checks on string size to avoid attempts
    //    to allocate huge amounts of memory when the value
    //    comes from a damaged file.
    if ( 0 != (0xF000000 & ui32) )
    {
      // 268 million chars oughta be plenty
      ON_ERROR("string element count is impossibly large");
      rc = false;
    }
    else if ( ui32 > 0 )
    {
      // make sure this is possible
      const ON_3DM_BIG_CHUNK* curchunk = m_chunk.Last();
      if ( 0 != curchunk && 0 == (TCODE_SHORT & curchunk->m_typecode) )
      {
        if (    curchunk->m_big_value < 0 
             || ((ON__INT64)ui32) > curchunk->m_big_value
           )
        {
          ON_ERROR("string byte count exceeds current chunk size");
          rc = false;
        }
      }
    }
  }
  if (!rc)
    ui32 = 0;
  if ( string_utf8_element_count )
    *string_utf8_element_count = (size_t)ui32;
  return rc;
}


bool
ON_BinaryArchive::ReadStringUTF16ElementCount(
    size_t* string_utf16_element_count
    )
{
  ON__UINT32 ui32 = 0;
  bool rc = ReadInt32(1,(ON__INT32*)&ui32);
  // Note that ui32 = number of elements in the string array, including
  // the null terminator.  So ui32 should either be 0 or be >= 2.
  // The string array elements can be chars or unsigned shorts;
  // therefore the number of bytes in the string cannot be determined
  // at this point because we don't know what type of string is
  // being read.
  if (rc)
  {
    if ( 0 != (0xF000000 & ui32) )
    {
      // 268 million chars oughta be plenty
      ON_ERROR("string element count is impossibly large");
      rc = false;
    }
    else if ( ui32 > 0 )
    {
      // make sure this is possible
      const ON_3DM_BIG_CHUNK* curchunk = m_chunk.Last();
      if ( 0 != curchunk && 0 == (TCODE_SHORT & curchunk->m_typecode) )
      {
        // 2*ui32 is used because opennurbs writes all wide character
        // strings as UTF-16 encoded strings.
        if (    curchunk->m_big_value < 0 
             || ((ON__INT64)2*ui32) > curchunk->m_big_value
           )
        {
          ON_ERROR("string byte count exceeds current chunk size");
          rc = false;
        }
      }
    }
  }

  if (!rc)
    ui32 = 0;
  if ( string_utf16_element_count )
    *string_utf16_element_count = (size_t)ui32;
  return rc;
}

bool
ON_BinaryArchive::ReadString(         // Read NULL terminated string
    size_t string_utf8_element_count, // = value from ReadStringUTF8ElementCount()
    char* p                           // array[string_utf8_element_count]
    )
{
  return ReadByte( string_utf8_element_count, p );
}

bool
ON_BinaryArchive::ReadString(         // Read NULL terminated string
    size_t string_utf8_element_count, // = value from ReadStringUTF8ElementCount()
    unsigned char* p                  // array[string_utf8_element_count]
    )
{
  return ReadByte( string_utf8_element_count, p );
}

bool
ON_BinaryArchive::ReadString(          // Read NULL terminated unicode string
    size_t string_utf16_element_count, // length = value ReadStringUTF16ElementCount ReadStringSize()
    unsigned short* p                  // array[string_utf16_element_count]
    )
{
  return ReadShort( string_utf16_element_count, p );
}

bool
ON_BinaryArchive::ReadString( ON_String& s )
{
  s.Destroy();
  size_t string_utf8_element_count = 0;
  bool rc = ReadStringUTF8ElementCount( &string_utf8_element_count );
  if ( rc && string_utf8_element_count > 0 ) 
  {
    const int istring_utf8_element_count = (int)string_utf8_element_count; // (int) converts 64 bits size_t
    s.ReserveArray(istring_utf8_element_count);
    ReadString( string_utf8_element_count, s.Array() );
    s.SetLength( istring_utf8_element_count-1 );
  }
  return rc;
}

bool
ON_BinaryArchive::ReadString( ON_wString& s )
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 2 == sizeof(wchar_t).  Since this code has to run on machines
// where sizeof(wchar_t) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  s.Destroy();
  size_t string_utf16_element_count = 0;
  bool rc = ReadStringUTF16ElementCount( &string_utf16_element_count );
  if ( rc && string_utf16_element_count > 0 ) 
  {
    // string_utf16_element_count = number of ON__INT16 elements in
    // the string.  This is almost always the same as the
    // number of unicode code points. However, if one of
    // the code points happens to require two ON__INT16 
    // values to encode, then string_utf16_element_count will be 
    // larger than the number of unicode code points in 
    // the array.
    const int istring_utf16_element_count = (int)string_utf16_element_count;
    if ( 2 == sizeof(wchar_t) ) 
    {
      // When sizeof(wchar_t) is 2 bytes, assume wchar_t strings are 
      // UTF-16 encoded unicode strings.
      s.ReserveArray( istring_utf16_element_count );
      rc = ReadInt16( string_utf16_element_count, (ON__INT16*)s.Array() );
      if (rc)
        s.SetLength( istring_utf16_element_count-1 );
    }
    else if ( 4 == sizeof(wchar_t)  )
    {
      // When sizeof(wchar_t) is 4 bytes, assume wchar_t strings are 
      // UTF-32 encoded unicode strings. (some gcc implementations do this.)

      // Read the UTF-16 encode string from the file into
      // utf16_buffer[].
      const int istring_utf16_element_count = (int)string_utf16_element_count;
      ON_SimpleArray<ON__UINT16> utf16_buffer(istring_utf16_element_count);
      rc = ReadInt16(string_utf16_element_count,(ON__INT16*)utf16_buffer.Array());
      if(rc)
      {
        // convert to a UTF-32 encoded unicode string.
        utf16_buffer.SetCount(istring_utf16_element_count);
        utf16_buffer[istring_utf16_element_count-1] = 0;
        rc = false;
        const ON__UINT16* sUTF16 = utf16_buffer.Array();
        const int bTestByteOrder = false;
        const int sUTF16_count = istring_utf16_element_count-1;
        const ON__UINT32 error_code_point = 0xFFFD;
        const unsigned int error_mask = 0xFFFFFFFF;
        unsigned int error_status = 0;

        const int utf32_array_count = ON_ConvertUTF16ToUTF32(
            bTestByteOrder,
            sUTF16,
            sUTF16_count,
            0, // unsigned int* sUTF32
            0, // int sUTF32_count
            &error_status,
            error_mask,
            error_code_point,
            0 // const ON__UINT16** sNextUTF16
            );

        if ( 0 == utf32_array_count )
        {
          rc = true;
        }
        else if ( utf32_array_count > 0 )
        {
          error_status = 0;
          s.ReserveArray(utf32_array_count+1);
          const int utf32_array_count1 = ON_ConvertUTF16ToUTF32(
              bTestByteOrder,
              sUTF16,
              sUTF16_count,
              (unsigned int*)s.Array(), // unsigned int* sUTF32
              utf32_array_count, // sUTF32_count
              &error_status,
              error_mask,
              error_code_point,
              0 // const ON__UINT16** sNextUTF16
              );
          if ( utf32_array_count1 == utf32_array_count )
          {
            s.SetLength( utf32_array_count );
            rc = true;
          }
        }
      }
    }
    if (!rc)
      s.Destroy();
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}


bool ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_MappingChannel>& a)
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    // ON_MappingChannel::Write() puts the element in a chunk
    rc = a[i].Write(*this);
  }
  return rc;
}

bool ON_BinaryArchive::WriteArray( const ON_ClassArray<ON_MaterialRef>& a)
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    // ON_MaterialRef::Write() puts the element in a chunk
    rc = a[i].Write(*this);
  }
  return rc;
}


bool ON_BinaryArchive::WriteArray( int count, const ON_Layer* a)
{
  int i;
  if ( count < 0 || 0 == a )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteObject(a[i]);
  }
  return rc;
}

bool ON_BinaryArchive::WriteArray( int count, const ON_Layer*const* a)
{
  int i;
  if ( count < 0 || 0 == a )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteObject(a[i]);
  }
  return rc;
}

bool ON_BinaryArchive::WriteArray( const ON_ClassArray<ON_MappingRef>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    // ON_MappingRef::Write() puts the element in a chunk
    rc = a[i].Write(*this);
  }
  return rc;
}


bool ON_BinaryArchive::WriteArray( const ON_ClassArray<ON_ObjRef>& a)
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    // ON_ObjRef::Write() puts the element in a chunk
    rc = a[i].Write(*this);
  }
  return rc;
}

bool ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_ObjRef_IRefID>& a)
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    // ON_ObjRef_IRefID::Write() puts the element in a chunk
    rc = a[i].Write(*this);
  }
  return rc;
}

bool ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_MappingChannel>& a )
{
  a.Empty();
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      rc = a.AppendNew().Read(*this);
    }
  }
  return rc;
}

bool ON_BinaryArchive::ReadArray( ON_ClassArray<ON_MaterialRef>& a)
{
  a.Empty();
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      rc = a.AppendNew().Read(*this);
    }
  }
  return rc;
}

bool ON_BinaryArchive::ReadArray( ON_ObjectArray<class ON_Layer>& a)
{
  a.Empty();
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      rc = (1 == ReadObject(a.AppendNew()));
      if (!rc)
      {
        a.Remove();
        break;
      }
    }
  }
  return rc;
}


bool ON_BinaryArchive::ReadArray( ON_SimpleArray<class ON_Layer*>& a)
{
  a.Empty();
  ON_Layer* layer;
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      layer = 0;
      ON_Object* p = 0;
      rc = (1==ReadObject(&p));
      if (rc)
      {
        layer = ON_Layer::Cast(p);
      }
      if (!rc || 0 == layer)
      {
        if ( p )
          delete p;
        rc = false;
        break;
      }
      a.Append(layer);
    }
  }
  return rc;
}

bool ON_BinaryArchive::ReadArray( ON_ClassArray<ON_MappingRef>& a)
{
  a.Empty();
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      rc = a.AppendNew().Read(*this);
    }
  }
  return rc;
}

bool ON_BinaryArchive::ReadArray( ON_ClassArray<ON_ObjRef>& a)
{
  a.Empty();
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      rc = a.AppendNew().Read(*this);
    }
  }
  return rc;
}

bool ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_ObjRef_IRefID>& a)
{
  a.Empty();
  int i, count;
  bool rc = ReadInt( &count );
  if (rc)
  {
    a.SetCapacity(count);
    for  ( i = 0; i < count && rc; i++ )
    {
      rc = a.AppendNew().Read(*this);
    }
  }
  return rc;
}


bool ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_DisplayMaterialRef>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    int i;
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadDisplayMaterialRef(a.AppendNew());
    }
  }
  return rc;
}

bool 
ON_BinaryArchive::ReadArray( ON_ClassArray<ON_String>& a)
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    int i;
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadString( a.AppendNew() );
    }
  }
  return rc;
}

bool 
ON_BinaryArchive::ReadArray( ON_ClassArray<ON_wString>& a)
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 )
  {
    a.SetCapacity( count );
    int i;
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadString( a.AppendNew() );
    }
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_DisplayMaterialRef>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteDisplayMaterialRef( a[i] );
  }
  return rc;
}

bool 
ON_BinaryArchive::WriteArray( const ON_ClassArray<ON_String>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteString( a[i] );
  }
  return rc;
}

bool 
ON_BinaryArchive::WriteArray( const ON_ClassArray<ON_wString>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteString( a[i] );
  }
  return rc;
}

bool 
ON_BinaryArchive::ReadArray( ON_SimpleArray<bool>& a )
{
#if defined(ON_COMPILER_MSC)
// Disable the MSC /W4 "conditional expression is constant" warning
// about sizeof(*c) == sizeof(*b).  Since this code has to run on machines
// where sizeof(bool) can be 1, 2, 4, or 8 bytes, the test is necessary.
#pragma warning( push )
#pragma warning( disable : 4127 )
#endif

  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    char* c = 0;
    bool* b = a.Array();
    if ( sizeof(*c) == sizeof(*b) )
    {
      // 8 bit "bool" on this compiler
      c = (char*)b;
    }
    else if ( b )
    {
      // bigger "bool" on this compiler
      c = (char*)onmalloc(count*sizeof(*c));
    }
    rc = ReadChar( count, c );
    if ( rc )
    {
      if ( c == (char*)b )
      {
        a.SetCount(count);
      }
      else if ( c )
      {
        int i;
        for ( i = 0; i < count; i++ )
        {
          a.Append(c[i]?true:false);
        }
        onfree(c);
      }
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif

}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<char>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadChar( count, a.Array() );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<short>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadShort( count, a.Array() );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<int>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadInt( count, a.Array() );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<float>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadFloat( count, a.Array() );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<double>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( count, a.Array() );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_Color>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadInt( count, (int*)a.Array() );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}


bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_2dPoint>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( 2*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_3dPoint>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( 3*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_4dPoint>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( 4*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_2dVector>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( 2*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_3dVector>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( 3*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_Xform>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    int i;
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadXform(a.AppendNew());
    }
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_2fPoint>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadFloat( 2*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_3fPoint>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadFloat( 3*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_4fPoint>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadFloat( 4*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_2fVector>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadFloat( 2*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_3fVector>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadFloat( 3*count, &a.Array()->x );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}

bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_UUID>& a )
{
  a.Empty();
  ON_UUID uuid;
  int i, count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadUuid( uuid );
      if ( rc )
        a.Append(uuid);
    }
  }
  return rc;
}




bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_UuidIndex>& a )
{
  a.Empty();
  ON_UuidIndex idi;
  int i, count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadUuid( idi.m_id );
      if ( rc )
      {
        rc = ReadInt(&idi.m_i);
        if(rc)
          a.Append(idi);
      }
    }
  }
  return rc;
}


bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_UUID>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteUuid( a[i] );
  }
  return rc;
}


bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_UuidIndex>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteUuid( a[i].m_id );
    if (rc)
      rc = WriteInt( a[i].m_i );
  }
  return rc;
}


bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_LinetypeSegment>& a )
{
  a.Empty();
  ON_LinetypeSegment seg;
  int i, count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity( count );
    for ( i = 0; i < count && rc; i++ )
    {
      rc = ReadLinetypeSegment( seg );
      if ( rc )
        a.Append(seg);
    }
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_LinetypeSegment>& a )
{
  int i, count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for  ( i = 0; i < count && rc; i++ )
  {
    rc = WriteLinetypeSegment( a[i] );
  }
  return rc;
}

bool ON_BinaryArchive::ReadLinetypeSegment(ON_LinetypeSegment& seg)
{
  seg.m_length = 1.0;
  seg.m_seg_type = ON_LinetypeSegment::stLine;
  unsigned int i;
  bool rc = ReadDouble(&seg.m_length);
  if (rc)
  {
    rc = ReadInt(&i);
    if( ON_LinetypeSegment::stLine == i )
      seg.m_seg_type = ON_LinetypeSegment::stLine;
    else if ( ON_LinetypeSegment::stSpace == i )
      seg.m_seg_type = ON_LinetypeSegment::stSpace;
  }
  return rc;
}


bool ON_BinaryArchive::WriteLinetypeSegment( const ON_LinetypeSegment& seg)
{
  // do not add chunk info here
  unsigned int i = seg.m_seg_type;
  bool rc = WriteDouble(seg.m_length);
  if (rc)
    rc = WriteInt(i);
  return rc;
}


bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_SurfaceCurvature>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) {
    a.SetCapacity( count );
    rc = ReadDouble( 2*count, &a.Array()->k1 );
    if ( rc )
      a.SetCount(count);
  }
  return rc;
}


bool
ON_BinaryArchive::ReadArray( ON_SimpleArray<ON_ClippingPlaneInfo>& a )
{
  a.Empty();
  int count = 0;
  bool rc = ReadInt( &count );
  if ( rc && count > 0 ) 
  {
    a.SetCapacity(count);
    for ( int i = 0; i < count && rc ; i++ )
    {
      rc = a.AppendNew().Read(*this);
      if (!rc)
        a.Remove();
    }
  }
  return rc;
}


bool ON_BinaryArchive::WriteBool( bool b )
{
  unsigned char c = (b?1:0);
  return WriteChar(c);
}

bool ON_BinaryArchive::ReadBool( bool *b )
{
  unsigned char c;
  bool rc = ReadChar(&c);
  if (rc && b)
  {
    if ( c != 0 && c != 1 )
    {
      // WriteBool always writes a 0 or 1.  So either your code
      // has a bug, the file is corrupt, the the file pointer
      // is where it should be.
      ON_ERROR("ON_BinaryArchive::ReadBool - bool value != 0 and != 1");
      rc = false;
    }
    *b = c?true:false;
  }
  return rc;
}

bool
ON_BinaryArchive::WriteChar(    // Write an array of 8 bit chars
		size_t count,       // number of chars to write
		const char* p   
		)
{
  return WriteByte( count, p );
}

bool
ON_BinaryArchive::WriteChar(    // Write an array of 8 bit unsigned chars
		size_t count,       // number of unsigned chars to write
		const unsigned char* p
		)
{
  return WriteByte( count, p );
}

bool
ON_BinaryArchive::WriteChar(    // Write a single 8 bit char
		char c
		)
{
  return WriteByte( 1, &c );
}

bool
ON_BinaryArchive::WriteChar(    // Write a single 8 bit unsigned char
		unsigned char c
		)
{
  return WriteByte( 1, &c );
}

bool
ON_BinaryArchive::WriteInt16(   // Write an array of 16 bit shorts
		size_t count,               // number of shorts to write
		const ON__INT16* p
		)
{
  bool rc = true;
  if ( m_endian == ON::big_endian )
  {
    if ( count > 0 ) 
    {
      const char* b = (const char*)p;
      while ( rc && count-- ) 
      {
        rc = WriteByte( 1, b+1 );
        if (rc)
          rc = WriteByte( 1, b );
        b++;
        b++;
      }
    }
  }
  else 
  {
    rc = WriteByte( count<<1, p );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteShort(   // Write an array of 16 bit shorts
		size_t count,       // number of shorts to write
		const short* p
		)
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 2 == sizeof(*p).  Since this code has to run on machines
// where sizeof(*p) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  bool rc;
  if ( 2 == sizeof(*p) )
  {
    rc = WriteInt16( count, (const ON__INT16*)p );
  }
  else
  {
    rc = true;
    ON__INT16 i16;
    size_t j;
    for ( j = 0; j < count; j++ )
    {
      i16 = (ON__INT16)(*p++);
      rc = WriteInt16( 1, &i16);
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::WriteShort(   // Write an array of 16 bit unsigned shorts
		size_t count,       // number of shorts to write
		const unsigned short* p
		)
{
  return WriteShort( count, (const short*)p );
}

bool
ON_BinaryArchive::WriteShort(   // Write a single 16 bit short
		short s
		)
{
  return WriteShort( 1, &s );
}

bool
ON_BinaryArchive::WriteShort(   // Write a single 16 bit unsigned short
		unsigned short s
		)
{
  return WriteShort( 1, &s );
}

bool
ON_BinaryArchive::WriteInt32( // Write an array of 32 bit integers
		size_t count,	            // number of ints to write
		const ON__INT32* p    
		)
{
  bool rc = true;
  if ( m_endian == ON::big_endian ) 
  {
    if ( count > 0 ) 
    {
      const char* b = (const char*)p;
      while ( rc && count-- ) 
      {
        rc = WriteByte( 1, b+3 );
        if (rc) rc = WriteByte( 1, b+2 );
        if (rc) rc = WriteByte( 1, b+1 );
        if (rc) rc = WriteByte( 1, b );
        b += 4;
      }
    }
  }
  else 
  {
    rc = WriteByte( count<<2, p );
  }
  return rc;
}

bool
ON_BinaryArchive::ReadInt64( // Read an array of 64 bit integers
		size_t count,            // number of 64 bit integers to read
		ON__INT64* p
		)
{
  bool rc = ReadByte( count<<3, p );
  if ( rc && m_endian == ON::big_endian ) 
  {
		unsigned char* b=(unsigned char*)p;
		unsigned char  c;
		while(count--) {
			c = b[0]; b[0] = b[7]; b[7] = c;
			c = b[1]; b[1] = b[6]; b[6] = c;
			c = b[2]; b[2] = b[5]; b[5] = c;
			c = b[3]; b[3] = b[4]; b[4] = c;
			b += 8;
		}
  }
  return rc;
}

bool
ON_BinaryArchive::WriteInt64( // Write an array of 64 bit integers
		size_t count,	            // number of ints to write
		const ON__INT64* p    
		)
{
  bool rc = true;
  if ( m_endian == ON::big_endian ) 
  {
    if ( count > 0 ) 
    {
      const char* b = (const char*)p;
      while ( rc && count-- ) 
      {
        rc = WriteByte( 1, b+7 );
        if (rc) rc = WriteByte( 1, b+6 );
        if (rc) rc = WriteByte( 1, b+5 );
        if (rc) rc = WriteByte( 1, b+4 );
        if (rc) rc = WriteByte( 1, b+3 );
        if (rc) rc = WriteByte( 1, b+2 );
        if (rc) rc = WriteByte( 1, b+1 );
        if (rc) rc = WriteByte( 1, b );
        b += 8;
      }
    }
  }
  else 
  {
    rc = WriteByte( count<<3, p );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteInt( // Write an array of integers
		size_t count,	          // number of ints to write
		const int* p    
		)
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 4 == sizeof(*p).  Since this code has to run on machines
// where sizeof(*p) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  bool rc;
  if ( 4 == sizeof(*p) )
  {
    rc = WriteInt32( count, (const ON__INT32*)p );
  }
  else
  {
    ON__INT32 i32;
    size_t j;
    rc = true;
    for ( j = 0; j < count && rc; j++ )
    {
      i32 = (ON__INT32)(*p++);
      rc = WriteInt32( 1, &i32 );
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::WriteSize(size_t sz)
{
  unsigned int u = (unsigned int)sz;
  return WriteInt(u);
}

bool
ON_BinaryArchive::ReadSize(size_t* sz)
{
  unsigned int u = 0;
  bool rc = ReadInt(&u);
  if (rc)
    *sz = u;
  return rc;
}

bool ON_BinaryArchive::WriteBigSize(size_t sz)
{
  ON__UINT64 u = (ON__UINT64)sz;
  return WriteInt64(1,(ON__INT64*)&u);;
}

bool ON_BinaryArchive::ReadBigSize( size_t* sz )
{
  ON__UINT64 u;
  bool rc = ReadInt64(1,(ON__INT64*)&u);
  if (rc)
    *sz = (size_t)u;
  return rc;
}

bool ON_BinaryArchive::WriteBigTime(time_t t)
{
  ON__UINT64 u = (ON__UINT64)t;
  return WriteInt64(1,(ON__INT64*)&u);
}

bool ON_BinaryArchive::ReadBigTime( time_t* t )
{
  ON__UINT64 u;
  bool rc = ReadInt64(1,(ON__INT64*)&u);
  if (rc)
    *t = (time_t)u;
  return rc;
}


bool
ON_BinaryArchive::WriteInt( // Write an array of 32 bit integers
		size_t count,	      // number of ints to write
		const unsigned int* p
		)
{
  return WriteInt( count, (const int*)p );
}

bool
ON_BinaryArchive::WriteInt( // Write a single 32 bit integer
		int i
		)
{
  return WriteInt( 1, &i );
}

bool
ON_BinaryArchive::WriteInt( // Write a single 32 bit integer
		unsigned int i
		)
{
  return WriteInt( 1, &i );
}

bool ON_BinaryArchive::WriteBigInt( // Write an array of 64 bit integers
		size_t count,
		const ON__INT64* p      
		)
{
  return WriteInt64(count,p);
}

bool ON_BinaryArchive::WriteBigInt( // Write an array of 64 bit integers
		size_t count,
		const ON__UINT64* p     
		)
{
  return WriteInt64(count,(const ON__INT64*)p);
}

bool ON_BinaryArchive:: WriteBigInt( // Write a single 64 bit integer
		ON__INT64 i
		)
{
  return WriteInt64(1,&i);
}

bool ON_BinaryArchive::WriteBigInt( // Write a single 64 bit unsigned integer
		ON__UINT64 u
		)
{
  return WriteInt64(1,(const ON__INT64*)&u);
}



bool
ON_BinaryArchive::WriteLong( // Write an array of longs
		size_t count,	      // number of longs to write
		const long* p    
		)
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 4 == sizeof(*p).  Since this code has to run on machines
// where sizeof(*p) can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  bool rc;
  if ( 4 == sizeof(*p) )
  {
    rc = WriteInt32( count, (const ON__INT32*)p );
  }
  else
  {
    ON__INT32 i32;
    size_t j;
    rc = true;
    for ( j = 0; j < count && rc; j++ )
    {
      i32 = (ON__INT32)(*p++);
      rc = WriteInt32( 1, &i32 );
    }
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::WriteLong( // Write an array of longs
		size_t count,	      // number of longs to write
		const unsigned long* p
		)
{
  return WriteLong( count, (const long*)p );
}

bool
ON_BinaryArchive::WriteLong( // Write a single long
		long i
		)
{
  return WriteLong( 1, &i );
}

bool
ON_BinaryArchive::WriteLong( // Write a single unsigned long
		unsigned long i
		)
{
  return WriteLong( 1, &i );
}


bool
ON_BinaryArchive::WriteFloat(   // Write a number of IEEE floats
		size_t count,       // number of doubles
		const float* p
		)
{
  // floats and integers have same size and endian issues
  return WriteInt( count, (const int*)p );
}

bool
ON_BinaryArchive::WriteFloat(   // Write a single float
		float f
		)
{
  return WriteFloat( 1, &f );
}

bool
ON_BinaryArchive::WriteDouble(  // Write a single double
		size_t count,       // number of doubles
		const double* p
		)
{
  bool rc = true;
  if ( m_endian == ON::big_endian ) {
    if ( count > 0 ) {
      const char* b = (const char*)p;
      while ( rc && count-- ) {
        rc = WriteByte( 1, b+7 );
        if (rc) rc = WriteByte( 1, b+6 );
        if (rc) rc = WriteByte( 1, b+5 );
        if (rc) rc = WriteByte( 1, b+4 );
        if (rc) rc = WriteByte( 1, b+3 );
        if (rc) rc = WriteByte( 1, b+2 );
        if (rc) rc = WriteByte( 1, b+1 );
        if (rc) rc = WriteByte( 1, b );
        b += 8;
      }
    }
  }
  else {
    rc = WriteByte( count<<3, p );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteComponentIndex(
		const ON_COMPONENT_INDEX& ci
		)
{
  bool rc = WriteInt( ci.m_type );
  if (rc)
    rc = WriteInt( ci.m_index );
  // do not add additional writes - you will break old file IO
  return rc;
}

bool
ON_BinaryArchive::ReadComponentIndex(
		ON_COMPONENT_INDEX& ci
		)
{
  int t;
  ci.m_type = ON_COMPONENT_INDEX::invalid_type;
  ci.m_index = 0;
  bool rc = ReadInt( &t );
  if (rc)
  {
    rc = ReadInt( &ci.m_index );
    if (rc)
    {
      ci.m_type = ON_COMPONENT_INDEX::Type(t);
    }
    // do not add additional read - you will break old file IO
  }
  return rc;
}

bool
ON_BinaryArchive::WriteDouble(  // Write a single double
		const double x
		)
{
  return WriteDouble( 1, &x );
}

bool
ON_BinaryArchive::WriteColor( const ON_Color& color )
{
  unsigned int colorref = color;
  return WriteInt( colorref );
}

bool
ON_BinaryArchive::WritePoint (
  const ON_2dPoint& p
  )
{
  return WriteDouble( 2, &p.x );
}

bool
ON_BinaryArchive::WritePoint (
  const ON_3dPoint& p
  )
{
  return WriteDouble( 3, &p.x );
}

bool
ON_BinaryArchive::WritePoint (
  const ON_4dPoint& p
  )
{
  return WriteDouble( 4, &p.x );
}

bool
ON_BinaryArchive::WriteVector (
  const ON_2dVector& v
  )
{
  return WriteDouble( 2, &v.x );
}

bool
ON_BinaryArchive::WriteVector (
  const ON_3dVector& v
  )
{
  return WriteDouble( 3, &v.x );
}

bool ON_BinaryArchive::WriteDisplayMaterialRef( const ON_DisplayMaterialRef& dmr )
{
  bool rc = WriteUuid( dmr.m_viewport_id );
  if (rc) rc = WriteUuid( dmr.m_display_material_id );
  return rc;
}

bool
ON_BinaryArchive::WriteUuid( const ON_UUID& uuid )
{
  bool    rc = WriteInt32( 1, (const ON__INT32*)(&uuid.Data1) );
  if (rc) rc = WriteInt16( 1, (const ON__INT16*)(&uuid.Data2) );
  if (rc) rc = WriteInt16( 1, (const ON__INT16*)(&uuid.Data3) );
  if (rc) rc = WriteByte( 8, uuid.Data4 );
  return rc;
}

bool
ON_BinaryArchive::WriteTime( const struct tm& utc )
{
  // utc = coordinated universal time ( a.k.a GMT, UTC )
  // (From ANSI C time() and gmtime().)
  // The checks are here so we can insure files don't contain
  // garbage dates and ReadTime() can treat out of bounds
  // values as file corruption errors.
  int i;
  i = (int)utc.tm_sec;  if ( i < 0 || i > 60 ) i = 0;
  bool rc = WriteInt( i );
  i = (int)utc.tm_min;  if ( i < 0 || i > 60 ) i = 0;
  if ( rc )
    rc = WriteInt( i );
  i = (int)utc.tm_hour;  if ( i < 0 || i > 24 ) i = 0;
  if ( rc )
    rc = WriteInt( i );
  i = (int)utc.tm_mday;  if ( i < 0 || i > 31 ) i = 0;
  if ( rc )
    rc = WriteInt( i );
  i = (int)utc.tm_mon;  if ( i < 0 || i > 12 ) i = 0;
  if ( rc )
    rc = WriteInt( i );

  // no year restrictions because dates are used in archeological userdata
  i = (int)utc.tm_year;
  if ( rc )
    rc = WriteInt( i );

  i = (int)utc.tm_wday;  if ( i < 0 || i > 7 ) i = 0;
  if ( rc )
    rc = WriteInt( i );
  i = (int)utc.tm_yday;  if ( i < 0 || i > 366 ) i = 0;
  if ( rc )
    rc = WriteInt( i );
  return rc;
}

bool
ON_BinaryArchive::WriteString( // Write NULL terminated UTF-8 encoded unicode string
    const char* sUTF8
    )
{
  size_t string_utf8_element_count = 0;
  if ( sUTF8 )
  {
    while(sUTF8[string_utf8_element_count])
      string_utf8_element_count++;
    if ( string_utf8_element_count )
      string_utf8_element_count++;
  }
  ON__UINT32 ui32 = (ON__UINT32)string_utf8_element_count;
  bool rc = WriteInt32(1,(ON__INT32*)&ui32);
  if ( rc && string_utf8_element_count > 0 )
    rc = WriteByte( string_utf8_element_count, sUTF8 );
  return rc;  
}

bool
ON_BinaryArchive::WriteString( // Write NULL terminated UTF-8 encoded unicode string
    const unsigned char* sUTF8
    )
{
  return WriteString( (const char*)sUTF8 );
}

bool
ON_BinaryArchive::WriteString(  // Write NULL terminated UTF-16 encoded unicode string
    const unsigned short* sUTF16
    )
{
  size_t string_utf16_element_count = 0;
  if ( sUTF16 )
  {
    while(sUTF16[string_utf16_element_count])
      string_utf16_element_count++;
    if ( string_utf16_element_count )
      string_utf16_element_count++;
  }
  ON__UINT32 ui32 = (ON__UINT32)string_utf16_element_count;
  bool rc = WriteInt32(1,(ON__INT32*)&ui32);
  if ( rc && string_utf16_element_count > 0 )
  {
    rc = WriteShort( string_utf16_element_count, sUTF16 );
  }
  return rc;  
}

bool
ON_BinaryArchive::WriteString( const ON_String& sUTF8 )
{
  size_t string_utf8_element_count = sUTF8.Length();
  if ( string_utf8_element_count )
    string_utf8_element_count++;
  ON__UINT32 ui32 = (ON__UINT32)string_utf8_element_count;
  bool rc = WriteInt32(1,(ON__INT32*)&ui32);
  if ( rc && string_utf8_element_count > 0 )
    rc = WriteByte( string_utf8_element_count, sUTF8.Array() );
  return rc;  
}

bool
ON_BinaryArchive::WriteString( const ON_wString& s )
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about 2 == sizeof(wchar_t).  Since this code has to run on machines
// where sizeof(wchar_t) can be 2, 4, or ... bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  size_t string_element_count = s.Length();
  if ( string_element_count > 0)
    string_element_count++;
  bool rc = false;
  if ( string_element_count <= 1 )
  {
    ON__UINT32 ui32 = 0;
    rc = WriteInt32(1,(ON__INT32*)&ui32);
  }
  else if ( 2 == sizeof(wchar_t) && string_element_count > 0 ) 
  {
    ON__UINT32 ui32 = (ON__UINT32)string_element_count;
    rc = WriteInt32(1,(ON__INT32*)&ui32);
    if (rc)
      rc = WriteInt16( string_element_count, (const ON__INT16*)s.Array() );
  }
  else if ( 4 == sizeof(wchar_t) && string_element_count > 0 )
  {
    // Assume the string is UTF-32 encoded (this is the case for some gcc implementations).
    const int bTestByteOrder = false;
    const ON__UINT32* sUTF32 = (const ON__UINT32*)s.Array();
    const int sUTF32_count = (int)(string_element_count-1);
    const unsigned int error_mask = 0xFFFFFFFF;
    const ON__UINT32 error_code_point = 0xFFFD; 
    unsigned int error_status = 0;

    const int sUTF16_count = ON_ConvertUTF32ToUTF16(
      bTestByteOrder,
      sUTF32,
      sUTF32_count,
      0, // ON__UINT16* sUTF16,
      0, // int sUTF16_count,
      &error_status,
      error_mask,
      error_code_point,
      0 // const ON__UINT32** sNextUTF32
      );

    if ( sUTF16_count > 0 )
    {
      error_status = 0;
      ON_SimpleArray<ON__UINT16> utf16_buffer(sUTF16_count+1);
      utf16_buffer.SetCount(sUTF16_count+1);
      const int sUTF16_count1 = ON_ConvertUTF32ToUTF16(
        bTestByteOrder,
        sUTF32,
        sUTF32_count,
        utf16_buffer.Array(),
        utf16_buffer.Count(),
        &error_status,
        error_mask,
        error_code_point,
        0 // const ON__UINT32** sNextUTF32
        );
      if ( sUTF16_count1 == sUTF16_count )
      {
        utf16_buffer[sUTF16_count] = 0;
        const ON__UINT32 ui32 = (ON__UINT32)(sUTF16_count+1);
        rc = WriteInt32(1,(const ON__INT32*)&ui32);
        if ( rc && ui32 > 0 )
          rc = WriteInt16( ui32, (const ON__INT16*)utf16_buffer.Array() );
      }
    }
  }
  return rc;  

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool ON_BinaryArchive::WriteArray( const ON_SimpleArray<bool>& a )
{
#if defined(ON_COMPILER_MSC)
#pragma warning( push )
// Disable the MSC /W4 "conditional expression is constant" warning
// about sizeof(*c) == sizeof(*b).  Since this code has to run on machines
// where sizeof(bool) can be 1, 2, 4, or 8 bytes, the test is necessary.
#pragma warning( disable : 4127 )
#endif

  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );

  if ( rc && count > 0 ) 
  {
    char* p = 0;
    const char* c = 0;
    const bool* b = a.Array();
    if ( sizeof(*c) == sizeof(*b) )
    {
      // 8 bit "bool" on this compiler
      c = (char*)(b);
    }
    else if ( b )
    {
      // bigger "bool" on this compiler
      p = (char*)onmalloc(count*sizeof(*p));
      int i;
      for ( i = 0; i < count; i++ )
        p[i] = (b[i]?1:0);
      c = p;
    }
    rc = WriteChar( count, c );
    if ( p )
      onfree(p);
  }

  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<char>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteChar( count, a.Array() );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<short>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteShort( count, a.Array() );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<int>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteInt( count, a.Array() );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<float>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteFloat( count, a.Array() );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<double>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count, a.Array() );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_Color>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteInt( count, (int*)a.Array() );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_2dPoint>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count*2, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_3dPoint>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count*3, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_4dPoint>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count*4, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_2dVector>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count*2, &a.Array()->x );
  }
  return rc;
}


bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_3dVector>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count*3, &a.Array()->x );
  }
  return rc;
}


bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_Xform>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) 
  {
    int i;
    for ( i = 0; i < count && rc; i++ )
      rc = WriteXform(a[i]);
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_2fPoint>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteFloat( count*2, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_3fPoint>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteFloat( count*3, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_4fPoint>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteFloat( count*4, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_2fVector>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteFloat( count*2, &a.Array()->x );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_3fVector>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteFloat( count*3, &a.Array()->x );
  }
  return rc;
}


bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_SurfaceCurvature>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  if ( rc && count > 0 ) {
    rc = WriteDouble( count*2, &a.Array()->k1 );
  }
  return rc;
}

bool
ON_BinaryArchive::WriteArray( const ON_SimpleArray<ON_ClippingPlaneInfo>& a )
{
  int count = a.Count();
  if ( count < 0 )
    count = 0;
  bool rc = WriteInt( count );
  for ( int i = 0; i < count && rc ; i++ )
  {
    rc = a[i].Write(*this);
  }
  return rc;
}


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

bool
ON_BinaryArchive::WriteObject( const ON_Object* o )
{
  bool rc = false;
  if ( o )
    rc = WriteObject(*o);
  else {
    // NULL object has nil UUID and no date
    rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS, 0 );
    if (rc) {
      rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_UUID, 0 );
      if ( rc ) {
        rc = WriteUuid( ON_nil_uuid );
        if ( !EndWrite3dmChunk() ) // end of TCODE_OPENNURBS_CLASS_UUID chunk
          rc = false;
      }
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

static
bool IsCriticalUserData( const ON_BinaryArchive& ar, const ON_UserData* ud )
{
  // {31F55AA3-71FB-49f5-A975-757584D937FF}
  static const ON_UUID ON_MeshNgonUserData_ID = 
  { 0x31F55AA3, 0x71FB, 0x49f5, { 0xA9, 0x75, 0x75, 0x75, 0x84, 0xD9, 0x37, 0xFF } };

  // Userdata that must be saved even when userdata saving is "off".
  // Please discuss any changes with Dale Lear.  In particular,
  // "critical" is used in a narrow sense and modifying this 
  // function to save in form of plug-in's user data saved is
  // not appropriate.  The definition of the user data class must
  // be in the opennurbs library and its purpose must be to extend
  // a core opennurbs data structure, usually because it was the
  // only way to add information to core opennurbs data structure
  // and not break the public C++ SDK.
  if ( 0 == ud )
    return false;

  switch( ar.Archive3dmVersion() )
  {
  case 50:
    if ( ON_opennurbs5_id == ud->m_application_uuid )
    {
      // As of 30 August 2012, all the core opennurbs user data
      // classes with an application id of ON_opennurbs5_id are
      // deemed "critical".
      return true;
    }

    if ( ON_opennurbs4_id == ud->m_application_uuid )
    {
      // ON_MeshNgonUserData is the only "critical" core 
      // opennurbs user data with an application id of 
      // ON_opennurbs4_id.
      if ( ON_MeshNgonUserData_ID == ud->m_userdata_uuid )
        return true;
    }
    break;

  case 4:
    if ( ON_opennurbs4_id == ud->m_application_uuid )
    {
      // ON_MeshNgonUserData is the only "critical" core 
      // opennurbs user data with an application id of 
      // ON_opennurbs4_id.
      if ( ON_MeshNgonUserData_ID == ud->m_userdata_uuid )
        return true;
    }
    break;
  }

  return false;
}

static
bool HasCriticalUserData( const ON_BinaryArchive& ar, const ON_Object* obj )
{
  if ( 0 == obj )
    return false;

  for ( const ON_UserData* ud = obj->FirstUserData(); 0 != ud; ud = ud->Next() )
  {
    if ( IsCriticalUserData(ar,ud) )
      return true;
  }

  return false;
}

static
bool IsCoreUserData( const ON_UserData* ud )
{
  // Userdata with IO code we trust.
  if ( 0 == ud )
    return false;
  if (    ud->m_application_uuid == ON_rhino4_id
       || ud->m_application_uuid == ON_rhino5_id
       || ud->m_application_uuid == ON_rhino_id
       || ud->m_application_uuid == ON_opennurbs4_id
       || ud->m_application_uuid == ON_opennurbs5_id
       || ud->m_application_uuid == ON_opennurbs_id
      )
  {
    return true;
  }
  return false;
}

bool
ON_BinaryArchive::WriteObject( const ON_Object& o )
{
  // writes polymorphic object derived from ON_Object in a way that
  // it can be recreated from ON_BinaryArchive::ReadObject
  ON_UUID uuid;
  bool rc = false;
  const ON_ClassId* pID = o.ClassId();
  if ( !pID ) {
    ON_ERROR("ON_BinaryArchive::WriteObject() o.ClassId() returned NULL.");
    return false;
  }
  uuid = pID->Uuid();

  if ( m_3dm_version <= 2 )
  {
    if ( ON_Curve::Cast(&o) && !ON_NurbsCurve::Cast(&o) )
    {
      ON_NurbsCurve nc;
      const ON_Curve* curve = static_cast<const ON_Curve*>(&o);
      if ( curve->GetNurbForm(nc) )
        return WriteObject( nc );
    }
    else if ( ON_Surface::Cast(&o) && !ON_NurbsSurface::Cast(&o) )
    {
      ON_NurbsSurface ns;
      const ON_Surface* surface = static_cast<const ON_Surface*>(&o);
      if ( surface->GetNurbForm(ns) )
        return WriteObject( ns );
    }
    else if( ON_Annotation2::Cast(&o))
    {
      // 6-23-03 lw added for writing annotation to v2 files
      ON_Annotation2* pA = (ON_Annotation2*)&o;
      switch( pA->Type())
      {
      case ON::dtNothing:
        break;
      case ON::dtDimLinear:
      case ON::dtDimAligned:
        {
          ON_LinearDimension dim;
          ((ON_LinearDimension2*)pA)->GetV2Form( dim);
          return WriteObject( dim);
        }
      case ON::dtDimAngular:
        {
          ON_AngularDimension dim;
          ((ON_AngularDimension2*)pA)->GetV2Form( dim);
          return WriteObject( dim);
        }
      case ON::dtDimDiameter:
      case ON::dtDimRadius:
        {
          ON_RadialDimension dim;
          ((ON_RadialDimension2*)pA)->GetV2Form( dim);
          return WriteObject( dim);
        }
      case ON::dtLeader:
        {
          ON_Leader leader;
          ((ON_Leader2*)pA)->GetV2Form( leader);
          return WriteObject( leader);
        }
      case ON::dtTextBlock:
        {
          ON_TextEntity text;
          ((ON_TextEntity2*)pA)->GetV2Form( text);
          return WriteObject( text);
        }
      case ON::dtDimOrdinate:
        // no V2 form of ordinate dimensions
        //  Fall through and save it in v4 format.
        //  It will be skipped by the v2 and v3 readers
        //  but users won't loose the ordinate dimensions
        //  when they encounter the situation described in
        //  the next comment.
        break;
      }

      // 7 August 2003 Dale Lear
      //    I commented out the "return false;".
      //    It is imporant to fall through to the working
      //    code because people overwrite their "good" files
      //    with a V2 version and loose everything when the
      //    writing code returns false.  By falling through,
      //    V2 will still read what it can from the file and
      //    V3 will read the entire file.
      //return false;
    }
  }

  if ( m_3dm_version <= 4 && ON::extrusion_object == o.ObjectType() )
  {
    // 29 September 2010 Dale Lear
    //   ON_Extrusion was added in V5.  It must be translated
    //   to a brep or surface to save it in a V4 file.
    const ON_Extrusion* extrusion = ON_Extrusion::Cast(&o);
    if ( 0 != extrusion )
    {
      ON_Object* v4object = 0;
      if ( extrusion->IsCapped() || extrusion->ProfileCount() >= 2 )
        v4object = extrusion->BrepForm(0);
      if ( 0 == v4object )
        v4object = extrusion->SumSurfaceForm(0);
      if ( 0 == v4object )
        v4object = extrusion->NurbsSurface(0);
      if ( 0 != v4object 
           && ON::extrusion_object != v4object->ObjectType() // no infinte recursion!
         )
      {
        // Some plug-in userdata code is not robust enough to check 
        // archive version numbers and correctly handle save as v4 
        // changes.  The upshot is that, extrusion user data
        // has to be lost when saving as V4.
        //v4object->MoveUserData(const_cast<ON_Object&>(o));
        rc = WriteObject( v4object );
        //const_cast<ON_Object&>(o).MoveUserData(*v4object);
        delete v4object;
        return rc;
      }
      if ( 0 != v4object )
        delete v4object;
    }
  }

  rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS, 0 );
  if (rc) {

    // TCODE_OPENNURBS_CLASS_UUID chunk contains class's UUID 
    rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_UUID, 0 );
    if ( rc ) {
      rc = WriteUuid( uuid );
      if ( !EndWrite3dmChunk() ) // end of TCODE_OPENNURBS_CLASS_UUID chunk
        rc = false;
    }

    // TCODE_OPENNURBS_CLASS_DATA chunk contains definition of class
    if ( rc ) {
      rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_DATA, 0 );
      if (rc)
      {
        rc = o.Write( *this )?true:false;
        if ( !rc ) {
          ON_ERROR("ON_BinaryArchive::WriteObject() o.Write() failed.");
        }
        if ( !EndWrite3dmChunk() ) // end of TCODE_OPENNURBS_CLASS_DATA chunk
          rc = false;
      }
    }

    if (rc && (m_bSaveUserData || HasCriticalUserData(*this,&o) ) ) 
    {
      // write user data.  Each piece of user data is in a 
      // TCODE_OPENNURBS_CLASS_USERDATA chunk.
      rc = WriteObjectUserData(o);
    }

    // TCODE_OPENNURBS_CLASS_END chunk marks end of class record
    if ( BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_END, 0 ) ) {
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
    else
      rc = false;

    if ( !EndWrite3dmChunk() ) // end of TCODE_OPENNURBS_CLASS chunk
      rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::WriteObjectUserData( const ON_Object& object )
{
  if ( m_3dm_version < 3 )
  {
    // no user data is saved in V1 and V2 files.
    return true;
  }

  // writes user data attached to object.
  bool rc = true;
  const ON_UserData* ud;
  ON_UUID userdata_classid;
  
  for (ud = object.m_userdata_list; ud && rc; ud = ud->m_userdata_next) 
  {
    if ( !ud->Archive() )
    {
      continue;
    }

    if ( false == m_bSaveUserData )
    {
      // user data is not being saved unless it is functioning
      // as a critcial extension of a core opennurbs data
      // structure.
      if ( false == IsCriticalUserData(*this,ud) )
        continue;
    }


    // LOTS of tests to weed out bogus user data
    if ( 0 == ON_UuidCompare( ud->m_userdata_uuid, ON_nil_uuid ) )
      continue;
    if ( &object != ud->m_userdata_owner )
      continue;
    const ON_ClassId* cid = ud->ClassId();
    if ( 0 == cid )
      continue;
    if ( cid == &ON_UserData::m_ON_UserData_class_id )
      continue;
    if ( cid == &ON_UserData::m_ON_Object_class_id )
      continue;

    // The UserDataClassUuid() function is used instead of 
    // calling cid->Uuid() so we get the value of the 
    // plug-in's class id when the plug-in is not loaded
    // and ud is ON_UnknownUserData.
    userdata_classid = ud->UserDataClassUuid();
    if ( 0 == ON_UuidCompare( userdata_classid, ON_nil_uuid ) )
      continue;
    if ( 0 == ON_UuidCompare( userdata_classid, ON_UserData::m_ON_UserData_class_id.Uuid() ) )
      continue;
    if ( 0 == ON_UuidCompare( userdata_classid, ON_Object::m_ON_Object_class_id.Uuid() ) )
      continue;
    if ( 0 == ON_UuidCompare( userdata_classid, ON_UnknownUserData::m_ON_Object_class_id.Uuid() ) )
      continue;

    if ( 3 == m_3dm_version )
    {
      // When saving a V3 archive and the user data is not
      // native V3 data, make sure the plug-in supports
      // writing V3 user data.
      if ( m_V3_plugin_id_list.BinarySearch( &ud->m_application_uuid, ON_UuidCompare ) < 0 )
        continue;
    }

    if ( ON_UuidIsNil( ud->m_application_uuid ) )
    {
      // As of version 200909190 - a non-nil application_uuid is 
      // required in order for user data to be saved in a 
      // 3dm archive.
      ON_Error(__FILE__,__LINE__,"Not saving %s userdata - m_application_uuid is nil.",cid->ClassName());
      continue; 
    }

    // See if we have unknown user data (goo) and make sure
    // IsUnknownUserData() agrees with ON_UnknownUserData::Cast().
    const ON_UnknownUserData* unknown_ud = ON_UnknownUserData::Cast(ud);
    if ( 0 == unknown_ud )
    {
      if ( ud->IsUnknownUserData() )
      {
        ON_ERROR("ON_UnknownUserData::Cast(ud) is null and ud->IsUnknownUserData() is true.");
        continue; // something's wrong
      }
    }
    else
    {
      if ( !ud->IsUnknownUserData() )
      {
        ON_ERROR("ON_UnknownUserData::Cast(ud) is not null and ud->IsUnknownUserData() is false.");
        continue; // something's wrong
      }
    }

    if ( 0 != unknown_ud )
    {
      if ( false == m_bSaveUserData )
        continue; // "unknown" user data cannot be "critical"

      if ( unknown_ud->m_3dm_version <= 3 )
        continue; // Unknown will not be resaved in V3 archives

      if ( unknown_ud->m_3dm_version > 5 && unknown_ud->m_3dm_version < 50 )
        continue;

      if ( unknown_ud->m_3dm_opennurbs_version < 200701010 )
        continue;

      if ( unknown_ud->m_3dm_version >= 50 && m_3dm_version < 50 )
      {
        // Unknown userdata with 8 byte chunk lengths cannot be
        // saved into a V4 file with 4 byte chunk lengths because
        // the resulting chunk will be unreadable in V4.
        // This is not an error condition.  It is a consequence
        // of V4 IO code not being robust enough to handle
        // 8 bytes chunk lengths.
        continue;
      }
    }

    // Each piece of user data is inside of 
    // a TCODE_OPENNURBS_CLASS_USERDATA chunk.
    rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_USERDATA, 0 );
    if (rc) {
      rc = Write3dmChunkVersion(2,2);
      // wrap user data header info in an TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk
      rc = BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_USERDATA_HEADER, 0 );
      if (rc) 
      {
        if ( rc ) rc = WriteUuid( userdata_classid );
        if ( rc ) rc = WriteUuid( ud->m_userdata_uuid );
        if ( rc ) rc = WriteInt( ud->m_userdata_copycount );
        if ( rc ) rc = WriteXform( ud->m_userdata_xform );

        // added for version 2.1
        if ( rc ) rc = WriteUuid( ud->m_application_uuid );

        // added for version 2.2 - 14, October 2009
        if ( rc )
        {
          rc = WriteBool( unknown_ud ? true : false );
          int ver = unknown_ud ? unknown_ud->m_3dm_version : m_3dm_version;
          rc = WriteInt( ver );
          ver = unknown_ud ? unknown_ud->m_3dm_opennurbs_version : m_3dm_opennurbs_version;
          if (rc) rc = WriteInt( ver );
        }

        if ( !EndWrite3dmChunk() )
          rc = false;
      }
      if (rc) 
      {
        // wrap user data in an anonymous chunk
        rc = BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 0 );
        if ( rc ) 
        {
          if ( 0 != unknown_ud )
          {
            // 22 January 2004 Dale Lear
            //   Disable crc checking when writing the
            //   unknow user data block.
            //   This has to be done so we don't get an extra
            //   32 bit CRC calculated on the block that
            //   ON_UnknownUserData::Write() writes.  The
            //   original 32 bit crc is at the end of this
            //   block and will be checked when the class
            //   that wrote this user data is present.
            //   The EndWrite3dmChunk() will reset the
            //   CRC checking flags to the appropriate
            //   values.
            m_chunk.Last()->m_do_crc16 = 0;
            m_chunk.Last()->m_do_crc32 = 0;
            m_bDoChunkCRC = false;
          }
          rc = ud->Write(*this)?true:false;
          if ( !EndWrite3dmChunk() )
            rc = false;
        }
      }
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

int
ON_BinaryArchive::LoadUserDataApplication( ON_UUID application_id )
{
  // This is a virtual function.
  // Rhino overrides this function to load plug-ins.
  return 0;
}

int ON_BinaryArchive::ReadObject( ON_Object** ppObject )
{
  if ( !ppObject ) 
  {
    ON_ERROR("ON_BinaryArchive::ReadObject() called with NULL ppObject.");
    return 0;
  }
  *ppObject = 0;
  return ReadObjectHelper(ppObject);
}

int ON_BinaryArchive::ReadObject( ON_Object& object )
{
  ON_Object* pObject = &object;
  return ReadObjectHelper(&pObject);
}

int ON_BinaryArchive::ReadObjectHelper( ON_Object** ppObject )
{
  // returns 0: failure - unable to read object because of file IO problems
  //         1: success
  //         3: unable to read object because it's UUID is not registered
  //            this could happen in cases where old code is attempting to read
  //            new objects.
  ON__UINT32 tcode;
  ON__INT64 length_TCODE_OPENNURBS_CLASS = 0;
  ON__INT64 length_TCODE_OPENNURBS_CLASS_UUID = 0;
  ON__INT64 length_TCODE_OPENNURBS_CLASS_DATA = 0;
  ON_UUID uuid;
  const ON_ClassId* pID = 0;
  ON_Object* pObject = *ppObject; // If not null, use input
  int rc = 0;
  const ON__INT64 sizeof_chunk_header = (ON__INT64)(4 + SizeofChunkLength());
  const ON__INT64 expected_length_TCODE_OPENNURBS_CLASS_UUID = 20;

  //bool bBogusUserData = false;


  // all ON_Objects written by WriteObject are in a TCODE_OPENNURBS_CLASS chunk
  rc = BeginRead3dmBigChunk( &tcode, &length_TCODE_OPENNURBS_CLASS );
  if ( !rc )
    return 0;

  // When a NULL ON_Object is written, the file has
  // 
  //  TCODE_OPENNURBS_CLASS, length = 20 + sizeof_chunk_header
  //  TCODE_OPENNURBS_CLASS_UUID, length = 20
  //    16 byte nil uuid
  //    4 byte TCODE_OPENNURBS_CLASS_UUID crc
  //
  // When a non-NULL ON_Object is written, the file has
  // 
  //  TCODE_OPENNURBS_CLASS, length = 20 + 3*sizeof_chunk_header + length_DATA_chunk + length_USER_DATA_chunk(s)
  //
  //    TCODE_OPENNURBS_CLASS_UUID, length = 20
  //      16 byte nil uuid
  //      4 byte TCODE_OPENNURBS_CLASS_UUID crc
  //
  //    TCODE_OPENNURBS_CLASS_DATA, length_DATA >= 4
  //      ...
  //      4 byte TCODE_OPENNURBS_CLASS_DATA crc
  //
  //    Optional TCODE_OPENNURBS_CLASS_USERDATA chunks
  //
  //    TCODE_OPENNURBS_CLASS_END, chunk value = 0

  if ( tcode != TCODE_OPENNURBS_CLASS ) 
  {
    ON_ERROR("ON_BinaryArchive::ReadObject() didn't find TCODE_OPENNURBS_CLASS block.");
    rc = 0;
  }
  else if ( length_TCODE_OPENNURBS_CLASS < expected_length_TCODE_OPENNURBS_CLASS_UUID + sizeof_chunk_header) 
  {
    ON_ERROR("ON_BinaryArchive::ReadObject() TCODE_OPENNURBS_CLASS chunk length too small.");
    rc = 0;
  }
  else 
  {
    // we break out of this loop if something bad happens
    for (;;) 
    {
      // read class's UUID ///////////////////////////////////////////////////////////
      rc = BeginRead3dmBigChunk( &tcode, &length_TCODE_OPENNURBS_CLASS_UUID );
      if ( !rc )
        break;
      if ( tcode != TCODE_OPENNURBS_CLASS_UUID )
      {
        ON_ERROR("ON_BinaryArchive::ReadObject() didn't find TCODE_OPENNURBS_CLASS_UUID block");
        rc = 0;
      }
      else if ( expected_length_TCODE_OPENNURBS_CLASS_UUID != length_TCODE_OPENNURBS_CLASS_UUID )
      {
        ON_ERROR("ON_BinaryArchive::ReadObject() TCODE_OPENNURBS_CLASS_UUID has invalid length");
        rc = 0;
      }
      else if ( !ReadUuid( uuid ) )
      {
        rc = 0;
      }
      if ( !EndRead3dmChunk() ) 
      {
        rc = 0;
      }
      if ( !rc ) {
        break;
      }
      ///////////////////////////////////////////////////////////////////////////////

      if ( !ON_UuidCompare( &uuid, &ON_nil_uuid ) ) {
        // nil UUID written if NULL pointer is passed to WriteObject();
        rc = 1;
        break;
      }

      // Use UUID to get ON_ClassId for this class //////////////////////////////////
      if ( pObject )
      {
        pID = pObject->ClassId();
        if ( uuid != pID->Uuid() )
        {
          ON_ERROR("ON_BinaryArchive::ReadObject() - uuid does not match intput pObject's class id.");
          pID = 0;
          rc = 2;
          break;
        }
      }
      else
      {
        pID = ON_ClassId::ClassId( uuid );
      }
      if ( !pID ) 
      {
        // If you get here and you are not calling ON::Begin() at some point in your
        // executable, then call ON::Begin() to force all class definition to be linked.
        // If you are callig ON::Begin(), then either the uuid is garbage or you are 
        // attempting to read an object with old code.
        // Visit http://www.opennurbs.org to get the latest OpenNURBS code.
        ON_WARNING("ON_BinaryArchive::ReadObject() ON_ClassId::ClassId(uuid) returned NULL.");
        rc = 3;
        break;
      }
      ///////////////////////////////////////////////////////////////////////////////

      // read class's definitions   /////////////////////////////////////////////////
      rc = BeginRead3dmBigChunk( &tcode, &length_TCODE_OPENNURBS_CLASS_DATA );
      if ( !rc )
        break;
      if ( tcode != TCODE_OPENNURBS_CLASS_DATA )
      {
        ON_ERROR("ON_BinaryArchive::ReadObject() didn't find TCODE_OPENNURBS_CLASS_DATA block");
        rc = 0;
      }
      else if ( length_TCODE_OPENNURBS_CLASS_DATA <= 0  )
      {
        ON_ERROR("ON_BinaryArchive::ReadObject() TCODE_OPENNURBS_CLASS_DATA chunk length too small");
        rc = 0;
      }
      else 
      {
        if ( !pObject )
        {
          pObject = pID->Create();
        }

        if ( !pObject ) 
        {
          ON_ERROR("ON_BinaryArchive::ReadObject() pID->Create() returned NULL.");
          rc = 0;
        }
        else 
        {
          rc = pObject->Read(*this);
          if ( !rc ) 
          {
            ON_ERROR("ON_BinaryArchive::ReadObject() pObject->Read() failed.");
            delete pObject;
            // don't break here - we still need to call end chunk.
          }
          else 
          {
            *ppObject = pObject;
          }
        }
      }
      if ( !EndRead3dmChunk() )
      {
        rc = 0;
      }

      if ( rc && 0 != pObject )
      {
        // read user data  /////////////////////////////////////////////////
        //   If TCODE_OPENNURBS_CLASS_USERDATA chunks exist, this reads them.
        //   ReadObjectUserData() stops when it reads a TCODE_OPENNURBS_CLASS_END chunk.
        if (!ReadObjectUserData(*pObject))
          rc = 0;
      }

      break; 
    }
  }
  if ( !EndRead3dmChunk() ) // TCODE_OPENNURBS_CLASS
    rc = 0;

  return rc;
}

bool ON_BinaryArchive::ReadObjectUserDataAnonymousChunk( 
          const ON__UINT64 length_TCODE_ANONYMOUS_CHUNK,
          const int archive_3dm_version,
          const int archive_opennurbs_version,
          ON_UserData* ud )
{
  // Reads the portion of the file containing the userdata into a buffer
  // and lets the plug-in try to read from that.  If the plug-in fails,
  // we press on because we cannot trust plug-ins to get IO code right.
  bool rc = false;

  if ( 0 == ud )
    return false;

  if ( ud->IsUnknownUserData()
       || (archive_3dm_version == Archive3dmVersion() 
           && archive_opennurbs_version == ArchiveOpenNURBSVersion() 
           && IsCoreUserData(ud))
     )
  {
    // assume this userdata's read function is robust.
    ON_ReadChunkHelper ch(*this);
    if (    !ch.m_bReadSuccess
         || TCODE_ANONYMOUS_CHUNK != ch.m_chunk_tcode
         || length_TCODE_ANONYMOUS_CHUNK != (ON__UINT64)ch.m_chunk_value
       )
    {
      return false;
    }
    if ( ud->IsUnknownUserData() )
    {
      // 22 January 2004 Dale Lear:
      //   Disable CRC checking while reading this chunk.  
      //   (If the user data has nested chunks, the crc we get
      //   by reading the thing as one large chunk will be wrong.)
      m_chunk.Last()->m_do_crc16 = false;
      m_chunk.Last()->m_do_crc32 = false;
      m_bDoChunkCRC = false;
    }
    rc = ud->Read(*this) ? true : false;
  }
  else
  {
    // Untrusted plug-in userdata.
    // Insulate file reading from possible bugs plug-in IO code by reading
    // entire anonymous chunk into memory and letting the plug-in use
    // the memory buffer archive.
    unsigned char stack_buffer[2048];
    const size_t sizeof_buffer = (size_t)(length_TCODE_ANONYMOUS_CHUNK + 4 + SizeofChunkLength());
    void* freeme = 0;
    void* buffer = (sizeof_buffer <= sizeof(stack_buffer))
                 ? &stack_buffer[0]
                 : (freeme = onmalloc(sizeof_buffer)); // generally, object userdata is small we almost never use heap
    if ( 0 != buffer 
         && sizeof_buffer == ReadBuffer(sizeof_buffer,buffer)
       ) 
    {
      ON_Read3dmBufferArchive memory_archive(
        sizeof_buffer,
        buffer,
        false,
        archive_3dm_version,
        archive_opennurbs_version
        );

      // The TCODE_ANONYMOUS_CHUNK wrapper has chunk lengths set 
      // by whatever version wrote this file.  The information
      // in the chunk has chunk lengths set by the plug-in that
      // originally wrote the user data.  If the plug-in used
      // worte to a version <= 5 archive and the user data has
      // was read as goo and saved as goo in a version 50+
      // archive, then we need to tweak the archive version
      // when reading the chunk length of the TCODE_ANONYMOUS_CHUNK wrapper.
      bool bTweakArchiveVersion = (memory_archive.SizeofChunkLength() != SizeofChunkLength());
      if ( bTweakArchiveVersion )
        memory_archive.SetArchive3dmVersion(Archive3dmVersion());
      ON_ReadChunkHelper ch(memory_archive);
      if ( bTweakArchiveVersion )
        memory_archive.SetArchive3dmVersion(archive_3dm_version);

      if (    !ch.m_bReadSuccess
           || TCODE_ANONYMOUS_CHUNK != ch.m_chunk_tcode
           || length_TCODE_ANONYMOUS_CHUNK != (ON__UINT64)ch.m_chunk_value
         )
        rc = false;
      else
        rc = ud->Read(memory_archive) ? true : false;
    }
    if ( freeme )
      onfree(freeme);
  }
  return rc;
}


class CUserDataHeaderInfo
{
public:
  CUserDataHeaderInfo();

  void Initialize();

  ON_UUID m_classid;
  ON_UUID m_itemid;
  ON_UUID m_appid;
  int m_3dm_version;
  int m_3dm_opennurbs_version;
  int m_copycount;
  bool m_bLastSavedAsGoo;
  ON_Xform m_xform;
};

CUserDataHeaderInfo::CUserDataHeaderInfo()
{
  Initialize();
}

void CUserDataHeaderInfo::Initialize()
{
  memset(this,0,sizeof(*this));
}

static
bool ReadObjectUserDataHeaderHelper( 
          ON_BinaryArchive& binary_archive,
          const int major_userdata_version,
          const int minor_userdata_version,
          CUserDataHeaderInfo& ud_header
          )
{
  bool rc = true;
  ON__UINT32 t = 0;
  ON__INT64 length_TCODE_OPENNURBS_CLASS_USERDATA_HEADER = 0;

  ud_header.Initialize();

  if ( major_userdata_version == 2 )
  {
    // major_userdata_version 2 started wrapping the userdata header info 
    // in a TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk
    rc = binary_archive.BeginRead3dmBigChunk( &t, &length_TCODE_OPENNURBS_CLASS_USERDATA_HEADER );
    if (!rc)
      return false;
    if ( t != TCODE_OPENNURBS_CLASS_USERDATA_HEADER )
    {
      ON_ERROR("version 2.0 TCODE_OPENNURBS_CLASS_USERDATA chunk is missing TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk.");
      binary_archive.EndRead3dmChunk(); // end of mystery chunk
      return false;
    }
  }
  
  if (rc) rc = binary_archive.ReadUuid( ud_header.m_classid );
  if (rc) rc = binary_archive.ReadUuid( ud_header.m_itemid );
  if (rc) rc = binary_archive.ReadInt( &ud_header.m_copycount );
  if (rc) rc = binary_archive.ReadXform( ud_header.m_xform );
  if ( major_userdata_version == 2 ) 
  {
    if ( minor_userdata_version >= 1 )
    {
      if (rc) rc = binary_archive.ReadUuid( ud_header.m_appid );
      if ( minor_userdata_version >= 2 )
      {
        // bLastSavedAsGoo is true if the user data was saved
        // into the file by ON_UnknownUserData.
        if (rc) rc = binary_archive.ReadBool( &ud_header.m_bLastSavedAsGoo ); 
        if (rc) rc = binary_archive.ReadInt( &ud_header.m_3dm_version );
        if (rc) rc = binary_archive.ReadInt( &ud_header.m_3dm_opennurbs_version );
      }
    }
    if ( !binary_archive.EndRead3dmChunk() ) // end of TCODE_OPENNURBS_CLASS_USERDATA_HEADER
      rc = 0;
  }

  if (!rc)
  {
    ON_ERROR("Unable to read user data header information.");
    return false;
  }

  if ( 0 == ud_header.m_3dm_version || 0 == ud_header.m_3dm_opennurbs_version )
  {
    // The userdata was saved in in an archive before
    // the 3dm_version and 3dm_opennurbs_version were saved in
    // userdata headers.  This means it has to be userdata
    // with 4 byte chunk lengths.  If the archive we are
    // reading was written with a newer version of opennurbs
    // that does save the 3dm version info, then this unknown
    // userdata was that has persisted through multiple read/write 
    // cycles and we cannot tell it's original version. So we
    // will default to a maximum of 5 and 200910180 - the 
    // 3dm versions just before we started saving 3dm 
    // version info n userdata headers.
    if ( binary_archive.Archive3dmVersion() < 50 )
    {
     ud_header.m_3dm_version = binary_archive.Archive3dmVersion();
    }
    else
    {
      // All Archive3dmVersion() >= 50 have userdata_3dm_version info,
      // so this userdata had to be saved as goo from a version 5 or
      // earlier archive.
      ud_header.m_bLastSavedAsGoo = true;
      ud_header.m_3dm_version = 5;
    }
    ud_header.m_3dm_opennurbs_version = binary_archive.ArchiveOpenNURBSVersion();
    if ( ud_header.m_3dm_opennurbs_version >= 200910190 )
    {
      ud_header.m_3dm_opennurbs_version = 200910180;
      ud_header.m_bLastSavedAsGoo = true;
    }
  }

  return rc;
}

bool ON_BinaryArchive::ReadObjectUserData( ON_Object& object )
{
  bool rc = true;
  while(rc) 
  {
    ON_ReadChunkHelper ch(*this);
    if ( !ch.m_bReadSuccess )
    {
      rc = false;
      break;
    }

    if ( TCODE_OPENNURBS_CLASS_END == ch.m_chunk_tcode )
    {
      // A short TCODE_OPENNURBS_CLASS_END chunk marks the end of the opennurbs class
      break; // done
    }

    if ( TCODE_OPENNURBS_CLASS_USERDATA != ch.m_chunk_tcode )
    {
      // skip new chunk type added by later version
      continue;
    }

    if ( ch.m_chunk_value < (ON__INT64)(8 + 4 * SizeofChunkLength()) )
    {
      ON_ERROR("TCODE_OPENNURBS_CLASS_USERDATA chunk is too short");
      continue;
    }

    // Read userdata header information
    int major_userdata_version = 0;
    int minor_userdata_version = 0;
    rc = Read3dmChunkVersion( &major_userdata_version, &minor_userdata_version );
    if ( !rc )
    {
      ON_ERROR("Unable to read TCODE_OPENNURBS_CLASS_USERDATA chunk version numbers");
      break;
    }

    if ( major_userdata_version < 1 || major_userdata_version > 2 )
    {
      // unsupported version - too old or added in new version
      continue;
    }

    CUserDataHeaderInfo ud_header;
    rc = ReadObjectUserDataHeaderHelper(*this,major_userdata_version,minor_userdata_version,ud_header);
    if (!rc)
    {
      ON_ERROR("Unable to read user data header information.");
      break;
    }

    // we should be ready to read a TCODE_ANONYMOUS_CHUNK containing userdata
    ON__INT64 length_TCODE_ANONYMOUS_CHUNK = 0;
    for(;;)
    {
      ON__UINT32 t = 0;
      rc = PeekAt3dmBigChunkType( &t, &length_TCODE_ANONYMOUS_CHUNK );
      if (!rc)
        break;
      if ( t != TCODE_ANONYMOUS_CHUNK )
      {
        ON_ERROR("Reading object user data - unable to find TCODE_ANONYMOUS_CHUNK");
        rc = false;
        break;
      }
      if ( length_TCODE_ANONYMOUS_CHUNK < 4 )
      {
        ON_ERROR("Reading object user data - length of TCODE_ANONYMOUS_CHUNK < 4");
        rc = false;
        break;
      }
      break;
    }
    if (!rc)
    {
      break;
    }

    // attempt to get an instance of the userdata class that saved this information
    ON_UserData* ud = 0;
    for(;;)
    {
      const ON_ClassId* udId = ON_ClassId::ClassId( ud_header.m_classid );
      if ( 0 == udId ) 
      {
        // The application that created this userdata is not active
        if ( !ON_UuidIsNil(ud_header.m_appid) )
        {
          // see if we can load the application
          if ( 1 == LoadUserDataApplication(ud_header.m_appid) )
          {
            // try again
            udId = ON_ClassId::ClassId( ud_header.m_classid );
          }
        }

        if ( 0 == udId )
        {
          // The application that created this user data is
          // not available.  This information will be stored
          // in an ON_UnknownUserData class so that it can
          // persist.
          udId = &ON_UnknownUserData::m_ON_UnknownUserData_class_id;
        }
      }

      ON_Object* tmp = udId->Create();
      ud = ON_UserData::Cast(tmp);
      if ( 0 == ud )
      {
        ON_ERROR("Reading object user data - unable to create userdata class");
        if ( tmp )
          delete tmp;
        tmp = 0;
        break;
      }
      tmp = 0;

      break;
    }

    if ( 0 == ud )
    {
      // no luck on this one 
      // One reason can be that the plug-in userdata class has something wrong with 
      // its ON_OBJECT_DECLARE/ON_OBJECT_IMPLEMENT stuff.
      ON_ERROR("Unable to create object user data class. Flawed class id information.");
      continue; // keep trying
    }

    if ( ON_UuidIsNil(ud->m_application_uuid) )
    {
      if ( ON_UuidIsNil(ud_header.m_appid) )
      {
        switch( Archive3dmVersion())
        {
        case 2:
          // V2 archives do not contain app ids.
          // This id flags the userdata as being read from a V3 archive.
          ud_header.m_appid = ON_v2_userdata_id;
          break;
        case 3:
          // V3 archives do not contain app ids.
          // This id flags the userdata as being
          // read from a V3 archive.
          ud_header.m_appid = ON_v3_userdata_id;
          break;
        case 4:
          if ( ArchiveOpenNURBSVersion() < 200909190 )
          {
            // V4 archives before version 200909190
            // did not require user data application ids.
            ud_header.m_appid = ON_v4_userdata_id;
          }
          break;
        }
      }
      ud->m_application_uuid = ud_header.m_appid;
    }
    ud->m_userdata_uuid = ud_header.m_itemid;
    ud->m_userdata_copycount = ud_header.m_copycount;
    ud->m_userdata_xform = ud_header.m_xform;
    if ( ud->IsUnknownUserData() ) 
    {
      ON_UnknownUserData* uud = ON_UnknownUserData::Cast(ud);
      if ( uud ) 
      {
        uud->m_sizeof_buffer = (int)length_TCODE_ANONYMOUS_CHUNK;
        uud->m_unknownclass_uuid = ud_header.m_classid;
        uud->m_3dm_version = ud_header.m_3dm_version;
        uud->m_3dm_opennurbs_version = ud_header.m_3dm_opennurbs_version;
      }
    }
    ud->m_userdata_owner = &object; // so reading code can look at owner
    bool bReadUserData = ReadObjectUserDataAnonymousChunk(
              length_TCODE_ANONYMOUS_CHUNK,
              ud_header.m_3dm_version,
              ud_header.m_3dm_opennurbs_version,
              ud
              );
    ud->m_userdata_owner = 0;
    if (bReadUserData) 
    {
      if ( !object.AttachUserData( ud ) )
        delete ud;
    }
    else
    {
      delete ud;
    }    
  }

  return rc;
}

bool ON_BinaryArchive::Write3dmChunkVersion(
  int major_version, // major // 0 to 15
  int minor_version // minor // 0 to 16
  )
{
  const unsigned char v = (unsigned char)(major_version*16+minor_version);
  return WriteChar( v );
}

bool ON_BinaryArchive::Read3dmChunkVersion(
  int* major_version, // major // 0 to 15
  int* minor_version // minor // 0 to 16
  )
{
  unsigned char v = 0;
  bool rc = ReadChar( &v );
  if ( minor_version) *minor_version = v%16;
  // The bit shift happens on the fly in the following
  // if statement.  It was happening twice which always
  // set the major version to 0
  //v >>= 4;
  if ( major_version) *major_version = (v>>4);
  return rc;
}

int ON_BinaryArchive::Archive3dmVersion() const
{
  return m_3dm_version;
}

int ON_BinaryArchive::ArchiveOpenNURBSVersion() const
{
  return m_3dm_opennurbs_version;
}

size_t ON_BinaryArchive::ArchiveStartOffset() const
{
  return m_3dm_start_section_offset;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

bool 
ON_BinaryArchive::BeginWrite3dmChunk( unsigned int typecode, int value )
{
  ON__INT64 value64 = 0;
  if ( 0 != value )
  {
    if ( ON_IsUnsignedChunkTypecode(typecode) )
    {
      // treat value parameter as an unsigned int
      ON__UINT32 u32 = (ON__UINT32)value;
      ON__UINT64 u64 = u32;
      value64 = (ON__INT64)u64;
    }
    else
    {
      // treat value paramter is a signed int
      value64 = value;
    }
  }
  return BeginWrite3dmBigChunk(typecode,value);
}

bool 
ON_BinaryArchive::BeginWrite3dmBigChunk( ON__UINT32 typecode, ON__INT64 value64 )
{
  m_bDoChunkCRC = false; // no CRC on chunks because length is written twice.
  bool rc = WriteInt32( 1, (ON__INT32*)&typecode );
  if (rc)
    rc = WriteChunkValue( typecode, value64 );
  if (rc)
    rc = PushBigChunk( typecode, value64 );
  return rc;
}

bool 
ON_BinaryArchive::BeginWrite3dmChunk(
          unsigned int tcode,
          int major_version,
          int minor_version
          )
{
  bool rc = false;
  if ( 0 == tcode )
  {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmChunk - input tcode = 0");
  }
  else if ( 0 != (tcode & TCODE_SHORT) )
  {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmChunk - input tcode has short flag set.");
  }
  else if ( major_version <= 0 )
  {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmChunk - input major_version <= 0.");
  }
  else if ( minor_version < 0 )
  {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmChunk - input minor_version < 0.");
  }
  else
  {
    rc = BeginWrite3dmChunk(tcode,0);
    if (rc)
    {
      rc = WriteInt(major_version);
      if (rc)
        rc = WriteInt(minor_version);
      if ( !rc)
        EndWrite3dmChunk();
    }
  }
  return rc;
}

bool 
ON_BinaryArchive::EndWrite3dmChunk()
{
  bool rc = false;
  ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c ) {
    if ( c->m_bLongChunk ) 
    {
      if ( c->m_do_crc16 ) 
      {
        // write 16 bit CRC
        unsigned char two_zero_bytes[2] = {0,0};
        ON__UINT16 crc = ON_CRC16( c->m_crc16, 2, two_zero_bytes );
        rc = WriteInt16( 1, (ON__INT16*)&crc );
        if (c->m_crc16) 
        {
          // should never happen unless ON_CRC16() code is damaged
          m_bad_CRC_count++;
          ON_ERROR("ON_BinaryArchive::EndWrite3dmChunk: CRC16 computation error.");
        }
      }
      else if ( c->m_do_crc32 ) 
      {
        // write 32 bit CRC
        const ON__UINT32 crc0 = c->m_crc32; 
        rc = WriteInt32( 1, (ON__INT32*)&crc0 );
      }
      else {
        rc = true;
      }

      // write length
      m_bDoChunkCRC = 0;
      const size_t offset = CurrentPosition();
      if ( offset < c->m_big_offset ) 
      {
        ON_ERROR("ON_BinaryArchive::EndWrite3dmChunk() - chunk length < 0");
        rc = false;
      }
      else 
      {
        ON__UINT64 length = (offset - c->m_big_offset);
        if ( !BigSeekBackward( length + SizeofChunkLength() ) ) 
        {
          rc = false;
        }
        else 
        {
          if ( !WriteChunkLength( length ) )
          {
            rc = false;
          }
          if ( !BigSeekForward( length ) ) 
          {
            rc = false;
          }
        }
        if ( CurrentPosition() != offset ) 
        {
          ON_ERROR("ON_BinaryArchive::EndWrite3dmChunk() - CurrentPosition() != offset");
          rc = false;
        }
      }
    }
    else 
    {
      // "short" chunks are completely written by call to BeginWrite3dmChunk().
      rc = true;
    }

    m_chunk.Remove();
    c = m_chunk.Last();
    if ( !c )
    {
      Flush();
    }
    m_bDoChunkCRC = c && (c->m_do_crc16 || c->m_do_crc32);
  }
  return rc;
}

bool ON_BinaryArchive::Write3dmGoo( const ON_3dmGoo& goo )
{
  bool rc = false;

  if ( goo.m_typecode ) {
    const bool savedDoCRC = m_bDoChunkCRC;
    m_bDoChunkCRC = false;
    if ( 0 != (goo.m_typecode & TCODE_SHORT) ) {
      if ( goo.m_value == 0 || (goo.m_value > 0 && goo.m_goo) ) {
        // write long chunk - do not use Begin/EndWrite3dmChunk() because
        //                    goo may contain subchunks and CRC would be
        //                    incorrectly computed.
        rc = WriteInt( goo.m_typecode );
        if (rc) rc = WriteInt( goo.m_value );
        if (rc && goo.m_value>0) rc = WriteByte( goo.m_value, goo.m_goo );
      }
    }
    else {
      // write short chunk
      rc = WriteInt( goo.m_typecode );
      if (rc) rc = WriteInt( goo.m_value );
    }
    m_bDoChunkCRC = savedDoCRC;
  }
  
  return rc;
}

bool ON_BinaryArchive::PeekAt3dmChunkType( unsigned int* typecode, int* value )
{
  // does not change file position
  bool rc;
  unsigned int tc = 0;
  ON__INT64 i64 = 0;
  rc = PeekAt3dmBigChunkType(&tc,&i64);
  if ( rc )
  {
    if ( 0 != typecode )
      *typecode = tc;
    if ( 0 != value )
    {
      ON__INT32 i32 = 0;
      if ( ON_IsUnsignedChunkTypecode(tc) )
        rc = DownSizeUINT((ON__UINT64)i64,(ON__UINT32*)&i32);
      else
        rc = DownSizeINT(i64,&i32);
      *value = i32;
    }
  }
  return rc;
}

bool ON_BinaryArchive::PeekAt3dmBigChunkType( 
      unsigned int* typecode,
      ON__INT64* big_value
      )
{
  // does not change file position

  // 10 January 2005 Dale Lear
  //     Do not let the "peeking" affect the CRC.
  bool bDoChunkCRC = m_bDoChunkCRC;
  m_bDoChunkCRC = false;

  const ON__UINT64 pos0 = CurrentPosition();
  ON__UINT32 t = 0;
  ON__INT64 v = 0;
  bool rc = ReadChunkTypecode( &t );
  if (rc) 
  {
    rc = ReadChunkValue( t, &v );
  }
  const ON__UINT64 pos1 = CurrentPosition();
  if ( pos1 > pos0 && !BigSeekBackward( pos1-pos0 ) )
  {
    rc = false;
  }

  m_bDoChunkCRC = bDoChunkCRC;

  if ( typecode )
    *typecode = t;
  if ( big_value )
    *big_value = v;

  return rc;
}


bool ON_BinaryArchive::Seek3dmChunkFromStart( 
      // beginning at the start of the active chunk, search portion of
      // archive included in active chunk for the start of a subchunk 
      // with the specified type.
      // if true is returned, then the position is set so the next call to
      // BeginRead3dmChunk() will read a chunk with the specified typecode
      unsigned int typecode   // typecode from opennurbs_3dm.h
      )
{
  bool rc = false;
  if ( ReadMode() ) 
  {
    const ON__UINT64 pos0 = CurrentPosition();
    const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
    if ( c ) 
    {
      // set archive position to the beginning of this chunk
      if ( !ON_IsLongChunkTypecode(c->m_typecode) ) 
      {
        ON_ERROR("ON_BinaryArchive::Seek3dmChunkFromStart() - current chunk is not a long chunk");
        return false;
      }
      if ( c->m_big_value < 0 ) 
      {
        ON_ERROR("ON_BinaryArchive::Seek3dmChunkFromStart() called with an active chunk that has m_value < 0");
        return false;
      }
      if ( pos0 < c->m_big_offset || pos0 > c->m_big_offset + c->Length() ) 
      {
        ON_ERROR("ON_BinaryArchive::Seek3dmChunkFromStart() called with out of bounds current position");
        return false;
      }
      rc = BigSeekBackward( pos0 - c->m_big_offset ); // pos0 >= c->m_offset, so this size_t subtraction is ok
    }
    else 
    {
      // set archive position to the beginning of archive chunks by skipping
      // 32 byte version info and any start section padding.
      size_t start_offset = ((m_3dm_start_section_offset > 0) ? m_3dm_start_section_offset : 0);
      rc = SeekFromStart( start_offset );
      if (!rc && start_offset > 0)
      {
        start_offset = 0;
        rc = SeekFromStart(start_offset);
      }
      char s3d[33];
      memset(s3d,0,sizeof(s3d));
      if (rc) 
        rc = ReadByte(32,s3d);

      if (rc)
      {
        rc = (0 == strncmp( s3d, "3D Geometry File Format ", 24));
        if ( !rc && start_offset > 0 )
        {
          start_offset = 0;
          rc = SeekFromStart(start_offset);
          if (rc) rc = ReadByte(32,s3d);
          rc = (0 == strncmp( s3d, "3D Geometry File Format ", 24));
        }
      }

      if (rc)
      {
        if ( start_offset != m_3dm_start_section_offset )
          m_3dm_start_section_offset = start_offset;
        unsigned int t=0;
        ON__INT64 v=-1;
        rc = PeekAt3dmBigChunkType(&t,&v);
        if (rc && (t != 1 || v < 0) )
          rc = false;
      }
    }

    if (rc) 
    {
      rc = Seek3dmChunkFromCurrentPosition( typecode );
    }

    if (!rc) 
    {
      BigSeekFromStart(pos0);
    }
  }
  return rc;
}

ON__UINT64 ON_3DM_BIG_CHUNK::Length() const
{
  return (ON_IsLongChunkTypecode(m_typecode) && m_big_value >= 0 )
    ? ((ON__UINT64)m_big_value)
    : 0;
}

bool ON_BinaryArchive::Seek3dmChunkFromCurrentPosition( 
      // beginning at the current position, search portion of archive
      // included in active chunk for the start of a subchunk with the
      // specified type.
      // if true is returned, then the position is set so the next call to
      // BeginRead3dmChunk() will read a chunk with the specified typecode
      unsigned int typecode // typecode from opennurbs_3dm.h
      )
{
  bool rc = false;
  if ( ReadMode() ) 
  {
    const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
    const ON__UINT64 pos1 = c ? c->m_big_offset + c->Length() : 0;
    const size_t pos_start = CurrentPosition();
    size_t pos_prev = 0;
    size_t pos = 0;
    unsigned int t;
    ON__INT64 v64;
    bool bFirstTime = true;
    while(pos > pos_prev || bFirstTime) 
    {
      bFirstTime = false;
      pos_prev = pos;
      pos = CurrentPosition();
      if ( pos1 && pos > pos1 )
        break;
      t = !typecode;
      if ( !PeekAt3dmBigChunkType( &t, 0 ) )
        break;
      if ( t == typecode ) 
      {
        rc = true;
        break;
      }
      if ( 0 == t )
      {
        // zero is not a valid typecode - file is corrupt at or before this position
        break;
      }
      if ( !BeginRead3dmBigChunk( &t, &v64 ) )
        break;
      if ( !EndRead3dmChunk() )
        break;
      if ( TCODE_ENDOFTABLE == t && 0 != v64 )
      {
        // TCODE_ENDOFTABLE chunks always have value = 0 - file is corrupt at or before this position
        break;
      }
    }
    if ( !rc ) 
    {
      SeekFromStart( pos_start );
    }
  }
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmChunk( unsigned int* typecode, int* value )
{
  ON__UINT32 tc = 0;
  ON__INT64 v64 = 0;
  bool rc = BeginRead3dmBigChunk(&tc,&v64);
  if (rc)
  {
    if ( 0 != typecode )
      *typecode = tc;
    rc = DownSizeChunkValue(tc,v64,value);
  }
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmBigChunk( unsigned int* typecode, ON__INT64* value )
{
  ON__UINT32 t = 0;
  ON__INT64 v = 0;
  m_bDoChunkCRC = false; // no CRC on chunk headers because length is written twice
  const unsigned int saved_error_message_mask = m_error_message_mask;
  m_error_message_mask |= 0x0001; // disable ReadByte() error message at EOF
  bool rc = ReadChunkTypecode( &t );
  m_error_message_mask = saved_error_message_mask;
  if (rc) 
  {
    if ( t == TCODE_ENDOFFILE ) 
    {
      // Either this chunk is a bona fide end of file mark, or it's "goo" 
      // that Rhino 1.0 or the pre-February 2000 Rhino 1.1 saved and wrote.
      ON__UINT64 sizeof_file = 0;
      if ( rc )
        rc = ReadChunkValue( t, &v );
      if ( rc && v >= 0 && ((ON__UINT64)v) >= SizeofChunkLength() ) 
      {
        ON__UINT64 EOF_chunk_length = (ON__UINT64)v;
        ON__UINT64 pos0 = CurrentPosition();
        rc = ReadEOFSizeOfFile( &sizeof_file );
        ON__UINT64 pos1 = CurrentPosition();
        if ( pos0 > 0 && pos1 > pos0 )
        {
          if ( !BigSeekBackward(pos1-pos0) )
            rc = false;
        }
        if ( rc ) 
        {
          if ( BigSeekForward( EOF_chunk_length ) )
          {
            ON__UINT64 pos2 = CurrentPosition(); 
            if ( m_3dm_version <= 1 )
            {
              if ( !AtEnd() ) 
              {
                // Rhino v1 reads chunks with unknown typecodes as a block of "goo"
                // and then saves them back into file.  When this happens to an
                // eof marker, we get TCODE_ENDOFFILE chunks in the middle of a file.
                // This can only happen in m_3dm_version = 1 files.
                t = TCODE_ENDOFFILE_GOO;
              }
            }
            else
            {
              // check that current position matches saved file size
              if ( pos2 != sizeof_file ) {
                ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk() - Rogue eof marker in v2 file.\n");
              }
            }
            rc = BigSeekBackward( EOF_chunk_length );
          }
        }
        if ( rc )
          rc = PushBigChunk( t, v );
      }
      else 
      {
        ON_ERROR( "ON_BinaryArchive::BeginRead3dmChunk() - file is damaged." );
        rc = false;
        t = 0; // ?? file is trashed ??
      }
    }
    else 
    {
      if ( rc )
        rc = ReadChunkValue( t, &v );
      if ( rc )
        rc = PushBigChunk( t, v );
    }
  }
  if ( typecode )
    *typecode = t;
  if ( value )
    *value = v;
  return rc;
}

bool 
ON_BinaryArchive::BeginRead3dmChunk(
          unsigned int expected_tcode,
          int* major_version,
          int* minor_version
          )
{
  bool rc = false;
  if ( 0 == expected_tcode )
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - input expected_tcode = 0");
  }
  else if ( 0 != (expected_tcode & TCODE_SHORT) )
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - input expected_tcode has short flag set.");
  }
  else if ( 0 == major_version )
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - input major_version NULL");
  }
  else if ( 0 == minor_version )
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - input minor_version NULL");
  }
  else
  {
    *major_version = 0;
    *minor_version = 0;
    unsigned int tcode = 0;
    ON__INT64 value = 0;
    rc = PeekAt3dmBigChunkType(&tcode,&value);
    if ( expected_tcode != tcode )
    {
      ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - unexpected tcode");
      rc = false;
    }
    else if ( value < 8 )
    {
      ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - unexpected chunk length");
      rc = false;
    }
    else
    {
      tcode = 0;
      value = 0;
      rc = BeginRead3dmBigChunk(&tcode,&value);
      if (rc)
      {
        if ( expected_tcode != tcode || value < 8 )
        {
          // can happen when seek fails
          ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - unexpected tcode or chunk length - archive driver or device may be bad");
          rc = false;
        }
        else
        {
          rc = ReadInt(major_version);
          if ( rc && *major_version < 1 )
          {
            ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - major_version < 1");
            rc = false;
          }
          if (rc)
          {
            rc = ReadInt(minor_version);
            if ( rc && *minor_version < 0 )
            {
              ON_ERROR("ON_BinaryArchive::BeginRead3dmChunk - minor_version < 0");
              rc = false;
            }
          }
        }

        if ( !rc )
        {
          // this is required to keep chunk accounting in synch
          EndRead3dmChunk();
        }
      }
    }
  }
  return rc;
}
bool ON_BinaryArchive::EndRead3dmChunk()
{
  return EndRead3dmChunk(false);
}

bool ON_BinaryArchive::EndRead3dmChunk(bool bSupressPartiallyReadChunkWarning)  
{
  //int length = 0;
  bool rc = false;
  ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c ) 
  {
    ON__UINT64 file_offset = CurrentPosition();
    ON__UINT64 end_offset = c->m_big_offset;
    if ( c->m_bLongChunk )
    {
      if ( c->m_big_value < 0 )
      {
        ON_ERROR("ON_BinaryArchive::EndRead3dmChunk - negative chunk length");
      }
      else
      {
        end_offset += ((ON__UINT64)c->m_big_value);
      }
    }

    if ( c->m_bLongChunk ) 
    {
      if ( c->m_do_crc16 ) 
      {
        if ( file_offset+2 == end_offset )
        {
          // read 16 bit CRC
          unsigned char two_crc_bytes[2] = {0,0};
          rc = ReadByte( 2, two_crc_bytes );
          if (rc) 
          {
            file_offset+=2;
            if (c->m_crc16) 
            {
              m_bad_CRC_count++;
              ON_ERROR("ON_BinaryArchive::EndRead3dmChunk: CRC16 error.");
            }
          }
        }
        else
        {
          // partially read chunk - crc check not possible.
          rc = true;
        }
      }
      else if ( c->m_do_crc32 ) 
      {
        if ( file_offset+4 == end_offset )
        {
          // read 32 bit CRC
          ON__UINT32 crc1 = c->m_crc32;
          ON__UINT32 crc0;
          rc = ReadInt32( 1, (ON__INT32*)&crc0 );
          if (rc) 
          {
            file_offset+=4;
            if (crc0 != crc1) 
            {
              m_bad_CRC_count++;
              ON_ERROR("ON_BinaryArchive::EndRead3dmChunk: CRC32 error.");
            }
          }
        }
        else
        {
          // partially read chunk - crc check not possible.
          rc = true;
        }
      }
      else 
      {
        // no crc in this chunk
        rc = true;
      }
    }
    else 
    {
      rc = true;
    }

    // check length and seek to end of chunk if things are amiss
    if ( file_offset < c->m_big_offset ) 
    {
      ON_ERROR("ON_BinaryArchive::EndRead3dmChunk: current position before start of current chunk.");
      if ( !BigSeekFromStart( end_offset ) )
        rc = false;
    }
    else if ( file_offset > end_offset ) 
    {
      ON_ERROR("ON_BinaryArchive::EndRead3dmChunk: current position after end of current chunk.");
      if ( !BigSeekFromStart( end_offset ) )
        rc = false;
    }
    else if ( file_offset != end_offset ) 
    {
      // partially read chunk - happens when chunks are skipped or old code
      // reads a new minor version of a chunk whnich has added information.
      if ( file_offset != c->m_big_offset ) 
      {
        if ( m_3dm_version != 1 || (m_error_message_mask&0x02) == 0 ) 
        {
          // when reading v1 files, there are some situations where
          // it is reasonable to attempt to read 4 bytes at the end
          // of a file.  The above test prevents making a call
          // to ON_WARNING() in these situations.

          const int app_opennurbs_version = ON::Version();
          const int filev_date = m_3dm_opennurbs_version/10;
          const int appv_date  = app_opennurbs_version/10;
          int file_v = m_3dm_opennurbs_version%10;
          int app_v  = app_opennurbs_version%10;
          if ( 9 == file_v || 9 == app_v )
          {
            // 9 means DEBUG opennurbs was used to write the file.
            file_v = app_v = 0;
          }
          if ( file_v <= app_v && filev_date <= appv_date )
          {
            // We are reading a file written by this version or an
            // earlier version of opennurbs.  
            // There should not be any partially read chunks.
            if (!bSupressPartiallyReadChunkWarning)
            {
            ON_WARNING("ON_BinaryArchive::EndRead3dmChunk: partially read chunk - skipping bytes at end of current chunk.");
          }
        }
      }
      }
      //int delta =  (end_offset >= file_offset) 
      //          ?  ((int)(end_offset-file_offset)) 
      //          : -((int)(file_offset-end_offset));
      //if ( !SeekFromCurrentPosition( delta ) )
      //  rc = false;
      if ( end_offset > file_offset )
      {
        if ( !BigSeekForward(end_offset - file_offset) )
          rc = false;
      }
      else if ( end_offset < file_offset )
      {
        if ( !BigSeekBackward(file_offset - end_offset) )
          rc = false;
      }
    }

    m_chunk.Remove();
    c = m_chunk.Last();
    m_bDoChunkCRC = (c && (c->m_do_crc16 || c->m_do_crc32));
  }
  return rc;
}

bool ON_BinaryArchive::BeginWriteDictionary(
        ON_UUID dictionary_id,
        unsigned int version,
        const wchar_t* dictionary_name
        )
{
#if defined(ON_COMPILER_MSC)
// Disable the MSC /W4 "conditional expression is constant" warning
// about sizeof(unsigned short) == sizeof(*dictionary_name).  
// Since this code has to run on machines where sizeof(wchar_t) 
// can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( push )
#pragma warning( disable : 4127 )
#endif

  bool rc = BeginWrite3dmChunk(TCODE_DICTIONARY,1,0);
  if ( !rc )
    return rc;

  // Write dictionary id chunk
  rc = BeginWrite3dmChunk(TCODE_DICTIONARY_ID,1,0);
  if ( rc )
  {
    for(;;)
    {
      rc = WriteUuid(dictionary_id);
      if (!rc) break;
      rc = WriteInt(version);
      if (!rc) break;
      if ( sizeof(unsigned short) == sizeof(*dictionary_name) )
      {
        rc = WriteString((const unsigned short*)dictionary_name);
      }
      else
      {
        ON_wString s(dictionary_name);
        rc = WriteString(s);
      }
      if (!rc) break;
      break;
    }
    if ( !EndWrite3dmChunk() ) // TCODE_DICTIONARY_ID end
      rc = false;
  }

  if ( !rc )
    EndWrite3dmChunk(); // TCODE_DICTIONARY end
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool ON_BinaryArchive::EndWriteDictionary()
{
  int chunk_count = m_chunk.Count();
  bool rc = ( chunk_count > 0 && TCODE_DICTIONARY == m_chunk[chunk_count-1].m_typecode );
  if (rc)
  {
    rc = BeginWrite3dmChunk(TCODE_DICTIONARY_END,0);
    if (rc)
      rc = EndWrite3dmChunk(); // TCODE_DICTIONARY_END

    if ( !EndWrite3dmChunk() ) // TCODE_DICTIONARY
      rc = false;
  }
  return rc;
}

bool ON_BinaryArchive::BeginWriteDictionaryEntry(
        int de_type, 
        const wchar_t* entry_name
        )
{
#if defined(ON_COMPILER_MSC)
// Disable the MSC /W4 "conditional expression is constant" warning
// about sizeof(unsigned short) == sizeof(*entry_name).  
// Since this code has to run on machines where sizeof(wchar_t) 
// can be 2, 4, or 8 bytes, the test is necessary.
#pragma warning( push )
#pragma warning( disable : 4127 )
#endif

  bool rc = BeginWrite3dmChunk(TCODE_DICTIONARY_ENTRY,0);
  if ( rc )
  {
    for(;;)
    {
      rc = WriteInt(de_type);
      if (!rc) break;
      if ( sizeof(unsigned short) == sizeof(*entry_name) )
      {
        rc = WriteString((const unsigned short*)entry_name);
      }
      else
      {
        ON_wString s(entry_name);
        rc = WriteString(s);
      }
      if (!rc) break;
      break;
    }
    if ( !rc )
      EndWrite3dmChunk(); // TCODE_DICTIONARY_ENTRY
  }
  return rc;

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif
}

bool ON_BinaryArchive::EndWriteDictionaryEntry()
{
  int chunk_count = m_chunk.Count();
  bool rc = ( chunk_count > 0 && TCODE_DICTIONARY_ENTRY == m_chunk[chunk_count-1].m_typecode )
          ? EndWrite3dmChunk()
          : false;
  return rc;
}

bool ON_BinaryArchive::BeginReadDictionary(
        ON_UUID* dictionary_id,
        unsigned int* version,
        ON_wString& dictionary_name
        )
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = BeginRead3dmChunk(TCODE_DICTIONARY,&major_version,&minor_version);
  if ( rc )
  {
    for(;;)
    {
      rc = (1 == major_version);
      if (!rc) break;

      // Read dictionary id chunk
      rc = BeginRead3dmChunk(TCODE_DICTIONARY_ID,&major_version,&minor_version);
      if ( !rc ) break;
      for(;;)
      {
        rc = (1==major_version);
        if (!rc) break;
        ON_UUID id;
        rc = ReadUuid(id);
        if (!rc) break;
        if ( dictionary_id )
          *dictionary_id = id;
        rc = ReadInt(version);
        if (!rc) break;
        rc = ReadString(dictionary_name);
        if (!rc) break;
        break;
      }
      if ( !EndRead3dmChunk() ) // TCODE_DICTIONARY_ID end
        rc = false;
      break;
    }

    if ( !rc )
      EndRead3dmChunk(); // TCODE_DICTIONARY end
  }
  return rc;
}

bool ON_BinaryArchive::EndReadDictionary()
{
  int chunk_count = m_chunk.Count();
  bool rc = ( chunk_count > 0 && TCODE_DICTIONARY == m_chunk[chunk_count-1].m_typecode )
          ? EndRead3dmChunk()
          : false;
  return rc;
}

int ON_BinaryArchive::BeginReadDictionaryEntry(
        int* de_type, 
        ON_wString& entry_name
        )
{
  unsigned int tcode = 0;
  ON__INT64 chunk_length = 0;
  int chunk_count = m_chunk.Count();
  int rc = ( chunk_count > 0 && TCODE_DICTIONARY == m_chunk[chunk_count-1].m_typecode )
          ? (BeginRead3dmBigChunk(&tcode,&chunk_length)?1:0)
          : 0;
  if ( de_type )
    *de_type = 0;
  if ( rc )
  {
    if ( TCODE_DICTIONARY_ENTRY == tcode )
    {
      for(;;)
      {
        rc = 0;
        if ( !ReadInt(de_type) )
        {
          entry_name.Empty();
          break;
        }
        if ( !ReadString(entry_name) )
        {
          entry_name.Empty();
          break;
        }
        rc = 1;
        break;
      }
    }
    else
    {
      rc = (TCODE_DICTIONARY_END == tcode) ? 2 : 0;
    }
    if ( 1 != rc )
    {
      if ( !EndRead3dmChunk() )
        rc = 0;
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndReadDictionaryEntry()
{
  int chunk_count = m_chunk.Count();
  bool rc = ( chunk_count > 0 && TCODE_DICTIONARY_ENTRY == m_chunk[chunk_count-1].m_typecode )
          ? EndRead3dmChunk()
          : false;
  return rc;
}



bool ON_BinaryArchive::Read3dmGoo( ON_3dmGoo& goo )
{
  // goo is an entire "chunk" that is not short.
  // A call to EndRead3dmChunk() must immediately follow 
  // the call to Read3dmGoo().
  bool rc = false;
  if (goo.m_goo) 
  {
    onfree(goo.m_goo);
    goo.m_goo = 0;
  }
  goo.m_typecode = 0;
  goo.m_value = 0;
  ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if (c) 
  {
    goo.m_typecode = c->m_typecode;
    if ( c->m_bLongChunk )
      rc = DownSizeUINT(c->Length(),(ON__UINT32*)&goo.m_value);
    else
      rc = DownSizeINT(c->m_big_value,&goo.m_value);
    if ( rc && c->m_bLongChunk && c->m_big_value > 0 ) 
    {
      if ( CurrentPosition() == c->m_big_offset )
      {
        // read the rest of this chunk into the goo.m_goo buffer.
        // 23 January 2004 Dale Lear:
        //   Have to turn of CRC checking because the goo may contiain
        //   subchunks.  If a CRC exixts, it is at the end of the
        //   goo and will persist until the application that
        //   wrote this chunk is available to parse the chunk.
        c->m_do_crc16 = 0;
        c->m_do_crc32 = 0;
        m_bDoChunkCRC = false;
        size_t sizeof_goo = (size_t)c->Length();
        goo.m_goo = (unsigned char*)onmalloc( sizeof_goo );
        rc = ReadByte( sizeof_goo, goo.m_goo );
      }
    }
  }
  return rc;
}

bool ON_BinaryArchive::WriteChunkTypecode( ON__UINT32 typecode )
{
  return WriteInt32(1,(ON__INT32*)&typecode);
}

bool ON_BinaryArchive::ReadChunkTypecode( ON__UINT32* typecode )
{
  ON__UINT32 tc = 0;
  bool rc = ReadInt32(1,(ON__INT32*)&tc);
  if (rc && typecode )
    *typecode = tc;
  return rc;
}

bool ON_BinaryArchive::WriteChunkValue( ON__UINT32 typecode, ON__INT64 big_value )
{
  bool rc;
  if ( 8 == SizeofChunkLength() )
  {
    rc = WriteInt64(1,&big_value);
  }
  else if ( ON_IsUnsignedChunkTypecode(typecode) )
  {
    // treat big_value as an unsigned int
    ON__UINT32 u32 = 0;
    rc = DownSizeUINT((ON__UINT64)big_value,&u32);
    if ( !WriteInt32(1,(ON__INT32*)&u32) )
      rc = false;
  }
  else
  {
    // treat big_value as a signed int
    ON__INT32 v32 = 0;
    rc = DownSizeINT(big_value,&v32);
    if ( !WriteInt32(1,&v32) )
      rc = false;
  }
  return rc;
}


bool ON_BinaryArchive::WriteChunkLength( ON__UINT64 length )
{
  bool rc;
  if ( 8 == SizeofChunkLength() )
  {
    rc = WriteInt64(1,(ON__INT64*)&length);
  }
  else
  {
    ON__UINT32 u32 = 0;
    rc = DownSizeUINT(length,&u32);
    if ( !WriteInt32(1,(ON__INT32*)&u32) )
      rc = false;
  }
  return rc;
}

bool ON_BinaryArchive::ReadEOFSizeOfFile( ON__UINT64* sizeof_file )
{
  bool rc;
  ON__INT64 u64 = 0;
  if ( 8 == SizeofChunkLength() )
  {
    // file has a 8 byte file size
    rc = ReadInt64(1,(ON__INT64*)&u64);
  }
  else
  {
    // file has a 4 byte file size
    ON__UINT32 u32 = 0;
    rc = ReadInt32(1,(ON__INT32*)&u32);
    if ( rc )
      u64 = u32;
  }
  if ( rc && 0 != sizeof_file )
    *sizeof_file = u64;
  return rc;
}

bool ON_BinaryArchive::WriteEOFSizeOfFile( ON__UINT64 sizeof_file )
{
  bool rc;
  if ( 8 == SizeofChunkLength() )
  {
    // file has a 8 byte file size
    rc = WriteInt64(1,(ON__INT64*)&sizeof_file);
  }
  else
  {
    // file has a 4 byte file size
    ON__UINT32 u32=0;
    DownSizeUINT(sizeof_file,&u32);
    rc = WriteInt32(1,(ON__INT32*)&u32);
  }
  return rc;
}

bool ON_BinaryArchive::ReadChunkValue( ON__UINT32 typecode, ON__INT64* value64 )
{
  bool rc;
  ON__INT64 i64 = 0;
  if ( 8 == SizeofChunkLength() )
  {
    // file has a 8 byte chunk value
    rc = ReadInt64(1,&i64);
  }
  else
  {
    // file has a 4 byte chunk value
    if ( ON_IsUnsignedChunkTypecode(typecode) )
    {
      // This Mickey Mouse is here to convince all compilers 
      // that when we read a 4 byte value with the high bit set, 
      // the resulting i64 value is positive. I.e.,
      // 0xFFFFFFFF is converted to 0x00000000FFFFFFFF
      ON__UINT32 u32 = 0;
      ON__UINT64 u64 = 0;
      rc = ReadInt32(1,(ON__INT32*)&u32);
      if ( rc )
        u64 = u32;
      i64 = (ON__INT64)u64;
    }
    else
    {
      // If we read a 4 byte value with the high bit set,
      // the resulting i64 value is negative. I.e.,
      // -1 is converted to -1 (0xFFFFFFFF to 0xFFFFFFFFFFFFFFFF)
      ON__INT32 i32 = 0;
      rc = ReadInt32(1,&i32);
      i64 = i32;
    }
  }
  if ( rc && 0 != value64 )
    *value64 = i64;
  return rc;
}

size_t ON_BinaryArchive::SizeofChunkLength() const
{
  // Version 1 - 4 and early version 5 files had
  // 4 byte chunk lengths.  In October of 2009,
  // 8 byte chunk lengths were phased in for V5
  // files.
  return (m_3dm_version < 50) ? 4 : 8;
}

bool ON_BinaryArchive::PushBigChunk( ON__UINT32 typecode, ON__INT64 big_value )
{
  ON_3DM_BIG_CHUNK c;
  memset(&c,0,sizeof(c));
  c.m_typecode  = typecode;
  c.m_big_value = big_value;

  // | and & are BITOPS - do NOT change to || and &&
  //
  // Some v1 files have a short chunk with typecode = 0.
  if ( 0 == ( TCODE_SHORT & typecode ) && (0 != typecode  || 1 != Archive3dmVersion()) ) 
  {
    if ( m_3dm_version == 1 && 0 != (TCODE_LEGACY_GEOMETRY & typecode) )
    {
      // these legacy typecodes have 16 bit CRCs
      c.m_do_crc16 = 1;
      c.m_crc16 = 1;
    }
    else 
    {
      // some other legacy typecodes that have 16 bit CRCs
      switch(typecode) 
      {

      case TCODE_SUMMARY:
        if ( m_3dm_version == 1 ) 
        {
          c.m_do_crc16 = 1;
          c.m_crc16 = 1;
        }
        break;

      case TCODE_OPENNURBS_OBJECT | TCODE_CRC | 0x7FFD:
        if ( m_3dm_version == 1 ) 
        {
          // 1.1 uuid has a 16 bit crc
          c.m_do_crc16 = 1;
          c.m_crc16 = 1;
        }
        else 
        {
          // 2.0 uuid has a 32 bit crc
          c.m_do_crc32 = 1;
          c.m_crc32 = 0;
        }
        break;

      default:
        if ( m_3dm_version != 1 && 0 != (TCODE_CRC & typecode) ) 
        {
          // 32 bit CRC
          c.m_do_crc32 = 1;
          c.m_crc32 = 0;
        }
        break;

      }
    }
    c.m_bLongChunk = 1;
  }
  c.m_big_offset = CurrentPosition();
  m_bDoChunkCRC = c.m_do_crc16 || c.m_do_crc32;
  
  if ( m_chunk.Capacity() == 0 )
    m_chunk.Reserve(128);
  m_chunk.Append( c );

  return true;
}

bool ON_BinaryArchive::EnableSave3dmRenderMeshes( ON_BOOL32  b /* default = true */ )
{
  bool oldb = m_bSaveRenderMeshes;
  m_bSaveRenderMeshes = b?true:false;
  return oldb;  
}

bool ON_BinaryArchive::Save3dmRenderMeshes() const
{
  return m_bSaveRenderMeshes;
}

bool ON_BinaryArchive::EnableSave3dmAnalysisMeshes( ON_BOOL32  b /* default = true */ )
{
  bool oldb = m_bSaveAnalysisMeshes;
  m_bSaveAnalysisMeshes = b?true:false;
  return oldb;  
}

bool ON_BinaryArchive::Save3dmAnalysisMeshes() const
{
  return m_bSaveAnalysisMeshes;
}

bool ON_BinaryArchive::EnableSaveUserData( ON_BOOL32 b )
{
  bool b0 = m_bSaveUserData;
  m_bSaveUserData = b?true:false;
  return b0;
}

bool ON_BinaryArchive::SaveUserData() const
{
  return m_bSaveUserData;
}

int ON_BinaryArchive::CurrentArchiveVersion()
{
  // Latest version of opennurbs binary archives supported by
  // this version of opennurbs.
  return 50; // Rhino 5.0 files with 8 byte chunk lengths.
}

bool ON_BinaryArchive::Write3dmStartSection( int version, const char* sInformation )
{
  if ( 0 == version )
    version = ON_BinaryArchive::CurrentArchiveVersion();

  // 2009 November 6 
  // Default 3dm files now have 8 byte chunk lengths.
  // 3dm archive version numbers >= 50 indicate the file has 8 byte chunk lengths.
  // Rather than change the hundreds of places in Rhino that use 5, 6, 7 ...
  // the "version *= 10" line handles it.  
  if ( version >= 5 && version < 50 )
    version *= 10;

  if ( version > ON_BinaryArchive::CurrentArchiveVersion() )
  {
    ON_ERROR("3dm archive version must be <= ON_BinaryArchive::CurrentArchiveVersion() ");
    return false;
  }

  if (    version < 2 
       || (version >= 5 && version < 50) 
       || (version >= 50 && 0 != (version % 10)) 
       )
  {
    ON_ERROR("3dm archive version must be 2, 3, 4 or 50");
    return false;
  }

  m_bad_CRC_count = 0;
  m_3dm_version = 0;
  m_3dm_opennurbs_version = ON::Version();
  m_3dm_version = version;

  char sVersion[64];
  memset( sVersion, 0, sizeof(sVersion) );
  if ( version < 1 )
    version = ON_BinaryArchive::CurrentArchiveVersion();
  sprintf(sVersion,"3D Geometry File Format %8d",version);
  bool rc = WriteByte( 32, sVersion );
  if ( rc )
    rc = BeginWrite3dmBigChunk( TCODE_COMMENTBLOCK, 0 );
  if ( rc ) {
    if ( sInformation && sInformation[0] ) {
      rc = WriteByte( strlen(sInformation), sInformation );
    }
    if ( rc ) {
      // write information that helps determine what code wrote the 3dm file
      char s[2048];
      size_t s_len = 0;
      memset( s, 0, sizeof(s) );
      sprintf(s," 3DM I/O processor: OpenNURBS toolkit version %d",ON::Version());
      strcat(s," (compiled on ");
      strcat(s,__DATE__);
      strcat(s,")\n");
      s_len = strlen(s);
      s[s_len++] = 26; // ^Z
      s[s_len++] = 0;
      rc = WriteByte( s_len, s );
    }
    if ( !EndWrite3dmChunk() ) // need to call EndWrite3dmChunk() even if a WriteByte() has failed
      rc = false;
  }
  return rc;
}

bool ON_BinaryArchive::Read3dmStartSection( int* version, ON_String& s )
{
  // The first 24 bytes of a 3dm file must be "3D Geometry File Format "
  // The next 8 bytes must be a right justified ASCII integer >= 1.  For
  // example, prior to March 2000, all 3DM files were version 1 files and
  // they began with "3D Geometry File Format        1".  At the time of
  // this writing (May 2011) there are version 1,2,3,4,5 and 50 files.
  //
  // The next section must contain a long chunk with typecode 1.
  m_bad_CRC_count = 0;
  m_3dm_version = 0;

  // In Read3dmProperties, m_3dm_opennurbs_version will be set to 
  // the version of OpenNURBS that was used to write this archive.
  m_3dm_opennurbs_version = 0;

  unsigned int typecode = 0;
  ON__INT64 length = -1;
  int ver = m_3dm_version;
  if ( version )
    *version = m_3dm_version;
  s.Destroy();
  char s3d[33];
  memset( s3d, 0, sizeof(s3d) );
  bool rc = ReadByte( 32, s3d );
  if ( rc ) 
  {

    if ( strncmp( s3d, "3D Geometry File Format ", 24) ) 
    {
      // it's not a "pure" .3DM file 
      // - see if we have a .3DM file with MS OLE-goo at the start
      // (generally, there is around 6kb of goo.  I keep looking
      // for up to 32mb just in case.)
      rc =  false;
      unsigned int n;
      int j;
      for ( n = 0; n < 33554432 && !rc; n++ ) 
      {
        for ( j = 0; j < 31; j++ )
          s3d[j] = s3d[j+1];
        if ( !ReadByte( 1, &s3d[31]) ) 
          break;
        if ( !strncmp( s3d, "3D Geometry File Format ", 24) ) 
        {
          m_3dm_start_section_offset = n+1;
          rc = true;
          break;
        }
      }
    }

    if (rc) {
      // get version
      //char* sVersion = s3d+24;
      // skip leading spaces
      int i = 24;
      while (i < 32 && s3d[i] == ' ')
        i++;
      while (i < 32 && rc) {
        // TEMPORARY 2 = X
        if ( i == 31 && s3d[i] == 'X' ) {
          s3d[i] = '2';
        }

        if ( s3d[i] < '0' || s3d[i] > '9' ) {
          rc = false; // it's not a valid .3DM file version
          break;
        }
        ver = ver*10 + ((int)(s3d[i]-'0'));
        i++;
      }
      if ( rc ) {
        m_3dm_version = ver;
        if ( version )
          *version = ver;
      }
    }
    if ( rc ) {
      rc = BeginRead3dmBigChunk( &typecode, &length );
      if ( rc ) {
        if ( typecode != 1 ) {
          rc = false; // it's not a .3DM file
        }
        else if ( length > 0 ) 
        {
          if ( length > 0x00FFFFFF )
          {
            ON_ERROR("ON_BinaryArchive::Read3dmStartSection - start section string is unreasonably long.");
            rc = false;
          }
          else 
          {
            int slen = (int)length;
            s.ReserveArray( slen+1 );
            s.SetLength( slen );
            s[slen] = 0;
            ReadByte( slen, s.Array() );
            while ( slen > 0 && (0 == s[slen-1] || 26 == s[slen-1]) )
            {
              s[slen-1] = 0;
              slen--;
            }
            s.SetLength(slen);
          }
        }
      }
      if ( !EndRead3dmChunk() )
        rc = false;
      if ( rc && m_3dm_version == 1 ) {
        // In March 2001, we got reports of files with V1 headers and
        // a V2 bodies.  We haven't been able to track down the application
        // that is creating these damaged files, but we can detect them
        // and read them because they all have a TCODE_PROPERTIES_TABLE
        // chunk right after the start comments chunk and no valid V1
        // file has a chunk with a TCODE_PROPERTIES_TABLE tcode.
        //
        // Rhino 1.1 version 31-May-2000 reads 2.0 files as goo.  If a user
        // saves this file for some reason (no instances have been reported)
        // the resulting file has a V1 header, V1 fluff, and a V2 body.
        // This code will cause opennurbs to read the V2 body.
        // a file that is different from those describe This code
        // detects these files.
        {

          const ON__UINT64 pos1 = CurrentPosition();
          //int v1_fluff_chunk_count = 0;
          bool bCheckChunks = true;

          //////////
          while(bCheckChunks) {
            if ( !PeekAt3dmBigChunkType(&typecode,&length) )
              break;
            switch(typecode)
            {
              case TCODE_SUMMARY:
              case TCODE_BITMAPPREVIEW:
              case TCODE_UNIT_AND_TOLERANCES:
              case TCODE_VIEWPORT:
              case TCODE_LAYER:
              case TCODE_RENDERMESHPARAMS:
              case TCODE_CURRENTLAYER:
              case TCODE_ANNOTATION_SETTINGS:
              case TCODE_NOTES:
              case TCODE_NAMED_CPLANE:
              case TCODE_NAMED_VIEW:
                // skip potential v1 fluff
                bCheckChunks = BeginRead3dmBigChunk( &typecode, &length );
                if ( bCheckChunks )
                  bCheckChunks = EndRead3dmChunk();
                break;

              //case TCODE_PROPERTIES_TABLE:
              //case TCODE_SETTINGS_TABLE:
              //case TCODE_OBJECT_TABLE:
              //case TCODE_BITMAP_TABLE:
              //case TCODE_LAYER_TABLE:
              //case TCODE_GROUP_TABLE:
              //case TCODE_LIGHT_TABLE:
              //case TCODE_MATERIAL_TABLE:
              //case TCODE_USER_TABLE:
              default:
                if ( TCODE_TABLE == (typecode & 0xFFFF0000) ) {
                  // Found a V2 table which has to be V1 goo
                  ON_WARNING("ON_BinaryArchive::Read3dmStartSection(): Archive has V1 header and V2 body. Continuing to read V2 body.");
                  m_3dm_version = 2;
                  if ( version )
                    *version = 2;
                }
                bCheckChunks = false;
                break;
            }
          }

          if ( m_3dm_version == 1 ) {
            // move archive pointer back to 
            ON__UINT64 pos2 = CurrentPosition();
            if ( pos2 > pos1 ) 
            {
              BigSeekBackward(pos2 - pos1);
            }
          }
        }
      }
    }
  }
  return rc;
}

bool ON_BinaryArchive::Write3dmProperties(
      const ON_3dmProperties& prop
      )
{
  bool rc = false;
  if ( m_3dm_version == 1 ) 
  {
    ON_String s;

    rc = true;

    if ( rc && prop.m_RevisionHistory.IsValid() ) 
    {
      rc = BeginWrite3dmChunk(TCODE_SUMMARY,0);
      if (rc)
      {
        // version 1 revision history chunk
        s = prop.m_RevisionHistory.m_sCreatedBy;
        if (rc) rc = WriteString(s);
        if (rc) rc = WriteTime( prop.m_RevisionHistory.m_create_time );
        if (rc) rc = WriteInt(0);
        s = prop.m_RevisionHistory.m_sLastEditedBy;
        if (rc) rc = WriteString(s);
        if (rc) rc = WriteTime( prop.m_RevisionHistory.m_last_edit_time );
        if (rc) rc = WriteInt(0);
        if (rc) rc = WriteInt( prop.m_RevisionHistory.m_revision_count );
        if ( !EndWrite3dmChunk() ) // writes 16 bit crc
          rc = false;
      }
    }

    if ( rc && prop.m_Notes.IsValid() ) 
    {
      rc = BeginWrite3dmChunk(TCODE_NOTES,0);
      // version 1 notes chunk
      if ( rc )
      {
        if ( rc ) rc = WriteInt( prop.m_Notes.m_bVisible );
        if ( rc ) rc = WriteInt( prop.m_Notes.m_window_left );
        if ( rc ) rc = WriteInt( prop.m_Notes.m_window_top );
        if ( rc ) rc = WriteInt( prop.m_Notes.m_window_right );
        if ( rc ) rc = WriteInt( prop.m_Notes.m_window_bottom );
        s = prop.m_Notes.m_notes;
        if ( rc ) rc = WriteString( s );
        if ( !EndWrite3dmChunk() )
          rc = false;
      }
    }

    if ( rc && prop.m_PreviewImage.IsValid() )
    {
      rc = BeginWrite3dmChunk(TCODE_BITMAPPREVIEW,0);
      if (rc)
      {
        // version 1 preview image chunk
        prop.m_PreviewImage.Write(*this);
        if ( !EndWrite3dmChunk() )
          rc = false;
      }
    }
  }
  else 
  {
    // version 2+ file properties chunk
    rc = BeginWrite3dmChunk(TCODE_PROPERTIES_TABLE,0);
    if ( rc ) {
      rc = prop.Write( *this )?true:false;
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

int on_strnicmp(const char * s1, const char * s2, int n)
{
#if defined(ON_OS_WINDOWS)
  //return stricmp(s1,s2,n);
  return _strnicmp(s1,s2,n);
#else
  return strncasecmp(s1,s2,n);
#endif
}

bool ON_BinaryArchive::Read3dmProperties( ON_3dmProperties& prop )
{
  // In ON_3dmProperties::Read(), m_3dm_opennurbs_version will be 
  // set to the version of OpenNURBS that was used to write this archive.
  // If the file was written with by a pre 200012210 version of OpenNURBS,
  // then m_3dm_opennurbs_version will be zero.
  m_3dm_opennurbs_version = 0;

  prop.Default();

  bool rc = true;

  // we need these when reading version 1 files
  const size_t pos0 = CurrentPosition();
  bool bHaveRevisionHistory = false;
  bool bHaveNotes = false;
  bool bHavePreviewImage = false;
  bool bDone = false;
  bool bRewindFilePointer = false;

  ON__UINT32 tcode;
  ON__INT64 big_value;
  int version = 0;

  if ( m_3dm_version != 1 ) {
    for(;;) 
    {
      tcode = 0;
      big_value = 0;
      rc = BeginRead3dmBigChunk( &tcode, &big_value );
      if ( !rc ) {
        bRewindFilePointer = true;
        break;
      }

      if ( tcode == TCODE_PROPERTIES_TABLE ) {
        rc = prop.Read(*this)?true:false;
      }
      else {
        bRewindFilePointer = true;
      }

      if ( !EndRead3dmChunk() ) {
        rc = false;
        bRewindFilePointer = true;
      }
      if ( tcode == TCODE_PROPERTIES_TABLE || !rc )
        break;
    }
  }
  else {
    // version 1 file
    rc = SeekFromStart(32)?true:false;
    bRewindFilePointer = true;
    for(;;) 
    {
      tcode = 0;
      big_value = 0;
      rc = BeginRead3dmBigChunk( &tcode, &big_value );
      if ( !rc ) {
        rc = true; // assume we are at the end of the file
        bRewindFilePointer = true;
        break;
      }

      switch ( tcode ) {

      case 1: // comments section has application name
        if ( big_value > 1000000 )
        {
          ON_ERROR("Comment length > 1000000");
        }
        else if ( big_value > 1 )
        {
          int slen = (int)big_value;
          int i;
          char* name = 0;
          ON_String s;
          s.ReserveArray( slen+1 );
          s.SetLength( slen );
          s[slen] = 0;
          ReadByte( slen, s.Array() );
          while ( slen > 0 && (0 == s[slen-1] || 26 == s[slen-1]) ) 
          {
            s[slen-1] = 0;
            slen--;
          }
          s.SetLength(slen);
          name = s.Array();
          if ( name ) {
            while(*name) {
              if ( !on_strnicmp(name,"Interface:",10) ) {
                name += 10;
                break;
              }
              name++;
            }
            while(*name && *name <= 32 )
              name++;
            for ( i = 0; name[i] ; i++ ) {
              if ( name[i] == '(' ) {
                name[i] = 0;
                while ( i > 0 && (name[i] <= 32 || name[i] == '-') ) {
                  name[i] = 0;
                  i--;
                }
                break;
              }
            }
            if ( *name )
            {
              char* details = 0;
              if ( !on_strnicmp(name,"Rhinoceros",10) ) {
                prop.m_Application.m_application_URL = "http://www.rhino3d.com";
                details = name+10;
                while ( *details && *details <= 32 )
                  details++;
                while ( (*details >= '0' && *details <= '9') || *details == '.' )
                  details++;
                if ( *details && *details <= 32 ) {
                  *details = 0;
                  details++;
                  while ( *details && (*details <= 32 ||*details == '-')) {
                    details++;
                  }
                }
              }
              if (*name)
                prop.m_Application.m_application_name = name;
              if (details && *details)
                prop.m_Application.m_application_details = details;
            }
          }
        }
        break;

      case TCODE_SUMMARY: 
        // version 1 revision history chunk (has 16 bit CRC)
        version = 1;
        bHaveRevisionHistory = true;
        {
          int slength = 0;
          char* s = 0;
          if (rc) rc = ReadInt(&slength);
          if (rc && slength > 0 ) {
            s = (char*)onmalloc((slength+1)*sizeof(*s));
            memset( s, 0, (slength+1)*sizeof(*s) );
            if (rc) rc = ReadChar( slength, s );
            if ( rc )
              prop.m_RevisionHistory.m_sCreatedBy = s;
            onfree(s);
            slength = 0;
            s = 0;
          }
          if (rc) rc = ReadTime( prop.m_RevisionHistory.m_create_time );
          int i32 = 0;
          if (rc) rc = ReadInt(&i32); // 0 in 1.x files
          if (rc) rc = ReadInt(&slength);
          if ( rc && slength > 0 ) 
          {
            s = (char*)onmalloc((slength+1)*sizeof(*s));
            memset( s, 0, (slength+1)*sizeof(*s) );
            if (rc) rc = ReadChar( slength, s );
            if ( rc )
              prop.m_RevisionHistory.m_sLastEditedBy = s;
            onfree(s);
            slength = 0;
            s = 0;
          }
          if (rc) rc = ReadTime( prop.m_RevisionHistory.m_last_edit_time );
          if (rc) rc = ReadInt(&i32); // 0 in 1.x files
          if (rc) rc = ReadInt( &prop.m_RevisionHistory.m_revision_count );
        }
        break;

      case TCODE_NOTES: 
        // version 1 notes chunk
        version = 1;
        bHaveNotes = true;
        for(;;)
        {
          int slength;
          char* s = 0;
          rc = ReadInt( &prop.m_Notes.m_bVisible );
          if(!rc) break;
          rc = ReadInt( &prop.m_Notes.m_window_left );
          if(!rc) break;
          rc = ReadInt( &prop.m_Notes.m_window_top );
          if(!rc) break;
          rc = ReadInt( &prop.m_Notes.m_window_right );
          if(!rc) break;
          rc = ReadInt( &prop.m_Notes.m_window_bottom );
          if(!rc) break;
          rc = ReadInt( &slength );
          if(!rc) break;
          if ( slength > 0 ) 
          {
            s = (char*)onmalloc( (slength+1)*sizeof(*s) );
            memset( s, 0, (slength+1)*sizeof(*s) );
            if ( rc ) rc = ReadChar( slength, s );
            if ( rc ) 
            {
              prop.m_Notes.m_notes = s;
            }
            onfree(s);
            slength = 0;
            s = 0;
          }
          break;
        }
        break;

      case TCODE_BITMAPPREVIEW: 
        // version 1 preview image chunk
        version = 1;
        rc = prop.m_PreviewImage.Read(*this)?true:false;
        bHavePreviewImage = rc;
        break;

      case TCODE_CURRENTLAYER:
      case TCODE_LAYER:
        // version 1 layer and current layer chunks always came after notes/bitmap/summary
        bDone = true;
        bRewindFilePointer = true;
        break;

      default:
        // the call to EndRead3dmChunk() will skip over this chunk
        bRewindFilePointer = true;
        break;
      }

      // this call to EndRead3dmChunk() skips any unread portions of the chunk
      if ( !EndRead3dmChunk() ) {
        rc = false;
        bRewindFilePointer = true;
      }

      if ( bHaveRevisionHistory && bHaveNotes && bHavePreviewImage )
        bDone = true;

      if ( bDone || !rc )
        break;
    }
  }

  if ( bRewindFilePointer ) {
    // reposition file pointer to pos0
    const ON__UINT64 pos1 = CurrentPosition();
    if ( pos0 != pos1 ) 
    {
      if (pos1 > pos0)
        BigSeekBackward(pos1-pos0);
      else if ( pos1 < pos0 )
        BigSeekForward(pos0-pos1);
    }
  }

  return rc;
}

bool ON_BinaryArchive::Write3dmSettings(
      const ON_3dmSettings& settings
      )
{
  bool rc = false;
  if ( m_3dm_version == 1 ) {
    // legacy v1 settings info is a bunch of unreleated top level chunks
    rc = settings.Write(*this)?true:false;
  }
  else 
  {
    // version 2+ file settings chunk
    rc = BeginWrite3dmChunk(TCODE_SETTINGS_TABLE,0);
    if ( rc ) {
      rc = settings.Write( *this );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }

    if ( rc && 3 == Archive3dmVersion() )
    {
      // Build a list of ids of plug-ins that support saving
      // V3 user data.  If a plug-in id is not in this list,
      // the user data will not be saved in the V3 archive.
      int i, count = settings.m_plugin_list.Count();
      m_V3_plugin_id_list.SetCount(0);
      m_V3_plugin_id_list.SetCapacity( count+5 );
      for ( i = 0; i < count; i++ )
      {
        const ON_UUID& pid = settings.m_plugin_list[i].m_plugin_id;
        if ( !ON_UuidIsNil(pid) )
          m_V3_plugin_id_list.Append(pid);
      }

      // These ids insure V3, V4 and V5 core user data will round trip
      // through SaveAs V3 and SaveAs V4
      m_V3_plugin_id_list.Append( ON_v3_userdata_id );
      m_V3_plugin_id_list.Append( ON_v4_userdata_id );
      m_V3_plugin_id_list.Append( ON_opennurbs4_id );
      m_V3_plugin_id_list.Append( ON_opennurbs5_id );
      m_V3_plugin_id_list.Append( ON_rhino3_id );
      m_V3_plugin_id_list.Append( ON_rhino4_id );
      m_V3_plugin_id_list.Append( ON_rhino5_id );
      m_V3_plugin_id_list.QuickSort( ON_UuidCompare );
    }
  }
  return rc;
}

bool ON_BinaryArchive::Read3dmSettings( ON_3dmSettings& settings )
{
  bool rc = false;
  ON__UINT32 tcode;
  ON__INT64 big_value;

  if ( m_3dm_version == 1 ) {
    // read legacy v 1 info that is scattered around the file
    rc = settings.Read(*this);
  }
  else {
    rc = true;
    while(rc) 
    {
      tcode = 0;
      big_value = 0;
      rc = BeginRead3dmBigChunk( &tcode, &big_value );
      if ( !rc )
        break;
      if ( tcode == TCODE_SETTINGS_TABLE ) {
        // version 2 model properties
        rc = settings.Read(*this);
      }
      if ( !EndRead3dmChunk() ) {
        rc = false;
        break;
      }
      if ( TCODE_SETTINGS_TABLE == tcode )
        break;
    }
  }

  return rc;
}

ON_BinaryArchive::table_type ON_BinaryArchive::TableTypeFromTypecode( unsigned int typecode )
{
  table_type tt = no_active_table;
  switch(typecode) 
  {
  case TCODE_PROPERTIES_TABLE: tt = properties_table; break;
  case TCODE_SETTINGS_TABLE:   tt = settings_table; break;
  case TCODE_BITMAP_TABLE:     tt = bitmap_table; break;
  case TCODE_TEXTURE_MAPPING_TABLE: tt = texture_mapping_table; break;
  case TCODE_MATERIAL_TABLE:   tt = material_table; break;
  case TCODE_LINETYPE_TABLE:   tt = linetype_table; break;
  case TCODE_LAYER_TABLE:      tt = layer_table; break;
  case TCODE_LIGHT_TABLE:      tt = light_table; break;
  case TCODE_OBJECT_TABLE:     tt = object_table; break;
  case TCODE_GROUP_TABLE:      tt = group_table; break;
  case TCODE_FONT_TABLE:       tt = font_table; break;
  case TCODE_DIMSTYLE_TABLE:   tt = dimstyle_table; break;
  case TCODE_HATCHPATTERN_TABLE: tt = hatchpattern_table; break;
  case TCODE_INSTANCE_DEFINITION_TABLE: tt = instance_definition_table; break;
  case TCODE_HISTORYRECORD_TABLE: tt = historyrecord_table; break;
  case TCODE_USER_TABLE:       tt = user_table; break;
  }
  return tt;
}

bool ON_BinaryArchive::BeginWrite3dmTable( unsigned int typecode )
{
  const table_type tt = TableTypeFromTypecode(typecode);
  if (tt == no_active_table) {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmTable() bad typecode");
    return false;
  }
  if ( m_active_table != no_active_table ) {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmTable() m_active_table != no_active_table");
    return false;
  }
  if ( m_chunk.Count() ) {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmTable() m_chunk.Count() > 0");
    return false;
  }
  bool rc = BeginWrite3dmChunk(typecode,0);
  if (rc)
    m_active_table = tt;
  return rc;
}

bool ON_BinaryArchive::EndWrite3dmTable( unsigned int typecode )
{
  const table_type tt = TableTypeFromTypecode(typecode);
  if (tt == no_active_table) {
    ON_ERROR("ON_BinaryArchive::EndWrite3dmTable() bad typecode");
    return false;
  }
  if ( m_active_table != tt ) {
    ON_ERROR("ON_BinaryArchive::EndWrite3dmTable() m_active_table != t");
    return false;
  }
  if ( m_chunk.Count() != 1 ) {
    ON_ERROR("ON_BinaryArchive::EndWrite3dmTable() m_chunk.Count() != 1");
    return false;
  }
  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( 0 == c || c->m_typecode != typecode ) {
    ON_ERROR("ON_BinaryArchive::EndWrite3dmTable() m_chunk.Last()->typecode != typecode");
    return false;
  }
  bool rc = BeginWrite3dmChunk( TCODE_ENDOFTABLE, 0 );
  if (rc) {
    if (!EndWrite3dmChunk())
      rc = false;
  }
  if (!EndWrite3dmChunk())
    rc = false;
  Flush();
  m_active_table = no_active_table;
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmTable( unsigned int typecode )
{
  bool rc = false;
  const table_type tt = TableTypeFromTypecode(typecode);
  if (tt == no_active_table) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmTable() bad typecode");
    return false;
  }
  if ( m_active_table != no_active_table ) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmTable() m_active_table != no_active_table");
    return false;
  }
  if ( m_chunk.Count() ) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmTable() m_chunk.Count() > 0");
    return false;
  }

  if ( m_3dm_version <= 1 ) 
  {
    if ( TCODE_USER_TABLE == typecode )
      return false; // no user tables in V1 files

    // version 1 files had can have chunks in any order.  To read a "table",
    // you have to go through the entire list of chunks looking for those
    // that belong to a particular table.
    rc = SeekFromStart(32)?true:false;
    m_active_table = tt;
  }
  else 
  {
    if ( TCODE_USER_TABLE == typecode && m_3dm_version <= 2 )
      return false; // no user tables in V2 files

    bool bReadTable = true;
    ON__UINT32 tcode = !typecode;
    ON__INT64 big_value = 0;
    rc = PeekAt3dmBigChunkType( &tcode, &big_value );
    if ( rc ) {
      if ( tcode != typecode ) 
      {
        if ( typecode == TCODE_USER_TABLE ) 
        {
          // it's ok to not have user tables
          rc = false;
          bReadTable = false;
        }
        else if ( typecode == TCODE_GROUP_TABLE && m_3dm_opennurbs_version < 200012210 ) 
        {
          // 3DM archives written before version 200012210 and before do not have group tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_FONT_TABLE && m_3dm_opennurbs_version < 200109180 )
        {
          // 3DM archives written before version 200109180 and before do not have font tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_DIMSTYLE_TABLE && m_3dm_opennurbs_version < 200109260 ) 
        {
          // 3DM archives written before version 200109260 and before do not have dimstyle tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_INSTANCE_DEFINITION_TABLE && m_3dm_opennurbs_version < 200205110 )
        {
          // 3DM archives written before version 200205110 and before do not have instance definition tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_HATCHPATTERN_TABLE && m_3dm_opennurbs_version < 200405030 )
        {
          // 3DM archives written before version 200405030 and before do not have hatch pattern tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_LINETYPE_TABLE && m_3dm_opennurbs_version < 200503170 )
        {
          // 3DM archives written before version 200503170 and before do not have linetype tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_TEXTURE_MAPPING_TABLE && m_3dm_opennurbs_version < 200511110 )
        {
          // 3DM archives written before version 200511110 and before do not have texture mapping tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else if ( typecode == TCODE_HISTORYRECORD_TABLE && m_3dm_opennurbs_version < 200601180 )
        {
          // 3DM archives written before version 200601180 and before do not have history record tables
          rc = true;
          m_active_table = tt;
          bReadTable = false;
        }
        else 
        {
          // A required table is not at the current position in the archive
          // see if we can find it someplace else in the archive.  This can
          // happen when old code encounters a table that was added later.

          bool bSeekFromStart = true;

          if (   TCODE_HATCHPATTERN_TABLE == tcode
              && TCODE_INSTANCE_DEFINITION_TABLE == typecode
              && 3 == m_3dm_version 
              && 200405190 <= m_3dm_opennurbs_version )
          {
            // Dale Lear
            //   V3 files from 19 may 2004 on contained bogus hatch pattern tables
            //   where the instance definition table was supposed to be.
            //
            // Do not set rc in this code.  The goal of this code is to
            // avoid seeking from the start of the file and posting
            // an ON_ERROR alert about something we can work around
            // and has been fixed in V4.
            tcode = 0;
            big_value = 0;
            if ( BeginRead3dmBigChunk( &tcode, &big_value ) )
            {
              if ( TCODE_HATCHPATTERN_TABLE == tcode )
              {
                bSeekFromStart = false;
              }

              if ( !EndRead3dmChunk() )
              {
                bSeekFromStart = true;
              }
              else if ( !bSeekFromStart )
              {
                tcode = 0;
                big_value = 0;
                PeekAt3dmBigChunkType( &tcode, &big_value );
                if ( tcode != typecode )
                  bSeekFromStart = true;
              }
            }
          }

          if ( bSeekFromStart )
          {
            ON_ERROR("ON_BinaryArchive::BeginRead3dmTable() - current file position not at start of table - searching");
            rc = Seek3dmChunkFromStart( typecode );
          }
        }
      }
      if ( rc && bReadTable ) {
        tcode = !typecode;
        big_value = 0;
        rc = BeginRead3dmBigChunk( &tcode, &big_value );
        if ( rc && tcode != typecode ) {
          ON_ERROR("ON_BinaryArchive::BeginRead3dmTable() - corrupt table - skipping");
          rc = false;
          EndRead3dmChunk();
        }
        else if (rc) {
          m_active_table = tt;
        }
      }
    }
  }

  return rc;
}


int ON_BinaryArchive::GetCurrentChunk(ON_3DM_CHUNK& chunk) const
{
  ON_3DM_BIG_CHUNK big_chunk;
  memset(&chunk,0,sizeof(ON_3DM_CHUNK));
  memset(&big_chunk,0,sizeof(big_chunk));
  int rc = GetCurrentChunk(big_chunk);
  if ( rc > 0 )
  {
    chunk.m_offset = (size_t)big_chunk.m_big_offset;
    chunk.m_typecode = big_chunk.m_typecode;

    ON__INT32 i32 = 0;
    if ( ON_IsLongChunkTypecode( big_chunk.m_typecode ) )
      DownSizeUINT( (ON__UINT64)big_chunk.m_big_value, (ON__UINT32*)&i32 );
    else
      DownSizeINT( big_chunk.m_big_value, &i32 );
    chunk.m_value = i32;

    chunk.m_do_length = big_chunk.m_bLongChunk ? 1 : 0;
    chunk.m_do_crc16 = big_chunk.m_do_crc16 ? 1 : 0;
    chunk.m_do_crc32 = big_chunk.m_do_crc32 ? 1 : 0;
    chunk.m_crc16 = big_chunk.m_crc16;
    chunk.m_crc32 = big_chunk.m_crc32;
  }
  return rc;
}


int ON_BinaryArchive::GetCurrentChunk(ON_3DM_BIG_CHUNK& chunk) const
{
  int rc = m_chunk.Count();
  if ( rc > 0 )
  {
    chunk = m_chunk[rc-1];
  }
  else
  {
    memset(&chunk,0,sizeof(ON_3DM_BIG_CHUNK));
  }
  return rc;
}

static
const unsigned char*  BufferToUINT16(
          bool bReverseByteOrder, 
          const unsigned char* buffer,
          const unsigned char* buffer_max,
          ON__UINT16* u16 )
{
  if ( buffer >= buffer_max || buffer_max - buffer < 2 )
    return 0;
  if ( u16 )
  {
    unsigned char* dst = (unsigned char*)u16;
    if ( bReverseByteOrder )
    {
      dst[0] = buffer[1];
      dst[1] = buffer[0];
    }
    else
    {
      dst[0] = buffer[0];
      dst[1] = buffer[1];
    }
  }
  return buffer+2;
}

static
const unsigned char* BufferToUINT32(
          bool bReverseByteOrder,
          const unsigned char* buffer, 
          const unsigned char* buffer_end,
          ON__UINT32* u32 )
{
  if ( buffer >= buffer_end || buffer_end - buffer < 4 )
    return 0;
  if ( u32 )
  {
    unsigned char* dst = (unsigned char*)u32;
    if ( bReverseByteOrder )
    {
      dst[0] = buffer[3];
      dst[1] = buffer[2];
      dst[2] = buffer[1];
      dst[3] = buffer[0];
    }
    else
    {
      dst[0] = buffer[0];
      dst[1] = buffer[1];
      dst[2] = buffer[2];
      dst[3] = buffer[3];
    }
  }
  return buffer+4;
}

static
const unsigned char* BufferToINT64(
          bool bReverseByteOrder, 
          const unsigned char* buffer, 
          const unsigned char* buffer_end,
          ON__INT64* i64 )
{
  if ( buffer >= buffer_end || buffer_end - buffer < 8 )
    return 0;
  if ( i64 )
  {
    unsigned char* dst = (unsigned char*)i64;
    if ( bReverseByteOrder )
    {
      dst[0] = buffer[7];
      dst[1] = buffer[6];
      dst[2] = buffer[5];
      dst[3] = buffer[4];
      dst[4] = buffer[3];
      dst[5] = buffer[2];
      dst[6] = buffer[1];
      dst[7] = buffer[0];
    }
    else
    {
      dst[0] = buffer[0];
      dst[1] = buffer[1];
      dst[2] = buffer[2];
      dst[3] = buffer[3];
      dst[4] = buffer[4];
      dst[5] = buffer[5];
      dst[6] = buffer[6];
      dst[7] = buffer[7];
    }
  }
  return buffer+8;
}

static
const unsigned char* BufferValidateTcode( 
          bool bReverseByteOrder,
          const unsigned char* buffer, 
          const unsigned char* buffer_end,
          ON__UINT32 expected_tcode )
{
  ON__UINT32 tc = expected_tcode ? 0 : 1;
  buffer = BufferToUINT32(bReverseByteOrder,buffer,buffer_end,&tc);
  return ( 0 != buffer && tc == expected_tcode ) ? buffer : 0;
}

static 
const unsigned char* BufferToChunkValue( 
          bool bReverseByteOrder,
          size_t sizeof_chunk_value, 
          const unsigned char* buffer, 
          const unsigned char* buffer_end,
          ON__INT64* chunk_value )
{
  if ( 8 == sizeof_chunk_value )
  {
    buffer = BufferToINT64(bReverseByteOrder,buffer,buffer_end,chunk_value);
  }
  else
  {
    ON__UINT32 u32;
    ON__UINT64 u64;
    buffer = BufferToUINT32(bReverseByteOrder,buffer,buffer_end,&u32);
    if ( buffer && chunk_value )
    {
      // this u64 = u32 is here so 4 byte unsigned ints with the high
      // bit set are converted to positive 8 bytes ints.
      u64 = u32; 
      *chunk_value = (ON__INT64)u64;
    }
  }
  return buffer;
}

static 
const unsigned char* BufferToUuid(
          bool bReverseByteOrder, 
          const unsigned char* buffer,
          const unsigned char* buffer_end,
          ON_UUID& uuid )
{
  ON__UINT32 data1=0;
  ON__UINT16 data2=0, data3=0;
  if ( buffer >= buffer_end || buffer_end - buffer < 16 )
    return 0;
  buffer = BufferToUINT32(bReverseByteOrder,buffer,buffer_end,&data1);
  if (buffer)
    buffer = BufferToUINT16(bReverseByteOrder,buffer,buffer_end,&data2);
  if (buffer)
    buffer = BufferToUINT16(bReverseByteOrder,buffer,buffer_end,&data3);
  if (buffer)
  {
    if ( buffer >= buffer_end || buffer_end - buffer < 8 )
      buffer = 0;
    else
    {
      uuid.Data1 = data1;
      uuid.Data2 = data2;
      uuid.Data3 = data3;
      memcpy(&uuid.Data4,buffer,8);
      buffer += 8;
    }
  }
  return buffer;
}

static
const unsigned char* EmergencyFindTable_UuidHelper( 
          bool bReverseByteOrder,
          size_t sizeof_chunk_value, 
          const unsigned char* buffer,
          const unsigned char* buffer_end,
          const ON__UINT32 expected_tcode,
          const ON_UUID* expected_uuid
          )
{
  ON__INT64 chunk_value;
  ON__UINT32 c, cc;
  ON_UUID uuid;

  // make sure the value at the start of the buffer = expected_tcode.
  buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,expected_tcode);
  if ( 0 == buffer )
    return 0;

  // get length of this chunk containing the uuid
  chunk_value = -1;
  buffer = BufferToChunkValue( bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,&chunk_value );
  if ( 0 == buffer || chunk_value < 0 )
    return 0;

  // determine how long the chunk is supposed to be and validate the length.
  //
  // The "16+4+sizeof_chunk_value+21+4+4" in the bLookForUserTableRecordHeader
  // breaks down as:
  //  16                       sizeof(uuid)
  //  +4                     + sizeof(TCODE_USER_TABLE_RECORD_HEADER chunk typecode)
  //  +sizeof_chunk_value    + sizeof(TCODE_USER_TABLE_RECORD_HEADER chunk length)
  //  +21                    + major ver, minor ver, bool, archive ver, 3dm ver
  //  +4                     + sizeof(TCODE_USER_TABLE_RECORD_HEADER chunk crc)
  //  +4                     + sizeof(TCODE_USER_TABLE_UUID chunk crc)
  const bool bLookForUserTableRecordHeader = (TCODE_USER_TABLE_UUID == expected_tcode
                                              && ((ON__UINT64)chunk_value) >= (16+4+sizeof_chunk_value+21+4+4)
                                             );
  if ( !bLookForUserTableRecordHeader && 20 != chunk_value )
    return 0;
  buffer = BufferToUuid(bReverseByteOrder,buffer,buffer_end,uuid);
  if ( 0 == buffer )
    return 0;
  if( 0 != expected_uuid && uuid != *expected_uuid )
    return 0;
  
  if ( bLookForUserTableRecordHeader )
  {
    // make sure there is a TCODE_USER_TABLE_RECORD_HEADER chunk and skip over it.
    buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,TCODE_USER_TABLE_RECORD_HEADER);
    if ( 0 == buffer )
      return 0;
    // get length of the TCODE_USER_TABLE_RECORD_HEADER chunk
    ON__INT64 header_length = -1;
    buffer = BufferToChunkValue( bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,&header_length );
    if ( 0 == buffer )
      return 0;
    if ( header_length < 25 )
      return 0;
    if ( buffer >= buffer_end || buffer_end - buffer < header_length )
      return 0;
    buffer += header_length;
  }

  buffer = BufferToUINT32(bReverseByteOrder,buffer,buffer_end,&c);
  if ( 0 == buffer )
    return 0;
  cc = ON_CRC32(0,4,&uuid.Data1);
  cc = ON_CRC32(cc,2,&uuid.Data2);
  cc = ON_CRC32(cc,2,&uuid.Data3);
  cc = ON_CRC32(cc,8,&uuid.Data4[0]);
  if ( c != cc )
    return 0;

  return buffer;
}


bool ON_BinaryArchive::FindMisplacedTable( 
        ON__UINT64 filelength,
        const ON__UINT32 table_tcode,
        const ON__UINT32 table_record_tcode,
        const ON_UUID class_uuid,
        const ON__UINT64 min_length_data
        )
{
  bool rc = false;
  unsigned char buffer2048[2048];
  const ON__UINT64 pos0 = CurrentPosition();
  if ( filelength > 0 && pos0 >= filelength )
    return false;

  ON__UINT32 tcode;
  ON__INT64 i64;

  const bool bReverseByteOrder = (ON::big_endian == Endian());
  const size_t sizeof_chunk_typecode = 4;
  const size_t sizeof_chunk_value = SizeofChunkLength();
  const size_t sizeof_chunk_header = sizeof_chunk_typecode + sizeof_chunk_value;
  size_t length_of_user_uuid_and_header = 0;
  const bool bFindObjectTable    = (  TCODE_OBJECT_TABLE == table_tcode
                                   && TCODE_OBJECT_RECORD == table_record_tcode );
  const bool bFindUserTable    = (    TCODE_USER_TABLE == table_tcode
                                   && TCODE_USER_RECORD == table_record_tcode );

  if ( TCODE_USER_TABLE == table_tcode && !bFindUserTable )
    return false;
  if ( TCODE_OBJECT_TABLE == table_tcode && !bFindObjectTable )
    return false;
  if ( bFindUserTable && ON_UuidIsNil(class_uuid) )
  {
    // must provide plug-in id when searching for user tables
    ON_ERROR("ON_BinaryArchive::FindMisplacedTable - must provide plug-in id when searching for user tables");
    return false;
  }

  if ( !SeekFromStart(0) )
    return false;

  ON__UINT64 pos1 = CurrentPosition();
  ON__UINT64 pos;
  ON__UINT64 empty_table_pos = 0;    // position of first empty table candidate
  int        empty_table_status = 0; // 1 = found a candidate for an empty table
                                     // 2 = found 2 or more candidates

  const size_t sizeof_buffer2048 = sizeof(buffer2048);
  bool bAtEOF = false;

  while(!bAtEOF)
  {
    pos = CurrentPosition();
    if ( pos < pos1 )
    {
      break;
    }
    else if ( pos > pos1 )
    {
      if ( !BigSeekBackward(pos - pos1) )
        break;
      if ( pos1 != CurrentPosition() )
        break;
    }

    memset(buffer2048,0,sizeof_buffer2048);
    // Depending on the table and the version of the file, less than 
    // sizeof_buffer128 bytes may be read.  Setting bit 0x04 of
    // m_error_message_mask disables calls to ON_Error when we
    // attempt to read beyond the end of file.
    const unsigned int saved_error_message_mask = m_error_message_mask;
    m_error_message_mask |= 0x04;
    const size_t sizeof_read = Read(sizeof_buffer2048,buffer2048);
    m_error_message_mask = saved_error_message_mask;
    if ( sizeof_read < sizeof_buffer2048 )
    {
      // we need to parse what was read, but there's nothing after this.
      bAtEOF = true;
    }
    if ( sizeof_read < 2*sizeof_chunk_header || sizeof_read > sizeof_buffer2048 )
      break;
    const unsigned char* buffer_end = (&buffer2048[0]) + sizeof_read;
    const unsigned char* buffer = buffer2048;

    pos1++;

    // "read" 4 byte tcode
    tcode = !table_tcode;
    buffer = BufferToUINT32(bReverseByteOrder,buffer,buffer_end,&tcode);
    if ( 0 == buffer )
      break;

    if ( table_tcode != tcode )
    {
      // This for loop looks through the buffer we just
      // read to reduce the amount of times we seek backwards
      // and re-read.
      for ( size_t i = 1; i <= sizeof_read - sizeof_chunk_typecode; i++ )
      {
        tcode = !table_tcode;
        buffer = BufferToUINT32(bReverseByteOrder,&buffer2048[i],buffer_end,&tcode);
        if ( 0 == buffer || table_tcode == tcode )
        {
          if ( bAtEOF && sizeof_read > 0 && 0 != buffer && table_tcode == tcode )
          {
            // this table starts within sizeof_buffer2048 bytes of the eof.
            bAtEOF = false;
          }
          break;
        }
        pos1++;
      }
      continue; // start again with archive positioned at the tcode we want
    }

    // "read" 4 or 8 byte chunk value
    i64 = -1;
    buffer = BufferToChunkValue(bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,&i64);
    if ( 0 == buffer || i64 <= 0 )
      continue;
    const ON__UINT64 length_of_table = (ON__UINT64)i64;

    if ( length_of_table < 2*sizeof_chunk_header + 4 + min_length_data )
    {
      if ( sizeof_chunk_header == length_of_table && 2 != empty_table_status )
      {
        // see if we are at a TCODE_ENDOFTABLE chunk
        buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,TCODE_ENDOFTABLE);
        if ( 0 != buffer )
        {
          i64 = -1;
          buffer = BufferToChunkValue(bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,&i64);
          if ( 0 == i64 )
          {
            if ( 0 == empty_table_status )
            {
              empty_table_pos = pos1-1;
              empty_table_status = 1;
            }
            else
            {
              // found 2 or more candidates for the end of table chunk
              empty_table_status = 2;
            }
          }
        }
      }
      continue;
    }

    if ( bFindUserTable )
    {
      // We found TCODE_USER_TABLE + chunk length.  If it is a user table,
      // there should be a TCODE_USER_TABLE_UUID chunk with a crc.
      const unsigned char* buffer0 = buffer;
      buffer = EmergencyFindTable_UuidHelper(bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,TCODE_USER_TABLE_UUID,&class_uuid);
      if ( 0 == buffer || buffer <= buffer0 )
        continue;

      length_of_user_uuid_and_header = buffer - buffer0;
      // At this point we should be postioned at the table_record_tcode = TCODE_USER_RECORD chunk
    }

    // see if the start of the buffer contains the 4 byte typecode value = table_record_tcode.
    buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,table_record_tcode);
    if ( 0 == buffer )
      continue;
    i64 = -1;
    buffer = BufferToChunkValue( bReverseByteOrder, sizeof_chunk_value,buffer,buffer_end,&i64);
    if ( 0 == buffer || i64 <= 0 )
      continue;
    const ON__UINT64 length_of_record = (ON__UINT64)i64;


    if ( bFindUserTable )
    {
      ON__UINT64 expected_length_of_table = length_of_user_uuid_and_header 
                                          + sizeof_chunk_header
                                          + length_of_record;
      if ( expected_length_of_table != length_of_table )
        continue;
    }
    else
    {
      if ( length_of_record < 4*sizeof_chunk_header + 20 + min_length_data )
        continue;
      if ( length_of_record + 2*sizeof_chunk_header > length_of_table)
        continue;

      if (bFindObjectTable)
      {
        buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,TCODE_OBJECT_RECORD_TYPE);
        if ( 0 == buffer )
          continue;
        // The TCODE_OBJECT_RECORD_TYPE is a shot chunk whose value is a bitfield 
        // used to filter reading of objects.  Checking the value will not help
        // validate the record, but we need to skip over it.
        buffer = BufferToChunkValue(bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,0);
        if ( 0 == buffer )
          continue;
      }

      buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,TCODE_OPENNURBS_CLASS);
      if ( 0 == buffer )
        continue;

      i64 = -1;
      buffer = BufferToChunkValue( bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,&i64);
      if ( 0 == buffer || i64 <= 0 )
        continue;
      const ON__UINT64 length_of_on_class = (ON__UINT64)i64;

      if ( length_of_on_class < 3*sizeof_chunk_header + 20 + min_length_data )
        continue;

      if ( length_of_on_class + sizeof_chunk_header + 4 > length_of_record)
        continue;

      const unsigned char* buffer0 = buffer;
      buffer = EmergencyFindTable_UuidHelper(bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,TCODE_OPENNURBS_CLASS_UUID,(ON_UuidIsNil(class_uuid) ? NULL : &class_uuid));
      if ( 0 == buffer || buffer <= buffer0)
        continue;
      const size_t length_of_uuid_chunk = buffer-buffer0;

      buffer = BufferValidateTcode(bReverseByteOrder,buffer,buffer_end,TCODE_OPENNURBS_CLASS_DATA);
      if ( 0 == buffer )
        continue;

      i64 = -1;
      buffer = BufferToChunkValue( bReverseByteOrder,sizeof_chunk_value,buffer,buffer_end,&i64);
      if ( 0 == buffer || i64 < 0 )
        continue;
      const ON__UINT64 length_of_data = (ON__UINT64)i64;

      if ( length_of_data < min_length_data )
        continue;
      if ( length_of_data + length_of_uuid_chunk + 2*sizeof_chunk_header > length_of_on_class)
        continue;
    }

    // position archive at point where the table tcode was read
    if ( !BigSeekBackward(sizeof_read) )
      break;
    pos = CurrentPosition();
    if ( pos+1 == pos1)
      rc = true;
    break;
  }

  if ( !rc )
  {
    // we didn't fing a table containing anything
    if ( 1 == empty_table_status )
    {
      // we found one candidate for an empty table.
      // This is reasonable for materials, bitmaps, and the like.
      rc = BigSeekFromStart(empty_table_pos);
    }
    else
    {
      // nothing in this file.
      BigSeekFromStart(pos0);
    }
  }
  return rc;
}

bool ON_BinaryArchive::FindTableInDamagedArchive(
                const unsigned int tcode_table,
                const unsigned int tcode_record,
                const ON_UUID class_uuid,
                const int min_length_data
                )
{
  bool rc = FindMisplacedTable(
                  0,
                  tcode_table,
                  tcode_record,
                  class_uuid,
                  min_length_data
                  );
  return rc;
}

/*
static
bool FindMaterialTable( ON_BinaryArchive& binary_archive, size_t filelength )
{
  bool rc = EmergencyFindTable( 
                binary_archive, filelength,
                TCODE_MATERIAL_TABLE, TCODE_MATERIAL_RECORD,
                ON_Material::m_ON_Material_class_id.Uuid(),
                114
                );
  return rc;
}
*/

bool ON_BinaryArchive::EndRead3dmTable( unsigned int typecode )
{
  bool rc = false;
  const table_type tt = TableTypeFromTypecode(typecode);
  if (tt == no_active_table) {
    ON_ERROR("ON_BinaryArchive::EndRead3dmTable() bad typecode");
    return false;
  }
  if ( m_active_table != tt ) {
    ON_ERROR("ON_BinaryArchive::EndRead3dmTable() m_active_table != t");
    return false;
  }
  if ( m_3dm_version == 1 ) {
    if ( m_chunk.Count() != 0 ) {
      ON_ERROR("ON_BinaryArchive::EndRead3dmTable() v1 file m_chunk.Count() != 0");
      return false;
    }
    rc = true;
  }
  else {
    if ( m_active_table == group_table && m_3dm_opennurbs_version < 200012210 ) 
    {
      // 3DM archives written before version 200012210 and before do not have group tables
      rc = true;
    }
    else if ( m_active_table == font_table && m_3dm_opennurbs_version < 200109180 ) 
    {
      // 3DM archives written before version 200109180 and before do not have font tables
      rc = true;
    }
    else if ( m_active_table == dimstyle_table && m_3dm_opennurbs_version < 200109260 ) 
    {
      // 3DM archives written before version 200109260 and before do not have dimstyle tables
      rc = true;
    }
    else if ( m_active_table == instance_definition_table && m_3dm_opennurbs_version < 200205110 ) 
    {
      // 3DM archives written before version 200205110 and before do not have instance definition tables
      rc = true;
    }
    else if ( m_active_table == hatchpattern_table && m_3dm_opennurbs_version < 200405030 ) 
    {
      // 3DM archives written before version 200405030 and before do not have hatch pattern tables
      rc = true;
    }
    else if ( m_active_table == linetype_table && m_3dm_opennurbs_version < 200503170 ) 
    {
      // 3DM archives written before version 200503170 and before do not have linetype tables
      rc = true;
    }
    else if ( m_active_table == texture_mapping_table && m_3dm_opennurbs_version < 200511110 )
    {
      // 3DM archives written before version 200511110 and before do not have texture mapping tables
      rc = true;
    }
    else if ( m_active_table == historyrecord_table && m_3dm_opennurbs_version < 200601180 )
    {
      // 3DM archives written before version 200601180 and before do not have history record tables
      rc = true;
    }
    else 
    {
      if ( m_chunk.Count() != 1 ) 
      {
        ON_ERROR("ON_BinaryArchive::EndRead3dmTable() v2 file m_chunk.Count() != 1");
        return false;
      }
      const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
      if ( 0 == c || c->m_typecode != typecode ) 
      {
        ON_ERROR("ON_BinaryArchive::EndRead3dmTable() m_chunk.Last()->typecode != typecode");
        return false;
      }
      rc = EndRead3dmChunk();
    }
  }
  m_active_table = no_active_table;
  return rc;
}

bool ON_BinaryArchive::BeginWrite3dmBitmapTable()
{
  return BeginWrite3dmTable( TCODE_BITMAP_TABLE );
}

bool ON_BinaryArchive::EndWrite3dmBitmapTable()
{
  return EndWrite3dmTable( TCODE_BITMAP_TABLE );
}

bool ON_BinaryArchive::Write3dmBitmap( const ON_Bitmap& bitmap )
{
  bool rc = false;
  if ( m_3dm_version != 1 ) 
  {
    const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
    if ( c && c->m_typecode == TCODE_BITMAP_TABLE ) 
    {
      rc = BeginWrite3dmChunk( TCODE_BITMAP_RECORD, 0 );
      if ( rc ) 
      {
        rc = WriteObject( bitmap );
        if ( !EndWrite3dmChunk() )
          rc = false;
      }
    }
    else 
    {
      ON_ERROR("ON_BinaryArchive::Write3dmBitmap() must be called in BeginWrite3dmBitmapTable() block");
      rc = false;
    }
  }
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmBitmapTable()
{
  bool rc =  BeginRead3dmTable( TCODE_BITMAP_TABLE );
  if ( !rc )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions when it makes sense.
    rc = FindMisplacedTable( 
                0,
                TCODE_BITMAP_TABLE, TCODE_BITMAP_RECORD,
                ON_nil_uuid, // multiple types of opennurbs objects in bitmap tables
                40 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_BITMAP_TABLE );
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndRead3dmBitmapTable()
{
  return EndRead3dmTable( TCODE_BITMAP_TABLE );
}

int ON_BinaryArchive::Read3dmBitmap(  // returns 0 at end of bitmap table
                                      //         1 bitmap successfully read
          ON_Bitmap** ppBitmap // bitmap returned here
          )
{
  if ( ppBitmap )
    *ppBitmap = 0;
  ON_Bitmap* bitmap = 0;
  int rc = 0;
  if ( m_3dm_version != 1 ) {
    ON__UINT32 tcode = 0;
    ON__INT64 big_value = 0;
    if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) 
    {
      if ( tcode == TCODE_BITMAP_RECORD ) 
      {
        ON_Object* p = 0;
        if ( ReadObject( &p ) ) 
        {
          bitmap = ON_Bitmap::Cast(p);
          if ( !bitmap )
            delete p;
          else
            rc = 1;
        }
        if (!bitmap) 
        {
          ON_ERROR("ON_BinaryArchive::Read3dmBitmap() - corrupt bitmap table");
        }
        if ( ppBitmap )
          *ppBitmap = bitmap;
        else if ( bitmap )
          delete bitmap;          
      }
      else if ( tcode != TCODE_ENDOFTABLE )
      {
        ON_ERROR("ON_BinaryArchive::Read3dmBitmap() - corrupt bitmap table");
      }
      EndRead3dmChunk();
    }
  }

  return rc;
}


bool ON_BinaryArchive::BeginWrite3dmLayerTable()
{
  bool rc = false;
  if ( m_3dm_version != 1 ) {
    rc = BeginWrite3dmTable( TCODE_LAYER_TABLE );
  }
  else {
    if ( m_chunk.Count() ) {
      ON_ERROR("ON_BinaryArchive::BeginWrite3dmLayerTable() - chunk stack should be empty");
      return false;
    }
    if ( m_active_table != no_active_table ) {
      ON_ERROR("ON_BinaryArchive::BeginWrite3dmLayerTable() - m_active_table != no_active_table");
    }
    m_active_table = layer_table;
    rc = true;
  }

  return rc;
}

bool ON_BinaryArchive::Write3dmLayer( const ON_Layer&  layer )
{
  bool rc = false;
  if ( m_active_table != layer_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmLayer() - m_active_table != layer_table");
  }

  if ( m_3dm_version == 1 ) {
    // legacy version 1 layer information is in a top level TCODE_LAYER chunk
    if ( m_chunk.Count() ) {
      ON_ERROR("ON_BinaryArchive::Write3dmLayer() - version 1 - chunk stack should be empty");
      return false;
    }
    ON_String s = layer.LayerName();
    if ( !s.IsEmpty() ) {
      rc = BeginWrite3dmChunk( TCODE_LAYER, 0 );

      // layer name
      if (rc) {
        rc = BeginWrite3dmChunk( TCODE_LAYERNAME, 0 );
        if(rc) rc = WriteString(s);
        if (!EndWrite3dmChunk())
          rc = false;
      }

      // layer color
      if (rc) {
        rc = BeginWrite3dmChunk( TCODE_RGB, layer.Color() );
        if (!EndWrite3dmChunk())
          rc = false;
      }

      // layer mode normal=0/hidden=1/locked=2
      if (rc) 
      {
        int mode;
        if ( layer.IsLocked() )
          mode = 2; // "locked"
        else if ( layer.IsVisible() )
          mode = 0; // "normal"
        else
          mode = 1; // "hidden"
        rc = BeginWrite3dmChunk( TCODE_LAYERSTATE, mode );
        if (!EndWrite3dmChunk())
          rc = false;
      }

      if ( !BeginWrite3dmChunk( TCODE_ENDOFTABLE, 0 ) )
        rc = false;
      if ( !EndWrite3dmChunk() )
        rc = false;

      if (!EndWrite3dmChunk()) // end of TCODE_LAYER chunk
        rc = false;
    }
  }
  else {
    // version 2+
    const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
    if ( c && c->m_typecode == TCODE_LAYER_TABLE ) {
      rc = BeginWrite3dmChunk( TCODE_LAYER_RECORD, 0 );
      if ( rc ) {
        rc = WriteObject( layer );
        if ( !EndWrite3dmChunk() )
          rc = false;
      }
    }
    else {
      ON_ERROR("ON_BinaryArchive::Write3dmLayer() must be called in BeginWrite3dmLayerTable(2) block");
      rc = false;
    }
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmLayerTable()
{
  bool rc = false;
  if ( m_3dm_version == 1 ) {
    if ( m_active_table != layer_table ) {
      ON_ERROR("ON_BinaryArchive::EndWrite3dmLayerTable() - m_active_table != layer_table");
    }
    rc = true;
    m_active_table = no_active_table;
  }
  else {
    rc = EndWrite3dmTable( TCODE_LAYER_TABLE );
  }
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmLayerTable()
{
  bool rc = false;
  m_3dm_v1_layer_index = 0;
  rc = BeginRead3dmTable( TCODE_LAYER_TABLE );
  if ( !rc )
  {
    // 8 October 2004 Dale Lear
    //    This fall back is slow but it will find
    //    layer tables in files that have been damaged.
    rc = FindMisplacedTable( 
                0,
                TCODE_LAYER_TABLE, TCODE_LAYER_RECORD,
                ON_Layer::m_ON_Layer_class_id.Uuid(),
                30
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_LAYER_TABLE );
    }

  }
  else if ( rc && m_3dm_version == 1 ) {
    rc = Seek3dmChunkFromStart( TCODE_LAYER );
    rc = true; // there are 1.0 files written by the old IO toolkit that have no layers
  }
  return rc;
}

int ON_BinaryArchive::Read3dmV1LayerIndex(const char* sV1LayerName) const
{
  // returns V1 layer index

  int layer_index = -1;

  if (    ON::read3dm == m_mode
       && 0 == m_3dm_opennurbs_version
       && 1 == m_3dm_version
       && 0 != m_V1_layer_list
       && 0 != sV1LayerName 
       && 0 != sV1LayerName[0] 
     )
  {
    struct ON__3dmV1LayerIndex* p = m_V1_layer_list;
    int i;
    for ( i = 0; 0 != p && i < 1000; i++ )
    {
      if ( p->m_layer_index < 0 )
        break;
      if ( p->m_layer_name_length < 1 || p->m_layer_name_length>256)
        break;
      if ( 0 == p->m_layer_name )
        break;
      if ( 0 == p->m_layer_name[0] )
        break;
      if ( 0 != p->m_layer_name[p->m_layer_name_length] )
        break;
      if ( !on_stricmp(p->m_layer_name,sV1LayerName) )
      {
        layer_index = p->m_layer_index;
        break;
      }
      p = p->m_next;
    }
  }

  return layer_index;
}

bool ON_BinaryArchive::Read3dmV1Layer( ON_Layer*& layer )
{
  ON_String s;
  bool rc = 0;
  ON__UINT32 tcode;
  ON__INT64 big_value;
  for (;;) 
  {
    tcode = 0;
    big_value = 0;
    if (!BeginRead3dmBigChunk(&tcode,&big_value))
      break; // assume we are at the end of the file
    if ( tcode == TCODE_LAYER ) {
      layer = new ON_Layer();
      layer->SetLayerIndex(m_3dm_v1_layer_index++);
      rc = 1;
      break;
    }
    if (!EndRead3dmChunk())
       break;
  }
  if ( layer ) {
    rc = false;
    for (;;) 
    {
      tcode = 0;
      big_value = 0;
      if (!BeginRead3dmBigChunk(&tcode,&big_value))
        break;
      switch(tcode) 
      {
      case TCODE_LAYERNAME:
        {
          int slen = 0;
          ReadInt(&slen);
          if ( slen < 0 || slen > 10000 )
          {
            ON_ERROR("ON_BinaryArchive::Read3dmV1Layer() - invalid layer name length");
          }
          else
          {
            s.SetLength(slen);
            if ( ReadByte( s.Length(), s.Array() ) ) 
            {
              layer->SetLayerName(s);
            }
          }
        }
        break;
      case TCODE_RGB:
        {
          ON__UINT64 rgb64 = (ON__UINT64)big_value;
          ON__UINT32 rgb32 = (ON__UINT32)rgb64;
          layer->SetColor( ON_Color((ON__UINT32)rgb32) );
        }
        break;
      case TCODE_LAYERSTATE:
        switch (big_value) 
        {
        case 1: // hidden
          layer->SetVisible(false);
          layer->SetLocked(false);
          break;
        case 2: // locked
          layer->SetVisible(true);
          layer->SetLocked(true);
          break;
        default: // normal
          layer->SetVisible(true);
          layer->SetLocked(false);
          break;
        }
        break;
      }
      if (!EndRead3dmChunk())
         break;
      if ( TCODE_ENDOFTABLE == tcode ) {
        rc = true;
        break;
      }
    }
    if ( !EndRead3dmChunk() ) // end of TCODE_LAYER chunk
      rc = false;
  }
  if ( !rc && layer ) 
  {
    delete layer;
    layer = 0;
  }
  else if (rc && layer)
  {
    if (    ON::read3dm == m_mode
         && 0 == m_3dm_opennurbs_version
         && 1 == m_3dm_version
         )
    {
      // save layer index and name in a linked list.
      int s_length = s.Length();
      const char* s_name = s.Array();
      if (    layer->LayerIndex() >= 0 
           && s_length > 0 
           && s_length < 256 
           && 0 != s_name 
           && 0 != s_name[0] 
         )
      {
        struct ON__3dmV1LayerIndex* p = (struct ON__3dmV1LayerIndex*)oncalloc(1, sizeof(*p) + (s_length+1)*sizeof(*p->m_layer_name) );
        p->m_layer_name = (char*)(p+1);
        p->m_layer_index = layer->LayerIndex();
        p->m_layer_name_length = s_length;
        memcpy(p->m_layer_name,s_name,s_length*sizeof(*p->m_layer_name));
        p->m_layer_name[s_length] = 0;
        p->m_next = m_V1_layer_list;
        m_V1_layer_list = p;
      }
    }
  }
  return rc;
}

int ON_BinaryArchive::Read3dmLayer( ON_Layer** ppLayer )
{
  if ( !ppLayer )
    return 0;
  *ppLayer = 0;
  if ( m_active_table != layer_table ) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmLayerTable() - m_active_table != no_active_table");
  }
  ON__UINT32 tcode;
  ON__INT64 big_value;
  ON_Layer* layer = NULL;
  // returns 0 at end of layer table
  if ( m_3dm_version == 1 ) {
    Read3dmV1Layer(layer);
  }
  else {
    // version 2+
    tcode = 0;
    big_value = 0;
    if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) {
      if ( tcode == TCODE_LAYER_RECORD ) {
        ON_Object* p = 0;
        if ( ReadObject( &p ) ) {
          layer = ON_Layer::Cast(p);
          if ( !layer )
            delete p;
        }
        if (!layer) {
          ON_ERROR("ON_BinaryArchive::Read3dmLayer() - corrupt layer table");
        }
      }
      else if ( tcode != TCODE_ENDOFTABLE ) {
        ON_ERROR("ON_BinaryArchive::Read3dmLayer() - corrupt layer table");
      }
      EndRead3dmChunk();
    }
  }
  if ( layer )
    layer->HasPerViewportSettings(ON_nil_uuid); // this call sets ON_Layer::m__runtime_flags
  *ppLayer = layer;
  return (layer) ? 1 : 0;
}

bool ON_BinaryArchive::EndRead3dmLayerTable()
{
  bool rc = false;
  if ( m_3dm_version == 1 ) {
    if ( m_active_table != layer_table ) {
      ON_ERROR("ON_BinaryArchive::EndRead3dmLayerTable() - m_active_table != no_active_table");
      rc = false;
    }
    else if ( m_chunk.Count() ) {
      ON_ERROR("ON_BinaryArchive::EndRead3dmLayerTable() - m_chunk.Count() > 0");
      rc = false;
    }
    else {
      // rewind to start of chunks
      rc = SeekFromStart(32)?true:false;
    }
    m_active_table = no_active_table;
  }
  else {
    rc = EndRead3dmTable( TCODE_LAYER_TABLE );
  }
  return rc;
}


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmGroupTable()
{
  bool rc = false;
  rc = BeginWrite3dmTable( TCODE_GROUP_TABLE );
  return rc;
}

bool ON_BinaryArchive::Write3dmGroup( const ON_Group&  group )
{
  bool rc = false;
  if ( m_active_table != group_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmGroup() - m_active_table != group_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_GROUP_TABLE ) {
    rc = BeginWrite3dmChunk( TCODE_GROUP_RECORD, 0 );
    if ( rc ) {
      rc = WriteObject( group );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  else {
    ON_ERROR("ON_BinaryArchive::Write3dmGroup() must be called in BeginWrite3dmGroupTable() block");
    rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmGroupTable()
{
  bool rc = false;
  rc = EndWrite3dmTable( TCODE_GROUP_TABLE );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmGroupTable()
{
  if ( m_3dm_version == 1 ) {
    return true;
  }
  bool rc = false;
  rc = BeginRead3dmTable( TCODE_GROUP_TABLE );

  if ( !rc )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions when it makes sense.
    rc = FindMisplacedTable( 
                0,
                TCODE_GROUP_TABLE, TCODE_GROUP_RECORD,
                ON_Group::m_ON_Group_class_id.Uuid(),
                20 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_GROUP_TABLE );
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmGroup( ON_Group** ppGroup )
{
  if ( !ppGroup )
    return 0;
  *ppGroup = 0;
  if ( m_3dm_version == 1 ) {
    return 0;
  }
  if ( m_active_table != group_table ) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmGroupTable() - m_active_table != no_active_table");
  }
  if ( m_3dm_opennurbs_version < 200012210 ) {
    // 3DM archives written before version 200012210 and before do not have group tables
    return 0;
  }

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_Group* group = NULL;
  if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) 
  {
    if ( tcode == TCODE_GROUP_RECORD ) {
      ON_Object* p = 0;
      if ( ReadObject( &p ) ) {
        group = ON_Group::Cast(p);
        if ( !group )
          delete p;
      }
      if (!group) {
        ON_ERROR("ON_BinaryArchive::Read3dmGroup() - corrupt group table");
      }
    }
    else if ( tcode != TCODE_ENDOFTABLE ) {
      ON_ERROR("ON_BinaryArchive::Read3dmGroup() - corrupt group table");
    }
    EndRead3dmChunk();
  }
  *ppGroup = group;
  return (group) ? 1 : 0;
}

bool ON_BinaryArchive::EndRead3dmGroupTable()
{
  bool rc = false;
  if ( m_3dm_version == 1 ) {
    return true;
  }
  else {
    rc = EndRead3dmTable( TCODE_GROUP_TABLE );
  }
  return rc;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmFontTable()
{
  bool rc = false;
  rc = BeginWrite3dmTable( TCODE_FONT_TABLE );
  return rc;
}

bool ON_BinaryArchive::Write3dmFont( const ON_Font&  font )
{
  bool rc = false;
  if ( m_active_table != font_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmFont() - m_active_table != font_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_FONT_TABLE ) {
    rc = BeginWrite3dmChunk( TCODE_FONT_RECORD, 0 );
    if ( rc ) {
      rc = WriteObject( font );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  else {
    ON_ERROR("ON_BinaryArchive::Write3dmFont() must be called in BeginWrite3dmFontTable() block");
    rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmFontTable()
{
  bool rc = false;
  rc = EndWrite3dmTable( TCODE_FONT_TABLE );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmFontTable()
{
  if ( m_3dm_version <= 2 ) {
    return true;
  }
  bool rc = false;
  rc = BeginRead3dmTable( TCODE_FONT_TABLE );

  if ( !rc )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions FindMisplacedTable it makes sense.
    rc = FindMisplacedTable( 
                0,
                TCODE_FONT_TABLE, TCODE_FONT_RECORD,
                ON_Font::m_ON_Font_class_id.Uuid(),
                30 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_FONT_TABLE );
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmFont( ON_Font** ppFont )
{
  if ( !ppFont )
    return 0;
  *ppFont = 0;
  if ( m_3dm_version <= 2 ) {
    return 0;
  }
  if ( m_active_table != font_table ) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmFontTable() - m_active_table != no_active_table");
  }
  if ( m_3dm_opennurbs_version < 200109180 ) {
    // 3DM archives written before version 200109180 and before do not have font tables
    return 0;
  }

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_Font* font = NULL;
  if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) 
  {
    if ( tcode == TCODE_FONT_RECORD ) {
      ON_Object* p = 0;
      if ( ReadObject( &p ) ) {
        font = ON_Font::Cast(p);
        if ( !font )
          delete p;
      }
      if (!font) {
        ON_ERROR("ON_BinaryArchive::Read3dmFont() - corrupt font table");
      }
    }
    else if ( tcode != TCODE_ENDOFTABLE ) {
      ON_ERROR("ON_BinaryArchive::Read3dmFont() - corrupt font table");
    }
    EndRead3dmChunk();
  }
  *ppFont = font;
  return (font) ? 1 : 0;
}

bool ON_BinaryArchive::EndRead3dmFontTable()
{
  bool rc = false;
  if ( m_3dm_version <= 2 ) {
    return true;
  }
  else {
    rc = EndRead3dmTable( TCODE_FONT_TABLE );
  }
  return rc;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmDimStyleTable()
{
  bool rc = false;
  rc = BeginWrite3dmTable( TCODE_DIMSTYLE_TABLE );
  return rc;
}

bool ON_BinaryArchive::Write3dmDimStyle( const ON_DimStyle&  dimstyle )
{
  bool rc = false;
  if ( m_active_table != dimstyle_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmDimStyle() - m_active_table != dimstyle_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_DIMSTYLE_TABLE ) {
    rc = BeginWrite3dmChunk( TCODE_DIMSTYLE_RECORD, 0 );
    if ( rc ) {
      rc = WriteObject( dimstyle );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  else {
    ON_ERROR("ON_BinaryArchive::Write3dmDimStyle() must be called in BeginWrite3dmDimStyleTable() block");
    rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmDimStyleTable()
{
  bool rc = false;
  rc = EndWrite3dmTable( TCODE_DIMSTYLE_TABLE );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmDimStyleTable()
{
  if ( m_3dm_version <= 2 ) {
    return true;
  }
  bool rc = false;
  rc = BeginRead3dmTable( TCODE_DIMSTYLE_TABLE );

  if ( !rc )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions when it makes sense.
    rc = FindMisplacedTable( 
                0,
                TCODE_DIMSTYLE_TABLE, TCODE_DIMSTYLE_RECORD,
                ON_DimStyle::m_ON_DimStyle_class_id.Uuid(),
                30 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_DIMSTYLE_TABLE );
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmDimStyle( ON_DimStyle** ppDimStyle )
{
  if ( !ppDimStyle )
    return 0;
  *ppDimStyle = 0;
  if ( m_3dm_version <= 2 ) {
    return 0;
  }
  if ( m_active_table != dimstyle_table ) {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmDimStyleTable() - m_active_table != no_active_table");
  }
  if ( m_3dm_opennurbs_version < 200109260 ) {
    // 3DM archives written before version 200109260 and before do not have dimstyle tables
    return 0;
  }

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_DimStyle* dimstyle = NULL;
  tcode = 0;
  if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) 
  {
    if ( tcode == TCODE_DIMSTYLE_RECORD ) {
      ON_Object* p = 0;
      if ( ReadObject( &p ) ) {
        dimstyle = ON_DimStyle::Cast(p);
        if ( !dimstyle )
          delete p;
      }
      if (!dimstyle) {
        ON_ERROR("ON_BinaryArchive::Read3dmDimStyle() - corrupt dimstyle table");
      }
    }
    else if ( tcode != TCODE_ENDOFTABLE ) {
      ON_ERROR("ON_BinaryArchive::Read3dmDimStyle() - corrupt dimstyle table");
    }
    EndRead3dmChunk();
  }
  *ppDimStyle = dimstyle;
  return (dimstyle) ? 1 : 0;
}

bool ON_BinaryArchive::EndRead3dmDimStyleTable()
{
  bool rc = false;
  if ( m_3dm_version <= 2 ) {
    return true;
  }
  else {
    rc = EndRead3dmTable( TCODE_DIMSTYLE_TABLE );
  }
  return rc;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmHatchPatternTable()
{
  bool rc = false;
  rc = BeginWrite3dmTable( TCODE_HATCHPATTERN_TABLE );
  return rc;
}

bool ON_BinaryArchive::Write3dmHatchPattern( const ON_HatchPattern&  pattern )
{
  bool rc = false;
  if ( m_active_table != hatchpattern_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmHatchPattern() - m_active_table != hatchpattern_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_HATCHPATTERN_TABLE ) 
  {
    rc = BeginWrite3dmChunk( TCODE_HATCHPATTERN_RECORD, 0 );
    if (rc)
    {
      rc = WriteObject( pattern );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }

    // 1 Nov 2005 Dale Lear:
    //
    //   This code was used before version 200511010.  The reader can
    //   still read the old files, but old versions of Rhino cannot read
    //   files written with version 200511010 or later.  This happened in
    //   the early beta cycile of V4.  V3 did not have hatch patterns.
    //   Please leave this comment here until Nov 2006 so I can remember
    //   what happened if I have to debug file IO.  By May 2006, all
    //   the betas that could write the bogus hatch tables should have
    //   expired.
    //
    //if ( rc ) {
    //  rc = pattern.Write( *this)?true:false;
    //  if ( !EndWrite3dmChunk())
    //    rc = false;
    //}
  }
  else 
  {
    ON_ERROR("ON_BinaryArchive::Write3dmHatchPattern() must be called in BeginWrite3dmHatchPatternTable() block");
    rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmHatchPatternTable()
{
  bool rc = false;
  rc = EndWrite3dmTable( TCODE_HATCHPATTERN_TABLE );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmHatchPatternTable()
{
  if ( m_3dm_version <= 3)
  {
    return true;
  }
  bool rc = BeginRead3dmTable( TCODE_HATCHPATTERN_TABLE );

  if ( !rc && m_3dm_opennurbs_version >= 200511010 )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions when it makes sense.
    //    It only works on files with ver
    rc = FindMisplacedTable( 
                0,
                TCODE_HATCHPATTERN_TABLE, TCODE_HATCHPATTERN_RECORD,
                ON_HatchPattern::m_ON_HatchPattern_class_id.Uuid(),
                30 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_HATCHPATTERN_TABLE );
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmHatchPattern( ON_HatchPattern** ppPattern )
{
  if( !ppPattern )
    return 0;
  
  *ppPattern = 0;
  if( m_3dm_version <= 3) // 1 Nov 2005 Dale lear: change < to <= 
    return 0;             // because v3 files don't have hatch patterns.

  if ( m_active_table != hatchpattern_table ) 
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmHatchPatternTable() - m_active_table != hatchpattern_table");
  }
  if ( m_3dm_opennurbs_version < 200405030 ) 
  {
    // 3DM archives written before version 200405030 do not have hatchpattern tables
    return 0;
  }

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_HatchPattern* pPat = NULL;
  if( BeginRead3dmBigChunk( &tcode, &big_value)) 
  {
    if ( tcode == TCODE_HATCHPATTERN_RECORD ) 
    {
      if ( m_3dm_opennurbs_version < 200511010 )
      {
        // There was a bug in Write3dmHatchPattern and files written 
        // before version 200511010 didn't use ON_Object IO.
        pPat = new ON_HatchPattern;
        if( !pPat->Read( *this))
        {
          delete pPat;
          pPat = NULL;
          ON_ERROR("ON_BinaryArchive::Read3dmHatchPattern() - corrupt hatch pattern table");
        }
      }
      else
      {
        ON_Object* p = 0;
        if ( ReadObject( &p ) ) 
        {
          pPat = ON_HatchPattern::Cast(p);
          if ( !pPat )
            delete p;
        }
        if (!pPat) 
        {
          ON_ERROR("ON_BinaryArchive::Read3dmLayer() - corrupt layer table");
        }
      }
    }
    else if ( tcode != TCODE_ENDOFTABLE ) 
    {
      ON_ERROR("ON_BinaryArchive::Read3dmHatchPattern() - corrupt hatch pattern table");
    }

    EndRead3dmChunk();
  }
  *ppPattern = pPat;
  return( pPat) ? 1 : 0;
}

bool ON_BinaryArchive::EndRead3dmHatchPatternTable()
{
  bool rc = false;
  if( m_3dm_version <= 3)
  {
    return true;
  }
  else 
  {
    rc = EndRead3dmTable( TCODE_HATCHPATTERN_TABLE);
  }
  return rc;
}



///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmLinetypeTable()
{
  bool rc = BeginWrite3dmTable( TCODE_LINETYPE_TABLE );
  return rc;
}

bool ON_BinaryArchive::Write3dmLinetype( const ON_Linetype&  linetype )
{
  bool rc = false;

  if( m_active_table != linetype_table ) 
  {
    ON_ERROR("ON_BinaryArchive::Write3dmLinetype() - m_active_table != linetype_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_LINETYPE_TABLE ) 
  {
    rc = BeginWrite3dmChunk( TCODE_LINETYPE_RECORD, 0 );
    if ( rc ) 
    {
      rc = WriteObject( linetype );
      if ( !EndWrite3dmChunk())
        rc = false;
    }
  }
  else 
  {
    ON_ERROR("ON_BinaryArchive::Write3dmLinetype() must be called in BeginWrite3dmLinetypeTable() block");
    rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmLinetypeTable()
{
  bool rc = EndWrite3dmTable( TCODE_LINETYPE_TABLE );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmLinetypeTable()
{
  bool rc = false;

  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200503170 )
  {
    rc = true;
  }
  else
  {
    rc = BeginRead3dmTable( TCODE_LINETYPE_TABLE );
    if ( !rc )
    {
      // 1 November 2005 Dale Lear
      //    This fall back is slow but it has been finding
      //    layer and object tables in damaged files.  I'm
      //    adding it to the other BeginRead3dm...Table()
      //    functions when it makes sense.
      rc = FindMisplacedTable( 
                  0,
                  TCODE_LINETYPE_TABLE, TCODE_LINETYPE_RECORD,
                  ON_Linetype::m_ON_Linetype_class_id.Uuid(),
                  20 
                  );
      if ( rc )
      {
        rc = BeginRead3dmTable( TCODE_LINETYPE_TABLE );
      }
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmLinetype( ON_Linetype** ppLinetype )
{
  if( !ppLinetype)
    return 0;
  
  *ppLinetype = 0;

  if( m_3dm_version < 4 || m_3dm_opennurbs_version < 200503170) 
    return 0;

  if ( m_active_table != linetype_table ) 
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmLinetypeTable() - m_active_table != linetype_table");
  }

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_Linetype* linetype = NULL;
  int rc = -1;
  if( BeginRead3dmBigChunk( &tcode, &big_value)) 
  {
    if ( tcode == TCODE_LINETYPE_RECORD ) 
    {
      ON_Object* p = 0;
      if ( ReadObject( &p ) )
      {
        linetype = ON_Linetype::Cast(p);
        if (!linetype )
          delete p;
        else
        {
          if (ppLinetype)
            *ppLinetype = linetype;
          rc = 1;
        }
      }
      if (!linetype)
      {
        ON_ERROR("ON_BinaryArchive::Read3dmLinetype() - corrupt linetype table");
      }
    }
    else if ( tcode == TCODE_ENDOFTABLE ) 
    {
     // end of linetype table
      rc = 0;
    }
    else
    {
      ON_ERROR("ON_BinaryArchive::Read3dmLinetype() - corrupt linetype table");
    }
    if (!EndRead3dmChunk())
      rc = -1;
  }

  return rc;
}

bool ON_BinaryArchive::EndRead3dmLinetypeTable()
{
  bool rc = false;
  if( m_3dm_version < 4 || m_3dm_opennurbs_version < 200503170) 
  {
    rc = true;
  }
  else
  {
    rc = EndRead3dmTable( TCODE_LINETYPE_TABLE);
  }
  return rc;
}


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmInstanceDefinitionTable()
{
  bool rc = false;
  rc = BeginWrite3dmTable( TCODE_INSTANCE_DEFINITION_TABLE );
  return rc;
}

bool ON_BinaryArchive::Write3dmInstanceDefinition( const ON_InstanceDefinition&  idef )
{
  bool rc = false;
  if ( m_active_table != instance_definition_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmInstanceDefinition() - m_active_table != instance_definition_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_INSTANCE_DEFINITION_TABLE ) {
    rc = BeginWrite3dmChunk( TCODE_INSTANCE_DEFINITION_RECORD, 0 );
    if ( rc ) {
      rc = WriteObject( idef );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  else {
    ON_ERROR("ON_BinaryArchive::Write3dmInstanceDefinition() must be called in BeginWrite3dmInstanceDefinitionTable() block");
    rc = false;
  }

  return rc;
}

bool ON_BinaryArchive::EndWrite3dmInstanceDefinitionTable()
{
  bool rc = false;
  rc = EndWrite3dmTable( TCODE_INSTANCE_DEFINITION_TABLE );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmInstanceDefinitionTable()
{
  if ( m_3dm_version <= 2 ) {
    return true;
  }
  bool rc = false;
  rc = BeginRead3dmTable( TCODE_INSTANCE_DEFINITION_TABLE );

  if ( !rc )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions when it makes sense.
    rc = FindMisplacedTable( 
                0,
                TCODE_INSTANCE_DEFINITION_TABLE, TCODE_INSTANCE_DEFINITION_RECORD,
                ON_InstanceDefinition::m_ON_InstanceDefinition_class_id.Uuid(),
                30 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_INSTANCE_DEFINITION_TABLE );
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmInstanceDefinition( ON_InstanceDefinition** ppInstanceDefinition )
{
  if ( !ppInstanceDefinition )
    return 0;
  *ppInstanceDefinition = 0;
  if ( m_3dm_version <= 2 ) {
    return 0;
  }
  if ( m_active_table != instance_definition_table )
  {
    ON_ERROR("ON_BinaryArchive::BeginRead3dmInstanceDefinitionTable() - m_active_table != no_active_table");
  }
  if ( m_3dm_opennurbs_version < 200205110 ) 
  {
    // 3DM archives written before version 200205110 and before do not have instance definition tables
    return 0;
  }

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_InstanceDefinition* idef = NULL;
  if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) 
  {
    if ( tcode == TCODE_INSTANCE_DEFINITION_RECORD ) {
      ON_Object* p = 0;
      if ( ReadObject( &p ) ) {
        idef = ON_InstanceDefinition::Cast(p);
        if ( !idef )
          delete p;
      }
      if (!idef) {
        ON_ERROR("ON_BinaryArchive::Read3dmInstanceDefinition() - corrupt instance definition table");
      }
    }
    else if ( tcode != TCODE_ENDOFTABLE ) {
      ON_ERROR("ON_BinaryArchive::Read3dmInstanceDefinition() - corrupt instance definition table");
    }
    EndRead3dmChunk();
  }
  *ppInstanceDefinition = idef;
  return (idef) ? 1 : 0;
}

bool ON_BinaryArchive::EndRead3dmInstanceDefinitionTable()
{
  bool rc = false;
  if ( m_3dm_version <= 2 ) {
    return true;
  }
  else {
    rc = EndRead3dmTable( TCODE_INSTANCE_DEFINITION_TABLE );
  }
  return rc;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmTextureMappingTable()
{
  return BeginWrite3dmTable( TCODE_TEXTURE_MAPPING_TABLE );
}

bool ON_BinaryArchive::Write3dmTextureMapping( const ON_TextureMapping& texture_mapping )
{
  bool rc = false;

  if ( m_active_table != texture_mapping_table ) 
  {
    ON_ERROR("ON_BinaryArchive::Write3dmTextureMapping() - m_active_table != texture_mapping_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( !c || c->m_typecode != TCODE_TEXTURE_MAPPING_TABLE )
  {
    ON_ERROR("ON_BinaryArchive::Write3dmTextureMapping() - active chunk typecode != TCODE_TEXTURE_MAPPING_TABLE");
  }
  else 
  {
    rc = BeginWrite3dmChunk( TCODE_TEXTURE_MAPPING_RECORD, 0 );
    if (rc) 
    {
      rc = WriteObject( texture_mapping );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndWrite3dmTextureMappingTable()
{
  return EndWrite3dmTable( TCODE_TEXTURE_MAPPING_TABLE );
}

bool ON_BinaryArchive::BeginRead3dmTextureMappingTable()
{
  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200511110 )
  {
    return true;
  }

  bool rc = BeginRead3dmTable( TCODE_TEXTURE_MAPPING_TABLE );
  if ( !rc )
  {
    // 31 October 2005 Dale Lear
    //    This fall back is slow but it will find
    //    texture_mapping tables in files that have been damaged.
    //
    //    This approach has been tested with layer tables
    //    for over a year and has successfully made files
    //    with garbled starts read correctly after the
    //    call to EmergencyFindTable was able to detect 
    //    the start of the layer table.  I'm adding it
    //    to texture_mapping tables now because I have a good
    //    test file.
    rc = FindMisplacedTable( 
                0,
                TCODE_TEXTURE_MAPPING_TABLE, TCODE_TEXTURE_MAPPING_RECORD,
                ON_TextureMapping::m_ON_TextureMapping_class_id.Uuid(),
                sizeof(ON_TextureMapping)
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_TEXTURE_MAPPING_TABLE );
    }
  }
  return rc;
}

int ON_BinaryArchive::Read3dmTextureMapping( ON_TextureMapping** ppTextureMapping )
{
  int rc = 0;
  if ( !ppTextureMapping )
    return 0;
  *ppTextureMapping = 0;
  ON_TextureMapping* texture_mapping = NULL;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200511110 ) 
  {
    // no texture mapping table until version 200511110 of v4 files
    return 0;
  }

  rc = -1;
  if ( BeginRead3dmBigChunk( &tcode, &big_value ) )
  {
    if ( tcode == TCODE_TEXTURE_MAPPING_RECORD ) 
    {
      ON_Object* p = 0;
      if ( ReadObject( &p ) ) 
      {
        texture_mapping = ON_TextureMapping::Cast(p);
        if ( !texture_mapping )
          delete p;
        else 
        {
          if ( ppTextureMapping )
            *ppTextureMapping = texture_mapping;
          rc = 1;
        }
      }
      if (!texture_mapping)
      {
        ON_ERROR("ON_BinaryArchive::Read3dmTextureMapping() - corrupt texture_mapping table");
      }
    }
    else if ( tcode == TCODE_ENDOFTABLE ) 
    {
      // end of texture_mapping table
      rc = 0;
    }
    else 
    {
      ON_ERROR("ON_BinaryArchive::Read3dmTextureMapping() - corrupt texture_mapping table");
    }
    if ( !EndRead3dmChunk() )
      rc = -1;
  }

  return rc;
}

bool ON_BinaryArchive::EndRead3dmTextureMappingTable()
{
  bool rc = false;
  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200511110 )
  {
    rc = true;
  }
  else
  {
    rc = EndRead3dmTable( TCODE_TEXTURE_MAPPING_TABLE );
  }
  return rc;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmHistoryRecordTable()
{
  return BeginWrite3dmTable( TCODE_HISTORYRECORD_TABLE );
}

bool ON_BinaryArchive::Write3dmHistoryRecord( const ON_HistoryRecord& history_record )
{
  bool rc = false;

  if ( m_active_table != historyrecord_table ) 
  {
    ON_ERROR("ON_BinaryArchive::Write3dmHistoryRecord() - m_active_table != history_record_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( !c || c->m_typecode != TCODE_HISTORYRECORD_TABLE )
  {
    ON_ERROR("ON_BinaryArchive::Write3dmHistoryRecord() - active chunk typecode != TCODE_HISTORYRECORD_TABLE");
  }
  else 
  {
    rc = BeginWrite3dmChunk( TCODE_HISTORYRECORD_RECORD, 0 );
    if (rc) 
    {
      rc = WriteObject( history_record );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndWrite3dmHistoryRecordTable()
{
  return EndWrite3dmTable( TCODE_HISTORYRECORD_TABLE );
}

bool ON_BinaryArchive::BeginRead3dmHistoryRecordTable()
{
  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200601180 )
  {
    return true;
  }

  bool rc = BeginRead3dmTable( TCODE_HISTORYRECORD_TABLE );
  if ( !rc )
  {
    // 31 October 2005 Dale Lear
    //    This fall back is slow but it will find
    //    history_record tables in files that have been damaged.
    //
    //    This approach has been tested with layer tables
    //    for over a year and has successfully made files
    //    with garbled starts read correctly after the
    //    call to EmergencyFindTable was able to detect 
    //    the start of the layer table.  I'm adding it
    //    to history_record tables now because I have a good
    //    test file.
    rc = FindMisplacedTable( 
                0,
                TCODE_HISTORYRECORD_TABLE, TCODE_HISTORYRECORD_RECORD,
                ON_HistoryRecord::m_ON_HistoryRecord_class_id.Uuid(),
                sizeof(ON_HistoryRecord)
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_HISTORYRECORD_TABLE );
    }
  }
  return rc;
}

int ON_BinaryArchive::Read3dmHistoryRecord( ON_HistoryRecord*& history_record )
{
  int rc = 0;
  history_record = 0;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200601180 ) 
  {
    // no history record table until version 200601180 of v4 files
    return 0;
  }

  rc = -1;
  if ( BeginRead3dmBigChunk( &tcode, &big_value ) )
  {
    if ( tcode == TCODE_HISTORYRECORD_RECORD ) 
    {
      ON_Object* p = 0;
      if ( ReadObject( &p ) ) 
      {
        history_record = ON_HistoryRecord::Cast(p);
        if ( !history_record )
        {
          delete p;
        }
        else 
        {
          rc = 1;
        }
      }
      if (!history_record)
      {
        ON_ERROR("ON_BinaryArchive::Read3dmHistoryRecord() - corrupt history_record table");
      }
    }
    else if ( tcode == TCODE_ENDOFTABLE ) 
    {
      // end of history_record table
      rc = 0;
    }
    else 
    {
      ON_ERROR("ON_BinaryArchive::Read3dmHistoryRecord() - corrupt history_record table");
    }
    if ( !EndRead3dmChunk() )
      rc = -1;
  }

  return rc;
}

bool ON_BinaryArchive::EndRead3dmHistoryRecordTable()
{
  bool rc = false;
  if ( m_3dm_version < 4 || m_3dm_opennurbs_version < 200601180 )
  {
    rc = true;
  }
  else
  {
    rc = EndRead3dmTable( TCODE_HISTORYRECORD_TABLE );
  }
  return rc;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool ON_BinaryArchive::BeginWrite3dmMaterialTable()
{
  return BeginWrite3dmTable( TCODE_MATERIAL_TABLE );
}

bool ON_BinaryArchive::Write3dmMaterial( const ON_Material& material )
{
  bool rc = false;

  if ( m_active_table != material_table ) 
  {
    ON_ERROR("ON_BinaryArchive::Write3dmMaterial() - m_active_table != material_table");
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( !c || c->m_typecode != TCODE_MATERIAL_TABLE )
  {
    ON_ERROR("ON_BinaryArchive::Write3dmMaterial() - active chunk typecode != TCODE_MATERIAL_TABLE");
  }
  else 
  {
    rc = BeginWrite3dmChunk( TCODE_MATERIAL_RECORD, 0 );
    if (rc) 
    {
      rc = WriteObject( material );
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndWrite3dmMaterialTable()
{
  return EndWrite3dmTable( TCODE_MATERIAL_TABLE );
}

bool ON_BinaryArchive::BeginRead3dmMaterialTable()
{
  m_3dm_v1_material_index = 0;
  bool rc = BeginRead3dmTable( TCODE_MATERIAL_TABLE );
  if ( !rc )
  {
    // 31 October 2005 Dale Lear
    //    This fall back is slow but it will find
    //    material tables in files that have been damaged.
    //
    //    This approach has been tested with layer tables
    //    for over a year and has successfully made files
    //    with garbled starts read correctly after the
    //    call to EmergencyFindTable was able to detect 
    //    the start of the layer table.  I'm adding it
    //    to material tables now because I have a good
    //    test file.
    rc = FindMisplacedTable( 
                0,
                TCODE_MATERIAL_TABLE, TCODE_MATERIAL_RECORD,
                ON_Material::m_ON_Material_class_id.Uuid(),
                114
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_MATERIAL_TABLE );
    }
  }
  return rc;
}

bool ON_BinaryArchive::Read3dmV1String( ON_String& s )
{
  int string_length = 0;
  s.Empty();
  bool rc = ReadInt( &string_length );
  if (rc) {
    s.ReserveArray(string_length+1);
    rc = ReadChar( string_length, s.Array() );
    if (rc)
      s.SetLength(string_length);
  }
  return rc;
}


class ON__3dmV1_XDATA
{
  // helper class to get V1 "xdata" out of attributes block.
public:
  enum
  {
    unknown_xdata = 0,
    hidden_object_layer_name, // m_string = actual layer name
    locked_object_layer_name, // m_string = actual layer name
    arrow_direction,          // m_vector = arrow head location
    dot_text                  // m_string = dot text
  }
  m_type;
  ON_String m_string; 
  ON_3dVector m_vector;
};

bool ON_BinaryArchive::Read3dmV1AttributesOrMaterial( 
                         ON_3dmObjectAttributes* attributes,
                         ON_Material* material,
                         ON_BOOL32& bHaveMat,
                         unsigned int end_mark_tcode,
                         ON__3dmV1_XDATA* xdata
                         )
{
  // Check ReadV1Material() if you fix any bugs in the mateial related items

  if ( 0 != xdata )
  {
    xdata->m_type = ON__3dmV1_XDATA::unknown_xdata;
  }

  bool rc = false;
  unsigned int u;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_Color c;
  bHaveMat = false;
  bool bEndRead3dmChunk_rc;

  const unsigned int saved_error_message_mask = m_error_message_mask;

  int xdata_layer_index = -1;

  if ( attributes )
  {
    attributes->Default();
  }

  if ( material ) 
  {
    material->Default();
    material->m_diffuse.SetRGB(255,255,255);
    material->m_specular.SetRGB(255,255,255);
    material->m_ambient.SetRGB(0,0,0);
  }

  for (;;) 
  {
    m_error_message_mask = saved_error_message_mask;

    if ( end_mark_tcode != TCODE_ENDOFTABLE ) {
      tcode = 0;
      big_value = 0;
      if ( !PeekAt3dmBigChunkType(&tcode,&big_value) ) {
        break; // should not happen
      }
      if ( tcode == end_mark_tcode ) {
        rc = true;
        break; // done reading attributes
      }
    }
    tcode = 0;
    big_value = 0;
    if ( !BeginRead3dmBigChunk(&tcode,&big_value) )
      break;
    if ( tcode == end_mark_tcode ) {
      rc = EndRead3dmChunk();
      break;
    }
    
    switch( tcode ) 
    {      
    case (TCODE_OPENNURBS_OBJECT | TCODE_CRC | 0x7FFD):
      // 1.1 object 16 byte UUID + 2 byte crc
      if ( attributes )
        ReadUuid( attributes->m_uuid );
      break;
      
    case TCODE_LAYERREF:
      if (    attributes 
           && (-1 == xdata_layer_index || attributes->m_layer_index != xdata_layer_index) 
           && (big_value >= 0 && big_value < 0x7FFFFFFF)
         )
      {
        attributes->m_layer_index = (int)big_value;
      }
      break;
      
    case TCODE_RGB:
      if ( big_value != 0xFFFFFF ) 
      {
        if ( material ) 
        {
          ON__UINT64 rgb64 = (ON__UINT64)big_value;
          ON__UINT32 rgb32 = (ON__UINT32)rgb64;
          u = rgb32;
          c.SetRGB( u%256,(u>>8)%256,(u>>16)%256 );
          material->SetDiffuse(c);
          material->SetShine((u >> 24)/100.0*ON_Material::MaxShine());
        }
        bHaveMat = true;
      }
      break;		
      
    case TCODE_RGBDISPLAY:
      if ( attributes ) 
      {
          ON__UINT64 rgb64 = (ON__UINT64)big_value;
          ON__UINT32 rgb32 = (ON__UINT32)rgb64;
          u = rgb32;
        attributes->m_color.SetRGB( u%256,(u>>8)%256,(u>>16)%256 );
      }
      break;
      
    case TCODE_TRANSPARENCY:
      if ( big_value > 0 && big_value <= 255 ) 
      {
        if ( material )
          material->SetTransparency(big_value/255.0);
        bHaveMat = true;
      }
      break;				
      
    case TCODE_NAME:
      if ( attributes ) {
        ON_String s;
        Read3dmV1String(s);
        if( s.Length() > 0 )
          attributes->m_name = s;
      }
      break;
   
    case TCODE_TEXTUREMAP:
      {
        ON_String s;
        Read3dmV1String(s);
        if ( s.Length() > 0 ) 
        {
          if ( material )
          {
            ON_Texture& tx = material->m_textures.AppendNew();
            tx.m_filename = s;
            tx.m_type = ON_Texture::bitmap_texture;
          }
          bHaveMat = true;
        }
      }
      break;

    case TCODE_BUMPMAP:
      if ( material ) {
        ON_String s;
        Read3dmV1String(s);
        if ( s.Length() ) 
        {
          if ( material )
          {
            ON_Texture& tx = material->m_textures.AppendNew();
            tx.m_filename = s;
            tx.m_type = ON_Texture::bump_texture;
          }
          bHaveMat = true;
        }
      }
      break;
      
    case TCODE_XDATA:
      // v1 "xdata"
      if ( attributes ) 
      {
        ON_String layer_name;
        ON_String xid;
        int sizeof_xid = 0;
        int sizeof_data = 0;
        ReadInt(&sizeof_xid);
        ReadInt(&sizeof_data);
        xid.SetLength(sizeof_xid);
        ReadByte(sizeof_xid,xid.Array());
        if ( !on_stricmp("RhHidePrevLayer",xid) )
        {
          // v1 object is hidden - real layer name is in xdata
          char* buffer = (char*)alloca((sizeof_data+1)*sizeof(buffer[0]));
          buffer[0] = 0;
          buffer[sizeof_data] = 0;
          if ( ReadByte(sizeof_data,buffer) )
          {
            if ( -1 == xdata_layer_index )
            {
              xdata_layer_index = Read3dmV1LayerIndex(buffer);
              if ( xdata_layer_index >= 0 )
              {
                attributes->m_layer_index = xdata_layer_index;
                attributes->SetVisible(false);
              }
            }
            else
            {
              xdata_layer_index = -2;
            }
            //if ( 0 != xdata )
            //{
            //  xdata->m_type = ON__3dmV1_XDATA::hidden_object_layer_name;
            //  xdata->m_string = buffer;               
            //}
          }
        }
        else if ( !on_stricmp("RhFreezePrevLayer",xid) )
        {
          // v1 object is locked - real layer name is in xdata
          char* buffer = (char*)alloca((sizeof_data+1)*sizeof(buffer[0]));
          buffer[0] = 0;
          buffer[sizeof_data] = 0;
          if ( ReadByte(sizeof_data,buffer)  )
          {
            if ( -1 == xdata_layer_index )
            {
              xdata_layer_index = Read3dmV1LayerIndex(buffer);
              if ( xdata_layer_index >= 0 )
              {
                attributes->m_layer_index = xdata_layer_index;
                attributes->SetMode(ON::locked_object);
              }
            }
            else
            {
              xdata_layer_index = -2;
            }
            //if ( 0 != xdata )
            //{
            //  xdata->m_type = ON__3dmV1_XDATA::locked_object_layer_name;
            //  xdata->m_string = buffer;            
            //}
          }
        }
        else if ( !on_stricmp("RhAnnotateArrow",xid) && 24 == sizeof_data )
        {
          // v1 annotation arrow objects were saved
          // as TCODE_RH_POINT objects with the
          // arrow tail location = point location and the
          // arrow head location saved in 24 bytes of "xdata".
          ON_3dVector arrow_direction;
          if ( ReadVector( arrow_direction ) && 0 != xdata )
          {
            xdata->m_type = ON__3dmV1_XDATA::arrow_direction;
            xdata->m_vector = arrow_direction;
          }
        }
        else if ( !on_stricmp("RhAnnotateDot",xid) )
        {
          // v1 annotation dot objects were saved
          // as TCODE_RH_POINT objects with the
          // dot text saved in "xdata".
          char* buffer = (char*)alloca((sizeof_data+1)*sizeof(buffer[0]));
          buffer[0] = 0;
          buffer[sizeof_data] = 0;
          if ( ReadByte(sizeof_data,buffer) && 0 != xdata )
          {
            xdata->m_type = ON__3dmV1_XDATA::dot_text;
            xdata->m_string = buffer;            
          }
        }
        else 
        {
          m_error_message_mask |= 0x0002; // disable v1 EndRead3dmChunk() partially read chunk warning
        }
        // call to EndRead3dmChunk() will skip unread junk
      }
      break;
      
    case TCODE_DISP_CPLINES:
      if ( big_value > 0 && big_value <= 0x7FFFFFFF && attributes )
        attributes->m_wire_density = (int)big_value;
      break;

    case TCODE_RENDER_MATERIAL_ID:
      {
        int flag;
        ON_String s;
        ReadInt(&flag);
        if ( flag == 1 ) {
          Read3dmV1String(s);
          if ( s.Length() > 0 ) 
          {
            if ( material )
              material->m_material_name = s;
            bHaveMat = true;
          }
        }
      }
      break;
      
    default:
      // obsolete attributes from v1
      m_error_message_mask |= 0x02; // disable v1 EndRead3dmChunk() partially read chunk warning
      break;
    }

    bEndRead3dmChunk_rc = EndRead3dmChunk();
    if ( !bEndRead3dmChunk_rc )
      break;
  }

  m_error_message_mask = saved_error_message_mask;

  if ( bHaveMat ) {
    if ( attributes )
      attributes->m_material_index = m_3dm_v1_material_index;
    if ( material )
      material->SetMaterialIndex(m_3dm_v1_material_index);
    m_3dm_v1_material_index++;
  }

  return rc;
}



int ON_BinaryArchive::Read3dmV1Material( ON_Material** ppMaterial )
{
  int rc = 0;
  // returns -1: failure
  //          0: end of material table
  //          1: success

  ON_Material material;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_BOOL32 bHaveMat;
  bool bEndReadChunk_rc;
  // reads NURBS, point, and mesh objects

  while( 0 == rc )
  {
    bHaveMat = false;
    rc = 0;
    tcode = 0;
    big_value = 0;
    if ( !BeginRead3dmBigChunk(&tcode,&big_value) ) 
    {
      // assume we are at the end of the file
      break;
    }

    switch(tcode)
    {
    case TCODE_RH_POINT:
      // v1 3d point
      {
        ON_3DM_BIG_CHUNK* point_chunk = m_chunk.Last();
        ON__UINT64 pos0 = 0;
        if (    0 != point_chunk 
             && TCODE_RH_POINT == point_chunk->m_typecode 
             && 0 == point_chunk->m_big_value )
        {
          // Some V1 files have TCODE_RH_POINT chunks with length=0.
          // (It appears that points with arrow xdata have this problem.)
          // For these chunks we need to set the chunk length so EndRead3dmChunk()
          // will keep going.
          pos0 = CurrentPosition();
        }
        else
          point_chunk = 0;

        ON_3dPoint pt; // need to read point to get to material defn
        bool bOK = ReadPoint( pt );
        
        if ( bOK )
          bOK = Read3dmV1AttributesOrMaterial( NULL, &material, bHaveMat, TCODE_ENDOFTABLE );
        
        if ( !bOK )
          rc = -1;
        // else if appropriate, rc will get set to +1 below.

        if ( bOK
             && 0 != point_chunk 
             && point_chunk == m_chunk.Last()
             && TCODE_RH_POINT == point_chunk->m_typecode 
             && 0 == point_chunk->m_big_value )
        {
          // set the chunk length so that reading can continue.
          ON__UINT64 pos1 = CurrentPosition();
          ON__UINT64 chunk_length = (pos1 > pos0) ? (pos1 - pos0) : 0;
          if ( chunk_length >= 32 && chunk_length < 0x0FFFFFFF )
            point_chunk->m_big_value = (ON__INT64)chunk_length;
        }
      }
      break;

    case TCODE_MESH_OBJECT:
      // v1 mesh
      {
        ON__UINT32 tc = 0;
        ON__INT64 i64 = 0;
        if ( !PeekAt3dmBigChunkType( &tc, &i64 ) )
          break;
        if ( tc != TCODE_COMPRESSED_MESH_GEOMETRY )
          break;
        // skip over the TCODE_COMPRESSED_MESH_GEOMETRY chunk
        if ( !BeginRead3dmBigChunk(&tc,&i64) )
          break;
        if ( !EndRead3dmChunk() )
          break;
        // attributes and material informtion follow the TCODE_COMPRESSED_MESH_GEOMETRY chunk
        if ( !Read3dmV1AttributesOrMaterial( NULL, &material, bHaveMat, TCODE_ENDOFTABLE ) )
          rc = -1;
        // if appropriate, rc will get set to +1 below
      }
      break;

    case TCODE_LEGACY_SHL:
      // v1 polysurface
      if ( !Read3dmV1AttributesOrMaterial( NULL, &material, bHaveMat, TCODE_LEGACY_SHLSTUFF ) )
        rc = -1;
        // if appropriate, rc will get set to +1 below
      break;
    case TCODE_LEGACY_FAC:
      // v1 trimmed surface
      if ( !Read3dmV1AttributesOrMaterial( NULL, &material, bHaveMat, TCODE_LEGACY_FACSTUFF ) )
        rc = -1;
        // if appropriate, rc will get set to +1 below
      break;
    case TCODE_LEGACY_CRV:
      // v1 curve
      if ( !Read3dmV1AttributesOrMaterial( NULL, &material, bHaveMat, TCODE_LEGACY_CRVSTUFF ) )
        rc = -1;
        // if appropriate, rc will get set to +1 below
      break;

    case TCODE_RHINOIO_OBJECT_NURBS_CURVE:
    case TCODE_RHINOIO_OBJECT_NURBS_SURFACE:
    case TCODE_RHINOIO_OBJECT_BREP:
      // old Rhino I/O toolkit nurbs curve, surface, and breps
      {
        ON__UINT32 tc = 0;
        ON__INT64 i64 = 0;
        if ( !PeekAt3dmBigChunkType( &tc, &i64 ) )
          break;
        if ( tc != TCODE_RHINOIO_OBJECT_DATA )
          break;
        // skip over the TCODE_RHINOIO_OBJECT_DATA chunk
        if ( !BeginRead3dmBigChunk(&tc,&i64) )
          break;
        if ( !EndRead3dmChunk() )
          break;
        if ( !Read3dmV1AttributesOrMaterial( NULL, &material, bHaveMat, TCODE_RHINOIO_OBJECT_END ) )
          rc = -1;
        // if appropriate, rc will get set to +1 below
      }
      break;
    }

    const unsigned int saved_error_message_mask = m_error_message_mask;
    m_error_message_mask |= 0x02; // disable v1 EndRead3dmChunk() partially read chunk warning
    bEndReadChunk_rc = EndRead3dmChunk();
    m_error_message_mask = saved_error_message_mask; // enable v1 EndRead3dmChunk() partially read chunk warning
    if (!bEndReadChunk_rc ) 
    {
      rc = -1;
      break;
    }
    if ( bHaveMat && ppMaterial) 
    {
      // found a valid non-default material
      *ppMaterial = new ON_Material(material);
      rc = 1;
      break;
    }
  }

  return rc;
}


int ON_BinaryArchive::Read3dmMaterial( ON_Material** ppMaterial )
{
  int rc = 0;
  if ( !ppMaterial )
    return 0;
  *ppMaterial = 0;
  ON_Material* material = NULL;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  if ( m_3dm_version == 1 ) 
  {
    rc = ON_BinaryArchive::Read3dmV1Material( ppMaterial );
  }
  else 
  {
    // version 2+
    rc = -1;
    if ( BeginRead3dmBigChunk( &tcode, &big_value ) )
    {
      if ( tcode == TCODE_MATERIAL_RECORD ) 
      {
        ON_Object* p = 0;
        if ( ReadObject( &p ) ) 
        {
          material = ON_Material::Cast(p);
          if ( !material )
            delete p;
          else 
          {
            if ( ppMaterial )
              *ppMaterial = material;
            rc = 1;
          }
        }
        if (!material)
        {
          ON_ERROR("ON_BinaryArchive::Read3dmMaterial() - corrupt material table");
        }
      }
      else if ( tcode == TCODE_ENDOFTABLE ) 
      {
        // end of material table
        rc = 0;
      }
      else 
      {
        ON_ERROR("ON_BinaryArchive::Read3dmMaterial() - corrupt material table");
      }
      if ( !EndRead3dmChunk() )
        rc = -1;
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndRead3dmMaterialTable()
{
  return EndRead3dmTable( TCODE_MATERIAL_TABLE );
}


bool ON_BinaryArchive::BeginWrite3dmLightTable()
{
  return BeginWrite3dmTable( TCODE_LIGHT_TABLE );
}

bool ON_BinaryArchive::Write3dmLight( const ON_Light& light,  const ON_3dmObjectAttributes* attributes )
{
  bool rc = false;
  if ( m_active_table != light_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmLight() - m_active_table != light_table");
  }
  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_LIGHT_TABLE ) {
    rc = BeginWrite3dmChunk( TCODE_LIGHT_RECORD, 0 );
    if (rc) {
      // WriteObject writes TCODE_OPENNURBS_CLASS chunk that contains light definition
      rc = WriteObject( light );

      // optional TCODE_LIGHT_RECORD_ATTRIBUTES chunk
      if ( rc && attributes ) 
      {
        rc = BeginWrite3dmChunk( TCODE_LIGHT_RECORD_ATTRIBUTES, 0 );
        if (rc) 
        {
          rc = attributes->Write( *this )?true:false;
          if (!EndWrite3dmChunk())
            rc = false;
          if( rc 
              && (m_bSaveUserData || HasCriticalUserData(*this,attributes))
              && Archive3dmVersion() >= 4 
              && 0 != attributes->FirstUserData() 
            )
          {
            // 14 May 2008 Dale Lear
            //    Added support for saving light attribute userdata
            rc = BeginWrite3dmChunk( TCODE_LIGHT_RECORD_ATTRIBUTES_USERDATA, 0 );
            if (rc)
            {
              // write user data
              rc = WriteObjectUserData(*attributes);
              if (rc)
              {
                // Because I'm not using Write3dmObject() to write
                // the attributes, the user data must be immediately 
                // followed by a short TCODE_OPENNURBS_CLASS_END chunk 
                // in order for ReadObjectUserData() to work correctly.
                //
                // The reason that this is hacked in is that V3 files did
                // not support attribute user data and doing it this way
                // means that V3 can still read V4 files.
                rc = BeginWrite3dmChunk(TCODE_OPENNURBS_CLASS_END,0);
                if (rc)
                {
                  if (!EndWrite3dmChunk())
                    rc = false;
                }
              }
              if (!EndWrite3dmChunk())
                rc = false;
            }
          }
        }
      }

      // TCODE_LIGHT_RECORD_END chunk marks end of light record
      if ( BeginWrite3dmChunk( TCODE_LIGHT_RECORD_END, 0 ) ) {
        if (!EndWrite3dmChunk())
          rc = false;
      }
      else {
        rc = false;
      }

      if ( !EndWrite3dmChunk() ) // end of TCODE_LIGHT_RECORD
        rc = false;
    }
  }
  else {
    ON_ERROR("ON_BinaryArchive::Write3dmMaterial() - active chunk typecode != TCODE_LIGHT_TABLE");
  }
  return rc;
}

bool ON_BinaryArchive::EndWrite3dmLightTable()
{
  return EndWrite3dmTable( TCODE_LIGHT_TABLE );
}

bool ON_BinaryArchive::BeginRead3dmLightTable()
{
  bool rc = BeginRead3dmTable( TCODE_LIGHT_TABLE );

  if ( !rc )
  {
    // 1 November 2005 Dale Lear
    //    This fall back is slow but it has been finding
    //    layer and object tables in damaged files.  I'm
    //    adding it to the other BeginRead3dm...Table()
    //    functions when it makes sense.
    rc = FindMisplacedTable( 
                0,
                TCODE_LIGHT_TABLE, TCODE_LIGHT_RECORD,
                ON_Light::m_ON_Light_class_id.Uuid(),
                30 
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_LIGHT_TABLE );
    }
  }

  return rc;
}

int ON_BinaryArchive::Read3dmV1Light(  // returns 0 at end of light table
                    //         1 light successfully read
                    //        -1 if file is corrupt
          ON_Light** ppLight, // light returned here
          ON_3dmObjectAttributes* pAttributes// optional - if NOT NULL, object attributes are
                                  //            returned here
          )
{
  ON_BOOL32 bHaveMat;
  ON_Material material;
  // TODO - read v1 lights
  if ( m_chunk.Count() != 0 ) {
    ON_ERROR("ON_BinaryArchive::Read3dmV1Light() m_chunk.Count() != 0");
    return false;
  }
  ON_BOOL32 rc = false;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;

  // find TCODE_RH_SPOTLIGHT chunk
  for(;;)
  {
    if ( !BeginRead3dmBigChunk(&tcode,&big_value) )
      break; // assume we are at the end of the file
    if ( tcode == TCODE_RH_SPOTLIGHT ) {
      rc = 1;
      break;
    }
    if ( !EndRead3dmChunk() )
      break;
  }
  if (rc) {
    ON_3dPoint origin;
    ON_3dVector xaxis, yaxis;
    double radius;
    double height;
    double hotspot;

    for(;;)
    {
      rc = ReadPoint( origin );
      if (!rc) break;
      rc = ReadVector( xaxis );
      if (!rc) break;
      rc = ReadVector( yaxis );
      if (!rc) break;
      rc = ReadDouble( &radius );
      if (!rc) break;
      rc = ReadDouble( &height );
      if (!rc) break;
      rc = ReadDouble( &hotspot );
      if (!rc) break;
      if (ppLight ) 
      {
        ON_3dVector Z = ON_CrossProduct( xaxis, yaxis );
        ON_3dPoint  location = height*Z + origin;
        ON_3dVector direction = -Z;

        if( height > 0.0)
          direction *= height;
        ON_Light* light = new ON_Light;
        light->SetStyle( ON::world_spot_light );
        light->SetLocation(location);
        light->SetDirection(direction);
        light->SetSpotExponent( 64.0);
        if( radius > 0.0 && height > 0.0 )
          light->SetSpotAngleRadians( atan( radius/height));
        *ppLight = light;
      }
      break;
    }

    if (rc && ppLight && *ppLight) {
      bHaveMat = false;
      Read3dmV1AttributesOrMaterial(pAttributes,&material,bHaveMat,TCODE_ENDOFTABLE);
      if ( pAttributes )
        pAttributes->m_material_index = -1;
      if (bHaveMat)
        (*ppLight)->SetDiffuse(material.Diffuse());
    }

    if ( !EndRead3dmChunk() ) // end of TCODE_RH_SPOTLIGHT chunk
      rc = false;
  }

  return rc;
}

int ON_BinaryArchive::Read3dmLight( ON_Light** ppLight, ON_3dmObjectAttributes* attributes )
{
  if ( attributes )
    attributes->Default();
  int rc = -1;
  if ( !ppLight )
    return 0;
  *ppLight = 0;
  if ( m_active_table != light_table ) {
    ON_ERROR("ON_BinaryArchive::Read3dmLight() - m_active_table != light_table");
  }
  else if ( m_3dm_version == 1 ) {
    rc = Read3dmV1Light( ppLight, attributes );
  }
  else {
    ON_Light* light = NULL;
    ON__UINT32 tcode = 0;
    ON__INT64 big_value = 0;
    if ( BeginRead3dmBigChunk( &tcode, &big_value ) ) {
      if ( tcode == TCODE_LIGHT_RECORD ) {
        ON_Object* p = 0;
        if ( ReadObject( &p ) ) {
          light = ON_Light::Cast(p);
          if ( !light )
            delete p;
        }
        if (!light) {
          ON_ERROR("ON_BinaryArchive::Read3dmLight() - corrupt light table");
        }
        else {
          *ppLight = light;
          rc = 1;
        }
      }
      else if ( tcode != TCODE_ENDOFTABLE ) 
      {
        ON_ERROR("ON_BinaryArchive::Read3dmLight() - corrupt light table");
      }
      else
        rc = 0;

      while(rc==1) 
      {
        tcode = 0;
        big_value = 0;
        if (!BeginRead3dmBigChunk( &tcode, &big_value )) 
        {
          rc = -1;
          break;
        }
        if ( tcode == TCODE_LIGHT_RECORD_ATTRIBUTES && attributes ) 
        {
          if ( !attributes->Read( *this ) )
            rc = -1;
        }
        else if ( tcode == TCODE_LIGHT_RECORD_ATTRIBUTES_USERDATA )
        {
          if ( 0 != attributes )
          {
            // 14 May 2008
            //   Added support for reading user data on light attributes
            if ( !ReadObjectUserData(*attributes))
              rc = -1;
          }
        }
        if ( !EndRead3dmChunk() )
        {
          rc = -1;
          break;
        }
        if ( tcode == TCODE_LIGHT_RECORD_END )
          break;
      }

      EndRead3dmChunk();
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndRead3dmLightTable()
{
  return EndRead3dmTable( TCODE_LIGHT_TABLE );
}

bool ON_BinaryArchive::BeginWrite3dmObjectTable()
{
  return BeginWrite3dmTable( TCODE_OBJECT_TABLE );
}

bool ON_BinaryArchive::Write3dmObject( 
          const ON_Object& object, 
          const ON_3dmObjectAttributes* attributes 
          )
{
  bool rc = false;
  if ( m_active_table != object_table ) {
    ON_ERROR("ON_BinaryArchive::Write3dmObject() - m_active_table != object_table");
  }

  if ( Archive3dmVersion() <= 2 && object.ObjectType() == ON::pointset_object )
  {
    // There were no point clouds in V1 and V2 files and we cannot handle
    // this problem inside of ON_PointCloud::Write() because we have to
    // write multiple point objects.  (c.f. ON_Brep::Write()).
    const ON_PointCloud* pc = ON_PointCloud::Cast(&object);
    if ( 0 != pc )
    {
      int i, count = pc->PointCount();
      rc = true;
      for ( i = 0; i < count && rc ; i++ )
      {
        ON_Point pt( pc->m_P[i] );
        rc = Write3dmObject( pt, attributes );
      }
      return rc;
    }
  }

  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_OBJECT_TABLE ) 
  {
    Flush();
    rc = BeginWrite3dmChunk( TCODE_OBJECT_RECORD, 0 );
    if (rc) {
      // TCODE_OBJECT_RECORD_TYPE chunk integer value that can be used
      // for skipping unwanted types of objects
      rc = BeginWrite3dmChunk( TCODE_OBJECT_RECORD_TYPE, object.ObjectType() );
      if (rc) {
        if (!EndWrite3dmChunk())
          rc = false;
      }

      // WriteObject writes TCODE_OPENNURBS_CLASS chunk that contains object definition
      rc = WriteObject( object );

      // optional TCODE_OBJECT_RECORD_ATTRIBUTES chunk
      if ( rc && attributes ) {
        rc = BeginWrite3dmChunk( TCODE_OBJECT_RECORD_ATTRIBUTES, 0 );
        if (rc) {
          rc = attributes->Write( *this )?true:false;
          if (!EndWrite3dmChunk())
            rc = false;

          if( rc 
              && (m_bSaveUserData || HasCriticalUserData(*this,attributes))
              && Archive3dmVersion() >= 4 
              && 0 != attributes->FirstUserData() 
              )
          {
            // 19 October 2004
            //   Added support for saving user data on object attributes
            rc = BeginWrite3dmChunk( TCODE_OBJECT_RECORD_ATTRIBUTES_USERDATA, 0 );
            if (rc)
            {
              // write user data
              rc = WriteObjectUserData(*attributes);
              if (rc)
              {
                // Because I'm not using Write3dmObject() to write
                // the attributes, the user data must be immediately 
                // followed by a short TCODE_OPENNURBS_CLASS_END chunk 
                // in order for ReadObjectUserData() to work correctly.
                //
                // The reason that this is hacked in is that V3 files did
                // not support attribute user data and doing it this way
                // means that V3 can still read V4 files.
                rc = BeginWrite3dmChunk(TCODE_OPENNURBS_CLASS_END,0);
                if (rc)
                {
                  if (!EndWrite3dmChunk())
                    rc = false;
                }
              }
              if (!EndWrite3dmChunk())
                rc = false;
            }
          }
        }
      }

      // TCODE_OBJECT_RECORD_END chunk marks end of object record
      if ( BeginWrite3dmChunk( TCODE_OBJECT_RECORD_END, 0 ) ) {
        if (!EndWrite3dmChunk())
          rc = false;
      }
      else {
        rc = false;
      }

      if (!EndWrite3dmChunk()) // end of TCODE_OBJECT_RECORD
      {
        rc = false;
      }
      if (!Flush())
        rc = false;
    }
    else {
      ON_ERROR("ON_BinaryArchive::Write3dmObject() - active chunk typecode != TCODE_OBJECT_TABLE");
    }
  }
  return rc;
}

bool ON_BinaryArchive::EndWrite3dmObjectTable()
{
  return EndWrite3dmTable( TCODE_OBJECT_TABLE );
}

bool ON_BinaryArchive::BeginRead3dmObjectTable()
{
  m_3dm_v1_material_index = 0;
  bool rc = BeginRead3dmTable( TCODE_OBJECT_TABLE );
  if ( !rc )
  {
    // 8 October 2004 Dale Lear
    //    This fall back is slow but it will find
    //    object tables in files that have been damaged.
    rc = FindMisplacedTable( 
                0,
                TCODE_OBJECT_TABLE, TCODE_OBJECT_RECORD,
                ON_nil_uuid,
                26 // ON_Point data is 3 doubles + 2 byte version number
                );
    if ( rc )
    {
      rc = BeginRead3dmTable( TCODE_OBJECT_TABLE );
    }

  }
  return rc;
}

bool ON_BinaryArchive::ReadV1_TCODE_RH_POINT(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  ON__UINT64 pos0 = 0;
  ON_3DM_BIG_CHUNK* point_chunk = m_chunk.Last();

  if (    0 != point_chunk 
       && TCODE_RH_POINT == point_chunk->m_typecode 
       && 0 == point_chunk->m_big_value )
  {
    // Some early V1 files have TCODE_RH_POINT chunks with arrow xdata
    // attached have a length set to zero.
    // For these chunks we need to set the chunk length so EndRead3dmChunk()
    // will keep going.
    pos0 = CurrentPosition();
  }
  else
    point_chunk = 0;

  // read v1 point
  bool rc = false;
  ON_BOOL32 bHaveMat = false;
  ON_3dPoint pt;
  ON__3dmV1_XDATA xdata;
  rc = ReadPoint(pt);
  if (rc) 
  {
    rc = Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_ENDOFTABLE,&xdata);
    // do switch even if Read3dmV1AttributesOrMaterial() fails
    switch ( xdata.m_type )
    {
    case ON__3dmV1_XDATA::arrow_direction:
      if ( xdata.m_vector.Length() > ON_ZERO_TOLERANCE )
      {
        ON_AnnotationArrow* arrow = new ON_AnnotationArrow();
        arrow->m_tail = pt;
        arrow->m_head = pt + xdata.m_vector;
        *ppObject = arrow;
      }
      else
      {
        *ppObject = new ON_Point(pt);
      }
      break;

    case ON__3dmV1_XDATA::dot_text:
      {
        ON_AnnotationTextDot* dot = new ON_AnnotationTextDot();
        dot->point = pt;
        dot->m_text = xdata.m_string;
        if ( dot->m_text.IsEmpty() )
          dot->m_text = " "; // a single blank
        *ppObject = dot;
      }
      break;

    default:
      *ppObject = new ON_Point(pt);
      break;
    }
  }

  // carefully test for the V1 zero length chunk bug
  if ( rc && pos0 > 0 && 0 != point_chunk && point_chunk == m_chunk.Last() )
  {
    if ( TCODE_RH_POINT == point_chunk->m_typecode && 0 == point_chunk->m_big_value )
    {
      // This TCODE_RH_POINT chunk has the zero length chunk bug
      // that was in some V1 files. 
      // Fill in the correct chunk length so that reading can continue.
      ON__UINT64 pos1 = CurrentPosition();
      ON__UINT64 chunk_length = (pos1 > pos0) ? (pos1 - pos0) : 0;
      if ( chunk_length >= 32 && chunk_length < 0x0FFFFFFF )
        point_chunk->m_big_value = (ON__INT64)chunk_length;
    }
  }

  return rc;
}

static
void TweakAnnotationPlane( ON_Plane& plane )
{
  // 10 November 2003 Dale Lear
  //   Fixed lots of bugs in annotation plane tweaking.
  //   Before the fix this block of code was cut-n-pasted 
  //   in three places.  The fabs() calls were wrong.  In addition
  //   and the
  //   .x values where tested and then the .y/.z values were set.

  //    if( fabs( plane.origin.x > 1e10 ))
  //      plane.origin.x = 0.0;
  //    if( fabs( plane.origin.y > 1e10 ))
  //      plane.origin.y = 0.0;
  //    if( fabs( plane.origin.z > 1e10 ))
  //      plane.origin.z = 0.0;
  //    
  //    if( fabs( plane.xaxis.x > 1e10 ))
  //      plane.xaxis.x = 1.0;
  //    if( fabs( plane.xaxis.x > 1e10 ))
  //      plane.xaxis.y = 0.0;
  //    if( fabs( plane.xaxis.x > 1e10 ))
  //      plane.xaxis.z = 0.0;
  //    
  //    if( fabs( plane.yaxis.x > 1e10 ))
  //      plane.yaxis.x = 0.0;
  //    if( fabs( plane.yaxis.x > 1e10 ))
  //      plane.yaxis.y = 1.0;
  //    if( fabs( plane.yaxis.x > 1e10 ))
  //      plane.yaxis.z = 0.0;

  // Lowell has decided that annotation plane
  // coordinates bigger than "too_big" should be
  // set to zero.
  const double too_big = 1.0e10;

  if( fabs( plane.origin.x ) > too_big )
    plane.origin.x = 0.0;
  if( fabs( plane.origin.y ) > too_big )
    plane.origin.y = 0.0;
  if( fabs( plane.origin.z ) > too_big )
    plane.origin.z = 0.0;
  
  if( fabs( plane.xaxis.x ) > too_big )
    plane.xaxis.x = 1.0;
  if( fabs( plane.xaxis.y ) > too_big )
    plane.xaxis.y = 0.0;
  if( fabs( plane.xaxis.z ) > too_big )
    plane.xaxis.z = 0.0;
  
  if( fabs( plane.yaxis.x ) > too_big )
    plane.yaxis.x = 0.0;
  if( fabs( plane.yaxis.y ) > too_big )
    plane.yaxis.y = 1.0;
  if( fabs( plane.yaxis.z ) > too_big )
    plane.yaxis.z = 0.0;

  plane.xaxis.Unitize();
  plane.yaxis.Unitize();
  plane.zaxis = ON_CrossProduct(plane.xaxis,plane.yaxis);
  plane.zaxis.Unitize();
  plane.UpdateEquation();
}


#define RHINO_ANNOTATION_SETTINGS_VERSION_1  1
#define RHINO_LINEAR_DIMENSION_VERSION_1     1
#define RHINO_RADIAL_DIMENSION_VERSION_1     1
#define RHINO_ANGULAR_DIMENSION_VERSION_1    1
#define RHINO_TEXT_BLOCK_VERSION_1           1
#define RHINO_TEXT_BLOCK_VERSION_2           2
#define RHINO_ANNOTATION_LEADER_VERSION_1    1

#define BUFLEN 128

static bool ReadV1_TCODE_ANNOTATION_Helper( ON_BinaryArchive& archive, char* buffer, ON_wString& tc )
{
  char* cp = 0;
  int j = 0;
  if( !archive.ReadInt( &j))
    return false;
  size_t sz = (j+1)*sizeof(cp[0]);
  if( j > BUFLEN - 1 || !buffer )
  {
    cp = (char*)onmalloc( sz );
    if( !cp)
      return false;
  }
  else
  {
    cp = buffer;
  }

  memset( cp, 0, sz );
  if( !archive.ReadChar( j,  cp))
  {
    if ( cp != buffer )
      onfree(cp);
    return false;
  }

  cp[j] = 0;
  tc = cp;
  if ( cp != buffer )
    onfree( cp );
  return true;
}

bool ON_BinaryArchive::ReadV1_TCODE_ANNOTATION(
  unsigned int tcode,
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  enum RhAnnotationType
  {
    Nothing = 0,
    TextBlock = 1,
    DimHorizontal = 2,
    DimVertical = 3,
    DimAligned = 4,
    DimRotated = 5,
    DimAngular = 6,
    DimDiameter = 7 ,
    DimRadius = 8,
    Leader = 9,
    DimLinear = 10,
  };
  
  bool rc = false;
  *ppObject = NULL;
  ON_wString tc;
  char buffer[BUFLEN];
  int i, j, k, byobject, version;
  //char* cp;
  double d, d4[4];
  //ON_3dPoint pt;

  switch( tcode)
  {
  case TCODE_TEXT_BLOCK:
    {
      // read the version id
      rc = ReadInt( &version);
      if ( rc &&
           (version == RHINO_TEXT_BLOCK_VERSION_1 ||
            version == RHINO_TEXT_BLOCK_VERSION_2) 
         )
      {
        //this is a version we can read....
        // this is a type flag that we throw away
        rc = ReadInt( &i);
        if( !rc)
          return rc;
 
        ON_TextEntity* text = new ON_TextEntity;
        text->SetType( ON::dtTextBlock);

        ON_Plane plane;

        // the entity plane 
        if( !ReadDouble( 3, &plane.origin.x))
          return false;
        if( !ReadDouble( 3, &plane.xaxis.x))
          return false;
        if( !ReadDouble( 3, &plane.yaxis.x))
          return false;

        // 11 November 2003 Dale Lear - see comments in TweakAnnotationPlane()
        TweakAnnotationPlane( plane );

        text->SetPlane( plane);

        // read string to display
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        text->SetUserText( tc.Array());

        // flags ( future )
        if( !ReadInt( 1, &j))
          return false;

        // settings byobject flag
        if( !ReadInt( 1, &byobject))
          return false;

        // depending on the value of byobject, more fields might be read here

        // facename
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        text->SetFaceName(tc);

        // face weight
        if( !ReadInt( 1, &j))
          return false;
        text->SetFontWeight( j);

        if( !ReadDouble( 1, &d))
          return false;
        text->SetHeight( d);

        // 2 extra doubles were written into the file by mistake in version 1
        if( version == RHINO_TEXT_BLOCK_VERSION_1 )
        {
          if( !ReadDouble( 1, &d))
            return false;
          if( !ReadDouble( 1, &d))
            return false;
        }
    
        if( text->UserText().Length() < 1 )
        {
          *ppObject = 0;
          return true;
        }
        *ppObject = text;
        rc = true;
      }
    }
    break;
 
  case TCODE_ANNOTATION_LEADER:
    {
      // read the version id
      if( !ReadInt( &i))
        return false;
      
      if( i == RHINO_ANNOTATION_LEADER_VERSION_1)
      {
        // redundant type code to throw away
        if( !ReadInt( &i))
          return false;

        ON_Leader* ldr = new ON_Leader;
        ldr->SetType( ON::dtLeader);

        ON_Plane plane;

        // the entity plane 
        if( !ReadDouble( 3, &plane.origin.x))
          return false;
        if( !ReadDouble( 3, &plane.xaxis.x))
          return false;
        if( !ReadDouble( 3, &plane.yaxis.x))
          return false;
        
        // 11 November 2003 Dale Lear - see comments in TweakAnnotationPlane()
        TweakAnnotationPlane( plane );

        ldr->SetPlane( plane);
        
        // flags ( future )
        if( !ReadInt( 1, &j))
          return false;
        
        // settings byobject flag
        if( !ReadInt( 1, &byobject))
          return false;
        
        // number of points to read
        if( !ReadInt( &k))
          return false;
        
        if( k < 2)
          return true;
        
        ON_SimpleArray<ON_2dPoint> points;
        for( j = 0; j < k; j++ )
        {
          double pt[3];
          if( !ReadDouble( 3, pt))
            return false;
          points.Append( ON_2dPoint( pt));
        }
        ldr->SetPoints( points);
        
        *ppObject = ldr;
        rc = true;
        break;
      }
    }
    break;
  case TCODE_LINEAR_DIMENSION:
    {
      // read the version id
      if( !ReadInt( &i))
        return false;
      
      if( i == RHINO_LINEAR_DIMENSION_VERSION_1)
      {
        if( !ReadInt( &i))
          return false;

        ON_LinearDimension* dim = new ON_LinearDimension;
        switch( i )
        {
        case DimHorizontal:
        case DimVertical:
        case DimRotated:
        case DimLinear:
          dim->SetType( ON::dtDimLinear);
          break;
        default:
          dim->SetType( ON::dtDimAligned);
          break;
        }

        ON_Plane plane;

        // the entity plane 
        if( !ReadDouble( 3, &plane.origin.x))
          return false;
        if( !ReadDouble( 3, &plane.xaxis.x))
          return false;
        if( !ReadDouble( 3, &plane.yaxis.x))
          return false;

        // 11 November 2003 Dale Lear - see comments in TweakAnnotationPlane()
        TweakAnnotationPlane( plane );

        dim->SetPlane( plane);
        
        // definition points in coordinates of entity plane
        ON_SimpleArray<ON_2dPoint> points;
        for( j = 0; j < 11; j++ )
        {
          double pt[3];
          if( !ReadDouble( 3, pt))
            return false;
          points.Append( ON_2dPoint( pt));
        }
        dim->SetPoints( points);
        
        // read user text string 
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        dim->SetUserText( tc.Array());
        
        // Set the symbols used in dimension strings to the selected options
        //        SetStringSymbols();
        
        // read string to display
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        dim->SetDefaultText( tc.Array());
        
        // user positioned text flag
        if( !ReadInt( &j))
          return false;
        dim->SetUserPositionedText( j);
        
        // flags ( future )
        if( !ReadInt( 1, &j))
          return false;
        
        // settings byobject flag
        if( !ReadInt( 1, &byobject))
          return false;
        
        *ppObject = dim;
        rc = true;
        break;
      }
    }
    break;

  case TCODE_ANGULAR_DIMENSION:
    {
      // read the version id
      if( !ReadInt( &i))
        return false;
      
      if( i == RHINO_ANGULAR_DIMENSION_VERSION_1)
      {
        if( !ReadInt( &i))
          return false;

        ON_AngularDimension* dim = new ON_AngularDimension;
        dim->SetType( ON::dtDimAngular);
        
        ON_Plane plane;

        // the entity plane 
        if( !ReadDouble( 3, &plane.origin.x))
          return false;
        if( !ReadDouble( 3, &plane.xaxis.x))
          return false;
        if( !ReadDouble( 3, &plane.yaxis.x))
          return false;

        // 11 November 2003 Dale Lear - see comments in TweakAnnotationPlane()
        TweakAnnotationPlane( plane );

        dim->SetPlane( plane);
        
        if( !ReadDouble( &d))
          return false;
        dim->SetAngle( d);

        if( !ReadDouble( &d))
          return false;
        dim->SetRadius( d);

        // distances from apes to start and end of dimensioned lines - not used
        if( !ReadDouble( 4, d4))
          return false;

        // definition points in coordinates of entity plane
        ON_SimpleArray<ON_2dPoint> points;
        for( j = 0; j < 5; j++ )
        {
          double pt[3];
          if( !ReadDouble( 3, pt))
            return false;
          points.Append( ON_2dPoint( pt));
        }
        dim->SetPoints( points);
        
        // read user text string 
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        dim->SetUserText( tc.Array());
        
        // Set the symbols used in dimension strings to the selected options
        //        SetStringSymbols();
        
        // read string to display
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        dim->SetDefaultText( tc.Array());
        
        // user positioned text flag
        if( !ReadInt( &j))
          return false;
        dim->SetUserPositionedText( j);
        
        
        // flags ( future )
        if( !ReadInt( 1, &j))
          return false;
        
        // settings byobject flag
        if( !ReadInt( 1, &byobject))
          return false;
        
        
        *ppObject = dim;
        rc = true;
        break;
      }
    }
    break;

  case TCODE_RADIAL_DIMENSION:
    {
      // read the version id
      if( !ReadInt( &i))
        return false;
      
      if( i == RHINO_RADIAL_DIMENSION_VERSION_1)
      {
        if( !ReadInt( &i))
          return false;

        ON_RadialDimension* dim = new ON_RadialDimension;

        switch( i)
        {
        case DimDiameter:
          dim->SetType( ON::dtDimDiameter);
          break;
        case DimRadius:
          dim->SetType( ON::dtDimRadius);
          break;
        }

        ON_Plane plane;

        // the entity plane 
        if( !ReadDouble( 3, &plane.origin.x))
          return false;
        if( !ReadDouble( 3, &plane.xaxis.x))
          return false;
        if( !ReadDouble( 3, &plane.yaxis.x))
          return false;

        // 11 November 2003 Dale Lear - see comments in TweakAnnotationPlane()
        TweakAnnotationPlane( plane );

        dim->SetPlane( plane);
        
        // definition points in coordinates of entity plane
        ON_SimpleArray<ON_2dPoint> points;
        for( j = 0; j < 5; j++ )
        {
          double pt[3];
          if( !ReadDouble( 3, pt))
            return false;
          points.Append( ON_2dPoint( pt));
        }
        dim->SetPoints( points);
        
        // read user text string 
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        dim->SetUserText( tc.Array());
        
        // Set the symbols used in dimension strings to the selected options
        //        SetStringSymbols();
        
        // read string to display
        if ( !ReadV1_TCODE_ANNOTATION_Helper( *this, buffer, tc ) )
          return false;
        dim->SetDefaultText( tc.Array());
        
        // user positioned text flag
        if( !ReadInt( &j))
          return false;
        dim->SetUserPositionedText( j);
        
        // flags ( future )
        if( !ReadInt( 1, &j))
          return false;
        
        // settings byobject flag
        if( !ReadInt( 1, &byobject))
          return false;
        
        
        *ppObject = dim;
        rc = true;
        break;
      }
    
    }
    break;

  default:  // some unknown type to skip over
    return true;
  } //switch
  
  if( rc)
  {
    ON_BOOL32 bHaveMat = false;
    Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_ENDOFTABLE);
  }
  
  return rc;
  
  // TODO: fill in this function
  
  // input tcode               returned *ppObject points to
  // TCODE_TEXT_BLOCK:         ON_TextEntity
  // TCODE_ANNOTATION_LEADER:  ON_Leader
  // TCODE_LINEAR_DIMENSION:   ON_LinearDimension
  // TCODE_ANGULAR_DIMENSION:  ON_AngularDimension
  // TCODE_RADIAL_DIMENSION:   ON_RadialDimension
  //return true if successful and false for failure.
}


bool ON_BinaryArchive::ReadV1_TCODE_MESH_OBJECT(
                                                ON_Object** ppObject,
                                                ON_3dmObjectAttributes* pAttributes
                                                )
{
  ON_Mesh* mesh = 0;
  bool rc = false;
  // read v1 mesh
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  int i;
  if ( !BeginRead3dmBigChunk(&tcode,&big_value) )
    return false;
  
  if ( tcode == TCODE_COMPRESSED_MESH_GEOMETRY ) for(;;) 
  {
    
    int point_count = 0;
    int face_count = 0;
    ON_BOOL32 bHasVertexNormals = false;
    ON_BOOL32 bHasTexCoords = false;
    ON_BoundingBox bbox;
    
    if (!ReadInt(&point_count) )
      break;
    if ( point_count <= 0 )
      break;
    if (!ReadInt(&face_count) )
      break;
    if ( face_count <= 0 )
      break;
    if (!ReadInt(&bHasVertexNormals) )
      break;
    if (!ReadInt(&bHasTexCoords) )
      break;
    if ( !ReadPoint(bbox.m_min) )
      break;
    if ( !ReadPoint(bbox.m_max) )
      break;

    mesh = new ON_Mesh(face_count,
                       point_count,
                       bHasVertexNormals?true:false,
                       bHasTexCoords?true:false
                       );

    // read 3d vertex locations
    {
      ON_3dVector d = bbox.Diagonal();
      double dx = d.x / 65535.0;
      double dy = d.y / 65535.0;
      double dz = d.z / 65535.0;
      unsigned short xyz[3];
      ON_3fPoint pt;
      for ( i = 0; i < point_count; i++ ) {
        if ( !ReadShort(3,xyz) )
          break;
        pt.x = (float)(dx*xyz[0] + bbox.m_min.x);
        pt.y = (float)(dy*xyz[1] + bbox.m_min.y);
        pt.z = (float)(dz*xyz[2] + bbox.m_min.z);
        mesh->m_V.Append(pt);
      }
    }
    if ( mesh->m_V.Count() != point_count )
      break;

    // read triangle/quadrangle faces
    if ( point_count < 65535 ) {
      unsigned short abcd[4];
      for ( i = 0; i < face_count; i++ ) {
        if ( !ReadShort(4,abcd) )
          break;
        ON_MeshFace& f = mesh->m_F.AppendNew();
        f.vi[0] = abcd[0];
        f.vi[1] = abcd[1];
        f.vi[2] = abcd[2];
        f.vi[3] = abcd[3];
      }
    }
    else {
      int abcd[4];
      for ( i = 0; i < face_count; i++ ) {
        if ( !ReadInt(4,abcd) )
          break;
        ON_MeshFace& f = mesh->m_F.AppendNew();
        f.vi[0] = abcd[0];
        f.vi[1] = abcd[1];
        f.vi[2] = abcd[2];
        f.vi[3] = abcd[3];
      }
    }
    if ( mesh->m_F.Count() != face_count )
      break;

    if ( bHasVertexNormals ) {
      char xyz[3];
      ON_3fVector normal;
      for ( i = 0; i < point_count; i++ ) {
        if ( !ReadChar(3,xyz) )
          break;
        normal.x = (float)(((signed char)xyz[0])/127.0);
        normal.y = (float)(((signed char)xyz[1])/127.0);
        normal.z = (float)(((signed char)xyz[2])/127.0);
        mesh->m_N.Append(normal);
      }
      if ( mesh->m_N.Count() != mesh->m_V.Count() )
        break;
    }

    if ( bHasTexCoords ) {
      unsigned short uv[2];
      ON_2fPoint t;
      for ( i = 0; i < point_count; i++ ) {
        if ( !ReadShort(2,uv) )
          break;
        t.x = (float)(uv[0]/65535.0);
        t.y = (float)(uv[1]/65535.0);
        mesh->m_T.Append(t);
      }
      if ( mesh->m_T.Count() != mesh->m_V.Count() )
        break;
    }

    rc = true;

    break;
  }
  
  if ( !EndRead3dmChunk() )
    rc = false;

  if ( rc && mesh ) {
    *ppObject = mesh;
  }
  else {
    if ( mesh )
      delete mesh;
    rc = false;
  }

  if ( rc && mesh ) {
    // attributes and material information follows the TCODE_COMPRESSED_MESH_GEOMETRY chunk;
    ON_BOOL32 bHaveMat = false;
    Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_ENDOFTABLE);
  }

  return rc;
}

static bool BeginRead3dmLEGACYSTUFF( ON_BinaryArchive& file, unsigned int stuff_tcode )
{
  // begins reading stuff chunk
  bool rc = false;
  ON__UINT32 tcode = !stuff_tcode;
  ON__INT64 big_value = 0;
  for (;;) 
  {
    if ( !file.BeginRead3dmBigChunk(&tcode,&big_value) )
      break;
    if ( tcode == stuff_tcode ) {
      rc = true;
      break;
    }
    if ( !file.EndRead3dmChunk() )
      break;
  }
  return rc;
}

static ON_NurbsCurve* ReadV1_TCODE_LEGACY_SPLSTUFF( ON_BinaryArchive& file )
{
  // reads contents of a v1 TCODE_LEGACY_SPLSTUFF chunk
  ON_NurbsCurve* pNurbsCurve = 0;
  int i, dim, is_rat, order, cv_count, is_closed, form;
  ON_BoundingBox bbox;
  char c;

  // read v1 agspline chunk
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c != 2 && c != 3 )
    return NULL;
  dim = c;
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c != 0 && c != 1 && c != 2 )
    return NULL;
  is_rat = c; // 0 = no, 1 = euclidean, 2 = homogeneous
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c < 2 )
    return NULL;
  order = c;   
  
  {
    // 5 February 2003 - An single case of a V1 file
    //     with a spline that had cv_count = 54467 (>32767)
    //     exists.  Changing from a signed short to
    //     an unsigned short fixed the problem.
    //     The ui casting stuff is here to keep all
    //     the various compilers/lints happy and to
    //     make sure the short with the high bit set
    //     gets properly converted to a positive cv_count.
    unsigned short s;
    if ( !file.ReadShort(1,&s) )
      return NULL;
    unsigned int ui = s;
    cv_count = (int)ui;
    if ( cv_count < order )
      return NULL;
  }

  // The "is_closed" and "form" flags are here to recording
  // the values of legacy data found in the Rhino file.  These
  // values are not used in the toolkit code.
  if ( !file.ReadByte(1,&c) )
    return NULL;  
  if (c != 0 && c != 1 && c != 2)
    return NULL;
  is_closed = c; // 0 = open, 1 = closed, 2 = periodic
  if ( !file.ReadByte(1,&c) )
    return NULL;
  form = c;

  // read bounding box
  if ( !file.ReadDouble( dim, bbox.m_min ) )
    return NULL;
  if ( !file.ReadDouble( dim, bbox.m_max ) )
    return NULL;

  pNurbsCurve = new ON_NurbsCurve( dim, is_rat?true:false, order, cv_count );

  ON_BOOL32 rc = false;
  for(;;) {

    // read legacy v1 knot vector 
    const int knot_count = order+cv_count-2;
    int       knot_index = 0;
    double    knot;

    // clamped_end_knot_flag: 0 = none, 1 = left, 2 = right, 3 = both
    char clamped_end_knot_flag = 0;
    if ( order > 2 ) 
      file.ReadChar(1,&clamped_end_knot_flag);

    // first knot(s)
    if ( !file.ReadDouble(&knot) )
      break;
    pNurbsCurve->m_knot[knot_index++] = knot;
    if (clamped_end_knot_flag % 2) {
      // clamped_start_knot
      while ( knot_index <= order-2 ) 
        pNurbsCurve->m_knot[knot_index++] = knot;
    }

    // middle knots
    while ( knot_index <= cv_count-1 ) {
      if ( !file.ReadDouble(&knot) )
        break;
      pNurbsCurve->m_knot[knot_index++] = knot;
    }
    if ( knot_index <= cv_count-1 )
      break;

    // end knot(s)
    if ( clamped_end_knot_flag >= 2 ) {
      while ( knot_index < knot_count )
        pNurbsCurve->m_knot[knot_index++] = knot;
    }
    else {
      while ( knot_index < knot_count ) {
        if ( !file.ReadDouble(&knot) )
          break;
        pNurbsCurve->m_knot[knot_index++] = knot;
      }
      if ( knot_index < knot_count )
        break;
    }

    // read legacy v1 control points
    const int cvdim = ( is_rat ) ? dim+1 : dim;
    for ( i = 0; i < cv_count; i++ ) {
      if ( !file.ReadDouble( cvdim, pNurbsCurve->CV(i) ) )
        break;
    }
    if ( i < cv_count )
      break;
    if ( is_rat ) {
      // is_rat == 1 check fails because invalid is_rat flags in old files
      // convert rational CVs from euclidean to homogeneous
      double w, *cv;
      int cv_index;
      for ( cv_index = 0; cv_index < cv_count; cv_index++ ) {
        cv = pNurbsCurve->CV(cv_index);
        w = cv[dim];
        for ( i = 0; i < dim; i++ )
          cv[i] *= w;
      }
    }
    if ( order == 2 && cv_count == 2 && pNurbsCurve->m_knot[0] > pNurbsCurve->m_knot[1] ) {
      // a few isolated old v1 3DM file created by Rhino 1.0 files have lines with reversed knots.
      pNurbsCurve->m_knot[0] = -pNurbsCurve->m_knot[0];
      pNurbsCurve->m_knot[1] = -pNurbsCurve->m_knot[1];
    }
    rc = true;

    break;
  }
  if ( !rc && pNurbsCurve ) {
    delete pNurbsCurve;
    pNurbsCurve = 0;
  }
  return pNurbsCurve;
}

static ON_BOOL32 ReadV1_TCODE_LEGACY_SPL( ON_BinaryArchive& file,
  ON_NurbsCurve*& pNurbsCurve
  )
{
  // reads contents of TCODE_LEGACY_SPL chunk
  pNurbsCurve = 0;
  ON_BOOL32 rc = BeginRead3dmLEGACYSTUFF(file, TCODE_LEGACY_SPLSTUFF );
  if ( !rc )
    return false;
  pNurbsCurve = ReadV1_TCODE_LEGACY_SPLSTUFF(file);
  if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_SPLSTUFF chunk
    rc = false;
  if ( !pNurbsCurve )
    rc = false;
  return rc;
}

static ON_NurbsSurface* ReadV1_TCODE_LEGACY_SRFSTUFF( ON_BinaryArchive& file )
{
  // reads contents of TCODE_LEGACY_SRFSTUFF chunk
  ON_NurbsSurface* pNurbsSurface = 0;
  int i, j, dim=0, is_rat=0, order[2], cv_count[2], is_closed[2], is_singular[2], form;
  ON_BoundingBox bbox;
  char c;

  // read contents of v1 TCODE_LEGACY_SRFSTUFF chunk
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c != 2 && c != 3 )
    return NULL;
  dim = c;
  if ( !file.ReadByte(1,&c) )
    return NULL;
  form = c;
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c < 1 )
    return NULL;
  order[0] = c+1;    
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c < 1 )
    return NULL;
  order[1] = c+1;    
  
  {
    // 5 February 2003 - An single case of a V1 files
    //     See the comment above in ReadV1_TCODE_LEGACY_SPLSTUFF
    //     concerning the spline with cv_count >= 0x8000.
    //     The analogous fix is here for the surface case.
    unsigned short s;
    if ( !file.ReadShort(1,&s) )
      return NULL;
    if ( s < 1 )
      return NULL;
    unsigned int ui = s;
    cv_count[0] = order[0]-1+((int)ui);    
    if ( !file.ReadShort(1,&s) )
      return NULL;
    if ( s < 1 )
      return NULL;
    ui = s;
    cv_count[1] = order[1]-1+((int)ui);    
  }
  
  // "ratu" 0 = no, 1 = euclidean, 2 = homogeneous
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c == 1 ) is_rat = 1; else if ( c == 2 ) is_rat = 2;
  
  // "ratv" 0 = no, 1 = euclidean, 2 = homogeneous
  if ( !file.ReadChar(1,&c) )
    return NULL;
  if ( c == 1 ) is_rat = 1; else if ( c == 2 ) is_rat = 2;

  // The "is_closed" and "is_singular" flags are here to recording
  // the values of legacy data found in the Rhino file.  These
  // values are not used in the toolkit code.
  if ( !file.ReadByte(1,&c) )
    return NULL;  
  if (c != 0 && c != 1 && c != 2)
    return NULL;
  is_closed[0] = c; // 0 = open, 1 = closed, 2 = periodic
  if ( !file.ReadByte(1,&c) )
    return NULL;  
  if (c != 0 && c != 1 && c != 2)
    return NULL;
  is_closed[1] = c; // 0 = open, 1 = closed, 2 = periodic

  if ( !file.ReadByte(1,&c) )
    return NULL;  
  if (c != 0 && c != 1 && c != 2 && c != 3)
    return NULL;
  is_singular[0] = c;
  if ( !file.ReadByte(1,&c) )
    return NULL;  
  if (c != 0 && c != 1 && c != 2 && c != 3)
    return NULL;
  is_singular[1] = c;

  // read bounding box
  if ( !file.ReadDouble( dim, bbox.m_min ) )
    return NULL;
  if ( !file.ReadDouble( dim, bbox.m_max ) )
    return NULL;

  pNurbsSurface = new ON_NurbsSurface( dim, is_rat?true:false, 
                                       order[0], order[1], 
                                       cv_count[0], cv_count[1] );

  ON_BOOL32 rc = false;
  for (;;) {

    // read legacy v1 knot vectors 
    if ( !file.ReadDouble( order[0]+cv_count[0]-2, pNurbsSurface->m_knot[0] ) )
      break;
    if ( !file.ReadDouble( order[1]+cv_count[1]-2, pNurbsSurface->m_knot[1] ) )
      break;

    // read legacy v1 control points
    const int cvdim = ( is_rat ) ? dim+1 : dim;
    for ( i = 0; i < cv_count[0]; i++ ) {
      for ( j = 0; j < cv_count[1]; j++ ) {
        if ( !file.ReadDouble( cvdim, pNurbsSurface->CV(i,j) ) )
          break;
      }
      if ( j < cv_count[1] )
        break;
    }
    if ( i < cv_count[0] )
      break;
    if ( is_rat == 1 ) {
      double w, *cv;
      int k;
      for ( i = 0; i < cv_count[0]; i++ ) for ( j = 0; j < cv_count[1]; j++ ) {
        cv = pNurbsSurface->CV(i,j);
        w = cv[dim];
        for ( k = 0; k < dim; k++ )
          cv[k] *= w;
      }
    }
    rc = true;

    break;
  }
  if ( !rc ) {
    delete pNurbsSurface;
    pNurbsSurface = 0;
  }

  return pNurbsSurface;
}

static ON_BOOL32 ReadV1_TCODE_LEGACY_SRF( ON_BinaryArchive& file,
  ON_NurbsSurface*& pNurbsSurface
  )
{
  pNurbsSurface = 0;
  ON_BOOL32 rc = BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_SRF );
  if ( rc ) {
    rc = BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_SRFSTUFF );
    if ( rc ) {
      pNurbsSurface = ReadV1_TCODE_LEGACY_SRFSTUFF( file );
      if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_SRFSTUFF chunk
        rc = false;
    }
    if ( !file.EndRead3dmChunk() )
      rc = false; // end of TCODE_LEGACY_SRF chunk
  }
  if ( !rc && pNurbsSurface ) {
    delete pNurbsSurface;
    pNurbsSurface = 0;
  }
  return rc;
}

ON_Curve* ReadV1_TCODE_LEGACY_CRVSTUFF( ON_BinaryArchive& file )
{
  // reads contents of a v1 TCODE_LEGACY_CRVSTUFF chunk
  ON_Curve* curve = 0;
  ON_PolyCurve* polycurve = 0;
  ON_NurbsCurve* segment = 0;
  ON_BOOL32 rc = false;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  int i;
  ON_BOOL32 bIsPolyline = false;
  ON_BoundingBox bbox;

  for (;;) {
    char c;
    short s;
    int segment_count = 0;
    file.ReadChar(1,&c);  
    if ( c != 2 && c != 3 )
      break;
    int dim = c;
    file.ReadChar(1,&c);
    if ( c != -1 && c != 0 && c != 1 && c != 2 )
      break;
    //int is_closed = (c) ? 1 : 0;
    file.ReadShort(&s);
    if ( s < 1 )
      break;
    file.ReadDouble( dim, bbox.m_min);
    file.ReadDouble( dim, bbox.m_max);
    segment_count = s;
    for ( i = 0; i < segment_count; i++ ) {
      segment = 0;
      tcode = 0;
      big_value = 0;
      if ( !file.BeginRead3dmBigChunk( &tcode, &big_value ) )
        break;
      if ( tcode == TCODE_LEGACY_SPL && big_value > 0 ) {
        ReadV1_TCODE_LEGACY_SPL(file,segment);
      }
      if ( !file.EndRead3dmChunk() ) {
        if ( segment ) {
          delete segment;
          segment = 0;
        }
        break;
      }
      if ( !segment )
        break;
      if ( i == 0 )
        polycurve = new ON_PolyCurve(segment_count);
      if ( segment->CVCount() > 2 || segment->Order() != 2 || segment->IsRational() ) 
      {
        if ( segment->Order() != 2 || segment->IsRational() )
          bIsPolyline = false;
        polycurve->Append(segment);
      }
      else 
      {
        ON_LineCurve* line = new ON_LineCurve();
        line->m_t.Set( segment->m_knot[0], segment->m_knot[1] );
        segment->GetCV( 0, line->m_line.from );
        segment->GetCV( 1, line->m_line.to );
        line->m_dim = segment->m_dim;
        delete segment;
        segment = 0;
        polycurve->Append(line);
      }
    }

    // 5 February 2003
    //   The check for a NULL polycurve was added to avoid
    //   crashes in files when the first NURBS curve in the
    //   polycurve could not be read.
    if ( 0 == polycurve )
      break;
    if ( polycurve->Count() != segment_count )
      break;
    rc = true;
    break;
  }

  if ( rc && polycurve ) 
  {
    if ( polycurve->Count() == 1 )
    {
      curve = polycurve->HarvestSegment(0);
      delete polycurve;
    }
    else if ( bIsPolyline ) 
    {
      ON_PolylineCurve* pline = new ON_PolylineCurve();
      pline->m_dim = polycurve->Dimension();
      pline->m_t.Reserve(polycurve->Count()+1);
      pline->m_t.SetCount(polycurve->Count()+1);
      polycurve->GetSpanVector( pline->m_t.Array() );
      pline->m_pline.Reserve(polycurve->Count()+1);
      for ( i = 0; i < polycurve->Count(); i++ ) {
        pline->m_pline.Append(polycurve->SegmentCurve(i)->PointAtStart());
      }
      pline->m_pline.Append(polycurve->SegmentCurve(polycurve->Count()-1)->PointAtEnd());
      curve = pline;
      delete polycurve;
    }
    else 
    {
      curve = polycurve;
    }
  }
  else 
  {
    if ( polycurve )
      delete polycurve;
    rc = false;
  }

  return curve;
}

bool ON_Brep::ReadV1_LegacyTrimStuff( ON_BinaryArchive& file, 
        ON_BrepFace&, // face - formal parameter intentionally ignored
        ON_BrepLoop& loop )
{
  // read contents of TCODE_LEGACY_TRMSTUFF chunk
  bool rc = false;
  int revedge, gcon, mono;
  int curve2d_index = -1, curve3d_index = -1, trim_index = -1;
  double tol_3d, tol_2d;
  ON_Curve* curve2d = NULL;
  ON_Curve* curve3d = NULL;

  char c;
  file.ReadChar( &c );

  ON_BOOL32 bHasEdge = (c % 2 ); // bit 0 = 1 if "tedge" has non NULL "->edge"
  ON_BOOL32 bHasMate = (c & 6 ); // bit 1 or 2 = 1 if "tedge" has non NULL "->twin"
  ON_BOOL32 bIsSeam  = (c & 2 ); // bit 1 = 1 if "tedge->twin" belongs to same face

  if ( !file.ReadInt(&revedge) )
    return false;
  if ( !file.ReadInt(&gcon) )
    return false;
  if ( !file.ReadInt(&mono) )
    return false;
  if ( !file.ReadDouble( &tol_3d ) )
    return false;
  if ( !file.ReadDouble( &tol_2d ) )
    return false;

  // 2d trim curve
  if ( !BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_CRV ) )
    return false;
  if ( BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_CRVSTUFF ) ) {
    curve2d = ReadV1_TCODE_LEGACY_CRVSTUFF(file);
    if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_CRVSTUFF chunk
      rc = false;
  }
  if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_CRV chunk
    rc = false;
  if ( !curve2d )
    return false;
  curve2d_index = AddTrimCurve(curve2d);
  if ( curve2d_index < 0 ) {
    delete curve2d;
    return false;
  }

  // 3d curve
  if ( bHasEdge ) {
    if ( !BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_CRV ) )
      return false;
    if ( BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_CRVSTUFF ) ) {
      curve3d = ReadV1_TCODE_LEGACY_CRVSTUFF(file);
      if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_CRVSTUFF chunk
        rc = false;
    }
    if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_CRV chunk
      rc = false;
    if ( !curve3d )
      return false;
    curve3d_index = AddEdgeCurve(curve3d);
    if ( curve3d_index < 0 ) {
      delete curve3d;
      return false;
    }
    ON_BrepEdge& edge = NewEdge(curve3d_index);
    ON_BrepTrim& trim = NewTrim( edge, 
                                    revedge ? true : false,
                                    loop,
                                    curve2d_index
                                    );
    trim_index = trim.m_trim_index;
  }
  else {
    ON_BrepTrim& trim = NewTrim( revedge ? true : false,
                                    loop,
                                    curve2d_index
                                    );
    trim_index = trim.m_trim_index;
  }
  if ( trim_index >= 0 ) {
    ON_BrepTrim& trim = m_T[trim_index];
    trim.m__legacy_2d_tol = tol_2d;
    trim.m__legacy_3d_tol = tol_3d;
    trim.m__legacy_flags_Set(gcon,mono);
    if ( bIsSeam ) {
      trim.m_type = ON_BrepTrim::seam;
    }
    else if ( bHasMate ) {
      trim.m_type = ON_BrepTrim::mated;
    }
    else if ( bHasEdge ) {
      trim.m_type = ON_BrepTrim::boundary;
    }
    else {
      trim.m_type = ON_BrepTrim::singular;
    }
  }

  return (trim_index>=0) ? true : false;
}

bool ON_Brep::ReadV1_LegacyTrim( ON_BinaryArchive& file, 
                            ON_BrepFace& face,
                            ON_BrepLoop& loop )
{
  bool rc = false;
  if ( !BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_TRM ) )
    return false;
  rc = BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_TRMSTUFF );
  if ( rc ) {
    rc = ReadV1_LegacyTrimStuff( file, face, loop );
    if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_TRMSTUFF
      rc = false;
  }
  if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_TRM chunk
    rc = false;
  return rc;
}


bool ON_Brep::ReadV1_LegacyLoopStuff( ON_BinaryArchive& file, 
                               ON_BrepFace& face )
{
  // reads contents of TCODE_LEGACY_BNDSTUFF chunk
  // read boundary
  ON_BrepLoop::TYPE loop_type = ON_BrepLoop::unknown;
  int tedge_count, btype, lti;
  double pspace_box[2][2]; // parameter space bounding box

  if ( !file.ReadInt( &tedge_count ) )
    return false;
  if ( tedge_count < 1 ) {
    return false;
  }
  if ( !file.ReadInt( &btype ) )
    return false;
  if ( btype < -1 || btype > 1 ) {
    return false;
  }
  if ( !file.ReadDouble( 4, &pspace_box[0][0] ) )
    return false;
  switch( btype ) {
  case -1:
    loop_type = ON_BrepLoop::slit;
    break;
  case  0:
    loop_type = ON_BrepLoop::outer;
    break;
  case  1:
    loop_type = ON_BrepLoop::inner;
    break;
  }
  ON_BrepLoop& loop = NewLoop( loop_type, face );

  for ( lti = 0; lti < tedge_count; lti++ ) {
    if ( !ReadV1_LegacyTrim( file, face, loop ) )
      return false;
  }

  return true;
}

bool ON_Brep::ReadV1_LegacyLoop( ON_BinaryArchive& file, 
                                        ON_BrepFace& face )
{
  bool rc = false;
  if ( !BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_BND ) )
    return false;
  rc = BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_BNDSTUFF );
  if ( rc ) {
    rc = ReadV1_LegacyLoopStuff( file, face );
    if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_BNDSTUFF
      rc = false;
  }
  if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_BND chunk
    rc = false;
  return rc;
}

bool ON_Brep::ReadV1_LegacyFaceStuff( ON_BinaryArchive& file )
{
  // reads contents of TCODE_LEGACY_FACSTUFF chunk
  ON_NurbsSurface* surface = 0;
  ON_Workspace ws;
  int flipnorm = 0;
  int ftype = 0;
  int bndcnt = 0;
  int twincnt = 0;
  ON_BOOL32 bHasOuter = false;
  ON_BoundingBox face_bbox;

  int ti0 = m_T.Count();

  bool rc = false;

  // read flags
  if ( !file.ReadInt(&flipnorm) )
    return false;
  if ( flipnorm < 0 || flipnorm > 1 )
    return false;
  if ( !file.ReadInt(&ftype) )
    return false;
  if ( !file.ReadInt(&bndcnt) )
    return false;
  bHasOuter = (bndcnt%2); // always true in v1 files
  bndcnt /= 2;

  // read bounding box
  if ( !file.ReadDouble( 3, face_bbox.m_min ) )
    return false;
  if ( !file.ReadDouble( 3, face_bbox.m_max ) )
    return false;

  // B-rep edge gluing info 
  if ( !file.ReadInt(&twincnt) )
    return false;
  short* glue = (twincnt > 0 ) ? (short*)ws.GetMemory(twincnt*sizeof(*glue)) : NULL;
  if (twincnt > 0) {
    if ( !file.ReadShort(twincnt,glue) )
      return false;
  }

  // read surface
  if ( !ReadV1_TCODE_LEGACY_SRF( file, surface ) )
    return false;
  if ( !surface )
    return false;
  const int srf_index = AddSurface(surface);

  // create face
  ON_BrepFace& face = NewFace(srf_index);
  face.m_bRev = (flipnorm) ? true : false;
  face.m_li.Reserve(bndcnt);

  // read boundary loops
  int loop_index = -1;
  if ( !bHasOuter ) {
    // TODO: cook up outer boundary loop (never happes with v1 files)
    face.m_li.Append(loop_index);
  }
  int bi;
  rc = true;
  for ( bi = 0; rc && bi < bndcnt; bi++ ) {
    rc = ReadV1_LegacyLoop( file, face );
  }

  if ( twincnt > 0 ) {
    // twincnt = number of seams edges in face
    // glue[] = order 2 permutation of {0,....,twincnt-1}

    // set seam_i[] = m_T[] indices of seam trims
    int si, ti;
    const int ti1 = m_T.Count();
    int* seam_i = (int*)ws.GetMemory(twincnt*sizeof(*seam_i));
    for ( ti = ti0, si = 0; ti < ti1 && si < twincnt; ti++ ) {
      if (m_T[ti].m_type != ON_BrepTrim::seam )
        continue;
      seam_i[si++] = ti;
    }

    if ( si == twincnt ) {
      // glue seams
      for ( si = 0; si < twincnt; si++ ) {
        if ( glue[si] >= 0 && glue[si] < twincnt ) {
          const int i0 = seam_i[si];
          const int i1 = seam_i[glue[si]];
          // m_T[i0] and m_T[i1] use the same edge;
          const int ei0 = m_T[i0].m_ei;
          const int ei1 = m_T[i1].m_ei;
          if ( ei0 == -1 && ei1 >= 0 ) {
            m_T[i0].m_ei = ei1;
            m_E[ei1].m_ti.Append(i0);
          }
          else if ( ei1 == -1 && ei0 >= 0 ) {
            m_T[i1].m_ei = ei0;
            m_E[ei0].m_ti.Append(i1);
          }
        }
      }
    }
  }

  return rc;
}

bool ON_Brep::ReadV1_LegacyShellStuff( ON_BinaryArchive& file )
{
  // read contents of TCODE_LEGACY_SHLSTUFF chunk
  ON_Workspace ws;
  int outer = 0;
  int facecnt = 0;
  int twincnt = 0;
  ON_BoundingBox shell_bbox;
  const int ti0 = m_T.Count();

  /* read flags */
  file.ReadInt(&outer);  
  file.ReadInt(&facecnt);  

  // read bounding box
  file.ReadPoint( shell_bbox.m_min );
  file.ReadPoint( shell_bbox.m_max );

  /* B-rep edge gluing info */
  file.ReadInt(&twincnt);
  short* glue = (twincnt > 0 ) ? (short*)ws.GetMemory(twincnt*sizeof(*glue)) : NULL;
  if (twincnt > 0)
    file.ReadShort(twincnt,glue);

  bool rc = true;
  int fi;
  for ( fi = 0; rc && fi < facecnt; fi++ ) {
    rc = BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_FAC );
    if ( rc ) {
      rc = BeginRead3dmLEGACYSTUFF( file, TCODE_LEGACY_FACSTUFF );
      if ( rc ) {
        rc = ReadV1_LegacyFaceStuff( file );
        if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_FACSTUFF chunk
          rc = false;
      }
      if ( !file.EndRead3dmChunk() ) // end of TCODE_LEGACY_FAC chunk
        rc = false;
    }
  }

  if ( twincnt > 0 ) {
    // twincnt = number of shared (inter-face) edges
    // glue[] = order 2 permutation of {0,....,twincnt-1}

    // set share_i[] = m_T[] indices of shared trims
    int si, ti;
    const int ti1 = m_T.Count();
    int* share_i = (int*)ws.GetMemory(twincnt*sizeof(*share_i));
    for ( ti = ti0, si = 0; ti < ti1 && si < twincnt; ti++ ) {
      if (m_T[ti].m_type != ON_BrepTrim::mated )
        continue;
      share_i[si++] = ti;
    }

    if ( si == twincnt ) {
      // glue seams
      for ( si = 0; si < twincnt; si++ ) {
        if ( glue[si] >= 0 && glue[si] < twincnt ) {
          const int i0 = share_i[si];
          const int i1 = share_i[glue[si]];
          // m_T[i0] and m_T[i1] use the same edge;
          const int ei0 = m_T[i0].m_ei;
          const int ei1 = m_T[i1].m_ei;
          if ( ei0 == -1 && ei1 >= 0 ) {
            m_T[i0].m_ei = ei1;
            m_E[ei1].m_ti.Append(i0);
          }
          else if ( ei1 == -1 && ei0 >= 0 ) {
            m_T[i1].m_ei = ei0;
            m_E[ei0].m_ti.Append(i1);
          }
        }
      }
    }
  }

  return rc;
}

bool ON_BinaryArchive::ReadV1_TCODE_LEGACY_CRV(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  ON_Curve* curve = NULL;
  bool rc = false;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  ON_BOOL32 bHaveMat = false;

  Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_LEGACY_CRVSTUFF);

  if ( !BeginRead3dmBigChunk( &tcode, &big_value ) )
    return false;
  if ( tcode == TCODE_LEGACY_CRVSTUFF )
    curve = ReadV1_TCODE_LEGACY_CRVSTUFF(*this);
  rc = EndRead3dmChunk();
  if ( !curve )
    rc = false;
  else
    *ppObject = curve;
  return rc;
}


bool ON_BinaryArchive::ReadV1_TCODE_LEGACY_FAC(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  // read V1 TCODE_LEGACY_FAC chunk
  ON_BOOL32 bHaveMat = false;
  if ( !Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_LEGACY_FACSTUFF) )
    return false;
  if ( !BeginRead3dmLEGACYSTUFF( *this, TCODE_LEGACY_FACSTUFF ) )
    return false;
  ON_Brep* brep = new ON_Brep();
  bool rc = brep->ReadV1_LegacyFaceStuff( *this );
  if ( !EndRead3dmChunk() ) // end of TCODE_LEGACY_FACSTUFF chunk
    rc = false;

  if ( !rc ) {
    delete brep;
  }
  else {
    brep->SetVertices();
    brep->SetTrimIsoFlags();
    brep->SetTolsFromLegacyValues();
    *ppObject = brep;
  }

  return rc;
}

bool ON_BinaryArchive::ReadV1_TCODE_LEGACY_SHL(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  // read v1 TCODE_LEGACY_SHL chunk
  ON_BOOL32 bHaveMat = false;
  if ( !Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_LEGACY_SHLSTUFF) )
    return false;
  if ( !BeginRead3dmLEGACYSTUFF( *this, TCODE_LEGACY_SHLSTUFF ) )
    return false;
  ON_Brep* brep = new ON_Brep();
  bool rc = brep->ReadV1_LegacyShellStuff( *this );
  if ( !EndRead3dmChunk() ) // end of TCODE_LEGACY_SHLSTUFF chunk
    rc = false;

  if ( !rc ) {
    delete brep;
  }
  else {
    brep->SetVertices();
    brep->SetTrimIsoFlags();
    brep->SetTolsFromLegacyValues();
    *ppObject = brep;
  }

  return rc;
}


static
ON_NurbsCurve* ReadV1_RHINOIO_NURBS_CURVE_OBJECT_DATA( ON_BinaryArchive& file )
{
  // read TCODE_RHINOIO_OBJECT_DATA chunk that is contained in a
  // TCODE_RHINOIO_OBJECT_NURBS_CURVE chunk.  The TCODE_RHINOIO_OBJECT_DATA
  // chunk contains the definition of NURBS curves written by the 
  // old RhinoIO toolkit.
  ON_NurbsCurve* curve = 0;
  ON_BOOL32 rc = false;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  int version, dim, is_rat, order, cv_count, flag, i;
  if ( !file.BeginRead3dmBigChunk( &tcode, &big_value ) )
    return NULL;
  if ( tcode == TCODE_RHINOIO_OBJECT_DATA ) for (;;) {
    if ( !file.ReadInt(&version) )
      break;
    // int bReverse = version & 0x100;
    version &= 0xFFFFFEFF;
    if ( version != 100 && version != 101 )
      break;
    file.ReadInt(&dim);
    if ( dim < 1 )
      break;
    file.ReadInt(&is_rat);
    if ( is_rat < 0 || is_rat > 1 )
      break;
    file.ReadInt(&order);
    if ( order < 2 )
      break;
    file.ReadInt(&cv_count);
    if ( cv_count < order )
      break;
    file.ReadInt(&flag);
    if ( flag != 0 )
      break;

    curve = new ON_NurbsCurve(dim,is_rat,order,cv_count);
    if ( !file.ReadDouble( order+cv_count-2, curve->m_knot ) )
      break;
    int cvdim = is_rat ? dim+1 : dim;
    for ( i = 0; i < cv_count; i++ ) {
      if ( !file.ReadDouble( cvdim, curve->CV(i) ) )
        break;
    }
    if ( i < cv_count )
      break;
    rc = true;
    break;
  }
  if ( !file.EndRead3dmChunk() ) // end of TCODE_RHINOIO_OBJECT_DATA chunk
    rc = false;
  if ( !rc && curve ) {
    delete curve;
    curve = 0;
  }

  return curve;
}

static
ON_NurbsSurface* ReadV1_RHINOIO_NURBS_SURFACE_OBJECT_DATA( ON_BinaryArchive& file )
{
  // read TCODE_RHINOIO_OBJECT_DATA chunk that is contained in a
  // TCODE_RHINOIO_OBJECT_NURBS_SURFACE chunk.  The TCODE_RHINOIO_OBJECT_DATA
  // chunk contains the definition of NURBS surfaces written by the 
  // old RhinoIO toolkit.
  bool rc = false;
  ON_NurbsSurface* surface = 0;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  int version, dim, is_rat, order[2], cv_count[2], flag, i, j;

  if ( !file.BeginRead3dmBigChunk( &tcode, &big_value ) )
    return NULL;
  if ( tcode == TCODE_RHINOIO_OBJECT_DATA ) for (;;) {
    if ( !file.ReadInt(&version) )
      break;
    // int bReverse = version & 0x100;
    version &= 0xFFFFFEFF;
    if ( version != 100 && version != 101 )
      break;
    file.ReadInt(&dim);
    if ( dim < 1 )
      break;
    file.ReadInt(&is_rat);
    if ( is_rat < 0 || is_rat > 1 )
      break;
    file.ReadInt(&order[0]);
    if ( order[0] < 2 )
      break;
    file.ReadInt(&order[1]);
    if ( order[1] < 2 )
      break;
    file.ReadInt(&cv_count[0]);
    if ( cv_count[0] < order[0] )
      break;
    file.ReadInt(&cv_count[1]);
    if ( cv_count[1] < order[1] )
      break;
    file.ReadInt(&flag);
    if ( flag != 0 )
      break;

    surface = new ON_NurbsSurface(dim,is_rat,order[0],order[1],cv_count[0],cv_count[1]);
    if ( !file.ReadDouble( order[0]+cv_count[0]-2, surface->m_knot[0] ) )
      break;
    if ( !file.ReadDouble( order[1]+cv_count[1]-2, surface->m_knot[1] ) )
      break;
    int cvdim = is_rat ? dim+1 : dim;
    for ( i = 0; i < cv_count[0]; i++ ) {
      for ( j = 0; j < cv_count[1]; j++ ) {
        if ( !file.ReadDouble( cvdim, surface->CV(i,j) ) )
          break;
      }
      if ( j < cv_count[1] )
        break;
    }
    if ( i < cv_count[0] )
      break;
    rc = true;
    break;
  }
  if ( !file.EndRead3dmChunk() ) // end of TCODE_RHINOIO_OBJECT_DATA
    rc = false;
  if ( !rc && surface ) {
    delete surface;
    surface = 0;
  }
  return surface;
}

bool ON_BinaryArchive::ReadV1_TCODE_RHINOIO_OBJECT_NURBS_CURVE(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  // read contents of ReadV1_TCODE_RHINOIO_OBJECT_NURBS_CURVE chunk
  // written by v1 RhinoIO toolkit
  ON_NurbsCurve* curve = 0;
  bool rc = false;
  ON_BOOL32 bHaveMat = false;
  
  // reads TCODE_RHINOIO_OBJECT_DATA header and nurbs curve definition
  curve = ReadV1_RHINOIO_NURBS_CURVE_OBJECT_DATA(*this);

  if ( curve ) {
    *ppObject = curve;
    rc = true;
    Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_RHINOIO_OBJECT_END);
  }

  return rc;
}

bool ON_BinaryArchive::ReadV1_TCODE_RHINOIO_OBJECT_NURBS_SURFACE(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  // read contents of TCODE_RHINOIO_OBJECT_NURBS_SURFACE chunk
  // written by v1 RhinoIO toolkit
  ON_BOOL32 bHaveMat = false;
  bool rc = false;
  ON_NurbsSurface* surface = 0;

  surface = ReadV1_RHINOIO_NURBS_SURFACE_OBJECT_DATA( *this );

  if ( surface ) {
    *ppObject = surface;
    rc = true;
    Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_RHINOIO_OBJECT_END);
  }

  return rc;
}

static
ON_Curve* ReadV1_RHINOIO_BREP_CURVE( ON_BinaryArchive& file )
{
  ON_Curve* curve = NULL;
  ON_PolyCurve* pcurve = NULL;
  ON_NurbsCurve* nurbs_curve = NULL;
  int segment_index, segment_count = 0;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  
  if ( !file.ReadInt(&segment_count) )
    return NULL;
  if ( segment_count < 1 )
    return NULL;

  for ( segment_index = 0; segment_index < segment_count; segment_index++ ) {
    if ( !file.BeginRead3dmBigChunk(&tcode,&big_value) )
      break;
    if ( tcode == TCODE_RHINOIO_OBJECT_NURBS_CURVE ) {
      nurbs_curve = ReadV1_RHINOIO_NURBS_CURVE_OBJECT_DATA( file );
    }
    if ( !file.EndRead3dmChunk() )
      break;
    if ( !nurbs_curve )
      break;
    if ( segment_index == 0 ) {
      curve = nurbs_curve;
      nurbs_curve = 0;
    }
    else {
      if ( segment_index == 1 ) {
        pcurve = new ON_PolyCurve();
        pcurve->Append(curve);
        curve = pcurve;
      }
      pcurve->Append(nurbs_curve);
      nurbs_curve = NULL;
    }
  }

  if ( segment_index < segment_count ) {
    if ( nurbs_curve ) {
      delete nurbs_curve;
      nurbs_curve = 0;
    }
    if ( curve ) {
      delete curve;
      curve = 0;
    }
  }
  return curve;
}

bool ON_BinaryArchive::ReadV1_TCODE_RHINOIO_OBJECT_BREP(
  ON_Object** ppObject,
  ON_3dmObjectAttributes* pAttributes
  )
{
  ON_3dPoint m_oldTrim_mP[2];
  ON_BOOL32 bHaveMat = false;
  bool rc = false;
  ON_Brep* brep = 0;
  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  if ( !BeginRead3dmBigChunk( &tcode, &big_value ) )
    return false;
  if ( tcode == TCODE_RHINOIO_OBJECT_DATA ) for (;;) {
    int version = -1;
    int sz, i, j;
    double tol2d, tol3d;
    if ( !ReadInt( &version ) ) 
      break; // serialization version
    // version = 100 means the b-rep was written by the RhinoIO toolkit
    // version = 101 means the b-rep was written by Rhino 1.0
    if ( version != 100 && version != 101 ) {
      return false;
    }

    brep = new ON_Brep();

    // 2d trimming curves
    if ( !ReadInt( &sz ) )
      break;
    if ( sz < 1 ) {
      break;
    }
    brep->m_C2.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_Curve* curve = ReadV1_RHINOIO_BREP_CURVE( *this );
      if ( !curve )
        break;
      brep->m_C2.Append(curve);
    }
    if ( i < sz )
      break;

    // 3d trimming curves
    if ( !ReadInt( &sz ) )
      break;
    if ( sz < 1 ) {
      break;
    }
    brep->m_C3.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_Curve* curve = ReadV1_RHINOIO_BREP_CURVE( *this );
      if ( !curve )
        break;
      brep->m_C3.Append(curve);
    }
    if ( i < sz )
      break;

    // 3d untrimmed surfaces
    if ( !ReadInt( &sz ) )
      break;
    if ( sz < 1 ) {
      break;
    }
    brep->m_S.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_NurbsSurface* surface = 0;
      tcode = 0;
      big_value = 0;
      if ( !BeginRead3dmBigChunk(&tcode,&big_value) )
        break;
      if ( tcode == TCODE_RHINOIO_OBJECT_NURBS_SURFACE ) {
        surface = ReadV1_RHINOIO_NURBS_SURFACE_OBJECT_DATA( *this );
      }
      if ( !EndRead3dmChunk() )
        break;
      if ( !surface )
        break;
      brep->m_S.Append(surface);
    }
    if ( i < sz )
      break;

    // vertices
    ReadInt( &sz );
    brep->m_V.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_BrepVertex& vertex = brep->NewVertex();
      if ( !ReadInt( &vertex.m_vertex_index ) ) break;
      if ( !ReadPoint( vertex.point ) ) break;
      if ( !ReadArray( vertex.m_ei ) ) break;
      if ( !ReadDouble( &vertex.m_tolerance ) ) break;
    }
    if ( i < sz )
      break;

    // edges
    ReadInt( &sz );
    brep->m_E.Reserve(sz);
    for ( i = 0; i < sz; i++ ) 
    {
      ON_Interval proxy_domain;
      ON_BrepEdge& edge = brep->NewEdge();
      if ( !ReadInt( &edge.m_edge_index ) ) break;
      if ( !ReadInt( &edge.m_c3i ) ) break;
      if ( !ReadInterval( proxy_domain ) ) break;
      edge.SetProxyCurveDomain(proxy_domain);
      if ( !ReadInt( 2, edge.m_vi ) ) break;
      if ( !ReadArray( edge.m_ti ) ) break;
      if ( !ReadDouble( &edge.m_tolerance ) ) break;
    }
    if ( i < sz )
      break;

    // trims
    ReadInt( &sz );
    brep->m_T.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_BrepTrim& trim = brep->NewTrim();
      if ( !ReadInt( &trim.m_trim_index ) ) break;
      if ( !ReadInt( &trim.m_c2i ) ) break;
      ON_Interval d;
      if ( !ReadInterval( d ) ) 
        break;
      trim.SetProxyCurve(NULL,d);
      if ( !ReadInt( &trim.m_ei ) ) break;
      if ( !ReadInt( 2, trim.m_vi ) ) break;
      j = trim.m_bRev3d;
      if ( !ReadInt( &j ) ) break;
      trim.m_bRev3d = (j!=0);
      if ( !ReadInt( &j ) ) break;
      switch(j) {
      case 1: trim.m_type = ON_BrepTrim::boundary; break;
      case 2: trim.m_type = ON_BrepTrim::mated; break;
      case 3: trim.m_type = ON_BrepTrim::seam; break;
      case 4: trim.m_type = ON_BrepTrim::singular; break;
      }
      if ( !ReadInt( &j ) ) break; // legacy iso flag - ignore and recaluate
      if ( !ReadInt( &trim.m_li ) ) break;
      if ( !ReadDouble( 2, trim.m_tolerance ) ) break;
      if ( !ReadPoint( m_oldTrim_mP[0] ) ) break;
      if ( !ReadPoint( m_oldTrim_mP[1] ) ) break;
      if ( !ReadDouble( &tol2d ) ) break;
      if ( !ReadDouble( &tol3d ) ) break;
    }
    if ( i < sz )
      break;

    // loops
    ReadInt( &sz );
    brep->m_L.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_BrepLoop& loop = brep->NewLoop(ON_BrepLoop::unknown);
      if ( !ReadInt( &loop.m_loop_index ) ) break;
      if ( !ReadArray( loop.m_ti ) ) break;
      if ( !ReadInt( &j ) ) break;
      switch (j) {
      case 1: loop.m_type = ON_BrepLoop::outer; break;
      case 2: loop.m_type = ON_BrepLoop::inner; break;
      case 3: loop.m_type = ON_BrepLoop::slit; break;
      }
      if ( !ReadInt( &loop.m_fi ) ) break;
    }
    if ( i < sz )
      break;

    // faces
    ReadInt( &sz );
    brep->m_F.Reserve(sz);
    for ( i = 0; i < sz; i++ ) {
      ON_BrepFace& face = brep->NewFace();
      if ( !ReadInt( &face.m_face_index ) ) break;
      if ( !ReadArray( face.m_li ) ) break;
      if ( !ReadInt( &face.m_si ) ) break;
      int k = face.m_bRev;
      if ( !ReadInt( &k ) ) break;
      face.m_bRev = (k!=0);
    }
    if ( i < sz )
      break;

    // bounding box
    {
      ON_BoundingBox bbox;
      if ( !ReadPoint( bbox.m_min ) ) break;
      if ( !ReadPoint( bbox.m_max ) ) break;
    }

    rc = true;
    break;
  }
  if ( !EndRead3dmChunk() )
    rc = false;
  if ( rc && brep ) {
    brep->SetTrimIsoFlags();
    *ppObject = brep;
  }
  else {
    if ( brep )
      delete brep;
    rc = false;
  }

  if ( rc && brep ) {
    Read3dmV1AttributesOrMaterial(pAttributes,NULL,bHaveMat,TCODE_RHINOIO_OBJECT_END);
  }

  return rc;
}

int
ON_BinaryArchive::Read3dmV1Object( 
  ON_Object** ppObject,                // object is returned here
  ON_3dmObjectAttributes* pAttributes, // optional - object attributes 
  unsigned int object_filter           // optional filter made by or-ing object_type bits
  )
{
  int rc = 0;
  // returns -1: failure
  //          0: end of geometry table
  //          1: success
  //          2: skipped filtered objects

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  // reads NURBS, point, and mesh objects
  for(;;)
  {
    tcode = 0;
    big_value = 0;
    if ( !BeginRead3dmBigChunk(&tcode,&big_value) ) {
      rc = 0; // at the end of the file
      break;
    }
    switch(tcode)
    {

    case  TCODE_TEXT_BLOCK:
    case  TCODE_ANNOTATION_LEADER:
    case  TCODE_LINEAR_DIMENSION:
    case  TCODE_ANGULAR_DIMENSION:
    case  TCODE_RADIAL_DIMENSION:
      if ( 0 != (ON::annotation_object & object_filter) )
      {
        if ( ReadV1_TCODE_ANNOTATION( tcode, ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else
      {
        rc = 2;
      }
      break;

    case TCODE_RH_POINT:
      // v1 3d point
      if ( 0 != (ON::point_object & object_filter) ) {
        if (ReadV1_TCODE_RH_POINT( ppObject, pAttributes ))
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_MESH_OBJECT:
      // v1 mesh
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_MESH_OBJECT( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_LEGACY_SHL:
      // v1 polysurface
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_LEGACY_SHL( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_LEGACY_FAC:
      // v1 trimmed surface
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_LEGACY_FAC( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_LEGACY_CRV:
      // v1 curve
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_LEGACY_CRV( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_RHINOIO_OBJECT_NURBS_CURVE:
      // old Rhino I/O toolkit nurbs curve
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_RHINOIO_OBJECT_NURBS_CURVE( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_RHINOIO_OBJECT_NURBS_SURFACE:
      // old Rhino I/O toolkit nurbs surface
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_RHINOIO_OBJECT_NURBS_SURFACE( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;

    case TCODE_RHINOIO_OBJECT_BREP:
      // old Rhino I/O toolkit nurbs brep
      if ( 0 != (ON::mesh_object & object_filter) ) {
        if ( ReadV1_TCODE_RHINOIO_OBJECT_BREP( ppObject, pAttributes ) )
          rc = 1;
        else
          rc = -1;
      }
      else {
        rc = 2;
      }
      break;
    }

    if (!EndRead3dmChunk() )
      break;
    if ( rc == 1 || rc == -1 )
      break;
  }

  return rc;
}

#if 1
class ON_OBSOLETE_CCustomMeshUserData : public ON_UserData
{
public:
	ON_OBJECT_DECLARE(ON_OBSOLETE_CCustomMeshUserData);
  ON_OBSOLETE_CCustomMeshUserData();
	~ON_OBSOLETE_CCustomMeshUserData();
	ON_BOOL32 GetDescription( ON_wString& );
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);
  bool m_bInUse;
  ON_MeshParameters m_mp;
};

ON_OBJECT_IMPLEMENT(ON_OBSOLETE_CCustomMeshUserData, ON_UserData, "69F27695-3011-4FBA-82C1-E529F25B5FD9");

ON_OBSOLETE_CCustomMeshUserData::ON_OBSOLETE_CCustomMeshUserData()
{
	m_userdata_copycount = 0;
	m_userdata_uuid = ON_OBSOLETE_CCustomMeshUserData::m_ON_OBSOLETE_CCustomMeshUserData_class_id.Uuid();
  m_application_uuid = ON_nil_uuid;
  m_bInUse = false;
}

ON_OBSOLETE_CCustomMeshUserData::~ON_OBSOLETE_CCustomMeshUserData()
{
}

ON_BOOL32 ON_OBSOLETE_CCustomMeshUserData::Read(ON_BinaryArchive& ba)
{
  int i = 0;
  if ( !ba.ReadInt( &i ) )
    return false;    
  if( !ba.ReadBool( &m_bInUse ) )
    return false;    
  return m_mp.Read( ba );
}

ON_BOOL32 ON_OBSOLETE_CCustomMeshUserData::GetDescription( ON_wString& s )
{
	s = "OBSOLETE CustomMeshUserData";
	return true;
}
#endif


int
ON_BinaryArchive::Read3dmObject( 
  ON_Object** ppObject,                // object is returned here
  ON_3dmObjectAttributes* pAttributes, // optional - object attributes 
  unsigned int object_filter           // optional filter made by or-ing object_type bits
  )
{
  // returns -1: failure
  //          0: end of geometry table
  //          1: success
  //          2: skipped filtered objects
  //          3: skipped new object (object's class UUID wasn't found in class list)
  ON_BOOL32 rc = -1;
  if ( pAttributes )
    pAttributes->Default();
  if ( !ppObject )
    return 0;
  *ppObject = 0;
  if ( !object_filter ) // default filter (0) reads every object
    object_filter = 0xFFFFFFFF;

  if ( m_3dm_version == 1 ) {
    rc = Read3dmV1Object(ppObject,pAttributes,object_filter);
  }
  else 
  {
    ON__UINT32 tcode = 0;
    ON__INT64 length_TCODE_OBJECT_RECORD = 0;
    ON__INT64 value_TCODE_OBJECT_RECORD_TYPE = 0;
    ON__INT64 length_TCODE_OBJECT_RECORD_ATTRIBUTES = 0;
    if ( BeginRead3dmBigChunk( &tcode, &length_TCODE_OBJECT_RECORD ) ) 
    {
      if ( tcode == TCODE_OBJECT_RECORD ) 
      {
        if (BeginRead3dmBigChunk( &tcode, &value_TCODE_OBJECT_RECORD_TYPE )) 
        {
          if ( tcode != TCODE_OBJECT_RECORD_TYPE ) {
            rc = -1;
            ON_ERROR("ON_BinaryArchive::Read3dmObject() - missing TCODE_OBJECT_RECORD_TYPE chunk.");
          }
          else if ( 0 != value_TCODE_OBJECT_RECORD_TYPE && 0 == (value_TCODE_OBJECT_RECORD_TYPE | object_filter) )
            rc = 2; // skip reading this object
          else
            rc = 1; // need to read this object

          if ( !EndRead3dmChunk() )
            rc = -1;

          switch(ReadObject(ppObject))
          {
          case 1:
            rc = 1; // successfully read this object
            break;
          case 3:
            rc = 3; // skipped object - assume it's just a newer object than this code reads
            break;
          default:
            rc = -1; // serious failure
            break;
          }
        }
        else
          rc = -1;
      }
      else if ( tcode != TCODE_ENDOFTABLE ) {
        ON_ERROR("ON_BinaryArchive::Read3dmObject() - corrupt object table");
        rc = -1;
      }
      else
        rc = 0;

      while(rc==1)
      {
        tcode = 0;
        if (!BeginRead3dmBigChunk( &tcode, &length_TCODE_OBJECT_RECORD_ATTRIBUTES )) {
          rc = -1;
          break;
        }
        if ( tcode == TCODE_OBJECT_RECORD_ATTRIBUTES ) 
        {
          if ( 0 != pAttributes )
          {
            if ( !pAttributes->Read( *this ) )
              rc = -1;
          }
        }
        else if ( tcode == TCODE_OBJECT_RECORD_ATTRIBUTES_USERDATA )
        {
          if ( 0 != pAttributes )
          {
            // 19 October 2004
            //   Added support for saving user data on object attributes
            if ( !ReadObjectUserData(*pAttributes))
              rc = -1;
            else
            {
#if 1
              // 3 March 2011 - convert obsolete user data
              ON_OBSOLETE_CCustomMeshUserData* ud = ON_OBSOLETE_CCustomMeshUserData::Cast(pAttributes->GetUserData(ON_OBSOLETE_CCustomMeshUserData::m_ON_OBSOLETE_CCustomMeshUserData_class_id.Uuid()));
              if ( ud )
              {
                ud->m_mp.m_bCustomSettingsEnabled = ud->m_bInUse ? true : false;
                pAttributes->SetCustomRenderMeshParameters(ud->m_mp);
                delete ud;
              }
#endif
            }
          }
        }

        if ( !EndRead3dmChunk() ) 
        {
          rc = -1;
        }
        if ( tcode == TCODE_OBJECT_RECORD_END )
          break;
      }

      if ( !EndRead3dmChunk() )
        rc = -1;
    }
  }

  return rc;
}

bool ON_BinaryArchive::EndRead3dmObjectTable()
{
  bool rc = EndRead3dmTable( TCODE_OBJECT_TABLE );

  if ( 0 != m_V1_layer_list )
  {
    struct ON__3dmV1LayerIndex* next = m_V1_layer_list;
    m_V1_layer_list = 0;
    for ( int i = 0; 0 != next && i < 1000; i++ )
    {
      struct ON__3dmV1LayerIndex* p = next;
      next = p->m_next;
      onfree(p);
    }
  }

  return rc;
}

bool ON_BinaryArchive::BeginWrite3dmUserTable( const ON_UUID& usertable_uuid )
{
  return BeginWrite3dmUserTable(usertable_uuid, false, 0, 0 );
}

bool ON_BinaryArchive::BeginWrite3dmUserTable(
    const ON_UUID& plugin_id,
    bool bSavingGoo,
    int goo_3dm_version,
    int goo_opennurbs_version
    )
{
  if ( m_active_table != no_active_table ) {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmUserTable() - m_active_table != no_active_table");
  }
  if ( !ON_UuidCompare( &ON_nil_uuid, &plugin_id ) ) {
    ON_ERROR("ON_BinaryArchive::BeginWrite3dmUserTable() - nil usertable_uuid not permitted.");
    return false;
  }
  if ( bSavingGoo )
  {
    if ( goo_3dm_version <= 3 )
      return false;
    if ( goo_opennurbs_version < 200601010 )
      return false;
    if ( goo_3dm_version >= 50 && Archive3dmVersion() < 50 )
    {
      // goo with 8 byte chunk lengths cannot be saved 
      // in files expecting 4 byte chunk lengths.
      return false;
    }
  }
  else
  {
    goo_3dm_version = Archive3dmVersion();
    goo_opennurbs_version = ArchiveOpenNURBSVersion();
  }

  bool rc = BeginWrite3dmTable( TCODE_USER_TABLE);
  if (rc) {
    rc = BeginWrite3dmChunk( TCODE_USER_TABLE_UUID, 0 );
    if (rc)
    {
      rc = WriteUuid( plugin_id );
      if (rc)
      {
        // The TCODE_USER_TABLE_RECORD_HEADER chunk was added in 
        // version 200910190 to contain the archive and opennurbs
        // version the plug-in used when writing the file.
        // This information is needed so "goo" can be correctly
        // read.
        rc = BeginWrite3dmChunk( TCODE_USER_TABLE_RECORD_HEADER, 1, 0 );
        if ( rc )
        {
          if (rc) rc = WriteBool(bSavingGoo);
          if (rc) rc = WriteInt(goo_3dm_version);
          if (rc) rc = WriteInt(goo_opennurbs_version);
          if ( !EndWrite3dmChunk() )
            rc = false;
        }
      }
      if ( !EndWrite3dmChunk() )
        rc = false;
    }
    if (rc) {
      rc = BeginWrite3dmChunk( TCODE_USER_RECORD, 0 );
    }
    if ( !rc ) {
      EndWrite3dmTable( TCODE_USER_TABLE);
    }
  }
  return rc;
}

bool ON_BinaryArchive::Write3dmAnonymousUserTableRecord( 
    const ON_UUID& plugin_id,
    int goo_3dm_version,
    int goo_opennurbs_version,
    const ON_3dmGoo& goo
    )
{
  if ( ON_UuidIsNil(plugin_id) )
    return false;
  if ( goo_3dm_version <= 3 )
    return false;
  if (goo_opennurbs_version < 200000000 )
    return false;
  if ( goo.m_typecode != TCODE_USER_RECORD )
    return false;
  if ( 0 == goo.m_value )
    return false;
  if ( 0 == goo.m_goo )
    return false;
  bool bSavingGoo = true;
  if ( !BeginWrite3dmUserTable( plugin_id, bSavingGoo, goo_3dm_version, goo_opennurbs_version ) )
    return false;
  bool rc = WriteByte( goo.m_value, goo.m_goo );
  if ( !EndWrite3dmUserTable() )
    rc = false;
  return rc;
}


bool ON_BinaryArchive::Write3dmAnonymousUserTable( const ON_3dmGoo& goo )
{
  bool rc = false;
  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( !c || c->m_typecode != TCODE_USER_RECORD ) {
    ON_ERROR("ON_BinaryArchive::Write3dmAnonymousUserTable() - active chunk not a TCODE_USER_RECORD.");
    rc = false;
  }
  else if ( goo.m_typecode != TCODE_USER_RECORD ) {
    ON_ERROR("ON_BinaryArchive::Write3dmAnonymousUserTable() - goo chunk not a TCODE_USER_RECORD.");
    rc = false;
  }
  else {
    rc = ( goo.m_value > 0 ) ? WriteByte( goo.m_value, goo.m_goo ) : true;
  }
  return rc;
}


bool ON_BinaryArchive::EndWrite3dmUserTable()
{
  bool rc = false;
  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( c && c->m_typecode == TCODE_USER_RECORD ) {
    rc = EndWrite3dmChunk();
  }
  else {
    ON_ERROR("ON_BinaryArchive::EndWrite3dmUserTable() - not in a TCODE_USER_RECORD chunk.");
    rc = false;
  }
  if ( !EndWrite3dmTable(TCODE_USER_TABLE) )
    rc = false;
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmUserTable( ON_UUID& usertable_uuid )
{
  // OBSOLETE - use the  BeginRead3dmUserTable() with more arguments
  bool bGoo = false;
  int archive_3dm_version = 0;
  int archive_opennurbs_version = 0;
  bool rc = BeginRead3dmUserTable( usertable_uuid, &bGoo, &archive_3dm_version, &archive_opennurbs_version );
  return rc;
}

bool ON_BinaryArchive::BeginRead3dmUserTable(
    ON_UUID& plugin_id,
    bool* bLastSavedAsGoo,
    int* archive_3dm_version,
    int* archive_opennurbs_version
    )
{
  bool bReadArchiveInfo = false;
  if ( bLastSavedAsGoo )
    *bLastSavedAsGoo = false;
  if ( archive_3dm_version )
    *archive_3dm_version = 0;
  if ( archive_opennurbs_version )
    *archive_opennurbs_version = 0;

  if ( m_3dm_version == 1 )
    return false;

  bool rc = BeginRead3dmTable( TCODE_USER_TABLE );

  // Do not add calls to EmergencyFindTable() here.
  // BeginRead3dmTable( TCODE_USER_TABLE ) returns
  // false when there are no user tables and that
  // is a common situation.

  if ( rc ) 
  {
    // read table id
    ON__UINT32 tcode = 0;
    ON__INT64 big_value = 0;
    if (rc) rc = BeginRead3dmBigChunk( &tcode, &big_value );
    if (rc)
    {
      if ( tcode != TCODE_USER_TABLE_UUID ) 
      {
        ON_ERROR("ON_BinaryArchive::BeginRead3dmUserTable() - missing user table UUID");
        rc = false;
      }
      else 
      {
        rc = ReadUuid( plugin_id );

        // Version 200910190 of OpenNURBS began writing a TCODE_USER_TABLE_RECORD_HEADER
        // section immediately after the uuid.  This was possible because the uuid
        // was wrapped in a TCODE_USER_TABLE_UUID chunk.  The TCODE_USER_TABLE_RECORD_HEADER
        // contains information that let's us determine what version of Rhino and
        // opennurbs wrote the user table.  We need to know this because "goo"
        // can have chunks with 4 byte lengths embedded in a file with 8 byte
        // chunk lengths.  If this information is missing, then we know the "goo"
        // must have 4 byte chunk lengths and we assume it is from a V4 file.
        //
        // 37 + SizeofChunkLength() = 
        //      16 bytes of uuid
        //    + 4 bytes of TCODE_USER_TABLE_RECORD_HEADER typecode
        //    + SizeofChunkLength() TCODE_USER_TABLE_RECORD_HEADER chunk length
        //    + 1 byte of bSlastSavedAsGoo bool
        //    + 4 bytes of archive_3dm_version
        //    + 4 bytes of archive_opennurbs_version
        //    + 4 bytes of TCODE_USER_TABLE_RECORD_HEADER chunk crc
        //    + 4 bytes of TCODE_USER_TABLE_UUID chunk crc
        const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
        if ( rc 
             && ArchiveOpenNURBSVersion() >= 200910190 
             && 0 != c
             && TCODE_USER_TABLE_UUID == c->m_typecode
             && c->Length() >= 45 + SizeofChunkLength()
           )
        {
          int major_chunk_version = 0;
          int minor_chunk_version = 0;
          rc = BeginRead3dmChunk(TCODE_USER_TABLE_RECORD_HEADER,&major_chunk_version,&minor_chunk_version);
          if (rc)
          {
            bReadArchiveInfo = true;
            bool b = true;
            int arch_ver = 0;
            int on_ver = 0;
            rc = ReadBool(&b);
            if ( rc && bLastSavedAsGoo )
              *bLastSavedAsGoo = b;
            if (rc)
              rc = ReadInt(&arch_ver);
            if (rc && archive_3dm_version)
              *archive_3dm_version = arch_ver;
            if (rc)
              rc = ReadInt(&on_ver);
            if (rc && archive_opennurbs_version)
              *archive_opennurbs_version = on_ver;
            if ( !EndRead3dmChunk() )
              rc = false;
          }
        }
      }
      if ( !EndRead3dmChunk() )
        rc = false;
    }

    tcode = 0;
    big_value = 0;
    if (rc) rc = BeginRead3dmBigChunk( &tcode, &big_value );
    if (rc) {
      if ( tcode != TCODE_USER_RECORD ) {
        ON_ERROR("ON_BinaryArchive::BeginRead3dmUserTable() - missing user table TCODE_USER_RECORD chunk.");
        EndRead3dmChunk();
        rc = false;
      }
    }

    if (!rc)
      EndRead3dmTable(TCODE_USER_TABLE);

    if ( rc && !bReadArchiveInfo )
    {
      // If the file we are reading is V4 or an early V5 file, then use the 
      // version numbers from the file.  Otherwise, assume the goo is from
      // an early V5 file.  All we know for sure is that the chunk lengths
      // in the user table are 4 bytes.
      if ( Archive3dmVersion() < 50 )
      {
        if (archive_3dm_version)
          *archive_3dm_version = Archive3dmVersion();
        if (archive_opennurbs_version)
          *archive_opennurbs_version = ArchiveOpenNURBSVersion();
      }
      else
      {
        if (archive_3dm_version) 
          *archive_3dm_version = 5;
        if (archive_opennurbs_version) 
          *archive_opennurbs_version = 200910180;
      }
    }
  }


  return rc;
}

bool ON_BinaryArchive::Read3dmAnonymousUserTable( ON_3dmGoo& goo )
{
  return Read3dmAnonymousUserTable(0,0,goo);
}

bool ON_BinaryArchive::Read3dmAnonymousUserTable( 
    int archive_3dm_version,
    int archive_opennurbs_version,
    ON_3dmGoo& goo
    )
{
  if ( 0 == archive_3dm_version )
  {
    if ( Archive3dmVersion() < 50 )
    {
      archive_3dm_version = Archive3dmVersion();
      archive_opennurbs_version = ArchiveOpenNURBSVersion();
    }
    else
    {
      // recent version with 4 byte chunk lengths.
      archive_3dm_version = 5;
      archive_opennurbs_version = 200910190;
    }
  }
  bool rc = Read3dmGoo( goo );
  if (rc && goo.m_typecode != TCODE_USER_RECORD ) {
    ON_ERROR("ON_BinaryArchive::Read3dmAnonymousUserTable() do not read a TCODE_USER_RECORD chunk.");
    rc = false;
  }
  return rc;
}

bool ON_BinaryArchive::EndRead3dmUserTable()
{
  if ( m_chunk.Count() != 2 ) {
    ON_ERROR("ON_BinaryArchive::EndRead3dmUserTable() m_chunk.Count() != 2");
    return false;
  }
  const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
  if ( 0 == c || c->m_typecode != TCODE_USER_RECORD ) {
    ON_ERROR("ON_BinaryArchive::EndRead3dmTable() m_chunk.Last()->typecode != TCODE_USER_RECORD");
    return false;
  }


  // end of TCODE_USER_RECORD chunk
  // Suppress the partially read chunk warning because plug-in IO
  // is too upredictable for this warning to be helpful.
  bool rc = EndRead3dmChunk(true); 

  if (rc) {
    // end of table chunk
    unsigned int tcode = 0;
    ON__INT64 big_value = -1;
    rc = BeginRead3dmBigChunk( &tcode, &big_value );
    if ( rc ) {
      if ( tcode != TCODE_ENDOFTABLE ) {
        ON_ERROR("ON_BinaryArchive::EndRead3dmTable() missing TCODE_ENDOFTABLE marker.");
      }
      if ( !EndRead3dmChunk() )
        rc = false;
    }
  }

  if ( !EndRead3dmTable(TCODE_USER_TABLE) )
    rc = false;
  return rc;
}

bool ON_BinaryArchive::Write3dmEndMark()
{
  Flush();
  if ( m_chunk.Count() != 0 ) {
    ON_ERROR( "ON_BinaryArchive::WriteEndMark() called with unfinished chunks.\n" );
    return false;
  }
  ON__UINT64 length = CurrentPosition(); // since no chunks are unfinished, everything
                                  // has been committed to disk in either
                                  // write mode.
  bool rc = BeginWrite3dmChunk( TCODE_ENDOFFILE, 0 );
  if ( rc ) 
  {    
    size_t sizeof_chunk_length = SizeofChunkLength();
    size_t sizeoffile_length = (8==SizeofChunkLength()) ? 8 : 4;
    length += (4 + sizeof_chunk_length + sizeoffile_length );
    rc = WriteEOFSizeOfFile(length);
    if ( !EndWrite3dmChunk() )
      rc = false;
  }
  Flush();

  return rc;
}

bool ON_BinaryArchive::Read3dmEndMark( size_t* file_length )
{
  unsigned int tcode=0;
  ON__INT64 value=0;
  if ( file_length )
    *file_length = 0;

  const unsigned int saved_error_message_mask = m_error_message_mask;
  m_error_message_mask |= 0x0001; // disable v1 ReadByte() error message at EOF
  bool rc = PeekAt3dmBigChunkType(&tcode,&value);
  m_error_message_mask = saved_error_message_mask;

  if (rc)
  {
    if ( tcode == TCODE_ENDOFFILE ) 
    {
      rc = BeginRead3dmBigChunk(&tcode,&value);
      if ( rc && value > 0 && ((ON__UINT64)value) >= SizeofChunkLength() )
      {
        ON__UINT64 u64 = 0;
        rc = ReadEOFSizeOfFile( &u64 );
        if ( rc && file_length )
          *file_length = (size_t)u64;
        if ( !EndRead3dmChunk() )
          rc = false;
      }
    }
  }
  return rc;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

ON::endian
ON_BinaryArchive::Endian() const // endian-ness of cpu
{
  return m_endian;
}

ON::archive_mode
ON_BinaryArchive::Mode() const
{
  return m_mode;
}

void ON_BinaryArchive::UpdateCRC( size_t count, const void* p )
{
  if ( m_bDoChunkCRC ) {
    ON_3DM_BIG_CHUNK* c = m_chunk.Last();
    if (c) {
      if ( c->m_do_crc16 )
        c->m_crc16 = ON_CRC16( c->m_crc16, count, p ); // version 1 files had 16 bit CRC
      if ( c->m_do_crc32 )
        c->m_crc32 = ON_CRC32( c->m_crc32, count, p ); // version 2 files have 32 bit CRC
    }
  }
}

int ON_BinaryArchive::BadCRCCount() const
{
  return m_bad_CRC_count;
}

unsigned int ON_BinaryArchive::ErrorMessageMask() const
{
  return m_error_message_mask;
}

bool ON_BinaryArchive::MaskReadError( ON__UINT64 sizeof_request, ON__UINT64 sizeof_read ) const
{
  if ( sizeof_request == sizeof_read )
    return true; // no error
  if ( sizeof_request > sizeof_read )
    return false; // something is seriously wrong
  if ( 0 != (0x04 & m_error_message_mask) )
    return true;
  if ( 0 != (0x01 & m_error_message_mask) && 4 == sizeof_request && 0 == sizeof_read )
    return true;
  return false; // parial read not permitted at this time.
}

ON__UINT64 ON_BinaryArchive::ReadBuffer( ON__UINT64 sizeof_buffer, void* buffer )
{
  // Expert user function to load a buffer with up to sizeof_buffer
  // bytes but tolerate encountering the end of the file.
  if ( 0 == buffer )
    return 0;
  unsigned int saved_error_mask = m_error_message_mask;
  m_error_message_mask |= 0x04; // tell Read to tolerate hitting the end of the file
  ON__UINT64 sizeof_read = Read((size_t)sizeof_buffer,buffer);
  m_error_message_mask = saved_error_mask;
  return sizeof_read;
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

ON_BinaryFile::ON_BinaryFile( ON::archive_mode mode ) 
              : ON_BinaryArchive( mode ), 
                m_fp(0),
                m_memory_buffer_capacity(0),
                m_memory_buffer_size(0),
                m_memory_buffer_ptr(0),
                m_memory_buffer(0)
{}

ON_BinaryFile::ON_BinaryFile( ON::archive_mode mode, FILE* fp ) 
              : ON_BinaryArchive( mode ), 
                m_fp(fp),
                m_memory_buffer_capacity(0),
                m_memory_buffer_size(0),
                m_memory_buffer_ptr(0),
                m_memory_buffer(0)
{}

ON_BinaryFile::~ON_BinaryFile()
{
  EnableMemoryBuffer(0);
}

bool
ON_BinaryArchive::ReadByte( size_t count, void* p )
{
  bool rc = false;
  if ( count > 0 ) {
    if ( !ReadMode() ) {
      ON_ERROR("ON_BinaryArchive::ReadByte() ReadMode() is false.");
    }
    else if ( p ) 
    {

#if defined(ON_DEBUG)
      {
        // this is slow (becuase of the call to CurrentPosition(), but it really helps find bugs in IO code
        const ON_3DM_BIG_CHUNK* c = m_chunk.Last();
        if ( c )
        {
          ON__UINT64 file_offset = CurrentPosition();
          if ( file_offset < c->m_big_offset )
          {
            ON_ERROR("ON_BinaryArchive::ReadByte will read before the start of the chunk.");
          }
          else if (c->m_big_offset + c->Length() < file_offset + count )
          {
            ON_ERROR("ON_BinaryArchive::ReadByte will read past end of the chunk");
          }
        }
      }
#endif

      size_t readcount = Read( count, p );
      if ( readcount == count ) 
      {
        UpdateCRC( count, p );
        rc = true;  
      }
      else 
      {
        // see if this is an error condition
        for(;;)
        {
          if ( 0 != (m_error_message_mask&0x01)
               && 0 == readcount && 4 == count 
             )
          {
            // when reading v1 files, there are some situations where
            // it is reasonable to attempt to read 4 bytes at the end
            // of a file.
            break;
          }
          if (    0 == m_3dm_version
               && 0 == m_3dm_opennurbs_version
               && 0 == m_3dm_start_section_offset
               && ON_BinaryArchive::no_active_table == m_active_table
               && 0 == m_chunk
               && ON::read3dm == m_mode
               )
          {
            // In Read3dmStartSection(), we search for the string 
            // "3D Geometry File Format ...".  When a non-.3dm file
            // is searched, we eventually reach the end of the file.
            // This error condition is reported by the returning
            // false from ON_BinaryArchive::Read3dmStartSection().
            // ON_ERROR is not called to prevent annoying everyone
            // when the open file dialog is digging around looking
            // for files.
            break;
          }
          ON_ERROR("ON_BinaryArchive::ReadByte() Read() failed.");
          break;
        }
      }
    }
    else 
    {
      ON_ERROR("ON_BinaryArchive::ReadByte() NULL file or buffer.");
    }
  }
  else 
  {
    rc = true;
  }
  return rc;
}

bool
ON_BinaryArchive::WriteByte( size_t count, const void* p )
{
  bool rc = false;
  if ( count > 0 ) {
    if ( !WriteMode() ) {
      ON_ERROR("ON_BinaryArchive::WriteByte() WriteMode() is false.");
    }
    else if ( p ) {
      size_t writecount = (size_t)Write( count, p );
      if ( writecount == count ) {
        UpdateCRC( count, p );
        rc = true;      
      }
      else {
        ON_ERROR("ON_BinaryArchive::WriteByte() fwrite() failed.");
      }
    }
    else {
      ON_ERROR("ON_BinaryArchive::WriteByte() NULL file or buffer.");
    }
  }
  else {
    rc = true;
  }
  return rc;
}

bool ON_BinaryArchive::EnableCRCCalculation( bool bEnable )
{
  bool rc = m_bDoChunkCRC;
  m_bDoChunkCRC = bEnable ? true : false;
  return rc;
}

bool
ON_BinaryArchive::SetArchive3dmVersion(int v)
{
  bool rc = false;
  // valid versions are 1,2,3,4,5,50,60,70,...
  // 1 - 4 correspond to Rhino V1 - V4.
  // 5 was used for early V5 betas with 4 byte chunk lengths
  // 50 is used for V5 files with 8 bytes chunk lengths.
  // 60, 70, ... will be used for Rhino V6, V7, etc.
  if ( (v >= 1 && v <= 5) || ( v >= 50 && 0 == (v % 10) ) )
  {
    m_3dm_version = v;
    rc = true;
  }
  else 
  {
    m_3dm_version = 0;
    ON_ERROR("ON_BinaryArchive::SetArchive3dmVersion - invalid version");
  }
  return rc;
}

void 
ON_BinaryFile::EnableMemoryBuffer(
       int buffer_capacity
       )
{
  if ( buffer_capacity > 0 && !m_memory_buffer) {
    m_memory_buffer = (unsigned char*)onmalloc(buffer_capacity);
    if ( m_memory_buffer ) {
      m_memory_buffer_capacity = buffer_capacity;
      m_memory_buffer_size = 0;
      m_memory_buffer_ptr = 0;
    }
  }
  else {
    if ( buffer_capacity == 0 && m_memory_buffer ) {
      Flush();
      onfree(m_memory_buffer);
    }
    m_memory_buffer = 0;
    m_memory_buffer_capacity = 0;
    m_memory_buffer_size = 0;
    m_memory_buffer_ptr = 0;
  }
}


size_t ON_BinaryFile::Read( size_t count, void* p )
{
  return (m_fp) ? fread( p, 1, count, m_fp ) : 0;
}

size_t ON_BinaryFile::Write( size_t count, const void* p )
{
  size_t rc = 0;
  if ( m_fp ) 
  {
    if ( m_memory_buffer ) 
    {
      if ( count+m_memory_buffer_ptr >= m_memory_buffer_capacity ) {
        if ( !Flush() ) // flush existing memory buffer to disk
          return 0;
        rc = fwrite( p, 1, count, m_fp ); // write directly to disk
      }
      else 
      {
        // copy to the memory buffer
        memcpy( m_memory_buffer+m_memory_buffer_ptr, p, count );
        m_memory_buffer_ptr += count;
        if ( m_memory_buffer_ptr > m_memory_buffer_size )
          m_memory_buffer_size = m_memory_buffer_ptr;
        rc = count;
      }      
    }
    else
    {
      rc = fwrite( p, 1, count, m_fp );
    }
  }
  return rc;
}

bool ON_BinaryFile::Flush()
{
  bool rc = true;
  if ( m_fp ) 
  {
    if ( m_memory_buffer && m_memory_buffer_size > 0 ) 
    {
      rc = ( m_memory_buffer_size == fwrite( m_memory_buffer, 1, m_memory_buffer_size, m_fp ));   
      if ( rc && m_memory_buffer_ptr != m_memory_buffer_size ) 
      {
        //if ( !fseek( m_fp, m_memory_buffer_size-m_memory_buffer_ptr, SEEK_CUR ) )
        int delta =  (m_memory_buffer_ptr >= m_memory_buffer_size) 
                  ?  ((int)(m_memory_buffer_ptr - m_memory_buffer_size))
                  : -((int)(m_memory_buffer_size - m_memory_buffer_ptr));
        if ( !fseek( m_fp, delta, SEEK_CUR ) )          
        {
          rc = false;
        }
      }
      m_memory_buffer_size = 0;
      m_memory_buffer_ptr = 0;
    }
  }
  return rc;
}

size_t ON_BinaryFile::CurrentPosition() const
{
  size_t offset = 0;

  if ( 0 != m_fp ) 
  {

#if defined(ON_COMPILER_MSC)
    // use an ftell() that supports file offsets > 2GB
    ON__INT64 offset64 = _ftelli64(m_fp);
    if ( offset64 < 0 )
    {
      ON_ERROR("ON_BinaryFile::CurrentPosition() - _ftelli64() failed");
    }
    else
    {
      offset = (size_t)((ON__UINT64)offset64);
    }
#else
    offset = ftell(m_fp);
#endif

    if ( m_memory_buffer && m_memory_buffer_size > 0 ) 
    {
      offset += m_memory_buffer_ptr;
    }
  }
  else 
  {
    ON_ERROR("ON_BinaryFile::CurrentPosition() NULL file.");
  }

  return offset;
}

bool ON_BinaryFile::AtEnd() const
{
  bool rc = true;
  if ( m_fp ) {
    rc = false;
    if ( ReadMode() ) {
      if ( feof( m_fp ) ) {
        rc = true;
      }
      else 
      {
        int buffer;
        fread( &buffer, 1, 1, m_fp );
        if ( feof( m_fp ) ) 
        {
          rc = true;
        }
        else 
        {
          // back up to the byte we just read
          fseek( m_fp, -1, SEEK_CUR );
        }
      }
    }
  }
  return rc;
}

bool ON_BinaryFile::SeekFromCurrentPosition( int offset )
{
  // it appears that the calls to SeekFromCurrentPosition(),
  // and consequent call to fseek(), in ON_BinaryArchive::EndWrite3DMChunk()
  // really slow down 3DM file writing on some networks.  There are
  // reports of a 100x difference in local vs network saves.
  // I could not reproduce these 100x slow saves in testing on McNeel's
  // network but we have a good quality network and a server that's
  // properly tuned.  My guess is that the slow saves are happening
  // on servers that do a poor job of cacheing because they are starved
  // for memory and/or heavily used at the time of the save.
  //
  // To speed up network saves, ON_BinaryFile can optionally use
  // it's own buffer for buffered I/O instead of relying on fwrite()
  // and the OS to handle this.
  bool rc = false;
  if ( m_fp ) 
  {
    if ( m_memory_buffer && 
         ((ON__INT_PTR)m_memory_buffer_ptr)+((ON__INT_PTR)offset) >= 0 &&
         m_memory_buffer_ptr+offset <= m_memory_buffer_size ) {
      m_memory_buffer_ptr += offset;
      rc = true;
    }
    else
    {
      // don't deal with memory buffer I/O if seek lands outside of
      // our current memory buffer.
      Flush();
      if ( !fseek(m_fp,offset,SEEK_CUR) )
      {
        rc = true;
      }
      else 
      {
        ON_ERROR("ON_BinaryFile::Seek() fseek(,SEEK_CUR) failed.");
      }
    }
  }
  return rc;
}

bool ON_BinaryFile::SeekFromEnd( int offset )
{
  bool rc = false;
  if ( m_fp ) 
  {
    Flush(); // don't deal with memory buffer I/O in rare seek from end
    if ( !fseek(m_fp,offset,SEEK_END) )
    {
      rc = true;
    }
    else 
    {
      ON_ERROR("ON_BinaryFile::SeekFromEnd() fseek(,SEEK_END) failed.");
    }
  }
  return rc;
}

bool ON_BinaryFile::SeekFromStart( size_t offset )
{
  bool rc = false;
  if ( m_fp ) 
  {
    Flush(); // don't deal with memory buffer I/O in rare seek from start
    long loffset = (long)offset;
    if ( !fseek(m_fp,loffset,SEEK_SET) )
    {
      rc = true;
    }
    else 
    {
      ON_ERROR("ON_BinaryFile::SeekFromStart() fseek(,SEEK_SET) failed.");
    }
  }
  return rc;
}

ON_3dmGoo::ON_3dmGoo()
        : m_typecode(0),
          m_value(0),
          m_goo(0),
          m_next_goo(0),
          m_prev_goo(0)
{}

ON_3dmGoo::~ON_3dmGoo()
{
  if ( m_prev_goo )
    m_prev_goo->m_next_goo = m_next_goo;
  if ( m_next_goo )
    m_next_goo->m_prev_goo = m_prev_goo;
  if ( m_goo ) {
    onfree(m_goo);
    m_goo = 0;
  }
}

ON_3dmGoo::ON_3dmGoo( const ON_3dmGoo& src )
        : m_typecode(0),
          m_value(0),
          m_goo(0),
          m_next_goo(0),
          m_prev_goo(0)
{
  *this = src;
}

ON_3dmGoo& ON_3dmGoo::operator=( const ON_3dmGoo& src )
{
  if ( this != &src ) {
    if ( m_goo ) {
      onfree(m_goo);
    }
    m_typecode = src.m_typecode;
    m_value = src.m_value;
    m_goo = (m_value > 0 && src.m_goo) ? (unsigned char*)onmemdup( src.m_goo, m_value ) : 0;
  }
  return *this;
}

void ON_3dmGoo::Dump(ON_TextLog& dump) const
{
  dump.Print("typecode = %08x value = %d\n",m_typecode,m_value);
}

bool ON_WriteOneObjectArchive( 
          ON_BinaryArchive& archive,
          int version,
          const ON_Object& object
          )
{
  bool rc = false;

  const ON_Object* pObject = &object;
  {
    if ( ON_BrepEdge::Cast(pObject) )
    {
      // write parent brep
      pObject = static_cast<const ON_BrepEdge*>(pObject)->Brep();
    }
    else if ( ON_BrepTrim::Cast(pObject) )
    {
      pObject = NULL;
    }
    else if ( ON_BrepLoop::Cast(pObject) )
    {
      pObject = static_cast<const ON_BrepLoop*>(pObject)->Brep();
    }
    else if ( ON_BrepFace::Cast(pObject) )
    {
      // write parent brep
      pObject = static_cast<const ON_BrepFace*>(pObject)->Brep();
    }
    else if ( ON_CurveProxy::Cast(pObject) )
    {
      // write actual curve
      pObject = static_cast<const ON_CurveProxy*>(pObject)->ProxyCurve();
    }
    else if ( ON_SurfaceProxy::Cast(pObject) )
    {
      // write actual surface
      pObject = static_cast<const ON_SurfaceProxy*>(pObject)->ProxySurface();
    }
  }
  
  ON_3dmProperties props;
  props.m_RevisionHistory.NewRevision();
  
  ON_3dmSettings settings;
  // 1 Feb 2012 Dale Lear 
  //   http://dev.mcneel.com/bugtrack/?q=98543
  //   Single object archives have no unit system so they
  //   can be read into a file with no scaling.  Prior to
  //   today the unit system was always millimeters.
  settings.m_ModelUnitsAndTolerances.m_unit_system.m_unit_system = ON::no_unit_system;

  ON_Layer layer;
  ON_3dmObjectAttributes attributes;

  // layer table will have one layer
  layer.SetLayerIndex(0);
  layer.SetLayerName(L"Default");
  
  // object attributes
  attributes.m_layer_index = 0;

  while(pObject)
  {
    rc = archive.Write3dmStartSection( version, "Archive created by ON_WriteOneObjectArchive " __DATE__ " " __TIME__ );
    if ( !rc )
      break;

    version = archive.Archive3dmVersion();

    rc = archive.Write3dmProperties( props );
    if ( !rc )
      break;

    rc = archive.Write3dmSettings( settings );
    if ( !rc )
      break;

    rc = archive.BeginWrite3dmBitmapTable();
    if ( !rc )
      break;
    rc = archive.EndWrite3dmBitmapTable();
    if ( !rc )
      break;

    if ( version >= 4 )
    {
      rc = archive.BeginWrite3dmTextureMappingTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmTextureMappingTable();
      if ( !rc )
        break;
    }

    rc = archive.BeginWrite3dmMaterialTable();
    if ( !rc )
      break;
    rc = archive.EndWrite3dmMaterialTable();
    if ( !rc )
      break;

    if ( version >= 4 )
    {
      rc = archive.BeginWrite3dmLinetypeTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmLinetypeTable();
      if ( !rc )
        break;
    }

    rc = archive.BeginWrite3dmLayerTable();
    if ( !rc )
      break;
    {
      rc = archive.Write3dmLayer(layer);
    }
    if (!archive.EndWrite3dmLayerTable())
      rc = false;
    if ( !rc )
      break;

    rc = archive.BeginWrite3dmGroupTable();
    if ( !rc )
      break;
    rc = archive.EndWrite3dmGroupTable();
    if ( !rc )
      break;

    if ( version >= 3 )
    {
      rc = archive.BeginWrite3dmFontTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmFontTable();
      if ( !rc )
        break;
    }

    if ( version >= 3 )
    {
      rc = archive.BeginWrite3dmDimStyleTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmDimStyleTable();
      if ( !rc )
        break;
    }

    rc = archive.BeginWrite3dmLightTable();
    if ( !rc )
      break;
    rc = archive.EndWrite3dmLightTable();
    if ( !rc )
      break;

    if ( version >= 4 )
    {
      rc = archive.BeginWrite3dmHatchPatternTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmHatchPatternTable();
      if ( !rc )
        break;
    }

    if ( version >= 3 )
    {
      rc = archive.BeginWrite3dmInstanceDefinitionTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmInstanceDefinitionTable();
      if ( !rc )
        break;
    }

    rc = archive.BeginWrite3dmObjectTable();
    if ( !rc )
      break;
    {
      rc = archive.Write3dmObject( *pObject, &attributes );
    }
    if ( !archive.EndWrite3dmObjectTable() )
      rc = false;
    if ( !rc )
      break;

    if ( version >= 4 )
    {
      rc = archive.BeginWrite3dmHistoryRecordTable();
      if ( !rc )
        break;
      rc = archive.EndWrite3dmHistoryRecordTable();
      if ( !rc )
        break;
    }

    rc = archive.Write3dmEndMark();

    break;
  }

  return rc;
}

static 
void Dump3dmChunk_ErrorReportHelper( size_t offset, const char* msg, ON_TextLog& dump )
{
  int ioffset = (int)offset;
  dump.Print("** ERROR near offset %d ** %s\n",ioffset,msg);
}
static
bool DumpChunk_PrintHeaderInfo( size_t offset0, ON__UINT32 typecode, ON__INT64 big_value, const char* typecode_name, ON_TextLog& dump)
{
  bool bShortChunk = (0 != (typecode & TCODE_SHORT));
  if ( 0 == typecode_name )
    typecode_name = ON_BinaryArchive::TypecodeName(typecode);
  if ( 0 == typecode_name )
    typecode_name = "unknown tcode";
  if ( bShortChunk )
  {
    dump.Print("%6d: %08X %s: value = %lld (%016llX)\n", offset0, typecode, typecode_name, big_value, big_value );
  }
  else 
  {
    // long chunk value = length of chunk data
    if ( big_value < 0 ) 
    {
      Dump3dmChunk_ErrorReportHelper(offset0,"BeginRead3dmChunk() returned length < 0.",dump);
      return false;
    }
    dump.Print("%6d: %08X %s: length = %lld bytes\n", offset0, typecode, typecode_name, big_value );
  }
  return true;
}

#define CASEtcode2string(tc) case tc: s = #tc ; break

const char* ON_BinaryArchive::TypecodeName( unsigned int tcode )
{

  const char* s;
  switch( tcode ) 
  {
  CASEtcode2string(TCODE_FONT_TABLE);
  CASEtcode2string(TCODE_FONT_RECORD);
  CASEtcode2string(TCODE_DIMSTYLE_TABLE);
  CASEtcode2string(TCODE_DIMSTYLE_RECORD);
  CASEtcode2string(TCODE_INSTANCE_DEFINITION_RECORD);
  CASEtcode2string(TCODE_COMMENTBLOCK);
  CASEtcode2string(TCODE_ENDOFFILE);
  CASEtcode2string(TCODE_ENDOFFILE_GOO);
  CASEtcode2string(TCODE_LEGACY_GEOMETRY);
  CASEtcode2string(TCODE_OPENNURBS_OBJECT);
  CASEtcode2string(TCODE_GEOMETRY);
  CASEtcode2string(TCODE_ANNOTATION);
  CASEtcode2string(TCODE_DISPLAY);
  CASEtcode2string(TCODE_RENDER);
  CASEtcode2string(TCODE_INTERFACE);
  CASEtcode2string(TCODE_TOLERANCE);
  CASEtcode2string(TCODE_TABLE);
  CASEtcode2string(TCODE_TABLEREC);
  CASEtcode2string(TCODE_USER);
  CASEtcode2string(TCODE_SHORT);
  CASEtcode2string(TCODE_CRC);
  CASEtcode2string(TCODE_ANONYMOUS_CHUNK);
  CASEtcode2string(TCODE_MATERIAL_TABLE);
  CASEtcode2string(TCODE_LAYER_TABLE);
  CASEtcode2string(TCODE_LIGHT_TABLE);
  CASEtcode2string(TCODE_OBJECT_TABLE);
  CASEtcode2string(TCODE_PROPERTIES_TABLE);
  CASEtcode2string(TCODE_SETTINGS_TABLE);
  CASEtcode2string(TCODE_BITMAP_TABLE);
  CASEtcode2string(TCODE_USER_TABLE);
  CASEtcode2string(TCODE_INSTANCE_DEFINITION_TABLE);
  CASEtcode2string(TCODE_HATCHPATTERN_TABLE);
  CASEtcode2string(TCODE_HATCHPATTERN_RECORD);
  CASEtcode2string(TCODE_LINETYPE_TABLE);
  CASEtcode2string(TCODE_LINETYPE_RECORD);
  CASEtcode2string(TCODE_OBSOLETE_LAYERSET_TABLE);
  CASEtcode2string(TCODE_OBSOLETE_LAYERSET_RECORD);
  CASEtcode2string(TCODE_TEXTURE_MAPPING_TABLE);
  CASEtcode2string(TCODE_TEXTURE_MAPPING_RECORD);
  CASEtcode2string(TCODE_HISTORYRECORD_TABLE);  
  CASEtcode2string(TCODE_HISTORYRECORD_RECORD);
  CASEtcode2string(TCODE_ENDOFTABLE);
  CASEtcode2string(TCODE_PROPERTIES_REVISIONHISTORY);
  CASEtcode2string(TCODE_PROPERTIES_NOTES);
  CASEtcode2string(TCODE_PROPERTIES_PREVIEWIMAGE);
  CASEtcode2string(TCODE_PROPERTIES_COMPRESSED_PREVIEWIMAGE);
  CASEtcode2string(TCODE_PROPERTIES_APPLICATION);
  CASEtcode2string(TCODE_PROPERTIES_OPENNURBS_VERSION);
  CASEtcode2string(TCODE_SETTINGS_PLUGINLIST);
  CASEtcode2string(TCODE_SETTINGS_UNITSANDTOLS);
  CASEtcode2string(TCODE_SETTINGS_RENDERMESH);
  CASEtcode2string(TCODE_SETTINGS_ANALYSISMESH);
  CASEtcode2string(TCODE_SETTINGS_ANNOTATION);
  CASEtcode2string(TCODE_SETTINGS_NAMED_CPLANE_LIST);
  CASEtcode2string(TCODE_SETTINGS_NAMED_VIEW_LIST);
  CASEtcode2string(TCODE_SETTINGS_VIEW_LIST);
  CASEtcode2string(TCODE_SETTINGS_CURRENT_LAYER_INDEX);
  CASEtcode2string(TCODE_SETTINGS_CURRENT_MATERIAL_INDEX);
  CASEtcode2string(TCODE_SETTINGS_CURRENT_COLOR);
  CASEtcode2string(TCODE_SETTINGS__NEVER__USE__THIS);
  CASEtcode2string(TCODE_SETTINGS_CURRENT_WIRE_DENSITY);
  CASEtcode2string(TCODE_SETTINGS_RENDER);
  CASEtcode2string(TCODE_SETTINGS_GRID_DEFAULTS);
  CASEtcode2string(TCODE_SETTINGS_MODEL_URL);
  CASEtcode2string(TCODE_SETTINGS_CURRENT_FONT_INDEX);
  CASEtcode2string(TCODE_SETTINGS_CURRENT_DIMSTYLE_INDEX);
  CASEtcode2string(TCODE_SETTINGS_ATTRIBUTES);
  CASEtcode2string(TCODE_VIEW_RECORD);
  CASEtcode2string(TCODE_VIEW_CPLANE);
  CASEtcode2string(TCODE_VIEW_VIEWPORT);
  CASEtcode2string(TCODE_VIEW_VIEWPORT_USERDATA);
  CASEtcode2string(TCODE_VIEW_SHOWCONGRID);
  CASEtcode2string(TCODE_VIEW_SHOWCONAXES);
  CASEtcode2string(TCODE_VIEW_SHOWWORLDAXES);
  CASEtcode2string(TCODE_VIEW_TRACEIMAGE);
  CASEtcode2string(TCODE_VIEW_WALLPAPER);
  CASEtcode2string(TCODE_VIEW_WALLPAPER_V3);
  CASEtcode2string(TCODE_VIEW_TARGET);
  CASEtcode2string(TCODE_VIEW_DISPLAYMODE);
  CASEtcode2string(TCODE_VIEW_NAME);
  CASEtcode2string(TCODE_VIEW_POSITION);
  CASEtcode2string(TCODE_VIEW_ATTRIBUTES);
  CASEtcode2string(TCODE_BITMAP_RECORD);
  CASEtcode2string(TCODE_MATERIAL_RECORD);
  CASEtcode2string(TCODE_LAYER_RECORD);
  CASEtcode2string(TCODE_LIGHT_RECORD);
  CASEtcode2string(TCODE_LIGHT_RECORD_ATTRIBUTES);
  CASEtcode2string(TCODE_OBJECT_RECORD_ATTRIBUTES_USERDATA);
  CASEtcode2string(TCODE_OBJECT_RECORD_HISTORY);
  CASEtcode2string(TCODE_OBJECT_RECORD_HISTORY_HEADER);
  CASEtcode2string(TCODE_OBJECT_RECORD_HISTORY_DATA);
  CASEtcode2string(TCODE_LIGHT_RECORD_END);
  CASEtcode2string(TCODE_USER_TABLE_UUID);
  CASEtcode2string(TCODE_USER_TABLE_RECORD_HEADER);
  CASEtcode2string(TCODE_USER_RECORD);
  CASEtcode2string(TCODE_GROUP_TABLE);
  CASEtcode2string(TCODE_GROUP_RECORD);
  CASEtcode2string(TCODE_OBJECT_RECORD);
  CASEtcode2string(TCODE_OBJECT_RECORD_TYPE);
  CASEtcode2string(TCODE_OBJECT_RECORD_ATTRIBUTES);
  CASEtcode2string(TCODE_OBJECT_RECORD_END);
  CASEtcode2string(TCODE_OPENNURBS_CLASS);
  CASEtcode2string(TCODE_OPENNURBS_CLASS_UUID);
  CASEtcode2string(TCODE_OPENNURBS_CLASS_DATA);
  CASEtcode2string(TCODE_OPENNURBS_CLASS_USERDATA);
  CASEtcode2string(TCODE_OPENNURBS_CLASS_USERDATA_HEADER);
  CASEtcode2string(TCODE_OPENNURBS_CLASS_END);
  CASEtcode2string(TCODE_OPENNURBS_BUFFER);
  CASEtcode2string(TCODE_ANNOTATION_SETTINGS);
  CASEtcode2string(TCODE_TEXT_BLOCK);
  CASEtcode2string(TCODE_ANNOTATION_LEADER);
  CASEtcode2string(TCODE_LINEAR_DIMENSION);
  CASEtcode2string(TCODE_ANGULAR_DIMENSION);
  CASEtcode2string(TCODE_RADIAL_DIMENSION);
  CASEtcode2string(TCODE_RHINOIO_OBJECT_NURBS_CURVE);
  CASEtcode2string(TCODE_RHINOIO_OBJECT_NURBS_SURFACE);
  CASEtcode2string(TCODE_RHINOIO_OBJECT_BREP);
  CASEtcode2string(TCODE_RHINOIO_OBJECT_DATA);
  CASEtcode2string(TCODE_RHINOIO_OBJECT_END);
  CASEtcode2string(TCODE_LEGACY_ASM);
  CASEtcode2string(TCODE_LEGACY_PRT);
  CASEtcode2string(TCODE_LEGACY_SHL);
  CASEtcode2string(TCODE_LEGACY_FAC);
  CASEtcode2string(TCODE_LEGACY_BND);
  CASEtcode2string(TCODE_LEGACY_TRM);
  CASEtcode2string(TCODE_LEGACY_SRF);
  CASEtcode2string(TCODE_LEGACY_CRV);
  CASEtcode2string(TCODE_LEGACY_SPL);
  CASEtcode2string(TCODE_LEGACY_PNT);
  CASEtcode2string(TCODE_STUFF);
  CASEtcode2string(TCODE_LEGACY_ASMSTUFF);
  CASEtcode2string(TCODE_LEGACY_PRTSTUFF);
  CASEtcode2string(TCODE_LEGACY_SHLSTUFF);
  CASEtcode2string(TCODE_LEGACY_FACSTUFF);
  CASEtcode2string(TCODE_LEGACY_BNDSTUFF);
  CASEtcode2string(TCODE_LEGACY_TRMSTUFF);
  CASEtcode2string(TCODE_LEGACY_SRFSTUFF);
  CASEtcode2string(TCODE_LEGACY_CRVSTUFF);
  CASEtcode2string(TCODE_LEGACY_SPLSTUFF);
  CASEtcode2string(TCODE_LEGACY_PNTSTUFF);
  CASEtcode2string(TCODE_RH_POINT);
  CASEtcode2string(TCODE_RH_SPOTLIGHT);
  CASEtcode2string(TCODE_OLD_RH_TRIMESH);
  CASEtcode2string(TCODE_OLD_MESH_VERTEX_NORMALS);
  CASEtcode2string(TCODE_OLD_MESH_UV);
  CASEtcode2string(TCODE_OLD_FULLMESH);
  CASEtcode2string(TCODE_MESH_OBJECT);
  CASEtcode2string(TCODE_COMPRESSED_MESH_GEOMETRY);
  CASEtcode2string(TCODE_ANALYSIS_MESH);
  CASEtcode2string(TCODE_NAME);
  CASEtcode2string(TCODE_VIEW);
  CASEtcode2string(TCODE_CPLANE);
  CASEtcode2string(TCODE_NAMED_CPLANE);
  CASEtcode2string(TCODE_NAMED_VIEW);
  CASEtcode2string(TCODE_VIEWPORT);
  CASEtcode2string(TCODE_SHOWGRID);
  CASEtcode2string(TCODE_SHOWGRIDAXES);
  CASEtcode2string(TCODE_SHOWWORLDAXES);
  CASEtcode2string(TCODE_VIEWPORT_POSITION);
  CASEtcode2string(TCODE_VIEWPORT_TRACEINFO);
  CASEtcode2string(TCODE_SNAPSIZE);
  CASEtcode2string(TCODE_NEAR_CLIP_PLANE);
  CASEtcode2string(TCODE_HIDE_TRACE);
  CASEtcode2string(TCODE_NOTES);
  CASEtcode2string(TCODE_UNIT_AND_TOLERANCES);
  CASEtcode2string(TCODE_MAXIMIZED_VIEWPORT);
  CASEtcode2string(TCODE_VIEWPORT_WALLPAPER);
  CASEtcode2string(TCODE_SUMMARY);
  CASEtcode2string(TCODE_BITMAPPREVIEW);
  CASEtcode2string(TCODE_VIEWPORT_DISPLAY_MODE);
  CASEtcode2string(TCODE_LAYERTABLE);
  CASEtcode2string(TCODE_LAYERREF);
  CASEtcode2string(TCODE_XDATA);
  CASEtcode2string(TCODE_RGB);
  CASEtcode2string(TCODE_TEXTUREMAP);
  CASEtcode2string(TCODE_BUMPMAP);
  CASEtcode2string(TCODE_TRANSPARENCY);
  CASEtcode2string(TCODE_DISP_AM_RESOLUTION);
  CASEtcode2string(TCODE_RGBDISPLAY);
  CASEtcode2string(TCODE_RENDER_MATERIAL_ID);
  CASEtcode2string(TCODE_LAYER);
  CASEtcode2string(TCODE_LAYER_OBSELETE_1);
  CASEtcode2string(TCODE_LAYER_OBSELETE_2);
  CASEtcode2string(TCODE_LAYER_OBSELETE_3);
  CASEtcode2string(TCODE_LAYERON);
  CASEtcode2string(TCODE_LAYERTHAWED);
  CASEtcode2string(TCODE_LAYERLOCKED);
  CASEtcode2string(TCODE_LAYERVISIBLE);
  CASEtcode2string(TCODE_LAYERPICKABLE);
  CASEtcode2string(TCODE_LAYERSNAPABLE);
  CASEtcode2string(TCODE_LAYERRENDERABLE);
  CASEtcode2string(TCODE_LAYERSTATE);
  CASEtcode2string(TCODE_LAYERINDEX);
  CASEtcode2string(TCODE_LAYERMATERIALINDEX);
  CASEtcode2string(TCODE_RENDERMESHPARAMS);
  CASEtcode2string(TCODE_DISP_CPLINES);
  CASEtcode2string(TCODE_DISP_MAXLENGTH);
  CASEtcode2string(TCODE_CURRENTLAYER);
  CASEtcode2string(TCODE_LAYERNAME);
  CASEtcode2string(TCODE_LEGACY_TOL_FIT);
  CASEtcode2string(TCODE_LEGACY_TOL_ANGLE);
  CASEtcode2string(TCODE_DICTIONARY);
  CASEtcode2string(TCODE_DICTIONARY_ID);
  CASEtcode2string(TCODE_DICTIONARY_ENTRY);
  CASEtcode2string(TCODE_DICTIONARY_END);
  default:
    // unknown typecode.
    s = 0; 
    break;
  }
  return s;
}

#undef CASEtcode2string

char* ON_BinaryArchive::ON_TypecodeParse( unsigned int tcode, char* typecode_name, size_t max_length )
{
  char* s;
  const char* sub_name;
  const char* h = "0123456789ABCDEF";
  char c, c0;
  size_t slen;

  if ( !typecode_name || max_length <= 0 )
    return 0;
  memset(typecode_name,0,max_length*sizeof(typecode_name[0]));
  slen = max_length-1; // the -1 insures the there is a null terminator
  if ( slen <= 0 )
    return 0;

  sub_name = ON_BinaryArchive::TypecodeName(tcode);
  if ( 0 != sub_name && 0 != sub_name[0] )
  {
    c0 = *sub_name++;
    s = typecode_name+1;
    slen--;
    while ( *sub_name )
    {
      if ( slen <= 0 )
        return 0;
      *s++ = *sub_name++;
      slen--;
    }
    typecode_name[0] = c0;
    return typecode_name;    
  }

  sub_name = ON_BinaryArchive::TypecodeName( tcode & 0x7FFF0000 );
  if ( !sub_name || 0 == sub_name[0] )
    return 0;

  c0 = *sub_name++;
  s = typecode_name+1;
  slen--;

  while ( *sub_name )
  {
    if ( slen <= 0 )
      return 0;
    *s++ = *sub_name++;
    slen--;
  }

  sub_name = ON_BinaryArchive::TypecodeName( tcode & TCODE_SHORT );
  if ( sub_name )
  {
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    if ( slen <= 0 ) return 0; *s++ = '|'; slen--;
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    while ( *sub_name )
    {
      if ( slen <= 0 )
        return 0;
      *s++ = *sub_name++;
      slen--;
    }
  }

  sub_name = ON_BinaryArchive::TypecodeName( tcode & TCODE_CRC );
  if ( sub_name )
  {
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    if ( slen <= 0 ) return 0; *s++ = '|'; slen--;
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    while ( *sub_name )
    {
      if ( slen <= 0 )
        return 0;
      *s++ = *sub_name++;
      slen--;
    }
  }

  sub_name = ON_BinaryArchive::TypecodeName( tcode & 0x7FFF );
  if ( sub_name )
  {
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    if ( slen <= 0 ) return 0; *s++ = '|'; slen--;
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    while ( *sub_name )
    {
      if ( slen <= 0 )
        return 0;
      *s++ = *sub_name++;
      slen--;
    }
  }
  else 
  {
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    if ( slen <= 0 ) return 0; *s++ = '|'; slen--;
    if ( slen <= 0 ) return 0; *s++ = ' '; slen--;
    if ( slen <= 0 ) return 0; *s++ = '0'; slen--;
    if ( slen <= 0 ) return 0; *s++ = 'x'; slen--;
    c = h[((tcode & 0x7000) / 0x1000) & 0xF];
    if ( slen > 0 ) {*s++ = c; slen--;}
    c = h[((tcode & 0xF00) / 0x100) & 0xF];
    if ( slen > 0 ) {*s++ = c; slen--;}
    c = h[((tcode & 0xF0) / 0x10) & 0xF];
    if ( slen > 0 ) {*s++ = c; slen--;}
    c = h[tcode & 0xF];
    if ( slen > 0 ) {*s++ = c; slen--;}
  }

  *typecode_name = c0;

  return typecode_name;
}

static
bool Dump3dmChunk_EndReadChunkHelper( ON_BinaryArchive& file, size_t offset0, ON__UINT32 tcode, ON__INT64 big_value, ON_TextLog& dump )
{
  const bool bShortChunk = (0 != (tcode & TCODE_SHORT));
  const size_t offset1 = file.CurrentPosition();
  bool rc = file.EndRead3dmChunk();
  if ( !rc ) 
  {
    Dump3dmChunk_ErrorReportHelper(offset1,"EndRead3dmChunk() failed.",dump);
  }
  else if (!bShortChunk) 
  {
    // The crc is read or skipped by the EndRead3dmChunk() call.
    // "extra" is the number of bytes we did not parse in the dump.
    ON__INT64 sizeof_crc = (0 != (TCODE_CRC & tcode)) ? 4 : 0;
    ON__INT64 sizeof_chunk_header = 4+file.SizeofChunkLength();
    ON__INT64 delta =  (offset1 > offset0)
              ?  ((ON__INT64)(offset1 - offset0))
              : -((ON__INT64)(offset0 - offset1));
    const ON__INT64 extra = big_value - (delta+sizeof_crc-sizeof_chunk_header);
    if ( extra < 0 ) 
    {
      Dump3dmChunk_ErrorReportHelper(offset0,"Read beyond end of chunk.",dump);
      rc = false;
    }
  }
  return rc;
}

static 
bool Dump3dmChunk_UserDataHeaderHelper( size_t offset, ON_BinaryArchive& file, 
                                 int major_userdata_version, int minor_userdata_version, 
                                 ON_TextLog& dump )
{
  // TCODE_OPENNURBS_CLASS_USERDATA chunks have 2 uuids
  // the first identifies the type of ON_Object class
  // the second identifies that kind of user data
  ON_UUID userdata_classid = ON_nil_uuid;
  ON_UUID userdata_itemid = ON_nil_uuid;
  ON_UUID userdata_appid = ON_nil_uuid;  // id of plug-in that owns the user data
  int userdata_copycount = -1;
  ON_Xform userdata_xform;
  bool bLastSavedAsGoo = false;
  int ud_archive_3dm_version = 0;
  int ud_archive_opennurbs_version = 0;

  bool rc = false;
  bool bCallEndRead3dmChunk = false;

  ON__UINT32 tcode = 0;
  ON__INT64 big_value = 0;
  const size_t offset0 = file.CurrentPosition();

  for(;;)
  {
    if ( 2 == major_userdata_version )
    {
      rc = file.PeekAt3dmBigChunkType(&tcode,&big_value);
      if ( !rc )
      {
        Dump3dmChunk_ErrorReportHelper(offset,"Unable to find the TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk header in a TCODE_OPENNURBS_CLASS_USERDATA chunk.",dump);
        break;
      }
      if ( TCODE_OPENNURBS_CLASS_USERDATA_HEADER != tcode )
      {
        Dump3dmChunk_ErrorReportHelper(offset,"Unable to find the TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk header in a TCODE_OPENNURBS_CLASS_USERDATA chunk.",dump);
        rc = false;
        break;
      }
      rc = file.BeginRead3dmBigChunk(&tcode,&big_value); 
      if ( !rc )
      {
        Dump3dmChunk_ErrorReportHelper(offset,"Unable to read the TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk header in a TCODE_OPENNURBS_CLASS_USERDATA chunk.",dump);
        break;
      }
      if ( TCODE_OPENNURBS_CLASS_USERDATA_HEADER != tcode )
      {
        Dump3dmChunk_ErrorReportHelper(offset,"Missing TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk header in a TCODE_OPENNURBS_CLASS_USERDATA chunk.",dump);
        Dump3dmChunk_EndReadChunkHelper(file,offset0,tcode,big_value,dump);
        rc = false;
        break;
      }
      bCallEndRead3dmChunk = true;
    }

    rc = file.ReadUuid( userdata_classid );
    if ( !rc ) 
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadUuid() failed to read the user data class id.",dump);
      break;
    }

    dump.Print("UserData class id = ");
    dump.Print( userdata_classid );
    const ON_ClassId* pUserDataClassId = ON_ClassId::ClassId(userdata_classid);
    if ( pUserDataClassId )
    {
      const char* sClassName = pUserDataClassId->ClassName();
      if ( sClassName )
      {
        dump.Print(" (%s)",sClassName);
      }
    }
    dump.Print("\n");

    rc = file.ReadUuid( userdata_itemid );
    if ( !rc ) 
    {
       Dump3dmChunk_ErrorReportHelper(offset,"ReadUuid() failed to read the user data item id.",dump);
       break;
    }
    dump.Print("UserData item id = ");
    dump.Print( userdata_itemid );
    dump.Print("\n");

    rc = file.ReadInt( &userdata_copycount );
    if ( !rc )
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadInt() failed to read the user data copy count.",dump);
      break;
    }
    dump.Print("UserData copy count = %d\n",userdata_copycount);

    rc = file.ReadXform( userdata_xform );
    if ( !rc )
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadXform() failed to read the user data xform.",dump);
      break;
    }

    if ( 2 != major_userdata_version )
      break;
    if ( minor_userdata_version < 1 )
      break;
    rc = file.ReadUuid( userdata_appid );
    if ( !rc)
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadUuid() failed to read the user data app plug-in id.",dump);
      break;
    }
    dump.Print("UserData app plug-in id = ");
    dump.Print( userdata_appid );
    dump.Print("\n");
    if ( minor_userdata_version < 2 )
      break;
    rc = file.ReadBool(&bLastSavedAsGoo);
    if (!rc)
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadBool() failed to read the user data header bSavedAsGoo value.",dump);
      break;
    }
    rc = file.ReadInt( &ud_archive_3dm_version );
    if (!rc)
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadBool() failed to read the user data header bSavedAsGoo value.",dump);
      break;
    }
    rc = file.ReadInt( &ud_archive_opennurbs_version );
    if (!rc)
    {
      Dump3dmChunk_ErrorReportHelper(offset,"ReadBool() failed to read the user data header bSavedAsGoo value.",dump);
      break;
    }
    if ( bLastSavedAsGoo )
      dump.Print("Userdata originally written by opennurbs %d in 3dm version %d and saved as goo in this file.\n",ud_archive_opennurbs_version,ud_archive_3dm_version);
    else
      dump.Print("Userdata written by opennurbs %d in 3dm version %d.\n",ud_archive_opennurbs_version,ud_archive_3dm_version);

    break;
  }

  if ( bCallEndRead3dmChunk )
  {
    if (!Dump3dmChunk_EndReadChunkHelper(file,offset0,tcode,big_value,dump))
    {
      if (rc)
      {
        Dump3dmChunk_ErrorReportHelper(offset,"EndRead3dmChunk() failed to close the TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk.",dump);
      }
      rc = false;
    }
  }

  return rc;
}


unsigned int 
ON_BinaryArchive::Dump3dmChunk( ON_TextLog& dump, int recursion_depth )
{
  //ON_BinaryArchive& file = *this;
  const char* typecode_name = 0;
  bool bShortChunk = false;
  const size_t offset0 = CurrentPosition();
  unsigned int typecode = 0;
  ON__INT64 big_value;
  bool rc = BeginRead3dmBigChunk( &typecode, &big_value );
  if (!rc) 
  {
    Dump3dmChunk_ErrorReportHelper(offset0,"BeginRead3dmChunk() failed.",dump);
  }
  else 
  {
    if ( 0 == typecode ) 
    {
      Dump3dmChunk_ErrorReportHelper(offset0,"BeginRead3dmChunk() returned typecode = 0.",dump);
      EndRead3dmChunk();
      return 0;
    }
    else {
      if ( 0 == recursion_depth )
      {
        dump.Print("\n");
      }

      ////bShortChunk = (0 != (typecode & TCODE_SHORT));
      typecode_name = ON_BinaryArchive::TypecodeName(typecode);
      bShortChunk = (0 != (typecode & TCODE_SHORT));
      if ( !DumpChunk_PrintHeaderInfo(offset0,typecode,big_value,typecode_name,dump) )
      {
        EndRead3dmChunk();
        return 0;
      }

      int major_userdata_version = -1;
      int minor_userdata_version = -1;

      switch( typecode ) 
      {
      case TCODE_PROPERTIES_TABLE:
      case TCODE_SETTINGS_TABLE:
      case TCODE_BITMAP_TABLE:
      case TCODE_MATERIAL_TABLE:
      case TCODE_LAYER_TABLE:
      case TCODE_GROUP_TABLE:
      case TCODE_LIGHT_TABLE:
      case TCODE_FONT_TABLE:
      case TCODE_DIMSTYLE_TABLE:
      case TCODE_HATCHPATTERN_TABLE:
      case TCODE_LINETYPE_TABLE:
      case TCODE_TEXTURE_MAPPING_TABLE:
      case TCODE_HISTORYRECORD_TABLE:
      case TCODE_USER_TABLE:
      case TCODE_INSTANCE_DEFINITION_TABLE:
      case TCODE_OBJECT_TABLE:
        // start of a table
        {
          dump.PushIndent();
          unsigned int record_typecode = 0;
          for (;;) {
            record_typecode = Dump3dmChunk( dump, recursion_depth+1 );
            if ( !record_typecode ) {
              break;
            }
            if ( TCODE_ENDOFTABLE == record_typecode ) {
              break;
            }
          }
          dump.PopIndent();
        }
        break;

      case TCODE_PROPERTIES_OPENNURBS_VERSION:
        {
          dump.PushIndent();
          dump.Print("Version of opennurbs that wrote this file: %lld\n",big_value);
          dump.PopIndent();           
          if ( 0 == m_3dm_opennurbs_version && big_value > 0 && big_value <= 299999999 )
            ON_SetBinaryArchiveOpenNURBSVersion(*this,(ON__INT32)big_value);
        }
        break;

      case TCODE_BITMAP_RECORD:
        {
          dump.PushIndent();
          unsigned int bitmap_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = bitmap_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_MATERIAL_RECORD:
        {
          dump.PushIndent();
          unsigned int material_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = material_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_LAYER_RECORD:
        {
          dump.PushIndent();
          unsigned int material_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = material_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_GROUP_RECORD:
        {
          dump.PushIndent();
          unsigned int group_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = group_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_FONT_RECORD:
        {
          dump.PushIndent();
          unsigned int font_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = font_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_DIMSTYLE_RECORD:
        {
          dump.PushIndent();
          unsigned int dimstyle_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = dimstyle_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_LIGHT_RECORD:
        {
          dump.PushIndent();
          unsigned int light_chunk_typecode = 0;
          for (;;) {
            light_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
            if ( !light_chunk_typecode ) {
              break;
            }
            if ( TCODE_LIGHT_RECORD_END == light_chunk_typecode ) {
              break;
            }
            switch( light_chunk_typecode ) {
            //case TCODE_OBJECT_RECORD_TYPE:
            case TCODE_LIGHT_RECORD_ATTRIBUTES:
            case TCODE_LIGHT_RECORD_ATTRIBUTES_USERDATA:
            case TCODE_OPENNURBS_CLASS:
              break;
            default:
              {
                Dump3dmChunk_ErrorReportHelper(offset0,"Rogue chunk in light record.",dump);
              }
            }
          }
          dump.PopIndent();
        }
        break;

      case TCODE_TEXTURE_MAPPING_RECORD:
        {
          dump.PushIndent();
          unsigned int mapping_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( !typecode )
            typecode = mapping_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_HISTORYRECORD_RECORD:
        {
          dump.PushIndent();
          unsigned int history_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( !typecode )
            typecode = history_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_HATCHPATTERN_RECORD:
        {
          dump.PushIndent();
          unsigned int hatch_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( !typecode )
            typecode = hatch_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_INSTANCE_DEFINITION_RECORD:
        {
          dump.PushIndent();
          unsigned int idef_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
          if ( 0 == typecode )
            typecode = idef_chunk_typecode;
          dump.PopIndent();
        }
        break;

      case TCODE_OBJECT_RECORD:
        {
          dump.PushIndent();
          unsigned int object_chunk_typecode = 0;
          for (;;) {
            object_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1 );
            if ( !object_chunk_typecode ) {
              break;
            }
            if ( TCODE_OBJECT_RECORD_END == object_chunk_typecode ) {
              break;
            }
            switch( object_chunk_typecode ) {
            case TCODE_OBJECT_RECORD_TYPE:
            case TCODE_OBJECT_RECORD_ATTRIBUTES:
            case TCODE_OBJECT_RECORD_ATTRIBUTES_USERDATA:
            case TCODE_OPENNURBS_CLASS:
              break;
            default:
              {
                Dump3dmChunk_ErrorReportHelper(offset0,"Rogue chunk in object record.",dump);
              }
            }
          }
          dump.PopIndent();
        }
        break;

      case TCODE_OBJECT_RECORD_ATTRIBUTES:
        {
          dump.PushIndent();
          if ( big_value < 14 )
          {
            Dump3dmChunk_ErrorReportHelper(offset0,"Length of chunk is too small.  Should be >= 14.",dump);
          }
          else
          {
            ON_UUID uuid = ON_nil_uuid;
            int layer_index = -99;
            int mj = -1;
            int mn = -1;
            if ( !Read3dmChunkVersion(&mj,&mn))
            {
              Dump3dmChunk_ErrorReportHelper(offset0,"Read3dmChunkVersion() failed.",dump);
            }
            else if (!ReadUuid(uuid))
            {
              Dump3dmChunk_ErrorReportHelper(offset0,"ReadUuid() failed.",dump);
            }
            else if ( !ReadInt(&layer_index) )
            {
              Dump3dmChunk_ErrorReportHelper(offset0,"ReadInt() failed to read layer index.",dump);
            }
            else
            {
              dump.Print("Rhino object uuid: ");
              dump.Print(uuid);
              dump.Print("\n");
              dump.Print("layer index: %d\n",layer_index);
            }
          }
          dump.PopIndent();
        }
        break;

      case TCODE_OPENNURBS_CLASS:
        {
          dump.PushIndent();
          unsigned int opennurbs_object_chunk_typecode = 0;
          for (;;) {
            opennurbs_object_chunk_typecode = Dump3dmChunk( dump, recursion_depth+1  );
            if ( !opennurbs_object_chunk_typecode ) {
              break;
            }
            if ( TCODE_OPENNURBS_CLASS_END == opennurbs_object_chunk_typecode ) {
              break;
            }
            switch( opennurbs_object_chunk_typecode ) 
            {
            case TCODE_OPENNURBS_CLASS_UUID:
              break;
            case TCODE_OPENNURBS_CLASS_DATA:
              break;
            case TCODE_OPENNURBS_CLASS_USERDATA:
              break;
            default:
              {
                Dump3dmChunk_ErrorReportHelper(offset0,"Rogue chunk in OpenNURBS class record.",dump);
              }
            }
          }
          dump.PopIndent();
        }
        break;

      case TCODE_OPENNURBS_CLASS_USERDATA:
        {
          if ( !Read3dmChunkVersion(&major_userdata_version, &minor_userdata_version ) )
          {
            Dump3dmChunk_ErrorReportHelper(offset0,"Read3dmChunkVersion() failed to read TCODE_OPENNURBS_CLASS_USERDATA chunk version.",dump);
          }
          else
          {
            dump.PushIndent();
            dump.Print("UserData chunk version: %d.%d\n",
                       major_userdata_version,
                       minor_userdata_version
                       );
            if ( 1 == major_userdata_version || 2 == major_userdata_version )
            {
              const size_t userdata_header_offset = CurrentPosition();
              switch ( major_userdata_version )
              {
              case 1:
              case 2:
                {
                  // version 1 user data header information was not wrapped
                  // in a chunk.
                  //
                  // version 2 user data header information is wrapped
                  // in a TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk.
                  if ( Dump3dmChunk_UserDataHeaderHelper( 
                                  userdata_header_offset, *this, 
                                  major_userdata_version, minor_userdata_version,
                                  dump ) 
                     )
                  {
                    // a TCODE_ANONYMOUS_CHUNK contains user data goo
                    int anon_typecode =  Dump3dmChunk( dump, recursion_depth+1 );
                    if ( TCODE_ANONYMOUS_CHUNK != anon_typecode )
                    {
                      Dump3dmChunk_ErrorReportHelper( offset0,"Userdata Expected a TCODE_ANONYMOUS_CHUNK chunk.",dump);
                    }
                  }
                }
                break;
              default:
                if ( major_userdata_version < 3 )
                {
                }
                else
                {
                  dump.Print("New user data format created after this diagnostic tool was written.\n");
                }
                break;
              }
            }

            dump.PopIndent();
          }
        }
        break;

      case TCODE_OPENNURBS_CLASS_UUID:
      case TCODE_USER_TABLE_UUID:
        {
          dump.PushIndent();
          ON_UUID uuid = ON_nil_uuid;
          const ON_ClassId* pClassId = 0;
          if ( !ReadUuid( uuid ) ) {
             Dump3dmChunk_ErrorReportHelper(offset0,"ReadUuid() failed.",dump);
          }
          else 
          {
            if ( typecode == TCODE_OPENNURBS_CLASS_UUID ) 
            {
              dump.Print("OpenNURBS class id = ");
              pClassId = ON_ClassId::ClassId(uuid);
            }
            else if ( typecode == TCODE_USER_TABLE_UUID ) 
            {
              dump.Print("User table id = ");
            }
            else {
              dump.Print("UUID = ");
            }
            dump.Print( uuid );
            if ( pClassId )
            {
              const char* sClassName = pClassId->ClassName();
              if ( sClassName )
              {
                dump.Print(" (%s)",sClassName);
              }
            }
            dump.Print("\n");
          }

          dump.PopIndent();
        }
        break;

      case TCODE_OPENNURBS_CLASS_USERDATA_HEADER:
        {
          // we should never get here because this chunk is parsed in the TCODE_OPENNURBS_CLASS_USERDATA case above.
          Dump3dmChunk_ErrorReportHelper(offset0,"Encountered TCODE_OPENNURBS_CLASS_USERDATA_HEADER chunk outside a TCODE_OPENNURBS_CLASS_USERDATA chunk.",dump);
        }
        break;

      case TCODE_ENDOFFILE:
      case TCODE_ENDOFFILE_GOO:
        {
          dump.PushIndent();
          if ( big_value < 4 ) {
            Dump3dmChunk_ErrorReportHelper(offset0,"TCODE_ENDOFFILE chunk withlength < 4.",dump);
          }
          else {
            ON__UINT64 sizeof_file = 0;
            ReadEOFSizeOfFile(&sizeof_file);
            dump.Print("current position = %d  stored size = %llu\n",
                       CurrentPosition(), 
                       sizeof_file
                       );
          }
          dump.PopIndent();
        }
        break;

      }
    }

    const size_t offset1 = CurrentPosition();
    if ( !EndRead3dmChunk(true) ) 
    {
      Dump3dmChunk_ErrorReportHelper(offset1,"EndRead3dmChunk() failed.",dump);
      rc = false;
    }
    else if (!bShortChunk) 
    {
      ON__INT64 delta =  (offset1 > offset0)
                ?  ((ON__INT64)(offset1 - offset0))
                : -((ON__INT64)(offset0 - offset1));
      const ON__INT64 extra = big_value - (delta-4-SizeofChunkLength());
      if ( extra < 0 ) 
      {
        Dump3dmChunk_ErrorReportHelper(offset0,"Read beyond end of chunk.",dump);
      }
    }
  }
  return typecode;
}



ON_Read3dmBufferArchive::ON_Read3dmBufferArchive( 
          size_t sizeof_buffer, 
          const void* buffer,
          bool bCopyBuffer,
          int archive_3dm_version,
          int archive_opennurbs_version
          )
: ON_BinaryArchive(ON::read3dm)
, m_p(0)
, m_buffer(0)
, m_sizeof_buffer(0)
, m_buffer_position(0)
, m_reserved1(0)
, m_reserved2(0)
, m_reserved3(0)
, m_reserved4(0)
{
  if ( sizeof_buffer > 0 && 0 != buffer )
  {
    if ( bCopyBuffer )
    {
      m_p = onmalloc(sizeof_buffer);
      if ( 0 != m_p )
        memcpy(m_p,buffer,sizeof_buffer);
      m_buffer = (const unsigned char*)m_p;
    }
    else
    {
      m_buffer = (const unsigned char*)buffer;
    }
    if ( m_buffer )
    {
      m_sizeof_buffer = sizeof_buffer;
      SetArchive3dmVersion(archive_3dm_version);
      ON_SetBinaryArchiveOpenNURBSVersion(*this,archive_opennurbs_version);
    }
  }
}

ON_Read3dmBufferArchive::~ON_Read3dmBufferArchive()
{
  if ( m_p )
    onfree(m_p);
}

// ON_BinaryArchive overrides
size_t ON_Read3dmBufferArchive::CurrentPosition() const
{
  return m_buffer_position;
}

bool ON_Read3dmBufferArchive::SeekFromCurrentPosition( int offset )
{
  bool rc = false;
  if ( m_buffer )
  {
    if (offset >= 0 )
    {
      m_buffer_position += offset;
      rc = true;
    }
    else if ( size_t(-offset) <= m_buffer_position )
    {
      m_buffer_position -= (size_t(-offset));
      rc = true;
    }
  }
  return rc;
}

bool ON_Read3dmBufferArchive::SeekFromStart( size_t offset )
{
  bool rc = false;
  if ( m_buffer ) 
  {
    if ( offset > 0 )
      m_buffer_position = offset;
    else
      m_buffer_position = 0;
    rc = true;
  }
  return rc;
}

bool ON_Read3dmBufferArchive::AtEnd() const
{
  return (m_buffer_position >= m_sizeof_buffer) ? true : false;
}

size_t ON_Read3dmBufferArchive::Read( size_t count, void* buffer )
{
  if ( count <= 0 || 0 == buffer )
    return 0;

  size_t maxcount = ( m_sizeof_buffer > m_buffer_position ) 
                  ? (m_sizeof_buffer - m_buffer_position)
                  : 0;
  if ( count > maxcount )
    count = maxcount;

  if ( count > 0 ) 
  {
    memcpy( buffer, m_buffer+m_buffer_position, count );
    m_buffer_position += count;
  }

  return count;
}

size_t ON_Read3dmBufferArchive::Write( size_t, const void* )
{
  // ON_Read3dmBufferArchive does not support Write() and Flush()
  return 0;
}

bool ON_Read3dmBufferArchive::Flush()
{
  // ON_Read3dmBufferArchive does not support Write() and Flush()
  return false;
}


size_t ON_Read3dmBufferArchive::SizeOfBuffer() const
{
  return m_sizeof_buffer;
}

const void* ON_Read3dmBufferArchive::Buffer() const
{
  return (const void*)m_buffer;
}





ON_Write3dmBufferArchive::ON_Write3dmBufferArchive( 
          size_t initial_sizeof_buffer, 
          size_t max_sizeof_buffer, 
          int archive_3dm_version,
          int archive_opennurbs_version
          )
: ON_BinaryArchive(ON::write3dm)
, m_p(0)
, m_buffer(0)
, m_sizeof_buffer(0)
, m_max_sizeof_buffer(max_sizeof_buffer)
, m_sizeof_archive(0)
, m_buffer_position(0)
, m_reserved1(0)
, m_reserved2(0)
, m_reserved3(0)
, m_reserved4(0)
{
  if ( initial_sizeof_buffer > 0 )
    AllocBuffer(initial_sizeof_buffer);
  if ( archive_3dm_version < 2 )
    archive_3dm_version = ON_BinaryArchive::CurrentArchiveVersion();
  SetArchive3dmVersion(archive_3dm_version);
  ON_SetBinaryArchiveOpenNURBSVersion(*this,archive_opennurbs_version);
}

ON_Write3dmBufferArchive::~ON_Write3dmBufferArchive()
{
  if ( m_p )
    onfree(m_p);
}

void ON_Write3dmBufferArchive::AllocBuffer( size_t sz )
{
  if ( sz > m_sizeof_buffer 
       && (m_max_sizeof_buffer <= 0 || sz <= m_max_sizeof_buffer) 
     )
  {
    if ( sz < 2*m_sizeof_buffer )
    {
      sz = 2*m_sizeof_buffer;
      if ( sz > m_max_sizeof_buffer )
        sz = m_max_sizeof_buffer;
    }

    m_p = onrealloc(m_p,sz);
    m_buffer = (unsigned char*)m_p;

    if ( 0 != m_buffer )
    {
      memset( m_buffer + m_sizeof_buffer, 0, sz - m_sizeof_buffer );
      m_sizeof_buffer = sz;
    }
    else
    {
      m_sizeof_buffer = 0;
    }

  }
}

// ON_BinaryArchive overrides
size_t ON_Write3dmBufferArchive::CurrentPosition() const
{
  return m_buffer_position;
}

bool ON_Write3dmBufferArchive::SeekFromCurrentPosition( int offset )
{
  bool rc = false;
  if ( m_buffer )
  {
    if (offset >= 0 )
    {
      m_buffer_position += offset;
      rc = true;
    }
    else if ( size_t(-offset) <= m_buffer_position )
    {
      m_buffer_position -= (size_t(-offset));
      rc = true;
    }
  }
  return rc;
}

bool ON_Write3dmBufferArchive::SeekFromStart( size_t offset )
{
  bool rc = false;
  if ( m_buffer ) 
  {
    if ( offset > 0 )
      m_buffer_position = offset;
    else
      m_buffer_position = 0;
    rc = true;
  }
  return rc;
}

bool ON_Write3dmBufferArchive::AtEnd() const
{
  return (m_buffer_position >= m_sizeof_buffer) ? true : false;
}

size_t ON_Write3dmBufferArchive::Read( size_t count, void* buffer )
{
  if ( count <= 0 || 0 == buffer )
    return 0;

  size_t maxcount = ( m_sizeof_buffer > m_buffer_position ) 
                  ? (m_sizeof_buffer - m_buffer_position)
                  : 0;
  if ( count > maxcount )
    count = maxcount;

  if ( count > 0 ) 
  {
    memcpy( buffer, m_buffer+m_buffer_position, count );
    m_buffer_position += count;
  }

  return count;
}

size_t ON_Write3dmBufferArchive::Write( size_t sz, const void* buffer )
{
  if ( sz <= 0 || 0 == buffer )
    return 0;

  if ( m_buffer_position + sz > m_sizeof_buffer )
  {
    AllocBuffer(m_buffer_position + sz);
  }

  if ( m_buffer_position + sz > m_sizeof_buffer )
    return 0;

  memcpy( m_buffer + m_buffer_position, buffer, sz );
  m_buffer_position += sz;
  if ( m_buffer_position > m_sizeof_archive )
    m_sizeof_archive = m_buffer_position;

  return sz;
}

bool ON_Write3dmBufferArchive::Flush()
{
  // Nothing to flush
  return true;
}


size_t ON_Write3dmBufferArchive::SizeOfBuffer() const
{
  return m_sizeof_buffer;
}

const void* ON_Write3dmBufferArchive::Buffer() const
{
  return (const void*)m_buffer;
}

void* ON_Write3dmBufferArchive::HarvestBuffer()
{
  void* buffer = m_buffer;

  m_p = 0;
  m_buffer = 0;
  m_sizeof_buffer = 0;
  m_sizeof_archive = 0;
  m_buffer_position = 0;

  return buffer;
}

size_t ON_Write3dmBufferArchive::SizeOfArchive() const
{
  return m_sizeof_archive;
}




ON_BinaryArchiveBuffer::ON_BinaryArchiveBuffer( ON::archive_mode mode, ON_Buffer* buffer )
: ON_BinaryArchive(mode)
, m_buffer(buffer)
{
}

ON_BinaryArchiveBuffer::~ON_BinaryArchiveBuffer()
{
  m_buffer = 0;
}

bool ON_BinaryArchiveBuffer::SetBuffer( ON_Buffer* buffer )
{
  if ( 0 == m_buffer )
  {
    m_buffer = buffer;
    return true;
  }

  return false;
}

ON_Buffer* ON_BinaryArchiveBuffer::Buffer() const
{
  return m_buffer;
}

size_t ON_BinaryArchiveBuffer::CurrentPosition() const
{
  if ( 0 != m_buffer )
    return (size_t)m_buffer->CurrentPosition();

  return 0;
}

bool ON_BinaryArchiveBuffer::SeekFromCurrentPosition(int offset)
{
  if ( 0 != m_buffer )
    return m_buffer->SeekFromCurrentPosition(offset);

  return false;
}

bool ON_BinaryArchiveBuffer::SeekFromStart(size_t offset)
{
  if ( 0 != m_buffer )
    return m_buffer->SeekFromStart((ON__INT64)offset);

  return false;
}

bool ON_BinaryArchiveBuffer::AtEnd() const
{
  if ( 0 != m_buffer )
    return m_buffer->AtEnd();

  return false;
}

bool ON_BinaryArchiveBuffer::SeekFromEnd( ON__INT64 offset )
{
  if ( 0 != m_buffer )
    return m_buffer->SeekFromEnd(offset);

  return false;
}

size_t ON_BinaryArchiveBuffer::Read( size_t count, void* a )
{
  if ( 0 != m_buffer )
    return (size_t)m_buffer->Read(count,a);

  return 0;
}

size_t ON_BinaryArchiveBuffer::Write( size_t count, const void* a )
{
  if ( 0 != m_buffer )
    return (size_t)m_buffer->Write(count,a);

  return 0;
}

bool ON_BinaryArchiveBuffer::Flush()
{
  if ( 0 != m_buffer )
    return true;

  return false;
}

ON_FileIterator::ON_FileIterator()
: m_count(0)
#if defined(ON_COMPILER_MSC)
, m_file_attributes_mask(0)
, m_h(0)
#else
, m_dir(0)
, m_current_file_attributes(0)
, m_current_file_size(0)
, m_current_file_create_time(0)
, m_current_last_modified_time(0)
, m_current_last_access_time(0)
#endif
{
  Destroy();
#if defined(ON_COMPILER_MSC)
  memset(&m_fd,0,sizeof(m_fd));
#else
  memset(&m_dirent,0,sizeof(m_dirent));
  m_dirent.d_name[0] = 0;
  memset(&m_dirent_name_buffer[0],0,sizeof(m_dirent_name_buffer));
  memset(&m_current_name[0],0,sizeof(m_current_name));
#endif
}

ON_FileIterator::~ON_FileIterator()
{
  Destroy();
}

void ON_FileIterator::Destroy()
{
#if defined(ON_COMPILER_MSC)
  if ( 0 != m_h )
  {
    ::FindClose(m_h);
    m_h = 0;
  }
  m_file_attributes_mask = 0;
  memset(&m_fd,0,sizeof(m_fd));
#else
  if ( 0 != m_dir )
  {
    closedir(m_dir);
    m_dir = 0;
  }
  memset(&m_dirent,0,sizeof(m_dirent));
  m_dirent.d_name[0] = 0;
  memset(&m_dirent_name_buffer[0],0,sizeof(m_dirent_name_buffer));
  m_ws_file_name_filter.Destroy();
  m_utf8_file_name_filter.Destroy();
  memset(&m_current_name[0],0,sizeof(m_current_name));
  m_current_file_attributes = 0;
  m_current_file_size = 0;
  m_current_file_create_time = 0;
  m_current_last_modified_time = 0;
  m_current_last_access_time = 0;
#endif
  m_count = 0;
  m_directory.Empty();
}

ON__UINT64 ON_FileIterator::Count() const
{
  return m_count;
}

#if defined(ON_COMPILER_MSC)
static bool IsDotOrDotDotDir( const wchar_t* s )
#else
static bool IsDotOrDotDotDir( const char* s )
#endif
{
  bool rc = false;
  for (;;)
  {
    if ( 0 == s )
      break;
    if ( '.' != s[0] )
      break;
    if ( 0 != s[1] )
    {
      if ( '.' != s[1] )
        break;
      if ( 0 != s[2] )
        break;
    }
    rc = true; // s = "." or s = ".."
    break;
  }
  return rc;
}

const wchar_t* ON_FileIterator::FirstFile(
    const char* directory_name, 
    const char* file_name_filter 
    )
{
  // assume directory_name and file_name_filter are UTF-8 encoded
  // strings, convert them to wchar_t strings and call the
  // wchar_t version of this function.
  ON_wString ws_directory_name = directory_name;
  ON_wString ws_file_name_filter = file_name_filter;
  const wchar_t* wchar_directory_name = ws_directory_name;
  const wchar_t* wchar_file_name_filter = ws_file_name_filter;
  return FirstFile(wchar_directory_name,wchar_file_name_filter);
}

const wchar_t* ON_FileIterator::FirstFile(
    const wchar_t* directory_name, 
    const wchar_t* file_name_filter 
    )
{
  ON_wString buffer(directory_name);
  {
    const wchar_t* dir_seps = L"/\\";
    buffer.TrimRight(dir_seps);
    if ( buffer.Length() <= 0 || buffer.IsEmpty() )
      buffer = directory_name;
    else
      directory_name = buffer;
  }

#if defined(ON_COMPILER_MSC)
  const ON__UINT32 saved_mask = m_file_attributes_mask;
  Destroy();
  m_file_attributes_mask = saved_mask;

  ON_wString s(directory_name);

  
  if ( 0 == file_name_filter )
  {
    // A null file file_name_filter means iterate 
    // through all items in the directory.  To do
    // this using Windows' ::FindFirstFile, set the
    // filter to "*.*", even though some items will
    // not contain a "dot".
    file_name_filter = L"*.*";  
  }
  
  if ( 0 != file_name_filter[0] )
  {
    s += L"\\";
    s += file_name_filter;
  }

  m_h = ::FindFirstFile(s, &m_fd);
  if ( 0 == m_h || INVALID_HANDLE_VALUE == m_h || 0 == m_fd.cFileName[0] )
  {
    // Happens on "fake" directories like "My Music" and "My Videos"
    m_h = 0;
    Destroy();
    m_file_attributes_mask = saved_mask;
    return 0;
  }

  m_directory = directory_name;

  if ( IsDotOrDotDotDir(m_fd.cFileName) || 0 != (m_file_attributes_mask & m_fd.dwFileAttributes) )
  {
    return NextFile();
  }
  
  m_count++;
  m_fd.cFileName[(sizeof(m_fd.cFileName)/sizeof(m_fd.cFileName[0]))-1] = 0;
  return m_fd.cFileName;

#else

  // gcc code
  Destroy();
  m_directory = directory_name;
  m_ws_file_name_filter = file_name_filter;
  m_utf8_file_name_filter = file_name_filter;
  const ON_String utf8_str(m_directory); // convert wchar_t to utf8 string
  const char* s = utf8_str;
  m_dir = (0 != s && 0 != s[0]) ? opendir(s) : 0;
  if ( 0 != m_dir )
    return NextFile();
  Destroy();
  return 0;

#endif
}

const wchar_t* ON_FileIterator::NextFile()
{
#if defined(ON_COMPILER_MSC)
  const ON__UINT32 saved_mask = m_file_attributes_mask;
  if ( 0 == m_h || INVALID_HANDLE_VALUE == m_h || 0 == m_fd.cFileName[0] )
  {
    Destroy();
    m_file_attributes_mask = saved_mask;
    return 0;
  }

  for (;;)
  {
    if ( !::FindNextFile( m_h, &m_fd) || 0 == m_fd.cFileName[0] )
    {
      Destroy();
      m_file_attributes_mask = saved_mask;
      return 0;
    }

    if ( IsDotOrDotDotDir(m_fd.cFileName) || 0 != (m_file_attributes_mask & m_fd.dwFileAttributes) )
    {
      continue;
    }

    break;
  }

  m_count++;
  m_fd.cFileName[(sizeof(m_fd.cFileName)/sizeof(m_fd.cFileName[0]))-1] = 0;
  return m_fd.cFileName;
#else

  // gcc code
  ON__UINT64 current_file_attributes = 0;
  wchar_t current_name[ sizeof(m_current_name)/sizeof(m_current_name[0]) ];
  for(;;)
  {
    current_file_attributes = 0;
    struct dirent* dp = 0;
    int readdir_errno = readdir_r(m_dir, &m_dirent, &dp);
    if ( 0 !=  readdir_errno )
      break;
    if ( 0 == dp )
      break;
    if ( 0 == m_dirent.d_name[0] )
      break;

    if ( IsDotOrDotDotDir(m_dirent.d_name) )
      continue;

    memset( current_name, 0, sizeof(current_name) );
    ON_ConvertUTF8ToWideChar(
      &m_dirent.d_name[0],-1, // null terminated utf8 string
      &current_name[0], ((int)(sizeof(current_name)/sizeof(current_name[0]))) - 1, // output wchar_t string
      0, // null output error status
      (4|8|16), // mask common conversion errors
      0, // error_code_point = null terminator inserted at point of conversion error
      0  // null output end-of-string pointer
      );
    // TODO
    //   Test m_dirent.d_name to make sure it passes m_ws/utf8_file_name_filter

    ON_wString wpath = m_directory;
    wpath += '/';
    wpath += current_name;

    // get a utf8 version of the full path to pass to stat
    const ON_String utf8buffer(wpath);
    const char* utf8path = utf8buffer;
    if ( 0 == utf8path )
      continue;

    struct stat buf;
    memset(&buf,0,sizeof(buf));
    int stat_errno = stat( utf8path, &buf);
    if ( 0 != stat_errno )
      continue;

    if ( S_ISDIR(buf.st_mode) )
    {
      current_file_attributes = 2;
    }
    else if ( S_ISREG(buf.st_mode) )
    {
      // Only *.ext filters work at this time for non-windows
      const wchar_t* file_name_filter = m_ws_file_name_filter;
      if (   0 != file_name_filter
            && '*' == file_name_filter[0] 
            && '.' == file_name_filter[1]
            && 0 != file_name_filter[2]
            && '*' != file_name_filter[2] )
      {
        // assume this is a *.extension filter
        const wchar_t* current_name_ext = 0;
        on_wsplitpath(current_name,0,0,0,&current_name_ext);
        if (   0 == current_name_ext 
            || 0 != wcscmp(file_name_filter+1,current_name_ext) 
           )
        {
          // current_name does pass match file_name_filter
          continue;
        }
      }
      current_file_attributes = 1;
    }
    else
      continue;

    // save current item information
    memcpy( m_current_name, current_name, sizeof(m_current_name) );
    m_current_file_attributes = current_file_attributes;
    m_current_file_size = buf.st_size;
    m_current_file_create_time = buf.st_mtime; // create time is not available on struct stat
    m_current_last_modified_time = buf.st_mtime;
    m_current_last_access_time = buf.st_atime;

    return m_current_name;
  }

  Destroy();
  return 0;
#endif
}


const wchar_t* ON_FileIterator::CurrentFileName() const
{
#if defined(ON_COMPILER_MSC)
  return ( 0 != m_h && 0 != m_fd.cFileName[0] ) ? m_fd.cFileName : 0;
#else
  return ( 0 != m_current_name[0] ) ? m_current_name : 0;
#endif
}

ON__UINT64 ON_FileIterator::CurrentFileSize() const
{
  ON__UINT64 file_size = 0;

#if defined(ON_COMPILER_MSC)
  if ( 0 != CurrentFileName() ) 
  {
    file_size  = m_fd.nFileSizeHigh;
    file_size *= ((ON__UINT64)0xFFFFFFFF);
    file_size += m_fd.nFileSizeLow;
  }
#else
  file_size = m_current_file_size;
#endif

  return file_size;
}

bool ON_FileIterator::CurrentFileIsDirectory() const
{
  bool rc = false;
  const wchar_t* current_file_name = CurrentFileName();
  if ( 0 != current_file_name && 0 != current_file_name[0] )
  {
#if defined(ON_COMPILER_MSC)
    if ( 0 != (FILE_ATTRIBUTE_DIRECTORY & m_fd.dwFileAttributes) )
    {
      rc = true;
    }
#else
    if ( 2 == m_current_file_attributes)
    {
      rc = true;
    }
#endif
  }
  return rc;
}

bool ON_FileIterator::CurrentFileIsHidden() const
{
  bool rc = false;
  const wchar_t* current_file_name = CurrentFileName();
  if ( 0 != current_file_name && 0 != current_file_name[0] )
  {
    if ( '.' == current_file_name[0] )
    {
      rc = true;
    }
#if defined(ON_COMPILER_MSC)
    else if ( 0 != (FILE_ATTRIBUTE_HIDDEN & m_fd.dwFileAttributes) )
    {
      rc = true;
    }
#endif
  }
  return rc;
}


bool ON_FileIterator::GetCurrentFullPathFileName( ON_wString& filename ) const
{
  bool rc = false;

#if defined(ON_COMPILER_MSC)
  if ( 0 == m_h || 0 == m_fd.cFileName )
  {
    filename.Empty();
  }
  else
  {
    filename = m_directory;
    filename += L"\\";
    filename += m_fd.cFileName;
    rc = true;
  }
#else

  // gcc implementation
  if ( 0 == m_current_name[0] )
  {
    filename.Empty();
  }
  else
  {
    filename = m_directory;
    filename += L"/";
    filename += m_current_name;
    rc = true;
  }

#endif

  return rc;
}


#if defined(ON_COMPILER_MSC)
static ON__UINT64 SecondsSinceJanOne1970( FILETIME ft )
{
  // The FILETIME is in 100-nanosecond intervals since January 1, 1601 UCT.
  //
  // Between midnight January 1, 1601 and midnight January 1, 1970 there 
  // were 134774 days = 11644473600 seconds. Each second has 10^7 intervals
  // that are one hundred nanoseconds long.  So, if N = number of one hundred
  // nanosecond intervals since midnight January 1, 1601, then
  // (N / 10000000) - 11644473600 = number of seconds since midnight
  // January 1, 1970.
  //
  // January 1, 1601 was the start of a Gregorian calendary 400 year cycle
  // and "the internet" sometimes cites that as the reason that date is 
  // the "beginning of time" for Windows' FILETIME values.  This convention
  // would slightly simplify the formulae used to account for leap years, 
  // so it is plausable this might might even be true.

  ON__UINT64 ft_since_jan_1_1601 = ft.dwHighDateTime;
  ft_since_jan_1_1601 *= 0xFFFFFFFF;
  ft_since_jan_1_1601 += ft.dwLowDateTime;

  ON__UINT64 hundrednanoseconds_per_second = 10000000;

  ON__UINT64 seconds_since_jan_1_1601 = ft_since_jan_1_1601 / hundrednanoseconds_per_second;
                                                   
  ON__UINT64 seconds_since_jan_1_1970 = seconds_since_jan_1_1601 - 11644473600;

  return seconds_since_jan_1_1970;
}
#endif

ON__UINT64 ON_FileIterator::CurrentFileCreateTime() const
{
#if defined(ON_COMPILER_MSC)
  return SecondsSinceJanOne1970(m_fd.ftCreationTime);
#else
  return m_current_file_create_time;
#endif
}

ON__UINT64 ON_FileIterator::CurrentFileLastModifiedTime() const
{
#if defined(ON_COMPILER_MSC)
  return SecondsSinceJanOne1970(m_fd.ftLastWriteTime);
#else
  return m_current_last_modified_time;
#endif
}

ON__UINT64 ON_FileIterator::CurrentFileLastAccessTime() const
{
#if defined(ON_COMPILER_MSC)
  return SecondsSinceJanOne1970(m_fd.ftLastAccessTime);
#else
  return m_current_last_access_time;
#endif
}


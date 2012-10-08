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

ON_VIRTUAL_OBJECT_IMPLEMENT(ON_UserData,ON_Object,"850324A7-050E-11d4-BFFA-0010830122F0");

ON_UserData::ON_UserData()
            : m_userdata_uuid(ON_nil_uuid), 
              m_application_uuid(ON_nil_uuid),
              m_userdata_copycount(0),
              m_userdata_xform(1),
              m_userdata_owner(0),
              m_userdata_next(0)
{}

ON_UserData::ON_UserData(const ON_UserData& src) 
            : ON_Object(src),
              m_userdata_uuid(src.m_userdata_uuid), 
              m_application_uuid(src.m_application_uuid),
              m_userdata_copycount(src.m_userdata_copycount),
              m_userdata_xform(src.m_userdata_xform),
              m_userdata_owner(0), // do not copy owner
              m_userdata_next(0)   // do not copy next
{
  if ( m_userdata_copycount) 
  {
    m_userdata_copycount++;
    if ( !m_userdata_copycount )
      m_userdata_copycount = 1;
  }
}

//virtual
ON_BOOL32 ON_UserData::Archive() const
{
  // If you want your userdata to be saved, you must override 
  // ON_UserData::Archive() and have it return true.
  return false;
}

//virtual
ON_BOOL32 ON_UserData::Transform(const ON_Xform& x )
{
  m_userdata_xform = x*m_userdata_xform;
  return true;
}

ON_UserData& ON_UserData::operator=(const ON_UserData& src)
{
  // 16 January 2004 Dale Lear
  //    Do not copy the m_userdata_uuid, m_application_uuid,
  //    m_userdata_owner, or m_userdata_next values.
  //    The m_userdata_uuid and m_application_uuid are
  //    set when the class is constructed and should not be 
  //    changed.  The m_userdata_owner and m_userdata_next
  //    values are set when the user data is attached
  //    to a parent object.
  if ( this != &src ) 
  {
    ON_Object::operator=(src);
    m_userdata_copycount = src.m_userdata_copycount;
    m_userdata_xform = src.m_userdata_xform;
    if ( 0 != m_userdata_copycount )
    {
      m_userdata_copycount++;
      if ( !m_userdata_copycount )
        m_userdata_copycount = 1;
    }
  }

  return *this;
}

ON_UserData::~ON_UserData()
{
  ON_Object* owner = m_userdata_owner;
  if ( owner ) {
    // remove this piece of user data from owner->m_userdata_list
    ON_UserData* prev = 0;
    ON_UserData* p;
    for ( p = owner->m_userdata_list; p; prev = p, p = p->m_userdata_next ) {
      if ( p == this ) {
        if ( prev ) {
          prev->m_userdata_next = p->m_userdata_next;
        }
        else {
          owner->m_userdata_list = p->m_userdata_next;
        }
        p->m_userdata_next = 0;
        p->m_userdata_owner = 0;
        break;
      }
    }
  }
}

void ON_UserData::Dump( ON_TextLog& text_log ) const
{
  text_log.Print("User Data:\n");
  text_log.PushIndent();

  // print class name and class uuid
  ON_Object::Dump(text_log);

  // developer's user data description
  ON_wString description;
  const_cast<ON_UserData*>(this)->GetDescription(description);
  if ( description.IsEmpty() )
    description = L"none";
  const wchar_t* ws = description;
  text_log.Print("user data description: %ls\n",ws);
  text_log.Print("user data uuid: ");
  text_log.Print(m_userdata_uuid);
  text_log.Print("\n");
  text_log.Print("user data copy count: %d\n",this->m_userdata_copycount);

  // archive setting
  text_log.Print("user data saved in 3dm archive: %s\n",Archive() ? "yes" : "no");

  text_log.PopIndent();
}

unsigned int ON_UserData::SizeOf() const
{
  unsigned int sz = ON_Object::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Object));
  return sz;
}

ON_BOOL32 ON_UserData::IsValid( ON_TextLog* text_log ) const
{
  if ( 0 == ON_UuidCompare( &m_userdata_uuid, &ON_nil_uuid ) )
  {
    if ( 0 != text_log )
    {
      text_log->Print("invalid userdata - m_userdata_uuid = nil\n");
    }
    return false;
  }

  if ( 0 == ON_UuidCompare( m_userdata_uuid, ON_UserData::ClassId()->Uuid() ) )
  {
    if ( 0 != text_log )
    {
      text_log->Print("invalid userdata - m_userdata_uuid in use. Use guidgen to get a unique id.\n");
    }
    return false;
  }

  if ( Archive() && 0 == ON_UuidCompare( ClassId()->Uuid(), ON_UserData::ClassId()->Uuid() ) )
  {
    // 8 January 2004 Dale Lear:
    //  I added this test to help developers remember to use
    //  the ON_DECLARE_OBJECT/ON_IMPLEMENT_OBJECT macros when
    //  they create user data that gets archived.
    if ( 0 != text_log )
    {
      text_log->Print("invalid userdata - classes derived from ON_UserData that get saved in 3dm archives must have a class id and name defined by ON_OBJECT_DECLARE/ON_OBJECT_IMPLEMENT.\n");
    }
    return false;
  }

  return true;
}

ON_Object* ON_UserData::Owner() const
{
  return m_userdata_owner;
}

ON_UserData* ON_UserData::Next() const
{
  return m_userdata_next;
}

ON_UUID ON_UserData::UserDataClassUuid() const
{
  const ON_ClassId* cid = ClassId();
  return ( cid == &ON_UnknownUserData::m_ON_UnknownUserData_class_id )
          ? ((ON_UnknownUserData*)this)->m_unknownclass_uuid
          : cid->Uuid();
}

ON_BOOL32 ON_UserData::IsUnknownUserData() const
{
  return (ClassId() == &ON_UnknownUserData::m_ON_UnknownUserData_class_id)?true:false;
}

ON_BOOL32 ON_UserData::GetDescription( ON_wString& description )
{
  return true;
}


ON_OBJECT_IMPLEMENT(ON_UnknownUserData,ON_UserData,"850324A8-050E-11d4-BFFA-0010830122F0");

ON_UnknownUserData::ON_UnknownUserData() 
: m_unknownclass_uuid(ON_nil_uuid)
, m_sizeof_buffer(0)
, m_buffer(0)
, m_3dm_version(0)
, m_3dm_opennurbs_version(0)
{}

ON_UnknownUserData::ON_UnknownUserData(const ON_UnknownUserData& src) 
: ON_UserData(src)
, m_unknownclass_uuid(ON_nil_uuid)
, m_sizeof_buffer(0)
, m_buffer(0)
, m_3dm_version(0)
, m_3dm_opennurbs_version(0)
{
  if ( m_userdata_copycount > 0 && src.m_sizeof_buffer > 0 && src.m_buffer ) 
  {
    // For most kinds of user data except ON_UnknownUserData,
    // m_userdata_uuid is set by the constructor and should not
    // be copied from src (which may be a derived class).  However,
    // for ON_UnknownUserData, the value of m_userdata_uuid is 
    // varies because it is set by the missing userdata class. 
    // So it has to be copied here.
    m_userdata_uuid = src.m_userdata_uuid;

    m_unknownclass_uuid = src.m_unknownclass_uuid;
    m_sizeof_buffer = src.m_sizeof_buffer;
    m_buffer = onmemdup( src.m_buffer, src.m_sizeof_buffer);
    m_3dm_version = src.m_3dm_version;
    m_3dm_opennurbs_version = src.m_3dm_opennurbs_version;
  }
}

ON_UnknownUserData& ON_UnknownUserData::operator=(const ON_UnknownUserData& src)
{
  if ( this != &src ) 
  {
    m_sizeof_buffer = 0;
    if ( 0 != m_buffer )
    {
      onfree(m_buffer);
      m_buffer = 0;
    }

    // ON_UserData::operator= handles setting m_userdata_copycount and
    // m_userdata_xform.
    ON_UserData::operator=(src);

    // For most kinds of user data except ON_UnknownUserData,
    // m_userdata_uuid and m_application_uuid are set by the
    // constructor and should not be altered by an operator=.  
    // However, for ON_UnknownUserData, the value of m_userdata_uuid 
    // and m_application_uuid vary because they are set by the
    // missing userdata class.  So they have to be copied here.
    m_userdata_uuid = src.m_userdata_uuid;
    m_application_uuid = src.m_application_uuid; // fix added 26 January 2010

    if ( m_userdata_copycount > 0 && src.m_sizeof_buffer > 0 && src.m_buffer ) 
    {
      m_unknownclass_uuid = src.m_unknownclass_uuid;
      m_sizeof_buffer = src.m_sizeof_buffer;
      m_buffer = onmemdup( src.m_buffer, src.m_sizeof_buffer);
      m_3dm_version = src.m_3dm_version;
      m_3dm_opennurbs_version = src.m_3dm_opennurbs_version;
    }
    else 
    {
      // The unknown user data is not supposed to copy
      m_userdata_uuid = ON_nil_uuid;
      m_unknownclass_uuid = ON_nil_uuid;
      m_sizeof_buffer = 0;
      m_buffer = 0;
      m_3dm_version = 0;
      m_3dm_opennurbs_version = 0;
    }
  }
  return *this;
}

ON_UnknownUserData::~ON_UnknownUserData()
{
  if ( m_buffer )
    onfree(m_buffer);
}

unsigned int ON_UnknownUserData::SizeOf() const
{
  return ON_UserData::SizeOf() 
    + (sizeof(ON_UnknownUserData)-sizeof(ON_UserData))
    + m_sizeof_buffer;
}

ON_BOOL32 ON_UnknownUserData::GetDescription( ON_wString& s )
{
  s = "Unknown user data. (Definition of class was not available when object was read.)";
  return true;
}

ON_BOOL32 ON_UnknownUserData::IsValid( ON_TextLog* text_log ) const
{
  ON_BOOL32 rc = ON_UserData::IsValid(text_log);
  
  // valid unknown user data must have something in it
  if (rc) 
    rc = (m_sizeof_buffer>0);
  if (rc) 
    rc = (m_buffer != NULL);
  
  // the unknown class uuid cannot be nil
  if (rc) 
    rc = ON_UuidCompare( &m_unknownclass_uuid, &ON_nil_uuid );
  
  // the unknown class uuid cannot be the ON_UnknownUserData class uuid
  if (rc) {
    ON_UUID ON_UnknownUserData_classuuid = ON_UnknownUserData::m_ON_UnknownUserData_class_id.Uuid();
    rc = ON_UuidCompare( &m_unknownclass_uuid, &ON_UnknownUserData_classuuid );
  }
  return rc?true:false;
}

void ON_UnknownUserData::Dump( ON_TextLog& dump ) const
{
  ON_UserData::Dump(dump);
  dump.PushIndent();
  dump.Print( "unknown class uuid: ");
  dump.Print( m_unknownclass_uuid );
  dump.Print( "\n");
  dump.Print( "Data size in 3dm archive: %d bytes\n",m_sizeof_buffer);
  dump.PopIndent();
}

ON_BOOL32 ON_UnknownUserData::Archive() const
{
  // 22 January 2004 Dale Lear
  //   Since unknown userdata is only created when
  //   an archive is read, it has to be saved.
  return true;
}

ON_BOOL32 ON_UnknownUserData::Write( ON_BinaryArchive& file ) const
{
  return file.WriteByte(m_sizeof_buffer,m_buffer);
}

ON_BOOL32 ON_UnknownUserData::Read( ON_BinaryArchive& file )
{
  m_buffer = onrealloc( m_buffer, m_sizeof_buffer );
  m_3dm_version = file.Archive3dmVersion();
  return file.ReadByte(m_sizeof_buffer,m_buffer);
}


class ON_UnknownUserDataArchive : public ON_BinaryArchive
{
  // This class is used to define an ON_BinaryArchive that can be used
  // in ON_UnknownUserData::Convert() to initialize a ON_UserData class
  // from a memory buffer.
public:
  ON_UnknownUserDataArchive( const ON_UnknownUserData& );
  ~ON_UnknownUserDataArchive();

  // ON_BinaryArchive overrides
  size_t CurrentPosition( // current offset (in bytes) into archive ( like ftell() )
                ) const; 
  bool SeekFromCurrentPosition( // seek from current position ( like fseek( ,SEEK_CUR) )
                int // byte offset ( >= -CurrentPostion() )
                ); 
  bool SeekFromStart(  // seek from current position ( like fseek( ,SEEK_SET) )
                size_t // byte offset ( >= 0 )
                );
  bool AtEnd() const; // true if at end of file

protected:
  size_t Read( size_t, void* ); // return actual number of bytes read (like fread())
  size_t Write( size_t, const void* );
  bool Flush();

private:
  ON_UnknownUserDataArchive();

  size_t m_sizeof_buffer;
  const unsigned char* m_buffer;
  size_t m_buffer_position;
};

ON_UnknownUserDataArchive::ON_UnknownUserDataArchive( const ON_UnknownUserData& ud ) : ON_BinaryArchive( ON::read3dm )
{
  SetArchive3dmVersion(ud.m_3dm_version);
  m_sizeof_buffer = ud.m_sizeof_buffer;
  m_buffer = (const unsigned char*)ud.m_buffer;
  m_buffer_position = 0;
}

ON_UnknownUserDataArchive::~ON_UnknownUserDataArchive()
{
}

size_t ON_UnknownUserDataArchive::CurrentPosition() const
{
  return m_buffer_position;
}

bool ON_UnknownUserDataArchive::SeekFromCurrentPosition( int offset )
{
  bool rc = false;
  if ( offset >= 0 || m_buffer_position >= ((size_t)(-offset)) )
  {
    size_t newpos = m_buffer_position + offset;
    if ( newpos < m_sizeof_buffer ) 
    {
      m_buffer_position = newpos;
      rc = true;
    }
  }
  return rc;
}

bool ON_UnknownUserDataArchive::SeekFromStart( size_t offset )
{
  bool rc = false;
  if ( offset < m_sizeof_buffer ) 
  {
    if ( offset > 0 )
      m_buffer_position = offset;
    else
      m_buffer_position = 0;
    rc = true;
  }
  return rc;
}

bool ON_UnknownUserDataArchive::AtEnd() const
{
  return (m_buffer_position >= m_sizeof_buffer) ? true : false;
}

size_t ON_UnknownUserDataArchive::Read( size_t count, void* buffer )
{
  size_t maxcount = 0;

  if ( m_sizeof_buffer > m_buffer_position ) 
  {
    maxcount = m_sizeof_buffer - m_buffer_position;
  }

  if ( count > maxcount )
  {
    count = maxcount;
  }

  if ( count > 0 ) 
  {
    memcpy( buffer, m_buffer+m_buffer_position, count );
    m_buffer_position += count;
  }

  return count;
}

size_t ON_UnknownUserDataArchive::Write( size_t, const void* )
{
  // ON_UnknownUserDataArchive does not support Write() and Flush()
  return 0;
}

bool ON_UnknownUserDataArchive::Flush()
{
  // ON_UnknownUserDataArchive does not support Write() and Flush()
  return false;
}

ON_UserData* ON_UnknownUserData::Convert() const
{
  ON_UserData* ud = NULL;
  if ( IsValid() ) {
    const ON_ClassId* pID = ON_ClassId::ClassId( m_unknownclass_uuid );
    // if pID is NULL, it means the definiton of the unknown user data
    // is still not available
    if ( pID ) {
      // The class definition has been dynamically loaded since the
      // user data was read from an archive.  Use the class's Read()
      // to convert the buffer into a valid class
      ON_Object* pObject = pID->Create();
      if ( pObject ) {
        ud = ON_UserData::Cast(pObject);
        if ( !ud )
          delete pObject;
        else 
        {
          // use class's Read() function to initialize class members from buffer
          ON_UnknownUserDataArchive file(*this);
          // copy values that would be set by reading the base class
          ud->m_userdata_copycount = m_userdata_copycount;
          ud->m_userdata_xform = m_userdata_xform;
          ud->Read(file);
        }
      }
    }
  }
  return ud;
}

bool ON_UserDataHolder::MoveUserDataFrom( const ON_Object& source_object )
{
  PurgeUserData();
  MoveUserData(*const_cast<ON_Object*>(&source_object));
  return (0 != FirstUserData());
}

bool ON_UserDataHolder::MoveUserDataTo(  const ON_Object& source_object, bool bAppend )
{
  if ( !bAppend )
  {
    const_cast<ON_Object*>(&source_object)->PurgeUserData();
  }
  const_cast<ON_Object*>(&source_object)->MoveUserData(*this);
  PurgeUserData();
  return (0 != source_object.FirstUserData());
}

ON_BOOL32 ON_UserDataHolder::IsValid( ON_TextLog* text_log ) const
{
  return true;
}


/////////////////////////////////////////////////////////////////
//
// Built in tool for attaching arbitrary string userdata
// to any opennurbs object.  The savvy user will probably
// use XML in the string.


ON_UserString::ON_UserString()
{
}

ON_UserString::~ON_UserString()
{
}


bool ON_UserString::Write(ON_BinaryArchive& archive) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;

  for(;;)
  {
    rc = archive.WriteString(m_key);
    if (!rc) break;
    rc = archive.WriteString(m_string_value);
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

bool ON_UserString::Read(ON_BinaryArchive& archive)
{
  m_key.Empty();
  m_string_value.Empty();

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    rc = ( 1 == major_version );
    if (!rc) break;
    rc = archive.ReadString(m_key);
    if (!rc) break;
    rc = archive.ReadString(m_string_value);
    if (!rc) break;

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

void ON_UserString::Dump(ON_TextLog& text_log) const
{
  const wchar_t* ws = m_key;
  if ( !ws )
    ws = L"";
  text_log.Print("Key: %ls\n", ws);
  
  ws = m_string_value;
  if ( !ws )
    ws = L"";
  text_log.Print("Value: %ls\n",ws);
}

ON_OBJECT_IMPLEMENT(ON_UserStringList,ON_UserData,"CE28DE29-F4C5-4faa-A50A-C3A6849B6329");

ON_UserStringList::ON_UserStringList()
{
  m_userdata_uuid = ON_UserStringList::m_ON_UserStringList_class_id.Uuid();
  m_application_uuid = ON_opennurbs4_id; // opennurbs.dll reads/writes this userdata
                                         // The id must be the version 4 id because
                                         // V5 SaveAs V4 needs to work.
  m_userdata_copycount = 1;
}

ON_UserStringList::~ON_UserStringList()
{
}

ON_BOOL32 ON_UserStringList::GetDescription( ON_wString& description )
{
  description.Format("User text (%d entries)",m_e.Count());
  return true;
}

ON_BOOL32 ON_UserStringList::Archive() const
{
  return true;
}

unsigned int ON_UserStringList::SizeOf() const
{
  unsigned int sz = ON_UserData::SizeOf();
  sz += sizeof(*this) - sizeof(ON_UserData);
  sz += m_e.SizeOfArray();
  int i = m_e.Count();
  while (i--)
    sz += m_e[i].m_string_value.Length()*sizeof(wchar_t);
  return sz;
}


ON__UINT32 ON_UserStringList::DataCRC(ON__UINT32 current_remainder) const
{
  int count = m_e.Count();
  for ( int i = 0; i < count; i++ )
  {
    current_remainder = m_e[i].m_key.DataCRC(current_remainder);
    current_remainder = m_e[i].m_string_value.DataCRC(current_remainder);
  }
  return current_remainder;
}


void ON_UserStringList::Dump( ON_TextLog& text_log ) const
{
  int i, count = m_e.Count();
  text_log.Print("%d entries\n",count);
  text_log.PushIndent();
  for ( i = 0; i < count; i++ )
  {
    m_e[i].Dump(text_log);
  }
  text_log.PopIndent();
}

ON_BOOL32 ON_UserStringList::Write(ON_BinaryArchive& archive) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;

  for(;;)
  {
    int count = m_e.Count();
    rc = archive.WriteInt(count);
    if(!rc) break;

    for ( int i = 0; i < count && rc; i++ )
    {
      rc = m_e[i].Write(archive);
    }
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

ON_BOOL32 ON_UserStringList::Read(ON_BinaryArchive& archive)
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    int count = 0;
    rc = archive.ReadInt(&count);
    if(!rc) break;

    for ( int i = 0; i < count; i++ )
    {
      rc = m_e.AppendNew().Read(archive);
      if ( !rc) 
      {
        m_e.Remove();
        break;
      }
    }
    if (!rc) break;

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}


bool ON_UserStringList::SetUserString( const wchar_t* key, const wchar_t* string_value )
{
  if ( !key || !key[0] )
    return false;

  int i, count = m_e.Count();
  for (i = 0; i < count; i++ )
  {
    if ( !m_e[i].m_key.CompareNoCase(key) )
    {
      if ( string_value && string_value[0] )
      {
        m_e[i].m_string_value = string_value;
      }
      else
      {
        m_e.Remove(i);
      }
      m_userdata_copycount++;
      return true;
    }
  }

  if ( string_value && string_value[0] )
  {
    ON_UserString& e = m_e.AppendNew();
    e.m_key = key;
    e.m_string_value = string_value;
    m_userdata_copycount++;
    return true;
  }

  return false;
}

bool ON_UserStringList::GetUserString( const wchar_t* key, ON_wString& string_value ) const
{
  if ( key && key[0] )
  {
    int i, count = m_e.Count();
    for (i = 0; i < count; i++ )
    {
      if ( !m_e[i].m_key.CompareNoCase(key) )
      {
        string_value = m_e[i].m_string_value;
        return true;
      }
    }
  }

  string_value.Empty();
  return false;
}

static int cmp_hash_2dex_ij(const void* a, const void* b)
{
  const int* ai = (const int*)a;
  const int* bi = (const int*)b;
  // 26 January 2012 Dale Lear
  //    Part of the fix for http://dev.mcneel.com/bugtrack/?q=97693
  //
  //    The "i" values are actually 32 bit hashes of a string
  //    and are often large.  The sign of (ai[0] - bi[0]) cannot
  //    be used to compare ai[0] and bi[0] because integer
  //    overflow occures with values that are large.
  //
  ////// NO!
  //////if ( 0 == (rc = ai[0] - bi[0]) )
  //////  rc = ai[1] - bi[1];
  //////return rc;

  if ( ai[0] < bi[0] )
    return -1;
  if ( ai[0] > bi[0] )
    return 1;
  if ( ai[1] < bi[1] )
    return -1;
  if ( ai[1] > bi[1] )
    return 1;
  return 0;
}

int ON_UserStringList::SetUserStrings( int count, const ON_UserString* us, bool bReplace )
{
  int added_count = 0;
  int i;

  if ( count <= 0 || 0 == us )
    return 0;

  if ( 1 == count )
  {
    // skip the hash table hoo haa
    if (  us[0].m_key.IsEmpty() )
      return 0;
    for ( i = 0; i < m_e.Count(); i++ )
    {
      if ( m_e[i].m_key.CompareNoCase(us[0].m_key ) )
        continue;
      if ( bReplace )
      {
        if ( us[0].m_string_value.IsEmpty() )
          m_e.Remove(i);
        else
          m_e[i] = us[0];
        added_count++;
      }
      break;
    }
    return added_count;
  }

  size_t k0, k1;
  int count0 = m_e.Count();
  size_t count0_plus_count = (size_t)(count0 + count);
  ON_2dex* hash = (ON_2dex*)onmalloc( (count0_plus_count + count)*sizeof(hash[0]) );
  ON_2dex* hash1 =  hash + (count0_plus_count);
  const ON_2dex* h;
  int deleted_count = 0;

  for ( i = 0; i < count0; i++ )
  {
    hash[i].i = (int)m_e[i].m_key.DataCRCLower(0);
    hash[i].j = i;
  }

  for ( i = 0; i < count; i++ )
  {
    hash1[i].i = (int)us[i].m_key.DataCRCLower(0);
    hash1[i].j = i;
    hash[i+count0].i = hash1[i].i;
    hash[i+count0].j = hash1[i].j+count0;
  }
  ON_qsort(hash,count0_plus_count,sizeof(hash[0]),cmp_hash_2dex_ij);

  m_e.Reserve(count0+count);
  for ( i = 0; i < count; i++)
  {
    if ( us[i].m_key.IsEmpty() )
      continue;

    // Set k0, k1 so that hash[k0]....,hash[k1-1] are 
    // the hash[] entries keys with the same hash code
    // as us[i].m_key.
    h = ON_BinarySearch2dexArray(hash1[i].i,hash,count0_plus_count);
    if ( 0 == h )
    {
      ON_ERROR("There is a bug in this function.");
      continue;
    }
    k0 = h-hash;
    while ( k0 > 0 && h[-1].i == h[0].i )
    {
      // set h = first element in hash[] with this hash code.
      k0--;
      h--;
    }
    for (k1 = k0+1; k1 < count0_plus_count; k1++ )
    {
      if ( hash[k1].i != hash[k0].i )
        break;
      if ( hash[k1].j > i+count0 )
        break;
    }

    if ( hash[k0].j >= count0 )
    {
      // There are no entries in m_e[] with key matching hash,
      // so us[i].m_key is not present in m_e.
      if ( !us[i].m_string_value.IsEmpty() )
      {
        hash[k0].j = count0++;
        m_e.Append(us[i]);
        added_count++;
      }
      continue;
    }

    for (/* empty init*/; k0 < k1; k0++ )
    {
      if ( hash[k0].j < count0 )
      {
        if ( m_e[hash[k0].j].m_key.CompareNoCase(us[i].m_key) )
          continue; // different keys with same hash
        if ( bReplace )
        {
          m_e[hash[k0].j] = us[i];
          added_count++;
          if ( us[i].m_string_value.IsEmpty() )
            deleted_count++;
        }
        break;
      }
    }

    if ( k0 >= k1 )
    {
      // hash is unique up to this point, so us[i].m_key is unique, 
      // so we add it if it is valid.
      if ( !us[i].m_string_value.IsEmpty() )
      {
        hash[k0].j = count0++;
        m_e.Append(us[i]);
        added_count++;
      }
    }
  }

  onfree(hash);

  // remove deleted items.
  i = m_e.Count();
  while ( i-- > 0 && deleted_count > 0 )
  {
    if ( m_e[i].m_string_value.IsEmpty() )
    {
      m_e.Remove(i);
      deleted_count--;
    }
  }

  return added_count;
}





bool ON_Object::SetUserString( const wchar_t* key, const wchar_t* string_value )
{
  ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  bool b = false;
  if ( !us )
  {
    us = new ON_UserStringList();
    if ( !AttachUserData(us) )
    {
      delete us;
      us = 0;
    }
    else
    {
      b = true;
    }
  }

  if ( us )
  {
    if ( us->SetUserString(key,string_value) )
    {
      if ( b && 2 == us->m_userdata_copycount )
      {
        // user data is brand new - roll back the 
        // m_userdata_copycount++ that happens in 
        // SetUserString().
        us->m_userdata_copycount = 1;
      }
      b = true;
    }
    else if ( b )
    {
      // user data was new-ed up and has nothing in it
      // because the input was bogus.
      delete us;
      us = 0;
      b = false;
    }
  }
  return b;
}


int ON_Object::SetUserStrings( int count, const ON_UserString* user_strings, bool bReplace )
{
  if ( 0 == count || 0 == user_strings )
    return 0;

  int add_count = 0;
  int del_count = 0;
  for ( int i = 0; i < count; i++ )
  {
    if ( user_strings[i].m_key.IsEmpty() )
      continue;
    if ( user_strings[i].m_string_value.IsEmpty() )
      del_count++;
    else
      add_count++;
  }
  if ( 0 == add_count && 0 == del_count )
    return 0;

  ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  if ( !us && add_count > 0)
  {
    us = new ON_UserStringList();
    if ( !AttachUserData(us) )
    {
      delete us;
      us = 0;
    }
  }

  return us ? us->SetUserStrings(count,user_strings,bReplace ) : 0;
}


bool ON_Object::GetUserString( const wchar_t* key, ON_wString& string_value ) const
{
  string_value.Empty();
  const ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  return us ? us->GetUserString(key,string_value) : false;
}

int ON_Object::UserStringCount() const
{
  const ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  return ( 0 != us )
         ? us->m_e.Count()
         : 0;
}


int ON_Object::GetUserStrings( 
  ON_ClassArray<ON_UserString>& user_strings 
  ) const
{
  const int count0 = user_strings.Count();
  const ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  if ( us )
    user_strings.Append(us->m_e.Count(),us->m_e.Array());

  return user_strings.Count() - count0;
}

int ON_Object::GetUserStringKeys( 
  ON_ClassArray<ON_wString>& user_string_keys 
  ) const
{
  const int count0 = user_string_keys.Count();
  const ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  if ( us )
  {
    user_string_keys.Reserve( count0 + us->m_e.Count() );
    for (int i = 0; i < us->m_e.Count(); i++ )
    {
      user_string_keys.Append(us->m_e[i].m_key);
    }
  }

  return user_string_keys.Count() - count0;
}

ON_OBJECT_IMPLEMENT(ON_DocumentUserStringList,ON_Object,"06F3218E-F5EC-4f6c-B74C-14583F0ED7BC");

ON_DocumentUserStringList::ON_DocumentUserStringList()
{
}

ON_DocumentUserStringList::~ON_DocumentUserStringList()
{
}

ON_BOOL32 ON_DocumentUserStringList::IsValid( ON_TextLog* text_log ) const
{
  return true;
}

void ON_DocumentUserStringList::Dump( ON_TextLog& ) const
{
}

ON__UINT32 ON_DocumentUserStringList::DataCRC(ON__UINT32 current_remainder) const
{
  const ON_UserStringList* us = ON_UserStringList::Cast(GetUserData(ON_UserStringList::m_ON_UserStringList_class_id.Uuid()));
  if ( us )
    current_remainder = us->DataCRC(current_remainder);
  return current_remainder;
}

ON_BOOL32 ON_DocumentUserStringList::Write(ON_BinaryArchive& binary_archive) const
{
  //  The key/value pairs are saved as ON_UserStringList user data.

  // A single char with value 1 is written to permit adding additional
  // information later.  In that case change the 1 to a 2, uncomment the
  // begin block code, and the IO will still work.
  //
  unsigned char c = 1;
  bool rc = binary_archive.WriteChar(c);

  ////rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  ////if (!rc)
  ////  return false;
  ////for(;;)
  ////{
  ////  rc = binary_archive.Write...();
  ////  if (!rc) break;

  ////  break;
  ////}
  ////if ( !binary_archive.EndWrite3dmChunk() )
  ////  rc = false;

  return rc;
}

ON_BOOL32 ON_DocumentUserStringList::Read(ON_BinaryArchive& binary_archive)
{
  //  The key/value pairs are saved as ON_UserStringList user data.

  unsigned char c = 0;
  bool rc =  binary_archive.ReadChar(&c);
  if ( !rc || c < 1 || c > 2 )
    return false;

  if ( 2 == c )
  {
    // The code in this if(2==c) will not be used unless
    // the 1 in Write is changed to a 2.  This code is here
    // so old versions of Rhino will be able to skip over
    // any new information that is added at a later date.
    int major_version = 0;
    int minor_version = 0;
    rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
    if (!rc)
      return false;
    for(;;)
    {
      rc = (1 == major_version);
      if (!rc) break;

      ////rc = binary_archive.Read(...);
      ////if (!rc) break;

      break;
    }
    if (!binary_archive.EndRead3dmChunk() )
      rc = false;
  }

  return rc;
}

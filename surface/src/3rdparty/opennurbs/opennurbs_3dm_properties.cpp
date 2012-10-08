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

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmRevisionHistory
//

ON_3dmRevisionHistory::ON_3dmRevisionHistory() : m_revision_count(0)
{
  Default();
}

ON_3dmRevisionHistory::~ON_3dmRevisionHistory()
{
  m_sCreatedBy.Destroy();
  m_sLastEditedBy.Destroy();
}

void ON_3dmRevisionHistory::Default()
{
  m_sCreatedBy.Destroy();
  m_sLastEditedBy.Destroy();
  memset( &m_create_time,    0, sizeof(m_create_time) );
  memset( &m_last_edit_time, 0, sizeof(m_last_edit_time) );
  m_revision_count = 0;
}

static int ON_CompareRevisionHistoryTime( const struct tm* time0, const struct tm* time1 )
{
  if ( 0 == time0 || 0 == time1 )
  {
    if ( 0 != time0 )
      return 1;
    if ( 0 != time1 )
      return -1;
    return 0;
  }

  if (time0->tm_year < time1->tm_year)
    return -1;
  if (time0->tm_year > time1->tm_year)
    return 1;

  if (time0->tm_mon < time1->tm_mon)
    return -1;
  if (time0->tm_mon > time1->tm_mon)
    return 1;

  if (time0->tm_mday < time1->tm_mday)
    return -1;
  if (time0->tm_mday > time1->tm_mday)
    return 1;

  if (time0->tm_hour < time1->tm_hour)
    return -1;
  if (time0->tm_hour > time1->tm_hour)
    return 1;

  if (time0->tm_min < time1->tm_min)
    return -1;
  if (time0->tm_min > time1->tm_min)
    return 1;

  if (time0->tm_sec < time1->tm_sec)
    return -1;
  if (time0->tm_sec > time1->tm_sec)
    return 1;

  return 0;
}

bool ON_3dmRevisionHistory::CreateTimeIsSet() const
{
  struct tm jan_1_1970;
  memset(&jan_1_1970,0,sizeof(jan_1_1970));
  jan_1_1970.tm_mday = 1;    /* day of the month - [1,31] */
  jan_1_1970.tm_year = 70;   /* years since 1900 */
  return ( ON_CompareRevisionHistoryTime(&jan_1_1970,&m_create_time) >= 0 );
}
/*
Returns:
  true 
    if m_last_edit_time is >= January 1, 1970
*/
bool ON_3dmRevisionHistory::LastEditedTimeIsSet() const
{
  struct tm jan_1_1970;
  memset(&jan_1_1970,0,sizeof(jan_1_1970));
  jan_1_1970.tm_mday = 1;    /* day of the month - [1,31] */
  jan_1_1970.tm_year = 70;   /* years since 1900 */
  return ( ON_CompareRevisionHistoryTime(&jan_1_1970,&m_last_edit_time) >= 0 );
}


ON_BOOL32 ON_3dmRevisionHistory::IsValid() const
{
  return (     LastEditedTimeIsSet() 
            && ON_CompareRevisionHistoryTime(&m_create_time, &m_last_edit_time) <= 0
         );
}

int ON_3dmRevisionHistory::NewRevision()
{
  struct tm current_time;
  memset(&current_time,0,sizeof(current_time));
  {
    time_t gmt = time(0);
    const struct tm* t = gmtime(&gmt);
    if ( t )
      current_time = *t;
  }
  m_last_edit_time = current_time;

#if defined(ON_OS_WINDOWS)  
  // use Windows ::GetUserNameW() to get current user name
  wchar_t current_user[512];
  memset( current_user, 0, sizeof(current_user) );
  ULONG len = 510;
  if( !::GetUserNameW(current_user,&len) )
    current_user[0] = 0;
  m_sLastEditedBy = current_user;
#endif

  if ( m_revision_count <= 0 ) 
  {
    m_revision_count = 0;
    m_sCreatedBy = m_sLastEditedBy;
    m_create_time = current_time;
  };

  m_revision_count++;

  return m_revision_count;
}


ON_BOOL32 ON_3dmRevisionHistory::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion( 1, 0 );
  if (rc) rc = file.WriteString( m_sCreatedBy );
  if (rc) rc = file.WriteTime( m_create_time );
  if (rc) rc = file.WriteString( m_sLastEditedBy );
  if (rc) rc = file.WriteTime(m_last_edit_time );
  if (rc) rc = file.WriteInt( m_revision_count );
  return rc;
}

ON_BOOL32 ON_3dmRevisionHistory::Read( ON_BinaryArchive& file )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion( &major_version, &minor_version );
  if ( rc && major_version == 1 ) {
    rc = file.ReadString( m_sCreatedBy );
    if (rc) rc = file.ReadTime( m_create_time );
    if (rc) rc = file.ReadString( m_sLastEditedBy );
    if (rc) rc = file.ReadTime(m_last_edit_time );
    if (rc) rc = file.ReadInt( &m_revision_count );
  }
  return rc;
}

void ON_3dmRevisionHistory::Dump( ON_TextLog& dump ) const
{
  const wchar_t* ws = m_sCreatedBy;
  if ( !ws ) ws = L"";
  dump.Print("Created by: %ls\n", ws );
  dump.Print("Created on: "); dump.PrintTime(m_create_time); dump.Print("\n");

  
  ws = m_sLastEditedBy;
  if ( !ws ) ws = L"";
  dump.Print("Last edited by: %ls\n", ws );
  dump.Print("Last edited on: "); dump.PrintTime(m_last_edit_time); dump.Print("\n");

  dump.Print("Revision count: %d\n",m_revision_count);
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmNotes
//

ON_3dmNotes::ON_3dmNotes()
            : m_bVisible(0), 
              m_bHTML(0),
              m_window_left(0),
              m_window_top(0),
              m_window_right(0),
              m_window_bottom(0)
{}

ON_3dmNotes::ON_3dmNotes( const ON_3dmNotes& src )
            : m_bVisible(0), 
              m_bHTML(0),
              m_window_left(0),
              m_window_top(0),
              m_window_right(0),
              m_window_bottom(0)
{
  *this = src;
}

ON_3dmNotes::~ON_3dmNotes()
{
  m_notes.Destroy();
}

ON_3dmNotes& ON_3dmNotes::operator=(const ON_3dmNotes& src)
{
  if ( this != &src ) {
    m_notes     = src.m_notes;
    m_bVisible  = src.m_bVisible;
    m_bHTML     = src.m_bHTML;
    m_window_left   = src.m_window_left;
    m_window_top    = src.m_window_top;
    m_window_right  = src.m_window_right;
    m_window_bottom = src.m_window_bottom;
  }
  return *this;
}

void ON_3dmNotes::Default()
{
  m_notes.Destroy();
  m_bVisible = 0;
  m_bHTML = 0;
  m_window_left = 0;
  m_window_top = 0;
  m_window_right = 0;
  m_window_bottom = 0;
}

ON_BOOL32 ON_3dmNotes::IsValid() const
{
  return m_notes.IsEmpty() ? false : true;
}

ON_BOOL32 ON_3dmNotes::Read( ON_BinaryArchive& file )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion( &major_version, &minor_version );
  if ( rc && major_version == 1 ) {
    m_notes.Destroy();
    rc = file.ReadInt( &m_bHTML );
    if ( rc ) rc = file.ReadString( m_notes );
    if ( rc ) rc = file.ReadInt( &m_bVisible );
    if ( rc ) rc = file.ReadInt( &m_window_left );
    if ( rc ) rc = file.ReadInt( &m_window_top );
    if ( rc ) rc = file.ReadInt( &m_window_right );
    if ( rc ) rc = file.ReadInt( &m_window_bottom );
  }
  return rc;
}

ON_BOOL32 ON_3dmNotes::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if ( rc ) rc = file.WriteInt( m_bHTML );
  if ( rc ) rc = file.WriteString( m_notes );
  if ( rc ) rc = file.WriteInt( m_bVisible );
  if ( rc ) rc = file.WriteInt( m_window_left );
  if ( rc ) rc = file.WriteInt( m_window_top );
  if ( rc ) rc = file.WriteInt( m_window_right );
  if ( rc ) rc = file.WriteInt( m_window_bottom );
  return rc;
}

void ON_3dmNotes::Dump(ON_TextLog& dump) const
{
  const wchar_t* s = m_notes;
  if ( s )
    dump.PrintWrappedText(s);
  dump.Print("\n");
}


//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmApplication
//
ON_3dmApplication::ON_3dmApplication()
{
}

ON_3dmApplication::ON_3dmApplication( const ON_3dmApplication& src)
{
  m_application_name = src.m_application_name;
  m_application_URL = src.m_application_URL;
  m_application_details = src.m_application_details;
}


ON_3dmApplication::~ON_3dmApplication()
{
  m_application_name.Empty();
  m_application_URL.Empty();
  m_application_details.Empty();
}

ON_3dmApplication& ON_3dmApplication::operator=(const ON_3dmApplication& src)
{
  if ( this != &src ) {
    m_application_name = src.m_application_name;
    m_application_URL = src.m_application_URL;
    m_application_details = src.m_application_details;
  }
  return *this;
}

void ON_3dmApplication::Default()
{
  m_application_name.Empty();
  m_application_URL.Empty();
  m_application_details.Empty();
}

void ON_3dmApplication::Dump( ON_TextLog& dump ) const
{
  const wchar_t* s = m_application_name;
  if ( s )
    dump.Print("Name: %ls\n",s);
  s = m_application_URL;
  if ( s )
    dump.Print("URL: %ls\n",s);
  s = m_application_details;
  if ( s )
    dump.Print("Details: %ls\n",s);
}

ON_BOOL32 ON_3dmApplication::IsValid() const
{
  return m_application_name.IsEmpty() ? false : true;
}

ON_BOOL32 ON_3dmApplication::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion( &major_version, &minor_version );
  if (rc) rc = file.ReadString( m_application_name );
  if (rc) rc = file.ReadString( m_application_URL );
  if (rc) rc = file.ReadString( m_application_details );
  return rc;
}

ON_BOOL32 ON_3dmApplication::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion( 1, 0 );
  if (rc) rc = file.WriteString( m_application_name );
  if (rc) rc = file.WriteString( m_application_URL );
  if (rc) rc = file.WriteString( m_application_details );
  return rc;
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmProperties
//

void ON_3dmProperties::Default()
{
  // default properties
  m_RevisionHistory.Default();
  m_Notes.Default();
  m_PreviewImage.Destroy();
  m_Application.Default();
}

ON_3dmProperties::ON_3dmProperties()
{
  Default();
};

ON_3dmProperties::~ON_3dmProperties()
{
  Default();
}

ON_3dmProperties::ON_3dmProperties(const ON_3dmProperties& src)
{
  Default();
  *this = src;
}

ON_3dmProperties& ON_3dmProperties::operator=(const ON_3dmProperties& src)
{
  if ( this != &src ) {
    m_RevisionHistory = src.m_RevisionHistory;
    m_Notes = src.m_Notes;
    m_PreviewImage = src.m_PreviewImage;
    m_Application = src.m_Application;
  }
  return *this;
}

void ON_SetBinaryArchiveOpenNURBSVersion(ON_BinaryArchive& file, int value)
{
  if ( value >= 200012210 )
  {
    file.m_3dm_opennurbs_version = value;
  }
  else
  {
    ON_ERROR("ON_SetBinaryArchiveOpenNURBSVersion - invalid opennurbs version");
    file.m_3dm_opennurbs_version = 0;
  }
}

ON_BOOL32 ON_3dmProperties::Read(ON_BinaryArchive& file )
{
  Default();

  ON_BOOL32 rc = true;

  unsigned int tcode;
  ON__INT64 value;

  for(;;) {

    rc = file.BeginRead3dmBigChunk( &tcode, &value );
    if ( !rc )
      break;

    switch(tcode) {

    case TCODE_PROPERTIES_OPENNURBS_VERSION:
      { 
        int on_version = 0;
        if ( value > 299912319 || (value != 0 && value < 200101010) )
        {
          ON_ERROR("ON_3dmProperties::Read - TCODE_PROPERTIES_OPENNURBS_VERSION corrupt value");
          rc = false;
        }
        else
        {
          on_version = (int)value;
        }
        ON_SetBinaryArchiveOpenNURBSVersion(file,on_version);
      }
      break;
      
    case TCODE_PROPERTIES_REVISIONHISTORY: // file creation/revision information
      m_RevisionHistory.Read(file);
      break;
      
    case TCODE_PROPERTIES_NOTES: // file notes
      m_Notes.Read(file);
      break;
      
    case TCODE_PROPERTIES_PREVIEWIMAGE: // uncompressed preview image
      m_PreviewImage.ReadUncompressed(file);
      break;
      
    case TCODE_PROPERTIES_COMPRESSED_PREVIEWIMAGE: // compressed preview image
      m_PreviewImage.ReadCompressed(file);
      break;
      
    case TCODE_PROPERTIES_APPLICATION: // application that created 3dm file
      m_Application.Read(file);
      break;
      
    default:
      // information added in future will be skipped by file.EndRead3dmChunk()
      break;
    }

    if ( !file.EndRead3dmChunk() ) {
      rc = false;
      break;
    }

    if ( TCODE_ENDOFTABLE == tcode )
      break;
  }

  return rc;
}

ON_BOOL32 ON_3dmProperties::Write(ON_BinaryArchive& file) const
{
  ON_BOOL32 rc = true;

  // This short chunk identifies the version of OpenNURBS that was used to write this file.
  rc = file.BeginWrite3dmChunk(TCODE_PROPERTIES_OPENNURBS_VERSION,ON::Version());
  if ( rc) rc = file.EndWrite3dmChunk();

  // optional TCODE_PROPERTIES_REVISIONHISTORY chunk - file creation/revision information
  if ( rc && m_RevisionHistory.IsValid() ) {
    rc = file.BeginWrite3dmChunk(TCODE_PROPERTIES_REVISIONHISTORY,0);
    if ( rc ) {
      rc = m_RevisionHistory.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // optional TCODE_PROPERTIES_NOTES chunk - file notes
  if ( rc && m_Notes.IsValid() ) {
    rc = file.BeginWrite3dmChunk(TCODE_PROPERTIES_NOTES,0);
    if ( rc ) {
      rc = m_Notes.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // optional TCODE_PROPERTIES_PREVIEWIMAGE chunk - bitmap preview
  //if ( rc && m_PreviewImage.IsValid() ) {
  //  rc = file.BeginWrite3dmChunk(TCODE_PROPERTIES_PREVIEWIMAGE,0);
  //  if ( rc ) {
  //    rc = m_PreviewImage.WriteUncompressed(file);
  //    if ( !file.EndWrite3dmChunk() )
  //      rc = false;
  //  }
  //}

  // optional TCODE_PROPERTIES_COMPRESSED_PREVIEWIMAGE chunk - bitmap preview
  if ( rc && m_PreviewImage.IsValid() ) {
    rc = file.BeginWrite3dmChunk(TCODE_PROPERTIES_COMPRESSED_PREVIEWIMAGE,0);
    if ( rc ) {
      rc = m_PreviewImage.WriteCompressed(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // optional TCODE_PROPERTIES_APPLICATION chunk - application information
  if ( rc && m_Application.IsValid() ) {
    rc = file.BeginWrite3dmChunk(TCODE_PROPERTIES_APPLICATION,0);
    if ( rc ) {
      rc = m_Application.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // required TCODE_ENDOFTABLE chunk - marks end of properties table
  if ( rc ) {
    rc = file.BeginWrite3dmChunk( TCODE_ENDOFTABLE, 0 );
    if ( rc ) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  return rc;
}

void ON_3dmProperties::Dump( ON_TextLog& dump ) const
{
  dump.Print("Revision history:\n");
  dump.PushIndent();
  m_RevisionHistory.Dump(dump);
  dump.PopIndent();

  dump.Print("\n");
  dump.Print("Notes:\n");
  if ( m_Notes.m_notes.Length() > 0 ) {
    dump.PushIndent();
    m_Notes.Dump(dump);
    dump.PopIndent();
  }

  dump.Print("\n");
  dump.Print("Application information:\n");
  dump.PushIndent();
  m_Application.Dump(dump);
  dump.PopIndent();

  if ( m_PreviewImage.IsValid() ) {
    dump.Print("\n");
    dump.Print("Preview image:\n");
    dump.PushIndent();
    m_PreviewImage.Dump(dump);
    dump.PopIndent();
  }
}


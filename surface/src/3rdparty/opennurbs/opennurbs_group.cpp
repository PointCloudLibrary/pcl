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

ON_OBJECT_IMPLEMENT( ON_Group, ON_Object, "721D9F97-3645-44c4-8BE6-B2CF697D25CE" );

ON_Group::ON_Group() : m_group_index(-1)
{
  memset(&m_group_id,0,sizeof(m_group_id));
}

ON_Group::~ON_Group()
{
}

//////////////////////////////////////////////////////////////////////
//
// ON_Object overrides

ON_BOOL32 ON_Group::IsValid( ON_TextLog* text_log ) const
{
  return ( m_group_name.Length() > 0 && m_group_index >= 0 );
}

void ON_Group::Dump( ON_TextLog& dump ) const
{
  const wchar_t* name = GroupName();
  if ( !name )
    name = L"";
  dump.Print("group index = %d\n",m_group_index);
  dump.Print("group name = \"%ls\"\n",name);
}

ON_BOOL32 ON_Group::Write(
       ON_BinaryArchive& file // serialize definition to binary archive
     ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,1);
  // version 1.0 fields
  if (rc) rc = file.WriteInt(m_group_index);
  if (rc) rc = file.WriteString(m_group_name);
  // version 1.1 fields
  if (rc) rc = file.WriteUuid(m_group_id);
  return rc;
}

ON_BOOL32 ON_Group::Read(
       ON_BinaryArchive& file // restore definition from binary archive
     )
{
  m_group_index = -1;
  m_group_name.Empty();
  memset(&m_group_id,0,sizeof(m_group_id));
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( major_version == 1 ) 
  {
    if (rc) rc = file.ReadInt( &m_group_index );
    if (rc) rc = file.ReadString( m_group_name );
    if ( minor_version >= 1 )
    {
      if (rc) rc = file.ReadUuid( m_group_id );
    }
  }
  else
    rc = false;
  return rc;
}

//////////////////////////////////////////////////////////////////////
//
// Interface
void ON_Group::SetGroupName( const wchar_t* s )
{
  m_group_name = s;
}

void ON_Group::SetGroupName( const char* s )
{
  m_group_name = s;
}

void ON_Group::GetGroupName( ON_wString& s ) const
{
  s = m_group_name;
}

const wchar_t* ON_Group::GroupName() const
{
  const wchar_t* s = m_group_name;
  return s;
}

void ON_Group::SetGroupIndex(int i )
{
  m_group_index = i;
}

int ON_Group::GroupIndex() const
{
  return m_group_index;
}


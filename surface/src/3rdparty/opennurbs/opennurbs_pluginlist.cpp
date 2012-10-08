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

ON_PlugInRef::ON_PlugInRef()
{
  Default();
}

void ON_PlugInRef::Default()
{
  memset(&m_plugin_id,0,sizeof(m_plugin_id));
  m_plugin_type = 0;
  m_plugin_platform = 0;
  m_plugin_sdk_version = 0;
  m_plugin_sdk_service_release = 0;
  m_plugin_name.Destroy();
  m_plugin_version.Destroy();
  m_plugin_filename.Destroy(); // name of executable file

  m_developer_organization.Destroy();
  m_developer_address.Destroy();
  m_developer_country.Destroy();
  m_developer_phone.Destroy();
  m_developer_email.Destroy();
  m_developer_website.Destroy();
  m_developer_updateurl.Destroy();
  m_developer_fax.Destroy();
}

bool ON_PlugInRef::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,2);
  if (rc)
  {
    //version 1.0 fields
    if (rc) rc = file.WriteUuid(m_plugin_id);
    if (rc) rc = file.WriteInt(m_plugin_type);
    if (rc) rc = file.WriteString(m_plugin_name);
    if (rc) rc = file.WriteString(m_plugin_version);
    if (rc) rc = file.WriteString(m_plugin_filename);

    // version 1.1 fields 
    if (rc) rc = file.WriteString(m_developer_organization);
    if (rc) rc = file.WriteString(m_developer_address);
    if (rc) rc = file.WriteString(m_developer_country);
    if (rc) rc = file.WriteString(m_developer_phone);
    if (rc) rc = file.WriteString(m_developer_email);
    if (rc) rc = file.WriteString(m_developer_website);
    if (rc) rc = file.WriteString(m_developer_updateurl);
    if (rc) rc = file.WriteString(m_developer_fax);

    // version 1.2 fields
    if (rc) rc = file.WriteInt(m_plugin_platform);
    if (rc) rc = file.WriteInt(m_plugin_sdk_version);
    if (rc) rc = file.WriteInt(m_plugin_sdk_service_release);

    if( !file.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

void ON_PlugInRef::Dump(ON_TextLog& text_log) const
{
  text_log.Print("Plug-in executable information:\n");
  text_log.PushIndent();
  text_log.Print("id = ");text_log.Print(m_plugin_id);text_log.Print("\n");
  text_log.Print("type = %d\n",m_plugin_type);
  text_log.Print("platform = %d\n",m_plugin_platform);
  text_log.Print("sdk version = %d.%d\n",m_plugin_sdk_version,m_plugin_sdk_service_release);
  text_log.Print("name = "); text_log.Print(m_plugin_name); text_log.Print("\n");
  text_log.Print("version = "); text_log.Print(m_plugin_version); text_log.Print("\n");
  text_log.Print("file name = "); text_log.Print(m_plugin_filename); text_log.Print("\n");
  text_log.PopIndent();

  text_log.Print("Developer information:\n");
  text_log.PushIndent();
  text_log.Print("website url = ");text_log.Print(m_developer_website); text_log.Print("\n");
  text_log.Print("update url = ");text_log.Print(m_developer_updateurl); text_log.Print("\n");
  text_log.Print("organization = ");text_log.Print(m_developer_organization); text_log.Print("\n");
  text_log.Print("address = ");text_log.Print(m_developer_address); text_log.Print("\n");
  text_log.Print("country = ");text_log.Print(m_developer_country); text_log.Print("\n");
  text_log.Print("voice = ");text_log.Print(m_developer_phone); text_log.Print("\n");
  text_log.Print("email = ");text_log.Print(m_developer_email); text_log.Print("\n");
  text_log.Print("fax = ");text_log.Print(m_developer_fax); text_log.Print("\n");
  text_log.PopIndent();

}


bool ON_PlugInRef::Read( ON_BinaryArchive& file )
{
  Default();

  int major_version = 0;
  int minor_version = 0;

  bool rc = file.BeginRead3dmChunk(
                  TCODE_ANONYMOUS_CHUNK,
                  &major_version,
                  &minor_version);

  if (rc)
  {
    if( 1 == major_version && minor_version >= 0 )
    {
      //version 1.0 fields
      if (rc) rc = file.ReadUuid(m_plugin_id);
      if (rc) rc = file.ReadInt(&m_plugin_type);
      if (rc) rc = file.ReadString(m_plugin_name);
      if (rc) rc = file.ReadString(m_plugin_version);
      if (rc) rc = file.ReadString(m_plugin_filename);

      if ( minor_version >= 1)
      {
        // version 1.1 fields
        if (rc) rc = file.ReadString(m_developer_organization);
        if (rc) rc = file.ReadString(m_developer_address);
        if (rc) rc = file.ReadString(m_developer_country);
        if (rc) rc = file.ReadString(m_developer_phone);
        if (rc) rc = file.ReadString(m_developer_email);
        if (rc) rc = file.ReadString(m_developer_website);
        if (rc) rc = file.ReadString(m_developer_updateurl);
        if (rc) rc = file.ReadString(m_developer_fax);

        if ( minor_version >= 2 )
        {
          if (rc) rc = file.ReadInt(&m_plugin_platform);
          if (rc) rc = file.ReadInt(&m_plugin_sdk_version);
          if (rc) rc = file.ReadInt(&m_plugin_sdk_service_release);
        }
      }
    }
    else
    {
      rc = false;
    }

    if( !file.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}



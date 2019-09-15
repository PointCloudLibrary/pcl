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

#define ON_BOZO_VACCINE_11EE2C1FF90D4C6AA7CDEC8532E1E32D
#define ON_BOZO_VACCINE_F42D967121EB46929B9ABC3507FF28F5

ON_OBJECT_IMPLEMENT( ON_InstanceDefinition, ON_Geometry, "26F8BFF6-2618-417f-A158-153D64A94989" );

ON_InstanceDefinition::ON_InstanceDefinition()
{
  m_uuid = ON_nil_uuid;
  m_idef_update_type = ON_InstanceDefinition::static_def;
  m_idef_update_depth = 0;
  m_us.m_unit_system = ON::no_unit_system;
  m_us.m_custom_unit_scale = 0.0;
  m_us.m_custom_unit_name.Destroy();
  m_source_bRelativePath = false;
  m_reserved1 = 0;
  m_idef_layer_style = 0;
  m_reserved2[0] = 0;
  m_reserved2[1] = 0;
}

ON_InstanceDefinition::~ON_InstanceDefinition()
{
}


void ON_InstanceDefinition::Dump( ON_TextLog& text_log ) const
{
  const wchar_t* wsIDefName = m_name;
  if ( 0 == wsIDefName ) 
    wsIDefName = L"";
  text_log.Print("Name: \"%ls\"\n",wsIDefName);


  text_log.Print("Type: ");
  switch(m_idef_update_type)
  {
  case ON_InstanceDefinition::static_def:
    text_log.Print("embedded.");
    break;
  case ON_InstanceDefinition::embedded_def:
    if ( m_source_archive.Length() > 0 )
      text_log.Print("OBSOLETE embedded_def with non-empty source archive - should be linked_and_embedded_def.");
    else
      text_log.Print("OBSOLETE embedded_def with empty source archive - should be static_def.");
    break;
  case ON_InstanceDefinition::linked_and_embedded_def:
    text_log.Print("embedded and linked - definition from source archive.");
    break;
  case ON_InstanceDefinition::linked_def:
    text_log.Print("linked - definition from source archive.");
    break;
  default:
    text_log.Print("not valid");
    break;
  }
  text_log.Print("\n");


  text_log.Print("Id: "); text_log.Print(m_uuid); text_log.Print("\n");

  const wchar_t* wsDescription = m_description;
  if ( 0 != wsDescription && 0 != wsDescription[0]) 
    text_log.Print("Description: \"%ls\"\n",wsDescription);

  const wchar_t* wsURL = m_url;
  if ( 0 != wsURL && 0 != wsURL[0]) 
    text_log.Print("URL: \"%ls\"\n",wsURL);

  const wchar_t* wsTag = m_url_tag;
  if ( 0 != wsTag && 0 != wsTag[0]) 
    text_log.Print("URL tag: \"%ls\"\n",wsTag);

  m_us.Dump(text_log);

  const wchar_t* wsSourceArchive = SourceArchive();
  text_log.Print("Source archive: ");
  if ( 0 == wsSourceArchive || 0 == wsSourceArchive[0] )
  {
    text_log.Print("none.\n");
  }
  else
  {
    bool bRel = m_source_bRelativePath;
    text_log.Print("\"%ls\" (%s)\n",wsSourceArchive,bRel?"relative":"absolute");

    text_log.PushIndent();
    ON_wString str;
    bRel = false;
    if ( GetAlternateSourceArchivePath(str,bRel) )
    {
      const wchar_t* wsAlternateArchive = str;
      if ( 0 == wsAlternateArchive || 0 == wsAlternateArchive[0] )
        wsAlternateArchive = L"";
      text_log.Print("Alternate archive: \"%ls\" (%s)\n",wsAlternateArchive,bRel?"relative":"absolute");
    }

    text_log.Print("Update depth: %d\n",m_idef_update_type);

    text_log.Print("Archive ");
    m_source_archive_checksum.Dump(text_log);    
    text_log.PopIndent();
  }

  const int id_count = m_object_uuid.Count();
  text_log.Print("Contains: %d objects\n",id_count);

  if ( id_count > 0 )
  {
    text_log.PushIndent();

    text_log.Print(m_object_uuid[0]); 
    text_log.Print("\n");

    if ( id_count > 4 )
    {
      text_log.Print("...\n");
    }
    else
    {
      for ( int i = 1; i < id_count; i++ )
      {
        text_log.Print(m_object_uuid[i]);
        text_log.Print("\n");
      }
    }

    text_log.PopIndent();
  }

  m_bbox.Dump(text_log);
}

ON_BOOL32 ON_InstanceDefinition::IsValid( ON_TextLog* text_log ) const
{
  if ( 0 == ON_UuidCompare( m_uuid, ON_nil_uuid) )
  {
    if (text_log)
    {
      text_log->Print("ON_InstanceDefinition has nil uuid.\n");
    }
    return false;
  }
  if ( !m_bbox.IsValid() )
  {
    if (text_log)
    {
      text_log->Print("ON_InstanceDefinition has invalid bounding box.\n");
    }
    return false;
  }

  switch( m_idef_update_type)
  {
    case static_def:
      // no source archive information should be present
      if ( m_source_archive.Length() > 0 )
      {
        if (text_log)
        {
          text_log->Print("ON_InstanceDefinition is static but m_source_archive is not empty.\n");
        }
        return false;
      }
      if ( m_source_archive_checksum.IsSet() )
      {
        if (text_log)
        {
          text_log->Print("ON_InstanceDefinition is static but m_source_archive_checksum is set.\n");
        }
        return false;
      }

      if ( 0 != m_idef_layer_style )
      {
        if (text_log)
        {
          text_log->Print("ON_InstanceDefinition is static but m_idef_layer_style is not zero.\n");
        }
        return false;
      }
      break;

    case embedded_def: // embedded_def is obsolete - 
      if ( text_log )
      {
        text_log->Print("ON_InstanceDefinition.m_idef_update_type = obsolete \"embedded_idef\". Use \"static_def\" or \"linked_and_embedded_def\".\n");
      }
      return false;
      break;

    case linked_and_embedded_def:
    case linked_def:
      // source archive information is required
      if( m_source_archive.IsEmpty())
      {
        if (text_log)
        {
          text_log->Print("ON_InstanceDefinition is linked or embedded but m_source_archive is empty.\n");
        }
        return false;
      }
      if( !m_source_archive_checksum.IsSet())
      {
        if (text_log)
        {
          text_log->Print("ON_InstanceDefinition is linked or embedded but m_source_archive_checksum is zero.\n");
        }
        return false;
      }

      if ( linked_def == m_idef_update_type )
      {
        if ( 1 != m_idef_layer_style && 2 != m_idef_layer_style )
        {
          if (text_log)
          {
            text_log->Print("ON_InstanceDefinition is linked_def but m_idef_layer_style is not 1 or 2.\n");
          }
          return false;
        }
      }
      else
      {
        if ( 0 != m_idef_layer_style )
        {
          if (text_log)
          {
            text_log->Print("ON_InstanceDefinition is linked_and_embedded_def but m_idef_layer_style is not zero.\n");
          }
          return false;
        }
      }

      break;

    default:
      if ( text_log )
      {
        text_log->Print("ON_InstanceDefinition.m_idef_update_type value is invalid.\n");
      }
      return false;
      break;
  }

  // TODO
  return true;
}


unsigned int ON_InstanceDefinition::SizeOf() const
{
  unsigned int sz = sizeof(*this) - sizeof(ON_Geometry);
  sz += ON_Geometry::SizeOf();
  sz += this->m_object_uuid.SizeOfArray();
  sz += this->m_name.SizeOf();
  sz += this->m_description.SizeOf();
  sz += this->m_url.SizeOf();
  sz += this->m_url_tag.SizeOf();
  sz += this->m_source_archive.SizeOf();
  return sz;
}


ON_BOOL32 ON_InstanceDefinition::Write(
       ON_BinaryArchive& binary_archive
     ) const
{
  bool rc = binary_archive.Write3dmChunkVersion(1,6);

  // version 1.0 fields
  if ( rc )
    rc = binary_archive.WriteUuid( m_uuid );
  if ( rc )
  {
    if (    binary_archive.Archive3dmVersion() >= 4
         && ON_InstanceDefinition::linked_def == m_idef_update_type )
    {
      // linked instance definition geometry is never in the file
      ON_SimpleArray<ON_UUID> empty_uuid_list;
      rc = binary_archive.WriteArray( empty_uuid_list );
    }
    else
    {
      rc = binary_archive.WriteArray( m_object_uuid );
    }
  }
  if ( rc )
    rc = binary_archive.WriteString( m_name );
  if ( rc )
    rc = binary_archive.WriteString( m_description );
  if ( rc )
    rc = binary_archive.WriteString( m_url );
  if ( rc )
    rc = binary_archive.WriteString( m_url_tag );
  if ( rc )
    rc = binary_archive.WriteBoundingBox( m_bbox );

  // m_idef_update_type was an unsigned int and got changed to an enum.  Read and write
  // as an unsigned int to support backwards compatibility
  const unsigned int idef_update_type = (unsigned int)this->IdefUpdateType();
  if ( rc )
    rc = binary_archive.WriteInt( idef_update_type );
  if ( rc )
  {
    // 7 February 2012
    //   Purge source archive information from static_defs
    if ( ON_InstanceDefinition::static_def == idef_update_type )
    {
      ON_wString empty_string;
      rc = binary_archive.WriteString( empty_string );
    }
    else
      rc = binary_archive.WriteString( m_source_archive );
  }
  
  // version 1.1 fields
  if (rc)
  {
    // 7 February 2012
    //   Purge source archive information from static_defs
    if ( ON_InstanceDefinition::static_def == idef_update_type )
      ON_CheckSum::UnsetCheckSum.Write(binary_archive);
    else
      rc = m_source_archive_checksum.Write( binary_archive );
  }
  
  // version 1.2 fields
  if (rc)
    rc = binary_archive.WriteInt( m_us.m_unit_system );

  // version 1.3 fields - added 6 March 2006
  if (rc)
    rc = binary_archive.WriteDouble( m_us.m_custom_unit_scale );

  if ( rc )
  {
    bool b = (ON_InstanceDefinition::static_def == idef_update_type)
           ? false
           : m_source_bRelativePath;
    rc = binary_archive.WriteBool( b );
  }

  // version 1.4 fields
  if (rc)
    rc = m_us.Write(binary_archive);

  // version 1.5 fields
  if (rc)
    rc = binary_archive.WriteInt(m_idef_update_depth);

  // version 1.6 fields ( added 14 February 2012 )
  if (rc)
    rc = binary_archive.WriteInt(  m_idef_layer_style );

  return rc;
}

ON_InstanceDefinition::IDEF_UPDATE_TYPE ON_InstanceDefinition::IdefUpdateType() const
{
  if ( ON_InstanceDefinition::embedded_def == m_idef_update_type )
  {
    ON_ERROR("Using obsolete ON_InstanceDefinition::embedded_def value - fix code.");
    const_cast< ON_InstanceDefinition* >(this)->m_idef_update_type 
                                  = ( m_source_archive.Length() > 0 )
                                  ? ON_InstanceDefinition::linked_and_embedded_def
                                  : ON_InstanceDefinition::static_def;
  }
  return m_idef_update_type;
}

ON_InstanceDefinition::IDEF_UPDATE_TYPE ON_InstanceDefinition::IdefUpdateType(int i)
{
  IDEF_UPDATE_TYPE t;
  switch(i)
  {
  case ON_InstanceDefinition::static_def:
    t = ON_InstanceDefinition::static_def;
    break;
  case ON_InstanceDefinition::embedded_def:
    t = ON_InstanceDefinition::embedded_def;
    break;
  case ON_InstanceDefinition::linked_and_embedded_def:
    t = ON_InstanceDefinition::linked_and_embedded_def;
    break;
  case ON_InstanceDefinition::linked_def:
    t = ON_InstanceDefinition::linked_def;
    break;
  default:
    t = ON_InstanceDefinition::static_def;
    break;
  }
  return t;
}


ON_BOOL32 ON_InstanceDefinition::Read(
       ON_BinaryArchive& binary_archive
     )
{
  int major_version = 0;
  int minor_version = 0;

  m_idef_layer_style = 0;

  m_us.m_custom_unit_scale = 0.0;
  m_us.m_custom_unit_name.Destroy();
  m_us.m_unit_system = ON::no_unit_system;
  m_source_bRelativePath = false;
  m_source_archive.Destroy();

  bool rc = binary_archive.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc )
  {
    if ( major_version != 1 )
      rc = false;
    // version 1.0 fields
    if ( rc )
      rc = binary_archive.ReadUuid( m_uuid );
    if ( rc )
      rc = binary_archive.ReadArray( m_object_uuid );
    if ( rc )
      rc = binary_archive.ReadString( m_name );
    if ( rc )
      rc = binary_archive.ReadString( m_description );
    if ( rc )
      rc = binary_archive.ReadString( m_url );
    if ( rc )
      rc = binary_archive.ReadString( m_url_tag );
    if ( rc )
      rc = binary_archive.ReadBoundingBox( m_bbox );
    // m_idef_update_type was an unsigned int and got changed to an enum.  Read and write
    // as an unsigned int to support backwards compatibility
    unsigned int source = m_idef_update_type;
    if ( rc )
      rc = binary_archive.ReadInt( &source );
    if( rc)
      m_idef_update_type = ON_InstanceDefinition::IdefUpdateType(source);
    if ( rc )
      rc = binary_archive.ReadString( m_source_archive );

    // version 1.1 fields
    if ( minor_version >= 1 )
    {
      if ( rc )
        rc = m_source_archive_checksum.Read( binary_archive );
    }

    // version 1.2 fields
    if ( minor_version >= 2 )
    {
      int us = ON::no_unit_system;
      if ( rc )
        rc = binary_archive.ReadInt( &us );
      m_us.m_unit_system = ON::UnitSystem(us);
      if ( ON::custom_unit_system != m_us.m_unit_system && ON::no_unit_system != m_us.m_unit_system )
      {
        m_us.m_custom_unit_scale = ON::UnitScale( m_us.m_unit_system, ON::meters );
      }
      else
      {
        m_us.m_custom_unit_scale = 0.0;
      }

      if ( minor_version >= 3 )
      {
        // version 1.3 fields - added 6 March 2006
        //int us = ON::no_unit_system;
        if ( rc )
          rc = binary_archive.ReadDouble( &m_us.m_custom_unit_scale );
        if ( rc )
          rc = binary_archive.ReadBool( &m_source_bRelativePath );
        if ( rc && minor_version >= 4 )
        {
          rc = m_us.Read(binary_archive);
          if (rc && minor_version >= 5 )
          {
            rc = binary_archive.ReadInt(&m_idef_update_depth);

            if ( rc && minor_version >= 6 )
            {
              unsigned int i = 0;
              rc = binary_archive.ReadInt(&i);
              if ( i && i > 0 && i < 256 )
                m_idef_layer_style = (unsigned char)i;
            }
          }
        }
      }
    }

    if ( ON_InstanceDefinition::embedded_def == m_idef_update_type )
    {
      // 7 February 2012
      //   "embedded_def" is obsolete.
      if (m_source_archive.Length() > 0 )
        m_idef_update_type = ON_InstanceDefinition::linked_and_embedded_def;
      else
        DestroySourceArchive(); // convert to static
    }

    if ( ON_InstanceDefinition::linked_def == m_idef_update_type )
    {
      if ( m_idef_layer_style < 1 || m_idef_layer_style > 2 )
      {
        // The goal of the next if/else clause is for Rhino users
        // to see what they saw when they created the file.
        if ( binary_archive.Archive3dmVersion() < 50 )
        {
          // V4 linked blocks and early V5 linked blocks treated
          // layers and materials the way newer "active" idefs work,
          // so when I read an archive with version < 50, the
          // default will be 1 for "active".  
          m_idef_layer_style = 1;
        }
        else
        {
          // The more recent V5 linked blocks treated layers and materials
          // the way "reference" style idefs work, so when I read an
          // archive with version >= 50 (meaning recent V5), the default
          // will be 2 for "reference".
          m_idef_layer_style = 2;
        }
      }
    }
    else
    {
      m_idef_layer_style= 0;
    }
  }
  return rc;
}

ON::object_type ON_InstanceDefinition::ObjectType() const
{
  return ON::instance_definition;
}


// virtual ON_Geometry overrides
int ON_InstanceDefinition::Dimension() const
{
  return 3;
}

ON_BOOL32 ON_InstanceDefinition::GetBBox(
       double* boxmin,
       double* boxmax,
       ON_BOOL32
       ) const
{
  if ( boxmin )
  {
    boxmin[0] = m_bbox.m_min.x;
    boxmin[1] = m_bbox.m_min.y;
    boxmin[2] = m_bbox.m_min.z;
  }
  if ( boxmax )
  {
    boxmax[0] = m_bbox.m_max.x;
    boxmax[1] = m_bbox.m_max.y;
    boxmax[2] = m_bbox.m_max.z;
  }
  return m_bbox.IsValid();
}

ON_BOOL32 ON_InstanceDefinition::Transform( 
       const ON_Xform&
       )
{
  // instance DEFs cannot be transformed
  return false;
}

const wchar_t* ON_InstanceDefinition::Name() const
{
  return m_name;
}

void ON_InstanceDefinition::SetName( const wchar_t* name )
{
  ON_wString s(name);
  s.TrimLeftAndRight();
  if ( s.IsEmpty() )
    m_name.Destroy();
  else
    m_name = s;
}


const wchar_t* ON_InstanceDefinition::URL() const
{
  return m_url;
}

void ON_InstanceDefinition::SetURL( const wchar_t* url )
{
  ON_wString s(url);
  s.TrimLeftAndRight();
  if ( s.IsEmpty() )
    m_url.Destroy();
  else
    m_url = s;
}

const wchar_t* ON_InstanceDefinition::URL_Tag() const
{
  return m_url_tag;
}

void ON_InstanceDefinition::SetURL_Tag( const wchar_t* url_tag )
{
  ON_wString s(url_tag);
  s.TrimLeftAndRight();
  if ( s.IsEmpty() )
    m_url_tag.Destroy();
  else
    m_url_tag = s;
}

ON_UUID ON_InstanceDefinition::Uuid() const
{
  return m_uuid;
}

void ON_InstanceDefinition::SetUuid( ON_UUID uuid )
{
  m_uuid = uuid;
}

const wchar_t* ON_InstanceDefinition::Description() const
{
  return m_description;
}

void ON_InstanceDefinition::SetDescription( const wchar_t* description )
{
  ON_wString s(description);
  s.TrimLeftAndRight();
  if ( s.IsEmpty() )
    m_description.Destroy();
  else
    m_description = s;
}

void ON_InstanceDefinition::SetBoundingBox( ON_BoundingBox bbox )
{
  m_bbox = bbox;
}

void ON_InstanceDefinition::SetSourceArchive(
        const wchar_t* source_archive, 
        ON_CheckSum checksum,
        ON_InstanceDefinition::IDEF_UPDATE_TYPE idef_update_type
        )
{
  ON_wString s(source_archive);
  s.TrimLeftAndRight();  
  if ( s.IsEmpty() )
  {
    DestroySourceArchive();
  }
  else
  {
    SetAlternateSourceArchivePath(0,false);
    m_source_archive = s;
    m_source_bRelativePath = false;
    m_source_archive_checksum = checksum;
    m_idef_update_type = ON_InstanceDefinition::IdefUpdateType(idef_update_type);
    if ( ON_InstanceDefinition::linked_def != m_idef_update_type )
    {
      m_idef_layer_style = 0;
    }
  }
}

void ON_InstanceDefinition::DestroySourceArchive()
{
  m_source_archive.Destroy();
  m_source_archive_checksum.Zero();
  m_source_bRelativePath = false;
  m_idef_update_type = ON_InstanceDefinition::static_def;
  m_idef_layer_style = 0;
  m_idef_update_depth = 0;
  SetAlternateSourceArchivePath(0,false);
}

const wchar_t* ON_InstanceDefinition::SourceArchive() const
{
  return m_source_archive;
}

ON_CheckSum ON_InstanceDefinition::SourceArchiveCheckSum() const
{
  return m_source_archive_checksum;
}

const ON_UnitSystem& ON_InstanceDefinition::UnitSystem() const
{
  return m_us;
}

void ON_InstanceDefinition::SetUnitSystem( ON::unit_system us )
{
  // make sure we are not getting garbage cast as an ON::unit_system
  if ( us == ON::UnitSystem(us) )
  {
    m_us.m_unit_system = us;
    if ( ON::custom_unit_system != m_us.m_unit_system )
    {
      m_us.m_custom_unit_scale = ( ON::no_unit_system == m_us.m_unit_system )
                               ? 0.0
                               : ON::UnitScale(ON::meters,m_us.m_unit_system);
    }
  }
}

void ON_InstanceDefinition::SetUnitSystem( const ON_UnitSystem& us )
{
  // make sure we are not getting garbage cast as an ON::unit_system
  if ( us.IsValid() )
  {
    m_us = us;
    if ( ON::custom_unit_system != m_us.m_unit_system )
    {
      m_us.m_custom_unit_scale = ( ON::no_unit_system == m_us.m_unit_system )
                               ? 0.0
                               : ON::UnitScale(ON::meters,m_us.m_unit_system);
    }
  }
}

ON_OBJECT_IMPLEMENT( ON_InstanceRef, ON_Geometry, "F9CFB638-B9D4-4340-87E3-C56E7865D96A" );

const double ON_InstanceRef::m_singular_xform_tol = 1.0e-6;

ON_InstanceRef::ON_InstanceRef()
{
  m_instance_definition_uuid = ON_nil_uuid;
  m_xform.Identity();
}

ON_BOOL32 ON_InstanceRef::IsValid( ON_TextLog* text_log ) const
{
  if ( 0 == ON_UuidCompare( m_instance_definition_uuid, ON_nil_uuid) )
  {
    if ( text_log )
      text_log->Print("ON_InstanceRef has nil m_instance_definition_uuid.\n");
    return false;
  }

  ON_Xform tmp = m_xform.Inverse()*m_xform;
  if ( !tmp.IsIdentity( ON_InstanceRef::m_singular_xform_tol ) )
  {
    if ( text_log )
      text_log->Print("ON_InstanceRef has singular m_xform.\n");
    return false;
  }
  return true;
}

ON_BOOL32 ON_InstanceRef::Write(
       ON_BinaryArchive& binary_archive
     ) const
{
  bool rc = binary_archive.Write3dmChunkVersion(1,0);
  if ( rc )
    rc = binary_archive.WriteUuid( m_instance_definition_uuid );
  if ( rc )
    rc = binary_archive.WriteXform( m_xform );
  if ( rc )
    rc = binary_archive.WriteBoundingBox( m_bbox );
  return rc;
}

ON_BOOL32 ON_InstanceRef::Read(
       ON_BinaryArchive& binary_archive
     )
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc )
  {
    if ( major_version != 1 )
      rc = false;
    if (rc )
      rc = binary_archive.ReadUuid( m_instance_definition_uuid );
    if ( rc )
      rc = binary_archive.ReadXform( m_xform );
    if ( rc )
      rc = binary_archive.ReadBoundingBox( m_bbox );
  }
  return rc;
}

ON::object_type ON_InstanceRef::ObjectType() const
{
  return ON::instance_reference;
}


// virtual ON_Geometry overrides
int ON_InstanceRef::Dimension() const
{
  return 3;
}

ON_BOOL32 ON_InstanceRef::GetBBox(
       double* boxmin,
       double* boxmax,
       ON_BOOL32 bGrowBox
       ) const
{
  if ( !boxmin || !boxmax )
  {
    bGrowBox = false;
  }
  else if ( bGrowBox )
  {
    bGrowBox = ON_BoundingBox(ON_3dPoint(boxmin),ON_3dPoint(boxmax)).IsValid();
  }

  if( m_bbox.IsValid() )
  {
    if( bGrowBox )
    {
      if( boxmin[0] > m_bbox.m_min.x ) boxmin[0] = m_bbox.m_min.x;
      if( boxmin[1] > m_bbox.m_min.y ) boxmin[1] = m_bbox.m_min.y;
      if( boxmin[2] > m_bbox.m_min.z ) boxmin[2] = m_bbox.m_min.z;

      if( boxmax[0] < m_bbox.m_max.x ) boxmax[0] = m_bbox.m_max.x;
      if( boxmax[1] < m_bbox.m_max.y ) boxmax[1] = m_bbox.m_max.y;
      if( boxmax[2] < m_bbox.m_max.z ) boxmax[2] = m_bbox.m_max.z;
    }
    else
    {
      if( boxmin )
      {
        boxmin[0] = m_bbox.m_min.x;
        boxmin[1] = m_bbox.m_min.y;
        boxmin[2] = m_bbox.m_min.z;
      }
      if( boxmax )
      {
        boxmax[0] = m_bbox.m_max.x;
        boxmax[1] = m_bbox.m_max.y;
        boxmax[2] = m_bbox.m_max.z;
      }
      bGrowBox = true;
    }
  }

  return bGrowBox;
}

ON_BOOL32 ON_InstanceRef::Transform( 
       const ON_Xform& xform
       )
{
  ON_Geometry::Transform(xform);
  m_xform = xform*m_xform;
  m_bbox.Transform(xform);
  return true;
}

bool ON_InstanceRef::IsDeformable() const
{
  // 25 Feb 2006 Dale Lear - this seems wrong to me.
  return true;
}

bool ON_InstanceRef::MakeDeformable()
{
  // 25 Feb 2006 Dale Lear - this seems wrong to me.
  return true;
}


class /*NEVER EXPORT THIS CLASS DEFINITION*/ ON__IDefLayerSettingsUserData : public ON_UserData
{
#if !defined(ON_BOZO_VACCINE_11EE2C1FF90D4C6AA7CDEC8532E1E32D)
#error Never copy this class definition or put this definition in a header file!
#endif
  ON_OBJECT_DECLARE(ON__IDefLayerSettingsUserData);

public:
  ON__IDefLayerSettingsUserData();
  ~ON__IDefLayerSettingsUserData();
  // default copy constructor and operator= work fine.

  ON__IDefLayerSettingsUserData(const ON__IDefLayerSettingsUserData& src);
  ON__IDefLayerSettingsUserData& operator=(const ON__IDefLayerSettingsUserData& src);


  static ON__IDefLayerSettingsUserData* FindOrCreate(const ON_InstanceDefinition& idef,bool bCreate);
  
private:
  void CreateHelper()
  {
    m_layers.Destroy();
    m_idef_layer_table_parent_layer = 0;
  }

  void CopyHelper(const ON__IDefLayerSettingsUserData& src)
  {
    m_layers.Reserve(src.m_layers.Count());
    for ( int i = 0; i < src.m_layers.Count(); i++ )
    {
      const ON_Layer* src_layer = src.m_layers[i];
      if ( 0 != src_layer )
      {
        m_layers.Append( new ON_Layer( *src_layer ) );
      }
    }
    
    if ( 0 != src.m_idef_layer_table_parent_layer )
    {
      m_idef_layer_table_parent_layer = new ON_Layer( *src.m_idef_layer_table_parent_layer );
    }

    m_runtime_layer_id_map = src.m_runtime_layer_id_map;
    m_runtime_layer_id_map.ImproveSearchSpeed();
  }

  void DestroyHelper()
  {
    for ( int i = 0; i < m_layers.Count(); i++ )
    {
      delete m_layers[i];
      m_layers[i] = 0;
    }
    m_layers.Destroy();
    if ( 0 != m_idef_layer_table_parent_layer )
    {
      delete m_idef_layer_table_parent_layer;
      m_idef_layer_table_parent_layer = 0;
    }
    m_runtime_layer_id_map.Empty();
  }

public:
  // virtual ON_Object override
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  // virtual ON_Object override
  unsigned int SizeOf() const;
  // virtual ON_Object override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;
  // virtual ON_Object override
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  // virtual ON_Object override
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);
  // virtual ON_UserData override
  ON_BOOL32 Archive() const;
  // virtual ON_UserData override
  ON_BOOL32 GetDescription( ON_wString& description );

public:
  // m_layers[] satisfies
  //  * has no null members
  //  * is always sorted by layer id
  //  * has no duplicate layer ids
  //  * has no nil layer ids
  //  * the m_layer_id and m_parent_layer_id
  //    values are from the linked file.
  ON_SimpleArray<ON_Layer*> m_layers; 

  // Settings for the layer that is the
  // parent of the layers in the linked
  // files layer table.  This layer is
  // not in the linked file and is not
  // saved in the layer table of file containing
  // the idef.  If null, it is created
  ON_Layer* m_idef_layer_table_parent_layer;

  // When a linked idef is inserted and a layer id 
  // collision occures, the runtime id of the layer
  // has to be changed. This list keeps track of the
  // changes so we can determine which runtime layer
  // correspondes to a layer in m_layers[].
  // The first index id in the pair is the runtime
  // id and the second id in the pair is the m_layer[]
  // id.
  ON_UuidPairList m_runtime_layer_id_map;
};

#undef ON_BOZO_VACCINE_11EE2C1FF90D4C6AA7CDEC8532E1E32D

ON_OBJECT_IMPLEMENT(ON__IDefLayerSettingsUserData,ON_UserData,"11EE2C1F-F90D-4C6A-A7CD-EC8532E1E32D");

ON__IDefLayerSettingsUserData* ON__IDefLayerSettingsUserData::FindOrCreate(const ON_InstanceDefinition& idef,bool bCreate)
{
  ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::Cast(idef.GetUserData(ON__IDefLayerSettingsUserData::m_ON__IDefLayerSettingsUserData_class_id.Uuid()));
  if ( !ud && bCreate )
  {
    ud = new ON__IDefLayerSettingsUserData();
    const_cast<ON_InstanceDefinition&>(idef).AttachUserData(ud);
  }
  return ud;
}

ON__IDefLayerSettingsUserData::ON__IDefLayerSettingsUserData()
{
  m_userdata_uuid = ON__IDefLayerSettingsUserData::m_ON__IDefLayerSettingsUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs5_id;
  m_userdata_copycount = 1;
  CreateHelper();
}

ON__IDefLayerSettingsUserData::~ON__IDefLayerSettingsUserData()
{
  DestroyHelper();
}

ON__IDefLayerSettingsUserData::ON__IDefLayerSettingsUserData(const ON__IDefLayerSettingsUserData& src)
: ON_UserData(src)
{
  m_userdata_uuid = ON__IDefLayerSettingsUserData::m_ON__IDefLayerSettingsUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs5_id;
  CreateHelper();
  CopyHelper(src);
}

ON__IDefLayerSettingsUserData& ON__IDefLayerSettingsUserData::operator=(const ON__IDefLayerSettingsUserData& src)
{
  if ( this != &src )
  {
    DestroyHelper();
    ON_UserData::operator=(src);
    CopyHelper(src);
  }
  return *this;
}

// virtual ON_Object override
ON_BOOL32 ON__IDefLayerSettingsUserData::IsValid( ON_TextLog* ) const
{
  return true;
}

// virtual ON_Object override
unsigned int ON__IDefLayerSettingsUserData::SizeOf() const
{
  return (unsigned int)(sizeof(*this));
}

// virtual ON_Object override
ON__UINT32 ON__IDefLayerSettingsUserData::DataCRC(ON__UINT32 current_remainder) const
{
  ON__UINT32 crc = current_remainder;
  for ( int i = 0; i < m_layers.Count(); i++ )
    crc = m_layers.DataCRC(crc);
  return crc;
}

// virtual ON_Object override
ON_BOOL32 ON__IDefLayerSettingsUserData::Write(ON_BinaryArchive& binary_archive) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if ( !rc )
    return false;

  rc = false;
  for(;;)
  {
    if ( !binary_archive.WriteArray(m_layers.Count(),m_layers.Array()) )
      break;

    // added in version 1.1 chunks
    bool bHaveParentLayer = ( 0 != m_idef_layer_table_parent_layer );
    if ( !binary_archive.WriteBool(bHaveParentLayer) )
      break;

    if ( bHaveParentLayer )
    {
      if ( !binary_archive.WriteObject(m_idef_layer_table_parent_layer) )
        break;
    }

    rc = true;
    break;
  }

  if ( !binary_archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_Object override
ON_BOOL32 ON__IDefLayerSettingsUserData::Read(ON_BinaryArchive& binary_archive)
{
  DestroyHelper();

  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  rc = false;
  while ( 1 == major_version )
  {
    if ( !binary_archive.ReadArray(m_layers) )
      break;

    if ( minor_version <= 0 )
    {
      rc = true;
      break;
    }

    // added in version 1.1 chunks
    bool bHaveParentLayer = false;
    if ( !binary_archive.ReadBool(&bHaveParentLayer) )
      break;

    if ( bHaveParentLayer )
    {
      ON_Object* p = 0;
      if ( !binary_archive.ReadObject(&p) || 0 == p )
      {
        if (p)
        {
          delete p;
          break;
        }
      }

      m_idef_layer_table_parent_layer = ON_Layer::Cast(p);
      if ( 0 == m_idef_layer_table_parent_layer )
      {
        delete p;
        break;
      }
    }

    rc = true;
    break;
  }

  if ( !binary_archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_UserData override
ON_BOOL32 ON__IDefLayerSettingsUserData::Archive() const
{
  // don't save empty settings
  return m_layers.Count() > 0;
}

// virtual ON_UserData override
ON_BOOL32 ON__IDefLayerSettingsUserData::GetDescription( ON_wString& description )
{
  description = L"Linked Instance Definition Layer Settings";
  return true;
}



bool ON_InstanceDefinition::HasLinkedIdefLayerSettings() const
{
  const ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,false);
  return ud ? (ud->m_layers.Count() > 0) : false;
}

///////*
//////Description:
//////  Get layer settings for the context where the idef
//////  is being used.
//////Parameters:
//////  layer_settings - [out]
//////    Each layer has the settings from the linked reference file
//////    and layer.GetSavedSettings() reports any changes for the
//////    corresponding layers in the current context.
//////*/
//////void GetLinkedIdefLayerSettings( class ON_ObjectArray<ON_Layer>& layer_settings ) const;
//////
///////*
//////Description:
//////  Get the most recently read layer settings from the reference file.
//////Parameters:
//////  layer_settings - [out]
//////    The pointers in this array point memory managed by this
//////    instance definition and may be deleted by other
//////    ON_InstanceDefinition layer settings tools. Use the
//////    returned information immediately or copy it to memory
//////    you are managing.
//////*/
//////void GetLinkedIdefLayerReferenceSettings( class ON_SimpleArray<const ON_Layer*>& layer_settings ) const;
//////
//////
//////void ON_InstanceDefinition::GetLinkedIdefLayerReferenceSettings( class ON_SimpleArray<const ON_Layer*>& layer_settings ) const
//////{
//////  const ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::LayerSettings(*this,false);
//////  const int count = 0 != ud ? ud->m_layers.Count() : 0;
//////  if ( count > 0 )
//////  {
//////    layer_settings.Reserve(count);
//////    for ( int i = 0; i < count; i++ )
//////    {
//////      if ( 0 != ud->m_layers[i] )
//////        layer_settings.Append(ud->m_layers[i]);
//////    }
//////  }
//////  else
//////  {
//////    layer_settings.SetCount(0);
//////  }
//////}
//////
//////void ON_InstanceDefinition::GetLinkedIdefLayerSettings( class ON_ObjectArray<ON_Layer>& layer_settings ) const
//////{
//////  layer_settings.SetCount(0);
//////
//////  // Get the layers with settings from the referenced file.
//////  ON_SimpleArray<const ON_Layer*> layers;
//////  GetLinkedIdefLayerReferenceSettings(layers);
//////
//////  // Update those settings with any modifications made in the idef's current context.
//////  layer_settings.Reserve(layers.Count());
//////  unsigned int settings;
//////  for ( int i = 0; i < layers.Count(); i++ )
//////  {
//////    if ( 0 != layers[i] )
//////    {
//////      ON_Layer& layer = layer_settings.AppendNew();
//////      layer = *layers[i];
//////      settings = 0;
//////      layer.GetSavedSettings(layer,settings);
//////    }
//////  }
//////}

static int compareLayerPtrId(const void* A, const void*B)
{
  if ( 0 == A )
  {
    return 0 == B ? 0 : -1;
  }
  if ( 0 == B )
  {
    return 1;
  }

  const ON_Layer* a = (0!=A) ? ( *((ON_Layer**)A) ) : 0;
  const ON_Layer* b = (0!=B) ? ( *((ON_Layer**)B) ) : 0;
  if ( 0 == a )
  {
    return (0 == b) ? 0 : -1;
  }
  if ( 0 == b )
  {
    return 1;
  }

  // NOTE WELL:
  //   Compare only m_layer_id.  Other values may differ and
  //   adding compares to them will break the code that uses
  //   this function.
  return ON_UuidCompare(a->m_layer_id,b->m_layer_id);
}

static int compareUuidIndexId(const void* a, const void* b)
{
  return ON_UuidIndex::CompareId((const ON_UuidIndex*)a,(const ON_UuidIndex*)b);
}

void ON_InstanceDefinition::UpdateLinkedIdefReferenceFileLayerRuntimeId( const ON_UuidPairList& id_map )
{
  ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,false);
  if ( 0 == ud || ud->m_layers.Count() <= 0 )
    return;
  ud->m_runtime_layer_id_map = id_map;
  ud->m_runtime_layer_id_map.ImproveSearchSpeed();
}

void ON_InstanceDefinition::UpdateLinkedIdefParentLayerSettings( const ON_Layer* linked_idef_parent_layer )
{
  bool bCreate = ( 0 != linked_idef_parent_layer );
  ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,bCreate);
  if ( 0 != ud && ud->m_idef_layer_table_parent_layer != linked_idef_parent_layer )
  {
    if ( ud->m_idef_layer_table_parent_layer )
    {
      delete ud->m_idef_layer_table_parent_layer;
      ud->m_idef_layer_table_parent_layer = 0;
    }
    if ( 0 != linked_idef_parent_layer )
    {
      ud->m_idef_layer_table_parent_layer = new ON_Layer( *linked_idef_parent_layer );
    }
  }
}

const ON_Layer* ON_InstanceDefinition::LinkedIdefParentLayerSettings() const
{
  ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,false);
  return (0 != ud) ? ud->m_idef_layer_table_parent_layer : 0;
}

void ON_InstanceDefinition::UpdateLinkedIdefReferenceFileLayerSettings( unsigned int layer_count, ON_Layer** layer_settings )
{
  ON__IDefLayerSettingsUserData* ud;

  if ( layer_count <= 0 || 0 == layer_settings )
  {
    // delete linked idef layer settings
    ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,false);
    if ( 0 != ud )
      delete ud;
    return;
  }

  // Create an index_map[] into the layer_settings[] array that is sorted
  // by layer_settings[]->m_layer_id
  ON_Workspace ws;
  int* index_map = (int*)ws.GetMemory(layer_count*sizeof(index_map[0]));
  ON_Sort(ON::quick_sort,index_map,layer_settings,layer_count,sizeof(layer_settings[0]),compareLayerPtrId);

  // Use index_map[] to get a unique list of layers with valid ids
  ON_UuidIndex* iddex = (ON_UuidIndex*)ws.GetMemory(layer_count*sizeof(iddex[0]));
  unsigned int iddex_count = 0;
  unsigned int i;
  ON_Layer* layer;
  for ( i = 0; i < layer_count; i++ )
  {
    layer = layer_settings[index_map[i]];
    if ( 0 == layer )
      continue;
    layer->SaveSettings(0,false); // remove any saved settings on input layers
    if ( ON_UuidIsNil(layer->m_layer_id) )
      continue;
    if ( iddex_count > 0 && iddex[iddex_count-1].m_id == layer->m_layer_id )
      continue;
    iddex[iddex_count].m_i = index_map[i];
    iddex[iddex_count].m_id = layer->m_layer_id;
    iddex_count++;
  }

  if ( iddex_count <= 0 )
  {
    // delete settings
    UpdateLinkedIdefReferenceFileLayerSettings(0,0);
    return;
  }

  // Create or get user data where the saved layer settings
  // are stored.
  ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,true);
  if ( 0 == ud )
    return;
    
  // Go through the saved settings that were previously
  // on this idef apply those settings to the layer_settings[]
  // list. Then delete the information from ud->m_layers[].
  ON_UuidIndex idx;
  idx.m_i = 0;
  unsigned int settings;
  for ( i = 0; i < ud->m_layers.UnsignedCount(); i++ )
  {
    if ( 0 == ud->m_layers[i] )
      continue;
    layer = ud->m_layers[i];
    ud->m_layers[i] = 0;
    for(;;)
    {
      settings = layer->SavedSettings();
      if ( 0 == settings )
        break; // no settings were modified
      idx.m_id = layer->m_layer_id;
      const ON_UuidIndex* idx0 = (const ON_UuidIndex*)bsearch(&idx,iddex,iddex_count,sizeof(iddex[0]),compareUuidIndexId);
      if ( 0 == idx0)
        break; // this layer is not in the current layer_settings[] list
      layer_settings[idx0->m_i]->SaveSettings(settings,false); // saves the layer settings found in linked file
      layer_settings[idx0->m_i]->Set(settings,*layer);   // applies modifications found on idef
      break;
    }
    delete layer;
  }

  // Save a copy of this information on the user data
  // so it will persist in the file containing the idef.
  ud->m_layers.SetCount(0);
  ud->m_layers.Reserve(iddex_count);
  for ( i = 0; i < iddex_count; i++ )
  {
    layer = new ON_Layer( *layer_settings[iddex[i].m_i] );
    ud->m_layers.Append(layer);
  }
}



void ON_InstanceDefinition::UpdateLinkedIdefLayerSettings( unsigned int layer_count, const ON_Layer*const* layer_settings )
{
  if ( layer_count <= 0 || 0 == layer_settings )
  {
    // delete linked idef layer settings
    UpdateLinkedIdefReferenceFileLayerSettings(0,0);
    return;
  }

  // Get layer information (saved on this idef) from the linked file 
  ON__IDefLayerSettingsUserData* ud = ON__IDefLayerSettingsUserData::FindOrCreate(*this,false);
  if ( 0 == ud )
    return;
  if ( ud->m_layers.Count() <= 0 )
  {
    delete ud;
    return;
  }

  // Apply any saved settings
  ON_Layer** ud_layers = ud->m_layers.Array();
  std::size_t ud_layers_count = ud->m_layers.Count();
  ON_Layer layerId;
  const ON_Layer* layerPtrId = &layerId;
  for ( unsigned int i = 0; i < layer_count; i++ )
  {
    const ON_Layer* layer1 = layer_settings[i];
    if ( !ud->m_runtime_layer_id_map.FindId1(layer1->m_layer_id,&layerId.m_layer_id) )
      layerId.m_layer_id = layer1->m_layer_id;
    ON_Layer** pp = (ON_Layer**)bsearch(&layerPtrId,ud_layers,ud_layers_count,sizeof(ud_layers[0]),compareLayerPtrId);
    ON_Layer* layer0 = (0 != pp) ? *pp : 0;
    if ( 0 == layer0 )
      continue;
    unsigned int settings0 = layer0->SavedSettings();
    unsigned int settings1 = ON_Layer::Differences(*layer0,*layer1);
    // settings0 = settings changes
    // new_settings = settings changed since we opened the model
    unsigned int new_settings = (settings1 ^ (settings0 & settings1));
    if ( 0 != new_settings )
      layer0->SaveSettings( new_settings, true );
    layer0->Set((settings0|settings1),*layer1);
  }
}



///////////////////////////////////////////////////////////////////


class /*NEVER EXPORT THIS CLASS DEFINITION*/ ON__IDefAlternativePathUserData : public ON_UserData
{
#if !defined(ON_BOZO_VACCINE_F42D967121EB46929B9ABC3507FF28F5)
#error Never copy this class definition or put this definition in a header file!
#endif
  ON_OBJECT_DECLARE(ON__IDefAlternativePathUserData);

public:
  ON__IDefAlternativePathUserData();
  ~ON__IDefAlternativePathUserData();
  // default copy constructor and operator= work fine.

  ON__IDefAlternativePathUserData(const ON__IDefAlternativePathUserData& src);
  ON__IDefAlternativePathUserData& operator=(const ON__IDefAlternativePathUserData& src);


  static ON__IDefAlternativePathUserData* FindOrCreate(const ON_InstanceDefinition& idef,bool bCreate);
  
private:
  void CreateHelper()
  {
    m_alternate_path.Destroy();
    m_bRelativePath = false;
  }

  void CopyHelper(const ON__IDefAlternativePathUserData& src)
  {
    m_alternate_path = src.m_alternate_path;
    m_bRelativePath = src.m_bRelativePath;
  }

  void DestroyHelper()
  {
    m_alternate_path.Destroy();
    m_bRelativePath = false;
  }

public:
  // virtual ON_Object override
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  // virtual ON_Object override
  unsigned int SizeOf() const;
  // virtual ON_Object override
  ON__UINT32 DataCRC(ON__UINT32 current_remainder) const;
  // virtual ON_Object override
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  // virtual ON_Object override
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);
  // virtual ON_UserData override
  ON_BOOL32 Archive() const;
  // virtual ON_UserData override
  ON_BOOL32 GetDescription( ON_wString& description );

public:
  ON_wString m_alternate_path;
  bool m_bRelativePath;
};

#undef ON_BOZO_VACCINE_F42D967121EB46929B9ABC3507FF28F5

ON_OBJECT_IMPLEMENT(ON__IDefAlternativePathUserData,ON_UserData,"F42D9671-21EB-4692-9B9A-BC3507FF28F5");

ON__IDefAlternativePathUserData* ON__IDefAlternativePathUserData::FindOrCreate(const ON_InstanceDefinition& idef,bool bCreate)
{
  ON__IDefAlternativePathUserData* ud = ON__IDefAlternativePathUserData::Cast(idef.GetUserData(ON__IDefAlternativePathUserData::m_ON__IDefAlternativePathUserData_class_id.Uuid()));
  if ( !ud && bCreate )
  {
    ud = new ON__IDefAlternativePathUserData();
    const_cast<ON_InstanceDefinition&>(idef).AttachUserData(ud);
  }
  return ud;
}

ON__IDefAlternativePathUserData::ON__IDefAlternativePathUserData()
{
  m_userdata_uuid = ON__IDefAlternativePathUserData::m_ON__IDefAlternativePathUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs5_id;
  m_userdata_copycount = 1;
  CreateHelper();
}

ON__IDefAlternativePathUserData::~ON__IDefAlternativePathUserData()
{
  DestroyHelper();
}

ON__IDefAlternativePathUserData::ON__IDefAlternativePathUserData(const ON__IDefAlternativePathUserData& src)
: ON_UserData(src)
{
  m_userdata_uuid = ON__IDefAlternativePathUserData::m_ON__IDefAlternativePathUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs5_id;
  CreateHelper();
  CopyHelper(src);
}

ON__IDefAlternativePathUserData& ON__IDefAlternativePathUserData::operator=(const ON__IDefAlternativePathUserData& src)
{
  if ( this != &src )
  {
    DestroyHelper();
    ON_UserData::operator=(src);
    CopyHelper(src);
  }
  return *this;
}

// virtual ON_Object override
ON_BOOL32 ON__IDefAlternativePathUserData::IsValid( ON_TextLog* ) const
{
  return !m_alternate_path.IsEmpty();
}

// virtual ON_Object override
unsigned int ON__IDefAlternativePathUserData::SizeOf() const
{
  return (unsigned int)(sizeof(*this) + m_alternate_path.SizeOf());
}

// virtual ON_Object override
ON__UINT32 ON__IDefAlternativePathUserData::DataCRC(ON__UINT32 current_remainder) const
{
  ON__UINT32 crc = ON_CRC32(current_remainder,sizeof(m_bRelativePath),&m_bRelativePath);
  crc = m_alternate_path.DataCRC(crc);
  return crc;
}

// virtual ON_Object override
ON_BOOL32 ON__IDefAlternativePathUserData::Write(ON_BinaryArchive& binary_archive) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;

  rc = false;
  for(;;)
  {
    if ( !binary_archive.WriteString(m_alternate_path) )
      break;
    if ( !binary_archive.WriteBool(m_bRelativePath) )
      break;
    rc = true;
    break;
  }

  if ( !binary_archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_Object override
ON_BOOL32 ON__IDefAlternativePathUserData::Read(ON_BinaryArchive& binary_archive)
{
  DestroyHelper();

  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  rc = false;
  while ( 1 == major_version )
  {
    if ( !binary_archive.ReadString(m_alternate_path) )
      break;
    if ( !binary_archive.ReadBool(&m_bRelativePath) )
      break;
    rc = true;
    break;
  }

  if ( !binary_archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_UserData override
ON_BOOL32 ON__IDefAlternativePathUserData::Archive() const
{
  // don't save empty settings
  return !m_alternate_path.IsEmpty();
}

// virtual ON_UserData override
ON_BOOL32 ON__IDefAlternativePathUserData::GetDescription( ON_wString& description )
{
  description = L"Linked Instance Definition Alternate Path";
  return true;
}


void ON_InstanceDefinition::SetAlternateSourceArchivePath( 
      const wchar_t* alternate_source_archive_path,
      bool bRelativePath
      )
{
  ON_wString s;
  if ( 0 != alternate_source_archive_path )
  {
    s = alternate_source_archive_path;
    s.TrimLeftAndRight();
    alternate_source_archive_path = s;
    if ( 0 != alternate_source_archive_path && 0 == alternate_source_archive_path[0] )
      alternate_source_archive_path = 0;
  }
  ON__IDefAlternativePathUserData* ud = ON__IDefAlternativePathUserData::FindOrCreate(*this,0!=alternate_source_archive_path);
  if ( 0 != ud )
  {
    if ( 0 == alternate_source_archive_path )
      delete ud;
    else
    {
      ud->m_alternate_path = alternate_source_archive_path;
      ud->m_bRelativePath = bRelativePath;
    }
  }
}

bool ON_InstanceDefinition::GetAlternateSourceArchivePath( 
      ON_wString& alternate_source_archive_path,
      bool& bRelativePath
      ) const
{
  const ON__IDefAlternativePathUserData* ud = ON__IDefAlternativePathUserData::FindOrCreate(*this,false);
  const wchar_t* s = (0 != ud) ? ((const wchar_t*)ud->m_alternate_path) : 0;
  if ( 0 != s && 0 != s[0] )
  {
    alternate_source_archive_path = s;
    bRelativePath = ud->m_bRelativePath;
  }
  else
  {
    alternate_source_archive_path.Destroy();
    bRelativePath = false;
  }
  return !alternate_source_archive_path.IsEmpty();
}

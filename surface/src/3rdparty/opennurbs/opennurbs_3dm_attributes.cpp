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

ON_OBJECT_IMPLEMENT( ON_3dmObjectAttributes, ON_Object, "A828C015-09F5-477c-8665-F0482F5D6996" );

ON_3dmObjectAttributes::ON_3dmObjectAttributes()
{
  Default();
}

ON_3dmObjectAttributes::~ON_3dmObjectAttributes()
{}

/*
void ON_3dmObjectAttributes::CopyHelper(const ON_3dmObjectAttributes& src)
{
  // private helper for the copy constructor and operator=.
  m_uuid = src.m_uuid;
  m_name = src.m_name;
  m_url  = src.m_url;
  m_layer_index = src.m_layer_index;
  m_material_index = src.m_material_index;
  m_color = src.m_color;
  m_object_decoration = src.m_object_decoration;
  m_wire_density = src.m_wire_density;
  m_mode = src.m_mode;
  m_color_source = src.m_color_source;
  m_material_source = src.m_material_source;
  m_group = src.m_group;
  m_bVisible = src.m_bVisible;
  m_linetype_index = src.m_linetype_index;
  m_linetype_source = src.m_linetype_source;
}
*/

/*
ON_3dmObjectAttributes::ON_3dmObjectAttributes(const ON_3dmObjectAttributes& src) 
                       : ON_Object(src)
{
  Default();
  CopyHelper(src);
}
*/

bool ON_3dmObjectAttributes::operator==(const ON_3dmObjectAttributes& other) const
{
  if ( ON_UuidCompare( m_uuid, other.m_uuid ) )
    return false;
  if ( m_name.Compare(other.m_name) )
    return false;
  if ( m_url.Compare(other.m_url) )
    return false;
  if ( m_layer_index != other.m_layer_index )
    return false;
  if ( m_material_index != other.m_material_index )
    return false;
  if ( m_linetype_index != other.m_linetype_index )
    return false;
  if ( m_color != other.m_color )
    return false;
  if ( m_plot_color != other.m_plot_color )
    return false;
  if ( m_display_order != other.m_display_order )
    return false;
  if ( m_object_decoration != other.m_object_decoration )
    return false;
  if ( m_wire_density != other.m_wire_density )
    return false;
  if ( m_mode != other.m_mode )
    return false;
  if ( m_color_source != other.m_color_source )
    return false;
  if ( m_linetype_source != other.m_linetype_source )
    return false;
  if ( m_plot_color_source != other.m_plot_color_source )
    return false;
  if ( m_material_source != other.m_material_source )
    return false;
  if ( m_plot_weight_mm != other.m_plot_weight_mm )
    return false;
  if ( m_plot_weight_source != other.m_plot_weight_source )
    return false;

  int count = m_group.Count();
  if ( count != other.m_group.Count() )
    return false;
  if ( count > 0 )
  {
    const int* a = m_group.Array();
    const int* b = other.m_group.Array();
    if ( memcmp( a, b, count*sizeof(*a) ) )
      return false;
  }

  if ( m_bVisible != other.m_bVisible )
    return false;

  if ( m_rendering_attributes.Compare(other.m_rendering_attributes) )
    return false;

  if ( m_space != other.m_space)
    return false;

  if ( m_viewport_id != other.m_viewport_id )
    return false;

  if ( m_dmref != other.m_dmref )
    return false;

  return true;
}

bool ON_3dmObjectAttributes::operator!=(const ON_3dmObjectAttributes& other) const
{
  return !ON_3dmObjectAttributes::operator==(other);
}


/*
ON_3dmObjectAttributes& ON_3dmObjectAttributes::operator=(const ON_3dmObjectAttributes& src )
{
  if ( this != &src ) 
  {
    ON_Object::operator=(src);
    CopyHelper(src);
  }
  return *this;
}
*/

void ON_3dmObjectAttributes::Default()
{
  PurgeUserData();
  m_uuid = ON_nil_uuid;
  m_name.Destroy();
  m_url.Destroy();
  m_layer_index = 0;
  m_linetype_index = -1; // continuous
  m_material_index = -1; // white diffuse
  m_rendering_attributes.Default();
  m_color = ON_Color(0,0,0);
  m_plot_color = ON_Color(0,0,0); // Do not change to ON_UNSET_COLOR
  m_display_order = 0;
  m_plot_weight_mm = 0.0;
  m_object_decoration = ON::no_object_decoration;
  m_wire_density = 1;
  m_mode = ON::normal_object;
  m_bVisible = true;
  m_color_source       = ON::color_from_layer;
  m_material_source    = ON::material_from_layer;
  m_linetype_source    = ON::linetype_from_layer;
  m_plot_color_source  = ON::plot_color_from_layer;
  m_plot_weight_source = ON::plot_weight_from_layer;
  m_group.Destroy();
  m_space = ON::model_space;
  m_viewport_id = ON_nil_uuid;
  m_dmref.Destroy();
}


// {9BBB37E9-2131-4fb8-B9C6-5524859B98B8}
const ON_UUID ON_ObsoletePageSpaceObjectId =
{ 0x9bbb37e9, 0x2131, 0x4fb8, { 0xb9, 0xc6, 0x55, 0x24, 0x85, 0x9b, 0x98, 0xb8 } };


bool ON_3dmObjectAttributes::ReadV5Helper( ON_BinaryArchive& file )
{
  unsigned char itemid, c;
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc && 2 != major_version )
    rc = false;

  itemid = 0xFF;

  while(rc)
  {
    if (!rc) break;
    rc = file.ReadUuid(m_uuid);
    if (!rc) break;
    rc = file.ReadInt(&m_layer_index);
    if (!rc) break;

    // read non-default settings - skip everything else
    rc = file.ReadChar(&itemid);
    if (!rc) break;
    if ( 0 == itemid )
      break;
    
    if ( 1 == itemid )
    {
      rc = file.ReadString(m_name);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 2 == itemid )
    {
      rc = file.ReadString(m_url);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 3 == itemid )
    {
      rc = file.ReadInt(&m_linetype_index);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 4 == itemid )
    {
      rc = file.ReadInt(&m_material_index);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 5 == itemid )
    {
      rc = m_rendering_attributes.Read(file);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 6 == itemid )
    {
      rc = file.ReadColor(m_color);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 7 == itemid )
    {
      rc = file.ReadColor(m_plot_color);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 8 == itemid )
    {
      rc = file.ReadDouble(&m_plot_weight_mm);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 9 == itemid )
    {
      rc = file.ReadChar(&c);
      if (!rc) break;
      m_object_decoration = ON::ObjectDecoration(c);
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 10 == itemid )
    {
      rc = file.ReadInt(&m_wire_density);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 11 == itemid )
    {
      rc = file.ReadBool(&m_bVisible);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 12 == itemid )
    {
      rc = file.ReadChar(&m_mode);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 13 == itemid )
    {
      rc = file.ReadChar(&m_color_source);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 14 == itemid )
    {
      rc = file.ReadChar(&m_plot_color_source);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 15 == itemid )
    {
      rc = file.ReadChar(&m_plot_weight_source);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 16 == itemid )
    {
      rc = file.ReadChar(&m_material_source);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 17 == itemid )
    {
      rc = file.ReadChar(&m_linetype_source);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 18 == itemid )
    {
      rc = file.ReadArray(m_group);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 19 == itemid )
    {
      rc = file.ReadChar(&c);
      if (!rc) break;
      m_space = ON::ActiveSpace(c);
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 20 == itemid )
    {
      rc = file.ReadUuid(m_viewport_id);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }
    if ( 21 == itemid )
    {
      rc = file.ReadArray(m_dmref);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }

    // items 1 - 21 are in chunk version 2.0 files
    if ( minor_version <= 0 )
      break;

    // 28 Nov. 2009 - S. Baer
    // Added m_display_order to version 2.1 files
    if ( 22 == itemid )
    {
      rc = file.ReadInt(&m_display_order);
      if (!rc) break;
      rc = file.ReadChar(&itemid);
      if ( !rc || 0 == itemid ) break;
    }

    if ( minor_version <= 1 )
      break;

    // Add new item reading above this code, and increment the "22"
    // in the following if statement to an appropriate value, and
    // update the comment.  Be sure to test reading of old and
    // new files by old and new code, before checking in your
    // changes.
    //
    if ( itemid > 22 )
    {
      // we are reading file written with code newer
      // than this code (minor_version > 1)
      itemid = 0;
    }

    break;
  }

  if ( rc && 0 != itemid )
  {
    ON_ERROR("Bug in ON_3dmObjectAttributes::ReadV5Helper or WriteV5Helper");
  }

  return rc;
}

ON_BOOL32 ON_3dmObjectAttributes::Read( ON_BinaryArchive& file )
{
  Default();
  if (    file.Archive3dmVersion() >= 5 
       && file.ArchiveOpenNURBSVersion() >= 200712190 )
  {
    return ReadV5Helper(file);
  }
  int i;
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc && major_version == 1 ) 
  {
    if (rc) rc = file.ReadUuid(m_uuid);
    if (rc) rc = file.ReadInt(&m_layer_index);
    if (rc) rc = file.ReadInt(&m_material_index);
    if (rc) rc = file.ReadColor(m_color);
    
    while(rc)
    {
      // OBSOLETE if (rc) rc = file.ReadLineStyle(m_line_style); // 23 March 2005 Dale Lear
      // replaced with
      short s = 0;
      double x;
      rc = file.ReadShort(&s); 
      if (!rc) break;
      if ( file.Archive3dmVersion() < 4 || file.ArchiveOpenNURBSVersion() < 200503170 )
      {
        // ignore unused linestyle info in old files
        // This bit keeps the curve arrowheads from V3 showing up
        // in V4.
        m_object_decoration = ON::ObjectDecoration( (s & ON::both_arrowhead) );
      }
      rc = file.ReadShort(&s);
      if (!rc) break;
      rc = file.ReadDouble(&x);
      if (!rc) break;
      rc = file.ReadDouble(&x);
      break;
    }

    if (rc) rc = file.ReadInt(&m_wire_density);
    if (rc) rc = file.ReadChar(&m_mode);

    if (rc) rc = file.ReadChar(&m_color_source);
    if (rc) m_color_source = (unsigned char)ON::ObjectColorSource(m_color_source);

    if (rc) rc = file.ReadChar(&m_linetype_source);
    if (rc) m_linetype_source = (unsigned char)ON::ObjectLinetypeSource(m_linetype_source);

    if (rc) rc = file.ReadChar(&m_material_source);
    if (rc) m_material_source = (unsigned char)ON::ObjectMaterialSource(m_material_source);

    if (rc) rc = file.ReadString(m_name);
    if (rc) rc = file.ReadString(m_url);

    m_bVisible = (Mode() != ON::hidden_object);
    if ( rc && minor_version >= 1 ) 
    {
      rc = file.ReadArray( m_group );
      if ( rc && minor_version >= 2 )
      {
        rc = file.ReadBool(&m_bVisible);

        if ( rc && minor_version >= 3 )
        {
          rc = file.ReadArray(m_dmref);     

          if (rc && minor_version >= 4 )
          {
            // 23 March 2005 Dale Lear
            //    Added m_plot_color_source and m_plot_color
            i = 0;
            if (rc) rc = file.ReadInt(&i);
            if (rc) m_object_decoration = ON::ObjectDecoration(i);
            if (rc) rc = file.ReadChar(&m_plot_color_source);
            if (rc) m_plot_color_source = (unsigned char)ON::PlotColorSource(m_plot_color_source);
            if (rc) rc = file.ReadColor( m_plot_color );
            if (rc) rc = file.ReadChar(&m_plot_weight_source);
            if (rc) m_plot_weight_source = (unsigned char)ON::PlotWeightSource(m_plot_weight_source);
            if (rc) rc = file.ReadDouble(&m_plot_weight_mm);


            if (rc && minor_version >= 5 )
            {
              // version 1.5 fields 11 April 2005
              if (rc) rc = file.ReadInt(&m_linetype_index);

              // version 1.6 fields 2 September 2005
              if (rc && minor_version >= 6 )
              {
                unsigned char uc = 0;
                rc = file.ReadChar(&uc);
                if (rc)
                {
                  m_space = (1 == uc) ? ON::page_space : ON::model_space;
                  m_dmref.Empty();
                  int i, count=0;
                  rc = file.ReadInt(&count);
                  if (rc && count > 0)
                  {
                    m_dmref.SetCapacity(count);
                    for ( i = 0; i < count && rc; i++)
                    {
                      ON_DisplayMaterialRef& dmr = m_dmref.AppendNew();
                      rc = file.ReadUuid(dmr.m_viewport_id);
                      if (rc) rc = file.ReadUuid(dmr.m_display_material_id);
                      if ( rc )
                      {
                        // Assigning an object to a page started out as
                        // using dmrs.  The way the runtime info is saved
                        // has changed, but, at this point, I can't change
                        // the way the information is saved in the file and
                        // it doesn't matter.
                        if ( 0 == ON_UuidCompare(&ON_ObsoletePageSpaceObjectId,&dmr.m_display_material_id) )
                        {
                          m_viewport_id = dmr.m_viewport_id;
                          m_dmref.Remove();
                        }
                      }
                    }
                    if ( 0 == m_dmref.Count() )
                      m_dmref.Destroy();
                  }
                }

                if ( rc && minor_version >= 7 )
                {
                  // version 1.7 fields 6 June 2006
                  if (rc) rc = m_rendering_attributes.Read(file);
                }
              }
            }
          }
        }
      }
    }
  }
  else 
  {
    rc = false;
  }
  return rc;
}

bool ON_3dmObjectAttributes::WriteV5Helper( ON_BinaryArchive& file ) const
{
  unsigned char c;
  // 29 Nov. 2009 S. Baer
  // Chunk version updated to 2.1 in order to support m_display_order
  bool rc = file.Write3dmChunkVersion(2,1);
  while(rc)
  {
    if (!rc) break;
    rc = file.WriteUuid(m_uuid);
    if (!rc) break;
    rc = file.WriteInt(m_layer_index);
    if (!rc) break;

    // write non-default settings - skip everything else
    if ( !m_name.IsEmpty() )
    {
      c = 1;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteString(m_name);
      if (!rc) break;
    }
    if ( !m_url.IsEmpty() )
    {
      c = 2;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteString(m_url);
      if (!rc) break;
    }
    if ( -1 != m_linetype_index )
    {
      c = 3;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteInt(m_linetype_index);
      if (!rc) break;
    }
    if ( -1 != m_material_index )
    {
      c = 4;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteInt(m_material_index);
      if (!rc) break;
    }
    if (    m_rendering_attributes.m_mappings.Count() > 0
         || m_rendering_attributes.m_materials.Count() > 0
         || true != m_rendering_attributes.m_bCastsShadows
         || true != m_rendering_attributes.m_bReceivesShadows
         || false != m_rendering_attributes.AdvancedTexturePreview()
         )
    {
      c = 5;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = m_rendering_attributes.Write(file);
      if (!rc) break;
    }
    if ( 0 != m_color )
    {
      c = 6;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteColor(m_color);
      if (!rc) break;
    }
    if ( 0 != m_plot_color )
    {
      c = 7;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteColor(m_plot_color);
      if (!rc) break;
    }
    if ( 0.0 != m_plot_weight_mm )
    {
      c = 8;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteDouble(m_plot_weight_mm);
      if (!rc) break;
    }
    if ( ON::no_object_decoration != m_object_decoration )
    {
      c = 9;
      rc = file.WriteChar(c);
      if (!rc) break;
      c = (unsigned char)m_object_decoration;
      rc = file.WriteChar(c);
      if (!rc) break;
    }
    if ( 1 != m_wire_density )
    {
      c = 10;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteInt(m_wire_density);
      if (!rc) break;
    }
    if ( true != m_bVisible )
    {
      c = 11;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteBool(m_bVisible);
      if (!rc) break;
    }
    if ( ON::normal_object != m_mode )
    {
      c = 12;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteChar(m_mode);
      if (!rc) break;
    }
    if ( ON::color_from_layer != m_color_source )
    {
      c = 13;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteChar(m_color_source);
      if (!rc) break;
    }
    if ( ON::plot_color_from_layer != m_plot_color_source )
    {
      c = 14;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteChar(m_plot_color_source);
      if (!rc) break;
    }
    if ( ON::plot_weight_from_layer != m_plot_weight_source )
    {
      c = 15;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteChar(m_plot_weight_source);
      if (!rc) break;
    }
    if ( ON::material_from_layer != m_material_source )
    {
      c = 16;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteChar(m_material_source);
      if (!rc) break;
    }
    if ( ON::linetype_from_layer != m_linetype_source )
    {
      c = 17;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteChar(m_linetype_source);
      if (!rc) break;
    }
    if ( m_group.Count() > 0 )
    {
      c = 18;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteArray(m_group);
      if (!rc) break;
    }
    if ( ON::model_space != m_space )
    {
      c = 19;
      rc = file.WriteChar(c);
      if (!rc) break;
      c = (unsigned char)m_space;
      rc = file.WriteChar(c);
      if (!rc) break;
    }
    if ( !ON_UuidIsNil(m_viewport_id) )
    {
      c = 20;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteUuid(m_viewport_id);
      if (!rc) break;
    }
    if ( m_dmref.Count() > 0 )
    {
      c = 21;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteArray(m_dmref);
      if (!rc) break;
    }

    // 29 Nov. 2009 - S. Baer
    // Only write m_display_order if it's value!=0
    // m_display_order is written to version 2.1 files
    if ( 0 != m_display_order )
    {
      c = 22;
      rc = file.WriteChar(c);
      if (!rc) break;
      rc = file.WriteInt(m_display_order);
      if (!rc) break;
    }

    // 0 indicates end of attributes;
    c = 0;
    rc = file.WriteChar(c);
    break;
  }
  return rc;
}

ON_BOOL32 ON_3dmObjectAttributes::Write( ON_BinaryArchive& file ) const
{
  if ( file.Archive3dmVersion() >= 5 )
  {
    // added at opennurbs version 200712190
    return WriteV5Helper(file);
  }

  bool rc = file.Write3dmChunkVersion(1,7);
  // version 1.0 fields
  if (rc) rc = file.WriteUuid(m_uuid);
  if (rc) rc = file.WriteInt(m_layer_index);
  if (rc) rc = file.WriteInt(m_material_index);
  if (rc) rc = file.WriteColor(m_color);

  if (rc)
  {
    // OBSOLETE if (rc) rc = file.WriteLineStyle(m_line_style); // 23 March 2005 Dale Lear
    short s;
    s = (short)m_object_decoration;
    if (rc) rc = file.WriteShort(s);
    s = 0;
    if (rc) rc = file.WriteShort(s);
    if (rc) rc = file.WriteDouble(0.0);
    if (rc) rc = file.WriteDouble(1.0);
  }

  if (rc) rc = file.WriteInt(m_wire_density);
  if (rc) rc = file.WriteChar(m_mode);
  if (rc) rc = file.WriteChar(m_color_source);
  if (rc) rc = file.WriteChar(m_linetype_source);
  if (rc) rc = file.WriteChar(m_material_source);
  if (rc) rc = file.WriteString(m_name);
  if (rc) rc = file.WriteString(m_url);

  // version 1.1 fields
  if (rc) rc = file.WriteArray(m_group);

  // version 1.2 fields
  if (rc) rc = file.WriteBool(m_bVisible);

  // version 1.3 fields
  if (rc) rc = file.WriteArray(m_dmref);

  // version 1.4 fields - 23 March 2005 Dale Lear
  if (rc) rc = file.WriteInt(m_object_decoration);
  if (rc) rc = file.WriteChar(m_plot_color_source);
  if (rc) rc = file.WriteColor(m_plot_color);
  if (rc) rc = file.WriteChar(m_plot_weight_source);
  if (rc) rc = file.WriteDouble(m_plot_weight_mm);

  // version 1.5 fields 11 April 2005
  if (rc) rc = file.WriteInt(m_linetype_index);

  // version 1.6 fields 2 September 2005
  if (rc)
  {
    unsigned char uc = 0;
    switch(m_space)
    {
    case ON::no_space:    uc = 0; break;
    case ON::model_space: uc = 0; break;
    case ON::page_space:  uc = 1; break;
    }
    rc = file.WriteChar(uc);
  }
  if (rc)
  {
    // 22 Sep 2006 - the way ON_3dmObjectAttiributes indicates
    // that an object is put on a particular page view changed
    // from being saved in the m_dmref[] list to using the
    // m_space and m_viewport_id settings.  But the file format
    // cannot change at this point.  So, the bAddPagespaceDMR
    // is here to save the page info in the old dmr format.
    int count = m_dmref.Count();
    if ( count < 0 )
      count = 0;
    bool bAddPagespaceDMR = ( ON::page_space == m_space && !ON_UuidIsNil(m_viewport_id) );
    rc = file.WriteInt( bAddPagespaceDMR ? (count+1) : count );
    if ( rc && bAddPagespaceDMR )
    {
      rc = file.WriteUuid(m_viewport_id);
      if (rc) rc = file.WriteUuid(ON_ObsoletePageSpaceObjectId);
    }
    int i;
    for ( i = 0; i < count && rc; i++ )
    {
      const ON_DisplayMaterialRef& dmr = m_dmref[i];
      rc = file.WriteUuid(dmr.m_viewport_id);
      if (rc) rc = file.WriteUuid(dmr.m_display_material_id);
    }
  }

  // version 1.7 fields 6 June 2006
  if (rc) rc = m_rendering_attributes.Write(file);

  return rc;
}


bool ON_3dmObjectAttributes::Transform( const ON_Xform& xform )
{
  // Please discuss any changes with Dale Lear.
  ON_Object::TransformUserData(xform);
  return m_rendering_attributes.Transform(xform);
}

ON_BOOL32 ON_3dmObjectAttributes::IsValid( ON_TextLog* text_log ) const
{
  if ( ON_UuidIsNil(m_uuid) )
  {
    if ( text_log )
    {
      text_log->Print("Object id is nil - this is not valid.\n");
    }
    return false;
  }

  if ( !m_rendering_attributes.IsValid(text_log) )
  {
    if ( text_log )
    {
      text_log->Print("Object rendering attributes are not valid.\n");
    }
    return false;
  }

  return true;
}

unsigned int ON_3dmObjectAttributes::SizeOf() const
{
  unsigned int sz = sizeof(*this) - sizeof(ON_Object) 
                  + m_name.Length()*sizeof(wchar_t)
                  + m_url.Length()*sizeof(wchar_t)
                  + m_group.SizeOfArray()
                  + ON_Object::SizeOf();
  return sz;
}

void ON_3dmObjectAttributes::Dump( ON_TextLog& dump ) const
{
  const wchar_t* wsName = m_name;
  if ( !wsName )
    wsName = L"";
  dump.Print("object name = \"%ls\"\n",wsName);

  dump.Print("object uuid = ");
  dump.Print(m_uuid);
  dump.Print("\n");

  const char* sMode = "unknown";
  switch( Mode() )
  {
  case ON::normal_object:
    sMode = "normal";
    break;
  case ON::hidden_object:
    sMode = "hidden";
    break;
  case ON::locked_object:
    sMode = "locked";
    break;
  default:
    sMode = "unknown";
    break;
  }
  dump.Print("object mode = %s\n",sMode); // sSMode is const char*

  dump.Print("object layer index = %d\n",m_layer_index);
  dump.Print("object material index = %d\n",m_material_index);
  const char* sMaterialSource = "unknown";
  switch(MaterialSource()) {
  case ON::material_from_layer: sMaterialSource = "layer material"; break;
  case ON::material_from_object: sMaterialSource = "object material"; break;
  case ON::material_from_parent: sMaterialSource = "parent material"; break;
  }
  dump.Print("material source = %s\n",sMaterialSource); // sMaterialSource is const char*
  const int group_count = GroupCount();
  if ( group_count > 0 ) {
    const int* group = GroupList();
    dump.Print("groups: ");
    int i;
    for ( i = 0; i < group_count; i++ ) {
      if ( i )
        dump.Print(",%d",group[i]);
      else
        dump.Print("%d",group[i]);
    }
    dump.Print("\n");
  }
}

ON::object_mode ON_3dmObjectAttributes::Mode() const
{
  return ON::ObjectMode( m_mode%16 );
}

void ON_3dmObjectAttributes::SetMode( ON::object_mode m )
{
  int om = ON::ObjectMode(m);
  int dm = DisplayMode();
  m_mode = (unsigned char)(16*dm+om);

  // temporary
  m_bVisible = (om != ON::hidden_object);
}

bool ON_3dmObjectAttributes::IsInstanceDefinitionObject() const
{
  return (ON::idef_object == Mode());
}

bool ON_3dmObjectAttributes::IsVisible() const
{
  return m_bVisible;
}

void ON_3dmObjectAttributes::SetVisible( bool bVisible )
{
  if ( m_bVisible != (bVisible?true:false) )
  {
    m_bVisible = (bVisible?true:false);

    // temporary
    if ( Mode() != ON::idef_object )
      SetMode( m_bVisible ? ON::normal_object : ON::hidden_object );
  }
}


ON::display_mode ON_3dmObjectAttributes::DisplayMode() const
{
  return ON::DisplayMode( m_mode/16 );
}

unsigned int ON_3dmObjectAttributes::ApplyParentalControl( 
        const ON_3dmObjectAttributes& parents_attributes,
        unsigned int control_limits
        )
{
  ON_ERROR("Do not use deprecated version of ON_3dmObjectAttributes::ApplyParentalControl()");
  ON_Layer bogus_layer;
  bogus_layer.m_layer_index = -1;
  return ApplyParentalControl(parents_attributes,bogus_layer,control_limits);
}

unsigned int ON_3dmObjectAttributes::ApplyParentalControl( 
        const ON_3dmObjectAttributes& parents_attributes,
        const ON_Layer& parent_layer,
        unsigned int control_limits
        )
{
  unsigned int rc = 0;

  if ( m_bVisible && !parents_attributes.m_bVisible )
  {
    if ( 0 != (0x01 & control_limits) )
    {
      rc |= 0x01;
      m_bVisible = false;
    }
  }

  if ( ON::color_from_parent == m_color_source )
  {
    if ( 0 != (0x02 & control_limits) )
    {
      rc |= 0x02;
      m_color_source = parents_attributes.m_color_source;
      m_color        = parents_attributes.m_color;
      // 2010 March 10 Dale Lear
      //   Changing the layer index is wrong!
      //   Color by parent means COLOR and not LAYER
      // WRONG! // m_layer_index = parents_attributes.m_layer_index;
      if ( ON::color_from_layer == m_color_source && parent_layer.m_layer_index >= 0 )
      {
        // this object will use the parent layer's color
        m_color_source = ON::color_from_object;
        m_color = parent_layer.m_color;
      }
    }
  }

  if ( ON::material_from_parent == m_material_source )
  {
    if ( 0 != (0x04 & control_limits) )
    {
      rc |= 0x04;
      m_material_source = parents_attributes.m_material_source;
      m_material_index  = parents_attributes.m_material_index;
      // 2010 March 10 Dale Lear
      //   Changing the layer index is wrong!
      //   Material by parent means MATERIAL and not LAYER
      // WRONG! // m_layer_index = parents_attributes.m_layer_index;
      if ( ON::material_from_layer == m_material_source && parent_layer.m_layer_index >= 0 )
      {
        // this object will use the parent layer's material
        m_material_source = ON::material_from_object;
        m_material_index = parent_layer.m_material_index;
      }
    }
  }

  if ( ON::plot_color_from_parent == m_plot_color_source )
  {
    if ( 0 != (0x08 & control_limits) )
    {
      rc |= 0x08;
      m_plot_color_source = parents_attributes.m_plot_color_source;
      m_plot_color        = parents_attributes.m_plot_color;
      if ( ON::plot_color_from_layer == m_plot_color_source && parent_layer.m_layer_index >= 0 )
      {
        // this object will use the parent layer's plot color
        m_plot_color_source = ON::plot_color_from_object;
        m_plot_color        = parent_layer.m_plot_color;
      }
    }
  }

  if ( ON::plot_weight_from_parent == m_plot_weight_source )
  {
    if ( 0 != (0x10 & control_limits) )
    {
      rc |= 0x10;
      m_plot_weight_source = parents_attributes.m_plot_weight_source;
      m_plot_weight_mm     = parents_attributes.m_plot_weight_mm;
      if ( ON::plot_weight_from_layer == m_plot_weight_source && parent_layer.m_layer_index >= 0 )
      {
        // this object will use the parent layer's plot weight
        m_plot_weight_source = ON::plot_weight_from_object;
        m_plot_weight_mm     = parent_layer.m_plot_weight_mm;
      }
    }
  }

  if ( ON::linetype_from_parent == m_linetype_source )
  {
    if ( 0 != (0x20 & control_limits) )
    {
      rc |= 0x20;
      m_linetype_source = parents_attributes.m_linetype_source;
      m_linetype_index  = parents_attributes.m_linetype_index;
      if ( ON::linetype_from_layer == m_linetype_source && parent_layer.m_layer_index >= 0 )
      {
        // this object will use the parent layer's line type
        m_linetype_source = ON::linetype_from_object;
        m_linetype_index  = parent_layer.m_linetype_index;
      }
    }
  }
  
  if ( 0 != (0x40 & control_limits) )
  {
    rc |= 0x40;
    m_display_order = parents_attributes.m_display_order;
  }

  return rc;
}


void ON_3dmObjectAttributes::SetDisplayMode( ON::display_mode m )
{
  int om = Mode();
  int dm = ON::DisplayMode(m);
  m_mode = (unsigned char)(16*dm+om);
}

ON::object_color_source ON_3dmObjectAttributes::ColorSource() const
{
  return ON::ObjectColorSource(m_color_source);
}

void ON_3dmObjectAttributes::SetColorSource( ON::object_color_source c )
{
  m_color_source = (unsigned char)ON::ObjectColorSource(c);
}

ON::object_linetype_source ON_3dmObjectAttributes::LinetypeSource() const
{
  return ON::ObjectLinetypeSource(m_linetype_source);
}

void ON_3dmObjectAttributes::SetLinetypeSource( ON::object_linetype_source c )
{
  m_linetype_source = (unsigned char)ON::ObjectLinetypeSource(c);
}

ON::object_material_source ON_3dmObjectAttributes::MaterialSource() const
{
  return ON::ObjectMaterialSource(m_material_source);
}

void ON_3dmObjectAttributes::SetMaterialSource( ON::object_material_source c )
{
  m_material_source = (unsigned char)ON::ObjectMaterialSource(c);
}


ON::plot_color_source ON_3dmObjectAttributes::PlotColorSource() const
{
  return ON::PlotColorSource(m_plot_color_source);
}

void ON_3dmObjectAttributes::SetPlotColorSource( ON::plot_color_source pcs )
{
  m_plot_color_source = (unsigned char)ON::PlotColorSource(pcs);
}

ON::plot_weight_source ON_3dmObjectAttributes::PlotWeightSource() const
{
  return ON::PlotWeightSource(m_plot_weight_source);
}

void ON_3dmObjectAttributes::SetPlotWeightSource( ON::plot_weight_source pws )
{
  m_plot_weight_source = (unsigned char)ON::PlotColorSource(pws);
}


int ON_3dmObjectAttributes::GroupCount() const
{
  return m_group.Count();
}

//////////
// returns an int array of GroupCount() zero based group indices
const int* ON_3dmObjectAttributes::GroupList() const
{
  return (m_group.Count()>0) ? m_group.Array() : 0;
}

int ON_3dmObjectAttributes::GetGroupList(ON_SimpleArray<int>& group_list) const
{
  group_list = m_group;
  return group_list.Count();
}

//////////
// returns true if object is in group with specified index
ON_BOOL32 ON_3dmObjectAttributes::IsInGroup(
  int group_index // zero based group index
  ) const
{
  ON_BOOL32 rc = false;
  const int count = m_group.Count();
  int i;
  for ( i = 0; i < count; i++ ) {
    if (m_group[i] == group_index) {
      rc = true;
      break;
    }
  }
  return rc;
}

ON_BOOL32 ON_3dmObjectAttributes::IsInGroups( int group_count, const int* group_list ) const
{
  // returns true if object is in any of the groups in the list
  ON_BOOL32 rc = false;
  if ( group_count > 0 && group_list ) {
    const int obj_group_count  = GroupCount();
    const int* obj_group_list = GroupList();
    // in practice these arrays will be very short and this search will be fast
    int i, j;
    for ( i = 0; i < obj_group_count; i++ ) for ( j = 0; j < group_count; j++ ) {
      if ( obj_group_list[i] == group_list[j] )
        return true;
    }
  }
  return rc;
}

ON_BOOL32 ON_3dmObjectAttributes::IsInGroups( const ON_SimpleArray<int>& group_list ) const
{
  return IsInGroups( group_list.Count(), group_list.Array() );
}


//////////
// Adds object to group with specified index (If object is already in
// group, nothing is changed.)
void ON_3dmObjectAttributes::AddToGroup(
  int group_index // zero based group index
  )
{
  if ( group_index >= 0 ) {
    if ( !IsInGroup(group_index) )
      m_group.Append(group_index);
  }
}

//////////
// returns the index of the last group in the group list
// or -1 if the object is not in any groups
int ON_3dmObjectAttributes::TopGroup() const
{
  const int* top_group = m_group.Last();
  return top_group ? *top_group : -1;
}

//////////
// removes the object from the last group in the group list
void ON_3dmObjectAttributes::RemoveFromTopGroup()
{
  int c = m_group.Count();
  if ( c > 0 ) {
    c--;
    m_group.SetCount(c);
  }
}

//////////
// Removes object from group with specified index.  If object is not
// in the group, nothing is changed.
void ON_3dmObjectAttributes::RemoveFromGroup(
  int group_index // zero based group index
  )
{
  int i;
  const int count = m_group.Count();
  for ( i = 0; i < count; i++ ) {
    if (m_group[i] == group_index) {
      m_group.Remove(i);
      break;
    }
  }
}

//////////
// Removes object from all groups.
void ON_3dmObjectAttributes::RemoveFromAllGroups()
{
  m_group.Destroy();
}


bool ON_3dmObjectAttributes::FindDisplayMaterialId( 
      const ON_UUID& viewport_id, 
      ON_UUID* display_material_id
      ) const
{
  bool rc = false;
  if ( m_dmref.Count() > 0 )
  {
    ON_DisplayMaterialRef search_material, found_material;
    search_material.m_viewport_id = viewport_id;
    if ( 0 != (rc = FindDisplayMaterialRef(search_material,&found_material)) )
    {
      if ( display_material_id )
        *display_material_id = found_material.m_display_material_id;
    }
  }
  return rc;
}


bool ON_3dmObjectAttributes::FindDisplayMaterialRef(
  const ON_DisplayMaterialRef& search_material,
  ON_DisplayMaterialRef* found_material
  ) const
{
  int i = m_dmref.Count();
  if ( i > 0 )
  {
    int j = -1;
    if ( search_material.m_viewport_id != ON_nil_uuid )
    {
      if ( search_material.m_display_material_id != ON_nil_uuid )
      {
        while(i--)
        {
          if ( (m_dmref[i].m_display_material_id == search_material.m_display_material_id) &&
               (m_dmref[i].m_viewport_id == search_material.m_viewport_id) )
          {
            if(found_material)
              *found_material = m_dmref[i];
            return true;
          }
        }
      }
      else
      {
        while(i--)
        {
          const ON_UUID& vid = m_dmref[i].m_viewport_id;
          if ( vid == search_material.m_viewport_id )
          {
            if(found_material)
              *found_material = m_dmref[i];
            return true;
          }
          if ( vid == ON_nil_uuid )
          {
            j = i;
          }
        }
        if ( j >= 0 )
        {
          if(found_material)
            *found_material = m_dmref[j];
          return true;
        }
      }
    }
    else
    {
      if ( search_material.m_display_material_id != ON_nil_uuid )
      {
        while(i--)
        {
          if ( m_dmref[i].m_display_material_id == search_material.m_display_material_id )
          {
            if ( m_dmref[i].m_viewport_id == ON_nil_uuid )
            {
              if(found_material)
                *found_material = m_dmref[i];
              return true;
            }
            if ( j < 0 )
              j = i;
          }
        }
        if ( j >= 0 )
        {
          if(found_material)
            *found_material = m_dmref[j];
          return true;
        }
      }
      else
      {
        while(i--)
        {
          if ( m_dmref[i].m_viewport_id == ON_nil_uuid )
          {
            if(found_material)
              *found_material = m_dmref[i];
            return true;
          }
        }
      }
    }
  }
  return false;
}


bool ON_3dmObjectAttributes::AddDisplayMaterialRef(
  ON_DisplayMaterialRef display_material
  )
{
  bool rc = false;
  if ( !(display_material.m_display_material_id == ON_nil_uuid) )
  {
    int i = m_dmref.Count();
    while(i--)
    {
      if ( m_dmref[i].m_viewport_id == display_material.m_viewport_id )
      {
         m_dmref[i] = display_material;
         return true;
      }
    }
    m_dmref.Append(display_material);
  }
  return rc;
}

void ON_3dmObjectAttributes::RemoveAllDisplayMaterialRefs()
{
  m_dmref.Destroy();
}

bool ON_3dmObjectAttributes::RemoveDisplayMaterialRef(
  ON_UUID viewport_id,
  ON_UUID display_material_id
  )
{
  bool rc = false;
  int i = m_dmref.Count();
  if ( i > 0 )
  {
    const bool bCheckViewportId = !ON_UuidIsNil(viewport_id);
    const bool bCheckMaterialId = !ON_UuidIsNil(display_material_id);
    if ( bCheckViewportId || bCheckMaterialId )
    {
      while(i--)
      {
        if ( bCheckViewportId && m_dmref[i].m_viewport_id != viewport_id )
          continue;
        if ( bCheckMaterialId && m_dmref[i].m_display_material_id != display_material_id )
          continue;

        // remove this item
        rc = true;
        m_dmref.Remove(i);
      }
    }
    else
    {
      // 20 Sep 2006 Dale Lear - this was added so we can
      // remove all entries with non-nil viewport and nil
      // uuid.
      while(i--)
      {
        if (   !ON_UuidIsNil(m_dmref[i].m_viewport_id)
             && ON_UuidIsNil(m_dmref[i].m_display_material_id)
           )
        {
          // remove this item
          rc = true;
          m_dmref.Remove(i);
        }
      }
    }
  }
  return rc;
}

int ON_3dmObjectAttributes::DisplayMaterialRefCount() const
{
  return m_dmref.Count();
}

// {1403A7E4-E7AD-4a01-A2AA-41DAE6BE7ECB}
const ON_UUID ON_DisplayMaterialRef::m_invisible_in_detail_id = 
{ 0x1403a7e4, 0xe7ad, 0x4a01, { 0xa2, 0xaa, 0x41, 0xda, 0xe6, 0xbe, 0x7e, 0xcb } };


ON_DisplayMaterialRef::ON_DisplayMaterialRef()
{
  m_viewport_id = ON_nil_uuid;
  m_display_material_id = ON_nil_uuid;
}

bool ON_DisplayMaterialRef::operator==(const ON_DisplayMaterialRef& other) const
{
  return (Compare(other)==0); 
}

bool ON_DisplayMaterialRef::operator!=(const ON_DisplayMaterialRef& other) const
{
  return (Compare(other)!=0); 
}

int ON_DisplayMaterialRef::Compare(const ON_DisplayMaterialRef& other) const
{
  int i = ON_UuidCompare(m_viewport_id,other.m_viewport_id);
  if (0==i)
    i = ON_UuidCompare(m_display_material_id,other.m_display_material_id);
  return i;
}

bool ON_DisplayMaterialRef::operator<(const ON_DisplayMaterialRef& other) const
{
  return (Compare(other)<0); 
}

bool ON_DisplayMaterialRef::operator<=(const ON_DisplayMaterialRef& other) const
{
  return (Compare(other)<=0); 
}

bool ON_DisplayMaterialRef::operator>(const ON_DisplayMaterialRef& other) const
{
  return (Compare(other)>0); 
}

bool ON_DisplayMaterialRef::operator>=(const ON_DisplayMaterialRef& other) const
{
  return (Compare(other)>=0); 
}


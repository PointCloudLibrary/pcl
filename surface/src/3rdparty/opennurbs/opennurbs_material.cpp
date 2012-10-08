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


////////////////////////////////////////////////////////////////
//   Class ON_Material
////////////////////////////////////////////////////////////////

ON_OBJECT_IMPLEMENT(ON_Material,ON_Object,"60B5DBBC-E660-11d3-BFE4-0010830122F0");

double ON_Material::m_max_shine = 255.0f;

double ON_Material::MaxShine()
{
  return m_max_shine;
}

void ON_Material::Default()
{
  PurgeUserData();

  m_material_index = 0;
  m_material_id = ON_nil_uuid;
  m_material_name.Destroy();
  m_flamingo_library.Destroy();

  m_ambient.SetRGB( 0, 0, 0 );
  m_diffuse.SetRGB( 128, 128, 128 );
  m_emission.SetRGB( 0, 0, 0 );
  m_specular.SetRGB( 255, 255, 255 );
  //m_reflection = m_specular;
  //m_transparent = m_diffuse;
  m_reflection.SetRGB( 255, 255, 255 );
  m_transparent.SetRGB( 255, 255, 255 );

  m_index_of_refraction = 1.0;
  m_reflectivity = 0.0;

  m_shine = 0.0;
  m_transparency = 0.0;

  m_bShared = false;
  m_bDisableLighting = false;
  m_reserved1[0] = 0;
  m_reserved1[1] = 0;
#if defined(ON_64BIT_POINTER)
  m_reserved2[0] = 0;
  m_reserved2[1] = 0;
  m_reserved2[2] = 0;
  m_reserved2[3] = 0;
#endif

  m_textures.Destroy();

  m_plugin_id = ON_nil_uuid;

  m_material_channel.Destroy();
}

// Default constructor
ON_Material::ON_Material() 
{
  Default();
}

ON_Material::~ON_Material()
{}

ON_BOOL32
ON_Material::IsValid( ON_TextLog* text_log ) const
{
  return true;
}


void
ON_Material::Dump( ON_TextLog& dump ) const
{
  const wchar_t* s;
  dump.Print("index = %d\n",MaterialIndex());
  dump.Print("id = "); dump.Print(m_material_id); dump.Print("\n");
  
  s = m_material_name;
  if ( !s ) 
    s = L"";
  dump.Print("name = \"%ls\"\n",s);
  
  dump.Print("ambient rgb = "); dump.PrintRGB( m_ambient ); dump.Print("\n");
  dump.Print("diffuse rgb = "); dump.PrintRGB( m_diffuse ); dump.Print("\n");
  dump.Print("emmisive rgb = "); dump.PrintRGB( m_emission ); dump.Print("\n");
  dump.Print("specular rgb = "); dump.PrintRGB( m_specular ); dump.Print("\n");
  dump.Print("reflection rgb = "); dump.PrintRGB( m_reflection ); dump.Print("\n");
  dump.Print("transparent rgb = "); dump.PrintRGB( m_transparent ); dump.Print("\n");
  dump.Print("shine = %g%%\n",100.0*m_shine/ON_Material::MaxShine() );
  dump.Print("transparency = %g%%\n",100.0*m_transparency);
  dump.Print("reflectivity = %g%%\n",100.0*m_reflectivity);
  dump.Print("index of refraction = %g\n",m_index_of_refraction);

  dump.Print("plug-in id = "); dump.Print(m_plugin_id); dump.Print("\n");
  int i;
  for( i = 0; i < m_textures.Count(); i++ )
  {
    dump.Print("texture[%d]:\n",i);
    dump.PushIndent();
    m_textures[i].Dump(dump);
    dump.PopIndent();
  }
}


ON_UUID ON_Material::MaterialPlugInUuid() const
{
  return m_plugin_id;
}

void ON_Material::SetMaterialPlugInUuid( ON_UUID u )
{
  m_plugin_id = u;
}

ON_BOOL32 ON_Material::Write( ON_BinaryArchive& file ) const
{
  bool rc = false;
  if ( file.Archive3dmVersion() <= 3 )
  {
    // V2 or V3 file format
    rc = WriteV3Helper(file);
  }
  else 
  {
    // V4 file format

    // The chunk version 2.0 prevents old V3 IO code from attempting
    // to read this material
    rc = file.Write3dmChunkVersion(2,0); // never change the 2,0


    // version 1.2 field (20061113*)
    // version 1.3 fields (20100917*)
    if (rc) rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,3);
    if (rc)
    {
      for(;;)
      {
        if ( rc ) rc = file.WriteUuid(m_material_id);
        if ( rc ) rc = file.WriteInt(m_material_index);
        if ( rc ) rc = file.WriteString(m_material_name);

        if ( rc ) rc = file.WriteUuid(m_plugin_id);

        if ( rc ) rc = file.WriteColor( m_ambient );
        if ( rc ) rc = file.WriteColor( m_diffuse );
        if ( rc ) rc = file.WriteColor( m_emission );
        if ( rc ) rc = file.WriteColor( m_specular );
        if ( rc ) rc = file.WriteColor( m_reflection );
        if ( rc ) rc = file.WriteColor( m_transparent );

        if ( rc ) rc = file.WriteDouble( m_index_of_refraction );
        if ( rc ) rc = file.WriteDouble( m_reflectivity );
        if ( rc ) rc = file.WriteDouble( m_shine );
        if ( rc ) rc = file.WriteDouble( m_transparency );

        if ( !rc )
          break;

        // array of textures written in a way that user data persists
        rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
        if (rc)
        {
          int i, count = m_textures.Count();
          rc = file.WriteInt(count);
          for ( i = 0; i < count && rc; i++ )
          {
            rc = file.WriteObject(&m_textures[i]);
          }
          if ( !file.EndWrite3dmChunk() )
            rc = false;
        }

        //version 1.1 field
        if (rc) rc = file.WriteString(m_flamingo_library);

        // version 1.2 field (20061113)
        if (rc) rc = file.WriteArray(m_material_channel);

        // version 1.3 fields (20100917*)
        rc = file.WriteBool(m_bShared);
        if (!rc) break;
        rc = file.WriteBool(m_bDisableLighting);
        if (!rc) break;

        break;
      }
      if (!file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

bool ON_Material::WriteV3Helper( ON_BinaryArchive& file ) const
{
  int i;
  // V2 and V3 file format

  bool rc = file.Write3dmChunkVersion(1,1);
  if ( rc ) rc = file.WriteColor( m_ambient );
  if ( rc ) rc = file.WriteColor( m_diffuse );
  if ( rc ) rc = file.WriteColor( m_emission );
  if ( rc ) rc = file.WriteColor( m_specular );
  if ( rc ) rc = file.WriteDouble( Shine() );
  if ( rc ) rc = file.WriteDouble( m_transparency );

  if ( rc ) rc = file.WriteChar( (unsigned char)1 ); // OBSOLETE // m_casts_shadows
  if ( rc ) rc = file.WriteChar( (unsigned char)1 ); // OBSOLETE // m_shows_shadows

  if ( rc ) rc = file.WriteChar( (unsigned char)0 ); // OBSOLETE // m_wire_mode
  if ( rc ) rc = file.WriteChar( (unsigned char)2 ); // OBSOLETE // m_wire_density

  if ( rc ) rc = file.WriteColor( ON_Color(0,0,0) ); // OBSOLETE // m_wire_color

  if (rc)
  {
    // OBSOLETE - line style info never used
    short s = 0;
    if (rc) rc = file.WriteShort(s);
    if (rc) rc = file.WriteShort(s);
    if (rc) rc = file.WriteDouble(0.0);
    if (rc) rc = file.WriteDouble(1.0);
  }  

  ON_wString filename;
  int j = 0;
  i = FindTexture( NULL, ON_Texture::bitmap_texture );
  if ( i >= 0 )
  {
    const ON_Texture& tmap = m_textures[i];
    if ( tmap.m_filename.Length() > 0  )
    {
      filename = tmap.m_filename;
      j = ( ON_Texture::decal_texture == tmap.m_mode ) ? 2 : 1;
    }
  }
  // OBSOLETE // if ( rc ) rc = file.WriteString( TextureBitmapFileName() );
  // OBSOLETE // i = TextureMode();
  // OBSOLETE // if ( rc ) rc = file.WriteInt( i );
  // OBSOLETE // if ( rc ) rc = file.WriteInt( m_texture_bitmap_index );
  if ( rc ) rc = file.WriteString(filename);
  if ( rc ) rc = file.WriteInt( j );
  if ( rc ) rc = file.WriteInt( 0 );

  filename.Destroy();
  j = 0;
  double bump_scale = 1.0;
  i = FindTexture( NULL, ON_Texture::bump_texture );
  if ( i >= 0 )
  {
    const ON_Texture& tmap = m_textures[i];
    if ( tmap.m_filename.Length() > 0  )
    {
      filename = tmap.m_filename;
      j = ( ON_Texture::decal_texture == tmap.m_mode ) ? 2 : 1;
      bump_scale = tmap.m_bump_scale[1];
    }
  }
  // OBSOLETE //if ( rc ) rc = file.WriteString( BumpBitmapFileName() );
  // OBSOLETE //i = BumpMode();
  // OBSOLETE //if ( rc ) rc = file.WriteInt( i );
  // OBSOLETE //if ( rc ) rc = file.WriteInt( m_bump_bitmap_index );
  // OBSOLETE //if ( rc ) rc = file.WriteDouble( m_bump_scale );
  if ( rc ) rc = file.WriteString( filename );
  if ( rc ) rc = file.WriteInt( j );
  if ( rc ) rc = file.WriteInt( 0 );
  if ( rc ) rc = file.WriteDouble( bump_scale );

  filename.Destroy();
  j = 0;
  i = FindTexture( NULL, ON_Texture::emap_texture );
  if ( i >= 0 )
  {
    const ON_Texture& tmap = m_textures[i];
    if ( tmap.m_filename.Length() > 0  )
    {
      filename = tmap.m_filename;
      j = ( ON_Texture::decal_texture == tmap.m_mode ) ? 2 : 1;
    }
  }
  // OBSOLETE //if ( rc ) rc = file.WriteString( EmapBitmapFileName() );
  // OBSOLETE //i = EmapMode();
  // OBSOLETE //if ( rc ) rc = file.WriteInt( i );
  // OBSOLETE //if ( rc ) rc = file.WriteInt( m_emap_bitmap_index );
  if ( rc ) rc = file.WriteString( filename );
  if ( rc ) rc = file.WriteInt( j );
  if ( rc ) rc = file.WriteInt( 0 );

  if ( rc ) rc = file.WriteInt( m_material_index );

  if ( rc ) rc = file.WriteUuid( m_plugin_id );
  if ( rc ) rc = file.WriteString( m_flamingo_library );
  if ( rc ) rc = file.WriteString( m_material_name );


  // 1.1 fields
  if (rc) rc = file.WriteUuid( m_material_id );
  if (rc) rc = file.WriteColor( m_reflection);
  if (rc) rc = file.WriteColor( m_transparent);
  if (rc) rc = file.WriteDouble( m_index_of_refraction );

  return rc;
}

ON_BOOL32 ON_Material::Read( ON_BinaryArchive& file )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc)
  {
    if ( 1 == major_version )
    {
      rc = ReadV3Helper(file,minor_version);
    }
    else if ( 2 == major_version )
    {
      // fancy V4 material
      // V4 file format

      // The chunk version 2.0 prevents old V3 IO code from attempting
      // to read this material
      rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
      if (rc)
      {
        for(;;)
        {
          if ( rc ) rc = file.ReadUuid(m_material_id);
          if ( rc ) rc = file.ReadInt(&m_material_index);
          if ( rc ) rc = file.ReadString(m_material_name);

          if ( rc ) rc = file.ReadUuid(m_plugin_id);

          if ( rc ) rc = file.ReadColor( m_ambient );
          if ( rc ) rc = file.ReadColor( m_diffuse );
          if ( rc ) rc = file.ReadColor( m_emission );
          if ( rc ) rc = file.ReadColor( m_specular );
          if ( rc ) rc = file.ReadColor( m_reflection );
          if ( rc ) rc = file.ReadColor( m_transparent );

          if ( rc 
               && file.ArchiveOpenNURBSVersion() < 200912010 
               && 128 == m_transparent.Red() 
               && 128 == m_transparent.Green()
               && 128 == m_transparent.Blue()
               )
          {
            // Prior to 1 Dec 2009 the ON_Material::Defaults() set
            // m_transparent to 128,128,128.  This was the wrong
            // value for the default.  This "hack" is here to 
            // make it appear that the default was always white.
            m_transparent = m_diffuse;
          }

          if ( rc ) rc = file.ReadDouble( &m_index_of_refraction );
          if ( rc ) rc = file.ReadDouble( &m_reflectivity );
          if ( rc ) rc = file.ReadDouble( &m_shine );
          if ( rc ) rc = file.ReadDouble( &m_transparency );

          if ( !rc )
            break;

          // array of textures read in a way that user data persists
          int texmajver = 0;
          int texminver = 0;
          rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&texmajver,&texminver);
          if (rc)
          {
            if ( 1 == texmajver )
            {
              int i, count = 0;
              rc = file.ReadInt(&count);
              if (rc) m_textures.Reserve(count);
              for ( i = 0; i < count && rc; i++ )
              {
                int trc = file.ReadObject(m_textures.AppendNew());
                if ( trc <= 0 )
                  rc = false;
                else if ( trc > 1 )
                  m_textures.Remove();
              }
            }
            if ( !file.EndRead3dmChunk() )
              rc = false;
          }

          if ( rc && minor_version >= 1 )
          {
            rc = file.ReadString(m_flamingo_library);
            if ( !rc ) break;

            if ( minor_version >= 2 )
            {
              // version 1.2 field (20061113)
              rc = file.ReadArray(m_material_channel);
              if (!rc) break;

              if ( minor_version >= 3 )
              {
                // version 1.3 fields (20100917*)
                rc = file.ReadBool(&m_bShared);
                if (!rc) break;
                rc = file.ReadBool(&m_bDisableLighting);
                if (!rc) break;
              }
            }

          }

          break;
        }
        if (!file.EndRead3dmChunk() )
          rc = false;
      }
    }
  }
  return rc;
}

bool ON_Material::ReadV3Helper( ON_BinaryArchive& file, int minor_version )
{
  double shine = 0.0, transparency = 0.0;
  int i, j;
  bool rc = true;
  {
    // common to all version 1.x formats
    if ( rc ) rc = file.ReadColor( m_ambient );
    if ( rc ) rc = file.ReadColor( m_diffuse );
    if ( rc ) rc = file.ReadColor( m_emission );
    if ( rc ) rc = file.ReadColor( m_specular );
    if ( rc ) rc = file.ReadDouble( &shine );
    if ( rc ) SetShine(shine);
    if ( rc ) rc = file.ReadDouble( &transparency );
    if ( rc ) SetTransparency(transparency);

    unsigned char obsolete_uc;
    if ( rc ) rc = file.ReadChar( &obsolete_uc ); // m_casts_shadows
    if ( rc ) rc = file.ReadChar( &obsolete_uc ); // m_shows_shadows

    if ( rc ) rc = file.ReadChar( &obsolete_uc ); // m_wire_mode
    if ( rc ) rc = file.ReadChar( &obsolete_uc ); // m_wire_density

    ON_Color obsolete_color;
    if ( rc ) rc = file.ReadColor( obsolete_color ); // m_wire_color

    if (rc)
    {
      // OBSOLETE if ( rc ) rc = file.ReadLineStyle( m_wire_style );
      short s;
      double x;
      if (rc) rc = file.ReadShort(&s);
      if (rc) rc = file.ReadShort(&s);
      if (rc) rc = file.ReadDouble(&x);
      if (rc) rc = file.ReadDouble(&x);
    }

    ON_wString str;

    if ( rc ) rc = file.ReadString( str ); //sTextureBitmapFileName
    i = 0;
    j = 0;
    if ( rc ) rc = file.ReadInt( &i );
    // OBSOLETE // if ( rc ) SetTextureMode( ON::TextureMode(i) );
    if ( rc ) rc = file.ReadInt( &j );//&m_texture_bitmap_index

    if ( rc && !str.IsEmpty() )
    {
      ON_Texture& texture = m_textures[AddTexture(str,ON_Texture::bitmap_texture)];
      if ( 2 == i )
      {
        texture.m_mode = ON_Texture::decal_texture;
      }
      else
      {
        texture.m_mode = ON_Texture::modulate_texture;
      }
    }

    if ( rc ) rc = file.ReadString( str ); // sBumpBitmapFileName
    if ( rc ) rc = file.ReadInt( &i );
   // OBSOLETE // if ( rc ) SetBumpMode( ON::TextureMode(i) );
    if ( rc ) rc = file.ReadInt( &j );//&m_bump_bitmap_index );
    double bump_scale = 0.0;
    if ( rc ) rc = file.ReadDouble( &bump_scale );

    if ( rc && !str.IsEmpty() )
    {
      ON_Texture& texture = m_textures[AddTexture(str,ON_Texture::bump_texture)];
      if ( 2 == i )
      {
        texture.m_mode = ON_Texture::decal_texture;
      }
      else
      {
        texture.m_mode = ON_Texture::modulate_texture;
      }
      texture.m_bump_scale.Set(0.0,bump_scale);
    }

    if ( rc ) rc = file.ReadString( str ); // sEmapBitmapFileName
    if ( rc ) rc = file.ReadInt( &i );
    // OBSOLETE // if ( rc ) SetEmapMode( ON::TextureMode(i) );
    if ( rc ) rc = file.ReadInt( &j ); //&m_emap_bitmap_index;

    if ( rc && !str.IsEmpty() )
    {
      ON_Texture& texture = m_textures[AddTexture(str,ON_Texture::emap_texture)];
      if ( 2 == i )
      {
        texture.m_mode = ON_Texture::decal_texture;
      }
      else
      {
        texture.m_mode = ON_Texture::modulate_texture;
      }
    }

    if ( rc ) rc = file.ReadInt( &m_material_index );

    if ( rc ) rc = file.ReadUuid( m_plugin_id );
    if ( rc ) rc = file.ReadString( m_flamingo_library );
    if ( rc ) rc = file.ReadString( m_material_name );

    if ( minor_version >= 1 )
    {
      // 1.1 fields
      if (rc) rc = file.ReadUuid( m_material_id );
      if (rc) rc = file.ReadColor( m_reflection);
      if (rc) rc = file.ReadColor( m_transparent);
      if (rc) rc = file.ReadDouble( &m_index_of_refraction );
    }
    else
    {
      // old material needs a valid id.
      ON_CreateUuid(m_material_id);
    }

  }

  return rc;
}

ON::object_type ON_Material::ObjectType() const
{
   return ON::material_object;
}

int ON_Material::FindTexture( const wchar_t* filename, 
                              ON_Texture::TYPE type,
                              int i0
                              ) const
{
  int i, count = m_textures.Count();
  for (i = ((i0 < 0) ? 0 : (i0+1)); i < count; i++ )
  {
    if (    type != m_textures[i].m_type 
         && type != ON_Texture::no_texture_type )
    {
      continue;
    }
    if ( filename && m_textures[i].m_filename.CompareNoCase(filename) )
    {
      continue;
    }
    return i;
  }
  return -1;
}

int ON_Material::FindTexture( ON_UUID texture_id ) const
{
  int i, count = m_textures.Count();
  for (i = 0; i < count; i++ )
  {
    if ( !ON_UuidCompare(&texture_id,&m_textures[i].m_texture_id) )
      return i;
  }
  return -1;
}

int ON_Material::DeleteTexture(const wchar_t* filename,ON_Texture::TYPE type )
{
  int deleted_count = 0;
  int i;

  if ( !filename && type == ON_Texture::no_texture_type )
  {
    deleted_count = m_textures.Count();
    m_textures.Destroy();
  }
  else
  {
    for ( i = m_textures.Count()-1; i >= 0; i--)
    {
      if ( type != ON_Texture::no_texture_type && type != m_textures[i].m_type )
        continue;
      if ( filename && m_textures[i].m_filename.CompareNoCase(filename) )
        continue;
      m_textures.Remove(i);
      deleted_count++;
    }
  }
  return deleted_count;
}

int ON_Material::AddTexture( const ON_Texture& tx )
{
  // has to copy user data
  int i = FindTexture( tx.m_filename, tx.m_type );
  if ( i < 0 )
  {
    i = m_textures.Count();
    m_textures.Append(tx);
  }
  else
  {
    m_textures[i] = tx;
  }
  if ( ON_UuidIsNil(m_textures[i].m_texture_id) )
  {
    ON_CreateUuid(m_textures[i].m_texture_id);
  }

  return i;
}


int ON_Material::AddTexture(const wchar_t* filename,ON_Texture::TYPE type)
{
  int ti = FindTexture(NULL,type);
  if ( ti < 0 )
  {
    ti = m_textures.Count();
    m_textures.AppendNew();
  }
  if (ti >= 0 )
  {
    m_textures[ti].m_filename = filename;
    m_textures[ti].m_type = type;
    m_textures[ti].m_mode = ON_Texture::modulate_texture;
    m_textures[ti].m_magfilter = ON_Texture::linear_filter;
    ON_CreateUuid(m_textures[ti].m_texture_id);
  }
  return ti;
}

// Shine values are in range 0.0 to ON_Material::GetMaxShine()
double ON_Material::Shine() const
{
  return m_shine;
}

void ON_Material::SetShine( double shine )
{
  if ( shine < 0.0 )
    m_shine = 0.0;
  else if ( shine > m_max_shine)
    m_shine = m_max_shine;
  else
    m_shine = (float)shine;
}

  // Transparency values are in range 0.0 = opaque to 1.0 = transparent
double ON_Material::Transparency( ) const
{
  return  m_transparency;
}

void ON_Material::SetTransparency( double transparency )
{
  if ( transparency < 0.0 )
    m_transparency = 0.0f;
  else if ( transparency > 1.0)
    m_transparency = 1.0f;
  else
    m_transparency = (float)transparency;
}

bool ON_Material::operator==( const ON_Material& src ) const
{
  return Compare(src) ? false : true;
}

bool ON_Material::operator!=( const ON_Material& src ) const
{
  return Compare(src) ? true : false;
}

static int CompareDouble( double a, double b )
{
  return ( ( a < b ) ? -1 : ((a > b) ? 1 : 0) );
}

static int CompareXform( const ON_Xform& a, const ON_Xform& b )
{
  int i,j;
  const double* da = &a.m_xform[0][0];
  const double* db = &b.m_xform[0][0];
  i = 16;
  j = 0;
  while ( i-- && !j)
  {
    j = CompareDouble(*da++,*db++);
  }

  return j;
}

static int CompareTextureDoubles( size_t count, const double* a, const double* b )
{
  while ( count-- )
  {
    if ( *a < *b )
      return -1;
    if ( *a > *b )
      return  1;
    a++;
    b++;
  }
  return 0;
}

int ON_Texture::Compare( const ON_Texture& other ) const
{
  int rc = ON_UuidCompare( &m_texture_id, &other.m_texture_id );
  while(!rc)
  {
    rc = m_mapping_channel_id - other.m_mapping_channel_id;
    if (rc) break;

    rc = m_filename.CompareNoCase(other.m_filename);    
    if (rc) break;

    rc = ((int)m_bOn) - ((int)other.m_bOn);
    if (rc) break;

    rc = ((int)m_type) - ((int)other.m_type);
    if (rc) break;

    rc = ((int)m_mode) - ((int)other.m_mode);
    if (rc) break;

    rc = ((int)m_minfilter) - ((int)other.m_minfilter);
    if (rc) break;

    rc = ((int)m_magfilter) - ((int)other.m_magfilter);
    if (rc) break;

    rc = ((int)m_wrapu) - ((int)other.m_wrapu);
    if (rc) break;

    rc = ((int)m_wrapv) - ((int)other.m_wrapv);
    if (rc) break;

    rc = ((int)m_wrapw) - ((int)other.m_wrapw);
    if (rc) break;

    rc = CompareXform(m_uvw, other.m_uvw);
    if (rc) break;

    rc = m_border_color.Compare(other.m_border_color);
    if (rc) break;

    rc = m_transparent_color.Compare(other.m_transparent_color);
    if (rc) break;

    rc = m_bump_scale.Compare(other.m_bump_scale);
    if (rc) break;

    rc = CompareTextureDoubles( 1, &m_blend_constant_A, &other.m_blend_constant_A );
    if (rc) break;

    rc = CompareTextureDoubles( sizeof(m_blend_A)/sizeof(m_blend_A[0]), m_blend_A, other.m_blend_A);
    if (rc) break;

    rc = CompareTextureDoubles( sizeof(m_blend_RGB)/sizeof(m_blend_RGB[0]), m_blend_RGB, other.m_blend_RGB);
    if (rc) break;

    break;
  }

  return rc;
}

int ON_Material::Compare( const ON_Material& other ) const
{
  // do NOT test m_material_index

  int rc = ON_UuidCompare( &m_material_id, &other.m_material_id );
  while(!rc)
  {
    rc = m_material_name.CompareNoCase( other.m_material_name );
    if (rc) break;

    rc = m_ambient.Compare(other.m_ambient);
    if (rc) break;

    rc = m_diffuse.Compare( other.m_diffuse );
    if (rc) break;

    rc = m_diffuse.Compare( other.m_diffuse );
    if (rc) break;

    rc = m_emission.Compare( other.m_emission );
    if (rc) break;

    rc = m_specular.Compare( other.m_specular );
    if (rc) break;

    rc = m_reflection.Compare( other.m_reflection );
    if (rc) break;

    rc = m_transparent.Compare( other.m_transparent );
    if (rc) break;

    rc = CompareDouble(m_index_of_refraction,other.m_index_of_refraction);
    if (rc) break;

    rc = CompareDouble(m_reflectivity,other.m_reflectivity);
    if (rc) break;

    rc = CompareDouble(m_shine,other.m_shine);
    if (rc) break;

    rc = CompareDouble(m_transparency,other.m_transparency);
    if (rc) break;

    rc = ON_UuidCompare( &m_plugin_id, &other.m_plugin_id );
    if (rc) break;

    const int tcount = m_textures.Count();
    rc = tcount - other.m_textures.Count();
    int i;
    for ( i = 0; i < tcount && !rc; i++ )
    {
      rc = m_textures[i].Compare( other.m_textures[i] );
    }
    if (rc) break;



    break;
  }

  return rc;  
}

ON_Color ON_Material::Ambient() const
{
  return m_ambient & 0x00FFFFFF;
}

ON_Color ON_Material::Diffuse( ) const
{
  return m_diffuse & 0x00FFFFFF;
}

ON_Color ON_Material::Emission( ) const
{
  return m_emission & 0x00FFFFFF;
}

ON_Color ON_Material::Specular() const
{
  return m_specular & 0x00FFFFFF;
}

void ON_Material::SetAmbient( ON_Color  c )
{
  m_ambient = c;
}

void ON_Material::SetDiffuse(  ON_Color c )
{
  m_diffuse = c ;
}

void ON_Material::SetEmission( ON_Color c )
{
  m_emission = c ;
}

void ON_Material::SetSpecular( ON_Color c )
{
  m_specular = c;
}

int ON_Material::MaterialIndex() const
{
  return m_material_index;
}

void ON_Material::SetMaterialIndex( int i )
{
  m_material_index = i;
}

const wchar_t* ON_Material::MaterialName( ) const
{
	return m_material_name;
}

void ON_Material::SetMaterialName( const wchar_t* sMaterialName )
{
  m_material_name = sMaterialName;
}

////////////////////////////////////////////////////////////////
//   Class ON_Texture
////////////////////////////////////////////////////////////////

ON_OBJECT_IMPLEMENT(ON_Texture,ON_Object,"D6FF106D-329B-4f29-97E2-FD282A618020");

ON_Texture::ON_Texture()
{
  Default(); // used to set defaults
}

ON_Texture::~ON_Texture()
{
}

ON_BOOL32 ON_Texture::IsValid( ON_TextLog* text_log ) const
{
  if ( no_texture_type == m_type || force_32bit_texture_type == m_type )
  {
    if ( text_log )
    {
      text_log->Print("ON_Texture m_type has invalid value.\n");
    }
    return false;
  }

  // TODO ...

  return true;
}

// overrides virtual ON_Object::Dump
void ON_Texture::Dump( ON_TextLog& ) const
{

}

// overrides virtual ON_Object::SizeOf
unsigned int ON_Texture::SizeOf() const
{
  unsigned int sz = ON_Object::SizeOf();
  sz += sizeof(*this) - sizeof(ON_Object);
  sz += m_filename.Length()*sizeof(wchar_t);
  return sz;
}

// overrides virtual ON_Object::Write
ON_BOOL32 ON_Texture::Write(
        ON_BinaryArchive& binary_archive
      ) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (rc)
  {

    for(;;)
    {
      // 1.0 values
      rc = binary_archive.WriteUuid(m_texture_id);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_mapping_channel_id);
      if (!rc) break;
      rc = binary_archive.WriteString(m_filename);
      if (!rc) break;
      rc = binary_archive.WriteBool(m_bOn);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_type);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_mode);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_minfilter);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_magfilter);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_wrapu);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_wrapv);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_wrapw);
      if (!rc) break;
      rc = binary_archive.WriteXform(m_uvw);
      if (!rc) break;
      rc = binary_archive.WriteColor(m_border_color);
      if (!rc) break;
      rc = binary_archive.WriteColor(m_transparent_color);
      if (!rc) break;
      rc = binary_archive.WriteUuid(m_transparency_texture_id);
      if (!rc) break;
      rc = binary_archive.WriteInterval(m_bump_scale);
      if (!rc) break;
      rc = binary_archive.WriteDouble(m_blend_constant_A);
      if (!rc) break;
      rc = binary_archive.WriteDouble(4,m_blend_A);
      if (!rc) break;
      rc = binary_archive.WriteColor(m_blend_constant_RGB);
      if (!rc) break;
      rc = binary_archive.WriteDouble(4,m_blend_RGB);
      if (!rc) break;
      rc = binary_archive.WriteInt(m_blend_order);
      if (!rc) break;

      break;
    }


    if ( !binary_archive.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

ON_Texture::TYPE ON_Texture::TypeFromInt( int i )
{
  ON_Texture::TYPE type = no_texture_type;

  switch((unsigned int)i)
  {
  case no_texture_type:
    type = no_texture_type;
    break;
  case bitmap_texture:
    type = bitmap_texture;
    break;
  case bump_texture:
    type = bump_texture;
    break;
  case emap_texture:
    type = emap_texture;
    break;
  case transparency_texture:
    type = transparency_texture;
    break;
  case force_32bit_texture_type:
    type = force_32bit_texture_type;
    break;
  }

  return type;
}

ON_Texture::MODE ON_Texture::ModeFromInt( int i )
{
  ON_Texture::MODE mode = no_texture_mode;
  switch((unsigned int)i)
  {
  case no_texture_mode:
    mode = no_texture_mode;
    break;
  case modulate_texture:
    mode = modulate_texture;
    break;
  case decal_texture:
    mode = decal_texture;
    break;
  case blend_texture:
    mode = blend_texture;
    break;
  case force_32bit_texture_mode:
    mode = force_32bit_texture_mode;
    break;
  }
  return mode;
}

ON_Texture::FILTER ON_Texture::FilterFromInt( int i )
{
  ON_Texture::FILTER filter = linear_filter;
  switch((unsigned int)i)
  {
  case nearest_filter:
    filter = nearest_filter;
    break;
  case linear_filter:
    filter = linear_filter;
    break;
  case force_32bit_texture_filter:
    filter = force_32bit_texture_filter;
    break;
  }
  return filter;
}

ON_Texture::WRAP ON_Texture::WrapFromInt( int i )
{
  ON_Texture::WRAP wrap = repeat_wrap;

  switch((unsigned int)i)
  {
  case repeat_wrap:
    wrap = repeat_wrap;
    break;
  case clamp_wrap:
    wrap = clamp_wrap;
    break;
  case force_32bit_texture_wrap:
    wrap = force_32bit_texture_wrap;
    break;
  }

  return wrap;
}





// overrides virtual ON_Object::Read
ON_BOOL32 ON_Texture::Read(
        ON_BinaryArchive& binary_archive
      )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (rc)
  {

    if ( 1 != major_version )
    {
      rc = false;
    }
    else
    {
      int i;
      for(;;)
      {
        // 1.0 values
        rc = binary_archive.ReadUuid( m_texture_id );
        if (!rc) break;

        rc = binary_archive.ReadInt( &m_mapping_channel_id );
        if (!rc) break;

        rc = binary_archive.ReadString(m_filename);
        if (!rc) break;

        rc = binary_archive.ReadBool(&m_bOn);
        if (!rc) break;

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_type = ON_Texture::TypeFromInt(i);

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_mode = ON_Texture::ModeFromInt(i);

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_minfilter = ON_Texture::FilterFromInt(i);

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_magfilter = ON_Texture::FilterFromInt(i);

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_wrapu = ON_Texture::WrapFromInt(i);

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_wrapv = ON_Texture::WrapFromInt(i);

        rc = binary_archive.ReadInt(&i);
        if (!rc) break;
        m_wrapw = ON_Texture::WrapFromInt(i);

        rc = binary_archive.ReadXform(m_uvw);
        if (!rc) break;

        rc = binary_archive.ReadColor(m_border_color);
        if (!rc) break;

        rc = binary_archive.ReadColor(m_transparent_color);
        if (!rc) break;

        rc = binary_archive.ReadUuid(m_transparency_texture_id);
        if (!rc) break;

        rc = binary_archive.ReadInterval(m_bump_scale);
        if (!rc) break;

        rc = binary_archive.ReadDouble(&m_blend_constant_A);
        if (!rc) break;
        rc = binary_archive.ReadDouble(4,m_blend_A);
        if (!rc) break;
        rc = binary_archive.ReadColor(m_blend_constant_RGB);
        if (!rc) break;
        rc = binary_archive.ReadDouble(4,m_blend_RGB);
        if (!rc) break;

        rc = binary_archive.ReadInt(&m_blend_order);
        if (!rc) break;



        break;
      }
    }

    if ( !binary_archive.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}


void ON_Texture::Default()
{
  PurgeUserData();
  m_texture_id = ON_nil_uuid;
  m_mapping_channel_id = 0;
  m_filename.Destroy();
  m_filename_bRelativePath = false;
  m_bOn = true;
  m_type = bitmap_texture;
  m_mode = modulate_texture;
  m_minfilter = linear_filter;
  m_magfilter = linear_filter;
  m_wrapu = repeat_wrap;
  m_wrapv = repeat_wrap;
  m_wrapw = repeat_wrap;
  m_uvw.Identity();
  m_border_color = ON_UNSET_COLOR;
  m_transparent_color = ON_UNSET_COLOR;
  m_transparency_texture_id = ON_nil_uuid;
  m_bump_scale.Set(0.0,1.0);
  m_blend_constant_A = 1.0;
  m_blend_A[0] = m_blend_A[1] = 1.0; m_blend_A[2] =  m_blend_A[3] = 0.0;
  m_blend_constant_RGB.SetRGB(0,0,0);
  m_blend_RGB[0] = m_blend_RGB[1] = 1.0; m_blend_RGB[2] = m_blend_RGB[3] = 0.0;
  m_blend_order = 0;
  m_runtime_ptr_id = ON_nil_uuid;
  m_runtime_ptr = 0;
}


ON_OBJECT_IMPLEMENT(ON_TextureMapping,ON_Object,"32EC997A-C3BF-4ae5-AB19-FD572B8AD554");


ON_TextureMapping::TYPE ON_TextureMapping::TypeFromInt(int i)
{
  ON_TextureMapping::TYPE t;
  switch(i)
  {
  case srfp_mapping:
    t = srfp_mapping;
    break;
  case plane_mapping:
    t = plane_mapping;
    break;
  case cylinder_mapping:
    t = cylinder_mapping;
    break;
  case sphere_mapping:
    t = sphere_mapping;
    break;
  case box_mapping:
    t = box_mapping;
    break;
  case mesh_mapping_primitive:
    t = mesh_mapping_primitive;
    break;
  case srf_mapping_primitive:
    t = srf_mapping_primitive;
    break;
  case brep_mapping_primitive:
    t = brep_mapping_primitive;
    break;
  default:
    t = no_mapping;
    break;
  }
  return t;
}

ON_TextureMapping::PROJECTION ON_TextureMapping::ProjectionFromInt(int i)
{
  ON_TextureMapping::PROJECTION p;
  switch(i)
  {
  case clspt_projection:
    p = clspt_projection;
    break;
  case ray_projection:
    p = ray_projection;
    break;
  default:
    p = no_projection;
    break;
  }
  return p;
}

ON_TextureMapping::TEXTURE_SPACE ON_TextureMapping::TextureSpaceFromInt(int i)
{
	ON_TextureMapping::TEXTURE_SPACE ts = single;

	switch(i)
	{
	case single:
		ts = single;
		break;
	case divided:
		ts = divided;
		break;
	}
	return ts;
}

ON_TextureMapping::ON_TextureMapping()
{
  m_mapping_primitive = 0;
  Default();
}

ON_TextureMapping::~ON_TextureMapping()
{
  if ( m_mapping_primitive )
  {
    delete m_mapping_primitive;
    m_mapping_primitive = 0;
  }
}

// The copy constructor and operator= overrides are needed
// to ensure m_mapping_primitive is properly copied.
ON_TextureMapping::ON_TextureMapping(const ON_TextureMapping& src)
                  : ON_Object(src)
{
  m_mapping_id    = src.m_mapping_id;
  m_mapping_index = src.m_mapping_index;
  m_mapping_name  = src.m_mapping_name;
  m_type          = src.m_type;
  m_projection    = src.m_projection;
  m_bCapped		    = src.m_bCapped;
	m_texture_space = src.m_texture_space;
  m_Pxyz          = src.m_Pxyz;
  m_Nxyz          = src.m_Nxyz;
  m_uvw           = src.m_uvw;
  m_mapping_primitive = ( src.m_mapping_primitive )
                      ? src.m_mapping_primitive->Duplicate()
                      : 0;
}

ON_TextureMapping& ON_TextureMapping::operator=(const ON_TextureMapping& src)
{
  if ( this != &src )
  {
    if ( m_mapping_primitive )
    {
      delete m_mapping_primitive;
      m_mapping_primitive = 0;
    }
    ON_Object::operator=(src);
    m_mapping_id    = src.m_mapping_id;
    m_mapping_index = src.m_mapping_index;
    m_mapping_name  = src.m_mapping_name;
    m_type          = src.m_type;
    m_projection    = src.m_projection;
    m_bCapped			  = src.m_bCapped;
		m_texture_space = src.m_texture_space;
    m_Pxyz          = src.m_Pxyz;
    m_Nxyz          = src.m_Nxyz;
    m_uvw           = src.m_uvw;
    if ( src.m_mapping_primitive )
      m_mapping_primitive = src.m_mapping_primitive->Duplicate();
  }
  return *this;
}

void ON_TextureMapping::Default()
{
  PurgeUserData();
  if ( m_mapping_primitive )
  {
    delete m_mapping_primitive;
    m_mapping_primitive = 0;
  }

  m_mapping_id = ON_nil_uuid;
  m_mapping_index = 0;
  m_mapping_name.Destroy();
  m_type = no_mapping;
  m_projection = no_projection;
  m_texture_space = single;
  m_Pxyz.Identity();
  m_Nxyz.Identity();
  m_uvw.Identity();
  m_bCapped = false;
}

ON_BOOL32 ON_TextureMapping::IsValid( ON_TextLog* text_log ) const
{
  if ( m_type != ON_TextureMapping::TypeFromInt(m_type) )
  {
    if ( text_log )
    {
      text_log->Print("ON_TextureMapping m_type = %d is not a valid value.\n",m_type);
    }
    return false;
  }

  if ( m_projection != ON_TextureMapping::ProjectionFromInt(m_projection) )
  {
    if ( text_log )
    {
      text_log->Print("ON_TextureMapping m_projection = %d is not a valid value.\n",m_projection);
    }
    return false;
  }

  if (m_texture_space != ON_TextureMapping::TextureSpaceFromInt(m_texture_space))
  {
	  if (text_log)
	  {
		  text_log->Print("ON_TextureMapping m_texture_space = %d is not a valid value.\n",m_texture_space);
	  }
	  return false;
  }

  return true;
}

void ON_TextureMapping::Dump( ON_TextLog& text_log ) const
{
  text_log.Print("Texture mapping id: "); text_log.Print(m_mapping_id); text_log.Print("\n");
  text_log.PushIndent();

  text_log.Print("type: ");
  switch(m_type)
  {
  case no_mapping:
    text_log.Print("no mapping\n");
    break;
  case plane_mapping:
    text_log.Print("plane mapping\n");
    break;
  case cylinder_mapping:
    text_log.Print("cylinder mapping\n");
    break;
  case sphere_mapping:
    text_log.Print("sphere mapping\n");
    break;
	case box_mapping:
    text_log.Print("box mapping\n");
    break;
  default:
    text_log.Print("%d\n",m_type);
    break;
  }

  text_log.Print("projection: ");
  switch(m_projection)
  {
  case no_projection:
    text_log.Print("no projection\n");
    break;
  case clspt_projection:
    text_log.Print("closest point to mesh vertex\n");
    break;
  case ray_projection:
    text_log.Print("mesh normal ray intersection\n");
    break;
  default:
    text_log.Print("%d\n",m_projection);
    break;
  }

	text_log.Print("texture_space: ");
  switch(m_texture_space)
  {
  case single:
    text_log.Print("single texture space\n");
    break;
  case clspt_projection:
    text_log.Print("divided texture space\n");
    break;
  default:
    text_log.Print("%d\n",m_texture_space);
    break;
  }

  text_log.Print("XYZ point transformation:\n");
  text_log.PushIndent();
  text_log.Print(m_Pxyz);
  text_log.PopIndent();

  text_log.Print("XYZ normal transformation:\n");
  text_log.PushIndent();
  text_log.Print(m_Nxyz);
  text_log.PopIndent();

  text_log.Print("UVW transformation:\n");
  text_log.PushIndent();
  text_log.Print(m_uvw);
  text_log.PopIndent();

  text_log.PopIndent();
}

unsigned int ON_TextureMapping::SizeOf() const
{
  unsigned int sz = ON_Object::SizeOf();
  sz = sizeof(*this) - sizeof(ON_Object);
  return sz;
}

bool ON_TextureMapping::ReverseTextureCoordinate( int dir )
{
  bool rc = false;
  if ( 0 <= dir && dir <= 3 )
  {
    ON_Xform x(1.0);
    x.m_xform[dir][dir] = -1.0;
    x.m_xform[dir][3] = 1.0;
    m_uvw = x*m_uvw;
    rc = true;
  }
  return rc;
}

bool ON_TextureMapping::SwapTextureCoordinate( int i, int j )
{
  bool rc = false;
  if (i!=j && 0 <= i && i <= 3 && 0 <= j && j <= 3)
  {
    ON_Xform x(1.0);
    x.m_xform[i][i] = x.m_xform[j][j] = 0.0;
    x.m_xform[i][j] = x.m_xform[j][i] = 1.0;
    m_uvw = x*m_uvw;
    rc = true;
  }
  return rc;
}

bool ON_TextureMapping::TileTextureCoordinate( int dir, double count, double offset )
{
  bool rc = false;
  if ( 0 <= dir && dir <= 3 && 0.0 != count && ON_IsValid(count) && ON_IsValid(offset) )
  {
    ON_Xform x(1.0);
    x.m_xform[dir][dir] = count;
    x.m_xform[dir][3] = offset;
    m_uvw = x*m_uvw;
    rc = true;
  }
  return rc;
}


bool ON_Texture::ReverseTextureCoordinate( int dir )
{
  bool rc = false;
  if ( 0 <= dir && dir <= 3 )
  {
    ON_Xform x(1.0);
    x.m_xform[dir][dir] = -1.0;
    x.m_xform[dir][3] = 1.0;
    m_uvw = x*m_uvw;
    rc = true;
  }
  return rc;
}

bool ON_Texture::SwapTextureCoordinate( int i, int j )
{
  bool rc = false;
  if (i!=j && 0 <= i && i <= 3 && 0 <= j && j <= 3)
  {
    ON_Xform x(1.0);
    x.m_xform[i][i] = x.m_xform[j][j] = 0.0;
    x.m_xform[i][j] = x.m_xform[j][i] = 1.0;
    m_uvw = x*m_uvw;
    rc = true;
  }
  return rc;
}

bool ON_Texture::TileTextureCoordinate( int dir, double count, double offset )
{
  bool rc = false;
  if ( 0 <= dir && dir <= 3 && 0.0 != count && ON_IsValid(count) && ON_IsValid(offset) )
  {
    ON_Xform x(1.0);
    x.m_xform[dir][dir] = count;
    x.m_xform[dir][3] = offset;
    m_uvw = x*m_uvw;
    rc = true;
  }
  return rc;
}

bool ON_Texture::IsTiled( int dir, double* count, double* offset ) const
{
  if ( count )
    *count = 1.0;
  if ( offset )
    *offset = 0.0;

  if ( 0 <= dir && dir <= 3 )
  {
    int row0=-1, row, col;
    for ( row = 0; row < 3; row++ )
    {
      for ( col = 0; col < 3; col++ )
      {
        if ( col != dir && 0.0 != m_uvw.m_xform[row][col] )
          break;
      }
      if ( 3 == col )
      {
        if ( -1 == row0 )
        {
          row0 = row;
        }
        else
          return false;
      }
    }
    if ( row0 >= 0 )
    {
      if (count)
        *count = m_uvw.m_xform[row0][dir];
      if ( offset )
        *offset = m_uvw.m_xform[row0][3];
      return true;
    }
  }

  return false;
}

static const double on__overflow_tol = 1.0e100;

static
int BestHitHelper(double t0, double t1)
{
  return ((t0 < 0.0 && t1 > t0) || (0.0 <= t1 && t1 < t0)) ? 1 : 0;
}

static
int IntersectBoxRayHelper(const ON_3dPoint& rst, const ON_3dVector& n, int dir, double* s)
{
  /*
  returns:
    0 = ray parallel to sides
    1 = ray hit left side (x=-1)
    2 = ray hit right side (x=+1)
    3 = ray hit back side (y=-1)
    4 = ray hit front side (y=+1)
    5 = ray hit bottom side (z=-1)
    6 = ray hit top side (z=+1)
  */
  double nx = (&n.x)[dir];
  ON_3dPoint Q;
  double t,t0,t1;

  // protect against overflow
  t = fabs(nx)*on__overflow_tol;
  t0 = (-1.0 - (&rst.x)[dir]);
  t1 = ( 1.0 - (&rst.x)[dir]);
  if ( fabs(t0) >= t || fabs(t1) >= t )
  {
    *s = ON_UNSET_VALUE;
    return 0;
  }

  t0 /= nx;
  Q = rst + t0*n;
  if ( dir )
  {
    t = Q.x;
    Q.x = Q[dir];
    Q[dir] = t;
  }
  if ( fabs(Q.x+1.0) > ON_SQRT_EPSILON
        || Q.y < -(1.0+ON_SQRT_EPSILON) || Q.y > (1.0+ON_SQRT_EPSILON)
        || Q.z < -(1.0+ON_SQRT_EPSILON) || Q.z > (1.0+ON_SQRT_EPSILON)
        )
  {
    // The ray's intersection with the plane missed the 
    // (-1,+1)x(-1,+1) square that is the side of the box.
    t0 = ON_UNSET_VALUE;
  }

  t1 /= nx;
  Q = rst + t1*n;
  if ( dir )
  {
    t = Q.x;
    Q.x = Q[dir];
    Q[dir] = t;
  }
  if ( fabs(Q.x-1.0) > ON_SQRT_EPSILON
        || Q.y < -(1.0+ON_SQRT_EPSILON) || Q.y > (1.0+ON_SQRT_EPSILON)
        || Q.z < -(1.0+ON_SQRT_EPSILON) || Q.z > (1.0+ON_SQRT_EPSILON)
        )
  {
    // The ray's intersection with the plane missed the 
    // (-1,+1)x(-1,+1) square that is the side of the box.
    t1 = ON_UNSET_VALUE;
    if ( ON_UNSET_VALUE == t0 )
    {
      *s = ON_UNSET_VALUE;
      return 0;
    }
  }

  int rc;
  if ( ON_UNSET_VALUE == t0 || 1 == BestHitHelper(t0,t1) )
  {
    rc = 2 + 2*dir;
    *s = t1;
  }
  else
  {
    rc = 1 + 2*dir;
    *s = t0;
  }
  return rc;
}


int ON_TextureMapping::EvaluatePlaneMapping( 
  const ON_3dPoint& P,
  const ON_3dVector& N,
  ON_3dPoint* T
  ) const
{
  // The matrix m_Pxyz transforms the world  coordinate
  // "mapping rectangle" into the rectangle
  //   -1 <= r <= 1, -1 <= s <= 1, and  (-1 <= t <= 1)

  ON_3dPoint rst(m_Pxyz*P);

  if ( ray_projection == m_projection )
  {
    ON_3dVector n(m_Nxyz*N);
    if ( fabs(rst.z) < fabs(n.z)*on__overflow_tol )
    {
      double t = -rst.z/n.z;
      rst.x = rst.x + t*n.x;
      rst.y = rst.y + t*n.y;
    }
  }

  // convert -1 <= r <= 1, -1 <= s <= 1
  // to normalized texture coordinate
	rst.x = 0.5*rst.x + 0.5;
	rst.y = 0.5*rst.y + 0.5;

  // Apply texture coordinate transformation 
  *T = m_uvw*rst;

  //See docs - if m_bCapped is false, then planar is truely flat.
  if (!m_bCapped)
	  T->z = 0.0;

  return 1;
}

int ON_TextureMapping::EvaluateSphereMapping( 
											  const ON_3dPoint& P,
											  const ON_3dVector& N,
											  ON_3dPoint* T
											  ) const
{
  // The matrix m_Pxyz transforms the world coordinate
  // "mapping sphere" into the sphere centered at
  // rst = (0,0,0) with radius 1.0.

  ON_3dPoint rst(m_Pxyz*P);
	const double r = ((const ON_3dVector*)(&rst.x))->Length();
	double t0, t1;
	
	if ( ray_projection == m_projection )
	{
		ON_3dVector n(m_Nxyz*N);
		// Shoot a ray from P in the direction N and see if it 
		// hits the sphere.
		int rc = ON_SolveQuadraticEquation( (n.x*n.x+n.y*n.y+n.z*n.z), 
			2.0*(rst.x*n.x+rst.y*n.y+rst.z*n.z), 
			(rst.x*rst.x+rst.y*rst.y+rst.z*rst.z) - 1.0, 
			&t0, &t1 );
		if (rc >= 0 )
		{
			if ( 2 != rc && 1 == BestHitHelper(t0,t1) )
			{
				t0 = t1;
			}
			rst = rst + t0*n;
		}
	}
	
	// convert sphere 3d location to longitude, latitude, radius
	double longitude = (0.0 != rst.y || 0.0 != rst.x) 
		? atan2(rst.y,rst.x) 
		: 0.0;
	double latitude = (0.0 != rst.z) 
		? atan2(rst.z,((const ON_2dVector*)(&rst.x))->Length()) 
		: 0.0;
	if ( latitude > ON_PI )
		latitude -= 2.0*ON_PI;
	
  // convert longitude to normalized texture coordinate
	rst.x = 0.5*longitude/ON_PI;
	if ( rst.x < -ON_EPSILON )
		rst.x += 1.0;
	else if (rst.x < 0.0)
		rst.x = 0.0;
	else if (rst.x > 1.0)
		rst.x = 1.0;

  // convert longitude to normalized texture coordinate
	rst.y = latitude/ON_PI + 0.5;
  if ( rst.y <= 0.0 )
    rst.y = 0.0;
	else if ( rst.y > 1.0 )
		  rst.y = 1.0;
	
  // radius is already normalized
	rst.z = r;
	
  // apply texture coordinate transformation
	*T = m_uvw*rst;

  return 1;
}

int ON_TextureMapping::EvaluateCylinderMapping( 
												const ON_3dPoint& P,
												const ON_3dVector& N,
												ON_3dPoint* T
												) const
{
  // The matrix m_Pxyz transforms the world coordinate
  // "mapping cylinder" into the cylinder centered at
  // rst = (0,0,0) with radius 1.0.  The axis runs
  // from rst = (0,0,-1) to rst = (0,0,+1).

	ON_3dPoint rst(m_Pxyz*P);

	ON_3dPoint Q;
	const double r = ((const ON_2dVector*)(&rst.x))->Length();
	double t, t0, t1;
	int side0, side1;
	PROJECTION mapping_proj = m_projection;
	
	side0 = 0;
	if ( ON_TextureMapping::ray_projection == mapping_proj )
	{
		ON_3dVector n(m_Nxyz*N);
		t = 0.0;
		
		if ( m_bCapped )
		{
			// shoot at caps
			//  The < t check prevents overflow when the 
			//  ray is nearly parallel to the cap.
			t = fabs(n.z)*on__overflow_tol;
			if ( fabs(1.0+rst.z) < t && fabs(1.0-rst.z) < t )
			{
				side0 = 2;
				side1 = 3;

				t0 = (-1.0 - rst.z)/n.z;
				Q = rst + t0*n;
				if ( fabs(1.0+Q.z) > ON_SQRT_EPSILON
					|| (Q.x*Q.x + Q.y*Q.y) > 1.0 + 2.0*ON_SQRT_EPSILON + ON_EPSILON )
				{
          // The ray's intersection with the bottom plane missed the 
          // radius 1 disk that is the bottom of the cylinder.
					side0 = 0;
				}

				t1 = ( 1.0 - rst.z)/n.z;
				Q = rst + t1*n;
				if ( fabs(1.0-Q.z) > ON_SQRT_EPSILON
					|| (Q.x*Q.x + Q.y*Q.y) > 1.0 + 2.0*ON_SQRT_EPSILON + ON_EPSILON )
				{
          // The ray's intersection with the top plane missed the 
          // radius 1 disk that is the top of the cylinder.
					side1 = 0;
				}
				if ( 0 == side0 || 1 == BestHitHelper(t0,t1) )
				{
					side0 = side1;
					t = t1;
				}
				else
				{
					t = t0;
				}
			}
		}
		
		// shoot ray at the cylinder wall
		int rc = ON_SolveQuadraticEquation( (n.x*n.x+n.y*n.y), 
			2.0*(rst.x*n.x+rst.y*n.y), 
			(rst.x*rst.x+rst.y*rst.y) - 1.0, 
			&t0, &t1 );
		if (rc >= 0 )
		{
			if ( 2 != rc  && 1 == BestHitHelper(t0,t1) )
			{
				t0 = t1;
			}
			if ( 0 == side0 )
			{
				// Either the caps are missing or the ray missed the caps.
        // The best hit is the cylinder wall.
				side0 = 1;
				rst = rst + t0*n;
			}
			else if ( 1 != BestHitHelper(t0,t) )
			{
				// The cylinder is capped and the ray hit the cap, 
        // hit the infinite cylinder wall, and the wall 
        // hit is "first".  If the ray hits the finite 
        // cylinder wall, the I will use the wall hit.
				t1 = rst.z + t0*n.z;
				if ( t1 >= -(1.0+ON_SQRT_EPSILON) && t1 <= 1.0+ON_SQRT_EPSILON )
				{
					// use the hit on the cylinder wall
					side0 = 1;
					rst.x = rst.x + t0*n.x;
					rst.y = rst.y + t0*n.y;
          rst.x = t1;
				}
			}
		}
		
		if ( side0 > 1 )
		{
			// best hit is on a cap
			rst = rst + t*n;
		}
	}
	
	if ( m_bCapped && 0 == side0 )
	{
    if ( fabs(rst.z) > 1.0+ON_SQRT_EPSILON )
    {
      if ( fabs(rst.z) > r )
      {
        side0 = (rst.z < 0.0) ? 2 : 3;
      }
    }
    else if ( r <= 1.001 )
    {
      // The point is inside the capped cylinder.
      // Use normal to dermine which surface to use
      // for closest point test.
		  ON_3dVector n(m_Nxyz*N);
      if (  ( fabs(n.z) > fabs(n.x) && fabs(n.z) > fabs(n.y) ) )
      {
        side0 = (n.z < 0.0) ? 2 : 3;
      }
    }
	}
	
	if ( 2 == side0 || 3 == side0 )
	{
    // The cylinder is capped and P maps to 
    // the top (1 == side0) or bottom (2 == side0)

    if ( 2 == side0 )
    {
      // This is the same convention as box mapping.
      // Put another way, if you change the mapping 
      // between box and cylinder, you get the same
      // picture on the top and bottom.
      rst.x = -rst.x; 
    }

		if ( ON_TextureMapping::divided == m_texture_space )
		{
		  if ( r >= 1.0-ON_SQRT_EPSILON )
		  {
			  rst.x /= (r+ON_SQRT_EPSILON);
			  rst.y /= (r+ON_SQRT_EPSILON);
		  }
    }
    else if ( r > 1.0 )
	  {
		  rst.x /= r;
		  rst.y /= r;
	  }

    
    // convert to normalized texture coordinates
		rst.x = 0.5*rst.x + 0.5;
    if ( rst.x < 0.0) rst.x = 0.0; else if (rst.x > 1.0) rst.x = 1.0;
		rst.y = 0.5*rst.y + 0.5;
    if ( rst.y < 0.0) rst.y = 0.0; else if (rst.y > 1.0) rst.y = 1.0;

		if ( ON_TextureMapping::divided == m_texture_space )
		{
      // bottom uses 4/6 <= x <= 5/6 region of the texture map.
      // top uses 5/6 <= x <= 1 region of the texture map.
			rst.x = (2.0 + side0 + rst.x)/6.0;
		} 
	}
	else
	{
    // P maps to side of the cylinder.
    //
    // convert longitude to normalized texture coordinate
		t = (0.0 != rst.y || 0.0 != rst.x) ? atan2(rst.y,rst.x) : 0.0;
		rst.x = 0.5*t/ON_PI;
		if ( rst.x < -ON_EPSILON )
			rst.x += 1.0;
		else if (rst.x < 0.0 )
			rst.x = 0.0;
		else if (rst.x > 1.0 )
			rst.x = 1.0;

    if ( ON_TextureMapping::divided == m_texture_space )
    {
      // side uses 0 <= x <= 2/3 region of the texture map
      rst.x *= 2.0;
			rst.x /= 3.0;
    }

    // convert height to normalized texture coordinate
  	rst.y = 0.5*rst.z + 0.5;
    if ( m_bCapped )
    {
      // clamp height
      if ( rst.y < 0.0 ) rst.y = 0.0; else if ( rst.y > 1.0 ) rst.y = 1.0;
    }
    side0 = 1;
	}
	rst.z = r;	
	
	*T = m_uvw*rst;

  return side0;
}

int ON_TextureMapping::EvaluateBoxMapping( 
										   const ON_3dPoint& P,
										   const ON_3dVector& N,
										   ON_3dPoint* T
										   ) const
{
  // The matrix m_Pxyz transforms the world coordinate
  // "mapping cylinder" into the cylinder centered at
  // rst = (0,0,0) with radius 1.0.  The axis runs
  // from rst = (0,0,-1) to rst = (0,0,+1).

  ON_3dPoint rst(m_Pxyz*P);

	ON_3dVector n(m_Nxyz*N);
  n.Unitize();

	int side0, side1;
	double t0, t1;
	
	side0 = 0;
	t0 = 0.0;

  // side flag
  //  1 =  left side (x=-1)
  //  2 =  right side (x=+1)
  //  3 =  back side (y=-1)
  //  4 =  front side (y=+1)
  //  5 =  bottom side (z=-1)
  //  6 =  top side (z=+1)
	
  if ( ON_TextureMapping::ray_projection == m_projection )
	{
		
		if ( m_bCapped )
		{
			// intersect ray with top and bottom
			side0 = IntersectBoxRayHelper(rst,n,2,&t0);
		}
		// intersect ray with front and back
		side1 = IntersectBoxRayHelper(rst,n,0,&t1);
		if ( 0 == side0 || 1 == BestHitHelper(t0,t1) )
		{
			side0 = side1;
			t0 = t1;
		}
		// intersect ray with left and right
		side1 = IntersectBoxRayHelper(rst,n,1,&t1);
		if ( 0 == side0 || 1 == BestHitHelper(t0,t1) )
		{
			side0 = side1;
			t0 = t1;
		}
		if ( 0 != side0 )
		{
			// ray hit the box
			rst = rst + t0*n;
		}
	} 

  if ( 0 == side0 )
  {
    // set side0 = side closest to the point
    side1 = (fabs(rst.x) >= fabs(rst.y)) ? 0 : 1;
    if ( m_bCapped && fabs(rst.z) > fabs(((double*)&rst.x)[side1]) )
      side1 = 2;
    t1 = (&rst.x)[side1];
    if ( t1 < 0.0 )
    {
      side0 = 2*side1 + 1;
    }
    else 
    {
      side0 = 2*side1 + 2;
    }
    
    //if ( fabs(t1) <= 1.0+ON_SQRT_EPSILON )...
    //// The point is inside the box.  If the normal
    //// is not zero, then use it to choose the side 
    //// used for the closest point projection.

    side1 = ( fabs(n.x) >= fabs(n.y) ) ? 0 : 1;
    if ( m_bCapped && fabs(n.z) > fabs((&n.x)[side1]))
    {
      side1 = 2;
    }
    t1 = n[side1];
    if ( 0.0 != t1 )
    {
      if ( t1 < 0.0 )
        side0 = 2*side1 + 1;
      else if ( t1 > 0.0 )
        side0 = 2*side1 + 2;
    }
  }

	double shift = 0.0;
	
  // side flag
  //  1 =  left side (x=-1)
  //  2 =  right side (x=+1)
  //  3 =  back side (y=-1)
  //  4 =  front side (y=+1)
  //  5 =  bottom side (z=-1)
  //  6 =  top side (z=+1)

	switch(side0)
	{
	case 1: // x = -1 
		rst.x = -rst.y; 
		rst.y =  rst.z; 
		shift =  3.0;
		break;
	case 2: // x = +1
		rst.x =  rst.y;     
		rst.y =  rst.z; 
		shift =  1.0;
		break;
	case 3: // y = -1
		rst.y =  rst.z; 
		shift =  0.0;
		break;
	case 4: // y = +1
		rst.x = -rst.x; 
		rst.y =  rst.z; 
		shift =  2.0;
		break;
	case 5: // z = -1
		rst.x = -rst.x; 
		shift =  4.0;
		break;
	case 6: // z = +1
		shift =  5.0;
		break;
	}

  // normalize texture coordinates
  rst.x = 0.5*rst.x + 0.5;
  rst.y = 0.5*rst.y + 0.5;
	rst.z = 0.0;
	
	if( divided == m_texture_space)
	{
    rst.x = (shift + rst.x)/(m_bCapped ? 6.0 : 4.0);
	}

	*T = m_uvw*rst;
  
  return side0;
}

int ON_TextureMapping::Evaluate(
        const ON_3dPoint& P,
        const ON_3dVector& N,
        ON_3dPoint* T,
        const ON_Xform& P_xform,
        const ON_Xform& N_xform
        ) const
{
  int rc;
  ON_3dPoint Q = P*P_xform;
  if ( ON_TextureMapping::ray_projection == m_projection )
  {
    // need a transformed normal
    ON_3dVector V = N_xform*N;
    V.Unitize();
    rc = Evaluate(Q,V,T);
  }
  else
  {
    // normal is ignored
    rc = Evaluate(Q,N,T);
  }
  return rc;
}

int ON_TextureMapping::Evaluate(
        const ON_3dPoint& P,
        const ON_3dVector& N,
        ON_3dPoint* T
        ) const
{
  int rc;

	switch(m_type)
	{
	case srfp_mapping:
		*T = m_uvw * P; // Do NOT apply m_Pxyz here.
    rc = 1;
		break;
	case sphere_mapping:
		rc = EvaluateSphereMapping(P,N,T);
		break;
	case cylinder_mapping:
		rc = EvaluateCylinderMapping(P,N,T);
		break;
	case box_mapping:
		rc = EvaluateBoxMapping(P,N,T);
		break;

	case mesh_mapping_primitive:
    rc = 0;
		break;

	case srf_mapping_primitive:
    rc = 0;
		break;

	case brep_mapping_primitive:
    rc = 0;
		break;

	default:
		rc = EvaluatePlaneMapping(P,N,T);
		break;
	}	
  return rc;
}

ON__UINT32 ON_TextureMapping::MappingCRC() const
{
  // include any member that can change values returned by Evaluate
  ON__UINT32 crc32 = 0x12345678;
  crc32 = ON_CRC32(crc32,sizeof(m_type),&m_type);
  if ( ON_TextureMapping::srfp_mapping != m_type )
  {
    // As of 21 June 2006 m_Pxyz cannot effect srfp_mapping,
    // so it shouldn't be included in the CRC for srfp_mappings.
    crc32 = ON_CRC32(crc32,sizeof(m_projection),    &m_projection);
    crc32 = ON_CRC32(crc32,sizeof(m_texture_space), &m_texture_space);
    crc32 = ON_CRC32(crc32,sizeof(m_bCapped),		    &m_bCapped);
    crc32 = ON_CRC32(crc32,sizeof(m_Pxyz),          &m_Pxyz);
    // do not include m_Nxyz here - it won't help and may hurt

	  if ( 0 != m_mapping_primitive )
	  {
      switch( m_type )
      {
      case ON_TextureMapping::mesh_mapping_primitive:
        {
          const ON_Mesh* mesh = ON_Mesh::Cast(m_mapping_primitive);
          if ( 0 == mesh )
            break;
          crc32 = mesh->DataCRC(crc32);
          if ( mesh->HasTextureCoordinates() )
          {
            // 25 August 2010 Dale Lear
            //   Including m_T[] in crc32 per Jussi and Andy email discussion.
            //   This is probably correct because users will expect the
            //   "picture" on the mesh to be applied to the target in
            //   a visual way.
            const ON_2fPoint* tex = mesh->m_T.Array();
            crc32 = ON_CRC32(crc32,mesh->m_T.Count()*sizeof(tex[0]),tex);
          }
        }
        break;

      case ON_TextureMapping::brep_mapping_primitive:
        {
          const ON_Brep* brep = ON_Brep::Cast(m_mapping_primitive);
          if ( 0 == brep )
            break;
          crc32 = brep->DataCRC(crc32);
          // 25 August 2010 Dale Lear
          //   Should brep's render meshes be included in the crc?
          //   The texture that is being mapped is actually
          //   being applied to the brep by the render mesh's
          //   m_T[] values and some users will want to see 
          //   the "picture" on the brep mapped to the 
          //   "picture" on the
          //   target.
        }
        break;

      case ON_TextureMapping::srf_mapping_primitive:
        {
          const ON_Surface* surface = ON_Surface::Cast(m_mapping_primitive);
          if ( 0 == surface )
            break;
          crc32 = surface->DataCRC(crc32);
        }
        break;

      case ON_TextureMapping::no_mapping:
      case ON_TextureMapping::srfp_mapping:
      case ON_TextureMapping::plane_mapping:
      case ON_TextureMapping::cylinder_mapping:
      case ON_TextureMapping::sphere_mapping:
      case ON_TextureMapping::box_mapping:
      case ON_TextureMapping::force_32bit_mapping_projection:
      default:
        break;
      }
    }

  }

  crc32 = ON_CRC32(crc32,sizeof(m_uvw), &m_uvw);
  return crc32;
}

bool ON_TextureMapping::RequiresVertexNormals() const
{
  if ( ON_TextureMapping::srfp_mapping == m_type )
    return false;

	if(m_projection == ray_projection) 
    return true;

  if(m_type == box_mapping) 
    return true;
	if(m_type == cylinder_mapping && m_bCapped) 
    return true;

	return false;
}

bool ON_TextureMapping::IsPeriodic(void) const
{
	return (m_type == sphere_mapping || m_type == cylinder_mapping);
}

bool ON_TextureMapping::HasMatchingTextureCoordinates( 
       const ON_Mesh& mesh,
       const ON_Xform* mesh_xform
       ) const
{
  bool rc = (mesh.HasTextureCoordinates())
          ? HasMatchingTextureCoordinates(mesh.m_Ttag,mesh_xform)
          : false;

  return rc;
}

bool ON_TextureMapping::HasMatchingTextureCoordinates( 
       const ON_MappingTag& tag,
       const ON_Xform* mesh_xform
       ) const
{
  bool rc = false;

  // DO NOT COMPARE m_mapping_id's in this function.
  // This function returns true if the tc values
  // calculated by the mapping will be the same
  // as the mapping that was used to set the tag.
  if ( tag.m_mapping_crc == MappingCRC() )
  {
    rc = true;

    // zero transformations indicate the mapping
    // values are independent of the 3d location
    // of the mesh.  The srfp_mapping != m_type
    // check is used because these mappings are
    // alwasy independent of 3d location but
    // the transformations are often set.
    if ( ON_TextureMapping::srfp_mapping != m_type
         && mesh_xform 
         && mesh_xform->IsValid()
         && !mesh_xform->IsZero() 
         && !tag.m_mesh_xform.IsZero()
       )
    {
      // compare xforms - these can have a bit of slop
      const double* a = &mesh_xform->m_xform[0][0];
      const double* b = &tag.m_mesh_xform.m_xform[0][0];
      for ( int i = 16; i--; /*empty*/ )
      {
        if ( fabs(*a++ - *b++) > ON_SQRT_EPSILON )
        {
          rc = false;
          break;
        }
      }
    }
  }

  return rc;
}

static
bool GetSPTCHelper(
  const ON_Mesh& mesh,
  const ON_TextureMapping& mapping,
  float* tc,
  int tc_stride
  )
{
  const int vcnt = mesh.m_V.Count();
  if ( vcnt <= 0 )
    return false;
  if ( !mesh.HasSurfaceParameters() )
    return false;
  const ON_2dPoint* S = mesh.m_S.Array();
  if ( !S )
    return false;

  int i;
  double u, v, a, b;

  // srf_udom and srf_vdom record the range
  // of parameters saved in the m_S[] array.
  ON_Interval srf_udom = mesh.m_srf_domain[0];
  ON_Interval srf_vdom = mesh.m_srf_domain[1];
  if ( !srf_udom.IsIncreasing() || !srf_vdom.IsIncreasing() )
  {
    // Attempt to calculate it from m_S[].
    srf_udom.m_t[0] = srf_udom.m_t[1] = S[0].x;
    srf_vdom.m_t[0] = srf_vdom.m_t[1] = S[0].y;
    for ( i = 1; i < vcnt; i++ )
    {
      u = S[i].x;
      if      (u < srf_udom.m_t[0]) srf_udom.m_t[0] = u; 
      else if (u > srf_udom.m_t[1]) srf_udom.m_t[1] = u; 
      v = S[i].y;
      if      (v < srf_vdom.m_t[0]) srf_vdom.m_t[0] = v; 
      else if (v > srf_vdom.m_t[1]) srf_vdom.m_t[1] = v; 
    }
    if (    !srf_udom.IsIncreasing() 
         || !srf_vdom.IsIncreasing() )
    {
      return false;
    }
  }

  bool bHaveUVWXform =   mapping.m_uvw.IsValid() 
                     && !mapping.m_uvw.IsIdentity() 
                     && !mapping.m_uvw.IsZero();

  if ( mesh.HasPackedTextureRegion() )
  {
    // Packed textures are not compatible with the use 
    // of m_uvw.  m_uvw is ignored in this block
    // of code on purpose.  //SEE BELOW
    const ON_Interval tex_udom = mesh.m_packed_tex_domain[0];
    const ON_Interval tex_vdom = mesh.m_packed_tex_domain[1];
    for ( i = 0; i < vcnt; i++)
    {
		//ALB 2011.01.14
		//Added support for m_uvw in packed textures.  Even though this conceptually makes
		//very little sense, it's one of the most requested features for the texture mapping
		//system, so I grudgingly add it.
		if (bHaveUVWXform)
		{
			const ON_2dPoint si = mapping.m_uvw*S[i];
			u = si.x;
			v = si.y;
		}
		else
		{
			u = S[i].x;
			v = S[i].y;
		}

	    // (u, v) = known surface parameter
	    if ( mesh.m_packed_tex_rotate ) 
	    {
        // verify this by checking with mesher
	       a = 1.0 - srf_vdom.NormalizedParameterAt( v );
	       b = srf_udom.NormalizedParameterAt( u );
	    }
	    else 
	    {
	      a = srf_udom.NormalizedParameterAt( u );
	      b = srf_vdom.NormalizedParameterAt( v );
	    }

      // When textures are packed, tex_udom and tex_vdom
      // are subintervals of (0,1).
	    u = tex_udom.ParameterAt(a);
	    v = tex_vdom.ParameterAt(b);

	    tc[0] = (float)u;
	    tc[1] = (float)v;
      tc += tc_stride;
    }
  }
  else if ( bHaveUVWXform )
  {
    const ON_Xform xform(mapping.m_uvw);
    ON_3dPoint P;
    for ( i = 0; i < vcnt; i++)
    {
	    // normalize surface parameter
      P.x = srf_udom.NormalizedParameterAt( S[i].x );
      P.y = srf_vdom.NormalizedParameterAt( S[i].y );
      P.z = 0.0;

      // apply m_uvw transformation
      P = xform*P;

      tc[0] = (float)P.x;
      tc[1] = (float)P.y;
      tc += tc_stride;
    }
  }
  else
  {
    // m_srf_tex_rotate is ignored on purpose.
    // It only applies if the texture is packed.
    for ( i = 0; i < vcnt; i++)
    {
	    // tc = normalized surface parameter
      a = srf_udom.NormalizedParameterAt( S[i].x );
      b = srf_vdom.NormalizedParameterAt( S[i].y );

	    tc[0] = (float)a;
	    tc[1] = (float)b;
      tc += tc_stride;
    }
  }

  return true;
}


bool ON_TextureMapping::GetTextureCoordinates(
          const ON_Mesh& mesh, 
          ON_SimpleArray<ON_3fPoint>& T,
          const ON_Xform* mesh_xform,
          bool bLazy,
          ON_SimpleArray<int>* Tside
          ) const
{
  if ( Tside )
    Tside->SetCount(0);

  int i;
  const int vcnt = mesh.m_V.Count();
  if ( vcnt <= 0 )
    return false;

  if ( bLazy )
  {
    int tci, tccount = mesh.m_TC.Count();
    for ( tci = 0; tci < tccount; tci++ )
    {
      if ( vcnt == mesh.m_TC[tci].m_T.Count() )
      {
        if ( HasMatchingTextureCoordinates(mesh.m_TC[tci].m_tag,mesh_xform) )
        {
          T = mesh.m_TC[tci].m_T;
          return true;
        }
      }
    }

    if ( HasMatchingTextureCoordinates(mesh,mesh_xform ) )
    {
      T.Reserve(vcnt);
      T.SetCount(vcnt);
      const ON_2fPoint* f = mesh.m_T.Array();
      ON_3fPoint* d = T.Array();
      for ( i = vcnt; i--; f++, d++ )
      {
        d->x = f->x;
        d->y = f->y;
        d->z = 0.0f;
      }
      return true;
    }
  }

	bool rc = false;

  if ( ON_TextureMapping::srfp_mapping == m_type )
  {
    // uv textures from surface parameterization
    T.Reserve(vcnt);
    T.SetCount(vcnt);
    T.Zero();
    rc = GetSPTCHelper(mesh,*this,&T[0].x,3);
	}
  else
  {
    ON_3dPoint  P, tc;
		ON_3dVector N(0.0,0.0,0.0);

		const ON_3fPoint*  mesh_V = mesh.m_V.Array();
		const ON_3fVector* mesh_N = mesh.HasVertexNormals()
                              ? mesh.m_N.Array()
                              : 0;

    T.Reserve(vcnt);
    T.SetCount(vcnt);

    int* Tsd = 0;
    if ( Tside )
    {
      Tside->Reserve(vcnt);
      Tside->SetCount(vcnt);
      Tsd = Tside->Array();
      memset(Tsd,0,vcnt*sizeof(Tsd[0]));
    }

    ON_Xform P_xform(1.0), N_xform(1.0);
    const double* PT = 0;
    const double* NT = 0;
    if ( mesh_xform )
    {
      if ( mesh_xform->IsZero() || mesh_xform->IsIdentity() )
      {
        // ignore transformation
        mesh_xform = 0;
      }
      else if ( 0.0 != mesh_xform->GetMappingXforms(P_xform,N_xform) )
      {
        PT = &P_xform[0][0];
        NT = &N_xform[0][0];
      }
      else
      {
        mesh_xform = 0;
      }
    }

    const float* f;
    double w;
    int sd;

		if (clspt_projection == m_projection && ON_TextureMapping::mesh_mapping_primitive == m_type && NULL != m_mapping_primitive)
		{
      rc = false;
		}
		else if ( mesh_N &&
          (   ray_projection == m_projection 
           || ON_TextureMapping::box_mapping == m_type 
           || ON_TextureMapping::cylinder_mapping == m_type
           || ON_TextureMapping::mesh_mapping_primitive == m_type
		   )
        )
  	{
			// calculation uses mesh vertex normal
      if ( PT && NT )
      {
        // need to transform vertex and normal
        // before calculating texture coordinates
			  for (i = 0; i < vcnt; i++)
			  {
          f = &mesh_V[i].x;
				  w = PT[12]*f[0] + PT[13]*f[1] + PT[14]*f[2] + PT[15];
          w = (0.0 != w) ? 1.0/w : 1.0;
				  P.x = w*(PT[0]*f[0] + PT[1]*f[1] + PT[ 2]*f[2] + PT[ 3]);
				  P.y = w*(PT[4]*f[0] + PT[5]*f[1] + PT[ 6]*f[2] + PT[ 7]);
				  P.z = w*(PT[8]*f[0] + PT[9]*f[1] + PT[10]*f[2] + PT[11]);

          f = &mesh_N[i].x;
          N.x = PT[0]*f[0] + PT[1]*f[1] + PT[ 2]*f[2];
				  N.y = PT[4]*f[0] + PT[5]*f[1] + PT[ 6]*f[2];
				  N.z = PT[8]*f[0] + PT[9]*f[1] + PT[10]*f[2];
          N.Unitize();
				  sd = Evaluate(P,N,&tc);
				  T[i] = tc;
          if ( Tsd ) Tsd[i] = sd;
			  }
      }
			else
      {
        // mesh vertex and normal are ok
			  for (i = 0; i < vcnt; i++)
			  {
				  P = mesh_V[i];
				  N = mesh_N[i];
				  sd = Evaluate(P,N,&tc);
				  T[i] = tc;
          if ( Tsd ) Tsd[i] = sd;
			  }
      }
		}
		else if ( PT )
    {
      // normal is not used
      // mesh vertex needs to be transformed
      for ( i = 0; i < vcnt; i++ )
      {
        f = &mesh_V[i].x;
			  w = PT[12]*f[0] + PT[13]*f[1] + PT[14]*f[2] + PT[15];
        w = (0.0 != w) ? 1.0/w : 1.0;
			  P.x = w*(PT[0]*f[0] + PT[1]*f[1] + PT[ 2]*f[2] + PT[ 3]);
			  P.y = w*(PT[4]*f[0] + PT[5]*f[1] + PT[ 6]*f[2] + PT[ 7]);
			  P.z = w*(PT[8]*f[0] + PT[9]*f[1] + PT[10]*f[2] + PT[11]);
        sd = Evaluate(P,N,&tc);
			  T[i] = tc;
        if ( Tsd )
          Tsd[i] = sd;
		  }
    }
    else
		{
			// normal is not used and mesh vertex is ok
			for ( i = 0; i < vcnt; i++ )
			{
				P = mesh_V[i];
				sd = Evaluate(P,N,&tc);
				T[i] = tc;
				if ( Tsd )
					Tsd[i] = sd;
			}
    }
    rc = true;
	}

	return rc;
}

static 
void ThreeToTwoHelper( 
      const ON_SimpleArray<ON_3fPoint>& T3,
      ON_SimpleArray<ON_2fPoint>& T2
      )
{
  int i = T3.Count();
  const ON_3fPoint* t3 = T3.Array();

  T2.Reserve(i);
  T2.SetCount(i);
  ON_2fPoint* t2 = T2.Array();
  while(i--)
  {
    t2->x = t3->x;
    t2->y = t3->y;
    t2++;
    t3++;
  }
}

bool ON_TextureMapping::GetTextureCoordinates(
            const ON_Mesh& mesh, 
            ON_SimpleArray<ON_2fPoint>& T, 
            const ON_Xform* mesh_xform,
            bool bLazy,
            ON_SimpleArray<int>* Tside
            ) const
{
  bool rc = false;
  if ( Tside )
    Tside->SetCount(0);
  if ( bLazy )
  {
    if ( HasMatchingTextureCoordinates(mesh,mesh_xform ) )
    {
      if ( T.Array() != mesh.m_T.Array() )
      {
        // different arrays - copy
        T = mesh.m_T;
      }
      return true;
    }
    else
    {
      int vcnt = mesh.m_V.Count();
      int tci, tccount = mesh.m_TC.Count();
      for ( tci = 0; tci < tccount; tci++ )
      {
        if ( vcnt == mesh.m_TC[tci].m_T.Count() )
        {
          if ( HasMatchingTextureCoordinates(mesh.m_TC[tci].m_tag,mesh_xform) )
          {
            // copy T3d[] results to T[]
            ThreeToTwoHelper(mesh.m_TC[tci].m_T,T);
            return true;
          }
        }
      }
    }
  }

  if ( ON_TextureMapping::srfp_mapping == m_type )
  {
    // uv textures from surface parameterization
    T.Reserve(mesh.m_V.Count());
    T.SetCount(mesh.m_V.Count());
    T.Zero();
    rc = GetSPTCHelper(mesh,*this,&T[0].x,2);
  }
  else
  {
    T.SetCount(0);
	  ON_SimpleArray<ON_3fPoint> T3;
    if ( GetTextureCoordinates(mesh, T3, mesh_xform, false, Tside ) )
    {
      // copy T3d[] results to T[]
      ThreeToTwoHelper(T3,T);
      rc = true;
	  }
  }
	return rc;
}


//bool ON_Mesh::GetSurfaceParameterTextureXform( 
//          class ON_Xform& StoT 
//          ) const
//
//{
//  bool rc = false;
//  StoT.Identity();
//
//  // Gets default mesh mapping
//  const ON_Interval surface_u_domain(m_srf_domain[0]);
//  const ON_Interval surface_v_domain(m_srf_domain[1]);
//  const ON_Interval texture_u_domain(m_tex_domain[0]);
//  const ON_Interval texture_v_domain(m_tex_domain[1]);
//  bool bRotateTexture = m_srf_tex_rotate;
//  if (   surface_u_domain.IsInterval() 
//      && surface_v_domain.IsInterval()
//      && texture_u_domain.IsInterval()
//      && texture_v_domain.IsInterval()
//      )
//  {
//    double du = 1.0/surface_u_domain.Length();
//    double dv = 1.0/surface_v_domain.Length();
//    ON_Xform x1(1.0), x2(1.0), x3(1.0);
//    x1.m_xform[0][0] = du; x1.m_xform[0][3] = -surface_u_domain[0]*du;
//    x1.m_xform[1][1] = dv; x1.m_xform[1][3] = -surface_v_domain[0]*dv;
//    if ( bRotateTexture )
//    {
//      x2.m_xform[0][0] =  0.0; x2.m_xform[0][1] = -1.0; x2.m_xform[0][3] = 1.0;
//      x2.m_xform[1][0] =  1.0; x2.m_xform[1][1] =  0.0;
//    }
//    x3.m_xform[0][0] = texture_u_domain.Length(); x3.m_xform[0][3] = texture_u_domain[0];
//    x3.m_xform[1][1] = texture_v_domain.Length(); x3.m_xform[1][3] = texture_v_domain[0];
//
//    // transforms surface(u,v) to texture(u,v)
//    StoT = x3*x2*x1;
//
//    rc = true;
//  }
//  return rc;
//}

class ON__CMeshFaceTC
{
  // DO NOT PUT THIS CLASS IN A HEADER FILE
  // IT IS A PRIVATE HELPER CLASS.
public:
  int   fi;
  int   quad[4];
  float Tx[4];
  bool  bSetT[4];
};

class ON__CChangeTextureCoordinateHelper
{
  // DO NOT PUT THIS CLASS IN A HEADER FILE
  // IT IS A PRIVATE HELPER CLASS.
public:
  ON__CChangeTextureCoordinateHelper( ON_Mesh& mesh, int newvcnt, float*& mesh_T );
  ~ON__CChangeTextureCoordinateHelper();

  int DupVertex(int vi);
  void ChangeTextureCoordinate(int* Fvi, int fvi, float x, float y, float* mesh_T, int mesh_T_stride );

  int m_tci;

  ON_Mesh& m_mesh;
  ON_3dPointArray* m_mesh_dV;
  bool m_bHasVertexNormals;
  bool m_bHasVertexTextures;
  bool m_bHasVertexColors;
  bool m_bHasSurfaceParameters;
  bool m_bHasPrincipalCurvatures;
  bool m_bHasHiddenVertices;

  bool m_bHasCachedTextures;  
  ON_SimpleArray< ON_TextureCoordinates* > m_TC;

  // m_vuse[] is an array of length = original number of 
  // vertices in m_mesh and m_vuse[vi] = number of faces 
  // that reference vertex vi. If this vertex needs to be
  // split, vuse[vi] is decremented.  The ultimate goal
  // is to split a few times as needed so we don't
  // bloat the mesh with repeated calls to changing
  // texture maps. m_vuse[] is set the first time
  // DupVertex() is called.
  int m_vuse_count;
  ON_SimpleArray< unsigned int > m_vuse;
private:
  // no implementation
  ON__CChangeTextureCoordinateHelper(const ON__CChangeTextureCoordinateHelper&);
  ON__CChangeTextureCoordinateHelper& operator=(const ON__CChangeTextureCoordinateHelper&);
};

void ON__CChangeTextureCoordinateHelper::ChangeTextureCoordinate(int* Fvi, int fvi, float x, float y,
                                                             float* mesh_T, int mesh_T_stride )
{
  int oldvi = Fvi[fvi];
  float* T = mesh_T+(oldvi*mesh_T_stride);
  if ( x != T[0] || (y != ON_UNSET_FLOAT && y != T[1]) )
  {
    int newvi = DupVertex(oldvi);
    T = mesh_T + (newvi*mesh_T_stride);
    T[0] = x;
    if ( y != ON_UNSET_FLOAT )
     T[1] = y;

    if ( 2 == fvi && oldvi == Fvi[3] )
    {
      Fvi[2] = newvi;
      Fvi[3] = newvi;
    }
    else
    {
      Fvi[fvi] = newvi;
    }
  }
}


ON__CChangeTextureCoordinateHelper::ON__CChangeTextureCoordinateHelper( 
    ON_Mesh& mesh,
    int newvcnt,
    float*& mesh_T ) 
: m_mesh(mesh)
, m_mesh_dV(0)
, m_vuse_count(0)
{
  // adding vertices invalidates this cached information.
  m_mesh.DestroyTopology();
  m_mesh.DestroyPartition();
  m_mesh.DestroyTree();

  m_tci = -1;

  const int vcnt = m_mesh.m_V.Count();

  // It is critical to reserve enough room in the arrays
  // before duplication starts.  Otherwise, during duplication,
  // a dyanamic array can be reallocated, which will make
  // saved array base pointers will be invalid, and you crash
  // the next time they are used.

  m_mesh.m_V.Reserve(vcnt+newvcnt);

  if (    m_mesh.HasDoublePrecisionVertices() 
       && m_mesh.DoublePrecisionVerticesAreValid() 
     )
  {
    m_mesh_dV = &m_mesh.DoublePrecisionVertices();
    m_mesh_dV->Reserve(vcnt+newvcnt);
  }
  else
  {
    m_mesh.DestroyDoublePrecisionVertices();
  }

  m_bHasVertexNormals = m_mesh.HasVertexNormals();
  if ( m_bHasVertexNormals ) 
    m_mesh.m_N.Reserve(vcnt+newvcnt);

  m_bHasVertexTextures = m_mesh.HasTextureCoordinates();
  if ( m_bHasVertexTextures )
  {
    float* p = (float*)m_mesh.m_T.Array();
    m_mesh.m_T.Reserve(vcnt+newvcnt);
    if ( p == mesh_T )
      mesh_T = (float*)m_mesh.m_T.Array();
  }

  m_bHasVertexColors = m_mesh.HasVertexColors();
  if ( m_bHasVertexColors )
    m_mesh.m_C.Reserve(vcnt+newvcnt);

  m_bHasSurfaceParameters = m_mesh.HasSurfaceParameters();
  if ( m_bHasSurfaceParameters )
    m_mesh.m_S.Reserve(vcnt+newvcnt);

  m_bHasPrincipalCurvatures = m_mesh.HasPrincipalCurvatures();
  if ( m_bHasPrincipalCurvatures )
    m_mesh.m_K.Reserve(vcnt+newvcnt);

  m_bHasHiddenVertices = (0 != m_mesh.HiddenVertexArray());
  if ( m_bHasHiddenVertices )
    m_mesh.m_H.Reserve(vcnt+newvcnt);

  // Set m_TC[] to be the subset of m_mesh.m_TC[] that is
  // valid for duplication.
  m_bHasCachedTextures = false;
  int tci, tccount = m_mesh.m_TC.Count();
  m_TC.Reserve(tccount);
  for ( tci = 0; tci < tccount; tci++ )
  {
    ON_TextureCoordinates& tc = m_mesh.m_TC[tci];
    if ( vcnt == tc.m_T.Count() )
    {
      m_bHasCachedTextures = true;
      float* p = (float*)tc.m_T.Array();
      tc.m_T.Reserve(vcnt+newvcnt);
      if ( p == mesh_T )
        mesh_T = (float*)tc.m_T.Array();
      m_TC.Append( &tc );
    }
  }
}


ON__CChangeTextureCoordinateHelper::~ON__CChangeTextureCoordinateHelper()
{
  if ( 0 != m_mesh_dV )
  {
    m_mesh.SetDoublePrecisionVerticesAsValid();
    m_mesh.SetSinglePrecisionVerticesAsValid();
    m_mesh_dV = 0;
  }
}

int ON__CChangeTextureCoordinateHelper::DupVertex(int vi)
{
  if ( 0 == m_vuse_count )
  {
    // m_vuse[] is an array of length = original number of 
    // vertices in m_mesh and m_vuse[vi] = number of faces 
    // that reference vertex vi. If this vertex needs to be
    // split, vuse[vi] is decremented.  The ultimate goal
    // is to split a few times as needed so we don't
    // bloat the mesh with repeated calls to changing
    // texture maps. m_vuse[] is set the first time
    // DupVertex() is called.
    m_vuse_count = m_mesh.m_V.Count();
    m_vuse.Reserve(m_vuse_count);
    m_vuse.SetCount(m_vuse_count);
    m_vuse.Zero();
    for ( int fi = 0; fi < m_mesh.m_F.Count(); fi++ )
    {
      const int* Fvi = m_mesh.m_F[fi].vi;
      int i = Fvi[0];
      if ( i >= 0 && i < m_vuse_count )
        m_vuse[i]++;
      i = Fvi[1];
      if ( i >= 0 && i < m_vuse_count )
        m_vuse[i]++;
      i = Fvi[2];
      if ( i >= 0 && i < m_vuse_count )
        m_vuse[i]++;
      i = Fvi[3];
      if ( Fvi[2] != i && i >= 0 && i < m_vuse_count )
        m_vuse[i]++;
    }
  }

  if ( vi >= 0 && vi < m_vuse_count )
  {
    if ( m_vuse[vi] <= 1 )
      return vi; // only one face uses this vertex - no need to dup the vertex

    // otherwise we will duplicate this vertex, reducing its use count by 1.
    m_vuse[vi]--;
  }


  m_mesh.m_V.AppendNew();
  *m_mesh.m_V.Last() = m_mesh.m_V[vi];
  if ( 0 != m_mesh_dV )
  {
    m_mesh_dV->AppendNew();
    *(m_mesh_dV->Last()) = m_mesh_dV->operator[](vi);
  }
  if ( m_bHasVertexTextures )
  {
    m_mesh.m_T.AppendNew();
    *m_mesh.m_T.Last() = m_mesh.m_T[vi];
  }
  if ( m_bHasVertexNormals )
  {
    m_mesh.m_N.AppendNew();
    *m_mesh.m_N.Last() = m_mesh.m_N[vi];
  }
  if ( m_bHasVertexColors )
  {
    m_mesh.m_C.AppendNew();
    *m_mesh.m_C.Last() = m_mesh.m_C[vi];
  }
  if ( m_bHasSurfaceParameters )
  {
    m_mesh.m_S.AppendNew();
    *m_mesh.m_S.Last() = m_mesh.m_S[vi];
  }
  if ( m_bHasPrincipalCurvatures )
  {
    m_mesh.m_K.AppendNew();
    *m_mesh.m_K.Last() = m_mesh.m_K[vi];
  }
  if ( m_bHasHiddenVertices )
  {
    m_mesh.m_H.AppendNew();
    if ( 0 != (*m_mesh.m_H.Last() = m_mesh.m_H[vi]) )
      m_mesh.m_hidden_count++;
  }

  if ( m_bHasCachedTextures )
  {
    // Note:  This m_TC[] is the subset of m_mesh.m_TC[]
    //        that need to be duped.  The constructor
    //        insures that m_TC[i] is not NULL and
    //        has the right count and capacity.
    //
    //        DO NOT REFERENCE m_mesh.m_TC[] in this block.
    int tccount = m_TC.Count();
    for ( int i = 0; i < tccount; i++ )
    {
      ON_SimpleArray<ON_3fPoint>& T = m_TC[i]->m_T;
      T.AppendNew();
      *T.Last() = T[vi];
    }
  }

  return m_mesh.m_V.Count()-1;
}


static
float PoleFix( float t0,  float t1 )
{
  float t = ( ON_UNSET_FLOAT == t0 )
          ? t1
          : ((ON_UNSET_FLOAT == t1 ) ? t0 : (0.5f*(t0+t1)));
  return t;
}

static
int IntersectBoxSideRayHelper(int side, const ON_3dPoint& rst, const ON_3dVector& n, double* s)
{
  /*
  returns:
    0 = ray parallel to sides
    1 = ray hit left side (x=-1)
    2 = ray hit right side (x=+1)
    3 = ray hit back side (y=-1)
    4 = ray hit front side (y=+1)
    5 = ray hit bottom side (z=-1)
    6 = ray hit top side (z=+1)
  */
  double nx;
  ON_3dPoint Q;
  double t,t0,t1;
  int dir;


  switch(side)
  {
  case 1: // =  left side (x=-1)
    t1 = -1.0;
    dir = 0;
    break;
  case 2: //   right side (x=+1)
    t1 = 1.0;
    dir = 0;
    break;
  case 3: //   back side (y=-1)
    t1 = -1.0;
    dir = 1;
    break;
  case 4: //   front side (y=+1)
    t1 = 1.0;
    dir = 1;
    break;
  case 5: //   bottom side (z=-1)
    t1 = -1.0;
    dir = 2;
    break;
  case 6: //   top side (z=+1)
    t1 = 1.0;
    dir = 2;
    break;
  default:
    *s = ON_UNSET_VALUE;
    return 0;
    break;
  }

  // protect against overflow
  nx = (&n.x)[dir];
  t0 = (t1 - (&rst.x)[dir]);
  if ( fabs(t0) >= fabs(nx)*on__overflow_tol )
  {
    *s = ON_UNSET_VALUE;
    return 0;
  }

  t0 /= nx;
  Q = rst + t0*n;
  if ( dir )
  {
    t = Q.x;
    Q.x = Q[dir];
    Q[dir] = t;
  }
  if ( fabs(Q.x-t1) > ON_SQRT_EPSILON || fabs(Q.y) > 1.0e8 || fabs(Q.z) > 1.0e8 )
  {
    *s = ON_UNSET_VALUE;
    return 0;
  }


  *s = t0;
  return side;
}

static
bool EvBoxSideTextureCoordinateHelper2( 
                       int side,
                       const ON_TextureMapping& box_mapping,
										   const ON_3dPoint& P,
										   const ON_3dVector& N,
										   ON_3dPoint* T
										   )
{
  // side flag
  //  1 =  left side (x=-1)
  //  2 =  right side (x=+1)
  //  3 =  back side (y=-1)
  //  4 =  front side (y=+1)
  //  5 =  bottom side (z=-1)
  //  6 =  top side (z=+1)
  // The matrix m_Pxyz transforms the world coordinate
  // "mapping cylinder" into the cylinder centered at
  // rst = (0,0,0) with radius 1.0.  The axis runs
  // from rst = (0,0,-1) to rst = (0,0,+1).

  ON_3dPoint rst(box_mapping.m_Pxyz*P);

	ON_3dVector n(box_mapping.m_Nxyz*N);
  n.Unitize();

  // side flag
  //  1 =  left side (x=-1)
  //  2 =  right side (x=+1)
  //  3 =  back side (y=-1)
  //  4 =  front side (y=+1)
  //  5 =  bottom side (z=-1)
  //  6 =  top side (z=+1)
	
  if ( ON_TextureMapping::ray_projection == box_mapping.m_projection )
	{
    double s;
    if ( side == IntersectBoxSideRayHelper(side, rst, n, &s) )
    {
		  // ray hit the box side
		  rst = rst + s*n;
    }
	} 

	double shift = 0.0;
	
  // side flag
  //  1 =  left side (x=-1)
  //  2 =  right side (x=+1)
  //  3 =  back side (y=-1)
  //  4 =  front side (y=+1)
  //  5 =  bottom side (z=-1)
  //  6 =  top side (z=+1)

	switch(side)
	{
	case 1: // x = -1 
		rst.x = -rst.y; 
		rst.y =  rst.z; 
		shift =  3.0;
		break;
	case 2: // x = +1
		rst.x =  rst.y;     
		rst.y =  rst.z; 
		shift =  1.0;
		break;
	case 3: // y = -1
		rst.y =  rst.z; 
		shift =  0.0;
		break;
	case 4: // y = +1
		rst.x = -rst.x; 
		rst.y =  rst.z; 
		shift =  2.0;
		break;
	case 5: // z = -1
		rst.x = -rst.x; 
		shift =  4.0;
		break;
	case 6: // z = +1
		shift =  5.0;
		break;
  default:
    return 0;
    break;
	}

  // normalize texture coordinates
  rst.x = 0.5*rst.x + 0.5;
  rst.y = 0.5*rst.y + 0.5;
	rst.z = 0.0;
	
	if( ON_TextureMapping::divided == box_mapping.m_texture_space)
	{
    rst.x = (shift + rst.x)/(box_mapping.m_bCapped ? 6.0 : 4.0);
	}

	*T = box_mapping.m_uvw*rst;
  
  return true;
}

static
bool EvBoxSideTextureCoordinateHelper1(
          const ON_Mesh& mesh, 
          const ON_Xform* mesh_xform,
          int vi,
          int side,
          const ON_TextureMapping& box_mapping,
          float* Tx,
          float* Ty
          )
{
	bool rc = false;
  ON_3dPoint  P, tc;
	ON_3dVector N(0.0,0.0,0.0);

	const ON_3fPoint*  mesh_V = mesh.m_V.Array();
	const ON_3fVector* mesh_N = mesh.HasVertexNormals()
                            ? mesh.m_N.Array()
                            : 0;

  ON_Xform P_xform(1.0), N_xform(1.0);
  const double* PT = 0;
  const double* NT = 0;
  if ( mesh_xform )
  {
    if ( mesh_xform->IsZero() || mesh_xform->IsIdentity() )
    {
      // ignore transformation
      mesh_xform = 0;
    }
    else if ( 0.0 != mesh_xform->GetMappingXforms(P_xform,N_xform) )
    {
      PT = &P_xform[0][0];
      NT = &N_xform[0][0];
    }
    else
    {
      mesh_xform = 0;
    }
  }

  const float* f;
  double w;

  if ( mesh_N && ON_TextureMapping::ray_projection == box_mapping.m_projection )
	{
		// calculation uses mesh vertex normal
    if ( PT && NT )
    {
      // need to transform vertex and normal
      // before calculating texture coordinates
      f = &mesh_V[vi].x;
		  w = PT[12]*f[0] + PT[13]*f[1] + PT[14]*f[2] + PT[15];
      w = (0.0 != w) ? 1.0/w : 1.0;
		  P.x = w*(PT[0]*f[0] + PT[1]*f[1] + PT[ 2]*f[2] + PT[ 3]);
		  P.y = w*(PT[4]*f[0] + PT[5]*f[1] + PT[ 6]*f[2] + PT[ 7]);
		  P.z = w*(PT[8]*f[0] + PT[9]*f[1] + PT[10]*f[2] + PT[11]);

      f = &mesh_N[vi].x;
      N.x = PT[0]*f[0] + PT[1]*f[1] + PT[ 2]*f[2];
		  N.y = PT[4]*f[0] + PT[5]*f[1] + PT[ 6]*f[2];
		  N.z = PT[8]*f[0] + PT[9]*f[1] + PT[10]*f[2];
      N.Unitize();
    }
    else
    {
      // mesh vertex and normal are ok
		  P = mesh_V[vi];
		  N = mesh_N[vi];
    }
	}
	else if ( PT )
  {
    // normal is not used
    // mesh vertex needs to be transformed
    f = &mesh_V[vi].x;
	  w = PT[12]*f[0] + PT[13]*f[1] + PT[14]*f[2] + PT[15];
    w = (0.0 != w) ? 1.0/w : 1.0;
	  P.x = w*(PT[0]*f[0] + PT[1]*f[1] + PT[ 2]*f[2] + PT[ 3]);
	  P.y = w*(PT[4]*f[0] + PT[5]*f[1] + PT[ 6]*f[2] + PT[ 7]);
	  P.z = w*(PT[8]*f[0] + PT[9]*f[1] + PT[10]*f[2] + PT[11]);
  }
  else
  {
    // normal is not used and mesh vertex is ok
    P = mesh_V[vi];
  }


  rc = EvBoxSideTextureCoordinateHelper2(side,box_mapping,P,N,&tc);
  if (rc)
  {
    rc = tc.IsValid();
    if (rc)
    {
      *Tx = (float)tc.x;
      *Ty = (float)tc.y;
    }
  }
	return rc;
}


class ON__CNewMeshFace
{
public:
  int fi;
  int newvcnt;
  bool bNewV[4];
  ON_2fPoint tc[4];
};

static
float TcDistanceHelper(const ON_2fPoint& tc)
{
  float dx = (tc.x > 0.5f) ? (1.0f-tc.x) : tc.x;
  if ( dx < 0.0f) 
    return 0.0f;
  float dy = (tc.y > 0.5f) ? (1.0f-tc.y) : tc.y;
  if ( dy < 0.0f) 
    return 0.0f;
  return (dx < dy) ? dx : dy;
}

static
void AdjustSingleBoxTextureCoordinatesHelper( 
          ON_Mesh& mesh, 
          const ON_Xform* mesh_xform,
          float* mesh_T,
          int    mesh_T_stride,
          const int* Tsd,
          const ON_TextureMapping& box_mapping
          )
{
  const int vcnt = mesh.m_V.Count();
  const int fcnt = mesh.m_F.Count();
  if ( vcnt < 3 || fcnt < 1 || vcnt != mesh.m_T.Count() || !Tsd )
    return;
  const ON_MeshFace* mesh_F = mesh.m_F.Array();
  const int* Fvi;
  int j, k, fi, sd[4], fvicnt, side, newvcnt=0;
  ON__CNewMeshFace mf;
  ON_2fPoint tc;
  ON_SimpleArray<ON__CNewMeshFace> mflist(512);
  float d;
  for ( fi = 0; fi < fcnt; fi++ )
  {
    Fvi = mesh_F[fi].vi;
    sd[0] = Tsd[Fvi[0]];
    sd[1] = Tsd[Fvi[1]];
    sd[2] = Tsd[Fvi[2]];
    sd[3] = Tsd[Fvi[3]];
    if ( sd[0] == sd[1] && sd[0] == sd[2] && sd[0] == sd[3] )
    {
      // all texture coords are on same side of box
      continue;
    }
    fvicnt = (Fvi[2] != Fvi[3]) ? 4 : 3;

    memset(&mf,0,sizeof(mf));
    mf.tc[0] = mesh_T + (Fvi[0]*mesh_T_stride);
    mf.tc[1] = mesh_T + (Fvi[1]*mesh_T_stride);
    mf.tc[2] = mesh_T + (Fvi[2]*mesh_T_stride);
    mf.tc[3] = mesh_T + (Fvi[3]*mesh_T_stride);

    // find the side we will use for this face
    side = sd[0];
    d = TcDistanceHelper(mf.tc[0]);
    for ( j = 1; j < fvicnt; j++ )
    {
      float d1 = TcDistanceHelper(mf.tc[j]);
      if (d1 > d)
      {
        side = sd[j];
        d = d1;
      }
    }

    // Jussi, 5th September 2011:
    // This 'continue' only works for faces having one or more of its tc's in (0,1)x(0,1).
    // I have commented it out as a fix to RR 90329.
    //if ( d <= 0.0f )
    //  continue;

    for ( j = 0; j < fvicnt; j++ )
    {
      if ( sd[j] != side )
      {
        // calculate new tc for this side
        if ( EvBoxSideTextureCoordinateHelper1(
          mesh,
          mesh_xform,
          Fvi[j],
          side,
          box_mapping,
          &tc.x,&tc.y) )
        {
          if ( tc.x != mf.tc[j].x || tc.y != mf.tc[j].y )
          {
            mf.tc[j] = tc;
            mf.bNewV[j] = true;
            mf.newvcnt++;
          }
        }
        else
          break;
      }
    }
    if ( j >= fvicnt && mf.newvcnt > 0 )
    {
      mf.fi = fi;
      newvcnt += mf.newvcnt;
      mflist.Append(mf);
    }
  }

  if ( newvcnt <= 0 )
    return;

  ON__CChangeTextureCoordinateHelper helper(mesh,vcnt+newvcnt,mesh_T);
  
  const int mflist_count = mflist.Count();

  for ( k = 0; k < mflist_count; k++ )
  {
    mf = mflist[k];
    int* fvi = mesh.m_F[mf.fi].vi;
    fvicnt = (fvi[2]!=fvi[3]) ? 4 : 3;
    for ( j = 0; j < fvicnt; j++ )
    {
      if ( mf.bNewV[j] )
      {
        helper.ChangeTextureCoordinate(fvi,j,mf.tc[j].x,mf.tc[j].y,mesh_T,mesh_T_stride);
      }
    }    
  }
}

static 
void AdjustMeshPeriodicTextureCoordinatesHelper( 
          ON_Mesh& mesh, 
          const ON_Xform* mesh_xform,
          float* mesh_T,
          int    mesh_T_stride,
          const int* Tsd,
          double two_pi_tc,
          const ON_TextureMapping& mapping
          )
{
  // This helper adjusts texture coordinates on faces that
  // span the seam on mapping spheres and cylinders and
  // resolves the mulitiple valued problem that
  // exists at the poles of sphere mappings.

  const int vcnt = mesh.m_V.Count();
  const int fcnt = mesh.m_F.Count();
  if ( vcnt < 3 || fcnt < 1 || vcnt != mesh.m_T.Count() )
    return;

  // see if any texture coordinate adjustment is necessary
  const ON_TextureMapping::TYPE mapping_type = mapping.m_type;
  const bool bSphereCheck = ( ON_TextureMapping::sphere_mapping == mapping_type );
  const bool bCylinderCheck = (Tsd && ON_TextureMapping::cylinder_mapping == mapping_type);
  const bool bBoxCheck = (Tsd && ON_TextureMapping::box_mapping == mapping_type);

  if ( bBoxCheck && ON_TextureMapping::single == mapping.m_texture_space )
  {
    AdjustSingleBoxTextureCoordinatesHelper( mesh, mesh_xform, mesh_T, mesh_T_stride, Tsd, mapping );
    return;
  }

  ON_Workspace ws;
  int* quad = ws.GetIntMemory(vcnt); // ~ws will free quad memory
  float* Tx = (float*)ws.GetMemory(vcnt*sizeof(Tx[0]));
  float t;
  int vi, ti, q=0;
  int ftc_count = 0;

  const float ang0 = (float)(0.25*two_pi_tc);
  const float ang1 = (float)(0.75*two_pi_tc);


  for ( vi = ti = 0; vi < vcnt; vi++, ti += mesh_T_stride )
  {
    quad[vi] = 0;
    Tx[vi] = mesh_T[ti];
    if ( bCylinderCheck )
    {
      if ( 1 != Tsd[vi] )
        continue;
    }
    else if ( bBoxCheck )
    {
      if ( 1 != Tsd[vi] && 3 != Tsd[vi] )
        continue;
    }
    else if ( bSphereCheck )
    {
      t = mesh_T[ti+1]; // t = "v" texture coordinate
      if ( t < 0.001f )
      {
        quad[vi] = 8; q |= 8; // south pole point
        ftc_count++;
        continue;
      }
      if ( t > 0.999f )
      {
        quad[vi] = 8; q |= 8; // north pole point
        ftc_count++;
        continue;
      }
    }

    t = Tx[vi]; // t = "u" texture coordinate
    if ( t < ang0 )
    {      
      quad[vi] = 1; q |= 1; // longitude < pi/2
      ftc_count++;
    }
    else if ( t > ang1 )
    {
      quad[vi] = 4; q |= 4; // longitude > 3pi/2
      ftc_count++;
    }
  }

  if ( 0 == q || 1 == q || 4 == q )
  {
    // nothing needs to be adjusted
    return;
  }

  // 4*ftc_count = (over) estimate of the number of faces that
  // will be changed.
  ON_SimpleArray<ON__CMeshFaceTC> ftc_list(ftc_count*4 + 128);
  ftc_count = 0;
  const ON_MeshFace* F = mesh.m_F.Array();
  const int* Fvi;
  int fi;
  ON__CMeshFaceTC ftc;
  memset(&ftc,0,sizeof(ftc));
  float t0, t1;

  for ( fi = 0; fi < fcnt; fi++ )
  {
    Fvi = F[fi].vi;

    ftc.quad[0] = quad[Fvi[0]];
    ftc.quad[1] = quad[Fvi[1]];
    ftc.quad[2] = quad[Fvi[2]];
    ftc.quad[3] = quad[Fvi[3]];

    q = (ftc.quad[0] | ftc.quad[1] | ftc.quad[2] | ftc.quad[3]);
    if ( 0 == q || 1 == q || 4 == q )
    {
      // no adjustments need to be made
      continue;
    }

    // ftc.fi will be set to fi if a texture coordinate needs to be adjusted
    ftc.fi = -1; 

    ftc.Tx[0] = Tx[Fvi[0]];
    ftc.Tx[1] = Tx[Fvi[1]];
    ftc.Tx[2] = Tx[Fvi[2]];
    ftc.Tx[3] = Tx[Fvi[3]];

    if ( 0 != (8&q) )
    {
      // see if check for north/south sphere mapping poles and fix them
      if ( 8 == ftc.quad[0] ) 
      {
        t0 = (8 == ftc.quad[3]) ? ON_UNSET_FLOAT : ftc.Tx[3];
        t1 = (8 == ftc.quad[1]) ? ON_UNSET_FLOAT : ftc.Tx[1];
        if ( ON_UNSET_FLOAT != t0 || ON_UNSET_FLOAT != t1 )
        {
          ftc.Tx[0] = PoleFix(t0,t1);
          ftc.quad[0] = ((ftc.Tx[0] < ang0) ? 1 : ((ftc.Tx[0] > ang1) ? 4 : 0));
          q |= ftc.quad[0];
          ftc.fi = fi;
        }
      }
      if ( 8 == ftc.quad[1] ) 
      {
        t0 = (8 == ftc.quad[0]) ? ON_UNSET_FLOAT : ftc.Tx[0];
        t1 = (8 == ftc.quad[2]) ? ON_UNSET_FLOAT : ftc.Tx[2];
        if ( ON_UNSET_FLOAT != t0 || ON_UNSET_FLOAT != t1 )
        {
          ftc.Tx[1] = PoleFix(t0,t1);
          ftc.quad[1] = ((ftc.Tx[1] < ang0) ? 1 : ((ftc.Tx[1] > ang1) ? 4 : 0));
          q |= ftc.quad[1];
          ftc.fi = fi;
        }
      }
      if ( 8 == ftc.quad[2] ) 
      {
        int k = (Fvi[2] == Fvi[3]) ? 0 : 3;
        t0 = (8 == ftc.quad[1]) ? ON_UNSET_FLOAT : ftc.Tx[1];
        t1 = (8 == ftc.quad[k]) ? ON_UNSET_FLOAT : ftc.Tx[k];
        if ( ON_UNSET_FLOAT != t0 || ON_UNSET_FLOAT != t1 )
        {
          ftc.Tx[2] = PoleFix(t0,t1);
          ftc.quad[2] = ((ftc.Tx[2] < ang0) ? 1 : ((ftc.Tx[2] > ang1) ? 4 : 0));
          if ( !k )
          {
            ftc.Tx[3] = ftc.Tx[2];
            ftc.quad[3] = ftc.quad[2];
          }
          q |= ftc.quad[2];
          ftc.fi = fi;
        }
      }
      if ( 8 == ftc.quad[3] && Fvi[2] != Fvi[3] ) 
      {
        t0 = (8 == ftc.quad[2]) ? ON_UNSET_FLOAT : ftc.Tx[2];
        t1 = (8 == ftc.quad[0]) ? ON_UNSET_FLOAT : ftc.Tx[0];
        if ( ON_UNSET_FLOAT != t0 || ON_UNSET_FLOAT != t1 )
        {
          ftc.Tx[3] = PoleFix(t0,t1);
          ftc.quad[3] = ((ftc.Tx[3] < ang0) ? 1 : ((ftc.Tx[3] > ang1) ? 4 : 0));
          q |= ftc.quad[3];
          ftc.fi = fi;
        }
      }
    }

    if ( 5 == (5&q) )
    {
      // The face has corners on both sides of the seam
      if ( two_pi_tc == 1.0 )
      {
        if ( 1 == ftc.quad[0] ) {ftc.Tx[0] += 1.0f; ftc.fi = fi;}
        if ( 1 == ftc.quad[1] ) {ftc.Tx[1] += 1.0f; ftc.fi = fi;}
        if ( 1 == ftc.quad[2] ) {ftc.Tx[2] += 1.0f; ftc.fi = fi;}
        if ( 1 == ftc.quad[3] ) {ftc.Tx[3] += 1.0f; ftc.fi = fi;}
      }
      else
      {
        // With divided textures, wrapping the texture coordinate
        // does not work because it wraps into a region of the
        // texture not use by this "side".  In this case, the
        // only thing to do is to pick the best end of the texture
        // map and clamp the tcs that hang over.  If the mesh
        // has edges near the texture seam, the picture will
        // still look ok.
        float f0=0.0f, f1=0.0f, twopitc = (float)two_pi_tc;;
        //int f0cnt=0, f1cnt=0;
        if ( 1 == ftc.quad[0] ) f0 += ftc.Tx[0]; else if ( 4 == ftc.quad[0] ) f1 += twopitc-ftc.Tx[0];
        if ( 1 == ftc.quad[1] ) f0 += ftc.Tx[1]; else if ( 4 == ftc.quad[1] ) f1 += twopitc-ftc.Tx[1];
        if ( 1 == ftc.quad[2] ) f0 += ftc.Tx[2]; else if ( 4 == ftc.quad[2] ) f1 += twopitc-ftc.Tx[2];
        if (Fvi[2] != Fvi[3])
        {
          if ( 1 == ftc.quad[3] ) f0 += ftc.Tx[3]; else if ( 4 == ftc.quad[3] ) f1 += twopitc-ftc.Tx[3];
        }
        if (f0 >= f1 )
        {
          // "most" of the face is on the left side of the texture 
          // If a vertex is on the right side, clamp its tc to 0.
          if ( 4 == ftc.quad[0] ) {ftc.Tx[0] = 0.0f; ftc.fi = fi;}
          if ( 4 == ftc.quad[1] ) {ftc.Tx[1] = 0.0f; ftc.fi = fi;}
          if ( 4 == ftc.quad[2] ) {ftc.Tx[2] = 0.0f; ftc.fi = fi;}
          if ( 4 == ftc.quad[3] ) {ftc.Tx[3] = 0.0f; ftc.fi = fi;}
        }
        else
        {
          // "most" of the face is on the right side of the texture 
          // If a vertex is on the left side, clamp its tc to two_pi_tc.
          if ( 1 == ftc.quad[0] ) {ftc.Tx[0] = twopitc; ftc.fi = fi;}
          if ( 1 == ftc.quad[1] ) {ftc.Tx[1] = twopitc; ftc.fi = fi;}
          if ( 1 == ftc.quad[2] ) {ftc.Tx[2] = twopitc; ftc.fi = fi;}
          if ( 1 == ftc.quad[3] ) {ftc.Tx[3] = twopitc; ftc.fi = fi;}
        }
      }
    }

    if ( ftc.fi >= 0 )
    {
      // face will require special handling
      ftc_list.Append(ftc);    
    }
  }

  ftc_count = ftc_list.Count();
  if ( ftc_count <= 0 )
    return;

  // Count the number of new vertices that will be added.
  int ftci;
  int newvcnt = 0;
  for ( ftci = 0; ftci < ftc_count; ftci++ )
  {
    ON__CMeshFaceTC& ftc = ftc_list[ftci];
    Fvi = F[ftc.fi].vi;
    if ( ftc.Tx[0] != Tx[Fvi[0]] )
    {
      ftc.bSetT[0] = true;
      newvcnt++;
    }
    if ( ftc.Tx[1] != Tx[Fvi[1]] )
    {
      ftc.bSetT[1] = true;
      newvcnt++;
    }
    if ( ftc.Tx[2] != Tx[Fvi[2]] )
    {
      ftc.bSetT[2] = true;
      newvcnt++;
    }
    if ( Fvi[2] != Fvi[3] )
    {
      if ( ftc.Tx[3] != Tx[Fvi[3]] )
      {
        ftc.bSetT[3] = true;
        newvcnt++;
      }
    }
  }

  if ( newvcnt <= 0 )
    return;


  F = 0; // Setting them to NULL makes sure anybody who
         // tries to use them below will crash.

  // reserve room for new vertex information
  ON__CChangeTextureCoordinateHelper helper(mesh,newvcnt,mesh_T);

  // add vertices and update mesh faces
  for ( ftci = 0; ftci < ftc_count; ftci++ )
  {
    const ON__CMeshFaceTC& ftc = ftc_list[ftci];
    int* meshFvi = mesh.m_F[ftc.fi].vi;

    if ( ftc.bSetT[0] )
    {
      helper.ChangeTextureCoordinate(meshFvi,0,ftc.Tx[0],ON_UNSET_FLOAT,mesh_T,mesh_T_stride);
    }
    if ( ftc.bSetT[1] )
    {
      helper.ChangeTextureCoordinate(meshFvi,1,ftc.Tx[1],ON_UNSET_FLOAT,mesh_T,mesh_T_stride);
    }
    if ( ftc.bSetT[2] )
    {
      helper.ChangeTextureCoordinate(meshFvi,2,ftc.Tx[2],ON_UNSET_FLOAT,mesh_T,mesh_T_stride);
    }
    if ( ftc.bSetT[3] )
    {
      helper.ChangeTextureCoordinate(meshFvi,3,ftc.Tx[3],ON_UNSET_FLOAT,mesh_T,mesh_T_stride);
    }
  }
}

static
bool SeamCheckHelper( const ON_TextureMapping& mp, 
                      double& two_pi_tc, 
                      ON_SimpleArray<int>& Tside,
                      ON_SimpleArray<int>*& Tsd )
{
  bool bSeamCheck = false;
  switch(mp.m_type)
  {
    case ON_TextureMapping::box_mapping:
      if ( ON_TextureMapping::divided == mp.m_texture_space )
      {
        if ( mp.m_bCapped )
          two_pi_tc = 2.0/3.0;
        Tsd = &Tside;
        bSeamCheck = true;
      }
      else if ( ON_TextureMapping::single == mp.m_texture_space )
      {
        Tsd = &Tside;
        bSeamCheck = true;
      }
      break;

    case ON_TextureMapping::cylinder_mapping:
      if ( ON_TextureMapping::divided == mp.m_texture_space )
      {
        two_pi_tc = 2.0/3.0;
        Tsd = &Tside;
      }
      bSeamCheck = true;
      break;

    case ON_TextureMapping::sphere_mapping:
      bSeamCheck = true;
      break;

    default:
      // intentionally skip other enum values
      break;
  }

  return bSeamCheck;
}

//If there are unused vertices, this may not work correctly - but it is very fast.
static inline bool HasSharedVertices(const ON_Mesh& mesh)
{
  return mesh.m_V.Count() < ((mesh.TriangleCount() * 3) + (mesh.QuadCount() * 4));
}


const ON_TextureCoordinates* ON_Mesh::SetCachedTextureCoordinates( 
        const class ON_TextureMapping& mapping,
				const class ON_Xform* mesh_xform,
        bool bLazy
        )
{
  if ( mapping.RequiresVertexNormals() && !HasVertexNormals() )
    ComputeVertexNormals();

  ON_TextureMapping mp = mapping;
  double two_pi_tc = 1.0;
  ON_SimpleArray<int> Tside;
  ON_SimpleArray<int>* Tsd = 0;
  bool bSeamCheck = SeamCheckHelper( mp, two_pi_tc, Tside, Tsd ) && HasSharedVertices(*this);
  if ( bSeamCheck )
    mp.m_uvw.Identity();

  ON_TextureCoordinates* TC = 0;
  {
    for ( int i = 0; i < m_TC.Count(); i++ )
    {
      if ( m_TC[i].m_tag.m_mapping_id == mapping.m_mapping_id )
      {
        TC = &m_TC[i];
        break;
      }
    }
  }
  if ( bLazy && TC && mapping.HasMatchingTextureCoordinates( TC->m_tag, mesh_xform ) )
    return TC;

  if ( !TC )
  {
    m_TC.AppendNew();
    TC = m_TC.Last();
  }

  // Use mp instead of mapping to call GetTextureCoordinates()
  // because m_uvw must be the identity if we have seams.
  if ( !mp.GetTextureCoordinates( *this,TC->m_T,mesh_xform,false,Tsd) )
  {
    int tci = (int)(TC - m_TC.Array());
    m_TC.Remove(tci);
    return 0;
  }

  TC->m_tag.Set(mapping);
  if (    mesh_xform && mesh_xform->IsValid()
       && !mesh_xform->IsIdentity() 
       && !mesh_xform->IsZero()
     )
  {
    TC->m_tag.m_mesh_xform = *mesh_xform;
  }

  TC->m_dim = 2;

  if ( bSeamCheck &&  m_F.Count() > 0 && TC->m_T.Count() == m_V.Count() )
  {
    float* mesh_T = (float*)TC->m_T.Array();
    int mesh_T_stride = sizeof(TC->m_T[0])/sizeof(mesh_T[0]);
    if ( Tsd && Tside.Count() != m_V.Count() )
      Tsd = 0;
    AdjustMeshPeriodicTextureCoordinatesHelper( *this, mesh_xform, mesh_T, mesh_T_stride, Tsd ? Tside.Array() : 0, two_pi_tc, mp );
    mesh_T = 0; // when the array is grown, the pointer may become invalid
    if ( !mapping.m_uvw.IsIdentity() && !mapping.m_uvw.IsZero() )
    {
      // Apply the uvw transformation that is on mapping
      // to the texture coordinates.
      ON_3dPoint T;
      int vi, vcnt = TC->m_T.Count();
      ON_3fPoint* meshT = TC->m_T.Array();
      for ( vi = 0; vi < vcnt; vi++ )
      {
        T = meshT[vi];
        T = mapping.m_uvw*T;
        meshT[vi] = T;
      }
    }
  }

  return TC;
}

bool ON_Mesh::SetTextureCoordinates(
                  const class ON_TextureMapping& mapping, 
                  const class ON_Xform* mesh_xform,
                  bool bLazy
                  )
{
  if ( mapping.RequiresVertexNormals() && !HasVertexNormals() )
    ComputeVertexNormals();

  InvalidateTextureCoordinateBoundingBox();

  ON_SimpleArray<int> Tside;
  ON_SimpleArray<int>* Tsd = 0;
  ON_TextureMapping mp = mapping;
  
  double two_pi_tc = 1.0;

  bool bSeamCheck = SeamCheckHelper( mp, two_pi_tc, Tside, Tsd ) && HasSharedVertices(*this);
  if ( bSeamCheck )
    mp.m_uvw.Identity();

  // Use mp instead of mapping to call GetTextureCoordinates()
  // because m_uvw must be the identity if we have seams.
  bool rc = mp.GetTextureCoordinates(*this,m_T,mesh_xform,bLazy,Tsd);
  
  if (rc)
  {
    // update the texture coordinate tag
    m_Ttag.Set(mapping);
    if (    mesh_xform 
         && mesh_xform->IsValid() 
         && !mesh_xform->IsIdentity() 
         && !mesh_xform->IsZero() 
       )
    {
      m_Ttag.m_mesh_xform  = *mesh_xform;
    }
  }

  if ( rc && bSeamCheck && HasTextureCoordinates() && m_F.Count() > 0 )
  {
    float* mesh_T = (float*)m_T.Array();
    int mesh_T_stride = sizeof(m_T[0])/sizeof(mesh_T[0]);
    if ( Tsd && Tside.Count() != m_V.Count() )
      Tsd = 0;
    AdjustMeshPeriodicTextureCoordinatesHelper( *this, mesh_xform, mesh_T, mesh_T_stride, Tsd ? Tside.Array() : 0, two_pi_tc, mp );
    mesh_T = 0; // when the array is grown, the pointer may become invalid
    if ( !mapping.m_uvw.IsIdentity() && !mapping.m_uvw.IsZero() )
    {
      // Apply the uvw transformation that is on mapping
      // to the texture coordinates.
      ON_2fPoint* meshT = m_T.Array();
      ON_3dPoint T;
      int vi, vcnt = m_T.Count();
      for ( vi = 0; vi < vcnt; vi++ )
      {
        T.x = meshT[vi].x;
        T.y = meshT[vi].y;
        T.z = 0.0;
        T = mapping.m_uvw*T;
        meshT[vi].x = (float)T.x;
        meshT[vi].y = (float)T.y;
      }
    }
  }

  return rc;
}

ON_MappingChannel::ON_MappingChannel()
{
  Default();
}

void ON_MappingChannel::Default()
{
  memset(this,0,sizeof(*this));
  m_mapping_channel_id = 1;
  m_mapping_index = -1;
  m_object_xform.Identity();
}

int ON_MappingChannel::Compare( const ON_MappingChannel& other ) const
{
  int rc = m_mapping_channel_id - other.m_mapping_channel_id;
  if (!rc)
    rc = ON_UuidCompare(m_mapping_id,other.m_mapping_id);
  return rc;
}

bool ON_MappingChannel::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if (rc)
  {
    rc = archive.WriteInt(m_mapping_channel_id);
    if (rc) rc = archive.WriteUuid(m_mapping_id);

    // 1.1 field added 6 June 2006
    if (rc) rc = archive.WriteXform(m_object_xform);

    if ( !archive.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_MappingChannel::Read( ON_BinaryArchive& archive )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (rc)
  {
    rc = (1 == major_version);
    if (rc) rc = archive.ReadInt(&m_mapping_channel_id);
    if (rc) rc = archive.ReadUuid(m_mapping_id);

    if ( rc && minor_version >= 1 )
    {
      // 1.1 field added 6 June 2006
      if (rc) rc = archive.ReadXform(m_object_xform);
      if (rc 
          && archive.ArchiveOpenNURBSVersion() < 200610030 
          && m_object_xform.IsZero()
          )
      {
        // Between versions 200606060 and 200610030,
        // there was a bug that created some mapping
        // channels with zero transformations.  This
        // if clause finds those and sets them to the
        // identity.
        m_object_xform.Identity();
      }
    }

    if ( !archive.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}


ON_MaterialRef::ON_MaterialRef()
{
  Default();
}

ON_MappingRef::ON_MappingRef()
{
  Default();
}

void ON_MaterialRef::Default()
{
  memset(this,0,sizeof(*this));
  // runtme index value of -1 means not set
  m_material_index          = -1;
  m_material_backface_index = -1;
  m_material_source = ON::material_from_layer;
}

void ON_MappingRef::Default()
{
  m_plugin_id = ON_nil_uuid;
  m_mapping_channels.Destroy();
}

int ON_MaterialRef::Compare( const ON_MaterialRef& other ) const
{
  int rc = ON_UuidCompare(m_plugin_id,other.m_plugin_id);
  if (rc)
    rc = ((int)m_material_source) - ((int)other.m_material_source);
  if (!rc)
    rc = ON_UuidCompare(m_material_id,other.m_material_id);
  if (!rc)
    rc = ON_UuidCompare(m_material_backface_id,other.m_material_backface_id);
  return rc;
}

int ON_MappingRef::Compare( const ON_MappingRef& other ) const
{
  int rc = ON_UuidCompare(m_plugin_id,other.m_plugin_id);
  if ( !rc)
  {
    const int count = m_mapping_channels.Count();
    rc = count - other.m_mapping_channels.Count();
    if (!rc)
    {
      for ( int i = 0; i < count && !rc; i++ )
      {
        rc = m_mapping_channels[i].Compare(other.m_mapping_channels[i]);
      }
    }
  }
  return rc;
}


bool ON_MaterialRef::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 1 );
  if (rc)
  {
    if (rc) rc = archive.WriteUuid( m_plugin_id );
    if (rc) rc = archive.WriteUuid( m_material_id );

    // 23 May 2006 Dale lear
    //   m_mapping_channels[] was removed from ON_MaterialRef.
    //   To keep from breaking the file format, I need to
    //   write a zero as the array length.
    //
    //if (rc) rc = archive.WriteArray( m_mapping_channels );
    if (rc) rc = archive.WriteInt(0);

    // 23 May 2006 added 
    if (rc) rc = archive.WriteUuid( m_material_backface_id );
    if (rc) rc = archive.WriteInt( m_material_source );


    if ( !archive.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_MaterialRef::Read( ON_BinaryArchive& archive )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (rc)
  {
    rc = (1 == major_version);

    if (rc) rc = archive.ReadUuid( m_plugin_id );
    if (rc) rc = archive.ReadUuid( m_material_id );

    // 23 May 2006 Dale lear
    //   m_mapping_channels[] was removed from ON_MaterialRef.
    //   To keep from breaking the file format, I need to
    //   write a zero as the array length.
    ON_SimpleArray<ON_MappingChannel> obsolete_mapping_channels;
    if (rc) rc = archive.ReadArray( obsolete_mapping_channels );

    if ( minor_version >= 1 )
    {
      if (rc) rc = archive.ReadUuid( m_material_backface_id );
      int i = m_material_source;
      if (rc) rc = archive.ReadInt( &i );
      if (rc) m_material_source = (unsigned char)ON::ObjectMaterialSource(i);
    }

    if ( !archive.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}

ON::object_material_source ON_MaterialRef::MaterialSource() const
{
  return ON::ObjectMaterialSource(m_material_source);
}

bool ON_MappingRef::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
  if (rc)
  {
    if (rc) rc = archive.WriteUuid( m_plugin_id );
    if (rc) rc = archive.WriteArray( m_mapping_channels );
    
    if ( !archive.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_MappingRef::Read( ON_BinaryArchive& archive )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (rc)
  {
    rc = (1 == major_version);

    if (rc) rc = archive.ReadUuid( m_plugin_id );
    if (rc) rc = archive.ReadArray( m_mapping_channels );

    if ( !archive.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_MappingRef::Transform( const ON_Xform& xform )
{
  int count = m_mapping_channels.Count();
  if ( count > 0 )
  {
    for ( ON_MappingChannel* mapping_channel = m_mapping_channels.Array();
          count--;
          mapping_channel++ )
    {
      mapping_channel->m_object_xform = xform*mapping_channel->m_object_xform;
    }
  }
  return true;
}

ON_ObjectRenderingAttributes::ON_ObjectRenderingAttributes()
{
  Default();
}

ON_RenderingAttributes::ON_RenderingAttributes()
{
  Default();
}

void ON_ObjectRenderingAttributes::Default()
{
  ON_RenderingAttributes::Default();
  m_mappings.Destroy();
  m_bCastsShadows = true;
  m_bReceivesShadows = true;
  m_bits = 0;
  m_reserved1 = 0;
}

void ON_RenderingAttributes::Default()
{
  m_materials.Destroy();
}

void ON_ObjectRenderingAttributes::EnableAdvancedTexturePreview(bool b)
{
  if ( b )
    m_bits |= 1;    // set bit 1
  else 
    m_bits &= 0xFE; // clear bit 1
}

bool ON_ObjectRenderingAttributes::AdvancedTexturePreview() const
{
  return (0 != (1 & m_bits)) ? true : false;
}

bool ON_RenderingAttributes::IsValid( ON_TextLog* text_log ) const
{
  // plug-in uuids must be unique
  int count;
  if( (count = m_materials.Count()) > 1 )
  {
    const ON_MaterialRef* mr = m_materials.Array();
    ON_UUID plugin_id;
    int i, j;
    for ( i = 0; i < count-1; i++ )
    {
      plugin_id = mr[i].m_plugin_id;
      for ( j = i+1; j < count; j++ )
      {
        if ( !ON_UuidCompare(&plugin_id,&mr[j].m_plugin_id ) )
        {
          if( text_log )
          {
            text_log->Print("ON_RenderingAttributes error: m_materials[%d] and m_materials[%d] have the same plug-in id.\n",i,j);
          }
          return false;
        }
      }
    }    
  }
  return true;
}

bool ON_ObjectRenderingAttributes::IsValid( ON_TextLog* text_log ) const
{
  if ( !ON_RenderingAttributes::IsValid(text_log) )
    return false;

  // plug-in uuids must be unique
  int count;
  if( (count = m_mappings.Count()) > 1 )
  {
    const ON_MappingRef* mr = m_mappings.Array();
    ON_UUID plugin_id;
    int i, j;
    for ( i = 0; i < count-1; i++ )
    {
      plugin_id = mr[i].m_plugin_id;
      for ( j = i+1; j < count; j++ )
      {
        if ( !ON_UuidCompare(&plugin_id,&mr[j].m_plugin_id ) )
        {
          if( text_log )
          {
            text_log->Print("ON_ObjectRenderingAttributes error: m_mappings[%d] and m_mappings[%d] have the same plug-in id.\n",i,j);
          }
          return false;
        }
      }
    }    
  }

  return true;
}

int ON_RenderingAttributes::Compare( const ON_RenderingAttributes& other ) const
{
  const int count = m_materials.Count();
  int rc = count - other.m_materials.Count();
  if (!rc)
  {
    int i;
    for ( i = 0; i < count && !rc; i++ )
    {
      rc = m_materials[i].Compare(other.m_materials[i]);
    }
  }
  return rc;
}

const ON_MaterialRef* ON_RenderingAttributes::MaterialRef( const ON_UUID& plugin_id ) const
{
  int count;
  if ( (count = m_materials.Count()) > 0 )
  {
    for ( const ON_MaterialRef* mr = m_materials.Array(); count--; mr++ )
    {
      if ( plugin_id == mr->m_plugin_id )
        return mr;
    }
  }
  return 0;
}

int ON_ObjectRenderingAttributes::Compare( const ON_ObjectRenderingAttributes& other ) const
{
  int rc = ON_RenderingAttributes::Compare(other);
  if (!rc)
  {
    int i;
    const int count = m_mappings.Count();
    rc = other.m_mappings.Count() - count;
    for ( i = 0; i < count && !rc; i++ )
    {
      rc = m_mappings[i].Compare(other.m_mappings[i]);
    }
    if ( !rc )
    {
      rc = ((int)(m_bCastsShadows?1:0)) - ((int)(other.m_bCastsShadows?1:0));
      if ( !rc )
      {
        rc = ((int)(m_bReceivesShadows?1:0)) - ((int)(other.m_bReceivesShadows?1:0));
      }
	  if ( !rc )
	  {
	    rc = ((int)(AdvancedTexturePreview()?1:0)) - ((int)(other.AdvancedTexturePreview()?1:0));
	  }
    }
  }
  return rc;
}

bool ON_ObjectRenderingAttributes::Transform( const ON_Xform& xform )
{
  int i;
  if ( (i = m_mappings.Count()) > 0 )
  {
    for( ON_MappingRef* mr = m_mappings.Array(); i--; mr++ )
      mr->Transform(xform);
  }
  return true;
}

const ON_MappingRef* ON_ObjectRenderingAttributes::MappingRef(
  const ON_UUID& plugin_id ) const
{
  int count;
  if ( (count = m_mappings.Count()) > 0 )
  {
    for ( const ON_MappingRef* mr = m_mappings.Array(); count--; mr++ )
    {
      if ( plugin_id == mr->m_plugin_id )
        return mr;
    }    
  }
  return 0;
}

ON_MappingRef* ON_ObjectRenderingAttributes::AddMappingRef( 
  const ON_UUID& plugin_id 
  )
{
  ON_MappingRef* mr = 0;
  int count;
  if ( (count = m_mappings.Count()) > 0 )
  {
    for ( mr = const_cast<ON_MappingRef*>(m_mappings.Array()); count--; mr++ )
    {
      if ( plugin_id == mr->m_plugin_id )
        break;
    }    
  }

  if ( !mr )
  {
    mr = &m_mappings.AppendNew();
    mr->m_plugin_id = plugin_id;
  }

  return mr;
}

bool ON_ObjectRenderingAttributes::DeleteMappingRef( 
  const ON_UUID& plugin_id 
  )
{
  const ON_MappingRef* mr = MappingRef(plugin_id);
  if ( mr ) 
    m_mappings.Remove( (int)(mr - m_mappings.Array()) ); // safe ptr to in conversion
  return (0 != mr);  
}

const ON_MappingChannel* ON_ObjectRenderingAttributes::MappingChannel( 
  const ON_UUID& plugin_id, 
  const ON_UUID& mapping_id
  ) const
{
  const ON_MappingRef* mr = MappingRef(plugin_id);
  if ( mr )
  {
    int count;
    if ( (count = mr->m_mapping_channels.Count()) > 0 )
    {
      for ( const ON_MappingChannel* mc = mr->m_mapping_channels.Array(); count--; mc++ )
      {
        if ( mapping_id == mc->m_mapping_id )
          return mc;
      }
    }
  }
  return 0;
}

const ON_MappingChannel* ON_ObjectRenderingAttributes::MappingChannel( 
  const ON_UUID& plugin_id, 
  int mapping_channel_id
  ) const
{
  const ON_MappingRef* mr = MappingRef(plugin_id);
  if ( mr )
  {
    int count;
    if ( (count = mr->m_mapping_channels.Count()) > 0 )
    {
      for ( const ON_MappingChannel* mc = mr->m_mapping_channels.Array(); count--; mc++ )
      {
        if ( mapping_channel_id == mc->m_mapping_channel_id )
          return mc;
      }
    }
  }
  return 0;
}



bool ON_ObjectRenderingAttributes::AddMappingChannel(
        const ON_UUID& plugin_id, 
        int mapping_channel_id,
        const ON_UUID& mapping_id
        )
{
  ON_MappingRef* mr = const_cast<ON_MappingRef*>(MappingRef(plugin_id));
  if ( !mr )
  {
    mr = &m_mappings.AppendNew();
    mr->m_plugin_id = plugin_id;
    ON_MappingChannel& mc = mr->m_mapping_channels.AppendNew();
    mc.m_mapping_channel_id = mapping_channel_id;
    mc.m_mapping_id = mapping_id;
    mc.m_mapping_index = -1; // 27th October 2011 John Croudy - constructor is not called by AppendNew().
    mc.m_object_xform.Identity();
    return true;
  }

  return mr->AddMappingChannel(mapping_channel_id,mapping_id);
}

bool ON_ObjectRenderingAttributes::DeleteMappingChannel(
  const ON_UUID& plugin_id, 
  int mapping_channel_id
  )
{
  ON_MappingRef* mr = const_cast<ON_MappingRef*>(MappingRef(plugin_id));
  return mr ? mr->DeleteMappingChannel(mapping_channel_id) : false;
}

bool ON_ObjectRenderingAttributes::DeleteMappingChannel(
  const ON_UUID& plugin_id, 
  const ON_UUID& mapping_id
  )
{
  ON_MappingRef* mr = const_cast<ON_MappingRef*>(MappingRef(plugin_id));
  return mr ? mr->DeleteMappingChannel(mapping_id) : false;
}

bool ON_ObjectRenderingAttributes::ChangeMappingChannel(
  const ON_UUID& plugin_id, 
  int old_mapping_channel_id,
  int new_mapping_channel_id
  )
{
  ON_MappingRef* mr = const_cast<ON_MappingRef*>(MappingRef(plugin_id));
  return mr ? mr->ChangeMappingChannel(old_mapping_channel_id,new_mapping_channel_id) : false;
}

const ON_MappingChannel* ON_MappingRef::MappingChannel( 
  const ON_UUID& mapping_id
  ) const
{
  int count;
  if ( (count = m_mapping_channels.Count()) > 0 )
  {
    for ( const ON_MappingChannel* mc = m_mapping_channels.Array(); count--; mc++ )
    {
      if ( mapping_id == mc->m_mapping_id )
        return mc;
    }
  }
  return 0;
}

const ON_MappingChannel* ON_MappingRef::MappingChannel( 
  int mapping_channel_id
  ) const
{
  int count;
  if ( (count = m_mapping_channels.Count()) > 0 )
  {
    for ( const ON_MappingChannel* mc = m_mapping_channels.Array(); count--; mc++ )
    {
      if ( mapping_channel_id == mc->m_mapping_channel_id )
        return mc;
    }
  }
  return 0;
}



bool ON_MappingRef::AddMappingChannel(
        int mapping_channel_id,
        const ON_UUID& mapping_id
        )
{
  int i;
  if ( (i = m_mapping_channels.Count()) > 0 )
  {
    for ( const ON_MappingChannel* mc = m_mapping_channels.Array(); i--; mc++ )
    {
      if ( mapping_channel_id == mc->m_mapping_channel_id )
      {
        // a matching mapping channel id exists
        // return true if mapping_id matches
        return ( 0 == ON_UuidCompare(&mapping_id,&mc->m_mapping_id) );
      }
    }
  }

  ON_MappingChannel& mc   = m_mapping_channels.AppendNew();
  mc.m_mapping_channel_id = mapping_channel_id;
  mc.m_mapping_id         = mapping_id;
  mc.m_mapping_index      = -1; // 27th October 2011 John Croudy - constructor is not called by AppendNew().
  mc.m_object_xform.Identity();

  return true;
}

bool ON_MappingRef::DeleteMappingChannel(int mapping_channel_id)
{
  const ON_MappingChannel* mc = MappingChannel(mapping_channel_id);
  if ( mc )
  {
    m_mapping_channels.Remove((int)(mc - m_mapping_channels.Array()));
  }
  return ( 0 != mc);
}

bool ON_MappingRef::DeleteMappingChannel(const ON_UUID& mapping_id)
{
  const ON_MappingChannel* mc = MappingChannel(mapping_id);
  if ( mc )
  {
    m_mapping_channels.Remove((int)(mc - m_mapping_channels.Array()));
  }
  return ( 0 != mc);
}

bool ON_MappingRef::ChangeMappingChannel(
  int old_mapping_channel_id,
  int new_mapping_channel_id
  )
{
  ON_MappingChannel* mc = const_cast<ON_MappingChannel*>(MappingChannel(old_mapping_channel_id));
  if ( mc )
  {
    mc->m_mapping_channel_id = new_mapping_channel_id;
  }
  return ( 0 != mc );
}

bool ON_RenderingAttributes::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
  if ( !rc )
    return false;
  for(;;)
  {
    rc = archive.WriteArray(m_materials);
    if ( !rc ) break;

    break;
  }
  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

bool ON_RenderingAttributes::Read( ON_BinaryArchive& archive )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (!rc) 
    return false;
  for(;;)
  {
    rc = ( 1 == major_version );
    if (!rc) break;
    rc = archive.ReadArray(m_materials);
    if (!rc) break;

    break;
  }
  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}

bool ON_ObjectRenderingAttributes::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 3 );
  if ( !rc )
    return false;
  for(;;)
  {
    // DO NOT CALL ON_RenderingAttributes::Write 
    rc = archive.WriteArray(m_materials);
    if ( !rc ) break;
    rc = archive.WriteArray(m_mappings);
    if ( !rc ) break;

    // version 1.2 fields added 20061129
    rc = archive.WriteBool(m_bCastsShadows);
    if ( !rc ) break;
    rc = archive.WriteBool(m_bReceivesShadows);
    if ( !rc ) break;

    // version 1.3 fields added 20101019
    bool b = AdvancedTexturePreview();
    rc = archive.WriteBool(b);
    if ( !rc ) break;

    break;
  }
  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

bool ON_ObjectRenderingAttributes::Read( ON_BinaryArchive& archive )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (!rc) 
    return false;
  for(;;)
  {
    rc = ( 1 == major_version && minor_version >= 1 );
    if (!rc) break;

    // DO NOT CALL ON_RenderingAttributes::Read 
    if (rc) rc = archive.ReadArray(m_materials);
    if (!rc) break;
    if (rc) rc = archive.ReadArray(m_mappings);
    if (!rc) break;

    if ( minor_version <= 1 )
      break;

    // version 1.2 fields added 20061129
    rc = archive.ReadBool(&m_bCastsShadows);
    if ( !rc ) break;
    rc = archive.ReadBool(&m_bReceivesShadows);
    if ( !rc ) break;

    if ( minor_version <= 2 )
      break;

    // version 1.3 fields added 20101019
    bool b = AdvancedTexturePreview();
    rc = archive.ReadBool(&b);
    if ( !rc ) break;
    // Jussi 20120430: We don't want to enable advanced texture preview by default. It will be
    //                 turned on when needed (depending on active render plug-in etc).
    //EnableAdvancedTexturePreview(b);

    break;
  }
  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}


bool ON_TextureMapping::SetSurfaceParameterMapping(void)
{
  Default();
	m_type = srfp_mapping;
  ON_CreateUuid(m_mapping_id);
	return true;
}


bool ON_TextureMapping::SetPlaneMapping(
          const ON_Plane& plane,
          const ON_Interval& dx,
          const ON_Interval& dy,
          const ON_Interval& dz
          )
{
  Default();

  // Don't call plane.IsValid(), because the plane
  // equation does not matter and many developers
  // forget to set it correctly.
  if ( !plane.origin.IsValid() )
    return false;
  if ( !ON_IsRightHandFrame( plane.xaxis, plane.yaxis, plane.zaxis ) )
    return false;
  if ( !dx.IsValid() || !dy.IsValid() || !dz.IsValid() )
    return false;

  ON_3dPoint C = plane.PointAt(dx.Mid(),dy.Mid(),dz.Mid());
  C.x = (0.0 == C.x) ? 0.0 : -C.x;
  C.y = (0.0 == C.y) ? 0.0 : -C.y;
  C.z = (0.0 == C.z) ? 0.0 : -C.z;
  ON_3dVector xaxis = plane.xaxis;
  ON_3dVector yaxis = plane.yaxis;
  ON_3dVector zaxis = plane.zaxis;

  // Any "cleanup" needs to be done here 
  // to xaxis, yaxis, zaxis.
  
  double sx,sy,sz;
  if ( 0.0 == (sx = dx.Length())) sx = 2.0;
  if ( 0.0 == (sy = dy.Length())) sy = 2.0;
  if ( 0.0 == (sz = dz.Length())) sz = 2.0;

  // The plane mapping matrix m_Pxyz transforms the
  // world coordinate rectangle to a (-1<=r<=1,
  // on plane to a 
  // 1 X 1 square in the xy plane centered at the
  // origin.  

  // m_Pxyz = surface point transformation
  ON_3dVector X = (2.0/sx)*xaxis;
  ON_3dVector Y = (2.0/sy)*yaxis;
  ON_3dVector Z = (2.0/sz)*zaxis;

  m_Pxyz.m_xform[0][0] = X.x;
  m_Pxyz.m_xform[0][1] = X.y;
  m_Pxyz.m_xform[0][2] = X.z;
  m_Pxyz.m_xform[0][3] = (X.x*C.x + X.y*C.y + X.z*C.z);

  m_Pxyz.m_xform[1][0] = Y.x;
  m_Pxyz.m_xform[1][1] = Y.y;
  m_Pxyz.m_xform[1][2] = Y.z;
  m_Pxyz.m_xform[1][3] = (Y.x*C.x + Y.y*C.y + Y.z*C.z);

  m_Pxyz.m_xform[2][0] = Z.x;
  m_Pxyz.m_xform[2][1] = Z.y;
  m_Pxyz.m_xform[2][2] = Z.z;
  m_Pxyz.m_xform[2][3] = (Z.x*C.x + Z.y*C.y + Z.z*C.z);

  m_Pxyz.m_xform[3][0] = 0.0;
  m_Pxyz.m_xform[3][1] = 0.0;
  m_Pxyz.m_xform[3][2] = 0.0;
  m_Pxyz.m_xform[3][3] = 1.0;

  // m_Nxyz = surface normal transformation
  //        = inverse transpose of upper 3x3 of m_Pxyz
  X = (0.5*sx)*xaxis;
  Y = (0.5*sy)*yaxis;
  Z = (0.5*sz)*zaxis;
  m_Nxyz.m_xform[0][0] = X.x;
  m_Nxyz.m_xform[0][1] = X.y;
  m_Nxyz.m_xform[0][2] = X.z;
  m_Nxyz.m_xform[0][3] = 0.0;

  m_Nxyz.m_xform[1][0] = Y.x;
  m_Nxyz.m_xform[1][1] = Y.y;
  m_Nxyz.m_xform[1][2] = Y.z;
  m_Nxyz.m_xform[1][3] = 0.0;

  m_Nxyz.m_xform[2][0] = Z.x;
  m_Nxyz.m_xform[2][1] = Z.y;
  m_Nxyz.m_xform[2][2] = Z.z;
  m_Nxyz.m_xform[2][3] = 0.0;

  m_Nxyz.m_xform[3][0] = 0.0;
  m_Nxyz.m_xform[3][1] = 0.0;
  m_Nxyz.m_xform[3][2] = 0.0;
  m_Nxyz.m_xform[3][3] = 1.0;

  m_type = plane_mapping;
  ON_CreateUuid(m_mapping_id);

#if defined(ON_DEBUG)
  {
    ON_Plane p;
    p.xaxis = (2.0/sx)*plane.xaxis;
    p.yaxis = (2.0/sy)*plane.yaxis;
    p.zaxis = (2.0/sz)*plane.zaxis;
    p.origin.Set(-C.x,-C.y,-C.z);
    p.UpdateEquation();
    ON_Xform P_dbg, N_dbg;
    P_dbg.Rotation(p,ON_xy_plane);
    P_dbg.GetSurfaceNormalXform(N_dbg);

    for ( int i = 0; i < 4; i++ )
    {
      for ( int j = 0; j < 4; j++ )
      {
        if ( fabs(m_Pxyz[i][j] - P_dbg[i][j]) >= ON_SQRT_EPSILON*(fabs(m_Pxyz[i][j])+128.0) )
        {
          ON_ERROR("m_Pxyz is nor right\n");
          break;
        }
        if ( fabs(m_Nxyz[i][j] - N_dbg[i][j]) >= ON_SQRT_EPSILON*(fabs(m_Nxyz[i][j])+128.0) )
        {
          ON_ERROR("m_Nxyz is nor right\n");
          break;
        }
      }
    }
  }
#endif
	return true;
}

bool ON_TextureMapping::SetBoxMapping(const ON_Plane& plane,
                                      ON_Interval dx,
                                      ON_Interval dy,
                                      ON_Interval dz,
                                      bool bCapped 
                                      )
{
  bool rc = SetPlaneMapping(plane,dx,dy,dz);
  if (rc)
  {
    m_bCapped = bCapped;
    m_type = ON_TextureMapping::box_mapping;
  }
  return rc;
}

bool ON_TextureMapping::SetCylinderMapping(const ON_Cylinder& cylinder, bool bIsCapped)
{
  ON_Interval dr, dh;
  if ( !ON_IsValid(cylinder.circle.radius ) )
    return false;
  double r = cylinder.circle.radius;
  if ( 0.0 == r )
    r = 1.0;
  dr.Set(-r,r);
  dh.Set(cylinder.height[0],cylinder.height[1]);
  if ( dh[0] == dh[1] )
  {
    if ( ON_UNSET_VALUE == dh[0] )
    {
      dh.Set(-1.0,1.0);
    }
    else
    {
      dh.m_t[0] -= 1.0;
      dh.m_t[0] += 1.0;
    }
  }
  if ( !dh.IsValid() )
    return false;

  bool rc = SetBoxMapping(cylinder.circle.plane,dr,dr,dh,bIsCapped);
  if (rc)
  {
	  m_type = cylinder_mapping;
  }

	return rc;
}

bool ON_TextureMapping::SetSphereMapping(const ON_Sphere& sphere)
{
  ON_Interval dr(-sphere.radius,sphere.radius);
  bool rc = SetBoxMapping(sphere.plane,dr,dr,dr,false);
  if (rc)
  {
	  m_type = sphere_mapping;
  }
	return rc;
}



bool ON_TextureMapping::GetMappingPlane(ON_Plane& plane,
                                        ON_Interval& dx,
                                        ON_Interval& dy,
                                        ON_Interval& dz
                                        ) const
{
  ON_Xform xform(m_Pxyz);

  ON_3dVector S(((ON_3dVector*)&xform.m_xform[0])->Length(),
                ((ON_3dVector*)&xform.m_xform[1])->Length(),
                ((ON_3dVector*)&xform.m_xform[2])->Length());

  if ( 0.0 == S.x )
    return false;
  S.x = 1.0/S.x;
  if ( 0.0 == S.y )
    return false;
  S.y = 1.0/S.y;
  if ( 0.0 == S.z )
    return false;
  S.z = 1.0/S.z;

  xform.m_xform[0][0] *= S.x; xform.m_xform[0][1] *= S.x; xform.m_xform[0][2] *= S.x;
  xform.m_xform[0][3] *= S.x;  
  
  xform.m_xform[1][0] *= S.y; xform.m_xform[1][1] *= S.y; xform.m_xform[1][2] *= S.y;
  xform.m_xform[1][3] *= S.y;  

  xform.m_xform[2][0] *= S.z; xform.m_xform[2][1] *= S.z; xform.m_xform[2][2] *= S.z;
  xform.m_xform[2][3] *= S.z;

  xform.m_xform[3][0] = 0.0;
  xform.m_xform[3][1] = 0.0;
  xform.m_xform[3][2] = 0.0;
  xform.m_xform[3][3] = 1.0;

  ON_Xform inv(xform);
  if ( !inv.Invert() )
    return false;

  plane.origin.Set(inv.m_xform[0][3],inv.m_xform[1][3],inv.m_xform[2][3]);
  xform.m_xform[0][3] = 0.0; 
  xform.m_xform[1][3] = 0.0;
  xform.m_xform[2][3] = 0.0;
  plane.xaxis = &xform.m_xform[0][0];
  plane.yaxis = &xform.m_xform[1][0];
  plane.zaxis = &xform.m_xform[2][0];

	plane.UpdateEquation();

  dx.Set(-S.x,S.x);
  dy.Set(-S.y,S.y);
  dz.Set(-S.z,S.z);

  return plane.IsValid();
}

bool ON_TextureMapping::GetMappingBox(ON_Plane& plane,
                                      ON_Interval& dx,
                                      ON_Interval& dy,
                                      ON_Interval& dz) const
{
	return GetMappingPlane(plane, dx, dy, dz);
}

bool ON_TextureMapping::GetMappingCylinder(ON_Cylinder& cylinder) const
{
  ON_Interval dx, dy, dz;
  ON_Plane plane;
  bool rc = GetMappingPlane(cylinder.circle.plane, dx, dy, dz);
  if (rc)
  {
    double r0 = 0.5*dx.Length();
    double r1 = 0.5*dy.Length();
    cylinder.circle.radius = (r0 == r1) ? r0 : 0.5*(r0+r1);
    cylinder.height[0] = dz[0];
    cylinder.height[1] = dz[1];
  }

  return rc && cylinder.IsValid();
}

bool ON_TextureMapping::GetMappingSphere(ON_Sphere& sphere) const
{
  ON_Interval dx, dy, dz;
  bool rc = GetMappingPlane(sphere.plane, dx, dy, dz);
  if (rc)
  {
    double r0 = 0.5*dx.Length();
    double r1 = 0.5*dy.Length();
    double r2 = 0.5*dz.Length();
    sphere.radius = (r0 == r1 && r0 == r2) ? r0 : (r0+r1+r2)/3.0;
  }
  return rc && sphere.IsValid();
}


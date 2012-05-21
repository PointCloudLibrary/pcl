/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
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

#include <pcl/surface/openNURBS/opennurbs.h>

ON_OBJECT_IMPLEMENT(ON_Layer,ON_Object,"95809813-E985-11d3-BFE5-0010830122F0");

#define ON_BOZO_VACCINE_3E4904E6E9304fbcAA42EBD407AEFE3B
#define ON_BOZO_VACCINE_BFB63C094BC7472789BB7CC754118200

ON_Layer::ON_Layer() 
{
  Default();
}

void ON_Layer::Default()
{
  m_layer_id = ON_nil_uuid;
  m_parent_layer_id = ON_nil_uuid;
  m_layer_index = -1; // 10 March 2006 Dale Lear - changed from 0 to -1
  m_iges_level = -1; 
  m_material_index = -1; 
  m_rendering_attributes.Default();
  m_linetype_index = -1;
  m_color.SetRGB(0,0,0);
  m_display_material_id = ON_nil_uuid;
  m_plot_color = ON_UNSET_COLOR;
  m_plot_weight_mm = 0.0;
  m_name.Destroy();
  m_bVisible = true;
  m_bLocked = false;
  m_bExpanded = true;
  m__runtime_flags = 0;
}

ON_Layer::~ON_Layer()
{
  m_name.Destroy();
}

ON_BOOL32 ON_Layer::IsValid( ON_TextLog* text_log ) const
{
  if ( m_name.IsEmpty() )
  {
    if ( text_log )
    {
      text_log->Print("Layer name is empty.\n");
    }
    return false;
  }
  return true;
}


const wchar_t* ON_Layer::LayerNameReferenceDelimiter()
{
  return L" : ";
}

const wchar_t* ON_Layer::LayerNamePathDelimiter()
{
  return L"::";
}

static const wchar_t* LayerFullName( const wchar_t* s0 )
{
  if ( 0 == s0 || 0 == s0[0] )
    return 0;
  const wchar_t* t;
  const wchar_t* d;
  const wchar_t* d0 = ON_Layer::LayerNameReferenceDelimiter();
  const wchar_t* s = s0;

  // start at the beginning and look for a reference delimiter
  while ( 0 != *s )
  {
    if ( *s == *d0 )
    {
      d = d0;
      t = s;
      while ( *t == *d)
      {
        t++;
        d++;
        if ( 0 == *d )
        {
          return ((0 != *t) ? t : 0);
        }
      }
    }
    s++;
  }
  return s0;
}


static const wchar_t* LayerLeafName( const wchar_t* s )
{
  // this static helper function assumes s0 does not being with "reference : ".
  if ( 0 == s || 0 == s[0] )
    return 0;
  
  const wchar_t* t;
  const wchar_t* d;
  const wchar_t* d0 = ON_Layer::LayerNamePathDelimiter();
  const wchar_t* s0 = s;
  
  while ( 0 != *s0 )
  {
    if ( *s0 == *d0 )
    {
      // NOTE:
      //  This code must work for a delimiter of length one or more
      //  so the string returned by ON_Layer::LayerNamePathDelimiter()
      //  can be adjusted as needed.
      d = d0;
      t = s0;
      while ( *t == *d)
      {
        t++;
        d++;
        if ( 0 == *d )
        {
          if ( 0 == *t )
            return 0;
          s = t;
          s0 = t-1;
          break;
        }
      }
    }
    s0++;
  }

  return s;
}



bool ON_Layer::GetLeafName( const wchar_t* layer_name, ON_wString& leaf_name)
{
  const wchar_t* s0 = LayerFullName(layer_name);
  const wchar_t* s1 = LayerLeafName( s0 );
  if ( 0 != s1 && 0 != *s1 )
  {
    leaf_name = s1;
    return true;
  }
  leaf_name.Empty();
  return false;
}

bool ON_Layer::GetParentName( const wchar_t* layer_name, ON_wString& parent_path_name)
{
  const wchar_t* s0 = LayerFullName(layer_name);
  const wchar_t* s1 = LayerLeafName( s0 );
  if ( 0 != s1 && 0 != *s1 && s0 < s1 )
  {
    // back up over the delimiter
    const wchar_t* d0 = ON_Layer::LayerNamePathDelimiter();
    const wchar_t* d = d0;
    while (*d)
      d++;
    while ( d > d0 && s0 < s1 && d[-1] == s1[-1] )
    {
      d--;
      s1--;
    }
    if ( s0 < s1 )
    {
      parent_path_name = s0;
      parent_path_name.SetLength(s1-s0);
      return true;
    }
  }
  parent_path_name.Empty();
  return false;
}

bool ON_Layer::RemoveReferenceName( const wchar_t* layer_name, ON_wString& layer_path_name)
{
  const wchar_t* s = LayerFullName(layer_name);
  if ( 0 != s && 0 != *s )
  {
    layer_path_name = s;
    return true;
  }
  layer_path_name.Empty();
  return false;
}

bool ON_Layer::GetReferenceName( const wchar_t* layer_name, ON_wString& reference_name)
{
  const wchar_t* s0 = layer_name;
  const wchar_t* s1 = LayerFullName(layer_name);
  if ( 0 != s1 && 0 != *s1 && s0 < s1 )
  {
    const wchar_t* d = ON_Layer::LayerNameReferenceDelimiter();
    while ( *d++ && s0 < s1 )
      s1--;
    if ( 0 != *s1 && s0 < s1 )
    {
      reference_name = s0;
      reference_name.SetLength(s1-s0);
      return true;
    }
  }
  reference_name.Empty();
  return false;
}

void ON_Layer::Dump( ON_TextLog& dump ) const
{
  const wchar_t* sName = LayerName();
  if ( !sName )
    sName = L"";
  dump.Print("index = %d\n",m_layer_index);
  dump.Print("name = \"%S\"\n",sName);
  dump.Print("display = %s\n",m_bVisible?"visible":"hidden");
  dump.Print("picking = %s\n",m_bLocked?"locked":"unlocked");
  dump.Print("display color rgb = "); dump.PrintRGB(m_color); dump.Print("\n");
  dump.Print("plot color rgb = "); dump.PrintRGB(m_plot_color); dump.Print("\n");
  dump.Print("default material index = %d\n",m_material_index);
}

ON_BOOL32 ON_Layer::Write(
       ON_BinaryArchive& file // serialize definition to binary archive
     ) const
{
  int i;
  bool rc = file.Write3dmChunkVersion(1,8);
  while(rc)
  {
    // Save OBSOLETE mode value so we don't break file format
    if ( m_bVisible )
      i = 0; // "normal" layer mode
    else if ( m_bLocked )
      i = 2; // "locked" layer mode
    else
      i = 1; // "hidden" layer mode

    rc = file.WriteInt( i );
    if (!rc) break;

    rc = file.WriteInt( m_layer_index );
    if (!rc) break;

    rc = file.WriteInt( m_iges_level );
    if (!rc) break;

    rc = file.WriteInt( m_material_index );
    if (!rc) break;

    // Starting with version 200312110, this value is zero.  For files written
    // with earlier versions, the number was a "model index" value that was
    // set to something >= 1, but never used.  We have to continue to 
    // read/write an integer here so that old/new versions of opennurbs can
    // read files written by new/old versions.
    i = 0;
    rc = file.WriteInt( i );
    if (!rc) break;

    rc = file.WriteColor( m_color );
    if (!rc) break;
    
    {
      // OBSOLETE LINE STYLE if ( rc ) rc = file.WriteLineStyle( LineStyle() );
      // Starting with version 200503170, this section is "officially" not used.
      // Prior to that, it was old ON_LineStyle information that has
      // never been used.
      short s = 0;
      if (rc) rc = file.WriteShort(s);    // default pattern
      if (rc) rc = file.WriteShort(s);    // default pattern index
      if (rc) rc = file.WriteDouble(0.0); // default thickness
      if (rc) rc = file.WriteDouble(1.0); // default scale
    }
    if (!rc) break;

    rc = file.WriteString( m_name );
    if (!rc) break;

    // 1.1 fields
    rc = file.WriteBool(m_bVisible);
    if (!rc) break;

    // 1.2 field
    rc = file.WriteInt( m_linetype_index);
    if (!rc) break;

    // 1.3 field - 23 March 2005 Dale Lear
    rc = file.WriteColor( m_plot_color);
    if (!rc) break;
    rc = file.WriteDouble( m_plot_weight_mm);
    if (!rc) break;

    // 1.4 field - 3 May 2005 Dale Lear 
    //           - locked and visible are independent settings
    rc = file.WriteBool( m_bLocked );
    if (!rc) break;

    // 1.5 field
    rc = file.WriteUuid( m_layer_id );
    if (!rc) break;

    // 1.6 field
    rc = file.WriteUuid( m_parent_layer_id );
    if (!rc) break;

    // 1.6 field
    rc = file.WriteBool( m_bExpanded );
    if (!rc) break;

    // 1.7 field - added 6 June 2006
    rc = m_rendering_attributes.Write(file);
    if (!rc) break;

    // 1.8 field - added 19 Sep 2006
    rc = file.WriteUuid(m_display_material_id);

    break;
  }

  return rc;
}

ON_BOOL32 ON_Layer::Read(
       ON_BinaryArchive& file // restore definition from binary archive
     )
{
  int obsolete_value1 = 0; // see ON_Layer::Write
  int major_version=0;
  int minor_version=0;
  int mode = ON::normal_layer;
  Default();
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc && major_version == 1 ) 
  {
    // common to all 1.x formats
    if ( rc ) rc = file.ReadInt( &mode );
    if ( rc ) 
    {
      switch(mode)
      {
      case 0: // OBSOLETE ON::normal_layer
        m_bVisible = true;
        m_bLocked = false;
        break;
      case 1: // OBSOLETE ON::hidden_layer
        m_bVisible = false;
        m_bLocked = false;
        break;
      case 2: // OBSOLETE ON::locked_layer
        m_bVisible = true;
        m_bLocked = true;
        break;
      default:
        m_bVisible = true;
        m_bLocked = false;
        break;
      }
    }
    if ( rc ) rc = file.ReadInt( &m_layer_index );
    if ( rc ) rc = file.ReadInt( &m_iges_level );
    if ( rc ) rc = file.ReadInt( &m_material_index );
    if ( rc ) rc = file.ReadInt( &obsolete_value1 );
    if ( rc ) rc = file.ReadColor( m_color );

    {
      // OBSOLETE line style was never used - read and discard the next 20 bytes
      short s;
      double x;
      if (rc) file.ReadShort(&s);
      if (rc) file.ReadShort(&s);
      if (rc) file.ReadDouble(&x);
      if (rc) file.ReadDouble(&x);
    }

    if ( rc ) rc = file.ReadString( m_name );
    if ( rc && minor_version >= 1 )
    {
      rc = file.ReadBool(&m_bVisible);
      if ( rc && minor_version >= 2 )
      {
        rc = file.ReadInt( &m_linetype_index);
        if (rc && minor_version >= 3 )
        {
          // 23 March 2005 Dale Lear
          rc = file.ReadColor( m_plot_color);
          if (rc) rc = file.ReadDouble( &m_plot_weight_mm);

          if (rc && minor_version >= 4 )
          {
            rc = file.ReadBool(&m_bLocked);
            if (rc && minor_version >= 5 )
            {
              rc = file.ReadUuid(m_layer_id);
              if ( rc 
                   && minor_version >= 6 
                   && file.ArchiveOpenNURBSVersion() > 200505110
                 )
              {
                // Some files saved with opennurbs version 200505110 
                // do not contain correctly written m_parent_layer_id
                // and m_bExpanded values.
                // It is ok to default these values.
                rc = file.ReadUuid(m_parent_layer_id);
                if (rc)
                  rc = file.ReadBool(&m_bExpanded);

              }

              if ( rc && minor_version >= 7 )
              {
                // 1.7 field - added 6 June 2006
                rc = m_rendering_attributes.Read(file);

                if ( rc && minor_version >= 8 )
                {
                  // 1.8 field - added 19 Sep 2006
                  rc = file.ReadUuid(m_display_material_id);
                }
              }
            }
          }
        }
      }
    }

    if ( ON_UuidIsNil(m_layer_id) )
    {
      // old files didn't have layer ids and we need unique ones.
      ON_CreateUuid(m_layer_id);
    }
  }
  else {
    ON_ERROR("ON_Layer::Read() encountered a layer written by future code.");
    rc = false;
  }

  return rc;
}

ON::object_type ON_Layer::ObjectType() const
{
  return ON::layer_object;
}

//////////////////////////////////////////////////////////////////////
//
// Interface

bool ON_Layer::SetLayerName( const char* s )
{
  m_name = s;
  return IsValid()?true:false;
}

bool ON_Layer::SetLayerName( const wchar_t* s )
{
  m_name = s;
  return IsValid()?true:false;
}

const ON_wString& ON_Layer::LayerName() const
{
  return m_name;
}

void ON_Layer::SetColor( ON_Color c)
{
  m_color = c;
}

void ON_Layer::SetPlotColor( ON_Color c)
{
  m_plot_color = c;
}

ON_Color ON_Layer::Color() const
{
  return m_color;
}

ON_Color ON_Layer::PlotColor() const
{
  return ((m_plot_color == ON_UNSET_COLOR) ? m_color : m_plot_color);
}

bool ON_Layer::SetLinetypeIndex( int index)
{
  if( index >= -1)
  {
    m_linetype_index = index;
    return true;
  }
  return false;
}

int ON_Layer::LinetypeIndex() const
{
  return m_linetype_index;
}

bool ON_Layer::IsVisible() const
{
  return m_bVisible;
}

void ON_Layer::SetVisible( bool bVisible )
{
  m_bVisible = ( bVisible ? true : false );
}

void ON_Layer::SetLocked( bool bLocked )
{
  m_bLocked = ( bLocked ? true : false );
}

bool ON_Layer::IsLocked() const
{
  return m_bLocked;
}

bool ON_Layer::IsVisibleAndNotLocked() const
{
  return (m_bVisible && !m_bLocked);
}

bool ON_Layer::IsVisibleAndLocked() const
{
  return (m_bVisible && m_bLocked);
}

bool ON_Layer::SetRenderMaterialIndex( int i )
{
  m_material_index = i;
  return true;
}

int ON_Layer::RenderMaterialIndex() const
{
  return m_material_index;
}

bool ON_Layer::SetLayerIndex( int i )
{
  m_layer_index = i;
  return true;
}

int ON_Layer::LayerIndex() const
{
  return m_layer_index;
}


bool ON_Layer::SetIgesLevel( int level )
{
  m_iges_level = level;
  return true;
}

int ON_Layer::IgesLevel() const
{
  return m_iges_level;
}

double ON_Layer::PlotWeight() const
{
  return m_plot_weight_mm;
}

void ON_Layer::SetPlotWeight(double plot_weight_mm)
{
  m_plot_weight_mm = (ON_IsValid(plot_weight_mm) && (plot_weight_mm>0.0 || -1.0==plot_weight_mm) ) 
                   ? plot_weight_mm 
                   : 0.0;
}


////////////////////////////////////////////////////////////////
//
// BEGIN ON__LayerPerViewSettings class
//

class /*NEVER EXPORT THIS CLASS DEFINITION*/ ON__LayerPerViewSettings
{
#if !defined(ON_BOZO_VACCINE_3E4904E6E9304fbcAA42EBD407AEFE3B)
#error Never copy this class definition or put this definition in a header file!
#endif
public:
  ON__LayerPerViewSettings();
  void SetDefaultValues();
  bool Write(ON_BinaryArchive& binary_archive) const;
  bool Read(ON_BinaryArchive& binary_archive);

  ON_UUID m_viewport_id;
  ON_Color m_color;        // ON_UNSET_COLOR means use ON_Layer::m_color
  ON_Color m_plot_color;   // ON_UNSET_COLOR means use ON_Layer::m_plot_color
  double m_plot_weight_mm; // ON_UNSET_VALUE means use ON_Layer::m_plot_weight_mm
  unsigned char m_visible; // 0 means use ON_Layer::m_bVisible
                           // 1 = visible in viewport
                           // 2 = off in viewport

  enum
  {
    // these values are saved in files - do not change
    viewport_id_bit =  1,
    color_bit       =  2,
    plot_color_bit  =  4,
    plot_weight_bit =  8,
    visible_bit     = 16
  };

  static
  int Compare( const ON__LayerPerViewSettings* a, const ON__LayerPerViewSettings* b );

  static
  int CompareViewportId( const ON__LayerPerViewSettings* a, const ON__LayerPerViewSettings* b );

  unsigned int ActiveElements() const;
};

int ON__LayerPerViewSettings::Compare( const ON__LayerPerViewSettings* a, const ON__LayerPerViewSettings* b )
{
  int rc;
  if ( 0 == (rc = ON_UuidCompare(a->m_viewport_id,b->m_viewport_id)) )
  {
    unsigned int abits = a->ActiveElements();
    unsigned int bbits = b->ActiveElements();
    if ( 0 == (rc = ((int)abits) - ((int)bbits)) )
    {
      if ( 0 != (visible_bit & abits) )
      {
        rc = ((int)a->m_visible) - ((int)b->m_visible);
      }
      if ( 0 == rc && 0 != (color_bit & abits) )
      {
        rc = ((int)a->m_color) - ((int)b->m_color);
      }
      if ( 0 == rc && 0 != (plot_color_bit & abits) )
      {
        rc = ((int)a->m_plot_color) - ((int)b->m_plot_color);
      }
      if ( 0 == rc && 0 != (plot_weight_bit & abits) )
      {
        if ( a->m_plot_weight_mm < b->m_plot_weight_mm )
          rc = -1;
        else if ( a->m_plot_weight_mm > b->m_plot_weight_mm )
          rc = 1;
      }
    }
  }
  return rc;
}

int ON__LayerPerViewSettings::CompareViewportId( const ON__LayerPerViewSettings* a, const ON__LayerPerViewSettings* b )
{
  return ON_UuidCompare(a->m_viewport_id,b->m_viewport_id);
}

unsigned int ON__LayerPerViewSettings::ActiveElements() const
{
  unsigned int bits = 0;
  if ( !ON_UuidIsNil(m_viewport_id) )
  {
    if ( ON_UNSET_COLOR != m_color )
      bits |= color_bit;
    if ( ON_UNSET_COLOR != m_plot_color )
      bits |= plot_color_bit;
    if ( (m_plot_weight_mm >= 0.0 || -1.0 == m_plot_weight_mm) && ON_IsValid(m_plot_weight_mm) )
      bits |= plot_weight_bit;
    if ( 1 == m_visible || 2 == m_visible )
      bits |= visible_bit;
    if ( 0 != bits )
      bits |= viewport_id_bit; // non-nil viewport id
  }
  return bits;
}

ON__LayerPerViewSettings::ON__LayerPerViewSettings()
{
  SetDefaultValues();
}

void ON__LayerPerViewSettings::SetDefaultValues()
{
  memset(this,0,sizeof(*this));
  m_color = ON_UNSET_COLOR;
  m_plot_color = ON_UNSET_COLOR;
  m_plot_weight_mm = ON_UNSET_VALUE;
}

bool ON__LayerPerViewSettings::Write(ON_BinaryArchive& binary_archive) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;

  for(;;)
  {
    // This complicated "bits" stuff is to minimize number of bytes
    // written in the file.  Even though long term storage space is 
    // nearly free, we have lots of customers who complain about 
    // large file size and so ...
    unsigned int bits = ActiveElements();
    rc = binary_archive.WriteInt(1,&bits); if (!rc) break;
    if ( 0 == bits )
      break; // all settings are defaults or viewport_id is nil

    rc = binary_archive.WriteUuid(m_viewport_id);

    if ( 0 != (color_bit & bits) )
    {
      rc = binary_archive.WriteColor(m_color); 
      if (!rc) break;
    }
    if ( 0 != (plot_color_bit & bits) )
    {
      rc = binary_archive.WriteColor(m_plot_color); 
      if (!rc) break;
    }
    if ( 0 != (plot_weight_bit & bits) )
    {
      rc = binary_archive.WriteDouble(m_plot_weight_mm);
      if (!rc) break;
    }
    if ( 0 != (visible_bit & bits) )
    {
      rc = binary_archive.WriteChar(m_visible);
      if (!rc) break;
    }

    break;
  }

  if ( !binary_archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

bool ON__LayerPerViewSettings::Read(ON_BinaryArchive& binary_archive)
{
  SetDefaultValues();

  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if ( !rc ) break;

    // This complicated "bits" stuff is to minimize number of bytes
    // written in the file.  Even though long term storage space is 
    // nearly free, we have lots of customers who complain about 
    // large file size and so ...
    unsigned int bits = 0;
    rc = binary_archive.ReadInt(1,&bits); 
    if (!rc) break;
    if ( 0 == bits )
      break;

    rc = binary_archive.ReadUuid(m_viewport_id);
    if (!rc) break;

    if ( 0 != (color_bit & bits) )
    {
      rc = binary_archive.ReadColor(m_color); 
      if (!rc) break;
    }
    if ( 0 != (plot_color_bit & bits) )
    {
      rc = binary_archive.ReadColor(m_plot_color); 
      if (!rc) break;
    }
    if ( 0 != (plot_weight_bit & bits) )
    {
      rc = binary_archive.ReadDouble(&m_plot_weight_mm);
      if (!rc) break;
    }
    if ( 0 != (visible_bit & bits) )
    {
      rc = binary_archive.ReadChar(&m_visible);
      if (!rc) break;
    }

    break;
  }

  if ( !binary_archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

//
//
// END ON__LayerPerViewSettings class
//
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
//
// BEGIN ON__LayerExtensions user data class
//

class /*NEVER EXPORT THIS CLASS DEFINITION*/ ON__LayerExtensions : public ON_UserData
{
#if !defined(ON_BOZO_VACCINE_3E4904E6E9304fbcAA42EBD407AEFE3B)
#error Never copy this class definition or put this definition in a header file!
#endif
  ON_OBJECT_DECLARE(ON__LayerExtensions);

public:
  ON__LayerExtensions();
  ~ON__LayerExtensions();
  // default copy constructor and operator= work fine.

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
  bool IsEmpty() const;

  static
  ON__LayerPerViewSettings* ViewportSettings( const ON_Layer& layer, const ON_UUID& viewport_id, bool bCreate );

  static
  void DeleteViewportSettings( const ON_Layer& layer, const ON__LayerPerViewSettings* vp_settings_to_delete );

  static
  ON__LayerExtensions* LayerExtensions(const ON_Layer& layer,bool bCreate);

  // per viewport overrides of color, linetype, plot color, plot weight, and visibility
  ON_SimpleArray<ON__LayerPerViewSettings> m_vp_settings;
};

#undef ON_BOZO_VACCINE_3E4904E6E9304fbcAA42EBD407AEFE3B

ON_OBJECT_IMPLEMENT(ON__LayerExtensions,ON_UserData,"3E4904E6-E930-4fbc-AA42-EBD407AEFE3B");

ON__LayerExtensions* ON__LayerExtensions::LayerExtensions(const ON_Layer& layer,bool bCreate)
{
  ON__LayerExtensions* ud = ON__LayerExtensions::Cast(layer.GetUserData(ON__LayerExtensions::m_ON__LayerExtensions_class_id.Uuid()));
  unsigned char* layer__runtme_flags = ((unsigned char*)&layer.m_bExpanded) + sizeof(layer.m_bExpanded);
  if ( !ud )
  {
    if ( bCreate )
    {
      ud = new ON__LayerExtensions();
      const_cast<ON_Layer&>(layer).AttachUserData(ud);
      if ( 0x00 != *layer__runtme_flags )
      {
        // Set ON_Layer::m__runtime_flags = 0x00
        //   This permits ON_Layer visibility and color queries to check for userdata.
        *layer__runtme_flags = 0x00;
      }
    }
    else
    {
      if ( 0x01 != *layer__runtme_flags)
      {
        // Set ON_Layer::m__runtime_flags = 0x01
        //   Setting this bit prevents ON_Layer visibility and color queries that
        //   are often used in time critical code from performing the time 
        //   consuming check for userdata that rarely exists.
        *layer__runtme_flags = 0x01;
      }
    }
  }
  else if ( 0x00 != *layer__runtme_flags)
  {
    // Set ON_Layer::m__runtime_flags = 0x00
    //   This permits ON_Layer visibility and color queries to check for userdata.
    *layer__runtme_flags = 0x00;
  }
  return ud;
}

ON__LayerExtensions::ON__LayerExtensions()
{
  m_userdata_uuid = ON__LayerExtensions::m_ON__LayerExtensions_class_id.Uuid();
  m_application_uuid = ON_opennurbs_id;
  m_userdata_copycount = 1;
}

ON__LayerExtensions::~ON__LayerExtensions()
{
}

// virtual ON_Object override
ON_BOOL32 ON__LayerExtensions::IsValid( ON_TextLog* text_log ) const
{
  return true;
}

// virtual ON_Object override
unsigned int ON__LayerExtensions::SizeOf() const
{
  size_t sz = sizeof(*this) - sizeof(ON_UserData);
  sz += m_vp_settings.SizeOfArray();
  return (unsigned int)sz;
}

// virtual ON_Object override
ON__UINT32 ON__LayerExtensions::DataCRC(ON__UINT32 current_remainder) const
{
  ON__UINT32 crc = 0;
  crc = m_vp_settings.DataCRC(crc);
  return crc;
}

// virtual ON_Object override
ON_BOOL32 ON__LayerExtensions::Write(ON_BinaryArchive& binary_archive) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;

  for(;;)
  {
    int count = m_vp_settings.Count();
    rc = binary_archive.WriteInt(count);
    if ( !rc ) break;
    for ( int i = 0; i < count && rc; i++ )
    {
      rc = m_vp_settings[i].Write(binary_archive);
    }
    if (!rc) break;

    break;
  }

  if ( !binary_archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_Object override
ON_BOOL32 ON__LayerExtensions::Read(ON_BinaryArchive& binary_archive)
{
  m_vp_settings.SetCount(0);

  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    int count = 0;
    rc = binary_archive.ReadInt(&count);
    if ( !rc ) break;
    m_vp_settings.Reserve(count);
    for ( int i = 0; i < count; i++ )
    {
      rc = m_vp_settings.AppendNew().Read(binary_archive);
      if (!rc) 
      {
        m_vp_settings.Remove();
        break;
      }
      if ( 0 == m_vp_settings.Last()->ActiveElements() )
        m_vp_settings.Remove();
    }

    // to make ON_Layer::PerViewportSettingsCRC() return
    // equal values for equal settings, it is critical
    // that m_vp_settings[] be sorted.
    m_vp_settings.QuickSort(ON__LayerPerViewSettings::Compare);

    if (!rc) break;

    break;
  }

  if ( !binary_archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_UserData override
ON_BOOL32 ON__LayerExtensions::Archive() const
{
  return !IsEmpty();
}

// virtual ON_UserData override
ON_BOOL32 ON__LayerExtensions::GetDescription( ON_wString& description )
{
  description = L"Layer Extensions";
  return true;
}

ON__LayerPerViewSettings* ON__LayerExtensions::ViewportSettings( const ON_Layer& layer, const ON_UUID& viewport_id, bool bCreate )
{
  if ( !ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(layer,bCreate);
    if ( ud )
    {
      int i;
      const int vp_settings_count = ud->m_vp_settings.Count();
      ON__LayerPerViewSettings* vp_settings = ud->m_vp_settings.Array();
      for ( i = 0; i < vp_settings_count; i++ )
      {
        if ( 0 == memcmp(&viewport_id,&vp_settings[i].m_viewport_id,sizeof(ON_UUID)) )
          return (vp_settings+i);
      }
      if ( bCreate )
      {
        ON__LayerPerViewSettings& new_vp_settings = ud->m_vp_settings.AppendNew();
        vp_settings = ud->m_vp_settings.Array(); // appending can grow the array
        new_vp_settings.SetDefaultValues();
        new_vp_settings.m_viewport_id = viewport_id;

        // to make ON_Layer::PerViewportSettingsCRC() return
        // equal values for equal settings, it is critical
        // that m_vp_settings[] be sorted.
        ud->m_vp_settings.QuickSort(ON__LayerPerViewSettings::Compare);

        for ( i = 0; i <= vp_settings_count; i++ ) // "i <= ..." is correct because of the .AppendNew()
        {
          if ( 0 == memcmp(&viewport_id,&vp_settings[i].m_viewport_id,sizeof(ON_UUID)) )
            return (vp_settings+i);
        }
      }
    }
  }
  return 0;
}

void ON__LayerExtensions::DeleteViewportSettings( const ON_Layer& layer, const ON__LayerPerViewSettings* vp_settings_to_delete )
{
  ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(layer,false);
  if ( ud )
  {
    if ( 0 == vp_settings_to_delete )
    {
      delete ud;
      // Set ON_Layer::m__runtime_flags = 0x01
      //   Setting this bit indicates this layer does not have 
      //   ON__LayerExtensions user data and we can avoid calling the
      //   slower code in time critical visibility and color queries.
      unsigned char* layer__runtme_flags = (unsigned char*)&layer.m_bExpanded;
      layer__runtme_flags += sizeof(layer.m_bExpanded);
      *layer__runtme_flags = 0x01; 
    }
    else
    {
      const size_t vp_settings_count = ud->m_vp_settings.Count();
      if ( vp_settings_count > 0 )
      {
        const ON__LayerPerViewSettings* vp_settings0 = ud->m_vp_settings.Array();
        if ( vp_settings0 <= vp_settings_to_delete )
        {
          int i = (int)(vp_settings_to_delete-vp_settings0);
          ud->m_vp_settings.Remove(i);
        }
      }
      if ( ud->IsEmpty() )
      {
        delete ud;
        // Set ON_Layer::m__runtime_flags = 0x01
        //   Setting this bit indicates this layer does not have 
        //   ON__LayerExtensions user data and we can avoid calling the
        //   slower code in time critical visibility and color queries.
        unsigned char* layer__runtme_flags = (unsigned char*)&layer.m_bExpanded;
        layer__runtme_flags += sizeof(layer.m_bExpanded);
        *layer__runtme_flags = 0x01; 
      }
    }
  }
}

bool ON__LayerExtensions::IsEmpty() const
{
  const int count = m_vp_settings.Count();

  for ( int i = 0; i < count; i++ )
    if ( m_vp_settings[i].ActiveElements() )
      return false;

  return true; // nothing of value in this user data
}

//
// END ON__LayerExtensions user data class
//
////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////
//
// BEGIN ON_Layer per viewport interface functions
//

void ON_Layer::SetColor( ON_Color layer_color, const ON_UUID& viewport_id )
{
  if ( ON_UNSET_COLOR == layer_color )
  {
    DeletePerViewportColor(viewport_id);
  }
  if ( ON_UuidIsNil(viewport_id) )
  {
    DeletePerViewportColor(viewport_id);
    SetColor(layer_color);
  }
  else
  {
    bool bSet = ( layer_color != ON_UNSET_COLOR );
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, bSet );
    if ( vp_settings )
    {
      vp_settings->m_color = layer_color;
      if ( !bSet && !vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}

ON_Color ON_Layer::Color( const ON_UUID& viewport_id ) const
{
  if ( m__runtime_flags )
  {
    // no per viewport settings
    return m_color;
  }

  ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
  return (vp_settings && vp_settings->m_color != ON_UNSET_COLOR)
         ? vp_settings->m_color
         : m_color;
}

void ON_Layer::SetPlotColor( ON_Color plot_color, const ON_UUID& viewport_id )
{
  if ( ON_UNSET_COLOR == plot_color )
  {
    DeletePerViewportPlotColor(viewport_id);
  }
  if ( ON_UuidIsNil(viewport_id) )
  {
    DeletePerViewportPlotColor(viewport_id);
    SetPlotColor(plot_color);
  }
  else
  {
    bool bSet = ( plot_color != ON_UNSET_COLOR );
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, bSet );
    if ( vp_settings )
    {
      vp_settings->m_plot_color = (bSet) ? plot_color : ON_Color::UnsetColor;
      if ( !bSet && !vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}

ON_Color ON_Layer::PlotColor( const ON_UUID& viewport_id ) const
{
  if ( m__runtime_flags )
  {
    // no per viewport settings
    // 2-Nov-2009 Dale Fugier, modified to call default PlotColor()
    return PlotColor();
  }
  ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
  return (vp_settings && vp_settings->m_plot_color != ON_UNSET_COLOR)
         ? vp_settings->m_plot_color
         : PlotColor(); // 2-Nov-2009 Dale Fugier, modified to call default PlotColor()
}

void ON_Layer::SetPlotWeight( double plot_weight_mm, const ON_UUID& viewport_id )
{
  if ( !ON_IsValid(plot_weight_mm) )
  {
    DeletePerViewportPlotWeight(viewport_id);
  }
  else if ( ON_UuidIsNil(viewport_id) )
  {
    DeletePerViewportPlotWeight(viewport_id);
    SetPlotWeight(plot_weight_mm);
  }
  else
  {
    bool bSet = ( ON_IsValid(plot_weight_mm) && (plot_weight_mm>=0.0 || -1.0==plot_weight_mm) );
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, bSet );
    if ( vp_settings )
    {
      vp_settings->m_plot_weight_mm = (bSet) ? plot_weight_mm : ON_UNSET_VALUE;
      if ( !bSet && !vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}

double ON_Layer::PlotWeight( const ON_UUID& viewport_id ) const
{
  if ( m__runtime_flags )
  {
    // no per viewport settings
    return m_plot_weight_mm;
  }
  ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
  return (vp_settings && (vp_settings->m_plot_weight_mm >= 0.0 || -1.0 == vp_settings->m_plot_weight_mm) )
         ? vp_settings->m_plot_weight_mm
         : m_plot_weight_mm;
}


bool ON_Layer::IsVisible( const ON_UUID& viewport_id ) const
{
  if ( m__runtime_flags )
  {
    // no per viewport settings
    return m_bVisible;
  }

  if ( ON_UuidIsNil(viewport_id) )
  {
    // see if layer is possibly visible in any viewport
    if ( !m_bVisible )
    {
      // default setting is off - check for per view visibility
      ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
      if ( 0 != ud )
      {
        int i, count = ud->m_vp_settings.Count();
        for ( i = 0; i < count; i++ )
        {
          if ( 1 == ud->m_vp_settings[i].m_visible )
            return true; // layer is visible in this viewport
        }
      }
    }
  }
  else 
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
    if (vp_settings && vp_settings->m_visible)
    {
      if ( 1 == vp_settings->m_visible )
        return true;  // per viewport ON setting overrides layer setting
      if ( 2 == vp_settings->m_visible )
        return false; // per viewport OFF setting overrides layer setting
    }
  }
  return m_bVisible; // use layer setting
}

void ON_Layer::SetVisible( bool bVisible, const ON_UUID& viewport_id )
{
  if ( ON_UuidIsNil(viewport_id) )
  {
    // remove per view visible settings
    DeletePerViewportVisible(viewport_id);

    // set general visibility setting
    m_bVisible = bVisible?true:false;
  }
  else 
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, true );
    if (vp_settings)
    {
      vp_settings->m_visible = (bVisible)
        ? 1  // layer is always on in this viewport
        : 2; // layer is always off in this viewport
    }
  }
}


void ON_Layer::DeletePerViewportColor( const ON_UUID& viewport_id )
{
  if ( ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
    if ( 0 != ud )
    {
      for ( int i = ud->m_vp_settings.Count(); i--; /*empty iterator*/ )
      {
        ud->m_vp_settings[i].m_color = ON_UNSET_COLOR;
        if ( 0 == ud->m_vp_settings[i].ActiveElements() )
          ud->m_vp_settings.Remove(i);
      }
      if ( ud->IsEmpty() )
      {
        delete ud;
        // setting m__runtime_flags prevents searching for ON__LayerExtensions userdata
        // and speeds up visibility and color queries.
        m__runtime_flags = 0x01; 
      }
    }
  }
  else
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
    if (vp_settings) 
    {
      vp_settings->m_color = ON_UNSET_COLOR;
      if ( 0 == vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}

void ON_Layer::DeletePerViewportPlotColor( const ON_UUID& viewport_id )
{
  if ( ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
    if ( 0 != ud )
    {
      for ( int i = ud->m_vp_settings.Count(); i--; /*empty iterator*/ )
      {
        ud->m_vp_settings[i].m_plot_color = ON_UNSET_COLOR;
        if ( 0 == ud->m_vp_settings[i].ActiveElements() )
          ud->m_vp_settings.Remove(i);
      }
      if ( ud->IsEmpty() )
      {
        delete ud;
        // setting m__runtime_flags prevents searching for ON__LayerExtensions userdata
        // and speeds up visibility and color queries.
        m__runtime_flags = 0x01; 
      }
    }
  }
  else
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
    if (vp_settings) 
    {
      vp_settings->m_plot_color = ON_UNSET_COLOR;
      if ( 0 == vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}

int ON_Layer::UpdateViewportIds( const ON_UuidPairList& viewport_id_map )
{
  if ( viewport_id_map.Count() <= 0 )
    return 0;
  ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
  if ( 0 == ud )
    return 0;
  int rc = 0;
  ON_UUID new_id;
  for ( int i = 0; i < ud->m_vp_settings.Count(); i++ )
  {
    ON__LayerPerViewSettings& s = ud->m_vp_settings[i];
    if ( viewport_id_map.FindId1(s.m_viewport_id,&new_id) && s.m_viewport_id != new_id )
    {
      s.m_viewport_id = new_id;
      rc++;
    }
  }
  return rc;
}

void ON_Layer::DeletePerViewportPlotWeight( const ON_UUID& viewport_id )
{
  if ( ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
    if ( 0 != ud )
    {
      for ( int i = ud->m_vp_settings.Count(); i--; /*empty iterator*/ )
      {
        ud->m_vp_settings[i].m_plot_weight_mm = ON_UNSET_VALUE;
        if ( 0 == ud->m_vp_settings[i].ActiveElements() )
          ud->m_vp_settings.Remove(i);
      }
      if ( ud->IsEmpty() )
      {
        delete ud;
        // setting m__runtime_flags prevents searching for ON__LayerExtensions userdata
        // and speeds up visibility and color queries.
        m__runtime_flags = 0x01; 
      }
    }
  }
  else
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
    if (vp_settings) 
    {
      vp_settings->m_plot_weight_mm = ON_UNSET_VALUE;
      if ( 0 == vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}

void ON_Layer::DeletePerViewportVisible( const ON_UUID& viewport_id )
{
  if ( ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
    if ( 0 != ud )
    {
      for ( int i = ud->m_vp_settings.Count(); i--; /*empty iterator*/ )
      {
        ud->m_vp_settings[i].m_visible = 0;
        if ( 0 == ud->m_vp_settings[i].ActiveElements() )
          ud->m_vp_settings.Remove(i);
      }
      if ( ud->IsEmpty() )
      {
        delete ud;
        // setting m__runtime_flags prevents searching for ON__LayerExtensions userdata
        // and speeds up visibility and color queries.
        m__runtime_flags = 0x01; 
      }
    }
  }
  else
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
    if (vp_settings) 
    {
      vp_settings->m_visible = 0;
      if ( 0 == vp_settings->ActiveElements() )
        ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
    }
  }
}


bool ON_Layer::HasPerViewportSettings( const ON_UUID& viewport_id ) const
{
  bool rc = false;
  if ( ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
    if ( ud && ud->m_vp_settings.Count() > 0 )
      rc = true;
  }
  else if ( 0 != ON__LayerExtensions::ViewportSettings( *this, viewport_id, false ) )
  {
    rc = true;
  }
  return rc;
}

bool ON_Layer::CopyPerViewportSettings(ON_UUID source, ON_UUID destination)
{
  bool rc = false;
  if ( ON_UuidIsNotNil(source) && ON_UuidIsNotNil(destination) && ON_UuidCompare(source, destination)!=0 )
  {
    ON__LayerPerViewSettings* pSourceSettings = ON__LayerExtensions::ViewportSettings( *this, source, false );
    if( pSourceSettings )
    {
      // Copy source settings, since pSourceSettings pointer may become bogus
      // after creating the destination settings
      ON__LayerPerViewSettings src = *pSourceSettings;
      ON__LayerPerViewSettings* pDestinationSettings = ON__LayerExtensions::ViewportSettings( *this, destination, true);
      if( pDestinationSettings )
      {
        pDestinationSettings->m_color = src.m_color;
        pDestinationSettings->m_plot_color = src.m_plot_color;
        pDestinationSettings->m_plot_weight_mm = src.m_plot_weight_mm;
        pDestinationSettings->m_visible = src.m_visible;
        rc = true;
      }
    }
  }
  return rc;
}


void ON_Layer::DeletePerViewportSettings( const ON_UUID& viewport_id ) const
{
  if ( ON_UuidIsNil(viewport_id) )
  {
    ON__LayerExtensions::DeleteViewportSettings(*this,0);
  }
  else
  {
    ON__LayerPerViewSettings* vp_settings = ON__LayerExtensions::ViewportSettings( *this, viewport_id, false );
    if ( vp_settings )
      ON__LayerExtensions::DeleteViewportSettings(*this,vp_settings);
  }
}


void ON_Layer::CullPerViewportSettings( int viewport_id_count, const ON_UUID* viewport_id_list )
{
  ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
  if ( 0 != ud )
  {
    if ( viewport_id_count <= 0 )
    {
      // delete all per viewport settings
      delete ud;
      // setting m__runtime_flags prevents searching for ON__LayerExtensions userdata
      // and speeds up visibility and color queries.
      m__runtime_flags = 0x01; 
    }
    else if ( viewport_id_count > 0 && 0 != viewport_id_list )
    {
      int i, j;
      for ( i = ud->m_vp_settings.Count(); i--; /*empty iterator*/ )
      {
        const ON_UUID vp_id = ud->m_vp_settings[i].m_viewport_id;
        for ( j = 0; j < viewport_id_count; j++ )
        {
          if ( 0 == memcmp(&viewport_id_list[i],&vp_id,sizeof(vp_id)) )
            break;
        }
        if ( j >= viewport_id_count )
        {
          // ud->m_vp_settings[i].m_viewport_id is NOT in viewport_id_list[]
          ud->m_vp_settings.Remove(i);
        }
      }
      if ( ud->IsEmpty() )
      {
        // nothing useful in ud
        delete ud;
        // setting m__runtime_flags prevents searching for ON__LayerExtensions userdata
        // and speeds up visibility and color queries.
        const_cast<ON_Layer*>(this)->m__runtime_flags = 0x01; 
      }
    }
  }
}

ON__UINT32 ON_Layer::PerViewportSettingsCRC() const
{
  ON__UINT32 crc = 0;
  ON__LayerExtensions* ud = ON__LayerExtensions::LayerExtensions(*this,false);
  if ( 0 != ud )
    crc = ud->m_vp_settings.DataCRC(crc);
  return crc;
}

//
// END ON_Layer per viewport interface functions
//
////////////////////////////////////////////////////////////////



unsigned int ON_Layer::Differences( const ON_Layer& layer0, const ON_Layer& layer1 )
{
  /*
  enum
  {
    none = 0,
    userdata = 1,
    color = 2,
    plot_color = 4,
    plot_weight = 8,
    visible = 16,
    locked = 32,
    expanded = 64,
    all = 0xFFFFFFFF
  }
  */
  unsigned int differences = 0;


  const ON_UserData* ud0 = layer0.FirstUserData();
  const ON_UserData* ud1 = layer1.FirstUserData();
  while ( 0 != ud0 && 0 != ud1 )
  {
    if ( ud0->m_userdata_uuid != ud1->m_userdata_uuid )
      break;
    ud0 = ud0->Next();
    ud1 = ud1->Next();
  }
  if ( 0 != ud0 || 0 != ud1 )
    differences |= ON_Layer::userdata_settings;

  if ( layer0.m_color != layer1.m_color )
    differences |= ON_Layer::color_settings;

  if ( layer0.m_plot_color != layer1.m_plot_color )
    differences |= ON_Layer::plot_color_settings;

  if ( layer0.m_plot_weight_mm != layer1.m_plot_weight_mm )
    differences |= ON_Layer::plot_weight_settings;

  if ( layer0.m_bVisible != layer1.m_bVisible )
    differences |= ON_Layer::visible_settings;

  if ( layer0.m_bLocked != layer1.m_bLocked )
    differences |= ON_Layer::locked_settings;

  // Note:
  //  This function is used for comparing layers from different
  //  documents.  It does not make sense to compare values
  //  like m_linetype_index, m_material_index and so on because
  //  different indices may actually reference the same linetype
  //  or material.  If there is a compelling reason to compare
  //  other settings, they can be added.

  return differences;
}


void ON_Layer::Set( unsigned int settings, const ON_Layer& settings_values )
{
  if ( 0 != (ON_Layer::userdata_settings & settings) )
  {
    // save original user data on this layer
    ON_UserDataHolder ud; 
    ud.MoveUserDataFrom(*this);

    // make a complete copy of the userdata on settings_values
    CopyUserData(settings_values);

    // now restore any original user data that was not present on settings_values.
    ud.MoveUserDataTo(*this,true);
  }

  if ( 0 != (ON_Layer::color_settings & settings) )
  {
    m_color = settings_values.m_color;
  }

  if ( 0 != (ON_Layer::plot_color_settings & settings) )
  {
    m_plot_color = settings_values.m_plot_color;
  }

  if ( 0 != (ON_Layer::plot_weight_settings & settings) )
  {
    m_plot_weight_mm = settings_values.m_plot_weight_mm;
  }

  if ( 0 != (ON_Layer::visible_settings & settings) )
  {
    m_bVisible = settings_values.m_bVisible ? true : false;
  }

  if ( 0 != (ON_Layer::locked_settings & settings) )
  {
    m_bLocked = settings_values.m_bLocked ? true : false;
  }
}

class /*NEVER EXPORT THIS CLASS DEFINITION*/ ON__LayerSettingsUserData : public ON_UserData
{
#if !defined(ON_BOZO_VACCINE_BFB63C094BC7472789BB7CC754118200)
#error Never copy this class definition or put this definition in a header file!
#endif
  ON_OBJECT_DECLARE(ON__LayerSettingsUserData);

public:
  ON__LayerSettingsUserData();
  ~ON__LayerSettingsUserData();
  // default copy constructor and operator= work fine.

  static ON__LayerSettingsUserData* LayerSettings(const ON_Layer& layer,bool bCreate);


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

  enum 
  {
    valid_settings = 
      (
          ON_Layer::color_settings
        | ON_Layer::plot_color_settings
        | ON_Layer::visible_settings
        | ON_Layer::locked_settings
        | ON_Layer::plot_weight_settings
      )
  };

  bool HaveSettings() const {return 0 != (m_settings & ON__LayerSettingsUserData::valid_settings);}

  bool HaveColor() const {return 0 != (m_settings & ON_Layer::color_settings);}
  bool HavePlotColor() const {return 0 != (m_settings & ON_Layer::plot_color_settings);}
  bool HaveVisible() const {return 0 != (m_settings & ON_Layer::visible_settings);}
  bool HaveLocked() const {return 0 != (m_settings & ON_Layer::locked_settings);}
  bool HavePlotWeight() const {return 0 != (m_settings & ON_Layer::plot_weight_settings);}

  void Defaults()
  {
    m_settings = 0;
    m_color = 0;
    m_plot_color = 0;
    m_bVisible = 0;
    m_bLocked = 0;
    m_plot_weight_mm = 0.0;
  }

  unsigned int m_settings;
  ON_Color m_color;
  ON_Color m_plot_color;
  bool m_bVisible;
  bool m_bLocked;
  double m_plot_weight_mm;

};

#undef ON_BOZO_VACCINE_BFB63C094BC7472789BB7CC754118200

ON_OBJECT_IMPLEMENT(ON__LayerSettingsUserData,ON_UserData,"BFB63C09-4BC7-4727-89BB-7CC754118200");

ON__LayerSettingsUserData* ON__LayerSettingsUserData::LayerSettings(const ON_Layer& layer,bool bCreate)
{
  ON__LayerSettingsUserData* ud = ON__LayerSettingsUserData::Cast(layer.GetUserData(ON__LayerSettingsUserData::m_ON__LayerSettingsUserData_class_id.Uuid()));
  if ( !ud && bCreate )
  {
    ud = new ON__LayerSettingsUserData();
    const_cast<ON_Layer&>(layer).AttachUserData(ud);
  }
  return ud;
}

ON__LayerSettingsUserData::ON__LayerSettingsUserData()
{
  m_userdata_uuid = ON__LayerSettingsUserData::m_ON__LayerSettingsUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs_id;
  m_userdata_copycount = 1;
  Defaults();
}

ON__LayerSettingsUserData::~ON__LayerSettingsUserData()
{
}

// virtual ON_Object override
ON_BOOL32 ON__LayerSettingsUserData::IsValid( ON_TextLog* text_log ) const
{
  return true;
}

// virtual ON_Object override
unsigned int ON__LayerSettingsUserData::SizeOf() const
{
  return (unsigned int)(sizeof(*this));
}

// virtual ON_Object override
ON__UINT32 ON__LayerSettingsUserData::DataCRC(ON__UINT32 current_remainder) const
{
  ON__UINT32 crc = current_remainder;
  crc = ON_CRC32(crc,sizeof(m_settings),&m_settings);
  if ( HaveColor() ) crc = ON_CRC32(crc,sizeof(m_color),&m_color);
  if ( HavePlotColor() ) crc = ON_CRC32(crc,sizeof(m_plot_color),&m_plot_color);
  if ( HaveVisible() ) crc = ON_CRC32(crc,sizeof(m_bVisible),&m_bVisible);
  if ( HaveLocked() ) crc = ON_CRC32(crc,sizeof(m_bLocked),&m_bLocked);
  if ( HavePlotWeight() ) crc = ON_CRC32(crc,sizeof(m_plot_weight_mm),&m_plot_weight_mm);
  return crc;
}

// virtual ON_Object override
ON_BOOL32 ON__LayerSettingsUserData::Write(ON_BinaryArchive& binary_archive) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;

  rc = false;
  for(;;)
  {
    if ( !binary_archive.WriteInt(m_settings) )
      break;
    if ( HaveColor() && !binary_archive.WriteColor(m_color) )
      break;
    if ( HavePlotColor() && !binary_archive.WriteColor(m_plot_color) )
      break;
    if ( HaveVisible() && !binary_archive.WriteBool(m_bVisible) )
      break;
    if ( HaveLocked() && !binary_archive.WriteBool(m_bLocked) )
      break;
    if ( HavePlotWeight() && !binary_archive.WriteDouble(m_plot_weight_mm) )
      break;

    rc = true;
    break;
  }

  if ( !binary_archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_Object override
ON_BOOL32 ON__LayerSettingsUserData::Read(ON_BinaryArchive& binary_archive)
{
  Defaults();

  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  rc = false;
  while ( 1 == major_version )
  {
    if ( !binary_archive.ReadInt(&m_settings) )
      break;
    if ( HaveColor() && !binary_archive.ReadColor(m_color) )
      break;
    if ( HavePlotColor() && !binary_archive.ReadColor(m_plot_color) )
      break;
    if ( HaveVisible() && !binary_archive.ReadBool(&m_bVisible) )
      break;
    if ( HaveLocked() && !binary_archive.ReadBool(&m_bLocked) )
      break;
    if ( HavePlotWeight() && !binary_archive.ReadDouble(&m_plot_weight_mm) )
      break;
    rc = true;
    break;
  }

  if ( !binary_archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

// virtual ON_UserData override
ON_BOOL32 ON__LayerSettingsUserData::Archive() const
{
  // don't save empty settings
  return HaveSettings();
}

// virtual ON_UserData override
ON_BOOL32 ON__LayerSettingsUserData::GetDescription( ON_wString& description )
{
  description = L"Saved Layer Settings";
  return true;
}



void ON_Layer::SaveSettings( unsigned int settings, bool bUpdate )
{
  ON__LayerSettingsUserData* ud;
  if ( 0 == (settings & ON__LayerSettingsUserData::valid_settings) )
  {
    if ( !bUpdate )
    {
      // delete any existing user data
      ud = ON__LayerSettingsUserData::LayerSettings(*this,false);
      if ( ud )
      {
        delete ud;
        ud = 0;
      }
    }
  }
  else
  {
    ud = ON__LayerSettingsUserData::LayerSettings(*this,true);
    if ( !bUpdate )
    {
      ud->Defaults();
      ud->m_settings = settings;
    }
    else
    {
      ud->m_settings |= settings;
    }
    if ( ud->HaveColor() )
      ud->m_color = m_color;
    if ( ud->HavePlotColor() )
      ud->m_plot_color = m_plot_color;
    if ( ud->HaveVisible() )
      ud->m_bVisible = m_bVisible;
    if ( ud->HaveLocked() )
      ud->m_bLocked = m_bLocked;
    if ( ud->HavePlotWeight() )
      ud->m_plot_weight_mm = m_plot_weight_mm;
  }
}

unsigned int ON_Layer::SavedSettings() const
{
  const ON__LayerSettingsUserData* ud = ON__LayerSettingsUserData::LayerSettings(*this,false);
  return ( 0 != ud ? ud->m_settings : 0 );
}

bool ON_Layer::GetSavedSettings( ON_Layer& layer, unsigned int& settings ) const
{
  const ON__LayerSettingsUserData* ud = ON__LayerSettingsUserData::LayerSettings(*this,false);
  if ( 0 == ud )
    return false;
  bool rc = false;

  if ( ud->HaveColor() )
  {
    layer.m_color = ud->m_color;
    rc = true;
  }

  if ( ud->HavePlotColor() )
  {
    layer.m_plot_color = ud->m_plot_color;
    rc = true;
  }

  if ( ud->HaveVisible() )
  {
    layer.m_bVisible = ud->m_bVisible;
    rc = true;
  }

  if ( ud->HaveLocked() )
  {
    layer.m_bLocked = ud->m_bLocked;
    rc = true;
  }

  if ( ud->HavePlotWeight() )
  {
    layer.m_plot_weight_mm = ud->m_plot_weight_mm;
    rc = true;
  }

  return true;
}


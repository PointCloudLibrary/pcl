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

/*
Changes and additions 5/01/07 LW
Adding several fields to ON_Dimstyle
Adding the concept of Parent and Child dimstyles so that individual dimension objects
can have their own copy of a dimension style to override some settings

  Adding several fields to ON_Dimstyle - This is done with ON_DimStyleExtra userdata
  class for now so the SDK doesn't break completely. When the SDK changes, the data in 
  ON_DimstyleExtra should be moved into ON_Dimstyle.

  Adding the concept of Parent and Child dimstyles to support per object overrides of
  dimstyle based properties.  Individual dimensions will be able to have one or more
  properties that differ from the dimension style for that dimension, but the rest of 
  their properties will be controlled by the parent dimstyle. In this implementation 
  (Rhino 5), dimstyles will only inherit one level deep.

  The first time an individual dimension has a dimstyle value overridden, a new child 
  dimstyle is made that is a copy of the dimension's dimstyle. If there is already a 
  child dimstyle for that dimension, it is used and no new dimstyle is made.
  The value being overridden is changed in the child dimstyle and a flag is set that 
  the field is being overridden.

  When a value is changed in a parent dimstyle, it should look through the other 
  dimstyles in the dimstyle table (or your application's equivalent) and change any 
  of its children appropriately.  Name and Index fields aren't propogated this way.
  If the parent's field being changed is not set in the child's m_valid_fields array,
  the child's copy of that field should be changed to match the parent's new value.
  Changing values in child dimstyles doesn't change values in their parents.

  When a value that has previously been overridden by an individual dimension
  is set to ByStyle, the corresponding field flag is unset in the valid field array. 
  If all of the flags in a child dimstyle become unset, the dimension is set to 
  reference the parent dimstyle directly.
  
*/


// Added for v5 - 5/01/07 LW
// Userdata class being used to extend ON_DimStyle so the v4 sdk still works
// Presumably, this will be moved into ON_DimStyle when the SDK is changed again
// Don't put this extension class in a header file or export it.
class ON_DimStyleExtra : public ON_UserData
{
  ON_OBJECT_DECLARE(ON_DimStyleExtra);
public:
  //////// 26 Oct 2010 - Changed to always create ON_DimStyleExtra
  //////static ON_DimStyleExtra* DimStyleExtension( ON_DimStyle* pDimStyle);
  //////static const ON_DimStyleExtra* DimStyleExtension( const ON_DimStyle* pDimStyle);

  // 2 November 2011 Dale Lear
  //   Undoing above change.  Adding this user data is not necessary
  //   and is causing large memory leaks when it is repeatedly
  //   added to the default dimstyle in the Rhino dimstyle table.
  static ON_DimStyleExtra* DimStyleExtensionGet( ON_DimStyle* pDimStyle, bool bCreateIfNoneExists );

  // 2 November 2011 Dale Lear
  //   Undoing above change.  Adding this user data is not necessary
  //   and is causing large memory leaks when it is repeatedly
  //   added to the default dimstyle in the Rhino dimstyle table.
  //   This const version never creates user data.
  static const ON_DimStyleExtra* DimStyleExtensionGet( const ON_DimStyle* pDimStyle);

  ON_DimStyleExtra();
  ~ON_DimStyleExtra();

  void SetDefaults();

  /*
  Returns:
    True if this ON_DimStyleExtra has default settings.
  */
  bool IsDefault() const;

  // override virtual ON_Object::Dump function
  void Dump( ON_TextLog& text_log ) const;

  // override virtual ON_Object::SizeOf function
  unsigned int SizeOf() const;

  // override virtual ON_Object::Write function
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;

  // override virtual ON_Object::Read function
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);

  // override virtual ON_UserData::GetDescription function
  ON_BOOL32 GetDescription( ON_wString& description );

  // override virtual ON_UserData::Archive function
  ON_BOOL32 Archive() const; 

  void SetFieldOverride( int field_id, bool bOverride);
  bool IsFieldOverride( int field_id) const;

  // Data access
  // Scale all of the length values
  void Scale( double scale);

  // Tolerances
  // Tolerance style
  //  0: None
  //  1: Symmetrical
  //  2: Deviation
  //  3: Limits
  //  4: Basic
  void SetToleranceStyle( int style);
  int  ToleranceStyle() const;
  
  void SetToleranceResolution( int resolution);
  int  ToleranceResolution() const;

  void SetToleranceUpperValue( double upper_value);
  double ToleranceUpperValue() const;

  void SetToleranceLowerValue( double lower_value);
  double ToleranceLowerValue() const;

  void SetToleranceHeightScale( double scale);
  double ToleranceHeightScale() const;

  void SetBaselineSpacing( double);
  double BaselineSpacing() const;

  // Determines whether or not to draw a Text Mask
  bool DrawTextMask() const;
  void SetDrawTextMask(bool bDraw);

  // Determines where to get the color to draw a Text Mask
  // 0: Use background color of the viewport.  Initially, gradient backgrounds will not be supported
  // 1: Use the ON_Color returned by MaskColor()
  int MaskColorSource() const;
  void SetMaskColorSource(int source);

  ON_Color MaskColor() const;  // Only works right if MaskColorSource returns 1.
                               // Does not return viewport background color
  void SetMaskColor(ON_Color color);

  void SetDimScale(double scale);
  double DimScale() const;
  void SetDimScaleSource(int source);
  int DimScaleSource() const;

  void SetSourceDimstyle(ON_UUID source_uuid);
  ON_UUID SourceDimstyle() const;

  bool CompareFields(const ON_DimStyleExtra* pOther) const;

  // Data storage

  ON_UUID m_parent_dimstyle;  // ON_nil_uuid if there is no parent dimstyle
  ON_SimpleArray<bool> m_valid_fields;
  enum { eFieldCount = 66 };

  int    m_tolerance_style;
  int    m_tolerance_resolution;
  double m_tolerance_upper_value;   // or both upper and lower in symmetrical style
  double m_tolerance_lower_value;
  double m_tolerance_height_scale;  // relative to the main dimension text

  double m_baseline_spacing;

  // Text mask - added Dec 12 2009
  bool     m_bDrawMask;
  int      m_mask_color_source;
  ON_Color m_mask_color;

  // Per dimstyle DimScale added Dec 16, 2009
  double   m_dimscale;
  int      m_dimscale_source;

  // 19 Oct 2010 - Added uuid of source dimstyle to restore defaults
  ON_UUID  m_source_dimstyle;
};





// Added for v5 - 5/01/07 LW
ON_OBJECT_IMPLEMENT(ON_DimStyleExtra,ON_UserData,"513FDE53-7284-4065-8601-06CEA8B28D6F");

////// 26 Oct 2010 - Lowell - Changed to always create ON_DimStyleExtra if there's not one
////ON_DimStyleExtra* ON_DimStyleExtra::DimStyleExtension( ON_DimStyle* pDimStyle)
////{
////  ON_DimStyleExtra* pExtra = 0;
////  if( pDimStyle)
////  {
////    pExtra = ON_DimStyleExtra::Cast( pDimStyle->GetUserData( ON_DimStyleExtra::m_ON_DimStyleExtra_class_id.Uuid()));
////    if( pExtra == 0)
////    {
////      pExtra = new ON_DimStyleExtra;
////      if( pExtra)
////      {
////        if( !pDimStyle->AttachUserData( pExtra))
////        {
////          delete pExtra;
////          pExtra = 0;
////        }
////      }
////    }
////  }
////  return pExtra;
////}

// 26 Oct 2010 - Lowell - Changed to always create ON_DimStyleExtra if there's not one
ON_DimStyleExtra* ON_DimStyleExtra::DimStyleExtensionGet( ON_DimStyle* pDimStyle, bool bCreateIfNoneExists )
{
  ON_DimStyleExtra* pExtra = 0;
  if( pDimStyle)
  {
    pExtra = ON_DimStyleExtra::Cast( pDimStyle->GetUserData( ON_DimStyleExtra::m_ON_DimStyleExtra_class_id.Uuid()));
    // 2 November 2011 Dale Lear
    //   I added the bCreateIfNoneExists parameter and I'm using
    //   is sparingly.  It is critical that we do not add
    //   ON_DimStyleExtra unless it is actually being used
    //   to override a default setting.  Otherwise, we 
    //   end up leaking vast amounts of memory when
    //   the default dimstyle in the Rhino dimstyle
    //   table is used due to the way annotation
    //   is currently drawn.
    //   If you have questions, please ask Dale Lear for details
    //   but please do not revert to constantly adding user
    //   data to dimstyles.
    if( pExtra == 0 && bCreateIfNoneExists )
    {
      pExtra = new ON_DimStyleExtra;
      if( pExtra)
      {
        if( !pDimStyle->AttachUserData( pExtra))
        {
          delete pExtra;
          pExtra = 0;
        }
      }
    }
  }
  return pExtra;
}

const 
ON_DimStyleExtra* ON_DimStyleExtra::DimStyleExtensionGet( const ON_DimStyle* pDimStyle)
{
  // Please do not changes the "false" to a "true" in the second argument.
  return ON_DimStyleExtra::DimStyleExtensionGet( (ON_DimStyle*)pDimStyle, false );
}

ON_DimStyleExtra::ON_DimStyleExtra()
{
  m_userdata_uuid = ON_DimStyleExtra::m_ON_DimStyleExtra_class_id.Uuid();
  m_application_uuid = ON_opennurbs5_id; // opennurbs.dll reads/writes this userdata
                                         // The id must be the version 5 id because
                                         // V6 SaveAs V5 needs to work, but SaveAs
                                         // V4 should not write this userdata.
  m_userdata_copycount = 1;
  m_valid_fields.Reserve( ON_DimStyleExtra::eFieldCount);
  m_valid_fields.SetCount( ON_DimStyleExtra::eFieldCount);
  m_parent_dimstyle = ON_nil_uuid;
  m_source_dimstyle = ON_nil_uuid;
  SetDefaults();
}

ON_DimStyleExtra::~ON_DimStyleExtra()
{
}

void ON_DimStyleExtra::SetDefaults()
{
  m_tolerance_style = ON_DimStyle::DefaultToleranceStyle();
  m_tolerance_resolution = ON_DimStyle::DefaultToleranceResolution();
  m_tolerance_upper_value = ON_DimStyle::DefaultToleranceUpperValue();
  m_tolerance_lower_value = ON_DimStyle::DefaultToleranceLowerValue();
  m_tolerance_height_scale = ON_DimStyle::DefaultToleranceHeightScale();
  m_baseline_spacing = ON_DimStyle::DefaultBaselineSpacing();
  m_bDrawMask = ON_DimStyle::DefaultDrawTextMask(); // false;
  m_mask_color_source = ON_DimStyle::DefaultMaskColorSource(); // 0;
  m_mask_color = ON_DimStyle::DefaultMaskColor(); // .SetRGB(255,255,255);
  m_dimscale = ON_DimStyle::DefaultDimScale(); // 1.0;
  m_dimscale_source = ON_DimStyle::DefaultDimScaleSource(); // 0;

  for( int i = 0; i < m_valid_fields.Count(); i++)
    m_valid_fields[i] = false;
}

bool ON_DimStyleExtra::IsDefault() const
{
  if ( m_tolerance_style != ON_DimStyle::DefaultToleranceStyle() ) return false;
  if ( m_tolerance_resolution != ON_DimStyle::DefaultToleranceResolution() ) return false;
  if ( m_tolerance_upper_value != ON_DimStyle::DefaultToleranceUpperValue() ) return false;
  if ( m_tolerance_lower_value != ON_DimStyle::DefaultToleranceLowerValue() ) return false;
  if ( m_tolerance_height_scale != ON_DimStyle::DefaultToleranceHeightScale() ) return false;
  if ( m_baseline_spacing != ON_DimStyle::DefaultBaselineSpacing() ) return false;
  if ( m_bDrawMask != ON_DimStyle::DefaultDrawTextMask() ) return false;
  if ( m_mask_color_source != ON_DimStyle::DefaultMaskColorSource() ) return false;
  if ( m_mask_color != ON_DimStyle::DefaultMaskColor() ) return false;
  if ( m_dimscale != ON_DimStyle::DefaultDimScale() ) return false;
  if ( m_dimscale_source != ON_DimStyle::DefaultDimScaleSource() ) return false;

  // The m_valid_fields[] settings only matter when
  // m_parent_dimstyle is not zero.
  if ( !(m_parent_dimstyle == ON_nil_uuid) )
  {
    for( int i = 0; i < m_valid_fields.Count() && i < ON_DimStyleExtra::eFieldCount; i++)
    {
      if ( !m_valid_fields[i] )
        return false;
    }
  }

  return true;
}

void ON_DimStyleExtra::Dump( ON_TextLog& ) const
{
  // do nothing
}

unsigned int ON_DimStyleExtra::SizeOf() const
{
  unsigned int sz = ON_UserData::SizeOf();
  sz += sizeof(*this) - sizeof(ON_UserData);
  return sz;
}

ON_BOOL32 ON_DimStyleExtra::Write(ON_BinaryArchive& archive) const
{
//  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0); Changed to 1,1 for mask settings 12/12/09
//  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1); Changed to 1,2 for dimscale 12/17/09
//  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,2); Changed to 1,3 for source_dimstyle 10/19/10
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,3);

  if(rc) rc = archive.WriteUuid( m_parent_dimstyle);
  if(rc) rc = archive.WriteArray( m_valid_fields);

  if(rc) rc = archive.WriteInt(m_tolerance_style);
  if(rc) rc = archive.WriteInt(m_tolerance_resolution);

  if(rc) rc = archive.WriteDouble(m_tolerance_upper_value);
  if(rc) rc = archive.WriteDouble(m_tolerance_lower_value);
  if(rc) rc = archive.WriteDouble(m_tolerance_height_scale);

  // March 22, 2010 - Global DimStyle was obsoleted and moved into DimStyles
  // So now for writing older version files, its multiplied into all distance values
  if(archive.Archive3dmVersion() < 5)
  {
    if(rc) rc = archive.WriteDouble(m_baseline_spacing * m_dimscale);
  }
  else
  {
    if(rc) rc = archive.WriteDouble(m_baseline_spacing);
  }

  if(rc) rc = archive.WriteBool(m_bDrawMask);
  if(rc) rc = archive.WriteInt(m_mask_color_source);
  if(rc) rc = archive.WriteColor(m_mask_color);

  if(archive.Archive3dmVersion() < 5)
  {
    if(rc) rc = archive.WriteDouble(1.0);
  }
  else
  {
    if(rc) rc = archive.WriteDouble(m_dimscale);
  }
  if(rc) rc = archive.WriteInt(m_dimscale_source); // Obsolete field

  if(rc) rc = archive.WriteUuid(m_source_dimstyle);  // Added 19 Oct 2010

  if(!archive.EndWrite3dmChunk())
    rc = false;

  return rc;
}

ON_BOOL32 ON_DimStyleExtra::Read(ON_BinaryArchive& archive)
{
  // Changed to 1,0 for mask settings 12/12/09
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if(major_version != 1)
    rc = false;

  if(rc) rc = archive.ReadUuid(m_parent_dimstyle);
  if(rc) rc = archive.ReadArray(m_valid_fields);

  if(rc) rc = archive.ReadInt(&m_tolerance_style);
  if(rc) rc = archive.ReadInt(&m_tolerance_resolution);

  if(rc) rc = archive.ReadDouble(&m_tolerance_upper_value);
  if(rc) rc = archive.ReadDouble(&m_tolerance_lower_value);
  if(rc) rc = archive.ReadDouble(&m_tolerance_height_scale);

  if(rc) rc = archive.ReadDouble(&m_baseline_spacing);

  if(minor_version >= 1)
  {
    if(rc) rc = archive.ReadBool(&m_bDrawMask);
    if(rc) rc = archive.ReadInt(&m_mask_color_source);
    if(rc) rc = archive.ReadColor(m_mask_color);
  }

  if(minor_version >= 2)
  {
    if(rc) rc = archive.ReadDouble(&m_dimscale);
    if(rc) rc = archive.ReadInt(&m_dimscale_source);
  }

  if(minor_version >= 3)
  {
    if(rc) rc = archive.ReadUuid(m_source_dimstyle);
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

ON_BOOL32 ON_DimStyleExtra::GetDescription( ON_wString& description)
{
  description.Format( "Userdata extension of ON_DimStyle");
  return true;
}

ON_BOOL32 ON_DimStyleExtra::Archive() const
{
  // true to write to file
  return true;
}

void ON_DimStyleExtra::Scale( double scale)
{
  if( ON_IsValid( scale) && scale > ON_SQRT_EPSILON)
    m_baseline_spacing *= scale;
}

// Tolerance style
void ON_DimStyleExtra::SetToleranceStyle( int style)
{
  if( style >= 0 && style <= 4)
    m_tolerance_style = style;
}

int ON_DimStyleExtra::ToleranceStyle() const
{
  return m_tolerance_style;
}

void ON_DimStyleExtra::SetToleranceResolution( int resolution)
{
  if( resolution >= 0 && resolution < 16)
    m_tolerance_resolution = resolution;
}

int ON_DimStyleExtra::ToleranceResolution() const
{
  return m_tolerance_resolution;
}

void ON_DimStyleExtra::SetToleranceUpperValue( double upper_value)
{
  if( ON_IsValid(upper_value))
    m_tolerance_upper_value = upper_value;
}

double ON_DimStyleExtra::ToleranceUpperValue() const
{
  return m_tolerance_upper_value;
}

void ON_DimStyleExtra::SetToleranceLowerValue( double lower_value)
{
  if( ON_IsValid(lower_value))
    m_tolerance_lower_value = lower_value;
}

double ON_DimStyleExtra::ToleranceLowerValue() const
{
  return m_tolerance_lower_value;
}

void ON_DimStyleExtra::SetToleranceHeightScale( double scale)
{
  if( ON_IsValid( scale) && scale > ON_SQRT_EPSILON)
    m_tolerance_height_scale = scale;
}

double ON_DimStyleExtra::ToleranceHeightScale() const
{
  return m_tolerance_height_scale;
}

void ON_DimStyleExtra::SetBaselineSpacing( double spacing)
{
  if( ON_IsValid( spacing) && spacing > ON_SQRT_EPSILON)
    m_baseline_spacing = spacing;
}

double ON_DimStyleExtra::BaselineSpacing() const
{
  return m_baseline_spacing;
}

bool ON_DimStyleExtra::DrawTextMask() const
{
  return m_bDrawMask;
}

void ON_DimStyleExtra::SetDrawTextMask(bool bDraw)
{
  m_bDrawMask = bDraw ? true : false;
}

int ON_DimStyleExtra::MaskColorSource() const
{
  return m_mask_color_source;
}

void ON_DimStyleExtra::SetMaskColorSource(int source)
{
  if(source == 1)
    m_mask_color_source = 1;
  else
    m_mask_color_source = 0;
}

ON_Color ON_DimStyleExtra::MaskColor() const
{
  return m_mask_color;
}

void ON_DimStyleExtra::SetMaskColor(ON_Color color)
{
  m_mask_color = color;
}

void ON_DimStyleExtra::SetDimScale(double scale)
{
  m_dimscale = scale;
}

double ON_DimStyleExtra::DimScale() const
{
  return m_dimscale;
}

void ON_DimStyleExtra::SetDimScaleSource(int source)
{
  m_dimscale_source = source;
}

int ON_DimStyleExtra::DimScaleSource() const
{
  return m_dimscale_source;
}

void ON_DimStyleExtra::SetSourceDimstyle(ON_UUID source_uuid)
{
  m_source_dimstyle = source_uuid;
}

ON_UUID ON_DimStyleExtra::SourceDimstyle() const
{
  return m_source_dimstyle;
}

// returns true if they are the same
bool ON_DimStyleExtra::CompareFields(const ON_DimStyleExtra* pOther) const
{
  if(pOther == 0)
    return false;

  if((m_parent_dimstyle        != pOther->m_parent_dimstyle) ||
     (m_tolerance_style        != pOther->m_tolerance_style) ||
     (m_tolerance_resolution   != pOther->m_tolerance_resolution) ||
     (m_tolerance_upper_value  != pOther->m_tolerance_upper_value) ||
     (m_tolerance_lower_value  != pOther->m_tolerance_lower_value) ||
     (m_tolerance_height_scale != pOther->m_tolerance_height_scale) ||
     (m_baseline_spacing       != pOther->m_baseline_spacing) ||
     (m_bDrawMask              != pOther->m_bDrawMask) ||
     (m_mask_color_source      != pOther->m_mask_color_source) ||
     (m_mask_color             != pOther->m_mask_color) ||
     (m_dimscale               != pOther->m_dimscale) ||
     (m_dimscale_source        != pOther->m_dimscale_source)
     )
    return false;

  for(int i = 0; i < m_valid_fields.Count(); i++)
  {
    if(m_valid_fields[i] != pOther->m_valid_fields[i])
      return false;
  }
  return true;
}


ON_OBJECT_IMPLEMENT( ON_DimStyle, ON_Object, "81BD83D5-7120-41c4-9A57-C449336FF12C" );

ON_DimStyle::ON_DimStyle()
{
  // 26 Oct 2010
  // Can't create this here because reading files won't attach what's read from the file if 
  // there's already one there.
//  ON_DimStyleExtra::DimStyleExtension( this);
  SetDefaultsNoExtension();
}

ON_DimStyle::~ON_DimStyle()
{
}

void ON_DimStyle::SetDefaults()
{
  // If there is already a userdata extension, reset it to the defaults,
  // but don't make one if its not there already
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, false );
  if( pDE)
  {
    // 2 November 2011 Dale Lear
    //    The "default" settings are easily handled
    //    by not having the user data present in 
    //    the first place.  Please discuss changes
    //    with Dale Lear.
    //
    delete pDE;

    ////// reset all field override flags
    ////for( int i = 0; i < pDE->m_valid_fields.Count(); i++)
    ////  pDE->m_valid_fields[i] = false;
    ////pDE->SetDefaults();
  }
  SetDefaultsNoExtension();
}

// Need to be able to set defaults in base without creating extension
// because file reading won't attach userdata if there's already one there
void ON_DimStyle::SetDefaultsNoExtension()
{
  m_dimstyle_index = -1;
  memset(&m_dimstyle_id,0,sizeof(m_dimstyle_id));
  m_dimstyle_name = L"Default";

  m_extextension = 0.5;
  m_extoffset = 0.5;
  m_arrowsize = 1.0;
  m_centermark = 0.5;
  m_textgap = 0.25;
  m_textheight = 1.0;
  m_textalign = ON::dtAboveLine;
  m_arrowtype = 0;
  m_angularunits = 0;
  m_lengthformat = 0;
  m_angleformat = 0;
  m_lengthresolution = 2;
  m_angleresolution = 2;
  m_fontindex = -1;

  // Added at 1.3
  m_lengthfactor = 1.0;  
  m_bAlternate = false;
  m_alternate_lengthfactor = 25.4;
  m_alternate_lengthformat = 0;
  m_alternate_lengthresolution = 2;
  m_alternate_angleformat = 0;
  m_alternate_angleresolution = 2;

  m_prefix = L"";
  m_suffix = L"";
  m_alternate_prefix = L" [";
  m_alternate_suffix = L"]";
  m_valid = 0;

  m_dimextension = 0.0;

  m_leaderarrowsize = 1.0;
  m_leaderarrowtype = 0;
  m_bSuppressExtension1 = false;
  m_bSuppressExtension2 = false;
}


// copy from ON_3dmAnnotationSettings and add a couple of fields
ON_DimStyle& ON_DimStyle::operator=( const ON_3dmAnnotationSettings& src)
{
  SetDefaults();

  m_extextension = src.m_dimexe;
  m_extoffset = src.m_dimexo;
  m_arrowsize = src.m_arrowlength;
  m_textalign = src.m_textalign;
  m_centermark = src.m_centermark;
  m_textgap = src.m_dimexo / 2.0;
  m_textheight = src.m_textheight;
  m_arrowtype = src.m_arrowtype;
  m_angularunits = src.m_angularunits;
  m_lengthformat = src.m_lengthformat;
  m_angleformat = src.m_angleformat;
  m_lengthresolution = src.m_resolution;
  m_angleresolution = src.m_resolution;

  return *this;
}

//////////////////////////////////////////////////////////////////////
//
// ON_Object overrides

ON_BOOL32 ON_DimStyle::IsValid( ON_TextLog* ) const
{
  return ( m_dimstyle_name.Length() > 0 && m_dimstyle_index >= 0);
}

void ON_DimStyle::Dump( ON_TextLog& dump ) const
{
  const wchar_t* wsName = m_dimstyle_name;
  if ( !wsName )
    wsName = L"";
  dump.Print("dimstyle index = %d\n",m_dimstyle_index);
  dump.Print("dimstyle name = \"%ls\"\n",wsName);
}

ON_BOOL32 ON_DimStyle::Write(
       ON_BinaryArchive& file // serialize definition to binary archive
     ) const
{
  // March 22, 2010 - Global DimStyle was obsoleted and moved into DimStyles
  // So now for writing older version files, its multiplied into all distance values
  double ds = 1.0;
  if(file.Archive3dmVersion() < 5)
    ds = DimScale();

  // changed to version 1.4 Dec 28, 05
  // changed to version 1.5 Mar 23, 06
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,5);

  if (rc) rc = file.WriteInt(m_dimstyle_index);
  if (rc) rc = file.WriteString(m_dimstyle_name);

  if (rc) rc = file.WriteDouble(m_extextension * ds);
  if (rc) rc = file.WriteDouble(m_extoffset * ds);
  if (rc) rc = file.WriteDouble(m_arrowsize * ds);
  if (rc) rc = file.WriteDouble(m_centermark * ds);
  if (rc) rc = file.WriteDouble(m_textgap * ds);
  
  if (rc) rc = file.WriteInt(m_textalign);
  if (rc) rc = file.WriteInt(m_arrowtype);
  if (rc) rc = file.WriteInt(m_angularunits);
  if (rc) rc = file.WriteInt(m_lengthformat);
  if (rc) rc = file.WriteInt(m_angleformat);
  if (rc) rc = file.WriteInt(m_lengthresolution);
  if (rc) rc = file.WriteInt(m_angleresolution);
  if (rc) rc = file.WriteInt(m_fontindex);

  if (rc) rc = file.WriteDouble(m_textheight * ds);

  // added 1/13/05 ver 1.2
  if (rc) rc = file.WriteDouble(m_lengthfactor);
  if (rc) rc = file.WriteString(m_prefix);
  if (rc) rc = file.WriteString(m_suffix);

  if (rc) rc = file.WriteBool(m_bAlternate);
  if (rc) rc = file.WriteDouble(m_alternate_lengthfactor);
  if (rc) rc = file.WriteInt(m_alternate_lengthformat);
  if (rc) rc = file.WriteInt(m_alternate_lengthresolution);
  if (rc) rc = file.WriteInt(m_alternate_angleformat);
  if (rc) rc = file.WriteInt(m_alternate_angleresolution);
  if (rc) rc = file.WriteString(m_alternate_prefix);
  if (rc) rc = file.WriteString(m_alternate_suffix);
  if (rc) rc = file.WriteInt(m_valid);

  // Added 24 October 2005 ver 1.3
  if (rc) rc = file.WriteUuid(m_dimstyle_id);

  // Added Dec 28, 05 ver 1.4
  if (rc) rc = file.WriteDouble(m_dimextension * ds);

  // Added Mar 23 06 ver 1.5
  if (rc) rc = file.WriteDouble(m_leaderarrowsize * ds);
  if (rc) rc = file.WriteInt(m_leaderarrowtype);
  if (rc) rc = file.WriteBool(m_bSuppressExtension1);
  if (rc) rc = file.WriteBool(m_bSuppressExtension2);

  return rc;
}

ON_BOOL32 ON_DimStyle::Read(
       ON_BinaryArchive& file // restore definition from binary archive
     )
{
  SetDefaultsNoExtension();

  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);


  if ( major_version >= 1 ) 
  {
    if ( rc) rc = file.ReadInt( &m_dimstyle_index);
    if ( rc) rc = file.ReadString( m_dimstyle_name);
    
    if ( rc) rc = file.ReadDouble( &m_extextension);
    if ( rc) rc = file.ReadDouble( &m_extoffset);
    if ( rc) rc = file.ReadDouble( &m_arrowsize);
    if ( rc) rc = file.ReadDouble( &m_centermark);
    if ( rc) rc = file.ReadDouble( &m_textgap);
    
    if ( rc) rc = file.ReadInt( &m_textalign);
    if ( rc) rc = file.ReadInt( &m_arrowtype);
    if ( rc) rc = file.ReadInt( &m_angularunits);
    if ( rc) rc = file.ReadInt( &m_lengthformat);
    if ( rc) rc = file.ReadInt( &m_angleformat);
    if ( rc) rc = file.ReadInt( &m_lengthresolution);
    if ( rc) rc = file.ReadInt( &m_angleresolution);
    if ( rc) rc = file.ReadInt( &m_fontindex);

    if( minor_version >= 1)
      if ( rc) rc = file.ReadDouble( &m_textheight);

    // added 1/13/05
    if( minor_version >= 2)
    {
      if (rc) rc = file.ReadDouble( &m_lengthfactor);
      if (rc) rc = file.ReadString( m_prefix);
      if (rc) rc = file.ReadString( m_suffix);

      if (rc) rc = file.ReadBool( &m_bAlternate);
      if (rc) rc = file.ReadDouble(&m_alternate_lengthfactor);
      if (rc) rc = file.ReadInt( &m_alternate_lengthformat);
      if (rc) rc = file.ReadInt( &m_alternate_lengthresolution);
      if (rc) rc = file.ReadInt( &m_alternate_angleformat);
      if (rc) rc = file.ReadInt( &m_alternate_angleresolution);
      if (rc) rc = file.ReadString( m_alternate_prefix);
      if (rc) rc = file.ReadString( m_alternate_suffix);
      if (rc) rc = file.ReadInt( &m_valid);

      if ( minor_version >= 3 )
      {
        if (rc) rc = file.ReadUuid(m_dimstyle_id);
      }
    }
    // Added Dec 28, 05 ver 1.4
    if( minor_version >= 4)
      if( rc) rc = file.ReadDouble( &m_dimextension);

    // Added Mar 23 06 ver 1.5
    if( minor_version >= 5)
    {
      if (rc) rc = file.ReadDouble( &m_leaderarrowsize);
      if (rc) rc = file.ReadInt( &m_leaderarrowtype);
      if (rc) rc = file.ReadBool( &m_bSuppressExtension1);
      if (rc) rc = file.ReadBool( &m_bSuppressExtension2);
    }
  }
  else
    rc = false;

  return rc;
}

void ON_DimStyle::EmergencyDestroy()
{
  m_prefix.EmergencyDestroy();
  m_suffix.EmergencyDestroy();
  m_alternate_prefix.EmergencyDestroy();
  m_alternate_suffix.EmergencyDestroy();
}

//////////////////////////////////////////////////////////////////////
//
// Interface
void ON_DimStyle::SetName( const wchar_t* s )
{
  m_dimstyle_name = s;
  m_dimstyle_name.TrimLeftAndRight();
}

void ON_DimStyle::SetName( const char* s )
{
  m_dimstyle_name = s;
  m_dimstyle_name.TrimLeftAndRight();
}

void ON_DimStyle::GetName( ON_wString& s ) const
{
  s = m_dimstyle_name;
}

const wchar_t* ON_DimStyle::Name() const
{
  const wchar_t* s = m_dimstyle_name;
  return s;
}

void ON_DimStyle::SetIndex(int i )
{
  m_dimstyle_index = i;
}

int ON_DimStyle::Index() const
{
  return m_dimstyle_index;
}

double ON_DimStyle::ExtExtension() const
{
  return m_extextension;
}

void ON_DimStyle::SetExtExtension( const double e)
{
  // Allow negative?
  m_extextension = e;
}

double ON_DimStyle::ExtOffset() const
{
  return m_extoffset;
}

void ON_DimStyle::SetExtOffset( const double e)
{
  m_extoffset = e;
}

double ON_DimStyle::ArrowSize() const
{
  return m_arrowsize;
}

void ON_DimStyle::SetArrowSize( const double e)
{
  m_arrowsize = e;
}

double ON_DimStyle::LeaderArrowSize() const
{
  return m_leaderarrowsize;
}

void ON_DimStyle::SetLeaderArrowSize( const double e)
{
  m_leaderarrowsize = e;
}

double ON_DimStyle::CenterMark() const
{
  return m_centermark;
}

void ON_DimStyle::SetCenterMark( const double e)
{
  m_centermark = e;
}

int ON_DimStyle::TextAlignment() const
{
  return m_textalign;
}

void ON_DimStyle::SetTextAlignment( ON::eTextDisplayMode a)
{
  m_textalign = a;
}

int ON_DimStyle::ArrowType() const
{
  return m_arrowtype;
}

void ON_DimStyle::SetArrowType( eArrowType a)
{
  m_arrowtype = a;
}

int ON_DimStyle::LeaderArrowType() const
{
  return m_leaderarrowtype;
}

void ON_DimStyle::SetLeaderArrowType( eArrowType a)
{
  m_leaderarrowtype = a;
}

int ON_DimStyle::AngularUnits() const
{
  return m_angularunits;
}

void ON_DimStyle::SetAngularUnits( int u)
{
  m_angularunits = u;
}

int ON_DimStyle::LengthFormat() const
{
  return m_lengthformat;
}

void ON_DimStyle::SetLengthFormat( int f)
{
  m_lengthformat = f;
}

int ON_DimStyle::AngleFormat() const
{
  return m_angleformat;
}

void ON_DimStyle::SetAngleFormat( int f)
{
  m_angleformat = f;
}

int ON_DimStyle::LengthResolution() const
{
  return m_lengthresolution;
}

void ON_DimStyle::SetLengthResolution( int r)
{
  if( r >= 0 && r < 16)
  {
    m_lengthresolution = r;
  }
}

int ON_DimStyle::AngleResolution() const
{
  return m_angleresolution;
}

void ON_DimStyle::SetAngleResolution( int r)
{
  if( r >= 0 && r < 16)
  {
    m_angleresolution = r;
  }
}

int ON_DimStyle::FontIndex() const
{
  return m_fontindex;
}

void ON_DimStyle::SetFontIndex( int index)
{
  m_fontindex = index;
}

double ON_DimStyle::TextGap() const
{
  return m_textgap;
}

void ON_DimStyle::SetTextGap( double gap)
{
  if( gap >= 0.0)
  {
    m_textgap = gap;
  }
}

double ON_DimStyle::TextHeight() const
{
  return m_textheight;
}

void ON_DimStyle::SetTextHeight( double height)
{
  if( ON_IsValid( height) && height > ON_SQRT_EPSILON)
  {
    m_textheight = height;
  }
}


double ON_DimStyle::LengthFactor() const
{
  return m_lengthfactor;
}
void ON_DimStyle::SetLengthactor( double factor)
{
  SetLengthFactor( factor);
}
void ON_DimStyle::SetLengthFactor( double factor)
{
  m_lengthfactor = factor;
  //ValidateField( fn_lengthfactor);
  m_valid |= ( 1 << fn_lengthfactor);
}

bool ON_DimStyle::Alternate() const
{
  return m_bAlternate;
}
void ON_DimStyle::SetAlternate( bool bAlternate)
{
  m_bAlternate = bAlternate;
}

double ON_DimStyle::AlternateLengthFactor() const
{
  return m_alternate_lengthfactor;
}
void ON_DimStyle::SetAlternateLengthactor( double factor)
{
  SetAlternateLengthFactor( factor);
}
void ON_DimStyle::SetAlternateLengthFactor( double factor)
{
  m_alternate_lengthfactor = factor;
  //ValidateField( fn_alternate_lengthfactor);
  m_valid |= ( 1 << fn_alternate_lengthfactor);
}

int ON_DimStyle::AlternateLengthFormat() const
{
  return m_alternate_lengthformat;
}
void ON_DimStyle::SetAlternateLengthFormat( int format)
{
  m_alternate_lengthformat = format;
}

int ON_DimStyle::AlternateLengthResolution() const
{
  return m_alternate_lengthresolution;
}
void ON_DimStyle::SetAlternateLengthResolution( int resolution)
{
  m_alternate_lengthresolution = resolution;
}

int ON_DimStyle::AlternateAngleFormat() const
{
  return m_alternate_angleformat;
}
void ON_DimStyle::SetAlternateAngleFormat( int format)
{
  m_alternate_angleformat = format;
}

int ON_DimStyle::AlternateAngleResolution() const
{
  return m_alternate_angleresolution;
}
void ON_DimStyle::SetAlternateAngleResolution( int resolution)
{
  m_alternate_angleresolution = resolution;
}

void ON_DimStyle::GetPrefix( ON_wString& prefix) const
{
  prefix = m_prefix;
}
const wchar_t* ON_DimStyle::Prefix() const
{
  return m_prefix;
}
void ON_DimStyle::SetPrefix( wchar_t* prefix)
{
  m_prefix = prefix;
}
void ON_DimStyle::SetPrefix( const wchar_t* prefix)
{
  m_prefix = prefix;
}

void ON_DimStyle::GetSuffix( ON_wString& suffix) const
{
  suffix = m_suffix;
}
const wchar_t* ON_DimStyle::Suffix() const
{
  return m_suffix;
}
void ON_DimStyle::SetSuffix( wchar_t* suffix)
{
  m_suffix = suffix;
}
void ON_DimStyle::SetSuffix( const wchar_t* suffix)
{
  m_suffix = suffix;
}

void ON_DimStyle::GetAlternatePrefix( ON_wString& prefix) const
{
  prefix = m_alternate_prefix;
}
const wchar_t* ON_DimStyle::AlternatePrefix() const
{
  return m_alternate_prefix;
}
void ON_DimStyle::SetAlternatePrefix( wchar_t* prefix)
{
  m_alternate_prefix = prefix;
}
void ON_DimStyle::SetAlternatePrefix( const wchar_t* prefix)
{
  m_alternate_prefix = prefix;
}

void ON_DimStyle::GetAlternateSuffix( ON_wString& suffix) const
{
  suffix = m_alternate_suffix;
}
const wchar_t* ON_DimStyle::AlternateSuffix() const
{
  return m_alternate_suffix;
}
void ON_DimStyle::SetAlternateSuffix( wchar_t* suffix)
{
  m_alternate_suffix = suffix;
}
void ON_DimStyle::SetAlternateSuffix( const wchar_t* suffix)
{
  m_alternate_suffix = suffix;
}

bool ON_DimStyle::SuppressExtension1() const
{
  return m_bSuppressExtension1;
}
void ON_DimStyle::SetSuppressExtension1( bool suppress)
{
  m_bSuppressExtension1 = suppress;
}
bool ON_DimStyle::SuppressExtension2() const
{
  return m_bSuppressExtension2;
}
void ON_DimStyle::SetSuppressExtension2( bool suppress)
{
  m_bSuppressExtension2 = suppress;
}

// This function deprecated 5/01/07 LW
void ON_DimStyle::Composite( const ON_DimStyle& /*OverRide*/)
{
/*
  InvalidateAllFields();

  if( OverRide.IsFieldValid( fn_name))                          { ValidateField( fn_name); m_dimstyle_name = OverRide.m_dimstyle_name; }
  if( OverRide.IsFieldValid( fn_index))                         { ValidateField( fn_index); m_dimstyle_index = OverRide.m_dimstyle_index; }
  if( OverRide.IsFieldValid( fn_extextension))                  { ValidateField( fn_extextension); m_extextension = OverRide.m_extextension; }
  if( OverRide.IsFieldValid( fn_extoffset))                     { ValidateField( fn_extoffset); m_extoffset = OverRide.m_extoffset; }
  if( OverRide.IsFieldValid( fn_arrowsize))                     { ValidateField( fn_arrowsize); m_arrowsize = OverRide.m_arrowsize; }
  if( OverRide.IsFieldValid( fn_leaderarrowsize))               { ValidateField( fn_leaderarrowsize); m_leaderarrowsize = OverRide.m_leaderarrowsize; }
  if( OverRide.IsFieldValid( fn_centermark))                    { ValidateField( fn_centermark); m_centermark = OverRide.m_centermark; }
  if( OverRide.IsFieldValid( fn_textgap))                       { ValidateField( fn_textgap); m_textgap = OverRide.m_textgap; }
  if( OverRide.IsFieldValid( fn_textheight))                    { ValidateField( fn_textheight); m_textheight = OverRide.m_textheight; }
  if( OverRide.IsFieldValid( fn_textalign))                     { ValidateField( fn_textalign); m_textalign = OverRide.m_textalign; }
  if( OverRide.IsFieldValid( fn_arrowtype))                     { ValidateField( fn_arrowtype); m_arrowtype = OverRide.m_arrowtype; }
  if( OverRide.IsFieldValid( fn_leaderarrowtype))               { ValidateField( fn_leaderarrowtype); m_leaderarrowtype = OverRide.m_leaderarrowtype; }
  if( OverRide.IsFieldValid( fn_angularunits))                  { ValidateField( fn_angularunits); m_angularunits = OverRide.m_angularunits; }
  if( OverRide.IsFieldValid( fn_lengthformat))                  { ValidateField( fn_lengthformat); m_lengthformat = OverRide.m_lengthformat; }
  if( OverRide.IsFieldValid( fn_angleformat))                   { ValidateField( fn_angleformat); m_angleformat = OverRide.m_angleformat; }
  if( OverRide.IsFieldValid( fn_angleresolution))               { ValidateField( fn_angleresolution); m_angleresolution = OverRide.m_angleresolution; }
  if( OverRide.IsFieldValid( fn_lengthresolution))              { ValidateField( fn_lengthresolution); m_lengthresolution = OverRide.m_lengthresolution; }
  if( OverRide.IsFieldValid( fn_fontindex))                     { ValidateField( fn_fontindex); m_fontindex = OverRide.m_fontindex; }
  if( OverRide.IsFieldValid( fn_lengthfactor))                  { ValidateField( fn_lengthfactor); m_lengthfactor = OverRide.m_lengthfactor; }
  if( OverRide.IsFieldValid( fn_bAlternate))                    { ValidateField( fn_bAlternate); m_bAlternate = OverRide.m_bAlternate; }
  if( OverRide.IsFieldValid( fn_alternate_lengthfactor))        { ValidateField( fn_alternate_lengthfactor); m_alternate_lengthfactor = OverRide.m_alternate_lengthfactor; }
  if( OverRide.IsFieldValid( fn_alternate_lengthformat))        { ValidateField( fn_alternate_lengthformat); m_alternate_lengthformat = OverRide.m_alternate_lengthformat; }
  if( OverRide.IsFieldValid( fn_alternate_lengthresolution))    { ValidateField( fn_alternate_lengthresolution); m_alternate_lengthresolution = OverRide.m_alternate_lengthresolution; }
  if( OverRide.IsFieldValid( fn_prefix))                        { ValidateField( fn_prefix); m_prefix = OverRide.m_prefix; }
  if( OverRide.IsFieldValid( fn_suffix))                        { ValidateField( fn_suffix); m_suffix = OverRide.m_suffix; }
  if( OverRide.IsFieldValid( fn_alternate_prefix))              { ValidateField( fn_alternate_prefix); m_alternate_prefix = OverRide.m_alternate_prefix; }
  if( OverRide.IsFieldValid( fn_alternate_suffix))              { ValidateField( fn_alternate_suffix); m_alternate_suffix = OverRide.m_alternate_suffix; }
  if( OverRide.IsFieldValid( fn_dimextension))                  { ValidateField( fn_dimextension); m_dimextension = OverRide.m_dimextension; }
  if( OverRide.IsFieldValid( fn_suppressextension1))            { ValidateField( fn_suppressextension1); m_bSuppressExtension1 = OverRide.m_bSuppressExtension1; }
  if( OverRide.IsFieldValid( fn_suppressextension2))            { ValidateField( fn_suppressextension2); m_bSuppressExtension2 = OverRide.m_bSuppressExtension2; }
*/
}

// This function deprecated 5/01/07 LW
void ON_DimStyle::InvalidateField( eField field)
{
  m_valid &= ~( 1 << field);
}
// This function deprecated 5/01/07 LW
void ON_DimStyle::InvalidateAllFields()
{
  m_valid = 0;
}
// This function deprecated 5/01/07 LW
void ON_DimStyle::ValidateField( eField field)
{
  m_valid |= ( 1 << field);
}
// This function deprecated 5/01/07 LW
bool ON_DimStyle::IsFieldValid( eField field) const
{
  return ( m_valid & ( 1 << field)) ? true : false;
}


// ver 1.4, Dec 28, 05
double ON_DimStyle::DimExtension() const
{
  return m_dimextension;
}

void ON_DimStyle::SetDimExtension( const double e)
{
  // Allow negative?
  m_dimextension = e;
}

//--------------------------------------
// ON_DimStyleExtra access functions
// Added for v5 5/01/07 LW

bool ON_DimStyle::IsFieldOverride( ON_DimStyle::eField field_id) const
{
  const ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this);
  if(pDE)
    return pDE->IsFieldOverride( field_id);

  return false;
}

void ON_DimStyle::SetFieldOverride(  ON_DimStyle::eField field_id, bool bOverride)
{
  // 2 November 2011 Dale Lear
  //   If bOverride is true, then create the userdata, otherwise get it 
  //   only if it exists.
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bOverride );
  if(pDE)
  {
    pDE->SetFieldOverride( field_id, bOverride);
  }
}
  
bool ON_DimStyle::HasOverrides() const 
{
  const ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this );
  if(pDE)
  {
    for( int i = 0; i < pDE->m_valid_fields.Count(); i++)
    {
      if( pDE->m_valid_fields[i])
        return true;
    }
  }
  return false;
}

bool ON_DimStyle::OverrideFields( const ON_DimStyle& src, const ON_DimStyle& parent)
{
  // 2 November 2011 Dale Lear
  //   I made lots of changes to this function to make sure "i" was a valid
  //   array index before it was passed to operator[].
  const ON_DimStyleExtra* pDEsrc = ON_DimStyleExtra::DimStyleExtensionGet( &src);

  // Please do not change the 2nd parameter from "false" to a "true" in the
  // call to DimStyleExtensionGet().  If "this" does not have ON_DimStyleExtra
  // user data at this point, it will be created if it is actually needed.
  // If you have questions, please ask Dale Lear.
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, false );

  const int src_valid_fields_count = pDEsrc ? pDEsrc->m_valid_fields.Count() : 0;
  int this_valid_fields_count = pDE ? pDE->m_valid_fields.Count() : 0;

  for( int i = 0; i < ON_DimStyleExtra::eFieldCount; i++)
  {
    bool bValidSrcField = ( 0 != pDEsrc && i < src_valid_fields_count )
                        ? pDEsrc->m_valid_fields[i]
                        : false;

    if ( bValidSrcField && 0 == pDE )
    {
      // Actually need to create ON_DimStyleExtra user data on "this".
      pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, true );
      if ( pDE )
        this_valid_fields_count = pDE->m_valid_fields.Count();
    }

    if ( 0 != pDE && i < this_valid_fields_count )
    {
      pDE->m_valid_fields[i] = bValidSrcField;
    }

    const ON_DimStyle* copyfrom = bValidSrcField ? (&src) : (&parent);

    switch( i)
    {
    case fn_extextension:
      SetExtExtension( copyfrom->ExtExtension());
      break;
    case fn_extoffset:
      SetExtOffset( copyfrom->ExtOffset());
      break;
    case fn_arrowsize:
      SetArrowSize( copyfrom->ArrowSize());
      break;
    case fn_centermark:
      SetCenterMark( copyfrom->CenterMark());
      break;
    case fn_textgap:
      SetTextGap( copyfrom->TextGap());
      break;
    case fn_textheight:
      SetTextHeight( copyfrom->TextHeight());
      break;
    case fn_textalign:
      SetTextAlignment( (ON::eTextDisplayMode)copyfrom->TextAlignment());
      break;
    case fn_arrowtype:
      SetArrowType( (eArrowType)copyfrom->ArrowType());
      break;
    case fn_angularunits:
      SetAngularUnits( (eArrowType)copyfrom->AngularUnits());
      break;
    case fn_lengthformat:
      SetLengthFormat( copyfrom->LengthFormat());
      break;
    case fn_angleformat:
      SetAngleFormat( copyfrom->AngleFormat());
      break;
    case fn_angleresolution:
      SetAngleResolution( copyfrom->AngleResolution());
      break;
    case fn_lengthresolution:
      SetLengthResolution( copyfrom->LengthResolution());
      break;
    case fn_fontindex:
      SetFontIndex( copyfrom->FontIndex());
      break;
    case fn_lengthfactor:
      SetLengthFactor( copyfrom->LengthFactor());
      break;
    case fn_bAlternate:
      SetAlternate( copyfrom->Alternate());
      break;
    case fn_alternate_lengthfactor:
      SetAlternateLengthFactor( copyfrom->AlternateLengthFactor());
      break;
    case fn_alternate_lengthformat:
      SetAlternateLengthFormat( copyfrom->AlternateLengthFormat());
      break;
    case fn_alternate_lengthresolution:
      SetAlternateLengthResolution( copyfrom->AlternateLengthResolution());
      break;
    case fn_alternate_angleformat:
      SetAlternateLengthResolution( copyfrom->AlternateLengthResolution());
      break;
    case fn_alternate_angleresolution:
      SetAlternateAngleResolution( copyfrom->AlternateAngleResolution());
      break;
    case fn_prefix:
      SetPrefix( copyfrom->Prefix());
      break;
    case fn_suffix:
      SetSuffix( copyfrom->Suffix());
      break;
    case fn_alternate_prefix:
      SetAlternatePrefix( copyfrom->AlternatePrefix());
      break;
    case fn_alternate_suffix:
      SetAlternateSuffix( copyfrom->AlternateSuffix());
      break;
    case fn_dimextension:
      SetDimExtension( copyfrom->DimExtension());
      break;
    case fn_leaderarrowsize:
      SetLeaderArrowSize( copyfrom->LeaderArrowSize());
      break;
    case fn_leaderarrowtype:
      SetLeaderArrowType( (eArrowType)copyfrom->LeaderArrowType());
      break;
    case fn_suppressextension1:
      SetSuppressExtension1( copyfrom->SuppressExtension1());
      break;
    case fn_suppressextension2:
      SetSuppressExtension2( copyfrom->SuppressExtension2());
      break;
    case fn_tolerance_style:
      SetToleranceStyle( copyfrom->ToleranceStyle());
      break;
    case fn_tolerance_resolution:
      SetToleranceResolution( copyfrom->ToleranceResolution());
      break;
    case fn_tolerance_upper_value:
      SetToleranceUpperValue( copyfrom->ToleranceUpperValue());
      break;
    case fn_tolerance_lower_value:
      SetToleranceLowerValue( copyfrom->ToleranceLowerValue());
      break;
    case fn_tolerance_height_scale:
      SetToleranceHeightScale( copyfrom->ToleranceHeightScale());
      break;
    case fn_baseline_spacing:
      SetBaselineSpacing( copyfrom->BaselineSpacing());
      break;
    case fn_draw_mask:
      SetDrawTextMask( copyfrom->DrawTextMask());
      break;
    case fn_mask_color_source:
      SetMaskColorSource( copyfrom->MaskColorSource());
      break;
    case fn_mask_color:
      SetMaskColor( copyfrom->MaskColor());
      break;
    case fn_dimscale:
      SetDimScale( copyfrom->DimScale());
      break;
    case fn_dimscale_source:
      SetDimScaleSource( copyfrom->DimScaleSource());
      break;
    }
  }
  return true;
}

bool ON_DimStyle::InheritFields( const ON_DimStyle& parent)
{
  bool rc = false;

  // 2 November 2011 Dale Lear
  //   Please do not create ON_DimStyleExtra user data if
  //   it does not exist on this.
  const ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this);
  for( int i = 0; i < ON_DimStyleExtra::eFieldCount; i++ )
  {
    bool bValidField = ( 0 != pDE && i < pDE->m_valid_fields.Count() )
                     ? pDE->m_valid_fields[i] : 
                     false;
                     
    switch( i)
    {
    case fn_extextension:
      if( !bValidField)
      {
        SetExtExtension( parent.ExtExtension());
        rc = true;
      }
      break;
    case fn_extoffset:
      if( !bValidField)
      {
        SetExtOffset( parent.ExtOffset());
        rc = true;
      }
      break;
    case fn_arrowsize:
      if( !bValidField)
      {
        SetArrowSize( parent.ArrowSize());
        rc = true;
      }
      break;
    case fn_centermark:
      if( !bValidField)
      {
        SetCenterMark( parent.CenterMark());
        rc = true;
      }
      break;
    case fn_textgap:
      if( !bValidField)
      {
        SetTextGap( parent.TextGap());
        rc = true;
      }
      break;
    case fn_textheight:
      if( !bValidField)
      {
        SetTextHeight( parent.TextHeight());
        rc = true;
      }
      break;
    case fn_textalign:
      if( !bValidField)
      {
        SetTextAlignment( (ON::eTextDisplayMode)parent.TextAlignment());
        rc = true;
      }
      break;
    case fn_arrowtype:
      if( !bValidField)
      {
        SetArrowType( (eArrowType)parent.ArrowType());
        rc = true;
      }
      break;
    case fn_angularunits:
      if( !bValidField)
      {
        SetAngularUnits( (eArrowType)parent.AngularUnits());
        rc = true;
      }
      break;
    case fn_lengthformat:
      if( !bValidField)
      {
        SetLengthFormat( parent.LengthFormat());
        rc = true;
      }
      break;
    case fn_angleformat:
      if( !bValidField)
      {
        SetAngleFormat( parent.AngleFormat());
        rc = true;
      }
      break;
    case fn_angleresolution:
      if( !bValidField)
      {
        SetAngleResolution( parent.AngleResolution());
        rc = true;
      }
      break;
    case fn_lengthresolution:
      if( !bValidField)
      {
        SetLengthResolution( parent.LengthResolution());
        rc = true;
      }
      break;
    case fn_fontindex:
      if( !bValidField)
      {
        SetFontIndex( parent.FontIndex());
        rc = true;
      }
      break;
    case fn_lengthfactor:
      if( !bValidField)
      {
        SetLengthFactor( parent.LengthFactor());
        rc = true;
      }
      break;
    case fn_bAlternate:
      if( !bValidField)
      {
        SetAlternate( parent.Alternate());
        rc = true;
      }
      break;
    case fn_alternate_lengthfactor:
      if( !bValidField)
      {
        SetAlternateLengthFactor( parent.LengthFactor());
        rc = true;
      }
      break;
    case fn_alternate_lengthformat:
      if( !bValidField)
      {
        SetAlternateLengthFormat( parent.AlternateLengthFormat());
        rc = true;
      }
      break;
    case fn_alternate_lengthresolution:
      if( !bValidField)
      {
        SetAlternateLengthResolution( parent.AlternateLengthResolution());
        rc = true;
      }
      break;
    case fn_alternate_angleformat:
      if( !bValidField)
      {
        SetAlternateLengthResolution( parent.AlternateLengthResolution());
        rc = true;
      }
      break;
    case fn_alternate_angleresolution:
      if( !bValidField)
      {
        SetAlternateAngleResolution( parent.AlternateAngleResolution());
        rc = true;
      }
      break;
    case fn_prefix:
      if( !bValidField)
      {
        SetPrefix( parent.Prefix());
        rc = true;
      }
      break;
    case fn_suffix:
      if( !bValidField)
      {
        SetSuffix( parent.Suffix());
        rc = true;
      }
      break;
    case fn_alternate_prefix:
      if( !bValidField)
      {
        SetAlternatePrefix( parent.AlternatePrefix());
        rc = true;
      }
      break;
    case fn_alternate_suffix:
      if( !bValidField)
      {
        SetAlternateSuffix( parent.AlternateSuffix());
        rc = true;
      }
      break;
    case fn_dimextension:
      if( !bValidField)
      {
        SetDimExtension( parent.DimExtension());
        rc = true;
      }
      break;
    case fn_leaderarrowsize:
      if( !bValidField)
      {
        SetLeaderArrowSize( parent.LeaderArrowSize());
        rc = true;
      }
      break;
    case fn_leaderarrowtype:
      if( !bValidField)
      {
        SetLeaderArrowType( (eArrowType)parent.LeaderArrowType());
        rc = true;
      }
      break;
    case fn_suppressextension1:
      if( !bValidField)
      {
        SetSuppressExtension1( parent.SuppressExtension1());
        rc = true;
      }
      break;
    case fn_suppressextension2:
      if( !bValidField)
      {
        SetSuppressExtension2( parent.SuppressExtension2());
        rc = true;
      }
      break;
    case fn_tolerance_style:
      if( !bValidField)
      {
        SetToleranceStyle( parent.ToleranceStyle());
        rc = true;
      }
      break;
    case fn_tolerance_resolution:
      if( !bValidField)
      {
        SetToleranceResolution( parent.ToleranceResolution());
        rc = true;
      }
      break;
    case fn_tolerance_upper_value:
      if( !bValidField)
      {
        SetToleranceUpperValue( parent.ToleranceUpperValue());
        rc = true;
      }
      break;
    case fn_tolerance_lower_value:
      if( !bValidField)
      {
        SetToleranceLowerValue( parent.ToleranceLowerValue());
        rc = true;
      }
      break;
    case fn_tolerance_height_scale:
      if( !bValidField)
      {
        SetToleranceHeightScale( parent.ToleranceHeightScale());
        rc = true;
      }
      break;
    case fn_baseline_spacing:
      if( !bValidField)
      {
        SetBaselineSpacing( parent.BaselineSpacing());
          rc = true;
      }
      break;
    case fn_draw_mask:
      if( !bValidField)
      {
        SetDrawTextMask( parent.DrawTextMask());
          rc = true;
      }
      break;
    case fn_mask_color_source:
      if( !bValidField)
      {
        SetMaskColorSource( parent.MaskColorSource());
          rc = true;
      }
      break;
    case fn_mask_color:
      if( !bValidField)
      {
        SetMaskColor( parent.MaskColor());
          rc = true;
      }
      break;
    case fn_dimscale:
      if( !bValidField)
      {
        SetDimScale( parent.DimScale());
          rc = true;
      }
      break;
    case fn_dimscale_source:
      if( !bValidField)
      {
        SetDimScaleSource( parent.DimScaleSource());
          rc = true;
      }
      break;
    }
  }
  return rc;
}


bool ON_DimStyle::IsChildDimstyle() const
{
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE && pDE->m_parent_dimstyle != ON_nil_uuid)
    return true;
  else
    return false;
}

bool ON_DimStyle::IsChildOf( ON_UUID& parent_uuid) const
{
  // delete this old function when SDK can be broken.
  // parameter needed a const
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE && parent_uuid != ON_nil_uuid && pDE->m_parent_dimstyle == parent_uuid)
    return true;
  else
    return false;
}

bool ON_DimStyle::IsChildOf( const ON_UUID& parent_uuid) const
{
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE && parent_uuid != ON_nil_uuid && pDE->m_parent_dimstyle == parent_uuid)
    return true;
  else
    return false;
}

void ON_DimStyle::SetParent( ON_UUID& parent_uuid)
{
  // delete this old function when SDK can be broken.
  // parameter needed a const
  SetParentId(parent_uuid);
}

void ON_DimStyle::SetParentId( ON_UUID parent_id )
{
  bool bCreateIfNoneExists = !ON_UuidIsNil(parent_id);
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfNoneExists );
  if ( pDE )
    pDE->m_parent_dimstyle = parent_id;
}


ON_UUID ON_DimStyle::ParentId() const
{
  const ON_DimStyleExtra* pDE = DimStyleExtension(); // get existing
  return ( 0 != pDE ? pDE->m_parent_dimstyle : ON_nil_uuid );
}

bool ON_DimStyleExtra::IsFieldOverride( int field_id) const
{
  if( field_id >= 0 && field_id < m_valid_fields.Count())
    return m_valid_fields[field_id];
  else
    return false;
}

void ON_DimStyleExtra::SetFieldOverride( int field_id, bool bOverride)
{
  if( field_id >= 0 && field_id < m_valid_fields.Count())
    m_valid_fields[field_id] = bOverride;
}

// Tolerances
// Tolerance style
//  0: None
//  1: Symmetrical
//  2: Deviation
//  3: Limits
//  4: Basic
int ON_DimStyle::ToleranceStyle() const
{
  // 2 November 2011 Dale Lear
  //   Changed to NOT create ON_DimStyleExtra user data
  //   if it did not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->ToleranceStyle();
  else
    return ON_DimStyle::DefaultToleranceStyle();
}

int ON_DimStyle::ToleranceResolution() const
{
  // 2 November 2011 Dale Lear
  //   Changed to NOT create ON_DimStyleExtra user data
  //   if it did not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->ToleranceResolution();
  else
    return ON_DimStyle::DefaultToleranceResolution();
}

double ON_DimStyle::ToleranceUpperValue() const
{
  // 2 November 2011 Dale Lear
  //   Changed to NOT create ON_DimStyleExtra user data
  //   if it did not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->ToleranceUpperValue();
  else
    return ON_DimStyle::DefaultToleranceUpperValue();
}

double ON_DimStyle::ToleranceLowerValue() const
{
  // 2 November 2011 Dale Lear
  //   Changed to NOT create ON_DimStyleExtra user data
  //   if it did not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->ToleranceLowerValue();
  else
    return ON_DimStyle::DefaultToleranceLowerValue();
}

double ON_DimStyle::ToleranceHeightScale() const
{
  // 2 November 2011 Dale Lear
  //   Changed to NOT create ON_DimStyleExtra user data
  //   if it did not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->ToleranceHeightScale();
  else
    return ON_DimStyle::DefaultToleranceHeightScale();
}

double ON_DimStyle::BaselineSpacing() const
{
  // 2 November 2011 Dale Lear
  //   Changed to NOT create ON_DimStyleExtra user data
  //   if it did not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->BaselineSpacing();
  else
    return ON_DimStyle::DefaultBaselineSpacing();
}

//static 
int ON_DimStyle::DefaultToleranceStyle()
{
  return 0;
}

//static 
int ON_DimStyle::DefaultToleranceResolution()
{
  return 4;
}

//static 
double ON_DimStyle::DefaultToleranceUpperValue()
{
  return 0.0;
}

//static 
double ON_DimStyle::DefaultToleranceLowerValue()
{
  return 0.0;
}

//static 
double ON_DimStyle::DefaultToleranceHeightScale()
{
  return 1.0;
}

//static 
double ON_DimStyle::DefaultBaselineSpacing()
{
  return 1.0;
}

// static
bool ON_DimStyle::DefaultDrawTextMask()
{
  return false;
}

// static
int ON_DimStyle::DefaultMaskColorSource()
{
  return 0;
}

// static
ON_Color ON_DimStyle::DefaultMaskColor()
{
  // 2 November 2011 Dale Lear
  //   TODO - Ask Lowell what this default is supposed to be.
  //          His ON_DimStyle::MaskColor() function defaulted to black
  //          and his ON_DimStyleExtra defaulted to white.
  //return 0;
  return ON_Color(255,255,255);
}

// static
double ON_DimStyle::DefaultDimScale()
{
  return 1.0;
}

// static
int ON_DimStyle::DefaultDimScaleSource()
{
  return 0;
}


//-------------------
void ON_DimStyle::Scale( double scale)
{
  if( ON_IsValid( scale) && scale > ON_SQRT_EPSILON && 1.0 != scale )
  {
    m_extextension    *= scale;
    m_extoffset       *= scale;
    m_arrowsize       *= scale;
    m_centermark      *= scale;
    m_textgap         *= scale;
    m_textheight      *= scale;
    m_dimextension    *= scale;
    m_leaderarrowsize *= scale;

    ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, true );
    if(pDE)
    {
      pDE->Scale( scale);
    }
  }
}

void ON_DimStyle::SetToleranceStyle( int style)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultToleranceStyle() != style );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
  {
    pDE->SetToleranceStyle( style);
  }
}

void ON_DimStyle::SetToleranceResolution( int resolution)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultToleranceResolution() != resolution );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
  {
    pDE->SetToleranceResolution( resolution);
  }
}

void ON_DimStyle::SetToleranceUpperValue( double upper_value)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultToleranceUpperValue() != upper_value );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
  {
    pDE->SetToleranceUpperValue( upper_value);
  }
}

void ON_DimStyle::SetToleranceLowerValue( double lower_value)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultToleranceLowerValue() != lower_value );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
  {
    pDE->SetToleranceLowerValue( lower_value);
  }
}

void ON_DimStyle::SetToleranceHeightScale( double scale)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultToleranceHeightScale() != scale );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
  {
    pDE->SetToleranceHeightScale( scale);
  }
}

void ON_DimStyle::SetBaselineSpacing( double spacing)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultBaselineSpacing() != spacing );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
  {
    pDE->SetBaselineSpacing( spacing);
  }
}

bool ON_DimStyle::DrawTextMask() const
{
  // 2 November 2011 Dale Lear 
  //   Do not create user data if it does not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->DrawTextMask();
  else
    return ON_DimStyle::DefaultDrawTextMask();
}

void ON_DimStyle::SetDrawTextMask(bool bDraw)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultDrawTextMask() != (bDraw?true:false) );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
    pDE->SetDrawTextMask(bDraw);
}

int ON_DimStyle::MaskColorSource() const
{
  // 2 November 2011 Dale Lear 
  //   Do not create user data if it does not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->MaskColorSource();
  else
    return ON_DimStyle::DefaultMaskColorSource();
}

void ON_DimStyle::SetMaskColorSource(int source)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultMaskColorSource() != source );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
    pDE->SetMaskColorSource(source);
}

ON_Color ON_DimStyle::MaskColor() const
{
  // 2 November 2011 Dale Lear 
  //   Do not create user data if it does not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->MaskColor();
  else
    return ON_DimStyle::DefaultMaskColor();
}

void ON_DimStyle::SetMaskColor(ON_Color color)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultMaskColor() != color );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
    pDE->SetMaskColor(color);
}

double ON_DimStyle::MaskOffsetFactor() const
{
  if(m_textheight != 0.0)
    return 0.5 * m_textgap / m_textheight;
  else
    return 0.0;
}

void ON_DimStyle::SetDimScale(double scale)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultDimScale() != scale );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
    pDE->SetDimScale(scale);
}

double ON_DimStyle::DimScale() const
{
  // 2 November 2011 Dale Lear 
  //   Do not create user data if it does not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE) // && pDE->DimScaleSource() == 1)
      return pDE->DimScale();
  else
    return ON_DimStyle::DefaultDimScale();
}

void ON_DimStyle::SetDimScaleSource(int source)
{
  bool bCreateIfDoesNotExist = ( ON_DimStyle::DefaultDimScaleSource() != source );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
    pDE->SetDimScaleSource(source);
}

int ON_DimStyle::DimScaleSource() const
{
  // 2 November 2011 Dale Lear 
  //   Do not create user data if it does not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->DimScaleSource();
  else
    return ON_DimStyle::DefaultDimScaleSource();
}

void ON_DimStyle::SetSourceDimstyle(ON_UUID source_uuid)
{
  bool bCreateIfDoesNotExist = ( !(ON_nil_uuid == source_uuid) );
  ON_DimStyleExtra* pDE = ON_DimStyleExtra::DimStyleExtensionGet( this, bCreateIfDoesNotExist );
  if(pDE)
    pDE->SetSourceDimstyle(source_uuid);
}

ON_UUID ON_DimStyle::SourceDimstyle() const
{
  // 2 November 2011 Dale Lear 
  //   Do not create user data if it does not already exist.
  const ON_DimStyleExtra* pDE = DimStyleExtension();
  if(pDE)
    return pDE->SourceDimstyle();
  else
    return ON_nil_uuid;
}




// This function is temporary and will be removed next time the SDK can be modified.
class ON_DimStyleExtra* ON_DimStyle::DimStyleExtension()
{
  // This function gets an extensions class if one exists.
  // It must not create an extensions class if one does not exist.
  ON_DimStyleExtra* pExtra = ON_DimStyleExtra::Cast( GetUserData( ON_DimStyleExtra::m_ON_DimStyleExtra_class_id.Uuid()));
  return pExtra;
}

const class ON_DimStyleExtra* ON_DimStyle::DimStyleExtension() const
{
  // This function gets an extensions class if one exists.
  // It must not create an extensions class if one does not exist.
  ON_DimStyleExtra* pExtra = ON_DimStyleExtra::Cast( GetUserData( ON_DimStyleExtra::m_ON_DimStyleExtra_class_id.Uuid()));
  return pExtra;
}

// returns true if they are the same
bool ON_DimStyle::CompareFields(const ON_DimStyle& other) const
{
  if((m_extextension               != other.m_extextension) ||
     (m_extoffset                  != other.m_extoffset) ||
     (m_arrowsize                  != other.m_arrowsize) ||
     (m_centermark                 != other.m_centermark) ||
     (m_textgap                    != other.m_textgap) ||
     (m_textheight                 != other.m_textheight) ||
     (m_textalign                  != other.m_textalign) ||
     (m_arrowtype                  != other.m_arrowtype) ||
     (m_angularunits               != other.m_angularunits) ||
     (m_lengthformat               != other.m_lengthformat) ||
     (m_angleformat                != other.m_angleformat) ||
     (m_angleresolution            != other.m_angleresolution) ||
     (m_lengthresolution           != other.m_lengthresolution) ||
     (m_fontindex                  != other.m_fontindex) ||
     (m_lengthfactor               != other.m_lengthfactor) ||
     (m_bAlternate                 != other.m_bAlternate) ||
     (m_alternate_lengthfactor     != other.m_alternate_lengthfactor) ||
     (m_alternate_lengthformat     != other.m_alternate_lengthformat) ||
     (m_alternate_lengthresolution != other.m_alternate_lengthresolution) ||
     (m_alternate_angleformat      != other.m_alternate_angleformat) ||
     (m_alternate_angleresolution  != other.m_alternate_angleresolution) ||
     (m_prefix                     != other.m_prefix) ||
     (m_suffix                     != other.m_suffix) ||
     (m_alternate_prefix           != other.m_alternate_prefix) ||
     (m_alternate_suffix           != other.m_alternate_suffix) ||
     (m_dimextension               != other.m_dimextension) ||
     (m_leaderarrowsize            != other.m_leaderarrowsize) ||
     (m_leaderarrowtype            != other.m_leaderarrowtype) ||
     (m_bSuppressExtension1        != other.m_bSuppressExtension1) ||
     (m_bSuppressExtension2        != other.m_bSuppressExtension2))
    return false;

  // 2 November 2011 Dale Lear:
  //   Do not create ON_DimStyleExtra if it does not already exist.
  const ON_DimStyleExtra* pDEo = ON_DimStyleExtra::DimStyleExtensionGet(&other);
  const ON_DimStyleExtra* pDE  = ON_DimStyleExtra::DimStyleExtensionGet(this);

  if ( 0 != pDEo && 0 != pDE && !pDE->CompareFields(pDEo) )
    return false;

  // 2 November 2011 Dale Lear:
  //   I added ON_DimStyleExtra::IsDefault() and used it here
  //   so the plethora of ON_DimStyleExtra user data
  //   that is is all files made before this date and which contains
  //   all default settings is correctly handled.  (For the past year,
  //   every single dimstyle has had ON_DimStyleExtra added to it
  //   that starts out containing default settings.
  if ( 0 == pDEo && 0 != pDE && !pDE->IsDefault() )
    return false;

  if ( 0 == pDE && 0 != pDEo && !pDEo->IsDefault() )
    return false;

  return true;
}

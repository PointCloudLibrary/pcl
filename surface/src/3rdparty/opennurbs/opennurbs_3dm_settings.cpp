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

// Easy way to toggle the name of the obsolete ON_3dmView::m_target
// variable when you want to change the name and compile to insure
// no rogue code is directly accessing this variable.  See
// ON_3dmView header for more comments.
//
//#define OBSOLETE_3DM_VIEW_TARGET m_target_HIDEME
#define OBSOLETE_3DM_VIEW_TARGET m_target

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmUnitsAndTolerances
//

ON_UnitSystem::ON_UnitSystem()
{
  m_unit_system = ON::millimeters;
  m_custom_unit_scale = 1.0;
}

ON_UnitSystem::ON_UnitSystem(ON::unit_system us)
{
  m_unit_system = ON::UnitSystem(us);
  m_custom_unit_scale = ON::UnitScale(ON::meters,m_unit_system);
}

ON_UnitSystem& ON_UnitSystem::operator=(ON::unit_system us)
{
  m_unit_system = ON::UnitSystem(us);
  if ( ON::custom_unit_system != us )
  {
    m_custom_unit_scale = ON::UnitScale(ON::meters,m_unit_system);
    m_custom_unit_name.Destroy();
  }
  return *this;
}

bool ON_UnitSystem::operator==(const ON_UnitSystem& other)
{
  if ( m_unit_system != other.m_unit_system )
    return false;

  if ( ON::custom_unit_system == m_unit_system )
  {
    if ( m_custom_unit_name.Compare(other.m_custom_unit_name) )
      return false;
    if ( !(m_custom_unit_scale == other.m_custom_unit_scale) )
      return false;
  }

  return true;
}

bool ON_UnitSystem::operator!=(const ON_UnitSystem& other)
{
  if ( m_unit_system != other.m_unit_system )
    return true;

  if ( ON::custom_unit_system == m_unit_system )
  {
    if ( m_custom_unit_name.Compare(other.m_custom_unit_name) )
      return true;
    if ( m_custom_unit_scale != other.m_custom_unit_scale )
      return true;
  }

  return false;
}

ON_UnitSystem::~ON_UnitSystem()
{
}

void ON_UnitSystem::Default()
{
  m_unit_system = ON::millimeters;
  m_custom_unit_scale = 1.0;
  m_custom_unit_name.Destroy();
}

bool ON_UnitSystem::IsValid() const
{
  if ( m_unit_system != ON::UnitSystem(m_unit_system) )
  {
    // bogus enum value
    return false;
  }

  if ( ON::custom_unit_system == m_unit_system )
  {
    if ( !ON_IsValid(m_custom_unit_scale) || m_custom_unit_scale <= 0.0 )
    {
      // m_custom_unit_scale should be > 0.0 and a valid double
      return false;
    }
  }

  return true;
}

bool ON_UnitSystem::Read( ON_BinaryArchive& file )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  
  if ( !file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version) )
    return false;

  bool rc = (1 == major_version);
  int i = m_unit_system;
  if (rc)
    rc = file.ReadInt( &i );
  if (rc)
    m_unit_system = ON::UnitSystem(i);
  if (rc)
    rc = file.ReadDouble( &m_custom_unit_scale );
  if (rc)
    rc = file.ReadString( m_custom_unit_name );

  if ( !file.EndRead3dmChunk() )
    rc = false;

  return rc;
}

bool ON_UnitSystem::Write( ON_BinaryArchive& file ) const
{
  if ( !file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0) )
    return false;


  // values saved in the file
  //no_unit_system =  0, 
  //microns        =  1,  // 1.0e-6 meters
  //millimeters    =  2,  // 1.0e-3 meters
  //centimeters    =  3,  // 1.0e-2 meters
  //meters         =  4,
  //kilometers     =  5,  // 1.0e+3 meters
  //microinches    =  6,  // 1.0e-6 inches
  //mils           =  7,  // 1.0e-3 inches
  //inches         =  8,  // 0.0254 meters
  //feet           =  9,  // 12 inches
  //miles          = 10,  // 63360 inches
  //custom_unit_system = 11, // x meters with x defined in ON_3dmUnitsAndTolerances.m_custom_unit_scale

  bool rc = file.WriteInt( m_unit_system );
  if (rc)
    rc = file.WriteDouble( m_custom_unit_scale );
  if (rc)
    rc = file.WriteString( m_custom_unit_name );

  if ( !file.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

void ON_UnitSystem::Dump( ON_TextLog& dump ) const
{
  ON_wString sUnitSystem;
  switch( m_unit_system )
  {
  case ON::no_unit_system:
    sUnitSystem = "no units";
    break;
  case ON::angstroms:
    sUnitSystem = "angstroms";
    break;
  case ON::nanometers:
    sUnitSystem = "nanometers";
    break;
  case ON::microns:
    sUnitSystem = "microns";
    break;
  case ON::millimeters:
    sUnitSystem = "millimeters";
    break;
  case ON::decimeters:
    sUnitSystem = "decimeters";
    break;
  case ON::centimeters:
    sUnitSystem = "centimeters";
    break;
  case ON::meters:
    sUnitSystem = "meters";
    break;
  case ON::dekameters:
    sUnitSystem = "dekameters";
    break;
  case ON::hectometers:
    sUnitSystem = "hectometers";
    break;
  case ON::kilometers:
    sUnitSystem = "kilometers";
    break;
  case ON::megameters:
    sUnitSystem = "megameters";
    break;
  case ON::gigameters:
    sUnitSystem = "gigameters";
    break;
  case ON::microinches:
    sUnitSystem = "microinches";
    break;
  case ON::mils:
    sUnitSystem = "mils (= 0.001 inches)";
    break;
  case ON::inches:
    sUnitSystem = "inches";
    break;
  case ON::feet:
    sUnitSystem = "feet";
    break;
  case ON::yards:
    sUnitSystem = "yards";
    break;
  case ON::miles:
    sUnitSystem = "miles";
    break;
  case ON::printer_point:
    sUnitSystem = "points (1/72 inch)";
    break;
  case ON::printer_pica:
    sUnitSystem = "picas (1/6 inch)";
    break;
  case ON::nautical_mile:
    sUnitSystem = "nautical miles";
    break;
  case ON::astronomical:
    sUnitSystem = "astronomical units";
    break;
  case ON::lightyears:
    sUnitSystem = "light years";
    break;
  case ON::parsecs:
    sUnitSystem = "parsecs";
    break;

  case ON::custom_unit_system:
    if ( m_custom_unit_name.Length() > 0 )
    {
      const wchar_t* wsCustomUnitName = m_custom_unit_name.Array();
      if ( 0 != wsCustomUnitName && 0 != wsCustomUnitName[0] )
      {
        sUnitSystem.Format("%ls (= %g meters)",
                           wsCustomUnitName,
                           m_custom_unit_scale);
      }
    }
    else
      sUnitSystem.Format("user defined unit (= %g meters)",m_custom_unit_scale);
    break;
  default:
    sUnitSystem = "unknown unit system";
    break;
  }
  const wchar_t* wsUnitSystem = sUnitSystem.Array();
  if ( 0 != wsUnitSystem )
    dump.Print("Unit system: %ls\n",wsUnitSystem);
}

void ON_3dmUnitsAndTolerances::Default()
{
  m_unit_system.Default();
  m_unit_system.m_unit_system = ON::millimeters;
  m_unit_system.m_custom_unit_name = L"Units";
  m_absolute_tolerance = 0.001;    // = 0.01;       // Dale Lear: Changed March 2006
  m_angle_tolerance = ON_PI/180.0; // = ON_PI/60.0; // Dale Lear: Changed 5 April 2006
  m_relative_tolerance = 0.01;

  m_distance_display_mode = ON::decimal;
  m_distance_display_precision = 3;
}

ON_3dmUnitsAndTolerances::ON_3dmUnitsAndTolerances()
                        : m_absolute_tolerance(0.0),
                          m_angle_tolerance(0.0),
                          m_relative_tolerance(0.0),
                          m_distance_display_mode(ON::decimal),
                          m_distance_display_precision(3)
{
  Default();
}

ON_3dmUnitsAndTolerances::~ON_3dmUnitsAndTolerances()
{}

ON_3dmUnitsAndTolerances::ON_3dmUnitsAndTolerances(const ON_3dmUnitsAndTolerances& src )
                        : m_absolute_tolerance(0.0),
                          m_angle_tolerance(0.0),
                          m_relative_tolerance(0.0),
                          m_distance_display_mode(ON::decimal),
                          m_distance_display_precision(3)
{
  Default();
  *this = src;
}

ON_3dmUnitsAndTolerances& ON_3dmUnitsAndTolerances::operator=(const ON_3dmUnitsAndTolerances& src )
{
  if ( this != &src ) 
  {
    m_unit_system = src.m_unit_system;
    m_absolute_tolerance = src.m_absolute_tolerance;
    m_angle_tolerance = src.m_angle_tolerance;
    m_relative_tolerance = src.m_relative_tolerance;
    m_distance_display_mode = src.m_distance_display_mode;
    m_distance_display_precision = src.m_distance_display_precision;
  }
  return *this;
}

bool ON_3dmUnitsAndTolerances::Write( ON_BinaryArchive& file ) const
{
  const int version = 102;
  int i;

  // version 100 ON_3dmUnitsAndTolerances settings
  bool rc = file.WriteInt( version );
  i = m_unit_system.m_unit_system;
  if ( rc ) rc = file.WriteInt( i );
  if ( rc ) rc = file.WriteDouble( m_absolute_tolerance );
  if ( rc ) rc = file.WriteDouble( m_angle_tolerance );
  if ( rc ) rc = file.WriteDouble( m_relative_tolerance );

  // added in version 101
  i = m_distance_display_mode;
  if ( rc ) rc = file.WriteInt( i );
  i = m_distance_display_precision;
  if ( i < 0 || i > 20 ) {
    ON_ERROR("ON_3dmUnitsAndTolerances::Write() - m_distance_display_precision out of range.");
    i = 3;
  }
  if ( rc ) rc = file.WriteInt( i );

  // added in version 102
  if ( rc ) rc = file.WriteDouble( m_unit_system.m_custom_unit_scale );
  if ( rc ) rc = file.WriteString( m_unit_system.m_custom_unit_name );
  return rc;
}

bool ON_3dmUnitsAndTolerances::Read( ON_BinaryArchive& file )
{
  Default();
  int version = 0;
  bool rc = file.ReadInt( &version );
  if ( rc && version >= 100 && version < 200 ) {
    int us = ON::no_unit_system;
    rc = file.ReadInt( &us );
    if ( rc )
      m_unit_system.m_unit_system = ON::UnitSystem(us);
    if ( rc ) rc = file.ReadDouble( &m_absolute_tolerance );
    if ( rc ) rc = file.ReadDouble( &m_angle_tolerance );
    if ( rc ) rc = file.ReadDouble( &m_relative_tolerance );
    if ( version >= 101 ) {
      int dm = ON::decimal;
      if ( rc ) rc = file.ReadInt( &dm );
      if ( rc ) m_distance_display_mode = ON::DistanceDisplayMode(dm);
      if ( rc ) rc = file.ReadInt( &m_distance_display_precision );
      if ( m_distance_display_precision < 0 || m_distance_display_precision > 20 )
        m_distance_display_precision = 3; // some beta files had bogus values stored in file
      if ( version >= 102 ) {
        if ( rc ) rc = file.ReadDouble( &m_unit_system.m_custom_unit_scale );
        if ( rc ) rc = file.ReadString( m_unit_system.m_custom_unit_name );
      }
    }
  }
  return rc;
}

void ON_3dmUnitsAndTolerances::Dump( ON_TextLog& dump) const
{
  m_unit_system.Dump(dump);
  dump.Print("Absolute tolerance: %g\n",m_absolute_tolerance);
  dump.Print("Angle tolerance: %g\n",m_angle_tolerance);
}

double ON_3dmUnitsAndTolerances::Scale( ON::unit_system us ) const
{
  // Example: If us = meters and m_unit_system = centimeters,
  // then Scale() returns 100.
  return ON::UnitScale( us, m_unit_system );
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmRenderSettings
//
void ON_3dmRenderSettings::Dump( ON_TextLog& text_log ) const
{
  text_log.Print("m_bCustomImageSize = %s\n",m_bCustomImageSize?"true":"false");
  text_log.Print("m_image_width = %d\n",m_image_width);
  text_log.Print("m_image_height = %d\n",m_image_height);
  text_log.Print("m_image_dpi = %g\n",m_image_dpi);
  text_log.Print("m_image_us = %d\n",m_image_us);
  text_log.Print("m_ambient_light rgb = ");text_log.PrintRGB(m_ambient_light);text_log.Print("\n");
  text_log.Print("m_background_style = %d\n",m_background_style);
  text_log.Print("m_background_color rgb = ");text_log.PrintRGB(m_background_color);text_log.Print("\n");
  text_log.Print("m_background_bitmap_filename = ");text_log.Print(m_background_bitmap_filename);text_log.Print("\n");
  text_log.Print("m_bUseHiddenLights = %s\n",m_bUseHiddenLights?"true":"false");
  text_log.Print("m_bDepthCue = %s\n",m_bDepthCue?"true":"false");
  text_log.Print("m_bFlatShade = %s\n",m_bFlatShade?"true":"false");
  text_log.Print("m_bRenderBackfaces = %s\n",m_bRenderBackfaces?"true":"false");
  text_log.Print("m_bRenderPoints = %s\n",m_bRenderPoints?"true":"false");
  text_log.Print("m_bRenderCurves = %s\n",m_bRenderCurves?"true":"false");
  text_log.Print("m_bRenderIsoparams = %s\n",m_bRenderIsoparams?"true":"false");
  text_log.Print("m_bRenderMeshEdges = %s\n",m_bRenderMeshEdges?"true":"false");
  text_log.Print("m_bRenderAnnotation = %s\n",m_bRenderAnnotation?"true":"false");

  text_log.Print("m_antialias_style = %d\n",m_antialias_style);
  text_log.Print("m_shadowmap_style = %d\n",m_shadowmap_style);
  text_log.Print("m_shadowmap_width = %d\n",m_shadowmap_width);
  text_log.Print("m_shadowmap_height = %d\n",m_shadowmap_height);
  text_log.Print("m_shadowmap_offset = %g\n",m_shadowmap_offset);

  text_log.Print("m_bScaleBackgroundToFit = %s\n",m_bScaleBackgroundToFit?"true":"false");
}

void ON_3dmRenderSettings::Default()
{
  m_bCustomImageSize = false;
  m_image_width  = 800;
  m_image_height = 600;
  m_bScaleBackgroundToFit = false;
  memset(m_reserved1,0,sizeof(m_reserved1));
  m_image_dpi = 72.0;
  m_image_us = ON::inches;


  m_ambient_light.SetRGB( 0, 0, 0);

  m_background_style = 0;
  m_background_color.SetRGB(160,160,160);
  m_background_bottom_color.SetRGB(160,160,160);
  m_background_bitmap_filename.Destroy();

  m_bUseHiddenLights = false;

  m_bDepthCue = false;
  m_bFlatShade = false;

  m_bRenderBackfaces = true;
  m_bRenderPoints = false;
  m_bRenderCurves = false;
  m_bRenderIsoparams = false;
  m_bRenderMeshEdges = false;
  m_bRenderAnnotation = false;

  m_antialias_style = 1;

  m_shadowmap_style = 1;
  m_shadowmap_width = 1000;
  m_shadowmap_height = 1000;
  m_shadowmap_offset = 0.75;

  m_bUsesAmbientAttr      = true;
  m_bUsesBackgroundAttr   = true;
  m_bUsesBackfaceAttr     = false;
  m_bUsesPointsAttr       = false;
  m_bUsesCurvesAttr       = true;
  m_bUsesIsoparmsAttr     = true;
  m_bUsesMeshEdgesAttr    = false;
  m_bUsesAnnotationAttr   = true;
  m_bUsesHiddenLightsAttr = true;

  memset(m_reserved2,0,sizeof(m_reserved2));
}

ON_3dmRenderSettings::ON_3dmRenderSettings()
{
  Default();
}

ON_3dmRenderSettings::~ON_3dmRenderSettings()
{
  m_background_bitmap_filename.Destroy();
}

ON_3dmRenderSettings::ON_3dmRenderSettings(const ON_3dmRenderSettings& src )
{
  Default();
  *this = src;
}

ON_3dmRenderSettings& ON_3dmRenderSettings::operator=(const ON_3dmRenderSettings& src )
{
  if ( this != &src ) {
    m_bCustomImageSize = src.m_bCustomImageSize;
    m_image_width = src.m_image_width;
    m_image_height = src.m_image_height;
    m_bScaleBackgroundToFit = src.m_bScaleBackgroundToFit;
    m_image_dpi = src.m_image_dpi;
    m_image_us = src.m_image_us;
    m_ambient_light = src.m_ambient_light;
    m_background_style = src.m_background_style;
    m_background_color = src.m_background_color;
    m_background_bitmap_filename = src.m_background_bitmap_filename;
    m_bUseHiddenLights = src.m_bUseHiddenLights;
    m_bDepthCue = src.m_bDepthCue;
    m_bFlatShade = src.m_bFlatShade;
    m_bRenderBackfaces = src.m_bRenderBackfaces;
    m_bRenderPoints = src.m_bRenderPoints;
    m_bRenderCurves = src.m_bRenderCurves;
    m_bRenderIsoparams = src.m_bRenderIsoparams;
    m_bRenderMeshEdges = src.m_bRenderMeshEdges;
    m_bRenderAnnotation = src.m_bRenderAnnotation;
    m_antialias_style = src.m_antialias_style;
    m_shadowmap_style = src.m_shadowmap_style;
    m_shadowmap_width = src.m_shadowmap_width;
    m_shadowmap_height = src.m_shadowmap_height;
    m_shadowmap_offset = src.m_shadowmap_offset;
    
    m_background_bottom_color = src.m_background_bottom_color;
    m_bUsesAmbientAttr      = src.m_bUsesAmbientAttr;
    m_bUsesBackgroundAttr   = src.m_bUsesBackgroundAttr;
    m_bUsesBackfaceAttr     = src.m_bUsesBackfaceAttr;  
    m_bUsesPointsAttr       = src.m_bUsesPointsAttr;        
    m_bUsesCurvesAttr       = src.m_bUsesCurvesAttr;       
    m_bUsesIsoparmsAttr     = src.m_bUsesIsoparmsAttr;      
    m_bUsesMeshEdgesAttr    = src.m_bUsesMeshEdgesAttr;     
    m_bUsesAnnotationAttr   = src.m_bUsesAnnotationAttr;    
    m_bUsesHiddenLightsAttr = src.m_bUsesHiddenLightsAttr;  
  }
  return *this;
}

bool ON_3dmRenderSettings::Write( ON_BinaryArchive& file ) const
{
  int i;
  // version 103: 11 November 2010
  const int version = 103;
  bool rc = file.WriteInt( version );
  // version >= 100
  if (rc) rc = file.WriteInt( m_bCustomImageSize );
  if (rc) rc = file.WriteInt( m_image_width );
  if (rc) rc = file.WriteInt( m_image_height );
  if (rc) rc = file.WriteColor( m_ambient_light );
  if (rc) rc = file.WriteInt( m_background_style );
  if (rc) rc = file.WriteColor( m_background_color );
  if (rc) rc = file.WriteString( m_background_bitmap_filename );
  if (rc) rc = file.WriteInt( m_bUseHiddenLights );
  if (rc) rc = file.WriteInt( m_bDepthCue );
  if (rc) rc = file.WriteInt( m_bFlatShade );

  // 26 August 2003 Dale Lear:
  //     When saving V2 files, turn on backfaces. RR 11656
  //
  i = (file.Archive3dmVersion() >= 3) ? m_bRenderBackfaces : 1;
  if (rc) rc = file.WriteInt( i );

  if (rc) rc = file.WriteInt( m_bRenderPoints );
  if (rc) rc = file.WriteInt( m_bRenderCurves );
  if (rc) rc = file.WriteInt( m_bRenderIsoparams );
  if (rc) rc = file.WriteInt( m_bRenderMeshEdges );
  if (rc) rc = file.WriteInt( m_bRenderAnnotation );
  if (rc) rc = file.WriteInt( m_antialias_style );
  if (rc) rc = file.WriteInt( m_shadowmap_style );
  if (rc) rc = file.WriteInt( m_shadowmap_width );
  if (rc) rc = file.WriteInt( m_shadowmap_height );
  if (rc) rc = file.WriteDouble( m_shadowmap_offset );
  // version >= 101 begins here
  if (rc) rc = file.WriteDouble( m_image_dpi );
  i = m_image_us;
  if (rc) rc = file.WriteInt( i );
  // version >= 102 begins here
  if (rc) rc = file.WriteColor( m_background_bottom_color );

  // version >= 103 begins here - added 11 November 2010
  if (rc) rc = file.WriteBool( m_bScaleBackgroundToFit );

  return rc;
}

bool ON_3dmRenderSettings::Read( ON_BinaryArchive& file )
{
  Default();
  int version = 0;
  bool rc = file.ReadInt( &version );
  if ( rc && version >= 100 && version < 200 ) 
  {
    if (rc) 
      rc = file.ReadInt( &m_bCustomImageSize );
    if (rc) 
      rc = file.ReadInt( &m_image_width );
    if (rc) 
      rc = file.ReadInt( &m_image_height );
    if (rc) 
      rc = file.ReadColor( m_ambient_light );
    if (rc) 
      rc = file.ReadInt( &m_background_style );
    if (rc) 
      rc = file.ReadColor( m_background_color );
    if (rc) 
      rc = file.ReadString( m_background_bitmap_filename );
    if (rc) 
      rc = file.ReadInt( &m_bUseHiddenLights );
    if (rc) 
      rc = file.ReadInt( &m_bDepthCue );
    if (rc) 
      rc = file.ReadInt( &m_bFlatShade );
    if (rc) 
      rc = file.ReadInt( &m_bRenderBackfaces );
    if (rc) 
      rc = file.ReadInt( &m_bRenderPoints );
    if (rc) 
      rc = file.ReadInt( &m_bRenderCurves );
    if (rc) 
      rc = file.ReadInt( &m_bRenderIsoparams );
    if (rc) 
      rc = file.ReadInt( &m_bRenderMeshEdges );
    if (rc) 
      rc = file.ReadInt( &m_bRenderAnnotation );
    if (rc) 
      rc = file.ReadInt( &m_antialias_style );
    if (rc) 
      rc = file.ReadInt( &m_shadowmap_style );
    if (rc) 
      rc = file.ReadInt( &m_shadowmap_width );
    if (rc) 
      rc = file.ReadInt( &m_shadowmap_height );
    if (rc) 
      rc = file.ReadDouble( &m_shadowmap_offset );
    if (rc && version >= 101) 
    {
      if (rc) 
        rc = file.ReadDouble( &m_image_dpi );
      if (rc) 
      {
        int i;
        rc = file.ReadInt(&i);
        if (rc)
          m_image_us = ON::UnitSystem(i);
      }
     
      if (rc && version >= 102) 
      {
        rc = file.ReadColor( m_background_bottom_color );
        if (rc && version >= 103)
        {
          rc = file.ReadBool( &m_bScaleBackgroundToFit );
        }
      }
    }
  }
  return rc;
}

bool ON_3dmRenderSettings::ScaleBackgroundToFit() const
{
  return m_bScaleBackgroundToFit;
}

void ON_3dmRenderSettings::SetScaleBackgroundToFit( bool bScaleBackgroundToFit )
{
  // The "? true : false" is here to prevent hacks from using a bool
  // to store settings besides 1 and 0.
  m_bScaleBackgroundToFit = bScaleBackgroundToFit?true:false;
}


//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmAnnotationSettings
//

ON_3dmAnnotationSettings::ON_3dmAnnotationSettings()
{
  Default();
}

ON_3dmAnnotationSettings::~ON_3dmAnnotationSettings()
{
}

ON_3dmAnnotationSettings::ON_3dmAnnotationSettings(const ON_3dmAnnotationSettings& src)
{
  Default();
  *this = src;

}

ON_3dmAnnotationSettings& ON_3dmAnnotationSettings::operator=(const ON_3dmAnnotationSettings& src)
{
  if ( this != &src ) {
    m_dimscale = src.m_dimscale;
    m_textheight = src.m_textheight;
    m_dimexe = src.m_dimexe;
    m_dimexo = src.m_dimexo;
    m_arrowlength = src.m_arrowlength;
    m_arrowwidth = src.m_arrowwidth;
    m_centermark = src.m_centermark;
    m_dimunits = src.m_dimunits;;
    m_arrowtype = src.m_arrowtype;
    m_angularunits = src.m_angularunits;
    m_lengthformat = src.m_lengthformat;
    m_angleformat = src.m_angleformat;
    m_textalign = src.m_textalign;
    m_resolution = src.m_resolution;
    m_facename = src.m_facename;
    m_world_view_text_scale = src.m_world_view_text_scale;
    m_world_view_hatch_scale = src.m_world_view_hatch_scale;
    m_bEnableAnnotationScaling = src.m_bEnableAnnotationScaling;
    m_bEnableHatchScaling = src.m_bEnableHatchScaling;
  }
  return *this;
}

void ON_3dmAnnotationSettings::Dump( ON_TextLog& text_log ) const
{
  // TODO
}

void ON_3dmAnnotationSettings::Default()
{
  memset(this,0,sizeof(*this));

  m_dimscale = 1.0;       // model size / plotted size
  m_textheight = 1.0;
  m_dimexe = 1.0;
  m_dimexo = 1.0;
  m_arrowlength = 1.0;
  m_arrowwidth = 1.0;
  m_centermark = 1.0;

  m_dimunits = ON::no_unit_system;  // units used to measure the dimension
  m_arrowtype = 0;     // 0: filled narrow triangular arrow
  m_angularunits = 0;  // 0: degrees, 1: radians
  m_lengthformat = 0;  // 0: decimal, ...
  m_angleformat = 0;   // 0: decimal degrees, ...
  m_textalign = 0;     // 0: above line, 1: in line, 2: horizontal
  m_resolution = 0;    // depends on m_lengthformat
                       // for decimal, digits past the decimal point

  m_facename.Destroy(); // [LF_FACESIZE] // windows font name

  m_world_view_text_scale = 1.0f;
  m_world_view_hatch_scale = 1.0f;
  m_bEnableAnnotationScaling = 1;
  m_bEnableHatchScaling = 1;
}

double ON_3dmAnnotationSettings::WorldViewTextScale() const
{
  return m_world_view_text_scale;
}

double ON_3dmAnnotationSettings::WorldViewHatchScale() const
{
  return m_world_view_hatch_scale;
}

void ON_3dmAnnotationSettings::SetWorldViewTextScale(double world_view_text_scale )
{
  if ( ON_IsValid(world_view_text_scale) && world_view_text_scale > 0.0 )
    m_world_view_text_scale = (float)world_view_text_scale;
}

void ON_3dmAnnotationSettings::SetWorldViewHatchScale(double world_view_hatch_scale )
{
  if ( ON_IsValid(world_view_hatch_scale) && world_view_hatch_scale > 0.0 )
    m_world_view_hatch_scale = (float)world_view_hatch_scale;
}

bool ON_3dmAnnotationSettings::IsAnnotationScalingEnabled() const
{
  return m_bEnableAnnotationScaling?true:false;
}

void ON_3dmAnnotationSettings::EnableAnnotationScaling( bool bEnable )
{
  m_bEnableAnnotationScaling = bEnable?1:0;
}


bool ON_3dmAnnotationSettings::IsHatchScalingEnabled() const
{
  return m_bEnableHatchScaling?true:false;
}

void ON_3dmAnnotationSettings::EnableHatchScaling( bool bEnable )
{
  m_bEnableHatchScaling = bEnable?1:0;
}


bool ON_3dmAnnotationSettings::Read( ON_BinaryArchive& file )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( major_version == 1 ) {
    if ( minor_version >= 0 ) {
      if (rc) rc = file.ReadDouble(&m_dimscale);
      if (rc) rc = file.ReadDouble(&m_textheight);
      if (rc) rc = file.ReadDouble(&m_dimexe);
      if (rc) rc = file.ReadDouble(&m_dimexo);
      if (rc) rc = file.ReadDouble(&m_arrowlength);
      if (rc) rc = file.ReadDouble(&m_arrowwidth);
      if (rc) rc = file.ReadDouble(&m_centermark);

      {
        int i;
        if (rc) 
        {
          rc = file.ReadInt( &i );
          if (rc)
            m_dimunits = ON::UnitSystem(i);
        }
      }

      if (rc) rc = file.ReadInt( &m_arrowtype );
      if (rc) rc = file.ReadInt( &m_angularunits );
      if (rc) rc = file.ReadInt( &m_lengthformat );
      if (rc) rc = file.ReadInt( &m_angleformat );
      if (rc) rc = file.ReadInt( &m_textalign );
      if (rc) rc = file.ReadInt( &m_resolution );

      if (rc) rc = file.ReadString( m_facename );

      // files that do not contain m_bEnableAnnotationScaling,
      // set m_bEnableAnnotationScaling = false so the display 
      // image does not change.
      m_bEnableAnnotationScaling = 0;

      // files that do not contain m_bEnableHatchScaling,
      // set m_bEnableHatchScaling = false so the display
      // image does not change.
      m_bEnableHatchScaling = 0;

      if ( minor_version >= 1 )
      {
        // Added 25 August 2010 chunk version 1.1
        double d = m_world_view_text_scale;
        if (rc) rc = file.ReadDouble(&d);
        if (rc && ON_IsValid(d) && d >= 0.0 ) m_world_view_text_scale = (float)d;
        if (rc) rc = file.ReadChar(&m_bEnableAnnotationScaling);
        if ( minor_version >= 2 )
        {
          d = m_world_view_hatch_scale;
          if (rc) rc = file.ReadDouble(&d);
          if (rc && ON_IsValid(d) && d >= 0.0) m_world_view_hatch_scale = (float)d;
          if (rc) rc = file.ReadChar(&m_bEnableHatchScaling);
        }
      }
    }
  }
  else {
    rc = false;
  }
  return rc;
}

bool ON_3dmAnnotationSettings::Write( ON_BinaryArchive& file ) const
{
  int i;
  bool rc = file.Write3dmChunkVersion(1,2);
  // March 22, 2010 - Global DimScale abandoned and moved into DimStyles, so now
  // in older files, the dimscale values are multiplied into the DimStyle lengths and
  // DimScale is written as 1.0
  if (rc) rc = file.WriteDouble(1.0);

  if (rc) rc = file.WriteDouble(m_textheight);
  if (rc) rc = file.WriteDouble(m_dimexe);
  if (rc) rc = file.WriteDouble(m_dimexo);
  if (rc) rc = file.WriteDouble(m_arrowlength);
  if (rc) rc = file.WriteDouble(m_arrowwidth);
  if (rc) rc = file.WriteDouble(m_centermark);

  i = m_dimunits;
  if (rc) rc = file.WriteInt( i );
  if (rc) rc = file.WriteInt( m_arrowtype );
  if (rc) rc = file.WriteInt( m_angularunits );
  if (rc) rc = file.WriteInt( m_lengthformat );
  if (rc) rc = file.WriteInt( m_angleformat );
  int textalign = (int)m_textalign;

  // 8-20-03 lw 
  // How the hell did this get changed?
  if( file.Archive3dmVersion() <= 2)
  {
    switch( m_textalign)
    {
    case ON::dtHorizontal:
      textalign = 2;
      break;
    case ON::dtInLine:
      textalign = 1;
      break;
    default:
      textalign = 0;
      break;
    }
  }
  if (rc) rc = file.WriteInt( textalign );
  if (rc) rc = file.WriteInt( m_resolution );

  if (rc) rc = file.WriteString( m_facename );

  // Added 25 August 2010 chunk version 1.1
  double d = m_world_view_text_scale;
  if (rc) rc = file.WriteDouble(d);
  if (rc) rc = file.WriteChar(m_bEnableAnnotationScaling);

  // Added 14 January 2011 chunk version 1.2
  d = m_world_view_hatch_scale;
  if (rc) rc = file.WriteDouble(d);
  if (rc) rc = file.WriteChar(m_bEnableHatchScaling);

  return rc;
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmConstructionPlane
//
ON_3dmConstructionPlane::ON_3dmConstructionPlane()
{
  Default();
}

ON_3dmConstructionPlane::~ON_3dmConstructionPlane()
{
}

// default copy constructor and operator= work fine
/*
ON_3dmConstructionPlane::ON_3dmConstructionPlane(const ON_3dmConstructionPlane& src)
{
  Default();
  *this = src;
}
ON_3dmConstructionPlane& ON_3dmConstructionPlane::operator=(const ON_3dmConstructionPlane& src)
{
  if ( this != &src ) 
  {
    m_plane = src.m_plane;
    m_grid_spacing = src.m_grid_spacing;
    m_snap_spacing = src.m_snap_spacing;
    m_grid_line_count = src.m_grid_line_count;
    m_grid_thick_frequency = src.m_grid_thick_frequency;
    m_name = src.m_name;
    m_bDepthBuffer = src.m_bDepthBuffer;
  }
  return *this;
}
*/

void ON_3dmConstructionPlane::Dump( ON_TextLog& text_log ) const
{
  // TODO
}

void ON_3dmConstructionPlane::Default()
{
  m_name.Destroy();
  m_plane = ON_xy_plane;

  // construction grid appearance
	m_grid_spacing = 1.0;   // distance between grid lines
	m_snap_spacing = 1.0;   // distance between grid snap points
	m_grid_line_count = 70;     // number of grid lines in each direction
  m_grid_thick_frequency = 5; // thick line frequency
  m_bDepthBuffer = true;
}

bool ON_3dmConstructionPlane::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.Write3dmChunkVersion(1,1);

  if (rc) rc = file.WritePlane(m_plane);
  if (rc) rc = file.WriteDouble(m_grid_spacing);
  if (rc) rc = file.WriteDouble(m_snap_spacing);
  if (rc) rc = file.WriteInt(m_grid_line_count);
  if (rc) rc = file.WriteInt(m_grid_thick_frequency);
  if (rc) rc = file.WriteString(m_name);

  // added for version 1.1 chunks
  if (rc) rc = file.WriteBool(m_bDepthBuffer);

  return rc;
}

bool ON_3dmConstructionPlane::Read( ON_BinaryArchive& file )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1) 
  {
    if (rc) rc = file.ReadPlane(m_plane);
    if (rc) rc = file.ReadDouble(&m_grid_spacing);
    if (rc) rc = file.ReadDouble(&m_snap_spacing);
    if (rc) rc = file.ReadInt(&m_grid_line_count);
    if (rc) rc = file.ReadInt(&m_grid_thick_frequency);
    if (rc) rc = file.ReadString(m_name);

    if ( minor_version >= 1 )
    {
      if (rc) rc = file.ReadBool(&m_bDepthBuffer);
    }
  }
  return rc;
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmConstructionPlaneGridDefaults
//
ON_3dmConstructionPlaneGridDefaults::ON_3dmConstructionPlaneGridDefaults()
{
  Default();
}

ON_3dmConstructionPlaneGridDefaults::~ON_3dmConstructionPlaneGridDefaults()
{
}

ON_3dmConstructionPlaneGridDefaults::ON_3dmConstructionPlaneGridDefaults(const ON_3dmConstructionPlaneGridDefaults& src)
{
  Default();
  *this = src;
}
ON_3dmConstructionPlaneGridDefaults& ON_3dmConstructionPlaneGridDefaults::operator=(const ON_3dmConstructionPlaneGridDefaults& src)
{
  if ( this != &src ) {
    m_grid_spacing = src.m_grid_spacing;
    m_snap_spacing = src.m_snap_spacing;
    m_grid_line_count = src.m_grid_line_count;
    m_grid_thick_frequency = src.m_grid_thick_frequency;
    m_bShowGrid = src.m_bShowGrid;
    m_bShowGridAxes = src.m_bShowGridAxes;
    m_bShowWorldAxes = src.m_bShowWorldAxes;
  }
  return *this;
}

void ON_3dmConstructionPlaneGridDefaults::Dump(ON_TextLog& text_log) const
{
  // TODO
}

void ON_3dmConstructionPlaneGridDefaults::Default()
{
  // construction grid appearance
	m_grid_spacing = 1.0;   // distance between grid lines
	m_snap_spacing = 1.0;   // distance between grid snap points
	m_grid_line_count = 70;     // number of grid lines in each direction
  m_grid_thick_frequency = 5; // thick line frequency
  m_bShowGrid = true;
  m_bShowGridAxes = true;
  m_bShowWorldAxes = true;
}

bool ON_3dmConstructionPlaneGridDefaults::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.Write3dmChunkVersion(1,0);
  if (rc) rc = file.WriteDouble(m_grid_spacing);
  if (rc) rc = file.WriteDouble(m_snap_spacing);
  if (rc) rc = file.WriteInt(m_grid_line_count);
  if (rc) rc = file.WriteInt(m_grid_thick_frequency);
  if (rc) rc = file.WriteInt(m_bShowGrid);
  if (rc) rc = file.WriteInt(m_bShowGridAxes);
  if (rc) rc = file.WriteInt(m_bShowWorldAxes);
  return rc;
}

bool ON_3dmConstructionPlaneGridDefaults::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1) {
    if (rc) rc = file.ReadDouble(&m_grid_spacing);
    if (rc) rc = file.ReadDouble(&m_snap_spacing);
    if (rc) rc = file.ReadInt(&m_grid_line_count);
    if (rc) rc = file.ReadInt(&m_grid_thick_frequency);
    if (rc) rc = file.ReadInt(&m_bShowGrid);
    if (rc) rc = file.ReadInt(&m_bShowGridAxes);
    if (rc) rc = file.ReadInt(&m_bShowWorldAxes);
  }
  return rc;
}



//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmViewPosition
//
ON_3dmViewPosition::ON_3dmViewPosition()
{
  Default();
}

ON_3dmViewPosition::~ON_3dmViewPosition()
{
}

ON_3dmViewPosition::ON_3dmViewPosition(const ON_3dmViewPosition& src)
{
  Default();
  *this = src;
}

ON_3dmViewPosition& ON_3dmViewPosition::operator=(const ON_3dmViewPosition& src)
{
  if ( this != &src ) {
    m_wnd_left   = src.m_wnd_left;
    m_wnd_right  = src.m_wnd_right;
    m_wnd_top    = src.m_wnd_top;
    m_wnd_bottom = src.m_wnd_bottom;
    m_bMaximized = src.m_bMaximized;
    m_floating_viewport = src.m_floating_viewport;

    // reserved fields are not used
    // m_reserved_1 = src.m_reserved_1;
    // m_reserved_2 = src.m_reserved_2;
    // m_reserved_3 = src.m_reserved_3;
  }
  return *this;
}

void ON_3dmViewPosition::Default()
{
  m_wnd_left   = 0.0;
  m_wnd_right  = 1.0;
  m_wnd_top    = 0.0;
  m_wnd_bottom = 1.0;
  m_bMaximized = false;

  m_floating_viewport = 0;
  m_reserved_1 = 0;
  m_reserved_2 = 0;
  m_reserved_3 = 0;
}

bool ON_3dmViewPosition::Write( ON_BinaryArchive& file ) const
{
  int minor_version =  ( file.Archive3dmVersion() >= 5 ) ? 1 : 0;

  bool rc = file.Write3dmChunkVersion(1,minor_version);
  if (rc) 
  {
    if (rc) rc = file.WriteInt( m_bMaximized );
    if (rc) rc = file.WriteDouble( m_wnd_left );
    if (rc) rc = file.WriteDouble( m_wnd_right );
    if (rc) rc = file.WriteDouble( m_wnd_top );
    if (rc) rc = file.WriteDouble( m_wnd_bottom );

    if ( minor_version >= 1 )
    {
      // 13 March 2009 S. Baer
      // version 1.1 added support for m_floating_viewport
      // in V5 files.  This info is not in V4 files because
      // they might use this to write userdata.
      if (rc) rc = file.WriteChar( m_floating_viewport );
    }
  }
  return rc;
}

bool ON_3dmViewPosition::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  double x;
  Default();
  if (rc && major_version==1) 
  {
    if (rc) rc = file.ReadInt( &m_bMaximized );
    if (rc) rc = file.ReadDouble( &m_wnd_left );
    if (rc) rc = file.ReadDouble( &m_wnd_right );
    if (rc) rc = file.ReadDouble( &m_wnd_top );
    if (rc) rc = file.ReadDouble( &m_wnd_bottom );

    // 13 March 2009 S. Baer
    // version 1.1 added support for m_floating_viewport
    if( rc && minor_version >= 1 )
    {
      rc = file.ReadChar( &m_floating_viewport );
    }
  }

  // if people put bogus values in a file, tune them up to something that will work
  if ( m_wnd_left > m_wnd_right ) {
    x = m_wnd_left; m_wnd_left = m_wnd_right; m_wnd_right = x;
  }
  if ( m_wnd_left  < 0.0 ) 
    m_wnd_left  = 0.0; 
  if ( m_wnd_right >= 1.0 ) 
    m_wnd_right = 1.0;
  if ( m_wnd_left >= m_wnd_right ) {
    m_wnd_left = 0.0;
    m_wnd_right = 1.0;
  }
  
  if ( m_wnd_top > m_wnd_bottom ) {
    x = m_wnd_top; m_wnd_top = m_wnd_bottom; m_wnd_bottom = x;
  }
  if ( m_wnd_top  < 0.0 ) 
    m_wnd_top  = 0.0; 
  if ( m_wnd_bottom >= 1.0 )
    m_wnd_bottom = 1.0;
  if ( m_wnd_top >= m_wnd_bottom ) {
    m_wnd_top = 0.0;
    m_wnd_bottom = 1.0;
  }

  return rc;
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmViewTraceImage
//
ON_3dmViewTraceImage::ON_3dmViewTraceImage()
{
  Default();
}

ON_3dmViewTraceImage::~ON_3dmViewTraceImage()
{
}

void ON_3dmViewTraceImage::Default()
{
  m_plane = ON_xy_plane;
  m_width = 0.0;
  m_height = 0.0;
  m_bitmap_filename.Destroy();
  m_bGrayScale = true;
  m_bHidden = false;
  m_bFiltered = false;
}

bool ON_3dmViewTraceImage::Write( ON_BinaryArchive& file ) const
{
  // opennurbs version  < 200307300 - version 1.0 or 1.1 chunk
  // opennurbs version >= 200307300 - version 1.2 chunk
  bool rc = file.Write3dmChunkVersion(1,3);
  if (rc) 
  {
    if (rc) rc = file.WriteString( m_bitmap_filename );
    if (rc) rc = file.WriteDouble( m_width );
    if (rc) rc = file.WriteDouble( m_height );
    if (rc) rc = file.WritePlane( m_plane );

    // version 1.1
    if (rc) rc = file.WriteBool( m_bGrayScale );

    // version 1.2
    if (rc) rc = file.WriteBool( m_bHidden );
    
    // version 1.3
    if (rc) rc = file.WriteBool( m_bFiltered );
  }
  return rc;
}


bool ON_3dmViewTraceImage::Read( ON_BinaryArchive& file )
{
  // opennurbs version  < 200307300 - version 1.0 or 1.1 chunk
  // opennurbs version >= 200307300 - version 1.2 chunk
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1)
  {
    if (rc) rc = file.ReadString( m_bitmap_filename );
    if (rc) rc = file.ReadDouble( &m_width );
    if (rc) rc = file.ReadDouble( &m_height );
    if (rc) rc = file.ReadPlane( m_plane );
    if ( minor_version >= 1 )
    {
      if (rc) rc = file.ReadBool(&m_bGrayScale);
      
      if ( minor_version >= 2 )
      {
        if (rc) rc = file.ReadBool(&m_bHidden);
        
        if ( minor_version >= 3 )
        {
          if (rc) rc = file.ReadBool( &m_bFiltered );
        }
      }
    }
  }
  else
    rc = false;
  return rc;
}


bool ON_3dmViewTraceImage::operator==( const ON_3dmViewTraceImage& other ) const
{
  if ( m_plane != other.m_plane )
    return false;
  if ( m_width != other.m_width )
    return false;
  if ( m_height != other.m_height )
    return false;
  if( m_bitmap_filename != other.m_bitmap_filename )
    return false;
  if ( m_bHidden != other.m_bHidden )
    return false;
  if ( m_bGrayScale != other.m_bGrayScale )
    return false;
  if ( m_bFiltered != other.m_bFiltered )
    return false;
    
  return true;
}

bool ON_3dmViewTraceImage::operator!=( const ON_3dmViewTraceImage& other ) const
{
  return operator==(other) ? false : true;
}


ON_3dmWallpaperImage::ON_3dmWallpaperImage()
{
  Default();
}

ON_3dmWallpaperImage::~ON_3dmWallpaperImage()
{
}

bool ON_3dmWallpaperImage::operator==( const ON_3dmWallpaperImage& other ) const
{
  if ( m_bitmap_filename != other.m_bitmap_filename )
    return false;
  if ( m_bHidden != other.m_bHidden )
    return false;
  return ( m_bGrayScale == other.m_bGrayScale );
}

bool ON_3dmWallpaperImage::operator!=( const ON_3dmWallpaperImage& other ) const
{
  return operator==(other) ? false : true;
}

void ON_3dmWallpaperImage::Default()
{
  m_bitmap_filename.Destroy();
  m_bGrayScale = true;
  m_bHidden = false;
}

bool ON_3dmWallpaperImage::Write( ON_BinaryArchive& file ) const
{
  // version  < 200307300 - version 1.0 chunk
  // version >= 200307300 - version 1.1 chunk
  bool rc = file.Write3dmChunkVersion(1,1);
  if (rc) 
  {
    if (rc) rc = file.WriteString( m_bitmap_filename );
    if (rc) rc = file.WriteBool( m_bGrayScale );

    if (rc) rc = file.WriteBool( m_bHidden ); // added in 1.1 chunk
  }
  return rc;
}

bool ON_3dmWallpaperImage::Read( ON_BinaryArchive& file )
{
  // version  < 200307300 - version 1.0 chunk
  // version >= 200307300 - version 1.1 chunk
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1)
  {
    if (rc) rc = file.ReadString( m_bitmap_filename );
    if (rc) rc = file.ReadBool( &m_bGrayScale );

    if ( minor_version >= 1 )
    {
      if (rc) rc = file.ReadBool( &m_bHidden );
    }
  }
  else
    rc = false;
  return rc;
}


//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmView
//

ON_3dmPageSettings::ON_3dmPageSettings()
{
  Default();
}

ON_3dmPageSettings::~ON_3dmPageSettings()
{
}

void ON_3dmPageSettings::Default()
{
  m_page_number = 0;

  m_width_mm  = 0.0;
  m_height_mm = 0.0;

  m_left_margin_mm   = 0.0;
  m_right_margin_mm  = 0.0;
  m_top_margin_mm    = 0.0;
  m_bottom_margin_mm = 0.0;

  m_printer_name.Destroy();
}


bool ON_3dmPageSettings::IsValid( ON_TextLog* text_log ) const
{
  bool rc = true;

  if ( m_width_mm != 0.0 || m_height_mm != 0.0 )
  {
    if ( !ON_IsValid(m_width_mm) || m_width_mm <= 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_width_mm = %g (should be > 0.0).\n",m_width_mm);
      }
      rc = false;
    }
    if ( !ON_IsValid(m_height_mm) || m_height_mm <= 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_height_mm = %g (should be > 0.0).\n",m_height_mm);
      }
      rc = false;
    }
    if ( !ON_IsValid(m_top_margin_mm) || m_top_margin_mm < 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_top_margin_mm = %g (should be >= 0.0).\n",m_top_margin_mm);
      }
      rc = false;
    }
    if ( !ON_IsValid(m_bottom_margin_mm) || m_bottom_margin_mm < 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_bottom_margin_mm = %g (should be >= 0.0).\n",m_bottom_margin_mm);
      }
      rc = false;
    }
    if ( !ON_IsValid(m_left_margin_mm) || m_left_margin_mm < 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_left_margin_mm = %g (should be >= 0.0).\n",m_left_margin_mm);
      }
      rc = false;
    }
    if ( !ON_IsValid(m_right_margin_mm) || m_right_margin_mm < 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_right_margin_mm = %g (should be >= 0.0).\n",m_right_margin_mm);
      }
      rc = false;
    }
    if ( m_left_margin_mm + m_right_margin_mm >= m_width_mm )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_left_margin_mm+m_right_margin_mm = %g > %g = m_width_mm.\n",m_left_margin_mm + m_right_margin_mm, m_width_mm);
      }
      rc = false;
    }
    if ( m_top_margin_mm + m_bottom_margin_mm >= m_height_mm )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_top_margin_mm+m_bottom_margin_mm = %g > %g = m_height_mm.\n",m_top_margin_mm + m_bottom_margin_mm, m_height_mm);
      }
      rc = false;
    }
  }
  else
  {
    if ( m_top_margin_mm != 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_top_margin_mm = %g (should be 0.0).\n",m_top_margin_mm);
      }
      rc = false;
    }
    if ( m_bottom_margin_mm != 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_bottom_margin_mm = %g (should be 0.0).\n",m_bottom_margin_mm);
      }
      rc = false;
    }
    if ( m_left_margin_mm != 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_left_margin_mm = %g (should be 0.0).\n",m_left_margin_mm);
      }
      rc = false;
    }
    if ( m_right_margin_mm != 0.0 )
    {
      if ( text_log )
      {
        text_log->Print("ON_3dmPageSettings has m_right_margin_mm = %g (should be 0.0).\n",m_right_margin_mm);
      }
      rc = false;
    }
  }

  return rc;
}

bool ON_3dmPageSettings::Write(ON_BinaryArchive& archive) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
  if ( !rc )
    return false;

  for(;;)
  {
    rc = archive.WriteInt( m_page_number );
    if (!rc) break;

    rc = archive.WriteDouble(m_width_mm);
    if ( !rc) break;

    rc = archive.WriteDouble(m_height_mm);
    if ( !rc) break;

    rc = archive.WriteDouble(m_left_margin_mm);
    if ( !rc) break;

    rc = archive.WriteDouble(m_right_margin_mm);
    if ( !rc) break;

    rc = archive.WriteDouble(m_top_margin_mm);
    if ( !rc) break;

    rc = archive.WriteDouble(m_bottom_margin_mm);
    if ( !rc) break;

    rc = archive.WriteString(m_printer_name);
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

bool ON_3dmPageSettings::Read(ON_BinaryArchive& archive)
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = archive.ReadInt(&m_page_number );
    if (!rc) break;

    rc = archive.ReadDouble(&m_width_mm);
    if ( !rc) break;

    rc = archive.ReadDouble(&m_height_mm);
    if ( !rc) break;

    rc = archive.ReadDouble(&m_left_margin_mm);
    if ( !rc) break;

    rc = archive.ReadDouble(&m_right_margin_mm);
    if ( !rc) break;

    rc = archive.ReadDouble(&m_top_margin_mm);
    if ( !rc) break;

    rc = archive.ReadDouble(&m_bottom_margin_mm);
    if ( !rc) break;

    rc = archive.ReadString(m_printer_name);
    if (!rc) break;

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmView
//
ON_3dmView::ON_3dmView()
{
  Default();
} 

ON_3dmView::~ON_3dmView()
{
}

void ON_3dmView::Dump( ON_TextLog& dump ) const
{
  const wchar_t* wsViewName = m_name;
  if ( !wsViewName )
    wsViewName = L"";
  ON::view_projection proj = m_vp.Projection();
  

  ON_3dPoint camLoc;
  ON_3dVector camX, camY, camZ;
  ON_BOOL32 bValidCamera = m_vp.GetCameraFrame( camLoc, camX, camY, camZ );
  double frus_left,frus_right,frus_bottom,frus_top,frus_near,frus_far;
  ON_BOOL32 bValidFrustum = m_vp.GetFrustum(&frus_left,&frus_right,&frus_bottom,&frus_top,&frus_near,&frus_far);
  int port_left, port_right, port_bottom, port_top, port_near, port_far;
  ON_BOOL32 bValidPort = m_vp.GetScreenPort(&port_left,&port_right,&port_bottom,&port_top,&port_near,&port_far);

  const char* sProjectionName;
  switch(proj)
  {
  case ON::parallel_view: sProjectionName = "parallel"; break;
  case ON::perspective_view: sProjectionName = "perspective"; break;
  case ON::unknown_view:
  default: 
    sProjectionName = "unknown";
    break;
  }
  dump.Print("Viewport: name = \"%ls\" projection = %s\n",wsViewName,sProjectionName);

  dump.PushIndent();

  if ( bValidCamera ) 
  {
    dump.Print("viewport camera frame\n"
           "  location = %g, %g, %g\n"
           "  X = %g, %g, %g\n"
           "  Y = %g, %g, %g\n"
           "  Z = %g, %g, %g\n",
           camLoc.x,camLoc.y,camLoc.z,
           camX.x,camX.y,camX.z,
           camY.x,camY.y,camY.z,
           camZ.x,camZ.y,camZ.z
           );
    ON_3dPoint target_point = TargetPoint();
    double target_distance = target_point.DistanceTo( camLoc );
    dump.Print("camera target\n"
               "  distance = %g\n"
               "  point = %g,%g,%g\n",
               target_distance,
               target_point.x,target_point.y,target_point.z
               );
  }


  if ( bValidFrustum ) {
    dump.Print("view frustum\n"
           "  left   = %g, right = %g\n"
           "  bottom = %g, top   = %g\n"
           "  near   = %g, far   = %g\n",
           frus_left,frus_right,
           frus_bottom,frus_top,
           frus_near,frus_far
           );
  }

  if ( bValidPort ) {
    // location of viewport window on screen
    dump.Print("viewport window screen location\n"
           "  left   = %4d, right = %4d\n"
           "  bottom = %4d, top   = %4d\n"
           "  near   = %4d, far   = %4d\n",
           port_left,port_right,
           port_bottom,port_top,
           port_near,port_far
           );
  }


  // relative position of viewport window in application main frame
  double rel_left,rel_right,rel_bottom,rel_top;
  rel_left = m_position.m_wnd_left;
  rel_right = m_position.m_wnd_right;
  rel_bottom = m_position.m_wnd_bottom;
  rel_top = m_position.m_wnd_top;
  dump.Print("relative viewport window position in application frame window\n"
             "  left   = %6.2f%%, right = %6.2f%%\n"
             "  bottom = %6.2f%%, top   = %6.2f%%\n",
             100.0*rel_left, 100.0*rel_right,
             100.0*rel_bottom, 100.0*rel_top
             );

  dump.PopIndent();

}

void ON_3dmView::Default()
{
  m_name.Destroy();

  m_vp.Initialize();
  // ON_3dmView::m_target is obsolete - keep it in sync with m_vp.m_target_point
  OBSOLETE_3DM_VIEW_TARGET = m_vp.TargetPoint(); 

  m_cplane.Default();
  m_display_mode_id = ON_nil_uuid;
  m_display_mode = ON::wireframe_display;
  m_view_type = ON::model_view_type;
  m_position.Default();
  if ( m_vp.Projection() == ON::parallel_view ) {
    m_cplane.m_plane.CreateFromFrame( m_cplane.m_plane.origin, m_vp.CameraX(), m_vp.CameraY() );
  }
  m_bShowConstructionGrid = true;
  m_bShowConstructionAxes = true;
  m_bShowWorldAxes = true;

  m_trace_image.Default();
  m_wallpaper_image.Default();

  m_page_settings.Default();

  m_bLockedProjection = false;
}

ON_3dPoint ON_3dmView::TargetPoint() const
{
  ON_3dPoint target_point = m_vp.TargetPoint();
  if ( OBSOLETE_3DM_VIEW_TARGET != target_point )
  {
    ON_ERROR("Obsolete ON_3dmView::m_target is not set correctly");
    const_cast<ON_3dmView*>(this)->OBSOLETE_3DM_VIEW_TARGET = target_point; // fix error condition
  }
  return target_point;
}

bool ON_3dmView::SetTargetPoint(ON_3dPoint target_point)
{
  bool rc = m_vp.SetTargetPoint(target_point);
  OBSOLETE_3DM_VIEW_TARGET = m_vp.TargetPoint(); // keep obsolete m_target in sync with m_vp.m_target_point
  return rc;
}

bool ON_3dmView::IsValid(ON_TextLog* text_log) const
{
  bool rc = m_vp.IsValid(text_log)?true:false;
  while(rc)
  {
    switch(m_view_type)
    {
    case ON::model_view_type:
      if ( m_page_settings.m_width_mm != 0.0 || m_page_settings.m_height_mm != 0.0 )
      {
        if ( text_log )
        {
          text_log->Print("ON_3dmView has m_view_type = ON::model_view_type but m_page_settings width,height = (%g,%g) (both should be zero).\n",
                          m_page_settings.m_width_mm,
                          m_page_settings.m_height_mm);
        }
        rc = false;
      }
      //if (    m_nested_view_position.m_min.x != 0.0 || m_nested_view_position.m_max.x != 0.0
      //     || m_nested_view_position.m_min.y != 0.0 || m_nested_view_position.m_max.y != 0.0
      //     || m_nested_view_position.m_min.z != 0.0 || m_nested_view_position.m_max.z != 0.0 )
      //{
      //  if ( text_log )
      //  {
      //    text_log->Print("ON_3dmView has m_view_type = ON::model_view_type and m_nested_view_position is not identically zero.\n");
      //  }
      //  rc = false;
      //}
      //if ( !ON_UuidIsNil(m_parent_viewport_id) )
      //{
      //  if ( text_log )
      //  {
      //    text_log->Print("ON_3dmView has m_view_type = ON::model_view_type and m_parent_viewport_id is not nil\n");
      //  }
      //  rc = false;
      //}
      break;
    case ON::page_view_type:
      //if (    m_nested_view_position.m_min.x != 0.0 || m_nested_view_position.m_max.x != 0.0
      //     || m_nested_view_position.m_min.y != 0.0 || m_nested_view_position.m_max.y != 0.0
      //     || m_nested_view_position.m_min.z != 0.0 || m_nested_view_position.m_max.z != 0.0 )
      //{
      //  if ( text_log )
      //  {
      //    text_log->Print("ON_3dmView has m_view_type = ON::page_view_type and m_nested_view_position is not identically zero.\n");
      //  }
      //  rc = false;
      //}
      //if ( !ON_UuidIsNil(m_parent_viewport_id) )
      //{
      //  if ( text_log )
      //  {
      //    text_log->Print("ON_3dmView has m_view_type = ON::page_view_type and m_parent_viewport_id is not nil\n");
      //  }
      //  rc = false;
      //}
      if ( m_page_settings.m_width_mm <= 0.0 || m_page_settings.m_height_mm <= 0.0 )
      {
        if ( text_log )
        {
          text_log->Print("ON_3dmView has m_view_type = ON::page_view_type but page width,height = (%g,%g)\n",
                          m_page_settings.m_width_mm,
                          m_page_settings.m_height_mm);
        }
        rc = false;
      }
      break;

    case ON::nested_view_type:
      if ( m_page_settings.m_width_mm != 0.0 || m_page_settings.m_height_mm != 0.0 )
      {
        if ( text_log )
        {
          text_log->Print("ON_3dmView has m_view_type = ON::nested_view_type but m_page_settings width,height = (%g,%g) (both should be zero).\n",
                          m_page_settings.m_width_mm,
                          m_page_settings.m_height_mm);
        }
        rc = false;
      }
      //if ( ON_UuidIsNil(m_parent_viewport_id) )
      //{
      //  if ( text_log )
      //  {
      //    text_log->Print("ON_3dmView has m_view_type = ON::nested_view_type and m_parent_viewport_id is nil.\n");
      //  }
      //  rc = false;
      //}

      //if ( !m_nested_view_position.IsValid() 
      //     || m_nested_view_position.m_min.x >= m_nested_view_position.m_max.x
      //     || m_nested_view_position.m_min.y >= m_nested_view_position.m_max.y
      //     || m_nested_view_position.m_min.z != m_nested_view_position.m_max.z )
      //{
      //  if ( text_log )
      //  {
      //    text_log->Print("ON_3dmView has m_view_type = ON::nested_view_type and m_nested_view_position is bogus.\n");
      //  }
      //  rc = false;
      //}
      break;

    default:
      if ( text_log )
      {
        text_log->Print("ON_3dmView m_view_type = %d (illegal enum value)\n",m_view_type);
      }
      rc = false;
      break;
    }
    if (rc)
      break;




    break;
  }
  return rc;
}

bool ON_3dmView::Write( ON_BinaryArchive& file ) const
{
  // Everything in a view is in a subchunk so new records can 
  // be added to a view and old I/O code will still
  // work.
  bool rc = true;

  // 27 June 2008 Dale Lear
  //   I added support for saving userdata attached to
  //   the m_vp ON_Viewport.  Ideally, I would just call
  //   file.WriteObject(m_vp), but userdata support is being
  //   added years after millions of files have been written
  //   by calling m_vp.Write().
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_VIEWPORT, 0 );
    if(rc) {
      rc = m_vp.Write(file)?true:false;
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc && 0 != m_vp.FirstUserData() && file.Archive3dmVersion() >= 4) 
  {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_VIEWPORT_USERDATA, 0 );
    if(rc) 
    {
      rc = file.WriteObjectUserData(m_vp);
      // write a "fake" TCODE_OPENNURBS_CLASS_END end of class 
      // mark so I can use
      // ON_BinaryArchive::ReadObjectUserData() 
      // to read this user data.
      if ( file.BeginWrite3dmChunk( TCODE_OPENNURBS_CLASS_END, 0 ) ) 
      {
        if ( !file.EndWrite3dmChunk() )
          rc = false;
      }
      else
      {
        rc = false;
      }
      if ( !file.EndWrite3dmChunk() ) // end of TCODE_VIEW_VIEWPORT_USERDATA
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_CPLANE, 0 );
    if(rc) {
      rc = m_cplane.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_TARGET, 0 );
    if(rc) {
      ON_3dPoint target_point = TargetPoint();
      if(rc) rc = file.WritePoint(target_point);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_DISPLAYMODE, m_display_mode );
    if(rc) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_POSITION, 0 );
    if(rc) {
      if(rc) rc = m_position.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_SHOWCONGRID, m_bShowConstructionGrid );
    if(rc) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_SHOWCONAXES, m_bShowConstructionAxes );
    if(rc) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_SHOWWORLDAXES, m_bShowWorldAxes );
    if(rc) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_NAME, 0 );
    if(rc) {
      if(rc) rc = file.WriteString(m_name);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_TRACEIMAGE, 0 );
    if(rc) 
    {
      if(rc) 
        rc = m_trace_image.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc) 
  {
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_WALLPAPER, 0 );
    if(rc) 
    {
      if(rc) rc = file.WriteString(m_wallpaper_image.m_bitmap_filename);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  if(rc && file.Archive3dmVersion() >= 3 ) 
  {
    // Added 5 June 2003 to support additional wallpaper attributes.
    // Older versions of Rhino/opennurbs
    // will just skip this chunk and get filename from the
    // TCODE_VIEW_WALLPAPER chunk written above.
    rc = file.BeginWrite3dmChunk( TCODE_VIEW_WALLPAPER_V3, 0 );
    if(rc) 
    {
      if(rc) 
        rc = m_wallpaper_image.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  if (rc && file.Archive3dmVersion() >= 4)
  {
    // 23 March 2005 Dale Lear:
    //   The "chunks" above trace their history back to Rhino 1.0;
    //   The TCODE_VIEW_ATTRIBUTES chunk uses a chunk version so that
    //   new view information can be added without inventing a new 
    //   TCODE for each new piece of information.

    rc = file.BeginWrite3dmChunk( TCODE_VIEW_ATTRIBUTES, 0 );
    if (rc)
    {
      rc = file.Write3dmChunkVersion( 1, 4 ); // (there are no 1.0 fields)

      while(rc)
      {
        // 1.1 fields (there are no 1.0 fields)
        rc = file.WriteInt( m_view_type );
        if (!rc) break;
        
        // obsolete values - superceded by m_page_settings
        rc = file.WriteDouble( m_page_settings.m_width_mm );
        if (!rc) break;

        rc = file.WriteDouble( m_page_settings.m_height_mm );
        if (!rc) break;

        ON_UUID obsolete_parent_viewport_id;
        memset(&obsolete_parent_viewport_id,0,sizeof(obsolete_parent_viewport_id));
        rc = file.WriteUuid( obsolete_parent_viewport_id );
        if (!rc) break;

        ON_BoundingBox obsolete_nested_view_position;
        rc = file.WriteBoundingBox( obsolete_nested_view_position );
        if (!rc) break;

        // 28 feb 2006 version 1.2 fields
        rc = file.WriteUuid(m_display_mode_id);
        if (!rc) break;

        rc = m_page_settings.Write(file);
        if (!rc) break;

        // 7 March 2006 version 1.3 fields
        rc = file.WriteBool(m_bLockedProjection);
        if (!rc) break;

        // 12 December 2010 version 1.4
        rc = file.WriteArray(m_clipping_planes);
        if (!rc) break;

        break;
      }

      // Since BeginWrite3dmChunk() returned true, EndWrite3dmChunk()
      // must be called even when rc is false
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // required TCODE_ENDOFTABLE chunk - marks end of view table
  if ( rc ) {
    rc = file.BeginWrite3dmChunk( TCODE_ENDOFTABLE, 0 );
    if ( rc ) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }
  return rc;
}

bool ON_3dmView::Read( ON_BinaryArchive& file )
{
  // Everything in a view is in a subchunk so new records can 
  // be added to a view and old I/O code will still
  // work.
  unsigned int tcode = 0;
  ON__INT64 big_value = 0;
  int i32;
  bool rc = true;

  Default();

  bool bHaveTargetPoint = false;
  bool bHaveViewport = false;
  ON_3dPoint target_point = ON_3dPoint::UnsetPoint;

  while(rc) {
    rc = file.BeginRead3dmBigChunk(&tcode,&big_value);
    if (!rc)
      break;
    switch(tcode) 
    {
    case TCODE_VIEW_CPLANE:
      rc = m_cplane.Read(file);
      break;
    case TCODE_VIEW_VIEWPORT:
      rc = m_vp.Read(file)?true:false;
      if (rc)
        bHaveViewport = true;
      break;
    case TCODE_VIEW_VIEWPORT_USERDATA:
      // 27 June 2008 Dale Lear
      //   I added support for saving userdata attached to
      //   the m_vp ON_Viewport.  Ideally, the ON_Viewport
      //   would be read by calling file.ReadObject(), but
      //   userdata support is being added years after 
      //   millions of files have been written by calling
      //   m_vp.Write()/Read().
      rc = file.ReadObjectUserData(m_vp);
      break;
    case TCODE_VIEW_SHOWCONGRID:
      m_bShowConstructionGrid = big_value?true:false;
      break;
    case TCODE_VIEW_SHOWCONAXES:
      m_bShowConstructionAxes = big_value?true:false;
      break;
    case TCODE_VIEW_SHOWWORLDAXES:
      m_bShowWorldAxes = big_value?true:false;
      break;
    case TCODE_VIEW_TRACEIMAGE:
      rc = m_trace_image.Read(file);
      break;
    case TCODE_VIEW_WALLPAPER:
      // used prior to 5 June 2003 and still written
      // after 5 June 2003 so older Rhinos/opennurbs
      // will not loose the filename information.
      rc = file.ReadString(m_wallpaper_image.m_bitmap_filename);
      m_wallpaper_image.m_bGrayScale = true;
      break;
    case TCODE_VIEW_WALLPAPER_V3:
      // Added 5 June 2003 to support additional wallpaper attributes.
      rc = m_wallpaper_image.Read(file);
      break;
    case TCODE_VIEW_TARGET:
      rc = file.ReadPoint(target_point);
      if (rc)
        bHaveTargetPoint = true;
      break;
    case TCODE_VIEW_DISPLAYMODE:
      i32 = (int)big_value;
      m_display_mode = ON::DisplayMode(i32);
      break;
    case TCODE_VIEW_NAME:
      rc = file.ReadString(m_name);
      break;
    case TCODE_VIEW_POSITION:
      rc = m_position.Read(file);
      break;

    case TCODE_VIEW_ATTRIBUTES:
      {
        int major_version = 0;
        int minor_version = 0;
        rc = file.Read3dmChunkVersion(&major_version,&minor_version);
        // there are no 1.0 fields in this chunk
        while ( rc 
                && major_version == 1 && minor_version >= 1 
                && file.Archive3dmVersion() >= 4 
                && file.ArchiveOpenNURBSVersion() >= 200503170 )
        {
          // Added 23 March 2005 Dale Lear
          // 1.1 fields (there are no 1.0 fields)
          i32 = 0;
          rc = file.ReadInt( &i32 );
          if (!rc) break;
          m_view_type = ON::ViewType(i32);
          
          rc = file.ReadDouble( &m_page_settings.m_width_mm );
          if (!rc) break;

          rc = file.ReadDouble( &m_page_settings.m_height_mm );
          if (!rc) break;

          ON_UUID obsolete_parent_viewport_id;
          rc = file.ReadUuid( obsolete_parent_viewport_id );
          if (!rc) break;

          ON_BoundingBox obsolete_nested_view_position;
          rc = file.ReadBoundingBox( obsolete_nested_view_position );
          if (!rc) break;

          if ( minor_version >= 2 )
          {
            // 28 feb 2006 version 1.2 field
            rc = file.ReadUuid(m_display_mode_id);
            if (!rc) break;

            rc = m_page_settings.Read(file);
            if (!rc) break;

            if ( minor_version >= 3 )
            {
              rc = file.ReadBool(&m_bLockedProjection);
              if (!rc) break;

              if ( minor_version >= 4 )
              {
                rc = file.ReadArray(m_clipping_planes);
                if (!rc) break;
              }
            }
          }

          // Add new inforamation here - ask Dale Lear for help.

          break;
        }
      }
      break;
    }

    if (!file.EndRead3dmChunk())
      rc = false;
    if ( tcode == TCODE_ENDOFTABLE )
      break;
  }

  if (    bHaveViewport 
       && bHaveTargetPoint
       && target_point.IsValid()
       && !OBSOLETE_3DM_VIEW_TARGET.IsValid()
     )
  {
    // m_target is obsolete, but some older files
    // have the good value stored in ON_3dmView. In this
    // case use the good value as the target point.
    SetTargetPoint(target_point); // sets both this->m_target and m_vp.m_target_point
  }
  else
  {
    // Assume the value on m_vp.m_target_point is the right one and
    // Keep the obsolete m_target in sync with m_vp.m_target_point.
    OBSOLETE_3DM_VIEW_TARGET = m_vp.TargetPoint();
  }

  return rc;
}

ON_EarthAnchorPoint::ON_EarthAnchorPoint()
{
  Default();
}

ON_EarthAnchorPoint::~ON_EarthAnchorPoint()
{
}

void ON_EarthAnchorPoint::Default()
{
  m_earth_basepoint_latitude = 0.0;
  m_earth_basepoint_longitude = 0.0;
  m_earth_basepoint_elevation = 0.0;
  m_earth_basepoint_elevation_zero = 0;

  m_model_basepoint.Set(0.0,0.0,0.0);
  m_model_north.Set(0.0,1.0,0.0);
  m_model_east.Set(1.0,0.0,0.0);

  m_id = ON_nil_uuid;
  m_name.Destroy();
  m_description.Destroy(); 
  m_url.Destroy();
  m_url_tag.Destroy();
}

int ON_EarthAnchorPoint::CompareEarthLocation(const ON_EarthAnchorPoint* a, const ON_EarthAnchorPoint* b)
{
  if ( !a )
  {
    return b ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }

  double xa = a->m_earth_basepoint_longitude;
  double xb = b->m_earth_basepoint_longitude;
  if ( !ON_IsValid(xa) )
  {
    if ( ON_IsValid(xb) ) return -1;
  }
  else if ( !ON_IsValid(xb) )
  {
    return 1;
  }
  else
  {
    while(xa <= 0.0)
      xa += 360.0;
    while(xa > 360.0)
      xa -= 360.0;
    while(xb <= 0.0)
      xb += 360.0;
    while(xb > 360.0)
      xb -= 360.0;
    if ( xa < xb ) return -1;
    if ( xa > xb ) return 1;
  }

  xa = a->m_earth_basepoint_latitude;
  xb = b->m_earth_basepoint_latitude;
  if ( !ON_IsValid(xa) )
  {
    if ( ON_IsValid(xb) ) return -1;
  }
  else if ( !ON_IsValid(xb) )
  {
    return 1;
  }
  else
  {
    while(xa <= 0.0)
      xa += 360.0;
    while(xa > 360.0)
      xa -= 360.0;
    while(xb <= 0.0)
      xb += 360.0;
    while(xb > 360.0)
      xb -= 360.0;
    if ( xa < xb ) return -1;
    if ( xa > xb ) return 1;
  }

  int i = a->m_earth_basepoint_elevation_zero - b->m_earth_basepoint_elevation_zero;
  if ( i != 0 )
    return i;

  xa = a->m_earth_basepoint_elevation;
  xb = b->m_earth_basepoint_elevation;
  if ( !ON_IsValid(xa) )
  {
    if ( ON_IsValid(xb) ) return -1;
  }
  else if ( !ON_IsValid(xb) )
  {
    return 1;
  }
  else
  {
    if ( xa < xb ) return -1;
    if ( xa > xb ) return 1;
  }

  return 0;   
}

int ON_EarthAnchorPoint::CompareModelDirection(const ON_EarthAnchorPoint* a, const ON_EarthAnchorPoint* b)
{
  if ( !a )
  {
    return b ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }

  int i = ON_ComparePoint(3,false,&a->m_model_basepoint.x,&b->m_model_basepoint.x);
  if ( !i )
  {
    i = ON_ComparePoint(3,false,&a->m_model_north.x,&b->m_model_north.x);
    if ( !i )
    {
      i = ON_ComparePoint(3,false,&a->m_model_east.x,&b->m_model_east.x);
    }
  }
  return i;  
}

int ON_EarthAnchorPoint::CompareIdentification(const ON_EarthAnchorPoint* a, const ON_EarthAnchorPoint* b)
{
  if ( !a )
  {
    return b ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }

  int i = ON_UuidCompare(a->m_id,b->m_id);
  if ( !i)
  {
    i = a->m_name.Compare(b->m_name);
    if (!i)
    {
      i = a->m_description.Compare(b->m_description);
      if (!i)
      {
        i = a->m_url.CompareNoCase(b->m_url);
        if ( !i)
        {
          i = a->m_url_tag.Compare(b->m_url_tag);
        }
      }
    }
  }
  return i;  
}

int ON_EarthAnchorPoint::Compare(const ON_EarthAnchorPoint* a, const ON_EarthAnchorPoint* b)
{
  int i = ON_EarthAnchorPoint::CompareEarthLocation(a,b);
  if ( !i)
  {
    i = ON_EarthAnchorPoint::CompareModelDirection(a,b);
    if (!i)
    {
      i = ON_EarthAnchorPoint::CompareIdentification(a,b);
    }
  }
  return i;
}

bool ON_EarthAnchorPoint::Read( ON_BinaryArchive& file )
{
  Default();
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = file.ReadDouble(&m_earth_basepoint_latitude);
    if (!rc) break;
    rc = file.ReadDouble(&m_earth_basepoint_longitude);
    if (!rc) break;
    rc = file.ReadDouble(&m_earth_basepoint_elevation);
    if (!rc) break;
    rc = file.ReadPoint(m_model_basepoint);
    if (!rc) break;
    rc = file.ReadVector(m_model_north);
    if (!rc) break;
    rc = file.ReadVector(m_model_east);
    if (!rc) break;

    if ( minor_version >= 1 )
    {
      // 1.1 fields
      rc = file.ReadInt(&m_earth_basepoint_elevation_zero);
      if (!rc) break;
      rc = file.ReadUuid(m_id);
      if (!rc) break;
      rc = file.ReadString(m_name);
      if (!rc) break;
      rc = file.ReadString(m_description);
      if (!rc) break;
      rc = file.ReadString(m_url);
      if (!rc) break;
      rc = file.ReadString(m_url_tag);
      if (!rc) break;
    }

    break;
  }

  if ( !file.EndRead3dmChunk() )
    rc = false;

  return rc;
}

bool ON_EarthAnchorPoint::Write( ON_BinaryArchive& file ) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if ( !rc )
    return false;

  for(;;)
  {
    rc = file.WriteDouble(m_earth_basepoint_latitude);
    if (!rc) break;
    rc = file.WriteDouble(m_earth_basepoint_longitude);
    if (!rc) break;
    rc = file.WriteDouble(m_earth_basepoint_elevation);
    if (!rc) break;
    rc = file.WritePoint(m_model_basepoint);
    if (!rc) break;
    rc = file.WriteVector(m_model_north);
    if (!rc) break;
    rc = file.WriteVector(m_model_east);
    if (!rc) break;

    // 1.1 fields
    rc = file.WriteInt(m_earth_basepoint_elevation_zero);
    if (!rc) break;
    rc = file.WriteUuid(m_id);
    if (!rc) break;
    rc = file.WriteString(m_name);
    if (!rc) break;
    rc = file.WriteString(m_description);
    if (!rc) break;
    rc = file.WriteString(m_url);
    if (!rc) break;
    rc = file.WriteString(m_url_tag);
    if (!rc) break;


    break;
  }

  if ( !file.EndWrite3dmChunk() )
    rc = false;

  return rc;
}


bool ON_EarthAnchorPoint::GetModelCompass(ON_Plane& model_compass) const
{
  ON_Plane mc;
  mc.xaxis = m_model_east;
  mc.yaxis = m_model_north;
  if ( fabs(mc.xaxis.Length() - 1.0) > ON_SQRT_EPSILON )
  {
    if ( !mc.xaxis.Unitize() )
      return false;
  }
  if ( fabs(mc.yaxis.Length() - 1.0) > ON_SQRT_EPSILON )
  {
    if ( !mc.yaxis.Unitize() )
      return false;
  }
  double d = mc.xaxis*mc.yaxis;
  if ( fabs(d) > ON_SQRT_EPSILON )
  {
    // assume north is correct
    mc.xaxis.x -= d*mc.yaxis.x;
    mc.xaxis.y -= d*mc.yaxis.y;
    mc.xaxis.z -= d*mc.yaxis.z;
    if( !mc.xaxis.Unitize() )
      return false;
  }
  mc.zaxis = ON_CrossProduct(mc.xaxis,mc.yaxis);
  if ( fabs(mc.zaxis.Length() - 1.0) > ON_SQRT_EPSILON )
  {
    if ( !mc.zaxis.Unitize() )
      return false;
  }
  mc.origin = m_model_basepoint;
  mc.UpdateEquation();
  model_compass = mc;
  return model_compass.IsValid();
}

bool ON_EarthAnchorPoint::GetModelToEarthXform(
          const ON_UnitSystem& model_unit_system,
          ON_Xform& model_to_earth
          ) const
{
  // The orient_model rotates the model so that
  //   xaxis runs from west to east
  //   yaxis runs from south to north
  //   zaxis points "up"
  ON_Plane model_compass;
  bool rc = GetModelCompass( model_compass );
  model_compass.origin = m_model_basepoint;
  model_compass.UpdateEquation();
  ON_Xform orient_model;
  orient_model.Rotation( model_compass, ON_xy_plane  );

  ON_Xform coord_change(1.0);

  const double lat_radians = m_earth_basepoint_latitude/180.0*ON_PI;
  const double cos_lat = cos(lat_radians);
  const double sin_lat = sin(lat_radians);
  
  // get radius of earth at this latitude
  const double earth_polar_radius      = 6356750.0; // Earth's radius at poles (meters)
  const double earth_equatorial_radius = 6378135.0; // Earth's radius at equator (meters)
  ON_2dVector r;
  r.x = cos_lat;
  r.y = sin_lat*(earth_equatorial_radius/earth_polar_radius);
  double earth_radius = earth_equatorial_radius/r.Length();
  if ( earth_radius > earth_equatorial_radius )
    earth_radius = earth_equatorial_radius;
  else if ( earth_radius < earth_polar_radius )
    earth_radius = earth_polar_radius;

  const double meters_per_degree_latitude = earth_radius*ON_PI/180.0; // meters per degree of latitude

  const double model_to_meters_scale = ON::UnitScale(model_unit_system, ON::meters);
  const double north_south_scale  = model_to_meters_scale/meters_per_degree_latitude;
  const double east_west_scale = ( 1.0e100*cos_lat < north_south_scale )
                               ? north_south_scale
                               : north_south_scale/cos_lat;

  coord_change.m_xform[0][0] = east_west_scale;
  coord_change.m_xform[0][3] = m_earth_basepoint_longitude;
  coord_change.m_xform[1][1] = north_south_scale;
  coord_change.m_xform[1][3] = m_earth_basepoint_latitude;
  coord_change.m_xform[2][2] = model_to_meters_scale;
  coord_change.m_xform[3][2] = m_earth_basepoint_elevation;

  model_to_earth = coord_change*orient_model;

  return rc;
}


//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmSettings
//

void ON_3dmSettings::Default()
{
  // default properties
  m_model_URL.Destroy();
  m_model_basepoint.Set(0.0,0.0,0.0);
  m_earth_anchor_point.Default();
  m_ModelUnitsAndTolerances.Default();
  m_PageUnitsAndTolerances.Default();
  m_RenderMeshSettings.Default();
  m_CustomRenderMeshSettings.Default();

  m_IO_settings.Default();
  
  // 28 Febuary 2003 Dale Lear:
  //     Add analysis mesh default settings
  m_AnalysisMeshSettings.DefaultAnalysisMeshParameters();

  m_AnnotationSettings.Default();
  m_named_cplanes.Empty();
  m_named_views.Empty();
  m_views.Empty();
  m_active_view_id = ON_nil_uuid;

  m_current_layer_index = 0;
  m_current_font_index = 0;
  m_current_dimstyle_index = 0;

  m_current_material_index = -1; // -1 = "default" material
  m_current_material_source = ON::material_from_layer;

  m_current_color.SetRGB(0,0,0);
  m_current_color_source = ON::color_from_layer;

  m_current_linetype_index = -1;  // -1 = "default" solid line
  m_current_linetype_source = ON::linetype_from_layer;

  m_current_plot_color = ON_UNSET_COLOR;
  m_current_plot_color_source = ON::plot_color_from_layer;

  m_current_wire_density = 1;
  m_RenderSettings.Default();
  m_GridDefaults.Default();

  m_linetype_display_scale = 1.0;

  m_plugin_list.Destroy();
}

ON_3dmSettings::ON_3dmSettings()
{
  Default();
};

ON_3dmSettings::~ON_3dmSettings()
{
}

ON_3dmIOSettings::ON_3dmIOSettings()
{
  Default();
}

void ON_3dmIOSettings::Default()
{
  m_bSaveTextureBitmapsInFile = false;
  // 7 February 2011 - default changed to 1.
  //m_idef_link_update = 0;
  m_idef_link_update = 1;
}


bool ON_3dmIOSettings::Read(ON_BinaryArchive& file)
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    rc = ( 1 == major_version );
    if (!rc) break;

    rc = file.ReadBool(&m_bSaveTextureBitmapsInFile);
    if(!rc) break;

    rc = file.ReadInt(&m_idef_link_update);
    if(!rc) break;

    if ( 0 == m_idef_link_update && file.Archive3dmVersion() >= 5 )
    {
      // 7 February 2011 - old 0 value is no longer an option.
      m_idef_link_update = 1;
    }

    break;
  }

  if ( !file.EndRead3dmChunk() )
    rc = false;

  return rc;
}

bool ON_3dmIOSettings::Write(ON_BinaryArchive& file) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;
  for(;;)
  {
    rc = file.WriteBool(m_bSaveTextureBitmapsInFile);
    if(!rc) break;

    int i = m_idef_link_update;
    if ( 0 == i && file.Archive3dmVersion() >= 5 )
    {
      // 7 February 2011 - old 0 value is no longer an option.
      i = 1;
    }
    rc = file.WriteInt(i);
    if(!rc) break;

    break;
  }
  if ( !file.EndWrite3dmChunk() )
    rc = false;

  return rc;
}


static bool ON_3dmSettings_Read_v1_TCODE_NAME(ON_BinaryArchive& file, ON_wString& str )
{
  // reads legacy 1.0 named view TCODE_NAME chunk
  str.Empty();
  int len = 0;
  bool rc = file.ReadInt( &len );
  if (rc && len > 0) {
    char* name = (char*)oncalloc( 1, len + 1);
    rc = file.ReadString( len, name );
    if (rc)
      str = name; // ASCII -> UNICODE
    if (name)
      onfree(name);
  }
  return rc;
}

static bool ON_3dmSettings_Read_v1_TCODE_CPLANE(ON_BinaryArchive& file, ON_3dmConstructionPlane& cplane)
{
  // reads legacy 1.0 named view TCODE_CPLANE chunk

  // do NOT call cplane.Default() here
  bool rc = true;
	ON_3dPoint origin;
	ON_3dVector xaxis, yaxis;
	double gridsize;
	int gridsections, gridthicksections;
  if (rc) rc = file.ReadPoint( origin );
  if (rc) rc = file.ReadVector( xaxis );
  if (rc) rc = file.ReadVector( yaxis );
  if (rc) 
  {
    rc = file.ReadDouble(&gridsize);
    if (rc) 
    {
      rc = file.ReadInt(&gridsections);
      if (rc) 
      {
        rc = file.ReadInt(&gridthicksections);
        if (rc) 
        {
          cplane.m_plane.CreateFromFrame(origin,xaxis,yaxis);
          cplane.m_grid_line_count = gridsections;
          cplane.m_grid_thick_frequency = gridthicksections;
          cplane.m_grid_spacing = gridsize;
          cplane.m_snap_spacing = gridsize;
        }
        }
    }
  }
  return rc;
}

static bool ON_3dmSettings_Read_v1_TCODE_VIEW(ON_BinaryArchive& file, ON_3dmView& view)
{
  // reads legacy 1.0 named view TCODE_VIEW chunk
  // do NOT call view.Default() here
  bool rc = true;

	int projection, valid;
	double angle1, angle2, angle3, viewsize, cameradist;
  ON_3dPoint target_point;
  while(rc)
  {
    rc = file.ReadInt(&projection);
    if (!rc) break;
    rc = file.ReadInt(&valid);
    if (!rc) break;
    rc = file.ReadPoint( target_point );
    if (!rc) break;
    rc = file.ReadDouble( &angle1 );
    if (!rc) break;
    rc = file.ReadDouble( &angle2 );
    if (!rc) break;
    rc = file.ReadDouble( &angle3 );
    if (!rc) break;
    rc = file.ReadDouble( &viewsize );
    if (!rc) break;
    rc = file.ReadDouble( &cameradist );
    if (!rc) break;

    if( cameradist <= 0.0 || cameradist >= ( DBL_MAX / 2.0 ))
      cameradist = 100.0;
    if( viewsize <= 0.0 || viewsize >= ( DBL_MAX / 2.0 ))
      viewsize = 0.125;
    ON_ViewportFromRhinoView(
          projection == 2 ? ON::perspective_view : ON::parallel_view,
          target_point,
          angle1,
          angle2,
          angle3,
          viewsize,
          cameradist,
          100, // screen_width, 
          100, // screen_height,
          view.m_vp
          );
    // keep obsolete view.m_target in sync with view.m_vp.m_target_point
    view.OBSOLETE_3DM_VIEW_TARGET = view.m_vp.TargetPoint(); 
    break;
  }

  return rc;
}

static bool ON_3dmSettings_Read_v1_TCODE_NAMED_VIEW(ON_BinaryArchive& file, ON_3dmView& view)
{
  // reads legacy 1.0 named view TCODE_NAMED_VIEW chunk
  view.Default();
  bool rc = true;
  unsigned int tcode;
  ON__INT64 big_value;

  while(rc) 
  {
    rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
    if (!rc )
      break;
    switch(tcode) {

    case TCODE_NAME:
      rc = ON_3dmSettings_Read_v1_TCODE_NAME(file,view.m_name);
      break;

    case TCODE_CPLANE:
      rc = ON_3dmSettings_Read_v1_TCODE_CPLANE(file,view.m_cplane);
      break;

    case TCODE_VIEW:
      rc = ON_3dmSettings_Read_v1_TCODE_VIEW( file, view );
      break;

    case TCODE_SHOWGRID:
      view.m_bShowConstructionGrid = big_value?true:false;
      break;
						
    case TCODE_SHOWGRIDAXES:
      view.m_bShowConstructionAxes = big_value?true:false;
      break;
						
    case TCODE_SHOWWORLDAXES:
      view.m_bShowWorldAxes = big_value?true:false;
      break; 			
      
    }
    if ( !file.EndRead3dmChunk() )
      rc = false;
    if ( tcode == TCODE_ENDOFTABLE )
      break;
  }
  return rc;
}

static bool ON_3dmSettings_Read_v1_TCODE_NAMED_CPLANE(ON_BinaryArchive& file, ON_3dmConstructionPlane& cplane)
{
  // reads legacy 1.0 named construction plane TCODE_NAMED_CPLANE chunk
  cplane.Default();

  bool rc = true;
  unsigned int tcode;
  ON__INT64 big_value;

  while(rc) 
  {
    rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
    if (!rc )
      break;
    switch(tcode) {

    case TCODE_NAME:
      rc = ON_3dmSettings_Read_v1_TCODE_NAME(file, cplane.m_name );
      break;

    case TCODE_CPLANE:
      rc = ON_3dmSettings_Read_v1_TCODE_CPLANE(file, cplane );
      break;
    }
    if ( !file.EndRead3dmChunk() )
      rc = false;
    if ( tcode == TCODE_ENDOFTABLE )
      break;
  }
  return rc;
}

static bool ON_3dmSettings_Read_v1_TCODE_UNIT_AND_TOLERANCES(ON_BinaryArchive& file, ON_3dmUnitsAndTolerances& UnitsAndTolerances )
{
  bool rc = true;
  int v = 0;
  int us = 0;
  UnitsAndTolerances.Default();
  if (rc) 
    rc = file.ReadInt( &v ); // should get v = 1
  if (rc) 
    rc = file.ReadInt( &us );
  switch (us) 
  {
  case 0: // NO_UNIT_SYSTEM:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::no_unit_system;
    break;
  case 1: // MICRONS:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::microns;
    break;
  case 2: // MILLIMETERS:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::millimeters;
    break;
  case 3: // CENTIMETERS:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::centimeters;
    break;
  case 4: // METERS:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::meters;
    break;
  case 5: // KILOMETERS:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::kilometers;
    break;
  case 6: // MICROINCHES:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::microinches;
    break;
  case 7: // MILS:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::mils;
    break;
  case 8: // INCHES:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::inches;
    break;
  case 9: // FEET:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::feet;
    break;
  case 10: // MILES:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::miles;
    break;
  default: // NO_UNIT_SYSTEM:
    UnitsAndTolerances.m_unit_system.m_unit_system = ON::no_unit_system;
    break;
  }
  if (rc) rc = file.ReadDouble( &UnitsAndTolerances.m_absolute_tolerance );
  if (rc) rc = file.ReadDouble( &UnitsAndTolerances.m_relative_tolerance );
  if (rc) rc = file.ReadDouble( &UnitsAndTolerances.m_angle_tolerance );
  return rc;
}

static bool ON_3dmSettings_Read_v1_TCODE_VIEWPORT(ON_BinaryArchive& file, ON_3dmView& view)
{
  // reads legacy 1.0 named construction plane TCODE_VIEWPORT chunk
  view.Default();
  bool rc = true;
  ON__UINT32 tcode;
  ON__INT64 big_value;

  double clipdist = 0.0;
  double snapsize = 0.0;

  int chunk_count = 0;// debugging counter
  for ( chunk_count = 0; rc; chunk_count++ )
  {
    rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
    if (!rc )
      break;
    switch(tcode) {

    case TCODE_NEAR_CLIP_PLANE:
      rc = file.ReadDouble(&clipdist);
      break;

    case TCODE_SNAPSIZE:
      rc = file.ReadDouble(&snapsize);
      break;
      
    case TCODE_NAME:
      rc = ON_3dmSettings_Read_v1_TCODE_NAME(file,view.m_name);
      break;

    case TCODE_CPLANE:
      rc = ON_3dmSettings_Read_v1_TCODE_CPLANE(file,view.m_cplane);
      break;

    case TCODE_VIEW:
      rc = ON_3dmSettings_Read_v1_TCODE_VIEW( file, view );
      break;

    case TCODE_SHOWGRID:
      view.m_bShowConstructionGrid = big_value?true:false;
      break;
						
    case TCODE_SHOWGRIDAXES:
      view.m_bShowConstructionAxes = big_value?true:false;
      break;
						
    case TCODE_SHOWWORLDAXES:
      view.m_bShowWorldAxes = big_value?true:false;
      break; 			
      
    case TCODE_VIEWPORT_POSITION:
      rc = file.ReadDouble(&view.m_position.m_wnd_left);
      rc = file.ReadDouble(&view.m_position.m_wnd_top);
      rc = file.ReadDouble(&view.m_position.m_wnd_right);
      rc = file.ReadDouble(&view.m_position.m_wnd_bottom);
      break;
						
    case TCODE_VIEWPORT_TRACEINFO:
      {
        ON_3dPoint origin;
        ON_3dVector xaxis, yaxis;
        if (rc) rc = file.ReadPoint( origin );
        if (rc) rc = file.ReadVector( xaxis );
        if (rc) rc = file.ReadVector( yaxis );
        view.m_trace_image.m_plane.CreateFromFrame(origin,xaxis,yaxis);
        if (rc) rc = file.ReadDouble(&view.m_trace_image.m_width);
        if (rc) rc = file.ReadDouble(&view.m_trace_image.m_height);
        if (rc) rc = ON_3dmSettings_Read_v1_TCODE_NAME(file,view.m_trace_image.m_bitmap_filename);
      }
      break;
      
    case TCODE_VIEWPORT_WALLPAPER:
      rc = ON_3dmSettings_Read_v1_TCODE_NAME(file,view.m_wallpaper_image.m_bitmap_filename);
      break;
						
    case TCODE_HIDE_TRACE:
      // TCODE_HIDE_TRACE was used in early 1.0 betas.  
      // It should have add the short bit set and it is no longer used.
      // This case is here so that these old files will read correctly.
      tcode |= TCODE_SHORT; // so goo skip will work
      break;
      
    case TCODE_MAXIMIZED_VIEWPORT:
      if ( big_value )
        view.m_position.m_bMaximized = true;
      break; 

    case TCODE_VIEWPORT_DISPLAY_MODE: // short TCODE with display mode value
      switch ( big_value ) 
      {
      case 0: // wireframe working mode
        view.m_display_mode = ON::wireframe_display;
        break;
      case 1: // shaded working mode
        view.m_display_mode = ON::shaded_display;
        break;
      }
      break;
						
    }
    if ( !file.EndRead3dmChunk() )
      rc = false;
    if ( tcode == TCODE_ENDOFTABLE )
      break;
  }
  return rc;
}

bool ON_3dmSettings::Read_v1( ON_BinaryArchive& file )
{
  bool bGotSomething = false;
  bool rc = false;
  // read settings from old version 1 file
  size_t pos0 = file.CurrentPosition();

  // need to start at the beginning of the file
  ON__UINT32 tcode;
  ON__INT64 big_value;
  rc = file.SeekFromStart(32)?true:false; // skip 32 byte header
  
  int chunk_count = 0; // debugging counter
  for ( chunk_count = 0; rc; chunk_count++ )
  {
    rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
    if ( !rc ) 
      break; // assume we are at the end of the file

    switch(tcode) {
    case TCODE_VIEWPORT:
      bGotSomething = true;
      {
        ON_3dmView view;
        rc = ON_3dmSettings_Read_v1_TCODE_VIEWPORT(file, view);
        if (rc)
          m_views.Append(view);
      }
      break;

    case TCODE_NAMED_CPLANE:
      bGotSomething = true;
      {
        ON_3dmConstructionPlane cplane;
        rc = ON_3dmSettings_Read_v1_TCODE_NAMED_CPLANE(file,cplane);
        if (rc)
          m_named_cplanes.Append(cplane);
      }
      break;

    case TCODE_NAMED_VIEW:
      bGotSomething = true;
      {
        ON_3dmView view;
        rc = ON_3dmSettings_Read_v1_TCODE_NAMED_VIEW(file, view);
        if (rc)
          m_named_views.Append(view);
      }
      break;
    
    case TCODE_UNIT_AND_TOLERANCES:
      bGotSomething = true;
      rc = ON_3dmSettings_Read_v1_TCODE_UNIT_AND_TOLERANCES(file,m_ModelUnitsAndTolerances);
      break;
    }

    rc = file.EndRead3dmChunk();
  }

  file.SeekFromStart(pos0);
  return bGotSomething;
}

bool ON_3dmSettings::Read_v2(ON_BinaryArchive& file )
{
  bool rc = true;
  ON__UINT32 tcode;
  ON__INT64 big_value;

  while(rc) 
  {
    tcode = 0;
    big_value = 0;
    rc = file.BeginRead3dmBigChunk( &tcode, &big_value );
    if ( !rc )
      break;

    switch(tcode) 
    {
    case TCODE_SETTINGS_PLUGINLIST: 
      {
        int major_version = 0, minor_version = 0, count = 0, i;
        rc = file.Read3dmChunkVersion(&major_version,&minor_version);
        if (rc && 1 == major_version && minor_version >= 0 )
        {
          rc = file.ReadInt( &count );
          if ( count > 0 )
          {
            for ( i = 0; rc && i < count; i++ )
            {
              rc = m_plugin_list.AppendNew().Read(file);
            }
          }
        }
      }
      break;
      
    case TCODE_SETTINGS_UNITSANDTOLS: // units and tolerances
      rc = m_ModelUnitsAndTolerances.Read(file);
      // Copy model settings to page settings so reading old files
      // will work right.  If the file is new enough to have page
      // units and tolerances in it, they get read later.
      m_PageUnitsAndTolerances = m_ModelUnitsAndTolerances;
      break;
      
    case TCODE_SETTINGS_RENDERMESH:
      rc = m_RenderMeshSettings.Read(file);
      break;
      
    case TCODE_SETTINGS_ANALYSISMESH:
      rc = m_AnalysisMeshSettings.Read(file);
      break;
      
    case TCODE_SETTINGS_ANNOTATION:
      rc = m_AnnotationSettings.Read(file);
      break;
      
    case TCODE_SETTINGS_NAMED_CPLANE_LIST: // named cplanes
      {
        m_named_cplanes.Empty();
        ON__UINT32 subtcode = 0;
        ON__INT64 subvalue = 0;
        int count, i;
        rc = file.ReadInt(&count);
        for ( i = 0; i < count && rc ; i++ ) {
          rc = file.BeginRead3dmBigChunk( &subtcode, &subvalue );
          if (rc ) {
            if ( subtcode != TCODE_VIEW_CPLANE )
              rc = false;
            else {
              ON_3dmConstructionPlane& cplane = m_named_cplanes.AppendNew();
              rc = cplane.Read(file);
            }
            if ( !file.EndRead3dmChunk() ) {
              rc = false;
            }
          }
        }
      }
      break;
      
    case TCODE_SETTINGS_NAMED_VIEW_LIST: // named views
      {
        m_named_views.Empty();
        ON__UINT32 subtcode = 0;
        ON__INT64 subvalue = 0;
        int count, i;
        rc = file.ReadInt(&count);
        for ( i = 0; i < count && rc ; i++ ) 
        {
          rc = file.BeginRead3dmBigChunk( &subtcode, &subvalue );
          if (rc ) 
          {
            if ( subtcode != TCODE_VIEW_RECORD )
              rc = false;
            else 
            {
              ON_3dmView& cplane = m_named_views.AppendNew();
              rc = cplane.Read(file);
            }
            if ( !file.EndRead3dmChunk() )
            {
              rc = false;
            }
          }
        }
      }
      break;
      
    case TCODE_SETTINGS_VIEW_LIST: // active view is first in list
      {
        m_views.Empty();
        ON__UINT32 subtcode = 0;
        ON__INT64 subvalue = 0;
        int count, i;
        rc = file.ReadInt(&count);
        m_views.Reserve(count);
        for ( i = 0; i < count && rc ; i++ ) 
        {
          rc = file.BeginRead3dmBigChunk( &subtcode, &subvalue );
          if (rc ) 
          {
            if ( subtcode != TCODE_VIEW_RECORD )
              rc = false;
            else 
            {
              ON_3dmView& view = m_views.AppendNew();
              rc = view.Read(file);
            }
            if ( !file.EndRead3dmChunk() )
            {
              rc = false;
            }
          }
        }
      }
      break;
      
    case TCODE_SETTINGS__NEVER__USE__THIS:
      {
        if ( 28 == big_value )
        {
          // 23 March 2005 Dale Lear - this was the ON_LineStyle
          //                           and a linesytlesource int
          //                           that never got used.
          unsigned char b[24];
          if (rc) rc = file.ReadByte(24,b);
          // The other 4 bytes are a 32 bit chunk crc
          // that gets read by EndRead3dmChunk()
        }
      }
      break;

    case TCODE_SETTINGS_CURRENT_LAYER_INDEX:
      if ( big_value < -1 || big_value > 0x7FFFFFFF )
      {
        ON_ERROR("ON_3dmSettings::Read_v2() - TCODE_SETTINGS_CURRENT_LAYER_INDEX - invalid layer index value");
      }
      else
      {
        m_current_layer_index = (int)big_value;
      }
      break;
      
    case TCODE_SETTINGS_CURRENT_FONT_INDEX:
      if ( big_value < -1 || big_value > 0x7FFFFFFF )
      {
        ON_ERROR("ON_3dmSettings::Read_v2() - TCODE_SETTINGS_CURRENT_FONT_INDEX - invalid font index value");
      }
      else
      {
        // in archives with opennurbs version >= 200106100
        m_current_font_index = (int)big_value;
      }
      break;
      
    case TCODE_SETTINGS_CURRENT_DIMSTYLE_INDEX:
      if ( big_value < -1 || big_value > 0x7FFFFFFF )
      {
        ON_ERROR("ON_3dmSettings::Read_v2() - TCODE_SETTINGS_CURRENT_DIMSTYLE_INDEX - invalid dimstyle index value");
      }
      else
      {
        // in archives with opennurbs version >= 200106100
        m_current_dimstyle_index = (int)big_value;
      }
      break;
      
    case TCODE_SETTINGS_CURRENT_MATERIAL_INDEX:
      {
        int i32 = 0;
        if (rc) rc = file.ReadInt( &m_current_material_index );
        if (rc) rc = file.ReadInt( &i32 );
        if (rc) m_current_material_source = ON::ObjectMaterialSource(i32);
      }
      break;
      
    case TCODE_SETTINGS_CURRENT_COLOR:
      {
        int i32 = 0;
        if (rc) rc = file.ReadColor( m_current_color );
        if (rc) rc = file.ReadInt( &i32 );
        if (rc) m_current_color_source = ON::ObjectColorSource(i32);
      }
      break;
      
    case TCODE_SETTINGS_CURRENT_WIRE_DENSITY:
      if ( big_value < -2 || big_value > 0x7FFFFFFF )
      {
        ON_ERROR("ON_3dmSettings::Read_v2() - TCODE_SETTINGS_CURRENT_WIRE_DENSITY - invalid current_wire_density value");
      }
      else
      {
        m_current_wire_density = (int)big_value;
      }
      break;
      
    case TCODE_SETTINGS_RENDER:
      rc = m_RenderSettings.Read(file);
      break;

    case TCODE_SETTINGS_GRID_DEFAULTS:
      rc = m_GridDefaults.Read(file);
      break;

    case TCODE_SETTINGS_MODEL_URL: // added 21 June 2001
      rc = file.ReadString(m_model_URL);
      break;

    case TCODE_SETTINGS_ATTRIBUTES:
      {
        int major_version = 0;
        int minor_version = 0;
        for(;;)
        {
          rc = file.Read3dmChunkVersion(&major_version,&minor_version);
          if (!rc) break;
          if ( 1 == major_version )
          {
            // version 1.0 fields 23 March 2005
            rc = file.ReadDouble( &m_linetype_display_scale );
            if (!rc) break;

            rc = file.ReadColor(m_current_plot_color);
            if (!rc) break;
  
            int i;
            rc = file.ReadInt(&i);
            if (!rc) break;
            m_current_plot_color_source = ON::PlotColorSource(i);

            rc = file.ReadInt(&m_current_linetype_index);
            if (!rc) break;

            rc = file.ReadInt(&i);
            if (!rc) break;
            m_current_linetype_source = ON::ObjectLinetypeSource(i);

            if ( minor_version >= 1 )
            {
              // Added 6 Feb 2006
              int mjv = 1, mnv = 1;
              rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&mjv,&mnv);
              if (rc)
              {
                rc = m_PageUnitsAndTolerances.Read(file);
                if ( !file.EndRead3dmChunk() )
                  rc = false;
              }


              if ( minor_version >= 2 )
              {
                // 1 Mar 2006 1.2 fields
                rc = file.ReadUuid(m_active_view_id);
                if (!rc) break;

                if ( minor_version >= 3 )
                {
                  rc = file.ReadPoint( m_model_basepoint);
                  if (!rc) break;
                  rc = m_earth_anchor_point.Read(file);
                  if (!rc) break;

                  if ( minor_version >= 4 )
                  {
                    rc = file.ReadBool(&m_IO_settings.m_bSaveTextureBitmapsInFile);
                    if (rc && minor_version >= 5)
                    {
                      rc = m_IO_settings.Read(file);
                      if (rc && minor_version >= 6 )
                      {
                        // 7 June 2006
                        m_CustomRenderMeshSettings.Read(file);
                      }
                    }
                  }
                }
              }
            }

          }

          break;
        }
      }
      break;

    default:
      // information added in future will be skipped by file.EndRead3dmChunk()
      break;
    }

    if ( !file.EndRead3dmChunk() )
      rc = false;
    if ( TCODE_ENDOFTABLE == tcode )
      break;
  }

  return rc;
}

bool ON_3dmSettings::Read(ON_BinaryArchive& file )
{
  bool rc = false;

  Default();

  if ( 1 == file.Archive3dmVersion() ) 
  {
    rc = Read_v1(file); 
  }
  else
  {
    rc = Read_v2(file); 
  }

  return rc;
}


static bool ON_3dmSettings_Write_v1_TCODE_UNIT_AND_TOLERANCES(ON_BinaryArchive& file, const ON_3dmUnitsAndTolerances& UnitsAndTolerances )
{
  bool rc = true;
  int v = 1, us = 0;
  if (rc) rc = file.WriteInt( v ); // v = 1
  switch (UnitsAndTolerances.m_unit_system.m_unit_system) 
  {
  case ON::no_unit_system: 
    us=0; // NO_UNIT_SYSTEM
    break;
  case ON::microns: 
    us=1; // MICRONS
    break;
  case ON::millimeters:
    us=2; // MILLIMETERS
    break;
  case ON::centimeters:
    us=3; // CENTIMETERS
    break;
  case ON::meters:
    us=4; // METERS
    break;
  case ON::kilometers:
    us=5; // KILOMETERS
    break;
  case ON::microinches:
    us=6; // MICROINCHES
    break;
  case ON::mils:
    us=7; // MILS
    break;
  case ON::inches:
    us=8; // INCHES
    break;
  case ON::feet:
    us=9; // FEET
    break;
  case ON::miles:
    us=10; // MILES
    break;
  default:
    us=0; // NO_UNIT_SYSTEM
    break;
  }
  if (rc) rc = file.WriteInt( us );
  if (rc) rc = file.WriteDouble( UnitsAndTolerances.m_absolute_tolerance );
  if (rc) rc = file.WriteDouble( UnitsAndTolerances.m_relative_tolerance );
  if (rc) rc = file.WriteDouble( UnitsAndTolerances.m_angle_tolerance );

  return rc;
}

bool ON_3dmSettings::Write_v1(ON_BinaryArchive& file) const
{
  bool rc = true;

  // version 1 units and tolerances chunk
  rc = file.BeginWrite3dmChunk(TCODE_UNIT_AND_TOLERANCES,0);
  if (rc) {
    rc = ON_3dmSettings_Write_v1_TCODE_UNIT_AND_TOLERANCES( file, m_ModelUnitsAndTolerances );
    if (!file.EndWrite3dmChunk())
      rc = false;
  }

  return rc;
}

bool ON_3dmSettings::Write_v2(ON_BinaryArchive& file) const
{
  int i;
  bool rc = true;

  // TCODE_SETTINGS_PLUGINLIST - plugins that may have saved userdata in the file
  if (rc && file.Archive3dmVersion() >= 4 && m_plugin_list.Count() > 0 )
  {
    // The plug-in list chunk needs to be first, so the plug-ins that save
    // userdata on views can be loaded as needed.
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_PLUGINLIST,0);
    if ( rc ) 
    {
      if (rc) rc = file.Write3dmChunkVersion(1,0);
      if (rc) rc = file.WriteInt( m_plugin_list.Count() );
      for ( i = 0; rc && i < m_plugin_list.Count(); i++ )
      {
        rc = m_plugin_list[i].Write(file);
      }

      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_PROPERTIES_UNITSANDTOLS - units and tolerances
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_UNITSANDTOLS,0);
    if ( rc ) {
      rc = m_ModelUnitsAndTolerances.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_RENDERMESH - rendering defaults
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_RENDERMESH,0);
    if ( rc ) {
      rc = m_RenderMeshSettings.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_ANALYSISMESH - analysis mesh defaults
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_ANALYSISMESH,0);
    if ( rc ) {
      rc = m_AnalysisMeshSettings.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_ANNOTATION - annotation settings
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_ANNOTATION,0);
    if ( rc ) {
      rc = m_AnnotationSettings.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_NAMED_CPLANE_LIST
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_NAMED_CPLANE_LIST,0);
    if ( rc ) {
      const int count = m_named_cplanes.Count();
      rc = file.WriteInt(count);
      for ( i = 0; i < count && rc; i++ ) {
        rc = file.BeginWrite3dmChunk( TCODE_VIEW_CPLANE, 0 );
        if (rc ) {
          rc = m_named_cplanes[i].Write(file);
          if ( !file.EndWrite3dmChunk() )
            rc = false;
        }
      }
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_NAMED_VIEW_LIST
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_NAMED_VIEW_LIST,0);
    if ( rc ) {
      const int count = m_named_views.Count();
      rc = file.WriteInt(count);
      for ( i = 0; i < count && rc; i++ ) {
        rc = file.BeginWrite3dmChunk( TCODE_VIEW_RECORD, 0 );
        if (rc ) {
          rc = m_named_views[i].Write(file);
          if ( !file.EndWrite3dmChunk() )
            rc = false;
        }
      }
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_VIEW_LIST
  if ( rc  ) {
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_VIEW_LIST,0);
    if ( rc ) {
      const int count = m_views.Count();
      rc = file.WriteInt(count);
      for ( i = 0; i < count && rc; i++ ) {
        rc = file.BeginWrite3dmChunk( TCODE_VIEW_RECORD, 0 );
        if (rc ) {
          rc = m_views[i].Write(file);
          if ( !file.EndWrite3dmChunk() )
            rc = false;
        }
      }
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_CURRENT_LAYER_INDEX
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_CURRENT_LAYER_INDEX, m_current_layer_index );
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }

  // TCODE_SETTINGS_CURRENT_MATERIAL_INDEX
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_CURRENT_MATERIAL_INDEX, 0 );
    if (rc) {
      rc = file.WriteInt( m_current_material_index );
      i = m_current_material_source;
      if (rc) rc = file.WriteInt( i );
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_CURRENT_COLOR
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_CURRENT_COLOR, 0 );
    if (rc) {
      rc = file.WriteColor( m_current_color );
      i = m_current_color_source;
      if (rc) rc = file.WriteInt( i );
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }


  // TCODE_SETTINGS_CURRENT_WIRE_DENSITY
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_CURRENT_WIRE_DENSITY, m_current_wire_density );
    if (rc) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_RENDER
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_RENDER, 0 );
    if (rc) {
      rc = m_RenderSettings.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_GRID_DEFAULTS
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_GRID_DEFAULTS, 0 );
    if (rc) {
      rc = m_GridDefaults.Write(file);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_MODEL_URL - added 21 June 2001
  if (rc && m_model_URL.Length() > 0 ) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_MODEL_URL, 0 );
    if (rc) {
      rc = file.WriteString(m_model_URL);
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // TCODE_SETTINGS_CURRENT_FONT_INDEX - added 10 June 2002
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_CURRENT_FONT_INDEX, m_current_font_index );
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }
    
  // TCODE_SETTINGS_CURRENT_DIMSTYLE_INDEX - added 10 June 2002
  if (rc) {
    rc = file.BeginWrite3dmChunk( TCODE_SETTINGS_CURRENT_DIMSTYLE_INDEX, m_current_dimstyle_index );
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }

  // TCODE_SETTINGS_ATTRIBUTES
  if (rc && file.Archive3dmVersion() >= 4 )
  {
    // 23 March 2005 Dale Lear
    rc = file.BeginWrite3dmChunk(TCODE_SETTINGS_ATTRIBUTES, 0 );
    if (rc)
    {
      for(;;)
      {
        // 1.0 - 23 March 2005
        // 1.1 -  6 Feb   2006
        // 1.2 -  1 March 2006
        // 1.3 - 29 March 2006
        // 1.4 - 18 April 2006
        // 1.5 - 21 April 2006
        // 1.6 -  7 June  2006
        rc = file.Write3dmChunkVersion(1,6);

        // version 1.0 fields 
        rc = file.WriteDouble( m_linetype_display_scale );
        if (!rc) break;

        rc = file.WriteColor(m_current_plot_color);
        if (!rc) break;

        rc = file.WriteInt(m_current_plot_color_source);
        if (!rc) break;

        rc = file.WriteInt(m_current_linetype_index);
        if (!rc) break;

        rc = file.WriteInt(m_current_linetype_source);
        if (!rc) break;

        // 6 Feb 2006, 1.1 fields

        // the units and tols has to be inside a chunk
        rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
        if (rc)
        {
          rc = m_PageUnitsAndTolerances.Write(file);
          if ( !file.EndWrite3dmChunk() )
            rc = false;
        }
        if (!rc) break;

        // 1 Mar 2006 1.2 fields
        rc = file.WriteUuid(m_active_view_id);
        if (!rc) break;

        // 29 March 2006 1.3 fields
        rc = file.WritePoint( m_model_basepoint);
        if (!rc) break;
        rc = m_earth_anchor_point.Write(file);
        if (!rc) break;

        // 18 April 2006 1.4 fields
        rc = file.WriteBool(m_IO_settings.m_bSaveTextureBitmapsInFile);
        if (!rc) break;

        // 21 April 2006 1.5 fields
        rc = m_IO_settings.Write(file);
        if (!rc) break;

        // 7 June 2006 1.6 fields
        rc = m_CustomRenderMeshSettings.Write(file);
        if (!rc) break;

        break;
      }

      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }

  // required TCODE_ENDOFTABLE chunk - marks end of settings table
  if ( rc ) {
    rc = file.BeginWrite3dmChunk( TCODE_ENDOFTABLE, 0 );
    if ( rc ) {
      if ( !file.EndWrite3dmChunk() )
        rc = false;
    }
  }


  return rc;
}

bool ON_3dmSettings::Write(ON_BinaryArchive& file) const
{
  bool rc = false;
  if ( 1 == file.Archive3dmVersion() ) 
  {
    rc = Write_v1(file); 
  }
  else
  {
    rc = Write_v2(file); 
  }
  return rc;
}

void ON_3dmSettings::Dump( ON_TextLog& dump ) const
{
  int i;

  const wchar_t* model_URL = m_model_URL;
  if ( model_URL && *model_URL ) 
  {
    dump.Print("Model URL: %ls\n",model_URL);
  }
  dump.Print("Model space units and tolerances:\n");
  dump.PushIndent();
  m_ModelUnitsAndTolerances.Dump(dump);
  dump.PopIndent();

  dump.Print("Page space units and tolerances:\n");
  dump.PushIndent();
  m_PageUnitsAndTolerances.Dump(dump);
  dump.PopIndent();

  dump.Print("Render mesh settings:\n");
  dump.PushIndent();
  m_RenderMeshSettings.Dump(dump);
  dump.PopIndent();

  dump.Print("Analysis mesh settings:\n");
  dump.PushIndent();
  m_AnalysisMeshSettings.Dump(dump);
  dump.PopIndent();

  dump.Print("Render settings:\n");
  dump.PushIndent();
  m_RenderSettings.Dump(dump);
  dump.PopIndent();

  dump.Print("Annotation settings:\n");
  dump.PushIndent();
  m_AnnotationSettings.Dump(dump);
  dump.PopIndent();

  dump.Print("Construction plane grid defaults:\n");
  dump.PushIndent();
  m_GridDefaults.Dump(dump);
  dump.PopIndent();

  dump.Print("Named construction planes:\n");
  dump.PushIndent();
  for ( i = 0; i < m_named_cplanes.Count(); i++ )
  {
    dump.Print("named construction plane %d:\n");
    dump.PushIndent();
    m_named_cplanes[i].Dump(dump);
    dump.PopIndent();
  }
  dump.PopIndent();

  dump.Print("Named views:\n");
  dump.PushIndent();
  for ( i = 0; i < m_named_views.Count(); i++ )
  {
    dump.Print("named view %d:\n",i);
    dump.PushIndent();
    m_named_views[i].Dump(dump);
    dump.PopIndent();
  }
  dump.PopIndent();

  dump.Print("Model views:\n");
  dump.PushIndent();
  for ( i = 0; i < m_views.Count(); i++ )
  {
    dump.Print("model view %d:\n",i);
    dump.PushIndent();
    m_views[i].Dump(dump);
    dump.PopIndent();
  }
  dump.PopIndent();

  dump.Print("New object attributes:\n");
  dump.PushIndent();
  {
    dump.Print("Current display color rgb");dump.PrintRGB(m_current_color); dump.Print(":\n");
    dump.Print("Current display color source = %d\n",m_current_color_source);
    dump.Print("Current plot color rgb");dump.PrintRGB(m_current_plot_color); dump.Print(":\n");
    dump.Print("Current plot color source = %d\n",m_current_plot_color_source);
    dump.Print("Current material index = %d\n",m_current_material_index);
    dump.Print("Current material source = %d\n",m_current_material_source);
    dump.Print("Current linetype index = %d\n",m_current_linetype_index);
    dump.Print("Current linetype source = %d\n",m_current_linetype_source);
    dump.Print("Current layer index = %d\n",m_current_layer_index);
    dump.Print("Current font index = %d\n",m_current_font_index);
    dump.Print("Current dimstyle index = %d\n",m_current_dimstyle_index);
    dump.Print("Current wire density = %d\n",m_current_wire_density);
    dump.Print("Linetype diaplay scale = %g\n",m_linetype_display_scale);
  }
  dump.PopIndent();

  dump.Print("Plug-in list\n");
  dump.PushIndent();
  for ( i = 0; i < m_plugin_list.Count(); i++ )
  {
    dump.Print("plug-in %d:\n",i);
    dump.PushIndent();
    m_plugin_list[i].Dump(dump);
    dump.PopIndent();
  }
  dump.PopIndent();

}


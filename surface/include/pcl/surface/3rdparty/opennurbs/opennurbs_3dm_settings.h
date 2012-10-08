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

#if !defined(OPENNURBS_3DM_SETTINGS_INC_)
#define OPENNURBS_3DM_SETTINGS_INC_


///////////////////////////////////////////////////////////////////////
//
// units and tolerances
//

class ON_CLASS ON_3dmUnitsAndTolerances
{
public:
  ON_3dmUnitsAndTolerances();
  ON_3dmUnitsAndTolerances(const ON_3dmUnitsAndTolerances&);
  ~ON_3dmUnitsAndTolerances();
  ON_3dmUnitsAndTolerances& operator=(const ON_3dmUnitsAndTolerances&);

  void Default();

  bool Read( ON_BinaryArchive& );
  bool Write( ON_BinaryArchive& ) const;

  void Dump( ON_TextLog& ) const;

  //////////
  // Returns scale factor that needs to be applied to change from
  // the argument's unit system to m_unit_system.  
  // When m_unit_system is not ON::custom_unit_system,
  // Scale(us) = ON::UnitScale(us,m_unit_system).  When Scale(us)
  // When m_unit_system is ON::custom_unit_system,
  // Scale(us) = ON::UnitScale(us,ON::meters)*m_custom_unit_scale.
  double Scale( ON::unit_system ) const;

  //ON::unit_system m_unit_system;
  ON_UnitSystem   m_unit_system;

  double          m_absolute_tolerance;  // in units (default = 1/100)
  double          m_angle_tolerance;     // in radians (default = 3 degrees)
  double          m_relative_tolerance;  // fraction >= 0 and < 1 (default = 1%)

  ON::distance_display_mode m_distance_display_mode;
  int m_distance_display_precision; // decimal mode: number of decimal places
                                    // fractional modes:
                                    //    denominator = (1/2)^m_distance_display_precision

  ////////////
  // These settings apply when m_unit_system is ON::custom_unit_system
  //
  //double m_custom_unit_scale;      // 1 meter = m_custom_unit_scale custom units
  //ON_wString m_custom_unit_name;   // name of custom units
};

///////////////////////////////////////////////////////////////////////
//
// Model settings
//   render mesh defaults
//   viewports
//   construction planes
//

class ON_CLASS ON_3dmAnnotationSettings
{
public:
  ON_3dmAnnotationSettings();
  ~ON_3dmAnnotationSettings();
  ON_3dmAnnotationSettings(const ON_3dmAnnotationSettings&);
  ON_3dmAnnotationSettings& operator=(const ON_3dmAnnotationSettings&);

  void Default();

  bool Read( ON_BinaryArchive& );
  bool Write( ON_BinaryArchive& ) const;

  void Dump( ON_TextLog& text_log ) const;

  // these are the running defaults for making dimensions
  // they are also the things written to the 3dm file as dimension settings
  double m_dimscale;       // model size / plotted size
  double m_textheight;
  double m_dimexe;
  double m_dimexo;
  double m_arrowlength;
  double m_arrowwidth;
  double m_centermark;

  /*
  Returns:
    Value of m_world_view_text_scale;
  */
  double WorldViewTextScale() const;

  /*
  Parameters:
    world_view_text_scale - [in]
      Sets value of m_world_view_text_scale.
  */
  void SetWorldViewTextScale(double world_view_text_scale );

  /*
  Returns:
    Value of m_world_view_hatch_scale;
  */
  double WorldViewHatchScale() const;

  /*
  Parameters:
    world_view_hatch_scale - [in]
      Sets value of m_world_view_hatch_scale.
  */
  void SetWorldViewHatchScale(double world_view_hatch_scale );


  /*
  Returns:
    Value of m_bEnableAnnotationScaling;
  */
  bool IsAnnotationScalingEnabled() const;

  /*
  Parameters:
    bEnable - [in]
      Sets value of m_bEnableAnnotationScaling.
  */
  void EnableAnnotationScaling( bool bEnable );

  /*
  Returns:
    Value of m_bEnableHatchScaling;
  */
  bool IsHatchScalingEnabled() const;

  /*
  Parameters:
    bEnable - [in]
      Sets value of m_bEnableHatchScaling.
  */
  void EnableHatchScaling( bool bEnable );

  // Present but not used in V4 or V5 - removed 5 August 2010 to make room
  // for m_world_view_text_scale and m_bEnableAnnotationScaling
  //// added 12/28/05 LW
  //double m_dimdle;
  //double m_dimgap;
private:
  // If m_bEnableAnnotationScaling is true,
  // and ON_Annotation2::m_annotative_scale is true,
  // and ON_Annotation2::m_type == ON::dtTextBlock,
  // and the text object is being displayed in a world
  // view (not a detail view and not a page view),
  // then the text will be scaled by m_world_view_text_scale.
  // The default is 1.0. Values <= 0.0 are not valid.
  float m_world_view_text_scale;
  float m_world_view_hatch_scale;
  
private:
  // If m_bEnableAnnotationScaling is false:
  //   * m_world_view_text_scale is ignored.
  //   * text is not scaled.
  //   * ON_DimStyle::DimScale() determines the scale 
  //     applied to all other annotation objects in all 
  //     types of views.
  //   * The value of ON_DetailView::m_page_per_model_ratio
  //     is applied to all objects (annotation and geometry)
  //     in the detail view.
  //
  // If m_bEnableAnnotationScaling is true:
  //   * m_world_view_text_scale is used as described above.
  //   * ON_DimStyle::DimScale() determines the scale 
  //     applied to all non text annotation objects in 
  //     world views. 
  //   * ON_DimStyle::DimScale() is ignored in page and 
  //     detail views. 
  //   * ON_DetailView::m_page_per_model_ratio is ingored
  //     for annotation objects in detail views, other
  //     geometry is scaled.
  //
  // Default is true.
  unsigned char m_bEnableAnnotationScaling;

  unsigned char m_bEnableHatchScaling;

private:
  unsigned char m_reserved[6];

public:

  ON::unit_system m_dimunits;  // units used to measure the dimension
  int m_arrowtype;     // 0: filled narrow triangular arrow
  int m_angularunits;  // 0: degrees, 1: radians
  int m_lengthformat;  // 0: decimal, ...
  int m_angleformat;   // 0: decimal degrees, ...
  int m_textalign;     // 0: above line, 1: in line, 2: horizontal
  int m_resolution;    // depends on m_lengthformat
                       // for decimal, digits past the decimal point

  ON_wString m_facename; // [LF_FACESIZE] // windows font name
};

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmConstructionPlaneGridDefaults
//
// Default settings used for construction plane grids
class ON_CLASS ON_3dmConstructionPlaneGridDefaults
{
public:
  ON_3dmConstructionPlaneGridDefaults();
  ~ON_3dmConstructionPlaneGridDefaults();
  ON_3dmConstructionPlaneGridDefaults(const ON_3dmConstructionPlaneGridDefaults&);
  ON_3dmConstructionPlaneGridDefaults& operator=(const ON_3dmConstructionPlaneGridDefaults&);

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  void Dump( ON_TextLog& text_log ) const;

	double m_grid_spacing;   // distance between grid lines
  double m_snap_spacing;   // when "grid snap" is enabled, the
                           // distance between snap points.  Typically
                           // this is the same distance as grid spacing.
	int m_grid_line_count;   // number of grid lines in each direction
  int m_grid_thick_frequency; // thick line frequency
                            // 0: none, 
                            // 1: all lines are thick, 
                            // 2: every other is thick, ...

  ON_BOOL32 m_bShowGrid;
  ON_BOOL32 m_bShowGridAxes;
  ON_BOOL32 m_bShowWorldAxes;
};

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmConstructionPlane
//
class ON_CLASS ON_3dmConstructionPlane
{
public:
  ON_3dmConstructionPlane();
  ~ON_3dmConstructionPlane();

  // default copy constructor and operator= work fine
  //ON_3dmConstructionPlane(const ON_3dmConstructionPlane&);
  //ON_3dmConstructionPlane& operator=(const ON_3dmConstructionPlane&);

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  void Dump( ON_TextLog& text_log ) const;

  ON_Plane    m_plane;

  // construction grid appearance
	double m_grid_spacing;   // distance between grid lines
  double m_snap_spacing;   // when "grid snap" is enabled, the
                           // distance between snap points.  Typically
                           // this is the same distance as grid spacing.
	int m_grid_line_count;   // number of grid lines in each direction
  int m_grid_thick_frequency; // thick line frequency
                            // 0: none, 
                            // 1: all lines are thick, 
                            // 2: every other is thick, ...
  bool m_bDepthBuffer; // false=grid is always drawn behind 3d geometry
                       // true=grid is drawn at its depth as a 3d plane
                       // and grid lines obscure things behind the grid.

  ON_wString  m_name;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_3dmConstructionPlane>;
#pragma warning( pop )
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmViewPosition
//
class ON_CLASS ON_3dmViewPosition
{
public:
  // view window relative position and state in parent frame
  ON_3dmViewPosition();
  ~ON_3dmViewPosition();
  ON_3dmViewPosition(const ON_3dmViewPosition&);
  ON_3dmViewPosition& operator=(const ON_3dmViewPosition&);

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  // relative position of view window in main frame
  // if m_floating_viewport>0, this is relative position of the view window
  // on the virtual screen (union of potentially multiple monitors)
  double m_wnd_left;    // 0.0 to 1.0
  double m_wnd_right;
  double m_wnd_top;
  double m_wnd_bottom;
  ON_BOOL32 m_bMaximized;    // true if view window is maximized

  // m_floating_viewport is used to track floating viewport information.
  //  0 = the view is docked in the main application window.
  // >0 = the view is floating. When floating, this corresponds to the
  //      number of monitors on on the user's computer when the file was saved
  unsigned char m_floating_viewport;
private:
  // reserved for future use
  unsigned char m_reserved_1;
  unsigned char m_reserved_2;
  unsigned char m_reserved_3;
};

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmViewTraceImage
//
class ON_CLASS ON_3dmViewTraceImage
{
public:
  ON_3dmViewTraceImage();
  ~ON_3dmViewTraceImage();
  bool operator==( const ON_3dmViewTraceImage& ) const;
  bool operator!=( const ON_3dmViewTraceImage& ) const;

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  // view window relative position and state in parent frame
  ON_Plane m_plane;
  double   m_width;
  double   m_height;

  ON_wString m_bitmap_filename;
  bool m_bGrayScale; // true if image should be black and white
  bool m_bHidden;    // true if image is currently hidden from view
  bool m_bFiltered;  // true if image should be filtered (bilinear) before displayed.
};


//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmViewTraceImage
//
class ON_CLASS ON_3dmWallpaperImage
{
public:
  ON_3dmWallpaperImage();
  ~ON_3dmWallpaperImage();
  bool operator==( const ON_3dmWallpaperImage& ) const;
  bool operator!=( const ON_3dmWallpaperImage& ) const;

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  ON_wString m_bitmap_filename;
  bool m_bGrayScale; // true if image should be black and white
  bool m_bHidden;    // true if image is currently hidden from view
};

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmView
//

class ON_CLASS ON_3dmPageSettings
{
public:
  ON_3dmPageSettings();
  ~ON_3dmPageSettings();

  bool IsValid( ON_TextLog* text_log = 0 ) const;

  void Default();

  int m_page_number;

  // Overall size of the page in millimeters
  double m_width_mm;
  double m_height_mm;

  // Page margins in millimeters
  double m_left_margin_mm;
  double m_right_margin_mm;
  double m_top_margin_mm;
  double m_bottom_margin_mm;

  ON_wString m_printer_name;

  bool Write(ON_BinaryArchive& archive) const;
  bool Read(ON_BinaryArchive& archive);
};


class ON_CLASS ON_3dmView
{
public:
  ON_3dmView();
  ~ON_3dmView();

  // The C++ default copy constructor and operator= work fine.
  // Do not provide customized versions.
  // NO // ON_3dmView(const ON_3dmView&);
  // NO // ON_3dmView& operator=(const ON_3dmView&);

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  void Dump( ON_TextLog& text_log ) const;

  bool IsValid( ON_TextLog* text_log = 0 ) const;

  // view projection information
  ON_Viewport m_vp;

  // clipping planes
  // Prior to Dec 14, 2010 m_clipping_planes was not saved with the view. 
  // After Dec 14, 2010 m_clipping_planes is saved.
  ON_SimpleArray<ON_ClippingPlaneInfo> m_clipping_planes;

  // If true, the the camera location, camera direction,
  // and lens angle should not be changed.
  // It is ok to adjust clipping planes.
  bool m_bLockedProjection;

  ///////////////////////////////////////////////////////////////////////
  //
  // target point
  //

  /*
  Returns:
    Target point.  This point is saved on m_vp.m_target_point.
    The default constructor sets the target point to 
    ON_3dPoint::UnsetPoint. You must explicitly set the target
    point if you want to use it.
  Remarks:
    The target point is stored on m_vp.m_target_point.  The
    value ON_3dmView.m_target is obsolete. This function always
    returns the value of m_vp.m_target_point.

  */
  ON_3dPoint TargetPoint() const;

  /*
  Description:
    Sets the target point. 
  Parameters:
    target_point - [in]
      When in double, the point m_vp.FrustumCenterPoint(ON_UNSET_VALUE)
      is a good choice.
  Remarks:
    This point is saved on m_vp.m_target_point. Using this function
    keeps the obsolete ON_3dmView.m_target value equal to
    m_vp.m_target_point.
  */
  bool SetTargetPoint(ON_3dPoint target_point);

  ///////////////////////////////////////////////////////////////////////
  // OBSOLETE                                                          //
  //   Use ON_3dmView::SetTargetPoint() and ON_3dmView::TargetPoint()  //
  //   functions to set and get the target point. The m_target member  //
  //   will be removed in V6. The only reason m_target is still here   //
  //   is to avoid breaking the public SDK.                            //
  /* OBSOLETE */ ON_3dPoint m_target; // OBSOLETE                      //
  //   Hmm, did you notice that m_target is obsolete?  Try using the   //
  //   SetTargetPoint() and TargetPoint() functions instead.           //
  // OBSOLETE                                                          //
  ///////////////////////////////////////////////////////////////////////

  //
  ///////////////////////////////////////////////////////////////////////

  ON_wString  m_name;   // name on window
  
  // If m_display_mode_id is nil, then use m_display_mode
  // to show one of the "standard" (wireframe, shaded, rendered)
  // display modes.  If m_display_mode_id is not nil, then
  // ignore m_display_mode.
  ON_UUID m_display_mode_id;
  ON::display_mode m_display_mode;

  // position of view in parent window 
  // (relative display device coordinates)
  ON_3dmViewPosition m_position;

  ON::view_type m_view_type; // model, page, or nested

  // If m_view_type == ON::page_view_type, then the m_page_settings
  // records the page size.  Otherwise, m_page_settings should
  // be ignored.
  ON_3dmPageSettings m_page_settings;

  // construction plane
  ON_3dmConstructionPlane m_cplane;
  bool m_bShowConstructionGrid;
  bool m_bShowConstructionAxes;

  // world axes icon
  bool m_bShowWorldAxes;

  // tracing image
  ON_3dmViewTraceImage m_trace_image;

  // wallpaper image
  ON_3dmWallpaperImage m_wallpaper_image;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_3dmView>;
#pragma warning( pop )
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmRenderSettings
//

class ON_CLASS ON_3dmRenderSettings
{
public:
  ON_3dmRenderSettings();
  ~ON_3dmRenderSettings();
  ON_3dmRenderSettings(const ON_3dmRenderSettings&);
  ON_3dmRenderSettings& operator=(const ON_3dmRenderSettings&);

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  void Dump( ON_TextLog& text_log ) const;

  bool ScaleBackgroundToFit() const;
  void SetScaleBackgroundToFit( bool bScaleBackgroundToFit );

  //////////
  // false: image pixel size = current viewport size
  // true:  image pixel size = m_image_width X m_image_height pixels
  ON_BOOL32 m_bCustomImageSize;
  int  m_image_width;   // image width in pixels
  int  m_image_height;  // image height in pixels

private:
  bool m_bScaleBackgroundToFit;
  unsigned char m_reserved1[3];
public:

  ////////
  // Number of dots/inch (dots=pixels) to use when printing and 
  // saving bitmaps. The default is 72.0 dots/inch.
  double m_image_dpi; 
  //////////
  // unit system to use when converting image pixel size and dpi
  // information into a print size.  Default = inches
  ON::unit_system m_image_us;

  ON_Color m_ambient_light;
  
  int m_background_style; // 0 = solid color, 1 = "wallpaper" image, 2 = Gradient, 3 = Environment
  ON_Color m_background_color; // also Top color of gradient...
  ON_wString m_background_bitmap_filename;

  ON_BOOL32 m_bUseHiddenLights;

  ON_BOOL32 m_bDepthCue;
  ON_BOOL32 m_bFlatShade;

  ON_BOOL32 m_bRenderBackfaces;
  ON_BOOL32 m_bRenderPoints;
  ON_BOOL32 m_bRenderCurves;
  ON_BOOL32 m_bRenderIsoparams;
  ON_BOOL32 m_bRenderMeshEdges;
  ON_BOOL32 m_bRenderAnnotation;

  int m_antialias_style; // 0 = none, 1 = normal, 2 = best

  int m_shadowmap_style;    // 0 = none, 1 = normal, 2 = best
  int m_shadowmap_width;
  int m_shadowmap_height;
  double m_shadowmap_offset;
  
  ON_Color  m_background_bottom_color;
  
  // Flags that are used to determine which render settings a render
  // plugin uses, and which ones the display pipeline should use.
  // Note: Render plugins set these, and they don't need to persist
  //       in the document...Also, when set, they turn OFF their
  //       corresponding setting in the Display Attributes Manager's
  //       UI pages for "Rendered" mode.
  bool    m_bUsesAmbientAttr;
  bool    m_bUsesBackgroundAttr;
  bool    m_bUsesBackfaceAttr;
  bool    m_bUsesPointsAttr;
  bool    m_bUsesCurvesAttr;
  bool    m_bUsesIsoparmsAttr;
  bool    m_bUsesMeshEdgesAttr;
  bool    m_bUsesAnnotationAttr;
  bool    m_bUsesHiddenLightsAttr;

private:
  unsigned char m_reserved2[3];
};


//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_EarthAnchorPoint
//

class ON_CLASS ON_EarthAnchorPoint
{
public:
  ON_EarthAnchorPoint();
  ~ON_EarthAnchorPoint();

  static
  int Compare(
          const ON_EarthAnchorPoint*, 
          const ON_EarthAnchorPoint*
          );

  static
  int CompareEarthLocation(
          const ON_EarthAnchorPoint*, 
          const ON_EarthAnchorPoint*
          );

  static
  int CompareModelDirection(
          const ON_EarthAnchorPoint*, 
          const ON_EarthAnchorPoint*
          );

  static
  int CompareIdentification(
          const ON_EarthAnchorPoint*, 
          const ON_EarthAnchorPoint*
          );

  void Default();
  bool Read( ON_BinaryArchive& );
  bool Write( ON_BinaryArchive& ) const;

  // Point on the Earth
  //   Latitude (degrees):  +90 = north pole, 0 = equator, -90 = south pole
  //   Longitude (degrees):   0 = prime meridian (Greenwich meridian)
  //   Elevation (meters):    
  double m_earth_basepoint_latitude;  // in decimal degrees
  double m_earth_basepoint_longitude; // in decimal degrees
  double m_earth_basepoint_elevation; // in meters
  int m_earth_basepoint_elevation_zero; // 0 = ground level
                                        // 1 = mean sea level
                                        // 2 = center of earth

  // Corresponding model point in model coordinates.
  ON_3dPoint  m_model_basepoint; // in model coordinates

  // Earth directions in model coordinates
  ON_3dVector m_model_north; // in model coordinates
  ON_3dVector m_model_east;  // in model coordinates

  // Identification information about this location
  ON_UUID    m_id;           // unique id for this anchor point
  ON_wString m_name;
  ON_wString m_description; 
  ON_wString m_url;
  ON_wString m_url_tag;      // UI link text for m_url

  /*
  Parameters:
    model_compass - [out]
      A plane in model coordinates whose xaxis points East,
      yaxis points North and zaxis points up.  The origin
      is set to m_model_basepoint.
  */
  bool GetModelCompass( 
          ON_Plane& model_compass 
          ) const;

  /*
  Description:
    Get a transformation from model coordinates to earth coordinates.
    This transformation assumes the model is small enough that
    the curvature of the earth can be ignored.  
  Parameters:
    model_unit_system - [in]
    model_to_earth - [out]
      Transformation from model coordinates to earth locations
      (degrees latitude,degrees longitude,elevation in meters)
  Remarks:
    If M is a point in model coordinates and E = model_to_earth*M,
    then 
       E.x = latitude in decimal degrees
       E.y = longitude in decimal degrees
       E.z = elevation in meters above mean sea level

    Because the earth is not flat, there is a small amount of error
    when using a linear transformation to calculate oblate spherical 
    coordinates.  This error is small.  If the distance from P to M
    is d meters, then the approximation error is

       latitude error  <=
       longitude error <=
       elevation error <= 6379000*((1 + (d/6356000)^2)-1) meters

    In particular, if every point in the model is within 1000 meters of
    the m_model_basepoint, then the maximum approximation errors are

       latitude error  <=
       longitude error <=
       elevation error <= 8 centimeters
  */
  bool GetModelToEarthXform(
          const ON_UnitSystem& model_unit_system,
          ON_Xform& model_to_earth
          ) const;
};



class ON_CLASS ON_3dmIOSettings
{
public:
  ON_3dmIOSettings();

  void Default();

  bool Read(ON_BinaryArchive&);
  bool Write(ON_BinaryArchive&) const;

  // bitmaps associated with rendering materials
  bool m_bSaveTextureBitmapsInFile;

  // As of 7 February 2012, the m_idef_link_update setting
  // controls if, when and how linked and linked_and_embedded
  // instance defintions are updated when the source archive
  // that was used to create the idef has changed.
  int m_idef_link_update;  
      // 1 = prompt - ask the user if the idef should be updated.
      // 2 = always update - no prompting
      // 3 = never update - no prompting
      // Any value not equal to 1,2 or 3 shall be treated as 1.
};

//////////////////////////////////////////////////////////////////////////////////////////
//
// ON_3dmSettings
//

class ON_CLASS ON_3dmSettings
{
public:
  ON_3dmSettings();
  ~ON_3dmSettings();

  // C++ copy constructor and operator= work fine.
  // Do not provide custom versions.
  // NO // ON_3dmSettings(const ON_3dmSettings&);
  // NO // ON_3dmSettings& operator=(const ON_3dmSettings&);

  void Default();

  bool Read(ON_BinaryArchive&);
  bool Write(ON_BinaryArchive&) const;

  void Dump( ON_TextLog& ) const;

  // model URL (can be empty)
  ON_wString m_model_URL;

  // Model basepoint is used when the file is read as
  // an instance definition and is the point that is
  // mapped to the origin in the instance definition.
  ON_3dPoint m_model_basepoint;


  // If set, this is the model's location on the earth.
  // This information is used when the model is used
  // with GIS information.
  ON_EarthAnchorPoint m_earth_anchor_point;

  // Model space tolerances and unit system
  ON_3dmUnitsAndTolerances m_ModelUnitsAndTolerances;

  // Page space (printing/paper) tolerances and unit system
  ON_3dmUnitsAndTolerances m_PageUnitsAndTolerances;

  // settings used for automatically created rendering meshes
  ON_MeshParameters m_RenderMeshSettings;

  // saved custom settings
  ON_MeshParameters m_CustomRenderMeshSettings;

  // settings used for automatically created analysis meshes
  ON_MeshParameters m_AnalysisMeshSettings;

  // settings used when annotation objects are created
  ON_3dmAnnotationSettings m_AnnotationSettings;

  ON_ClassArray<ON_3dmConstructionPlane> m_named_cplanes;
  ON_ClassArray<ON_3dmView>              m_named_views;
  ON_ClassArray<ON_3dmView>              m_views; // current viewports
  ON_UUID m_active_view_id; // id of "active" viewport              

  // These fields determine what layer, material, color, line style, and
  // wire density are used for new objects.
  int m_current_layer_index;

  int m_current_material_index;
  ON::object_material_source m_current_material_source;
  
  ON_Color m_current_color;
  ON::object_color_source m_current_color_source;

  ON_Color m_current_plot_color;
  ON::plot_color_source m_current_plot_color_source;

  int m_current_linetype_index;
  ON::object_linetype_source m_current_linetype_source;

  int m_current_font_index;

  int m_current_dimstyle_index;
 
  // Surface wireframe density
  //
  //   @untitled table
  //   0       boundary + "knot" wires 
  //   1       boundary + "knot" wires + 1 interior wire if no interior "knots"
  //   N>=2    boundry + "knot" wires + (N-1) interior wires
  int m_current_wire_density;

  ON_3dmRenderSettings m_RenderSettings;

  // default settings for construction plane grids
  ON_3dmConstructionPlaneGridDefaults m_GridDefaults;

  // World scale factor to apply to non-solid linetypes
  // for model display.  For plotting, the linetype settings
  // are used without scaling.
  double m_linetype_display_scale;

  // Plugins that were loaded when the file was saved.
  ON_ClassArray<ON_PlugInRef> m_plugin_list;

  ON_3dmIOSettings m_IO_settings;
private:
  bool Read_v1(ON_BinaryArchive&);
  bool Read_v2(ON_BinaryArchive&);
  bool Write_v1(ON_BinaryArchive&) const;
  bool Write_v2(ON_BinaryArchive&) const;
};

#endif

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

#if !defined(OPENNURBS_LIGHT_INC_)
#define OPENNURBS_LIGHT_INC_

class ON_CLASS ON_Light : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_Light);

public:
  ON_Light();
  ~ON_Light();
  // C++ defaults work fine
  //ON_Light& operator=(const ON_Light&);
  //ON_Light(const ON_Light&);

  /////////////////////////////////////////////////////////////////
  //
  // ON_Object virtual functions 
  //

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const; // for debugging

  // Use ON_BinaryArchive::WriteObject() and ON_BinaryArchive::ReadObject()
  // for top level serialization.  These Read()/Write() members should just
  // write/read specific definitions.  In particular, they should not write/
  // read any chunk typecode or length information.  The default 
  // implementations return false and do nothing.
  ON_BOOL32 Write(
         ON_BinaryArchive&  // serialize definition to binary archive
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // restore definition from binary archive
       );

  ON::object_type ObjectType() const;

  // virtual
  ON_UUID ModelObjectId() const;


  /////////////////////////////////////////////////////////////////
  //
  // ON_Geometry virtual functions 
  //
  int Dimension() const;

  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // boxmin[dim]
         double*,    // boxmax[dim]
         ON_BOOL32 = false  // true means grow box
         ) const;

  ON_BOOL32 Transform( 
         const ON_Xform&
         );
 
  /////////////////////////////////////////////////////////
  //
  // Interface
  //

  void Default(); // make default light

  /////////////////////////////////////////////////////////
  //
  // turn light on/off
  //
  ON_BOOL32 Enable( ON_BOOL32 = true ); // returns previous state
  ON_BOOL32 IsEnabled() const;
  
  /////////////////////////////////////////////////////////
  //
  // style, location, and direction
  //   direction is ignored for "point" and "ambient" lights
  //   location is ignored for "directional" and "ambient" lights
  void SetStyle(ON::light_style);
  ON::light_style Style() const;

  ON_BOOL32 IsPointLight() const;
  ON_BOOL32 IsDirectionalLight() const;
  ON_BOOL32 IsSpotLight() const;
  ON_BOOL32 IsLinearLight() const;
  ON_BOOL32 IsRectangularLight() const;

  ON::coordinate_system CoordinateSystem() const; // determined by style

  /*
  Description:
    A light's location and direction can be defined with respect
    to world, camera, or view coordinates.  GetLightXform gets
    the transformation from the light's intrinsic coordinate
    system to the destination coordinate system specified
    by dest_cs.
  Parameters:
    vp - [in] viewport where light is being used
    dest_cs - [in] destination coordinate system
    xform - [out] transformation from the light's intrinsic
                  coordinate system to cs.
  Returns:
    true if successful.
  */
  ON_BOOL32 GetLightXform( 
           const ON_Viewport& vp,
           ON::coordinate_system dest_cs, 
           ON_Xform& xform 
           ) const;

  void SetLocation( const ON_3dPoint& );
  void SetDirection( const ON_3dVector& );

  ON_3dPoint Location() const;
  ON_3dVector Direction() const;
  ON_3dVector PerpindicularDirection() const;

  double Intensity() const; // 0.0 = 0%  1.0 = 100%
  void SetIntensity(double);

  double PowerWatts() const;
  double PowerLumens() const;
  double PowerCandela() const;
  
  void SetPowerWatts( double );
  void SetPowerLumens( double );
  void SetPowerCandela( double );

  /////////////////////////////////////////////////////////
  //
  // colors
  //
  void SetAmbient(  ON_Color );
  void SetDiffuse(  ON_Color );
  void SetSpecular( ON_Color );
  ON_Color Ambient() const;
  ON_Color Diffuse() const;
  ON_Color Specular() const;

  /////////////////////////////////////////////////////////
  //
  // attenuation settings (ignored for "directional" and "ambient" lights)
  // attenuation = 1/(a[0] + d*a[1] + d^2*a[2]) where d = distance to light
  //
  void SetAttenuation(double,double,double);
  void SetAttenuation(const ON_3dVector&);
  ON_3dVector Attenuation() const;
  double Attenuation(double) const; // computes 1/(a[0] + d*a[1] + d^2*a[2]) where d = argument
                                    // returns 0 if a[0] + d*a[1] + d^2*a[2] <= 0

  /////////////////////////////////////////////////////////
  //
  // spot light parameters (ignored for non-spot lights)
  //
  // angle = 0 to 90 degrees
  // exponent = 0 to 128 (0=uniform, 128=high focus)
  //
  void SetSpotAngleDegrees( double );
  double SpotAngleDegrees() const;

  void SetSpotAngleRadians( double );
  double SpotAngleRadians() const;

  //////////
  // The spot exponent varies from 0.0 to 128.0 and provides
  // an exponential interface for controling the focus or 
  // concentration of a spotlight (like the 
  // OpenGL GL_SPOT_EXPONENT parameter).  The spot exponent
  // and hot spot parameters are linked; changing one will
  // change the other.
  // A hot spot setting of 0.0 corresponds to a spot exponent of 128.
  // A hot spot setting of 1.0 corresponds to a spot exponent of 0.0.
  void SetSpotExponent( double );
  double SpotExponent() const;

  //////////
  // The hot spot setting runs from 0.0 to 1.0 and is used to
  // provides a linear interface for controling the focus or 
  // concentration of a spotlight.
  // A hot spot setting of 0.0 corresponds to a spot exponent of 128.
  // A hot spot setting of 1.0 corresponds to a spot exponent of 0.0.
  void SetHotSpot( double );
  double HotSpot() const;

  // The spotlight radii are useful for display UI.
  bool GetSpotLightRadii( double* inner_radius, double* outer_radius ) const;


  /////////////////////////////////////////////////////////
  //
  // linear and rectangular light parameters
  // (ignored for non-linear/rectangular lights)
  //
  void SetLength( const ON_3dVector& );
  ON_3dVector Length() const;

  void SetWidth( const ON_3dVector& );
  ON_3dVector Width() const;

  /////////////////////////////////////////////////////////
  //
  // shadow parameters (ignored for non-spot lights)
  //
  // shadow intensity 0.0 = does not cast any shadows
  //                  1.0 = casts black shadows
  //
  void SetShadowIntensity(double);
  double ShadowIntensity() const;
                                 

  /////////////////////////////////////////////////////////
  //
  // light index
  //
  void SetLightIndex( int );
  int LightIndex() const;

  /////////////////////////////////////////////////////////
  //
  // light name
  //
  void SetLightName( const char* );
  void SetLightName( const wchar_t* );
  const ON_wString& LightName() const;

public:
  int           m_light_index;
  ON_UUID       m_light_id;
  ON_wString    m_light_name;

  ON_BOOL32                 m_bOn;   // true if light is on
  ON::light_style      m_style; // style of light

  ON_Color m_ambient;
  ON_Color m_diffuse;
  ON_Color m_specular;
  
  ON_3dVector m_direction; // ignored for "point" and "ambient" lights
  ON_3dPoint  m_location;  // ignored for "directional" and "ambient" lights
  ON_3dVector m_length;    // only for linear and rectangular lights
                           // ends of linear lights are m_location and m_location+m_length
  ON_3dVector m_width;     // only for rectangular lights
                           // corners of rectangular lights are m_location, m_location+m_length,
                           // m_location+m_width, m_location+m_width+m_length

  double      m_intensity; // Linear dimming/brightening factor: 0.0 = off, 1.0 = 100%.
                           // Values < 0.0 and values > 1.0 are permitted but are
                           // not consistently interpreted by various renderers.
                           // Renderers should clamp the range to [0.0, 1.0] if their
                           // lighting model does not support more exotic interpretations
                           // of m_intensity.
  double      m_watts;     // Used by lighting models that reference lighting fixtures.
                           // Values < 0.0 are invalid.  If m_watts is 0.0, the
                           // value is ignored.

  // spot settings - ignored for non-spot lights
  double       m_spot_angle;    // 0.0 to 90.0
  double       m_spot_exponent; // 0.0 to 128.0
                                // 0.0 = uniform
                                // 128.0 = high focus
  double       m_hotspot;       // 0.0 to 1.0 (See SetHotSpot() for details)

  // attenuation settings - ignored for "directional" and "ambient" lights
  ON_3dVector m_attenuation;    // each entry >= 0.0
                                // att = 1/(a[0] + d*a[1] + d^2*a[2])
                                // where d = distance to light

  // shawdow casting
  double       m_shadow_intensity; // 0.0 = no shadow casting, 1.0 = full shadow casting
};



#endif

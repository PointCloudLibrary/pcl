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

////////////////////////////////////////////////////////////////
//
//   defines ON_Color and ON_Material
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_TEXTURE_INC_)
#define OPENNURBS_TEXTURE_INC_

///////////////////////////////////////////////////////////////////////////////
//
// Class ON_Texture
// 

class ON_CLASS ON_Texture : public ON_Object
{
public:
  ON_OBJECT_DECLARE(ON_Texture);

  ON_Texture();
  ~ON_Texture();

  // default copy constructor and operator= work fine


  // overrides virtual ON_Object::IsValid
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // overrides virtual ON_Object::Dump
  void Dump( ON_TextLog& ) const;

  // overrides virtual ON_Object::SizeOf
  unsigned int SizeOf() const;

  // overrides virtual ON_Object::Write
  ON_BOOL32 Write(
         ON_BinaryArchive& binary_archive
       ) const;

  // overrides virtual ON_Object::Read
  ON_BOOL32 Read(
         ON_BinaryArchive& binary_archive
       );

  void Default();

  int Compare( const ON_Texture& other ) const; 

  /*
  Description:
    Reverses the texture in the specified direction.
  Parameters:
    dir - [in] 0 = reverse "u", 1 = reverse "v", 2 = reverse "w".
  Remarks:
    Modifes m_uvw so that the spedified direction transforms
    the texture coordinate t to 1-t.
  Returns:
    True if input is valid.
  */
  bool ReverseTextureCoordinate( int dir );

  /*
  Description:
    Swaps the specified texture coordinates.
  Parameters:
    i - [in]
    j - [in]  (0 <= i, j <= 3 and i != j)
  Remarks:
    Modifes m_uvw so that the specified texture coordinates are swapped.
  Returns:
    True if input is valid.
  */
  bool SwapTextureCoordinate( int i, int j );

  /*
  Description:
    Tiles the specified texture coordinates.
  Parameters:
    dir - [in] 0 = reverse "u", 1 = reverse "v", 2 = reverse "w".
    count - [in] number of tiles (can be negative)
    offset - [in] offset of the tile (can be any number)
  Remarks:
    Modifes m_uvw so that the specified texture coordinate is
    tiled.
  Returns:
    True if input is valid.
  */
  bool TileTextureCoordinate( int dir, double count, double offset );

  /*
  Description:
    Examines the m_uvw matrix and harvests tiling constants.
  Parameters:
    dir - [in] 0 = reverse "u", 1 = reverse "v", 2 = reverse "w".
    count - [out] number of tiles
    offset - [out] offset of the tile
  Returns:
    True if if the m_uvw matrix had entries that were compatible
    with tiling.
  */
  bool IsTiled( int dir, double* count, double* offset ) const;


  ON_UUID m_texture_id;

  // list of pre-defined channel ids
  enum MAPPING_CHANNEL
  {
    tc_channel      = 0,     // Use the texture coordinate values
                             // currently on the geometric object.
    default_channel = 1,	   // Use either default mapping, or the "Custom"
							               // mapping applied to the object
    srfp_channel = 0xFFFFFFFE, // Use surface parameterization.
    emap_channel = 0xFFFFFFFF  // Environment map the geometric object.
  };

  // If the m_mapping_channel_id value is one of the built-in 
  // mappings listed in the MAPPING_CHANNEL enum, then that 
  // mapping is used.  Otherwise, if an object has rendering
  // attributes with an ON_MappingChannel entry that has a 
  // matching m_mapping_channel_id value, then the mapping 
  // identified by ON_MappingChannel::m_mapping_id is used.
  // A value of zero means no mapping is supplied
  // and the texture coordinates on the mesh are
  // used.
  int m_mapping_channel_id;

  // Bitmap filename  
  //   During runtime, m_filename is the absolute path to the
  //   file in use.  If m_filename_bRelativePath is true, then
  //   the value saved in the 3dm archive will be a relative path.
  //   When m_filename_bRelativePath is true, user interface
  //   should display a relative path.
  ON_wString m_filename;
  bool m_filename_bRelativePath;

  // If false, texture is off and should be ignored.
  // The intended use is to allow people to turn textures
  // on and off without have to create/destroy or change 
  // other texture settings.
  bool m_bOn;

  // do not change TYPE enum values - they are saved in 3dm files.
  // The "TYPE" setting controls how the pixels in the bitmap
  // are interpreted.
  enum TYPE
  {
    no_texture_type = 0,

    bitmap_texture       = 1, // "standard" image texture.
    bump_texture         = 2, // bump map - see m_bump_scale comment
    transparency_texture = 3, // value = alpha (see m_tranparancy_id)

    // OBSOLETE - set m_mapping_channel_id = ON_MappingChannel::emap_mapping
    emap_texture = 86, // spherical environment mapping.

    force_32bit_texture_type = 0xFFFFFFFF
  };

  TYPE m_type;

  // m_mode determines how the texture is
  // do not change MODE enum values - they are saved in 3dm files.
  enum MODE
  {
    no_texture_mode  = 0,
    modulate_texture = 1,  // modulate with material diffuse color
    decal_texture    = 2,  // decal
    blend_texture    = 3,  // blend texture with others in the material
                           // To "add" a texture, set m_blend_amount = +1
                           // To "subtract" a texture, set m_blend_amount = -1

    force_32bit_texture_mode = 0xFFFFFFFF
  };

  MODE m_mode;

  enum FILTER
  {
    nearest_filter = 0, // nearest texture pixel is used
    linear_filter  = 1, // weighted average of corresponding texture pixels

    force_32bit_texture_filter = 0xFFFFFFFF
  };
  
  // The value of m_minfilter determines how the color
  // of the image pixel is calculated when the image pixel
  // corresponds to multiple texture bitmap pixels.
  FILTER m_minfilter;  

  // The magfilter setting controls how the color
  // of the image pixel is calculated when the image pixel
  // corresponds to a fraction of a texture bitmap pixel.
  FILTER m_magfilter;

  enum WRAP
  {
    repeat_wrap      = 0,
    clamp_wrap       = 1,

    force_32bit_texture_wrap = 0xFFFFFFFF
  };

  WRAP m_wrapu;
  WRAP m_wrapv;
  WRAP m_wrapw;

  // Texture coordinate transformation.
  bool m_bApply_uvw; // true if m_uvw is active.
  ON_Xform m_uvw;

  // If ON_UNSET_COLOR != m_border_color, then this color
  // is used when the texture coordinates are <=0 or >=1
  // and the m_wrap* value is clamp_wrap.
  ON_Color m_border_color;

  // This field is used for textures with type
  // bitmap_texture that reference bitmap files that do
  // not have an alpha channel and is used to set
  // runtime alpha values.  It needs to be parsed when the
  // texture is loaded and can be ignored at runtime.
  // 
  // If ON_UNSET_COLOR != m_transparent_color, then 
  // a pixel in the bitmap file with a matching RGB
  // value is assigned the alpha value (ON_Color::Alpha)
  // in m_transparent_color. The intended use is 
  // for non-rectangular decals defined by RGB bitmaps in
  // files that don't save an alpha channel.
  //
  // For example if the decal is a red number 7 with a 
  // white background, then you would set m_transparent_color's
  // RGB to white and its A to zero.
  ON_Color m_transparent_color;

  // This field is used for textures with type
  // bitmap_texture that reference bitmap files that do
  // not have an alpha channel and is used to set
  // runtime alpha values.  It needs to be parsed when the
  // texture is loaded and can be ignored at runtime.
  // 
  // If m_transparency_id is not nil, it is the id of another
  // texture in the ON_Material.m_textures[] array that has
  // type m_transparency_texture.  The runtime bitmap_texture's
  // alpha is set to (255-max(R,G,B)) (the "value" in the hue,
  // saturation,value sense) of the correspondeing 
  // transparency_texture pixel. 
  //
  // For example, if you had a bitmap texuture that was green 
  // with purple dots saved in a RGB .bmp file and you wanted
  // the purple dots to be semi-transparent, you could create
  // another bitmap that was black, where the original was green,
  // and gray, where the original was purple, have an 
  // transparency_texture reference the white/gray bitmap,
  // and have the bitmap_texture's m_transparency_id 
  // reference the transparency map.
  ON_UUID m_transparency_texture_id;

  // If the m_type is bump_texture, the height of the
  // bump is m_bump_scale.ParameterAt(value), where
  // value is in the HSV sense and normalized 
  // (black=0, white=1).  The interval can be 
  // decreasing.
  ON_Interval m_bump_scale;

  // If the m_mode is blend_texture, then m_blend_A[]
  // and m_blend_RGB[] determine the blending function.
  //  new alpha  = m_blend_constant_A 
  //             + m_blend_A[0]*(current alpha)
  //             + m_blend_A[1]*(texture alpha)
  //             + m_blend_A[2]*min(current alpha,texture alpha)
  //             + m_blend_A[3]*max(current alpha,texture alpha)
  //  new rgb    = m_blend_constant_RGB 
  //             + m_blend_RGB[0]*(current RGB)
  //             + m_blend_RGB[1]*(texture RGB)
  //             + m_blend_RGB[2]*min(current RGB,texture RGB)
  //             + m_blend_RGB[3]*max(current RGB,texture RGB)
  // Results are clamped to handle underflow or overflow.
  double m_blend_constant_A;
  double m_blend_A[4];
  ON_Color m_blend_constant_RGB;
  double m_blend_RGB[4];

  // If an ON_Material m_textures[] array has more than
  // one texture, the textures are blended, and the textures
  // have different m_blend_order values, the the texture 
  // with the smaller m_blend_order is first. 
  int m_blend_order;

  // Applications use the m_runtime_ptr_id and m_runtime_ptr fields
  // to cached runtime bitmaps. If either the id or the pointer
  // are non-zero, then you cannot use them.  If you hang something
  // on the pointer, then set the id to something unique to
  // prevent others from messing with it.
  ON_UUID m_runtime_ptr_id;
  const void* m_runtime_ptr;

  static TYPE   TypeFromInt( int i );
  static MODE   ModeFromInt( int i );
  static FILTER FilterFromInt( int i );
  static WRAP   WrapFromInt( int i );
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Texture>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_Texture>;
#pragma warning( pop )
#endif

#endif


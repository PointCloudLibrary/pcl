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

#if !defined(OPENNURBS_MATERIAL_INC_)
#define OPENNURBS_MATERIAL_INC_


///////////////////////////////////////////////////////////////////////////////
//
// Class ON_Material
// 
class ON_CLASS ON_Material : public ON_Object
{
  ON_OBJECT_DECLARE(ON_Material);

public:
  static double MaxShine();			// maximum value of shine exponent

  ON_Material();					// Default grey color
  ~ON_Material();					// destructor
  // C++ default copy construction and operator= work fine.

  bool operator==(const ON_Material&) const; // ignores m_material_index
  bool operator!=(const ON_Material&) const; // ignores m_material_index

  void Default();

  /////////////////////////////////////////////////////////////////
  // ON_Object overrides

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

  ON_BOOL32 Write(
         ON_BinaryArchive&  // open binary file
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // open binary file
       );

  ON::object_type ObjectType() const;

  // virtual
  ON_UUID ModelObjectId() const;


  /////////////////////////////////////////////////////////////////
  // Interface

  // ignores m_material_index
  int Compare( const ON_Material& other ) const; 

  // OBSOLETE - use m_ambient
  ON_Color Ambient() const;
  // OBSOLETE - use m_diffuse
  ON_Color Diffuse() const;
  // OBSOLETE - use m_emission
  ON_Color Emission() const;
  // OBSOLETE - use m_specular
  ON_Color Specular() const;

  // OBSOLETE - use m_ambient
  void SetAmbient(  ON_Color );
  // OBSOLETE - use m_diffuse
  void SetDiffuse(  ON_Color );
  // OBSOLETE - use m_emission
  void SetEmission( ON_Color );
  // OBSOLETE - use m_specular
  void SetSpecular( ON_Color );

  // Shine values are in range 0.0 to ON_Material::MaxShine()
  double Shine() const;
  void SetShine( double );         // 0 to ON_Material::MaxShine()

  // Transparency values are in range 0.0 = opaque to 1.0 = transparent
  double Transparency() const;
  void SetTransparency( double );  // 0.0 = opaque, 1.0 = transparent

  // OBSOLETE - use m_material_index
  int MaterialIndex() const;
  // OBSOLETE - use m_material_index
  void SetMaterialIndex( int );

  // OBSOLETE - just use m_plugin_id
  ON_UUID MaterialPlugInUuid() const;

  // OBSOLETE - just use m_plugin_id
  void SetMaterialPlugInUuid( ON_UUID );

  // OBSOLETE - just use m_material_name
  const wchar_t* MaterialName() const;

  // OBSOLETE - just use m_material_name
  void SetMaterialName( const wchar_t* );

  // The only reliable and persistent way to reference 
  // materials is by the material_id.
  ON_UUID m_material_id;

  // Runtime material table index. This value is constant
  // for each runtim instance of Rhino, but can change
  // each time a model is loaded or saved.  Once a material
  // is in the CRhinoDoc material table, its id and index
  // never change in that instance of Rhino.
  int m_material_index;

  // 
  ON_wString m_material_name;  // For user comfort - duplicates permitted
  
  ON_wString m_flamingo_library; // Legacy information from V3.
                                 // Will vanish in V5.

  ON_Color   m_ambient;
  ON_Color   m_diffuse;
  ON_Color   m_emission;
  ON_Color   m_specular;
  ON_Color   m_reflection;
  ON_Color   m_transparent;
  double     m_index_of_refraction; // generally >= 1.0 (speed of light in vacum)/(speed of light in material)
  double     m_reflectivity; // 0.0 = none, 1.0 = 100%
  double     m_shine;        // 0.0 = none to GetMaxShine()=maximum
  double     m_transparency; // 0.0 = opaque to 1.0 = transparent (1.0-alpha)

  bool m_bShared; // default = false.
  // True means this material can be shared.  When an
  // object that uses this material is copied,
  // the new object will share the material.
  // False means this material is not shared.
  // When an object that uses this material is
  // duplicated.

  bool m_bDisableLighting; // default = false.
  // True means render this object without
  // applying any modulation based on lights.
  // Basically, the diffuse, ambient, specular and
  // emissive channels get combined additively, clamped,
  // and then get treated as an emissive channel.
  // Another way to think about it is when
  // m_bDisableLighting is true, render the same way
  // OpenGL does when ::glDisable( GL_LIGHTING ) is called.

private:
  unsigned char m_reserved1[2];
#if defined(ON_64BIT_POINTER)
  unsigned char m_reserved2[4];
#endif
public:

  /*
  Description:
    Searches for a texure with matching texture_id.
    If more than one texture matches, the first match
    is returned.
  Parameters:
    texture_id - [in]
  Returns:
    >=0 m_textures[] index of matching texture
    -1 if no match is found.
  */
  int FindTexture(
    ON_UUID texture_id
    ) const;

  /*
  Description:
    Searches for a texure with matching filename and type.
    If more than one texture matches, the first match
    is returned.
  Parameters:
    filename - [in]  If NULL, then any filename matches.
    type - [in] If ON_Texture::no_texture_type, then
                any texture type matches.
    i0 - [in] If i0 is < 0, the search begins at 
              m_textures[0], if i0 >= m_textures.Count(),
              -1 is returnd, otherwise, the search begins
              at m_textures[i0+1].
  Example:
    Iterate through all the the bitmap textures on 
    a material.

          ON_Material& mat = ...;
          int ti = -1;
          int bitmap_texture_count = 0;
          for(;;)
          {
            ti = mat.FindTexture( 
                        NULL, 
                        ON_Texture::bitmap_texture, 
                        ti );

            if ( ti < 0 )
            {
              // no more bitmap textures
              break;
            }

            // we have a bitmap texture
            bitmap_texture_count++;
            const ON_Texture& bitmap_texture = mat.m_textures[ti];
            ...
          }

  Returns:
    >=0 m_textures[] index of matching texture
    -1 if no match is found.
  */
  int FindTexture(
    const wchar_t* filename,
    ON_Texture::TYPE type,
    int i0 = -1
    ) const;

  /*
  Description:
    If there is already a texture with the same file name and
    type, then that texture is modified, otherwise a new texture
    is added.  If tx has user data, the user data is copied
    to the m_textures[] element.
  Parameters:
    tx - [in]
  Returns:
    Index of the added texture in the m_textures[] array.
  Remarks:
    This is intended to be a quick and simple way to add
    textures to the material.  If you need to do something
    different, then just work on the m_textures[] array.
  */
  int AddTexture( 
    const ON_Texture& tx
    );

  /*
  Description:
    If there is a texture with a matching type, that texture's
    filename is modified, otherwise a new texture is added.    
  Parameters:
    filename - [in] new filename
    type - [in]
  Returns:
    Index of the added texture in the m_textures[] array.
  Remarks:
    This is intended to be a quick and simple way to add
    textures to the material.  If you need to do something
    different, then just work on the m_textures[] array.
  */
  int AddTexture(
    const wchar_t* filename,
    ON_Texture::TYPE type 
    );

  /*
  Description:
    Deletes all texures with matching filenames and types.
  Parameters:
    filename - [in]  If NULL, then any filename matches.
    type - [in] If ON_Texture::no_texture_type, then
                any texture type matches.
  Returns:
    Number of textures deleted.
  */
  int DeleteTexture(
    const wchar_t* filename,
    ON_Texture::TYPE type 
    );

  ON_ObjectArray<ON_Texture> m_textures;

  /*
  Description:
    Used to provide per face material support. 
    The parent object reference a basic material. 
    When a brep face or mesh facet wants to use
    a material besides the base material, it specifies
    a channelSupports material channel.  The default
    material channel is 0 and that indicates the base
    material.  A channel of n > 0 means that face
    used the material with id m_material_channel[n-1].
    If (n-1) >= m_material_channel.Count(), then the base
    material is used.  The value of 
    m_material_channel[n].m_id is persistent.  The
    value of m_material_channel[n].m_i is a runtime
    index in the CRhinoDoc::m_material_table[].  If 
    CRhinoDoc::m_material_table[m_i].m_uuid != m_id,
    then m_id is assumed to be correct.
  */
  ON_SimpleArray<ON_UuidIndex> m_material_channel;

  ON_UUID m_plugin_id; // ID of the last plug-in to modify this material

private:
  static double m_max_shine;
  bool ReadV3Helper( ON_BinaryArchive& file, int minor_version );
  bool WriteV3Helper( ON_BinaryArchive& file ) const;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Material>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_Material>;
#pragma warning( pop )
#endif

#endif


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

#if !defined(OPENNURBS_RENDERING_INC_)
#define OPENNURBS_RENDERING_INC_


class ON_CLASS ON_RenderingAttributes
{
public:
  ON_RenderingAttributes();
  void Default();
  int Compare( const ON_RenderingAttributes& other ) const;
  bool Write( ON_BinaryArchive& archive ) const;
  bool Read( ON_BinaryArchive& archive );

  bool IsValid( ON_TextLog* text_log ) const;


  const ON_MaterialRef* MaterialRef( const ON_UUID& plugin_id ) const;

  ON_ClassArray<ON_MaterialRef> m_materials;
};

class ON_CLASS ON_ObjectRenderingAttributes : public ON_RenderingAttributes
{
public:
  ON_ObjectRenderingAttributes();
  void Default();
  int Compare( const ON_ObjectRenderingAttributes& other ) const;
  bool Write( ON_BinaryArchive& archive ) const;
  bool Read( ON_BinaryArchive& archive );

  bool IsValid( ON_TextLog* text_log ) const;

  /*
  Description:
    Update mapping channel transformations.
  Parameters:
    xform - [in]
      Transformation applied to parent object.
  Returns:
    True is successful.  False if there are mapping channels
    and xform cannot be inverted.
  */
  bool Transform( const ON_Xform& xform );

  /*
  Parameters:
    plugin_id - [in]
  Returns:
    A pointer to the plug-in's mapping reference, if there
    is one. Otherwise NULL is returned.
  */
  const ON_MappingRef* MappingRef( 
    const ON_UUID& plugin_id 
    ) const;

  /*
  Parameters:
    plugin_id - [in]
  Returns:
    If a mapping ref exists, it is returned.  Otherwise
    one is added.
  */
  ON_MappingRef* AddMappingRef( 
    const ON_UUID& plugin_id 
    );

  /*
  Parameters:
    plugin_id - [in]
  Returns:
    If a mapping ref exists, it is returned.  Otherwise
    one is added.
  */
  bool DeleteMappingRef( 
    const ON_UUID& plugin_id 
    );


  /*
  Parameters:
    plugin_id - [in]
    mapping_channel_id - [in]
    mapping_id - [in]
      ON_TextureMapping id
  Returns:
    A pointer to the plug-in's mapping channel, if there
    is one. Otherwise NULL is returned.
  */
  const ON_MappingChannel* MappingChannel( 
    const ON_UUID& plugin_id, 
    int mapping_channel_id
    ) const;

  const ON_MappingChannel* MappingChannel( 
    const ON_UUID& plugin_id, 
    const ON_UUID& mapping_id
    ) const;


  /*
  Parameters:
    plugin_id - [in]
    mapping_channel_id - [in]
    mapping_id - [in]
      ON_TextureMapping id
  Returns:
    True if the mapping channel was added or a pefect
    match already existed.  False if a mapping channel 
    with a different mapping_id already exists for this
    plug-in and channel.
  */
  bool AddMappingChannel(
    const ON_UUID& plugin_id, 
    int mapping_channel_id,
    const ON_UUID& mapping_id
    );

  /*
  Parameters:
    plugin_id - [in]
    mapping_channel_id - [in]
    mapping_id - [in]
      ON_TextureMapping id
  Returns:
    True if a matching mapping channel was deleted.
  */
  bool DeleteMappingChannel(
    const ON_UUID& plugin_id, 
    int mapping_channel_id
    );

  bool DeleteMappingChannel(
    const ON_UUID& plugin_id, 
    const ON_UUID& mapping_id
    );

  /*
  Parameters:
    plugin_id - [in]
    old_mapping_channel_id - [in]
    new_mapping_channel_id - [in]
  Returns:
    True if a matching mapping channel was found and changed.
  */
  bool ChangeMappingChannel(
    const ON_UUID& plugin_id, 
    int old_mapping_channel_id,
    int new_mapping_channel_id
    );

  // Use AddMappingRef() or AddMappingChannel() if you 
  // want to add an element to this array.
  //
  // Every mapping ref in this array must have
  // a distinct value of ON_MappingRef.m_plugin_id.
  ON_ClassArray<ON_MappingRef> m_mappings;

  /*
  Parameters:
    bEnable - [in]
      false - (default)
       Do not generate bitmap textures that 
       approximate procedural textures.
      true - 
       generate bitmap textures that approximate
       procedural textures and use these for
       quick previews.
  Returns:
    True if advancded texture preview is enabled.
  */
  void EnableAdvancedTexturePreview(bool b);

  /*
  Returns:
    True if advancded texture preview is enabled.
  */
  bool AdvancedTexturePreview() const;

  bool m_bCastsShadows;    // default is true
  bool m_bReceivesShadows; // default is true

private:
  // m_bits encodes 8 true/false settings
  unsigned char m_bits; // (m_bits & 1) == AdvancedTexturePreview();

  unsigned char m_reserved1;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_RenderingAttributes>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_ObjectRenderingAttributes>;
#pragma warning( pop )
#endif


#endif


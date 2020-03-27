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

#if !defined(OPENNURBS_INSTANCE_INC_)
#define OPENNURBS_INSTANCE_INC_

/*
Description:
  An ON_InstanceDefinition defines the geometry used by 
  instance references.
See Also:
  ON_InstanceRef
*/
class ON_CLASS ON_InstanceDefinition : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_InstanceDefinition);

public:

  // IDEF_UPDATE_TYPE lists the possible relationships between
  // the instance definition geometry and the archive 
  // (m_source_archive) containing the original defition.
  enum IDEF_UPDATE_TYPE
  {
    static_def = 0,
    embedded_def = 1,
      // As of 7 February, "static_def" and "embedded_def" 
      // and shall be treated the same. Using "static_def"
      // is prefered and "embedded_def" is obsolete.
      // The geometry for the instance definition
      // is saved in archives, is fixed and has no
      // connection to a source archive.
      // All source archive information should be
      // empty strings and m_source_archive_checksum
      // shoule be "zero".
    linked_and_embedded_def = 2,
      // The geometry for the instance definition
      // is saved in archives.  Complete source
      // archive and checksum information will be 
      // present. The document setting 
      // ON_3dmIOSettings.m_idef_link_update 
      // determines if, when and how the instance
      // definition geometry is updated by reading the
      // source archive.
    linked_def = 3,   
      // The geometry for this instance definition
      // is not saved in the archive that contains
      // this instance definition. This instance 
      // definition geometry is imported from a
      // "source archive" The "source archive" file
      // name and checksum information are saved
      // in m_source_archive and m_source_archive_checksum.
      // If file named in m_source_archive is not available, 
      // then this instance definition is not valid and any
      // references to it are not valid.
    force_32bit_idef_update_type = 0xFFFFFFFF
  };

  // Converts and integer into an IDEF_UPDATE_TYPE enum.
  static IDEF_UPDATE_TYPE IdefUpdateType(int i);

  // Bits that identify subsets of the instance defintion
  // fields. These bits are used to determine which fields to
  // set when an ON_InstanceDefinition class is used to
  // modify an existing instance definition.
  enum
  {
    no_idef_settings            =    0,
    idef_name_setting           =    1,  // m_name
    idef_description_setting    =    2,  // m_description
    idef_url_setting            =    4,  // all m_url_* fields
    idef_units_setting          =    8,  // m_us and m_unit_scale
    idef_source_archive_setting = 0x10,  // all m_source_*, layer style, update depth fields
    idef_userdata_setting       = 0x20, 
    all_idef_settings           = 0xFFFFFFFF
  };

public:
  ON_InstanceDefinition();
  ~ON_InstanceDefinition();

  // virtual ON_Object overrides
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // virtual ON_Object::Dump override
  void Dump( ON_TextLog& ) const;

  ON_BOOL32 Write(
         ON_BinaryArchive& binary_archive
       ) const;
  ON_BOOL32 Read(
         ON_BinaryArchive& binary_archive
       );
  ON::object_type ObjectType() const;
  unsigned int SizeOf() const;

  // virtual ON_Geometry overrides
  int Dimension() const;
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );

  // virtual
  ON_UUID ModelObjectId() const;

  // Interface
  const wchar_t* Name() const;
  void SetName( const wchar_t* name );

  const wchar_t* Description() const;
  void SetDescription( const wchar_t* description );

  const wchar_t* URL() const;
  void SetURL( const wchar_t* url );

  const wchar_t* URL_Tag() const;
  void SetURL_Tag( const wchar_t* url_tag );

  ON_UUID Uuid() const;
  void SetUuid( ON_UUID uuid );

  void SetBoundingBox( ON_BoundingBox bbox );

  // list of object ids in the instance geometry table.
  ON_SimpleArray<ON_UUID> m_object_uuid;

  /*
  Description:
    If the instance definition is linked or embedded, use
    SetSource to specify the source archive.
  Parameters:
    source_archive - [in] name of source archive
    checksum - [in] check sum used to detect changed.
      Generally, you will pass ON_CheckSum::UnsetCheckSum
      for this argument and Rhino will handle setting
      the checksum to the appropriate value at the appropriate
      time.
    source_type - [in]
      If source_archive and checksum are empty, then
      source_type is ignored and static_def will be used.
      If source_archive is a nonempty string and checksum
      is set, then source_type must be either 
      linked_and_embedded_def or linked_def.  If you
      are changing the source archive of a valid idef,
      then simply pass this->IdefUpdateType().
  Remarks:
    See the IDEF_UPDATE_TYPE comments for more details.
  */
  void SetSourceArchive( 
        const wchar_t* source_archive, 
        ON_CheckSum checksum,
        IDEF_UPDATE_TYPE update_type
        );

  /*
  Description:
    Destroys all source archive information.
    Specifically:
      * m_source_archive is set to the empty string.
      * m_source_bRelativePath is set to false
      * The alternative source archive path is set
        to the empty string.
      * m_source_archive_checksum.Zero() is used to
        destroy all checksum information.
      * m_idef_update_type is set to static_def.
  */
  void DestroySourceArchive();

  /*
  Returns:
    Name of source archive.
  */
  const wchar_t* SourceArchive() const;

  /*
  Returns:
    Check sum of source archive.
  */
  ON_CheckSum SourceArchiveCheckSum() const;

  const ON_UnitSystem& UnitSystem() const;

  /*
  Description:
    Use this function to specify an alternate location to
    look for a linked instance defininition archive if it
    cannot be found in the location specified by m_source_archive.
  Parameters:
    alternate_source_archive_path - [in]
      alterate location. pass null to delete the alternate path.
    bRelativePath - [in]
      true if alternate_source_archive_path is a relative path.
  */
  void SetAlternateSourceArchivePath( 
        const wchar_t* alternate_source_archive_path,
        bool bRelativePath
        );

  /*
  Description:
    If there is an alternate location to look for a linked instance
    defininition archive when it cannot be found in the location 
    specified by m_source_archive, then function will return the
    alterate location.
  Parameters:
    alternate_source_archive_path - [out]
    bRelativePath - [out]
      true if alternate_source_archive_path is a relative path.
  */
  bool GetAlternateSourceArchivePath( 
        ON_wString& alternate_source_archive_path,
        bool& bRelativePath
        ) const;
  /*
  Description:
    Sets m_us and m_unit_scale.
  */
  void SetUnitSystem( ON::unit_system us );
  void SetUnitSystem( const ON_UnitSystem& us );

  /*
  Returns:
    True if this is a linked instance definition with
    layer settings information.
  */
  bool HasLinkedIdefLayerSettings() const;

  /*
  Description:
    Set linked instance definition reference file layer settings.
  Parameters:
    layer_settings - [in/out]
      input: layer settings read from the linked file.
      output: layer settings to use in the context of the idef.
  */
  void UpdateLinkedIdefReferenceFileLayerSettings( unsigned int layer_count, ON_Layer** layer_settings );

  /*
  Description:
    Set linked instance definition parent layer information. 
    Typically this is done just before the linked idef is 
    saved to a file.
  Parameters:
    linked_idef_parent_layer - [in]
  */
  void UpdateLinkedIdefParentLayerSettings( const ON_Layer* linked_idef_parent_layer );

  const ON_Layer* LinkedIdefParentLayerSettings() const;

  /*
  Description:
    When a linked instance definition is read and its layers are added to the
    context when the idef exists, runtime layer ids may need to be changed
    when an id collision occures.  In this case, use this function to
    inform the linked instance definition of the map from runtime layer
    id to the layer id found in the linked file.
  Parameters:
    id_map - [in]
      The first id in the pair is the layer id in the current context
      where the idef is being used.
      The second id in the pair is the layer id found in the linked file.
  */
  void UpdateLinkedIdefReferenceFileLayerRuntimeId( const ON_UuidPairList& id_map );

  /*
  Description:
    Set linked instance definition layer settings.
    Typically this is done just before the linked idef is 
    saved to a file.
  Parameters:
    layer_settings - [in]
      Layer settings in the context where the linked idef is being used.
  Remarks:
    Linked idefs save the original layer informtion from the linked file.
    In the context where the idef is used, some of those settings (color,
    visibility, ...) can be modified. This function saves those modifications
    so the can be applied the next time the linked idef is read.
  */
  void UpdateLinkedIdefLayerSettings( unsigned int layer_count, const ON_Layer*const* layer_settings );

public:

  ON_UUID m_uuid;     // unique id for this instance definition
  ON_wString m_name;  // The "name" is for human comfort.  
                      // It can be empty and duplicates
                      // may exist. Instance reference use
                      // m_uuid to find instance definitions.
  ON_wString m_description; 

  ON_wString m_url;
  ON_wString m_url_tag;     // UI link text for m_url

#if defined(ON_32BIT_POINTER)
private:
  // 24 January 2011:
  //   Because the Rhino 4 and 5 SDKs are fixed, the offset of 
  //   existing fields cannot be changed and the m_reserved1
  //   value has to be located in different places for 
  //   32 and 64 bit builds.
  unsigned int m_reserved1;
#endif

public:
  ON_BoundingBox m_bbox;

  ON_UnitSystem  m_us;
  
  // Note: the embedded_def type is obsolete.
  //  To avoid having to deal with this obsolete type in
  //  your code, using ON_InstanceDefintion::IdefUpdateType()
  //  to get this value.  The IdefUpdateType() function
  //  with convert the obsolte value to the correct
  //  value.
  IDEF_UPDATE_TYPE m_idef_update_type; 

  IDEF_UPDATE_TYPE IdefUpdateType() const;

  int m_idef_update_depth; // Controls how much geometry is read when
                           // a linked idef is updated.
                           //   0: read everything, included nested linked idefs
                           //   1: skip nested linked idefs.

  ON_wString m_source_archive;   // filename used to update idef 
                                 // (it can be empty or relative)
  bool m_source_bRelativePath;  // True if the filename in m_source_archive is
                                 // a relative the location of the 3dm file
                                 // containing this instance definition.

  // A static or linked_and_embedded idef must have m_layer_style = 0
  // A linked idef must have m_layer_style = 1 or 2
  //   0 = unset
  //   1 = active (linked idef layers will be active)
  //   2 = reference (linked idef layers will be reference)
  unsigned char m_idef_layer_style;
                               
private:
  unsigned char m_reserved2[2];

#if defined(ON_64BIT_POINTER)
private:
  // 24 January 2011:
  //   Because the Rhino 4 and 5 SDKs are fixed, the offset of 
  //   existing fields cannot be changed and the m_runtime_sn
  //   value has to be located in different places for 
  //   32 and 64 bit builds.
  unsigned int m_reserved1;
#endif

public:
  ON_CheckSum m_source_archive_checksum; // used to detect when idef is out of
                                         // synch with source archive.
};


/*
Description:
  An ON_InstanceRef is a reference to an instance definition
  along with transformation to apply to the definition.
See Also:
  ON_InstanceRef
*/
class ON_CLASS ON_InstanceRef : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_InstanceRef);

public:
  ON_InstanceRef();

  /////////////////////////////////////////////////////////////
  //
  // virtual ON_Object overrides
  //
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  ON_BOOL32 Write(
         ON_BinaryArchive& binary_archive
       ) const;
  ON_BOOL32 Read(
         ON_BinaryArchive& binary_archive
       );
  ON::object_type ObjectType() const;

  /////////////////////////////////////////////////////////////
  //
  // virtual ON_Geometry overrides
  //
  int Dimension() const;
  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;
  ON_BOOL32 Transform( 
         const ON_Xform& xform
         );

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  /////////////////////////////////////////////////////////////
  //

  // Unique id of the instance definition (ON_InstanceDefinition) 
  // in the instance definition table that defines the geometry
  // used by this reference.
  ON_UUID m_instance_definition_uuid;

  // Transformation for this reference.
  ON_Xform m_xform;

  // Bounding box for this reference.
  ON_BoundingBox m_bbox;

  // Tolerance to use for flagging instance xforms
  // as singular.
  static const double m_singular_xform_tol;
};

#endif

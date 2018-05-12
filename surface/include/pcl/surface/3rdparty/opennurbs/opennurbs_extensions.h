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


#if !defined(OPENNURBS_EXTENSIONS_INC_)
#define OPENNURBS_EXTENSIONS_INC_

#include <pcl/pcl_exports.h>

/*
Description:
  Used to store user data information in an ONX_Model.
*/
class PCL_EXPORTS ON_CLASS ONX_Model_UserData
{
public:
#if defined(ON_DLL_EXPORTS) || defined(ON_DLL_IMPORTS)
  // See comments at the top of opennurbs_extensions.cpp for details.

  // new/delete
  void* operator new(size_t);
  void  operator delete(void*);

  // array new/delete
  void* operator new[] (size_t);
  void  operator delete[] (void*);

  // in place new/delete
  void* operator new(size_t,void*);
  void  operator delete(void*,void*);
#endif

  ONX_Model_UserData();
  ~ONX_Model_UserData();
  ONX_Model_UserData(const ONX_Model_UserData&);
  ONX_Model_UserData& operator=(const ONX_Model_UserData&);

  void Dump( ON_TextLog& ) const;

  ON_UUID  m_uuid;
  ON_3dmGoo m_goo;

private:
  void Destroy();
  unsigned int* m_ref_count; // reference counts used to avoid expensive object copying

public:
  int m_usertable_3dm_version ;
  int m_usertable_opennurbs_version;
};

/*
Description:
  Used to store geometry table object definition and attributes in an ONX_Model.
*/
class PCL_EXPORTS ON_CLASS ONX_Model_Object
{
public:
#if defined(ON_DLL_EXPORTS) || defined(ON_DLL_IMPORTS)
  // See comments at the top of opennurbs_extensions.cpp for details.

  // new/delete
  void* operator new(size_t);
  void  operator delete(void*);

  // array new/delete
  void* operator new[] (size_t);
  void  operator delete[] (void*);

  // in place new/delete
  void* operator new(size_t,void*);
  void  operator delete(void*,void*);
#endif

  ONX_Model_Object();
  ~ONX_Model_Object();
  ONX_Model_Object(const ONX_Model_Object&);
  ONX_Model_Object& operator=(const ONX_Model_Object&);

  void Dump( ON_TextLog& ) const;

  // If m_bDeleteObject is true, then m_object will be deleted when
  // the last ONX_Model_Object that refers to it is destroyed.  The
  // default value of m_bDeleteObject is false.
  bool m_bDeleteObject;
  const ON_Object* m_object;
  ON_3dmObjectAttributes m_attributes;

private:
  void Destroy();
  unsigned int* m_ref_count; // reference counts used to avoid expensive object copying
};

/*
Description:
  Used to store render light table light definition and attributes in an ONX_Model.
*/
class PCL_EXPORTS ON_CLASS ONX_Model_RenderLight
{
public:
#if defined(ON_DLL_EXPORTS) || defined(ON_DLL_IMPORTS)
  // See comments at the top of opennurbs_extensions.cpp for details.

  // new/delete
  void* operator new(size_t);
  void  operator delete(void*);

  // array new/delete
  void* operator new[] (size_t);
  void  operator delete[] (void*);

  // in place new/delete
  void* operator new(size_t,void*);
  void  operator delete(void*,void*);
#endif

  ONX_Model_RenderLight();
  ~ONX_Model_RenderLight();
  ONX_Model_RenderLight(const ONX_Model_RenderLight&);
  ONX_Model_RenderLight& operator=(const ONX_Model_RenderLight&);

  ON_Light m_light;
  ON_3dmObjectAttributes m_attributes;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )

ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_Bitmap*>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Linetype>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_Linetype>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Layer>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_Layer>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Group>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_Group>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_Font>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_Font>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_DimStyle>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_DimStyle>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ONX_Model_RenderLight>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_HatchPattern>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_HatchPattern>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_InstanceDefinition>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ObjectArray<ON_InstanceDefinition>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ONX_Model_Object>;
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ONX_Model_UserData>;
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_HistoryRecord*>;

#pragma warning( pop )
#endif


/*
Description:
  Pedegodgical example of all the things in an OpenNURBS 3dm archive.
  The openNURBS examples use ONX_Model to store the information
  read from 3dm archives.  Please study example_read.cpp for
  details.
*/
class PCL_EXPORTS ON_CLASS ONX_Model
{
public:
#if defined(ON_DLL_EXPORTS) || defined(ON_DLL_IMPORTS)
  // See comments at the top of opennurbs_extensions.cpp for details.

  // new/delete
  void* operator new(size_t);
  void  operator delete(void*);

  // array new/delete
  void* operator new[] (size_t);
  void  operator delete[] (void*);

  // in place new/delete
  void* operator new(size_t,void*);
  void  operator delete(void*,void*);
#endif

  ONX_Model();
  virtual ~ONX_Model();

  /*
  Description:
    Destroys contents of this model and leaves it ready to be reused.
  */
  void Destroy();

  /*
  Description:
    Reads an openNURBS archive and saves the information in this model
  Parameters:
    archive - [in] archive to read from
    error_log - [out] any archive reading errors are logged here.
  Returns:
    true if archive is read with no error.  False if errors occur.
    Error details are logged in error_log.  If crc errors are in
    the archive, then m_crc_error_count is set to the number of
    errors.
  Example:

            // for ASCII file names
            const char* sFileName = ....;
            FILE* fp = ON::OpenFile( sFileName, "rb");

            // for UNICODE file names
            const wchar_t* wsFileName = ....;
            FILE* fp = ON::OpenFile( wsFileName, L"rb");

            bool bModelRead = false;
            bool bModelIsValid = false;

            ON_TextLog error_log;
            ONX_Model model;

            if ( 0 != fp )
            {
              ON_BinaryFile archive( ON::read3dm, fp );
              bModelRead = model.read( archive, error_log );
              ON::CloseFile( fp );
            }

            if ( bModelRead )
            {
              bModelIsValid = model.Validate(error_log);
            }

  See Also:
    ONX_Model::IsValid
    ONX_Model::Write
    ONX_Model::m_crc_error_count
  */
  bool Read( 
         ON_BinaryArchive& archive,
         ON_TextLog* error_log = NULL
         );

  bool Read( 
         const char* filename,
         ON_TextLog* error_log = NULL
         );

  bool Read( 
         const wchar_t* filename,
         ON_TextLog* error_log = NULL
         );

  /*
  Description:
    Writes contents of this model to an openNURBS archive.
    It is a good practice to call Polish() before calling 
    Write so that your file has all the "fluff" that makes it
    complete.  If the model is not valid, then Write will refuse
    to write it.

  Parameters:
    archive - [in]
      archive to write to

    version - [in] 
      Version of the openNURBS archive to write.
        0 default value and suggested.
           When 0 is passed in, the value of ON_BinaryArchive::CurrentArchiveVersion()
           is used.
        2, 3, 4
          If you pass in one of these values, some information 
          in current data structures will not be saved in the
          file. 
          Rhino 2.x can read version 2 files.
          Rhino 3.x can read version 2 and 3 files.
          Rhino 4.x can read version 2, 3 and 4 files.
          Rhino 5.x can read version 2, 3, 4, 5 and 50 files.
          Rhino 5.x writes version 50 files.

    sStartSectionComment - [in] 
      Brief desciption of your app, today's date, etc.

    error_log - [out]
      any archive writing errors are logged here.

  Returns:
    True if archive is written with no error. 
    False if errors occur.
    Error details are logged in error_log.

  Example:

            model = ...;

            model.Polish(); // fill in defaults as needed.

            ON_TextLog error_log;
            if ( !model.IsValid( error_log ) )
            {
              // try to repair the model
              model.Audit(true);
            }

            if ( model.IsValid( error_log ) )
            {

              // for ASCII file names
              const char* sFileName = ....;
              FILE* fp = ON::OpenFile( sFileName, "wb");

              // for UNICODE file names
              const wchar_t* wsFileName = ....;
              FILE* fp = ON::OpenFile( wsFileName, L"wb");

              bool ok = false;
              if ( 0 != fp )
              {
                const char* sStartSectionComment = "...";
                int version = 5; // 2, 3, 4 or 5 are valid
                ON_BinaryFile archive( ON::write3dm, fp );
                ok = model.write( archive, 
                                  version, 
                                  sStartSectionComment, 
                                  error_log );
                ON::CloseFile( fp );
              }
           }

  See Also:
    ONX_Model::Polish
    ONX_Model::IsValid
    ONX_Model::Read
  */
  bool Write( 
         ON_BinaryArchive& archive,
         int version = 0,
         const char* sStartSectionComment = NULL,
         ON_TextLog* error_log = NULL
         );

  bool Write( 
         const char* filename,
         int version = 0,
         const char* sStartSectionComment = NULL,
         ON_TextLog* error_log = NULL
         );

  bool Write( 
         const wchar_t* filename,
         int version = 0,
         const char* sStartSectionComment = NULL,
         ON_TextLog* error_log = NULL
         );

  /*
  Description:
    Check a model to make sure it is valid.
  Parameters:
    text_log - [in] if not NULL and errors are found,
                    a description of the problem is put in
                    this text_log.
  Returns:
    True if the model is valid.
  */
  bool IsValid( ON_TextLog* text_log = NULL ) const;

  /*
  Description:
    Quickly fills in the little details, like making sure there is 
    at least one layer and table indices make sense.  
    For a full blown check and repair, call Audit(true).
  See Also:
    ONX_Model::Audit
  */
  virtual
  void Polish();

  /*
  Description:
    Check a model to make sure it is valid and, if possible
    and requrested, attempt to repair.
  Parameters:
    bAttemptRepair - [in] if true and a problem is found,
         the problem is repaired.
    repair_count - [out] number of successful repairs.
    text_log - [in] if not NULL and errors are found,
                    a description of the problem is put in
                    this text_log.
    warnings - [out]
        If problems were found, warning ids are appended to this list.
          @untitled table
           1      m_material_table[] flaws
           2      layer table is not perfect.
           3      some m_object_table[].m_attributes.m_uuid was nil or not unique.
           4      some m_object_table[].IsValid() is false
           5      some m_idef_table[] has an invalid or duplicate name
           6      warning some m_idef_table[].m_object_uuid[] is not valid
           7      warning some m_object_table[].m_object is null
           8      warning some m_object_table[].m_object->IsValid() is false
           9      warning some m_object_table[].m_attributes is not valid
          10      linetype table is not perfect.
          11      lineset table is not perfect.
          12      some m_idef_table[].m_uuid was nil or not unique.
          13      some m_texture_mapping_table[i].m_mapping_id was nil or not unique.
          14      some m_material_table[i].m_material_id was nil or not unique.
          15      some m_light_table[i].m_light_id was nil or not unique.
  Returns:
    True if model is valid and false if the model has serious 
    @untitled table
    <0      model has serious errors
    =0      model is ok
    >0      number of problems that were found.
  */
  virtual
  int Audit( 
        bool bAttemptRepair,
        int* repair_count,
        ON_TextLog* text_log,
        ON_SimpleArray<int>* warnings
        );

  /////////////////////////////////////////////////////////////////////
  //
  // BEGIN model definitions
  //

  // start section information
  int m_3dm_file_version;
  int m_3dm_opennurbs_version;
  ON_String m_sStartSectionComments;

  // Properties include revision history, notes, information about
  // the applicaton that created the file, and an option preview image.
  ON_3dmProperties m_properties;

  // Settings include tolerance, and unit system, and defaults used
  // for creating views and objects.
  ON_3dmSettings   m_settings;

  // Tables in an openNURBS archive
  ON_SimpleArray<ON_Bitmap*>            m_bitmap_table;
  ON_ObjectArray<ON_TextureMapping>     m_mapping_table;
  ON_ObjectArray<ON_Material>           m_material_table;
  ON_ObjectArray<ON_Linetype>           m_linetype_table;
  ON_ObjectArray<ON_Layer>              m_layer_table;
  ON_ObjectArray<ON_Group>              m_group_table;
  ON_ObjectArray<ON_Font>               m_font_table;
  ON_ObjectArray<ON_DimStyle>           m_dimstyle_table;
  ON_ClassArray<ONX_Model_RenderLight>  m_light_table;
  ON_ObjectArray<ON_HatchPattern>       m_hatch_pattern_table;
  ON_ObjectArray<ON_InstanceDefinition> m_idef_table;
  ON_ClassArray<ONX_Model_Object>       m_object_table;
  ON_SimpleArray<ON_HistoryRecord*>     m_history_record_table;
  ON_ClassArray<ONX_Model_UserData>     m_userdata_table;

  // The id index fields are used to lookup objects by id
  ON_UuidIndexList m_mapping_id_index;
  ON_UuidIndexList m_material_id_index;
  ON_UuidIndexList m_object_id_index;
  ON_UuidIndexList m_idef_id_index;

  // length of archive returned by ON_BinaryArchive::Read3dmEndMark()
  size_t m_file_length;

  // Number of crc errors found during archive reading.
  // If > 0, then the archive is corrupt.
  int m_crc_error_count;

  //
  // END model definitions
  //
  /////////////////////////////////////////////////////////////////////

  /*
  Returns:
    Bounding box of every object in m_object_table[].
  */
  ON_BoundingBox BoundingBox() const;

  /*
  Description:
    Get render material from object attributes.
  Parameters:
    attributes - [in] object attributes.
    material - [out] render material
  */
  void GetRenderMaterial( 
        const ON_3dmObjectAttributes& attributes,
        ON_Material& material 
        ) const;

  /*
  Description:
    Get render material from object_index.
  Parameters:
    object_index - [in] m_object_table[] index
    material - [out] render material
  */
  void GetRenderMaterial( 
        int object_index,
        ON_Material& material 
        ) const;

  /*
  Description:
    Get linetype from object attributes.
  Parameters:
    attributes - [in] object attributes.
    linetype - [out] linetype
  */
  void GetLinetype( 
        const ON_3dmObjectAttributes& attributes,
        ON_Linetype& linetype 
        ) const;

  /*
  Description:
    Get linetype from object_index.
  Parameters:
    object_index - [in] m_object_table[] index
    linetype - [out] linetype
  */
  void GetLinetype(
        int object_index,
        ON_Linetype& linetype 
        ) const;

  /*
  Description:
    Get wireframe drawing color from object attributes.
  Parameters:
    attributes - [in] object attributes.
  Returns:
    Wireframe drawing color.
  */
  ON_Color WireframeColor(const ON_3dmObjectAttributes& attributes) const;

  /*
  Description:
    Get wireframe drawing color from object attributes.
  Parameters:
    object_index - [in] m_object_table[] index
  Returns:
    Wireframe drawing color.
  */
  ON_Color WireframeColor(int object_index) const;

  /* 
  Description:
    Get index of object in m_object_table from object_uuid.
  Parameters:
    object_uuid - [in] object uuid.
  Returns:
    Index of the object or -1 if it is not found.
  */
  virtual
  int ObjectIndex( 
    ON_UUID object_uuid 
    ) const;

  /* 
  Description:
    Get instance definition from instance definition table.
  Parameters:
    idef_uuid - [in] instance definition uuid.
  Example:

          ON_XModel model = ...;
          ..
          ON_InstanceRef* pIRef = ..;
          ON_UUID idef_uuid = pIRef->m_instance_definition_uuid;
          int idef_index = model.IDefIndex( idef_uuid );
          if ( idef_index >= 0 )
          {
            const ON_InstanceDefinition& idef = model.m_idef_table[idef_index];
            ...
          }

  Returns:
    Index of the instance definition or -1 if it is not found.
  */
  virtual
  int IDefIndex( 
    ON_UUID idef_uuid 
    ) const;

  /* 
  Description:
    Get instance definition index from instance definition name.
  Parameters:
    idef_name - [in] name to search for
  Returns:
    Index of the instance definition or -1 if it is not found.
  */
  virtual
  int IDefIndex( 
    const wchar_t* idef_name
    ) const;

  /* 
  Description:
    Get instance definition name that is not currently in use.
  */
  virtual
  void GetUnusedIDefName( ON_wString& idef_name ) const;

  /* 
  Description:
    See if the instance reference iref refers to an instance
    definition.
  Parameters:
    iref - [in]
    idef_uuid - [in] id of idef we are looking for
  Returns:
    @untitled table
     0         iref does not use idef
     1         iref directly references idef
    >1         iref has a nested reference to idef (nesting depth returned)
    -1         iref.m_instance_definition_uuid is not valid
    -2         invalid idef found
  */
  virtual
  int UsesIDef( 
        const ON_InstanceRef& iref,
        ON_UUID idef_uuid
        ) const;

  /* 
  Description:
    Get layer definition from layer table.
  Parameters:
    layer_name - [in] name to search for
  Example:

          ON_XModel model = ...;
          ..
          ON_InstanceRef* pIRef = ..;
          ON_UUID idef_uuid = pIRef->m_instance_definition_uuid;
          int layer_index = model.IDefIndex( idef_uuid );
          if ( idef_index >= 0 )
          {
            const ON_InstanceDefinition& idef = model.m_idef_table[idef_index];
            ...
          }

  Returns:
    Index of the layer or -1 if it is not found.
  */
  virtual
  int LayerIndex( 
    const wchar_t* layer_name
    ) const;

  /* 
  Description:
    Get layer name that is not currently in use.
  */
  virtual
  void GetUnusedLayerName( ON_wString& layer_name ) const;

  /////////////////////////////////////////////////////////////////////
  //
  // BEGIN model document level user string tools
  //

  /*
  Description:
    Attach a user string to the document.
  Parameters:
    key - [in] id used to retrieve this string.
    string_value - [in] 
      If NULL, the string with this id will be removed.
  Returns:
    True if successful.
  */
  bool SetDocumentUserString( 
    const wchar_t* key, 
    const wchar_t* string_value 
    );

  /*
  Description:
    Get user string from the document.
  Parameters:
    key - [in] id used to retrieve the string.
    string_value - [out]
  Returns:
    True if a string with id was found.
  */
  bool GetDocumentUserString( 
    const wchar_t* key, 
    ON_wString& string_value 
    ) const;

  /*
  Description:
    Get a list of all user strings in the document.
  Parameters:
    user_strings - [out]
      user strings are appended to this list.
  Returns:
    Number of elements appended to the user_strings list.
  */
  int GetDocumentUserStrings( ON_ClassArray<ON_UserString>& user_strings ) const;

  //
  // END model document level user string tools
  //
  /////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////
  //
  // BEGIN model text dump tools
  //

  // text dump of entire model
  void Dump( ON_TextLog& ) const;
  
  // text dump of model properties and settings
  void DumpSummary( ON_TextLog& ) const;

  // text dump of bitmap table
  void DumpBitmapTable( ON_TextLog& ) const; 

  // text dump of texture mapping table
  void DumpTextureMappingTable( ON_TextLog& ) const; 

  // text dump of render material table
  void DumpMaterialTable( ON_TextLog& ) const; 

  // text dump of line type table
  void DumpLinetypeTable( ON_TextLog& ) const; 

  // text dump of layer table
  void DumpLayerTable( ON_TextLog& ) const;

  // text dump of light table
  void DumpLightTable( ON_TextLog& ) const;

  // text dump of group table
  void DumpGroupTable( ON_TextLog& ) const;

  // text dump of font table
  void DumpFontTable( ON_TextLog& ) const;

  // text dump of dimstyle table
  void DumpDimStyleTable( ON_TextLog& ) const;

  // text dump of hatch pattern table
  void DumpHatchPatternTable( ON_TextLog& ) const;

  // text dump of instance definition table
  void DumpIDefTable( ON_TextLog& ) const;

  // text dump of object table
  void DumpObjectTable( ON_TextLog& ) const;

  // text dump of object table
  void DumpHistoryRecordTable( ON_TextLog& ) const;

  // text dump of user data table
  void DumpUserDataTable( ON_TextLog& ) const;

  //
  // END model text dump tools
  //
  /////////////////////////////////////////////////////////////////////

  /*
  Description:
    Destroys cached searching and bounding box information.  Call
    if you modify the m_object_table or m_idef_table.
  */
  void DestroyCache();

  /////////////////////////////////////////////////////////////////////
  //
  // BEGIN Render Development Toolkit (RDK) information
  //
  static bool IsRDKDocumentInformation(const ONX_Model_UserData& docud);
  static bool GetRDKDocumentInformation(const ONX_Model_UserData& docud,ON_wString& rdk_xml_document_data);

  static bool IsRDKObjectInformation(const ON_UserData& objectud);
  static bool GetRDKObjectInformation(const ON_Object& object,ON_wString& rdk_xml_object_data);
  //
  // END Render Development Toolkit (RDK) information
  //
  /////////////////////////////////////////////////////////////////////


private:
  // prohibit use of copy construction and operator=
  ONX_Model(const ONX_Model&);
  ONX_Model& operator=(const ONX_Model&);

private:

  // This bounding box contains all objects in the object table.
  ON_BoundingBox m__object_table_bbox;
};

/*
Description:
  Tests a string to see if it is valid as a name for a layer,
  object, material, linetype, instance definition, etc.
Parameters:
  name - [in] string to test
Returns:
  True if the string is a valid name.
*/
ON_DECL
bool ONX_IsValidName( 
          const wchar_t* name 
          );

#endif

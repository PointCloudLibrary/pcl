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

#if !defined(OPENNURBS_GROUP_INC_)
#define OPENNURBS_GROUP_INC_

class ON_CLASS ON_Group : public ON_Object
{
  ON_OBJECT_DECLARE(ON_Group);
public:
  ON_Group();
  ~ON_Group();
  // C++ default copy construction and operator= work fine.
  // Do not add custom versions.

  //////////////////////////////////////////////////////////////////////
  //
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
         ON_BinaryArchive&  // serialize definition to binary archive
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // restore definition from binary archive
       );

  //////////////////////////////////////////////////////////////////////
  //
  // Obsolete interface - just work on the public members
  void SetGroupName( const wchar_t* );
  void SetGroupName( const char* );
  
  void GetGroupName( ON_wString& ) const;
  const wchar_t* GroupName() const;

  void SetGroupIndex(int);
  int GroupIndex() const;

public:
  ON_wString m_group_name;
  int m_group_index;
  ON_UUID m_group_id;
};

#endif

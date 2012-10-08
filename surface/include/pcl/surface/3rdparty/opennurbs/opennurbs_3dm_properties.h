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

#if !defined(OPENNURBS_3DM_PROPERTIES_INC_)
#define OPENNURBS_3DM_PROPERTIES_INC_

//////////////////////////////////////////////////////////////////////////////////////////

class ON_CLASS ON_3dmRevisionHistory
{
public:
  ON_3dmRevisionHistory();
  ~ON_3dmRevisionHistory();
  // C++ default operator= and copy constructor work fine.

  void Default();
  ON_BOOL32 IsValid() const;
  int NewRevision(); // returns updated revision count

  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  void Dump( ON_TextLog& ) const;

  /*
  Returns:
    true 
      if m_create_time is >= January 1, 1970
  */
  bool CreateTimeIsSet() const;

  /*
  Returns:
    true 
      if m_last_edit_time is >= January 1, 1970
  */
  bool LastEditedTimeIsSet() const;

  ON_wString m_sCreatedBy;
  ON_wString m_sLastEditedBy;
  struct tm  m_create_time;     // UCT create time
  struct tm  m_last_edit_time;  // UCT las edited time
  int        m_revision_count;
};

//////////////////////////////////////////////////////////////////////////////////////////

class ON_CLASS ON_3dmNotes
{
public:
  ON_3dmNotes();
  ON_3dmNotes( const ON_3dmNotes& );
  ~ON_3dmNotes();
  ON_3dmNotes& operator=(const ON_3dmNotes&);

  void Default();
  ON_BOOL32 IsValid() const;

  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  void Dump(ON_TextLog&) const;

  ////////////////////////////////////////////////////////////////
  //
  // Interface - this information is serialized.  Applications
  // may want to derive a runtime class that has additional
  // window and font information.
  ON_wString m_notes; // UNICODE
  ON_BOOL32 m_bVisible;    // true if notes window is showing
  ON_BOOL32 m_bHTML;       // true if notes are in HTML

  // last window position
  int m_window_left;
  int m_window_top;
  int m_window_right;
  int m_window_bottom;
};

//////////////////////////////////////////////////////////////////////////////////////////

class ON_CLASS ON_3dmApplication
{
  // application that created the 3dm file
public:
  ON_3dmApplication();
  ON_3dmApplication( const ON_3dmApplication& );
  ~ON_3dmApplication();
  ON_3dmApplication& operator=(const ON_3dmApplication&);

  void Default();
  ON_BOOL32 IsValid() const;

  ON_BOOL32 Read( ON_BinaryArchive& );
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  void Dump( ON_TextLog& ) const;

  ON_wString m_application_name;    // short name like "Rhino 2.0"
  ON_wString m_application_URL;     // URL
  ON_wString m_application_details; // whatever you want
};

//////////////////////////////////////////////////////////////////////////////////////////

class ON_CLASS ON_3dmProperties
{
public:
  ON_3dmProperties();
  ~ON_3dmProperties();
  ON_3dmProperties(const ON_3dmProperties&);
  ON_3dmProperties& operator=(const ON_3dmProperties&);

  void Default();

  ON_BOOL32 Read(ON_BinaryArchive&);
  ON_BOOL32 Write(ON_BinaryArchive&) const;

  void Dump( ON_TextLog& ) const;

  ON_3dmRevisionHistory  m_RevisionHistory;
  ON_3dmNotes            m_Notes;
  ON_WindowsBitmap       m_PreviewImage;     // preview image of model
  ON_3dmApplication      m_Application;      // application that created 3DM file
};

//////////////////////////////////////////////////////////////////////////////////////////

#endif

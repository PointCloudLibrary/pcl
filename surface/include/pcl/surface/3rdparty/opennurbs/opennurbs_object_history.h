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

#if !defined(ON_OBJECT_HISTORY_INC_)
#define ON_OBJECT_HISTORY_INC_

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray< class ON_Value* >;
#pragma warning( pop )
#endif

class ON_CLASS ON_CurveProxyHistory
{
public:
  // Used to save information needed to create an ON_CurveProxy
  // reference in history records.
  ON_CurveProxyHistory();
  ~ON_CurveProxyHistory();

  ON_ObjRef m_curve_ref;                // from ON_CurveProxy.m_real_curve
  bool      m_bReversed;                // from ON_CurveProxy.m_bReversed
  ON_Interval m_full_real_curve_domain; // from ON_CurveProxy.m_real_curve.Domain()
  ON_Interval m_sub_real_curve_domain;  // from ON_CurveProxy.m_real_curve_domain
  ON_Interval m_proxy_curve_domain;     // from ON_CurveProxy.m_this_domain

  void Destroy();
  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );
  void Dump( ON_TextLog& ) const;

private:
  ON__UINT8 m_reserved[64];
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_CurveProxyHistory>;
#pragma warning( pop )
#endif

class ON_CLASS ON_PolyEdgeHistory
{
public:
  // Used to save information needed to create an CRhinoPolyEdge
  // reference in history records.
  ON_PolyEdgeHistory();
  ~ON_PolyEdgeHistory();

  void Destroy();
  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );
  void Dump( ON_TextLog& ) const;

  ON_ClassArray< ON_CurveProxyHistory > m_segment;
  ON_SimpleArray<double> m_t;
  int m_evaluation_mode;
private:
  ON__UINT8 m_reserved[64];
};

class ON_CLASS ON_HistoryRecord : public ON_Object
{
  ON_OBJECT_DECLARE(ON_HistoryRecord);
public:
  ON_HistoryRecord();
  ~ON_HistoryRecord();

  // The copy constructor and operator= create duplicates
  // of the linked list of ON_Value classes.
  ON_HistoryRecord(const ON_HistoryRecord& src);
  ON_HistoryRecord& operator=(const ON_HistoryRecord& src);

  // virtual ON_Object::IsValid override
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  // virtual ON_Object::Dump override
  void Dump( ON_TextLog& ) const;
  // virtual ON_Object::Write override
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  // virtual ON_Object::Read override
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);
  // virtual
  ON_UUID ModelObjectId() const;

  //////////
  // If history becomes invalid, call Destroy()
  void Destroy();

  void DestroyValue( int value_id );

  /*
  Description:
    For setting values.
  Parameters:
    value_id - [in]
      If there a value with the same input
      id exists, the old value is replaced.
    count - [in]
      Number of values
    b - [in]
      array of count bools
    i - [in]
      array of count ints
    x - [in]
      array of count doubles
    p - [in]
      array of count 3d points
    v - [in]
      array of count 3d vectors
    xform - [in]
      array of count xforms
    c - [in]
      array of count colors
    or - [in]
      array of count object references
    g - [in]
      array of count geometry pointers
    u - [in]
      array of uuids
    s - [in]
      string
  */
  bool SetBoolValue(     int value_id, bool b);
  bool SetIntValue(      int value_id, int i);
  bool SetDoubleValue(   int value_id, double x);
  bool SetPointValue(    int value_id, ON_3dPoint p);
  bool SetVectorValue(   int value_id, ON_3dVector v);
  bool SetXformValue(    int value_id, ON_Xform xform);
  bool SetColorValue(    int value_id, ON_Color c);
  bool SetObjRefValue(   int value_id, const ON_ObjRef& oref);
  bool SetPointOnObjectValue( int value_id, const ON_ObjRef& oref, ON_3dPoint point );
  bool SetUuidValue(     int value_id, ON_UUID uuid );
  bool SetStringValue(   int value_id, const wchar_t* s );
  bool SetGeometryValue( int value_id, ON_Geometry* g);
  bool SetPolyEdgeValue( int value_id, const ON_PolyEdgeHistory& polyedge );

  /*
  Description:
    For setting values.
  Parameters:
    value_id - [in]
      If there a value with the same input
      id exists, the old value is replaced.
    count - [in]
      Number of values
    b - [in]
      array of count bools
    i - [in]
      array of count ints
    x - [in]
      array of count doubles
    P - [in]
      array of count 3d points
    V - [in]
      array of count 3d vectors
    xform - [in]
      array of count xforms
    c - [in]
      array of count colors
    or - [in]
      array of count object references
    g - [in]
      array of count geometry pointers
    u - [in]
      array of uuids
    s - [in]
      array of strings
  */
  bool SetBoolValues(     int value_id, int count, const bool* b);
  bool SetIntValues(      int value_id, int count, const int* i);
  bool SetDoubleValues(   int value_id, int count, const double* x);
  bool SetPointValues(    int value_id, int count, const ON_3dPoint* P);
  bool SetVectorValues(   int value_id, int count, const ON_3dVector* V);
  bool SetXformValues(    int value_id, int count, const ON_Xform* xform);
  bool SetColorValues(    int value_id, int count, const ON_Color* c);
  bool SetObjRefValues(   int value_id, int count, const ON_ObjRef* oref);
  bool SetUuidValues(     int value_id, int count, const ON_UUID* u );
  bool SetStringValues(   int value_id, int count, const wchar_t* const* s );
  bool SetStringValues(   int value_id, const ON_ClassArray<ON_wString>& s );
  bool SetGeometryValues( int value_id, const ON_SimpleArray<ON_Geometry*> a);
  bool SetPolyEdgeValues( int value_id, int count, const ON_PolyEdgeHistory* a );

  /*
  Description:
    For retrieving values.
  */
  bool GetStringValue( int value_id, ON_wString& str ) const;
  bool GetBoolValue( int value_id, bool* b ) const;
  bool GetIntValue( int value_id, int* i ) const;
  bool GetDoubleValue( int value_id, double* number ) const;
  bool GetPointValue( int value_id, ON_3dPoint& point ) const;
  bool GetVectorValue( int value_id, ON_3dVector& point ) const;
  bool GetXformValue( int value_id, ON_Xform& point ) const;
  bool GetColorValue( int value_id, ON_Color* color ) const;
  bool GetObjRefValue( int value_id, ON_ObjRef& oref ) const;
  bool GetPointOnObjectValue( int value_id, ON_ObjRef& oref ) const;
  bool GetCurveValue( int value_id, const ON_Curve*& ) const;
  bool GetSurfaceValue( int value_id, const ON_Surface*& ) const;
  bool GetBrepValue( int value_id, const ON_Brep*& ) const;
  bool GetMeshValue( int value_id, const ON_Mesh*& ) const;
  bool GetGeometryValue( int value_id, const ON_Geometry*& ) const;
  bool GetUuidValue( int value_id, ON_UUID* uuid ) const;
  bool GetPolyEdgeValue( int value_id, const ON_PolyEdgeHistory*& polyedge ) const;

  int GetStringValues( int value_id, ON_ClassArray<ON_wString>& string ) const;
  int GetBoolValues( int value_id, ON_SimpleArray<bool>& ) const;
  int GetIntValues( int value_id, ON_SimpleArray<int>& ) const;
  int GetDoubleValues( int value_id, ON_SimpleArray<double>& ) const;
  int GetPointValues( int value_id, ON_SimpleArray<ON_3dPoint>& ) const;
  int GetVectorValues( int value_id, ON_SimpleArray<ON_3dVector>& ) const;
  int GetXformValues( int value_id, ON_SimpleArray<ON_Xform>& ) const;
  int GetColorValues( int value_id, ON_SimpleArray<ON_Color>& ) const;
  int GetObjRefValues( int value_id, ON_ClassArray<ON_ObjRef>& objects ) const;
  int GetGeometryValues( int value_id, ON_SimpleArray<const ON_Geometry*>& ) const;
  int GetUuidValues( int value_id, ON_SimpleArray<ON_UUID>& ) const;
  int GetPolyEdgeValues( int value_id, ON_SimpleArray<const ON_PolyEdgeHistory*>& ) const;

  /*
  Desccription:
    Determine if object is an antecedent (input) in this
    history record.
  Parameters:
    object_uuid - [in] 
  Returns:
    Returns true if object_uuid is the id of an input
    object.
  */
  bool IsAntecedent( ON_UUID object_uuid ) const;


  /*
  Description:
    Print a list of the values in text_log.
  Parameters:
    text_log - [in]
  Returns:
    Number of values listed.
  */
  int ValueReport( ON_TextLog& text_log ) const;

  // CRhinoCommand::CommandId() value of the command that
  // created this history record.  Each time the command
  // is run, it can create a history record.
  ON_UUID m_command_id;

  // A YYYYMMDDn version number that gets updated when
  // a command changes.  This version is checked so that
  // new versions of a command's ReplayHistory don't 
  // attempt to use information saved in old files.
  int m_version;

  enum RECORD_TYPE
  {
    history_parameters = 0, // parameters for UpdateHistory
    feature_parameters = 1, // parameters for a feature
    force_32bit_record_type = 0xFFFFFFFF
  };

  RECORD_TYPE m_record_type;

  /*
  Description:
    Convert integer into an ON_HistoryRecord::RECORD_TYPE.
  Parameters:
    i - [in]
  Returns:
    ON_HistoryRecord::RECORD_TYPE enum with same value as i.
  */
  static
  RECORD_TYPE RecordType(int i);

  // Each history record has a unique id that is assigned
  // when the record is added to Rhino's history record table.
  ON_UUID m_record_id;

  // List of object id values of antecedent objects that 
  // are referenced in the list of input events in m_value[].
  // These were the command's "input" objects.
  ON_UuidList m_antecedents;

  // List of object id values of descendant objects that 
  // were created.  These were the command's "output" objects 
  ON_UuidList m_descendants;

  // Information needed to update the descendant objects
  // when an antecedent object is modified.
  ON_SimpleArray< class ON_Value* > m_value;

  /*
  Description:
    This tool is used in rare situations when the object ids 
    stored in the uuid list need to be remapped.
  Parameters:
    uuid_remap - [in]
      Is it critical that uuid_remap[] be sorted with respect
      to ON_UuidPair::CompareFirstUuid.
  */
  void RemapObjectIds( const ON_SimpleArray<ON_UuidPair>& uuid_remap );

private:
  bool m_bValuesSorted;
  ON_Value* FindValueHelper( int, int, bool ) const;
  void CopyHelper( const ON_HistoryRecord&);
};


#endif

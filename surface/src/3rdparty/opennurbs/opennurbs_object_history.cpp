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

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

class ON_Value
{
public:

  // The VALUE_TYPE enum values must never be changed
  // because the values are used to determine the parameter
  // type during file reading.  Additions can be made.
  enum VALUE_TYPE
  {
    no_value_type         =  0,

    bool_value            =  1,
    int_value             =  2,
    double_value          =  3,
    color_value           =  4,
    point_value           =  5,
    vector_value          =  6,
    xform_value           =  7,
    string_value          =  8,
    objref_value          =  9,
    geometry_value        = 10,
    uuid_value            = 11,
    point_on_object_value = 12,
    polyedge_value        = 13,

    // each value type must have a case in ON_Value::CreateValue().

    force_32bit_enum = 0xFFFFFFFF
  };

  static
  ON_Value* CreateValue( int value_type );

  void Report(
    ON_TextLog& text_log
    ) const;

  // The valid id is a nonzero integer the developer
  // assigns to this value.  Developers are responsible
  // for ensuring the 
  int m_value_id;
  
  const VALUE_TYPE m_value_type;

  ON_Value( VALUE_TYPE );
  virtual ~ON_Value();

  virtual ON_Value* Duplicate() const=0;
  virtual int  Count() const=0;
  virtual bool ReadHelper(ON_BinaryArchive& )=0;
  virtual bool WriteHelper(ON_BinaryArchive& ) const=0;
  virtual bool ReportHelper(ON_TextLog& ) const=0;

  virtual int  GetBools( const bool*& ) const;
  virtual int  GetInts( const int*& ) const;
  virtual int  GetColors( const ON_Color*& ) const;
  virtual int  GetDoubles( const double*& ) const;
  virtual int  Get3dPoints( const ON_3dPoint*& ) const;
  virtual int  Get3dVectors( const ON_3dVector*& ) const;
  virtual int  GetXforms( const ON_Xform*& ) const;
  virtual int  GetUuids( const ON_UUID*& ) const;
  virtual int  GetObjRefs( ON_ClassArray<ON_ObjRef>& ) const;
  virtual int  GetGeometryPointers( const ON_Geometry* const*& ) const;
  virtual int  GetStrings( ON_ClassArray<ON_wString>& ) const;
  virtual int  GetPolyEdgePointers( ON_ClassArray<ON_PolyEdgeHistory>& ) const;

private:
  // no implementation
  ON_Value(); 
  ON_Value& operator=(const ON_Value&);
};

ON_Value::ON_Value( ON_Value::VALUE_TYPE value_type )
         : m_value_type(value_type)
{
  m_value_id = -1;
}

ON_Value::~ON_Value()
{}

// base class virtuals do nothing.
int  ON_Value::GetBools( const bool*& ) const {return 0;}
int  ON_Value::GetInts( const int*& ) const {return 0;}
int  ON_Value::GetColors( const ON_Color*& ) const {return 0;}
int  ON_Value::GetDoubles( const double*& ) const {return 0;}
int  ON_Value::Get3dPoints( const ON_3dPoint*& ) const {return 0;}
int  ON_Value::Get3dVectors( const ON_3dVector*& ) const {return 0;}
int  ON_Value::GetXforms( const ON_Xform*& ) const {return 0;}
int  ON_Value::GetUuids( const ON_UUID*& ) const {return 0;}
int  ON_Value::GetObjRefs( ON_ClassArray<ON_ObjRef>& ) const {return 0;}
int  ON_Value::GetGeometryPointers( const ON_Geometry* const*& ) const {return 0;}
int  ON_Value::GetStrings( ON_ClassArray<ON_wString>& ) const {return 0;}
int  ON_Value::GetPolyEdgePointers( ON_ClassArray<ON_PolyEdgeHistory>& ) const {return 0;}

class ON_DummyValue : public ON_Value
{
public:
  ON_DummyValue();
  ~ON_DummyValue();
  ON_Value* Duplicate() const;
  int  Count() const;
  bool ReadHelper(ON_BinaryArchive& );
  bool WriteHelper(ON_BinaryArchive& ) const;
  bool ReportHelper(ON_TextLog& ) const;
};

ON_DummyValue::ON_DummyValue() : ON_Value(ON_Value::no_value_type)
{
}

ON_DummyValue::~ON_DummyValue()
{
}

ON_Value* ON_DummyValue::Duplicate() const {return 0;}
int  ON_DummyValue::Count() const {return 0;}
bool ON_DummyValue::ReadHelper(ON_BinaryArchive& ) {return 0;}
bool ON_DummyValue::WriteHelper(ON_BinaryArchive& ) const {return 0;}
bool ON_DummyValue::ReportHelper(ON_TextLog& ) const {return 0;}
///////////////////////////////////////////////////////////////////////
//
// ON_BoolValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_BoolValue : public ON_Value
{
public:
  ON_BoolValue();
  ~ON_BoolValue();

  ON_SimpleArray<bool> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetBools( const bool*& b ) const;
};

ON_BoolValue::ON_BoolValue() 
             : ON_Value(ON_Value::bool_value) 
{
}

ON_BoolValue::~ON_BoolValue()
{
}

// virtual 
class ON_Value* ON_BoolValue::Duplicate() const
{
  return new ON_BoolValue(*this);
}

// virtual
int ON_BoolValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_BoolValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_BoolValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_BoolValue::ReportHelper(ON_TextLog& text_log ) const
{
  int i, count = m_value.Count();
  text_log.Print("bool value\n");
  text_log.PushIndent();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]?"true":"false");
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_BoolValue::GetBools( const bool*& b ) const
{
  b = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_IntValue saves int values in the ON_HistoryRecord::m_value[] array
//

class ON_IntValue : public ON_Value
{
public:
  ON_IntValue();
  ~ON_IntValue();

  ON_SimpleArray<int> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetInts( const int*& b ) const;
};

ON_IntValue::ON_IntValue() 
             : ON_Value(ON_Value::int_value) 
{
}

ON_IntValue::~ON_IntValue()
{
}

// virtual 
class ON_Value* ON_IntValue::Duplicate() const
{
  return new ON_IntValue(*this);
}

// virtual
int ON_IntValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_IntValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_IntValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_IntValue::ReportHelper(ON_TextLog& text_log ) const
{
  int i, count = m_value.Count();
  text_log.Print("integer value\n");
  text_log.PushIndent();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print("%d",m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_IntValue::GetInts( const int*& b ) const
{
  b = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_DoubleValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_DoubleValue : public ON_Value
{
public:
  ON_DoubleValue();
  ~ON_DoubleValue();

  ON_SimpleArray<double> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetDoubles( const double*& a ) const;
};

ON_DoubleValue::ON_DoubleValue() 
             : ON_Value(ON_Value::double_value) 
{
}

ON_DoubleValue::~ON_DoubleValue()
{
}

// virtual 
class ON_Value* ON_DoubleValue::Duplicate() const
{
  return new ON_DoubleValue(*this);
}

// virtual
int ON_DoubleValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_DoubleValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_DoubleValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_DoubleValue::ReportHelper(ON_TextLog& text_log ) const
{
  int i, count = m_value.Count();
  text_log.Print("number value\n");
  text_log.PushIndent();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_DoubleValue::GetDoubles( const double*& a ) const
{
  a = m_value.Array();
  return m_value.Count();
}

///////////////////////////////////////////////////////////////////////
//
// ON_PointValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_PointValue : public ON_Value
{
public:
  ON_PointValue();
  ~ON_PointValue();

  ON_SimpleArray<ON_3dPoint> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int Get3dPoints( const ON_3dPoint*& a ) const;
};

ON_PointValue::ON_PointValue() 
             : ON_Value(ON_Value::point_value) 
{
}

ON_PointValue::~ON_PointValue()
{
}

// virtual 
class ON_Value* ON_PointValue::Duplicate() const
{
  return new ON_PointValue(*this);
}

// virtual
int ON_PointValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_PointValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_PointValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_PointValue::ReportHelper(ON_TextLog& text_log ) const
{
  int i, count = m_value.Count();
  text_log.Print("point value\n");
  text_log.PushIndent();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_PointValue::Get3dPoints( const ON_3dPoint*& a ) const
{
  a = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_VectorValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_VectorValue : public ON_Value
{
public:
  ON_VectorValue();
  ~ON_VectorValue();

  ON_SimpleArray<ON_3dVector> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int Get3dVectors( const ON_3dVector*& a ) const;
};

ON_VectorValue::ON_VectorValue() 
             : ON_Value(ON_Value::vector_value) 
{
}

ON_VectorValue::~ON_VectorValue()
{
}

// virtual 
class ON_Value* ON_VectorValue::Duplicate() const
{
  return new ON_VectorValue(*this);
}

// virtual
int ON_VectorValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_VectorValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_VectorValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_VectorValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("vector value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_VectorValue::Get3dVectors( const ON_3dVector*& a ) const
{
  a = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_XformValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_XformValue : public ON_Value
{
public:
  ON_XformValue();
  ~ON_XformValue();

  ON_SimpleArray<ON_Xform> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetXforms( const ON_Xform*& a ) const;
};

ON_XformValue::ON_XformValue() 
             : ON_Value(ON_Value::xform_value) 
{
}

ON_XformValue::~ON_XformValue()
{
}

// virtual 
class ON_Value* ON_XformValue::Duplicate() const
{
  return new ON_XformValue(*this);
}

// virtual
int ON_XformValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_XformValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_XformValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_XformValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("xform value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_XformValue::GetXforms( const ON_Xform*& a ) const
{
  a = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_ColorValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_ColorValue : public ON_Value
{
public:
  ON_ColorValue();
  ~ON_ColorValue();

  ON_SimpleArray<ON_Color> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetColors( const ON_Color*& a ) const;
};

ON_ColorValue::ON_ColorValue() 
             : ON_Value(ON_Value::color_value) 
{
}

ON_ColorValue::~ON_ColorValue()
{
}

// virtual 
class ON_Value* ON_ColorValue::Duplicate() const
{
  return new ON_ColorValue(*this);
}

// virtual
int ON_ColorValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_ColorValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_ColorValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_ColorValue::ReportHelper(ON_TextLog& text_log ) const
{
  ON_Color c;
  text_log.Print("color value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    c = m_value[i];
    text_log.Print("rbg(%d,%d,%d)",c.Red(),c.Green(),c.Blue());
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_ColorValue::GetColors( const ON_Color*& a ) const
{
  a = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_UuidValue saves bool values in the ON_HistoryRecord::m_value[] array
//

class ON_UuidValue : public ON_Value
{
public:
  ON_UuidValue();
  ~ON_UuidValue();

  ON_SimpleArray<ON_UUID> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetUuids( const ON_UUID*& a ) const;
};

ON_UuidValue::ON_UuidValue() 
             : ON_Value(ON_Value::uuid_value) 
{
}

ON_UuidValue::~ON_UuidValue()
{
}

// virtual 
class ON_Value* ON_UuidValue::Duplicate() const
{
  return new ON_UuidValue(*this);
}

// virtual
int ON_UuidValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_UuidValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_UuidValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_UuidValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("uuid value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_UuidValue::GetUuids( const ON_UUID*& a ) const
{
  a = m_value.Array();
  return m_value.Count();
}

///////////////////////////////////////////////////////////////////////
//
// ON_StringValue saves string values in the ON_HistoryRecord::m_value[] array
//

class ON_StringValue : public ON_Value
{
public:
  ON_StringValue();
  ~ON_StringValue();

  ON_ClassArray<ON_wString> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetStrings( ON_ClassArray<ON_wString>& s ) const;
};

ON_StringValue::ON_StringValue() 
             : ON_Value(ON_Value::string_value) 
{
}

ON_StringValue::~ON_StringValue()
{
}

// virtual 
class ON_Value* ON_StringValue::Duplicate() const
{
  return new ON_StringValue(*this);
}

// virtual
int ON_StringValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_StringValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_StringValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_StringValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("string value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print(m_value[i]);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_StringValue::GetStrings( ON_ClassArray<ON_wString>& s ) const
{
  s = m_value;
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_ObjRefValue saves objref values in the ON_HistoryRecord::m_value[] array
//

class ON_ObjRefValue : public ON_Value
{
public:
  ON_ObjRefValue();
  ~ON_ObjRefValue();

  ON_ClassArray<ON_ObjRef> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetObjRefs( ON_ClassArray<ON_ObjRef>& oref ) const;
};

ON_ObjRefValue::ON_ObjRefValue() 
             : ON_Value(ON_Value::objref_value) 
{
}

ON_ObjRefValue::~ON_ObjRefValue()
{
}

// virtual 
class ON_Value* ON_ObjRefValue::Duplicate() const
{
  return new ON_ObjRefValue(*this);
}

// virtual
int ON_ObjRefValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_ObjRefValue::ReadHelper(ON_BinaryArchive& archive )
{
  return archive.ReadArray(m_value);
}

// virtual 
bool ON_ObjRefValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  return archive.WriteArray(m_value);
}

// virtual 
bool ON_ObjRefValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("objref value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    text_log.Print("object id: ");
    text_log.Print(m_value[i].m_uuid);
    text_log.Print("\n");
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_ObjRefValue::GetObjRefs( ON_ClassArray<ON_ObjRef>& s ) const
{
  s = m_value;
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_GeometryValue saves geometry values in the ON_HistoryRecord::m_value[] array
//

class ON_GeometryValue : public ON_Value
{
public:
  ON_GeometryValue();
  ~ON_GeometryValue();
  ON_GeometryValue(const ON_GeometryValue& src);
  ON_GeometryValue& operator=(const ON_GeometryValue& src);

  ON_SimpleArray<ON_Geometry*> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int GetGeometryPointers( const ON_Geometry* const*& ) const;
};

ON_GeometryValue::ON_GeometryValue() 
             : ON_Value(ON_Value::geometry_value) 
{
}

ON_GeometryValue::~ON_GeometryValue()
{
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Geometry* p = m_value[i];
    m_value[i] = 0;
    if (p)
    {
      delete p;
    }
  }
}

ON_GeometryValue::ON_GeometryValue(const ON_GeometryValue& src) : ON_Value(src)
{
  *this = src;
}

ON_GeometryValue& ON_GeometryValue::operator=(const ON_GeometryValue& src)
{
  if ( this != &src )
  {
    int i, count = m_value.Count();
    for ( i = 0; i < count; i++ )
    {
      ON_Geometry* p = m_value[i];
      m_value[i] = 0;
      if (p)
      {
        delete p;
      }
    }
    m_value.Destroy();

    m_value_id = src.m_value_id;

    count = src.m_value.Count();
    m_value.Reserve(count);
    for ( i = 0; i < count; i++ )
    {
      const ON_Geometry* src_ptr = src.m_value[i];
      if ( !src_ptr )
        continue;
      ON_Geometry* ptr = src_ptr->Duplicate();
      if ( ptr )
        m_value.Append(ptr);
    }
  }
  return *this;
}

// virtual 
class ON_Value* ON_GeometryValue::Duplicate() const
{
  return new ON_GeometryValue(*this);
}

// virtual
int ON_GeometryValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_GeometryValue::ReadHelper(ON_BinaryArchive& archive )
{
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    delete m_value[i];
  }
  m_value.SetCount(0);

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    rc = archive.ReadInt(&count);
    if (!rc) break;
    m_value.Reserve(count);
    
    for( i = 0; i < count && rc; i++ )
    {
      ON_Object* p=0;
      rc = archive.ReadObject(&p) > 0;
      if (rc)
      {
        ON_Geometry* g = ON_Geometry::Cast(p);
        if (g)
        {
          p = 0;
          m_value.Append(g);
        }
      }
      if ( p )
        delete p;
    }
    if (!rc) break;

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}

// virtual 
bool ON_GeometryValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;

  for(;;)
  {
    rc = archive.WriteInt(m_value.Count());
    if (!rc) break;
    
    int i, count = m_value.Count();
    for( i = 0; i < count && rc; i++ )
    {
      rc = archive.WriteObject(m_value[i]);
    }
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

// virtual 
bool ON_GeometryValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("geometry value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    const ON_Geometry* p = m_value[i];
    if ( p )
      p->Dump(text_log);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_GeometryValue::GetGeometryPointers( const ON_Geometry* const*&a ) const
{
  a = m_value.Array();
  return m_value.Count();
}


///////////////////////////////////////////////////////////////////////
//
// ON_PolyEdgeHistoryValue saves geometry values in the ON_HistoryRecord::m_value[] array
//

class ON_PolyEdgeHistoryValue : public ON_Value
{
public:
  ON_PolyEdgeHistoryValue();
  ~ON_PolyEdgeHistoryValue();

  ON_ClassArray<ON_PolyEdgeHistory> m_value;

  // virtual 
  class ON_Value* Duplicate() const;

  // virtual
  int Count() const;

  // virtual 
  bool ReadHelper(ON_BinaryArchive& archive );

  // virtual 
  bool WriteHelper(ON_BinaryArchive& archive ) const;

  // virtual 
  bool ReportHelper(ON_TextLog& text_log ) const;

  // virtual 
  int  GetPolyEdgePointers( ON_ClassArray<ON_PolyEdgeHistory>& ) const;
};

ON_PolyEdgeHistoryValue::ON_PolyEdgeHistoryValue() 
             : ON_Value(ON_Value::polyedge_value) 
{
}

ON_PolyEdgeHistoryValue::~ON_PolyEdgeHistoryValue()
{
  m_value.Destroy();
}

//ON_PolyEdgeHistoryValue::ON_PolyEdgeHistoryValue(const ON_PolyEdgeHistoryValue& src) : ON_Value(src)
//{
//  *this = src;
//}
//
//ON_PolyEdgeHistoryValue& ON_PolyEdgeHistoryValue::operator=(const ON_PolyEdgeHistoryValue& src)
//{
//  if ( this != &src )
//  {
//    int i, count = m_value.Count();
//    for ( i = 0; i < count; i++ )
//    {
//      ON_Geometry* p = m_value[i];
//      m_value[i] = 0;
//      if (p)
//      {
//        delete p;
//      }
//    }
//    m_value.Destroy();
//
//    m_value_id = src.m_value_id;
//
//    count = src.m_value.Count();
//    m_value.Reserve(count);
//    for ( i = 0; i < count; i++ )
//    {
//      const ON_Geometry* src_ptr = src.m_value[i];
//      if ( !src_ptr )
//        continue;
//      ON_Geometry* ptr = src_ptr->Duplicate();
//      if ( ptr )
//        m_value.Append(ptr);
//    }
//  }
//  return *this;
//}

// virtual 
class ON_Value* ON_PolyEdgeHistoryValue::Duplicate() const
{
  return new ON_PolyEdgeHistoryValue(*this);
}

// virtual
int ON_PolyEdgeHistoryValue::Count() const 
{
  return m_value.Count();
}

// virtual 
bool ON_PolyEdgeHistoryValue::ReadHelper(ON_BinaryArchive& archive )
{
  m_value.Destroy();

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    int count = 0;
    rc = archive.ReadInt(&count);
    if (!rc) break;
    m_value.Reserve(count);
    
    for( int i = 0; i < count && rc; i++ )
    {
      if ( !m_value.AppendNew().Read(archive) )
      {
        m_value.Destroy();
        rc = false;
        break;
      }
    }
    if (!rc) break;

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}

// virtual 
bool ON_PolyEdgeHistoryValue::WriteHelper(ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;

  for(;;)
  {
    rc = archive.WriteInt(m_value.Count());
    if (!rc) break;
    
    int i, count = m_value.Count();
    for( i = 0; i < count && rc; i++ )
    {
      rc = m_value[i].Write(archive);
    }
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

// virtual 
bool ON_PolyEdgeHistoryValue::ReportHelper(ON_TextLog& text_log ) const
{
  text_log.Print("polyedge value\n");
  text_log.PushIndent();
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    m_value[i].Dump(text_log);
  }
  text_log.PopIndent();
  return true;
}

// virtual 
int ON_PolyEdgeHistoryValue::GetPolyEdgePointers( ON_ClassArray<ON_PolyEdgeHistory>& a ) const
{
  a = m_value;
  return m_value.Count();
}

///////////////////////////////////////////////////////////////////////
//

// static
ON_Value* ON_Value::CreateValue( int value_type )
{
  ON_Value* value = 0;
  switch((unsigned int)value_type)
  {
  case no_value_type:
    break;
  case bool_value:
    value = new ON_BoolValue();
    break;
  case int_value:
    value = new ON_IntValue();
    break;
  case double_value:
    value = new ON_DoubleValue();
    break;
  case color_value:
    value = new ON_ColorValue();
    break;
  case point_value:
    value = new ON_PointValue();
    break;
  case vector_value:
    value = new ON_VectorValue();
    break;
  case xform_value:
    value = new ON_XformValue();
    break;
  case string_value:
    value = new ON_StringValue();
    break;
  case objref_value:
    value = new ON_ObjRefValue();
    break;
  case geometry_value:
    value = new ON_PolyEdgeHistoryValue();
    break;
  case uuid_value:
    value = new ON_UuidValue();
    break;
  case point_on_object_value:
    //value = new ON_PointOnObjectValue();
    break;
  case polyedge_value:
    value = new ON_PolyEdgeHistoryValue();
    break;
  case force_32bit_enum:
    break;
  default:
    break;
  }
  return value;
}

///////////////////////////////////////////////////////////////////////
//
// ON_HistoryRecord implementation
//

ON_OBJECT_IMPLEMENT(ON_HistoryRecord,ON_Object,"ECD0FD2F-2088-49dc-9641-9CF7A28FFA6B");

ON_HistoryRecord::ON_HistoryRecord() 
                 : m_antecedents(2),
                   m_descendants(1)                   
{
  m_command_id    = ON_nil_uuid;
  m_version       = 0;
  m_record_type   = history_parameters;
  m_record_id     = ON_nil_uuid;
  m_bValuesSorted = true;
}

ON_HistoryRecord::~ON_HistoryRecord()
{
  int i, count = m_value.Count();
  m_value.SetCount(0);
  for ( i = 0; i < count; i++ )
  {
    ON_Value* v = m_value[i];
    if ( v )
      delete v;
  }
}

void  ON_HistoryRecord::CopyHelper(const ON_HistoryRecord& src)
{
  // input value of this->m_value[] is known to be empty
  m_command_id   = src.m_command_id;
  m_version      = src.m_version;
  m_record_type  = src.m_record_type;
  m_record_id    = src.m_record_id;
  m_descendants  = src.m_descendants;
  m_antecedents  = src.m_antecedents;
  m_bValuesSorted = true;

  int i, count = src.m_value.Count();
  m_value.SetCapacity(count);
  const ON_Value* prev_v = 0;
  for ( i = 0; i < count; i++ )
  {
    const ON_Value* src_v = src.m_value[i];
    if ( src_v )
    {
      ON_Value* v = src_v->Duplicate();
      if ( v )
      {
        m_value.Append(v);
        if ( m_bValuesSorted && prev_v && prev_v->m_value_id > v->m_value_id )
          m_bValuesSorted = false;
        prev_v = v;
      }
    }
  }
}

ON_HistoryRecord::ON_HistoryRecord(const ON_HistoryRecord& src) : ON_Object(src)
{
  CopyHelper(src);
}

ON_HistoryRecord& ON_HistoryRecord::operator=(const ON_HistoryRecord& src)
{
  if ( this != &src )
  {
    Destroy();
    ON_Object::operator =(src);
    CopyHelper(src);
  }
  return *this;
}

ON_BOOL32 ON_HistoryRecord::IsValid( ON_TextLog* ) const
{
  return true;
}

void ON_HistoryRecord::Destroy()
{
  int i, count = m_value.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Value* v = m_value[i];
    m_value[i] = 0;
    if (v)
      delete v;
  }
  m_value.SetCount(0);
  m_record_id = ON_nil_uuid;
  m_record_type = ON_HistoryRecord::history_parameters;
  m_version = 0;
  m_command_id = ON_nil_uuid;
  m_antecedents.Empty();
  m_descendants.Empty();
}

ON_HistoryRecord::RECORD_TYPE ON_HistoryRecord::RecordType(int i)
{
  RECORD_TYPE rc = ( ON_HistoryRecord::feature_parameters == i )
                 ? ON_HistoryRecord::feature_parameters
                 : ON_HistoryRecord::history_parameters;
  return rc;
}

bool ON_HistoryRecord::SetBoolValue( int value_id, bool b)
{
  return ( 1 == SetBoolValues(value_id, 1, &b) );
}

bool ON_HistoryRecord::SetIntValue( int value_id, int i)
{
  return ( 1 == SetIntValues(value_id, 1, &i) );
}

bool ON_HistoryRecord::SetDoubleValue( int value_id, double x)
{
  return ( 1 == SetDoubleValues(value_id, 1, &x) );
}

bool ON_HistoryRecord::SetPointValue( int value_id, ON_3dPoint p)
{
  return ( 1 == SetPointValues(value_id, 1, &p) );
}

bool ON_HistoryRecord::SetVectorValue( int value_id, ON_3dVector v)
{
  return ( 1 == SetVectorValues(value_id, 1, &v) );
}

bool ON_HistoryRecord::SetXformValue( int value_id, ON_Xform xform)
{
  return ( 1 == SetXformValues(value_id, 1, &xform) );
}

bool ON_HistoryRecord::SetColorValue( int value_id, ON_Color c)
{
  return ( 1 == SetColorValues(value_id, 1, &c) );
}

bool ON_HistoryRecord::SetObjRefValue( int value_id, const ON_ObjRef& oref)
{
  return ( 1 == SetObjRefValues(value_id, 1, &oref) );
}

bool ON_HistoryRecord::SetPointOnObjectValue( int value_id, const ON_ObjRef& oref, ON_3dPoint point )
{
  ON_ObjRef poo = oref;
  poo.m_point = point;
  return SetObjRefValue(value_id,poo);
}

bool ON_HistoryRecord::GetPointOnObjectValue( int value_id, ON_ObjRef& oref ) const
{
  bool rc = GetObjRefValue(value_id,oref);
  return (rc && oref.m_point.IsValid());
}

bool ON_HistoryRecord::SetGeometryValue( int value_id, ON_Geometry* g)
{
  ON_SimpleArray<ON_Geometry*> a(1);
  a.Append(g);
  return ( 1 == SetGeometryValues(value_id, a) );
}

bool ON_HistoryRecord::SetPolyEdgeValue( int value_id, const ON_PolyEdgeHistory& polyedge )
{
  return ( 1 == SetPolyEdgeValues(value_id, 1, &polyedge) );
}

bool ON_HistoryRecord::SetUuidValue( int value_id, ON_UUID uuid )
{
  return ( 1 == SetUuidValues(value_id, 1, &uuid) );
}

static
int CompareValueIdHelper(const ON_Value* a, const ON_Value* b )
{
  if (!a)
  {
    return b ? -1 : 0;
  }
  if (!b)
  {
    return 1;
  }
  return (a->m_value_id - b->m_value_id);
}

static int CompareValueId( ON_Value * const * a, ON_Value * const * b )
{
  // handle NULLs in case somebody messes up the m_value[] array.
  if ( !a )
  {
    return b ? -1 : 0;
  }
  if (!b)
    return 1;

  return CompareValueIdHelper(*a,*b);
}

ON_Value* ON_HistoryRecord::FindValueHelper( int value_id, int value_type, bool bCreateOne ) const
{
  ON_HistoryRecord* vp = const_cast<ON_HistoryRecord*>(this);
  if ( m_value.Count() > 0 )
  {
    if ( !m_bValuesSorted )
    {
      vp->m_value.QuickSort(CompareValueId);
      vp->m_bValuesSorted = true;
    }

    ON_DummyValue dummy_value;
    dummy_value.m_value_id = value_id;
    ON_Value* p = &dummy_value;
    int i = m_value.BinarySearch(&p,CompareValueId);

    if ( i >= 0 )
    {
      // m_value[i]->m_value_id == value_id

      if ( value_type == ((int)m_value[i]->m_value_type) )
      {
        // type matches
        return m_value[i];
      }

      if ( bCreateOne )
      {
        // type does not match - replace the existing one
        ON_Value* new_value = ON_Value::CreateValue(value_type);
        if ( new_value )
        {
          new_value->m_value_id = value_id;
          delete m_value[i];
          vp->m_value[i] = new_value;
          return new_value;
        }
      }
    }
    else if ( bCreateOne )
    {
      // no value in m_value[] array with a matching value_id
      ON_Value* new_value = ON_Value::CreateValue(value_type);
      if ( new_value )
      {
        new_value->m_value_id = value_id;
        if ( m_bValuesSorted && (*m_value.Last())->m_value_id > value_id )
          vp->m_bValuesSorted = false;
        vp->m_value.Append(new_value);
        return new_value;
      }
    }
  }
  else if ( bCreateOne )
  {
    ON_Value* new_value = ON_Value::CreateValue(value_type);
    if ( new_value )
    {
      new_value->m_value_id = value_id;
      vp->m_bValuesSorted = true;
      vp->m_value.Append(new_value);
      return new_value;
    }
  }
  return 0;
}


bool ON_HistoryRecord::SetBoolValues( int value_id, int count, const bool* b)
{
  ON_BoolValue* v = static_cast<ON_BoolValue*>(FindValueHelper(value_id,ON_Value::bool_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,b);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetIntValues( int value_id, int count, const int* i)
{
  ON_IntValue* v = static_cast<ON_IntValue*>(FindValueHelper(value_id,ON_Value::int_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,i);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetDoubleValues( int value_id, int count, const double* x)
{
  ON_DoubleValue* v = static_cast<ON_DoubleValue*>(FindValueHelper(value_id,ON_Value::double_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,x);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetPointValues( int value_id, int count, const ON_3dPoint* P)
{
  ON_PointValue* v = static_cast<ON_PointValue*>(FindValueHelper(value_id,ON_Value::point_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,P);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetVectorValues( int value_id, int count, const ON_3dVector* V)
{
  ON_VectorValue* v = static_cast<ON_VectorValue*>(FindValueHelper(value_id,ON_Value::vector_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,V);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetXformValues( int value_id, int count, const ON_Xform* xform)
{
  ON_XformValue* v = static_cast<ON_XformValue*>(FindValueHelper(value_id,ON_Value::xform_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,xform);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetColorValues( int value_id, int count, const ON_Color* c)
{
  ON_ColorValue* v = static_cast<ON_ColorValue*>(FindValueHelper(value_id,ON_Value::color_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,c);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetUuidValues( int value_id, int count, const ON_UUID* u )
{
  ON_UuidValue* v = static_cast<ON_UuidValue*>(FindValueHelper(value_id,ON_Value::uuid_value,true));
  if ( v )
  {
    v->m_value.SetCount(0);
    v->m_value.SetCapacity(count);
    v->m_value.Append(count,u);
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetStringValues( int value_id, int count, const wchar_t* const* s )
{
  ON_StringValue* v = static_cast<ON_StringValue*>(FindValueHelper(value_id,ON_Value::string_value,true));
  if ( v )
  {
    v->m_value.Destroy();
    v->m_value.Reserve(count);
    int i;
    for( i = 0; i < count; i++ )
    {
      v->m_value.AppendNew() = s[i];
    }
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetStringValues( int value_id, const ON_ClassArray<ON_wString>& s )
{
  ON_StringValue* v = static_cast<ON_StringValue*>(FindValueHelper(value_id,ON_Value::string_value,true));
  if ( v )
  {
    v->m_value = s;
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetStringValue( int value_id, const wchar_t* s )
{
  ON_StringValue* v = static_cast<ON_StringValue*>(FindValueHelper(value_id,ON_Value::string_value,true));
  if ( v )
  {
    v->m_value.Destroy();
    v->m_value.AppendNew() = s;
  }
  return (0 != v);
}


bool ON_HistoryRecord::SetObjRefValues( int value_id, int count, const ON_ObjRef* oref)
{
  ON_ObjRefValue* v = static_cast<ON_ObjRefValue*>(FindValueHelper(value_id,ON_Value::objref_value,true));
  if ( v )
  {
    v->m_value.Destroy();
    v->m_value.Reserve(count);
    int i;
    for ( i = 0; i < count; i++ )
    {
      // The call to DecrementProxyReferenceCount() is critical.
      // It makes sure there are no active runtime pointers 
      // saved in the history record.  If this call is not here,
      // you will eventually crash and history update will never
      // work right even when it doesn't crash.
      ON_ObjRef& vor = v->m_value.AppendNew();
      vor = oref[i];
      vor.DecrementProxyReferenceCount();
      // Feb 12 2010 - Fixing bug in ExtrudeCrv history
      //  and probably lots of other subtle history bugs.
      //  History must lookup by UUID and not by runtime serial number.
      vor.m_runtime_sn = 0; 
      ON_UUID object_id = v->m_value[i].m_uuid;
      if ( !ON_UuidIsNil(object_id) )
      {
        m_antecedents.AddUuid(object_id);
      }
    }
  }
  return (0 != v);
}


bool ON_HistoryRecord::SetGeometryValues( int value_id, const ON_SimpleArray<ON_Geometry*> a)
{
  ON_GeometryValue* v = static_cast<ON_GeometryValue*>(FindValueHelper(value_id,ON_Value::geometry_value,true));
  if ( v )
  {
    v->m_value = a;
  }
  return (0 != v);
}

bool ON_HistoryRecord::SetPolyEdgeValues( int value_id,  int count, const ON_PolyEdgeHistory* a )
{
  ON_PolyEdgeHistoryValue* v = static_cast<ON_PolyEdgeHistoryValue*>(FindValueHelper(value_id,ON_Value::polyedge_value,true));
  if ( v )
  {
    v->m_value.Destroy();
    v->m_value.Append(count,a);

    for ( int i = 0; i < count; i++ )
    {
      const ON_PolyEdgeHistory& pe_history = a[i];
      for ( int j = 0; j < pe_history.m_segment.Count(); j++ )
      {
        const ON_CurveProxyHistory& segment = pe_history.m_segment[j];
        m_antecedents.AddUuid(segment.m_curve_ref.m_uuid);
      }
    }
  }
  return (0 != v);
}

bool ON_HistoryRecord::GetBoolValue( int value_id, bool* b ) const
{
  bool rc = false;
  const ON_BoolValue* v = static_cast<ON_BoolValue*>(FindValueHelper(value_id,ON_Value::bool_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    *b = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetIntValue( int value_id, int* i ) const
{
  bool rc = false;
  const ON_IntValue* v = static_cast<ON_IntValue*>(FindValueHelper(value_id,ON_Value::int_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    *i = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetDoubleValue( int value_id, double* number ) const
{
  bool rc = false;
  const ON_DoubleValue* v = static_cast<ON_DoubleValue*>(FindValueHelper(value_id,ON_Value::double_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    *number = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetPointValue( int value_id, ON_3dPoint& point ) const
{
  bool rc = false;
  const ON_PointValue* v = static_cast<ON_PointValue*>(FindValueHelper(value_id,ON_Value::point_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    point = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetVectorValue( int value_id, ON_3dVector& vector ) const
{
  bool rc = false;
  const ON_VectorValue* v = static_cast<ON_VectorValue*>(FindValueHelper(value_id,ON_Value::vector_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    vector = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetXformValue( int value_id, ON_Xform& xform ) const
{
  bool rc = false;
  const ON_XformValue* v = static_cast<ON_XformValue*>(FindValueHelper(value_id,ON_Value::xform_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    xform = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetColorValue( int value_id, ON_Color* color ) const
{
  bool rc = false;
  const ON_ColorValue* v = static_cast<ON_ColorValue*>(FindValueHelper(value_id,ON_Value::color_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    *color = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetObjRefValue( int value_id, ON_ObjRef& oref ) const
{
  bool rc = false;
  const ON_ObjRefValue* v = static_cast<ON_ObjRefValue*>(FindValueHelper(value_id,ON_Value::objref_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    oref = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetStringValue( int value_id, ON_wString& str ) const
{
  bool rc = false;
  ON_StringValue* v = static_cast<ON_StringValue*>(FindValueHelper(value_id,ON_Value::string_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    str = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetGeometryValue( int value_id, const ON_Geometry*& g ) const
{
  bool rc = false;
  g = 0;
  const ON_GeometryValue* v = static_cast<ON_GeometryValue*>(FindValueHelper(value_id,ON_Value::geometry_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    g = v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetPolyEdgeValue( int value_id, const ON_PolyEdgeHistory*& polyedge ) const
{
  bool rc = false;
  polyedge = 0;
  const ON_PolyEdgeHistoryValue* v = static_cast<ON_PolyEdgeHistoryValue*>(FindValueHelper(value_id,ON_Value::polyedge_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    polyedge = &v->m_value[0];
    rc = true;
  }
  return rc;
}

bool ON_HistoryRecord::GetCurveValue( int value_id, const ON_Curve*& c ) const
{
  c = 0;
  const ON_Geometry* g = 0;
  if (GetGeometryValue( value_id, g ))
  {
    c = ON_Curve::Cast(g);
  }
  return (0 != c);
}

bool ON_HistoryRecord::GetSurfaceValue( int value_id, const ON_Surface*& s ) const
{
  s = 0;
  const ON_Geometry* g = 0;
  if (GetGeometryValue( value_id, g ))
  {
    s = ON_Surface::Cast(g);
  }
  return (0 != s);
}

bool ON_HistoryRecord::GetBrepValue( int value_id, const ON_Brep*& b ) const
{
  b = 0;
  const ON_Geometry* g = 0;
  if (GetGeometryValue( value_id, g ))
  {
    b = ON_Brep::Cast(g);
  }
  return (0 != b);
}

bool ON_HistoryRecord::GetMeshValue( int value_id, const ON_Mesh*& m ) const
{
  m = 0;
  const ON_Geometry* g = 0;
  if (GetGeometryValue( value_id, g ))
  {
    m = ON_Mesh::Cast(g);
  }
  return (0 != m);
}

bool ON_HistoryRecord::GetUuidValue( int value_id, ON_UUID* uuid ) const
{
  bool rc = false;
  const ON_UuidValue* v = static_cast<ON_UuidValue*>(FindValueHelper(value_id,ON_Value::uuid_value,0));
  if ( v && 1 == v->m_value.Count())
  {
    *uuid = v->m_value[0];
    rc = true;
  }
  return rc;
}

int ON_HistoryRecord::GetBoolValues( int value_id, ON_SimpleArray<bool>& a ) const
{
  a.SetCount(0);
  const ON_BoolValue* v = static_cast<ON_BoolValue*>(FindValueHelper(value_id,ON_Value::bool_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}


int ON_HistoryRecord::GetStringValues( int value_id, ON_ClassArray<ON_wString>& a ) const
{
  a.SetCount(0);
  const ON_StringValue* v = static_cast<ON_StringValue*>(FindValueHelper(value_id,ON_Value::string_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}


int ON_HistoryRecord::GetIntValues( int value_id, ON_SimpleArray<int>& a ) const
{
  a.SetCount(0);
  const ON_IntValue* v = static_cast<ON_IntValue*>(FindValueHelper(value_id,ON_Value::int_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetDoubleValues( int value_id, ON_SimpleArray<double>& a) const
{
  a.SetCount(0);
  const ON_DoubleValue* v = static_cast<ON_DoubleValue*>(FindValueHelper(value_id,ON_Value::double_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetPointValues( int value_id, ON_SimpleArray<ON_3dPoint>& a ) const
{
  a.SetCount(0);
  const ON_PointValue* v = static_cast<ON_PointValue*>(FindValueHelper(value_id,ON_Value::point_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetVectorValues( int value_id, ON_SimpleArray<ON_3dVector>& a ) const
{
  a.SetCount(0);
  const ON_VectorValue* v = static_cast<ON_VectorValue*>(FindValueHelper(value_id,ON_Value::vector_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetXformValues( int value_id, ON_SimpleArray<ON_Xform>& a ) const
{
  a.SetCount(0);
  const ON_XformValue* v = static_cast<ON_XformValue*>(FindValueHelper(value_id,ON_Value::xform_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetColorValues( int value_id, ON_SimpleArray<ON_Color>& a ) const
{
  a.SetCount(0);
  const ON_ColorValue* v = static_cast<ON_ColorValue*>(FindValueHelper(value_id,ON_Value::color_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetObjRefValues( int value_id, ON_ClassArray<ON_ObjRef>& a ) const
{
  a.SetCount(0);
  const ON_ObjRefValue* v = static_cast<ON_ObjRefValue*>(FindValueHelper(value_id,ON_Value::objref_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}

int ON_HistoryRecord::GetGeometryValues( int value_id, ON_SimpleArray<const ON_Geometry*>& a) const
{
  a.SetCount(0);
  const ON_GeometryValue* v = static_cast<ON_GeometryValue*>(FindValueHelper(value_id,ON_Value::geometry_value,0));
  if ( v )
  {
    int i, count = v->m_value.Count();
    a.Reserve(count);
    for ( i = 0; i < count; i++ )
      a.Append(v->m_value[i]);
  }
  return a.Count();
}

int ON_HistoryRecord::GetPolyEdgeValues( int value_id, ON_SimpleArray<const ON_PolyEdgeHistory*>& a) const
{
  a.SetCount(0);
  const ON_PolyEdgeHistoryValue* v = static_cast<ON_PolyEdgeHistoryValue*>(FindValueHelper(value_id,ON_Value::polyedge_value,0));
  if ( v )
  {
    int i, count = v->m_value.Count();
    a.Reserve(count);
    for ( i = 0; i < count; i++ )
      a.Append(&v->m_value[i]);
  }
  return a.Count();
}

int ON_HistoryRecord::GetUuidValues( int value_id, ON_SimpleArray<ON_UUID>& a) const
{
  a.SetCount(0);
  const ON_UuidValue* v = static_cast<ON_UuidValue*>(FindValueHelper(value_id,ON_Value::uuid_value,0));
  if ( v )
  {
    a = v->m_value;
  }
  return a.Count();
}


bool ON_HistoryRecord::IsAntecedent( ON_UUID object_uuid ) const
{
  return m_antecedents.FindUuid(object_uuid);
}

int ON_HistoryRecord::ValueReport( ON_TextLog& text_log ) const
{
  int value_count = 0;
  int i, vi, count = m_value.Count();

  // list values
  ON_SimpleArray<int> vi_list(count);
  vi_list.SetCount(count);
  vi_list.Zero();

  m_value.Sort( ON::quick_sort, vi_list.Array(), CompareValueId );

  for ( i = 0; i < count; i++ )
  {
    vi = vi_list[i];
    const ON_Value* v = m_value[vi];
    if (!v)
      continue;
    text_log.Print("Value ID %d:\n",v->m_value_id);
    text_log.PushIndent();
    m_value[i]->ReportHelper(text_log);
    text_log.PopIndent();
    value_count++;
  }
  return value_count;
}

void ON_HistoryRecord::Dump( ON_TextLog& text_log ) const
{
  int i, count;
  ON_SimpleArray<ON_UUID> uuid_list;

  text_log.Print("Command ID: ");
  text_log.Print(m_command_id);
  text_log.Print("\n");

  text_log.Print("Version %d\n",m_version);

  text_log.Print("Record ID: ");
  text_log.Print(m_record_id);
  text_log.Print("\n");

  text_log.Print("Record type: %s\n",
                 (m_record_type == ON_HistoryRecord::feature_parameters) 
                 ? "feature parameters" : "history parameters");

  // list antededents
  uuid_list.SetCount(0);
  m_antecedents.GetUuids(uuid_list);
  count = uuid_list.Count();
  if ( count <= 0 )
  {
    text_log.Print("No antededents.\n");
  }
  else
  {
    text_log.Print("Antededent ID:\n");
    text_log.PushIndent();
    for ( i = 0; i < count; i++ )
    {
      text_log.Print(uuid_list[i]);
      text_log.Print("\n");
    }
    text_log.PopIndent();
  }

  // list descendants
  uuid_list.SetCount(0);
  m_descendants.GetUuids(uuid_list);
  count = uuid_list.Count();
  if ( count <= 0 )
  {
    text_log.Print("No descendants.\n");
  }
  else
  {
    text_log.Print("Descendant ID:\n");
    text_log.PushIndent();
    for ( i = 0; i < count; i++ )
    {
      text_log.Print(uuid_list[i]);
      text_log.Print("\n");
    }
    text_log.PopIndent();
  }

  text_log.Print("Values:\n");
  text_log.PushIndent();
  int value_count = ValueReport(text_log);
  if ( 0 == value_count )
    text_log.Print("none\n");
  text_log.PopIndent();
}

void ON_HistoryRecord::DestroyValue( int value_id )
{
  if ( m_value.Count() > 0 )
  {
    if ( !m_bValuesSorted )
    {
      m_value.QuickSort(CompareValueId);
      m_bValuesSorted = true;
    }
    ON_DummyValue dummy_value;
    dummy_value.m_value_id = value_id;
    ON_Value* p = &dummy_value;
    int i = m_value.BinarySearch(&p,CompareValueId);
    if ( i >= 0 )
    {
      ON_Value* v = m_value[i];
      m_value.Remove();
      delete v;
    }
  }
}

ON_BOOL32 ON_HistoryRecord::Read( ON_BinaryArchive& archive )
{
  Destroy();

  // put entire history record in a chunk
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = archive.ReadUuid(m_record_id);
    if(!rc) break;

    rc = archive.ReadInt(&m_version);
    if(!rc) break;

    rc = archive.ReadUuid(m_command_id);
    if(!rc) break;

    rc = m_descendants.Read(archive);
    if(!rc) break;

    rc = m_antecedents.Read(archive);
    if(!rc) break;

    // all values are in a chunk
    int mjvs=0,mnvs=0;
    rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&mjvs,&mnvs);
    if (rc)
    {
      rc = ( 1 == mjvs );

      int i, count = 0;
      if (rc) 
        rc = archive.ReadInt(&count);

      if (rc)
        m_value.Reserve(count);

      for ( i = 0; i < count && rc; i++ )
      {
        int mjv=0,mnv=0;
        rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&mjv,&mnv);
        if ( !rc)
          break;
        for(;;)
        {
          rc = ( 1 == mjv);
          if (!rc) break;
          int value_type = ON_Value::no_value_type;
          rc = archive.ReadInt(&value_type);
          if (!rc) 
            break;

          int value_id = 0;
          rc = archive.ReadInt(&value_id);
          if (!rc) 
            break;

          ON_Value* value = ON_Value::CreateValue(value_type);
          if ( value )
          {        
            value->m_value_id = value_id;
            rc = value->ReadHelper(archive);
            if (!rc) 
            {
              delete value;
              break;
            }
            m_value.Append(value);
          }

          break;
        }
        if (!archive.EndRead3dmChunk())
          rc = false;
      }

      // end of all values
      if (!archive.EndRead3dmChunk())
        rc = false;
    }

    if ( rc && minor_version >= 1 )
    {
      // 1.1 fields added to opennurbs version 200603200
      int rec_type = ON_HistoryRecord::history_parameters;
      if (rc)
        rc = archive.ReadInt( &rec_type );
      if (rc )
        m_record_type = RecordType(rec_type);

    }

    break;
  }

  // end of entire history record in a chunk
  if (!archive.EndRead3dmChunk())
    rc = false;
  return rc;
}

ON_BOOL32 ON_HistoryRecord::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if (!rc)
    return false;

  for(;;)
  {
    rc = archive.WriteUuid(m_record_id);
    if(!rc) break;

    rc = archive.WriteInt(m_version);
    if(!rc) break;

    rc = archive.WriteUuid(m_command_id);
    if(!rc) break;

    rc = m_descendants.Write(archive);
    if(!rc) break;

    rc = m_antecedents.Write(archive);
    if(!rc) break;


    // wrap all values in a chunk
    rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
    if (rc)
    {
      // value count
      int i, count = m_value.Count();
      rc = archive.WriteInt(count);

      for ( i = 0; i < count && rc; i++ )
      {
        // put individual value in its own chunk
        rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
        if ( !rc)
          break;
        for(;;)
        {
          const ON_Value* value = m_value[i];
          rc = archive.WriteInt(value ? value->m_value_type : ON_Value::no_value_type );
          if (!rc) break;

          rc = archive.WriteInt(value ? value->m_value_id : 0 );
          if (!rc) break;
          
          if ( value && value->m_value_type != ON_Value::no_value_type )
            rc = value->WriteHelper(archive);
          if (!rc) break;

          break;
        }
        // end of individual value chunk
        if (!archive.EndWrite3dmChunk())
          rc = false;
      }

      // end of all values chunk
      if ( !archive.EndWrite3dmChunk() )
        rc = false;

    }

    // 1.1 fields added to opennurbs version 200603200
    if (rc)
      rc = archive.WriteInt( m_record_type );

    break;
  }

  if (!archive.EndWrite3dmChunk())
    rc = false;
  return rc;
}

ON_UUID ON_HistoryRecord::ModelObjectId() const
{
  return m_record_id;
}

void ON_HistoryRecord::RemapObjectIds( const ON_SimpleArray<ON_UuidPair>& id_remap )
{
  if ( id_remap.Count() > 0 )
  {
    int i, j;
    m_antecedents.RemapUuids(id_remap);
    m_descendants.RemapUuids(id_remap);
    for ( i = 0; i < m_value.Count(); i++ )
    {
      ON_Value* v = m_value[i];
      if ( v && ON_Value::objref_value == v->m_value_type )
      {
        ON_ObjRefValue* objrev_v = static_cast<ON_ObjRefValue*>(v);
        for ( j = 0; j < objrev_v->m_value.Count(); j++ )
        {
          objrev_v->m_value[j].RemapObjectId(id_remap);
        }
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////
//
// ON_CurveProxyHistory
//

ON_CurveProxyHistory::ON_CurveProxyHistory()
{
}

ON_CurveProxyHistory::~ON_CurveProxyHistory()
{
}

void ON_CurveProxyHistory::Destroy()
{
  m_curve_ref.Destroy();
  m_bReversed = false;
  m_full_real_curve_domain.Destroy();
  m_sub_real_curve_domain.Destroy();
  m_proxy_curve_domain.Destroy();
}


bool ON_CurveProxyHistory::Write( ON_BinaryArchive& file ) const
{
  if ( !file.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK,1,0) )
    return false;

  bool rc = false;
  for(;;)
  {
    if ( !m_curve_ref.Write(file) )
      break;
    if ( !file.WriteBool(m_bReversed) )
      break;
    if ( !file.WriteInterval(m_full_real_curve_domain) )
      break;
    if ( !file.WriteInterval(m_sub_real_curve_domain) )
      break;
    if ( !file.WriteInterval(m_proxy_curve_domain) )
      break;
    rc = true;
    break;
  }

  if ( !file.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

bool ON_CurveProxyHistory::Read( ON_BinaryArchive& file )
{
  int version_major = 0;
  int version_minor = 0;
  Destroy();

  if ( !file.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK,&version_major,&version_minor) )
    return false;

  bool rc = false;
  for(;;)
  {
    if ( 1 != version_major )
      break;
    if ( !m_curve_ref.Read(file) )
      break;
    if ( !file.ReadBool(&m_bReversed) )
      break;
    if ( !file.ReadInterval(m_full_real_curve_domain) )
      break;
    if ( !file.ReadInterval(m_sub_real_curve_domain) )
      break;
    if ( !file.ReadInterval(m_proxy_curve_domain) )
      break;
    
    rc = true;
    break;
  }

  if ( !file.EndRead3dmChunk() )
    rc = false;

  return rc;
}

void ON_CurveProxyHistory::Dump( ON_TextLog& ) const
{
}

/////////////////////////////////////////////////////////////////////////////////
//
// ON_PolyEdgeHistory
//

ON_PolyEdgeHistory::ON_PolyEdgeHistory()
: m_evaluation_mode(0)
{
}

ON_PolyEdgeHistory::~ON_PolyEdgeHistory()
{
}

void ON_PolyEdgeHistory::Destroy()
{
  m_segment.Destroy();
  m_t.Destroy();
  m_evaluation_mode = 0;
}


bool ON_PolyEdgeHistory::Write( ON_BinaryArchive& file ) const
{
  int i;
  if ( !file.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK,1,0) )
    return false;

  bool rc = false;
  for(;;)
  {
    if ( !file.WriteInt(m_segment.Count()) )
      break;
    for ( i = 0; i < m_segment.Count(); i++ )
    {
      if ( !m_segment[i].Write(file) )
        break;
    }
    if ( i < m_segment.Count() )
      break;
    if ( !file.WriteInt(m_t.Count()) )
      break;
    if ( m_t.Count() > 0 )
    {
      if ( !file.WriteDouble(m_t.Count(),m_t.Array()) )
        break;
    }
    if ( !file.WriteInt(m_evaluation_mode) )
      break;
    rc = true;
    break;
  }

  if ( !file.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

bool ON_PolyEdgeHistory::Read( ON_BinaryArchive& file )
{
  int count, i;
  int version_major = 0;
  int version_minor = 0;

  Destroy();

  if ( !file.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK,&version_major,&version_minor) )
    return false;

  bool rc = false;
  for(;;)
  {
    if ( 1 != version_major )
      break;
    count = 0;
    if ( !file.ReadInt(&count) )
      break;
    m_segment.Reserve(count);
    for ( i = 0; i < count; i++ )
    {
      if ( !m_segment.AppendNew().Read(file) )
        break;
    }
    if ( i < count )
      break;
    count = 0;
    if ( !file.ReadInt(&count) )
      break;
    if ( count > 0 )
    {
      m_t.Reserve(count);
      m_t.SetCount(count);
      if ( !file.ReadDouble(count,m_t.Array()) )
        break;
    }
    if ( !file.ReadInt(&m_evaluation_mode) )
      break;
    rc = true;
    break;
  }

  if ( !file.EndRead3dmChunk() )
    rc = false;
  return rc;
}

void ON_PolyEdgeHistory::Dump( ON_TextLog& ) const
{
}

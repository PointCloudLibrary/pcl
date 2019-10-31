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


#if defined(ON_COMPILER_MSC)
// Disable the MSC /W4 warning C4127: conditional expression is constant
//
// This file has a lot of for( i = 0; true; i < 123 )...
// loops where the "true" generates a 4127 warning.
// This source code has to work on many different
// compilers I do not trust all of them to correctly
// compile for( i = 0; /* empty condition*/; i < 123) ...
#pragma warning( push )
#pragma warning( disable : 4127 )
#endif


#if defined(ON_DLL_EXPORTS)

////////////////////////////////////////////////////////////////////////////////
//
// When openNURBS is used as a Microsoft Windows DLL, it is possible 
// for new/delete to allocate memory in one executable and delete
// it in another.  Because Microsoft Windows has incompatible memory 
// managers in its plethora of C libraries and the choice of which
// C library actually gets used depends on the code generation 
// options you choose,  we get lots of support questions asking
// about hard to trace crashes.
//
// If you are using openNURBS as a Windows DLL, you are sure you know
// what you are doing, and you promise never to ask for support, then
// feel free to delete these overrides.
//
//
#pragma message( " --- OpenNURBS overriding ONX_Model new and delete" )

// ONX_Model_UserData new/delete

void* ONX_Model_UserData::operator new(std::size_t sz)
{
  // ONX_Model_UserData new
  return onmalloc(sz);
}

void ONX_Model_UserData::operator delete(void* p)
{
  // ONX_Model_UserData delete
  onfree(p);
}

void* ONX_Model_UserData::operator new[] (std::size_t sz)
{
  // ONX_Model_UserData array new
  return onmalloc(sz);
}

void ONX_Model_UserData::operator delete[] (void* p)
{
  // ONX_Model_UserData array delete
  onfree(p);
}

void* ONX_Model_UserData::operator new(std::size_t, void* p)
{
  // ONX_Model_UserData placement new
  return p;
}

void ONX_Model_UserData::operator delete(void*, void*)
{
  // ONX_Model_UserData placement delete
  return;
}


// ONX_Model_Object new/delete

void* ONX_Model_Object::operator new(std::size_t sz)
{
  // ONX_Model_Object new
  return onmalloc(sz);
}

void ONX_Model_Object::operator delete(void* p)
{
  // ONX_Model_Object delete
  onfree(p);
}

void* ONX_Model_Object::operator new[] (std::size_t sz)
{
  // ONX_Model_Object array new
  return onmalloc(sz);
}

void ONX_Model_Object::operator delete[] (void* p)
{
  // ONX_Model_Object array delete
  onfree(p);
}

void* ONX_Model_Object::operator new(std::size_t, void* p)
{
  // ONX_Model_Object placement new
  return p;
}

void ONX_Model_Object::operator delete(void*, void*)
{
  // ONX_Model_Object placement delete
  return;
}


// ONX_Model_RenderLight new/delete

void* ONX_Model_RenderLight::operator new(std::size_t sz)
{
  // ONX_Model_RenderLight new
  return onmalloc(sz);
}

void ONX_Model_RenderLight::operator delete(void* p)
{
  // ONX_Model_RenderLight delete
  onfree(p);
}

void* ONX_Model_RenderLight::operator new[] (std::size_t sz)
{
  // ONX_Model_RenderLight array new
  return onmalloc(sz);
}

void ONX_Model_RenderLight::operator delete[] (void* p)
{
  // ONX_Model_RenderLight array delete
  onfree(p);
}

void* ONX_Model_RenderLight::operator new(std::size_t, void* p)
{
  // ONX_Model_RenderLight placement new
  return p;
}

void ONX_Model_RenderLight::operator delete(void*, void*)
{
  // ONX_Model_RenderLight placement delete
  return;
}


// ONX_Model new/delete

void* ONX_Model::operator new(std::size_t sz)
{
  // ONX_Model new
  return onmalloc(sz);
}

void ONX_Model::operator delete(void* p)
{
  // ONX_Model delete
  onfree(p);
}

void* ONX_Model::operator new[] (std::size_t sz)
{
  // ONX_Model array new
  return onmalloc(sz);
}

void ONX_Model::operator delete[] (void* p)
{
  // ONX_Model array delete
  onfree(p);
}

void* ONX_Model::operator new(std::size_t, void* p)
{
  // ONX_Model placement new
  return p;
}

void ONX_Model::operator delete(void*, void*)
{
  // ONX_Model placement delete
  return;
}

#endif

//
//
////////////////////////////////////////////////////////////////////////////////


static
bool ONX_IsValidNameFirstChar( wchar_t c )
{
  if ( c > 127 )
    return true;
  if ( c < '0' )
    return false;
  if ( c <= '9' )
    return true;
  if ( c < 'A' )
    return false;
  if ( c <= 'Z' )
    return true;
  if ( c == '_' )
    return true;
  if ( c < 'a' )
    return false;
  if ( c <= 'z' )
    return true;
  return false;
}

static
bool ONX_IsValidNameSecondChar( wchar_t c )
{
  // control characters, double quote, and DEL are 
  // not permited in names
  return (c >= 32 && c != 34 && c != 127);
}

bool ONX_IsValidName( 
          const wchar_t* name 
          )
{
  bool is_valid = (0 != name && ONX_IsValidNameFirstChar(*name));
  if ( is_valid )
  {
    bool is_integer = (*name >= '0' && *name <= '9');
    name++;
    while ( ONX_IsValidNameSecondChar(*name) )
    {
      if ( *name < '0' || *name >= '9' )
        is_integer = false;
      name++;
    }
    if ( *name || is_integer )
      is_valid = false;
    else if ( name[-1] <= 32 )
      is_valid = false; // last character cannot be a space
  }
  return is_valid;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

ONX_Model_RenderLight::ONX_Model_RenderLight()
{
}

ONX_Model_RenderLight::~ONX_Model_RenderLight()
{
}

ONX_Model_RenderLight::ONX_Model_RenderLight(const ONX_Model_RenderLight& src) 
             : m_light(src.m_light), 
               m_attributes(src.m_attributes)
{
}

ONX_Model_RenderLight& ONX_Model_RenderLight::operator=(const ONX_Model_RenderLight& src)
{
  if ( this != &src )
  {
    m_light = src.m_light;
    m_attributes = src.m_attributes;
  }
  return *this;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


ONX_Model_Object::ONX_Model_Object() 
                 : m_bDeleteObject(0),
                   m_object(0), 
                   m_ref_count(0)
{
}

void ONX_Model_Object::Destroy()
{
  if ( m_ref_count ) 
  {
    if ( *m_ref_count > 0 )
      (*m_ref_count)--;
    if ( *m_ref_count <= 0 ) 
    {
      delete m_ref_count;
      m_ref_count = 0;
    }
  }
  if ( 0 == m_ref_count && 0 != m_object && m_bDeleteObject )
  {
    delete m_object;
  }
  m_object = 0;
  m_bDeleteObject = false;
}

ONX_Model_Object::~ONX_Model_Object()
{
  Destroy();
}

ONX_Model_Object::ONX_Model_Object(const ONX_Model_Object& src) 
             : m_bDeleteObject(0),
               m_object(0),
               m_ref_count(0)
{
  *this = src;
}

ONX_Model_Object& ONX_Model_Object::operator=(const ONX_Model_Object& src)
{
  if ( this != &src )
  {
    Destroy();
    m_bDeleteObject = src.m_bDeleteObject;
    m_object = src.m_object;
    m_attributes = src.m_attributes;
    m_ref_count = src.m_ref_count;
    if ( 0 != m_object && m_bDeleteObject ) 
    {
      if ( 0 != m_ref_count )
        (*m_ref_count)++;
      else 
      {
        m_ref_count = new unsigned int;
        *m_ref_count = 2; // 2 because this and src reference same m_object
        const_cast<ONX_Model_Object*>(&src)->m_ref_count = m_ref_count; // need to defeat const
      }
    }
  }
  return *this;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


ONX_Model_UserData::ONX_Model_UserData() 
: m_uuid(ON_nil_uuid)
, m_usertable_3dm_version(0)
, m_usertable_opennurbs_version(0)
{
}

ONX_Model_UserData::~ONX_Model_UserData()
{
}

ONX_Model_UserData::ONX_Model_UserData(const ONX_Model_UserData& src) 
: m_uuid(src.m_uuid)
, m_goo(src.m_goo)
, m_usertable_3dm_version(src.m_usertable_3dm_version)
, m_usertable_opennurbs_version(src.m_usertable_opennurbs_version)
{
}

ONX_Model_UserData& ONX_Model_UserData::operator=(const ONX_Model_UserData& src)
{
  if ( this != &src )
  {
    m_uuid = src.m_uuid;
    m_goo = src.m_goo;
    m_usertable_3dm_version = src.m_usertable_3dm_version;
    m_usertable_opennurbs_version = src.m_usertable_opennurbs_version;
  }
  return *this;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

ONX_Model::ONX_Model() 
          : m_3dm_file_version(0), 
            m_3dm_opennurbs_version(0),
            m_file_length(0),
            m_crc_error_count(0)
{
  m_sStartSectionComments.Empty();
  m_properties.Default();
  m_settings.Default();
}

ONX_Model::~ONX_Model()
{
  Destroy();
}

void ONX_Model::Destroy()
{
  int i;
  m_3dm_file_version = 0;
  m_3dm_opennurbs_version = 0;
  m_sStartSectionComments.Empty();
  m_properties.Default();
  m_settings.Default();

  for ( i = 0; i < m_history_record_table.Count(); i++ )
    delete m_history_record_table[i];
  m_history_record_table.Zero();

  for ( i = 0; i < m_bitmap_table.Count(); i++ )
    delete m_bitmap_table[i];
  m_bitmap_table.Zero();

  m_bitmap_table.SetCapacity(0);
  m_mapping_table.SetCapacity(0);
  m_material_table.SetCapacity(0);
  m_linetype_table.SetCapacity(0);
  m_layer_table.SetCapacity(0);
  m_group_table.SetCapacity(0);
  m_font_table.SetCapacity(0);
  m_dimstyle_table.SetCapacity(0);
  m_light_table.SetCapacity(0);
  m_hatch_pattern_table.SetCapacity(0);
  m_idef_table.SetCapacity(0);
  m_object_table.SetCapacity(0);
  m_history_record_table.SetCapacity(0);  
  m_userdata_table.SetCapacity(0);

  m_file_length = 0;
  m_crc_error_count = 0;

  DestroyCache();
}


void ONX_Model::DestroyCache()
{
  m_mapping_id_index.Empty();
  m_material_id_index.Empty();
  m_idef_id_index.Empty();
  m_object_id_index.Empty();

  m__object_table_bbox.Destroy();
}

ON_BoundingBox ONX_Model::BoundingBox() const
{
  if( !m__object_table_bbox.IsValid() && m_object_table.Count() > 0 )
  {
    ON_BoundingBox bbox;
    int i, object_count = m_object_table.Count();
    for ( i = 0; i < object_count; i++ )
    {
      const ON_Geometry* geo = ON_Geometry::Cast(m_object_table[i].m_object);
      if ( geo )
        bbox.Union(geo->BoundingBox());
      const_cast<ONX_Model*>(this)->m__object_table_bbox = bbox;
    }
  }
  return m__object_table_bbox;
}

void ONX_Model::GetRenderMaterial( const ON_3dmObjectAttributes& attributes, ON_Material& material ) const
{
  int material_index = -1;

  switch ( attributes.MaterialSource() )
  {
  case ON::material_from_layer:
    if ( attributes.m_layer_index >= 0 && attributes.m_layer_index < m_layer_table.Count() )
      material_index = m_layer_table[attributes.m_layer_index].RenderMaterialIndex();
    break;
  case ON::material_from_object:
    material_index = attributes.m_material_index;
    break;

  case ON::material_from_parent:
    material_index = attributes.m_material_index;
    // TODO: If object is an idef, get material from iref attributes.
    break;
  }

  if ( material_index < 0 || material_index >= m_material_table.Count() )
  {
    material_index = -1;
    material.Default();
  }
  else
  {
    material = m_material_table[material_index];
  }

  material.SetMaterialIndex(material_index);
}

void ONX_Model::GetRenderMaterial( 
      int object_index,
      ON_Material& material 
      ) const
{
  if ( object_index < 0 || object_index >= m_object_table.Count() )
  {
    material.Default();
    material.SetMaterialIndex(-1);    
  }
  else
    GetRenderMaterial( m_object_table[object_index].m_attributes, material );
}

void ONX_Model::GetLinetype( const ON_3dmObjectAttributes& attributes, ON_Linetype& linetype ) const
{
  int linetype_index = -1;

  switch ( attributes.LinetypeSource() )
  {
  case ON::linetype_from_layer:
    if ( attributes.m_layer_index >= 0 && attributes.m_layer_index < m_layer_table.Count() )
      linetype_index = m_layer_table[attributes.m_layer_index].LinetypeIndex();
    break;
  case ON::linetype_from_object:
    linetype_index = attributes.m_linetype_index;
    break;
  case ON::linetype_from_parent:
    linetype_index = attributes.m_linetype_index;
    // TODO: if object is an instance definition, get linetype
    //       from instance references.
    break;
  }

  if ( linetype_index < 0 || linetype_index >= m_linetype_table.Count() )
  {
    linetype_index = -1;
    linetype.Default();
  }
  else
  {
    linetype = m_linetype_table[linetype_index];
  }

  linetype.SetLinetypeIndex(linetype_index);
}

void ONX_Model::GetLinetype( 
      int object_index,
      ON_Linetype& linetype
      ) const
{
  if ( object_index < 0 || object_index >= m_object_table.Count() )
  {
    linetype.Default();
    linetype.SetLinetypeIndex(-1);    
  }
  else
  {
    GetLinetype( m_object_table[object_index].m_attributes, linetype );
  }
}



ON_Color ONX_Model::WireframeColor( const ON_3dmObjectAttributes& attributes ) const
{
  ON_Color color = ON_UNSET_COLOR;

  switch ( attributes.ColorSource() )
  {
  case ON::color_from_layer:
    if ( attributes.m_layer_index >= 0 && attributes.m_layer_index < m_layer_table.Count() )
      color = m_layer_table[attributes.m_layer_index].Color();
    break;

  case ON::color_from_object:
    color = attributes.m_color;
    break;

  case ON::color_from_material:
    {
      ON_Material mat;
      GetRenderMaterial( attributes, mat );
      color = mat.Diffuse();
    }
    break;

  case ON::color_from_parent:
    color = attributes.m_color;
    // TODO: if object is an instance definition, get color
    //       from instance references.
    break;
  }

  if ( color == ON_UNSET_COLOR )
    color.SetRGB(128,128,128);

  return color;
}

ON_Color ONX_Model::WireframeColor(int object_index) const
{
  ON_Color c;
  if ( object_index < 0 || object_index >= m_object_table.Count() )
  {
    ON_3dmObjectAttributes a;
    c = a.m_color;
  }
  else
    c = WireframeColor( m_object_table[object_index].m_attributes );
  return c;
}


void ONX_DumpView( ON_TextLog& dump, const ON_3dmView& view )
{
  view.Dump(dump);
}

void ONX_Model::DumpSummary( ON_TextLog& dump ) const
{
  dump.Print("File version: %d\n",m_3dm_file_version);
  dump.Print("File openNURBS version: %d\n",m_3dm_opennurbs_version);
  if ( m_file_length > 0 )
    dump.Print("File length: %d bytes\n",m_file_length);

  if ( m_sStartSectionComments.Length() > 0 ) 
  {
    dump.Print("Start section comments:\n");
    dump.PushIndent();
    dump.PrintWrappedText(m_sStartSectionComments);
    dump.PopIndent();
    dump.Print("\n");
  }
  
  m_properties.Dump(dump);

  dump.Print("\n");

  m_settings.Dump(dump);

  dump.Print("\n");

  dump.Print("Contents:\n");
  dump.PushIndent();
  dump.Print("%d embedded bitmaps\n",m_bitmap_table.Count());
  dump.Print("%d render material definitions\n",m_material_table.Count());
  dump.Print("%d line type definitions\n",m_linetype_table.Count());
  dump.Print("%d layers\n",m_layer_table.Count());
  dump.Print("%d render lights\n",m_light_table.Count());
  dump.Print("%d groups\n",m_group_table.Count());
  dump.Print("%d objects\n",m_object_table.Count());
  dump.Print("%d user data objects\n",m_userdata_table.Count());
  dump.PopIndent();
}

void ONX_Model::DumpBitmapTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_bitmap_table.Count(); i++ )
  {
    dump.Print("Bitmap %d:\n",i);
    dump.PushIndent();
    m_bitmap_table[i]->Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpTextureMappingTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_mapping_table.Count(); i++ )
  {
    dump.Print("Texture Mapping %d:\n",i);
    dump.PushIndent();
    m_mapping_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpMaterialTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_material_table.Count(); i++ )
  {
    dump.Print("Material %d:\n",i);
    dump.PushIndent();
    m_material_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpLinetypeTable( ON_TextLog& dump ) const
{
  int i;
  for ( i = 0; i < m_linetype_table.Count(); i++ )
  {
    dump.Print("Linetype %d:\n",i);
    dump.PushIndent();
    m_linetype_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpLayerTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_layer_table.Count(); i++ )
  {
    dump.Print("Layer %d:\n",i);
    dump.PushIndent();
    m_layer_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpLightTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_light_table.Count(); i++ )
  {
    dump.Print("Light %d:\n",i);
    dump.PushIndent();
    m_light_table[i].m_attributes.Dump(dump);
    m_light_table[i].m_light.Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpGroupTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_group_table.Count(); i++ )
  {
    dump.Print("Group %d:\n",i);
    dump.PushIndent();
    m_group_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpFontTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_font_table.Count(); i++ )
  {
    dump.Print("Font %d:\n",i);
    dump.PushIndent();
    m_font_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpDimStyleTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_dimstyle_table.Count(); i++ )
  {
    dump.Print("DimStyle %d:\n",i);
    dump.PushIndent();
    m_dimstyle_table[i].Dump(dump);
    dump.PopIndent();
  }
}


void ONX_Model::DumpHatchPatternTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_hatch_pattern_table.Count(); i++ )
  {
    dump.Print("HatchPattern %d:\n",i);
    dump.PushIndent();
    m_hatch_pattern_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model::DumpIDefTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_idef_table.Count(); i++ )
  {
    dump.Print("Instance Definition %d:\n",i);
    dump.PushIndent();
    m_idef_table[i].Dump(dump);
    dump.PopIndent();
  }
}

void ONX_Model_Object::Dump( ON_TextLog& dump ) const
{
  if ( 0 != m_object )
  {
    m_object->Dump(dump);

    // user data attached to this object
    const ON_UserData* ud = m_object->FirstUserData();
    while(0 != ud)
    {
      dump.Print("object user data:\n");
      dump.PushIndent();
      ud->Dump(dump);
      dump.PopIndent();
      ud = ud->Next();
    }
  }
  else
  {
    dump.Print("NULL m_object pointer\n");
  }

  // Use Cast() if you need to get at the details.
  /*
  const ON_Geometry* pGeometry = ON_Geometry::Cast(m_object);
  if ( pGeometry ) 
  {
    // m_object is some type of geometric object
    if ( ON_Extrusion::Cast(m_object) ) 
    {
      // m_object is derived from ON_Extrusion
      //  Note: 
      //   ON_Extrusion::BrepForm() will return a brep form
      //   if you don't want to explicitly handle extrusions. 
      const ON_Extrusion* extrusion = ON_Extrusion::Cast(m_object);
    }
    else if ( ON_Brep::Cast(m_object) ) 
    {
      // m_object is derived from ON_Brep
      const ON_Brep* brep = ON_Brep::Cast(m_object);
    }
    else if ( pGeometry->HasBrepForm() ) 
    {
      // m_object is note derived from ON_Brep but its geometry can
      // be represented by an ON_Brep.
      ON_Brep* brep = pGeometry->BrepForm();
      // you manage the ON_Brep returned by pGeometry->BrepForm();
      delete brep;
    }
    else if ( ON_Curve::Cast(m_object) ) 
    {
      // curve objects
      if ( ON_NurbsCurve::Cast(m_object) ) {
        const ON_NurbsCurve* pCurve = ON_NurbsCurve::Cast(m_object);
      }
      else if ( ON_ArcCurve::Cast(m_object) ) {
        const ON_ArcCurve* pCurve = ON_ArcCurve::Cast(m_object);
      }
      else if ( ON_CurveOnSurface::Cast(m_object) ) {
        const ON_CurveOnSurface* pCurve = ON_CurveOnSurface::Cast(m_object);
      }
      else if ( ON_BrepEdge::Cast(m_object) ) {
        const ON_BrepEdge* pCurve = ON_BrepEdge::Cast(m_object);
      }
      else if ( ON_LineCurve::Cast(m_object) ) {
        const ON_LineCurve* pCurve = ON_LineCurve::Cast(m_object);
      }
      else if ( ON_PolyCurve::Cast(m_object) ) {
        const ON_PolyCurve* pCurve = ON_PolyCurve::Cast(m_object);
      }
      else if ( ON_PolylineCurve::Cast(m_object) ) {
        const ON_PolylineCurve* pCurve = ON_PolylineCurve::Cast(m_object);
      }
      else if ( ON_CurveProxy::Cast(m_object) ) {
        const ON_CurveProxy* pCurve = ON_CurveProxy::Cast(m_object);
      }
      else {
        const ON_Curve* pCurve = ON_Curve::Cast(m_object);
      }
    }
    else if ( ON_Surface::Cast(m_object) ) 
    {
      // surface objects
      if ( ON_NurbsSurface::Cast(m_object) ) {
        const ON_NurbsSurface* pSurface = ON_NurbsSurface::Cast(m_object);
      }
      else if ( ON_PlaneSurface::Cast(m_object) ) {
        const ON_PlaneSurface* pSurface = ON_PlaneSurface::Cast(m_object);
      }
      else if ( ON_RevSurface::Cast(m_object) ) {
        const ON_RevSurface* pSurface = ON_RevSurface::Cast(m_object);
      }
      else if ( ON_BrepFace::Cast(m_object) ) {
        const ON_BrepFace* pSurface = ON_BrepFace::Cast(m_object);
      }
      else if ( ON_SurfaceProxy::Cast(m_object) ) {
        const ON_SurfaceProxy* pSurface = ON_SurfaceProxy::Cast(m_object);
      }
      else {
        const ON_Surface* pSurface = ON_Surface::Cast(m_object);
      }
    }
    else if ( ON_Mesh::Cast(m_object) ) 
    {
      const ON_Mesh* pMesh = ON_Mesh::Cast(m_object);
    }
  }
  */
}

void ONX_Model::DumpObjectTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_object_table.Count(); i++ )
  {
    dump.Print("Object %d:\n",i);
    dump.PushIndent();

    // object's attibutes
    m_object_table[i].m_attributes.Dump(dump);

    // object definition
    m_object_table[i].Dump(dump);

    dump.PopIndent();
  }
}

void ONX_Model::DumpHistoryRecordTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_history_record_table.Count(); i++ )
  {
    dump.Print("History record %d:\n",i);
    dump.PushIndent();

    const ON_HistoryRecord* history_record = m_history_record_table[i];
    if ( history_record )
    {
      history_record->Dump(dump);
    }
    else
    {
      dump.Print("Missing.\n");
    }

    dump.PopIndent();
  }
}

void ONX_Model::DumpUserDataTable( ON_TextLog& dump) const
{
  int i;
  for ( i = 0; i < m_userdata_table.Count(); i++ )
  {
    const ONX_Model_UserData& ud = m_userdata_table[i];
    dump.Print("User Data Table %d:\n",i);
    dump.PushIndent();
    dump.Print("uuid = "); dump.Print(ud.m_uuid); dump.Print("\n");
    ud.m_goo.Dump(dump);    
    dump.PopIndent();
  }
}

void ONX_Model::Dump( ON_TextLog& dump ) const
{
  dump.Print("Model summary:\n");
  dump.PushIndent();
  DumpSummary(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Bitmap table:\n");
  dump.PushIndent();
  DumpBitmapTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("TextureMapping table:\n");
  dump.PushIndent();
  DumpTextureMappingTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Material table:\n");
  dump.PushIndent();
  DumpMaterialTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Line type table:\n");
  dump.PushIndent();
  DumpLinetypeTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Layer table:\n");
  dump.PushIndent();
  DumpLayerTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Group table:\n");
  dump.PushIndent();
  DumpGroupTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Font table:\n");
  dump.PushIndent();
  DumpFontTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("DimStyle table:\n");
  dump.PushIndent();
  DumpDimStyleTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Light table:\n");
  dump.PushIndent();
  DumpLightTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("HatchPattern table:\n");
  dump.PushIndent();
  DumpHatchPatternTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Instance Definition table:\n");
  dump.PushIndent();
  DumpIDefTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("Object table:\n");
  dump.PushIndent();
  DumpObjectTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("History record table:\n");
  dump.PushIndent();
  DumpHistoryRecordTable(dump);
  dump.PopIndent();
  dump.Print("\n");

  dump.Print("User data table:\n");
  dump.PushIndent();
  DumpUserDataTable(dump);
  dump.PopIndent();
  dump.Print("\n");
}

static 
bool CheckForCRCErrors( 
          ON_BinaryArchive& archive, 
          ONX_Model& model,
          ON_TextLog* error_log,
          const char* sSection
          )
{
  // returns true if new CRC errors are found
  bool rc = false;
  int new_crc_count = archive.BadCRCCount();
  
  if ( model.m_crc_error_count != new_crc_count ) 
  {
    if ( error_log )
    {
      error_log->Print("ERROR: Corrupt %s. (CRC errors).\n",sSection);
      error_log->Print("-- Attempting to continue.\n");
    }
    model.m_crc_error_count = new_crc_count;
    rc = true;
  }

  return rc;
}

class ON__CIndexPair
{
public:
  static int CompareOldIndex( const ON__CIndexPair* a, const ON__CIndexPair* b );
  static int CompareOldAndNewIndex( const ON__CIndexPair* a, const ON__CIndexPair* b );
  int m_old_index;  // value in model.m_..._table[m_table_index].m_..._index; (Read from file)
  int m_new_index;  // index in model.m_..._table[] array
};

int ON__CIndexPair::CompareOldIndex( const ON__CIndexPair* a, const ON__CIndexPair* b )
{
  return (a->m_old_index - b->m_old_index);
}

int ON__CIndexPair::CompareOldAndNewIndex( const ON__CIndexPair* a, const ON__CIndexPair* b )
{
  int i;
  if ( 0 == (i = a->m_old_index - b->m_old_index) )
    i = a->m_new_index - b->m_new_index;
  return i;
}

class ON__CIndexMaps
{
public:
  ON__CIndexMaps( ONX_Model& model ) 
    : m_model(model),
      m_bRemapLayerIndex(0), 
      m_bRemapMaterialIndex(0), 
      m_bRemapLinetypeIndex(0),
      m_bRemapGroupIndex(0),
      m_bRemapFontIndex(0),
      m_bRemapDimstyleIndex(0),
      m_bRemapHatchPatternIndex(0),
      m_layer_count(0),
      m_group_count(0),
      m_material_count(0),
      m_linetype_count(0),
      m_font_count(0),
      m_dimstyle_count(0),
      m_hatch_pattern_count(0),
      m_default_layer_index(0),
      m_default_group_index(-1),
      m_default_material_index(-1),
      m_default_linetype_index(-1),
      m_default_font_index(0),
      m_default_dimstyle_index(0),
      m_default_hatch_pattern_index(-1)
  { 
    CreateHelper();
  }

  /*
  Description:
    Remap all tables in m_model.
  Returns:
    Number of indices that were changed.
  */
  int RemapModel();

  ONX_Model& m_model;

  bool m_bRemapLayerIndex;
  bool m_bRemapMaterialIndex;
  bool m_bRemapLinetypeIndex;
  bool m_bRemapGroupIndex;
  bool m_bRemapFontIndex;
  bool m_bRemapDimstyleIndex;
  bool m_bRemapHatchPatternIndex;

  int m_layer_count;
  int m_group_count;
  int m_material_count;
  int m_linetype_count;
  int m_font_count;
  int m_dimstyle_count;
  int m_hatch_pattern_count;

  int m_default_layer_index;
  int m_default_group_index;
  int m_default_material_index;
  int m_default_linetype_index;
  int m_default_font_index;
  int m_default_dimstyle_index;
  int m_default_hatch_pattern_index;

  ON_SimpleArray<ON__CIndexPair> m_layer_map;
  ON_SimpleArray<ON__CIndexPair> m_group_map;
  ON_SimpleArray<ON__CIndexPair> m_material_map;
  ON_SimpleArray<ON__CIndexPair> m_linetype_map;
  ON_SimpleArray<ON__CIndexPair> m_font_map;
  ON_SimpleArray<ON__CIndexPair> m_dimstyle_map;
  ON_SimpleArray<ON__CIndexPair> m_hatch_pattern_map;


  /*
  Description:
    Low level tool to convert old_layer_index into a valid 
    m_layer_table[] index.
  Parameters:
    old_layer_index - [in]
  Returns:
    new layer index to use.
  */
  int RemapLayerIndex( int old_layer_index ) const;

  /*
  Description:
    Low level tool to convert old_material_index into a valid 
    m_material_table[] index  or -1 if the default material 
    should be used.
  Parameters:
    old_material_index - [in]
  Returns:
    new material index to use.
  */
  int RemapMaterialIndex( int old_material_index ) const;

  /*
  Description:
    Low level tool to convert old_linetype_index into a valid 
    m_linetype_table[] index or -1 if the default linetype 
    should be used.
  Parameters:
    old_linetype_index - [in]
  Returns:
    new linetype index to use.
  */
  int RemapLinetypeIndex( int old_linetype_index ) const;

  /*
  Description:
    Low level tool to convert old_group_index into a valid 
    m_group_table[] index or -1 if no conversion is possible.
  Parameters:
    old_group_index - [in]
  Returns:
    new group index to use or -1 if old_group_index makes no sense.
  */
  int RemapGroupIndex( int old_group_index ) const;

  /*
  Description:
    Low level tool to convert old_font_index into a valid 
    m_font_table[] index or -1 if no conversion is possible.
  Parameters:
    old_font_index - [in]
  Returns:
    new font index to use or -1 if old_font_index makes no sense.
  */
  int RemapFontIndex( int old_font_index ) const;

  /*
  Description:
    Low level tool to convert old_dimstyle_index into a valid 
    m_dimstyle_table[] index or -1 if no conversion is possible.
  Parameters:
    old_dimstyle_index - [in]
  Returns:
    new dimstyle index to use or -1 if old_dimstyle_index makes no sense.
  */
  int RemapDimstyleIndex( int old_dimstyle_index ) const;

  /*
  Description:
    Low level tool to convert old_hatch_pattern_index into a valid 
    m_hatch_pattern_table[] index or -1 if no conversion is possible.
  Parameters:
    old_hatch_pattern_index - [in]
  Returns:
    new hatch pattern index to use or -1 if old_hatch_pattern_index makes no sense.
  */
  int RemapHatchPatternIndex( int old_hatch_pattern_index ) const;

  /*
  Description:
    Low level tool to remap table indices used by model objects in
    the object attributes class.
  */
  int RemapGeometryAndObjectAttributes( ONX_Model_Object& );

  /*
  Description:
    Low level tool to remap table indices used by model objects in
    the object attributes class.
  Returns:
    Number of indices that were changed.
  */
  int RemapGeometryAttributes( ON_Object* );

  /*
  Description:
    Low level tool to remap table indices saved in
    the object attributes class.
  Returns:
    Number of indices that were changed.
  */
  int RemapObjectAttributes( ON_3dmObjectAttributes& );

  /*
  Description:
    Low level tool to remap table indices saved in
    the object attributes class.
  Returns:
    Number of indices that were changed.
  */
  int RemapLayerAttributes( ON_Layer& );

  /*
  Description:
    Low level tool to remap material table indices saved in
    the rendering attributes class.
  Returns:
    Number of indices that were changed.
  */
  int RemapRenderingAttributes( ON_RenderingAttributes& ra );

private:
  int CreateHelper();

private:
  // no implementation - prohibit use
  ON__CIndexMaps(const ON__CIndexMaps&);
  ON__CIndexMaps& operator=(const ON__CIndexMaps&);
};


int ON__CIndexMaps::CreateHelper()
{
  int change_count = 0;
  int i;

  // bitmaps are not referenced by index any place,
  // so just set bitmap index to match table index
  for ( i = 0; i < m_model.m_bitmap_table.Count(); i++ )
  {
    ON_Bitmap* bitmap = m_model.m_bitmap_table[i];
    if ( !bitmap )
    {
      change_count++;
      m_model.m_bitmap_table.Remove(i);
      i--;
      continue;
    }

    if ( bitmap->m_bitmap_index != i )
    {
      bitmap->m_bitmap_index = i;
      change_count++;
    }
    if ( ON_nil_uuid == bitmap->m_bitmap_id )
    {
      ON_CreateUuid(bitmap->m_bitmap_id);
      change_count++;
    }
  }

  // texture maps are not referenced  by index 
  // so just set texture map index to match table
  // index
  m_model.m_mapping_id_index.Empty();
  m_model.m_mapping_id_index.Reserve(m_model.m_mapping_table.Count());
  for ( i = 0; i < m_model.m_mapping_table.Count(); i++ )
  {
    ON_TextureMapping& mapping = m_model.m_mapping_table[i];
    if ( mapping.m_mapping_index != i )
    {
      mapping.m_mapping_index = i;
      change_count++;
    }
    if ( ON_nil_uuid == mapping.m_mapping_id )
    {
      ON_CreateUuid(mapping.m_mapping_id);
      change_count++;
    }
    m_model.m_mapping_id_index.AddUuidIndex(mapping.m_mapping_id,i,false);
  }

  // make sure material indices are valid
  m_model.m_material_id_index.Empty();
  m_model.m_material_id_index.Reserve(m_model.m_material_table.Count());
  m_bRemapMaterialIndex = false;
  m_material_count = m_model.m_material_table.Count();
  m_material_map.SetCount(0);
  m_material_map.Reserve(m_material_count);
  for ( i = 0; i < m_material_count; i++ )
  {
    ON_Material& material = m_model.m_material_table[i];
    ON__CIndexPair& ip = m_material_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = material.m_material_index;
    if ( material.m_material_index != i )
    {
      material.m_material_index = i;
      m_bRemapMaterialIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == material.m_material_id )
    {
      ON_CreateUuid(material.m_material_id);
      change_count++;
    }
    m_model.m_material_id_index.AddUuidIndex(material.m_material_id,i,false);
  }

  // make sure linetype indices are valid
  m_bRemapLinetypeIndex = false;
  m_linetype_count = m_model.m_linetype_table.Count();
  m_linetype_map.SetCount(0);
  m_linetype_map.Reserve(m_linetype_count);
  for ( i = 0; i < m_linetype_count; i++ )
  {
    ON_Linetype& linetype = m_model.m_linetype_table[i];
    ON__CIndexPair& ip = m_linetype_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = linetype.m_linetype_index;
    if ( linetype.m_linetype_index != i )
    {
      linetype.m_linetype_index = i;
      m_bRemapLinetypeIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == linetype.m_linetype_id )
    {
      ON_CreateUuid(linetype.m_linetype_id);
      change_count++;
    }
  }

  // make sure there is at least one layer
  if ( m_model.m_layer_table.Count() < 1 )
  {
    ON_Layer layer;
    layer.Default();
    m_model.GetUnusedLayerName(layer.m_name);
    if ( !ONX_IsValidName(layer.m_name) )
      layer.m_name = L"Default";
    layer.m_layer_index = 0;
    ON_CreateUuid(layer.m_layer_id);
    m_model.m_layer_table.Append(layer);
    change_count++;
  }

  // make sure layer indices are valid
  m_bRemapLayerIndex = false;
  m_layer_count = m_model.m_layer_table.Count();
  m_layer_map.SetCount(0);
  m_layer_map.Reserve(m_layer_count);
  for ( i = 0; i < m_layer_count; i++ )
  {
    ON_Layer& layer = m_model.m_layer_table[i];
    ON__CIndexPair& ip = m_layer_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = layer.m_layer_index;
    if ( layer.m_layer_index != i )
    {
      layer.m_layer_index = i;
      m_bRemapLayerIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == layer.m_layer_id )
    {
      ON_CreateUuid(layer.m_layer_id);
      change_count++;
    }
  }

  // make sure group indices are valid
  m_bRemapGroupIndex = false;
  m_group_count = m_model.m_group_table.Count();
  m_group_map.SetCount(0);
  m_group_map.Reserve(m_group_count);
  for ( i = 0; i < m_group_count; i++ )
  {
    ON_Group& group = m_model.m_group_table[i];
    ON__CIndexPair& ip = m_group_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = group.m_group_index;
    if ( group.m_group_index != i )
    {
      group.m_group_index = i;
      m_bRemapGroupIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == group.m_group_id )
    {
      ON_CreateUuid(group.m_group_id);
      change_count++;
    }
  }

  // make sure there is at least one font
  if ( m_model.m_font_table.Count() < 1 )
  {
    ON_Font font;
    font.Defaults();
    if ( !ONX_IsValidName(font.m_font_name) )
      font.m_font_name = L"Default";
    font.m_font_index = 0;
    ON_CreateUuid(font.m_font_id);
    m_model.m_font_table.Append(font);
    change_count++;
  }

  // make sure font indices are valid
  m_bRemapFontIndex = false;
  m_font_count = m_model.m_font_table.Count();
  m_font_map.SetCount(0);
  m_font_map.Reserve(m_font_count);
  for ( i = 0; i < m_font_count; i++ )
  {
    ON_Font& font = m_model.m_font_table[i];
    ON__CIndexPair& ip = m_font_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = font.m_font_index;
    if ( font.m_font_index != i )
    {
      font.m_font_index = i;
      m_bRemapFontIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == font.m_font_id )
    {
      ON_CreateUuid(font.m_font_id);
      change_count++;
    }
  }

  // make sure there is at least one dimstyle
  if ( m_model.m_dimstyle_table.Count() < 1 )
  {
    ON_DimStyle dimstyle;
    dimstyle.SetDefaults();
    if ( !ONX_IsValidName(dimstyle.m_dimstyle_name) )
      dimstyle.m_dimstyle_name = L"Default";
    dimstyle.m_dimstyle_index = 0;
    ON_CreateUuid(dimstyle.m_dimstyle_id);
    dimstyle.m_fontindex = 0;
    m_model.m_dimstyle_table.Append(dimstyle);
    change_count++;
  }

  // make sure dimstyle indices are valid
  m_bRemapDimstyleIndex = false;
  m_dimstyle_count = m_model.m_dimstyle_table.Count();
  m_dimstyle_map.SetCount(0);
  m_dimstyle_map.Reserve(m_dimstyle_count);
  for ( i = 0; i < m_dimstyle_count; i++ )
  {
    ON_DimStyle& dimstyle = m_model.m_dimstyle_table[i];
    ON__CIndexPair& ip = m_dimstyle_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = dimstyle.m_dimstyle_index;
    if ( dimstyle.m_dimstyle_index != i )
    {
      dimstyle.m_dimstyle_index = i;
      m_bRemapDimstyleIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == dimstyle.m_dimstyle_id )
    {
      ON_CreateUuid(dimstyle.m_dimstyle_id);
      change_count++;
    }
  }

  // lights are not referenced by index any place,
  // so just set light index to match table index
  for ( i = 0; i < m_model.m_light_table.Count(); i++ )
  {
    ONX_Model_RenderLight& light = m_model.m_light_table[i];
    if ( light.m_light.m_light_index != i )
    {
      light.m_light.m_light_index = i;
      change_count++;
    }

    if ( light.m_light.m_light_id == light.m_attributes.m_uuid )
    {
      // ids match - this is good
      if ( ON_nil_uuid == light.m_light.m_light_id )
      {
        // ids not set
        ON_CreateUuid(light.m_light.m_light_id);
        light.m_attributes.m_uuid = light.m_light.m_light_id;
        change_count++;
      }
    }
    else if ( ON_nil_uuid == light.m_light.m_light_id )
    {
      // id not set on the light object
      light.m_light.m_light_id = light.m_attributes.m_uuid;
      change_count++;
    }
    else if ( ON_nil_uuid == light.m_attributes.m_uuid )
    {
      // id not set on the attributes
      light.m_attributes.m_uuid = light.m_light.m_light_id;
      change_count++;
    }
    else 
    {
      // id's are different - the one on the light object wins
      light.m_attributes.m_uuid = light.m_light.m_light_id;
      change_count++;
    }
  }

  // make sure hatch pattern indices are valid
  m_bRemapHatchPatternIndex = false;
  m_hatch_pattern_count = m_model.m_hatch_pattern_table.Count();
  m_hatch_pattern_map.SetCount(0);
  m_hatch_pattern_map.Reserve(m_hatch_pattern_count);
  for ( i = 0; i < m_hatch_pattern_count; i++ )
  {
    ON_HatchPattern& hatchpattern = m_model.m_hatch_pattern_table[i];
    ON__CIndexPair& ip = m_hatch_pattern_map.AppendNew();
    ip.m_new_index = i;
    ip.m_old_index = hatchpattern.m_hatchpattern_index;
    if ( ip.m_new_index != ip.m_old_index )
    {
      hatchpattern.m_hatchpattern_index = i;
      m_bRemapHatchPatternIndex = true;
      change_count++;
    }
    if ( ON_nil_uuid == hatchpattern.m_hatchpattern_id )
    {
      ON_CreateUuid(hatchpattern.m_hatchpattern_id);
      change_count++;
    }
  }

  // make sure idefs have valid ids
  m_model.m_idef_id_index.Empty();
  m_model.m_idef_id_index.Reserve(m_model.m_idef_table.Count());
  for ( i = 0; i < m_model.m_idef_table.Count(); i++ )
  {
    ON_InstanceDefinition& idef = m_model.m_idef_table[i];
    if ( ON_nil_uuid == idef.m_uuid )
    {
      ON_CreateUuid(idef.m_uuid);
      change_count++;
    }
    m_model.m_idef_id_index.AddUuidIndex(idef.m_uuid,i,false);
  }

  // make sure objects have valid ids
  m_model.m_object_id_index.Empty();
  m_model.m_object_id_index.Reserve(m_model.m_object_table.Count());
  for ( i = 0; i < m_model.m_object_table.Count(); i++ )
  {
    ONX_Model_Object& mo = m_model.m_object_table[i];
    if ( ON_nil_uuid == mo.m_attributes.m_uuid )
    {
      ON_CreateUuid(mo.m_attributes.m_uuid);
      change_count++;
    }
    m_model.m_object_id_index.AddUuidIndex(mo.m_attributes.m_uuid,i,false);
  }

  // make sure history records have valid ids
  for ( i = 0; i < m_model.m_history_record_table.Count(); i++ )
  {
    ON_HistoryRecord* hr = m_model.m_history_record_table[i];
    if ( !hr )
    {
      change_count++;
      m_model.m_history_record_table.Remove(i);
      i--;
      continue;
    }
    if ( ON_nil_uuid == hr->m_record_id )
    {
      ON_CreateUuid(hr->m_record_id);
      change_count++;
    }
  }

  // Sort the maps
  if ( m_bRemapLayerIndex )
    m_layer_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );
  if ( m_bRemapGroupIndex )
    m_group_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );
  if ( m_bRemapMaterialIndex )
    m_material_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );
  if ( m_bRemapLinetypeIndex )
    m_linetype_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );
  if ( m_bRemapFontIndex )
    m_font_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );
  if ( m_bRemapDimstyleIndex )
    m_dimstyle_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );
  if ( m_bRemapHatchPatternIndex )
    m_hatch_pattern_map.QuickSort( ON__CIndexPair::CompareOldAndNewIndex );

  return change_count;
}

int ON__CIndexMaps::RemapGeometryAttributes( ON_Object* object )
{
  int change_count = 0;

  switch(object ? object->ObjectType() : ON::unknown_object_type )
  {
  case ON::layer_object:
    {
      ON_Layer* layer = ON_Layer::Cast(object);
      if ( layer )
        change_count += RemapLayerAttributes(*layer);
    }
    break;

  case ON::annotation_object:
    {
      ON_Annotation2* ann = ON_Annotation2::Cast(object);
      if ( ann )
      {
        if (ann->IsText() )
        {
          // ann->m_index is a font index
          int old_font_index = ann->m_index;
          int new_font_index = RemapFontIndex(old_font_index);
          if ( new_font_index != old_font_index )
          {
            ann->m_index = new_font_index;
            change_count++;
          }
        }
        else
        {
          // ann->m_index is a dimstyle index
          int old_dimstyle_index = ann->m_index;
          int new_dimstyle_index = RemapDimstyleIndex(old_dimstyle_index);
          {
            if ( old_dimstyle_index != new_dimstyle_index )
            {
              ann->m_index = new_dimstyle_index;
              change_count++;
            }
          }
        }
      }
    }
    break;

  case ON::hatch_object:
    {
      ON_Hatch* hatch_object = ON_Hatch::Cast(object);
      if ( hatch_object )
      {
        int old_hatch_pattern_index = hatch_object->PatternIndex();
        int new_hatch_pattern_index = RemapHatchPatternIndex(old_hatch_pattern_index);
        if ( old_hatch_pattern_index != new_hatch_pattern_index )
          hatch_object->SetPatternIndex(new_hatch_pattern_index);
      }
    }
    break;

  default:
    // other object types skipped on purpose
    break;
  }

  return change_count;
}

int ON__CIndexMaps::RemapGeometryAndObjectAttributes( ONX_Model_Object& model_object )
{
  int geometry_change_count  = RemapGeometryAttributes(const_cast<ON_Object*>(model_object.m_object));
  int attribute_change_count = RemapObjectAttributes( model_object.m_attributes );
  return (geometry_change_count + attribute_change_count);
}

int ON__CIndexMaps::RemapModel()
{
  int change_count = 0;

  int i, old_index, new_index;

  // make sure current layer is valid and in "normal" mode
  old_index =  m_model.m_settings.m_current_layer_index;
  new_index = RemapLayerIndex(old_index);
  if ( new_index < 0 || new_index >= m_layer_count )
  {
    new_index = 0;
  }
  m_model.m_settings.m_current_layer_index = new_index;
  if ( !m_model.m_layer_table[new_index].IsVisibleAndNotLocked() )
  {
    m_model.m_layer_table[new_index].SetVisible( true );
    m_model.m_layer_table[new_index].SetLocked( false );
  }
  m_default_layer_index = m_model.m_settings.m_current_layer_index;

  for ( i = 0; i < m_model.m_layer_table.Count(); i++ )
  {
    change_count += RemapLayerAttributes(m_model.m_layer_table[i]);
  }

  for ( i = 0; i < m_model.m_dimstyle_table.Count(); i++ )
  {
    old_index = m_model.m_dimstyle_table[i].m_fontindex;
    new_index = RemapFontIndex(old_index);
    if ( new_index != old_index )
    {
      m_model.m_dimstyle_table[i].m_fontindex = new_index;
      change_count++;
    }
  }

  for ( i = 0; i < m_model.m_light_table.Count(); i++ )
  {
    change_count += RemapObjectAttributes( m_model.m_light_table[i].m_attributes );
  }

  for ( i = 0; i < m_model.m_object_table.Count(); i++ )
  {
    change_count += RemapGeometryAndObjectAttributes( m_model.m_object_table[i] );
  }

  return change_count;
}



static int RemapIndexHelper( 
                int old_index, 
                bool bRemapIndex, 
                int count, 
                int default_index,
                const ON_SimpleArray<ON__CIndexPair>& imap
                )
{
  int new_index = old_index;
  if ( bRemapIndex )
  {
    ON__CIndexPair ip;
    memset(&ip,0,sizeof(ip));
    ip.m_old_index = old_index;
    int j = imap.BinarySearch(&ip,ON__CIndexPair::CompareOldIndex);
    if ( j >= 0 )
      new_index = imap[j].m_new_index;
  }
  if ( new_index < 0 || new_index >= count )
    new_index = default_index;
  return new_index;
}

int ON__CIndexMaps::RemapLayerIndex( int old_layer_index ) const
{
  return RemapIndexHelper(
              old_layer_index,
              m_bRemapLayerIndex,
              m_layer_count,
              m_default_layer_index,
              m_layer_map
              );
}

int ON__CIndexMaps::RemapMaterialIndex( int old_material_index ) const
{
  return RemapIndexHelper(
              old_material_index,
              m_bRemapMaterialIndex,
              m_material_count,
              m_default_material_index,
              m_material_map
              );
}

int ON__CIndexMaps::RemapLinetypeIndex( int old_linetype_index ) const
{
  return RemapIndexHelper(
              old_linetype_index,
              m_bRemapLinetypeIndex,
              m_linetype_count,
              m_default_linetype_index,
              m_linetype_map
              );
}

int ON__CIndexMaps::RemapGroupIndex( int old_group_index ) const
{
  return RemapIndexHelper(
              old_group_index,
              m_bRemapGroupIndex,
              m_group_count,
              m_default_group_index,
              m_group_map
              );
}

int ON__CIndexMaps::RemapFontIndex( int old_font_index ) const
{
  return RemapIndexHelper(
              old_font_index,
              m_bRemapFontIndex,
              m_font_count,
              m_default_font_index,
              m_font_map
              );
}

int ON__CIndexMaps::RemapDimstyleIndex( int old_dimstyle_index ) const
{
  return RemapIndexHelper(
              old_dimstyle_index,
              m_bRemapDimstyleIndex,
              m_dimstyle_count,
              m_default_dimstyle_index,
              m_dimstyle_map
              );
}

int ON__CIndexMaps::RemapHatchPatternIndex( int old_hatch_pattern_index ) const
{
  return RemapIndexHelper(
              old_hatch_pattern_index,
              m_bRemapHatchPatternIndex,
              m_hatch_pattern_count,
              m_default_hatch_pattern_index,
              m_hatch_pattern_map
              );
}

int ON__CIndexMaps::RemapRenderingAttributes( ON_RenderingAttributes& ra )
{
  int change_count = 0;
  int old_material_index, new_material_index, i;
  for ( i = ra.m_materials.Count()-1; i >= 0; i-- )
  {
    ON_MaterialRef& mr = ra.m_materials[i]; 

    old_material_index = mr.m_material_index;
    if ( old_material_index >= 0 )
    {
      new_material_index = RemapMaterialIndex(old_material_index);
      if ( old_material_index != new_material_index )
      {
        mr.m_material_index = new_material_index;
        change_count++;
      }
    }
    else
      mr.m_material_index = -1;

    old_material_index = mr.m_material_backface_index;
    if ( old_material_index >= 0 )
    {
      new_material_index = RemapMaterialIndex(old_material_index);
      if ( old_material_index != new_material_index )
      {
        mr.m_material_backface_index = new_material_index;
        change_count++;
      }
    }
    else
      mr.m_material_backface_index = -1;

    if ( -1 == mr.m_material_index || mr.m_material_index >= m_material_count )
    {
      if ( ON_nil_uuid == mr.m_material_id )
        mr.m_material_index = -1;
      else if ( !m_model.m_material_id_index.FindUuid(mr.m_material_id,&mr.m_material_index) )
        mr.m_material_index = -1;
    }
    else if ( m_model.m_material_table[mr.m_material_index].m_material_id != mr.m_material_id )
    {
      new_material_index = -1;
      if (    !ON_UuidIsNil(mr.m_material_id)
           && m_model.m_material_id_index.FindUuid(mr.m_material_id,&new_material_index) 
           && new_material_index >= 0 
           && new_material_index < m_material_count
         )
      {
        mr.m_material_index = new_material_index;
      }
      else
      {
        mr.m_material_id = m_model.m_material_table[mr.m_material_index].m_material_id;
      }
    }

    if ( -1 == mr.m_material_backface_index || mr.m_material_backface_index >= m_material_count )
    {
      if ( ON_nil_uuid == mr.m_material_backface_id )
        mr.m_material_backface_index = -1;
      else if ( !m_model.m_material_id_index.FindUuid(mr.m_material_backface_id,&mr.m_material_backface_index) )
        mr.m_material_backface_index = -1;
    }
    else if ( m_model.m_material_table[mr.m_material_backface_index].m_material_id != mr.m_material_backface_id )
    {
      new_material_index = -1;
      if (   !ON_UuidIsNil(mr.m_material_backface_id)
           && m_model.m_material_id_index.FindUuid(mr.m_material_backface_id,&new_material_index) 
           && new_material_index >= 0 
           && new_material_index < m_material_count
          )
      {
        mr.m_material_backface_index = new_material_index;
      }
      else
      {
        mr.m_material_backface_id = m_model.m_material_table[mr.m_material_backface_index].m_material_id;
      }
    }

    if ( mr.m_material_index < 0 && mr.m_material_backface_index < 0 )
    {
      ra.m_materials.Remove(i);
    }
  }
  return change_count;
}

int ON__CIndexMaps::RemapLayerAttributes( ON_Layer& layer )
{
  int change_count = 0;

  if ( ON_UuidIsNil(layer.m_layer_id) )
  {
    ON_CreateUuid(layer.m_layer_id);
    change_count++;
  }

  int old_linetype_index = layer.m_linetype_index;
  int new_linetype_index = RemapLinetypeIndex(old_linetype_index);
  if ( old_linetype_index != new_linetype_index )
  {
    layer.m_linetype_index = new_linetype_index;
    change_count++;
  }

  int old_material_index = layer.m_material_index;
  int new_material_index = RemapMaterialIndex(old_material_index);
  if ( old_material_index != new_material_index )
  {
    layer.m_material_index = new_material_index;
    change_count++;
  }

  change_count += RemapRenderingAttributes(layer.m_rendering_attributes);

  return change_count;
}

int ON__CIndexMaps::RemapObjectAttributes( ON_3dmObjectAttributes& a )
{
  int change_count = 0;

  int i;
  if ( ON_UuidIsNil(a.m_uuid) )
  {
    ON_CreateUuid(a.m_uuid);
    change_count++;
  }

  int old_layer_index = a.m_layer_index;
  int new_layer_index = RemapLayerIndex(old_layer_index);
  if ( old_layer_index != new_layer_index )
  {
    a.m_layer_index = new_layer_index;
    change_count++;
  }

  int old_linetype_index = a.m_linetype_index;
  int new_linetype_index = RemapLinetypeIndex(old_linetype_index);
  if ( old_linetype_index != new_linetype_index )
  {
    a.m_linetype_index = new_linetype_index;
    change_count++;
  }

  int old_material_index = a.m_material_index;
  int new_material_index = RemapMaterialIndex(old_material_index);
  if ( old_material_index != new_material_index )
  {
    a.m_material_index = new_material_index;
    change_count++;
  }

  if ( a.TopGroup() != -1 )
  {
    bool bUpdateGroupList = true;
    ON_SimpleArray<int> group_list;
    a.GetGroupList(group_list);
    for ( i = group_list.Count()-1; i >= 0; i-- )
    {
      int old_group_index = group_list[i];
      int new_group_index = RemapGroupIndex(old_group_index);
      if ( new_group_index < 0 )
      {
        group_list.Remove(i);
        bUpdateGroupList = true;
        change_count++;
      }
      else if ( old_group_index != new_group_index )
      {
        group_list[i] = new_group_index;
        bUpdateGroupList = true;
        change_count++;
      }
    } 

    if ( bUpdateGroupList || group_list.Count() == 0 )
    {
      a.RemoveFromAllGroups();
      for( i = 0; i < group_list.Count(); i++ )
        a.AddToGroup(group_list[i]);
    }
  }

  change_count += RemapRenderingAttributes(a.m_rendering_attributes);

  return change_count;
}

void ONX_Model::Polish()
{
  DestroyCache();

  // make sure there is a valid revision history
  if ( m_properties.m_RevisionHistory.m_revision_count == 0 )
    m_properties.m_RevisionHistory.NewRevision();


  // Get maps sorted so BinarySearch calls in PolishAttributes
  // will work.
  ON__CIndexMaps imaps(*this);
  imaps.RemapModel();
}

bool ONX_Model::Read( 
       const char* filename,
       ON_TextLog* error_log
       )
{
  Destroy(); // get rid of any residual stuff
  bool rc = false;
  if ( 0 != filename )
  {
    FILE* fp = ON::OpenFile(filename,"rb");
    if ( 0 != fp )
    {
      ON_BinaryFile file(ON::read3dm,fp);
      rc = Read(file,error_log);
      ON::CloseFile(fp);
    }
  }
  return rc;
}

bool ONX_Model::Read( 
       const wchar_t* filename,
       ON_TextLog* error_log
       )
{
  Destroy(); // get rid of any residual stuff
  bool rc = false;
  if ( 0 != filename )
  {
    FILE* fp = ON::OpenFile(filename,L"rb");
    if ( 0 != fp )
    {
      ON_BinaryFile file(ON::read3dm,fp);
      rc = Read(file,error_log);
      ON::CloseFile(fp);
    }
  }
  return rc;
}

bool ONX_Model::Read( 
       ON_BinaryArchive& archive,
       ON_TextLog* error_log
       )
{
  const int max_error_count = 2000;
  int error_count = 0;
  bool return_code = true;
  int count, rc;

  Destroy(); // get rid of any residual stuff

  // STEP 1: REQUIRED - Read start section
  if ( !archive.Read3dmStartSection( &m_3dm_file_version, m_sStartSectionComments ) )
  {
    if ( error_log) error_log->Print("ERROR: Unable to read start section. (ON_BinaryArchive::Read3dmStartSection() returned false.)\n");
    return false;
  }
  else if ( CheckForCRCErrors( archive, *this, error_log, "start section" ) )
    return_code = false;

  // STEP 2: REQUIRED - Read properties section
  if ( !archive.Read3dmProperties( m_properties ) )
  {
    if ( error_log) error_log->Print("ERROR: Unable to read properties section. (ON_BinaryArchive::Read3dmProperties() returned false.)\n");
    return false;
  }
  else if ( CheckForCRCErrors( archive, *this, error_log, "properties section" ) )
    return_code = false;

  // version of opennurbs used to write the file.
  m_3dm_opennurbs_version = archive.ArchiveOpenNURBSVersion();

  // STEP 3: REQUIRED - Read properties section
  if ( !archive.Read3dmSettings( m_settings ) )
  {
    if ( error_log) error_log->Print("ERROR: Unable to read settings section. (ON_BinaryArchive::Read3dmSettings() returned false.)\n");
    return false;
  }
  else if ( CheckForCRCErrors( archive, *this, error_log, "settings section" ) )
    return_code = false;

  // STEP 4: REQUIRED - Read embedded bitmap table
  if ( archive.BeginRead3dmBitmapTable() )
  {
    // At the moment no bitmaps are embedded so this table is empty
    ON_Bitmap* pBitmap = NULL;
    for( count = 0; true; count++ ) 
    {
      pBitmap = NULL;
      rc = archive.Read3dmBitmap(&pBitmap);
      if ( rc==0 )
        break; // end of bitmap table
      if ( rc < 0 ) 
      {
        if ( error_log) 
        {
          error_log->Print("ERROR: Corrupt bitmap found. (ON_BinaryArchive::Read3dmBitmap() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        return_code = false;
      }
      m_bitmap_table.Append(pBitmap);
    }

    // If BeginRead3dmBitmapTable() returns true, 
    // then you MUST call EndRead3dmBitmapTable().
    if ( !archive.EndRead3dmBitmapTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt bitmap table. (ON_BinaryArchive::EndRead3dmBitmapTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "bitmap table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log) 
    {
      error_log->Print("WARNING: Missing or corrupt bitmap table. (ON_BinaryArchive::BeginRead3dmBitmapTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }



  // STEP 5: REQUIRED - Read texture mapping table
  if ( archive.BeginRead3dmTextureMappingTable() )
  {
    ON_TextureMapping* pTextureMapping = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmTextureMapping(&pTextureMapping);
      if ( rc==0 )
        break; // end of texture_mapping table
      if ( rc < 0 ) 
      {
        if ( error_log) 
        {
          error_log->Print("ERROR: Corrupt render texture_mapping found. (ON_BinaryArchive::Read3dmTextureMapping() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        continue;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pTextureMapping);
      m_mapping_table.Append(*pTextureMapping);
      pTextureMapping->m_mapping_index = count;
      ud.MoveUserDataTo(*m_mapping_table.Last(),false);
      delete pTextureMapping;
      pTextureMapping = NULL;
    }
    
    // If BeginRead3dmTextureMappingTable() returns true, 
    // then you MUST call EndRead3dmTextureMappingTable().
    if ( !archive.EndRead3dmTextureMappingTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt render texture_mapping table. (ON_BinaryArchive::EndRead3dmTextureMappingTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "render texture_mapping table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt render texture_mapping table. (ON_BinaryArchive::BeginRead3dmTextureMappingTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }


  // STEP 6: REQUIRED - Read render material table
  if ( archive.BeginRead3dmMaterialTable() )
  {
    ON_Material* pMaterial = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmMaterial(&pMaterial);
      if ( rc==0 )
        break; // end of material table
      if ( rc < 0 ) 
      {
        if ( error_log) 
        {
          error_log->Print("ERROR: Corrupt render material found. (ON_BinaryArchive::Read3dmMaterial() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pMaterial = new ON_Material; // use default
        pMaterial->m_material_index = count;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pMaterial);
      m_material_table.Append(*pMaterial);
      ud.MoveUserDataTo(*m_material_table.Last(),false);
      delete pMaterial;
      pMaterial = NULL;
    }
    
    // If BeginRead3dmMaterialTable() returns true, 
    // then you MUST call EndRead3dmMaterialTable().
    if ( !archive.EndRead3dmMaterialTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt render material table. (ON_BinaryArchive::EndRead3dmMaterialTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "render material table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt render material table. (ON_BinaryArchive::BeginRead3dmMaterialTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }


  // STEP 7: REQUIRED - Read line type table
  if ( archive.BeginRead3dmLinetypeTable() )
  {
    ON_Linetype* pLinetype = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmLinetype(&pLinetype);
      if ( rc==0 )
        break; // end of linetype table
      if ( rc < 0 ) 
      {
        if ( error_log) 
        {
          error_log->Print("ERROR: Corrupt render linetype found. (ON_BinaryArchive::Read3dmLinetype() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pLinetype = new ON_Linetype; // use default
        pLinetype->m_linetype_index = count;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pLinetype);
      m_linetype_table.Append(*pLinetype);
      ud.MoveUserDataTo(*m_linetype_table.Last(),false);
      delete pLinetype;
      pLinetype = NULL;
    }
    
    // If BeginRead3dmLinetypeTable() returns true, 
    // then you MUST call EndRead3dmLinetypeTable().
    if ( !archive.EndRead3dmLinetypeTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt render linetype table. (ON_BinaryArchive::EndRead3dmLinetypeTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "render linetype table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt render linetype table. (ON_BinaryArchive::BeginRead3dmLinetypeTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 8: REQUIRED - Read layer table
  if ( archive.BeginRead3dmLayerTable() )
  {
    ON_Layer* pLayer = NULL;
    for( count = 0; true; count++ ) 
    {
      pLayer = NULL;
      rc = archive.Read3dmLayer(&pLayer);
      if ( rc==0 )
        break; // end of layer table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt layer found. (ON_BinaryArchive::Read3dmLayer() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pLayer = new ON_Layer; // use default
        pLayer->m_layer_index = count;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pLayer);
      m_layer_table.Append(*pLayer);
      ud.MoveUserDataTo(*m_layer_table.Last(),false);
      delete pLayer;
      pLayer = NULL;
    }
    
    // If BeginRead3dmLayerTable() returns true, 
    // then you MUST call EndRead3dmLayerTable().
    if ( !archive.EndRead3dmLayerTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt render layer table. (ON_BinaryArchive::EndRead3dmLayerTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "layer table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log) 
    {
      error_log->Print("WARNING: Missing or corrupt layer table. (ON_BinaryArchive::BeginRead3dmLayerTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 9: REQUIRED - Read group table
  if ( archive.BeginRead3dmGroupTable() )
  {
    ON_Group* pGroup = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmGroup(&pGroup);
      if ( rc==0 )
        break; // end of group table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt group found. (ON_BinaryArchive::Read3dmGroup() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pGroup = new ON_Group; // use default
        pGroup->m_group_index = -1;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pGroup);
      m_group_table.Append(*pGroup);
      ud.MoveUserDataTo(*m_group_table.Last(),false);
      delete pGroup;
      pGroup = NULL;
    }
    
    // If BeginRead3dmGroupTable() returns true, 
    // then you MUST call EndRead3dmGroupTable().
    if ( !archive.EndRead3dmGroupTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt group table. (ON_BinaryArchive::EndRead3dmGroupTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "group table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log) 
    {
      error_log->Print("WARNING: Missing or corrupt group table. (ON_BinaryArchive::BeginRead3dmGroupTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 10: REQUIRED - Read font table
  if ( archive.BeginRead3dmFontTable() )
  {
    ON_Font* pFont = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmFont(&pFont);
      if ( rc==0 )
        break; // end of font table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt font found. (ON_BinaryArchive::Read3dmFont() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pFont = new ON_Font; // use default
        pFont->m_font_index = -1;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pFont);
      m_font_table.Append(*pFont);
      ud.MoveUserDataTo(*m_font_table.Last(),false);
      delete pFont;
      pFont = NULL;
    }
    
    // If BeginRead3dmFontTable() returns true, 
    // then you MUST call EndRead3dmFontTable().
    if ( !archive.EndRead3dmFontTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt font table. (ON_BinaryArchive::EndRead3dmFontTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "font table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt font table. (ON_BinaryArchive::BeginRead3dmFontTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 11: REQUIRED - Read dimstyle table
  if ( archive.BeginRead3dmDimStyleTable() )
  {
    ON_DimStyle* pDimStyle = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmDimStyle(&pDimStyle);
      if ( rc==0 )
        break; // end of dimstyle table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt dimstyle found. (ON_BinaryArchive::Read3dmDimStyle() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pDimStyle = new ON_DimStyle; // use default
        pDimStyle->m_dimstyle_index = count;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pDimStyle);
      m_dimstyle_table.Append(*pDimStyle);
      ud.MoveUserDataTo(*m_dimstyle_table.Last(),false);
      delete pDimStyle;
      pDimStyle = NULL;
    }
    
    // If BeginRead3dmDimStyleTable() returns true, 
    // then you MUST call EndRead3dmDimStyleTable().
    if ( !archive.EndRead3dmDimStyleTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt dimstyle table. (ON_BinaryArchive::EndRead3dmDimStyleTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "dimstyle table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt dimstyle table. (ON_BinaryArchive::BeginRead3dmDimStyleTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 12: REQUIRED - Read render lights table
  if ( archive.BeginRead3dmLightTable() )
  {
    ON_Light* pLight = NULL;
    ON_3dmObjectAttributes object_attributes;
    for( count = 0; true; count++ ) 
    {
      object_attributes.Default();
      rc = archive.Read3dmLight(&pLight,&object_attributes);
      if ( rc==0 )
        break; // end of light table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt render light found. (ON_BinaryArchive::Read3dmLight() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        continue;
      }
      ONX_Model_RenderLight& light = m_light_table.AppendNew();
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pLight);
      light.m_light = *pLight;
      ud.MoveUserDataTo(light.m_light,false);
      light.m_attributes = object_attributes;
      delete pLight;
      pLight = NULL;
    }
    
    // If BeginRead3dmLightTable() returns true, 
    // then you MUST call EndRead3dmLightTable().
    if ( !archive.EndRead3dmLightTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt render light table. (ON_BinaryArchive::EndRead3dmLightTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "render light table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt render light table. (ON_BinaryArchive::BeginRead3dmLightTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 13 - read hatch pattern table
  if ( archive.BeginRead3dmHatchPatternTable() )
  {
    ON_HatchPattern* pHatchPattern = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmHatchPattern(&pHatchPattern);
      if ( rc==0 )
        break; // end of hatchpattern table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt hatchpattern found. (ON_BinaryArchive::Read3dmHatchPattern() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        pHatchPattern = new ON_HatchPattern; // use default
        pHatchPattern->m_hatchpattern_index = count;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pHatchPattern);
      m_hatch_pattern_table.Append(*pHatchPattern);
      ud.MoveUserDataTo(*m_hatch_pattern_table.Last(),false);
      delete pHatchPattern;
      pHatchPattern = NULL;
    }
    
    // If BeginRead3dmHatchPatternTable() returns true, 
    // then you MUST call EndRead3dmHatchPatternTable().
    if ( !archive.EndRead3dmHatchPatternTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt hatchpattern table. (ON_BinaryArchive::EndRead3dmHatchPatternTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "hatchpattern table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt hatchpattern table. (ON_BinaryArchive::BeginRead3dmHatchPatternTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 14: REQUIRED - Read instance definition table
  if ( archive.BeginRead3dmInstanceDefinitionTable() )
  {
    ON_InstanceDefinition* pIDef = NULL;
    for( count = 0; true; count++ ) 
    {
      rc = archive.Read3dmInstanceDefinition(&pIDef);
      if ( rc==0 )
        break; // end of instance definition table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Corrupt instance definition found. (ON_BinaryArchive::Read3dmInstanceDefinition() < 0.)\n");
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        continue;
      }
      ON_UserDataHolder ud;
      ud.MoveUserDataFrom(*pIDef);
      m_idef_table.Append(*pIDef);
      ud.MoveUserDataTo(*m_idef_table.Last(),false);
      delete pIDef;
    }
    
    // If BeginRead3dmInstanceDefinitionTable() returns true, 
    // then you MUST call EndRead3dmInstanceDefinitionTable().
    if ( !archive.EndRead3dmInstanceDefinitionTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt instance definition table. (ON_BinaryArchive::EndRead3dmInstanceDefinitionTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "instance definition table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt instance definition table. (ON_BinaryArchive::BeginRead3dmInstanceDefinitionTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }



  // STEP 15: REQUIRED - Read object (geometry and annotation) table
  if ( archive.BeginRead3dmObjectTable() )
  {
    // optional filter made by setting ON::object_type bits 
    // For example, if you just wanted to just read points and meshes, you would use
    // object_filter = ON::point_object | ON::mesh_object;
    int object_filter = 0; 

    for( count = 0; true; count++ ) 
    {
      ON_Object* pObject = NULL;
      ON_3dmObjectAttributes attributes;
      rc = archive.Read3dmObject(&pObject,&attributes,object_filter);
      if ( rc == 0 )
        break; // end of object table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Object table entry %d is corrupt. (ON_BinaryArchive::Read3dmObject() < 0.)\n",count);
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        continue;
      }
      if ( m_crc_error_count != archive.BadCRCCount() ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: Object table entry %d is corrupt. (CRC errors).\n",count);
          error_log->Print("-- Attempting to continue.\n");
        }
        m_crc_error_count = archive.BadCRCCount();
      }
      if ( pObject ) 
      {
        ONX_Model_Object& mo = m_object_table.AppendNew();
        mo.m_object = pObject;
        mo.m_bDeleteObject = true;
        mo.m_attributes = attributes;
      }
      else
      {
        if ( error_log)
        {
          if ( rc == 2 )
            error_log->Print("WARNING: Skipping object table entry %d because it's filtered.\n",count);
          else if ( rc == 3 )
            error_log->Print("WARNING: Skipping object table entry %d because it's newer than this code.  Update your OpenNURBS toolkit.\n",count);
          else
            error_log->Print("WARNING: Skipping object table entry %d for unknown reason.\n",count);
        }
      }
    }
    
    // If BeginRead3dmObjectTable() returns true, 
    // then you MUST call EndRead3dmObjectTable().
    if ( !archive.EndRead3dmObjectTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt object light table. (ON_BinaryArchive::EndRead3dmObjectTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "object table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt object table. (ON_BinaryArchive::BeginRead3dmObjectTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 16: Read history table
  if ( archive.BeginRead3dmHistoryRecordTable() )
  {
    for( count = 0; true; count++ ) 
    {
      ON_HistoryRecord* pHistoryRecord = NULL;
      rc = archive.Read3dmHistoryRecord(pHistoryRecord);
      if ( rc == 0 )
        break; // end of history record table
      if ( rc < 0 ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: History record table entry %d is corrupt. (ON_BinaryArchive::Read3dmHistoryRecord() < 0.)\n",count);
          error_count++;
          if ( error_count > max_error_count )
            return false;
          error_log->Print("-- Attempting to continue.\n");
        }
        continue;
      }
      if ( m_crc_error_count != archive.BadCRCCount() ) 
      {
        if ( error_log)
        {
          error_log->Print("ERROR: History record table entry %d is corrupt. (CRC errors).\n",count);
          error_log->Print("-- Attempting to continue.\n");
        }
        m_crc_error_count = archive.BadCRCCount();
      }
      if ( pHistoryRecord ) 
      {
        m_history_record_table.Append(pHistoryRecord);
      }
      else
      {
        if ( error_log)
        {
          error_log->Print("WARNING: Skipping history record table entry %d for unknown reason.\n",count);
        }
      }
    }
    
    // If BeginRead3dmHistoryRecordTable() returns true, 
    // then you MUST call EndRead3dmHistoryRecordTable().
    if ( !archive.EndRead3dmHistoryRecordTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt object light table. (ON_BinaryArchive::EndRead3dmObjectTable() returned false.)\n");
      return false;
    }
    if ( CheckForCRCErrors( archive, *this, error_log, "history record table" ) )
      return_code = false;
  }
  else     
  {
    if ( error_log)
    {
      error_log->Print("WARNING: Missing or corrupt history record table. (ON_BinaryArchive::BeginRead3dmHistoryRecordTable() returned false.)\n");
      error_log->Print("-- Attempting to continue.\n");
    }
    return_code = false;
  }

  // STEP 17: OPTIONAL - Read user tables as anonymous goo
  // If you develop a plug-ins or application that uses OpenNURBS files,
  // you can store anything you want in a user table.
  for(count=0;true;count++)
  {
    if ( archive.Archive3dmVersion() <= 1 )
    {
      // no user tables in version 1 archives.
      break;
    }

    {
      ON__UINT32 tcode = 0;
      ON__INT64 big_value = 0;
      if ( !archive.PeekAt3dmBigChunkType(&tcode,&big_value) )
        break;
      if ( TCODE_USER_TABLE != tcode )
        break;
    }
    ON_UUID plugin_id = ON_nil_uuid;
    bool bGoo = false;
    int usertable_3dm_version = 0;
    int usertable_opennurbs_version = 0;
    if ( !archive.BeginRead3dmUserTable( plugin_id, &bGoo, &usertable_3dm_version, &usertable_opennurbs_version ) )
    {
      // attempt to skip bogus user table
      const ON__UINT64 pos0 = archive.CurrentPosition();
      ON__UINT32 tcode = 0;
      ON__INT64 big_value = 0;
      if  ( !archive.BeginRead3dmBigChunk(&tcode,&big_value) )
        break;
      if ( !archive.EndRead3dmChunk() )
        break;
      const ON__UINT64 pos1 = archive.CurrentPosition();
      if (pos1 <= pos0)
        break;
      if ( TCODE_USER_TABLE != tcode )
        break;

      continue; // skip this bogus user table
    }

    ONX_Model_UserData& ud = m_userdata_table.AppendNew();
    ud.m_uuid = plugin_id;
    ud.m_usertable_3dm_version = usertable_3dm_version;
    ud.m_usertable_opennurbs_version = usertable_opennurbs_version;

    if ( !archive.Read3dmAnonymousUserTable( usertable_3dm_version, usertable_opennurbs_version, ud.m_goo ) )
    {
      if ( error_log) error_log->Print("ERROR: User data table entry %d is corrupt. (ON_BinaryArchive::Read3dmAnonymousUserTable() is false.)\n",count);
      break;
    }

    // If BeginRead3dmObjectTable() returns true, 
    // then you MUST call EndRead3dmUserTable().
    if ( !archive.EndRead3dmUserTable() )
    {
      if ( error_log) error_log->Print("ERROR: Corrupt user data table. (ON_BinaryArchive::EndRead3dmUserTable() returned false.)\n");
      break;
    }
  }

  // STEP 18: OPTIONAL - check for end mark
  if ( !archive.Read3dmEndMark(&m_file_length) )
  {
    if ( archive.Archive3dmVersion() != 1 ) 
    {
      // some v1 files are missing end-of-archive markers
      if ( error_log) error_log->Print("ERROR: ON_BinaryArchive::Read3dmEndMark(&m_file_length) returned false.\n");
    }
  }

  // Remap layer, material, linetype, font, dimstyle, hatch pattern, etc., 
  // indices so the correspond to the model's table array index.
  //
  // Polish also sets revision history information if it is missing.
  // In this case, that is not appropriate so the value of
  // m_properties.m_RevisionHistory is saved before calling Polish()
  // and restored afterwards.
  const ON_3dmRevisionHistory saved_revision_history(m_properties.m_RevisionHistory);
  Polish();
  m_properties.m_RevisionHistory = saved_revision_history;

  return return_code;
}

static 
void ONX_Model_WriteHelper(ON_BinaryFile& file)
{
  file.EnableSave3dmRenderMeshes(true);
  file.EnableSave3dmAnalysisMeshes(true);
  file.EnableSaveUserData(true);
}

bool ONX_Model::Write( 
       const char* filename,
       int version,
       const char* sStartSectionComment,
       ON_TextLog* error_log
       )
{
  bool rc = false;
  if ( 0 != filename )
  {
    FILE* fp = ON::OpenFile( filename, "wb" );
    if ( 0 != fp )
    {
      ON_BinaryFile file( ON::write3dm, fp );
      ONX_Model_WriteHelper(file);
      rc = Write( file, version, sStartSectionComment, error_log );
      ON::CloseFile(fp);
    }
  }
  return rc;
}

bool ONX_Model::Write( 
       const wchar_t* filename,
       int version,
       const char* sStartSectionComment,
       ON_TextLog* error_log
       )
{
  bool rc = false;
  if ( 0 != filename )
  {
    FILE* fp = ON::OpenFile( filename, L"wb" );
    if ( 0 != fp )
    {
      ON_BinaryFile file( ON::write3dm, fp );
      ONX_Model_WriteHelper(file);
      rc = Write( file, version, sStartSectionComment, error_log );
      ON::CloseFile(fp);
    }
  }
  return rc;
}


bool ONX_Model::Write( 
       ON_BinaryArchive& archive,
       int version,
       const char*,
       ON_TextLog* error_log
       )
{
  int i;

  if ( !IsValid(error_log) )
  {
    // This model is not valid.  See the error_log for details.
    if ( error_log) error_log->Print("ONX_Model::Write Your model is not valid and will not be saved.\n");
    return false;
  }

  if ( 0 != version )
  {
    if (    version < 2 
         || version > ON_BinaryArchive::CurrentArchiveVersion() 
         || (version >= 50 && 0 != (version%10))
         || (version < 50 && version > ON_BinaryArchive::CurrentArchiveVersion()/10)
         )
    {
      // version must be 0, 2, 3, 4, 5 or 50
      version = 0;
      if ( error_log) error_log->Print("ONX_Model::Write version parameter = %d; it must be 0, or >= 2 and <= %d, or a multiple of 10 >= 50 and <= %d.\n",
                      version,ON_BinaryArchive::CurrentArchiveVersion()/10,ON_BinaryArchive::CurrentArchiveVersion());
    }
  }

  if ( !archive.WriteMode() )
  {
    // You passed in a bogus archive.  You must pass ON::write3dm to the
    // archive constructor.
    if ( error_log) error_log->Print("ONX_Model::Write archive.Mode() is not ON::write3dm.\n"
                    "See ONX_Model::Write example in the header file.\n");
    return false;
  }

  bool ok;

  // START SECTION
  ok = archive.Write3dmStartSection( version, m_sStartSectionComments );
  if ( !ok )
  {
    // make sure your archive was created with ON::write3dm mode.
    if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmStartSection() failed.\n"
                    "Your archive is not properly initialized\n"
                    "(make sure you passed ON::write3dm to the constructor),\n"
                    "a file is locked, a disk is locked, or something along those lines.\n");
    return false;
  }

  // PROPERTIES SECTION
  ok = archive.Write3dmProperties( m_properties );
  if ( !ok )
  {
    // make sure m_properties is valid
    if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmProperties() failed.\n"
                    "Your m_properties information is not valid or basic file writing failed.\n"
                    );
    return false;
  }

  // SETTINGS SECTION
  ok = archive.Write3dmSettings( m_settings );
  if ( !ok )
  {
    // make sure m_settings is valid
    if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmSettings() failed.\n"
                    "Your m_settings information is not valid or basic file writing failed.\n");
    return false;
  }

  // BITMAP TABLE
  ok = archive.BeginWrite3dmBitmapTable();
  if ( !ok )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmBitmapTable() failed.\n");
    return false;
  }
  for( i = 0; ok && i < m_bitmap_table.Count(); i++ )
  {
    ok = archive.Write3dmBitmap(*m_bitmap_table[i]);
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmBitmap(m_bitmap_table[%d]) failed.\n",i);
    }
  }
  if ( !archive.EndWrite3dmBitmapTable() )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmBitmapTable() failed.\n");
    return false;
  }
  if (!ok)
    return false;

  // RENDER TEXTURE MAPPING TABLE
  if ( archive.Archive3dmVersion() >= 4 )
  {
    ok = archive.BeginWrite3dmTextureMappingTable();
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmTextureMappingTable() failed.\n");
      return false;
    }
    for( i = 0; ok && i < m_mapping_table.Count(); i++ )
    {
      ok = archive.Write3dmTextureMapping(m_mapping_table[i]);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmTextureMapping(m_mapping_table[%d]) failed.\n",i);
      }
    }
    if ( !archive.EndWrite3dmTextureMappingTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmTextureMappingTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }

  // RENDER MATERIAL TABLE
  ok = archive.BeginWrite3dmMaterialTable();
  if ( !ok )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmMaterialTable() failed.\n");
    return false;
  }
  for( i = 0; ok && i < m_material_table.Count(); i++ )
  {
    ok = archive.Write3dmMaterial(m_material_table[i]);
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmMaterial(m_material_table[%d]) failed.\n",i);
    }
  }
  if ( !archive.EndWrite3dmMaterialTable() )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmMaterialTable() failed.\n");
    return false;
  }
  if (!ok)
    return false;


  // LINETYPE TABLE
  if ( archive.Archive3dmVersion() >= 4 )
  {
    ok = archive.BeginWrite3dmLinetypeTable();
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmLinetypeTable() failed.\n");
      return false;
    }
    for( i = 0; ok && i < m_linetype_table.Count(); i++ )
    {
      ok = archive.Write3dmLinetype(m_linetype_table[i]);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmLinetype(m_linetype_table[%d]) failed.\n",i);
      }
    }
    if ( !archive.EndWrite3dmLinetypeTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmLinetypeTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }

  // LAYER TABLE
  ok = archive.BeginWrite3dmLayerTable();
  if ( !ok )
  {
    // make sure m_settings is valid
    if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmLayerTable() failed.\n");
    return false;
  }
  for( i = 0; ok && i < m_layer_table.Count(); i++ )
  {
    ok = archive.Write3dmLayer(m_layer_table[i]);
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmLayer(m_layer_table[%d]) failed.\n",i);
    }
  }
  if ( !archive.EndWrite3dmLayerTable() )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmLayerTable() failed.\n");
    return false;
  }
  if (!ok)
    return false;

  // GROUP TABLE
  ok = archive.BeginWrite3dmGroupTable();
  if ( !ok )
  {
    // make sure m_settings is valid
    if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmGroupTable() failed.\n");
    return false;
  }
  for( i = 0; ok && i < m_group_table.Count(); i++ )
  {
    ok = archive.Write3dmGroup(m_group_table[i]);
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmGroup(m_group_table[%d]) failed.\n",i);
    }
  }
  if ( !archive.EndWrite3dmGroupTable() )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmGroupTable() failed.\n");
    return false;
  }
  if (!ok)
    return false;


  // FONT TABLE
  if ( archive.Archive3dmVersion() >= 3 )
  {
    ok = archive.BeginWrite3dmFontTable();
    if ( !ok )
    {
      // make sure m_settings is valid
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmFontTable() failed.\n");
      return false;
    }
    for( i = 0; ok && i < m_font_table.Count(); i++ )
    {
      ok = archive.Write3dmFont(m_font_table[i]);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmFont(m_font_table[%d]) failed.\n",i);
      }
    }
    if ( !archive.EndWrite3dmFontTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmFontTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }


  // DIMSTYLE TABLE
  if ( archive.Archive3dmVersion() >= 3 )
  {
    ok = archive.BeginWrite3dmDimStyleTable();
    if ( !ok )
    {
      // make sure m_settings is valid
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmDimStyleTable() failed.\n");
      return false;
    }
    for( i = 0; ok && i < m_dimstyle_table.Count(); i++ )
    {
      ok = archive.Write3dmDimStyle(m_dimstyle_table[i]);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmDimStyle(m_dimstyle_table[%d]) failed.\n",i);
      }
    }
    if ( !archive.EndWrite3dmDimStyleTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmDimStyleTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }


  // LIGHT TABLE
  ok = archive.BeginWrite3dmLightTable();
  if ( !ok )
  {
    // make sure m_settings is valid
    if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmLightTable() failed.\n");
    return false;
  }
  for( i = 0; ok && i < m_light_table.Count(); i++ )
  {
    ok = archive.Write3dmLight(m_light_table[i].m_light,&m_light_table[i].m_attributes);
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmLight(m_light_table[%d]) failed.\n",i);
    }
  }
  if ( !archive.EndWrite3dmLightTable() )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmLightTable() failed.\n");
    return false;
  }
  if (!ok)
    return false;


  // HATCH PATTERN TABLE
  if ( archive.Archive3dmVersion() >= 4 )
  {
    ok = archive.BeginWrite3dmHatchPatternTable();
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmHatchPatternTable() failed.\n");
      return false;
    }
    for( i = 0; ok && i < m_hatch_pattern_table.Count(); i++ )
    {
      ok = archive.Write3dmHatchPattern(m_hatch_pattern_table[i]);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmHatchPattern(m_hatch_pattern_table[%d]) failed.\n",i);
      }
    }
    if ( !archive.EndWrite3dmHatchPatternTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmHatchPatternTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }


  // INSTANCE DEFINITION TABLE
  if ( archive.Archive3dmVersion() >= 3 )
  {
    ok = archive.BeginWrite3dmInstanceDefinitionTable();
    if ( !ok )
    {
      // make sure m_settings is valid
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmInstanceDefinitionTable() failed.\n");
      return false;
    }
    for( i = 0; ok && i < m_idef_table.Count(); i++ )
    {
      ok = archive.Write3dmInstanceDefinition(m_idef_table[i]);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmInstanceDefinition(m_IDef_table[%d]) failed.\n",i);
      }
    }
    if ( !archive.EndWrite3dmInstanceDefinitionTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmInstanceDefinitionTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }


  // OBJECT TABLE
  ok = archive.BeginWrite3dmObjectTable();
  if ( !ok )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmObjectTable() failed.\n");
    return false;
  }
  for( i = 0; ok && i < m_object_table.Count(); i++ )
  {
    if ( 0 != m_object_table[i].m_object )
    {
      ok = archive.Write3dmObject(*m_object_table[i].m_object,&m_object_table[i].m_attributes);
      if ( !ok )
      {
        if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmObject(m_IDef_table[%d]) failed.\n",i);
      }
    }
  }
  if ( !archive.EndWrite3dmObjectTable() )
  {
    if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmObjectTable() failed.\n");
    return false;
  }
  if (!ok)
    return false;


  // HISTORY RECORD TABLE
  if ( archive.Archive3dmVersion() >= 4 )
  {
    ok = archive.BeginWrite3dmHistoryRecordTable();
    if ( !ok )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.BeginWrite3dmHistoryRecordTable() failed.\n");
      return false;
    }
    for ( i = 0; ok && i < m_history_record_table.Count(); i++ )
    {
      const ON_HistoryRecord* history_record = m_history_record_table[i];
      if( history_record)
        ok = archive.Write3dmHistoryRecord( *history_record );
    }
    if( !archive.EndWrite3dmHistoryRecordTable() )
    {
      if ( error_log) error_log->Print("ONX_Model::Write archive.EndWrite3dmHistoryTable() failed.\n");
      return false;
    }
    if (!ok)
      return false;
  }

  // USER DATA TABLE
  for( i = 0; ok && i < m_userdata_table.Count(); i++ )
  {
    const ONX_Model_UserData& ud = m_userdata_table[i];
    if ( ON_UuidIsNotNil(ud.m_uuid) )
    {
      if ( !archive.Write3dmAnonymousUserTableRecord(
        ud.m_uuid,
        ud.m_usertable_3dm_version,
        ud.m_usertable_opennurbs_version,ud.m_goo
        ) )
      {
        continue;
      }
    }
  }

  if ( !archive.Write3dmEndMark() )
  {
    ok = false;
    if ( error_log) error_log->Print("ONX_Model::Write archive.Write3dmEndMark() failed.\n");
  }

  return ok;
}

bool ONX_Model::IsValid( ON_TextLog* text_log ) const
{
  // Audit with no repairs will simply complain if it
  // finds something wrong;
  int i = const_cast<ONX_Model*>(this)->Audit(false,NULL,text_log,NULL);
  return (i>=0);
}

int ONX_Model::ObjectIndex( ON_UUID object_uuid ) const
{
  // In a high quality app, you may want to override this
  // and do something a little smarter.

  int object_index = -1;
  if ( ON_UuidIsNotNil(object_uuid) )
  {
    int i, object_count = m_object_table.Count();
    if ( object_count > 0 )
    {
      if ( object_count != m_object_id_index.Count() )
      {
        // rebuild m__object_uuid_index[]
        ON_UuidIndexList* p = const_cast< ON_UuidIndexList* >(&m_object_id_index);
        p->Empty();
        p->Reserve(object_count);
        for ( i = 0; i < object_count; i++ )
        {
          ON_UUID id = m_object_table[i].m_attributes.m_uuid;
          if ( ON_UuidIsNil(id) )
          {
            ON_ERROR("Nil object ids in model");
            ON_CreateUuid(id);
            *(const_cast<ON_UUID*>(&m_object_table[i].m_attributes.m_uuid)) = id;
          }
          if ( !p->AddUuidIndex(id,i,true) )
          {
            ON_ERROR("Duplicate object ids in model");
            ON_CreateUuid(id);
            *(const_cast<ON_UUID*>(&m_object_table[i].m_attributes.m_uuid)) = id;
            p->AddUuidIndex(id,i,false);
          }
        }
      }

      if ( !m_object_id_index.FindUuid(object_uuid,&object_index) )
        object_index = -1;
    }
  }

  return object_index;
}

int ONX_Model::IDefIndex( ON_UUID idef_uuid ) const
{
  // In a high quality app, you may want to override this
  // and do something a little smarter.

  int idef_index = -1;
  if ( ON_UuidIsNotNil(idef_uuid) )
  {
    int i, idef_count = m_idef_table.Count();
    if ( idef_count > 0 )
    {
      if ( idef_count != m_idef_id_index.Count() )      
      {
        // rebuild m__idef_uuid_index[]
        ON_UuidIndexList* p = const_cast<ON_UuidIndexList*>(&m_idef_id_index);
        p->Empty();
        p->Reserve(idef_count);
        for ( i = 0; i < idef_count; i++ )
        {
          ON_UUID id = m_idef_table[i].m_uuid;
          if ( ON_UuidIsNil(id) )
          {
            ON_ERROR("Nil idef ids in model");
            ON_CreateUuid(id);
            (const_cast<ON_InstanceDefinition*>(&m_idef_table[i]))->m_uuid = id;
          }
          if ( !p->AddUuidIndex(id,i,true) )
          {
            ON_ERROR("Duplicate idef ids in model");
            ON_CreateUuid(id);
            (const_cast<ON_InstanceDefinition*>(&m_idef_table[i]))->m_uuid = id;
            p->AddUuidIndex(id,i,false);
          }
        }
      }

      if ( !m_idef_id_index.FindUuid(idef_uuid,&idef_index) )
        idef_index = -1;
    }
  }

  return idef_index;
}

int ONX_Model::IDefIndex( const wchar_t* idef_name ) const
{
  // slow and stupid search - what do you expect for free?
  // In a high quality app, you will want to override this
  // and do something a little smarter.
  //
  int idef_index = -1;
  if ( 0 != idef_name && 0 != idef_name[0]  )
  {
    int i, idef_count = m_idef_table.Count();
    for ( i = 0; i < idef_count; i++ )
    {
      if ( 0 == on_wcsicmp(idef_name, m_idef_table[i].Name() ) )
      {
        idef_index = i;
        break;
      }
    }
  }
  return idef_index;
}

void ONX_Model::GetUnusedIDefName( ON_wString& idef_name ) const
{
  int i = 1;
  for(i = 1; i < 100000; i++ )
  {
    idef_name.Format("IDef_%02d",i);
    if ( IDefIndex(idef_name) < 0 )
      return;
  }
  idef_name = "IDef";
  return;
}

int ONX_Model::UsesIDef(
        const ON_InstanceRef& iref,
        ON_UUID idef_uuid
        ) const
{
  // get id of idef we are looking for
  if ( ON_UuidIsNil(idef_uuid) )
    return 0;

  // id of idef that defines iref
  ON_UUID iref_idef_uuid = iref.m_instance_definition_uuid;
  if ( 0 == ON_UuidCompare( idef_uuid, iref_idef_uuid ) )
    return 1;

  const int iref_idef_index = IDefIndex(iref_idef_uuid);
  if ( -1 == iref_idef_index )
    return -1;
  const ON_InstanceDefinition& iref_idef = m_idef_table[iref_idef_index];


  int i0 = 0;
  int i1 = 0;
  int i, j, k, depth, obj_index;
  const ON_InstanceRef* pNestedIRef;

  // set iref_list[] = list of all nested instance references in iref_idef.
  ON_SimpleArray<const ON_InstanceRef*> iref_list(256);
  for ( j = 0; j < iref_idef.m_object_uuid.Count(); j++ )
  {
    obj_index = ObjectIndex(iref_idef.m_object_uuid[j]);
    if ( obj_index < 0 )
      continue;
    const ONX_Model_Object& obj = m_object_table[obj_index];
    if ( 0 == obj.m_object )
      continue;
    if ( obj.m_object->ObjectType() == ON::instance_reference )
    {
      pNestedIRef = ON_InstanceRef::Cast(obj.m_object);
      if ( 0 != pNestedIRef )
      {
        if ( 0 == ON_UuidCompare( idef_uuid, pNestedIRef->m_instance_definition_uuid ) )
          return 2;
        iref_list.Append(pNestedIRef);
      }
    }
  }

  // test the nested instance references to see if they use idef_index.
  const int max_depth = 1000;
  for ( depth=3; depth < max_depth && i1 < iref_list.Count(); depth++ )
  {
    i0 = i1;
    i1 = iref_list.Count();
    for ( i = i0; i < i1; i++ )
    {
      pNestedIRef = iref_list[i];
      if ( 0 == pNestedIRef )
        continue;
      k = IDefIndex( pNestedIRef->m_instance_definition_uuid );
      if ( k < 0 )
        continue;
      const ON_InstanceDefinition& nested_idef = m_idef_table[k];
      for ( j = 0; j < nested_idef.m_object_uuid.Count(); j++ )
      {
        obj_index = ObjectIndex(nested_idef.m_object_uuid[j]);
        if ( obj_index < 0 )
          continue;
        const ONX_Model_Object& obj = m_object_table[obj_index];
        if ( 0 == obj.m_object )
          continue;
        if ( obj.m_object->ObjectType() == ON::instance_reference )
        {
          pNestedIRef = ON_InstanceRef::Cast(obj.m_object);
          if ( 0 != pNestedIRef )
          {
            if ( 0 == ON_UuidCompare( idef_uuid, pNestedIRef->m_instance_definition_uuid ) )
              return depth;
            iref_list.Append(pNestedIRef);
          }
        }
      }
    }
  }

  return (depth < max_depth) ? 0 : -2;
}

int ONX_Model::LayerIndex( const wchar_t* layer_name ) const
{
  // slow and stupid search - what do you expect for free?
  // In a high quality app, you will want to override this
  // and do something a little smarter.

  int layer_index = -1;
  if ( 0 != layer_name && 0 != layer_name[0] )
  {
    int i, layer_count = m_layer_table.Count();
    for ( i = 0; i < layer_count; i++ )
    {
      if ( 0 == on_wcsicmp(layer_name, m_layer_table[i].LayerName() ) )
      {
        layer_index = i;
        break;
      }
    }
  }
  return layer_index;
}


void ONX_Model::GetUnusedLayerName( ON_wString& layer_name ) const
{
  int i = 1;
  for(i = 1; i < 100000; i++ )
  {
    layer_name.Format("Layer_%02d",i);
    if ( LayerIndex(layer_name) < 0 )
      return;
  }
  layer_name = "Layer";
  return;
}



bool ONX_Model::SetDocumentUserString( const wchar_t* key, const wchar_t* string_value )
{
  // This is a slow and stupid way to set a single string,
  // but I cannot modify the ONX_Model class to transparently
  // store document user string information until I can break
  // the public SDK in V6.
  bool rc = false;
  if ( 0 != key && 0 != key[0] )
  {
    ON_UUID doc_userstring_id = ON_DocumentUserStringList::m_ON_DocumentUserStringList_class_id.Uuid();
    for (int i = 0; i < m_userdata_table.Count(); i++ )
    {
      ONX_Model_UserData& ud = m_userdata_table[i];
      if ( ud.m_uuid == doc_userstring_id )
      {
        if ( TCODE_USER_RECORD == ud.m_goo.m_typecode && ud.m_goo.m_value != 0 )
        {
          ON_Read3dmBufferArchive ba( 
            (unsigned int)ud.m_goo.m_value,
            ud.m_goo.m_goo, 
            false,
            m_3dm_file_version,
            m_3dm_opennurbs_version
            );
          ON_Object* p = 0;
          if ( ba.ReadObject(&p) )
          {
            ON_DocumentUserStringList* sl = ON_DocumentUserStringList::Cast(p);
            if ( 0 != sl )
            {
              // modify the user string information
              rc = sl->SetUserString(key,string_value);
              if ( rc )
              {
                // write the new informtion to a memory buffer
                ON_Write3dmBufferArchive newgoo(ud.m_goo.m_value+1024,0,m_3dm_file_version,ON::Version());
                if (    newgoo.BeginWrite3dmUserTable(doc_userstring_id,false,0,0)
                     && newgoo.WriteObject(sl) 
                     && newgoo.EndWrite3dmUserTable()
                     )
                {
                  if (    newgoo.SizeOfArchive() > 0 
                       && newgoo.SizeOfArchive() <= 0xFFFFFFFF // max goo size if 4GB because we used an unsigned int back in the ice ages
                     )
                  {
                    // update the "goo"
                    unsigned char* goo = (unsigned char*)newgoo.HarvestBuffer();
                    unsigned int value = (unsigned int)newgoo.SizeOfArchive();
                    if ( 0 != goo && value > 0 )
                    {
                      onfree(ud.m_goo.m_goo); // delete old "goo"
                      ud.m_goo.m_value = (int)value;
                      ud.m_goo.m_goo = goo;
                    }
                  }
                }
              }
            }
          }
          if ( 0 != p )
          {
            delete p;
            p = 0;
          }
        }
        break;
      }
    }
  }
  return rc;
}


bool ONX_Model::GetDocumentUserString( const wchar_t* key, ON_wString& string_value ) const
{
  const wchar_t* s = 0;
  if ( 0 != key && 0 != key[0] )
  {
    // This is a slow and stupid way to get a single string,
    // but I cannot modify the ONX_Model class to transparently
    // store document user string information until I can break
    // the public SDK in V6.
    ON_ClassArray<ON_UserString> user_strings;
    GetDocumentUserStrings( user_strings );
    for ( int i = 0; i < user_strings.Count(); i++ )
    {
      if ( !user_strings[i].m_key.CompareNoCase(key) )
      {
        s = user_strings[i].m_string_value;
        break;
      }
    }
  }
  string_value = s;
  return (0 != s);
}


int ONX_Model::GetDocumentUserStrings( ON_ClassArray<ON_UserString>& user_strings ) const
{
  int rc = 0;
  // user strings are stored as ON_Object user strings on
  // an ON_DocumentUserStringList object in a user table 
  // with id = doc_userstring_id.
  ON_UUID doc_userstring_id = ON_DocumentUserStringList::m_ON_DocumentUserStringList_class_id.Uuid();
  for (int i = 0; i < m_userdata_table.Count(); i++ )
  {
    const ONX_Model_UserData& ud = m_userdata_table[i];
    if ( ud.m_uuid == doc_userstring_id )
    {
      if ( TCODE_USER_RECORD == ud.m_goo.m_typecode && ud.m_goo.m_value != 0 )
      {
        ON_Read3dmBufferArchive ba( 
          (unsigned int)ud.m_goo.m_value,
          ud.m_goo.m_goo, 
          false,
          m_3dm_file_version,
          m_3dm_opennurbs_version
          );

        ON_Object* p = 0;
        if ( ba.ReadObject(&p) )
        {
          const ON_DocumentUserStringList* sl = ON_DocumentUserStringList::Cast(p);
          if ( 0 != sl )
          {
            rc = sl->GetUserStrings(user_strings);
          }
        }
        if ( 0 != p )
        {
          delete p;
          p = 0;
        }
      }
      break;
    }
  }
  return rc;
}


static int AuditTextureMappingTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_mapping_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_TextureMapping& mapping = model.m_mapping_table[i];
    if ( mapping.m_mapping_index != i )
    {
      if ( text_log )
      {
        text_log->Print("m_mapping_table[%d].m_mapping_index == %d (should be %d)\n",
                        i,mapping.m_mapping_index,i);
      }
      if ( bAttemptRepair )
      {
        mapping.m_mapping_index = i;
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !mapping.IsValid(text_log) )
      return 1;
  }
  return 0;
}


static int AuditGroupTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_group_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Group& group = model.m_group_table[i];
    if ( group.m_group_index != i )
    {
      if ( text_log )
      {
        text_log->Print("m_group_table[%d].m_group_index == %d (should be %d)\n",
                        i,group.m_group_index,i);
      }
      if ( bAttemptRepair )
      {
        group.m_group_index = i;
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !group.IsValid(text_log) )
      return 1;
  }
  return 0;
}


static int AuditFontTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_font_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Font& font = model.m_font_table[i];
    if ( font.m_font_index != i )
    {
      if ( text_log )
      {
        text_log->Print("m_font_table[%d].m_font_index == %d (should be %d)\n",
                        i,font.m_font_index,i);
      }
      if ( bAttemptRepair )
      {
        font.m_font_index = i;
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !font.IsValid(text_log) )
      return 1;
  }
  return 0;
}


static int AuditDimStyleTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_dimstyle_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_DimStyle& dimstyle = model.m_dimstyle_table[i];
    if ( dimstyle.m_dimstyle_index != i )
    {
      if ( text_log )
      {
        text_log->Print("m_dimstyle_table[%d].m_dimstyle_index == %d (should be %d)\n",
                        i,dimstyle.m_dimstyle_index,i);
      }
      if ( bAttemptRepair )
      {
        dimstyle.m_dimstyle_index = i;
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !dimstyle.IsValid(text_log) )
      return 1;
  }
  return 0;
}


static int AuditHatchPatternTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_hatch_pattern_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_HatchPattern& hatchpattern = model.m_hatch_pattern_table[i];
    if ( hatchpattern.m_hatchpattern_index != i )
    {
      if ( text_log )
      {
        text_log->Print("m_hatch_pattern_table[%d].m_hatchpattern_index == %d (should be %d)\n",
                        i,hatchpattern.m_hatchpattern_index,i);
      }
      if ( bAttemptRepair )
      {
        hatchpattern.m_hatchpattern_index = i;
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !hatchpattern.IsValid(text_log) )
      return 1;
  }
  return 0;
}


static int AuditObjectAttributesHelper(
      ONX_Model& model,
      ON_3dmObjectAttributes& attributes,
      const char* parent_name,
      int parent_index,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int repcount = 0;
  int errcount = 0;

  // validate key attributes.
  int layer_index = attributes.m_layer_index;
  if ( layer_index < 0 || layer_index >= model.m_layer_table.Count() )
  {
    errcount++;
    if ( text_log )
    {
      text_log->Print(parent_name,parent_index);
      text_log->Print("m_layer_index = %d is not valid.",layer_index);
    }

    if ( bAttemptRepair )
    {
      layer_index = model.m_settings.m_current_layer_index;
      if ( layer_index < 0 || layer_index >= model.m_layer_table.Count() 
         )
      {
        layer_index = 0;
      }
      if ( layer_index >= 0 && layer_index < model.m_layer_table.Count() )
      {
        repcount++;
        attributes.m_layer_index = layer_index;
        if ( text_log )
          text_log->Print(" Repaired.");
      }
    }
    if ( text_log )
    {
      text_log->Print("\n");
    }
  }

  int linetype_index = attributes.m_linetype_index;
  if ( linetype_index < -1 || linetype_index >= model.m_linetype_table.Count() )
  {
    errcount++;
    if ( text_log )
    {
      text_log->Print(parent_name,parent_index);
      text_log->Print("m_linetype_index = %d is not valid.",linetype_index);
    }

    if ( bAttemptRepair )
    {
      linetype_index = -1;
      repcount++;
      attributes.m_linetype_index = linetype_index;
      if ( text_log )
        text_log->Print(" Repaired.");
    }
    if ( text_log )
    {
      text_log->Print("\n");
    }
  }

  int material_index = attributes.m_material_index;
  if ( material_index < -1 || material_index >= model.m_material_table.Count() )
  {
    errcount++;
    if ( text_log )
    {
      text_log->Print(parent_name,parent_index);
      text_log->Print("m_material_index = %d is not valid.",material_index);
    }

    if ( bAttemptRepair )
    {
      material_index = -1;
      repcount++;
      attributes.m_material_index = material_index;
      if ( text_log )
        text_log->Print(" Repaired.");
    }
    if ( text_log )
    {
      text_log->Print("\n");
    }
  }

  if ( repcount > 0 && repair_count )
    *repair_count = *repair_count + repcount;

  return (errcount>0) ? 9 : 0;
}



static int AuditLightTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_light_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ONX_Model_RenderLight& xlight = model.m_light_table[i];
    ON_Light& light = xlight.m_light;
    if ( light.m_light_index != i )
    {
      if ( text_log )
      {
        text_log->Print("m_light_table[%d].m_light_index == %d (should be %d)\n",
                        i,light.m_light_index,i);
      }
      if ( bAttemptRepair )
      {
        light.m_light_index = i;
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }

    int attrc = AuditObjectAttributesHelper(model,
      xlight.m_attributes,
      "m_light_table[%d].m_attributes",
      i,
      bAttemptRepair,
      repair_count,
      text_log 
      );

    if ( attrc && 0 == rc )
      rc = 9;

    if ( !light.IsValid(text_log) )
    {
      if ( 0 == rc )
        rc = 1;
    }

  }
  return rc;
}

static int AuditMaterialTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_material_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Material& mat = model.m_material_table[i];
    if ( mat.MaterialIndex() != i )
    {
      if ( text_log )
      {
        text_log->Print("m_material_table[%d].MaterialIndex() == %d (should be %d)\n",
                        i,mat.MaterialIndex(),i);
      }
      if ( bAttemptRepair )
      {
        mat.SetMaterialIndex(i);
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !mat.IsValid(text_log) )
      return 1;
  }
  return 0;
}

static int AuditLinetypeTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_linetype_table.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Linetype& lt = model.m_linetype_table[i];
    if ( lt.LinetypeIndex() != i )
    {
      if ( text_log )
      {
        text_log->Print("m_linetype_table[%d].LinetypeIndex() == %d (should be %d)\n",
                        i,lt.LinetypeIndex(),i);
      }
      if ( bAttemptRepair )
      {
        lt.SetLinetypeIndex(i);
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        return -1;
      }
    }
    if ( !lt.IsValid(text_log) )
      return 10;
  }
  return 0;
}

static int AuditLayerTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  if ( repair_count ) 
    *repair_count = 0;
  int i, count = model.m_layer_table.Count();
  if ( count == 0 && (model.m_object_table.Count()>0 || model.m_light_table.Count()>0))
    rc = 2;
  for ( i = 0; i < count; i++ )
  {
    ON_Layer& layer = model.m_layer_table[i];
    if ( layer.LayerIndex() != i )
    {
      if ( text_log )
      {
        text_log->Print("m_layer_table[%d].LayerIndex() == %d (should be %d)\n",
                        i,layer.LayerIndex(),i);
      }
      if ( bAttemptRepair )
      {
        layer.SetLayerIndex(i);
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else
      {
        rc = -1;
      }
    }

    const wchar_t* layer_name = layer.LayerName();

    if ( 0 == layer_name || 0 == layer_name[0] )
    {
      if ( text_log )
      {
        text_log->Print("m_layer_table[%d].LayerName() is empty\n",i);
      }
      if ( bAttemptRepair )
      {
        ON_wString name;
        model.GetUnusedLayerName( name );
        layer.SetLayerName( name );
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
        layer_name = layer.LayerName();
      }
      else if ( rc == 0 )
      {
        rc = 2;
      }
    }

    if ( !ONX_IsValidName(layer_name) )
    {
      if ( text_log )
      {
        text_log->Print("m_layer_table[%d].LayerName() is not valid\n",i);
      }
      if ( bAttemptRepair )
      {
        ON_wString name;
        model.GetUnusedLayerName( name );
        layer.SetLayerName( name );
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
        layer_name = layer.LayerName();
      }
      else if ( rc == 0 )
      {
        rc = 2;
      }
    }

    int j = model.LayerIndex(layer_name);
    if ( i != j )
    {
      if ( text_log )
      {
        text_log->Print("m_layer_table[%d] and m_layer_table[%d] have same layer name.\n",i,j);
      }
      if ( bAttemptRepair )
      {
        ON_wString name;
        model.GetUnusedLayerName( name );
        layer.SetLayerName( name );
        if ( text_log )
        {
          text_log->PushIndent();
          text_log->Print("Repaired.\n");
          text_log->PopIndent();
        }
        if ( repair_count )
          *repair_count += 1;
      }
      else if ( rc == 0 )
      {
        rc = 2;
      }
    }

    if ( rc == 0 && !layer.IsValid(text_log) )
      rc = 2;
  }
  return rc;
}

static int AuditIdsHelper(
      ON_SimpleArray<ON_UuidIndex>& id_list,
      ON_UuidIndexList* index_list,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log,
      const char* nil_id_msg,
      const char* dup_id_msg
      )
{
  int nil_count = 0;
  int dup_count = 0;
  int rep_count = 0;

  if ( index_list )
    index_list->Empty();
  const int count = id_list.Count();
  if ( count > 0 )
  {
    int i;
    ON_UUID id;

    // Make sure objects have non-nil ids
    for ( i = 0; i < count; i++ )
    {
      id_list[i].m_i = i;
      if ( ON_nil_uuid == id_list[i].m_id )
      {
        nil_count++;
        if ( text_log )
          text_log->Print(nil_id_msg,i);

        if ( bAttemptRepair )
        {
          id = ON_nil_uuid;
          if ( ON_CreateUuid(id) && !ON_UuidIsNil(id) )
          {
            id_list[i].m_id = id;
            rep_count++;
            if ( text_log )
              text_log->Print(" Repaired.");
          }
        }
        if ( text_log )
          text_log->Print("\n");
      }
    }

    if ( count > 1 )
    {
      // Make sure objects have unique ids
      id_list.QuickSort( ON_UuidIndex::CompareIdAndIndex );
      ON_UuidIndex id0 = id_list[0];
      for ( i = 1; i < count; i++ )
      {
        if ( ON_nil_uuid == id_list[i].m_id )
        {
          // nil ids were handled and counted above.
          // Either repair is false or we are not running
          // on Windows and the user supplied ON_CreateUuid
          // is returning nil ids.
          continue;
        }

        if ( id_list[i].m_id == id0.m_id )
        {
          dup_count++;
          if ( text_log )
            text_log->Print(dup_id_msg,id0.m_i,id_list[i].m_i);

          if ( bAttemptRepair )
          {
            // fix duplicate object id
            id = ON_nil_uuid;
            if ( ON_CreateUuid(id) && !ON_UuidIsNil(id) )
            {
              rep_count++;
              id_list[i].m_id = id;
              if ( text_log )
                text_log->Print(" Repaired.");
            }
          }
          if ( text_log )
            text_log->Print("\n");
        }
        else
        {
          id0 = id_list[i];
        }
      }
    }

    if ( index_list )
    {
      // rebuild index_list
      index_list->Reserve(count);
      for ( i = 0; i < count; i++ )
      {
        index_list->AddUuidIndex(id_list[i].m_id,id_list[i].m_i,false);
      }
    }
  }

  if ( repair_count )
    *repair_count = rep_count;

  return dup_count + nil_count;
}

static int AuditObjectIdsHelper(
      ONX_Model& model, 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  const int count = model.m_object_table.Count();
  model.m_object_id_index.Empty();
  if ( count > 0 )
  {
    int i;
    ON_SimpleArray<ON_UuidIndex> id_list(count);
    for ( i = 0; i < count; i++ )
    {
      id_list.AppendNew().m_id = model.m_object_table[i].m_attributes.m_uuid;
    }
    rc = AuditIdsHelper(
              id_list,
              &model.m_object_id_index,
              bAttemptRepair,
              repair_count,
              text_log,
              "m_object_table[%d].m_attributes.m_uuid is nil.",
              "m_object_table[%d] and [%d] have the same id."
              );
    if (rc && bAttemptRepair )
    {
      for ( i = 0; i < count; i++ )
      {
        model.m_object_table[id_list[i].m_i].m_attributes.m_uuid = id_list[i].m_id;
      }
    }
  }
  return rc;
}


static int AuditLightIdsHelper(
      ONX_Model& model, 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  int mismatch_count = 0;
  const int count = model.m_light_table.Count();
  if ( count > 0 )
  {
    int i;
    ON_SimpleArray<ON_UuidIndex> id_list(count);
    for ( i = 0; i < count; i++ )
    {
      ONX_Model_RenderLight& light = model.m_light_table[i];
      if ( ON_UuidCompare(light.m_light.m_light_id,light.m_attributes.m_uuid) )
      {
        mismatch_count++;
        if ( text_log )
        {
          text_log->Print("m_light_table[%d] light id and attributes id differ.",i);
        }
        if ( bAttemptRepair )
        {
          if ( repair_count )
            *repair_count = *repair_count + 1;
          if ( ON_nil_uuid == light.m_light.m_light_id )
            light.m_light.m_light_id = light.m_attributes.m_uuid;
          else
            light.m_attributes.m_uuid = light.m_light.m_light_id;
          if ( text_log )
            text_log->Print(" Repaired.");
        }
        if ( text_log )
          text_log->Print("\n");
      }

      if ( ON_nil_uuid == light.m_light.m_light_id )
      {
        id_list.AppendNew().m_id = light.m_attributes.m_uuid;
      }
      else 
      {
        id_list.AppendNew().m_id = light.m_light.m_light_id;
      }
    }

    rc = AuditIdsHelper(
              id_list,
              0, // no ONX_Model id index for lights
              bAttemptRepair,
              repair_count,
              text_log,
              "m_light_table[%d] light id is nil.",
              "m_light_table[%d] and[%d] have the same id."
              );

    rc += mismatch_count;

    if ( rc && bAttemptRepair )
    {
      for ( i = 0; i < count; i++ )
      {
        ONX_Model_RenderLight& light = model.m_light_table[id_list[i].m_i];
        light.m_light.m_light_id = id_list[i].m_id;
        light.m_attributes.m_uuid = light.m_light.m_light_id;
      }
    }

    // make sure light ids are not duplicated in object id list
    for ( i = 0; i < count; i++ )
    {
      ONX_Model_RenderLight& light = model.m_light_table[i];
      int oi = -1;
      if ( model.m_object_id_index.FindUuid(light.m_light.m_light_id,&oi) )
      {
        rc++;
        if ( text_log )
        {
          text_log->Print("m_light_table[%d] and m_object_table[%d] have same id.",
                          i,
                          oi
                          );
        }
        if ( bAttemptRepair )
        {
          ON_UuidIndex light_id;
          memset(&light_id,0,sizeof(light_id));
          light_id.m_id = ON_nil_uuid;
          light_id.m_i = -1;
          if ( ON_CreateUuid(light_id.m_id) && ON_nil_uuid != light_id.m_id )
          {
            if (    !model.m_object_id_index.FindUuid(light_id.m_id) 
                 && id_list.BinarySearch( &light_id, ON_UuidIndex::CompareId ) < 0 
               )
            {
              if ( repair_count )
                *repair_count = *repair_count + 1;
              light.m_light.m_light_id = light_id.m_id;
              light.m_attributes.m_uuid = light.m_light.m_light_id;
              if ( text_log )
                text_log->Print(" Repaired.");
            }
          }
        }
        if ( text_log )
          text_log->Print("\n");
      }
    }
  }

  return rc;
}

static int AuditIDefIdsHelper(
      ONX_Model& model, 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  const int count = model.m_idef_table.Count();
  model.m_idef_id_index.Empty();
  if ( count > 0 )
  {
    int i;
    ON_SimpleArray<ON_UuidIndex> id_list(count);
    for ( i = 0; i < count; i++ )
    {
      id_list.AppendNew().m_id = model.m_idef_table[i].m_uuid;
    }
    rc = AuditIdsHelper(
              id_list,
              &model.m_idef_id_index,
              bAttemptRepair,
              repair_count,
              text_log,
              "m_idef_table[%d].m_attributes.m_uuid is nil.",
              "m_idef_table[%d] and[%d] are the same."
              );
    if (rc && bAttemptRepair )
    {
      for ( i = 0; i < count; i++ )
      {
        model.m_idef_table[id_list[i].m_i].m_uuid = id_list[i].m_id;
      }
    }
  }
  return rc;
}

static int AuditMappingIdsHelper(
      ONX_Model& model, 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  const int count = model.m_mapping_table.Count();
  model.m_mapping_id_index.Empty();
  if ( count > 0 )
  {
    int i;
    ON_SimpleArray<ON_UuidIndex> id_list(count);
    for ( i = 0; i < count; i++ )
    {
      id_list.AppendNew().m_id = model.m_mapping_table[i].m_mapping_id;
    }
    rc = AuditIdsHelper(
              id_list,
              &model.m_mapping_id_index,
              bAttemptRepair,
              repair_count,
              text_log,
              "m_mapping_table[%d].m_mapping_id is nil.",
              "m_mapping_table[%d] and[%d] are the same."
              );
    if (rc && bAttemptRepair )
    {
      for ( i = 0; i < count; i++ )
      {
        model.m_mapping_table[id_list[i].m_i].m_mapping_id = id_list[i].m_id;
      }
    }
  }
  return rc;
}


static int AuditMaterialIdsHelper(
      ONX_Model& model, 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  const int count = model.m_material_table.Count();
  model.m_material_id_index.Empty();
  if ( count > 0 )
  {
    int i;
    ON_SimpleArray<ON_UuidIndex> id_list(count);
    for ( i = 0; i < count; i++ )
    {
      id_list.AppendNew().m_id = model.m_material_table[i].m_material_id;
    }
    rc = AuditIdsHelper(
              id_list,
              &model.m_material_id_index,
              bAttemptRepair,
              repair_count,
              text_log,
              "m_material_table[%d].m_material_id is nil.",
              "m_material_table[%d] and[%d] are the same."
              );
    if (rc && bAttemptRepair )
    {
      for ( i = 0; i < count; i++ )
      {
        model.m_material_table[id_list[i].m_i].m_material_id = id_list[i].m_id;
      }
    }
  }
  return rc;
}

static int AuditModelIdsHelper( 
      ONX_Model& model, 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log, 
      ON_SimpleArray<int>* warnings
      )
{
  int warning_count = 0;
  int i;

  // object ids
  i = AuditObjectIdsHelper(model,bAttemptRepair,repair_count,text_log);
  if (i < 0 ) 
    return i;
  if (i > 0 )
  {
    warning_count += i;
    if (warnings)
      warnings->Append(3);
  }

  // light ids
  i = AuditLightIdsHelper(model,bAttemptRepair,repair_count,text_log);
  if (i < 0 ) 
    return i;
  if (i > 0 )
  {
    warning_count += i;
    if (warnings)
      warnings->Append(15);
  }

  // idef ids
  i = AuditIDefIdsHelper(model,bAttemptRepair,repair_count,text_log);
  if (i < 0 ) 
    return i;
  if (i > 0 )
  {
    warning_count += i;
    if (warnings)
      warnings->Append(12);
  }

  // mapping ids
  i = AuditMappingIdsHelper(model,bAttemptRepair,repair_count,text_log);
  if (i < 0 ) 
    return i;
  if (i > 0 )
  {
    warning_count += i;
    if (warnings)
      warnings->Append(13);
  }

  // material ids
  i = AuditMaterialIdsHelper(model,bAttemptRepair,repair_count,text_log);
  if (i < 0 ) 
    return i;
  if (i > 0 )
  {
    warning_count += i;
    if (warnings)
      warnings->Append(13);
  }

  return warning_count;
}

static int AuditIDefTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  int rc = 0;
  int i, j, count = model.m_idef_table.Count();

  // AuditInstanceDefinitionIds() are already validated/repaired idef ids.

  // make sure idef names are empty or valid and unique
  for ( i = 0; i < count; i++ )
  {
    bool idef_ok = true;
    ON_InstanceDefinition& idef = model.m_idef_table[i];
    const wchar_t* idef_name = idef.Name();
    if ( 0 == idef_name )
      continue;

    if ( !ONX_IsValidName(idef_name) )
    {
      if ( text_log )
        text_log->Print("m_idef_table[%d].Name() = \"%ls\" is not valid.\n",i,idef_name);
      idef_ok = false;
    }
    else
    {
      j = model.IDefIndex( idef_name );
      if ( j != i )
      {
        if ( text_log )
          text_log->Print("m_idef_table[%d].Name() = m_idef_table[%d].Name().\n",i,j);
        idef_ok = false;
      }
    }
    if ( bAttemptRepair )
    {
      ON_wString new_name;
      model.GetUnusedIDefName(new_name);
      if ( ONX_IsValidName(new_name) && -1 == model.IDefIndex(new_name) )
      {
        idef.SetName(new_name);
        if ( repair_count )
          *repair_count = *repair_count + 1;
        if ( text_log )
          text_log->Print("Repaired.\n");
        idef_ok = true;
      }
    }
    if ( !idef_ok && rc == 0 )
    {
      rc = 5;
    }
  }

  for ( i = 0; i < count; i++ )
  {
    bool idef_ok = true;
    ON_InstanceDefinition& idef = model.m_idef_table[i];

    // make sure object uuid's are valid
    int k;
    j = 0;
    for ( j = k = 0; j < idef.m_object_uuid.Count(); k++ )
    {
      ON_UUID obj_uuid = idef.m_object_uuid[j];
      int obj_index = model.ObjectIndex(obj_uuid);
      if ( obj_index < 0 )
      {
        if ( text_log )
          text_log->Print("m_idef_table[%d].m_object_uuid[%d] is not an object's uuid.\n",i,k);
        if ( bAttemptRepair )
        {
          idef.m_object_uuid.Remove(j);
          if ( repair_count )
            *repair_count = *repair_count + 1;
          if ( text_log )
            text_log->Print("Repaired.\n");
        }
        else
        {
          j++;
          idef_ok = false;
        }
      }
      else
      {
        ONX_Model_Object& obj = model.m_object_table[obj_index];
        if ( ON::idef_object != obj.m_attributes.Mode() )
        {
          if ( text_log )
            text_log->Print("Object with uuid m_idef_table[%d].m_object_uuid[%d] does not have m_attributes.Mode()=ON::idef_object.\n",i,k);
          if ( bAttemptRepair )
          {
            idef.m_object_uuid.Remove(j);
            if ( repair_count )
              *repair_count = *repair_count + 1;
            if ( text_log )
              text_log->Print("Repaired.\n");
          }
          else
          {
            j++;
            idef_ok = false;
          }
        }
        else if ( 0 == obj.m_object )
        {
          if ( text_log )
            text_log->Print("Object with uuid m_idef_table[%d].m_object_uuid[%d] has NULL m_object.\n",i,k);
          if ( bAttemptRepair )
          {
            idef.m_object_uuid.Remove(j);
            if ( repair_count )
              *repair_count = *repair_count + 1;
            if ( text_log )
              text_log->Print("Repaired.\n");
          }
          else
          {
            j++;
            idef_ok = false;
          }
        }
        else if ( obj.m_object->ObjectType() == ON::instance_reference )
        {
          const ON_InstanceRef* pNestedRef = ON_InstanceRef::Cast(obj.m_object);
          if ( pNestedRef )
          {
            if ( model.UsesIDef( *pNestedRef, idef.Uuid() ) )
            {
              if ( text_log )
                text_log->Print("m_idef_table[%d].m_object_uuid[%d] is a circular reference.\n",i,k);
              if ( bAttemptRepair )
              {
                idef.m_object_uuid.Remove(j);
                if ( repair_count )
                  *repair_count = *repair_count + 1;
                if ( text_log )
                  text_log->Print("Repaired.\n");
              }
              else
              {
                j++;
                idef_ok = false;
                rc = -1; // circular reference
              }
            }
            else
              j++;
          }
          else
            j++;
        }
        else
          j++;
      }
    }
    if ( idef.m_object_uuid.Count() <= 0 )
    {
      if ( text_log )
        text_log->Print("m_idef_table[%d].m_object_uuid.Count() = 0.\n",i);
      idef_ok = false;
    }
    if ( !idef_ok && rc == 0 )
      rc = 6;

  }

  return rc;
}


static int AuditObjectTableHelper( 
      ONX_Model& model,
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log 
      )
{
  // AuditObjectIds() are already validated/repaired object ids.

  int rc = 0;
  int i, count = model.m_object_table.Count();

  for ( i = 0; i < count; i++ )
  {
    ONX_Model_Object& obj = model.m_object_table[i];
    if ( 0 == obj.m_object )
    {
      rc = 7;
      if ( text_log )
        text_log->Print("m_object_table[%d].m_object is NULL.\n",i);
    }
    else if ( false == obj.m_object->IsValid(NULL) )
    {
      rc = 8;
      if ( text_log )
      {
        text_log->Print("m_object_table[%d].m_object->IsValid() = false.\n",i);
        text_log->PushIndent();
        text_log->PushIndent();
        obj.m_object->IsValid(text_log);
        text_log->PopIndent();
        text_log->PopIndent();
      }
    }

    int attrc = AuditObjectAttributesHelper(model,
                                obj.m_attributes,
                                "m_object_table[%d].m_attributes",
                                i,
                                bAttemptRepair,
                                repair_count,
                                text_log
                                );
    if ( 0 == rc && attrc )
      rc = attrc;
  }

  return rc;
}

static int AuditHistoryRecordTableHelper( 
      ONX_Model&,
      bool,
      int*,
      ON_TextLog*
      )
{
  // TODO - make sure object id's exist
  return 0;
}

int ONX_Model::Audit( 
      bool bAttemptRepair,
      int* repair_count,
      ON_TextLog* text_log,
      ON_SimpleArray<int>* warnings
      )
{
  int warning_count = 0;
  int rc = 0, i;
  int repcnt;
  if ( repair_count ) 
    *repair_count = 0;

  repcnt = 0;
  i = AuditModelIdsHelper( *this, bAttemptRepair, &repcnt, text_log, warnings );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  warning_count += i;

  repcnt = 0;
  i = AuditTextureMappingTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditMaterialTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditLinetypeTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditLayerTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditGroupTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditFontTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditDimStyleTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditLightTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditHatchPatternTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditIDefTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditObjectTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  repcnt = 0;
  i = AuditHistoryRecordTableHelper( *this, bAttemptRepair, &repcnt, text_log );
  if ( repair_count ) 
    *repair_count += repcnt;
  if ( i < 0 )
    return i;
  if ( 0 == rc && 0 != i )
    rc = i;

  return warning_count;
}

static const ON_UnknownUserData* RDKObjectUserDataHelper(const ON_UserData* objectud)
{
  // CRhRdkUserData object id: AFA82772-1525-43dd-A63C-C84AC5806911
  // CRhRdkUserData::m_userdata_uuid = B63ED079-CF67-416c-800D-22023AE1BE21

  // CRhRdkUserData object id
  // {AFA82772-1525-43dd-A63C-C84AC5806911}
  static const ON_UUID CRhRdkUserData_object_id = 
  { 0xAFA82772, 0x1525, 0x43dd, { 0xA6, 0x3C, 0xC8, 0x4A, 0xC5, 0x80, 0x69, 0x11 } };

  // CRhRdkUserData::m_userdata_uuid
  // {B63ED079-CF67-416c-800D-22023AE1BE21}
  static const ON_UUID CRhRdkUserData_userdata_uuid = 
  { 0xB63ED079, 0xCF67, 0x416c, { 0x80, 0x0D, 0x22, 0x02, 0x3A, 0xE1, 0xBE, 0x21 } };
  
  const ON_UnknownUserData* unknown_ud = ON_UnknownUserData::Cast(objectud);
  
  bool rc = ( 0 != unknown_ud 
              && unknown_ud->m_sizeof_buffer > 0
              && 0 != unknown_ud->m_buffer
              && 0 == ON_UuidCompare(CRhRdkUserData_object_id,unknown_ud->m_unknownclass_uuid)
              && 0 == ON_UuidCompare(CRhRdkUserData_userdata_uuid,unknown_ud->m_userdata_uuid)
            );
  return rc ? unknown_ud : 0;
}

bool ONX_Model::IsRDKObjectInformation(const ON_UserData& objectud)
{
  return 0 != RDKObjectUserDataHelper(&objectud);
}

bool ONX_Model::GetRDKObjectInformation(const ON_Object& object,ON_wString& rdk_xml_object_data)
{
  rdk_xml_object_data.SetLength(0);
  const ON_UnknownUserData* unknown_ud = 0;
  const ON_UserData* ud = ON_UserData::Cast(&object);
  if ( 0 != ud )
  {
    unknown_ud = RDKObjectUserDataHelper(ud);
  }
  else
  {
    for ( ud = object.FirstUserData(); 0 != ud && 0 == unknown_ud; ud = ud->Next() )
    {
      unknown_ud = RDKObjectUserDataHelper(ud);
    }
  }

  if ( 0 == unknown_ud )
    return false;

  ON_Read3dmBufferArchive a(unknown_ud->m_sizeof_buffer,unknown_ud->m_buffer,false,unknown_ud->m_3dm_version,unknown_ud->m_3dm_opennurbs_version);
  int version = 0;
  if (!a.ReadInt(&version) )
    return false;
  
  if ( 1 == version )
  {
    if ( !a.ReadString(rdk_xml_object_data) )
      return false;
  }
  else if ( 2 == version )
  {
    // UTF8 string
    ON_SimpleArray< char > s;
    int slen = 0;
    if ( !a.ReadInt(&slen) )
      return false;
    if ( slen <= 0 )
      return false;
    if ( slen + 4 > unknown_ud->m_sizeof_buffer )
      return false;
    s.Reserve(slen+1);
    s.SetCount(slen+1);
    s[slen] = 0;
    if ( !a.ReadChar(slen,s.Array() ) ) 
      return false;
    const char* sArray = s.Array();
    if ( 0 != sArray && 0 != sArray[0] )
    {
      unsigned int error_status = 0;
      int wLen = ON_ConvertUTF8ToWideChar(sArray,-1,0,0,&error_status,0,0,0);
      if ( wLen > 0 && 0 == error_status )
      {
        rdk_xml_object_data.SetLength(wLen+2);
        wLen = ON_ConvertUTF8ToWideChar(sArray,-1,rdk_xml_object_data.Array(),wLen+1,&error_status,0,0,0);
        if ( wLen > 0 && 0 == error_status )
          rdk_xml_object_data.SetLength(wLen);
        else
          rdk_xml_object_data.SetLength(0);
      }
      if ( 0 != error_status )
      {
        ON_ERROR("RDK xml object information is not a valid UTF-8 string.");
      }
    }
  }

  return rdk_xml_object_data.Length() > 0;
}

bool ONX_Model::IsRDKDocumentInformation(const ONX_Model_UserData& docud)
{
  // {16592D58-4A2F-401D-BF5E-3B87741C1B1B}
  static const ON_UUID rdk_plugin_id = 
  { 0x16592D58, 0x4A2F, 0x401D, { 0xBF, 0x5E, 0x3B, 0x87, 0x74, 0x1C, 0x1B, 0x1B } };

  return ( 0 == ON_UuidCompare(rdk_plugin_id,docud.m_uuid) && docud.m_goo.m_value >= 4 && 0 != docud.m_goo.m_goo );
}


bool ONX_Model::GetRDKDocumentInformation(const ONX_Model_UserData& docud,ON_wString& rdk_xml_document_data)
{
  if ( !ONX_Model::IsRDKDocumentInformation(docud) )
    return false;

  ON_Read3dmBufferArchive a(docud.m_goo.m_value,docud.m_goo.m_goo,false,docud.m_usertable_3dm_version,docud.m_usertable_opennurbs_version);

  int version = 0;
  if (!a.ReadInt(&version) )
    return false;
  
  if ( 1 == version )
  {
    // UTF-16 string
    if ( !a.ReadString(rdk_xml_document_data) )
      return false;
  }
  else if ( 3 == version )
  {
    // UTF-8 string
    int slen = 0;
    if ( !a.ReadInt(&slen) )
      return 0;
    if ( slen <= 0 )
      return 0;
    if ( slen + 4 > docud.m_goo.m_value )
      return 0;
    ON_String s;
    s.SetLength(slen);
    if ( !a.ReadChar(slen,s.Array()) )
      return 0;
    const char* sArray = s.Array();
    if ( 0 != sArray && 0 != sArray[0] )
    {
      unsigned int error_status = 0;
      int wLen = ON_ConvertUTF8ToWideChar(sArray,-1,0,0,&error_status,0,0,0);
      if ( wLen > 0 && 0 == error_status )
      {
        rdk_xml_document_data.SetLength(wLen+2);
        wLen = ON_ConvertUTF8ToWideChar(sArray,-1,rdk_xml_document_data.Array(),wLen+1,&error_status,0,0,0);
        if ( wLen > 0 && 0 == error_status )
          rdk_xml_document_data.SetLength(wLen);
        else
        {
          rdk_xml_document_data.SetLength(0);
        }
      }
      if ( 0 != error_status )
      {
        ON_ERROR("RDK xml document settings is not a valid UTF-8 string.");
      }
    }
  }

  return rdk_xml_document_data.Length() > 0;
}

#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif

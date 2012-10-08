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

ON_COMPONENT_INDEX::ON_COMPONENT_INDEX() 
                   : m_type(ON_COMPONENT_INDEX::invalid_type),
                     m_index(-1)
{
}

ON_COMPONENT_INDEX::ON_COMPONENT_INDEX(
                                       ON_COMPONENT_INDEX::TYPE type,
                                       int index
                                       ) 
                   : m_type(type),
                     m_index(index)
{
}

ON_COMPONENT_INDEX::TYPE ON_COMPONENT_INDEX::Type(int i)
{
  TYPE t = invalid_type;
  switch((unsigned int)i)
  {
  case no_type:            t = no_type;            break;
  case brep_vertex:        t = brep_vertex;        break;
  case brep_edge:          t = brep_edge;          break;
  case brep_face:          t = brep_face;          break;
  case brep_trim:          t = brep_trim;          break;
  case brep_loop:          t = brep_loop;          break;
  case mesh_vertex:        t = mesh_vertex;        break;
  case meshtop_vertex:     t = meshtop_vertex;     break;
  case meshtop_edge:       t = meshtop_edge;       break;
  case mesh_face:          t = mesh_face;          break;
  case idef_part:          t = idef_part;          break;
  case polycurve_segment:  t = polycurve_segment;  break;
  case pointcloud_point:   t = pointcloud_point;   break;
  case group_member:       t = group_member;       break;

  case extrusion_bottom_profile: t = extrusion_bottom_profile; break;
  case extrusion_top_profile:    t = extrusion_top_profile;    break;
  case extrusion_wall_edge:      t = extrusion_wall_edge;      break;
  case extrusion_wall_surface:   t = extrusion_wall_surface;   break;
  case extrusion_cap_surface:    t = extrusion_cap_surface;    break;
  case extrusion_path:           t = extrusion_path;           break;

  case dim_linear_point:   t = dim_linear_point;   break;
  case dim_radial_point:   t = dim_radial_point;   break;
  case dim_angular_point:  t = dim_angular_point;  break;
  case dim_ordinate_point: t = dim_ordinate_point; break;
  case dim_text_point:     t = dim_text_point;     break;
  }
  return t;
}


void ON_COMPONENT_INDEX::Set(
                                       ON_COMPONENT_INDEX::TYPE type,
                                       int index
                                       ) 
{
  m_type = type;
  m_index = index;
}

void ON_COMPONENT_INDEX::UnSet()
{
  m_type = ON_COMPONENT_INDEX::invalid_type;
  m_index = -1;
}

bool ON_COMPONENT_INDEX::IsMeshComponentIndex() const
{
  bool rc = false;
  switch(m_type)
  {
  case ON_COMPONENT_INDEX::mesh_vertex:
  case ON_COMPONENT_INDEX::meshtop_vertex:
  case ON_COMPONENT_INDEX::meshtop_edge:
  case ON_COMPONENT_INDEX::mesh_face:
    if ( m_index >= 0 )
    {
      rc = true;
    }
    break;
  default:
    // intentionally skipping other ON_COMPONENT_INDEX::TYPE enum values
    break;
  }
  return rc;
}

bool ON_COMPONENT_INDEX::IsAnnotationComponentIndex() const
{
  bool rc = false;
  switch(m_type)
  {
  case ON_COMPONENT_INDEX::dim_linear_point:
  case ON_COMPONENT_INDEX::dim_radial_point:
  case ON_COMPONENT_INDEX::dim_angular_point:
  case ON_COMPONENT_INDEX::dim_ordinate_point:
  case ON_COMPONENT_INDEX::dim_text_point:
    if ( m_index >= 0 )
    {
      rc = true;
    }
    break;
  default:
    // intentionally skipping other ON_COMPONENT_INDEX::TYPE enum values
    break;
  }
  return rc;
}

bool ON_COMPONENT_INDEX::IsBrepComponentIndex() const
{
  bool rc = false;
  switch(m_type)
  {
  case ON_COMPONENT_INDEX::brep_vertex:
  case ON_COMPONENT_INDEX::brep_trim:
  case ON_COMPONENT_INDEX::brep_loop:
  case ON_COMPONENT_INDEX::brep_edge:
  case ON_COMPONENT_INDEX::brep_face:
    if ( m_index >= 0 )
    {
      rc = true;
    }
    break;
  default:
    // intentionally skipping other ON_COMPONENT_INDEX::TYPE enum values
    break;
  }
  return rc;
}

bool  ON_COMPONENT_INDEX::IsIDefComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::idef_part == m_type && m_index >= 0 );
}

bool  ON_COMPONENT_INDEX::IsPolyCurveComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::polycurve_segment == m_type && m_index >= 0 );
}

bool  ON_COMPONENT_INDEX::IsGroupMemberComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::group_member == m_type && m_index >= 0 );
}

bool  ON_COMPONENT_INDEX::IsExtrusionProfileComponentIndex() const
{
  return ( (   ON_COMPONENT_INDEX::extrusion_bottom_profile  == m_type 
            || ON_COMPONENT_INDEX::extrusion_top_profile     == m_type
            )
            && m_index >= 0 
         );
}

bool  ON_COMPONENT_INDEX::IsExtrusionPathComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::extrusion_path  == m_type 
           && m_index >= -1 
           && m_index <= 1
         );
}

bool  ON_COMPONENT_INDEX::IsExtrusionWallEdgeComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::extrusion_wall_edge  == m_type 
           && m_index >= 0 
         );
}

bool  ON_COMPONENT_INDEX::IsExtrusionWallSurfaceComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::extrusion_wall_surface  == m_type 
           && m_index >= 0
         );
}

bool  ON_COMPONENT_INDEX::IsExtrusionWallComponentIndex() const
{
  return ( (   ON_COMPONENT_INDEX::extrusion_wall_edge == m_type 
             || ON_COMPONENT_INDEX::extrusion_wall_surface == m_type
           )
           && m_index >= 0
         );
}

bool  ON_COMPONENT_INDEX::IsExtrusionComponentIndex() const
{
  return ( (   ON_COMPONENT_INDEX::extrusion_bottom_profile  == m_type 
            || ON_COMPONENT_INDEX::extrusion_top_profile     == m_type
            || ON_COMPONENT_INDEX::extrusion_wall_edge       == m_type
            || ON_COMPONENT_INDEX::extrusion_wall_surface    == m_type
            || ON_COMPONENT_INDEX::extrusion_cap_surface     == m_type
            || ON_COMPONENT_INDEX::extrusion_path            == m_type
            )
            && 
            (  m_index >= 0 
              || (-1 == m_index && ON_COMPONENT_INDEX::extrusion_path == m_type)
            )
         );
}

bool  ON_COMPONENT_INDEX::IsPointCloudComponentIndex() const
{
  return ( ON_COMPONENT_INDEX::pointcloud_point == m_type && m_index >= 0 );
}

bool ON_COMPONENT_INDEX::IsSet() const
{
  bool rc = false;
  switch(m_type)
  {
  case invalid_type:
    rc = false;
    break;

  case no_type:
    rc = false;
    break;

  case brep_vertex:
  case brep_edge:
  case brep_face:
  case brep_trim:
  case brep_loop:
  case mesh_vertex:
  case meshtop_vertex:
  case meshtop_edge:
  case mesh_face:
  case idef_part:
  case polycurve_segment:
  case pointcloud_point:
  case group_member:
    rc = (m_index != -1);
    break;

  default:
    rc = false;
    break;
  }
  return rc;
}


int ON_COMPONENT_INDEX::Compare( const ON_COMPONENT_INDEX* a, const ON_COMPONENT_INDEX* b )
{
  int i = ((int)a->m_type) - ((int)b->m_type);
  if ( 0 == i )
  {
    i = a->m_index - b->m_index;
  }
  return i;
}

bool ON_COMPONENT_INDEX::operator==(const ON_COMPONENT_INDEX& other) const
{
  return (m_type == other.m_type && m_index == other.m_index);
}

bool ON_COMPONENT_INDEX::operator!=(const ON_COMPONENT_INDEX& other) const
{
  return (m_type != other.m_type || m_index != other.m_index);
}

bool ON_COMPONENT_INDEX::operator<(const ON_COMPONENT_INDEX& other) const
{
  return (ON_COMPONENT_INDEX::Compare(this,&other) < 0);
}

bool ON_COMPONENT_INDEX::operator<=(const ON_COMPONENT_INDEX& other) const
{
  return (ON_COMPONENT_INDEX::Compare(this,&other) <= 0);
}

bool ON_COMPONENT_INDEX::operator>(const ON_COMPONENT_INDEX& other) const
{
  return (ON_COMPONENT_INDEX::Compare(this,&other) > 0);
}

bool ON_COMPONENT_INDEX::operator>=(const ON_COMPONENT_INDEX& other) const
{
  return (ON_COMPONENT_INDEX::Compare(this,&other) >= 0);
}

ON_ObjRefEvaluationParameter::ON_ObjRefEvaluationParameter()
: m_t_type(0)
, m_reserved(0)
{
  m_t[0] = ON_UNSET_VALUE;
  m_t[1] = ON_UNSET_VALUE;
  m_t[2] = ON_UNSET_VALUE;
  m_t[3] = ON_UNSET_VALUE;
}

void ON_ObjRefEvaluationParameter::Default()
{
  ON_ObjRefEvaluationParameter d;
  *this = d;
}

ON_ObjRefEvaluationParameter::~ON_ObjRefEvaluationParameter()
{
}

bool ON_ObjRefEvaluationParameter::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return rc;

  for(;;)
  {
    rc = archive.WriteInt(m_t_type);
    if (!rc) break;

    rc = archive.WriteComponentIndex(m_t_ci);
    if (!rc) break;

    rc = archive.WriteDouble(4,m_t);
    if (!rc) break;

    rc = archive.WriteInterval(m_s[0]);
    if (!rc) break;

    rc = archive.WriteInterval(m_s[1]);
    if (!rc) break;

    rc = archive.WriteInterval(m_s[2]);
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

bool ON_ObjRefEvaluationParameter::Read( ON_BinaryArchive& archive )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return rc;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = archive.ReadInt(&m_t_type);
    if (!rc) break;

    rc = archive.ReadComponentIndex(m_t_ci);
    if (!rc) break;

    rc = archive.ReadDouble(4,m_t);
    if (!rc) break;

    rc = archive.ReadInterval(m_s[0]);
    if (!rc) break;

    rc = archive.ReadInterval(m_s[1]);
    if (!rc) break;

    rc = archive.ReadInterval(m_s[2]);
    if (!rc) break;

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

ON_ObjRef::ON_ObjRef() 
          : m_uuid(ON_nil_uuid),
            m_geometry(0),
            m_parent_geometry(0),
            m_geometry_type(ON::unknown_object_type),
            m_runtime_sn(0),
            m_point(ON_UNSET_POINT),
            m_osnap_mode(ON::os_none),
            m__proxy1(0),
            m__proxy2(0),
            m__proxy_ref_count(0)
{
}

void ON_ObjRef::Destroy()
{
  DecrementProxyReferenceCount();
  m_uuid = ON_nil_uuid;
  m_geometry = 0;
  m_parent_geometry = 0;
  m_geometry_type = ON::unknown_object_type;
  m_runtime_sn = 0;
  m_point = ON_UNSET_POINT;
  m_osnap_mode = ON::os_none;
  m__proxy1 = 0;
  m__proxy2 = 0;
  m__proxy_ref_count = 0;
}


ON_ObjRef::ON_ObjRef( const ON_ObjRef& src ) 
          : m_uuid(src.m_uuid),
            m_geometry(src.m_geometry),
            m_parent_geometry(src.m_parent_geometry),
            m_component_index(src.m_component_index),
            m_geometry_type(src.m_geometry_type),
            m_runtime_sn(src.m_runtime_sn),
            m_point(src.m_point),
            m_osnap_mode(src.m_osnap_mode),
            m_evp(src.m_evp),
            m__iref(src.m__iref),
            m__proxy1(src.m__proxy1),
            m__proxy2(src.m__proxy2),
            m__proxy_ref_count(src.m__proxy_ref_count)
{
  if ( m__proxy_ref_count && *m__proxy_ref_count > 0 )
  {
    *m__proxy_ref_count = *m__proxy_ref_count + 1;
  }
}

ON_ObjRef& ON_ObjRef::operator=( const ON_ObjRef& src ) 
{
  if ( this != &src )
  {
    // Remove any reference this ON_ObjRef class 
    // may currently have.
    DecrementProxyReferenceCount();

    // copy the values from src
    m_uuid = src.m_uuid;
    m_geometry = src.m_geometry;
    m_parent_geometry = src.m_parent_geometry;
    m_component_index = src.m_component_index;
    m_geometry_type = src.m_geometry_type;
    m_runtime_sn = src.m_runtime_sn;
    m_point = src.m_point;
    m_osnap_mode = src.m_osnap_mode;
    m_evp = src.m_evp;
    m__iref = src.m__iref;
    m__proxy1 = src.m__proxy1;
    m__proxy2 = src.m__proxy2;
    m__proxy_ref_count = src.m__proxy_ref_count;

    if ( m__proxy_ref_count && *m__proxy_ref_count > 0 )
    {
      *m__proxy_ref_count = *m__proxy_ref_count + 1;
    }
  }

  return *this;
}

bool ON_ObjRef_IRefID::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 1 );
  if ( !rc )
    return false;

  for(;;)
  {
    rc = archive.WriteUuid(m_iref_uuid);
    if (!rc) break;

    rc = archive.WriteXform(m_iref_xform);
    if (!rc) break;

    rc = archive.WriteUuid(m_idef_uuid);
    if (!rc) break;

    rc = archive.WriteInt(m_idef_geometry_index);
    if (!rc) break;

    // 13 July 2006 - 1.1 - added m_component_index and m_evp
    rc = archive.WriteComponentIndex(m_component_index);
    if (!rc) break;

    rc = m_evp.Write(archive);
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

bool ON_ObjRef_IRefID::Read( ON_BinaryArchive& archive )
{
  Default();

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( 
                          TCODE_ANONYMOUS_CHUNK, 
                          &major_version, 
                          &minor_version );
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = archive.ReadUuid(m_iref_uuid);
    if (!rc) break;

    rc = archive.ReadXform(m_iref_xform);
    if (!rc) break;

    rc = archive.ReadUuid(m_idef_uuid);
    if (!rc) break;

    rc = archive.ReadInt(&m_idef_geometry_index);
    if (!rc) break;

    if ( minor_version >= 1 )
    {
      // 13 July 2006 - 1.1 - added m_component_index and m_evp
      rc = archive.ReadComponentIndex(m_component_index);
      if (!rc) break;

      rc = m_evp.Read(archive);
      if (!rc) break;
    }

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}


bool ON_ObjRef::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 2 );
  if ( !rc )
    return false;

  for(;;)
  {
    rc = archive.WriteUuid(m_uuid);
    if (!rc) break;

    rc = archive.WriteComponentIndex(m_component_index);
    if (!rc) break;

    rc = archive.WriteInt(m_geometry_type);
    if (!rc) break;

    // Do not save the value of m_runtime_sn in the
    // archive.  When the file is read in, the object
    // will have a different value of m_runtime_sn.

    rc = archive.WritePoint(m_point);
    if (!rc) break;

    // Prior to 13 July 2006, the evaluation parameters
    // m_evp were members of ON_ObjRef.  That's why the
    // m_evp fields are written directly rather than
    // using m_evp.Write().
    rc = archive.WriteInt(m_evp.m_t_type);
    if (!rc) break;

    rc = archive.WriteComponentIndex(m_evp.m_t_ci);
    if (!rc) break;

    rc = archive.WriteDouble(4,m_evp.m_t);
    if (!rc) break;

    rc = archive.WriteArray(m__iref);
    if (!rc) break;

    // 1.1 IO fields
    rc = archive.WriteInterval(m_evp.m_s[0]);
    if (!rc) break;

    rc = archive.WriteInterval(m_evp.m_s[1]);
    if (!rc) break;

    // 1.2 IO fields
    rc = archive.WriteInterval(m_evp.m_s[2]);
    if (!rc) break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

bool ON_ObjRef::Read( ON_BinaryArchive& archive )
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if ( !rc )
    return false;

  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = archive.ReadUuid(m_uuid);
    if (!rc) break;

    rc = archive.ReadComponentIndex(m_component_index);
    if (!rc) break;

    rc = archive.ReadInt(&m_geometry_type);
    if (!rc) break;

    rc = archive.ReadPoint(m_point);
    if (!rc) break;

    // Prior to 13 July 2006, the evaluation parameters
    // m_evp were members of ON_ObjRef.  That's why the
    // m_evp fields are read directly rather than
    // using m_evp.Read().
    rc = archive.ReadInt(&m_evp.m_t_type);
    if (!rc) break;

    rc = archive.ReadComponentIndex(m_evp.m_t_ci);
    if (!rc) break;

    rc = archive.ReadDouble(4,m_evp.m_t);
    if (!rc) break;

    rc = archive.ReadArray(m__iref);
    if (!rc) break;

    if ( minor_version >= 1 )
    {
      // 1.1 IO fields
      rc = archive.ReadInterval(m_evp.m_s[0]);
      if (!rc) break;
      rc = archive.ReadInterval(m_evp.m_s[1]);
      if (!rc) break;
      if ( minor_version >= 2 )
      {
        rc = archive.ReadInterval(m_evp.m_s[2]);
        if (!rc) break;
      }
    }

    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}


ON_ObjRef::~ON_ObjRef()
{
  DecrementProxyReferenceCount();
}


void ON_ObjRef::RemapObjectId( const ON_SimpleArray<ON_UuidPair>& id_remap )
{
  // The cast is a lie but it works because ON_UuidPair::CompareFirstUuid
  // looks for an id in the first 16 bytes of the ON_UuidPair.
  int i = id_remap.BinarySearch((const ON_UuidPair*)&m_uuid,ON_UuidPair::CompareFirstUuid);
  if ( i >= 0 )
    m_uuid = id_remap[i].m_uuid[1];
}

int ON_ObjRef::ProxyReferenceCount() const
{
  return m__proxy_ref_count ? *m__proxy_ref_count : 0;
}

const ON_Brep* ON_BrepParent( const ON_Geometry* geo )
{
  const ON_Brep* brep = 0;

  if ( geo == NULL )
    return NULL;

  if  ( ON::brep_object == geo->ObjectType() )
  {
    brep = ON_Brep::Cast(geo);
  }
  else
  {
    // ComponentIndex() is the fastest way
    switch( geo->ComponentIndex().m_type )
    {
    case ON_COMPONENT_INDEX::brep_edge:
      {
        const ON_BrepEdge* edge = ON_BrepEdge::Cast(geo);
        if ( edge )
          brep = edge->Brep();
      }
      break;

    case ON_COMPONENT_INDEX::brep_face:
      {
        const ON_BrepFace* face = ON_BrepFace::Cast(geo);
        if ( face )
          brep = face->Brep();
      }
      break;

    case ON_COMPONENT_INDEX::brep_trim:
      {
        const ON_BrepTrim* trim = ON_BrepTrim::Cast(geo);
        if ( trim )
          brep = trim->Brep();
      }
      break;

    case ON_COMPONENT_INDEX::brep_loop:
      {
        const ON_BrepLoop* loop = ON_BrepLoop::Cast(geo);
        if ( loop )
          brep = loop->Brep();
      }
      break;

    default:
      // intentionally skipping other ON_COMPONENT_INDEX::TYPE enum values
      break;
    }
  }

  return brep;
}


const ON_Mesh* ON_MeshParent( const ON_Geometry* geo )
{
  const ON_Mesh* mesh = 0;

  if ( geo == NULL )
    return NULL;

  if  ( ON::mesh_object == geo->ObjectType() )
  {
    mesh = ON_Mesh::Cast(geo);
  }
  else
  {
    // ComponentIndex() is the fastest way
    switch( geo->ComponentIndex().m_type )
    {
    case ON_COMPONENT_INDEX::mesh_vertex:
    case ON_COMPONENT_INDEX::meshtop_vertex:
      {
        const ON_MeshVertexRef* vref = ON_MeshVertexRef::Cast(geo);
        if ( vref )
          mesh = vref->m_mesh;
      }
      break;

    case ON_COMPONENT_INDEX::meshtop_edge:
      {
        const ON_MeshEdgeRef* eref = ON_MeshEdgeRef::Cast(geo);
        if ( eref )
          mesh = eref->m_mesh;
      }
      break;

    case ON_COMPONENT_INDEX::mesh_face:
      {
        const ON_MeshFaceRef* fref = ON_MeshFaceRef::Cast(geo);
        if ( fref )
          mesh = fref->m_mesh;
      }
      break;

    default:
      // intentionally skipping other ON_COMPONENT_INDEX::TYPE enum values
      break;
    }
  }

  return mesh;
}

bool ON_ObjRef::SetParentIRef( const ON_InstanceRef& iref,
                               ON_UUID iref_id,
                               int idef_geometry_index
                               )
{
  bool rc = false;

  if ( m__iref.Count() > 0 )
  {
    // nested irefs
    if (    0 == m__proxy2
         || 0 == m__proxy_ref_count 
         || *m__proxy_ref_count <= 0 )
    {
      return false;
    }
    ON_Geometry* proxy_geo = ON_Geometry::Cast(m__proxy2);
    if ( !proxy_geo )
      return false;
    if ( !proxy_geo->Transform(iref.m_xform) )
      return false;
    rc = true;
  }
  else if ( ON_COMPONENT_INDEX::invalid_type == m_component_index.m_type )
  {
    // handle top level objects    
    while ( m__proxy1 || m__proxy2 || m__proxy_ref_count )
    {
      // It it's an brep proxy for an extrusion object, then keep going.
      if (    0 != m__proxy1
           && 0 == m__proxy2 
           && 0 != m__proxy_ref_count 
           && 1 == *m__proxy_ref_count 
           && m__proxy1 != m_geometry
           && 0 != ON_Brep::Cast(m_geometry)
           )
      {
        // 13 July 2011 - Part of the fix for bug 87827
        // is to break here instead of returning false
        // because we have something like a brep proxy 
        // of an extrusion.
        break;        
      }
      return false;
    }

    if ( !m_geometry )
    {
      return false;
    }
    if ( m_geometry->ComponentIndex().m_type != ON_COMPONENT_INDEX::invalid_type )
    {
      return false;
    }
    if ( m_parent_geometry && m_geometry != m_parent_geometry )
    {
      return false;
    }
    ON_Geometry* proxy_geo = m_geometry->Duplicate();
    if ( !proxy_geo->Transform(iref.m_xform) )
    {
      delete proxy_geo;
      return false;
    }

    // 13 July 2011 - Part of the fix for bug 87827
    // was to put the m_geometry and m_parent_geometry
    // assignments after the call to SetProxy() which
    // was zeroing m_geometry and m_parent_geometry.
    SetProxy(0,proxy_geo,true);
    m_geometry = proxy_geo;
    m_parent_geometry = proxy_geo;
    rc = true;
  }
  else
  {
    // handle brep and mesh subobjects
    // create proxy object
    if ( m__proxy2 )
      return false;

    const ON_Brep* parent_brep = ON_BrepParent(m_parent_geometry);
    if ( !parent_brep)
      parent_brep = ON_BrepParent(m_geometry);
    if ( parent_brep )
    {
      // handle breps and their parts
      if ( m__proxy1 || m__proxy_ref_count )
      {
        return false;
      }
      if ( m_parent_geometry != parent_brep && 0 != m_parent_geometry )
      {
        return false;
      }
      if ( m_geometry != parent_brep->BrepComponent(m_component_index) )
      {
        return false;
      }
      ON_Brep* proxy_brep = parent_brep->Duplicate();
      if ( !proxy_brep->Transform(iref.m_xform) )
      {
        delete proxy_brep;
        return false;
      }
      const ON_Geometry* brep_component = proxy_brep->BrepComponent(m_component_index);
      if ( !brep_component )
      {
        delete brep_component;
        return false;
      }
      SetProxy(0,proxy_brep,true);
      m_geometry        = brep_component;
      m_parent_geometry = proxy_brep;
      rc = true;
    }
    else
    {
      const ON_Mesh* parent_mesh = ON_MeshParent(m_parent_geometry);
      if ( !parent_mesh)
        parent_mesh = ON_MeshParent(m_geometry);
      if ( parent_mesh  )
      {
        // handle meshes and their parts
        switch(m_component_index.m_type)
        {
        case ON_COMPONENT_INDEX::mesh_vertex:
        case ON_COMPONENT_INDEX::meshtop_vertex:
        case ON_COMPONENT_INDEX::meshtop_edge:
        case ON_COMPONENT_INDEX::mesh_face:
          {
            if ( m_geometry->ComponentIndex() != m_component_index )
              return false;
            ON_Mesh* proxy_mesh = parent_mesh->Duplicate();
            if ( !proxy_mesh->Transform(iref.m_xform) )
            {
              delete proxy_mesh;
              return false;
            }
            ON_Geometry* proxy_component = proxy_mesh->MeshComponent(m_component_index);
            if( !proxy_component )
            {
              delete proxy_mesh;
              return false;
            }
            m_geometry = proxy_component;
            m_parent_geometry = proxy_mesh;
            SetProxy(proxy_component,proxy_mesh,true);
            rc = true;
          }
          break;
        default:
          return false;
          break;
        }
      }
    }
  }

  if ( rc )
  {
    // This is a valid reference to a piece of geometry
    // in an instance definition.

    ON_Xform geometry_xform(1.0);
    if ( m__iref.Count() > 0 )
      geometry_xform = m__iref.Last()->m_geometry_xform;

    ON_ObjRef_IRefID& this_ref     = m__iref.AppendNew();
    this_ref.m_iref_uuid           = iref_id;
    this_ref.m_iref_xform          = iref.m_xform;
    this_ref.m_idef_uuid           = iref.m_instance_definition_uuid;
    this_ref.m_idef_geometry_index = idef_geometry_index;
    this_ref.m_geometry_xform      = iref.m_xform*geometry_xform;

    m_uuid = this_ref.m_iref_uuid;
  }

  return rc;
}

const ON_Object* ON_ObjRef::ProxyObject(int proxy_object_index) const
{
  return ( (1 == proxy_object_index) 
           ? m__proxy1 
           : ((2==proxy_object_index) ? m__proxy2 : 0) 
         );
}

void ON_ObjRef::SetProxy( 
          ON_Object* proxy1, 
          ON_Object* proxy2, 
          bool bCountReferences 
          )
{
  if ( m__proxy1 || m__proxy2 || m__proxy_ref_count )
  {
    // Remove any reference this ON_ObjRef class 
    // may currently have.
    DecrementProxyReferenceCount();
  }

  m__proxy1 = proxy1;
  m__proxy2 = proxy2;
  if ( bCountReferences && (m__proxy1 || m__proxy2) )
  {
    m__proxy_ref_count = (int*)onmalloc_from_pool( 
          ON_MainMemoryPool(), 
          sizeof(*m__proxy_ref_count) 
          );
    *m__proxy_ref_count = 1;
  }
}

void ON_ObjRef::DecrementProxyReferenceCount()
{
  if ( 0 != m__proxy_ref_count ) 
  {
    if (*m__proxy_ref_count > 1) 
    {
      // Including this class, there are *m__proxy_ref_count
      // ON_ObjRef classes using m__proxy and m_geometry.
      // Decrement the reference counter and set the
      // pointers to zero.
      *m__proxy_ref_count = *m__proxy_ref_count - 1;
    }
    else if ( 1 == *m__proxy_ref_count )
    {
      // This is the only ON_ObjRef class using
      // m__proxy and m_geometry.  Set *m__proxy_ref_count
      // to zero (in case some rogue reference still exists),
      // delete m__proxy and m__proxy_ref_count, and
      // set m_geometry (which points to some part of m__proxy)
      // to NULL.

      // Setting *m__proxy_ref_count to zero, prevents crashes
      // if somebody incorrectly uses memcpy() instead of the 
      // copy constructor or operator= to duplicate this class.
      *m__proxy_ref_count = 0;
      if ( m__proxy1 )
      {
        // delete proxy geometry
        delete m__proxy1;
      }
      if ( m__proxy2 )
      {
        // delete proxy geometry
        delete m__proxy2;
      }
      onfree(m__proxy_ref_count);
    }
    else
    {
      // Somebody did something along the lines of using
      // memcpy() instead of the copy constructor or operator=
      // to duplicate this class.
      ON_ERROR("ON_ObjRef::DecrementReferenceCount() *m__proxy_ref_count <= 0");
    }
  }

  // In all cases, setting these pointers to zero indicates this
  // ON_ObjRef is no longer referencing any runtime geometry.
  m__proxy_ref_count = 0;
  m__proxy1 = 0;
  m__proxy2 = 0;
  m_geometry = 0;
}

ON_ObjRef_IRefID::ON_ObjRef_IRefID()
          : m_iref_uuid(ON_nil_uuid),
            m_iref_xform(0.0),
            m_idef_uuid(ON_nil_uuid),
            m_idef_geometry_index(0),
            m_geometry_xform(0.0)
{
}

void ON_ObjRef_IRefID::Default()
{
  ON_ObjRef_IRefID d;
  *this = d;
}

ON_ObjRef_IRefID::~ON_ObjRef_IRefID()
{
}



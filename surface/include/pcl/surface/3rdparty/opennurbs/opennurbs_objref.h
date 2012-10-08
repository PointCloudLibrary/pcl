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

#if !defined(ON_OBJREF_INC_)
#define ON_OBJREF_INC_

class ON_CLASS ON_ObjRefEvaluationParameter
{
public:
  ON_ObjRefEvaluationParameter();
  ~ON_ObjRefEvaluationParameter();

  void Default();

  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );


  // If m_point != ON_UNSET_POINT and m_t_type != 0, then
  // m_t_type, m_t, and m_t_ci record the m_geometry evaluation
  // parameters of the m_point.
  //
  //  m_t_type values
  //
  //  0: no parameter values; m_t_ci and m_t[] have no meaning.
  //
  //  1: m_geometry points to a curve, m_t[0] is a curve
  //     parameter for m_point, and m_t_ci has no meaning.
  //
  //  2: m_geometry points to surface or single faced brep,
  //     (m_t[0],m_t[1]) is a surface parameter for m_point,
  //     and m_t_ci has no meaning.
  //     In this case, m_component_index may not be set or,
  //     if m_geometry points to a brep face, m_component_index
  //     may identify the face in the parent brep.
  //
  //  3: m_geometry points to a brep edge with an associated
  //     trim and m_t[0] is the edge parameter for m_point.
  //     m_t_ci is the ON_BrepTrim component index and m_t[1]
  //     is the ON_BrepTrim parameter that corresponds to the
  //     edge point.  m_s[0] and m_s[1] are normalized parameters.
  //     In this case m_component_index identifies the
  //     the edge in the brep and m_t_ci identifies a trim.
  //
  //  4: m_geometry points to a mesh or mesh face and 
  //     m_t_ci identifies the mesh face.
  //     If the face is a triangle, the barycentric coordinates
  //     of m_point are(m_t[0], m_t[1], m_t[2]) and m_t[3] is zero. 
  //     If the mesh face is a quadrangle, the barycentric coordinates
  //     of m_point are (m_t[0], m_t[1], m_t[2], m_t[3]) and at least 
  //     one of the coordinates is zero.  In both cases, the point
  //     can be evaluated using the formula
  //     m_t[0]*mesh.m_V[f.vi[0]] + ... + m_t[3]*mesh.m_V[f.vi[3]],
  //     where f = mesh.m_F[m_component_index.m_index].
  //     In this case, if m_geometry points to a mesh, then
  //     m_component_index !=  m_t_ci.
  //
  //  5: m_geometry points to a mesh or mesh edge and m_t_ci
  //     identifies the mesh edge. The normalized coordinate of
  //     the point on the mesh edge is m_t[0].  The point can be evaluated
  //     using the formula
  //     m_t[0]*mesh.m_V[v0] + (1.0-m_t[0])*mesh.m_V[v1],
  //     where v0 and v1 are the indices of the mesh vertices at
  //     the edge's ends.
  //     In this case, if m_geometry points to a mesh, then
  //     m_component_index !=  m_t_ci.
  //
  //  6: m_geometry points to a NURBS cage and (m_t[0],m_t[1],m_t[2])
  //     are cage evaluation parameters.
  //
  //  7: m_geometry points to an annotation object and m_t_ci identifies
  //     a point on the annotation object.
  //
  //  8: m_geometry points to a mesh or mesh vertex object and m_t_ci
  //     identifies a vertex on the mesh object.
  //
  int m_t_type;
private:
  int m_reserved; // for future use to record snap info.
public:
  double m_t[4];
  ON_Interval m_s[3]; // curve/surface/cage domains
  ON_COMPONENT_INDEX m_t_ci; // Not necesarily the same as m_component_index
                             // See comment above for details.
};

class ON_CLASS ON_ObjRef_IRefID
{
public:
  ON_ObjRef_IRefID();
  ~ON_ObjRef_IRefID();

  bool Write(ON_BinaryArchive&) const;
  bool Read(ON_BinaryArchive&);

  void Default();

  // m_iref_uuid is the CRhinoInstanceObject's uuid stored
  // in its ON_3dmObjectAttributes.m_uuid.
  ON_UUID  m_iref_uuid;

  // m_iref_xform is the value stored in ON_InstanceRef.m_xform.
  ON_Xform m_iref_xform;

  // m_idef_uuid is the instance definition id stored in
  // ON_InstanceRef.m_instance_definition_uuid and
  // ON_InstanceDefinition.m_uuid.
  ON_UUID  m_idef_uuid;

  // m_geometry_index is the index of the uuid of the pertinant
  // piece of geometry in the ON_InstanceRef.m_object_uuid[] 
  // array.  This index is identical to the index of the
  // geometry's CRhinoObject in the
  // CRhinoInstanceDefinition.m_objects[] array.
  int m_idef_geometry_index;

  // m_geometry_xform is the transformation to map the
  // base geometry to world coordinates.  If the
  // instance reference is not nested, then
  // m_geometry_xform = m_iref_xform.  If the instance
  // reference is nested, then
  //   m_geometry_xform = m_iref_xform * .... * T1
  // where the Ts are the transformations from the children.
  ON_Xform m_geometry_xform;

  // If this ON_ObjRef_IRefID is the first entry in the 
  // ON_ObjRef.m__iref[] array, then it references a "real"
  // piece of geometry (not a nested instance reference).  
  // If the reference is to a subobject of the real piece
  // of geometry, then m_component_index records
  // the subobject index.
  // In all other cases, m_component_index is not set.
  ON_COMPONENT_INDEX m_component_index;

  // If this ON_ObjRef_IRefID is the first entry in the 
  // ON_ObjRef.m__iref[] array, then it references a "real"
  // piece of geometry (not a nested instance reference).  
  // If there is an evaluation parameter for the geometry,
  // it is saved in m_evp.
  // In all other cases, m_evp is not set.
  ON_ObjRefEvaluationParameter m_evp;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_SimpleArray<ON_ObjRef_IRefID>;
#pragma warning( pop )
#endif

class ON_CLASS ON_ObjRef
{
public:
  ON_ObjRef();
  ON_ObjRef(const ON_ObjRef& src);
  ON_ObjRef& operator=(const ON_ObjRef& src);
  ~ON_ObjRef();

  void Destroy();
  bool Write( ON_BinaryArchive& ) const;
  bool Read( ON_BinaryArchive& );

  // In Rhino, this uuid is the persistent id of the CRhinoObject
  // that owns the referenced geometry.  The value of the
  // CRhinoObject id is stored on ON_3dmObjectAttributes.m_uuid.
  ON_UUID m_uuid;

  // The m_geometry and m_parent_geometry pointers are runtime values
  // that point to the object being referenced.  The destructor 
  // ~ON_ObjRef  does not delete the objects these pointers reference.
  //
  // m_geometry_type records the type of geometry m_geometry points to.
  //
  // When the referenced object is a subobject, like a part of a brep 
  // or mesh, m_geometry points to the subobject and m_parent_geometry 
  // points to the parent object, like the brep or mesh.  In this case
  // m_component_index records the location of the subobject.
  //
  // Parts of instance reference objects:
  //   When the geometry belongs to an instance reference
  //   m_uuid is the id of the CRhinoInstanceObject,
  //   m_parent_geometry points to the instance definition
  //   geometry or a transformed proxy, and m_geometry points
  //   to the piece of m_geometry.  The m__iref[] array records
  //   the connection between the instance reference and the
  //   geometry the ON_ObjRef refers to.
  //
  //   For example if the ON_ObjRef is to an edge of a brep in
  //   and instance reference, m_uuid would be the Rhino id of
  //   the CRhinoInstanceObject, m_parent_geometry would point
  //   to a, possibly proxy, ON_Brep object, m_geometry would point
  //   to the ON_BrepEdge in the ON_Brep, m_component_index would
  //   record the edge's index in the ON_Brep.m_E[] array and 
  //   m_geometry_type would be ON::curve_object or ON::brep_edge.
  //   m__iref->Last() would contain the information about the
  //   top level instance reference.  If the brep was at the bottom
  //   of a chain of instance references, m__iref[0] would be the
  //   reference that immediately used the brep.
  const ON_Geometry* m_geometry;
  const ON_Geometry* m_parent_geometry;
  ON_COMPONENT_INDEX m_component_index;
  int m_geometry_type;

  // If m_runtime_sn > 0, then it is the value of a Rhino object's
  // CRhinoObject::m_runtime_object_serial_number field.
  // The serial number is used instead of the pointer to
  // prevent crashes in cases when the CRhinoObject is deleted
  // but an ON_ObjRef continues to reference the Rhino object.
  // The value of m_runtime_sn is not saved in archives because
  // it generally changes if you save and reload an archive.
  unsigned int m_runtime_sn;

  // If m_point != ON_UNSET_POINT, then the ObjRef resolves to 
  // a point location.  The point location is saved here so the
  // information can persist if the object itself vanishes.
  ON_3dPoint m_point;

  // If the point was the result of some type of object snap, then
  // the object snap is recorded here. 
  ON::osnap_mode m_osnap_mode;

  // If m_point != ON_UNSET_POINT and m_evp.m_t_type != 0, then
  // m_evp records the records the m_geometry evaluation
  // parameters for the m_point.
  ON_ObjRefEvaluationParameter m_evp;

  // If m__iref[] is not empty, then m_uuid identifies
  // and instance reference (ON_InstanceRef/CRhinoInstanceObject)
  // and m__iref[] records the chain of instance references from
  // the base piece of geometry to the instance reference.
  // The top level instance reference is last in the list.
  ON_SimpleArray<ON_ObjRef_IRefID> m__iref;

  /*
  Description:
    Expert user tool to decrement reference counts.  Most
    users will never need to call this tool.  It is called
    by ~ON_ObjRef and used in rare cases when a
    ON_ObjRef needs to reference an object only by uuid
    and component index.
  */
  void DecrementProxyReferenceCount();

  /*
  Description:
    Expert user tool to initialize the ON_ObjRef 
    m__proxy1, m__proxy2, and m__proxy_ref_count fields.
  */
  void SetProxy( 
          ON_Object* proxy1, 
          ON_Object* proxy2, 
          bool bCountReferences 
          );

  bool SetParentIRef( const ON_InstanceRef& iref,
                      ON_UUID iref_id,
                      int idef_geometry_index
                      );

  /*
  Returns:
     0:  This ON_ObjRef is not counting references.
    >0:  Number of references.
  */
  int ProxyReferenceCount() const;

  /*
  Parameters:
    proxy_object_index - [in] 1 or 2.
  Returns:
    A pointer to the requested proxy object.
  */
  const ON_Object* ProxyObject(int proxy_object_index) const;

  /*
  Description:
    This tool is used in rare situations when the object ids 
    stored in the uuid list need to be remapped.
  Parameters:
    uuid_remap - [in]
      Is it critical that uuid_remap[] be sorted with respect
      to ON_UuidPair::CompareFirstUuid.
  */
  void RemapObjectId( const ON_SimpleArray<ON_UuidPair>& uuid_remap );

private:
  // In simple (and the most common) cases where m_geometry
  // is managed by something outside of the ON_ObjRef class,
  // m__proxy_ref_count is NULL.  In this case, the m__proxy1
  // and m__proxy2 pointers may still be used to store 
  // references to a parent object.
  //
  // In cases when the referenced geometry pointed at by
  // m_geometry is not being managed by another class,
  // m_proxy1 and m_proxy2 are not NULL and *m_proxy_ref_count 
  // counts the number of ON_ObjRef classes that refer to m__proxy1/2.
  // When the last ON_ObjRef is destroyed, m__proxy1/2 is deleted.
  // When the ON_ObjRef is using reference counting and managing
  // m__proxy1/2, m_geometry points to some part of m__proxy1/2 and
  // m_geometry is destroyed when m__proxy1/2 is destroyed.
  //
  // The convention is to use m__proxy1 to store
  // ON_MeshVertex/Edge/FaceRefs and CRhinoPolyEdges
  // and m__proxy2 to store transformed copies if instance
  // definition geometry.  
  ON_Object* m__proxy1;
  ON_Object* m__proxy2;
  int* m__proxy_ref_count;
  //ON__INT_PTR m_reserved;
};

#if defined(ON_DLL_TEMPLATE)
// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_ObjRef>;
#pragma warning( pop )
#endif

#endif

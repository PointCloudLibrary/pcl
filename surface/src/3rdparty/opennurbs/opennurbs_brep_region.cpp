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



class ON_BrepRegionTopologyUserData : public ON_UserData
{
  ON_OBJECT_DECLARE(ON_BrepRegionTopologyUserData);
public:
  static ON_BrepRegionTopology* RegionTopology(const ON_Brep* brep,bool bValidateFaceCount);

  ON_BrepRegionTopologyUserData();
  ~ON_BrepRegionTopologyUserData();
  ON_BrepRegionTopologyUserData(const ON_BrepRegionTopologyUserData&);
  ON_BrepRegionTopologyUserData& operator=(const ON_BrepRegionTopologyUserData&);

  unsigned int SizeOf() const;
  ON_BOOL32 Archive() const; 
  ON_BOOL32 Transform( const ON_Xform& ); 
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);

  ON_BOOL32 GetDescription( ON_wString& description );

  ON_BrepRegionTopology m_region_topology;
};

ON_OBJECT_IMPLEMENT(ON_BrepRegionTopologyUserData,ON_UserData,"7FE23D63-E536-43f1-98E2-C807A2625AFF");

ON_BrepRegionTopology* ON_BrepRegionTopologyUserData::RegionTopology(const ON_Brep* brep,bool bValidateFaceCount)
{
  ON_BrepRegionTopology* rtop = 0;
  if ( brep )
  {
    ON_BrepRegionTopologyUserData* ud = ON_BrepRegionTopologyUserData::Cast(brep->GetUserData(ON_BrepRegionTopologyUserData::m_ON_BrepRegionTopologyUserData_class_id.Uuid()));
    if (ud)
    {
      rtop = &ud->m_region_topology;
      if (bValidateFaceCount && rtop->m_FS.Count() != 2*brep->m_F.Count())
        rtop = 0;
    }
  }
  return rtop;
}

ON_BrepRegionTopologyUserData::ON_BrepRegionTopologyUserData()
{
  m_userdata_copycount = 1;
  m_userdata_uuid = ON_BrepRegionTopologyUserData::m_ON_BrepRegionTopologyUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs4_id;
}

ON_BrepRegionTopologyUserData::~ON_BrepRegionTopologyUserData()
{
}

ON_BrepRegionTopologyUserData::ON_BrepRegionTopologyUserData( const ON_BrepRegionTopologyUserData& src ) 
                              : ON_UserData(src)
                              , m_region_topology(src.m_region_topology)
{
  m_userdata_uuid = ON_BrepRegionTopologyUserData::m_ON_BrepRegionTopologyUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs4_id;
}

ON_BrepRegionTopologyUserData& ON_BrepRegionTopologyUserData::operator=(const ON_BrepRegionTopologyUserData& src)
{
  if ( this != &src )
  {
    ON_UserData::operator=(src);
    m_region_topology = src.m_region_topology;
  }
  return *this;
}

unsigned int ON_BrepRegionTopologyUserData::SizeOf() const
{
  return ON_UserData::SizeOf() + m_region_topology.SizeOf();
}

ON_BOOL32 ON_BrepRegionTopologyUserData::Archive() const
{
  return true;
}

ON_BOOL32 ON_BrepRegionTopologyUserData::Transform( const ON_Xform& xform)
{
  // Transforming the bbox makes it grow too large under repeated
  // rotations.  So, we will destroy it here and reset it below.
  //m_bbox.Transform(xform);
  int i, j;
  const int region_count = m_region_topology.m_R.Count();
  const int faceside_count = m_region_topology.m_FS.Count();
  const ON_Brep* brep = ON_Brep::Cast(Owner());
  if ( brep )
  {
    const int face_count = brep->m_F.Count();
    for (i = 0; i < region_count; i++ )
    {
      ON_BrepRegion& r = m_region_topology.m_R[i];
      r.m_bbox.Destroy();
      for ( j = 0; j < r.m_fsi.Count(); j++ )
      {
        int fsi = r.m_fsi[j];
        if ( fsi >= 0 && fsi < faceside_count )
        {
          int fi = m_region_topology.m_FS[fsi].m_fi;
          if ( fi >= 0 && fi < face_count )
          {
            r.m_bbox.Union(brep->m_F[fi].BoundingBox());
          }
        }
      }
    }
  }

  for ( i = 0; i < faceside_count; i++ )
    m_region_topology.m_FS[i].TransformUserData(xform);
  for ( i = 0; i < region_count; i++ )
    m_region_topology.m_R[i].TransformUserData(xform);

  return true;
}

ON_BOOL32 ON_BrepRegionTopologyUserData::Write(ON_BinaryArchive& binary_archive) const
{
  return m_region_topology.Write(binary_archive);
}

ON_BOOL32 ON_BrepRegionTopologyUserData::Read(ON_BinaryArchive& binary_archive)
{
  m_region_topology.m_brep = ON_Brep::Cast(Owner());
  return m_region_topology.Read(binary_archive);
}

ON_BOOL32 ON_BrepRegionTopologyUserData::GetDescription( ON_wString& description )
{
  description=L"Brep Region Topology";
  return true;
}

ON_OBJECT_IMPLEMENT(ON_BrepFaceSide,ON_Object,"30930370-0D5B-4ee4-8083-BD635C7398A4");

ON_BOOL32 ON_BrepFaceSide::IsValid( ON_TextLog* ) const
{
  return true;
}

ON_BrepFaceSide::ON_BrepFaceSide()
{
  m_faceside_index = -1;
  m_ri = -1;
  m_fi = -1;
  m_srf_dir = 0;
  m_rtop = 0;
  memset(&m_faceside_user,0,sizeof(m_faceside_user));
}

ON_BrepFaceSide::~ON_BrepFaceSide()
{
}

ON_BrepFaceSide& ON_BrepFaceSide::operator=(const ON_BrepFaceSide& src)
{
  if ( this != &src)
  {
    // do not copy m_brep pointer
    m_faceside_user = src.m_faceside_user;
    m_faceside_index = src.m_faceside_index;
    m_ri = src.m_ri;
    m_fi = src.m_fi;
    m_srf_dir = src.m_srf_dir;
    ON_Object::operator=(src);
  }
  return *this;
}

ON_BOOL32 ON_BrepFaceSide::Write(ON_BinaryArchive& file) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;
  for(;;)
  {
    rc = file.WriteInt( m_faceside_index );
    if (!rc) break;
    rc = file.WriteInt( m_ri );
    if (!rc) break;
    rc = file.WriteInt( m_fi );
    if (!rc) break;
    rc = file.WriteInt( m_srf_dir );
    if (!rc) break;

    break;
  }
  if (!file.EndWrite3dmChunk())
    rc = false;
  return rc;
}

ON_BOOL32 ON_BrepFaceSide::Read(ON_BinaryArchive& file)
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;
  for(;;)
  {
    rc = (1==major_version);
    if (!rc) break;
    rc = file.ReadInt( &m_faceside_index );
    if (!rc) break;
    rc = file.ReadInt( &m_ri );
    if (!rc) break;
    rc = file.ReadInt( &m_fi );
    if (!rc) break;
    rc = file.ReadInt( &m_srf_dir );
    if (!rc) break;

    break;
  }
  if (!file.EndRead3dmChunk())
    rc = false;
  return rc;
}


ON_Brep* ON_BrepFaceSide::Brep() const
{
  return m_rtop ? m_rtop->Brep() : 0;
}


ON_BrepRegionTopology* ON_BrepFaceSide::RegionTopology() const
{
  return m_rtop;
}

ON_BrepRegion* ON_BrepFaceSide::Region() const
{
  ON_BrepRegion* region = 0;
  if ( m_rtop && m_ri >= 0 && m_ri < m_rtop->m_R.Count() )
  {
    region = &m_rtop->m_R[m_ri];
    //region = const_cast<ON_BrepRegion*>(&m_rtop->m_R[m_ri]);
  }
  return region;
}

class ON_BrepFace* ON_BrepFaceSide::Face() const
{
  class ON_BrepFace* face = 0;
  if ( m_rtop && m_fi >= 0 )
  {
    ON_Brep* brep = m_rtop->Brep();
    if ( brep && m_fi < brep->m_F.Count() )
    {
      face = &brep->m_F[m_fi];
    }
  }
  return face;
}

int ON_BrepFaceSide::SurfaceNormalDirection() const
{
  return m_srf_dir;
}

ON_OBJECT_IMPLEMENT(ON_BrepRegion,ON_Object,"CA7A0092-7EE6-4f99-B9D2-E1D6AA798AA1");

ON_BOOL32 ON_BrepRegion::IsValid( ON_TextLog* ) const
{
  return true;
}


ON_BrepRegion::ON_BrepRegion()
{
  m_region_index = -1;
  m_type = -1;
  m_rtop = 0;
  memset(&m_region_user,0,sizeof(m_region_user));
}

ON_BrepRegion::~ON_BrepRegion()
{
}

ON_BrepRegion& ON_BrepRegion::operator=(const ON_BrepRegion& src)
{
  if ( this != &src )
  {
    // do not copy m_brep pointer
    m_region_user = src.m_region_user;
    m_region_index = src.m_region_index;
    m_fsi = src.m_fsi;
    m_type = src.m_type;
    m_bbox = src.m_bbox;
    ON_Object::operator=(src);
  }
  return *this;
}


ON_BOOL32 ON_BrepRegion::Write(ON_BinaryArchive& file) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if ( !rc )
    return false;
  for(;;)
  {
    rc = file.WriteInt( m_region_index );
    if (!rc) break;
    rc = file.WriteInt( m_type );
    if (!rc) break;
    rc = file.WriteArray( m_fsi );
    if (!rc) break;
    rc = file.WriteBoundingBox( m_bbox );
    if (!rc) break;

    break;
  }
  if (!file.EndWrite3dmChunk())
    rc = false;
  return rc;
}

ON_BOOL32 ON_BrepRegion::Read(ON_BinaryArchive& file)
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;
  for(;;)
  {
    rc = (1==major_version);
    if (!rc) break;
    rc = file.ReadInt( &m_region_index );
    if (!rc) break;
    rc = file.ReadInt( &m_type );
    if (!rc) break;
    rc = file.ReadArray( m_fsi );
    if (!rc) break;
    rc = file.ReadBoundingBox( m_bbox );
    if (!rc) break;

    break;
  }
  if (!file.EndRead3dmChunk())
    rc = false;
  return rc;
}


ON_Brep* ON_BrepRegion::Brep() const
{
  return m_rtop ? m_rtop->Brep() : 0;
}

ON_BrepRegionTopology* ON_BrepRegion::RegionTopology() const
{
  return m_rtop;
}


ON_BrepFaceSide* ON_BrepRegion::FaceSide(int rfsi) const
{
  ON_BrepFaceSide* faceside = 0;
  if ( m_rtop && rfsi >= 0 && rfsi < m_fsi.Count() )
  {
    int fsi = m_fsi[rfsi];
    if ( fsi >= 0 && fsi < m_rtop->m_FS.Count() )
    {
      faceside = &m_rtop->m_FS[fsi];
    }
  }
  return faceside;
}


bool ON_BrepRegion::IsFinite() const
{
  return (1 == m_type);
}

const ON_BoundingBox& ON_BrepRegion::BoundingBox() const
{
  return m_bbox;
}

ON_BrepFaceSideArray::ON_BrepFaceSideArray()
{
}

ON_BrepFaceSideArray::~ON_BrepFaceSideArray()
{
}

bool ON_BrepFaceSideArray::Read( ON_BinaryArchive& file )
{
  Empty();
  int count = 0;
  int i;
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (rc) 
  {
    for(;;)
    {
      rc = (1 == major_version);
      if (!rc) break;
      if (rc) rc = file.ReadInt(&count);
      SetCapacity(count);
      for ( i = 0; i < count && rc; i++ ) 
      {
        ON_BrepFaceSide& faceside = AppendNew();
        rc = faceside.Read(file)?true:false;
      }    
      break;
    }
    if ( !file.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_BrepFaceSideArray::Write( ON_BinaryArchive& file ) const
{
  int i;
  bool rc = file.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
  if (rc) 
  {
    const int count = Count();
    if (rc) rc = file.WriteInt( count );
    for ( i = 0; rc && i < count; i++ ) 
    {
      rc = m_a[i].Write(file)?true:false;
    }
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

unsigned int ON_BrepFaceSideArray::SizeOf() const
{
  unsigned int sz = SizeOfArray();
  for ( int i = 0; i < m_count; i++ )
    sz += (m_a[i].SizeOf() - ((unsigned int)sizeof(ON_BrepFaceSide)));
  return sz;
}

ON_BrepRegionArray::ON_BrepRegionArray()
{
}

ON_BrepRegionArray::~ON_BrepRegionArray()
{
}

bool ON_BrepRegionArray::Read( ON_BinaryArchive& file )
{
  Empty();
  int count = 0;
  int i;
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (rc) 
  {
    for(;;)
    {
      rc = (1 == major_version);
      if (!rc) break;
      if (rc) rc = file.ReadInt(&count);
      SetCapacity(count);
      for ( i = 0; i < count && rc ; i++ ) 
      {
        ON_BrepRegion& region = AppendNew();
        rc = region.Read(file)?true:false;
      }    
      break;
    }
    if ( !file.EndRead3dmChunk() )
      rc = false;
  }
  return rc;
}

bool ON_BrepRegionArray::Write( ON_BinaryArchive& file ) const
{
  int i;
  bool rc = file.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
  if (rc) 
  {
    const int count = Count();
    if (rc) rc = file.WriteInt( count );
    for ( i = 0; rc && i < count; i++ ) 
    {
      rc = m_a[i].Write(file)?true:false;
    }
    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

unsigned int ON_BrepRegionArray::SizeOf() const
{
  unsigned int sz = SizeOfArray();
  for ( int i = 0; i < m_count; i++ )
    sz += (m_a[i].SizeOf() - ((unsigned int)sizeof(ON_BrepRegion)));
  return sz;
}

ON_BrepRegionTopology::ON_BrepRegionTopology()
{
}

ON_BrepRegionTopology::~ON_BrepRegionTopology()
{
}

ON_BrepRegionTopology::ON_BrepRegionTopology(const ON_BrepRegionTopology& src)
{
  int i;
  // do not copy m_brep
  m_brep = 0;
  m_FS = src.m_FS;
  m_R = src.m_R;
  for (i = 0; i < m_FS.Count(); i++ )
    m_FS[i].m_rtop = this;
  for (i = 0; i < m_R.Count(); i++ )
    m_R[i].m_rtop = this;
}

ON_BrepRegionTopology& ON_BrepRegionTopology::operator=(const ON_BrepRegionTopology& src)
{
  if ( this != &src )
  {
    // do not copy m_brep
    m_FS = src.m_FS;
    m_R = src.m_R;
    int i;
    for (i = 0; i < m_FS.Count(); i++ )
      m_FS[i].m_rtop = this;
    for (i = 0; i < m_R.Count(); i++ )
      m_R[i].m_rtop = this;
  }
  return *this;
}


bool ON_BrepRegionTopology::IsValid( ON_TextLog* text_log) const
{
#define PRINT_MSG(s) if (text_log) text_log->Print(s)
#define PRINT_MSG1(s,a1) if (text_log) text_log->Print(s,a1)
#define PRINT_MSG2(s,a1,a2) if (text_log) text_log->Print(s,a1,a2)
#define PRINT_MSG3(s,a1,a2,a3) if (text_log) text_log->Print(s,a1,a2,a3)
  int infinite_region_index = -1;
  int rfs_count = 0;
  int ri, fsi;
  if ( !m_brep )
  {
    PRINT_MSG("ON_BrepRegionTopology::m_brep is NULL\n");
    return false;
  }
  const int faceside_count = m_FS.Count();
  if ( faceside_count != 2*m_brep->m_F.Count() )
  {
    PRINT_MSG("ON_BrepRegionTopology::m_FS.Count() != 2*m_brep->m_F.Count()\n");
    return false;
  }

  int void_regionside_count = 0;
  for ( fsi = 0; fsi < faceside_count; fsi++ )
  {
    const ON_BrepFaceSide& fs = m_FS[fsi];
    const int fi = fsi/2;
    const int srf_dir = (fsi%2) ? -1 : 1;
    if ( fs.m_rtop != this )
    {
      PRINT_MSG1("ON_BrepRegionTopology::m_FS[%d].m_rtop != this\n",fsi);
      return false;
    }
    if ( fi != fs.m_fi )
    {
      PRINT_MSG3("ON_BrepRegionTopology::m_FS[%d].m_fi = %d != %d\n",fsi,fs.m_fi,fi);
      return false;
    }
    if ( fs.m_srf_dir != srf_dir )
    {
      PRINT_MSG3("ON_BrepRegionTopology::m_FS[%d].m_srf_dir = %d != %d\n",fsi,fs.m_srf_dir,srf_dir);
      return false;
    }
    if ( -1 == fs.m_ri )
    {
      void_regionside_count++;
    }
  }

  const int region_count = m_R.Count();
  if ( region_count <= 0 )
  {
    PRINT_MSG("ON_BrepRegionTopology::m_R.Count() <= 0\n");
    return false;
  }
  for ( ri = 0; ri < region_count; ri++ )
  {
    const ON_BrepRegion& region = m_R[ri];
    if ( region.m_rtop != this )
    {
      PRINT_MSG1("ON_BrepRegionTopology::m_R[%d].m_rtop != this\n",ri);
      return false;
    }
    if ( region.m_type < 0 )
    {
      PRINT_MSG("ON_BrepRegionTopology::m_R[%d].m_type < 0\n");
      return false;
    }
    if ( region.m_type > 1 )
    {
      PRINT_MSG("ON_BrepRegionTopology::m_R[%d].m_type > 1\n");
      return false;
    }
    if ( 0 == region.m_type )
    {
      if ( infinite_region_index >= 0 )
      {
        PRINT_MSG2("ON_BrepRegionTopology::m_R[%d and %d].m_type = 0\n",infinite_region_index,ri);
        return false;
      }
      infinite_region_index = ri;
    }
    if ( region.m_fsi.Count() <= 0 )
    {
      PRINT_MSG1("ON_BrepRegionTopology::m_R[%d].m_fsi.Count() <= 0\n",ri);
      return false;
    }
    for ( int rfsi = 0; rfsi < region.m_fsi.Count(); rfsi++ )
    {
      fsi = region.m_fsi[rfsi];
      if ( fsi < 0 || fsi >= faceside_count)
      {
        PRINT_MSG2("ON_BrepRegionTopology::m_R[%d].m_fsi[%d] is out of range\n",ri,rfsi);
        return false;
      }
      const ON_BrepFaceSide& faceside = m_FS[fsi];
      if ( faceside.m_ri != ri )
      {
        PRINT_MSG3("ON_BrepRegionTopology::m_FS[m_R[%d].m_fsi[%d]].m_ri != %d\n",ri,rfsi,ri);
        return false;
      }
      for ( int j = rfsi+1; j < region.m_fsi.Count(); j++ )
      {
        if ( fsi == region.m_fsi[j] )
        {
          PRINT_MSG3("ON_BrepRegionTopology::m_R[%d].m_fsi[%d and %d]] are duplicates\n",ri,rfsi,j);
          return false;
        }
      }
      rfs_count++;
    }
  }

  if ( faceside_count != rfs_count+void_regionside_count )
  {
    PRINT_MSG2("Sum of ON_BrepRegionTopology::m_R[%d].m_fsi.Count() = %d != m_FS.Count()\n",ri,rfs_count);
    return false;
  }

  if ( infinite_region_index < 0 )
  {
    PRINT_MSG("ON_BrepRegionTopology::m_R[] has no infinte region\n");
    return false;
  }

#undef PRINT_MSG
#undef PRINT_MSG1
#undef PRINT_MSG2
#undef PRINT_MSG3
  return true;
}

ON_Brep* ON_BrepRegionTopology::Brep() const
{
  return m_brep;
}


bool ON_BrepRegionTopology::Read( ON_BinaryArchive& file )
{
  int i;
  int major_version = 0;
  int minor_version = 0;
  bool rc = file.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( !rc )
    return false;
  for(;;)
  {
    rc = (1 == major_version);
    if (!rc) break;

    rc = m_FS.Read(file);
    for ( i = 0; i < m_FS.Count(); i++ )
      m_FS[i].m_rtop = this;
    if (!rc) break;

    rc = m_R.Read(file);
    for ( i = 0; i < m_R.Count(); i++ )
      m_R[i].m_rtop = this;
    if (!rc) break;

    break;
  }
  if ( !file.EndRead3dmChunk() )
    rc = false;
  return rc;
}

bool ON_BrepRegionTopology::Write( ON_BinaryArchive& file) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;
  for(;;)
  {
    rc = m_FS.Write(file);
    if (!rc) break;
    rc = m_R.Write(file);
    if (!rc) break;

    break;
  }
  if ( !file.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

unsigned int ON_BrepRegionTopology::SizeOf() const
{
  return m_FS.SizeOf() + m_R.SizeOf();
}

ON_BrepFaceSide* ON_BrepFace::FaceSide(int dir) const
{
  ON_BrepFaceSide* faceside = 0;
  const ON_BrepRegionTopology* rtop = ON_BrepRegionTopologyUserData::RegionTopology(m_brep,true);
  if ( rtop )
  {
    if ( m_face_index >= 0 && m_face_index < m_brep->m_F.Count() )
    {
      int fsi = 2*m_face_index + ((dir<1)?1:0);
      faceside = const_cast<ON_BrepFaceSide*>(&rtop->m_FS[fsi]);
      if ( m_face_index != faceside->m_fi || dir != faceside->m_srf_dir )
        faceside = 0;
    }
  }
  return faceside;
}

bool ON_Brep::HasRegionTopology() const
{
  ON_UserData* ud = GetUserData(ON_BrepRegionTopologyUserData::m_ON_BrepRegionTopologyUserData_class_id.Uuid());
  return (0 != ud);
}

const ON_BrepRegionTopology& ON_Brep::RegionTopology() const
{
  ON_BrepRegionTopology* rtop = ON_BrepRegionTopologyUserData::RegionTopology(this,false);
  if ( 0 == rtop )
  {
    ON_BrepRegionTopologyUserData* ud = new ON_BrepRegionTopologyUserData();
    if ( const_cast<ON_Brep*>(this)->AttachUserData(ud) )
    {
      rtop = &ud->m_region_topology;
    }
    else
    {
      ON_ERROR("Unable to create brep region topology");
      delete ud;
    }
  }

  // no region toplogy is available in public opennurbs.

  return *rtop;
}

void ON_Brep::DestroyRegionTopology()
{
  ON_UserData* ud = GetUserData(ON_BrepRegionTopologyUserData::m_ON_BrepRegionTopologyUserData_class_id.Uuid());
  if ( ud )
    delete ud;
}

void ON_Brep::MemoryRelocate()
{
  int i, count;

  // The call to the base class MemoryRelocate() takes care of 
  // updating user data back-pointers.
  ON_Geometry::MemoryRelocate();

  // When the memory location of an ON_Brep changes,
  // the m_brep backpointers on its pieces need to be updated.

  count = m_E.Count();
  for ( i = 0; i < count; i++ )
  {
    m_E[i].m_brep = this;
  }

  count = m_T.Count();
  for ( i = 0; i < count; i++ )
  {
    m_T[i].m_brep = this;
  }

  count = m_L.Count();
  for ( i = 0; i < count; i++ )
  {
    m_L[i].m_brep = this;
  }

  count = m_F.Count();
  for ( i = 0; i < count; i++ )
  {
    m_F[i].m_brep = this;
  }

  ON_BrepRegionTopology* rtop = ON_BrepRegionTopologyUserData::RegionTopology(this,false);
  if ( rtop )
  {
    rtop->m_brep = this;
    count = rtop->m_FS.Count();
    for ( i = 0; i < count; i++ )
      rtop->m_FS[i].m_rtop = rtop;
    count = rtop->m_R.Count();
    for ( i = 0; i < count; i++ )
      rtop->m_R[i].m_rtop = rtop;
  }

}

ON_Brep* ON_Brep::SubBrep( 
          int subfi_count, 
          const int* subfi, 
          ON_Brep* sub_brep
          ) const
{
  class LeakStopper : public ON_Workspace
  {
    // If an error occures during construction,
    // this class cleans up sub_brep in an
    // appropriate fashion.
  public:
    LeakStopper() {m_p=0;m_sub_brep=0;}
    ~LeakStopper() {if (m_p) delete m_p; else if (m_sub_brep) m_sub_brep->Destroy();}
    ON_Brep* m_p;        // ON_Brep::SubBrep allocated sub_brep
    ON_Brep* m_sub_brep; // user's sub_brep argument
  };
  LeakStopper leak_stopper;

  if ( sub_brep )
    sub_brep->Destroy();

  if ( subfi_count <= 0 || 0 == subfi )
    return 0;

  if ( subfi_count > m_F.Count() )
    return 0;

  // validate indices in extract_fi[] and
  // make sure there are no duplicates.
  int fi, fli, lti, i, j;
  int Lcount = 0;
  int Tcount = 0;
  int Ecount = 0;
  int Vcount = 0;
  int maxfi = -1;
  int minfi = m_F.Count();
  int* Emap = leak_stopper.GetIntMemory(m_E.Count());
  memset(Emap,0,m_E.Count()*sizeof(Emap[0]));
  int* Vmap = leak_stopper.GetIntMemory(m_V.Count());
  memset(Vmap,0,m_V.Count()*sizeof(Vmap[0]));
  for ( i = 0; i < subfi_count; i++ )
  {
    fi = subfi[i];
    if ( fi < 0 || fi >= m_F.Count() )
    {
      ON_ERROR("ON_Brep::SubBrep sub_fi[] has invalid indices");
      return 0;
    }
    if ( fi > maxfi )
      maxfi = fi;
    else if ( fi < minfi )
      minfi = fi;
    else
    {
      for ( j = 0; j < i; j++ )
      {
        if ( subfi[j] == fi )
        {
          ON_ERROR("ON_Brep::SubBrep sub_fi[] has duplicate indices");
          return 0;
        }
      }
    }

    const ON_BrepFace& face = m_F[fi];
    for ( fli = 0; fli < face.m_li.Count(); fli++ )
    {
      const ON_BrepLoop* loop = face.Loop(fli);
      if ( !loop || this != loop->Brep() )
        return 0;
      Lcount++;
      for ( lti = 0; lti < loop->m_ti.Count(); lti++ )
      {
        const ON_BrepTrim* trim = loop->Trim(lti);
        if ( !trim || this != trim->Brep() )
          return 0;
        Tcount++;
        if ( trim->m_vi[0] < 0 || trim->m_vi[0] >= m_V.Count() )
          return 0;
        if ( trim->m_vi[1] < 0 || trim->m_vi[1] >= m_V.Count() )
          return 0;
        if ( 0 == Vmap[trim->m_vi[0]] )
        {
          Vmap[trim->m_vi[0]] = 1;
          Vcount++;
        }
        if ( 0 == Vmap[trim->m_vi[1]] )
        {
          Vmap[trim->m_vi[1]] = 1;
          Vcount++;
        }
        if ( ON_BrepTrim::singular == trim->m_type ||
             ON_BrepTrim::ptonsrf == trim->m_type)   // March 29, 2010 Lowell - Allow ptonsrf
        {
          if ( trim->m_ei >= 0 || trim->m_vi[0] != trim->m_vi[1] )
            return 0;
        }
        else if ( trim->m_ei >= 0 )
        {
          const ON_BrepEdge* edge = trim->Edge();
          if ( 0 == edge || this != edge->Brep() )
            return 0;
          if ( 0 == Emap[trim->m_ei] )
          {
            Emap[trim->m_ei] = 1;
            Ecount++;
            // edge's vertices should already be mapped.
            if ( 0 == Vmap[edge->m_vi[0]] )
              return 0;
            if ( 0 == Vmap[edge->m_vi[1]] )
              return 0;
          }          
        }
        else
        {
          return 0;
        }
      }
    }
  }

  if ( !sub_brep )
  {
    sub_brep = ON_Brep::New();
    leak_stopper.m_p = sub_brep;
  }
  else
  {
    leak_stopper.m_sub_brep = sub_brep;
  }

  sub_brep->m_F.Reserve(subfi_count);
  sub_brep->m_L.Reserve(Lcount);
  sub_brep->m_T.Reserve(Tcount);
  sub_brep->m_E.Reserve(Ecount);
  sub_brep->m_V.Reserve(Vcount);
  sub_brep->m_S.Reserve(subfi_count);
  sub_brep->m_C2.Reserve(Tcount);
  sub_brep->m_C3.Reserve(Ecount);

  // build sub_brep vertices
  for ( i = 0; i < m_V.Count(); i++ )
  {
    if ( Vmap[i] )
    {
      const ON_BrepVertex& vertex = m_V[i];
      ON_BrepVertex& sub_vertex = sub_brep->NewVertex(vertex.point,vertex.m_tolerance);
      Vmap[i] = sub_vertex.m_vertex_index;
      sub_vertex.CopyUserData(vertex);
      // March 29, 2010 Lowell - Copy user fields
      memcpy(&sub_vertex.m_vertex_user, &vertex.m_vertex_user, sizeof(sub_vertex.m_vertex_user));
    }
    else
      Vmap[i] = -1;
  }

  // build sub_brep edges
  for ( i = 0; i < m_E.Count(); i++ )
  {
    if ( Emap[i] )
    {
      const ON_BrepEdge& edge = m_E[i];
      if ( Vmap[edge.m_vi[0]] < 0 )
        return 0;
      if ( Vmap[edge.m_vi[1]] < 0 )
        return 0;
      ON_Curve* c3 = edge.DuplicateCurve();
      if ( 0 == c3 )
        return 0;
      sub_brep->m_C3.Append(c3);
      ON_BrepVertex& sub_v0 = sub_brep->m_V[Vmap[edge.m_vi[0]]];
      ON_BrepVertex& sub_v1 = sub_brep->m_V[Vmap[edge.m_vi[1]]];
      ON_BrepEdge& sub_edge = sub_brep->NewEdge(sub_v0,sub_v1,sub_brep->m_C3.Count()-1,0,edge.m_tolerance);
      Emap[i] = sub_edge.m_edge_index;
      sub_edge.CopyUserData(edge);
      // March 29, 2010 Lowell - Copy user fields
      memcpy(&sub_edge.m_edge_user, &edge.m_edge_user, sizeof(sub_edge.m_edge_user));
    }
    else
      Emap[i] = -1;
  }

  bool bHaveBBox = m_bbox.IsValid();
  ON_BoundingBox sub_bbox;

  for ( i = 0; i < subfi_count; i++ )
  {
    const ON_BrepFace& face = m_F[subfi[i]];
    ON_Surface* srf = face.DuplicateSurface();
    if (!srf)
      return 0;
    sub_brep->m_S.Append(srf);
    ON_BrepFace& sub_face = sub_brep->NewFace(sub_brep->m_S.Count()-1);
    sub_face.CopyUserData(face);
    sub_face.m_bRev = face.m_bRev;
    sub_face.m_face_material_channel = face.m_face_material_channel;
    sub_face.m_face_uuid = face.m_face_uuid;
    sub_face.m_bbox = face.m_bbox;
    sub_face.m_domain[0] = face.m_domain[0];
    sub_face.m_domain[1] = face.m_domain[1];
    // March 29, 2010 Lowell - Copy user fields
    memcpy(&sub_face.m_face_user, &face.m_face_user, sizeof(sub_face.m_face_user));

    if ( bHaveBBox )
    {
      if ( sub_face.m_bbox.IsValid() )
        sub_bbox.Union(sub_face.m_bbox);
      else
      {
        bHaveBBox = false;
        sub_bbox.Destroy();
      }
    }


    for ( fli = 0; fli < face.m_li.Count(); fli++ )
    {
      const ON_BrepLoop& loop = m_L[face.m_li[fli]];
      ON_BrepLoop& sub_loop = sub_brep->NewLoop(loop.m_type,sub_face);
      sub_loop.CopyUserData(loop);
      sub_loop.m_pbox = loop.m_pbox;
      // April 19, 2010 Lowell - Copy user fields
      memcpy(&sub_loop.m_loop_user, &loop.m_loop_user, sizeof(sub_loop.m_loop_user));

      for ( lti = 0; lti < loop.m_ti.Count(); lti++ )
      {
        const ON_BrepTrim& trim = m_T[loop.m_ti[lti]];
        if ( Vmap[trim.m_vi[0]] < 0 || Vmap[trim.m_vi[1]] < 0 )
          return 0;
        if ( trim.m_ei >= 0 && Emap[trim.m_ei] < 0 )
          return 0;
        if(trim.m_c2i >= 0)
        {
          ON_Curve* c2 = trim.DuplicateCurve();
          if ( !c2 )
            return 0;
          sub_brep->m_C2.Append(c2);
        }
        else if(trim.m_type != ON_BrepTrim::ptonsrf)
          return 0;
        if ( trim.m_ei >= 0 )
        {
          ON_BrepEdge& sub_edge = sub_brep->m_E[Emap[trim.m_ei]];
          sub_brep->NewTrim(sub_edge,trim.m_bRev3d,sub_loop,sub_brep->m_C2.Count()-1);
        }
        else if ( ON_BrepTrim::singular == trim.m_type )
        {
          ON_BrepVertex& sub_vertex = sub_brep->m_V[Vmap[trim.m_vi[0]]];
          sub_brep->NewSingularTrim(sub_vertex,sub_loop,trim.m_iso,sub_brep->m_C2.Count()-1);
        }
        // March 29, 2010 Lowell - copy ptonsrf type
        else if ( ON_BrepTrim::ptonsrf == trim.m_type)
        {
          ON_BrepTrim& sub_trim = sub_brep->NewTrim(false, sub_loop, -1);
          sub_trim.m_type = ON_BrepTrim::ptonsrf;
          ON_BrepVertex& sub_vertex = sub_brep->m_V[Vmap[trim.m_vi[0]]];
          sub_trim.m_vi[0] = sub_trim.m_vi[1] = sub_vertex.m_vertex_index;
        }
        else
        {
          return 0;
        }
        ON_BrepTrim& sub_trim = sub_brep->m_T[sub_brep->m_T.Count()-1];
        sub_trim.CopyUserData(trim);
        sub_trim.m__legacy_2d_tol = trim.m__legacy_2d_tol;
        sub_trim.m__legacy_3d_tol = trim.m__legacy_3d_tol;
        sub_trim.m__legacy_flags = trim.m__legacy_flags;
        sub_trim.m_tolerance[0] = trim.m_tolerance[0];
        sub_trim.m_tolerance[1] = trim.m_tolerance[1];
        sub_trim.m_pbox = trim.m_pbox;
        sub_trim.m_iso = trim.m_iso;
        // April 19, 2010 Lowell - Copy user fields
        memcpy(&sub_trim.m_trim_user, &trim.m_trim_user, sizeof(sub_trim.m_trim_user));

        // Since we are extracting a subset of the original brep,
        // some mated edges could turn into boundary edges. The
        // call to NewTrim() above will correctly handle setting
        // and updating sub_trims that came from mated trims.
        if ( ON_BrepTrim::mated != trim.m_type )
          sub_trim.m_type = trim.m_type;
      }
    }
  }

  if ( !bHaveBBox || !sub_bbox.IsValid() )
    sub_brep->BoundingBox();
  else
    sub_brep->m_bbox = sub_bbox;

  // return subbrep after disabling the leak stopper
  leak_stopper.m_p = 0;
  leak_stopper.m_sub_brep = 0;
  return sub_brep;
}

ON_Brep* ON_BrepRegion::RegionBoundaryBrep( ON_Brep* brep ) const
{
  ON_Workspace ws;
  if ( 0 == m_rtop )
    return 0;

  const ON_Brep* rtop_brep = m_rtop->Brep();

  if ( rtop_brep == brep || 0 == rtop_brep || rtop_brep->m_F.Count() <= 0 || m_fsi.Count() <= 0 )
    return 0;

  ON_SimpleArray<const ON_BrepFaceSide*> FS(m_fsi.Count());
  ON_SimpleArray<int> subfi(m_fsi.Count());

  int rfsi, i;
  for ( rfsi = 0; rfsi < m_fsi.Count(); rfsi++ )
  {
    const ON_BrepFaceSide* fs = FaceSide(rfsi);
    if ( 0 == fs || fs->m_fi < 0 || fs->m_fi >= rtop_brep->m_F.Count() )
      return 0;
    for ( i = 0; i < FS.Count(); i++ )
    {
      if ( fs->m_fi == FS[i]->m_fi )
        break;
    }
    if ( i < FS.Count() )
      continue;
    FS.Append(fs);
    subfi.Append(fs->m_fi);
  }

  brep = rtop_brep->SubBrep(subfi.Count(),subfi.Array(),brep);
  if ( !brep )
    return 0;
  if ( brep->m_F.Count() != FS.Count() )
    return 0;
  for ( i = 0; i < FS.Count(); i++ )
  {
    ON_BrepFace& face = brep->m_F[i];
    face.m_bRev = ( FS[i]->m_srf_dir < 0 );
  }

  ON_BOOL32 bIsOriented = false;
  ON_BOOL32 bHasBoundary = true;
  if ( brep->IsManifold(&bIsOriented,&bHasBoundary) )
  {
    if ( bIsOriented && !bHasBoundary )
    {
      if ( 1 == m_type )
        brep->m_is_solid = 2;
      else if ( 0 == m_type )
        brep->m_is_solid = 1;
    }
  }

  return brep;
}


#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"



// DO NOT COPY THIS DEFINE
#define ON_NGON_BOZO_VACCINE





// Do not copy or move the definition of
// the ON_NGON_MEMBLK structure.
struct ON_NGON_MEMBLK
{
#if !defined(ON_NGON_BOZO_VACCINE)
#error You are a bozo!  Read the comments.
#endif
  struct ON_NGON_MEMBLK* next;
};

ON_MeshNgonList::ON_MeshNgonList()
{
  m_ngons_count = 0;
  m_ngons_capacity = 0;
  m_ngons = 0;
  m_memblk_list = 0; 
}

ON_MeshNgonList::~ON_MeshNgonList()
{
  Destroy();
}

void ON_MeshNgonList::Destroy()
{
  m_ngons_count = 0;
  m_ngons_capacity = 0;
  if ( 0 != m_ngons )
  {
    onfree(m_ngons);
    m_ngons = 0;
  }
  struct ON_NGON_MEMBLK* p = m_memblk_list;
  m_memblk_list = 0;
  while(p)
  {
    struct ON_NGON_MEMBLK* next = p->next;
    onfree(p);
    p = next;
  }
}


ON_MeshNgonList::ON_MeshNgonList(const ON_MeshNgonList& src)
{
  m_ngons_count = 0;
  m_ngons_capacity = 0;
  m_ngons = 0;
  m_memblk_list = 0; 
  if ( src.m_ngons_count > 0 && 0 != src.m_ngons )
  {
    *this = src;
  }
}


ON_MeshNgonList& ON_MeshNgonList::operator=(const ON_MeshNgonList& src)
{
  if ( this != &src )
  {
    Destroy();
    ReserveNgonCapacity(src.m_ngons_count);
    for ( int i = 0; i < src.m_ngons_count; i++ )
    {
      const ON_MeshNgon& ngon = src.m_ngons[i];
      AddNgon(ngon.N,ngon.vi,ngon.fi);
    }
  }
  return *this;
}


bool ON_MeshNgonList::ReserveNgonCapacity(int capacity)
{
  bool rc = true;
  if ( capacity > m_ngons_capacity )
  {
    m_ngons = (ON_MeshNgon*)onrealloc(m_ngons,capacity*sizeof(m_ngons[0]));
    if ( 0 == m_ngons )
    {
      m_ngons_capacity = 0;
      m_ngons_count = 0;
      rc = false;
    }
    else
    {
      m_ngons_capacity = capacity;
    }
  }
  return rc;
}

struct ON_MeshNgon* ON_MeshNgonList::AddNgon(int N)
{
  if ( N < 3 || N > 100000 )
    return 0;

  if ( m_ngons_count >= m_ngons_capacity )
  {
    int capacity = 2*m_ngons_count;
    if (capacity < m_ngons_count+16)
      capacity = m_ngons_count+16;
    if ( !ReserveNgonCapacity(capacity) )
      return 0;
  }
  ON_MeshNgon& ngon = m_ngons[m_ngons_count++];

  ngon.N = N;
  struct ON_NGON_MEMBLK* blk = (struct ON_NGON_MEMBLK*)onmalloc(sizeof(*blk) + (2*N)*sizeof(int));
  if ( 0 == blk )
    return 0;
  ngon.vi = (int*)(blk + 1);
  ngon.fi = ngon.vi + N;
  memset(ngon.vi,0xFF,(2*N)*sizeof(int)); // set all indicies to -1
  blk->next = m_memblk_list;
  m_memblk_list = blk;
  return &ngon;
}

bool ON_MeshNgonList::AddNgon(int N, const int* vi, const int* fi)
{
  if ( 0 == vi || 0 == fi )
    return false;
  struct ON_MeshNgon* ngon = AddNgon(N);
  if ( 0 == ngon )
    return false;
  memcpy(ngon->vi,vi,N*sizeof(ngon->vi[0]));
  memcpy(ngon->fi,fi,(N-2)*sizeof(ngon->fi[0]));
  return true;
}

int ON_MeshNgonList::NgonCount() const
{
  return m_ngons_count;
}

ON_MeshNgon* ON_MeshNgonList::Ngon(int Ngon_index) const
{
  return (Ngon_index < 0 || Ngon_index >= m_ngons_count) ? 0 : m_ngons+Ngon_index;
}


class /* DO NOT EXPORT THIS CLASS */ ON_MeshNgonUserData : public ON_UserData
{
#if !defined(ON_NGON_BOZO_VACCINE)
#error You are a bozo!  Read the comments.
#endif
  ON_OBJECT_DECLARE(ON_MeshNgonUserData);

public:
  ON_MeshNgonUserData();
  ~ON_MeshNgonUserData();
  ON_MeshNgonUserData(const ON_MeshNgonUserData&);
  ON_MeshNgonUserData& operator=(const ON_MeshNgonUserData&);

  // vitual ON_UserData override
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;
  unsigned int SizeOf() const;
  ON_BOOL32 Write(ON_BinaryArchive&) const;
  ON_BOOL32 Read(ON_BinaryArchive&);

  // vitual ON_UserData override
  ON_BOOL32 GetDescription( ON_wString& );
  ON_BOOL32 Archive() const; 

public:
  ON_MeshNgonList* m_ngon_list;

  // used to validate ngon list.
  // If the information here does not match the
  // information in the mesh, then the ngon list
  // is known to be invalid.
  int m_mesh_F_count;
  int m_mesh_V_count;
};

ON_OBJECT_IMPLEMENT(ON_MeshNgonUserData,ON_UserData,"31F55AA3-71FB-49f5-A975-757584D937FF");

ON_MeshNgonUserData::ON_MeshNgonUserData()
{
  m_userdata_uuid = ON_MeshNgonUserData::m_ON_MeshNgonUserData_class_id.Uuid();
  m_application_uuid = ON_opennurbs4_id;
  m_userdata_copycount = 1;
  m_ngon_list = 0;
  m_mesh_F_count = 0;
  m_mesh_V_count = 0;
}

ON_MeshNgonUserData::~ON_MeshNgonUserData()
{
  if ( 0 != m_ngon_list )
  {
    delete m_ngon_list;
    m_ngon_list = 0;
  }
}

ON_MeshNgonUserData::ON_MeshNgonUserData(const ON_MeshNgonUserData& src) 
: ON_UserData(src)
, m_mesh_F_count(src.m_mesh_F_count)
, m_mesh_V_count(src.m_mesh_V_count)
{
  m_ngon_list = (0 != src.m_ngon_list) 
              ? new ON_MeshNgonList(*src.m_ngon_list)
              : 0;
}

ON_MeshNgonUserData& ON_MeshNgonUserData::operator=(const ON_MeshNgonUserData& src)
{
  if ( this != &src )
  {
    if (0 != m_ngon_list )
    {
      delete m_ngon_list;
      m_ngon_list = 0;
    }
    ON_UserData::operator=(src);
    if (0 != src.m_ngon_list) 
    {
       m_ngon_list = new ON_MeshNgonList(*src.m_ngon_list);
    }
    m_mesh_F_count = src.m_mesh_F_count;
    m_mesh_V_count = src.m_mesh_V_count;
  }
  return *this;
}

ON_BOOL32 ON_MeshNgonUserData::IsValid( ON_TextLog* text_log ) const
{
  return true;
}

unsigned int ON_MeshNgonList::SizeOf() const
{
  unsigned int sz = sizeof(*this);
  int icount = 0;
  for ( int i = 0; i < m_ngons_count; i++ )
  {
    icount += 2*m_ngons[i].N;
  }
  sz += m_ngons_capacity*sizeof(m_ngons[0]);
  sz += icount*sizeof(int);
  return sz;
}

unsigned int ON_MeshNgonUserData::SizeOf() const
{
  unsigned int sz = ON_UserData::SizeOf();
  if ( 0 != m_ngon_list )
    sz += m_ngon_list->SizeOf();
  return sz;
}


ON_BOOL32 ON_MeshNgonUserData::Write(ON_BinaryArchive& archive) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if (!rc)
    return false;
  for (;;)
  {
    int count = ( 0 == m_ngon_list ) ? 0 : m_ngon_list->NgonCount();
    const ON_MeshNgon* ngon_array = (count > 0) ? m_ngon_list->Ngon(0) : 0;
    if ( 0 == ngon_array )
      count = 0;
    rc = archive.WriteInt(count);
    if (count <= 0 || !rc)
      break;
    for ( int i = 0; i < count; i++ )
    {
      const struct ON_MeshNgon& ngon = ngon_array[i];
      rc = archive.WriteInt(ngon.N);
      if (!rc)
        break;
      rc = archive.WriteInt(ngon.N,ngon.vi);
      if (!rc)
        break;
      rc = archive.WriteInt(ngon.N,ngon.fi);
      if (!rc)
        break;
    }
    if (!rc)
      break;

    // chunk version 1.1 added face and vertex validation counts.
    rc = archive.WriteInt(m_mesh_F_count);
    if (!rc)
      break;
    rc = archive.WriteInt(m_mesh_V_count);
    if (!rc)
      break;

    break;
  }
  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

ON_BOOL32 ON_MeshNgonUserData::Read(ON_BinaryArchive& archive)
{
  if ( 0 != m_ngon_list )
  {
    delete m_ngon_list;
    m_ngon_list = 0;
  }
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;
  for (;;)
  {
    rc = (1 == major_version);
    if (!rc)
      break;
    int count = 0;
    rc = archive.ReadInt(&count);
    if (count <= 0 || !rc)
      break;

    m_ngon_list = new ON_MeshNgonList();
    if ( 0 == m_ngon_list )
      break;

    m_ngon_list->ReserveNgonCapacity(count);

    for ( int i = 0; i < count; i++ )
    {
      int N = 0;
      rc = archive.ReadInt(&N);
      if (!rc)
        break;
      if ( N <= 0 )
        continue;
      struct ON_MeshNgon* ngon = m_ngon_list->AddNgon(N);
      if ( 0 == ngon )
        break;

      rc = archive.ReadInt(N,ngon->vi);
      if (!rc)
        break;
      rc = archive.ReadInt(N,ngon->fi);
      if (!rc)
        break;
      ngon->N = N;
    }
    if (!rc)
      break;

    if ( minor_version >= 1 )
    {
      // chunk version 1.1 added face and vertex validation counts.
      rc = archive.ReadInt(&m_mesh_F_count);
      if (!rc)
        break;
      rc = archive.ReadInt(&m_mesh_V_count);
      if (!rc)
        break;
    }

    break;
  }
  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}

// vitual ON_UserData override
ON_BOOL32 ON_MeshNgonUserData::GetDescription( ON_wString& description )
{
  description = L"Mesh N-gon list";
  return true;
}

ON_BOOL32 ON_MeshNgonUserData::Archive() const
{
  return ( 0 != m_ngon_list && m_ngon_list->NgonCount() > 0 );
}

static
bool ON_ValidateNgon(
  const ON_MeshNgon* ngon,
  int mesh_V_count,
  int mesh_F_count
  )
{
  unsigned int j;
  if ( 0 == ngon || ngon->N < 0 )
    return false;
  const unsigned int N = ngon->N;
  for ( j = 0; j < N; j++ )
  {
    if ( ngon->vi[j] < 0 || ngon->vi[j] >= mesh_V_count )
      return false;
    if ( ngon->fi[j] < -1 || ngon->fi[j] >= mesh_F_count )
      return false;
  }
  return true;
}

static 
bool ON_ValidateMeshNgonUserData(
  ON_MeshNgonUserData* ngud,
  const ON_Mesh& mesh
  )
{
  int i;
  if ( 0 == ngud || 0 == ngud->m_ngon_list )
    return false;

  const int mesh_V_count = mesh.m_V.Count();
  const int mesh_F_count = mesh.m_F.Count();

  if ( 0 == ngud->m_mesh_V_count && 0 == ngud->m_mesh_F_count )
  {
    // This is old user data that did not have validation counts
    // saved in the file.

    // Set validation counts to -1 so we never do this slow validation again.
    ngud->m_mesh_V_count = -1;
    ngud->m_mesh_F_count = -1;

    const int ngon_count = ngud->m_ngon_list->NgonCount();

    for ( i = 0; i < ngon_count; i++ )
    {
      if ( !ON_ValidateNgon(ngud->m_ngon_list->Ngon(i),mesh_V_count,mesh_F_count) )
        return false;
    }

    // Set validation counts to proper values because we will 
    // assume this old ngon information is valid since the indices
    // are in range.  This assumption may not be valid, but
    // at least the ngon won't cause a crash.
    ngud->m_mesh_V_count = mesh_V_count;
    ngud->m_mesh_F_count = mesh_F_count;
  }

  return ( ngud->m_mesh_F_count == mesh_F_count && ngud->m_mesh_V_count == mesh_V_count );
}
                         

const class ON_MeshNgonList* ON_Mesh::NgonList() const
{
  ON_UserData* ud = GetUserData(ON_MeshNgonUserData::m_ON_MeshNgonUserData_class_id.Uuid());
  ON_MeshNgonUserData* ngud = ON_MeshNgonUserData::Cast(ud);
  
  if ( 0 != ngud && !ON_ValidateMeshNgonUserData(ngud,*this) )
  {
    delete ngud;
    ngud = 0;
  }

  return (0 == ngud) ? 0 : ngud->m_ngon_list;
}


class ON_MeshNgonList* ON_Mesh::ModifyNgonList()
{
  ON_UserData* ud = GetUserData(ON_MeshNgonUserData::m_ON_MeshNgonUserData_class_id.Uuid());
  ON_MeshNgonUserData* ngud = ON_MeshNgonUserData::Cast(ud);
  if ( 0 == ngud )
  {
    if ( ud )
    {
      delete ud;
      ud = 0;
    }
    ngud = new ON_MeshNgonUserData();
    ngud->m_mesh_F_count = m_F.Count();
    ngud->m_mesh_V_count = m_V.Count();
    AttachUserData(ngud);
  }
  else if ( 0 != ngud->m_ngon_list && !ON_ValidateMeshNgonUserData(ngud,*this) )
  {
    delete ngud->m_ngon_list;
    ngud->m_ngon_list = 0;
  }

  if ( 0 == ngud->m_ngon_list )
  {
    ngud->m_ngon_list = new ON_MeshNgonList();
    ngud->m_mesh_F_count = m_F.Count();
    ngud->m_mesh_V_count = m_V.Count();
  }

  return ngud->m_ngon_list;
}


void ON_Mesh::DestroyNgonList()
{
  ON_UserData* ud = GetUserData(ON_MeshNgonUserData::m_ON_MeshNgonUserData_class_id.Uuid());
  if ( 0 != ud )
  {
    delete ud;
    ud = 0;
  }
}


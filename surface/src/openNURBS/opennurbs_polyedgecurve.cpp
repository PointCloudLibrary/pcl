#include <pcl/surface/openNURBS/opennurbs.h>
#include <pcl/surface/openNURBS/opennurbs_polyedgecurve.h>

ON_OBJECT_IMPLEMENT(ON_PolyEdgeSegment,ON_CurveProxy,"42F47A87-5B1B-4e31-AB87-4639D78325D6");

ON_PolyEdgeSegment::ON_PolyEdgeSegment()
{
  Init();
}

ON_PolyEdgeSegment::~ON_PolyEdgeSegment()
{
  Init();
}

void ON_PolyEdgeSegment::Init()
{
  m_component_index.UnSet();
  m_object_id = ON_nil_uuid;
  m_brep = 0;
  m_trim = 0;
  m_edge = 0;
  m_face = 0;
  m_surface = 0;
  m_edge_domain.Destroy();
  m_trim_domain.Destroy();
  ON_CurveProxy::SetProxyCurve(0);

  ClearEvalCacheHelper();
}

void ON_PolyEdgeSegment::ClearEvalCacheHelper()
{
  m_t = ON_UNSET_VALUE;
  m_edge_t = ON_UNSET_VALUE;
  m_trim_t = ON_UNSET_VALUE;
  m_srf_uv[0] = ON_UNSET_VALUE;
  m_srf_uv[1] = ON_UNSET_VALUE;
  m_trim_hint = 0;
  m_edge_hint = 0;
  m_evsrf_hint[0] = 0;
  m_evsrf_hint[1] = 0;
  m_evsrf_uv[0] = ON_UNSET_VALUE;
  m_evsrf_uv[1] = ON_UNSET_VALUE;
  m_evsrf_pt = ON_UNSET_POINT;
}

ON_OBJECT_IMPLEMENT(ON_PolyEdgeCurve,ON_PolyCurve,"39FF3DD3-FE0F-4807-9D59-185F0D73C0E4");

ON_PolyEdgeCurve::ON_PolyEdgeCurve()
{
}

ON_PolyEdgeCurve::~ON_PolyEdgeCurve()
{
}


void ON_PolyEdgeCurve::DestroyRuntimeCache( bool bDelete )
{
  //m_is_closed_helper = 0;
  ON_PolyCurve::DestroyRuntimeCache(bDelete);
}

void ON_PolyEdgeSegment::DestroyRuntimeCache( bool bDelete )
{
  ClearEvalCacheHelper();
  ON_CurveProxy::DestroyRuntimeCache(bDelete);
}


ON_BOOL32 ON_PolyEdgeSegment::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
  if (!rc)
    return false;
  for(;;)
  {
    rc = archive.WriteUuid(m_object_id);
    if (!rc) break;
    rc = archive.WriteComponentIndex(m_component_index);
    if (!rc) break;
    rc = archive.WriteInterval(m_edge_domain);
    if (!rc) break;
    rc = archive.WriteInterval(m_trim_domain);
    if (!rc) break;
    bool b = ON_CurveProxy::ProxyCurveIsReversed();
    rc = archive.WriteBool(b);
    if (!rc) break;
    rc = archive.WriteInterval(ON_CurveProxy::Domain());
    if (!rc) break;
    rc = archive.WriteInterval(ON_CurveProxy::ProxyCurveDomain());
    if (!rc) break;

    break;
  }
  if ( !archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}

ON_BOOL32 ON_PolyEdgeSegment::Read( ON_BinaryArchive& archive )
{
  Init();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;
  for(;;)
  {
    rc = (1 == major_version);
    if ( !rc )
      break;

    rc = archive.ReadUuid(m_object_id);
    if (!rc) break;
    rc = archive.ReadComponentIndex(m_component_index);
    if (!rc) break;
    rc = archive.ReadInterval(m_edge_domain);
    if (!rc) break;
    rc = archive.ReadInterval(m_trim_domain);
    if (!rc) break;

    // Read ON_ProxyCurve values we need
    bool bReversed = false;
    rc = archive.ReadBool(&bReversed);
    if (!rc) break;
    ON_Interval this_domain;
    rc = archive.ReadInterval(this_domain);
    if (!rc) break;
    ON_Interval real_curve_domain;
    rc = archive.ReadInterval(real_curve_domain);
    if (!rc) break;

    if ( bReversed)
      ON_CurveProxy::Reverse();
    ON_CurveProxy::SetDomain(this_domain);
    ON_CurveProxy::SetProxyCurveDomain(real_curve_domain);

    break;
  }
  if ( !archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}


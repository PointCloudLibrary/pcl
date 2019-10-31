#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_polyedgecurve.h"

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

bool ON_PolyEdgeSegment::Create( 
                const ON_BrepTrim* trim,
                const ON_UUID& object_id
                )
{
  Init();
  if ( !trim )
    return false;
  if ( trim->m_ei >= 0 )
    return false;
  const ON_Brep* brep = trim->Brep();
  if ( !brep )
    return false;
  const ON_BrepEdge* edge = trim->Edge();
  if ( !edge )
    return false;
  const ON_Curve* c3 = edge->EdgeCurveOf();
  if ( !c3 )
    return false;
  m_brep = brep;
  m_trim = trim;
  m_edge = edge;
  m_face = trim->Face();
  if ( m_face )
    m_surface = m_face->SurfaceOf();
  m_edge_domain = m_edge->Domain();
  m_trim_domain = m_trim->Domain();
  ON_CurveProxy::SetProxyCurve( 
                    c3,
                    edge->ProxyCurveDomain()
                    );    
  if ( m_edge->ProxyCurveIsReversed() )
    ON_CurveProxy::Reverse();
  ON_CurveProxy::SetDomain( m_edge_domain[0], m_edge_domain[1] );
  m_component_index = trim->ComponentIndex();
  m_object_id = object_id;
  return true;
}


bool ON_PolyEdgeSegment::ReversedEdgeDir() const
{
  bool rc = false;
  if ( m_edge )
  {
    rc = m_edge->ProxyCurveIsReversed() != ON_CurveProxy::ProxyCurveIsReversed();
  }
  return rc;
}

bool ON_PolyEdgeSegment::ReversedTrimDir() const
{
  bool rc = false;
  if ( m_trim && m_edge )
  {
    rc = ReversedEdgeDir();
    if ( m_trim->m_bRev3d )
      rc = !rc;
  }
  return rc;
}

bool ON_PolyEdgeSegment::Create( 
          const ON_Curve* curve,
          const ON_UUID& object_id 
          )
{
  //bool rc = false;
  Init();
  if ( !curve )
    return false;
  const ON_BrepEdge* edge = ON_BrepEdge::Cast(curve);
  if ( edge )
  {
    const ON_Brep* brep = edge->Brep();
    if ( !brep )
      return false;
    const ON_Curve* c3 = edge->EdgeCurveOf();
    if ( !c3 )
      return false;
    m_edge = edge;
    m_brep = brep;
    m_component_index = edge->ComponentIndex();
    m_edge_domain = m_edge->Domain();
    m_trim_domain = m_trim->Domain();
    ON_CurveProxy::SetProxyCurve( 
                      c3,
                      edge->ProxyCurveDomain()
                      );    
    if ( m_edge->ProxyCurveIsReversed() )
      ON_CurveProxy::Reverse();
    ON_CurveProxy::SetDomain( m_edge_domain[0], m_edge_domain[1] );
  }
  else
  {
    ON_CurveProxy::SetProxyCurve(const_cast<ON_Curve*>(curve));   
  }
  m_object_id = object_id;
  return true;
}

const ON_BrepEdge* ON_PolyEdgeSegment::Edge() const
{
  return m_edge;
}

const ON_BrepTrim* ON_PolyEdgeSegment::Trim() const
{
  return m_trim;
}

const ON_Brep* ON_PolyEdgeSegment::Brep() const
{
  return m_brep;
}

const ON_BrepFace* ON_PolyEdgeSegment::Face() const
{
  return m_face;
}

const ON_Surface*  ON_PolyEdgeSegment::Surface() const
{
  return m_surface;
}

ON_Surface::ISO ON_PolyEdgeSegment::IsoType() const
{
  return m_trim ? m_trim->m_iso : ON_Surface::not_iso;
}

ON_Interval ON_PolyEdgeSegment::EdgeDomain() const
{
  return m_edge_domain;
}

ON_Interval ON_PolyEdgeSegment::TrimDomain() const
{
  return m_trim_domain;
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

double ON_PolyEdgeSegment::EdgeParameter(double t) const
{
  double edge_t = ON_UNSET_VALUE;
  if ( m_edge )
  {
    if ( m_t == t && m_edge_t != ON_UNSET_VALUE )
      edge_t = m_edge_t;
    else
    {
      ON_PolyEdgeSegment* p = const_cast<ON_PolyEdgeSegment*>(this);
      if ( t != m_t )
      {
        p->m_t = t;
        p->m_trim_t = ON_UNSET_VALUE;
        p->m_srf_uv[0] = ON_UNSET_VALUE;
        p->m_srf_uv[1] = ON_UNSET_VALUE;
      }
      ON_Interval d = Domain();
      bool bReversedEdgeDir = ReversedEdgeDir();
      if ( bReversedEdgeDir || m_edge_domain != d )
      {
        double s = d.NormalizedParameterAt(t);
        if ( bReversedEdgeDir )
          s = 1.0 - s;
        edge_t = m_edge_domain.ParameterAt(s);
      }
      else
        edge_t = t;
      p->m_edge_t = edge_t;
    }
  }
  return edge_t;
}


ON_OBJECT_IMPLEMENT(ON_PolyEdgeCurve,ON_PolyCurve,"39FF3DD3-FE0F-4807-9D59-185F0D73C0E4");

ON_PolyEdgeCurve::ON_PolyEdgeCurve()
{
}

ON_PolyEdgeCurve::~ON_PolyEdgeCurve()
{
}

ON_BOOL32 ON_PolyEdgeCurve::SetStartPoint( ON_3dPoint )
{
  return false; // cannot change edges
}

ON_BOOL32 ON_PolyEdgeCurve::SetEndPoint( ON_3dPoint )
{
  return false; // cannot change edges
}

ON_BOOL32 ON_PolyEdgeCurve::ChangeClosedCurveSeam( double t )
{
  //int saved_is_closed_helper = m_is_closed_helper;

  if ( SegmentCount() == 1 )
  {
    // A ON_PolyEdgeSegment cannot have its start/end
    // changed. Split it into two segments and let 
    // ON_PolyCurve::ChangeClosedCurveSeam() do the work.
    if ( !IsClosed() )
      return false;

    ON_Interval crvd = Domain();
    double s = crvd.NormalizedParameterAt(t);

    if ( s <= ON_SQRT_EPSILON || s >= (1.0 - ON_SQRT_EPSILON) )
    {
      s = fmod(s,1.0);
      if ( s < 0.0 )
        s += 1.0;
      if ( fabs(s) <= ON_SQRT_EPSILON || fabs(1.0-s) <= ON_SQRT_EPSILON )
      {
        // split parameter is at start/end of this segemnt
        if ( t != crvd[0] )
        {
          DestroyRuntimeCache();
          SetDomain(t,t+crvd.Length() );
          //m_is_closed_helper = saved_is_closed_helper;
        }
        return true;
      }
      return false;
    }

    ON_PolyEdgeSegment* left_seg = SegmentCurve(0);
    if ( 0 == left_seg )
      return false;

    DestroyRuntimeCache();

    ON_Curve* left = left_seg;
    ON_Curve* right = 0;
    double segt = SegmentCurveParameter(t);
    if ( !left_seg->Split(segt,left,right) )
      return false;
    SetDomain(crvd[0],t);
    
    ON_PolyEdgeSegment* right_seg = ON_PolyEdgeSegment::Cast(right);
    if ( 0 == right_seg )
      return false;
    Append(right_seg);

    double st[3];
    st[0] = crvd[0];
    st[1] = t;
    st[2] = crvd[1];
    SetParameterization( st );
  }

  // ON_PolyCurve::ChangeClosedCurveSeam works fine on
  // two or more segments.
  ON_BOOL32 rc = ON_PolyCurve::ChangeClosedCurveSeam(t);
  //if ( saved_is_closed_helper )
  //  m_is_closed_helper = saved_is_closed_helper;

  return rc;
}

ON_BOOL32 ON_PolyEdgeCurve::PrependAndMatch(ON_Curve*)
{
  return false; // cannot change edges
}


ON_BOOL32 ON_PolyEdgeCurve::AppendAndMatch(ON_Curve*)
{
  return false; // cannot change edges
}

ON_Curve* ON_PolyEdgeSegment::DuplicateCurve() const
{
  return ON_CurveProxy::DuplicateCurve();

  // 21 December 2004 Dale Lear
  //     This is wrong.  I did it some time ago as a quick
  //     fix for one of Lowell's early uses of CRhinoPolyEdges
  //     however, this will cause lots of crashes now that
  //     all commands have to deal with polyedges and the code
  //     in those commands assumes that DuplicateCurve() returns
  //     a valid stand-alone curve.  If you end up here and
  //     wish this code still worked the old way, please get
  //     in touch with Dale Lear and we'll find a way to get
  //     your code to work.
  // NO // ON_PolyEdgeSegment* dup = Duplicate();
  // NO // return dup;
}

ON_Curve* ON_PolyEdgeCurve::DuplicateCurve() const
{
  return ON_PolyCurve::DuplicateCurve();

  // 21 December 2004 Dale Lear
  //     The code below is wrong.  I wrote it some time ago as a quick
  //     fix for one of Lowell's early uses of CRhinoPolyEdges
  //     however, this will cause lots of crashes now that
  //     all commands have to deal with polyedges and the code
  //     in those commands assumes that DuplicateCurve() returns
  //     a valid stand-alone curve.  If you end up here and
  //     wish this code still worked the old way, please get
  //     in touch with Dale Lear and we'll find a way to get
  //     your code to work.

	// NO //  int cnt = Count();
  // NO //  ON_SimpleArray<double> t(cnt+1);
	// NO //  ON_PolyEdgeCurve* dup_crv = new ON_PolyEdgeCurve();
  // NO //  
  // NO //  t.Append(Domain()[0]);
  // NO // 
  // NO // for( int i=0; i<cnt; i++)
  // NO //  {
  // NO // 	const ON_Curve* seg = SegmentCurve(i);
  // NO //    if ( seg )
  // NO //    {
  // NO //      t.Append(SegmentDomain(i)[1]);
  // NO //      dup_crv->ON_PolyCurve::Append( seg->DuplicateCurve() );
  // NO //    }
  // NO // }	
  // NO // 
  // NO //  if( cnt > 0 && cnt+1 == t.Count() )
  // NO //  {
  // NO //    dup_crv->SetParameterization( t.Array() );
  // NO //  }
  // NO // 
  // NO //  dup_crv->m_ev_srf_tan_mode = m_ev_srf_tan_mode;
  // NO //  dup_crv->m_is_closed_helper = m_is_closed_helper;
  // NO // 
  // NO // return dup_crv;
}

ON_BOOL32 ON_PolyEdgeSegment::IsClosed(void) const
{
  ON_BOOL32 rc = ON_CurveProxy::IsClosed();
  if ( !rc
       && m_edge
       && m_edge->m_vi[0] == m_edge->m_vi[1]
       && m_edge->ProxyCurve() == ProxyCurve()
       && m_edge->ProxyCurveDomain() == ProxyCurveDomain()
       && 0 != ProxyCurve()
       && ProxyCurve()->Domain() == ProxyCurveDomain()
       )
  {
    rc = m_edge->IsClosed();
  }
  return rc;
}

// 7-1-03 lw added override to unset cached closed flag
// when a segment is removed
ON_BOOL32 ON_PolyEdgeCurve::Remove( int segment_index )
{
  ON_BOOL32 rc = ON_PolyCurve::Remove( segment_index);
  //if( rc)
  //  m_is_closed_helper = 0;  // Cached closed flag...
  return rc;
}
ON_BOOL32 ON_PolyEdgeCurve::Remove( )
{
  return Remove(Count()-1);
}

ON_BOOL32 ON_PolyEdgeCurve::IsClosed(void) const
{
  ON_BOOL32 rc = ON_PolyCurve::IsClosed();

  if ( !rc && SegmentCount() > 1 )
  {
    // Since the segments that make up a ON_PolyEdgeCurve
    // cannot have their ends matched (becuase the curves
    // belong to objects alread in the rhino model), 
    // the IsClosed() test has to tolerate larger gaps
    // in brep topology.
    //
    // If the start and end segments are edges that belong
    // to the same brep, then they "join" if and only if 
    // they share a vertex.
    const ON_PolyEdgeSegment* seg0 = SegmentCurve(0);
    const ON_PolyEdgeSegment* seg1 = SegmentCurve(SegmentCount()-1);

    const ON_BrepEdge* edge0 = seg0->Edge();
    const ON_BrepEdge* edge1 = seg1->Edge();
    
    if ( edge0 && edge1 && edge0->Brep() == edge1->Brep() )
    {
      // check for topological closure
      //
      // Do NOT add a test for sloppy geometric closure here.
      // If the edges are in the same brep and they don't 
      // meet topologicially, then there is something in the
      // brep that is separating the edges.
      int evi0 = seg0->ReversedEdgeDir() ? 1 : 0;
      int evi1 = seg1->ReversedEdgeDir() ? 0 : 1;
      double et0 = seg0->EdgeParameter(seg0->Domain()[0]);
      double et1 = seg1->EdgeParameter(seg1->Domain()[1]);
      if ( et0 != ON_UNSET_VALUE && et1 != ON_UNSET_VALUE )
      {
        ON_Interval edom0 = edge0->Domain();
        ON_Interval edom1 = edge1->Domain();
        if ( et0 == edom0[evi0] && et1 == edom1[evi1] )
        {
          // The polyedge starts at the (evi0?end:start) of edge0
          // and ends at the (ev1?end:start) of edge1.
          if ( edge0->m_vi[evi0] == edge1->m_vi[evi1] )
          {
            // the polyedge start/ends at a common vertex
            rc = true;
          }
        }
        else if ( edge0 == edge1 
                  && fabs(et0-et1) <= ON_ZERO_TOLERANCE 
                  && edom0.Includes(et0,true)
                  && edom1.Includes(et1,true)
                  )
        {
          // The start/end of the polyedge is an interior point
          // of a single edge.  (This happens when the "seam" gets
          // adjusted to be inside of an edge.)  It is unlikely that
          // ON_PolyCurve::IsClosed() would return false in this
          // case, but this check should keep everybody happy.
          rc = true;
        }
      }
    }
  }

  return rc;
}

bool ON_PolyEdgeCurve::Create( const ON_BrepTrim* trim, const ON_UUID& object_id )
{
  bool rc = false;
  Destroy();
  //m_is_closed_helper = 0;
  if ( trim )
  {
    ON_PolyEdgeSegment* segment = new ON_PolyEdgeSegment();
    rc = segment->Create(trim,object_id);
    if (rc )
      Append(segment);
    else
      delete segment;
  }
  return rc;
}

bool ON_PolyEdgeCurve::Create( const ON_Curve* curve, const ON_UUID& object_id )
{
  bool rc = false;
  Destroy();
  //m_is_closed_helper = 0;
  if ( curve )
  {
    ON_PolyEdgeSegment* segment = new ON_PolyEdgeSegment();
    rc = segment->Create(curve,object_id);
    if (rc )
      Append(segment);
    else
      delete segment;
  }
  return rc;
}

int ON_PolyEdgeCurve::SegmentCount() const
{
  return ON_PolyCurve::Count();
}

ON_PolyEdgeSegment* ON_PolyEdgeCurve::SegmentCurve(
  int segment_index
  ) const
{
  return ON_PolyEdgeSegment::Cast(ON_PolyCurve::SegmentCurve(segment_index));
}


ON_PolyEdgeSegment* ON_PolyEdgeCurve::operator[](int segment_index) const
{
  return SegmentCurve(segment_index);
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


ON_BOOL32 ON_PolyEdgeCurve::Prepend( ON_PolyEdgeSegment* new_segment )
{
  DestroyRuntimeCache();
  ON_BOOL32 rc = false;
  if ( new_segment )
  {
    if ( Count() > 0 )
    {
      // keep segment domains in synch with polycurve domain
      // so that parameter bookkeeping is easy.
      ON_Interval cdom = Domain();
      ON_Interval sdom = new_segment->Domain();
      if ( sdom[1] != cdom[0] )
      {
        sdom[0] = cdom[0] - sdom.Length();
        sdom[1] = cdom[0];
        new_segment->SetDomain(sdom[0],sdom[1]);
      }
    }
    rc = ON_PolyCurve::Prepend(new_segment);
  }
  return rc;
}

ON_BOOL32 ON_PolyEdgeCurve::Append( ON_PolyEdgeSegment* new_segment )
{
  DestroyRuntimeCache();
  ON_BOOL32 rc = false;
  if ( new_segment )
  {
    //m_is_closed_helper = 0;
    if ( Count() > 0 )
    {
      // keep segment domains in synch with polycurve domain
      // so that parameter bookkeeping is easy.
      ON_Interval cdom = Domain();
      ON_Interval sdom = new_segment->Domain();
      if ( sdom[0] != cdom[1] )
      {
        sdom[1] = cdom[1] + sdom.Length();
        sdom[0] = cdom[1];
        new_segment->SetDomain(sdom[0],sdom[1]);
      }
    }
    rc = ON_PolyCurve::Append(new_segment);
  }
  return rc;
}

ON_BOOL32 ON_PolyEdgeCurve::Insert( 
         int segment_index,
         ON_PolyEdgeSegment* new_segment
         )
{
  DestroyRuntimeCache();
  ON_BOOL32 rc = false;
  if ( segment_index > 0 )
  {
    //m_is_closed_helper = 0;
    rc = ON_PolyCurve::Insert(segment_index,new_segment);
    if ( rc )
    {
      int i;
      for ( i = segment_index; i < Count(); i++ )
      {
        ON_PolyEdgeSegment* seg = SegmentCurve(i);
        ON_Interval d = SegmentDomain(i);
        seg->SetDomain(d[0],d[1]);
      }
    }
  }
  else if ( segment_index == 0 )
    rc = Prepend(new_segment);
  return rc;
}

const ON_BrepEdge* ON_PolyEdgeCurve::EdgeAt(double t) const
{
  ON_PolyEdgeSegment* seg = SegmentCurve( SegmentIndex(t) );
  return seg ? seg->Edge() : 0;
}

const ON_BrepTrim* ON_PolyEdgeCurve::TrimAt(double t) const
{
  ON_PolyEdgeSegment* seg = SegmentCurve( SegmentIndex(t) );
  return seg ? seg->Trim() : 0;
}

const ON_Brep*     ON_PolyEdgeCurve::BrepAt(double t) const
{
  ON_PolyEdgeSegment* seg = SegmentCurve( SegmentIndex(t) );
  return seg ? seg->Brep() : 0;
}

const ON_BrepFace* ON_PolyEdgeCurve::FaceAt(double t) const
{
  ON_PolyEdgeSegment* seg = SegmentCurve( SegmentIndex(t) );
  return seg ? seg->Face() : 0;
}

const ON_Surface*  ON_PolyEdgeCurve::SurfaceAt(double t) const
{
  ON_PolyEdgeSegment* seg = SegmentCurve( SegmentIndex(t) );
  return seg ? seg->Surface() : 0;
}

ON_Surface::ISO ON_PolyEdgeCurve::IsoType( double t) const
{
  ON_PolyEdgeSegment* seg = SegmentCurve( SegmentIndex(t) );
  return seg ? seg->IsoType() : ON_Surface::not_iso;
}


double ON_PolyEdgeCurve::EdgeParameter(double t) const
{
  double edge_t = ON_UNSET_VALUE;
  int segment_index = SegmentIndex(t);
  ON_PolyEdgeSegment* seg = SegmentCurve( segment_index );
  if ( seg )
  {
    ON_Interval pdom = SegmentDomain(segment_index);
    ON_Interval sdom = seg->Domain();
    if ( sdom != pdom )
    {
      double s = pdom.NormalizedParameterAt(t);
      t = sdom.ParameterAt(s);
    }
    edge_t = seg->EdgeParameter(t);
  }
  return edge_t;
}

// Test if there are any surface edges in the polyedge
bool ON_PolyEdgeCurve::ContainsAnyEdges() const
{
  int i, count = SegmentCount();
  for( i = 0; i < count; i++)
  {
    ON_PolyEdgeSegment* segment = SegmentCurve(i);
    if( 0 != segment && NULL != segment->Edge())
    {
      return true;
    }
  }
  return false;
}

// Test if all segments of the polyedge are surface edges
bool ON_PolyEdgeCurve::ContainsAllEdges() const
{
  int i, count = SegmentCount();
  for( i = 0; i < count; i++)
  {
    ON_PolyEdgeSegment* segment = SegmentCurve(i);
    if( NULL == segment || NULL == segment->Edge())
    {
      return false;
    }
  }
  return true;
}

int ON_PolyEdgeCurve::FindEdge( const ON_BrepEdge* edge) const
{
  int rc = -1;
  if ( 0 != edge )
  {
    int i, count = SegmentCount();
    for( i = 0; i < count; i++)
    {
      ON_PolyEdgeSegment* segment = SegmentCurve(i);
      if ( 0 != segment && edge == segment->Edge() )
      {
        rc = i;
        break;
      }
    }
  }
  return rc;
}

int ON_PolyEdgeCurve::FindTrim( const ON_BrepTrim* trim) const
{
  int rc = -1;
  if ( 0 != trim )
  {
    int i, count = SegmentCount();
    for( i = 0; i < count; i++)
    {
      ON_PolyEdgeSegment* segment = SegmentCurve(i);
      if ( 0 != segment && trim == segment->Trim() )
      {
        rc = i;
        break;
      }
    }
  }
  return rc;
}

int ON_PolyEdgeCurve::FindCurve( const ON_Curve* curve) const
{
  int rc = -1;
  if ( 0 != curve )
  {
    int i, count = SegmentCount();
    for( i = 0; i < count; i++)
    {
      ON_PolyEdgeSegment* segment = SegmentCurve(i);
      if (    0 != segment 
           && (curve == segment || curve == segment->ProxyCurve() || curve == segment->Edge()) )
      {
        rc = i;
        break;
      }
    }
  }
  return rc;
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


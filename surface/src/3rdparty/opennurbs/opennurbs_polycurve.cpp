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

ON_OBJECT_IMPLEMENT(ON_PolyCurve,ON_Curve,"4ED7D4E0-E947-11d3-BFE5-0010830122F0");

ON_PolyCurve::ON_PolyCurve()
{
}

ON_PolyCurve::ON_PolyCurve( int capacity )
              : m_segment(capacity), m_t(capacity+1)
{
  m_segment.Zero();
}

ON_PolyCurve::ON_PolyCurve( const ON_PolyCurve& src )
              : ON_Curve(src), m_segment(src.Count()), m_t(src.Count()+1)
{
  *this = src;
}

void ON_PolyCurve::Destroy()
{
  // release memory
  m_segment.Destroy();
  m_t.Destroy();
}

ON_PolyCurve::~ON_PolyCurve()
{
  Destroy();
}

void ON_PolyCurve::EmergencyDestroy()
{
  m_segment.EmergencyDestroy();
  m_t.EmergencyDestroy();
}

void ON_PolyCurve::DestroyRuntimeCache( bool bDelete )
{
  ON_Curve::DestroyRuntimeCache(bDelete);
  int i, count = m_segment.Count();
  for ( i = 0; i < count; i++ )
  {
    ON_Curve* segment_curve = m_segment[i];
    if ( 0 != segment_curve && this != segment_curve )
      segment_curve->DestroyRuntimeCache(bDelete);
  }
}

unsigned int ON_PolyCurve::SizeOf() const
{
  unsigned int sz = ON_Curve::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Curve));
  sz += m_t.SizeOfArray();
  sz += m_segment.SizeOfArray();
  int i, count = m_segment.Count();
  for ( i = 0; i < count; i++ )
  {
    const ON_Curve* crv = m_segment[i];
    if ( crv )
      sz += crv->SizeOf();
  }
  return sz;
}

ON__UINT32 ON_PolyCurve::DataCRC(ON__UINT32 current_remainder) const
{
  int i, count = m_segment.Count();
  for ( i = 0; i < count; i++ )
  {
    const ON_Curve* crv = m_segment[i];
    if ( crv )
      current_remainder = crv->DataCRC(current_remainder);
  }
  current_remainder = m_t.DataCRC(current_remainder);
  return current_remainder;
}


ON_PolyCurve& ON_PolyCurve::operator=( const ON_PolyCurve& src )
{
  if ( this != &src ) {
    ON_Curve::operator=(src);
    const int segment_capacity = m_segment.Capacity();
    ON_Curve** segment = m_segment.Array();
    int i;
    for ( i = 0; i < segment_capacity; i++ ) {
      if ( segment[i] ) {
        delete segment[i];
        segment[i] = 0;
      }
    }
    src.m_segment.Duplicate(m_segment);
    m_t.SetCount(0);
    m_t.SetCapacity(src.m_t.Count());
    m_t.Zero();
    m_t = src.m_t;
  }
  return *this;
}

int ON_PolyCurve::Dimension() const
{
  const ON_Curve* p = SegmentCurve(0);
  return (p) ? p->Dimension() : 0;
}

ON_BOOL32 
ON_PolyCurve::GetBBox( // returns true if successful
         double* boxmin,    // minimum
         double* boxmax,    // maximum
         ON_BOOL32 bGrowBox
         ) const
{
  const int count = Count();
  int segment_index;
  ON_BOOL32 rc = (count>0) ? true : false;
  for ( segment_index = 0; segment_index < count && rc; segment_index++ ) {
    rc = m_segment[segment_index]->GetBBox( boxmin, boxmax, bGrowBox );
    bGrowBox = true;
  }
  return rc;
}

ON_BOOL32
ON_PolyCurve::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
  DestroyRuntimeCache();
  const int count = Count();
  int segment_index;
  ON_BOOL32 rc = (count>0) ? true : false;
  for ( segment_index = 0; segment_index < count && rc; segment_index++ ) {
    rc = m_segment[segment_index]->Transform( xform );
  }
  return rc;
}

bool ON_PolyCurve::IsDeformable() const
{
  bool rc = true;
  const int count = Count();
  int segment_index;
  for ( segment_index = 0; segment_index < count ; segment_index++ )
  {
    const ON_Curve* seg = m_segment[segment_index];
    if ( seg && !seg->IsDeformable() )
    {
      rc = false;
      break;
    }
  }
  return rc;
}

bool ON_PolyCurve::MakeDeformable()
{
  bool rc = true;
  bool bDestroyRuntimeCache = false;
  const int count = Count();
  int segment_index;
  for ( segment_index = 0; segment_index < count ; segment_index++ )
  {
    ON_Curve* seg = m_segment[segment_index];
    if ( seg && !seg->IsDeformable() )
    {
      bDestroyRuntimeCache = true;
      if ( !seg->MakeDeformable() )
      {
        ON_NurbsCurve* nurbs_curve = seg->NurbsCurve();
        if ( nurbs_curve )
        {
          delete seg;
          m_segment[segment_index] = nurbs_curve;
        }
        else
          rc = false;
      }
    }
  }
  if ( bDestroyRuntimeCache )
    DestroyRuntimeCache();
  return rc;
}


ON_BOOL32
ON_PolyCurve::SwapCoordinates( int i, int j )
{
  const int count = Count();
  int segment_index;
  ON_BOOL32 rc = (count>0) ? true : false;
  for ( segment_index = 0; segment_index < count && rc; segment_index++ ) {
    rc = m_segment[segment_index]->SwapCoordinates( i, j );
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 ON_PolyCurve::IsValid( ON_TextLog* text_log ) const
{
  return IsValid(false,text_log);
}

bool ON_PolyCurve::IsValid( bool bAllowGaps, ON_TextLog* text_log ) const
{
  const int count = Count();
  const int dim = Dimension();
  ON_3dPoint p0, p1;
  int segment_index;
  if ( count <= 0 || dim <= 0 )
  {
    if ( text_log )
      text_log->Print("Polycurve segment count = %d and dim = %d\n",count,dim);
    return ON_IsNotValid();
  }

  if ( m_t.Count() != count+1 )
  {
    if ( text_log )
      text_log->Print("Polycurve segment count = %d and m_t.Count()=%d (should be segment count+1)\n",
                      count,m_t.Count());
    return ON_IsNotValid();
  }

  for ( segment_index = 0; segment_index < count; segment_index++ ) 
  {
    if ( 0 == m_segment[segment_index] )
    {
      if ( text_log )
      {
        text_log->Print("Polycurve segment[%d] is null.\n",segment_index);
      }
      return ON_IsNotValid();
    }

    if ( !m_segment[segment_index]->IsValid( text_log ) )
    {
      if ( text_log )
      {
        text_log->Print("Polycurve segment[%d] is not valid.\n",segment_index);
      }
      return ON_IsNotValid();
    }

    int seg_dim = m_segment[segment_index]->Dimension();
    if ( seg_dim != dim )
    {
      if ( text_log )
        text_log->Print("Polycurve segment[%d]->Dimension()=%d (should be %d).\n",segment_index,seg_dim,dim);
      return ON_IsNotValid(); // all segments must have same dimension
    }

    if ( m_t[segment_index] >= m_t[segment_index+1] )
    {
      if ( text_log )
        text_log->Print("Polycurve m_t[%d]=%g and m_t[%d]=%g (should be increasing)\n",
                         segment_index,   m_t[segment_index],
                         segment_index+1, m_t[segment_index+1]);
      return ON_IsNotValid(); // segment domain must be non-empty
    }

    if ( count > 1 && !bAllowGaps && m_segment[segment_index]->IsClosed() ) 
    {
      if ( text_log )
        text_log->Print("Polycurve segment[%d] is closed (%d segments).\n",segment_index,count);
      return ON_IsNotValid(); // closed segments not permitted in multi segment curve
    }
  }

  if ( !bAllowGaps )
  {
    // check for gaps
    int gap_index = FindNextGap(0);
    if ( gap_index > 0 )
    {
      p0 = m_segment[gap_index-1]->PointAtEnd();
      p1 = m_segment[gap_index]->PointAtStart();
      double d = p0.DistanceTo(p1);
      if ( text_log )
        text_log->Print("Polycurve end of segment[%d] != start of segment[%d] (distance=%g)\n",
                        gap_index-1, gap_index, d );
      return ON_IsNotValid(); // not contiguous
    }
  }

  return true;
}

void ON_PolyCurve::Dump( ON_TextLog& dump ) const
{
  const int count = Count();
  int i;

  ON_3dPoint segment_start = ON_3dPoint::UnsetPoint;
  ON_3dPoint segment_end = ON_3dPoint::UnsetPoint;
  double gap;

  dump.Print( "ON_PolyCurve segment count = %d\n", count );
  dump.PushIndent();
  for ( i = 0; i < count; i++ )
  {
    if ( 0 != m_segment[i] )
      segment_start = m_segment[i]->PointAtStart();
    else
      segment_start = ON_3dPoint::UnsetPoint;
    gap = (segment_start.IsValid() && segment_end.IsValid())
        ? segment_start.DistanceTo(segment_end)
        : ON_UNSET_VALUE;
    dump.Print( "Segment %d: (%g,%g)", i+1, m_t[i], m_t[i+1] );
    if ( i > 0 )
    {
      if ( ON_IsValid(gap) )
        dump.Print(" gap = %.17g",gap);
      else if ( !segment_start.IsValid() )
        dump.Print(" invalid segment curve");
      else if ( !segment_end.IsValid() )
        dump.Print(" invalid previous segment curve");
    }
    dump.Print("\n");

    dump.PushIndent();
    if ( 0 == m_segment[i] )
    {
      dump.Print("null curve pointer\n");
      segment_end = ON_3dPoint::UnsetPoint;
    }
    else
    {
      m_segment[i]->Dump(dump);
      segment_end = m_segment[i]->PointAtEnd();
    }
    dump.PopIndent();
  }
  dump.PopIndent();
}

ON_BOOL32 ON_PolyCurve::Write(
       ON_BinaryArchive& file // open binary file
     ) const
{
  // NOTE - if changed, check legacy I/O code
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if (rc) {
    int reserved1 = 0;
    int reserved2 = 0;
    const int count = Count();
    int segment_index;

    rc = file.WriteInt( count );
    if (rc) file.WriteInt(reserved1);
    if (rc) file.WriteInt(reserved2);
    if (rc) {
      // may be set in future
      ON_BoundingBox bbox;
      rc = file.WriteBoundingBox(bbox);
    }
    if (rc) rc = file.WriteArray( m_t );

    for ( segment_index = 0; segment_index < count && rc; segment_index++ ) {
      rc = file.WriteObject( *SegmentCurve(segment_index) );
    }


  }
  return rc;
}

ON_BOOL32 ON_PolyCurve::Read(
       ON_BinaryArchive& file  // open binary file
     )
{
  // NOTE - if changed, check legacy I/O code
  Destroy();
  int major_version = 0;
  int minor_version = 0;
  
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  
  if (rc) 
  {
    ON_Object* obj;
    ON_Curve* crv;
    int segment_index;
    int segment_count = 0;
    int reserved1 = 0;
    int reserved2 = 0;
    rc = file.ReadInt(&segment_count);
    if (rc) rc = file.ReadInt(&reserved1);
    if (rc) rc = file.ReadInt(&reserved2);
    if (rc) {
      // may be set in future
      ON_BoundingBox bbox;
      rc = file.ReadBoundingBox(bbox);
    }
    if (rc) rc = file.ReadArray( m_t );

    for ( segment_index = 0; segment_index < segment_count && rc; segment_index++ ) {
      obj = 0;
      crv = 0;
      rc = file.ReadObject( &obj );
      if (rc) {
        crv = ON_Curve::Cast(obj);
        if (crv) {
          //Append(crv); - this one adds to m_t[]
          m_segment.Append(crv);
        }
        else {
          ON_ERROR("ON_PolyCurve::Read() - non ON_Curve object in segment list\n");
          delete obj;
          rc = false;
        }
      }
    }

    if ( rc && m_segment.Count()>0 && 
								m_segment.Count()==segment_count && m_t.Count()==segment_count+1)
    {
      // remove "fuzz" in m_t[] domain array that is in some older files.
      double s, t, d0, d1, fuzz;
      ON_Interval in0, in1;
      in1 = SegmentCurve(0)->Domain();
      d1 = in1.Length();
      for ( segment_index = 1; segment_index < segment_count; segment_index++ )
      {
        t = m_t[segment_index];
        in0 = in1;
        d0 = d1;
        in1 = SegmentCurve(segment_index)->Domain();
        d1 = in1.Length();
        s = in0[1];
        if ( s != t && s == in1[0] && t > in0[0] && t < in1[1] )
        {
          fuzz = ((d0<=d1)?d0:d1)*ON_SQRT_EPSILON;
          if ( fabs(t-s) <= fuzz )
            m_t[segment_index] = s;
        }
      }
      fuzz = d1*ON_SQRT_EPSILON;
      t = m_t[segment_count];
      s = in1[1];
      if ( s != t && t > in1[0] && fabs(s-t) <= fuzz )
        m_t[segment_count] = s;
    }
  }

  if (rc && file.ArchiveOpenNURBSVersion() < 200304080 )
  {
    // 8 April 2003 Dale Lear:
    //   Some archives written by earlier versions
    //   of Rhino had nested polycurves and polycurves with
    //   zero length segments.  This code cleans up
    //   those problems.  See RR 8932.
    RemoveNesting();
    //RemoveShortSegments(ON_ZERO_TOLERANCE);
  }

  return rc;
}


ON_Curve* ON_PolyCurve::DuplicateCurve() const
{
	// Call DuplicateCurve on each segment to construct duplicate curve.
	int cnt = Count();
	ON_PolyCurve* dup_crv = new ON_PolyCurve( cnt );
  dup_crv->CopyUserData(*this);
	for( int i=0; i<cnt; i++){
		const ON_Curve* seg = SegmentCurve(i);
		if(seg)
			dup_crv->Append( seg->DuplicateCurve() );
	}	
	if( cnt == dup_crv->Count() )
		dup_crv->SetParameterization( m_t);
	return dup_crv;
}


ON_Interval ON_PolyCurve::Domain() const
{
  ON_Interval d;
  const int count = Count();
  if ( count > 0 && count+1 == m_t.Count() && m_t[0] < m_t[count] ) {
    d.Set(m_t[0],m_t[count]);
  }
  return d;
}

ON_BOOL32 ON_PolyCurve::SetDomain( double t0, double t1 )
{
  ON_Interval d0 = Domain();
  ON_Interval d1(t0,t1);
  ON_BOOL32 rc = d1.IsIncreasing();
  if ( rc && d0 != d1 )
  {
    int i, count = m_t.Count();
    double s;
    for ( i = 0; i < count; i++ )
    {
      s = d0.NormalizedParameterAt( m_t[i] );
      m_t[i] = d1.ParameterAt( s );
    }
  	DestroyRuntimeCache();
  }
  return rc;
}


bool ON_PolyCurve::ChangeDimension( int desired_dimension )
{
  int i, count = m_segment.Count();
  bool rc = (count>0);
  for ( i = 0; i < count; i++ )
  {
    ON_Curve* curve = m_segment[i];
    if ( 0 != curve )
    {
      if ( !curve->ChangeDimension(desired_dimension) )
        rc = false;
    }
    else
      rc = false;
  }
  return rc;
}

bool ON_PolyCurve::SetParameterization( const double* t )
{
  bool rc = false;
  int i, count = m_segment.Count()+1;
  if ( count >= 2 && 0 != t && ON_UNSET_VALUE != t[0] )
  {
    for ( i = 1; i < count; i++ )
    {
      if ( t[i] == ON_UNSET_VALUE )
        break;
      if ( t[i-1] >= t[i] )
        break;
    }
    if ( i == count )
    {
      m_t.Reserve(count);
      m_t.SetCount(0);
      m_t.Append( count, t );
      rc = true;
    }
  }
  return rc;
}

ON_BOOL32 ON_PolyCurve::ChangeClosedCurveSeam( double t )
{
  ON_BOOL32 rc = IsClosed();
  if ( rc )
  {
  	DestroyRuntimeCache();
    rc = false;
    const int old_count = Count();
    const ON_Interval old_dom = Domain();
    ON_Curve* scrv = 0;
    if ( old_count == 1 )
    {
      scrv = SegmentCurve(0);
      if ( scrv )
      {
        ON_Interval sdom = scrv->Domain();
        double s = ( old_dom == sdom )
                 ? t
                 : sdom.ParameterAt( old_dom.NormalizedParameterAt(t) );
        rc = scrv->ChangeClosedCurveSeam(s);
        if ( rc )
          SetDomain( t, t + old_dom.Length() );
      }
    }
    else
    {
      double k = t;
      if ( !old_dom.Includes(t) )
      {
        double s = old_dom.NormalizedParameterAt(t);
        s = fmod(s,1.0);
        if ( s < 0.0 )
          s += 1.0;
        k = old_dom.ParameterAt(s);
      }
      if ( old_dom.Includes(k,true) )
      {
        int segment_index = ON_NurbsSpanIndex(2,old_count+1,m_t.Array(),k,0,0);
        scrv = m_segment[segment_index];
        if ( k < m_t[segment_index] )
          return false;
        if ( k >= m_t[segment_index+1] )
          return false;
        int new_count = (k==m_t[segment_index]) ? old_count : old_count+1;
        ON_Curve* sleft = 0;
        ON_Curve* sright = 0;
        if ( new_count > old_count )
        {
					ON_Interval subdom(m_t[segment_index], m_t[segment_index+1]);
					double nt = subdom.NormalizedParameterAt(k);
					ON_Interval Segdom = scrv->Domain();
					double segt = Segdom.ParameterAt(nt);
          rc = scrv->Split( segt, sleft, sright );

//				Greg Arden 6 May 2003. Fixes TRR#10332.  If split fails we break the
//				curve between segments and adjust the parameterization
					if(!rc){
						if(nt>.5){
							segment_index++;
							if(segment_index<old_count)
								scrv = m_segment[segment_index];		
							else
								scrv = NULL;
						}
						new_count--;
					}
        }
        if(new_count==old_count)
        {
          sright = scrv;
          scrv = 0;
          rc = true;
        }
        if ( rc && segment_index<old_count)
        {
          m_segment[segment_index] = 0;//todo
          ON_SimpleArray<ON_Curve*> new_c(new_count);
          ON_SimpleArray<double> new_t(new_count+1);
          new_c.Append(sright);
          new_t.Append(k);
          new_c.Append( old_count-segment_index-1, m_segment.Array()+segment_index+1);
          new_t.Append( old_count-segment_index-1, m_t.Array()+segment_index+1);
          int j = new_t.Count();
          new_c.Append( segment_index, m_segment.Array() );
          new_t.Append( segment_index, m_t.Array() );
          if ( sleft )
          {
            new_c.Append(sleft);
            new_t.Append(m_t[segment_index]);
          }
          new_t.Append(k);
          double d = old_dom.Length();
          while (j < new_t.Count() )
          {
            new_t[j] += d;
            j++;
          }

          // take care when moving new_c pointers to m_segment
          // so that we don't delete any curves.
          memset( m_segment.Array(), 0, m_segment.Capacity()*sizeof( *m_segment.Array() ) );
          m_segment.SetCount(0);
          m_segment.Append( new_c.Count(), new_c.Array() );
          m_t = new_t;
          if ( scrv )
          {
            delete scrv;
            scrv = 0;
          }
        }
      }
      else
      {
        // k is already the start or  end of the existing curve
        rc = true;
      }
      if ( rc )
        SetDomain( t, t + old_dom.Length() );
    }
  }
  return rc;
}

int ON_PolyCurve::SpanCount() const
{
  int span_count = 0;
  const int segment_count = Count();
  int i, j;
  for ( i = 0; i < segment_count; i++  ) {
    if ( !m_segment[i] )
      return false;
    j = m_segment[i]->SpanCount();
    if ( j == 0 )
      return 0;
    span_count += j;
  }
  return span_count;
}

ON_BOOL32 ON_PolyCurve::GetSpanVector( // span "knots" 
       double* s // array of length SpanCount() + 1 
       ) const
{
  ON_Interval sp;
  double t;
  const int segment_count = Count();
  int i, j, k;
  for ( i = 0; i < segment_count; i++  ) {
    if ( !m_segment[i] )
      return false;
    j = m_segment[i]->SpanCount();
    if ( j == 0 )
      return 0;
    if ( !m_segment[i]->GetSpanVector( s ) )
      return false;
    sp.Set( m_t[i], m_t[i+1] );
		ON_Interval segloc(s[0],s[j]);
    if ( sp.Min() != s[0] || sp.Max() != s[j] ) {
      for ( k = 0; k <= j; k++ ) {
        t = segloc.NormalizedParameterAt(s[k]);
        s[k] = sp.ParameterAt(t);
      }
    }

    s += j;
  }
  return true;
}

int ON_PolyCurve::Degree() const
{
  const int segment_count = Count();
  int span_degree = 0;
  int segment_index, d;
  for ( segment_index = 0; segment_index < segment_count; segment_index++  ) {
    if ( !m_segment[segment_index] )
      return false;
    d = m_segment[segment_index]->Degree();
    if ( d <= 0 )
      return 0;
    if ( d > span_degree )
      span_degree = d;
  }
  return span_degree;
}


ON_BOOL32
ON_PolyCurve::IsLinear( // true if curve locus is a line segment
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = false;
  int i, count = Count();
	if ( count==1)
		return m_segment[0]->IsLinear(tolerance);
  
	else if ( count > 1 ) {
    rc = true;
    for ( i = 0; rc && i < count; i++ ) {
      if ( !m_segment[i] )
        rc = false;
      else
        rc = m_segment[i]->IsLinear(tolerance);
    
    }
    if (rc)
      rc = ON_Curve::IsLinear(tolerance);
  }
  return rc;
}

int ON_PolyCurve::IsPolyline(
      ON_SimpleArray<ON_3dPoint>* pline_points,
      ON_SimpleArray<double>* pline_t
      ) const
{
  int i, seg_i, seg_rc;
  ON_Interval sdom, cdom;
  int rc = 0;
  if ( pline_points )
    pline_points->SetCount(0);
  if ( pline_t )
    pline_t->SetCount(0);
  const int seg_count = Count();
  if ( seg_count == 1 )
  {
    if ( m_segment[0] )
      rc = m_segment[0]->IsPolyline(pline_points,pline_t);
    if (pline_t && rc > 0)
    {
      sdom.Set(m_t[0],m_t[1]);
      cdom = m_segment[0]->Domain();
      if ( sdom != cdom )
      {
        for ( i = 0; i < pline_t->Count(); i++ )
          (*pline_t)[i] = sdom.ParameterAt(cdom.NormalizedParameterAt((*pline_t)[i]));
      }
    }
  }
  else if (seg_count > 1 )
  {
    ON_SimpleArray<ON_3dPoint> seg_points;
    ON_SimpleArray<double> seg_t;
    for ( seg_i = 0; seg_i < seg_count; seg_i++ )
    {
      seg_points.SetCount(0);
      seg_t.SetCount(0);
      seg_rc = m_segment[seg_i]->IsPolyline(pline_points?&seg_points:0,pline_t?&seg_t:0);
      if ( seg_rc < 2 )
      {
        if ( pline_points )
          pline_points->SetCount(0);
        if ( pline_t )
          pline_t->SetCount(0);
        rc = 0;
        break;
      }
      rc += seg_rc;
      if ( seg_i )
        rc--;
      if ( pline_t )
      {
        sdom.Set( m_t[seg_i], m_t[seg_i+1] );
        cdom = m_segment[seg_i]->Domain();
        if ( sdom != cdom )
        {
          for ( i = 0; i < seg_t.Count(); i++ )
            seg_t[i] = sdom.ParameterAt(cdom.NormalizedParameterAt(seg_t[i]));
        }
        if ( pline_t->Count() > 0 )
          pline_t->Remove();
        pline_t->Append( seg_t.Count(), seg_t.Array() );
      }
      if ( pline_points )
      {
        if ( pline_points->Count() > 0 )
          pline_points->Remove();
        pline_points->Append( seg_points.Count(), seg_points.Array() );
      }
    }
    if(IsClosed() && pline_points && pline_points->Count() > 3)
    {
      // GBA 2/26/03. Make closed polylines spot on closed
      *pline_points->Last() = *pline_points->First();
    }
  }
  return rc;
}


ON_BOOL32
ON_PolyCurve::IsArc( // true if curve locus in an arc or circle
      const ON_Plane* plane, // if not NULL, test is performed in this plane
      ON_Arc* arc,         // if not NULL and true is returned, then arc
                              // arc parameters are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  bool rc = false;
  if ( 1 == m_segment.Count() && 0 != m_segment[0] )
  {
    rc = m_segment[0]->IsArc( plane, arc, tolerance )?true:false;
  }
  return rc;
}


static bool GetTestPlane( const ON_Curve& curve, ON_Plane& plane )
{
  int i, j;
  ON_3dPoint P, Q, R;
  ON_3dVector X;
  ON_Interval d = curve.Domain();
  if ( !curve.Ev1Der( d[0], P, X ) )
  {
    return false;
  }
  if ( !X.Unitize() )
  {
    return false;
  }

  Q = P+X;
  for ( i = 2; i <= 16; i += 2 )
  {
    for ( j = 1; j < i; j += 2 )
    {
      R = curve.PointAt( d.ParameterAt( ((double)j)/((double)i) ) );
      if ( plane.CreateFromFrame( P, X, R-P ) )
        return true;
    }
  }
  return false;
}


ON_BOOL32
ON_PolyCurve::IsPlanar(
      ON_Plane* plane, // if not NULL and true is returned, then plane parameters
                         // are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  if ( Dimension() == 2 )
  {
    return ON_Curve::IsPlanar(plane,tolerance);
  }

  ON_BOOL32 rc = false;
  ON_Plane test_plane;
  const int count = Count();
  const ON_Curve* crv = FirstSegmentCurve();
  if ( count == 1 && crv )
    rc = crv->IsPlanar( plane, tolerance );
  else if ( count > 1 )
  {
    if ( IsLinear(tolerance) )
    {
      if ( plane )
      {
        ON_Line line(PointAtStart(), PointAtEnd() );
        if ( !line.InPlane( *plane, tolerance ) )
          line.InPlane( *plane, 0.0 );
      }
      return true;
    }
    if ( !GetTestPlane( *this, test_plane ) )
    {
      // 5 May 2006 Dale Lear
      //   Added additiional attempts to get a test_plane
      //   for poorly parameterized polycurves. (RR 20057 fix).
      ON_3dPoint P, Q;
      ON_3dVector X;
      if (!Ev1Der(m_t[0],P,X))
        return false;

      if ( !X.Unitize() )
      {
        X = PointAt(Domain().ParameterAt(0.5)) - P;
        if ( !X.Unitize() )
          return false;
      }

      int i;
      for ( i = 1; i < count; i++ )
      {
        if ( m_segment[i] )
        {
          Q = m_segment[i]->PointAt(m_segment[i]->Domain().ParameterAt(0.5));
          if ( test_plane.CreateFromFrame(P,X,Q-P) )
            break;
        }
      }
      if ( i >= count )
        return false;
    }
    rc = IsInPlane( test_plane, tolerance );
    if (rc && plane)
      *plane = test_plane;
  }
  return rc;
}

ON_BOOL32
ON_PolyCurve::IsInPlane(
      const ON_Plane& plane, // plane to test
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = false;
  int i, count = Count();
  for ( i = 0; i < count; i++ )
  {
    if ( !m_segment[i] )
      return false;
    rc = m_segment[i]->IsInPlane( plane, tolerance );
    if ( !rc )
      break;
  }
  return rc;
}

ON_BOOL32 
ON_PolyCurve::IsClosed() const
{
  ON_BOOL32 bIsClosed = false;
  const int count = Count();
  if ( count == 1 ) {
    // evaluation test required
    const ON_Curve* c = FirstSegmentCurve();
    if ( c )
      bIsClosed = c->IsClosed();
  }
  else if ( count > 1 ) 
  {
    // 17 May2005 Dale Lear 
    //  I added the FindNextGap(0) <= 0 test to
    //  prevent discontinuous curves from being
    //  classified as closed.
    bIsClosed = ( ON_Curve::IsClosed() && FindNextGap(0) <= 0 );
  }
  return bIsClosed;
}

static bool GetLineIsoCoordinates( const ON_Line& line, const ON_3dPoint P, ON_3dPoint& C )
{
  C.x = (line.from.x == line.to.x) ? P.x : ON_UNSET_VALUE;
  C.y = (line.from.y == line.to.y) ? P.y : ON_UNSET_VALUE;
  C.z = (line.from.z == line.to.z) ? P.z : ON_UNSET_VALUE;
  return ( ON_3dPoint::UnsetPoint != C );
}

static void LineLineTieBreaker( const ON_Line& line0, const ON_Line& line1, 
                                ON_3dPoint& Q0, ON_3dPoint& Q1 )
{
  double line0_length = line0.Length();
  double line1_length = line1.Length();

  ON_3dPoint C0, C1;
  bool bHaveIsoCoords0 = GetLineIsoCoordinates(line0,Q0,C0);
  bool bHaveIsoCoords1 = GetLineIsoCoordinates(line1,Q1,C1);
  if ( bHaveIsoCoords0 || bHaveIsoCoords1 )
  {
    for ( int i = 0; i < 3; i++ )
    {
      double c0 = C0[i];
      double c1 = C1[i];
      if ( ON_UNSET_VALUE == c0 && ON_UNSET_VALUE == c1 )
        continue;
      double c = ON_UNSET_VALUE;
      if ( c0 == c1 )
        c = c0;
      else if ( ON_UNSET_VALUE == c0 )
        c = c1;
      else if ( ON_UNSET_VALUE == c1 )
        c = c0;
      else if ( line0_length > line1_length )
        c = c0;
      else
        c = c1;
      if ( ON_UNSET_VALUE != c && ON_IsValid(c) )
      {
        Q0[i] = c;
        Q1[i] = c;
      }
    }
  }
}

static void SetLineIsoCoords( const ON_Line& line, const ON_3dPoint& P, ON_3dPoint& Q )
{
  ON_3dPoint C;
  if ( GetLineIsoCoordinates(line,P,C) )
  {
    if ( ON_UNSET_VALUE != C.x && ON_IsValid(C.x) )
      Q.x = P.x;
    if ( ON_UNSET_VALUE != C.y && ON_IsValid(C.y) )
      Q.y = P.y;
    if ( ON_UNSET_VALUE != C.z && ON_IsValid(C.z) )
      Q.z = P.z;
  }
}

static ON_NurbsCurve* ChangeArcEnd( const ON_ArcCurve* arc, ON_3dPoint P, ON_3dPoint Q, int end_index )
{
  if ( P == Q )
    return 0;

  ON_NurbsCurve* nc = arc->NurbsCurve();
  if ( 0 == nc || nc->m_cv_count < 3 )
    return 0;
  
  int cv0_dex, cv1_dex;
  if ( 1 == end_index )
  {
    cv0_dex = nc->m_cv_count-1;
    cv1_dex = cv0_dex - 1;
  }
  else
  {
    cv0_dex = 0;
    cv1_dex = cv0_dex + 1;
  }

  if ( !nc->SetCV(cv0_dex,Q) )
  {
    delete nc;
    return 0;
  }

  ON_4dPoint R;
  if ( !nc->GetCV(cv1_dex,R) )
  {
    delete nc;
    return 0;
  }

  R.x += (Q.x-P.x)*R.w;
  R.y += (Q.y-P.y)*R.w;
  R.z += (Q.z-P.z)*R.w;
  nc->SetCV(cv1_dex,R);

  return nc;
}

bool ON_PolyCurve::CloseGap( int gap_index, int )
{
  const int count = m_segment.Count();

  if ( gap_index <= 0 || gap_index >= count )
  {
    ON_ERROR("Invalid gap_index parameter.");
    return 0; // nothing to do
  }

  ON_Curve* c0 = m_segment[gap_index-1];
  ON_Curve* c1 = m_segment[gap_index];
  if ( 0 == c0 || 0 == c1 )
  {
    ON_ERROR("Null curve segments.");
    return false; // invalid polycurve
  }

  const ON_3dPoint P0 = c0->PointAtEnd();
  const ON_3dPoint P1 = c1->PointAtStart();
  if ( P0 == P1 )
    return false; // nothing to do

  ON_3dPoint Q0(P0);
  ON_3dPoint Q1(P1);

  const ON_ArcCurve* arc0 = ON_ArcCurve::Cast(c0);
  const ON_ArcCurve* arc1 = ON_ArcCurve::Cast(c1);

  if ( 0 != arc0 && 0 != arc1 )
  {
    if ( arc1->m_arc.Length() < arc0->m_arc.Length() )
      Q1 = P0;
    else
      Q0 = P1;
  }
  else if ( 0 != arc0 && 0 == arc1 )
  {
    Q1 = P0;
  }
  else if ( 0 != arc1 && 0 == arc0 )
  {
    Q0 = P1;
  }
  else
  {
    ON_Line line0, line1;
    double min_line_length = 0.0;
    double is_linear_tolerance = 0.0;
    bool bLine0 = (0 == arc0)
                ? c0->LastSpanIsLinear(min_line_length,is_linear_tolerance,&line0)
                : false;
    bool bLine1 = (0 == arc0)
                ? c1->FirstSpanIsLinear(min_line_length,is_linear_tolerance,&line1)
                : false;
    if ( bLine0 && bLine1 )
      LineLineTieBreaker(line0,line1,Q0,Q1);
    else if ( bLine0 )
      SetLineIsoCoords(line0,P0,Q1);
    else if ( bLine1 )
      SetLineIsoCoords(line1,P1,Q0);
  }

  if ( Q0.x != Q1.x )
    Q0.x = Q1.x = 0.5*(P0.x + P1.x);
  if ( Q0.y != Q1.y )
    Q0.y = Q1.y = 0.5*(P0.y + P1.y);
  if ( Q0.z != Q1.z )
    Q0.z = Q1.z = 0.5*(P0.z + P1.z);

  if ( Q0 != P0 )
  {
    if ( 0 != arc0 )
    {
      ON_NurbsCurve* nc0 = ChangeArcEnd( arc0, P0, Q0 , 1 );
      if ( nc0 )
      {
        delete m_segment[gap_index-1];
        m_segment[gap_index-1] = nc0;
        c0 = nc0;
        arc0 = 0;
      }
    }
    else
    {
      c0->SetEndPoint(Q0);
    }
  }

  if ( Q1 != P1 )
  {
    if ( 0 != arc1 )
    {
      ON_NurbsCurve* nc1 = ChangeArcEnd( arc1, P1, Q1, 0 );
      if ( nc1 )
      {
        delete m_segment[gap_index];
        m_segment[gap_index] = nc1;
        c0 = nc1;
        arc1 = 0;
      }
    }
    else
    {
      c1->SetStartPoint(Q1);
    }
  }

  return HasGapAt(gap_index-1) ? false : true;
}

int ON_PolyCurve::CloseGaps()
{
  int rc = 0;
  int segment_index0 = 0;
  int gap_index = 0;
  
  for(;;)
  {
    gap_index = FindNextGap(segment_index0);
    if ( gap_index <= segment_index0 || gap_index >= m_segment.Count() )
      break;
    if ( CloseGap(gap_index,0) )
      rc++;
    segment_index0 = gap_index;
  }

  return rc;
}

int ON_PolyCurve::HasGap() const
{
  return FindNextGap(0);
}


bool ON_PolyCurve::HasGapAt(int segment_index) const
{
  const int count = m_segment.Count();

  if ( segment_index < 0 || segment_index >= count-1 )
    return 0;

  const ON_Curve* c0 = m_segment[segment_index];
  const ON_Curve* c1 = m_segment[segment_index+1];
  if ( 0 == c0 || 0 == c1 )
    return false;

  ON_3dPoint P0 = c0->PointAtEnd();
  ON_3dPoint P1 = c1->PointAtStart();
  // Note:  The point compare test should be the same
  //        as the one used in ON_Curve::IsClosed().
  if ( false == ON_PointsAreCoincident( 3, false, &P0.x, &P1.x ) )
  {
    // To fix RR 13325 I allow a little more leeway for arcs.
    const ON_ArcCurve* arc0 = ON_ArcCurve::Cast(c0);
    const ON_ArcCurve* arc1 = ON_ArcCurve::Cast(c1);
    if ( 0 == arc0 && 0 == arc1 )
      return true; // gap

    double tol = ON_ZERO_TOLERANCE;
    const double tol0 = arc0  ? ( arc0->m_arc.radius*arc0->m_arc.AngleRadians()*1.0e-10 ) : 0.0;
    const double tol1 = arc1  ? ( arc1->m_arc.radius*arc1->m_arc.AngleRadians()*1.0e-10 ) : 0.0;
    if ( tol < tol0 ) 
      tol = tol0;
    if ( tol < tol1 ) 
      tol = tol1;
    const double d = P0.DistanceTo(P1);
    if ( d > tol )
    {
      return true; // gap
    }
  }

  return false; // no gap
}


int ON_PolyCurve::FindNextGap(int segment_index0) const
{
  if ( segment_index0 >= 0 )
  {
    const int count = m_segment.Count();
    for (int gap_index = segment_index0+1; gap_index < count; gap_index++ )
    {
      if ( HasGapAt(gap_index-1) )
        return gap_index;
    }
  }
  return 0;
}


ON_BOOL32 
ON_PolyCurve::IsPeriodic() const
{
  ON_BOOL32 bIsPeriodic = false;
  if ( Count() == 1 ) {
    const ON_Curve* c = FirstSegmentCurve();
    if ( c )
      bIsPeriodic = c->IsPeriodic();
  }
  return bIsPeriodic;
}

bool ON_PolyCurve::GetNextDiscontinuity( 
        ON::continuity c,
        double t0,
        double t1,
        double* t,
        int* hint,
        int* dtype,
        double cos_angle_tolerance,
        double curvature_tolerance
        ) const
{
  ON_3dPoint Pm, Pp;
  ON_3dVector D1m, D1p, D2m, D2p, Tm, Tp, Km, Kp;
  double s0, s1, s;
  bool rc = false;
  ON_Interval sdom, cdom;
  const int count = Count();
  int segment_hint=0, curve_hint=0;
  if ( dtype )
    *dtype = 0;
  if ( count > 0 && t0 != t1 ) 
  {
    // 20 March 2003 Dale Lear:
    //     look for parametric discontinuities on the interior.
    //     If we don't find any, then well check for locus 
    //     discontinuities at the appropriate end
    ON::continuity input_c = c;
    c = ON::ParametricContinuity(c);

    segment_hint = (hint) ? (*hint & 0x3FFF) : 0;
    int segment_index = ON_NurbsSpanIndex(2,count+1,m_t,t0,(t0>t1)?-1:1,segment_hint);
    curve_hint = ( hint && segment_hint == segment_index ) ? ((*hint)>>14) : 0;


    {
      // 20 March 2003 Dale Lear:
      //     If t0 is very near interior m_t[] value, see if it
      //     should be set to that value.  A bit or two of 
      //     precision sometimes gets lost in proxy
      //     domain to real curve domain conversions on the interior
      //     of a curve domain.
      double segtol = (fabs(m_t[segment_index]) + fabs(m_t[segment_index+1]) + fabs(m_t[segment_index+1]-m_t[segment_index]))*ON_SQRT_EPSILON;
      if ( m_t[segment_index]+segtol < m_t[segment_index+1]-segtol )
      {
        if ( t0 < t1 )
        {
          if ( t0 < m_t[segment_index+1] && t1 > m_t[segment_index+1] && fabs(t0-m_t[segment_index+1]) <= segtol && segment_index+1 < count )
          {
            t0 = m_t[segment_index+1];
            segment_index = ON_NurbsSpanIndex(2,count+1,m_t,t0,1,segment_hint);
          }
        }
        else
        {
          if ( t0 > m_t[segment_index] && t1 < m_t[segment_index] && fabs(t0-m_t[segment_index]) <= segtol && segment_index > 0 )
          {
            t0 = m_t[segment_index];
          }
        }
      }
    }

    double tmin, tmax;
    int segment_index_delta;
    if (t0 > t1)
    {
      segment_index_delta = -1;
      tmin = t1;
      tmax = t0;
    }
    else
    {
      segment_index_delta = 1;
      tmin = t0;
      tmax = t1;
    }

    const ON_Curve* crv;
    for ( /*empty*/; 
              segment_index >= 0 
           && segment_index < count
           && tmin < m_t[segment_index+1] && m_t[segment_index] < tmax; 
          segment_index += segment_index_delta )
    {
      crv = m_segment[segment_index];
      if ( !crv )
        break;
      cdom = crv->Domain();
      sdom.Set( m_t[segment_index], m_t[segment_index+1] );
      if ( sdom == cdom )
      {
        s0 = t0;
        s1 = t1;
      }
      else
      {
        s0 = cdom.ParameterAt( sdom.NormalizedParameterAt(t0) );
        s1 = cdom.ParameterAt( sdom.NormalizedParameterAt(t1) );
      }
      rc = crv->GetNextDiscontinuity( c, s0, s1, &s, &curve_hint, dtype, cos_angle_tolerance, curvature_tolerance );
      if ( rc )
      {
        double kink_t;
        if ( sdom == cdom )
        {
          kink_t = s;
        }
        else
        {
          kink_t = sdom.ParameterAt( cdom.NormalizedParameterAt(s) );
          double t_tol = (fabs(t0)+fabs(t1)+fabs(t0-t1))*ON_ZERO_TOLERANCE;
          if ( kink_t <= tmin+t_tol || kink_t >= tmax-t_tol)
          {
            // 24 January 2002 Dale Lear -
            // It is possible that lost precision in the 
            // domain conversion is giving us trouble.
            // In particular, if this code is not here,
            // "t0" is right at a kink, and s0 gets bumped
            // down a little bit due to rounding/truncation,
            // we end up finding the kink at "t0" that we were
            // supposed to skip.
            double e = fabs(sdom.Length()/cdom.Length());
            if ( e < 1.0 ) e = 1.0; else if (e > 1000.0) e = 1000.0;
            double s_tol = (fabs(s0)+fabs(s1)+fabs(s0-s1))*ON_ZERO_TOLERANCE*e;
            if ( kink_t <= tmin+t_tol )
            {
              if( s0>s1 )
                s1 = s1 + s_tol;
              else
                s0 = s0 + s_tol;
            }
            if ( kink_t >= tmax-t_tol )
            {
              if ( s0>s1 )
                s0 = s0 - s_tol;
              else
                s1 = s1 - s_tol;
            }
            rc = crv->GetNextDiscontinuity( c, s0, s1, &s, &curve_hint, dtype, cos_angle_tolerance, curvature_tolerance );
            if (rc)
            {
              kink_t = sdom.ParameterAt( cdom.NormalizedParameterAt(s) );
              if ( kink_t <= tmin || kink_t >= tmax )
                rc = false;
            }
          }
        }

        if (rc)
        {
          if ( t )
          {
            *t = kink_t;
            if ( hint )
            {
              *hint = segment_index | (curve_hint<<14);
            }
          }
          break;
        }
      }


      // check for discontinity between curve segments
      int next_segment_index = segment_index+segment_index_delta;
      if ( next_segment_index < 0 || next_segment_index >= count )
      {
        // no more curve segments in search interval
        break;
      }
      const ON_Curve* crv1 = m_segment[next_segment_index];
      if ( !crv1 )
        break;

      if ( t0 > t1 )
      {
        if ( sdom[0] <= t1 ) // this line is correct - search is decreasing towards t1
        {
          // INTERIOR of search interval does not include 
          // start this crv = end of next curve
          break;
        }
      }
      else
      {
        if ( t1 <= sdom[1] )
        {
          // INTERIOR of search interval does not include 
          // end of this crv = start of next curve
          break;
        }
      }

      double crv0_t, crv1_t;
      int crv0_side;
      if ( t0 > t1 )
      {
        // compare start if this curve against end of next curve
        crv0_t = cdom[0];
        crv1_t = crv1->Domain()[1];
        crv0_side = 1;
      }
      else
      {
        // compare end if this curve against start of next curve
        crv0_t = cdom[1];
        crv1_t = crv1->Domain()[0];
        crv0_side = -1;
      }

      switch( c )
      {
      case ON::C1_continuous:
      case ON::G1_continuous:
        crv->Ev1Der( crv0_t, Pm, D1m, crv0_side );   // point on this curve
        crv1->Ev1Der( crv1_t, Pp, D1p, -crv0_side ); // corresponding point on next curve
        if ( c == ON::C1_continuous )
        {
          if ( !(D1m-D1p).IsTiny(D1m.MaximumCoordinate()*ON_SQRT_EPSILON) )
            rc = true;
        }
        else
        {
          Tm = D1m;
          Tp = D1p;
          Tm.Unitize();
          Tp.Unitize();
          if ( Tm*Tp < cos_angle_tolerance )
            rc = true;
        }
        if ( rc && dtype )
          *dtype = 1;
        break;

      case ON::C2_continuous:
      case ON::G2_continuous:
      case ON::Gsmooth_continuous:
        crv->Ev2Der( crv0_t, Pm, D1m, D2m, crv0_side );   // point on this curve
        crv1->Ev2Der( crv1_t, Pp, D1p, D2p, -crv0_side ); // corresponding point on next curve
        if ( c == ON::C2_continuous )
        {
          if ( !(D1m-D1p).IsTiny(D1m.MaximumCoordinate()*ON_SQRT_EPSILON) ) 
          {
            rc = true;
            if ( dtype )
              *dtype = 1;
          }
          else if ( !(D2m-D2p).IsTiny(D2m.MaximumCoordinate()*ON_SQRT_EPSILON) )
          {
            rc = true;
            if ( dtype )
              *dtype = 2;
          }
        }
        else
        {
          ON_EvCurvature( D1m, D2m, Tm, Km );
          ON_EvCurvature( D1p, D2p, Tp, Kp );
          if ( Tm*Tp < cos_angle_tolerance )
          {
            rc = true;
            if ( dtype )
              *dtype = 1;
          }
          else 
          {
            bool bIsCurvatureContinuous = ( ON::Gsmooth_continuous == c )
              ? ON_IsGsmoothCurvatureContinuous(Km, Kp, cos_angle_tolerance, curvature_tolerance)
              : ON_IsG2CurvatureContinuous(Km, Kp, cos_angle_tolerance, curvature_tolerance);
            if ( !bIsCurvatureContinuous )
            {
              // NOTE:
              //   The test to enter this scope must exactly match
              //   the one used in ON_NurbsCurve::GetNextDiscontinuity().
              rc = true;
              if ( dtype )
                *dtype = 2;
            }
            else if ( ON::Gsmooth_continuous == c )
            {
              const double is_linear_tolerance = 1.0e-8;  
              const double is_linear_min_length = 1.0e-8;
              const ON_Curve* seg0;
              const ON_Curve* seg1;
              if (crv0_side<0)
              {
                seg0 = crv;
                seg1 = crv1;
              }
              else
              {
                seg0 = crv1;
                seg1 = crv;
              }
              bool b0 = seg0->LastSpanIsLinear(is_linear_min_length,is_linear_tolerance);
              bool b1 = seg1->FirstSpanIsLinear(is_linear_min_length,is_linear_tolerance);
              if ( b0 != b1 )
              {
                rc = true;
                if ( dtype )
                  *dtype = 3;
              }
            }
          }
        }
        break;
      default:
        // intentionally ignoring other ON::continuity enum values
        break;
      }
      if (rc)
      {
        int tindex = (t0>t1)?segment_index:(segment_index+1); 
        if ( t )
          *t = m_t[tindex];
        if ( hint )
        {
          *hint = tindex;
        }
        break;
      }
    }

    if ( !rc && input_c != c )
    {
      // 20 March 2003 Dale Lear
      //   See if we need to do a locus check at an end
      rc = ON_Curve::GetNextDiscontinuity( input_c, 
                        t0, t1, t, NULL, 
                        dtype, 
                        cos_angle_tolerance, curvature_tolerance );
    }
  }
  return rc;
}

bool ON_PolyCurve::IsContinuous(
    ON::continuity desired_continuity,
    double t, 
    int* hint, // default = NULL,
    double point_tolerance, // default=ON_ZERO_TOLERANCE
    double d1_tolerance, // default==ON_ZERO_TOLERANCE
    double d2_tolerance, // default==ON_ZERO_TOLERANCE
    double cos_angle_tolerance, // default==ON_DEFAULT_ANGLE_TOLERANCE_COSINE
    double curvature_tolerance  // default==ON_SQRT_EPSILON
    ) const
{
  bool rc = true;
  const int count = Count();
  if ( count > 0 ) 
  {
    if ( t <= m_t[0] || t >= m_t[count] )
    {
      // 20 March 2003 Dale Lear
      //     Consistently handles locus case and out of domain case.
      rc = ON_Curve::IsContinuous( 
                 desired_continuity, t, hint, 
                 point_tolerance, 
                 d1_tolerance, d2_tolerance, 
                 cos_angle_tolerance, 
                 curvature_tolerance );
      return rc;
    }

    // "locus" and "parametric" are the same at this point.
    desired_continuity = ON::ParametricContinuity(desired_continuity);


    int segment_hint = 0, curve_hint = 0;
    if ( hint )
      segment_hint = (*hint & 0x3FFF);
    int segment_index = ON_NurbsSpanIndex(2,count+1,m_t,t,1,segment_hint);

    {
      // 20 March 2003 Dale Lear:
      //     If t is very near interior m_t[] value, see if it
      //     should be set to that value.  A bit or two of 
      //     precision sometimes gets lost in proxy
      //     domain to real curve domain conversions on the interior
      //     of a curve domain.
      double segtol = (fabs(m_t[segment_index]) + fabs(m_t[segment_index+1]) + fabs(m_t[segment_index+1]-m_t[segment_index]))*ON_SQRT_EPSILON;
      if ( m_t[segment_index]+segtol < m_t[segment_index+1]-segtol )
      {
        if ( fabs(t-m_t[segment_index]) <= segtol && segment_index > 0 )
        {
          t = m_t[segment_index];
        }
        else if ( fabs(t-m_t[segment_index+1]) <= segtol && segment_index+1 < count )
        {
          t = m_t[segment_index+1];
          segment_index = ON_NurbsSpanIndex(2,count+1,m_t,t,1,segment_hint);
        }
      }
    }

    if ( hint )
    {
      if ( segment_hint == segment_index )
        curve_hint = ((*hint)>>14);
      else
      {
        segment_hint = segment_index;
        *hint = segment_hint;
      }
    }

    if ( m_t[segment_index] < t && t < m_t[segment_index+1] )
    {
      // test interior of this segment
      const ON_Curve* segment_curve = SegmentCurve(segment_index);
      if ( segment_curve )
      {
        ON_Interval sdom, cdom;
        cdom = segment_curve->Domain();
        sdom.Set( m_t[segment_index], m_t[segment_index+1] );
        if ( sdom != cdom )
          t = cdom.ParameterAt( sdom.NormalizedParameterAt(t) );
        rc = segment_curve->IsContinuous( desired_continuity, t, &curve_hint, 
                                          point_tolerance, d1_tolerance, d2_tolerance, 
                                          cos_angle_tolerance, curvature_tolerance );
        if ( hint )
          *hint = (segment_index | (curve_hint<<14));
      }
    }
    else if ( count > 0 )
    {
      if ( segment_index == 0 && t == m_t[0] )
        rc = true; // t at start of domain
      else if ( segment_index == count-1 && t == m_t[count] )
        rc = true; // t and end of domain
      else
      {
        // evaluate ends of segments
        rc = ON_Curve::IsContinuous( desired_continuity, t, hint, 
                           point_tolerance, d1_tolerance, d2_tolerance, 
                           cos_angle_tolerance, curvature_tolerance );
        if ( 0 != rc 
             && ON::Gsmooth_continuous == desired_continuity 
             && segment_index >= 0
             && segment_index < count
           )
        {
          // check for linear to non-linear transition
          const int i0 = ( t >= m_t[segment_index] ) ? segment_index-1 : segment_index;
          if ( i0 >= 0 && t == m_t[i0+1] )
          {
            const ON_Curve* seg0 = SegmentCurve(i0);
            const ON_Curve* seg1 = SegmentCurve(i0+1);
            if ( 0 != seg0 && 0 != seg1 )
            {
              const double is_linear_tolerance = 1.0e-8;  
              const double is_linear_min_length = 1.0e-8;
              bool b0 = seg0->LastSpanIsLinear(is_linear_min_length,is_linear_tolerance);
              bool b1 = seg1->FirstSpanIsLinear(is_linear_min_length,is_linear_tolerance);
              if ( b0 != b1 )
                rc = false;
            }
          }
        }
      }
    }
  }
  return rc;
}

ON_BOOL32
ON_PolyCurve::Reverse()
{
  const int count = Count();
  int i;
  ON_BOOL32 rc = (count>0) ? true : false;
  if ( rc ) {
    m_segment.Reverse();
    m_t.Reverse();
    for ( i = 0; i < count; i++ ) {
      m_segment[i]->Reverse();
      m_t[i] = -m_t[i];
    }
    m_t[count] = -m_t[count];
  }
	DestroyCurveTree();
  return rc;
}

bool ON_TuneupEvaluationParameter( 
   int side,
   double s0, double s1, // segment domain
   double *s             // segment parameter
   )
{
  double t = *s;
  if ( 0 != side && s0 < t && t < s1 )
  {
    // 9 November 2010 Dale Lear
    //   I wrote this function today and chose
    //   1.0e-10 as the "noise" factor.  1.0e-10
    //   may need to be adjusted but it should
    //   not be larger unless there is a very
    //   good reason.  You must document any changes
    //   and include a bug track number so subsequent
    //   changes can be tested.  Any value used to
    //   replace 1.0e-10 must be strictly smaller
    //   than ON_SQRT_EPSILON because some solvers
    //   use (s1-s0)*ON_SQRT_EPSILON as a minimum step
    //   size.
    double ds = (s1-s0)*1.0e-10;
    if ( side < 0 )
    {
      if ( t <= s0+ds )
      {
        *s = s0;
        return true;
      }
    }
    else // side > 0
    {
      if ( t >= s1-ds )
      {
        *s = s1;
        return true;
      }
    }
  }
  return false;
}


ON_BOOL32 ON_PolyCurve::Evaluate( // returns false if unable to evaluate
       double t,       // evaluation parameter
       int der_count,  // number of derivatives (>=0)
       int v_stride,   // v[] array stride (>=Dimension())
       double* v,      // v[] array of length stride*(ndir+1)
       int side,       // optional - determines which side to evaluate from
                       //         0 = default
                       //      <  0 to evaluate from below, 
                       //      >  0 to evaluate from above
       int* hint       // optional - evaluation hint (int) used to speed
                       //            repeated evaluations
       ) const
{
  ON_BOOL32 rc = false;
  const int count = Count();
  const int dim = Dimension();
  int segment_hint, curve_hint;
  if ( count > 0 && dim > 0 && dim <= v_stride ) 
  {
    segment_hint = (hint) ? (*hint & 0x3FFF) : 0;
    int segment_index = ON_NurbsSpanIndex(2,count+1,m_t,t,side,segment_hint);
    if ( -2 == side || 2 == side )
    {
      // 9 November 2010 Dale Lear - ON_TuneupEvaluationParameter fix
      //   When evluation passes through ON_CurveProxy or ON_PolyCurve reparamterization
      //   and the original side parameter was -1 or +1, it is changed to -2 or +2
      //   to indicate that if t is numerically closed to an end paramter, then
      //   it should be tuned up to be at the end paramter.
      double a = t;
      if ( ON_TuneupEvaluationParameter( side, m_t[segment_index], m_t[segment_index+1], &a) )
      {
        // recalculate segment index
        t = a;
        segment_index = ON_NurbsSpanIndex(2,count+1,m_t,t,side,segment_index);
      }
    }
    const ON_Curve* c = m_segment[segment_index];
    if ( c ) {
      double s0, s1;
      {
        ON_Interval dom = c->Domain();
        s0 = dom.Min();
        s1 = dom.Max();
      }
      if ( s0 != s1 ) 
      {
        const double t0 = m_t[segment_index];
        const double t1 = m_t[segment_index+1];
        double s;
        if ( s0 == t0 && s1 == t1 )
        {
          // segment domain = c->Domain()
          s = t;
        }
        else 
        {
          // adjust segment domain parameter
          if ( fabs(t1 - t0) < (ON_ZERO_TOLERANCE + ON_EPSILON*fabs(t0)) )
          {
            // segment domain is insanely short
            s = (fabs(t-t0) < fabs(t-t1)) ? s0 : s1;
          }
          else 
          {
            // 30 May 2012 Dale Lear bug # 105974
            //   The arithmetic below was setting b = 0 and a = 0.9999999999999...
            //   so I added the checking for 0 and 1 stuff.
            const double d = t1-t0;
            double a = (t - t0)/d;
            double b = (t1 - t)/d;
            if ( 0.0 == b )
              a = 1.0;
            else if ( 1.0 == b )
              a = 0.0;
            else if ( 0.0 == a )
              b = 1.0;
            else if ( 1.0 == a )
              b = 0.0;
            s = b*s0 + a*s1;
          }
          if ( -1 == side )
            side = -2;
          else if ( 1 == side )
            side = 2;
        }
        curve_hint = ( hint && segment_hint == segment_index ) ? ((*hint)>>14) : 0;
        rc = c->Evaluate(
           s, 
           der_count, 
           v_stride, v, 
           side, 
           &curve_hint );

        if ( rc ) 
        {
          if ( der_count > 0 && s1 - s0 != t1 - t0 && t0 != t1 )
          {
            // Adjust segment derivative evaluation bug by applying chain rule
            // to get polycurve derivative value.
            const double d = (s1-s0)/(t1-t0);
            s = d;
            int di, vi;
            v += v_stride;
            for ( di = 1; di <= der_count; di++ )
            {
              for ( vi = 0; vi < dim; vi++ ) 
              {
                v[vi] = s*v[vi];
              }
              s *= d;
              v += v_stride;
            }
          }

          if ( hint )
            *hint = segment_index | (curve_hint<<14);
        }

      }
    }
  }
  return rc;
}

int
ON_PolyCurve::Count() const
{
  return m_segment.Count();
}


ON_Curve*
ON_PolyCurve::operator[](int segment_index) const
{
  return SegmentCurve(segment_index);
}

ON_Curve*
ON_PolyCurve::SegmentCurve(int segment_index) const
{
  return ( segment_index >= 0 && segment_index < Count() ) 
         ? m_segment[segment_index] 
         : NULL;
}


double ON_PolyCurve::SegmentCurveParameter(
  double polycurve_parameter
  ) const
{
  int segment_index = SegmentIndex( polycurve_parameter );
  const ON_Curve* segment_curve = SegmentCurve(segment_index);
  if ( !segment_curve )
    return ON_UNSET_VALUE;
  ON_Interval cdom = segment_curve->Domain();
  ON_Interval sdom = SegmentDomain(segment_index);
  if ( cdom == sdom )
    return polycurve_parameter;
  double s = sdom.NormalizedParameterAt(polycurve_parameter);
  return cdom.ParameterAt(s);
}


double ON_PolyCurve::PolyCurveParameter(
  int segment_index,
  double segmentcurve_parameter
  ) const
{
  const ON_Curve* segment_curve = SegmentCurve(segment_index);
  if ( !segment_curve )
    return ON_UNSET_VALUE;
  ON_Interval cdom = segment_curve->Domain();
  ON_Interval sdom = SegmentDomain(segment_index);
  if ( cdom == sdom )
    return segmentcurve_parameter;
  double s = cdom.NormalizedParameterAt(segmentcurve_parameter);
  return sdom.ParameterAt(s);
}


ON_Interval 
ON_PolyCurve::SegmentDomain( int segment_index ) const
{
  ON_Interval domain;
  if ( segment_index >= 0 && segment_index < Count() ) {
    domain.m_t[0] = m_t[segment_index];
    domain.m_t[1] = m_t[segment_index+1];
  }
  return domain;
}


ON_Curve*
ON_PolyCurve::FirstSegmentCurve() const
{
  return SegmentCurve(0);
}

ON_Curve*
ON_PolyCurve::LastSegmentCurve() const
{
  return SegmentCurve(Count()-1);
}

void
ON_PolyCurve::Reserve( int capacity )
{
  m_segment.Reserve(capacity);
  m_t.Reserve(capacity+1);
}

ON_BOOL32 ON_PolyCurve::Prepend( ON_Curve* c )
{
	DestroyCurveTree();
  return Insert( 0, c );
}

ON_BOOL32 ON_PolyCurve::Append( ON_Curve* c )
{
	DestroyCurveTree();
  return Insert( Count(), c );
}

ON_BOOL32 ON_PolyCurve::PrependAndMatch(ON_Curve* c)

{
  if (Count() == 0) return Prepend(c);
  //if (IsClosed() || c->IsClosed()) return false;
  if (!c->SetEndPoint(PointAtStart())){
    if (!SetStartPoint(c->PointAtEnd()))
      return false;
  }
  return Prepend(c);
}

ON_BOOL32 ON_PolyCurve::AppendAndMatch(ON_Curve* c)

{
  if (Count() == 0) return Append(c);
  //if (IsClosed() || c->IsClosed()) return false;
  if (!c->SetStartPoint(PointAtEnd())){
    if (!SetEndPoint(c->PointAtStart()))
      return false;
  }
  return Append(c);
}


ON_BOOL32 ON_PolyCurve::Remove( )
{
  return Remove(Count()-1);
}

ON_BOOL32 ON_PolyCurve::Remove( int segment_index )
{
  ON_BOOL32 rc = false;
  const int segment_count = Count();
  if ( segment_index >= 0 && segment_index < segment_count ) {
    delete m_segment[segment_index];
    m_segment[segment_index] = 0;
    m_segment.Remove(segment_index);
		// GBA 18 September 2003. m_t array not properly updated when last segment removed.
    if ( segment_index >= 1 ) {
      double* d = m_t.Array();
      const double delta = d[segment_index] - d[segment_index+1];
      int i;
      for (i=segment_index+1; i <= segment_count; i++ ) {
        d[i] += delta;
      }
    }
		// GBA 12/02/02.  When removing the last segment remove both m_t values so
		// the polycurve will have the same state as ON_PolyCurve().
		if( segment_count==1)
			m_t.Empty();
		else
	    m_t.Remove(segment_index);
    rc = true;
  }
  return rc;
}

ON_BOOL32 ON_PolyCurve::Insert( int segment_index, ON_Curve* c )
{
  double s0, s1;
  ON_BOOL32 rc = false;
  const int count = Count();
  if ( segment_index >= 0 && segment_index <= count && c && c != this && c->GetDomain(&s0,&s1) ) 
  {
    rc = true;
    m_segment.Insert( segment_index, c );

    // determine polycurve parameters for this segment
    double t0, t1;
    if ( segment_index == count ) {
      // append segment
      if ( count == 0 ) {
        m_t.Append(s0);
        m_t.Append(s1);
      }
      else {
        t0 = m_t[count];
        t1 = ( s0 == t0 ) ? s1 : (s1-s0+t0);
        m_t.Append(t1);
      }
    }
    else if ( segment_index == 0 ) {
      // prepend segment
      t1 = m_t[0];
      t0 = ( s1 == t1 ) ? s0 : (s0-s1+t1);
      m_t.Insert(0,t0);
    }
    else {
      // insert segment
      t0 = m_t[segment_index];
      t1 = ( s0 == t0 ) ? s1 : (s1-s0+t0);
      const double dt = t1-t0;
      m_t.Insert(segment_index+1,t1);
      double* t = m_t.Array();
      for ( int i = segment_index+2; i <= count+1; i++ ) {
        t[i] += dt;
      }      
    }
  }
  return rc;
}

ON_BOOL32 ON_PolyCurve::SetStartPoint(ON_3dPoint start_point)
{
  ON_BOOL32 rc = false;
  // just do it // if ( !IsClosed() )
  {
    ON_Curve* c = FirstSegmentCurve();
    if ( c )
      rc = c->SetStartPoint(start_point);
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 ON_PolyCurve::SetEndPoint(ON_3dPoint end_point)
{
  ON_BOOL32 rc = false;
  // just do it // if ( !IsClosed() )
  {
    ON_Curve* c = LastSegmentCurve();
    if ( c )
      rc = c->SetEndPoint(end_point);
  }
	DestroyCurveTree();
  return rc;
}

int ON_PolyCurve::GetNurbForm( 
                              ON_NurbsCurve& nurb, 
                              double tol,
                              const ON_Interval* subdomain  // OPTIONAL subdomain of ON::ProxyCurve::Domain()
                              ) const
{
  ON_Interval domain = Domain();
  if ( !domain.IsIncreasing() )
    return false;
  int rc = 0;
  int si0 = 0;
  int si1 = Count();
  if ( subdomain ) {
    if ( !subdomain->IsIncreasing() )
      return 0;
    if ( !domain.Includes(subdomain->Min()) )
      return 0;
    if ( !domain.Includes(subdomain->Max()) )
      return 0;
    domain = *subdomain;
  }

  while ( si0 < si1 && m_t[si0+1] <= domain.m_t[0] )
    si0++;
  while ( si0 < si1 && m_t[si1-1] >= domain.m_t[1] )
    si1--;
  if ( si0 >= si1 )
    return 0;
  {
    ON_NurbsCurve c;
    int i, rci;
    for ( i = si0; i < si1; i++ ) {
      if ( !m_segment[i] )
        return 0;
      if ( i == si0 ) {
        rc = m_segment[i]->GetNurbForm( nurb, tol, NULL );
        if ( rc < 1 )
          return rc;
        nurb.SetDomain( m_t[i], m_t[i+1] );
      }
      else {
        rci = m_segment[i]->GetNurbForm( c, tol, NULL );
        if ( rci < 1 )
          return rci;
        else if ( rc < rci )
          rc = rci;
        c.SetDomain( m_t[i], m_t[i+1] );
        if ( !nurb.Append( c ) )
          return 0;
        c.Destroy();
      }
    }
  }

  if ( subdomain )
    nurb.Trim( *subdomain );

  return rc;
}

int ON_PolyCurve::HasNurbForm() const

{
  int count = m_segment.Count();
  if (!count)
    return 0;
  int i;
  int rc = 1;
  for (i=0; i<count; i++){
    const ON_Curve* scrv = SegmentCurve(i);
    if (!scrv)
      return 0;
    int nf = scrv->HasNurbForm();
    if (nf == 0)
      return 0;
    if (nf == 2)
      rc = 2;
  }
  return rc;
}


int ON_PolyCurve::SegmentIndex( double curve_t ) const
{
  int count = m_segment.Count();
  int seg_index = ON_SearchMonotoneArray( m_t.Array(), m_t.Count(), curve_t );
  if ( seg_index < 0 )
    seg_index = 0;
  else if ( seg_index >= count )
    seg_index = count-1;
  return seg_index;
}

int ON_PolyCurve::SegmentIndex(
  ON_Interval sub_domain,
  int* segment_index0,
  int* segment_index1
  ) const
{
  const int segment_count = m_segment.Count();
  int s0 = 0, s1 = 0;
  ON_Interval seg_dom;
  sub_domain.Intersection( Domain() );
  if ( sub_domain.IsIncreasing() )
  {
    s0 = SegmentIndex(sub_domain.Min());
    for ( s1 = s0+1; s1 < segment_count; s1++ )
    {
      seg_dom = SegmentDomain(s1);
      if ( seg_dom[0] >= sub_domain.Max() )
        break;
    }
  }
  if ( segment_index0 )
    *segment_index0 = s0;
  if ( segment_index1 )
    *segment_index1 = s1;
  return s1-s0;
}


ON_BOOL32 ON_PolyCurve::GetCurveParameterFromNurbFormParameter(
      double nurbs_t,
      double* curve_t
      ) const
{
  ON_BOOL32 rc = false;
  int i = SegmentIndex( nurbs_t );
  const ON_Curve* curve = SegmentCurve(i);
  if ( curve ) {
    ON_Interval in( m_t[i], m_t[i+1] );
    ON_Interval cdom = curve->Domain();
    if ( in != cdom ) {
      nurbs_t = cdom.ParameterAt(in.NormalizedParameterAt(nurbs_t));
      rc = curve->GetCurveParameterFromNurbFormParameter(nurbs_t,curve_t);
      if ( rc )
        *curve_t = in.ParameterAt(cdom.NormalizedParameterAt(*curve_t));
    }
    else {
      rc = curve->GetCurveParameterFromNurbFormParameter(nurbs_t,curve_t);
    }
  }
  return rc;
}

ON_BOOL32 ON_PolyCurve::GetNurbFormParameterFromCurveParameter(
      double curve_t,
      double* nurbs_t
      ) const
{
  ON_BOOL32 rc = false;
  int i = SegmentIndex( curve_t );
  const ON_Curve* curve = SegmentCurve(i);
  if ( curve ) {
    ON_Interval in( m_t[i], m_t[i+1] );
    ON_Interval cdom = curve->Domain();
    if ( in != cdom ) {
      curve_t = cdom.ParameterAt(in.NormalizedParameterAt(curve_t));
      rc = curve->GetNurbFormParameterFromCurveParameter(curve_t,nurbs_t);
      if ( rc )
        *nurbs_t = in.ParameterAt(cdom.NormalizedParameterAt(*nurbs_t));
    }
    else {
      rc = curve->GetNurbFormParameterFromCurveParameter(curve_t,nurbs_t);
    }
  }
  return rc;
}


ON_Curve* ON_PolyCurve::HarvestSegment( int i )
{
  ON_Curve* segment_curve = 0;
  if ( i >= 0 && i < m_segment.Count() ) {
    segment_curve = m_segment[i];
    m_segment[i] = 0;
  }
  return segment_curve;
}

ON_BOOL32 ON_PolyCurve::Trim(
  const ON_Interval& domain
  )
{
  // Please talk to Dale Lear before you change code in this function.

  // m_t[] = Increasing array of segment_count+1 parameter values
  //         that specify segment domains.  
  //         Domain of polycurve = (m_t[0],m_t[segment_count]).
  // m_segment[] = array of segment curves
  int segment_count = m_segment.Count();
  if ( m_t.Count() < 2 || segment_count+1 != m_t.Count() || !domain.IsIncreasing() )
  {
    // bogus input
    return false;
  }

  const ON_Interval original_polycurve_domain = Domain();
  if ( !original_polycurve_domain.IsIncreasing() )
    return false;

  ON_Interval output_domain = domain;
  if ( !output_domain.Intersection(original_polycurve_domain) )
    return false;

	if(!output_domain.IsIncreasing())
		return false;

  if (output_domain == original_polycurve_domain )
    return true;

  ON_Interval actual_trim_domain = output_domain;

  int s0 = -2; // s0 gets set to index of first segment we keep
  int s1 = -3; // s1 gets set to index of last segment we keep
  
  // 22 October 2003 Dale Lear - redid Greg's parameter search 
  //   snapping stuff.  New stuff is in sourcesafe version 72.
  //   In particular, attempting using "Trim" to extend polycurves
  //   will not be supported.  You have to use "Extend" if you
  //   want a curve to get longer.

  // In mid 3003, Greg added ParameterSearch to do "microtol" snapping 
  // to segment end parameters.  The goal was to handle fuzz that gets
  // introduces by reparameterizations that happen when the top level
  // curve is a proxy/poly curve and the proxy/polycurve trim parameters get
  // readjusted as we move toward trimming the "real" curve that is 
  // stored in the m_segment[] array.
	if ( ParameterSearch(output_domain[0], s0, true ) )
  {
    // ParameterSearch says domain[0] is within "microtol" of
    // m_t[s0].  So we will actually trim at m_t[s0].
    if (s0 >= 0 && s0 <= segment_count)
    {
      actual_trim_domain[0]=m_t[s0];
    }
  }

	if ( ParameterSearch(output_domain[1], s1, true ) )
  {
    if (s1 >= 0 && s1 <= segment_count )
    {
      // ParameterSearch says domain[1] is within "microtol" of
      // m_t[s1].  So we will actually trim at m_t[s1].
      actual_trim_domain[1]=m_t[s1];
      s1--;
    }
  }

  if ( !actual_trim_domain.IsIncreasing() )
  {
    // After microtol snapping, there is not enough curve left to trim.
    return false;
  }

  if ( s0 < 0 || s0 > s1 || s1 >= segment_count )
  {
    // Because output_domain is a subinterval of original_polycurve_domain,
    // the only way that (s0 < 0 || s0 > s1 || s1 >= segment_count) can be true
    // is if something is seriously wrong with the m_t[] values.
    return false;
  }

  // we will begin modifying the polycurve
  DestroyCurveTree();

  if ( actual_trim_domain == original_polycurve_domain )
  {
    // ParameterSearch says that the ends of output_domain
    // were microtol away from being the entire curve.  
    // Set the domain and return.
    m_t[0] = output_domain[0];
    m_t[segment_count] = output_domain[1];
    return true;
  }
	
  int i;
  for ( i = 0; i < s0; i++ )
  {
    // delete curves in segments [0,...,s0-1]
    delete m_segment[i];
    m_segment[i] = 0;
  }
  for ( i = s1+1; i < segment_count; i++ )
  {
    // delete curves in segments [s1+1,...,segment_count-1]
    delete m_segment[i];
    m_segment[i] = 0;
  }

  // remove segments [s1+1,...,segment_count-1] from polycurve
  m_segment.SetCount( s1+1 );
  m_t.SetCount(s1+2);
  segment_count = s1+1;

  if ( s0 > 0 )
  {
    // remove segments [0,...,s0-1] from polycurve
    ON_SimpleArray<ON_Curve*> tmp_seg(s1+1-s0);
    ON_SimpleArray<double> tmp_t(s1+2-s0);
    tmp_seg.Append( s1+1-s0, m_segment.Array()+s0 );
    tmp_t.Append( s1+2-s0, m_t.Array()+s0 );
    m_segment.Zero();
    m_segment.SetCount( 0 );
    m_segment.Append( tmp_seg.Count(), tmp_seg.Array() );
    m_t = tmp_t;
    segment_count = s1-s0+1;
    s1 = segment_count-1;
    s0 = 0;
  }


  const double fuzz = 0.001; // Greg says: anything small and > 1.0e-6 will work about the same
  bool bTrimFirstSegment = ( m_t[0] < actual_trim_domain[0] || (0 == s1 && actual_trim_domain[1] < m_t[s1+1]) );
  bool bTrimLastSegment = (s1>s0 && actual_trim_domain[1] < m_t[s1+1]);

  // if needed, trim left end of first segment
  ON_Interval trim_s_dom, trim_c_dom, c_dom, s_dom;

  if ( bTrimFirstSegment )
  {
    ON_Curve* curve = SegmentCurve(0);
    if ( 0 == curve )
      return false; // bogus polycurve (m_segment[0] is a NULL pointer)

    c_dom = curve->Domain();
    if ( !c_dom.IsIncreasing() )
    {
      // first segment curve is bogus
      return false;
    }

    s_dom = SegmentDomain(0);
    if ( !s_dom.IsIncreasing() )
    {
      // m_t[0] or m_t[1] is bogus
      return false;
    }

    trim_s_dom = s_dom;
	  if ( !trim_s_dom.Intersection(actual_trim_domain) )
    {
      // Should never happen; if it does, we have to give up.
      // (and there is probably a bug in the code above)
      return false;
    }

    if ( s1 > 0 && trim_s_dom[1] != s_dom[1] )
    {
      // Should never happen; if it does, we have to give up.
      // (and there is probably a bug in the code above)
      return false;
    }

    if ( !trim_s_dom.IsIncreasing() )
    {
      // Should never happen; if it does, we have to give up.
      // (and there is probably a bug in the code above)
      return false;
    }

    if ( c_dom != s_dom )
    {
      // need to convert polycurve parameters to "real" segment curve parameters
      trim_c_dom[0] = c_dom.ParameterAt( s_dom.NormalizedParameterAt(trim_s_dom[0]) );
      trim_c_dom[1] = c_dom.ParameterAt( s_dom.NormalizedParameterAt(trim_s_dom[1]) );
      if ( !trim_c_dom.IsIncreasing() )
      {
        if ( s_dom.NormalizedParameterAt(trim_s_dom[0]) >= 1.0-fuzz && s1 > 0 )
        {
          // We were trying to throw away all but a microscopic bit on the right
          // end of the first segment of a multi segment polycurve
          // and the parameter conversion killed the "real" trim interval.
          // In this case, we can just throw away the first segment.
          bTrimFirstSegment = false;
          curve = 0;
          delete m_segment[0];
          m_segment[0] = 0;
			    m_t.Remove(0);
			    m_segment.Remove(0);
			    s1--; // removing a segment, means s1=(index of last valid segment) has to get decremented.
        }
        else
          return false;
      }
    }
    else
    {
      trim_c_dom = trim_s_dom;
    }

    // trim_s_dom = polycurve segment parameters after trimming
    // trim_c_dom = "real" segment curve parameters after trimming

    if ( bTrimFirstSegment && trim_c_dom != c_dom )
    {
      // trim first segment
      if ( !curve->Trim(trim_c_dom) )
      {
        // trimming first segment failed - see if we should give up or
        // or just discard the first segment.

			  if ( c_dom.NormalizedParameterAt(trim_c_dom[0]) >= 1.0 - fuzz && s1 > 0 )
        {
			    // remove entire first segment
          bTrimFirstSegment = false;
          curve = 0;
          delete m_segment[0];
          m_segment[0] = 0;
			    m_t.Remove(0);
			    m_segment.Remove(0);
			    s1--; // removing a segment, means s1=(index of last valid segment) has to get decremented.
        }
        else
          return false;
		  }
      else
      {
        m_t[0] = actual_trim_domain[0]; // will be tweaked below when we've finished.
        if ( 0 == s1 && 2 == m_t.Count() && !bTrimLastSegment )
          m_t[1] = actual_trim_domain[1];
      }
    }
  }



  if ( bTrimLastSegment ) 
  {
    // If we get in here, it means we need to trim a portion off of
    // the right end of the last segment.
    
    if ( s1+1 != m_segment.Count() )
    {
      // Should never happen; if it does, we have to give up.
      // (and there is probably a bug in the code above)
      return false;
    }

    ON_Curve* curve = SegmentCurve(s1);
    if ( 0 == curve )
      return false; // bogus polycurve (m_segment[s1] array has a null pointer)

    c_dom = curve->Domain();
    if ( !c_dom.IsIncreasing() )
    {
      // first segment curve is bogus
      return false;
    }

    s_dom = SegmentDomain(s1);
    if ( !s_dom.IsIncreasing() )
    {
      // m_t[s1] or m_t[s1+1] is bogus
      return false;
    }

		// trim the curve on the right
    trim_s_dom= ON_Interval(m_t[s1], actual_trim_domain[1]);

    if ( !trim_s_dom.IsIncreasing() )
    {
      // Should never happen; if it does, we have to give up.
      // (and there is probably a bug in the code above)
      return false;
    }

    trim_c_dom[0] = c_dom[0];
    if ( c_dom != s_dom )
    {
      trim_c_dom[1] = c_dom.ParameterAt( s_dom.NormalizedParameterAt(trim_s_dom[1]) );
      if ( !trim_c_dom.IsIncreasing() )
      {
        if ( s_dom.NormalizedParameterAt(trim_s_dom[1]) <= fuzz && s1 > 0 )
        {
          // We were trying to throw away all but a microscopic bit on the left
          // end of the last segment of a multi segment polycurve
          // and the parameter conversion killed the "real" trim interval.
          // In this case, we can just throw away the last segment.
          bTrimLastSegment = false;
          curve = 0;
          delete m_segment[s1];
          m_segment[s1] = 0;
			    m_t.Remove();       // remove last array entry in the m_t[] array
			    m_segment.Remove(); // remove last array entry in the m_segment[] array
			    s1--; // removing a segment, means s1=(index of last valid segment) has to get decremented.
        }
        else
          return false;
      }
    }
    else
    {
      trim_c_dom[1] = trim_s_dom[1];
    }

    if ( bTrimLastSegment && c_dom != trim_c_dom )
    {
      // trim last segment
      if ( !curve->Trim(trim_c_dom) )
      {

				if ( c_dom.NormalizedParameterAt(trim_c_dom[1]) <= fuzz && s1 > 0)
        {
          // We were trying to throw away all but a microscopic bit on the left
          // end of the last segment of a multi segment polycurve
          // and the segment curve's trimmer failed.  I'm assuming the
          // failure was caused because the part that would be left was
          // too short.
          // In this case, we can just throw away the last segment.
          bTrimLastSegment = false;
          curve = 0;
          delete m_segment[s1];
          m_segment[s1] = 0;
			    m_t.Remove();       // remove last array entry in the m_t[] array
			    m_segment.Remove(); // remove last array entry in the m_segment[] array
			    s1--; // removing a segment, means s1=(index of last valid segment) has to get decremented.
        }
        else
          return false;
			}
      else
        m_t[m_t.Count()-1] = actual_trim_domain[1]; // will be tweaked below when we've finished.
		}
  }

  // If we get this far, trims were is successful.
  // The following makes potential tiny adjustments
  // that need to happen when trims get snapped to
  // m_t[] values that are within fuzz of the
  // output_domain[] values.
	m_t[0] = output_domain[0];
  m_t[m_t.Count()-1] = output_domain[1];

	DestroyCurveTree();

  return true;
}




bool ON_PolyCurve::Extend(
  const ON_Interval& domain
  )
 
{
  if (IsClosed() || Count() < 1) return false;
 
  bool changed = false;
  if (Domain()[0] > domain[0]){
    ON_Curve* seg = SegmentCurve(0);
    if (!seg) return false;
    ON_Interval sdom = SegmentDomain(0);
    ON_Interval cdom = seg->Domain();
    double a = (sdom == cdom) ? domain[0] : cdom.ParameterAt(sdom.NormalizedParameterAt(domain[0]));
    ON_Interval DesiredDom(a, cdom[1]);
    changed = seg->Extend(DesiredDom);
    if (changed) {
      if (seg->Domain() == DesiredDom)
        m_t[0] = domain[0];
      else
        m_t[0] = sdom.ParameterAt(cdom.NormalizedParameterAt(seg->Domain()[0]));
    }
  }
  if (Domain()[1] < domain[1]){
    bool chgd = false;
    ON_Curve* seg = SegmentCurve(Count()-1);
    if (!seg) return false;
    ON_Interval sdom = SegmentDomain(Count()-1);
    ON_Interval cdom = seg->Domain();
    double a = (sdom == cdom) ? domain[1] : cdom.ParameterAt(sdom.NormalizedParameterAt(domain[1]));
    ON_Interval DesiredDom(cdom[0], a);
    chgd = seg->Extend(DesiredDom);
    if (chgd) {
      if (seg->Domain() == DesiredDom)
        m_t[Count()] = domain[1];
      else
        m_t[Count()] = sdom.ParameterAt(cdom.NormalizedParameterAt(seg->Domain()[1]));
      changed = true;
    }
  }
 
  if (changed){
    DestroyCurveTree();
  }
 
  return changed;
}
 



ON_BOOL32 ON_PolyCurve::Split(
    double split_parameter,
    ON_Curve*& left_side, // left portion returned here
    ON_Curve*& right_side // right portion returned here
  ) const
{
  int si;
  ON_Interval dom = Domain();
  
  ON_PolyCurve* pLeftSide  = ON_PolyCurve::Cast(left_side);
  ON_PolyCurve* pRightSide = ON_PolyCurve::Cast(right_side);

  if ( pLeftSide && pLeftSide != this )
    pLeftSide->Destroy();
  else if ( pLeftSide == this )
    pLeftSide->DestroyCurveTree();

  if ( pRightSide && pRightSide != this )
    pRightSide->Destroy();
  else if ( pRightSide == this )
    pRightSide->DestroyCurveTree();

  if ( left_side && !pLeftSide )
    return false;
  if ( right_side && !pRightSide )
    return false;
  if ( !dom.Includes( split_parameter, true ) )
    return false; // split_parameter is not an interior parameter


  const ON_BOOL32 bDupSegs = ( this != pLeftSide && this != pRightSide );

		/* 4 April 2003 Greg Arden		Made the following changes:
																		1.	Use ParameterSearch() to decide if we should snap domain
																				boundries to m_t array values.  
																		2.  Make sure resulting polycurves have Domain() specified as 
																				split parameter.   
																		3.  When true is returned the result passes IsValid().
		*/
	bool split_at_break = ParameterSearch(split_parameter, si, true);
	if( split_at_break && (si<=0 || si>=Count() ) )
		return false;

	ON_Interval s_dom = SegmentDomain(si);
  ON_Curve* seg_curve = SegmentCurve(si);
  if ( !seg_curve )
    return false;
  ON_Interval c_dom = seg_curve->Domain();

  double c;
	if( split_at_break)
		c = c_dom[0];
	else
		c = ( c_dom == s_dom )
           ? split_parameter
           : c_dom.ParameterAt( s_dom.NormalizedParameterAt(split_parameter) );

  ON_Curve* seg_left = 0;
  ON_Curve* seg_right = 0;

  if ( !split_at_break  && c_dom.Includes(c,true) )
  {
    if ( !seg_curve->Split( c, seg_left, seg_right ) )
    {
      double fuzz = 0.001; // anything small and > 1.0e-6 will work about the same
      if ( c_dom.NormalizedParameterAt(c) <= fuzz )
        c = c_dom[0];
      else if ( c_dom.NormalizedParameterAt(c) >= 1.0 - fuzz )
        c = c_dom[1];
      else
        return false; // unable to split this segment
    }
  }
  else if ( c <= c_dom.ParameterAt(0.5) )
    c = c_dom[0];
  else
    c = c_dom[1];

  // use scratch arrays since this may also be pLeftSide or pRightSide
  ON_SimpleArray< ON_Curve* > left_segment;
  ON_SimpleArray< ON_Curve* > right_segment;
  ON_SimpleArray< double > left_t;
  ON_SimpleArray< double > right_t;

  int i;

  if ( seg_left && seg_right )
  {
    // we split a segment
    left_segment.Reserve(si+1);
    right_segment.Reserve(m_segment.Count()-si);
    left_t.Reserve(left_segment.Count()+1);
    right_t.Reserve(right_segment.Count()+1);
    if ( !bDupSegs )
    {
      delete m_segment[si];
      const_cast<ON_PolyCurve*>(this)->m_segment[si] = 0;
    }
    
    for ( i = 0; i < si; i++ )
    {
      if ( bDupSegs )
        left_segment.Append( m_segment[i]->Duplicate() );
      else
        left_segment.Append( m_segment[i] );
      left_t.Append( m_t[i] );
    }
    left_segment.Append( seg_left );
    left_t.Append( m_t[si] );
    left_t.Append( split_parameter );

    right_segment.Append(seg_right);
    right_t.Append( split_parameter );
    for ( i = si+1; i < m_segment.Count(); i++ )
    {
      if ( bDupSegs )
        right_segment.Append( m_segment[i]->Duplicate() );
      else
        right_segment.Append( m_segment[i] );
      right_t.Append( m_t[i] );
    }
    right_t.Append( m_t[m_segment.Count()] );
  }
  else 
  {
    if ( c == c_dom[1] )
      si++;
		if( (c==c_dom[0] && si==0 ) ||								// attempting split at curve start
				(c==c_dom[1] && si==m_segment.Count() ) )	// attempting split at curve end
			return false;

    left_segment.Reserve(si);
    right_segment.Reserve(m_segment.Count()-si);
    left_t.Reserve(left_segment.Count()+1);
    right_t.Reserve(right_segment.Count()+1);
    
    for ( i = 0; i < si; i++ )
    {
      if ( bDupSegs )
        left_segment.Append( m_segment[i]->Duplicate() );
      else
        left_segment.Append( m_segment[i] );
      left_t.Append( m_t[i] );
    }
    left_t.Append( split_parameter );

    for ( i = si; i < m_segment.Count(); i++ )
    {
      if ( bDupSegs )
        right_segment.Append( m_segment[i]->Duplicate() );
      else
        right_segment.Append( m_segment[i] );
      if ( i == si )
        right_t.Append( split_parameter );
      else
        right_t.Append( m_t[i] );
    }
    right_t.Append( m_t[m_segment.Count()] );
  }

  if ( !pLeftSide )
    pLeftSide = new ON_PolyCurve();
  if ( !pRightSide )
    pRightSide = new ON_PolyCurve();
  if ( !bDupSegs )
  {
    // pLeftSide or pRightSide is the same as this
    ON_PolyCurve* this_ptr = const_cast<ON_PolyCurve*>(this);
    this_ptr->m_segment.Zero();
    this_ptr->m_t.Zero();
    this_ptr->m_segment.SetCount(0);
    this_ptr->m_t.SetCount(0);
  }

  pLeftSide->m_segment.Append( left_segment.Count(), left_segment.Array() );
  pLeftSide->m_t.Append( left_t.Count(), left_t.Array() );
  pRightSide->m_segment.Append( right_segment.Count(), right_segment.Array() );
  pRightSide->m_t.Append( right_t.Count(), right_t.Array() );

  left_side = pLeftSide;
  right_side = pRightSide;

  return true;
}

// Flatten a poly curve reparameterized over pdom. 
// Harvests all the segments recursively and places them in the arrays
static
void Flatten( ON_PolyCurve* poly, ON_Interval pdom, ON_SimpleArray<double>& new_t, ON_SimpleArray<ON_Curve*>& new_seg){
		int n= poly->Count();
		double t0 = pdom[0];
		ON_Interval pcdom = poly->Domain();
		for(int i=0; i<n; i++){
			double sdom=poly->SegmentDomain(i)[1];
			double ndom=pcdom.NormalizedParameterAt(sdom);	
			double t1 =pdom.ParameterAt(ndom);
			ON_Curve* seg = poly->SegmentCurve(i);
			ON_PolyCurve* spoly =  ON_PolyCurve::Cast(seg);
			if(spoly){
				Flatten(spoly, ON_Interval(t0,t1), new_t, new_seg );
				poly->HarvestSegment(i);		
				delete spoly;
			} else {
				new_t.Append(t1);
				new_seg.Append(seg);
				poly->HarvestSegment(i);
			}
			t0 = t1;
		}
} 

bool ON_PolyCurve::HasSynchronizedSegmentDomains() const
{
  double t0, t1;
  int i, count = m_segment.Count();
  const ON_Curve* const * c = m_segment.Array();
  if ( count < 1 || 0 == c )
    return false;
  if ( count != m_t.Count()+1 )
    return false;
  const double* t = m_t.Array();
  if ( 0 == t )
    return false;

  for ( i = 0; i < count; i++ )
  {
    t0 = -ON_UNSET_VALUE;
    t1 = ON_UNSET_VALUE;
    if ( 0 != c[i] 
         && c[i]->GetDomain(&t0,&t1) 
         && t0 == t[i]
         && t1 == t[i+1]
         )
     {
       continue;
     }
     return false;
  }

  return true;  
}

/* 
Description:
  Sets the domain of the curve int the m_segment[] array to exactly
  match the domain defined in the m_t[] array.  This is not required,
  but can simplify some coding situations.
Returns:
  True if at least one segment was reparameterized. False if no
  changes were made.
*/
bool ON_PolyCurve::SynchronizeSegmentDomains()
{
  double t0, t1;
  int i, count = m_segment.Count();
  ON_Curve** c = m_segment.Array();
  if ( count < 1 || 0 == c )
    return false;
  if ( count+1 != m_t.Count() )
    return false;
  const double* t = m_t.Array();
  if ( 0 == t )
    return false;

  bool rc = false;
  for ( i = 0; i < count; i++ )
  {
    if ( !c[i] )
      continue;
    t0 = -ON_UNSET_VALUE;
    t1 = ON_UNSET_VALUE;
    if ( c[i]->GetDomain(&t0,&t1) 
         && t0 == t[i]
         && t1 == t[i+1]
         )
    {
     continue;
    }

    if (    ON_IsValid(t[i]) 
        && ON_IsValid(t[i+1])
        && t[i] < t[i+1] 
        && c[i]->SetDomain(t[i],t[i+1]) 
      )
    {
     rc = true; // indicates a change was made
    }
  }
  return rc;
}



bool ON_PolyCurve::RemoveNestingEx( )
{
  // 19 March 2003 Dale Lear
  //     Added the true/false return code to RemoveNesting()
  //     so that CRhinoDoc::AddObjects() can easily detect when
  //     nested polycurves were added.  I had to add use
  //     a new function name to avoid breaking the SDK.
  bool rc = false;
	int n = Count();

	ON_SimpleArray<double> old_t = m_t;
	ON_SimpleArray<ON_Curve*> old_seg = m_segment;

	m_t.SetCount(1);
	m_segment.SetCount(0);

	for(int i=0; i<n;i++){
		ON_PolyCurve* poly = ON_PolyCurve::Cast( old_seg[i]);
		if(poly){
      rc = true;
			Flatten( poly, ON_Interval(old_t[i], old_t[i+1]), m_t, m_segment );
			delete poly;
		} else {
			m_t.Append( old_t[i+1]); 
			m_segment.Append( old_seg[i] );
		}
	}
  return rc;
}

void  ON_PolyCurve::RemoveNesting( )
{
  RemoveNestingEx();
}

bool ON_PolyCurve::IsNested() const
{
  int i, count = m_segment.Count();
	for ( i = 0; i < count; i++ )
  {
    if (  ON_PolyCurve::Cast(m_segment[i]) )
      return true;
  }
  return false;
}


//   Sets the m_segment[index] to crv. 
void ON_PolyCurve::SetSegment(int i, ON_Curve* crv){
	if(i>=0 && i<Count())
		m_segment[i] = crv;
}

// returns true if t is sufficiently close to m_t[index]
bool ON_PolyCurve::ParameterSearch(double t, int& index, bool bEnableSnap) const{
	return ON_Curve::ParameterSearch(t, index, bEnableSnap, m_t, ON_SQRT_EPSILON);
}

const ON_CurveArray& ON_PolyCurve::SegmentCurves() const
{
  return m_segment;
}

const ON_SimpleArray<double>& ON_PolyCurve::SegmentParameters() const
{
  return m_t;
}

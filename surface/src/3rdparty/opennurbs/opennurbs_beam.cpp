#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

static bool ON_ExtrusionPolyCurveProfileIsNotValid()
{
  return false; // good place for a breakpoint
}

bool ON_Extrusion::IsValidPolyCurveProfile( const ON_PolyCurve& polycurve, ON_TextLog* text_log )
{
  const bool bAllowGaps = true;
  bool rc = polycurve.IsValid(bAllowGaps,text_log) ? true : ON_ExtrusionPolyCurveProfileIsNotValid();
  if (!rc)
    return ON_ExtrusionPolyCurveProfileIsNotValid();

  const int profile_count = polycurve.Count();

  if ( profile_count < 1 )
  {
    if ( text_log )
    {
      text_log->Print("polycurve has < 1 segments.\n");
    }
    return ON_ExtrusionPolyCurveProfileIsNotValid();
  }

  if ( 2 != polycurve.Dimension() )
  {
    if ( 3 != polycurve.Dimension() )
    {
      if ( text_log )
      {
        text_log->Print("polycurve dimension = %d (should be 2).\n",polycurve.Dimension());
      }
      return ON_ExtrusionPolyCurveProfileIsNotValid();
    }
    ON_BoundingBox bbox = polycurve.BoundingBox();
    if ( !bbox.IsValid() )
    {
      if ( text_log )
      {
        text_log->Print("polycurve.BoundingBox() is not valid.\n");
      }
      return ON_ExtrusionPolyCurveProfileIsNotValid();
    }
    if ( !( 0.0 == bbox.m_min.z) || !(0.0 == bbox.m_max.z) )
    {
      if ( text_log )
      {
        text_log->Print("polycurve.BoundingBox() z values are not both 0.0.\n");
      }
      return ON_ExtrusionPolyCurveProfileIsNotValid();
    }
  }

  if ( 1 == profile_count )
    return true;

  if ( profile_count > 1 )
  {
    for ( int i = 0; i < profile_count; i++ )
    {
      const ON_Curve* segment = polycurve.SegmentCurve(i);
      if ( 0 == segment )
      {
        if ( text_log )
        {
          text_log->Print("polycurve.SegmentCurve(%d) is null.\n",i);
        }
        return ON_ExtrusionPolyCurveProfileIsNotValid();
      }
      if ( !segment->IsClosed() )
      {
        if ( text_log )
        {
          text_log->Print("polycurve.SegmentCurve(%d) is not closed.\n",i);
        }
        return ON_ExtrusionPolyCurveProfileIsNotValid();
      }
      if ( segment->Domain() != polycurve.SegmentDomain(i) )
      {
        if ( text_log )
        {
          text_log->Print("polycurve.Segment(%d).Domain() does not match polycurve.SegmentDomain(%d).\n",i,i);
        }
        return ON_ExtrusionPolyCurveProfileIsNotValid();
      }
    }
  }
  return true;
}

bool ON_Extrusion::CleanupPolyCurveProfile( ON_PolyCurve& polycurve )
{
  if ( !ON_Extrusion::IsValidPolyCurveProfile(polycurve) )
  {
    // see if we can fix it
    int i;
    const int old_count = polycurve.Count();
    if ( old_count <= 1 )
      return false;

    // make segments 2d 
    for ( i = 0; i < old_count; i++ )
    {
      ON_Curve* old_segment = polycurve.SegmentCurve(i);
      if ( 0 == old_segment )
        return false;
      if ( 2 != old_segment->Dimension() && !old_segment->ChangeDimension(2) )
        return false;
    }

    // make segment domains match the polycurve's m_t[] array
    polycurve.SynchronizeSegmentDomains();

    // make each segment a closed curve
    ON_SimpleArray<ON_PolyCurve*> new_polycurves(old_count);
    ON_SimpleArray<ON_Curve*> new_segments(old_count);
    ON_PolyCurve* new_segment = 0;
    bool rc = true;
    for ( i = 0; i < old_count && rc; i++ )
    {
      ON_Curve* old_segment = polycurve.SegmentCurve(i);
      if ( old_segment->IsClosed() )
      {
        if ( 0 != new_segment )
        {
          rc = false;
          break;
        }
        new_segments.Append(old_segment);
      }
      else if ( 0 == new_segment )
      {
        new_segment = new ON_PolyCurve();
        new_polycurves.Append(new_segment);
        new_segment->Append(old_segment);
      }
      else
      {
        new_segment->Append(old_segment);
        if ( new_segment->FindNextGap(0) )
        {
          rc = false;
          break;
        }
        if ( new_segment->IsClosed() )
        {
          new_segments.Append(new_segment);
          new_segment = 0;
        }
      }
    }

    if ( 0 != new_segment )
    {
      rc = false;
    }

    if ( !rc )
    {
      // unable to fix polycurve. Delete the new stuff we allocated.
      for ( i = 0; i < new_polycurves.Count(); i++ )
      {
        new_segment = new_polycurves[i];
        if ( new_segment )
        {
          for ( int j =  new_segment->Count()-1; j >= 0; j-- )
          {
            new_segment->HarvestSegment(j); // do not delete parts of input polycurve
          }
          delete new_segment;
        }
      }
      return false;
    }

    for ( i = 0; i < new_polycurves.Count(); i++ )
    {
      new_polycurves[i]->RemoveNesting();
    }

    for ( i = old_count-1; i >= 0; i-- )
    {
      polycurve.HarvestSegment(i);
      polycurve.Remove(i);
    }
    for ( i = 0; i < new_segments.Count(); i++ )
    {
      polycurve.Append(new_segments[i]);
    }
  }
  else
  {
    polycurve.ChangeDimension(2);
  }
  return true;
}


bool ON_GetEndCapTransformation(ON_3dPoint P, ON_3dVector T, ON_3dVector U, 
                                const ON_3dVector* Normal, 
                                ON_Xform& xform,
                                ON_Xform* scale2d,
                                ON_Xform* rot3d
                                )
{
  if ( scale2d )
    scale2d->Identity();
  if ( rot3d )
    rot3d->Identity();
  if ( !T.IsUnitVector() && !T.Unitize() )
    return false;
  if ( !U.IsUnitVector() && !U.Unitize() )
    return false;
  ON_3dVector N(0.0,0.0,0.0);
  if ( Normal )
  {
    N = *Normal;
    if ( !N.IsUnitVector() && !N.Unitize() )
      N.Zero();
  }

  ON_Plane p0;
  p0.origin = P;
  p0.zaxis = T;
  p0.yaxis = U;
  p0.xaxis = ON_CrossProduct(U,T);
  if ( !p0.xaxis.IsUnitVector() )
    p0.xaxis.Unitize();
  p0.UpdateEquation();
  xform.Rotation(ON_xy_plane,p0);
  if ( rot3d )
    *rot3d = xform;
  if ( N.z > ON_Extrusion::m_Nz_min && N.IsUnitVector() )
  {
    //double cosa = T.x*N.x + T.y*N.y + T.z*N.z; // N is relative to T
    double cosa = N.z; // N is relative to xy plane.
    for(;;)
    {
      ON_3dVector A(-N.y,N.x,0.0); // N is relative to xy plane.
      if ( !A.IsValid() )
        break;
      double sina = A.Length();
      if ( !ON_IsValid(sina) )
        break;
      if ( !A.Unitize() )
        break;
      
      // S is a non-uniform scale that maps A to A and perpA to 1/cosa*perpA.
      // The scale distorts the profile so that after it is rotated
      // into the miter plane, the projection of the rotated profile
      // onto the xy-plane matches the original profile.
      ON_Xform S(0.0);
      const double c = 1.0 - 1.0/cosa;
      S.m_xform[0][0] = 1.0 - c*A.y*A.y;
      S.m_xform[0][1] = c*A.x*A.y;

      S.m_xform[1][0] = S.m_xform[0][1];
      S.m_xform[1][1] = 1.0 - c*A.x*A.x;

      S.m_xform[2][2] = 1.0;

      S.m_xform[3][3] = 1.0;
      if (scale2d)
        *scale2d = S;

      // R rotates the profile plane so its normal is equal to N
      ON_Xform R;
      R.Rotation(sina,cosa,A,ON_origin);
      if ( rot3d )
        *rot3d = xform*R;

      xform = xform*R*S;
      break;
    }
  }
  return true;
}

static void ON_ExtrusionInitializeHelper(ON_Extrusion& extrusion)
{
  extrusion.m_path.from.Zero();
  extrusion.m_path.to.Zero();
  extrusion.m_t.m_t[0] = 0.0;
  extrusion.m_t.m_t[1] = 1.0;
  extrusion.m_up.Zero();
  extrusion.m_profile_count = 0;
  extrusion.m_profile = 0;
  extrusion.m_bCap[0] = false;
  extrusion.m_bCap[1] = false;
  extrusion.m_bHaveN[0] = false;
  extrusion.m_bHaveN[1] = false;
  extrusion.m_N[0].Zero();
  extrusion.m_N[1].Zero();
  extrusion.m_path_domain.m_t[0] = 0.0;
  extrusion.m_path_domain.m_t[1] = 1.0;
  extrusion.m_bTransposed = false;
}

static void ON_ExtrusionCopyHelper(const ON_Extrusion& src,ON_Extrusion& dst)
{
  if ( &src != &dst )
  {
    if ( dst.m_profile )
    {
      delete dst.m_profile;
      dst.m_profile = 0;
    }
    dst.m_path = src.m_path;
    dst.m_t = src.m_t;
    dst.m_up = src.m_up;
    dst.m_profile_count = src.m_profile_count;
    dst.m_profile = src.m_profile 
                  ? src.m_profile->DuplicateCurve() 
                  : 0;
    dst.m_bCap[0] = src.m_bCap[0];
    dst.m_bCap[1] = src.m_bCap[1];
    dst.m_bHaveN[0] = src.m_bHaveN[0];
    dst.m_bHaveN[1] = src.m_bHaveN[1];
    dst.m_N[0] = src.m_N[0];
    dst.m_N[1] = src.m_N[1];
    dst.m_path_domain = src.m_path_domain;
    dst.m_bTransposed  = src.m_bTransposed;
  }
}

bool ON_Extrusion::SetPath(ON_3dPoint A, ON_3dPoint B)
{
  double distAB = 0.0;
  bool rc =    A.IsValid() && B.IsValid() 
            && (distAB = A.DistanceTo(B)) > ON_ZERO_TOLERANCE;
  if (rc)
  {
    m_path.from = A;
    m_path.to = B;
    m_t.Set(0.0,1.0);
    m_path_domain.Set(0.0,distAB);
  }
  return rc;
}

bool ON_Extrusion::SetPathAndUp( ON_3dPoint A, ON_3dPoint B, ON_3dVector up )
{
  double distAB = 0.0;

  bool rc =    up.IsValid() 
            && up.Length() > ON_ZERO_TOLERANCE
            && A.IsValid() 
            && B.IsValid() 
            && (distAB = A.DistanceTo(B)) > ON_ZERO_TOLERANCE;

  if (rc)
  {
    ON_3dVector D = A-B;
    D.Unitize();
    double d = up*D;
    if ( !up.IsUnitVector() || fabs(d) > distAB*ON_SQRT_EPSILON*0.015625 )
    {
      // need to make up perpendicular to the line segment
      // and unitize.
      D.Unitize();
      up = up - d*D;
      up.Unitize();
      // validate up
      d = up*D;
      rc = ( up.IsUnitVector() && fabs(d) <= ON_SQRT_EPSILON );
    }

    if (rc)
    {
      m_path.from = A;
      m_path.to = B;
      m_t.Set(0.0,1.0);
      m_path_domain.Set(0.0,distAB);
      m_up = up;
    }
  }

  return rc;
}

int ON_Extrusion::PathParameter() const
{
  return m_bTransposed ? 0 : 1;
}

int ON_Extrusion::ProfileParameter() const
{
  return m_bTransposed ? 1 : 0;
}

ON_3dPoint ON_Extrusion::PathStart() const
{
  ON_3dPoint P(ON_UNSET_POINT);
  const double t = m_t.m_t[0];
  if ( 0.0 <= t && t <= 1.0 && m_path.IsValid() )
    P = m_path.PointAt(t);
  return P;
}

ON_3dPoint ON_Extrusion::PathEnd() const
{
  ON_3dPoint P(ON_UNSET_POINT);
  const double t = m_t.m_t[1];
  if ( 0.0 <= t && t <= 1.0 && m_path.IsValid() )
    P = m_path.PointAt(t);
  return P;
}

ON_3dVector ON_Extrusion::PathTangent() const
{
  ON_3dVector T(ON_UNSET_VECTOR);
  if ( m_path.IsValid() )
    T = m_path.Tangent();
  return T;
}

void ON_Extrusion::Destroy()
{
  if ( m_profile)
  {
    delete m_profile;
    m_profile = 0;
  }
  ON_ExtrusionInitializeHelper(*this);
  DestroyRuntimeCache();
  PurgeUserData();
}

bool ON_Extrusion::SetMiterPlaneNormal(ON_3dVector N, int end)
{
  bool rc = false;
  if ( end >= 0 && end <= 1 )
  {
    if (    N.IsValid() 
         && N.z > ON_Extrusion::m_Nz_min
         && (N.IsUnitVector() || N.Unitize())
       )
    {
      if (fabs(N.x) <= ON_SQRT_EPSILON && fabs(N.y) <= ON_SQRT_EPSILON)
        N.Set(0.0,0.0,1.0);
      m_N[end] = N;
      m_bHaveN[end] = (N.z != 1.0);
      rc = true;
    }
    else if ( N.IsZero() || ON_UNSET_VECTOR == N )
    {
      m_bHaveN[end] = false;
      rc = true;
    }
  }
  return rc;
}

void ON_Extrusion::GetMiterPlaneNormal(int end, ON_3dVector& N) const
{
  if ( end >= 0 && end <= 1 && m_bHaveN[end] )
    N = m_N[end];
  else
    N.Set(0.0,0.0,1.0);
}

int ON_Extrusion::IsMitered() const
{
  int rc = 0;
  if ( m_bHaveN[0] && m_N[0].IsUnitVector() && m_N[0].z > m_Nz_min && (m_N[0].x != 0.0 || m_N[0].y != 0.0) )
    rc += 1;
  if ( m_bHaveN[1] && m_N[1].IsUnitVector() && m_N[1].z > m_Nz_min && (m_N[1].x != 0.0 || m_N[1].y != 0.0) )
    rc += 2;
  return rc;
}

int ON_Extrusion::CapCount() const
{
  // Returns number of end caps.
  switch (IsCapped())
  {
  case 1:
  case 2:
    return 1;
  case 3:
    return 2;
  }
  return 0;
}

int ON_Extrusion::IsCapped() const
{
  // 0 = no caps, 1 = bottom cap, 2 = top cap, 3 = both caps

  if ( !m_bCap[0] && !m_bCap[1] )
    return 0;

  if ( m_profile_count < 1 || 0 == m_profile )
    return 0;

  if ( 1 == m_profile_count )
  {
    if ( !m_profile->IsClosed() )
      return 0;
  }
  else if ( m_profile_count > 1 )
  {
    const ON_PolyCurve* p = ON_PolyCurve::Cast(m_profile);
    if ( 0 == p )
      return 0;
    const ON_Curve* outer_profile = p->SegmentCurve(0);
    if ( 0 == outer_profile )
      return 0;
    if ( !outer_profile->IsClosed() )
      return 0;
  }

  return (m_bCap[0] ? (m_bCap[1] ? 3 : 1) : 2);
}

int ON_Extrusion::FaceCount() const
{
  int face_count = 0;
  const ON_Curve* profile0 = Profile(0);

  if ( m_profile_count > 0 && 0 != profile0 )
  {
    int is_capped = IsCapped();
    if ( is_capped != 0 && !profile0->IsClosed() )
    {
      is_capped = 0;
    }

    switch(is_capped)
    {
    case 1: // single bottom cap + sides
    case 2: // single top cap + sides
      face_count = m_profile_count + 1;
      break;

    case 3: // bottom and top cap + sides
      face_count = m_profile_count + 2;
      break;

    default: // no caps
      face_count = 1;
      break;
    }
  }

  return face_count;
}


bool ON_Extrusion::IsSolid() const
{
  if ( !m_bCap[0] || !m_bCap[1] )
    return false;
  return 3 == IsCapped();
}

bool ON_Extrusion::GetPathPlane( double s, ON_Plane& plane ) const
{
  ON_Plane p;
  p.origin = ON_3dPoint::Origin;
  p.zaxis = PathTangent();
  p.yaxis = m_up;
  p.xaxis = ON_CrossProduct(p.yaxis,p.zaxis);
  if ( !p.xaxis.Unitize() )
    return false;
  if ( !p.yaxis.Unitize() )
    return false;
  p.UpdateEquation();
  if ( !p.IsValid() )
  {
    p.yaxis = ON_CrossProduct(p.zaxis,p.xaxis);
    p.yaxis.Unitize();
    if ( !p.IsValid() )
      return false;
  }
  p.origin = m_path.PointAt(m_t.ParameterAt(s));
  p.UpdateEquation();
  plane = p;
  return plane.IsValid();
}

bool ON_Extrusion::GetProfilePlane( double s, ON_Plane& plane ) const
{
  ON_Plane p;
  p.origin = ON_3dPoint::Origin;
  p.zaxis = PathTangent();
  p.yaxis = m_up;
  p.xaxis = ON_CrossProduct(p.yaxis,p.zaxis);
  if ( !p.xaxis.Unitize() )
    return false;
  if ( !p.yaxis.Unitize() )
    return false;
  p.UpdateEquation();
  if ( !p.IsValid() )
  {
    p.yaxis = ON_CrossProduct(p.zaxis,p.xaxis);
    p.yaxis.Unitize();
    if ( !p.IsValid() )
      return false;
  }
  if (    (!m_bHaveN[0] || (0.0 == m_N[0].x && 0.0 == m_N[0].y)) 
       && (!m_bHaveN[1] || (0.0 == m_N[1].x && 0.0 == m_N[1].y)) 
     )
  {
    p.origin = m_path.PointAt(m_t.ParameterAt(s));
    p.UpdateEquation();
    plane = p;
  }
  else
  {
    ON_Xform xform;
    if ( !GetProfileTransformation(s,xform) )
      return false;
    if (!p.Transform(xform))
      return false;
    plane = p;
  }
  return plane.IsValid();
}


bool ON_Extrusion::GetProfileTransformation( double s, ON_Xform& xform ) const
{
  //const ON_3dVector* N = (end?(m_bHaveN[1]?&m_N[1]:0):(m_bHaveN[0]?&m_N[0]:0));
  const ON_3dVector T = m_path.Tangent();
  if ( 0.0 == s )
  {
    return ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[0]),T,m_up,m_bHaveN[0]?&m_N[0]:0,xform,0,0);
  }
  if ( 1.0 == s )
  {
    return ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[1]),T,m_up,m_bHaveN[1]?&m_N[1]:0,xform,0,0);
  }
  ON_Xform xform0, xform1;
  if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[0]),T,m_up,m_bHaveN[0]?&m_N[0]:0,xform0,0,0) )
    return false;
  if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[1]),T,m_up,m_bHaveN[1]?&m_N[1]:0,xform1,0,0) )
    return false;

  const double s0 = 1.0-s;
  xform.m_xform[0][0] = s0*xform0.m_xform[0][0] + s*xform1.m_xform[0][0];
  xform.m_xform[0][1] = s0*xform0.m_xform[0][1] + s*xform1.m_xform[0][1];
  xform.m_xform[0][2] = s0*xform0.m_xform[0][2] + s*xform1.m_xform[0][2];
  xform.m_xform[0][3] = s0*xform0.m_xform[0][3] + s*xform1.m_xform[0][3];
  xform.m_xform[1][0] = s0*xform0.m_xform[1][0] + s*xform1.m_xform[1][0];
  xform.m_xform[1][1] = s0*xform0.m_xform[1][1] + s*xform1.m_xform[1][1];
  xform.m_xform[1][2] = s0*xform0.m_xform[1][2] + s*xform1.m_xform[1][2];
  xform.m_xform[1][3] = s0*xform0.m_xform[1][3] + s*xform1.m_xform[1][3];
  xform.m_xform[2][0] = s0*xform0.m_xform[2][0] + s*xform1.m_xform[2][0];
  xform.m_xform[2][1] = s0*xform0.m_xform[2][1] + s*xform1.m_xform[2][1];
  xform.m_xform[2][2] = s0*xform0.m_xform[2][2] + s*xform1.m_xform[2][2];
  xform.m_xform[2][3] = s0*xform0.m_xform[2][3] + s*xform1.m_xform[2][3];
  xform.m_xform[3][0] = s0*xform0.m_xform[3][0] + s*xform1.m_xform[3][0];
  xform.m_xform[3][1] = s0*xform0.m_xform[3][1] + s*xform1.m_xform[3][1];
  xform.m_xform[3][2] = s0*xform0.m_xform[3][2] + s*xform1.m_xform[3][2];
  xform.m_xform[3][3] = s0*xform0.m_xform[3][3] + s*xform1.m_xform[3][3];

  return true;
}

static bool CleanProfileSegment( ON_Curve* curve )
{
  ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(curve);
  if ( nurbs_curve )
  {
    nurbs_curve->RemoveSingularSpans();
    return ( nurbs_curve->IsValid() && false == nurbs_curve->SpanIsSingular(0) );
  }
  
  return true;
}

static bool ProfileHelper( int desired_orientation, ON_Curve* profile )
{
  // desired_orientation  0: outer profile that can be open or closed.
  // desired_orientation  1: outer profile that must be closed
  // desired_orientation -1: inner profile

  if ( 0 == profile )
  {
    ON_ERROR("ON_Extrusion::Set/Add Profile - null input curve pointer.");
    return false;
  }
  ON_BoundingBox bbox = profile->BoundingBox();
  if ( !bbox.IsValid() )
  {
    ON_ERROR("ON_Extrusion::Set/Add Profile - profile->BoundingBox() failed.");
    return false;
  }
  if ( fabs(bbox.m_min.z) > ON_ZERO_TOLERANCE || fabs(bbox.m_max.z) > ON_ZERO_TOLERANCE )
  {
    ON_ERROR("ON_Extrusion::Set/Add Profile - profile->BoundingBox() is not in the world xy plane.");
    return false;
  }
  if ( !profile->ChangeDimension(2) )
  {
    ON_ERROR("ON_Extrusion::Set/Add Profile - profile->ChangeDimension(2) failed.");
    return false;
  }

  if ( profile->IsClosed() )
  {
    int profile_orientation = ON_ClosedCurveOrientation(*profile,0);
    switch(desired_orientation)
    {
    case 1:
    case 0:
      if ( -1 == profile_orientation )
      {
        if ( !profile->Reverse() )
        {
          ON_ERROR("ON_Extrusion::SetOuterProfile() - profile->Reverse() failed.");
          return false;
        }
        profile_orientation = 1;
      }
      if ( 1 == desired_orientation && 1 != profile_orientation )
      {
        ON_ERROR("ON_Extrusion::SetOuterProfile() - profile has wrong orientation.");
        return false;
      }
      break;
    case -1:
      if ( 1 == profile_orientation )
      {
        if ( !profile->Reverse() )
        {
          ON_ERROR("ON_Extrusion::AddInnerProfile() - profile->Reverse() failed.");
          return false;
        }
        profile_orientation = -1;
      }
      if ( -1 != profile_orientation )
      {
        ON_ERROR("ON_Extrusion::AddInnerProfile() - profile has wrong orientation.");
        return false;
      }
      break;
    default:
      {
        ON_ERROR("ON_Extrusion::Set/Add Profile - invalid desired_orientation parameter.");
        return false;
      }
      break;
    }
  }
  else if ( 0 != desired_orientation )
  {
    ON_ERROR("ON_Extrusion::Set/Add Profile - profile is an open curve.");
    return false;
  }

  ON_PolyCurve* polycurve = ON_PolyCurve::Cast(profile);
  if ( 0 != polycurve )
  {
    polycurve->RemoveNesting();
    
    if ( polycurve->SegmentCurves().Count() < 1 )
    {
      ON_ERROR("ON_Extrusion::Set/Add Profile - ON_PolyCurve has no segments.");
      return false;
    }
    
    if ( polycurve->SegmentCurves().Count() + 1 != polycurve->SegmentParameters().Count() )
    {
      ON_ERROR("ON_Extrusion::Set/Add Profile - ON_PolyCurve segment and parameter counts do not agree.");
      return false;
    }

    for ( int i = polycurve->Count()-1; i >= 0; i-- )
    {
      ON_Curve* segment = polycurve->SegmentCurve(i);
      if ( !CleanProfileSegment(segment) )
        polycurve->Remove(i);
    }

    for ( int i = 0; i < polycurve->Count(); i++ )
    {
      ON_Curve* segment = polycurve->SegmentCurve(i);
      if ( 0 == segment )
      {
        ON_ERROR("ON_Extrusion::Set/Add Profile - ON_PolyCurve has null segment.");
        return false;
      }
      const ON_Interval segment_domain = polycurve->SegmentDomain(i);
      if ( !segment_domain.IsIncreasing() )
      {
        ON_ERROR("ON_Extrusion::Set/Add Profile - segment has invalid domain.");
        return false;
      }
      if ( !segment->SetDomain( segment_domain ) )
      {
        ON_ERROR("ON_Extrusion::Set/Add Profile - segment->SetDomain() failed.");
        return false;
      }
    }
  }
  else if ( !CleanProfileSegment(profile) )
  {
  }

  return true;
}

bool ON_Extrusion::SetOuterProfile( ON_Curve* outer_profile, bool bCap )
{
  if ( 0 != m_profile )
  {
    ON_ERROR("ON_Extrusion::SetOuterProfile() called when m_profile is already not null.");
    return false;
  }

  if ( !ProfileHelper( 0, outer_profile ) )
    return false;

  m_profile_count = 1;
  m_profile = outer_profile;

  if ( outer_profile->IsClosed() )
  {
    m_bCap[0] = m_bCap[1] = (bCap ? true : false);
  }
  else
  {
    m_bCap[0] = m_bCap[1] = false;
  }

  return true;
}

bool ON_Extrusion::AddInnerProfile( ON_Curve* inner_profile )
{
  if ( m_profile_count < 1 )
  {
    ON_ERROR("ON_Extrusion::AddInnerProfile() called when m_profile_count < 1.");
    return false;
  }
  if ( 0 == m_profile )
  {
    ON_ERROR("ON_Extrusion::AddInnerProfile() called when m_profile is null.");
    return false;
  }

  if ( 1 == m_profile_count && !m_profile->IsClosed() )
  {
    ON_ERROR("ON_Extrusion::AddInnerProfile() called when outer profile is not closed.");
    return false;
  }

  ON_PolyCurve* polycurve = ON_PolyCurve::Cast(m_profile);
  if (  m_profile_count > 1 && 0 == polycurve )
  {
    ON_ERROR("ON_Extrusion::AddInnerProfile() called when  m_profile_count > 1 but m_profile is not an ON_PolyCurve.");
    return false;
  }
  if ( m_profile_count > 1 && m_profile_count != polycurve->Count() )
  {
    ON_ERROR("ON_Extrusion::AddInnerProfile() called when  m_profile_count > 1 but m_profile_count != m_profile->Count().");
    return false;
  }

  if ( !ProfileHelper( -1, inner_profile ) )
    return false;

  if ( 1 == m_profile_count )
  {
    if ( 0 != polycurve )
    {
      polycurve->RemoveNesting();
    }

    if ( 0 == polycurve || 1 != polycurve->Count() )
    {
      polycurve = new ON_PolyCurve();
      polycurve->Append(m_profile);
      m_profile = polycurve;
    }
  }

  polycurve->Append(inner_profile);
  if ( polycurve->SegmentDomain(m_profile_count) != inner_profile->Domain() )
  {
    inner_profile->SetDomain( polycurve->SegmentDomain(m_profile_count) );

    // If inner_profile is itself a polycurve, clean up segment domains
    // so we don't get bugs caused by fuzz as we adjust parameters when evaluating.
    polycurve = ON_PolyCurve::Cast(inner_profile);
    if ( 0 != polycurve )
    {
      polycurve->SynchronizeSegmentDomains();
    }
  }
  m_profile_count++;

 
  return true;
}

const ON_PolyCurve* ON_Extrusion::PolyProfile() const
{
  if ( m_profile_count <= 1 )
    return 0;
  const ON_PolyCurve* poly_profile = ON_PolyCurve::Cast(m_profile);
  return (0 != poly_profile && m_profile_count == poly_profile->Count() ) ? poly_profile : 0;
}

const ON_Curve* ON_Extrusion::Profile(int profile_index) const
{
  if ( 0 == profile_index && 1 == m_profile_count )
    return m_profile;
  if ( profile_index < 0 || profile_index > m_profile_count )
    return 0;
  const ON_PolyCurve* poly_profile = PolyProfile();
  return ( 0 != poly_profile ? poly_profile->SegmentCurve(profile_index) : 0 );
}

ON_Curve* ON_Extrusion::Profile3d( ON_COMPONENT_INDEX ci ) const
{
  double s = ON_UNSET_VALUE;
  if ( ON_COMPONENT_INDEX::extrusion_bottom_profile == ci.m_type )
    s = 0.0;
  else if ( ON_COMPONENT_INDEX::extrusion_top_profile == ci.m_type )
    s = 1.0;
  else
    return 0;
  return Profile3d(ci.m_index,s);
}

ON_LineCurve* ON_Extrusion::PathLineCurve(ON_LineCurve* line_curve) const
{
  if ( !m_path.IsValid() )
    return 0;

  ON_Interval path_domain = Domain(PathParameter());
  if ( !path_domain.IsIncreasing() )
    return 0;

  if ( 0 == line_curve )
    line_curve = new ON_LineCurve();
  line_curve->m_line = m_path;
  line_curve->SetDomain( path_domain[0], path_domain[1] );

  return line_curve;
}


ON_Curve* ON_Extrusion::WallEdge( ON_COMPONENT_INDEX ci ) const
{
  if ( ON_COMPONENT_INDEX::extrusion_wall_edge != ci.m_type )
    return 0;
  if ( ci.m_index < 0 )
    return 0;

  int profile_index = ci.m_index/2;
  int profile_end   = ci.m_index % 2;
  const ON_Curve* profile2d = Profile(profile_index);
  if ( 0 == profile2d )
    return 0;

  ON_3dPoint profileP = profile_end
                      ? profile2d->PointAtEnd()
                      : profile2d->PointAtStart();
  if ( !profileP.IsValid() )
    return 0;
  profileP.z = 0.0;

  ON_Xform xform0, xform1;
  if ( !GetProfileTransformation(0.0,xform0) )
    return 0;
  if ( !GetProfileTransformation(1.0,xform1) )
    return 0;

  ON_Line line;
  line.from = xform0*profileP;
  line.to   = xform1*profileP;
  if ( !line.IsValid() )
    return 0;

  ON_LineCurve* line_curve = new ON_LineCurve();
  line_curve->m_line = line;
  
  ON_Interval path_domain = Domain(PathParameter());
  line_curve->SetDomain( path_domain[0], path_domain[1] );

  return line_curve;
}


ON_Surface* ON_Extrusion::WallSurface( ON_COMPONENT_INDEX ci ) const
{
  if ( ON_COMPONENT_INDEX::extrusion_wall_surface != ci.m_type )
    return 0;

  const ON_Curve* profile2d = Profile(ci.m_index);
  if ( 0 == profile2d )
    return 0;

  ON_Interval wall_profile2d_domain = m_path_domain;
  if ( m_profile_count > 1 )
  {
    const ON_PolyCurve* polyprofile2d = PolyProfile();
    if ( 0 == polyprofile2d )
      return 0;
    if ( polyprofile2d->Count() != m_profile_count )
      return 0;
    wall_profile2d_domain = polyprofile2d->SegmentDomain(ci.m_index);
  }

  ON_Curve* wall_profile2d = profile2d->DuplicateCurve();
  if ( 0 == wall_profile2d )
    return 0;
  wall_profile2d->SetDomain(wall_profile2d_domain);
  wall_profile2d->ChangeDimension(2);

  ON_Extrusion* wall = new ON_Extrusion();
  wall->SetOuterProfile(wall_profile2d,false);

  wall->m_path = m_path;
  wall->m_t    = m_t;
  wall->m_up   = m_up;
  wall->m_bHaveN[0] = m_bHaveN[0];
  wall->m_bHaveN[1] = m_bHaveN[1];
  wall->m_N[0] = m_N[0];
  wall->m_N[1] = m_N[1];
  wall->m_bTransposed = m_bTransposed;

  return wall;
}

ON_Curve* ON_Extrusion::Profile3d( int profile_index, double s ) const
{
  if ( profile_index < 0 || !(0.0 <= s && s <= 1.0) || 0 == m_profile )
    return 0;

  ON_Xform xform;
  if ( !GetProfileTransformation(s,xform) )
    return 0;

  const ON_Curve* profile2d = Profile(profile_index);
  if ( 0 == profile2d )
    return 0;

  ON_Curve* profile3d = profile2d->DuplicateCurve();
  if ( 0 == profile3d )
    return 0;

  if (    !profile3d->ChangeDimension(3) 
       || !profile3d->Transform(xform)
     )
  {
    delete profile3d;
    return 0;
  }

  return profile3d;
}

int ON_Extrusion::ProfileIndex( double profile_parameter ) const
{
  if ( !m_profile )
    return -1;

  if ( m_profile_count < 1 )
    return -1;

  if ( 1 == m_profile_count )
  {
    return m_profile->Domain().Includes(profile_parameter,false) ? 0 : -1;
  }

  const ON_PolyCurve* polycurve = PolyProfile();
  if ( 0 == polycurve )
    return -1;

  const ON_SimpleArray<double>& polycurve_t = polycurve->SegmentParameters();
  if ( polycurve_t.Count() != m_profile_count+1 )
    return -1;

  int profile_index = ON_SearchMonotoneArray( polycurve_t.Array(), polycurve_t.Count(), profile_parameter );
  if ( profile_index == m_profile_count )
    profile_index = m_profile_count-1;
  else if ( profile_index < 0 || profile_index > m_profile_count )
    profile_index = -1;
  return profile_index;
}

int ON_Extrusion::ProfileCount() const
{
  if ( !m_profile )
    return 0;

  if ( m_profile_count < 1 )
    return 0;

  if ( m_profile_count > 1 )
  {
    const ON_PolyCurve* polycurve = ON_PolyCurve::Cast(m_profile);
    if ( 0 == polycurve )
      return 0;
    if ( polycurve->Count() != m_profile_count )
      return 0;
  }

  return m_profile_count;
}


int ON_Extrusion::GetProfileCurves( ON_SimpleArray< const ON_Curve* >& profile_curves ) const
{
  if ( !m_profile )
    return 0;

  if ( m_profile_count < 1 )
    return 0;

  if ( 1 == m_profile_count )
  {
    profile_curves.Reserve(profile_curves.Count()+1);
    profile_curves.Append(m_profile);
  }
  else
  {
    const ON_PolyCurve* polycurve = ON_PolyCurve::Cast(m_profile);
    if ( 0 == polycurve )
      return 0;
    if ( polycurve->Count() != m_profile_count )
      return 0;
    const int count0 = profile_curves.Count();
    profile_curves.Reserve(count0+m_profile_count);
    for ( int i = 0; i < m_profile_count; i++ )
    {
      const ON_Curve* segment = polycurve->SegmentCurve(i);
      if ( 0 == segment )
      {
        profile_curves.SetCount(count0);
        return 0;
      }
      profile_curves.Append(segment);
    }
  }

  return m_profile_count;
}


const double ON_Extrusion::m_Nz_min = 1.0/64.0;

const double ON_Extrusion::m_path_length_min = ON_ZERO_TOLERANCE;

ON_OBJECT_IMPLEMENT(ON_Extrusion,ON_Surface,"36F53175-72B8-4d47-BF1F-B4E6FC24F4B9");

ON_Extrusion::ON_Extrusion()
{
  ON_ExtrusionInitializeHelper(*this);
}

ON_Extrusion::ON_Extrusion(const ON_Extrusion& src) : ON_Surface(src), m_profile(0)
{
  ON_ExtrusionCopyHelper(src,*this);
}

ON_Extrusion::~ON_Extrusion()
{
  if ( m_profile)
  {
    delete m_profile;
  }
}

ON_Extrusion& ON_Extrusion::operator=(const ON_Extrusion& src)
{
  if ( this != &src )
  {
    Destroy();
    ON_Surface::operator=(src);
    ON_ExtrusionCopyHelper(src,*this);
  }
  return *this;
}

static
void ON_Extrusion_IsNotValidMessage( ON_TextLog* text_log, const char* msg )
{
  // this is a good place for a breakpoint
  if ( text_log && msg && msg[0] )
    text_log->Print("%s\n",msg);
}

static bool ON_ExtrusionIsNotValid()
{
  return ON_IsNotValid(); // good place for a breakpoint
}

ON_BOOL32 ON_Extrusion::IsValid( ON_TextLog* text_log ) const
{
  // check m_profile
  if ( m_profile_count < 1 )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_profile_count < 1.");
    return ON_ExtrusionIsNotValid();
  }
  if ( !m_profile )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_profile is NULL.");
    return ON_ExtrusionIsNotValid();
  }
  if ( m_profile_count > 1 )
  {
    const ON_PolyCurve* c = ON_PolyCurve::Cast(m_profile);
    if ( 0 == c )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_profile_count > 1 but m_profile is not an ON_PolyCurve.");
      return ON_ExtrusionIsNotValid();
    }
    if ( m_profile_count != c->Count() )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_profile_count > 1 but m_profile_count != m_profile->SegmentCount().");
      return ON_ExtrusionIsNotValid();
    }

    if ( !ON_Extrusion::IsValidPolyCurveProfile(*c,text_log) )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_profile is not a valid ON_PolyCurve.");
      return ON_ExtrusionIsNotValid();
    }
    for ( int i = 0; i < m_profile_count; i++ )
    {
      const ON_Curve* segment = c->SegmentCurve(i);
      if ( 0 == segment )
      {
        ON_Extrusion_IsNotValidMessage(text_log,"m_profile_count > 1 but a m_profile_count->SegmentCurve() is null.");
        return ON_ExtrusionIsNotValid();
      }
      if ( !segment->IsClosed() )
      {
        ON_Extrusion_IsNotValidMessage(text_log,"m_profile_count > 1 but a m_profile_count->SegmentCurve() is not closed.");
        return ON_ExtrusionIsNotValid();
      }
    }
  }
  else if ( !m_profile->IsValid(text_log) )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_profile is not valid.");
    return ON_ExtrusionIsNotValid();
  }

  // check m_path
  if ( !m_path.IsValid() )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_path is not valid.");
    return ON_ExtrusionIsNotValid();
  }
  ON_3dVector D = m_path.to - m_path.from;
  double len = D.Length();
  if ( !ON_IsValid(len) || len <= 0.0 )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_path has zero length.");
    return ON_ExtrusionIsNotValid();
  }
  if ( !ON_IsValid(len) || len <= ON_Extrusion::m_path_length_min )
  {
    if ( text_log )
      text_log->Print("m_path has zero length <= ON_Extrusion::m_path_length_min.");
    return ON_ExtrusionIsNotValid();
  }
  if ( !D.Unitize() || !D.IsUnitVector() )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_path has zero direction.");
    return ON_ExtrusionIsNotValid();
  }
  
  // check m_t
  if ( !(0.0 <= m_t.m_t[0] && m_t.m_t[0] < m_t.m_t[1] && m_t.m_t[1] <= 1.0) )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_t does not satisfy 0<=m_t[0]<m_t[1]<=1");
    return ON_ExtrusionIsNotValid();
  }

  // check m_up
  if ( !m_up.IsUnitVector() )
  {
   ON_Extrusion_IsNotValidMessage(text_log,"m_up is not a unit vector.");
    return ON_ExtrusionIsNotValid();
  }
  len = m_up*D;
  if ( fabs(len) > ON_SQRT_EPSILON )
  {
    ON_Extrusion_IsNotValidMessage(text_log,"m_up is not perpendicular to m_path.");
    return ON_ExtrusionIsNotValid();
  }

  // validate ends
  if ( m_bHaveN[0] )
  {
    if ( !m_N[0].IsUnitVector() )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_N[0] is not a unit vector.");
      return ON_ExtrusionIsNotValid();
    }
    if ( !(m_N[0].z > ON_Extrusion::m_Nz_min) )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_N[0].z is too small (<=ON_Extrusion::m_Nz_min) or negative");
      return ON_ExtrusionIsNotValid();
    }
  }
  if ( m_bHaveN[1] )
  {
    if ( !m_N[1].IsUnitVector() )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_N[1] is not a unit vector.");
      return ON_ExtrusionIsNotValid();
    }
    if ( !(m_N[1].z > ON_Extrusion::m_Nz_min) )
    {
      ON_Extrusion_IsNotValidMessage(text_log,"m_N[1].z is too small (<=ON_Extrusion::m_Nz_min) or negative");
      return ON_ExtrusionIsNotValid();
    }
  }

  return true;
}

void ON_Extrusion::Dump( ON_TextLog& text_log ) const
{
  text_log.Print("ON_Extrusion: \n");
  {
    text_log.PushIndent();
  text_log.Print("Path: ");
  text_log.Print(m_path.PointAt(m_t[0]));
  text_log.Print(" ");
  text_log.Print(m_path.PointAt(m_t[1]));
  text_log.Print("\n");
  text_log.Print("Up: ");
  text_log.Print(m_up);
  text_log.Print("\n");
  text_log.Print("m_bCap[] = (%d, %d)\n",m_bCap[0],m_bCap[1]);
  text_log.Print("m_bHaveN[] = (%d, %d)\n",m_bHaveN[0],m_bHaveN[1]);
  text_log.Print("m_N[] = (");
  text_log.Print(m_N[0]);
  text_log.Print(", ");
  text_log.Print(m_N[1]);
  text_log.Print("\n");
  text_log.Print("m_path_domain = (%.17g, %.17g)\n",m_path_domain[0],m_path_domain[1]);
  text_log.Print("m_bTransposed = %d\n",m_bTransposed);
  text_log.Print("Profile Count: %d\n",m_profile_count);
  text_log.Print("Profile:\n");
    {
  text_log.PushIndent();
  if ( !m_profile )
    text_log.Print("NULL");
  else
    m_profile->Dump(text_log);
  text_log.PopIndent();
    }
    text_log.PopIndent();
  }
  return;
}

unsigned int ON_Extrusion::SizeOf() const
{
  unsigned int sz = sizeof(*this) - sizeof(ON_Surface);
  if ( m_profile )
    sz += m_profile->SizeOf();
  return sz;
}

ON__UINT32 ON_Extrusion::DataCRC(ON__UINT32 current_remainder) const
{
  if ( m_profile )
    current_remainder = m_profile->DataCRC(current_remainder);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_path),&m_path);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_t),&m_t);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_up),&m_up);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bHaveN[0]), &m_bHaveN[0]);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bHaveN[1]), &m_bHaveN[1]);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_N[0]), &m_N[0]);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_N[1]), &m_N[1]);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_path_domain), &m_path_domain);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bTransposed), &m_bTransposed);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_profile_count), &m_profile_count);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bCap[0]), &m_bCap[0]);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bCap[1]), &m_bCap[1]);
  if ( m_profile )
    current_remainder = m_profile->DataCRC(current_remainder);
  return current_remainder;
}

ON_BOOL32 ON_Extrusion::Write(
       ON_BinaryArchive& binary_archive
     ) const
{
  bool rc = binary_archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,2);
  if (!rc)
    return false;
  for (;;)
  {
    rc = binary_archive.WriteObject(m_profile);
    if (!rc) break;
    rc = binary_archive.WriteLine(m_path);
    if (!rc) break;
    rc = binary_archive.WriteInterval(m_t);
    if (!rc) break;
    rc = binary_archive.WriteVector(m_up);
    if (!rc) break;
    rc = binary_archive.WriteBool(m_bHaveN[0]);
    if (!rc) break;
    rc = binary_archive.WriteBool(m_bHaveN[1]);
    if (!rc) break;
    rc = binary_archive.WriteVector(m_N[0]);
    if (!rc) break;
    rc = binary_archive.WriteVector(m_N[1]);
    if (!rc) break;
    rc = binary_archive.WriteInterval(m_path_domain);
    if (!rc) break;
    rc = binary_archive.WriteBool(m_bTransposed);
    if (!rc) break;
    // 1.1 
    rc = binary_archive.WriteInt(m_profile_count);
    if (!rc) break;
    // 1.2
    rc = binary_archive.WriteBool(m_bCap[0]);
    if (!rc) break;
    rc = binary_archive.WriteBool(m_bCap[1]);
    if (!rc) break;

    break;
  }
  if ( !binary_archive.EndWrite3dmChunk() )
    rc = false;
  return rc;
}


ON_BOOL32 ON_Extrusion::Read(
       ON_BinaryArchive& binary_archive
     )
{
  Destroy();
  int major_version = 0;
  int minor_version = 0;
  bool rc = binary_archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if (!rc)
    return false;
  for (;;)
  {
    rc = (1 == major_version);
    if (!rc) break;
    ON_Object* obj = 0;
    rc = (1==binary_archive.ReadObject(&obj));
    if (!rc) break;
    if ( obj )
    {
      m_profile = ON_Curve::Cast(obj);
      if ( !m_profile )
      {
        delete obj;
        rc = false;
        break;
      }
    }
    rc = binary_archive.ReadLine(m_path);
    if (!rc) break;
    rc = binary_archive.ReadInterval(m_t);
    if (!rc) break;
    rc = binary_archive.ReadVector(m_up);
    if (!rc) break;
    rc = binary_archive.ReadBool(&m_bHaveN[0]);
    if (!rc) break;
    rc = binary_archive.ReadBool(&m_bHaveN[1]);
    if (!rc) break;
    rc = binary_archive.ReadVector(m_N[0]);
    if (!rc) break;
    rc = binary_archive.ReadVector(m_N[1]);
    if (!rc) break;
    rc = binary_archive.ReadInterval(m_path_domain);
    if (!rc) break;
    rc = binary_archive.ReadBool(&m_bTransposed);
    if (!rc) break;

    // set profile count if extrusion was read from an old file.
    m_profile_count = (0 != m_profile) ? 1 : 0;

    if ( minor_version >= 1 )
    {
      // 1.1 
      rc = binary_archive.ReadInt(&m_profile_count);
      if (!rc) break;

      if ( minor_version >= 2 )
      {
        // 1.2
        rc = binary_archive.ReadBool(&m_bCap[0]);
        if (!rc) break;
        rc = binary_archive.ReadBool(&m_bCap[1]);
        if (!rc) break;
      }
    }

    if ( minor_version < 2 )
    {
      const ON_Curve* outer_profile = Profile(0);
      if ( 0 != outer_profile && outer_profile->IsClosed() )
      {
        m_bCap[0] = m_bCap[1] = true;
      }
    }

    break;
  }
  if ( !binary_archive.EndRead3dmChunk() )
    rc = false;
  return rc;
}

ON::object_type ON_Extrusion::ObjectType() const
{
  return ON::extrusion_object;
}



int ON_Extrusion::Dimension() const
{
  return 3;
}

static 
bool GetBoundingBoxHelper(const ON_Extrusion& extrusion, 
                          ON_BoundingBox& bbox,
                          const ON_Xform* xform
                          )
{
  // input bbox = profile curve bounding box.
  ON_3dPoint corners[8];
  bbox.m_min.z = 0.0;
  bbox.m_max.z = 0.0;
  corners[0] = corners[1] = bbox.m_min;
  corners[1].x = bbox.m_max.x;
  corners[2] = corners[3] = bbox.m_max;
  corners[3].x = bbox.m_min.x;
  corners[4] = corners[0];
  corners[5] = corners[1];
  corners[6] = corners[2];
  corners[7] = corners[3];

  ON_Xform xform0;
  if ( !extrusion.GetProfileTransformation(0,xform0) )
    return false;
  ON_Xform xform1;
  if ( !extrusion.GetProfileTransformation(1,xform1) )
    return false;
  if ( xform && !xform->IsIdentity() )
  {
    xform0 = (*xform)*xform0;
    xform1 = (*xform)*xform1;
  }
  corners[0] = xform0*corners[0];
  corners[1] = xform0*corners[1];
  corners[2] = xform0*corners[2];
  corners[3] = xform0*corners[3];
  corners[4] = xform1*corners[4];
  corners[5] = xform1*corners[5];
  corners[6] = xform1*corners[6];
  corners[7] = xform1*corners[7];
  bbox.Set(3,0,8,3,&corners[0].x,false);
  return true;
}

ON_BOOL32 ON_Extrusion::GetBBox(double* boxmin,double* boxmax,int bGrowBox) const
{
  bool rc = false;
  if ( m_path.IsValid() && m_profile )
  {
    ON_BoundingBox bbox;
    if ( m_profile->GetTightBoundingBox(bbox) && GetBoundingBoxHelper(*this,bbox,0) )
    {
      rc = true;
      if ( bGrowBox )
      {
        bGrowBox =  ( boxmin[0] <= boxmax[0] 
                      && boxmin[1] <= boxmax[1] 
                      && boxmin[2] <= boxmax[2]
                      && ON_IsValid(boxmax[0]) 
                      && ON_IsValid(boxmax[1]) 
                      && ON_IsValid(boxmax[2]));
      }
      if ( bGrowBox )
      {
        if ( boxmin[0] > bbox.m_min.x ) boxmin[0] = bbox.m_min.x;
        if ( boxmin[1] > bbox.m_min.y ) boxmin[1] = bbox.m_min.y;
        if ( boxmin[2] > bbox.m_min.z ) boxmin[2] = bbox.m_min.z;
        if ( boxmax[0] < bbox.m_max.x ) boxmax[0] = bbox.m_max.x;
        if ( boxmax[1] < bbox.m_max.y ) boxmax[1] = bbox.m_max.y;
        if ( boxmax[2] < bbox.m_max.z ) boxmax[2] = bbox.m_max.z;
      }
      else
      {
        boxmin[0] = bbox.m_min.x;
        boxmin[1] = bbox.m_min.y;
        boxmin[2] = bbox.m_min.z;
        boxmax[0] = bbox.m_max.x;
        boxmax[1] = bbox.m_max.y;
        boxmax[2] = bbox.m_max.z;
      }
    }
  }
  return rc;
}


bool ON_Extrusion::GetTightBoundingBox(ON_BoundingBox& tight_bbox, int bGrowBox, const ON_Xform* xform ) const
{
  bool rc = false;
  if ( m_path.IsValid() && m_profile )
  {
    ON_BoundingBox bbox;
    if ( m_profile->GetTightBoundingBox(bbox) && GetBoundingBoxHelper(*this,bbox,xform) )
    {
      if ( bGrowBox )
        tight_bbox.Union(bbox);
      else
        tight_bbox = bbox;
      rc = true;
    }
  }
  return rc;
}

static bool ON_Extrusion_TransformFailed()
{
  return false; // good place for a breakpoint
}

static bool Profile2dTransform( ON_Extrusion& e, const ON_Xform& profile_xform, bool bNeedReverse )
{
  if ( profile_xform.IsIdentity() )
  {
    // profile curves are unchanged (translation, rotation, 1D scale along path, ... )
    return true; 
  }

  // transform 2d profiles
  bool rc = false;
  bool bNeedDeformable = (    fabs(profile_xform.m_xform[0][0]) != fabs(profile_xform.m_xform[1][1]) 
                           || 0.0 != profile_xform.m_xform[1][0] 
                         );
  ON_PolyCurve* polyprofile = const_cast<ON_PolyCurve*>(e.PolyProfile());
  if ( 0 != polyprofile )
  {
    rc = true;
    if ( bNeedDeformable )
      polyprofile->MakeDeformable();
    for ( int i = 0; i < polyprofile->Count(); i++ )
    {
      ON_Curve* profile_segment = polyprofile->SegmentCurve(i);
      if ( 0 == profile_segment )
      {
        continue;
      }
      if ( !profile_segment->Transform(profile_xform) )
      {
        rc = ON_Extrusion_TransformFailed();
        continue;
      }
      if ( bNeedReverse )
      {
        double t0, t1;
        if ( profile_segment->GetDomain(&t0,&t1) )
        {
          // reverse the segment so the correct clounterclockwise/clockwise
          // orientation is maintained after reflection across the y axis.
          profile_segment->Reverse();
          // preserve the evaluation domain of each segment
          profile_segment->SetDomain(t0,t1);
        }
      }
    }
  }
  else
  {
    if ( bNeedDeformable && !e.m_profile->IsDeformable() )
    {
      ON_NurbsCurve* c = e.m_profile->NurbsCurve();
      if ( 0 != c )
      {
        c->CopyUserData(*e.m_profile);
        if( c->Transform(profile_xform) )
        {
          rc = true;
          delete e.m_profile;
          e.m_profile = c;
        }
        else
        {
          delete c;
          rc = ON_Extrusion_TransformFailed();
        }
      }
      else
      {
        rc = ON_Extrusion_TransformFailed();
      }
    }
    else
    {
      rc = e.m_profile->Transform(profile_xform)?true:ON_Extrusion_TransformFailed();
    }
    if ( rc && bNeedReverse )
    {
      double t0, t1;
      if ( e.m_profile->GetDomain(&t0,&t1) )
      {
        // reverse the segment so the correct clounterclockwise/clockwise
        // orientation is maintained after reflection across the y axis.
        e.m_profile->Reverse();
        // preserve the evaluation domain of each segment
        e.m_profile->SetDomain(t0,t1);
      }
    }
  }

  return rc;
}

ON_BOOL32 ON_Extrusion::Transform( const ON_Xform& xform )
{
  if ( !m_path.IsValid() || !m_up.IsUnitVector() || m_profile_count < 1 )
    return ON_Extrusion_TransformFailed();

  ON_3dVector T = m_path.Tangent();
  ON_3dVector UxT = ON_CrossProduct(m_up,T);
  if ( !UxT.IsUnitVector() && !UxT.Unitize() )
    return ON_Extrusion_TransformFailed();

  const bool bUseVectorXform = (0.0 == xform[3][0] && 0.0 == xform[3][1] && 0.0 == xform[3][2]);

  ON_3dPoint E[2], QE[2];
  E[0] = m_path.from;
  E[1] = m_path.to;
  QE[0] = xform*E[0];
  QE[1] = xform*E[1];
  if ( !QE[0].IsValid() )
    return ON_Extrusion_TransformFailed();
  if ( !QE[1].IsValid() )
    return ON_Extrusion_TransformFailed();
  ON_3dVector QT0 = bUseVectorXform ? (xform*(E[1] - E[0])) : (QE[1]-QE[0]);
  if ( !QT0.Unitize() )
    return ON_Extrusion_TransformFailed();
  
  const int base_index = ( QE[1].DistanceTo(E[1]) < QE[0].DistanceTo(E[0]) ) ? 1 : 0;
  const ON_3dPoint B(E[base_index]);
  const ON_3dPoint QB(QE[base_index]);

  const double QT0oT = QT0*T;
  bool bSamePathDir = ( fabs(fabs(QT0oT) - 1.0) <= ON_SQRT_EPSILON );
  ON_3dVector QT = bSamePathDir ? ((QT0oT < 0.0) ? -T : T) : QT0;
  const double Qlen = QE[0].DistanceTo(QE[1]);
  if ( bSamePathDir )
  {
    ON_3dPoint R =  QB + (base_index ? -Qlen : Qlen)*QT;
    if ( QE[1-base_index].DistanceTo( R ) <= ON_SQRT_EPSILON*Qlen )
    {
      QE[1-base_index] = R;
    }
    else
    {
      bSamePathDir = false;
      QT = QT0;
    }
  }

  const ON_3dPoint X0 = xform*(B + UxT);
  const ON_3dPoint Y0 = xform*(B + m_up);
  const ON_3dVector QDY = bUseVectorXform ? (xform*m_up) : (Y0-QB);
  ON_3dVector QY = QDY;
  if ( !QY.Unitize() )
    return ON_Extrusion_TransformFailed();
  ON_3dVector QU0 = QDY - (QDY*QT)*QT;
  if ( !QU0.Unitize() )
    return ON_Extrusion_TransformFailed();

  const double QU0oU = QU0*m_up;
  bool bSameUpDir = ( fabs(fabs(QU0oU) - 1.0) <= ON_SQRT_EPSILON );
  ON_3dVector QU = bSameUpDir ? ((QU0oU < 0.0) ? -m_up : m_up) : QU0;

  if (bSameUpDir && !bSamePathDir && fabs(QU*QT) > fabs(QU0*QT) )
  {
    // 12 July 2008 Dale Lear - this test fixes http://dev.mcneel.com/bugtrack/?q=88130
    bSameUpDir = false;
    QU = QU0;
  }

  ON_3dVector QUxQT = ON_CrossProduct(QU,QT);
  if ( !QUxQT.Unitize() )
    return ON_Extrusion_TransformFailed();

  const double QUxQToUxT = QUxQT*UxT;
  if ( (bSamePathDir && bSameUpDir) || fabs(fabs(QUxQToUxT) - 1.0) <= ON_SQRT_EPSILON )
  {
    if ( QUxQToUxT < 0.0 )
      QUxQT = -UxT;
    else
      QUxQT = UxT;
  }

  const double QUoQY = QU*QY;
  if ( fabs(QUoQY - 1.0) < ON_SQRT_EPSILON )
    QY = QU;
  else if ( fabs(QUoQY + 1.0) < ON_SQRT_EPSILON )
    QY = -QU;

  // profile_xform will be the transformation 
  // applied to the 2d profile curve.
  const ON_3dVector QDX = bUseVectorXform ? (xform*UxT) : (X0 - QB);
  ON_3dVector QX = QDX - (QDX*QY)*QY;
  if ( !QX.Unitize() )
    return ON_Extrusion_TransformFailed();

  const double QUxQToQX = QUxQT*QX;
  if ( fabs(QUxQToQX - 1.0) < ON_SQRT_EPSILON )
    QX = QUxQT;
  else if ( fabs(QUxQToQX + 1.0) < ON_SQRT_EPSILON )
    QX = -QUxQT;


  ON_3dVector QXxQY = ON_CrossProduct(QX,QY);
  if ( !QXxQY.IsUnitVector() && !QXxQY.Unitize() )
    return ON_Extrusion_TransformFailed();
  if ( QXxQY*QT < 0.0 )
  {
    QX.Reverse();
    QXxQY.Reverse();
  }

  ON_3dVector path_shear(0.0,QU*QXxQY,QT*QXxQY);
  bool bHavePathShear =    path_shear.z > ON_Extrusion::m_Nz_min 
                        && path_shear.z < 1.0 - ON_SQRT_EPSILON
                        && path_shear.y < 1.0 - ON_SQRT_EPSILON;
  if ( bHavePathShear )
  {
    double x2 = path_shear.y*path_shear.y + path_shear.z*path_shear.z;
    if ( x2 > ON_SQRT_EPSILON && x2 <= 1.0 )
    {
      double QUxQToQXxQY = QUxQT*QXxQY;
      path_shear.x = sqrt(1.0 - x2);
      if ( QUxQToQXxQY < 0.0 )
        path_shear.x = -path_shear.x;
    }
    if ( fabs(path_shear.y) <= ON_SQRT_EPSILON )
      path_shear.y = 0.0;
    bHavePathShear = path_shear.IsUnitVector() && (path_shear.x != 0.0 || path_shear.y != 0.0);
  }

  if ( !bHavePathShear )
  {
    path_shear.Set(0.0,0.0,1.0);
    QXxQY = QT;
  }

  // miter normals
  ON_3dVector QN[2] = {ON_3dVector::ZeroVector,ON_3dVector::ZeroVector};
  bool bHaveQN[2] = {false,false};
  bool bUseQN[2] = {false,false};
  for ( int i = 0; i < 2; i++ )
  {
    if ( m_bHaveN[i] )
    {
      QN[i] = m_N[i];
      bHaveQN[i] = true;
      bUseQN[i] = true;

      // In order for this to work with  transformations that
      // have bHavePathShear = true, we have to see where the
      // miter *plane* is mapped which, for a shear 
      // transformation, cannot be done by transforming 
      // m_N[i] = normal to the miter plane.
      ON_3dVector QN3d;
      if ( bHavePathShear )
      {
        // calculate a world 3d basis for the miter plane
        ON_3dVector V1(m_N[i].y,-m_N[i].x,0.0);
        V1.Unitize();
        ON_3dVector V2 = ON_CrossProduct( m_N[i],V1);
        V1 = V1.x*UxT + V1.y*m_up + V1.z*T;
        V2 = V2.x*UxT + V2.y*m_up + V2.z*T;

        // transform the basis to get a world 3d basis
        // for the transformed miter plane.
        ON_3dVector QV1 = xform*V1;
        ON_3dVector QV2 = xform*V2;
        double lenQV1 = QV1.Length();
        double lenQV2 = QV2.Length();
        if ( fabs(lenQV1 - lenQV2) > 0.125*(lenQV1 + lenQV2) )
        {
          // if lengths are "different", then normalize
          // before taking the cross product. Otherwise,
          // skip normalization to get slightly
          // more precision in a calculation that is
          // already pretty fuzzy.
          QV1.Unitize();
          QV2.Unitize();
        }

        // now get a world 3d normal to the miter plane
        QN3d = ON_CrossProduct(QV1,QV2);
      }
      else
      {
        const ON_3dVector N3d = m_N[i].x*UxT + m_N[i].y*m_up + m_N[i].z*T;
        ON_3dPoint QTip = xform*(E[i] + N3d);
        QN3d = bUseVectorXform ? (xform*N3d) : (QTip - QE[i]);
      }
      if ( !QN3d.Unitize() )
        continue;

      // QN[i] = miter vector
      QN[i].x = QUxQT*QN3d;
      QN[i].y = QU*QN3d;
      QN[i].z = QT*QN3d;
      if ( QN[i].z < 0.0 )
        QN[i].Reverse(); // necessary for some mirror transformations
      if ( !QN[i].Unitize() )
      {
         bHaveQN[i] = false;
      }
      if ( fabs(QN[i].x) <= ON_SQRT_EPSILON )
        QN[i].x = 0.0;
      if ( fabs(QN[i].y) <= ON_SQRT_EPSILON )
        QN[i].y = 0.0;
      if (    QN[i].z < ON_Extrusion::m_Nz_min
           || QN[i].z >= 1.0 - ON_SQRT_EPSILON
           || (fabs(QN[i].x) <= ON_SQRT_EPSILON && fabs(QN[i].y) <= ON_SQRT_EPSILON )
         )
      {
        bHaveQN[i] = false;
      }

      if ( !bHaveQN[i] )
        QN[i] = ON_3dVector::ZeroVector;
      else if ( fabs(QN[i]*m_N[i] - 1.0) <= 1.0e-6 ) // ON_SQRT_EPSILON 
        QN[i] = m_N[i];
    }
  }

  // get 2d profile transformation
  ON_Xform profile_xform(1.0);
  // set 2d profile y scale
  const double profile_y_scale = QDY.Length(); // = QYoQDY "mathematically"

  if ( !bHavePathShear )
  {
    const double profile_x_scale = QX*QDX;

    if ( !ON_IsValid(profile_y_scale) || 0.0 == profile_y_scale )
    {
      // cannot flatten profile
      return ON_Extrusion_TransformFailed(); 
    } 

    if ( !ON_IsValid(profile_x_scale) || 0.0 == profile_x_scale )
    {
      // cannot flatten profile
      return ON_Extrusion_TransformFailed(); 
    } 

    // NOTE: 
    //   profile_xform.m_xform[1][1] must always be > 0.0 and
    //   is the most precise number number in the matrix.
    const double profile_y_scale_tol = ON_SQRT_EPSILON;
    const double profile_x_scale_tol = 10.0*profile_y_scale_tol;

    if ( fabs(profile_y_scale - 1.0) <= profile_y_scale_tol )
      profile_xform.m_xform[1][1] = 1.0;
    else
      profile_xform.m_xform[1][1] = profile_y_scale;

    if ( fabs( profile_x_scale - 1.0 ) <= profile_x_scale_tol )
      profile_xform.m_xform[0][0] = 1.0;
    else if ( fabs( profile_x_scale + 1.0 ) <= profile_x_scale_tol )
      profile_xform.m_xform[0][0] = -1.0;
    else
      profile_xform.m_xform[0][0] = profile_x_scale;

    const double profile_xform_tol = profile_x_scale_tol*(fabs(profile_xform.m_xform[0][0]) + profile_xform.m_xform[1][1]);
    if ( fabs(fabs(profile_xform.m_xform[0][0]) - profile_xform.m_xform[1][1]) <= profile_xform_tol )
    {
      profile_xform.m_xform[0][0] = ((profile_xform.m_xform[0][0] < 0.0)?-1.0:1.0)*profile_xform.m_xform[1][1];
    }
    else if ( fabs(profile_xform.m_xform[0][0]) <= profile_xform_tol )
    {
      // cannot flatten profile
      return ON_Extrusion_TransformFailed(); 
    }

    double profile_det = profile_xform.m_xform[0][0]*profile_xform.m_xform[1][1];
    if ( !ON_IsValid(profile_det) || 0.0 == profile_det )
      return ON_Extrusion_TransformFailed();

    // set 2d profile shear
    const double profile_shear_tol = profile_xform_tol;
    const double QYoQDX = QY*QDX;
    if ( fabs( QYoQDX ) <= profile_shear_tol )
      profile_xform.m_xform[1][0] = 0.0;
    else
      profile_xform.m_xform[1][0] = QYoQDX;
  }
  else
  {
    // plane0 is world 3d plane that provides the coordinate system
    // for mapping the original 2d profile into world 3d.
    ON_Plane plane0;
    plane0.origin = B; plane0.xaxis = UxT; plane0.yaxis = m_up;   plane0.zaxis = T;
    plane0.UpdateEquation();

    // plane1 is world 3d plane that provides the coordinate system
    // for mapping the transformed 2d profile into world 3d.
    ON_Plane plane1;
    plane1.origin = QB; plane1.xaxis = QUxQT; plane1.yaxis = QU;   plane1.zaxis = QT;
    plane1.UpdateEquation();

    ON_Xform xform1, xform3, xform4, xform5;

    // xform 1 maps original 2d profile to world 3d
    xform1.Rotation(ON_Plane::World_xy,plane0);

    // xform maps plane1 to 

    // xform 3 maps the transformed 3d profile to the world 3d plane
    // that is orthoganal to the transformed axis vector.
    xform3.PlanarProjection(plane1);

    // xform4 maps the transformed world 3d profile back to the xy plane
    xform4.Rotation(plane1, ON_Plane::World_xy);

    profile_xform = xform4*xform3*xform*xform1;
    profile_xform.m_xform[0][2] = 0.0;
    profile_xform.m_xform[1][2] = 0.0;
    profile_xform.m_xform[2][0] = 0.0; profile_xform.m_xform[2][1] = 0.0; profile_xform.m_xform[2][2] = 1.0; profile_xform.m_xform[2][3] = 0.0;
    profile_xform.m_xform[3][0] = 0.0; profile_xform.m_xform[3][1] = 0.0; profile_xform.m_xform[3][2] = 0.0; profile_xform.m_xform[3][3] = 1.0;

    const double xform_tol = 10.0*ON_SQRT_EPSILON;
    if ( profile_y_scale > xform_tol && fabs(profile_y_scale - 1.0) > xform_tol )
    {
      double profile_scale_tol = profile_y_scale*ON_SQRT_EPSILON;
      if ( fabs(profile_xform.m_xform[0][0] - profile_y_scale) < profile_scale_tol )
      {
        profile_xform.m_xform[0][0] = profile_y_scale;
        if ( fabs(profile_xform.m_xform[1][1] - profile_y_scale) < profile_scale_tol )
          profile_xform.m_xform[1][1] = profile_y_scale;
        else if ( fabs(profile_xform.m_xform[1][1] + profile_y_scale) < profile_scale_tol )
          profile_xform.m_xform[1][1] = -profile_y_scale;
      }
    }

    for ( int i = 0; i < 2; i++ ) for ( int j = 0; j < 4; j++ )
    {
      if ( 2 == j )
        continue;
      double x = fabs(profile_xform.m_xform[i][j]);
      if ( fabs(x) <= xform_tol )
        profile_xform.m_xform[i][j] = 0.0;
      else if ( fabs(1.0-x) <= xform_tol )
        profile_xform.m_xform[i][j] = 1.0;
      else if ( fabs(x-1.0) <= xform_tol )
        profile_xform.m_xform[i][j] = -1.0;
    }
  }

  // start transforming informtion here
  TransformUserData(xform);

  // transform path, up, and miter directions
  m_path.from = QE[0];
  m_path.to = QE[1];
  m_up = QU;
  for ( int i = 0; i < 2; i++ )
  {
    if ( bUseQN[i] )
    {
      m_N[i] = QN[i];
      m_bHaveN[i] = bHaveQN[i];
    }
    else if ( bHavePathShear )
    {
      m_N[i] = path_shear;
      m_bHaveN[i] = true;
    }
    else
    {
      m_N[i].Set(0.0,0.0,0.0);
      m_bHaveN[i] = false;
    }
  }

  //if ( m_bHaveN[0] && QN[0].IsValid() && QN[0].IsUnitVector() )
  //  m_N[0] = QN[0];
  //if ( m_bHaveN[1] && QN[1].IsValid() && QN[1].IsUnitVector() )
  //  m_N[1] = QN[1];
  //if ( bHavePathShear )
  //{
  //  // TODO: 
  //  //  1) combine with eisting mitering
  //  //  2) fix bug when eigenvector of the shear transform
  //  //     is not parallel to a profile plane axis.
  //  m_bHaveN[0] = true;
  //  m_N[0] = path_shear;
  //  m_bHaveN[1] = true;
  //  m_N[1] = path_shear;
  //}

  if ( profile_xform.IsIdentity() )
    return true; // should happen on all rotations and translations.
  
  double profile_det = profile_xform[0][0]*profile_xform[1][1] - profile_xform[1][0]*profile_xform[0][1];
  bool bNeedReverse = (profile_det < 0.0);
  return  Profile2dTransform( *this, profile_xform, bNeedReverse );
}

class CMyBrepIsSolidSetter : public ON_Brep
{
public:
  void SetIsSolid(int is_solid) {m_is_solid = is_solid;}
  void SetBBox( const ON_Extrusion& extrusion)
  {
    ON_BoundingBox brep_bbox = BoundingBox();
    ON_BoundingBox extr_bbox = extrusion.BoundingBox();
    ON_BoundingBox bbox;
    bbox.Intersection(brep_bbox,extr_bbox);
    m_bbox = bbox;
  }
};

class ON_Extrusion_BrepForm_FaceInfo
{
public:
  ON_Extrusion_BrepForm_FaceInfo();

  ~ON_Extrusion_BrepForm_FaceInfo();

  /*
  Returns
    false if m_face_index < 0.  This happens when the 3d profile
    for the extrusion is too short or bogus or the nurbs surface
    generated from this profile is bogus.  One way this happens
    is when the transformation from 2d to 3d has an enormous
    translation component.
  */
  bool HaveBrepFaceFace() const;

  void Init();

  // Information about the entire profile
  //   If this profile has kinks and we are splitting up
  //   kinky faces, then the ON_Extrusion_BrepForm_FaceInfo
  //   will be face made by extruding a smooth sub-curve
  //   of the entire profile.
  bool m_bClosedProfile;
  int m_profile_orientation;
  int m_profile_index;

  // Information for this wall face
  ON_Curve* m_extrusion_profile;
  ON_Extrusion* m_extrusion_srf;
  int m_face_index;
  int m_vid[4];
  int m_eid[4];
  ON_BOOL32 m_bRev3d[4];

  // capping information (bottom,top)
  int m_cap_trim_index[2]; // indices of wall trims
  int m_cap_edge_index[2];
  ON_NurbsCurve* m_cap_c2[2]; // 2d cap trim curves
};

ON_Extrusion_BrepForm_FaceInfo::ON_Extrusion_BrepForm_FaceInfo()
{
  Init();
}

ON_Extrusion_BrepForm_FaceInfo::~ON_Extrusion_BrepForm_FaceInfo()
{
  // when m_extrusion_srf is not null, its destructor deletes m_extrusion_profile.
  if ( 0 != m_extrusion_srf )
  {
    // When m_extrusion_srf is not null, it 
    // manages the m_extrusion_profile curve
    m_extrusion_profile = 0;
    delete m_extrusion_srf;
    m_extrusion_srf = 0;
  }
  
  if ( 0 != m_extrusion_profile )
  {
    delete m_extrusion_profile;
    m_extrusion_profile = 0;
  }

  if ( m_cap_c2[0] )
  {
    delete m_cap_c2[0];
    m_cap_c2[0] = 0;
  }
  
  if ( m_cap_c2[1] )
  {
    delete m_cap_c2[1];
    m_cap_c2[1] = 0;
  }

  memset(this,0,sizeof(*this));
}

bool ON_Extrusion_BrepForm_FaceInfo::HaveBrepFaceFace() const
{
  if ( m_face_index < 0 )
    return false;
  return true;
}

void ON_Extrusion_BrepForm_FaceInfo::Init()
{
  memset(this,0,sizeof(*this));
  
  m_bClosedProfile = false;
  m_profile_orientation = 0;
  m_profile_index = -1;

  m_extrusion_profile = 0;
  m_extrusion_srf = 0;
  m_face_index = -1;
  m_vid[0] = m_vid[1] = m_vid[2] = m_vid[3] = -1;
  m_eid[0] = m_eid[1] = m_eid[2] = m_eid[3] = -1;
  // Set the m_bRev3d[] flags so the 3d edges orientation
  // matches the surface iso-curve orientation.
  m_bRev3d[0] = m_bRev3d[1] = 0;
  m_bRev3d[2] = m_bRev3d[3] = 1;

  m_cap_trim_index[0] = m_cap_trim_index[1] = -1;
  m_cap_edge_index[0] = m_cap_edge_index[1] = -1;
  m_cap_c2[0] = m_cap_c2[1] = 0;
}

ON_Brep* ON_Extrusion::BrepForm( ON_Brep* brep ) const
{
  return BrepForm(brep,true);
}

static 
ON_PlaneSurface* MakeCapPlaneHelper(
    ON_ClassArray< ON_Extrusion_BrepForm_FaceInfo >& finfo,
    int cap_index, // 0 = bottom, 1 = top
    const ON_Xform& rot 
    )
{
  double d;
  ON_BoundingBox bbox;

  // get bounding box of trimming curves
  int outer_loop_trim_count = 0;
  int outer_loop_iso_trim_count = 0;
  for ( int i = 0; i < finfo.Count(); i++ )
  {
    if ( false == finfo[i].HaveBrepFaceFace() )
      continue; // profile curve segment was too short or bogus

    if ( 0 != finfo[i].m_profile_index )
    {
      // this and the rest of the finfo[] elements attach
      // attach to an inner boundary on the cap.
      break;
    }
    const ON_NurbsCurve* c2 = finfo[i].m_cap_c2[cap_index];
    if ( 0 == c2 )
      return 0;
    ON_BoundingBox c2bbox;
    c2->GetTightBoundingBox( c2bbox, false );
    if ( 0 == i )
      bbox = c2bbox;
    else
      bbox.Union(c2bbox);
    if ( outer_loop_iso_trim_count ==  outer_loop_trim_count )
    {
      // check for iso trims as long as all previous trims are iso trims.
      ON_3dVector D = c2bbox.Diagonal();
      double zero_tol = ON_ZERO_TOLERANCE;
      double nonzero_tol = 1000.0*ON_ZERO_TOLERANCE;
      if (     (D.x <= zero_tol && D.y > nonzero_tol)
            || (D.y <= zero_tol && D.x > nonzero_tol)
         )
      {
        outer_loop_iso_trim_count++;
      }
    }
    outer_loop_trim_count++;
  }

  if ( outer_loop_trim_count <= 0 )
  {
    // must have an outer boundary.
    return 0; 
  }

  ON_Interval u(bbox.m_min.x,bbox.m_max.x);
  ON_Interval v(bbox.m_min.y,bbox.m_max.y);
  
  if (    outer_loop_iso_trim_count < outer_loop_trim_count 
       || !u.IsIncreasing()
       || !v.IsIncreasing()
     )
  {
    // grow cap plane to extend a little beyond the trims
    d = u.Length();
    if ( 0.0 == d )
      d = 1.0; // happens when profile is a line
    d *= 0.125; u.m_t[0] -= d; u.m_t[1] += d;
    
    d = v.Length();
    if ( 0.0 == d )
      d = 1.0; // happens when profile is a line
    d *= 0.125; v.m_t[0] -= d; v.m_t[1] += d;
  }

  if ( !u.IsIncreasing() || !v.IsIncreasing() )
    return 0;

  ON_PlaneSurface* plane = new ON_PlaneSurface(ON_xy_plane);
  plane->SetExtents(0,u,true);
  plane->SetExtents(1,v,true);
  if ( !rot.IsIdentity() )
    plane->Transform(rot);

  return plane;
}

static 
int MakeCapLoopHelper(
          ON_ClassArray< ON_Extrusion_BrepForm_FaceInfo >& finfo,
          int fi0,
          int cap_index, // 0 = bottom, 1 = top
          ON_BrepFace* capface, 
          ON_BrepLoop::TYPE loop_type,
          bool* bTrimsWereModified
          )
{
  ON_BrepEdge* edge;

  if ( 0 == capface )
    return fi0; // happens when extrusion has an open end

  ON_Brep* brep = capface->Brep();
  if ( 0 == brep )
    return fi0;

  if ( fi0 < 0 || fi0 >= finfo.Count() )
    return fi0;

  ON_BrepLoop& loop = brep->NewLoop(loop_type,*capface);

  bool bRev3d = false;
  bool bCloseGaps = true;

  int fi1;
  for ( fi1 = fi0; fi1 < finfo.Count(); fi1++ )
  {
    if ( false == finfo[fi1].HaveBrepFaceFace() )
      continue; // profile segment for this face was too short or bogus

    if ( finfo[fi0].m_profile_index != finfo[fi1].m_profile_index )
      break;

    ON_NurbsCurve* c2 = finfo[fi1].m_cap_c2[cap_index];
    if ( 0 == c2 )
    {
      bCloseGaps = false;
      break;
    }
    int c2i = brep->AddTrimCurve(c2);
    finfo[fi1].m_cap_c2[cap_index] = 0;
    edge = brep->Edge(finfo[fi1].m_cap_edge_index[cap_index]);
    if ( 0 == edge )
    {
      bCloseGaps = false;
      break;
    }
    ON_BrepTrim& trim = brep->NewTrim(*edge, bRev3d, loop, c2i);
    trim.m_tolerance[0] = trim.m_tolerance[1] = 0.0;
  }
  brep->SetTrimIsoFlags( loop );

  // 6 June 2012 Dale Lear
  //    Fix bug 105311
  //    Sometimes there are gaps in ON_PolyCurve profiles that
  //    need to be closed to make a valid ON_Brep.  The gaps
  //    need to be closed after the call to SetTrimIsoFlags().
  if ( bCloseGaps ) for ( int lti = 0; lti < loop.m_ti.Count(); lti++ )
  {
    ON_BrepTrim& trim0 = brep->m_T[loop.m_ti[lti]];
    ON_BrepTrim& trim1 = brep->m_T[loop.m_ti[(lti+1)%loop.m_ti.Count()]];
    if ( trim0.PointAtEnd() != trim1.PointAtStart() )
    {
      if ( brep->CloseTrimGap(trim0,trim1) )
      {
        edge = trim0.Edge();
        if ( edge )
          edge->m_tolerance = ON_UNSET_VALUE;
        edge = trim1.Edge();
        if ( edge )
          edge->m_tolerance = ON_UNSET_VALUE;
        if ( 0 != bTrimsWereModified )
          *bTrimsWereModified = true;
      }
    }
  }

  return fi1;
}

static
bool MakeCap2dCurvesHelper(
        ON_ClassArray< ON_Extrusion_BrepForm_FaceInfo >& finfo,
        int is_capped,
        const ON_Xform& scale0,
        const ON_Xform& scale1
        )
{
  switch(is_capped)
  {
  case 1:
  case 2:
  case 3:
    break;
  default:
    return false;
  }

  for ( int i = 0; i < finfo.Count(); i++ )
  {
    ON_NurbsCurve* c20 = 0;
    ON_NurbsCurve* c21 = 0;

    if ( false == finfo[i].HaveBrepFaceFace() )
      continue; // profile curve was too short or bogus

    c20 = finfo[i].m_extrusion_srf->m_profile->NurbsCurve();
    if ( 0 == c20 )
      return false;

    c20->ChangeDimension(2);
    c20->MakePiecewiseBezier(true);
    if ( 2 == is_capped )
    {
      c21 = c20;
      c20 = 0;
    }
    else if ( 3 == is_capped )
    {
      c21 = c20->Duplicate();
    }

    if ( 0 != c20 && !scale0.IsIdentity() )
      c20->Transform(scale0);
    if ( 0 != c21 && !scale1.IsIdentity() )
      c21->Transform(scale1);

    finfo[i].m_cap_c2[0] = c20;
    finfo[i].m_cap_c2[1] = c21;
  }

  return true;
}


static bool GetNextProfileSegmentDiscontinuity(
                const ON_Curve* profile_segment,
                double t0,
                double t1,
                double *t
                )
{
  // Do NOT change ON_DEFAULT_ANGLE_TOLERANCE_COSINE to another value.
  // See comments in CRhinoDoc::AddObject() for details.
  if ( 0 == profile_segment )
    return false;
  return profile_segment->GetNextDiscontinuity(
                            ON::Gsmooth_continuous,
                            t0,t1,t,
                            0,0,
                            ON_DEFAULT_ANGLE_TOLERANCE_COSINE,
                            ON_SQRT_EPSILON
                            );
}


bool ON_Extrusion::ProfileIsKinked( int profile_index ) const
{
  const ON_Curve* profile = Profile(profile_index);
  if ( 0 == profile )
    return false;
  double t0 = ON_UNSET_VALUE;
  double t1 = ON_UNSET_VALUE;
  double t;
  if ( !profile->GetDomain(&t0,&t1) )
    return 0;
  if ( !ON_IsValid(t0) || !(t0 < t1) )
    return 0;
  t  = t0;
  return (GetNextProfileSegmentDiscontinuity( profile, t0,t1, &t) && t0 < t && t < t1);
}

int ON_Extrusion::ProfileSmoothSegmentCount( int profile_index ) const
{
  if ( 0 == Profile(profile_index) )
    return 0;
  ON_SimpleArray<double> * k  = 0;
  return (1 + GetProfileKinkParameters(profile_index,*k));
}

int ON_Extrusion::GetProfileKinkParameters( int profile_index, ON_SimpleArray<double>& profile_kink_parameters ) const
{
  const ON_Curve* profile2d = Profile(profile_index);
  if ( 0 == profile2d )
    return 0;
  double t0 = ON_UNSET_VALUE;
  double t1 = ON_UNSET_VALUE;
  double t;
  if ( !profile2d->GetDomain(&t0,&t1) )
    return 0;
  if ( !ON_IsValid(t0) || !(t0 < t1) )
    return 0;
  ON_SimpleArray<double> * k = &profile_kink_parameters;
  int count = 0;
  while ( GetNextProfileSegmentDiscontinuity( profile2d, t0,t1, &t) )
  {
    if ( t0 < t && t < t1 )
    {
      if ( 0 != k )
      {
        k->Append(t);
        count++;
      }
      t0 = t;
    }
  }
  return count;
}


ON_Brep* ON_Extrusion::BrepForm( ON_Brep* brep, bool bSmoothFaces ) const
{
  if ( brep )
    brep->Destroy();

  ON_SimpleArray<const ON_Curve*> profile_curves;
  const int profile_count = GetProfileCurves(profile_curves );
  if ( profile_count < 1 || profile_count != profile_curves.Count() )
    return 0;

  // get end cap transformation information

  const ON_3dVector T = m_path.Tangent();
  if ( !T.IsUnitVector() )
    return 0;

  ON_Xform xform0(1.0), xform1(1.0), scale0(1.0), scale1(1.0), rot0(1.0), rot1(1.0);
  if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[0]),T,m_up,m_bHaveN[0]?&m_N[0]:0,xform0,&scale0,&rot0) )
    return 0;

  if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[1]),T,m_up,m_bHaveN[1]?&m_N[1]:0,xform1,&scale1,&rot1) )
    return 0;

  ON_Brep* newbrep = brep ? brep : ON_Brep::New();

  if ( 0 == newbrep )
    return 0;

  int is_capped = IsCapped();
  if ( is_capped < 0 || is_capped > 3 )
    is_capped = 0; // don't let future changes or bugs in IsCapped() crash this code.

  int cap_count = (3==is_capped) ? 2 : (0 != is_capped ? 1 : 0);

  newbrep->m_S.Reserve(profile_count + cap_count);
  newbrep->m_F.Reserve(profile_count + cap_count);
  newbrep->m_L.Reserve((1 + cap_count)*profile_count);

  // Note: 
  //  If the profiles have kinks and bSplitKinkyFaces
  //  is true, then the m_C2, m_T, m_C3 and m_E arrays will
  //  generally be longer than these initial reservations.
  newbrep->m_C2.Reserve((4 + cap_count)*profile_count);
  newbrep->m_T.Reserve((4 + cap_count)*profile_count);
  newbrep->m_C3.Reserve(4*profile_count);
  newbrep->m_E.Reserve(4*profile_count);


  int vidmap[4] = {0,1,2,3};
  int eidmap[4] = {0,1,2,3};
  if ( m_bTransposed )
  {
    vidmap[1] = 3;
    vidmap[3] = 1;
    eidmap[0] = 3;
    eidmap[1] = 2;
    eidmap[2] = 1;
    eidmap[3] = 0;
  }

  int fi0, fi1;

  ON_ClassArray< ON_Extrusion_BrepForm_FaceInfo > finfo(2*profile_count);
  for ( int profile_index = 0; profile_index < profile_count; profile_index++ )
  {
    const ON_Curve* profile_segment = profile_curves[profile_index];
    if ( 0 == profile_segment )
    {
      if (newbrep != brep )
        delete newbrep;
      return 0;
    }

    fi0 = finfo.Count();
    ON_Extrusion_BrepForm_FaceInfo profile_fi;
    profile_fi.Init();
    {
      ON_Curve* newprofile = profile_segment->DuplicateCurve();
      if ( 0 == newprofile )
      {
        if (newbrep != brep )
          delete newbrep;
        return 0;
      }

      profile_fi.m_bClosedProfile = newprofile->IsClosed() ? true : false;
      profile_fi.m_profile_orientation = ( profile_fi.m_bClosedProfile )
                              ? ON_ClosedCurveOrientation(*newprofile,0)
                              : 0;

      if ( is_capped && profile_fi.m_profile_orientation != ((0==profile_index) ? 1 : -1) )
      {
        // do not attempt to cap extrusions with incorrectly oriented profiles
        is_capped = 0;
        cap_count = 0;
      }

      if ( bSmoothFaces )
      {
        // It is important to use the original profile, "profile_segment"
        // and not the trimmed copy in the GetNextDiscontinuity() loop
        // so the code that creates this brep divides the extrusion profile
        // exactly the same way as the code that calculates component indices.
        double t0 = ON_UNSET_VALUE;
        double t1 = ON_UNSET_VALUE;
        if ( !profile_segment->GetDomain(&t0,&t1) 
             || !ON_IsValid(t0)
             || !ON_IsValid(t1) 
             || !(t0 < t1)
           )
        {
          delete newprofile;
          if (newbrep != brep )
            delete newbrep;
          return 0;
        }
        double t = t1;
        while ( GetNextProfileSegmentDiscontinuity(profile_segment,t0,t1,&t) )
        {
          if ( t0 < t && t < t1 )
          {
            ON_Curve* left_side = 0;
            ON_Curve* right_side = 0;
            if ( newprofile->Split(t,left_side,right_side)
                 && 0 != left_side
                 && 0 != right_side
               )
            {
              finfo.AppendNew().m_extrusion_profile = left_side;
              left_side = 0;
              delete newprofile;
              newprofile = right_side;
              right_side = 0;
              t0 = t;
              t = t1;
              continue;
            }
            if ( 0 != left_side )
              delete left_side;
            if ( 0 != right_side )
              delete right_side;
          }
          break;
        }
      }
      finfo.AppendNew().m_extrusion_profile = newprofile;
    }

    fi1 = finfo.Count();

    if ( fi1 <= fi0 )
    {
      if (newbrep != brep )
        delete newbrep;
      return 0;
    }

    for ( int finfo_index = fi0; finfo_index < fi1; finfo_index++ )
    {
      // create an extrusion that represents a single surface
      // to store in the brep.
      ON_Extrusion_BrepForm_FaceInfo& fi = finfo[finfo_index];
      fi.m_face_index = -1;

      fi.m_extrusion_srf = new ON_Extrusion();
      fi.m_extrusion_srf->m_path = m_path;
      fi.m_extrusion_srf->m_t = m_t;
      fi.m_extrusion_srf->m_up = m_up;
      fi.m_extrusion_srf->m_profile_count = 1;
      fi.m_extrusion_srf->m_profile = fi.m_extrusion_profile;
      fi.m_extrusion_srf->m_bCap[0] = false;
      fi.m_extrusion_srf->m_bCap[1] = false;
      fi.m_extrusion_srf->m_bHaveN[0] = m_bHaveN[0];
      fi.m_extrusion_srf->m_bHaveN[1] = m_bHaveN[1];
      fi.m_extrusion_srf->m_N[0] = m_N[0];
      fi.m_extrusion_srf->m_N[1] = m_N[1];
      fi.m_extrusion_srf->m_path_domain = m_path_domain;
      fi.m_extrusion_srf->m_bTransposed = m_bTransposed;

      fi.m_profile_index = profile_index;
      fi.m_bClosedProfile = profile_fi.m_bClosedProfile;
      fi.m_profile_orientation = profile_fi.m_profile_orientation;

      // When a profile has multiple profile_list[] entries,
      // the west side of the "next" face is joined to
      // the east side of the "previous" face.
      fi.m_vid[vidmap[0]] = profile_fi.m_vid[vidmap[1]];
      fi.m_vid[vidmap[3]] = profile_fi.m_vid[vidmap[2]];
      fi.m_eid[eidmap[3]] = profile_fi.m_eid[eidmap[1]];
      if ( fi.m_eid[eidmap[3]] >= 0 )
        fi.m_bRev3d[eidmap[3]] = (profile_fi.m_bRev3d[eidmap[1]]) ? false : true;

      if (    profile_fi.m_bClosedProfile
           && finfo_index > fi0
           && finfo_index == fi1 - 1
         )
      {
        // When a profile is closed and there are multiple
        // profile_list[] entries, the east side of the 
        // last face is joined to the west side of the
        // first face.
        fi.m_vid[vidmap[1]] = profile_fi.m_vid[vidmap[0]];
        fi.m_vid[vidmap[2]] = profile_fi.m_vid[vidmap[3]];
        fi.m_eid[eidmap[1]] = profile_fi.m_eid[eidmap[3]];
        if ( fi.m_eid[eidmap[1]] >= 0 )
          fi.m_bRev3d[eidmap[1]] = (profile_fi.m_bRev3d[eidmap[3]]) ? false : true;
      }

      // Create a face topology around the surface fi.m_extrusion_srf
      ON_NurbsSurface* face_srf = fi.m_extrusion_srf->NurbsSurface();
      if ( 0 == face_srf )
      {
        continue;
        ////if (newbrep != brep )
        ////  delete newbrep;
        ////return 0;
      }

      const ON_BrepFace* face = newbrep->NewFace(face_srf,fi.m_vid,fi.m_eid,fi.m_bRev3d);
      if ( 0 == face )
      {
        // When NewFace() returns null, face_srf is not reference in newbrep.
        delete face_srf;

        // 10 July 2012 Dale Lear
        //    Continue to attempt to get a brep form.  This failure
        //    typically happens when face_srf has two singularities
        //    because a profile component is very short.
        continue;
        ////if (newbrep != brep )
        ////  delete newbrep;
        ////return 0;
      }
      fi.m_face_index = face->m_face_index; 

      // Save vid[] and eid[] informtion so subsequent faces
      // are properly joined.
      if ( finfo_index == fi0 )
      {
        profile_fi.m_vid[vidmap[0]] = fi.m_vid[vidmap[0]];
        profile_fi.m_vid[vidmap[3]] = fi.m_vid[vidmap[3]];
        profile_fi.m_eid[eidmap[3]] = fi.m_eid[eidmap[3]];
        profile_fi.m_bRev3d[eidmap[3]] = fi.m_bRev3d[eidmap[3]];
      }
      profile_fi.m_vid[vidmap[1]] = fi.m_vid[vidmap[1]];
      profile_fi.m_vid[vidmap[2]] = fi.m_vid[vidmap[2]];
      profile_fi.m_eid[eidmap[1]] = fi.m_eid[eidmap[1]];
      profile_fi.m_bRev3d[eidmap[1]] = fi.m_bRev3d[eidmap[1]];

      if ( 0 == is_capped )
        continue;

      const ON_BrepLoop* loop = (1 == face->LoopCount()) ? face->OuterLoop() : 0;
      if ( 0 == loop || 4 != loop->TrimCount() )
      {
        is_capped = 0;
        cap_count = 0;
        continue;
      }

      const ON_BrepTrim* bottom_trim = loop->Trim(eidmap[0]);
      if ( 0 == bottom_trim || ON_BrepTrim::boundary != bottom_trim->m_type )
      {
        is_capped = 0;
        cap_count = 0;
        continue;
      }

      const ON_BrepTrim* top_trim = loop->Trim(eidmap[2]);
      if ( 0 == top_trim || ON_BrepTrim::boundary != top_trim->m_type )
      {
        is_capped = 0;
        cap_count = 0;
        continue;
      }

      const ON_BrepEdge* edge0 = bottom_trim->Edge();
      const ON_BrepEdge* edge1 = top_trim->Edge();
      if ( 0 == edge0 || 0 == edge1 )
      {
        is_capped = 0;
        cap_count = 0;
        continue;
      }
      fi.m_cap_trim_index[0] = bottom_trim->m_trim_index;
      fi.m_cap_trim_index[1] = top_trim->m_trim_index;
      fi.m_cap_edge_index[0] = edge0->m_edge_index;
      fi.m_cap_edge_index[1] = edge1->m_edge_index;
    }
  }

  // Add end caps
  bool bSetEdgeTolerances = false;

  while ( is_capped > 0 && cap_count == ((3 == is_capped) ? 2 : 1) )
  {
    // while(...) not a loop 
    // - break's are used for flow control and there is a break at the bottom of this scope.

    // Add end caps.
    if ( !MakeCap2dCurvesHelper(finfo,is_capped,scale0,scale1) )
      break;

    newbrep->m_S.Reserve(newbrep->m_S.Count()+cap_count);
    newbrep->m_F.Reserve(newbrep->m_F.Count()+cap_count);
    ON_BrepFace* capface0 = 0;
    ON_BrepFace* capface1 = 0;
    {
      ON_PlaneSurface* plane = 0;
      int si;
      if ( 3 == is_capped  || 1 == is_capped )
      {
        // create bottom face cap
        plane = MakeCapPlaneHelper( finfo, 0, rot0 );
        if ( plane )
        {
          si = newbrep->AddSurface(plane);
          plane = 0;
          capface0 = &newbrep->NewFace(si);
          capface0->m_bRev = m_bTransposed ? false : true;
        }
      }
      if ( 3 == is_capped  || 2 == is_capped )
      {
        // create top face cap
        plane = MakeCapPlaneHelper( finfo, 1, rot1 );
        if ( plane )
        {
          si = newbrep->AddSurface(plane);
          plane = 0;
          capface1 = &newbrep->NewFace(si);
          capface1->m_bRev = m_bTransposed ? true : false;
        }
      }
    }

    if ( 0 == capface0 && 0 == capface1 )
      break;

    newbrep->m_C2.Reserve(newbrep->m_C2.Count()+cap_count*finfo.Count());
    newbrep->m_L.Reserve(newbrep->m_L.Count()+cap_count*profile_count);
    newbrep->m_T.Reserve(newbrep->m_T.Count()+cap_count*finfo.Count());

    int cap_index = 0;
    for ( cap_index = 0; cap_index < 2; cap_index++ )
    {
      // make all the loops and trims for the bottom cap
      // before making them for the top cap. This makes
      // it easier for the picking code to create ON_Brep
      // component indices when picking the extrusion
      // and the brep index is not present.
      ON_BrepFace* capface = ( 0 == cap_index ) ? capface0 : capface1;
      if ( 0 == capface )
        continue;

      fi0 = 0;
      int profile_index;
      for ( profile_index = 0; profile_index < profile_count && fi0 < finfo.Count(); profile_index++ )
      {
        if ( finfo[fi0].m_profile_index != profile_index )
          break;
        ON_BrepLoop::TYPE loop_type = ( 0 == profile_index ) ? ON_BrepLoop::outer : ON_BrepLoop::inner;
        fi1 = MakeCapLoopHelper(finfo,fi0,cap_index,capface,loop_type,&bSetEdgeTolerances);
        if ( fi1 <= fi0 )
          break;
        fi0 = fi1;
      }

      if ( fi0 != finfo.Count() || profile_index != profile_count )
      {
        ON_ERROR("Failed to add caps to extrusion brep form.");
        if ( 0 != capface0 )
          newbrep->DeleteFace(*capface0,false);
        if ( 0 != capface1 )
          newbrep->DeleteFace(*capface1,false);
        newbrep->Compact();
        break;
      }
    }

    if ( 2 == cap_index && 2 == cap_count && 3 == is_capped )
      ((CMyBrepIsSolidSetter*)newbrep)->SetIsSolid(m_bTransposed?2:1);

    break;
  }

  if ( newbrep )
  {
    // set a tight bounding box
    ((CMyBrepIsSolidSetter*)newbrep)->SetBBox(*this);
    newbrep->SetTrimBoundingBoxes(true);

    if ( bSetEdgeTolerances )
      newbrep->SetEdgeTolerances(true);
  }

#if defined(ON_DEBUG)
  if ( !newbrep->IsValid() )
  {
    newbrep->IsValid(); // breakpoint here
  }
#endif

  return newbrep;
}

bool ON_Extrusion::GetBrepFormComponentIndex(
  ON_COMPONENT_INDEX extrusion_ci,
  ON_COMPONENT_INDEX& brep_ci
  ) const
{
  const ON_Brep* null_brep_pointer = 0;
  return GetBrepFormComponentIndex(extrusion_ci,ON_UNSET_VALUE,*null_brep_pointer,brep_ci);
}

static bool GetBrepFormFaceIndex(
        const ON_Extrusion& extrusion,
        int extrusion_profile_index,
        double extrusion_profile_parameter,
        bool bCountProfileDiscontinuities,
        int* brep_form_face_index,
        ON_Interval* profile_segment_domain
        )
{
  // When extrusion_profile_index = profile_count,
  // it means the caller is looking for the index of
  // the bottom cap.
  int profile_segment_count = 0;
  const int profile_count = extrusion.ProfileCount();
  double t0 = ON_UNSET_VALUE;
  double t1 = ON_UNSET_VALUE;
  const ON_Curve* profile;
  if ( ON_UNSET_VALUE == extrusion_profile_parameter )
  {
    if ( extrusion_profile_index != profile_count )
    {
      profile = extrusion.Profile(extrusion_profile_index);
      if (    0 == profile
           || !profile->GetDomain(&t0,&t1) 
           || !ON_IsValid(t0)
           || !ON_IsValid(t1) 
           || !(t0 < t1)
         )
      {
        return false;
      }
    }
    profile_segment_count = extrusion_profile_index;
  }
  else
  {
    for ( int i = 0; i < profile_count; i++ )
    {
      profile = extrusion.Profile(i);
      if ( 0 == profile )
        return false;
      // It is important to use the original profile, "profile_segment"
      // and not the trimmed copy in the GetNextDiscontinuity() loop
      // so the code that creates this brep divides the extrusion profile
      // exactly the same way as the code that calculates component indices.
      if (    0 == profile
           || !profile->GetDomain(&t0,&t1) 
           || !ON_IsValid(t0)
           || !ON_IsValid(t1) 
           || !(t0 < t1)
         )
      {
        return false;
      }

      double t = t1;
      if ( bCountProfileDiscontinuities )
      {
        while ( GetNextProfileSegmentDiscontinuity(profile,t0,t1,&t) )
        {
          if ( t0 < t && t < t1 )
          {
            if ( i == extrusion_profile_index 
                 && t0 <= extrusion_profile_parameter 
                 && extrusion_profile_parameter < t 
               )
            {
              break;
            }
            t0 = t;
            t = t1;
            profile_segment_count++;
            continue;
          }
          break;
        }
      }
      if ( i == extrusion_profile_index )
        break;
      profile_segment_count++;
    }
  }

  if ( 0 != brep_form_face_index )
    *brep_form_face_index = profile_segment_count;

  if ( 0 != profile_segment_domain && extrusion_profile_index < profile_count )
    profile_segment_domain->Set(t0,t1);

  return true;
}

bool ON_Extrusion::GetBrepFormComponentIndex(
  ON_COMPONENT_INDEX extrusion_ci,
  double extrusion_profile_parameter,
  const ON_Brep& brep_form,
  ON_COMPONENT_INDEX& brep_ci
  ) const
{
  brep_ci.UnSet();
  int face_index = -1;
  ON_Interval face_profile_domain(ON_UNSET_VALUE,ON_UNSET_VALUE);
  const ON_Brep* brep = &brep_form; // brep pointer can be null

  const int is_capped = IsCapped();
  if ( is_capped < 0 || is_capped > 3 )
    return false;
  const int profile_count = ProfileCount();
  if ( profile_count < 1 )
    return false;
  const ON_Curve* profile0 = Profile(0);
  if ( 0 == profile0 )
    return false;
  const bool bClosedProfile = profile0->IsClosed() ? true : false;
  if ( profile_count > 1 && !bClosedProfile )
    return false;
  const int edges_per_wall_face = bClosedProfile ? 3 : 4;
  const int cap_count = (0 == is_capped || !bClosedProfile) ? 0 : ((3==is_capped)?2:1);
  int brep_face_count = ( 0 != brep ) ? brep->m_F.Count() : 0;
  if ( 0 != brep && brep_face_count < profile_count + cap_count )
  {
    ON_ERROR("brep_form parameter cannot be extrusion's BrepForm()");
    return false;
  }
  bool bCountProfileDiscontinuities = ( brep_face_count > profile_count + cap_count );

  switch(extrusion_ci.m_type)
  {
  case ON_COMPONENT_INDEX::extrusion_bottom_profile:
  case ON_COMPONENT_INDEX::extrusion_top_profile:
    if ( extrusion_ci.m_index < 0 || extrusion_ci.m_index >= profile_count )
      return false;
    if ( !GetBrepFormFaceIndex( *this, extrusion_ci.m_index, extrusion_profile_parameter, bCountProfileDiscontinuities, &face_index, &face_profile_domain ) )
      return false;
    brep_ci.m_index = edges_per_wall_face*face_index;
    if ( ON_COMPONENT_INDEX::extrusion_top_profile == extrusion_ci.m_type )
      brep_ci.m_index += 2;
    brep_ci.m_type = ON_COMPONENT_INDEX::brep_edge;
    break;

  case ON_COMPONENT_INDEX::extrusion_wall_edge:
    if ( extrusion_ci.m_index < 0 || extrusion_ci.m_index >= 2*profile_count )
      return false;
    if ( !GetBrepFormFaceIndex( *this, extrusion_ci.m_index/2, extrusion_profile_parameter, bCountProfileDiscontinuities, &face_index, &face_profile_domain ) )
      return false;
    brep_ci.m_index = edges_per_wall_face*face_index+1;
    if ( !bClosedProfile && 1 == (face_index % 1) )
      brep_ci.m_index += 2;
    brep_ci.m_type = ON_COMPONENT_INDEX::brep_edge;
    break;

  case ON_COMPONENT_INDEX::extrusion_wall_surface:
    if ( extrusion_ci.m_index < 0 || extrusion_ci.m_index >= profile_count )
      return false;
    if ( !GetBrepFormFaceIndex( *this, extrusion_ci.m_index, extrusion_profile_parameter, bCountProfileDiscontinuities, &face_index, &face_profile_domain ) )
      return false;
    brep_ci.m_index = face_index;
    brep_ci.m_type = ON_COMPONENT_INDEX::brep_face;
    break;

  case ON_COMPONENT_INDEX::extrusion_cap_surface:
    if ( extrusion_ci.m_index < 0 || extrusion_ci.m_index > 2 )
      return false;
    if ( 1 == extrusion_ci.m_index && (is_capped != 1 && is_capped != 3) )
      return false;
    if ( 2 == extrusion_ci.m_index && (is_capped != 2 && is_capped != 3) )
      return false;
    if ( 0 != brep )
    {
      face_index = brep->m_F.Count()-cap_count;
    }
    else if ( !GetBrepFormFaceIndex( *this, profile_count, extrusion_profile_parameter, bCountProfileDiscontinuities, &face_index, &face_profile_domain ) )
    {
      return false;
    }

    brep_ci.m_index = face_index+extrusion_ci.m_index-1;
    brep_ci.m_type = ON_COMPONENT_INDEX::brep_face;
    break;

  case ON_COMPONENT_INDEX::extrusion_path:
    // There is no corresponding brep component index
    break;

  default:
    // Other ON_COMPONENT_INDEX::TYPE values intentionally ignored
    break;
  }

  if ( !brep_ci.IsBrepComponentIndex() )
  {
    brep_ci.UnSet();
    return false;
  }

  return true;
}

ON_BOOL32 ON_Extrusion::SetDomain( 
  int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
  double t0, 
  double t1
  )
{
  bool rc = false;
  if ( ON_IsValid(t0) && ON_IsValid(t1) && t0 < t1 )
  {
    const int path_dir = PathParameter();
    if ( path_dir == dir )
    {
      m_path_domain.Set(t0,t1);
      rc = true;
    }
    else if ( 1-path_dir == dir )
    {
      rc = m_profile->SetDomain(t0,t1)?true:false;
    }
  }
  return rc;
}

ON_Interval ON_Extrusion::Domain(
  int dir // 0 gets first parameter's domain, 1 gets second parameter's domain
  ) const 
{
  const int path_dir = PathParameter();
  return (path_dir == dir ) 
         ? m_path_domain 
         : ((1-path_dir == dir && m_profile) ? m_profile->Domain() : ON_Interval());
}

ON_BOOL32 ON_Extrusion::GetSurfaceSize( 
    double* width, 
    double* height 
    ) const
{
  bool rc = true;
  //int path_dir = PathParameter();
  if ( PathParameter() )
  {
    double* p = width;
    width = height;
    height = p;
  }
  if ( width )
  {
    if ( m_path.IsValid() && m_t.IsIncreasing() )
      *width = m_path.Length()*m_t.Length();
    else
    {
      *width = 0.0;
      rc = false;
    }
  }
  if (height)
  {
    if ( m_profile )
    {
      // A crude over estimate is good enough for file IO 
      // needs in the public souce code version. When there
      // are multiple profile components, the nurbs_profile_curve
      // will have a whacky control polygon, but it's length will
      // be ok as an estimate.
      ON_NurbsCurve nurbs_profile_curve;
      if ( m_profile->GetNurbForm(nurbs_profile_curve) <= 0 )
      {
        *height = 0.0;
        rc = false;
      }
      else
      {
        *height = nurbs_profile_curve.ControlPolygonLength();
      }
    }
    else 
    {
      rc = false;
      *height = 0.0;
    }
  }
  return rc;
}

int ON_Extrusion::SpanCount(
  int dir // 0 gets first parameter's domain, 1 gets second parameter's domain
  ) const // number of smooth nonempty spans in the parameter direction
{
  const int path_dir = PathParameter();
  if ( path_dir == dir )
    return 1;
  if ( 1-path_dir == dir && m_profile )
    return m_profile->SpanCount();
  return 0;
}

ON_BOOL32 ON_Extrusion::GetSpanVector( // span "knots" 
      int dir, // 0 gets first parameter's domain, 1 gets second parameter's domain
      double* span_vector // array of length SpanCount() + 1 
      ) const  // 
{
  if ( span_vector )
  {
    const int path_dir = PathParameter();
    if ( path_dir == dir )
    {
      span_vector[0] = m_path_domain[0];
      span_vector[1] = m_path_domain[1];
      return true;
    }
    if ( 1-path_dir == dir && m_profile )
    {
      return m_profile->GetSpanVector(span_vector);
    }
  }
  return false;
}

ON_BOOL32 ON_Extrusion::GetSpanVectorIndex(
      int dir , // 0 gets first parameter's domain, 1 gets second parameter's domain
      double t,      // [IN] t = evaluation parameter
      int side,         // [IN] side 0 = default, -1 = from below, +1 = from above
      int* span_vector_index,        // [OUT] span vector index
      ON_Interval* span_interval // [OUT] domain of the span containing "t"
      ) const
{
  const int path_dir = PathParameter();
  if ( path_dir == dir )
  {
    if ( span_vector_index )
      *span_vector_index = 0;
    if ( span_interval )
      *span_interval = m_path_domain;
    return true;
  }
  if ( 1-path_dir == dir && m_profile )
  {
    return m_profile->GetSpanVectorIndex(t,side,span_vector_index,span_interval);
  }
  return false;
}

int ON_Extrusion::Degree( // returns maximum algebraic degree of any span 
                // ( or a good estimate if curve spans are not algebraic )
  int dir // 0 gets first parameter's domain, 1 gets second parameter's domain
  ) const  
{
  const int path_dir = PathParameter();
  if ( path_dir == dir )
    return 1;
  if ( 1-path_dir == dir && m_profile )
    return m_profile->Degree();
  return 0;
}

ON_BOOL32 ON_Extrusion::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
       int dir,        // 0 gets first parameter, 1 gets second parameter
       double t,       // t = parameter in domain
       double* tminus, // tminus
       double* tplus   // tplus
       ) const
{
  const int path_dir = PathParameter();
  if ( path_dir == dir )
    return ON_Surface::GetParameterTolerance(dir,t,tminus,tplus);
  if ( 1-path_dir==dir && m_profile)
    return m_profile->GetParameterTolerance(t,tminus,tplus);
  return false;
}

ON_Surface::ISO ON_Extrusion::IsIsoparametric(
      const ON_Curve& curve,
      const ON_Interval* curve_domain
      ) const
{
  return ON_Surface::IsIsoparametric(curve,curve_domain);
}

ON_BOOL32 ON_Extrusion::IsPlanar(
      ON_Plane* plane,
      double tolerance
      ) const
{
  if ( m_profile && m_profile->IsLinear(tolerance) )
  {
    if ( plane )
    {
      ON_3dPoint P0 = m_profile->PointAtStart();
      ON_3dPoint P1 = m_profile->PointAtEnd();
      ON_3dVector pathT = m_path.Tangent();
      ON_3dVector Y = m_up;
      ON_3dVector X = ON_CrossProduct(Y,pathT);
      if ( !X.IsUnitVector() )
        X.Unitize();
      ON_3dPoint Q0 = m_path.from + P0.x*X + P0.y*Y;
      ON_3dPoint Q1 = m_path.from + P1.x*X + P1.y*Y;
      ON_3dVector N = ON_CrossProduct(pathT,Q1-Q0);
      N.Unitize();
      plane->origin = Q0;
      if ( false == m_bTransposed )
      {
        plane->yaxis = pathT;
        plane->zaxis = -N;
        plane->xaxis = ON_CrossProduct(plane->yaxis,plane->zaxis);
        plane->xaxis.Unitize();
      }
      else
      {
        plane->xaxis = pathT;
        plane->zaxis = N;
        plane->yaxis = ON_CrossProduct(plane->zaxis,plane->xaxis);
        plane->yaxis.Unitize();
      }
      plane->UpdateEquation();
    }    
    return true;
  }
  return false;
}

ON_BOOL32 ON_Extrusion::IsClosed(int dir) const
{
  const int path_dir = PathParameter();
  if ( 1-path_dir == dir && m_profile )
    return m_profile->IsClosed();
  return false;
}

ON_BOOL32 ON_Extrusion::IsPeriodic( int dir ) const
{
  const int path_dir = PathParameter();
  if ( 1-path_dir == dir && m_profile )
    return m_profile->IsPeriodic();
  return false;
}

bool ON_Extrusion::GetNextDiscontinuity( 
                int dir,
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
  const int path_dir = PathParameter();
  if ( path_dir == dir )
  {
    return ON_Surface::GetNextDiscontinuity(dir,c,t0,t1,t,hint,dtype,cos_angle_tolerance,curvature_tolerance);
  }
  if ( 1-path_dir==dir && m_profile)
  {
    return m_profile->GetNextDiscontinuity(c,t0,t1,t,hint,dtype,cos_angle_tolerance,curvature_tolerance);
  }
  return false;
}

bool ON_Extrusion::IsContinuous(
  ON::continuity c,
  double s, 
  double t, 
  int* hint,
  double point_tolerance,
  double d1_tolerance,
  double d2_tolerance,
  double cos_angle_tolerance,
  double curvature_tolerance
  ) const
{
  if ( !m_profile )
    return false;
  int* crv_hint = 0;
  double curvet;
  if ( m_bTransposed )
  {
    curvet = s;
    crv_hint = hint;
  }
  else
  {
    curvet = t;
    crv_hint = hint ? hint+1 : 0;
  }
  return m_profile->IsContinuous(c,curvet,crv_hint,point_tolerance,d1_tolerance,d2_tolerance,cos_angle_tolerance,curvature_tolerance);
}

ON_Surface::ISO ON_Extrusion::IsIsoparametric(
      const ON_BoundingBox& bbox
      ) const
{
  return ON_Surface::IsIsoparametric(bbox);
}

ON_BOOL32 ON_Extrusion::Reverse( int dir )
{
  if ( 0 == m_profile )
    return false;

  const int path_dir = PathParameter();

  if ( path_dir == dir )
  {
    m_path_domain.Reverse();
    m_path.Reverse();

    // Need to mirror profile about 2d x-axis
    // NOTE:
    //   If the profile is closed and the extrusion
    //   is capped, then this will leave the extrusion
    //   in an invalid "inside-out" state.
    //   It is the responsibility of the caller
    //   to explicitly reverse the profile as well.
    //   I cannot "automatically" do it here because
    //   it breaks existing reparamterization code 
    //   that checks for closed objects and makes
    //   a sequence changes that results in a valid
    //   final result.
    ON_Xform profile_xform(1.0);
    profile_xform.m_xform[0][0] = -1.0;
    bool bNeedReverse = false;
    return Profile2dTransform(*this,profile_xform,bNeedReverse);
  }

  if ( 1-path_dir == dir )
  {
    return m_profile->Reverse();
    // NOTE:
    //   If the profile is closed and the extrusion
    //   is capped, then this will leave the extrusion
    //   in an invalid "inside-out" state.
    //   It is the responsibility of the caller
    //   to explicitly reverse the profile as well.
    //   I cannot "automatically" do it here because
    //   it breaks existing reparamterization code 
    //   that checks for closed objects and makes
    //   a sequence changes that results in a valid
    //   final result.
  }

  return false;
}

ON_BOOL32 ON_Extrusion::Transpose() // transpose surface parameterization (swap "s" and "t")
{
  m_bTransposed = m_bTransposed?false:true;
  return true;
}

ON_BOOL32 ON_Extrusion::Evaluate( // returns false if unable to evaluate
       double u, double v,   // evaluation parameters
       int num_der,          // number of derivatives (>=0)
       int array_stride,     // array stride (>=Dimension())
       double* der_array,    // array of length stride*(ndir+1)*(ndir+2)/2
       int quadrant ,     // optional - determines which quadrant to evaluate from
                             //         0 = default
                             //         1 from NE quadrant
                             //         2 from NW quadrant
                             //         3 from SW quadrant
                             //         4 from SE quadrant
       int* hint          // optional - evaluation hint (int[2]) used to speed
                             //            repeated evaluations
       ) const 
{
  if ( !m_profile )
    return false;

  double x,y,dx,dy;
  //int side = 0;
  if ( m_bTransposed ) 
  {
    x = u; u = v; v = x;
    if ( 4 == quadrant )
      quadrant = 2;
    else if ( 2 == quadrant )
      quadrant = 4;
  }

  if ( !m_profile->Evaluate( u, num_der, array_stride, der_array,
                             (1==quadrant||4==quadrant)?1:((2==quadrant||3==quadrant)?-1:0),
                               hint) 
     )
  {
    return false;
  }

  // TODO: After testing, add special case that avoids
  //       two calls to GetProfileTransformation() when 
  //       mitering is trivial.
  const double t1 = m_path_domain.NormalizedParameterAt(v);
  const double t0 = 1.0-t1;
  ON_Xform xform0, xform1;
  const ON_3dVector T = m_path.Tangent();
  if ( 0.0 != t0 || num_der > 0 )
  {
    if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[0]),T,m_up,m_bHaveN[0]?&m_N[0]:0,xform0,0,0) )
      return false;
  }
  else
  {
    xform0.Zero();
  }
  if ( 0.0 != t1 || num_der > 0 )
  {
    if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[1]),T,m_up,m_bHaveN[1]?&m_N[1]:0,xform1,0,0) )
      return false;
  }
  else
  {
    xform1.Zero();
  }

  double xformP[3][3], xformD[3][3];
  xformP[0][0] = t0*xform0.m_xform[0][0] + t1*xform1.m_xform[0][0];
  xformP[0][1] = t0*xform0.m_xform[0][1] + t1*xform1.m_xform[0][1];
  xformP[0][2] = t0*xform0.m_xform[0][3] + t1*xform1.m_xform[0][3];
  xformP[1][0] = t0*xform0.m_xform[1][0] + t1*xform1.m_xform[1][0];
  xformP[1][1] = t0*xform0.m_xform[1][1] + t1*xform1.m_xform[1][1];
  xformP[1][2] = t0*xform0.m_xform[1][3] + t1*xform1.m_xform[1][3];
  xformP[2][0] = t0*xform0.m_xform[2][0] + t1*xform1.m_xform[2][0];
  xformP[2][1] = t0*xform0.m_xform[2][1] + t1*xform1.m_xform[2][1];
  xformP[2][2] = t0*xform0.m_xform[2][3] + t1*xform1.m_xform[2][3];

  int i,j;
  i = num_der+1;
  double* d1 = der_array + array_stride*(i*(i+1)/2 - 1);
  double* d0 = der_array + array_stride*(i - 1);
  x = d0[0];
  y = d0[1];
  if ( num_der > 0 )
  {
    double d = m_path_domain.m_t[1] - m_path_domain.m_t[0];
    if ( d > 0.0 )
      d = 1.0/d;
    xformD[0][0] = d*(xform1.m_xform[0][0] - xform0.m_xform[0][0]);
    xformD[0][1] = d*(xform1.m_xform[0][1] - xform0.m_xform[0][1]);
    xformD[0][2] = d*(xform1.m_xform[0][3] - xform0.m_xform[0][3]);
    xformD[1][0] = d*(xform1.m_xform[1][0] - xform0.m_xform[1][0]);
    xformD[1][1] = d*(xform1.m_xform[1][1] - xform0.m_xform[1][1]);
    xformD[1][2] = d*(xform1.m_xform[1][3] - xform0.m_xform[1][3]);
    xformD[2][0] = d*(xform1.m_xform[2][0] - xform0.m_xform[2][0]);
    xformD[2][1] = d*(xform1.m_xform[2][1] - xform0.m_xform[2][1]);
    xformD[2][2] = d*(xform1.m_xform[2][3] - xform0.m_xform[2][3]);

    for ( i = num_der; i > 0; i-- )
    {
      dx = x;
      dy = y;
      d0 -= array_stride;
      x = d0[0];
      y = d0[1];

      // all partials involving two or more derivatives with
      // respect to "v" are zero.
      j = i;
      while ( --j )
      {
        d1[0] = d1[1] = d1[2] = 0.0;
        d1 -= array_stride;
      }    

      // The partial involving a single derivative with respect to "v"
      if ( 1 == i )
      {
        // xformD transform is applied to curve location ((x,y) = point)
        d1[0] = xformD[0][0]*x + xformD[0][1]*y + xformD[0][2];
        d1[1] = xformD[1][0]*x + xformD[1][1]*y + xformD[1][2];
        d1[2] = xformD[2][0]*x + xformD[2][1]*y + xformD[2][2];
      }
      else
      {
        // xformD transform is applied to a curve derivative ((x,y) = vector)
        d1[0] = xformD[0][0]*x + xformD[0][1]*y;
        d1[1] = xformD[1][0]*x + xformD[1][1]*y;
        d1[2] = xformD[2][0]*x + xformD[2][1]*y;
      }
      d1 -= array_stride;

      // The partial involving a all derivatives with respect to "u"
      // xformP transformation is applied to a curve derivative ((x,y) = vector)
      d1[0] = xformP[0][0]*dx + xformP[0][1]*dy;
      d1[1] = xformP[1][0]*dx + xformP[1][1]*dy;
      d1[2] = xformP[2][0]*dx + xformP[2][1]*dy;
      d1 -= array_stride;
    }
  }
  // xformP transformation is applied curve location ((x,y) = point)
  d1[0] = xformP[0][0]*x + xformP[0][1]*y + xformP[0][2];
  d1[1] = xformP[1][0]*x + xformP[1][1]*y + xformP[1][2];
  d1[2] = xformP[2][0]*x + xformP[2][1]*y + xformP[2][2];

  if ( m_bTransposed && num_der > 0)
  {
    // reverse order of derivatives
    const size_t sz = ((3 <= array_stride)?3:array_stride)*sizeof(double);
    void* tmp = ( sz <= sizeof(xform0) )
              ? ((void*)&xform0.m_xform[0][0])
              : onmalloc(sz);
    for ( i = 1; i <= num_der; i++ )
    {
      d0 = der_array + array_stride*(i*(i+1))/2;
      d1 = d0 + array_stride*i;
      while ( d0 < d1)
      {
        memcpy(tmp,d0,sz);
        memcpy(d0,d1,sz);
        memcpy(d1,tmp,sz);
        d0 += array_stride;
        d1 -= array_stride;
      }
    }
    if ( tmp != ((void*)&xform0.m_xform[0][0]) )
      onfree(tmp);
  }

  return true;
}

ON_Curve* ON_Extrusion::IsoCurve(
       int dir,
       double c
       ) const
{
  // dir 0 first parameter varies and second parameter is constant
  //       e.g., point on IsoCurve(0,c) at t is srf(t,c)
  //     1 first parameter is constant and second parameter varies
  //       e.g., point on IsoCurve(1,c) at t is srf(c,t)

  if ( !m_profile )
    return 0;

  if ( m_bTransposed )
    dir = 1-dir;
  const ON_3dVector T = m_path.Tangent();

  ON_Xform xform0, xform1;
  if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[0]),T,m_up,m_bHaveN[0]?&m_N[0]:0,xform0,0,0) )
    return 0;
  if ( !ON_GetEndCapTransformation(m_path.PointAt(m_t.m_t[1]),T,m_up,m_bHaveN[1]?&m_N[1]:0,xform1,0,0) )
    return 0;

  ON_Curve*  isocurve = 0;
  if ( 1 == dir )
  {
    ON_3dPoint P = m_profile->PointAt(c);
    ON_LineCurve* line_curve = new ON_LineCurve();
    line_curve->m_t = m_path_domain;
    line_curve->m_dim = 3;
    line_curve->m_line.from = xform0*P;
    line_curve->m_line.to = xform1*P;
    isocurve = line_curve;
  }
  else if ( 0 == dir )
  {
    double s1 = m_path_domain.NormalizedParameterAt(c);
    const double s0 = 1.0-s1;
    xform1.m_xform[0][0] = s0*xform0.m_xform[0][0] + s1*xform1.m_xform[0][0];
    xform1.m_xform[0][1] = s0*xform0.m_xform[0][1] + s1*xform1.m_xform[0][1];
    xform1.m_xform[0][2] = s0*xform0.m_xform[0][2] + s1*xform1.m_xform[0][2];
    xform1.m_xform[0][3] = s0*xform0.m_xform[0][3] + s1*xform1.m_xform[0][3];

    xform1.m_xform[1][0] = s0*xform0.m_xform[1][0] + s1*xform1.m_xform[1][0];
    xform1.m_xform[1][1] = s0*xform0.m_xform[1][1] + s1*xform1.m_xform[1][1];
    xform1.m_xform[1][2] = s0*xform0.m_xform[1][2] + s1*xform1.m_xform[1][2];
    xform1.m_xform[1][3] = s0*xform0.m_xform[1][3] + s1*xform1.m_xform[1][3];

    xform1.m_xform[2][0] = s0*xform0.m_xform[2][0] + s1*xform1.m_xform[2][0];
    xform1.m_xform[2][1] = s0*xform0.m_xform[2][1] + s1*xform1.m_xform[2][1];
    xform1.m_xform[2][2] = s0*xform0.m_xform[2][2] + s1*xform1.m_xform[2][2];
    xform1.m_xform[2][3] = s0*xform0.m_xform[2][3] + s1*xform1.m_xform[2][3];

    xform1.m_xform[3][0] = s0*xform0.m_xform[3][0] + s1*xform1.m_xform[3][0];
    xform1.m_xform[3][1] = s0*xform0.m_xform[3][1] + s1*xform1.m_xform[3][1];
    xform1.m_xform[3][2] = s0*xform0.m_xform[3][2] + s1*xform1.m_xform[3][2];
    xform1.m_xform[3][3] = s0*xform0.m_xform[3][3] + s1*xform1.m_xform[3][3];

    isocurve = m_profile->DuplicateCurve();
    if ( isocurve )
    {
      isocurve->ChangeDimension(3);
      if ( !isocurve->Transform(xform1) )
      {
        // isocurve is probably a circle
        ON_NurbsCurve* nurbs_curve = isocurve->NurbsCurve();
        delete isocurve;
        isocurve = nurbs_curve;
        nurbs_curve = 0;
        if ( isocurve )
          isocurve->Transform(xform1);
      }
    }
  }

  return isocurve;
}

ON_BOOL32 ON_Extrusion::Trim(
       int dir,
       const ON_Interval& domain
       )
{
  bool rc = false;
  if (!domain.IsIncreasing())
    return false;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( 1 == dir )
  {
    rc = m_path_domain.IsIncreasing();
    if ( rc && m_path_domain != domain )
    {
      ON_Interval dom;
      dom.Intersection(domain,m_path_domain);
      rc = dom.IsIncreasing();
      if (rc)
      {
        double s0 = m_path_domain.NormalizedParameterAt(dom[0]);
        double s1 = m_path_domain.NormalizedParameterAt(dom[1]);
        double t0 = (1.0-s0)*m_t[0] + s0*m_t[1];
        double t1 = (1.0-s1)*m_t[0] + s1*m_t[1];
        rc = (s0 < s1 && 0.0 <= t0 && t0 < t1 && t1 <= 1.0);
        if (rc)
        {
          bool bChanged = false;
          if (t0 != m_t[0] && t0 > 0.0 )
          {
            bChanged = true;
            m_t[0] = t0;
            m_bHaveN[0] = false;
          }
          if ( t1 != m_t[1] && t1 < 1.0 )
          {
            bChanged = true;
            m_t[1] = t1;
            m_bHaveN[1] = false;
          }
          if ( bChanged )
          {
            m_path_domain = dom;
            DestroySurfaceTree();
          }
        }
      }
    }
  }
  else if ( 0 == dir )
  {
    if ( m_profile )
    {
      rc = m_profile->Trim(domain)?true:false;
      DestroySurfaceTree();
    }
  }
  return rc;
}

bool ON_Extrusion::Extend(
  int dir,
  const ON_Interval& domain
  )
{
  bool rc = false;
  if ( 1 == dir )
  {
    rc = domain.IsIncreasing() && m_path_domain.IsIncreasing();
    if ( rc )
    {
      double s0 = m_path_domain.NormalizedParameterAt(domain[0]);
      if ( s0 > 0.0 )
        s0 = 0.0;
      double s1 = m_path_domain.NormalizedParameterAt(domain[1]);
      if ( s1 < 1.0 )
        s1 = 1.0;
      double t0 = (1.0-s0)*m_t[0] + s0*m_t[1];
      double t1 = (1.0-s1)*m_t[0] + s1*m_t[1];
      bool bChanged = false;
      ON_3dPoint P0 = m_path.from;
      ON_3dPoint P1 = m_path.to;
      if ( t0 < m_t[0] )
      {
        bChanged = true;
        m_path_domain.m_t[0] = domain[0];
        if ( t0 < 0.0 )
        {
          P0 = m_path.PointAt(t0);
          m_t[0] = 0.0;
        }
        else
          m_t[0] = t0;
      }
      if ( t1 > m_t[1] )
      {
        bChanged = true;
        m_path_domain.m_t[1] = domain[1];
        if ( t1 > 1.0 )
        {
          P1 = m_path.PointAt(t1);
          m_t[1] = 1.0;
        }
        else
          m_t[1] = t1;
      }
      if ( bChanged )
      {
        m_path.from = P0;
        m_path.to = P1;
        DestroySurfaceTree();
      }
    }
  }
  else if ( 0 == dir )
  {
    if ( m_profile )
    {
      rc = m_profile->Extend(domain);
      if (rc) 
        DestroySurfaceTree();
    }
  }
  return rc;
}

ON_BOOL32 ON_Extrusion::Split(
       int dir,
       double c,
       ON_Surface*& west_or_south_side,
       ON_Surface*& east_or_north_side
       ) const
{
  if ( dir < 0 || dir > 1 || !ON_IsValid(c) )
    return false;
  if ( 0 != west_or_south_side && west_or_south_side == east_or_north_side )
    return false;

  ON_Interval domain = Domain(dir);
  double s = domain.NormalizedParameterAt(c);
  if ( s <= 0.0 || s >= 1.0 )
    return false;
  if (c <= domain[0] || c >= domain[1] )
    return false;

  ON_Extrusion* left = 0;
  ON_Extrusion* right = 0;
  if ( west_or_south_side )
  {
    left = ON_Extrusion::Cast(west_or_south_side);
    if ( !left )
      return false;
  }
  if ( east_or_north_side )
  {
    right = ON_Extrusion::Cast(east_or_north_side);
    if ( !right )
      return false;
  }

  const int path_dir = PathParameter();
  bool rc = false;
  if ( dir == path_dir )
  {
    // split path
    ON_Line left_path, right_path;
    ON_Interval left_domain, right_domain;
    ON_Interval left_t, right_t;

    const double t0 = m_t[0];
    const double t1 = m_t[1];
    const double t = (1.0-s)*t0 + s*t1;
    if ( !ON_IsValid(t) || t <= t0 || t >= t1 )
      return false;

    ON_3dPoint P = m_path.PointAt(s);
    left_path.from = m_path.from;
    left_path.to = P;
    right_path.from = P;
    right_path.to = m_path.to;
    left_domain.Set(domain[0],c);
    right_domain.Set(c,domain[1]);
    left_t.Set(t0,t);
    right_t.Set(t,t1);
    if ( !left_path.IsValid() || left_path.Length() <= m_path_length_min )
      return false;
    if ( !right_path.IsValid() || right_path.Length() <= m_path_length_min )
      return false;

    // return result
    if ( !left )
      left = new ON_Extrusion(*this);
    else if ( left != this )
      left->operator =(*this);
    else
      left->DestroyRuntimeCache();
    if ( !right )
      right = new ON_Extrusion(*this);
    else if ( right != this )
      right->operator =(*this);
    else
      right->DestroyRuntimeCache();
    left->m_path = left_path;
    left->m_path_domain = left_domain;
    left->m_t = left_t;
    right->m_path = right_path;
    right->m_path_domain = right_domain;
    right->m_t = right_t;

    west_or_south_side = left;
    east_or_north_side = right;
    rc = true;
  }
  else
  {
    if ( 0 == m_profile )
      return false;
    ON_Curve* left_profile = 0;
    ON_Curve* right_profile = 0;

    if ( left == this )
    {
      left_profile = left->m_profile;
      left->DestroyRuntimeCache();
    }
    else if ( 0 != left && 0 != left->m_profile )
    {
      delete left->m_profile;
      left->m_profile = 0;
    }

    if ( right == this )
    {
      right_profile = right->m_profile;
      right->DestroyRuntimeCache();
    }
    else if ( 0 != right && 0 != right->m_profile )
    {
      delete right->m_profile;
      right->m_profile = 0;
    }

    if ( !m_profile->Split(c,left_profile,right_profile) )
      return false;
    if ( 0 == left_profile || 0 == right_profile )
    {
      if ( 0 != left_profile && m_profile != left_profile )
        delete left_profile;
      if ( 0 != right_profile && m_profile != right_profile )
        delete right_profile;
      return false;
    }

    ON_Curve* this_profile = 0;
    if ( left_profile != m_profile && right_profile != m_profile )
    {
      if ( left == this || right == this )
      {
        delete m_profile;
      }
      else
      {
        this_profile = m_profile;
      }
    }

    // Prevent this m_profile from being copied
    const_cast<ON_Extrusion*>(this)->m_profile = 0;

    // Create new left and right sides with NULL profiles
    if ( !left )
      left = new ON_Extrusion(*this);
    else if ( left != this )
      left->operator =(*this);
    if ( !right )
      right = new ON_Extrusion(*this);
    else if ( right != this )
      right->operator =(*this);

    // Restore this m_profile
    const_cast<ON_Extrusion*>(this)->m_profile = this_profile;

    // Set left and right profiles
    left->m_profile = left_profile;
    right->m_profile = right_profile;

    west_or_south_side = left;
    east_or_north_side = right;
    rc = true;
  }

  return rc;
}

int ON_Extrusion::GetNurbForm(
      ON_NurbsSurface& nurbs_surface,
      double tolerance
      ) const
{
  if ( !m_profile )
    return 0;

  ON_Xform xform0,xform1;
  if ( !GetProfileTransformation(0,xform0) )
    return false;
  if ( !GetProfileTransformation(1,xform1) )
    return false;

  ON_NurbsCurve nc0;
  int rc = m_profile->GetNurbForm(nc0,tolerance);
  if ( rc <= 0 )
    return rc;
  if ( 3 != nc0.m_dim )
    nc0.ChangeDimension(3);
  ON_NurbsCurve nc1 = nc0;
  nc0.Transform(xform0);
  nc1.Transform(xform1);

  nurbs_surface.Create(3,nc0.m_is_rat,nc0.m_order,2,nc0.m_cv_count,2);
  memcpy(nurbs_surface.m_knot[0],nc0.m_knot,nurbs_surface.KnotCount(0)*sizeof(nurbs_surface.m_knot[0][0]));
  nurbs_surface.m_knot[1][0] = m_path_domain[0];
  nurbs_surface.m_knot[1][1] = m_path_domain[1];
  for ( int i = 0; i < nurbs_surface.m_cv_count[0]; i++ )
  {
    nurbs_surface.SetCV(i,0,ON::intrinsic_point_style,nc0.CV(i));
    nurbs_surface.SetCV(i,1,ON::intrinsic_point_style,nc1.CV(i));
  }

  if ( m_bTransposed )
    nurbs_surface.Transpose();

  return rc;
}

int ON_Extrusion::HasNurbForm() const
{
  return m_profile ? m_profile->HasNurbForm() : 0;
}

bool ON_Extrusion::GetSurfaceParameterFromNurbFormParameter(
      double nurbs_s, double nurbs_t,
      double* surface_s, double* surface_t
      ) const
{
  bool rc = true;
  if ( m_bTransposed )
  {
    double* p = surface_s; 
    surface_s = surface_t; 
    surface_t = p;
    double t = nurbs_s;
    nurbs_s = nurbs_t;
    nurbs_t = t;
  }
  if ( surface_s )
  {
    rc = m_profile 
       ? (m_profile->GetCurveParameterFromNurbFormParameter(nurbs_s,surface_s)?true:false) 
       : false;
  }
  if ( surface_t )
    *surface_t = nurbs_t;
  return rc;
}

bool ON_Extrusion::GetNurbFormParameterFromSurfaceParameter(
      double surface_s, double surface_t,
      double* nurbs_s,  double* nurbs_t
      ) const
{
  bool rc = true;
  if ( m_bTransposed )
  {
    double p = surface_s; 
    surface_s = surface_t; 
    surface_t = p;
    double* t = nurbs_s;
    nurbs_s = nurbs_t;
    nurbs_t = t;
  }
  if ( nurbs_s )
  {
    rc = m_profile 
      ? (m_profile->GetNurbFormParameterFromCurveParameter(surface_s,nurbs_s)?true:false)
      : false;
  }
  if ( nurbs_t )
    *nurbs_t = surface_t;
  return rc;
}

ON_SumSurface* ON_Extrusion::SumSurfaceForm( 
  ON_SumSurface* sum_surface 
  ) const
{
  int i;
  if ( 0 != sum_surface )
  {
    for ( i = 0; i < 2; i++ )
    {
      if ( sum_surface->m_curve[i] )
      {
        delete sum_surface->m_curve[i];
        sum_surface->m_curve[i] = 0;
      }
      sum_surface->m_basepoint = ON_3dVector::ZeroVector;
      sum_surface->m_bbox.Destroy();
    }
  }

  if ( 0 == m_profile || !m_path.IsValid() )
    return 0;

  if ( IsMitered() )
    return 0; // mitered extrusions cannot be represented as sum surfaces

  ON_Xform xform0;
  if ( !GetProfileTransformation(0.0,xform0) )
    return 0;

  ON_Curve* profile3d = 0;
  ON_LineCurve* path = 0;
  ON_Curve* curve0 = 0;
  ON_Curve* curve1 = 0;
  for(;;)
  {
    if ( 1 == ProfileCount() )
    {
      const ON_PolyCurve* polycurve = ON_PolyCurve::Cast(m_profile);
      if ( 0 != polycurve && 1 == polycurve->Count() )
      {
        const ON_Curve* segment = polycurve->SegmentCurve(0);
        if ( 0 != segment )
        {
          profile3d = segment->DuplicateCurve();
          profile3d->SetDomain( m_profile->Domain() );
        }
      }
    }
    if ( 0 == profile3d )
    {
      profile3d = m_profile->DuplicateCurve();
      if ( 0 == profile3d )
        break;
    }
    if ( profile3d->IsLinear() && 0 == ON_LineCurve::Cast(profile3d) )
    {
      ON_LineCurve* line_curve = new ON_LineCurve();
      line_curve->m_line.from = profile3d->PointAtStart();
      line_curve->m_line.to = profile3d->PointAtEnd();
      line_curve->ON_Curve::SetDomain(profile3d->Domain());
      delete profile3d;
      profile3d = line_curve;
    }
    if ( !profile3d->ChangeDimension(3) )
      break;
    if ( !xform0.IsIdentity() && !profile3d->Transform(xform0) )
      break;

    path = new ON_LineCurve();
    if ( 0 == path )
      break;
    path->m_line.from = ON_3dPoint::Origin;
    path->m_line.to = (m_path.to - m_path.from);
    if ( !path->SetDomain( m_path_domain[0], m_path_domain[1] ) )
      break;

    curve0 = profile3d;
    curve1 = path;
    profile3d = 0;
    path = 0;
    break;
  }
  if ( 0 == curve0 || 0 == curve1 )
  {
    if ( 0 != profile3d )
      delete profile3d;
    if ( 0 != path )
      delete path;
    return 0;
  }

  ON_SumSurface* sumsrf = ( 0 != sum_surface ) ? sum_surface : new ON_SumSurface();
  if ( 0 == sumsrf )
  {
    delete curve0;
    delete curve1;
    return 0;
  }

  sumsrf->m_curve[0] = curve0;
  sumsrf->m_curve[1] = curve1;
  sumsrf->m_basepoint = ON_3dVector::ZeroVector;
  sumsrf->m_bbox = BoundingBox();

  if ( m_bTransposed )
    sumsrf->Transpose();

  return sumsrf;
}

ON_Extrusion* ON_Extrusion::Cylinder( 
  const ON_Cylinder& cylinder, 
  bool bCapBottom,
  bool bCapTop,
  ON_Extrusion* extrusion
  )
{
  if ( !cylinder.IsValid() || !cylinder.IsFinite() )
    return 0;

  ON_Line path;
  path.from = cylinder.circle.plane.PointAt(0.0,0.0,cylinder.height[0]);
  path.to   = cylinder.circle.plane.PointAt(0.0,0.0,cylinder.height[1]);
  if ( !path.IsValid() || !(path.Length() > ON_ZERO_TOLERANCE) )
    return 0;

  ON_3dVector up = cylinder.circle.plane.yaxis;
  if (    !up.IsValid()
       || !up.IsUnitVector()
       || fabs(up*path.Tangent()) > ON_SQRT_EPSILON 
       )
    return 0;

  ON_ArcCurve* circle_curve = new ON_ArcCurve(cylinder.circle);
  circle_curve->m_arc.plane = ON_Plane::World_xy;
  circle_curve->m_dim = 2;
  if ( !circle_curve->IsValid() )
  {
    delete circle_curve;
    return 0;
  }

  ON_Extrusion* extrusion_cylinder = 0;
  if ( extrusion )
  {
    extrusion->Destroy();
    extrusion_cylinder = extrusion;
  }
  else
  {
    extrusion_cylinder = new ON_Extrusion();
  }

  if (    !extrusion_cylinder->SetPathAndUp(path.from,path.to,up)
       || !extrusion_cylinder->SetOuterProfile(circle_curve,false)
       || !extrusion_cylinder->IsValid()
       || !extrusion_cylinder->SetDomain(extrusion_cylinder->PathParameter(),cylinder.height[0],cylinder.height[1])
     )
  {
    if ( 0 == extrusion )
      delete extrusion_cylinder;
    return 0;
  }

  extrusion_cylinder->m_bCap[0] = bCapBottom ? true : false;
  extrusion_cylinder->m_bCap[1] = bCapTop    ? true : false;

  if ( !extrusion_cylinder->IsValid() )
  {
    if ( 0 == extrusion )
      delete extrusion_cylinder;
    return 0;
  }

  return extrusion_cylinder;
}



ON_Extrusion* ON_Extrusion::Pipe( 
  const ON_Cylinder& cylinder, 
  double other_radius,
  bool bCapBottom,
  bool bCapTop,
  ON_Extrusion* extrusion
  )
{
  if (    !cylinder.IsValid() 
       || !ON_IsValid(other_radius)
       || !(fabs(other_radius - cylinder.circle.Radius()) > ON_ZERO_TOLERANCE)
       )
  {
    return 0;
  }

  double inner_radius = (other_radius < cylinder.circle.radius)
                      ? other_radius
                      : cylinder.circle.radius;
  double outer_radius = (other_radius < cylinder.circle.radius)
                      ? cylinder.circle.radius
                      : other_radius;
  if (    !ON_IsValid(inner_radius) 
       || !ON_IsValid(outer_radius)
       || !(outer_radius - inner_radius > ON_ZERO_TOLERANCE)
     )
  {
    return 0;
  }

  ON_Cylinder outer_cylinder = cylinder;
  outer_cylinder.circle.radius = outer_radius;

  ON_Circle inner_circle(ON_Plane::World_xy,inner_radius);
  ON_ArcCurve* inner_profile = new ON_ArcCurve(inner_circle);
  inner_profile->m_dim = 2;
  if ( !inner_profile->IsValid() )
  {
    delete inner_profile;
    return 0;
  }

  ON_Extrusion* extrusion_pipe = ON_Extrusion::Cylinder(outer_cylinder,bCapBottom,bCapTop,extrusion);
  if ( 0 == extrusion_pipe )
  {
    delete inner_profile;
    return 0;
  }

  if ( !extrusion_pipe->IsValid() )
  {
    if ( 0 == extrusion )
      delete extrusion_pipe;
    delete inner_profile;
    return 0;
  }

  if ( !extrusion_pipe->AddInnerProfile(inner_profile) )
  {
    if ( 0 == extrusion )
      delete extrusion_pipe;
    delete inner_profile;
    return 0;
  }

  if ( !extrusion_pipe->IsValid() )
  {
    if ( 0 == extrusion )
      delete extrusion_pipe;
    return 0;
  }

  return extrusion_pipe;
}


ON_Extrusion* ON_Extrusion::CreateFrom3dCurve( 
    const ON_Curve& curve,
    const ON_Plane* plane,
    double height,
    bool bCap,
    ON_Extrusion* extrusion
    )
  {
    if ( 0 != extrusion )
      extrusion->Destroy();

    if ( ON_IsValid(height) && 0.0 == height )
      return 0;

    ON_Interval z(0.0,height);
    if ( z.IsDecreasing() )
      z.Swap();
    if ( !z.IsIncreasing() )
      return 0;
    
    if ( !curve.IsValid() )
      return 0;

    ON_Plane curve_plane;
    if ( 0 == plane )
    {
      if ( !curve.IsPlanar(&curve_plane) )
        return 0;
      plane = &curve_plane;
    }

    if ( !plane->IsValid() )
      return 0;

    ON_Xform xform2d;
    xform2d.ChangeBasis(ON_Plane::World_xy,*plane);

    ON_Curve* curve2d = curve.DuplicateCurve();
    if ( 0 == curve2d )
      return 0;

    ON_Extrusion* result = 0;

    for (;;)
    {
      if ( !curve2d->Transform(xform2d) )
        break;
      curve2d->ChangeDimension(2);

      if ( 0 == extrusion )
        result = new ON_Extrusion();
      else
        result = extrusion;

      if ( !result->SetPathAndUp(
                  plane->PointAt(0.0,0.0,z[0]),
                  plane->PointAt(0.0,0.0,z[1]),
                  plane->yaxis
                  ) )
        break;

      if ( !result->SetOuterProfile(curve2d,bCap) )
        break;

      if ( !result->IsValid() )
        break;

      // success
      curve2d = 0;

      break;
    }

    if ( 0 != curve2d )
    {
      // failure
      delete curve2d;
      curve2d = 0;
      
      if ( 0 != result && result != extrusion )
        delete result;

      if ( extrusion )
        extrusion->Destroy();

      result = 0;
    }

    return result;
  }

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

ON_OBJECT_IMPLEMENT( ON_SumSurface, ON_Surface, "C4CD5359-446D-4690-9FF5-29059732472B" );

void ON_SumSurface::DestroyRuntimeCache( bool bDelete )
{
  ON_Surface::DestroyRuntimeCache(bDelete);
  if ( 0 != m_curve[0] )
    m_curve[0]->DestroyRuntimeCache(bDelete);
  if ( 0 != m_curve[1] )
    m_curve[1]->DestroyRuntimeCache(bDelete);
  // 15 August 2003 Dale Lear:
  //    Added the call to destroy the cached bounding box
  m_bbox.Destroy();
}

ON_SumSurface* ON_SumSurface::New()
{
  return new ON_SumSurface();
}

ON_SumSurface* ON_SumSurface::New( const ON_SumSurface& rev_surface )
{
  return new ON_SumSurface(rev_surface);
}

ON_SumSurface::ON_SumSurface() : m_basepoint(0.0,0.0,0.0)
{
  ON__SET__THIS__PTR(m_s_ON_SumSurface_ptr);
  m_curve[0] = 0;
  m_curve[1] = 0;
}

ON_SumSurface::~ON_SumSurface()
{
  Destroy();
}

void ON_SumSurface::Destroy()
{
	// GBA 24-Sept-2004. Since there now could be a surface tree it need to be cleared out.
	DestroyRuntimeCache();
  for ( int i = 0; i < 2; i++ )
  {
    if ( m_curve[i] ) {
      delete m_curve[i];
      m_curve[i] = 0;
    }
  }
  m_bbox.Destroy();
  m_basepoint.Set(0.0,0.0,0.0);
}

void ON_SumSurface::EmergencyDestroy()
{
  m_curve[0] = 0;
  m_curve[1] = 0;
}

ON_SumSurface::ON_SumSurface( const ON_SumSurface& src )
{
  ON__SET__THIS__PTR(m_s_ON_SumSurface_ptr);
  m_curve[0] = 0;
  m_curve[1] = 0;
  *this = src;
}

unsigned int ON_SumSurface::SizeOf() const
{
  unsigned int sz = ON_Surface::SizeOf();
  if ( m_curve[0] )
    sz += m_curve[0]->SizeOf();
  if ( m_curve[1] )
    sz += m_curve[1]->SizeOf();
  return sz;
}

ON__UINT32 ON_SumSurface::DataCRC(ON__UINT32 current_remainder) const
{
  if ( m_curve[0] )
    current_remainder = m_curve[0]->DataCRC(current_remainder);
  if ( m_curve[1] )
    current_remainder = m_curve[1]->DataCRC(current_remainder);
  return current_remainder;
}

ON_SumSurface& ON_SumSurface::operator=(const ON_SumSurface& src )
{
  if ( this != &src ) 
  {
    Destroy();
    for ( int i = 0; i < 2; i++ )
    {
      if ( src.m_curve[i] ) 
      {
        ON_Object* obj = src.m_curve[i]->DuplicateCurve();
        m_curve[i] = ON_Curve::Cast(obj);
        if ( !m_curve[i] )
          delete obj;
      }
    }
    m_basepoint = src.m_basepoint;
    m_bbox = src.m_bbox;
  }
  return *this;
}

ON_BOOL32 ON_SumSurface::Create( const ON_Curve& curve, ON_3dVector vector )
{
  Destroy();
  ON_BOOL32 rc = false;
  if ( !vector.IsZero() )
  {
    ON_Curve* pCurve = curve.DuplicateCurve();
    rc = Create( pCurve, vector );
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::Create( ON_Curve* pCurve, ON_3dVector vector )
{
  Destroy();
  ON_BOOL32 rc = false;
  if ( !vector.IsZero() )
  {
    ON_LineCurve* pLineCurve = new ON_LineCurve( ON_Line( ON_origin, vector ) );
    pLineCurve->SetDomain( 0.0, vector.Length() );
    m_curve[0] = pCurve;
    m_curve[1] = pLineCurve;
    m_basepoint.Set(0.0,0.0,0.0);
    ON_BoundingBox bbox0 = pCurve->BoundingBox();
    ON_BoundingBox bbox1 = bbox0;
    bbox1.m_min += vector;
    bbox1.m_max += vector;
    m_bbox.Union( bbox0, bbox1 );
    rc = true;
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::Create( const ON_Curve& curve, const ON_Curve& path_curve )
{
  Destroy();
  ON_Curve* pCurve = curve.DuplicateCurve();
  ON_Curve* pPathCurve = path_curve.DuplicateCurve();
  ON_BOOL32 rc = Create( pCurve, pPathCurve );
  return rc;
}

ON_BOOL32 ON_SumSurface::Create( ON_Curve* pCurve, ON_Curve* pPathCurve )
{
  Destroy();
  ON_BOOL32 rc = false;
  if ( pCurve && pPathCurve )
  {
    m_curve[0] = pCurve;
    m_curve[1] = pPathCurve;
    m_basepoint = ON_origin - pPathCurve->PointAtStart();
    m_bbox.Destroy();
    BoundingBox();
    rc = true;
  }
  return rc;
}

////////////////////////////////////////////////////////////
//
// overrides of virtual ON_Object functions
//
ON_BOOL32 ON_SumSurface::IsValid( ON_TextLog* text_log ) const
{
  for ( int i = 0; i < 2; i++ )
  {
    if ( !m_curve[i] )
    {
      if ( text_log )
        text_log->Print("ON_SumSurface.m_curve[%d] is NULL.\n",i);
      return false;
    }
    if ( m_curve[i]->Dimension() != 3 )
    {
      if ( text_log )
        text_log->Print("ON_SumSurface.m_curve[%d]->m_dim = %d (should be 3).\n",i,m_curve[i]->Dimension());
      return false;
    }
    if ( !m_curve[i]->IsValid(text_log) )
    {
      if ( text_log )
        text_log->Print("ON_SumSurface.m_curve[%d] is not valid.\n",i);
      return false;
    }
  }
  if ( !m_basepoint.IsValid() )
  {
    if ( text_log )
      text_log->Print("ON_SumSurface.m_basepoint is not valid.\n");
    return false;
  }
  return true;
}

void ON_SumSurface::Dump( ON_TextLog& dump ) const
{
  ON_Object::Dump(dump);
  dump.PushIndent();
  dump.Print("basepoint = ");
  dump.Print(m_basepoint);
  dump.Print("\n");
  for ( int i = 0; i < 2; i++ )
  {
    if ( m_curve[i] )
    {
      dump.Print("m_curve[%d]:\n",i);
      dump.PushIndent();
      m_curve[i]->Dump(dump);
      dump.PopIndent();
    }
    else
      dump.Print("m_curve[%d] = NULL\n",i);
  }
}

ON_BOOL32 ON_SumSurface::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if ( rc ) {
    rc = file.WriteVector( m_basepoint );
    rc = file.WriteBoundingBox( m_bbox );
    if ( rc ) rc = file.WriteObject( m_curve[0] );
    if ( rc ) rc = file.WriteObject( m_curve[1] );
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::Read( ON_BinaryArchive& file )
{
  Destroy();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion( &major_version, &minor_version );
  if (rc && major_version == 1 ) {
    ON_Object* obj;
    rc = file.ReadVector( m_basepoint );
    if (rc) rc = file.ReadBoundingBox( m_bbox );
    obj = 0;
    if (rc) rc = file.ReadObject(&obj);
    if (rc) {
      m_curve[0] = ON_Curve::Cast(obj);
      if ( !m_curve[0] )
        delete obj;
    }
    obj = 0;
    if (rc) rc = file.ReadObject(&obj);
    if (rc) {
      m_curve[1] = ON_Curve::Cast(obj);
      if ( !m_curve[1] )
        delete obj;
    }
  }
  return rc;
}

////////////////////////////////////////////////////////////
//
// overrides of virtual ON_Geometry functions
//
int ON_SumSurface::Dimension() const
{
  int dim  = 0;
  if ( m_curve[0] && m_curve[1] ) {
    dim = m_curve[0]->Dimension();
    if ( dim > 0 ) {
      if ( dim != m_curve[1]->Dimension() )
        dim = 0;
    }
  }
  return dim;
}

void ON_SumSurface::ClearBoundingBox()
{
  m_bbox.Destroy();
}

ON_BOOL32 ON_SumSurface::GetBBox( // returns true if successful
       double* boxmin,    // boxmin[dim]
       double* boxmax,    // boxmax[dim]
       ON_BOOL32 bGrowBox
       ) const
{
  ON_BOOL32 rc = m_bbox.IsValid();
  if (!rc )
  {
    // lazy bounding box evaluation
    ON_BoundingBox bboxA, bboxB;
    if ( m_curve[0] )
      bboxA = m_curve[0]->BoundingBox();
    if ( m_curve[1] )
      bboxB = m_curve[1]->BoundingBox();
    if ( bboxA.IsValid() && bboxB.IsValid() )
    {
      ON_SumSurface* pS = const_cast<ON_SumSurface*>(this);
      pS->m_bbox.m_min = bboxA.m_min + bboxB.m_min + m_basepoint;
      pS->m_bbox.m_max = bboxA.m_max + bboxB.m_max + m_basepoint;
    }
    rc = m_bbox.IsValid();
  }

  if ( rc )
  {
    int dim = Dimension();
    int j;
    ON_BoundingBox bbox;
    if ( bGrowBox && boxmin && boxmax )
    {
      for ( j = 0; j < 3 && j < dim; j++ )
      {
        bbox.m_min[j] = boxmin[j];
        bbox.m_max[j] = boxmax[j];
      }
      if ( !bbox.IsValid() )
        bbox = m_bbox;
      else
        bbox.Union(m_bbox);
    }
    else
      bbox = m_bbox;
    dim = Dimension();
    for ( j = 0; j < 3 && j < dim; j++ )
    {
      if(boxmin) 
        boxmin[j] = bbox.m_min[j];
      if(boxmax) 
        boxmax[j] = bbox.m_max[j];
    }
    for ( j = 3; j < dim; j++ )
    {
      if (boxmin) 
        boxmin[j] = 0.0;
      if (boxmax) 
        boxmax[j] = 0.0;
    }
  }
  return rc;
}

bool ON_SumSurface::IsDeformable() const
{
  bool rc = true;
  if ( m_curve[0] )
    rc = m_curve[0]->IsDeformable();
  if (rc && m_curve[1] )
    rc = m_curve[1]->IsDeformable();
  return rc;
}

bool ON_SumSurface::MakeDeformable()
{
  bool rc = true;
  if ( m_curve[0] && !m_curve[0]->IsDeformable() )
  {
    DestroyRuntimeCache();
    rc = rc && m_curve[0]->MakeDeformable();
  }
  if (m_curve[1] && !m_curve[1]->IsDeformable() )
  {
    DestroyRuntimeCache();
    rc = rc && m_curve[1]->MakeDeformable();
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::Transform( const ON_Xform& xform )
{
  DestroyRuntimeCache();
  TransformUserData(xform);
  ON_BOOL32 rc = false;

  ON_3dPoint A0, A1, A2;
  if ( m_curve[0] ) 
  {
    A0 = m_curve[0]->PointAtStart();
    rc = m_curve[0]->Transform(xform);
  }
  if ( m_curve[1] ) 
  {
    A1 = m_curve[1]->PointAtStart();
    if ( !m_curve[1]->Transform(xform) )
      rc = false;
  }
  else
    rc = false;
  if ( rc )
  {
    // because xform may be affine
    A2 = m_basepoint;
    m_basepoint = xform*(A0+A1+A2) - xform*A0 - xform*A1;
  }
  m_bbox.Destroy();
  m_bbox = BoundingBox();
  return rc;
}

////////////////////////////////////////////////////////////
//
// overrides of virtual ON_Surface functions
//

ON_BOOL32 ON_SumSurface::SetDomain( 
  int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
  double t0, 
  double t1
  )
{
  bool rc = false;
  if ( t0 < t1 && dir >= 0 && dir <= 1 )
  {
    if ( 0 != m_curve[dir] )
    {
      rc = m_curve[dir]->SetDomain(t0,t1) ? true : false;
      DestroyRuntimeCache();
    }
  }
  return rc;
}



ON_Interval ON_SumSurface::Domain( int dir ) const
{
  ON_Interval domain;
  if ( dir == 0 && m_curve[0] )
    domain = m_curve[0]->Domain();
  else if ( dir == 1 && m_curve[1] )
    domain = m_curve[1]->Domain();
  return domain;
}

ON_BOOL32 ON_SumSurface::GetSurfaceSize( 
    double* width, 
    double* height 
    ) const
{
  ON_BOOL32 rc = true;
  double* ptr[2];
  ptr[0] = width;
  ptr[1] = height;
  int j;
  for ( j = 0; j < 2; j++ )
  {
    if ( ptr[j] == NULL )
      continue;
    *ptr[j] = 0.0;
    if ( m_curve[j] == NULL )
      rc = false;

    if ( ! (*ptr[j] > 0.0) )
    {
      int i, imax = 64, hint = 0;
      double length_estimate = 0.0, d = 1.0/((double)imax);
      ON_Interval cdom = m_curve[j]->Domain();
      ON_3dPoint pt0 = ON_UNSET_POINT;
      ON_3dPoint pt;
      for ( i = 0; i <= imax; i++ )
      {
        if ( m_curve[j]->EvPoint( cdom.ParameterAt(i*d), pt, 0, &hint ) )
        {
          if ( pt0 != ON_UNSET_POINT )
            length_estimate += pt0.DistanceTo(pt);
          pt0 = pt;
        }
      }
      *ptr[j] = length_estimate;
    }
  }

  return rc;
}


int ON_SumSurface::SpanCount( int dir ) const
{
  int span_count = 0;
  if ( dir == 0 && m_curve[0] )
    span_count = m_curve[0]->SpanCount();
  else if ( dir == 1 && m_curve[1] )
    span_count = m_curve[1]->SpanCount();
  return span_count;
}

ON_BOOL32 ON_SumSurface::GetSpanVector( int dir, double* s ) const
{
  ON_BOOL32 rc = false;
  if ( dir == 0 && m_curve[0] )
    rc = m_curve[0]->GetSpanVector(s);
  else if ( dir == 1 && m_curve[1] )
    rc = m_curve[1]->GetSpanVector(s);
  return rc;
}


int ON_SumSurface::Degree( int dir ) const
{
  int degree = 0;
  if ( dir == 0 && m_curve[0] )
    degree = m_curve[0]->Degree();
  else if ( dir == 1 && m_curve[1] )
    degree = m_curve[1]->Degree();
  return degree;
}


ON_BOOL32 ON_SumSurface::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
       int dir,     // 0 gets first parameter, 1 gets second parameter
       double t,  // t = parameter in domain
       double* tminus, // tminus
       double* tplus // tplus
       ) const
{
  ON_BOOL32 rc = false;
  if ( dir == 0 && m_curve[0] )
    rc = m_curve[0]->GetParameterTolerance(t,tminus,tplus);
  else if ( dir == 1 && m_curve[1] )
    rc = m_curve[1]->GetParameterTolerance(t,tminus,tplus);
  return rc;
}

ON_BOOL32 ON_SumSurface::IsPlanar(
      ON_Plane* plane,
      double tolerance
      ) const
{
  ON_Plane pln;
  ON_3dPoint center;
  ON_3dVector normal, du, dv;
  ON_Interval udom = Domain(0);
  ON_Interval vdom = Domain(1);
  ON_BOOL32 rc = EvNormal( udom.ParameterAt(0.5), vdom.ParameterAt(0.5), center, du, dv, normal );
  if (rc)
  {
    if ( fabs( normal.Length() - 1.0 ) > 0.01 )
      rc = false;
    else
    {
      pln.origin = center;
      pln.zaxis = normal;
      if ( du.Unitize() )
      {
        pln.xaxis = du;
        pln.yaxis = ON_CrossProduct( pln.zaxis, pln.xaxis );
        pln.yaxis.Unitize();
        pln.UpdateEquation();
      }
      else if ( dv.Unitize() )
      {
        pln.yaxis = dv;
        pln.xaxis = ON_CrossProduct( pln.yaxis, pln.zaxis );
        pln.xaxis.Unitize();
        pln.UpdateEquation();
      }
      else
      {
        pln.CreateFromNormal( center, normal );
      }
      if ( plane )
        *plane = pln;

      int j;
      for ( j = 0; j < 2 && rc ; j++ )
      {
        pln.origin = m_curve[j]->PointAtStart();
        pln.UpdateEquation();
        rc = m_curve[j]->IsInPlane( pln, tolerance );
      }

      if (rc && plane )
      {
        pln.origin = center;
        pln.UpdateEquation();
        *plane = pln;
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::IsClosed( int dir ) const
{
  ON_BOOL32 rc = false;
  if ( dir == 0 && m_curve[0] )
    rc = m_curve[0]->IsClosed();
  else if ( dir == 1 && m_curve[1] )
    rc = m_curve[1]->IsClosed();
  return rc;
}


ON_BOOL32 ON_SumSurface::IsPeriodic( int dir ) const
{
  ON_BOOL32 rc = false;
  if ( dir == 0 && m_curve[0] )
    rc = m_curve[0]->IsPeriodic();
  else if ( dir == 1 && m_curve[1] )
    rc = m_curve[1]->IsPeriodic();
  return rc;
}

ON_BOOL32 ON_SumSurface::IsSingular( int side ) const
{
  return false;
}


bool ON_SumSurface::GetNextDiscontinuity( 
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
  // 28 Jan 2005 - untested code
  bool rc = false;
  if ( 0 == dir || 1 == dir )
  {
    if (0 !=  m_curve[dir] )
    {
      rc = m_curve[dir]->GetNextDiscontinuity(
                c,
                t0,t1,t,
                (hint?&hint[dir]:0),
                dtype,
                cos_angle_tolerance,
                curvature_tolerance);
    }
  }
  return rc;
}

bool ON_SumSurface::IsContinuous(
    ON::continuity desired_continuity,
    double s, 
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
  if ( m_curve[0] && m_curve[1] )
  {
    int crv_hint[2] = {0,0};
    if ( hint ) 
    {
      crv_hint[0] = (*hint) & 0xFFFF;
      crv_hint[1] = ((*hint) & 0xFFFF0000) >> 16;
    }
    rc = m_curve[0]->IsContinuous( desired_continuity, s, &crv_hint[0],
                                      point_tolerance, d1_tolerance, d2_tolerance,
                                      cos_angle_tolerance, curvature_tolerance );
    if (rc )
      rc = m_curve[1]->IsContinuous( desired_continuity, t, &crv_hint[1],
                                      point_tolerance, d1_tolerance, d2_tolerance,
                                      cos_angle_tolerance, curvature_tolerance );
    if ( hint )
    {
      *hint = ( (crv_hint[0]&0xFFFF) | (crv_hint[1]<<16) );
    }
  }
  return rc;
}


ON_BOOL32 ON_SumSurface::Reverse( int dir )
{
  ON_BOOL32 rc = false;
  if ( dir == 0 && m_curve[0] )
    rc = m_curve[0]->Reverse();
  else if ( dir == 1 && m_curve[1] )
    rc = m_curve[1]->Reverse();
	DestroySurfaceTree();
  return rc;
}


ON_BOOL32 ON_SumSurface::Transpose()
{
  ON_Curve* c = m_curve[0];
  m_curve[0] = m_curve[1];
  m_curve[1] = c;
	DestroySurfaceTree();
  return true;
}


ON_BOOL32 ON_SumSurface::Evaluate( // returns false if unable to evaluate
       double s, double t, // evaluation parameters
       int nder,            // number of derivatives (>=0)
       int v_stride,            // array stride (>=Dimension())
       double* v,        // array of length stride*(ndir+1)*(ndir+2)/2
       int side,        // optional - determines which quadrant to evaluate from
                       //         0 = default
                       //         1 from NE quadrant
                       //         2 from NW quadrant
                       //         3 from SW quadrant
                       //         4 from SE quadrant
       int* hint        // optional - evaluation hint (int[2]) used to speed
                       //            repeated evaluations
       ) const
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  if ( dim > 0 ) 
  {
    int crv_hint[2] = {0,0};
    if ( hint ) 
    {
      crv_hint[0] = (*hint) & 0xFFFF;
      crv_hint[1] = ((*hint) & 0xFFFF0000) >> 16;
    }
    double* v0 = (double*)onmalloc( 2*(nder+1)*dim*sizeof(*v0) );
    double* v1 = v0 + (nder+1)*dim;
    int side0, side1;
    switch(side) 
    {
    case 1:
      side0 =  1;
      side1 =  1;
      break;
    case 2:
      side0 = -1;
      side1 =  1;
      break;
    case 3:
      side0 = -1;
      side1 = -1;
      break;
    case 4:
      side0 =  1;
      side1 = -1;
      break;
    default:
      side0 = 1;
      side1 = 1;
      break;
    }
    rc = m_curve[0]->Evaluate(s,nder,dim,v0,side0,hint ? &crv_hint[0] : 0);
    if ( rc )
      rc = m_curve[1]->Evaluate(t,nder,dim,v1,side1,hint ? &crv_hint[1] : 0);
    if (rc) 
    {
      int j,ds,dt,der;
      for ( j = 0; j < dim; j++ ) 
      {
        v[j] = m_basepoint[j] + v0[j] + v1[j];
      }
      for ( der = 1; der <= nder; der++ ) 
      {
        for ( ds = der, dt = 0; ds >= 0; ds--, dt++ )
        {
          v += v_stride;
          for ( j = 0; j < dim; j++ )
            v[j] = 0.0;
          
          // Mar 18 CCW - Fixed bug in evaluator that
          //              returned non-zero values for mixed partials.
          if (ds && dt)
            continue;

          if ( ds )
          {
            for ( j = 0; j < dim; j++ )
              v[j] += v0[j+dim*ds];
          }
          if ( dt )
          {
            for ( j = 0; j < dim; j++ )
              v[j] += v1[j+dim*dt];
          }
        }
      }
    }
    if ( hint ) 
    {
      *hint = crv_hint[0] | (crv_hint[1] << 16);
    }
    onfree(v0);
  }
  return rc;
}

ON_Curve* ON_SumSurface::IsoCurve(int dir, double c ) const
{
  ON_Curve* iso_curve = 0;
  if ( dir >= 0 && dir <= 1 && m_curve[0] && m_curve[1] )
  {
    iso_curve = m_curve[dir]->Duplicate();
    ON_3dPoint p = m_curve[1-dir]->PointAt(c);
    ON_3dVector v = p + m_basepoint;
    if ( !v.IsZero() )
    {
      if ( !iso_curve->Translate(v) )
      {
        delete iso_curve;
        iso_curve = 0;
      }
    }
  }
  return iso_curve;
}

class ON_SumTensor : public ON_TensorProduct
{
public:
  int dim;
  ON_3dPoint basepoint;
  int DimensionA() const;
  int DimensionB() const;
  int DimensionC() const;
  bool Evaluate( double,        // a
                 const double*, // A
                 double,        // b
                 const double*, // B
                 double*        // C
                );
};

int ON_SumTensor::DimensionA() const
{
  return dim;
}

int ON_SumTensor::DimensionB() const
{
  return dim;
}

int ON_SumTensor::DimensionC() const
{
  return dim;
}

bool ON_SumTensor::Evaluate( double a, const double* CurveA, double b, const double* CurveB, double* SrfPoint )
{
  SrfPoint[0] = a*CurveA[0] + b*CurveB[0] + basepoint.x;
  SrfPoint[1] = a*CurveA[1] + b*CurveB[1] + basepoint.y;
  SrfPoint[2] = a*CurveA[2] + b*CurveB[2] + basepoint.z;
	return true;
}


int ON_SumSurface::GetNurbForm(
      ON_NurbsSurface& nurbs_surface,
      double tolerance
      ) const
{
  nurbs_surface.Destroy();
  int rc = 0;
  int dim = Dimension();
  if ( dim > 0 )
  {
    ON_NurbsCurve tmpA, tmpB;
    int rcA = 0;
    int rcB = 0;
    const ON_NurbsCurve* nurbs_curveA=0;
    const ON_NurbsCurve* nurbs_curveB=0;
    nurbs_curveA = ON_NurbsCurve::Cast(m_curve[0]);
    if ( !nurbs_curveA ) 
    {
      rcA = 1;
      rcA = m_curve[0]->GetNurbForm( tmpA, tolerance );
      if ( rcA > 0 )
        nurbs_curveA = &tmpA;
    }
    if ( nurbs_curveA )
    {
      rcB = 1;
      nurbs_curveB = ON_NurbsCurve::Cast(m_curve[1]);
      if ( !nurbs_curveB ) 
      {
        rcB = m_curve[1]->GetNurbForm( tmpB, tolerance );
        if ( rcB > 0 )
          nurbs_curveB = &tmpB;
      }
    }
    if ( nurbs_curveA && nurbs_curveB )
    {
      ON_SumTensor sum_tensor;
      sum_tensor.dim = dim;
      sum_tensor.basepoint = m_basepoint;
      if ( !nurbs_surface.TensorProduct( *nurbs_curveA, *nurbs_curveB, sum_tensor ) )
        nurbs_surface.Destroy();
      else
        rc = (rcA >= rcB) ? rcA : rcB;
    }
  }
  return rc;
}

int ON_SumSurface::HasNurbForm() const

{
  if (Dimension() <= 0)
    return 0;
  int rc = 1;
  int i;
  for (i=0; i<2; i++){
    int nf = m_curve[i]->HasNurbForm();
    if (nf == 0)
      return 0;
    if (nf == 2)
      rc = 2;
  }

  return rc;
}



bool ON_SumSurface::GetSurfaceParameterFromNurbFormParameter(
      double nurbs_s, double nurbs_t,
      double* surface_s, double* surface_t
      ) const
{
  // NOTE: overrides ON_Surface virtual function
  bool rc = (m_curve[0] && m_curve[1]) ? true : false;
  *surface_s = nurbs_s;
  *surface_t = nurbs_t;
  if ( m_curve[0] )
  {
    if ( !m_curve[0]->GetCurveParameterFromNurbFormParameter( nurbs_s, surface_s ) )
      rc = false;
  }

  if ( m_curve[1] )
  {
    if (!m_curve[1]->GetCurveParameterFromNurbFormParameter( nurbs_t, surface_t ))
      rc = false;
  }

  return rc;
}

bool ON_SumSurface::GetNurbFormParameterFromSurfaceParameter(
      double surface_s, double surface_t,
      double* nurbs_s,  double* nurbs_t
      ) const
{
  // NOTE: overrides ON_Surface virtual function
  bool rc = (m_curve[0] && m_curve[1]) ? true : false;
  *nurbs_s = surface_s;
  *nurbs_t = surface_t;
  if ( m_curve[0] )
  {
    if ( !m_curve[0]->GetNurbFormParameterFromCurveParameter( surface_s, nurbs_s  ) )
      rc = false;
  }

  if ( m_curve[1] )
  {
    if (!m_curve[1]->GetNurbFormParameterFromCurveParameter( surface_t, nurbs_t ))
      rc = false;
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::Trim(int dir,
                         const ON_Interval& domain
                         )

{
  if ( dir < 0 || dir > 1 )
    return false;
  ON_Interval current_domain = Domain(dir);
  if ( current_domain[0] == ON_UNSET_VALUE && current_domain[1] == ON_UNSET_VALUE )
    current_domain = domain;
  ON_Interval trim_domain;
  trim_domain.Intersection(domain, Domain(dir) );
  if ( !trim_domain.IsIncreasing() )
    return false;
  if (trim_domain[0] == current_domain[0] 
       && trim_domain[1] == current_domain[1] )
    return true;
  m_bbox.Destroy();
  DestroySurfaceTree();
  return m_curve[dir]->Trim(trim_domain);
}

bool ON_SumSurface::Extend(
      int dir,
      const ON_Interval& domain
      )
{
  if ( dir < 0 || dir > 1 ) return false;
  if (IsClosed(dir)) return false;
  ON_Interval current_domain = Domain(dir);
  if (!m_curve[dir]) return false;
  bool rc = m_curve[dir]->Extend(domain);
  if (rc){
    DestroySurfaceTree();
    m_bbox.Destroy();
  }
  return rc;
}

ON_BOOL32 ON_SumSurface::Split(int dir,
                          double c,
                          ON_Surface*& west_or_south_side,
                          ON_Surface*& east_or_north_side
                          ) const

{
  if ( dir < 0 || dir > 1 )
    return false;
  if ( !Domain(dir).Includes( c, true ) )
    return false;
  ON_SumSurface* left_srf = 0;
  ON_SumSurface* right_srf = 0;


  if ( west_or_south_side )
  {
    left_srf = ON_SumSurface::Cast( west_or_south_side );
    if ( !left_srf )
      return false;
    left_srf->DestroySurfaceTree();
    left_srf->m_bbox.Destroy();
  }

  if ( east_or_north_side )
  {
    right_srf = ON_SumSurface::Cast( east_or_north_side );
    if ( !right_srf )
      return false;
    right_srf->DestroySurfaceTree();
    right_srf->m_bbox.Destroy();
  }

  if (!left_srf) left_srf = ON_SumSurface::New(*this);
  else if (left_srf != this) *left_srf = *this;

  if (!right_srf) right_srf = ON_SumSurface::New(*this);
  else if (right_srf != this) *right_srf = *this;

  if (left_srf == this && right_srf == this)
    return false;

  if (left_srf != this) {
    delete left_srf->m_curve[dir];
    left_srf->m_curve[dir] = 0;
  }

  if (right_srf != this) {
    delete right_srf->m_curve[dir];
    right_srf->m_curve[dir] = 0;
  }


  if (!m_curve[dir]->Split(c, left_srf->m_curve[dir], 
    right_srf->m_curve[dir])){
    if (!west_or_south_side) delete left_srf;
    if (!east_or_north_side) delete right_srf;
    return false;
  }

  if (!west_or_south_side) west_or_south_side = left_srf;
  if (!east_or_north_side) east_or_north_side = right_srf;

  return true;

}

/*

bool ON_SumSurface::GetSurfaceParameterFromNurbFormParameter(
      double nurbs_s, double nurbs_t,
      double* surface_s, double* surface_t
      ) const
{
  // NOTE: overrides ON_Surface virtual function
  bool rc;
  if ( m_curve[0] )
  {
    rc = m_curve[0]->GetCurveParameterFromNurbFormParameter( nurbs_s, surface_s );
  }
  else
  {
    rc = false;
    *surface_s = nurbs_s;
  }

  if ( m_curve[1] )
  {
    if ( m_curve[1]->GetCurveParameterFromNurbFormParameter( nurbs_t, surface_t ) )
      rc = false;
  }
  else
  {
    rc = false;
    *surface_t = nurbs_t;
  }

  return rc;
}


bool ON_SumSurface::GetNurbFormParameterFromSurfaceParameter(
      double surface_s, double surface_t,
      double* nurbs_s,  double* nurbs_t
      ) const
{
  // NOTE: overrides ON_Surface virtual function
  bool rc;

  if ( m_curve[0] )
  {
    rc = m_curve[0]->GetNurbFormParameterFromCurveParameter( surface_s, nurbs_s );
  }
  else
  {
    rc = false;
    *surface_s = nurbs_s;
  }

  if ( m_curve[1] )
  {
    if ( m_curve[1]->GetNurbFormParameterFromCurveParameter( surface_t, nurbs_t ) )
      rc = false;
  }
  else
  {
    rc = false;
    *surface_t = nurbs_t;
  }

  return rc;
}

*/

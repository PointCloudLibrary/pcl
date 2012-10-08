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

ON_OBJECT_IMPLEMENT(ON_CurveOnSurface,ON_Curve,"4ED7D4D8-E947-11d3-BFE5-0010830122F0");

ON_CurveOnSurface::ON_CurveOnSurface() : m_c2(0), m_c3(0), m_s(0)
{}

ON_CurveOnSurface::ON_CurveOnSurface( ON_Curve* c2, ON_Curve* c3, ON_Surface* s ) 
                 : m_c2(c2), m_c3(c3), m_s(s)
{}

ON_CurveOnSurface::ON_CurveOnSurface( const ON_CurveOnSurface& src ) : m_c2(0), m_c3(0), m_s(0)
{
  *this = src;
}

unsigned int ON_CurveOnSurface::SizeOf() const
{
  unsigned int sz = ON_Curve::SizeOf();
  sz += sizeof(*this) - sizeof(ON_Curve);
  if ( m_c2 )
    sz += m_c2->SizeOf();
  if ( m_c3 )
    sz += m_c3->SizeOf();
  if ( m_s )
    sz += m_s->SizeOf();
  return sz;
}

ON_CurveOnSurface& ON_CurveOnSurface::operator=( const ON_CurveOnSurface& src )
{
  if ( this != &src ) {
    ON_Curve::operator=(src);
    if ( m_c2 ) {
      delete m_c2;
      m_c2 = 0;
    }
    if ( m_c3 ) {
      delete m_c3;
      m_c3 = 0;
    }
    if ( m_s ) {
      delete m_s;
      m_s = 0;
    }
    if ( ON_Curve::Cast(src.m_c2) ) {
      m_c2 = ON_Curve::Cast(src.m_c2->Duplicate());
    }
    if ( ON_Curve::Cast(src.m_c3) ) {
      m_c3 = ON_Curve::Cast(src.m_c3->Duplicate());
    }
    if ( ON_Surface::Cast(src.m_s) ) {
      m_s = ON_Surface::Cast(src.m_s->Duplicate());
    }
  }
  return *this;
}

ON_CurveOnSurface::~ON_CurveOnSurface()
{
  if ( m_c2 ) {
    delete m_c2;
    m_c2 = 0;
  }
  if ( m_c3 ) {
    delete m_c3;
    m_c3 = 0;
  }
  if ( m_s ) {
    delete m_s;
    m_s = 0;
  }
}

ON_BOOL32
ON_CurveOnSurface::IsValid( ON_TextLog* text_log ) const
{
  if ( !m_c2 )
    return false;
  if ( !m_s )
    return false;
  if ( !m_c2->IsValid() )
    return false;
  if ( m_c2->Dimension() != 2 ) {
    ON_ERROR("ON_CurveOnSurface::IsValid() m_c2 is not 2d.");
    return false;
  }
  if ( !m_s->IsValid() )
    return false;
  if ( m_c3 ) {
    if ( !m_c3->IsValid() )
      return false;
    if ( m_c3->Dimension() != m_s->Dimension() ) {
      ON_ERROR("ON_CurveOnSurface::IsValid() m_c3 and m_s have different dimensions.");
      return false;
    }
  }

  return true;
}

void
ON_CurveOnSurface::Dump( ON_TextLog& dump ) const
{
  dump.Print("ON_CurveOnSurface \n");
}

ON_BOOL32 
ON_CurveOnSurface::Write(
       ON_BinaryArchive& file // open binary file
     ) const
{
  ON_BOOL32 rc = IsValid();
  if (rc)
    rc = file.WriteObject(*m_c2);
  if (rc)
    rc = file.WriteInt( m_c3?1:0 );
  if ( rc && m_c3 )
    rc = file.WriteObject(*m_c3);
  if (rc)
    rc = file.WriteObject(*m_s);
  return rc;
}

ON_BOOL32 
ON_CurveOnSurface::Read(
       ON_BinaryArchive& file // open binary file
     )
{
  delete m_c2; 
  delete m_c3; 
  m_c2 = 0;
  m_c3 = 0;
  delete m_s;
  m_s = 0;

  ON_Object *o=0;
  ON_BOOL32 rc = file.ReadObject(&o);
  if (rc && o) {
    m_c2 = ON_Curve::Cast(o);
    if ( !m_c2 ) {
      delete o;
    } rc = false;
  }

  o = 0;

  ON_BOOL32 bHasC3 = 0;
  rc = file.ReadInt( &bHasC3 );
  if ( rc && bHasC3 ) {
    if (rc)
      rc = file.ReadObject(&o);
    if ( rc && o ) {
      m_c2 = ON_Curve::Cast(o);
      if ( !m_c2 ) {
        delete o;
      } rc = false;
    }
  }

  o = 0;

  if (rc)
    rc = file.ReadObject(&o);
  if (rc&&o) {
    m_s = ON_Surface::Cast(o);
    if ( !m_s ) {
      delete o;
      rc = false;
    }
  }

  return rc;
}

int 
ON_CurveOnSurface::Dimension() const
{
  return ( m_s ) ? m_s->Dimension() : false;
}

ON_BOOL32 
ON_CurveOnSurface::GetBBox( // returns true if successful
         double* boxmin,    // minimum
         double* boxmax,    // maximum
         ON_BOOL32 bGrowBox
         ) const
{
  return ( m_s ) ? m_s->GetBBox(boxmin,boxmax,bGrowBox) : false;
}

ON_BOOL32
ON_CurveOnSurface::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
	DestroyCurveTree();
  return ( m_s ) ? m_s->Transform(xform) : false;
}

ON_BOOL32
ON_CurveOnSurface::SwapCoordinates( int i, int j )
{
  return ( m_s ) ? m_s->SwapCoordinates(i,j) : false;
}

ON_Interval ON_CurveOnSurface::Domain() const
{
  ON_Interval d;
  if ( m_c2 )
    d = m_c2->Domain();
  return d;
}

int ON_CurveOnSurface::SpanCount() const
{
  return m_c2 ? m_c2->SpanCount() : 0;
}

ON_BOOL32 ON_CurveOnSurface::GetSpanVector( // span "knots" 
       double* s // array of length SpanCount() + 1 
       ) const
{
  return m_c2 ? m_c2->GetSpanVector(s) : false;
}

int ON_CurveOnSurface::Degree() const
{
  return m_c2 ? m_c2->Degree() : 0;
}

ON_BOOL32 
ON_CurveOnSurface::GetParameterTolerance(
         double t,  // t = parameter in domain
         double* tminus, // tminus
         double* tplus   // tplus
         ) const
{
  return (m_c2) ? m_c2->GetParameterTolerance(t,tminus,tplus) : false;
}


ON_BOOL32
ON_CurveOnSurface::IsLinear( // true if curve locus is a line segment
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = (m_c2&&ON_PlaneSurface::Cast(m_s)) ? (ON_PlaneSurface::Cast(m_s) && m_c2->IsLinear(tolerance)) : false;
  if ( rc ) {
    // TODO: rc = m_s->IsPlanar(tolerance)
  }
  return rc;
}

ON_BOOL32
ON_CurveOnSurface::IsArc( // true if curve locus in an arc or circle
      const ON_Plane* plane, // if not NULL, test is performed in this plane
      ON_Arc* arc,         // if not NULL and true is returned, then arc
                              // arc parameters are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  return (m_c2&&ON_PlaneSurface::Cast(m_s)) ? m_c2->IsArc(plane,arc,tolerance) : false;
}

ON_BOOL32
ON_CurveOnSurface::IsPlanar(
      ON_Plane* plane, // if not NULL and true is returned, then plane parameters
                         // are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  return ( ON_PlaneSurface::Cast(m_s) ) ? true : false;
}

ON_BOOL32
ON_CurveOnSurface::IsInPlane(
      const ON_Plane& plane, // plane to test
      double tolerance // tolerance to use when checking linearity
      ) const
{
  return false;
}

ON_BOOL32 
ON_CurveOnSurface::IsClosed() const
{
  ON_BOOL32 rc = ( m_c2 && m_s ) ? m_c2->IsClosed() : false;
  if ( !rc )
    rc = ON_Curve::IsClosed();
  return rc;
}

ON_BOOL32 
ON_CurveOnSurface::IsPeriodic() const
{
  return ( m_c2 && m_s ) ? m_c2->IsPeriodic() : false;
}

ON_BOOL32
ON_CurveOnSurface::Reverse()
{
  ON_BOOL32 rc = ( m_c2 ) ? m_c2->Reverse() : false;
  if ( rc && m_c3 ) rc = m_c3->Reverse();
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 
ON_CurveOnSurface::Evaluate( // returns false if unable to evaluate
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
  ON_3dVector c[5];
  ON_3dVector s[15], d;

  const int dim = Dimension();
  ON_BOOL32 rc = (dim > 0 && dim <= 3 ) ? true : false;
  if ( rc ) {
    int chint=0, shint[2]={0,0};
    if(hint) {
      chint = (*hint)&0xFFF;
      shint[0] = (*hint)>>16;
      shint[1] = shint[0]>>8;
      shint[0] &= 0xFF;
    }

    rc = ( m_c2&&m_s ) ? m_c2->Evaluate(t,der_count,3,c[0],side,&chint) : false;
    if (rc) {
      side = 0;
      if ( der_count>0 ) {
        if ( c[1].x >= 0.0 ) {
          side = ( c[1].y >= 0.0 ) ? 1 : 4;
        }
        else {
          side = ( c[1].y >= 0.0 ) ? 2 : 3;
        }
      }
      rc = m_s->Evaluate( c[0].x, c[0].y, der_count, 3, s[0], side, shint );
      if ( rc ) {
        if ( hint ) {
          *hint = (chint&0xFFFF) | ((shint[0]&0xFF)<<16) | ((shint[1]&0xFF)<<24);
        }

        v[0] = s[0].x;
        if ( dim > 1 ) v[1] = s[0].y;
        if ( dim > 2 ) v[2] = s[0].z;
        v += v_stride;

        if (der_count >= 1 ) {
          const double du = c[1].x;
          const double dv = c[1].y;
          d = du*s[1] + dv*s[2];
          v[0] = d.x;
          if ( dim > 1 ) v[1] = d.y;
          if ( dim > 2 ) v[2] = d.z;
          v += v_stride;
          if ( der_count >= 2 ) {
            const double ddu = c[2].x;
            const double ddv = c[2].y;
            d = ddu*s[1] + ddv*s[2] + du*du*s[3] + 2.0*du*dv*s[4] + dv*dv*s[5];
            v[0] = d.x;
            if ( dim > 1 ) v[1] = d.y;
            if ( dim > 2 ) v[2] = d.z;
            v += v_stride;
            if ( der_count >= 3 ) {
              const double dddu = c[3].x;
              const double dddv = c[3].y;
              d = dddu*s[1] + dddv*s[2]
                + 3.0*du*ddu*s[3] + 3.0*(ddu*dv + du*ddv)*s[4] + 3.0*dv*ddv*s[5]
                + du*du*du*s[6] + 3.0*du*du*dv*s[7] + 3.0*du*dv*dv*s[8] + dv*dv*dv*s[9];
              v[0] = d.x;
              if ( dim > 1 ) v[1] = d.y;
              if ( dim > 2 ) v[2] = d.z;
              v += v_stride;
              if ( der_count >= 4 ) {
                int n;
                for ( n = 4; n <= der_count; n++ ) {
                  v[0] = 0.0;
                  if ( dim > 1 ) v[1] = 0.0;
                  if ( dim > 2 ) v[2] = 0.0;
                  v += v_stride;
                  rc = false; // TODO - generic chain rule 
                }
              }
            }
          }
        }
      }
    }
  }
  return rc;
}


int 
ON_CurveOnSurface::GetNurbForm( // returns 0: unable to create NURBS representation
                 //            with desired accuracy.
                 //         1: success - returned NURBS parameterization
                 //            matches the curve's to wthe desired accuracy
                 //         2: success - returned NURBS point locus matches
                 //            the curve's to the desired accuracy but, on
                 //            the interior of the curve's domain, the 
                 //            curve's parameterization and the NURBS
                 //            parameterization may not match to the 
                 //            desired accuracy.
      ON_NurbsCurve& nurbs,
      double tolerance,  // (>=0)
      const ON_Interval* subdomain  // OPTIONAL subdomain of 2d curve
      ) const
{
  ON_ERROR("TODO - finish ON_CurveOnSurface::GetNurbForm().");
  return false;
}


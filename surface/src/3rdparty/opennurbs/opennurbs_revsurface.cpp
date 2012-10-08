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

ON_OBJECT_IMPLEMENT( ON_RevSurface, ON_Surface, "A16220D3-163B-11d4-8000-0010830122F0");


void ON_RevSurface::DestroyRuntimeCache( bool bDelete )
{
  ON_Surface::DestroyRuntimeCache(bDelete);
  if ( 0 != m_curve )
    m_curve->DestroyRuntimeCache(bDelete);
  // 15 August 2003 Dale Lear
  //    Added the call to destroy m_bbox.
  m_bbox.Destroy();
}


ON_RevSurface* ON_RevSurface::New()
{
  return new ON_RevSurface();
}

ON_RevSurface* ON_RevSurface::New( const ON_RevSurface& rev_surface )
{
  return new ON_RevSurface(rev_surface);
}

ON_RevSurface::ON_RevSurface() : m_curve(0), 
                                 m_axis( ON_origin, ON_zaxis ), 
                                 m_angle( 0.0, 2.0*ON_PI ),
                                 m_t( 0.0, 2.0*ON_PI ),
                                 m_bTransposed(0)
{
  ON__SET__THIS__PTR(m_s_ON_RevSurface_ptr);
}

ON_RevSurface::~ON_RevSurface()
{
  Destroy();
}

void ON_RevSurface::Destroy()
{
  DestroySurfaceTree();
  if ( m_curve)
  {
    delete m_curve;
    m_curve = 0;
  }
  m_axis.Create( ON_origin, ON_zaxis );
  m_angle.Set(0.0,2.0*ON_PI);
  m_t = m_angle;
  m_bTransposed = false;
  m_bbox.Destroy();
}

ON_RevSurface::ON_RevSurface( const ON_RevSurface& src ) : ON_Surface(src)
{
  ON__SET__THIS__PTR(m_s_ON_RevSurface_ptr);
  m_curve = src.m_curve ? src.m_curve->Duplicate() : NULL;
  m_axis = src.m_axis;
  m_angle = src.m_angle;
  m_t = src.m_t;
  m_bTransposed = src.m_bTransposed;
  m_bbox = src.m_bbox;
}


unsigned int ON_RevSurface::SizeOf() const
{
  unsigned int sz = ON_Surface::SizeOf();
  if ( m_curve )
    sz += m_curve->SizeOf();
  return sz;
}

ON__UINT32 ON_RevSurface::DataCRC(ON__UINT32 current_remainder) const
{
  if ( m_curve )
    current_remainder = m_curve->DataCRC(current_remainder);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_axis),&m_axis);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_angle),&m_angle);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_t),&m_t);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_bTransposed),&m_bTransposed);
  return current_remainder;
}


ON_RevSurface& ON_RevSurface::operator=( const ON_RevSurface& src )
{
  if ( this != &src ) 
  {
    Destroy();
    ON_Surface::operator=(src);
    if ( src.m_curve ) 
      m_curve = src.m_curve->Duplicate();
    m_axis = src.m_axis;
    m_angle = src.m_angle;
    m_t = src.m_t;
    m_bTransposed = src.m_bTransposed;
    m_bbox = src.m_bbox;
  }
  return *this;
}


ON_BOOL32 ON_RevSurface::SetAngleRadians(
  double start_angle_radians,
  double end_angle_radians
  )
{
  bool rc = false;
  double d = end_angle_radians-start_angle_radians;
  if ( d >= 0.0 )
  {
    if ( d <= ON_ZERO_TOLERANCE || d > 2.0*ON_PI )
    {
      end_angle_radians = start_angle_radians + 2.0*ON_PI;
    }
    m_angle.Set( start_angle_radians, end_angle_radians );
    rc = true;
    DestroySurfaceTree();
  }
  return rc;
}


ON_BOOL32 ON_RevSurface::SetAngleDegrees (
  double start_angle_degrees,
  double end_angle_degrees
  )
{
  return SetAngleRadians( 
    start_angle_degrees*ON_PI/180.0, 
    end_angle_degrees*ON_PI/180.0 
    );
}


ON_BOOL32 ON_RevSurface::IsValid( ON_TextLog* text_log ) const
{
  if ( !m_curve )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_curve is NULL.\n");
    return false;
  }
  if ( !m_curve->IsValid(text_log) )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_curve is not valid.\n");
    return false;
  }
  int dim = m_curve->Dimension();
  if ( dim != 3 )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_curve->Dimension()=%d (should be 3).\n",dim);
    return false;
  }
  if ( !m_axis.IsValid() )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_axis is not valid.\n");
    return false;
  }
  if ( !m_angle.IsIncreasing() )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_angle = (%g,%g) (should be an increasing interval)\n",
                       m_angle[0],m_angle[1]);
    return false;
  }
  double length = m_angle.Length();
  if ( length > 2.0*ON_PI + ON_ZERO_TOLERANCE )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_angle.Length() = %g (should be <= 2*pi radians).\n",length);
    return false;
  }
  if ( m_angle.Length() <= ON_ZERO_TOLERANCE )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_angle.Length() = %g (should be > ON_ZERO_TOLERANCE).\n",length);
    return false;
  }
  if ( !m_t.IsIncreasing() )
  {
    if ( text_log )
      text_log->Print( "ON_RevSurface.m_t = (%g,%g) (should be an increasing interval)\n",
                       m_t[0],m_t[1]);
    return false;
  }
  return true;
}

void ON_RevSurface::Dump( ON_TextLog& dump ) const
{
  ON_Object::Dump(dump); // print class id
  dump.PushIndent();
  if ( m_bTransposed )
    dump.Print("Paramerization: (curve,angle)\n");
  else
    dump.Print("Paramerization: (angle,curve)\n");
  dump.Print("Axis: ");
  dump.Print(m_axis.from);
  dump.Print(" to ");
  dump.Print(m_axis.to);
  dump.Print("\n");
  dump.Print("Rotation angle: %g to %g radians.\n",m_angle[0],m_angle[1]);
  dump.Print("Angle evaluation parameter interval: [%g,%g].\n",m_t[0],m_t[1]);
  if ( m_curve ) {
    dump.Print("Revolute: \n");
    dump.PushIndent();
    m_curve->Dump(dump);
    dump.PopIndent();
  }  
}

ON_BOOL32 ON_RevSurface::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(2,0);
  if (rc) 
  {
    rc = file.WriteLine( m_axis );
    rc = file.WriteInterval( m_angle );
    rc = file.WriteInterval( m_t );
    rc = file.WriteBoundingBox( m_bbox );
    rc = file.WriteInt( m_bTransposed );
    if ( m_curve ) 
    {
      rc = file.WriteChar((char)1);
      if (rc) rc = file.WriteObject(*m_curve);
    }
    else 
    {
      rc = file.WriteChar((char)0);
    }
  }
  return rc;
}


ON_BOOL32 ON_RevSurface::Read( ON_BinaryArchive& file )
{
  int major_version = 0;
  int minor_version = 0;
  char bHaveCurve = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version == 1) 
  {
    rc = file.ReadLine( m_axis );
    rc = file.ReadInterval( m_angle );
    rc = file.ReadBoundingBox( m_bbox );
    rc = file.ReadInt( &m_bTransposed );
    rc = file.ReadChar( &bHaveCurve );
    if ( bHaveCurve ) 
    {
      ON_Object* obj = 0;
      rc = file.ReadObject(&obj);
      if ( obj ) 
      {
        m_curve = ON_Curve::Cast(obj);
        if ( !m_curve )
          delete obj;
      }
    }
    m_t[0] = m_angle.Min();
    m_t[1] = m_angle.Max();
  }
  else if (rc && major_version == 2) 
  {
    rc = file.ReadLine( m_axis );
    rc = file.ReadInterval( m_angle );
    rc = file.ReadInterval( m_t );
    rc = file.ReadBoundingBox( m_bbox );
    rc = file.ReadInt( &m_bTransposed );
    rc = file.ReadChar( &bHaveCurve );
    if ( bHaveCurve ) 
    {
      ON_Object* obj = 0;
      rc = file.ReadObject(&obj);
      if ( obj ) 
      {
        m_curve = ON_Curve::Cast(obj);
        if ( !m_curve )
          delete obj;
      }
    }
  }
  return rc;
}

int ON_RevSurface::Dimension() const
{
  return 3;
}

ON_BOOL32 ON_RevSurface::Transform( const ON_Xform& xform )
{
  DestroyRuntimeCache();
  TransformUserData(xform);
  ON_BOOL32 rc = (m_curve) ? m_curve->Transform(xform) : false;
  ON_3dVector X, Y, Z;
  Z = m_axis.Tangent();
  X.PerpendicularTo( Z );
  X.Unitize();
  Y = ON_CrossProduct( Z, X );
  if ( !m_axis.Transform(xform) )
    rc = false;
  ON_3dVector transZ = m_axis.Tangent();

  if ( transZ.Length() == 0.0 )
  {
    // transformation collapsed axis
    m_axis.to = m_axis.from + Z;
  }
  else
  {
    // see if axis needs to be reversed.
    // (Happens with transformations that
    // have negative determinant - like mirroring.)
    ON_3dVector transX = xform*X;
    ON_3dVector transY = xform*Y;
    ON_3dVector transXxY = ON_CrossProduct( transX, transY );
    double d = transXxY*transZ;
    if ( d < 0.0 )
      m_axis.to = m_axis.from - m_axis.Direction();
  }

  m_bbox.Destroy();
  m_bbox = BoundingBox();
  return rc;
}

ON_BOOL32 ON_RevSurface::Evaluate( // returns false if unable to evaluate
       double s, // angle
       double t, // curve_parameter
       // evaluation parameters
       int der_count,            // number of derivatives (>=0)
       int v_stride,            // array stride (>=Dimension())
       double* v,        // array of length stride*(ndir+1)*(ndir+2)/2
       int side,        // optional - determines which quadrant to evaluate from
                       //         0 = default
                       //         1 from NE quadrant
                       //         2 from NW quadrant
                       //         3 from SW quadrant
                       //         4 from SE quadrant
       int* hint       // optional - evaluation hint (int[2]) used to speed
                       //            repeated evaluations
       ) const
{
  bool rc = false;

  double ds = 1.0;
  double x,y,z;
  int i, j, k, src_i, dst_i;
  ON_3dPoint pt;

  if ( m_bTransposed ) 
  {
    x = s; s = t; t = x;
    if ( side == 2 ) side = 4; else if ( side == 4 ) side = 2;
  }
  
  if ( m_t != m_angle )
  {
    if ( m_t.m_t[1] != m_t.m_t[0] )
    {
      ds = (m_angle.m_t[1] - m_angle.m_t[0])/(m_t.m_t[1] - m_t.m_t[0]);
      x = m_t.NormalizedParameterAt(s);
      y = m_angle.ParameterAt(x);
      s = y;
    }
  }

  
  double a = cos(s);
  double b = sin(s);
  const double ca[4] = {a, -b, -a,  b}; // cosine derivatives
  const double sa[4] = {b,  a, -b, -a}; // sine derivatives

  const int curve_dim = m_curve ? m_curve->Dimension() : 0;
  if ( curve_dim == 2 || curve_dim == 3 ) 
  {
    int curve_side = 0;
    switch(side) 
    {
    case 1:
    case 2:
      curve_side = 1;
      break;
    case 3:
    case 4:
      curve_side = -1;
      break;
    }
    rc = m_curve->Evaluate( t, der_count, v_stride, v, curve_side, hint )?true:false;
    if ( rc ) 
    {
      ON_3dVector zaxis = m_axis.Tangent();
      ON_3dVector xaxis, yaxis;
      xaxis.PerpendicularTo(zaxis);
      xaxis.Unitize();
      yaxis = ON_CrossProduct(zaxis,xaxis);

      // move curve derivatives to pure t partial spaces in v[]
      if ( curve_dim == 2 ) 
      {
        for ( i = der_count; i >= 1; i-- ) 
        {
          // move curve derivative to proper spots
          src_i = v_stride*i;
          dst_i = v_stride*((i+1)*(i+2)/2 - 1);
          v[dst_i++] = v[src_i++];
          v[dst_i++] = 0.0;
          v[dst_i]   = v[src_i];
        }
      }
      else 
      {
        for ( i = der_count; i >= 1; i-- ) 
        {
          // move curve derivative to proper spots
          src_i = v_stride*i;
          dst_i = v_stride*((i+1)*(i+2)/2 - 1);
          v[dst_i++] = v[src_i++];
          v[dst_i++] = v[src_i++];
          v[dst_i]   = v[src_i];
        }
      }

      // convert location coordinates to local frame with origin at m_axis.from
      {
        pt = ON_3dPoint(v)-m_axis.from;
        v[0] = pt*xaxis;
        v[1] = pt*yaxis;
        v[2] = pt*zaxis;
      }

      // convert curve derivative coordinates to local frame
      for ( i = 1; i <= der_count; i++ ) 
      {
        dst_i = v_stride*((i+1)*(i+2)/2 - 1);
        pt = ON_3dPoint(v+dst_i); // pt = curve derivative in world coords
        v[dst_i++] = pt*xaxis;
        v[dst_i++] = pt*yaxis;
        v[dst_i]   = pt*zaxis;
      }

      for ( i = der_count; i >= 0; i-- )
      {
        // i = total order of derivative
        double f = 1.0; // f = chain rule scale factor
        for ( j = i; j >= 0; j-- )
        {
          // j = number of partials w.r.t curve parameter
          // i-j = number of partials w.r.t angular parameter
          dst_i = v_stride*(i*(i+1)/2 + j); // 
          src_i = v_stride*((j+1)*(j+2)/2 - 1); // curve derivative
          k=(i-j)%4;
          a = f*ca[k];
          b = f*sa[k];
          f *= ds;

          // calculate derivative in local frame
          x = a*v[src_i] - b*v[src_i+1];
          y = b*v[src_i] + a*v[src_i+1];
          z = (j<i) ? 0.0 : v[src_i+2];
          // store answer in world coordinates
          pt = x*xaxis + y*yaxis + z*zaxis;
          v[dst_i++] = pt.x;
          v[dst_i++] = pt.y;
          v[dst_i]   = pt.z;
        }
      }

      // translate location
      v[0] += m_axis.from.x;
      v[1] += m_axis.from.y;
      v[2] += m_axis.from.z;

      if ( m_bTransposed ) 
      {
        for ( i = 1; i <= der_count; i++ ) 
        {
          for ( j = 0, k = i; j < k; j++, k-- ) 
          {
            dst_i = i*(i+1)/2;
            src_i = dst_i + k;
            dst_i += j;
            src_i *= v_stride;
            dst_i *= v_stride;
            x = v[src_i]; v[src_i++] = v[dst_i]; v[dst_i++] = x;
            x = v[src_i]; v[src_i++] = v[dst_i]; v[dst_i++] = x;
            x = v[src_i]; v[src_i]   = v[dst_i]; v[dst_i]   = x;
          }
        }
      }
    }
  }
  return rc;
}


class ON_RevolutionTensor : public ON_TensorProduct
{
public:
  ON_3dPoint  O;
  ON_3dVector X;
  ON_3dVector Y;
  ON_3dVector Z;
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

int ON_RevolutionTensor::DimensionA() const
{
  return 2;
}

int ON_RevolutionTensor::DimensionB() const
{
  return 3;
}

int ON_RevolutionTensor::DimensionC() const
{
  return 3;
}

bool ON_RevolutionTensor::Evaluate( double a, const double* ArcPoint, double b, const double* ShapePoint, double* SrfPoint )
{
	double x, y, z, c, s, rx, ry, A[2], B[3];

	if (a != 1.0) {
		A[0] = a*ArcPoint[0];
		A[1] = a*ArcPoint[1];
		ArcPoint = A;
	}

	if (b != 1.0) {
		B[0] = b*ShapePoint[0];
		B[1] = b*ShapePoint[1];
		B[2] = b*ShapePoint[2];
		ShapePoint = B;
	}
	

	x = (ShapePoint[0] - O.x)*X.x + (ShapePoint[1] - O.y)*X.y + (ShapePoint[2] - O.z)*X.z;
	y = (ShapePoint[0] - O.x)*Y.x + (ShapePoint[1] - O.y)*Y.y + (ShapePoint[2] - O.z)*Y.z;
	z = (ShapePoint[0] - O.x)*Z.x + (ShapePoint[1] - O.y)*Z.y + (ShapePoint[2] - O.z)*Z.z;

	c = ArcPoint[0];
	s = ArcPoint[1];

	rx = c*x - s*y;
	ry = s*x + c*y;

	SrfPoint[0] = O.x + rx*X.x + ry*Y.x + z*Z.x;
	SrfPoint[1] = O.y + rx*X.y + ry*Y.y + z*Z.y;
	SrfPoint[2] = O.z + rx*X.z + ry*Y.z + z*Z.z;

	return true;
}

int ON_RevSurface::GetNurbForm(class ON_NurbsSurface& srf , double tolerance ) const
{
  int rc = 0;
  if ( 0 != m_curve ) 
  {
    ON_NurbsCurve a, c;
    ON_Arc arc;
    arc.plane.CreateFromNormal( ON_origin, ON_zaxis );
    arc.radius = 1.0;
    arc.SetAngleRadians(m_angle[1]-m_angle[0]);
    if ( arc.GetNurbForm(a) ) 
    {
      if ( m_t.IsIncreasing() )
        a.SetDomain( m_t[0], m_t[1] );
      rc = m_curve->GetNurbForm(c,tolerance);
      if (rc) 
      {
        if ( 2 == c.m_dim )
        {
          // Increasing the dimension of a 2d curve to 3d fixes
          // was added to make the Scale1D operation in
          // bug # 103845 work.
          ON_WARNING("ON_RevSurface.m_curve is 2-dimensional.");
          c.ChangeDimension(3);
        }
        if ( 3 != c.m_dim )
        {
          ON_ERROR("ON_RevSurface.m_curve is not valid.");
          return 0;
        }

        if ( m_angle[0] != 0.0 )
        {
          c.Rotate( m_angle[0], m_axis.Direction(), m_axis.from );
        }
        ON_RevolutionTensor rho;
        rho.O = m_axis.from;
        rho.Z = m_axis.Direction();
        rho.Z.Unitize();
        rho.X.PerpendicularTo(rho.Z);
        rho.X.Unitize();
        rho.Y = ON_CrossProduct(rho.Z,rho.X);
        rho.Y.Unitize();
        if ( !srf.TensorProduct( a, c, rho ) )
        {
          // Testing for false here prevents crashes
          // when and was added as part of investigating
          // bug # 103845.  A change a few lines up
          // made it so that particular bug no longer
          // fails to create a nurbs surface.
          return 0;
        }

        // make singular points "spot on"
        ON_3dPoint C0 = c.PointAtStart();
        ON_3dPoint C1 = c.PointAtEnd();
        ON_3dPoint A0, A1;
        ON_4dPoint CV;
        double t0 = ON_UNSET_VALUE;
        double t1 = ON_UNSET_VALUE;
        if (m_axis.ClosestPointTo(C0,&t0) && ON_IsValid(t0) )
        {
          A0 = m_axis.PointAt(t0);
          if ( C0.DistanceTo(A0) <= ON_ZERO_TOLERANCE )
          {
            // SouthPole
            int j = 0;
            for ( int i = 0; i < srf.m_cv_count[0]; i++ )
            {
              CV.w = srf.Weight(i,j);
              CV.x = CV.w*A0.x;
              CV.y = CV.w*A0.y;
              CV.z = CV.w*A0.z;
              srf.SetCV(i,j,CV);
            }
          }
        }
        if (m_axis.ClosestPointTo(C1,&t1) && ON_IsValid(t1) )
        {
          A1 = m_axis.PointAt(t1);
          if ( C1.DistanceTo(A1) <= ON_ZERO_TOLERANCE )
          {
            // NorthPole
            int j = srf.m_cv_count[1]-1;
            for ( int i = 0; i < srf.m_cv_count[0]; i++ )
            {
              CV.w = srf.Weight(i,j);
              CV.x = CV.w*A1.x;
              CV.y = CV.w*A1.y;
              CV.z = CV.w*A1.z;
              srf.SetCV(i,j,CV);
            }
          }
        }

        if ( m_bTransposed )
          srf.Transpose();
      }
    }
  }
  return (rc > 0) ? 2 : 0;
}

int ON_RevSurface::HasNurbForm() const

{
  if (!IsValid())
    return 0;
  return 2;
}

bool ON_RevSurface::GetSurfaceParameterFromNurbFormParameter(
      double nurbs_s, double nurbs_t,
      double* surface_s, double* surface_t
      ) const
{
  // NOTE: overrides ON_Surface virtual function
  bool rc = (0 != m_curve);



  if ( m_bTransposed )
  {
    double* pTemp = surface_s; surface_s = surface_t; surface_t = pTemp;
    double temp = nurbs_s; nurbs_s = nurbs_t; nurbs_t = temp;
  }

  *surface_s = nurbs_s;
  *surface_t = nurbs_t;

  ON_Circle circle( ON_xy_plane, 1.0 );
  ON_Arc arc( circle, m_angle );
  ON_ArcCurve arc_curve(arc, m_t[0], m_t[1]);
  if ( !arc_curve.GetCurveParameterFromNurbFormParameter( nurbs_s, surface_s ) )
    rc = false;

  if ( m_curve )
  {
    if (!m_curve->GetCurveParameterFromNurbFormParameter( nurbs_t, surface_t ))
      rc = false;
  }


  return rc;
}

bool ON_RevSurface::GetNurbFormParameterFromSurfaceParameter(
      double surface_s, double surface_t,
      double* nurbs_s,  double* nurbs_t
      ) const
{
  // NOTE: overrides ON_Surface virtual function
  bool rc = (0 != m_curve);

  if ( m_bTransposed )
  {
    double temp = surface_s; surface_s = surface_t; surface_t = temp;
    double* pTemp = nurbs_s; nurbs_s = nurbs_t; nurbs_t = pTemp;
  }

  *nurbs_s = surface_s;
  *nurbs_t = surface_t;

  ON_Circle circle( ON_xy_plane, 1.0 );
  ON_Arc arc( circle, m_angle );
  ON_ArcCurve arc_curve(arc, m_t[0], m_t[1]);
  if ( !arc_curve.GetNurbFormParameterFromCurveParameter( surface_s, nurbs_s ) )
    rc = false;

  if ( m_curve )
  {
    if (!m_curve->GetNurbFormParameterFromCurveParameter( surface_t, nurbs_t ))
      rc = false;
  }

  return rc;
}


ON_Curve* ON_RevSurface::IsoCurve( int dir, double c ) const
{
  if ( dir < 0 || dir > 1 || !m_curve )
    return NULL;

  ON_Curve* crv = 0;
  
  if ( m_bTransposed )
    dir = 1-dir;

  if ( dir == 0 )
  {
    // 8 December 2003 Chuck - fix iso extraction bug
    //   when m_angle[0] != 0.
    ON_Circle circle;
    ON_3dPoint P = m_curve->PointAt(c);
    circle.plane.origin = m_axis.ClosestPointTo(P);
    circle.plane.zaxis = m_axis.Tangent();
    circle.plane.xaxis = P - circle.plane.origin;
    circle.radius = circle.plane.xaxis.Length();
    if ( !circle.plane.xaxis.Unitize() )
    {
      // 8 December 2003 Dale Lear - get valid zero radius
      //   arc/circle when revolute hits the axis.
      // First: try middle of revolute for x-axis
      P = m_curve->PointAt(m_curve->Domain().ParameterAt(0.5));
      ON_3dPoint Q = m_axis.ClosestPointTo(P);
      circle.plane.xaxis = P-Q;
      if ( !circle.plane.xaxis.Unitize() )
      {
        // Then: just use a vector perp to zaxis
        circle.plane.xaxis.PerpendicularTo(circle.plane.zaxis);
      }
    }
    circle.plane.yaxis = ON_CrossProduct( circle.plane.zaxis, circle.plane.xaxis );
    circle.plane.yaxis.Unitize();
    circle.plane.UpdateEquation();
    ON_Arc arc( circle, m_angle );
    crv = new ON_ArcCurve(arc, m_t[0], m_t[1]);
  }
  else if ( dir == 1 && m_curve )
  {
    crv = m_curve->DuplicateCurve();
    if ( crv )
    {
      double a = c;
      if ( m_t != m_angle )
      {
        double t = m_t.NormalizedParameterAt(c);
        a = m_angle.ParameterAt(t);
      }
      if ( a != 0.0 )
      {
        crv->Rotate( a, m_axis.Direction(), m_axis.from );
      }
    }
  }
  return crv;
}

ON_BOOL32 ON_RevSurface::Trim( int dir, const ON_Interval& domain )
{
  ON_BOOL32 rc = false;
  if ( dir != 0 && dir != 1 )
    return false;
  if ( !domain.IsIncreasing() )
    return false;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( dir == 0 )
  {
    ON_Interval dom;
    dom.Intersection(domain,m_t);
    if ( !dom.IsIncreasing() || !m_t.IsIncreasing() || !m_angle.IsIncreasing() )
      return false;
    double t0 = m_t.NormalizedParameterAt(dom[0]);
    double t1 = m_t.NormalizedParameterAt(dom[1]);
    ON_Interval a;
    a[0] = m_angle.ParameterAt(t0);
    a[1] = m_angle.ParameterAt(t1);
    double d = a.Length();
    if ( fabs(d) > ON_ZERO_TOLERANCE && fabs(d) <= 2.0*ON_PI+ON_ZERO_TOLERANCE )
    {
      m_angle = a;
      m_t = domain;
      rc = true;
    }
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->Trim( domain );
  }
  if ( rc )
  {
    // update bounding box
    ON_BoundingBox bbox0 = m_bbox;
    m_bbox.Destroy();
    BoundingBox();
    if ( m_bbox.IsValid() && bbox0.IsValid() )
      m_bbox.Intersection(bbox0);
  }
  return rc;
}

bool ON_RevSurface::Extend(
      int dir,
      const ON_Interval& domain
      )
{
  if ( dir != 0 && dir != 1 ) 
    return false;
  if (IsClosed(dir)) 
    return false;
  bool do_it = false;
  ON_Interval dom = Domain(dir);
  if (domain[0] < dom[0])
  {
    dom[0] = domain[0];
    do_it = true;
  }
  
  if (domain[1] > dom[1])
  {
    dom[1] = domain[1];
    do_it = true;
  }

  if (!do_it) 
    return false;

  if ( m_bTransposed ) 
    dir = 1-dir;

  bool rc = false;

  if ( dir == 0 )
  {
    double t0 = m_t.NormalizedParameterAt(dom[0]);
    double t1 = m_t.NormalizedParameterAt(dom[1]);
    ON_Interval a;
    a[0] = m_angle.ParameterAt(t0);
    a[1] = m_angle.ParameterAt(t1);
    if (a.Length() > 2.0*ON_PI+ON_ZERO_TOLERANCE) a[1] = a[0]+2.0*ON_PI;
    m_angle = a;
    m_t = dom;
    rc = true;
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->Extend(dom);
  }

  if ( rc )
  {
    DestroySurfaceTree();
    // update bounding box
    ON_BoundingBox bbox0 = m_bbox;
    m_bbox.Destroy();
    BoundingBox();
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::Split(
       int dir,
       double c,
       ON_Surface*& west_or_south_side,
       ON_Surface*& east_or_north_side
       ) const
{
  ON_BOOL32 rc = false;

  ON_RevSurface* srf_ws=ON_RevSurface::Cast(west_or_south_side);
  ON_RevSurface* srf_en=ON_RevSurface::Cast(east_or_north_side);
  if ( srf_ws && srf_ws == srf_en )
    return false;
  if ( west_or_south_side && !srf_ws )
    return false;
  if ( east_or_north_side && !srf_en )
    return false;

  if ( dir != 0 && dir != 1 )
    return false;
  if ( m_bTransposed )
    dir = 1-dir;

  ON_Curve* left_side = 0;
  ON_Curve* right_side = 0;
  ON_Interval left_angle, right_angle;
  ON_Interval left_t, right_t;
  left_angle = m_angle;
  right_angle = m_angle;
  left_t = m_t;
  right_t = m_t;

  if ( dir == 0 )
  {
    double t = m_t.NormalizedParameterAt(c);
    if ( m_t.Includes(c,true) && 0.0 < t && t < 1.0 )
    {
      double a = m_angle.ParameterAt( t );
      if ( m_angle.Includes(a,true) )
      {
        rc = true;

        left_angle[1] = a;
        right_angle[0] = a;
        left_t[1] = c;
        right_t[0] = c;

        if ( srf_ws == this )
          left_side = m_curve;
        else
          left_side = m_curve->Duplicate();
        if ( srf_en == this )
          right_side = m_curve;
        else
          right_side = m_curve->Duplicate();
      }
    }
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->Split( c, left_side, right_side );
    if ( rc )
    {
      if ( this == srf_ws )
      {
        delete m_curve;
        srf_ws->m_curve = left_side;
      }
      else if ( this == srf_en )
      {
        delete m_curve;
        srf_en->m_curve = right_side;
      }
    }
  }

  if ( rc )
  {
    // save input bounding box
    const ON_BoundingBox input_bbox(m_bbox);

    if ( !srf_ws )
      west_or_south_side = srf_ws = new ON_RevSurface();
    else if ( srf_ws != this && srf_ws->m_curve )
    {
      delete srf_ws->m_curve;
      srf_ws->m_curve = 0;
    }
    if ( !srf_en )
      east_or_north_side = srf_en = new ON_RevSurface();
    if ( srf_en != this && srf_en->m_curve )
    {
      delete srf_en->m_curve;
      srf_en->m_curve = 0;
    }

    srf_ws->m_axis = m_axis;
    srf_ws->m_angle = left_angle;
    srf_ws->m_t = left_t;
    srf_ws->m_bTransposed = m_bTransposed;
    srf_ws->m_curve = left_side;
    srf_ws->m_bbox.Destroy();

    srf_en->m_axis = m_axis;
    srf_en->m_angle = right_angle;
    srf_en->m_t = right_t;
    srf_en->m_bTransposed = m_bTransposed;
    srf_en->m_curve = right_side;
    srf_en->m_bbox.Destroy();

    // update bounding boxes
    // if input box was valid, intesect calculated boxes with
    // input box
    srf_ws->BoundingBox(); // calcluates srf_ws->m_bbox
    if ( srf_ws->m_bbox.IsValid() && input_bbox.IsValid() )
      srf_ws->m_bbox.Intersection(input_bbox);

    srf_en->BoundingBox(); // calcluates srf_en->m_bbox
    if ( srf_en->m_bbox.IsValid() && input_bbox.IsValid() )
      srf_en->m_bbox.Intersection(input_bbox);
  }

  return rc;
}

ON_BOOL32 ON_RevSurface::Transpose()
{
  m_bTransposed = m_bTransposed ? false : true;
  return true;
}

bool ON_RevSurface::IsContinuous(
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
  if ( m_curve )
  {
    double curve_t =  m_bTransposed ? s : t;
    rc = m_curve->IsContinuous( desired_continuity, curve_t, hint,
                                point_tolerance, d1_tolerance, d2_tolerance,
                                cos_angle_tolerance, curvature_tolerance );
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::Reverse( int dir )
{
  ON_BOOL32 rc = false;
  if ( m_bTransposed )
    dir = dir ? 0 : 1;
  if ( dir == 0 )
  {
    m_axis.Reverse();
    double a0 = m_angle[0];
    double a1 = m_angle[1];
    m_angle.Set( 2.0*ON_PI - a1, 2.0*ON_PI - a0 );
    m_t.Reverse();
    rc = true;
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->Reverse();
  }
  return rc;
}

bool ON_RevSurface::GetNextDiscontinuity( 
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
  if ( (m_bTransposed ? 1 : 0) == dir )
  {
    // angle direction
    ON_Circle circle(ON_xy_plane,1.0);
    ON_Arc arc( circle, m_angle );
    ON_ArcCurve arc_curve(arc, m_t[0], m_t[1]);
    rc = arc_curve.GetNextDiscontinuity(
      c,
      t0,t1,t,
      (hint? &hint[dir] : 0),
      dtype,cos_angle_tolerance,
      curvature_tolerance);
  }
  else
  {
    rc = m_curve->GetNextDiscontinuity(
      c,
      t0,t1,t,
      (hint? &hint[dir] : 0),
      dtype,cos_angle_tolerance,
      curvature_tolerance);
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::IsSingular( int side ) const // 0 = south, 1 = east, 2 = north, 3 = west
{
  bool rc = false;
  ON_3dPoint P, Q;
  //double d, tol;
  if ( side < 0 || side > 3 )
    return false;

  if ( m_bTransposed )
  {
    switch(side)
    {
    case 0: side = 3; break;
    case 1: side = 2; break;
    case 2: side = 1; break;
    case 3: side = 0; break;
    }
  }

  if ( 0 == side || 2 == side )
  {
    P = (0 == side)
      ? m_curve->PointAtStart()
      : m_curve->PointAtEnd();
    Q = m_axis.ClosestPointTo(P);

    // 26 Feb 2003 Dale Lear
    //
    //     I changed the code to handle cases when the
    //     "Q" has some coordinates that are small and
    //     other coordinates that are very large.  The
    //     fabs(Q.*)*ON_SQRT_EPSILON term is required
    //     in cases where the evaluations in PointAtStart()
    //     and/or ClosestPointTo() have more numerical 
    //     error than ON_ZERO_TOLERANCE.  The numerical
    //     tolerance term has to be calculated on a
    //     coordinate-by-coordinate basis.  See RR 9683.
    //
    // old test:
    //d = P.DistanceTo(Q);
    //if ( d <= ON_ZERO_TOLERANCE )
    //  rc = true;

    // 12 July 2012 Dale Lear
    //   Use ON_PointsAreCoincident() for all IsSingular queries
    //
    //d = fabs(P.x - Q.x);
    //tol = ON_ZERO_TOLERANCE + fabs(Q.x)*ON_SQRT_EPSILON;
    //if ( d <= tol )
    //{
    //  d = fabs(P.y - Q.y);
    //  tol = ON_ZERO_TOLERANCE + fabs(Q.y)*ON_SQRT_EPSILON;
    //  if ( d <= tol )
    //  {
    //    d = fabs(P.z - Q.z);
    //    tol = ON_ZERO_TOLERANCE + fabs(Q.z)*ON_SQRT_EPSILON;
    //    if ( d <= tol )
    //      rc = true;
    //  }
    //}
    rc = ON_PointsAreCoincident(3,0,&P.x,&Q.x);

  }

  return rc;
}

ON_BOOL32 ON_RevSurface::IsPeriodic( int dir ) const // dir  0 = "s", 1 = "t"
{
  ON_BOOL32 rc = false;
  if ( m_bTransposed )
    dir = dir ? 0 : 1;
  if ( dir == 0 )
  {
    if (m_angle.Length() >= 2.0*ON_PI-ON_ZERO_TOLERANCE )
      rc = true;
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->IsPeriodic();
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::IsPlanar(
      ON_Plane* plane,
      double tolerance
      ) const
{
  ON_BOOL32 rc = false;
  if( IsValid()){
    ON_Plane AxisNormal( m_curve->PointAtStart(), m_axis.Tangent() );
    rc = m_curve->IsInPlane( AxisNormal, tolerance);
    if(rc && plane)
      *plane = AxisNormal;
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::IsClosed( int dir ) const  // dir  0 = "s", 1 = "t"
{
  ON_BOOL32 rc = false;
  if ( m_bTransposed )
    dir = dir ? 0 : 1;
  if ( dir == 0 )
  {
    if (m_angle.Length() >= 2.0*ON_PI-ON_ZERO_TOLERANCE )
      rc = true;
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->IsClosed();
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
         int dir,        // 0 gets first parameter, 1 gets second parameter
         double t,       // t = parameter in domain
         double* tminus, // tminus
         double* tplus   // tplus
         ) const
{
  ON_BOOL32 rc = false;
  if ( m_bTransposed )
    dir = dir ? 0 : 1;
  if ( dir == 0 )
  {
    if ( m_t.IsIncreasing() )
      rc = ON_GetParameterTolerance( m_t[0], m_t[1], t, tminus, tplus );
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->GetParameterTolerance( t, tminus, tplus );
  }
  return rc;
}

ON_BOOL32 ON_RevSurface::SetDomain( 
  int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
  double t0, 
  double t1
  )
{
  bool rc = false;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( dir == 0 )
  {
    if ( t0 < t1 )
    {
      m_t.Set(t0,t1);
      DestroyRuntimeCache();
      rc = true;
    }
  }
  else if ( dir == 1 && m_curve )
  {
    rc = m_curve->SetDomain( t0, t1 ) ? true : false;
    DestroyRuntimeCache();
  }
  return rc;
}

ON_Interval ON_RevSurface::Domain( int dir ) const
{
  ON_Interval d;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( dir == 0 )
  {
    d = m_t;
  }
  else if ( dir == 1 && m_curve )
  {
    d = m_curve->Domain();
  }
  return d;
}

ON_BOOL32 ON_RevSurface::GetSurfaceSize( 
    double* width, 
    double* height 
    ) const
{
  ON_BOOL32 rc = false;
  if ( m_bTransposed )
  {
    double* ptr = width;
    width = height;
    height = ptr;
  }
  if ( m_curve )
  {
    rc = true;

    ON_Interval cdom = m_curve->Domain();
    int i, hint = 0;
    int imax = 64;
    double d = 1.0/((double)imax);
    ON_3dPoint pt0 = ON_UNSET_POINT;
    ON_3dPoint pt;
    double length_estimate = 0.0;

    if ( width != NULL || height != NULL )
    {
      double radius_estimate = 0.0;
      double r;
      for ( i = 0; i <= imax; i++ )
      {
        if ( m_curve->EvPoint( cdom.ParameterAt(i*d), pt, 0, &hint ) )
        {
          r = m_axis.DistanceTo(pt);
          if ( r > radius_estimate )
            radius_estimate = r;
          if ( pt0 != ON_UNSET_POINT )
            length_estimate += pt0.DistanceTo(pt);
          pt0 = pt;
        }
      }
      if ( width != NULL )
        *width = m_angle.Length()*radius_estimate;
    }

    if ( height != NULL )
    {
      *height = length_estimate;
    }
  }
  else
  {
    if ( width )
      *width = 0.0;
    if ( height )
      *height = 0.0;
  }
  return rc;
}

int ON_RevSurface::SpanCount( int dir ) const
{
  int span_count = 0;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( dir==0 && m_t.IsIncreasing() )
  {
	  double a = (0.5 + ON_SQRT_EPSILON)*ON_PI;
    double da = fabs(m_angle.Length());
	  if (da <= a)
		  span_count = 1;
	  else if (da <= 2.0*a)
		  span_count = 2;
	  else
		  span_count = 4;
  }
  else if ( dir == 1 && m_curve )
    span_count = m_curve->SpanCount();
  return span_count;
}


ON_BOOL32 ON_RevSurface::GetSpanVector( int dir, double* s ) const
{
  ON_BOOL32 rc = false;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( dir==0 && m_t.IsIncreasing() ) {
    int span_count = SpanCount(m_bTransposed?1-dir:dir);
    if ( span_count > 0 )
    {
      double d = 1.0/span_count;
      s[0] = m_t[0];
      for ( int i = 1; i < span_count; i++ )
        s[i] = m_t.ParameterAt( i*d );
      s[span_count] = m_t[1];
      rc = true;
    }
  }
  else if ( dir==1 && m_curve ) {
    rc = m_curve->GetSpanVector(s);
  }
  return rc;
}


int ON_RevSurface::Degree( int dir ) const
{
  int span_degree = 0;
  if ( m_bTransposed )
    dir = 1-dir;
  if ( dir == 0  )
    span_degree = 2;
  else if ( dir == 1 && m_curve )
    span_degree = m_curve->Degree();
  return span_degree;
}

void ON_RevSurface::ClearBoundingBox()
{
  m_bbox.Destroy();
}

ON_BOOL32 ON_RevSurface::GetBBox(    // returns true if successful
       double* boxmin,  // boxmin[dim]
       double* boxmax,  // boxmax[dim]
       ON_BOOL32 bGrowBox    // true means grow box
       ) const
{
  ON_BOOL32 rc = m_bbox.IsValid();

  if ( !rc )
  {
    ON_BoundingBox bbox, cbox, abox;
    rc = m_curve->GetBoundingBox( cbox );
    if (rc)
    {
      //Dec 16 2010 - Chuck - if the angle range does not include 0, m_curve is not part of the surface.
      //bbox = cbox;

      ON_3dPointArray corners;
      cbox.GetCorners(corners);
      ON_3dPoint P;
      ON_Arc arc;
      arc.plane.zaxis = m_axis.Tangent();
      arc.SetAngleRadians(m_angle[1] - m_angle[0]);
      int i;
      double t;
      for ( i = 0; i < 8; i++ )
      {
        P = corners[i];
        abox.Set(P,false);
        while( m_axis.ClosestPointTo(P,&t) ) // not a loop - used for flow control
        {
          abox.Set(arc.plane.origin,true);
          // If we cannot construct a valid arc, then P and the point on the axis
          // are added to the bounding box.  One case where this happens is when
          // P is on the axis of revolution.  See bug 84354.

          arc.plane.origin = m_axis.PointAt(t);
          arc.plane.xaxis = P-arc.plane.origin;
          arc.radius = arc.plane.xaxis.Length();
          if ( !arc.plane.xaxis.Unitize() )
            break;
          if ( fabs(arc.plane.xaxis*arc.plane.zaxis) > 0.0001 )
            break; 
          arc.plane.yaxis = ON_CrossProduct(arc.plane.zaxis,arc.plane.xaxis);
          if ( !arc.plane.yaxis.Unitize() )
            break;
          arc.plane.UpdateEquation();
          arc.plane.Rotate( m_angle[0], arc.plane.zaxis );
          if ( !arc.IsValid() )
            break;
          abox = arc.BoundingBox();
          break;
        }
        bbox.Union(abox);
      }

      if ( bbox.IsValid() )
      {
        ON_RevSurface* ptr = const_cast<ON_RevSurface*>(this);
        ptr->m_bbox = bbox;
        rc = true;
      }
    }
  }

  if ( rc )
  {
    if ( boxmin )
    {
      if (bGrowBox){
        if (m_bbox.m_min.x < boxmin[0]) boxmin[0] = m_bbox.m_min.x;
        if (m_bbox.m_min.y < boxmin[1]) boxmin[1] = m_bbox.m_min.y;
        if (m_bbox.m_min.z < boxmin[2]) boxmin[2] = m_bbox.m_min.z;
      }
      else {
        boxmin[0] = m_bbox.m_min.x;
        boxmin[1] = m_bbox.m_min.y;
        boxmin[2] = m_bbox.m_min.z;
      }
    }
    if ( boxmax )
    {
      if (bGrowBox){
        if (m_bbox.m_max.x > boxmax[0]) boxmax[0] = m_bbox.m_max.x;
        if (m_bbox.m_max.y > boxmax[1]) boxmax[1] = m_bbox.m_max.y;
        if (m_bbox.m_max.z > boxmax[2]) boxmax[2] = m_bbox.m_max.z;
      }
      else {
        boxmax[0] = m_bbox.m_max.x;
        boxmax[1] = m_bbox.m_max.y;
        boxmax[2] = m_bbox.m_max.z;
      }
    }
  }

  return rc;
}

ON_BOOL32 ON_RevSurface::IsSpherical(ON_Sphere* sphere, double tolerance ) const
{
  ON_BOOL32 rc = false;
  if ( m_curve )
  {
    ON_Plane plane;
    ON_Arc arc;
    ON_3dPoint P = m_curve->PointAt( m_curve->Domain().Mid() );
    plane.origin = m_axis.from;
    plane.yaxis = m_axis.Tangent();
    plane.zaxis = ON_CrossProduct( P-plane.origin, plane.yaxis );
    plane.zaxis.Unitize();
    plane.xaxis = ON_CrossProduct( plane.yaxis, plane.zaxis );
    plane.UpdateEquation();
    if ( plane.IsValid() )
    {
      if ( m_curve->IsArc( &plane, &arc, tolerance ) )
      {
        P = m_axis.ClosestPointTo( arc.Center() );
        if ( P.DistanceTo(arc.Center()) <= tolerance )
        {
          rc = true;
          if ( sphere )
          {
            sphere->plane.origin = arc.Center();
            sphere->plane.zaxis = m_axis.Tangent();
            sphere->plane.yaxis = arc.plane.zaxis;
            sphere->plane.xaxis = ON_CrossProduct( sphere->plane.zaxis, sphere->plane.yaxis );
            sphere->plane.UpdateEquation();
            sphere->radius = arc.radius;
          }
        }
      }
    }
  }
  return rc;
}


bool ON_Surface::IsSphere( ON_Sphere* sphere, double tolerance ) const
{
  if ( !ON_IsValid(tolerance) || tolerance <= 0.0 )
    tolerance = ON_ZERO_TOLERANCE;
  const ON_RevSurface* rs = ON_RevSurface::Cast(this);
  if (rs)
  {
    return rs->IsSpherical(sphere,tolerance) ? true : false;
  }

  ON_Curve* crv = IsoCurve(0,Domain(1).Mid());
  if ( !crv )
    return false;

  ON_Arc arc0;
  int bIsArc0 = crv->IsArc(0,&arc0,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
  delete crv;
  crv = 0;
  if ( !bIsArc0 )
    return false;

  crv = IsoCurve(1,Domain(0).Mid());
  if ( !crv )
    return false;
  ON_Arc arc1;
  int bIsArc1 = crv->IsArc(0,&arc1,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
  delete crv;
  crv = 0;
  if ( !bIsArc1 )
    return false;

  // Determine if one of these arcs is a a portion of a 
  // great circle.

  ON_Sphere sph0;
  sph0.plane = arc0.plane;
  sph0.radius = arc0.radius;
  bool bTestSphere0 = sph0.IsValid();

  ON_Sphere sph1;
  sph1.plane = arc1.plane;
  sph1.radius = arc1.radius;
  bool bTestSphere1 = sph1.IsValid();

  if ( !bTestSphere0 && !bTestSphere1 )
    return false;

  double sph0tol = 0.0;
  double sph1tol = 0.0;

  double tol = 0.5*ON_SQRT_EPSILON*(arc0.radius+arc1.radius);

  ON_3dPoint P, S;
  double a, d;
  for ( a = 0.0; a < 1.0; a += 0.25 )
  {
    P = arc0.PointAt(a*2.0*ON_PI);
    if ( bTestSphere0 )
    {
      S = sph0.ClosestPointTo(P);
      d = S.DistanceTo(P);
      if ( d > tol )
      {
        bTestSphere0 = false;
        if ( !bTestSphere1 )
          return false;
      }
      else if ( d > sph0tol )
        sph0tol = d;
    }
    if ( bTestSphere1 )
    {
      S = sph1.ClosestPointTo(P);
      d = S.DistanceTo(P);
      if ( d > tol )
      {
        bTestSphere1 = false;
        if ( !bTestSphere0 )
          return false;
      }
      else if ( d > sph1tol )
        sph1tol = d;
    }

    P = arc1.PointAt(a*2.0*ON_PI);
    if ( bTestSphere0 )
    {
      S = sph0.ClosestPointTo(P);
      d = S.DistanceTo(P);
      if ( d > tol )
      {
        bTestSphere0 = false;
        if ( !bTestSphere1 )
          return false;
      }
      else if ( d > sph0tol )
        sph0tol = d;
    }
    if ( bTestSphere1 )
    {
      S = sph1.ClosestPointTo(P);
      d = S.DistanceTo(P);
      if ( d > tol )
      {
        bTestSphere1 = false;
        if ( !bTestSphere0 )
          return false;
      }
      else if ( d > sph1tol )
        sph1tol = d;
    }
  }
  // If the arc's are both great circles, then
  // both will be true unless we have a bug or
  // numerical issues.
  if (!bTestSphere0 && !bTestSphere1)
    return false;

  if ( tol < tolerance )
    tol = tolerance;

  double u, v;
  int sc0 = SpanCount(0);
  int sc1 = SpanCount(1);
  double* s = (double*)onmalloc( (sc0+sc1+2)*sizeof(s[0]) );
  double* t = s + (sc0+1);
  GetSpanVector(0,s);
  GetSpanVector(1,t);
  for ( int i = 0; i < sc0; i++ )
  {
    for ( int ii = i?1:0; ii <= 4; ii++ )
    {
      u = 0.25*((4-ii)*s[i] + ii*s[i+1]);
      for ( int j = 0; j <= sc1; j++ )
      {
        for ( int jj = j?1:0; jj <= 4; jj++ )
        {
          v = 0.25*((4-jj)*t[j] + jj*t[j+1]);
          P = PointAt(u,v);
          if ( bTestSphere0 )
          {
            S = sph0.ClosestPointTo(P);
            d = S.DistanceTo(P);
            if ( d > tol )
            {
              bTestSphere0 = false;
              if ( !bTestSphere1 )
              {
                onfree(s);
                return false;
              }
            }
            else if ( d > sph0tol )
              sph0tol = d;
          }
          if ( bTestSphere1 )
          {
            S = sph1.ClosestPointTo(P);
            d = S.DistanceTo(P);
            if ( d > tol )
            {
              bTestSphere1 = false;
              if ( !bTestSphere0 )
              {
                onfree(s);
                return false;
              }
            }
            else if ( d > sph1tol )
              sph1tol = d;
          }
        }
      }
    }
  }
  onfree(s);

  bool rc = (bTestSphere0 || bTestSphere1);
  if ( rc && sphere )
  {
    if (!bTestSphere0)
      *sphere = sph1;
    else if (!bTestSphere1)
      *sphere = sph0;
    else if (sph0tol <= sph1tol)
      *sphere = sph0;
    else
      *sphere = sph1;
  }

  return rc;
}

bool ON_Surface::IsCylinder( ON_Cylinder* cylinder, double tolerance ) const
{
  if ( !ON_IsValid(tolerance) || tolerance <= 0.0 )
    tolerance = ON_ZERO_TOLERANCE;
  const ON_RevSurface* rs = ON_RevSurface::Cast(this);
  bool rc = rs && rs->IsCylindrical(cylinder,tolerance);

  if ( !rc && !rs )
  {
    ON_Curve* crv = IsoCurve(0,Domain(1).Mid());
    if ( !crv )
      return false;

    ON_Arc arc;
    ON_Line line;
    int bIsLine = 0;
    int bIsArc = crv->IsArc(0,&arc,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
    if ( !bIsArc )
    {
      bIsLine = crv->IsLinear(tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
      if ( bIsLine )
      {
        line.from = crv->PointAtStart();
        line.to = crv->PointAtEnd();
      }
    }
    delete crv;
    crv = 0;
    if ( !bIsArc && !bIsLine )
      return false;

    crv = IsoCurve(1,Domain(0).Mid());
    if ( !crv )
      return false;
    if ( !bIsArc )
      bIsArc = crv->IsArc(0,&arc,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
    else if ( !bIsLine )
    {
      bIsLine = crv->IsLinear(tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
      if ( bIsLine )
      {
        line.from = crv->PointAtStart();
        line.to = crv->PointAtEnd();
      }
    }
    delete crv;
    crv = 0;
    if ( !bIsArc || !bIsLine )
      return false;

    double tol = 0.5*ON_SQRT_EPSILON*(arc.radius);
    if ( tol < tolerance )
      tol = tolerance;
    double r = arc.plane.origin.DistanceTo(arc.plane.ClosestPointTo(line.from));
    if ( fabs(arc.radius - r) > tol )
      return false;
    r = arc.plane.origin.DistanceTo(arc.plane.ClosestPointTo(line.to));
    if ( fabs(arc.radius - r) > tol )
      return false;

    ON_3dPoint P;
    double u, v;
    int sc0 = SpanCount(0);
    int sc1 = SpanCount(1);
    double* s = (double*)onmalloc( (sc0+sc1+2)*sizeof(s[0]) );
    double* t = s + (sc0+1);
    GetSpanVector(0,s);
    GetSpanVector(1,t);
    for ( int i = 0; i < sc0; i++ )
    {
      for ( int ii = i?1:0; ii <= 4; ii++ )
      {
        u = 0.25*((4-ii)*s[i] + ii*s[i+1]);
        for ( int j = 0; j < sc1; j++ )
        {
          for ( int jj = j?1:0; jj <= 4; jj++ )
          {
            v = 0.25*((4-jj)*t[j] + jj*t[j+1]);
            P = PointAt(u,v);
            r = arc.plane.origin.DistanceTo(arc.plane.ClosestPointTo(P));
            if ( fabs(arc.radius - r) > tol )
            {
              onfree(s);
              return false;
            }
          }
        }
      }
    }
    onfree(s);


    rc = true;
    if ( cylinder )
    {
      cylinder->Create(arc);
      rc = cylinder->IsValid();
    }
  }

  return rc;
}


bool ON_Surface::IsCone( ON_Cone* cone, double tolerance ) const
{
  if ( !ON_IsValid(tolerance) || tolerance <= 0.0 )
    tolerance = ON_ZERO_TOLERANCE;

  const ON_RevSurface* rs = ON_RevSurface::Cast(this);
  if (rs)
  {
    return rs->IsConical(cone,tolerance) ? true : false;
  }


  ON_Curve* crv = IsoCurve(0,Domain(1).Mid());
  if ( !crv )
    return false;

  ON_Arc arc;
  ON_Line line;
  int bIsLine = 0;
  int bIsArc = crv->IsArc(0,&arc,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
  if ( !bIsArc )
  {
    bIsLine = crv->IsLinear(tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
    if ( bIsLine )
    {
      line.from = crv->PointAtStart();
      line.to = crv->PointAtEnd();
    }
  }
  delete crv;
  crv = 0;
  if ( !bIsArc && !bIsLine )
    return false;

  crv = IsoCurve(1,Domain(0).Mid());
  if ( !crv )
    return false;
  if ( !bIsArc )
    bIsArc = crv->IsArc(0,&arc,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
  else if ( !bIsLine )
  {
    bIsLine = crv->IsLinear(tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
    if ( bIsLine )
    {
      line.from = crv->PointAtStart();
      line.to = crv->PointAtEnd();
    }
  }
  delete crv;
  crv = 0;
  if ( !bIsArc || !bIsLine )
    return false;

  double r0 = arc.plane.origin.DistanceTo(arc.plane.ClosestPointTo(line.from));
  double r1 = arc.plane.origin.DistanceTo(arc.plane.ClosestPointTo(line.to));
  if ( fabs(r0-r1) <= ON_ZERO_TOLERANCE )
    return false;
  double h0 = arc.plane.plane_equation.ValueAt(line.from);
  double h1 = arc.plane.plane_equation.ValueAt(line.to);
  if ( fabs(h0-h1) <= ON_ZERO_TOLERANCE )
    return false;
  double tol = 0.5*ON_SQRT_EPSILON*(r0+r1);

  ON_Cone cn;
  cn.height = (h0*r1 - r0*h1)/(r1-r0);
  if ( !ON_IsValid(cn.height) || fabs(cn.height) <= ON_ZERO_TOLERANCE )
    return false;
  cn.plane = arc.plane;
  cn.plane.origin = cn.plane.origin + cn.height*cn.plane.zaxis;
  cn.plane.UpdateEquation();
  if ( r0 >= r1 )
  {
    cn.radius = r0;
    cn.height = h0-cn.height;
  }
  else
  {
    cn.radius = r1;
    cn.height = h1-cn.height;
  }
  if ( !cn.IsValid() )
    return false;
  tol *= fabs(cn.height);

  // if (r - h*cn.radius/cn.height > tolerance) return false;
  double h = cn.plane.plane_equation.ValueAt(line.from);
  if ( fabs(r0*cn.height - h*cn.radius) > tol )
    return false;
  h = cn.plane.plane_equation.ValueAt(line.to);
  if ( fabs(r1*cn.height - h*cn.radius) > tol )
    return false;

  ON_3dPoint P;
  double u, v, r;
  int sc0 = SpanCount(0);
  int sc1 = SpanCount(1);
  double* s = (double*)onmalloc( (sc0+sc1+2)*sizeof(s[0]) );
  double* t = s + (sc0+1);
  GetSpanVector(0,s);
  GetSpanVector(1,t);
  tol = 0.5*ON_SQRT_EPSILON*(r0+r1);
  if ( tol < tolerance )
    tol = tolerance;
  tol *= fabs(cn.height);
  for ( int i = 0; i < sc0; i++ )
  {
    for ( int ii = i?1:0; ii <= 4; ii++ )
    {
      u = 0.25*((4-ii)*s[i] + ii*s[i+1]);
      for ( int j = 0; j < sc1; j++ )
      {
        for ( int jj = j?1:0; jj <= 4; jj++ )
        {
          v = 0.25*((4-jj)*t[j] + jj*t[j+1]);
          P = PointAt(u,v);
          h = cn.plane.plane_equation.ValueAt(P);
          r = cn.plane.origin.DistanceTo(cn.plane.ClosestPointTo(P));
          // if (r - h*cn.radius/cn.height > tolerance) return false;
          if ( fabs(r*cn.height - h*cn.radius) > tol )
          {
            onfree(s);
            return false;
          }
        }
      }
    }
  }
  onfree(s);

  if ( cone )
  {
    *cone = cn;
  }

  return true;
}


bool ON_Surface::IsTorus( ON_Torus* torus, double tolerance ) const
{
  if ( !ON_IsValid(tolerance) || tolerance <= 0.0 )
    tolerance = ON_ZERO_TOLERANCE;

  ON_Curve* crv = IsoCurve(0,Domain(1).Mid());
  if ( !crv )
    return false;

  ON_Arc arc0;
  int bIsArc0 = crv->IsArc(0,&arc0,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
  delete crv;
  crv = 0;
  if ( !bIsArc0 )
    return false;

  crv = IsoCurve(1,Domain(0).Mid());
  if ( !crv )
    return false;
  ON_Arc arc1;
  int bIsArc1 = crv->IsArc(0,&arc1,tolerance > ON_ZERO_TOLERANCE ? tolerance : 0.0);
  delete crv;
  crv = 0;
  if ( !bIsArc1 )
    return false;

  double tol = 0.5*ON_SQRT_EPSILON*(arc0.radius+arc1.radius);

  ON_Torus tr0;
  tr0.plane = arc0.plane;
  double h = tr0.plane.plane_equation.ValueAt(arc1.plane.origin);
  tr0.plane.origin = tr0.plane.origin + h*tr0.plane.zaxis;
  tr0.plane.UpdateEquation();
  tr0.major_radius = tr0.plane.origin.DistanceTo(arc1.plane.origin);
  tr0.minor_radius = arc1.radius;

  ON_Torus tr1;
  tr1.plane = arc1.plane;
  h = tr1.plane.plane_equation.ValueAt(arc0.plane.origin);
  tr1.plane.origin = tr1.plane.origin + h*tr1.plane.zaxis;
  tr1.plane.UpdateEquation();
  tr1.major_radius = tr1.plane.origin.DistanceTo(arc0.plane.origin);
  tr1.minor_radius = arc0.radius;

  bool bTestTorus0 = tr0.IsValid()?true:false;
  bool bTestTorus1 = tr1.IsValid()?true:false;
  if ( !bTestTorus0 && !bTestTorus1 )
    return false;
  double tr0tol = 0.0;
  double tr1tol = 0.0;

  ON_3dPoint P, T;
  double a, d;
  for ( a = 0.0; a < 1.0; a += 0.25 )
  {
    P = arc0.PointAt(a*2.0*ON_PI);
    if ( bTestTorus0 )
    {
      T = tr0.ClosestPointTo(P);
      d = T.DistanceTo(P);
      if ( d > tol )
      {
        bTestTorus0 = false;
        if ( !bTestTorus1 )
          return false;
      }
      else if ( d > tr0tol )
        tr0tol = d;
    }
    if ( bTestTorus1 )
    {
      T = tr1.ClosestPointTo(P);
      d = T.DistanceTo(P);
      if ( d > tol )
      {
        bTestTorus1 = false;
        if ( !bTestTorus0 )
          return false;
      }
      else if ( d > tr1tol )
        tr1tol = d;
    }

    P = arc1.PointAt(a*2.0*ON_PI);
    if ( bTestTorus0 )
    {
      T = tr0.ClosestPointTo(P);
      d = T.DistanceTo(P);
      if ( d > tol )
      {
        bTestTorus0 = false;
        if ( !bTestTorus1 )
          return false;
      }
      else if ( d > tr0tol )
        tr0tol = d;
    }
    if ( bTestTorus1 )
    {
      T = tr1.ClosestPointTo(P);
      d = T.DistanceTo(P);
      if ( d > tol )
      {
        bTestTorus1 = false;
        if ( !bTestTorus0 )
          return false;
      }
      else if ( d > tr1tol )
        tr1tol = d;
    }
  }
  // If the arc's planes are perpendicular, then
  // both will be true unless we have a bug or
  // numerical issues.
  if (!bTestTorus0 && !bTestTorus1)
    return false;

  if ( tol < tolerance )
    tol = tolerance;

  double u, v;
  int sc0 = SpanCount(0);
  int sc1 = SpanCount(1);
  double* s = (double*)onmalloc( (sc0+sc1+2)*sizeof(s[0]) );
  double* t = s + (sc0+1);
  GetSpanVector(0,s);
  GetSpanVector(1,t);
  for ( int i = 0; i < sc0; i++ )
  {
    for ( int ii = i?1:0; ii <= 4; ii++ )
    {
      u = 0.25*((4-ii)*s[i] + ii*s[i+1]);
      for ( int j = 0; j < sc1; j++ )
      {
        for ( int jj = j?1:0; jj <= 4; jj++ )
        {
          v = 0.25*((4-jj)*t[j] + jj*t[j+1]);
          P = PointAt(u,v);
          if ( bTestTorus0 )
          {
            T = tr0.ClosestPointTo(P);
            d = T.DistanceTo(P);
            if ( d > tol )
            {
              bTestTorus0 = false;
              if ( !bTestTorus1 )
              {
                onfree(s);
                return false;
              }
            }
            else if ( d > tr0tol )
              tr0tol = d;
          }
          if ( bTestTorus1 )
          {
            T = tr1.ClosestPointTo(P);
            d = T.DistanceTo(P);
            if ( d > tol )
            {
              bTestTorus1 = false;
              if ( !bTestTorus0 )
              {
                onfree(s);
                return false;
              }
            }
            else if ( d > tr1tol )
              tr1tol = d;
          }
        }
      }
    }
  }
  onfree(s);

  bool rc = (bTestTorus0 || bTestTorus1);
  if ( rc && torus )
  {
    if (!bTestTorus0)
      *torus = tr1;
    else if (!bTestTorus1)
      *torus = tr0;
    else if (tr0tol <= tr1tol)
      *torus = tr0;
    else
      *torus = tr1;
  }

  return rc;
}

static
bool ON__IsCylConeHelper(
          const ON_Line& axis, 
          const ON_Curve* curve,
          double tolerance,
          ON_Plane& plane,
          ON_Line& line,
          double r[2],
          double& h
          )
{
  if ( !axis.IsValid() )
    return false;
  if ( !curve )
    return false;
  line.from = curve->PointAtStart();
  line.to   = curve->PointAtEnd();
  if ( !line.IsValid() )
    return false;
  if ( line.Length() <= ON_ZERO_TOLERANCE )
    return false;

  plane.zaxis = axis.Tangent();
  h = plane.zaxis*line.Direction();
  if ( !ON_IsValid(h) )
    return false;
  if ( h < 0.0 )
  {
    plane.zaxis.Reverse();
    h = -h;
  }
  if ( h <= ON_ZERO_TOLERANCE )
    return false;

  double a0 = ON_UNSET_VALUE;
  if ( !axis.ClosestPointTo(line.from,&a0) )
    return false;
  double a1 = ON_UNSET_VALUE;
  if ( !axis.ClosestPointTo(line.to,&a1) )
    return false;
  if ( !ON_IsValid(a0) || !ON_IsValid(a1) )
    return false;


  ON_3dPoint A0 = axis.PointAt(a0);
  ON_3dPoint A1 = axis.PointAt(a1);
  plane.origin = A0;
  ON_3dVector x0 = line.from - A0;
  ON_3dVector x1 = line.to   - A1;
  r[0] = x0.Length();
  r[1] = x1.Length();
  if ( x0*x0 < 0.0 && r[0] > ON_ZERO_TOLERANCE && r[1] > ON_ZERO_TOLERANCE )
    return false;
  plane.xaxis = ( r[0] >= r[1] ) ? x0 : x1;
  if (fabs(plane.xaxis.Length()) <= ON_ZERO_TOLERANCE)
    return false;
  if ( !plane.xaxis.Unitize() )
    return false;
  plane.yaxis = ON_CrossProduct(plane.zaxis,plane.xaxis);
  if ( !plane.yaxis.Unitize() )
    return false;
  plane.UpdateEquation();
  if ( !plane.IsValid() )
    return false;

  x0 = line.Tangent();
  if ( fabs(x0*plane.yaxis) > ON_ZERO_TOLERANCE )
    return false;

  return (curve->IsLinear( tolerance )?true:false);
}

ON_BOOL32 ON_RevSurface::IsCylindrical(
      ON_Cylinder* cylinder,
      double tolerance
      ) const
{
  ON_Cylinder c;
  ON_Line line;
  double r[2] = {0.0,0.0};
  double h = 0.0;
  if ( !ON_IsValid(tolerance) || tolerance <= 0.0 )
    tolerance = ON_ZERO_TOLERANCE;
  if ( !ON__IsCylConeHelper(m_axis,m_curve,tolerance,c.circle.plane,line,r,h) )
    return false;
  if ( fabs(r[0]-r[1]) > tolerance )
    return false;
  if ( fabs(line.Tangent()*c.circle.plane.xaxis) > ON_ZERO_TOLERANCE )
    return false;
  c.circle.radius = (r[0]==r[1]) ? r[0] : (0.5*(r[0]+r[1]));
  c.height[0] = 0.0;
  c.height[1] = h;
  if ( cylinder )
    *cylinder = c;
  return c.IsValid();
}

ON_BOOL32 ON_RevSurface::IsConical(
      ON_Cone* cone,
      double tolerance
      ) const
{
  ON_Cone c;
  ON_Line line;
  double r[2] = {0.0,0.0};
  double h = 0.0;
  if ( !ON_IsValid(tolerance) || tolerance <= 0.0 )
    tolerance = ON_ZERO_TOLERANCE;
  if ( !ON__IsCylConeHelper(m_axis,m_curve,tolerance,c.plane,line,r,h) )
    return false;
  double dr = r[0]-r[1];
  if ( fabs(dr) <= ON_ZERO_TOLERANCE )
    return false;
  if ( 0.0 == r[0] )
  {
    c.radius = r[1];
    c.height = h;
  }
  else if ( 0.0 == r[1] )
  {
    c.plane.origin += h*c.plane.zaxis;
    c.plane.UpdateEquation();
    c.radius = r[0];
    c.height = -h;
  }
  else if ( dr > 0.0 )
  {
    h *= (r[0]/dr);
    c.plane.origin += h*c.plane.zaxis;
    c.plane.UpdateEquation();
    c.radius = r[0];
    c.height = -h;
  }
  else
  {
    double d = h*r[0]/dr;
    c.plane.origin += d*c.plane.zaxis;
    c.plane.UpdateEquation();
    c.radius = r[1];
    c.height = h-d;
  }

  if ( cone )
    *cone = c;
  return c.IsValid();
}


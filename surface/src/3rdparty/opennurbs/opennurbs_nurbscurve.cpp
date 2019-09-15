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
#include <pcl/pcl_macros.h>

ON_OBJECT_IMPLEMENT(ON_NurbsCurve,ON_Curve,"4ED7D4DD-E947-11d3-BFE5-0010830122F0");

ON_NurbsCurve* ON_NurbsCurve::New()
{
  return new ON_NurbsCurve();
}

ON_NurbsCurve* ON_NurbsCurve::New(
          const ON_NurbsCurve& nurbs_curve 
          )
{
  return new ON_NurbsCurve(nurbs_curve);
}

ON_NurbsCurve* ON_NurbsCurve::New(
          const ON_BezierCurve& bezier_curve 
          )
{
  return new ON_NurbsCurve(bezier_curve);
}

ON_NurbsCurve* ON_NurbsCurve::New(
          int dimension,
          ON_BOOL32 bIsRational,
          int order,
          int cv_count
          )
{
  return new ON_NurbsCurve(dimension,bIsRational,order,cv_count);
}


ON_NurbsCurve::ON_NurbsCurve()
{
  ON__SET__THIS__PTR(m_s_ON_NurbsCurve_ptr);
  Initialize();
}

ON_NurbsCurve::ON_NurbsCurve( const ON_NurbsCurve& src ) : ON_Curve(src)
{
  ON__SET__THIS__PTR(m_s_ON_NurbsCurve_ptr);
  Initialize();
  *this = src;
}

ON_NurbsCurve::ON_NurbsCurve( int dim, ON_BOOL32 bIsRational, int order, int cv_count )
{
  ON__SET__THIS__PTR(m_s_ON_NurbsCurve_ptr);
  Initialize();
  Create(dim,bIsRational,order,cv_count);
}

ON_NurbsCurve::ON_NurbsCurve(const ON_BezierCurve& src)
{
  ON__SET__THIS__PTR(m_s_ON_NurbsCurve_ptr);
  Initialize();
  ON_NurbsCurve::operator=(src);
}

ON_NurbsCurve::~ON_NurbsCurve()
{
  Destroy();
}

unsigned int ON_NurbsCurve::SizeOf() const
{
  unsigned int sz = ON_Curve::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Curve));
  sz += m_knot_capacity*sizeof(*m_knot);
  sz += m_cv_capacity*sizeof(*m_cv);
  return sz;
}

ON__UINT32 ON_NurbsCurve::DataCRC(ON__UINT32 current_remainder) const
{
  current_remainder = ON_CRC32(current_remainder,sizeof(m_dim),&m_dim);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_is_rat),&m_is_rat);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_order),&m_order);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_cv_count),&m_cv_count);
  if ( m_cv_count > 0 && m_cv_stride > 0 && m_cv )
  {
    std::size_t sizeof_cv = CVSize()*sizeof(m_cv[0]);
    const double* cv = m_cv;
    int i;
    for ( i = 0; i < m_cv_count; i++ )
    {
      current_remainder = ON_CRC32(current_remainder,sizeof_cv,cv);
      cv += m_cv_stride;
    }
  }
  current_remainder = ON_CRC32(current_remainder,KnotCount()*sizeof(m_knot[0]),m_knot);
  return current_remainder;
}

int ON_NurbsCurve::Dimension() const
{
  return m_dim;
}

bool ON_NurbsCurve::IsRational() const
{
  return m_is_rat?true:false;
}

int ON_NurbsCurve::CVSize() const
{
  return (m_dim>0) ? (m_is_rat?(m_dim+1):m_dim) : 0;
}

int ON_NurbsCurve::Order( void ) const
{
  return m_order;
}

int ON_NurbsCurve::CVCount( void ) const
{
  return m_cv_count;
}

int ON_NurbsCurve::KnotCount( void ) const
{
  return ON_KnotCount( m_order, m_cv_count );
}

double* ON_NurbsCurve::CV( int i ) const
{
  return (m_cv) ? (m_cv + i*m_cv_stride) : NULL;
}

ON::point_style ON_NurbsCurve::CVStyle() const
{
  return m_is_rat ? ON::homogeneous_rational : ON::not_rational;
}


double ON_NurbsCurve::Weight( int i ) const
{
  return (m_cv && m_is_rat) ? m_cv[i*m_cv_stride+m_dim] : 1.0;
}


double ON_NurbsCurve::Knot( int i ) const
{
  return (m_knot) ? m_knot[i] : 0.0;
}

int ON_NurbsCurve::KnotMultiplicity( int i ) const
{
  return ON_KnotMultiplicity(m_order,m_cv_count,m_knot,i);
}

const double* ON_NurbsCurve::Knot() const
{
  return m_knot;
}

double ON_NurbsCurve::SuperfluousKnot( int end ) const
{
  return(m_knot) ? ON_SuperfluousKnot(m_order,m_cv_count,m_knot,end) : 0.0;
}


bool ON_NurbsCurve::MakePeriodicUniformKnotVector( double delta )
{
	DestroyCurveTree();
  ReserveKnotCapacity( ON_KnotCount( m_order, m_cv_count ) );
  return ON_MakePeriodicUniformKnotVector( m_order, m_cv_count, m_knot, delta );
}


bool ON_NurbsCurve::MakeClampedUniformKnotVector( double delta )
{
	DestroyCurveTree();
  ReserveKnotCapacity( ON_KnotCount( m_order, m_cv_count ) );
  return ON_MakeClampedUniformKnotVector( m_order, m_cv_count, m_knot, delta );
}

bool ON_NurbsCurve::CreateClampedUniformNurbs( 
        int dimension,
        int order,
        int point_count,
        const ON_3dPoint* point,
        double knot_delta
        )
{
  bool rc = (dimension >= 1 && dimension<=3 && point!=NULL);
  if (rc)
    rc = Create( dimension, false, order, point_count );
  if ( rc )
  {
    int i;
    for (i = 0; i < point_count; i++)
      SetCV( i, ON::intrinsic_point_style, point[i] );
  }
  if ( rc )
    rc = MakeClampedUniformKnotVector( knot_delta );
  return rc;
}

bool ON_NurbsCurve::CreatePeriodicUniformNurbs( 
        int dimension,
        int order,
        int point_count,
        const ON_3dPoint* point,
        double knot_delta
        )
{
  bool rc = (dimension >= 1 && dimension<=3 && point!=NULL);
  if (rc)
    rc = Create( dimension, false, order, point_count+(order-1) );
  if ( rc )
  {
    int i;
    for (i = 0; i < point_count; i++)
      SetCV( i, ON::intrinsic_point_style, point[i] );
    for (i = 0; i <= order-2; i++)
      SetCV( m_cv_count-m_order+1+i, ON::intrinsic_point_style, CV(i) );
  }
  if ( rc )
    rc = MakePeriodicUniformKnotVector( knot_delta );
  return rc;
}

bool ON_NurbsCurve::Create(
        int dim,      // dimension (>= 1)
        ON_BOOL32 is_rat,  // true to make a rational NURBS
        int order,    // order (>= 2)
        int cv_count  // cv count (>= order)
        )
{
  DestroyCurveTree();
  if ( dim < 1 )
    return false;
  if ( order < 2 )
    return false;
  if ( cv_count < order )
    return false;
  m_dim = dim;
  m_is_rat = (is_rat) ? true : false;
  m_order = order;
  m_cv_count = cv_count;
  m_cv_stride = (m_is_rat) ? m_dim+1 : m_dim;
  bool rc = ReserveKnotCapacity( KnotCount() );
  if ( !ReserveCVCapacity( CVCount()*m_cv_stride ) )
    rc = false;
  return rc;
}

void ON_NurbsCurve::Destroy()
{
  DestroyCurveTree();
  double* cv = ( m_cv && m_cv_capacity ) ? m_cv : NULL;
  double* knot = ( m_knot && m_knot_capacity ) ? m_knot : NULL;
  Initialize();
  if ( cv )
    onfree(cv);
  if ( knot )
    onfree(knot);
}

void ON_NurbsCurve::EmergencyDestroy()
{
  Initialize();
}


void ON_NurbsCurve::Initialize()
{
  m_dim = 0;
  m_is_rat = 0;
  m_order = 0;
  m_cv_count = 0;
  m_knot_capacity = 0;
  m_knot = 0;
  m_cv_stride = 0;
  m_cv_capacity = 0;
  m_cv = 0;
}

static void ON_NurbsCurveCopyHelper( const ON_NurbsCurve& src, ON_NurbsCurve& dest )
{
  dest.DestroyCurveTree();
  dest.m_dim       = src.m_dim;
  dest.m_is_rat    = src.m_is_rat;
  dest.m_order     = src.m_order;
  dest.m_cv_count  = src.m_cv_count;
  dest.m_cv_stride = dest.m_is_rat ? dest.m_dim+1 : dest.m_dim;
  if ( src.m_knot ) 
  {
    // copy knot array
    dest.ReserveKnotCapacity( dest.KnotCount() );
    memcpy( dest.m_knot, src.m_knot, dest.KnotCount()*sizeof(*dest.m_knot) );
  }
  if ( src.m_cv ) 
  {
    // copy cv array
    dest.ReserveCVCapacity( dest.m_cv_stride*dest.m_cv_count );
    const int dst_cv_size = dest.CVSize()*sizeof(*dest.m_cv);
    const int src_stride = src.m_cv_stride;
    const int dst_stride = dest.m_cv_stride;
    const double *src_cv = src.CV(0);
    double *dst_cv = dest.m_cv;
    if ( src_stride == dst_stride ) 
    {
      memcpy( dst_cv, src_cv, dest.m_cv_count*dst_stride*sizeof(*dst_cv) );
    }
    else 
    {
      int i;
      for ( i = 0; i < dest.m_cv_count; i++ )
      {
        memcpy( dst_cv, src_cv, dst_cv_size );
        dst_cv += dst_stride;
        src_cv += src_stride;
      }
    }
  }
}

ON_NurbsCurve& ON_NurbsCurve::operator=( const ON_NurbsCurve& src )
{
  if ( this != &src ) 
  {
    ON_Curve::operator=(src);
    ON_NurbsCurveCopyHelper(src,*this);
  }
  return *this;
}


ON_NurbsCurve& ON_NurbsCurve::operator=(const ON_BezierCurve& src)
{
  int i;
  Create(src.m_dim,src.m_is_rat,src.m_order,src.m_order);
  const int sizeof_cv = src.CVSize()*sizeof(double);
  for ( i = 0; i < m_cv_count; i++ ) {
    memcpy( CV(i), src.CV(i), sizeof_cv );
  }
  for ( i = 0; i <= m_order-2; i++ )
    m_knot[i] = 0.0;
  const int knot_count = KnotCount();
  for ( i = m_order-1; i < knot_count; i++ )
    m_knot[i] = 1.0;
  return *this;
}

void ON_NurbsCurve::Dump( ON_TextLog& dump ) const
{
  dump.Print( "ON_NurbsCurve dim = %d is_rat = %d\n"
               "        order = %d cv_count = %d\n",
               m_dim, m_is_rat, m_order, m_cv_count );
  dump.Print( "Knot Vector ( %d knots )\n", KnotCount() );
  dump.PrintKnotVector( m_order, m_cv_count, m_knot );
  dump.Print( "Control Points  %d %s points\n"
               "  index               value\n",
               m_cv_count, 
               (m_is_rat) ? "rational" : "non-rational" );
  if ( !m_cv ) {
    dump.Print("  NULL cv array\n");
  }
  else {
    dump.PrintPointList( m_dim, m_is_rat, 
                      m_cv_count, m_cv_stride,
                      m_cv, 
                      "  CV" );
  }
}


static bool ON_NurbsCurveIsNotValid()
{
  return ON_IsNotValid(); // <-- good place for a breakpoint
}

static bool ON_ControlPointsAreNotValid()
{
  return ON_IsNotValid(); // <-- good place for a breakpoint
}

static bool ON_ControlPointsAreValid( int cv_size, int cv_count, int cv_stride, const double* cv, ON_TextLog* text_log )
{
  int i, j;
  if ( 0 == cv  )
  {
    if ( text_log )
    {
      text_log->Print("cv pointer is null.\n");
    }
    return ON_ControlPointsAreNotValid();
  }

  if ( cv_count < 2 )
  {
    if ( text_log )
    {
      text_log->Print("cv_count = %d (must be >= 2).\n",cv_count);
    }
    return ON_ControlPointsAreNotValid();
  }

  if ( cv_size < 1 )
  {
    if ( text_log )
    {
      text_log->Print("cv_size = %d (must be >= 1).\n",cv_size);
    }
    return ON_ControlPointsAreNotValid();
  }

  if ( cv_stride < cv_size )
  {
    if ( text_log )
    {
      text_log->Print("cv_stride = %d and cv_size = %d (cv_stride must be >= cv_size).\n",cv_stride,cv_size);
    }
    return ON_ControlPointsAreNotValid();
  }

  for ( i = 0; i < cv_count; i++, cv += cv_stride )
  {
    for ( j = 0; j < cv_size; j++ )
    {
      if ( !ON_IsValid(cv[j]) )
      {
        if ( text_log )
        {
          text_log->Print("cv[%d*cv_stride + %d] = %g is not valid.\n",i,j,cv[j]);
        }
        return ON_ControlPointsAreNotValid();
      }
    }    
  }

  return true;
}

ON_BOOL32 ON_NurbsCurve::IsValid( ON_TextLog* text_log ) const
{
  if ( m_dim <= 0 )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_dim = %d (should be > 0).\n",m_dim);
    }
    return ON_NurbsCurveIsNotValid();
  }
  
  if (m_order < 2 )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_order = %d (should be >= 2).\n",m_order);
    }
    return ON_NurbsCurveIsNotValid();
  }
  
  if (m_cv_count < m_order )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_cv_count = %d (should be >= m_order=%d).\n",m_cv_count,m_order);
    }
    return ON_NurbsCurveIsNotValid();
  }
  
  if (m_cv_stride < CVSize() )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_cv_stride = %d (should be >= %d).\n",m_cv_stride,CVSize());
    }
  }
  
  if (m_cv == NULL)
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_cv is NULL.\n");
    }
    return ON_NurbsCurveIsNotValid();
  }
  
  if (m_knot == NULL)
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_knot is NULL.\n");
    }
    return ON_NurbsCurveIsNotValid();
  }

  if ( !ON_IsValidKnotVector( m_order, m_cv_count, m_knot, text_log ) ) 
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_knot[] is not a valid knot vector.\n");
    }
    return ON_NurbsCurveIsNotValid();
  }

  if ( !ON_ControlPointsAreValid(CVSize(),m_cv_count,m_cv_stride,m_cv,text_log) )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsCurve.m_cv[] is not valid.\n");
    }
    return ON_NurbsCurveIsNotValid();
  }

  if ( m_is_rat )
  {
    // weights at fully multiple knots must be nonzero
    // partial test for weight function being zero
    double sign = 0.0;
    const double* w = &m_cv[m_dim];
    int zcount = 0;
    int i;
    for ( i = 0; i < m_cv_count; i++, w += m_cv_stride )
    {
      if ( *w == 0.0 )
        zcount++;
      else
        zcount = 0;

      if ( zcount >= m_order )
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsCurve.m_cv has zero weights for CV[%d],...,CV[%d].\n",i-m_order+1,i);
        }
        return ON_NurbsCurveIsNotValid(); // denominator is zero for entire span
      }
      
      if ( m_knot[i] == m_knot[i+m_order-2] ) 
      {
        if ( *w == 0.0 )
        {
          if ( text_log )
          {
            text_log->Print("ON_NurbsCurve.m_cv has zero weights for CV[%d].\n",i);
          }
          return ON_NurbsCurveIsNotValid();
        }
        
        if (sign == 0.0) 
        {
          sign = (*w > 0.0) ? 1.0 : -1.0;
        }
        else if ( *w * sign <= 0.0 ) 
        {
          if ( text_log )
          {
            text_log->Print("ON_NurbsCurve.m_cv has a zero denominator in the parameter interval [%g,%g].\n",
                            m_knot[i-1],m_knot[i]);
          }
          return ON_NurbsCurveIsNotValid();
        }
      }
    }

    if ( m_dim <= 3 && 2 == m_order && 2 == m_cv_count )
    {
      // fix for RR 21239
      // 16 November 2010 Chuck and Dale Lear added m_dim <= 3
      // so the 3d checking does not interfer with applications
      // that use high dimension curves where the start and end
      // points can be arbitrary.
      ON_3dPoint P0 = PointAtStart();
      ON_3dPoint P1 = PointAtEnd();
      if ( P0 == P1 )
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsCurve is a line with no length.\n");
        }
        return ON_NurbsCurveIsNotValid();
      }
    }
  }

  return true;
}

ON_BOOL32 ON_NurbsCurve::GetBBox( // returns true if successful
       double* boxmin,    // minimum
       double* boxmax,    // maximum
       ON_BOOL32 bGrowBox  // true means grow box
       ) const
{
  return ON_GetPointListBoundingBox( 
                 m_dim, m_is_rat, m_cv_count, 
                 m_cv_stride, m_cv, 
                 boxmin, boxmax, bGrowBox?true:false
                 );
}

ON_BOOL32 ON_NurbsCurve::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
	DestroyCurveTree();
  if ( 0 == m_is_rat )
  {
    if ( xform.m_xform[3][0] != 0.0 || xform.m_xform[3][1] != 0.0 || xform.m_xform[3][2] != 0.0 )
    {
      MakeRational();
    }
  }
  return ON_TransformPointList( m_dim, m_is_rat, m_cv_count, m_cv_stride, m_cv, xform );
}

bool ON_NurbsCurve::IsDeformable() const
{
  return true;
}

bool ON_NurbsCurve::MakeDeformable()
{
  return true;
}

bool ON_PolylineCurve::IsDeformable() const
{
  return true;
}

bool ON_PolylineCurve::MakeDeformable()
{
  return true;
}


ON_BOOL32 ON_NurbsCurve::Write(
       ON_BinaryArchive& file // open binary file
     ) const
{
  // NOTE - check legacy I/O code if changed
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if (rc)
  {
    if (rc) rc = file.WriteInt( m_dim );
    if (rc) rc = file.WriteInt( m_is_rat );
    if (rc) rc = file.WriteInt( m_order );
    if (rc) rc = file.WriteInt( m_cv_count );
    if (rc) rc = file.WriteInt( 0 ); // reserved - legacy flag values
    if (rc) rc = file.WriteInt( 0 ); // reserved   
    
    // write invalid bounding box - may be used in future
    if (rc) 
    {
      ON_BoundingBox bbox;
      rc = file.WriteBoundingBox(bbox);
    }

    // write knots
    int count = (0 != m_knot && m_cv_count >= m_order && m_order >= 2)
              ? KnotCount() 
              : 0;
    if (rc) rc = file.WriteInt(count);
    if (rc) rc = file.WriteDouble( count, m_knot );

    // write cvs
    const int cv_size = CVSize();
    count = ( m_cv && cv_size > 0 && m_cv_count > 0  && m_cv_stride >= cv_size ) 
          ? m_cv_count
          : 0;
    if (rc) rc = file.WriteInt(count);
    if (rc && count > 0 )
    {
      int i;
      for ( i = 0; i < m_cv_count && rc; i++ )
      {
        rc = file.WriteDouble( cv_size, CV(i) );
      }
    }

  }
  return rc;
}

ON_BOOL32 ON_NurbsCurve::Read(
       ON_BinaryArchive& file // open binary file
     )
{
  // NOTE - check legacy I/O code if changed
  Destroy();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1)
  {
    // common to all 1.x versions
    int dim = 0, is_rat = 0, order = 0, cv_count = 0;
    int reserved1 = 0, reserved2 = 0;
    if (rc) rc = file.ReadInt( &dim );
    if (rc) rc = file.ReadInt( &is_rat );
    if (rc) rc = file.ReadInt( &order );
    if ( order < 0 )
      rc = false;
    if (rc) rc = file.ReadInt( &cv_count );
    if ( cv_count < order )
      rc = false;
    if (rc) rc = file.ReadInt( &reserved1 ); // reserved - legacy flag values
    if (rc) rc = file.ReadInt( &reserved2 ); // reserved    

    // bounding box
    if (rc) {
      // write invalid bounding box - may be used in future
      ON_BoundingBox bbox;
      rc = file.ReadBoundingBox( bbox );
    }
    
    if ( !Create( dim, is_rat, order, cv_count ) )
      rc = false;

    // read knots
    int count = 0;
    if (rc) rc = file.ReadInt(&count);
    if ( count < 0 )
      rc = false; // fix crash bug 92841
    else if ( 0 != count && count != ON_KnotCount(order,cv_count) )
      rc = false;
    if (rc ) rc = ReserveKnotCapacity(count);
    if (rc) rc = file.ReadDouble( count, m_knot );

    // read cvs
    count = 0;
    if (rc) rc = file.ReadInt(&count);
    const int cv_size = CVSize();
    if (rc) rc = ReserveCVCapacity( count*cv_size );
    if (count > 0 && cv_size > 0 && rc )
    {
      int i;
      for ( i = 0; i < m_cv_count && rc; i++ )
      {
        rc = file.ReadDouble( cv_size, CV(i) );
      }
    }
  }
  if ( !rc )
    Destroy();
  return rc;
}

ON_Interval ON_NurbsCurve::Domain() const
{
  ON_Interval d;
  if ( !ON_GetKnotVectorDomain( m_order, m_cv_count, m_knot, &d.m_t[0], &d.m_t[1] ) )
    d.Destroy();
  return d;
}

ON_BOOL32 ON_NurbsCurve::SetDomain( double t0, double t1 )
{
  ON_BOOL32 rc = false;
  if ( m_order >= 2 && m_cv_count >= m_order && m_knot && t0 < t1 ) {
	 //DestroyCurveTree();
   const double k0 = m_knot[m_order-2];
    const double k1 = m_knot[m_cv_count-1];
    if ( k0 == t0 && k1 == t1 )
      rc = true;
    else if ( k0 < k1 ) 
    {
      DestroyCurveTree();
      const double d = (t1-t0)/(k1-k0);
      const double km = 0.5*(k0+k1);
      const int knot_count = KnotCount();
      int i;
      for ( i = 0; i < knot_count; i++ ) 
      {
        if ( m_knot[i] <= km ) {
          m_knot[i] = (m_knot[i] - k0)*d + t0;
        }
        else {
          m_knot[i] = (m_knot[i] - k1)*d + t1;
        }
      }
      rc = true;
    }
  }
  return rc;
}

ON_BOOL32 ON_NurbsCurve::ChangeClosedCurveSeam( double t )
{
  ON_BOOL32 rc = IsClosed();
  if ( rc )
  {
    //ON_BOOL32 bIsPeriodic = IsPeriodic();
    const ON_Interval old_dom = Domain();
    double k = t;
    double s = old_dom.NormalizedParameterAt(t);
    if ( s < 0.0 || s > 1.0 )
    {
      s = fmod( s, 1.0 );
      if ( s < 0.0 )
        s += 1.0;
      k = old_dom.ParameterAt(s);
    }
		s = old_dom.NormalizedParameterAt( k);
    if ( old_dom.Includes(k,true) )
    {
      ON_NurbsCurve left, right;
      // TODO - if periodic - dont' put multi knot in middle
      //if ( IsPeriodic() ){} else
      {
        ON_Curve* pleft = &left;
        ON_Curve* pright = &right;
        rc = Split( k, pleft, pright );
        if ( rc )
        {
          //int a = left.IsValid();
          //int b = right.IsValid();
          right.Append(left);
          *this = right;
        }
      }
    }
    else
    {
      // k already at start/end of domain
      rc = true;
    }
    if (rc)
      SetDomain( t, t+old_dom.Length() );
  }
  return rc;
}


int ON_NurbsCurve::SpanCount() const
{
  // number of smooth spans in curve
  return ON_KnotVectorSpanCount( m_order, m_cv_count, m_knot );
}

ON_BOOL32 ON_NurbsCurve::GetSpanVector( // span "knots" 
       double* s // array of length SpanCount() + 1 
       ) const
{
  return ON_GetKnotVectorSpanVector( m_order, m_cv_count, m_knot, s );
}

int ON_NurbsCurve::Degree() const
{
  return m_order >= 2 ? m_order-1 : 0;
}


// true if curve locus is a line segment
// tolerance to use when checking linearity
ON_BOOL32
ON_NurbsCurve::IsLinear(
      double tolerance
      ) const
{
  // 9 March 2009 Dale Lear
  //   Speed up IsLinear() test to accelerate loading curves into Rhino.
  //

  if ( !IsClamped() )
    return false;

  // IsClamped returning true insures that 2 <= order <= cv_count
  // and that the knot vector is clamped.

  ON_Line L;
  ON_3dPoint P, Q;
  ON_3dVector V0, V1, D;
  double t0, t, d, s;
  int i;

  // When m_cv is a null pointer, the GetCV() fails.
  if ( !GetCV(0,L.from) )
    return false;
  if ( !GetCV(m_cv_count-1,L.to) )
    return false;

  // if CV coordinates are NaNs, infinities or ON_UNSET_VALUE,
  // then "t" will end failing the ON_IsValid() test.
  D.x = L.to.x - L.from.x; D.y = L.to.y - L.from.y; D.z = L.to.z - L.from.z; 
  t = D.x*D.x + D.y*D.y + D.z*D.z;
  if ( !ON_IsValid(t) || !(t > 0.0) )
    return false;

  if ( 2 == m_cv_count )
    return true;

  t0 = 0.0;
  bool bDefaultTolerance = false;
  if ( !(tolerance > 0.0) )
  {
    bDefaultTolerance = true;
    tolerance = ON_ZERO_TOLERANCE;
  }
  const double tol2 = tolerance*tolerance;
  t = 1.0/t;
  D.x *= t; D.y *= t; D.z *= t;

  for ( i = 1; i < m_cv_count-1; i++ )
  {
    // This call to GetCV will return true because earlier calls to
    // GetCV(0,...) and GetCV(m_cv_count-1,...) worked
    GetCV(i,P); 

    // Set t = parameter of point on line L closest to P
    V0.x = P.x - L.from.x; V0.y = P.y - L.from.y; V0.z = P.z - L.from.z;
    V1.x = P.x - L.to.x;   V1.y = P.y - L.to.y;   V1.z = P.z - L.to.z;
    if ( V0.x*V0.x + V0.y*V0.y + V0.z*V0.z <= V1.x*V1.x + V1.y*V1.y + V1.z*V1.z )
    {
      // P is closest to L.from
      t = V0.x*D.x + V0.y*D.y + V0.z*D.z;
      if ( t < -0.01 )
        return false;
    }
    else 
    {
      // P is closest to L.to
      t = 1.0 + V1.x*D.x + V1.y*D.y + V1.z*D.z;
      if ( t > 1.01 )
        return false;
    }

    // set Q = L.PointAt(t)
    s = 1.0-t;
    Q.x = s*L.from.x + t*L.to.x; Q.y = s*L.from.y + t*L.to.y; Q.z = s*L.from.z + t*L.to.z;

    if ( bDefaultTolerance )
    {
      if ( !ON_PointsAreCoincident(3,0,&P.x,&Q.x) )
        return false;
    }
    else
    {
      // verify that the distance from P to Q is <= tolerance
      s = P.x-Q.x;
      d = tol2 - s*s;
      if ( d < 0.0 )
        return false;
      s = P.y-Q.y;
      d -= s*s;
      if ( d < 0.0 )
        return false;
      s = P.z-Q.z;
      d -= s*s;
      if ( d < 0.0 )
        return false;
    }

    if ( t > t0 && t0 < 1.0 )
    {
      t0 = (t < 1.0) ? t : 1.0;
    }

    if ( !(t >= t0 && t <= 1.0) )
    {
      // 14 Dec 2004
      //  Chuck and Dale Lear added this test to weed out
      //  garbage curves whose locus is a line but that self intersect.
      //  For example, circles that have been projected onto a plane 
      //  perpendicular to their axis are garbage and are not "linear".

      // either a (nearly) stacked control point or
      // the curve has reversed direction        
      P = L.PointAt(t0);
      d = Q.DistanceTo(P);
      if ( !(d <= tolerance) )
        return false; // curve has reversed direction ("garbage")
    }
  }

  return true;
}

int ON_NurbsCurve::IsPolyline(
      ON_SimpleArray<ON_3dPoint>* pline_points,
      ON_SimpleArray<double>* pline_t
      ) const
{
  int i;
  int rc = 0;
  if ( pline_points )
    pline_points->SetCount(0);
  if ( pline_t )
    pline_t->SetCount(0);
  if ( IsValid() )
  {
    if ( 2 == m_order )
    {
      rc = m_cv_count;
      if ( pline_points )
      {
        pline_points->Reserve(m_cv_count);
        for ( i = 0; i < m_cv_count; i++ )
        {
          GetCV(i,pline_points->AppendNew());
        }
      }
      if ( pline_t )
      {
        pline_t->Reserve(m_cv_count);
        for ( i = 0; i < m_cv_count; i++ )
          pline_t->Append(m_knot[i]);
      }
    }
    else if ( m_order > 2 && m_dim >= 2 && m_dim <= 3 )
    {
      // 16 April 2003 Dale Lear:
      //     Added to so the high degree polylines get
      //     flagged as polylines.
      const double *k = m_knot;
      double t;
      int i, j, span_count = m_cv_count-m_order+1;
      ON_Line line;
      ON_3dPoint P, Q;
      GetCV(0,line.to);
      for ( i = 0; i < span_count; i++, k++ )
      {
        if ( k[m_order-2] < k[m_order-1] )
        {
          if ( k[0] != k[m_order-2] )
          {
            // not a bezier span
            return 0;
          }
          if ( k[m_order-1] != k[2*m_order-3] )
          {
            // not a bezier span
            return 0;
          }

          // see if this bezier segment is a line
          // with (roughly) linear parameterization.
          line.from = line.to;
          GetCV(i+m_order-1,line.to);
          for ( j = 1; j < m_order-1; j++ )
          {
            GetCV(i+j,P);
            t = 0.0;
            if ( !line.ClosestPointTo(P,&t) )
              return 0;
            if ( fabs( t*(m_order-1) - j ) > 0.01 )
            {
              // not linearly parameterized.
              // Please check with Dale Lear before changing
              // this test.
              return 0;
            }
            Q = line.PointAt(t);
            // Please check with Dale Lear before changing
            // this test.
            if ( fabs(P.x - Q.x) > (fabs(P.x) + fabs(Q.x))*0.00001 + ON_ZERO_TOLERANCE )
              return 0;
            if ( fabs(P.y - Q.y) > (fabs(P.y) + fabs(Q.y))*0.00001 + ON_ZERO_TOLERANCE )
              return 0;
            if ( fabs(P.z - Q.z) > (fabs(P.z) + fabs(Q.z))*0.00001 + ON_ZERO_TOLERANCE )
              return 0;
          }
          rc++;
        }
      }

      if (rc > 0 )
      {
        rc++;
        // it's a polyline
        if ( 0 != pline_points || 0 != pline_t )
        {
          GetCV(0,P);
          if ( 0 != pline_points )
          {
            pline_points->Reserve(rc);
            GetCV(0,pline_points->AppendNew());
          }
          if ( 0 != pline_t )
          {
            pline_t->Reserve(rc);
            pline_t->Append( m_knot[m_order-2] );
          }

          const double *k = m_knot;
          for ( i = 0; i < span_count; i++, k++ )
          {
            if ( k[m_order-2] < k[m_order-1] )
            {
              // see if this bezier segment is a line
              // with (roughly) linear parameterization.
              if ( 0 != pline_points )
                GetCV(i+m_order-1,pline_points->AppendNew());
              if ( 0 != pline_t )
                pline_t->Append(k[m_order-1]);
            }
          }
        }
      }
    }

    if( IsClosed() && rc >= 4 && 0 != pline_points )
    {
      // GBA 2/26/03
      // Make polyline spot on closed.	
      *pline_points->Last() = *pline_points->First();
    }

  }
  return rc;
}



ON_BOOL32
ON_NurbsCurve::IsArc(
      const ON_Plane* plane,
      ON_Arc* arc,
      double tolerance
      ) const
{
  // If changes are made, verify that RR 8497 still works.
  // On 30 April 2003, Mikko changed the strict test to a sloppy
  // test to fix RR 8497.  This change broke other things like 
  // IGES simple entity export tests that require a strict test.
  // So on 6 May 2003 Dale Lear added in the "bLooseTest" check.
  // If the tolerance > ON_ZERO_TOLERANCE a fuzzy fit to arc
  // arc test is performed.  If tolerance <= ON_ZERO_TOLERANCE,
  // a stricter arc test is performed.
  const bool bLooseTest = (tolerance > ON_ZERO_TOLERANCE);
  const int knotcount = KnotCount();
  const int degree = m_order-1;
  if ( (2 == m_dim || 3 == m_dim)
       && m_cv_count >= m_order
       && degree >= 2
       && 0 != m_knot
       && 0 != m_cv
       && (bLooseTest || 0 != m_is_rat)
       )
  {
    if ( bLooseTest || 0 == (knotcount % degree) )
    {

      if ( !bLooseTest )
      {
        for ( int i = 0; i < m_cv_count; i += degree )
        {
          if ( m_knot[i] != m_knot[i+degree-1] )
            return false;
        }
      }

      // 24 July 2012 Dale Lear
      //   You can't be a line and an arc.  This test prevents
      //   creating arcs with small angles and large radii and
      //   prevents sloppy tolerances from classifying a span
      //   as an arc when it makes no sense.
      if ( IsLinear(tolerance) )
        return false;

      if ( ON_Curve::IsArc( plane, arc, tolerance ) )
        return true;
    }
  }

  return false;
}

ON_BOOL32
ON_NurbsCurve::IsPlanar(
      ON_Plane* plane, // if not NULL and true is returned, then plane parameters
                         // are filled in
      double tolerance // tolerance to use when checking linearity
      ) const
{
  if ( m_dim == 2 )
  {
    return ON_Curve::IsPlanar(plane,tolerance);
  }

  ON_BOOL32 rc = false;
  ON_3dPoint P;
  ON_3dVector X;
  EvTangent( Domain()[0], P, X );
  if ( IsLinear(tolerance) )
  {
    if ( plane ) 
    {
      ON_Line line( P, PointAtEnd() );
      if ( !line.InPlane( *plane, tolerance) )
        line.InPlane( *plane, 0.0 );
    }
    rc = true;
  }
  else if ( m_cv_count >= 3 )
  {
    // set P, Q, R to corners of biggest triangle
    // with corners at control points.
    ON_Plane test_plane;
    ON_3dPoint A, B, Q, R;
    Q = P;
    R = P;
    double d, maxd = 0.0;
    int i, j, k;
    k = m_cv_count/64; // to keep sample time reasonable on giant NURBS
    if ( k < 1 )
      k = 1;
    for ( i = 1; i < m_cv_count; i += k )
    {
      GetCV(i,A);
      for ( j = i+k; j < m_cv_count; j += k )
      {
        GetCV(j,B);
        d = ON_CrossProduct( A-P, B-P ).Length();
        if ( d > maxd )
        {
          maxd = d;
          Q = A;
          R = B;
        }
      }
    }
    if ( test_plane.CreateFromPoints(P,Q,R) )
    {
      ON_2dVector v( X*test_plane.xaxis, X*test_plane.yaxis );
      if ( v.Unitize() )
      {
        // Rotate test_plane's x,y axes so its x axis is parallel
        // to the curve's initial tangent.
        if ( fabs(v.y) <= ON_SQRT_EPSILON )
        {
          v.x = (v.x >= 0.0) ? 1.0 : -1.0;
          v.y = 0.0;
        }
        else if ( fabs(v.x) <= ON_SQRT_EPSILON )
        {
          v.y = (v.y >= 0.0) ? 1.0 : -1.0;
          v.x = 0.0;
        }
        X = test_plane.xaxis;
        ON_3dVector Y = test_plane.yaxis;
        test_plane.xaxis = v.x*X + v.y*Y;
        test_plane.yaxis = v.x*Y - v.y*X;
      }
      rc = IsInPlane( test_plane, tolerance );
      if ( rc && plane )
        *plane = test_plane;
    }
  }
  return rc;
}

ON_BOOL32
ON_NurbsCurve::IsInPlane(
      const ON_Plane& plane, // plane to test
      double tolerance // tolerance to use when checking linearity
      ) const
{
  ON_BOOL32 rc = IsValid();
  ON_3dPoint P;
  int i;
  for ( i = 0; rc && i < m_cv_count; i++ )
  {
    GetCV(i,P);
    if ( fabs( plane.DistanceTo(P)) > tolerance)
      rc = false;
  }
  return rc;
}

ON_BOOL32
ON_NurbsCurve::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
       double t,       // t = parameter in domain
       double* tminus, // tminus
       double* tplus   // tplus
       ) const
{
  ON_BOOL32 rc = false;
  ON_Interval d = Domain();
  if ( d.IsIncreasing() ) {
    const double* knot = Knot();
    const int order = Order();
    const int cv_count = CVCount();
    if ( t < knot[order-1] )
      d.m_t[1] = knot[order-1];
    else if ( t > knot[cv_count-2] )
      d.m_t[0] = knot[cv_count-2];
    rc = ON_GetParameterTolerance( d.m_t[0], d.m_t[1], t, tminus, tplus );
  }
  return rc;
}

ON_BOOL32 
ON_NurbsCurve::Evaluate( // returns false if unable to evaluate
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

  if( m_order<2)      // GBA added 01-12-06 to fix crash bug
     return false;

  int span_index = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t,side,(hint)?*hint:0);

  if ( -2 == side || 2 == side )
  {
    // 9 November 2010 Dale Lear - ON_TuneupEvaluationParameter fix
    //   When evluation passes through ON_CurveProxy or ON_PolyCurve reparamterization
    //   and the original side parameter was -1 or +1, it is changed to -2 or +2
    //   to indicate that if t is numerically closed to an end paramter, then
    //   it should be tuned up to be at the end paramter.
    double a = t;
    if ( ON_TuneupEvaluationParameter( side, m_knot[span_index+m_order-2], m_knot[span_index+m_order-1], &a) )
    {
      // recalculate span_index
      t = a;
      span_index = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t,side,span_index);
    }
  }

  rc = ON_EvaluateNurbsSpan(
     m_dim, m_is_rat, m_order, 
     m_knot + span_index, 
     m_cv_stride, m_cv + (m_cv_stride*span_index),
     der_count, 
     t,
     v_stride, v 
     );
  if ( hint ) 
    *hint = span_index;
  return rc;
}


ON_BOOL32 
ON_NurbsCurve::IsClosed() const
{
  ON_BOOL32 bIsClosed = false;
  if ( m_dim > 0 && m_cv_count >= 4 ) 
  {
    if ( IsPeriodic() ) {
      bIsClosed = true;
    }
    else {
      bIsClosed = ON_Curve::IsClosed();
    }
  }
  return bIsClosed;
}

ON_BOOL32 
ON_NurbsCurve::IsPeriodic() const
{
  ON_BOOL32 bIsPeriodic = ON_IsKnotVectorPeriodic( m_order, m_cv_count, m_knot );
  if ( bIsPeriodic ) {
    int i0 = m_order-2;
    int i1 = m_cv_count-1;
    const double* cv0 = m_cv + i0*m_cv_stride;
    const double* cv1 = m_cv + i1*m_cv_stride;
    for ( /*empty*/; i0 >= 0; i0--, i1-- ) {
      if ( false == ON_PointsAreCoincident( m_dim, m_is_rat, cv0, cv1 ) )
        return false;
      cv0 -= m_cv_stride;
      cv1 -= m_cv_stride;      
    }
  }
  return bIsPeriodic;
}


bool ON_IsCurvatureDiscontinuity( 
  const ON_3dVector Km, 
  const ON_3dVector Kp,
  double cos_angle_tolerance,
  double curvature_tolerance,
  double zero_curvature,
  double radius_tolerance
  )
{
  // This function is provided so old code will link.
  // New code should use the version with an explicit relative_tolerance
  return ON_IsCurvatureDiscontinuity( Km, Kp, 
    cos_angle_tolerance, 
    curvature_tolerance,
    zero_curvature,
    radius_tolerance,
    0.001 // relative_tolerance - used to be hard coded in this version of the function
    );
}

bool ON_IsG2CurvatureContinuous(
  const ON_3dVector Km, 
  const ON_3dVector Kp,
  double cos_angle_tolerance,
  double curvature_tolerance
  )
{
  // 6 November 2010 Dale Lear
  //  I made up these values used for cos_Kangle_tolerance
  //  and rel_tol today.  They may need adjusting as we 
  //  get test results.
  double rel_tol = 0.05; // disables using curvature_tolerance to flag unequal scalars

  double cos_Kangle_tolerance = cos_angle_tolerance;
  if ( cos_Kangle_tolerance > ON_DEFAULT_ANGLE_TOLERANCE_COSINE )
    cos_Kangle_tolerance = ON_DEFAULT_ANGLE_TOLERANCE_COSINE;
  if ( cos_Kangle_tolerance > 0.95 )
  {
    // double the tangent angle
    if ( cos_angle_tolerance < 0.0 )
    {
      cos_Kangle_tolerance = -1.0;
    }
    else
    {
      cos_Kangle_tolerance = 2.0*cos_Kangle_tolerance*cos_Kangle_tolerance - 1.0; // = cos(2*tangent_angle_tolerace)
      if ( cos_angle_tolerance >= 0.0 && cos_Kangle_tolerance < 0.0 )
        cos_Kangle_tolerance = 0.0;
    }
  }

  return !ON_IsCurvatureDiscontinuity(Km,Kp,
            cos_Kangle_tolerance,
            curvature_tolerance,
            ON_ZERO_CURVATURE_TOLERANCE,
            ON_UNSET_VALUE, // no absolute delta radius tolerance
            rel_tol
            );
}

bool ON_IsGsmoothCurvatureContinuous(
  const ON_3dVector Km, 
  const ON_3dVector Kp,
  double,
  double curvature_tolerance
  )
{
  double rel_tol = 1.0; // disables using curvature_tolerance to flag unequal scalars

  // point of inflection angle >= 90 degrees
  double cos_Kangle_tolerance = 0.0;

  // Since cos_Kangle_tolerance and rel_tol are valid settings,
  // curvature_tolerance is used only to test for equality.
  return !ON_IsCurvatureDiscontinuity(Km,Kp,
            cos_Kangle_tolerance,
            curvature_tolerance,
            ON_ZERO_CURVATURE_TOLERANCE,
            ON_UNSET_VALUE, // no absolute delta radius tolerance
            rel_tol
            );
}

bool ON_IsCurvatureDiscontinuity( 
  const ON_3dVector Km, 
  const ON_3dVector Kp,
  double cos_angle_tolerance,
  double curvature_tolerance,
  double zero_curvature,
  double radius_tolerance,
  double relative_tolerance
  )
{
  const double d = (Km-Kp).Length();
  if ( !ON_IsValid(d) )
  {
    // invalid curvatures - call it discontinuous because
    // there is a good chance the derivative evaluation 
    // failed or the first derivative is zero.
    return true;
  }

  // The d <= 0.0 test handles the case when 
  // curvature_tolerance is ON_UNSET_VALUE
  if ( d <= 0.0 || d <= curvature_tolerance )
    return false; // "equal" curvature vectors

  // If the scalar curvature is <= zero_curvature,
  // then K is small enough that we assume it is zero.
  // The primary reason for doing this is to prevent
  // reporting a curvature discontinuity when one
  // or both of the curvatures is small1.0e-300 and 1.0e-200
  if ( !(zero_curvature > 7.7037197787136e-34) )
    zero_curvature = 7.7037197787136e-34;
  double km = Km.Length();
  double kp = Kp.Length();
  // If km or kp are NaNs, I treat them as zero so
  // the rest of this code can assume they are 0.0
  // or positive.
  if ( !(km > zero_curvature) )
    km = 0.0;
  if ( !(kp > zero_curvature) )
  {
    kp = 0.0;
    if ( 0.0 == km )
    {
      // both curvatures are "zero"
      return false;
    }
  }

  if ( !(km > 0.0 && kp > 0.0) )
  {
    // one side is flat and the other is curved.
    return true;
  }

  if ( 0.0 == curvature_tolerance )
  {
    // User is asking for exact match.
    return true;
  }


  // At this point we know km > 0 and kp > 0.

  bool bPointOfInflection = (curvature_tolerance > 0.0);
  bool bDifferentScalars = bPointOfInflection;
  // NOTE: 
  //   At this point either curvature_tolerance was not
  //   specified or |Km - Kp| > curvature_tolerance. Because
  //   it is difficult to come up with a meaningful value for
  //   curvature_tolerance that works at all scales, the
  //   additional tests below are used to determine if 
  //   Km and Kp are different enough to matter. If additional
  //   tests are performed, then bTestCurvatureTolerance
  //   is set to false. If no additional tests are performed,
  //   then bTestCurvatureTolerance will remain true and
  //   the |Km-Kp| > curvature_tolerance test is performed
  //   at the end of this function.

  if ( cos_angle_tolerance >= -1.0 && cos_angle_tolerance <= 1.0 )
  {
    const double KmoKp = Kp*Km;
    if ( KmoKp < km*kp*cos_angle_tolerance )
      return true; // Km and Kp are not parallel
    bPointOfInflection = false;
  }

  // At this point we assume Km and Kp are parallel and we
  // know km > 0 and kp > 0.
  if ( radius_tolerance >= 0.0 )
  {
    // This test is if (fabs(rm - rp) > radius_tolerance) return true
    // where rm = 1.0/km and rp = 1.0/kp.  It is coded the way
    // it is to avoid divides (not that that really matters any more).
    if ( fabs(km-kp) > kp*km*radius_tolerance )
      // radii do not agree
      return true;
    bDifferentScalars = false;
  }

  if ( relative_tolerance > 0.0 )
  {
    if ( fabs(km-kp) > ((km>kp)?km:kp)*relative_tolerance )
    {
      return true;
    }
    bDifferentScalars = false;
  }

  if ( bPointOfInflection || bDifferentScalars )
  {
    // |Km - Kp| > curvature_tolerance and we were not asked to
    // perform and other tests.
    return true;
  }

  return false; // treat Km and Kp as equal curvatures.
}


bool ON_NurbsCurve::GetNextDiscontinuity( 
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
  const double is_linear_tolerance = 1.0e-8;  
  const double is_linear_min_length = 1.0e-8;
  int tmp_hint = 0, tmp_dtype=0;
  double d, tmp_t;
  ON_3dPoint Pm, Pp;
  ON_3dVector D1m, D1p, D2m, D2p, Tm, Tp, Km, Kp;
  int ki;
  if ( !hint )
    hint = &tmp_hint;
  if ( !dtype )
    dtype = &tmp_dtype;
  if ( !t )
    t = &tmp_t;
  
  if ( c == ON::C0_continuous )
    return false;
  if ( c == ON::C0_locus_continuous )
  {
    return ON_Curve::GetNextDiscontinuity( c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
  }
  if ( t0 == t1 )
    return false;

  // First test for parametric discontinuities.  If none are found
  // then we will look for locus discontinuities at ends
  if ( m_order <= 2 )
    c = ON::PolylineContinuity(c); // no need to check 2nd derivatives that are zero
  const ON::continuity input_c = c;
  c = ON::ParametricContinuity(c);
  bool bEv2ndDer    = (c == ON::C2_continuous || c == ON::G2_continuous || c == ON::Gsmooth_continuous) && (m_order>2);
  bool bTestKappa   = ( bEv2ndDer && c != ON::C2_continuous );
  bool bTestTangent = ( bTestKappa || c == ON::G1_continuous );

  int delta_ki = 1;
  int delta = ((bEv2ndDer) ? 3 : 2) - m_order; // delta <= 0
  if ( ON::Cinfinity_continuous == c )
    delta = 0;

  ki = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t0,(t0>t1)?-1:1,*hint);
  double segtol = (fabs(m_knot[ki]) + fabs(m_knot[ki+1]) + fabs(m_knot[ki+1]-m_knot[ki]))*ON_SQRT_EPSILON;

  const bool bLineWiggleTest = (c == ON::Gsmooth_continuous && m_order >= 4);
  bool bSpanIsLinear = false;

  if ( t0 < t1 )
  {
    int ii = ki+m_order-2;
    if ( t0 < m_knot[ii+1] && t1 > m_knot[ii+1] && (m_knot[ii+1]-t0) <= segtol && ii+2 < m_cv_count )
    {
      t0 = m_knot[ii+1];
      ki = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t0,1,*hint);
    }
    if ( bLineWiggleTest )
      bSpanIsLinear = SpanIsLinear(ki,is_linear_min_length,is_linear_tolerance);
    *hint = ki;
    ki += m_order-2;
    while (ki < m_cv_count-1 && m_knot[ki] <= t0) 
      ki++;
    if (ki >= m_cv_count-1) 
    {
      if ( input_c != c && t0 < m_knot[m_cv_count-1] && t1 >= m_knot[m_cv_count-1] )
      {
        // have to do locus end test
        return ON_Curve::GetNextDiscontinuity( input_c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
      }
      return false;
    }
  }
  else
  {
    // (t0 > t1) work backwards
    int ii = ki+m_order-2;
    if ( t0 > m_knot[ii] && t1 < m_knot[ii] && (t0-m_knot[ii]) <= segtol && ii > m_order-2 )
    {
      t0 = m_knot[ii];
      ki = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t0,-1,*hint);
    }
    if ( bLineWiggleTest )
      bSpanIsLinear = SpanIsLinear(ki,is_linear_min_length,is_linear_tolerance);
    *hint = ki;
    ki += m_order-2;
    while (ki > m_order-2 && m_knot[ki] >= t0) 
      ki--;
    if (ki <= m_order-2) 
    {
      if ( input_c != c && t0 > m_knot[m_order-2] && t1 < m_knot[m_order-2] )
      {
        // have to do locus end test
        return ON_Curve::GetNextDiscontinuity( input_c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
      }
      return false;
    }
    delta_ki = -1;
    delta = -delta;
  }

  double search_domain[2];
  if ( t0 <= t1 )
  {
    search_domain[0] = t0;
    search_domain[1] = t1;
  }
  else
  {
    search_domain[0] = t1;
    search_domain[1] = t0;
  }

  while ( search_domain[0] < m_knot[ki] && m_knot[ki] < search_domain[1] ) 
  {
    if ( delta_ki > 0 )
    {
      // t0 < t1 case
      while (ki < m_cv_count-1 && m_knot[ki] == m_knot[ki+1])
        ki++;
      if (ki >= m_cv_count-1) 
        break;    
    }
    else
    {
      // t0 > t1 case
      // 20 March 2003 Dale Lear:
      //     Added to make t0 > t1 case work
      while (ki > m_order-2 && m_knot[ki] == m_knot[ki-1])
        ki--;
      if (ki <= m_order-2) 
        break;    
    }

    if (m_knot[ki] == m_knot[ki+delta]) 
    {  
      if ( ON::Cinfinity_continuous == c )
      {
        // Cinfinity_continuous is treated as asking for the next knot
        *dtype = 3;
        *t = m_knot[ki];
        return true;
      }

      if ( bEv2ndDer ) {
        Ev2Der( m_knot[ki], Pm, D1m, D2m, -1, hint );
        Ev2Der( m_knot[ki], Pp, D1p, D2p,  1, hint );
      }
      else {
        Ev1Der( m_knot[ki], Pm, D1m, -1, hint );
        Ev1Der( m_knot[ki], Pp, D1p,  1, hint );
      }

      if ( bTestTangent )
      {
        if ( bTestKappa )
        {
          ON_EvCurvature( D1m, D2m, Tm, Km );
          ON_EvCurvature( D1p, D2p, Tp, Kp );
        }
        else
        {
          Tm = D1m;
          Tp = D1p;
          Tm.Unitize();
          Tp.Unitize();
        }
        d = Tm*Tp;
        if ( d < cos_angle_tolerance )
        {
          *dtype = 1;
          *t = m_knot[ki];
          return true;
        }
        else if ( bTestKappa )
        {
          bool bIsCurvatureContinuous = ( ON::Gsmooth_continuous == c )
                  ? ON_IsGsmoothCurvatureContinuous( Km, Kp, cos_angle_tolerance, curvature_tolerance )
                  : ON_IsG2CurvatureContinuous( Km, Kp, cos_angle_tolerance, curvature_tolerance );
          if ( !bIsCurvatureContinuous )
          {
            // NOTE:
            //   The test to enter this scope must exactly match
            //   the one used in ON_PolyCurve::GetNextDiscontinuity()
            //   and  ON_Curve::GetNextDiscontinuity().
            *dtype = 2;
            *t = m_knot[ki];
            return true;
          }
          if ( bLineWiggleTest )
          {
            if ( bSpanIsLinear != (( delta_ki < 0 )
                                  ? SpanIsLinear(ki - m_order + 1,is_linear_min_length,is_linear_tolerance)
                                  : SpanIsLinear(ki - m_order + 2,is_linear_min_length,is_linear_tolerance))
               )
            {
              // we are at a transition between a line segment and a wiggle
              *dtype = 3;
              *t = m_knot[ki];
              return true;
            }
          }
        }
      }
      else
      {
        if ( !(D1m-D1p).IsTiny(D1m.MaximumCoordinate()*ON_SQRT_EPSILON) )
        {
          *dtype = 1;
          *t = m_knot[ki];
          return true;
        }
        else if ( bEv2ndDer )
        {
          if ( !(D2m-D2p).IsTiny(D2m.MaximumCoordinate()*ON_SQRT_EPSILON) )
          {
            *dtype = 2;
            *t = m_knot[ki];
            return true;
          }         
        }
      }
    }
    ki += delta_ki;
  }

  // 20 March 2003 Dale Lear:
  //   If we get here, there are not discontinuities strictly between
  //   t0 and t1.
  bool rc = false;

  if ( input_c != c )
  {
    // use base class for consistent start/end locus testing 
    rc = ON_Curve::GetNextDiscontinuity( input_c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
  }

  return rc;
}

bool ON_NurbsCurve::IsContinuous(
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

  if ( m_order <= 2 )
    desired_continuity = ON::PolylineContinuity(desired_continuity);

  if ( t <= m_knot[m_order-2] || t >= m_knot[m_cv_count-1] )
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

  if ( m_order < m_cv_count && desired_continuity != ON::C0_continuous )
  {
    int tmp_hint;
    if ( !hint )
    {
      tmp_hint = 0;
      hint = &tmp_hint;
    }
    int ki = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t,1,*hint);

    {
      // 20 March 2003 Dale Lear:
      //     If t is very near interior m_t[] value, see if it
      //     should be set to that value.  A bit or two of 
      //     precision sometimes gets lost in proxy
      //     domain to real curve domain conversions on the interior
      //     of a curve domain.
      int ii = ki+m_order-2;
      double segtol = (fabs(m_knot[ii]) + fabs(m_knot[ii+1]) + fabs(m_knot[ii+1]-m_knot[ii]))*ON_SQRT_EPSILON;
      if ( m_knot[ii]+segtol < m_knot[ii+1]-segtol )
      {
        if ( fabs(t-m_knot[ii]) <= segtol && ii > m_order-2 )
        {
          t = m_knot[ii];
        }
        else if ( fabs(t-m_knot[ii+1]) <= segtol && ii+2 < m_cv_count )
        {
          t = m_knot[ii+1];
          ki = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t,1,*hint);
        }
      }
    }

    if ( ki < 0 )
      ki = 0;
    *hint = ki;
    ki += m_order-2;
    if ( ki > m_order-2 && ki < m_cv_count-1 && m_knot[ki] == t  )
    {
      if ( ON::Cinfinity_continuous == desired_continuity )
      {
        // Cinfinity_continuous is a euphanisim for "at a knot"
        return false;
      }

      // t = interior knot value - check for discontinuity
      int knot_mult = ON_KnotMultiplicity( m_order, m_cv_count, m_knot, ki );

      switch(desired_continuity)
      {
      case ON::C2_continuous: 
        if ( m_order - knot_mult >= 3 )
          return true;
        break;
      case ON::C1_continuous: 
        if ( m_order - knot_mult >= 2 )
          return true;
        break;
      case ON::G2_continuous: 
      case ON::Gsmooth_continuous: 
        if ( m_order - knot_mult >= 3 )
          return true;
        break;
      case ON::G1_continuous: 
        if ( m_order - knot_mult >= 2 )
          return true;
        break;
      default:
        // intentionally ignoring other ON::continuity enum values
        break;
      }
 
      // need to evaluate at knot
      rc = ON_Curve::IsContinuous( desired_continuity, t, hint, 
                           point_tolerance, d1_tolerance, d2_tolerance, 
                           cos_angle_tolerance, curvature_tolerance );

      if ( rc 
           && ON::Gsmooth_continuous == desired_continuity 
           && knot_mult == m_order-1 
           && ki > m_order-2
           && ki < m_cv_count-1
           )
      {
        // See if we are transitioning from linear to non-linear
        const double is_linear_tolerance = 1.0e-8;  
        const double is_linear_min_length = 1.0e-8;
        if ( SpanIsLinear(ki - m_order + 2,is_linear_min_length,is_linear_tolerance) != SpanIsLinear(ki - 2*m_order + 3,is_linear_min_length,is_linear_tolerance) )
          rc = false;
        
      }
    }
  }
  return rc;
}


ON_BOOL32 
ON_NurbsCurve::SetWeight( int i, double w )
{
  ON_BOOL32 rc = false;
  if ( m_is_rat ) {
    double* cv = CV(i);
    if (cv) {
      cv[m_dim] = w;
      rc = true;
    }
  }
  else if ( w == 1.0 ) {
    rc = true;
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 
ON_NurbsCurve::SetCV( int i, ON::point_style style, const double* Point )
{
  ON_BOOL32 rc = true;
  int k;
  double w;

  // feeble but fast check for properly initialized class
  if ( !m_cv || i < 0 || i >= m_cv_count )
    return false;

  double* cv = m_cv + i*m_cv_stride;

  switch ( style ) {

  case ON::not_rational:  // input Point is not rational
    memcpy( cv, Point, m_dim*sizeof(*cv) );
    if ( IsRational() ) {
      // NURBS curve is rational - set weight to one
      cv[m_dim] = 1.0;
    }
    break;

  case ON::homogeneous_rational:  // input Point is homogeneous rational
    if ( IsRational() ) {
      // NURBS curve is rational
      memcpy( cv, Point, (m_dim+1)*sizeof(*cv) );
    }
    else {
      // NURBS curve is not rational
      w = (Point[m_dim] != 0.0) ? 1.0/Point[m_dim] : 1.0;
      for ( k = 0; k < m_dim; k++ ) {
        cv[k] = w*Point[k];
      }
    }
    break;

  case ON::euclidean_rational:  // input Point is euclidean rational
    if ( IsRational() ) {
      // NURBS curve is rational - convert euclean point to homogeneous form
      w = Point[m_dim];
      for ( k = 0; k < m_dim; k++ )
        cv[k] = w*Point[k]; // 22 April 2003 - bug fix [i] to [k]
      cv[m_dim] = w;
    }
    else {
      // NURBS curve is not rational
      memcpy( cv, Point, m_dim*sizeof(*cv) );
    }
    break;

  case ON::intrinsic_point_style:
    memcpy( cv, Point, CVSize()*sizeof(*cv) );
    break;
    
  default:
    rc = false;
    break;
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 
ON_NurbsCurve::SetCV( int i, const ON_3dPoint& point )
{
  ON_BOOL32 rc = false;
  double* cv = CV(i);
  if ( cv ) {
    cv[0] = point.x;
    if ( m_dim > 1 ) {
      cv[1] = point.y;
      if ( m_dim > 2 )
        cv[2] = point.z;
      if ( m_dim > 3 ) {
        memset( &cv[3], 0, (m_dim-3)*sizeof(*cv) );
      }
    }
    if ( m_is_rat ) {
      cv[m_dim] = 1.0;
    }
    rc = true;
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 
ON_NurbsCurve::SetCV( int i, const ON_4dPoint& point )
{
  ON_BOOL32 rc = false;
  double* cv = CV(i);
  if ( cv ) {
    if ( m_is_rat ) {
      cv[0] = point.x;
      if ( m_dim > 1 ) {
        cv[1] = point.y;
        if ( m_dim > 2 )
          cv[2] = point.z;
        if ( m_dim > 3 ) {
          memset( &cv[3], 0, (m_dim-3)*sizeof(*cv) );
        }
      }
      cv[m_dim] = point.w;
      rc = true;
    }
    else {
      double w;
      if ( point.w != 0.0 ) {
        w = 1.0/point.w;
        rc = true;
      }
      else {
        w = 1.0;
      }
      cv[0] = w*point.x;
      if ( m_dim > 1 ) {
        cv[1] = w*point.y;
        if ( m_dim > 2 ) {
          cv[2] = w*point.z;
        }
        if ( m_dim > 3 ) {
          memset( &cv[3], 0, (m_dim-3)*sizeof(*cv) );
        }
      }
    }
  }
	DestroyCurveTree();
  return rc;
}

ON_BOOL32 
ON_NurbsCurve::GetCV( int i, ON::point_style style, double* Point ) const
{
  const double* cv = CV(i);
  if ( !cv )
    return false;
  int dim = Dimension();
  double w = ( IsRational() ) ? cv[dim] : 1.0;
  switch(style) {
  case ON::euclidean_rational:
    Point[dim] = w;
    PCL_FALLTHROUGH
  case ON::not_rational:
    if ( w == 0.0 )
      return false;
    w = 1.0/w;
    while(dim--) *Point++ = *cv++ * w;
    break;
  case ON::homogeneous_rational:
    Point[dim] = w;
    memcpy( Point, cv, dim*sizeof(*Point) );
    break;
  case ON::intrinsic_point_style:
    memcpy( Point, cv, CVSize()*sizeof(*Point) );
    break;
  default:
    return false;
  }
  return true;
}

ON_BOOL32 
ON_NurbsCurve::GetCV( int i, ON_3dPoint& point ) const
{
  ON_BOOL32 rc = false;
  const double* cv = CV(i);
  if ( cv ) {
    if ( m_is_rat ) {
      if (cv[m_dim] != 0.0) {
        const double w = 1.0/cv[m_dim];
        point.x = cv[0]*w;
        point.y = (m_dim>1)? cv[1]*w : 0.0;
        point.z = (m_dim>2)? cv[2]*w : 0.0;
        rc = true;
      }
    }
    else {
      point.x = cv[0];
      point.y = (m_dim>1)? cv[1] : 0.0;
      point.z = (m_dim>2)? cv[2] : 0.0;
      rc = true;
    }
  }
  return rc;
}

ON_BOOL32 
ON_NurbsCurve::GetCV( int i, ON_4dPoint& point ) const
{
  ON_BOOL32 rc = false;
  const double* cv = CV(i);
  if ( cv ) {
    point.x = cv[0];
    point.y = (m_dim>1)? cv[1] : 0.0;
    point.z = (m_dim>2)? cv[2] : 0.0;
    point.w = (m_is_rat) ? cv[m_dim] : 1.0;
    rc = true;
  }
  return rc;
}

bool ON_NurbsCurve::SetKnot( int knot_index, double k )
{
  if ( knot_index < 0 || knot_index >= KnotCount() )
    return false;
  m_knot[knot_index] = k;
	DestroyCurveTree();
  return true;
}

ON_BOOL32 ON_NurbsCurve::SetStartPoint(
        ON_3dPoint start_point
        )
{
  ON_BOOL32 rc = false;
  if ( IsValid() )
  {
    if ( PointAtStart() == start_point )
    {
      rc = true;
    }
    else
    {
      ClampEnd(2);

      double w = 1.0;
      if (IsRational()) {
        w = Weight(0);
        start_point *= w;
      }
      SetCV(0,start_point);
      if (IsRational())
        SetWeight(0, w);

      DestroyCurveTree();
      rc = true;
    }
  }
  return rc;
}

ON_BOOL32 ON_NurbsCurve::SetEndPoint(
        ON_3dPoint end_point
        )
{
  bool rc = false;
  if ( IsValid() )
  {
    if ( PointAtEnd() == end_point )
    {
      rc = true;
    }
    else
    {
      ClampEnd(2);

      double w = 1.0;
      if (IsRational()) {
        w = Weight(m_cv_count-1);
        end_point *= w;
      }
      SetCV(m_cv_count-1,end_point);
      if (IsRational())
        SetWeight(m_cv_count-1, w);

      DestroyCurveTree();
      rc = true;
    }
  }
  return rc;
}


ON_BOOL32
ON_NurbsCurve::Reverse()
{
  ON_BOOL32 rc0 = ON_ReverseKnotVector( m_order, m_cv_count, m_knot );
  ON_BOOL32 rc1 = ON_ReversePointList( m_dim, m_is_rat, m_cv_count, m_cv_stride, m_cv );
	DestroyCurveTree();
  return rc0 && rc1;
}

ON_BOOL32
ON_NurbsCurve::SwapCoordinates( int i, int j )
{
	DestroyCurveTree();
  return  ON_SwapPointListCoordinates( m_cv_count, m_cv_stride, m_cv, i, j );
}

double ON_NurbsCurve::GrevilleAbcissa(
         int gindex  // index (0 <= index < CVCount(dir)
         ) const
{
  return ON_GrevilleAbcissa( m_order, m_knot+gindex );
}

bool ON_NurbsCurve::GetGrevilleAbcissae( // see ON_GetGrevilleAbcissa() for details
         double* g         // g[m_cv_count]
         ) const
{
  return ON_GetGrevilleAbcissae( m_order, m_cv_count, m_knot, false, g );
}


bool ON_NurbsCurve::ZeroCVs()
{
  bool rc = false;
  int i;
  if ( m_cv ) {
    if ( m_cv_capacity > 0 ) {
      memset( m_cv, 0, m_cv_capacity*sizeof(*m_cv) );
      if ( m_is_rat ) {
        for ( i = 0; i < m_cv_count; i++ ) {
          SetWeight( i, 1.0 );
        }
      }
      rc = true;
    }
    else {
      double* cv;
      int s = CVSize()*sizeof(*cv);
      for ( i = 0; i < m_cv_count; i++ ) {
        cv = CV(i);
        memset(cv,0,s);
        if ( m_is_rat )
          cv[m_dim] = 1.0;
      }
      rc = (i>0) ? true : false;
    }
  }
	DestroyCurveTree();
  return rc;
}

bool ON_NurbsCurve::IsClamped( // determine if knot vector is clamped
      int end // (default= 2) end to check: 0 = start, 1 = end, 2 = start and end
      ) const
{
  return ON_IsKnotVectorClamped( m_order, m_cv_count, m_knot, end );
}
  

bool ON_NurbsCurve::ReserveCVCapacity(int desired_capacity)
{
  // If m_cv_capacity == 0 and m_cv != NULL, then the user
  // has hand built the ON_NurbsCurve.m_cv array and is responsible
  // for making sure it's always big enough.
  bool rc = true;
  if ( desired_capacity > m_cv_capacity ) {
    if ( !m_cv ) {
      // no cv array - allocate one
      m_cv = (double*)onmalloc(desired_capacity*sizeof(*m_cv));
      if ( !m_cv ) {
        rc = false;
      }
      else {
        m_cv_capacity = desired_capacity;
      }
    }
    else if ( m_cv_capacity > 0 ) {
      // existing m_cv[] is too small and the fact that
      // m_cv_capacity > 0 indicates that ON_NurbsCurve is
      // managing the m_cv[] memory, so we need to grow
      // the m_cv[] array.
      m_cv = (double*)onrealloc(m_cv,desired_capacity*sizeof(*m_cv));
      if ( !m_cv ) {
        rc = false;
        m_cv_capacity = 0;
      }
      else {
        m_cv_capacity = desired_capacity;
      }
    }
  }
  return rc;
}

bool ON_NurbsCurve::ReserveKnotCapacity(int desired_capacity)
{
  // If m_knot_capacity == 0 and m_knot != NULL, then the user
  // has hand built the ON_NurbsCurve.m_knot array and is responsible
  // for making sure it's always big enough.
  bool rc = true;
  if ( desired_capacity > m_knot_capacity ) {
    if ( !m_knot ) {
      // no knot array - allocate one
      m_knot = (double*)onmalloc(desired_capacity*sizeof(*m_knot));
      if ( !m_knot ) {
        m_knot_capacity = 0;
        rc = false;
      }
      else {
        m_knot_capacity = desired_capacity;
      }
    }
    else if ( m_knot_capacity > 0 ) {
      // existing m_knot[] is too small and the fact that
      // m_knot_capacity > 0 indicates that ON_NurbsCurve is
      // managing the m_knot[] memory, so we need to grow
      // the m_knot[] array.
      m_knot = (double*)onrealloc(m_knot,desired_capacity*sizeof(*m_knot));
      if ( !m_knot ) {
        rc = false;
        m_knot_capacity = 0;
      }
      else {
        m_knot_capacity = desired_capacity;
      }
    }
  }
  return rc;
}

bool ON_NurbsCurve::ConvertSpanToBezier( int span_index, ON_BezierCurve& bez ) const
{
  bool rc = false;
  if ( span_index >= 0 && span_index <= m_cv_count-m_order && m_knot && m_cv ) 
  {
    const int cvdim = CVSize();
    const int sizeof_cv = cvdim*sizeof(*bez.m_cv);
    int i;
    rc = bez.ReserveCVCapacity( cvdim*m_order );
    if ( rc ) {
      bez.m_dim = m_dim;
      bez.m_is_rat = m_is_rat;
      bez.m_order = m_order;
      bez.m_cv_stride = cvdim;
      if ( bez.m_cv_stride == m_cv_stride )
      {
        memcpy( bez.m_cv, CV(span_index), bez.m_order*sizeof_cv );
      }
      else
      {
        for ( i = 0; i < m_order; i++ ) {
          memcpy( bez.CV(i), CV(span_index+i), sizeof_cv );
        }
      }
      const double* knot = m_knot + span_index;
			if( knot[m_order-2] < knot[m_order-1] )
				ON_ConvertNurbSpanToBezier( cvdim, bez.m_order, bez.m_cv_stride, bez.m_cv,
																		knot, knot[m_order-2], knot[m_order-1] );
			else
				rc = false;
    }
  }
  return rc;
}


bool ON_NurbsCurve::HasBezierSpans() const
{
  return ON_KnotVectorHasBezierSpans( m_order, m_cv_count, m_knot );
}

bool ON_NurbsCurve::MakePiecewiseBezier( bool bSetEndWeightsToOne )
{
  bool rc = HasBezierSpans();
  if ( !rc && IsValid() ) 
  {
    ON_Workspace ws;
    DestroyRuntimeCache();
    if ( !ClampEnd(2) )
      return false;
    int span_count = SpanCount();
    //int knot_count = KnotCount();
    ReserveKnotCapacity( (span_count + 1)*(m_order-1) );
    ReserveCVCapacity( m_cv_stride*(span_count*(m_order-1) + 1) );
    double* t = ws.GetDoubleMemory( span_count+1);
    GetSpanVector( t );
    int cvdim = CVSize();
    ON_BezierCurve* bez = new ON_BezierCurve[span_count];
    int ki, spani, i;
    for ( ki = m_order-2, spani = 0; ki < m_cv_count-1 && spani < span_count; ki++ ) {
      if ( m_knot[ki] < m_knot[ki+1] ) {
        bez[spani].Create(m_dim,m_is_rat,m_order);
        for ( i = 0; i < m_order; i++ )
          bez[spani].SetCV(  i, ON::intrinsic_point_style, CV( i + ki - m_order + 2 ) );
        ON_ConvertNurbSpanToBezier( cvdim, bez[spani].m_order, bez[spani].m_cv_stride, bez[spani].m_cv,
                                    m_knot+ki-m_order+2, m_knot[ki], m_knot[ki+1] );
        spani++;
      }
    }
    m_cv_count = span_count*(m_order-1) + 1;
    for ( spani = 0; spani < span_count; spani++ ) {
      for ( i = 0; i < m_order; i++ ) {
        SetCV(  spani*(m_order-1) + i, ON::intrinsic_point_style, bez[spani].CV( i ) );
      }
      for ( ki = 0; ki < m_order-1; ki++ )
        m_knot[ki+spani*(m_order-1)] = t[spani];
    }
    for ( ki = 0; ki < m_order-1; ki++ )
      m_knot[ki+span_count*(m_order-1)] = t[spani];
    delete[] bez;
    rc = true;
  }
  if ( rc && bSetEndWeightsToOne && m_is_rat )
  {
    // 2 June 2003 Dale Lear - added support for bSetEndWeightsToOne=true option.
    double w0, w1;
    ON_BezierCurve bez;
    bez.m_dim = m_dim;
    bez.m_is_rat = m_is_rat;
    bez.m_order = m_order;
    bez.m_cv_stride = m_cv_stride;

    bez.m_cv = CV(0);
    if ( bez.Weight(0) != 1.0 )
    {
      DestroyRuntimeCache();
      w0 = 1.0;
      w1 = (m_order == m_cv_count) ? 1.0 : bez.Weight(m_order-1);
      bez.ChangeWeights(0,w0,m_order-1,w1);
    }
 
    bez.m_cv = CV(m_cv_count-m_order);
    if ( bez.Weight(m_order-1) != 1.0 )
    {
      DestroyRuntimeCache();
      w0 = bez.Weight(0);
      w1 = 1.0; // 23 June 2003
      bez.ChangeWeights(0,w0,m_order-1,w1);
    }

    bez.m_cv = 0;
  }
  return rc;
}


double ON_NurbsCurve::ControlPolygonLength() const
{
  double length = 0.0;
  ON_GetPolylineLength( m_dim, m_is_rat, m_cv_count, m_cv_stride, m_cv, &length );
  return length;
}


bool ON_NurbsCurve::InsertKnot( double knot_value, int knot_multiplicity )
{
  bool rc = false;

  const int degree = Degree();

  double t0, t1;
  {
    ON_Interval d = Domain();
    if ( !d.IsIncreasing() )
      return false;
    t0 = d[0];
    t1 = d[1];
  }
  
  if ( knot_multiplicity < 1 || knot_multiplicity > degree ) 
  {
    ON_ERROR("ON_NurbsCurve::ON_InsertKnot(): knot_multiplicity < 1 or knot_multiplicity > degree.");
    return false;
  }

  if( knot_value < t0 || knot_value > t1 )
  {
    ON_ERROR("ON_InsertKnot(): knot_value not in NURBS curve domain.");
    return false;
  }

  if ( knot_value == t0 ) 
  {
    if ( knot_multiplicity == degree ) 
    {
      rc = ClampEnd(0);
    }
    else if ( knot_multiplicity == 1 ) 
    {
      rc = true;
    }
    else 
    {
      ON_ERROR("ON_InsertKnot(): knot_value = t0 and 1 < knot_multiplicity < degree.");
      rc = false;
    }
    return rc;
  }

  if ( knot_value == t1 ) 
  {
    if ( knot_multiplicity == degree ) 
    {
      rc = ClampEnd(1);
    }
    else if ( knot_multiplicity == 1 ) 
    {
      rc = true;
    }
    else 
    {
      ON_ERROR("ON_InsertKnot(): knot_value = t1 and 1 < knot_multiplicity < degree.");
      rc = false;
    }
    return rc;
  }

  DestroyCurveTree();

  ON_BOOL32 bIsPeriodic = (degree>1) ? IsPeriodic() : false;
  int span_index = ON_NurbsSpanIndex( m_order, m_cv_count, m_knot, knot_value, 0, 0 );

  // reserve room for new knots and cvs
  if ( !ReserveCVCapacity( m_cv_stride*(m_cv_count+knot_multiplicity) ) )
    return false;
  if ( !ReserveKnotCapacity( KnotCount()+knot_multiplicity ) )
    return false;
  if ( bIsPeriodic ) {
  }

  rc = true;
  int span_hint = span_index;
  int new_knot_count = ON_InsertKnot( knot_value, knot_multiplicity, 
                                      CVSize(), m_order, m_cv_count, 
                                      m_cv_stride, m_cv, m_knot, &span_hint );
  if ( new_knot_count > 0 ) 
  {
    m_cv_count += new_knot_count;
  }

  if ( bIsPeriodic && rc && !IsPeriodic() ) {
    // restore periodic form
    if ( ON_MakeKnotVectorPeriodic( m_order, m_cv_count, m_knot ) ) {
      int i0, i1;
      for ( i0 = 0, i1 = m_cv_count-degree; i0 < degree; i0++, i1++ ) {
        if ( span_index < degree-1 )
          SetCV( i1, ON::intrinsic_point_style, CV(i0) ); // cv[i1] = cv[i0]
        else
          SetCV( i0, ON::intrinsic_point_style, CV(i1) ); // cv[i0] = cv[i1]
      }
    }
    else {
      ClampEnd(2);
    }
  }

  return rc;
}

bool ON_NurbsCurve::MakeRational()
{
  if ( !IsRational() ) {
    const int dim = Dimension();
    const int cv_count = CVCount();
    if ( cv_count > 0 && m_cv_stride >= dim && dim > 0 ) {
      const int new_stride = (m_cv_stride == dim) ? dim+1 : m_cv_stride;
      ReserveCVCapacity( cv_count*new_stride );
      const double* old_cv;
      double* new_cv;
      int cvi, j;
      for ( cvi = cv_count-1; cvi>=0; cvi-- ) {
        old_cv = CV(cvi);
        new_cv = m_cv+(cvi*new_stride);
        for ( j = dim-1; j >= 0; j-- ) {
          new_cv[j] = old_cv[j];
        }
        new_cv[dim] = 1.0;
      }
      m_cv_stride = new_stride;
      m_is_rat = 1;
    }
  }
  return IsRational();
}

bool ON_NurbsCurve::MakeNonRational()
{
  if ( IsRational() ) {
    const int dim = Dimension();
    const int cv_count = CVCount();
    if ( cv_count > 0 && m_cv_stride >= dim+1 && dim > 0 ) {
      double w;
      const double* old_cv;
      double* new_cv = m_cv;
      int cvi, j;
      for ( cvi = 0; cvi < cv_count; cvi++ ) {
        old_cv = CV(cvi);
        w = old_cv[dim];
        w = ( w != 0.0 ) ? 1.0/w : 1.0;
        for ( j = 0; j < dim; j++ ) {
          *new_cv++ = w*(*old_cv++);
        }
      }
      m_is_rat = 0;
      m_cv_stride = dim;
    }
  }
	DestroyCurveTree();
  return ( !IsRational() ) ? true : false;
}

static ON_BOOL32 GetRaisedDegreeCV(int old_order, 
                              int cvdim,
                              int old_cvstride,
                              const double* oldCV, //old_cvstride*old_order
                              const double* oldkn, //2*old_degree
                              const double* newkn, //2*old_order
                              int cv_id,     //0 <= cv_id <= old_order
                              double* newCV  //cvdim
                              )

{
  int i,j,k;

  if (!oldCV || !oldkn || !newkn || !newCV || cv_id < 0 || cv_id > old_order)
    return false;

  int old_degree = old_order-1;
  int new_degree = old_degree+1;

  double* t = (double*)onmalloc(old_degree*sizeof(double));
  if (!t) return false;
  double* P = (double*)onmalloc(cvdim*sizeof(double));
  if (!P) {
    onfree((void*)t);
    return false;
  }

  memset(newCV, 0, cvdim*sizeof(double));

  const double* kn = newkn + cv_id;

  for (i=0; i<new_degree; i++){
    k=0;
    for (j=0; j<new_degree; j++){
      if (j != i) {
        t[k] = kn[j];
        k++;
      }
    }
    if (!ON_EvaluateNurbsBlossom(cvdim, old_order, 
      old_cvstride, oldCV, oldkn, t, P)){
      onfree((void*)t);
      onfree((void*)P);
      return false;
    }
    for (k=0; k<cvdim; k++) newCV[k] += P[k];
  }

  double denom = (double)new_degree;
  for (i=0; i<cvdim; i++) 
    newCV[i] /= denom;

  onfree((void*)t);
  onfree((void*)P);

  return true;
}


static ON_BOOL32 IncrementNurbDegree(ON_NurbsCurve& N)
//for use only in ON_NurbsCurve::IncreaseDegree().  No validation done.
//N is assumed to have sufficient knot and cv capacities, clamped ends

{
  ON_NurbsCurve M = N;
  int span_count = M.SpanCount();
  int new_kcount = M.KnotCount() + span_count + 1;
  N.m_order = M.Order()+1;
  N.m_cv_count = new_kcount - N.Order() + 2;

  //set N's knots
  int i=0;
  int k=0;
  int j;
  while (i<M.CVCount()){
    double kn = M.Knot(i);
    int mult = M.KnotMultiplicity(i);
    for (j=0; j<=mult; j++) {
      N.SetKnot(k, kn);
      k++;
    }
    i+=mult;
  }
  //zero out N's cvs
  memset(N.m_cv, 0, N.m_cv_capacity*sizeof(double));
  int cvdim = N.CVSize();
  const double* cvM;
  double* cvN;
  const double* knotN;
  const double* knotM;
  int siN = 0;
  int siM = 0;

  for (i=0; i<span_count; i++){
    knotN = &N.m_knot[siN];
    knotM = &M.m_knot[siM];
    cvM = M.CV(siM);
    cvN = N.CV(siN);
    int span_mult = N.KnotMultiplicity(siN+N.Degree()-1);
    int skip = N.Order()-span_mult;
    cvN += skip*N.m_cv_stride;
    for (j=skip; j<N.Order(); j++){//calculate each cv of the span
      GetRaisedDegreeCV(M.Order(), cvdim, M.m_cv_stride, cvM, knotM, knotN, j, cvN);
      cvN += N.m_cv_stride;
    }
    siN = ON_NextNurbsSpanIndex(N.Order(), N.CVCount(), N.m_knot, siN);
    siM = ON_NextNurbsSpanIndex(M.Order(), M.CVCount(), M.m_knot, siM);
  }

  //set first and last equal to original
  cvM = M.CV(0);
  cvN = N.CV(0);
  for (i=0; i<cvdim; i++) cvN[i] = cvM[i];

  cvM = M.CV(M.CVCount()-1);
  cvN = N.CV(N.CVCount()-1);
  for (i=0; i<cvdim; i++) cvN[i] = cvM[i];

  return true;
}

bool ON_NurbsCurve::IncreaseDegree( int desired_degree )
{
  if ( desired_degree < 1 || desired_degree < m_order-1 ) return false;
  if ( desired_degree == m_order-1 ) return true;
  if (!ClampEnd(2)) return false;

  int del = desired_degree - Degree();
  int new_order = Order()+del;
  int span_count = SpanCount();
  int new_kcount = KnotCount()+(span_count+1)*del;
  int new_cvcount = new_kcount - new_order + 2;

  if (!ReserveKnotCapacity(new_kcount)) return false;
  if (!ReserveCVCapacity(new_cvcount*m_cv_stride)) return false;

  for (int i=0; i<del; i++) {
    if (!IncrementNurbDegree(*this)) return false;
  }

  return true;
}

bool ON_NurbsCurve::ChangeDimension( int desired_dimension )
{
  bool rc = false;
  int i, j;
  if ( desired_dimension < 1 )
    return false;
  if ( desired_dimension == m_dim )
    return true;

  DestroyCurveTree();

  if ( desired_dimension < m_dim ) 
  {
    if ( m_is_rat ) {
      double* cv;
      for ( i = 0; i < m_cv_count; i++ ) {
        cv = CV(i);
        cv[desired_dimension] = cv[m_dim];
      }
    }
    m_dim = desired_dimension;
    rc = true;
  }
  else 
  {
    const double* cv0;
    double* cv1;
    //const int cv_size0 = CVSize();
    const int cv_size1 = m_is_rat ? desired_dimension + 1 : desired_dimension;
    const int cv_stride1 = (m_cv_stride < cv_size1) ? cv_size1 : m_cv_stride;
    if ( m_cv_stride < cv_stride1 && m_cv_capacity > 0 ) {
      m_cv_capacity = cv_stride1*m_cv_count;
      m_cv = (double*)onrealloc( m_cv, m_cv_capacity*sizeof(*m_cv) );
    }
    for ( i = CVCount()-1; i >= 0; i-- ) {
      cv0 = CV(i);
      cv1 = m_cv + (i*cv_stride1);
      if ( m_is_rat )
        cv1[desired_dimension] = cv0[m_dim];
      for ( j = desired_dimension-1; j >= m_dim; j-- ) {
        cv1[j] = 0.0;
      }
      for ( j = m_dim-1; j >= 0; j-- ) {
        cv1[j] = cv0[j];
      }
    }
    m_dim = desired_dimension;
    m_cv_stride = cv_stride1;
    rc = true;
  }
  return rc;
}

bool ON_NurbsCurve::Append( const ON_NurbsCurve& c )
{
  bool rc = false;

  if ( CVCount() == 0 ) {
    *this = c;
    return IsValid()?true:false;
  }

  if ( c.IsRational() && !IsRational() ) {
    if ( !MakeRational() )
      return false;
  }
  if ( c.Degree() > Degree() ) {
    if ( !IncreaseDegree( c.Degree() ) )
      return false;
  }
  if ( c.Dimension() > Dimension() ) {
    if ( !ChangeDimension( c.Dimension() ) )
      return false;
  }

  if (    (IsRational() && !c.IsRational())
       || c.Degree() < Degree() 
       || !c.IsClamped(0) 
       || c.Dimension() < Dimension() ) 
  {
    ON_NurbsCurve tmp(c);
    if ( !tmp.IncreaseDegree( Degree() ) )
      return false;
    if ( !tmp.ChangeDimension( Dimension() ) )
      return false;
    if ( IsRational() ) {
      if ( !tmp.MakeRational() )
        return false;
    }
    if ( !tmp.ClampEnd(0) )
      return false;

    // make sure we don't reenter this scope
    if ( tmp.IsRational() != IsRational() )
      return false;
    if ( tmp.Degree() != Degree() )
      return false;
    if ( tmp.Dimension() != Dimension() )
      return false;
    if ( !tmp.IsClamped(0) )
      return false;
    return Append(tmp);
  }

  if (    IsValid() 
       && c.IsValid() 
       && Degree() == c.Degree() 
       && IsRational() == c.IsRational() 
       && Dimension() == c.Dimension() ) 
  {
    if ( !ClampEnd(1) )
      return false;
    const double w0 = c.Weight(0);
    const double w1 = Weight(CVCount()-1);
    double w = 1.0;
    if ( IsRational() && w0 != w1 ) {
      w = w1/w0;
    }
    ReserveCVCapacity( (CVCount()+c.CVCount())*m_cv_stride );
    ReserveKnotCapacity( ON_KnotCount(Order(),CVCount()+c.CVCount()) );
    const double dk = Knot(CVCount()-1) - c.Knot(c.Order()-2);

    double* cv = CV(CVCount()-1);
    const int cv_dim = CVSize();
    const int sizeof_cv = cv_dim*sizeof(*cv);
    const int c_cv_count = c.CVCount();
    const int c_knot_count = c.KnotCount();
    int i0, i1, j;

    i0 = KnotCount();
    for ( i1 = c.Order()-1; i1 < c_knot_count; i1++ ) {
      m_knot[i0++] = c.Knot(i1) + dk;
    }

    for ( i1 = 1; i1 < c_cv_count; i1++ ) {
      cv += m_cv_stride;
      memcpy( cv, c.CV(i1), sizeof_cv );
      if ( w != 1.0 ) {
        for ( j = 0; j < cv_dim; j++ )
          cv[j] *= w;
      }
      m_cv_count++;
    }
    rc = true;
  }
  return rc;
}

static
ON_BOOL32 TweakSplitTrimParameter( double k0, double k1, double& t )
{
  // [k0,k1] = knots that bracket trim/split parameter t
  // returns true if parameter is tweaked
  // The purpose of the tweak is to avoid creating knot
  // vectors that cause numerical problems during evaluation.
  ON_BOOL32 rc = false;
  double ktol;
  if ( k0 < t && t < k1 )
  {
    // Nov 7, 2008 Lowell Walmsley 
    // Changed this from ON_SQRT_EPSILON to 4*ON_SQRT_EPSILON because 
    // to make it less likely to get very narrow knot spans
    // Dale says the intention is to round off to 8 sig figs
    ktol = (fabs(k0) + fabs(k1))*4.0*ON_SQRT_EPSILON;
    if ( t-k0 <= ktol && k1-t > 16.0*ktol )
    {
      t = k0;
      rc = true;
    }
    else if ( k1-t <= ktol && t-k0 > 16.0*ktol )
    {
      t = k1;
      rc = true;
    }
  }
  return rc;
}



ON_BOOL32 ON_NurbsCurve::Trim( const ON_Interval& in )
{
  if ( !in.IsIncreasing() )
    return false;

  const int cv_dim = CVSize();
  const int order = Order();
  double t, split_t;
  int ki, side, i0, i1, i1_max, new_cv_count;

	//Greg Arden 28 April 2003.  Do not change any curve that is trimmed to its entire domain.
	//             This is especiallly important for periodic curves.
	if(in==Domain())
	  return true;


  DestroyCurveTree();

  // cut off right end (or extend if in.m_t[1] > Domain.Max()
  side = -1;
  t = in.m_t[1]; // trimming parameter
  ki = ON_NurbsSpanIndex( order, m_cv_count, m_knot, t, side, 0 );

  // if t is very close to a knot value, then trim at the knot
  split_t = t;
  if ( TweakSplitTrimParameter( m_knot[ki+order-2], m_knot[ki+order-1], split_t ) )
    ki = ON_NurbsSpanIndex( order, m_cv_count, m_knot, split_t, side, ki );

  if ( !ON_EvaluateNurbsDeBoor( cv_dim, order, m_cv_stride, CV(ki),
                                m_knot + ki, side, 0.0, t ) ) 
  {
    ON_ERROR("ON_NurbsCurve::Trim() - right end de Boor algorithm failed.");
    return false;
  }
  // clamp right end knots
  m_cv_count = ki + order;
  for ( i0 = ON_KnotCount( order, m_cv_count)-1; i0 >= m_cv_count-1; i0--)
    m_knot[i0] = t;

  // cut off left end (or extend if in.m_t[0] < Domain.Max()
  side = 1;
  t = in.m_t[0]; // trimming parameter
  ki = ON_NurbsSpanIndex( order, m_cv_count, m_knot, t, side, 0 );

  // if t is very close to a knot value, then trim at the knot
  split_t = t;
  if ( TweakSplitTrimParameter( m_knot[ki+order-2], m_knot[ki+order-1], split_t ) )
    ki = ON_NurbsSpanIndex( order, m_cv_count, m_knot, split_t, side, ki );

  if ( !ON_EvaluateNurbsDeBoor( cv_dim, order, m_cv_stride, CV(ki),
                                m_knot + ki, side, 0.0, t ) ) 
  {
    ON_ERROR("ON_NurbsCurve::Trim() - right end de Boor algorithm failed.");
    return false;
  }

  // remove surplus cvs and knots
  new_cv_count = m_cv_count - ki;
  if ( new_cv_count < m_cv_count ) {
    // move cvs and knots over
    i1_max = m_cv_stride*m_cv_count;
    for ( i0 = 0, i1 = ki*m_cv_stride; i1 < i1_max; i0++, i1++ )
      m_cv[i0] = m_cv[i1];
    i1_max = ON_KnotCount( order, m_cv_count );
    for ( i0 = 0, i1 = ki; i1 < i1_max; i0++, i1++ )
      m_knot[i0] = m_knot[i1];
    m_cv_count = new_cv_count;
  }

  // clamp left end knots
  for (i0 = 0; i0 <= order-2; i0++)
    m_knot[i0] = t;

  ClampEnd(2); // 26 June 2003 Dale Lear

	DestroyCurveTree();
  return true;
}

bool ON_NurbsCurve::Extend(
  const ON_Interval& domain
  )

{
  if (IsClosed()) return false;
  int is_rat = IsRational() ? 1 : 0;
  int dim = Dimension();
  int cvdim = dim+is_rat;

  bool changed = false;
  if (domain[0] < Domain()[0]){
    ClampEnd(0);
    ON_EvaluateNurbsDeBoor(cvdim,Order(),m_cv_stride, CV(0),m_knot,1,0.0,domain[0]);
    for (int i = 0; i < Order()-1; i++)
			m_knot[i] = domain[0];
    changed = true;
  }
  if (domain[1] > Domain()[1]){
    ClampEnd(1);
    int i = CVCount() - Order();
    ON_EvaluateNurbsDeBoor(cvdim,Order(),m_cv_stride, CV(i),m_knot + i,-1,0.0,domain[1]);
    for (i = KnotCount()-1; i >= CVCount()-1; i--)
			m_knot[i] = domain[1];
    changed = true;
  }

  if (changed){
    DestroyCurveTree();
  }
  return changed;
}

ON_BOOL32 ON_NurbsCurve::Split(
    double t,    // t = curve parameter to split curve at
    ON_Curve*& left_crv, // left portion returned here (must be an ON_NurbsCurve)
    ON_Curve*& right_crv // right portion returned here (must be an ON_NurbsCurve)
  ) const
{

  int i;
  ON_BOOL32 rc = false;
  if ( left_crv && !ON_NurbsCurve::Cast(left_crv) )
    return false;
  if ( right_crv && !ON_NurbsCurve::Cast(right_crv) )
    return false;
  if ( IsValid() && t > m_knot[m_order-2] && t < m_knot[m_cv_count-1] ) 
  {
    ON_NurbsCurve* left = (ON_NurbsCurve*)left_crv;
    ON_NurbsCurve* right = (ON_NurbsCurve*)right_crv;
    if ( !left )
      left = new ON_NurbsCurve();
    else if ( left == right )
      return false;

    if ( !right )
      right = new ON_NurbsCurve();
    left->DestroyCurveTree();
    right->DestroyCurveTree();
    
    int span_index = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,t,1,0);
    // if t is very close to a knot value, then split at the knot
    double split_t = t;
    if ( TweakSplitTrimParameter( m_knot[span_index+m_order-2], m_knot[span_index+m_order-1], split_t ) )
    {
      if ( split_t <= m_knot[m_order-2] || split_t >= m_knot[m_cv_count-1] ){
        // 22 October - greg added bailout code but didn't clean up???
        //   chuck fixed leak.
        if (!left_crv) delete left;
        if (!right_crv) delete right;
        return false;
      }
      span_index = ON_NurbsSpanIndex(m_order,m_cv_count,m_knot,split_t,1,span_index);
    }

    if ( span_index >= 0 && span_index <= m_cv_count - m_order ) 
    {
      const int cvdim = CVSize();
      const int sizeof_cv = cvdim*sizeof(double);

      int left_cv_count = m_order + span_index;
      if ( span_index > 0 && split_t == m_knot[span_index+m_order-2] )
      { 
        // splitting exactly at a knot value
        int k;
        for ( k = 0; left_cv_count >= m_order && k <= span_index+m_order-2; k++ )
        {
          if ( split_t == m_knot[span_index+m_order-2-k] )
            left_cv_count--;
          else
            break;
        }
      }
      int right_cv_count = m_cv_count - span_index;
      if ( left_cv_count < m_order || right_cv_count < m_order ){
        // 22 October - greg added bailout code but didn't clean up???
        //   chuck fixed leak.
        if (!left_crv) delete left;
        if (!right_crv) delete right;
        return false;
      }

      if ( left != this )
      {
        left->m_dim = m_dim;
        left->m_is_rat = m_is_rat;
        left->m_order = m_order;
        left->m_cv_count = left_cv_count;
        left->m_cv_stride = cvdim;
      }
      if ( right != this )
      {
        right->m_dim = m_dim;
        right->m_is_rat = m_is_rat;
        right->m_order = m_order;
        right->m_cv_count = right_cv_count;
        right->m_cv_stride = cvdim;
      }

      // fill in left allowing for possibility that left = this
      if ( left->m_cv != m_cv ) 
      {
        left->ReserveCVCapacity(cvdim*left_cv_count);
        for( i = 0; i < left_cv_count; i++ ) {
          memcpy( left->m_cv + i*cvdim, CV(i), sizeof_cv );
        }
      }
      if ( left->m_knot != m_knot ) 
      {
        i = ON_KnotCount( m_order, left_cv_count);
        left->ReserveKnotCapacity( i );
        memcpy( left->m_knot, m_knot, i*sizeof(left->m_knot[0]) );
      }
      
      // fill in right allowing for possibility that right = this
      if ( right->m_cv != m_cv || span_index > 0 )
      {
        right->ReserveCVCapacity(cvdim*right_cv_count);
        for( i = 0; i < right_cv_count; i++ ) {
          memmove( right->m_cv + i*cvdim, CV(i+span_index), sizeof_cv );
        }
      }
      if ( right->m_knot != m_knot || span_index > 0 ) 
      {
        i = ON_KnotCount(m_order, right_cv_count);
        right->ReserveKnotCapacity( i );
        memmove( right->m_knot, m_knot + span_index, i*sizeof(right->m_knot[0]) );
      }

      if ( right == this ) {
        right->m_cv_count = right_cv_count;
        right->m_cv_stride = cvdim;
      }

      if ( left == this ) {
        left->m_cv_count = left_cv_count;
        left->m_cv_stride = cvdim;
      }

      // trim right end of left NURBS
      i = left->m_cv_count - left->m_order;
      ON_EvaluateNurbsDeBoor( cvdim, m_order, cvdim, left->CV(i), left->m_knot + i, -1, 0.0, t );
      for ( i = left->m_cv_count-1; i < ON_KnotCount(left->m_order,left->m_cv_count); i++ )
        left->m_knot[i] = t;
      left->ClampEnd(2); // 26 June 2003 Dale Lear

      // trim left end of right NURBS
      ON_EvaluateNurbsDeBoor( cvdim, m_order, cvdim, right->m_cv, right->m_knot, +1, 0.0, t );
      for ( i = 0; i <= right->m_order-2; i++ )
        right->m_knot[i] = t;
      right->ClampEnd(2); // 26 June 2003 Dale Lear

      if ( 0 == left_crv )
        left_crv = left;
      if ( 0 == right_crv )
        right_crv = right;
      rc = true;
    }
  }
  return rc;
}


int ON_NurbsCurve::GetNurbForm( ON_NurbsCurve& curve, 
                                double,
                                const ON_Interval* subdomain  // OPTIONAL subdomain of ON::ProxyCurve::Domain()
                                ) const
{
  int rc = 1;

  // 4 May 2007 Dale Lear
  //   I'm replacing the call to operator= with a call to
  //   ON_NurbsCurveCopyHelper().  The operator= call
  //   was copying userdata and that does not happen for
  //   any other GetNurbForm overrides.  Copying userdata
  //   in GetNurbForm is causing trouble in Make2D and 
  //   other places that are creating NURBS copies in
  //   worker memory pools.

  // curve = *this; // copied user data
  ON_NurbsCurveCopyHelper(*this,curve); // does not copy user data

  if ( subdomain ) 
  {
    if ( !curve.Trim(*subdomain) )
      rc = 0;
  }
  return rc;
}

int ON_NurbsCurve::HasNurbForm() const
{
  return 1;
}


ON_BOOL32 ON_NurbsCurve::GetCurveParameterFromNurbFormParameter(
      double nurbs_t,
      double* curve_t
      ) const
{
  *curve_t = nurbs_t;
  return true;
}

ON_BOOL32 ON_NurbsCurve::GetNurbFormParameterFromCurveParameter(
      double curve_t,
      double* nurbs_t
      ) const
{
  *nurbs_t = curve_t;
  return true;
}

bool ON_NurbsCurve::ClampEnd( 
        int end // 0 = clamp start, 1 = clamp end, 2 = clamp start and end 
        )
{
  // Curve tree doesn't change when you clamp // DestroyCurveTree();
  return ON_ClampKnotVector( CVSize(), m_order, 
                             m_cv_count, m_cv_stride, m_cv, 
                             m_knot, end );
}


/*
static 
bool ON_DuplicateKnots( int order, int cv_count, bool bRevKnot1,
                        double* knot0, 
                        double* knot1
                        )
{
  return false;
}

static 
bool ON_DuplicateContolPoints( int dim, int is_rat, int cv_count, 
                               bool bRevCV1,
                               int stride0, double* cv0, 
                               int stride1, double* cv1
                              )
{
  return false;
}
*/

static bool ON_IsDuplicateKnotVector( int order, int cv_count, 
               const double* knot, const double* other_knot,
               bool bIgnoreParameterization )
{
  bool rc = (    0 != knot 
              && 0 != other_knot 
              && order >= 2 
              && cv_count >= order);

  if (rc)
  {
    const int knot_count = ON_KnotCount( order, cv_count );
    int i;
    if ( bIgnoreParameterization )
    {
      const ON_Interval dom(knot[order-2],knot[cv_count-1]);
      const ON_Interval other_dom(other_knot[order-2],other_knot[cv_count-1]);
      double k, other_k;
      for ( i = 0; i < knot_count && rc; i++ )
      {
        k = dom.NormalizedParameterAt(knot[i]);
        other_k = dom.NormalizedParameterAt(other_knot[i]);
        rc = (fabs(k-other_k) <= ON_ZERO_TOLERANCE);
      }
    }
    else
    {
      for ( i = 0; i < knot_count && rc; i++ )
      {
        rc = (knot[i] == other_knot[i]);
      }
    }
  }
  return rc;
}

static bool ON_IsDuplicatePointList( int dim, int is_rat,
                                     int count, 
                                     int stride, 
                                     const double* cv,
                                     int other_stride,
                                     const double* other_cv,
                                     double tolerance
                                     )
{
  bool rc = (dim > 0 && is_rat >= 0 && is_rat <= 1 && count > 0
             && abs(stride) >= dim+is_rat && abs(other_stride) >= (dim+is_rat)
             && 0 != cv && 0 != other_cv);
  if (rc)
  {
    if ( tolerance < 0.0 )
      tolerance = 0.0;
    int i, j;
    double w = 1.0;
    double other_w = 1.0;
    double cv_tol = tolerance;
    for ( i = 0; i < count && rc; i++ )
    {
      if ( is_rat )
      {
        w = cv[dim];
        other_w = other_cv[dim];
        cv_tol = fabs(w*tolerance);
        rc = ( w == other_w );
      }

      for ( j = 0; j < dim && rc; j++ )
      {
        rc = (fabs(cv[j] - other_cv[j]) <= cv_tol);
      }
      cv += stride;
      other_cv += other_stride;
    }
  }
  return rc;
}
                                   

bool ON_NurbsCurve::IsDuplicate( 
        const ON_NurbsCurve& other, 
        bool bIgnoreParameterization,
        double tolerance 
        ) const
{
  bool rc = (this == &other);
  if ( !rc
       && m_dim      == other.m_dim
       && m_is_rat   == other.m_is_rat
       && m_order    == other.m_order
       && m_cv_count == other.m_cv_count
       )
  {
    // compare knots
    rc = ON_IsDuplicateKnotVector( m_order, m_cv_count, m_knot, other.m_knot, bIgnoreParameterization );

    // compare control points
    if (rc)
      rc = ON_IsDuplicatePointList( m_dim, m_is_rat?1:0, m_cv_count,
                                    m_cv_stride, m_cv,
                                    other.m_cv_stride, other.m_cv,
                                    tolerance );
  }
  return rc;
}

bool ON_NurbsSurface::IsDuplicate( 
        const ON_NurbsSurface& other, 
        bool bIgnoreParameterization,
        double tolerance 
        ) const
{
  bool rc = (this == &other);
  if ( !rc
       && m_dim      == other.m_dim
       && m_is_rat   == other.m_is_rat
       && m_order[0] == other.m_order[0]
       && m_order[1] == other.m_order[1]
       && m_cv_count[0] == other.m_cv_count[0]
       && m_cv_count[1] == other.m_cv_count[1]
       )
  {
    // compare knots
    rc = ON_IsDuplicateKnotVector( m_order[0], m_cv_count[0], m_knot[0], other.m_knot[0], bIgnoreParameterization );
    if (rc)
      rc = ON_IsDuplicateKnotVector( m_order[1], m_cv_count[1], m_knot[1], other.m_knot[1], bIgnoreParameterization );

    // compare control points
    int i;
    for ( i = 0; i < m_cv_count[0] && rc; i++ )
    {
      rc = ON_IsDuplicatePointList( m_dim, m_is_rat?1:0, m_cv_count[1],
                                    m_cv_stride[1], CV(i,0),
                                    other.m_cv_stride[1], other.CV(i,0),
                                    tolerance );
    }
  }
  return rc;
}


bool ON_Brep::IsDuplicate( 
        const ON_Brep&,
        double
        ) const
{
  // OBSOLETE FUNCTION - REMOVE
  return false;
}

bool ON_NurbsCurve::Reparameterize(double c)
{
  if ( !ON_IsValid(c) || 0.0 == c )
    return false;

  if ( 1.0 == c )
    return true;

  if ( !MakeRational() )
    return false;

  return ON_ReparameterizeRationalNurbsCurve(
           c,
           m_dim,m_order,m_cv_count,
           m_cv_stride,m_cv,m_knot
           );
}

bool ON_ReparameterizeRationalNurbsCurve(
          double c, 
          int dim, 
          int order, 
          int cv_count,
          int cvstride,
          double* cv,
          double* knot
          )
{
  // Reference
  //  E. T. Y. Lee and M. L. Lucian 
  //  Mobius reparameterization of rational B-splines
  //  CAGD Vol8 pp 213-215 1991
  const double c1 = c-1.0;
  double k0, k1, k, d, w0, w1;
  int i,j;

  if ( !ON_IsValid(c) || !ON_IsValid(c1) || 0.0 == c )
    return false;

  if ( 1.0 == c )
    return true;

  // change domain to [0,1] and then adjust knots
  k0 = knot[order-2];
  k1 = knot[cv_count-1];
  d = k1 - k0;
  if ( !ON_IsValid(d) || d <= 0.0 )
    return false;
  d = 1.0/d;
  j = cv_count+order-2;
  for ( i = 0; i < j; i++ )
  {
    k = knot[i];
    k = (k - k0)*d;
    knot[i] = c*k/(c1*k+1.0);
  }

  // adjust cvs
  order -= 2;
  cvstride -= (dim+1);
  for ( i = 0; i < cv_count; i++ )
  {
    d = c - c1*(*knot++);
    j = order;
    while(j--)
    {
      d *= c - c1*knot[j];
    }
    w0 = cv[dim];
    w1 = w0*d;
    j = dim;
    while(j--)
      *cv++ *= d;
    *cv++ = w1;
    cv += cvstride;
  }
  order += 2;
  cvstride += (dim+1);
  cv -= cv_count*cvstride;
  knot -= cv_count;

  // change domain back to [k0,k1]
  j = cv_count+order-2;
  for ( i = 0; i < j; i++ )
  {
    k = knot[i];
    knot[i] = (1.0-k)*k0 + k*k1;
  }

  return true;
}

bool ON_NurbsCurve::ChangeEndWeights(double w0, double w1)
{
  if ( !ON_IsValid(w0) || !ON_IsValid(w1) || 0.0 == w0 || 0.0 == w1 )
    return false;
  if ( (w0 < 0.0 && w1 > 0.0) || (w0 > 0.0 && w0 < 0.0) )
    return false;

  if (!ClampEnd(2))
    return false;

  if ( w0 == Weight(0) && w1 == Weight(m_cv_count) )
    return true;

  if ( !MakeRational() )
    return false;

  return ON_ChangeRationalNurbsCurveEndWeights(
          m_dim,m_order,
          m_cv_count,m_cv_stride,m_cv,
          m_knot,
          w0,w1);
}

bool ON_ChangeRationalNurbsCurveEndWeights(
          int dim, 
          int order, 
          int cv_count,
          int cvstride, 
          double* cv, 
          double* knot,
          double w0, 
          double w1
          )
{
  double r, s, v0, v1;
  int i, j;

  if ( !ON_IsValid(w0) || !ON_IsValid(w1) || 0.0 == w0 || 0.0 == w1 )
    return false;
  if ( (w0 < 0.0 && w1 > 0.0) || (w0 > 0.0 && w0 < 0.0) )
    return false;

  if ( !ON_ClampKnotVector( dim+1, order, cv_count, cvstride, cv, knot, 2 ) )
    return false;

  v0 = cv[dim];
  v1 = cv[cvstride*(cv_count-1)+dim];
  if (!ON_IsValid(v0) || !ON_IsValid(v1) || v0 == 0.0 || v1 == 0.0)
    return false;
  if (v0 < 0.0 && v1 > 0.0)
    return false;
  if ( v0 > 0.0 && v1 < 0.0)
    return false;

  r = w0/v0;
  s = w1/v1;
  if ( fabs(r-s) <= fabs(s)*ON_SQRT_EPSILON )
  {
    // simply scale
    if ( r != s )
      s = 0.5*(r+s);
    r = s;
  }

  if ( 1.0 != s && v1 != w1 )
  {
    // scale to get last weight set to w1
    dim++;
    cvstride -= dim;
    i = cv_count;
    while(i--)
    {
      j = dim;
      while(j--)
        *cv++ *= s;
      cv += cvstride;
    }
    cvstride += dim;
    dim--;
    cv -= cvstride*cv_count;
  }

  if ( r != s )
  {
    v0 = cv[dim];
    v1 = cv[cvstride*(cv_count-1)+dim];
    if ( ON_IsValid(v0) && ON_IsValid(v1) && 0.0 != v0 )
    {
      // need to scale and reparameterize
      r = pow(w0/v0,1.0/((double)(order-1)));
      if ( !ON_IsValid(r) )
        return false;
      if ( !ON_ReparameterizeRationalNurbsCurve(r,dim,order,cv_count,cvstride,cv,knot) )
        return false;
    }
  }

  // make sure weights agree to the last bit! 
  cv[dim] = w0;
  cv[cvstride*(cv_count-1)+dim] = w1;

  return true;
}

double ON_Fuzz( double x, double absolute_tolerance )
{
  double fuzz = fabs(x)*ON_RELATIVE_TOLERANCE;
  return(fuzz > absolute_tolerance) ? fuzz : absolute_tolerance;
}

bool ON_NurbsCurve::SpanIsSingular( 
  int span_index 
  ) const
{
  const int cv_size = CVSize();
  if (    m_order < 2 
       || m_cv_count < m_order
       || m_dim <= 0
       || cv_size > m_cv_stride 
       || 0 == m_knot
       || 0 == m_cv
       )
  {
    ON_ERROR("Invalid NURBS curve.");
    return false;
  }

  if ( span_index < 0 || span_index > m_cv_count-m_order )
  {
    ON_ERROR("span_index parameter is out of range.");
    return false;
  }

  const double* cv = CV(span_index);
  const double* knot = m_knot + span_index;

  if ( !(knot[m_order-2] < knot[m_order-1]) )
  {
    // vacuous question because there is no "span" evaluate.
    // I chose return false here so people won't try to
    // remove this empty span.
    // no call to ON_ERROR here
    return false; 
  }

  double* p = 0;
  int cv_stride = m_cv_stride;
  if ( knot[0] != knot[m_order-2] || knot[m_order-1] != knot[2*m_order-3] )
  {
    const std::size_t sizeof_cv = cv_size*sizeof(p[0]);
    p = (double*)onmalloc(sizeof_cv*m_order);
    for ( int i = 0; i < m_order; i++ )
      memcpy( p+(i*cv_size), cv+(i*cv_stride), sizeof_cv );
    ON_ConvertNurbSpanToBezier( cv_size, m_order, cv_size, p,
															  knot, knot[m_order-2], knot[m_order-1] 
                              );
    cv_stride = cv_size;
    cv = p;
  }
  const bool rc = ON_PointsAreCoincident(m_dim,m_is_rat,m_order,cv_stride,cv);
  if ( 0 != p )
    onfree(p);

  return rc;
}

bool ON_NurbsCurve::RemoveSpan(
  int span_index 
  )
{
  const int cv_size = CVSize();
  if (    m_order < 2 
       || m_cv_count < m_order
       || m_dim <= 0
       || cv_size > m_cv_stride 
       || 0 == m_knot
       || 0 == m_cv
       )
  {
    ON_ERROR("Invalid NURBS curve.");
    return false;
  }

  if ( span_index < 0 || span_index > m_cv_count-m_order )
  {
    ON_ERROR("span_index parameter is out of range.");
    return false;
  }

  if ( m_cv_count == m_order )
  {
    ON_ERROR("Cannot remove the only span from a Bezier NURBS curve.");
    return false;
  }

  const std::size_t sizeof_cv = cv_size*sizeof(m_cv[0]);
  int i, j;

  const double knot0 = m_knot[span_index+m_order-2];
  const double knot1 = m_knot[span_index+m_order-1];
  const double knot_delta = (knot0 < knot1) ? (knot1 - knot0) : 0.0;

  const bool bIsPeriodic0 = IsPeriodic()?true:false;

  if ( span_index <= 0 )
  {
    // remove initial span
    // set span_index = index of the span to keep.
    for ( span_index = 1; span_index < m_cv_count-m_order; span_index++ )
    {
      if ( m_knot[span_index+m_order-2] < m_knot[span_index+m_order-1] )
        break;
    }
    for ( i = 0; i+span_index < m_cv_count; i++ )
      memcpy(CV(i),CV(i+span_index),sizeof_cv);
    for ( i = 0; i+span_index < m_cv_count+m_order-2; i++ )
    {
      m_knot[i] = (knot1 == m_knot[i+span_index])
                ? knot0
                : (m_knot[i+span_index] - knot_delta);
    }
    m_cv_count -= span_index;
  }
  else if ( span_index >= m_cv_count-m_order )
  {
    // remove final span
    // set span_index = index of the span to keep.
    for ( span_index = m_cv_count-m_order-1; span_index > 0; span_index-- )
    {
      if ( m_knot[span_index+m_order-2] < m_knot[span_index+m_order-1] )
        break;
    }
    m_cv_count = span_index+m_order;
  }
  else
  {
    // remove interior span
    int k0 = span_index+m_order-2;
    int k1 = span_index+m_order-1;
    int i0 = k0;
    int i1 = k1;
    for ( i0 = k0; i0 > 0; i0-- )
    {
      if ( m_knot[i0-1] < m_knot[k0] )
        break;
    }
    for ( i1 = k1; i1 < m_cv_count+m_order-3; i1++ )
    {
      if ( m_knot[i1+1] > m_knot[k1] )
        break;
    }
    int m = (i1-i0+1);
    if ( !(knot_delta > 0.0) )
    {
      if ( !(m_knot[i0] == m_knot[i1]) || m < m_order )
      {
        ON_ERROR("span_index parameter identifies an empty span.");
        return false;
      }
    }

    int span_index0 = i0 - (m_order-1);
    double* cv0 = 0;
    if ( span_index0 >= 0 && k0 - i0 + 1 < m_order-1 )
    {
      cv0 = (double*)onmalloc( (m_order*cv_size + 2*m_order-2)*sizeof(cv0[0]) );
      double* knot0 = cv0 + (m_order*cv_size);
      memcpy( knot0, m_knot+span_index0, (2*m_order-2)*sizeof(knot0[0]) );
      for ( i = 0; i < m_order; i++ )
        memcpy( cv0 + (i*cv_size), CV(span_index0+i), sizeof_cv );
      ON_ClampKnotVector( cv_size, m_order, m_order, cv_size, cv0, knot0, 1 );
    }

    if ( m < m_order-1 )
    {
      i = m_order-1 - m;
      ReserveCVCapacity( m_cv_stride*(m_cv_count+i) );
      ReserveKnotCapacity( m_cv_count+m_order-2+i );
      for ( j = m_cv_count+m_order-3; j >= i1-m_order+2; j-- )
        m_knot[j+i] = m_knot[j];
      for ( j = m_cv_count-1; j >= i1-m_order+2; j-- )
        memcpy(CV(j+i),CV(j),sizeof_cv);
      i1 += i;
      k1 += i;
      m_cv_count += i;
    }

    if ( i1-k1 < m_order-2 )
      ON_ClampKnotVector( cv_size, m_order, m_order, m_cv_stride, 
                          m_cv + ((i1-m_order+2)*m_cv_stride), 
                          m_knot + (i1-m_order+2), 
                          0 );

    k0 = i0;
    k1 = i1-m_order+2;

    if ( 0 != cv0 )
    {
      for ( i = 0; i < m_order-1; i++ )
        memcpy(CV(i+span_index0),cv0 + (i*cv_size),sizeof_cv);
      onfree(cv0);
      cv0 = 0;
    }

    if ( k0 < k1 )
    {
      for ( i = 0; i + k1 < m_cv_count; i++ )
        memcpy(CV(i+k0),CV(i+k1),sizeof_cv);
      for ( i = 0; i + k1 < m_cv_count+m_order-2; i++ )
      {
        m_knot[i+k0] = (knot1 == m_knot[i+k1]) 
                     ? knot0
                     : (m_knot[i+k1] - knot_delta);
      }
      m_cv_count -= (k1-k0);
    }
    else if ( k0 == k1 && knot_delta > 0.0 )
    {
      for ( i = k0; i < m_cv_count+m_order-2; i++ )
      {
        m_knot[i] = (knot1 == m_knot[i])
                  ? knot0
                  : (m_knot[i] - knot_delta);
      }
    }
  }

  if ( false == bIsPeriodic0 || false == IsPeriodic() )
    ClampEnd(2);

  return true;
}

int ON_NurbsCurve::RemoveSingularSpans()
{
  const int cv_size = CVSize();
  if (    m_order < 2 
       || m_cv_count < m_order
       || m_dim <= 0
       || cv_size > m_cv_stride 
       || 0 == m_knot
       || 0 == m_cv
       )
  {
    ON_ERROR("Invalid NURBS curve.");
    return 0;
  }

  int singular_span_count = 0;

  for ( int span_index = 0; m_cv_count > m_order && span_index <= m_cv_count-m_order; span_index++ )
  {
    if (    m_knot[span_index+m_order-2] < m_knot[span_index+m_order-1] 
         && SpanIsSingular(span_index) 
       )
    {
      const int cv_count0 = m_cv_count;
      if ( RemoveSpan(span_index) )
        singular_span_count++;
      if ( 0 == span_index || m_cv_count < cv_count0 )
        span_index--;
    }
  }

  return singular_span_count;
}

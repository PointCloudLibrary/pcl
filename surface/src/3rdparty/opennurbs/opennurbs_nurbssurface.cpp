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

ON_OBJECT_IMPLEMENT(ON_NurbsSurface,ON_Surface,"4ED7D4DE-E947-11d3-BFE5-0010830122F0");


ON_NurbsSurface* ON_NurbsSurface::New()
{
  // static function replaces new ON_NurbsSurface();
  return new ON_NurbsSurface();
}

ON_NurbsSurface* ON_NurbsSurface::New(
        const ON_NurbsSurface& nurbs_surface
        )
{
  // static function replaces new ON_NurbsSurface(const ON_NurbsSurface& nurbs_surface);
  return new ON_NurbsSurface(nurbs_surface);
}

ON_NurbsSurface* ON_NurbsSurface::New(
        const ON_BezierSurface& bezier_surface
        )
{
  // static function replaces new ON_NurbsSurface(const ON_BezierSurface& bezier_surface);
  return new ON_NurbsSurface(bezier_surface);
}

ON_NurbsSurface* ON_NurbsSurface::New(
        int dimension,
        ON_BOOL32 bIsRational,
        int order0,
        int order1,
        int cv_count0,
        int cv_count1
        )
{
  // static function replaces new ON_NurbsSurface(dim, is_rat, order0, ..., cv_count1 );
  return new ON_NurbsSurface(dimension,bIsRational,order0,order1,cv_count0,cv_count1);
}

ON_NurbsSurface::ON_NurbsSurface()
{
  ON__SET__THIS__PTR(m_s_ON_NurbsSurface_ptr);
  Initialize();
}

ON_NurbsSurface::ON_NurbsSurface( const ON_NurbsSurface& src )
{
  ON__SET__THIS__PTR(m_s_ON_NurbsSurface_ptr);
  Initialize();
  *this = src;
}

ON_NurbsSurface::ON_NurbsSurface( const ON_BezierSurface& bezier_surface )
{
  ON__SET__THIS__PTR(m_s_ON_NurbsSurface_ptr);
  Initialize();
  *this = bezier_surface;
}

ON_NurbsSurface::ON_NurbsSurface(
        int dim,      // dimension (>= 1)
        ON_BOOL32 is_rat,  // true to make a rational NURBS
        int order0,    // order (>= 2)
        int order1,    // order (>= 2)
        int cv_count0,  // cv count0 (>= order0)
        int cv_count1   // cv count1 (>= order1)
        )
{
  ON__SET__THIS__PTR(m_s_ON_NurbsSurface_ptr);
  Initialize();
  Create( dim, is_rat, order0, order1, cv_count0, cv_count1 );
}

ON_NurbsSurface::~ON_NurbsSurface()
{
  Destroy();
}


unsigned int ON_NurbsSurface::SizeOf() const
{
  unsigned int sz = ON_Surface::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Surface));
  sz += m_knot_capacity[0]*sizeof(*m_knot[0]);
  sz += m_knot_capacity[1]*sizeof(*m_knot[1]);
  sz += m_cv_capacity*sizeof(*m_cv);
  return sz;
}

ON__UINT32 ON_NurbsSurface::DataCRC(ON__UINT32 current_remainder) const
{
  current_remainder = ON_CRC32(current_remainder,sizeof(m_dim),&m_dim);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_is_rat),&m_is_rat);
  current_remainder = ON_CRC32(current_remainder,2*sizeof(m_order[0]),&m_order[0]);
  current_remainder = ON_CRC32(current_remainder,2*sizeof(m_cv_count[0]),&m_cv_count[0]);
  if (   m_cv_count[0] > 0 && m_cv_count[1] > 0 
      && m_cv_stride[0] > 0 && m_cv_stride[1] > 0 
      && m_cv )
  {
    size_t sizeof_cv = CVSize()*sizeof(m_cv[0]);
    const double* cv = m_cv;
    int i, j;
    for ( i = 0; i < m_cv_count[0]; i++ )
    {
      cv = CV(i,0);
      for ( j = 0; j < m_cv_count[1]; j++ )
      {
        current_remainder = ON_CRC32(current_remainder,sizeof_cv,cv);
        cv += m_cv_stride[1];
      }
    }
  }
  current_remainder = ON_CRC32(current_remainder,KnotCount(0)*sizeof(m_knot[0][0]),m_knot[0]);
  current_remainder = ON_CRC32(current_remainder,KnotCount(1)*sizeof(m_knot[1][0]),m_knot[1]);

  return current_remainder;
}


ON_BOOL32 ON_NurbsSurface::SetDomain( 
            int dir, // 0 sets first parameter's domain, 1 gets second parameter's domain
            double t0, 
            double t1
            )
{
  bool rc = false;
  if ( m_order[dir] >= 2 && m_cv_count[dir] >= m_order[dir] && m_knot && t0 < t1 ) {
    const double k0 = m_knot[dir][m_order[dir]-2];
    const double k1 = m_knot[dir][m_cv_count[dir]-1];
    if ( k0 == t0 && k1 == t1 )
      rc = true;
    else if ( k0 < k1 ) {
      const double d = (t1-t0)/(k1-k0);
      const double km = 0.5*(k0+k1);
      const int knot_count = KnotCount(dir);
      int i;
      for ( i = 0; i < knot_count; i++ ) {
        if ( m_knot[dir][i] <= km ) {
          m_knot[dir][i] = (m_knot[dir][i] - k0)*d + t0;
        }
        else {
          m_knot[dir][i] = (m_knot[dir][i] - k1)*d + t1;
        }
      }
			rc = true;
      DestroySurfaceTree();
    }
  }
  return rc;

}


int ON_NurbsSurface::Dimension() const
{
  return m_dim;
}

bool ON_NurbsSurface::IsRational() const
{
  return m_is_rat?true:false;
}

int ON_NurbsSurface::CVSize() const
{
  return (m_is_rat) ? m_dim+1 : m_dim;
}

int ON_NurbsSurface::Order( int dir ) const
{
  return m_order[dir?1:0];
}

int ON_NurbsSurface::Degree( int dir ) const
{
  return (m_order[dir?1:0]>=2) ? m_order[dir?1:0]-1 : 0;
}

int ON_NurbsSurface::CVCount( int dir ) const
{
  return m_cv_count[dir?1:0];
}

int ON_NurbsSurface::CVCount( void ) const
{
  return m_cv_count[0]*m_cv_count[1];
}

int ON_NurbsSurface::KnotCount( int dir ) const
{
  return ON_KnotCount( m_order[dir?1:0], m_cv_count[dir?1:0] );
}

double* ON_NurbsSurface::CV( int i, int j ) const
{
  return (m_cv) ? (m_cv + i*m_cv_stride[0] + j*m_cv_stride[1]) : NULL;
}

ON::point_style ON_NurbsSurface::CVStyle() const
{
  return m_is_rat ? ON::homogeneous_rational : ON::not_rational;
}


double ON_NurbsSurface::Weight( int i, int j ) const
{
  return (m_cv && m_is_rat) ? m_cv[i*m_cv_stride[0] + j*m_cv_stride[1] + m_dim] : 1.0;
}


double ON_NurbsSurface::Knot( int dir, int i ) const
{
  return (m_knot[dir?1:0]) ? m_knot[dir?1:0][i] : 0.0;
}

int ON_NurbsSurface::KnotMultiplicity( int dir, int i ) const
{
  dir = dir?1:0;
  return ON_KnotMultiplicity( m_order[dir], m_cv_count[dir], m_knot[dir], i );
}

const double* ON_NurbsSurface::Knot( int dir ) const
{
  return m_knot[dir?1:0];
}

bool ON_NurbsSurface::MakeClampedUniformKnotVector( 
  int dir,
  double delta
  )
{
  if ( dir < 0 || dir > 1 )
    return false;
	DestroySurfaceTree();
  ReserveKnotCapacity( dir, ON_KnotCount( m_order[dir], m_cv_count[dir] ) );
  return ON_MakeClampedUniformKnotVector( m_order[dir], m_cv_count[dir], m_knot[dir], delta );
}

bool ON_NurbsSurface::MakePeriodicUniformKnotVector( 
  int dir,
  double delta
  )
{
  if ( dir < 0 || dir > 1 )
    return false;
	DestroySurfaceTree();
  ReserveKnotCapacity( dir, ON_KnotCount( m_order[dir], m_cv_count[dir] ) );
  return ON_MakePeriodicUniformKnotVector( m_order[dir], m_cv_count[dir], m_knot[dir], delta );
}


double ON_NurbsSurface::SuperfluousKnot( int dir, int end ) const
  { return(m_knot[dir?1:0]) ? ON_SuperfluousKnot(m_order[dir?1:0],m_cv_count[dir?1:0],m_knot[dir?1:0],end) : 0.0;}

ON_BOOL32 ON_NurbsSurface::Create(
        int dim,      // dimension (>= 1)
        ON_BOOL32 is_rat,  // true to make a rational NURBS
        int order0,    // order (>= 2)
        int order1,    // order (>= 2)
        int cv_count0,  // cv count0 (>= order0)
        int cv_count1   // cv count1 (>= order1)
        )
{
  DestroySurfaceTree();
  if ( dim < 1 )
    return false;
  if ( order0 < 2 )
    return false;
  if ( order1 < 2 )
    return false;
  if ( cv_count0 < order0 )
    return false;
  if ( cv_count1 < order1 )
    return false;
  m_dim = dim;
  m_is_rat = (is_rat) ? true : false;
  m_order[0] = order0;
  m_order[1] = order1;
  m_cv_count[0] = cv_count0;
  m_cv_count[1] = cv_count1;
  m_cv_stride[1] = (m_is_rat) ? m_dim+1 : m_dim;
  m_cv_stride[0] = m_cv_stride[1]*m_cv_count[1];
  ON_BOOL32 rc = ReserveKnotCapacity( 0, KnotCount(0) );
  if ( !ReserveKnotCapacity( 1, KnotCount(1) ) )
    rc = false;
  if ( !ReserveCVCapacity( m_cv_count[0]*m_cv_count[1]*m_cv_stride[1] ) )
    rc = false;
  return rc;
}

void ON_NurbsSurface::Destroy()
{
  double* cv = ( m_cv && m_cv_capacity ) ? m_cv : NULL;
  double* knot0 = ( m_knot[0] && m_knot_capacity[0] ) ? m_knot[0] : NULL;
  double* knot1 = ( m_knot[1] && m_knot_capacity[1] ) ? m_knot[1] : NULL;
  Initialize();
  if ( cv )
    onfree(cv);
  if ( knot0 )
    onfree(knot0);
  if ( knot1 )
    onfree(knot1);
}

void ON_NurbsSurface::EmergencyDestroy()
{
  Initialize();
}


void ON_NurbsSurface::Initialize()
{
  m_dim = 0;
  m_is_rat = 0;
  m_order[0] = 0;
  m_order[1] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_capacity = 0;
  m_cv = 0;
}



ON_BOOL32 ON_NurbsSurface::ReserveKnotCapacity( int dir, int capacity )
{
  if (dir)
    dir = 1;
  if ( m_knot_capacity[dir] < capacity ) {
    if ( m_knot[dir] ) {
      if ( m_knot_capacity[dir] ) {
        m_knot[dir] = (double*)onrealloc( m_knot[dir], capacity*sizeof(*m_knot[dir]) );
        m_knot_capacity[dir] = (m_knot[dir]) ? capacity : 0;
      }
      // else user supplied m_knot[] array
    }
    else {
      m_knot[dir] = (double*)onmalloc( capacity*sizeof(*m_knot[dir]) );
      m_knot_capacity[dir] = (m_knot[dir]) ? capacity : 0;
    }
  }
  return ( m_knot[dir] ) ? true : false;
}

ON_BOOL32 ON_NurbsSurface::ReserveCVCapacity( int capacity )
{
  if ( m_cv_capacity < capacity ) {
    if ( m_cv ) {
      if ( m_cv_capacity ) {
        m_cv = (double*)onrealloc( m_cv, capacity*sizeof(*m_cv) );
        m_cv_capacity = (m_cv) ? capacity : 0;
      }
      // else user supplied m_cv[] array
    }
    else {
      m_cv = (double*)onmalloc( capacity*sizeof(*m_cv) );
      m_cv_capacity = (m_cv) ? capacity : 0;
    }
  }
  return ( m_cv ) ? true : false;
}

static void ON_NurbsSurfaceCopyHelper( const ON_NurbsSurface& src, ON_NurbsSurface& dest )
{
  dest.m_dim          = src.m_dim;
  dest.m_is_rat       = src.m_is_rat;
  dest.m_order[0]     = src.m_order[0];
  dest.m_order[1]     = src.m_order[1];
  dest.m_cv_count[0]  = src.m_cv_count[0];
  dest.m_cv_count[1]  = src.m_cv_count[1];
  dest.m_cv_stride[1] = dest.m_is_rat ? dest.m_dim+1 : dest.m_dim;
  dest.m_cv_stride[0] = dest.m_cv_count[1]*dest.m_cv_stride[1];
  if ( src.m_knot[0] ) 
  {
    // copy knot array
    dest.ReserveKnotCapacity( 0, dest.KnotCount(0) );
    memcpy( dest.m_knot[0], src.m_knot[0], dest.KnotCount(0)*sizeof(*dest.m_knot[0]) );
  }
  if ( src.m_knot[1] ) 
  {
    // copy knot array
    dest.ReserveKnotCapacity( 1, dest.KnotCount(1) );
    memcpy( dest.m_knot[1], src.m_knot[1], dest.KnotCount(1)*sizeof(*dest.m_knot[1]) );
  }
  if ( src.m_cv ) 
  {
    // copy cv array
    dest.ReserveCVCapacity( dest.m_cv_count[0]*dest.m_cv_count[1]*dest.m_cv_stride[1] );
    const int dst_cv_size = dest.CVSize()*sizeof(*dest.m_cv);
    const int src_stride[2] = {src.m_cv_stride[0],src.m_cv_stride[1]};
    if ( src_stride[0] == dest.m_cv_stride[0] && src_stride[1] == dest.m_cv_stride[1] ) 
    {
      memcpy( dest.m_cv, src.m_cv, dest.m_cv_count[0]*dest.m_cv_count[1]*dest.m_cv_stride[1]*sizeof(*dest.m_cv) );
    }
    else 
    {
      const double *src_cv;
      double *dst_cv = dest.m_cv;
      int i, j;
      for ( i = 0; i < dest.m_cv_count[0]; i++ ) 
      {
        src_cv = src.CV(i,0);
        for ( j = 0; j < dest.m_cv_count[1]; j++ ) 
        {
          memcpy( dst_cv, src_cv, dst_cv_size );
          dst_cv += dest.m_cv_stride[1];
          src_cv += src_stride[1];
        }
      }
    }
  }
}


ON_NurbsSurface& ON_NurbsSurface::operator=( const ON_NurbsSurface& src )
{
  if ( this != &src ) 
  {
    ON_Surface::operator=(src);
    ON_NurbsSurfaceCopyHelper(src,*this);
  }
  return *this;
}

ON_NurbsSurface& ON_NurbsSurface::operator=( const ON_BezierSurface& bezier_surface )
{
  int i, j;
  DestroySurfaceTree();

  m_dim = bezier_surface.m_dim;
  m_is_rat = bezier_surface.m_is_rat;
  m_order[0] = bezier_surface.m_order[0];
  m_order[1] = bezier_surface.m_order[1];
  m_cv_count[0] = m_order[0];
  m_cv_count[1] = m_order[1];
  m_cv_stride[1] = m_is_rat ? (m_dim+1) : m_dim;
  m_cv_stride[0] = m_cv_stride[1]*m_cv_count[1];

  // copy control points
  if ( bezier_surface.m_cv )
  {
    ReserveCVCapacity(m_cv_count[0]*m_cv_count[1]*m_cv_stride[1]);
    const int sizeof_cv = m_cv_stride[1]*sizeof(*m_cv);
    const double* src_cv;
    double* dst_cv;
    for ( i = 0; i < m_order[0]; i++ ) for( j = 0; j < m_order[1]; j++ )
    {
      src_cv = bezier_surface.CV(i,j);
      dst_cv = CV(i,j);
      memcpy( dst_cv, src_cv, sizeof_cv );
    }
  }

  // set clamped knots for domain [0,1]
  for ( j = 0; j < 2; j++ )
  {
    const int knot_count = KnotCount(j);
    ReserveKnotCapacity( j, knot_count );
    for ( i = 0; i <= m_order[j]-2; i++ )
      m_knot[j][i] = 0.0;
    for ( i = m_order[j]-1; i < knot_count; i++ )
      m_knot[j][i] = 1.0;
  }

  return *this;
}

void ON_NurbsSurface::Dump( ON_TextLog& dump ) const
{
  dump.Print( "ON_NurbsSurface dim = %d is_rat = %d\n"
               "        order = %d X %d cv_count = %d X %d\n",
               m_dim, m_is_rat, m_order[0], m_order[1], m_cv_count[0], m_cv_count[1] );
  int dir;
  for ( dir = 0; dir < 2; dir++ ) 
  {
    dump.Print( "Knot Vector %d ( %d knots )\n", dir, KnotCount(dir) );
    dump.PrintKnotVector( m_order[dir], m_cv_count[dir], m_knot[dir] );
  }

  dump.Print( "Control Points  %d %s points\n"
               "  index               value\n",
               m_cv_count[0]*m_cv_count[1], 
               (m_is_rat) ? "rational" : "non-rational" );
  if ( !m_cv ) 
  {
    dump.Print("  NULL cv array\n");
  }
  else 
  {
    int i;
    char sPreamble[128]; 
    memset(sPreamble,0,sizeof(sPreamble));
    for ( i = 0; i < m_cv_count[0]; i++ )
    {
      if ( i > 0 )
        dump.Print("\n");
      sPreamble[0] = 0;
      sprintf(sPreamble,"  CV[%2d]",i);
      dump.PrintPointList( m_dim, m_is_rat, 
                        m_cv_count[1], m_cv_stride[1],
                        CV(i,0), 
                        sPreamble );
    }
  }
}

ON_BOOL32 ON_NurbsSurface::IsValid( ON_TextLog* text_log ) const
{
  ON_BOOL32 rc = false;

  if ( m_dim <= 0 )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsSurface.m_dim = %d (should be > 0).\n",m_dim);
    }
  }
  else if ( m_cv == NULL )
  {
    if ( text_log )
    {
      text_log->Print("ON_NurbsSurface.m_cv is NULL.\n",m_dim);
    }
  }
  else
  {
    rc = true;
    int i;
    for ( i = 0; i < 2 && rc; i++ )
    {
      rc = false;
      if (m_order[i] < 2 )
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsSurface.m_order[i] = %d (should be >= 2).\n",i,m_order[i]);
        }
      }
      else if (m_cv_count[i] < m_order[i] )
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsSurface.m_cv_count[%d] = %d (should be >= m_order[%d]=%d).\n",i,m_cv_count[i],i,m_order[i]);
        }
      }
      else if (m_knot[i] == NULL)
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsSurface.m_knot[i] is NULL.\n");
        }
      }
      else if ( !ON_IsValidKnotVector( m_order[i], m_cv_count[i], m_knot[i], text_log ) )
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsSurface.m_knot[%d] is not a valid knot vector.\n",i);
        }
      }
      else if ( m_cv_stride[i] < CVSize() )
      {
        if ( text_log )
        {
          text_log->Print("ON_NurbsSurface.m_cv_stride[%d]=%d is too small (should be >= %d).\n",i,m_cv_stride[i],CVSize());
        }
      }
      else
        rc = true;
    }
    if ( rc )
    {
      int a0 = CVSize();
      int a1 = m_cv_count[0]*a0;
      int b1 = CVSize();
      int b0 = m_cv_count[1]*b1;
      if ( m_cv_stride[0] < a0 || m_cv_stride[1] < a1 )
      {
        if ( m_cv_stride[0] < b0 || m_cv_stride[1] < b1 )
        {
          if ( text_log )
          {
            text_log->Print("ON_NurbsSurface.m_cv_stride[] = {%d,%d} is not valid.\n",m_cv_stride[0],m_cv_stride[1]);
          }
          rc = false;
        }
      }
    }
  }

  return rc;
}

ON_BOOL32 ON_NurbsSurface::GetBBox( // returns true if successful
       double* boxmin,    // minimum
       double* boxmax,    // maximum
       ON_BOOL32 bGrowBox  // true means grow box
       ) const
{
  return ON_GetPointGridBoundingBox( m_dim, m_is_rat, 
            m_cv_count[0], m_cv_count[1],
            m_cv_stride[0], m_cv_stride[1], 
            m_cv, 
            boxmin, boxmax, bGrowBox?true:false );
}

ON_BOOL32 ON_NurbsSurface::Transform( const ON_Xform& xform )
{
  DestroySurfaceTree();
  TransformUserData(xform);
  if ( 0 == m_is_rat )
  {
    if ( xform.m_xform[3][0] != 0.0 || xform.m_xform[3][1] != 0.0 || xform.m_xform[3][2] != 0.0 )
    {
      MakeRational();
    }
  }
  return ON_TransformPointGrid( m_dim, m_is_rat, m_cv_count[0], m_cv_count[1], m_cv_stride[0], m_cv_stride[1], m_cv, xform );
}

bool ON_NurbsSurface::IsDeformable() const
{
  return true;
}

bool ON_NurbsSurface::MakeDeformable()
{
  return true;
}


ON_BOOL32 ON_NurbsSurface::Write(
       ON_BinaryArchive&  file // open binary file
     ) const
{
  // NOTE - check legacy I/O code if changed
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if (rc) {
    if (rc) rc = file.WriteInt( m_dim );
    if (rc) rc = file.WriteInt( m_is_rat );
    if (rc) rc = file.WriteInt( m_order[0] );
    if (rc) rc = file.WriteInt( m_order[1] );
    if (rc) rc = file.WriteInt( m_cv_count[0] );
    if (rc) rc = file.WriteInt( m_cv_count[1] );

    if (rc) rc = file.WriteInt(0); // reserved1
    if (rc) rc = file.WriteInt(0); // reserved2

    if (rc) {
      ON_BoundingBox bbox; // write invalid bounding box - may be used in future
      rc = file.WriteBoundingBox(bbox);
    }
    
    int count = m_knot[0] ? KnotCount(0) : 0;
    if (rc) rc = file.WriteInt(count);
    if (rc) rc = file.WriteDouble( count, m_knot[0] );

    count = m_knot[1] ? KnotCount(1) : 0;
    if (rc) rc = file.WriteInt(count);
    if (rc) rc = file.WriteDouble( count, m_knot[1] );

    const int cv_size = CVSize();
    count = ( m_cv && cv_size > 0
              && m_cv_count[0] > 0 && m_cv_count[1] > 0 
              && m_cv_stride[0] >= cv_size && m_cv_stride[1] >= cv_size) 
          ? m_cv_count[0]*m_cv_count[1]
          : 0;
    if (rc) rc = file.WriteInt(count);
    if (rc && count > 0 ) {
      int i, j;
      for ( i = 0; i < m_cv_count[0] && rc; i++ ) {
        for ( j = 0; j < m_cv_count[1] && rc; j++ ) {
          rc = file.WriteDouble( cv_size, CV(i,j) );
        }
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_NurbsSurface::Read(
       ON_BinaryArchive&  file // open binary file
     )
{
  DestroySurfaceTree();
  // NOTE - check legacy I/O code if changed
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1) {
    // common to all 1.x versions
    int dim = 0, is_rat = 0, order0 = 0, order1 = 0, cv_count0 = 0, cv_count1 = 0;
    int reserved1 = 0, reserved2 = 0;
    if (rc) rc = file.ReadInt( &dim );
    if (rc) rc = file.ReadInt( &is_rat );
    if (rc) rc = file.ReadInt( &order0 );
    if (rc) rc = file.ReadInt( &order1 );
    if (rc) rc = file.ReadInt( &cv_count0 );
    if (rc) rc = file.ReadInt( &cv_count1 );

    if (rc) rc = file.ReadInt(&reserved1);
    if (rc) rc = file.ReadInt(&reserved2);

    if (rc) {
      ON_BoundingBox bbox; // read bounding box - may be used in future
      rc = file.ReadBoundingBox(bbox);
    }
    
    Create( dim, is_rat, order0, order1, cv_count0, cv_count1 );

    int count = 0;
    if (rc) rc = file.ReadInt(&count);
    if (rc ) rc = ReserveKnotCapacity(0,count);
    if (rc) rc = file.ReadDouble( count, m_knot[0] );

    count = 0;
    if (rc) rc = file.ReadInt(&count);
    if (rc ) rc = ReserveKnotCapacity(1,count);
    if (rc) rc = file.ReadDouble( count, m_knot[1] );

    count = 0;
    if (rc) rc = file.ReadInt(&count);
    const int cv_size = CVSize();
    if (rc) rc = ReserveCVCapacity( count*cv_size );
    if (count > 0 && cv_size > 0 && rc ) {
      int i, j;
      for ( i = 0; i < m_cv_count[0] && rc; i++ ) {
        for ( j = 0; j < m_cv_count[1] && rc; j++ ) {
          rc = file.ReadDouble( cv_size, CV(i,j) );
        }
      }
    }
  }
  if ( !rc )
    Destroy();
  return rc;
}

ON_Interval ON_NurbsSurface::Domain( int dir ) const
{
  ON_Interval d;
  if (dir) dir = 1;
  ON_GetKnotVectorDomain( m_order[dir], m_cv_count[dir], m_knot[dir], &d.m_t[0], &d.m_t[1] );
  return d;
}

double ON_NurbsSurface::ControlPolygonLength( int dir ) const
{
  double max_length = 0.0;
  if ( dir >= 0 && dir <= 1 && m_cv_count[0] >= 2 && m_cv_count[1] >= 2 && m_cv != NULL )
  {
    double length;
    const double* p;
    int i;
    for( i = 0; i < m_cv_count[1-dir]; i++ )
    {
      length = 0.0;
      p = (dir) ? CV(i,0) : CV(0,i);
      ON_GetPolylineLength( m_dim, m_is_rat, m_cv_count[dir], m_cv_stride[dir], p, &length );
      if ( length > max_length )
        max_length = length;
    }
  }

  return max_length;
}


ON_BOOL32 ON_NurbsSurface::GetSurfaceSize( 
    double* width, 
    double* height 
    ) const
{
  // TODO - get lengths of polygon
  ON_BOOL32 rc = true;
  if ( width )
  {
    *width = ControlPolygonLength( 0 );
  }
  if ( height )
  {
    *height = ControlPolygonLength( 1 );
  }
  return rc;
}

int ON_NurbsSurface::SpanCount( int dir ) const
{
  if (dir) dir = 1;
  return ON_KnotVectorSpanCount( m_order[dir], m_cv_count[dir], m_knot[dir] );
}

ON_BOOL32 ON_NurbsSurface::GetSpanVector( int dir, double* s ) const
{
  if (dir) dir = 1;
  return ON_GetKnotVectorSpanVector( m_order[dir], m_cv_count[dir], m_knot[dir], s );
}

ON_BOOL32 ON_NurbsSurface::GetParameterTolerance( // returns tminus < tplus: parameters tminus <= s <= tplus
       int dir,
       double t,       // t = parameter in domain
       double* tminus, // tminus
       double* tplus   // tplus
       ) const
{
  ON_BOOL32 rc = false;
  ON_Interval d = Domain(dir);
  double t0 = d.Min();
  double t1 = d.Max();
  if ( t0 <= t1 ) {
    const double* knot = Knot(dir);
    const int order = Order(dir);
    const int cv_count = CVCount(dir);
    if ( t < knot[order-1] )
      t1 = knot[order-1];
    else if ( t > knot[cv_count-2] )
      t0 = knot[cv_count-2];
    rc = ON_GetParameterTolerance( t0, t1, t, tminus, tplus );
  }
  return rc;
}


ON_BOOL32
ON_NurbsSurface::Evaluate( // returns false if unable to evaluate
       double s, double t,       // evaluation parameter
       int der_count,  // number of derivatives (>=0)
       int v_stride,   // v[] array stride (>=Dimension())
       double* v,      // v[] array of length stride*(ndir+1)
       int side,       // optional - determines which side to evaluate from
                       //         0 = default
                       //         1 = from NE quadrant
                       //         2 = from NW quadrant
                       //         3 = from SW quadrant
                       //         4 = from SE quadrant
       int hint[2]       // optional - evaluation hint (int) used to speed
                       //            repeated evaluations
       ) const
{
  ON_BOOL32 rc = false;
  int span_index[2];
  span_index[0] = ON_NurbsSpanIndex(m_order[0],m_cv_count[0],m_knot[0],s,(side==2||side==3)?-1:1,(hint)?hint[0]:0);
  span_index[1] = ON_NurbsSpanIndex(m_order[1],m_cv_count[1],m_knot[1],t,(side==3||side==4)?-1:1,(hint)?hint[1]:0);
  rc = ON_EvaluateNurbsSurfaceSpan(
     m_dim, m_is_rat, 
     m_order[0], m_order[1],
     m_knot[0] + span_index[0], 
     m_knot[1] + span_index[1],
     m_cv_stride[0], m_cv_stride[1],
     m_cv + (span_index[0]*m_cv_stride[0] + span_index[1]*m_cv_stride[1]),
     der_count, 
     s, t,
     v_stride, v 
     );
  if ( hint ) {
    hint[0] = span_index[0];
    hint[1] = span_index[1];
  }
  return rc;
}


ON_Curve* ON_NurbsSurface::IsoCurve(
       int dir,          // 0 first parameter varies and second parameter is constant
                         //   e.g., point on IsoCurve(0,c) at t is srf(t,c)
                         // 1 first parameter is constant and second parameter varies
                         //   e.g., point on IsoCurve(1,c) at t is srf(c,t)
       double c          // value of constant parameter 
       ) const
{
  ON_Curve* crv = 0;
  int i,j,k,Scvsize,span_index;
  double* Ncv;
  const double* Scv;
  if ( (dir == 0 || dir == 1) && IsValid() )
  {
    Scvsize = CVSize();
    ON_NurbsCurve* nurbscrv = new ON_NurbsCurve( m_dim, m_is_rat, m_order[dir], m_cv_count[dir] );
    memcpy( nurbscrv->m_knot, m_knot[dir], nurbscrv->KnotCount()*sizeof(*nurbscrv->m_knot) );
    span_index = ON_NurbsSpanIndex(m_order[1-dir],m_cv_count[1-dir],m_knot[1-dir],c,1,0);
    if ( span_index < 0 )
      span_index = 0;
    else if ( span_index > m_cv_count[1-dir]-m_order[1-dir] )
      span_index = m_cv_count[1-dir]-m_order[1-dir];
    ON_NurbsCurve N( Scvsize*nurbscrv->CVCount(), 0, m_order[1-dir], m_order[1-dir] );
    memcpy( N.m_knot, m_knot[1-dir]+span_index, N.KnotCount()*sizeof(*N.m_knot) );
    for ( i = 0; i < N.m_cv_count; i++ ) {
      Ncv = N.CV(i);
      for ( j = 0; j < m_cv_count[dir]; j++ ) {
        Scv = (dir) ? CV(i+span_index,j) : CV(j,i+span_index);
        for ( k = 0; k < Scvsize; k++ )
          *Ncv++ = *Scv++;
      }
    }
    N.Evaluate( c, 0, N.Dimension(), nurbscrv->m_cv );
    crv = nurbscrv;
  }
  return crv;
}

// Converts a surface to a high degree NURBS curve.
// Use FromCurve to convert back to a surface.
static ON_NurbsCurve* ToCurve( const ON_NurbsSurface& srf, int dir, 
                               ON_NurbsCurve* crv )
{
  double* tmp_cv = NULL;
  if ( dir < 0 || dir > 1 )
    return NULL;
  if ( !srf.m_cv )
    return NULL;
  if ( !crv )
    crv = new ON_NurbsCurve();
  int srf_cv_size = srf.CVSize();
  if ( !crv->Create(
          srf_cv_size*srf.m_cv_count[1-dir], // dim
          false, // is_rat
          srf.m_order[dir],
          srf.m_cv_count[dir]
          ) )
    return NULL;
  if ( crv->m_cv == srf.m_cv )
  {
    tmp_cv = (double*)onmalloc( crv->m_dim*crv->m_cv_stride*sizeof(tmp_cv[0]) );
    crv->m_cv = tmp_cv;
  }
  double* pdst;
  const double* psrc;
  int i, j;
  size_t sz = srf_cv_size*sizeof(pdst[0]);
  for ( i = 0; i < srf.m_cv_count[dir]; i++ )
  {
    pdst = crv->CV(i);
    psrc = dir ? srf.CV(0,i) : srf.CV(i,0);
    for ( j = 0; j < srf.m_cv_count[1-dir]; j++ )
    {
      memcpy( pdst, psrc, sz );
      psrc += srf.m_cv_stride[1-dir];
      pdst += srf_cv_size;
    }
  }
  if ( tmp_cv )
  {
    crv->m_cv = srf.m_cv;
    memcpy( crv->m_cv, tmp_cv, crv->m_dim*crv->m_cv_stride*sizeof(tmp_cv[0]) );
    onfree(tmp_cv);
  }
  if ( crv->m_knot != srf.m_knot[dir] )
    memcpy( crv->m_knot, srf.m_knot[dir], crv->KnotCount()*sizeof(crv->m_knot[0]) );
  return crv;
}

// Converts the curve created in ToCurve back into a surface.
// The "srf" parameter must be the surface passed to ToCurve().
static ON_BOOL32 FromCurve( ON_NurbsCurve& crv, 
                                   ON_NurbsSurface& srf, 
                                   int dir )
{
  srf.DestroySurfaceTree();
  crv.DestroyCurveTree();
  if ( dir < 0 || dir > 1 )
    return false;
  if ( !crv.m_cv )
    return false;
  if ( crv.m_is_rat )
    return false;
  int srf_cv_size = srf.CVSize();
  if ( srf.m_cv_count[1-dir]*srf_cv_size != crv.m_dim )
    return false;
  if ( srf.m_cv_capacity > 0 && srf.m_cv && srf.m_cv != crv.m_cv )
  {
    onfree( srf.m_cv );
  }
  srf.m_cv_capacity = crv.m_cv_capacity;
  srf.m_cv = crv.m_cv;
  crv.m_cv_capacity = 0;
  crv.m_cv = 0;
  if ( srf.m_knot_capacity[dir] > 0 && srf.m_knot[dir] && srf.m_knot[dir] != crv.m_knot )
  {
    onfree(srf.m_knot[dir]);
  }
  srf.m_order[dir] = crv.m_order;
  srf.m_cv_count[dir] = crv.m_cv_count;
  srf.m_knot_capacity[dir] = crv.m_knot_capacity;
  srf.m_knot[dir] = crv.m_knot;
  crv.m_knot_capacity = 0;
  crv.m_knot = 0;
  srf.m_cv_stride[dir] = crv.m_cv_stride;
  srf.m_cv_stride[1-dir] = srf_cv_size;
  return true;
}

ON_BOOL32 ON_NurbsSurface::Trim(
       int dir,
       const ON_Interval& domain
       )
{
  ON_BOOL32 rc = false;
  if ( dir < 0 || dir > 1 )
    return false;
  ON_Interval current_domain = Domain(dir);
  if ( current_domain[0] == ON_UNSET_VALUE && current_domain[1] == ON_UNSET_VALUE )
    current_domain = domain;
  ON_Interval trim_domain;
  trim_domain.Intersection(domain, Domain(dir) );
  if ( !trim_domain.IsIncreasing() )
    return false;
  if (    trim_domain[0] == current_domain[0] 
       && trim_domain[1] == current_domain[1] )
    return true;

  DestroySurfaceTree();

  ON_NurbsCurve crv;
  if ( ToCurve(*this,dir,&crv) )
  {
    rc = crv.Trim( trim_domain );
    if ( rc )
      rc = FromCurve( crv, *this, dir );
  }

  return true;
}

bool ON_NurbsSurface::Extend(
      int dir,
      const ON_Interval& domain
      )
{

  bool rc = false;
  if ( dir < 0 || dir > 1 )
    return false;
  if (IsClosed(dir)) return false;

  ON_NurbsCurve crv;
  if ( ToCurve(*this,dir,&crv) )
  {
    rc = crv.Extend( domain );
    FromCurve( crv, *this, dir );
  }

  if (rc){
    DestroySurfaceTree();
  }
  return rc;

}

ON_BOOL32 ON_NurbsSurface::Split(
       int dir,
       double c,
       ON_Surface*& west_or_south_side,
       ON_Surface*& east_or_north_side
       ) const
{
  if ( dir < 0 || dir > 1 )
    return false;
  if ( !Domain(dir).Includes( c, true ) )
    return false;
  ON_NurbsSurface* left_srf = 0;
  ON_NurbsSurface* right_srf = 0;

  if ( west_or_south_side )
  {
    left_srf = ON_NurbsSurface::Cast( west_or_south_side );
    if ( !left_srf )
      return false;
    left_srf->DestroySurfaceTree();
  }

  if ( east_or_north_side )
  {
    right_srf = ON_NurbsSurface::Cast( east_or_north_side );
    if ( !right_srf )
      return false;
    right_srf->DestroySurfaceTree();
  }

  ON_NurbsCurve crv, left_crv, right_crv;
  if ( !ToCurve( *this, dir, &crv ) )
    return false;
  ON_Curve *left_side, *right_side;
  left_side = &left_crv;
  right_side = &right_crv;
  if ( !crv.Split( c, left_side, right_side ) )
    return false;
  
  if ( !left_srf )
    left_srf = new ON_NurbsSurface();

  if ( left_srf != this )
  {
    left_srf->m_dim = m_dim;
    left_srf->m_is_rat = m_is_rat;
    left_srf->m_order[1-dir] = m_order[1-dir];
    left_srf->m_cv_count[1-dir] = m_cv_count[1-dir];
    left_srf->ReserveKnotCapacity(1-dir,KnotCount(1-dir));
    memcpy( left_srf->m_knot[1-dir], m_knot[1-dir], KnotCount(1-dir)*sizeof(m_knot[1-dir][0] ) );
  }

  if ( !FromCurve( left_crv, *left_srf, dir ) )
  {
    if ( left_srf != this && left_srf !=  west_or_south_side )
      delete left_srf;
    else
      left_srf->Destroy();

    return false;
  }

  if ( !right_srf )
    right_srf = new ON_NurbsSurface();

  if ( right_srf != this )
  {
    right_srf->m_dim = m_dim;
    right_srf->m_is_rat = m_is_rat;
    right_srf->m_order[1-dir] = m_order[1-dir];
    right_srf->m_cv_count[1-dir] = m_cv_count[1-dir];
    right_srf->ReserveKnotCapacity(1-dir,KnotCount(1-dir));
    memcpy( right_srf->m_knot[1-dir], m_knot[1-dir], KnotCount(1-dir)*sizeof(m_knot[1-dir][0] ) );
  }

  if ( !FromCurve( right_crv, *right_srf, dir ) )
  {
    if ( left_srf != this && left_srf !=  west_or_south_side )
      delete left_srf;
    else
      left_srf->Destroy();

    if ( right_srf != this && right_srf !=  east_or_north_side )
      delete right_srf;
    else
      right_srf->Destroy();

    return false;
  }

  if ( !west_or_south_side)
    west_or_south_side = left_srf;

  if ( !east_or_north_side)
    east_or_north_side = right_srf;

  return true;  
}


int 
ON_NurbsSurface::GetNurbForm( // returns 0: unable to create NURBS representation
                 //            with desired accuracy.
                 //         1: success - returned NURBS parameterization
                 //            matches the surface's to wthe desired accuracy
                 //         2: success - returned NURBS point locus matches
                 //            the surfaces's to the desired accuracy but, on
                 //            the interior of the surface's domain, the 
                 //            surface's parameterization and the NURBS
                 //            parameterization may not match to the 
                 //            desired accuracy.
      ON_NurbsSurface& srf,
      double // tolerance
      ) const
{
  // 4 May 2007 Dale Lear
  //   I'm replacing the call to operator= with a call to
  //   ON_NurbsSurfaceCopyHelper().  The operator= call
  //   was copying userdata and that does not happen for
  //   any other GetNurbForm overrides.  Copying userdata
  //   in GetNurbForm is causing trouble in Make2D and 
  //   other places that are creating NURBS copies in
  //   worker memory pools.

  //srf = *this; // copied user data
  ON_NurbsSurfaceCopyHelper(*this,srf); // does not copy user data
  return 1;
}

ON_Surface* ON_NurbsSurface::Offset(
      double offset_distance, 
      double tolerance, 
      double* max_deviation
      ) const
{
  // 3rd party developers who want to enhance openNURBS
  // may provide a working offset here.
  return NULL;
}


ON_BOOL32 ON_NurbsSurface::IsPlanar(
      ON_Plane* plane,
      double tolerance
      ) const
{
  ON_Plane pln;
  double d;
  ON_3dPoint center, cv;
  ON_3dVector normal, du, dv;
  ON_Interval udom = Domain(0);
  ON_Interval vdom = Domain(1);
  ON_BOOL32 rc = EvNormal( udom.ParameterAt(0.5), vdom.ParameterAt(0.5), center, du, dv, normal );
  if ( rc && normal.Length() < 0.9 )
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

    // 7 July 2006 Dale Lear
    //    Add a slight bias for coordinate axes planes.
    //    This appears to do more good than harm.  The
    //    issue is keeping customers from getting alarmed
    //    when a coordinate they know is "zero" turns out to
    //    be 1e-23.
    if (    fabs(pln.zaxis.x) <= ON_ZERO_TOLERANCE 
         && fabs(pln.zaxis.y) <= ON_ZERO_TOLERANCE
         && fabs(fabs(pln.zaxis.z)-1.0) <= ON_SQRT_EPSILON
       )
    {
      pln.xaxis.z = 0.0;
      pln.yaxis.z = 0.0;
      pln.zaxis.x = 0.0;
      pln.zaxis.y = 0.0;
      pln.zaxis.z = (pln.zaxis.z<0.0) ? -1.0 : 1.0;
      pln.UpdateEquation();
    }
    else if (    fabs(pln.zaxis.y) <= ON_ZERO_TOLERANCE 
              && fabs(pln.zaxis.z) <= ON_ZERO_TOLERANCE
              && fabs(fabs(pln.zaxis.x)-1.0) <= ON_SQRT_EPSILON
            )
    {
      pln.xaxis.x = 0.0;
      pln.yaxis.x = 0.0;
      pln.zaxis.y = 0.0;
      pln.zaxis.z = 0.0;
      pln.zaxis.x = (pln.zaxis.x<0.0) ? -1.0 : 1.0;
      pln.UpdateEquation();
    }
    else if (    fabs(pln.zaxis.z) <= ON_ZERO_TOLERANCE 
              && fabs(pln.zaxis.x) <= ON_ZERO_TOLERANCE
              && fabs(fabs(pln.zaxis.y)-1.0) <= ON_SQRT_EPSILON
            )
    {
      pln.xaxis.y = 0.0;
      pln.yaxis.y = 0.0;
      pln.zaxis.z = 0.0;
      pln.zaxis.x = 0.0;
      pln.zaxis.y = (pln.zaxis.y<0.0) ? -1.0 : 1.0;
      pln.UpdateEquation();
    }

    int i, j;
    for ( i = 0; i < m_cv_count[0] && rc; i++ )
    {
      for ( j = 0; j < m_cv_count[1] && rc; j++ )
      {
        GetCV( i, j, cv );
        d = pln.DistanceTo(cv);
        if ( fabs(d) > tolerance )
          rc = false;
      }
    }

    if ( rc && plane )
      *plane = pln;
  }

  return rc;
}

ON_BOOL32 
ON_NurbsSurface::IsClosed( int dir ) const
{
  bool bIsClosed = false;
  if ( dir >= 0 && dir <= 1 && m_dim > 0 )
  {
    if ( ON_IsKnotVectorClamped( m_order[dir], m_cv_count[dir], m_knot[dir] ) )
    {
      const double* corners[4];
      corners[0] = CV(0,0);
      corners[(0==dir)?1:2] = CV(m_cv_count[0]-1,0);
      corners[(0==dir)?2:1] = CV(0,m_cv_count[1]-1);
      corners[3] = CV(m_cv_count[0]-1,m_cv_count[1]-1);
      if (    ON_PointsAreCoincident(m_dim,m_is_rat,corners[0],corners[1])
           && ON_PointsAreCoincident(m_dim,m_is_rat,corners[2],corners[3])
           && ON_IsPointGridClosed( m_dim, m_is_rat, m_cv_count[0], m_cv_count[1], m_cv_stride[0], m_cv_stride[1], m_cv, dir ) 
         )
      {
        bIsClosed = true;
      }
    }
    else if ( IsPeriodic(dir) )
    {
      bIsClosed = true;
    }
  }
  return bIsClosed;
}


ON_BOOL32 
ON_NurbsSurface::ChangeSurfaceSeam( 
						int dir,
            double t 
            )
{
	bool rc = true;

  if ( dir < 0 || dir > 1 )
    return false;

  ON_Interval current_domain = Domain(dir);
	if( !current_domain.Includes( t) )
		rc = false;
	
	if(rc && IsClosed(dir) ){
		DestroySurfaceTree();
		ON_NurbsCurve crv;
		rc = ToCurve(*this, dir, &crv)!=NULL;
		if(rc)
			rc = crv.ChangeClosedCurveSeam(t)!=0;
		rc = FromCurve(crv,*this, dir) && rc;
	}

	return rc;
}

ON_BOOL32 
ON_NurbsSurface::IsPeriodic( int dir ) const
{
  bool bIsPeriodic = false;
  if ( dir >= 0 && dir <= 1 )
  {
    int k;
    bIsPeriodic = ON_IsKnotVectorPeriodic( m_order[dir], m_cv_count[dir], m_knot[dir] );
    if ( bIsPeriodic ) 
    {
      const double *cv0, *cv1;
      int i0 = m_order[dir]-2;
      int i1 = m_cv_count[dir]-1;
      for ( k = 0; k < m_cv_count[1-dir]; k++ )
      {
        cv0 = (dir)?CV(k,i0):CV(i0,k);
        cv1 = (dir)?CV(k,i1):CV(i1,k);
        for ( /*empty*/; i0 >= 0; i0--, i1-- ) 
        {
          if ( false == ON_PointsAreCoincident( m_dim, m_is_rat, cv0, cv1 ) )
            return false;
          cv0 -= m_cv_stride[dir];
          cv1 -= m_cv_stride[dir];      
        }
      }
    }
  }
  return bIsPeriodic;
}

bool ON_NurbsSurface::GetNextDiscontinuity( 
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
  int tmp_hint[2];
  int tmp_dtype=0;

  double d, tmp_t;

  ON_2dVector st;
  ON_Interval span_domain;
  ON_3dVector Vm[6], Vp[6];
  ON_3dVector Tm, Tp, Km, Kp;
  ON_3dVector& D1m = Vm[1+dir];
  ON_3dVector& D2m = Vm[3+2*dir];
  ON_3dVector& D1p = Vp[1+dir];
  ON_3dVector& D2p = Vp[3+2*dir];

  int ki;
  if ( !hint )
  {
    tmp_hint[0] = 0;
    tmp_hint[1] = 0;
    hint = &tmp_hint[0];
  }
  if ( !dtype )
    dtype = &tmp_dtype;
  if ( !t )
    t = &tmp_t;
  
  if ( c == ON::C0_continuous )
    return false;

  if ( c == ON::C0_locus_continuous )
  {
    return ON_Surface::GetNextDiscontinuity( 
      dir, c, t0, t1, t, hint, dtype, 
      cos_angle_tolerance, curvature_tolerance );
  }
  if ( t0 == t1 )
    return false;

  // First test for parametric discontinuities.  If none are found
  // then we will look for locus discontinuities at ends
  if ( m_order[dir] <= 2 )
    c = ON::PolylineContinuity(c);  // no need to look a zero 2nd derivatives
  const ON::continuity input_c = c; // saved so we can tell if "locus" needs to be dealt with
  c = ON::ParametricContinuity(c);  // strips "locus" from c

  bool bEv2ndDer    = ( c == ON::C2_continuous || c == ON::G2_continuous || c == ON::Gsmooth_continuous );
  bool bTestKappa   = ( bEv2ndDer && c != ON::C2_continuous );
  bool bTestTangent = ( bTestKappa || c == ON::G1_continuous );

  int delta_ki = 1;
  int delta = ((bEv2ndDer) ? 3 : 2) - m_order[dir];
  if ( ON::Cinfinity_continuous == c )
    delta = 0;

  ki = ON_NurbsSpanIndex(m_order[dir],m_cv_count[dir],m_knot[dir],t0,1,*hint);
  double segtol = (fabs(m_knot[dir][ki]) + fabs(m_knot[dir][ki+1]) + fabs(m_knot[dir][ki+1]-m_knot[dir][ki]))*ON_SQRT_EPSILON;

  if ( t0 < t1 )
  {
    int ii = ki+m_order[dir]-2;
    if ( t0 < m_knot[dir][ii+1] && t1 > m_knot[dir][ii+1] && (m_knot[dir][ii+1]-t0) <= segtol && ii+2 < m_cv_count[dir] )
    {
      t0 = m_knot[dir][ii+1];
      ki = ON_NurbsSpanIndex(m_order[dir],m_cv_count[dir],m_knot[dir],t0,1,*hint);
    }
    *hint = ki;
    ki += m_order[dir]-2;
    while (ki < m_cv_count[dir]-1 && m_knot[dir][ki] <= t0) 
      ki += delta_ki;
    if (ki >= m_cv_count[dir]-1) 
    {
      if ( input_c != c && t0 < m_knot[dir][m_cv_count[dir]-1] && t1 >= m_knot[dir][m_cv_count[dir]-1] )
      {
        // have to do locus end test
        return ON_Surface::GetNextDiscontinuity( dir, input_c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
      }
      return false;
    }
  }
  else
  {
    // (t0 > t1) work backwards
    int ii = ki+m_order[dir]-2;
    if ( t0 > m_knot[dir][ii] && t1 < m_knot[dir][ii] && (t0-m_knot[dir][ii]) <= segtol && ii > m_order[dir]-2 )
    {
      t0 = m_knot[dir][ii];
      ki = ON_NurbsSpanIndex(m_order[dir],m_cv_count[dir],m_knot[dir],t0,1,*hint);
    }

    *hint = ki;
    ki += m_order[dir]-2;
    while (ki < m_order[dir]-2 && m_knot[dir][ki] >= t0) 
      ki--;
    if (ki <= m_order[dir]-2) 
    {
      if ( input_c != c && t0 > m_knot[dir][m_order[dir]-2] && t1 < m_knot[dir][m_order[dir]-2] )
      {
        // have to do locus end test
        return ON_Surface::GetNextDiscontinuity( dir,input_c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
      }
      return false;
    }
    delta_ki = -1;
    delta = -delta;
  }
  
  while (m_knot[dir][ki] < t1) 
  {
    if ( delta_ki > 0 )
    {
      // t0 < t1 case
      while (ki < m_cv_count[dir]-1 && m_knot[dir][ki] == m_knot[dir][ki+1])
        ki++;
      if (ki >= m_cv_count[dir]-1) 
        break;    
    }
    else
    {
      // t0 > t1 case
      // 20 March 2003 Dale Lear:
      //     Added to make t0 > t1 case work
      while (ki > m_order[dir]-2 && m_knot[dir][ki] == m_knot[dir][ki-1])
        ki--;
      if (ki <= m_order[dir]-2) 
        break;    
    }

    if (m_knot[dir][ki] == m_knot[dir][ki+delta]) 
    {  
      if ( ON::Cinfinity_continuous == c )
      {
        // Cinfinity_continuous is treated as asking for the next knot
        *dtype = 3;
        *t = m_knot[dir][ki];
        return true;
      }
      
      st[dir] = m_knot[dir][ki];

      int j, j0=0, otherki;
      for ( otherki = m_order[1-dir]-2; otherki < m_cv_count[1-dir]-1; otherki++ )
      {
        span_domain.Set(m_knot[1-dir][otherki],m_knot[1-dir][otherki+1]);
        for ( j = j0; j <= 2; j++ )
        {
          st[1-dir] = span_domain.ParameterAt(0.5*j);

          Evaluate( st.x, st.y, bEv2ndDer?2:1, 3, &Vm[0].x, 3, hint);
          Evaluate( st.x, st.y, bEv2ndDer?2:1, 3, &Vp[0].x, 1, hint);

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
              *t = m_knot[dir][ki];
              return true;
            }
            else if ( bTestKappa )
            {
              bool bIsCurvatureContinuous = ( ON::Gsmooth_continuous == c)
                ? ON_IsGsmoothCurvatureContinuous(Km,Kp,cos_angle_tolerance,curvature_tolerance)
                : ON_IsG2CurvatureContinuous(Km,Kp,cos_angle_tolerance,curvature_tolerance);
              if ( !bIsCurvatureContinuous )
              {
                *dtype = 2;
                *t = m_knot[dir][ki];
                return true;
              }
            }
          }
          else
          {
            if ( !(D1m-D1p).IsTiny(D1m.MaximumCoordinate()*ON_SQRT_EPSILON) )
            {
              *dtype = 1;
              *t = m_knot[dir][ki];
              return true;
            }
            else if ( bEv2ndDer )
            {
              if ( !(D2m-D2p).IsTiny(D2m.MaximumCoordinate()*ON_SQRT_EPSILON) )
              {
                *dtype = 2;
                *t = m_knot[dir][ki];
                return true;
              }         
            }
          }

        }
        j0 = 1;
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
    rc = ON_Surface::GetNextDiscontinuity( dir, input_c, t0, t1, t, hint, dtype, 
                                    cos_angle_tolerance, curvature_tolerance );
  }

  return rc;
}


ON_BOOL32
ON_NurbsSurface::IsSingular( // true if surface side is collapsed to a point
      int side   // side of parameter space to test
                 // 0 = south, 1 = east, 2 = north, 3 = west
      ) const
{
  bool rc = false;
  const double* points = 0;
  int point_count = 0;
  int point_stride = 0;

  switch ( side ) 
  {
  case 0: // south
    rc = IsClamped(1,0)?true:false;
    if ( rc )
    {
      points = CV(0,0);
      point_count = m_cv_count[0];
      point_stride = m_cv_stride[0];
    }
    break;

  case 1: // east
    rc = IsClamped(0,1)?true:false;
    if (rc)
    {
      points = CV(m_cv_count[0]-1,0);
      point_count = m_cv_count[1];
      point_stride = m_cv_stride[1];
    }
    break;

  case 2: // north
    rc  = IsClamped(1,1)?true:false;
    if (rc)
    {
      points = CV(0,m_cv_count[1]-1);
      point_count = m_cv_count[0];
      point_stride = m_cv_stride[0];
    }
    break;

  case 3: // west
    rc = IsClamped( 0, 0 )?true:false;
    if (rc) 
    {
      points = CV(0,0);
      point_count = m_cv_count[1];
      point_stride = m_cv_stride[1];
    }
    break;

  default:
    rc = false;
    break;
  }

  if (rc)
    rc = ON_PointsAreCoincident(m_dim,m_is_rat,point_count,point_stride,points);

  return rc;
}


ON_BOOL32 
ON_NurbsSurface::SetWeight( int i, int j, double w )
{
  DestroySurfaceTree();

  ON_BOOL32 rc = false;
  if ( m_is_rat ) {
    double* cv = CV(i,j);
    if (cv) {
      cv[m_dim] = w;
      rc = true;
    }
  }
  else if ( w == 1.0 ) {
    rc = true;
  }
  return rc;
}

ON_BOOL32 
ON_NurbsSurface::SetCV( int i, int j, ON::point_style style, const double* Point )
{
  DestroySurfaceTree();

  ON_BOOL32 rc = true;
  int k;
  double w;

  double* cv = CV(i,j);
  if ( !cv )
    return false;

  switch ( style ) {

  case ON::not_rational:  // input Point is not rational
    memcpy( cv, Point, m_dim*sizeof(*cv) );
    if ( IsRational() ) {
      // NURBS surface is rational - set weight to one
      cv[m_dim] = 1.0;
    }
    break;

  case ON::homogeneous_rational:  // input Point is homogeneous rational
    if ( IsRational() ) {
      // NURBS surface is rational
      memcpy( cv, Point, (m_dim+1)*sizeof(*cv) );
    }
    else {
      // NURBS surface is not rational
      w = (Point[m_dim] != 0.0) ? 1.0/Point[m_dim] : 1.0;
      for ( k = 0; k < m_dim; k++ ) {
        cv[k] = w*Point[k];
      }
    }
    break;

  case ON::euclidean_rational:  // input Point is euclidean rational
    if ( IsRational() ) {
      // NURBS surface is rational - convert euclean point to homogeneous form
      w = Point[m_dim];
      for ( k = 0; k < m_dim; k++ )
        cv[k] = w*Point[k];  // 22 April 2003 - bug fix [i] to [k]
      cv[m_dim] = w;
    }
    else {
      // NURBS surface is not rational
      memcpy( cv, Point, m_dim*sizeof(*cv) );
    }
    break;

  case ON::intrinsic_point_style:  // input Point is euclidean rational
    memcpy( cv, Point, CVSize()*sizeof(*cv) );
    break;

  default:
    rc = false;
    break;
  }
  return rc;
}

ON_BOOL32 
ON_NurbsSurface::SetCV( int i, int j, const ON_3dPoint& point )
{
  DestroySurfaceTree();

  ON_BOOL32 rc = false;
  double* cv = CV(i,j);
  if ( cv ) {
    cv[0] = point.x;
    if ( m_dim > 1 ) {
      cv[1] = point.y;
      if ( m_dim > 2 )
        cv[2] = point.z;
    }
    if ( m_is_rat ) {
      cv[m_dim] = 1.0;
    }
    rc = true;
  }
  return rc;
}

ON_BOOL32 
ON_NurbsSurface::SetCV( int i, int j, const ON_4dPoint& point )
{
  DestroySurfaceTree();

  ON_BOOL32 rc = false;
  double* cv = CV(i,j);
  if ( cv ) {
    if ( m_is_rat ) {
      cv[0] = point.x;
      if ( m_dim > 1 ) {
        cv[1] = point.y;
        if ( m_dim > 2 )
          cv[2] = point.z;
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
      }
    }
  }
  return rc;
}

ON_BOOL32 
ON_NurbsSurface::GetCV( int i, int j, ON::point_style style, double* Point ) const
{
  const double* cv = CV(i,j);
  if ( !cv )
    return false;
  int dim = Dimension();
  double w = ( IsRational() ) ? cv[dim] : 1.0;
  switch(style) {
  case ON::euclidean_rational:
    Point[dim] = w;
    // no break here
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
  default:
    return false;
  }
  return true;
}

ON_BOOL32 
ON_NurbsSurface::GetCV( int i, int j, ON_3dPoint& point ) const
{
  ON_BOOL32 rc = false;
  const double* cv = CV(i,j);
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
ON_NurbsSurface::GetCV( int i, int j, ON_4dPoint& point ) const
{
  ON_BOOL32 rc = false;
  const double* cv = CV(i,j);
  if ( cv ) {
    point.x = cv[0];
    point.y = (m_dim>1)? cv[1] : 0.0;
    point.z = (m_dim>2)? cv[2] : 0.0;
    point.w = (m_is_rat) ? cv[m_dim] : 1.0;
    rc = true;
  }
  return rc;
}

ON_BOOL32 
ON_NurbsSurface::SetKnot( int dir, int knot_index, double k )
{
  DestroySurfaceTree();
  if ( dir ) dir = 1;
  if ( knot_index < 0 || knot_index >= KnotCount(dir) )
    return false;
  m_knot[dir][knot_index] = k;
  return true;
}

bool ON_NurbsSurface::IsContinuous(
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
  // TODO: speed up by avoiding evaluation at non-multi knots
  return ON_Surface::IsContinuous( desired_continuity, s, t, hint,
                                    point_tolerance, d1_tolerance, d2_tolerance,
                                    cos_angle_tolerance, curvature_tolerance );
}

ON_BOOL32
ON_NurbsSurface::Reverse(int dir)
{
  if (dir < 0 || dir > 1) return false;
  DestroySurfaceTree();
  ON_BOOL32 rc0 = ON_ReverseKnotVector( m_order[dir], m_cv_count[dir], m_knot[dir] );
  ON_BOOL32 rc1 = ON_ReversePointGrid( 3, m_is_rat, m_cv_count[0], m_cv_count[1], m_cv_stride[0], m_cv_stride[1], m_cv, dir );
  return rc0 && rc1;
}

ON_BOOL32
ON_NurbsSurface::Transpose()
{
  DestroySurfaceTree();
  int i;
  // transpose CV grid
  i = m_order[0]; m_order[0] = m_order[1]; m_order[1] = i;
  i = m_cv_count[0]; m_cv_count[0] = m_cv_count[1]; m_cv_count[1] = i;
  i = m_cv_stride[0]; m_cv_stride[0] = m_cv_stride[1]; m_cv_stride[1] = i;

  // swap knot vectors
  i = m_knot_capacity[0]; m_knot_capacity[0] = m_knot_capacity[1]; m_knot_capacity[1] = i;
  double* k = m_knot[0]; m_knot[0] = m_knot[1]; m_knot[1] = k;
  return true;
}

ON_BOOL32
ON_NurbsSurface::SwapCoordinates( int i, int j )
{
  DestroySurfaceTree();
  ON_BOOL32 rc = true;
  int k;
  if ( m_cv_count[0] <= m_cv_count[1] ) {
    for ( k = 0; k < m_cv_count[0]; k++ ) {
      if ( !ON_SwapPointListCoordinates( m_cv_count[1], m_cv_stride[1], CV(k,0), i, j ) )
        rc = false;
    }
  }
  else {
    for ( k = 0; k < m_cv_count[1]; k++ ) {
      if ( !ON_SwapPointListCoordinates( m_cv_count[0], m_cv_stride[0], CV(0,k), i, j ) )
        rc = false;
    }
  }
  return rc;
}

ON_BOOL32 ON_NurbsSurface::SetCVRow(
       int row_index,
       const ON_3dPoint& point
       )
{
  DestroySurfaceTree();
  int i;

  if ( row_index < 0 || row_index > m_cv_count[1] )
    return false;

  for ( i = 0; i < m_cv_count[0]; i++ ) {
    if ( !SetCV( i, row_index, point ) )
      return false;
  }

  return true;
}

ON_BOOL32 ON_NurbsSurface::SetCVRow(
       int row_index,
       int v_stride,   // v stride
       const double* v // values (same dim and is_rat as surface)
       )
{
  DestroySurfaceTree();
  int i;
  unsigned int s;
  double* cv;

  if ( row_index < 0 || row_index > m_cv_count[1] )
    return false;
  cv = CV(0,row_index);
  if ( !cv )
    return false;
  if ( v_stride < CVSize() )
    return false;
  s = CVSize()*sizeof(*cv);
  if ( s < m_dim*sizeof(*cv) )
    return false;

  for ( i = 0; i < m_cv_count[0]; i++ ) {
    memcpy( cv, v, s );
    cv += m_cv_stride[0];
    v += v_stride;
  }

  return true;
}

ON_BOOL32 ON_NurbsSurface::SetCVColumn(
       int col_index,
       const ON_3dPoint& point
       )
{
  DestroySurfaceTree();
  int j;

  if ( col_index < 0 || col_index > m_cv_count[0] )
    return false;

  for ( j = 0; j < m_cv_count[1]; j++ ) {
    if ( !SetCV( col_index, j, point ) )
      return false;
  }

  return true;
}

ON_BOOL32 ON_NurbsSurface::SetCVColumn(
       int col_index,
       int v_stride,   // v stride
       const double* v // values (same dim and is_rat as surface)
       )
{
  DestroySurfaceTree();
  int i;
  unsigned int s;
  double* cv;

  if ( col_index < 0 || col_index > m_cv_count[0] )
    return false;
  cv = CV(col_index,0);
  if ( !cv )
    return false;
  if ( v_stride < CVSize() )
    return false;
  s = CVSize()*sizeof(*cv);
  if ( s < m_dim*sizeof(*cv) )
    return false;

  for ( i = 0; i < m_cv_count[1]; i++ ) {
    memcpy( cv, v, s );
    cv += m_cv_stride[1];
    v += v_stride;
  }

  return true;
}

double ON_NurbsSurface::GrevilleAbcissa(
         int dir,    // dir
         int gindex  // index (0 <= index < CVCount(dir)
         ) const
{
  if (dir) 
    dir = 1;
  return ON_GrevilleAbcissa( m_order[dir], m_knot[dir] + gindex );
}

bool ON_NurbsSurface::GetGrevilleAbcissae( // see ON_GetGrevilleAbcissae() for details
         int dir,          // dir
         double* g         // g[cv1-cv0]
         ) const
{
  if (dir) 
    dir = 1;
  return ON_GetGrevilleAbcissae( m_order[dir], m_cv_count[dir], m_knot[dir], false, g );
}

bool ON_NurbsSurface::SetClampedGrevilleKnotVector(
         int dir,        // dir
         int g_stride,   // g_stride
         const double* g // g[], Greville abcissa
         )
{
  DestroySurfaceTree();
  if ( !m_knot[dir] && m_order[dir] >= 2 && m_cv_count[dir] >= m_order[dir] )
    ReserveKnotCapacity(dir,KnotCount(dir));
  return ON_GetGrevilleKnotVector( g_stride, g, false, Order(dir), CVCount(dir), m_knot[dir] );
}

bool ON_NurbsSurface::SetPeriodicGrevilleKnotVector(
         int dir,        // dir
         int g_stride,   // g_stride
         const double* g // g[], Greville abcissa
         )
{
  DestroySurfaceTree();
  if ( !m_knot[dir] && m_order[dir] >= 2 && m_cv_count[dir] >= m_order[dir] )
    ReserveKnotCapacity(dir,KnotCount(dir));
  return ON_GetGrevilleKnotVector( g_stride, g, true, Order(dir), CVCount(dir), m_knot[dir] );
}


bool ON_NurbsSurface::ZeroCVs()
{
  DestroySurfaceTree();
  bool rc = false;
  int i, j;
  if ( m_cv ) {
    if ( m_cv_capacity > 0 ) {
      memset( m_cv, 0, m_cv_capacity*sizeof(*m_cv) );
      if ( m_is_rat ) {
        for ( i = 0; i < m_cv_count[0]; i++ ) for ( j = 0; j < m_cv_count[1]; j++ ) {
          SetWeight( i, j, 1.0 );
        }
      }
      rc = true;
    }
    else {
      double* cv;
      int s = CVSize()*sizeof(*cv);
      j = 0;
      for ( i = 0; i < m_cv_count[0]; i++ ) for ( j = 0; j < m_cv_count[1]; j++ ) {
        cv = CV(i,j);
        if ( !cv )
          return false;
        memset(cv,0,s);
        if ( m_is_rat )
          cv[m_dim] = 1.0;
      }
      rc = (i>0 && j>0) ? true : false;
    }
  }
  return rc;
}

bool ON_NurbsSurface::IsClamped( // determine if knot vector is clamped
      int dir, // dir 0 = "s", 1 = "t", 2 = both
      int end  // (default =2) end to check: 0 = start, 1 = end, 2 = start and end
      ) const
{
  bool rc = false;
  if ( dir == 0 || dir == 1)
    rc = ON_IsKnotVectorClamped( m_order[dir], m_cv_count[dir], m_knot[dir], end );
  return rc;
}

static void ConvertToCurve( const ON_NurbsSurface& srf, int dir, ON_NurbsCurve& crv )
{
  // DO NOT MAKE THIS FUNCTION PUBLIC - IT IS DELICATE AND DEDICATED TO USE IN THIS FILE

  crv.DestroyCurveTree();
  if (dir)
    dir = 1;
  const int Sdim = srf.CVSize();
  const int n = srf.CVCount(1-dir);
  const int Ndim = Sdim*n;
  const int knot_count = srf.KnotCount(dir);
  int i, j;
  double *Ncv;
  const double *Scv;

  crv.m_dim = Ndim;
  crv.m_is_rat = 0;
  crv.m_order = srf.Order(dir);
  crv.m_cv_count = srf.CVCount(dir);
  crv.m_cv_stride = crv.m_dim;
  crv.ReserveCVCapacity(srf.CVCount(dir)*Ndim);
  crv.ReserveKnotCapacity(srf.KnotCount(dir));

  if ( crv.m_knot != srf.m_knot[dir] && srf.m_knot[dir] ) 
  {
    memcpy( crv.m_knot, srf.m_knot[dir], knot_count*sizeof(crv.m_knot[0]) );
  }

  if ( crv.m_cv != srf.m_cv && srf.m_cv ) 
  {
    if (dir) {
      for ( i = 0; i < crv.m_cv_count; i++ ) {
        Ncv = crv.CV(i);
        for ( j = 0; j < n; j++ ) {
          Scv = srf.CV(j,i);
          memcpy( Ncv, Scv, Sdim*sizeof(*Ncv) );
          Ncv += Sdim;
        }
      }
    }
    else {
      for ( i = 0; i < crv.m_cv_count; i++ ) {
        Ncv = crv.CV(i);
        for ( j = 0; j < n; j++ ) {
          Scv = srf.CV(i,j);
          memcpy( Ncv, Scv, Sdim*sizeof(*Ncv) );
          Ncv += Sdim;
        }
      }
    }
  }
}

static void ConvertFromCurve( ON_NurbsCurve& crv, int dir, ON_NurbsSurface& srf )
{
  // DO NOT MAKE THIS FUNCTION PUBLIC - IT IS DELICATE AND DEDICATED TO USE IN THIS FILE

  crv.DestroyCurveTree();
  srf.DestroySurfaceTree();
  if (dir)
    dir = 1;
  const int Sdim = srf.CVSize();

  srf.m_order[dir]       = crv.m_order;
  srf.m_cv_count[dir]    = crv.m_cv_count;
  srf.m_cv_stride[dir]   = crv.m_cv_stride;
  srf.m_cv_stride[1-dir] = Sdim;

  if ( crv.m_cv ) 
  {
    if (    srf.m_cv 
         && crv.m_cv != srf.m_cv
         && srf.m_cv_capacity > 0 
         && srf.m_cv_capacity <  crv.m_cv_stride*crv.m_cv_count ) 
    {
      // discard surface cvs because there isn't enough room
      onfree( srf.m_cv );
      srf.m_cv = 0;
      srf.m_cv_capacity = 0;
    }

    if ( srf.m_cv )
    {
      // use existing surface cvs
      memcpy( srf.m_cv, crv.m_cv, crv.m_cv_stride*crv.m_cv_count*sizeof(*srf.m_cv) );
    }
    else
    {
      // move curve cvs to surface
      srf.m_cv = crv.m_cv;
      srf.m_cv_capacity = crv.m_cv_capacity;
      crv.m_cv = 0;
      crv.m_cv_capacity  = 0;
    }

    crv.m_cv_stride  = 0;
  }

  if ( crv.m_knot && crv.m_knot != srf.m_knot[dir] ) 
  {
    if ( srf.m_knot_capacity[dir] > 0 ) 
    {
      onfree( srf.m_knot[dir] );
      srf.m_knot[dir] = 0;
      srf.m_knot_capacity[dir] = 0;
    }
    srf.m_knot[dir] = crv.m_knot;
    srf.m_knot_capacity[dir] = crv.m_knot_capacity;
    crv.m_knot = 0;
    crv.m_knot_capacity = 0;
  }
}



bool ON_NurbsSurface::ClampEnd(
          int dir,         // dir 0 = "s", 1 = "t"
          int end// 0 = clamp start, 1 = clamp end, 2 = clamp start and end
          )
{
  DestroySurfaceTree();
  if (dir)
    dir = 1;
  ON_NurbsCurve crv;
  crv.m_knot = m_knot[dir];
  ConvertToCurve(*this,dir,crv);
  bool rc = crv.ClampEnd(end);
  ConvertFromCurve(crv,dir,*this);
  return rc;
}

bool ON_NurbsSurface::InsertKnot(
         int dir,         // dir 0 = "s", 1 = "t"
         double knot_value,
         int knot_multiplicity // default = 1
         )
{
  DestroySurfaceTree();
  bool rc = false;

  if ( (dir == 0 || dir == 1) && IsValid() && knot_multiplicity > 0 && knot_multiplicity < Order(dir) ) 
  {
    ON_Interval domain = Domain( dir );
    if ( knot_value < domain.Min() || knot_value > domain.Max() ) 
    {
      ON_ERROR("ON_NurbsSurface::InsertKnot() knot_value not inside domain.");
    }
    else 
    {
      ON_NurbsCurve crv;
      crv.m_knot = m_knot[dir];
      crv.m_knot_capacity = m_knot_capacity[dir];
      m_knot[dir] = 0;
      m_knot_capacity[dir] = 0;
      crv.ReserveKnotCapacity(CVCount(dir)+knot_multiplicity);
      ConvertToCurve(*this,dir,crv);
      rc = crv.InsertKnot(knot_value,knot_multiplicity);
      ConvertFromCurve(crv,dir,*this);
    }
  }

  return rc;
}

bool ON_NurbsSurface::MakeRational()
{
  if ( !IsRational() ) 
  {
    DestroySurfaceTree();
    ON_BezierSurface b;
    b.m_dim = m_dim;
    b.m_is_rat = m_is_rat;
    b.m_order[0] = m_cv_count[0];
    b.m_order[1] = m_cv_count[1];
    b.m_cv_stride[0] = m_cv_stride[0];
    b.m_cv_stride[1] = m_cv_stride[1];
    b.m_cv = m_cv;
		b.m_cv_capacity = m_cv_capacity;
	  b.MakeRational();
    m_is_rat = b.m_is_rat;
    m_cv_stride[0] = b.m_cv_stride[0];
    m_cv_stride[1] = b.m_cv_stride[1];
    m_cv = b.m_cv;
    b.m_cv = 0;
  }
  return IsRational();
}

bool ON_NurbsSurface::ChangeDimension(
          int desired_dimension  //  desired_dimension
          )
{
  bool rc = false;
  int i, j, k;
  if ( desired_dimension < 1 )
    return false;
  if ( desired_dimension == m_dim )
    return true;

  DestroySurfaceTree();

  if ( desired_dimension < m_dim ) 
  {
    if ( m_is_rat ) {
      double* cv;
      for ( i = 0; i < m_cv_count[0]; i++ ) 
      {
        for ( j = 0; j < m_cv_count[1]; j++ ) 
        {
          cv = CV(i,j);
          cv[desired_dimension] = cv[m_dim];
        }
      }
    }
    m_dim = desired_dimension;
    rc = true;
  }
  else 
  {
    const double* old_cv;
    double* new_cv;
    //const int old_cv_size = m_is_rat ? (m_dim + 1) : m_dim;
    const int old_stride0 = m_cv_stride[0];
    const int old_stride1 = m_cv_stride[1];
    const int cv_size = m_is_rat ? (desired_dimension + 1) : desired_dimension;
    int new_stride0 = old_stride0;
    int new_stride1 = old_stride1;
    if ( cv_size > old_stride0 && cv_size > old_stride1 )
    {      
      new_stride0 = (old_stride0 <= old_stride1) ? cv_size : (cv_size*m_cv_count[1]);
      new_stride1 = (old_stride0 <= old_stride1) ? (cv_size*m_cv_count[0]) : cv_size;
      ReserveCVCapacity(cv_size*m_cv_count[0]*m_cv_count[1]);
    }

    if ( old_stride0 <= old_stride1 )
    {
      for ( j = m_cv_count[1]-1; j >= 0; j-- )
      {
        for ( i = m_cv_count[0]-1; i >= 0; i-- )
        {
          old_cv = m_cv + (old_stride0*i + old_stride1*j);
          new_cv = m_cv + (new_stride0*i + new_stride1*j);
          if ( m_is_rat )
          {
            new_cv[desired_dimension] = old_cv[m_dim];
          }
          for ( k = desired_dimension-1; k >= m_dim; k-- )
          {
            new_cv[k] = 0.0;
          }
          for ( k = m_dim-1; k >= 0; k-- )
          {
            new_cv[k] = old_cv[k];
          }
        }
      }
    }
    else
    {
      for ( i = m_cv_count[0]-1; i >= 0; i-- )
      {
        for ( j = m_cv_count[1]-1; j >= 0; j-- )
        {
          old_cv = m_cv + (old_stride0*i + old_stride1*j);
          new_cv = m_cv + (new_stride0*i + new_stride1*j);
          if ( m_is_rat )
          {
            new_cv[desired_dimension] = old_cv[m_dim];
          }
          for ( k = desired_dimension-1; k >= m_dim; k-- )
          {
            new_cv[k] = 0.0;
          }
          for ( k = m_dim-1; k >= 0; k-- )
          {
            new_cv[k] = old_cv[k];
          }
        }
      }
    }
    m_cv_stride[0] = new_stride0;
    m_cv_stride[1] = new_stride1;
    m_dim = desired_dimension;
    rc = true;
  }
  return rc;
}

bool ON_NurbsSurface::IncreaseDegree(
         int dir,  // dir 0 = "s", 1 = "t"
         int desired_degree  //  desired_degree
         )
{
  DestroySurfaceTree();
  bool rc = false;

  if ( (dir == 0 || dir == 1) && IsValid() && desired_degree >= 1 )
  {
    if ( m_order[dir] == desired_degree+1 )
      rc = true;
    else
    {
      ON_NurbsCurve crv;
      crv.m_knot = m_knot[dir];
      crv.m_knot_capacity = m_knot_capacity[dir];
      m_knot[dir] = 0;
      m_knot_capacity[dir] = 0;
      ConvertToCurve(*this,dir,crv);
      rc = crv.IncreaseDegree(desired_degree);
      ConvertFromCurve(crv,dir,*this);
    }
  }

  return rc;
}

bool ON_NurbsSurface::MakeNonRational()
{
  if ( IsRational() ) 
  {
    DestroySurfaceTree();
    ON_BezierSurface b;
    b.m_dim = m_dim;
    b.m_is_rat = m_is_rat;
    b.m_order[0] = m_cv_count[0];
    b.m_order[1] = m_cv_count[1];
    b.m_cv_stride[0] = m_cv_stride[0];
    b.m_cv_stride[1] = m_cv_stride[1];
    b.m_cv = m_cv;
    b.MakeNonRational();
    m_is_rat = b.m_is_rat;
    m_cv_stride[0] = b.m_cv_stride[0];
    m_cv_stride[1] = b.m_cv_stride[1];
    m_cv = b.m_cv;
    b.m_cv = 0;
  }
  return IsRational() ? false : true;
}

ON_TensorProduct::ON_TensorProduct()
{}

ON_TensorProduct::~ON_TensorProduct()
{}

ON_BOOL32 ON_NurbsSurface::TensorProduct(
      const ON_NurbsCurve& nurbscurveA,
      const ON_NurbsCurve& nurbscurveB,
      ON_TensorProduct& tensor
      )
{
  DestroySurfaceTree();
  //   The resulting surface will satisfy
  // 	 NurbSrf(s,t) = T( NurbA(s), NurbB(t) )
  // 
  //   If you want to understand the relationship between multilinear maps
  //   and tensor products, read chapter 16 of Serge Lang's Algebra book.
  //   The connection between Lang and tensor product nurb surfaces being
  //   that NurbA and NurbB are elements of the module of piecewise polynomial
  //   functions that satisfy certain degree and continuity constraints.

  ON_BOOL32 rc;
	double wA, wB, wC;
	const double *cvA, *cvB;
	double *cvC;
	int i, j, k,  cv_countA, cv_countB, dimA, dimB, dimC, is_ratA, is_ratB, is_ratC;

	dimA = nurbscurveA.Dimension();
	dimB = nurbscurveB.Dimension();
	dimC = tensor.DimensionC();

  if ( tensor.DimensionA() > dimA ) {
    ON_ERROR("ON_NurbsSurface::TensorProduct() - tensor.DimensionA() > dimA");
    return false;
  }
  if ( tensor.DimensionB() > dimB ) {
    ON_ERROR("ON_NurbsSurface::TensorProduct() - tensor.DimensionB() > dimB");
    return false;
  }

	is_ratA = nurbscurveA.IsRational();
	is_ratB = nurbscurveB.IsRational();
	is_ratC = (is_ratA || is_ratB);
  cv_countA = nurbscurveA.CVCount();
  cv_countB = nurbscurveB.CVCount();

  Create( dimC, is_ratC, nurbscurveA.Order(), nurbscurveB.Order(), cv_countA, cv_countB );

  if ( m_knot[0] != nurbscurveA.m_knot )
    memcpy( m_knot[0], nurbscurveA.m_knot, KnotCount(0)*sizeof(*m_knot[0]) );
  if ( m_knot[1] != nurbscurveB.m_knot )
    memcpy( m_knot[1], nurbscurveB.m_knot, KnotCount(1)*sizeof(*m_knot[1]) );

	for (i = 0; i < cv_countA; i++) {
    cvA = nurbscurveA.CV(i);
		for (j = 0; j < cv_countB; j++) {
  		cvB = nurbscurveB.CV(j);
      cvC = CV(i,j);
			wA = (is_ratA) ? cvA[dimA] : 1.0;
			wB = (is_ratB) ? cvB[dimB] : 1.0;
			rc = tensor.Evaluate( (wA == 0.0) ? 0.0 : 1.0/wA, cvA, 
									          (wB == 0.0) ? 0.0 : 1.0/wB, cvB, 
									          cvC );
      if ( !rc )
        return false;
			if (is_ratC) {
  			wC = wA*wB;
        for ( k = 0; k < dimC; k++ )
          *cvC++ *= wC;
        *cvC = wC;
			}
		}
	}
	return true;
}

static
bool ON_MakeDegreesCompatible(
       ON_NurbsCurve& nurbs_curveA,
       ON_NurbsCurve& nurbs_curveB
       )
{
  bool rc = false;
  if ( nurbs_curveA.m_order > nurbs_curveB.m_order )
    rc = nurbs_curveB.IncreaseDegree( nurbs_curveA.Degree() )?true:false;
  else
    rc = nurbs_curveA.IncreaseDegree( nurbs_curveB.Degree() )?true:false;
  return (nurbs_curveA.m_order == nurbs_curveA.m_order);
}

static
bool ON_MakeDomainsCompatible(
       ON_NurbsCurve& nurbs_curveA,
       ON_NurbsCurve& nurbs_curveB
       )
{
  ON_Interval dA = nurbs_curveA.Domain();
  ON_Interval dB = nurbs_curveB.Domain();
  bool rc = false;
  if ( dA.Length() >= dB.Length() )
    rc = (nurbs_curveB.SetDomain(dA[0],dA[1])?true:false);
  else
    rc = (nurbs_curveA.SetDomain(dB[0],dB[1])?true:false);
  return rc;
}

static
bool ON_MakeKnotVectorsCompatible(
       ON_NurbsCurve& nurbs_curveA,
       ON_NurbsCurve& nurbs_curveB
       )
{
  if ( !ON_MakeDegreesCompatible( nurbs_curveA, nurbs_curveB ) )
    return false;
  if ( !ON_MakeDomainsCompatible( nurbs_curveA, nurbs_curveB ) )
    return false;

  const int order = nurbs_curveA.m_order;
  ON_Interval span;
  double ktol, a, b;
  int i, ki, multA, multB;
  int knot_countA = nurbs_curveA.KnotCount();
  int knot_countB = nurbs_curveB.KnotCount();
  int max_knot_capacity = knot_countA + knot_countB - 2*(order-1);

  bool bPeriodic = false;
  if ( nurbs_curveA.IsPeriodic() || nurbs_curveB.IsPeriodic() )
  {
    bPeriodic = true;
    for ( ki = 0; ki < order-2 && bPeriodic; ki++ )
    {
      if ( nurbs_curveA.m_knot[ki] != nurbs_curveB.m_knot[ki] )
        bPeriodic = false;
    }
    int kiA, kiB;
    for (
      kiA = nurbs_curveA.m_cv_count,
      kiB = nurbs_curveB.m_cv_count;
      kiA < knot_countA && kiB < knot_countB && bPeriodic;
      kiA++, kiB++ )
    {
      if ( nurbs_curveA.m_knot[kiA] != nurbs_curveB.m_knot[kiA] )
        bPeriodic = false;
    }
  }
  if ( !bPeriodic )
  {
    if ( !nurbs_curveA.ClampEnd(2) )
      return false;
    if ( !nurbs_curveB.ClampEnd(2) )
      return false;
  }

  ki = order-1; 
  while ( (ki < nurbs_curveA.m_cv_count-1 || ki < nurbs_curveB.m_cv_count-1)
          && nurbs_curveA.m_knot[ki-1] == nurbs_curveB.m_knot[ki-1]
          && ki <= nurbs_curveA.m_cv_count-1
          && ki <= nurbs_curveB.m_cv_count-1
          )
  {
    a = nurbs_curveA.m_knot[ki];
    if ( a == nurbs_curveA.m_knot[ki-1] )
      return false;
    b = nurbs_curveB.m_knot[ki];
    if ( b == nurbs_curveB.m_knot[ki-1] )
      return false;
    multA = ON_KnotMultiplicity( order, nurbs_curveA.m_cv_count, nurbs_curveA.m_knot, ki );
    multB = ON_KnotMultiplicity( order, nurbs_curveB.m_cv_count, nurbs_curveB.m_knot, ki );

    if ( a < b )
    {
      // insert a in nurbs_curveB
      span.Set(nurbs_curveB.m_knot[ki-1], nurbs_curveB.m_knot[ki] );
      ktol = ON_SQRT_EPSILON*(span.Length() + fabs(nurbs_curveB.m_knot[ki-1]) + fabs(nurbs_curveB.m_knot[ki]) );
      if ( a >= span[1] - ktol )
      {
        for ( i = ki; i < ki+multB; i++ )
          nurbs_curveB.m_knot[i] = a;          
      }
      else
      {
        nurbs_curveB.ReserveKnotCapacity( max_knot_capacity );
        nurbs_curveB.InsertKnot( a, multA );
      }
      b = nurbs_curveB.m_knot[ki];
      multB = ON_KnotMultiplicity( order, nurbs_curveB.m_cv_count, nurbs_curveB.m_knot, ki );
    }
    else if ( b < a )
    {
      // insert b in nurbs_curveA
      span.Set(nurbs_curveA.m_knot[ki-1], nurbs_curveA.m_knot[ki] );
      ktol = ON_SQRT_EPSILON*(span.Length() + fabs(nurbs_curveA.m_knot[ki-1]) + fabs(nurbs_curveA.m_knot[ki]) );
      if ( b >= span[1] - ktol )
      {
        for ( i = ki; i < ki+multA; i++ )
          nurbs_curveA.m_knot[i] = b;
      }
      else
      {
        nurbs_curveA.ReserveKnotCapacity( max_knot_capacity );
        nurbs_curveA.InsertKnot( b, multB );
      }
      a = nurbs_curveA.m_knot[ki];
      multA = ON_KnotMultiplicity( order, nurbs_curveA.m_cv_count, nurbs_curveA.m_knot, ki );
    }

    if ( a != b )
      return false;

    if ( a == b )
    {
      if ( multA < multB )
      {
        nurbs_curveA.ReserveKnotCapacity( max_knot_capacity );
        if ( !nurbs_curveA.InsertKnot( a, multB ) )
          return false;
        multA = multB;
      }
      else if ( multB < multA )
      {
        nurbs_curveB.ReserveKnotCapacity( max_knot_capacity );
        if ( !nurbs_curveB.InsertKnot( a, multA ) )
          return false;
        multB = multA;
      }
      ki += multA;
    }
  }

  if ( nurbs_curveA.m_cv_count != nurbs_curveB.m_cv_count )
    return false;
  knot_countA = nurbs_curveA.KnotCount();
  for ( ki = 0; ki < knot_countA; ki++ )
  {
    if ( nurbs_curveA.m_knot[ki] != nurbs_curveB.m_knot[ki] )
      return false;
  }

  return true;
}

int ON_NurbsSurface::CreateRuledSurface(
       const ON_Curve& curveA,
       const ON_Curve& curveB,
       const ON_Interval* curveA_domain,
       const ON_Interval* curveB_domain
       )
{
  DestroySurfaceTree();
  int rcA=1, rcB=1;
  ON_NurbsCurve nurbs_curveA, nurbs_curveB;
  if ( m_cv && m_cv_capacity == 0 )
    nurbs_curveA.m_cv = m_cv;
  if ( m_knot[0] && m_knot_capacity[0] == 0 )
    nurbs_curveA.m_knot = m_knot[0];
  rcA = curveA.GetNurbForm( nurbs_curveA, 0.0, curveA_domain );
  if ( rcA<=0 )
    return 0;
  rcB = curveB.GetNurbForm( nurbs_curveB, 0.0, curveB_domain );
  if ( rcB<=0 )
    return 0;

  if ( !ON_MakeKnotVectorsCompatible( nurbs_curveA, nurbs_curveB ) )
    return false;

  if ( nurbs_curveA.m_cv_count != nurbs_curveB.m_cv_count )
    return 0;
  if ( nurbs_curveA.m_order != nurbs_curveB.m_order )
    return 0;

  int srf_dim = 3;
  if ( nurbs_curveA.Dimension() > srf_dim )
    srf_dim = nurbs_curveA.Dimension();
  if ( nurbs_curveB.Dimension() > srf_dim )
    srf_dim = nurbs_curveB.Dimension();

  if (nurbs_curveA.Dimension() < srf_dim )
    nurbs_curveA.ChangeDimension(srf_dim);
  else if (nurbs_curveB.Dimension() < srf_dim )
    nurbs_curveB.ChangeDimension(srf_dim);

  if ( nurbs_curveA.IsRational() )
    nurbs_curveB.MakeRational();
  else if ( nurbs_curveB.IsRational() )
    nurbs_curveA.MakeRational();

  // reserve enough room in nurbs_curveA.m_cv 
  // for two rows of surface cvs.
  const int is_rat = nurbs_curveA.m_is_rat ? 1 : 0;
  if ( is_rat )
  {
    nurbs_curveA.m_is_rat = 0;
    nurbs_curveA.m_dim++;
  }
  nurbs_curveA.ChangeDimension( 2*nurbs_curveA.m_dim );
  nurbs_curveA.m_dim = srf_dim;
  nurbs_curveA.m_is_rat = is_rat;

  // transfer m_cv and m_knot[0] memory from nurbs_curveA to
  // this nurbs surface.
  if ( m_cv && m_cv_capacity > 0 )
    onfree(m_cv);
  m_cv = nurbs_curveA.m_cv;
  m_cv_capacity = nurbs_curveA.m_cv_capacity;
  nurbs_curveA.m_cv_capacity = 0;

  if ( m_knot[0] && m_knot_capacity[0] > 0 )
    onfree(m_knot[0]);
  m_knot[0] = nurbs_curveA.m_knot;
  m_knot_capacity[0] = nurbs_curveA.m_knot_capacity;
  nurbs_curveA.m_knot_capacity = 0;

  // Fill in linear knots
  ReserveKnotCapacity( 1, 2 );
  m_knot[1][0] = 0.0;
  m_knot[1][1] = 1.0;

  m_dim = srf_dim;
  m_is_rat = nurbs_curveA.m_is_rat;
  m_order[0] = nurbs_curveA.m_order;
  m_order[1] = 2;
  m_cv_count[0] = nurbs_curveA.m_cv_count;
  m_cv_count[1] = 2;
  m_cv_stride[0] = nurbs_curveA.m_cv_stride;
  m_cv_stride[1] = m_cv_stride[0]/2;

  // fill in "B" row of cvs
  for ( int i = 0; i < m_cv_count[0]; i++ )
  {
    SetCV(i,1,ON::intrinsic_point_style, nurbs_curveB.CV(i));
  }

  return ((rcA<=rcB) ? rcB : rcA);
}

static
ON_3dPoint CornerAt( const ON_Surface& srf, int corner )
{
  double s, t;
  switch (corner)
  {
  case 0: // sw
    s = srf.Domain(0)[0];
    t = srf.Domain(1)[0];
    break;
  case 1: // se
    s = srf.Domain(0)[1];
    t = srf.Domain(1)[0];
    break;
  case 2: // ne
    s = srf.Domain(0)[1];
    t = srf.Domain(1)[1];
    break;
  case 3: // nw
    s = srf.Domain(0)[0];
    t = srf.Domain(1)[1];
    break;
  default:
    return ON_UNSET_POINT;
    break;
  }
  return srf.PointAt(s,t);
}

bool ON_NurbsSurface::CollapseSide(
       int side,
       ON_3dPoint point
       )
{
  if ( point == ON_UNSET_POINT )
  {
    point = CornerAt(*this,side);
    if ( point == ON_UNSET_POINT )
      return false;
  }

  if ( !m_cv )
    return false;

  int i0 = 0;
  int i1 = m_cv_count[0];
  int j0 = 0;
  int j1 = m_cv_count[1];

  switch (side)
  {
  case 0: // south
    j1 = j0+1;
    break;
  case 1: // east
    i0 = i1-1;
    break;
  case 2: // north
    j0 = j1-1;
    break;
  case 3: // west
    i1 = i0+1;
    break;
  default:
    return false;
    break;
  }

  if ( i0 >= i1 || j0 >= j1 )
    return false;

  int i, j;
  ON_4dPoint cv;
  for ( i = i0; i < i1; i++ ) for ( j = j0; j < j1; j++ )
  {
    if ( !GetCV(i,j,cv) )
      return false;
    cv.x = point.x*cv.w;
    cv.y = point.y*cv.w;
    cv.z = point.z*cv.w;
    if ( !SetCV(i,j,cv) )
      return false;
  }
  return true;
}

int ON_NurbsSurface::CreateConeSurface(
       ON_3dPoint apex_point,
       const ON_Curve& curve,
       const ON_Interval* curve_domain
       )
{
  DestroySurfaceTree();
  ON_NurbsCurve nurbs_curve;
  if ( m_cv && m_cv_capacity == 0 )
    nurbs_curve.m_cv = m_cv;
  if ( m_knot[0] && m_knot_capacity[0] == 0 )
    nurbs_curve.m_knot = m_knot[0];
  int rc = curve.GetNurbForm( nurbs_curve, 0.0, curve_domain );
  if (rc>0)
  {
    // reserve enough room in nurbs_curve.m_cv 
    // for two rows of surface cvs.
    nurbs_curve.ChangeDimension(3);
    const int is_rat = nurbs_curve.m_is_rat?1:0;
    if ( is_rat )
    {
      nurbs_curve.m_is_rat = 0;
      nurbs_curve.m_dim++;
    }
    nurbs_curve.ChangeDimension( 2*nurbs_curve.m_dim );
    nurbs_curve.m_is_rat = is_rat;
    nurbs_curve.m_dim = 3;

    // transfer m_cv and m_knot[0] memory from nurbs_curve to
    // this nurbs surface.
    if ( m_cv && m_cv_capacity > 0 )
      onfree(m_cv);
    m_cv = nurbs_curve.m_cv;
    m_cv_capacity = nurbs_curve.m_cv_capacity;
    nurbs_curve.m_cv_capacity = 0;

    if ( m_knot[0] && m_knot_capacity[0] > 0 )
      onfree(m_knot[0]);
    m_knot[0] = nurbs_curve.m_knot;
    m_knot_capacity[0] = nurbs_curve.m_knot_capacity;
    nurbs_curve.m_knot_capacity = 0;

    // Fill in linear knots
    ReserveKnotCapacity( 1, 2 );
    m_knot[1][0] = 0.0;
    m_knot[1][1] = 1.0;

    m_dim = 3;
    m_is_rat = is_rat;
    m_order[0] = nurbs_curve.m_order;
    m_order[1] = 2;
    m_cv_count[0] = nurbs_curve.m_cv_count;
    m_cv_count[1] = 2;
    m_cv_stride[0] = nurbs_curve.m_cv_stride;
    m_cv_stride[1] = nurbs_curve.m_cv_stride/2;

    for ( int i = 0; i < m_cv_count[0]; i++ )
    {
      SetCV(i,1,apex_point);
      if ( is_rat )
      {
        double* cv = CV(i,1);
        double w = Weight(i,0);
        cv[0] *= w;
        cv[1] *= w;
        cv[2] *= w;
        cv[3] = w;
      }
    }
  }
  else
    Destroy();
  return rc;
}



ON_NurbsSurface* ON_NurbsSurfaceQuadrilateral( 
             const ON_3dPoint& P, 
             const ON_3dPoint& Q, 
             const ON_3dPoint& R, 
             const ON_3dPoint& S,
             ON_NurbsSurface* nurbs_surface
             )
{
  if ( !nurbs_surface )
    nurbs_surface = new ON_NurbsSurface( 3, false, 2, 2, 2, 2 );
  else
    nurbs_surface->Create( 3, false, 2, 2, 2, 2 );
  nurbs_surface->SetCV(0,0,P);
  nurbs_surface->SetCV(1,0,Q);
  nurbs_surface->SetCV(1,1,R);
  nurbs_surface->SetCV(0,1,S);
  double d1 = P.DistanceTo(Q);
  double d2 = R.DistanceTo(S);
  double d = (d1 >= d2) ? d1 : d2;
  if (d <= ON_ZERO_TOLERANCE )
    d = 1.0;
  nurbs_surface->m_knot[0][0] = 0.0;
  nurbs_surface->m_knot[0][1] = d;
  d1 = P.DistanceTo(S);
  d2 = Q.DistanceTo(R);
  d = (d1 >= d2) ? d1 : d2;
  if (d <= ON_ZERO_TOLERANCE )
    d = 1.0;
  nurbs_surface->m_knot[1][0] = 0.0;
  nurbs_surface->m_knot[1][1] = d;
  return nurbs_surface;
}

ON_BOOL32 ON_NurbsSurface::ConvertSpanToBezier(
    int span_index0,
    int span_index1, 
    ON_BezierSurface& bezier_surface
    ) const
{
  int i, j;
  if ( !m_cv || !m_knot[0] || !m_knot[1] )
    return false;
  if ( span_index0 < 0 ||  span_index0 > m_cv_count[0]-m_order[0] )
    return false;
  if ( span_index1 < 0 ||  span_index1 > m_cv_count[1]-m_order[1] )
    return false;
  i = span_index0+m_order[0]-2;
  if ( m_knot[0][i] >= m_knot[0][i+1] )
    return false;
  j = span_index1+m_order[1]-2;
  if ( m_knot[1][j] >= m_knot[1][j+1] )
    return false;
  {
    ON_NurbsSurface bispan;
    bispan.m_cv = bezier_surface.m_cv;
    bispan.m_cv_capacity = bezier_surface.m_cv_capacity;
    bispan.Create( m_dim, m_is_rat, m_order[0], m_order[1], m_order[0], m_order[1] );
    const int sizeof_cv = CVSize()*sizeof(*bispan.m_cv);
    for ( i = 0; i < m_order[0]; i++ ) for ( j = 0; j < m_order[1]; j++ )
    {
      memcpy( bispan.CV(i,j), CV(span_index0+i,span_index1+j), sizeof_cv ); 
    }
    i = span_index0+m_order[0]-2;
    j = span_index1+m_order[1]-2;
    ON_BOOL32 bClamp = false;
    if ( m_knot[0][span_index0] != m_knot[0][span_index0+m_order[0]-2] )
      bClamp = true;
    if ( m_knot[0][span_index0+m_order[0]-1] != m_knot[0][span_index0+2*m_order[0]-3] )
      bClamp = true;
    if ( m_knot[1][span_index1] != m_knot[1][span_index1+m_order[1]-2] )
      bClamp = true;
    if ( m_knot[1][span_index1+m_order[1]-1] != m_knot[1][span_index1+2*m_order[1]-3] )
      bClamp = true;
    if ( bClamp )
    {
      memcpy( bispan.m_knot[0], m_knot[0]+span_index0, bispan.KnotCount(0)*sizeof(bispan.m_knot[0][0]) );
      memcpy( bispan.m_knot[1], m_knot[1]+span_index1, bispan.KnotCount(1)*sizeof(bispan.m_knot[1][0]) );
      bispan.ClampEnd(1,2);
      bispan.ClampEnd(0,2);
    }
    bezier_surface.m_dim = bispan.m_dim;
    bezier_surface.m_is_rat = bispan.m_is_rat;
    bezier_surface.m_order[0] = bispan.m_order[0];
    bezier_surface.m_order[1] = bispan.m_order[1];
    bezier_surface.m_cv_stride[0] = bispan.m_cv_stride[0];
    bezier_surface.m_cv_stride[1] = bispan.m_cv_stride[1];
    bezier_surface.m_cv = bispan.m_cv;
    bezier_surface.m_cv_capacity = bispan.m_cv_capacity;
    bispan.m_cv = 0;
    bispan.m_cv_capacity = 0;
  }
  return true;
}

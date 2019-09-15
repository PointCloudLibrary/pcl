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

ON_OBJECT_IMPLEMENT(ON_PointGrid,ON_Geometry,"4ED7D4E5-E947-11d3-BFE5-0010830122F0");

ON_3dPoint ON_PointGrid::m_no_point(0.0,0.0,0.0);

ON_PointGrid::ON_PointGrid()
{
  Initialize();
}

ON_PointGrid::ON_PointGrid( int c0, int c1 )
{
  Initialize();
  Create(c0,c1);
}

ON_PointGrid::ON_PointGrid( const ON_PointGrid& src ) : ON_Geometry(src)
{
  *this = src;
}

ON_PointGrid::~ON_PointGrid()
{
  Destroy();
}

int ON_PointGrid::Dimension() const
{
  return 3;
}

int ON_PointGrid::PointCount( int dir ) const
{
  return m_point_count[dir?1:0];
}

int ON_PointGrid::PointCount( void ) const
{
  return m_point_count[0]*m_point_count[1];
}

ON_3dPoint& ON_PointGrid::Point( int i, int j )
{
  return (0 <= i && i < m_point_count[0] && 0 <= j && j < m_point_count[1]) 
         ? m_point[i*m_point_stride0 + j] 
         : m_no_point;
}

ON_3dPoint ON_PointGrid::Point( int i, int j ) const
{
  return (0 <= i && i < m_point_count[0] && 0 <= j && j < m_point_count[1]) 
         ? m_point[i*m_point_stride0 + j] 
         : m_no_point;
}

double* ON_PointGrid::PointArray()
{
  return (m_point_count[0]>0&&m_point_count[1]>0) ? &m_point[0].x : NULL;
}

const double* ON_PointGrid::PointArray() const
{
  return (m_point_count[0]>0&&m_point_count[1]>0) ? &m_point[0].x : NULL;
}

int ON_PointGrid::PointArrayStride(  // point stride in grid direction
      int dir        // dir 0 = "s", 1 = "t"
      ) const
{
  return ((dir) ? 3 : 3*m_point_stride0);
}


ON_BOOL32 ON_PointGrid::SetPoint( int i, int j, const ON_3dPoint& point )
{
  ON_BOOL32 rc = false;
  if ( 0 <= i && i < m_point_count[0] && 0 <= j && j < m_point_count[1] ) {
    m_point[i*m_point_stride0+j] = point;
    rc = true;
  }
  return rc;
}

ON_BOOL32 ON_PointGrid::GetPoint( int i, int j, ON_3dPoint& point ) const
{
  ON_BOOL32 rc = false;
  if ( 0 <= i && i < m_point_count[0] && 0 <= j && j < m_point_count[1] ) {
    point = m_point[i*m_point_stride0+j];
    rc = true;
  }
  return rc;
}

ON_3dPoint* ON_PointGrid::operator[](int i)
{
  return ( 0 <= i && i < m_point_count[0] ) 
         ? m_point.Array() + i*m_point_stride0 : 0;
}

const ON_3dPoint* ON_PointGrid::operator[](int i) const
{
  return ( 0 <= i && i < m_point_count[0] ) 
         ? m_point.Array() + i*m_point_stride0 : 0;
}

ON_BOOL32
ON_PointGrid::Create(
        int point_count0,  // cv count0 (>= order0)
        int point_count1   // cv count1 (>= order1)
        )
{
  if ( point_count0 < 1 )
    return false;
  if ( point_count1 < 1 )
    return false;
  m_point_count[0] = point_count0;
  m_point_count[1] = point_count1;
  m_point_stride0 = m_point_count[1];
  m_point.Reserve(m_point_count[0]*m_point_count[1]);
  return true;
}

void ON_PointGrid::Destroy()
{
  Initialize();
  m_point.SetCapacity(0);
}

void ON_PointGrid::EmergencyDestroy()
{
  // call if memory used by point grid becomes invalid
  m_point_count[0] = 0;
  m_point_count[1] = 0;
  m_point_stride0 = 0;
  m_point.EmergencyDestroy();
}

void ON_PointGrid::Initialize()
{
  m_point_count[0] = 0;
  m_point_count[1] = 0;
  m_point_stride0 = 0;
  m_point.SetCount(0);
}

ON_PointGrid& ON_PointGrid::operator=( const ON_PointGrid& src )
{
  if ( this != &src ) {
    ON_Geometry::operator=(src);
    m_point_count[0] = src.m_point_count[0];
    m_point_count[1] = src.m_point_count[1];
    m_point_stride0 = m_point_count[1];
    m_point.Reserve(PointCount());
    m_point.SetCount(PointCount());
    if ( PointCount() > 0 ) {
      // copy cv array
      if ( m_point_stride0 == src.m_point_stride0 ) {
        memcpy( m_point.Array(), src.m_point.Array(), PointCount()*sizeof(ON_3dPoint) );
      }
      else {
        int i, j;
        for ( i = 0; i < m_point_count[0]; i++ ) for ( j = 0; j < m_point_count[1]; j++ ) {
          m_point[i*m_point_stride0+j] = src[i][j];
        }
      }
    }
  }
  return *this;
}

void ON_PointGrid::Dump( ON_TextLog& dump ) const
{
  dump.Print( "ON_PointGrid size = %d X %d\n",
               m_point_count[0], m_point_count[1] );
  if ( m_point.Count() < 1 ) {
    dump.Print("  NO point array\n");
  }
  else {
    dump.PrintPointGrid( 3, false, m_point_count[0], m_point_count[1], 
                         3*m_point_stride0, 3, 
                         &m_point[0].x,
                         "  point" 
                         );
  }
}

ON_BOOL32 ON_PointGrid::IsValid( ON_TextLog* ) const
{
  ON_BOOL32 rc = false;
  if ( ON_IsValidPointGrid( 3, false, 
                            m_point_count[0], m_point_count[1], 
                            m_point_stride0*3, 3, 
                            &m_point[0].x ) ) 
  {
    if ( m_point.Count() >= m_point_stride0*m_point_count[0] )
      rc = true;
  }
  return rc;
}

ON_BOOL32 ON_PointGrid::GetBBox( // returns true if successful
       double* boxmin,    // minimum
       double* boxmax,    // maximum
       ON_BOOL32 bGrowBox  // true means grow box
       ) const
{
  return ON_GetPointGridBoundingBox( 3, 0, 
            m_point_count[0], m_point_count[1], 
            m_point_stride0*3, 3, &m_point[0].x, 
            boxmin, boxmax, bGrowBox?true:false );
}

bool ON_PointGrid::GetTightBoundingBox(
         ON_BoundingBox& tight_bbox,
         int bGrowBox,
				 const ON_Xform* xform 
         ) const
{
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }
  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }
  
  int i;
  for ( i = 0; i < m_point_count[0]; i++ )
  {
    if ( ON_GetPointListBoundingBox( 3, 0, m_point_count[1], 3, &m_point[i].x, tight_bbox, bGrowBox, xform ) )
      bGrowBox = true;
  }
  return bGrowBox?true:false;
}

ON_BOOL32 ON_PointGrid::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
  return ON_TransformPointGrid( 3, false, 
            m_point_count[0], m_point_count[1],
            m_point_stride0*3, 3,
            Point(0,0), 
            xform );
}

// virtual ON_Geometry::IsDeformable() override
bool ON_PointGrid::IsDeformable() const
{
  return true;
}

// virtual ON_Geometry::MakeDeformable() override
bool ON_PointGrid::MakeDeformable()
{
  return true;
}

ON_BOOL32 ON_PointGrid::SwapCoordinates(
      int i, int j // indices of coords to swap
      )
{
  return ON_SwapPointGridCoordinates( 
            m_point_count[0], m_point_count[1], 
            m_point_stride0*3, 3, 
            Point(0,0), 
            i, j );

}


ON_BOOL32 ON_PointGrid::Write(
       ON_BinaryArchive&  // open binary file
     ) const
{
  // TODO
  return false;
}

ON_BOOL32 ON_PointGrid::Read(
       ON_BinaryArchive&  // open binary file
     )
{
  // TODO
  return false;
}

ON::object_type ON_PointGrid::ObjectType() const
{
  return ON::pointset_object;
}

ON_BOOL32 
ON_PointGrid::IsClosed( int dir ) const
{
  return ON_IsPointGridClosed( 3, 0, 
                    m_point_count[0], m_point_count[1], 
                    m_point_stride0*3, 3, 
                    &m_point[0].x, dir );
}

ON_BOOL32
ON_PointGrid::Reverse(int dir)
{
  return ON_ReversePointGrid( 3, false, m_point_count[0], m_point_count[1], m_point_stride0*3, 3, Point(0,0), dir );
}

ON_BOOL32
ON_PointGrid::Transpose()
{
  int i, j;
  ON_BOOL32 rc = false;
  if ( IsValid() ) {
    // slow stupid way - can be imporved if necessary
    ON_PointGrid t(m_point_count[1],m_point_count[0]);
    for ( i = 0; i < m_point_count[0]; i++ ) for ( j = 0; j < m_point_count[1]; j++ ) {
      t[j][i] = Point(i,j);
    }
    *this = t;
    rc = true;
  }
  return rc;
}



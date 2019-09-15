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


ON_OBJECT_IMPLEMENT(ON_NurbsCage,ON_Geometry,"06936AFB-3D3C-41ac-BF70-C9319FA480A1");

ON_OBJECT_IMPLEMENT(ON_MorphControl,ON_Geometry,"D379E6D8-7C31-4407-A913-E3B7040D034A");


ON_BOOL32 ON_NurbsCage::Read(ON_BinaryArchive& archive)
{
  Destroy();

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);
  if ( rc )
  {
    while(rc)
    {
      if ( major_version != 1 )
      {
        ON_ERROR("ON_NurbsCage::Read - old code unable to read new version of chunk");
        rc = false;
        break;
      }

      int dim=0;
      int order0=0,order1=0,order2=0;
      int cv_count0=0,cv_count1=0,cv_count2=0;
      int is_rat=0;

      rc = archive.ReadInt(&dim);
      if (!rc)
        break;
      if (dim < 1 || dim > 10000)
      {
        ON_ERROR("ON_NurbsCage::Read - invalid dim");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&is_rat);
      if (!rc)
        break;
      if (is_rat != 0 && is_rat != 1)
      {
        ON_ERROR("ON_NurbsCage::Read - invalid is_rat");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&order0);
      if (!rc)
        break;
      if ( order0 < 2 || order0 > 10000 )
      {
        ON_ERROR("ON_NurbsCage::Read - invalid order0");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&order1);
      if (!rc)
        break;
      if ( order1 < 2 || order1 > 10000 )
      {
        ON_ERROR("ON_NurbsCage::Read - invalid order1");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&order2);
      if (!rc)
        break;
      if ( order2 < 2 || order2 > 10000 )
      {
        ON_ERROR("ON_NurbsCage::Read - invalid order2");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&cv_count0);
      if (!rc)
        break;
      if ( cv_count0 < order0 || cv_count0 > 100000 )
      {
        ON_ERROR("ON_NurbsCage::Read - invalid cv_count0");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&cv_count1);
      if (!rc)
        break;
      if ( cv_count1 < order1 || cv_count1 > 100000 )
      {
        ON_ERROR("ON_NurbsCage::Read - invalid cv_count1");
        rc=false;
        break;
      }

      rc = archive.ReadInt(&cv_count2);
      if (!rc)
        break;
      if ( cv_count2 < order2 || cv_count2 > 100000 )
      {
        ON_ERROR("ON_NurbsCage::Read - invalid cv_count2");
        rc=false;
        break;
      }

      rc = Create(dim,is_rat==1,order0,order1,order2,cv_count0,cv_count1,cv_count2);
      if (!rc)
        break;

      if (rc)
        rc = archive.ReadDouble(KnotCount(0),m_knot[0]);
      if (rc)
        rc = archive.ReadDouble(KnotCount(1),m_knot[1]);
      if (rc)
        rc = archive.ReadDouble(KnotCount(2),m_knot[2]);

      int i,j,k;
      const int cv_dim = m_is_rat?(m_dim+1):m_dim;
      for(i = 0; i < cv_count0 && rc; i++)
      {
        for(j = 0; j < cv_count1 && rc; j++)
        {
          for ( k = 0; k < cv_count2 && rc; k++)
          {
            rc = archive.ReadDouble(cv_dim,CV(i,j,k));
          }
        }
      }

      

      break;
    }

    if ( !archive.EndRead3dmChunk() )
    {
      rc = false;
    }
  }
  return rc;
}

ON_BOOL32 ON_NurbsCage::Write(ON_BinaryArchive& archive) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);

  if (rc)
  {
    rc = archive.WriteInt(m_dim);
    if(rc)
      rc = archive.WriteInt(m_is_rat);

    if (rc)
      rc = archive.WriteInt(m_order[0]);
    if (rc)
      rc = archive.WriteInt(m_order[1]);
    if (rc)
      rc = archive.WriteInt(m_order[2]);

    if (rc)
      rc = archive.WriteInt(m_cv_count[0]);
    if (rc)
      rc = archive.WriteInt(m_cv_count[1]);
    if (rc)
      rc = archive.WriteInt(m_cv_count[2]);

    if (rc)
      rc = archive.WriteDouble(KnotCount(0),m_knot[0]);
    if (rc)
      rc = archive.WriteDouble(KnotCount(1),m_knot[1]);
    if (rc)
      rc = archive.WriteDouble(KnotCount(2),m_knot[2]);

    int i,j,k;
    const int cv_dim = m_is_rat?(m_dim+1):m_dim;
    double* bogus_cv = (double*)alloca(cv_dim*sizeof(*bogus_cv));
    for ( i = 0; i < cv_dim; i++ )
      bogus_cv[i] = ON_UNSET_VALUE;
    for(i = 0; i < m_cv_count[0] && rc; i++)
    {
      for(j = 0; j < m_cv_count[1] && rc; j++)
      {
        for ( k = 0; k < m_cv_count[2] && rc; k++)
        {
          const double* cv = CV(i,j,k);
          if ( !cv )
            cv = bogus_cv;
          rc = archive.WriteDouble(cv_dim,cv);
        }
      }
    }

    if ( !archive.EndWrite3dmChunk() )
    {
      rc = false;
    }
  }

  return rc;
}



ON_NurbsCage::ON_NurbsCage()
: m_dim(0),m_is_rat(0),m_cv_capacity(0),m_cv(0)
{
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_cv_count[2] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;
}

ON_NurbsCage::ON_NurbsCage( int dim, bool is_rat, 
                                int order0,
                                int order1,
                                int order2,
                                int cv_count0,
                                int cv_count1,
                                int cv_count2
                                )
                 : m_dim(0),m_is_rat(0),m_cv_capacity(0),m_cv(0)
{
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_cv_count[2] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;
  Create( dim, is_rat, order0, order1, order2, cv_count0, cv_count1, cv_count2 );
}

ON_NurbsCage::ON_NurbsCage( const ON_BoundingBox& bbox, 
                               int order0, int order1, int order2,
                               int cv_count0, int cv_count1, int cv_count2
                               )
: m_dim(0),m_is_rat(0),m_cv_capacity(0),m_cv(0)
{
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_cv_count[2] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;
  Create(bbox,order0,order1,order2, cv_count0, cv_count1, cv_count2);
}

bool ON_NurbsCage::IsParallelogram(double tolerance) const
{
  int i,j,k;
  double r,s,t;
  double x,y,z,dist;
  ON_Interval d[3];
  ON_3dPoint P, X, Y, Z, Q, B;

  bool rc = IsValid()?true:false;

  for ( i = 0; i < 3 && rc; i++ )
  {
    d[i] = Domain(i);
    rc = (    d[i][0] == m_knot[i][0]
           && d[i][1] == m_knot[i][m_cv_count[i]+m_order[i]-3]
           );
  }

  if (rc)
  {
    GetCV(0,0,0,P);
    GetCV(m_cv_count[0]-1,0,0,X);
    GetCV(0,m_cv_count[1]-1,0,Y);
    GetCV(0,0,m_cv_count[2]-1,Z);

    if ( tolerance < ON_ZERO_TOLERANCE )
      tolerance = ON_ZERO_TOLERANCE;

    for ( i = 0; i < m_cv_count[0]; i++ )
    {
      r = ON_GrevilleAbcissa(m_order[0],m_knot[0]+i);
      x = d[0].NormalizedParameterAt(r);
      for ( j = 0; j < m_cv_count[1]; j++ )
      {
        s = ON_GrevilleAbcissa(m_order[1],m_knot[1]+j);
        y = d[1].NormalizedParameterAt(s);
        for ( k = 0; k < m_cv_count[2]; k++ )
        {
          t = ON_GrevilleAbcissa(m_order[2],m_knot[2]+k);
          z = d[2].NormalizedParameterAt(t);
          Evaluate(r,s,t,0,3,&Q.x);
          B = (1.0-x-y-z)*P + x*X + y*Y + z*Z;
          dist = B.DistanceTo(Q);
          if ( dist > tolerance )
            return false;          
        }
      }
    }
  }

  return rc;
}


ON_NurbsCage::ON_NurbsCage( const ON_3dPoint* box_corners, 
                               int order0, int order1, int order2,
                               int cv_count0, int cv_count1, int cv_count2
                               )
: m_dim(0),m_is_rat(0),m_cv_capacity(0),m_cv(0)
{
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_cv_count[2] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;
  Create(box_corners,order0,order1,order2, cv_count0, cv_count1, cv_count2);
}

ON_NurbsCage::ON_NurbsCage( const ON_BezierCage& src )
: m_dim(0),m_is_rat(0),m_cv_capacity(0),m_cv(0)
{
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_cv_count[2] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;
  *this = src;
}

ON_NurbsCage::~ON_NurbsCage()
{
  Destroy();
}

int ON_NurbsCage::KnotCount(int dir) const
{
  return (dir>=0 && dir<=2) ? ON_KnotCount(m_order[dir],m_cv_count[dir]) : 0;
}

double ON_NurbsCage::GrevilleAbcissa(
         int dir,    // dir
         int gindex  // index (0 <= index < CVCount(dir)
         ) const
{
  return (dir >= 0 && dir <= 2)
         ? ON_GrevilleAbcissa( m_order[dir], m_knot[dir] + gindex )
         : ON_UNSET_VALUE;
}

bool ON_NurbsCage::MakeDeformable()
{
  return true;
}

bool ON_NurbsCage::IsDeformable() const
{
  return true;
}

bool ON_NurbsCage::GetTightBoundingBox( ON_BoundingBox& tight_bbox,int bGrowBox,const ON_Xform* xform) const
{
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }

  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }

  if( xform && !xform->IsIdentity() )
  {
    ON_3dPoint P;
    int i,j,k;
    for ( i = 0; i < m_cv_count[0]; i++ )
    {
      for ( j = 0; j < m_cv_count[1]; j++ )
      {
        for ( k = 0; k < m_cv_count[2]; k++ )
        {
          GetCV(i,j,k,P);
          P = (*xform)*P;
          if ( tight_bbox.Set(P,bGrowBox) )
          {
            bGrowBox = true;
          }
        }           
      }
    }
  }
  else
  {
    if ( GetBoundingBox(tight_bbox,bGrowBox) )
      bGrowBox = true;
  }

  return bGrowBox?true:false;
}

int ON_NurbsCage::CVCount() const
{
  return CVCount(0)*CVCount(1)*CVCount(2);
}

int ON_NurbsCage::CVCount(int dir) const
{
  return (dir>=0&&dir<=2) ? m_cv_count[dir] : 0;
}

void ON_NurbsCage::DestroyRuntimeCache(bool bDelete)
{
  ON_Geometry::DestroyRuntimeCache(bDelete);
}

ON::object_type ON_NurbsCage::ObjectType() const
{
  return ON::cage_object;
}

ON_NurbsCage& ON_NurbsCage::operator=( const ON_BezierCage& src )
{
  if ( Create(src.m_dim,src.m_is_rat,
              src.m_order[0],src.m_order[1],src.m_order[2],
              src.m_order[0],src.m_order[1],src.m_order[2]) )
  {
    int i,j,k;
    for ( i = 0; i < m_cv_count[0]; i++ )
    {
      for ( j = 0; j < m_cv_count[1]; j++ )
      {
        for ( k = 0; k < m_cv_count[2]; k++ )
        {
          SetCV(i,j,k,ON::intrinsic_point_style,src.CV(i,j,k));
        }           
      }
    }
  }
  return *this;
}

unsigned int ON_NurbsCage::SizeOf() const
{
  unsigned int sz = ON_Geometry::SizeOf();
  sz += (sizeof(*this) - sizeof(ON_Geometry));
  sz += (KnotCount(0) + KnotCount(1) + KnotCount(2) + CVSize()*CVCount())*sizeof(double);
  return sz;
}

ON__UINT32 ON_NurbsCage::DataCRC(ON__UINT32 current_remainder) const
{
  current_remainder = ON_CRC32(current_remainder,sizeof(m_dim),&m_dim);
  current_remainder = ON_CRC32(current_remainder,sizeof(m_is_rat),&m_is_rat);
  current_remainder = ON_CRC32(current_remainder,3*sizeof(m_order[0]),&m_order[0]);
  current_remainder = ON_CRC32(current_remainder,3*sizeof(m_cv_count[0]),&m_cv_count[0]);
  if (   m_cv_count[0] > 0 && m_cv_count[1] > 0 && m_cv_count[2] > 0
      && m_cv_stride[0] > 0 && m_cv_stride[1] > 0 && m_cv_stride[2] > 0
      && m_cv )
  {
    std::size_t sizeof_cv = CVSize()*sizeof(m_cv[0]);
    const double* cv = m_cv;
    int i, j, k;
    for ( i = 0; i < m_cv_count[0]; i++ )
    {
      for ( j = 0; j < m_cv_count[1]; j++ )
      {
        cv = CV(i,j,0);
        for (k = 0; i < m_cv_count[2]; k++ )
        {
          current_remainder = ON_CRC32(current_remainder,sizeof_cv,cv);
          cv += m_cv_stride[2];
        }
      }
    }
  }
  current_remainder = ON_CRC32(current_remainder,KnotCount(0)*sizeof(m_knot[0][0]),m_knot[0]);
  current_remainder = ON_CRC32(current_remainder,KnotCount(1)*sizeof(m_knot[1][0]),m_knot[1]);
  current_remainder = ON_CRC32(current_remainder,KnotCount(2)*sizeof(m_knot[2][0]),m_knot[2]);

  return current_remainder;
}

ON_NurbsCage::ON_NurbsCage(const ON_NurbsCage& src)
                 : ON_Geometry(src),
                   m_dim(0),m_is_rat(0),m_cv_capacity(0),m_cv(0)
{
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
  m_cv_count[0] = 0;
  m_cv_count[1] = 0;
  m_cv_count[2] = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;

  *this = src;
}


ON_NurbsCage& ON_NurbsCage::operator=(const ON_NurbsCage& src)
{
  if ( this != &src ) 
  {
    ON_Geometry::operator=(src);
    if ( Create( src.m_dim, src.m_is_rat, 
         src.m_order[0], src.m_order[1], src.m_order[2],
         src.m_cv_count[0], src.m_cv_count[1], src.m_cv_count[2]
         ) )
    {
      if ( m_order[0] >= 2 && m_cv_count[0] >= m_order[0] && m_knot[0] && src.m_knot[0] )
        memcpy( m_knot[0], src.m_knot[0], KnotCount(0)*sizeof(m_knot[0][0]));
      if ( m_order[1] >= 2 && m_cv_count[1] >= m_order[1] && m_knot[1] && src.m_knot[1]  )
        memcpy( m_knot[1], src.m_knot[1], KnotCount(1)*sizeof(m_knot[1][0]));
      if ( m_order[2] >= 2 && m_cv_count[2] >= m_order[2] && m_knot[2] && src.m_knot[2]  )
        memcpy( m_knot[2], src.m_knot[2], KnotCount(2)*sizeof(m_knot[2][0]));

      if ( m_cv && src.m_cv && m_cv_stride[0] > 0 && m_cv_stride[1] > 0 && m_cv_stride[2] > 0 )
      {
        const int cv_dim = CVSize();
        const int sizeofcv = cv_dim*sizeof(m_cv[0]);
        if (    m_cv_stride[0] == src.m_cv_stride[0] 
            && m_cv_stride[1] == src.m_cv_stride[1]
            && m_cv_stride[2] == src.m_cv_stride[2] )
        {
          memcpy(m_cv,src.m_cv,m_cv_count[0]*m_cv_count[1]*m_cv_count[2]*sizeofcv);
        }
        else
        {
          int i, j, k;
          double* cv = m_cv;
          for ( i = 0; i < m_cv_count[0]; i++ ) 
          for ( j = 0; j < m_cv_count[1]; j++ ) 
          for ( k = 0; k < m_cv_count[2]; k++ ) 
          {
            memcpy( cv, src.CV(i,j,k), sizeofcv );
            cv += cv_dim;
          }
        }
      }
    }
    else 
    {
      Destroy();
    }
  }
  return *this;
}

ON_BOOL32 ON_NurbsCage::IsValid( 
          ON_TextLog* //text_log 
          ) const
{
  if ( 0 == m_cv )
    return false;

  if ( 0 == m_knot[0] )
    return false;

  if ( 0 == m_knot[1] )
    return false;

  if ( 0 == m_knot[2] )
    return false;

  if ( m_order[0] < 2 )
    return false;
  if ( m_order[1] < 2 )
    return false;
  if ( m_order[2] < 2 )
    return false;

  if ( m_cv_count[0] < m_order[0] )
    return false;
  if ( m_cv_count[1] < m_order[1] )
    return false;
  if ( m_cv_count[2] < m_order[2] )
    return false;

  if ( m_dim <= 0 )
    return false;
  if ( m_is_rat != 0 && m_is_rat != 1 )
    return false;

  const int cvdim = m_is_rat ? (m_dim+1) : m_dim;

  if ( m_cv_capacity > 0 && m_cv_capacity < cvdim*m_cv_count[0]*m_cv_count[1]*m_cv_count[2] )
    return false;

  int i[3];
  i[0] = (m_cv_stride[0] <= m_cv_stride[1]) ? 0 : 1;
  i[1] = 1-i[0];
  if ( m_cv_stride[2] < m_cv_stride[i[0]] )
  {
    i[2] = i[1];
    i[1] = i[0];
    i[0] = 2;
  }
  else if ( m_cv_stride[2] < m_cv_stride[i[1]] )
  {
    i[2] = i[1];
    i[1] = 2;
  }
  else
  {
    i[2] = 2;
  }

  if ( m_cv_stride[i[0]] < cvdim )
    return false;
  if ( m_cv_stride[i[1]] < m_cv_stride[i[0]]*m_cv_count[i[0]] )
    return false;
  if ( m_cv_stride[i[2]] < m_cv_stride[i[1]]*m_cv_count[i[1]] )
    return false;

  return true;
}

void ON_NurbsCage::Dump( ON_TextLog& dump ) const
{
  dump.Print( "ON_NurbsCage dim = %d is_rat = %d\n"
               "        order = (%d, %d, %d) \n",
               "        cv_count = (%d, %d, %d) \n",
               m_dim, m_is_rat, 
               m_order[0], m_order[1], m_order[2], 
               m_cv_count[0], m_cv_count[1], m_cv_count[2] );

  int dir;
  for ( dir = 0; dir < 3; dir++ ) 
  {
    dump.Print( "Knot Vector %d ( %d knots )\n", dir, KnotCount(dir) );
    dump.PrintKnotVector( m_order[dir], m_cv_count[dir], m_knot[dir] );
  }

  dump.Print( "Control Points  %d %s points\n"
               "  index               value\n",
               m_cv_count[0]*m_cv_count[1]*m_cv_count[2], 
               (m_is_rat) ? "rational" : "non-rational" );
  if ( !m_cv ) 
  {
    dump.Print("  NULL cv array\n");
  }
  else 
  {
    int i,j;
    char sPreamble[128]; 
    memset(sPreamble,0,sizeof(sPreamble));
    for ( i = 0; i < m_order[0]; i++ )
    {
      for ( j = 0; j < m_order[1]; j++ )
      {
        if ( i > 0 || j > 0)
          dump.Print("\n");
        sPreamble[0] = 0;
        sprintf(sPreamble,"  CV[%2d][%2d]",i,j);
        dump.PrintPointList( m_dim, m_is_rat, 
                          m_cv_count[2], m_cv_stride[2],
                          CV(i,j,0), 
                          sPreamble );
      }
      if ( i < m_order[0]-1)
        dump.Print("\n");
    }
  }
}

int ON_NurbsCage::Dimension() const
{
  return m_dim;
}

bool ON_NurbsCage::Create( const ON_BoundingBox& bbox, 
                            int order0, int order1, int order2,
                            int cv_count0, int cv_count1, int cv_count2 
                            )
{
  /*
            7______________6
            |\             |\
            | \            | \
            |  \ _____________\
            |   4          |   5
            |   |          |   |
            |   |          |   |
            3---|----------2   |
            \   |          \   |
             \  |z          \  |
            y \ |            \ |
               \0_____________\1
                       x
  */
  ON_3dPoint box_corners[8];
  box_corners[0] = bbox.Corner(0,0,0);
  box_corners[1] = bbox.Corner(1,0,0);
  box_corners[2] = bbox.Corner(1,1,0);
  box_corners[3] = bbox.Corner(0,1,0);
  box_corners[4] = bbox.Corner(0,0,1);
  box_corners[5] = bbox.Corner(1,0,1);
  box_corners[6] = bbox.Corner(1,1,1);
  box_corners[7] = bbox.Corner(0,1,1);
  return Create(box_corners,order0,order1,order2,cv_count0,cv_count1,cv_count2);
}


bool ON_NurbsCage::Create( int dim, bool is_rat, 
                            int order0, int order1, int order2,
                            int cv_count0, int cv_count1, int cv_count2 
                            )
{
  Destroy();
  if ( order0 < 2 || order1 < 2 || order2 < 2 )
  {
    if (   0 == dim && 0 == is_rat 
         && 0 == order0 && 0 == order1 && 0 == order2 
         && 0 == cv_count0 && 0 == cv_count1 && 0 == cv_count2 )
    {
      return true;
    }
    ON_ERROR("ON_NurbsCage::Create - invalid orders");
    return false;
  }

  if ( cv_count0 < order0 || cv_count1 < order1 || cv_count2 < order2 )
  {
    ON_ERROR("ON_NurbsCage::Create - invalid cv counts");
    return false;
  }

  if ( dim < 1 )
  {
    ON_ERROR("ON_NurbsCage::Create - invalid dim");
    return false;
  }

  if ( is_rat != true && is_rat != false )
  {
    ON_ERROR("ON_NurbsCage::Create - invalid is_rat");
    return false;
  }

  m_dim = dim;
  m_is_rat = is_rat ? 1 : 0;
  
  m_order[0] = order0;
  m_order[1] = order1;
  m_order[2] = order2;
  
  m_cv_count[0] = cv_count0;
  m_cv_count[1] = cv_count1;
  m_cv_count[2] = cv_count2;
  
  // Other ON_NurbsCage member functions, like operator=,
  // depend on the strides being set this way.  If you anything
  // in the next three lines, then you need to read all the
  // code in the ON_NurbsCage member functions and adjust
  // it accordingly.
  m_cv_stride[2] = m_dim+m_is_rat;
  m_cv_stride[1] = m_cv_stride[2]*m_cv_count[2];
  m_cv_stride[0] = m_cv_stride[1]*m_cv_count[1];
  
  ReserveCVCapacity(m_cv_stride[0]*m_cv_count[0]);
  
  ReserveKnotCapacity(0,ON_KnotCount(m_order[0],m_cv_count[0]));
  ReserveKnotCapacity(1,ON_KnotCount(m_order[1],m_cv_count[1]));
  ReserveKnotCapacity(2,ON_KnotCount(m_order[2],m_cv_count[2]));
  
  ON_MakeClampedUniformKnotVector(m_order[0],m_cv_count[0],m_knot[0],1.0);
  ON_MakeClampedUniformKnotVector(m_order[1],m_cv_count[1],m_knot[1],1.0);
  ON_MakeClampedUniformKnotVector(m_order[2],m_cv_count[2],m_knot[2],1.0);

  ON_SetKnotVectorDomain( m_order[0], m_cv_count[0], m_knot[0], 0.0, 1.0);
  ON_SetKnotVectorDomain( m_order[1], m_cv_count[1], m_knot[1], 0.0, 1.0);
  ON_SetKnotVectorDomain( m_order[2], m_cv_count[2], m_knot[2], 0.0, 1.0);
  
  return IsValid() ? true : false;
}

bool ON_NurbsCage::Create(
  const ON_3dPoint* box_corners,
  int order0, int order1, int order2,
  int cv_count0, int cv_count1, int cv_count2 
  )
{
  int i, j, k;
  double r,s,t;
  //bool rc = false;
  if ( 0 == box_corners )
    return false;
  for( i = 0; i < 8; i++ )
  {
    if ( !box_corners[i].IsValid() )
      return false;
  }

  // create trilinear "cube" to make it easy
  // to calculate CV locations.
  ON_BezierCage cube(3,0,2,2,2);
  cube.SetCV(0,0,0,box_corners[0]);
  cube.SetCV(1,0,0,box_corners[1]);
  cube.SetCV(1,1,0,box_corners[2]);
  cube.SetCV(0,1,0,box_corners[3]);
  cube.SetCV(0,0,1,box_corners[4]);
  cube.SetCV(1,0,1,box_corners[5]);
  cube.SetCV(1,1,1,box_corners[6]);
  cube.SetCV(0,1,1,box_corners[7]);

  if ( 2 == cv_count0 && 2 == cv_count1 && 2 == cv_count2 )
  {
    operator=(cube);
  }
  else
  {
    if (!Create(3,0,order0,order1,order2,cv_count0,cv_count1,cv_count2))
      return false;

    double* g0 = (double*)onmalloc(m_cv_count[0]*m_cv_count[1]*m_cv_count[2]*sizeof(*g0));
    double* g1 = g0 + m_cv_count[0];
    double* g2 = g1 + m_cv_count[1];

    ON_GetGrevilleAbcissae(m_order[0],m_cv_count[0],m_knot[0],false,g0);
    ON_GetGrevilleAbcissae(m_order[1],m_cv_count[1],m_knot[1],false,g1);
    ON_GetGrevilleAbcissae(m_order[2],m_cv_count[2],m_knot[2],false,g2);

    for (i = 0; i < m_cv_count[0]; i++)
    {
      r = g0[i];
      for (j = 0; j < m_cv_count[1]; j++)
      {
        s = g1[j];
        for (k = 0; k < m_cv_count[2]; k++)
        {
          t = g2[k];
          SetCV(i,j,k,cube.PointAt(r,s,t));
        }
      }
    }

    onfree(g0);
  }
  return IsValid()?true:false;
}


void ON_NurbsCage::Destroy()
{
  DestroyRuntimeCache();

  if ( m_cv && m_cv_capacity > 0 )
  {
    onfree(m_cv);
    m_cv = 0;
  }

  if ( m_knot[0] && m_knot_capacity[0] > 0 )
  {
    onfree(m_knot[0]);
    m_knot[0] = 0;
  }

  if ( m_knot[1] && m_knot_capacity[1] > 0 )
  {
    onfree(m_knot[1]);
    m_knot[1] = 0;
  }

  if ( m_knot[2] && m_knot_capacity[2] > 0 )
  {
    onfree(m_knot[2]);
    m_knot[2] = 0;
  }

  m_cv_capacity = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;

  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;

  m_dim = 0;
  m_is_rat = 0;

  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
}

void ON_NurbsCage::EmergencyDestroy()
{
  DestroyRuntimeCache(false);
  m_cv = 0;
  m_knot[0] = 0;
  m_knot[1] = 0;
  m_knot[2] = 0;
  m_cv_capacity = 0;
  m_knot_capacity[0] = 0;
  m_knot_capacity[1] = 0;
  m_knot_capacity[2] = 0;
  m_cv_stride[0] = 0;
  m_cv_stride[1] = 0;
  m_cv_stride[2] = 0;
  m_dim = 0;
  m_is_rat = 0;
  m_order[0] = 0;
  m_order[1] = 0;
  m_order[2] = 0;
}


ON_BOOL32 ON_NurbsCage::GetBBox( // returns true if successful
       double* boxmin,    // minimum
       double* boxmax,    // maximum
       ON_BOOL32 bGrowBox  // true means grow box
       ) const
{
  int i, j;
  bool rc = ( 0 != m_cv 
              && m_cv_count[0] >= 2 && m_cv_count[1] >= 2 && m_cv_count[2] >= 2 
              && m_cv_stride[0] > 0 && m_cv_stride[1] > 0 && m_cv_stride[2] > 0 ) ? true : false;
  if ( !rc )
  {
    ON_ERROR("ON_NurbsCage::GetBBox - invalid input");
  }
  else
  {
    for ( i = 0; rc && i < m_cv_count[0]; i++ ) 
    for ( j = 0; rc && j < m_cv_count[1]; j++ ) 
    {
      rc = ON_GetPointListBoundingBox( m_dim, m_is_rat, m_cv_count[2], m_cv_stride[2],
                                      CV(i,j,0), 
                                      boxmin, boxmax, bGrowBox?true:false );
      bGrowBox = true;
    }
  }
  return rc;
}

ON_BOOL32 ON_NurbsCage::Transform( const ON_Xform& xform )
{
  int i,j;
  bool rc = (m_cv_count[0] > 0 && m_cv_count[1] > 0 && m_cv_count[2]) ? true : false;
  if ( rc || !xform.IsIdentity() )
  {  
    if ( 0 == m_is_rat )
    {
      if ( xform.m_xform[3][0] != 0.0 || xform.m_xform[3][1] != 0.0 || xform.m_xform[3][2] != 0.0 )
      {
        MakeRational();
      }
    }
  
    for ( i = 0; rc && i < m_cv_count[0]; i++ ) 
    {
      for ( j = 0; rc && j < m_cv_count[1]; j++ ) 
      {
        rc = ON_TransformPointList( m_dim, m_is_rat, 
                                    m_cv_count[2], m_cv_stride[2], 
                                    CV(i,j,0), xform );
      }
    }
  }
  return rc;
}

ON_Interval ON_NurbsCage::Domain( 
      int dir
      ) const
{
  ON_Interval d;
  if ( dir < 0 || dir > 2 || !ON_GetKnotVectorDomain( m_order[dir], m_cv_count[dir], m_knot[dir], &d.m_t[0], &d.m_t[1] ) )
    d.Destroy();
  return d;
}





bool ON_EvaluateNurbsCageSpan(
        int dim,
        int is_rat,
        int order0, int order1, int order2,
        const double* knot0,
        const double* knot1,
        const double* knot2,
        int cv_stride0, int cv_stride1, int cv_stride2,
        const double* cv0,
        int der_count,
        double t0, double t1, double t2,
        int v_stride, 
        double* v
        )
{
  double c;
  double* N_0, *N_1, *N_2, *P, *P0, *P00;
	const double *cv;
  int j0, j1, j2, d0, d1, d2, d;
  const int cvdim = is_rat ? (dim+1) : dim;
  const int dcv2 = cv_stride2 - cvdim;
	const int der_count0 = (der_count >= order0) ? order0-1 : der_count;
	const int der_count1 = (der_count >= order1) ? order1-1 : der_count;
	const int der_count2 = (der_count >= order1) ? order2-2 : der_count;
  int Pcount = der_count*(der_count*(der_count*2 + 9)+13)/6 + 1;
  int Psize = cvdim<<3;
  int i = order0*order0;
  int j = order1*order1;
  int k = order2*order2;

  // don't declare any variable below here to avoid problems caused
  // by compiler/optimizer/alloca() bugs that can't keep the SP 
  // properly set.

  N_0 = (double*)alloca( ((i+j+k)*sizeof(*N_0)) + Pcount*Psize);
  N_1 = N_0 + i;
  N_2 = N_1 + j;
  P0  = N_2 + k;
  memset( P0, 0, Pcount*Psize );

  ON_EvaluateNurbsBasis( order0, knot0, t0, N_0 );
  ON_EvaluateNurbsBasis( order1, knot1, t1, N_1 );
  ON_EvaluateNurbsBasis( order2, knot2, t2, N_2 );
  if ( der_count0 > 0 )
  {
		ON_EvaluateNurbsBasisDerivatives( order0, knot0, der_count0, N_0 );
		ON_EvaluateNurbsBasisDerivatives( order1, knot1, der_count1, N_1 );
		ON_EvaluateNurbsBasisDerivatives( order2, knot2, der_count2, N_2 );
  }

  // compute point
	P = P0;
	for ( j0 = 0; j0 < order0; j0++) 
  {
		for ( j1 = 0; j1 < order1; j1++ ) 
    {
      cv = cv0 + j0*cv_stride0 + j1*cv_stride1;
      for ( j2 = 0; j2 < order2; j2++ )
      {
			  c = N_0[j0]*N_1[j1]*N_2[j2];
			  j = cvdim;
			  while (j--) 
        {
				  *P++ += c* *cv++;
        }
			  P -= cvdim;
        cv += dcv2;
		  }
	  }
  }

  if ( der_count > 0 ) 
  {
    // quickly compute first derivatives
  	P += cvdim; // step over point
		for ( j0 = 0; j0 < order0; j0++) 
    {
			for ( j1 = 0; j1 < order1; j1++ ) 
      {
        cv = cv0 + j0*cv_stride0 + j1*cv_stride1;
        for ( j2 = 0; j2 < order2; j2++ )
        {
          // "Dr"
				  c = N_0[j0+order0]*N_1[j1]*N_2[j2];
				  j = cvdim;
				  while (j--) 
					  *P++ += c* *cv++;
          cv -= cvdim;

          // "Ds"
				  c = N_0[j0]*N_1[j1+order1]*N_2[j2];
				  j = cvdim;
				  while (j--) 
					  *P++ += c* *cv++;
          cv -= cvdim;

          // "Dt"
				  c = N_0[j0]*N_1[j1]*N_2[j2+order2];
				  j = cvdim;
				  while (j--) 
					  *P++ += c* *cv++;

				  P -= 3*cvdim;
          cv += dcv2;
			  }
		  }
    }

    // compute higher order derivatives in generic loop
    for ( d = 2; d <= der_count; d++ )
    {
      // compute second derivatives
      P += (cvdim*d*(d+1))>>1; // step over (d-1) derivatives
      // P points to first coordinate of Dr^d
      if ( der_count0+der_count1+der_count2 > 1 ) 
      {
		    for ( j0 = 0; j0 < order0; j0++) 
        {
		      for ( j1 = 0; j1 < order1; j1++) 
          {
            cv = cv0 + j0*cv_stride0 + j1*cv_stride1;
			      for ( j2 = 0; j2 < order2; j2++ ) 
            {
              P00 = P;
              for ( d0 = d; d0 >= 0; d0-- )
              {
                for ( d1 = d-d0; d1 >= 0; d1-- )
                {
                  d2 = d-d0-d1;
                  if ( d0 > der_count0 || d1 > der_count1 || d2 > der_count2 )
                  {
                    // this partial is zero
                    P += cvdim;
                  }
                  else
                  {
                    c = N_0[j0 + d0*order0]*N_1[j1 + d1*order1]*N_2[j2 + d2*order2];
                    j = cvdim;
                    while(j--)
                      *P++ += c* *cv++;
                    cv -= cvdim;
                  }
                }
              }
              P = P00;
              cv += cv_stride2;
			      }
		      }
        }
      }
    }

  }

	if ( is_rat ) 
  {
		ON_EvaluateQuotientRule3( dim, der_count, cvdim, P0 );
		Psize -= 8;
	}

  Pcount = (der_count+1)*(der_count+2)*(der_count+3)/6;
	for ( i = 0; i < Pcount; i++) 
  {
		memcpy( v, P0, Psize );
    v += v_stride;
		P0 += cvdim;
	}

  return true;
}

bool ON_NurbsCage::Evaluate( // returns false if unable to evaluate
       double r, double s, double t,       // evaluation parameter
       int der_count,            // number of derivatives (>=0)
       int v_stride,             // array stride (>=Dimension())
       double* v,                // array of length stride*(ndir+1)*(ndir+2)/2
       int side,       // optional - determines which side to evaluate from
                       //         0 = default
                       //         1 = from upper NE quadrant
                       //         2 = from upper NW quadrant
                       //         3 = from upper SW quadrant
                       //         4 = from upper SE quadrant
                       //         5 = from lower NE quadrant
                       //         6 = from lower NW quadrant
                       //         7 = from lower SW quadrant
                       //         8 = from lower SE quadrant
       int* hint
       ) const
{  
  int side0 = (side&&(side==2||side==3||side==6||side==7))?-1:1;
  int side1 = (side&&(side==3||side==4||side==7||side==8))?-1:1;
  int side2 = (side>=5&&side<=8)?-1:1;

  int hint0 = (hint) ? hint[0] : 0;
  int hint1 = (hint) ? hint[1] : 0;
  int hint2 = (hint) ? hint[2] : 0;
  const int span_index0 = ON_NurbsSpanIndex(m_order[0],m_cv_count[0],m_knot[0],r,side0,hint0);
  const int span_index1 = ON_NurbsSpanIndex(m_order[1],m_cv_count[1],m_knot[1],s,side1,hint1);
  const int span_index2 = ON_NurbsSpanIndex(m_order[2],m_cv_count[2],m_knot[2],t,side2,hint2);

  bool rc = ON_EvaluateNurbsCageSpan(m_dim,m_is_rat,
    m_order[0],m_order[1],m_order[2],
    m_knot[0]+span_index0,
    m_knot[1]+span_index1,
    m_knot[2]+span_index2,
    m_cv_stride[0],m_cv_stride[1],m_cv_stride[2],
    m_cv + (m_cv_stride[0]*span_index0+m_cv_stride[1]*span_index1+m_cv_stride[2]*span_index2),
    der_count,
    r,s,t,
    v_stride,v);

  if( hint )
  {
    hint[0] = span_index0;
    hint[1] = span_index1;
    hint[2] = span_index2;
  }

  return rc;
}

ON_3dPoint ON_NurbsCage::PointAt(
        double r, 
        double s, 
        double t
        ) const
{
  ON_3dPoint pt;
  if ( m_dim <= 3 )
  {
    pt.x  = 0.0;
    pt.y  = 0.0;
    pt.z  = 0.0;
    Evaluate(r,s,t,0,3,&pt.x);
  }
  else
  {
    double* v = (double*)alloca(m_dim*sizeof(*v));
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
    Evaluate(r,s,t,0,m_dim,v);
    pt.x = v[0];
    pt.y = v[1];
    pt.z = v[2];
  }
  return pt;
}

ON_3dPoint ON_NurbsCage::PointAt( ON_3dPoint rst ) const
{
  ON_3dPoint pt;
  if ( m_dim <= 3 )
  {
    pt.x  = 0.0;
    pt.y  = 0.0;
    pt.z  = 0.0;
    Evaluate(rst.x,rst.y,rst.z,0,3,&pt.x);
  }
  else
  {
    double* v = (double*)alloca(m_dim*sizeof(*v));
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
    Evaluate(rst.x,rst.y,rst.z,0,m_dim,v);
    pt.x = v[0];
    pt.y = v[1];
    pt.z = v[2];
  }
  return pt;
}

ON_NurbsSurface* ON_NurbsCage::IsoSurface(
        int dir,
        double c,
        ON_NurbsSurface* srf
        ) const
{
  // c = 0; cage(c,s,t) = srf(s,t)
  // c = 1; cage(r,c,t) = srf(r,t)
  // c = 2; cage(r,s,c) = srf(r,s)
  if ( dir < 0 || dir > 2 )
  {
    ON_ERROR("ON_NurbsCage::IsoSurface - invalid dir parameter");
    return 0;
  }
  if ( m_order[dir] < 2 || m_cv_count[dir] < m_order[dir] || 0 == m_knot[dir] )
  {
    ON_ERROR("ON_NurbsCage::IsoSurface - invalid NURBS cage");
    return 0;
  }

  const int cage_cvdim = CVSize();

  int span_index = ON_NurbsSpanIndex(m_order[dir],m_cv_count[dir],m_knot[dir],c,0,0);
  ON_NurbsCurve nurbs_curve;
  nurbs_curve.m_dim = cage_cvdim*m_cv_count[0]*m_cv_count[1]*m_cv_count[2]/m_cv_count[dir];
  nurbs_curve.m_is_rat = 0;
  nurbs_curve.m_order = m_order[dir];
  nurbs_curve.m_cv_count = nurbs_curve.m_order;
  nurbs_curve.ReserveCVCapacity(nurbs_curve.m_dim*nurbs_curve.m_cv_count);
  nurbs_curve.m_cv_stride = nurbs_curve.m_dim;
  nurbs_curve.m_knot = m_knot[dir] + span_index;
  nurbs_curve.m_knot_capacity = 0;

  int ii,jj,kk;
  /*
  int i0 = 0;
  int i1 = m_cv_count[0];
  int j0 = 0;
  int j1 = m_cv_count[1];
  int k0 = 0;
  int k1 = m_cv_count[2];
  */

  //int i0 = span_index;
  //int i1 = span_index+m_order[dir];
  ii = dir;
  switch(dir)
  {
  case 0:
    jj = 1;
    kk = 2;
    break;
  case 1:
    jj = 0;
    kk = 2;
    break;
  case 2:
    jj = 0;
    kk = 1;
    break;
  default: // to keep lint happy
    ii = 0;
    jj = 1;
    kk = 2;
    break;
  };


  double* cv;
  const int cage_sizeofcv = cage_cvdim*sizeof(*cv);
  //int i0 = span_index;
  int i1 = span_index+m_order[ii];
  int j1 = m_cv_count[jj];
  int k1 = m_cv_count[kk];

  int i,j,k;
  int cage_ijk[3];
  for ( i = span_index; i < i1; i++) 
  {
    cv = nurbs_curve.CV(i-span_index);
    cage_ijk[ii] = i;
    for ( j = 0; j < j1; j++ ) 
    {
      cage_ijk[jj] = j;
      for ( k = 0; k < k1; k++ )
      {
        cage_ijk[kk] = k;
        memcpy(cv,CV(cage_ijk[0],cage_ijk[1],cage_ijk[2]),cage_sizeofcv);
        cv += cage_cvdim;
      }
    }
  }

  ON_NurbsSurface* iso_srf = srf ? srf : ON_NurbsSurface::New();
  iso_srf->Create(m_dim,m_is_rat,m_order[jj],m_order[kk],m_cv_count[jj],m_cv_count[kk]);
  nurbs_curve.Evaluate(c,0,nurbs_curve.m_dim,iso_srf->m_cv,0,0);
  nurbs_curve.m_knot = 0;
  memcpy(iso_srf->m_knot[0],m_knot[jj],iso_srf->KnotCount(0)*sizeof(*iso_srf->m_knot[0]));
  memcpy(iso_srf->m_knot[1],m_knot[kk],iso_srf->KnotCount(1)*sizeof(*iso_srf->m_knot[1]));

  return iso_srf;
}

bool ON_NurbsCage::IsRational() const
{
  return m_is_rat ? true : false;
}

int ON_NurbsCage::CVSize() const
{
  return ( m_is_rat && m_dim>0 ) ? m_dim+1 : m_dim;
}

int ON_NurbsCage::Order( int dir ) const
{
  return (dir>=0&&dir<=2) ? m_order[dir] : 0;
}

int ON_NurbsCage::Degree(int dir) const
{
  int order = Order(dir);
  return (order>=2) ? order-1 : 0;
}

bool ON_NurbsCage::IsClosed(int dir) const
{
  bool bIsClosed = false;
  if ( dir >= 0 && dir <= 2 && m_dim > 0)
  {
    if ( ON_IsKnotVectorClamped( m_order[dir], m_cv_count[dir], m_knot[dir] ) ) 
    {
      const double *cv0, *cv1;
      int i,j,k,d[3] = {0,0,0};
      d[dir] = m_cv_count[dir] - 1;
      for ( i = 0; i+d[0] < m_cv_count[0]; i++ )
      {
        for ( j = 0; j+d[1] < m_cv_count[1]; j++ )
        {
          for ( k = 0; k+d[2] < m_cv_count[2]; k++ )
          {
            cv0 = CV(i,j,k);
            cv1 = CV(i+d[0],j+d[1],k+d[2]);
            if ( false == ON_PointsAreCoincident( m_dim, m_is_rat, cv0, cv1 ) )
              return false;
          }
        }
      }
      bIsClosed = true;
    }
    else
    {
      bIsClosed =  IsPeriodic(dir);
    }
  }
  return bIsClosed;
}

bool ON_NurbsCage::IsPeriodic(int dir) const
{
  bool bIsPeriodic = false;
  if ( dir >= 0 && dir <= 2 && m_dim > 0 )
  {
    bIsPeriodic = ON_IsKnotVectorPeriodic( m_order[dir], m_cv_count[dir], m_knot[dir] );
    if ( bIsPeriodic ) 
    {
      const double *cv0, *cv1;
      int i,j,k,d[3] = {0,0,0};
      d[dir] = m_cv_count[dir] - (m_order[dir]-1);
      for ( i = 0; i+d[0] < m_cv_count[0]; i++ )
      {
        for ( j = 0; j+d[1] < m_cv_count[1]; j++ )
        {
          for ( k = 0; k+d[2] < m_cv_count[2]; k++ )
          {
            cv0 = CV(i,j,k);
            cv1 = CV(i+d[0],j+d[1],k+d[2]);
            if ( false == ON_PointsAreCoincident(m_dim, m_is_rat, cv0, cv1 ) )
              return false;
          }
        }
      }
    }
  }
  return bIsPeriodic;
}

double* ON_NurbsCage::CV( int i, int j, int k ) const
{

#if defined(ON_DEBUG)
  if ( 0 == m_cv )
  {
    ON_ERROR("ON_NurbsCage::CV - NULL m_cv");
    return 0;
  }
  if ( i < 0 || i >= m_cv_count[0] || j< 0 || j >= m_cv_count[1] || k < 0 || k >= m_cv_count[2])
  {
    ON_ERROR("ON_NurbsCage::CV - (i,j,k) out of range");
    return 0;
  }
#endif

  return (m_cv) ? (m_cv + i*m_cv_stride[0] + j*m_cv_stride[1] + k*m_cv_stride[2]) : 0;
}

ON::point_style ON_NurbsCage::CVStyle() const
{
  return m_is_rat ? ON::homogeneous_rational : ON::not_rational;
}

double ON_NurbsCage::Weight( int i, int j, int k ) const
{
  return (m_cv && m_is_rat) ? m_cv[i*m_cv_stride[0] + j*m_cv_stride[1] + k*m_cv_stride[2] + + m_dim] : 1.0;
}


bool ON_NurbsCage::SetWeight( int i, int j, int k, double w )
{
  bool rc = false;
  if ( m_is_rat ) 
  {
    double* cv = CV(i,j,k);
    if (cv) 
    {
      cv[m_dim] = w;
      rc = true;
    }
  }
  else if ( w == 1.0 ) 
  {
    rc = true;
  }
  return rc;
}

bool ON_NurbsCage::SetCV( int i, int j, int k, ON::point_style style, const double* Point )
{
  bool rc = true;
  int n;
  double w;

  double* cv = CV(i,j,k);
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
      for ( n = 0; n < m_dim; n++ ) {
        cv[n] = w*Point[n];
      }
    }
    break;

  case ON::euclidean_rational:  // input Point is euclidean rational
    if ( IsRational() ) {
      // NURBS surface is rational - convert euclean point to homogeneous form
      w = Point[m_dim];
      for ( n = 0; n < m_dim; n++ )
        cv[i] = w*Point[i];
      cv[m_dim] = w;
    }
    else {
      // NURBS surface is not rational
      memcpy( cv, Point, m_dim*sizeof(*cv) );
    }
    break;

  case ON::intrinsic_point_style:
    n = m_is_rat?m_dim+1:m_dim;
    memcpy(cv,Point,n*sizeof(*cv));
    break;
    
  default:
    rc = false;
    break;
  }
  return rc;
}

bool ON_NurbsCage::SetCV( int i, int j, int k, const ON_3dPoint& point )
{
  bool rc = false;
  double* cv = CV(i,j,k);
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

bool ON_NurbsCage::SetCV( int i, int j, int k, const ON_4dPoint& point )
{
  bool rc = false;
  double* cv = CV(i,j,k);
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

bool ON_NurbsCage::GetCV( int i, int j, int k, ON::point_style style, double* Point ) const
{
  const double* cv = CV(i,j,k);
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
  default:
    return false;
  }
  return true;
}

bool ON_NurbsCage::GetCV( int i, int j, int k, ON_3dPoint& point ) const
{
  bool rc = false;
  const double* cv = CV(i,j,k);
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

bool ON_NurbsCage::GetCV( int i, int j, int k, ON_4dPoint& point ) const
{
  bool rc = false;
  const double* cv = CV(i,j,k);
  if ( cv ) {
    point.x = cv[0];
    point.y = (m_dim>1)? cv[1] : 0.0;
    point.z = (m_dim>2)? cv[2] : 0.0;
    point.w = (m_is_rat) ? cv[m_dim] : 1.0;
    rc = true;
  }
  return rc;
}



bool ON_NurbsCage::SetKnot(
      int dir,
      int knot_index,
      double knot_value
      )
{
  bool rc;

  // Validate input so invalid input does not crash Rhino.
  // Expert programmers who want to write fast code can directly
  // access the m_knot[] arrays.
  if (    dir >= 0 && dir < 3 
       && 0 != m_knot[dir]
       && knot_index >= 0 
       && knot_index < m_order[dir]+m_cv_count[dir]-2
     )
  {
    m_knot[dir][knot_index] = knot_value;
    rc = true;
  }
  else
  {
    ON_ERROR("ON_NurbsCage::SetKnot - invalid input parameters");
    rc = false;
  }
  return rc;
}

double ON_NurbsCage::Knot(
      int dir,
      int knot_index
      ) const
{
  double knot_value;

  // Validate input so invalid input does not crash Rhino.
  // Expert programmers who want to write fast code can directly
  // access the m_knot[] arrays.
  if (    dir >= 0 && dir < 3 
       && 0 != m_knot[dir]
       && knot_index >= 0 
       && knot_index < m_order[dir]+m_cv_count[dir]-2
     )
  {
    knot_value = m_knot[dir][knot_index];
  }
  else
  {
    ON_ERROR("ON_NurbsCage::Knot - invalid input parameters");
    knot_value = ON_UNSET_VALUE;
  }
  return knot_value;
}

bool ON_NurbsCage::ZeroCVs()
{
  // zeros control vertices and, if rational, sets weights to 1
  bool rc = false;
  int i,j,k;
  if ( m_cv ) {
    if ( m_cv_capacity > 0 ) {
      memset( m_cv, 0, m_cv_capacity*sizeof(*m_cv) );
      if ( m_is_rat ) {
        for ( i = 0; i < m_order[0]; i++ ) {
          for ( j = 0; j < m_order[1]; j++ ) {
            for ( k = 0; k < m_order[2]; k++ ) {
              SetWeight( i,j,k, 1.0 );
            }
          }
        }
      }
      rc = true;
    }
    else {
      double* cv;
      int s = CVSize()*sizeof(*cv);
      for ( i = 0; i < m_order[0]; i++ ) {
        for ( j = 0; j < m_order[1]; j++ ) {
          for ( k = 0; k < m_order[2]; k++ ) {
            cv = CV(i,j,k);
            memset(cv,0,s);
            if ( m_is_rat )
              cv[m_dim] = 1.0;
          }
        }
      }
      rc = (i>0) ? true : false;
    }
  }
  return rc;
}

bool ON_NurbsCage::MakeRational()
{
  if ( !IsRational() ) 
  {
    const int dim = Dimension();
    if ( m_cv_count[0] > 0 && m_cv_count[1] > 0 && m_cv_count[2] > 0 && dim > 0 ) 
    {
      int i,j,k;
      if ( m_cv_stride[0] <= dim || m_cv_stride[1] <= dim || m_cv_stride[2] <= dim )
      {
        // there's not room for the weight in the existing m_cv array - adjust the strides
        double* new_cv = (double*)onmalloc(m_cv_count[0]*m_cv_count[1]*m_cv_count[2]*(dim+1)*sizeof(*new_cv));
        double* cv1 = new_cv;
        const int sizeofoldcv = dim*sizeof(*cv1);
        for (i = 0; i < m_cv_count[0]; i++) 
        {
          for(j = 0; j < m_cv_count[1]; j++) 
          {
            for(k = 0; k < m_cv_count[2]; k++)
            {
              memcpy(cv1,CV(i,j,k),sizeofoldcv);
              cv1 += dim;
              *cv1++ = 1.0;
            }
          }
        }
        m_is_rat = 1;
        ReserveCVCapacity(m_cv_count[0]*m_cv_count[1]*m_cv_count[2]*(dim+1));
        memcpy(m_cv,new_cv,m_cv_count[0]*m_cv_count[1]*m_cv_count[2]*(dim+1)*sizeof(*m_cv));
        onfree(new_cv);
        m_cv_stride[2] = dim+1;
        m_cv_stride[1] = m_cv_stride[2]*m_cv_count[2];
        m_cv_stride[0] = m_cv_stride[1]*m_cv_count[1];
      }
      else
      {
        // there's room for the weight in the existing m_cv array
        for (i = 0; i < m_cv_count[0]; i++) 
        {
          for(j = 0; j < m_cv_count[1]; j++) 
          {
            for(k = 0; k < m_cv_count[2]; k++)
            {
              CV(i,j,k)[dim] = 1.0;
            }
          }
        }
        m_is_rat = 1;
      }
    }
  }
  return IsRational();
}

bool ON_NurbsCage::MakeNonRational()
{
  if ( IsRational() && m_dim > 0 ) 
  {
    int i,j,k,n;
    double* cv;
    double w;

    for (i = 0; i < m_cv_count[0]; i++)
    for (j = 0; j < m_cv_count[1]; j++)
    for (k = 0; k < m_cv_count[2]; k++)
    {
      cv = CV(i,j,k);
      w = cv[m_dim];
      if ( w != 1.0 && w != 0.0 )
      {
        w = 1.0/w;
        n = m_dim;
        while(n--)
        {
          *cv++ *= w;
        }
        *cv = 1.0;
      }
    }

    m_is_rat = 0;
  }
  return ( IsRational() ) ? false : true;
}

/////////////////////////////////////////////////////////////////
// Tools for managing CV and knot memory
bool ON_NurbsCage::ReserveCVCapacity(
  int capacity// number of doubles to reserve
  )
{
  if ( capacity > 0 && m_cv_capacity < capacity ) 
  {
    if ( m_cv ) 
    {
      if ( m_cv_capacity ) 
      {
        m_cv = (double*)onrealloc( m_cv, capacity*sizeof(*m_cv) );
        m_cv_capacity = (m_cv) ? capacity : 0;
      }
      // else user supplied m_cv[] array
    }
    else 
    {
      m_cv = (double*)onmalloc( capacity*sizeof(*m_cv) );
      m_cv_capacity = (m_cv) ? capacity : 0;
    }
  }
  return ( m_cv ) ? true : false;
}

bool ON_NurbsCage::ReserveKnotCapacity(
  int dir,
  int knot_capacity
  )
{
  bool rc = false;
  if ( dir >= 0 && dir <= 2 && knot_capacity > 0 )
  {
    if ( m_knot_capacity[dir] < knot_capacity )
    {
      if ( m_knot[dir] )
      {
        if( m_knot_capacity[dir] )
        {
          m_knot[dir] = (double*)onrealloc(m_knot[dir],knot_capacity*sizeof(m_knot[dir][0]));
          m_knot_capacity[dir] = m_knot[dir] ? knot_capacity : 0;
        }
        // else user supplied m_knot[dir] array.
      }
      else
      {
        m_knot[dir] = (double*)onmalloc(knot_capacity*sizeof(m_knot[dir][0]));
        m_knot_capacity[dir] = m_knot[dir] ? knot_capacity : 0;
      }
    }
    rc = m_knot[dir] != 0;
  }
  return rc;
}



bool ON_NurbsCage::IsSingular(		 // true if surface side is collapsed to a point
       int //side														 // side of parameter space to test
																			// 0 = south, 1 = east, 2 = north, 3 = west, 4 = bottom, 5 =top
				) const
{
  ON_ERROR("TODO: fill in ON_NurbsCage::IsSingular\n");
  return false;
}

bool ON_GetCageXform( const ON_NurbsCage& cage, ON_Xform& cage_xform )
{
  bool rc = false;
  cage_xform.Identity();
  if ( cage.IsValid() )
  {
    ON_3dPoint P000, P100, P010, P001;
    if ( !cage.GetCV(0,0,0,P000) )
      return false;
    if ( !cage.GetCV(cage.CVCount(0)-1,0,0,P100))
      return false;
    if (!cage.GetCV(0,cage.CVCount(1)-1,0,P010))
      return false;
    if (!cage.GetCV(0,0,cage.CVCount(2)-1,P001))
      return false;

    ON_3dVector X0 = P100 - P000;
    ON_3dVector Y0 = P010 - P000;
    ON_3dVector Z0 = P001 - P000;

    double dx0 = X0.Length();
    double dy0 = Y0.Length();
    double dz0 = Z0.Length();

    ON_Interval d0 = cage.Domain(0);
    ON_Interval d1 = cage.Domain(1);
    ON_Interval d2 = cage.Domain(2);

    X0.Unitize();
    Y0.Unitize();
    Z0.Unitize();

    ON_Xform x1;
    x1.Rotation( 
      P000, X0, Y0, Z0, 
      ON_origin, ON_xaxis, ON_yaxis, ON_zaxis 
      );

    ON_Xform x2;
    x2.Scale( d0.Length()/dx0, d1.Length()/dy0, d2.Length()/dz0 );
      
    ON_Xform x3;
    x3.Translation( d0[0],d1[0],d2[0]);


    cage_xform = x3*(x2*x1);
    rc = true;
  }
  return rc;
}

ON_CageMorph::ON_CageMorph()
{
  m_control = 0;
}

ON_CageMorph::~ON_CageMorph()
{
  m_control = 0;
}

bool ON_MorphControl::IsIdentity( const ON_BoundingBox& bbox ) const
{
  int i, count = m_localizers.Count();
  bool rc = (count > 0);
  for (i = 0; i < count && rc; i++ )
  {
    rc = m_localizers[i].IsZero(bbox);
  }
  return rc;
}

bool ON_CageMorph::IsIdentity( const ON_BoundingBox& bbox ) const
{
  return m_control ? m_control->IsIdentity(bbox) : true;
}

ON_MorphControl::ON_MorphControl() 
                : m_varient(0),
                  m_nurbs_cage0(1.0)
{
  m_sporh_tolerance          = 0.0;
  m_sporh_bQuickPreview      = false;
  m_sporh_bPreserveStructure = false;
}


ON_MorphControl::~ON_MorphControl()
{
}

void ON_MorphControl::Destroy()
{
  m_varient = 0;
  m_nurbs_cage0.Identity();
  m_nurbs_curve0.Destroy();
  m_nurbs_curve.Destroy();
  m_nurbs_curve_domain.Destroy();
  m_nurbs_surface0.Destroy();
  m_nurbs_surface.Destroy();
  m_nurbs_surface_domain[0].Destroy();
  m_nurbs_surface_domain[1].Destroy();
  m_nurbs_cage.Destroy();
  m_captive_id.Empty();
  m_localizers.Destroy();
  m_sporh_tolerance = 0.0;
  m_sporh_bQuickPreview = false;
  m_sporh_bPreserveStructure = false;
}


void ON_MorphControl::MemoryRelocate()
{
  m_nurbs_curve0.MemoryRelocate();
  m_nurbs_curve.MemoryRelocate();
  m_nurbs_surface0.MemoryRelocate();
  m_nurbs_surface.MemoryRelocate();
  m_nurbs_cage.MemoryRelocate();
  ON_Geometry::MemoryRelocate();
}

ON_BOOL32 ON_MorphControl::IsValid( ON_TextLog* text_log ) const
{
  ON_BOOL32 rc = false;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve0.IsValid(text_log);
    if (rc)
      rc = m_nurbs_curve.IsValid(text_log);
    break;

  case 2:
    rc = m_nurbs_surface0.IsValid(text_log);
    if (rc)
      rc = m_nurbs_surface.IsValid(text_log);
    break;

  case 3:
    rc = m_nurbs_cage.IsValid(text_log);
    break;

  default:
    rc = false;
    if ( text_log )
    {
      text_log->Print("m_varient = %d - should be 1, 2, or 3\n",m_varient);
    }
    break;
  }
  return rc;
}

void ON_MorphControl::Dump( ON_TextLog& text_log ) const
{
  text_log.Print("Varient: %d\n",m_varient);
  text_log.Print("Control object:\n");
  text_log.PushIndent();
  switch(m_varient)
  {
  case 1:
    m_nurbs_curve0.Dump(text_log);
    m_nurbs_curve.Dump(text_log);
    break;
  case 2:
    m_nurbs_surface0.Dump(text_log);
    m_nurbs_surface.Dump(text_log);
    break;
  case 3:
    text_log.Print(m_nurbs_cage0);
    m_nurbs_cage.Dump(text_log);
    break;
  }
  text_log.PopIndent();
}

unsigned int ON_MorphControl::SizeOf() const
{
  unsigned int sz = sizeof(*this) 
                  - 2*sizeof(ON_NurbsCurve) 
                  - 2*sizeof(ON_NurbsSurface) 
                  - sizeof(m_nurbs_cage);
  sz += m_nurbs_curve0.SizeOf();
  sz += m_nurbs_curve.SizeOf();
  sz += m_nurbs_surface0.SizeOf();
  sz += m_nurbs_surface.SizeOf();
  sz += m_nurbs_cage.SizeOf();
  sz += m_localizers.SizeOfArray();

  return sz;
}


ON::object_type ON_MorphControl::ObjectType() const
{
  return ON::morph_control_object;
}

void ON_MorphControl::DestroyRuntimeCache( bool bDelete )
{
  m_nurbs_curve.DestroyRuntimeCache(bDelete);
  m_nurbs_surface.DestroyRuntimeCache(bDelete);
  m_nurbs_cage.DestroyRuntimeCache(bDelete);
}

int ON_MorphControl::Dimension() const
{
  int dim = 0;
  switch(m_varient)
  {
  case 1:
    dim = m_nurbs_curve.Dimension();
    break;
  case 2:
    dim = m_nurbs_surface.Dimension();
    break;
  case 3:
    dim = m_nurbs_cage.Dimension();
    break;
  }
  return dim;
}

ON_BOOL32 ON_MorphControl::GetBBox(
        double* boxmin,
        double* boxmax,
        int bGrowBox
        ) const
{
  ON_BOOL32 rc = false;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.GetBBox(boxmin,boxmax,bGrowBox);
    break;
  case 2:
    rc = m_nurbs_surface.GetBBox(boxmin,boxmax,bGrowBox);
    break;
  case 3:
    rc = m_nurbs_cage.GetBBox(boxmin,boxmax,bGrowBox);
    break;
  }
  return rc;
}

bool ON_MorphControl::GetTightBoundingBox( 
		ON_BoundingBox& tight_bbox, 
    int bGrowBox,
		const ON_Xform*
    ) const
{
  bool rc = false;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.GetTightBoundingBox(tight_bbox,bGrowBox);
    break;
  case 2:
    rc = m_nurbs_surface.GetTightBoundingBox(tight_bbox,bGrowBox);
    break;
  case 3:
    rc = m_nurbs_cage.GetTightBoundingBox(tight_bbox,bGrowBox);
    break;
  }
  return rc;
}

void ON_MorphControl::ClearBoundingBox()
{
}

ON_BOOL32 ON_MorphControl::Transform( 
        const ON_Xform& xform
        )
{
  ON_BOOL32 rc = false;

  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.Transform(xform);
    break;

  case 2:
    rc = m_nurbs_surface.Transform(xform);
    break;

  case 3:
    rc = m_nurbs_cage.Transform(xform);
    break;
  }

  return rc;
}

ON_BOOL32 ON_MorphControl::HasBrepForm() const
{
  ON_BOOL32 rc = false;

  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.HasBrepForm();
    break;

  case 2:
    rc = m_nurbs_surface.HasBrepForm();
    break;

  case 3:
    rc = m_nurbs_cage.HasBrepForm();
    break;
  }

  return rc;
}

ON_Brep* ON_MorphControl::BrepForm( ON_Brep* brep ) const
{
  switch(m_varient)
  {
  case 1:
    brep = m_nurbs_curve.BrepForm(brep);
    break;

  case 2:
    brep = m_nurbs_surface.BrepForm(brep);
    break;

  case 3:
    brep = m_nurbs_cage.BrepForm(brep);
    break;

  default:
    brep = 0;
    break;
  }

  return brep;
}


bool ON_MorphControl::IsRational() const
{
  bool rc = false;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.IsRational();
    break;
  case 2:
    rc = m_nurbs_surface.IsRational();
    break;
  case 3:
    rc = m_nurbs_cage.IsRational();
    break;
  }
  return rc;
}

bool ON_MorphControl::MakeRational()
{
  bool rc = false;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.MakeRational();
    break;
  case 2:
    rc = m_nurbs_surface.MakeRational();
    break;
  case 3:
    rc = m_nurbs_cage.MakeRational();
    break;
  }
  return rc;
}

bool ON_MorphControl::MakeNonRational()
{
  bool rc = false;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.MakeNonRational();
    break;
  case 2:
    rc = m_nurbs_surface.MakeNonRational();
    break;
  case 3:
    rc = m_nurbs_cage.MakeNonRational();
    break;
  }
  return rc;
}

int ON_MorphControl::CVCount() const
{
  int rc = 0;
  switch(m_varient)
  {
  case 1:
    rc = m_nurbs_curve.CVCount();
    break;
  case 2:
    rc = m_nurbs_surface.CVCount();
    break;
  case 3:
    rc = m_nurbs_cage.CVCount();
    break;
  }
  return rc;
}

int ON_MorphControl::CVCount(int dir) const
{
  int rc = 0;
  switch(m_varient)
  {
  case 1:
    rc = (0==dir) ? m_nurbs_curve.CVCount() : 0;
    break;
  case 2:
    rc = m_nurbs_surface.CVCount(dir);
    break;
  case 3:
    rc = m_nurbs_cage.CVCount(dir);
    break;
  }
  return rc;
}

int ON_MorphControl::Order(int dir) const
{
  int rc = 0;
  switch(m_varient)
  {
  case 1:
    rc = (0==dir) ? m_nurbs_curve.Order() : 0;
    break;
  case 2:
    rc = m_nurbs_surface.Order(dir);
    break;
  case 3:
    rc = m_nurbs_cage.Order(dir);
    break;
  }
  return rc;
}

ON_3dex ON_MorphControl::MaxCVIndex() const
{
  ON_3dex maxdex;
  maxdex.i = maxdex.j = maxdex.k = 0;
  switch(m_varient)
  {
  case 1:
    maxdex.i = m_nurbs_curve.CVCount();
    maxdex.j = maxdex.k = 1;
    break;
  case 2:
    maxdex.i = m_nurbs_surface.CVCount(0);
    maxdex.j = m_nurbs_surface.CVCount(1);
    maxdex.k = 1;
    break;
  case 3:
    maxdex.i = m_nurbs_cage.CVCount(0);
    maxdex.j = m_nurbs_cage.CVCount(1);
    maxdex.k = m_nurbs_cage.CVCount(2);
    break;
  }
  return maxdex;
}

const double* ON_MorphControl::Knot(int dir) const
{
  const double* knot = 0;

  switch(m_varient)
  {
  case 1:
    knot = (0 == dir) ? m_nurbs_curve.m_knot : 0;
    break;
  case 2:
    knot = (0 == dir || 1 == dir) ? m_nurbs_surface.m_knot[dir] : 0;
    break;
  case 3:
    knot = (0 <= dir && dir <= 2) ? m_nurbs_cage.m_knot[dir] : 0;
    break;
  }

  return knot;
}

const double* ON_MorphControl::CV(ON_3dex ijk) const
{
  const double* cv = 0;
  switch(m_varient)
  {
  case 1:
    cv = (0 == ijk.j && 0 == ijk.k) ? m_nurbs_curve.CV(ijk.i) : 0;
    break;
  case 2:
    cv = (0 == ijk.k) ? m_nurbs_surface.CV(ijk.i,ijk.j) : 0;
    break;
  case 3:
    cv = m_nurbs_cage.CV(ijk.i,ijk.j,ijk.k);
    break;
  }
  return cv;
}

double ON_MorphControl::Weight(ON_3dex ijk) const
{
  double w = 1.0;

  switch(m_varient)
  {
  case 1:
    w = (0 == ijk.j && 0 == ijk.k) ? m_nurbs_curve.Weight(ijk.i) : 1.0;
    break;
  case 2:
    w = (0 == ijk.k) ? m_nurbs_surface.Weight(ijk.i,ijk.j) : 1.0;
    break;
  case 3:
    w = m_nurbs_cage.Weight(ijk.i,ijk.j,ijk.k);
    break;
  }

  return w;
}

bool ON_MorphControl::GetCageMorph(ON_CageMorph& cage_morph) const
{
  cage_morph.m_control = this;
  cage_morph.SetPreserveStructure(m_sporh_bPreserveStructure);
  cage_morph.SetQuickPreview(m_sporh_bQuickPreview);
  cage_morph.SetTolerance(m_sporh_tolerance);
  return true;
}

ON_BOOL32 ON_MorphControl::Read( ON_BinaryArchive& archive )
{
  Destroy();
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,
                                      &major_version,
                                      &minor_version);
  while(rc)
  {
    if ( 1 == major_version )
    {
      m_varient = 3;
      if (rc)
        rc = m_nurbs_cage.Read(archive)?true:false;
      if (rc)
        rc = m_captive_id.Read(archive);
      if (rc)
        rc = archive.ReadXform(m_nurbs_cage0);
    }
    else if ( 2 == major_version )
    {
      rc = archive.ReadInt(&m_varient);
      if (!rc) break;

      int mjv = 0;
      int mnv = 0;
      rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&mjv,&mnv);
      if (!rc) break;
      rc = (1 == mjv);
      if (rc)
      {
        switch(m_varient)
        {
        case 1:
          rc = m_nurbs_curve0.Read(archive)?true:false;
          if (rc)
            m_nurbs_curve_domain = m_nurbs_curve0.Domain();
          break;
        case 2:
          rc = m_nurbs_surface0.Read(archive)?true:false;
          if (rc)
          {
            m_nurbs_surface_domain[0] = m_nurbs_surface0.Domain(0);
            m_nurbs_surface_domain[1] = m_nurbs_surface0.Domain(1);
          }
          break;
        case 3:
          rc = archive.ReadXform(m_nurbs_cage0);
          break;
        }
      }
      if ( !archive.EndRead3dmChunk() )
        rc = false;

      if(!rc)
        break;

      mjv = 0;
      mnv = 0;
      rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&mjv,&mnv);
      if (!rc) break;
      rc = (1 == mjv);
      if (rc)
      {
        switch(m_varient)
        {
        case 1:
          rc = m_nurbs_curve.Read(archive)?true:false;
          break;
        case 2:
          rc = m_nurbs_surface.Read(archive)?true:false;
          break;
        case 3:
          rc = m_nurbs_cage.Read(archive)?true:false;
          break;
        }
      }
      if ( !archive.EndRead3dmChunk() )
        rc = false;

      // captive ids
      rc = m_captive_id.Read(archive);
      if (!rc)
        break;

      // localizers
      mjv = 0;
      mnv = 0;
      rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&mjv,&mnv);
      if (!rc) 
        break;
      int i, count = 0;
      rc = (1 == mjv);
      if (rc)
        rc = archive.ReadInt(&count);
      if (rc)
        m_localizers.Reserve(count);
      for ( i = 0; i < count && rc; i++ )
      {
        m_localizers.AppendNew();
        rc = m_localizers[i].Read(archive);
      }
      if ( !archive.EndRead3dmChunk() )
        rc = false;
      if ( !rc)
        break;

      if ( minor_version >= 1 )
      {
        rc = archive.ReadDouble(&m_sporh_tolerance);
        if (!rc) 
          break;
        rc = archive.ReadBool(&m_sporh_bQuickPreview);
        if (!rc) 
          break;
        rc = archive.ReadBool(&m_sporh_bPreserveStructure);
        if (!rc) 
          break;
      }
    }
    else
    {
      rc = false;
    }

    if ( !archive.EndRead3dmChunk() )
      rc = false;
    break;
  }

  return rc;
}

ON_BOOL32 ON_MorphControl::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,2,1);
  if (!rc)
    return false;

  while(rc)
  {
    rc = archive.WriteInt(m_varient);
    if (!rc) break;

    // control start location
    rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
    if (!rc) break;
    switch(m_varient)
    {
    case 1:
      rc = m_nurbs_curve0.Write(archive)?true:false;
      break;
    case 2:
      rc = m_nurbs_surface0.Write(archive)?true:false;
      break;
    case 3:
      rc = archive.WriteXform(m_nurbs_cage0);
      break;
    }
    if ( !archive.EndWrite3dmChunk() )
      rc = false;

    if(!rc)
      break;

    // control end location
    rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
    if (!rc) break;
    switch(m_varient)
    {
    case 1:
      rc = m_nurbs_curve.Write(archive)?true:false;
      break;
    case 2:
      rc = m_nurbs_surface.Write(archive)?true:false;
      break;
    case 3:
      rc = m_nurbs_cage.Write(archive)?true:false;
      break;
    }
    if ( !archive.EndWrite3dmChunk() )
      rc = false;

    if ( !rc)
      break;

    // captive ids
    rc = m_captive_id.Write(archive);
    if (!rc)
      break;

    // localizers
    rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);
    if (!rc) 
      break;
    int i, count = m_localizers.Count();
    rc = archive.WriteInt(count);
    for ( i = 0; i < count && rc; i++ )
    {
      rc = m_localizers[i].Write(archive);
    }
    if ( !archive.EndWrite3dmChunk() )
      rc = false;
    if ( !rc)
      break;

    // 2.1 fields 
    rc = archive.WriteDouble(m_sporh_tolerance);
    if (!rc) 
      break;
    rc = archive.WriteBool(m_sporh_bQuickPreview);
    if (!rc) 
      break;
    rc = archive.WriteBool(m_sporh_bPreserveStructure);
    if (!rc) 
      break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

bool ON_MorphControl::AddControlLocalizer(
  double support_distance, 
  double falloff_distance
  )
{
  bool rc = (support_distance >= 0.0 && falloff_distance > 0.0 );
  if (rc)
  {
    switch(m_varient)
    {
    case 1:
    case 2:
      {
        ON_Localizer& localizer = m_localizers.AppendNew();
        localizer.m_type = ON_Localizer::distance_type;
        localizer.m_d.Set(support_distance+falloff_distance,support_distance);
        rc = true;
      }
      break;

    case 3:
      {
        ON_Xform xform = m_nurbs_cage0;
        xform.Invert();
        ON_Interval d[3];
        d[0] = m_nurbs_cage.Domain(0);
        d[1] = m_nurbs_cage.Domain(1);
        d[2] = m_nurbs_cage.Domain(2);

        ON_SimpleArray<ON_Plane> planes(6);

        ON_3dPoint C(d[0].ParameterAt(0.5),d[1].ParameterAt(0.5),d[2].ParameterAt(0.5));
        ON_3dPoint P;
        ON_3dVector N;
        int i;
        double det = (xform.Determinant() < 0.0) ? -1.0 : 1.0;
        for ( i = 0; i < 3; i++ )
        {
          P = C;
          N.Zero();

          N[i] = -det;
          P[i] = d[i][0];
          ON_Plane& plane0 = planes.AppendNew();
          plane0.CreateFromNormal(P,N);
          plane0.Transform(xform);

          P[i] = d[i][1];
          N[i] = det;
          ON_Plane& plane1 = planes.AppendNew();
          plane1.CreateFromNormal(P,N);
          plane1.Transform(xform);
        }

        rc = AddConvexPolygonLocalizer(planes,support_distance,falloff_distance);
      }
      break;

    default:
      rc = false;
      break;
    }
  }
  return rc;
}

bool ON_MorphControl::AddSphereLocalizer(
  ON_3dPoint center,
  double support_distance, 
  double falloff_distance
  )
{
  bool rc = (center.IsValid() && support_distance >= 0.0 && falloff_distance > 0.0 );
  if (rc)
  {
    ON_Localizer& localizer = m_localizers.AppendNew();
    rc = localizer.CreateSphereLocalizer(
                  center,
                  support_distance+falloff_distance,
                  support_distance);
  }
  return rc;
}

bool ON_MorphControl::AddCylinderLocalizer(
  ON_Line axis,
  double support_distance, 
  double falloff_distance
  )
{
  bool rc = (axis.IsValid() && support_distance >= 0.0 && falloff_distance > 0.0 );
  if (rc)
  {
    ON_Localizer& localizer = m_localizers.AppendNew();
    rc = localizer.CreateCylinderLocalizer(
                  axis.from,axis.Tangent(),
                  support_distance+falloff_distance,
                  support_distance);
  }
  return rc;
}

bool ON_MorphControl::AddBoxLocalizer(
  ON_BoundingBox bbox,
  double support_distance, 
  double falloff_distance
  )
{
  ON_SimpleArray<ON_Plane> planes(6);
  bool rc = (bbox.IsValid() && support_distance >= 0.0 && falloff_distance > 0.0 );
  if (rc)
  {
    ON_3dPoint C = bbox.Center();
    ON_3dVector N;
    ON_3dPoint P;
    int i;
    for ( i = 0; i < 3; i++ )
    {
      P = C;
      N.Zero();
      ON_Plane& plane0 = planes.AppendNew();
      P[i] = bbox.m_min[i];
      N[i] = -1.0;
      plane0.CreateFromNormal(P,N);

      ON_Plane& plane1 = planes.AppendNew();
      P[i] = bbox.m_max[i];
      N[i] = 1.0;
      plane1.CreateFromNormal(P,N);
    }
    rc = AddConvexPolygonLocalizer(planes,support_distance,falloff_distance);
  }
  return rc;
}

bool ON_MorphControl::AddPlaneLocalizer(
            const ON_Plane& plane,
            double support_distance, 
            double falloff_distance
            )
{
  ON_SimpleArray<ON_Plane> planes(1);
  planes.Append(plane);
  return AddConvexPolygonLocalizer(planes,support_distance,falloff_distance);
}

bool ON_MorphControl::AddConvexPolygonLocalizer(
  const ON_SimpleArray<ON_Plane>& planes,
  double support_distance, 
  double falloff_distance
  )
{
  int i, count = planes.Count();
  bool rc = (support_distance >= 0.0 && falloff_distance > 0.0 );
  if (rc)
  {
    m_localizers.Reserve(m_localizers.Count() + count);
    for( i = 0; i < count && rc; i++)
    {
      const ON_Plane& plane = planes[i];
      ON_Localizer& localizer = m_localizers.AppendNew();
      rc = localizer.CreatePlaneLocalizer(
                    plane.origin,plane.zaxis,
                    support_distance+falloff_distance,
                    support_distance);
    }
  }
  return rc;
}


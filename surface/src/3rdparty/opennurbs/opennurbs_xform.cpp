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

static void SwapRow( double matrix[4][4], int i0, int i1 )
{
  double* p0;
  double* p1;
  double t;
  p0 = &matrix[i0][0];
  p1 = &matrix[i1][0];
  t = *p0; *p0++ = *p1; *p1++ = t;
  t = *p0; *p0++ = *p1; *p1++ = t;
  t = *p0; *p0++ = *p1; *p1++ = t;
  t = *p0; *p0   = *p1; *p1 = t;
}

static void SwapCol( double matrix[4][4], int j0, int j1 )
{
  double* p0;
  double* p1;
  double t;
  p0 = &matrix[0][j0];
  p1 = &matrix[0][j1];
  t = *p0; *p0 = *p1; *p1 = t;
  p0 += 4; p1 += 4;
  t = *p0; *p0 = *p1; *p1 = t;
  p0 += 4; p1 += 4;
  t = *p0; *p0 = *p1; *p1 = t;
  p0 += 4; p1 += 4;
  t = *p0; *p0 = *p1; *p1 = t;
}

//static void ScaleRow( double matrix[4][4], double c, int i )
//{
//  double* p = &matrix[i][0];
//  *p++ *= c;
//  *p++ *= c;
//  *p++ *= c;
//  *p   *= c;
//}
//
//static void InvScaleRow( double matrix[4][4], double c, int i )
//{
//  double* p = &matrix[i][0];
//  *p++ /= c;
//  *p++ /= c;
//  *p++ /= c;
//  *p   /= c;
//}

static void AddCxRow( double matrix[4][4], double c, int i0, int i1 )
{
  const double* p0;
  double* p1;
  p0 = &matrix[i0][0];
  p1 = &matrix[i1][0];
  *p1++ += c* *p0++;
  *p1++ += c* *p0++;
  *p1++ += c* *p0++;
  *p1   += c* *p0;
}

/*
static void AddCxCol( double matrix[4][4], double c, int j0, int j1 )
{
  const double* p0;
  double* p1;
  p0 = &matrix[0][j0];
  p1 = &matrix[0][j1];
  *p1 += c* *p0;
  p0 += 4; p1 += 4;
  *p1 += c* *p0;
  p0 += 4; p1 += 4;
  *p1 += c* *p0;
  p0 += 4; p1 += 4;
  *p1 += c* *p0;
}
*/

static int Inv( const double* src, double dst[4][4], double* determinant, double* pivot )
{
  // returns rank (0, 1, 2, 3, or 4), inverse, and smallest pivot

	double M[4][4], I[4][4], x, c, d;
	int i, j, ix, jx;
  int col[4] = {0,1,2,3};
  int swapcount = 0;
  int rank = 0;

  *pivot = 0.0;
  *determinant = 0.0;

  memset( I, 0, sizeof(I) );
  I[0][0] = I[1][1] = I[2][2] = I[3][3] = 1.0;

  memcpy( M, src, sizeof(M) );

  // some loops unrolled for speed

  ix = jx = 0;
	x = fabs(M[0][0]);
  for ( i = 0; i < 4; i++ ) for ( j = 0; j < 4; j++ ) {
    if ( fabs(M[i][j]) > x ) {
      ix = i;
      jx = j;
      x = fabs(M[i][j]);
    }
  }
  *pivot = x;
  if ( ix != 0 ) {
    SwapRow( M, 0, ix );
    SwapRow( I, 0, ix );
    swapcount++;
  }
  if ( jx != 0 ) {
    SwapCol( M, 0, jx );
    col[0] = jx;
    swapcount++;
  }

  if ( x > 0.0 ) {
    rank++;

    // 17 August 2011 Dale Lear
    //   The result is slightly more accurate when using division
    //   instead of multiplying by the inverse of M[0][0]. If there
    //   is any speed penalty at this point in history, the accuracy
    //   is more important than the additional clocks.
    //c = d = 1.0/M[0][0];
    //M[0][1] *= c; M[0][2] *= c; M[0][3] *= c;
    //ScaleRow( I, c, 0 );
    c = M[0][0];
    M[0][1] /= c; M[0][2] /= c; M[0][3] /= c;
    I[0][0] /= c; I[0][1] /= c; I[0][2] /= c; I[0][3] /= c;
    d = 1.0/c;

    x *=  ON_EPSILON;

	  if (fabs(M[1][0]) > x) {
		  c = -M[1][0];
      M[1][1] += c*M[0][1]; M[1][2] += c*M[0][2]; M[1][3] += c*M[0][3];
      AddCxRow( I, c, 0, 1 );
	  }
	  if (fabs(M[2][0]) >  x) {
		  c = -M[2][0];
      M[2][1] += c*M[0][1]; M[2][2] += c*M[0][2]; M[2][3] += c*M[0][3];
      AddCxRow( I, c, 0, 2 );
	  }
	  if (fabs(M[3][0]) >  x) {
		  c = -M[3][0];
      M[3][1] += c*M[0][1]; M[3][2] += c*M[0][2]; M[3][3] += c*M[0][3];
      AddCxRow( I, c, 0, 3 );
	  }

    ix = jx = 1;
	  x = fabs(M[1][1]);
    for ( i = 1; i < 4; i++ ) for ( j = 1; j < 4; j++ ) {
      if ( fabs(M[i][j]) > x ) {
        ix = i;
        jx = j;
        x = fabs(M[i][j]);
      }
    }
    if ( x < *pivot )
      *pivot = x;
    if ( ix != 1 ) {
      SwapRow( M, 1, ix );
      SwapRow( I, 1, ix );
      swapcount++;
    }
    if ( jx != 1 ) {
      SwapCol( M, 1, jx );
      col[1] = jx;
      swapcount++;
    }
    if ( x > 0.0 ) {
      rank++;

      // 17 August 2011 Dale Lear
      //   The result is slightly more accurate when using division
      //   instead of multiplying by the inverse of M[1][1]. If there
      //   is any speed penalty at this point in history, the accuracy
      //   is more important than the additional clocks.
      //c = 1.0/M[1][1];
      //d *= c;
      //M[1][2] *= c; M[1][3] *= c;
      //ScaleRow( I, c, 1 );
      c = M[1][1];
      M[1][2] /= c; M[1][3] /= c;
      I[1][0] /= c; I[1][1] /= c; I[1][2] /= c; I[1][3] /= c;
      d /= c;

      x *= ON_EPSILON;
      if (fabs(M[0][1]) >  x) {
        c = -M[0][1];
        M[0][2] += c*M[1][2]; M[0][3] += c*M[1][3];
        AddCxRow( I, c, 1, 0 );
      }
      if (fabs(M[2][1]) >  x) {
        c = -M[2][1];
        M[2][2] += c*M[1][2]; M[2][3] += c*M[1][3];
        AddCxRow( I, c, 1, 2 );
      }
      if (fabs(M[3][1]) >  x) {
        c = -M[3][1];
        M[3][2] += c*M[1][2]; M[3][3] += c*M[1][3];
        AddCxRow( I, c, 1, 3 );
      }

      ix = jx = 2;
	    x = fabs(M[2][2]);
      for ( i = 2; i < 4; i++ ) for ( j = 2; j < 4; j++ ) {
        if ( fabs(M[i][j]) > x ) {
          ix = i;
          jx = j;
          x = fabs(M[i][j]);
        }
      }
      if ( x < *pivot )
        *pivot = x;
      if ( ix != 2 ) {
        SwapRow( M, 2, ix );
        SwapRow( I, 2, ix );
        swapcount++;
      }
      if ( jx != 2 ) {
        SwapCol( M, 2, jx );
        col[2] = jx;
        swapcount++;
      }
      if ( x > 0.0 ) {
        rank++;

        // 17 August 2011 Dale Lear
        //   The result is slightly more accurate when using division
        //   instead of multiplying by the inverse of M[2][2]. If there
        //   is any speed penalty at this point in history, the accuracy
        //   is more important than the additional clocks.
        //c = 1.0/M[2][2];
        //d *= c;
        //M[2][3] *= c;
        //ScaleRow( I, c, 2 );
        c = M[2][2];
        M[2][3] /= c;
        I[2][0] /= c; I[2][1] /= c; I[2][2] /= c; I[2][3] /= c;
        d /= c;

        x *= ON_EPSILON;
        if (fabs(M[0][2]) >  x) {
          c = -M[0][2];
          M[0][3] += c*M[2][3];
          AddCxRow( I, c, 2, 0 );
        }
        if (fabs(M[1][2]) >  x) {
          c = -M[1][2];
          M[1][3] += c*M[2][3];
          AddCxRow( I, c, 2, 1 );
        }
        if (fabs(M[3][2]) >  x) {
          c = -M[3][2];
          M[3][3] += c*M[2][3];
          AddCxRow( I, c, 2, 3 );
        }

        x = fabs(M[3][3]);
        if ( x < *pivot )
          *pivot = x;

        if ( x > 0.0 ) {
          rank++;

          // 17 August 2011 Dale Lear
          //   The result is slightly more accurate when using division
          //   instead of multiplying by the inverse of M[3][3]. If there
          //   is any speed penalty at this point in history, the accuracy
          //   is more important than the additional clocks.
          //c = 1.0/M[3][3];
          //d *= c;
          //ScaleRow( I, c, 3 );
          c = M[3][3];
          I[3][0] /= c; I[3][1] /= c; I[3][2] /= c; I[3][3] /= c;
          d /= c;

          x *= ON_EPSILON;
          if (fabs(M[0][3]) >  x) {
            AddCxRow( I, -M[0][3], 3, 0 );
          }
          if (fabs(M[1][3]) >  x) {
            AddCxRow( I, -M[1][3], 3, 1 );
          }
          if (fabs(M[2][3]) >  x) {
            AddCxRow( I, -M[2][3], 3, 2 );
          }

          *determinant = (swapcount%2) ? -d : d;
        }
      }
    }
  }

  if ( col[3] != 3 )
    SwapRow( I, 3, col[3] );
  if ( col[2] != 2 )
    SwapRow( I, 2, col[2] );
  if ( col[1] != 1 )
    SwapRow( I, 1, col[1] );
  if ( col[0] != 0 )
    SwapRow( I, 0, col[0] );

  memcpy( dst, I, sizeof(I) );
	return rank;
}

///////////////////////////////////////////////////////////////
//
// ON_Xform constructors
//

ON_Xform::ON_Xform()
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[3][3] = 1.0;
}

ON_Xform::ON_Xform( int d )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = (double)d;
  m_xform[3][3] = 1.0;
}

ON_Xform::ON_Xform( double d )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = d;
  m_xform[3][3] = 1.0;
}

#if defined(ON_COMPILER_MSC)
ON_Xform::ON_Xform( double m[4][4] )
{
  memcpy( &m_xform[0][0], &m[0][0], sizeof(m_xform) );
}
#endif

ON_Xform::ON_Xform( const double m[4][4] )
{
  memcpy( &m_xform[0][0], &m[0][0], sizeof(m_xform) );
}

#if defined(ON_COMPILER_MSC)
ON_Xform::ON_Xform( float m[4][4] )
{
  m_xform[0][0] = (double)m[0][0];
  m_xform[0][1] = (double)m[0][1];
  m_xform[0][2] = (double)m[0][2];
  m_xform[0][3] = (double)m[0][3];

  m_xform[1][0] = (double)m[1][0];
  m_xform[1][1] = (double)m[1][1];
  m_xform[1][2] = (double)m[1][2];
  m_xform[1][3] = (double)m[1][3];

  m_xform[2][0] = (double)m[2][0];
  m_xform[2][1] = (double)m[2][1];
  m_xform[2][2] = (double)m[2][2];
  m_xform[2][3] = (double)m[2][3];

  m_xform[3][0] = (double)m[3][0];
  m_xform[3][1] = (double)m[3][1];
  m_xform[3][2] = (double)m[3][2];
  m_xform[3][3] = (double)m[3][3];
}
#endif

ON_Xform::ON_Xform( const float m[4][4] )
{
  m_xform[0][0] = (double)m[0][0];
  m_xform[0][1] = (double)m[0][1];
  m_xform[0][2] = (double)m[0][2];
  m_xform[0][3] = (double)m[0][3];

  m_xform[1][0] = (double)m[1][0];
  m_xform[1][1] = (double)m[1][1];
  m_xform[1][2] = (double)m[1][2];
  m_xform[1][3] = (double)m[1][3];

  m_xform[2][0] = (double)m[2][0];
  m_xform[2][1] = (double)m[2][1];
  m_xform[2][2] = (double)m[2][2];
  m_xform[2][3] = (double)m[2][3];

  m_xform[3][0] = (double)m[3][0];
  m_xform[3][1] = (double)m[3][1];
  m_xform[3][2] = (double)m[3][2];
  m_xform[3][3] = (double)m[3][3];
}

ON_Xform::ON_Xform( const double* m )
{
  memcpy( &m_xform[0][0], m, sizeof(m_xform) );
}

ON_Xform::ON_Xform( const float* m )
{
  m_xform[0][0] = (double)m[0];
  m_xform[0][1] = (double)m[1];
  m_xform[0][2] = (double)m[2];
  m_xform[0][3] = (double)m[3];

  m_xform[1][0] = (double)m[4];
  m_xform[1][1] = (double)m[5];
  m_xform[1][2] = (double)m[6];
  m_xform[1][3] = (double)m[7];

  m_xform[2][0] = (double)m[8];
  m_xform[2][1] = (double)m[9];
  m_xform[2][2] = (double)m[10];
  m_xform[2][3] = (double)m[11];

  m_xform[3][0] = (double)m[12];
  m_xform[3][1] = (double)m[13];
  m_xform[3][2] = (double)m[14];
  m_xform[3][3] = (double)m[15];
}

ON_Xform::ON_Xform( const ON_3dPoint& P,
														 const ON_3dVector& X,
														 const ON_3dVector& Y,
														 const ON_3dVector& Z)
{
  m_xform[0][0] = X[0];
  m_xform[1][0] = X[1];
  m_xform[2][0] = X[2];
  m_xform[3][0] = 0;

  m_xform[0][1] = Y[0];
  m_xform[1][1] = Y[1];
  m_xform[2][1] = Y[2];
  m_xform[3][1] = 0;

  m_xform[0][2] = Z[0];
  m_xform[1][2] = Z[1];
  m_xform[2][2] = Z[2];
  m_xform[3][2] = 0;

  m_xform[0][3] = P[0];
  m_xform[1][3] = P[1];
  m_xform[2][3] = P[2];
  m_xform[3][3] = 1;
}

ON_Xform::ON_Xform( const ON_Matrix& m )
{
  *this = m;
}

///////////////////////////////////////////////////////////////
//
// ON_Xform operator[]
//


double* ON_Xform::operator[](int i)
{
  return ( i >= 0 && i < 4 ) ? &m_xform[i][0] : NULL;
}

const double* ON_Xform::operator[](int i) const
{
  return ( i >= 0 && i < 4 ) ? &m_xform[i][0] : NULL;
}

///////////////////////////////////////////////////////////////
//
// ON_Xform operator=
//

ON_Xform& ON_Xform::operator=( int d )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = (double)d;
  m_xform[3][3] = 1.0;
  return *this;
}

ON_Xform& ON_Xform::operator=( float d )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = (double)d;
  m_xform[3][3] = 1.0;
  return *this;
}

ON_Xform& ON_Xform::operator=( double d )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = d;
  m_xform[3][3] = 1.0;
  return *this;
}
  
///////////////////////////////////////////////////////////////
//
// ON_Xform operator* operator- operator+
//
// All non-commutative operations have "this" as left hand side and
// argument as right hand side.
ON_Xform ON_Xform::operator*( const ON_Xform& rhs ) const
{
  double m[4][4];
  const double* p = &rhs.m_xform[0][0];

  m[0][0] = m_xform[0][0]*p[0] + m_xform[0][1]*p[4] + m_xform[0][2]*p[ 8] + m_xform[0][3]*p[12];
  m[0][1] = m_xform[0][0]*p[1] + m_xform[0][1]*p[5] + m_xform[0][2]*p[ 9] + m_xform[0][3]*p[13];
  m[0][2] = m_xform[0][0]*p[2] + m_xform[0][1]*p[6] + m_xform[0][2]*p[10] + m_xform[0][3]*p[14];
  m[0][3] = m_xform[0][0]*p[3] + m_xform[0][1]*p[7] + m_xform[0][2]*p[11] + m_xform[0][3]*p[15];

  m[1][0] = m_xform[1][0]*p[0] + m_xform[1][1]*p[4] + m_xform[1][2]*p[ 8] + m_xform[1][3]*p[12];
  m[1][1] = m_xform[1][0]*p[1] + m_xform[1][1]*p[5] + m_xform[1][2]*p[ 9] + m_xform[1][3]*p[13];
  m[1][2] = m_xform[1][0]*p[2] + m_xform[1][1]*p[6] + m_xform[1][2]*p[10] + m_xform[1][3]*p[14];
  m[1][3] = m_xform[1][0]*p[3] + m_xform[1][1]*p[7] + m_xform[1][2]*p[11] + m_xform[1][3]*p[15];

  m[2][0] = m_xform[2][0]*p[0] + m_xform[2][1]*p[4] + m_xform[2][2]*p[ 8] + m_xform[2][3]*p[12];
  m[2][1] = m_xform[2][0]*p[1] + m_xform[2][1]*p[5] + m_xform[2][2]*p[ 9] + m_xform[2][3]*p[13];
  m[2][2] = m_xform[2][0]*p[2] + m_xform[2][1]*p[6] + m_xform[2][2]*p[10] + m_xform[2][3]*p[14];
  m[2][3] = m_xform[2][0]*p[3] + m_xform[2][1]*p[7] + m_xform[2][2]*p[11] + m_xform[2][3]*p[15];

  m[3][0] = m_xform[3][0]*p[0] + m_xform[3][1]*p[4] + m_xform[3][2]*p[ 8] + m_xform[3][3]*p[12];
  m[3][1] = m_xform[3][0]*p[1] + m_xform[3][1]*p[5] + m_xform[3][2]*p[ 9] + m_xform[3][3]*p[13];
  m[3][2] = m_xform[3][0]*p[2] + m_xform[3][1]*p[6] + m_xform[3][2]*p[10] + m_xform[3][3]*p[14];
  m[3][3] = m_xform[3][0]*p[3] + m_xform[3][1]*p[7] + m_xform[3][2]*p[11] + m_xform[3][3]*p[15];

  return ON_Xform(m);
}

ON_Xform ON_Xform::operator+( const ON_Xform& rhs ) const
{
  double m[4][4];
  const double* p = &rhs.m_xform[0][0];

  m[0][0] = m_xform[0][0] + p[0];
  m[0][1] = m_xform[0][1] + p[1];
  m[0][2] = m_xform[0][2] + p[2];
  m[0][3] = m_xform[0][3] + p[3];

  m[1][0] = m_xform[1][0] + p[4];
  m[1][1] = m_xform[1][1] + p[5];
  m[1][2] = m_xform[1][2] + p[6];
  m[1][3] = m_xform[1][3] + p[7];

  m[2][0] = m_xform[2][0] + p[ 8];
  m[2][1] = m_xform[2][1] + p[ 9];
  m[2][2] = m_xform[2][2] + p[10];
  m[2][3] = m_xform[2][3] + p[11];

  m[3][0] = m_xform[3][0] + p[12];
  m[3][1] = m_xform[3][1] + p[13];
  m[3][2] = m_xform[3][2] + p[14];
  m[3][3] = m_xform[3][3] + p[15];

  return ON_Xform(m);
}

ON_Xform ON_Xform::operator-( const ON_Xform& rhs ) const
{
  double m[4][4];
  const double* p = &rhs.m_xform[0][0];

  m[0][0] = m_xform[0][0] - p[0];
  m[0][1] = m_xform[0][1] - p[1];
  m[0][2] = m_xform[0][2] - p[2];
  m[0][3] = m_xform[0][3] - p[3];

  m[1][0] = m_xform[1][0] - p[4];
  m[1][1] = m_xform[1][1] - p[5];
  m[1][2] = m_xform[1][2] - p[6];
  m[1][3] = m_xform[1][3] - p[7];

  m[2][0] = m_xform[2][0] - p[ 8];
  m[2][1] = m_xform[2][1] - p[ 9];
  m[2][2] = m_xform[2][2] - p[10];
  m[2][3] = m_xform[2][3] - p[11];

  m[3][0] = m_xform[3][0] - p[12];
  m[3][1] = m_xform[3][1] - p[13];
  m[3][2] = m_xform[3][2] - p[14];
  m[3][3] = m_xform[3][3] - p[15];

  return ON_Xform(m);
}
  
///////////////////////////////////////////////////////////////
//
// ON_Xform
//


void ON_Xform::Zero()
{
  memset( m_xform, 0, sizeof(m_xform) );
}

void ON_Xform::Identity()
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = m_xform[3][3] = 1.0;
}

void ON_Xform::Diagonal( double d )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = m_xform[1][1] = m_xform[2][2] = d;
  m_xform[3][3] = 1.0;
}

void ON_Xform::Scale( double x, double y, double z )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = x;
  m_xform[1][1] = y;
  m_xform[2][2] = z;
  m_xform[3][3] = 1.0;
}

void ON_Xform::Scale( const ON_3dVector& v )
{
  memset( m_xform, 0, sizeof(m_xform) );
  m_xform[0][0] = v.x;
  m_xform[1][1] = v.y;
  m_xform[2][2] = v.z;
  m_xform[3][3] = 1.0;
}

void ON_Xform::Scale
  (
  ON_3dPoint fixed_point,
  double scale_factor
  )
{
  if ( fixed_point.x == 0.0 && fixed_point.y == 0.0 && fixed_point.z == 0.0 )
  {
    Scale( scale_factor, scale_factor, scale_factor );
  }
  else
  {
    ON_Xform t0, t1, s;
    t0.Translation( ON_origin - fixed_point );
    s.Scale( scale_factor, scale_factor, scale_factor );
    t1.Translation( fixed_point - ON_origin );
    operator=(t1*s*t0);
  }
}

void ON_Xform::Scale
  (
  const ON_Plane& plane,
  double x_scale_factor,
  double y_scale_factor,
  double z_scale_factor
  )
{
  Shear( plane, x_scale_factor*plane.xaxis, y_scale_factor*plane.yaxis, z_scale_factor*plane.zaxis );
}

void ON_Xform::Shear
  (
  const ON_Plane& plane,
  const ON_3dVector& x1,
  const ON_3dVector& y1,
  const ON_3dVector& z1
  )
{
  ON_Xform t0, t1, s0(1), s1(1);
  t0.Translation( ON_origin - plane.origin );
  s0.m_xform[0][0] = plane.xaxis.x;
  s0.m_xform[0][1] = plane.xaxis.y;
  s0.m_xform[0][2] = plane.xaxis.z;
  s0.m_xform[1][0] = plane.yaxis.x;
  s0.m_xform[1][1] = plane.yaxis.y;
  s0.m_xform[1][2] = plane.yaxis.z;
  s0.m_xform[2][0] = plane.zaxis.x;
  s0.m_xform[2][1] = plane.zaxis.y;
  s0.m_xform[2][2] = plane.zaxis.z;
  s1.m_xform[0][0] = x1.x;
  s1.m_xform[1][0] = x1.y;
  s1.m_xform[2][0] = x1.z;
  s1.m_xform[0][1] = y1.x;
  s1.m_xform[1][1] = y1.y;
  s1.m_xform[2][1] = y1.z;
  s1.m_xform[0][2] = z1.x;
  s1.m_xform[1][2] = z1.y;
  s1.m_xform[2][2] = z1.z;
  t1.Translation( plane.origin - ON_origin );
  operator=(t1*s1*s0*t0);
}

void ON_Xform::Translation( double x, double y, double z )
{
  Identity();
  m_xform[0][3] = x;
  m_xform[1][3] = y;
  m_xform[2][3] = z;
  m_xform[3][3] = 1.0;
}

void ON_Xform::Translation( const ON_3dVector& v )
{
  Identity();
  m_xform[0][3] = v.x;
  m_xform[1][3] = v.y;
  m_xform[2][3] = v.z;
  m_xform[3][3] = 1.0;
}

void ON_Xform::PlanarProjection( const ON_Plane& plane )
{
  int i, j;
  double x[3] = {plane.xaxis.x,plane.xaxis.y,plane.xaxis.z};
  double y[3] = {plane.yaxis.x,plane.yaxis.y,plane.yaxis.z};
  double p[3] = {plane.origin.x,plane.origin.y,plane.origin.z};
  double q[3];
  for ( i = 0; i < 3; i++ ) 
  {
    for ( j = 0; j < 3; j++ )
    {
      m_xform[i][j] = x[i]*x[j] + y[i]*y[j];
    }
    q[i] = m_xform[i][0]*p[0] + m_xform[i][1]*p[1] + m_xform[i][2]*p[2];
  }
  for ( i = 0; i < 3; i++ )
  {
    m_xform[3][i] = 0.0;
    m_xform[i][3] = p[i]-q[i];
  }
  m_xform[3][3] = 1.0;
}

///////////////////////////////////////////////////////////////
//
// ON_Xform
//

void ON_Xform::ActOnLeft(double x,double y,double z,double w,double v[4]) const
{
  if ( v )
  {
    v[0] = m_xform[0][0]*x + m_xform[0][1]*y + m_xform[0][2]*z + m_xform[0][3]*w;
    v[1] = m_xform[1][0]*x + m_xform[1][1]*y + m_xform[1][2]*z + m_xform[1][3]*w;
    v[2] = m_xform[2][0]*x + m_xform[2][1]*y + m_xform[2][2]*z + m_xform[2][3]*w;
    v[3] = m_xform[3][0]*x + m_xform[3][1]*y + m_xform[3][2]*z + m_xform[3][3]*w;
  }
}

void ON_Xform::ActOnRight(double x,double y,double z,double w,double v[4]) const
{
  if ( v )
  {
    v[0] = m_xform[0][0]*x + m_xform[1][0]*y + m_xform[2][0]*z + m_xform[3][0]*w;
    v[1] = m_xform[0][1]*x + m_xform[1][1]*y + m_xform[2][1]*z + m_xform[3][1]*w;
    v[2] = m_xform[0][2]*x + m_xform[1][2]*y + m_xform[2][2]*z + m_xform[3][2]*w;
    v[3] = m_xform[0][3]*x + m_xform[1][3]*y + m_xform[2][3]*z + m_xform[3][3]*w;
  }
}

ON_2dPoint ON_Xform::operator*( const ON_2dPoint& p ) const
{
  const double x = p.x; // optimizer should put x,y in registers
  const double y = p.y;
  double xh[2], w;
  const double* m = &m_xform[0][0];
  xh[0] = m[ 0]*x + m[ 1]*y + m[ 3];
  xh[1] = m[ 4]*x + m[ 5]*y + m[ 7];
  w     = m[12]*x + m[13]*y + m[15];
  w = (w != 0.0) ? 1.0/w : 1.0;
  return ON_2dPoint( w*xh[0], w*xh[1] );
}

ON_3dPoint ON_Xform::operator*( const ON_3dPoint& p ) const
{
  const double x = p.x; // optimizer should put x,y,z in registers
  const double y = p.y;
  const double z = p.z;
  double xh[3], w;
  const double* m = &m_xform[0][0];
  xh[0] = m[ 0]*x + m[ 1]*y + m[ 2]*z + m[ 3];
  xh[1] = m[ 4]*x + m[ 5]*y + m[ 6]*z + m[ 7];
  xh[2] = m[ 8]*x + m[ 9]*y + m[10]*z + m[11];
  w     = m[12]*x + m[13]*y + m[14]*z + m[15];
  w = (w != 0.0) ? 1.0/w : 1.0;
  return ON_3dPoint( w*xh[0], w*xh[1], w*xh[2] );
}

ON_4dPoint ON_Xform::operator*( const ON_4dPoint& h ) const
{
  const double x = h.x; // optimizer should put x,y,z,w in registers
  const double y = h.y;
  const double z = h.z;
  const double w = h.w;
  double xh[4];
  const double* m = &m_xform[0][0];
  xh[0] = m[ 0]*x + m[ 1]*y + m[ 2]*z + m[ 3]*w;
  xh[1] = m[ 4]*x + m[ 5]*y + m[ 6]*z + m[ 7]*w;
  xh[2] = m[ 8]*x + m[ 9]*y + m[10]*z + m[11]*w;
  xh[3] = m[12]*x + m[13]*y + m[14]*z + m[15]*w;
  return ON_4dPoint( xh[0],xh[1],xh[2],xh[3] );
}

ON_2dVector ON_Xform::operator*( const ON_2dVector& v ) const
{
  const double x = v.x; // optimizer should put x,y in registers
  const double y = v.y;
  double xh[2];
  const double* m = &m_xform[0][0];
  xh[0] = m[0]*x + m[1]*y;
  xh[1] = m[4]*x + m[5]*y;
  return ON_2dVector( xh[0],xh[1] );
}

ON_3dVector ON_Xform::operator*( const ON_3dVector& v ) const
{
  const double x = v.x; // optimizer should put x,y,z in registers
  const double y = v.y;
  const double z = v.z;
  double xh[3];
  const double* m = &m_xform[0][0];
  xh[0] = m[0]*x + m[1]*y + m[ 2]*z;
  xh[1] = m[4]*x + m[5]*y + m[ 6]*z;
  xh[2] = m[8]*x + m[9]*y + m[10]*z;
  return ON_3dVector( xh[0],xh[1],xh[2] );
}

bool ON_Xform::IsValid() const
{
  int i;
  const double* x = &m_xform[0][0];
  bool rc = true;
  for (i = 0; i < 16 && rc; i++)
  {
    rc = ON_IsValid(*x++);
  }
  return rc;
}

bool ON_Xform::IsIdentity( double zero_tolerance ) const
{
  // The code below will return false if m_xform[][] contains
  // a nan value.
  const double* v = &m_xform[0][0];
  for ( int i = 0; i < 3; i++ )
  {
    if ( !(fabs(1.0 - *v++) <= zero_tolerance) )
      return false;
    if ( !(fabs(*v++) <= zero_tolerance) )
      return false;
    if ( !(fabs(*v++) <= zero_tolerance) )
      return false;
    if ( !(fabs(*v++) <= zero_tolerance) )
      return false;
    if ( !(fabs(*v++) <= zero_tolerance) )
      return false;
  }
  if ( !(fabs( 1.0 - *v ) <= zero_tolerance) )
    return false;
  return true;
}

bool ON_Xform::IsNotIdentity( double zero_tolerance ) const
{
  // The code below will return false if m_xform[][] contains
  // a nan value.
  const double* v = &m_xform[0][0];
  for ( int i = 0; i < 3; i++ )
  {
    if ( fabs(1.0 - *v++) > zero_tolerance )
      return true;
    if ( fabs(*v++) > zero_tolerance )
      return true;
    if ( fabs(*v++) > zero_tolerance )
      return true;
    if ( fabs(*v++) > zero_tolerance )
      return true;
    if ( fabs(*v++) > zero_tolerance )
      return true;
  }
  if ( fabs( 1.0 - *v ) > zero_tolerance )
    return true;

  return false;
}

bool ON_Xform::IsTranslation( double zero_tolerance ) const
{
  const double* v = &m_xform[0][0];
  if ( fabs(1.0 - *v++) > zero_tolerance )
    return false;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  v++;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs(1.0 - *v++) > zero_tolerance )
    return false;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  v++;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs(1.0 - *v++) > zero_tolerance )
    return false;
  v++;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs(*v++) >  zero_tolerance )
    return false;
  if ( fabs( 1.0 - *v ) > zero_tolerance )
    return false;
  return true;
}


int ON_Xform::Compare( const ON_Xform& other ) const
{
  const double* a = &m_xform[0][0];
  const double* b = &other.m_xform[0][0];
  int i = 16;
  while(i--)
  {
    if ( *a < *b )
      return -1;
    if ( *a > *b )
      return 1;
    a++;
    b++;
  }
  return 0;
}

int ON_Xform::IsSimilarity() const
{
  int rc = 0;
  if (    m_xform[3][0] != 0.0 
       || m_xform[3][1] != 0.0
       || m_xform[3][2] != 0.0
       || m_xform[3][3] != 1.0 )
  {
    rc = 0;
  }
  else
  {
    double tol = 1.0e-4;
    double dottol = 1.0e-3;
    double det = Determinant();
    if ( fabs(det) <= ON_SQRT_EPSILON )
    {
      // projection or worse
      rc = 0;
    }
    else
    {
      ON_3dVector X(m_xform[0][0],m_xform[1][0],m_xform[2][0]);
      ON_3dVector Y(m_xform[0][1],m_xform[1][1],m_xform[2][1]);
      ON_3dVector Z(m_xform[0][2],m_xform[1][2],m_xform[2][2]);
      double sx = X.Length();
      double sy = Y.Length();
      double sz = Z.Length();
      if (   sx == 0.0 || sy == 0.0 || sz == 0.0 
          || fabs(sx-sy) > tol || fabs(sy-sz) > tol || fabs(sz-sx) > tol )
      {
        // non-uniform scale or worse
        rc = 0;
      }
      else
      {
        double xy = (X*Y)/(sx*sy);
        double yz = (Y*Z)/(sy*sz);
        double zx = (Z*X)/(sz*sx);
        if ( fabs(xy) > dottol || fabs(yz) > dottol || fabs(zx) > dottol )
        {
          // shear or worse
          rc = 0;
        }
        else
        {
          rc = (det > 0.0) ? 1 : -1;
        }
      }
    }
  }
  return rc;
}


bool ON_Xform::IsZero() const
{
  const double* v = &m_xform[0][0];
  for ( int i = 0; i < 15; i++ )
  {
    if ( *v++ != 0.0 )
      return false;
  }
  return true;
}


void ON_Xform::Transpose()
{
  double t;
  t = m_xform[0][1]; m_xform[0][1] = m_xform[1][0]; m_xform[1][0] = t;
  t = m_xform[0][2]; m_xform[0][2] = m_xform[2][0]; m_xform[2][0] = t;
  t = m_xform[0][3]; m_xform[0][3] = m_xform[3][0]; m_xform[3][0] = t;
  t = m_xform[1][2]; m_xform[1][2] = m_xform[2][1]; m_xform[2][1] = t;
  t = m_xform[1][3]; m_xform[1][3] = m_xform[3][1]; m_xform[3][1] = t;
  t = m_xform[2][3]; m_xform[2][3] = m_xform[3][2]; m_xform[3][2] = t;
}

int ON_Xform::Rank( double* pivot ) const
{
  double I[4][4], d = 0.0, p = 0.0;
  int r = Inv( &m_xform[0][0], I, &d, &p );
  if ( pivot )
    *pivot = p;
  return r;
}

double ON_Xform::Determinant( double* pivot ) const
{
  double I[4][4], d = 0.0, p = 0.0;
  //int rank = 
  Inv( &m_xform[0][0], I, &d, &p );
  if ( pivot )
    *pivot = p;
  if (d != 0.0 )
    d = 1.0/d;
  return d;
}

bool ON_Xform::Invert( double* pivot )
{
  double mrofx[4][4], d = 0.0, p = 0.0;
  int rank = Inv( &m_xform[0][0], mrofx, &d, &p );
  memcpy( m_xform, mrofx, sizeof(m_xform) );
  if ( pivot )
    *pivot = p;
  return (rank == 4) ? true : false;
}

ON_Xform ON_Xform::Inverse( double* pivot ) const
{
  ON_Xform inv;
  double d = 0.0, p = 0.0;
  //int rank = 
  Inv( &m_xform[0][0], inv.m_xform, &d, &p );
  if ( pivot )
    *pivot = p;
  return inv;
}

double ON_Xform::GetSurfaceNormalXform( ON_Xform& N_xform ) const
{
  // since were are transforming vectors, we don't need
  // the translation column or bottom row.
  memcpy(&N_xform.m_xform[0][0],&m_xform[0][0], 3*sizeof(N_xform.m_xform[0][0]) );
  N_xform.m_xform[0][3] = 0.0;
  memcpy(&N_xform.m_xform[1][0],&m_xform[1][0], 3*sizeof(N_xform.m_xform[0][0]) );
  N_xform.m_xform[1][3] = 0.0;
  memcpy(&N_xform.m_xform[2][0],&m_xform[2][0], 3*sizeof(N_xform.m_xform[0][0]) );
  N_xform.m_xform[2][3] = 0.0;
  N_xform.m_xform[3][0] = 0.0;
  N_xform.m_xform[3][1] = 0.0;
  N_xform.m_xform[3][2] = 0.0;
  N_xform.m_xform[3][3] = 1.0;

  double mrofx[4][4], d = 0.0, p = 0.0;
  double dtol = ON_SQRT_EPSILON*ON_SQRT_EPSILON*ON_SQRT_EPSILON;
  if ( 4 == Inv( &N_xform.m_xform[0][0], mrofx, &d, &p ) 
       && fabs(d) > dtol 
       && fabs(d)*dtol < 1.0
       && fabs(p) > ON_EPSILON*fabs(d)
     )
  {
    // Set N_xform = transpose of mrofx (only upper 3x3 matters)
    N_xform.m_xform[0][0] = mrofx[0][0];
    N_xform.m_xform[0][1] = mrofx[1][0]; 
    N_xform.m_xform[0][2] = mrofx[2][0];

    N_xform.m_xform[1][0] = mrofx[0][1];
    N_xform.m_xform[1][1] = mrofx[1][1];
    N_xform.m_xform[1][2] = mrofx[2][1];

    N_xform.m_xform[2][0] = mrofx[0][2];
    N_xform.m_xform[2][1] = mrofx[1][2];
    N_xform.m_xform[2][2] = mrofx[2][2];
  }
  else
  {
    d = 0.0;
  }
  return d;
}

double ON_Xform::GetMappingXforms( ON_Xform& P_xform, ON_Xform& N_xform ) const
{
  double d = 0.0, p = 0.0;
  double dtol = ON_SQRT_EPSILON*ON_SQRT_EPSILON*ON_SQRT_EPSILON;
  if ( 4 == Inv( &m_xform[0][0], P_xform.m_xform, &d, &p ) 
       && fabs(d) > dtol 
       && fabs(d)*dtol < 1.0
       && fabs(p) > ON_EPSILON*fabs(d)
     )
  {
    // Set N_xform = transpose of this (only upper 3x3 matters)
    N_xform.m_xform[0][0] = m_xform[0][0];
    N_xform.m_xform[0][1] = m_xform[1][0]; 
    N_xform.m_xform[0][2] = m_xform[2][0];
    N_xform.m_xform[0][3] = 0.0;

    N_xform.m_xform[1][0] = m_xform[0][1];
    N_xform.m_xform[1][1] = m_xform[1][1];
    N_xform.m_xform[1][2] = m_xform[2][1];
    N_xform.m_xform[1][3] = 0.0;

    N_xform.m_xform[2][0] = m_xform[0][2];
    N_xform.m_xform[2][1] = m_xform[1][2];
    N_xform.m_xform[2][2] = m_xform[2][2];
    N_xform.m_xform[2][3] = 0.0;

    N_xform.m_xform[3][0] = 0.0;
    N_xform.m_xform[3][1] = 0.0;
    N_xform.m_xform[3][2] = 0.0;
    N_xform.m_xform[3][3] = 1.0;
  }
  else
  {
    P_xform.Identity();
    N_xform.Identity();
    d = 0.0;
  }
  return d;
}


void ON_Xform::Rotation( 
        double angle,
        ON_3dVector axis,  // 3d nonzero axis of rotation
        ON_3dPoint center  // 3d center of rotation
        )
{
  Rotation( sin(angle), cos(angle), axis, center );
}

void ON_Xform::Rotation(
  ON_3dVector start_dir,
  ON_3dVector end_dir,
  ON_3dPoint rotation_center
  )
{
  if ( fabs(start_dir.Length()-1.0) > ON_SQRT_EPSILON )
    start_dir.Unitize();
  if ( fabs(end_dir.Length()-1.0) > ON_SQRT_EPSILON )
    end_dir.Unitize();
  double cos_angle = start_dir*end_dir;
  ON_3dVector axis = ON_CrossProduct(start_dir,end_dir);
  double sin_angle = axis.Length();
  if ( 0.0 == sin_angle || !axis.Unitize() )
  {
    axis.PerpendicularTo(start_dir);
    axis.Unitize();
    sin_angle = 0.0;
    cos_angle = (cos_angle < 0.0) ? -1.0 : 1.0;
  }
  Rotation(sin_angle,cos_angle,axis,rotation_center);
}

void ON_Xform::Rotation(  
        double sin_angle,
        double cos_angle,
        ON_3dVector axis,
        ON_3dPoint center
        )
{
  Identity();

  for(;;)
  {
    // 29 June 2005 Dale Lear
    //     Kill noise in input
    if ( fabs(sin_angle) >= 1.0-ON_SQRT_EPSILON && fabs(cos_angle) <= ON_SQRT_EPSILON )
    {
      cos_angle = 0.0;
      sin_angle = (sin_angle < 0.0) ? -1.0 : 1.0; 
      break;
    }
    
    if ( fabs(cos_angle) >= 1.0-ON_SQRT_EPSILON && fabs(sin_angle) <= ON_SQRT_EPSILON )
    {
      cos_angle = (cos_angle < 0.0) ? -1.0 : 1.0; 
      sin_angle = 0.0;
      break;
    }
    
    if ( fabs(cos_angle*cos_angle + sin_angle*sin_angle - 1.0) > ON_SQRT_EPSILON )
    {
      ON_2dVector cs(cos_angle,sin_angle);
      if ( cs.Unitize() )
      {
        cos_angle = cs.x;
        sin_angle = cs.y;
        // no break here
      }
      else
      {
        ON_ERROR("sin_angle and cos_angle are both zero.");
        cos_angle = 1.0;
        sin_angle = 0.0;
        break;
      }
    }

    if ( fabs(cos_angle) > 1.0-ON_EPSILON || fabs(sin_angle) < ON_EPSILON )
    {
      cos_angle = (cos_angle < 0.0) ? -1.0 : 1.0; 
      sin_angle = 0.0;
      break;
    }

    if ( fabs(sin_angle) > 1.0-ON_EPSILON || fabs(cos_angle) < ON_EPSILON )
    {
      cos_angle = 0.0;
      sin_angle = (sin_angle < 0.0) ? -1.0 : 1.0; 
      break;
    }

    break;
  }

  if (sin_angle != 0.0 || cos_angle != 1.0) 
  {
    const double one_minus_cos_angle = 1.0 - cos_angle;
    ON_3dVector a = axis;
    if ( fabs(a.LengthSquared() - 1.0) >  ON_EPSILON )
      a.Unitize();

    m_xform[0][0] = a.x*a.x*one_minus_cos_angle + cos_angle;
    m_xform[0][1] = a.x*a.y*one_minus_cos_angle - a.z*sin_angle;
    m_xform[0][2] = a.x*a.z*one_minus_cos_angle + a.y*sin_angle;

    m_xform[1][0] = a.y*a.x*one_minus_cos_angle + a.z*sin_angle;
    m_xform[1][1] = a.y*a.y*one_minus_cos_angle + cos_angle;
    m_xform[1][2] = a.y*a.z*one_minus_cos_angle - a.x*sin_angle;

    m_xform[2][0] = a.z*a.x*one_minus_cos_angle - a.y*sin_angle;
    m_xform[2][1] = a.z*a.y*one_minus_cos_angle + a.x*sin_angle;
    m_xform[2][2] = a.z*a.z*one_minus_cos_angle + cos_angle;

    if ( center.x != 0.0 || center.y != 0.0 || center.z != 0.0 ) {
      m_xform[0][3] = -((m_xform[0][0]-1.0)*center.x + m_xform[0][1]*center.y + m_xform[0][2]*center.z);
      m_xform[1][3] = -(m_xform[1][0]*center.x + (m_xform[1][1]-1.0)*center.y + m_xform[1][2]*center.z);
      m_xform[2][3] = -(m_xform[2][0]*center.x + m_xform[2][1]*center.y + (m_xform[2][2]-1.0)*center.z);
    }

    m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0;
    m_xform[3][3] = 1.0;
  }
}


void ON_Xform::Rotation(
  const ON_3dVector&  X0, // initial frame X (X,Y,Z = right handed orthonormal frame)
  const ON_3dVector&  Y0, // initial frame Y
  const ON_3dVector&  Z0, // initial frame Z
  const ON_3dVector&  X1, // final frame X (X,Y,Z = another right handed orthonormal frame)
  const ON_3dVector&  Y1, // final frame Y
  const ON_3dVector&  Z1  // final frame Z
  )
{
  // transformation maps X0 to X1, Y0 to Y1, Z0 to Z1

  // F0 changes x0,y0,z0 to world X,Y,Z
  ON_Xform F0;
  F0[0][0] = X0.x; F0[0][1] = X0.y; F0[0][2] = X0.z;
  F0[1][0] = Y0.x; F0[1][1] = Y0.y; F0[1][2] = Y0.z;
  F0[2][0] = Z0.x; F0[2][1] = Z0.y; F0[2][2] = Z0.z;
  F0[3][3] = 1.0;

  // F1 changes world X,Y,Z to x1,y1,z1
  ON_Xform F1;
  F1[0][0] = X1.x; F1[0][1] = Y1.x; F1[0][2] = Z1.x;
  F1[1][0] = X1.y; F1[1][1] = Y1.y; F1[1][2] = Z1.y;
  F1[2][0] = X1.z; F1[2][1] = Y1.z; F1[2][2] = Z1.z;
  F1[3][3] = 1.0;

  *this = F1*F0;
}

void ON_Xform::Rotation( 
  const ON_Plane& plane0,
  const ON_Plane& plane1
  )
{
  Rotation( 
    plane0.origin, plane0.xaxis, plane0.yaxis, plane0.zaxis,
    plane1.origin, plane1.xaxis, plane1.yaxis, plane1.zaxis
    );
}


void ON_Xform::Rotation(   // (not strictly a rotation)
                            // transformation maps P0 to P1, P0+X0 to P1+X1, ...
  const ON_3dPoint&   P0,  // initial frame center
  const ON_3dVector&  X0, // initial frame X
  const ON_3dVector&  Y0, // initial frame Y
  const ON_3dVector&  Z0, // initial frame Z
  const ON_3dPoint&   P1,  // final frame center
  const ON_3dVector&  X1, // final frame X
  const ON_3dVector&  Y1, // final frame Y
  const ON_3dVector&  Z1  // final frame Z
  )
{
  // transformation maps P0 to P1, P0+X0 to P1+X1, ...

  // T0 translates point P0 to (0,0,0)
  ON_Xform T0;
  T0.Translation( -P0.x, -P0.y, -P0.z );

  ON_Xform R;
  R.Rotation(X0,Y0,Z0,X1,Y1,Z1);

  // T1 translates (0,0,0) to point o1
  ON_Xform T1;
  T1.Translation( P1 );

  *this = T1*R*T0;
}

void ON_Xform::Mirror(
  ON_3dPoint point_on_mirror_plane,
  ON_3dVector normal_to_mirror_plane
  )
{
  ON_3dPoint P = point_on_mirror_plane;
  ON_3dVector N = normal_to_mirror_plane;
  N.Unitize();
  ON_3dVector V = (2.0*(N.x*P.x + N.y*P.y + N.z*P.z))*N;
  m_xform[0][0] = 1 - 2.0*N.x*N.x;
  m_xform[0][1] = -2.0*N.x*N.y;
  m_xform[0][2] = -2.0*N.x*N.z;
  m_xform[0][3] = V.x;

  m_xform[1][0] = -2.0*N.y*N.x;
  m_xform[1][1] = 1.0 -2.0*N.y*N.y;
  m_xform[1][2] = -2.0*N.y*N.z;
  m_xform[1][3] = V.y;

  m_xform[2][0] = -2.0*N.z*N.x;
  m_xform[2][1] = -2.0*N.z*N.y;
  m_xform[2][2] = 1.0 -2.0*N.z*N.z;
  m_xform[2][3] = V.z;

  m_xform[3][0] = 0.0;
  m_xform[3][1] = 0.0;
  m_xform[3][2] = 0.0;
  m_xform[3][3] = 1.0;
}



bool ON_Xform::ChangeBasis( 
  // General: If you have points defined with respect to planes, this
  //          computes the transformation to change coordinates from
  //          one plane to another.  The predefined world plane
  //          ON_world_plane can be used as an argument.
  // Details: If P = plane0.Evaluate( a0,b0,c0 ) and
  //          {a1,b1,c1} = ChangeBasis(plane0,plane1)*ON_3dPoint(a0,b0,c0),
  //          then P = plane1.Evaluate( a1, b1, c1 )
  //          
  const ON_Plane& plane0, // initial plane
  const ON_Plane& plane1  // final plane
  )
{
  return ChangeBasis( 
    plane0.origin, plane0.xaxis, plane0.yaxis, plane0.zaxis,
    plane1.origin, plane1.xaxis, plane1.yaxis, plane1.zaxis
    );
}


bool ON_Xform::ChangeBasis(
  const ON_3dVector&  X0, // initial frame X (X,Y,Z = arbitrary basis)
  const ON_3dVector&  Y0, // initial frame Y
  const ON_3dVector&  Z0, // initial frame Z
  const ON_3dVector&  X1, // final frame X (X,Y,Z = arbitrary basis)
  const ON_3dVector&  Y1, // final frame Y
  const ON_3dVector&  Z1  // final frame Z
  )
{
  // Q = a0*X0 + b0*Y0 + c0*Z0 = a1*X1 + b1*Y1 + c1*Z1
  // then this transform will map the point (a0,b0,c0) to (a1,b1,c1)

  Zero();
  m_xform[3][3] = 1.0;
  double a,b,c,d;
  a = X1*Y1;
  b = X1*Z1;
  c = Y1*Z1;
  double R[3][6] = {{X1*X1,      a,      b,       X1*X0, X1*Y0, X1*Z0},
                    {    a,  Y1*Y1,      c,       Y1*X0, Y1*Y0, Y1*Z0},
                    {    b,      c,  Z1*Z1,       Z1*X0, Z1*Y0, Z1*Z0}};
  //double R[3][6] = {{X1*X1,      a,      b,       X0*X1, X0*Y1, X0*Z1},
  //                  {    a,  Y1*Y1,      c,       Y0*X1, Y0*Y1, Y0*Z1},
  //                  {    b,      c,  Z1*Z1,       Z0*X1, Z0*Y1, Z0*Z1}};

  // row reduce R
  int i0 = (R[0][0] >= R[1][1]) ? 0 : 1;
  if ( R[2][2] > R[i0][i0] )
    i0 = 2;
  int i1 = (i0+1)%3;
  int i2 = (i1+1)%3;
  if ( R[i0][i0] == 0.0 )
    return false;
  d = 1.0/R[i0][i0];
  R[i0][0] *= d;
  R[i0][1] *= d;
  R[i0][2] *= d;
  R[i0][3] *= d;
  R[i0][4] *= d;
  R[i0][5] *= d;
  R[i0][i0] = 1.0;
  if ( R[i1][i0] != 0.0 ) {
    d = -R[i1][i0];
    R[i1][0] += d*R[i0][0];
    R[i1][1] += d*R[i0][1];
    R[i1][2] += d*R[i0][2];
    R[i1][3] += d*R[i0][3];
    R[i1][4] += d*R[i0][4];
    R[i1][5] += d*R[i0][5];
    R[i1][i0] = 0.0;
  }
  if ( R[i2][i0] != 0.0 ) {
    d = -R[i2][i0];
    R[i2][0] += d*R[i0][0];
    R[i2][1] += d*R[i0][1];
    R[i2][2] += d*R[i0][2];
    R[i2][3] += d*R[i0][3];
    R[i2][4] += d*R[i0][4];
    R[i2][5] += d*R[i0][5];
    R[i2][i0] = 0.0;
  }

  if ( fabs(R[i1][i1]) < fabs(R[i2][i2]) ) {
    int i = i1; i1 = i2; i2 = i;
  }
  if ( R[i1][i1] == 0.0 )
    return false;
  d = 1.0/R[i1][i1];
  R[i1][0] *= d;
  R[i1][1] *= d;
  R[i1][2] *= d;
  R[i1][3] *= d;
  R[i1][4] *= d;
  R[i1][5] *= d;
  R[i1][i1] = 1.0;
  if ( R[i0][i1] != 0.0 ) {
    d = -R[i0][i1];
    R[i0][0] += d*R[i1][0];
    R[i0][1] += d*R[i1][1];
    R[i0][2] += d*R[i1][2];
    R[i0][3] += d*R[i1][3];
    R[i0][4] += d*R[i1][4];
    R[i0][5] += d*R[i1][5];
    R[i0][i1] = 0.0;
  }
  if ( R[i2][i1] != 0.0 ) {
    d = -R[i2][i1];
    R[i2][0] += d*R[i1][0];
    R[i2][1] += d*R[i1][1];
    R[i2][2] += d*R[i1][2];
    R[i2][3] += d*R[i1][3];
    R[i2][4] += d*R[i1][4];
    R[i2][5] += d*R[i1][5];
    R[i2][i1] = 0.0;
  }

  if ( R[i2][i2] == 0.0 )
    return false;
  d = 1.0/R[i2][i2];
  R[i2][0] *= d;
  R[i2][1] *= d;
  R[i2][2] *= d;
  R[i2][3] *= d;
  R[i2][4] *= d;
  R[i2][5] *= d;
  R[i2][i2] = 1.0;
  if ( R[i0][i2] != 0.0 ) {
    d = -R[i0][i2];
    R[i0][0] += d*R[i2][0];
    R[i0][1] += d*R[i2][1];
    R[i0][2] += d*R[i2][2];
    R[i0][3] += d*R[i2][3];
    R[i0][4] += d*R[i2][4];
    R[i0][5] += d*R[i2][5];
    R[i0][i2] = 0.0;
  }
  if ( R[i1][i2] != 0.0 ) {
    d = -R[i1][i2];
    R[i1][0] += d*R[i2][0];
    R[i1][1] += d*R[i2][1];
    R[i1][2] += d*R[i2][2];
    R[i1][3] += d*R[i2][3];
    R[i1][4] += d*R[i2][4];
    R[i1][5] += d*R[i2][5];
    R[i1][i2] = 0.0;
  }

  m_xform[0][0] = R[0][3];
  m_xform[0][1] = R[0][4];
  m_xform[0][2] = R[0][5];

  m_xform[1][0] = R[1][3];
  m_xform[1][1] = R[1][4];
  m_xform[1][2] = R[1][5];

  m_xform[2][0] = R[2][3];
  m_xform[2][1] = R[2][4];
  m_xform[2][2] = R[2][5];

  return true;
}

bool ON_Xform::ChangeBasis(
  const ON_3dPoint&   P0,  // initial frame center
  const ON_3dVector&  X0, // initial frame X (X,Y,Z = arbitrary basis)
  const ON_3dVector&  Y0, // initial frame Y
  const ON_3dVector&  Z0, // initial frame Z
  const ON_3dPoint&   P1,  // final frame center
  const ON_3dVector&  X1, // final frame X (X,Y,Z = arbitrary basis)
  const ON_3dVector&  Y1, // final frame Y
  const ON_3dVector&  Z1  // final frame Z
  )
{
  bool rc = false;
  // Q = P0 + a0*X0 + b0*Y0 + c0*Z0 = P1 + a1*X1 + b1*Y1 + c1*Z1
  // then this transform will map the point (a0,b0,c0) to (a1,b1,c1)

  ON_Xform F0(P0,X0,Y0,Z0);		// Frame 0

  // T1 translates by -P1
  ON_Xform T1;
  T1.Translation( -P1.x, -P1.y, -P1.z );
	
  ON_Xform CB;
  rc = CB.ChangeBasis(ON_xaxis, ON_yaxis, ON_zaxis,X1,Y1,Z1);

  *this = CB*T1*F0;
  return rc;
}

void ON_Xform::WorldToCamera( 
         const ON_3dPoint& cameraLocation,
         const ON_3dVector& cameraX,
         const ON_3dVector& cameraY,
         const ON_3dVector& cameraZ
         )
{
  // see comments in tl2_xform.h for details.
  /* compute world to camera coordinate xform */
  m_xform[0][0] = cameraX.x; m_xform[0][1] = cameraX.y; m_xform[0][2] = cameraX.z;
  m_xform[0][3] = -(cameraX.x*cameraLocation.x + cameraX.y*cameraLocation.y + cameraX.z*cameraLocation.z);
  m_xform[1][0] = cameraY.x; m_xform[1][1] = cameraY.y; m_xform[1][2] = cameraY.z;
  m_xform[1][3] = -(cameraY.x*cameraLocation.x + cameraY.y*cameraLocation.y + cameraY.z*cameraLocation.z);
  m_xform[2][0] = cameraZ.x; m_xform[2][1] = cameraZ.y; m_xform[2][2] = cameraZ.z;
  m_xform[2][3] = -(cameraZ.x*cameraLocation.x + cameraZ.y*cameraLocation.y + cameraZ.z*cameraLocation.z);
  m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; m_xform[3][3] = 1.0;
}
  
void ON_Xform::CameraToWorld(
         const ON_3dPoint& cameraLocation,
         const ON_3dVector& cameraX,
         const ON_3dVector& cameraY,
         const ON_3dVector& cameraZ
         )
{
  // see comments in tl2_xform.h for details.
  /* compute camera to world coordinate m_xform */
  m_xform[0][0] = cameraX.x; m_xform[0][1] = cameraY.x; m_xform[0][2] = cameraZ.x; 
  m_xform[0][3] = cameraLocation.x;
  m_xform[1][0] = cameraX.y; m_xform[1][1] = cameraY.y; m_xform[1][2] = cameraZ.y; 
  m_xform[1][3] = cameraLocation.y;
  m_xform[2][0] = cameraX.z; m_xform[2][1] = cameraY.z; m_xform[2][2] = cameraZ.z; 
  m_xform[2][3] = cameraLocation.z;
  m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; m_xform[3][3] = 1.0;
}

bool ON_Xform::CameraToClip(
      ON_BOOL32 bPerspective,
      double left,      double right,
      double bottom,    double top,
      double near_dist, double far_dist
      )
{
  double dd;

  if ( left == right || bottom == top || near_dist == far_dist )
    return false;

  if ( !bPerspective ) {
    // parallel projection
    //d = 1.0/(left-right);
    //m_xform[0][0] = -2.0*d; m_xform[0][3] = (left+right)*d; m_xform[0][1] = m_xform[0][2] = 0.0;
    //d = 1.0/(bottom-top);
    //m_xform[1][1] = -2.0*d; m_xform[1][3] = (bottom+top)*d; m_xform[1][0] = m_xform[1][2] = 0.0;
    //d = 1.0/(far_dist-near_dist);
    //m_xform[2][2] = 2.0*d;  m_xform[2][3] = (far_dist+near_dist)*d;   m_xform[2][0] = m_xform[2][1] = 0.0;
    //m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; m_xform[3][3] = 1.0;

    dd = (left-right);
    m_xform[0][0] = -2.0/dd; m_xform[0][3] = (left+right)/dd; m_xform[0][1] = m_xform[0][2] = 0.0;
    dd = (bottom-top);
    m_xform[1][1] = -2.0/dd; m_xform[1][3] = (bottom+top)/dd; m_xform[1][0] = m_xform[1][2] = 0.0;
    dd = (far_dist-near_dist);
    m_xform[2][2] = 2.0/dd;  m_xform[2][3] = (far_dist+near_dist)/dd;   m_xform[2][0] = m_xform[2][1] = 0.0;
    m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; m_xform[3][3] = 1.0;
  }
  else 
  {
    // perspective projection

    //  2n/(r-l)     0        (r+l)/(r-l)     0
    //    0        2n/(t-b)   (t+b)/(t-b)     0
    //    0          0        (f+n)/(f-n)  2fn/(f-n)
    //    0          0            -1          0
    //
    // To get a linear map from camera z to clip z, apply the linear
    // fractional transformation that maps [-1,1] -> [-1,1]
    //
    //   f(s): s -> (a*s + b)/(a + bs),
    //
    //  where a = (n+f) and b = (f-n), to clip z.
    //
    // The inverse of f is g
    //
    //   g(t): t -> (a*t - b)/(a - b*t)
    //
    // to the z coordinate after applying this transformation
    //d = 1.0/(right-left);
    //m_xform[0][0] = 2.0*near_dist*d; 
    //m_xform[0][2] = (right+left)*d; 
    //m_xform[0][1] = m_xform[0][3] = 0.0;

    //d = 1.0/(top-bottom);
    //m_xform[1][1] = 2.0*near_dist*d; 
    //m_xform[1][2] = (top+bottom)*d; 
    //m_xform[1][0] = m_xform[1][3] = 0.0;

    //d = 1.0/(far_dist-near_dist);
    //m_xform[2][2] = (far_dist+near_dist)*d; 
    //m_xform[2][3] = 2.0*near_dist*far_dist*d; 
    //m_xform[2][0] = m_xform[2][1] = 0.0;

    dd = (right-left);
    m_xform[0][0] = 2.0*near_dist/dd; 
    m_xform[0][2] = (right+left)/dd; 
    m_xform[0][1] = m_xform[0][3] = 0.0;

    dd = (top-bottom);
    m_xform[1][1] = 2.0*near_dist/dd; 
    m_xform[1][2] = (top+bottom)/dd; 
    m_xform[1][0] = m_xform[1][3] = 0.0;

    dd = (far_dist-near_dist);
    m_xform[2][2] = (far_dist+near_dist)/dd; 
    m_xform[2][3] = 2.0*near_dist*far_dist/dd; 
    m_xform[2][0] = m_xform[2][1] = 0.0;

    m_xform[3][0] = m_xform[3][1] = m_xform[3][3] = 0.0; m_xform[3][2] = -1.0;
  }
  return true;
}

bool ON_Xform::ClipToCamera(
      ON_BOOL32 bPerspective,
      double left,      double right,
      double bottom,    double top,
      double near_dist, double far_dist
      )
{
  // see comments in tl2_xform.h for details.
  double dd;
  if ( left == right || bottom == top || near_dist == far_dist )
    return false;

  if ( !bPerspective ) {
    // parallel projection
    m_xform[0][0] = 0.5*(right-left); m_xform[0][3] = 0.5*(right+left); m_xform[0][1] = m_xform[0][2] = 0.0;
    m_xform[1][1] = 0.5*(top-bottom); m_xform[1][3] = 0.5*(top+bottom); m_xform[1][0] = m_xform[1][2] = 0.0;
    m_xform[2][2] = 0.5*(far_dist-near_dist);   m_xform[2][3] = -0.5*(far_dist+near_dist);  m_xform[2][0] = m_xform[2][1] = 0.0;
    m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; m_xform[3][3] = 1.0;
  }
  else {
    // perspective projection
    //  (r-l)/(2n)       0            0       (r+l)/(2n)
    //    0         (t-b)/(2n)        0       (t+b)/(2n)
    //    0              0            0           -1
    //    0              0       (f-n)/(2fn)  (f+n)/(2fn)
    //d = 0.5/near_dist;
    //m_xform[0][0] = d*(right-left); 
    //m_xform[0][3] = d*(right+left); 
    //m_xform[0][1] = m_xform[0][2] = 0.0;

    //m_xform[1][1] = d*(top-bottom); 
    //m_xform[1][3] = d*(top+bottom); 
    //m_xform[1][0] = m_xform[1][2] = 0.0;

    //m_xform[2][0] = m_xform[2][1] = m_xform[2][2] = 0.0; m_xform[2][3] = -1.0;

    //d /= far_dist;
    //m_xform[3][2] = d*(far_dist-near_dist); 
    //m_xform[3][3] = d*(far_dist+near_dist); 
    //m_xform[3][0] = m_xform[3][1] = 0.0;

    dd = 2.0*near_dist;
    m_xform[0][0] = (right-left)/dd; 
    m_xform[0][3] = (right+left)/dd; 
    m_xform[0][1] = m_xform[0][2] = 0.0;

    m_xform[1][1] = (top-bottom)/dd; 
    m_xform[1][3] = (top+bottom)/dd; 
    m_xform[1][0] = m_xform[1][2] = 0.0;

    m_xform[2][0] = m_xform[2][1] = m_xform[2][2] = 0.0; m_xform[2][3] = -1.0;

    dd *= far_dist;
    m_xform[3][2] = (far_dist-near_dist)/dd; 
    m_xform[3][3] = (far_dist+near_dist)/dd; 
    m_xform[3][0] = m_xform[3][1] = 0.0;
  }

  return true;
}

bool ON_Xform::ClipToScreen(
      double left,   double right,
      double bottom, double top,
      double near_z, double far_z
      )
{
  // see comments in tl2_xform.h for details.
  if ( left == right || bottom == top )
    return false;

  m_xform[0][0] = 0.5*(right-left);
  m_xform[0][3] = 0.5*(right+left);
  m_xform[0][1] = m_xform[0][2] = 0.0;

  m_xform[1][1] = 0.5*(top-bottom);
  m_xform[1][3] = 0.5*(top+bottom);
  m_xform[1][0] = m_xform[1][2] = 0.0;

  if (far_z != near_z) {
    m_xform[2][2] = 0.5*(near_z-far_z);
    m_xform[2][3] = 0.5*(near_z+far_z);
  }
  else {
    m_xform[2][2] = 1.0;
    m_xform[2][3] = 0.0;
  }
  m_xform[2][0] = m_xform[2][1] = 0.0;

  m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; 
  m_xform[3][3] = 1.0;

  return true;
}

bool ON_Xform::ScreenToClip(
      double left,   double right,
      double bottom, double top,
      double near_z, double far_z
      )
{
  // see comments in tl2_xform.h for details.
  ON_Xform c2s;
  bool rc = c2s.ClipToScreen( left, right, bottom, top, near_z, far_z );
  if (rc) {
    m_xform[0][0] = 1.0/c2s[0][0]; m_xform[0][3] = -c2s[0][3]/c2s[0][0];
    m_xform[0][1] = m_xform[0][2] = 0.0;

    m_xform[1][1] = 1.0/c2s[1][1]; m_xform[1][3] = -c2s[1][3]/c2s[1][1];
    m_xform[1][0] = m_xform[1][2] = 0.0;

    m_xform[2][2] = 1.0/c2s[2][2]; m_xform[2][3] = -c2s[2][3]/c2s[2][2];
    m_xform[2][0] = m_xform[2][1] = 0.0;

    m_xform[3][0] = m_xform[3][1] = m_xform[3][2] = 0.0; 
    m_xform[3][3] = 1.0;
  }
  return rc;
}


int ON_Xform::ClipFlag4d( const double* point ) const
{
  if ( !point )
    return 1|2|4|8|16|32;
  int clip = 0;
  double x = m_xform[0][0]*point[0] + m_xform[0][1]*point[1] + m_xform[0][2]*point[2] + m_xform[0][3]*point[3];
  double y = m_xform[1][0]*point[0] + m_xform[1][1]*point[1] + m_xform[1][2]*point[2] + m_xform[1][3]*point[3];
  double z = m_xform[2][0]*point[0] + m_xform[2][1]*point[1] + m_xform[2][2]*point[2] + m_xform[2][3]*point[3];
  double w = m_xform[3][0]*point[0] + m_xform[3][1]*point[1] + m_xform[3][2]*point[2] + m_xform[3][3]*point[3];
  if ( point[3] < 0.0 ) {
    x = -x; y = -y; z = -z; w = -w;
  }
  if ( x <= -w )
    clip |= 1;
  else if ( x >= w )
    clip |= 2;
  if ( y <= -w )
    clip |= 4;
  else if ( y >= w )
    clip |= 8;
  if ( z <= -w )
    clip |= 16;
  else if ( z >= w )
    clip |= 32;
  return clip;
}

int ON_Xform::ClipFlag3d( const double* point ) const
{
  if ( !point )
    return 1|2|4|8|16|32;
  int clip = 0;
  const double x = m_xform[0][0]*point[0] + m_xform[0][1]*point[1] + m_xform[0][2]*point[2] + m_xform[0][3];
  const double y = m_xform[1][0]*point[0] + m_xform[1][1]*point[1] + m_xform[1][2]*point[2] + m_xform[1][3];
  const double z = m_xform[2][0]*point[0] + m_xform[2][1]*point[1] + m_xform[2][2]*point[2] + m_xform[2][3];
  const double w = m_xform[3][0]*point[0] + m_xform[3][1]*point[1] + m_xform[3][2]*point[2] + m_xform[3][3];
  if ( x <= -w )
    clip |= 1;
  else if ( x >= w )
    clip |= 2;
  if ( y <= -w )
    clip |= 4;
  else if ( y >= w )
    clip |= 8;
  if ( z <= -w )
    clip |= 16;
  else if ( z >= w )
    clip |= 32;
  return clip;
}

int ON_Xform::ClipFlag4d( int count, int stride, const double* point, 
                            ON_BOOL32 bTestZ ) const
{
  int clip = 1|2|4|8;
  if ( bTestZ)
    clip |= (16|32);
  if ( point && ((count > 0 && stride >= 4) || count == 1) ) {
    for ( /*empty*/; clip && count--; point += stride ) {
      clip &= ClipFlag4d( point );
    }
  }
  return clip;
}

int ON_Xform::ClipFlag3d( int count, int stride, const double* point, 
                            ON_BOOL32 bTestZ ) const
{
  int clip = 1|2|4|8;
  if ( bTestZ)
    clip |= (16|32);
  if ( point && ((count > 0 && stride >= 3) || count == 1) ) {
    for ( /*empty*/; clip && count--; point += stride ) {
      clip &= ClipFlag3d( point );
    }
  }
  return clip;
}

int ON_Xform::ClipFlag3dBox( const double* boxmin, const double* boxmax ) const
{
  int clip = 1|2|4|8|16|32;
  double point[3];
  int i,j,k;
  if ( boxmin && boxmax ) {
    for (i=0;i<2;i++) {
      point[0] = (i)?boxmax[0]:boxmin[0];
      for (j=0;j<2;j++) {
        point[1] = (j)?boxmax[1]:boxmin[1];
        for (k=0;k<2;k++) {
          point[2] = (k)?boxmax[2]:boxmin[2];
          clip &= ClipFlag3d(point);
          if ( !clip )
            return 0;
        }
      }
    }
  }
  return clip;
}

ON_Xform& ON_Xform::operator=(const ON_Matrix& src)
{
  int i,j;
  i = src.RowCount();
  const int maxi = (i>4)?4:i;
  j = src.ColCount();
  const int maxj = (j>4)?4:j;
  Identity();
  for ( i = 0; i < maxi; i++ ) for ( j = 0; j < maxj; j++ ) {
    m_xform[i][j] = src.m[i][j];
  }
  return *this;
}

bool ON_Xform::IntervalChange(
  int dir,
  ON_Interval old_interval,
  ON_Interval new_interval
  )
{
  bool rc = false;
  Identity();
  if (   dir >= 0 
       && dir <= 3 
       && old_interval[0] != ON_UNSET_VALUE
       && old_interval[1] != ON_UNSET_VALUE
       && new_interval[0] != ON_UNSET_VALUE
       && new_interval[1] != ON_UNSET_VALUE
       && old_interval.Length() != 0.0
       )
  {
    rc = true;
    if ( new_interval != old_interval )
    {
      double s = new_interval.Length()/old_interval.Length();;
      double d = (new_interval[0]*old_interval[1] - new_interval[1]*old_interval[0])/old_interval.Length();
      m_xform[dir][dir] = s;
      m_xform[dir][3] = d;
    }
  }
  return rc;
}

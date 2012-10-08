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

// 8 July 2003 Dale Lear
//    changed ON_Matrix to use multiple allocations
//    for coefficient memory in large matrices.

// ON_Matrix.m_cmem points to a struct DBLBLK
struct DBLBLK
{
  int count;
  double* a;
  struct DBLBLK *next;
};

double const * const * ON_Matrix::ThisM() const
{
  // When the "expert" constructor Create(row_count,col_count,user_memory,...)
  // is used, m_rowmem[] is empty and m = user_memory;
  // When any other constructor is used, m_rowmem[] is the 0-based
  // row memory.
  return (m_row_count == m_rowmem.Count()) ? m_rowmem.Array() : m;
}

double * * ON_Matrix::ThisM()
{
  // When the "expert" constructor Create(row_count,col_count,user_memory,...)
  // is used, m_rowmem[] is empty and m = user_memory;
  // When any other constructor is used, m_rowmem[] is the 0-based
  // row memory.
  return (m_row_count == m_rowmem.Count()) ? m_rowmem.Array() : m;
}


double* ON_Matrix::operator[](int i)
{
  return m[i];
}

const double* ON_Matrix::operator[](int i) const
{
  return m[i];
}

ON_Matrix::ON_Matrix()
: m(0)
, m_row_count(0)
, m_col_count(0)
, m_Mmem(0)
, m_row_offset(0)
, m_col_offset(0)
, m_cmem(0)
{
}

ON_Matrix::ON_Matrix( int row_size, int col_size ) 
: m(0)
, m_row_count(0)
, m_col_count(0)
, m_Mmem(0)
, m_row_offset(0)
, m_col_offset(0)
, m_cmem(0)
{
  Create(row_size,col_size);
}

ON_Matrix::ON_Matrix( int row0, int row1, int col0, int col1 ) 
: m(0)
, m_row_count(0)
, m_col_count(0)
, m_Mmem(0)
, m_row_offset(0)
, m_col_offset(0)
, m_cmem(0)
{
  Create(row0,row1,col0,col1);
}

ON_Matrix::ON_Matrix( const ON_Xform& x ) 
: m(0)
, m_row_count(0)
, m_col_count(0)
, m_Mmem(0)
, m_row_offset(0)
, m_col_offset(0)
, m_cmem(0)
{
  *this = x;
}

ON_Matrix::ON_Matrix( const ON_Matrix& src )
: m(0)
, m_row_count(0)
, m_col_count(0)
, m_Mmem(0)
, m_row_offset(0)
, m_col_offset(0)
, m_cmem(0)
{
  *this = src;
}

ON_Matrix::ON_Matrix(
  int row_count,
  int col_count,
  double** M,
  bool bDestructorFreeM
  )
: m(0)
, m_row_count(0)
, m_col_count(0)
, m_Mmem(0)
, m_row_offset(0)
, m_col_offset(0)
, m_cmem(0)
{
  Create(row_count,col_count,M,bDestructorFreeM);
}

ON_Matrix::~ON_Matrix()
{
  if ( 0 != m_Mmem )
  {
    onfree(m_Mmem);
    m_Mmem = 0;
  }
  m_row_offset = 0;
  m_col_offset = 0;
  struct DBLBLK* p = (struct DBLBLK*)m_cmem;
  m_cmem = 0;
  while(0 != p)
  {
    struct DBLBLK* next = p->next;
    onfree(p);
    p = next;
  }
}

int ON_Matrix::RowCount() const
{
  return m_row_count;
}

int ON_Matrix::ColCount() const
{
  return m_col_count;
}

int ON_Matrix::MinCount() const
{
  return (m_row_count <= m_col_count) ? m_row_count : m_col_count;
}

int ON_Matrix::MaxCount() const
{
  return (m_row_count >= m_col_count) ? m_row_count : m_col_count;
}

bool ON_Matrix::Create( int row_count, int col_count)
{
  bool b = false;
  Destroy();
  if ( row_count > 0 && col_count > 0 ) 
  {
    m_rowmem.Reserve(row_count);
    if ( 0 != m_rowmem.Array() )
    {
      m_rowmem.SetCount(row_count);
      // In general, allocate coefficient memory in chunks 
      // of <= max_dblblk_size bytes.  The value of max_dblblk_size
      // is tuned to maximize speed on calculations involving
      // large matrices.  If all of the coefficients will fit
      // into a chunk of memory <= 1.1*max_dblblk_size, then 
      // a single chunk is allocated.
      
      // In limited testing, these two values appeared to work ok.
      // The latter was a hair faster in solving large row reduction
      // problems (for reasons I do not understand).
      //const int max_dblblk_size = 1024*1024*8;
      const int max_dblblk_size = 512*1024;

      int rows_per_block = max_dblblk_size/(col_count*sizeof(double));
      if ( rows_per_block > row_count )
        rows_per_block = row_count;
      else if ( rows_per_block < 1 )
        rows_per_block = 1;
      else if ( rows_per_block < row_count && 11*rows_per_block >= 10*row_count )
        rows_per_block = row_count;

      int j, i = row_count; 
      m = m_rowmem.Array();
      double** row = m;
      for ( i = row_count; i > 0; i -= rows_per_block )
      {
        if ( i < rows_per_block )
          rows_per_block = i;
        int dblblk_count = rows_per_block*col_count;
        struct DBLBLK* p = (struct DBLBLK*)onmalloc(sizeof(*p) + dblblk_count*sizeof(p->a[0]));
        p->a = (double*)(p+1);
        p->count = dblblk_count;
        p->next = (struct DBLBLK*)m_cmem;
        m_cmem = p;
        *row = p->a;
        j = rows_per_block-1;
        while(j--)
        {
          row[1] = row[0] + col_count;
          row++;
        }
        row++;
      }
      m_row_count = row_count;
      m_col_count = col_count;
      b = true;
    }
  }
  return b;
}

bool ON_Matrix::Create( // E.g., Create(1,5,1,7) creates a 5x7 sized matrix that with
             // "top" row = m[1][1],...,m[1][7] and "bottom" row
             // = m[5][1],...,m[5][7].  The result of Create(0,m,0,n) is
             // identical to the result of Create(m+1,n+1).
   int ri0, // first valid row index
   int ri1, // last valid row index
   int ci0, // first valid column index
   int ci1 // last valid column index
   )
{
  bool b = false;
  if ( ri1 > ri0 && ci1 > ci0 ) 
  {
    // juggle m[] pointers so that m[ri0+i][ci0+j] = m_row[i][j];
    b = Create( ri1-ri0, ci1-ci0 );
    if (b)
    {
      m_row_offset = ri0; // this is the only line of code where m_row_offset should be set to a non-zero value
      m_col_offset = ci0; // this is the only line of code where m_col_offset should be set to a non-zero value
      if ( ci0 != 0 )
      {
        int i;
        for ( i = 0; i < m_row_count; i++ ) {
          m[i] -= ci0;
        }
      }
      if ( ri0 != 0 )
        m -= ri0;
    }
  }
  return b;
}

bool ON_Matrix::Create(
  int row_count,
  int col_count,
  double** M,
  bool bDestructorFreeM
  )
{
  Destroy();
  if ( row_count < 1 || col_count < 1 || 0 == M )
    return false;
  m = M;
  m_row_count = row_count;
  m_col_count = col_count;
  if ( bDestructorFreeM )
    m_Mmem = M;
  return true;
}


void ON_Matrix::Destroy()
{
  m = 0;
  m_row_count = 0;
  m_col_count = 0;
  m_rowmem.SetCount(0);
  if ( 0 != m_Mmem )
  {
    // pointer passed to Create( row_count, col_count, M, bDestructorFreeM )
    // when bDestructorFreeM = true.
    onfree(m_Mmem);
    m_Mmem = 0;
  }
	m_row_offset = 0;
	m_col_offset = 0;
  struct DBLBLK* cmem = (struct DBLBLK*)m_cmem;
  m_cmem = 0;
  while( 0 != cmem )
  {
    struct DBLBLK* next_cmem = cmem->next;
    onfree(cmem);
    cmem = next_cmem;
  }
}

void ON_Matrix::EmergencyDestroy()
{
  // call if memory pool used matrix by becomes invalid
  m = 0;
  m_row_count = 0;
  m_col_count = 0;
  m_rowmem.EmergencyDestroy();
	m_Mmem = 0;
	m_row_offset = 0;
	m_col_offset = 0;
  m_cmem = 0;
}

ON_Matrix& ON_Matrix::operator=(const ON_Matrix& src)
{
  if ( this != &src ) 
  {
    if ( src.m_row_count != m_row_count || src.m_col_count != m_col_count || 0 == m )
    {
      Destroy();
      Create( src.RowCount(), src.ColCount() );
    }
    if (src.m_row_count == m_row_count && src.m_col_count == m_col_count && 0 != m )
    {
      int i;
      // src rows may be permuted - copy row by row
      double** m_dest = ThisM();
      double const*const* m_src = src.ThisM();
      const int sizeof_row = m_col_count*sizeof(m_dest[0][0]);
      for ( i = 0; i < m_row_count; i++ ) 
      {
        memcpy( m_dest[i], m_src[i], sizeof_row );
      }
      m_row_offset = src.m_row_offset;
      m_col_offset = src.m_col_offset;
    }
  }
  return *this;
}

ON_Matrix& ON_Matrix::operator=(const ON_Xform& src)
{
  m_row_offset = 0;
  m_col_offset = 0;
  if ( 4 != m_row_count || 4 != m_col_count || 0 == m )
  {
    Destroy();
    Create( 4, 4 );
  }
  if ( 4 == m_row_count && 4 == m_col_count && 0 != m )
  {
    double** this_m = ThisM();
    if ( this_m )
    {
      memcpy( this_m[0], &src.m_xform[0][0], 4*sizeof(this_m[0][0]) );
      memcpy( this_m[1], &src.m_xform[1][0], 4*sizeof(this_m[0][0]) );
      memcpy( this_m[2], &src.m_xform[2][0], 4*sizeof(this_m[0][0]) );
      memcpy( this_m[3], &src.m_xform[3][0], 4*sizeof(this_m[0][0]) );
    }
  }
  return *this;
}

bool ON_Matrix::Transpose()
{
  bool rc = false;
  int i, j;
  double t;
  const int row_count = RowCount();
  const int col_count = ColCount();
  if ( row_count > 0 && col_count > 0 ) 
  {
    double** this_m = ThisM();
    if ( row_count == col_count )
    {
      rc = true;
      for ( i = 0; i < row_count; i++ ) for ( j = i+1; j < row_count; j++ )
      {
        t = this_m[i][j]; this_m[i][j] = this_m[j][i]; this_m[j][i] = t;
      }
    }
    else if ( this_m == m_rowmem.Array() )
    {
      ON_Matrix A(*this);
      rc = Create(col_count,row_count) 
           && m_row_count == A.ColCount()
           && m_col_count == A.RowCount();
      if (rc)
      {
        double const*const* Am = A.ThisM();
  		  this_m = ThisM(); // Create allocates new memory
        for ( i = 0; i < row_count; i++ ) for ( j = 0; j < col_count; j++ ) 
        {
          this_m[j][i] = Am[i][j];
        }
        m_row_offset = A.m_col_offset;
        m_col_offset = A.m_row_offset;
      }
      else
      {
        // attempt to put values back
        *this = A;
      }
    }
  }
  return rc;
}

bool ON_Matrix::SwapRows( int row0, int row1 )
{
  bool b = false;
  double** this_m = ThisM();
  row0 -= m_row_offset;
  row1 -= m_row_offset;
  if ( this_m && 0 <= row0 && row0 < m_row_count && 0 <= row1 && row1 < m_row_count )
  {
    if ( row0 != row1 )
    {
      double* tmp = this_m[row0]; this_m[row0] = this_m[row1]; this_m[row1] = tmp;
    }
    b = true;
  }
  return b;
}

bool ON_Matrix::SwapCols( int col0, int col1 )
{
  bool b = false;
  int i;
  double t;
  double** this_m = ThisM();
  col0 -= m_col_offset;
  col1 -= m_col_offset;
  if ( this_m && 0 <= col0 && col0 < m_col_count && 0 <= col1 && col1 < m_col_count ) 
  {
    if ( col0 != col1 ) 
    {
      for ( i = 0; i < m_row_count; i++ )
      {
        t = this_m[i][col0]; this_m[i][col0] = this_m[i][col1]; this_m[i][col1] = t;
      }
    }
    b = true;
  }
  return b;
}

void ON_Matrix::RowScale( int dest_row, double s )
{
  double** this_m = ThisM();
  dest_row -= m_row_offset;
  ON_ArrayScale( m_col_count, s, this_m[dest_row], this_m[dest_row] );
}

void ON_Matrix::RowOp( int dest_row, double s, int src_row )
{
  double** this_m = ThisM();
  dest_row -= m_row_offset;
  src_row -= m_row_offset;
  ON_Array_aA_plus_B( m_col_count, s, this_m[src_row], this_m[dest_row], this_m[dest_row] );
}

void ON_Matrix::ColScale( int dest_col, double s )
{
  int i;
  double** this_m = ThisM();
  dest_col -= m_col_offset;
  for ( i = 0; i < m_row_count; i++ ) 
  {
    this_m[i][dest_col] *= s;
  }
}

void ON_Matrix::ColOp( int dest_col, double s, int src_col )
{
  int i;
  double** this_m = ThisM();
  dest_col -= m_col_offset;
  src_col  -= m_col_offset;
  for ( i = 0; i < m_row_count; i++ ) 
  {
    this_m[i][dest_col] += s*this_m[i][src_col];
  }
}

int
ON_Matrix::RowReduce( 
    double zero_tolerance,
    double& determinant,
    double& pivot 
    )
{
  double x, piv, det;
  int i, k, ix, rank;

  double** this_m = ThisM();
  piv = det = 1.0;
  rank = 0;
  const int n = m_row_count <= m_col_count ? m_row_count : m_col_count;
  for ( k = 0; k < n; k++ ) {
    ix = k;
    x = fabs(this_m[ix][k]);
    for ( i = k+1; i < m_row_count; i++ ) {
      if ( fabs(this_m[i][k]) > x ) {
        ix = i;
        x = fabs(this_m[ix][k]);
      }
    }
    if ( x < piv || k == 0 ) {
      piv = x;
    }
    if ( x <= zero_tolerance ) {
      det = 0.0;
      break;
    }
    rank++;

    // swap rows
    SwapRows( ix, k );
    det = -det;

    // scale row k of matrix and B
    det *= this_m[k][k];
    x = 1.0/this_m[k][k];
    this_m[k][k] = 1.0;
    ON_ArrayScale( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[k][k+1] );

    // zero column k for rows below this_m[k][k]
    for ( i = k+1; i < m_row_count; i++ ) {
      x = -this_m[i][k];
      this_m[i][k] = 0.0;
      if ( fabs(x) > zero_tolerance ) {
        ON_Array_aA_plus_B( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[i][k+1], &this_m[i][k+1] );
      }
    }
  }

  pivot = piv;
  determinant = det;

  return rank;
}

int
ON_Matrix::RowReduce( 
    double zero_tolerance,
    double* B,
    double* pivot 
    )
{
  double t;
  double x, piv;
  int i, k, ix, rank;

  double** this_m = ThisM();
  piv = 0.0;
  rank = 0;
  const int n = m_row_count <= m_col_count ? m_row_count : m_col_count;
  for ( k = 0; k < n; k++ ) {
    ix = k;
    x = fabs(this_m[ix][k]);
    for ( i = k+1; i < m_row_count; i++ ) {
      if ( fabs(this_m[i][k]) > x ) {
        ix = i;
        x = fabs(this_m[ix][k]);
      }
    }
    if ( x < piv || k == 0 ) {
      piv = x;
    }
    if ( x <= zero_tolerance )
      break;
    rank++;

    // swap rows of matrix and B
    SwapRows( ix, k );
    t = B[ix]; B[ix] = B[k]; B[k] = t;

    // scale row k of matrix and B
    x = 1.0/this_m[k][k];
    this_m[k][k] = 1.0;
    ON_ArrayScale( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[k][k+1] );
    B[k] *= x;

    // zero column k for rows below this_m[k][k]
    for ( i = k+1; i < m_row_count; i++ ) {
      x = -this_m[i][k];
      this_m[i][k] = 0.0;
      if ( fabs(x) > zero_tolerance ) {
        ON_Array_aA_plus_B( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[i][k+1], &this_m[i][k+1] );
        B[i] += x*B[k];
      }
    }
  }

  if ( pivot )
    *pivot = piv;

  return rank;
}

int
ON_Matrix::RowReduce( 
    double zero_tolerance,
    ON_3dPoint* B,
    double* pivot 
    )
{
  ON_3dPoint t;
  double x, piv;
  int i, k, ix, rank;

  double** this_m = ThisM();
  piv = 0.0;
  rank = 0;
  const int n = m_row_count <= m_col_count ? m_row_count : m_col_count;
  for ( k = 0; k < n; k++ ) {
    //onfree( onmalloc( 1)); // 8-06-03 lw for cancel thread responsiveness
    onmalloc( 0); // 9-4-03 lw changed to 0
    ix = k;
    x = fabs(this_m[ix][k]);
    for ( i = k+1; i < m_row_count; i++ ) {
      if ( fabs(this_m[i][k]) > x ) {
        ix = i;
        x = fabs(this_m[ix][k]);
      }
    }
    if ( x < piv || k == 0 ) {
      piv = x;
    }
    if ( x <= zero_tolerance )
      break;
    rank++;

    // swap rows of matrix and B
    SwapRows( ix, k );
    t = B[ix]; B[ix] = B[k]; B[k] = t;

    // scale row k of matrix and B
    x = 1.0/this_m[k][k];
    this_m[k][k] = 1.0;
    ON_ArrayScale( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[k][k+1] );
    B[k] *= x;

    // zero column k for rows below this_m[k][k]
    for ( i = k+1; i < m_row_count; i++ ) {
      x = -this_m[i][k];
      this_m[i][k] = 0.0;
      if ( fabs(x) > zero_tolerance ) {
        ON_Array_aA_plus_B( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[i][k+1], &this_m[i][k+1] );
        B[i] += x*B[k];
      }
    }
  }

  if ( pivot )
    *pivot = piv;

  return rank;
}

int
ON_Matrix::RowReduce( 
    double zero_tolerance,
    int pt_dim, int pt_stride, double* pt,
    double* pivot 
    )
{
  const int sizeof_pt = pt_dim*sizeof(pt[0]);
  double* tmp_pt = (double*)onmalloc(pt_dim*sizeof(tmp_pt[0]));
  double *ptA, *ptB;
  double x, piv;
  int i, k, ix, rank, pti;

  double** this_m = ThisM();
  piv = 0.0;
  rank = 0;
  const int n = m_row_count <= m_col_count ? m_row_count : m_col_count;
  for ( k = 0; k < n; k++ ) {
//    onfree( onmalloc( 1)); //  8-06-03 lw for cancel thread responsiveness
    onmalloc( 0); // 9-4-03 lw changed to 0
    ix = k;
    x = fabs(this_m[ix][k]);
    for ( i = k+1; i < m_row_count; i++ ) {
      if ( fabs(this_m[i][k]) > x ) {
        ix = i;
        x = fabs(this_m[ix][k]);
      }
    }
    if ( x < piv || k == 0 ) {
      piv = x;
    }
    if ( x <= zero_tolerance )
      break;
    rank++;

    // swap rows of matrix and B
    if ( ix != k ) {
      SwapRows( ix, k );
      ptA = pt + (ix*pt_stride);
      ptB = pt + (k*pt_stride);
      memcpy( tmp_pt, ptA, sizeof_pt );
      memcpy( ptA, ptB, sizeof_pt );
      memcpy( ptB, tmp_pt, sizeof_pt );
    }

    // scale row k of matrix and B
    x = 1.0/this_m[k][k];
    if ( x != 1.0 ) {
      this_m[k][k] = 1.0;
      ON_ArrayScale( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[k][k+1] );
      ptA = pt + (k*pt_stride);
      for ( pti = 0; pti < pt_dim; pti++ )
        ptA[pti] *= x;
    }

    // zero column k for rows below this_m[k][k]
    ptB = pt + (k*pt_stride);
    for ( i = k+1; i < m_row_count; i++ ) {
      x = -this_m[i][k];
      this_m[i][k] = 0.0;
      if ( fabs(x) > zero_tolerance ) {
        ON_Array_aA_plus_B( m_col_count - 1 - k, x, &this_m[k][k+1], &this_m[i][k+1], &this_m[i][k+1] );
        ptA = pt + (i*pt_stride);
        for ( pti = 0; pti < pt_dim; pti++ ) {
          ptA[pti] += x*ptB[pti];
        }
      }
    }
  }

  if ( pivot )
    *pivot = piv;

  onfree(tmp_pt);

  return rank;
}


bool
ON_Matrix::BackSolve( 
    double zero_tolerance,
    int Bsize,
    const double* B,
    double* X
    ) const
{
  int i;

  if ( m_col_count > m_row_count )
    return false; // under determined

  if ( Bsize < m_col_count || Bsize > m_row_count )
    return false; // under determined

  for ( i = m_col_count; i < Bsize; i++ ) {
    if ( fabs(B[i]) > zero_tolerance )
      return false; // over determined
  }

  // backsolve
  double const*const* this_m = ThisM();
  const int n = m_col_count-1;
  if ( X != B )
    X[n] = B[n];
  for ( i = n-1; i >= 0; i-- ) {
    X[i] = B[i] - ON_ArrayDotProduct( n-i, &this_m[i][i+1], &X[i+1] );
  }

  return true;
}

bool
ON_Matrix::BackSolve( 
    double zero_tolerance,
    int Bsize,
    const ON_3dPoint* B,
    ON_3dPoint* X
    ) const
{
  int i, j;

  if ( m_col_count > m_row_count )
    return false; // under determined

  if ( Bsize < m_col_count || Bsize > m_row_count )
    return false; // under determined

  for ( i = m_col_count; i < Bsize; i++ ) {
    if ( B[i].MaximumCoordinate() > zero_tolerance )
      return false; // over determined
  }

  // backsolve
  double const*const* this_m = ThisM();
  if ( X != B )
  {
    X[m_col_count-1] = B[m_col_count-1];
    for ( i = m_col_count-2; i >= 0; i-- ) {
      X[i] = B[i];
      for ( j = i+1; j < m_col_count; j++ ) {
        X[i] -= this_m[i][j]*X[j];
      }
    }
  }
  else {
    for ( i = m_col_count-2; i >= 0; i-- ) {
      for ( j = i+1; j < m_col_count; j++ ) {
        X[i] -= this_m[i][j]*X[j];
      }
    }
  }

  return true;
}


bool
ON_Matrix::BackSolve( 
    double zero_tolerance,
    int pt_dim,
    int Bsize,
    int Bpt_stride,
    const double* Bpt,
    int Xpt_stride,
    double* Xpt
    ) const
{
  const int sizeof_pt = pt_dim*sizeof(double);
  double mij;
  int i, j, k;
  const double* Bi;
  double* Xi;
  double* Xj;

  if ( m_col_count > m_row_count )
    return false; // under determined

  if ( Bsize < m_col_count || Bsize > m_row_count )
    return false; // under determined

  for ( i = m_col_count; i < Bsize; i++ ) 
  {
    Bi = Bpt + i*Bpt_stride;
    for( j = 0; j < pt_dim; j++ )
    {
      if ( fabs(Bi[j]) > zero_tolerance )
        return false; // over determined
    }
  }

  // backsolve
  double const*const* this_m = ThisM();
  if ( Xpt != Bpt )
  {
    Xi = Xpt + (m_col_count-1)*Xpt_stride;
    Bi = Bpt + (m_col_count-1)*Bpt_stride;
    memcpy(Xi,Bi,sizeof_pt);
    for ( i = m_col_count-2; i >= 0; i-- ) {
      Xi = Xpt + i*Xpt_stride;
      Bi = Bpt + i*Bpt_stride;
      memcpy(Xi,Bi,sizeof_pt);
      for ( j = i+1; j < m_col_count; j++ ) {
        Xj = Xpt + j*Xpt_stride;
        mij = this_m[i][j];
        for ( k = 0; k < pt_dim; k++ )
          Xi[k] -= mij*Xj[k];
      }
    }
  }
  else {
    for ( i = m_col_count-2; i >= 0; i-- ) {
      Xi = Xpt + i*Xpt_stride;
      for ( j = i+1; j < m_col_count; j++ ) {
        Xj = Xpt + j*Xpt_stride;
        mij = this_m[i][j];
        for ( k = 0; k < pt_dim; k++ )
          Xi[k] -= mij*Xj[k];
      }
    }
  }

  return true;
}


void ON_Matrix::Zero()
{
  struct DBLBLK* cmem = (struct DBLBLK*)m_cmem;
  while ( 0 != cmem )
  {
    if ( 0 != cmem->a && cmem->count > 0 )
    {
      memset( cmem->a, 0, cmem->count*sizeof(cmem->a[0]) );
    }
    cmem = cmem->next;
  }

  //m_a.Zero();
}

void ON_Matrix::SetDiagonal( double d)
{
  const int n = MinCount();
  int i;
  Zero();
  double** this_m = ThisM();
  for ( i = 0; i < n; i++ ) 
  {
    this_m[i][i] = d;
  }
}

void ON_Matrix::SetDiagonal( const double* d )
{
  Zero();
  if (d) 
  {
    double** this_m = ThisM();
    const int n = MinCount();
    int i;
    for ( i = 0; i < n; i++ )
    {
      this_m[i][i] = *d++;
    }
  }
}

void ON_Matrix::SetDiagonal( int count, const double* d )
{
  Create(count,count);
  Zero();
  SetDiagonal(d);
}

void ON_Matrix::SetDiagonal( const ON_SimpleArray<double>& a )
{
  SetDiagonal( a.Count(), a.Array() );
}

bool ON_Matrix::IsValid() const
{
  if ( m_row_count < 1 || m_col_count < 1 )
    return false;
  if ( 0 == m )
    return false;
  return true;
}

int ON_Matrix::IsSquare() const
{
  return ( m_row_count > 0 && m_col_count == m_row_count ) ? m_row_count : 0;
}

bool ON_Matrix::IsRowOrthoganal() const
{
  double d0, d1, d;
  int i0, i1, j;
  double const*const* this_m = ThisM();
  bool rc = ( m_row_count <= m_col_count && m_row_count > 0 );
  for ( i0 = 0; i0 < m_row_count && rc; i0++ ) for ( i1 = i0+1; i1 < m_row_count && rc; i1++ ) {
    d0 = d1 = d = 0.0;
    for ( j = 0; j < m_col_count; j++ ) {
      d0 += fabs(this_m[i0][j]);
      d1 += fabs(this_m[i0][j]);
      d  += this_m[i0][j]*this_m[i1][j];
    }
    if ( d0 <=  ON_EPSILON || d1 <=  ON_EPSILON || fabs(d) >= d0*d1* ON_SQRT_EPSILON )
      rc = false;
  }
  return rc;
}

bool ON_Matrix::IsRowOrthoNormal() const
{
  double d;
  int i, j;
  bool rc = IsRowOrthoganal();
  if ( rc ) {
    double const*const* this_m = ThisM();
    for ( i = 0; i < m_row_count; i++ ) {
      d = 0.0;
      for ( j = 0; j < m_col_count; j++ ) {
        d += this_m[i][j]*this_m[i][j];
      }
      if ( fabs(1.0-d) >=  ON_SQRT_EPSILON )
        rc = false;
    }
  }
  return rc;
}

bool ON_Matrix::IsColOrthoganal() const
{
  double d0, d1, d;
  int i, j0, j1;
  bool rc = ( m_col_count <= m_row_count && m_col_count > 0 );
  double const*const* this_m = ThisM();
  for ( j0 = 0; j0 < m_col_count && rc; j0++ ) for ( j1 = j0+1; j1 < m_col_count && rc; j1++ ) {
    d0 = d1 = d = 0.0;
    for ( i = 0; i < m_row_count; i++ ) {
      d0 += fabs(this_m[i][j0]);
      d1 += fabs(this_m[i][j0]);
      d  += this_m[i][j0]*this_m[i][j1];
    }
    if ( d0 <=  ON_EPSILON || d1 <=  ON_EPSILON || fabs(d) >  ON_SQRT_EPSILON )
      rc = false;
  }
  return rc;
}

bool ON_Matrix::IsColOrthoNormal() const
{
  double d;
  int i, j;
  bool rc = IsColOrthoganal();
  double const*const* this_m = ThisM();
  if ( rc ) {
    for ( j = 0; j < m_col_count; j++ ) {
      d = 0.0;
      for ( i = 0; i < m_row_count; i++ ) {
        d += this_m[i][j]*this_m[i][j];
      }
      if ( fabs(1.0-d) >=  ON_SQRT_EPSILON )
        rc = false;
    }
  }
  return rc;
}

bool ON_Matrix::Invert( double zero_tolerance )
{
  ON_Workspace ws;
  int i, j, k, ix, jx, rank;
  double x;
  const int n = MinCount();
  if ( n < 1 )
    return false;

  ON_Matrix I(m_col_count, m_row_count);

  int* col = ws.GetIntMemory(n);

  I.SetDiagonal(1.0);
  rank = 0;

  double** this_m = ThisM();

  for ( k = 0; k < n; k++ ) {
    // find largest value in sub matrix
    ix = jx = k;
    x = fabs(this_m[ix][jx]);
    for ( i = k; i < n; i++ ) {
      for ( j = k; j < n; j++ ) {
        if ( fabs(this_m[i][j]) > x ) {
          ix = i;
          jx = j;
          x = fabs(this_m[ix][jx]);
        }
      }
    }

    SwapRows( k, ix );
    I.SwapRows( k, ix );

    SwapCols( k, jx );
    col[k] = jx;

    if ( x <= zero_tolerance ) {
      break;
    }
    x = 1.0/this_m[k][k];
    this_m[k][k] = 1.0;
    ON_ArrayScale( m_col_count-k-1, x, &this_m[k][k+1], &this_m[k][k+1] );
    I.RowScale( k, x );

    // zero this_m[!=k][k]'s 
    for ( i = 0; i < n; i++ ) {
      if ( i != k ) {
        x = -this_m[i][k];
        this_m[i][k] = 0.0;
        if ( fabs(x) > zero_tolerance ) {
          ON_Array_aA_plus_B( m_col_count-k-1, x, &this_m[k][k+1], &this_m[i][k+1], &this_m[i][k+1] );
          I.RowOp( i, x, k );
        }
      }
    }
  }

  // take care of column swaps
  for ( i = k-1; i >= 0; i-- ) {
    if ( i != col[i] )
      I.SwapRows(i,col[i]);
  }

  *this = I;

  return (k == n) ? true : false;
}

bool ON_Matrix::Multiply( const ON_Matrix& a, const ON_Matrix& b )
{
  int i, j, k, mult_count;
  double x;
  if (a.ColCount() != b.RowCount() )
    return false;
  if ( a.RowCount() < 1 || a.ColCount() < 1 || b.ColCount() < 1 )
    return false;
  if ( this == &a ) {
    ON_Matrix tmp(a);
    return Multiply(tmp,b);
  }
  if ( this == &b ) {
    ON_Matrix tmp(b);
    return Multiply(a,tmp);
  }
  Create( a.RowCount(), b.ColCount() );
  mult_count = a.ColCount();
  double const*const* am = a.ThisM();
  double const*const* bm = b.ThisM();
  double** this_m = ThisM();
  for ( i = 0; i < m_row_count; i++ ) for ( j = 0; j < m_col_count; j++ ) {
    x = 0.0;
    for (k = 0; k < mult_count; k++ ) {
      x += am[i][k] * bm[k][j];
    }
    this_m[i][j] = x;
  }
  return true;  
}

bool ON_Matrix::Add( const ON_Matrix& a, const ON_Matrix& b )
{
  int i, j;
  if (a.ColCount() != b.ColCount() )
    return false;
  if (a.RowCount() != b.RowCount() )
    return false;
  if ( a.RowCount() < 1 || a.ColCount() < 1 )
    return false;
  if ( this != &a && this != &b ) {
    Create( a.RowCount(), b.ColCount() );
  }
  double const*const* am = a.ThisM();
  double const*const* bm = b.ThisM();
  double** this_m = ThisM();
  for ( i = 0; i < m_row_count; i++ ) for ( j = 0; j < m_col_count; j++ ) {
    this_m[i][j] = am[i][j] + bm[i][j];
  }
  return true;  
}

bool ON_Matrix::Scale( double s )
{
  bool rc = false;
  if ( m_row_count > 0 && m_col_count > 0 ) 
  {
    struct DBLBLK* cmem = (struct DBLBLK*)m_cmem;
    int i;
    double* p;
    while ( 0 != cmem )
    {
      if ( 0 != cmem->a && cmem->count > 0 )
      {
        p = cmem->a;
        i = cmem->count;
        while(i--)
          *p++ *= s;
      }
      cmem = cmem->next;
    }
    rc = true;
  }
  /*
  int i = m_a.Capacity();
  if ( m_row_count > 0 && m_col_count > 0 && m_row_count*m_col_count <= i ) {
    double* p = m_a.Array();
    while ( i-- )
      *p++ *= s;
    rc = true;
  }
  */
  return rc;
}


int ON_RowReduce( int row_count, 
                  int col_count,
                  double zero_pivot,
                  double** A, 
                  double** B, 
                  double pivots[2] 
                 )
{
  // returned A is identity, B = inverse of input A
  const int M = row_count;
  const int N = col_count;
  const size_t sizeof_row = N*sizeof(A[0][0]);
  int i, j, ii;
  double a, p, p0, p1;
  const double* ptr0;
  double* ptr1;

  if ( pivots )
  {
    pivots[0] = 0.0;
    pivots[1] = 0.0;
  }

  if ( zero_pivot <= 0.0 || !ON_IsValid(zero_pivot) )
    zero_pivot = 0.0;


  for ( i = 0; i < M; i++ )
  {
    memset(B[i],0,sizeof_row);
    if ( i < N )
      B[i][i] = 1.0;
  }

  p0 = p1 = A[0][0];

  for ( i = 0; i < M; i++ )
  {
    p = fabs(a = A[i][i]);
    if ( p < p0 ) p0 = p; else if (p > p1) p1 = p;
    if ( 1.0 != a )
    {
      if ( p <= zero_pivot || !ON_IsValid(a) )
      {
        break;
      }
      a = 1.0/a;

      //A[i][i] = 1.0; // no need to do this

      // The "ptr" voodoo is faster but does the same thing as
      //
      //for ( j = i+1; j < N; j++ )
      //  A[i][j] *= a;
      //      
      j = i+1;
      ptr1 = A[i] + j;
      j = N - j;
      while(j--) *ptr1++ *= a;
      
      // The "ptr" voodoo is faster but does the same thing as
      //
      //for ( j = 0; j <= i; j++ )
      //  B[i][j] *= a;
      //            
      ptr1 = B[i];
      j = i+1;
      while(j--) *ptr1++ *= a;
    }

    for ( ii = i+1; ii < M; ii++ )
    {
      a = A[ii][i];
      if ( 0.0 == a )
        continue;
      a = -a;
      
      //A[ii][i] = 0.0;  // no need to do this

      // The "ptr" voodoo is faster but does the same thing as
      //
      //for( j = i+1; j < N; j++ )
      //  A[ii][j] += a*A[i][j];
      //
      j = i+1;
      ptr0 = A[i] + j;
      ptr1 = A[ii] + j;
      j = N - j;
      while(j--) *ptr1++ += a* *ptr0++;

      for( j = 0; j <= i; j++ )
        B[ii][j] += a*B[i][j];
    }
  }

  if ( pivots )
  {
    pivots[0] = p0;
    pivots[1] = p1;
  }

  if ( i < M )
  {
    return i;
  }


  // A is now upper triangular with all 1s on diagonal 
  //   (That is, if the lines that say "no need to do this" are used.)
  // B is lower triangular with a nonzero diagonal
  for ( i = M-1; i >= 0; i-- )
  {
    for ( ii = i-1; ii >= 0; ii-- )
    {
      a = A[ii][i];
      if ( 0.0 == a )
        continue;
      a = -a;
      //A[ii][i] = 0.0; // no need to do this

      // The "ptr" voodoo is faster but does the same thing as
      //
      //for( j = 0; j < N; j++ )
      //  B[ii][j] += a*B[i][j];
      //
      ptr0 = B[i];
      ptr1 = B[ii];
      j = N;
      while(j--) *ptr1++ += a* *ptr0++;
    }
  }

  // At this point, A is trash.
  // If the input A was really nice (positive definite....)
  // the B = inverse of the input A.  If A was not nice,
  // B is also trash.
  return M;
}


int ON_InvertSVDW(
  int count, 
  const double* W,
  double*& invW
  )
{
  double w, maxw;
  int i;

  if ( 0 == W || count <= 0 )
    return -1;

  if ( 0 == invW )
  {
    invW = (double*)onmalloc(count*sizeof(invW[0]));
  }
  maxw = fabs(W[0]);
  for (i = 1; i < count; i++) {
    w = fabs(W[i]);
    if (w > maxw) maxw = w;
  }
  if (maxw == 0.0)
  {
    if ( W != invW )
      memset(invW,0,count*sizeof(invW[0]));
    return 0;
  }

  i = 0;
  maxw *= ON_SQRT_EPSILON;
  while (count--) 
  {
    if (fabs(W[count]) > maxw) 
    {
      i++;
      invW[count] = 1.0/W[count];
    }
    else
      invW[count] = 0.0;
  }
  return i; // number of nonzero terms in invW[]
}

bool ON_SolveSVD(
  int row_count,
  int col_count,
  double const * const * U,
  const double* invW,
  double const * const * V,
  const double* B,
  double*& X
  )  
{
  int i, j;
  double *Y;
  const double* p0;
  double workY[128], x;

  if ( row_count < 1 || col_count < 1 || 0 == U || 0 == invW || 0 == V || 0 == B)
    return false;

  if ( 0 == X )
    X = (double*)onmalloc(col_count*sizeof(X[0]));
  Y = (col_count > 128)
    ? ( (double*)onmalloc(col_count*sizeof(*Y)) )
    : workY;
  for (i = 0; i < col_count; i++)
  {
    double y = 0.0;
    for (j = 0; j < row_count; j++)
      y += U[j][i] * *B++;
    B -= row_count;
    Y[i] = invW[i] * y;
  }
  for (i = 0; i < col_count; i++)
  {
    p0 = V[i];
    j = col_count;
    x = 0.0;
    while (j--)
      x += *p0++ * *Y++;
    Y -= col_count;
    X[i] = x;
  }
  if (Y != workY) 
    onfree(Y);

  return true;
}

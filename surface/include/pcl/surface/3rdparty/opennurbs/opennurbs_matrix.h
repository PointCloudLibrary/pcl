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

#if !defined(ON_MATRIX_INC_)
#define ON_MATRIX_INC_

class ON_Xform;

class ON_CLASS ON_Matrix
{
public:
  ON_Matrix();
  ON_Matrix( 
    int row_count,
    int col_count
     );
  ON_Matrix( // see ON_Matrix::Create(int,int,int,int) for details
     int, // first valid row index
     int, // last valid row index
     int, // first valid column index
     int  // last valid column index
     );
  ON_Matrix( const ON_Xform& );
  ON_Matrix( const ON_Matrix& );

  /*
  Description:
    This constructor is for experts who have storage for a matrix
    and need to use it in ON_Matrix form.
  Parameters:
    row_count - [in]
    col_count - [in]
    M - [in]
    bDestructorFreeM - [in]
      If true, ~ON_Matrix will call onfree(M).
      If false, caller is managing M's memory.
  Remarks:
    ON_Matrix functions that increase the value of row_count or col_count
    will fail on a matrix created with this constructor.
  */
  ON_Matrix(
    int row_count,
    int col_count,
    double** M,
    bool bDestructorFreeM
    );

  virtual ~ON_Matrix();
  void EmergencyDestroy(); // call if memory pool used matrix by becomes invalid

  // ON_Matrix[i][j] = value at row i and column j
  //           0 <= i < RowCount()
  //           0 <= j < ColCount()
  double* operator[](int);
  const double* operator[](int) const;

  ON_Matrix& operator=(const ON_Matrix&);
  ON_Matrix& operator=(const ON_Xform&);

  bool IsValid() const;
  int IsSquare() const; // returns 0 for no and m_row_count (= m_col_count) for yes
  int RowCount() const;
  int ColCount() const;
  int MinCount() const; // smallest of row and column count
  int MaxCount() const; // largest of row and column count

  void RowScale(int,double); 
  void ColScale(int,double);
  void RowOp(int,double,int);
  void ColOp(int,double,int);

  bool Create(
     int, // number of rows
     int  // number of columns
     );

  bool Create( // E.g., Create(1,5,1,7) creates a 5x7 sized matrix that with
               // "top" row = m[1][1],...,m[1][7] and "bottom" row
               // = m[5][1],...,m[5][7].  The result of Create(0,m,0,n) is
               // identical to the result of Create(m+1,n+1).
     int, // first valid row index
     int, // last valid row index
     int, // first valid column index
     int  // last valid column index
     );

  /*
  Description:
    This constructor is for experts who have storage for a matrix
    and need to use it in ON_Matrix form.
  Parameters:
    row_count - [in]
    col_count - [in]
    M - [in]
    bDestructorFreeM - [in]
      If true, ~ON_Matrix will call onfree(M).
      If false, caller is managing M's memory.
  Remarks:
    ON_Matrix functions that increase the value of row_count or col_count
    will fail on a matrix created with this constructor.
  */
  bool Create(
    int row_count,
    int col_count,
    double** M,
    bool bDestructorFreeM
    );


  void Destroy();

  void Zero();

  void SetDiagonal(double); // sets diagonal value and zeros off diagonal values
  void SetDiagonal(const double*); // sets diagonal values and zeros off diagonal values
  void SetDiagonal(int, const double*); // sets size to count x count and diagonal values and zeros off diagonal values
  void SetDiagonal(const ON_SimpleArray<double>&); // sets size to length X lengthdiagonal values and zeros off diagonal values

  bool Transpose();

  bool SwapRows( int, int ); // ints are row indices to swap
  bool SwapCols( int, int ); // ints are col indices to swap
  bool Invert( 
          double // zero tolerance
          );

  /*
  Description:
    Set this = A*B.
  Parameters:
    A - [in]
      (Can be this)
    B - [in]
      (Can be this)
  Returns:
    True when A is an mXk matrix and B is a k X n matrix; in which case
    "this" will be an mXn matrix = A*B.
    False when A.ColCount() != B.RowCount().
  */
  bool Multiply( const ON_Matrix& A, const ON_Matrix& B );

  /*
  Description:
    Set this = A+B.
  Parameters:
    A - [in]
      (Can be this)
    B - [in]
      (Can be this)
  Returns:
    True when A and B are mXn matrices; in which case
    "this" will be an mXn matrix = A+B.
    False when A and B have different sizes.
  */
  bool Add( const ON_Matrix& A, const ON_Matrix& B );


  /*
  Description:
    Set this = s*this.
  Parameters:
    s - [in]
  Returns:
    True when A and s are valid.
  */
  bool Scale( double s );


  // Description:
  //   Row reduce a matrix to calculate rank and determinant.
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) zero tolerance for pivot test
  //       If the absolute value of a pivot is <= zero_tolerance,
  //       then the pivot is assumed to be zero.
  //   determinant - [out] value of determinant is returned here.
  //   pivot - [out] value of the smallest pivot is returned here
  // Returns:
  //   Rank of the matrix.
  // Remarks:
  //   The matrix itself is row reduced so that the result is
  //   an upper triangular matrix with 1's on the diagonal.
  int RowReduce( // returns rank
    double,  // zero_tolerance
    double&, // determinant
    double&  // pivot
    ); 

  // Description:
  //   Row reduce a matrix as the first step in solving M*X=B where
  //   B is a column of values.
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) zero tolerance for pivot test
  //       If the absolute value of a pivot is <= zero_tolerance,
  //       then the pivot is assumed to be zero.
  //   B - [in/out] an array of m_row_count values that is row reduced
  //       with the matrix.
  //   determinant - [out] value of determinant is returned here.
  //   pivot - [out] If not NULL, then the value of the smallest 
  //       pivot is returned here
  // Returns:
  //   Rank of the matrix.
  // Remarks:
  //   The matrix itself is row reduced so that the result is
  //   an upper triangular matrix with 1's on the diagonal.
  // Example:
  //   Solve M*X=B;
  //   double B[m] = ...;
  //   double B[n] = ...;
  //   ON_Matrix M(m,n) = ...;
  //   M.RowReduce(ON_ZERO_TOLERANCE,B); // modifies M and B
  //   M.BackSolve(m,B,X); // solution is in X
  // See Also: 
  //   ON_Matrix::BackSolve
  int RowReduce(
    double,        // zero_tolerance
    double*,       // B
    double* = NULL // pivot
    ); 

  // Description:
  //   Row reduce a matrix as the first step in solving M*X=B where
  //   B is a column of 3d points
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) zero tolerance for pivot test
  //       If the absolute value of a pivot is <= zero_tolerance,
  //       then the pivot is assumed to be zero.
  //   B - [in/out] an array of m_row_count 3d points that is 
  //       row reduced with the matrix.
  //   determinant - [out] value of determinant is returned here.
  //   pivot - [out] If not NULL, then the value of the smallest 
  //       pivot is returned here
  // Returns:
  //   Rank of the matrix.
  // Remarks:
  //   The matrix itself is row reduced so that the result is
  //   an upper triangular matrix with 1's on the diagonal.
  // See Also: 
  //   ON_Matrix::BackSolve
  int RowReduce(
    double,        // zero_tolerance
    ON_3dPoint*,   // B
    double* = NULL // pivot
    ); 

  // Description:
  //   Row reduce a matrix as the first step in solving M*X=B where
  //   B is a column arbitrary dimension points.
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) zero tolerance for pivot test
  //       If a the absolute value of a pivot is <= zero_tolerance,
  //       then the pivoit is assumed to be zero.
  //   pt_dim - [in] dimension of points
  //   pt_stride - [in] stride between points (>=pt_dim)
  //   pt - [in/out] array of m_row_count*pt_stride values.
  //        The i-th point is
  //        (pt[i*pt_stride],...,pt[i*pt_stride+pt_dim-1]).
  //        This array of points is row reduced along with the 
  //        matrix.
  //   pivot - [out] If not NULL, then the value of the smallest 
  //       pivot is returned here
  // Returns:
  //   Rank of the matrix.
  // Remarks:
  //   The matrix itself is row reduced so that the result is
  //   an upper triangular matrix with 1's on the diagonal.
  // See Also: 
  //   ON_Matrix::BackSolve
  int RowReduce( // returns rank
    double,      // zero_tolerance
    int,         // pt_dim
    int,         // pt_stride
    double*,     // pt
    double* = NULL // pivot
    ); 

  // Description:
  //   Solve M*X=B where M is upper triangular with a unit diagonal and
  //   B is a column of values.
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) used to test for "zero" values in B
  //       in under determined systems of equations.
  //   Bsize - [in] (>=m_row_count) length of B.  The values in
  //       B[m_row_count],...,B[Bsize-1] are tested to make sure they are
  //       "zero".
  //   B - [in] array of length Bsize.
  //   X - [out] array of length m_col_count.  Solutions returned here.
  // Remarks:
  //   Actual values M[i][j] with i <= j are ignored. 
  //   M[i][i] is assumed to be one and M[i][j] i<j is assumed to be zero.
  //   For square M, B and X can point to the same memory.
  // See Also:
  //   ON_Matrix::RowReduce
  bool BackSolve(
    double,        // zero_tolerance
    int,           // Bsize
    const double*, // B
    double*        // X
      ) const;

  // Description:
  //   Solve M*X=B where M is upper triangular with a unit diagonal and
  //   B is a column of 3d points.
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) used to test for "zero" values in B
  //       in under determined systems of equations.
  //   Bsize - [in] (>=m_row_count) length of B.  The values in
  //       B[m_row_count],...,B[Bsize-1] are tested to make sure they are
  //       "zero".
  //   B - [in] array of length Bsize.
  //   X - [out] array of length m_col_count.  Solutions returned here.
  // Remarks:
  //   Actual values M[i][j] with i <= j are ignored. 
  //   M[i][i] is assumed to be one and M[i][j] i<j is assumed to be zero.
  //   For square M, B and X can point to the same memory.
  // See Also:
  //   ON_Matrix::RowReduce
  bool BackSolve(
    double,            // zero_tolerance
    int,               // Bsize
    const ON_3dPoint*, // B
    ON_3dPoint*        // X
      ) const;

  // Description:
  //   Solve M*X=B where M is upper triangular with a unit diagonal and
  //   B is a column of points
  // Parameters:
  //   zero_tolerance - [in] (>=0.0) used to test for "zero" values in B
  //       in under determined systems of equations.
  //   pt_dim - [in] dimension of points
  //   Bsize - [in] (>=m_row_count) number of points in B[].  The points
  //       correspoinding to indices m_row_count, ..., (Bsize-1)
  //       are tested to make sure they are "zero".
  //   Bpt_stride - [in] stride between B points (>=pt_dim)
  //   Bpt - [in/out] array of m_row_count*Bpt_stride values.
  //        The i-th B point is
  //        (Bpt[i*Bpt_stride],...,Bpt[i*Bpt_stride+pt_dim-1]).
  //   Xpt_stride - [in] stride between X points (>=pt_dim)
  //   Xpt - [out] array of m_col_count*Xpt_stride values.
  //        The i-th X point is
  //        (Xpt[i*Xpt_stride],...,Xpt[i*Xpt_stride+pt_dim-1]).
  // Remarks:
  //   Actual values M[i][j] with i <= j are ignored. 
  //   M[i][i] is assumed to be one and M[i][j] i<j is assumed to be zero.
  //   For square M, B and X can point to the same memory.
  // See Also:
  //   ON_Matrix::RowReduce
  bool BackSolve(
    double,       // zero_tolerance
    int,          // pt_dim
    int,          // Bsize
    int,          // Bpt_stride
    const double*,// Bpt
    int,          // Xpt_stride
    double*       // Xpt
      ) const;

  bool IsRowOrthoganal() const;
  bool IsRowOrthoNormal() const;

  bool IsColOrthoganal() const;
  bool IsColOrthoNormal() const;


  double** m; // m[i][j] = value at row i and column j
              //           0 <= i < RowCount()
              //           0 <= j < ColCount()
private:
  int m_row_count;
  int m_col_count;
  // m_rowmem[i][j] = row i+m_row_offset and column j+m_col_offset.
  ON_SimpleArray<double*> m_rowmem; 
	double** m_Mmem; // used by Create(row_count,col_count,user_memory,true);
	int   m_row_offset; // = ri0 when sub-matrix constructor is used
	int   m_col_offset; // = ci0 when sub-matrix constructor is used
  void* m_cmem;
  // returns 0 based arrays, even in submatrix case.
  double const * const * ThisM() const;
  double * * ThisM();
};

/*
Description:
  Calculate the singular value decomposition of a matrix.

Parameters:
  row_count - [in]
    number of rows in matrix A
  col_count - [in]
    number of columns in matrix A
  A - [in]
    Matrix for which you want the singular value decomposition.
    A[0][0] = coefficeint in the first row and first column.
    A[row_count-1][col_count-1] = coefficeint in the last row
    and last column.
  U - [out]
    The singular value decomposition of A is U*Diag(W)*Transpose(V),
    where U has the same size as A, Diag(W) is a col_count X col_count
    diagonal matrix with (W[0],...,W[col_count-1]) on the diagonal
    and V is a col_count X col_count matrix.
    U and A may be the same pointer.  If the input value of U is
    null, heap storage will be allocated using onmalloc()
    and the calling function must call onfree(U).  If the input
    value of U is not null, U[i] must point to an array of col_count
    doubles.  
  W - [out]
    If the input value W is null, then heap storage will be allocated
    using onmalloc() and the calling function must call onfree(W).
    If the input value of W is not null, then W must point to
    an array of col_count doubles.
  V - [out]
    If the input value V is null, then heap storage will be allocated
    using onmalloc() and the calling function must call onfree(V).
    If the input value of V is not null, then V[i] must point
    to an array of col_count doubles.

Example:

          int m = row_count;
          int n = col_count;
          ON_Matrix A(m,n);
          for (i = 0; i < m; i++ ) for ( j = 0; j < n; j++ )
          {
            A[i][j] = ...;
          }
          ON_Matrix U(m,n);
          double* W = 0; // ON_GetMatrixSVD() will allocate W
          ON_Matrix V(n,n);
          bool rc = ON_GetMatrixSVD(m,n,A.m,U.m,W,V.m);
          ...
          onfree(W); // W allocated in ON_GetMatrixSVD()

Returns:
  True if the singular value decomposition was cacluated.
  False if the algorithm failed to converge.
*/
ON_DECL
bool ON_GetMatrixSVD(
  int row_count,
  int col_count,
  double const * const * A,
  double**& U,
  double*& W,
  double**& V
  );

/*
Description:
  Invert the diagonal matrix in a the singular value decomposition.
Parameters:
  count - [in] number of elements in W
  W - [in]
    diagonal values in the singular value decomposition.
  invW - [out]
    The inverted diagonal is returned here.  invW may be the same
    pointer as W.  If the input value of invW is not null, it must
    point to an array of count doubles.  If the input value of
    invW is null, heap storage will be allocated using onmalloc()
    and the calling function must call onfree(invW).
Remarks:
  If the singular value decomposition were mathematically perfect, then
  this function would be:
    for (i = 0; i < count; i++) 
      invW[i] = (W[i] != 0.0) ? 1.0/W[i] : 0.0;
  Because the double precision arithmetic is not mathematically perfect,
  very small values of W[i] may well be zero and this function makes
  a reasonable guess as to when W[i] should be treated as zero.  
Returns:
  Number of non-zero elements in invW, which, in a mathematically perfect
  situation, is the rank of Diag(W).
*/
ON_DECL
int ON_InvertSVDW(
  int count, 
  const double* W,
  double*& invW
  );

/*
Description:
  Solve a linear system of equations using the singular value decomposition.
Parameters:
  row_count - [in]
    number of rows in matrix U
  col_count - [in]
    number of columns in matrix U
  U - [in]
    row_count X col_count matix.
    See the remarks section for the definition of U.
  invW - [in]
    inverted DVD diagonal.
    See the remarks section for the definition of invW.
  V - [in]
    col_count X col_count matrix.
    See the remarks section for the definition of V.
  B - [in]
    An array of row_count values.
  X - [out]
    The solution array of col_count values is returned here.
    If the input value of X is not null, it must point to an
    array of col_count doubles.  If the input value of X is
    null, heap storage will be allocated using onmalloc() and
    the calling function must call onfree(X).
Remarks:
  If A*X = B is an m X n system of equations (m = row_count, n = col_count)
  and A = U*Diag(W)*Transpose(V) is the singular value decompostion of A,
  then a solution is X = V*Diag(1/W)*Transpose(U).
Example:

          int m = row_count;
          int n = col_count;
          ON_Matrix A(m,n);
          for (i = 0; i < m; i++ ) for ( j = 0; j < n; j++ )
          {
            A[i][j] = ...;
          }
          ON_SimpleArray<double> B(m);
          for (i = 0; i < m; i++ )
          {
            B[i] = ...;
          }

          ON_SimpleArray<double> X; // solution returned here.
          {
            double** U = 0;
            double* W = 0;
            double** V = 0;
            if ( ON_GetMatrixSVD(m,n,A.m,U,W,V) )
            {
              double* invW = 0;
              int rankW = ON_InvertSVDW(n,W,W); // save invW into W
              X.Reserve(n);
              if ( ON_SolveSVD(m,n,U,W,V,B,X.Array()) )
                X.SetCount(n);
            }
            onfree(U); // U allocated in ON_GetMatrixSVD()
            onfree(W); // W allocated in ON_GetMatrixSVD()
            onfree(V); // V allocated in ON_GetMatrixSVD()
          }

          if ( n == X.Count() )
          {
            ... use solution
          }  
Returns:
  True if input is valid and X[] was calculated. 
  False if input is not valid.
*/
ON_DECL
bool ON_SolveSVD(
  int row_count,
  int col_count,
  double const * const * U,
  const double* invW,
  double const * const * V,
  const double* B,
  double*& X
  );
  

/*
Description:
  Perform simple row reduction on a matrix.  If A is square, positive
  definite, and really really nice, then the returned B is the inverse
  of A.  If A is not positive definite and really really nice, then it
  is probably a waste of time to call this function.
Parameters:
  row_count - [in]
  col_count - [in]
  zero_pivot - [in]
    absolute values <= zero_pivot are considered to be zero
  A - [in/out]
    A row_count X col_count matrix.  Input is the matrix to be
    row reduced.  The calculation destroys A, so output A is garbage.
  B - [out]
    A a row_count X row_count matrix. That records the row reduction.
  pivots - [out]
    minimum and maximum absolute values of pivots.
Returns:
  Rank of A.  If the returned value < min(row_count,col_count),
  then a zero pivot was encountered.
  If C = input value of A, then B*C = (I,*)
*/
ON_DECL
int ON_RowReduce( 
          int row_count, 
          int col_count,
          double zero_pivot,
          double** A, 
          double** B, 
          double pivots[2] 
          );

#endif

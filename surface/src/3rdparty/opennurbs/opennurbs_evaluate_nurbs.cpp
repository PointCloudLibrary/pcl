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

double ON_EvaluateBernsteinBasis(int degree, int i, double t)
/*****************************************************************************
Evaluate Bernstein basis polynomial

INPUT:
  degree
    If degree < 0, then 0.0 is returned.
  i
    If i < 0 or i > degree, then 0.0 is returned.
  t 
    The formula for the Bernstein polynomial is valid
    for all values of t.
OUTPUT:
  TL_EvBernsteinBasis()

         degree!
     ---------------- * (1-t)^(degree-i) * t^i, if 0 <= i <= degree
     (degree-i)! * i!

     0, otherwise.

  (In this function, 0^0 is treated as 1.)
COMMENTS:
  Below, B(d,i,t) is used to denote the i-th Bernstein basis polynomial of 
  degree d; i.e., B(d,i,t) = TL_EvBernsteinBasis(d,i,t).

  When degree <= 4, TL_EvBernsteinBasis() computes the value directly. 
  When 4 < degree < 9, the value is computed recursively using the formula

    B(d,i,t) = t*B(d-1,i-1,t) + (1-t)*B(d-1,i,t).

  For 9 <= degree, the value is computed using the formula

    B(d,i,t) = TL_EvBinomial(degree-i,i)
               *((degree==i) ? 1.0 : pow(1.0-t,(double)(degree-i)))
               *((i)         ? pow(t,(double)i) : 1.0);

  The value of a degree d Bezier at t with control vertices
  {P_0, ..., P_d} is equal to B(d,0,t)*P_0 + ... + B(d,d,t)*P_d.   
  Numerically, this formula is inefficient and unstable.  The
  de Casteljau algorithm used in TL_EvdeCasteljau() is faster
  and more stable.

EXAMPLE:
  // Use TL_EvBernsteinBasis() to evaluate a 3 dimensional
  // non-rational cubic Bezier at 1/4.

  double cv[4][3], t, B[4], answer[3];
  int    i, j, degree;

  degree = 3;
  t = 0.25;
  answer[0] = answer[1] = answer[2] = 0.0;
  for (i = 0; i <= degree; i++)
    cv[i] = something;
  for (i = 0; i <=degree; i++) {
    B[i] = TL_EvBernsteinBasis(degree,i,t);
    for (j = 0; j < 3; j++)
      answer[j] += B[i]*cv[i][j];
  }

REFERENCE:
  BOHM-01, Page 7.
RELATED FUNCTIONS:
  TL_EvNurbBasis
  TL_EvdeCasteljau()
  TL_EvBezier()
  TL_EvHorner()
*****************************************************************************/
{
  double
    s;
  if (degree < 0 || i < 0 || i > degree)
    return 0.0;
  switch(degree) {
  case 0:                       /* degree 0 */
    return 1.0;
  case 1:                       /* degree 1 */
    return ((i) ? t : 1.0-t);
  case 2:                       /* degree 2 */
    switch(i) {
    case 0:
      t = 1.0-t;
      return t*t;
    case 1:
      return 2.0*t*(1.0-t);
    default:                    /* i == 2 */
      return t*t;
    }
  case 3:                       /* degree 3 */
    switch(i) {
    case 0:
      t = 1.0 - t;
      return t*t*t;
    case 1:
      s = 1.0-t;
      return 3.0*s*s*t;
    case 2:
      return 3.0*(1.0-t)*t*t;
    default:                    /* i == 3 */
      return t*t*t;
    }
  case 4:                       /* degree 4 */
    switch(i) {
    case 0:
      t = 1.0-t;
      t = t*t;
      return t*t;
    case 1:                     /* 4*(1-t)^3*t */
      s = 1.0-t;
      return 4.0*s*s*s*t;
    case 2:                     /* 6*(1-t)^2*t */
      s = 1.0-t;
      return 6.0*s*s*t*t;
    case 3:                     /* 4*(1-t)*t^3 */
      return 4.0*(1.0-t)*t*t*t;
    default:                    /* t^4 (i == 4) */
      t = t*t;
      return t*t;
    }
  default:                      /* degree >= 5 */
    /* The "9" was determined to produce the fastest code when
     * tested on a SUN Sparc (SUNOS 4.3, gcc -O)
     */
    if (degree < 9)
      return (t*ON_EvaluateBernsteinBasis(degree-1,i-1,t)
              + (1-t)*ON_EvaluateBernsteinBasis(degree-1,i,t));
    else
      return ON_BinomialCoefficient(degree-i,i)*((degree==i)?1.0:pow(1.0-t,(double)(degree-i)))*((i)?pow(t,(double)i):1.0);
  }
}


void ON_EvaluatedeCasteljau(int dim, int order, int side, int cv_stride, double* cv, double t)
/*****************************************************************************
Evaluate a Bezier using the de Casteljau algorithm

INPUT:
  dim    ( >= 1)
  order  ( >= 2)
  side   <= 0  return left side of bezier in cv array
         >  0  return right side of bezier in cv array
  cv     array of order*cv_stride doubles that specify the Bezier's control
         vertices.
  cv_stride ( >= dim) number of doubles between cv's (typically a multiple of dim).
  t      If side <= 0, then t must be > 0.0.
         If side >  0, then t must be < 1.0.
OUTPUT:
  cv
         If side <= 0, the input cv's are replaced with the cv's for
         the bezier trimmed/extended to [0,t].  In particular,
         {cv[(order-1)*cv_stride], ..., cv[order*cv_stride - 1]} is the value of 
         the Bezier at t.
         If side > 0, the input cv's are replaced with the cv's for
         the Bezier trimmed/extended to [t,1].  In particular,
         {cv[0], ..., cv[dim-1]} is the value of the Bezier at t.
COMMENTS:
  Set C[i,j] = {cv[j*cv_stride], ..., cv[(j+1)*cv_stride-1]}, if i = 0
                (1-t)*C[i-1,j-1] + t*C[i-1,j], if 0 < i <= d = degree

  The collection of C[i,j]'s is typically drawn in a triangular array:

  C[0,0]
           C[1,1]
  C[0,1]              C[2,2]
           C[1,2]             ...
  C[0,2]
       
  ...                                  C[d,d]
                              ...
  C[0,d-1]            C[2,d]
           C[1,d]
  C[0,d]
      
  The value of the Bezier at t is equal to C[d,d].
      
  When side < 0, the input cv's are replaced with
          C[0,0], C[1,2], ..., C[d,d].  
  If the output cv's are used as control vertices for a Bezier, 
  then output_bezier(s) = input_bezier(t*s).
      
  When side >= 0, the input cv's are replace with
          C[d,d], C[d-1,d], ..., C[0,d].
  If the output cv's are used as control vertices for a Bezier, 
  then output_bezier(s) = input_bezier((1-s)*t + s).

  If a Bezier is going to be evaluated more than a few times, it is
  faster to convert the Bezier to power basis and evaluate using
  TL_EvHorner.  However, Horner's algorithm is not a stable as
  de Casteljau's.
EXAMPLE:
  // Use TL_EvdeCasteljau() to trim/extend a Bezier
  // to the interval [t0,t1], where t0 < t1

  double cv[order][dim], t0, t1

  cv = whatever;
  if (1.0 - t0 > t1) {
    // first trim at t0, then trim at t1
    if (t0 != 0.0) TL_EvdeCasteljau(dim,order, 1,cv,dim,t0); 
    t1 = (t1-t0)/(1.0 - t0); // adjust t1 to new domain
    if (t1 != 1.0) TL_EvdeCasteljau(dim,order,-1,cv,dim,t1);
  }
  else {
    // first trim at t1, then trim at t0
    if (t1 != 1.0) TL_EvdeCasteljau(dim,order,-1,cv,dim,t1);
    t0 /= t1; // adjust t0 to new domain
    if (t0 != 0.0) TL_EvdeCasteljau(dim,order, 1,cv,dim,t0);
  }
REFERENCE:
  BOHM-01, Page 8.
RELATED FUNCTIONS:
  TL_EvBernsteinBasis
  TL_EvBezier
  TL_EvdeBoor
  TL_EvHorner
  TL_ConvertBezierToPolynomial
*****************************************************************************/
{
  double
    s, *P0, *P1;
  int
    j, d, off_minus_dim;

  if (t == 0.0 || t == 1.0)
    return;

  s = 1.0 - t;

  /* it's ugly and it's fast */
  if (cv_stride > dim) {
    off_minus_dim = cv_stride - dim;
    if (side > 0) {
      /* output cv's = bezier trimmed to [t,1] */
      while (--order) {
        P0 = cv;                /* first cv */
        P1 = P0 + cv_stride;       /* next cv */
        j = order;
        while (j--) {
          d = dim;
          while (d--) {*P0 = (*P0 * s) + (*P1 * t); P0++; P1++;}
          P0 += off_minus_dim; P1 += off_minus_dim;}}
    }
    else {
      /* side <= 0, so output cv's = bezier trimmed to [0,t] */
      cv += order*dim;          /* now cv = last control vertex */
      while (--order) {
        P1 = cv;                /* last cv */
        P0 = P1 - cv_stride;       /* next to last cv */
        j = order;
        while (j--) {
          d = dim;
          while (d--) {P0--; P1--; *P1 = (*P0 * s) + (*P1 * t);}
          P0 -= off_minus_dim; P1 -= off_minus_dim;}}
    }
  }
  else {
    if (side > 0) {
      /* output cv's = bezier trimmed to [t,1] */
      while (--order) {
        P0 = cv;                /* first cv */
        P1 = P0 + dim;          /* next cv */
        j = order;
        while (j--) {
          d = dim;
          while (d--) {*P0 = (*P0 * s) + (*P1 * t); P0++; P1++;}}
      }
    }
    else {
      /* side <= 0, so output cv's = bezier trimmed to [0,t] */
      cv += order*dim;          /* now cv = last control vertex */
      while (--order) {
        P1 = cv;                /* last cv */
        P0 = P1 - dim;          /* next to last cv */
        j = order;
        while (j--) {
          d = dim;
          while (d--) {P0--; P1--; *P1 = (*P0 * s) + (*P1 * t);}}
      }
    }
  }
}



bool ON_IncreaseBezierDegree(
        int     dim, 
        ON_BOOL32    is_rat, 
        int     order, 
        int     cv_stride,
        double* cv 
        )
/*****************************************************************************
Increase the degree of a Bezier
 
INPUT:
  cvdim (dim + is_rat)
  order ( >= 2 )
    order of input bezier
  cv            
    control vertices of bezier
  newcv    
    array of cvdim*(order+1) doubles (The cv and newcv pointers may be equal.)
OUTPUT:
  newcv  Control vertices of an Bezier with order (order+1).  The new Bezier
         and the old Bezier evaluate to the same point.  
COMMENTS:
  If {B_0, ... B_d} are the control vertices of the input Bezier, then
  {C_0, ..., C_{d+1}} are the control vertices of the returned Bezier, 
  where,
    C_0     = B_0
    C_k     = k/(d+1) * B_{k-1}  +  (d+1-k)/(d+1) * B_{k}(1 < k <= d)
    C_{d+1} = B_d
  The computation is done in a way that permits the pointers cv and newcv
  to be equal; i.e., if the cv array is long enough, the degree may be
  raised with a call like
    TL_IncreaseBezierDegree(cvdim,order,cv,cv);
EXAMPLE:
  raise_degree(TL_BEZIER* bez)
  {  
    // raise the degree of a TL_BEZIER
    bez->cv = (double*) onrealloc ( bez->cv, (bez->order+1)*(bez->dim+bez->is_rat) );
    TL_IncreaseBezierDegree ( bez->dim+bez->is_rat, bez->order,bez->cv,bez->cv );
    bez->order++;
  }
REFERENCE:
  BOHM-01, Page 7.
RELATED FUNCTIONS:
  TL_DecreaseBezierDegree
*****************************************************************************/
{
  double a0, a1, d, c0, c1; 
  int j;
  double* newcv = cv;
  const int cvdim = (is_rat)?dim+1:dim;
  const int dcv = cv_stride - cvdim;


  j = cv_stride*order;
  newcv += j;
  memcpy( newcv, newcv-cv_stride, cvdim*sizeof(*newcv) );
  newcv -= (dcv+1);
  cv = newcv - cv_stride;
  a0 = order;
  a1 = 0.0;
  d = 1.0/a0;
  while (--order) {
    a0 -= 1.0;
    a1 += 1.0;
    c0 = d*a0;
    c1 = d*a1;
    j = cvdim;
    while(j--) {
      *newcv = c0 * *cv + c1 * *newcv; 
      cv--; 
      newcv--;
    }
    cv -= dcv;
    newcv -= dcv;
  }
  return true;
}


bool ON_RemoveBezierSingAt0(
                  int dim, 
                  int order, 
                  int cv_stride,
                  double* cv
                  )
{
  const int cvdim = dim+1;
  int j,k,ord0;
  ord0 = order;
  while(cv[dim] == 0.0) {
    order--; 
    if (order < 2)
      return false;
    j = dim; 
    while(j--) {
      if (cv[j] != 0.0) 
        return false;
    }
    for (j=0;  j<order;  j++) {
      for (k=0;  k<cvdim;  k++) 
        cv[j*cv_stride+k] = (order*cv[(j+1)*cv_stride+k])/(j+1);
    }
  }
  while (order < ord0)
    ON_IncreaseBezierDegree(dim,true,order++,cv_stride,cv);
  return true;
}


bool ON_RemoveBezierSingAt1(
                  int dim, 
                  int order, 
                  int cv_stride,
                  double* cv
                  )
{
  const int cvdim = dim+1;
  int i,k,ord0,CVlen;
  ord0 = order;
  CVlen=order*cvdim;
  while (order > 1 && cv[CVlen-1] == 0.0) {
    order--;
    if (order < 2)
      return false;
    i = dim; 
    while(i--) {
      if (cv[CVlen-1-i] != 0.0)
        return false;
    }
    for (i=0;  i<order;  i++) {
      for (k=0;  k<cvdim;  k++) 
        cv[i*cv_stride+k] = (order*cv[i*cv_stride+k])/(order-i);
    }
    CVlen -= cvdim;
  }
  while(order < ord0)
    ON_IncreaseBezierDegree(dim,true,order++,cv_stride,cv);
  return false;
}


bool ON_EvaluateBezier(
                int dim,              // dimension
                ON_BOOL32 is_rat,          // true if NURBS is rational
                int order,            // order
                int cv_stride,        // cv_stride >= (is_rat)?dim+1:dim
                const double* cv,     // cv[order*cv_stride] array
                double t0, double t1, // domain
                int der_count,        // number of derivatives to compute
                double t,             // evaluation parameter
                int v_stride,         // v_stride (>=dimension)
                double* v             // v[(der_count+1)*v_stride] array
                )
/*****************************************************************************
Evaluate a Bezier
 
INPUT:
  dim
    (>= 1) dimension of Bezier's range
  is_rat
    0: bezier is not rational
    1: bezier is rational
  order
    (>= 2) (order = degree+1)
  cv
    array of (dim+is_rat)*order doubles that define the
    Bezier's control vertices.
  t0, t1  (t0 != t1)
    Bezier's domain.   Mathematically, Beziers have domain [0,1].  In practice
    Beziers are frequently evaluated at (t-t0)/(t1-t0) and the chain
    rule is used to evaluate the derivatives.  This function is the most
    efficient place to apply the chain rule.
  t
    Evaluation parameter
  der_count
    (>= 0)  number of derivatives to evaluate
  answer
     answer[i] is NULL or points to an array of dim doubles.
OUTPUT:
  ON_EvBezier()
    0: successful
   -1: failure - rational function had nonzero numerator and zero
                 denominator
  answer
     bez(t)   = answer[0]
     bez'(t)  = answer[1]
              ...
        (n)
     bez  (t) = answer[n]
COMMENTS:
  Use de Casteljau's algorithm.  Rational fuctions with removable singularities
  (like x^2/x) are efficiently and correctly handled.
EXAMPLE:
  // ...
REFERENCE:
  AUTHOR page ?
RELATED FUNCTIONS:
  ON_EvaluatedeCasteljau
  ON_EvQuotientRule
  ON_EvNurb
  ON_EvPolynomialPoint
  ON_onvertBezierToPolynomial
  ON_onvertPolynomialToBezier
  ON_onvertNurbToBezier
*****************************************************************************/
{
  unsigned char stack_buffer[4*64*sizeof(double)];
  double delta_t;
  double alpha0;
  double alpha1;
  double *cv0, *cv1;
  int i, j, k; 
  double* CV, *tmp;
  void* free_me = 0;
  const int degree = order-1;
  const int cvdim = (is_rat)?dim+1:dim;

  if ( cv_stride < cvdim )
    cv_stride = cvdim;

  memset( v, 0, v_stride*(der_count+1)*sizeof(*v) );

#if defined( ON_DEBUG)
  if (t0==t1) {
    return false;
  }
#endif

  i = order*cvdim;
  j = 0;
  if (der_count > degree) {
    if (is_rat)
      j = (der_count-degree)*cvdim;
    else {
      der_count = degree;
    }    
  }

  std::size_t sizeofCV = (i+j)*sizeof(*CV);


  // 21 November 2007 Dale Lear RR 29005 - remove call to alloca()
  CV = (double*)( (sizeofCV <= sizeof(stack_buffer)) ? stack_buffer : (free_me=onmalloc(sizeofCV)) );
  if (j) {
    memset( CV+i, 0, j*sizeof(*CV) );
  }
  cv0=CV;
  if (    t0 == t 
       || (t <= 0.5*(t0+t1) && t != t1) 
     ) 
  {
    for ( i = 0; i < order; i++ ) 
    {
      memcpy( cv0, cv, cvdim*sizeof(*cv0) );
      cv0 += cvdim;
      cv += cv_stride;
    }
    cv -= (cv_stride*order);
    delta_t = 1.0/(t1 - t);
    alpha1 = 1.0/(t1-t0);
    alpha0 = (t1-t)*alpha1;
    alpha1 *= t-t0;
  }
  else 
  {
    cv += (cv_stride*order);
    k=order;
    while(k--) 
    {
      cv -= cv_stride; 
      memcpy( cv0, cv, cvdim*sizeof(*cv0) );
      cv0 += cvdim;
    }
    delta_t = 1.0/(t0 - t);
    alpha0 = 1.0/(t1-t0);
    alpha1 = (t1-t)*alpha0;
    alpha0 *= t-t0;
  }

  /* deCasteljau (from the right) */
  if (alpha1 != 0.0) {
    j = order; while (--j) {
      cv0 = CV;
      cv1 = cv0 + cvdim;
      i = j; while (i--) {
        k = cvdim; 
        while (k--) {
          *cv0 = *cv0 * alpha0 + *cv1 * alpha1; 
          cv0++; 
          cv1++;
        }
      }
    }
  }
  
  /* check for removable singularity */
  if (is_rat && CV[dim] == 0.0)
  {
    if ( !ON_RemoveBezierSingAt0(dim,order,cvdim,CV) )
    {
      if ( free_me )
        onfree(free_me);  
      return false;
    }
  }

  /* Lee (from the right) */
  if (der_count) { 
    tmp=CV;
    alpha0 = order;
    j = (der_count>=order)?order:der_count+1;
    CV += cvdim*j; while(--j) {
      alpha0 -= 1.0; cv1 = CV; cv0 = cv1-cvdim;
      i=j; while(i--) {
        alpha1 = alpha0 * delta_t;
        k=cvdim; while(k--) {
          cv0--; 
          cv1--; 
          *cv1 = alpha1*(*cv1 - *cv0);
        }
      }
    }
    CV=tmp;
  }

  if ( 2 == order )
  {
    // 7 January 2004  Dale Lear
    //    Added to fix those cases when, numerically, t*a + (1.0-t)*a != a.
    //    Similar to fix for RR 9683.
    j = cv_stride;
    for ( i = 0; i < cvdim; i++, j++ )
    {
      if ( cv[i] == cv[j] )
      {
        CV[i] = cv[i];
      }
    }
  }

  if (is_rat) {
    ON_EvaluateQuotientRule( dim, der_count, cvdim, CV );
  }

  for (i=0;i<=der_count;i++) {
    memcpy( v, CV, dim*sizeof(*v) );
    v += v_stride;
    CV += cvdim;
  }

  if ( free_me )
    onfree(free_me);  

  return true;
}


bool ON_EvaluateNurbsBasis( int order, const double* knot, 
                                       double t, double* N )
{
/*****************************************************************************
Evaluate B-spline basis functions
 
INPUT:
  order >= 1 
    d = degree = order - 1
  knot[]
    array of length 2*d.  
    Generally, knot[0] <= ... <= knot[d-1] < knot[d] <= ... <= knot[2*d-1].
  N[]
    array of length order*order

OUTPUT:
  If "N" were declared as double N[order][order], then

                 k
    N[d-k][i] = N (t) = value of i-th degree k basis function.
                 i
  where 0 <= k <= d and k <= i <= d.

	In particular, N[0], ..., N[d] - values of degree d basis functions.
  The "lower left" triangle is not initialized.

  Actually, the above is true when knot[d-1] <= t < knot[d].  Otherwise, the
  value returned is the value of the polynomial that agrees with N_i^k on the
  half open domain [ knot[d-1], knot[d] )

COMMENTS:
  If a degree d NURBS has n control points, then the TL knot vector has
  length d+n-1. ( Most literature, including DeBoor and The NURBS Book,
  duplicate the TL start and end knot and have knot vectors of length
  d+n+1. )
  
  Assume C is a B-spline of degree d (order=d+1) with n control vertices
  (n>=d+1) and knot[] is its knot vector.  Then

    C(t) = Sum( 0 <= i < n, N_{i}(t) * C_{i} )

  where N_{i} are the degree d b-spline basis functions and C_{i} are the control
  vertices.  The knot[] array length d+n-1 and satisfies

    knot[0] <= ... <= knot[d-1] < knot[d]
    knot[n-2] < knot[n-1] <= ... <= knot[n+d-2]
    knot[i] < knot[d+i] for 0 <= i < n-1
    knot[i] <= knot[i+1] for 0 <= i < n+d-2

  The domain of C is [ knot[d-1], knot[n-1] ].

  The support of N_{i} is [ knot[i-1], knot[i+d] ).

  If d-1 <= k < n-1 and knot[k] <= t < knot[k+1], then 
  N_{i}(t) = 0 if i <= k-d
           = 0 if i >= k+2
           = B[i-k+d-1] if k-d+1 <= i <= k+1, where B[] is computed by the call
             TL_EvNurbBasis( d+1, knot+k-d+1, t, B );

  If 0 <= j < n-d, 0 <= m <= d, knot[j+d-1] <= t < knot[j+d], and B[] is 
  computed by the call

    TL_EvNurbBasis( d+1, knot+j, t, B ),

  then 

    N_{j+m}(t) = B[m].

EXAMPLE:
REFERENCE:
  The NURBS book
RELATED FUNCTIONS:
  TL_EvNurbBasis
  TL_EvNurbBasisDer
*****************************************************************************/
  double a0, a1, x, y;
  const double *k0;
  double *t_k, *k_t, *N0;
  const int d = order-1;
  int j, r;

  t_k = (double*)alloca( d<<4 );
  k_t = t_k + d;
  
  if (knot[d-1] == knot[d]) {
		/* value is defined to be zero on empty spans */
    memset( N, 0, order*order*sizeof(*N) );
    return true;
  }

  N  += order*order-1;
	N[0] = 1.0;
  knot += d;
  k0 = knot - 1;

  for (j = 0; j < d; j++ ) {
		N0 = N;
    N -= order+1;
    t_k[j] = t - *k0--;
    k_t[j] = *knot++ - t;
		
    x = 0.0;
    for (r = 0; r <= j; r++) {
      a0 = t_k[j-r];
      a1 = k_t[r];
      y = N0[r]/(a0 + a1);
      N[r] = x + a1*y;
      x = a0*y;
    }

    N[r] = x;
  }

  // 16 September 2003 Dale Lear (at Chuck's request)
  //   When t is at an end knot, do a check to
  //   get exact values of basis functions.
  //   The problem being that a0*y above can
  //   fail to be one by a bit or two when knot
  //   values are large.
  x = 1.0-ON_SQRT_EPSILON;
  if ( N[0] > x )
  {
    if ( N[0] != 1.0 && N[0] < 1.0 + ON_SQRT_EPSILON )
    {
      r = 1;
      for ( j = 1; j <= d && r; j++ )
      {
        if ( N[j] != 0.0 )
          r = 0;
      }
      if (r)
        N[0] = 1.0;
    }
  }
  else if ( N[d] > x )
  {
    if ( N[d] != 1.0 && N[d] < 1.0 + ON_SQRT_EPSILON )
    {
      r = 1;
      for ( j = 0; j < d && r; j++ )
      {
        if ( N[j] != 0.0 )
          r = 0;
      }
      if (r)
        N[d] = 1.0;
    }
  }

  return true;
}


bool ON_EvaluateNurbsBasisDerivatives( int order, const double* knot, 
                       int der_count, double* N )
{
	/* INPUT:
	 *   Results of the call
	 *      TL_EvNurbBasis( order, knot, t, N );  (initializes N[] )
	 *   are sent to
	 *      TL_EvNurbBasisDer( order, knot, der_count, N ),
	 *   where 1 <= der_count < order
	 *
	 * OUTPUT:
   *  If "N" were declared as double N[order][order], then
	 *
   *                                    d
   *    N[d-k][i] = k-th derivative of N (t)
   *                                    i
   *
	 *  where 0 <= k <= d and 0 <= i <= d.
	 *
	 * In particular, 
	 *   N[0], ..., N[d] - values of degree d basis functions.
	 *   N[order], ..., N[order_d] - values of first derivative.
	 *
   * Actually, the above is true when knot[d-1] <= t < knot[d].  Otherwise, the
   * values returned are the values of the polynomials that agree with N_i^k on the
   * half open domain [ knot[d-1], knot[d] )
	 *
	 * Ref: The NURBS Book
	 */
	double dN, c;
	const double *k0, *k1;
	double *a0, *a1, *ptr, **dk;
	int i, j, k, jmax;

	const int d = order-1;
	const int Nstride = -der_count*order;

	/* workspaces for knot differences and coefficients 
	 *
	 * a0[] and a1[] have order doubles
	 *
	 * dk[0] = array of d knot differences
	 * dk[1] = array of (d-1) knot differences
	 *
	 * dk[der_count-1] = 1.0/(knot[d] - knot[d-1])
	 * dk[der_count] = dummy pointer to make loop efficient
	 */
	dk = (double**)alloca( (der_count+1) << 3 ); /* << 3 in case pointers are 8 bytes long */
	a0 = (double*)alloca( (order*(2 + ((d+1)>>1))) << 3 ); /* d for a0, d for a1, d*order/2 for dk[]'s and slop to avoid /2 */
	a1 = a0 + order;

	/* initialize reciprocal of knot differences */
	dk[0] = a1 + order;
	for (k = 0; k < der_count; k++) {
		j = d-k;
		k0 = knot++;
		k1 = k0 + j;
		for (i = 0; i < j; i++) 
			dk[k][i] = 1.0/(*k1++ - *k0++);
		dk[k+1] = dk[k] + j;
	}
	dk--;
	/* dk[1] = 1/{t[d]-t[0], t[d+1]-t[1], ..., t[2d-2] - t[d-2], t[2d-1] - t[d-1]}
	 *       = diffs needed for 1rst derivative
	 * dk[2] = 1/{t[d]-t[1], t[d+1]-t[2], ..., t[2d-2] - t[d-1]}
	 *       = diffs needed for 2nd derivative
	 * ...
	 * dk[d] = 1/{t[d] - t[d-1]}
	 *       = diff needed for d-th derivative
	 *
	 * d[k][n] = 1.0/( t[d+n] - t[k-1+n] )
	 */
	
	N += order;
	/* set N[0] ,..., N[d] = 1rst derivatives, 
	 * N[order], ..., N[order+d] = 2nd, etc.
	 */
	for ( i=0; i<order; i++) {
		a0[0] = 1.0;
		for (k = 1; k <= der_count; k++) {
			/* compute k-th derivative of N_i^d up to d!/(d-k)! scaling factor */
			dN = 0.0;
			j = k-i; 
			if (j <= 0) {
				dN = (a1[0] = a0[0]*dk[k][i-k])*N[i];
				j = 1;
			}
			jmax = d-i; 
			if (jmax < k) {
				while (j <= jmax) {
					dN += (a1[j] = (a0[j] - a0[j-1])*dk[k][i+j-k])*N[i+j];
					j++;
				}
			}
			else {
				/* sum j all the way to j = k */
				while (j < k) {
					dN += (a1[j] = (a0[j] - a0[j-1])*dk[k][i+j-k])*N[i+j];
					j++;
				}
				dN += (a1[k] = -a0[k-1]*dk[k][i])*N[i+k];
			}

			/* d!/(d-k)!*dN = value of k-th derivative */
			N[i] = dN;
			N += order;
			/* a1[] s for next derivative = linear combination
			 * of a[]s used to compute this derivative.
			 */
			ptr = a0; a0 = a1; a1 = ptr;
		}
		N += Nstride;
	}

	/* apply d!/(d-k)! scaling factor */
	dN = c = (double)d;
	k = der_count;
	while (k--) {
		i = order;
		while (i--)
			*N++ *= c;
		dN -= 1.0;
		c *= dN;
	}
  return true;
}

static
bool ON_EvaluateNurbsNonRationalSpan( 
                  int dim,             // dimension
                  int order,           // order
                  const double* knot,  // knot[] array of (2*order-2) doubles
                  int cv_stride,       // cv_stride >= (is_rat)?dim+1:dim
                  const double* cv,    // cv[order*cv_stride] array
                  int der_count,       // number of derivatives to compute
                  double t,            // evaluation parameter
                  int v_stride,        // v_stride (>=dimension)
                  double* v            // v[(der_count+1)*v_stride] array
                  )
{
  const int stride_minus_dim = cv_stride - dim;
  const int cv_len = cv_stride*order;
  int i, j, k;
  double *N;
	double a;

	N = (double*)alloca( (order*order)<<3 );

  if ( stride_minus_dim > 0)
  {
    i = (der_count+1);
    while( i--)
    {
      memset(v,0,dim*sizeof(v[0]));
      v += v_stride;
    }
    v -= ((der_count+1)*v_stride);
  }
  else
  {
    memset( v, 0, (der_count+1)*v_stride*sizeof(*v) );
  }

  if ( der_count >= order )
    der_count = order-1;

	// evaluate basis functions
	ON_EvaluateNurbsBasis( order, knot, t, N );
	if ( der_count ) 
		ON_EvaluateNurbsBasisDerivatives( order, knot, der_count, N );

	// convert cv's into answers
	for (i = 0; i <= der_count; i++, v += v_stride, N += order) {
    for ( j = 0; j < order; j++ ) {
      a = N[j];
      for ( k = 0; k < dim; k++ ) {
        *v++ += a* *cv++;
      }
      v -= dim;
      cv += stride_minus_dim;
    }
    cv -= cv_len;
	}

  if ( 2 == order )
  {
    // 7 January 2004  Dale Lear
    //    Added to fix those cases when, numerically, t*a + (1.0-t)*a != a.
    //    Similar to fix for RR 9683.
    v -= (der_count+1)*v_stride;
    j = cv_stride;
    for ( i = 0; i < dim; i++, j++ )
    {
      if (cv[i] == cv[j] )
        v[i] = cv[i];
    }
  }
		
	return true;
}

static
bool ON_EvaluateNurbsRationalSpan( 
                  int dim,             // dimension
                  int order,           // order
                  const double* knot,  // knot[] array of (2*order-2) doubles
                  int cv_stride,       // cv_stride >= (is_rat)?dim+1:dim
                  const double* cv,    // cv[order*cv_stride] array
                  int der_count,       // number of derivatives to compute
                  double t,            // evaluation parameter
                  int v_stride,        // v_stride (>=dimension)
                  double* v            // v[(der_count+1)*v_stride] array
                  )
{
  const int hv_stride = dim+1;
  double *hv;
  int i;
  bool rc;

  hv = (double*)alloca( (der_count+1)*hv_stride*sizeof(*hv) );
  
  rc = ON_EvaluateNurbsNonRationalSpan( dim+1, order, knot, 
          cv_stride, cv, der_count, t, hv_stride, hv );
  if (rc) {
    rc = ON_EvaluateQuotientRule(dim, der_count, hv_stride, hv);
  }
  if ( rc ) {
    // copy answer to v[]
    for ( i = 0; i <= der_count; i++ ) {
      memcpy( v, hv, dim*sizeof(*v) );
      v += v_stride;
      hv += hv_stride;
    }
  }
  return rc;
}


bool ON_EvaluateNurbsSpan( 
                  int dim,             // dimension
                  ON_BOOL32 is_rat,         // true if NURBS is rational
                  int order,           // order
                  const double* knot,  // knot[] array of (2*order-2) doubles
                  int cv_stride,       // cv_stride >= (is_rat)?dim+1:dim
                  const double* cv,    // cv[order*cv_stride] array
                  int der_count,       // number of derivatives to compute
                  double t,            // evaluation parameter
                  int v_stride,        // v_stride (>=dimension)
                  double* v            // v[(der_count+1)*v_stride] array
                  )
{
  bool rc = false;
  if ( knot[0] == knot[order-2] && knot[order-1] == knot[2*order-3] ) {
    // Bezier span - use faster Bezier evaluator
    rc = ON_EvaluateBezier(dim, is_rat, order, cv_stride, cv,
                            knot[order-2], knot[order-1],
                            der_count, t, v_stride, v);
  }
  else {
    // generic NURBS span evaluation
    rc = (is_rat)
         ? ON_EvaluateNurbsRationalSpan( 
              dim, order, knot, cv_stride, cv,
              der_count, t, v_stride, v )
         : ON_EvaluateNurbsNonRationalSpan( 
              dim, order, knot, cv_stride, cv,
              der_count, t, v_stride, v );
  }
  return rc;
}


bool ON_EvaluateNurbsSurfaceSpan(
        int dim,
        ON_BOOL32 is_rat,
        int order0, int order1,
        const double* knot0,
        const double* knot1,
        int cv_stride0, int cv_stride1,
        const double* cv0, // cv at "lower left" of bispan
        int der_count,
        double t0, double t1,
        int v_stride, 
        double* v      // returns values
        )
{
	const int der_count0 = (der_count >= order0) ? order0-1 : der_count;
	const int der_count1 = (der_count >= order1) ? order1-1 : der_count;
	const double *cv;

	double *N_0, *N_1, *P0, *P;
	double c;
	int d1max, d, d0, d1, i, j, j0, j1, Pcount, Psize;

  const int cvdim = (is_rat) ? dim+1 : dim;
  const int dcv1 = cv_stride1 - cvdim;

	// get work space memory
	i = order0*order0;
	j = order1*order1;
	Pcount = ((der_count+1)*(der_count+2))>>1;
	Psize = cvdim<<3;
  N_0 = (double*)alloca( ((i + j) << 3) + Pcount*Psize );
	N_1 = N_0 + i;
	P0  = N_1 + j;
	memset( P0, 0, Pcount*Psize );

	/* evaluate basis functions */
	ON_EvaluateNurbsBasis( order0, knot0, t0, N_0 );
	ON_EvaluateNurbsBasis( order1, knot1, t1, N_1 );
	if (der_count0) {
    // der_count0 > 0 iff der_count1 > 0 
		ON_EvaluateNurbsBasisDerivatives( order0, knot0, der_count0, N_0 );
		ON_EvaluateNurbsBasisDerivatives( order1, knot1, der_count1, N_1 );
	}

  // compute point
	P = P0;
	for ( j0 = 0; j0 < order0; j0++) {
    cv = cv0 + j0*cv_stride0;
		for ( j1 = 0; j1 < order1; j1++ ) {
			c = N_0[j0]*N_1[j1];
			j = cvdim;
			while (j--) 
				*P++ += c* *cv++;
			P -= cvdim;
      cv += dcv1;
		}
	}

  if ( der_count > 0 ) {
    // compute first derivatives
  	P += cvdim; // step over point
		for ( j0 = 0; j0 < order0; j0++) {
      cv = cv0 + j0*cv_stride0;
			for ( j1 = 0; j1 < order1; j1++ ) {
        // "Ds"
				c = N_0[j0+order0]*N_1[j1];
				j = cvdim;
				while (j--) 
					*P++ += c* *cv++;
        cv -= cvdim;

        // "Dt"
				c = N_0[j0]*N_1[j1+order1];
				j = cvdim;
				while (j--) 
					*P++ += c* *cv++;
				P -= cvdim;
				P -= cvdim;

        cv += dcv1;
			}
		}

    if ( der_count > 1 ) {
      // compute second derivatives
      P += cvdim; // step over "Ds"
      P += cvdim; // step over "Dt"
      if ( der_count0+der_count1 > 1 ) {
        // compute "Dss"
		    for ( j0 = 0; j0 < order0; j0++) {
          // P points to first coordinate of Dss
          cv = cv0 + j0*cv_stride0;
			    for ( j1 = 0; j1 < order1; j1++ ) {
            if ( der_count0 > 1 ) {
              // "Dss"
				      c = N_0[j0+2*order0]*N_1[j1];
				      j = cvdim;
				      while (j--) 
					      *P++ += c* *cv++;
              cv -= cvdim;
            }
            else {
              P += cvdim; // Dss = 0
            }

            // "Dst"
				    c = N_0[j0+order0]*N_1[j1+order1];
				    j = cvdim;
				    while (j--) 
					    *P++ += c* *cv++;
            cv -= cvdim;

            if ( der_count1 > 1 ) {
              // "Dtt"
				      c = N_0[j0]*N_1[j1+2*order1];
				      j = cvdim;
				      while (j--) 
					      *P++ += c* *cv++;
              cv -= cvdim;
  				    P -= cvdim;
            }

				    P -= cvdim;
				    P -= cvdim;
            cv += cv_stride1;
			    }
		    }
      }

      if ( der_count > 2 ) 
      {
        // 12 February 2004 Dale Lear
        //     Bug fix for d^n/ds^n when n >= 3
        // compute higher derivatives in slower generic loop
        for ( d = 3; d <= der_count; d++ ) 
        {
          P += d*cvdim; // step over (d-1)th derivatives
          d1max = (d > der_count1) ? der_count1 : d;
			    for ( j0 = 0; j0 < order0; j0++) 
          {
            cv = cv0 + j0*cv_stride0;
				    for ( j1 = 0; j1 < order1; j1++ ) 
            {
              for (d0 = d, d1 = 0; 
                   d0 > der_count0 && d1 <= d1max; 
                   d0--, d1++ ) 
              {
                // partial with respect to "s" is zero
                P += cvdim;
              }
              for ( /*empty*/; d1 <= d1max; d0--, d1++ ) 
              {
                c = N_0[j0 + d0*order0]*N_1[j1 + d1*order1];
					      j = cvdim;
					      while (j--) 
						      *P++ += c* *cv++;
                cv -= cvdim;
              }
              // remaining partials with respect to "t" are zero
              // - reset and add contribution from the next cv
              P -= d1*cvdim;
              cv += cv_stride1;
				    }
			    }
        }
      }

    }
  }

	if ( is_rat ) {
		ON_EvaluateQuotientRule2( dim, der_count, cvdim, P0 );
		Psize -= 8;
	}
	for ( i = 0; i < Pcount; i++) {
		memcpy( v, P0, Psize );
    v += v_stride;
		P0 += cvdim;
	}

	return true;
}


bool ON_EvaluateNurbsDeBoor(
                           int cv_dim,
                           int order, 
                           int cv_stride,
                           double *cv,
                           const double *knots, 
                           int side,
                           double mult_k, 
                           double t
                           )
/*
 * Evaluate a B-spline span using the de Boor algorithm
 *
 * INPUT:
 *   cv_dim
 *      >= 1
 *   order
 *      (>= 2)  There is no restriction on order.  For order >= 18,
 *      the necessary workspace is dynamically allocated and deallocated.
 *      (The function requires a workspace of 2*order-2 doubles.)
 *   cv
 *      array of order*cv_dim doubles that specify the B-spline span's
 *      control vertices.
 *   knots
 *      array of 2*(order-1) doubles that specify the B-spline span's
 *      knot vector.
 *   side
 *      -1  return left side of B-spline span in cv array
 *      +1  return right side of B-spline span in cv array
 *      -2  return left side of B-spline span in cv array
 *          Ignore values of knots[0,...,order-3] and assume
 *          left end of span has a fully multiple knot with
 *          value "mult_k".
 *      +2  return right side of B-spline span in cv array
 *          Ignore values of knots[order,...,2*order-2] and
 *          assume right end of span has a fully multiple
 *          knot with value "mult_k".
 *      WARNING: If side is != {-2,-1,+1,+2}, this function may crash
 *               or return garbage.
 *   mult_k
 *      Used when side = -2 or +2.
 *   t 
 *      If side < 0, then the cv's for the portion of the NURB span to
 *      the LEFT of t are computed.  If side > 0, then the cv's for the
 *      portion the span to the RIGHT of t are computed.  The following
 *      table summarizes the restrictions on t:
 *
 *       value of side         condition t must satisfy
 *          -2                    mult_k < t and mult_k < knots[order-1]
 *          -1                    knots[order-2] < t
 *          +1                    t < knots[order-1]
 *          +2                    t < mult_k and knots[order-2] < mult_k
 *
 * OUTPUT:
 *   cv
 *      If side <= 0, the input cv's are replaced with the cv's for
 *      the B-spline span trimmed/extended to [knots[order-2],t]  with
 *      new knot vector {knots[0], ..., knots[order-2], t, ..., t}.
 *                                                      \________/
 *                                                       order-1 t's
 *      In particular, {cv[(order-1)*cv_dim], ..., cv[order*cv_dim - 1]}
 *      is the value of the B-spline at t.
 *      If side > 0, the input cv's are replaced with the cv's for
 *      the B-spline span trimmed/extended to [t,knots[order-1]]  with
 *      new knot vector {t, ..., t, knots[order-1], ..., knots[2*order-3]}.
 *                       \________/
 *                       order-1 t's
 *      In particular, {cv[0], ..., cv[cv_dim-1]} is the value of the B-spline
 *      at t.
 *
 *      NOTE WELL: The input knot vector is NOT modified.  If you want to
 *                 use the returned control points with the input knot vector,
 *                 then it is your responsibility to set
 *                    knots[0] = ... = knots[order-2] = t (side > 0)
 *                 or
 *                    knots[order-1] = ... = knots[2*order-2] = t (side < 0).
 *                 See the comments concering +/- 2 values of the "side"
 *                 argument.  In most cases, you can avoid resetting knots
 *                 by carefully choosing the value of "side" and "mult_k".
 *  TL_EvDeBoor()
 *   0: successful
 *      -1: knot[order-2] == knot[order-1]
 *
 * COMMENTS:
 *
 *   This function is the single most important NURB curve function in the
 *   TL library.  It is used to evaluate, trim, split and extend NURB curves.
 *   It is used to convert portions of NURB curves to Beziers and to create
 *   fully multiple end knots.  The functions that perform the above tasks
 *   simply call this function with appropriate values and take linear
 *   combonations of the returned cv's to compute the desired result.
 *
 *   Rational cases are handled adding one to the dimension and applying the
 *   quotient rule as needed.
 *
 *   Set a[i,j] = (t-knots[i+j-1])/(knots[i+j+order-2] - knots[i+j-1])
 *   Set D[i,j] = {cv[j*cv_dim], ..., cv[(j+1)*cv_dim-1]}, if i = 0
 *                (1-a[i,j])*D[i-1,j-1] + a[i,j]*D[i-1,j], if 0 < i <= d = degree
 * 
 *   The collection of D[i,j]'s is typically drawn in a triangular array:
 *
 *   D[0,0]
 *            D[1,1]
 *   D[0,1]              D[2,2]
 *            D[1,2]             ...
 *   D[0,2]
 *  
 *   ...                                  D[d,d]
 *                               ...
 *   D[0,d-1]            D[2,d]
 *            D[1,d]
 *   D[0,d]
 *
 *   When side <= 0, the input cv's are replaced with
 *   D[0,0], D[1,2], ..., D[d,d].
 *
 *   When side >  0, the input cv's are replace with
 *   D[d,d], D[d-1,d], ..., D[0,d].
 *
 * EXAMPLE:
 *
 * REFERENCE:
 *   BOHM-01, Page 16.
 *   LEE-01, Section 6.
 *
 * RELATED FUNCTIONS:
 *   TL_EvNurbBasis(), TL_EvNurb(), TL_EvdeCasteljau(), TL_EvQuotientRule()
 */
{
  double 
    workarray[21], alpha0, alpha1, t0, t1, dt, *delta_t, *free_delta_t, *cv0, *cv1;
  const double 
    *k0, *k1;
  int
    degree, i, j, k;

  const int cv_inc = cv_stride - cv_dim;

  j = 0;
  delta_t = workarray;
  free_delta_t = 0;
  degree = order-1;  
  t0 = knots[degree-1];
  t1 = knots[degree];
  if (t0 == t1) {
    ON_ERROR("ON_EvaluateNurbsDeBoor(): knots[degree-1] == knots[degree]");
    return false;
  }

  if (side < 0) {
    /* if right end of span is fully multiple and t = end knot,
     * then we're done.
     */
    if (t == t1 && t1 == knots[2*degree-1])
      return true;
    /* if left end of span is fully multiple, save time */
    if (side == -2)
      t0 = mult_k;
    else if (t0 == knots[0])
      side = -2;
    else {
      side = -1;
      if (degree > 21)
        delta_t = free_delta_t = (double*)onmalloc(degree*sizeof(*delta_t));
    }
    /* delta_t = {t - knot[order-2], t - knot[order-1], ... , t - knot[0]} */
    knots += degree-1;
    if (side != -2) {
      k0=knots; k=degree; while(k--) *delta_t++ = t - *k0--; delta_t -= degree;
      cv += order*cv_stride;
      k = order; while (--k) { 
        cv1 = cv;
        cv0 = cv1 - cv_stride;
        k0 = knots;             /* *k0 = input_knots[d-1]          */
        k1 = k0+k;              /* *k1 = input_knots[d-1+k]        */ 
        i = k; while(i--) {
          alpha1 = *delta_t++/(*k1-- - *k0--); 
          alpha0 = 1.0 - alpha1;
          cv0 -= cv_inc;
          cv1 -= cv_inc;
          j = cv_dim;
          while (j--) {cv0--; cv1--; *cv1 = *cv0 * alpha0 + *cv1 * alpha1;} 
        }
        delta_t -= k;
      }
    }
    else {
      dt = t - t0;
      // cv += order*cv_dim; // Chuck-n-Dale 21 Sep bug fix change cv_dim to cv_stride
      cv += order*cv_stride;
      k = order; while (--k) { 
        cv1 = cv;
        cv0 = cv1 - cv_stride;
        k1 = knots+k;
        i = k; while(i--) {
          alpha1 = dt/(*k1-- - t0); 
          alpha0 = 1.0 - alpha1;
          cv0 -= cv_inc;
          cv1 -= cv_inc;
          j = cv_dim;
          while (j--) {cv0--; cv1--; *cv1 = *cv0 * alpha0 + *cv1 * alpha1;} 
        }
      }
    }
  }
  else {
    /* if left end of span is fully multiple and t = start knot,
     * then we're done.
     */
    if (t == t0 && t0 == knots[0])
      return true;
    /* if right end of span is fully multiple, save time */
    if (side == 2)
      t1 = mult_k;
    else if (t1 == knots[2*degree-1])
      side = 2;
    else {
      side = 1;
      if (degree > 21)
        delta_t = free_delta_t = (double*)onmalloc(degree*sizeof(*delta_t));
    }
    knots += degree;
    if (side == 1) {
      /* variable right end knots
       * delta_t = {knot[order-1] - t, knot[order] -  t, .. knot[2*order-3] - t}
       */
      k1=knots; k = degree; while (k--) *delta_t++ = *k1++ - t; delta_t -= degree;
      k = order; while (--k) {
        cv0 = cv;
        cv1 = cv0 + cv_stride;
        k1 = knots;
        k0 = k1 - k;
        i = k; while(i--) {
          alpha0 = *delta_t++/(*k1++ - *k0++);
          alpha1 = 1.0 - alpha0;
          j = cv_dim;
          while(j--) {*cv0 = *cv0 * alpha0 + *cv1 * alpha1; cv0++; cv1++;}
          cv0 += cv_inc;
          cv1 += cv_inc;
        }
        delta_t -= k;
      }
    }
    else {
      /* all right end knots = t1  delta_t = t1 - t */
      dt = t1 - t;
      k = order; while (--k) {
        cv0 = cv;
        cv1 = cv0 + cv_stride;
        k0 = knots - k;         /* *knots = input_knots[d]       */ 
        i = k; while(i--) {
          alpha0 = dt/(t1 - *k0++);
          alpha1 = 1.0 - alpha0;
          j = cv_dim;
          while(j--) {*cv0 = *cv0 * alpha0 + *cv1 * alpha1; cv0++; cv1++;}
          cv0 += cv_inc;
          cv1 += cv_inc;
        }
      }
    }
  }
    
  if (free_delta_t)
    onfree(free_delta_t);

  return true;
}


bool ON_EvaluateNurbsBlossom(int cvdim,
                             int order, 
                             int cv_stride,
                             const double *CV,//size cv_stride*order
                             const double *knot, //nondecreasing, size 2*(order-1)
                             // knot[order-2] != knot[order-1]
                             const double *t, //input parameters size order-1
                             double* P
                            )

{

  if (!CV || !t || !knot) return false;
  if (cv_stride < cvdim) return false;
  const double* cv;
  int degree = order-1;
  double workspace[32];
  double* space = workspace;
  double* free_space = NULL;
  if (order > 32){
    free_space = (double*)onmalloc(order*sizeof(double));
    space = free_space;
  }

  int i, j, k;

  for (i=1; i<2*degree; i++){
    if (knot[i] - knot[i-1] < 0.0) return false;
  }

  if (knot[degree] - knot[degree-1] < ON_EPSILON)
    return false;

  for (i=0; i<cvdim; i++){ //do one coordinate at a time

    //load in cvs.
    cv = CV+i;
    for (j=0; j<order; j++){
      space[j] = *cv; //d0j
      cv+=cv_stride;
    }

    for (j=1; j<order; j++){
      for (k=j; k<order; k++){//djk
        space[k-j] = (knot[degree+k-j]-t[j-1])/(knot[degree+k-j]-knot[k-1])*space[k-j] +
          (t[j-1]-knot[k-1])/(knot[degree+k-j]-knot[k-1])*space[k-j+1];
      }
    }

    P[i] = space[0];
  }

  if (free_space) onfree((void*)free_space);
  return true;
}


void ON_ConvertNurbSpanToBezier(int cvdim, int order, 
                                int cvstride, double *cv, 
                                const double *knot, double t0, double t1)
/* 
 * Convert a Nurb span to a Bezier
 *
 * INPUT:
 *   cvdim
 *      (>= 1) 
 *   order
 *      (>= 2)
 *   cv
 *      array of order*cvdim doubles that define the Nurb
 *      span's control vertices
 *   knot
 *      array of (2*order - 2) doubles the define the Nurb
 *      span's knot vector.  The array should satisfiy
 *      knot[0] <= ... <= knot[order-2] < knot[order-1] 
 *      <= ... <= knot[2*order-3]
 *   t0, t1
 *      The portion of the Nurb span to convert to a Bezier.
 *      (t0 < t1)
 *
 * OUTPUT:
 *   TL_ConvNurbToBezier()
 *       0: successful
 *      -1: failure
 *   cv
 *      Control vertices for the Bezier.
 *
 * COMMENTS:
 *   If you want to convert the entire
 *   span to a Bezier, set t0 = knots[order-2] and
 *   t1 = knots[order-1].  
 *
 *   If you want to extend the left end of the span a bit,
 *   set t0 = knots[order-2] - a_bit and t1 = knots[order-1].
 *
 *   If you want to extend the right end of the span a bit,
 *   set t0 = knots[order-2] and t1 = knots[order-1] + a_bit.
 *
 * EXAMPLE:
 *   // ...
 *
 * REFERENCE:
 *   BOHM-01, Page 7.
 *
 * RELATED FUNCTIONS:
 *   TL_EvdeBoor(), TL_ConvertBezierToPolynomial
 */
{
  ON_EvaluateNurbsDeBoor(cvdim,order,cvstride,cv,knot, 1, 0.0, t0);
  ON_EvaluateNurbsDeBoor(cvdim,order,cvstride,cv,knot,-2,  t0, t1);
}


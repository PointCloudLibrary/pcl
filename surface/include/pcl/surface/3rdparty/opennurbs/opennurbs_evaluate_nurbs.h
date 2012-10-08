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

#if !defined(ON_EVALUATE_NURBS_INC_)
#define ON_EVALUATE_NURBS_INC_

ON_DECL
bool ON_IncreaseBezierDegree(
        int,    // dimension 
        ON_BOOL32,   // true if Bezier is rational
        int,    // order (>=2)
        int,    // cv_stride (>=dim+1)
        double* // cv[(order+1)*cv_stride] array
        );

ON_DECL
bool ON_RemoveBezierSingAt0( // input bezier is rational with 0/0 at start
        int,    // dimension 
        int,    // order (>=2)
        int,    // cv_stride (>=dim+1)
        double* // cv[order*cv_stride] array
        );

ON_DECL
bool ON_RemoveBezierSingAt1( // input bezier is rational with 0/0 at end
        int,    // dimension 
        int,    // order (>=2)
        int,    // cv_stride (>=dim+1)
        double* // cv[order*cv_stride] array
        );

ON_DECL
double ON_EvaluateBernsteinBasis( // returns (i choose d)*(1-t)^(d-i)*t^i
        int, // degree, 
        int, // 0 <= i <= degree
        double //  t
        );

ON_DECL
void ON_EvaluatedeCasteljau(
        int,     //  dim
        int,     //  order
        int,     //  side <= 0  return left side of bezier in cv array
                 //       >  0  return right side of bezier in cv array
        int,     //  cv_stride
        double*, //  cv
        double   //  t 0 <= t <= 1
        );

ON_DECL
bool ON_EvaluateBezier(
        int,            // dimension
        ON_BOOL32,           // true if Bezier is rational
        int,            // order (>=2)
        int,            // cv_stride >= (is_rat)?dim+1:dim
        const double*,  // cv[order*cv_stride] array
        double, double, // t0,t1 = domain of bezier
        int,            // number of derivatives to compute (>=0)
        double,         // evaluation parameter
        int,            // v_stride (>=dimension)
        double*         // v[(der_count+1)*v_stride] array
        );
                                      
ON_DECL
bool ON_EvaluateNurbsBasis( 
                  int,           // order (>=1)
                  const double*, // knot[] array of 2*(order-1) knots
                  double,        // evaluation parameter
                  double*        // basis_values[] array of length order*order
                  );

ON_DECL
bool ON_EvaluateNurbsBasisDerivatives( 
                  int,           // order (>=1)
                  const double*, // knot[] array of 2*(order-1) knots
                  int,           // number of derivatives
                  double*        // basis_values[] array of length order*order
                  );                      




/*
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
        int,           // dimension
        ON_BOOL32,          // true if NURBS is rational
        int,           // order
        const double*, // knot[] array of (2*order-2) doubles
        int,           // cv_stride
        const double*, // cv[] array of order*cv_stride  doubles
        int,           // number of derivatives to compute (>=0)
        double,        // evaluation parameter
        int,           // answer_stride (>=dimension)
        double*        // answer[] array of length (ndir+1)*answer_stride
*/


ON_DECL

/*
Description:
  Evaluate a NURBS curve span.
Parameters:
  dim - [in]
    dimension (> 0).
  is_rat - [in] 
    true or false.
  order - [in]
    order=degree+1 (order>=2)
  knot - [in] NURBS knot vector.
    NURBS knot vector with 2*(order-1) knots, knot[order-2] != knot[order-1]
  cv_stride - [in]
  cv - [in]
    For 0 <= i < order the i-th control vertex is

          cv[n],...,cv[n+(is_rat?dim:dim+1)], 

    where n = i*cv_stride.  If is_rat is true the cv is
    in homogeneous form.
  der_count - [in] 
    number of derivatives to evaluate (>=0)
  t - [in] 
    evaluation parameter
  v_stride - [in]
  v - [out]
    An array of length v_stride*(der_count+1). The evaluation 
    results are returned in this array.

              P = v[0],...,v[m_dim-1]
              Dt = v[v_stride],...
              Dtt = v[2*v_stride],...
              ...

            In general, Dt^i returned in v[n],...,v[n+m_dim-1], where

              n = v_stride*i.
    
Returns:
  True if successful.
See Also:
  ON_NurbsCurve::Evaluate
  ON_EvaluateNurbsSurfaceSpan
  ON_EvaluateNurbsCageSpan
*/
bool ON_EvaluateNurbsSpan( 
        int dim,
        int is_rat,
        int order,
        const double* knot,
        int cv_stride,
        const double* cv,
        int der_count,
        double t,
        int v_stride,
        double* v
        );

/*
Description:
  Evaluate a NURBS surface bispan.
Parameters:
  dim - [in] >0
  is_rat - [in] true of false
  order0 - [in] >= 2
  order1 - [in] >= 2
  knot0 - [in] 
    NURBS knot vector with 2*(order0-1) knots, knot0[order0-2] != knot0[order0-1]
  knot1 - [in]
    NURBS knot vector with 2*(order1-1) knots, knot1[order1-2] != knot1[order1-1]
  cv_stride0 - [in]
  cv_stride1 - [in]
  cv - [in]
    For 0 <= i < order0 and  0 <= j < order1, the (i,j) control vertex is

          cv[n],...,cv[n+(is_rat?dim:dim+1)], 

    where n = i*cv_stride0 + j*cv_stride1.  If is_rat is true the cv is
    in homogeneous form.
   
  der_count - [in] (>=0)
  s - [in]
  t - [in] (s,t) is the evaluation parameter
  v_stride - [in] (>=dim)
  v - [out] An array of length v_stride*(der_count+1)*(der_count+2)/2.
            The evaluation results are stored in this array.

              P = v[0],...,v[m_dim-1]
              Ds = v[v_stride],...
              Dt = v[2*v_stride],...
              Dss = v[3*v_stride],...
              Dst = v[4*v_stride],...
              Dtt = v[5*v_stride],...

            In general, Ds^i Dt^j is returned in v[n],...,v[n+m_dim-1], where

              n = v_stride*( (i+j)*(i+j+1)/2 + j).

Returns:
  True if succcessful.
See Also:
  ON_NurbsSurface::Evaluate
  ON_EvaluateNurbsSpan
  ON_EvaluateNurbsCageSpan
*/
ON_DECL
bool ON_EvaluateNurbsSurfaceSpan(
        int dim,
        int is_rat,
        int order0, 
        int order1,
        const double* knot0,
        const double* knot1,
        int cv_stride0,
        int cv_stride1,
        const double* cv,
        int der_count,
        double s,
        double t,
        int v_stride,
        double* v
        );
            


/*
Description:
  Evaluate a NURBS cage trispan.
Parameters:
  dim - [in] >0
  is_rat - [in] true of false
  order0 - [in] >= 2
  order1 - [in] >= 2
  order2 - [in] >= 2
  knot0 - [in] 
    NURBS knot vector with 2*(order0-1) knots, knot0[order0-2] != knot0[order0-1]
  knot1 - [in]
    NURBS knot vector with 2*(order1-1) knots, knot1[order1-2] != knot1[order1-1]
  knot2 - [in]
    NURBS knot vector with 2*(order1-1) knots, knot2[order2-2] != knot2[order2-1]
  cv_stride0 - [in]
  cv_stride1 - [in]
  cv_stride2 - [in]
  cv - [in]
    For 0 <= i < order0, 0 <= j < order1, and 0 <= k < order2, 
    the (i,j,k)-th control vertex is

          cv[n],...,cv[n+(is_rat?dim:dim+1)], 

    where n = i*cv_stride0 + j*cv_stride1 *k*cv_stride2.  
    If is_rat is true the cv is in homogeneous form.
   
  der_count - [in] (>=0)
  r - [in]
  s - [in]
  t - [in] (r,s,t) is the evaluation parameter
  v_stride - [in] (>=dim)
  v - [out] An array of length v_stride*(der_count+1)*(der_count+2)*(der_count+3)/6.
            The evaluation results are stored in this array.

              P = v[0],...,v[m_dim-1]
              Dr = v[v_stride],...
              Ds = v[2*v_stride],...
              Dt = v[3*v_stride],...
              Drr = v[4*v_stride],...
              Drs = v[5*v_stride],...
              Drt = v[6*v_stride],...
              Dss = v[7*v_stride],...
              Dst = v[8*v_stride],...
              Dtt = v[9*v_stride],...

            In general, Dr^i Ds^j Dt^k is returned in v[n],...,v[n+dim-1], where

               d = (i+j+k)
               n = v_stride*( d*(d+1)*(d+2)/6 + (j+k)*(j+k+1)/2 + k) 

Returns:
  True if succcessful.
See Also:
  ON_NurbsCage::Evaluate
  ON_EvaluateNurbsSpan
  ON_EvaluateNurbsSurfaceSpan
*/
ON_DECL
bool ON_EvaluateNurbsCageSpan(
        int dim,
        int is_rat,
        int order0, int order1, int order2,
        const double* knot0,
        const double* knot1,
        const double* knot2,
        int cv_stride0, int cv_stride1, int cv_stride2,
        const double* cv,
        int der_count,
        double t0, double t1, double t2,
        int v_stride, 
        double* v
        );


ON_DECL
bool ON_EvaluateNurbsDeBoor( // for expert users only - no support available
        int,            // cv_dim ( dim+1 for rational cvs )
        int,            // order (>=2)
        int,            // cv_stride (>=cv_dim)
        double*,        // cv array - values changed to result of applying De Boor's algorithm
        const double*,  // knot array
        int,            // side,
                        //    -1  return left side of B-spline span in cv array
                        //    +1  return right side of B-spline span in cv array
                        //    -2  return left side of B-spline span in cv array
                        //        Ignore values of knots[0,...,order-3] and assume
                        //        left end of span has a fully multiple knot with
                        //        value "mult_k".
                        //    +2  return right side of B-spline span in cv array
                        //        Ignore values of knots[order,...,2*order-2] and
                        //        assume right end of span has a fully multiple
                        //        knot with value "mult_k".
        double,         // mult_k - used when side is +2 or -2.  See above for usage.
        double          // t
                        //    If side < 0, then the cv's for the portion of the NURB span to
                        //    the LEFT of t are computed.  If side > 0, then the cv's for the
                        //    portion the span to the RIGHT of t are computed.  The following
                        //    table summarizes the restrictions on t:
                        //
                        //     value of side         condition t must satisfy
                        //        -2                    mult_k < t and mult_k < knots[order-1]
                        //        -1                    knots[order-2] < t
                        //        +1                    t < knots[order-1]
                        //        +2                    t < mult_k and knots[order-2] < mult_k
        );


ON_DECL
bool ON_EvaluateNurbsBlossom(int, // cvdim,
                             int, // order, 
                             int, // cv_stride,
                             const double*, //CV, size cv_stride*order
                             const double*, //knot, nondecreasing, size 2*(order-1)
                             // knot[order-2] != knot[order-1]
                             const double*, //t, input parameters size order-1
                             double* // P

                             // DeBoor algorithm with different input at each step.
                             // returns false for bad input.
                            );


ON_DECL
void ON_ConvertNurbSpanToBezier(
        int,       // cvdim (dim+1 for rational curves)
        int,       // order, 
        int,       // cvstride (>=cvdim)
        double*,   // cv array - input has NURBS cvs, output has Bezier cvs
        const double*, // (2*order-2) knots for the NURBS span
        double,        // t0, NURBS span parameter of start point
        double         // t1, NURBS span parameter of end point
        );
#endif

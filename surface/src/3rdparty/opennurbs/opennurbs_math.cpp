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

/*
Description:
  Test math library functions.
Parameters:
  function_index - [in]  Determines which math library function is called.

           1:    z = x+y
           2:    z = x-y
           3:    z = x*y
           4:    z = x/y
           5:    z = fabs(x)
           6:    z = exp(x)
           7:    z = log(x)
           8:    z = log10(x)
           9:    z = frexp(x)
          10:    z = pow(x,y)
          11:    z = sqrt(x)
          12:    z = sin(x)
          13:    z = cos(x)
          14:    z = tan(x)
          15:    z = sinh(x)
          16:    z = cosh(x)
          17:    z = tanh(x)
          18:    z = asin(x)
          19:    z = acos(x)
          20:    z = atan(x)
          21:    z = atan2(y,x)
          22:    z = fmod(x,y)
          23:    z = modf(x,&y)

  double x - [in]
  double y - [in]
Returns:
  Returns the "z" value listed in the function_index parameter
  description.
Remarks:
  This function is used to test the results of class floating
  point functions.  It is primarily used to see what happens
  when opennurbs is used as a DLL and illegal operations are
  performed.
*/

double ON_TestMathFunction( int function_index, double x, double y )
{
  // This function is used to test the results of performing operations.
  //
  // module          function
  // opennurbs.dll   ON_TestMathFunction
  // tl.dll          TL_TestMathFunction
  // rhino.exe       Rhino_TestMathFunction

  double z = ON_UNSET_VALUE;
  int i;

  switch(function_index)
  {
  case 1: // addition
    z = x+y;
    break;
  case 2: // subtraction
    z = x-y;
    break;
  case 3: // multiplication
    z = x*y;
    break;
  case 4: // division
    z = x/y;
    break;
  case 5: // absolute value
    z = fabs(x);
    break;
  case 6: // exp
    z = exp(x);
    break;
  case 7: // log
    z = log(x);
    break;
  case 8: // log10
    z = log10(x);
    break;
  case 9: // frexp
    z = frexp(x,&i);
    break;
  case 10: // pow
    z = pow(x,y);
    break;
  case 11: // square root
    z = sqrt(x);
    break;
  case 12: // sine
    z = sin(x);
    break;
  case 13: // cosine
    z = cos(x);
    break;
  case 14: // tangent
    z = tan(x);
    break;
  case 15: // hyperbolic sine
    z = sinh(x);
    break;
  case 16: // hyperbolic cosine
    z = cosh(x);
    break;
  case 17: // hyperbolic tangent
    z = tanh(x);
    break;
  case 18: // arcsine
    z = asin(x);
    break;
  case 19: // arccosine
    z = acos(x);
    break;
  case 20: // arctangent
    z = atan(x);
    break;
  case 21: // arctangent
    z = atan2(y,x);
    break;
  case 22:
    z = fmod(x,y);
      break;
  case 23:
    z = modf(x,&y);
      break;
  default:
    z = 0.0;
    break;
  }

  return z;
}

double ON_ArrayDotProduct(int dim, const double* A, const double* B)
{
  double AoB;
  // do low dimensional cases on one line so we get 80 bit
  // intermediate precision in optimized mode.
  if (dim==1) return (A[0]*B[0]);
  if (dim==2) return (A[0]*B[0] + A[1]*B[1]);
  if (dim==3) return (A[0]*B[0] + A[1]*B[1] + A[2]*B[2]);
  if (dim==4) return (A[0]*B[0] + A[1]*B[1] + A[2]*B[2] +A[3]*B[3]);
  AoB = 0.0;
  while (dim--) AoB += *A++ * *B++;
  return AoB;
}
  

double 
ON_ArrayDotDifference( int dim, const double* A, const double* B, const double* C )
{
  // returns A o ( B - C )
  double AoBminusC; // low dim cases inline for better optimization
  if (dim==1) return (A[0]*(B[0] - C[0]));
  if (dim==2) return (A[0]*(B[0] - C[0]) + A[1]*(B[1] - C[1]));
  if (dim==3) return (A[0]*(B[0] - C[0]) + A[1]*(B[1] - C[1]) + A[2]*(B[2] - C[2]));
  AoBminusC = 0.0;
  while (dim--) AoBminusC += *A++ * (*B++ - *C++);
  return AoBminusC;
}


double ON_ArrayDistance(int dim, const double *A, const double *B)
{
  // returns sqrt((A-B)o(A-B))
  double a, b, c, len;
  switch(dim) {
  case 1:
    len = fabs(*B - *A); 
    break;
  case 2:
    a = fabs(*B++ - *A++); b = fabs(*B - *A);
    if (a > b) 
      {b /= a; len = a*sqrt(1.0+b*b);}
    else if (b > a) 
      {a /= b; len = b*sqrt(1.0+a*a);}
    else
      len = a*ON_SQRT2;
    break;
  case 3:
    a = fabs(*B++ - *A++); b = fabs(*B++ - *A++); c = fabs(*B - *A);
    if (a >= b) {
      if (a >= c) {
        if      (a == 0.0)         len = 0.0;
        else if (a == b && a == c) len = a*ON_SQRT3;
        else      {b /= a; c /= a; len = a*sqrt(1.0 + (b*b + c*c));}
      }
      else 
        {a /= c; b /= c; len = c*sqrt(1.0 + (a*a + b*b));}
    }
    else if (b >= c) 
      {a /= b; c /= b; len = b*sqrt(1.0 + (a*a + c*c));}
    else 
      {b /= c; a /= c; len = c*sqrt(1.0 + (a*a + b*b));}
    break;
  default:
    len = 0.0;
    while (dim--) {a = *B++ - *A++; len += a*a;}
    len = sqrt(len);
    break;
  }
  return len;
}


double ON_ArrayDistanceSquared(int dim, const double* A, const double* B)
{
  // returns (A-B)o(A-B)
  double x, dist_sq = 0.0;
  while (dim--) {
    x = (*B++) - (*A++); 
    dist_sq += x*x;
  }
  return dist_sq;
}


double ON_ArrayMagnitude(int dim, const double* A)
{
  double a, b, c, len;
  switch(dim) {
  case 1:
    len = fabs(*A); 
    break;
  case 2:
    a = fabs(*A++); b = fabs(*A);
    if (a > b) 
      {b /= a; len = a*sqrt(1.0+b*b);}
    else if (b > a) 
      {a /= b; len = b*sqrt(1.0+a*a);}
    else
      len = a*ON_SQRT2;
    break;
  case 3:
    a = fabs(*A++); b = fabs(*A++); c = fabs(*A);
    if (a >= b) {
      if (a >= c) {
        if (a == b && a == c) 
          len = a*ON_SQRT3;
        else
          {b /= a; c /= a; len = a*sqrt(1.0 + (b*b + c*c));}
      }
      else 
        {a /= c; b /= c; len = c*sqrt(1.0 + (a*a + b*b));}
    }
    else if (b >= c) 
      {a /= b; c /= b; len = b*sqrt(1.0 + (a*a + c*c));}
    else 
      {b /= c; a /= c; len = c*sqrt(1.0 + (a*a + b*b));}
    break;
  default:
    len = 0.0;
    while (dim--) {a = *A++; len += a*a;}
    len = sqrt(len);
    break;
  }
  return len;
}


double ON_ArrayMagnitudeSquared(int dim, const double* A)
{
  double x, len_sq=0.0;
  while (dim--) {
    x = *A++;
    len_sq += x*x;
  }
  return len_sq;
}



void 
ON_ArrayScale( int dim, double s, const double* A, double* sA )
{
  if ( dim > 0 ) {
    while ( dim-- )
      *sA++ = s * *A++;
  }
}


void
ON_Array_aA_plus_B( int dim, double a, const double* A, const double* B, double* aA_plus_B )
{
  if ( dim > 0 ) {
    while ( dim-- )
      *aA_plus_B++ = a * *A++ + *B++;
  }
}


float 
ON_ArrayDotProduct( int dim, const float* A, const float* B )
{
  float d = 0.0;
  if ( dim > 0 ) {
    while(dim--)
      d += *A++ * *B++;
  }
  return d;
}


void
ON_ArrayScale( int dim, float s, const float* A, float* sA )
{
  if ( dim > 0 ) {
    while ( dim-- )
      *sA++ = s * *A++;
  }
}


void
ON_Array_aA_plus_B( int dim, float a, const float* A, const float* B, float* aA_plus_B )
{
  if ( dim > 0 ) {
    while ( dim-- )
      *aA_plus_B++ = a * *A++ + *B++;
  }
}


int 
ON_DecomposeVector(
        const ON_3dVector& V,
        const ON_3dVector& A,
        const ON_3dVector& B,
        double* x, double* y
        )
{
  int rank;
  double pr;
  const double AoV = A*V;
  const double BoV = B*V;
  const double AoA = A*A;
  const double AoB = A*B;
  const double BoB = B*B;
  rank =  ON_Solve2x2(  AoA, AoB, AoB, BoB, AoV, BoV, x, y, &pr );
  return (rank==2)?true:false;
}


ON_BOOL32 
ON_EvJacobian( double ds_o_ds, double ds_o_dt, double dt_o_dt,
                    double* det_addr )
/* Carefully compute the Jacobian determinant
 *
 * INPUT:
 *   ds_o_ds, ds_o_dt, dt_o_dt
 *      Dot products of the first partial derivatives
 *   det_addr
 *      address of an unused double
 * OUTPUT:
 *   *det_addr = ds_o_ds*dt_o_dt - ds_o_dt^2
 *   ON_EvJacobian()
 *       0: successful
 *      -1: failure
 *
 * COMMENTS:
 *      ...
 *
 * EXAMPLE:
 *      // ...
 *
 * REFERENCE:
 *      
 *
 * RELATED FUNCTIONS:
 *      ON_EvBsplineBasis(), ON_EvdeCasteljau(), ON_EvBezier()
 */
{
  ON_BOOL32 rc = false;
  double det, a, b;
  a = ds_o_ds*dt_o_dt;
  b = ds_o_dt*ds_o_dt;
  /* NOTE: a = |Du|^2 * |Dv|^2  and b = (Du o Dv)^2 are always >= 0 */
  det = a - b;
  if (ds_o_ds <= dt_o_dt* ON_EPSILON || dt_o_dt <= ds_o_ds* ON_EPSILON) {
    /* one of the paritals is (numerically) zero with respect
     * to the other partial - value of det is unreliable
     */
    rc = false;
  }
  else if (fabs(det) <= ((a > b) ? a : b)* ON_SQRT_EPSILON) {
    /* Du and Dv are (numerically) (anti) parallel - 
     * value of det is unreliable.
     */
    rc = false;
  }
  else {
    rc = true;
  }
  if (det_addr) *det_addr = det;
  return rc;
}


ON_BOOL32 
ON_EvNormalPartials(
        const ON_3dVector& ds,
        const ON_3dVector& dt,
        const ON_3dVector& dss,
        const ON_3dVector& dst,
        const ON_3dVector& dtt,
        ON_3dVector& ns,
        ON_3dVector& nt
        )
{
  ON_BOOL32 rc = false;
  const double ds_o_ds = ds*ds;
  const double ds_o_dt = ds*dt;
  const double dt_o_dt = dt*dt;

  rc = ON_EvJacobian( ds_o_ds, ds_o_dt, dt_o_dt, NULL );
  if (!rc) 
  {
    // degenerate Jacobian and unit surface normal is not well defined
    ns.Zero();
    nt.Zero();
  }
  else 
  {
    // If V: . -> R^3 is nonzero and C^2 and N = V/|V|, then 
    //
    //          V'       V o V'
    //   N' = -----  -  ------- * V.
    //         |V|       |V|^3
    //
    // When a surface has a non-degenerate Jacobian, V = ds X dt
    // and the derivatives of N may be computed from the first
    // and second partials.
    ON_3dVector V = ON_CrossProduct(ds,dt);
    double len = V.Length();
    double len3 = len*len*len;
    if (len < ON_EPSILON) {
      ns.Zero();
      nt.Zero();
      return false;
    }

    ns.x = dss.y*dt.z - dss.z*dt.y + ds.y*dst.z - ds.z*dst.y;
    ns.y = dss.z*dt.x - dss.x*dt.z + ds.z*dst.x - ds.x*dst.z;
    ns.z = dss.x*dt.y - dss.y*dt.x + ds.x*dst.y - ds.y*dst.x;

    nt.x = dst.y*dt.z - dst.z*dt.y + ds.y*dtt.z - ds.z*dtt.y;
    nt.y = dst.z*dt.x - dst.x*dt.z + ds.z*dtt.x - ds.x*dtt.z;
    nt.z = dst.x*dt.y - dst.y*dt.x + ds.x*dtt.y - ds.y*dtt.x;

    ns = ns/len - ((V*ns)/len3)*V;
    nt = nt/len - ((V*nt)/len3)*V;
  }

  return rc;
}


ON_BOOL32 
ON_Pullback3dVector( // use to pull 3d vector back to surface parameter space
      const ON_3dVector& vector,   // 3d vector
      double distance,              // signed distance from vector location to closet point on surface
                                    // < 0 if point is below with respect to Du x Dv
      const ON_3dVector&  ds,      // surface first partials
      const ON_3dVector&  dt,
      const ON_3dVector&  dss,     // surface 2nd partials
      const ON_3dVector&  dst,     // (used only when dist != 0)
      const ON_3dVector&  dtt, 
      ON_2dVector&  pullback       // pullback
      )
{
  ON_BOOL32 rc = false;
  //int bIsDegenerate = false;
  if (distance != 0.0) {
    ON_3dVector ns, nt;
    rc = ON_EvNormalPartials(ds,dt,dss,dst,dtt,ns,nt);
    if ( rc ) {
      // adjust ds and dt to take account of offset distance
      rc = ON_DecomposeVector( vector, ds + distance*ns, dt + distance*nt, &pullback.x, &pullback.y );
    }
  }
  else {
    rc = ON_DecomposeVector( vector, ds, dt, &pullback.x, &pullback.y );
  }
  return rc;
}


ON_BOOL32 
ON_GetParameterTolerance(
        double t0, double t1, // domain
        double t,          // parameter in domain
        double* tminus, double* tplus// parameter tolerance (tminus, tplus) returned here
        )
{
  const ON_BOOL32 rc = (t0 < t1) ? true : false;
  if ( rc ) {
    if ( t < t0 )
      t = t0;
    else if (t > t1 )
      t = t1;
    double dt = (t1-t0)*8.0* ON_SQRT_EPSILON + (fabs(t0) + fabs(t1))* ON_EPSILON;
    if ( dt >= t1-t0 )
      dt = 0.5*(t1-t0);
    const double tmin = t-dt;
    const double tmax = t+dt;
    if ( tminus )
      *tminus = tmin;
    if ( tplus )
      *tplus = tmax;
  }

  return rc;
}


ON_BOOL32
ON_EvNormal(int limit_dir,
                const ON_3dVector& Du, const ON_3dVector& Dv, 
                const ON_3dVector& Duu, const ON_3dVector& Duv, const ON_3dVector& Dvv, 
                ON_3dVector& N)
{
  const double DuoDu = Du.LengthSquared();
  const double DuoDv = Du*Dv;
  const double DvoDv = Dv.LengthSquared();
  if ( ON_EvJacobian(DuoDu,DuoDv,DvoDv,NULL) ) {
    N = ON_CrossProduct(Du,Dv);
  }
  else {
    /* degenerate jacobian - try to compute normal using limit
     *
     * P,Du,Dv,Duu,Duv,Dvv = srf and partials evaluated at (u0,v0).
     * Su,Sv,Suu,Suv,Svv = partials evaluated at (u,v).
     * Assume that srf : R^2 -> R^3 is analytic near (u0,v0).
     *
     * srf(u0+u,v0+v) = srf(u0,v0) + u*Du + v*Dv 
     *                  + (1/2)*u^2*Duu + u*v*Duv + (1/2)v^2*Dvv
     *                  + cubic and higher order terms.
     *
     * Su X Sv = Du X Dv + u*(Du X Duv + Duu X Dv) + v*(Du X Dvv + Duv X Dv) 
     *           + quadratic and higher order terms
     * 
     * Set 
     * (1) A = (Du X Duv + Duu X Dv), 
     * (2) B = (Du X Dvv + Duv X Dv) and assume
     * Du X Dv = 0.  Then 
     *
     * |Su X Sv|^2 = u^2*AoA + 2uv*AoB + v^2*BoB + cubic and higher order terms
     *
     * If u = a*t, v = b*t and (a^2*AoA + 2ab*AoB + b^2*BoB) != 0, then
     * 
     *        Su X Sv                   a*A + b*B
     * lim   ---------  =  ----------------------------------
     * t->0  |Su X Sv|      sqrt(a^2*AoA + 2ab*AoB + b^2*BoB)
     *
     * All I need is the direction, so I just need to compute a*A + b*B as carefully
     * as possible.  Note that
     * (3)  a*A + b*B = Du X (a*Duv + b*Dvv)  - Dv X (a*Duu + b*Duv).
     * Formaula (3) requires fewer flops than using formulae (1) and (2) to 
     * compute a*A + b*B.  In addition, when |Du| << |Dv| or |Du| >> |Dv|,
     * formula (3) reduces the number of subtractions between numbers of
     * similar size.  Since the (nearly) zero first partial is the most common
     * is more common than the (nearly) (anti) parallel case, I'll use
     * formula (3).  If you're reading this because you're not getting
     * the right answer and you can't find any bugs, you might want to
     * try using formulae (1) and (2).
     *
     * The limit_dir argument determines which direction is used to compute the
     * limit.
     *                  |
     *   limit_dir == 2 |  limit_dir == 1
     *           \      |      /
     *            \     |     /
     *             \    |    /
     *              \   |   /
     *               \  |  /
     *                \ | /
     *                 \|/
     *   ---------------*--------------
     *                 /|\
     *                / | \
     *               /  |  \
     *              /   |   \
     *             /    |    \
     *            /     |     \
     *           /      |      \
     *   limit_dir == 3 |  limit_dir == 4
     *                  |
     *
     */

    double a,b;
    ON_3dVector V, Au, Av;

    switch(limit_dir) {
    case  2: /* from 2nd  quadrant to point */
      a = -1.0; b =  1.0; break;
    case  3: /* from 3rd  quadrant to point */
      a = -1.0; b = -1.0; break;
    case  4: /* from 4rth quadrant to point */
      a =  1.0; b = -1.0; break;
    default: /* from 1rst quadrant to point */
      a =  1.0; b =  1.0; break;
    }

    V = a*Duv + b*Dvv;
    Av.x = Du.y*V.z - Du.z*V.y;
    Av.y = Du.z*V.x - Du.x*V.z;
    Av.z = Du.x*V.y - Du.y*V.x;

    V = a*Duu + b*Duv;
    Au.x = V.y*Dv.z - V.z*Dv.y;
    Au.y = V.z*Dv.x - V.x*Dv.z;
    Au.z = V.x*Dv.y - V.y*Dv.x;

    N = Av + Au;
  }
  
  return N.Unitize();
}

bool ON_EvTangent( 
        const ON_3dVector& D1, // first derivative
        const ON_3dVector& D2, // second derivative
        ON_3dVector& T         // Unit tangent returned here
        )
{
  // Evaluate unit tangent from first and second derivatives
  // T = D1 / |D1|

  bool rc = false;
  double d1 = D1.Length();

  if (d1 == 0.0) 
  {
    // Use L'hopital's rule to show that if the unit tanget
    // exists and the 1rst derivative is zero and the 2nd derivative is
    // nonzero, then the unit tangent is equal to +/-the unitized 
    // 2nd derivative.  The sign is equal to the sign of D1(s) o D2(s)
    // as s approaches the evaluation parameter.
    //
    d1 = D2.Length();
    if (d1 > 0.0) 
    {
      T = D2/d1;
      rc = true;
    }
    else 
    {
      T.Zero();
    }
  }
  else 
  {
    T = D1/d1;
    rc = true;
  }
  return rc;  
}


ON_BOOL32 
ON_EvCurvature( 
        const ON_3dVector& D1, // first derivative
        const ON_3dVector& D2, // second derivative
        ON_3dVector& T,       // Unit tangent returned here
        ON_3dVector& K        // Curvature returned here
        )
{
  // Evaluate unit tangent and curvature from first and second derivatives
  // T = D1 / |D1|
  // K = ( D2 - (D2 o T)*T )/( D1 o D1)

  ON_BOOL32 rc = false;
  double d1 = D1.Length();

  if (d1 == 0.0) 
  {
    // Use L'hopital's rule to show that if the unit tanget
    // exists and the 1rst derivative is zero and the 2nd derivative is
    // nonzero, then the unit tangent is equal to +/-the unitized 
    // 2nd derivative.  The sign is equal to the sign of D1(s) o D2(s)
    // as s approaches the evaluation parameter.
    //
    d1 = D2.Length();
    if (d1 > 0.0) {
      T = D2/d1;
    }
    else 
    {
      T.Zero();
    }
    K.Zero();
  }
  else 
  {
    T = D1/d1;
    const double negD2oT = -D2*T;
    d1 = 1.0/(d1*d1);
    K = d1*( D2 + negD2oT*T );
    rc = true;
  }
  return rc;  
}

bool ON_EvSectionalCurvature( 
    const ON_3dVector& S10, 
    const ON_3dVector& S01,
    const ON_3dVector& S20, 
    const ON_3dVector& S11, 
    const ON_3dVector& S02,
    const ON_3dVector& planeNormal,
    ON_3dVector& K 
    )
{
  ON_3dVector M, D1, M1, D2;
  double a, b, e, pr;
  int rank;

  // Calculates the curvature of the intersection of the surface
  // and plane at the point were the surface partials were evaluated.
  // If D1 and D2 are the derivatives of any parametric curve,
  // then the curvature is
  //
  // K = (D2 - (D2oD1)/(D1oD1)*D1)/(D1oD1)
  //
  // So, the trick is to assign a parameterization to the intersection
  // curve and use the surface partials and plane normal
  // to calculate the curve's derivatives.  For computational reasons, 
  // I'm choosing the parameterization such that
  //
  // D1 = (Su X Sv) X sectionNormal.
  //
  // Then
  //
  // D2 = ((Suu*u' + Suv*v') X Sv  +  Su X (Suv*u' + Svv*v')) X sectionNormal,
  //
  // where the (unknown) intersection curve is srf(u(t),v(t)).  But, we
  // do know D1 can also be computed as
  // 
  // D1 = Su*u' + Sv*v'
  //
  // So, if Su and Sv are linearly independent, then we have
  // (Su X Sv) X sectionNormal = Su*u' + Sv*v' and can solve for u' and v'.
  //


  // M = Su X Sv  (surface normal = M/|M|)
  //M = ON_CrossProduct(S10,S01); 
  M.x = S10.y*S01.z - S01.y*S10.z;
  M.y = S10.z*S01.x - S01.z*S10.x;
  M.z = S10.x*S01.y - S01.x*S10.y;

  // D1 = 1st derivative of the intersection curve
  //D1 = ON_CrossProduct(M,sectionN);
  D1.x = M.y*planeNormal.z - planeNormal.y*M.z;
  D1.y = M.z*planeNormal.x - planeNormal.z*M.x;
  D1.z = M.x*planeNormal.y - planeNormal.x*M.y;


  // D1 is tangent to the surface.  Find a, b so that D1 = a*Su + b*Sv.
  rank = ON_Solve3x2( &S10.x, &S01.x, D1.x, D1.y, D1.z, &a, &b, &e, &pr );
  if ( rank < 2 )
  {
    K.x = 0.0;
    K.y = 0.0;
    K.z = 0.0;
    return false;
  }

  // M1 = derivative of M = (a*Suu + v*Suv) x Sv  +  Su x (a*Suv + b*Svv)
  //M1 = ON_CrossProduct(a*S20 + b*S11, S01) + ON_CrossProduct(S10, a*S11 + b*S02);
  
  D2.x = a*S20.x + b*S11.x;
  D2.y = a*S20.y + b*S11.y;
  D2.z = a*S20.z + b*S11.z;
  M.x = D2.y*S01.z - S01.y*D2.z;
  M.y = D2.z*S01.x - S01.z*D2.x;
  M.z = D2.x*S01.y - S01.x*D2.y;
  
  D2.x = a*S11.x + b*S02.x;
  D2.y = a*S11.y + b*S02.y;
  D2.z = a*S11.z + b*S02.z;
  M.x += S10.y*D2.z - D2.y*S10.z;
  M.y += S10.z*D2.x - D2.z*S10.x;
  M.z += S10.x*D2.y - D2.x*S10.y;

  // D2 = 2nd derivative of the intersection curve
  //D2 = ON_CrossProduct(M1,sectionN);
  D2.x = M.y*planeNormal.z - planeNormal.y*M.z;
  D2.y = M.z*planeNormal.x - planeNormal.z*M.x;
  D2.z = M.x*planeNormal.y - planeNormal.x*M.y;

  a = D1.x*D1.x + D1.y*D1.y + D1.z*D1.z;

  if ( !(a > ON_DBL_MIN) )
  {
    K.x = 0.0;
    K.y = 0.0;
    K.z = 0.0;
    return false;
  }

  a = 1.0/a;
  b = -a*(D2.x*D1.x + D2.y*D1.y + D2.z*D1.z);
  K.x = a*(D2.x + b*D1.x);
  K.y = a*(D2.y + b*D1.y);
  K.z = a*(D2.z + b*D1.z);

  return true;
}

ON_BOOL32 ON_IsContinuous(
  ON::continuity desired_continuity,
  ON_3dPoint Pa, ON_3dVector D1a, ON_3dVector D2a,
  ON_3dPoint Pb, ON_3dVector D1b, ON_3dVector D2b,
  double point_tolerance,
  double d1_tolerance,
  double d2_tolerance,
  double cos_angle_tolerance,
  double curvature_tolerance
  )
{
  ON_3dVector Ta, Tb, Ka, Kb;

  switch( ON::ParametricContinuity(desired_continuity) )
  {
  case ON::unknown_continuity:
    break;

  case ON::C0_continuous:  
  case ON::C0_locus_continuous:  
    if ( !(Pa-Pb).IsTiny(point_tolerance) )
      return false;
    break;

  case ON::C1_continuous:
  case ON::C1_locus_continuous:
    if ( !(Pa-Pb).IsTiny(point_tolerance) || !(D1a-D1b).IsTiny(d1_tolerance) )
      return false;
    break;

  case ON::G1_continuous:
  case ON::G1_locus_continuous:
    Ta = D1a;
    if ( !Ta.Unitize() )
      ON_EvCurvature( D1a, D2a, Ta, Ka );
    Tb = D1b;
    if ( !Tb.Unitize() )
      ON_EvCurvature( D1b, D2b, Tb, Kb );
    if ( !(Pa-Pb).IsTiny(point_tolerance) || Ta*Tb < cos_angle_tolerance )
      return false;
    break;

  case ON::C2_continuous:
  case ON::C2_locus_continuous:
  case ON::Cinfinity_continuous:
    if ( !(Pa-Pb).IsTiny(point_tolerance) || !(D1a-D1b).IsTiny(d1_tolerance) || !(D2a-D2b).IsTiny(d2_tolerance) )
      return false;
    break;

  case ON::G2_continuous:
  case ON::G2_locus_continuous:
  case ON::Gsmooth_continuous:
    ON_EvCurvature( D1a, D2a, Ta, Ka );
    ON_EvCurvature( D1b, D2b, Tb, Kb );
    if ( !(Pa-Pb).IsTiny(point_tolerance) || Ta*Tb < cos_angle_tolerance )
      return false;
    if ( ON::Gsmooth_continuous == desired_continuity )
    {
      if ( !ON_IsGsmoothCurvatureContinuous(Ka,Kb,cos_angle_tolerance,curvature_tolerance) )
        return false;
    }
    else
    {
      if ( !ON_IsG2CurvatureContinuous(Ka,Kb,cos_angle_tolerance,curvature_tolerance) )
        return false;
    }
    break;
  }

  return true;
}

int 
ON_SearchMonotoneArray(const double* array, int length, double t)
/*****************************************************************************
Find interval in an increasing array of doubles
 
INPUT:
  array
    A monotone increasing (array[i] <= array[i+1]) array of length doubles.
  length (>=1)
    number of doubles in array
  t
    parameter
OUTPUT:
  ON_GetdblArrayIndex()
    -1:         t < array[0]
     i:         (0 <= i <= length-2) array[i] <= t < array[i+1]
     length-1:  t == array[length-1]
     length:    t  > array[length-1]
COMMENTS:
  If length < 1 or array is not monotone increasing, you will get a meaningless
  answer and may crash your program.
EXAMPLE:
  // Given a "t", find the knots that define the span used to evaluate a
  // nurb at t; i.e., find "i" so that
  // knot[i] <= ... <= knot[i+order-2] 
  //   <= t < knot[i+order-1] <= ... <= knot[i+2*(order-1)-1]
  i = ON_GetdblArrayIndex(knot+order-2,cv_count-order+2,t);
  if (i < 0) i = 0; else if (i > cv_count - order) i = cv_count - order;
RELATED FUNCTIONS:
  ON_
  ON_
*****************************************************************************/

{                 
  int 
    i, i0, i1;

  length--;

  /* Since t is frequently near the ends and bisection takes the
   * longest near the ends, trap those cases here.
   */
  if (t < array[0]) 
    return -1;
  if (t >= array[length])
    return (t > array[length]) ? length+1 : length;
  if (t < array[1])
    return 0;
  if (t >= array[length-1])
    return (length-1);


  i0 = 0;
  i1 = length;
  while (array[i0] == array[i0+1]) i0++;
  while (array[i1] == array[i1-1]) i1--;
  /* From now on we have 
   *  1.) array[i0] <= t < array[i1] 
   *  2.) i0 <= i < i1.
   * When i0+1 == i1, we have array[i0] <= t < array[i0+1] 
   * and i0 is the answer we seek.
   */
  while (i0+1 < i1) {
    i = (i0+i1)>>1;
    if (t < array[i]) {
      i1 = i;
      while (array[i1] == array[i1-1]) i1--;
    }
    else {
      i0 = i;
      while (array[i0] == array[i0+1]) i0++;
    }
  }

  return i0;
}


double 
ON_BinomialCoefficient(int i, int j)
{
#define MAX_HALF_N 26
static const double bc[((MAX_HALF_N-2)*(MAX_HALF_N-1))/2 + MAX_HALF_N - 2] =
 {15.0, 20.0, 28.0, 56.0, 70.0, 45.0, 120.0, 210.0, 252.0, 66.0,
  220.0, 495.0, 792.0, 924.0, 91.0, 364.0, 1001.0, 2002.0, 3003.0,
  3432.0, 120.0, 560.0, 1820.0, 4368.0, 8008.0, 11440.0, 12870.0,
  153.0, 816.0, 3060.0, 8568.0, 18564.0, 31824.0, 43758.0, 48620.0,
  190.0, 1140.0, 4845.0, 15504.0, 38760.0, 77520.0, 125970.0,
  167960.0, 184756.0, 231.0, 1540.0, 7315.0, 26334.0, 74613.0,
  170544.0, 319770.0, 497420.0, 646646.0, 705432.0, 276.0, 2024.0,
  10626.0, 42504.0, 134596.0, 346104.0, 735471.0, 1307504.0,
  1961256.0, 2496144.0, 2704156.0, 325.0, 2600.0, 14950.0, 65780.0,
  230230.0, 657800.0, 1562275.0, 3124550.0, 5311735.0, 7726160.0,
  9657700.0, 10400600.0, 378.0, 3276.0, 20475.0, 98280.0, 376740.0,
  1184040.0, 3108105.0, 6906900.0, 13123110.0, 21474180.0,
  30421755.0, 37442160.0, 40116600.0, 435.0, 4060.0, 27405.0,
  142506.0, 593775.0, 2035800.0, 5852925.0, 14307150.0, 30045015.0,
  54627300.0, 86493225.0, 119759850.0, 145422675.0, 155117520.0,
  496.0, 4960.0, 35960.0, 201376.0, 906192.0, 3365856.0,
  10518300.0, 28048800.0, 64512240.0, 129024480.0, 225792840.0,
  347373600.0, 471435600.0, 565722720.0, 601080390.0, 561.0,
  5984.0, 46376.0, 278256.0, 1344904.0, 5379616.0, 18156204.0,
  52451256.0, 131128140.0, 286097760.0, 548354040.0, 927983760.0,
  1391975640.0, 1855967520.0, 2203961430.0, 2333606220.0, 630.0,
  7140.0, 58905.0, 376992.0, 1947792.0, 8347680.0, 30260340.0,
  94143280.0, 254186856.0, 600805296.0, 1251677700.0, 2310789600.0,
  3796297200.0, 5567902560.0, 7307872110.0, 8597496600.0,
  9075135300.0, 703.0, 8436.0, 73815.0, 501942.0, 2760681.0,
  12620256.0, 48903492.0, 163011640.0, 472733756.0, 1203322288.0,
  2707475148.0, 5414950296.0, 9669554100.0, 15471286560.0,
  22239974430.0, 28781143380.0, 33578000610.0, 35345263800.0,
  780.0, 9880.0, 91390.0, 658008.0, 3838380.0, 18643560.0,
  76904685.0, 273438880.0, 847660528.0, 2311801440.0, 5586853480.0,
  12033222880.0, 23206929840.0, 40225345056.0, 62852101650.0,
  88732378800.0, 113380261800.0, 131282408400.0, 137846528820.0,
  861.0, 11480.0, 111930.0, 850668.0, 5245786.0, 26978328.0,
  118030185.0, 445891810.0, 1471442973.0, 4280561376.0,
  11058116888.0, 25518731280.0, 52860229080.0, 98672427616.0,
  166509721602.0, 254661927156.0, 353697121050.0, 446775310800.0,
  513791607420.0, 538257874440.0, 946.0, 13244.0, 135751.0,
  1086008.0, 7059052.0, 38320568.0, 177232627.0, 708930508.0,
  2481256778.0, 7669339132.0, 21090682613.0, 51915526432.0,
  114955808528.0, 229911617056.0, 416714805914.0, 686353797976.0,
  1029530696964.0, 1408831480056.0, 1761039350070.0,
  2012616400080.0, 2104098963720.0, 1035.0, 15180.0, 163185.0,
  1370754.0, 9366819.0, 53524680.0, 260932815.0, 1101716330.0,
  4076350421.0, 13340783196.0, 38910617655.0, 101766230790.0,
  239877544005.0, 511738760544.0, 991493848554.0, 1749695026860.0,
  2818953098830.0, 4154246671960.0, 5608233007146.0,
  6943526580276.0, 7890371113950.0, 8233430727600.0, 1128.0,
  17296.0, 194580.0, 1712304.0, 12271512.0, 73629072.0,
  377348994.0, 1677106640.0, 6540715896.0, 22595200368.0,
  69668534468.0, 192928249296.0, 482320623240.0, 1093260079344.0,
  2254848913647.0, 4244421484512.0, 7309837001104.0,
  11541847896480.0, 16735679449896.0, 22314239266528.0,
  27385657281648.0, 30957699535776.0, 32247603683100.0, 1225.0,
  19600.0, 230300.0, 2118760.0, 15890700.0, 99884400.0,
  536878650.0, 2505433700.0, 10272278170.0, 37353738800.0,
  121399651100.0, 354860518600.0, 937845656300.0, 2250829575120.0,
  4923689695575.0, 9847379391150.0, 18053528883775.0,
  30405943383200.0, 47129212243960.0, 67327446062800.0,
  88749815264600.0, 108043253365600.0, 121548660036300.0,
  126410606437752.0, 1326.0, 22100.0, 270725.0, 2598960.0,
  20358520.0, 133784560.0, 752538150.0, 3679075400.0,
  15820024220.0, 60403728840.0, 206379406870.0, 635013559600.0,
  1768966344600.0, 4481381406320.0, 10363194502115.0,
  21945588357420.0, 42671977361650.0, 76360380541900.0,
  125994627894135.0, 191991813933920.0, 270533919634160.0,
  352870329957600.0, 426384982032100.0, 477551179875952.0,
  495918532948104.0};

  int n, half_n, bc_i;

  if (i  < 0 || j  < 0) return  0.0;
  if (0 == i || 0 == j) return  1.0;
  n = i+j;
  if (1 == i || 1 == j) return (double)n;
  if (4 == n)           return  6.0;
  if (5 == n)           return 10.0;

  if (n%2)
    return ON_BinomialCoefficient(i-1,j)+ON_BinomialCoefficient(i,j-1);

  half_n = n >> 1;
  if (half_n > MAX_HALF_N)
    return ON_BinomialCoefficient(i-1,j)+ON_BinomialCoefficient(i,j-1);
  if (i > half_n)
    i = n - i;
  /* at this point we have n even,
   * MAX_HALF_N*2 >= n >= 6 and 1 < i <= n/2
   * and we grab the answer from the bc[] table.
   */
  half_n -= 2;
  bc_i = ((half_n*(half_n+1))>>1) + i - 3;
  return bc[bc_i];

#undef MAX_HALF_N
}

ON_DECL
double ON_TrinomialCoefficient( 
          int i,
          int j,
          int k
          )
{
  return (ON_BinomialCoefficient(i,j+k)*ON_BinomialCoefficient(j,k));
}


ON_BOOL32 
ON_IsValidPointList(
       int dim,
       int is_rat,
       int count,
       int stride,
       const float* p
       )
{
  return ( dim > 0 && stride >= (is_rat?(dim+1):dim) && count >= 0 && p != NULL ) ? true : false;
}


ON_BOOL32 
ON_IsValidPointList(
       int dim,
       int is_rat,
       int count,
       int stride,
       const double* p
       )
{
  return ( dim > 0 && stride >= (is_rat?(dim+1):dim) && count >= 0 && p != NULL ) ? true : false;
}


ON_BOOL32 
ON_IsValidPointGrid(
        int dim,
        ON_BOOL32 is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        const double* p
        )
{
  if ( dim < 1 || point_count0 < 1 || point_count1 < 1 || p == NULL )
    return false;
  if ( is_rat )
    dim++;
  if ( point_stride0 < dim || point_stride1 < dim )
    return false;
  if ( point_stride0 <= point_stride1 ) {
    if ( point_stride1 < point_stride0*point_count0 )
      return false;
  }
  else {
    if ( point_stride0 < point_stride1*point_count1 )
      return false;
  }
  return true;
}


bool ON_ReversePointList(
       int dim,
       int is_rat,
       int count,
       int stride,
       double* p
       )
{
  if ( !ON_IsValidPointList(dim,is_rat,count,stride,p) )
    return false;
  if ( is_rat )
    dim++;
  if ( count <= 1 )
    return true;
  const size_t ele_size = dim*sizeof(*p);
  void* t = onmalloc(ele_size);
  int i, j;
  for ( i = 0, j = (count-1)*stride; i < j; i += stride, j -= stride ) {
    memcpy( t,   p+i, ele_size );
    memcpy( p+i, p+j, ele_size );
    memcpy( p+j, t,   ele_size );
  }
  onfree(t);
  return true;
}


ON_BOOL32 
ON_ReversePointGrid(
        int dim,
        ON_BOOL32 is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        double* p,
        int dir
        )
{
  ON_BOOL32 rc = false;
  if ( !dir ) {
    rc = ON_ReversePointGrid( dim, is_rat, point_count1, point_count0, point_stride1, point_stride0, p, 1 );
  }
  else {
    int i;
    for ( i = 0; i < point_count0; i++ ) {
      if ( !ON_ReversePointList( dim, is_rat, point_count1, point_stride1, p + i*point_stride0 ) ) {
        rc = false;
        break;
      }
      else if (!i) {
        rc = true;
      }
    }
  }
  return rc;
}


bool 
ON_SwapPointListCoordinates( int count, int stride, float* p,
                                   int i, int j )
{
  float t;
  int k;
  if ( !ON_IsValidPointList(stride,0,count,stride,p) )
    return false;
  if ( i < 0 || j < 0 || i >= stride || j >= stride )
    return false;
  if ( i == j || count == 0 )
    return true;
  for ( k = 0; k < count; k++, p += stride ) {
    t = p[i]; 
    p[i] = p[j];
    p[j] = t;
  }
  return true;
}


bool 
ON_SwapPointListCoordinates( int count, int stride, double* p,
                                             int i, int j )
{
  double t;
  int k;
  if ( !ON_IsValidPointList(stride,0,count,stride,p) )
    return false;
  if ( i < 0 || j < 0 || i >= stride || j >= stride )
    return false;
  if ( i == j || count == 0 )
    return true;
  for ( k = 0; k < count; k++, p += stride ) {
    t = p[i]; 
    p[i] = p[j];
    p[j] = t;
  }
  return true;
}


ON_BOOL32 
ON_SwapPointGridCoordinates(
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        double* p,
        int i, int j // coordinates to swap
        )
{
  ON_BOOL32 rc = false;
  if ( p ) {
    double t;
    int k, m;
    double* pk;
    for ( k = 0; k < point_count0; k++ ) {
      pk = p + k*point_stride0;
      for ( m = 0; m < point_count1; m++ ) {
        t = pk[i]; pk[i] = pk[j]; pk[j] = t;
        pk += point_stride1;
      }
    }
    rc = true;
  }
  return rc;
}


bool 
ON_TransformPointList(
                  int dim, int is_rat, int count, 
                  int stride, float* point,
                  const ON_Xform& xform
                  )
{
  bool rc = true;
  double x, y, z, w;

  if ( !ON_IsValidPointList( dim, is_rat, count, stride, point ) )
    return false;
  if ( xform.m_xform == NULL )
    return false;
  if (count == 0)
    return true;

  if (is_rat) {
    switch(dim) {
    case 1:
      while(count--) {
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][3]*point[1];
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][3]*point[1];
				point[0] = (float)x; point[1] = (float)w;
				point += stride;
      }
      break;
    case 2:
      while(count--) {
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][3]*point[2];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][3]*point[2];
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][3]*point[2];
				point[0] = (float)x; point[1] = (float)y; point[2] = (float)w;
				point += stride;
      }
      break;
    default: // dim >= 3
      while(count--) {
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][2]*point[2] + xform.m_xform[0][3]*point[dim];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][2]*point[2] + xform.m_xform[1][3]*point[dim];
				z = xform.m_xform[2][0]*point[0] + xform.m_xform[2][1]*point[1] + xform.m_xform[2][2]*point[2] + xform.m_xform[2][3]*point[dim];
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][2]*point[2] + xform.m_xform[3][3]*point[dim];
				point[0] = (float)x; point[1] = (float)y; point[2] = (float)z; point[dim] = (float)w;
				point += stride;
      }
      break;
    }
  }
  else {
    switch(dim) {
    case 1:
      while(count--) {
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][3];
        if (w==0.0) {
          rc = false;
          w = 1.0;
        }
        else
				  w = 1.0/w;
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][3];
				point[0] = (float)(w*x);
				point += stride;
      }
      break;
    case 2:
      while(count--) {
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][3];
        if (w==0.0) {
          rc = false;
          w = 1.0;
        }
        else
				  w = 1.0/w;
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][3];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][3];
				point[0] = (float)(w*x); point[1] = (float)(w*y);
				point += stride;
      }
      break;
    default: // dim = 3
      while(count--) {
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][2]*point[2] + xform.m_xform[3][3];
        if (w==0.0)  {
          rc = false;
          w = 1.0;
        }
        else
				  w = 1.0/w;
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][2]*point[2] + xform.m_xform[0][3];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][2]*point[2] + xform.m_xform[1][3];
				z = xform.m_xform[2][0]*point[0] + xform.m_xform[2][1]*point[1] + xform.m_xform[2][2]*point[2] + xform.m_xform[2][3];
				point[0] = (float)(w*x); point[1] = (float)(w*y); point[2] = (float)(w*z);
				point += stride;
      }
      break;
    }
  }
  return rc;
}


bool 
ON_TransformPointList(
                  int dim, int is_rat, int count, 
                  int stride, double* point,
                  const ON_Xform& xform
                  )
{
  bool rc = true;
  double x, y, z, w;

  if ( !ON_IsValidPointList( dim, is_rat, count, stride, point ) )
    return false;
  if ( xform.m_xform == NULL )
    return false;
  if (count == 0)
    return true;

  if (is_rat) {
    switch(dim) {
    case 1:
      while(count--) {
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][3]*point[1];
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][3]*point[1];
				point[0] = x; point[1] = w;
				point += stride;
      }
      break;
    case 2:
      while(count--) {
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][3]*point[2];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][3]*point[2];
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][3]*point[2];
				point[0] = x; point[1] = y; point[2] = w;
				point += stride;
      }
      break;
    default: // dim >= 3
      while(count--) {
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][2]*point[2] + xform.m_xform[0][3]*point[dim];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][2]*point[2] + xform.m_xform[1][3]*point[dim];
				z = xform.m_xform[2][0]*point[0] + xform.m_xform[2][1]*point[1] + xform.m_xform[2][2]*point[2] + xform.m_xform[2][3]*point[dim];
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][2]*point[2] + xform.m_xform[3][3]*point[dim];
				point[0] = x; point[1] = y; point[2] = z; point[dim] = w;
				point += stride;
      }
      break;
    }
  }
  else {
    switch(dim) {
    case 1:
      while(count--) {
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][3];
        if (w==0.0) {
          rc = false;
          w = 1.0;
        }
        else
				  w = 1.0/w;
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][3];
				point[0] = w*x;
				point += stride;
      }
      break;
    case 2:
      while(count--) {
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][3];
        if (w==0.0) {
          rc = false;
          w = 1.0;
        }
        else
				  w = 1.0/w;
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][3];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][3];
				point[0] = w*x; point[1] = w*y;
				point += stride;
      }
      break;
    default: // dim = 3
      while(count--) {
				w = xform.m_xform[3][0]*point[0] + xform.m_xform[3][1]*point[1] + xform.m_xform[3][2]*point[2] + xform.m_xform[3][3];
        if (w==0.0)  {
          rc = false;
          w = 1.0;
        }
        else
				  w = 1.0/w;
				x = xform.m_xform[0][0]*point[0] + xform.m_xform[0][1]*point[1] + xform.m_xform[0][2]*point[2] + xform.m_xform[0][3];
				y = xform.m_xform[1][0]*point[0] + xform.m_xform[1][1]*point[1] + xform.m_xform[1][2]*point[2] + xform.m_xform[1][3];
				z = xform.m_xform[2][0]*point[0] + xform.m_xform[2][1]*point[1] + xform.m_xform[2][2]*point[2] + xform.m_xform[2][3];
				point[0] = w*x; point[1] = w*y; point[2] = w*z;
				point += stride;
      }
      break;
    }
  }
  return rc;
}


ON_BOOL32 
ON_TransformPointGrid(
                  int dim, int is_rat, 
                  int point_count0, int point_count1,
                  int point_stride0, int point_stride1,
                  double* point,
                  const ON_Xform& xform
                  )
{
  ON_BOOL32 rc = false;
  int i;
  double* pt = point;
  for ( i = 0; i < point_count0; i++ ) {
    if ( !ON_TransformPointList( dim, is_rat, point_count1, point_stride1, pt, xform ) ) {
      rc = false;
    }
    else if ( !i ) {
      rc = true;
    }
    pt += point_stride0;
  }
  return rc;
}


ON_BOOL32 
ON_TransformVectorList(
                  int dim, int count, 
                  int stride, float* vector,
                  const ON_Xform& xform
                  )
{
  ON_BOOL32 rc = true;
  double x, y, z;

  if ( !ON_IsValidPointList( dim, 0, count, stride, vector ) )
    return false;
  if ( xform.m_xform == NULL )
    return false;
  if (count == 0)
    return true;

  switch(dim) {
  case 1:
    while(count--) {
			x = xform.m_xform[0][0]*vector[0];
			vector[0] = (float)x;
			vector += stride;
    }
    break;
  case 2:
    while(count--) {
			x = xform.m_xform[0][0]*vector[0] + xform.m_xform[0][1]*vector[1];
			y = xform.m_xform[1][0]*vector[0] + xform.m_xform[1][1]*vector[1];
			vector[0] = (float)x; vector[1] = (float)y;
			vector += stride;
    }
    break;
  default: // dim >= 3
    while(count--) {
			x = xform.m_xform[0][0]*vector[0] + xform.m_xform[0][1]*vector[1] + xform.m_xform[0][2]*vector[2];
			y = xform.m_xform[1][0]*vector[0] + xform.m_xform[1][1]*vector[1] + xform.m_xform[1][2]*vector[2];
			z = xform.m_xform[2][0]*vector[0] + xform.m_xform[2][1]*vector[1] + xform.m_xform[2][2]*vector[2];
			vector[0] = (float)x; vector[1] = (float)y; vector[2] = (float)z;
			vector += stride;
    }
    break;
  }

  return rc;
}



ON_BOOL32 
ON_TransformVectorList(
                  int dim, int count, 
                  int stride, double* vector,
                  const ON_Xform& xform
                  )
{
  ON_BOOL32 rc = true;
  double x, y, z;

  if ( !ON_IsValidPointList( dim, 0, count, stride, vector ) )
    return false;
  if ( xform.m_xform == NULL )
    return false;
  if (count == 0)
    return true;

  switch(dim) {
  case 1:
    while(count--) {
			x = xform.m_xform[0][0]*vector[0];
			vector[0] = x;
			vector += stride;
    }
    break;
  case 2:
    while(count--) {
			x = xform.m_xform[0][0]*vector[0] + xform.m_xform[0][1]*vector[1];
			y = xform.m_xform[1][0]*vector[0] + xform.m_xform[1][1]*vector[1];
			vector[0] = x; vector[1] = y;
			vector += stride;
    }
    break;
  default: // dim >= 3
    while(count--) {
			x = xform.m_xform[0][0]*vector[0] + xform.m_xform[0][1]*vector[1] + xform.m_xform[0][2]*vector[2];
			y = xform.m_xform[1][0]*vector[0] + xform.m_xform[1][1]*vector[1] + xform.m_xform[1][2]*vector[2];
			z = xform.m_xform[2][0]*vector[0] + xform.m_xform[2][1]*vector[1] + xform.m_xform[2][2]*vector[2];
			vector[0] = x; vector[1] = y; vector[2] = z;
			vector += stride;
    }
    break;
  }

  return rc;
}

bool ON_PointsAreCoincident(
    int dim,
    int is_rat,
    const double* pointA,
    const double* pointB
    )
{
  double d, a, b, wa, wb;
  
  if ( dim < 1 || 0 == pointA || 0 == pointB )
    return false;

  if ( is_rat )
  {
    wa = pointA[dim];
    wb = pointB[dim];
    if ( 0.0 == wa || 0.0 == wb )
    {
      if ( 0.0 == wa && 0.0 == wb )
        return ON_PointsAreCoincident(dim,0,pointA,pointB);
      return false;
    }
    while(dim--)
    {
      a = *pointA++ / wa;
      b = *pointB++ / wb;
      d = fabs(a-b);
      if ( d <= ON_ZERO_TOLERANCE )
        continue;
      if ( d <= (fabs(a)+fabs(b))*ON_RELATIVE_TOLERANCE )
        continue;
      return false;
    }
  }
  else
  {
    while(dim--)
    {
      a = *pointA++;
      b = *pointB++;
      d = fabs(a-b);
      if ( d <= ON_ZERO_TOLERANCE )
        continue;
      if ( d <= (fabs(a)+fabs(b))*ON_RELATIVE_TOLERANCE )
        continue;
      return false;
    }
  }

  return true;
}

bool ON_PointsAreCoincident(
    int dim,
    int is_rat,
    int point_count,
    int point_stride,
    const double* points
    )
{
  if ( 0 == points || point_count < 2 )
    return false;
  if ( point_stride < (is_rat?(dim+1):dim) )
    return false;
  if ( false == ON_PointsAreCoincident( dim, is_rat, points, points + ((point_count-1)*point_stride) ) )
    return false;
  if ( point_count > 2 )
  {
    point_count--;
    while ( point_count-- )
    {
      if ( false == ON_PointsAreCoincident(dim,is_rat,points,points + point_stride ) )
        return false;
      points += point_stride;
    }
  }
  return true;
}

int 
ON_ComparePoint( // returns 
                              // -1: first < second
                              //  0: first == second
                              // +1: first > second
          int dim,
          ON_BOOL32 is_rat,
          const double* pointA,
          const double* pointB
          )
{
  const double wA = (is_rat && pointA[dim] != 0.0) ? 1.0/pointA[dim] : 1.0;
  const double wB = (is_rat && pointB[dim] != 0.0) ? 1.0/pointB[dim] : 1.0;
  double a, b, tol;
  int i;
  for ( i = 0; i < dim; i++ ) {
    a = wA* *pointA++;
    b = wB* *pointB++;
    tol = (fabs(a) + fabs(b))* ON_RELATIVE_TOLERANCE;
    if ( tol <  ON_ZERO_TOLERANCE )
      tol =  ON_ZERO_TOLERANCE;
    if ( a < b-tol )
      return -1;
    if ( b < a-tol )
      return 1;
    if ( wA < wB- ON_SQRT_EPSILON )
      return -1;
    if ( wB < wA- ON_SQRT_EPSILON )
      return -1;
  }
  return 0;
}


int 
ON_ComparePointList( // returns 
                              // -1: first < second
                              //  0: first == second
                              // +1: first > second
          int dim,
          ON_BOOL32 is_rat,
          int point_count,
          int point_strideA,
          const double* pointA,
          int point_strideB,
          const double* pointB
          )
{
  int i, rc = 0, rc1 = 0;
  bool bDoSecondCheck = ( 1 == is_rat && dim <= 3 && point_count > 0 
                         && ON_IsValid(pointA[dim]) && 0.0 != pointA[dim]
                         && ON_IsValid(pointB[dim]) && 0.0 != pointB[dim]
                         );
  const double wA = bDoSecondCheck ? pointA[dim] : 1.0; 
  const double wB = bDoSecondCheck ? pointB[dim] : 1.0; 
  const double wAtol = wA*ON_ZERO_TOLERANCE;
  const double wBtol = wB*ON_ZERO_TOLERANCE;
  double A[3] = {0.0,0.0,0.0};
  double B[3] = {0.0,0.0,0.0};
  const size_t AB_size = dim*sizeof(A[0]);

  for ( i = 0; i < point_count && !rc; i++ ) 
  {
    rc = ON_ComparePoint( dim, is_rat, pointA, pointB );
    if (    rc && bDoSecondCheck 
         && fabs(wA - pointA[dim]) <= wAtol 
         && fabs(wB - pointB[dim]) <= wBtol )
    {
      if ( !rc1 )
        rc1 = rc;
      memcpy(A,pointA,AB_size);
      A[0] /= pointA[dim]; A[1] /= pointA[dim]; A[2] /= pointA[dim];
      memcpy(B,pointB,AB_size);
      B[0] /= pointB[dim]; B[1] /= pointB[dim]; B[2] /= pointB[dim];
      rc = ( 0 == ON_ComparePoint( dim, 0, A, B ) ) ? 0 : rc1;
    }
    pointA += point_strideA;
    pointB += point_strideB;
  }

  return rc;
}


ON_BOOL32 
ON_IsPointListClosed(
       int dim,
       int is_rat,
       int count,
       int stride,
       const double* p
       )
{
  ON_BOOL32 rc = false;
  if ( count >= 4 && 0 == ON_ComparePoint( dim, is_rat, p, p+stride*(count-1) ) ) 
  {
    // a bunch of points piled on top of each other is not considered to be closed.
    for ( int i = 1; i < count-1; i++ ) {
      if ( ON_ComparePoint( dim, is_rat, p, p+stride*i ) ) {
        rc = true;
        break;
      }
    }
  }
  return rc;
}


ON_BOOL32 
ON_IsPointGridClosed(
        int dim,
        ON_BOOL32 is_rat,
        int point_count0, int point_count1,
        int point_stride0, int point_stride1,
        const double* p,
        int dir
       )
{
  ON_BOOL32 rc = false;
  if ( point_count0>0 && point_count1>0 && p != NULL ) {
    int count, stride;
    const double* p0;
    const double* p1;
    if ( dir ) {
      p0 = p;
      p1 = p + (point_count1-1)*point_stride1;
      count = point_count0;
      stride = point_stride0;
    }
    else {
      p0 = p;
      p1 = p + (point_count0-1)*point_stride0;
      count = point_count1;
      stride = point_stride1;
    }
    rc = (0==ON_ComparePointList( dim, is_rat, count, stride, p0, stride, p1 ))?true:false;
  }
  return rc;
}




int
ON_SolveQuadraticEquation(
       double a, double b, double c, 
       double *r0, double *r1
       )
/* Find solutions of a quadratic equation
 *
 * INPUT:
 *   a, b, c  coefficients defining the quadratic equation
 *            a*t^2 + b*t + c = 0
 *   r0, r1   address of doubles
 * OUTPUT:
 *   ON_QuadraticEquation()
 *      0: successful - two distinct real roots (*r0 < *r1)
 *      1: successful - one real root (*r0 = *r1)
 *      2: successful - two complex conjugate roots (*r0 +/- (*r1)*sqrt(-1))
 *     -1: failure - a = 0, b != 0        (*r0 = *r1 = -c/b)
 *     -2: failure - a = 0, b  = 0 c != 0 (*r0 = *r1 = 0.0)
 *     -3: failure - a = 0, b  = 0 c  = 0 (*r0 = *r1 = 0.0)
 *
 * COMMENTS:
 *   The quadratic equation is solved using the formula
 *   roots = q/a, c/q, q = 0.5*(b + sgn(b)*sqrt(b^2 - 4ac)).
 *
 *   When |b^2 - 4*a*c| <= b*b*ON_EPSILON, the discriminant
 *   is numerical noise and is assumed to be zero.
 *
 *   If it is really important to have the best possible answer,
 *   you sould probably tune up the returned roots using
 *   Brent's algorithm.
 *
 * REFERENCE:
 *   Numerical Recipes in C, section 5.5
 *
 * RELATED FUNCTIONS:
 *   ON_CubicEquation()
 */
{
  double q, x0, x1, y0, y1, y;

  if (a == 0.0) {
    if (b == 0.0) 
      {*r0 = *r1 = 0.0; return (c == 0.0) ? -3 : -2;}
    *r0 = *r1 = -c/b; return -1;
  }

  if (c == 0.0) {
    if (b == 0.0) 
      {*r0 = *r1 = 0.0; return 1;}
    b /= -a;
    if (b < 0.0) 
      {*r0=b;*r1=0.0;} 
    else
      {*r0=0.0;*r1=b;}
    return 0;
  }

  if (b == 0.0) {
    c /= -a;
    *r1 = sqrt(fabs(c));
    if (c < 0.0) 
      {*r0 = 0.0; return 2;}
    *r0 = -(*r1);
    return 0;
  }
  q = b*b - 4.0*a*c;
  if (fabs(q) <= b*b* ON_EPSILON) 
    q = 0.0; /* q is noise - set it to zero */
  if (q <= 0.0) {
    /* multiple real root or complex conjugate roots */
    *r0 = -0.5*b/a;
    if (q == 0.0) 
      {*r1 = *r0; return 1;}

    /* complex conjugate roots (probably) */
    *r1 = fabs(0.5*sqrt(fabs(q))/a); 
    x0 = *r0;
    x1 = *r1;
    y = (a*x0 + b)*x0 + c;            /* y = quadratic evaluated at -b/2a */
    if ((a > 0.0 && y <= 0.0) || (a < 0.0 && y >= 0.0))
      {*r1 = *r0; return 1;}
    y0 = y - a*x1*x1;                 /* y0 = real part of "zero" */
    y1 = (2.0*a*x0 + b)*x1;           /* y1 = imaginary part of "zero" */
    if (fabs(y) <= fabs(y0) || fabs(y) <= fabs(y1)) 
      {*r1 = *r0; return 1;}
    return 2;
  }

  /* distinct roots (probably) */
  q = 0.5*(fabs(b) + sqrt(q));
  if (b > 0.0) q = -q;
  x0 = q/a;
  x1 = c/q;
  if (x0 == x1) 
    {*r0 = *r1 = x0; return 1;}

  if (x0 > x1) 
    {y = x0; x0 = x1; x1 = y;}

  /* quick test to see if roots are numerically distinct from extrema */
  y = -0.5*b/a;
  if (x0 <= y && y <= x1) {
    y = (a*y + b)*y + c;              /* y = quadratic evaluated at -b/2a */
    y0 = (a*x0 + b)*x0 + c;
    y1 = (a*x1 + b)*x1 + c;
    if (fabs(y) <= fabs(y0) || fabs(y) <= fabs(y1)
        || (a > 0.0 && y > 0.0) || (a < 0.0 && y < 0.0))
      {*r0 = *r1 = -0.5*b/a; return 1;}
  }

  /* distinct roots */
  *r0 = x0;
  *r1 = x1;
  return 0;
}


ON_BOOL32
ON_SolveTriDiagonal( int dim, int n, 
                          double* a, const double* b, double* c,
                          const double* d, double* X)
/*****************************************************************************
Solve a tridiagonal linear system of equations using backsubstution
 
INPUT:
  dim   (>=1) dimension of X and d
  n     (>=2) number of equations
  a,b,c,d
        coefficients of the linear system. a and c are arrays of n-1 doubles.
        b and d are arrays of n doubles.  Note that "a", "b" and "d" are
        not modified. "c" is modified.
  X     array of n doubles 
OUTPUT:
  ON_SolveTriDiagonal()  0: success
                        -1: failure - illegal input
                        -2: failure - zero pivot encountered
                                      (can happen even when matrix is
                                       non-singular)

  X     if ON_SolveTriDiagonal() returns 0, then X is the solution to

  b[0]   c[0]                                X[0]        d[0]
  a[0]   b[1]  c[1]                          X[1]        d[1]
         a[1]  b[2]  c[2]                  * X[2]     =  d[2]
               ....  ....  ....              ...         ...
                     a[n-3] b[n-2] c[n-2]    X[n-2]      d[n-2]
                            a[n-2] b[n-1]    X[n-1]      d[n-1]

COMMENTS:
  If n <= 3, this function uses ON_Solve2x2() or ON_Solve3x3().  
  If n > 3, the system is solved in the fastest possible manner; 
  in particular,  no pivoting is performed, b[0] must be nonzero.
  If |b[i]| > |a[i-1]| + |c[i]|, then this function will succeed.
  The computation is performed in such a way that the output
  "X" pointer can be equal to the input "d" pointer; i.e., if the
  d array will not be used after the call to ON_SolveTriDiagonal(), then
  it is not necessary to allocate seperate space for X and d.
EXAMPLE:
REFERENCE:
  NRC, section 2.6
RELATED FUNCTIONS:
  ON_Solve2x2
  ON_Solve3x3
  ON_SolveSVD
*****************************************************************************/
{
  double beta, g, q;
  int i, j;
  if (dim < 1 || n < 2 || !a || !b || !c || !d || !X)
    return -1;

  if (dim == 1) {
    /* standard tri-diagonal problem -  X and d are scalars */
    beta = *b++;
    if (beta == 0.0)
      return -2;
    beta = 1.0/beta;
    *X = *d++ *beta;
    i = n-1;
    while(i--) {
      g = (*c++ *= beta);
      beta = *b++ - *a * g;
      if (beta == 0.0) return -2;
      beta = 1.0/beta;
      X[1] = (*d++ - *a++ * *X)*beta;
      X++;      
    }
    X--;
    c--;
    i = n-1;
    while(i--) {
      *X -= *c-- * X[1];
      X--;
    }
  }
  else {
    /* X and d are vectors */
    beta = *b++;
    if (beta == 0.0)
      return -2;
    beta = 1.0/beta;
    j = dim;
    while(j--)
      *X++ = *d++ *beta;
    X -= dim;
    i = n-1;
    while(i--) {
      g = (*c++ *= beta);
      beta = *b++ - *a * g;
      if (beta == 0.0) return -2;
      beta = 1.0/beta;
      j = dim;
      q = *a++;
      while(j--) {
        X[dim] = (*d++ - q * *X)*beta;
        X++;      
      }
    }
    X--;
    c--;
    i = n-1;
    while(i--) {
      q = *c--;
      j = dim;
      while(j--) {
        *X -= q * X[dim];
        X--;
      }
    }
  }

  return 0;
}


int
ON_Solve2x2( double m00, double m01, double m10, double m11, double d0, double d1,
                  double* x_addr, double* y_addr, double* pivot_ratio)
/* Solve a 2x2 system of linear equations
 *
 * INPUT:
 *   m00, m01, m10, m11, d0, d1
 *      coefficients for the 2x2 the linear system:
 *   x_addr, y_addr
 *      addresses of doubles
 *   pivot_ratio
 *      address of double
 * OUTPUT:
 *   ON_Solve2x2() returns rank (0,1,2)
 *
 *   If ON_Solve2x2() is successful (return code 2), then
 *   the solution is returned in {*x_addr, *y_addr} and
 *   *pivot_ratio = min(|pivots|)/max(|pivots|).
 *
 *   WARNING: If the pivot ratio is small, then the matrix may
 *   be singular or ill conditioned.  You should test the results
 *   before you use them.
 *
 * COMMENTS:
 *      The system of 2 equations and 2 unknowns (x,y),
 *         m00*x + m01*y = d0
 *         m10*x + m11*y = d1,
 *      is solved using Gauss-Jordan elimination
 *      with full pivoting.
 * EXAMPLE:
 *      // Find the intersection of 2 2D lines where
 *      // P0, P1  are points on the lines and
 *      // D0, D1, are nonzero directions
 *      rc = ON_Solve2x2(D0[0],-D1[0],D0[1],-D1[1],P1[0]-P0[0],P1[1]-P0[1],
 *                       &x, &y,&pivot_ratio);
 *      switch(rc) {
 *      case  0: // P0 + x*D0 = P1 + y*D1 = intersection point
 *        if (pivot_ratio < 0.001) {
 *          // small pivot ratio - test answer before using ...
 *        }
 *        break;
 *      case -1: // both directions are zero!
 *        break;
 *      case -2: // parallel directions
 *        break;
 *      }
 *
 * REFERENCE:
 *      STRANG
 *
 * RELATED FUNCTIONS:
 *      ON_Solve3x2(), ON_Solve3x3
 */
{
  int i = 0;
  double maxpiv, minpiv;
  double x = fabs(m00);
  double y = fabs(m01); if (y > x) {x = y; i = 1;}
  y = fabs(m10); if (y > x) {x = y; i = 2;}
  y = fabs(m11); if (y > x) {x = y; i = 3;}
  *pivot_ratio = *x_addr = *y_addr = 0.0;
  if (x == 0.0) 
    return 0; // rank = 0
  minpiv = maxpiv = x;
  if (i%2) {
    {double* tmp = x_addr; x_addr = y_addr; y_addr = tmp;}
    x = m00; m00 = m01; m01 = x;
    x = m10; m10 = m11; m11 = x;
  }
  if (i > 1) {
    x = d0; d0 = d1; d1 = x;
    x = m00; m00 = m10; m10 = x;
    x = m01; m01 = m11; m11 = x;
  }
  
  x = 1.0/m00;
  m01 *= x; d0 *= x;
  if (m10 != 0.0) {m11 -= m10*m01; d1 -= m10*d0;}

  if (m11 == 0.0) 
    return 1; // rank = 1

  y = fabs(m11);
  if (y > maxpiv) maxpiv = y; else if (y < minpiv) minpiv = y;
  
  d1 /= m11;
  if (m01 != 0.0)
    d0 -= m01*d1;

  *x_addr = d0;
  *y_addr = d1;
  *pivot_ratio = minpiv/maxpiv;
  return 2;  
}


int 
ON_Solve3x2(const double col0[3], const double col1[3], 
                double d0, double d1, double d2,
                double* x_addr, double* y_addr, double* err_addr, double* pivot_ratio)
/* Solve a 3x2 system of linear equations
 *
 * INPUT:
 *   col0, col1
 *      arrays of 3 doubles
 *   d0, d1, d2
 *      right hand column of system
 *   x_addr, y_addr, err_addr, pivot_ratio
 *      addresses of doubles
 * OUTPUT:
 *   TL_Solve3x2()
 *       2: successful
 *       0: failure - 3x2 matrix has rank 0
 *       1: failure - 3x2 matrix has rank 1
 *      If the return code is zero, then
 *        (*x_addr)*{col0} + (*y_addr)*{col1}
 *        + (*err_addr)*({col0 X col1}/|col0 X col1|)
 *        = {d0,d1,d2}.
 *      pivot_ratio = min(|pivots|)/max(|pivots|)  If this number
 *      is small, then the 3x2 matrix may be singular or ill 
 *      conditioned.
 * COMMENTS:
 *      The system of 3 equations and 2 unknowns (x,y),
 *              x*col0[0] + y*col1[1] = d0
 *              x*col0[0] + y*col1[1] = d1
 *              x*col0[0] + y*col1[1] = d2,
 *      is solved using Gauss-Jordan elimination
 *      with full pivoting.
 * EXAMPLE:
 *      // If A, B and T are 3D vectors, find a and b so that
 *      // T - a*A + b*B is perpendicular to both A and B.
 *      rc = TL_Solve3x3(A,B,T[0],T[1],T[2],&a,&b,&len);
 *      switch(rc) {
 *      case  0: // {x,y,z} = intersection point, len = T o (A X B / |A X B|)
 *        break;
 *      case -1: // both A and B are zero!
 *        break;
 *      case -2: // A and B are parallel, or one of A and B is zero.
 *        break;
 *      }
 * REFERENCE:
 *      STRANG
 * RELATED FUNCTIONS:
 *      ON_Solve2x2, ON_Solve3x3,
 */
{
  /* solve 3x2 linear system using Gauss-Jordan elimination with
   * full pivoting.  Input columns not modified.
   * returns 0: rank 0, 1: rank 1, 2: rank 2
   *         *err = signed distance from (d0,d1,d2) to plane
   *                through origin with normal col0 X col1.
   */
  int i;
  double x, y;
  ON_3dVector c0, c1;

  *x_addr = *y_addr = *pivot_ratio = 0.0;
  *err_addr = ON_DBL_MAX;
  i = 0;
  x = fabs(col0[0]);
  y = fabs(col0[1]); if (y>x) {x = y; i = 1;}
  y = fabs(col0[2]); if (y>x) {x = y; i = 2;}
  y = fabs(col1[0]); if (y>x) {x = y; i = 3;}
  y = fabs(col1[1]); if (y>x) {x = y; i = 4;}
  y = fabs(col1[2]); if (y>x) {x = y; i = 5;}
  if (x == 0.0) return 0;
  *pivot_ratio = fabs(x);
  if (i >= 3) {
    /* swap columns */
    double* ptr = x_addr; x_addr = y_addr; y_addr = ptr;
    c0 = col1;
    c1 = col0;
  }
  else {
    c0 = col0;
    c1 = col1;
  }

  switch((i%=3)) {
  case 1: /* swap rows 0 and 1*/
    x=c0.y;c0.y=c0.x;c0.x=x;
    x=c1.y;c1.y=c1.x;c1.x=x;
    x=d1;d1=d0;d0=x;
    break;
  case 2: /* swap rows 0 and 2*/
    x=c0.z;c0.z=c0.x;c0.x=x;
    x=c1.z;c1.z=c1.x;c1.x=x;
    x=d2;d2=d0;d0=x;
    break;
  }

  c1.x /= c0.x; d0 /= c0.x;
  x = -c0.y; if (x != 0.0) {c1.y += x*c1.x; d1 += x*d0;}
  x = -c0.z; if (x != 0.0) {c1.z += x*c1.x; d2 += x*d0;}

  if (fabs(c1.y) > fabs(c1.z)) {
    if (fabs(c1.y) > *pivot_ratio)
      *pivot_ratio /= fabs(c1.y); 
    else 
      *pivot_ratio = fabs(c1.y)/ *pivot_ratio;
    d1 /= c1.y;
    x = -c1.x; if (x != 0.0) d0 += x*d1;
    x = -c1.z; if (x != 0.0) d2 += x*d1;
    *x_addr = d0;
    *y_addr = d1;
    *err_addr = d2;
  }
  else if (c1.z == 0.0) 
    return 1; /* 3x2 matrix has rank = 1 */
  else {
    if (fabs(c1.z) > *pivot_ratio)
      *pivot_ratio /= fabs(c1.z); 
    else 
      *pivot_ratio = fabs(c1.z)/ *pivot_ratio;
    d2 /= c1.z;
    x = -c1.x; if (x != 0.0) d0 += x*d2;
    x = -c1.y; if (x != 0.0) d1 += x*d2;
    *x_addr = d0;
    *err_addr = d1;
    *y_addr = d2;
  }

  return 2;
}

double ON_SolveNxN(bool bFullPivot, bool bNormalize, int n, double* M[], double B[], double X[])
{
  if ( n <= 0 || 0 == M || 0 == B || 0 == X )
    return 0.0;

  int i,j,maxi,maxj,n0, Xdex_buffer[64];
  double x,minpivot=0.0,maxpivot=1.0,*p;
  int* Xdex = 0;
  
  if ( bNormalize )
  {
    for ( i = 0; i < n; i++ )
    {
      x = 0.0;
      for ( j = 0; j < n; j++ )
      {
        x += M[i][j]*M[i][j];
      }
      if ( x > 0.0 )
      {
        x = 1.0/sqrt(x);
        B[i] *= x;
        for ( j = 0; j < n; j++ )
          M[i][j] *= x;
      }
    }
  }

  if ( bFullPivot )
  {
    // The Xdex_buffer[] hoo haa is here to avoid an potentially time
    // consuming call to heap services when the matrix is small.
    // When n > 64 the numerical portion of the computation is 
    // long enough that the time to call onmalloc() is negligable.
    // (When n > 10-ish, this calculation is likely to return junk
    // unless you have a special case matrix, in which case this
    // function will be much slower than one that is designed to
    // takes advantage of the special case.
    Xdex = (n <= (int)((sizeof(Xdex_buffer)/sizeof(Xdex_buffer[0]))))
         ? &Xdex_buffer[0] 
         : (int*)onmalloc(n*sizeof(Xdex[0]));
    for ( i = 0; i < n; i++ )
      Xdex[i] = i;
  }

  // reduce system of equations to upper triangular
  for (n0=0; n0<n; n0++)
  {
    // find pivot = biggest entry in sub-matrix
    maxi=n0;
    maxj=n0;
    x=0.0;
    for(j=n0;j<n;j++) 
    {
      for(i=n0;i<n;i++)  
      {
        if ( fabs(M[i][j]) > x )
        {
          x=fabs(M[i][j]);
          maxi=i;
          maxj=j;
        }
      }
      if ( !bFullPivot )
        break;
    }
    if ( 0.0==x )
    {
      // system of equations is degenerate
      // Return -(rank of M) (M has rank n0)
      if ( 0 != Xdex && Xdex != &Xdex_buffer[0] )
        onfree(Xdex);
      return -n0;
    }
    else if ( 0==n0 )
    {
      minpivot=x;
      maxpivot=x;
    }
    else if (x < minpivot)
      minpivot=x;
    else if  (x > maxpivot)
      maxpivot=x;

    // put pivot in M[n0][n0]
    if ( n0 != maxi )
    {
      // swap rows n0 and maxi
      p = M[n0];M[n0]=M[maxi];M[maxi]=p;
      x=B[n0];B[n0]=B[maxi];B[maxi]=x;
    }
    if ( n0 != maxj )
    {
      // swap columns n0 and maxj
      for (i=0;i<n;i++)
      {
        x=M[i][n0];M[i][n0]=M[i][maxj];M[i][maxj]=x;
      }
      j=Xdex[n0];Xdex[n0]=Xdex[maxj];Xdex[maxj]=j;
    }

    // divide row n0 by M[n0][n0] to unitize M[n0][n0]
    x = 1.0/M[n0][n0];
    //M[n0][n0] = 1.0; // cosmetic because M[n0][n0] will never be used again
    B[n0] *= x;
    for (j=n0+1;j<n;j++) 
      M[n0][j] *= x;

    // For each i > n0, replace row[i] with
    // row[i] - M[i][n0]*row[n0] to zero out M[i>n0][n0]
    for ( i = n0+1; i < n; i++ )
    {
      x = -M[i][n0];
      if ( 0.0 != x )
      {
        //M[i][n0] = 0.0; // cosmetic because M[i][n0] will never be used again
        B[i] += x*B[n0];
        for ( j = n0+1; j < n; j++ )
        {
          M[i][j] += x*M[n0][j];
        }
      }
    }
  }

  // At this point M is upper triangular with 1's
  // on its diagonal.  Backsolve.
  for ( j = n-1; j >= 0; j-- )
  {
    for ( i = 0; i < j; i++ )
    {
      x = -M[i][j];
      if ( x != 0 )
      {
        B[i] += x*B[j];
        //M[i][j] += x; // cosmetic because M[i][j] will never be used again
      }
    }
  }

  // solution is now in B
  if ( bFullPivot )
  {
    for(i=0;i<n;i++)
      X[Xdex[i]] = B[i];

    if ( 0 != Xdex && Xdex != &Xdex_buffer[0] )
      onfree(Xdex);
  }
  else
  {
    memcpy(&X[0],&B[0],n*sizeof(X[0]));
  }

  return minpivot/maxpivot;
}


int
ON_Solve4x4(const double row0[4], const double row1[4], const double row2[4], const double row3[4],
                double d0, double d1, double d2, double d3,
                double* x_addr, double* y_addr, double* z_addr, double* w_addr,
                double* pivot_ratio)
{
  /* Solve a 4x4 linear system using Gauss-Jordan elimination 
   * with full pivoting.
   */
  int i, j;
  double *p0, *p1, *p2, *p3, *p;
  double x, y, workarray[20], maxpiv, minpiv;

  const int sizeof_row = 4*sizeof(row0[4]);

  *pivot_ratio = *x_addr = *y_addr = *z_addr = *w_addr = 0.0;
  x = fabs(row0[0]); i=j=0;
  y = fabs(row0[1]); if (y>x) {x=y;j=1;}
  y = fabs(row0[2]); if (y>x) {x=y;j=2;}
  y = fabs(row0[3]); if (y>x) {x=y;j=3;}

  y = fabs(row1[0]); if (y>x) {x=y;i=1;j=0;}
  y = fabs(row1[1]); if (y>x) {x=y;i=1;j=1;}
  y = fabs(row1[2]); if (y>x) {x=y;i=1;j=2;}
  y = fabs(row1[3]); if (y>x) {x=y;i=1;j=3;}

  y = fabs(row2[0]); if (y>x) {x=y;i=2;j=0;}
  y = fabs(row2[1]); if (y>x) {x=y;i=2;j=1;}
  y = fabs(row2[2]); if (y>x) {x=y;i=2;j=2;}
  y = fabs(row2[3]); if (y>x) {x=y;i=2;j=3;}

  y = fabs(row3[0]); if (y>x) {x=y;i=3;j=0;}
  y = fabs(row3[1]); if (y>x) {x=y;i=3;j=1;}
  y = fabs(row3[2]); if (y>x) {x=y;i=3;j=2;}
  y = fabs(row3[3]); if (y>x) {x=y;i=3;j=3;}

  if (x == 0.0) 
    return 0; // rank = 0

  maxpiv = minpiv = x;
  p0 = workarray;
  p1 = p0+5;
  p2 = p1+5;
  p3 = p2+5;
  switch(i) 
  {
  case 1: /* swap rows 0 and 1 */
    memcpy(p0,row1,sizeof_row); p0[4] = d1;
    memcpy(p1,row0,sizeof_row); p1[4] = d0;
    memcpy(p2,row2,sizeof_row); p2[4] = d2;
    memcpy(p3,row3,sizeof_row); p3[4] = d3;
    break;
  case 2: /* swap rows 0 and 2 */
    memcpy(p0,row2,sizeof_row); p0[4] = d2;
    memcpy(p1,row1,sizeof_row); p1[4] = d1;
    memcpy(p2,row0,sizeof_row); p2[4] = d0;
    memcpy(p3,row3,sizeof_row); p3[4] = d3;
    break;
  case 3: /* swap rows 0 and 3 */
    memcpy(p0,row3,sizeof_row); p0[4] = d3;
    memcpy(p1,row1,sizeof_row); p1[4] = d1;
    memcpy(p2,row2,sizeof_row); p2[4] = d2;
    memcpy(p3,row0,sizeof_row); p3[4] = d0;
    break;
  default:
    memcpy(p0,row0,sizeof_row); p0[4] = d0;
    memcpy(p1,row1,sizeof_row); p1[4] = d1;
    memcpy(p2,row2,sizeof_row); p2[4] = d2;
    memcpy(p3,row3,sizeof_row); p3[4] = d3;
    break;
  }

  switch(j) 
  {
  case 1: /* swap columns 0 and 1 */
    p = x_addr; x_addr = y_addr; y_addr = p;
    x = p0[0]; p0[0]=p0[1]; p0[1]=x;
    x = p1[0]; p1[0]=p1[1]; p1[1]=x;
    x = p2[0]; p2[0]=p2[1]; p2[1]=x;
    x = p3[0]; p3[0]=p3[1]; p3[1]=x;
    break;
  case 2: /* swap columns 0 and 2 */
    p = x_addr; x_addr = z_addr; z_addr = p;
    x = p0[0]; p0[0]=p0[2]; p0[2]=x;
    x = p1[0]; p1[0]=p1[2]; p1[2]=x;
    x = p2[0]; p2[0]=p2[2]; p2[2]=x;
    x = p3[0]; p3[0]=p3[2]; p3[2]=x;
    break;
  case 3: /* swap columns 0 and 3 */
    p = x_addr; x_addr = w_addr; w_addr = p;
    x = p0[0]; p0[0]=p0[3]; p0[3]=x;
    x = p1[0]; p1[0]=p1[3]; p1[3]=x;
    x = p2[0]; p2[0]=p2[3]; p2[3]=x;
    x = p3[0]; p3[0]=p3[3]; p3[3]=x;
    break;
  }

  /////////////////
  //
  //  p0 = M * * *   *
  //  p1 = ~ * * *   *
  //  p2 = ~ * * *   *
  //  p3 = ~ * * *   *
  //
  x = 1.0/p0[0];
  /* debugger set p0[0] = 1 */
  p0[1] *= x; p0[2] *= x; p0[3] *= x; p0[4] *= x;

  x = -p1[0];
  /* debugger set p1[0] = 0 */
  if (x != 0.0) 
  {
    p1[1] += x*p0[1]; p1[2] += x*p0[2]; p1[3] += x*p0[3]; p1[4] += x*p0[4];
  }

  x = -p2[0];
  if (x != 0.0) 
  {
    /* debugger set p2[0] = 0 */
    p2[1] += x*p0[1]; p2[2] += x*p0[2]; p2[3] += x*p0[3]; p2[4] += x*p0[4];
  }

  x = -p3[0];
  if (x != 0.0) 
  {
    /* debugger set p3[0] = 0 */
    p3[1] += x*p0[1]; p3[2] += x*p0[2]; p3[3] += x*p0[3]; p3[4] += x*p0[4];
  }

  /////////////////
  //
  //  p0 = 1 * * *   *
  //  p1 = 0 * * *   *
  //  p2 = 0 * * *   *
  //  p3 = 0 * * *   *
  //
  x = fabs(p1[1]); i=j=0;
  y = fabs(p1[2]); if (y>x) {x=y;j=1;}
  y = fabs(p1[3]); if (y>x) {x=y;j=2;}

  y = fabs(p2[1]); if (y>x) {x=y;i=1;j=0;}
  y = fabs(p2[2]); if (y>x) {x=y;i=1;j=1;}
  y = fabs(p2[3]); if (y>x) {x=y;i=1;j=2;}

  y = fabs(p3[1]); if (y>x) {x=y;i=2;j=0;}
  y = fabs(p3[2]); if (y>x) {x=y;i=2;j=1;}
  y = fabs(p3[3]); if (y>x) {x=y;i=2;j=2;}
  if (x == 0.0) 
  {
    *x_addr = p0[4];
    return 1; // rank = 1;
  }

  if (x > maxpiv) maxpiv = x; else if (x < minpiv) minpiv = x;
  if ( 1 == j )
  {
    /* swap columns 1 and 2 */
    x = p0[1]; p0[1] = p0[2]; p0[2] = x;
    x = p1[1]; p1[1] = p1[2]; p1[2] = x;
    x = p2[1]; p2[1] = p2[2]; p2[2] = x;
    x = p3[1]; p3[1] = p3[2]; p3[2] = x;
    p = y_addr; y_addr = z_addr; z_addr = p;
  }
  else if ( 2 == j )
  {
    /* swap columns 1 and 3 */
    x = p0[1]; p0[1] = p0[3]; p0[3] = x;
    x = p1[1]; p1[1] = p1[3]; p1[3] = x;
    x = p2[1]; p2[1] = p2[3]; p2[3] = x;
    x = p3[1]; p3[1] = p3[3]; p3[3] = x;
    p = y_addr; y_addr = w_addr; w_addr = p;
  }

  if (1 == i) 
  {
    /* swap rows 1 and 2 */
    p = p1; p1 = p2; p2 = p;
  }
  else if (2 == i) 
  {
    /* swap rows 1 and 3 */
    p = p1; p1 = p3; p3 = p;
  }

  /////////////////
  //
  //  p0 = 1 * * *   *
  //  p1 = 0 M * *   *
  //  p2 = 0 ~ * *   *
  //  p3 = 0 ~ * *   *
  //
  x = 1.0/p1[1];
  /* debugger set p1[1] = 1 */
  p1[2] *= x; p1[3] *= x; p1[4] *= x;

  x = -p2[1];
  if (x != 0.0) 
  {
    /* debugger set p2[1] = 0 */
    p2[2] += x*p1[2]; p2[3] += x*p1[3]; p2[4] += x*p1[4];
  }

  x = -p3[1];
  if (x != 0.0) 
  {
    /* debugger set p3[1] = 0 */
    p3[2] += x*p1[2]; p3[3] += x*p1[3]; p3[4] += x*p1[4];
  }

  /////////////////
  //
  //  p0 = 1 * * *   *
  //  p1 = 0 1 * *   *
  //  p2 = 0 0 * *   *
  //  p3 = 0 0 * *   *
  //
  x = fabs(p2[2]);i=j=0;
  y = fabs(p2[3]);if (y>x) {x=y;j=1;}
  y = fabs(p3[2]);if (y>x) {x=y;i=1;j=0;}
  y = fabs(p3[3]);if (y>x) {x=y;i=j=1;}
  if (x == 0.0) 
  {
    *y_addr = p2[4];
    *x_addr = p0[4] - p0[1]*(*y_addr);
    return 2; // rank = 2;
  }
  if (x > maxpiv) maxpiv = x; else if (x < minpiv) minpiv = x;
  if (j) 
  {
    /* swap columns 2 and 3 */
    x = p0[2]; p0[2] = p0[3]; p0[3] = x;
    x = p1[2]; p1[2] = p1[3]; p1[3] = x;
    x = p2[2]; p2[2] = p2[3]; p2[3] = x;
    x = p3[2]; p3[2] = p3[3]; p3[3] = x;
    p = z_addr; z_addr = w_addr; w_addr = p;
  }

  if (i) 
  {
    /* swap rows 2 and 3 */
    p = p2;
    p2 = p3;
    p3 = p;
  }

  /////////////////
  //
  //  p0 = 1 * * *   *
  //  p1 = 0 1 * *   *
  //  p2 = 0 0 M *   *
  //  p3 = 0 0 ~ *   *
  //

  /* debugger set p2[2] = 1 */
  x = 1.0/p2[2]; p2[3] *= x; p2[4] *= x;
  x = - p3[2];
  if (x != 0.0) 
  {
    /* debugger set p3[2] = 0 */
    p3[3] += x*p2[3]; p3[4] += x*p2[4];
  }
  /////////////////
  //
  //  p0 = 1 * * *   *
  //  p1 = 0 1 * *   *
  //  p2 = 0 0 1 *   *
  //  p3 = 0 0 0 *   *
  //

  x = fabs(p3[3]);
  if ( x == 0.0 )
  {
    *z_addr = p2[4];
    *y_addr = p1[4] - p1[2]*(*z_addr);
    *x_addr = p0[4] - p0[1]*(*y_addr) - p0[2]*(*z_addr);
    return 3; // rank = 3
  }
  if (x > maxpiv) maxpiv = x; else if (x < minpiv) minpiv = x;

  // backsolve in a that optimizers can use cache/registers
  p3[4] /= p3[3];
  p2[4] -= p2[3]*p3[4];
  p1[4] -= p1[2]*p2[4] + p1[3]*p3[4];
  p0[4] -= p1[4]*p0[1] + p2[4]*p0[2] + p3[4]*p0[3];

  // return answer
  *x_addr = p0[4];
  *y_addr = p1[4];
  *z_addr = p2[4];
  *w_addr = p3[4];
  *pivot_ratio = minpiv/maxpiv;

  return 4; // rank = 4
}




int
ON_Solve3x3(const double row0[3], const double row1[3], const double row2[3],
                double d0, double d1, double d2,
                double* x_addr, double* y_addr, double* z_addr,
                double* pivot_ratio)
{
  /* Solve a 3x3 linear system using Gauss-Jordan elimination 
   * with full pivoting.
   */
  int i, j;
  double* p0;
  double* p1;
  double* p2;
  double x, y, workarray[12], maxpiv, minpiv;

  const int sizeof_row = 3*sizeof(row0[0]);

  *pivot_ratio = *x_addr = *y_addr = *z_addr = 0.0;
  x = fabs(row0[0]); i=j=0;
  y = fabs(row0[1]); if (y>x) {x=y;j=1;}
  y = fabs(row0[2]); if (y>x) {x=y;j=2;}
  y = fabs(row1[0]); if (y>x) {x=y;i=1;j=0;}
  y = fabs(row1[1]); if (y>x) {x=y;i=1;j=1;}
  y = fabs(row1[2]); if (y>x) {x=y;i=1;j=2;}
  y = fabs(row2[0]); if (y>x) {x=y;i=2;j=0;}
  y = fabs(row2[1]); if (y>x) {x=y;i=2;j=1;}
  y = fabs(row2[2]); if (y>x) {x=y;i=2;j=2;}
  if (x == 0.0) 
    return 0;
  maxpiv = minpiv = fabs(x);
  p0 = workarray;
  switch(i) {
  case 1: /* swap rows 0 and 1 */
    memcpy(p0,row1,sizeof_row); p0[3] = d1; p0 += 4;
    memcpy(p0,row0,sizeof_row); p0[3] = d0; p0 += 4;
    memcpy(p0,row2,sizeof_row); p0[3] = d2;
    break;
  case 2: /* swap rows 0 and 2 */
    memcpy(p0,row2,sizeof_row); p0[3] = d2; p0 += 4;
    memcpy(p0,row1,sizeof_row); p0[3] = d1; p0 += 4;
    memcpy(p0,row0,sizeof_row); p0[3] = d0;
    break;
  default:
    memcpy(p0,row0,sizeof_row); p0[3] = d0; p0 += 4;
    memcpy(p0,row1,sizeof_row); p0[3] = d1; p0 += 4;
    memcpy(p0,row2,sizeof_row); p0[3] = d2;
    break;
  }
  switch(j) {
  case 1: /* swap columns 0 and 1 */
    p0 = x_addr; x_addr = y_addr; y_addr = p0;
    p0 = &workarray[0]; 
    x = p0[0]; p0[0]=p0[1]; p0[1]=x; p0 += 4;
    x = p0[0]; p0[0]=p0[1]; p0[1]=x; p0 += 4;
    x = p0[0]; p0[0]=p0[1]; p0[1]=x;
    break;
  case 2: /* swap columns 0 and 2 */
    p0 = x_addr; x_addr = z_addr; z_addr = p0;
    p0 = &workarray[0]; 
    x = p0[0]; p0[0]=p0[2]; p0[2]=x; p0 += 4;
    x = p0[0]; p0[0]=p0[2]; p0[2]=x; p0 += 4;
    x = p0[0]; p0[0]=p0[2]; p0[2]=x;
    break;
  }

  x = 1.0/workarray[0];
  /* debugger set workarray[0] = 1 */
  p0 = p1 = workarray + 1;
  *p1++ *= x; *p1++ *= x; *p1++ *= x;
  x = -(*p1++);
  /* debugger set workarray[4] = 0 */
  if (x == 0.0) 
    p1 += 3;
  else 
    {*p1++ += x*(*p0++); *p1++ += x*(*p0++); *p1++ += x*(*p0); p0 -= 2;}
  x = -(*p1++);
  /* debugger set workarray[8] = 0 */
  if (x != 0.0)
    {*p1++ += x*(*p0++); *p1++ += x*(*p0++); *p1++ += x*(*p0); p0 -= 2;}

  x = fabs(workarray[ 5]);i=j=0;
  y = fabs(workarray[ 6]);if (y>x) {x=y;j=1;}
  y = fabs(workarray[ 9]);if (y>x) {x=y;i=1;j=0;}
  y = fabs(workarray[10]);if (y>x) {x=y;i=j=1;}
  if (x == 0.0) 
    return 1; // rank = 1;
  y = fabs(x);
  if (y > maxpiv) maxpiv = y; else if (y < minpiv) minpiv = y;
  if (j) {
    /* swap columns 1 and 2 */
    p0 = workarray+1;
    p1 = p0+1;
    x = *p0; *p0 = *p1; *p1 = x; p0 += 4; p1 += 4;
    x = *p0; *p0 = *p1; *p1 = x; p0 += 4; p1 += 4;
    x = *p0; *p0 = *p1; *p1 = x; p0 += 4; p1 += 4;
    p0 = y_addr; y_addr = z_addr; z_addr = p0;
  }

  if (i) {
    /* pivot is in row 2 */
    p0 = workarray+1;
    p1 = p0 + 8;
    p2 = p0 + 4;
  }
  else {
    /* pivot is in row 1 */
    p0 = workarray+1;
    p1 = p0 + 4;
    p2 = p0 + 8;
  }

  /* debugger set workarray[5+4*i] = 1 */
  x = 1.0/(*p1++); *p1++ *= x; *p1 *= x; p1--;
  x = -(*p0++);
  /* debugger set p0[-1] = 0 */
  if (x != 0.0) {*p0++ += x*(*p1++); *p0 += x*(*p1); p0--; p1--;}
  x = -(*p2++);
  /* debugger set p2[-1] = 0 */
  if (x != 0.0) {*p2++ += x*(*p1++); *p2 += x*(*p1); p2--; p1--;}
  x = *p2++;
  if (x == 0.0) 
    return 2; // rank = 2;
  y = fabs(x);
  if (y > maxpiv) maxpiv = y; else if (y < minpiv) minpiv = y;
  /* debugger set p2[-1] = 1 */
  *p2 /= x;
  x = -(*p1++);  if (x != 0.0) *p1 += x*(*p2);
  /* debugger set p1[-1] = 0 */
  x = -(*p0++);  if (x != 0.0) *p0 += x*(*p2);
  /* debugger set p0[-1] = 0 */
  *x_addr = workarray[3];
  if (i) {
    *y_addr = workarray[11];
    *z_addr = workarray[7];
  }
  else {
    *y_addr = workarray[7];
    *z_addr = workarray[11];
  }
  *pivot_ratio = minpiv/maxpiv;
  return 3;
}



struct tagON_SORT_CONTEXT
{
  void* users_context;
  const unsigned char* qdata;
  int (*compar2)(const void*,const void*);
  int (*compar3)(const void*,const void*,void*);
};

static int qicompar2(void* p, const void* a, const void* b)
{
  return ((struct tagON_SORT_CONTEXT*)p)->compar2(
    ((struct tagON_SORT_CONTEXT*)p)->qdata + *((unsigned int*)a),
    ((struct tagON_SORT_CONTEXT*)p)->qdata + *((unsigned int*)b)
    );
}

static int qicompar3(void* p, const void* a, const void* b)
{
  return ((struct tagON_SORT_CONTEXT*)p)->compar3(
    ((struct tagON_SORT_CONTEXT*)p)->qdata + *((unsigned int*)a),
    ((struct tagON_SORT_CONTEXT*)p)->qdata + *((unsigned int*)b),
    ((struct tagON_SORT_CONTEXT*)p)->users_context
    );
}

void
ON_Sort( ON::sort_algorithm method, 
         int* index, 
         const void* data, 
         size_t count, 
         size_t sizeof_element, 
         int (*compar)(const void*,const void*)
         )
/*****************************************************************************
Sort an index
 
INPUT:
  method
    ON::quick_sort (generally best) or ON::heap_sort
  data
    pointer passed to compar() function
  index, count
    index is array of count integers. This function takes care of initializing 
    the input array.
  int compar(void* data, int i, int j)   ( 0 <= i,j < count )
    function that returns 
      -1: datum i is less than datum j
       0: datum i is equal to datum j
       1: datum i is greater than datum j
OUTPUT:
  index
    Sorted so that datum at index[i] is less than or equal to datum at index[i+1]  
COMMENTS:
  If you want the data sorted in place, then use ON_qsort() or ON_hsort().
EXAMPLE:
  // ...
REFERENCE:
  KNUTH
*/
{
  tagON_SORT_CONTEXT context;
  unsigned int* idx;
  const void* tmp;
  unsigned int i, j, k, tmpi, icount, isizeof_element;
  
  if (count < 1 || 0 == index || sizeof_element <= 0) 
  {
    return;
  }
  if (1 == count) 
  {
    index[0]=0;
    return;
  }

  isizeof_element = (unsigned int)sizeof_element; // (int) converts 64 bit size_t
  icount = (unsigned int)count;
  idx = (unsigned int*)index; // convert to unsigned int
  
  for ( i = 0; icount--; i += isizeof_element )
  {
    *idx++ = i;
  }

  memset(&context,0,sizeof(context));
  context.qdata = (const unsigned char*)data;
  context.compar2 = compar;
  idx    = (unsigned int*)index; // convert to unsigned int
  if ( ON::quick_sort == method )
  {
    ON_qsort(idx,count,sizeof(idx[0]),qicompar2,&context);
  }
  else
  {
    // heap sort
    icount = (unsigned int)count;

    k = icount >> 1;
    icount--;
    for (;;) 
    {
      if (k) 
      {
        tmpi = idx[--k];
        tmp  = context.qdata + tmpi;
      }
      else 
      {
        tmpi = idx[icount];
        tmp  = context.qdata + tmpi;
        idx[icount] = idx[0];
        if (!(--icount)) 
        {
          idx[0] = tmpi;
          break;
        }
      }
      i = k;
      j = (k<<1) + 1;
      while (j <= icount) 
      {
        if (j < icount && context.compar2(context.qdata + idx[j], context.qdata + idx[j+1]) < 0) 
        {
          j++;
        }
        if (context.compar2(tmp,context.qdata + idx[j]) < 0) 
        {
          idx[i] = idx[j];
          i = j;
          j = (j<<1) + 1;
        } 
        else 
        {
          j = icount + 1;
        }
      }
      idx[i] = tmpi;
    }
  }

  for (i = (unsigned int)count; i--; idx++ )
  {
    *idx /= isizeof_element;
  }
}

void
ON_Sort( ON::sort_algorithm method, 
         int* index, 
         const void* data, 
         size_t count, 
         size_t sizeof_element, 
         int (*compar)(const void*,const void*,void*),
         void* p
         )
{
  tagON_SORT_CONTEXT context;
  unsigned int* idx;
  const void* tmp;
  unsigned int i, j, k, tmpi, icount, isizeof_element;
  
  if (count < 1 || 0 == index || sizeof_element <= 0) 
  {
    return;
  }
  if (1 == count) 
  {
    index[0]=0;
    return;
  }

  isizeof_element = (unsigned int)sizeof_element; // (int) converts 64 bit size_t
  icount = (unsigned int)count;
  idx = (unsigned int*)index; // convert to unsigned int
  
  for ( i = 0; icount--; i += isizeof_element )
  {
    *idx++ = i;
  }

  memset(&context,0,sizeof(context));
  context.users_context = p;
  context.qdata = (const unsigned char*)data;
  context.compar3 = compar;
  idx    = (unsigned int*)index; // convert to unsigned int
  if ( ON::quick_sort == method )
  {
    ON_qsort(idx,count,sizeof(idx[0]),qicompar3,&context);
  }
  else
  {
    // heap sort
    icount = (unsigned int)count;

    k = icount >> 1;
    icount--;
    for (;;) 
    {
      if (k) 
      {
        tmpi = idx[--k];
        tmp  = context.qdata + tmpi;
      }
      else 
      {
        tmpi = idx[icount];
        tmp  = context.qdata + tmpi;
        idx[icount] = idx[0];
        if (!(--icount)) 
        {
          idx[0] = tmpi;
          break;
        }
      }
      i = k;
      j = (k<<1) + 1;
      while (j <= icount) 
      {
        if (j < icount && context.compar3(context.qdata + idx[j], context.qdata + idx[j+1], context.users_context) < 0) 
        {
          j++;
        }
        if (context.compar3(tmp,context.qdata + idx[j], context.users_context) < 0) 
        {
          idx[i] = idx[j];
          i = j;
          j = (j<<1) + 1;
        } 
        else 
        {
          j = icount + 1;
        }
      }
      idx[i] = tmpi;
    }
  }

  for (i = (unsigned int)count; i--; idx++ )
  {
    *idx /= isizeof_element;
  }
}

static void ON_hsort_str(char **e, size_t nel)
{
  size_t
    i_end,k;
  char
    *e_tmp;

  if (nel < 2) return;
  k = nel >> 1;
  i_end = nel-1;
  for (;;) {
    if (k) {
      --k;
      e_tmp = e[k];
    } else {
      e_tmp = e[i_end];
      e[i_end] = e[0];
      if (!(--i_end)) {
        e[0] = e_tmp;
        break;
      }
    }
    { size_t i, j;
      i = k;
      j = (k<<1) + 1;
      while (j <= i_end) {
        if (j < i_end && strcmp(e[j],e[j + 1])<0) j++;
        if (strcmp(e_tmp,e[j])<0) {
          e[i] = e[j];
          i = j;
          j = (j<<1) + 1;
        } else j = i_end + 1;
      }
      e[i] = e_tmp;
    }
  }
}

const int* ON_BinarySearchIntArray( int key, const int* base, size_t nel )
{
  if (nel > 0 && base )
  {
    size_t i;
    int d;

    // The end tests are not necessary, but they
    // seem to provide overall speed improvement
    // for the types of searches that call this
    // function.
    d = key-base[0];
    if ( d < 0 )
      return 0;
    if ( !d )
      return base;

    d = key-base[nel-1];
    if ( d > 0 )
      return 0;
    if ( !d )
      return (base + (nel-1));

    while ( nel > 0 )
    {
      i = nel/2;
      d = key - base[i];
      if ( d < 0 )
      {
        nel = i;
      }
      else if ( d > 0 )
      {
        i++;
        base += i;
        nel -= i;
      }
      else
      {
        return base+i;
      }
    }
  }
  return 0;
}

const unsigned int* ON_BinarySearchUnsignedIntArray( unsigned int key, const unsigned int* base, size_t nel )
{
  if (nel > 0 && base )
  {
    size_t i;
    unsigned int d;

    // The end tests are not necessary, but they
    // seem to provide overall speed improvement
    // for the types of searches that call this
    // function.
    d = base[0];
    if ( key < d )
      return 0;
    if ( key == d )
      return base;

    d = base[nel-1];
    if ( key > d )
      return 0;
    if ( key == d )
      return (base + (nel-1));

    while ( nel > 0 )
    {
      i = nel/2;
      d = base[i];
      if ( key < d )
      {
        nel = i;
      }
      else if ( key > d )
      {
        i++;
        base += i;
        nel -= i;
      }
      else
      {
        return base+i;
      }
    }
  }
  return 0;
}

const double* ON_BinarySearchDoubleArray( double key, const double* base, size_t nel )
{
  if (nel > 0 && base )
  {
    size_t i;
    double d;

    // The end tests are not necessary, but they
    // seem to provide overall speed improvement
    // for the types of searches that call this
    // function.
    d = key-base[0];
    if ( d < 0.0 )
      return 0;
    if ( 0.0 == d )
      return base;

    d = key-base[nel-1];
    if ( d > 0.0 )
      return 0;
    if ( 0.0 == d )
      return (base + (nel-1));

    while ( nel > 0 )
    {
      i = nel/2;
      d = key - base[i];
      if ( d < 0.0 )
      {
        nel = i;
      }
      else if ( d > 0.0 )
      {
        i++;
        base += i;
        nel -= i;
      }
      else
      {
        return base+i;
      }
    }
  }
  return 0;
}


int ON_Compare2dex( const ON_2dex* a, const ON_2dex* b)
{
  int d;
  if ( 0 == (d = (a->i - b->i)) )
  {
    d = a->j - b->j;
  }
  return d;
}


int ON_Compare3dex( const ON_3dex* a, const ON_3dex* b)
{
  int d;
  if ( 0 == (d = (a->i - b->i)) )
  {
    if ( 0 == (d = a->j - b->j) )
      d = a->k - b->k;
  }
  return d;
}


int ON_Compare4dex( const ON_4dex* a, const ON_4dex* b)
{
  int d;
  if ( 0 == (d = (a->i - b->i)) )
  {
    if ( 0 == (d = a->j - b->j) )
    {
      if ( 0 == (d = a->k - b->k) )
        d = a->l - b->l;
    }
  }
  return d;
}

static int compar_string(const void* pa, const void* pb)
{
  const char* sa = (const char*)pa;
  const char* sb = (const char*)pb;
  if ( !sa ) {
    return (sb) ? -1 : 0;
  }
  else if ( !sb ) {
    return 1;
  }
  return strcmp(sa,sb);
}

void
ON_SortStringArray(
        ON::sort_algorithm method,
        char** e,   // array of strings
        size_t nel    // length of array
        )
{
  if ( nel > 1 )
  {
    switch ( method ) 
    {
    case ON::heap_sort:
      ON_hsort_str( e, nel );
      break;
    case ON::quick_sort:
    default:
      ON_qsort( e, nel, sizeof(*e), compar_string );
      break;
    }
  }
}


/*
Description:
  Use the quotient rule to compute derivatives of a one parameter
  rational function F(t) = X(t)/W(t), where W is a scalar
  and F and X are vectors of dimension dim.
Parameters:
  dim - [in]
  der_count - [in] number of derivative (>=0)
  v_stride - [in] (>= dim+1)
  v - [in/out]
    v[] is an array of length (der_count+1)*v_stride.
    The input v[] array contains  derivatives of the numerator and
    denominator	functions in the order (X, W), (Xt, Wt), (Xtt, Wtt), ...
    In general, the (dim+1) coordinates of the d-th derivative 
    are in (v[n],...,v[n+dim]) where n = d*v_stride.
    In the output v[] array the derivatives of X are replaced with
    the derivatives of F and the derivatives of W are divided by
    w = v[dim].
Returns:
  True if input is valid; i.e., v[dim] != 0.
See Also:
  ON_EvaluateQuotientRule2
  ON_EvaluateQuotientRule3
*/

bool ON_EvaluateQuotientRule( int dim, int der_count, int v_stride, double *v )
{
  /*
  The quotient rule says the n-th derivative is

            (n)       (n)          (n)             (n-1)    (1)              (1)    (n-1)
          f  (t) =  x   (t)  -  (w  (t)*f(t) + n*w    (t)*f  (t) + ... + n*w  (t)*f    (t))
                    ---------------------------------------------------------------------
                                             w(t)


                                                                        (i)   (j)
         (The missing summands look like  ON_BinomialCoefficient(i,j)*w   * f    )
  */

  double
    wt, w2, *f, *x, *w;
  int
    i, j, n, df;

  wt = v[dim];
  if (wt == 0.0)
    return false;
  wt = 1.0/wt;
  i = (der_count+1)*v_stride;
  x = v;
  while(i--) *x++ *= wt;

  if (der_count) {
    // 1rst derivative - faster special case 
    f = v;            // f = func(t)
    x = v + v_stride; // x = numerator'(t)/w
    wt = -x[dim];     // wt = -denominator'(t)/w
    j = dim; while (j--) *x++ +=  wt* *f++;
    if (der_count> 1) {
      // 2nd derivative - faster special case 
      f = v + v_stride;
      x = f + v_stride;
      // v = func(t), f = func'(t), x = numerator''(t)/w, 
      // * wt = -2*denominator'(t)/w, w2 = denominator''(t)/w
      wt *= 2.0;
      w2 = -x[dim];
      j = dim; while(j--) *x++ += w2* *v++ + wt* *f++;
      if (der_count>2) {
        df = v_stride-dim;
        // higher derivatives use slower loop
        v -= dim;
        x = v + v_stride*2;
        for (n = 3; n <= der_count; n++) {
          // computing n-th derivative
          f = v;
          x += v_stride; // x = numerator^(n)/weight 
          w = v + n*v_stride + dim;
          for (i = 0; i < n; i++) {
            // f = value of i-th derivative 
            // w = ((n-i)-th derivative of denominator)/weight
            wt = -ON_BinomialCoefficient(n-i,i) * *w;
            w -= v_stride;
            j = dim; while (j--) *x++ += *f++ * wt;
            x -= dim;
            f += df;
          }
        }
      }
    }
  }

  return true;
}

/*
Description:
  Use the quotient rule to compute partial derivatives of a two parameter
  rational function F(s,t) = X(s,t)/W(s,t), where W is a scalar
  and F and X are vectors of dimension dim.
Parameters:
  dim - [in]
  der_count - [in] number of derivative (>=0)
  v_stride - [in] (>= dim+1)
  v - [in/out]
    v[] is an array of length (der_count+2)*(der_count+1)*v_stride.
    The input array contains derivatives of the numerator and denominator
		functions in the order X, W, Xs, Ws, Xt, Wt, Xss, Wss, Xst, Wst, Xtt, Wtt, ...
    In general, the (i,j)-th derivatives are in the (dim+1) entries of v[]
		v[k], ..., answer[k+dim], where	k = ((i+j)*(i+j+1)/2 + j)*v_stride.
    In the output v[] array the derivatives of X are replaced with
    the derivatives of F and the derivatives of W are divided by
    w = v[dim].
Returns:
  True if input is valid; i.e., v[dim] != 0.
See Also:
  ON_EvaluateQuotientRule
  ON_EvaluateQuotientRule3
*/

bool ON_EvaluateQuotientRule2( int dim, int der_count, int v_stride, double *v )
{
  double
    F, Fs, Ft, ws, wt, wss, wtt, wst, *f, *x;
  int
    i, j, n, q, ii, jj, Fn;

  // comment notation:
  //  X = value of numerator
  //  W = value of denominator
  //  F = X/W
  //  Xs = partial w.r.t. 1rst parameter
  //  Xt = partial w.r.t. 2nd parameter 
  //  ...
  // 

	// divide everything by the weight
  F = v[dim];
  if (F == 0.0)
    return false;
  F = 1.0/F;
  if ( v_stride > dim+1 )
  {
    i = ((der_count+1)*(der_count+2)>>1);
    x = v;
    j = dim+1;
    q = v_stride-j;
    while(i--)
    {
      jj = j;
      while(jj--)
        *x++ *= F;
      x += q;
    }
  }
  else
  {
    i = (((der_count+1)*(der_count+2))>>1)*v_stride;
    x = v;
    while(i--) *x++ *= F;
  }

  if (der_count) 
  {
		// first derivatives
    f = v;                    // f = F
    x = v + v_stride;         // x = Xs/w, x[v_stride] = Xt/w
    ws = -x[dim];             // ws = -Ws/w
    wt = -x[dim+v_stride];    // wt = -Wt/w
    j = dim; 
		while (j--) 
    {
			F = *f++;
			*x += ws*F;
			x[v_stride] += wt*F;
			x++;
		}

    if (der_count> 1) 
    {
      // 2nd derivatives
      f+= (v_stride-dim);	    // f = Fs, f[cvdim] = Ft 
      x = v + 3*v_stride;     // x = Xss, x[v_stride] = Xst, x[2*v_stride] = Xtt
      wss = -x[dim];          // wss = -wss/W
			wst = -x[v_stride+dim];	// wst = -Wst/W 
			n = 2*v_stride;
      wtt = -x[n+dim];        // wtt = -Wtt/w
      j = dim; 
			while(j--) 
      {
				F = *v++;
				Ft = f[v_stride];
				Fs = *f++;
				*x          += wss*F + 2.0*ws*Fs;     // Dss
				x[v_stride] += wst*F + wt*Fs + ws*Ft; // Dst
				x[n]        += wtt*F + 2.0*wt*Ft;     // Dtt
				x++;
			}

      if (der_count>2) 
      {
				// general loop for higher derivatives 
        v -= dim; // restore v pointer to input value
        f = v + 6*v_stride;    // f = Xsss
        for ( n = 3; n <= der_count; n++ ) 
        {
          for ( j = 0; j <= n; j++ ) 
          {
            // f = Ds^i Dt^j
            // 13 Jan 2005 Dale Lear bug fix - added missing a binomial coefficients.
            i = n-j;
            for ( ii = 0; ii <= i; ii++ ) 
            {
              ws = ON_BinomialCoefficient(ii,i-ii);
              for ( jj = ii?0:1; jj <= j; jj++ ) 
              {
                q = ii+jj;
                Fn = ((q*(q+1))/2 + jj)*v_stride+dim;
                // wt = -(i choose ii)*(j choose jj)*W(ii,jj)
                wt = -ws*ON_BinomialCoefficient(jj,j-jj)*v[Fn];
                q = n-q;
                Fn = ((q*(q+1))/2 + j-jj)*v_stride; // X(i-ii,j-jj) = v[Fn]
                for ( q = 0; q < dim; q++ )
                  f[q] += wt*v[Fn+q];                
              }
            }
            f += v_stride;
          }
        }
      }
    }
  }

  return true;
}


/*
Description:
  Use the quotient rule to compute partial derivatives of a 3 parameter
  rational function F(r,s,t) = X(r,s,t)/W(r,s,t), where W is a scalar
  and F and X are vectors of dimension dim.
Parameters:
  dim - [in]
  der_count - [in] number of derivative (>=0)
  v_stride - [in] (>= dim+1)
  v - [in/out]
    v[] is an array of length 
    v_stride*(der_count+1)*(der_count+2)*(der_count+3)/6.
    The input v[] array contains  derivatives of the numerator and
    denominator	functions in the order (X, W), (Xr, Wr), (Xs, Ws),
    (Xt, Wt), (Xrr, Wrr), (Xrs, Wrs), (Xrt, Wrt), (Xss, Wss), 
    (Xst, Wst), (Xtt, Wtt), ...
    In general, the (dim+1) coordinates of the input derivative 
    (Dr^i Ds^j Dt^k, i+j+k=d) are at v[n], ..., v[n+dim] where 

          n = v_stride*( d*(d+1)*(d+2)/6 + (j+k)*(j+k+1)/2 + k).

    In the output v[] array the derivatives of X are replaced with
    the derivatives of F and the derivatives of W are divided by
    w = v[dim].
Returns:
  True if input is valid; i.e., v[dim] != 0.
See Also:
  ON_EvaluateQuotientRule
  ON_EvaluateQuotientRule2
*/

bool ON_EvaluateQuotientRule3( int dim, int der_count, int v_stride, double *v )
{
  double
    F, Fr, Fs, Ft, wr, ws, wt, wrr, wrs, wrt, wss, wst, wtt, *f, *x;
  int
    i, j, k, n;

  // comment notation:
  //   X = value of numerator
  //   W = value of denominator
  //   F = X/W
  //   Xr = partial w.r.t. 1st parameter
  //   Xs = partial w.r.t. 2nd parameter
  //   Xt = partial w.r.t. 3rd parameter 
  //   ...

	// divide everything by the weight
  F = v[dim];
  if (F == 0.0)
    return false;
  F = 1.0/F;
  n = der_count+1;
  i = v_stride*n*(n+1)*(n+2)/6;
  x = v;
  while(i--) 
    *x++ *= F;

  if (der_count) 
  {
		// first derivatives
    f = v;                   // f = F
    x = v + v_stride;        // x = Xr/w, x[v_stride] = Xs/w
    wr = -x[dim];            // wr = -Wr/w
    ws = -x[dim+v_stride];   // ws = -Ws/w
    wt = -x[dim+2*v_stride]; // wt = -Wt/w
    j = dim; 
		while (j--) 
    {
			F = *f++;
			x[0]          += wr*F;
			x[v_stride]   += ws*F;
			x[2*v_stride] += wt*F;
			x++;
		}

    if (der_count> 1) 
    {
      // 2nd derivatives
      f = v;                // f = F, f[v_stride] = Fr, f[2*v_stride] = Fs, f[3*v_stride] = Ft
      x = v + 4*v_stride;   // x = Xrr, x[v_strde] = Xrs, ...
      wrr = -x[dim];
			wrs = -x[dim+v_stride];
			wrt = -x[dim+2*v_stride];
			wss = -x[dim+3*v_stride];
			wst = -x[dim+4*v_stride];
			wtt = -x[dim+5*v_stride];
      j = dim; 
			while(j--) 
      {
				Fr = f[v_stride];
				Fs = f[2*v_stride];
				Ft = f[3*v_stride];
				F = *f++;
				x[0]          += wrr*F + 2.0*wr*Fr;     // Drr
				x[v_stride]   += wrs*F + wr*Fs + ws*Fr; // Drs
				x[2*v_stride] += wrt*F + wr*Ft + wt*Fr; // Drt
				x[3*v_stride] += wss*F + 2.0*ws*Fs;     // Dss
				x[4*v_stride] += wst*F + ws*Ft + wt*Fs; // Dst
				x[5*v_stride] += wtt*F + 2.0*wt*Ft;     // Dtt
				x++;
			}

      if ( der_count > 2 )
      {
        int ii, jj, kk, Fn, q;
        f = v + 10*v_stride; // f = Xrrr
        for ( n = 3; n <= der_count; n++ ) 
        {
          for ( i = n; i >= 0; i-- ) 
          {
            for ( j = n-i; j >= 0; j-- )
            {
              k = n-i-j;
              // f = Dr^i Ds^j Dt^k
              for ( ii = 0; ii <= i; ii++ ) 
              {
                wr = ON_BinomialCoefficient(ii,i-ii);
                for ( jj = 0; jj <= j; jj++ ) 
                {
                  ws = wr*ON_BinomialCoefficient(jj,j-jj);
                  for ( kk = (ii||jj)?0:1; kk <= k; kk++ ) 
                  {
                    q = ii+jj+kk;
                    Fn=q-ii;
                    Fn = v_stride*( q*(q+1)*(q+2)/6  +  Fn*(Fn+1)/2  +  kk )+dim;
                    // wt = -(i choose ii)*(j choose jj)*(k choose kk)*W(ii,jj,kk)
                    wt = -ws*ON_BinomialCoefficient(kk,k-kk)*v[Fn];
                    q = n-q;
                    Fn = q-i+ii;
                    // F(i-ii,j-jj,k-kk) = v[Fn]
                    Fn = v_stride*( q*(q+1)*(q+2)/6  +  Fn*(Fn+1)/2  +  k-kk );
                    for ( q = 0; q < dim; q++ )
                      f[q] += wt*v[Fn+q];
                  }
                }
              }
              f += v_stride;
            }
          }
        }
      }
    }
  }

  return true;
}

//#define ON_TEST_EV_KAPPAS

#if defined(ON_TEST_EV_KAPPAS)

void ON_EPC_WARNING(const char* msg)
{
  ON_Warning(__FILE__,__LINE__,msg);
}

#else

#define ON_EPC_WARNING(msg) 

#endif

ON_BOOL32 ON_EvPrincipalCurvatures( 
        const ON_3dVector& Ds,
        const ON_3dVector& Dt,
        const ON_3dVector& Dss,
        const ON_3dVector& Dst,
        const ON_3dVector& Dtt,
        const ON_3dVector& N, // unit normal (use TL_EvNormal())
        double* gauss,        // = Gaussian curvature = kappa1*kappa2
        double* mean,         // = mean curvature = (kappa1+kappa2)/2
        double* kappa1,       // = largest principal curvature value (may be negative)
        double* kappa2,       // = smallest principal curvature value (may be negative)
        ON_3dVector& K1,      // kappa1 unit principal curvature direction
        ON_3dVector& K2       // kappa2 unit principal curvature direction
                              // output K1,K2,N is right handed frame
        )
{
  const double l = N.x*Dss.x + N.y*Dss.y + N.z*Dss.z;
  const double m = N.x*Dst.x + N.y*Dst.y + N.z*Dst.z;
  const double n = N.x*Dtt.x + N.y*Dtt.y + N.z*Dtt.z;

	return ON_EvPrincipalCurvatures(  Ds, Dt, l, m, n, N, 
											gauss, mean,  kappa1, kappa2,   K1,  K2 );   
}

ON_BOOL32 ON_EvPrincipalCurvatures( 
        const ON_3dVector& Ds,
        const ON_3dVector& Dt,
        double l,							// Second fundamental form coefficients
        double m,
        double n,
        const ON_3dVector& N, // unit normal (use TL_EvNormal())
        double* gauss,        // = Gaussian curvature = kappa1*kappa2
        double* mean,         // = mean curvature = (kappa1+kappa2)/2
        double* kappa1,       // = largest principal curvature value (may be negative)
        double* kappa2,       // = smallest principal curvature value (may be negative)
        ON_3dVector& K1,      // kappa1 unit principal curvature direction
        ON_3dVector& K2       // kappa2 unit principal curvature direction
                              // output K1,K2,N is right handed frame
        )
{

  //return ON_WRONG_EvPrincipalCurvatures( Ds, Dt, Dss, Dst, Dtt, N, gauss, mean, kappa1, kappa2, K1, K2 );

  //double e, f, g, l, m, n, jac, trace, det;
  double x, k1, k2;

  const double e = Ds.x*Ds.x + Ds.y*Ds.y + Ds.z*Ds.z;
  const double f = Ds.x*Dt.x + Ds.y*Dt.y + Ds.z*Dt.z;
  const double g = Dt.x*Dt.x + Dt.y*Dt.y + Dt.z*Dt.z;


  if (gauss) *gauss = 0.0;
  if (mean) *mean = 0.0;
  if (kappa1) *kappa1 = 0.0;
  if (kappa2) *kappa2 = 0.0;
  K1.x = K1.y = K1.z = 0.0;
  K2.x = K2.y = K2.z = 0.0;

  const double jac = e*g - f*f;
  if ( jac == 0.0 )
    return false;
  x = 1.0/jac;
  const double det   = (l*n - m*m)*x;           // = Gaussian curvature
  const double trace = (g*l - 2.0*f*m + e*n)*x; // = 2*(mean curvature)
  if (gauss) *gauss = det;
  if (mean) *mean = 0.5*trace;

  {
    // solve  k^2 - trace*k + det = 0 to get principal curvatures
    //    k = 0.5*(trace +/-sqrt(trace*trace - 4.0*det)
    x = trace*trace;
    double tol = fabs(det)*1.0e-12; // 31 March 2008 Dale Lear - added tol to fix 32091
    if ( x < 4.0*det-tol ) 
    {
      if ( det <= ON_EPSILON )
      {
        k1 = k2 = 0.0;
        if (gauss) *gauss = 0.0;
        if (mean) *mean = 0.0;
      }
      else
      {
        return false;
      }
    }
    else if ( 0.0 == x )
    {
      if ( det > 0.0 )
        return false;
      k1 = sqrt(-det);
      k2 = -k1;
    }
    else
    {
      x = 4.0*det/x;
      if (x > 1.0)
        x = 1.0;
      // k1 = root with largest absolute value
      k1 = 0.5*fabs(trace)*(1.0 + sqrt(1.0 - x));
      if ( trace < 0.0 )
        k1 = -k1;
      // Calculate k2 this way instead of using
      // the numerically sloppy difference in the
      // standard quadratic formula.
      k2 = det/k1;
      if ( fabs(k2) > fabs(k1) )
      {
        // numerical nonsense
        k2 = (det < 0.0) ? -k1 : k1;
      }
    }

    if ( kappa1 )
      *kappa1 = k1;
    if ( kappa2 )
      *kappa2 = k2;

#if defined(ON_TEST_EV_KAPPAS)
    double gggg = k1*k2;
    double tttt = k1+k2;
    if ( fabs(gggg - det) > 1.0e-4*fabs(det) )
    {
      ON_EPC_WARNING("ON_EvPrincipalCurvatures() Det(shape op) != gaussian");
    }
    if ( fabs(tttt - trace) > 1.0e-4*fabs(trace) )
    {
      ON_EPC_WARNING("ON_EvPrincipalCurvatures() Trace(shape op) != trace");
    }

    double zzz1 = k1*k1 - trace*k1 + det;
    double zzz2 = k2*k2 - trace*k2 + det;
    if ( fabs(zzz1) > (fabs(trace)+fabs(det))*1e-10 )
    {
      ON_EPC_WARNING("ON_EvPrincipalCurvatures() k1 is bogus");
    }
    if ( fabs(zzz2) > (fabs(trace)+fabs(det))*1e-10 )
    {
      ON_EPC_WARNING("ON_EvPrincipalCurvatures() k2 is bogus");
    }
#endif


    {
      int bUmbilic = true;
      if ( fabs(k1-k2) > 1.0e-6*(fabs(k1) + fabs(k2)) ) 
      {
        // different principle curvatures - see if we can get an answer
        int ki, bFixK1, bFixK2;
        double a, b, c, d, kk, x, y, len1, len2, E[2];

        bUmbilic = false;

        // use ShapeOp(Ds) = d(N) along s, etc., to figure out that with respect
        // to Du,Dv basis for tangent space, ShapeOp is given by the matrix
        //
        //            a  b
        // ShapeOp = 
        //            c  d
        // 
        // where and a,b,c,d are ..

        x = 1.0/jac;
        a = (g*l - f*m)*x;
        b = (g*m - f*n)*x;
        c = (e*m - f*l)*x;
        d = (e*n - f*m)*x;

#if defined(ON_TEST_EV_KAPPAS)
        //det   = (l*n - m*m)*x;           // = Gaussian curvature
        //trace = (g*l - 2.0*f*m + e*n)*x; // = 2*(mean curvature)
        double ggg = a*d - b*c;
        double ttt = a+d;
        if ( fabs(ggg - det) > 1.0e-4*fabs(det) )
        {
          ON_EPC_WARNING("ON_EvPrincipalCurvatures() Det(shape op) != gaussian");
        }
        if ( fabs(ttt - trace) > 1.0e-4*fabs(trace) )
        {
          ON_EPC_WARNING("ON_EvPrincipalCurvatures() Trace(shape op) != trace");
        }
#endif

        //  Since I'm looking for eigen vectors, I can ignore scale factor "x".
        // So I need to solve
        //
        //          a   b 
        //                *  Di = ki*Di
        //          c   d
        //
        // and set Ki = Di[0]*Ds + Di[1] *Dt;
        //
        len1 = len2 = 0.0;
        for ( ki = 0; ki < 2; ki++ ) 
        {
          //                   a-ki   b
          // The matrix
          //                    c    d-ki
          //
          // should have rank = 1.  This means (a-ki, b) and
          // (c,d-ki) must be linearly dependent.   The code
          // below sets (x,y) = the (best) average of the two 
          // vectors, and sets Ki = y*Ds - x*Dt.
          kk = (ki) ? k2 : k1;

#if defined(ON_TEST_EV_KAPPAS)
          x = (a-kk)*(d-kk) - b*c; // kk = eigen value of ShapeOp means
                                   // x should be zero
          if ( fabs(x) > 1.0e-8 )
          {
            if ( 0==ki )
            {
              ON_EPC_WARNING("ON_EvPrincipalCurvatures() |det(shape op - [k1,k1])| > 1.0e-8");
            }
            else
            {
              ON_EPC_WARNING("ON_EvPrincipalCurvatures() |det(shape op - [k2,k2])| > 1.0e-8");
            }
          }
#endif


          if ( (a-kk)*c + b*(d-kk) >= 0.0 ) 
          {
            x = (a-kk+c);
            y = (b+d-kk);
          }
          else {
            x = (a-kk-c);
            y = (b-d+kk);
          }

          E[0] = -y; E[1] = x;
          
#if defined(ON_TEST_EV_KAPPAS)
          // debugging check: should have shapeE[] = kk*E[]
          double shapeE[2];
          shapeE[0] = a*E[0] + b*E[1];
          shapeE[1] = c*E[0] + d*E[1];

          x = shapeE[0] - kk*E[0];
          y = shapeE[1] - kk*E[1];

          if ( fabs(x) > 1.0e-8 || fabs(y) > 1.0e-8 )
          {
            if ( 0==ki )
            {
              ON_EPC_WARNING("ON_EvPrincipalCurvatures() (shape op k1 eigenvector is noisy).");
            }
            else
            {
              ON_EPC_WARNING("ON_EvPrincipalCurvatures() (shape op k2 eigenvector is noisy).");
            }
          }
#endif

          if ( ki == 0 )
          {
            K1 = E[0]*Ds + E[1]*Dt;
            len1 = K1.Length();
            if ( len1 > 0.0 ) 
              K1 *= (1.0/len1);
          }
          else if ( ki == 1 )
          {
            K2 = E[0]*Ds + E[1]*Dt;
            len2 = K2.Length();
            if ( len2 > 0.0 ) 
              K2 *= (1.0/len2);
          }
        }

        bFixK1 = bFixK2 = false;
        {
          // make sure K1 and K2 are perp to N.
          x = K1*N;
          if ( fabs(x) >= 1.0e-4 )
          {
            ON_EPC_WARNING("ON_EvPrincipalCurvatures() K1*N > 1.0e-4.");
            bFixK1 = true;
          }

          x = K2*N;
          if ( fabs(x) >= 1.0e-4 )
          {
            ON_EPC_WARNING("ON_EvPrincipalCurvatures() K2*N > 1.0e-4.");
            bFixK2 = true;
          }
        }

        if ( !bFixK1 && !bFixK2 ) 
        {
          // make sure K1 and K2 are perp.
          x = K1*K2;
          if ( fabs(x) >= 1.0e-4 ) 
          {
#if defined(ON_TEST_EV_KAPPAS)
            {
              static bool bSecondTry = false;
              if ( !bSecondTry )
              {
                ON_EPC_WARNING("ON_EvPrincipalCurvatures() K1*K2 > 0.1.");

                // 15 July 2005 - Dale Lear
                //        There is a bug in converting the 2d eigenvectors
                //        Into 3d K1 and K2.  I'll look into it later.
                bSecondTry = true;
                double ggg, mmm, kkk1, kkk2;
                ON_3dVector KKK1, KKK2;
                ON_EvPrincipalCurvatures(Ds,Dt,Dss,Dst,Dtt,N,
                                        &ggg,&mmm,&kkk1,&kkk2,KKK1,KKK2);
                bSecondTry = false;
              }
            }
#endif
            if ( len1 < len2 )
              bFixK1 = true;
            else
              bFixK2 = true;
          }
        }

        if ( bFixK1 || bFixK2 ) 
        {
          if ( bFixK1 && bFixK2 ) 
          {
            bUmbilic = true;
          }
          else if ( bFixK1 ) 
          {
            K1 = ON_CrossProduct( K2, N );
            K1.Unitize();
          }
          else if ( bFixK2 ) 
          {
            K2 = ON_CrossProduct( N, K1 );
            K2.Unitize();
          }
        }
      }

      if ( bUmbilic ) {
        // equal principle curvatures
        if ( e >= g ) {
          // Ds is longest derivative
          K1 = Ds;
          K1.Unitize();
        }
        else {
          // Dt is longest derivative
          K1 = Dt;
          K1.Unitize();
        }
        K2 = ON_CrossProduct( N, K1 );
        K2.Unitize();
      }
    }
  }
  return true;
}


ON_3dVector 
ON_NormalCurvature( 
    const ON_3dVector& S10, const ON_3dVector& S01,
    const ON_3dVector& S20, const ON_3dVector& S11, const ON_3dVector& S02,
    const ON_3dVector& UnitNormal, const ON_3dVector& UnitTangent )
/*****************************************************************************
Evaluate normal curvature from surface derivatives and direction
 
INPUT:
  S10, S01
    surface 1st partial derivatives
  S20, S11, S02
    surface 2nd partial derivatives
  SrfUnitNormal
    Unit normal to surface
  CrvUnitTangent
    3d unit tangent to the surface
OUTPUT:
   The return value is the normal curvature vector for the surface in the direction of UnitTangent
   Normal curvature vector ((anti)parallel to UnitNormal).
   The scalar normal curvature is equal to NormalCurvature o UnitNormal.
*****************************************************************************/
{
  ON_3dVector NormalCurvature, D2, T, K;
  double a, b, d, e, pr;
  int rc;

  a = b = 0.0;
  // solve T = a*S10 + b*S01
  rc = ON_Solve3x2( S10, S01, UnitTangent.x, UnitTangent.y, UnitTangent.z, 
                    &a, &b, &e, &pr );
  if (rc < 2) {
    NormalCurvature.Zero();
  }
  else {
    // compute 2nd derivative of 3d curve(t) = srf( u0 + a*t, v0 + b*t ) at t=0
    D2 = a*a*S20 + 2.0*a*b*S11 + b*b*S02;

    // compute curvature of 3d curve(t) = srf( u0 + a*t, v0 + b*t ) at t=0
    ON_EvCurvature( UnitTangent, D2, T, K );

    // get component of K parallel to N
    d = K*UnitNormal;
    NormalCurvature = d*UnitNormal;
  }

  return NormalCurvature;
}



bool 
ON_GetPolylineLength(
    int dim, 
    ON_BOOL32 is_rat,
    int count, 
    int stride, 
    const double* P, 
    double* length )
/* first point  = {P[0], ... }
 * second point = {P[stride], ... }
 */
{
#define SUM_SIZE 128
  double       L, d, dd, w0, w1; 
	const double *p0, *p1;
  double       *sum;
	int          j, i, sumi;

  if (length)
		*length = 0.0;

	if (stride == 0) stride = (dim+is_rat);
  if ( dim < 1 || count < 2 || stride < ((is_rat)?dim+1:dim) || !P || !length )
    return false;


	p1 = P;
  L = 0.0;

  sumi = count/SUM_SIZE;
  sumi++;
  sum = (double*)alloca( sumi*sizeof(*sum) );
  sumi = 0;

  if (is_rat) {
    w1 = p1[dim];
    if (w1 == 0.0) {
      ON_ERROR("ON_GetPolylineLength: Zero weight");
      return false;
    }
    w1 = 1.0/w1;
    for ( i = 1; i < count; i++ ) {
      w0 = w1;
			p0 = p1;
			p1 = p1 + stride;
      w1 = p1[dim];
      if (w1 == 0.0) {
        ON_ERROR("ON_GetPolylineLength: Zero weight");
        return false;
      }
      w1 = 1.0/w1;
			dd = 0.0;
      for (j = 0; j < dim; j++) {
        d = w0* p0[j] - w1*p1[j];
        dd += d*d;
      }
      L += sqrt(dd);
      if ( !(i % SUM_SIZE) ) {
        sum[sumi++] = L;
        L = 0.0;
      }
    }
  }
  else {
    for (i = 1; i < count; i++) {
			p0 = p1;
			p1 = p1 + stride;
      dd = 0.0;
      for (j = 0; j < dim; j++) {
        d = p1[j] - p0[j];
        dd += d*d;
      }
      L += sqrt(dd);
      if ( !(i % SUM_SIZE) ) {
        sum[sumi++] = L;
        L = 0.0;
      }
    } 
  }

  for (i = 0; i < sumi; i++) {
    L += sum[i];
  }

  *length = L;

  return true;
#undef SUM_SIZE
}



ON_Evaluator::ON_Evaluator( 
                           int parameter_count,
                           int value_count,
                           const ON_Interval* domain,
                           const bool* periodic
                           )
             : m_parameter_count(parameter_count),
               m_value_count(value_count>0?value_count:parameter_count)
{
  int i;

  if (domain)
  {
    m_domain.Reserve(m_parameter_count);
    for ( i = 0; i < parameter_count; i++ )
      m_domain.Append(domain[i]);

    if (periodic )
    {
      for ( i = 0; i < parameter_count; i++ )
      {
        if  (periodic[i])
        {
          m_bPeriodicParameter.Reserve(m_parameter_count);
          for ( i = 0; i < m_parameter_count; i++ )
          {
            m_bPeriodicParameter.Append(periodic[i]?true:false);
          }
          break;
        }
      }
    }
  }
}

ON_Evaluator::~ON_Evaluator()
{
}


int ON_Evaluator::EvaluateHessian(
     const double* parameters,
     double* value,
     double* gradient,
     double** hessian
     )
{
  if ( m_parameter_count==1)
  {
    if ( 0 != gradient )
    {
      // we have enough information to get the 
      // value and the gradient
      Evaluate(parameters,value,&gradient);
    }

    if ( 0 != hessian )
    {
      // zero out the hessian matrix
      int i;
      for ( i = 0; i < m_parameter_count; i++ )
      {
        memset(hessian[0],0,m_parameter_count*sizeof(hessian[i][0]));
      }
    }
  }

  // -1 indicates that this evaluator does not support hessian evaluation.
  return -1;
}


bool ON_Evaluator::FiniteDomain() const
{
  return ((m_parameter_count==m_domain.Count() && m_parameter_count>0) 
          ? true
          :false
          );
}

bool ON_Evaluator::Periodic(
  int parameter_index
  ) const
{
  return ((m_parameter_count==m_bPeriodicParameter.Count() && m_parameter_count>0) 
          ? m_bPeriodicParameter[parameter_index] 
          : false
          );
}

ON_Interval ON_Evaluator::Domain(
  int parameter_index
  ) const
{
  return ((m_parameter_count==m_domain.Count() && m_parameter_count>0) 
          ? m_domain[parameter_index] 
          : ON_Interval(-1.0e300,1.0e300)
          );
}


 
double ON_Max(double a, double b){
  return (a<b)? b:a;
}

 
float ON_Max(float a, float b){
  return (a<b)? b:a;
}

 
int ON_Max(int a, int b){
  return (a<b)? b:a;
}

 
double ON_Min(double a, double b){
  return (a<b)? a:b;
}

 
float ON_Min(float a, float b){
  return (a<b)? a:b;
}

 
int ON_Min(int a, int b){
  return (a<b)? a:b;
}

int ON_Round(double x)
{
  // 12 March 2009 Dale Lear
  //   Added error checking to this function.
  //   Do not call this function in any opennurbs code, tl code
  //   or any other code that does critical calculations.

  if (!ON_IsValid(x))
  {
    ON_ERROR("ON_Round - invalid input");
    return 0;
  }

  // Do not use "INT_MAX" - the define is not portable
  // and this code needs to compile and execute the same
  // way on all OSs.
  if ( fabs(x) >= 2147483647.0 )
  {
    ON_ERROR("ON_Round - integer overflow");
    return (x > 0.0) ? 2147483647 : -2147483647;
  }

  return (x>=0.0) ? ((int)(x+0.5)) : -((int)(0.5-x));
}

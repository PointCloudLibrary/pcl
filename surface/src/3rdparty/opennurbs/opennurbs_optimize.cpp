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



int ON_FindLocalMinimum(
                int (*f)(void*,double,double*,double*), void* farg,
                double ax, double bx, double cx,
                double rel_stepsize_tol, double abs_stepsize_tol, int max_it, 
                double *t_addr
                )
/* Use Brent's algorithm (with derivative) to Find a (local) minimum of a function
 *
 * INPUT:
 *   ax, bx, cx a bracketed minimum satisfying conditions 1 and 2.
 *      1) either ax < bx < cx or cx < bx < ax.
 *      2) f(bx) < f(ax) and f(bx) < f(ax).
 *   farg
 *      pointer passed to function f()
 *   f
 *      evaluation function with prototype
 *              int f(void* farg,double t,double* ft,double* dft)
 *      f(farg,t,&ft,&dft) should compute ft = value of function at t
 *      and dft = value of derivative at t.
 *      -1: failure
 *       0: success
 *       1: |f(x)| is small enough - TL_NRdbrent() will return *t_addr = x
 *          and the return code 1.
 *   rel_stepsize_tol, abs_stepsize_tol  (0 < rel_stepsize_tol < 1 and 0 < abs_stepsize_tol)
 *      rel_stepsize_tol is a fractional tolerance and abs_stepsize_tol is an absolute tolerance
 *      that determine the minimum step size for a given iteration.
 *        minimum delta t = rel_stepsize_tol*|t| + abs_stepsize_tol.
 *      When in doubt, use 
 *         rel_stepsize_tol = ON_EPSILON 
 *         abs_stepsize_tol = 1/2*(desired absolute precision for *t_addr).
 *   max_it ( >= 2)
 *      maximum number of iterations to permit (when in doubt use 100)
 *      Closest Point to bezier minimizations typically take < 30
 *      iterations.
 *
 * OUTPUT:
 *   *t_addr abcissa of a local minimum between ax and cx.
 *       0: failure
 *       1: success
 *       2: After max_iteration_cnt iterations the tolerance restrictions
 *          where not satisfied.  Try increasing max_it, rel_stepsize_tol and/or abs_stepsize_tol
 *          or use the value of (*t_addr) with extreme caution.
 */
{
  // See Numerical Recipes in C's dbrent() for a description of the basic algorithm
  int rc,ok1,ok2;
  double a,b,d,d1,d2,du,dv,dw,dx,e,fu,fv,fw,fx,olde,tol1,tol2,u,u1,u2,v,w,x,xm;

  d=e=0.0;

  if ( 0 == t_addr )
  {
    ON_ERROR("t_addr is NULL");
    return 0;
  }

  *t_addr = bx;

  if ( max_it < 2 )
  {
    ON_ERROR("max_it must be >= 2");
    return 0;
  }
  if ( !ON_IsValid(rel_stepsize_tol) || rel_stepsize_tol <= 0.0 || rel_stepsize_tol >= 1.0 )
  {
    ON_ERROR("rel_stepsize_tol must be strictly between 0.0 and 1.0");
    return 0;
  }
  if ( !ON_IsValid(abs_stepsize_tol) || abs_stepsize_tol <= 0.0 )
  {
    ON_ERROR("abs_stepsize_tol must be > 0");
    return 0;
  }

  a=(ax < cx ? ax : cx);
  b=(ax > cx ? ax : cx);
  x=w=v=bx;
  rc = f(farg,x,&fx,&dx);
  if (rc) {
     // f() returned nonzero return code which means we need to bailout
    if ( rc < 0 ) {
      ON_ERROR("ON_FindLocalMinimum() f() failed to evaluate.");
    }
    *t_addr = x;
    return rc>0 ? 1 : 0; // return 1 means f() said result is good enough, return = 0 means f() failed
  }
  fw=fv=fx;
  dw=dv=dx;
  while(max_it--) {
    xm=0.5*(a+b);
    tol1=rel_stepsize_tol*fabs(x)+abs_stepsize_tol;
    tol2=2.0*tol1;
    if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
      // further adjustments to x are smaller than stepsize tolerance
      *t_addr=x;
      return 1;
    }
    if (fabs(e) > tol1) {
      d1=2.0*(b-a);
      d2=d1;
      if (dw != dx) d1=(w-x)*dx/(dx-dw);
      if (dv != dx) d2=(v-x)*dx/(dx-dv);
      u1=x+d1;
      u2=x+d2;
      ok1 = (a-u1)*(u1-b) > 0.0 && dx*d1 <= 0.0;
      ok2 = (a-u2)*(u2-b) > 0.0 && dx*d2 <= 0.0;
      olde=e;
      e=d;
      if (ok1 || ok2) {
        if (ok1 && ok2)
          d=(fabs(d1) < fabs(d2) ? d1 : d2);
        else if (ok1)
          d=d1;
        else
          d=d2;
        if (fabs(d) <= fabs(0.5*olde)) {
          u=x+d;
          if (u-a < tol2 || b-u < tol2)
            {d = (xm >= x) ? tol1 : -tol1;}
        } else {
          d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
        }
      } else {
        d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
      }
    } else {
      d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
    }
    if (fabs(d) >= tol1) {
      u=x+d;
      rc = f(farg,u,&fu,&du);
    }
    else {
      u = (d >= 0.0) ? x+tol1 : x-tol1;
      rc = f(farg,u,&fu,&du);
      if (rc >= 0 && fu > fx) {
        // tweaking x any more increases function value - x is a numerical minimum
        *t_addr=x;
        return 1;
      }
    }
    if (rc) {
      // f() returned nonzero return code which means we need to bailout
      if ( rc < 0 ) {
        ON_ERROR("ON_FindLocalMinimum() f() failed to evaluate.");
      }
      else {
        *t_addr = (fu < fx) ? u : x;
      }
      return rc>0 ? 1 : 0;
    }
    if (fu <= fx) {
      if (u >= x) a=x; else b=x;
      v=w;fv=fw;dv=dw;
      w=x;fw=fx;dw=dx;
      x=u;fx=fu;dx=du;
    } else {
      if (u < x) a=u; else b=u;
      if (fu <= fw || w == x) {
        v=w;fv=fw;dv=dw;
        w=u;fw=fu;dw=du;
      } else if (fu < fv || v == x || v == w) {
        v=u;fv=fu;dv=du;
      }
    }
  }
  *t_addr = x; // best known answer
  ON_ERROR("ON_FindLocalMinimum() failed to converge");
  return 2; // 2 means we failed to converge
}


ON_LocalZero1::ON_LocalZero1() 
               : m_t0(ON_UNSET_VALUE), m_t1(ON_UNSET_VALUE),
                 m_f_tolerance(0.0), m_t_tolerance(0.0),
                 m_k(NULL), m_k_count(0)
{}

ON_LocalZero1::~ON_LocalZero1()
{}

ON_BOOL32
ON_LocalZero1::BracketZero( double s0, double f0, 
                             double s1, double f1,
                             int level )
{
  double s, f, d;

  // private helper for FindSearchDomain()
  if (    (f0 <= 0.0 && f1 >= 0.0) || (f0 >= 0.0 && f1 <= 0.0)
       || fabs(f0) <= m_f_tolerance || fabs(f1) <= m_f_tolerance ) {
    m_t0 = s0;
    m_t1 = s1;
    return true;
  }

  if ( level++ <= 8 ) {
    s = 0.5*s0+s1;
    if ( s0 < s && s < s1 && Evaluate(s,&f,&d,0) ) {
      if ( f*d >= 0.0 ) {
        // search left side first
        if ( BracketZero(s0,f0,s,f,level ) ) {
          m_s0 = s0;
          m_f0 = f0;
          m_s1 = s;
          m_f1 = f;
          return true;
        }
        if ( BracketZero(s,f,s1,f1,level ) ) {
          m_s0 = s;
          m_f0 = f;
          m_s1 = s1;
          m_f1 = f1;
          return true;
        }
      }
      else {
        // search right side first
        if ( BracketZero(s,f,s1,f1,level ) ) {
          m_s0 = s;
          m_f0 = f;
          m_s1 = s1;
          m_f1 = f1;
          return true;
        }
        if ( BracketZero(s0,f0,s,f,level ) ) {
          m_s0 = s0;
          m_f0 = f0;
          m_s1 = s;
          m_f1 = f;
          return true;
        }
      }
    }
  }
  return false;
}

ON_BOOL32
ON_LocalZero1::BracketSpan( double s0, double f0, double s1, double f1 )
{
  int i0, i1, i;
  double fm, fp;
  ON_BOOL32 rc = true;
  if ( m_k && m_k_count >= 3 ) {
    i0 = ON_SearchMonotoneArray(m_k,m_k_count,s0);
    if ( i0 < 0 )
      i0 = 0;
    i1 = ON_SearchMonotoneArray(m_k,m_k_count,s1);
    if ( i1 >= m_k_count )
      i1 = m_k_count-1;
    while ( i1 >= 0 && s1 == m_k[i1] ) {
      i1--;
    }
    i0++;
    while ( i0 < m_k_count-1 && m_k[i0] == m_k[i0+1] )
      i0++;
    if ( i0 <= i1 ) {
      // we have s0 < m_k[i0] <= ... <= m_k[i1] < s1
      Evaluate( m_k[i0], &fm, NULL,-1 ); // gaurd against C0 discontinuities
      Evaluate( m_k[i0], &fp, NULL, 1 );
      if ( (f0 <= 0.0 && fm >= 0.0) || (f0 >= 0.0 && fm <= 0.0) ) {
        m_s1 = m_k[i0];
        m_f1 = fm;
      }
      else if ( (f1 <= 0.0 && fp >= 0.0) || (f1 >= 0.0 && fp <= 0.0) ) {
        m_s0 = m_k[i0];
        m_f0 = fp;
        if ( i0 < i1 ) {
          Evaluate( m_k[i1], &fm, NULL, -1 );
          Evaluate( m_k[i1], &fp, NULL,  1 );
          if ( (f1 <= 0.0 && fp >= 0.0) || (f1 >= 0.0 && fp <= 0.0) ) {
            m_s0 = m_k[i1];
            m_f0 = fp;
          }
          else if ( (f0 <= 0.0 && fm >= 0.0) || (f0 >= 0.0 && fm <= 0.0) ) {
            m_s1 = m_k[i1];
            m_f1 = fm;
            // we have s0 = m_k[i0] < m_k[i1] = s1 and a bonafide sign change
            while (i0+1 < i1) {
              // bisect m_k[i0],...,m_k[i1] until we get a sign change between
              // m_k[i],m_k[i+1].  We need to do this in order to make sure
              // we are passing a C2 function to the repeated zero finders.
              i = (i0+i1)>>1;
              Evaluate( m_k[i], &fm, NULL, -1 );
              Evaluate( m_k[i], &fp, NULL,  1 );
              if ( (f0 <= 0.0 && fm >= 0.0) || (f0 >= 0.0 && fm <= 0.0) ) {
                m_s1 = m_k[i];
                m_f1 = fm;
                i1 = i;
                while ( i1>0 && m_k[i1-1]==m_k[i1])
                  i1--;
              }
              else if ( (f1 <= 0.0 && fp >= 0.0) || (f1 >= 0.0 && fp <= 0.0) ) {
                m_s0 = m_k[i];
                m_f0 = fp;
                i0 = i;
                while ( i0 < m_k_count-2 && m_k[i0] == m_k[i0+1] )
                  i0++;
              }
              else {
                // discontinuous sign change across m_k[i]
                rc = false;
                break;
              }
            }
          }
          else {
            // discontinuous sign change across m_k[i1]
            rc = false;
          }
        }
      }
      else {
        // discontinuous sign change across m_k[i0]
        rc = false;
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_LocalZero1::FindZero( double* t )
{
  // Find values of m_t0 and m_t1 between t0 and t1 such that
  // f(m_t0) and f(m_t1) have different signs
  ON_BOOL32 rc = ( m_t0 == ON_UNSET_VALUE || m_t0 == ON_UNSET_VALUE ) ? true : false;

  if ( rc ) {
    if ( m_t0 < m_t0 ) {
      m_s0 = m_t0;
      m_s1 = m_t0;
    }
    else {
      m_s0 = m_t1;
      m_s1 = m_t0;
      if ( m_t0 == m_t1 ) {
        if ( Evaluate( m_t0, &m_f0, NULL, 1 ) ) {
          m_f1 = m_f0;
          if ( fabs(m_f0) <= m_f_tolerance ) {
            *t = m_t0;
            return true;
          }
        }
        ON_ERROR("Illegal input");
        return false;
      }
    }
  }

  if (rc)
    rc = Evaluate( m_s0, &m_f0, NULL, 1 );
  if (rc)
    rc = Evaluate( m_s1, &m_f1, NULL, -1 );

  if (rc)
    rc = BracketZero( m_s0, m_f0, m_s1, m_f1 );
  if ( rc ) {
    if ( fabs(m_f0) <= m_f_tolerance && fabs(m_f0) <= fabs(m_f1) ) {
      // |f(s0)| <= user specified stopping tolerance
      *t = m_s0;
    }
    else if ( fabs(m_f1) <= m_f_tolerance ) {
      // |f(s1)| <= user specified stopping tolerance
      *t = m_s1;
    }
    else {
      if (rc)
        rc = BracketSpan( m_s0, m_f0, m_s1, m_f1 );
      if (rc)
        rc = NewtonRaphson( m_s0, m_f0, m_s1, m_f1, 128, t );
    }
  }
  if (!rc) {
    ON_ERROR("ON_LocalZero1::FindZero() failed");
  }

  return rc;
}

ON_BOOL32 ON_LocalZero1::NewtonRaphson( double s0, double f0,
                                    double s1, double f1,
                                    int maxit, double* t )
{
  // private function - input must satisfy
  //
  // 1) t is not NULL
  //
  // 2) maxit >= 2
  //
  // 3) s0 != s1, 
  //
  // 4) f0 = f(s0), f1 = f(s1), 
  //
  // 5) either f0 < 0.0 and f1 > 0.0, or f0 > 0.0 and f1 < 0.0
  //
  // 6) f() is C0 on [s0,s1] and C2 on (s0,s1)
  //
  // 7) ds_tolerance >= 0.0 - the search for a zero will stop
  //    if the zero in bracketed in an interval that is no
  //    larger than ds_tolerance.
  //
  //    When in doubt, ds_tolerance = (fabs(s0) + fabs(s1))*ON_EPSILON
  //    works well.

  double s, f, d, x, ds, prevds;

  if ( fabs(f0) <= m_f_tolerance && fabs(f0) <= fabs(f1) ) {
    // |f(s0)| <= user specified stopping tolerance
    *t = s0;
    return true;
  }
  if ( fabs(f1) <= m_f_tolerance ) {
    // |f(s1)| <= user specified stopping tolerance
    *t = s1;
    return true;
  }

  if ( f0 > 0.0 ) {
    x = s0; s0 = s1; s1 = x;
    x = f0; f0 = f1; f1 = x;
  }

  s = 0.5*(s0+s1);
  if ( !Evaluate( s, &f, &d, 0 ) ) {
    *t = (fabs(f0) <= fabs(f1)) ? s0 : s1;
    return false;
  }

  if ( fabs(f) <= m_f_tolerance ) {
    // |f(s)| <= user specified stopping tolerance
    *t = s;
    return true;
  }

  if ( f1 <= 0.0 ) {
    *t = (fabs(f0) <= fabs(f1)) ? s0 : s1;
    return false;
  }

  ds = fabs(s1-s0);
  prevds = 0.0;

  while (maxit--) {
    if ( (f+(s0-s)*d)*(f+(s1-s)*d) > 0.0  // true if NR's line segment doesn't cross zero inside interval
         || fabs(2.0*f) > fabs(prevds*d)  // true if expected decrease in function value from previous step didn't happen
         ) {
      // bisect
      prevds = ds;
      ds = 0.5*(s1-s0);
      s = s0+ds;
      if ( s == s0 ) {
        // interval is too small to be divided using double division
        if ( fabs(f1) < fabs(f0) ) {
          s = s1;
        }
        *t = s;
        return true;
      }      
    }
    else {
      // Newton iterate
      prevds = ds;
      ds = -f/d;
      x = s;
      s += ds;
      if ( s == x ) {
        // Newton step size < smallest double than can be added to s
        if ( fabs(f0) < fabs(f) ) {
          f = f0;
          s = s0;
        }
        if ( fabs(f1) < fabs(f) ) {
          s = s1;
        }
        *t = s;
        return true;
      } 
    }

    if ( !Evaluate( s, &f, &d, 0 ) ) {
      *t = (fabs(f0) <= fabs(f1)) ? s0 : s1; // emergency bailout
      return false;
    }

    if ( fabs(f) <= m_f_tolerance ) {
      // |f(s)| <= user specified stopping tolerance
      if ( fabs(f0) < fabs(f) ) {
        f = f0;
        *t = s0;
      }
      if ( fabs(f1) < fabs(f) ) {
        *t = s1;
      }
      return true;
    }

    if ( f < 0.0 ) {
      f0 = f; // f0 needed for emergency bailout
      s0 = s;
    }
    else { // f > 0.0
      f1 = f; // f1 needed for emergency bailout
      s1 = s;
    }

    if ( fabs(s1-s0) <= m_t_tolerance ) {
      // a root has been bracketed to an interval that is small enough
      // to satisify user.
      *t = (fabs(f0) <= fabs(f1)) ? s0 : s1;
      return true;
    }
  }

  *t = (fabs(f0) <= fabs(f1)) ? s0 : s1; // emergency bailout
  return false;
}


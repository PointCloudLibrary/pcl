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

#if !defined(OPENNURBS_OPTIMIZE_INC_)
#define OPENNURBS_OPTIMIZE_INC_

// find a local minimum of a 1 parameter function
ON_BOOL32 ON_FindLocalMinimum( // returns 0 - failed to converge, 1 - success, 2 - failed to converge to requested tolerances
        int (*)(void*,double,double*,double*), // f(void*, double t, double* value, double* derivative );
        void*, // passed as the void* argument to the above function
        double, double, double, // ax,bx,cx, 3 abcissa  ax<bx<cx or ax>bx>cx, and
                                // f(bx) < f(ax), and f(bx) < f(cx)
        double, // tol > 0 (minimum relative step size (use ON_EPSILON when in doubt)
        double, // zeps > 0 (minimum absolute step size (use 1/2*(desired absolute precision))
        int,     // maximum number of iterations ( use 100 when in doubt)
        double*  // abcissa of local minimum returned here
        );

// find a local zero of a 1 parameter function
class ON_LocalZero1
{
public:
  ON_LocalZero1();
  virtual ~ON_LocalZero1();

  virtual
  ON_BOOL32 Evaluate( // returns true if successful
     double,  // evaluation parameter
     double*, // f(t) returned here - NULL never passed
     double*, // If not NULL, then f'(t) returned here
     int      // <  0: evaluate from below
              // >= 0: evaluate from above
  ) = 0;


  ON_BOOL32 FindZero( double* );  // Searches domain between m_to and m_t1
                             // domain for a root.  Returns true if
                             // a root is found.

  // m_t0 and m_t1 specify the domain to search and must satisfy 
  //
  //          1) m_t0 != m_t1
  //          2) f(m_t0) and f(m_t1) must have different signs
  //             or one must have absolute value <= m_f_tolerance
  double m_t0, m_t1; 

  double m_f_tolerance; // (>= 0.0)  If this value is > 0.0, then
                        // the search is terminated when a parameter
                        // "t" is found where |f(t)| <= m_f_tolerance.

  double m_t_tolerance; // (>= 0.0)  If this value is > 0.0, then
                        // the search is terminated when a parameter
                        // the root is bracketed in a domain with width
                        // <= m_t_tolerance.

  // m_k[] is either NULL or monotone increasing array of length m_k_count.
  //
  // This zero finder works on continuous piecewise c2 functions.
  // If the function is c2 on the interior of the domain 
  //
  //          [min(t0,t1), max(m_t0,m_t1)]
  //
  // then there is no need to initialize m_k[].  If the function
  // is not c2 on the domain in question, then the m_k[m_count] array
  // is a list of parameters that define the c2 domains.  When m_k[] 
  // is not NULL, m_count must be >= 2 and m_k[] must be monotone 
  // increasing and satisfy 
  //
  //          m_k[0] <= min(m_t0,m_t1) 
  //          and
  //          m_k[m_count-1] >= max(m_t0,m_t1).
  //
  // Duplicate values in m_k[] are permitted so that NURBS knot
  // vector arrays may be used directly.
  const double* m_k;
  
  // length of m_k[] array ( 0 or >= 2 ).
  int m_k_count;     

private:
  double m_s0, m_f0, m_s1, m_f1;
  ON_BOOL32 BracketZero(double,double,double,double,int=0);
  ON_BOOL32 BracketSpan(double,double,double,double);
  ON_BOOL32 NewtonRaphson( double, double, double, double, int, double* );
};

#endif


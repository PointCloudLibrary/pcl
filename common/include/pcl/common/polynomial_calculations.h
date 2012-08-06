/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_POLYNOMIAL_CALCULATIONS_H_
#define PCL_POLYNOMIAL_CALCULATIONS_H_

#include <pcl/common/eigen.h>
#include <pcl/common/bivariate_polynomial.h>

namespace pcl 
{
  /** \brief This provides some functionality for polynomials,
    *         like finding roots or approximating bivariate polynomials
    *  \author Bastian Steder 
    *  \ingroup common
    */
  template <typename real>
  class PolynomialCalculationsT 
  {
    public:
      // =====CONSTRUCTOR & DESTRUCTOR=====
      PolynomialCalculationsT ();
      ~PolynomialCalculationsT ();
      
      // =====PUBLIC STRUCTS=====
      //! Parameters used in this class
      struct Parameters
      {
        Parameters () : zero_value (), sqr_zero_value () { setZeroValue (1e-6);}
        //! Set zero_value
        void
        setZeroValue (real new_zero_value);

        real zero_value;       //!< Every value below this is considered to be zero
        real sqr_zero_value;   //!< sqr of the above
      };
      
      // =====PUBLIC METHODS=====
      /** Solves an equation of the form ax^4 + bx^3 + cx^2 +dx + e = 0
       *  See http://en.wikipedia.org/wiki/Quartic_equation#Summary_of_Ferrari.27s_method */
      inline void
      solveQuarticEquation (real a, real b, real c, real d, real e, std::vector<real>& roots) const;

      /** Solves an equation of the form ax^3 + bx^2 + cx + d = 0
       *  See http://en.wikipedia.org/wiki/Cubic_equation */
      inline void
      solveCubicEquation (real a, real b, real c, real d, std::vector<real>& roots) const;

      /** Solves an equation of the form ax^2 + bx + c = 0
       *  See http://en.wikipedia.org/wiki/Quadratic_equation */
      inline void
      solveQuadraticEquation (real a, real b, real c, std::vector<real>& roots) const;

      /** Solves an equation of the form ax + b = 0 */
      inline void
      solveLinearEquation (real a, real b, std::vector<real>& roots) const;
      
      /** Get the bivariate polynomial approximation for Z(X,Y) from the given sample points.
       *  The parameters a,b,c,... for the polynom are returned.
       *  The order is, e.g., for degree 1: ax+by+c and for degree 2: ax²+bxy+cx+dy²+ey+f.
       *  error is set to true if the approximation did not work for any reason
       *  (not enough points, matrix not invertible, etc.) */
      inline BivariatePolynomialT<real>
      bivariatePolynomialApproximation (std::vector<Eigen::Matrix<real, 3, 1> >& samplePoints,
                                        unsigned int polynomial_degree, bool& error) const;
      
      //! Same as above, using a reference for the return value
      inline bool
      bivariatePolynomialApproximation (std::vector<Eigen::Matrix<real, 3, 1> >& samplePoints,
                                        unsigned int polynomial_degree, BivariatePolynomialT<real>& ret) const;

      //! Set the minimum value under which values are considered zero
      inline void
      setZeroValue (real new_zero_value) { parameters_.setZeroValue(new_zero_value); }
      
    protected:  
      // =====PROTECTED METHODS=====
      //! check if fabs(d)<zeroValue
      inline bool
      isNearlyZero (real d) const 
      { 
        return (fabs (d) < parameters_.zero_value);
      }
      
      //! check if sqrt(fabs(d))<zeroValue
      inline bool
      sqrtIsNearlyZero (real d) const 
      { 
        return (fabs (d) < parameters_.sqr_zero_value);
      }
      
      // =====PROTECTED MEMBERS=====
      Parameters parameters_;
  };

  typedef PolynomialCalculationsT<double> PolynomialCalculationsd;
  typedef PolynomialCalculationsT<float>  PolynomialCalculations;

}  // end namespace

#include <pcl/common/impl/polynomial_calculations.hpp>

#endif

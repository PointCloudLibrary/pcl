/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
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
 * $Id$
 *
 */

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

namespace pcl
{
  /** \brief This represents a bivariate polynomial and provides some functionality for it
    * \author Bastian Steder
    * \ingroup common
    */
  template<typename real>
  class BivariatePolynomialT
  {
    public:
      //-----CONSTRUCTOR&DESTRUCTOR-----
      /** Constructor */
      BivariatePolynomialT (int new_degree=0);
      /** Copy constructor */
      BivariatePolynomialT (const BivariatePolynomialT& other);
      /** Destructor */
      ~BivariatePolynomialT ();

      //-----OPERATORS-----
      /** = operator */
      BivariatePolynomialT&
      operator= (const BivariatePolynomialT& other) { deepCopy (other); return *this;}

      //-----METHODS-----
      /** Initialize members to default values */
      void
      setDegree (int new_degree);

      /** How many parameters has a bivariate polynomial with this degree */
      unsigned int
      getNoOfParameters () const { return getNoOfParametersFromDegree (degree);}

      /** Calculate the value of the polynomial at the given point */
      real
      getValue (real x, real y) const;

      /** Calculate the gradient of this polynomial
       *  If forceRecalc is false, it will do nothing when the gradient already exists */
      void
      calculateGradient (bool forceRecalc=false);

      /** Calculate the value of the gradient at the given point */
      void
      getValueOfGradient (real x, real y, real& gradX, real& gradY);

      /** Returns critical points of the polynomial. type can be 0=maximum, 1=minimum, or 2=saddle point
       *  !!Currently only implemented for degree 2!! */
      void
      findCriticalPoints (std::vector<real>& x_values, std::vector<real>& y_values, std::vector<int>& types) const;

      /** write as binary to a stream */
      void
      writeBinary (std::ostream& os) const;

      /** write as binary into a file */
      void
      writeBinary (const char* filename) const;

      /** read binary from a stream */
      void
      readBinary (std::istream& os);

      /** read binary from a file */
      void
      readBinary (const char* filename);

      /** How many parameters has a bivariate polynomial of the given degree */
      static unsigned int
      getNoOfParametersFromDegree (int n) { return ((n+2)* (n+1))/2;}

      //-----VARIABLES-----
      int degree;
      real* parameters;
      BivariatePolynomialT<real>* gradient_x, * gradient_y;

    protected:
      //-----METHODS-----
      /** Delete all members */
      void
      memoryCleanUp ();

      /** Create a deep copy of the given polynomial */
      void
      deepCopy (const BivariatePolynomialT<real>& other);
    //-----VARIABLES-----
  };

  template<typename real>
  std::ostream&
    operator<< (std::ostream& os, const BivariatePolynomialT<real>& p);

  using BivariatePolynomiald = BivariatePolynomialT<double>;
  using BivariatePolynomial = BivariatePolynomialT<float>;

}  // end namespace

#include <pcl/common/impl/bivariate_polynomial.hpp>

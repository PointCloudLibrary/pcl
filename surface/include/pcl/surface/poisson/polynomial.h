/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef PCL_POISSON_POLYNOMIAL_H_
#define PCL_POISSON_POLYNOMIAL_H_

#include <vector>

namespace pcl {
  namespace poisson {

    template<int Degree>
    class Polynomial{
    public:
      double coefficients[Degree+1];

      Polynomial(void);
      template<int Degree2>
      Polynomial(const Polynomial<Degree2>& P);
      double operator() (const double& t) const;
      double integral (const double& tMin,const double& tMax) const;

      int operator == (const Polynomial& p) const;
      int operator != (const Polynomial& p) const;
      int isZero(void) const;
      void setZero(void);

      template<int Degree2>
      Polynomial& operator  = (const Polynomial<Degree2> &p);
      Polynomial& operator += (const Polynomial& p);
      Polynomial& operator -= (const Polynomial& p);
      Polynomial  operator -  (void) const;
      Polynomial  operator +  (const Polynomial& p) const;
      Polynomial  operator -  (const Polynomial& p) const;
      template<int Degree2>
      Polynomial<Degree+Degree2>  operator *  (const Polynomial<Degree2>& p) const;

      Polynomial& operator += (const double& s);
      Polynomial& operator -= (const double& s);
      Polynomial& operator *= (const double& s);
      Polynomial& operator /= (const double& s);
      Polynomial  operator +  (const double& s) const;
      Polynomial  operator -  (const double& s) const;
      Polynomial  operator *  (const double& s) const;
      Polynomial  operator /  (const double& s) const;

      Polynomial scale (const double& s) const;
      Polynomial shift (const double& t) const;

      Polynomial<Degree-1> derivative (void) const;
      Polynomial<Degree+1> integral (void) const;

      void printnl (void) const;

      Polynomial& addScaled (const Polynomial& p, const double& scale);

      static void Negate (const Polynomial& in, Polynomial& out);
      static void Subtract (const Polynomial& p1, const Polynomial& p2, Polynomial& q);
      static void Scale (const Polynomial& p, const double& w, Polynomial& q);
      static void AddScaled (const Polynomial& p1, const double& w1, const Polynomial& p2, const double& w2, Polynomial& q);
      static void AddScaled (const Polynomial& p1, const Polynomial& p2, const double& w2, Polynomial& q);
      static void AddScaled (const Polynomial& p1, const double& w1, const Polynomial& p2, Polynomial& q);

      void getSolutions (const double& c, std::vector<double>& roots, const double& EPS) const;
    };


  }
}


#include <pcl/surface/impl/poisson/polynomial.hpp>
#endif /* PCL_POISSON_POLYNOMIAL_H_ */

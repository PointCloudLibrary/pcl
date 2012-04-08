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

#ifndef PCL_POISSON_PPOLYNOMIAL_H_
#define PCL_POISSON_PPOLYNOMIAL_H_

#include <vector>
#include "polynomial.h"


namespace pcl
{
  namespace poisson
  {
    template <int Degree> 
    class StartingPolynomial
    {
      public:
        Polynomial<Degree> p;
        double start;

        StartingPolynomial () : p (), start () {}

        template <int Degree2> StartingPolynomial<Degree+Degree2> operator* (const StartingPolynomial<Degree2>&p) const;
        StartingPolynomial scale (const double&s) const;
        StartingPolynomial shift (const double&t) const;
        int operator < (const StartingPolynomial &sp) const;
        static int Compare (const void *v1,const void *v2);
    };

    template <int Degree> 
    class PPolynomial
    {
      public:
        size_t polyCount;
        StartingPolynomial<Degree>*polys;

        PPolynomial (void);
        PPolynomial (const PPolynomial<Degree>&p);
        ~PPolynomial (void);

        PPolynomial& operator = (const PPolynomial&p);

        int size (void) const;

        void set (const size_t&size);
        // Note: this method will sort the elements in sps
        void set (StartingPolynomial<Degree>*sps,const int&count);
        void reset (const size_t&newSize);


        double operator() (const double &t) const;
        double integral (const double &tMin,const double &tMax) const;
        double Integral (void) const;

        template <int Degree2> PPolynomial<Degree>& operator = (const PPolynomial<Degree2>&p);

        PPolynomial operator + (const PPolynomial&p) const;
        PPolynomial operator - (const PPolynomial &p) const;

        template <int Degree2> PPolynomial<Degree+Degree2> operator * (const Polynomial<Degree2> &p) const;

        template <int Degree2> PPolynomial<Degree+Degree2> operator* (const PPolynomial<Degree2>&p) const;


        PPolynomial& operator += (const double&s);
        PPolynomial& operator -= (const double&s);
        PPolynomial& operator *= (const double&s);
        PPolynomial& operator /= (const double&s);
        PPolynomial operator +  (const double&s) const;
        PPolynomial operator -  (const double&s) const;
        PPolynomial operator*  (const double&s) const;
        PPolynomial operator /  (const double &s) const;

        PPolynomial& addScaled (const PPolynomial &poly,const double &scale);

        PPolynomial scale (const double &s) const;
        PPolynomial shift (const double &t) const;

        PPolynomial<Degree-1> derivative (void) const;
        PPolynomial<Degree+1> integral (void) const;

        void getSolutions (const double &c,
                           std::vector<double> &roots,
                           const double &EPS,
                           const double &min =- DBL_MAX,
                           const double &max=DBL_MAX) const;

        void printnl (void) const;

        PPolynomial<Degree+1> MovingAverage (const double &radius);

        static PPolynomial ConstantFunction (const double &width=0.5);
        static PPolynomial GaussianApproximation (const double &width=0.5);
        void write (FILE *fp,
                    const int &samples,
                    const double &min,
                    const double &max) const;
    };


  }
}

#include <pcl/surface/impl/poisson/ppolynomial.hpp>

#endif /* PCL_POISSON_PPOLYNOMIAL_H_ */

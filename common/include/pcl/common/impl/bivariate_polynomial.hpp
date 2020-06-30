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

#include <pcl/common/bivariate_polynomial.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>


namespace pcl
{

template<typename real>
BivariatePolynomialT<real>::BivariatePolynomialT (int new_degree) :
  degree(0), parameters(nullptr), gradient_x(nullptr), gradient_y(nullptr)
{
  setDegree(new_degree);
}


template<typename real>
BivariatePolynomialT<real>::BivariatePolynomialT (const BivariatePolynomialT& other) :
  degree(0), parameters(NULL), gradient_x(NULL), gradient_y(NULL)
{
  deepCopy (other);
}


template<typename real>
BivariatePolynomialT<real>::~BivariatePolynomialT ()
{
  memoryCleanUp ();
}


template<typename real> void
BivariatePolynomialT<real>::setDegree (int newDegree)
{
  if (newDegree <= 0)
  {
    degree = -1;
    memoryCleanUp();
    return;
  }
  int oldDegree = degree;
  degree = newDegree;
  if (oldDegree != degree)
  {
    delete[] parameters;
    parameters = new real[getNoOfParameters ()];
  }
  delete gradient_x; gradient_x = nullptr;
  delete gradient_y; gradient_y = nullptr;
}


template<typename real> void
BivariatePolynomialT<real>::memoryCleanUp ()
{
  delete[] parameters; parameters = nullptr;
  delete gradient_x; gradient_x = nullptr;
  delete gradient_y; gradient_y = nullptr;
}


template<typename real> void
BivariatePolynomialT<real>::deepCopy (const pcl::BivariatePolynomialT<real>& other)
{
  if (this == &other) return;
  if (degree != other.degree)
  {
    memoryCleanUp ();
    degree = other.degree;
    parameters = new real[getNoOfParameters ()];
  }
  if (!other.gradient_x)
  {
    delete gradient_x;
    delete gradient_y;
    gradient_x = nullptr;
    gradient_y = nullptr;
  }
  else if (!gradient_x)
  {
    gradient_x = new pcl::BivariatePolynomialT<real> ();
    gradient_y = new pcl::BivariatePolynomialT<real> ();
  }

  std::copy_n(other.parameters, getNoOfParameters (), parameters);

  if (other.gradient_x != nullptr)
  {
    gradient_x->deepCopy (*other.gradient_x);
    gradient_y->deepCopy (*other.gradient_y);
  }
}


template<typename real> void
BivariatePolynomialT<real>::calculateGradient (bool forceRecalc)
{
  if (gradient_x!=NULL && !forceRecalc) return;

  if (gradient_x == NULL)
    gradient_x = new pcl::BivariatePolynomialT<real> (degree-1);
  if (gradient_y == NULL)
    gradient_y = new pcl::BivariatePolynomialT<real> (degree-1);

  unsigned int parameterPosDx=0, parameterPosDy=0;
  for (int xDegree=degree; xDegree>=0; xDegree--)
  {
    for (int yDegree=degree-xDegree; yDegree>=0; yDegree--)
    {
      if (xDegree > 0)
      {
        gradient_x->parameters[parameterPosDx] = xDegree * parameters[parameterPosDx];
        parameterPosDx++;
      }
      if (yDegree > 0)
      {
        gradient_y->parameters[parameterPosDy] = yDegree * parameters[ ( (degree+2-xDegree)* (degree+1-xDegree))/2 -
                                                                        yDegree - 1];
        parameterPosDy++;
      }
    }
  }
}


template<typename real> real
BivariatePolynomialT<real>::getValue (real x, real y) const
{
  unsigned int parametersSize = getNoOfParameters ();
  real* tmpParameter = &parameters[parametersSize-1];
  real tmpX=1.0, tmpY, ret=0;
  for (int xDegree=0; xDegree<=degree; xDegree++)
  {
    tmpY = 1.0;
    for (int yDegree=0; yDegree<=degree-xDegree; yDegree++)
    {
      ret += (*tmpParameter)*tmpX*tmpY;
      tmpY *= y;
      tmpParameter--;
    }
    tmpX *= x;
  }
  return ret;
}


template<typename real> void
BivariatePolynomialT<real>::getValueOfGradient (real x, real y, real& gradX, real& gradY)
{
  calculateGradient ();
  gradX = gradient_x->getValue (x, y);
  gradY = gradient_y->getValue (x, y);
}


template<typename real> void
BivariatePolynomialT<real>::findCriticalPoints (std::vector<real>& x_values, std::vector<real>& y_values,
                                                std::vector<int>& types) const
{
  x_values.clear ();
  y_values.clear ();
  types.clear ();

  if (degree == 2)
  {
    real x = (real(2)*parameters[2]*parameters[3] - parameters[1]*parameters[4]) /
             (parameters[1]*parameters[1] - real(4)*parameters[0]*parameters[3]),
         y = (real(-2)*parameters[0]*x - parameters[2]) / parameters[1];

    if (!std::isfinite(x) || !std::isfinite(y))
      return;

    int type = 2;
    real det_H = real(4)*parameters[0]*parameters[3] - parameters[1]*parameters[1];
    //std::cout << "det(H) = "<<det_H<<"\n";
    if (det_H > real(0))  // Check Hessian determinant
    {
      if (parameters[0]+parameters[3] < real(0))  // Check Hessian trace
        type = 0;
      else
        type = 1;
    }
    x_values.push_back(x);
    y_values.push_back(y);
    types.push_back(type);
  }
  else
  {
    std::cerr << __PRETTY_FUNCTION__ << " is not implemented for polynomials of degree "<<degree<<". Sorry.\n";
  }
}


template<typename real> std::ostream&
operator<< (std::ostream& os, const pcl::BivariatePolynomialT<real>& p)
{
  real* tmpParameter = p.parameters;
  bool first = true;
  real currentParameter;
  for (int xDegree=p.degree; xDegree>=0; xDegree--)
  {
    for (int yDegree=p.degree-xDegree; yDegree>=0; yDegree--)
    {
      currentParameter = *tmpParameter;
      if (!first)
      {
        os << (currentParameter<0.0?" - ":" + ");
        currentParameter = std::abs (currentParameter);
      }
      os << currentParameter;
      if (xDegree>0)
      {
        os << "x";
        if (xDegree>1)
          os<<"^"<<xDegree;
      }
      if (yDegree>0)
      {
        os << "y";
        if (yDegree>1)
          os<<"^"<<yDegree;
      }

      first = false;
      tmpParameter++;
    }
  }
  return (os);
}


template<typename real> void
BivariatePolynomialT<real>::writeBinary (std::ostream& os) const
{
  os.write (reinterpret_cast<const char*> (&degree), sizeof (int));
  unsigned int paramCnt = getNoOfParametersFromDegree (this->degree);
  os.write (reinterpret_cast<const char*> (this->parameters), paramCnt * sizeof (real));
}


template<typename real> void
BivariatePolynomialT<real>::writeBinary (const char* filename) const
{
  std::ofstream fout (filename);
  writeBinary (fout);
}


template<typename real> void
BivariatePolynomialT<real>::readBinary (std::istream& os)
{
  memoryCleanUp ();
  os.read (reinterpret_cast<char*> (&this->degree), sizeof (int));
  unsigned int paramCnt = getNoOfParametersFromDegree (this->degree);
  parameters = new real[paramCnt];
  os.read (reinterpret_cast<char*> (&(*this->parameters)), paramCnt * sizeof (real));
}


template<typename real> void
BivariatePolynomialT<real>::readBinary (const char* filename)
{
  std::ifstream fin (filename);
  readBinary (fin);
}

} // namespace pcl

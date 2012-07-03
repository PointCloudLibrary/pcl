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
 * $Id: vector.h 5531 2012-04-08 09:14:33Z aichim $
 *
 */

#ifndef PCL_POISSON__VECTOR_H_
#define PCL_POISSON__VECTOR_H_

#define Assert assert
#include <assert.h>

namespace pcl {
  namespace poisson {

    template<class T>
    class Vector
    {
    public:
      Vector ();
      Vector (const Vector<T>& V);
      Vector (size_t N);
      Vector (size_t N, T* pV);
      ~Vector();

      const T& operator () (size_t i) const;
      T& operator () (size_t i);
      const T& operator [] (size_t i) const;
      T& operator [] (size_t i);

      void SetZero();

      size_t Dimensions() const;
      void Resize( size_t N );

      Vector operator * (const T& A) const;
      Vector operator / (const T& A) const;
      Vector operator - (const Vector& V) const;
      Vector operator + (const Vector& V) const;

      Vector& operator *= (const T& A);
      Vector& operator /= (const T& A);
      Vector& operator += (const Vector& V);
      Vector& operator -= (const Vector& V);

      Vector& AddScaled (const Vector& V,const T& scale);
      Vector& SubtractScaled (const Vector& V,const T& scale);
      static void Add (const Vector& V1,const T& scale1,const Vector& V2,const T& scale2,Vector& Out);
      static void Add (const Vector& V1,const T& scale1,const Vector& V2,Vector& Out);

      Vector operator - () const;

      Vector& operator = (const Vector& V);

      T Dot (const Vector& V) const;

      T Length() const;

      T Norm (size_t Ln) const;
      void Normalize();

      T* m_pV;
    protected:
      size_t m_N;

    };

    template<class T,int Dim>
    class NVector
    {
    public:
      NVector ();
      NVector (const NVector& V);
      NVector (size_t N);
      NVector (size_t N, T* pV);
      ~NVector ();

      const T* operator () (size_t i) const;
      T* operator () (size_t i);
      const T* operator [] (size_t i) const;
      T* operator [] (size_t i);

      void SetZero();

      size_t Dimensions() const;
      void Resize( size_t N );

      NVector operator * (const T& A) const;
      NVector operator / (const T& A) const;
      NVector operator - (const NVector& V) const;
      NVector operator + (const NVector& V) const;

      NVector& operator *= (const T& A);
      NVector& operator /= (const T& A);
      NVector& operator += (const NVector& V);
      NVector& operator -= (const NVector& V);

      NVector& AddScaled (const NVector& V,const T& scale);
      NVector& SubtractScaled (const NVector& V,const T& scale);
      static void Add (const NVector& V1,const T& scale1,const NVector& V2,const T& scale2,NVector& Out);
      static void Add (const NVector& V1,const T& scale1,const NVector& V2, NVector& Out);

      NVector operator - () const;

      NVector& operator = (const NVector& V);

      T Dot (const NVector& V) const;

      T Length () const;

      T Norm (size_t Ln) const;
      void Normalize ();

      T* m_pV;
    protected:
      size_t m_N;

    };

  }
}


#include <pcl/surface/impl/poisson/vector.hpp>



#endif /* PCL_POISSON__VECTOR_H_ */

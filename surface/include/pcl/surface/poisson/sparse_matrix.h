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

#ifndef PCL_POISSON_SPARSE_MATRIX_H_
#define PCL_POISSON_SPARSE_MATRIX_H_

#include "vector.h"
#include "allocator.h"

namespace pcl 
{
  namespace poisson 
  {
    template <class T>
    struct MatrixEntry
    {
      MatrixEntry () : N (-1), Value (0) {}
      MatrixEntry (int i) : N (i), Value (0) {}
      int N;
      T Value;
    };

    template <class T,int Dim>
    struct NMatrixEntry
    {
      NMatrixEntry () : N (-1), Value () { memset (Value, 0, sizeof (T) * Dim); }
      NMatrixEntry (int i) : N (i), Value () { memset (Value, 0, sizeof (T) * Dim); }
      int N;
      T Value[Dim];
    };

    template<class T> class SparseMatrix
    {
      private:
        static int UseAlloc;
      public:
        static Allocator<MatrixEntry<T> > AllocatorMatrixEntry;
        static int UseAllocator (void);
        static void SetAllocator (const int& blockSize);

        int rows;
        int* rowSizes;
        MatrixEntry<T>** m_ppElements;

        SparseMatrix ();
        SparseMatrix (int rows);
        void Resize (int rows);
        void SetRowSize (int row , int count);
        int Entries (void);

        SparseMatrix (const SparseMatrix& M);
        virtual ~SparseMatrix ();

        void SetZero ();
        void SetIdentity ();

        SparseMatrix<T>& operator = (const SparseMatrix<T>& M);

        SparseMatrix<T> operator * (const T& V) const;
        SparseMatrix<T>& operator *= (const T& V);


        SparseMatrix<T> operator * (const SparseMatrix<T>& M) const;
        SparseMatrix<T> Multiply (const SparseMatrix<T>& M) const;
        SparseMatrix<T> MultiplyTranspose (const SparseMatrix<T>& Mt) const;

        template<class T2>
        Vector<T2> operator * (const Vector<T2>& V) const;
        template<class T2>
        Vector<T2> Multiply (const Vector<T2>& V) const;
        template<class T2>
        void Multiply (const Vector<T2>& In, Vector<T2>& Out) const;


        SparseMatrix<T> Transpose() const;

        static int Solve (const SparseMatrix<T>& M,
                          const Vector<T>& b,
                          const int& iters,
                          Vector<T>& solution,
                          const T eps = 1e-8);

        template<class T2>
        static int SolveSymmetric (const SparseMatrix<T>& M,
                                   const Vector<T2>& b,
                                   const int& iters,
                                   Vector<T2>& solution,
                                   const T2 eps = 1e-8,
                                   const int& reset=1);

    };

    template<class T,int Dim> class SparseNMatrix
    {
      private:
        static int UseAlloc;
      public:
        static Allocator<NMatrixEntry<T,Dim> > AllocatorNMatrixEntry;
        static int UseAllocator (void);
        static void SetAllocator (const int& blockSize);

        int rows;
        int* rowSizes;
        NMatrixEntry<T,Dim>** m_ppElements;

        SparseNMatrix ();
        SparseNMatrix (int rows);
        void Resize (int rows);
        void SetRowSize (int row, int count);
        int Entries ();

        SparseNMatrix (const SparseNMatrix& M);
        ~SparseNMatrix ();

        SparseNMatrix& operator = (const SparseNMatrix& M);

        SparseNMatrix  operator *  (const T& V) const;
        SparseNMatrix& operator *= (const T& V);

        template<class T2>
        NVector<T2,Dim> operator * (const Vector<T2>& V) const;
        template<class T2>
        Vector<T2> operator * (const NVector<T2,Dim>& V) const;
    };

    template <class T>
    class SparseSymmetricMatrix : public SparseMatrix<T>
    {
      public:
        virtual ~SparseSymmetricMatrix () {}

        template<class T2>
        Vector<T2> operator * (const Vector<T2>& V) const;

        template<class T2>
        Vector<T2> Multiply (const Vector<T2>& V ) const;

        template<class T2> void 
        Multiply (const Vector<T2>& In, Vector<T2>& Out ) const;

        template<class T2> static int 
        Solve (const SparseSymmetricMatrix<T>& M,
               const Vector<T2>& b,
               const int& iters,
               Vector<T2>& solution,
               const T2 eps = 1e-8,
               const int& reset=1);

        template<class T2> static int 
        Solve (const SparseSymmetricMatrix<T>& M,
               const Vector<T>& diagonal,
               const Vector<T2>& b,
               const int& iters,
               Vector<T2>& solution,
               const T2 eps = 1e-8,
               const int& reset=1);
    };
  }
}

#include <pcl/surface/impl/poisson/sparse_matrix.hpp>

#endif /* PCL_POISSON_SPARSE_MATRIX_H_ */


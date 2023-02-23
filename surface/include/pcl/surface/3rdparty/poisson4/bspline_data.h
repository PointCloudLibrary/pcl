/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef BSPLINE_DATA_INCLUDED
#define BSPLINE_DATA_INCLUDED


#include "ppolynomial.h"

#include <cstdio>
#include <cstring>

namespace pcl
{
  namespace poisson
  {


    template< int Degree , class Real >
    class BSplineData
    {
        bool useDotRatios;
        bool reflectBoundary;
      public:
        struct BSplineComponents
        {
            Polynomial< Degree > polys[Degree+1];
            Polynomial< Degree >& operator[] ( int idx ) { return polys[idx]; }
            const Polynomial< Degree >& operator[] ( int idx ) const { return polys[idx]; }
            void printnl( ) const  { for( int d=0 ; d<=Degree ; d++ ) polys[d].printnl(); }
            BSplineComponents scale( double s ) const { BSplineComponents b ; for( int d=0 ; d<=Degree ; d++ ) b[d] = polys[d].scale(s) ; return b; }
            BSplineComponents shift( double s ) const { BSplineComponents b ; for( int d=0 ; d<=Degree ; d++ ) b[d] = polys[d].shift(s) ; return b; }
        };
        const static int  VV_DOT_FLAG = 1;
        const static int  DV_DOT_FLAG = 2;
        const static int  DD_DOT_FLAG = 4;
        const static int   VALUE_FLAG = 1;
        const static int D_VALUE_FLAG = 2;

        int depth , functionCount , sampleCount;
        Real *vvDotTable , *dvDotTable , *ddDotTable;
        Real *valueTables , *dValueTables;
        PPolynomial< Degree   >  baseFunction ,  leftBaseFunction ,  rightBaseFunction;
        PPolynomial< Degree-1 > dBaseFunction , dLeftBaseFunction , dRightBaseFunction;
        BSplineComponents baseBSpline, leftBSpline , rightBSpline;
        PPolynomial<Degree>* baseFunctions;
        BSplineComponents* baseBSplines;

        BSplineData();
        virtual ~BSplineData();

        virtual void   setDotTables( int flags );
        virtual void clearDotTables( int flags );

        virtual void   setValueTables( int flags,double smooth=0);
        virtual void   setValueTables( int flags,double valueSmooth,double normalSmooth);
        virtual void clearValueTables();

        void setSampleSpan( int idx , int& start , int& end , double smooth=0 ) const;

        /********************************************************
   * Sets the translates and scales of the basis function
   * up to the prescribed depth
   * <maxDepth> the maximum depth
   * <useDotRatios> specifies if dot-products of derivatives
   * should be pre-divided by function integrals
   * <reflectBoundary> spcifies if function space should be
   * forced to be reflectively symmetric across the boundary
   ********************************************************/
        void set( int maxDepth , bool useDotRatios=true , bool reflectBoundary=false );

        inline int Index( int i1 , int i2 ) const;
        static inline int SymmetricIndex( int i1 , int i2 );
        static inline int SymmetricIndex( int i1 , int i2 , int& index  );
    };

    template< int Degree >
    struct BSplineElementCoefficients
    {
        int coeffs[Degree+1] = {};
        BSplineElementCoefficients( ) = default;
        int& operator[]( int idx ){ return coeffs[idx]; }
        const int& operator[]( int idx ) const { return coeffs[idx]; }
    };
    template< int Degree >
    struct BSplineElements : public std::vector< BSplineElementCoefficients< Degree > >
    {
        static const int _off = (Degree+1)/2;
        void _addLeft ( int offset , int boundary );
        void _addRight( int offset , int boundary );
      public:
        enum
        {
          NONE      =  0,
          DIRICHLET = -1,
          NEUMANN   =  1
        };
        // Coefficients are ordered as "/" "-" "\"
        int denominator;

        BSplineElements( ) { denominator = 1; }
        BSplineElements( int res , int offset , int boundary=NONE );

        void upSample( BSplineElements& high ) const;
        void differentiate( BSplineElements< Degree-1 >& d ) const;

        void print( FILE* ) const
        {
          for( int i=0 ; i<this->size() ; i++ )
          {
            printf( "%d]" , i );
            for( int j=0 ; j<=Degree ; j++ ) printf( " %d" , (*this)[i][j] );
            printf( " (%d)\n" , denominator );
          }
        }
    };
    template< int Degree1 , int Degree2 > void SetBSplineElementIntegrals( double integrals[Degree1+1][Degree2+1] );


  }
}


#include "bspline_data.hpp"

#endif // BSPLINE_DATA_INCLUDED

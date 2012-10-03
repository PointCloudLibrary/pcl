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


namespace pcl
{
  namespace poisson
  {

    /////////////////
    // BSplineData //
    /////////////////
    // Support[i]:
    //		Odd:  i +/- 0.5 * ( 1 + Degree )
    //			i - 0.5 * ( 1 + Degree ) < 0
    // <=>		i < 0.5 * ( 1 + Degree )
    //			i + 0.5 * ( 1 + Degree ) > 0
    // <=>		i > - 0.5 * ( 1 + Degree )
    //			i + 0.5 * ( 1 + Degree ) > r
    // <=>      i > r - 0.5 * ( 1 + Degree )
    //			i - 0.5 * ( 1 + Degree ) < r
    // <=>      i < r + 0.5 * ( 1 + Degree )
    //		Even: i + 0.5 +/- 0.5 * ( 1 + Degree )
    //			i - 0.5 * Degree < 0
    // <=>		i < 0.5 * Degree
    //			i + 1 + 0.5 * Degree > 0
    // <=>		i > -1 - 0.5 * Degree
    //			i + 1 + 0.5 * Degree > r
    // <=>		i > r - 1 - 0.5 * Degree
    //			i - 0.5 * Degree < r
    // <=>		i < r + 0.5 * Degree
    template< int Degree > inline bool LeftOverlap( unsigned int depth , int offset )
    {
      offset <<= 1;
      if( Degree & 1 ) return (offset < 1+Degree) && (offset > -1-Degree );
      else             return (offset <   Degree) && (offset > -2-Degree );
    }
    template< int Degree > inline bool RightOverlap( unsigned int depth , int offset )
    {
      offset <<= 1;
      int r = 1<<(depth+1);
      if( Degree & 1 ) return (offset > 2-1-Degree) && (offset < 2+1+Degree );
      else             return (offset > 2-2-Degree) && (offset < 2+  Degree );
    }
    template< int Degree > inline int ReflectLeft( unsigned int depth , int offset )
    {
      if( Degree&1 ) return   -offset;
      else           return -1-offset;
    }
    template< int Degree > inline int ReflectRight( unsigned int depth , int offset )
    {
      int r = 1<<(depth+1);
      if( Degree&1 ) return r  -offset;
      else           return r-1-offset;
    }

    template< int Degree , class Real >
    BSplineData<Degree,Real>::BSplineData( void )
    {
      vvDotTable = dvDotTable = ddDotTable = NULL;
      valueTables = dValueTables = NULL;
      functionCount = sampleCount = 0;
    }

    template< int Degree , class Real >
    BSplineData< Degree , Real >::~BSplineData(void)
    {
      if( functionCount )
      {
        if( vvDotTable ) delete[] vvDotTable;
        if( dvDotTable ) delete[] dvDotTable;
        if( ddDotTable ) delete[] ddDotTable;

        if(  valueTables ) delete[]  valueTables;
        if( dValueTables ) delete[] dValueTables;
      }
      vvDotTable = dvDotTable = ddDotTable = NULL;
      valueTables = dValueTables=NULL;
      functionCount = 0;
    }

    template<int Degree,class Real>
    void BSplineData<Degree,Real>::set( int maxDepth , bool useDotRatios , bool reflectBoundary )
    {
      this->useDotRatios    = useDotRatios;
      this->reflectBoundary = reflectBoundary;

      depth = maxDepth;
      // [Warning] This assumes that the functions spacing is dual
      functionCount = BinaryNode< double >::CumulativeCenterCount( depth );
      sampleCount   = BinaryNode< double >::CenterCount( depth ) + BinaryNode< double >::CornerCount( depth );
      baseFunctions = new PPolynomial<Degree>[functionCount];
      baseBSplines = new BSplineComponents[functionCount];

      baseFunction = PPolynomial< Degree >::BSpline();
      for( int i=0 ; i<=Degree ; i++ ) baseBSpline[i] = Polynomial< Degree >::BSplineComponent( i ).shift( double(-(Degree+1)/2) + i - 0.5 );
      dBaseFunction = baseFunction.derivative();
      StartingPolynomial< Degree > sPolys[Degree+3];

      for( int i=0 ; i<Degree+3 ; i++ )
      {
        sPolys[i].start = double(-(Degree+1)/2) + i - 1.5;
        sPolys[i].p *= 0;
        if(         i<=Degree   )  sPolys[i].p += baseBSpline[i  ].shift( -1 );
        if( i>=1 && i<=Degree+1 )  sPolys[i].p += baseBSpline[i-1];
        for( int j=0 ; j<i ; j++ ) sPolys[i].p -= sPolys[j].p;
      }
      leftBaseFunction.set( sPolys , Degree+3 );
      for( int i=0 ; i<Degree+3 ; i++ )
      {
        sPolys[i].start = double(-(Degree+1)/2) + i - 0.5;
        sPolys[i].p *= 0;
        if(         i<=Degree   )  sPolys[i].p += baseBSpline[i  ];
        if( i>=1 && i<=Degree+1 )  sPolys[i].p += baseBSpline[i-1].shift( 1 );
        for( int j=0 ; j<i ; j++ ) sPolys[i].p -= sPolys[j].p;
      }
      rightBaseFunction.set( sPolys , Degree+3 );
      dLeftBaseFunction  =  leftBaseFunction.derivative();
      dRightBaseFunction = rightBaseFunction.derivative();
      leftBSpline = rightBSpline = baseBSpline;
      leftBSpline [1] +=  leftBSpline[2].shift( -1 ) ,  leftBSpline[0] *= 0;
      rightBSpline[1] += rightBSpline[0].shift(  1 ) , rightBSpline[2] *= 0;
      double c , w;
      for( int i=0 ; i<functionCount ; i++ )
      {
        BinaryNode< double >::CenterAndWidth( i , c , w );
        baseFunctions[i] = baseFunction.scale(w).shift(c);
        baseBSplines[i] = baseBSpline.scale(w).shift(c);
        if( reflectBoundary )
        {
          int d , off , r;
          BinaryNode< double >::DepthAndOffset( i , d , off );
          r = 1<<d;
          if     ( off==0   ) baseFunctions[i] =  leftBaseFunction.scale(w).shift(c);
          else if( off==r-1 ) baseFunctions[i] = rightBaseFunction.scale(w).shift(c);
          if     ( off==0   ) baseBSplines[i] =  leftBSpline.scale(w).shift(c);
          else if( off==r-1 ) baseBSplines[i] = rightBSpline.scale(w).shift(c);
        }
      }
    }
    template<int Degree,class Real>
    void BSplineData<Degree,Real>::setDotTables( int flags )
    {
      clearDotTables( flags );
      int size = ( functionCount*functionCount + functionCount )>>1;
      int fullSize = functionCount*functionCount;
      if( flags & VV_DOT_FLAG )
      {
        vvDotTable = new Real[size];
        memset( vvDotTable , 0 , sizeof(Real)*size );
      }
      if( flags & DV_DOT_FLAG )
      {
        dvDotTable = new Real[fullSize];
        memset( dvDotTable , 0 , sizeof(Real)*fullSize );
      }
      if( flags & DD_DOT_FLAG )
      {
        ddDotTable = new Real[size];
        memset( ddDotTable , 0 , sizeof(Real)*size );
      }
      double vvIntegrals[Degree+1][Degree+1];
      double vdIntegrals[Degree+1][Degree  ];
      double dvIntegrals[Degree  ][Degree+1];
      double ddIntegrals[Degree  ][Degree  ];
      int vvSums[Degree+1][Degree+1];
      int vdSums[Degree+1][Degree  ];
      int dvSums[Degree  ][Degree+1];
      int ddSums[Degree  ][Degree  ];
      SetBSplineElementIntegrals< Degree   , Degree   >( vvIntegrals );
      SetBSplineElementIntegrals< Degree   , Degree-1 >( vdIntegrals );
      SetBSplineElementIntegrals< Degree-1 , Degree   >( dvIntegrals );
      SetBSplineElementIntegrals< Degree-1 , Degree-1 >( ddIntegrals );

      for( int d1=0 ; d1<=depth ; d1++ )
        for( int off1=0 ; off1<(1<<d1) ; off1++ )
        {
          int ii = BinaryNode< Real >::CenterIndex( d1 , off1 );
          BSplineElements< Degree > b1( 1<<d1 , off1 , reflectBoundary ? BSplineElements<Degree>::NEUMANN   : BSplineElements< Degree>::NONE );
          BSplineElements< Degree-1 > db1;
          b1.differentiate( db1 );

          int start1 , end1;

          start1 = -1;
          for( int i=0 ; i<int(b1.size()) ; i++ ) for( int j=0 ; j<=Degree ; j++ )
          {
            if( b1[i][j] && start1==-1 ) start1 = i;
            if( b1[i][j] ) end1 = i+1;
          }
          for( int d2=d1 ; d2<=depth ; d2++ )
          {
            for( int off2=0 ; off2<(1<<d2) ; off2++ )
            {
              int start2 = off2-Degree;
              int end2   = off2+Degree+1;
              if( start2>=end1 || start1>=end2 ) continue;
              start2 = std::max< int >( start1 , start2 );
              end2   = std::min< int >(   end1 ,   end2 );
              if( d1==d2 && off2<off1 ) continue;
              int jj = BinaryNode< Real >::CenterIndex( d2 , off2 );
              BSplineElements< Degree > b2( 1<<d2 , off2 , reflectBoundary ? BSplineElements<Degree>::NEUMANN   : BSplineElements< Degree>::NONE );
              BSplineElements< Degree-1 > db2;
              b2.differentiate( db2 );

              int idx = SymmetricIndex( ii , jj );
              int idx1 = Index( ii , jj ) , idx2 = Index( jj , ii );

              memset( vvSums , 0 , sizeof( int ) * ( Degree+1 ) * ( Degree+1 ) );
              memset( vdSums , 0 , sizeof( int ) * ( Degree+1 ) * ( Degree   ) );
              memset( dvSums , 0 , sizeof( int ) * ( Degree   ) * ( Degree+1 ) );
              memset( ddSums , 0 , sizeof( int ) * ( Degree   ) * ( Degree   ) );
              for( int i=start2 ; i<end2 ; i++ )
              {
                for( int j=0 ; j<=Degree ; j++ ) for( int k=0 ; k<=Degree ; k++ ) vvSums[j][k] +=  b1[i][j] *  b2[i][k];
                for( int j=0 ; j<=Degree ; j++ ) for( int k=0 ; k< Degree ; k++ ) vdSums[j][k] +=  b1[i][j] * db2[i][k];
                for( int j=0 ; j< Degree ; j++ ) for( int k=0 ; k<=Degree ; k++ ) dvSums[j][k] += db1[i][j] *  b2[i][k];
                for( int j=0 ; j< Degree ; j++ ) for( int k=0 ; k< Degree ; k++ ) ddSums[j][k] += db1[i][j] * db2[i][k];
              }
              double vvDot = 0 , dvDot = 0 , vdDot = 0 , ddDot = 0;
              for( int j=0 ; j<=Degree ; j++ ) for( int k=0 ; k<=Degree ; k++ ) vvDot += vvIntegrals[j][k] * vvSums[j][k];
              for( int j=0 ; j<=Degree ; j++ ) for( int k=0 ; k< Degree ; k++ ) vdDot += vdIntegrals[j][k] * vdSums[j][k];
              for( int j=0 ; j< Degree ; j++ ) for( int k=0 ; k<=Degree ; k++ ) dvDot += dvIntegrals[j][k] * dvSums[j][k];
              for( int j=0 ; j< Degree ; j++ ) for( int k=0 ; k< Degree ; k++ ) ddDot += ddIntegrals[j][k] * ddSums[j][k];
              vvDot /= (1<<d2);
              ddDot *= (1<<d2);
              vvDot /= ( b1.denominator * b2.denominator );
              dvDot /= ( b1.denominator * b2.denominator );
              vdDot /= ( b1.denominator * b2.denominator );
              ddDot /= ( b1.denominator * b2.denominator );
              if( fabs(vvDot)<1e-15 ) continue;
              if( flags & VV_DOT_FLAG ) vvDotTable [idx] = Real( vvDot );
              if( useDotRatios )
              {
                if( flags & DV_DOT_FLAG ) dvDotTable[idx1] = Real( dvDot / vvDot );
                if( flags & DV_DOT_FLAG ) dvDotTable[idx2] = Real( vdDot / vvDot );
                if( flags & DD_DOT_FLAG ) ddDotTable[idx ] = Real( ddDot / vvDot );
              }
              else
              {
                if( flags & DV_DOT_FLAG ) dvDotTable[idx1] = Real( dvDot );
                if( flags & DV_DOT_FLAG ) dvDotTable[idx2] = Real( dvDot );
                if( flags & DD_DOT_FLAG ) ddDotTable[idx ] = Real( ddDot );
              }
            }
            BSplineElements< Degree > b;
            b = b1;
            b.upSample( b1 );
            b1.differentiate( db1 );
            start1 = -1;
            for( int i=0 ; i<int(b1.size()) ; i++ ) for( int j=0 ; j<=Degree ; j++ )
            {
              if( b1[i][j] && start1==-1 ) start1 = i;
              if( b1[i][j] ) end1 = i+1;
            }
          }
        }
    }
    template<int Degree,class Real>
    void BSplineData<Degree,Real>::clearDotTables( int flags )
    {
      if( (flags & VV_DOT_FLAG) && vvDotTable ) delete[] vvDotTable , vvDotTable = NULL;
      if( (flags & DV_DOT_FLAG) && dvDotTable ) delete[] dvDotTable , dvDotTable = NULL;
      if( (flags & DD_DOT_FLAG) && ddDotTable ) delete[] ddDotTable , ddDotTable = NULL;
    }
    template< int Degree , class Real >
    void BSplineData< Degree , Real >::setSampleSpan( int idx , int& start , int& end , double smooth ) const
    {
      int d , off , res;
      BinaryNode< double >::DepthAndOffset( idx , d , off );
      res = 1<<d;
      double _start = ( off + 0.5 - 0.5*(Degree+1) ) / res - smooth;
      double _end   = ( off + 0.5 + 0.5*(Degree+1) ) / res + smooth;
      //   (start)/(sampleCount-1) >_start && (start-1)/(sampleCount-1)<=_start
      // => start > _start * (sampleCount-1 ) && start <= _start*(sampleCount-1) + 1
      // => _start * (sampleCount-1) + 1 >= start > _start * (sampleCount-1)
      start = int( floor( _start * (sampleCount-1) + 1 ) );
      if( start<0 ) start = 0;
      //   (end)/(sampleCount-1)<_end && (end+1)/(sampleCount-1)>=_end
      // => end < _end * (sampleCount-1 ) && end >= _end*(sampleCount-1) - 1
      // => _end * (sampleCount-1) > end >= _end * (sampleCount-1) - 1
      end = int( ceil( _end * (sampleCount-1) - 1 ) );
      if( end>=sampleCount ) end = sampleCount-1;
    }
    template<int Degree,class Real>
    void BSplineData<Degree,Real>::setValueTables( int flags , double smooth )
    {
      clearValueTables();
      if( flags &   VALUE_FLAG )  valueTables = new Real[functionCount*sampleCount];
      if( flags & D_VALUE_FLAG ) dValueTables = new Real[functionCount*sampleCount];
      PPolynomial<Degree+1> function;
      PPolynomial<Degree>  dFunction;
      for( int i=0 ; i<functionCount ; i++ )
      {
        if(smooth>0)
        {
          function  = baseFunctions[i].MovingAverage(smooth);
          dFunction = baseFunctions[i].derivative().MovingAverage(smooth);
        }
        else
        {
          function  = baseFunctions[i];
          dFunction = baseFunctions[i].derivative();
        }
        for( int j=0 ; j<sampleCount ; j++ )
        {
          double x=double(j)/(sampleCount-1);
          if(flags &   VALUE_FLAG){ valueTables[j*functionCount+i]=Real( function(x));}
          if(flags & D_VALUE_FLAG){dValueTables[j*functionCount+i]=Real(dFunction(x));}
        }
      }
    }
    template<int Degree,class Real>
    void BSplineData<Degree,Real>::setValueTables(int flags,double valueSmooth,double normalSmooth){
      clearValueTables();
      if(flags &   VALUE_FLAG){ valueTables=new Real[functionCount*sampleCount];}
      if(flags & D_VALUE_FLAG){dValueTables=new Real[functionCount*sampleCount];}
      PPolynomial<Degree+1> function;
      PPolynomial<Degree>  dFunction;
      for(int i=0;i<functionCount;i++){
        if(valueSmooth>0)	{ function=baseFunctions[i].MovingAverage(valueSmooth);}
        else				{ function=baseFunctions[i];}
        if(normalSmooth>0)	{dFunction=baseFunctions[i].derivative().MovingAverage(normalSmooth);}
        else				{dFunction=baseFunctions[i].derivative();}

        for(int j=0;j<sampleCount;j++){
          double x=double(j)/(sampleCount-1);
          if(flags &   VALUE_FLAG){ valueTables[j*functionCount+i]=Real( function(x));}
          if(flags & D_VALUE_FLAG){dValueTables[j*functionCount+i]=Real(dFunction(x));}
        }
      }
    }


    template<int Degree,class Real>
    void BSplineData<Degree,Real>::clearValueTables(void){
      if( valueTables){delete[]  valueTables;}
      if(dValueTables){delete[] dValueTables;}
      valueTables=dValueTables=NULL;
    }

    template<int Degree,class Real>
    inline int BSplineData<Degree,Real>::Index( int i1 , int i2 ) const { return i1*functionCount+i2; }
    template<int Degree,class Real>
    inline int BSplineData<Degree,Real>::SymmetricIndex( int i1 , int i2 )
    {
      if( i1>i2 ) return ((i1*i1+i1)>>1)+i2;
      else        return ((i2*i2+i2)>>1)+i1;
    }
    template<int Degree,class Real>
    inline int BSplineData<Degree,Real>::SymmetricIndex( int i1 , int i2 , int& index )
    {
      if( i1<i2 )
      {
        index = ((i2*i2+i2)>>1)+i1;
        return 1;
      }
      else
      {
        index = ((i1*i1+i1)>>1)+i2;
        return 0;
      }
    }


    ////////////////////////
    // BSplineElementData //
    ////////////////////////
    template< int Degree >
    BSplineElements< Degree >::BSplineElements( int res , int offset , int boundary )
    {
      denominator = 1;
      this->resize( res , BSplineElementCoefficients<Degree>() );

      for( int i=0 ; i<=Degree ; i++ )
      {
        int idx = -_off + offset + i;
        if( idx>=0 && idx<res ) (*this)[idx][i] = 1;
      }
      if( boundary!=0 )
      {
        _addLeft( offset-2*res , boundary ) , _addRight( offset+2*res , boundary );
        if( Degree&1 ) _addLeft( offset-res , boundary ) , _addRight(  offset+res     , boundary );
        else           _addLeft( -offset-1  , boundary ) , _addRight( -offset-1+2*res , boundary );
      }
    }
    template< int Degree >
    void BSplineElements< Degree >::_addLeft( int offset , int boundary )
    {
      int res = int( this->size() );
      bool set = false;
      for( int i=0 ; i<=Degree ; i++ )
      {
        int idx = -_off + offset + i;
        if( idx>=0 && idx<res ) (*this)[idx][i] += boundary , set = true;
      }
      if( set ) _addLeft( offset-2*res , boundary );
    }
    template< int Degree >
    void BSplineElements< Degree >::_addRight( int offset , int boundary )
    {
      int res = int( this->size() );
      bool set = false;
      for( int i=0 ; i<=Degree ; i++ )
      {
        int idx = -_off + offset + i;
        if( idx>=0 && idx<res ) (*this)[idx][i] += boundary , set = true;
      }
      if( set ) _addRight( offset+2*res , boundary );
    }
    template< int Degree >
    void BSplineElements< Degree >::upSample( BSplineElements< Degree >& high ) const
    {
      fprintf( stderr , "[ERROR] B-spline up-sampling not supported for degree %d\n" , Degree );
      exit( 0 );
    }
    template<>
    void BSplineElements< 1 >::upSample( BSplineElements< 1 >& high ) const
    {
      high.resize( size()*2 );
      high.assign( high.size() , BSplineElementCoefficients<1>() );
      for( int i=0 ; i<int(size()) ; i++ )
      {
        high[2*i+0][0] += 1 * (*this)[i][0];
        high[2*i+0][1] += 0 * (*this)[i][0];
        high[2*i+1][0] += 2 * (*this)[i][0];
        high[2*i+1][1] += 1 * (*this)[i][0];

        high[2*i+0][0] += 1 * (*this)[i][1];
        high[2*i+0][1] += 2 * (*this)[i][1];
        high[2*i+1][0] += 0 * (*this)[i][1];
        high[2*i+1][1] += 1 * (*this)[i][1];
      }
      high.denominator = denominator * 2;
    }
    template<>
    void BSplineElements< 2 >::upSample( BSplineElements< 2 >& high ) const
    {
      //    /----\
      //   /      \
      //  /        \  = 1  /--\       +3    /--\     +3      /--\   +1        /--\
      // /          \     /    \           /    \           /    \           /    \
      // |----------|     |----------|   |----------|   |----------|   |----------|

      high.resize( size()*2 );
      high.assign( high.size() , BSplineElementCoefficients<2>() );
      for( int i=0 ; i<int(size()) ; i++ )
      {
        high[2*i+0][0] += 1 * (*this)[i][0];
        high[2*i+0][1] += 0 * (*this)[i][0];
        high[2*i+0][2] += 0 * (*this)[i][0];
        high[2*i+1][0] += 3 * (*this)[i][0];
        high[2*i+1][1] += 1 * (*this)[i][0];
        high[2*i+1][2] += 0 * (*this)[i][0];

        high[2*i+0][0] += 3 * (*this)[i][1];
        high[2*i+0][1] += 3 * (*this)[i][1];
        high[2*i+0][2] += 1 * (*this)[i][1];
        high[2*i+1][0] += 1 * (*this)[i][1];
        high[2*i+1][1] += 3 * (*this)[i][1];
        high[2*i+1][2] += 3 * (*this)[i][1];

        high[2*i+0][0] += 0 * (*this)[i][2];
        high[2*i+0][1] += 1 * (*this)[i][2];
        high[2*i+0][2] += 3 * (*this)[i][2];
        high[2*i+1][0] += 0 * (*this)[i][2];
        high[2*i+1][1] += 0 * (*this)[i][2];
        high[2*i+1][2] += 1 * (*this)[i][2];
      }
      high.denominator = denominator * 4;
    }

    template< int Degree >
    void BSplineElements< Degree >::differentiate( BSplineElements< Degree-1 >& d ) const
    {
      d.resize( this->size() );
      d.assign( d.size()  , BSplineElementCoefficients< Degree-1 >() );
      for( int i=0 ; i<int(this->size()) ; i++ ) for( int j=0 ; j<=Degree ; j++ )
      {
        if( j-1>=0 )   d[i][j-1] -= (*this)[i][j];
        if( j<Degree ) d[i][j  ] += (*this)[i][j];
      }
      d.denominator = denominator;
    }
    // If we were really good, we would implement this integral table to store
    // rational values to improve precision...
    template< int Degree1 , int Degree2 >
    void SetBSplineElementIntegrals( double integrals[Degree1+1][Degree2+1] )
    {
      for( int i=0 ; i<=Degree1 ; i++ )
      {
        Polynomial< Degree1 > p1 = Polynomial< Degree1 >::BSplineComponent( i );
        for( int j=0 ; j<=Degree2 ; j++ )
        {
          Polynomial< Degree2 > p2 = Polynomial< Degree2 >::BSplineComponent( j );
          integrals[i][j] = ( p1 * p2 ).integral( 0 , 1 );
        }
      }
    }


  }
}

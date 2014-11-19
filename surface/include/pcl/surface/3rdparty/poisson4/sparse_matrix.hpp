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

#include <float.h>
#ifdef _WIN32
# ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
# endif // WIN32_LEAN_AND_MEAN
# ifndef NOMINMAX
#  define NOMINMAX
# endif // NOMINMAX
# include <windows.h>
#endif //_WIN32

///////////////////
//  SparseMatrix //
///////////////////
///////////////////////////////////////////
// Static Allocator Methods and Memebers //
///////////////////////////////////////////

namespace pcl
{
  namespace poisson
  {


    template<class T> int SparseMatrix<T>::UseAlloc=0;
    template<class T> Allocator<MatrixEntry<T> > SparseMatrix<T>::internalAllocator;
    template<class T> int SparseMatrix<T>::UseAllocator(void){return UseAlloc;}
    template<class T>
    void SparseMatrix<T>::SetAllocator( int blockSize )
    {
      if(blockSize>0){
        UseAlloc=1;
        internalAllocator.set(blockSize);
      }
      else{UseAlloc=0;}
    }
    ///////////////////////////////////////
    // SparseMatrix Methods and Memebers //
    ///////////////////////////////////////

    template< class T >
    SparseMatrix< T >::SparseMatrix( void )
    {
      _contiguous = false;
      _maxEntriesPerRow = 0;
      rows = 0;
      rowSizes = NULL;
      m_ppElements = NULL;
    }

    template< class T > SparseMatrix< T >::SparseMatrix( int rows                        ) : SparseMatrix< T >() { Resize( rows ); }
    template< class T > SparseMatrix< T >::SparseMatrix( int rows , int maxEntriesPerRow ) : SparseMatrix< T >() { Resize( rows , maxEntriesPerRow ); }

    template< class T >
    SparseMatrix< T >::SparseMatrix( const SparseMatrix& M ) : SparseMatrix< T >()
    {
      if( M._contiguous ) Resize( M.rows , M._maxEntriesPerRow );
      else                Resize( M.rows );
      for( int i=0 ; i<rows ; i++ )
      {
        SetRowSize( i , M.rowSizes[i] );
        memcpy( (*this)[i] , M[i] , sizeof( MatrixEntry< T > ) * rowSizes[i] );
      }
    }
    template<class T>
    int SparseMatrix<T>::Entries( void ) const
    {
      int e = 0;
      for( int i=0 ; i<rows ; i++ ) e += int( rowSizes[i] );
      return e;
    }
    template<class T>
    SparseMatrix<T>& SparseMatrix<T>::operator = (const SparseMatrix<T>& M)
    {
      if( M._contiguous ) Resize( M.rows , M._maxEntriesPerRow );
      else                Resize( M.rows );
      for( int i=0 ; i<rows ; i++ )
      {
        SetRowSize( i , M.rowSizes[i] );
        memcpy( (*this)[i] , M[i] , sizeof( MatrixEntry< T > ) * rowSizes[i] );
      }
      return *this;
    }

    template<class T>
    SparseMatrix<T>::~SparseMatrix( void ){ Resize( 0 ); }

    template< class T >
    bool SparseMatrix< T >::write( const char* fileName ) const
    {
      FILE* fp = fopen( fileName , "wb" );
      if( !fp ) return false;
      bool ret = write( fp );
      fclose( fp );
      return ret;
    }
    template< class T >
    bool SparseMatrix< T >::read( const char* fileName )
    {
      FILE* fp = fopen( fileName , "rb" );
      if( !fp ) return false;
      bool ret = read( fp );
      fclose( fp );
      return ret;
    }
    template< class T >
    bool SparseMatrix< T >::write( FILE* fp ) const
    {
      if( fwrite( &rows , sizeof( int ) , 1 , fp )!=1 ) return false;
      if( fwrite( rowSizes , sizeof( int ) , rows , fp )!=rows ) return false;
      for( int i=0 ; i<rows ; i++ ) if( fwrite( (*this)[i] , sizeof( MatrixEntry< T > ) , rowSizes[i] , fp )!=rowSizes[i] ) return false;
      return true;
    }
    template< class T >
    bool SparseMatrix< T >::read( FILE* fp )
    {
      int r;
      if( fread( &r , sizeof( int ) , 1 , fp )!=1 ) return false;
      Resize( r );
      if( fread( rowSizes , sizeof( int ) , rows , fp )!=rows ) return false;
      for( int i=0 ; i<rows ; i++ )
      {
        r = rowSizes[i];
        rowSizes[i] = 0;
        SetRowSize( i , r );
        if( fread( (*this)[i] , sizeof( MatrixEntry< T > ) , rowSizes[i] , fp )!=rowSizes[i] ) return false;
      }
      return true;
    }


    template< class T >
    void SparseMatrix< T >::Resize( int r )
    {
      if( rows>0 )
      {

        if( !UseAlloc )
          if( _contiguous ){ if( _maxEntriesPerRow ) free( m_ppElements[0] ); }
          else for( int i=0 ; i<rows ; i++ ){ if( rowSizes[i] ) free( m_ppElements[i] ); }
        free( m_ppElements );
        free( rowSizes );
      }
      rows = r;
      if( r )
      {
        rowSizes = ( int* )malloc( sizeof( int ) * r );
        memset( rowSizes , 0 , sizeof( int ) * r );
        m_ppElements = ( MatrixEntry< T >** )malloc( sizeof( MatrixEntry< T >* ) * r );
      }
      _contiguous = false;
      _maxEntriesPerRow = 0;
    }
    template< class T >
    void SparseMatrix< T >::Resize( int r , int e )
    {
      if( rows>0 )
      {
        if( !UseAlloc )
          if( _contiguous ){ if( _maxEntriesPerRow ) free( m_ppElements[0] ); }
          else for( int i=0 ; i<rows ; i++ ){ if( rowSizes[i] ) free( m_ppElements[i] ); }
        free( m_ppElements );
        free( rowSizes );
      }
      rows = r;
      if( r )
      {
        rowSizes = ( int* )malloc( sizeof( int ) * r );
        memset( rowSizes , 0 , sizeof( int ) * r );
        m_ppElements = ( MatrixEntry< T >** )malloc( sizeof( MatrixEntry< T >* ) * r );
        m_ppElements[0] = ( MatrixEntry< T >* )malloc( sizeof( MatrixEntry< T > ) * r * e );
        for( int i=1 ; i<r ; i++ ) m_ppElements[i] = m_ppElements[i-1] + e;
      }
      _contiguous = true;
      _maxEntriesPerRow = e;
    }

    template<class T>
    void SparseMatrix< T >::SetRowSize( int row , int count )
    {
      if( _contiguous )
      {
        if( count>_maxEntriesPerRow ) fprintf( stderr , "[ERROR] Cannot set row size on contiguous matrix: %d<=%d\n" , count , _maxEntriesPerRow ) , exit( 0 );
        rowSizes[row] = count;
      }
      else if( row>=0 && row<rows )
      {
        if( UseAlloc ) m_ppElements[row] = internalAllocator.newElements(count);
        else
        {
          if( rowSizes[row] ) free( m_ppElements[row] );
          if( count>0 ) m_ppElements[row] = ( MatrixEntry< T >* )malloc( sizeof( MatrixEntry< T > ) * count );
        }
      }
    }


    template<class T>
    void SparseMatrix<T>::SetZero()
    {
      Resize(this->m_N, this->m_M);
    }

    template<class T>
    void SparseMatrix<T>::SetIdentity()
    {
      SetZero();
      for(int ij=0; ij < Min( this->Rows(), this->Columns() ); ij++)
        (*this)(ij,ij) = T(1);
    }

    template<class T>
    SparseMatrix<T> SparseMatrix<T>::operator * (const T& V) const
    {
      SparseMatrix<T> M(*this);
      M *= V;
      return M;
    }

    template<class T>
    SparseMatrix<T>& SparseMatrix<T>::operator *= (const T& V)
    {
      for (int i=0; i<this->Rows(); i++)
      {
        for(int ii=0;ii<m_ppElements[i].size();i++){m_ppElements[i][ii].Value*=V;}
      }
      return *this;
    }

    template<class T>
    SparseMatrix<T> SparseMatrix<T>::Multiply( const SparseMatrix<T>& M ) const
    {
      SparseMatrix<T> R( this->Rows(), M.Columns() );
      for(int i=0; i<R.Rows(); i++){
        for(int ii=0;ii<m_ppElements[i].size();ii++){
          int N=m_ppElements[i][ii].N;
          T Value=m_ppElements[i][ii].Value;
          for(int jj=0;jj<M.m_ppElements[N].size();jj++){
            R(i,M.m_ppElements[N][jj].N) += Value * M.m_ppElements[N][jj].Value;
          }
        }
      }
      return R;
    }

    template<class T>
    template<class T2>
    Vector<T2> SparseMatrix<T>::Multiply( const Vector<T2>& V ) const
    {
      Vector<T2> R( rows );

      for (int i=0; i<rows; i++)
      {
        T2 temp=T2();
        for(int ii=0;ii<rowSizes[i];ii++){
          temp+=m_ppElements[i][ii].Value * V.m_pV[m_ppElements[i][ii].N];
        }
        R(i)=temp;
      }
      return R;
    }

    template<class T>
    template<class T2>
    void SparseMatrix<T>::Multiply( const Vector<T2>& In , Vector<T2>& Out , int threads ) const
    {
#pragma omp parallel for num_threads( threads ) schedule( static )
      for( int i=0 ; i<rows ; i++ )
      {
        T2 temp = T2();
        temp *= 0;
        for( int j=0 ; j<rowSizes[i] ; j++ ) temp += m_ppElements[i][j].Value * In.m_pV[m_ppElements[i][j].N];
        Out.m_pV[i] = temp;
      }
    }

    template<class T>
    SparseMatrix<T> SparseMatrix<T>::operator * (const SparseMatrix<T>& M) const
    {
      return Multiply(M);
    }
    template<class T>
    template<class T2>
    Vector<T2> SparseMatrix<T>::operator * (const Vector<T2>& V) const
    {
      return Multiply(V);
    }

    template<class T>
    SparseMatrix<T> SparseMatrix<T>::Transpose() const
    {
      SparseMatrix<T> M( this->Columns(), this->Rows() );

      for (int i=0; i<this->Rows(); i++)
      {
        for(int ii=0;ii<m_ppElements[i].size();ii++){
          M(m_ppElements[i][ii].N,i) = m_ppElements[i][ii].Value;
        }
      }
      return M;
    }

    template<class T>
    template<class T2>
    int SparseMatrix<T>::SolveSymmetric( const SparseMatrix<T>& M , const Vector<T2>& b , int iters , Vector<T2>& solution , const T2 eps , int reset , int threads )
    {
      if( reset )
      {
        solution.Resize( b.Dimensions() );
        solution.SetZero();
      }
      Vector< T2 > r;
      r.Resize( solution.Dimensions() );
      M.Multiply( solution , r );
      r = b - r;
      Vector< T2 > d = r;
      double delta_new , delta_0;
      for( int i=0 ; i<r.Dimensions() ; i++ ) delta_new += r.m_pV[i] * r.m_pV[i];
      delta_0 = delta_new;
      if( delta_new<eps ) return 0;
      int ii;
      Vector< T2 > q;
      q.Resize( d.Dimensions() );
      for( ii=0; ii<iters && delta_new>eps*delta_0 ; ii++ )
      {
        M.Multiply( d , q , threads );
        double dDotQ = 0 , alpha = 0;
        for( int i=0 ; i<d.Dimensions() ; i++ ) dDotQ += d.m_pV[i] * q.m_pV[i];
        alpha = delta_new / dDotQ;
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<r.Dimensions() ; i++ ) solution.m_pV[i] += d.m_pV[i] * T2( alpha );
        if( !(ii%50) )
        {
          r.Resize( solution.Dimensions() );
          M.Multiply( solution , r , threads );
          r = b - r;
        }
        else
#pragma omp parallel for num_threads( threads ) schedule( static )
          for( int i=0 ; i<r.Dimensions() ; i++ ) r.m_pV[i] = r.m_pV[i] - q.m_pV[i] * T2(alpha);

        double delta_old = delta_new , beta;
        delta_new = 0;
        for( int i=0 ; i<r.Dimensions() ; i++ ) delta_new += r.m_pV[i]*r.m_pV[i];
        beta = delta_new / delta_old;
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<d.Dimensions() ; i++ ) d.m_pV[i] = r.m_pV[i] + d.m_pV[i] * T2( beta );
      }
      return ii;
    }

    // Solve for x s.t. M(x)=b by solving for x s.t. M^tM(x)=M^t(b)
    template<class T>
    int SparseMatrix<T>::Solve(const SparseMatrix<T>& M,const Vector<T>& b,int iters,Vector<T>& solution,const T eps){
      SparseMatrix mTranspose=M.Transpose();
      Vector<T> bb=mTranspose*b;
      Vector<T> d,r,Md;
      T alpha,beta,rDotR;
      int i;

      solution.Resize(M.Columns());
      solution.SetZero();

      d=r=bb;
      rDotR=r.Dot(r);
      for(i=0;i<iters && rDotR>eps;i++){
        T temp;
        Md=mTranspose*(M*d);
        alpha=rDotR/d.Dot(Md);
        solution+=d*alpha;
        r-=Md*alpha;
        temp=r.Dot(r);
        beta=temp/rDotR;
        rDotR=temp;
        d=r+d*beta;
      }
      return i;
    }




    ///////////////////////////
    // SparseSymmetricMatrix //
    ///////////////////////////
    template<class T>
    template<class T2>
    Vector<T2> SparseSymmetricMatrix<T>::operator * (const Vector<T2>& V) const {return Multiply(V);}
    template<class T>
    template<class T2>
    Vector<T2> SparseSymmetricMatrix<T>::Multiply( const Vector<T2>& V ) const
    {
      Vector<T2> R( SparseMatrix<T>::rows );

      for(int i=0; i<SparseMatrix<T>::rows; i++){
        for(int ii=0;ii<SparseMatrix<T>::rowSizes[i];ii++){
          int j=SparseMatrix<T>::m_ppElements[i][ii].N;
          R(i)+=SparseMatrix<T>::m_ppElements[i][ii].Value * V.m_pV[j];
          R(j)+=SparseMatrix<T>::m_ppElements[i][ii].Value * V.m_pV[i];
        }
      }
      return R;
    }

    template<class T>
    template<class T2>
    void SparseSymmetricMatrix<T>::Multiply( const Vector<T2>& In , Vector<T2>& Out , bool addDCTerm ) const
    {
      Out.SetZero();
      const T2* in = &In[0];
      T2* out = &Out[0];
      T2 dcTerm = T2( 0 );
      if( addDCTerm )
      {
        for( int i=0 ; i<SparseMatrix<T>::rows ; i++ ) dcTerm += in[i];
        dcTerm /= SparseMatrix<T>::rows;
      }
      for( int i=0 ; i<this->SparseMatrix<T>::rows ; i++ )
      {
        const MatrixEntry<T>* temp = SparseMatrix<T>::m_ppElements[i];
        const MatrixEntry<T>* end = temp + SparseMatrix<T>::rowSizes[i];
        const T2& in_i_ = in[i];
        T2 out_i = T2(0);
        for( ; temp!=end ; temp++ )
        {
          int j=temp->N;
          T2 v=temp->Value;
          out_i += v * in[j];
          out[j] += v * in_i_;
        }
        out[i] += out_i;
      }
      if( addDCTerm ) for( int i=0 ; i<SparseMatrix<T>::rows ; i++ ) out[i] += dcTerm;
    }
    template<class T>
    template<class T2>
    void SparseSymmetricMatrix<T>::Multiply( const Vector<T2>& In , Vector<T2>& Out , MapReduceVector< T2 >& OutScratch , bool addDCTerm ) const
    {
      int dim = int( In.Dimensions() );
      const T2* in = &In[0];
      int threads = OutScratch.threads();
      if( addDCTerm )
      {
        T2 dcTerm = 0;
#pragma omp parallel for num_threads( threads ) reduction ( + : dcTerm )
        for( int t=0 ; t<threads ; t++ )
        {
          T2* out = OutScratch[t];
          memset( out , 0 , sizeof( T2 ) * dim );
          for( int i=(SparseMatrix<T>::rows*t)/threads ; i<(SparseMatrix<T>::rows*(t+1))/threads ; i++ )
          {
            const T2& in_i_ = in[i];
            T2& out_i_ = out[i];
            for( const MatrixEntry< T > *temp = SparseMatrix<T>::m_ppElements[i] , *end = temp+SparseMatrix<T>::rowSizes[i] ; temp!=end ; temp++ )
            {
              int j = temp->N;
              T2 v = temp->Value;
              out_i_ += v * in[j];
              out[j] += v * in_i_;
            }
            dcTerm += in_i_;
          }
        }
        dcTerm /= dim;
        dim = int( Out.Dimensions() );
        T2* out = &Out[0];
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<dim ; i++ )
        {
          T2 _out = dcTerm;
          for( int t=0 ; t<threads ; t++ ) _out += OutScratch[t][i];
          out[i] = _out;
        }
      }
      else
      {
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
        {
          T2* out = OutScratch[t];
          memset( out , 0 , sizeof( T2 ) * dim );
          for( int i=(SparseMatrix<T>::rows*t)/threads ; i<(SparseMatrix<T>::rows*(t+1))/threads ; i++ )
          {
            const T2& in_i_ = in[i];
            T2& out_i_ = out[i];
            for( const MatrixEntry< T > *temp = SparseMatrix<T>::m_ppElements[i] , *end = temp+SparseMatrix<T>::rowSizes[i] ; temp!=end ; temp++ )
            {
              int j = temp->N;
              T2 v = temp->Value;
              out_i_ += v * in[j];
              out[j] += v * in_i_;
            }
          }
        }
        dim = int( Out.Dimensions() );
        T2* out = &Out[0];
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<dim ; i++ )
        {
          T2 _out = T2(0);
          for( int t=0 ; t<threads ; t++ ) _out += OutScratch[t][i];
          out[i] = _out;
        }
      }
    }
    template<class T>
    template<class T2>
    void SparseSymmetricMatrix<T>::Multiply( const Vector<T2>& In , Vector<T2>& Out , std::vector< T2* >& OutScratch , const std::vector< int >& bounds ) const
    {
      int dim = In.Dimensions();
      const T2* in = &In[0];
      int threads = OutScratch.size();
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
        for( int i=0 ; i<dim ; i++ ) OutScratch[t][i] = T2(0);
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        T2* out = OutScratch[t];
        for( int i=bounds[t] ; i<bounds[t+1] ; i++ )
        {
          const MatrixEntry<T>* temp = SparseMatrix<T>::m_ppElements[i];
          const MatrixEntry<T>* end = temp + SparseMatrix<T>::rowSizes[i];
          const T2& in_i_ = in[i];
          T2& out_i_ = out[i];
          for(  ; temp!=end ; temp++ )
          {
            int j = temp->N;
            T2 v = temp->Value;
            out_i_ += v * in[j];
            out[j] += v * in_i_;
          }
        }
      }
      T2* out = &Out[0];
#pragma omp parallel for num_threads( threads ) schedule( static )
      for( int i=0 ; i<Out.Dimensions() ; i++ )
      {
        T2& _out = out[i];
        _out = T2(0);
        for( int t=0 ; t<threads ; t++ ) _out += OutScratch[t][i];
      }
    }
#if defined _WIN32 && !defined __MINGW32__
#ifndef _AtomicIncrement_
#define _AtomicIncrement_
    inline void AtomicIncrement( volatile float* ptr , float addend )
    {
      float newValue = *ptr;
      LONG& _newValue = *( (LONG*)&newValue );
      LONG  _oldValue;
      for( ;; )
      {
        _oldValue = _newValue;
        newValue += addend;
        _newValue = InterlockedCompareExchange( (LONG*) ptr , _newValue , _oldValue );
        if( _newValue==_oldValue ) break;
      }
    }
    inline void AtomicIncrement( volatile double* ptr , double addend )
    //inline void AtomicIncrement( double* ptr , double addend )
    {
      double newValue = *ptr;
      LONGLONG& _newValue = *( (LONGLONG*)&newValue );
      LONGLONG  _oldValue;
      do
      {
        _oldValue = _newValue;
        newValue += addend;
        _newValue = InterlockedCompareExchange64( (LONGLONG*) ptr , _newValue , _oldValue );
      }
      while( _newValue!=_oldValue );
    }
#endif // _AtomicIncrement_
    template< class T >
    void MultiplyAtomic( const SparseSymmetricMatrix< T >& A , const Vector< float >& In , Vector< float >& Out , int threads , const int* partition=NULL )
    {
      Out.SetZero();
      const float* in = &In[0];
      float* out = &Out[0];
      if( partition )
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
          for( int i=partition[t] ; i<partition[t+1] ; i++ )
          {
            const MatrixEntry< T >* temp = A[i];
            const MatrixEntry< T >* end = temp + A.rowSizes[i];
            const float& in_i = in[i];
            float out_i = 0.;
            for( ; temp!=end ; temp++ )
            {
              int j = temp->N;
              float v = temp->Value;
              out_i += v * in[j];
              AtomicIncrement( out+j , v * in_i );
            }
            AtomicIncrement( out+i , out_i );
          }
      else
#pragma omp parallel for num_threads( threads )
        for( int i=0 ; i<A.rows ; i++ )
        {
          const MatrixEntry< T >* temp = A[i];
          const MatrixEntry< T >* end = temp + A.rowSizes[i];
          const float& in_i = in[i];
          float out_i = 0.f;
          for( ; temp!=end ; temp++ )
          {
            int j = temp->N;
            float v = temp->Value;
            out_i += v * in[j];
            AtomicIncrement( out+j , v * in_i );
          }
          AtomicIncrement( out+i , out_i );
        }
    }
    template< class T >
    void MultiplyAtomic( const SparseSymmetricMatrix< T >& A , const Vector< double >& In , Vector< double >& Out , int threads , const int* partition=NULL )
    {
      Out.SetZero();
      const double* in = &In[0];
      double* out = &Out[0];

      if( partition )
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
          for( int i=partition[t] ; i<partition[t+1] ; i++ )
          {
            const MatrixEntry< T >* temp = A[i];
            const MatrixEntry< T >* end = temp + A.rowSizes[i];
            const double& in_i = in[i];
            double out_i = 0.;
            for( ; temp!=end ; temp++ )
            {
              int j = temp->N;
              T v = temp->Value;
              out_i += v * in[j];
              AtomicIncrement( out+j , v * in_i );
            }
            AtomicIncrement( out+i , out_i );
          }
      else
#pragma omp parallel for num_threads( threads )
        for( int i=0 ; i<A.rows ; i++ )
        {
          const MatrixEntry< T >* temp = A[i];
          const MatrixEntry< T >* end = temp + A.rowSizes[i];
          const double& in_i = in[i];
          double out_i = 0.;
          for( ; temp!=end ; temp++ )
          {
            int j = temp->N;
            T v = temp->Value;
            out_i += v * in[j];
            AtomicIncrement( out+j , v * in_i );
          }
          AtomicIncrement( out+i , out_i );
        }
    }

    template< class T >
    template< class T2 >
    int SparseSymmetricMatrix< T >::SolveAtomic( const SparseSymmetricMatrix< T >& A , const Vector< T2 >& b , int iters , Vector< T2 >& x , T2 eps , int reset , int threads , bool solveNormal )
    {
      eps *= eps;
      int dim = b.Dimensions();
      if( reset )
      {
        x.Resize( dim );
        x.SetZero();
      }
      Vector< T2 > r( dim ) , d( dim ) , q( dim );
      Vector< T2 > temp;
      if( solveNormal ) temp.Resize( dim );
      T2 *_x = &x[0] , *_r = &r[0] , *_d = &d[0] , *_q = &q[0];
      const T2* _b = &b[0];

      std::vector< int > partition( threads+1 );
      {
        int eCount = 0;
        for( int i=0 ; i<A.rows ; i++ ) eCount += A.rowSizes[i];
        partition[0] = 0;
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
        {
          int _eCount = 0;
          for( int i=0 ; i<A.rows ; i++ )
          {
            _eCount += A.rowSizes[i];
            if( _eCount*threads>=eCount*(t+1) )
            {
              partition[t+1] = i;
              break;
            }
          }
        }
        partition[threads] = A.rows;
      }
      if( solveNormal )
      {
        MultiplyAtomic( A , x , temp , threads , &partition[0] );
        MultiplyAtomic( A , temp , r , threads , &partition[0] );
        MultiplyAtomic( A , b , temp , threads , &partition[0] );
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] = temp[i] - _r[i];
      }
      else
      {
        MultiplyAtomic( A , x , r , threads , &partition[0] );
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] = _b[i] - _r[i];
      }
      double delta_new = 0 , delta_0;
      for( size_t i=0 ; i<dim ; i++ ) delta_new += _r[i] * _r[i];
      delta_0 = delta_new;
      if( delta_new<eps )
      {
        fprintf( stderr , "[WARNING] Initial residual too low: %g < %f\n" , delta_new , eps );
        return 0;
      }
      int ii;
      for( ii=0; ii<iters && delta_new>eps*delta_0 ; ii++ )
      {
        if( solveNormal ) MultiplyAtomic( A , d , temp , threads , &partition[0] ) , MultiplyAtomic( A , temp , q , threads , &partition[0] );
        else              MultiplyAtomic( A , d , q , threads , &partition[0] );
        double dDotQ = 0;
        for( int i=0 ; i<dim ; i++ ) dDotQ += _d[i] * _q[i];
        T2 alpha = T2( delta_new / dDotQ );
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<dim ; i++ ) _x[i] += _d[i] * alpha;
        if( (ii%50)==(50-1) )
        {
          r.Resize( dim );
          if( solveNormal ) MultiplyAtomic( A , x , temp , threads , &partition[0] ) , MultiplyAtomic( A , temp , r , threads , &partition[0] );
          else              MultiplyAtomic( A , x , r , threads , &partition[0] );
#pragma omp parallel for num_threads( threads ) schedule( static )
          for( int i=0 ; i<dim ; i++ ) _r[i] = _b[i] - _r[i];
        }
        else
#pragma omp parallel for num_threads( threads ) schedule( static )
          for( int i=0 ; i<dim ; i++ ) _r[i] -= _q[i] * alpha;

        double delta_old = delta_new;
        delta_new = 0;
        for( size_t i=0 ; i<dim ; i++ ) delta_new += _r[i] * _r[i];
        T2 beta = T2( delta_new / delta_old );
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] + _d[i] * beta;
      }
      return ii;
    }
#endif // _WIN32 && !__MINGW32__
    template< class T >
    template< class T2 >
    int SparseSymmetricMatrix< T >::Solve( const SparseSymmetricMatrix<T>& A , const Vector<T2>& b , int iters , Vector<T2>& x , MapReduceVector< T2 >& scratch , T2 eps , int reset , bool addDCTerm , bool solveNormal )
    {
      int threads = scratch.threads();
      eps *= eps;
      int dim = int( b.Dimensions() );
      Vector< T2 > r( dim ) , d( dim ) , q( dim ) , temp;
      if( reset ) x.Resize( dim );
      if( solveNormal ) temp.Resize( dim );
      T2 *_x = &x[0] , *_r = &r[0] , *_d = &d[0] , *_q = &q[0];
      const T2* _b = &b[0];

      double delta_new = 0 , delta_0;
      if( solveNormal )
      {
        A.Multiply( x , temp , scratch , addDCTerm ) , A.Multiply( temp , r , scratch , addDCTerm ) , A.Multiply( b , temp , scratch , addDCTerm );
#pragma omp parallel for num_threads( threads ) reduction( + : delta_new )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] = temp[i] - _r[i] , delta_new += _r[i] * _r[i];
      }
      else
      {
        A.Multiply( x , r , scratch , addDCTerm );
#pragma omp parallel for num_threads( threads )  reduction ( + : delta_new )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] = _b[i] - _r[i] , delta_new += _r[i] * _r[i];
      }
      delta_0 = delta_new;
      if( delta_new<eps )
      {
        fprintf( stderr , "[WARNING] Initial residual too low: %g < %f\n" , delta_new , eps );
        return 0;
      }
      int ii;
      for( ii=0 ; ii<iters && delta_new>eps*delta_0 ; ii++ )
      {
        if( solveNormal ) A.Multiply( d , temp , scratch , addDCTerm ) , A.Multiply( temp , q , scratch , addDCTerm );
        else              A.Multiply( d , q , scratch , addDCTerm );
        double dDotQ = 0;
#pragma omp parallel for num_threads( threads ) reduction( + : dDotQ )
        for( int i=0 ; i<dim ; i++ ) dDotQ += _d[i] * _q[i];
        T2 alpha = T2( delta_new / dDotQ );
        double delta_old = delta_new;
        delta_new = 0;
        if( (ii%50)==(50-1) )
        {
#pragma omp parallel for num_threads( threads )
          for( int i=0 ; i<dim ; i++ ) _x[i] += _d[i] * alpha;
          r.Resize( dim );
          if( solveNormal ) A.Multiply( x , temp , scratch , addDCTerm ) , A.Multiply( temp , r , scratch , addDCTerm );
          else              A.Multiply( x , r , scratch , addDCTerm );
#pragma omp parallel for num_threads( threads ) reduction( + : delta_new )
          for( int i=0 ; i<dim ; i++ ) _r[i] = _b[i] - _r[i] , delta_new += _r[i] * _r[i] , _x[i] += _d[i] * alpha;
        }
        else
#pragma omp parallel for num_threads( threads ) reduction( + : delta_new )
          for( int i=0 ; i<dim ; i++ ) _r[i] -= _q[i] * alpha , delta_new += _r[i] * _r[i] ,  _x[i] += _d[i] * alpha;

        T2 beta = T2( delta_new / delta_old );
#pragma omp parallel for num_threads( threads )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] + _d[i] * beta;
      }
      return ii;
    }
    template< class T >
    template< class T2 >
    int SparseSymmetricMatrix<T>::Solve( const SparseSymmetricMatrix<T>& A , const Vector<T2>& b , int iters , Vector<T2>& x , T2 eps , int reset , int threads , bool addDCTerm , bool solveNormal )
    {
      eps *= eps;
      int dim = int( b.Dimensions() );
      MapReduceVector< T2 > outScratch;
      if( threads<1 ) threads = 1;
      if( threads>1 ) outScratch.resize( threads , dim );
      if( reset ) x.Resize( dim );
      Vector< T2 > r( dim ) , d( dim ) , q( dim );
      Vector< T2 > temp;
      if( solveNormal ) temp.Resize( dim );
      T2 *_x = &x[0] , *_r = &r[0] , *_d = &d[0] , *_q = &q[0];
      const T2* _b = &b[0];

      double delta_new = 0 , delta_0;

      if( solveNormal )
      {
        if( threads>1 ) A.Multiply( x , temp , outScratch , addDCTerm ) , A.Multiply( temp , r , outScratch , addDCTerm ) , A.Multiply( b , temp , outScratch , addDCTerm );
        else            A.Multiply( x , temp , addDCTerm )              , A.Multiply( temp , r , addDCTerm )              , A.Multiply( b , temp , addDCTerm );
#pragma omp parallel for num_threads( threads ) reduction( + : delta_new )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] = temp[i] - _r[i] , delta_new += _r[i] * _r[i];
      }
      else
      {
        if( threads>1 ) A.Multiply( x , r , outScratch , addDCTerm );
        else            A.Multiply( x , r , addDCTerm );
#pragma omp parallel for num_threads( threads ) reduction( + : delta_new )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] = _b[i] - _r[i] , delta_new += _r[i] * _r[i];
      }

      delta_0 = delta_new;
      if( delta_new<eps )
      {
        fprintf( stderr , "[WARNING] Initial residual too low: %g < %f\n" , delta_new , eps );
        return 0;
      }
      int ii;
      for( ii=0 ; ii<iters && delta_new>eps*delta_0 ; ii++ )
      {
        if( solveNormal )
        {
          if( threads>1 ) A.Multiply( d , temp , outScratch , addDCTerm ) , A.Multiply( temp , q , outScratch , addDCTerm );
          else            A.Multiply( d , temp , addDCTerm )              , A.Multiply( temp , q , addDCTerm );
        }
        else
        {
          if( threads>1 ) A.Multiply( d , q , outScratch , addDCTerm );
          else            A.Multiply( d , q , addDCTerm );
        }
        double dDotQ = 0;
#pragma omp parallel for num_threads( threads ) reduction( + : dDotQ )
        for( int i=0 ; i<dim ; i++ ) dDotQ += _d[i] * _q[i];
        T2 alpha = T2( delta_new / dDotQ );
        double delta_old = delta_new;
        delta_new = 0;

        if( (ii%50)==(50-1) )
        {
#pragma omp parallel for num_threads( threads )
          for( int i=0 ; i<dim ; i++ ) _x[i] += _d[i] * alpha;
          r.SetZero();
          if( solveNormal )
          {
            if( threads>1 ) A.Multiply( x , temp , outScratch , addDCTerm ) , A.Multiply( temp , r , outScratch , addDCTerm );
            else            A.Multiply( x , temp , addDCTerm )              , A.Multiply( temp , r , addDCTerm );
          }
          else
          {
            if( threads>1 ) A.Multiply( x , r , outScratch , addDCTerm );
            else            A.Multiply( x , r , addDCTerm );
          }
#pragma omp parallel for num_threads( threads ) reduction ( + : delta_new )
          for( int i=0 ; i<dim ; i++ ) _r[i] = _b[i] - _r[i] , delta_new += _r[i] * _r[i] , _x[i] += _d[i] * alpha;
        }
        else
        {
#pragma omp parallel for num_threads( threads ) reduction( + : delta_new )
          for( int i=0 ; i<dim ; i++ ) _r[i] -= _q[i] * alpha , delta_new += _r[i] * _r[i] , _x[i] += _d[i] * alpha;
        }

        T2 beta = T2( delta_new / delta_old );
#pragma omp parallel for num_threads( threads )
        for( int i=0 ; i<dim ; i++ ) _d[i] = _r[i] + _d[i] * beta;
      }
      return ii;
    }

    template<class T>
    template<class T2>
    int SparseSymmetricMatrix<T>::Solve( const SparseSymmetricMatrix<T>& M , const Vector<T2>& diagonal , const Vector<T2>& b , int iters , Vector<T2>& solution , int reset )
    {
      Vector<T2> d,r,Md;

      if(reset)
      {
        solution.Resize(b.Dimensions());
        solution.SetZero();
      }
      Md.Resize(M.rows);
      for( int i=0 ; i<iters ; i++ )
      {
        M.Multiply( solution , Md );
        r = b-Md;
        // solution_new[j] * diagonal[j] + ( Md[j] - solution_old[j] * diagonal[j] ) = b[j]
        // solution_new[j] = ( b[j] - ( Md[j] - solution_old[j] * diagonal[j] ) ) / diagonal[j]
        // solution_new[j] = ( b[j] - Md[j] ) / diagonal[j] + solution_old[j]
        for( int j=0 ; j<int(M.rows) ; j++ ) solution[j] += (b[j]-Md[j])/diagonal[j];
      }
      return iters;
    }
    template< class T >
    template< class T2 >
    void SparseSymmetricMatrix< T >::getDiagonal( Vector< T2 >& diagonal ) const
    {
      diagonal.Resize( SparseMatrix<T>::rows );
      for( int i=0 ; i<SparseMatrix<T>::rows ; i++ )
      {
        diagonal[i] = 0.;
        for( int j=0 ; j<SparseMatrix<T>::rowSizes[i] ; j++ ) if( SparseMatrix<T>::m_ppElements[i][j].N==i ) diagonal[i] += SparseMatrix<T>::m_ppElements[i][j].Value * 2;
      }
    }

  }
}

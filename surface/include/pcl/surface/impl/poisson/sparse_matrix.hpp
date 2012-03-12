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

#include <float.h>


namespace pcl 
{
  namespace poisson 
  {
    ///////////////////
    //  SparseMatrix //
    ///////////////////
    ///////////////////////////////////////////
    // Static Allocator Methods and Memebers //
    ///////////////////////////////////////////
    template<class T> int SparseMatrix<T>::UseAlloc=0;
    template<class T> Allocator<MatrixEntry<T> > SparseMatrix<T>::AllocatorMatrixEntry;
    template<class T> int SparseMatrix<T>::UseAllocator(void){return UseAlloc;}
    template<class T>
    void SparseMatrix<T>::SetAllocator(const int& blockSize){
      if(blockSize>0){
        UseAlloc=1;
        AllocatorMatrixEntry.set(blockSize);
      }
      else{UseAlloc=0;}
    }
    ///////////////////////////////////////
    // SparseMatrix Methods and Memebers //
    ///////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T>
    SparseMatrix<T>::SparseMatrix () : rows (0), rowSizes (NULL), m_ppElements (NULL)
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T>
    SparseMatrix<T>::SparseMatrix (int rows) : rows (rows), rowSizes (NULL), m_ppElements (NULL)
    {
      Resize (rows);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T>
    SparseMatrix<T>::SparseMatrix (const SparseMatrix& M) :
      rows (M.rows), rowSizes (M.rowSizes), m_ppElements (M.m_ppElements)
    {
      Resize (M.rows);
      for (int i = 0; i < rows; i++)
      {
        SetRowSize (i, M.rowSizes[i]);
        for (int j = 0; j < rowSizes[i]; j++)
          m_ppElements[i][j] = M.m_ppElements[i][j];
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> int 
    SparseMatrix<T>::Entries ()
    {
      int e = 0;
      for (int i = 0; i < rows; i++)
        e += int (rowSizes[i]);

      return (e);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T>
    SparseMatrix<T>& SparseMatrix<T>::operator = (const SparseMatrix<T>& M)
    {
      Resize(M.rows);
      for (int i=0; i<rows; i++){
        SetRowSize(i,M.rowSizes[i]);
        for (int j=0; j<rowSizes[i]; j++){m_ppElements[i][j]=M.m_ppElements[i][j];}
      }
      return *this;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T>
    SparseMatrix<T>::~SparseMatrix(){Resize(0);}

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> void 
    SparseMatrix<T>::Resize (int r)
    {
      int i;
      if(rows>0){
        if(!UseAlloc){for(i=0;i<rows;i++){if(rowSizes[i]){free(m_ppElements[i]);}}}
        free(m_ppElements);
        free(rowSizes);
      }
      rows=r;
      if (r)
      {
        rowSizes = reinterpret_cast<int*> (malloc (sizeof (int) * r));
        
        memset (rowSizes, 0, sizeof (int) * r);
        
        m_ppElements = reinterpret_cast<MatrixEntry<T>**> (malloc (sizeof (MatrixEntry<T>*) * r));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> void 
    SparseMatrix<T>::SetRowSize (int row, int count)
    {
      if (row >= 0 && row < rows)
      {
        if (UseAlloc)
          m_ppElements[row] = AllocatorMatrixEntry.newElements (count);
        else
        {
          if (rowSizes[row])
            free (m_ppElements[row]);
          if (count > 0)
            m_ppElements[row] = reinterpret_cast<MatrixEntry<T>*> (malloc (sizeof (MatrixEntry<T>) * count));
        }
        rowSizes[row] = count;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> void 
    SparseMatrix<T>::SetZero ()
    {
      Resize (this->m_N, this->m_M);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> void 
    SparseMatrix<T>::SetIdentity ()
    {
      SetZero();
      for(int ij=0; ij < Min( this->Rows(), this->Columns() ); ij++)
        (*this)(ij,ij) = T(1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> SparseMatrix<T> 
    SparseMatrix<T>::operator * (const T& V) const
    {
      SparseMatrix<T> M(*this);
      M *= V;
      return M;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> SparseMatrix<T>& 
    SparseMatrix<T>::operator *= (const T& V)
    {
      for (int i=0; i<this->Rows(); i++)
      {
        for(int ii=0;ii<m_ppElements[i].size();i++){m_ppElements[i][ii].Value*=V;}
      }
      return *this;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> SparseMatrix<T> 
    SparseMatrix<T>::Multiply( const SparseMatrix<T>& M ) const
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> Vector<T2> 
    SparseMatrix<T>::Multiply( const Vector<T2>& V ) const
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> void 
    SparseMatrix<T>::Multiply( const Vector<T2>& In,Vector<T2>& Out) const
    {
      for (int i=0; i<rows; i++){
        T2 temp=T2();
        for(int j=0;j<rowSizes[i];j++){temp+=m_ppElements[i][j].Value * In.m_pV[m_ppElements[i][j].N];}
        Out.m_pV[i]=temp;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> SparseMatrix<T> 
    SparseMatrix<T>::operator * (const SparseMatrix<T>& M) const
    {
      return (Multiply (M));
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> Vector<T2> 
    SparseMatrix<T>::operator * (const Vector<T2>& V) const
    {
      return (Multiply (V));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> int 
    SparseMatrix<T>::SolveSymmetric(const SparseMatrix<T>& M,const Vector<T2>& b,const int& iters,Vector<T2>& solution,const T2 eps,const int& reset)
    {
      Vector<T2> d,r,Md;
      T2 alpha,beta,rDotR;
      Md.Resize(b.Dimensions());
      if(reset){
        solution.Resize(b.Dimensions());
        solution.SetZero();
      }
      d=r=b-M.Multiply(solution);
      rDotR=r.Dot(r);
      if(b.Dot(b)<=eps){
        solution.SetZero();
        return 0;
      }

      int i;
      for(i=0;i<iters;i++){
        T2 temp;
        M.Multiply(d,Md);
        temp=d.Dot(Md);
        if(temp<=eps){break;}
        alpha=rDotR/temp;
        r.SubtractScaled(Md,alpha);
        temp=r.Dot(r);
        if(temp/b.Dot(b)<=eps){break;}
        beta=temp/rDotR;
        solution.AddScaled(d,alpha);
        if(beta<=eps){break;}
        rDotR=temp;
        Vector<T2>::Add(d,beta,r,d);
      }
      return i;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Solve for x s.t. M(x)=b by solving for x s.t. M^tM(x)=M^t(b)
    template<class T> int 
    SparseMatrix<T>::Solve(const SparseMatrix<T>& M,const Vector<T>& b,const int& iters,Vector<T>& solution,const T eps)
    {
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

    ////////////////////
    //  SparseNMatrix //
    ////////////////////
    ///////////////////////////////////////////
    // Static Allocator Methods and Memebers //
    ///////////////////////////////////////////
    template<class T,int Dim> int SparseNMatrix<T,Dim>::UseAlloc=0;
    template<class T,int Dim> Allocator<NMatrixEntry<T,Dim> > SparseNMatrix<T,Dim>::AllocatorNMatrixEntry;
    template<class T,int Dim> int SparseNMatrix<T,Dim>::UseAllocator(void){return UseAlloc;}
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim> void 
    SparseNMatrix<T,Dim>::SetAllocator (const int& blockSize)
    {
      if (blockSize>0)
      {
        UseAlloc = 1;
        AllocatorNMatrixEntry.set (blockSize);
      }
      else
        UseAlloc = 0;
    }
    ////////////////////////////////////////
    // SparseNMatrix Methods and Memebers //
    ////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim>::SparseNMatrix () : rows (0), rowSizes (NULL), m_ppElements (NULL)
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim>::SparseNMatrix (int rows) : rows (rows), rowSizes (NULL), m_ppElements (NULL)
    {
      Resize (rows);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim>::SparseNMatrix( const SparseNMatrix& M )
    {
      Resize (M.rows);
      for (int i = 0; i < rows; i++)
      {
        SetRowSize (i, M.rowSizes[i]);
        for (int j = 0; j < rowSizes[i]; j++)
          m_ppElements[i][j] = M.m_ppElements[i][j];
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim> int 
    SparseNMatrix<T,Dim>::Entries ()
    {
      int e = 0;
      for(int i = 0; i < rows; i++)
        e+=int (rowSizes[i]);
      return (e);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim>& SparseNMatrix<T,Dim>::operator = (const SparseNMatrix<T,Dim>& M)
    {
      Resize(M.rows);
      for (int i = 0; i < rows; i++)
      {
        SetRowSize (i, M.rowSizes[i]);
        for (int j = 0; j < rowSizes[i]; j++)
          m_ppElements[i][j] = M.m_ppElements[i][j];
      }
      return (*this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim>::~SparseNMatrix ()
    {
      Resize (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim> void 
    SparseNMatrix<T,Dim>::Resize (int r)
    {
      int i;
      if (rows > 0)
      {
        if (!UseAlloc)
          for (i = 0; i < rows; i++)
            if (rowSizes[i])
              free (m_ppElements[i]);
        free (m_ppElements);
        free (rowSizes);
      }
      rows = r;
      if (r)
      {
        rowSizes = reinterpret_cast<int*> (malloc (sizeof (int) *r));
        memset (rowSizes, 0, sizeof (int) * r);
        m_ppElements = reinterpret_cast<NMatrixEntry<T,Dim>**> (malloc (sizeof (NMatrixEntry<T,Dim>*) * r));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim> void 
    SparseNMatrix<T,Dim>::SetRowSize (int row,int count)
    {
      if (row >= 0 && row < rows)
      {
        if (UseAlloc)
          m_ppElements[row] = AllocatorNMatrixEntry.newElements (count);
        else
        {
          if (rowSizes[row])
            free (m_ppElements[row]);
          if (count>0)
            m_ppElements[row] = reinterpret_cast<NMatrixEntry<T,Dim>*> (malloc (sizeof (NMatrixEntry<T,Dim>) * count));
        }
        rowSizes[row]=count;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim> SparseNMatrix<T,Dim>::operator * (const T& V) const
    {
      SparseNMatrix<T,Dim> M (*this);
      M *= V;
      return (M);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim>
    SparseNMatrix<T,Dim>& SparseNMatrix<T,Dim>::operator *= (const T& V)
    {
      for (int i = 0; i < this->Rows (); i++)
        for (int ii = 0; ii < m_ppElements[i].size (); i++)
          for (int jj = 0; jj < Dim; jj++)
            m_ppElements[i][ii].Value[jj] *= V;
      return (*this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim> template<class T2>
    NVector<T2,Dim> SparseNMatrix<T,Dim>::operator * (const Vector<T2>& V) const
    {
      NVector<T2,Dim> R (rows);

      for (int i = 0; i < rows; i++)
      {
        T2 temp[Dim];
        for (int ii = 0; ii < Dim; ii++)
          temp[ii] = T2 ();
        for (int ii = 0; ii < rowSizes[i]; ii++)
          for (int jj = 0; jj < Dim; jj++)
            temp[jj] += m_ppElements[i][ii].Value[jj] * V.m_pV[m_ppElements[i][jj].N];
        for (int ii = 0; ii < Dim; ii++)
          R[i][ii] = temp[ii];
      }
      return (R);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T,int Dim> template<class T2> Vector<T2> 
    SparseNMatrix<T,Dim>::operator * (const NVector<T2,Dim>& V) const
    {
      Vector<T2> R (rows);

      for (int i=0; i<rows; i++)
      {
        T2 temp ();
        for (int ii = 0; ii < rowSizes[i]; ii++)
        {
          for (int jj = 0; jj < Dim; jj++)
          {
            temp += m_ppElements[i][ii].Value[jj] * V.m_pV[m_ppElements[i][ii].N][jj];
          }
        }
        R(i)  =temp;
      }
      return (R);
    }

    ///////////////////////////
    // SparseSymmetricMatrix //
    ///////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> Vector<T2> 
    SparseSymmetricMatrix<T>::operator * (const Vector<T2>& V) const 
    {
      return (Multiply (V));
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> Vector<T2> 
    SparseSymmetricMatrix<T>::Multiply (const Vector<T2>& V) const
    {
      Vector<T2> R (this->rows);

      for (int i = 0; i < this->rows; i++)
      {
        for (int ii = 0; ii < this->rowSizes[i]; ii++)
        {
          int j = this->m_ppElements[i][ii].N;
          R(i) += this->m_ppElements[i][ii].Value * V.m_pV[j];
          R(j) += this->m_ppElements[i][ii].Value * V.m_pV[i];
        }
      }
      return (R);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> void 
    SparseSymmetricMatrix<T>::Multiply (const Vector<T2>& In,Vector<T2>& Out) const
    {
      Out.SetZero ();
      for (int i = 0; i < this->rows; i++)
      {
        MatrixEntry<T>* temp = this->m_ppElements[i];
        T2& in1 = In.m_pV[i];
        T2& out1 = Out.m_pV[i];
        int rs = this->rowSizes[i];
        for (int ii = 0; ii <rs; ii++)
        {
          MatrixEntry<T>& temp2 = temp[ii];
          int j = temp2.N;
          T2 v = temp2.Value;
          out1 += v * In.m_pV[j];
          Out.m_pV[j] += v * in1;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> int 
    SparseSymmetricMatrix<T>::Solve (
        const SparseSymmetricMatrix<T>& M,
        const Vector<T2>& b,
        const int& iters,
        Vector<T2>& solution,
        const T2 eps,
        const int& reset)
    {
      Vector<T2> d, r, Md;
      T2 alpha,beta,rDotR,bDotB;
      Md.Resize (b.Dimensions ());
      if (reset)
      {
        solution.Resize (b.Dimensions ());
        solution.SetZero ();
      }
      d = r = b - M.Multiply (solution);
      rDotR = r.Dot (r);
      bDotB = b.Dot (b);
      if (b.Dot (b) <= eps)
      {
        solution.SetZero ();
        return (0);
      }
      int i;
      for (i = 0; i < iters; i++)
      {
        T2 temp;
        M.Multiply (d, Md);
        temp = d.Dot (Md);
        if (fabs (temp) <= eps)
          break;
        alpha = rDotR / temp;

        r.SubtractScaled (Md, alpha);
        temp = r.Dot (r);

        if (temp / bDotB <= eps)
          break;

        beta = temp / rDotR;
        solution.AddScaled (d, alpha);
        if (beta<=eps)
          break;
        rDotR = temp;
        Vector<T2>::Add (d, beta, r, d);
      }
      return i;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<class T> template<class T2> int 
    SparseSymmetricMatrix<T>::Solve (
        const SparseSymmetricMatrix<T>& M,
        const Vector<T>& diagonal,
        const Vector<T2>& b,
        const int& iters,
        Vector<T2>& solution,
        const T2 eps,
        const int& reset)
    {
      Vector<T2> d, r, Md;

      if (reset)
      {
        solution.Resize (b.Dimensions ());
        solution.SetZero ();
      }
      Md.Resize (M.rows);
      for (int i = 0; i <iters; i++)
      {
        M.Multiply (solution, Md);
        r = b - Md;
        for (int j = 0; j < int (M.rows); j++)
          solution[j] += r[j] / diagonal[j];
      }
      return (iters);
    }
  }
}

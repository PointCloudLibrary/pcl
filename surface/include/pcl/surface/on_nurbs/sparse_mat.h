/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * 
 *
 */

#pragma once

#include <vector>
#include <map>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Sparse matrix implementation. */
    class SparseMat
    {
    protected:
      std::map<int, std::map<int, double> > m_mat;

    public:
      /** \brief Constructor clearing memory of matrix (=set to zero) */
      SparseMat ()
      {
        m_mat.clear ();
      }

      /** \brief Get a set of values defined by index vectors
       *  \param[in] i vector of row indices
       *  \param[in] j vector of column indices
       *  \param[out] v vector of matrix entries at (i,j)    */
      void
      get (std::vector<int> &i, std::vector<int> &j, std::vector<double> &v);
      /** \brief Get matrix values at index pair (i,j)
       *  \param[in] i row index
       *  \param[in] j column index
       *  return v matrix value at (i,j)       */
      double
      get (int i, int j);
      /** \brief Set matrix values at index pair (i,j)
       *  \param[in] i row index
       *  \param[in] j column index
       *  \param[in] v matrix value at (i,j)       */
      void
      set (int i, int j, double v);

      /** \brief Delete row at index i (= set to zero) */
      void
      deleteRow (int i);
      /** \brief Delete column at index j (= set to zero) */
      void
      deleteColumn (int j);

      /** \brief Clear memory of matrix (=set to zero) */
      inline void
      clear ()
      {
        m_mat.clear ();
      }
      /** \brief Get size of matrix with respect to last non-zero entries */
      void
      size (int &si, int &sj);
      /** \brief Get number of non-zero entries in matrix */
      int
      nonzeros ();
      /** \brief Print all matrix entries with respect to size. */
      void
      printLong ();
      /** \brief Print indices and values of non-zero entries. */
      void
      print ();

    };

  }
}

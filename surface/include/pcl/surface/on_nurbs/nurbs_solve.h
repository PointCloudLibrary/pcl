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

#ifndef _NURBS_SOLVE_H_
#define _NURBS_SOLVE_H_

#undef Success
#include <Eigen/Dense>

#ifdef EIGENVERSION_GEQUAL_3_2
#include <Eigen/Sparse> // May require Eigen 3.2.1 -> preprocessor flag
#endif

#include <pcl/surface/on_nurbs/sparse_mat.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Solves the linear system of equations of the type (A^T*A)*x = A^T*f (normal equations)
    * arising in spline-fitting problems.
    *  Based on paper: TODO
    * \author Thomas Mörwald / Philipp Schapotschnikow
    * \ingroup surface
    */
    class NurbsSolve
    {
    public:
      /** \brief Empty constructor */
      NurbsSolve () :
        m_quiet (true)
#ifdef EIGENVERSION_GEQUAL_3_2
      ,m_KeigenSparse(NULL)
#endif
      {
      }

      ~NurbsSolve()
      {
#ifdef EIGENVERSION_GEQUAL_3_2
        if(m_KeigenSparse!=NULL)
          delete m_KeigenSparse;
#endif
      }

      /** \brief Assign size and dimension (2D, 3D) of system of equations. */
      void
      assign (unsigned rows, unsigned cols, unsigned dims);

      /** \brief Set value for system matrix K (stiffness matrix, basis functions) */
      void
      K (unsigned i, unsigned j, double v);
      /** \brief Set value for state vector x (control points) */
      void
      x (unsigned i, unsigned j, double v);
      /** \brief Set value for target vector f (force vector) */
      void
      f (unsigned i, unsigned j, double v);

#ifdef EIGENVERSION_GEQUAL_3_2
      /** \brief Add a row of the matix A to the set of equations (LHS, RHS) */
//      void AddRow(const std::vector<double> &row,
//                  const std::vector<double> &f);

      /** \brief Add a row to the set of equations (LHS, RHS). Sprasepattern contains indices of non-zero elements of the row */
      void AddRow(const std::vector<double>& row,
                  const std::vector<double>& f,
                  const std::vector<int>& sparsePattern);
#endif

      /** \brief Get value for system matrix K (stiffness matrix, basis functions) */
      double
      K (unsigned i, unsigned j);
      /** \brief Get value for state vector x (control points) */
      double
      x (unsigned i, unsigned j);
      /** \brief Get value for target vector f (force vector) */
      double
      f (unsigned i, unsigned j);

      /** \brief Resize target vector f (force vector) */
      void
      resize (unsigned rows);

      /** \brief Print system matrix K (stiffness matrix, basis functions) */
      void
      printK ();
      /** \brief Print state vector x (control points) */
      void
      printX ();
      /** \brief Print target vector f (force vector) */
      void
      printF ();

      /** \brief Solves the system of equations with respect to x.
       *  - Using UmfPack incredibly speeds up this function.      */
      bool
      solve ();

      /** \brief Compute the difference between solution K*x and target f */
      Eigen::MatrixXd
      diff ();

      /** \brief Enable/Disable debug outputs in console. */
      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
      }

      /** \brief get size of system */
      inline void
      getSize (unsigned &rows, unsigned &cols, unsigned &dims)
      {
        rows = static_cast<unsigned>(m_feig.rows());
        cols = static_cast<unsigned>(m_xeig.rows());
        dims = static_cast<unsigned>(m_feig.cols());
      }

    private:
      bool m_quiet;
      SparseMat m_Ksparse; // Eigen version >= 3.2: The square matrix A^T*A is stored !!
      Eigen::MatrixXd m_Keig;
      Eigen::MatrixXd m_xeig;
      Eigen::MatrixXd m_feig; // Eigen version >= 3.2: The vector A^T*f is stored !!
#ifdef EIGENVERSION_GEQUAL_3_2
      Eigen::SparseMatrix<double>* m_KeigenSparse; // May not be available in older Eigen version
#endif
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

#endif

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

#undef Success
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/surface/on_nurbs/sparse_mat.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Solving the linear system of equations using Eigen or UmfPack.
     * (can be defined in on_nurbs.cmake)*/
    class NurbsSolve
    {
    public:
      /** \brief Empty constructor */
      NurbsSolve () :
        m_quiet (true)
      {
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
      SparseMat m_Ksparse;
      Eigen::MatrixXd m_Keig;
      Eigen::MatrixXd m_xeig;
      Eigen::MatrixXd m_feig;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

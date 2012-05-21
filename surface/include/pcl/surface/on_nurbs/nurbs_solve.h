/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#ifndef _NURBS_SOLVE_H_
#define _NURBS_SOLVE_H_

#undef Success
#include <Eigen/Dense>

#include <pcl/surface/on_nurbs/sparse_mat.h>

namespace pcl
{
  namespace on_nurbs
  {

    class NurbsSolve
    {
    public:
      NurbsSolve () :
        m_quiet (true)
      {
      }

      void
      assign (unsigned rows, unsigned cols, unsigned dims);

      void
      K (unsigned i, unsigned j, double v);
      void
      x (unsigned i, unsigned j, double v);
      void
      f (unsigned i, unsigned j, double v);

      double
      K (unsigned i, unsigned j);
      double
      x (unsigned i, unsigned j);
      double
      f (unsigned i, unsigned j);

      void
      resizeF (unsigned rows);

      void
      printK ();
      void
      printX ();
      void
      printF ();

      bool
      solve ();

      Eigen::MatrixXd
      diff ();

    private:
      bool m_quiet;
      SparseMat m_Ksparse;
      Eigen::MatrixXd m_Keig;
      Eigen::MatrixXd m_xeig;
      Eigen::MatrixXd m_feig;

    };

  }
}

#endif

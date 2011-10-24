/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer
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

#ifndef _NURBS_TOOLS_H_
#define _NURBS_TOOLS_H_

#include <vector>
#include <opennurbs.h>
//#undef Success
#include <Eigen/Dense>

namespace pcl_nurbs
{

  enum
  {
    NORTH = 1, NORTHEAST = 2, EAST = 3, SOUTHEAST = 4, SOUTH = 5, SOUTHWEST = 6, WEST = 7, NORTHWEST = 8
  };

  class NurbsTools
  {

  public:
    ON_NurbsSurface* surf_;

    class myvec
    {
    public:
      int side;
      double hint;
      myvec (int side, double hint)
      {
        this->side = side;
        this->hint = hint;
      }
    };

    NurbsTools (ON_NurbsSurface* surf);

    // evaluations in the parameter domain
    Eigen::Vector3d
    x (double u, double v);

    Eigen::MatrixXd
    jacX (double u, double v);

    std::vector<double>
    getElementVector (int dim);

    std::vector<double>
    getElementVectorDeltas (int dim);

    Eigen::Vector3d
    getDistanceVector (Eigen::Vector3d pt, Eigen::Vector2d params);

    Eigen::Vector2d
    inverseMapping (Eigen::Vector3d pt, Eigen::Vector2d hint, double &error, int maxSteps = 100,
                    double accuracy = 1e-6, bool quiet = true);

    Eigen::Vector2d
    inverseMappingBoundary (Eigen::Vector3d pt, double &error, int maxSteps = 100, double accuracy = 1e-6,
                            bool quiet = true);

    Eigen::Vector2d
    inverseMappingBoundary (Eigen::Vector3d pt, int side, double hint, double &error, int maxSteps = 100,
                            double accuracy = 1e-6, bool quiet = true);

    Eigen::Vector2d
    inverseMapping (Eigen::Vector3d pt, Eigen::Vector2d* phint, double &error, int maxSteps = 100,
                    double accuracy = 1e-6, bool quiet = true);

    // index routines
    int
    A (int I, int J)
    {
      return surf_->m_cv_count[1] * I + J;
    } // two global indices to one global index (lexicographic)

    int
    a (int i, int j)
    {
      return surf_->m_order[1] * i + j;
    } // two local indices into one local index (lexicographic)

    int
    i (int a)
    {
      return (int)(a / surf_->m_order[1]);
    } // local lexicographic in local row index

    int
    j (int a)
    {
      return (int)(a % surf_->m_order[1]);
    } // local lexicographic in local col index

    int
    I (int A)
    {
      return (int)(A / surf_->m_cv_count[1]);
    } // global lexicographic in global row index

    int
    J (int A)
    {
      return (int)(A % surf_->m_cv_count[1]);
    } // global lexicographic in global col index

    int
    A (int E, int F, int i, int j)
    {
      return A (E + i, F + j);
    }
    ; // check this: element + local indices to one global index (lexicographic)

    int
    E (double u)
    {
      return ON_NurbsSpanIndex (surf_->m_order[0], surf_->m_cv_count[0], surf_->m_knot[0], u, 0, 0);
    } // element index in u-direction

    int
    F (double v)
    {
      return ON_NurbsSpanIndex (surf_->m_order[1], surf_->m_cv_count[1], surf_->m_knot[1], v, 0, 0);
    } // element index in v-direction

  };

} // namespace pcl_nurbs

#endif /* NTOOLS_H_ */

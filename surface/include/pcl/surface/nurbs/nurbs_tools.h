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

#ifndef _NURBS_TOOLS_H_
#define _NURBS_TOOLS_H_

#include <vector>

#include <pcl/pcl_macros.h>
#include "nurbs_surface.h"
#include "nurbs_data.h"

#ifdef USE_UMFPACK
#include <suitesparse/cholmod.h>
#include <suitesparse/umfpack.h>
#endif

namespace pcl
{
  namespace nurbs
  {

    enum
    {
      NORTH = 1, NORTHEAST = 2, EAST = 3, SOUTHEAST = 4, SOUTH = 5, SOUTHWEST = 6, WEST = 7, NORTHWEST = 8
    };

    class NurbsTools
    {

    public:
      NurbsSurface* m_surf;

      class myvec
      {
      public:
        int side_;
        double hint_;
        myvec (int side, double hint)
            : side_(side)
            , hint_(hint)
        {}
      };

      NurbsTools (NurbsSurface* surf);

      static void
      initNurbsPCA (NurbsSurface *nurbs, NurbsData *data, Eigen::Vector3d z=Eigen::Vector3d(0.0,0.0,1.0));

      static void
      initNurbsPCABoundingBox (NurbsSurface *nurbs, NurbsData *data, Eigen::Vector3d z);

      static Eigen::Vector3d
      computeMean (const vector_vec3d &data);

      static void
          pca (const vector_vec3d &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors,
               Eigen::Vector3d &eigenvalues);

      static void
      downSampleRandom (vector_vec3d &data, unsigned size);

      // evaluations in the parameter domain
      vec3
      x (double u, double v);
      //  ON_3dPoint evaluatePointOnNurbs(cvec params);
      Eigen::MatrixXd
      jacX (double u, double v);

      std::vector<double>
      getElementVector (int dim);
      std::vector<double>
      getElementVectorDeltas (int dim);

      vec2
      inverseMapping (const vec3 &pt, const vec2 &hint, double &error, vec3 &p, vec3 &tu, vec3 &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);

      vec2
      inverseMapping (const vec3 &pt, vec2* phint, double &error, vec3 &p, vec3 &tu, vec3 &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);

      vec2
      inverseMappingBoundary (const vec3 &pt, double &error, vec3 &p, vec3 &tu, vec3 &tv, int maxSteps = 100,
                              double accuracy = 1e-6, bool quiet = true);

      vec2
      inverseMappingBoundary (const vec3 &pt, int side, double hint, double &error, vec3 &p, vec3 &tu, vec3 &tv,
                              int maxSteps = 100, double accuracy = 1e-6, bool quiet = true);

#ifdef USE_UMFPACK
      bool solveSparseLinearSystem(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x, bool transpose);
      bool solveSparseLinearSystemLQ(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x);
#endif

      // index routines
      std::size_t
      A (std::size_t I, std::size_t J)
      {
        return (m_surf->nbControlPointsV () * I + J);
      }

      // two global indices to one global index (lexicographic)
      std::size_t
      a (std::size_t i, std::size_t j)
      {
        return ((m_surf->degreeV () + 1) * i + j);
      }

      // two local indices into one local index (lexicographic)
      std::size_t
      i (std::size_t a)
      {
        return (std::size_t(a / (m_surf->degreeV () + 1)));
      } // local lexicographic in local row index
      std::size_t
      j (std::size_t a)
      {
        return (a % (m_surf->degreeV () + 1));
      } // local lexicographic in local col index
      std::size_t
      I (std::size_t A)
      {
        return (A / (m_surf->degreeV () + 1));
      } // global lexicographic in global row index
      std::size_t
      J (std::size_t A)
      {
        return (A % (m_surf->degreeV () + 1));
      } // global lexicographic in global col index
      std::size_t
      A (std::size_t E, std::size_t F, std::size_t i, std::size_t j)
      {
        return (A (E + i, F + j));
      }
      // check this: element + local indices to one global index (lexicographic)
      std::size_t
      E (const double &u)
      {
        return (m_surf->basis_u_.getSpan (u) - m_surf->degreeU ());
        //    return ON_NurbsSpanIndex((m_surf->Degree(0)+1), (m_surf->Degree(0)+1), m_surf->m_knot[0], u, 0, 0);
      } // element index in u-direction
      std::size_t
      F (const double &v)
      {
        return (m_surf->basis_u_.getSpan (v) - m_surf->degreeV ());
        //    return ON_NurbsSpanIndex(m_surf->m_order[1], m_surf->m_cv_count[1], m_surf->m_knot[1], v, 0, 0);
      } // element index in v-direction

    };
  } // namespace nurbs
} // namespace pcl

#endif /* NTOOLS_H_ */

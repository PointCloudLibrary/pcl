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

#ifndef NURBS_FITTING_SURFACE_IM_H
#define NURBS_FITTING_SURFACE_IM_H

#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief      */
    class FittingSurfaceIM
    {
    public:

      /** \brief Parameters for fitting */
      struct Parameter
      {
        double smoothness;
        Parameter () :
          smoothness (0.1)
        {
        }
      };

    protected:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
      std::vector<int> m_indices;
      Eigen::Matrix3d m_intrinsic;
      Eigen::Vector4d m_bb;

      ON_NurbsSurface m_nurbs;
      pcl::on_nurbs::vector_vec2i m_cps_px; // control points in pixel space
      NurbsSolve m_solver;
      bool m_quiet;

      pcl::PointXYZRGB
      computeMean () const;

    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      inline ON_NurbsSurface&
      getSurface ()
      {
        return m_nurbs;
      }

    public:
      void
      setInputCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

      void
      setIndices (std::vector<int> &_indices);

      void
      setCamera (const Eigen::Matrix3d &i);

      void
      setCamera (const Eigen::Matrix3f &i);

      static std::vector<double>
      getElementVector (const ON_NurbsSurface &nurbs, int dim);

      void
      refine ();

      /** compute bounding box of cloud in index space */
      static Eigen::Vector4d
      computeIndexBoundingBox (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::vector<int> &indices);
      //      void
      //      compute (int nurbs_order, double damp = 1.0);

    public:
      void
      initSurface (int order, const Eigen::Vector4d &bb);

      void
      assemble (bool inverse_mapping = false);

      void
      solve (double damp = 1.0);

      void
      updateSurf (double damp);

      Eigen::Vector2d
      findClosestElementMidPoint (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt);

      Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                      double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps,
                      double accuracy, bool quiet);

      void
      addPointConstraint (const Eigen::Vector2i &params, double z, double weight, unsigned &row);

      void
      addCageInteriorRegularisation (double weight, unsigned &row);

      void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);

      void
      addCageCornerRegularisation (double weight, unsigned &row);

      // index routines
      int
      grc2gl (int I, int J)
      {
        return m_nurbs.CVCount (1) * I + J;
      } // global row/col index to global lexicographic index
      int
      lrc2gl (int E, int F, int i, int j)
      {
        return grc2gl (E + i, F + j);
      } // local row/col index to global lexicographic index
      int
      gl2gr (int A)
      {
        return (static_cast<int> (A / m_nurbs.CVCount (1)));
      } // global lexicographic in global row index
      int
      gl2gc (int A)
      {
        return (static_cast<int> (A % m_nurbs.CVCount (1)));
      } // global lexicographic in global col index

    };
  }
}

#endif

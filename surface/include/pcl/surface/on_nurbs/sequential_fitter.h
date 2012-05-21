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

#ifndef SEQUENTIAL_FITTER_H
#define SEQUENTIAL_FITTER_H

//#include <opencv2/core/core.hpp>
//#include "v4r/TomGine/tgTomGine.h"
//#include "v4r/NurbsConvertion/NurbsConvertion.h"

// remove multiple #defines from xlib and OpenMesh
#undef True
#undef False
#undef None
#undef Status

#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//#include "v4r/NurbsConvertion/DataLoading.h"

namespace pcl
{
  namespace on_nurbs
  {

    class SequentialFitter
    {
    private:
      //  TomGine::tgTomGineThread* m_dbgWin;
      //  TomGine::tgEngine *m_engine;
      //  bool m_quiet;

    public:
      struct Parameter
      {
        int order;
        int refinement;
        int iterationsQuad;
        int iterationsBoundary;
        int iterationsAdjust;
        int iterationsInterior;
        double forceBoundary;
        double forceBoundaryInside;
        double forceInterior;
        double stiffnessBoundary;
        double stiffnessInterior;
        int resolution;
        Parameter (int order = 3, int refinement = 1, int iterationsQuad = 5, int iterationsBoundary = 5,
                   int iterationsAdjust = 5, int iterationsInterior = 2, double forceBoundary = 200.0,
                   double forceBoundaryInside = 400.0, double forceInterior = 1.0, double stiffnessBoundary = 20.0,
                   double stiffnessInterior = 0.1, int resolution = 16);
      };

      //  void fitNurbsSurface(const cv::Mat_<cv::Vec4f> &matCloud, const cv::Mat_<uchar> &mask, const cv::Mat_<uchar> &contour,
      //      const std::vector<cv::Point2i> &corners, ON_NurbsSurface &nurbs, cv::Mat_<uchar> &nurbs_mask, cv::Mat_<double> &error_img, double &error_interior,
      //      double &error_boundary);

    private:
      Parameter m_params;

      NurbsDataSurface m_data;
      ON_NurbsSurface m_nurbs;

      ON_3dPoint m_corners[4];
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
      pcl::PointIndices::Ptr m_boundary_indices;
      pcl::PointIndices::Ptr m_interior_indices;

      Eigen::Matrix4d m_intrinsic;
      Eigen::Matrix4d m_extrinsic;

      bool m_have_cloud;
      bool m_have_corners;

      int m_surf_id;

      //  void ExtractError(DataSet &data, cv::Mat_<double> &error_img, double &error_interior, double &error_boundary);
      //  void UpdateInterior(const cv::Mat_<uchar> &maskNurbs, DataSet* data);
      //  void AdjustBoundary(const cv::Mat_<cv::Vec4f> &matCloud, const cv::Mat_<uchar> &mask, FittingPatch *patchFit, DataSet* data, std::vector<double> &wBnd,
      //      bool move_contour);

      void
      compute_quadfit ();
      void
      compute_refinement (FittingPatch* fitting);
      void
      compute_boundary (FittingPatch* fitting);
      void
      compute_interior (FittingPatch* fitting);

      Eigen::Vector2d
      project (const Eigen::Vector3d &pt);
      bool
      is_back_facing (const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
                      const Eigen::Vector3d &v3);

    public:
      //  SequentialFitter(Parameter p = Parameter(), bool quiet = true, TomGine::tgTomGineThread* dbgWin=NULL);
      SequentialFitter (Parameter p = Parameter ());

      inline void
      setParams (const Parameter &p)
      {
        m_params = p;
      }

      /** Set input point cloud **/
      void
      setInputCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

      void
      setBoundary (pcl::PointIndices::Ptr &pcl_cloud_indices);

      void
      setInterior (pcl::PointIndices::Ptr &pcl_cloud_indices);

      void
      setCorners (pcl::PointIndices::Ptr &corners, bool flip_on_demand = true);

      void
      setProjectionMatrix (Eigen::Matrix4d &intrinsic, Eigen::Matrix4d &extrinsic);

      /** Compute point cloud and fit (multiple) models **/
      ON_NurbsSurface
      compute (bool assemble = false);

      ON_NurbsSurface
      compute_boundary (const ON_NurbsSurface &nurbs);

      ON_NurbsSurface
      compute_interior (const ON_NurbsSurface &nurbs);

      inline ON_NurbsSurface
      getNurbs ()
      {
        return m_nurbs;
      }

      /** @brief Get error of each interior point (L2-norm of point to closest point on surface) and square-error */
      void
      getInteriorError (std::vector<double> &error);

      /** @brief Get error of each boundary point (L2-norm of point to closest point on surface) and square-error */
      void
      getBoundaryError (std::vector<double> &error);

      /** @brief Get parameter of each interior point */
      void
      getInteriorParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params);

      /** @brief Get parameter of each boundary point */
      void
      getBoundaryParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params);

      /** @brief get the normals to the interior points given by setInterior() */
      void
      getInteriorNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normal);

      /** @brief get the normals to the boundary points given by setBoundary() */
      void
      getBoundaryNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals);

      /** @brief Get the closest point on a NURBS from a point pt in parameter space
       *  @param nurbs  The NURBS surface
       *  @param pt     A point in 3D from which the closest point is calculated
       *  @param params The closest point on the NURBS in parameter space
       *  @param maxSteps Maximum iteration steps
       *  @param accuracy Accuracy below which the iterations stop */
      static void
      getClosestPointOnNurbs (ON_NurbsSurface nurbs, Eigen::Vector3d pt, Eigen::Vector2d& params, int maxSteps = 100,
                              double accuracy = 1e-4);

      ON_NurbsSurface
      grow (float max_dist = 1.0f, float max_angle = M_PI_4, unsigned min_length = 0, unsigned max_length = 10);

      static unsigned
      PCL2ON (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, const std::vector<int> &indices, vector_vec3d &cloud);

    };

  }
}

#endif

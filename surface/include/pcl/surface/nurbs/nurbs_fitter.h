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

#ifndef _NURBS_FITTER_H_
#define _NURBS_FITTER_H_

// remove multiple #defines from xlib and OpenMesh
//#undef True
//#undef False
//#undef None
//#undef Status
#include <pcl/pcl_base.h>
#include "pcl/surface/nurbs/nurbs_fitting.h"

namespace pcl
{

  template<typename PointInT>
    class NurbsFitter : public PCLBase<PointInT>
    {
    public:
      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

//      typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;
      typedef PCLBase<PointInT> PCLBaseT;
      typedef typename PCLBaseT::PointCloudPtr PointCloudPtr;
      //typedef typename pcl::PointCloud<PointInT>::ConstPtr PointCloudConstPtr;

      struct Parameter
      {
        int order;
        int refinement;
        int iterationsQuad;
        int iterationsBoundary;
        int iterationsInterior;
        double forceBoundary;
        double forceBoundaryInside;
        double forceInterior;
        double stiffnessBoundary;
        double stiffnessInterior;
        int resolution;
        Parameter (int order = 3, int refinement = 1, int iterationsQuad = 5, int iterationsBoundary = 5,
                   int iterationsInterior = 2, double forceBoundary = 200.0, double forceBoundaryInside = 400.0,
                   double forceInterior = 1.0, double stiffnessBoundary = 20.0, double stiffnessInterior = 0.1,
                   int resolution = 16);
      };

      //  void fitNurbsSurface(const cv::Mat_<cv::Vec4f> &matCloud, const cv::Mat_<uchar> &mask, const cv::Mat_<uchar> &contour,
      //      const std::vector<cv::Point2i> &corners, ON_NurbsSurface &nurbs, cv::Mat_<uchar> &nurbs_mask, cv::Mat_<double> &error_img, double &error_interior,
      //      double &error_boundary);

    private:
      Parameter m_params;
      bool m_quiet;

      NurbsData m_data;
      ON_NurbsSurface m_nurbs;

      ON_3dPoint m_corners[4];
      //    pcl::PointCloud<PointInT>::Ptr m_cloud;           // input_

      //    pcl::PointIndices::Ptr m_interior_indices;          // indices_
      pcl::PointIndices::Ptr m_boundary_indices;

      Eigen::Matrix4d m_intrinsic;
      Eigen::Matrix4d m_extrinsic;

      bool m_have_cloud;
      bool m_have_corners;

      int m_surf_id;

      //  void ExtractError(DataSet &data, cv::Mat_<double> &error_img, double &error_interior, double &error_boundary);
      //  void UpdateInterior(const cv::Mat_<uchar> &maskNurbs, DataSet* data);
      //  void AdjustBoundary(const cv::Mat_<cv::Vec4f> &matCloud, const cv::Mat_<uchar> &mask, NurbsFitting *patchFit, DataSet* data, std::vector<double> &wBnd,
      //      bool move_contour);

      void
      compute_quadfit ();
      void
      compute_refinement (NurbsFitting* fitting);
      void
      compute_boundary (NurbsFitting* fitting);
      void
      compute_interior (NurbsFitting* fitting);

      Eigen::Vector2d
      project (const Eigen::Vector3d &pt);
      bool
      is_back_facing (const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
                      const Eigen::Vector3d &v3);

    public:
      NurbsFitter (Parameter p = Parameter (), bool quiet = true);

      inline void
      setParams (const Parameter &p)
      {
        m_params = p;
      }

      void
      setBoundary (pcl::PointIndices::Ptr &pcl_cloud_indices);

      void
      setCorners (pcl::PointIndices::Ptr &corners, bool flip_on_demand = true);

      void
      setProjectionMatrix (Eigen::Matrix4d &intrinsic, Eigen::Matrix4d &extrinsic);

      /** Compute point cloud and fit (multiple) models **/
      ON_NurbsSurface
      compute ();

      ON_NurbsSurface
      computeBoundary (const ON_NurbsSurface &nurbs);

      ON_NurbsSurface
      computeInterior (const ON_NurbsSurface &nurbs);

      inline ON_NurbsSurface
      getNurbs ()
      {
        return m_nurbs;
      }

      /** Get error of each interior point (L2-norm of point to closest point on surface) and square-error */
      void
      getInteriorError (std::vector<double> &error);

      /** Get error of each boundary point (L2-norm of point to closest point on surface) and square-error */
      void
      getBoundaryError (std::vector<double> &error);

      ON_NurbsSurface
      grow (float max_dist = 1.0f, float max_angle = M_PI_4, unsigned min_length = 0, unsigned max_length = 10);

    };

}

#endif

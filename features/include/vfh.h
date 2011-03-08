/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: vfh.h 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FEATURES_VFH_H_
#define PCL_FEATURES_VFH_H_

#include <pcl/features/feature.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b VFHEstimation estimates the <b>Viewpoint Feature Histogram (VFH)</b> descriptor for a given point cloud
    * dataset containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> R.B. Rusu, G. Bradski, R. Thibaux, J. Hsu.
    *      Fast 3D Recognition and Pose Using the Viewpoint Feature Histogram.
    *      In Proceedings of International Conference on Intelligent Robots and Systems (IROS)
    *      Taipei, Taiwan, October 18-22 2010.
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class VFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      VFHEstimation () : nr_bins_f1_ (45), nr_bins_f2_ (45), nr_bins_f3_ (45), nr_bins_f4_ (45), nr_bins_vp_ (128), vpx_ (0), vpy_ (0), vpz_ (0), d_pi_ (1.0 / (2.0 * M_PI))
      {
        hist_f1_.setZero (nr_bins_f1_);
        hist_f2_.setZero (nr_bins_f2_);
        hist_f3_.setZero (nr_bins_f3_);
        hist_f4_.setZero (nr_bins_f4_);
        search_radius_ = 0;
        k_ = 1;
        feature_name_ = "VFHEstimation";
      };

      /** \brief Estimate the SPFH (Simple Point Feature Histograms) signatures of the angular
        * (f1, f2, f3) and distance (f4) features for a given point from its neighborhood 
        * \param centroid_p the centroid point
        * \param centroid_n the centroid normal
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param normals the dataset containing the surface normals at each point in \a cloud
        * \param indices the k-neighborhood point indices in the dataset
        */
      void computePointSPFHSignature (const Eigen::Vector4f &centroid_p, const Eigen::Vector4f &centroid_n, const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, const std::vector<int> &indices);

      /** \brief Set the number of subdivisions for each feature interval.
        * \param nr_bins_f1 number of subdivisions for the first angular feature
        * \param nr_bins_f2 number of subdivisions for the second angular feature
        * \param nr_bins_f3 number of subdivisions for the third angular feature
        * \param nr_bins_f4 number of subdivisions for the fourth distance feature
        */
      void
      setNrSubdivisions (int nr_bins_f1, int nr_bins_f2, int nr_bins_f3, int nr_bins_f4)
      {
        nr_bins_f1_ = nr_bins_f1;
        nr_bins_f2_ = nr_bins_f2;
        nr_bins_f3_ = nr_bins_f3;
        nr_bins_f4_ = nr_bins_f4;
      }

      /** \brief Get the number of subdivisions for each feature interval. */
      void
      getNrSubdivisions (int &nr_bins_f1, int &nr_bins_f2, int &nr_bins_f3, int &nr_bins_f4)
      {
        nr_bins_f1 = nr_bins_f1_;
        nr_bins_f2 = nr_bins_f2_;
        nr_bins_f3 = nr_bins_f3_;
        nr_bins_f4 = nr_bins_f4_;
      }

      /** \brief Set the number of subdivisions for the viewpoint feature interval.
        * \param nr_bins number of subdivisions for the viewpoint feature interval.
        */
      void setNrViewpointSubdivisions (int nr_bins) { nr_bins_vp_ = nr_bins; }

      /** \brief Get the number of subdivisions for the viewpoint feature interval. */
      void getNrViewpointSubdivisions (int &nr_bins) { nr_bins = nr_bins_vp_; }

      /** \brief Set the viewpoint.
        * \param vpx the X coordinate of the viewpoint
        * \param vpy the Y coordinate of the viewpoint
        * \param vpz the Z coordinate of the viewpoint
        */
      inline void
      setViewPoint (float vpx, float vpy, float vpz)
      {
        vpx_ = vpx;
        vpy_ = vpy;
        vpz_ = vpz;
      }

      /** \brief Get the viewpoint. */
      inline void
      getViewPoint (float &vpx, float &vpy, float &vpz)
      {
        vpx = vpx_;
        vpy = vpy_;
        vpz = vpz_;
      }

    private:

      /** \brief The number of subdivisions for each feature interval. */
      int nr_bins_f1_, nr_bins_f2_, nr_bins_f3_, nr_bins_f4_, nr_bins_vp_;

      /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
        * from VFHEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
      float vpx_, vpy_, vpz_;

      /** \brief Estimate the Viewpoint Feature Histograms (VFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the VFH feature estimates
        */
      void computeFeature (PointCloudOut &output);

    protected:
      /** \brief Placeholder for the f1 histogram. */
      Eigen::VectorXf hist_f1_;
      /** \brief Placeholder for the f2 histogram. */
      Eigen::VectorXf hist_f2_;
      /** \brief Placeholder for the f3 histogram. */
      Eigen::VectorXf hist_f3_;
      /** \brief Placeholder for the f4 histogram. */
      Eigen::VectorXf hist_f4_;
      /** \brief Placeholder for the vp histogram. */
      Eigen::VectorXf hist_vp_;

    private:
      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_; 
  };
}

#endif  //#ifndef PCL_FEATURES_VFH_H_

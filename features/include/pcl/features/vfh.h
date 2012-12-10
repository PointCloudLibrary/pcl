/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_VFH_H_
#define PCL_FEATURES_VFH_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief VFHEstimation estimates the <b>Viewpoint Feature Histogram (VFH)</b> descriptor for a given point cloud
    * dataset containing points and normals. The default VFH implementation uses 45 binning subdivisions for each of
    * the three extended FPFH values, plus another 45 binning subdivisions for the distances between each point and
    * the centroid and 128 binning subdivisions for the viewpoint component, which results in a
    * 308-byte array of float values. These are stored in a pcl::VFHSignature308 point type.
    * A major difference between the PFH/FPFH descriptors and VFH, is that for a given point cloud dataset, only a
    * single VFH descriptor will be estimated (vfhs->points.size() should be 1), while the resultant PFH/FPFH data
    * will have the same number of entries as the number of points in the cloud.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - R.B. Rusu, G. Bradski, R. Thibaux, J. Hsu.
    *     Fast 3D Recognition and Pose Using the Viewpoint Feature Histogram.
    *     In Proceedings of International Conference on Intelligent Robots and Systems (IROS)
    *     Taipei, Taiwan, October 18-22 2010.
    *
    * \note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for an example of a parallel implementation of the FPFH (Fast Point Feature Histogram).
    * \author Radu B. Rusu
    * \ingroup features
    */
  template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
  class VFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename boost::shared_ptr<VFHEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef typename boost::shared_ptr<const VFHEstimation<PointInT, PointNT, PointOutT> > ConstPtr;


      /** \brief Empty constructor. */
      VFHEstimation () :
        nr_bins_f1_ (45), nr_bins_f2_ (45), nr_bins_f3_ (45), nr_bins_f4_ (45), nr_bins_vp_ (128),
        vpx_ (0), vpy_ (0), vpz_ (0),
        hist_f1_ (), hist_f2_ (), hist_f3_ (), hist_f4_ (), hist_vp_ (),
        normal_to_use_ (), centroid_to_use_ (), use_given_normal_ (false), use_given_centroid_ (false),
        normalize_bins_ (true), normalize_distances_ (false), size_component_ (false),
        d_pi_ (1.0f / (2.0f * static_cast<float> (M_PI)))
      {
        hist_f1_.setZero (nr_bins_f1_);
        hist_f2_.setZero (nr_bins_f2_);
        hist_f3_.setZero (nr_bins_f3_);
        hist_f4_.setZero (nr_bins_f4_);
        search_radius_ = 0;
        k_ = 0;
        feature_name_ = "VFHEstimation";
      }

      /** \brief Estimate the SPFH (Simple Point Feature Histograms) signatures of the angular
        * (f1, f2, f3) and distance (f4) features for a given point from its neighborhood
        * \param[in] centroid_p the centroid point
        * \param[in] centroid_n the centroid normal
        * \param[in] cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param[in] normals the dataset containing the surface normals at each point in \a cloud
        * \param[in] indices the k-neighborhood point indices in the dataset
        */
      void
      computePointSPFHSignature (const Eigen::Vector4f &centroid_p, const Eigen::Vector4f &centroid_n,
                                 const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
                                 const std::vector<int> &indices);

      /** \brief Set the viewpoint.
        * \param[in] vpx the X coordinate of the viewpoint
        * \param[in] vpy the Y coordinate of the viewpoint
        * \param[in] vpz the Z coordinate of the viewpoint
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

      /** \brief Set use_given_normal_
        * \param[in] use Set to true if you want to use the normal passed to setNormalUse(normal)
        */
      inline void
      setUseGivenNormal (bool use)
      {
        use_given_normal_ = use;
      }

      /** \brief Set the normal to use
        * \param[in] normal Sets the normal to be used in the VFH computation. It is is used
        * to build the Darboux Coordinate system.
        */
      inline void
      setNormalToUse (const Eigen::Vector3f &normal)
      {
        normal_to_use_ = Eigen::Vector4f (normal[0], normal[1], normal[2], 0);
      }

      /** \brief Set use_given_centroid_
        * \param[in] use Set to true if you want to use the centroid passed through setCentroidToUse(centroid)
        */
      inline void
      setUseGivenCentroid (bool use)
      {
        use_given_centroid_ = use;
      }

      /** \brief Set centroid_to_use_
        * \param[in] centroid Centroid to be used in the VFH computation. It is used to compute the distances
        * from all points to this centroid.
        */
      inline void
      setCentroidToUse (const Eigen::Vector3f &centroid)
      {
        centroid_to_use_ = Eigen::Vector4f (centroid[0], centroid[1], centroid[2], 0);
      }

      /** \brief set normalize_bins_
        * \param[in] normalize If true, the VFH bins are normalized using the total number of points
        */
      inline void
      setNormalizeBins (bool normalize)
      {
        normalize_bins_ = normalize;
      }

      /** \brief set normalize_distances_
        * \param[in] normalize If true, the 4th component of VFH (shape distribution component) get normalized
        * by the maximum size between the centroid and the point cloud
        */
      inline void
      setNormalizeDistance (bool normalize)
      {
        normalize_distances_ = normalize;
      }

      /** \brief set size_component_
        * \param[in] fill_size True if the 4th component of VFH (shape distribution component) needs to be filled.
        * Otherwise, it is set to zero.
        */
      inline void
      setFillSizeComponent (bool fill_size)
      {
        size_component_ = fill_size;
      }

      /** \brief Overloaded computed method from pcl::Feature.
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      compute (PointCloudOut &output);

    private:

      /** \brief The number of subdivisions for each feature interval. */
      int nr_bins_f1_, nr_bins_f2_, nr_bins_f3_, nr_bins_f4_, nr_bins_vp_;

      /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
        * from VFHEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0.
        */
      float vpx_, vpy_, vpz_;

      /** \brief Estimate the Viewpoint Feature Histograms (VFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the VFH feature estimates
        */
      void
      computeFeature (PointCloudOut &output);

    protected:
      /** \brief This method should get called before starting the actual computation. */
      bool
      initCompute ();

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

      /** \brief Normal to be used to computed VFH. Default, the average normal of the whole point cloud */
      Eigen::Vector4f normal_to_use_;
      /** \brief Centroid to be used to computed VFH. Default, the centroid of the whole point cloud */
      Eigen::Vector4f centroid_to_use_;

      // VFH configuration parameters because CVFH instantiates it. See constructor for default values.

      /** \brief Use the normal_to_use_ */
      bool use_given_normal_;
      /** \brief Use the centroid_to_use_ */
      bool use_given_centroid_;
      /** \brief Normalize bins by the number the total number of points. */
      bool normalize_bins_;
      /** \brief Normalize the shape distribution component of VFH */
      bool normalize_distances_;
      /** \brief Activate or deactivate the size component of VFH */
      bool size_component_;

    private:
      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/vfh.hpp>
#endif

#endif  //#ifndef PCL_FEATURES_VFH_H_

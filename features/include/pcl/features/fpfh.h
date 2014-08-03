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

#ifndef PCL_FPFH_H_
#define PCL_FPFH_H_

#include <pcl/features/feature.h>
#include <set>

namespace pcl
{
  /** \brief FPFHEstimation estimates the <b>Fast Point Feature Histogram (FPFH)</b> descriptor for a given point 
    * cloud dataset containing points and normals.
    *
    * A commonly used type for PointOutT is pcl::FPFHSignature33.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - R.B. Rusu, N. Blodow, M. Beetz.
    *     Fast Point Feature Histograms (FPFH) for 3D Registration.
    *     In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA),
    *     Kobe, Japan, May 12-17 2009.
    *   - R.B. Rusu, A. Holzbach, N. Blodow, M. Beetz.
    *     Fast Geometric Point Labeling using Conditional Random Fields.
    *     In Proceedings of the 22nd IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *     St. Louis, MO, USA, October 11-15 2009.
    *
    * \attention 
    * The convention for FPFH features is:
    *   - if a query point's nearest neighbors cannot be estimated, the FPFH feature will be set to NaN 
    *     (not a number)
    *   - it is impossible to estimate a FPFH descriptor for a point that
    *     doesn't have finite 3D coordinates. Therefore, any point that contains
    *     NaN data on x, y, or z, will have its FPFH feature property set to NaN.
    *
    * \note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::FPFHSignature33>
  class FPFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<FPFHEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const FPFHEstimation<PointInT, PointNT, PointOutT> > ConstPtr;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      FPFHEstimation () : 
        nr_bins_f1_ (11), nr_bins_f2_ (11), nr_bins_f3_ (11), 
        hist_f1_ (), hist_f2_ (), hist_f3_ (), fpfh_histogram_ (),
        d_pi_ (1.0f / (2.0f * static_cast<float> (M_PI)))
      {
        feature_name_ = "FPFHEstimation";
      };

      /** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
        * represented by Cartesian coordinates and normals.
        * \note For explanations about the features, please see the literature mentioned above (the order of the
        * features might be different).
        * \param[in] cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param[in] normals the dataset containing the surface normals (assuming normalized vectors) at each point in cloud
        * \param[in] p_idx the index of the first point (source)
        * \param[in] q_idx the index of the second point (target)
        * \param[out] f1 the first angular feature (angle between the projection of nq_idx and u)
        * \param[out] f2 the second angular feature (angle between nq_idx and v)
        * \param[out] f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)
        * \param[out] f4 the distance feature (p_idx - q_idx)
        */
      bool 
      computePairFeatures (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, 
                           int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4);

      /** \brief Estimate the SPFH (Simple Point Feature Histograms) individual signatures of the three angular
        * (f1, f2, f3) features for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param[in] normals the dataset containing the surface normals at each point in \a cloud
        * \param[in] p_idx the index of the query point (source)
        * \param[in] row the index row in feature histogramms
        * \param[in] indices the k-neighborhood point indices in the dataset
        * \param[out] hist_f1 the resultant SPFH histogram for feature f1
        * \param[out] hist_f2 the resultant SPFH histogram for feature f2
        * \param[out] hist_f3 the resultant SPFH histogram for feature f3
        */
      void 
      computePointSPFHSignature (const pcl::PointCloud<PointInT> &cloud, 
                                 const pcl::PointCloud<PointNT> &normals, int p_idx, int row, 
                                 const std::vector<int> &indices, 
                                 Eigen::MatrixXf &hist_f1, Eigen::MatrixXf &hist_f2, Eigen::MatrixXf &hist_f3);

      /** \brief Weight the SPFH (Simple Point Feature Histograms) individual histograms to create the final FPFH
        * (Fast Point Feature Histogram) for a given point based on its 3D spatial neighborhood
        * \param[in] hist_f1 the histogram feature vector of \a f1 values over the given patch
        * \param[in] hist_f2 the histogram feature vector of \a f2 values over the given patch
        * \param[in] hist_f3 the histogram feature vector of \a f3 values over the given patch
        * \param[in] indices the point indices of p_idx's k-neighborhood in the point cloud
        * \param[in] dists the distances from p_idx to all its k-neighbors
        * \param[out] fpfh_histogram the resultant FPFH histogram representing the feature at the query point
        */
      void 
      weightPointSPFHSignature (const Eigen::MatrixXf &hist_f1, 
                                const Eigen::MatrixXf &hist_f2, 
                                const Eigen::MatrixXf &hist_f3, 
                                const std::vector<int> &indices, 
                                const std::vector<float> &dists, 
                                Eigen::VectorXf &fpfh_histogram);

      /** \brief Set the number of subdivisions for each angular feature interval.
        * \param[in] nr_bins_f1 number of subdivisions for the first angular feature
        * \param[in] nr_bins_f2 number of subdivisions for the second angular feature
        * \param[in] nr_bins_f3 number of subdivisions for the third angular feature
        */
      inline void
      setNrSubdivisions (int nr_bins_f1, int nr_bins_f2, int nr_bins_f3)
      {
        nr_bins_f1_ = nr_bins_f1;
        nr_bins_f2_ = nr_bins_f2;
        nr_bins_f3_ = nr_bins_f3;
      }

      /** \brief Get the number of subdivisions for each angular feature interval. 
        * \param[out] nr_bins_f1 number of subdivisions for the first angular feature
        * \param[out] nr_bins_f2 number of subdivisions for the second angular feature
        * \param[out] nr_bins_f3 number of subdivisions for the third angular feature
         */
      inline void
      getNrSubdivisions (int &nr_bins_f1, int &nr_bins_f2, int &nr_bins_f3)
      {
        nr_bins_f1 = nr_bins_f1_;
        nr_bins_f2 = nr_bins_f2_;
        nr_bins_f3 = nr_bins_f3_;
      }

    protected:

      /** \brief Estimate the set of all SPFH (Simple Point Feature Histograms) signatures for the input cloud
        * \param[out] spf_hist_lookup a lookup table for all the SPF feature indices
        * \param[out] hist_f1 the resultant SPFH histogram for feature f1
        * \param[out] hist_f2 the resultant SPFH histogram for feature f2
        * \param[out] hist_f3 the resultant SPFH histogram for feature f3
        */
      void 
      computeSPFHSignatures (std::vector<int> &spf_hist_lookup, 
                             Eigen::MatrixXf &hist_f1, Eigen::MatrixXf &hist_f2, Eigen::MatrixXf &hist_f3);

      /** \brief Estimate the Fast Point Feature Histograms (FPFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the FPFH feature estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_bins_f1_, nr_bins_f2_, nr_bins_f3_;

      /** \brief Placeholder for the f1 histogram. */
      Eigen::MatrixXf hist_f1_;

      /** \brief Placeholder for the f2 histogram. */
      Eigen::MatrixXf hist_f2_;

      /** \brief Placeholder for the f3 histogram. */
      Eigen::MatrixXf hist_f3_;

      /** \brief Placeholder for a point's FPFH signature. */
      Eigen::VectorXf fpfh_histogram_;

      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_; 
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/fpfh.hpp>
#endif

#endif  //#ifndef PCL_FPFH_H_

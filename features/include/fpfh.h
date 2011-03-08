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
 * $Id: fpfh.h 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FPFH_H_
#define PCL_FPFH_H_

#include <pcl/features/feature.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b FPFHEstimation estimates the <b>Fast Point Feature Histogram (FPFH)</b> descriptor for a given point cloud
    * dataset containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> R.B. Rusu, N. Blodow, M. Beetz.
    *      Fast Point Feature Histograms (FPFH) for 3D Registration.
    *      In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA),
    *      Kobe, Japan, May 12-17 2009.
    * </li>
    * <li> R.B. Rusu, A. Holzbach, N. Blodow, M. Beetz.
    *      Fast Geometric Point Labeling using Conditional Random Fields.
    *      In Proceedings of the 22nd IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *      St. Louis, MO, USA, October 11-15 2009.
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class FPFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      FPFHEstimation () : nr_bins_f1_ (11), nr_bins_f2_ (11), nr_bins_f3_ (11), d_pi_ (1.0 / (2.0 * M_PI))
      {
        feature_name_ = "FPFHEstimation";
      };

      /** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
        * represented by Cartesian coordinates and normals.
        * \note For explanations about the features, please see the literature mentioned above (the order of the
        * features might be different).
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param normals the dataset containing the surface normals (assuming normalized vectors) at each point in cloud
        * \param p_idx the index of the first point (source)
        * \param q_idx the index of the second point (target)
        * \param f1 the first angular feature (angle between the projection of nq_idx and u)
        * \param f2 the second angular feature (angle between nq_idx and v)
        * \param f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)
        * \param f4 the distance feature (p_idx - q_idx)
        */
      bool computePairFeatures (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, 
                                int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4);

      /** \brief Estimate the SPFH (Simple Point Feature Histograms) individual signatures of the three angular
        * (f1, f2, f3) features for a given point based on its spatial neighborhood of 3D points with normals
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param normals the dataset containing the surface normals at each point in \a cloud
        * \param p_idx the index of the query point (source)
        * \param indices the k-neighborhood point indices in the dataset
        * \param hist_f1 the resultant SPFH histogram for feature f1
        * \param hist_f2 the resultant SPFH histogram for feature f2
        * \param hist_f3 the resultant SPFH histogram for feature f3
        */
      void computePointSPFHSignature (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, int p_idx, const std::vector<int> &indices, Eigen::MatrixXf &hist_f1, Eigen::MatrixXf &hist_f2, Eigen::MatrixXf &hist_f3);

      /** \brief Weight the SPFH (Simple Point Feature Histograms) individual histograms to create the final FPFH
        * (Fast Point Feature Histogram) for a given point based on its 3D spatial neighborhood
        * \param hist_f1 the histogram feature vector of \a f1 values over the given patch
        * \param hist_f2 the histogram feature vector of \a f2 values over the given patch
        * \param hist_f3 the histogram feature vector of \a f3 values over the given patch
        * \param indices the point indices of p_idx's k-neighborhood in the point cloud
        * \param dists the distances from p_idx to all its k-neighbors
        * \param fpfh_histogram the resultant FPFH histogram representing the feature at the query point
        */
      void weightPointSPFHSignature (const Eigen::MatrixXf &hist_f1, const Eigen::MatrixXf &hist_f2, const Eigen::MatrixXf &hist_f3, const std::vector<int> &indices, const std::vector<float> &dists, Eigen::VectorXf &fpfh_histogram);

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param indices a pointer to the vector of indices that represents the input data.
        */
      inline void
      setIndices (const IndicesConstPtr &indices)
      {
        indices_ = indices;
        size_t data_size = indices_->size ();
        hist_f1_.setZero (data_size, nr_bins_f1_);
        hist_f2_.setZero (data_size, nr_bins_f2_);
        hist_f3_.setZero (data_size, nr_bins_f3_);
      }

      /** \brief Set the number of subdivisions for each angular feature interval.
        * \param nr_bins_f1 number of subdivisions for the first angular feature
        * \param nr_bins_f2 number of subdivisions for the second angular feature
        * \param nr_bins_f3 number of subdivisions for the third angular feature
        */
      inline void
      setNrSubdivisions (int nr_bins_f1, int nr_bins_f2, int nr_bins_f3)
      {
        nr_bins_f1_ = nr_bins_f1;
        nr_bins_f2_ = nr_bins_f2;
        nr_bins_f3_ = nr_bins_f3;
      }

      /** \brief Get the number of subdivisions for each angular feature interval. */
      inline void
      getNrSubdivisions (int &nr_bins_f1, int &nr_bins_f2, int &nr_bins_f3)
      {
        nr_bins_f1 = nr_bins_f1_;
        nr_bins_f2 = nr_bins_f2_;
        nr_bins_f3 = nr_bins_f3_;
      }

    protected:

      /** \brief Estimate the Fast Point Feature Histograms (FPFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the FPFH feature estimates
        */
      void computeFeature (PointCloudOut &output);

      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_bins_f1_, nr_bins_f2_, nr_bins_f3_;

      /** \brief Placeholder for the f1 histogram. */
      Eigen::MatrixXf hist_f1_;

      /** \brief Placeholder for the f2 histogram. */
      Eigen::MatrixXf hist_f2_;

      /** \brief Placeholder for the f3 histogram. */
      Eigen::MatrixXf hist_f3_;

    private:
      /** \brief Placeholder for a point's FPFH signature. */
      Eigen::VectorXf fpfh_histogram_;

      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_; 
  };
}

#endif  //#ifndef PCL_FPFH_H_



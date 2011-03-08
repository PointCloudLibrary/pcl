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
 * $Id: vfh.hpp 36231 2011-02-25 03:01:45Z aaldoma $
 *
 */

#ifndef PCL_FEATURES_IMPL_VFH_H_
#define PCL_FEATURES_IMPL_VFH_H_

#include "pcl/features/vfh.h"
#include "pcl/features/pfh.h"
#include <pcl/common/common.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::VFHEstimation<PointInT, PointNT, PointOutT>::computePointSPFHSignature (const Eigen::Vector4f &centroid_p,
                                                                             const Eigen::Vector4f &centroid_n,
                                                                             const pcl::PointCloud<PointInT> &cloud,
                                                                             const pcl::PointCloud<PointNT> &normals,
                                                                             const std::vector<int> &indices)
{
  Eigen::Vector4f pfh_tuple;
  // Reset the whole thing
  hist_f1_.setZero (nr_bins_f1_);
  hist_f2_.setZero (nr_bins_f2_);
  hist_f3_.setZero (nr_bins_f3_);
  hist_f4_.setZero (nr_bins_f4_);

  // Get the bounding box of the current cluster
  //Eigen::Vector4f min_pt, max_pt;
  //pcl::getMinMax3D (cloud, indices, min_pt, max_pt);
  //double distance_normalization_factor = (std::max)((centroid_p - min_pt).norm (), (centroid_p - max_pt).norm ());

  //Instead of using the bounding box to normalize the VFH distance component, it is better to use the max_distance
  //from any point to centroid. VFH is invariant to rotation about the roll axis but the bounding box is not,
  //resulting in different normalization factors for point clouds that are just rotated about that axis.

  /*Eigen::Vector4f max_pt;
  pcl::getMaxDistance (cloud, indices, centroid_p, max_pt);
  double distance_normalization_factor = (centroid_p - max_pt).norm ();*/

  // Factorization constant
  float hist_incr = 100.0 / (float)(indices.size () - 1);

  // Iterate over all the points in the neighborhood
  for (size_t idx = 0; idx < indices.size (); ++idx)
  {
    // Compute the pair P to NNi
    if (!computePairFeatures (centroid_p, centroid_n, cloud.points[indices[idx]].getVector4fMap (),
                              normals.points[indices[idx]].getNormalVector4fMap (), pfh_tuple[0], pfh_tuple[1],
                              pfh_tuple[2], pfh_tuple[3]))
      continue;

    // Normalize the f1, f2, f3, f4 features and push them in the histogram
    int h_index = floor (nr_bins_f1_ * ((pfh_tuple[0] + M_PI) * d_pi_));
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f1_)
      h_index = nr_bins_f1_ - 1;
    hist_f1_ (h_index) += hist_incr;

    h_index = floor (nr_bins_f2_ * ((pfh_tuple[1] + 1.0) * 0.5));
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f2_)
      h_index = nr_bins_f2_ - 1;
    hist_f2_ (h_index) += hist_incr;

    h_index = floor (nr_bins_f3_ * ((pfh_tuple[2] + 1.0) * 0.5));
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f3_)
      h_index = nr_bins_f3_ - 1;
    hist_f3_ (h_index) += hist_incr;

    //h_index = floor (nr_bins_f4_ * (pfh_tuple[3] / distance_normalization_factor));
    h_index = round (pfh_tuple[3] * 100);
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f4_)
      h_index = nr_bins_f4_ - 1;
    hist_f4_ (h_index) += hist_incr;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::VFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    ROS_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    ROS_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // ---[ Step 1a : compute the centroid in XYZ space
  Eigen::Vector4f xyz_centroid;
  compute3DCentroid (*surface_, *indices_, xyz_centroid);          // Estimate the XYZ centroid

  // ---[ Step 1b : compute the centroid in normal space
  Eigen::Vector4f normal_centroid = Eigen::Vector4f::Zero ();
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (normals_->is_dense)
  {
    for (size_t i = 0; i < indices_->size (); ++i)
    {
      normal_centroid += normals_->points[(*indices_)[i]].getNormalVector4fMap ();
      cp++;
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices_->size (); ++i)
    {
      if (!pcl_isfinite (normals_->points[(*indices_)[i]].normal[0]) || 
          !pcl_isfinite (normals_->points[(*indices_)[i]].normal[1]) || 
          !pcl_isfinite (normals_->points[(*indices_)[i]].normal[2]))
        continue;
      normal_centroid += normals_->points[(*indices_)[i]].getNormalVector4fMap ();
      cp++;
    }
  }
  normal_centroid /= cp;

  // Compute the direction of view from the viewpoint to the centroid
  Eigen::Vector4f viewpoint (vpx_, vpy_, vpz_, 0);
  Eigen::Vector4f d_vp_p = viewpoint - xyz_centroid;
  d_vp_p.normalize ();

  // Estimate the SPFH at nn_indices[0] using the entire cloud
  computePointSPFHSignature (xyz_centroid, normal_centroid, *surface_, *normals_, *indices_);

  // We only output _1_ signature
  output.points.resize (1);
  output.width = 1;
  output.height = 1;

  // Estimate the FPFH at nn_indices[0] using the entire cloud and copy the resultant signature
  for (int d = 0; d < hist_f1_.size (); ++d)
    output.points[0].histogram[d + 0] = hist_f1_[d];

  size_t data_size = hist_f1_.size ();
  for (int d = 0; d < hist_f2_.size (); ++d)
    output.points[0].histogram[d + data_size] = hist_f2_[d];

  data_size += hist_f2_.size ();
  for (int d = 0; d < hist_f3_.size (); ++d)
    output.points[0].histogram[d + data_size] = hist_f3_[d];

  data_size += hist_f3_.size ();
  for (int d = 0; d < hist_f4_.size (); ++d)
    output.points[0].histogram[d + data_size] = hist_f4_[d];

  // ---[ Step 2 : obtain the viewpoint component
  hist_vp_.setZero (nr_bins_vp_);
  double hist_incr = 100.0 / (double)(indices_->size ());
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f normal (normals_->points[(*indices_)[i]].normal[0],
                            normals_->points[(*indices_)[i]].normal[1],
                            normals_->points[(*indices_)[i]].normal[2], 0);
    // Normalize
    double alpha = (normal.dot (d_vp_p) + 1.0) * 0.5;
    int fi = floor (alpha * hist_vp_.size ());
    if (fi < 0)
      fi = 0;
    if (fi > ((int)hist_vp_.size () - 1))
      fi = hist_vp_.size () - 1;
    // Bin into the histogram
    hist_vp_ [fi] += hist_incr;
  }
  data_size += hist_f4_.size ();
  // Copy the resultant signature
  for (int d = 0; d < hist_vp_.size (); ++d)
    output.points[0].histogram[d + data_size] = hist_vp_[d];
}

#define PCL_INSTANTIATE_VFHEstimation(T,NT,OutT) template class pcl::VFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_VFH_H_ 

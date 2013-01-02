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

#ifndef PCL_FEATURES_IMPL_VFH_H_
#define PCL_FEATURES_IMPL_VFH_H_

#include <pcl/features/vfh.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> bool
pcl::VFHEstimation<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (input_->points.size () < 2 || (surface_ && surface_->points.size () < 2))
  {
    PCL_ERROR ("[pcl::VFHEstimation::initCompute] Input dataset must have at least 2 points!\n");
    return (false);
  }
  if (search_radius_ == 0 && k_ == 0)
    k_ = 1;
  return (Feature<PointInT, PointOutT>::initCompute ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::VFHEstimation<PointInT, PointNT, PointOutT>::compute (PointCloudOut &output)
{
  if (!initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc 
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.is_dense = input_->is_dense;
  output.points.resize (1);

  // Perform the actual feature computation
  computeFeature (output);

  Feature<PointInT, PointOutT>::deinitCompute ();
}

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

  double distance_normalization_factor = 1.0;
  if (normalize_distances_) 
  {
    Eigen::Vector4f max_pt;
    pcl::getMaxDistance (cloud, indices, centroid_p, max_pt);
    max_pt[3] = 0;
    distance_normalization_factor = (centroid_p - max_pt).norm ();
  }

  // Factorization constant
  float hist_incr;
  if (normalize_bins_)
    hist_incr = 100.0f / static_cast<float> (indices.size () - 1);
  else
    hist_incr = 1.0f;

  float hist_incr_size_component;
  if (size_component_)
    hist_incr_size_component = hist_incr;
  else
    hist_incr_size_component = 0.0;

  // Iterate over all the points in the neighborhood
  for (size_t idx = 0; idx < indices.size (); ++idx)
  {
    // Compute the pair P to NNi
    if (!computePairFeatures (centroid_p, centroid_n, cloud.points[indices[idx]].getVector4fMap (),
                              normals.points[indices[idx]].getNormalVector4fMap (), pfh_tuple[0], pfh_tuple[1],
                              pfh_tuple[2], pfh_tuple[3]))
      continue;

    // Normalize the f1, f2, f3, f4 features and push them in the histogram
    int h_index = static_cast<int> (floor (nr_bins_f1_ * ((pfh_tuple[0] + M_PI) * d_pi_)));
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f1_)
      h_index = nr_bins_f1_ - 1;
    hist_f1_ (h_index) += hist_incr;

    h_index = static_cast<int> (floor (nr_bins_f2_ * ((pfh_tuple[1] + 1.0) * 0.5)));
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f2_)
      h_index = nr_bins_f2_ - 1;
    hist_f2_ (h_index) += hist_incr;

    h_index = static_cast<int> (floor (nr_bins_f3_ * ((pfh_tuple[2] + 1.0) * 0.5)));
    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f3_)
      h_index = nr_bins_f3_ - 1;
    hist_f3_ (h_index) += hist_incr;

    if (normalize_distances_)
      h_index = static_cast<int> (floor (nr_bins_f4_ * (pfh_tuple[3] / distance_normalization_factor)));
    else
      h_index = static_cast<int> (pcl_round (pfh_tuple[3] * 100));

    if (h_index < 0)
      h_index = 0;
    if (h_index >= nr_bins_f4_)
      h_index = nr_bins_f4_ - 1;

    hist_f4_ (h_index) += hist_incr_size_component;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::VFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // ---[ Step 1a : compute the centroid in XYZ space
  Eigen::Vector4f xyz_centroid;

  if (use_given_centroid_) 
    xyz_centroid = centroid_to_use_;
  else
    compute3DCentroid (*surface_, *indices_, xyz_centroid);          // Estimate the XYZ centroid

  // ---[ Step 1b : compute the centroid in normal space
  Eigen::Vector4f normal_centroid = Eigen::Vector4f::Zero ();
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (use_given_normal_)
    normal_centroid = normal_to_use_;
  else
  {
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
        if (!pcl_isfinite (normals_->points[(*indices_)[i]].normal[0])
            ||
            !pcl_isfinite (normals_->points[(*indices_)[i]].normal[1])
            ||
            !pcl_isfinite (normals_->points[(*indices_)[i]].normal[2]))
          continue;
        normal_centroid += normals_->points[(*indices_)[i]].getNormalVector4fMap ();
        cp++;
      }
    }
    normal_centroid /= static_cast<float> (cp);
  }

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

  double hist_incr;
  if (normalize_bins_)
    hist_incr = 100.0 / static_cast<double> (indices_->size ());
  else
    hist_incr = 1.0;

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f normal (normals_->points[(*indices_)[i]].normal[0],
                            normals_->points[(*indices_)[i]].normal[1],
                            normals_->points[(*indices_)[i]].normal[2], 0);
    // Normalize
    double alpha = (normal.dot (d_vp_p) + 1.0) * 0.5;
    int fi = static_cast<int> (floor (alpha * static_cast<double> (hist_vp_.size ())));
    if (fi < 0)
      fi = 0;
    if (fi > (static_cast<int> (hist_vp_.size ()) - 1))
      fi = static_cast<int> (hist_vp_.size ()) - 1;
    // Bin into the histogram
    hist_vp_ [fi] += static_cast<float> (hist_incr);
  }
  data_size += hist_f4_.size ();
  // Copy the resultant signature
  for (int d = 0; d < hist_vp_.size (); ++d)
    output.points[0].histogram[d + data_size] = hist_vp_[d];
}

#define PCL_INSTANTIATE_VFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::VFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_VFH_H_

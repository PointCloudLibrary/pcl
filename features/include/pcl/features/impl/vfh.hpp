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
  float hist_incr = 1;
  if (normalize_bins_)
    hist_incr = 100.0f / static_cast<float> (indices.size () - 1);

  float hist_incr_size_component = 0;;
  if (size_component_)
    hist_incr_size_component = hist_incr;

  // Iterate over all the points in the neighborhood
  for (const int &index : indices)
  {
    // Compute the pair P to NNi
    if (!computePairFeatures (centroid_p, centroid_n, cloud.points[index].getVector4fMap (),
                              normals.points[index].getNormalVector4fMap (), pfh_tuple[0], pfh_tuple[1],
                              pfh_tuple[2], pfh_tuple[3]))
      continue;

    // Normalize the f1, f2, f3, f4 features and push them in the histogram
    {
      int h1_index = static_cast<int> (std::floor (nr_bins_f1_ * ((pfh_tuple[0] + M_PI) * d_pi_)));
      h1_index = std::max(h1_index, 0);
      h1_index = std::min(h1_index, nr_bins_f1_ - 1);
      hist_f1_ (h1_index) += hist_incr;
    }

    {
      int h2_index = static_cast<int> (std::floor (nr_bins_f2_ * ((pfh_tuple[1] + 1.0) * 0.5)));
      h2_index = std::max(h2_index, 0);
      h2_index = std::min(h2_index, nr_bins_f2_ - 1);
      hist_f2_ (h2_index) += hist_incr;
    }

    {
      int h3_index = static_cast<int> (std::floor (nr_bins_f3_ * ((pfh_tuple[2] + 1.0) * 0.5)));
      h3_index = std::max(h3_index, 0);
      h3_index = std::min(h3_index, nr_bins_f3_ - 1);
      hist_f3_ (h3_index) += hist_incr;
    }

    if (hist_incr_size_component)
    {
      int h4_index;
      if (normalize_distances_)
        h4_index = static_cast<int> (std::floor (nr_bins_f4_ * (pfh_tuple[3] / distance_normalization_factor)));
      else
        h4_index = static_cast<int> (pcl_round (pfh_tuple[3] * 100));

      h4_index = std::max(h4_index, 0);
      h4_index = std::min(h4_index, nr_bins_f4_ - 1);
      hist_f4_ (h4_index) += hist_incr_size_component;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::VFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // ---[ Step 1a : compute the centroid in XYZ space
  Eigen::Vector4f xyz_centroid (0, 0, 0, 0);

  if (use_given_centroid_) 
    xyz_centroid = centroid_to_use_;
  else
    compute3DCentroid (*surface_, *indices_, xyz_centroid);          // Estimate the XYZ centroid

  // ---[ Step 1b : compute the centroid in normal space
  Eigen::Vector4f normal_centroid = Eigen::Vector4f::Zero ();
  std::size_t cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (use_given_normal_)
    normal_centroid = normal_to_use_;
  else
  {
    if (normals_->is_dense)
    {
      for (const auto& index: *indices_)
      {
        normal_centroid.noalias () += normals_->points[index].getNormalVector4fMap ();
      }
      cp = indices_->size();
    }
    // NaN or Inf values could exist => check for them
    else
    {
      for (const auto& index: *indices_)
      {
        if (!std::isfinite (normals_->points[index].normal[0]) ||
            !std::isfinite (normals_->points[index].normal[1]) ||
            !std::isfinite (normals_->points[index].normal[2]))
          continue;
        normal_centroid.noalias () += normals_->points[index].getNormalVector4fMap ();
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

  // ---[ Step 2 : obtain the viewpoint component
  hist_vp_.setZero (nr_bins_vp_);

  float hist_incr = 1.0;
  if (normalize_bins_)
    hist_incr = 100.0 / static_cast<double> (indices_->size ());

  for (const auto& index: *indices_)
  {
    Eigen::Vector4f normal (normals_->points[index].normal[0],
                            normals_->points[index].normal[1],
                            normals_->points[index].normal[2], 0);
    // Normalize
    double alpha = (normal.dot (d_vp_p) + 1.0) * 0.5;
    std::size_t fi = static_cast<std::size_t> (std::floor (alpha * hist_vp_.size ()));
    fi = std::max<std::size_t> (0u, fi);
    fi = std::min<std::size_t> (hist_vp_.size () - 1, fi);
    // Bin into the histogram
    hist_vp_ [fi] += hist_incr;
  }

  // We only output _1_ signature
  output.points.resize (1);
  output.width = 1;
  output.height = 1;

  // Estimate the FPFH at nn_indices[0] using the entire cloud and copy the resultant signature
  auto outPtr = std::begin (output.points[0].histogram);

  outPtr = std::copy_n (hist_f1_.data (), hist_f1_.size (), outPtr);
  outPtr = std::copy_n (hist_f2_.data (), hist_f2_.size (), outPtr);
  outPtr = std::copy_n (hist_f3_.data (), hist_f3_.size (), outPtr);
  outPtr = std::copy_n (hist_f4_.data (), hist_f4_.size (), outPtr);
  outPtr = std::copy_n (hist_vp_.data (), hist_vp_.size (), outPtr);
}

#define PCL_INSTANTIATE_VFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::VFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_VFH_H_

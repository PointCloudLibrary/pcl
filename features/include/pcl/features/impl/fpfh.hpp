/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 */

#pragma once

#include <pcl/features/fpfh.h>

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/features/pfh_tools.h>

#include <set> // for std::set

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computePairFeatures (
    const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
    int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4)
{
  pcl::computePairFeatures (cloud[p_idx].getVector4fMap (), normals[p_idx].getNormalVector4fMap (),
      cloud[q_idx].getVector4fMap (), normals[q_idx].getNormalVector4fMap (),
      f1, f2, f3, f4);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computePointSPFHSignature (
    const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
    int p_idx, int row, const std::vector<int> &indices,
    Eigen::MatrixXf &hist_f1, Eigen::MatrixXf &hist_f2, Eigen::MatrixXf &hist_f3)
{
  Eigen::Vector4f pfh_tuple;
  // Get the number of bins from the histograms size
  // @TODO: use arrays
  int nr_bins_f1 = static_cast<int> (hist_f1.cols ());
  int nr_bins_f2 = static_cast<int> (hist_f2.cols ());
  int nr_bins_f3 = static_cast<int> (hist_f3.cols ());

  // Factorization constant
  float hist_incr = 100.0f / static_cast<float>(indices.size () - 1);

  // Iterate over all the points in the neighborhood
  for (const auto &index : indices)
  {
    // Avoid unnecessary returns
    if (p_idx == index)
        continue;

    // Compute the pair P to NNi
    if (!computePairFeatures (cloud, normals, p_idx, index, pfh_tuple[0], pfh_tuple[1], pfh_tuple[2], pfh_tuple[3]))
        continue;

    // Normalize the f1, f2, f3 features and push them in the histogram
    int h_index = static_cast<int> (std::floor (nr_bins_f1 * ((pfh_tuple[0] + M_PI) * d_pi_)));
    if (h_index < 0)           h_index = 0;
    if (h_index >= nr_bins_f1) h_index = nr_bins_f1 - 1;
    hist_f1 (row, h_index) += hist_incr;

    h_index = static_cast<int> (std::floor (nr_bins_f2 * ((pfh_tuple[1] + 1.0) * 0.5)));
    if (h_index < 0)           h_index = 0;
    if (h_index >= nr_bins_f2) h_index = nr_bins_f2 - 1;
    hist_f2 (row, h_index) += hist_incr;

    h_index = static_cast<int> (std::floor (nr_bins_f3 * ((pfh_tuple[2] + 1.0) * 0.5)));
    if (h_index < 0)           h_index = 0;
    if (h_index >= nr_bins_f3) h_index = nr_bins_f3 - 1;
    hist_f3 (row, h_index) += hist_incr;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::weightPointSPFHSignature (
    const Eigen::MatrixXf &hist_f1, const Eigen::MatrixXf &hist_f2, const Eigen::MatrixXf &hist_f3,
    const std::vector<int> &indices, const std::vector<float> &dists, Eigen::VectorXf &fpfh_histogram)
{
  assert (indices.size () == dists.size ());
  // @TODO: use arrays
  double sum_f1 = 0.0, sum_f2 = 0.0, sum_f3 = 0.0;
  float weight = 0.0, val_f1, val_f2, val_f3;

  // Get the number of bins from the histograms size
  const auto nr_bins_f1 = hist_f1.cols ();
  const auto nr_bins_f2 = hist_f2.cols ();
  const auto nr_bins_f3 = hist_f3.cols ();
  const auto nr_bins_f12 = nr_bins_f1 + nr_bins_f2;

  // Clear the histogram
  fpfh_histogram.setZero (nr_bins_f1 + nr_bins_f2 + nr_bins_f3);

  // Use the entire patch
  for (std::size_t idx = 0; idx < indices.size (); ++idx)
  {
    // Minus the query point itself
    if (dists[idx] == 0)
      continue;

    // Standard weighting function used
    weight = 1.0f / dists[idx];

    // Weight the SPFH of the query point with the SPFH of its neighbors
    for (Eigen::MatrixXf::Index f1_i = 0; f1_i < nr_bins_f1; ++f1_i)
    {
      val_f1 = hist_f1 (indices[idx], f1_i) * weight;
      sum_f1 += val_f1;
      fpfh_histogram[f1_i] += val_f1;
    }

    for (Eigen::MatrixXf::Index f2_i = 0; f2_i < nr_bins_f2; ++f2_i)
    {
      val_f2 = hist_f2 (indices[idx], f2_i) * weight;
      sum_f2 += val_f2;
      fpfh_histogram[f2_i + nr_bins_f1] += val_f2;
    }

    for (Eigen::MatrixXf::Index f3_i = 0; f3_i < nr_bins_f3; ++f3_i)
    {
      val_f3 = hist_f3 (indices[idx], f3_i) * weight;
      sum_f3 += val_f3;
      fpfh_histogram[f3_i + nr_bins_f12] += val_f3;
    }
  }

  if (sum_f1 != 0)
    sum_f1 = 100.0 / sum_f1;           // histogram values sum up to 100
  if (sum_f2 != 0)
    sum_f2 = 100.0 / sum_f2;           // histogram values sum up to 100
  if (sum_f3 != 0)
    sum_f3 = 100.0 / sum_f3;           // histogram values sum up to 100

  // Adjust final FPFH values
  const auto denormalize_with = [](auto factor)
    {
      return [=](const auto& data) { return data * factor; };
    };

  auto last = fpfh_histogram.data ();
  last = std::transform(last, last + nr_bins_f1, last, denormalize_with (sum_f1));
  last = std::transform(last, last + nr_bins_f2, last, denormalize_with (sum_f2));
  std::transform(last, last + nr_bins_f3, last, denormalize_with (sum_f3));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computeSPFHSignatures (std::vector<int> &spfh_hist_lookup,
    Eigen::MatrixXf &hist_f1, Eigen::MatrixXf &hist_f2, Eigen::MatrixXf &hist_f3)
{
  // Allocate enough space to hold the NN search results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  std::set<int> spfh_indices_set;
  std::vector<int> spfh_indices_vec;
  spfh_hist_lookup.resize (surface_->size ());

  // Build a list of (unique) indices for which we will need to compute SPFH signatures
  // (We need an SPFH signature for every point that is a neighbor of any point in input_[indices_])
  if (surface_ != input_ ||
      indices_->size () != surface_->size ())
  {
    for (const auto& p_idx: *indices_)
    {
      if (this->searchForNeighbors (p_idx, search_parameter_, nn_indices, nn_dists) == 0)
        continue;

      spfh_indices_set.insert (nn_indices.begin (), nn_indices.end ());
    }
    spfh_indices_vec.resize (spfh_indices_set.size ());
    std::copy (spfh_indices_set.cbegin (), spfh_indices_set.cend (), spfh_indices_vec.begin ());
  }
  else
  {
    // Special case: When a feature must be computed at every point, there is no need for a neighborhood search
    spfh_indices_vec.resize (indices_->size ());
    std::iota(spfh_indices_vec.begin (), spfh_indices_vec.end (),
              static_cast<decltype(spfh_indices_vec)::value_type>(0));
  }

  // Initialize the arrays that will store the SPFH signatures
  const auto data_size = spfh_indices_vec.size ();
  hist_f1.setZero (data_size, nr_bins_f1_);
  hist_f2.setZero (data_size, nr_bins_f2_);
  hist_f3.setZero (data_size, nr_bins_f3_);

  // Compute SPFH signatures for every point that needs them
#pragma omp parallel for \
  default(none) \
  shared(spfh_hist_lookup, spfh_indices_vec, hist_f1, hist_f2, hist_f3) \
  firstprivate(nn_indices, nn_dists) \
  num_threads(threads_) \
  if (threads_ != -1)
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t> (spfh_indices_vec.size ()); ++i)
  {
    // Get the next point index
    int p_idx = spfh_indices_vec[i];

    // Find the neighborhood around p_idx
    if (this->searchForNeighbors (*surface_, p_idx, search_parameter_, nn_indices, nn_dists) == 0)
      continue;

    // Estimate the SPFH signature around p_idx
    computePointSPFHSignature (*surface_, *normals_, p_idx, i, nn_indices, hist_f1, hist_f2, hist_f3);

    // Populate a lookup table for converting a point index to its corresponding row in the spfh_hist_* matrices
    spfh_hist_lookup[p_idx] = i;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the NN search results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  std::vector<int> spfh_hist_lookup;
  computeSPFHSignatures (spfh_hist_lookup, hist_f1_, hist_f2_, hist_f3_);

  output.is_dense = true;

  // Iterate over the entire index vector
  for (std::size_t idx = 0; idx < indices_->size (); ++idx)
  {
    if (input_->is_dense || isFinite ((*input_)[(*indices_)[idx]])) {
      if (this->searchForNeighbors(
              (*indices_)[idx], search_parameter_, nn_indices, nn_dists)) {
        // ... and remap the nn_indices values so that they represent row indices in the spfh_hist_* matrices
        // instead of indices into surface_->points
        for (auto &nn_index : nn_indices)
          nn_index = spfh_hist_lookup[nn_index];

        // Compute the FPFH signature (i.e. compute a weighted combination of local SPFH signatures) ...
        weightPointSPFHSignature (hist_f1_, hist_f2_, hist_f3_, nn_indices, nn_dists, fpfh_histogram_);

        // ...and copy it into the output cloud
        std::copy_n(fpfh_histogram_.data (), fpfh_histogram_.size (), output[idx].histogram);
        continue;
      }
    }

    for (Eigen::Index d = 0; d < fpfh_histogram_.size(); ++d)
      output[idx].histogram[d] = std::numeric_limits<float>::quiet_NaN();

    if (output.is_dense)
      output.is_dense = false;
  }
}

#define PCL_INSTANTIATE_FPFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::FPFHEstimation<T,NT,OutT>;


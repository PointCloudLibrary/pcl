/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2016-, Open Perception, Inc.
 *  Copyright (c) 2016, Voxar Labs, CIn-UFPE / DEINFO-UFRPE
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
 */

#ifndef PCL_FEATURES_IMPL_GASD_H_
#define PCL_FEATURES_IMPL_GASD_H_

#include <pcl/features/gasd.h>
#include <pcl/common/common.h> // for getMinMax3D
#include <pcl/common/transforms.h>

#include <vector>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDEstimation<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  // Resize the output dataset
  output.resize (1);

  // Copy header and is_dense flag from input
  output.header = surface_->header;
  output.is_dense = surface_->is_dense;

  // Perform the actual feature computation
  computeFeature (output);

  Feature<PointInT, PointOutT>::deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDEstimation<PointInT, PointOutT>::computeAlignmentTransform ()
{
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  // compute centroid of the object's partial view
  pcl::compute3DCentroid (*surface_, *indices_, centroid);

  // compute covariance matrix from points and centroid of the object's partial view
  pcl::computeCovarianceMatrix (*surface_, *indices_, centroid, covariance_matrix);

  Eigen::Matrix3f eigenvectors;
  Eigen::Vector3f eigenvalues;

  // compute eigenvalues and eigenvectors of the covariance matrix
  pcl::eigen33 (covariance_matrix, eigenvectors, eigenvalues);

  // z axis of the reference frame is the eigenvector associated with the minimal eigenvalue
  Eigen::Vector3f z_axis = eigenvectors.col (0);

  // if angle between z axis and viewing direction is in the [-90 deg, 90 deg] range, then z axis is negated
  if (z_axis.dot (view_direction_) > 0)
  {
    z_axis = -z_axis;
  }

  // x axis of the reference frame is the eigenvector associated with the maximal eigenvalue 
  const Eigen::Vector3f x_axis = eigenvectors.col (2);

  // y axis is the cross product of z axis and x axis
  const Eigen::Vector3f y_axis = z_axis.cross (x_axis);

  const Eigen::Vector3f centroid_xyz = centroid.head<3> ();

  // compute alignment transform from axes and centroid
  transform_ << x_axis.transpose (), -x_axis.dot (centroid_xyz),
                y_axis.transpose (), -y_axis.dot (centroid_xyz),
                z_axis.transpose (), -z_axis.dot (centroid_xyz),
                0.0f, 0.0f, 0.0f, 1.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDEstimation<PointInT, PointOutT>::addSampleToHistograms (const Eigen::Vector4f &p,
                                                                 const float max_coord,
                                                                 const std::size_t half_grid_size,
                                                                 const HistogramInterpolationMethod interp,
                                                                 const float hbin,
                                                                 const float hist_incr,
                                                                 std::vector<Eigen::VectorXf> &hists)
{
  const std::size_t grid_size = half_grid_size * 2;

  // compute normalized coordinates with respect to axis-aligned bounding cube centered on the origin
  const Eigen::Vector3f scaled ( (p[0] / max_coord) * half_grid_size, (p[1] / max_coord) * half_grid_size, (p[2] / max_coord) * half_grid_size);

  // compute histograms array coords
  Eigen::Vector4f coords (scaled[0] + half_grid_size, scaled[1] + half_grid_size, scaled[2] + half_grid_size, hbin);

  // if using histogram interpolation, subtract 0.5 so samples with the central value of the bin have full weight in it
  if (interp != INTERP_NONE)
  {
    coords -= Eigen::Vector4f (0.5f, 0.5f, 0.5f, 0.5f);
  }

  // compute histograms bins indices
  const Eigen::Vector4f bins (std::floor (coords[0]), std::floor (coords[1]), std::floor (coords[2]), std::floor (coords[3]));

  // compute indices of the bin where the sample falls into
  const std::size_t grid_idx = ( (bins[0] + 1) * (grid_size + 2) + bins[1] + 1) * (grid_size + 2) + bins[2] + 1;
  const std::size_t h_idx = bins[3] + 1;

  if (interp == INTERP_NONE)
  {
    // no interpolation
    hists[grid_idx][h_idx] += hist_incr;
  }
  else
  {
    // if using histogram interpolation, compute trilinear interpolation
    coords -= Eigen::Vector4f (bins[0], bins[1], bins[2], 0.0f);

    const float v_x1 = hist_incr * coords[0];
    const float v_x0 = hist_incr - v_x1;

    const float v_xy11 = v_x1 * coords[1];
    const float v_xy10 = v_x1 - v_xy11;
    const float v_xy01 = v_x0 * coords[1];
    const float v_xy00 = v_x0 - v_xy01;

    const float v_xyz111 = v_xy11 * coords[2];
    const float v_xyz110 = v_xy11 - v_xyz111;
    const float v_xyz101 = v_xy10 * coords[2];
    const float v_xyz100 = v_xy10 - v_xyz101;
    const float v_xyz011 = v_xy01 * coords[2];
    const float v_xyz010 = v_xy01 - v_xyz011;
    const float v_xyz001 = v_xy00 * coords[2];
    const float v_xyz000 = v_xy00 - v_xyz001;

    if (interp == INTERP_TRILINEAR)
    {
      // trilinear interpolation
      hists[grid_idx][h_idx] += v_xyz000;
      hists[grid_idx + 1][h_idx] += v_xyz001;
      hists[grid_idx + (grid_size + 2)][h_idx] += v_xyz010;
      hists[grid_idx + (grid_size + 3)][h_idx] += v_xyz011;
      hists[grid_idx + (grid_size + 2) * (grid_size + 2)][h_idx] += v_xyz100;
      hists[grid_idx + (grid_size + 2) * (grid_size + 2) + 1][h_idx] += v_xyz101;
      hists[grid_idx + (grid_size + 3) * (grid_size + 2)][h_idx] += v_xyz110;
      hists[grid_idx + (grid_size + 3) * (grid_size + 2) + 1][h_idx] += v_xyz111;
    }
    else
    {
      // quadrilinear interpolation
      coords[3] -= bins[3];

      const float v_xyzh1111 = v_xyz111 * coords[3];
      const float v_xyzh1110 = v_xyz111 - v_xyzh1111;
      const float v_xyzh1101 = v_xyz110 * coords[3];
      const float v_xyzh1100 = v_xyz110 - v_xyzh1101;
      const float v_xyzh1011 = v_xyz101 * coords[3];
      const float v_xyzh1010 = v_xyz101 - v_xyzh1011;
      const float v_xyzh1001 = v_xyz100 * coords[3];
      const float v_xyzh1000 = v_xyz100 - v_xyzh1001;
      const float v_xyzh0111 = v_xyz011 * coords[3];
      const float v_xyzh0110 = v_xyz011 - v_xyzh0111;
      const float v_xyzh0101 = v_xyz010 * coords[3];
      const float v_xyzh0100 = v_xyz010 - v_xyzh0101;
      const float v_xyzh0011 = v_xyz001 * coords[3];
      const float v_xyzh0010 = v_xyz001 - v_xyzh0011;
      const float v_xyzh0001 = v_xyz000 * coords[3];
      const float v_xyzh0000 = v_xyz000 - v_xyzh0001;

      hists[grid_idx][h_idx] += v_xyzh0000;
      hists[grid_idx][h_idx + 1] += v_xyzh0001;
      hists[grid_idx + 1][h_idx] += v_xyzh0010;
      hists[grid_idx + 1][h_idx + 1] += v_xyzh0011;
      hists[grid_idx + (grid_size + 2)][h_idx] += v_xyzh0100;
      hists[grid_idx + (grid_size + 2)][h_idx + 1] += v_xyzh0101;
      hists[grid_idx + (grid_size + 3)][h_idx] += v_xyzh0110;
      hists[grid_idx + (grid_size + 3)][h_idx + 1] += v_xyzh0111;
      hists[grid_idx + (grid_size + 2) * (grid_size + 2)][h_idx] += v_xyzh1000;
      hists[grid_idx + (grid_size + 2) * (grid_size + 2)][h_idx + 1] += v_xyzh1001;
      hists[grid_idx + (grid_size + 2) * (grid_size + 2) + 1][h_idx] += v_xyzh1010;
      hists[grid_idx + (grid_size + 2) * (grid_size + 2) + 1][h_idx + 1] += v_xyzh1011;
      hists[grid_idx + (grid_size + 3) * (grid_size + 2)][h_idx] += v_xyzh1100;
      hists[grid_idx + (grid_size + 3) * (grid_size + 2)][h_idx + 1] += v_xyzh1101;
      hists[grid_idx + (grid_size + 3) * (grid_size + 2) + 1][h_idx] += v_xyzh1110;
      hists[grid_idx + (grid_size + 3) * (grid_size + 2) + 1][h_idx + 1] += v_xyzh1111;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDEstimation<PointInT, PointOutT>::copyShapeHistogramsToOutput (const std::size_t grid_size,
                                                                       const std::size_t hists_size,
                                                                       const std::vector<Eigen::VectorXf> &hists,
                                                                       PointCloudOut &output,
                                                                       std::size_t &pos)
{
  for (std::size_t i = 0; i < grid_size; ++i)
  {
    for (std::size_t j = 0; j < grid_size; ++j)
    {
      for (std::size_t k = 0; k < grid_size; ++k)
      {
        const std::size_t idx = ( (i + 1) * (grid_size + 2) + (j + 1)) * (grid_size + 2) + (k + 1);

        std::copy (hists[idx].data () + 1, hists[idx].data () + hists_size + 1, output[0].histogram + pos);
        pos += hists_size;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // compute alignment transform using reference frame
  computeAlignmentTransform ();

  // align point cloud
  pcl::transformPointCloud (*surface_, *indices_, shape_samples_, transform_);

  const std::size_t shape_grid_size = shape_half_grid_size_ * 2;

  // each histogram dimension has 2 additional bins, 1 in each boundary, for performing interpolation
  std::vector<Eigen::VectorXf> shape_hists ((shape_grid_size + 2) * (shape_grid_size + 2) * (shape_grid_size + 2),
                                             Eigen::VectorXf::Zero (shape_hists_size_ + 2));

  Eigen::Vector4f centroid_p = Eigen::Vector4f::Zero ();

  // compute normalization factor for distances between samples and centroid
  Eigen::Vector4f far_pt;
  pcl::getMaxDistance (shape_samples_, centroid_p, far_pt);
  far_pt[3] = 0;
  const float distance_normalization_factor = (centroid_p - far_pt).norm ();

  // compute normalization factor with respect to axis-aligned bounding cube centered on the origin
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D (shape_samples_, min_pt, max_pt);

  max_coord_ = std::max (min_pt.head<3> ().cwiseAbs ().maxCoeff (), max_pt.head<3> ().cwiseAbs ().maxCoeff ());

  // normalize sample contribution with respect to the total number of points in the cloud
  hist_incr_ = 100.0f / static_cast<float> (shape_samples_.size () - 1);

  // for each sample
  for (const auto& sample: shape_samples_)
  {
    // compute shape histogram array coord based on distance between sample and centroid
    const Eigen::Vector4f p (sample.x, sample.y, sample.z, 0.0f);
    const float d = p.norm ();

    const float shape_grid_step = distance_normalization_factor / shape_half_grid_size_;

    float integral;
    const float dist_hist_val = std::modf(d / shape_grid_step, &integral);

    const float dbin = dist_hist_val * shape_hists_size_;

    // add sample to shape histograms, optionally performing interpolation
    addSampleToHistograms (p, max_coord_, shape_half_grid_size_, shape_interp_, dbin, hist_incr_, shape_hists);
  }

  pos_ = 0;

  // copy shape histograms to output
  copyShapeHistogramsToOutput (shape_grid_size, shape_hists_size_, shape_hists, output, pos_);

  // set remaining values of the descriptor to zero (if any)
  std::fill (output[0].histogram + pos_, output[0].histogram + output[0].descriptorSize (), 0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDColorEstimation<PointInT, PointOutT>::copyColorHistogramsToOutput (const std::size_t grid_size,
                                                                            const std::size_t hists_size,
                                                                            std::vector<Eigen::VectorXf> &hists,
                                                                            PointCloudOut &output,
                                                                            std::size_t &pos)
{
  for (std::size_t i = 0; i < grid_size; ++i)
  {
    for (std::size_t j = 0; j < grid_size; ++j)
    {
      for (std::size_t k = 0; k < grid_size; ++k)
      {
        const std::size_t idx = ( (i + 1) * (grid_size + 2) + (j + 1)) * (grid_size + 2) + (k + 1);

        hists[idx][1] += hists[idx][hists_size + 1];
        hists[idx][hists_size] += hists[idx][0];

        std::copy (hists[idx].data () + 1, hists[idx].data () + hists_size + 1, output[0].histogram + pos);
        pos += hists_size;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::GASDColorEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // call shape feature computation
  GASDEstimation<PointInT, PointOutT>::computeFeature (output);

  const std::size_t color_grid_size = color_half_grid_size_ * 2;

  // each histogram dimension has 2 additional bins, 1 in each boundary, for performing interpolation
  std::vector<Eigen::VectorXf> color_hists ((color_grid_size + 2) * (color_grid_size + 2) * (color_grid_size + 2),
                                             Eigen::VectorXf::Zero (color_hists_size_ + 2));

  // for each sample
  for (const auto& sample: shape_samples_)
  {
    // compute shape histogram array coord based on distance between sample and centroid
    const Eigen::Vector4f p (sample.x, sample.y, sample.z, 0.0f);

    // compute hue value
    float hue = 0.f;

    const unsigned char max = std::max (sample.r, std::max (sample.g, sample.b));
    const unsigned char min = std::min (sample.r, std::min (sample.g, sample.b));

    const float diff_inv = 1.f / static_cast <float> (max - min);

    if (std::isfinite (diff_inv))
    {
      if (max == sample.r)
      {
        hue = 60.f * (static_cast <float> (sample.g - sample.b) * diff_inv);
      }
      else if (max == sample.g)
      {
        hue = 60.f * (2.f + static_cast <float> (sample.b - sample.r) * diff_inv);
      }
      else
      {
        hue = 60.f * (4.f + static_cast <float> (sample.r - sample.g) * diff_inv); // max == b
      }

      if (hue < 0.f)
      {
        hue += 360.f;
      }
    }

    // compute color histogram array coord based on hue value
    const float hbin = (hue / 360) * color_hists_size_;

    // add sample to color histograms, optionally performing interpolation
    GASDEstimation<PointInT, PointOutT>::addSampleToHistograms (p, max_coord_, color_half_grid_size_, color_interp_, hbin, hist_incr_, color_hists);
  }

  // copy color histograms to output
  copyColorHistogramsToOutput (color_grid_size, color_hists_size_, color_hists, output, pos_);

  // set remaining values of the descriptor to zero (if any)
  std::fill (output[0].histogram + pos_, output[0].histogram + output[0].descriptorSize (), 0.0f);
}

#define PCL_INSTANTIATE_GASDEstimation(InT, OutT) template class PCL_EXPORTS pcl::GASDEstimation<InT, OutT>;
#define PCL_INSTANTIATE_GASDColorEstimation(InT, OutT) template class PCL_EXPORTS pcl::GASDColorEstimation<InT, OutT>;

#endif  // PCL_FEATURES_IMPL_GASD_H_

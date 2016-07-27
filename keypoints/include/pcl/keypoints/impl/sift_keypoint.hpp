/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_SIFT_KEYPOINT_IMPL_H_
#define PCL_SIFT_KEYPOINT_IMPL_H_

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::SIFTKeypoint<PointInT, PointOutT>::setScales (float min_scale, int nr_octaves, int nr_scales_per_octave)
{
  min_scale_ = min_scale;
  nr_octaves_ = nr_octaves;
  nr_scales_per_octave_ = nr_scales_per_octave;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::SIFTKeypoint<PointInT, PointOutT>::setMinimumContrast (float min_contrast)
{
  min_contrast_ = min_contrast;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SIFTKeypoint<PointInT, PointOutT>::initCompute ()
{
  if (min_scale_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : Minimum scale (%f) must be strict positive!\n", 
               name_.c_str (), min_scale_);
    return (false);
  }
  if (nr_octaves_ < 1)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : Number of octaves (%d) must be at least 1!\n", 
               name_.c_str (), nr_octaves_);
    return (false);
  }
  if (nr_scales_per_octave_ < 1)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : Number of scales per octave (%d) must be at least 1!\n", 
               name_.c_str (), nr_scales_per_octave_);
    return (false);
  }
  if (min_contrast_ < 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : Minimum contrast (%f) must be non-negative!\n", 
               name_.c_str (), min_contrast_);
    return (false);
  }
  
  this->setKSearch (1);
  tree_.reset (new pcl::search::KdTree<PointInT> (true));
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::SIFTKeypoint<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  if (surface_ && surface_ != input_)
  {
    PCL_WARN ("[pcl::%s::detectKeypoints] : ", name_.c_str ());
    PCL_WARN ("A search surface has been set by setSearchSurface, but this SIFT keypoint detection algorithm does ");
    PCL_WARN ("not support search surfaces other than the input cloud.  ");
    PCL_WARN ("The cloud provided in setInputCloud is being used instead.\n");
  }

  // Check if the output has a "scale" field
  scale_idx_ = pcl::getFieldIndex<PointOutT> (output, "scale", out_fields_);

  // Make sure the output cloud is empty
  output.points.clear ();

  // Create a local copy of the input cloud that will be resized for each octave
  boost::shared_ptr<pcl::PointCloud<PointInT> > cloud (new pcl::PointCloud<PointInT> (*input_));

  VoxelGrid<PointInT> voxel_grid;
  // Search for keypoints at each octave
  float scale = min_scale_;
  for (int i_octave = 0; i_octave < nr_octaves_; ++i_octave)
  {
    // Downsample the point cloud
    const float s = 1.0f * scale; // note: this can be adjusted
    voxel_grid.setLeafSize (s, s, s);
    voxel_grid.setInputCloud (cloud);
    boost::shared_ptr<pcl::PointCloud<PointInT> > temp (new pcl::PointCloud<PointInT>);    
    voxel_grid.filter (*temp);
    cloud = temp;

    // Make sure the downsampled cloud still has enough points
    const size_t min_nr_points = 25;
    if (cloud->points.size () < min_nr_points)
      break;

    // Update the KdTree with the downsampled points
    tree_->setInputCloud (cloud);

    // Detect keypoints for the current scale
    detectKeypointsForOctave (*cloud, *tree_, scale, nr_scales_per_octave_, output);

    // Increase the scale by another octave
    scale *= 2;
  }

  // Set final properties
  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.header = input_->header;
  output.sensor_origin_ = input_->sensor_origin_;
  output.sensor_orientation_ = input_->sensor_orientation_;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::SIFTKeypoint<PointInT, PointOutT>::detectKeypointsForOctave (
    const PointCloudIn &input, KdTree &tree, float base_scale, int nr_scales_per_octave, 
    PointCloudOut &output)
{
  // Compute the difference of Gaussians (DoG) scale space
  std::vector<float> scales (nr_scales_per_octave + 3);
  for (int i_scale = 0; i_scale <= nr_scales_per_octave + 2; ++i_scale)
  {
    scales[i_scale] = base_scale * powf (2.0f, (1.0f * static_cast<float> (i_scale) - 1.0f) / static_cast<float> (nr_scales_per_octave));
  }
  Eigen::MatrixXf diff_of_gauss;
  computeScaleSpace (input, tree, scales, diff_of_gauss);

  // Find extrema in the DoG scale space
  std::vector<int> extrema_indices, extrema_scales;
  findScaleSpaceExtrema (input, tree, diff_of_gauss, extrema_indices, extrema_scales);

  output.points.reserve (output.points.size () + extrema_indices.size ());
  // Save scale?
  if (scale_idx_ != -1)
  {
    // Add keypoints to output
    for (size_t i_keypoint = 0; i_keypoint < extrema_indices.size (); ++i_keypoint)
    {
      PointOutT keypoint;
      const int &keypoint_index = extrema_indices[i_keypoint];
   
      keypoint.x = input.points[keypoint_index].x;
      keypoint.y = input.points[keypoint_index].y;
      keypoint.z = input.points[keypoint_index].z;
      memcpy (reinterpret_cast<char*> (&keypoint) + out_fields_[scale_idx_].offset,
              &scales[extrema_scales[i_keypoint]], sizeof (float));
      output.points.push_back (keypoint); 
    }
  }
  else
  {
    // Add keypoints to output
    for (size_t i_keypoint = 0; i_keypoint < extrema_indices.size (); ++i_keypoint)
    {
      PointOutT keypoint;
      const int &keypoint_index = extrema_indices[i_keypoint];
   
      keypoint.x = input.points[keypoint_index].x;
      keypoint.y = input.points[keypoint_index].y;
      keypoint.z = input.points[keypoint_index].z;

      output.points.push_back (keypoint); 
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> 
void pcl::SIFTKeypoint<PointInT, PointOutT>::computeScaleSpace (
    const PointCloudIn &input, KdTree &tree, const std::vector<float> &scales, 
    Eigen::MatrixXf &diff_of_gauss)
{
  diff_of_gauss.resize (input.size (), scales.size () - 1);

  // For efficiency, we will only filter over points within 3 standard deviations 
  const float max_radius = 3.0f * scales.back ();

  for (int i_point = 0; i_point < static_cast<int> (input.size ()); ++i_point)
  {
    std::vector<int> nn_indices;
    std::vector<float> nn_dist;
    tree.radiusSearch (i_point, max_radius, nn_indices, nn_dist); // *
    // * note: at this stage of the algorithm, we must find all points within a radius defined by the maximum scale, 
    //   regardless of the configurable search method specified by the user, so we directly employ tree.radiusSearch 
    //   here instead of using searchForNeighbors.

    // For each scale, compute the Gaussian "filter response" at the current point
    float filter_response = 0.0f;
    float previous_filter_response;
    for (size_t i_scale = 0; i_scale < scales.size (); ++i_scale)
    {
      float sigma_sqr = powf (scales[i_scale], 2.0f);

      float numerator = 0.0f;
      float denominator = 0.0f;
      for (size_t i_neighbor = 0; i_neighbor < nn_indices.size (); ++i_neighbor)
      {
        const float &value = getFieldValue_ (input.points[nn_indices[i_neighbor]]);
        const float &dist_sqr = nn_dist[i_neighbor];
        if (dist_sqr <= 9*sigma_sqr)
        {
          float w = expf (-0.5f * dist_sqr / sigma_sqr);
          numerator += value * w;
          denominator += w;
        }
        else break; // i.e. if dist > 3 standard deviations, then terminate early
      }
      previous_filter_response = filter_response;
      filter_response = numerator / denominator;

      // Compute the difference between adjacent scales
      if (i_scale > 0)
        diff_of_gauss (i_point, i_scale - 1) = filter_response - previous_filter_response;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::SIFTKeypoint<PointInT, PointOutT>::findScaleSpaceExtrema (
    const PointCloudIn &input, KdTree &tree, const Eigen::MatrixXf &diff_of_gauss, 
    std::vector<int> &extrema_indices, std::vector<int> &extrema_scales)
{
  const int k = 25;
  std::vector<int> nn_indices (k);
  std::vector<float> nn_dist (k);

  const int nr_scales = static_cast<int> (diff_of_gauss.cols ());
  std::vector<float> min_val (nr_scales), max_val (nr_scales);

  for (int i_point = 0; i_point < static_cast<int> (input.size ()); ++i_point)
  {
    // Define the local neighborhood around the current point
    const size_t nr_nn = tree.nearestKSearch (i_point, k, nn_indices, nn_dist); //*
    // * note: the neighborhood for finding local extrema is best defined as a small fixed-k neighborhood, regardless of
    //   the configurable search method specified by the user, so we directly employ tree.nearestKSearch here instead 
    //   of using searchForNeighbors

    // At each scale, find the extreme values of the DoG within the current neighborhood
    for (int i_scale = 0; i_scale < nr_scales; ++i_scale)
    {
      min_val[i_scale] = std::numeric_limits<float>::max ();
      max_val[i_scale] = -std::numeric_limits<float>::max ();

      for (size_t i_neighbor = 0; i_neighbor < nr_nn; ++i_neighbor)
      {
        const float &d = diff_of_gauss (nn_indices[i_neighbor], i_scale);

        min_val[i_scale] = (std::min) (min_val[i_scale], d);
        max_val[i_scale] = (std::max) (max_val[i_scale], d);
      }
    }

    // If the current point is an extreme value with high enough contrast, add it as a keypoint 
    for (int i_scale = 1; i_scale < nr_scales - 1; ++i_scale)
    {
      const float &val = diff_of_gauss (i_point, i_scale);

      // Does the point have sufficient contrast?
      if (fabs (val) >= min_contrast_)
      {
        // Is it a local minimum?
        if ((val == min_val[i_scale]) && 
            (val <  min_val[i_scale - 1]) && 
            (val <  min_val[i_scale + 1]))
        {
          extrema_indices.push_back (i_point);
          extrema_scales.push_back (i_scale);
        }
        // Is it a local maximum?
        else if ((val == max_val[i_scale]) && 
                 (val >  max_val[i_scale - 1]) && 
                 (val >  max_val[i_scale + 1]))
        {
          extrema_indices.push_back (i_point);
          extrema_scales.push_back (i_scale);
        }
      }
    }
  }
}

#define PCL_INSTANTIATE_SIFTKeypoint(T,U) template class PCL_EXPORTS pcl::SIFTKeypoint<T,U>;

#endif // #ifndef PCL_SIFT_KEYPOINT_IMPL_H_


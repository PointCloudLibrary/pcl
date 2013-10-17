/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 * $Id$
 */

#ifndef PCL_KEYPOINTS_IMPL_SMOOTHEDSURFACESKEYPOINT_H_
#define PCL_KEYPOINTS_IMPL_SMOOTHEDSURFACESKEYPOINT_H_

#include <pcl/keypoints/smoothed_surfaces_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include <pcl/io/pcd_io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT>::addSmoothedPointCloud (const PointCloudTConstPtr &cloud,
                                                                       const PointCloudNTConstPtr &normals,
                                                                       KdTreePtr &kdtree,
                                                                       float &scale)
{
  clouds_.push_back (cloud);
  cloud_normals_.push_back (normals);
  cloud_trees_.push_back (kdtree);
  scales_.push_back (std::pair<float, size_t> (scale, scales_.size ()));
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT>::resetClouds ()
{
  clouds_.clear ();
  cloud_normals_.clear ();
  scales_.clear ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT>::detectKeypoints (PointCloudT &output)
{
  // Calculate differences for each cloud
  std::vector<std::vector<float> > diffs (scales_.size ());

  // The cloud with the smallest scale has no differences
  std::vector<float> aux_diffs (input_->points.size (), 0.0f);
  diffs[scales_[0].second] = aux_diffs;

  cloud_trees_[scales_[0].second]->setInputCloud (clouds_[scales_[0].second]);
  for (size_t scale_i = 1; scale_i < clouds_.size (); ++scale_i)
  {
    size_t cloud_i = scales_[scale_i].second,
        cloud_i_minus_one = scales_[scale_i - 1].second;
    diffs[cloud_i].resize (input_->points.size ());
    PCL_INFO ("cloud_i %u cloud_i_minus_one %u\n", cloud_i, cloud_i_minus_one);
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
      diffs[cloud_i][point_i] = cloud_normals_[cloud_i]->points[point_i].getNormalVector3fMap ().dot (
          clouds_[cloud_i]->points[point_i].getVector3fMap () - clouds_[cloud_i_minus_one]->points[point_i].getVector3fMap ());

    // Setup kdtree for this cloud
    cloud_trees_[cloud_i]->setInputCloud (clouds_[cloud_i]);
  }


  // Find minima and maxima in differences inside the input cloud
  typename pcl::search::Search<PointT>::Ptr input_tree = cloud_trees_.back ();
  for (int point_i = 0; point_i < static_cast<int> (input_->points.size ()); ++point_i)
  {
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    input_tree->radiusSearch (point_i, input_scale_ * neighborhood_constant_, nn_indices, nn_distances);

    bool is_min = true, is_max = true;
    for (std::vector<int>::iterator nn_it = nn_indices.begin (); nn_it != nn_indices.end (); ++nn_it)
      if (*nn_it != point_i)
      {
        if (diffs[input_index_][point_i] < diffs[input_index_][*nn_it])
          is_max = false;
        else if (diffs[input_index_][point_i] > diffs[input_index_][*nn_it])
          is_min = false;
      }

    // If the point is a local minimum/maximum, check if it is the same over all the scales
    if (is_min || is_max)
    {
      bool passed_min = true, passed_max = true;
      for (size_t scale_i = 0; scale_i < scales_.size (); ++scale_i)
      {
        size_t cloud_i = scales_[scale_i].second;
        // skip input cloud
        if (cloud_i == clouds_.size () - 1)
          continue;

        nn_indices.clear (); nn_distances.clear ();
        cloud_trees_[cloud_i]->radiusSearch (point_i, scales_[scale_i].first * neighborhood_constant_, nn_indices, nn_distances);

        bool is_min_other_scale = true, is_max_other_scale = true;
        for (std::vector<int>::iterator nn_it = nn_indices.begin (); nn_it != nn_indices.end (); ++nn_it)
          if (*nn_it != point_i)
          {
            if (diffs[input_index_][point_i] < diffs[cloud_i][*nn_it])
              is_max_other_scale = false;
            else if (diffs[input_index_][point_i] > diffs[cloud_i][*nn_it])
              is_min_other_scale = false;
          }

        if (is_min == true && is_min_other_scale == false)
          passed_min = false;
        if (is_max == true && is_max_other_scale == false)
          passed_max = false;

        if (!passed_min && !passed_max)
          break;
      }

      // check if point was minimum/maximum over all the scales
      if (passed_min || passed_max)
      {
        output.points.push_back (input_->points[point_i]);
        keypoints_indices_->indices.push_back (point_i);
      }
    }
  }

  output.header = input_->header;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.height = 1;

  // debug stuff
//  for (size_t scale_i = 0; scale_i < scales_.size (); ++scale_i)
//  {
//    PointCloud<PointXYZI>::Ptr debug_cloud (new PointCloud<PointXYZI> ());
//    debug_cloud->points.resize (input_->points.size ());
//    debug_cloud->width = input_->width;
//    debug_cloud->height = input_->height;
//    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
//    {
//      debug_cloud->points[point_i].intensity = diffs[scales_[scale_i].second][point_i];
//      debug_cloud->points[point_i].x = input_->points[point_i].x;
//      debug_cloud->points[point_i].y = input_->points[point_i].y;
//      debug_cloud->points[point_i].z = input_->points[point_i].z;
//    }

//    char str[512]; sprintf (str, "diffs_%2d.pcd", scale_i);
//    io::savePCDFile (str, *debug_cloud);
//  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SmoothedSurfacesKeypoint<PointT, PointNT>::initCompute ()
{
  PCL_INFO ("SmoothedSurfacesKeypoint initCompute () called\n");
  if ( !Keypoint<PointT, PointT>::initCompute ())
  {
    PCL_ERROR ("[pcl::SmoothedSurfacesKeypoints::initCompute] Keypoint::initCompute failed\n");
    return false;
  }

  if (!normals_)
  {
    PCL_ERROR ("[pcl::SmoothedSurfacesKeypoints::initCompute] Input normals were not set\n");
    return false;
  }
  if (clouds_.size () == 0)
  {
    PCL_ERROR ("[pcl::SmoothedSurfacesKeypoints::initCompute] No other clouds were set apart from the input\n");
    return false;
  }

  if (input_->points.size () != normals_->points.size ())
  {
    PCL_ERROR ("[pcl::SmoothedSurfacesKeypoints::initCompute] The input cloud and the input normals differ in size\n");
    return false;
  }

  for (size_t cloud_i = 0; cloud_i < clouds_.size (); ++cloud_i)
  {
    if (clouds_[cloud_i]->points.size () != input_->points.size ())
    {
      PCL_ERROR ("[pcl::SmoothedSurfacesKeypoints::initCompute] Cloud %d does not have the same number of points as the input cloud\n", cloud_i);
      return false;
    }

    if (cloud_normals_[cloud_i]->points.size () != input_->points.size ())
    {
      PCL_ERROR ("[pcl::SmoothedSurfacesKeypoints::initCompute] Normals for cloud %d do not have the same number of points as the input cloud\n", cloud_i);
      return false;
    }
  }

  // Add the input cloud as the last index
  scales_.push_back (std::pair<float, size_t> (input_scale_, scales_.size ()));
  clouds_.push_back (input_);
  cloud_normals_.push_back (normals_);
  cloud_trees_.push_back (tree_);
  // Sort the clouds by their scales
  sort (scales_.begin (), scales_.end (), compareScalesFunction);

  // Find the index of the input after sorting
  for (size_t i = 0; i < scales_.size (); ++i)
    if (scales_[i].second == scales_.size () - 1)
    {
      input_index_ = i;
      break;
    }

  PCL_INFO ("Scales: ");
  for (size_t i = 0; i < scales_.size (); ++i) PCL_INFO ("(%d %f), ", scales_[i].second, scales_[i].first);
  PCL_INFO ("\n");

  return (true);
}


#define PCL_INSTANTIATE_SmoothedSurfacesKeypoint(T,NT) template class PCL_EXPORTS pcl::SmoothedSurfacesKeypoint<T,NT>;


#endif /* PCL_KEYPOINTS_IMPL_SMOOTHEDSURFACESKEYPOINT_H_ */

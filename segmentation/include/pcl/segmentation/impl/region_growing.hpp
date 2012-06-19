/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Sergey Ushakov
 * Email  : mine_all_mine@bk.ru
 *
 */

#ifndef PCL_SEGMENTATION_REGION_GROWING_HPP_
#define PCL_SEGMENTATION_REGION_GROWING_HPP_

#include <pcl/segmentation/region_growing.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>
#include <list>
#include <cmath>
#include <time.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::comparePair (std::pair<float, int> i, std::pair<float, int> j)
{
  return (i.first < j.first);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RegionGrowing<PointT>::RegionGrowing () :
  curvature_flag_ (true),
  smooth_mode_ (true),
  residual_flag_ (false),
  theta_threshold_ (30.0f / 180.0f * static_cast<float> (M_PI)),
  residual_threshold_ (0.05f),
  curvature_threshold_ (0.05f),
  neighbour_number_ (30),
  normal_flag_ (true),
  number_of_segments_ (0),
  segments_ (0),
  point_labels_ (0),
  num_pts_in_segment_ (0),
  point_neighbours_ (0),
  search_ (),
  normals_ (),
  cloud_for_segmentation_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RegionGrowing<PointT>::~RegionGrowing ()
{
  if (normals_ != 0)
    normals_.reset ();

  if (search_ != 0)
    search_.reset ();

  segments_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  point_neighbours_.clear ();

  if (cloud_for_segmentation_ != 0)
    cloud_for_segmentation_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setSmoothMode (bool value)
{
  smooth_mode_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setCurvatureTest (bool value)
{
  curvature_flag_ = value;

  if (curvature_flag_ == false && residual_flag_ == false)
    residual_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setResidualTest (bool value)
{
  residual_flag_ = value;

  if (curvature_flag_ == false && residual_flag_ == false)
    curvature_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowing<PointT>::getSmoothModeFlag () const
{
  return (smooth_mode_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowing<PointT>::getCurvatureTestFlag () const
{
  return (curvature_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowing<PointT>::getResidualTestFlag () const
{
  return (residual_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RegionGrowing<PointT>::getSmoothnessThreshold () const
{
  return (theta_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RegionGrowing<PointT>::getResidualThreshold () const
{
  return (residual_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RegionGrowing<PointT>::getCurvatureThreshold () const
{
  return (curvature_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowing<PointT>::getNumberOfNeighbours () const
{
  return (neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::search::Search<PointT>::Ptr
pcl::RegionGrowing<PointT>::getNeighbourSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::Normal>::Ptr
pcl::RegionGrowing<PointT>::getNormals () const
{
  return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
pcl::RegionGrowing<PointT>::getCloud () const
{
  return (cloud_for_segmentation_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<std::vector<int> >
pcl::RegionGrowing<PointT>::getSegments () const
{
  return (segments_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::RegionGrowing<PointT>::getSegmentFromPoint (int index)
{
  std::vector<int> result;
  if (cloud_for_segmentation_ == 0)
    return (result);

  // first of all we need to find out if this point belongs to cloud
  bool point_was_found = false;
  if (index < static_cast<int> (cloud_for_segmentation_->points.size ()) && index >= 0)
    point_was_found = true;

  if (point_was_found)
  {
    if (segments_.empty ())
    {
      segmentPoints ();
    }
    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector<std::vector<int> >::iterator i_segment;
    for (i_segment = segments_.begin (); i_segment != segments_.end (); i_segment++)
    {
      bool segment_was_found = false;
      result.clear ();
      result = *i_segment;
      std::vector<int>::iterator i_point;
      for (i_point = result.begin (); i_point != result.end (); i_point++)
      {
        if (*i_point == index)
        {
          segment_was_found = true;
          break;
        }
      }
      if (segment_was_found)
      {
        break;
      }
    }// next segment
  }// end if point was found

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::RegionGrowing<PointT>::getSegmentFromPoint (const PointT &point)
{
  std::vector<int> result;
  if (cloud_for_segmentation_ == 0)
    return (result);

  // first of all we need to find out if this point belongs to cloud
  bool point_was_found = false;
  int index = 0;
  for (size_t i = 0; i < cloud_for_segmentation_->points.size (); i++)
  {
    if (cloud_for_segmentation_->points[i].x != point.x) continue;
    if (cloud_for_segmentation_->points[i].y != point.y) continue;
    if (cloud_for_segmentation_->points[i].z != point.z) continue;

    point_was_found = true;
    index = static_cast<int> (i);
    break;
  }

  if (point_was_found)
  {
    if (segments_.empty ())
    {
      segmentPoints ();
    }
    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector<std::vector<int> >::iterator i_segment;
    for (i_segment = segments_.begin (); i_segment != segments_.end (); i_segment++)
    {
      bool segment_was_found = false;
      result.clear ();
      result = *i_segment;
      std::vector<int>::iterator i_point;
      for (i_point = result.begin (); i_point != result.end (); i_point++)
      {
        if (*i_point == index)
        {
          segment_was_found = true;
          break;
        }
      }
      if (segment_was_found)
      {
        break;
      }
    }// next segment
  }// end if point was found

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::RegionGrowing<PointT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!segments_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < segments_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = cloud_for_segmentation_->width;
    colored_cloud->height = cloud_for_segmentation_->height;
    colored_cloud->is_dense = cloud_for_segmentation_->is_dense;
    for (size_t i_point = 0; i_point < cloud_for_segmentation_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(cloud_for_segmentation_->points[i_point].data);
      point.y = *(cloud_for_segmentation_->points[i_point].data + 1);
      point.z = *(cloud_for_segmentation_->points[i_point].data + 2);
      colored_cloud->points.push_back (point);
    }

    std::vector<std::vector<int> >::iterator i_segment;
    int next_color = 0;
    for (i_segment = segments_.begin (); i_segment != segments_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->begin (); i_point != i_segment->end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setSmoothnessThreshold (float theta)
{
  theta_threshold_ = theta;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setResidualThreshold (float residual)
{
  residual_threshold_ = residual;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setCurvatureThreshold (float curvature)
{
  curvature_threshold_ = curvature;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setNumberOfNeighbours (unsigned int neighbour_number)
{
  neighbour_number_ = neighbour_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setNeighbourSearchMethod (typename pcl::search::Search<PointT>::Ptr search)
{
  if (search_ != 0)
    search_.reset ();

  search_ = search;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setNormals (pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  if (normals_ != 0)
    normals_.reset ();

  normals_ = normals;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::setCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud)
{
  if (cloud_for_segmentation_ != 0)
    cloud_for_segmentation_.reset ();

  cloud_for_segmentation_ = input_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowing<PointT>::segmentPoints ()
{
  number_of_segments_ = 0;

  segments_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  point_neighbours_.clear ();

  bool segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
    return (number_of_segments_);

  findPointNeighbours ();
  number_of_segments_ = applySmoothRegionGrowingAlgorithm ();
  assembleRegions ();

  return (number_of_segments_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowing<PointT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( cloud_for_segmentation_ == 0 || cloud_for_segmentation_->points.size () == 0 )
    return (false);

  // if user forgot to pass normals or the sizes of point and normal cloud are different
  if ( normals_ == 0 || cloud_for_segmentation_->points.size () != normals_->points.size () )
    return (false);

  // if residual test is on then we need to check if all needed parameters were correctly initialized
  if (residual_flag_)
  {
    if (residual_threshold_ <= 0.0f)
      return (false);
  }

  // if curvature test is on ...
  // if (curvature_flag_)
  // {
  //   in this case we do not need to check anything that related to it
  //   so we simply commented it
  // }

  // from here we check those parameters that are always valuable
  if (neighbour_number_ == 0)
    return (false);

  // if user didn't set search method
  if (search_ == 0)
    search_ = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

  search_->setInputCloud (cloud_for_segmentation_);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowing<PointT>::applySmoothRegionGrowingAlgorithm ()
{
  int num_of_pts = static_cast<int> (cloud_for_segmentation_->points.size ());
  point_labels_.resize (num_of_pts, -1);

  std::vector< std::pair<float, int> > point_residual;
  std::pair<float, int> pair;
  point_residual.resize (num_of_pts, pair);

  if (normal_flag_ == true)
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      point_residual[i_point].first = normals_->points[i_point].curvature;
      point_residual[i_point].second = i_point;
    }
    std::sort (point_residual.begin (), point_residual.end (), comparePair);
  }
  else
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      point_residual[i_point].first = 0;
      point_residual[i_point].second = i_point;
    }
  }
  int seed_counter = 0;
  int seed = point_residual[seed_counter].second;

  int segmented_pts_num = 0;
  int number_of_segments = 0;
  while (segmented_pts_num < num_of_pts)
  {
    int pts_in_segment;
    pts_in_segment = growRegion (seed, number_of_segments);
    segmented_pts_num += pts_in_segment;
    num_pts_in_segment_.push_back (pts_in_segment);
    number_of_segments++;

    for (int i_seed = seed_counter + 1; i_seed < num_of_pts; i_seed++)
    {
      int index = point_residual[i_seed].second;
      if (point_labels_[index] == -1)
      {
        seed = index;
        break;
      }
    }
  }

  return (number_of_segments);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::RegionGrowing<PointT>::growRegion (int initial_seed, int segment_number)
{
  std::queue<int> seeds;
  seeds.push (initial_seed);
  point_labels_[initial_seed] = segment_number;

  int num_pts_in_segment = 1;

  while (!seeds.empty ())
  {
    int curr_seed;
    curr_seed = seeds.front ();
    seeds.pop ();

    size_t i_nghbr = 0;
    while ( i_nghbr < neighbour_number_ && i_nghbr < point_neighbours_[curr_seed].size () )
    {
      int index = point_neighbours_[curr_seed][i_nghbr];
      if (point_labels_[index] != -1)
      {
        i_nghbr++;
        continue;
      }

      bool is_a_seed = false;
      bool belongs_to_segment = validatePoint (initial_seed, curr_seed, index, is_a_seed);

      if (belongs_to_segment == false)
      {
        i_nghbr++;
        continue;
      }

      point_labels_[index] = segment_number;
      num_pts_in_segment++;

      if (is_a_seed)
      {
        seeds.push (index);
      }

      i_nghbr++;
    }// next neighbour
  }// next seed

  return (num_pts_in_segment);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowing<PointT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
  is_a_seed = true;

  float cosine_threshold = cos (theta_threshold_);
  Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (cloud_for_segmentation_->points[point].data));
  Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (normals_->points[point].normal));

  //check the angle between normals
  if (smooth_mode_ == true)
  {
    Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
    float dot_product = fabs ( nghbr_normal.dot (initial_normal) );
    if (dot_product < cosine_threshold)
    {
      return (false);
    }
  }
  else
  {
    Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
    Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> (normals_->points[initial_seed].normal));
    float dot_product = fabsf ( nghbr_normal.dot (initial_seed_normal) );
    if (dot_product < cosine_threshold)
      return (false);
  }

  // check the curvature if needed
  if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
  {
    is_a_seed = false;
  }

  // check the residual if needed
  Eigen::Map<Eigen::Vector3f> nghbr_point ( (float*)cloud_for_segmentation_->points[nghbr].data );
  float residual = fabs ( initial_normal.dot (initial_point - nghbr_point) );
  if (residual_flag_ && residual > residual_threshold_)
    is_a_seed = false;

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::assembleRegions ()
{
  int number_of_segments = static_cast<int> (num_pts_in_segment_.size ());
  int number_of_points = static_cast<int> (cloud_for_segmentation_->points.size ());

  std::vector<int> segment;
  segments_.resize (number_of_segments, segment);

  for(int i_seg = 0; i_seg < number_of_segments; i_seg++)
  {
    segments_[i_seg].resize ( num_pts_in_segment_[i_seg], 0);
  }

  std::vector<int> counter;
  counter.resize (number_of_segments, 0);

  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int segment_index = point_labels_[i_point];
    int point_index = counter[segment_index];
	segments_[segment_index][point_index] = i_point;
    counter[segment_index] = point_index + 1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowing<PointT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (cloud_for_segmentation_->points.size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (point_number, neighbours);

  for (int i_point = 0; i_point < point_number; i_point++)
  {
    neighbours.clear ();
    search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
    point_neighbours_[i_point].swap (neighbours);
  }
}

#define PCL_INSTANTIATE_RegionGrowing(T) template class pcl::RegionGrowing<T>;

#endif    // PCL_SEGMENTATION_REGION_GROWING_HPP_

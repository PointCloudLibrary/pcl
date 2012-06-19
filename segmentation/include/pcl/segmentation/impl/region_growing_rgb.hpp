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

#ifndef PCL_SEGMENTATION_REGION_GROWING_RGB_HPP_
#define PCL_SEGMENTATION_REGION_GROWING_RGB_HPP_

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <queue>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RegionGrowingRGB<PointT>::RegionGrowingRGB () :
  region_neighbour_number_ (100),
  color_p2p_threshold_ (1225.0f),
  color_r2r_threshold_ (10.0f),
  min_point_number_ (10),
  distance_threshold_ (0.05f),
  point_distances_ (0),
  segment_labels_ (0),
  segment_neighbours_ (0),
  segment_distances_ (0)
{
  normal_flag_ = false;
  curvature_flag_ = false;
  residual_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RegionGrowingRGB<PointT>::~RegionGrowingRGB ()
{
  point_distances_.clear ();
  segment_labels_.clear ();
  segment_neighbours_.clear ();
  segment_distances_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowingRGB<PointT>::segmentPoints ()
{
  number_of_segments_ = 0;

  segments_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  point_neighbours_.clear ();
  point_distances_.clear ();
  segment_labels_.clear ();
  segment_neighbours_.clear ();
  segment_distances_.clear ();

  bool segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
    return (number_of_segments_);

  findPointNeighbours ();
  number_of_segments_ = applySmoothRegionGrowingAlgorithm ();
  RegionGrowing<PointT>::assembleRegions ();

  findSegmentNeighbours ();
  number_of_segments_ = applyRegionMergingAlgorithm ();

  return (number_of_segments_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowingRGB<PointT>::applyRegionMergingAlgorithm ()
{
  int number_of_points = static_cast<int> (cloud_for_segmentation_->points.size ());

  // calculate color of each segment
  std::vector< std::vector<unsigned int> > segment_color;
  std::vector<unsigned int> color;
  color.resize (3, 0);
  segment_color.resize (number_of_segments_, color);

  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int segment_index = point_labels_[i_point];
    segment_color[segment_index][0] += cloud_for_segmentation_->points[i_point].r;
    segment_color[segment_index][1] += cloud_for_segmentation_->points[i_point].g;
    segment_color[segment_index][2] += cloud_for_segmentation_->points[i_point].b;
  }
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    segment_color[i_seg][0] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][0]) / static_cast<float> (num_pts_in_segment_[i_seg]));
    segment_color[i_seg][1] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][1]) / static_cast<float> (num_pts_in_segment_[i_seg]));
    segment_color[i_seg][2] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][2]) / static_cast<float> (num_pts_in_segment_[i_seg]));
  }

  // now it is time to find out if there are segments with a similar color
  // and merge them together
  std::vector<unsigned int> num_pts_in_homogeneous_region;
  std::vector<int> num_seg_in_homogeneous_region;

  segment_labels_.resize (number_of_segments_, -1);

  float dist_thresh = distance_threshold_ * distance_threshold_;
  int homogeneous_region_number = 0;
  int curr_homogeneous_region = 0;
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    curr_homogeneous_region = 0;
    if (segment_labels_[i_seg] == -1)
    {
      segment_labels_[i_seg] = homogeneous_region_number;
      curr_homogeneous_region = homogeneous_region_number;
      num_pts_in_homogeneous_region.push_back (num_pts_in_segment_[i_seg]);
      num_seg_in_homogeneous_region.push_back (1);
      homogeneous_region_number++;
    }
	else
      curr_homogeneous_region = segment_labels_[i_seg];

    unsigned int i_nghbr = 0;
    while ( i_nghbr < region_neighbour_number_ && i_nghbr < segment_neighbours_[i_seg].size () )
    {
      int index = segment_neighbours_[i_seg][i_nghbr];
      if (segment_distances_[i_seg][i_nghbr] > dist_thresh)
      {
        i_nghbr++;
        continue;
      }
      if ( segment_labels_[index] == -1 )
      {
        float difference = calculateColorimetricalDifference (segment_color[i_seg], segment_color[index]);
        if (difference < color_r2r_threshold_)
        {
          segment_labels_[index] = curr_homogeneous_region;
          num_pts_in_homogeneous_region[curr_homogeneous_region] += num_pts_in_segment_[index];
          num_seg_in_homogeneous_region[curr_homogeneous_region] += 1;
        }
      }
      i_nghbr++;
    }// next neighbour
  }// next segment

  segment_color.clear ();
  color.clear ();

  std::vector< std::vector<int> > final_segments;
  std::vector<int> region;
  final_segments.resize (homogeneous_region_number, region);
  for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
  {
    final_segments[i_reg].resize (num_seg_in_homogeneous_region[i_reg], 0);
  }

  std::vector<int> counter;
  counter.resize (homogeneous_region_number, 0);
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    int index = segment_labels_[i_seg];
    final_segments[ index ][ counter[index] ] = i_seg;
    counter[index] += 1;
  }

  std::vector< std::vector< std::pair<float, int> > > region_neighbours;
  findRegionNeighbours (region_neighbours, final_segments);

  int final_segment_number = homogeneous_region_number;
  for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
  {
    if (num_pts_in_homogeneous_region[i_reg] < min_point_number_)
    {
      if ( region_neighbours[i_reg].empty () )
        continue;
      int nearest_neighbour = region_neighbours[i_reg][0].second;
      if ( region_neighbours[i_reg][0].first == std::numeric_limits<float>::max () )
        continue;
      int reg_index = segment_labels_[nearest_neighbour];
      int num_seg_in_reg = num_seg_in_homogeneous_region[i_reg];
      for (int i_seg = 0; i_seg < num_seg_in_reg; i_seg++)
      {
        int segment_index = final_segments[i_reg][i_seg];
        final_segments[reg_index].push_back (segment_index);
        segment_labels_[segment_index] = reg_index;
      }
      final_segments[i_reg].clear ();
      num_pts_in_homogeneous_region[reg_index] += num_pts_in_homogeneous_region[i_reg];
      num_pts_in_homogeneous_region[i_reg] = 0;
      num_seg_in_homogeneous_region[reg_index] += num_seg_in_homogeneous_region[i_reg];
      num_seg_in_homogeneous_region[i_reg] = 0;
      final_segment_number -= 1;

      int nghbr_number = static_cast<int> (region_neighbours[reg_index].size ());
      for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
      {
        if ( segment_labels_[ region_neighbours[reg_index][i_nghbr].second ] == reg_index )
        {
          region_neighbours[reg_index][i_nghbr].first = std::numeric_limits<float>::max ();
          region_neighbours[reg_index][i_nghbr].second = 0;
        }
      }
      nghbr_number = static_cast<int> (region_neighbours[i_reg].size ());
      for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
      {
        if ( segment_labels_[ region_neighbours[i_reg][i_nghbr].second ] != reg_index )
        {
          std::pair<float, int> pair;
          pair.first = region_neighbours[i_reg][i_nghbr].first;
          pair.second = region_neighbours[i_reg][i_nghbr].second;
          region_neighbours[reg_index].push_back (pair);
        }
      }
      region_neighbours[i_reg].clear ();
      std::sort (region_neighbours[reg_index].begin (), region_neighbours[reg_index].end (), comparePair);
    }
  }

  assembleRegions (num_pts_in_homogeneous_region, static_cast<int> (num_pts_in_homogeneous_region.size ()));

  return (final_segment_number);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RegionGrowingRGB<PointT>::calculateColorimetricalDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color) const
{
  float difference = 0.0f;
  difference += float ((first_color[0] - second_color[0]) * (first_color[0] - second_color[0]));
  difference += float ((first_color[1] - second_color[1]) * (first_color[1] - second_color[1]));
  difference += float ((first_color[2] - second_color[2]) * (first_color[2] - second_color[2]));
  return (difference);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowingRGB<PointT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
  is_a_seed = true;

  // check the color difference
  std::vector<unsigned int> point_color;
  point_color.resize (3, 0);
  std::vector<unsigned int> nghbr_color;
  nghbr_color.resize (3, 0);
  point_color[0] = cloud_for_segmentation_->points[point].r;
  point_color[1] = cloud_for_segmentation_->points[point].g;
  point_color[2] = cloud_for_segmentation_->points[point].b;
  nghbr_color[0] = cloud_for_segmentation_->points[nghbr].r;
  nghbr_color[1] = cloud_for_segmentation_->points[nghbr].g;
  nghbr_color[2] = cloud_for_segmentation_->points[nghbr].b;
  float difference = calculateColorimetricalDifference (point_color, nghbr_color);
  if (difference > color_p2p_threshold_)
    return (false);

  float cosine_threshold = cosf (theta_threshold_);

  // check the angle between normals if needed
  if (normal_flag_)
  {
    Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (cloud_for_segmentation_->points[point].data));
    Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (normals_->points[point].normal));
    if (smooth_mode_ == true)
    {
      Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
      float dot_product = fabsf (nghbr_normal.dot (initial_normal));
      if (dot_product < cosine_threshold)
        return (false);
    }
    else
    {
      Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
      Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> (normals_->points[initial_seed].normal));
      float dot_product = fabsf (nghbr_normal.dot (initial_seed_normal));
      if (dot_product < cosine_threshold)
        return (false);
    }
  }

  // check the curvature if needed
  if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
    is_a_seed = false;

  // check the residual if needed
  if (residual_flag_)
  {
    Eigen::Map<Eigen::Vector3f> nghbr_point (static_cast<float*> (cloud_for_segmentation_->points[nghbr].data));
    Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (cloud_for_segmentation_->points[point].data));
    Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (normals_->points[point].normal));
    float residual = fabsf (initial_normal.dot (initial_point - nghbr_point));
    if (residual > residual_threshold_)
      is_a_seed = false;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RegionGrowingRGB<PointT>::getPointColorThreshold () const
{
  return (powf (color_p2p_threshold_, 0.5f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RegionGrowingRGB<PointT>::getRegionColorThreshold () const
{
  return (color_r2r_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowingRGB<PointT>::getMinPointNumber () const
{
  return (min_point_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::RegionGrowingRGB<PointT>::getNumberOfRegionNeighbours () const
{
  return (region_neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setPointColorThreshold (float thresh)
{
  color_p2p_threshold_ = thresh * thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setRegionColorThreshold (float thresh)
{
  color_r2r_threshold_ = thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setMinPointNumber (unsigned int point_number)
{
  min_point_number_ = point_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setNumberOfRegionNeighbours (unsigned int nghbr_number)
{
  region_neighbour_number_ = nghbr_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowingRGB<PointT>::getNormalTestFlag () const
{
  return (normal_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setNormalTest (bool value)
{
  normal_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setCurvatureTest (bool value)
{
  curvature_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setResidualTest (bool value)
{
  residual_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::RegionGrowingRGB<PointT>::getSegmentFromPoint (int index)
{
  std::vector<int> result;
  if (cloud_for_segmentation_ == 0)
    return (result);

  // first of all we need to find out if this point belongs to cloud
  bool point_was_found = false;
  if (index < int (cloud_for_segmentation_->points.size ()) && index >= 0)
    point_was_found = true;

  if (point_was_found)
  {
    if (segments_.empty ())
    {
      // if we haven't done the segmentation yet, then we need to lanch the segmentation algorithm
      unsigned int number_of_segments = 0;
      number_of_segments = segmentPoints ();
      if (number_of_segments == 0)
        return (result);
    }// end if segments are empty

    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector< std::vector<int> >::iterator i_segment;
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
        break;
    }// next segment
  }// end if point was found

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::RegionGrowingRGB<PointT>::getSegmentFromPoint (const PointT& point)
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
      // if we haven't done the segmentation yet, then we need to launch the segmentation algorithm
      unsigned int number_of_segments = 0;
      number_of_segments = segmentPoints ();
      if (number_of_segments == 0)
        return (result);
    }// end if segments are empty

    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector< std::vector<int> >::iterator i_segment;
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
        break;
    }// next segment
  }// end if point was found

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegionGrowingRGB<PointT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( cloud_for_segmentation_ == 0 || cloud_for_segmentation_->points.size () == 0 )
    return (false);

  // if normal/smoothness test is on then we need to check if all needed variables and parameters
  // were correctly initialized
  if (normal_flag_)
  {
    // if user forgot to pass normals or the sizes of point and normal cloud are different
    if ( normals_ == 0 || cloud_for_segmentation_->points.size () != normals_->points.size () )
      return (false);
  }

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

  // here we check the parameters related to color-based segmentation
  if ( region_neighbour_number_ == 0 || color_p2p_threshold_ < 0.0f || color_r2r_threshold_ < 0.0f || distance_threshold_ < 0.0f )
    return (false);

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
template <typename PointT> float
pcl::RegionGrowingRGB<PointT>::getDistanceThreshold () const
{
  return (distance_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::setDistanceThreshold (float thresh)
{
  distance_threshold_ = thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (cloud_for_segmentation_->points.size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (point_number, neighbours);
  point_distances_.resize (point_number, distances);

  for (int i_point = 0; i_point < point_number; i_point++)
  {
    neighbours.clear ();
    distances.clear ();
    search_->nearestKSearch (i_point, region_neighbour_number_, neighbours, distances);
    point_neighbours_[i_point].swap (neighbours);
    point_distances_[i_point].swap (distances);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::findSegmentNeighbours ()
{
  std::vector<int> neighbours;
  std::vector<float> distances;
  segment_neighbours_.resize (number_of_segments_, neighbours);
  segment_distances_.resize (number_of_segments_, distances);

  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    std::vector<int> nghbrs;
    std::vector<float> dist;
    findRegionsKNN (i_seg, region_neighbour_number_, nghbrs, dist);
    segment_neighbours_[i_seg].swap (nghbrs);
    segment_distances_[i_seg].swap (dist);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::findRegionsKNN (int index, int nghbr_number, std::vector<int>& nghbrs, std::vector<float>& dist)
{
  std::vector<float> distances;
  float max_dist = std::numeric_limits<float>::max ();
  distances.resize (segments_.size (), max_dist);

  int number_of_points = num_pts_in_segment_[index];
  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    // find out to which segments these points belong
    // if it belongs to neighbouring segment and is close enough then remember segment and its distance
    int point_index = segments_[index][i_point];
    int number_of_neighbours = static_cast<int> (point_neighbours_[point_index].size ());
    for (int i_nghbr = 0; i_nghbr < number_of_neighbours; i_nghbr++)
    {
      // find segment
      int segment_index = -1;
      segment_index = point_labels_[ point_neighbours_[point_index][i_nghbr] ];

      if ( segment_index != index )
      {
        // try to push it to the queue
        if (distances[segment_index] > point_distances_[point_index][i_nghbr])
          distances[segment_index] = point_distances_[point_index][i_nghbr];
      }
    }
  }// next point

  std::priority_queue<std::pair<float, int> > segment_neighbours;
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    if (distances[i_seg] < max_dist)
    {
      segment_neighbours.push (std::make_pair<float, int> (distances[i_seg], i_seg) );
      if (int (segment_neighbours.size ()) > nghbr_number)
        segment_neighbours.pop ();
    }
  }

  int size = std::min<int> (static_cast<int> (segment_neighbours.size ()), nghbr_number);
  nghbrs.resize (size, 0);
  dist.resize (size, 0);
  int counter = 0;
  while ( !segment_neighbours.empty () && counter < nghbr_number )
  {
    dist[counter] = segment_neighbours.top ().first;
    nghbrs[counter] = segment_neighbours.top ().second;
    segment_neighbours.pop ();
    counter++;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::assembleRegions (std::vector<unsigned int>& num_pts_in_region, int num_regions)
{
  segments_.clear ();
  std::vector<int> segment;
  segments_.resize (num_regions, segment);
  for (int i_seg = 0; i_seg < num_regions; i_seg++)
  {
    segments_[i_seg].resize (num_pts_in_region[i_seg]);
  }

  std::vector<int> counter;
  counter.resize (num_regions, 0);
  int point_number = static_cast<int> (point_labels_.size ());
  for (int i_point = 0; i_point < point_number; i_point++)
  {
    int index = point_labels_[i_point];
    index = segment_labels_[index];
    segments_[index][ counter[index] ] = i_point;
    counter[index] += 1;
  }

  // now we need to erase empty regions
  std::vector< std::vector<int> >::iterator i_region;
  i_region = segments_.begin ();
  while(i_region != segments_.end ())
  {
    if ( i_region->empty () )
      i_region = segments_.erase (i_region);
    else
      i_region++;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegionGrowingRGB<PointT>::findRegionNeighbours (std::vector< std::vector< std::pair<float, int> > >& neighbours_out, std::vector< std::vector<int> >& regions_in)
{
  int region_number = static_cast<int> (regions_in.size ());
  neighbours_out.clear ();
  neighbours_out.resize (region_number);

  for (int i_reg = 0; i_reg < region_number; i_reg++)
  {
    int segment_num = static_cast<int> (regions_in[i_reg].size ());
    neighbours_out[i_reg].reserve (segment_num * region_neighbour_number_);
	for (int i_seg = 0; i_seg < segment_num; i_seg++)
    {
      int curr_segment = regions_in[i_reg][i_seg];
      int nghbr_number = static_cast<int> (segment_neighbours_[curr_segment].size ());
      std::pair<float, int> pair;
      for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
      {
        int segment_index = segment_neighbours_[curr_segment][i_nghbr];
        if ( segment_distances_[curr_segment][i_nghbr] == std::numeric_limits<float>::max () )
          continue;
        if (segment_labels_[segment_index] != i_reg)
        {
          pair.first = segment_distances_[curr_segment][i_nghbr];
          pair.second = segment_index;
          neighbours_out[i_reg].push_back (pair);
        }
      }// next neighbour
    }// next segment
    std::sort (neighbours_out[i_reg].begin (), neighbours_out[i_reg].end (), comparePair);
  }// next homogeneous region
}

#endif    // PCL_SEGMENTATION_REGION_GROWING_RGB_HPP_

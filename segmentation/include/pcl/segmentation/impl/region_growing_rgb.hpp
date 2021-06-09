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
 * Author : Sergey Ushakov
 * Email  : mine_all_mine@bk.ru
 *
 */

#ifndef PCL_SEGMENTATION_REGION_GROWING_RGB_HPP_
#define PCL_SEGMENTATION_REGION_GROWING_RGB_HPP_

#include <pcl/console/print.h> // for PCL_ERROR
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <queue>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowingRGB<PointT, NormalT>::RegionGrowingRGB () :
  color_p2p_threshold_ (1225.0f),
  color_r2r_threshold_ (10.0f),
  distance_threshold_ (0.05f),
  region_neighbour_number_ (100),
  point_distances_ (0),
  segment_neighbours_ (0),
  segment_distances_ (0),
  segment_labels_ (0)
{
  normal_flag_ = false;
  curvature_flag_ = false;
  residual_flag_ = false;
  min_pts_per_cluster_ = 10;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowingRGB<PointT, NormalT>::~RegionGrowingRGB ()
{
  point_distances_.clear ();
  segment_neighbours_.clear ();
  segment_distances_.clear ();
  segment_labels_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingRGB<PointT, NormalT>::getPointColorThreshold () const
{
  return (powf (color_p2p_threshold_, 0.5f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setPointColorThreshold (float thresh)
{
  color_p2p_threshold_ = thresh * thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingRGB<PointT, NormalT>::getRegionColorThreshold () const
{
  return (powf (color_r2r_threshold_, 0.5f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setRegionColorThreshold (float thresh)
{
  color_r2r_threshold_ = thresh * thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingRGB<PointT, NormalT>::getDistanceThreshold () const
{
  return (powf (distance_threshold_, 0.5f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setDistanceThreshold (float thresh)
{
  distance_threshold_ = thresh * thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> unsigned int
pcl::RegionGrowingRGB<PointT, NormalT>::getNumberOfRegionNeighbours () const
{
  return (region_neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setNumberOfRegionNeighbours (unsigned int nghbr_number)
{
  region_neighbour_number_ = nghbr_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowingRGB<PointT, NormalT>::getNormalTestFlag () const
{
  return (normal_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setNormalTestFlag (bool value)
{
  normal_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setCurvatureTestFlag (bool value)
{
  curvature_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::setResidualTestFlag (bool value)
{
  residual_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters_.clear ();
  clusters.clear ();
  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  point_distances_.clear ();
  segment_neighbours_.clear ();
  segment_distances_.clear ();
  segment_labels_.clear ();
  number_of_segments_ = 0;

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  findPointNeighbours ();
  applySmoothRegionGrowingAlgorithm ();
  RegionGrowing<PointT, NormalT>::assembleRegions ();

  findSegmentNeighbours ();
  applyRegionMergingAlgorithm ();

  std::vector<pcl::PointIndices>::iterator cluster_iter = clusters_.begin ();
  while (cluster_iter != clusters_.end ())
  {
    if (cluster_iter->indices.size () < min_pts_per_cluster_ ||
        cluster_iter->indices.size () > max_pts_per_cluster_)
    {
      cluster_iter = clusters_.erase (cluster_iter);
    }
    else
      ++cluster_iter;
  }

  clusters.reserve (clusters_.size ());
  std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowingRGB<PointT, NormalT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.empty () )
    return (false);

  // if normal/smoothness test is on then we need to check if all needed variables and parameters
  // were correctly initialized
  if (normal_flag_)
  {
    // if user forgot to pass normals or the sizes of point and normal cloud are different
    if ( !normals_ || input_->size () != normals_->size () )
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
  if (!search_)
    search_.reset (new pcl::search::KdTree<PointT>);

  if (indices_)
  {
    if (indices_->empty ())
      PCL_ERROR ("[pcl::RegionGrowingRGB::prepareForSegmentation] Empty given indices!\n");
    search_->setInputCloud (input_, indices_);
  }
  else
    search_->setInputCloud (input_);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  pcl::Indices neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (input_->size (), neighbours);
  point_distances_.resize (input_->size (), distances);

  for (int i_point = 0; i_point < point_number; i_point++)
  {
    int point_index = (*indices_)[i_point];
    neighbours.clear ();
    distances.clear ();
    search_->nearestKSearch (i_point, region_neighbour_number_, neighbours, distances);
    point_neighbours_[point_index].swap (neighbours);
    point_distances_[point_index].swap (distances);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::findSegmentNeighbours ()
{
  pcl::Indices neighbours;
  std::vector<float> distances;
  segment_neighbours_.resize (number_of_segments_, neighbours);
  segment_distances_.resize (number_of_segments_, distances);

  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    pcl::Indices nghbrs;
    std::vector<float> dist;
    findRegionsKNN (i_seg, region_neighbour_number_, nghbrs, dist);
    segment_neighbours_[i_seg].swap (nghbrs);
    segment_distances_[i_seg].swap (dist);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT,typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::findRegionsKNN (pcl::index_t index, pcl::uindex_t nghbr_number, pcl::Indices& nghbrs, std::vector<float>& dist)
{
  std::vector<float> distances;
  float max_dist = std::numeric_limits<float>::max ();
  distances.resize (clusters_.size (), max_dist);

  const auto number_of_points = num_pts_in_segment_[index];
  //loop through every point in this segment and check neighbours
  for (pcl::uindex_t i_point = 0; i_point < number_of_points; i_point++)
  {
    const auto point_index = clusters_[index].indices[i_point];
    const auto number_of_neighbours = point_neighbours_[point_index].size ();
    //loop through every neighbour of the current point, find out to which segment it belongs
    //and if it belongs to neighbouring segment and is close enough then remember segment and its distance
    for (std::size_t i_nghbr = 0; i_nghbr < number_of_neighbours; i_nghbr++)
    {
      // find segment
      const pcl::index_t segment_index = point_labels_[ point_neighbours_[point_index][i_nghbr] ];

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
      segment_neighbours.push (std::make_pair (distances[i_seg], i_seg) );
      if (segment_neighbours.size () > nghbr_number)
        segment_neighbours.pop ();
    }
  }

  const std::size_t size = std::min<std::size_t> (segment_neighbours.size (), static_cast<std::size_t>(nghbr_number));
  nghbrs.resize (size, 0);
  dist.resize (size, 0);
  pcl::uindex_t counter = 0;
  while ( !segment_neighbours.empty () && counter < nghbr_number )
  {
    dist[counter] = segment_neighbours.top ().first;
    nghbrs[counter] = segment_neighbours.top ().second;
    segment_neighbours.pop ();
    counter++;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::applyRegionMergingAlgorithm ()
{
  // calculate color of each segment
  std::vector< std::vector<unsigned int> > segment_color;
  std::vector<unsigned int> color;
  color.resize (3, 0);
  segment_color.resize (number_of_segments_, color);

  for (const auto& point_index : (*indices_))
  {
    int segment_index = point_labels_[point_index];
    segment_color[segment_index][0] += (*input_)[point_index].r;
    segment_color[segment_index][1] += (*input_)[point_index].g;
    segment_color[segment_index][2] += (*input_)[point_index].b;
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

  float dist_thresh = distance_threshold_;
  int homogeneous_region_number = 0;
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    int curr_homogeneous_region = 0;
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

  std::vector< std::vector< std::pair<float, pcl::index_t> > > region_neighbours;
  findRegionNeighbours (region_neighbours, final_segments);

  int final_segment_number = homogeneous_region_number;
  for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
  {
    if (num_pts_in_homogeneous_region[i_reg] < min_pts_per_cluster_)
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

      for (auto& nghbr : region_neighbours[reg_index])
      {
        if ( segment_labels_[ nghbr.second ] == reg_index )
        {
          nghbr.first = std::numeric_limits<float>::max ();
          nghbr.second = 0;
        }
      }
      for (const auto& nghbr : region_neighbours[i_reg])
      {
        if ( segment_labels_[ nghbr.second ] != reg_index )
        {
          region_neighbours[reg_index].push_back (nghbr);
        }
      }
      region_neighbours[i_reg].clear ();
      std::sort (region_neighbours[reg_index].begin (), region_neighbours[reg_index].end (), comparePair);
    }
  }

  assembleRegions (num_pts_in_homogeneous_region, static_cast<int> (num_pts_in_homogeneous_region.size ()));

  number_of_segments_ = final_segment_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingRGB<PointT, NormalT>::calculateColorimetricalDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color) const
{
  float difference = 0.0f;
  difference += float ((first_color[0] - second_color[0]) * (first_color[0] - second_color[0]));
  difference += float ((first_color[1] - second_color[1]) * (first_color[1] - second_color[1]));
  difference += float ((first_color[2] - second_color[2]) * (first_color[2] - second_color[2]));
  return (difference);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::findRegionNeighbours (std::vector< std::vector< std::pair<float, pcl::index_t> > >& neighbours_out, std::vector< std::vector<int> >& regions_in)
{
  int region_number = static_cast<int> (regions_in.size ());
  neighbours_out.clear ();
  neighbours_out.resize (region_number);

  for (int i_reg = 0; i_reg < region_number; i_reg++)
  {
    neighbours_out[i_reg].reserve (regions_in[i_reg].size () * region_neighbour_number_);
    for (const auto& curr_segment : regions_in[i_reg])
    {
      const std::size_t nghbr_number = segment_neighbours_[curr_segment].size ();
      std::pair<float, pcl::index_t> pair;
      for (std::size_t i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
      {
        const auto segment_index = segment_neighbours_[curr_segment][i_nghbr];
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::assembleRegions (std::vector<unsigned int>& num_pts_in_region, int num_regions)
{
  clusters_.clear ();
  pcl::PointIndices segment;
  clusters_.resize (num_regions, segment);
  for (int i_seg = 0; i_seg < num_regions; i_seg++)
  {
    clusters_[i_seg].indices.resize (num_pts_in_region[i_seg]);
  }

  std::vector<int> counter;
  counter.resize (num_regions, 0);
  for (const auto& point_index : (*indices_))
  {
    int index = point_labels_[point_index];
    index = segment_labels_[index];
    clusters_[index].indices[ counter[index] ] = point_index;
    counter[index] += 1;
  }

  // now we need to erase empty regions
  if (clusters_.empty ()) 
    return;

  std::vector<pcl::PointIndices>::iterator itr1, itr2;
  itr1 = clusters_.begin ();
  itr2 = clusters_.end () - 1;

  while (itr1 < itr2)
  {
    while (!(itr1->indices.empty ()) && itr1 < itr2) 
      ++itr1;
    while (  itr2->indices.empty ()  && itr1 < itr2) 
      --itr2;
	  
    if (itr1 != itr2)
      itr1->indices.swap (itr2->indices);
  }

  if (itr2->indices.empty ())
    clusters_.erase (itr2, clusters_.end ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowingRGB<PointT, NormalT>::validatePoint (pcl::index_t initial_seed, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const
{
  is_a_seed = true;

  // check the color difference
  std::vector<unsigned int> point_color;
  point_color.resize (3, 0);
  std::vector<unsigned int> nghbr_color;
  nghbr_color.resize (3, 0);
  point_color[0] = (*input_)[point].r;
  point_color[1] = (*input_)[point].g;
  point_color[2] = (*input_)[point].b;
  nghbr_color[0] = (*input_)[nghbr].r;
  nghbr_color[1] = (*input_)[nghbr].g;
  nghbr_color[2] = (*input_)[nghbr].b;
  float difference = calculateColorimetricalDifference (point_color, nghbr_color);
  if (difference > color_p2p_threshold_)
    return (false);

  float cosine_threshold = std::cos (theta_threshold_);

  // check the angle between normals if needed
  if (normal_flag_)
  {
    float data[4];
    data[0] = (*input_)[point].data[0];
    data[1] = (*input_)[point].data[1];
    data[2] = (*input_)[point].data[2];
    data[3] = (*input_)[point].data[3];

    Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (data));
    Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> ((*normals_)[point].normal));
    if (smooth_mode_flag_ == true)
    {
      Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> ((*normals_)[nghbr].normal));
      float dot_product = std::abs (nghbr_normal.dot (initial_normal));
      if (dot_product < cosine_threshold)
        return (false);
    }
    else
    {
      Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> ((*normals_)[nghbr].normal));
      Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> ((*normals_)[initial_seed].normal));
      float dot_product = std::abs (nghbr_normal.dot (initial_seed_normal));
      if (dot_product < cosine_threshold)
        return (false);
    }
  }

  // check the curvature if needed
  if (curvature_flag_ && (*normals_)[nghbr].curvature > curvature_threshold_)
    is_a_seed = false;

  // check the residual if needed
  if (residual_flag_)
  {
    float data_p[4];
    data_p[0] = (*input_)[point].data[0];
    data_p[1] = (*input_)[point].data[1];
    data_p[2] = (*input_)[point].data[2];
    data_p[3] = (*input_)[point].data[3];
    float data_n[4];
    data_n[0] = (*input_)[nghbr].data[0];
    data_n[1] = (*input_)[nghbr].data[1];
    data_n[2] = (*input_)[nghbr].data[2];
    data_n[3] = (*input_)[nghbr].data[3];
    Eigen::Map<Eigen::Vector3f> nghbr_point (static_cast<float*> (data_n));
    Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (data_p));
    Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> ((*normals_)[point].normal));
    float residual = std::abs (initial_normal.dot (initial_point - nghbr_point));
    if (residual > residual_threshold_)
      is_a_seed = false;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingRGB<PointT, NormalT>::getSegmentFromPoint (pcl::index_t index, pcl::PointIndices& cluster)
{
  cluster.indices.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  // first of all we need to find out if this point belongs to cloud
  bool point_was_found = false;
  for (const auto& point : (*indices_))
    if (point == index)
    {
      point_was_found = true;
      break;
    }

  if (point_was_found)
  {
    if (clusters_.empty ())
    {
      clusters_.clear ();
      point_neighbours_.clear ();
      point_labels_.clear ();
      num_pts_in_segment_.clear ();
      point_distances_.clear ();
      segment_neighbours_.clear ();
      segment_distances_.clear ();
      segment_labels_.clear ();
      number_of_segments_ = 0;

      segmentation_is_possible = prepareForSegmentation ();
      if ( !segmentation_is_possible )
      {
        deinitCompute ();
        return;
      }

      findPointNeighbours ();
      applySmoothRegionGrowingAlgorithm ();
      RegionGrowing<PointT, NormalT>::assembleRegions ();

      findSegmentNeighbours ();
      applyRegionMergingAlgorithm ();
    }
    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    for (const auto& i_segment : clusters_)
    {
      const auto it = std::find (i_segment.indices.cbegin (), i_segment.indices.cend (), index);
      if (it != i_segment.indices.cend())
      {
        // if segment was found
        cluster.indices.clear ();
        cluster.indices.reserve (i_segment.indices.size ());
        std::copy (i_segment.indices.begin (), i_segment.indices.end (), std::back_inserter (cluster.indices));
        break;
      }
    }// next segment
  }// end if point was found

  deinitCompute ();
}

#endif    // PCL_SEGMENTATION_REGION_GROWING_RGB_HPP_

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
template <typename PointT, typename NormalT>
pcl::RegionGrowing<PointT, NormalT>::RegionGrowing () :
  min_pts_per_cluster_ (1),
  max_pts_per_cluster_ (std::numeric_limits<int>::max ()),
  smooth_mode_flag_ (true),
  curvature_flag_ (true),
  residual_flag_ (false),
  theta_threshold_ (30.0f / 180.0f * static_cast<float> (M_PI)),
  residual_threshold_ (0.05f),
  curvature_threshold_ (0.05f),
  neighbour_number_ (30),
  search_ (),
  normals_ (),
  point_neighbours_ (0),
  point_labels_ (0),
  normal_flag_ (true),
  num_pts_in_segment_ (0),
  clusters_ (0),
  number_of_segments_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowing<PointT, NormalT>::~RegionGrowing ()
{
  if (search_ != 0)
    search_.reset ();
  if (normals_ != 0)
    normals_.reset ();

  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  clusters_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
pcl::RegionGrowing<PointT, NormalT>::getMinClusterSize ()
{
  return (min_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setMinClusterSize (int min_cluster_size)
{
  min_pts_per_cluster_ = min_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
pcl::RegionGrowing<PointT, NormalT>::getMaxClusterSize ()
{
  return (max_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setMaxClusterSize (int max_cluster_size)
{
  max_pts_per_cluster_ = max_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowing<PointT, NormalT>::getSmoothModeFlag () const
{
  return (smooth_mode_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setSmoothModeFlag (bool value)
{
  smooth_mode_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowing<PointT, NormalT>::getCurvatureTestFlag () const
{
  return (curvature_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setCurvatureTestFlag (bool value)
{
  curvature_flag_ = value;

  if (curvature_flag_ == false && residual_flag_ == false)
    residual_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowing<PointT, NormalT>::getResidualTestFlag () const
{
  return (residual_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setResidualTestFlag (bool value)
{
  residual_flag_ = value;

  if (curvature_flag_ == false && residual_flag_ == false)
    curvature_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowing<PointT, NormalT>::getSmoothnessThreshold () const
{
  return (theta_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setSmoothnessThreshold (float theta)
{
  theta_threshold_ = theta;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowing<PointT, NormalT>::getResidualThreshold () const
{
  return (residual_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setResidualThreshold (float residual)
{
  residual_threshold_ = residual;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowing<PointT, NormalT>::getCurvatureThreshold () const
{
  return (curvature_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setCurvatureThreshold (float curvature)
{
  curvature_threshold_ = curvature;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> unsigned int
pcl::RegionGrowing<PointT, NormalT>::getNumberOfNeighbours () const
{
  return (neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setNumberOfNeighbours (unsigned int neighbour_number)
{
  neighbour_number_ = neighbour_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename pcl::RegionGrowing<PointT, NormalT>::KdTreePtr
pcl::RegionGrowing<PointT, NormalT>::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setSearchMethod (const KdTreePtr& tree)
{
  if (search_ != 0)
    search_.reset ();

  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename pcl::RegionGrowing<PointT, NormalT>::NormalPtr
pcl::RegionGrowing<PointT, NormalT>::getInputNormals () const
{
  return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::setInputNormals (const NormalPtr& norm)
{
  if (normals_ != 0)
    normals_.reset ();

  normals_ = norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters_.clear ();
  clusters.clear ();
  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
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
  assembleRegions ();

  clusters.resize (clusters_.size ());
  std::vector<pcl::PointIndices>::iterator cluster_iter_input = clusters.begin ();
  for (std::vector<pcl::PointIndices>::const_iterator cluster_iter = clusters_.begin (); cluster_iter != clusters_.end (); cluster_iter++)
  {
    if ((static_cast<int> (cluster_iter->indices.size ()) >= min_pts_per_cluster_) &&
        (static_cast<int> (cluster_iter->indices.size ()) <= max_pts_per_cluster_))
    {
      *cluster_iter_input = *cluster_iter;
      cluster_iter_input++;
    }
  }

  clusters_ = std::vector<pcl::PointIndices> (clusters.begin (), cluster_iter_input);
  clusters.resize(clusters_.size());

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowing<PointT, NormalT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  // if user forgot to pass normals or the sizes of point and normal cloud are different
  if ( normals_ == 0 || input_->points.size () != normals_->points.size () )
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
  if (!search_)
    search_.reset (new pcl::search::KdTree<PointT>);

  if (indices_)
  {
    if (indices_->empty ())
      PCL_ERROR ("[pcl::RegionGrowing::prepareForSegmentation] Empty given indices!\n");
    search_->setInputCloud (input_, indices_);
  }
  else
    search_->setInputCloud (input_);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (input_->points.size (), neighbours);
  if (input_->is_dense)
  {
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      int point_index = (*indices_)[i_point];
      neighbours.clear ();
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
  else
  {
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      neighbours.clear ();
      int point_index = (*indices_)[i_point];
      if (!pcl::isFinite (input_->points[point_index]))
        continue;
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::applySmoothRegionGrowingAlgorithm ()
{
  int num_of_pts = static_cast<int> (indices_->size ());
  point_labels_.resize (input_->points.size (), -1);

  std::vector< std::pair<float, int> > point_residual;
  std::pair<float, int> pair;
  point_residual.resize (num_of_pts, pair);

  if (normal_flag_ == true)
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = normals_->points[point_index].curvature;
      point_residual[i_point].second = point_index;
    }
    std::sort (point_residual.begin (), point_residual.end (), comparePair);
  }
  else
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = 0;
      point_residual[i_point].second = point_index;
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

    //find next point that is not segmented yet
    for (int i_seed = seed_counter + 1; i_seed < num_of_pts; i_seed++)
    {
      int index = point_residual[i_seed].second;
      if (point_labels_[index] == -1)
      {
        seed = index;
        seed_counter = i_seed;
        break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
pcl::RegionGrowing<PointT, NormalT>::growRegion (int initial_seed, int segment_number)
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
template <typename PointT, typename NormalT> bool
pcl::RegionGrowing<PointT, NormalT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
  is_a_seed = true;

  float cosine_threshold = cosf (theta_threshold_);
  float data[4];

  data[0] = input_->points[point].data[0];
  data[1] = input_->points[point].data[1];
  data[2] = input_->points[point].data[2];
  data[3] = input_->points[point].data[3];
  Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (data));
  Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (normals_->points[point].normal));

  //check the angle between normals
  if (smooth_mode_flag_ == true)
  {
    Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
    float dot_product = fabsf (nghbr_normal.dot (initial_normal));
    if (dot_product < cosine_threshold)
    {
      return (false);
    }
  }
  else
  {
    Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
    Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> (normals_->points[initial_seed].normal));
    float dot_product = fabsf (nghbr_normal.dot (initial_seed_normal));
    if (dot_product < cosine_threshold)
      return (false);
  }

  // check the curvature if needed
  if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
  {
    is_a_seed = false;
  }

  // check the residual if needed
  float data_1[4];
  
  data_1[0] = input_->points[nghbr].data[0];
  data_1[1] = input_->points[nghbr].data[1];
  data_1[2] = input_->points[nghbr].data[2];
  data_1[3] = input_->points[nghbr].data[3];
  Eigen::Map<Eigen::Vector3f> nghbr_point (static_cast<float*> (data_1));
  float residual = fabsf (initial_normal.dot (initial_point - nghbr_point));
  if (residual_flag_ && residual > residual_threshold_)
    is_a_seed = false;

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::assembleRegions ()
{
  int number_of_segments = static_cast<int> (num_pts_in_segment_.size ());
  int number_of_points = static_cast<int> (input_->points.size ());

  pcl::PointIndices segment;
  clusters_.resize (number_of_segments, segment);

  for (int i_seg = 0; i_seg < number_of_segments; i_seg++)
  {
    clusters_[i_seg].indices.resize ( num_pts_in_segment_[i_seg], 0);
  }

  std::vector<int> counter;
  counter.resize (number_of_segments, 0);

  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int segment_index = point_labels_[i_point];
    if (segment_index != -1)
    {
      int point_index = counter[segment_index];
      clusters_[segment_index].indices[point_index] = i_point;
      counter[segment_index] = point_index + 1;
    }
  }

  number_of_segments_ = number_of_segments;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::getSegmentFromPoint (int index, pcl::PointIndices& cluster)
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
  int number_of_points = static_cast <int> (indices_->size ());
  for (int point = 0; point < number_of_points; point++)
    if ( (*indices_)[point] == index)
    {
      point_was_found = true;
      break;
    }

  if (point_was_found)
  {
    if (clusters_.empty ())
    {
      point_neighbours_.clear ();
      point_labels_.clear ();
      num_pts_in_segment_.clear ();
      number_of_segments_ = 0;

      segmentation_is_possible = prepareForSegmentation ();
      if ( !segmentation_is_possible )
      {
        deinitCompute ();
        return;
      }

      findPointNeighbours ();
      applySmoothRegionGrowingAlgorithm ();
      assembleRegions ();
    }
    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector <pcl::PointIndices>::iterator i_segment;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      bool segment_was_found = false;
      for (size_t i_point = 0; i_point < i_segment->indices.size (); i_point++)
      {
        if (i_segment->indices[i_point] == index)
        {
          segment_was_found = true;
          cluster.indices.clear ();
          cluster.indices.reserve (i_segment->indices.size ());
          std::copy (i_segment->indices.begin (), i_segment->indices.end (), std::back_inserter (cluster.indices));
          break;
        }
      }
      if (segment_was_found)
      {
        break;
      }
    }// next segment
  }// end if point was found

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::RegionGrowing<PointT, NormalT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
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
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
pcl::RegionGrowing<PointT, NormalT>::getColoredCloudRGBA ()
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGBA>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGBA point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      point.a = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
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

#define PCL_INSTANTIATE_RegionGrowing(T) template class pcl::RegionGrowing<T, pcl::Normal>;

#endif    // PCL_SEGMENTATION_REGION_GROWING_HPP_

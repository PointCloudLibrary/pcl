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

#ifndef _REGIONGROWING_HPP_
#define _REGIONGROWING_HPP_

#include "pcl/segmentation/region_growing.h"

#include "pcl/search/search.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <list>
#include <math.h>
#include <time.h>

namespace pcl
{
  template <typename PointT>
  RegionGrowing<PointT>::RegionGrowing ()
  {
    theta_threshold_ = 30.0 / 180.0 * M_PI;
    residual_threshold_ = 0.05;
    curvature_threshold_ = 0.05;
    neighbour_number_ = 30;
    curvature_flag_ = true;
    residual_flag_ = false;
    smooth_mode_ = true;
    segments_.clear();
  }

  template <typename PointT>
  RegionGrowing<PointT>::~RegionGrowing ()
  {
    if (normals_ != 0)
    {
      normals_.reset();
    }

    if (search_ != 0)
    {
      search_.reset();
    }

    segments_.clear();

    if (cloud_for_segmentation_ != 0)
    {
      cloud_for_segmentation_.reset();
    }
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setSmoothMode (bool value)
  {
    smooth_mode_ = value;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setCurvatureTest (bool value)
  {
    curvature_flag_ = value;

    if (curvature_flag_ == false && residual_flag_ == false)
    {
      residual_flag_ = true;
    }
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setResidualTest (bool value)
  {
    residual_flag_ = value;

    if (curvature_flag_ == false && residual_flag_ == false)
    {
      curvature_flag_ = true;
    }
  }

  template <typename PointT> bool
  RegionGrowing<PointT>::getSmoothModeFlag () const
  {
    return (smooth_mode_);
  }

  template <typename PointT> bool
  RegionGrowing<PointT>::getCurvatureTestFlag () const
  {
    return (curvature_flag_);
  }

  template <typename PointT> bool
  RegionGrowing<PointT>::getResidualTestFlag () const
  {
    return (residual_flag_);
  }

  template <typename PointT> float
  RegionGrowing<PointT>::getSmoothnessThreshold () const
  {
    return (theta_threshold_);
  }

  template <typename PointT> float
  RegionGrowing<PointT>::getResidualThreshold () const
  {
    return (residual_threshold_);
  }

  template <typename PointT> float
  RegionGrowing<PointT>::getCurvatureThreshold () const
  {
    return (curvature_threshold_);
  }

  template <typename PointT> unsigned int
  RegionGrowing<PointT>::getNumberOfNeighbours () const
  {
    return (neighbour_number_);
  }

  template <typename PointT> typename pcl::search::Search<PointT>::Ptr
  RegionGrowing<PointT>::getNeighbourSearchMethod () const
  {
    return (search_);
  }

  template <typename PointT> pcl::PointCloud<pcl::Normal>::Ptr
  RegionGrowing<PointT>::getNormals () const
  {
    return (normals_);
  }

  template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
  RegionGrowing<PointT>::getCloud () const
  {
    return (cloud_for_segmentation_);
  }

  template <typename PointT> std::vector<std::list<int>>
  RegionGrowing<PointT>::getSegments () const
  {
    return (segments_);
  }

  template <typename PointT> std::list<int>
  RegionGrowing<PointT>::getSegmentFromPoint (int index)
  {
    std::list<int> result;
    if (cloud_for_segmentation_ == 0)
    {
      return (result);
    }

    // first of all we need to find out if this point belongs to cloud
    bool point_was_found = false;
    if (index < cloud_for_segmentation_->points.size() && index >= 0)
    {
      point_was_found = true;
    }

    if (point_was_found)
    {
      if ( !segments_.empty() )
      {
        // if we have already made the segmentation, then find the segment
        // to which this point belongs
        std::vector<std::list<int>>::iterator i_segment;
        for (i_segment = segments_.begin(); i_segment != segments_.end(); i_segment++)
        {
          bool segment_was_found = false;
          result.clear();
          result = *i_segment;
          std::list<int>::iterator i_point;
          for (i_point = result.begin(); i_point != result.end(); i_point++)
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
      }// end if segments are not empty
	  else
      {
        bool segmentation_is_possible = prepareForSegmentation();
        if ( !segmentation_is_possible )
        {
          return (result);
        }
        // if we haven't done the segmentation yet, then we need to grow the segment
        std::vector<int> point_is_used;
        point_is_used.resize(cloud_for_segmentation_->points.size(), 0);
        growRegion(index, point_is_used, result);
      }
    }// end if point was found

    return (result);
  }

  template <typename PointT> std::list<int>
  RegionGrowing<PointT>::getSegmentFromPoint (typename PointT point)
  {
    std::list<int> result;
    if (cloud_for_segmentation_ == 0)
    {
      return (result);
    }

    // first of all we need to find out if this point belongs to cloud
    bool point_was_found = false;
    int index = 0;
    for (int i = 0; i < cloud_for_segmentation_->points.size(); i++)
    {
      if (cloud_for_segmentation_->points[i].x != point.x) continue;
      if (cloud_for_segmentation_->points[i].x != point.x) continue;
      if (cloud_for_segmentation_->points[i].x != point.x) continue;

      point_was_found = true;
      index = i;
      break;
    }

    if (point_was_found)
    {
      if ( !segments_.empty() )
      {
        // if we have already made the segmentation, then find the segment
        // to which this point belongs
        std::vector<std::list<int>>::iterator i_segment;
        for (i_segment = segments_.begin(); i_segment != segments_.end(); i_segment++)
        {
          bool segment_was_found = false;
          result.clear();
          result = *i_segment;
          std::list<int>::iterator i_point;
          for (i_point = result.begin(); i_point != result.end(); i_point++)
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
      }// end if segments are not empty
	  else
      {
        bool segmentation_is_possible = prepareForSegmentation();
        if ( !segmentation_is_possible )
        {
          return (result);
        }
        // if we haven't done the segmentation yet, then we need to grow the segment
        std::vector<int> point_is_used;
        point_is_used.resize(cloud_for_segmentation_->points.size(), 0);
        growRegion(index, point_is_used, result);
      }
    }// end if point was found

    return (result);
  }

  template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  RegionGrowing<PointT>::getColoredCloud ()
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    if (!segments_.empty())
    {
      colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

      srand(static_cast<unsigned int>(time(0)));
      std::vector<unsigned char> colors;
      for (int i_segment = 0; i_segment < segments_.size(); i_segment++)
      {
        colors.push_back(rand() % 256);
        colors.push_back(rand() % 256);
        colors.push_back(rand() % 256);
      }

      colored_cloud->width = cloud_for_segmentation_->width;
      colored_cloud->height = cloud_for_segmentation_->height;
      colored_cloud->is_dense = cloud_for_segmentation_->is_dense;
      for (int i_point = 0; i_point < cloud_for_segmentation_->points.size(); i_point++)
      {
        pcl::PointXYZRGB point;
        point.x = *(cloud_for_segmentation_->points[i_point].data);
        point.y = *(cloud_for_segmentation_->points[i_point].data + 1);
        point.z = *(cloud_for_segmentation_->points[i_point].data + 2);
        colored_cloud->points.push_back(point);
      }

      std::vector<std::list<int>>::iterator i_segment;
      int next_color = 0;
      for (i_segment = segments_.begin(); i_segment != segments_.end(); i_segment++)
      {
        std::list<int>::iterator i_point;
        for (i_point = i_segment->begin(); i_point != i_segment->end(); i_point++)
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

  template <typename PointT> void
  RegionGrowing<PointT>::setSmoothnessThreshold (float theta)
  {
    theta_threshold_ = theta;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setResidualThreshold (float residual)
  {
    residual_threshold_ = residual;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setCurvatureThreshold (float curvature)
  {
    curvature_threshold_ = curvature;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setNumberOfNeighbours (unsigned int neighbour_number)
  {
    neighbour_number_ = neighbour_number;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setNeighbourSearchMethod (typename pcl::search::Search<PointT>::Ptr search)
  {
    if (search_ != 0)
    {
      search_.reset();
    }

    search_ = search;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setNormals (pcl::PointCloud<pcl::Normal>::Ptr normals)
  {
    if (normals_ != 0)
    {
      normals_.reset();
    }

    normals_ = normals;
  }

  template <typename PointT> void
  RegionGrowing<PointT>::setCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud)
  {
    if (cloud_for_segmentation_ != 0)
    {
      cloud_for_segmentation_.reset();
    }

    cloud_for_segmentation_ = input_cloud;
  }

  template <typename PointT> unsigned int
  RegionGrowing<PointT>::segmentPoints ()
  {
    segments_.clear();
    unsigned int number_of_segments = 0;
    number_of_segments = applySmoothRegionGrowingAlgorithm();
    return (number_of_segments);
  }

  template <typename PointT> bool
  RegionGrowing<PointT>::prepareForSegmentation ()
  {
    // if user forgot to pass points or normals
    if ( cloud_for_segmentation_ == 0 || normals_ == 0 )
    {
      return (false);
    }

    // if number of points an number of normals are different
    if (cloud_for_segmentation_->points.size() != normals_->points.size())
    {
      return (false);
    }

    // if the cloud is empty
    if ( cloud_for_segmentation_->points.size() == 0 || normals_->points.size() == 0 )
    {
      return (false);
    }

    // if user passed wrong parameters
    if ( neighbour_number_ == 0 || residual_threshold_ <= 0.0f )
    {
      return (false);
    }

    // if user didn't set search method
    if (search_ == 0)
    {
      search_ = boost::shared_ptr< pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
    }

    search_->setInputCloud(cloud_for_segmentation_);

    return (true);
  }

  template <typename PointT> unsigned int
  RegionGrowing<PointT>::applySmoothRegionGrowingAlgorithm ()
  {
    bool segmentation_is_possible = prepareForSegmentation();
    if ( !segmentation_is_possible )
    {
      return (0);
    }

    int num_of_pts = static_cast<int>( cloud_for_segmentation_->points.size() );

    std::vector<float> point_residual;
    point_residual.resize(num_of_pts, -1.0);

    std::vector<int> point_is_used;
    point_is_used.resize(num_of_pts, 0);

    for (size_t i_point = 0; i_point < num_of_pts; i_point++)
    {
      point_residual[i_point] = normals_->points[i_point].curvature;
    }
    std::sort(point_residual.begin(), point_residual.end());
    int seed = 0;

    int segmented_pts_num = 0;
    while ( segmented_pts_num < num_of_pts )
    {
      int pts_in_segment;
      std::list<int> curr_segment;
      pts_in_segment = growRegion(seed, point_is_used, curr_segment);
      segments_.push_back(curr_segment);
      segmented_pts_num += pts_in_segment;

      for (int i_seed = seed + 1; i_seed < num_of_pts; i_seed++)
      {
        if (point_is_used[i_seed] == 0)
        {
          seed = i_seed;
          break;
        }
      }
    }

    return (static_cast<int>( segments_.size() ));
  }

  template <typename PointT> int
  RegionGrowing<PointT>::growRegion (int initial_seed, std::vector<int>& point_is_used, std::list<int>& out_new_segment)
  {
    std::queue<int> seeds;
    seeds.push(initial_seed);
    out_new_segment.push_back(initial_seed);

    point_is_used[initial_seed] = 1;

    while (!seeds.empty())
    {
      int curr_seed;
      curr_seed = seeds.front();
      seeds.pop();

      std::vector<int> nghbr_indices;
      std::vector<float> nghbr_distances;
      search_->nearestKSearch(curr_seed, neighbour_number_, nghbr_indices, nghbr_distances);

      for (int i_nghbr = 0; i_nghbr < nghbr_indices.size(); i_nghbr++)
      {
        int index = nghbr_indices[i_nghbr];
        if (point_is_used[index] == 1)
        {
          continue;
        }

        bool is_a_seed = false;
        bool belongs_to_segment = validatePoint(initial_seed, curr_seed, index, is_a_seed);

        if (belongs_to_segment == false)
        {
          continue;
        }

        point_is_used[index] = 1;
        out_new_segment.push_back(index);

        if (is_a_seed)
        {
          seeds.push(index);
        }
      }// next neighbour
    }// next seed

    return (static_cast<int>( out_new_segment.size() ));
  }

  template <typename PointT> bool
  RegionGrowing<PointT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
  {
    is_a_seed = true;

    float cosine_threshold = cos(theta_threshold_);
    Eigen::Map<Eigen::Vector3f> initial_point( (float*)cloud_for_segmentation_->points[point].data );
    Eigen::Map<Eigen::Vector3f> initial_normal( (float*)normals_->points[point].normal );

    //check the angle between normals
    if (smooth_mode_ == true)
    {
      Eigen::Map<Eigen::Vector3f> nghbr_normal( (float*)normals_->points[nghbr].normal );
      float dot_product = fabs( nghbr_normal.dot(initial_normal) );
      if (dot_product < cosine_threshold)
      {
        return (false);
      }
    }
    else
    {
      Eigen::Map<Eigen::Vector3f> nghbr_normal( (float*)normals_->points[nghbr].normal );
	  Eigen::Map<Eigen::Vector3f> initial_seed_normal( (float*)normals_->points[initial_seed].normal );
      float dot_product = fabs( nghbr_normal.dot(initial_seed_normal) );
      if (dot_product < cosine_threshold)
      {
        return (false);
      }
    }

    // check the curvature if needed
    if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
    {
      is_a_seed = false;
    }

    // check the residual if needed
    Eigen::Map<Eigen::Vector3f> nghbr_point( (float*)cloud_for_segmentation_->points[nghbr].data );
    float residual = fabs( initial_normal.dot(initial_point - nghbr_point) );
    if (residual_flag_ && residual > residual_threshold_)
    {
      is_a_seed = false;
    }

    return (true);
  }
}

#define PCL_INSTANTIATE_RegionGrowing(T) template class pcl::RegionGrowing<T>;

#endif
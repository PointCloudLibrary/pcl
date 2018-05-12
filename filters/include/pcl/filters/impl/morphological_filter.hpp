/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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

#ifndef PCL_FILTERS_IMPL_MORPHOLOGICAL_FILTER_H_
#define PCL_FILTERS_IMPL_MORPHOLOGICAL_FILTER_H_

#include <limits>
#include <vector>

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/octree/octree_search.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::applyMorphologicalOperator (const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                                 float resolution, const int morphological_operator,
                                 pcl::PointCloud<PointT> &cloud_out)
{
  if (cloud_in->empty ())
    return;

  pcl::copyPointCloud<PointT, PointT> (*cloud_in, cloud_out);

  pcl::octree::OctreePointCloudSearch<PointT> tree (resolution);

  tree.setInputCloud (cloud_in);
  tree.addPointsFromInputCloud ();

  float half_res = resolution / 2.0f;

  switch (morphological_operator)
  {
    case MORPH_DILATE:
    case MORPH_ERODE:
    {
      for (size_t p_idx = 0; p_idx < cloud_in->points.size (); ++p_idx)
      {
        Eigen::Vector3f bbox_min, bbox_max;
        std::vector<int> pt_indices;
        float minx = cloud_in->points[p_idx].x - half_res;
        float miny = cloud_in->points[p_idx].y - half_res;
        float minz = -std::numeric_limits<float>::max ();
        float maxx = cloud_in->points[p_idx].x + half_res;
        float maxy = cloud_in->points[p_idx].y + half_res;
        float maxz = std::numeric_limits<float>::max ();
        bbox_min = Eigen::Vector3f (minx, miny, minz);
        bbox_max = Eigen::Vector3f (maxx, maxy, maxz);
        tree.boxSearch (bbox_min, bbox_max, pt_indices);

        if (pt_indices.size () > 0)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D<PointT> (*cloud_in, pt_indices, min_pt, max_pt);

          switch (morphological_operator)
          {
            case MORPH_DILATE:
            {
              cloud_out.points[p_idx].z = max_pt.z ();
              break;
            }
            case MORPH_ERODE:
            {
              cloud_out.points[p_idx].z = min_pt.z ();
              break;
            }
          }
        }
      }
      break;
    }
    case MORPH_OPEN:
    case MORPH_CLOSE:
    {
      pcl::PointCloud<PointT> cloud_temp;

      pcl::copyPointCloud<PointT, PointT> (*cloud_in, cloud_temp);

      for (size_t p_idx = 0; p_idx < cloud_temp.points.size (); ++p_idx)
      {
        Eigen::Vector3f bbox_min, bbox_max;
        std::vector<int> pt_indices;
        float minx = cloud_temp.points[p_idx].x - half_res;
        float miny = cloud_temp.points[p_idx].y - half_res;
        float minz = -std::numeric_limits<float>::max ();
        float maxx = cloud_temp.points[p_idx].x + half_res;
        float maxy = cloud_temp.points[p_idx].y + half_res;
        float maxz = std::numeric_limits<float>::max ();
        bbox_min = Eigen::Vector3f (minx, miny, minz);
        bbox_max = Eigen::Vector3f (maxx, maxy, maxz);
        tree.boxSearch (bbox_min, bbox_max, pt_indices);

        if (pt_indices.size () > 0)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D<PointT> (cloud_temp, pt_indices, min_pt, max_pt);

          switch (morphological_operator)
          {
            case MORPH_OPEN:
            {
              cloud_out.points[p_idx].z = min_pt.z ();
              break;
            }
            case MORPH_CLOSE:
            {
              cloud_out.points[p_idx].z = max_pt.z ();
              break;
            }
          }
        }
      }

      cloud_temp.swap (cloud_out);

      for (size_t p_idx = 0; p_idx < cloud_temp.points.size (); ++p_idx)
      {
        Eigen::Vector3f bbox_min, bbox_max;
        std::vector<int> pt_indices;
        float minx = cloud_temp.points[p_idx].x - half_res;
        float miny = cloud_temp.points[p_idx].y - half_res;
        float minz = -std::numeric_limits<float>::max ();
        float maxx = cloud_temp.points[p_idx].x + half_res;
        float maxy = cloud_temp.points[p_idx].y + half_res;
        float maxz = std::numeric_limits<float>::max ();
        bbox_min = Eigen::Vector3f (minx, miny, minz);
        bbox_max = Eigen::Vector3f (maxx, maxy, maxz);
        tree.boxSearch (bbox_min, bbox_max, pt_indices);

        if (pt_indices.size () > 0)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D<PointT> (cloud_temp, pt_indices, min_pt, max_pt);

          switch (morphological_operator)
          {
            case MORPH_OPEN:
            default:
            {
              cloud_out.points[p_idx].z = max_pt.z ();
              break;
            }
            case MORPH_CLOSE:
            {
              cloud_out.points[p_idx].z = min_pt.z ();
              break;
            }
          }
        }
      }
      break;
    }
    default:
    {
      PCL_ERROR ("Morphological operator is not supported!\n");
      break;
    }
  }

  return;
}

#define PCL_INSTANTIATE_applyMorphologicalOperator(T) template PCL_EXPORTS void pcl::applyMorphologicalOperator<T> (const pcl::PointCloud<T>::ConstPtr &, float, const int, pcl::PointCloud<T> &);

#endif  //#ifndef PCL_FILTERS_IMPL_MORPHOLOGICAL_FILTER_H_


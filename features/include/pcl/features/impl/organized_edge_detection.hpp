/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_FEATURES_IMPL_ORGANIZED_EDGE_DETECTION_H_
#define PCL_FEATURES_IMPL_ORGANIZED_EDGE_DETECTION_H_

#include <pcl/2d/edge.h>
#include <pcl/2d/kernel.h>
#include <pcl/2d/convolution.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

/**
 *  Directions: 1 2 3
 *              0 x 4
 *              7 6 5
 * e.g. direction y means we came from pixel with label y to the center pixel x
 */
//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedEdgeBase<PointT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  pcl::Label invalid_pt;
  invalid_pt.label = unsigned (0);
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  
  extractEdges (labels);

  assignLabelIndices (labels, label_indices);
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedEdgeBase<PointT, PointLT>::assignLabelIndices (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  const unsigned invalid_label = unsigned (0);
  label_indices.resize (num_of_edgetype_);
  for (unsigned idx = 0; idx < input_->points.size (); idx++)
  {
    if (labels[idx].label != invalid_label)
    {
      for (int edge_type = 0; edge_type < num_of_edgetype_; edge_type++)
      {
        if ((labels[idx].label >> edge_type) & 1)
          label_indices[edge_type].indices.push_back (idx);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedEdgeBase<PointT, PointLT>::extractEdges (pcl::PointCloud<PointLT>& labels) const
{
  if ((detecting_edge_types_ & EDGELABEL_NAN_BOUNDARY) || (detecting_edge_types_ & EDGELABEL_OCCLUDING) || (detecting_edge_types_ & EDGELABEL_OCCLUDED))
  {
    // Fill lookup table for next points to visit
    const int num_of_ngbr = 8;
    Neighbor directions [num_of_ngbr] = {Neighbor(-1, 0, -1),
      Neighbor(-1, -1, -labels.width - 1), 
      Neighbor( 0, -1, -labels.width    ),
      Neighbor( 1, -1, -labels.width + 1),
      Neighbor( 1,  0,                 1),
      Neighbor( 1,  1,  labels.width + 1),
      Neighbor( 0,  1,  labels.width    ),
      Neighbor(-1,  1,  labels.width - 1)};

    for (int row = 1; row < int(input_->height) - 1; row++)
    {
      for (int col = 1; col < int(input_->width) - 1; col++)
      {
        int curr_idx = row*int(input_->width) + col;
        if (!pcl_isfinite (input_->points[curr_idx].z))
          continue;

        float curr_depth = fabsf (input_->points[curr_idx].z);

        // Calculate depth distances between current point and neighboring points
        std::vector<float> nghr_dist;
        nghr_dist.resize (8);
        bool found_invalid_neighbor = false;
        for (int d_idx = 0; d_idx < num_of_ngbr; d_idx++)
        {
          int nghr_idx = curr_idx + directions[d_idx].d_index;
          assert (nghr_idx >= 0 && nghr_idx < input_->points.size ());
          if (!pcl_isfinite (input_->points[nghr_idx].z))
          {
            found_invalid_neighbor = true;
            break;
          }
          nghr_dist[d_idx] = curr_depth - fabsf (input_->points[nghr_idx].z);
        }

        if (!found_invalid_neighbor)
        {
          // Every neighboring points are valid
          std::vector<float>::iterator min_itr = std::min_element (nghr_dist.begin (), nghr_dist.end ());
          std::vector<float>::iterator max_itr = std::max_element (nghr_dist.begin (), nghr_dist.end ());
          float nghr_dist_min = *min_itr;
          float nghr_dist_max = *max_itr;
          float dist_dominant = fabsf (nghr_dist_min) > fabsf (nghr_dist_max) ? nghr_dist_min : nghr_dist_max;
          if (fabsf (dist_dominant) > th_depth_discon_*fabsf (curr_depth))
          {
            // Found a depth discontinuity
            if (dist_dominant > 0.f)
            {
              if (detecting_edge_types_ & EDGELABEL_OCCLUDED)
                labels[curr_idx].label |= EDGELABEL_OCCLUDED;
            }
            else
            {
              if (detecting_edge_types_ & EDGELABEL_OCCLUDING)
                labels[curr_idx].label |= EDGELABEL_OCCLUDING;
            }
          }
        }
        else
        {
          // Some neighboring points are not valid (nan points)
          // Search for corresponding point across invalid points
          // Search direction is determined by nan point locations with respect to current point
          int dx = 0;
          int dy = 0;
          int num_of_invalid_pt = 0;
          for (int d_idx = 0; d_idx < num_of_ngbr; d_idx++)
          {
            int nghr_idx = curr_idx + directions[d_idx].d_index;
            assert (nghr_idx >= 0 && nghr_idx < input_->points.size ());
            if (!pcl_isfinite (input_->points[nghr_idx].z))
            {
              dx += directions[d_idx].d_x;
              dy += directions[d_idx].d_y;
              num_of_invalid_pt++;
            }
          }

          // Search directions
          assert (num_of_invalid_pt > 0);
          float f_dx = static_cast<float> (dx) / static_cast<float> (num_of_invalid_pt);
          float f_dy = static_cast<float> (dy) / static_cast<float> (num_of_invalid_pt);

          // Search for corresponding point across invalid points
          float corr_depth = std::numeric_limits<float>::quiet_NaN ();
          for (int s_idx = 1; s_idx < max_search_neighbors_; s_idx++)
          {
            int s_row = row + static_cast<int> (std::floor (f_dy*static_cast<float> (s_idx)));
            int s_col = col + static_cast<int> (std::floor (f_dx*static_cast<float> (s_idx)));

            if (s_row < 0 || s_row >= int(input_->height) || s_col < 0 || s_col >= int(input_->width))
              break;

            if (pcl_isfinite (input_->points[s_row*int(input_->width)+s_col].z))
            {
              corr_depth = fabsf (input_->points[s_row*int(input_->width)+s_col].z);
              break;
            }
          }

          if (!pcl_isnan (corr_depth))
          {
            // Found a corresponding point
            float dist = curr_depth - corr_depth;
            if (fabsf (dist) > th_depth_discon_*fabsf (curr_depth))
            {
              // Found a depth discontinuity
              if (dist > 0.f)
              {
                if (detecting_edge_types_ & EDGELABEL_OCCLUDED)
                  labels[curr_idx].label |= EDGELABEL_OCCLUDED;
              }
              else
              {
                if (detecting_edge_types_ & EDGELABEL_OCCLUDING)
                  labels[curr_idx].label |= EDGELABEL_OCCLUDING;
              }
            }
          } 
          else
          {
            // Not found a corresponding point, just nan boundary edge
            if (detecting_edge_types_ & EDGELABEL_NAN_BOUNDARY)
              labels[curr_idx].label |= EDGELABEL_NAN_BOUNDARY;
          }
        }
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedEdgeFromRGB<PointT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  pcl::Label invalid_pt;
  invalid_pt.label = unsigned (0);
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;

  OrganizedEdgeBase<PointT, PointLT>::extractEdges (labels);
  extractEdges (labels);

  this->assignLabelIndices (labels, label_indices);
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedEdgeFromRGB<PointT, PointLT>::extractEdges (pcl::PointCloud<PointLT>& labels) const
{
  if ((detecting_edge_types_ & EDGELABEL_RGB_CANNY))
  {
    pcl::PointCloud<PointXYZI>::Ptr gray (new pcl::PointCloud<PointXYZI>);
    gray->width = input_->width;
    gray->height = input_->height;
    gray->resize (input_->height*input_->width);

    for (size_t i = 0; i < input_->size (); ++i)
      (*gray)[i].intensity = float (((*input_)[i].r + (*input_)[i].g + (*input_)[i].b) / 3);

    pcl::PointCloud<pcl::PointXYZIEdge> img_edge_rgb;
    pcl::Edge<PointXYZI, pcl::PointXYZIEdge> edge;
    edge.setInputCloud (gray);
    edge.setHysteresisThresholdLow (th_rgb_canny_low_);
    edge.setHysteresisThresholdHigh (th_rgb_canny_high_);
    edge.detectEdgeCanny (img_edge_rgb);
    
    for (uint32_t row=0; row<labels.height; row++)
    {
      for (uint32_t col=0; col<labels.width; col++)
      {
        if (img_edge_rgb (col, row).magnitude == 255.f)
          labels[row * labels.width + col].label |= EDGELABEL_RGB_CANNY;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  pcl::Label invalid_pt;
  invalid_pt.label = unsigned (0);
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  
  OrganizedEdgeBase<PointT, PointLT>::extractEdges (labels);
  extractEdges (labels);

  this->assignLabelIndices (labels, label_indices);
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::extractEdges (pcl::PointCloud<PointLT>& labels) const
{
  if ((detecting_edge_types_ & EDGELABEL_HIGH_CURVATURE))
  {

    pcl::PointCloud<PointXYZI> nx, ny;
    nx.width = normals_->width;
    nx.height = normals_->height;
    nx.resize (normals_->height*normals_->width);

    ny.width = normals_->width;
    ny.height = normals_->height;
    ny.resize (normals_->height*normals_->width);

    for (uint32_t row=0; row<normals_->height; row++)
    {
      for (uint32_t col=0; col<normals_->width; col++)
      {
        nx (col, row).intensity = normals_->points[row*normals_->width + col].normal_x;
        ny (col, row).intensity = normals_->points[row*normals_->width + col].normal_y;
      }
    }

    pcl::PointCloud<pcl::PointXYZIEdge> img_edge;
    pcl::Edge<PointXYZI, pcl::PointXYZIEdge> edge;
    edge.setHysteresisThresholdLow (th_hc_canny_low_);
    edge.setHysteresisThresholdHigh (th_hc_canny_high_);
    edge.canny (nx, ny, img_edge);

    for (uint32_t row=0; row<labels.height; row++)
    {
      for (uint32_t col=0; col<labels.width; col++)
      {
        if (img_edge (col, row).magnitude == 255.f)
          labels[row * labels.width + col].label |= EDGELABEL_HIGH_CURVATURE;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromRGBNormals<PointT, PointNT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  pcl::Label invalid_pt;
  invalid_pt.label = unsigned (0);
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  
  OrganizedEdgeBase<PointT, PointLT>::extractEdges (labels);
  OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::extractEdges (labels);
  OrganizedEdgeFromRGB<PointT, PointLT>::extractEdges (labels);

  this->assignLabelIndices (labels, label_indices);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices, PointCloudNPtr& normals)
{
  pcl::Label invalid_pt;
  invalid_pt.label = unsigned(0);
  labels.points.resize(input_->points.size(), invalid_pt);
  labels.header = input_->header;
  labels.is_dense = input_->is_dense;
  labels.width = input_->width;
  labels.height = input_->height;

  // if slow mode is selected for depth discontinuities use the standard procedure with all edge label information (occluded, occluding, etc.)
  if (use_fast_depth_discontinuity_mode_ == false)
    OrganizedEdgeBase<PointT, PointLT>::extractEdges(labels);
  // compute depth edges (if fast mode is selected) and surface discontinuities
  extractEdges(labels, normals);

  if (return_label_indices_ == true)
    this->assignLabelIndices(labels, label_indices);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::extractEdges (pcl::PointCloud<PointLT>& labels, PointCloudNPtr& normals)
{
  PCLTimer tim;

  // setup coordinate images
//  pcl::Intensity zero_intensity;
//  zero_intensity.intensity = 0.f;
  pcl::PointCloud<pcl::Intensity>::Ptr x_image(new pcl::PointCloud<pcl::Intensity>);  // pcl::Intensity = type with one member (float) intensity
  pcl::PointCloud<pcl::Intensity>::Ptr y_image(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr z_image(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr x_dx(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr y_dy(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr z_dx(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr z_dy(new pcl::PointCloud<pcl::Intensity>);
  prepareImages(x_image, y_image, z_image, x_dx, y_dy, z_dx, z_dy);

  std::cout << "1: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();

  // intermediate edge data structure for computing depth and surface discontinuities
  pcl::Intensity8u zero_intensity_8u;
  zero_intensity_8u.intensity = (uint8_t)0;
  pcl::PointCloud<pcl::Intensity8u>::Ptr edge(new pcl::PointCloud<pcl::Intensity8u>(labels.width, labels.height, zero_intensity_8u));

  std::cout << "2: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();

  // depth discontinuities
  if (detecting_edge_types_ & EDGELABEL_OCCLUDING)
    computeDepthDiscontinuities(edge, z_image, z_dx, z_dy, labels);

  std::cout << "3: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();

  // surface discontinuities
  if (detecting_edge_types_ & EDGELABEL_HIGH_CURVATURE)
    computeSurfaceDiscontinuities(edge, x_dx, y_dy, z_image, z_dx, z_dy, normals);

  std::cout << "4: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();

  // copy edge data into point cloud labels
  setLabels(edge, labels);

  std::cout << "5: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::updateKernels()
{
  pcl::kernel<pcl::Intensity> kernel;

  gaussian_kernel_ = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
  kernel.kernel_size_ = edge_detection_config_.noise_reduction_kernel_size_;
  kernel.sigma_ = 0.3 * ( (kernel.kernel_size_ - 1) / 2 - 1) + 0.8;
  kernel.gaussianKernel(*gaussian_kernel_);

  const float kernel_scale = 1. / 8.;      // 1./8. (@ 3x3)   or   1./(6.*16.) (@ 5x5)
  sobel_kernel_x_3x3_ = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
  kernel.sobelKernelX(*sobel_kernel_x_3x3_);
  for (uint32_t v = 0; v < sobel_kernel_x_3x3_->height; ++v)
    for (uint32_t u = 0; u < sobel_kernel_x_3x3_->width; ++u)
      sobel_kernel_x_3x3_->at(u, v).intensity *= kernel_scale;

  sobel_kernel_y_3x3_ = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
  kernel.sobelKernelY(*sobel_kernel_y_3x3_);
  for (uint32_t v = 0; v < sobel_kernel_y_3x3_->height; ++v)
    for (uint32_t u = 0; u < sobel_kernel_y_3x3_->width; ++u)
      sobel_kernel_y_3x3_->at(u, v).intensity *= kernel_scale;
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::prepareImages(pcl::PointCloud<pcl::Intensity>::Ptr& x_image,
                                                                      pcl::PointCloud<pcl::Intensity>::Ptr& y_image,
                                                                      pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                                                                      pcl::PointCloud<pcl::Intensity>::Ptr& x_dx,
                                                                      pcl::PointCloud<pcl::Intensity>::Ptr& y_dy,
                                                                      pcl::PointCloud<pcl::Intensity>::Ptr& z_dx,
                                                                      pcl::PointCloud<pcl::Intensity>::Ptr& z_dy)
{
  PCLTimer tim;
  // setup images
  const int width = input_->width;
  const int height = input_->height;
  x_image->resize(width*height);
  x_image->width = width;
  x_image->height = height;
  y_image->resize(width*height);
  y_image->width = width;
  y_image->height = height;
  z_image->resize(width*height);
  z_image->width = width;
  z_image->height = height;
  for (unsigned int v = 0; v < height; ++v)
  {
    const size_t point_index = v * width;
    pcl::Intensity* x_ptr = & (x_image->points[point_index]);
    pcl::Intensity* y_ptr = & (y_image->points[point_index]);
    pcl::Intensity* z_ptr = & (z_image->points[point_index]);
    const PointT* point = & (input_->points[point_index]);
    for (unsigned int u = 0; u < width; ++u)
    {
      // point cloud matrix indices: row y, column x!
      if (point->z == point->z)  // test for NaN
      {
        x_ptr->intensity = point->x;
        y_ptr->intensity = point->y;
        z_ptr->intensity = point->z;
      }
      else
      {
        x_ptr->intensity = 0.f;
        y_ptr->intensity = 0.f;
        z_ptr->intensity = 0.f;
      }
      ++x_ptr;
      ++y_ptr;
      ++z_ptr;
      ++point;
    }
  }
  std::cout << "f1: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();

  // Sobel and smoothing
  // fast 3x3 kernel implementation
  filterSeparable33(x_image, x_dx, 0.125, 0.25, 0.125, -1, 0, 1);
  filterSeparable33(z_image, z_dx, 0.125, 0.25, 0.125, -1, 0, 1);
  filterSeparable33(y_image, y_dy, -1, 0, 1, 0.125, 0.25, 0.125);
  filterSeparable33(z_image, z_dy, -1, 0, 1, 0.125, 0.25, 0.125);
  // slower implementation for arbitrary separable kernels
//  std::vector<float> sobel_vertical(3, 0.125); sobel_vertical[1] = 0.25;
//  std::vector<float> sobel_horizontal(3, 0); sobel_horizontal[0] = -1; sobel_horizontal[2] = 1;
//  filterSeparable(x_image, x_dx, sobel_vertical, sobel_horizontal);
//  filterSeparable(z_image, z_dx, sobel_vertical, sobel_horizontal);
//  filterSeparable(y_image, y_dy, sobel_horizontal, sobel_vertical);
//  filterSeparable(z_image, z_dy, sobel_horizontal, sobel_vertical);

  // the PCL implementation is too slow, our quick and dirty filterSeparable33 is about 10 times faster (but less general)
//  pcl::Convolution<pcl::Intensity> convolution;
//  convolution.setKernel(*sobel_kernel_x_3x3_);
//  convolution.setInputCloud(x_image);
//  convolution.filter(*x_dx);
//  convolution.setInputCloud(z_image);
//  convolution.filter(*z_dx);
//  convolution.setKernel(*sobel_kernel_y_3x3_);
//  convolution.setInputCloud(y_image);
//  convolution.filter(*y_dy);
//  convolution.setInputCloud(z_image);
//  convolution.filter(*z_dy);

  std::cout << "f2: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();

  if (edge_detection_config_.noise_reduction_mode_ == EdgeDetectionConfig::GAUSSIAN)
  {
    if (edge_detection_config_.noise_reduction_kernel_size_ == 3)
    {
      // use fast implementation for 3x3 Gaussian kernel
      filterSeparable33(z_dx, z_dx, 0.25, 0.5, 0.25, 0.25, 0.5, 0.25);
      filterSeparable33(z_dy, z_dy, 0.25, 0.5, 0.25, 0.25, 0.5, 0.25);
    }
    else
    {
      // arbitrary kernel size
      const int k = edge_detection_config_.noise_reduction_kernel_size_;
      const double sigma = 0.3*((k-1)*0.5-1) + 0.8;
      std::vector<float> gaussian_vector(k);
      double sum = 0.;
      for (int i=0; i<k; ++i)
      {
        gaussian_vector[i] = exp(-(i-(k-1)/2)*(i-(k-1)/2)/(2*sigma*sigma));
        sum += gaussian_vector[i];
      }
      const double factor = 1./sum;
      for (int i=0; i<k; ++i)
        gaussian_vector[i] *= factor;
      filterSeparable(z_dx, z_dx, gaussian_vector, gaussian_vector);
      filterSeparable(z_dy, z_dy, gaussian_vector, gaussian_vector);
    }

//    pcl::PointCloud<pcl::Intensity>::Ptr temp(new pcl::PointCloud<pcl::Intensity>);
//    convolution.setKernel(*gaussian_kernel_);
//    pcl::copyPointCloud(*z_dx, *temp);
//    convolution.setInputCloud(temp);
//    convolution.filter(*z_dx);
//    pcl::copyPointCloud(*z_dy, *temp);
//    convolution.setInputCloud(temp);
//    convolution.filter(*z_dy);
  }
  std::cout << "f3: " << tim.getElapsedTimeInMilliSec() << std::endl;
  tim.start();
}


//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::filterSeparable33(pcl::PointCloud<pcl::Intensity>::Ptr& image, pcl::PointCloud<pcl::Intensity>::Ptr& result,
                                                               const float v1, const float v2, const float v3, const float h1, const float h2, const float h3)
{
  const int height = image->height;
  const int width = image->width;

  // first pass: vertical v1/v2/v3 convolution
  pcl::PointCloud<pcl::Intensity>::Ptr temp(new pcl::PointCloud<pcl::Intensity>);
  temp->resize(height*width);
  temp->width = width;
  temp->height = height;
  pcl::Intensity* ptr = &(temp->points[0]);
  pcl::Intensity* ptr1 = &(image->points[0]);
  pcl::Intensity* ptr2 = &(image->points[0]);
  pcl::Intensity* ptr3 = &(image->points[width]);
  for (unsigned int u = 0; u < width; ++u, ++ptr, ++ptr2, ++ptr3)
    ptr->intensity = v2*ptr2->intensity + v3*ptr3->intensity;
  for (unsigned int v=1; v < height-1; ++v)
    for (unsigned int u = 0; u < width; ++u, ++ptr, ++ptr1, ++ptr2, ++ptr3)
      ptr->intensity = v1*ptr1->intensity + v2*ptr2->intensity + v3*ptr3->intensity;
  for (unsigned int u = 0; u < width; ++u, ++ptr, ++ptr1, ++ptr2)
    ptr->intensity = v1*ptr1->intensity + v2*ptr2->intensity;

  // second pass: horizontal h1/h2/h3 convolution
  result->resize(height*width);
  result->width = width;
  result->height = height;
  ptr1 = &(temp->points[0]);
  ptr2 = &(temp->points[1]);
  ptr3 = &(temp->points[2]);
  ptr = &(result->points[0]);
  for (unsigned int v=0; v < height; ++v)
  {
    ptr->intensity = h2*ptr1->intensity + h3*ptr2->intensity;   // pointers 0 and 1 are already preset one field in advance, hence the pointer name does not match with h2 and h3 (but the computations are correct)
    ++ptr;
    for (unsigned int u = 1; u < width-1; ++u, ++ptr, ++ptr1, ++ptr2, ++ptr3)
      ptr->intensity = h1*ptr1->intensity + h2*ptr2->intensity + h3*ptr3->intensity;
    ptr->intensity = h1*ptr1->intensity + h2*ptr2->intensity;
    ++ptr;
    ptr1+=2; ptr2+=2; ptr3+=2;
  }
}

template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::filterSeparable(pcl::PointCloud<pcl::Intensity>::Ptr& image, pcl::PointCloud<pcl::Intensity>::Ptr& result,
                                                                        const std::vector<float> vertical_coefficients, const std::vector<float> horizontal_coefficients)
{
  const int height = image->height;
  const int width = image->width;

  // first pass: vertical convolution
  // --------------------------------
  pcl::PointCloud<pcl::Intensity>::Ptr temp(new pcl::PointCloud<pcl::Intensity>);
  temp->resize(height*width);
  temp->width = width;
  temp->height = height;
  const int kernel_size_v = (int)vertical_coefficients.size();
  const int kernel_min_dv = -kernel_size_v/2;  // this is the kernel offset
  int base_index = 0;
  for (unsigned int v=0; v < height; ++v)
  {
    for (unsigned int u = 0; u < width; ++u, ++base_index)
    {
      float value = 0.f;
      int dv=kernel_min_dv;
      for (int i=0; i<kernel_size_v; ++i, ++dv)
      {
        if (v+dv >= 0 && v+dv<height)
          value += vertical_coefficients[i] * image->points[base_index+dv*width].intensity;
      }
      temp->points[base_index].intensity = value;
    }
  }

  // second pass: horizontal convolution
  // -----------------------------------
  result->resize(height*width);
  result->width = width;
  result->height = height;
  const int kernel_size_h = (int)horizontal_coefficients.size();
  const int kernel_min_dh = -kernel_size_h/2;  // this is the kernel offset
  base_index = 0;
  for (unsigned int v=0; v < height; ++v)
  {
    for (unsigned int u = 0; u < width; ++u, ++base_index)
    {
      float value = 0.f;
      int du=kernel_min_dh;
      for (int i=0; i<kernel_size_h; ++i, ++du)
      {
        if (u+du >= 0 && u+du<width)
          value += horizontal_coefficients[i] * temp->points[base_index+du].intensity;
      }
      result->points[base_index].intensity = value;
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::computeDepthDiscontinuities(pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                                                                                    const pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                                                                                    const pcl::PointCloud<pcl::Intensity>::Ptr& z_dx,
                                                                                    const pcl::PointCloud<pcl::Intensity>::Ptr& z_dy,
                                                                                    const pcl::PointCloud<PointLT>& labels)
{
  if (use_fast_depth_discontinuity_mode_ == true)
  {
    // if not computed in the standard way, compute depth edges in a fast way, without distinguishing between occluding, occluded, NaN, etc.
    pcl::Intensity* z_image_ptr = &(z_image->points[0]);
    pcl::Intensity* z_dx_ptr = &(z_dx->points[0]);
    pcl::Intensity* z_dy_ptr = &(z_dy->points[0]);
    const float depth_factor = edge_detection_config_.depth_step_factor_;
    for (int v = 0; v < z_dx->height; ++v)
    {
      for (int u = 0; u < z_dx->width; ++u, ++z_image_ptr, ++z_dx_ptr, ++z_dy_ptr)
      {
        float depth = z_image_ptr->intensity; //z_image->at(u, v).intensity;
        if (depth == 0.f)
          continue;
        const float edge_threshold = std::max(0.0f, depth_factor * depth * depth);
        const float z_dx_val = z_dx_ptr->intensity; //z_dx->at(u, v).intensity;
        const float z_dy_val = z_dy_ptr->intensity; //z_dy->at(u, v).intensity;
        if (z_dx_val <= -edge_threshold || z_dx_val >= edge_threshold || z_dy_val <= -edge_threshold || z_dy_val >= edge_threshold)
          edge->at(u, v).intensity = (uint8_t) std::min<float>(255.f, 50.f * (1. + sqrt(z_dx_val * z_dx_val + z_dy_val * z_dy_val)));  // store a proportional measure for the edge strength
      }
    }
    nonMaximumSuppression(edge, z_dx, z_dy);
  }
  else
  {
    // copy pre-computed depth edges into edge image
    const unsigned invalid_label = unsigned(0);
    for (unsigned int v = 0; v < labels.height; ++v)
    {
      const size_t point_index = v * input_->width;
      pcl::Intensity8u* dst = & (edge->points[point_index]);

      const PointLT* src = & (labels.points[point_index]);
      for (unsigned int u = 0; u < labels.width; u++)
      {
        if (src->label != invalid_label)
          dst->intensity = (uint8_t)254;
        ++src;
        ++dst;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::computeSurfaceDiscontinuities(pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, const pcl::PointCloud<pcl::Intensity>::Ptr& x_dx,
                                                                                      const pcl::PointCloud<pcl::Intensity>::Ptr& y_dy, const pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                                                                                      const pcl::PointCloud<pcl::Intensity>::Ptr& z_dx, const pcl::PointCloud<pcl::Intensity>::Ptr& z_dy,
                                                                                      PointCloudNPtr& normals)
{
  PCLTimer tim;
  // surface discontinuities
  const int min_line_width = edge_detection_config_.min_scan_line_width_;
  const int max_line_width = edge_detection_config_.max_scan_line_width_;
  const int max_v = z_dx->height - max_line_width - 2;
  const int max_u = z_dx->width - max_line_width - 2;
  const float min_detectable_edge_angle = edge_detection_config_.min_detectable_edge_angle_;  // minimum angle between two planes to consider their intersection an edge, measured in [° degree]
  const float scan_line_model_m = edge_detection_config_.scan_line_model_m_;
  const float scan_line_model_n = edge_detection_config_.scan_line_model_n_;

  // x direction scan lines
  pcl::PointCloud<pcl::Intensity>::Ptr x_dx_integral_x(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr z_dx_integral_x(new pcl::PointCloud<pcl::Intensity>);
  computeIntegralImageX(x_dx, x_dx_integral_x, z_dx, z_dx_integral_x);
  std::vector< std::pair<int,int> > distance_map_horizontal;
  computeEdgeDistanceMapHorizontal(edge, distance_map_horizontal);
  std::cout << "surface1: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
  tim.start();
  const int width_xscan = z_image->width;
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int v = max_line_width+1; v < max_v; ++v)
  {
    int scan_line_width = 10; // width of scan line left or right of a query pixel, measured in [px]
    int last_line_width = scan_line_width;
    int edge_start_index = -1;
    float max_edge_strength = 0;
    for (int u = max_line_width+1; u < max_u; ++u)
    {
      const int point_index = v*width_xscan + u;
      const float depth = z_image->points[point_index].intensity;
      if (depth == 0.f)
        continue;

      // depth dependent scan line width for slope computation (e.g. 1px width per 0.10m depth)
      scan_line_width = std::min(int(scan_line_model_m*depth+scan_line_model_n), max_line_width);
      if (scan_line_width <= min_line_width)
        scan_line_width = last_line_width;
      else
        last_line_width = scan_line_width;
      int scan_line_width_left = scan_line_width;
      int scan_line_width_right = scan_line_width;
      bool edge_hit = false;
      if (edge_detection_config_.use_adaptive_scan_line_ == true)
        edge_hit = !adaptScanLine(scan_line_width_left, scan_line_width_right, distance_map_horizontal[point_index], min_line_width);
      else
        edge_hit = (distance_map_horizontal[point_index].first <= scan_line_width_left+2 || distance_map_horizontal[point_index].second <= scan_line_width_right); // do not compute surface edges if a depth discontinuity is on the scan line
      if (edge_hit == true)
      {
        // not enough support for solid slope estimation
        edge_start_index = -1;
        max_edge_strength = 0.f;
        continue;
      }

      // get average differences in x and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
      // remark: the indexing of the integral image here differs from the OpenCV definition (here: the value of a cell is included in the sum of the integral image's cell)
      const float avg_dx_l = x_dx_integral_x->points[point_index-1].intensity                     - x_dx_integral_x->points[point_index-scan_line_width_left].intensity;
      const float avg_dx_r = x_dx_integral_x->points[point_index+scan_line_width_right].intensity - x_dx_integral_x->points[point_index+1].intensity;
      const float avg_dz_l = z_dx_integral_x->points[point_index-1].intensity                     - z_dx_integral_x->points[point_index-scan_line_width_left].intensity;
      const float avg_dz_r = z_dx_integral_x->points[point_index+scan_line_width_right].intensity - z_dx_integral_x->points[point_index+1].intensity;

      // estimate angle difference
      const float alpha_left = fast_atan2f_1(-avg_dz_l, -avg_dx_l);
      const float alpha_right = fast_atan2f_1(avg_dz_r, avg_dx_r);
      const float diff = fabs(alpha_left - alpha_right);
      if (diff!=0 && (diff < (180.-min_detectable_edge_angle) * 1./180. * PI_DOUBLE || diff > (180.+min_detectable_edge_angle) * 1./180. * PI_DOUBLE))
      {
        // found an edge, i.e. the directions of left and right surface lines differ enough
        if (edge_start_index == -1)
          edge_start_index = u;
        const float dist = fabs(PI_DOUBLE - diff);
        if (dist > max_edge_strength)
        {
          max_edge_strength = dist;
          edge_start_index = u;
        }
      }
      else
      {
        if (edge_start_index != -1)
        {
          // just after having found an edge before -> label edge in edge image
          edge->at(edge_start_index, v).intensity = (uint8_t)255;
          edge_start_index = -1;
          max_edge_strength = 0;
        }
      }
    }
  }
  std::cout << "surfaceX: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
  tim.start();

  // y direction scan lines
  pcl::PointCloud<pcl::Intensity>::Ptr y_dy_t(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr z_dy_t(new pcl::PointCloud<pcl::Intensity>);
  transposeImage(y_dy, y_dy_t); // transpose for better memory access latency (blocks with simultaneous and successive accesses will already reside in cache)
  transposeImage(z_dy, z_dy_t);
  pcl::PointCloud<pcl::Intensity>::Ptr y_dy_integral_y(new pcl::PointCloud<pcl::Intensity>);
  pcl::PointCloud<pcl::Intensity>::Ptr z_dy_integral_y(new pcl::PointCloud<pcl::Intensity>);
  computeIntegralImageX(y_dy_t, y_dy_integral_y, z_dy_t, z_dy_integral_y);
  std::vector< std::pair<int,int> > distance_map_vertical;
  computeEdgeDistanceMapVertical(edge, distance_map_vertical);
  std::cout << "surface2: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
  tim.start();
  const int width_yscan = z_dy_t->width;
  const int max_uy = width_yscan - max_line_width - 2;
  const int max_vy = z_dy_t->height - max_line_width - 2;
#ifdef _OPENMP
#pragma omp parallel for //num_threads(2)
#endif
  for (int v = max_line_width+1; v < max_vy; ++v)   // this is the original u coordinate, but the y-data structures are transposed for memory access efficiency
  {
    int scan_line_width = 10; // width of scan line left or right of a query pixel, measured in [px]
    int last_line_width = scan_line_width;
    int edge_start_index = -1;
    float max_edge_strength = 0;
    for (int u = max_line_width+1; u < max_uy; ++u)
    {
      const int point_index = u*z_image->width + v; // indices swapped because the data matrices (y_dy_t, z_dy_t) are transposed for faster access in memory
      const float depth = z_image->points[point_index].intensity;
      if (depth==0.f)
        continue;

      // depth dependent scan line width for slope computation (e.g. 1px width per 0.10m depth)
      scan_line_width = std::min(int(scan_line_model_m*depth+scan_line_model_n), max_line_width);
      if (scan_line_width <= min_line_width)
        scan_line_width = last_line_width;
      else
        last_line_width = scan_line_width;
      int scan_line_height_upper = scan_line_width;
      int scan_line_height_lower = scan_line_width;
      bool edge_hit = false;
      if (edge_detection_config_.use_adaptive_scan_line_ == true)
        edge_hit = !adaptScanLine(scan_line_height_upper, scan_line_height_lower, distance_map_vertical[point_index], min_line_width);
      else
        edge_hit = (distance_map_vertical[point_index].first <= scan_line_height_upper+2 || distance_map_vertical[point_index].second <= scan_line_height_lower); // do not compute surface edges if a depth discontinuity is on the scan line
      if (edge_hit == true)
      {
        // not enough support for solid slope estimation
        edge_start_index = -1;
        max_edge_strength = 0.f;
        continue;
      }

      // get average differences in y and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
      // remark: the indexing of the integral image here differs from the OpenCV definition (here: the value of a cell is included in the sum of the integral image's cell)
      const int base_index = v*width_yscan + u;
      const float avg_dy_u = y_dy_integral_y->points[base_index-1].intensity                      - y_dy_integral_y->points[base_index-scan_line_height_upper].intensity;
      const float avg_dy_b = y_dy_integral_y->points[base_index+scan_line_height_lower].intensity - y_dy_integral_y->points[base_index+1].intensity;
      const float avg_dz_u = z_dy_integral_y->points[base_index-1].intensity                      - z_dy_integral_y->points[base_index-scan_line_height_upper].intensity;
      const float avg_dz_b = z_dy_integral_y->points[base_index+scan_line_height_lower].intensity - z_dy_integral_y->points[base_index+1].intensity;

      // estimate angle difference
      const float alpha_above = fast_atan2f_1(-avg_dz_u, -avg_dy_u);
      const float alpha_below = fast_atan2f_1(avg_dz_b, avg_dy_b);
      const float diff = fabs(alpha_above - alpha_below);
      if (diff!=0 && (diff < (180.-min_detectable_edge_angle) * 1./180. * PI_DOUBLE || diff > (180.+min_detectable_edge_angle) * 1./180. * PI_DOUBLE))
      {
        // found an edge, i.e. the directions of upper and lower surface lines differ enough
        if (edge_start_index == -1)
          edge_start_index = u;
        const float dist = fabs(PI_DOUBLE - diff);
        if (dist > max_edge_strength)
        {
          max_edge_strength = dist;
          edge_start_index = u;
        }
      }
      else
      {
        if (edge_start_index != -1)
        {
          // just after having found an edge before -> label edge in edge image
          edge->at(v, edge_start_index).intensity = (uint8_t)255;
          edge_start_index = -1;
          max_edge_strength = 0;
        }
      }
    }
  }
  std::cout << "surfaceY: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
  tim.start();

  // close 45 degree edges with an additional edge pixel for better neighborhood selection during normal estimation
  const int width = edge->width;
  for (int v = max_line_width; v < edge->height - max_line_width; ++v)
  {
    int base_index = v*width + max_line_width; // attention: needs to start with same u-offset as u in the following loop
    for (int u = max_line_width; u < width - max_line_width; ++u, ++base_index)
    {
      if (edge->points[base_index].intensity == 0)
      {
        if (edge->points[base_index+1].intensity != 0 && edge->points[base_index+width].intensity != 0 && edge->points[base_index+width+1].intensity == 0)
          edge->points[base_index].intensity = edge->points[base_index+1].intensity;
      }
      else
      {
        if (edge->points[base_index+1].intensity == 0 && edge->points[base_index+width].intensity == 0 && edge->points[base_index+width+1].intensity != 0)
          edge->points[base_index+1].intensity = edge->points[base_index+width+1].intensity;
      }
    }
  }
  std::cout << "surface_refine: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
  tim.start();

  // optionally: compute edge-aware normals exploiting the already computed structures
  if (normals != 0)
  {
    computeEdgeDistanceMapHorizontal(edge, distance_map_horizontal);
    computeEdgeDistanceMapVertical(edge, distance_map_vertical);
    normals->resize(input_->points.size());
    normals->header = input_->header;
    normals->is_dense = input_->is_dense;
    normals->height = input_->height;
    normals->width = input_->width;
    const int width = normals->width;
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
#ifdef _OPENMP
#pragma omp parallel for //num_threads(2)
#endif
    for (int v = max_line_width+1; v < max_v; ++v)
    {
      int scan_line_width = 10; // width of scan line left or right of a query pixel, measured in [px]
      int last_line_width = scan_line_width;
      for (int u = max_line_width+1; u < max_u; ++u)
      {
        const int point_index = v*width + u;
        const float depth = z_image->points[point_index].intensity;
        if (depth == 0.f || edge->points[point_index].intensity != 0)
        {
          normals->points[point_index].getNormalVector3fMap().setConstant(bad_point);
          continue;
        }

        // depth dependent scan line width for slope computation (e.g. 1px width per 0.10m depth)
        scan_line_width = std::min(int(scan_line_model_m*depth+scan_line_model_n), max_line_width);
        if (scan_line_width <= min_line_width)
          scan_line_width = last_line_width;
        else
          last_line_width = scan_line_width;
        int scan_line_width_left = scan_line_width;
        int scan_line_width_right = scan_line_width;
        int scan_line_height_upper = scan_line_width;
        int scan_line_height_lower = scan_line_width;
        bool edge_hit = false;
        if (edge_detection_config_.use_adaptive_scan_line_ == true)
          edge_hit = !adaptScanLineNormal(scan_line_width_left, scan_line_width_right, distance_map_horizontal[point_index], min_line_width) ||
                     !adaptScanLineNormal(scan_line_height_upper, scan_line_height_lower, distance_map_vertical[point_index], min_line_width);
        else
          edge_hit = distance_map_horizontal[point_index].first <= scan_line_width_left+2 || distance_map_horizontal[point_index].second <= scan_line_width_right ||
                     distance_map_vertical[point_index].first <= scan_line_height_upper+2 || distance_map_vertical[point_index].second <= scan_line_height_lower; // do not compute surface edges if a depth discontinuity is on the scan line
        if (edge_hit == true)
        {
          // not enough support for solid normal estimation
          normals->points[point_index].getNormalVector3fMap().setConstant(bad_point);
          continue;
        }

        // get average differences in x, y and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
        // remark: the indexing of the integral image here differs from the OpenCV definition (here: the value of a cell is included in the sum of the integral image's cell)
        const int ybase_index = u*width_yscan + v;
        const float avg_dx1 = x_dx_integral_x->points[point_index+scan_line_width_right].intensity  - x_dx_integral_x->points[point_index-scan_line_width_left].intensity;
        const float avg_dz1 = z_dx_integral_x->points[point_index+scan_line_width_right].intensity  - z_dx_integral_x->points[point_index-scan_line_width_left].intensity;
        const float avg_dy2 = y_dy_integral_y->points[ybase_index+scan_line_height_lower].intensity - y_dy_integral_y->points[ybase_index-scan_line_height_upper].intensity;
        const float avg_dz2 = z_dy_integral_y->points[ybase_index+scan_line_height_lower].intensity - z_dy_integral_y->points[ybase_index-scan_line_height_upper].intensity;

        const Eigen::Vector3f v1(avg_dx1, 0, avg_dz1);
        const Eigen::Vector3f v2(0, avg_dy2, avg_dz2);
        Eigen::Vector3f n = (v2.cross(v1)).normalized();
        pcl::flipNormalTowardsViewpoint<PointT>(input_->points[point_index], input_->sensor_origin_(0), input_->sensor_origin_(1), input_->sensor_origin_(2), n(0), n(1), n(2));
        normals->points[point_index].getNormalVector3fMap() = n;
      }
    }
  }
  std::cout << "surface_normals: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::nonMaximumSuppression(pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                                                                              const pcl::PointCloud<pcl::Intensity>::Ptr& dx,
                                                                              const pcl::PointCloud<pcl::Intensity>::Ptr& dy)
{
  const int height = edge->height;
  const int width = edge->width;
  std::vector<std::pair<uint32_t, uint32_t> > setToZerosSet;
  for (uint32_t v = 1; v < height - 1; ++v)
  {
    int index = v*width + 1;
    for (uint32_t u = 1; u < width - 1; ++u, ++index)
    {
      if (edge->points[index].intensity == 0)
        continue;
      double x = dx->points[index].intensity;
      double y = dy->points[index].intensity;
      if (x == 0 && y == 0)
        continue;
      const double mag = sqrt(x * x + y * y);
      x = floor(x / mag + 0.5);
      y = floor(y / mag + 0.5);
      uint8_t& edge_val = edge->points[index].intensity;
      if (edge_val >= edge->at(u + x, v + y).intensity && edge_val >= edge->at(u - x, v - y).intensity)
        edge_val = (uint8_t) 254;   // local maximum
      else
        setToZerosSet.push_back(std::pair<uint32_t, uint32_t>(u, v));    // later set to zero -> no (maximal) edge
    }
  }
  for (size_t i = 0; i < setToZerosSet.size(); ++i)
    edge->at(setToZerosSet[i].first, setToZerosSet[i].second).intensity = (uint8_t)0;
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::computeIntegralImageX(const pcl::PointCloud<pcl::Intensity>::Ptr& src_x, pcl::PointCloud<pcl::Intensity>::Ptr& dst_x,
                                                                              const pcl::PointCloud<pcl::Intensity>::Ptr& src_z, pcl::PointCloud<pcl::Intensity>::Ptr& dst_z)
{
  dst_x->resize(src_x->height*src_x->width);
  dst_x->width = src_x->width;
  dst_x->height = src_x->height;
  dst_z->resize(src_x->height*src_x->width);
  dst_z->width = src_x->width;
  dst_z->height = src_x->height;
  for (int v=0; v<src_x->height; ++v)
  {
    const size_t point_index = v * src_x->width;
    const pcl::Intensity* src_x_ptr = & (src_x->points[point_index]);
    pcl::Intensity* dst_x_ptr = & (dst_x->points[point_index]);
    const pcl::Intensity* src_z_ptr = & (src_z->points[point_index]);
    pcl::Intensity* dst_z_ptr = & (dst_z->points[point_index]);
    float sum_x = 0.f;
    float sum_z = 0.f;
    for (int u=0; u<src_x->width; ++u)
    {
      sum_x += src_x_ptr->intensity;
      dst_x_ptr->intensity = sum_x;
      src_x_ptr++;
      dst_x_ptr++;
      sum_z += src_z_ptr->intensity;
      dst_z_ptr->intensity = sum_z;
      src_z_ptr++;
      dst_z_ptr++;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::computeEdgeDistanceMapHorizontal(const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, std::vector< std::pair<int,int> >& distance_map)
{
  const int height = edge->height;
  const int width = edge->width;
  distance_map.resize(height*width);
  for (int v=0; v<height; ++v)
  {
    // distance to next edge left of query pixel
    int index = v*width;
    distance_map[index].first = 0;
    ++index;
    pcl::Intensity8u* e_ptr = &(edge->points[index]);
    for (int u=1; u<width; ++u, ++index)
    {
      //if (edge->at(u,v).intensity != 0)
      if (e_ptr->intensity != 0)
        distance_map[index].first = 0;
      else
        distance_map[index].first = distance_map[index-1].first + 1;
      ++e_ptr;
    }
    // distance to next edge right of query pixel
    index = (v+1)*width - 1;
    distance_map[index].second = 0;
    --index;
    e_ptr = &(edge->points[index]);
    for (int u=width-2; u>=0; --u, --index)
    {
      //if (edge->at(u,v).intensity != 0)
      if (e_ptr->intensity != 0)
        distance_map[index].second = 0;
      else
        distance_map[index].second = distance_map[index+1].second + 1;
      --e_ptr;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::computeEdgeDistanceMapVertical(const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, std::vector< std::pair<int,int> >& distance_map)
{
  const int height = edge->height;
  const int width = edge->width;
  distance_map.resize(height * width);

  // distance to next edge above query pixel
  std::vector< std::pair<int,int> >::iterator d_ptr = distance_map.begin();
  std::vector< std::pair<int,int> >::iterator d_prev_ptr = distance_map.begin();
  for (int u = 0; u < width; ++u, ++d_ptr)
    d_ptr->first = 0;
  pcl::Intensity8u* e_ptr = &(edge->points[width]);
  for (int v = 1; v < height; ++v)
  {
    for (int u = 0; u < width; ++u, ++d_ptr, ++d_prev_ptr, ++e_ptr)
    {
      //if (edge->at(u, v).intensity != 0)
      if (e_ptr->intensity != 0)
        //distance_map->at(u, v).x = 0;
        d_ptr->first = 0;
      else
        //distance_map->at(u, v).x = distance_map->at(u, v - 1).x + 1;
        d_ptr->first = d_prev_ptr->first + 1;
    }
  }
  // distance to next edge below query pixel
  d_ptr = distance_map.end()-1;
  d_prev_ptr = distance_map.end()-1;
  for (int u = width-1; u >= 0; --u, --d_ptr)
    d_ptr->second = 0;
  e_ptr = &(edge->points[(height-1)*width-1]);
  for (int v = height - 2; v >= 0; --v)
  {
    for (int u = width-1; u >= 0; --u, --d_ptr, --d_prev_ptr, --e_ptr)
    {
      //if (edge->at(u, v).intensity != 0)
      if (e_ptr->intensity != 0)
        //distance_map->at(u, v).y = 0;
        d_ptr->second = 0;
      else
        //distance_map->at(u, v).y = distance_map->at(u, v + 1).y + 1;
        d_ptr->second = d_prev_ptr->second + 1;
    }
  }

//  this is the version without pointers, which is around 4 times slower than with pointers
//  // distance to next edge above query pixel
//  for (int u = 0; u < edge->width; ++u)
//    distance_map->at(u, 0).x = 0;
//  for (int v = 1; v < edge->height; ++v)
//  {
//    for (int u = 0; u < edge->width; ++u)
//    {
//      if (edge->at(u, v).intensity != 0)
//        distance_map->at(u, v).x = 0;
//      else
//        distance_map->at(u, v).x = distance_map->at(u, v - 1).x + 1;
//    }
//  }
//  // distance to next edge below query pixel
//  for (int u = 0; u < edge->width; ++u)
//    distance_map->at(u, edge->height - 1).y = 0;
//  for (int v = edge->height - 2; v >= 0; --v)
//  {
//    for (int u = 0; u < edge->width; ++u)
//    {
//      if (edge->at(u, v).intensity != 0)
//        distance_map->at(u, v).y = 0;
//      else
//        distance_map->at(u, v).y = distance_map->at(u, v + 1).y + 1;
//    }
//  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> inline bool
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::adaptScanLine(int& scan_line_length_1, int& scan_line_length_2, const std::pair<int,int>& distance_to_edge, const int min_scan_line_length)
{
  const int min_scan_line_width_fraction_from_max = 3;
  const int min_distance_to_depth_edge = 2;
  const int max_1 = scan_line_length_1;
  const int max_2 = scan_line_length_2;
  scan_line_length_1 = std::min(scan_line_length_1, distance_to_edge.first-1-min_distance_to_depth_edge);
  scan_line_length_2 = std::min(scan_line_length_2, distance_to_edge.second-1-min_distance_to_depth_edge);

  if ( (scan_line_length_1 < min_scan_line_length || min_scan_line_width_fraction_from_max * scan_line_length_1 < max_1)
      || (scan_line_length_2 < min_scan_line_length || min_scan_line_width_fraction_from_max * scan_line_length_2 < max_2))
    return false;
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> inline bool
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::adaptScanLineNormal(int& scan_line_length_1, int& scan_line_length_2, const std::pair<int,int>& distance_to_edge,
                                                                            const int min_scan_line_length)
{
  const int min_distance_to_depth_edge = 2;
  scan_line_length_1 = std::min(scan_line_length_1, distance_to_edge.first-1-min_distance_to_depth_edge);
  scan_line_length_2 = std::min(scan_line_length_2, distance_to_edge.second-1-min_distance_to_depth_edge);

  if ((scan_line_length_1+scan_line_length_2+1)<min_scan_line_length)
    return false;
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::transposeImage(const pcl::PointCloud<pcl::Intensity>::Ptr& src,
                                                                       pcl::PointCloud<pcl::Intensity>::Ptr& dst)
{
  const int height = src->height;
  const int width = src->width;
  dst->resize(height * width);
  dst->height = width;
  dst->width = height;

  int dst_index = 0;
  for (unsigned int v = 0; v < width; ++v)
  {
    int src_index = v;
    for (unsigned int u = 0; u < height; ++u, ++dst_index, src_index+=width)
      dst->points[dst_index] = src->points[src_index];
  }
//  for (unsigned int v = 0; v < width; ++v)  // no measurable speedup with this version
//  {
//    pcl::Intensity* src_ptr = & (src->points[v]);
//    pcl::Intensity* dst_ptr = & (dst->points[v * dst->width]);
//    for (unsigned int u = 0; u < height; ++u)
//    {
//      *dst_ptr = *src_ptr;
//      src_ptr += src->width;
//      ++dst_ptr;
//    }
//  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> float
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::fast_atan2f_1(float y, float x)
{
  if (x == 0.0f)
  {
    if (y > 0.0f) return PI_BY_2_FLOAT;
    if (y == 0.0f) return 0.0f;
    return -PI_BY_2_FLOAT;
  }
  float atan;
  float z = y/x;
  if (fabsf(z) < 1.0f)
  {
    atan = z/(1.0f + 0.28f*z*z);
    if (x < 0.0f)
    {
      if (y < 0.0f) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PI_BY_2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::setLabels(const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, pcl::PointCloud<PointLT>& labels)
{
  const int width = edge->width;
  const int height = edge->height;
  if (use_fast_depth_discontinuity_mode_ == true)
  {
    pcl::Intensity8u* e_ptr = &(edge->points[0]);
    PointLT* l_ptr = &(labels[0]);
    for (uint32_t v = 0; v < height; ++v)
    {
      for (uint32_t u = 0; u < width; ++u, ++l_ptr, ++e_ptr)
      {
        const uint8_t val = e_ptr->intensity;
        if (val == 254)
          l_ptr->label |= EDGELABEL_OCCLUDING;
        else if (val == 255)
          l_ptr->label |= EDGELABEL_HIGH_CURVATURE;
      }
    }
  }
  else
  {
    pcl::Intensity8u* e_ptr = &(edge->points[0]);
    PointLT* l_ptr = &(labels[0]);
    for (uint32_t v = 0; v < height; ++v)
      for (uint32_t u = 0; u < width; ++u, ++l_ptr, ++e_ptr)
        if (e_ptr->intensity == 255)
          l_ptr->label |= EDGELABEL_HIGH_CURVATURE;
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedEdgeFromRGBPoints<PointT, PointNT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices,
                                                                    PointCloudNPtr& normals)
{
  pcl::Label invalid_pt;
  invalid_pt.label = unsigned (0);
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;

  // if slow mode is selected for depth discontinuities use the standard procedure with all edge label information (occluded, occluding, etc.)
  if (use_fast_depth_discontinuity_mode_ == false)
    OrganizedEdgeBase<PointT, PointLT>::extractEdges(labels);
  // compute depth edges (if fast mode is selected) and surface discontinuities
  OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::extractEdges(labels, normals);
  // compute RGB edges with Canny edge
  OrganizedEdgeFromRGB<PointT, PointLT>::extractEdges (labels);

  if (return_label_indices_ == true)
    this->assignLabelIndices(labels, label_indices);
}

#define PCL_INSTANTIATE_OrganizedEdgeBase(T,LT)               template class PCL_EXPORTS pcl::OrganizedEdgeBase<T,LT>;
#define PCL_INSTANTIATE_OrganizedEdgeFromRGB(T,LT)            template class PCL_EXPORTS pcl::OrganizedEdgeFromRGB<T,LT>;
#define PCL_INSTANTIATE_OrganizedEdgeFromNormals(T,NT,LT)     template class PCL_EXPORTS pcl::OrganizedEdgeFromNormals<T,NT,LT>;
#define PCL_INSTANTIATE_OrganizedEdgeFromRGBNormals(T,NT,LT)  template class PCL_EXPORTS pcl::OrganizedEdgeFromRGBNormals<T,NT,LT>;
#define PCL_INSTANTIATE_OrganizedEdgeFromPoints(T,NT,LT)      template class PCL_EXPORTS pcl::OrganizedEdgeFromPoints<T,NT,LT>;
#define PCL_INSTANTIATE_OrganizedEdgeFromRGBPoints(T,NT,LT)   template class PCL_EXPORTS pcl::OrganizedEdgeFromRGBPoints<T,NT,LT>;

#endif //#ifndef PCL_FEATURES_IMPL_ORGANIZED_EDGE_DETECTION_H_

/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * person_cluster.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_PERSON_CLUSTER_HPP_
#define PCL_PEOPLE_PERSON_CLUSTER_HPP_

#include <pcl/people/person_cluster.h>

template <typename PointT>
pcl::people::PersonCluster<PointT>::PersonCluster (
    const PointCloudPtr& input_cloud,
    const pcl::PointIndices& indices,
    const Eigen::VectorXf& ground_coeffs,
    float sqrt_ground_coeffs,
    bool head_centroid,
    bool vertical)
    {
      init(input_cloud, indices, ground_coeffs, sqrt_ground_coeffs, head_centroid, vertical);
    }

template <typename PointT> void
pcl::people::PersonCluster<PointT>::init (
    const PointCloudPtr& input_cloud,
    const pcl::PointIndices& indices,
    const Eigen::VectorXf& ground_coeffs,
    float sqrt_ground_coeffs,
    bool head_centroid,
    bool vertical)
{

  vertical_ = vertical;
  head_centroid_ = head_centroid;
  person_confidence_ = std::numeric_limits<float>::quiet_NaN();

  min_x_ = 1000.0f;
  min_y_ = 1000.0f;
  min_z_ = 1000.0f;

  max_x_ = -1000.0f;
  max_y_ = -1000.0f;
  max_z_ = -1000.0f;

  sum_x_ = 0.0f;
  sum_y_ = 0.0f;
  sum_z_ = 0.0f;

  n_ = 0;

  points_indices_.indices = indices.indices;

  for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
  {
    PointT* p = &input_cloud->points[*pit];

    min_x_ = std::min(p->x, min_x_);
    max_x_ = std::max(p->x, max_x_);
    sum_x_ += p->x;

    min_y_ = std::min(p->y, min_y_);
    max_y_ = std::max(p->y, max_y_);
    sum_y_ += p->y;

    min_z_ = std::min(p->z, min_z_);
    max_z_ = std::max(p->z, max_z_);
    sum_z_ += p->z;

    n_++;
  }

  c_x_ = sum_x_ / n_;
  c_y_ = sum_y_ / n_;
  c_z_ = sum_z_ / n_;


  Eigen::Vector4f height_point(c_x_, c_y_, c_z_, 1.0f);
  if(!vertical_)
  {
    height_point(1) = min_y_;
    distance_ = std::sqrt(c_x_ * c_x_ + c_z_ * c_z_);
  }
  else
  {
    height_point(0) = max_x_;
    distance_ = std::sqrt(c_y_ * c_y_ + c_z_ * c_z_);
  }

  float height = std::fabs(height_point.dot(ground_coeffs));
  height /= sqrt_ground_coeffs;
  height_ = height;

  if(head_centroid_)
  {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    int n = 0;

    float head_threshold_value;    // vertical coordinate of the lowest head point
    if (!vertical_)
    {
      head_threshold_value = min_y_ + height_ / 8.0f;    // head is suppose to be 1/8 of the human height
      for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
      {
        PointT* p = &input_cloud->points[*pit];

        if(p->y < head_threshold_value)
        {
          sum_x += p->x;
          sum_y += p->y;
          sum_z += p->z;
          n++;
        }
      }
    }
    else
    {
      head_threshold_value = max_x_ - height_ / 8.0f;    // head is suppose to be 1/8 of the human height
      for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
      {
        PointT* p = &input_cloud->points[*pit];

        if(p->x > head_threshold_value)
        {
          sum_x += p->x;
          sum_y += p->y;
          sum_z += p->z;
          n++;
        }
      }
    }

    c_x_ = sum_x / n;
    c_y_ = sum_y / n;
    c_z_ = sum_z / n;
  }

  if(!vertical_)
  {
    float min_x = c_x_;
    float min_z = c_z_;
    float max_x = c_x_;
    float max_z = c_z_;
    for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
    {
      PointT* p = &input_cloud->points[*pit];

      min_x = std::min(p->x, min_x);
      max_x = std::max(p->x, max_x);
      min_z = std::min(p->z, min_z);
      max_z = std::max(p->z, max_z);
    }

    angle_ = std::atan2(c_z_, c_x_);
    angle_max_ = std::max(std::atan2(min_z, min_x), std::atan2(max_z, min_x));
    angle_min_ = std::min(std::atan2(min_z, max_x), std::atan2(max_z, max_x));

    Eigen::Vector4f c_point(c_x_, c_y_, c_z_, 1.0f);
    float t = c_point.dot(ground_coeffs) / std::pow(sqrt_ground_coeffs, 2);
    float bottom_x = c_x_ - ground_coeffs(0) * t;
    float bottom_y = c_y_ - ground_coeffs(1) * t;
    float bottom_z = c_z_ - ground_coeffs(2) * t;

    tbottom_ = Eigen::Vector3f(bottom_x, bottom_y, bottom_z);
    Eigen::Vector3f v = Eigen::Vector3f(c_x_, c_y_, c_z_) - tbottom_;

    ttop_ = v * height / v.norm() + tbottom_;
    tcenter_ = v * height * 0.5 / v.norm() + tbottom_;
    top_ = Eigen::Vector3f(c_x_, min_y_, c_z_);
    bottom_ = Eigen::Vector3f(c_x_, max_y_, c_z_);
    center_ = Eigen::Vector3f(c_x_, c_y_, c_z_);

    min_ = Eigen::Vector3f(min_x_, min_y_, min_z_);

    max_ = Eigen::Vector3f(max_x_, max_y_, max_z_);
  }
  else
  {
    float min_y = c_y_;
    float min_z = c_z_;
    float max_y = c_y_;
    float max_z = c_z_;
    for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
    {
      PointT* p = &input_cloud->points[*pit];

      min_y = std::min(p->y, min_y);
      max_y = std::max(p->y, max_y);
      min_z = std::min(p->z, min_z);
      max_z = std::max(p->z, max_z);
    }

    angle_ = std::atan2(c_z_, c_y_);
    angle_max_ = std::max(std::atan2(min_z_, min_y_), std::atan2(max_z_, min_y_));
    angle_min_ = std::min(std::atan2(min_z_, max_y_), std::atan2(max_z_, max_y_));

    Eigen::Vector4f c_point(c_x_, c_y_, c_z_, 1.0f);
    float t = c_point.dot(ground_coeffs) / std::pow(sqrt_ground_coeffs, 2);
    float bottom_x = c_x_ - ground_coeffs(0) * t;
    float bottom_y = c_y_ - ground_coeffs(1) * t;
    float bottom_z = c_z_ - ground_coeffs(2) * t;

    tbottom_ = Eigen::Vector3f(bottom_x, bottom_y, bottom_z);
    Eigen::Vector3f v = Eigen::Vector3f(c_x_, c_y_, c_z_) - tbottom_;

    ttop_ = v * height / v.norm() + tbottom_;
    tcenter_ = v * height * 0.5 / v.norm() + tbottom_;
    top_ = Eigen::Vector3f(max_x_, c_y_, c_z_);
    bottom_ = Eigen::Vector3f(min_x_, c_y_, c_z_);
    center_ = Eigen::Vector3f(c_x_, c_y_, c_z_);

    min_ = Eigen::Vector3f(min_x_, min_y_, min_z_);

    max_ = Eigen::Vector3f(max_x_, max_y_, max_z_);
  }
}

template <typename PointT> pcl::PointIndices&
pcl::people::PersonCluster<PointT>::getIndices ()
{
  return (points_indices_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::getHeight ()
{
  return (height_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::updateHeight (const Eigen::VectorXf& ground_coeffs)
{
  float sqrt_ground_coeffs = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
  return (updateHeight(ground_coeffs, sqrt_ground_coeffs));
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::updateHeight (const Eigen::VectorXf& ground_coeffs, float sqrt_ground_coeffs)
{
  Eigen::Vector4f height_point;
  if (!vertical_)
    height_point << c_x_, min_y_, c_z_, 1.0f;
  else
    height_point << max_x_, c_y_, c_z_, 1.0f;

  float height = std::fabs(height_point.dot(ground_coeffs));
  height /= sqrt_ground_coeffs;
  height_ = height;
  return (height_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::getDistance ()
{
  return (distance_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTTop ()
{
  return (ttop_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTBottom ()
{
  return (tbottom_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTCenter ()
{
  return (tcenter_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTop ()
{
  return (top_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getBottom ()
{
  return (bottom_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getCenter ()
{
  return (center_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getMin ()
{
  return (min_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getMax ()
{
  return (max_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::getAngle ()
{
  return (angle_);
}

template <typename PointT>
float pcl::people::PersonCluster<PointT>::getAngleMax ()
{
  return (angle_max_);
}

template <typename PointT>
float pcl::people::PersonCluster<PointT>::getAngleMin ()
{
  return (angle_min_);
}

template <typename PointT>
int pcl::people::PersonCluster<PointT>::getNumberPoints ()
{
  return (n_);
}

template <typename PointT>
float pcl::people::PersonCluster<PointT>::getPersonConfidence ()
{
  return (person_confidence_);
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::setPersonConfidence (float confidence)
{
  person_confidence_ = confidence;
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::setHeight (float height)
{
  height_ = height;
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::drawTBoundingBox (pcl::visualization::PCLVisualizer& viewer, int person_number)
{
  // draw theoretical person bounding box in the PCL viewer:
  pcl::ModelCoefficients coeffs;
  // translation
  coeffs.values.push_back (tcenter_[0]);
  coeffs.values.push_back (tcenter_[1]);
  coeffs.values.push_back (tcenter_[2]);
  // rotation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  // size
  if (vertical_)
  {
    coeffs.values.push_back (height_);
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (0.5);
  }
  else
  {
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (height_);
    coeffs.values.push_back (0.5);
  }

  std::stringstream bbox_name;
  bbox_name << "bbox_person_" << person_number;
  viewer.removeShape (bbox_name.str());
  viewer.addCube (coeffs, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, bbox_name.str());

  //      std::stringstream confid;
  //      confid << person_confidence_;
  //      PointT position;
  //      position.x = tcenter_[0]- 0.2;
  //      position.y = ttop_[1];
  //      position.z = tcenter_[2];
  //      viewer.addText3D(confid.str().substr(0, 4), position, 0.1);
}

template <typename PointT>
pcl::people::PersonCluster<PointT>::~PersonCluster ()
{
  // Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_PERSON_CLUSTER_HPP_ */

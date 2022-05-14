/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * $Id$
 *
 */

#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/common/point_tests.h>
#include <pcl/features/integral_image_normal.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InputDataProcessing::InputDataProcessing()
: normal_estimation_(new NormalEstimation())
, x_min_(-15.f)
, x_max_(15.f)
, y_min_(-15.f)
, y_max_(15.f)
, z_min_(48.f)
, z_max_(70.f)
, h_min_(210.f)
, h_max_(270.f)
, s_min_(0.2f)
, s_max_(1.f)
, v_min_(0.2f)
, v_max_(1.f)
, hsv_inverted_(false)
, hsv_enabled_(true)
, size_dilate_(3)
, size_erode_(3)
{
  // Normal estimation
  normal_estimation_->setNormalEstimationMethod(NormalEstimation::AVERAGE_3D_GRADIENT);
  normal_estimation_->setMaxDepthChangeFactor(0.02f); // in meters
  normal_estimation_->setNormalSmoothingSize(10.0f);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::InputDataProcessing::segment(const CloudXYZRGBAConstPtr& cloud_in,
                                       CloudXYZRGBNormalPtr& cloud_out,
                                       CloudXYZRGBNormalPtr& cloud_discarded) const
{
  if (!cloud_in) {
    std::cerr << "ERROR in input_data_processing.cpp: Input cloud is invalid.\n";
    return (false);
  }
  if (!cloud_in->isOrganized()) {
    std::cerr << "ERROR in input_data_processing.cpp: Input cloud must be organized.\n";
    return (false);
  }

  if (!cloud_out)
    cloud_out = CloudXYZRGBNormalPtr(new CloudXYZRGBNormal());
  if (!cloud_discarded)
    cloud_discarded = CloudXYZRGBNormalPtr(new CloudXYZRGBNormal());

  const unsigned int width = cloud_in->width;
  const unsigned int height = cloud_in->height;

  // Calculate the normals
  CloudNormalsPtr cloud_normals(new CloudNormals());
  normal_estimation_->setInputCloud(cloud_in);
  normal_estimation_->compute(*cloud_normals);

  // Get the XYZ and HSV masks.
  MatrixXb xyz_mask(height, width);
  MatrixXb hsv_mask(height, width);

  // cm -> m for the comparison
  const float x_min = .01f * x_min_;
  const float x_max = .01f * x_max_;
  const float y_min = .01f * y_min_;
  const float y_max = .01f * y_max_;
  const float z_min = .01f * z_min_;
  const float z_max = .01f * z_max_;

  float h, s, v;
  for (MatrixXb::Index r = 0; r < xyz_mask.rows(); ++r) {
    for (MatrixXb::Index c = 0; c < xyz_mask.cols(); ++c) {
      const PointXYZRGBA& xyzrgb = (*cloud_in)[r * width + c];
      const Normal& normal = (*cloud_normals)[r * width + c];

      xyz_mask(r, c) = hsv_mask(r, c) = false;

      if (!std::isnan(xyzrgb.x) && !std::isnan(normal.normal_x) && xyzrgb.x >= x_min &&
          xyzrgb.x <= x_max && xyzrgb.y >= y_min && xyzrgb.y <= y_max &&
          xyzrgb.z >= z_min && xyzrgb.z <= z_max) {
        xyz_mask(r, c) = true;

        this->RGBToHSV(xyzrgb.r, xyzrgb.g, xyzrgb.b, h, s, v);
        if (h >= h_min_ && h <= h_max_ && s >= s_min_ && s <= s_max_ && v >= v_min_ &&
            v <= v_max_) {
          if (!hsv_inverted_)
            hsv_mask(r, c) = true;
        }
        else {
          if (hsv_inverted_)
            hsv_mask(r, c) = true;
        }
      }
    }
  }

  this->erode(xyz_mask, size_erode_);
  if (hsv_enabled_)
    this->dilate(hsv_mask, size_dilate_);
  else
    hsv_mask.setZero();

  // Copy the normals into the clouds.
  cloud_out->reserve(cloud_in->size());
  cloud_discarded->reserve(cloud_in->size());

  pcl::PointXYZRGBNormal pt_out, pt_discarded;
  pt_discarded.r = 50;
  pt_discarded.g = 50;
  pt_discarded.b = 230;

  PointXYZRGBA xyzrgb;
  Normal normal;

  for (MatrixXb::Index r = 0; r < xyz_mask.rows(); ++r) {
    for (MatrixXb::Index c = 0; c < xyz_mask.cols(); ++c) {
      if (xyz_mask(r, c)) {
        xyzrgb = (*cloud_in)[r * width + c];
        normal = (*cloud_normals)[r * width + c];

        // m -> cm
        xyzrgb.getVector3fMap() = 100.f * xyzrgb.getVector3fMap();

        if (hsv_mask(r, c)) {
          pt_discarded.getVector4fMap() = xyzrgb.getVector4fMap();
          pt_discarded.getNormalVector4fMap() = normal.getNormalVector4fMap();

          pt_out.x = std::numeric_limits<float>::quiet_NaN();
        }
        else {
          pt_out.getVector4fMap() = xyzrgb.getVector4fMap();
          pt_out.getNormalVector4fMap() = normal.getNormalVector4fMap();
          pt_out.rgba = xyzrgb.rgba;

          pt_discarded.x = std::numeric_limits<float>::quiet_NaN();
        }
      }
      else {
        pt_out.x = std::numeric_limits<float>::quiet_NaN();
        pt_discarded.x = std::numeric_limits<float>::quiet_NaN();
      }

      cloud_out->push_back(pt_out);
      cloud_discarded->push_back(pt_discarded);
    }
  }

  cloud_out->width = cloud_discarded->width = width;
  cloud_out->height = cloud_discarded->height = height;
  cloud_out->is_dense = cloud_discarded->is_dense = false;

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::InputDataProcessing::calculateNormals(const CloudXYZRGBAConstPtr& cloud_in,
                                                CloudXYZRGBNormalPtr& cloud_out) const
{
  if (!cloud_in) {
    std::cerr << "ERROR in input_data_processing.cpp: Input cloud is invalid.\n";
    return (false);
  }

  if (!cloud_out)
    cloud_out = CloudXYZRGBNormalPtr(new CloudXYZRGBNormal());

  // Calculate the normals
  CloudNormalsPtr cloud_normals(new CloudNormals());
  normal_estimation_->setInputCloud(cloud_in);
  normal_estimation_->compute(*cloud_normals);

  // Copy the input cloud and normals into the output cloud.
  cloud_out->resize(cloud_in->size());
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  cloud_out->is_dense = false;

  CloudNormals::const_iterator it_n = cloud_normals->begin();
  CloudXYZRGBNormal::iterator it_out = cloud_out->begin();

  PointXYZRGBNormal invalid_pt;
  invalid_pt.x = invalid_pt.y = invalid_pt.z = std::numeric_limits<float>::quiet_NaN();
  invalid_pt.normal_x = invalid_pt.normal_y = invalid_pt.normal_z =
      std::numeric_limits<float>::quiet_NaN();
  invalid_pt.data[3] = 1.f;
  invalid_pt.data_n[3] = 0.f;

  for (auto it_in = cloud_in->begin(); it_in != cloud_in->end();
       ++it_in, ++it_n, ++it_out) {
    if (!it_n->getNormalVector4fMap().hasNaN()) {
      // m -> cm
      it_out->getVector4fMap() = 100.f * it_in->getVector4fMap();
      it_out->data[3] = 1.f;
      it_out->rgba = it_in->rgba;
      it_out->getNormalVector4fMap() = it_n->getNormalVector4fMap();
    }
    else {
      *it_out = invalid_pt;
    }
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::erode(MatrixXb& mask, const int k) const
{
  mask = manhattan(mask, false).array() > k;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::dilate(MatrixXb& mask, const int k) const
{
  mask = manhattan(mask, true).array() <= k;
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InputDataProcessing::MatrixXi
pcl::ihs::InputDataProcessing::manhattan(const MatrixXb& mat, const bool comp) const
{
  MatrixXi dist(mat.rows(), mat.cols());
  const int d_max = dist.rows() + dist.cols();

  // Forward
  for (MatrixXi::Index r = 0; r < dist.rows(); ++r) {
    for (MatrixXi::Index c = 0; c < dist.cols(); ++c) {
      if (mat(r, c) == comp) {
        dist(r, c) = 0;
      }
      else {
        dist(r, c) = d_max;
        if (r > 0)
          dist(r, c) = std::min(dist(r, c), dist(r - 1, c) + 1);
        if (c > 0)
          dist(r, c) = std::min(dist(r, c), dist(r, c - 1) + 1);
      }
    }
  }

  // Backward
  for (int r = dist.rows() - 1; r >= 0; --r) {
    for (int c = dist.cols() - 1; c >= 0; --c) {
      if (r < dist.rows() - 1)
        dist(r, c) = std::min(dist(r, c), dist(r + 1, c) + 1);
      if (c < dist.cols() - 1)
        dist(r, c) = std::min(dist(r, c), dist(r, c + 1) + 1);
    }
  }

  return (dist);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::RGBToHSV(const unsigned char r,
                                        const unsigned char g,
                                        const unsigned char b,
                                        float& h,
                                        float& s,
                                        float& v) const
{
  const unsigned char max = std::max(r, std::max(g, b));
  const unsigned char min = std::min(r, std::min(g, b));

  v = static_cast<float>(max) / 255.f;

  if (max == 0) // division by zero
  {
    s = 0.f;
    h = 0.f; // h = -1.f;
    return;
  }

  const float diff = static_cast<float>(max - min);
  s = diff / static_cast<float>(max);

  if (min == max) // diff == 0 -> division by zero
  {
    h = 0;
    return;
  }

  if (max == r)
    h = 60.f * (static_cast<float>(g - b) / diff);
  else if (max == g)
    h = 60.f * (2.f + static_cast<float>(b - r) / diff);
  else
    h = 60.f * (4.f + static_cast<float>(r - g) / diff); // max == b

  if (h < 0.f)
    h += 360.f;
}

////////////////////////////////////////////////////////////////////////////////

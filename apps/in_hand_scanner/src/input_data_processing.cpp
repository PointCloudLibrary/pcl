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
#include <pcl/apps/in_hand_scanner/boost.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InputDataProcessing::InputDataProcessing ()
  : normal_estimation_ (new NormalEstimation ()),

    x_min_ (-15.f),
    x_max_ ( 15.f),
    y_min_ (-15.f),
    y_max_ ( 15.f),
    z_min_ ( 48.f),
    z_max_ ( 70.f)
{
  // Normal estimation
  normal_estimation_->setNormalEstimationMethod (NormalEstimation::AVERAGE_3D_GRADIENT);
  normal_estimation_->setMaxDepthChangeFactor (0.02f); // in meters
  normal_estimation_->setNormalSmoothingSize (10.0f);
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InputDataProcessing::CloudXYZRGBNormalPtr
pcl::ihs::InputDataProcessing::process (const CloudXYZRGBAConstPtr& cloud) const
{
  const CloudXYZRGBNormalPtr cloud_out (new CloudXYZRGBNormal ());

  // Calculate the normals
  CloudNormalsPtr cloud_normals (new CloudNormals ());
  normal_estimation_->setInputCloud (cloud);
  normal_estimation_->compute (*cloud_normals);

  // Copy the input cloud and normals into the output cloud.
  // While we are already iterating over the whole input we can also remove some points.
  cloud_out->resize (cloud->size ());
  cloud_out->header              = cloud->header;
  cloud_out->width               = cloud->width;
  cloud_out->height              = cloud->height;
  cloud_out->is_dense            = false; /*cloud->is_dense && cloud_normals->is_dense;*/
  cloud_out->sensor_origin_      = cloud->sensor_origin_;
  cloud_out->sensor_orientation_ = cloud->sensor_orientation_;

  CloudXYZRGBA::const_iterator it_in  = cloud->begin ();
  CloudNormals::const_iterator it_n   = cloud_normals->begin ();
  CloudXYZRGBNormal::iterator  it_out = cloud_out->begin ();

  for (; it_in!=cloud->end (); ++it_in, ++it_n, ++it_out)
  {
    // m -> cm
    const Eigen::Vector3f xyz = 100.f * it_in->getVector3fMap ();

    if (!boost::math::isnan (it_in->x) && !boost::math::isnan (it_n->normal_x) &&
        xyz.x () >= x_min_             && xyz.x () <= x_max_                   &&
        xyz.y () >= y_min_             && xyz.y () <= y_max_                   &&
        xyz.z () >= z_min_             && xyz.z () <= z_max_)
    {
      it_out->getVector3fMap ()       = xyz;
      it_out->rgba                    = it_in->rgba;
      it_out->getNormalVector4fMap () = it_n->getNormalVector4fMap ();
    }
    else
    {
      // Fourth value should stay 1 (default)
      it_out->getVector3fMap () = Eigen::Vector3f (std::numeric_limits <float>::quiet_NaN (),
                                                   std::numeric_limits <float>::quiet_NaN (),
                                                   std::numeric_limits <float>::quiet_NaN ());
      // it_out->r = 0;
      // it_out->g = 0;
      // it_out->b = 0;
      // it_out->a = 255;

      // Fourth value should stay 0 (default)
      it_out->getNormalVector3fMap () = Eigen::Vector3f (std::numeric_limits <float>::quiet_NaN (),
                                                         std::numeric_limits <float>::quiet_NaN (),
                                                         std::numeric_limits <float>::quiet_NaN ());
    }
  }

  return (cloud_out);
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InputDataProcessing::CloudXYZRGBNormalPtr
pcl::ihs::InputDataProcessing::calculateNormals (const CloudXYZRGBAConstPtr& cloud) const
{
  const CloudXYZRGBNormalPtr cloud_out (new CloudXYZRGBNormal ());

  // Calculate the normals
  CloudNormalsPtr cloud_normals (new CloudNormals ());
  normal_estimation_->setInputCloud (cloud);
  normal_estimation_->compute (*cloud_normals);

  // Copy the input cloud and normals into the output cloud.
  cloud_out->resize (cloud->size ());
  cloud_out->header              = cloud->header;
  cloud_out->width               = cloud->width;
  cloud_out->height              = cloud->height;
  cloud_out->is_dense            = cloud->is_dense && cloud_normals->is_dense;
  cloud_out->sensor_origin_      = cloud->sensor_origin_;
  cloud_out->sensor_orientation_ = cloud->sensor_orientation_;

  CloudXYZRGBA::const_iterator it_in  = cloud->begin ();
  CloudNormals::const_iterator it_n   = cloud_normals->begin ();
  CloudXYZRGBNormal::iterator  it_out = cloud_out->begin ();

  for (; it_in!=cloud->end (); ++it_in, ++it_n, ++it_out)
  {
    // m -> cm
    it_out->getVector4fMap ()       = 100.f * it_in->getVector4fMap ();
    it_out->rgba                    = it_in->rgba;
    it_out->getNormalVector4fMap () = it_n->getNormalVector4fMap ();
  }

  return (cloud_out);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::setCropBox (const float x_min, const float x_max,
                                           const float y_min, const float y_max,
                                           const float z_min, const float z_max)
{
  x_min_ = x_min; x_max_ = x_max;
  y_min_ = y_min; y_max_ = y_max;
  z_min_ = z_min; z_max_ = z_max;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::getCropBox (float& x_min, float& x_max,
                                           float& y_min, float& y_max,
                                           float& z_min, float& z_max) const
{
  x_min = x_min_; x_max = x_max_;
  y_min = y_min_; y_max = y_max_;
  z_min = z_min_; z_max = z_max_;
}

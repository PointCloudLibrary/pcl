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
    z_max_ ( 70.f),

    h_min_ (210.f),
    h_max_ (270.f),
    s_min_ (  0.2f),
    s_max_ (  1.f),
    v_min_ (  0.2f),
    v_max_ (  1.f)
{
  // Normal estimation
  normal_estimation_->setNormalEstimationMethod (NormalEstimation::AVERAGE_3D_GRADIENT);
  normal_estimation_->setMaxDepthChangeFactor (0.02f); // in meters
  normal_estimation_->setNormalSmoothingSize (10.0f);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::InputDataProcessing::segment (const CloudXYZRGBAConstPtr& cloud_in,
                                        CloudXYZRGBNormalPtr&       cloud_out,
                                        CloudXYZRGBNormalPtr&       cloud_discarded) const
{
  if (!cloud_in)
  {
    std::cerr << "ERROR in input_data_processing.cpp: Input cloud is invalid.\n";
    return (false);
  }

  const unsigned int n_threads = 5;
  const unsigned int w         = cloud_in->width;
  const unsigned int h         = cloud_in->height;

  if (h % n_threads != 0)
  {
    std::cerr << "ERROR in input_data_processing.cpp: Input cloud must have a size multiple of " << n_threads << ".\n";
    return (false);
  }
  if (!cloud_out)       cloud_out       = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
  if (!cloud_discarded) cloud_discarded = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());

  // Calculate the normals
  CloudNormalsPtr cloud_normals (new CloudNormals ());
  normal_estimation_->setInputCloud (cloud_in);
  normal_estimation_->compute (*cloud_normals);

  // Copy the input cloud and normals into the output cloud.
  // While we are already iterating over the whole input we can also remove some points.
  PointXYZRGBNormal invalid_pt;
  invalid_pt.x = invalid_pt.y = invalid_pt.z =
      std::numeric_limits <float>::quiet_NaN ();
  invalid_pt.normal_x = invalid_pt.normal_y = invalid_pt.normal_z =
      std::numeric_limits <float>::quiet_NaN ();
  invalid_pt.data   [3] = 1.f;
  invalid_pt.data_n [3] = 0.f;

  pcl::PointXYZRGB pt_discarded;
  pt_discarded.r = 50;
  pt_discarded.g = 50;
  pt_discarded.b = 230;

  cloud_out->resize (cloud_in->size ());
  cloud_out->header   = cloud_in->header;
  cloud_out->width    = w;
  cloud_out->height   = h;
  cloud_out->is_dense = false; /*cloud->is_dense && cloud_normals->is_dense;*/
  *cloud_discarded    = *cloud_out;

  const unsigned int dH = h / n_threads;
  unsigned int offset;

  typedef boost::thread              Thread;
  typedef boost::shared_ptr <Thread> ThreadPtr;
  typedef std::vector <ThreadPtr>    ThreadPtrVec;
  ThreadPtrVec threads;
  threads.reserve (n_threads);

  for (unsigned int thread=0; thread<n_threads; ++thread)
  {
    offset = thread * dH * w;
    const PointXYZRGBA* p_in  = &cloud_in->front ()        + offset;
    const Normal*       p_n   = &cloud_normals->front ()   + offset;
    PointXYZRGBNormal*  p_out = &cloud_out->front ()       + offset;
    PointXYZRGBNormal*  p_dis = &cloud_discarded->front () + offset;

    threads.push_back (ThreadPtr (new Thread (boost::bind (&pcl::ihs::InputDataProcessing::threadSegmentation, this,
                                                           p_in, p_n, invalid_pt, pt_discarded.rgba, w, dH, p_out, p_dis))));
  }

  for (ThreadPtrVec::iterator it=threads.begin (); it!=threads.end (); ++it)
  {
    (*it)->join ();
  }

  return (true);
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

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::setColorRange (const float h_min, const float h_max,
                                              const float s_min, const float s_max,
                                              const float v_min, const float v_max)
{
  h_min_ = h_min; h_max_ = h_max;
  s_min_ = s_min; s_max_ = s_max;
  v_min_ = v_min; v_max_ = v_max;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::getColorRange (float& h_min, float& h_max,
                                              float& s_min, float& s_max,
                                              float& v_min, float& v_max) const
{
  h_min = h_min_; h_max = h_max_;
  s_min = s_min_; s_max = s_max_;
  v_min = v_min_; v_max = v_max_;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::threadSegmentation (const PointXYZRGBA*const p_pt_in,
                                                   const Normal*const       p_n_in,
                                                   const PointXYZRGBNormal& invalid_pt,
                                                   const uint32_t           color_discarded,
                                                   const unsigned int       width,
                                                   const unsigned int       n_rows,
                                                   PointXYZRGBNormal*const  p_pt_out,
                                                   PointXYZRGBNormal*const  p_pt_discarded) const
{
  unsigned int offset;
  Eigen::Vector3f xyz;
  float h, s, v;

  for (unsigned int r=0; r<n_rows; ++r)
  {
    for (unsigned int c=0; c<width; ++c)
    {
      offset = r*width + c;
      const PointXYZRGBA& pt_in  = *(p_pt_in        + offset);
      const Normal&       n_in   = *(p_n_in         + offset);
      PointXYZRGBNormal&  pt_out = *(p_pt_out       + offset);
      PointXYZRGBNormal&  pt_dis = *(p_pt_discarded + offset);

      // m -> cm
      xyz = 100.f * pt_in.getVector3fMap ();

      if (!boost::math::isnan (pt_in.x) && !boost::math::isnan (n_in.normal_x) &&
          xyz.x () >= x_min_            && xyz.x () <= x_max_                   &&
          xyz.y () >= y_min_            && xyz.y () <= y_max_                   &&
          xyz.z () >= z_min_            && xyz.z () <= z_max_)
      {
        this->RGBToHSV (pt_in.r, pt_in.g, pt_in.b, h, s, v);

        if (!(h >= h_min_ && h <= h_max_ &&
              s >= s_min_ && s <= s_max_ &&
              v >= v_min_ && v <= v_max_))
        {
          pt_out.getVector3fMap ()       = xyz;
          pt_out.getNormalVector4fMap () = n_in.getNormalVector4fMap ();
          pt_out.rgba                    = pt_in.rgba;

          pt_dis = invalid_pt;
        }
        else
        {
          pt_out = invalid_pt;

          pt_dis.getVector3fMap ()       = xyz;
          pt_dis.getNormalVector4fMap () = n_in.getNormalVector4fMap ();
          pt_dis.rgba                    = color_discarded;
        }
      }
      else
      {
        pt_out = invalid_pt;
        pt_dis = invalid_pt;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InputDataProcessing::RGBToHSV (const unsigned char r,
                                         const unsigned char g,
                                         const unsigned char b,
                                         float&              h,
                                         float&              s,
                                         float&              v) const
{
  const unsigned char max = std::max (r, std::max (g, b));
  const unsigned char min = std::min (r, std::min (g, b));

  v = static_cast <float> (max) / 255.f;

  if (max == 0) // division by zero
  {
    s = 0.f;
    h = 0.f; // h = -1.f;
    return;
  }

  const float diff = static_cast <float> (max - min);
  s = diff / static_cast <float> (max);

  if (min == max) // diff == 0 -> division by zero
  {
    h = 0;
    return;
  }

  if      (max == r) h = 60.f * (      static_cast <float> (g - b) / diff);
  else if (max == g) h = 60.f * (2.f + static_cast <float> (b - r) / diff);
  else               h = 60.f * (4.f + static_cast <float> (r - g) / diff); // max == b

  if (h < 0.f) h += 360.f;
}

////////////////////////////////////////////////////////////////////////////////

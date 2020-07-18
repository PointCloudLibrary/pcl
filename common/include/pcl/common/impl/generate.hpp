/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#pragma once

#include <pcl/common/generate.h>
#include <pcl/console/print.h>


namespace pcl
{

namespace common
{

template <typename PointT, typename GeneratorT>
CloudGenerator<PointT, GeneratorT>::CloudGenerator ()
  : x_generator_ ()
  , y_generator_ ()
  , z_generator_ ()
{}


template <typename PointT, typename GeneratorT>
CloudGenerator<PointT, GeneratorT>::CloudGenerator (const GeneratorParameters& params)
{
  setParameters (params);
}


template <typename PointT, typename GeneratorT>
CloudGenerator<PointT, GeneratorT>::CloudGenerator (const GeneratorParameters& x_params,
                                                    const GeneratorParameters& y_params,
                                                    const GeneratorParameters& z_params)
  : x_generator_ (x_params)
  , y_generator_ (y_params)
  , z_generator_ (z_params)
{}


template <typename PointT, typename GeneratorT> void
CloudGenerator<PointT, GeneratorT>::setParameters (const GeneratorParameters& params)
{
  GeneratorParameters y_params = params;
  y_params.seed += 1;
  GeneratorParameters z_params = y_params;
  z_params.seed += 1;
  x_generator_.setParameters (params);
  y_generator_.setParameters (y_params);
  z_generator_.setParameters (z_params);
}


template <typename PointT, typename GeneratorT> void
CloudGenerator<PointT, GeneratorT>::setParametersForX (const GeneratorParameters& x_params)
{
  x_generator_.setParameters (x_params);
}


template <typename PointT, typename GeneratorT> void
CloudGenerator<PointT, GeneratorT>::setParametersForY (const GeneratorParameters& y_params)
{
  y_generator_.setParameters (y_params);
}


template <typename PointT, typename GeneratorT> void
CloudGenerator<PointT, GeneratorT>::setParametersForZ (const GeneratorParameters& z_params)
{
  z_generator_.setParameters (z_params);
}


template <typename PointT, typename GeneratorT> const typename CloudGenerator<PointT, GeneratorT>::GeneratorParameters& 
CloudGenerator<PointT, GeneratorT>::getParametersForX () const
{
  x_generator_.getParameters ();
}


template <typename PointT, typename GeneratorT> const typename CloudGenerator<PointT, GeneratorT>::GeneratorParameters& 
CloudGenerator<PointT, GeneratorT>::getParametersForY () const
{
  y_generator_.getParameters ();
}


template <typename PointT, typename GeneratorT> const typename CloudGenerator<PointT, GeneratorT>::GeneratorParameters& 
CloudGenerator<PointT, GeneratorT>::getParametersForZ () const
{
  z_generator_.getParameters ();
}


template <typename PointT, typename GeneratorT> PointT
CloudGenerator<PointT, GeneratorT>::get ()
{
  PointT p;
  p.x = x_generator_.run ();
  p.y = y_generator_.run ();
  p.z = z_generator_.run ();
  return (p);
}


template <typename PointT, typename GeneratorT> int
CloudGenerator<PointT, GeneratorT>::fill (pcl::PointCloud<PointT>& cloud)
{
  return (fill (cloud.width, cloud.height, cloud));
}


template <typename PointT, typename GeneratorT> int
CloudGenerator<PointT, GeneratorT>::fill (int width, int height, pcl::PointCloud<PointT>& cloud)
{
  if (width < 1)
  {
    PCL_ERROR ("[pcl::common::CloudGenerator] Cloud width must be >= 1!\n");
    return (-1);
  }

  if (height < 1)
  {
    PCL_ERROR ("[pcl::common::CloudGenerator] Cloud height must be >= 1!\n");
    return (-1);
  }

  if (!cloud.empty ())
  {
    PCL_WARN ("[pcl::common::CloudGenerator] Cloud data will be erased with new data!\n");
  }

  cloud.width = width;
  cloud.height = height;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;
  for (auto& point: cloud)
  {
    point.x = x_generator_.run ();
    point.y = y_generator_.run ();
    point.z = z_generator_.run ();
  }
  return (0);
}


template <typename GeneratorT>
CloudGenerator<pcl::PointXY, GeneratorT>::CloudGenerator ()
  : x_generator_ ()
  , y_generator_ ()
{}


template <typename GeneratorT>
CloudGenerator<pcl::PointXY, GeneratorT>::CloudGenerator (const GeneratorParameters& x_params,
                                                          const GeneratorParameters& y_params)
  : x_generator_ (x_params)
  , y_generator_ (y_params)
{}


template <typename GeneratorT>
CloudGenerator<pcl::PointXY, GeneratorT>::CloudGenerator (const GeneratorParameters& params)
{
  setParameters (params);
}


template <typename GeneratorT> void
CloudGenerator<pcl::PointXY, GeneratorT>::setParameters (const GeneratorParameters& params)
{
  x_generator_.setParameters (params);
  GeneratorParameters y_params = params;
  y_params.seed += 1;
  y_generator_.setParameters (y_params);
}


template <typename GeneratorT> void
CloudGenerator<pcl::PointXY, GeneratorT>::setParametersForX (const GeneratorParameters& x_params)
{
  x_generator_.setParameters (x_params);
}


template <typename GeneratorT> void
CloudGenerator<pcl::PointXY, GeneratorT>::setParametersForY (const GeneratorParameters& y_params)
{
  y_generator_.setParameters (y_params);
}


template <typename GeneratorT> const typename CloudGenerator<pcl::PointXY, GeneratorT>::GeneratorParameters&
CloudGenerator<pcl::PointXY, GeneratorT>::getParametersForX () const
{
  x_generator_.getParameters ();
}


template <typename GeneratorT> const typename CloudGenerator<pcl::PointXY, GeneratorT>::GeneratorParameters&
CloudGenerator<pcl::PointXY, GeneratorT>::getParametersForY () const
{
  y_generator_.getParameters ();
}


template <typename GeneratorT> pcl::PointXY
CloudGenerator<pcl::PointXY, GeneratorT>::get ()
{
  pcl::PointXY p;
  p.x = x_generator_.run ();
  p.y = y_generator_.run ();
  return (p);
}


template <typename GeneratorT> int
CloudGenerator<pcl::PointXY, GeneratorT>::fill (pcl::PointCloud<pcl::PointXY>& cloud)
{
  return (fill (cloud.width, cloud.height, cloud));
}


template <typename GeneratorT> int
CloudGenerator<pcl::PointXY, GeneratorT>::fill (int width, int height, pcl::PointCloud<pcl::PointXY>& cloud)
{
  if (width < 1)
  {
    PCL_ERROR ("[pcl::common::CloudGenerator] Cloud width must be >= 1\n!");
    return (-1);
  }

  if (height < 1)
  {
    PCL_ERROR ("[pcl::common::CloudGenerator] Cloud height must be >= 1\n!");
    return (-1);
  }

  if (!cloud.empty ())
    PCL_WARN ("[pcl::common::CloudGenerator] Cloud data will be erased with new data\n!");

  cloud.width = width;
  cloud.height = height;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  for (auto &point : cloud)
  {
    point.x = x_generator_.run ();
    point.y = y_generator_.run ();
  }
  return (0);
}

} // namespace common
} // namespace pcl


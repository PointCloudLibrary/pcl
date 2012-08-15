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

#ifndef PCL_IN_HAND_SCANNER_IN_HAND_SCANNER_H
#define PCL_IN_HAND_SCANNER_IN_HAND_SCANNER_H

#include <string>

#include <boost/thread/mutex.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  class OpenNIGrabber;

  template <class PointInT, class PointOutT>
  class IntegralImageNormalEstimation;

  template <class PointT>
  class PassThrough;

  namespace visualization
  {
    class PCLVisualizer;
  } // End namespace visualization

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InHandScanner
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  class InHandScanner
  {

    private:

      typedef pcl::PointXYZRGBA      Point;
      typedef pcl::PointCloud<Point> Cloud;
      typedef Cloud::Ptr             CloudPtr;
      typedef Cloud::ConstPtr        CloudConstPtr;

      typedef pcl::PointXYZRGBNormal           PointWithNormal;
      typedef pcl::PointCloud<PointWithNormal> CloudWithNormals;
      typedef CloudWithNormals::Ptr            CloudWithNormalsPtr;
      typedef CloudWithNormals::ConstPtr       CloudWithNormalsConstPtr;

      typedef pcl::OpenNIGrabber               Grabber;
      typedef boost::shared_ptr<Grabber>       GrabberPtr;
      typedef boost::shared_ptr<const Grabber> GrabberConstPtr;

      typedef pcl::IntegralImageNormalEstimation<Point, PointWithNormal> NormalEstimation;
      typedef boost::shared_ptr<NormalEstimation>                        NormalEstimationPtr;
      typedef boost::shared_ptr<const NormalEstimation>                  NormalEstimationConstPtr;

      typedef pcl::PassThrough<PointWithNormal>    PassThrough;
      typedef boost::shared_ptr<PassThrough>       PassThroughPtr;
      typedef boost::shared_ptr<const PassThrough> PassThroughConstPtr;

      typedef pcl::visualization::PCLVisualizer      PCLVisualizer;
      typedef boost::shared_ptr<PCLVisualizer>       PCLVisualizerPtr;
      typedef boost::shared_ptr<const PCLVisualizer> PCLVisualizerConstPtr;

    public:

      InHandScanner ();
      ~InHandScanner ();

      void
      setVisualizer (const PCLVisualizerPtr& p_visualizer);

      void
      start ();

      void
      draw ();

    private:

      void
      grabbedDataCallback (const CloudConstPtr& p_cloud_in);

      void
      showFPS (const std::string& what) const;

    private:

      boost::mutex        mutex_;

      GrabberPtr          p_grabber_;
      PCLVisualizerPtr    p_visualizer_;
      NormalEstimationPtr p_normal_estimation_;
      PassThroughPtr      p_pass_through_;

      CloudPtr            p_drawn_cloud_;
  };

} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_IN_HAND_SCANNER_H

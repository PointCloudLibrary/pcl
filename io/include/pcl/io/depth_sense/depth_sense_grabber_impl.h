/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_IO_DEPTH_SENSE_GRABBER_IMPL_H
#define PCL_IO_DEPTH_SENSE_GRABBER_IMPL_H

#include <boost/thread/mutex.hpp>

#include <DepthSense.hxx>

#include <pcl/common/time.h>
#include <pcl/io/buffers.h>
#include <pcl/io/depth_sense_grabber.h>

namespace pcl
{

  namespace io
  {

    namespace depth_sense
    {

      struct DepthSenseGrabberImpl
      {

        /// Parent grabber
        DepthSenseGrabber* p_;

        /// Serial number of the device captured by this grabber
        std::string device_id_;

        bool is_running_;

        int confidence_threshold_;
        DepthSenseGrabber::TemporalFilteringType temporal_filtering_type_;

        boost::shared_ptr<DepthSense::ProjectionHelper> projection_;

        typedef DepthSenseGrabber::sig_cb_depth_sense_point_cloud sig_cb_depth_sense_point_cloud;
        typedef DepthSenseGrabber::sig_cb_depth_sense_point_cloud_rgba sig_cb_depth_sense_point_cloud_rgba;

        /// Signal to indicate whether new XYZ cloud is available
        boost::signals2::signal<sig_cb_depth_sense_point_cloud>* point_cloud_signal_;
        /// Signal to indicate whether new XYZRGBA cloud is available
        boost::signals2::signal<sig_cb_depth_sense_point_cloud_rgba>* point_cloud_rgba_signal_;

        /// Indicates whether there are subscribers for PointXYZ signal. This is
        /// computed and stored on start()
        bool need_xyz_;

        /// Indicates whether there are subscribers for PointXYZRGBA signal. This
        /// is computed and stored on start()
        bool need_xyzrgba_;

        EventFrequency frequency_;
        mutable boost::mutex fps_mutex_;

        /// Temporary buffer to store color data
        std::vector<uint8_t> color_data_;

        boost::shared_ptr<pcl::io::Buffer<float> > depth_buffer_;

        static const int FRAMERATE = 30;
        static const int WIDTH = 320;
        static const int HEIGHT = 240;
        static const int SIZE = WIDTH * HEIGHT;
        static const int COLOR_WIDTH = 640;
        static const int COLOR_HEIGHT = 480;
        static const int COLOR_SIZE = COLOR_WIDTH * COLOR_HEIGHT;

        DepthSenseGrabberImpl (DepthSenseGrabber* parent, const std::string& device_id);

        ~DepthSenseGrabberImpl () throw ();

        void
        start ();

        void
        stop ();

        float
        getFramesPerSecond () const;

        void
        setConfidenceThreshold (int threshold);

        void
        enableTemporalFiltering (DepthSenseGrabber::TemporalFilteringType type, size_t window_size);

        void
        setCameraParameters (const DepthSense::StereoCameraParameters& parameters);

        void
        configureDepthNode (DepthSense::DepthNode node) const;

        void
        configureColorNode (DepthSense::ColorNode node) const;

        /** A callback for processing depth data.
          *
          * It is supposed to be called from the DepthSense::Context thread that
          * is managed by DepthSenseDeviceManager. */
        void
        onDepthDataReceived (DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

        /** A callback for processing color data.
          *
          * It is supposed to be called from the DepthSense::Context thread that
          * is managed by DepthSenseDeviceManager. */
        void
        onColorDataReceived (DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

        template <typename Point> void
        computeXYZ (PointCloud<Point>& cloud);

      };

    }

  }

}

#endif /* PCL_IO_DEPTH_SENSE_GRABBER_IMPL_H */


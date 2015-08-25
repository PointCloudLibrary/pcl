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

#ifndef PCL_IO_DEPTH_SENSE_GRABBER_H
#define PCL_IO_DEPTH_SENSE_GRABBER_H

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{

  // Forward declaration of a class that contains actual grabber implementation
  namespace io { namespace depth_sense { struct DepthSenseGrabberImpl; } }

  /** Grabber for DepthSense devices (e.g. Creative Senz3D, SoftKinetic DS325).
    *
    * Requires [SoftKinetic DepthSense SDK](http://www.softkinetic.com/Support/Download).
    *
    * \author Sergey Alexandrov
    * \ingroup io */
  class PCL_EXPORTS DepthSenseGrabber : public Grabber
  {

    public:

      typedef
        void (sig_cb_depth_sense_point_cloud)
          (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);

      typedef
        void (sig_cb_depth_sense_point_cloud_rgba)
          (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

      enum Mode
      {
        DepthSense_QVGA_30Hz = 0,
      };

      enum TemporalFilteringType
      {
        DepthSense_None = 0,
        DepthSense_Median = 1,
        DepthSense_Average = 2,
      };

      /** Create a grabber for a DepthSense device.
        *
        * The grabber "captures" the device, making it impossible for other
        * grabbers to interact with it. The device is "released" when the
        * grabber is destructed.
        *
        * This will throw pcl::IOException if there are no free devices that
        * match the supplied \a device_id.
        *
        * \param[in] device_id device identifier, which might be a serial
        * number, an index (with '#' prefix), or an empty string (to select the
        * first available device)
        */
      DepthSenseGrabber (const std::string& device_id = "");

      virtual
      ~DepthSenseGrabber () throw ();

      virtual void
      start ();

      virtual void
      stop ();

      virtual bool
      isRunning () const;

      virtual std::string
      getName () const
      {
        return (std::string ("DepthSenseGrabber"));
      }

      virtual float
      getFramesPerSecond () const;

      /** Set the confidence threshold for depth data.
        *
        * Each pixel in a depth image output by the device has an associated
        * confidence value. The higher this value is, the more reliable the
        * datum is.
        *
        * The depth pixels (and their associated 3D points) are filtered based
        * on the confidence value. Those that are below the threshold are
        * discarded (i.e. their coordinates are set to NaN). */
      void
      setConfidenceThreshold (int threshold);

      /** Enable temporal filtering of the depth data received from the device.
        *
        * The window size parameter is not relevant for `DepthSense_None`
        * filtering type. */
      void
      enableTemporalFiltering (TemporalFilteringType type, size_t window_size = 1);

      /** Disable temporal filtering. */
      void
      disableTemporalFiltering ();

      /** Get the serial number of device captured by the grabber. */
      std::string
      getDeviceSerialNumber () const;

    private:

      pcl::io::depth_sense::DepthSenseGrabberImpl* p_;
      friend struct pcl::io::depth_sense::DepthSenseGrabberImpl;

  };

}

#endif /* PCL_IO_DEPTH_SENSE_GRABBER_H */


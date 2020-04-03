/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
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

#pragma once

#include <pcl/io/grabber.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace pcl
{

  namespace io
  {

    template <typename T> class Buffer;

    namespace real_sense
    {
      class RealSenseDevice;
    }

  }

  class PCL_EXPORTS RealSenseGrabber : public Grabber
  {

    public:

      using Ptr = shared_ptr<RealSenseGrabber>;
      using ConstPtr = shared_ptr<const RealSenseGrabber>;

      using sig_cb_real_sense_point_cloud = void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
      using sig_cb_real_sense_point_cloud_rgba = void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

      /** A descriptor for capturing mode.
        *
        * Consists of framerate and resolutions of depth and color streams.
        * Serves two purposes: to describe the desired capturing mode when
        * creating a grabber, and to list the available modes supported by the
        * grabber (see getAvailableModes()). In the first case setting some
        * fields to zero means "don't care", i.e. the grabber is allowed to
        * decide itself which concrete values to use. */
      struct PCL_EXPORTS Mode
      {
        unsigned int fps;
        unsigned int depth_width;
        unsigned int depth_height;
        unsigned int color_width;
        unsigned int color_height;

        /** Set all fields to zero (i.e. "don't care"). */
        Mode ();

        /** Set desired framerate, the rest is "don't care". */
        Mode (unsigned int fps);

        /** Set desired depth resolution, the rest is "don't care". */
        Mode (unsigned int depth_width, unsigned int depth_height);

        /** Set desired framerate and depth resolution, the rest is "don't
          * care". */
        Mode (unsigned int fps, unsigned int depth_width, unsigned int depth_height);

        /** Set desired depth and color resolution, the rest is "don't
          * care". */
        Mode (unsigned int depth_width, unsigned int depth_height, unsigned int color_width, unsigned int color_height);

        /** Set desired framerate, depth and color resolution. */
        Mode (unsigned int fps, unsigned int depth_width, unsigned int depth_height, unsigned int color_width, unsigned int color_height);

        bool
        operator== (const pcl::RealSenseGrabber::Mode& m) const;
      };

      enum TemporalFilteringType
      {
        RealSense_None = 0,
        RealSense_Median = 1,
        RealSense_Average = 2,
      };

      /** Create a grabber for a RealSense device.
        *
        * The grabber "captures" the device, making it impossible for other
        * grabbers to interact with it. The device is "released" when the
        * grabber is destructed.
        *
        * This will throw pcl::io::IOException if there are no free devices
        * that match the supplied \a device_id.
        *
        * \param[in] device_id device identifier, which can be a serial number,
        * a zero-based index (with '#' prefix), or an empty string (to select
        * the first available device)
        * \param[in] mode desired framerate and stream resolution (see Mode).
        * If the default is supplied, then the mode closest to VGA at 30 Hz
        * will be chosen.
        * \param[in] strict if set to \c true, an exception will be thrown if
        * device does not support exactly the mode requested. Otherwise the
        * closest available mode is selected. */
      RealSenseGrabber (const std::string& device_id = "", const Mode& mode = Mode (), bool strict = false);

      virtual
      ~RealSenseGrabber () noexcept;

      virtual void
      start ();

      virtual void
      stop ();

      virtual bool
      isRunning () const;

      virtual std::string
      getName () const
      {
        return (std::string ("RealSenseGrabber"));
      }

      virtual float
      getFramesPerSecond () const;

      /** Set the confidence threshold for depth data.
        *
        * Valid range is [0..15]. Discarded points will have their coordinates
        * set to NaNs). */
      void
      setConfidenceThreshold (unsigned int threshold);

      /** Enable temporal filtering of the depth data received from the device.
        *
        * The window size parameter is not relevant for `RealSense_None`
        * filtering type.
        *
        * \note if the grabber is running and the new parameters are different
        * from the current parameters, grabber will be restarted. */
      void
      enableTemporalFiltering (TemporalFilteringType type, std::size_t window_size);

      /** Disable temporal filtering. */
      void
      disableTemporalFiltering ();

      /** Get the serial number of device captured by the grabber. */
      const std::string&
      getDeviceSerialNumber () const;

      /** Get a list of capturing modes supported by the PXC device
        * controlled by this grabber.
        *
        * \param[in] only_depth list depth-only modes
        *
        * \note: this list exclude modes where framerates of the depth and
        * color streams do not match. */
      std::vector<Mode>
      getAvailableModes (bool only_depth = false) const;

      /** Set desired capturing mode.
        *
        * \note if the grabber is running and the new mode is different the
        * one requested previously, grabber will be restarted. */
      void
      setMode (const Mode& mode, bool strict = false);

      /** Get currently active capturing mode.
        *
        * \note: capturing mode is selected when start() is called; output of
        * this function before grabber was started is undefined. */
      const Mode&
      getMode () const
      {
        return (mode_selected_);
      }

    private:

      void
      run ();

      void
      createDepthBuffer ();

      void
      selectMode ();

      /** Compute a score which indicates how different is a given mode is from
        * the mode requested by the user.
        *
        * Importance of factors: fps > depth resolution > color resolution. The
        * lower the score the better. */
      float
      computeModeScore (const Mode& mode);

      // Signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_real_sense_point_cloud>* point_cloud_signal_;
      boost::signals2::signal<sig_cb_real_sense_point_cloud_rgba>* point_cloud_rgba_signal_;

      std::shared_ptr<pcl::io::real_sense::RealSenseDevice> device_;

      bool is_running_;
      unsigned int confidence_threshold_;

      TemporalFilteringType temporal_filtering_type_;
      std::size_t temporal_filtering_window_size_;

      /// Capture mode requested by the user at construction time
      Mode mode_requested_;

      /// Whether or not selected capture mode should strictly match what the user
      /// has requested
      bool strict_;

      /// Capture mode selected by grabber (among the modes supported by the
      /// device), computed and stored on start()
      Mode mode_selected_;

      /// Indicates whether there are subscribers for PointXYZ signal, computed
      /// and stored on start()
      bool need_xyz_;

      /// Indicates whether there are subscribers for PointXYZRGBA signal,
      /// computed and stored on start()
      bool need_xyzrgba_;

      EventFrequency frequency_;
      mutable std::mutex fps_mutex_;

      std::thread thread_;

      /// Depth buffer to perform temporal filtering of the depth images
      std::shared_ptr<pcl::io::Buffer<unsigned short> > depth_buffer_;
  };
}

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

#pragma once

#include <pcl/pcl_config.h>
#ifdef HAVE_FZAPI

#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/common/synchronizer.h>

#include <boost/thread.hpp>  

#include <fz_api.h>

#include <string>
#include <deque>

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;
  template <typename T> class PointCloud;


  /** \brief Grabber for Fotonic devices
    * \author Stefan Holzer <holzers@in.tum.de>
    * \ingroup io
    */
  class PCL_EXPORTS FotonicGrabber : public Grabber
  {
    public:

      typedef enum
      {
        Fotonic_Default_Mode = 0, // This can depend on the device. For now all devices (PSDK, Xtion, Kinect) its VGA@30Hz
      } Mode;

      //define callback signature typedefs
      typedef void (sig_cb_fotonic_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
      typedef void (sig_cb_fotonic_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
      typedef void (sig_cb_fotonic_point_cloud_rgba) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);
      typedef void (sig_cb_fotonic_point_cloud_i) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);

    public:
      /** \brief Constructor
        * \param[in] device_id ID of the device, which might be a serial number, bus@address or the index of the device.
        * \param[in] depth_mode the mode of the depth stream
        * \param[in] image_mode the mode of the image stream
        */
      FotonicGrabber (const FZ_DEVICE_INFO& device_info,
                      const Mode& depth_mode = Fotonic_Default_Mode,
                      const Mode& image_mode = Fotonic_Default_Mode);

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      virtual ~FotonicGrabber () throw ();

      /** \brief Initializes the Fotonic API. */
      static void
      initAPI ();

      /** \brief Exits the Fotonic API. */
      static void
      exitAPI ();

      /** \brief Searches for available devices. */
      static std::vector<FZ_DEVICE_INFO>
      enumDevices ();

      /** \brief Start the data acquisition. */
      virtual void
      start ();

      /** \brief Stop the data acquisition. */
      virtual void
      stop ();

      /** \brief Check if the data acquisition is still running. */
      virtual bool
      isRunning () const;

      virtual std::string
      getName () const;

      /** \brief Obtain the number of frames per second (FPS). */
      virtual float 
      getFramesPerSecond () const;

    protected:

      /** \brief On initialization processing. */
      void
      onInit (const FZ_DEVICE_INFO& device_info, const Mode& depth_mode, const Mode& image_mode);

      /** \brief Sets up an OpenNI device. */
      void
      setupDevice (const FZ_DEVICE_INFO& device_info, const Mode& depth_mode, const Mode& image_mode);

      /** \brief Continuously asks for data from the device and publishes it if available. */
      void
      processGrabbing ();

      boost::signals2::signal<sig_cb_fotonic_point_cloud>* point_cloud_signal_;
      //boost::signals2::signal<sig_cb_fotonic_point_cloud_i>* point_cloud_i_signal_;
      boost::signals2::signal<sig_cb_fotonic_point_cloud_rgb>* point_cloud_rgb_signal_;
      boost::signals2::signal<sig_cb_fotonic_point_cloud_rgba>* point_cloud_rgba_signal_;

    protected:
      bool running_;

      FZ_Device_Handle_t * fotonic_device_handle_;

      boost::thread grabber_thread_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace pcl
#endif // HAVE_FOTONIC

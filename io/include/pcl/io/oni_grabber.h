/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>

#ifdef HAVE_OPENNI

#include <pcl/point_cloud.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_camera/openni_device_oni.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_ir_image.h>
#include <string>
#include <pcl/common/synchronizer.h>

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;

  /** \brief A simple ONI grabber.
    * \author Suat Gedikli
    * \ingroup io
    */
  class PCL_EXPORTS ONIGrabber : public Grabber
  {
    public:
      //define callback signature typedefs
      using sig_cb_openni_image = void (const openni_wrapper::Image::Ptr &);
      using sig_cb_openni_depth_image = void (const openni_wrapper::DepthImage::Ptr &);
      using sig_cb_openni_ir_image = void (const openni_wrapper::IRImage::Ptr &);
      using sig_cb_openni_image_depth_image = void (const openni_wrapper::Image::Ptr &, const openni_wrapper::DepthImage::Ptr &, float) ;
      using sig_cb_openni_ir_depth_image = void (const openni_wrapper::IRImage::Ptr &, const openni_wrapper::DepthImage::Ptr &, float) ;
      using sig_cb_openni_point_cloud = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);
      using sig_cb_openni_point_cloud_rgb = void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
      using sig_cb_openni_point_cloud_rgba = void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &);
      using sig_cb_openni_point_cloud_i = void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);

      /** \brief constructor
        * \param[in] file_name the path to the ONI file
        * \param[in] repeat whether the play back should be in an infinite loop or not
        * \param[in] stream whether the playback should be in streaming mode or in triggered mode.
        */
      ONIGrabber (const std::string& file_name, bool repeat, bool stream);

      /** \brief destructor never throws an exception */
      ~ONIGrabber () noexcept override;

      /** \brief For devices that are streaming, the streams are started by calling this method.
        *        Trigger-based devices, just trigger the device once for each call of start.
        */
      void
      start () override;

      /** \brief For devices that are streaming, the streams are stopped.
        *        This method has no effect for triggered devices.
        */
      void
      stop () override;

      /** \brief returns the name of the concrete subclass.
        * \return the name of the concrete driver.
        */
      std::string
      getName () const override;

      /** \brief Indicates whether the grabber is streaming or not. This value is not defined for triggered devices.
        * \return true if grabber is running / streaming. False otherwise.
        */
      bool
      isRunning () const override;

      /** \brief returns the frames pre second. 0 if it is trigger based. */
      float
      getFramesPerSecond () const override;

      /** \brief Check if there is any data left in the ONI file to process. */
      inline bool
      hasDataLeft ()
      {
        return (device_->hasDataLeft ());
      }

     protected:
      /** \brief internal OpenNI (openni_wrapper) callback that handles image streams */
      void
      imageCallback (openni_wrapper::Image::Ptr image, void* cookie);

      /** \brief internal OpenNI (openni_wrapper) callback that handles depth streams */
      void
      depthCallback (openni_wrapper::DepthImage::Ptr depth_image, void* cookie);

      /** \brief internal OpenNI (openni_wrapper) callback that handles IR streams */
      void
      irCallback (openni_wrapper::IRImage::Ptr ir_image, void* cookie);

      /** \brief internal callback that handles synchronized image + depth streams */
      void
      imageDepthImageCallback (const openni_wrapper::Image::Ptr &image,
                               const openni_wrapper::DepthImage::Ptr &depth_image);

      /** \brief internal callback that handles synchronized IR + depth streams */
      void
      irDepthImageCallback (const openni_wrapper::IRImage::Ptr &image,
                            const openni_wrapper::DepthImage::Ptr &depth_image);

      /** \brief internal method to assemble a point cloud object */
      pcl::PointCloud<pcl::PointXYZ>::Ptr
      convertToXYZPointCloud (const openni_wrapper::DepthImage::Ptr &depth) const;

      /** \brief internal method to assemble a point cloud object */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      convertToXYZRGBPointCloud (const openni_wrapper::Image::Ptr &image,
                                 const openni_wrapper::DepthImage::Ptr &depth_image) const;

      /** \brief internal method to assemble a point cloud object */
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      convertToXYZRGBAPointCloud (const openni_wrapper::Image::Ptr &image,
                                  const openni_wrapper::DepthImage::Ptr &depth_image) const;

      /** \brief internal method to assemble a point cloud object */
      pcl::PointCloud<pcl::PointXYZI>::Ptr
      convertToXYZIPointCloud (const openni_wrapper::IRImage::Ptr &image,
                               const openni_wrapper::DepthImage::Ptr &depth_image) const;

      /** \brief synchronizer object to synchronize image and depth streams*/
      Synchronizer<openni_wrapper::Image::Ptr, openni_wrapper::DepthImage::Ptr > rgb_sync_;

      /** \brief synchronizer object to synchronize IR and depth streams*/
      Synchronizer<openni_wrapper::IRImage::Ptr, openni_wrapper::DepthImage::Ptr > ir_sync_;

      /** \brief the actual openni device*/
      openni_wrapper::DeviceONI::Ptr device_;
      std::string rgb_frame_id_;
      std::string depth_frame_id_;
      bool running_;
      unsigned image_width_;
      unsigned image_height_;
      unsigned depth_width_;
      unsigned depth_height_;
      openni_wrapper::OpenNIDevice::CallbackHandle depth_callback_handle;
      openni_wrapper::OpenNIDevice::CallbackHandle image_callback_handle;
      openni_wrapper::OpenNIDevice::CallbackHandle ir_callback_handle;
      boost::signals2::signal<sig_cb_openni_image >*            image_signal_;
      boost::signals2::signal<sig_cb_openni_depth_image >*      depth_image_signal_;
      boost::signals2::signal<sig_cb_openni_ir_image >*         ir_image_signal_;
      boost::signals2::signal<sig_cb_openni_image_depth_image>* image_depth_image_signal_;
      boost::signals2::signal<sig_cb_openni_ir_depth_image>*    ir_depth_image_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud >*      point_cloud_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud_i >*    point_cloud_i_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud_rgb >*  point_cloud_rgb_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud_rgba >*  point_cloud_rgba_signal_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace
#endif // HAVE_OPENNI

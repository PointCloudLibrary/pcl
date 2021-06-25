/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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

#include <pcl/memory.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>

#ifdef HAVE_OPENNI

#include <pcl/point_cloud.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_camera/openni_device_kinect.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_ir_image.h>
#include <string>
#include <pcl/common/synchronizer.h>
#include <boost/shared_array.hpp> // for shared_array

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;

  /** \brief Grabber for OpenNI devices (i.e., Primesense PSDK, Microsoft Kinect, Asus XTion Pro/Live)
    * \author Nico Blodow <blodow@cs.tum.edu>, Suat Gedikli <gedikli@willowgarage.com>
    * \ingroup io
    */
  class PCL_EXPORTS OpenNIGrabber : public Grabber
  {
    public:
      using Ptr = shared_ptr<OpenNIGrabber>;
      using ConstPtr = shared_ptr<const OpenNIGrabber>;

      enum Mode
      {
        OpenNI_Default_Mode = 0, // This can depend on the device. For now all devices (PSDK, Xtion, Kinect) its VGA@30Hz
        OpenNI_SXGA_15Hz = 1,    // Only supported by the Kinect
        OpenNI_VGA_30Hz = 2,     // Supported by PSDK, Xtion and Kinect
        OpenNI_VGA_25Hz = 3,     // Supportged by PSDK and Xtion
        OpenNI_QVGA_25Hz = 4,    // Supported by PSDK and Xtion
        OpenNI_QVGA_30Hz = 5,    // Supported by PSDK, Xtion and Kinect
        OpenNI_QVGA_60Hz = 6,    // Supported by PSDK and Xtion
        OpenNI_QQVGA_25Hz = 7,   // Not supported -> using software downsampling (only for integer scale factor and only NN)
        OpenNI_QQVGA_30Hz = 8,   // Not supported -> using software downsampling (only for integer scale factor and only NN)
        OpenNI_QQVGA_60Hz = 9    // Not supported -> using software downsampling (only for integer scale factor and only NN)
      };

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

    public:
      /** \brief Constructor
        * \param[in] device_id ID of the device, which might be a serial number, bus\@address or the index of the device.
        * \param[in] depth_mode the mode of the depth stream
        * \param[in] image_mode the mode of the image stream
        */
      OpenNIGrabber (const std::string& device_id = "",
                     const Mode& depth_mode = OpenNI_Default_Mode,
                     const Mode& image_mode = OpenNI_Default_Mode);

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      ~OpenNIGrabber () noexcept override;

      /** \brief Start the data acquisition. */
      void
      start () override;

      /** \brief Stop the data acquisition. */
      void
      stop () override;

      /** \brief Check if the data acquisition is still running. */
      bool
      isRunning () const override;

      std::string
      getName () const override;

      /** \brief Obtain the number of frames per second (FPS). */
      float
      getFramesPerSecond () const override;

      /** \brief Get a pcl::shared pointer to the openni_wrapper::OpenNIDevice object. */
      inline openni_wrapper::OpenNIDevice::Ptr
      getDevice () const;

      /** \brief Obtain a list of the available depth modes that this device supports. */
      std::vector<std::pair<int, XnMapOutputMode> >
      getAvailableDepthModes () const;

      /** \brief Obtain a list of the available image modes that this device supports. */
      std::vector<std::pair<int, XnMapOutputMode> >
      getAvailableImageModes () const;

      /** \brief Set the RGB camera parameters (fx, fy, cx, cy)
        * \param[in] rgb_focal_length_x the RGB focal length (fx)
        * \param[in] rgb_focal_length_y the RGB focal length (fy)
        * \param[in] rgb_principal_point_x the RGB principal point (cx)
        * \param[in] rgb_principal_point_y the RGB principal point (cy)
        * Setting the parameters to non-finite values (e.g., NaN, Inf) invalidates them
        * and the grabber will use the default values from the camera instead.
        */
      inline void
      setRGBCameraIntrinsics (const double rgb_focal_length_x,
                              const double rgb_focal_length_y,
                              const double rgb_principal_point_x,
                              const double rgb_principal_point_y)
      {
        rgb_focal_length_x_ = rgb_focal_length_x;
        rgb_focal_length_y_ = rgb_focal_length_y;
        rgb_principal_point_x_ = rgb_principal_point_x;
        rgb_principal_point_y_ = rgb_principal_point_y;
      }

      /** \brief Get the RGB camera parameters (fx, fy, cx, cy)
        * \param[out] rgb_focal_length_x the RGB focal length (fx)
        * \param[out] rgb_focal_length_y the RGB focal length (fy)
        * \param[out] rgb_principal_point_x the RGB principal point (cx)
        * \param[out] rgb_principal_point_y the RGB principal point (cy)
        */
      inline void
      getRGBCameraIntrinsics (double &rgb_focal_length_x,
                              double &rgb_focal_length_y,
                              double &rgb_principal_point_x,
                              double &rgb_principal_point_y) const
      {
        rgb_focal_length_x = rgb_focal_length_x_;
        rgb_focal_length_y = rgb_focal_length_y_;
        rgb_principal_point_x = rgb_principal_point_x_;
        rgb_principal_point_y = rgb_principal_point_y_;
      }


      /** \brief Set the RGB image focal length (fx = fy).
        * \param[in] rgb_focal_length the RGB focal length (assumes fx = fy)
        * Setting the parameter to a non-finite value (e.g., NaN, Inf) invalidates it
        * and the grabber will use the default values from the camera instead.
        * These parameters will be used for XYZRGBA clouds.
        */
      inline void
      setRGBFocalLength (const double rgb_focal_length)
      {
        rgb_focal_length_x_ = rgb_focal_length_y_ = rgb_focal_length;
      }

      /** \brief Set the RGB image focal length
        * \param[in] rgb_focal_length_x the RGB focal length (fx)
        * \param[in] rgb_focal_length_y the RGB focal length (fy)
        * Setting the parameters to non-finite values (e.g., NaN, Inf) invalidates them
        * and the grabber will use the default values from the camera instead.
        * These parameters will be used for XYZRGBA clouds.
        */
      inline void
      setRGBFocalLength (const double rgb_focal_length_x, const double rgb_focal_length_y)
      {
        rgb_focal_length_x_ = rgb_focal_length_x;
        rgb_focal_length_y_ = rgb_focal_length_y;
      }

      /** \brief Return the RGB focal length parameters (fx, fy)
        * \param[out] rgb_focal_length_x the RGB focal length (fx)
        * \param[out] rgb_focal_length_y the RGB focal length (fy)
        */
      inline void
      getRGBFocalLength (double &rgb_focal_length_x, double &rgb_focal_length_y) const
      {
        rgb_focal_length_x = rgb_focal_length_x_;
        rgb_focal_length_y = rgb_focal_length_y_;
      }

      /** \brief Set the Depth camera parameters (fx, fy, cx, cy)
        * \param[in] depth_focal_length_x the Depth focal length (fx)
        * \param[in] depth_focal_length_y the Depth focal length (fy)
        * \param[in] depth_principal_point_x the Depth principal point (cx)
        * \param[in] depth_principal_point_y the Depth principal point (cy)
        * Setting the parameters to non-finite values (e.g., NaN, Inf) invalidates them
        * and the grabber will use the default values from the camera instead.
        */
      inline void
      setDepthCameraIntrinsics (const double depth_focal_length_x,
                                const double depth_focal_length_y,
                                const double depth_principal_point_x,
                                const double depth_principal_point_y)
      {
        depth_focal_length_x_ = depth_focal_length_x;
        depth_focal_length_y_ = depth_focal_length_y;
        depth_principal_point_x_ = depth_principal_point_x;
        depth_principal_point_y_ = depth_principal_point_y;
      }

      /** \brief Get the Depth camera parameters (fx, fy, cx, cy)
        * \param[out] depth_focal_length_x the Depth focal length (fx)
        * \param[out] depth_focal_length_y the Depth focal length (fy)
        * \param[out] depth_principal_point_x the Depth principal point (cx)
        * \param[out] depth_principal_point_y the Depth principal point (cy)
        */
      inline void
      getDepthCameraIntrinsics (double &depth_focal_length_x,
                                double &depth_focal_length_y,
                                double &depth_principal_point_x,
                                double &depth_principal_point_y) const
      {
        depth_focal_length_x = depth_focal_length_x_;
        depth_focal_length_y = depth_focal_length_y_;
        depth_principal_point_x = depth_principal_point_x_;
        depth_principal_point_y = depth_principal_point_y_;
      }

      /** \brief Set the Depth image focal length (fx = fy).
        * \param[in] depth_focal_length the Depth focal length (assumes fx = fy)
        * Setting the parameter to a non-finite value (e.g., NaN, Inf) invalidates it
        * and the grabber will use the default values from the camera instead.
        */
      inline void
      setDepthFocalLength (const double depth_focal_length)
      {
        depth_focal_length_x_ = depth_focal_length_y_ = depth_focal_length;
      }


      /** \brief Set the Depth image focal length
        * \param[in] depth_focal_length_x the Depth focal length (fx)
        * \param[in] depth_focal_length_y the Depth focal length (fy)
        * Setting the parameter to non-finite values (e.g., NaN, Inf) invalidates them
        * and the grabber will use the default values from the camera instead.
        */
      inline void
      setDepthFocalLength (const double depth_focal_length_x, const double depth_focal_length_y)
      {
        depth_focal_length_x_ = depth_focal_length_x;
        depth_focal_length_y_ = depth_focal_length_y;
      }

      /** \brief Return the Depth focal length parameters (fx, fy)
        * \param[out] depth_focal_length_x the Depth focal length (fx)
        * \param[out] depth_focal_length_y the Depth focal length (fy)
        */
      inline void
      getDepthFocalLength (double &depth_focal_length_x, double &depth_focal_length_y) const
      {
        depth_focal_length_x = depth_focal_length_x_;
        depth_focal_length_y = depth_focal_length_y_;
      }

      /** \brief Convert vector of raw shift values to depth values
        * \param[in] shift_data_ptr input shift data
        * \param[out] depth_data_ptr generated depth data
        * \param[in] size of shift and depth buffer
        */
      inline void
      convertShiftToDepth (
          const std::uint16_t* shift_data_ptr,
          std::uint16_t* depth_data_ptr,
          std::size_t size) const
      {
        // get openni device instance
        auto openni_device = this->getDevice ();

        const std::uint16_t* shift_data_it = shift_data_ptr;
        std::uint16_t* depth_data_it = depth_data_ptr;

        // shift-to-depth lookup
        for (std::size_t i=0; i<size; ++i)
        {
          *depth_data_it = openni_device->shiftToDepth(*shift_data_it);

          shift_data_it++;
          depth_data_it++;
        }

      }


    protected:
      /** \brief On initialization processing. */
      void
      onInit (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode);

      /** \brief Sets up an OpenNI device. */
      void
      setupDevice (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode);

      /** \brief Update mode maps. */
      void
      updateModeMaps ();

      /** \brief Start synchronization. */
      void
      startSynchronization ();

      /** \brief Stop synchronization. */
      void
      stopSynchronization ();

      /** \brief Map config modes. */
      bool
      mapConfigMode2XnMode (int mode, XnMapOutputMode &xnmode) const;

      // callback methods
      /** \brief RGB image callback. */
      virtual void
      imageCallback (openni_wrapper::Image::Ptr image, void* cookie);

      /** \brief Depth image callback. */
      virtual void
      depthCallback (openni_wrapper::DepthImage::Ptr depth_image, void* cookie);

      /** \brief IR image callback. */
      virtual void
      irCallback (openni_wrapper::IRImage::Ptr ir_image, void* cookie);

      /** \brief RGB + Depth image callback. */
      virtual void
      imageDepthImageCallback (const openni_wrapper::Image::Ptr &image,
                               const openni_wrapper::DepthImage::Ptr &depth_image);

      /** \brief IR + Depth image callback. */
      virtual void
      irDepthImageCallback (const openni_wrapper::IRImage::Ptr &image,
                            const openni_wrapper::DepthImage::Ptr &depth_image);

      /** \brief Process changed signals. */
      void
      signalsChanged () override;

      // helper methods

      /** \brief Check if the RGB and Depth images are required to be synchronized or not. */
      virtual void
      checkImageAndDepthSynchronizationRequired ();

      /** \brief Check if the RGB image stream is required or not. */
      virtual void
      checkImageStreamRequired ();

      /** \brief Check if the depth stream is required or not. */
      virtual void
      checkDepthStreamRequired ();

      /** \brief Check if the IR image stream is required or not. */
      virtual void
      checkIRStreamRequired ();


      /** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
        * \param[in] depth the depth image to convert
        */
      pcl::PointCloud<pcl::PointXYZ>::Ptr
      convertToXYZPointCloud (const openni_wrapper::DepthImage::Ptr &depth) const;

      /** \brief Convert a Depth + RGB image pair to a pcl::PointCloud<PointT>
        * \param[in] image the RGB image to convert
        * \param[in] depth_image the depth image to convert
        */
      template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
      convertToXYZRGBPointCloud (const openni_wrapper::Image::Ptr &image,
                                 const openni_wrapper::DepthImage::Ptr &depth_image) const;

      /** \brief Convert a Depth + Intensity image pair to a pcl::PointCloud<pcl::PointXYZI>
        * \param[in] image the IR image to convert
        * \param[in] depth_image the depth image to convert
        */
      pcl::PointCloud<pcl::PointXYZI>::Ptr
      convertToXYZIPointCloud (const openni_wrapper::IRImage::Ptr &image,
                               const openni_wrapper::DepthImage::Ptr &depth_image) const;


      Synchronizer<openni_wrapper::Image::Ptr, openni_wrapper::DepthImage::Ptr > rgb_sync_;
      Synchronizer<openni_wrapper::IRImage::Ptr, openni_wrapper::DepthImage::Ptr > ir_sync_;

      /** \brief The actual openni device. */
      openni_wrapper::OpenNIDevice::Ptr device_;

      std::string rgb_frame_id_;
      std::string depth_frame_id_;
      unsigned image_width_;
      unsigned image_height_;
      unsigned depth_width_;
      unsigned depth_height_;

      bool image_required_;
      bool depth_required_;
      bool ir_required_;
      bool sync_required_;

      boost::signals2::signal<sig_cb_openni_image>* image_signal_;
      boost::signals2::signal<sig_cb_openni_depth_image>* depth_image_signal_;
      boost::signals2::signal<sig_cb_openni_ir_image>* ir_image_signal_;
      boost::signals2::signal<sig_cb_openni_image_depth_image>* image_depth_image_signal_;
      boost::signals2::signal<sig_cb_openni_ir_depth_image>* ir_depth_image_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud>* point_cloud_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud_i>* point_cloud_i_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud_rgb>* point_cloud_rgb_signal_;
      boost::signals2::signal<sig_cb_openni_point_cloud_rgba>* point_cloud_rgba_signal_;

      struct modeComp
      {

        bool operator () (const XnMapOutputMode& mode1, const XnMapOutputMode & mode2) const
        {
          if (mode1.nXRes < mode2.nXRes)
            return true;
          if (mode1.nXRes > mode2.nXRes)
            return false;
          if (mode1.nYRes < mode2.nYRes)
            return true;
          if (mode1.nYRes > mode2.nYRes)
            return false;
          return (mode1.nFPS < mode2.nFPS);
        }
      } ;
      std::map<int, XnMapOutputMode> config2xn_map_;

      openni_wrapper::OpenNIDevice::CallbackHandle depth_callback_handle;
      openni_wrapper::OpenNIDevice::CallbackHandle image_callback_handle;
      openni_wrapper::OpenNIDevice::CallbackHandle ir_callback_handle;
      bool running_;

      mutable unsigned rgb_array_size_;
      mutable unsigned depth_buffer_size_;
      mutable boost::shared_array<unsigned char> rgb_array_;
      mutable boost::shared_array<unsigned short> depth_buffer_;
      mutable boost::shared_array<unsigned short> ir_buffer_;

      /** \brief The RGB image focal length (fx). */
      double rgb_focal_length_x_;
      /** \brief The RGB image focal length (fy). */
      double rgb_focal_length_y_;
      /** \brief The RGB image principal point (cx). */
      double rgb_principal_point_x_;
      /** \brief The RGB image principal point (cy). */
      double rgb_principal_point_y_;
      /** \brief The depth image focal length (fx). */
      double depth_focal_length_x_;
      /** \brief The depth image focal length (fy). */
      double depth_focal_length_y_;
      /** \brief The depth image principal point (cx). */
      double depth_principal_point_x_;
      /** \brief The depth image principal point (cy). */
      double depth_principal_point_y_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  openni_wrapper::OpenNIDevice::Ptr
  OpenNIGrabber::getDevice () const
  {
    return device_;
  }

} // namespace pcl
#endif // HAVE_OPENNI

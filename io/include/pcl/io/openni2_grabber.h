/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2014, respective authors.
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
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>

#ifdef HAVE_OPENNI2

#include <pcl/point_cloud.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni2/openni2_device.h>
#include <string>
#include <pcl/common/synchronizer.h>

#include <pcl/io/image.h>
#include <pcl/io/image_rgb24.h>
#include <pcl/io/image_yuv422.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image_ir.h>

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;

  namespace io
  {

    /** \brief Grabber for OpenNI 2 devices (i.e., Primesense PSDK, Microsoft Kinect, Asus XTion Pro/Live)
    * \ingroup io
    */
    class PCL_EXPORTS OpenNI2Grabber : public Grabber
    {
      public:
        using Ptr = shared_ptr<OpenNI2Grabber>;
        using ConstPtr = shared_ptr<const OpenNI2Grabber>;

        // Templated images
        using DepthImage = pcl::io::DepthImage;
        using IRImage = pcl::io::IRImage;
        using Image = pcl::io::Image;

        /** \brief Basic camera parameters placeholder. */
        struct CameraParameters
        {
          /** fx */
          double focal_length_x;
          /** fy */
          double focal_length_y;
          /** cx */
          double principal_point_x;
          /** cy */
          double principal_point_y;

          CameraParameters (double initValue)
            : focal_length_x (initValue), focal_length_y (initValue),
            principal_point_x (initValue),  principal_point_y (initValue)
          {}

          CameraParameters (double fx, double fy, double cx, double cy)
            : focal_length_x (fx), focal_length_y (fy), principal_point_x (cx), principal_point_y (cy)
          { }
        };

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
        using sig_cb_openni_image = void (const Image::Ptr &);
        using sig_cb_openni_depth_image = void (const DepthImage::Ptr &);
        using sig_cb_openni_ir_image = void (const IRImage::Ptr &);
        using sig_cb_openni_image_depth_image = void (const Image::Ptr &, const DepthImage::Ptr &, float) ;
        using sig_cb_openni_ir_depth_image = void (const IRImage::Ptr &, const DepthImage::Ptr &, float) ;
        using sig_cb_openni_point_cloud = void (const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);
        using sig_cb_openni_point_cloud_rgb = void (const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
        using sig_cb_openni_point_cloud_rgba = void (const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &);
        using sig_cb_openni_point_cloud_i = void (const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);

      public:
        /** \brief Constructor
        * \param[in] device_id ID of the device, which might be a serial number, bus@address, URI or the index of the device.
        * \param[in] depth_mode the mode of the depth stream
        * \param[in] image_mode the mode of the image stream
        * Depending on the value of \a device_id, the device is opened as follows:
        * * If it corresponds to a file path, the device is opened with OpenNI2DeviceManager::getFileDevice
        * * If it is an index of the form "#1234", the device is opened with OpenNI2DeviceManager::getDeviceByIndex
        * * If it corresponds to an URI, the device is opened with OpenNI2DeviceManager::getDevice
        * * If it is an empty string, the device is opened with OpenNI2DeviceManager::getAnyDevice
        * * Otherwise a pcl::IOException instance is thrown
        */
        OpenNI2Grabber (const std::string& device_id = "",
          const Mode& depth_mode = OpenNI_Default_Mode,
          const Mode& image_mode = OpenNI_Default_Mode);

        /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
        ~OpenNI2Grabber () noexcept override;

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

        /** \brief Get a boost shared pointer to the \ref OpenNIDevice object. */
        inline pcl::io::openni2::OpenNI2Device::Ptr
        getDevice () const;

        /** \brief Obtain a list of the available depth modes that this device supports. */
        std::vector<std::pair<int, pcl::io::openni2::OpenNI2VideoMode> >
        getAvailableDepthModes () const;

        /** \brief Obtain a list of the available image modes that this device supports. */
        std::vector<std::pair<int, pcl::io::openni2::OpenNI2VideoMode> >
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
          rgb_parameters_ = CameraParameters (
            rgb_focal_length_x, rgb_focal_length_y,
            rgb_principal_point_x, rgb_principal_point_y);
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
          rgb_focal_length_x = rgb_parameters_.focal_length_x;
          rgb_focal_length_y = rgb_parameters_.focal_length_y;
          rgb_principal_point_x = rgb_parameters_.principal_point_x;
          rgb_principal_point_y = rgb_parameters_.principal_point_y;
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
          rgb_parameters_.focal_length_x = rgb_focal_length;
          rgb_parameters_.focal_length_y = rgb_focal_length;
        }

        /** \brief Set the RGB image focal length
        * \param[in] rgb_focal_length_x the RGB focal length (fx)
        * \param[in] rgb_focal_ulength_y the RGB focal length (fy)
        * Setting the parameters to non-finite values (e.g., NaN, Inf) invalidates them
        * and the grabber will use the default values from the camera instead.
        * These parameters will be used for XYZRGBA clouds.
        */
        inline void
        setRGBFocalLength (const double rgb_focal_length_x, const double rgb_focal_length_y)
        {
          rgb_parameters_.focal_length_x = rgb_focal_length_x;
          rgb_parameters_.focal_length_y = rgb_focal_length_y;
        }

        /** \brief Return the RGB focal length parameters (fx, fy)
        * \param[out] rgb_focal_length_x the RGB focal length (fx)
        * \param[out] rgb_focal_length_y the RGB focal length (fy)
        */
        inline void
        getRGBFocalLength (double &rgb_focal_length_x, double &rgb_focal_length_y) const
        {
          rgb_focal_length_x = rgb_parameters_.focal_length_x;
          rgb_focal_length_y = rgb_parameters_.focal_length_y;
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
          depth_parameters_ = CameraParameters (
            depth_focal_length_x, depth_focal_length_y,
            depth_principal_point_x, depth_principal_point_y);
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
          depth_focal_length_x = depth_parameters_.focal_length_x;
          depth_focal_length_y = depth_parameters_.focal_length_y;
          depth_principal_point_x = depth_parameters_.principal_point_x;
          depth_principal_point_y = depth_parameters_.principal_point_y;
        }

        /** \brief Set the Depth image focal length (fx = fy).
        * \param[in] depth_focal_length the Depth focal length (assumes fx = fy)
        * Setting the parameter to a non-finite value (e.g., NaN, Inf) invalidates it
        * and the grabber will use the default values from the camera instead.
        */
        inline void
        setDepthFocalLength (const double depth_focal_length)
        {
          depth_parameters_.focal_length_x = depth_focal_length;
          depth_parameters_.focal_length_y = depth_focal_length;
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
          depth_parameters_.focal_length_x = depth_focal_length_x;
          depth_parameters_.focal_length_y = depth_focal_length_y;
        }

        /** \brief Return the Depth focal length parameters (fx, fy)
        * \param[out] depth_focal_length_x the Depth focal length (fx)
        * \param[out] depth_focal_length_y the Depth focal length (fy)
        */
        inline void
        getDepthFocalLength (double &depth_focal_length_x, double &depth_focal_length_y) const
        {
          depth_focal_length_x = depth_parameters_.focal_length_x;
          depth_focal_length_y = depth_parameters_.focal_length_y;
        }

      protected:

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

        // TODO: rename to mapMode2OniMode
        /** \brief Map config modes. */
        bool
        mapMode2XnMode (int mode, pcl::io::openni2::OpenNI2VideoMode& videoMode) const;

        // callback methods
        /** \brief RGB image callback. */
        virtual void
        imageCallback (pcl::io::openni2::Image::Ptr image, void* cookie);

        /** \brief Depth image callback. */
        virtual void
        depthCallback (pcl::io::openni2::DepthImage::Ptr depth_image, void* cookie);

        /** \brief IR image callback. */
        virtual void
        irCallback (pcl::io::openni2::IRImage::Ptr ir_image, void* cookie);

        /** \brief RGB + Depth image callback. */
        virtual void
        imageDepthImageCallback (const pcl::io::openni2::Image::Ptr &image,
        const pcl::io::openni2::DepthImage::Ptr &depth_image);

        /** \brief IR + Depth image callback. */
        virtual void
        irDepthImageCallback (const pcl::io::openni2::IRImage::Ptr &image,
        const pcl::io::openni2::DepthImage::Ptr &depth_image);

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


        // Point cloud conversion ///////////////////////////////////////////////

        /** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
        * \param[in] depth the depth image to convert
        */
        pcl::PointCloud<pcl::PointXYZ>::Ptr
        convertToXYZPointCloud (const pcl::io::openni2::DepthImage::Ptr &depth);

        /** \brief Convert a Depth + RGB image pair to a pcl::PointCloud<PointT>
        * \param[in] image the RGB image to convert
        * \param[in] depth_image the depth image to convert
        */
        template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
        convertToXYZRGBPointCloud (const pcl::io::openni2::Image::Ptr &image,
          const pcl::io::openni2::DepthImage::Ptr &depth_image);

        /** \brief Convert a Depth + Intensity image pair to a pcl::PointCloud<pcl::PointXYZI>
        * \param[in] image the IR image to convert
        * \param[in] depth_image the depth image to convert
        */
        pcl::PointCloud<pcl::PointXYZI>::Ptr
        convertToXYZIPointCloud (const pcl::io::openni2::IRImage::Ptr &image,
          const pcl::io::openni2::DepthImage::Ptr &depth_image);

        std::vector<std::uint8_t> color_resize_buffer_;
        std::vector<std::uint16_t> depth_resize_buffer_;
        std::vector<std::uint16_t> ir_resize_buffer_;

        // Stream callbacks /////////////////////////////////////////////////////
        void
        processColorFrame (openni::VideoStream& stream);

        void
        processDepthFrame (openni::VideoStream& stream);

        void
        processIRFrame (openni::VideoStream& stream);


        Synchronizer<pcl::io::openni2::Image::Ptr, pcl::io::openni2::DepthImage::Ptr > rgb_sync_;
        Synchronizer<pcl::io::openni2::IRImage::Ptr, pcl::io::openni2::DepthImage::Ptr > ir_sync_;

        /** \brief The actual openni device. */
        pcl::io::openni2::OpenNI2Device::Ptr device_;

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
          bool operator () (const openni::VideoMode& mode1, const openni::VideoMode & mode2) const
          {
            if (mode1.getResolutionX () < mode2.getResolutionX ())
              return true;
            if (mode1.getResolutionX () > mode2.getResolutionX ())
              return false;
            if (mode1.getResolutionY () < mode2.getResolutionY ())
              return true;
            if (mode1.getResolutionY () > mode2.getResolutionY ())
              return false;
            return (mode1.getFps () < mode2.getFps ());
          }
        };

        // Mapping from config (enum) modes to native OpenNI modes
        std::map<int, pcl::io::openni2::OpenNI2VideoMode> config2oni_map_;

        pcl::io::openni2::OpenNI2Device::CallbackHandle depth_callback_handle_;
        pcl::io::openni2::OpenNI2Device::CallbackHandle image_callback_handle_;
        pcl::io::openni2::OpenNI2Device::CallbackHandle ir_callback_handle_;
        bool running_;


        CameraParameters rgb_parameters_;
        CameraParameters depth_parameters_;

      public:
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    pcl::io::openni2::OpenNI2Device::Ptr
    OpenNI2Grabber::getDevice () const
    {
      return device_;
    }

  } // namespace
}

#endif // HAVE_OPENNI2

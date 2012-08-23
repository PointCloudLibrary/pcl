/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#ifndef __PCL_IO_OPENNI_GRABBER__
#define __PCL_IO_OPENNI_GRABBER__

#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_camera/openni_device_kinect.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_ir_image.h>
#include <string>
#include <deque>
#include <pcl/common/synchronizer.h>

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;
  template <typename T> class PointCloud;

  /** \brief Grabber for OpenNI devices (i.e., Primesense PSDK, Microsoft Kinect, Asus XTion Pro/Live)
    * \author Nico Blodow <blodow@cs.tum.edu>, Suat Gedikli <gedikli@willowgarage.com>
    * \ingroup io
    */
  class PCL_EXPORTS OpenNIGrabber : public Grabber
  {
    public:

      typedef enum
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
      } Mode;

      //define callback signature typedefs
      typedef void (sig_cb_openni_image) (const boost::shared_ptr<openni_wrapper::Image>&);
      typedef void (sig_cb_openni_depth_image) (const boost::shared_ptr<openni_wrapper::DepthImage>&);
      typedef void (sig_cb_openni_ir_image) (const boost::shared_ptr<openni_wrapper::IRImage>&);
      typedef void (sig_cb_openni_image_depth_image) (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant) ;
      typedef void (sig_cb_openni_ir_depth_image) (const boost::shared_ptr<openni_wrapper::IRImage>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant) ;
      typedef void (sig_cb_openni_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
      typedef void (sig_cb_openni_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
      typedef void (sig_cb_openni_point_cloud_rgba) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);
      typedef void (sig_cb_openni_point_cloud_i) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
      typedef void (sig_cb_openni_point_cloud_eigen) (const boost::shared_ptr<const pcl::PointCloud<Eigen::MatrixXf> >&);

    public:
      /** \brief Constructor
        * \param[in] device_id ID of the device, which might be a serial number, bus@address or the index of the device.
        * \param[in] depth_mode the mode of the depth stream
        * \param[in] image_mode the mode of the image stream
        */
      OpenNIGrabber (const std::string& device_id = "",
                     const Mode& depth_mode = OpenNI_Default_Mode,
                     const Mode& image_mode = OpenNI_Default_Mode);

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      virtual ~OpenNIGrabber () throw ();

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

      /** \brief Get a boost shared pointer to the \ref OpenNIDevice object. */
      inline boost::shared_ptr<openni_wrapper::OpenNIDevice>
      getDevice () const;

      /** \brief Obtain a list of the available depth modes that this device supports. */
      std::vector<std::pair<int, XnMapOutputMode> >
      getAvailableDepthModes () const;

      /** \brief Obtain a list of the available image modes that this device supports. */
      std::vector<std::pair<int, XnMapOutputMode> >
      getAvailableImageModes () const;

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
      imageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie);

      /** \brief Depth image callback. */
      virtual void
      depthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);

      /** \brief IR image callback. */
      virtual void
      irCallback (boost::shared_ptr<openni_wrapper::IRImage> ir_image, void* cookie);

      /** \brief RGB + Depth image callback. */
      virtual void
      imageDepthImageCallback (const boost::shared_ptr<openni_wrapper::Image> &image,
                               const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image);

      /** \brief IR + Depth image callback. */
      virtual void
      irDepthImageCallback (const boost::shared_ptr<openni_wrapper::IRImage> &image,
                            const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image);

      /** \brief Process changed signals. */
      virtual void
      signalsChanged ();

      // helper methods

      /** \brief Check if the RGB and Depth images are required to be synchronized or not. */
      virtual inline void
      checkImageAndDepthSynchronizationRequired ();

      /** \brief Check if the RGB image stream is required or not. */
      virtual inline void
      checkImageStreamRequired ();

      /** \brief Check if the depth stream is required or not. */
      virtual inline void
      checkDepthStreamRequired ();

      /** \brief Check if the IR image stream is required or not. */
      virtual inline void
      checkIRStreamRequired ();

      /** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
        * \param[in] depth the depth image to convert 
        */
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >
      convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage> &depth) const;

      /** \brief Convert a Depth + RGB image pair to a pcl::PointCloud<PointT>
        * \param[in] image the RGB image to convert 
        * \param[in] depth_image the depth image to convert 
        */
      template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
      convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
                                 const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;

      /** \brief Convert a Depth + Intensity image pair to a pcl::PointCloud<pcl::PointXYZI>
        * \param[in] image the IR image to convert 
        * \param[in] depth_image the depth image to convert 
        */
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
      convertToXYZIPointCloud (const boost::shared_ptr<openni_wrapper::IRImage> &image,
                               const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;

      /** \brief Convert a pair of depth + RGB images to a PointCloud<MatrixXf> dataset.
        * \param[in] image the RGB image
        * \param[in] depth_image the depth image
        * \return a PointCloud<MatrixXf> dataset
        */
      boost::shared_ptr<pcl::PointCloud<Eigen::MatrixXf> >
      convertToEigenPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
                                const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;
      
      Synchronizer<boost::shared_ptr<openni_wrapper::Image>, boost::shared_ptr<openni_wrapper::DepthImage> > rgb_sync_;
      Synchronizer<boost::shared_ptr<openni_wrapper::IRImage>, boost::shared_ptr<openni_wrapper::DepthImage> > ir_sync_;

      /** \brief The actual openni device. */
      boost::shared_ptr<openni_wrapper::OpenNIDevice> device_;

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
      boost::signals2::signal<sig_cb_openni_point_cloud_eigen>* point_cloud_eigen_signal_;

      struct modeComp
      {

        bool operator () (const XnMapOutputMode& mode1, const XnMapOutputMode & mode2) const
        {
          if (mode1.nXRes < mode2.nXRes)
            return true;
          else if (mode1.nXRes > mode2.nXRes)
            return false;
          else if (mode1.nYRes < mode2.nYRes)
            return true;
          else if (mode1.nYRes > mode2.nYRes)
            return false;
          else if (mode1.nFPS < mode2.nFPS)
            return true;
          else
            return false;
        }
      } ;
      std::map<int, XnMapOutputMode> config2xn_map_;

      openni_wrapper::OpenNIDevice::CallbackHandle depth_callback_handle;
      openni_wrapper::OpenNIDevice::CallbackHandle image_callback_handle;
      openni_wrapper::OpenNIDevice::CallbackHandle ir_callback_handle;
      bool running_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  boost::shared_ptr<openni_wrapper::OpenNIDevice>
  OpenNIGrabber::getDevice () const
  {
    return device_;
  }

} // namespace pcl
#endif // __PCL_IO_OPENNI_GRABBER__
#endif // HAVE_OPENNI

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Nico Blodow (blodow@cs.tum.edu), Suat Gedikli (gedikli@willowgarage.com)
 */

#include "pcl/pcl_config.h"
#ifdef HAVE_OPENNI

#ifndef __PCL_IO_OPENNI_GRABBER__
#define __PCL_IO_OPENNI_GRABBER__

#include <pcl/io/grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_camera/openni_device_kinect.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <string>
#include <deque>
#include <boost/thread/mutex.hpp>
#include <pcl/common/synchronizer.h>

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  template <typename T> class PointCloud;

  /** /brief
   * /ingroup io
   */
  class PCL_EXPORTS OpenNIGrabber : public Grabber
  {
  public:

    typedef enum
    {
      OpenNI_Default_Mode = 0, /*This can depend on the device. For now all devices (PSDK, Xtion, Kinect) its VGA@30Hz*/
      OpenNI_SXGA_15Hz = 1,  /*Only supported by the Kinect*/
      OpenNI_VGA_30Hz = 2,   /*Supported by PSDK, Xtion and Kinect*/
      OpenNI_VGA_25Hz = 3,   /*Supportged by PSDK and Xtion*/
      OpenNI_QVGA_25Hz = 4,  /*Supported by PSDK and Xtion*/
      OpenNI_QVGA_30Hz = 5,  /*Supported by PSDK, Xtion and Kinect*/
      OpenNI_QVGA_60Hz = 6,  /*Supported by PSDK and Xtion*/
      OpenNI_QQVGA_25Hz = 7, /*Not supported -> using software downsampling*/
      OpenNI_QQVGA_30Hz = 8, /*Not supported -> using software downsampling*/
      OpenNI_QQVGA_60Hz = 9  /*Not supported -> using software downsampling*/
    } Mode;

    //define callback signature typedefs
    typedef void (sig_cb_openni_image) (const boost::shared_ptr<openni_wrapper::Image>&);
    typedef void (sig_cb_openni_depth_image) (const boost::shared_ptr<openni_wrapper::DepthImage>&);
    typedef void (sig_cb_openni_image_depth_image) (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant) ;
    typedef void (sig_cb_openni_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
    typedef void (sig_cb_openni_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);

  public:
    //enable using some openni parameters in the constructor.
    OpenNIGrabber (const std::string& device_id = "", const Mode& depth_mode = OpenNI_Default_Mode, const Mode& image_mode = OpenNI_Default_Mode) throw (openni_wrapper::OpenNIException);

    virtual ~OpenNIGrabber ();
    virtual void start ();
    virtual void stop ();
    virtual bool isRunning () const;
    virtual std::string getName () const;

    inline const openni_wrapper::OpenNIDevice& getDevice () const;
    std::vector<std::pair<int, XnMapOutputMode> > getAvailableDepthModes () const;
    std::vector<std::pair<int, XnMapOutputMode> > getAvailableImageModes () const;
  private:
    bool onInit (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode);
    bool setupDevice (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode);
    void updateModeMaps ();
    void startSynchronization ();
    void stopSynchronization ();

    bool mapConfigMode2XnMode (int mode, XnMapOutputMode &xnmode) const;

    // callback methods
    void imageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie);
    void depthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);
    void imageDepthImageCallback (const boost::shared_ptr<openni_wrapper::Image> &image,
                                  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image);

    virtual void signalsChanged ();

    // helper methods

    virtual inline void checkImageAndDepthSynchronizationRequired ();
    virtual inline void checkImageStreamRequired ();
    virtual inline void checkDepthStreamRequired ();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage> &depth) const;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
                                                                                     const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;

    Synchronizer<boost::shared_ptr<openni_wrapper::Image>, boost::shared_ptr<openni_wrapper::DepthImage> > sync_;

    /** \brief the actual openni device*/
    boost::shared_ptr<openni_wrapper::OpenNIDevice> device_;

    std::string rgb_frame_id_;
    std::string depth_frame_id_;
    unsigned image_width_;
    unsigned image_height_;
    unsigned depth_width_;
    unsigned depth_height_;

    bool image_required_;
    bool depth_required_;
    bool sync_required_;

    boost::signals2::signal<sig_cb_openni_image >* image_signal_;
    boost::signals2::signal<sig_cb_openni_depth_image >* depth_image_signal_;
    boost::signals2::signal<sig_cb_openni_image_depth_image>* image_depth_image_signal_;
    boost::signals2::signal<sig_cb_openni_point_cloud >* point_cloud_signal_;
    boost::signals2::signal<sig_cb_openni_point_cloud_rgb >* point_cloud_rgb_signal_;

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
    bool running_;
  } ;

  const openni_wrapper::OpenNIDevice&
  OpenNIGrabber::getDevice () const
  {
    return (*device_);
  }

} // namespace pcl
#endif // __PCL_IO_OPENNI_GRABBER__
#endif // HAVE_OPENNI

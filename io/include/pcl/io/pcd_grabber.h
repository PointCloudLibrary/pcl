/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#include <pcl/common/io.h>
#include <pcl/io/grabber.h>
#include <pcl/io/file_grabber.h>
#include <pcl/common/time_trigger.h>
#include <pcl/conversions.h>
#include <pcl/memory.h>

#ifdef HAVE_OPENNI
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#endif

#include <string>
#include <vector>

namespace pcl
{
  /** \brief Base class for PCD file grabber.
    * \ingroup io
    */
  class PCL_EXPORTS PCDGrabberBase : public Grabber
  {
    public:
      /** \brief Constructor taking just one PCD file or one TAR file containing multiple PCD files.
        * \param[in] pcd_file path to the PCD file
        * \param[in] frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
        * \param[in] repeat whether to play PCD file in an endless loop or not.
        */
      PCDGrabberBase (const std::string& pcd_file, float frames_per_second, bool repeat);

      /** \brief Constructor taking a list of paths to PCD files, that are played in the order they appear in the list.
        * \param[in] pcd_files vector of paths to PCD files.
        * \param[in] frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
        * \param[in] repeat whether to play PCD file in an endless loop or not.
        */
      PCDGrabberBase (const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat);

      /** \brief Virtual destructor. */
      ~PCDGrabberBase () noexcept override;

      /** \brief Starts playing the list of PCD files if frames_per_second is > 0. Otherwise it works as a trigger: publishes only the next PCD file in the list. */
      void
      start () override;

      /** \brief Stops playing the list of PCD files if frames_per_second is > 0. Otherwise the method has no effect. */
      void
      stop () override;

      /** \brief Triggers a callback with new data */
      virtual void
      trigger ();

      /** \brief Indicates whether the grabber is streaming or not.
        * \return true if grabber is started and hasn't run out of PCD files.
        */
      bool
      isRunning () const override;

      /** \return The name of the grabber */
      std::string
      getName () const override;

      /** \brief Rewinds to the first PCD file in the list.*/
      virtual void
      rewind ();

      /** \brief Returns the frames_per_second. 0 if grabber is trigger-based */
      float
      getFramesPerSecond () const override;

      /** \brief Returns whether the repeat flag is on */
      bool
      isRepeatOn () const;

      /** \brief Get cloud (in ROS form) at a particular location */
      bool
      getCloudAt (std::size_t idx,
                  pcl::PCLPointCloud2 &blob,
                  Eigen::Vector4f &origin,
                  Eigen::Quaternionf &orientation) const;

      /** \brief Returns the size */
      std::size_t
      numFrames () const;

    private:
      virtual void
      publish (const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation, const std::string& file_name) const = 0;

      // to separate and hide the implementation from interface: PIMPL
      struct PCDGrabberImpl;
      PCDGrabberImpl* impl_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename T> class PointCloud;
  template <typename PointT> class PCDGrabber : public PCDGrabberBase, public FileGrabber<PointT>
  {
    public:
      using Ptr = shared_ptr<PCDGrabber>;
      using ConstPtr = shared_ptr<const PCDGrabber>;

      PCDGrabber (const std::string& pcd_path, float frames_per_second = 0, bool repeat = false);
      PCDGrabber (const std::vector<std::string>& pcd_files, float frames_per_second = 0, bool repeat = false);

      /** \brief Virtual destructor. */
      ~PCDGrabber () noexcept override
      {
        stop ();
      }

      // Inherited from FileGrabber
      const typename pcl::PointCloud<PointT>::ConstPtr
      operator[] (std::size_t idx) const override;

      // Inherited from FileGrabber
      std::size_t
      size () const override;
    protected:

      void
      publish (const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation, const std::string& file_name) const override;

      boost::signals2::signal<void (const typename pcl::PointCloud<PointT>::ConstPtr&)>* signal_;
      boost::signals2::signal<void (const std::string&)>* file_name_signal_;

#ifdef HAVE_OPENNI
      boost::signals2::signal<void (const openni_wrapper::DepthImage::Ptr&)>*     depth_image_signal_;
      boost::signals2::signal<void (const openni_wrapper::Image::Ptr&)>*     image_signal_;
      boost::signals2::signal<void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float constant)>*     image_depth_image_signal_;
#endif
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  PCDGrabber<PointT>::PCDGrabber (const std::string& pcd_path, float frames_per_second, bool repeat)
  : PCDGrabberBase (pcd_path, frames_per_second, repeat)
  {
    signal_ = createSignal<void (const typename pcl::PointCloud<PointT>::ConstPtr&)>();
    file_name_signal_ = createSignal<void (const std::string&)>();
#ifdef HAVE_OPENNI
    depth_image_signal_ = createSignal <void (const openni_wrapper::DepthImage::Ptr&)> ();
    image_signal_ = createSignal <void (const openni_wrapper::Image::Ptr&)> ();
    image_depth_image_signal_ = createSignal <void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float constant)> ();
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  PCDGrabber<PointT>::PCDGrabber (const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat)
    : PCDGrabberBase (pcd_files, frames_per_second, repeat), signal_ ()
  {
    signal_ = createSignal<void (const typename pcl::PointCloud<PointT>::ConstPtr&)>();
    file_name_signal_ = createSignal<void (const std::string&)>();
#ifdef HAVE_OPENNI
    depth_image_signal_ = createSignal <void (const openni_wrapper::DepthImage::Ptr&)> ();
    image_signal_ = createSignal <void (const openni_wrapper::Image::Ptr&)> ();
    image_depth_image_signal_ = createSignal <void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float constant)> ();
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> const typename pcl::PointCloud<PointT>::ConstPtr
  PCDGrabber<PointT>::operator[] (std::size_t idx) const
  {
    pcl::PCLPointCloud2 blob;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    getCloudAt (idx, blob, origin, orientation);
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    pcl::fromPCLPointCloud2 (blob, *cloud);
    cloud->sensor_origin_ = origin;
    cloud->sensor_orientation_ = orientation;
    return (cloud);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT> std::size_t
  PCDGrabber<PointT>::size () const
  {
    return (numFrames ());
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  PCDGrabber<PointT>::publish (const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation, const std::string& file_name) const
  {
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    pcl::fromPCLPointCloud2 (blob, *cloud);
    cloud->sensor_origin_ = origin;
    cloud->sensor_orientation_ = orientation;

    signal_->operator () (cloud);
    if (file_name_signal_->num_slots() > 0)
        file_name_signal_->operator()(file_name);

#ifdef HAVE_OPENNI
    // If dataset is not organized, return
    if (!cloud->isOrganized ())
      return;

    shared_ptr<xn::DepthMetaData> depth_meta_data (new xn::DepthMetaData);
    depth_meta_data->AllocateData (cloud->width, cloud->height);
    XnDepthPixel* depth_map = depth_meta_data->WritableData ();
    std::uint32_t k = 0;
    for (std::uint32_t i = 0; i < cloud->height; ++i)
      for (std::uint32_t j = 0; j < cloud->width; ++j)
      {
        depth_map[k] = static_cast<XnDepthPixel> ((*cloud)[k].z * 1000);
        ++k;
      }

    openni_wrapper::DepthImage::Ptr depth_image (new openni_wrapper::DepthImage (depth_meta_data, 0.075f, 525, 0, 0));
    if (depth_image_signal_->num_slots() > 0)
      depth_image_signal_->operator()(depth_image);

    // ---[ RGB special case
    std::vector<pcl::PCLPointField> fields;
    int rgba_index = pcl::getFieldIndex<PointT> ("rgb", fields);
    if (rgba_index == -1)
      rgba_index = pcl::getFieldIndex<PointT> ("rgba", fields);
    if (rgba_index >= 0)
    {
      rgba_index = fields[rgba_index].offset;

      shared_ptr<xn::ImageMetaData> image_meta_data (new xn::ImageMetaData);
      image_meta_data->AllocateData (cloud->width, cloud->height, XN_PIXEL_FORMAT_RGB24);
      XnRGB24Pixel* image_map = image_meta_data->WritableRGB24Data ();
      k = 0;
      for (std::uint32_t i = 0; i < cloud->height; ++i)
      {
        for (std::uint32_t j = 0; j < cloud->width; ++j)
        {
          // Fill r/g/b data, assuming that the order is BGRA
          pcl::RGB rgb;
          memcpy (&rgb, reinterpret_cast<const char*> (&(*cloud)[k]) + rgba_index, sizeof (RGB));
          image_map[k].nRed = static_cast<XnUInt8> (rgb.r);
          image_map[k].nGreen = static_cast<XnUInt8> (rgb.g);
          image_map[k].nBlue = static_cast<XnUInt8> (rgb.b);
          ++k;
        }
      }

      openni_wrapper::Image::Ptr image (new openni_wrapper::ImageRGB24 (image_meta_data));
      if (image_signal_->num_slots() > 0)
        image_signal_->operator()(image);

      if (image_depth_image_signal_->num_slots() > 0)
        image_depth_image_signal_->operator()(image, depth_image, 1.0f / 525.0f);
    }
#endif
  }
}

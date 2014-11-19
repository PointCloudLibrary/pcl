
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#pragma once
#ifndef __PCL_IO_IMAGE_GRABBER__
#define __PCL_IO_IMAGE_GRABBER__

#include "pcl/pcl_config.h"
#include <pcl/io/grabber.h>
#include <pcl/io/file_grabber.h>
#include <pcl/common/time_trigger.h>
#include <string>
#include <vector>
#include <pcl/conversions.h>

namespace pcl
{
  /** \brief Base class for Image file grabber.
   * \ingroup io
   */
  class PCL_EXPORTS ImageGrabberBase : public Grabber
  {
    public:
    /** \brief Constructor taking a folder of depth+[rgb] images.
     * \param[in] directory Directory which contains an ordered set of images corresponding to an [RGB]D video, stored as TIFF, PNG, JPG, or PPM files. The naming convention is: frame_[timestamp]_["depth"/"rgb"].[extension]
     * \param[in] frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
     * \param[in] repeat whether to play PCD file in an endless loop or not.
     * \param pclzf_mode
     */
    ImageGrabberBase (const std::string& directory, float frames_per_second, bool repeat, bool pclzf_mode);

    ImageGrabberBase (const std::string& depth_directory, const std::string& rgb_directory, float frames_per_second, bool repeat);
    /** \brief Constructor taking a list of paths to PCD files, that are played in the order they appear in the list.
     * \param[in] depth_image_files Path to the depth image files files.
     * \param[in] frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
     * \param[in] repeat whether to play PCD file in an endless loop or not.
     */
    ImageGrabberBase (const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat);

    /** \brief Copy constructor.
     * \param[in] src the Image Grabber base object to copy into this
     */
    ImageGrabberBase (const ImageGrabberBase &src) : Grabber (), impl_ ()
    {
      *this = src;
    }

    /** \brief Copy operator.
     * \param[in] src the Image Grabber base object to copy into this
     */
    ImageGrabberBase&
    operator = (const ImageGrabberBase &src)
    {
      impl_ = src.impl_;
      return (*this);
    }

    /** \brief Virtual destructor. */
    virtual ~ImageGrabberBase () throw ();

    /** \brief Starts playing the list of PCD files if frames_per_second is > 0. Otherwise it works as a trigger: publishes only the next PCD file in the list. */
    virtual void 
    start ();
      
    /** \brief Stops playing the list of PCD files if frames_per_second is > 0. Otherwise the method has no effect. */
    virtual void 
    stop ();
      
    /** \brief Triggers a callback with new data */
    virtual void 
    trigger ();

    /** \brief whether the grabber is started (publishing) or not.
     * \return true only if publishing.
     */
    virtual bool 
    isRunning () const;
      
    /** \return The name of the grabber */
    virtual std::string 
    getName () const;
      
    /** \brief Rewinds to the first PCD file in the list.*/
    virtual void 
    rewind ();

    /** \brief Returns the frames_per_second. 0 if grabber is trigger-based */
    virtual float 
    getFramesPerSecond () const;

    /** \brief Returns whether the repeat flag is on */
    bool 
    isRepeatOn () const;

    /** \brief Returns if the last frame is reached */
    bool
    atLastFrame () const;

    /** \brief Returns the filename of the current indexed file */
    std::string
    getCurrentDepthFileName () const;

    /** \brief Returns the filename of the previous indexed file 
     *  SDM: adding this back in, but is this useful, or confusing? */
    std::string
    getPrevDepthFileName () const;

    /** \brief Get the depth filename at a particular index */
    std::string
    getDepthFileNameAtIndex (size_t idx) const;

    /** \brief Query only the timestamp of an index, if it exists */
    bool
    getTimestampAtIndex (size_t idx, pcl::uint64_t &timestamp) const;

    /** \brief Manually set RGB image files.
     * \param[in] rgb_image_files A vector of [tiff/png/jpg/ppm] files to use as input. There must be a 1-to-1 correspondence between these and the depth images you set
     */
    void
    setRGBImageFiles (const std::vector<std::string>& rgb_image_files);

    /** \brief Define custom focal length and center pixel. This will override ANY other setting of parameters for the duration of the grabber's life, whether by factory defaults or explicitly read from a frame_[timestamp].xml file. 
     *  \param[in] focal_length_x Horizontal focal length (fx)
     *  \param[in] focal_length_y Vertical focal length (fy)
     *  \param[in] principal_point_x Horizontal coordinates of the principal point (cx)
     *  \param[in] principal_point_y Vertical coordinates of the principal point (cy)
     */
    virtual void
    setCameraIntrinsics (const double focal_length_x, 
                         const double focal_length_y, 
                         const double principal_point_x, 
                         const double principal_point_y);
    
    /** \brief Get the current focal length and center pixel. If the intrinsics have been manually set with setCameraIntrinsics, this will return those values. Else, if start () has been called and the grabber has found a frame_[timestamp].xml file, this will return the most recent values read. Else, returns factory defaults.
     *  \param[out] focal_length_x Horizontal focal length (fx)
     *  \param[out] focal_length_y Vertical focal length (fy)
     *  \param[out] principal_point_x Horizontal coordinates of the principal point (cx)
     *  \param[out] principal_point_y Vertical coordinates of the principal point (cy)
     */
    virtual void
    getCameraIntrinsics (double &focal_length_x, 
                         double &focal_length_y, 
                         double &principal_point_x, 
                         double &principal_point_y) const;

    /** \brief Define the units the depth data is stored in.
     *  Defaults to mm (0.001), meaning a brightness of 1000 corresponds to 1 m*/
    void
    setDepthImageUnits (float units);
    
    /** \brief Set the number of threads, if we wish to use OpenMP for quicker cloud population.
     *  Note that for a standard (< 4 core) machine this is unlikely to yield a drastic speedup.*/
    void
    setNumberOfThreads (unsigned int nr_threads = 0);

    protected:
    /** \brief Convenience function to see how many frames this consists of
      */
    size_t
    numFrames () const;
    
    /** \brief Gets the cloud in ROS form at location idx */
    bool
    getCloudAt (size_t idx, pcl::PCLPointCloud2 &blob, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation) const;


    private:
    virtual void 
    publish (const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) const = 0;


    // to separate and hide the implementation from interface: PIMPL
    struct ImageGrabberImpl;
    ImageGrabberImpl* impl_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename T> class PointCloud;
  template <typename PointT> class ImageGrabber : public ImageGrabberBase, public FileGrabber<PointT>
  {
    public:
    ImageGrabber (const std::string& dir, 
                  float frames_per_second = 0, 
                  bool repeat = false, 
                  bool pclzf_mode = false);

    ImageGrabber (const std::string& depth_dir, 
                  const std::string& rgb_dir, 
                  float frames_per_second = 0, 
                  bool repeat = false);

    ImageGrabber (const std::vector<std::string>& depth_image_files, 
                  float frames_per_second = 0, 
                  bool repeat = false);
      
    /** \brief Empty destructor */
    virtual ~ImageGrabber () throw () {}
    
    // Inherited from FileGrabber
    const boost::shared_ptr< const pcl::PointCloud<PointT> >
    operator[] (size_t idx) const;

    // Inherited from FileGrabber
    size_t
    size () const;

    protected:
    virtual void 
    publish (const pcl::PCLPointCloud2& blob,
             const Eigen::Vector4f& origin, 
             const Eigen::Quaternionf& orientation) const;
    boost::signals2::signal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>* signal_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ImageGrabber<PointT>::ImageGrabber (const std::string& dir, 
                                      float frames_per_second, 
                                      bool repeat, 
                                      bool pclzf_mode)
    : ImageGrabberBase (dir, frames_per_second, repeat, pclzf_mode)
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ImageGrabber<PointT>::ImageGrabber (const std::string& depth_dir, 
                                      const std::string& rgb_dir, 
                                      float frames_per_second, 
                                      bool repeat)
    : ImageGrabberBase (depth_dir, rgb_dir, frames_per_second, repeat)
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ImageGrabber<PointT>::ImageGrabber (const std::vector<std::string>& depth_image_files, 
                                      float frames_per_second, 
                                      bool repeat)
    : ImageGrabberBase (depth_image_files, frames_per_second, repeat), signal_ ()
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> const boost::shared_ptr< const pcl::PointCloud<PointT> >
  ImageGrabber<PointT>::operator[] (size_t idx) const
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
  template <typename PointT> size_t
  ImageGrabber<PointT>::size () const
  {
    return (numFrames ());
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  ImageGrabber<PointT>::publish (const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) const
  {
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    pcl::fromPCLPointCloud2 (blob, *cloud);
    cloud->sensor_origin_ = origin;
    cloud->sensor_orientation_ = orientation;

    signal_->operator () (cloud);
  }
}
#endif

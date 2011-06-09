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
 * Author: Suat Gedikli (gedikli@willowgarage.com)
 */

#include "pcl/pcl_config.h"
#ifdef HAVE_OPENNI

#ifndef __PCL_IO_PCD_GRABBER__
#define __PCL_IO_PCD_GRABBER__

#include <pcl/io/grabber.h>
#include <pcl/common/time_trigger.h>
#include <string>
#include <vector>
#include <pcl/ros/conversions.h>

namespace pcl
{
/**
 * @brief Base class for PCD file grabber.
 * @ingroup io
 */
  class PCL_EXPORTS PCDGrabberBase : public Grabber
  {
    public:
      /**
       * @brief Constuctor taking just one PCD file.
       * @param pcd_file path to the PCD file
       * @param frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
       * @param repeat wheter to play PCD file in an endless lopp or not.
       */
      PCDGrabberBase (const std::string& pcd_file, float frames_per_second, bool repeat);
      /**
       * @brief Constuctor taking a list of paths to PCD files, that are played in the order the appear in the list.
       * @param pcd_files vector of paths to PCD files.
       * @param frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
       * @param repeat wheter to play PCD file in an endless lopp or not.
       */
      PCDGrabberBase (const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat);
      /**
       * @brief virtual destructor
       */
      virtual ~PCDGrabberBase () throw ();
      /**
       * @brief starts playing the list of PCD files if frames_per_second is > 0. Otherwise it works as a trigger: publishes only the next PCD file in the list.
       */
      virtual void start () throw (pcl::PCLIOException);
      /**
       * @brief stops playing the list of PCD files if frames_per_second is > 0. Otherwise the method has no effect.
       */
      virtual void stop () throw (pcl::PCLIOException);
      /**
       * @brief wheter the grabber is started (publishing) or not.
       * @return true only if publishing.
       */
      virtual bool isRunning () const throw (pcl::PCLIOException);
      /**
       *
       * @return name of the grabber
       */
      virtual std::string getName () const;
      /**
       * @brief rewinds to the first PCD file in the list.
       */
      virtual void rewind ();
    private:
      virtual void publish (const sensor_msgs::PointCloud2& blob) const = 0;

      // to seperate and hide the implementation from interface: PIMPL
      struct PCDGrabberImpl;
      PCDGrabberImpl* impl_;
  };

  /**
   * @ingroup io
   */
  template <typename T> class PointCloud;
  //class sensor_msgs::PointCloud2;
  template <typename PointT> class PCDGrabber : public PCDGrabberBase
  {
    public:
      PCDGrabber (const std::string& pcd_path, float frames_per_second = 0, bool repeat = false);
      PCDGrabber (const std::vector<std::string>& pcd_files, float frames_per_second = 0, bool repeat = false);
    protected:
      virtual void publish (const sensor_msgs::PointCloud2& blob) const;
      boost::signals2::signal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>* signal_;
  };

  template<typename PointT>
  PCDGrabber<PointT>::PCDGrabber (const std::string& pcd_path, float frames_per_second, bool repeat)
  : PCDGrabberBase ( pcd_path, frames_per_second, repeat)
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }

  template<typename PointT>
  PCDGrabber<PointT>::PCDGrabber (const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat)
  : PCDGrabberBase ( pcd_files, frames_per_second, repeat)
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }

  template<typename PointT>
  void PCDGrabber<PointT>::publish (const sensor_msgs::PointCloud2& blob) const
  {
    typename pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT> () );
    pcl::fromROSMsg (blob, *cloud);

    signal_->operator () (cloud);
  }
}
#endif
#endif

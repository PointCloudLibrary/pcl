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

#include <pcl/common/time_trigger.h>
#include <pcl/io/grabber.h>
#include <pcl/stereo/stereo_matching.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

namespace pcl {

/** \brief Base class for Stereo file grabber.
 * \ingroup io
 */
class PCL_EXPORTS StereoGrabberBase : public Grabber {
public:
  /** \brief Constructor taking just one Stereo pair.
   * \param[in] pair_files the name of the the stereo (left + right) images.
   * \param[in] frames_per_second frames per second. If 0, start() functions like a
   *                              trigger, publishing the next pair in the list.
   * \param[in] repeat whether to play files in an endless loop or not.
   */
  StereoGrabberBase(const std::pair<std::string, std::string>& pair_files,
                    float frames_per_second,
                    bool repeat);

  /** \brief Constructor taking a list of paths to Stereo pair files, that are played in
   * the order they appear in the list.
   *
   * \param[in] files vector of paths to stereo (left+right) images.
   * \param[in] frames_per_second frames per second. If 0, start() functions like a
   *            trigger, publishing the next pair in the list.
   * \param[in] repeat whether to play files in an endless loop or not.
   */
  StereoGrabberBase(const std::vector<std::pair<std::string, std::string>>& files,
                    float frames_per_second,
                    bool repeat);

  /** \brief Virtual destructor. */
  ~StereoGrabberBase() noexcept;

  /** \brief Starts playing the list of Stereo images if frames_per_second is > 0.
   * Otherwise it works as a trigger: publishes only the next pair in the list. */
  void
  start() override;

  /** \brief Stops playing the list of Stereo images if frames_per_second is > 0.
   * Otherwise the method has no effect. */
  void
  stop() override;

  /** \brief Triggers a callback with new data */
  virtual void
  trigger();

  /** \brief whether the grabber is started (publishing) or not.
   * \return true only if publishing.
   */
  bool
  isRunning() const override;

  /** \return The name of the grabber */
  std::string
  getName() const override;

  /** \brief Rewinds to the first pair of files in the list.*/
  virtual void
  rewind();

  /** \brief Returns the frames_per_second. 0 if grabber is trigger-based */
  float
  getFramesPerSecond() const override;

  /** \brief Returns whether the repeat flag is on */
  bool
  isRepeatOn() const;

private:
  virtual void
  publish(const pcl::PCLPointCloud2& blob,
          const Eigen::Vector4f& origin,
          const Eigen::Quaternionf& orientation) const = 0;

  // to separate and hide the implementation from interface: PIMPL
  struct StereoGrabberImpl;
  StereoGrabberImpl* impl_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class StereoGrabber : public StereoGrabberBase {
public:
  StereoGrabber(const std::pair<std::string, std::string>& pair_files,
                float frames_per_second = 0,
                bool repeat = false);
  StereoGrabber(const std::vector<std::pair<std::string, std::string>>& files,
                float frames_per_second = 0,
                bool repeat = false);

protected:
  void
  publish(const pcl::PCLPointCloud2& blob,
          const Eigen::Vector4f& origin,
          const Eigen::Quaternionf& orientation) const override;

  boost::signals2::signal<void(const typename pcl::PointCloud<PointT>::ConstPtr&)>*
      signal_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
StereoGrabber<PointT>::StereoGrabber(
    const std::pair<std::string, std::string>& pair_files,
    float frames_per_second,
    bool repeat)
: StereoGrabberBase(pair_files, frames_per_second, repeat)
{
  signal_ = createSignal<void(const typename pcl::PointCloud<PointT>::ConstPtr&)>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
StereoGrabber<PointT>::StereoGrabber(
    const std::vector<std::pair<std::string, std::string>>& files,
    float frames_per_second,
    bool repeat)
: StereoGrabberBase(files, frames_per_second, repeat), signal_()
{
  signal_ = createSignal<void(const typename pcl::PointCloud<PointT>::ConstPTr&)>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
StereoGrabber<PointT>::publish(const pcl::PCLPointCloud2& blob,
                               const Eigen::Vector4f& origin,
                               const Eigen::Quaternionf& orientation) const
{
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromPCLPointCloud2(blob, *cloud);
  cloud->sensor_origin_ = origin;
  cloud->sensor_orientation_ = orientation;

  signal_->operator()(cloud);
}

} // namespace pcl

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

#include <pcl/stereo/stereo_grabber.h>
#include <pcl/PCLPointCloud2.h> // for PCLPointCloud2

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// GrabberImplementation //////////////////////
struct pcl::StereoGrabberBase::StereoGrabberImpl {
  StereoGrabberImpl(pcl::StereoGrabberBase& grabber,
                    const std::pair<std::string, std::string>& pair_files,
                    float frames_per_second,
                    bool repeat);
  StereoGrabberImpl(pcl::StereoGrabberBase& grabber,
                    const std::vector<std::pair<std::string, std::string>>& files,
                    float frames_per_second,
                    bool repeat);
  void
  trigger();
  void
  readAhead();

  pcl::StereoGrabberBase& grabber_;
  float frames_per_second_;
  bool repeat_;
  bool running_;
  std::vector<std::pair<std::string, std::string>> pair_files_;
  std::vector<std::pair<std::string, std::string>>::iterator pair_iterator_;
  TimeTrigger time_trigger_;

  pcl::PCLPointCloud2 next_cloud_;
  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;
  bool valid_;
};

///////////////////////////////////////////////////////////////////////////////////////////
pcl::StereoGrabberBase::StereoGrabberImpl::StereoGrabberImpl(
    pcl::StereoGrabberBase& grabber,
    const std::pair<std::string, std::string>& pair_files,
    float frames_per_second,
    bool repeat)
: grabber_(grabber)
, frames_per_second_(frames_per_second)
, repeat_(repeat)
, running_(false)
, time_trigger_(1.0 / static_cast<double>(std::max(frames_per_second, 0.001f)),
                [this] { trigger(); })
, valid_(false)
{
  pair_files_.push_back(pair_files);
  pair_iterator_ = pair_files_.begin();
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::StereoGrabberBase::StereoGrabberImpl::StereoGrabberImpl(
    pcl::StereoGrabberBase& grabber,
    const std::vector<std::pair<std::string, std::string>>& files,
    float frames_per_second,
    bool repeat)
: grabber_(grabber)
, frames_per_second_(frames_per_second)
, repeat_(repeat)
, running_(false)
, time_trigger_(1.0 / static_cast<double>(std::max(frames_per_second, 0.001f)),
                [this] { trigger(); })
, valid_(false)
{
  pair_files_ = files;
  pair_iterator_ = pair_files_.begin();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StereoGrabberBase::StereoGrabberImpl::readAhead()
{
  if (pair_iterator_ != pair_files_.end()) {
    // read next image pair and produce a cloud
    // valid_ = //(reader.read (*pair_iterator_, next_cloud_, origin_, orientation_,
    // pcd_version) == 0);

    if (++pair_iterator_ == pair_files_.end() && repeat_)
      pair_iterator_ = pair_files_.begin();
  }
  else
    valid_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StereoGrabberBase::StereoGrabberImpl::trigger()
{
  // If the stereo image pair was successfully read and a cloud was produced, simply
  // publish it
  if (valid_)
    grabber_.publish(next_cloud_, origin_, orientation_);

  // use remaining time, if there is time left!
  readAhead();
}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// GrabberBase //////////////////////
pcl::StereoGrabberBase::StereoGrabberBase(
    const std::pair<std::string, std::string>& pair_files,
    float frames_per_second,
    bool repeat)
: impl_(new StereoGrabberImpl(*this, pair_files, frames_per_second, repeat))
{}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::StereoGrabberBase::StereoGrabberBase(
    const std::vector<std::pair<std::string, std::string>>& files,
    float frames_per_second,
    bool repeat)
: impl_(new StereoGrabberImpl(*this, files, frames_per_second, repeat))
{}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::StereoGrabberBase::~StereoGrabberBase() noexcept
{
  stop();
  delete impl_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StereoGrabberBase::start()
{
  if (impl_->frames_per_second_ > 0) {
    impl_->running_ = true;
    impl_->time_trigger_.start();
  }
  else // manual trigger
    impl_->trigger();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StereoGrabberBase::stop()
{
  if (impl_->frames_per_second_ > 0) {
    impl_->time_trigger_.stop();
    impl_->running_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StereoGrabberBase::trigger()
{
  if (impl_->frames_per_second_ > 0)
    return;
  impl_->trigger();
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::StereoGrabberBase::isRunning() const
{
  return (impl_->running_);
}

///////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::StereoGrabberBase::getName() const
{
  return ("StereoGrabber");
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StereoGrabberBase::rewind()
{
  impl_->pair_iterator_ = impl_->pair_files_.begin();
}

///////////////////////////////////////////////////////////////////////////////////////////
float
pcl::StereoGrabberBase::getFramesPerSecond() const
{
  return (impl_->frames_per_second_);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::StereoGrabberBase::isRepeatOn() const
{
  return (impl_->repeat_);
}

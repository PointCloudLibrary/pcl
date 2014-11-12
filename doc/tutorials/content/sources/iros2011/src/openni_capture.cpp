/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include "openni_capture.h"
#include <pcl/io/pcd_io.h>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>

OpenNICapture::OpenNICapture (const std::string& device_id)
  : grabber_ (device_id)
  , most_recent_frame_ ()
  , frame_counter_ (0)
  , use_trigger_ (false)
  , trigger_ (false)
  , preview_ ()
{
  // Register a callback function to our OpenNI grabber...
  boost::function<void (const PointCloudConstPtr&)> frame_cb = boost::bind (&OpenNICapture::onNewFrame, this, _1);
  // ... and start grabbing frames
  grabber_.registerCallback (frame_cb);
  grabber_.start ();
}

OpenNICapture::~OpenNICapture ()
{
  // Stop the grabber when shutting down
  grabber_.stop ();
  if (preview_)
    delete preview_;
}

void
OpenNICapture::setTriggerMode (bool use_trigger)
{
  use_trigger_ = use_trigger;
}

const PointCloudPtr
OpenNICapture::snap ()
{
  if (use_trigger_)
  {
    if (!preview_)
    {
      // Initialize the visualizer ONLY if use_trigger is set to true
      preview_ = new pcl::visualization::PCLVisualizer ();

      boost::function<void (const pcl::visualization::KeyboardEvent&)> keyboard_cb =
        boost::bind (&OpenNICapture::onKeyboardEvent, this, _1);

      preview_->registerKeyboardCallback (keyboard_cb);
    }
    waitForTrigger ();
  }
  // Wait for a fresh frame
  int old_frame = frame_counter_;
  while (frame_counter_ == old_frame) continue;
  return (most_recent_frame_);
}

const PointCloudPtr
OpenNICapture::snapAndSave (const std::string & filename)
{
  PointCloudPtr snapped_frame = snap ();
  if (snapped_frame)
    pcl::io::savePCDFile (filename, *snapped_frame);
  return (snapped_frame);
}


void
OpenNICapture::onNewFrame (const PointCloudConstPtr &cloud)
{
  mutex_.lock ();
  ++frame_counter_;
  most_recent_frame_ = boost::make_shared<PointCloud> (*cloud); // Make a copy of the frame
  mutex_.unlock ();
}

void
OpenNICapture::onKeyboardEvent (const pcl::visualization::KeyboardEvent & event)
{
  // When the spacebar is pressed, trigger a frame capture
  mutex_.lock ();
  if (event.keyDown () && event.getKeySym () == "space")
  {
    trigger_ = true;
  }
  mutex_.unlock ();
}

/** \brief Display a preview window and wait for the user to trigger a frame capture */
void
OpenNICapture::waitForTrigger ()
{
  // Reset the trigger state
  trigger_ = false;

  int last_frame = frame_counter_;

  // Now wait for the trigger to be flipped
  while (!trigger_)
  {
    // Update the preview window on new frames
    if (frame_counter_ > last_frame)
    {
      last_frame = frame_counter_;
      if (most_recent_frame_)
      {
        mutex_.lock ();
        if (!preview_->updatePointCloud (most_recent_frame_, "preview"))
        {
          preview_->addPointCloud (most_recent_frame_, "preview");
          preview_->resetCameraViewpoint ("preview");
        }
        mutex_.unlock ();
      }
      preview_->spinOnce ();
    }
  }
}

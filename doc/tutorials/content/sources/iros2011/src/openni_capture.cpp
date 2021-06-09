#include "openni_capture.h"

#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>  // for pcl::make_shared

#include <mutex>

OpenNICapture::OpenNICapture (const std::string& device_id)
  : grabber_ (device_id)
  , preview_ ()
  , frame_counter_ (0)
  , most_recent_frame_ ()
  , use_trigger_ (false)
  , trigger_ (false)
{
  // Register a callback function to our OpenNI grabber...
  std::function<void (const PointCloudConstPtr&)> frame_cb = [this] (const PointCloudConstPtr& cloud) { onNewFrame (cloud); };
  // ... and start grabbing frames
  grabber_.registerCallback (frame_cb);
  grabber_.start ();
}

OpenNICapture::~OpenNICapture ()
{
  // Stop the grabber when shutting down
  grabber_.stop ();
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

      std::function<void (const pcl::visualization::KeyboardEvent&)> keyboard_cb =
        [this] (const pcl::visualization::KeyboardEvent& event) { onKeyboardEvent (event); };

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
  most_recent_frame_ = pcl::make_shared<PointCloud> (*cloud); // Make a copy of the frame
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

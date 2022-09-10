#include <pcl/apps/face_detection/openni_frame_source.h>
#include <pcl/memory.h>

namespace OpenNIFrameSource {

OpenNIFrameSource::OpenNIFrameSource(const std::string& device_id)
: grabber_(device_id), frame_counter_(0), active_(true)
{
  std::function<void(const PointCloudConstPtr&)> frame_cb =
      [this](const PointCloudConstPtr& cloud) { onNewFrame(cloud); };
  grabber_.registerCallback(frame_cb);
  grabber_.start();
}

OpenNIFrameSource::~OpenNIFrameSource()
{
  // Stop the grabber when shutting down
  grabber_.stop();
}

bool
OpenNIFrameSource::isActive() const
{
  return active_;
}

const PointCloudPtr
OpenNIFrameSource::snap()
{
  return most_recent_frame_;
}

void
OpenNIFrameSource::onNewFrame(const PointCloudConstPtr& cloud)
{
  mutex_.lock();
  ++frame_counter_;
  most_recent_frame_ = pcl::make_shared<PointCloud>(*cloud); // Make a copy of the frame
  mutex_.unlock();
}

void
OpenNIFrameSource::onKeyboardEvent(const pcl::visualization::KeyboardEvent& event)
{
  // When the spacebar is pressed, trigger a frame capture
  mutex_.lock();
  if (event.keyDown() && event.getKeySym() == "e") {
    active_ = false;
  }
  mutex_.unlock();
}

} // namespace OpenNIFrameSource

#pragma once

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <mutex>

namespace OpenNIFrameSource {

using PointT = pcl::PointXYZRGBA;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;

/* A simple class for capturing data from an OpenNI camera */
class PCL_EXPORTS OpenNIFrameSource {
public:
  OpenNIFrameSource(const std::string& device_id = "");
  ~OpenNIFrameSource();

  const PointCloudPtr
  snap();
  bool
  isActive() const;
  void
  onKeyboardEvent(const pcl::visualization::KeyboardEvent& event);

protected:
  void
  onNewFrame(const PointCloudConstPtr& cloud);

  pcl::OpenNIGrabber grabber_;
  PointCloudPtr most_recent_frame_;
  int frame_counter_;
  std::mutex mutex_;
  bool active_;
};

} // namespace OpenNIFrameSource

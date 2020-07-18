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

#include <pcl/common/time.h>
#include <pcl/io/dinast_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#include <thread>

using namespace std::chrono_literals;

template <typename PointType>
class DinastProcessor {
public:
  using Cloud = pcl::PointCloud<PointType>;
  using CloudConstPtr = typename Cloud::ConstPtr;

  DinastProcessor(pcl::Grabber& grabber)
  : interface(grabber), viewer("Dinast Cloud Viewer")
  {}

  void
  cloud_cb_(CloudConstPtr cloud_cb)
  {
    static unsigned count = 0;
    static double last = pcl::getTime();
    if (++count == 30) {
      double now = pcl::getTime();
      std::cout << "Average framerate: " << double(count) / double(now - last) << " Hz"
                << std::endl;
      count = 0;
      last = now;
    }
    if (!viewer.wasStopped())
      viewer.showCloud(cloud_cb);
  }

  int
  run()
  {

    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb_(cloud);
    };

    boost::signals2::connection c = interface.registerCallback(f);

    interface.start();

    while (!viewer.wasStopped()) {
      std::this_thread::sleep_for(1s);
    }

    interface.stop();

    return 0;
  }

  pcl::Grabber& interface;
  pcl::visualization::CloudViewer viewer;
};

int
main()
{
  pcl::DinastGrabber grabber;
  DinastProcessor<pcl::PointXYZI> v(grabber);
  v.run();
  return 0;
}

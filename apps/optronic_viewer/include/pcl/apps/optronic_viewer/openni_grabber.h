/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_APPS_OPTRONIC_VIEWER_OPENNI_GRABBER_H_
#define PCL_APPS_OPTRONIC_VIEWER_OPENNI_GRABBER_H_

#include <pcl/apps/optronic_viewer/qt.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/openni_grabber.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <fz_api.h>


namespace pcl
{
  namespace apps
  {
    namespace optronic_viewer
    {
      
      /** \brief Wrapper for the grabbing from an OpenNI device. Wrapper is used to run
       *         the grabbing in a separate thread (QThread).
       */
      class OpenNIGrabber : public QThread
      {
        Q_OBJECT

        public:
          /** \brief Creates a new wrapper object from the specified grabber. */
          OpenNIGrabber (pcl::Grabber * grabber);
          virtual ~OpenNIGrabber ();

          /** \brief Starts the thread. */
          void run ();

          /** \brief Callback that is used to get cloud data from the grabber. */
          void cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud);

        Q_SIGNALS:
          /** \brief Omitted when a new cloud is received. */
          void cloudReceived (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);

        private:
          /** \brief The grabber data is received from. */
          pcl::Grabber * grabber_;
      };
      
    }
  }
}

#endif // PCL_APPS_OPTRONIC_VIEWER_OPENNI_GRABBER_H_

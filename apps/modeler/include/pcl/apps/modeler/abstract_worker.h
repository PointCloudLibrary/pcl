/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#ifndef PCL_MODELER_ABSTRACT_WORKER_H_
#define PCL_MODELER_ABSTRACT_WORKER_H_

#include <pcl/apps/modeler/parameter_dialog.h>
#include <sensor_msgs/PointCloud2.h>

namespace pcl
{
  namespace modeler
  {

    class AbstractWorker : public ParameterDialog
    {
      public:
        typedef sensor_msgs::PointCloud2  PointCloud2;
        typedef PointCloud2::Ptr          PointCloud2Ptr;
        typedef PointCloud2::ConstPtr     PointCloud2ConstPtr;

        AbstractWorker(QWidget* parent=0);
        ~AbstractWorker(void);

        virtual std::string
        getName () const {return ("");}

        virtual void
        initParameters(PointCloud2Ptr input_cloud) = 0;

        virtual int
        exec();

        virtual void
        apply(PointCloud2Ptr input_cloud, PointCloud2Ptr output_cloud) const = 0;

        bool
        isParameterReady() const {return parameter_ready_;}
      protected:
        virtual void
        setupParameters() = 0;

      private:
        bool      parameter_ready_;
    };

  }
}

#endif // PCL_MODELER_ABSTRACT_WORKER_H_

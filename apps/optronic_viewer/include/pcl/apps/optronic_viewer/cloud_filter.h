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

#ifndef PCL_APPS_OPTRONIC_VIEWER_CLOUD_FILTER_H_
#define PCL_APPS_OPTRONIC_VIEWER_CLOUD_FILTER_H_

#include <pcl/apps/optronic_viewer/qt.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/openni_grabber.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <fz_api.h>

#include <string>
#include <vector>

namespace pcl
{
  namespace apps
  {
    namespace optronic_viewer
    {
      
      // Cloud -> CloudFilter -> Cloud
      //              | 
      //              |
      // Parameter ----

      /////////////////////////////////////////////////////////////////////////
      struct Parameter
      {
        std::string name;

        int type;

        union 
        {
          short value_16s;
          int value_32s;
          float value_32f;
          double value_64f;
        };
      };

      //class CloudFilterWizardPage
      //  : public QWizardPage
      //{
      //public:
      //  CloudFilterWizardPage (int id)
      //    : QWizardPage ()
      //    , id_ (id)
      //  {}
      //  virtual ~CloudFilterWizardPage () {}

      //  virtual void cleanupPage ()
      //  {
      //    std::cerr << "cleanup page " << id_ << "..." << std::endl;
      //  }

      //  int id_;
      //};

      /////////////////////////////////////////////////////////////////////////
      class CloudFilter
      {
      public:
        virtual ~CloudFilter () {}

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out) = 0;

        virtual std::vector<Parameter*>
        getParameters () = 0;

        virtual void
        setParameters (const Parameter &parameter) = 0;

        virtual std::string
        getName ()
        {
          return name_;
        }

        virtual void
        setName (std::string & name)
        {
          name_ = name;
        }

        virtual QWizardPage *
        getParameterPage () = 0;

      protected:
        std::string name_;
      };

      /////////////////////////////////////////////////////////////////////////
      class CloudFilterFactory
      {
      public:
        virtual ~CloudFilterFactory () {}

        virtual std::string
        getName ()
        {
          return filter_type_name_;
        }

        virtual CloudFilter*
        create () = 0;

      protected:
        CloudFilterFactory (std::string name)
          : filter_type_name_ (name)
        {}

      protected:
        std::string filter_type_name_;
      };

      /////////////////////////////////////////////////////////////////////////
      class VoxelGridCF
        : public CloudFilter
      {
      public:
        VoxelGridCF ();
        virtual ~VoxelGridCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);

        virtual std::vector<Parameter*>
        getParameters ();

        virtual void
        setParameters (const Parameter &parameter);
        
        virtual QWizardPage *
        getParameterPage ();

      protected:
        std::vector<pcl::apps::optronic_viewer::Parameter*> parameters_;

        float voxel_grid_size_x_;
        float voxel_grid_size_y_;
        float voxel_grid_size_z_;

        QLabel * voxel_grid_size_x_label_;
        QLabel * voxel_grid_size_y_label_;
        QLabel * voxel_grid_size_z_label_;
        QLineEdit * voxel_grid_size_x_line_edit_;
        QLineEdit * voxel_grid_size_y_line_edit_;
        QLineEdit * voxel_grid_size_z_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      class VoxelGridCFF
        : public CloudFilterFactory
      {
      public:
        VoxelGridCFF ();
        virtual ~VoxelGridCFF ();

        virtual CloudFilter*
        create ()
        {
          return (new VoxelGridCF ());
        }

      protected:
        //QWizardPage * filter_selection_page_;

      };

    }
  }
}

#endif // PCL_APPS_OPTRONIC_VIEWER_CLOUD_FILTER_H_

/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2012, Jeremie Papon.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#pragma once

#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>
#include <pcl/visualization/qvtk_compatibility.h>
#include <pcl/visualization/pcl_plotter.h>

namespace pcl
{
  namespace cloud_composer
  {
    
    class FPFHItem : public CloudComposerItem
    {
      public:

        FPFHItem (QString name, 
                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_ptr,
                     double radius);
        FPFHItem (const FPFHItem& to_copy);
        
        inline int 
        type () const override { return FPFH_ITEM; }

        FPFHItem*
        clone () const override;
        
        /** \brief Inspector additional tabs paint function - get the histogram plot widget*/
        QMap <QString, QWidget*>
        getInspectorTabs () override;
        
      private:
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_ptr_;
        double radius_;
        pcl::visualization::PCLPlotter::Ptr plot_;
        PCLQVTKWidget* qvtk_;
        QWidget *hist_page_;
    };

  }
}

Q_DECLARE_METATYPE (pcl::PointCloud<pcl::FPFHSignature33>::Ptr);
Q_DECLARE_METATYPE (pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr);

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

#ifndef CLOUD_ITEM_H_
#define CLOUD_ITEM_H_

#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>

//Typedefs to make things sane
typedef pcl::visualization::PointCloudGeometryHandler<sensor_msgs::PointCloud2> GeometryHandler;
typedef pcl::visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;

namespace pcl
{
  namespace cloud_composer
  {
    
    class CloudItem : public CloudComposerItem
    {
      public:
        //This is needed because we have members which are Vector4f and Quaternionf
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        CloudItem (const QString name,
                   const sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                   const Eigen::Vector4f& origin = Eigen::Vector4f (),
                   const Eigen::Quaternionf& orientation = Eigen::Quaternionf ());
        CloudItem (const CloudItem& to_copy);
        virtual ~CloudItem ();
        
        inline virtual int 
        type () const { return CLOUD_ITEM; }

        virtual CloudItem*
        clone () const;
        
        /** \brief Paint View function - puts this cloud item into a PCLVisualizer object*/
        virtual void
        paintView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const;
        
        /** \brief Remove from View function - removes this cloud from a PCLVisualizer object*/
        virtual void
        removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const;
        
      private:
        
        //These are just stored for convenience 
        sensor_msgs::PointCloud2::Ptr cloud_ptr_;
        ColorHandler::ConstPtr color_handler_;
        GeometryHandler::ConstPtr geometry_handler_;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr search_;

        //We keep actual local copies of these.
        Eigen::Vector4f origin_;
        Eigen::Quaternionf orientation_;
        
    };
    
    
    
  }
}

//Add PointCloud types to QT MetaType System
Q_DECLARE_METATYPE (sensor_msgs::PointCloud2::ConstPtr);
Q_DECLARE_METATYPE (GeometryHandler::ConstPtr);
Q_DECLARE_METATYPE (ColorHandler::ConstPtr);
Q_DECLARE_METATYPE (Eigen::Vector4f);
Q_DECLARE_METATYPE (Eigen::Quaternionf);
Q_DECLARE_METATYPE (pcl::search::KdTree<pcl::PointXYZ>::Ptr);
Q_DECLARE_METATYPE (pcl::PointCloud <pcl::PointXYZ>::ConstPtr);
#endif //CLOUD_ITEM_H_
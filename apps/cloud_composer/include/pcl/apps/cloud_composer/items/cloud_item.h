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
typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;

namespace pcl
{
  namespace cloud_composer
  {
    namespace PointTypeFlags
    {
      enum PointType
      {
        NONE = 0,
        XYZ = (1 << 0),
        RGB = (1 << 1),
        RGBA = (1 << 2),
        NORMAL = (1 << 3),
        HSV = (1 << 4),
        AXIS = (1 << 5), 
      };
    }
    class PCL_EXPORTS CloudItem : public CloudComposerItem
    {
      public:
        
        //This is needed because we have members which are Vector4f and Quaternionf
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        CloudItem (const QString name,
                   const pcl::PCLPointCloud2::Ptr cloud_ptr,
                   const Eigen::Vector4f& origin = Eigen::Vector4f (),
                   const Eigen::Quaternionf& orientation = Eigen::Quaternionf (),
                   bool make_templated_cloud = true);
        
        CloudItem (const CloudItem& to_copy);
        virtual ~CloudItem ();
        
        /** \brief This creates a CloudItem from a templated cloud type */
        template <typename PointT>
        static CloudItem*
        createCloudItemFromTemplate (const QString name, typename PointCloud<PointT>::Ptr cloud_ptr);
        
        /** \brief virtual data getter which calls QStandardItem::data; used to create template cloud if not created yet
         *    WARNING : This function modifies "this" - it sets up the templated type if you request one when it doesn't exist yet!
         *      It had to remain const because it is virtual, and we need to keep run-time polymorphism
         */        
        virtual QVariant
        data (int role = Qt::UserRole +1) const;
        
        /** \brief Virtual data setter which calls QStandardItem::data; used to ensure that template_cloud_set_ is set 
         *         when a templated cloud is added */
        virtual void
        setData ( const QVariant & value, int role = Qt::UserRole + 1 );
        
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
        
        /** \brief Initializes and stores a templated PointCloud object with point type matching the blob */
        void
        setTemplateCloudFromBlob ();
        
        int
        getPointType () const { return point_type_; }
        
        template <typename PointT> void
        printNumPoints () const;        
        
        virtual bool
        isSanitized () const { return is_sanitized_; }
      private:
        
        //These are just stored for convenience 
        pcl::PCLPointCloud2::Ptr cloud_blob_ptr_;
        ColorHandler::ConstPtr color_handler_;
        GeometryHandler::ConstPtr geometry_handler_;

        //We keep actual local copies of these.
        Eigen::Vector4f origin_;
        Eigen::Quaternionf orientation_;
        
        bool template_cloud_set_;

        //Internal Storage of the templated type of this cloud
        int point_type_;
        
        bool
        checkIfFinite ();
        
        bool is_sanitized_;
        
        //Helper functions which set the point_type_ based on the current point type
        template <typename PointT> inline void 
        setPointType ()
        {
          qCritical () << "CloudItem::setPointType for type with no specialization";
          point_type_ = PointTypeFlags::NONE;
        }
        
    };
    
    template <> inline void
    CloudItem::setPointType <PointXYZ> ()
    {
      point_type_ = PointTypeFlags::XYZ;  
    }
    template <> inline void
    CloudItem::setPointType <PointXYZRGB> ()
    {
      point_type_ = PointTypeFlags::XYZ | PointTypeFlags::RGB;  
    }
    template <> inline void
    CloudItem::setPointType <PointXYZRGBA> ()
    {
      point_type_ = PointTypeFlags::XYZ | PointTypeFlags::RGBA;
    }    
    
    
  }
}

//Add PointCloud types to QT MetaType System
Q_DECLARE_METATYPE (pcl::PCLPointCloud2::ConstPtr);
Q_DECLARE_METATYPE (GeometryHandler::ConstPtr);
Q_DECLARE_METATYPE (ColorHandler::ConstPtr);
Q_DECLARE_METATYPE (Eigen::Vector4f);
Q_DECLARE_METATYPE (Eigen::Quaternionf);

Q_DECLARE_METATYPE (pcl::search::KdTree<pcl::PointXYZ>::Ptr);
Q_DECLARE_METATYPE (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr);
Q_DECLARE_METATYPE (pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr);

Q_DECLARE_METATYPE (pcl::PointCloud <pcl::PointXYZ>::Ptr);
Q_DECLARE_METATYPE (pcl::PointCloud <pcl::PointXYZRGB>::Ptr);
Q_DECLARE_METATYPE (pcl::PointCloud <pcl::PointXYZRGBA>::Ptr);

#endif //CLOUD_ITEM_H_

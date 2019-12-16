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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/cloud_composer/properties_model.h>

static QStringList ITEM_TYPES_STRINGS(QStringList() 
      << "Cloud Composer Item"
      << "Cloud Item"
      << "Normals Item"
      << "FPFH Item"
      );

namespace pcl
{
  namespace cloud_composer
  {
    class PropertiesModel;
    namespace ItemDataRole 
    { 
      enum
      { 
        PROPERTIES = Qt::UserRole,
        ITEM_ID,
        CLOUD_BLOB,
        CLOUD_TEMPLATED,
        GEOMETRY_HANDLER, 
        COLOR_HANDLER,
        ORIGIN,
        ORIENTATION,
        KD_TREE_SEARCH
      };
    };
    class CloudComposerItem : public QStandardItem
    {
      public:  
        
  
        enum ItemType 
        { 
          CLOUD_COMPOSER_ITEM = QStandardItem::UserType,
          CLOUD_ITEM,
          NORMALS_ITEM,
          FPFH_ITEM
        };

        CloudComposerItem (const QString& name = "default item");
        CloudComposerItem (const CloudComposerItem& to_copy);
        ~CloudComposerItem ();
        
        inline int 
        type () const override { return CLOUD_COMPOSER_ITEM; }
      
        /** \brief Convenience function to get Item's ID String */
        inline QString
        getId () const { return data (ItemDataRole::ITEM_ID).toString (); }
        
        /** \brief Convenience function to get Item's Property Pointer */
        inline PropertiesModel*
        getPropertiesModel () const { return properties_; }
        
        /** \brief Returns all children of item type type*/
        QList <CloudComposerItem*>
        getChildren (ItemType type) const;
        
        void 
        addChild (CloudComposerItem* item_arg);
        
        CloudComposerItem*
        clone () const override;

     //   /** \brief Convenience function which pulls out a cloud Ptr of type CloudPtrT */
    //    template <typename CloudPtrT>
    //    CloudPtrT
    //    getCloudPtr () const;
        
        /** \brief Paint View function - reimpliment in item subclass if it can be displayed in PCLVisualizer*/
        virtual void
        paintView (pcl::visualization::PCLVisualizer::Ptr vis) const;
        
        /** \brief Remove from View function - reimpliment in item subclass if it can be displayed in PCLVisualizer*/
        virtual void
        removeFromView (pcl::visualization::PCLVisualizer::Ptr vis) const;
        
        /** \brief Inspector additional tabs paint function - reimpliment in item subclass if item has additional tabs to show in Inspector*/
        virtual QMap <QString, QWidget*>
        getInspectorTabs ();
              
        /** \brief The property model calls this when a property changes */
        inline void 
        propertyChanged ()
        {
          emitDataChanged ();
        }
        
        virtual bool
        isSanitized () const { return false; }
      protected:

        /** \brief Model for storing the properties of the item - pointer kept for convenience   */
        PropertiesModel* properties_;
        
    };
    
    
   
    /** \brief Templated helper class for converting QVariant to/from pointer classes   */
    template <class T> class VPtr
    {
      public:
        static T* asPtr (const QVariant& v)
        {
          return (static_cast<T *> (v.value<void *> ()));
        }

        static QVariant asQVariant (T* ptr)
        {
          return (QVariant::fromValue (static_cast<void*>(ptr)));
        }
    };
    
  }
}

using ConstItemList = QList<const pcl::cloud_composer::CloudComposerItem *>;

Q_DECLARE_METATYPE (pcl::cloud_composer::CloudComposerItem);

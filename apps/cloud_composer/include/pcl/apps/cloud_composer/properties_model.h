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

#ifndef PROPERTIES_MODEL_H_
#define PROPERTIES_MODEL_H_

#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/common/boost.h> 


namespace pcl
{
  namespace cloud_composer
  {
    class CloudComposerItem;
    class PropertiesModel : public QStandardItemModel
    {
      Q_OBJECT
      public:
        
        /** \brief Constructor used for tool parameters */
        PropertiesModel (QObject *parent = 0);
        /** \brief Constructor used for item parameters */
        PropertiesModel (CloudComposerItem* parent_item, QObject *parent = 0);
        PropertiesModel (const PropertiesModel& to_copy);
        virtual ~PropertiesModel ();
        
        /** \brief Helper function for adding a new property */
        void
        addProperty (const QString prop_name, const QVariant value, const Qt::ItemFlags flags = Qt::ItemIsSelectable, const QString category = "");
        
        /** \brief Helper function for adding a new property category */
        void
        addCategory (const QString category_name);
        
        /** \brief Helper function to get a property */
        QVariant 
        getProperty (const QString prop_name) const;
        
        void 
        copyProperties (const PropertiesModel* to_copy);
        
      public Q_SLOTS:
        void
        propertyChanged (QStandardItem* property_item);
      
      Q_SIGNALS:
        void 
        propertyChanged (const QStandardItem* property_item, const CloudComposerItem* parent_item_);
        
        
      private:
        CloudComposerItem* parent_item_;
        
     };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::PropertiesModel);
Q_DECLARE_METATYPE (pcl::cloud_composer::PropertiesModel*);

#endif //PROPERTIES_MODEL_H_

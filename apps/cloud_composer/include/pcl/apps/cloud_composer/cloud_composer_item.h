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

#ifndef CLOUD_COMPOSER_ITEM_H_
#define CLOUD_COMPOSER_ITEM_H_

#include <QStandardItem>

enum ITEM_ROLES { 
  PROPERTIES = Qt::UserRole
};

//This currently isn't used for anything, it will probably be removed
enum ITEM_TYPES { 
  CLOUD_COMPOSER_ITEM = QStandardItem::UserType,
  CLOUD_ITEM
};

namespace pcl
{
  namespace cloud_composer
  {
    class PropertyModel;
    
    class CloudComposerItem : public QStandardItem
    {
      public:
        CloudComposerItem (const QString name);
        CloudComposerItem (const CloudComposerItem& to_copy);
        virtual ~CloudComposerItem ();
        
        inline int const
        type () { return CLOUD_COMPOSER_ITEM; }

      protected:
        /** \brief Helper function for adding a new property */
        void
        addProperty (const QString prop_name, QVariant value, Qt::ItemFlags flags = Qt::ItemIsSelectable, QStandardItem* parent = 0);
        


        /** \brief Model for storing the properties of the item   */
        QStandardItemModel* properties_;
        
    };
    
    /** \brief Templated helper class for converting QVariant to/from pointer classes   */
    template <class T> class VPtr
    {
      public:
        static T* asPtr (QVariant v)
        {
          return (T *) v.value<void *> ();
        }

        static QVariant asQVariant (T* ptr)
        {
          return qVariantFromValue ( (void *) ptr);
        }
    };
    
  }
}













#endif //CLOUD_COMPOSER_ITEM_H_
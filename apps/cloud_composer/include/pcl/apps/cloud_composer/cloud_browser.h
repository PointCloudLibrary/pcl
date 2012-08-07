/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CLOUD_BROWSER_H_
#define CLOUD_BROWSER_H_

#include <pcl/apps/cloud_composer/qt.h>

class QItemSelectionModel;

namespace pcl
{
  namespace cloud_composer
  {
    /** \brief View class for displaying project composition
     * \author Jeremie Papon
     * \ingroup cloud_composer
     */
    class ProjectModel;
    class CloudBrowser : public QTreeView
    {
      Q_OBJECT
      public:
        CloudBrowser (QWidget* parent = 0);
        
        virtual void 
        setModel (QAbstractItemModel* new_model);
      
      private:  
        ProjectModel* current_project_model_;
       
    };
    
    class BackgroundDelegate : public QStyledItemDelegate
    {
      public:
        explicit 
        BackgroundDelegate (QObject *parent = 0)
          : QStyledItemDelegate(parent) {}
          
        void
        paint (QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
        
    };
    
  }
}

















#endif

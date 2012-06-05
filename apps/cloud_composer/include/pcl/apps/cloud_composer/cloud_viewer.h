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

#ifndef CLOUD_VIEWER_H_
#define CLOUD_VIEWER_H_


#include <QAbstractItemView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QFont>
#include <QItemSelection>
#include <QItemSelectionModel>
#include <QModelIndex>
#include <QRect>
#include <QSize>
#include <QPoint>
#include <QWidget>

#include <QVTKWidget.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
  namespace cloud_composer
  {

    class CloudViewer : public QAbstractItemView
    {
        Q_OBJECT

      public:
        CloudViewer (QVTKWidget* qvtk_widget, QWidget* parent = 0);

        QRect
        visualRect (const QModelIndex& index) const;
        void
        scrollTo (const QModelIndex& index, ScrollHint hint = EnsureVisible);
        QModelIndex 
        indexAt (const QPoint& point) const;

      protected slots:
        void
        dataChanged (const QModelIndex& topLeft, const QModelIndex& bottomRight);
        void
        rowsInserted (const QModelIndex& parent, int start, int end);
        void
        rowsAboutToBeRemoved (const QModelIndex& parent, int start, int end);

      protected:
        QModelIndex 
        moveCursor (QAbstractItemView::CursorAction cursorAction, Qt::KeyboardModifiers modifiers);
        int 
        horizontalOffset () const;
        int 
        verticalOffset () const;

        bool 
        isIndexHidden (const QModelIndex& index) const;

        void 
        setSelection (const QRect&, QItemSelectionModel::SelectionFlags command);
        QRegion 
        visualRegionForSelection (const QItemSelection& selection) const;
        void 
        paintEvent (QPaintEvent* event);
        void 
        resizeEvent (QResizeEvent* event);
        //   void scrollContentsBy (int dx, int dy);



      private:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
        QVTKWidget* qvtk_ptr_;
    };
  }
}
#endif

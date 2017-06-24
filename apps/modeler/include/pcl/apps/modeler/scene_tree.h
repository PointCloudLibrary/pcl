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

#ifndef PCL_MODELER_SCENE_TREE_H_
#define PCL_MODELER_SCENE_TREE_H_

#include <pcl/apps/modeler/qt.h>

namespace pcl
{
  namespace modeler
  {
    class CloudMeshItem;
    class RenderWindowItem;

    class SceneTree : public QTreeWidget
    {
      Q_OBJECT

      public:
        SceneTree(QWidget * parent = 0);
        ~SceneTree();

        virtual QSize
        sizeHint() const;

        bool 
        openPointCloud(const QString& filename);

        bool 
        savePointCloud(const QString& filename);

        void
        selectRenderWindowItem(RenderWindowItem* render_window_item);

        void
        addTopLevelItem(RenderWindowItem* render_window_item);

      public Q_SLOTS:
        // slots for file menu
        void 
        slotOpenPointCloud();

        void 
        slotImportPointCloud();

        void
        slotSavePointCloud();

        void
        slotClosePointCloud();

        // slots for edit menu
        void
        slotICPRegistration();
        void
        slotVoxelGridDownsampleFilter();
        void
        slotStatisticalOutlierRemovalFilter();
        void
        slotEstimateNormal();
        void
        slotPoissonReconstruction();

        // slots for view menu
        void
        slotCloseRenderWindow();

      Q_SIGNALS:
        void
        fileOpened(const QString& filename);

        void
        itemInsertedOrRemoved();

      protected:
        virtual void
        dropEvent(QDropEvent * event);

        virtual bool
        dropMimeData(QTreeWidgetItem * parent, int index, const QMimeData * data, Qt::DropAction action);

      private Q_SLOTS:
        void
        slotUpdateOnSelectionChange(const QItemSelection& selected, const QItemSelection& deselected);

        void
        slotUpdateOnInsertOrRemove();

        void
        slotOnItemDoubleClicked(QTreeWidgetItem * item);

      private:
        template <class T> QList<T*>
        selectedTypeItems() const;

        QList<RenderWindowItem*>
        selectedRenderWindowItems() const;

        static void
        closePointCloud(const QList<CloudMeshItem*>& items);

        virtual void
        contextMenuEvent(QContextMenuEvent *event);
    };
  }
}

#include <pcl/apps/modeler/impl/scene_tree.hpp>

#endif // PCL_MODELER_SCENE_TREE_H_

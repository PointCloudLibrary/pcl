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

#ifndef PCL_MODELER_TREE_MODEL_H_
#define PCL_MODELER_TREE_MODEL_H_

#include <QStandardItemModel>


namespace pcl
{
  namespace modeler
  {
    class TreeItem;
    class MainWindow;

    /** \brief The model class that holds the scene data.
      * \author Yangyan Li
      * \ingroup apps
      */
    class TreeModel : public QStandardItemModel
    {
      Q_OBJECT

      public:
        TreeModel (QObject* parent);

        virtual ~TreeModel ();

        virtual MainWindow*
        parent();

        void
        emitItemChanged(QStandardItem* item);

      private:

      private slots:
        void
        slotOnDataChanged(QStandardItem* item);

        void
        slotOnAboutToBeInserted(const QModelIndex& parent, int start, int end);

        void
        slotOnAboutToBeRemoved(const QModelIndex& parent, int start, int end);

        void
        slotOnInserted(const QModelIndex& parent, int start, int end);

        void
        slotOnRemoved(const QModelIndex& parent, int start, int end);

      private:

    };
  }
}

#include <pcl/apps/modeler/impl/tree_model.hpp>

#endif // PCL_MODELER_TREE_MODEL_H_

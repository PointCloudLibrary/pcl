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

#ifndef PCL_MODELER_RENDER_WIDGET_H_
#define PCL_MODELER_RENDER_WIDGET_H_

#include <QModelIndex>
#include <QVTKWidget.h>
#include <vtkSmartPointer.h>

class vtkRenderer;

namespace pcl
{
  namespace modeler
  {
    class MainWindow;

    class RenderWidget : public QVTKWidget
    {
      public:
        RenderWidget(MainWindow* main_window, size_t id, QWidget *parent = 0, Qt::WFlags flags = 0);
        ~RenderWidget();

        bool
        getActive() const {return active_;}
        void
        setActive(bool active) {active_=active;}

        size_t
        getID() const {return id_;}
        void
        setID(size_t id) {id_=id;}

        QModelIndex
        getModelIndex() const {return model_index_;}
        void
        setModelIndex(const QModelIndex& model_index) {model_index_=model_index;}

        vtkSmartPointer<vtkRenderer>
        getRenderer();

        virtual QSize
        sizeHint() const {return QSize(512, 512);}
      protected:
        virtual void
        focusInEvent ( QFocusEvent * event );

      private:
        MainWindow*       main_window_;
        bool              active_;
        size_t            id_;
        QModelIndex       model_index_;

      private:
        void
        initRenderer();
    };
  }
}

#endif // PCL_MODELER_RENDER_WIDGET_H_

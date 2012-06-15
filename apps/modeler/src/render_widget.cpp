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

#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/visualization/common/common.h>

#include <vtkTextActor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>

#include <QMenu>
#include <QContextMenuEvent>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWidget::RenderWidget(MainWindow* main_window, size_t id, QWidget *parent, Qt::WFlags flags) : 
  QVTKWidget(parent, flags),
  main_window_(main_window),
  active_(false),
  id_(id),
  TreeItem((id == 0)?(QObject::tr("Main Render Window")):(QObject::tr("Render Window %1").arg(id)))
{
  setFocusPolicy(Qt::StrongFocus);
  initRenderer();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWidget::~RenderWidget()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWidget::focusInEvent ( QFocusEvent * event )
{
  main_window_->setActiveDockWidget(this);
  QVTKWidget::focusInEvent(event);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWidget::contextMenuEvent(QContextMenuEvent *event)
{
  if (event->modifiers()&Qt::ControlModifier) {
    showContextMenu(event->globalPos());
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWidget::initRenderer()
{
  vtkSmartPointer<vtkRenderWindow> win = GetRenderWindow();
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  win->AddRenderer(renderer);

  // FPS callback
  //vtkSmartPointer<vtkTextActor> txt = vtkSmartPointer<vtkTextActor>::New ();
  //typedef pcl::visualization::FPSCallback FPSCallback;
  //vtkSmartPointer<FPSCallback> update_fps = vtkSmartPointer<FPSCallback>::New ();
  //update_fps->setTextActor (txt);
  //renderer->AddObserver (vtkCommand::EndEvent, update_fps);
  //renderer->AddActor (txt);

  // Set up render window
  win->AlphaBitPlanesOff ();
  win->PointSmoothingOff ();
  win->LineSmoothingOff ();
  win->PolygonSmoothingOff ();
  win->SwapBuffersOn ();
  win->SetStereoTypeToAnaglyph ();
  win->GetInteractor()->SetDesiredUpdateRate (30.0);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWidget::showContextMenu(const QPoint& position)
{
  QMenu menu(this);
  main_window_->addActionsToRenderWidget(&menu);
  menu.exec(position);
}

/////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkRenderer>
pcl::modeler::RenderWidget::getRenderer()
{
  return (GetRenderWindow()->GetRenderers()->GetFirstRenderer());
}

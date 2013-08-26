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

#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/render_window_item.h>
#include <pcl/apps/modeler/scene_tree.h>
#include <pcl/apps/modeler/dock_widget.h>
#include <pcl/apps/modeler/main_window.h>
#include <vtkProp.h>
#include <vtkRenderer.h>
#include <vtkBoundingBox.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeAxesActor.h>
#include <vtkRendererCollection.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWindow::RenderWindow(RenderWindowItem* render_window_item, QWidget *parent, Qt::WindowFlags flags)
  : QVTKWidget(parent, flags),
  axes_(vtkSmartPointer<vtkCubeAxesActor>::New()),
  render_window_item_(render_window_item)
{
  setFocusPolicy(Qt::StrongFocus);
  initRenderer();
  updateAxes();
  setShowAxes(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWindow::~RenderWindow()
{
  DockWidget* dock_widget = dynamic_cast<DockWidget*>(parent());
  if (dock_widget != NULL)
  {
    MainWindow::getInstance().removeDockWidget(dock_widget);
    dock_widget->deleteLater();
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::initRenderer()
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

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::focusInEvent(QFocusEvent * event)
{
  dynamic_cast<SceneTree*>(render_window_item_->treeWidget())->selectRenderWindowItem(render_window_item_);

  QVTKWidget::focusInEvent(event);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::setActive(bool flag)
{
  DockWidget* dock_widget = dynamic_cast<DockWidget*>(parent());
  if (dock_widget != NULL)
    dock_widget->setFocusBasedStyle(flag);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::setTitle(const QString& title)
{
  DockWidget* dock_widget = dynamic_cast<DockWidget*>(parent());
  if (dock_widget != NULL)
    dock_widget->setWindowTitle(title);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::render()
{
  GetRenderWindow()->Render();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::resetCamera()
{
  double bounds[6];
  GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ComputeVisiblePropBounds(bounds);
  GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera(bounds);
  render();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::getBackground(double& r, double& g, double& b)
{
  GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetBackground(r, g, b);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::setBackground(double r, double g, double b)
{
  GetRenderWindow()->GetRenderers()->GetFirstRenderer()->SetBackground(r, g, b);
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::updateAxes()
{
  vtkBoundingBox bb;

  vtkActorCollection* actors = GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActors();

  actors->InitTraversal();
  for (int i = 0, i_end = actors->GetNumberOfItems(); i < i_end; ++ i)
  {
    vtkActor* actor = actors->GetNextActor();
    if (actor == axes_.GetPointer())
      continue;

    double actor_bounds[6];
    actor->GetBounds(actor_bounds);
    bb.AddBounds(actor_bounds);
  }

  double bounds[6];
  bb.GetBounds(bounds);
  axes_->SetBounds(bounds);
  axes_->SetCamera(GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindow::setShowAxes(bool flag)
{
  if (flag)
    GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(axes_);
  else
    GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(axes_);

  return;
}

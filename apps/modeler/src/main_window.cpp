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

#include <pcl/apps/modeler/main_window.h>

#include <pcl/apps/modeler/scene_tree.h>
#include <pcl/apps/modeler/dock_widget.h>
#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/render_window_item.h>

#include <QFileInfo>
#include <vtkActor.h>
#include <vtkRenderer.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::MainWindow::MainWindow()
  : ui_(new Ui::MainWindow)
{
  ui_->setupUi(this);

  RenderWindowItem* central_render_window_item = new RenderWindowItem(ui_->scene_tree_);
  central_render_window_item->getRenderWindow()->setParent(this);
  setCentralWidget(central_render_window_item->getRenderWindow());
  ui_->scene_tree_->addTopLevelItem(central_render_window_item);

  connectFileMenuActions();
  connectViewMenuActions();
  connectEditMenuActions();

  loadGlobalSettings();

  showMaximized();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::MainWindow::~MainWindow()
{
  saveGlobalSettings();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectFileMenuActions()
{
  connect(ui_->actionOpenPointCloud, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotOpenPointCloud()));
  connect(ui_->actionImportPointCloud, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotImportPointCloud()));
  connect(ui_->actionSavePointCloud, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotSavePointCloud()));
  connect(ui_->actionClosePointCloud, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotClosePointCloud()));
  connect(ui_->scene_tree_, SIGNAL(fileOpened(QString)), this, SLOT(slotUpdateRecentFile(QString)));
  createRecentPointCloudActions();

  connect(ui_->actionOpenProject, SIGNAL(triggered()), this, SLOT(slotOpenProject()));
  connect(ui_->actionSaveProject, SIGNAL(triggered()), this, SLOT(slotSaveProject()));
  connect(ui_->actionCloseProject, SIGNAL(triggered()), this, SLOT(slotCloseProject()));
  createRecentProjectActions();

  connect(ui_->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectViewMenuActions()
{
  connect(ui_->actionCreateRenderWindow, SIGNAL(triggered()), this, SLOT(slotCreateRenderWindow()));
  connect(ui_->actionCloseRenderWindow, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotCloseRenderWindow()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectEditMenuActions()
{
  connect(ui_->actionICPRegistration, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotICPRegistration()));
  connect(ui_->actionVoxelGridDownsample, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotVoxelGridDownsampleFilter()));
  connect(ui_->actionStatisticalOutlierRemoval, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotStatisticalOutlierRemovalFilter()));
  connect(ui_->actionEstimateNormals, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotEstimateNormal()));
  connect(ui_->actionPoissonReconstruction, SIGNAL(triggered()), ui_->scene_tree_, SLOT(slotPoissonReconstruction()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOpenProject()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotSaveProject()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotCloseProject()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::slotExit() {
    saveGlobalSettings();
    qApp->exit();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::slotUpdateRecentFile(const QString& filename)
{
  recent_files_.removeAll(filename);
  recent_files_.prepend(filename);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWindowItem*
pcl::modeler::MainWindow::createRenderWindow()
{
  DockWidget* dock_widget = new DockWidget(this);
  addDockWidget(Qt::RightDockWidgetArea, dock_widget);
  dock_widget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  RenderWindowItem* render_window_item = new RenderWindowItem(ui_->scene_tree_);
  render_window_item->getRenderWindow()->setParent(dock_widget);
  dock_widget->setWidget(render_window_item->getRenderWindow());
  ui_->scene_tree_->addTopLevelItem(render_window_item);

  // add the toggle action to view menu
  QList<QAction *> actions = ui_->menuView->actions();
  ui_->menuView->insertAction(actions[actions.size()-2], dock_widget->toggleViewAction());

  return render_window_item;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::slotCreateRenderWindow()
{
  createRenderWindow();

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOpenRecentPointCloud()
{
  QAction* action = qobject_cast<QAction*>(sender());
  if (action)
    ui_->scene_tree_->openPointCloud(action->data().toString());

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOpenRecentProject()
{
  QAction* action = qobject_cast<QAction*>(sender());
  if (action)
    openProjectImpl(action->data().toString());

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::createRecentPointCloudActions()
{
  for (size_t i = 0; i < MAX_RECENT_NUMBER; ++ i)
  {
    recent_pointcloud_actions_.push_back(boost::shared_ptr<QAction>(new QAction(this)));
    ui_->menuRecentPointClouds->addAction(recent_pointcloud_actions_[i].get());
    recent_pointcloud_actions_[i]->setVisible(false);
    connect(recent_pointcloud_actions_[i].get(), SIGNAL(triggered()), this, SLOT(slotOpenRecentPointCloud()));
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::updateRecentPointCloudActions()
{
  updateRecentActions(recent_pointcloud_actions_, recent_files_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::createRecentProjectActions()
{
  for (size_t i = 0; i < MAX_RECENT_NUMBER; ++ i)
  {
    recent_project_actions_.push_back(boost::shared_ptr<QAction>(new QAction(this)));
    ui_->menuRecentPointClouds->addAction(recent_project_actions_[i].get());
    recent_project_actions_[i]->setVisible(false);
    connect(recent_project_actions_[i].get(), SIGNAL(triggered()), this, SLOT(slotOpenRecentProject()));
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::updateRecentProjectActions()
{
  updateRecentActions(recent_project_actions_, recent_projects_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::modeler::MainWindow::openProjectImpl (const QString&)
{
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::updateRecentActions (std::vector<boost::shared_ptr<QAction> >& recent_actions, QStringList& recent_items)
{
  QMutableStringListIterator recent_items_it (recent_items);
  while (recent_items_it.hasNext ())
  {
    if (!QFile::exists (recent_items_it.next ()))
      recent_items_it.remove ();
  }

  recent_items.removeDuplicates ();
  int recent_number = std::min (int (MAX_RECENT_NUMBER), recent_items.size ());
  for (int i = 0; i < recent_number; ++ i)
  {
    QString text = tr ("%1 %2").arg (i + 1).arg (recent_items[i]);
    recent_actions[i]->setText (text);
    recent_actions[i]->setData (recent_items[i]);
    recent_actions[i]->setVisible (true);
  }

  for (size_t i = recent_number, i_end = recent_actions.size (); i < i_end; ++ i)
    recent_actions[i]->setVisible (false);

  while (recent_items.size () > recent_number)
    recent_items.pop_back ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
QString 
pcl::modeler::MainWindow::getRecentFolder()
{
  QString recent_filename;
  if (!recent_projects_.empty())
    recent_filename = recent_projects_.front();
  else if (!recent_files_.empty())
    recent_filename = recent_files_.front();

  if (!recent_filename.isEmpty())
    return QFileInfo(recent_filename).path();

  return (QString("."));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::loadGlobalSettings()
{
  QSettings global_settings("PCL", "Modeler");

  recent_files_ = global_settings.value("recent_pointclouds").toStringList();
  updateRecentPointCloudActions();

  recent_projects_ = global_settings.value("recent_projects").toStringList();
  updateRecentProjectActions();

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::saveGlobalSettings()
{
  QSettings global_settings("PCL", "Modeler");

  global_settings.setValue("recent_pointclouds", recent_files_);

  global_settings.setValue("recent_projects", recent_projects_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOnWorkerStarted()
{
  statusBar()->showMessage(QString("Working thread running..."));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOnWorkerFinished()
{
  statusBar()->showMessage(QString("Working thread finished..."));
}
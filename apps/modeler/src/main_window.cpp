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

#include <pcl/apps/modeler/qt.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/tree_model.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/dock_widget.h>
#include <pcl/apps/modeler/points_item.h>
#include <pcl/apps/modeler/color_handler_switcher.h>

#include <QFileInfo>
#include <vtkActor.h>
#include <vtkRenderer.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::MainWindow::MainWindow() :
  ui_(new Ui::MainWindow),
  scene_tree_(new pcl::modeler::TreeModel(this))
{
  ui_->setupUi(this);
  ui_->treeViewSceneExplorer->setHeaderHidden(true);
  ui_->treeViewSceneExplorer->setModel(scene_tree_.get());
  ui_->treeViewSceneExplorer->setMainWindow(this);

  connect(ui_->treeViewSceneExplorer->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
    ui_->treeViewSceneExplorer, SLOT(slotOnSelectionChange(QItemSelection, QItemSelection)));

  RenderWidget* main_render_widget = new RenderWidget(this, 0);
  setCentralWidget(main_render_widget);
  scene_tree_->appendRow(main_render_widget);

  connectFileMenuActions();
  connectViewMenuActions();
  connectRenderMenuActions();
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
pcl::modeler::MainWindow::setActiveDockWidget(RenderWidget* render_widget)
{
  ui_->treeViewSceneExplorer->selectionModel()->clearSelection();
  ui_->treeViewSceneExplorer->selectionModel()->select(
    scene_tree_->indexFromItem(render_widget), QItemSelectionModel::Select);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectFileMenuActions()
{
  connect(ui_->actionOpenPointCloud, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotOpenPointCloud()));
  connect(ui_->actionImportPointCloud, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotImportPointCloud()));
  connect(ui_->actionSavePointCloud, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotSavePointCloud()));
  connect(ui_->actionClosePointCloud, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotClosePointCloud()));
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
  connect(ui_->actionChangeBackgroundColor, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotChangeBackgroundColor()));

  QList<QAction *> actions = ui_->menuView->actions();
  ui_->menuView->insertAction(actions[actions.size()-2], ui_->dockWidgetSceneExplorer->toggleViewAction());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectRenderMenuActions()
{
  connect(ui_->actionSwitchColorHandler, SIGNAL(triggered()), this, SLOT(slotSwitchColorHandler()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectEditMenuActions()
{
  connect(ui_->actionDownSampleFilter, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotDownSampleFilter()));
  connect(ui_->actionEstimateNormal, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotEstimateNormal()));
  connect(ui_->actionPoissonSurfaceReconstruction, SIGNAL(triggered()), ui_->treeViewSceneExplorer, SLOT(slotPoissonReconstruction()));
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
pcl::modeler::MainWindow::slotCreateRenderWindow()
{
  DockWidget* dock_widget = new DockWidget(this);
  dock_widget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  RenderWidget* render_widget = new RenderWidget(this, dock_widget);

  dock_widget->setWidget(render_widget);
  addDockWidget(Qt::RightDockWidgetArea, dock_widget);

  // add the toggle action to view menu
  QList<QAction *> actions = ui_->menuView->actions();
  ui_->menuView->insertAction(actions[actions.size()-2], dock_widget->toggleViewAction());

  // keep a track of the qvtk widget
  scene_tree_->appendRow(render_widget);
  render_widget->setCheckState(Qt::Checked);
  setActiveDockWidget(render_widget);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOpenRecentPointCloud()
{
  QAction* action = qobject_cast<QAction*>(sender());
  if (action)
    ui_->treeViewSceneExplorer->openPointCloud(action->data().toString());

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

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::slotSwitchColorHandler()
{
  ColorHandlerSwitcher color_handler_switcher(this);

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
QStringList&
pcl::modeler::MainWindow::getRecentFiles()
{
  return (recent_files_);
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

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

#include <ui_main_window.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/pcl_modeler.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/dock_widget.h>
#include <pcl/apps/modeler/cloud_actor.h>
#include <pcl/apps/modeler/color_handler_switcher.h>
#include <pcl/apps/modeler/downsample_worker.h>

#include <QFile>
#include <QSettings>
#include <QFileDialog>
#include <QColor>
#include <QColorDialog>
#include <QMessageBox>
#include <QStandardItem>
#include <QStandardItemModel>

#include <vtkActor.h>
#include <vtkRenderer.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::MainWindow::MainWindow() :
  ui_(new Ui::MainWindow),
  pcl_modeler_(new pcl::modeler::PCLModeler(this))
{
  ui_->setupUi(this);
  ui_->treeViewSceneExplorer->setHeaderHidden(true);
  ui_->treeViewSceneExplorer->setModel(pcl_modeler_.get());

  RenderWidget* main_render_widget = new RenderWidget(this, 0);
  setCentralWidget(main_render_widget);
  addRenderWidget(main_render_widget);

  // Set up action signals and slots
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
  active_render_widget_idx_ = 0;
  for (size_t i = 0, i_end = render_widgets_.size(); i < i_end; ++ i)
  {
    RenderWidget* current_render_widget = render_widgets_[i];
    bool focused = (current_render_widget == render_widget);
    current_render_widget->setActive(focused);
    active_render_widget_idx_ = i;
    DockWidget* dock_widget = dynamic_cast<DockWidget*>(current_render_widget->QVTKWidget::parent());
    if (dock_widget != NULL)
      dock_widget->setFocusBasedStyle(focused);
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkRenderer>
pcl::modeler::MainWindow::getActiveRender()
{
  return (getActiveRenderWidget()->getRenderer());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::addActionsToRenderWidget(QMenu* menu)
{
  menu->addAction(ui_->actionOpenPointCloud);
  menu->addAction(ui_->actionChangeBackgroundColor);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::addActionsToCloudActor(QMenu* menu)
{
  menu->addAction(ui_->actionSwitchColorHandler);
  menu->addAction(ui_->actionDownSampleFilter);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::connectFileMenuActions()
{
  connect(ui_->actionOpenPointCloud, SIGNAL(triggered()), this, SLOT(slotOpenPointCloud()));
  connect(ui_->actionImportPointCloud, SIGNAL(triggered()), this, SLOT(slotImportPointCloud()));
  connect(ui_->actionSavePointCloud, SIGNAL(triggered()), this, SLOT(slotSavePointCloud()));
  connect(ui_->actionClosePointCloud, SIGNAL(triggered()), this, SLOT(slotClosePointCloud()));
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
  connect(ui_->actionChangeBackgroundColor, SIGNAL(triggered()), this, SLOT(slotChangeBackgroundColor()));

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
  connect(ui_->actionDownSampleFilter, SIGNAL(triggered()), this, SLOT(slotDownSampleFilter()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOpenPointCloud()
{
  QStringList filenames = QFileDialog::getOpenFileNames(this,
    tr("Open Point Cloud"),
    getRecentFolder(),
    tr("Point Cloud(*.pcd)\n")
    );

  if (filenames.isEmpty())
    return;

  slotClosePointCloud();

  openPointCloudImpl(filenames);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotImportPointCloud()
{
  QStringList filenames = QFileDialog::getOpenFileNames(this,
    tr("Import Point Cloud"),
    getRecentFolder(),
    tr("Point Cloud(*.pcd)\n")
    );

  if (filenames.isEmpty())
    return;

  openPointCloudImpl(filenames);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotSavePointCloud()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotClosePointCloud()
{

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
  DockWidget* dock_widget = new DockWidget(tr("Render Window %0").arg(render_widgets_.size()), this);
  dock_widget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  RenderWidget* render_widget = new RenderWidget(this, render_widgets_.size(), dock_widget);

  dock_widget->setWidget(render_widget);
  addDockWidget(Qt::RightDockWidgetArea, dock_widget);

  // add the toggle action to view menu
  QList<QAction *> actions = ui_->menuView->actions();
  ui_->menuView->insertAction(actions[actions.size()-2], dock_widget->toggleViewAction());

  // keep a track of the qvtk widget
  addRenderWidget(render_widget);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::slotOpenRecentPointCloud()
{
  QAction* action = qobject_cast<QAction*>(sender());
  if (action)
    openPointCloudImpl(action->data().toString());

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
pcl::modeler::MainWindow::slotChangeBackgroundColor()
{
  double r, g, b;
  getActiveRender()->GetBackground(r, g, b);
  QColor color = QColorDialog::getColor(QColor(r, g, b), this);

  if (color.isValid()) {
    r = color.red();
    g = color.green();
    b = color.blue();
    getActiveRender()->SetBackground(r, g, b);
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::modeler::CloudActor*>
pcl::modeler::MainWindow::getSelectedCloud()
{
  std::vector<CloudActor*> cloud_actors;
  QModelIndexList selected_indexes = ui_->treeViewSceneExplorer->selectionModel()->selectedIndexes();
  for (QModelIndexList::const_iterator selected_indexes_it = selected_indexes.begin();
    selected_indexes_it != selected_indexes.end();
    ++ selected_indexes_it)
  {
    const QModelIndex& index = *selected_indexes_it;
    QStandardItem* item = pcl_modeler_->itemFromIndex(index);
    CloudActor* cloud_actor = dynamic_cast<CloudActor*>(item);
    if (cloud_actor == NULL)
      continue;

    cloud_actors.push_back(cloud_actor);
  }

  return (cloud_actors);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::slotSwitchColorHandler()
{
  std::vector<CloudActor*> cloud_actors = getSelectedCloud();

  ColorHandlerSwitcher color_handler_switcher(cloud_actors, this);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::slotDownSampleFilter()
{
  std::vector<CloudActor*> cloud_actors = getSelectedCloud();

  DownSampleWorker downsample_worker(this);
  for (size_t i = 0, i_end = cloud_actors.size(); i < i_end; ++ i)
    cloud_actors[i]->accept(&downsample_worker);

  if (downsample_worker.exec() == QDialog::Accepted)
  {
    for (size_t i = 0, i_end = cloud_actors.size(); i < i_end; ++ i)
      cloud_actors[i]->accept(&downsample_worker);
  }

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
  updateRecentActions(recent_pointcloud_actions_, recent_pointclouds_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::MainWindow::openPointCloudImpl(const QStringList& filenames)
{
  for (QStringList::const_iterator filenames_it = filenames.begin();
    filenames_it != filenames.end();
    ++ filenames_it)
  {
    if (!openPointCloudImpl(*filenames_it))
      QMessageBox::warning(this, 
      tr("Failed to Import Point Cloud"), 
      tr("Can not import point cloud file %1, please check if it's in valid .pcd format!").arg(*filenames_it));
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::modeler::MainWindow::openPointCloudImpl(const QString& filename)
{
  if(!pcl_modeler_->openPointCloud(filename.toStdString()))
  {
    return (false);
  }

  ui_->treeViewSceneExplorer->expand(pcl_modeler_->indexFromItem(getActiveRenderWidget()));

  recent_pointclouds_.removeAll(filename);
  recent_pointclouds_.prepend(filename);

  return (true);
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
pcl::modeler::MainWindow::openProjectImpl(const QString& filename)
{
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::updateRecentActions(std::vector<boost::shared_ptr<QAction> >& recent_actions, QStringList& recent_items)
{
  QMutableStringListIterator recent_items_it(recent_items);
  while (recent_items_it.hasNext())
  {
    if (!QFile::exists(recent_items_it.next()))
      recent_items_it.remove();
  }

  recent_items.removeDuplicates();
  int recent_number = std::min((int)MAX_RECENT_NUMBER, recent_items.size());
  for (int i = 0; i < recent_number; ++ i)
  {
    QString text = tr("%1 %2").arg(i+1).arg(recent_items[i]);
    recent_actions[i]->setText(text);
    recent_actions[i]->setData(recent_items[i]);
    recent_actions[i]->setVisible(true);
  }

  for (size_t i = recent_number, i_end = recent_actions.size(); i < i_end; ++ i)
  {
    recent_actions[i]->setVisible(false);
  }

  while (recent_items.size() > recent_number) {
    recent_items.pop_back();
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
QString 
pcl::modeler::MainWindow::getRecentFolder()
{
  QString recent_filename;
  if (!recent_projects_.empty())
    recent_filename = recent_projects_.front();
  else if (!recent_pointclouds_.empty())
    recent_filename = recent_pointclouds_.front();

  if (!recent_filename.isEmpty())
    return QFileInfo(recent_filename).path();

  return (QString("."));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::MainWindow::loadGlobalSettings()
{
  QSettings global_settings("PCL", "Modeler");

  recent_pointclouds_ = global_settings.value("recent_pointclouds").toStringList();
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

  global_settings.setValue("recent_pointclouds", recent_pointclouds_);

  global_settings.setValue("recent_projects", recent_projects_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWidget*
pcl::modeler::MainWindow::getActiveRenderWidget()
{
  return (render_widgets_[active_render_widget_idx_]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::addRenderWidget(pcl::modeler::RenderWidget* render_widget)
{
  pcl_modeler_->appendRow(render_widget);
  render_widgets_.push_back(render_widget);

  setActiveDockWidget(render_widget);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::MainWindow::triggerRender(vtkActor* actor)
{
  for (size_t i = 0, i_end = render_widgets_.size(); i < i_end; ++ i)
  {
    vtkSmartPointer<vtkRenderer> renderer = render_widgets_[i]->getRenderer();
    vtkSmartPointer<vtkActorCollection> actors = renderer->GetActors();
    actors->InitTraversal();
    for(vtkIdType i = 0; i < actors->GetNumberOfItems(); i++)
    {
      vtkActor* nextActor = actors->GetNextActor();
      if (nextActor == actor)
      {
        renderer->GetRenderWindow()->Render();
        break;
      }
    }
  }

  return;
}

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

#include <pcl/apps/modeler/scene_tree.h>

#include <set>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/render_window_item.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/apps/modeler/cloud_mesh_item_updater.h>
#include <pcl/apps/modeler/thread_controller.h>
#include <pcl/apps/modeler/voxel_grid_downsample_worker.h>
#include <pcl/apps/modeler/statistical_outlier_removal_worker.h>
#include <pcl/apps/modeler/normal_estimation_worker.h>
#include <pcl/apps/modeler/icp_registration_worker.h>
#include <pcl/apps/modeler/poisson_worker.h>
#include <pcl/io/pcd_io.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::SceneTree::SceneTree(QWidget * parent)
  : QTreeWidget(parent)
{
  setDragEnabled(true);
  setAcceptDrops(true);
  setDropIndicatorShown(true);
  setDragDropMode(QAbstractItemView::DragDrop);

  setSelectionMode(QAbstractItemView::ExtendedSelection);

  connect(selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
    this, SLOT(slotUpdateOnSelectionChange(QItemSelection, const QItemSelection)));

  connect(this, SIGNAL(itemInsertedOrRemoved()),this, SLOT(slotUpdateOnInsertOrRemove()));

  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),this, SLOT(slotOnItemDoubleClicked(QTreeWidgetItem*)));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::SceneTree::~SceneTree()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
QSize
pcl::modeler::SceneTree::sizeHint() const
{
  return QSize(256, 512);
}

//////////////////////////////////////////////////////////////////////////////////////////////
QList<pcl::modeler::RenderWindowItem*>
pcl::modeler::SceneTree::selectedRenderWindowItems() const
{
  QList<RenderWindowItem*> selected_render_window_items;
  if (topLevelItemCount() == 1)
    selected_render_window_items.push_back(dynamic_cast<RenderWindowItem*>(topLevelItem(0)));
  else
    selected_render_window_items = selectedTypeItems<RenderWindowItem>();

  return (selected_render_window_items);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::contextMenuEvent(QContextMenuEvent *event)
{
  AbstractItem* item = dynamic_cast<AbstractItem*>(currentItem());
  item->showContextMenu(&(event->globalPos()));

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotOnItemDoubleClicked(QTreeWidgetItem * item)
{
  AbstractItem* abstract_item = dynamic_cast<AbstractItem*>(item);
  abstract_item->showPropertyEditor();
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::modeler::SceneTree::openPointCloud(const QString& filename)
{
  QList<RenderWindowItem*> selected_render_window_items = selectedRenderWindowItems();

  for (QList<RenderWindowItem*>::iterator selected_render_window_items_it = selected_render_window_items.begin();
    selected_render_window_items_it != selected_render_window_items.end();
    ++ selected_render_window_items_it)
  {
    if(!(*selected_render_window_items_it)->openPointCloud(filename))
      return (false);
    expandItem(*selected_render_window_items_it);
  }

  emit fileOpened(filename);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::modeler::SceneTree::savePointCloud(const QString& filename)
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();

  return (CloudMeshItem::savePointCloud(selected_cloud_mesh_items, filename));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::SceneTree::slotOpenPointCloud()
{
  MainWindow* main_window = &MainWindow::getInstance();
  QList<RenderWindowItem*> selected_render_window_items = selectedRenderWindowItems();

  if (selected_render_window_items.empty())
  {
    QMessageBox::warning(main_window, 
    tr("Failed to Open Point Cloud"), 
    tr("Please specify in which render window(s) you want to open the point cloud(s)"));
    return;
  }

  QStringList filenames = QFileDialog::getOpenFileNames(main_window,
    tr("Open Point Cloud"),
    main_window->getRecentFolder(),
    tr("Point Cloud(*.pcd)\n")
    );

  if (filenames.isEmpty())
    return;

  for (QList<RenderWindowItem*>::iterator selected_render_window_items_it = selected_render_window_items.begin();
    selected_render_window_items_it != selected_render_window_items.end();
    ++ selected_render_window_items_it)
  {
    RenderWindowItem* render_window_item = *selected_render_window_items_it;

    QList<CloudMeshItem*> cloud_mesh_items;
    for (int i = 0, i_end = render_window_item->childCount(); i < i_end; ++ i)
      cloud_mesh_items.push_back(dynamic_cast<CloudMeshItem*>(render_window_item->child(i)));

    closePointCloud(cloud_mesh_items);
  }

  for (QStringList::const_iterator filenames_it = filenames.begin();
    filenames_it != filenames.end();
    ++ filenames_it)
  {
    if (!openPointCloud(*filenames_it))
      QMessageBox::warning(main_window, 
      tr("Failed to Open Point Cloud"), 
      tr("Can not open point cloud file %1, please check if it's in valid .pcd format!").arg(*filenames_it));
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::SceneTree::slotImportPointCloud()
{
  MainWindow* main_window = &MainWindow::getInstance();

  if (selectedRenderWindowItems().empty())
  {
    QMessageBox::warning(main_window, 
      tr("Failed to Import Point Cloud"), 
      tr("Please specify in which render window(s) you want to import the point cloud(s)"));
    return;
  }

  QStringList filenames = QFileDialog::getOpenFileNames(main_window,
    tr("Import Point Cloud"),
    main_window->getRecentFolder(),
    tr("Point Cloud(*.pcd)\n")
    );
  if (filenames.isEmpty())
    return;

  for (QStringList::const_iterator filenames_it = filenames.begin();
    filenames_it != filenames.end();
    ++ filenames_it)
  {
    if (!openPointCloud(*filenames_it))
      QMessageBox::warning(main_window, 
      tr("Failed to Import Point Cloud"), 
      tr("Can not import point cloud file %1, please check if it's in valid .pcd format!").arg(*filenames_it));
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::SceneTree::slotSavePointCloud()
{
  MainWindow* main_window = &MainWindow::getInstance();

  if (selectedTypeItems<CloudMeshItem>().empty())
  {
    QMessageBox::warning(main_window, 
      tr("Failed to Save Point Cloud"), 
      tr("Please specify the point cloud(s) you want to save"));
    return;
  }

  QString filename = QFileDialog::getSaveFileName(main_window,
    tr("Save Point Cloud"),
    main_window->getRecentFolder(),
    tr("Save Cloud(*.pcd)\n"));

  if (filename.isEmpty())
    return;

  savePointCloud(filename);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::SceneTree::closePointCloud(const QList<CloudMeshItem*>& items)
{
  QList<RenderWindowItem*> render_window_items;

  for (QList<CloudMeshItem*>::const_iterator items_it = items.begin();
    items_it != items.end();
    ++ items_it)
  {
    CloudMeshItem* item = *items_it;

    RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(item->parent());
    if (render_window_item != NULL)
      render_window_items.push_back(render_window_item);

    item->parent()->removeChild(item);
    delete item;
  }

  for (QList<RenderWindowItem*>::const_iterator render_window_items_it = render_window_items.begin();
    render_window_items_it != render_window_items.end();
    ++ render_window_items_it)
  {
    (*render_window_items_it)->getRenderWindow()->render();
  }
  
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::SceneTree::slotClosePointCloud()
{
  MainWindow* main_window = &MainWindow::getInstance();
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();

  if (selected_cloud_mesh_items.empty())
  {
    QMessageBox::warning(main_window, 
      tr("Failed to Close Point Cloud"), 
      tr("Please specify the point cloud(s) you want to close"));
    return;
  }

  closePointCloud(selected_cloud_mesh_items);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotICPRegistration()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  CloudMesh::PointCloudPtr result(new CloudMesh::PointCloud());

  AbstractWorker* worker = new ICPRegistrationWorker(result, selected_cloud_mesh_items,&MainWindow::getInstance());
  ThreadController* thread_controller = new ThreadController();

  QList<RenderWindowItem*> selected_render_window_items = selectedRenderWindowItems();
  for (QList<RenderWindowItem*>::iterator selected_render_window_items_it = selected_render_window_items.begin();
    selected_render_window_items_it != selected_render_window_items.end();
    ++ selected_render_window_items_it)
  {
    CloudMeshItem* cloud_mesh_item = (*selected_render_window_items_it)->addPointCloud(result);
    expandItem(*selected_render_window_items_it);
    connect(worker, SIGNAL(finished()), new CloudMeshItemUpdater(cloud_mesh_item), SLOT(updateCloudMeshItem()));
  }
  
  thread_controller->runWorker(worker);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotVoxelGridDownsampleFilter()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* worker = new VoxelGridDownampleWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  ThreadController* thread_controller = new ThreadController();
  connect(worker, SIGNAL(dataUpdated(CloudMeshItem*)), thread_controller, SLOT(slotOnCloudMeshItemUpdate(CloudMeshItem*)));
  thread_controller->runWorker(worker);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotStatisticalOutlierRemovalFilter()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* worker = new StatisticalOutlierRemovalWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  ThreadController* thread_controller = new ThreadController();
  connect(worker, SIGNAL(dataUpdated(CloudMeshItem*)), thread_controller, SLOT(slotOnCloudMeshItemUpdate(CloudMeshItem*)));
  thread_controller->runWorker(worker);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotEstimateNormal()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* worker = new NormalEstimationWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  ThreadController* thread_controller = new ThreadController();
  connect(worker, SIGNAL(dataUpdated(CloudMeshItem*)), thread_controller, SLOT(slotOnCloudMeshItemUpdate(CloudMeshItem*)));
  thread_controller->runWorker(worker);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotPoissonReconstruction()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* worker = new PoissonReconstructionWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  ThreadController* thread_controller = new ThreadController();
  connect(worker, SIGNAL(dataUpdated(CloudMeshItem*)), thread_controller, SLOT(slotOnCloudMeshItemUpdate(CloudMeshItem*)));
  thread_controller->runWorker(worker);

  return;
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::selectRenderWindowItem(RenderWindowItem* render_window_item)
{
  selectionModel()->clearSelection();
  selectionModel()->select(indexFromItem(render_window_item), QItemSelectionModel::SelectCurrent);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotUpdateOnSelectionChange(const QItemSelection & selected, const QItemSelection & deselected)
{
  QModelIndexList selected_indices = selected.indexes();
  for (QModelIndexList::const_iterator selected_indices_it = selected_indices.begin();
    selected_indices_it != selected_indices.end();
    ++ selected_indices_it)
  {
    QTreeWidgetItem* item = itemFromIndex(*selected_indices_it);
    RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(item);
    if (render_window_item != NULL)
    {
      render_window_item->getRenderWindow()->setActive(true);
    }
  }

  QModelIndexList deselected_indices = deselected.indexes();
  for (QModelIndexList::const_iterator deselected_indices_it = deselected_indices.begin();
    deselected_indices_it != deselected_indices.end();
    ++ deselected_indices_it)
  {
    QTreeWidgetItem* item = itemFromIndex(*deselected_indices_it);
    RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(item);
    if (render_window_item != NULL)
    {
      render_window_item->getRenderWindow()->setActive(false);
    }
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotUpdateOnInsertOrRemove()
{
  for (int i = 0, i_end = topLevelItemCount(); i < i_end; ++ i)
  {
    RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(topLevelItem(i));
    if (render_window_item == NULL)
      continue;

    QString title = (i == 0)?("Central Render Window"):(QString("Render Window #%1").arg(i));
    render_window_item->setText(0, title);

    render_window_item->getRenderWindow()->setTitle(title);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::addTopLevelItem(RenderWindowItem* render_window_item)
{
  QTreeWidget::addTopLevelItem(render_window_item);

  selectionModel()->clearSelection();
  selectionModel()->select(indexFromItem(render_window_item), QItemSelectionModel::SelectCurrent);

  emit itemInsertedOrRemoved();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotCloseRenderWindow()
{
  QList<RenderWindowItem*> selected_render_window_items = selectedTypeItems<RenderWindowItem>();

  for (QList<RenderWindowItem*>::iterator selected_render_window_items_it = selected_render_window_items.begin();
    selected_render_window_items_it != selected_render_window_items.end();
    ++ selected_render_window_items_it)
  {
    removeItemWidget((*selected_render_window_items_it), 0);
    delete (*selected_render_window_items_it);
  }

  emit itemInsertedOrRemoved();

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::dropEvent(QDropEvent * event)
{
  QList<CloudMeshItem*> selected_cloud_meshes = selectedTypeItems<CloudMeshItem>();

  std::set<RenderWindowItem*> previous_parents;
  for (QList<CloudMeshItem*>::iterator selected_cloud_meshes_it = selected_cloud_meshes.begin();
    selected_cloud_meshes_it != selected_cloud_meshes.end();
    selected_cloud_meshes_it ++)
  {
    CloudMeshItem* cloud_mesh_item = *selected_cloud_meshes_it;
    RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(cloud_mesh_item->parent());
    if (render_window_item != NULL)
      previous_parents.insert(render_window_item);
  }

  QTreeWidget::dropEvent(event);

  std::vector<CloudMeshItem*> cloud_mesh_items;
  for (QList<CloudMeshItem*>::iterator selected_cloud_meshes_it = selected_cloud_meshes.begin();
    selected_cloud_meshes_it != selected_cloud_meshes.end();
    selected_cloud_meshes_it ++)
  {
    CloudMeshItem* cloud_mesh_item = *selected_cloud_meshes_it;
    if (dynamic_cast<RenderWindowItem*>(cloud_mesh_item->parent()) == NULL)
      cloud_mesh_items.push_back(cloud_mesh_item);
    else
      cloud_mesh_item->updateRenderWindow();
  }

  // put the cloud mesh items in a new render window
  if (!cloud_mesh_items.empty())
  {
    for (size_t i = 0, i_end = cloud_mesh_items.size(); i < i_end; ++ i)
      takeTopLevelItem(indexFromItem(cloud_mesh_items[i]).row());
    RenderWindowItem* render_window_item = MainWindow::getInstance().createRenderWindow();
    for (size_t i = 0, i_end = cloud_mesh_items.size(); i < i_end; ++ i)
      render_window_item->addChild(cloud_mesh_items[i]);
    render_window_item->setExpanded(true);
  }

  for (std::set<RenderWindowItem*>::iterator previous_parents_it = previous_parents.begin();
    previous_parents_it != previous_parents.end();
    previous_parents_it ++)
  {
    (*previous_parents_it)->getRenderWindow()->updateAxes();
    (*previous_parents_it)->getRenderWindow()->render();
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::SceneTree::dropMimeData(QTreeWidgetItem * parent, int, const QMimeData *, Qt::DropAction)
{
  QList<CloudMeshItem*> selected_cloud_meshes = selectedTypeItems<CloudMeshItem>();

  RenderWindowItem* render_window_item =
    (parent == NULL)?(MainWindow::getInstance().createRenderWindow()):(dynamic_cast<RenderWindowItem*>(parent));

  for (QList<CloudMeshItem*>::iterator selected_cloud_meshes_it = selected_cloud_meshes.begin();
    selected_cloud_meshes_it != selected_cloud_meshes.end();
    selected_cloud_meshes_it ++)
  {
    CloudMeshItem* cloud_mesh_item_copy = new CloudMeshItem(render_window_item, *(*selected_cloud_meshes_it));
    render_window_item->addChild(cloud_mesh_item_copy);
    setCurrentItem(cloud_mesh_item_copy);
  }
  render_window_item->setExpanded(true);

  return true;
}

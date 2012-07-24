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

#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/render_window_item.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/apps/modeler/downsample_worker.h>
#include <pcl/apps/modeler/normal_estimation_worker.h>
#include <pcl/apps/modeler/poisson_worker.h>
#include <pcl/io/pcd_io.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::SceneTree::SceneTree(QWidget * parent)
  : QTreeWidget(parent)
{
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
  for (QList<CloudMeshItem*>::const_iterator items_it = items.begin();
    items_it != items.end();
    ++ items_it)
  {
    CloudMeshItem* item = *items_it;
    item->parent()->removeChild(item);
    delete item;
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
pcl::modeler::SceneTree::slotDownSampleFilter()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* down_sample_filter = new DownSampleWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  down_sample_filter->run();

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotEstimateNormal()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* normal_estimation_worker = new NormalEstimationWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  normal_estimation_worker->run();

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SceneTree::slotPoissonReconstruction()
{
  QList<CloudMeshItem*> selected_cloud_mesh_items = selectedTypeItems<CloudMeshItem>();
  AbstractWorker* poisson_reconstruction_worker = new PoissonReconstructionWorker(selected_cloud_mesh_items,&MainWindow::getInstance());
  poisson_reconstruction_worker->run();

  return;
}

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
#include <pcl/apps/modeler/tree_view.h>
#include <pcl/apps/modeler/tree_model.h>
#include <pcl/apps/modeler/tree_item.h>
#include <pcl/apps/modeler/polymesh_item.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/downsample_worker.h>
#include <pcl/apps/modeler/normal_estimation_worker.h>
#include <pcl/apps/modeler/poisson_worker.h>
#include <pcl/io/pcd_io.h>
#include <QThread>
#include <QMessageBox>
#include <QFileDialog>
#include <QItemSelectionModel>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeView::TreeView(QWidget * parent) : 
  main_window_(NULL),
  QTreeView(parent)
{
  connect(this, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(slotOnDoubleClick(QModelIndex)));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeView::~TreeView()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::setMainWindow(MainWindow* main_window)
{
  main_window_ = main_window;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeModel*
pcl::modeler::TreeView::model()
{
  return (dynamic_cast<TreeModel*>(QTreeView::model()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
QSize
pcl::modeler::TreeView::sizeHint() const
{
  return QSize(256, 512);
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::modeler::TreeItem*>
pcl::modeler::TreeView::selectedItems()
{
  std::vector<TreeItem*> selected_items;
  QModelIndexList selected_indices = selectionModel()->selectedIndexes();
  TreeModel* tree_model = dynamic_cast<TreeModel*>(model());
  for (QModelIndexList::const_iterator selected_indices_it = selected_indices.begin();
    selected_indices_it != selected_indices.end();
    ++ selected_indices_it)
  {
    TreeItem* tree_item = dynamic_cast<TreeItem*>(tree_model->itemFromIndex(*selected_indices_it));
    selected_items.push_back(tree_item);
  }

  if (selected_items.empty())
    selected_items.push_back(dynamic_cast<TreeItem*>(tree_model->itemFromIndex(tree_model->index(0, 0))));

  return selected_items;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::contextMenuEvent(QContextMenuEvent *event)
{
  QStandardItem* item = dynamic_cast<QStandardItemModel*>(model())->itemFromIndex(currentIndex());
  TreeItem* tree_item = dynamic_cast<TreeItem*>(item);
  tree_item->showContextMenu(&(event->globalPos()));

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::modeler::TreeView::openPointCloud(const QString& filename)
{
  std::vector<RenderWidget*> polymesh_items = selectedItems<RenderWidget>();
  if (polymesh_items.empty())
    return (false);
  
  PolymeshItem* polymesh = new PolymeshItem(main_window_, filename.toStdString());
  polymesh_items[0]->appendRow(polymesh);
  polymesh->setCheckState(Qt::Checked);

  main_window_->getRecentFiles().removeAll(filename);
  main_window_->getRecentFiles().prepend(filename);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::TreeView::slotOpenPointCloud()
{
  QStringList filenames = QFileDialog::getOpenFileNames(main_window_,
    tr("Open Point Cloud"),
    main_window_->getRecentFolder(),
    tr("Point Cloud(*.pcd)\n")
    );

  if (filenames.isEmpty())
    return;

  slotClosePointCloud();

  for (QStringList::const_iterator filenames_it = filenames.begin();
    filenames_it != filenames.end();
    ++ filenames_it)
  {
    if (!openPointCloud(*filenames_it))
      QMessageBox::warning(this, 
      tr("Failed to Import Point Cloud"), 
      tr("Can not import point cloud file %1, please check if it's in valid .pcd format!").arg(*filenames_it));
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::TreeView::slotImportPointCloud()
{
  QStringList filenames = QFileDialog::getOpenFileNames(main_window_,
    tr("Import Point Cloud"),
    main_window_->getRecentFolder(),
    tr("Point Cloud(*.pcd)\n")
    );

  if (filenames.isEmpty())
    return;

  for (QStringList::const_iterator filenames_it = filenames.begin();
    filenames_it != filenames.end();
    ++ filenames_it)
  {
    if (!openPointCloud(*filenames_it))
      QMessageBox::warning(this, 
      tr("Failed to Import Point Cloud"), 
      tr("Can not import point cloud file %1, please check if it's in valid .pcd format!").arg(*filenames_it));
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::TreeView::slotSavePointCloud()
{
  QString filename = QFileDialog::getSaveFileName(main_window_,
    tr("Save Point Cloud"),
    main_window_->getRecentFolder(),
    tr("Save Cloud(*.pcd)\n"));

  if (filename.isEmpty())
    return;

  std::vector<PolymeshItem*> polymesh_items = selectedItems<PolymeshItem>();
  PolymeshItem::save(polymesh_items, filename.toStdString());

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::TreeView::slotClosePointCloud()
{
  std::vector<PolymeshItem*> polymesh_items = selectedItems<PolymeshItem>();
  for (size_t i = 0, i_end = polymesh_items.size(); i < i_end; ++ i)
    model()->removeRow(polymesh_items[i]->row(), model()->indexFromItem(polymesh_items[i]->parent()));

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::updateOnSelectionChange(const QItemSelection & selection, bool selected)
{
  QModelIndexList selected_indices = selection.indexes();
  for (QModelIndexList::iterator selected_indices_it = selected_indices.begin();
    selected_indices_it != selected_indices.end();
    ++ selected_indices_it)
  {
    TreeModel* tree_model = dynamic_cast<TreeModel*>(model());
    TreeItem* tree_item = dynamic_cast<TreeItem*>(tree_model->itemFromIndex(*selected_indices_it));
    tree_item->updateOnSelectionChange(selected);
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::TreeView::slotOnSelectionChange(const QItemSelection & selected, const QItemSelection & deselected)
{
  updateOnSelectionChange(selected, true);
  updateOnSelectionChange(deselected, false);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::modeler::TreeView::slotOnDoubleClick(const QModelIndex & index)
{
  QStandardItem* item = dynamic_cast<QStandardItemModel*>(model())->itemFromIndex(index);
  if (!item->isCheckable())
    return;

  item->setCheckState((item->checkState()==Qt::Checked)?Qt::Unchecked:Qt::Checked);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::slotChangeBackgroundColor()
{
  std::vector<RenderWidget*> render_widgets = selectedItems<RenderWidget>();
  for (size_t i = 0, i_end = render_widgets.size(); i < i_end; ++ i)
    render_widgets[i]->changeBackgroundColor();

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::executeWorker(AbstractWorker* worker)
{
  if (worker->exec() == QDialog::Accepted)
  {
    QThread* thread = new QThread;
    worker->moveToThread(thread);
    connect(thread, SIGNAL(started()), worker, SLOT(process()));
    connect(worker, SIGNAL(processed()), worker, SLOT(postProcess()));
    connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    thread->start();
  }
  else
  {
    delete worker;
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::slotDownSampleFilter()
{
  std::vector<PolymeshItem*> polymesh_items = selectedItems<PolymeshItem>();

  AbstractWorker* worker = new DownSampleWorker(polymesh_items, main_window_);

  executeWorker(worker);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::slotEstimateNormal()
{
  std::vector<PolymeshItem*> polymesh_items = selectedItems<PolymeshItem>();

  AbstractWorker* worker = new NormalEstimationWorker(polymesh_items, main_window_);

  executeWorker(worker);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeView::slotPoissonReconstruction()
{
  std::vector<PolymeshItem*> polymesh_items = selectedItems<PolymeshItem>();

  AbstractWorker* worker = new PoissonReconstructionWorker(polymesh_items, main_window_);

  executeWorker(worker);

  return;
}

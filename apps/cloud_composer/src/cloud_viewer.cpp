#include <QtGui>

#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_composer.h>

pcl::cloud_composer::CloudViewer::CloudViewer (QVTKWidget* qvtk_widget, QWidget* parent)
  : QAbstractItemView (parent)
  , qvtk_ptr_ (qvtk_widget)

{
  horizontalScrollBar ()->setRange (0, 0);
  verticalScrollBar ()->setRange (0, 0);

  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  qvtk_ptr_->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (qvtk_ptr_->GetInteractor (), qvtk_ptr_->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  qvtk_ptr_->update ();

}


void
pcl::cloud_composer::CloudViewer::dataChanged (const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
  QAbstractItemView::dataChanged (topLeft, bottomRight);

  for (int row = 0; row < model ()->rowCount (rootIndex ()); ++row)
  {

      QModelIndex index = model ()->index (row, 0, rootIndex ());

  }
  viewport ()->update ();
  qvtk_ptr_->update ();
}


void
pcl::cloud_composer::CloudViewer::rowsInserted (const QModelIndex& parent, int start, int end)
{
  for (int row = start; row <= end; ++row)
  {

    QModelIndex index = model ()->index (row, 0, rootIndex ());
    QStandardItem* item =  static_cast<QStandardItemModel*> (model ())->itemFromIndex (index);
    
    
    QString cloud_name = item->text ();
    QVariant cloud_ptr = item->data (CLOUD);
    sensor_msgs::PointCloud2::Ptr cloud_blob = cloud_ptr.value<sensor_msgs::PointCloud2::Ptr> ();
    QVariant color_ptr = item->data (COLOR);
    ColorHandler::ConstPtr color_handler = color_ptr.value<ColorHandler::ConstPtr> ();
    QVariant geometry_ptr = item->data (GEOMETRY);
    GeometryHandler::ConstPtr geometry_handler = geometry_ptr.value<GeometryHandler::ConstPtr> ();
    
    Eigen::Vector4f origin = item->data (ORIGIN).value<Eigen::Vector4f> (); 
    Eigen::Quaternionf orientation = item->data (ORIENTATION).value<Eigen::Quaternionf> (); 
 
    vis_->addPointCloud (cloud_blob, geometry_handler, color_handler, origin, orientation, cloud_name.toStdString ());
    
    
  }

  qvtk_ptr_->update ();
  QAbstractItemView::rowsInserted (parent, start, end);
}

void
pcl::cloud_composer::CloudViewer::rowsAboutToBeRemoved (const QModelIndex& parent, int start, int end)
{
  for (int row = start; row <= end; ++row)
  {

      QModelIndex index = model ()->index (row, 1, rootIndex ());

  }

  QAbstractItemView::rowsAboutToBeRemoved (parent, start, end);
}

void 
pcl::cloud_composer::CloudViewer::paintEvent (QPaintEvent* event)
{
  qvtk_ptr_->update ();
}

void 
pcl::cloud_composer::CloudViewer::resizeEvent (QResizeEvent* event)
{
  qvtk_ptr_->update ();
}



//These are required, but currently aren't used for anything
QModelIndex 
pcl::cloud_composer::CloudViewer::indexAt (const QPoint &point) const
{
  return QModelIndex ();
}

bool
pcl::cloud_composer::CloudViewer::isIndexHidden (const QModelIndex & /*index*/) const
{
  return false;
}

int 
pcl::cloud_composer::CloudViewer::horizontalOffset () const
{
  return horizontalScrollBar ()->value ();
}

QModelIndex 
pcl::cloud_composer::CloudViewer::moveCursor (QAbstractItemView::CursorAction cursorAction,
                                 Qt::KeyboardModifiers /*modifiers*/)
{
  QModelIndex current = currentIndex ();
  return current;
}

void 
pcl::cloud_composer::CloudViewer::setSelection (const QRect &rect, QItemSelectionModel::SelectionFlags command)
{
     
}

int 
pcl::cloud_composer::CloudViewer::verticalOffset () const
{
  return verticalScrollBar ()->value ();
}

QRect 
pcl::cloud_composer::CloudViewer::visualRect (const QModelIndex &index) const
{
  return QRect ();
}

QRegion 
pcl::cloud_composer::CloudViewer::visualRegionForSelection (const QItemSelection &selection) const
{
  return QRegion ();
}

void 
pcl::cloud_composer::CloudViewer::scrollTo (const QModelIndex &index, ScrollHint)
{
     
}

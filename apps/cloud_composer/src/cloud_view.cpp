#include <QtGui>

#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>

pcl::cloud_composer::CloudView::CloudView (QWidget* parent)
  : QWidget (parent)
{
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  
}

pcl::cloud_composer::CloudView::CloudView (ProjectModel* model, QWidget* parent)
  : QWidget (parent)
{
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  setModel(model);
  
}

pcl::cloud_composer::CloudView::CloudView (const CloudView& to_copy)
  :vis_(to_copy.vis_)
  ,model_(to_copy.model_)
  ,qvtk_(to_copy.qvtk_)
{
}


pcl::cloud_composer::CloudView::~CloudView ()
{
}

void
pcl::cloud_composer::CloudView::setModel (ProjectModel* new_model)
{
  //Create the QVTKWidget
  qvtk_ = new QVTKWidget ();
  qvtk_->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (qvtk_->GetInteractor (), qvtk_->GetRenderWindow ());
 
  QGridLayout *mainLayout = new QGridLayout (this);
  mainLayout-> addWidget (qvtk_,0,0);
  
  model_ = new_model;
  //Make Connections!
  connectSignalsAndSlots();
  //Refresh the view
  qvtk_->show();
  qvtk_->update ();
}

void
pcl::cloud_composer::CloudView::connectSignalsAndSlots()
{
  connect (model_, SIGNAL (dataChanged (const QModelIndex&, const QModelIndex&)),
           this, SLOT (dataChanged (const QModelIndex&, const QModelIndex&)));
  connect (model_, SIGNAL (rowsInserted (const QModelIndex&, int, int)),
           this, SLOT (rowsInserted (const QModelIndex&, int, int)));
}

void
pcl::cloud_composer::CloudView::refresh ()
{
  qvtk_->update (); 
}

void
pcl::cloud_composer::CloudView::dataChanged (const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
  
  
  for (int row = 0; row < model_->rowCount (); ++row)
  {
    
    QModelIndex index = model_->index (row, 0);
    
  }
  
  qvtk_->update ();
}


void
pcl::cloud_composer::CloudView::rowsInserted (const QModelIndex& parent, int start, int end)
{
  for (int row = start; row <= end; ++row)
  {
    
    QModelIndex index = model_->index (row, 0);
    QStandardItem* item =  static_cast<QStandardItemModel*> (model_)->itemFromIndex (index);
    
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
  
  qvtk_->update ();

}

void
pcl::cloud_composer::CloudView::rowsAboutToBeRemoved (const QModelIndex& parent, int start, int end)
{
  for (int row = start; row <= end; ++row)
  {
    
    QModelIndex index = model_->index (row, 1);
    
  }
  
}


void 
pcl::cloud_composer::CloudView::paintEvent (QPaintEvent* event)
{
  qvtk_->update ();
}

void 
pcl::cloud_composer::CloudView::resizeEvent (QResizeEvent* event)
{
  qvtk_->update ();
}

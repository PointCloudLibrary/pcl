#include <pcl/apps/cloud_composer/items/fpfh_item.h>
#include <pcl/apps/cloud_composer/qt.h>


pcl::cloud_composer::FPFHItem::FPFHItem (QString name, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_ptr, double radius)
  : CloudComposerItem (name)
  , fpfh_ptr_ (fpfh_ptr)
  , radius_ (radius)

{
  
  this->setData (QVariant::fromValue (fpfh_ptr_), FPFH_CLOUD);
  
  properties_->addProperty ("Radius", QVariant (radius_), Qt::ItemIsEnabled);
  
  
}

pcl::cloud_composer::FPFHItem*

pcl::cloud_composer::FPFHItem::clone () const
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_copy (new pcl::PointCloud<pcl::FPFHSignature33> (*fpfh_ptr_));
  FPFHItem* new_item = new FPFHItem (this->text (), fpfh_copy, radius_);
  
  PropertiesModel* new_item_properties = new_item->getProperties ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

pcl::cloud_composer::FPFHItem::~FPFHItem ()
{
  
}

QMap <QString, QWidget*>
pcl::cloud_composer::FPFHItem::getInspectorTabs ()
{
 
  
  //Create the plotter and QVTKWidget if it doesnt exist
  if (!plot_)
  {
    plot_ = boost::shared_ptr<pcl::visualization::PCLPlotter> (new pcl::visualization::PCLPlotter);
    qvtk_ = new QVTKWidget ();
    hist_page_ = new QWidget ();
    QGridLayout *mainLayout = new QGridLayout (hist_page_);
    mainLayout-> addWidget (qvtk_,0,0);
  }

  //Plot the histogram
  plot_->addFeatureHistogram (*fpfh_ptr_, fpfh_ptr_->width, item_id_.toStdString ());
  //Set the render window of the QVTK widget, update
  plot_->setViewInteractor (vtkSmartPointer<vtkRenderWindowInteractor> (qvtk_->GetInteractor ()));
  qvtk_->SetRenderWindow (plot_->getRenderWindow ());
  qvtk_->show ();
  qvtk_->update ();
  
  QMap <QString, QWidget*> tabs;
  tabs.insert ("Histogram",hist_page_);
  return tabs;
}



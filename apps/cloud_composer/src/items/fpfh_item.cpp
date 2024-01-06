#include <pcl/apps/cloud_composer/items/fpfh_item.h>
#include <vtkRenderWindow.h>

#include <QGridLayout>

pcl::cloud_composer::FPFHItem::FPFHItem (QString name, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_ptr, double radius)
  : CloudComposerItem (std::move(name))
  , fpfh_ptr_ (fpfh_ptr)
  , radius_ (radius)

{
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfh_const = fpfh_ptr;
  this->setData (QVariant::fromValue (fpfh_const), ItemDataRole::CLOUD_TEMPLATED);
  properties_->addCategory ("Core Properties");
  properties_->addProperty ("Radius", QVariant (radius_), Qt::ItemIsEnabled, "Core Properties");  
}

pcl::cloud_composer::FPFHItem*
pcl::cloud_composer::FPFHItem::clone () const
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_copy (new pcl::PointCloud<pcl::FPFHSignature33> (*fpfh_ptr_));
  FPFHItem* new_item = new FPFHItem (this->text (), fpfh_copy, radius_);
  
  PropertiesModel* new_item_properties = new_item->getPropertiesModel ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

QMap <QString, QWidget*>
pcl::cloud_composer::FPFHItem::getInspectorTabs ()
{  
  //Create the plotter and QVTKWidget if it doesn't exist
  if (!plot_)
  {
    plot_.reset (new pcl::visualization::PCLPlotter);
    qvtk_ = new PCLQVTKWidget();
    hist_page_ = new QWidget ();
    QGridLayout *mainLayout = new QGridLayout (hist_page_);
    mainLayout-> addWidget (qvtk_,0,0);
  }

  //Plot the histogram
  plot_->addFeatureHistogram (*fpfh_ptr_, fpfh_ptr_->width, data(ItemDataRole::ITEM_ID).toString().toStdString ());
  //Set the render window of the QVTK widget, update
  plot_->setViewInteractor(getInteractorCompat(*qvtk_));
  setRenderWindowCompat(*qvtk_, *(plot_->getRenderWindow()));
#if VTK_MAJOR_VERSION > 8
  qvtk_->renderWindow()->Render();
#else
  qvtk_->update();
#endif // VTK_MAJOR_VERSION > 8
  qvtk_->show ();
  
  
  QMap <QString, QWidget*> tabs;
  tabs.insert ("Histogram",hist_page_);
  return tabs;
}



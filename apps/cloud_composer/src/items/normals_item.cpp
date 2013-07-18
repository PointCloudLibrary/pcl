#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>


pcl::cloud_composer::NormalsItem::NormalsItem (QString name, pcl::PointCloud<pcl::Normal>::Ptr normals_ptr, double radius)
  : CloudComposerItem (name)
  , normals_ptr_ (normals_ptr)

{
  pcl::PointCloud<pcl::Normal>::ConstPtr normals_const = normals_ptr;
  this->setData (QVariant::fromValue (normals_const), ItemDataRole::CLOUD_TEMPLATED);
  properties_->addCategory ("Core Properties");
  properties_->addProperty ("Radius", QVariant (radius), Qt::ItemIsSelectable, "Core Properties");
  properties_->addCategory ("Display Variables");
  properties_->addProperty ("Scale", QVariant (0.02), Qt::ItemIsEditable | Qt::ItemIsEnabled, "Display Variables");
  properties_->addProperty ("Level", QVariant (100), Qt::ItemIsEditable | Qt::ItemIsEnabled, "Display Variables");
}

pcl::cloud_composer::NormalsItem*
pcl::cloud_composer::NormalsItem::clone () const
{
  pcl::PointCloud<pcl::Normal>::Ptr normals_copy (new pcl::PointCloud<pcl::Normal> (*normals_ptr_));
  //Vector4f and Quaternionf do deep copies using copy constructor
  NormalsItem* new_item = new NormalsItem (this->text (), normals_copy, 0);
  
  PropertiesModel* new_item_properties = new_item->getPropertiesModel ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

pcl::cloud_composer::NormalsItem::~NormalsItem ()
{
  
}

void
pcl::cloud_composer::NormalsItem::paintView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{
  //Get the parent cloud, convert to XYZ 
  if (parent ()->type () == CLOUD_ITEM)
  {
    QVariant cloud_ptr = parent ()->data (ItemDataRole::CLOUD_BLOB);
    pcl::PCLPointCloud2::ConstPtr cloud_blob = cloud_ptr.value<pcl::PCLPointCloud2::ConstPtr> ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
    double scale = properties_->getProperty ("Scale").toDouble ();
    int level = properties_->getProperty ("Level").toInt ();
    qDebug () << "Removing old normals...";
    vis->removePointCloud (getId ().toStdString ());
    qDebug () << QString("Adding point cloud normals, level=%1, scale=%2").arg(level).arg(scale);
    vis->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals_ptr_, level, scale, getId ().toStdString ());
    std::cout << cloud->points[0]<<std::endl;
    std::cout << normals_ptr_->points[0]<<std::endl;
    
  }
  else
    qWarning () << "Normal item inserted, but parent not a cloud. Don't know how to draw that!";
}

void
pcl::cloud_composer::NormalsItem::removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{  
  //qDebug () << "Removing Normals "<<item_id_;
  vis->removePointCloud (getId ().toStdString ());
}

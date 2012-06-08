#include <QtGui>

#include <pcl/apps/cloud_composer/project_model.h>

pcl::cloud_composer::ProjectModel::ProjectModel (QObject* parent)
  : QStandardItemModel (parent)
{
  selection_model_ = new QItemSelectionModel(this);
}

pcl::cloud_composer::ProjectModel::ProjectModel (const ProjectModel& to_copy)
{
}

pcl::cloud_composer::ProjectModel::~ProjectModel ()
{
}

pcl::cloud_composer::ProjectModel::ProjectModel (QString project_name, QObject* parent)
: QStandardItemModel (parent)
{
  selection_model_ = new QItemSelectionModel(this);
  setName (project_name);
}

void 
pcl::cloud_composer::ProjectModel::setName (QString new_name)
{ 
  //If it hasn't been set yet
  if (!horizontalHeaderItem (0))
    setHorizontalHeaderItem (0, new QStandardItem (new_name));
  else
  {
    QStandardItem* header = horizontalHeaderItem (0);  
    header->setText(new_name);
  }
}

void
pcl::cloud_composer::ProjectModel::insertNewCloudFromFile (QString filename)
{
  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2);
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version;
  
  pcl::PCDReader pcd;
  if (pcd.read (filename.toStdString (), *cloud_blob, origin, orientation, version) < 0)
  {
    qDebug () << "Failed to read cloud from file";
    return;
  }
  if (cloud_blob->width * cloud_blob->height == 0)
  {
    qDebug () << "Cloud read has zero size!";
    return;
  }
  
  QFileInfo file_info (filename);
  QString short_filename = file_info.baseName ();
  //Check if this name already exists in the project - if so, append digit
  QList <QStandardItem*> items = findItems (short_filename);
  if (items.size () > 0)
  {
    int k = 2;
    items = findItems (short_filename+ tr ("-%1").arg (k));
    while (items.size () > 0)
    {  
      ++k;
      items = findItems (short_filename+ tr ("-%1").arg (k));
    }
    short_filename = short_filename+ tr ("-%1").arg (k);
  }
  QStandardItem* new_item = createNewCloudItem (cloud_blob, short_filename, origin, orientation);
   
  invisibleRootItem ()->appendRow (new_item);
}

QStandardItem* 
pcl::cloud_composer::ProjectModel::createNewCloudItem (sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                                                             QString cloud_name, 
                                                             Eigen::Vector4f origin, 
                                                             Eigen::Quaternionf orientation)
{
  QStandardItem* new_item = new QStandardItem (cloud_name);
  QVariant new_cloud = QVariant::fromValue (cloud_ptr);
  new_item->setData (new_cloud, CLOUD);
  
  //Create a color and geometry handler for this cloud
  ColorHandler::ConstPtr color_handler;
  color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_ptr));
  // QVariant new_color_handler = QVariant::fromValue (color_handler);
  new_item->setData (QVariant::fromValue (color_handler), COLOR);
  
  GeometryHandler::ConstPtr geometry_handler;
  geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2> (cloud_ptr));
  //QVariant new_geometry_handler = ;
  new_item->setData (QVariant::fromValue (geometry_handler), GEOMETRY);
  
  new_item->setData (QVariant::fromValue (origin), ORIGIN);
  new_item->setData (QVariant::fromValue (orientation), ORIENTATION);
  
  return new_item;
}
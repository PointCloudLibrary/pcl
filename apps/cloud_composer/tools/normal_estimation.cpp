#include <pcl/apps/cloud_composer/tools/normal_estimation.h>
#include <pcl/apps/cloud_composer/items/normals_item.h>

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>


Q_EXPORT_PLUGIN2(cloud_composer_normal_estimation_tool, pcl::cloud_composer::NormalEstimationToolFactory)


pcl::cloud_composer::NormalEstimationTool::NormalEstimationTool (PropertiesModel* parameter_model, QObject* parent)
  : NewItemTool (parameter_model, parent)
{

  
}

pcl::cloud_composer::NormalEstimationTool::~NormalEstimationTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::NormalEstimationTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.size () == 0)
  {
    qCritical () << "Empty input in Normal Estimation Tool!";
    return output;
  }
  else if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in Normal Estimation!";
  }
  input_item = input_data.value (0);
    
  pcl::PCLPointCloud2::ConstPtr input_cloud;
  if (input_item->type () == CloudComposerItem::CLOUD_ITEM)
  {
    double radius = parameter_model_->getProperty("Radius").toDouble();
    qDebug () << "Received Radius = " <<radius;
    pcl::PCLPointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
    qDebug () << "Got cloud size = "<<input_cloud->width;
    //////////////// THE WORK - COMPUTING NORMALS ///////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*input_cloud, *cloud);
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (radius);

    // Compute the features
    ne.compute (*cloud_normals);
    //////////////////////////////////////////////////////////////////
    NormalsItem* normals_item = new NormalsItem (tr("Normals r=%1").arg(radius),cloud_normals,radius);
    output.append (normals_item);
    qDebug () << "Calced normals";
  }
  else
  {
    qDebug () << "Input item in Normal Estimation is not a cloud!!!";
  }
  
  
  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::NormalEstimationToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Radius", 0.04,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  return parameter_model;
}

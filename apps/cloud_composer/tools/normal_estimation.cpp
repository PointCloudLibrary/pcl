#include <pcl/apps/cloud_composer/tools/normal_estimation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>


Q_EXPORT_PLUGIN2(cloud_composer_normal_estimation_tool, pcl::cloud_composer::NormalEstimationToolFactory)


pcl::cloud_composer::NormalEstimationTool::NormalEstimationTool (QObject* parent)
  : NewItemTool(parent)
{

  
}

pcl::cloud_composer::NormalEstimationTool::~NormalEstimationTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::NormalEstimationTool::performAction (QList <const CloudComposerItem*> input_data)
{
  QList <CloudComposerItem*> output;
  
  //QVariant cloud_ptr = item->data (CLOUD);
  //sensor_msgs::PointCloud2::ConstPtr input_cloud = cloud_ptr.value<sensor_msgs::PointCloud2::ConstPtr> ();
  //sensor_msgs::PointCloud2::Ptr cloud_copy (new sensor_msgs::PointCloud2 (*input_cloud));
  //QList <sensor_msgs::PointCloud2::Ptr> working_copy;
  //working_copy.append (cloud_copy);
  //QList <sensor_msgs::PointCloud2::Ptr> output = tool->performAction (working_copy);
  
  
  return output;
}


QStandardItemModel*
pcl::cloud_composer::NormalEstimationToolFactory::createToolParameterModel (QObject* parent)
{
  QStandardItemModel* parameter_model = new QStandardItemModel(parent);
  
  return parameter_model;
}
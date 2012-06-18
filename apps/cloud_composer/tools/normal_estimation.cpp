#include <pcl/apps/cloud_composer/tools/normal_estimation.h>

pcl::cloud_composer::NormalEstimationTool::NormalEstimationTool (QObject* parent)
  : NewItemTool(parent)
{

  
}

pcl::cloud_composer::NormalEstimationTool::~NormalEstimationTool ()
{
  
}

QList <sensor_msgs::PointCloud2::Ptr>
pcl::cloud_composer::NormalEstimationTool::performAction (QList <sensor_msgs::PointCloud2::ConstPtr> input_data)
{
  QList <sensor_msgs::PointCloud2::Ptr> output;
  
  
  return output;
}


QStandardItemModel*
pcl::cloud_composer::NormalEstimationToolFactory::createToolParameterModel (QObject* parent)
{
  QStandardItemModel* parameter_model = new QStandardItemModel(parent);
  
  return parameter_model;
}
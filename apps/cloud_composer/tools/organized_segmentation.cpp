#include <pcl/apps/cloud_composer/tools/organized_segmentation.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/apps/cloud_composer/tools/impl/organized_segmentation.hpp>

Q_EXPORT_PLUGIN2(cloud_composer_organized_segmentation_tool, pcl::cloud_composer::OrganizedSegmentationToolFactory)

pcl::cloud_composer::OrganizedSegmentationTool::OrganizedSegmentationTool (PropertiesModel* parameter_model, QObject* parent)
: SplitItemTool (parameter_model, parent)
{
  
}

pcl::cloud_composer::OrganizedSegmentationTool::~OrganizedSegmentationTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::OrganizedSegmentationTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  if (type != PointTypeFlags::NONE)
  {
    switch (type)
    {
      case (PointTypeFlags::XYZ):
        return this->performTemplatedAction<pcl::PointXYZ> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGB):
        return this->performTemplatedAction<pcl::PointXYZRGB> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGBA):
        return this->performTemplatedAction<pcl::PointXYZRGBA> (input_data);
    }
  }
  
  QList <CloudComposerItem*> output;
  
  qCritical () << "organized_segmentation requires templated types!";
  
  return output;
} 

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::OrganizedSegmentationToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Min Inliers", 1000,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Min Plane Size", 10000,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Angular Threshold", 2.0,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Distance Threshold", 0.02,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Cluster Dist. Thresh.", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Min Cluster Size", 1000,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  
  return parameter_model;
}
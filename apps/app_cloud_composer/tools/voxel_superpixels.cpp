#include <pcl/apps/cloud_composer/tools/voxel_superpixels.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/apps/cloud_composer/tools/impl/voxel_superpixels.hpp>

Q_EXPORT_PLUGIN2(cloud_composer_voxel_superpixels_tool, pcl::cloud_composer::VoxelSuperpixelsToolFactory)

pcl::cloud_composer::VoxelSuperpixelsTool::VoxelSuperpixelsTool (PropertiesModel* parameter_model, QObject* parent)
: SplitItemTool (parameter_model, parent)
{
  
}

pcl::cloud_composer::VoxelSuperpixelsTool::~VoxelSuperpixelsTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::VoxelSuperpixelsTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  if (type != PointTypeFlags::NONE)
  {
    switch (type)
    {
      case (PointTypeFlags::XYZ | PointTypeFlags::RGB):
        return this->performTemplatedAction<pcl::PointXYZRGB> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGBA):
        return this->performTemplatedAction<pcl::PointXYZRGBA> (input_data);
    }
  }
  
  QList <CloudComposerItem*> output;
  
  qCritical () << "voxel_superpixels requires templated types!";
  
  return output;
} 

template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::VoxelSuperpixelsTool::performTemplatedAction <pcl::PointXYZRGB> (QList <const CloudComposerItem*>);
//template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::VoxelSuperpixelsTool::performTemplatedAction <pcl::PointXYZRGBA> (QList <const CloudComposerItem*>);


/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::VoxelSuperpixelsToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Resolution", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Seed Resolution", 0.1,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  //parameter_model->addProperty ("Min Plane Size", 10000,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  //parameter_model->addProperty ("Angular Threshold", 2.0,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  //parameter_model->addProperty ("Distance Threshold", 0.02,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  //parameter_model->addProperty ("Cluster Dist. Thresh.", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  //parameter_model->addProperty ("Min Cluster Size", 1000,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  
  return parameter_model;
}
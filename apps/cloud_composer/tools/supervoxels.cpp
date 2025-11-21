#include <pcl/apps/cloud_composer/tools/supervoxels.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/apps/cloud_composer/tools/impl/supervoxels.hpp>

Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")

pcl::cloud_composer::SupervoxelsTool::SupervoxelsTool (PropertiesModel* parameter_model, QObject* parent)
: SplitItemTool (parameter_model, parent)
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::SupervoxelsTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  if (type != PointTypeFlags::NONE)
  {
    switch ((std::uint8_t) type)
    {
      case (PointTypeFlags::XYZ | PointTypeFlags::RGB):
        return this->performTemplatedAction<pcl::PointXYZRGB> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGBA):
        return this->performTemplatedAction<pcl::PointXYZRGBA> (input_data);
    }
  }
  
  QList <CloudComposerItem*> output;
  
  qCritical () << "supervoxels requires templated types!";
  
  return output;
} 

template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::SupervoxelsTool::performTemplatedAction <pcl::PointXYZRGB> (const QList <const CloudComposerItem*>&);
//template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::SupervoxelsTool::performTemplatedAction <pcl::PointXYZRGBA> (const QList <const CloudComposerItem*>&);


/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::SupervoxelsToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Resolution", 0.008,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Seed Resolution", 0.08,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("RGB Weight", 0.2,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Normals Weight", 0.8,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Spatial Weight", 0.4,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  return parameter_model;
}

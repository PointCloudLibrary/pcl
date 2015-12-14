#include <pcl/apps/cloud_composer/tools/supervoxels.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/apps/cloud_composer/tools/impl/supervoxels.hpp>

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  Q_EXPORT_PLUGIN2(cloud_composer_voxel_superpixels_tool, pcl::cloud_composer::SupervoxelsToolFactory)
#else
  Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")
#endif

pcl::cloud_composer::SupervoxelsTool::SupervoxelsTool (PropertiesModel* parameter_model, QObject* parent)
: SplitItemTool (parameter_model, parent)
{
  
}

pcl::cloud_composer::SupervoxelsTool::~SupervoxelsTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::SupervoxelsTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
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
  
  qCritical () << "supervoxels requires templated types!";
  
  return output;
} 

template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::SupervoxelsTool::performTemplatedAction <pcl::PointXYZRGB> (QList <const CloudComposerItem*>);
//template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::SupervoxelsTool::performTemplatedAction <pcl::PointXYZRGBA> (QList <const CloudComposerItem*>);


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

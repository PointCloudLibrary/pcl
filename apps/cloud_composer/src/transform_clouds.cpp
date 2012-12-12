#include <pcl/apps/cloud_composer/transform_clouds.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

#include <pcl/apps/cloud_composer/impl/transform_clouds.hpp>

pcl::cloud_composer::TransformClouds::TransformClouds (QMap <QString, vtkSmartPointer<vtkMatrix4x4> > transform_map, QObject* parent)
  : ModifyItemTool (0, parent)
  , transform_map_ (transform_map)
{
  
}

pcl::cloud_composer::TransformClouds::~TransformClouds ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::TransformClouds::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
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

  qCritical () << "Transform requires templated types!";
  
  return output;
}
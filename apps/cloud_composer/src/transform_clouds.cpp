#include <pcl/apps/cloud_composer/transform_clouds.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/point_types.h>

#include <pcl/apps/cloud_composer/impl/transform_clouds.hpp>

pcl::cloud_composer::TransformClouds::TransformClouds (QMap <QString, vtkSmartPointer<vtkMatrix4x4> > transform_map, QObject* parent)
  : ModifyItemTool (nullptr, parent)
  , transform_map_ (std::move(transform_map))
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::TransformClouds::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  if (type != PointTypeFlags::NONE)
  {
    switch ((std::uint8_t) type)
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

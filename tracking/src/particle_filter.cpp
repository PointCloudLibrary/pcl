#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/impl/particle_filter.hpp>

#define PCL_TRACKING_NORMAL_SUPPORTED
PCL_INSTANTIATE_PRODUCT(ParticleFilterTracker, ((pcl::PointNormal) (pcl::PointXYZINormal) (pcl::PointXYZRGBNormal))(PCL_STATE_POINT_TYPES))
#undef PCL_TRACKING_NORMAL_SUPPORTED
PCL_INSTANTIATE_PRODUCT(ParticleFilterTracker, ((pcl::PointXYZ) (pcl::PointXYZI) (pcl::PointXYZRGBA) (pcl::PointXYZRGB) (pcl::InterestPoint) (pcl::PointWithRange) (pcl::PointWithViewpoint) (pcl::PointWithScale))(PCL_STATE_POINT_TYPES))


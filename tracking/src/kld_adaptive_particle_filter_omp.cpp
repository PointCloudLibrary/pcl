#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/impl/kld_adaptive_particle_filter_omp.hpp>

#define PCL_TRACKING_NORMAL_SUPPORTED
PCL_INSTANTIATE_PRODUCT(KLDAdaptiveParticleFilterOMPTracker, ((pcl::PointNormal) (pcl::PointXYZINormal) (pcl::PointXYZRGBNormal))(PCL_STATE_POINT_TYPES))
#undef PCL_TRACKING_NORMAL_SUPPORTED
PCL_INSTANTIATE_PRODUCT(KLDAdaptiveParticleFilterOMPTracker, ((pcl::PointXYZ) (pcl::PointXYZI) (pcl::PointXYZRGBA) (pcl::PointXYZRGB) (pcl::InterestPoint) (pcl::PointWithRange) (pcl::PointWithViewpoint) (pcl::PointWithScale))(PCL_STATE_POINT_TYPES))


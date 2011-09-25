#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/tracking/particle_filter.h"
#include "pcl/tracking/impl/particle_filter.hpp"

// particlefiltertracker requires xyz and normal
PCL_INSTANTIATE_PRODUCT(ParticleFilterTracker, ((pcl::PointNormal) (pcl::PointXYZINormal) (pcl::PointXYZRGBNormal))(PCL_STATE_POINT_TYPES));

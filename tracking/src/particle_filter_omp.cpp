#include "pcl/impl/instantiate.hpp"
#include "pcl/tracking/particle_filter_omp.h"
#include "pcl/tracking/impl/particle_filter_omp.hpp"

PCL_INSTANTIATE_PRODUCT(ParticleFilterOMPTracker, ((pcl::PointNormal) (pcl::PointXYZINormal) (pcl::PointXYZRGBNormal))(PCL_STATE_POINT_TYPES));

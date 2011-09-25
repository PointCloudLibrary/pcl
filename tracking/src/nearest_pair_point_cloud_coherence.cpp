#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "pcl/tracking/nearest_pair_point_cloud_coherence.h"
#include "pcl/tracking/impl/nearest_pair_point_cloud_coherence.hpp"

PCL_INSTANTIATE_PRODUCT(NearestPairPointCloudCoherence, (PCL_XYZ_POINT_TYPES));

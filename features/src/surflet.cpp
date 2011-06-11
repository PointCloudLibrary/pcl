#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"
#include "pcl/features/surflet.h"
#include "pcl/features/impl/surflet.hpp"


// Instantiations of specific point types
/// @TODO add some kind of output point type for the surflet feature hash map
PCL_INSTANTIATE_PRODUCT(SurfletModelEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES));

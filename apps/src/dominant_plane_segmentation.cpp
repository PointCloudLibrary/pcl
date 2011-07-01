#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/apps/impl/dominant_plane_segmentation.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(DominantPlaneSegmentation, PCL_XYZ_POINT_TYPES);

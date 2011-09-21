#ifndef PCL_TRACKING_TRACKING_H_
#define PCL_TRACKING_TRACKING_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>

namespace pcl
{
  namespace tracking
  {
    /* state definition */
    struct ParticleXYZRPY;
  }
}

#include <pcl/tracking/impl/tracking.hpp>

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::tracking::_ParticleXYZRPY,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYZRPY, pcl::tracking::_ParticleXYZRPY)


#endif

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#ifndef PCL_KINFU_POINT_INTENSITY_
#define PCL_KINFU_POINT_INTENSITY_

struct EIGEN_ALIGN16 PointIntensity
{

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  union
  {
    struct
    {
      float intensity;
    };
    float data[4];
  };
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointIntensity,
    (float, intensity, intensity) )

#endif

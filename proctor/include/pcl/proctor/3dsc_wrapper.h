#ifndef PCL_PROCTOR_3DSC_WRAPPER_H
#define PCL_PROCTOR_3DSC_WRAPPER_H

#include "proctor/feature_wrapper.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  namespace proctor
  {

    class ShapeContextWrapper : public FeatureWrapper {
      public:
        ShapeContextWrapper() : FeatureWrapper(std::string("3d shape context"))
        {}

        void
        compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output);

      private:
    };

  }
}

#endif

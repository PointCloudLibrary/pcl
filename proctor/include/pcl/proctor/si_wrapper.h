#ifndef SI_WRAPPER_H
#define SI_WRAPPER_H

#include "proctor/feature_wrapper.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  namespace proctor
  {

    class SIWrapper : public FeatureWrapper {
      public:
        SIWrapper() : FeatureWrapper(std::string("spin image"))
        {}

        void
        compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output);

      private:
    };

  }
}

#endif

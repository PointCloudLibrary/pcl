#ifndef FEATURE_WRAPPER_H
#define FEATURE_WRAPPER_H

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace pcl
{
  namespace proctor
  {

    class FeatureWrapper {
      public:
        typedef boost::shared_ptr<FeatureWrapper> Ptr;
        typedef boost::shared_ptr<const FeatureWrapper> ConstPtr;

        typedef PointNormal PointInT;
        typedef PointNormal PointOutT;
        typedef pcl::PointCloud<PointInT> PointCloudIn;
        typedef PointCloudIn::Ptr PointCloudInPtr;
        typedef PointCloudIn::ConstPtr PointCloudInConstPtr;
        typedef pcl::PointCloud<PointOutT> PointCloudOut;
        typedef PointCloudOut::Ptr PointCloudOutPtr;
        typedef PointCloudOut::ConstPtr PointCloudOutConstPtr;

        FeatureWrapper()
        {}

        virtual void
        compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output) = 0;

      private:
    };

  }
}

#endif

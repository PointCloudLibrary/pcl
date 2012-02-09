#ifndef KEYPOINT_WRAPPER_H
#define KEYPOINT_WRAPPER_H

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace pcl
{
  namespace proctor
  {

    class KeypointWrapper {
      public:
        typedef boost::shared_ptr<KeypointWrapper> Ptr;
        typedef boost::shared_ptr<const KeypointWrapper> ConstPtr;

        typedef PointNormal PointInT;
        typedef PointNormal PointOutT;
        typedef pcl::PointCloud<PointInT> PointCloudIn;
        typedef PointCloudIn::Ptr PointCloudInPtr;
        typedef PointCloudIn::ConstPtr PointCloudInConstPtr;
        typedef pcl::PointCloud<PointOutT> PointCloudOut;
        typedef PointCloudOut::Ptr PointCloudOutPtr;
        typedef PointCloudOut::ConstPtr PointCloudOutConstPtr;

        KeypointWrapper()
        {}

        virtual void
        compute(PointCloudInPtr input, PointCloudOut &output) = 0;

      private:
    };

  }
}

#endif

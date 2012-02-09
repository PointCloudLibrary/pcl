#ifndef HARRIS_WRAPPER_H
#define HARRIS_WRAPPER_H

#include "proctor/keypoint_wrapper.h"

namespace pcl
{
  namespace proctor
  {

    class HarrisWrapper : public KeypointWrapper {
      public:
        HarrisWrapper()
        {}

        void
        compute(PointCloudInPtr input, PointCloudOut &output);

      private:
    };

  }
}

#endif

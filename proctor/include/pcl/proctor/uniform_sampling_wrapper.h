#ifndef UNIFORM_SAMPLING_WRAPPER_H
#define UNIFORM_SAMPLING_WRAPPER_H

#include "proctor/keypoint_wrapper.h"

namespace pcl
{
  namespace proctor
  {

    class UniformSamplingWrapper : public KeypointWrapper {
      public:
        UniformSamplingWrapper()
        {}

        void
        compute(PointCloudInPtr input, PointCloudOut &output);

      private:
    };

  }
}

#endif

#include "proctor/uniform_sampling_wrapper.h"

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/io.h>
#include <vector>

namespace pcl
{

  namespace proctor
  {
    void
    UniformSamplingWrapper::compute(PointCloudInPtr input, PointCloudOut &output)
    {
      float keypoint_separation = 0.01f;

      IndicesPtr indices (new std::vector<int>());
      PointCloud<int> leaves;
      UniformSampling<PointNormal> us;
      us.setRadiusSearch(keypoint_separation);
      us.setInputCloud(input);
      us.compute(leaves);

      // Copy point cloud and return
      indices->assign(leaves.points.begin(), leaves.points.end()); // can't use operator=, probably because of different allocators
      copyPointCloud(*input, *indices, output);
    }
  }

}


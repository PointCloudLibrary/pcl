#include "proctor/harris_wrapper.h"

#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/common/io.h>
#include <vector>

namespace pcl
{

  namespace proctor
  {
    void
    HarrisWrapper::compute(PointCloudInPtr input, PointCloudOut &output)
    {
      HarrisKeypoint3D<PointInT, PointXYZI> keypoint_est;
      keypoint_est.setMethod (HarrisKeypoint3D<PointInT, PointXYZI>::LOWE);
      keypoint_est.setRadius (0.01f);
      //keypoint_est.setRadiusSearch (0.0001);
      keypoint_est.setKSearch(10);
      //keypoint_est.setNonMaxSupression (true);
      keypoint_est.setRefine(false);
      keypoint_est.setInputCloud(input);

      PointCloud<PointXYZI>::Ptr keypoints (new PointCloud<PointXYZI>());
      keypoint_est.compute(*keypoints);

      PointCloud<PointXYZ>::Ptr no_intensity (new PointCloud<PointXYZ>());
      copyPointCloud(*keypoints, *no_intensity);


      PointCloud<PointXYZ>::Ptr dense_keypoints (new PointCloud<PointXYZ>());
      std::vector<int> index;
      no_intensity->is_dense = false;
      removeNaNFromPointCloud(*no_intensity, *dense_keypoints, index);

      //copyPointCloud(*keypoints, output);
      //output.width = keypoints.width;
      //output.height = keypoints.height;
      //std::cout << *keypoints << std::endl;
      //std::cout << keypoints->at(0) << std::endl;
      //std::cout << keypoints->at(1) << std::endl;
      //std::cout << dense_keypoints->is_dense << std::endl;
      //for (int i = 0; i < dense_keypoints->size(); i++) {
        //std::cout << dense_keypoints->at(i) << std::endl;
      //}

      PointCloud<Normal>::Ptr pcn (new PointCloud<Normal>());
      NormalEstimation<PointXYZ, Normal> ne;
      search::KdTree<PointXYZ>::Ptr kdt (new search::KdTree<PointXYZ>());
      ne.setInputCloud(dense_keypoints);
      ne.setSearchMethod(kdt);
      ne.setKSearch(20);
      //ne.setViewPoint(vx, vy, vz);
      ne.compute(*pcn);

      PointCloud<PointXYZINormal> done;
      concatenateFields(*dense_keypoints, *pcn, output);


      //float keypoint_separation = 0.05;

      //IndicesPtr indices (new std::vector<int>());
      //PointCloud<int> leaves;
      //UniformSampling<PointNormal> us;
      //us.setRadiusSearch(keypoint_separation);
      //us.setInputCloud(input);
      //us.compute(leaves);

      //// Copy point cloud and return
      //indices->assign(leaves.points.begin(), leaves.points.end()); // can't use operator=, probably because of different allocators
      //copyPointCloud(*input, *indices, output);
    }
  }

}


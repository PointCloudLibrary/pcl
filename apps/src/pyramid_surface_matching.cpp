/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/pyramid_feature_matching.h>

using namespace pcl;

#include <iostream>
using namespace std;

const Eigen::Vector4f subsampling_leaf_size (0.02f, 0.02f, 0.02f, 0.0f);
const float normal_estimation_search_radius = 0.05f;

//const Eigen::Vector4f subsampling_leaf_size (5, 5, 5, 0.0);
//const float normal_estimation_search_radius = 11;

void
subsampleAndCalculateNormals (PointCloud<PointXYZ>::Ptr &cloud,
                              PointCloud<PointXYZ>::Ptr &cloud_subsampled,
                              PointCloud<Normal>::Ptr &cloud_subsampled_normals)
{
  cloud_subsampled = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  cloud_subsampled_normals = PointCloud<Normal>::Ptr (new PointCloud<Normal> ());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled);
  search::KdTree<PointXYZ>::Ptr search_tree (new search::KdTree<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (*cloud_subsampled_normals);

  cerr << "Before -> After subsampling: " << cloud->points.size () << " -> " << cloud_subsampled->points.size () << endl;
}


int
main (int argc, char **argv)
{
  if (argc != 3)
  {
    PCL_ERROR ("Syntax: ./pyramid_surface_matching model_1 model_2\n");
    return -1;
  }


  /// read point clouds from HDD
  PCL_INFO ("Reading scene ...\n");
  PointCloud<PointXYZ>::Ptr cloud_a (new PointCloud<PointXYZ> ()),
      cloud_b (new PointCloud<PointXYZ> ());
  PCDReader reader;
  reader.read (argv[1], *cloud_a);
  reader.read (argv[2], *cloud_b);

  PointCloud<PointXYZ>::Ptr cloud_a_subsampled, cloud_b_subsampled;
  PointCloud<Normal>::Ptr cloud_a_subsampled_normals, cloud_b_subsampled_normals;
  subsampleAndCalculateNormals (cloud_a, cloud_a_subsampled, cloud_a_subsampled_normals);
  subsampleAndCalculateNormals (cloud_b, cloud_b_subsampled, cloud_b_subsampled_normals);

  PCL_INFO ("Finished subsampling the clouds ...\n");


  PointCloud<PPFSignature>::Ptr ppf_signature_a (new PointCloud<PPFSignature> ()),
      ppf_signature_b (new PointCloud<PPFSignature> ());
  PointCloud<PointNormal>::Ptr cloud_subsampled_with_normals_a (new PointCloud<PointNormal> ()),
      cloud_subsampled_with_normals_b (new PointCloud<PointNormal> ());
  concatenateFields (*cloud_a_subsampled, *cloud_a_subsampled_normals, *cloud_subsampled_with_normals_a);
  concatenateFields (*cloud_b_subsampled, *cloud_b_subsampled_normals, *cloud_subsampled_with_normals_b);

  PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;
  ppf_estimator.setInputCloud (cloud_subsampled_with_normals_a);
  ppf_estimator.setInputNormals (cloud_subsampled_with_normals_a);
  ppf_estimator.compute (*ppf_signature_a);
  ppf_estimator.setInputCloud (cloud_subsampled_with_normals_b);
  ppf_estimator.setInputNormals (cloud_subsampled_with_normals_b);
  ppf_estimator.compute (*ppf_signature_b);

  PCL_INFO ("Feature cloud sizes: %u , %u\n", ppf_signature_a->points.size (), ppf_signature_b->points.size ());

  PCL_INFO ("Finished calculating the features ...\n");
  vector<pair<float, float> > dim_range_input, dim_range_target;
  for (size_t i = 0; i < 3; ++i) dim_range_input.push_back (pair<float, float> (float (-M_PI), float (M_PI)));
  dim_range_input.push_back (pair<float, float> (0.0f, 1.0f));
  for (size_t i = 0; i < 3; ++i) dim_range_target.push_back (pair<float, float> (float (-M_PI) * 10.0f, float (M_PI) * 10.0f));
  dim_range_target.push_back (pair<float, float> (0.0f, 50.0f));


  PyramidFeatureHistogram<PPFSignature>::Ptr pyramid_a (new PyramidFeatureHistogram<PPFSignature> ());
  pyramid_a->setInputCloud (ppf_signature_a);
  pyramid_a->setInputDimensionRange (dim_range_input);
  pyramid_a->setTargetDimensionRange (dim_range_target);
  pyramid_a->compute ();
  PCL_INFO ("Done with the first pyramid\n");

  PyramidFeatureHistogram<PPFSignature>::Ptr pyramid_b (new PyramidFeatureHistogram<PPFSignature> ());
  pyramid_b->setInputCloud (ppf_signature_b);
  pyramid_b->setInputDimensionRange (dim_range_input);
  pyramid_b->setTargetDimensionRange (dim_range_target);
  pyramid_b->compute ();
  PCL_INFO ("Done with the second pyramid\n");

  float value = PyramidFeatureHistogram<PPFSignature>::comparePyramidFeatureHistograms (pyramid_a, pyramid_b);
  PCL_INFO ("Surface comparison value between %s and %s is: %f\n", argv[1], argv[2], value);


  return 0;
}

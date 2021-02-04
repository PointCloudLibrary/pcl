/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */
#include <pcl/test/gtest.h>
#include <pcl/common/random.h>
#include <pcl/ml/kmeans.h>

using namespace pcl;
using namespace pcl::common;

using Point = std::vector<float>;

// Prepare random number generator in PCL
UniformGenerator<float> engine (-100000.0, 100000.0, 2021);

class SampleDataChecker
{
  public:
    int data_size_;
    int dim_;
    int cluster_size_;
    std::vector<Point> data_sequence_;
    std::vector<Point> answer_centroids_;

    // Create sample data
    void
    createDataSequence ()
    {
      for (int data_id = 0; data_id < data_size_; ++data_id)
      {
        Point data;
        for (int dim_i = 0; dim_i < dim_; ++dim_i)
          data.push_back (engine.run ());
        data_sequence_.push_back (data);
      }
    }

    void
    testKmeans (Kmeans& k_means)
    {
      k_means.setClusterSize (cluster_size_);
      k_means.setInputData (data_sequence_);
      k_means.initialClusterPoints ();
      k_means.computeCentroids ();

      // Input centroids that should be the correct answer
      answer_centroids_ = k_means.get_centroids ();

      // If centroids_ was initialized before calculating it,
      // then it should not change
      // no matter how many times this class method is called.
      k_means.computeCentroids ();
      k_means.computeCentroids ();
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ComputeCentroids, Case1)
{
  // Create sample data sequence
  SampleDataChecker sdc;
  sdc.data_size_ = 20;
  sdc.dim_ = 21;
  sdc.cluster_size_ = 9;
  sdc.createDataSequence ();

  // Compute centroids with K-means
  Kmeans k_means (sdc.data_size_, sdc.dim_);
  sdc.testKmeans (k_means);

  // Evaluate if the two centroids are the same
  EXPECT_EQ (sdc.cluster_size_, k_means.get_centroids ().size ());
  EXPECT_EQ (sdc.answer_centroids_, k_means.get_centroids ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ComputeCentroids, Case2)
{
  // Create sample data sequence
  SampleDataChecker sdc;
  sdc.data_size_ = 1;
  sdc.dim_ = 1;
  sdc.cluster_size_ = 1;
  sdc.createDataSequence ();

  // Compute centroids with K-means
  Kmeans k_means (sdc.data_size_, sdc.dim_);
  sdc.testKmeans (k_means);

  // Evaluate if the two centroids are the same
  EXPECT_EQ (sdc.cluster_size_, k_means.get_centroids ().size ());
  EXPECT_EQ (sdc.answer_centroids_, k_means.get_centroids ());
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

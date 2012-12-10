/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/gfpfh.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> void
testIndicesAndSearchSurface (const typename PointCloud<PointT>::Ptr & points,
                             const typename PointCloud<NormalT>::Ptr & normals,
                             const boost::shared_ptr<vector<int> > & indices, int ndims)
{
  //
  // Test setIndices and setSearchSurface
  //
  PointCloud<OutputT> full_output, output0, output1, output2;

  // Compute for all points and then subsample the results
  FeatureEstimation est0;
  est0.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est0.setKSearch (10);
  est0.setInputCloud (points);
  est0.setInputNormals (normals);
  est0.compute (full_output);
  copyPointCloud (full_output, *indices, output0);

  // Compute with all points as "search surface" and the specified sub-cloud as "input"
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT>);
  copyPointCloud (*points, *indices, *subpoints);
  FeatureEstimation est1;
  est1.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est1.setKSearch (10);
  est1.setInputCloud (subpoints);
  est1.setSearchSurface (points);
  est1.setInputNormals (normals);
  est1.compute (output1);

  // Compute with all points as "input" and the specified indices
  FeatureEstimation est2;
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est2.setKSearch (10);
  est2.setInputCloud (points);
  est2.setInputNormals (normals);
  est2.setIndices (indices);
  est2.compute (output2);

  // All three of the above cases should produce equivalent results
  ASSERT_EQ (output0.size (), output1.size ());
  ASSERT_EQ (output1.size (), output2.size ());
  for (size_t i = 0; i < output1.size (); ++i)
  {
    for (int j = 0; j < ndims; ++j)
    {
      ASSERT_EQ (output0.points[i].histogram[j], output1.points[i].histogram[j]);
      ASSERT_EQ (output1.points[i].histogram[j], output2.points[i].histogram[j]);
    }
  }

  //
  // Test the combination of setIndices and setSearchSurface
  //
  PointCloud<OutputT> output3, output4;

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (static_cast<int> (i));

  // Compute with all points as search surface + the specified sub-cloud as "input" but for only a subset of indices
  FeatureEstimation est3;
  est3.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est3.setKSearch (10);
  est3.setSearchSurface (points);
  est3.setInputNormals (normals);
  est3.setInputCloud (subpoints);
  est3.setIndices (indices2);
  est3.compute (output3);

  // Start with features for each point in "subpoints" and then subsample the results
  copyPointCloud (output0, *indices2, output4); // (Re-using "output0" from above)

  // The two cases above should produce equivalent results
  ASSERT_EQ (output3.size (), output4.size ());
  for (size_t i = 0; i < output3.size (); ++i)
  {
    for (int j = 0; j < ndims; ++j)
    {
      ASSERT_EQ (output3.points[i].histogram[j], output4.points[i].histogram[j]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PFHEstimation)
{
  float f1, f2, f3, f4;

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  PFHEstimation<PointXYZ, Normal, PFHSignature125> pfh;
  pfh.setInputNormals (normals);
  EXPECT_EQ (pfh.getInputNormals (), normals);

  // computePairFeatures
  pfh.computePairFeatures (cloud, *normals, 0, 12, f1, f2, f3, f4);
  EXPECT_NEAR (f1, -0.072575, 1e-4);
  EXPECT_NEAR (f2, -0.040221, 1e-4);
  EXPECT_NEAR (f3, 0.068133, 1e-4);
  EXPECT_NEAR (f4, 0.006130, 1e-4);

  // computePointPFHSignature
  int nr_subdiv = 3;
  Eigen::VectorXf pfh_histogram (nr_subdiv * nr_subdiv * nr_subdiv);
  pfh.computePointPFHSignature (cloud, *normals, indices, nr_subdiv, pfh_histogram);
  EXPECT_NEAR (pfh_histogram[0],  0.932506, 1e-2);
  EXPECT_NEAR (pfh_histogram[1],  2.32429 , 1e-2);
  EXPECT_NEAR (pfh_histogram[2],  0.357477, 1e-2);
  EXPECT_NEAR (pfh_histogram[3],  0.848541, 1e-2);
  EXPECT_NEAR (pfh_histogram[4],  3.65565 , 2e-2); // larger error w.r.t. considering all point pairs (feature bins=0,1,1 where 1 is middle, so angle of 0)
  EXPECT_NEAR (pfh_histogram[5],  0.178104, 1e-2);
  EXPECT_NEAR (pfh_histogram[6],  1.45284 , 1e-2);
  EXPECT_NEAR (pfh_histogram[7],  3.60666 , 1e-2);
  EXPECT_NEAR (pfh_histogram[8],  0.298959, 1e-2);
  EXPECT_NEAR (pfh_histogram[9],  0.295143, 1e-2);
  EXPECT_NEAR (pfh_histogram[10], 2.13474 , 1e-2);
  EXPECT_NEAR (pfh_histogram[11], 0.41218 , 1e-2);
  EXPECT_NEAR (pfh_histogram[12], 0.165382, 1e-2);
  EXPECT_NEAR (pfh_histogram[13], 8.97407 , 1e-2);
  EXPECT_NEAR (pfh_histogram[14], 0.306592, 1e-2);
  EXPECT_NEAR (pfh_histogram[15], 0.455432, 1e-2);
  EXPECT_NEAR (pfh_histogram[16], 4.5977 ,  1e-2);
  EXPECT_NEAR (pfh_histogram[17], 0.393097, 1e-2);
  EXPECT_NEAR (pfh_histogram[18], 7.54668 , 1e-2);
  EXPECT_NEAR (pfh_histogram[19], 6.78336 , 1e-2);
  EXPECT_NEAR (pfh_histogram[20], 1.63858 , 1e-2);
  EXPECT_NEAR (pfh_histogram[21], 9.93842 , 1e-2);
  EXPECT_NEAR (pfh_histogram[22], 18.4947 , 2e-2); // larger error w.r.t. considering all point pairs (feature bins=2,1,1 where 1 is middle, so angle of 0)
  EXPECT_NEAR (pfh_histogram[23], 1.96553 , 1e-4);
  EXPECT_NEAR (pfh_histogram[24], 8.04793 , 1e-4);
  EXPECT_NEAR (pfh_histogram[25], 11.2793  , 1e-4);
  EXPECT_NEAR (pfh_histogram[26], 2.91714 , 1e-4);

  // Sum of values should be 100
  EXPECT_NEAR (pfh_histogram.sum (), 100.0, 1e-2);
  //std::cerr << pfh_histogram << std::endl;

  // Object
  PointCloud<PFHSignature125>::Ptr pfhs (new PointCloud<PFHSignature125> ());

  // set parameters
  pfh.setInputCloud (cloud.makeShared ());
  pfh.setIndices (indicesptr);
  pfh.setSearchMethod (tree);
  pfh.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  pfh.compute (*pfhs);
  EXPECT_EQ (pfhs->points.size (), indices.size ());

  for (size_t i = 0; i < pfhs->points.size (); ++i)
  {
    EXPECT_NEAR (pfhs->points[i].histogram[0],  0.156477  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[1],  0.539396  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[2],  0.410907  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[3],  0.184465  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[4],  0.115767  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[5],  0.0572475 , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[6],  0.206092  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[7],  0.339667  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[8],  0.265883  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[9],  0.0038165 , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[10], 0.103046  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[11], 0.214997  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[12], 0.398186  , 3e-2); // larger error w.r.t. considering all point pairs (feature bins=0,2,2 where 2 is middle, so angle of 0)
    EXPECT_NEAR (pfhs->points[i].histogram[13], 0.298959  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[14], 0.00127217, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[15], 0.11704   , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[16], 0.255706  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[17], 0.356205  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[18], 0.265883  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[19], 0.00127217, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[20], 0.148844  , 1e-4);
    //EXPECT_NEAR (pfhs->points[i].histogram[21], 0.721316  , 1e-3);
    //EXPECT_NEAR (pfhs->points[i].histogram[22], 0.438899  , 1e-2);
    EXPECT_NEAR (pfhs->points[i].histogram[23], 0.22263   , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[24], 0.0216269 , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[25], 0.223902  , 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[26], 0.07633   , 1e-4);
  }
  //Eigen::Map<Eigen::VectorXf> h (&(pfhs->points[0].histogram[0]), 125);
  //std::cerr << h.head<27> () << std::endl;

  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testIndicesAndSearchSurface<PFHEstimation<PointXYZ, Normal, PFHSignature125>, PointXYZ, Normal, PFHSignature125>
  (cloud.makeShared (), normals, test_indices, 125);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPFHEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
  fpfh.setInputNormals (normals);
  EXPECT_EQ (fpfh.getInputNormals (), normals);

  // computePointSPFHSignature
  int nr_subdiv = 11; // use the same number of bins for all three angular features
  Eigen::MatrixXf hist_f1 (indices.size (), nr_subdiv), hist_f2 (indices.size (), nr_subdiv), hist_f3 (indices.size (), nr_subdiv);
  hist_f1.setZero (); hist_f2.setZero (); hist_f3.setZero ();
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
    fpfh.computePointSPFHSignature (cloud, *normals, i, i, indices, hist_f1, hist_f2, hist_f3);

  EXPECT_NEAR (hist_f1 (0, 0), 0.757576, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 1), 0.757576, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 2), 4.54545,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 3), 19.697,   1e-4);
  EXPECT_NEAR (hist_f1 (0, 4), 40.6566,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 5), 21.4647,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 6), 7.575759, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 7), 0.000000, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 8), 0.000000, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 9), 0.50505,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 10), 4.0404,  1e-4);

  EXPECT_NEAR (hist_f2 (0, 0), 0.757576, 1e-4);
  EXPECT_NEAR (hist_f2 (0, 1), 1.51515,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 2), 6.31313,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 3), 9.59596,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 4), 20.7071,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 5), 18.9394,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 6), 15.9091,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 7), 12.8788,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 8), 6.56566,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 9), 4.29293,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 10), 2.52525, 1e-4);

  EXPECT_NEAR (hist_f3 (0, 0), 0.000000, 1e-4);
  EXPECT_NEAR (hist_f3 (0, 1), 5.05051,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 2), 4.54545,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 3), 5.05051,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 4), 1.76768,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 5), 3.0303,   1e-4);
  EXPECT_NEAR (hist_f3 (0, 6), 9.09091,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 7), 31.8182,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 8), 22.2222,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 9), 11.8687,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 10), 5.55556, 1e-4);

  // weightPointSPFHSignature
  Eigen::VectorXf fpfh_histogram (nr_subdiv + nr_subdiv + nr_subdiv);
  fpfh_histogram.setZero ();
  vector<float> dists (indices.size ());
  for (size_t i = 0; i < dists.size (); ++i) dists[i] = static_cast<float> (i);
  fpfh.weightPointSPFHSignature (hist_f1, hist_f2, hist_f3, indices, dists, fpfh_histogram);

  EXPECT_NEAR (fpfh_histogram[0],  1.9798 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[1],  2.86927,  1e-2);
  EXPECT_NEAR (fpfh_histogram[2],  8.47911,  1e-2);
  EXPECT_NEAR (fpfh_histogram[3],  22.8784,  1e-2);
  EXPECT_NEAR (fpfh_histogram[4],  29.8597,  1e-2);
  EXPECT_NEAR (fpfh_histogram[5],  19.6877,  1e-2);
  EXPECT_NEAR (fpfh_histogram[6],  7.38611,  1e-2);
  EXPECT_NEAR (fpfh_histogram[7],  1.44265,  1e-2);
  EXPECT_NEAR (fpfh_histogram[8],  0.69677,  1e-2);
  EXPECT_NEAR (fpfh_histogram[9],  1.72609,  1e-2);
  EXPECT_NEAR (fpfh_histogram[10], 2.99435,  1e-2);
  EXPECT_NEAR (fpfh_histogram[11], 2.26313,  1e-2);
  EXPECT_NEAR (fpfh_histogram[12], 5.16573,  1e-2);
  EXPECT_NEAR (fpfh_histogram[13], 8.3263 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[14], 9.92427,  1e-2);
  EXPECT_NEAR (fpfh_histogram[15], 16.8062,  1e-2);
  EXPECT_NEAR (fpfh_histogram[16], 16.2767,  1e-2);
  EXPECT_NEAR (fpfh_histogram[17], 12.251 ,  1e-2);
  //EXPECT_NEAR (fpfh_histogram[18], 10.354,  1e-1);
  //EXPECT_NEAR (fpfh_histogram[19], 6.65578,  1e-2);
  EXPECT_NEAR (fpfh_histogram[20], 6.1437 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[21], 5.83341,  1e-2);
  EXPECT_NEAR (fpfh_histogram[22], 1.08809,  1e-2);
  EXPECT_NEAR (fpfh_histogram[23], 3.34133,  1e-2);
  EXPECT_NEAR (fpfh_histogram[24], 5.59236,  1e-2);
  EXPECT_NEAR (fpfh_histogram[25], 5.6355 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[26], 3.03257,  1e-2);
  EXPECT_NEAR (fpfh_histogram[27], 1.37437,  1e-2);
  EXPECT_NEAR (fpfh_histogram[28], 7.99746,  1e-2);
  EXPECT_NEAR (fpfh_histogram[29], 18.0343,  1e-2);
  EXPECT_NEAR (fpfh_histogram[30], 23.691 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[31], 19.8475,  1e-2);
  EXPECT_NEAR (fpfh_histogram[32], 10.3655,  1e-2);

  // Object
  PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());

  // set parameters
  fpfh.setInputCloud (cloud.makeShared ());
  fpfh.setNrSubdivisions (11, 11, 11);
  fpfh.setIndices (indicesptr);
  fpfh.setSearchMethod (tree);
  fpfh.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  fpfh.compute (*fpfhs);
  EXPECT_EQ (fpfhs->points.size (), indices.size ());

  EXPECT_NEAR (fpfhs->points[0].histogram[0],  1.58591, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[1],  1.68365, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[2],  6.71   , 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[3],  23.0717, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[4],  33.3844, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[5],  20.4002, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[6],  7.31067, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[7],  1.02635, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[8],  0.48591, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[9],  1.47069, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[10], 2.87061, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[11], 1.78321, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[12], 4.30795, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[13], 7.05514, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[14], 9.37615, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[15], 17.963 , 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[16], 18.2801, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[17], 14.2766, 1e-2);
  //EXPECT_NEAR (fpfhs->points[0].histogram[18], 10.8542, 1e-2);
  //EXPECT_NEAR (fpfhs->points[0].histogram[19], 6.07925, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[20], 5.28565, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[21], 4.73887, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[22], 0.56984, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[23], 3.29826, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[24], 5.28156, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[25], 5.26939, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[26], 3.13191, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[27], 1.74453, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[28], 9.41971, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[29], 21.5894, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[30], 24.6302, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[31], 17.7764, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[32], 7.28878, 1e-2);

  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testIndicesAndSearchSurface<FPFHEstimation<PointXYZ, Normal, FPFHSignature33>, PointXYZ, Normal, FPFHSignature33>
  (cloud.makeShared (), normals, test_indices, 33);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPFHEstimationOpenMP)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh (4); // instantiate 4 threads
  fpfh.setInputNormals (normals);

  // Object
  PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());

  // set parameters
  fpfh.setInputCloud (cloud.makeShared ());
  fpfh.setNrSubdivisions (11, 11, 11);
  fpfh.setIndices (indicesptr);
  fpfh.setSearchMethod (tree);
  fpfh.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  fpfh.compute (*fpfhs);
  EXPECT_EQ (fpfhs->points.size (), indices.size ());

  EXPECT_NEAR (fpfhs->points[0].histogram[0],  1.58591, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[1],  1.68365, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[2],  6.71   , 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[3],  23.073, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[4],  33.3828, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[5],  20.4002, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[6],  7.31067, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[7],  1.02635, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[8],  0.48591, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[9],  1.47069, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[10], 2.87061, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[11], 1.78321, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[12], 4.30795, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[13], 7.05514, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[14], 9.37615, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[15], 17.963 , 1e-3);
  //EXPECT_NEAR (fpfhs->points[0].histogram[16], 18.2801, 1e-3);
  //EXPECT_NEAR (fpfhs->points[0].histogram[17], 14.2766, 1e-3);
  //EXPECT_NEAR (fpfhs->points[0].histogram[18], 10.8542, 1e-3);
  //EXPECT_NEAR (fpfhs->points[0].histogram[19], 6.07925, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[20], 5.28991, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[21], 4.73438, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[22], 0.56984, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[23], 3.29826, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[24], 5.28156, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[25], 5.26939, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[26], 3.13191, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[27], 1.74453, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[28], 9.41971, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[29], 21.5894, 1e-2);
  EXPECT_NEAR (fpfhs->points[0].histogram[30], 24.6302, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[31], 17.7764, 1e-3);
  EXPECT_NEAR (fpfhs->points[0].histogram[32], 7.28878, 1e-3);

  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testIndicesAndSearchSurface<FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33>, PointXYZ, Normal, FPFHSignature33>
  (cloud.makeShared (), normals, test_indices, 33);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, VFHEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  VFHEstimation<PointXYZ, Normal, VFHSignature308> vfh;
  vfh.setInputNormals (normals);

  //  PointCloud<PointNormal> cloud_normals;
  //  concatenateFields (cloud, normals, cloud_normals);
  //  savePCDFile ("bun0_n.pcd", cloud_normals);

  // Object
  PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

  // set parameters
  vfh.setInputCloud (cloud.makeShared ());
  vfh.setIndices (indicesptr);
  vfh.setSearchMethod (tree);

  // estimate
  vfh.compute (*vfhs);
  EXPECT_EQ (int (vfhs->points.size ()), 1);

  //for (size_t d = 0; d < 308; ++d)
  //  std::cerr << vfhs.points[0].histogram[d] << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GFPFH)
{
  PointCloud<PointXYZL>::Ptr cloud (new PointCloud<PointXYZL>());

  const unsigned num_classes = 3;

  // Build a cubic shape with a hole and changing labels.
  for (int z = -10; z < 10; ++z)
    for (int y = -10; y < 10; ++y)
      for (int x = -10; x < 10; ++x)
      {
        if (x >= -9 && x < 9 && y >= -9 && y < 9 && z >= -9 && z < 9)
          continue;
        unsigned label = 1 + (std::abs (x+y+z) % num_classes);
        PointXYZL p;
        p.label = label;
        p.x = static_cast<float> (x);
        p.y = static_cast<float> (y);
        p.z = static_cast<float> (z);
        cloud->points.push_back (p);
      }
  cloud->width = static_cast<uint32_t> (cloud->points.size ());
  cloud->height = 1;

  GFPFHEstimation<PointXYZL, PointXYZL, GFPFHSignature16> gfpfh;
  gfpfh.setNumberOfClasses (num_classes);
  gfpfh.setOctreeLeafSize (2);
  gfpfh.setInputCloud (cloud);
  gfpfh.setInputLabels (cloud);
  PointCloud<GFPFHSignature16> descriptor;
  gfpfh.compute (descriptor);

  const float ref_values[] = { 3216, 7760, 8740, 26584, 4645, 2995, 3029, 4349, 6192, 5440, 9514, 47563, 21814, 22073, 5734, 1253 };

  EXPECT_EQ (descriptor.points.size (), 1);
  for (size_t i = 0; i < size_t (descriptor.points[0].descriptorSize ()); ++i)
  {
    EXPECT_EQ (descriptor.points[0].histogram[i], ref_values[i]);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

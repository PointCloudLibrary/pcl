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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include "pcl/features/shot_lrf.h"
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

///////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
shotCopyPointCloud (const PointCloud<PointT> &cloud_in, const std::vector<int> &indices,
                    PointCloud<PointT> &cloud_out)
{
  pcl::copyPointCloud<PointT>(cloud_in, indices, cloud_out);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
checkDesc(const pcl::PointCloud<PointT>& d0, const pcl::PointCloud<PointT>& d1)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < d0.points[i].descriptor.size(); ++j)
      ASSERT_EQ (d0.points[i].descriptor[j], d1.points[i].descriptor[j]);
}

///////////////////////////////////////////////////////////////////////////////////
template <> void
checkDesc<SHOT352>(const pcl::PointCloud<SHOT352>& d0, const pcl::PointCloud<SHOT352>& d1)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < 352; ++j)
      ASSERT_EQ (d0.points[i].descriptor[j], d1.points[i].descriptor[j]);
}

///////////////////////////////////////////////////////////////////////////////////
template <> void
checkDesc<SHOT1344>(const pcl::PointCloud<SHOT1344>& d0, const pcl::PointCloud<SHOT1344>& d1)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < 1344; ++j)
      ASSERT_EQ (d0.points[i].descriptor[j], d1.points[i].descriptor[j]);
}

///////////////////////////////////////////////////////////////////////////////////
template <> void
checkDesc<ShapeContext1980>(const pcl::PointCloud<ShapeContext1980>& d0, const pcl::PointCloud<ShapeContext1980>& d1)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < 1980; ++j)
      ASSERT_EQ (d0.points[i].descriptor[j], d1.points[i].descriptor[j]);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT>
struct createSHOTDesc
{
  FeatureEstimation operator ()(const typename PointCloud<NormalT>::Ptr & normals,
                                const int nr_shape_bins_ = 10,
                                const int = 30,
                                const bool = true,
                                const bool = false) const
  {
    FeatureEstimation f(nr_shape_bins_);
    f.setInputNormals (normals);
    return (f);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT>
struct createSHOTDesc<FeatureEstimation, PointT, NormalT, SHOT352>
{
  FeatureEstimation operator ()(const typename PointCloud<NormalT>::Ptr & normals,
                                const int = 10,
                                const int = 30,
                                const bool = true,
                                const bool = false) const
  {
    FeatureEstimation f;
    f.setInputNormals (normals);
    return (f);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT>
struct createSHOTDesc<FeatureEstimation, PointT, NormalT, SHOT1344>
{
  FeatureEstimation operator ()(const typename PointCloud<NormalT>::Ptr & normals,
                                const int = 10,
                                const int = 30,
                                const bool describe_shape = true,
                                const bool describe_color = false) const
  {
    FeatureEstimation f (describe_shape, describe_color);
    f.setInputNormals (normals);
    return (f);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT, typename OutputT>
struct createSHOTDesc<ShapeContext3DEstimation<PointT, NormalT, OutputT>, PointT, NormalT, OutputT>
{
  ShapeContext3DEstimation<PointT, NormalT, OutputT>
    operator ()(const typename PointCloud<NormalT>::Ptr & normals,
                const int,
                const int,
                const bool,
                const bool) const
  {
    ShapeContext3DEstimation<PointT, NormalT, OutputT> sc3d;
    //sc3d.setAzimuthBins (4);
    //sc3d.setElevationBins (4);
    //sc3d.setRadiusBins (4);
    sc3d.setMinimalRadius (0.004);
    sc3d.setPointDensityRadius (0.008);
    sc3d.setInputNormals (normals);
    return (sc3d);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT, typename OutputT>
struct createSHOTDesc<UniqueShapeContext<PointT, OutputT>, PointT, NormalT, OutputT>
{
  UniqueShapeContext<PointT, OutputT>
    operator ()(const typename PointCloud<NormalT>::Ptr &,
                const int,
                const int,
                const bool,
                const bool) const
  {
    UniqueShapeContext<PointT, OutputT> usc;
    //usc.setAzimuthBins (4);
    //usc.setElevationBins (4);
    //usc.setRadiusBins (4);
    usc.setMinimalRadius (0.004);
    usc.setPointDensityRadius (0.008);
    usc.setLocalRadius (0.04);
    return (usc);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> void
testSHOTIndicesAndSearchSurface (const typename PointCloud<PointT>::Ptr & points,
                                 const typename PointCloud<NormalT>::Ptr & normals,
                                 const boost::shared_ptr<vector<int> > & indices,
                                 const int nr_shape_bins = 10,
                                 const int nr_color_bins = 30,
                                 const bool describe_shape = true,
                                 const bool describe_color = false)
{
  double radius = 0.04;
  //
  // Test setIndices and setSearchSurface
  //
  PointCloud<OutputT> full_output, output0, output1, output2;

  // Compute for all points and then subsample the results
  FeatureEstimation est0 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est0.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est0.setRadiusSearch (radius);
  est0.setInputCloud (points);
  est0.compute (full_output);

  shotCopyPointCloud<OutputT> (full_output, *indices, output0);

  // Compute with all points as "search surface" and the specified sub-cloud as "input"
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT>);
  copyPointCloud (*points, *indices, *subpoints);
  FeatureEstimation est1 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est1.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est1.setRadiusSearch (radius);
  est1.setInputCloud (subpoints);
  est1.setSearchSurface (points);
  est1.compute (output1);

  //// Compute with all points as "input" and the specified indices
  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est2.setRadiusSearch (radius);
  est2.setInputCloud (points);
  est2.setIndices (indices);
  est2.compute (output2);

  // All three of the above cases should produce equivalent results
  checkDesc<OutputT> (output0, output1);
  checkDesc<OutputT> (output1, output2);

  //
  // Test the combination of setIndices and setSearchSurface
  //
  PointCloud<OutputT> output3, output4;

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (static_cast<int> (i));

  // Compute with all points as search surface + the specified sub-cloud as "input" but for only a subset of indices
  FeatureEstimation est3 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est3.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est3.setRadiusSearch (radius);
  est3.setSearchSurface (points);
  est3.setInputCloud (subpoints);
  est3.setIndices (indices2);
  est3.compute (output3);

  // Start with features for each point in "subpoints" and then subsample the results
  shotCopyPointCloud<OutputT> (output0, *indices2, output4); // (Re-using "output0" from above)

  // The two cases above should produce equivalent results
  checkDesc<OutputT> (output3, output4);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> void
testSHOTLocalReferenceFrame (const typename PointCloud<PointT>::Ptr & points,
                             const typename PointCloud<NormalT>::Ptr & normals,
                             const boost::shared_ptr<vector<int> > & indices,
                             const int nr_shape_bins = 10,
                             const int nr_color_bins = 30,
                             const bool describe_shape = true,
                             const bool describe_color = false)
{
  double radius = 0.04;

  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT> ());
  copyPointCloud (*points, *indices, *subpoints);

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (static_cast<int> (i));
  //
  // Test an external computation for the local reference frames
  //
  PointCloud<ReferenceFrame>::Ptr frames (new PointCloud<ReferenceFrame> ());
  SHOTLocalReferenceFrameEstimation<PointT, pcl::ReferenceFrame> lrf_estimator;
  lrf_estimator.setRadiusSearch (radius);
  lrf_estimator.setInputCloud (subpoints);
  lrf_estimator.setIndices (indices2);
  lrf_estimator.setSearchSurface(points);
  lrf_estimator.compute (*frames);

  PointCloud<OutputT> output, output2;

  FeatureEstimation est = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est.setRadiusSearch (radius);
  est.setSearchSurface (points);
  est.setInputCloud (subpoints);
  est.setIndices (indices2);
  est.compute (output);

  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est2.setRadiusSearch (radius);
  est2.setSearchSurface (points);
  est2.setInputCloud (subpoints);
  est2.setIndices (indices2);
  est2.setInputReferenceFrames (frames);
  est2.compute (output2);

  // Check frames
  pcl::PointCloud<pcl::ReferenceFrame>::ConstPtr f = est.getInputReferenceFrames ();
  pcl::PointCloud<pcl::ReferenceFrame>::ConstPtr f2 = est2.getInputReferenceFrames ();
  ASSERT_EQ (frames->points.size (), f->points.size ());
  ASSERT_EQ (f2->points.size (), f->points.size ());
  for (int i = 0; i < static_cast<int> (frames->points.size ()); ++i)
  {
    for (unsigned j = 0; j < 9; ++j)
      ASSERT_EQ (frames->points[i].rf[j], f->points[i].rf[j]);

    for (unsigned j = 0; j < 9; ++j)
      ASSERT_EQ (frames->points[i].rf[j], f2->points[i].rf[j]);
  }

  // The two cases above should produce equivalent results
  checkDesc<OutputT> (output, output2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTShapeEstimation)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

/*
  SHOTEstimation<PointXYZ, Normal, SHOT> shot;
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);
  shot.setRadiusSearch (20 * mr);

  // Object
  PointCloud<SHOT>::Ptr shots (new PointCloud<SHOT> ());

  // set parameters
  shot.setInputCloud (cloud.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (tree);

  // estimate
  shot.compute (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[9 ], 0.0072018504, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[10], 0.0023103887, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[11], 0.0024724449, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[19], 0.0031367359, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[20], 0.17439659, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.070665278, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[42], 0.013304681, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[53], 0.0073520984, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[54], 0.013584172, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[55], 0.0050609680, 1e-4);
*/

  // SHOT352
  SHOTEstimation<PointXYZ, Normal, SHOT352> shot352;
  shot352.setInputNormals (normals);
  EXPECT_EQ (shot352.getInputNormals (), normals);
  shot352.setRadiusSearch (20 * mr);

  // Object
  PointCloud<SHOT352>::Ptr shots352 (new PointCloud<SHOT352> ());

  // set parameters
  shot352.setInputCloud (cloud.makeShared ());
  shot352.setIndices (indicesptr);
  shot352.setSearchMethod (tree);

  // estimate
  shot352.compute (*shots352);
  EXPECT_EQ (shots352->points.size (), indices.size ());

  EXPECT_NEAR (shots352->points[103].descriptor[9 ], 0.0072018504, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[10], 0.0023103887, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[11], 0.0024724449, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[19], 0.0031367359, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[20], 0.17439659, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[21], 0.070665278, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[42], 0.013304681, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[53], 0.0073520984, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[54], 0.013584172, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[55], 0.0050609680, 1e-4);


  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  //testSHOTIndicesAndSearchSurface<SHOTEstimation<PointXYZ, Normal, SHOT>, PointXYZ, Normal, SHOT> (cloud.makeShared (), normals, test_indices);
  //testSHOTLocalReferenceFrame<SHOTEstimation<PointXYZ, Normal, SHOT>, PointXYZ, Normal, SHOT> (cloud.makeShared (), normals, test_indices);

  testSHOTIndicesAndSearchSurface<SHOTEstimation<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
  testSHOTLocalReferenceFrame<SHOTEstimation<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TEST (PCL, GenericSHOTShapeEstimation)
{
  // SHOT length
  const int shapeStep_ = 20;
  //const int dim = 32*(shapeStep_+1);

  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  SHOTEstimation<PointXYZ, Normal, SHOT> shot (shapeStep_);
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch (20 * mr);

  PointCloud< SHOT >::Ptr shots (new PointCloud< SHOT > ());

  // set parameters
  shot.setInputCloud (cloud.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (tree);

  // estimate
  shot.compute (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[18], 0.0077019366, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[19], 0.0024708188, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.0079652183, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[38], 0.0067090928, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[39], 0.17498907, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[40], 0.078413926, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[81], 0.014228539, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[103], 0.022390056, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[105], 0.0058866320, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[123], 0.019105887, 1e-5);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testSHOTIndicesAndSearchSurface<SHOTEstimation<PointXYZ, Normal, SHOT>, PointXYZ, Normal, SHOT> (cloud.makeShared (), normals, test_indices, shapeStep_);
  testSHOTLocalReferenceFrame<SHOTEstimation<PointXYZ, Normal, SHOT>, PointXYZ, Normal, SHOT> (cloud.makeShared (), normals, test_indices, shapeStep_);
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTShapeAndColorEstimation)
{
  double mr = 0.002;
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  search::KdTree<PointXYZRGBA>::Ptr rgbaTree;
  rgbaTree.reset (new search::KdTree<PointXYZRGBA> (false));

  // Create fake point cloud with colors
  PointCloud<PointXYZRGBA> cloudWithColors;
  for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
  {
    PointXYZRGBA p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;

    p.rgba = ( (i%255) << 16 ) + ( ( (255 - i ) %255) << 8) + ( ( i*37 ) %255);
    cloudWithColors.push_back(p);
  }

/*
  // Object
  SHOTEstimation<PointXYZRGBA, Normal, SHOT> shot (true, true);
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch ( 20 * mr);

  rgbaTree->setInputCloud (cloudWithColors.makeShared ());
  PointCloud<SHOT>::Ptr shots (new PointCloud<SHOT>);

  shot.setInputCloud (cloudWithColors.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (rgbaTree);

  // estimate
  shot.compute (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[10], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[11], 0.0021887729, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.062557608, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[42], 0.011778189, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[53], 0.0065085669, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[54], 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[55], 0.0044803056, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[64], 0.064429596, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[65], 0.046486385, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[86], 0.011518310, 1e-5);

  EXPECT_NEAR (shots->points[103].descriptor[357], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[360], 0.0027993850, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[386], 0.045115642, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[387], 0.059068538, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[389], 0.0047547864, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[453], 0.0051176427, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[481], 0.0053625242, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[482], 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[511], 0.0057367259, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[512], 0.048357654, 1e-5);
*/

  // SHOT1344
  SHOTColorEstimation<PointXYZRGBA, Normal, SHOT1344> shot1344 (true, true);
  shot1344.setInputNormals (normals);
  EXPECT_EQ (shot1344.getInputNormals (), normals);

  shot1344.setRadiusSearch ( 20 * mr);

  PointCloud<SHOT1344>::Ptr shots1344 (new PointCloud<SHOT1344>);

  shot1344.setInputCloud (cloudWithColors.makeShared ());
  shot1344.setIndices (indicesptr);
  shot1344.setSearchMethod (rgbaTree);

  // estimate
  shot1344.compute (*shots1344);
  EXPECT_EQ (shots1344->points.size (), indices.size ());

  EXPECT_NEAR (shots1344->points[103].descriptor[10], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[11], 0.0021887729, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[21], 0.062557608, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[42], 0.011778189, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[53], 0.0065085669, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[54], 0.012025614, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[55], 0.0044803056, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[64], 0.064429596, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[65], 0.046486385, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[86], 0.011518310, 1e-5);

  EXPECT_NEAR (shots1344->points[103].descriptor[357], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[360], 0.0027993850, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[386], 0.045115642, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[387], 0.059068538, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[389], 0.0047547864, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[453], 0.0051176427, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[481], 0.0053625242, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[482], 0.012025614, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[511], 0.0057367259, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[512], 0.048357654, 1e-5);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  //testSHOTIndicesAndSearchSurface<SHOTEstimation<PointXYZRGBA, Normal, SHOT>, PointXYZRGBA, Normal, SHOT> (cloudWithColors.makeShared (), normals, test_indices);
  //testSHOTLocalReferenceFrame<SHOTEstimation<PointXYZRGBA, Normal, SHOT>, PointXYZRGBA, Normal, SHOT> (cloudWithColors.makeShared (), normals, test_indices);

  testSHOTIndicesAndSearchSurface<SHOTColorEstimation<PointXYZRGBA, Normal, SHOT1344>, PointXYZRGBA, Normal, SHOT1344> (cloudWithColors.makeShared (), normals, test_indices);
  testSHOTLocalReferenceFrame<SHOTColorEstimation<PointXYZRGBA, Normal, SHOT1344>, PointXYZRGBA, Normal, SHOT1344> (cloudWithColors.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTShapeEstimationOpenMP)
{
  // Estimate normals first
  double mr = 0.002;
#ifdef _OPENMP
  NormalEstimationOMP<PointXYZ, Normal> n (omp_get_max_threads ());
#else
  NormalEstimationOMP<PointXYZ, Normal> n;
#endif
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

/*
  SHOTEstimationOMP<PointXYZ, Normal, SHOT> shot;
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch ( 20 * mr);

  // Object
  PointCloud<SHOT>::Ptr shots (new PointCloud<SHOT>);

  // set parameters
  shot.setInputCloud (cloud.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (tree);

  // estimate
  shot.compute (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[9 ], 0.0072018504, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[10], 0.0023103887, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[11], 0.0024724449, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[19], 0.0031367359, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[20], 0.17439659, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.070665278, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[42], 0.013304681, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[53], 0.0073520984, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[54], 0.013584172, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[55], 0.0050609680, 1e-4);
*/

  // SHOT352
  SHOTEstimationOMP<PointXYZ, Normal, SHOT352> shot352;
  shot352.setInputNormals (normals);
  EXPECT_EQ (shot352.getInputNormals (), normals);

  shot352.setRadiusSearch ( 20 * mr);

  // Object
  PointCloud<SHOT352>::Ptr shots352 (new PointCloud<SHOT352>);

  // set parameters
  shot352.setInputCloud (cloud.makeShared ());
  shot352.setIndices (indicesptr);
  shot352.setSearchMethod (tree);

  // estimate
  shot352.compute (*shots352);
  EXPECT_EQ (shots352->points.size (), indices.size ());

  EXPECT_NEAR (shots352->points[103].descriptor[9 ], 0.0072018504, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[10], 0.0023103887, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[11], 0.0024724449, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[19], 0.0031367359, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[20], 0.17439659, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[21], 0.070665278, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[42], 0.013304681, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[53], 0.0073520984, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[54], 0.013584172, 1e-4);
  EXPECT_NEAR (shots352->points[103].descriptor[55], 0.0050609680, 1e-4);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  //testSHOTIndicesAndSearchSurface<SHOTEstimationOMP<PointXYZ, Normal, SHOT>, PointXYZ, Normal, SHOT> (cloud.makeShared (), normals, test_indices);
  //testSHOTLocalReferenceFrame<SHOTEstimationOMP<PointXYZ, Normal, SHOT>, PointXYZ, Normal, SHOT> (cloud.makeShared (), normals, test_indices);

  testSHOTIndicesAndSearchSurface<SHOTEstimationOMP<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
  testSHOTLocalReferenceFrame<SHOTEstimationOMP<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL,SHOTShapeAndColorEstimationOpenMP)
{
  double mr = 0.002;
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  search::KdTree<PointXYZRGBA>::Ptr rgbaTree;

  rgbaTree.reset (new search::KdTree<PointXYZRGBA> (false));

  // Create fake point cloud with colors
  PointCloud<PointXYZRGBA> cloudWithColors;
  for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
  {
    PointXYZRGBA p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;

    p.rgba = ( (i%255) << 16 ) + ( ( (255 - i ) %255) << 8) + ( ( i*37 ) %255);
    cloudWithColors.push_back(p);
  }

/*
  // Object
  SHOTEstimationOMP<PointXYZRGBA, Normal, SHOT> shot (true, true, -1);
  shot.setInputNormals (normals);

  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch ( 20 * mr);

  rgbaTree->setInputCloud (cloudWithColors.makeShared ());

  PointCloud<SHOT>::Ptr shots (new PointCloud<SHOT> ());

  shot.setInputCloud (cloudWithColors.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (rgbaTree);

  // estimate
  shot.compute (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[10], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[11], 0.0021887729, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.062557608, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[42], 0.011778189, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[53], 0.0065085669, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[54], 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[55], 0.0044803056, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[64], 0.064429596, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[65], 0.046486385, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[86], 0.011518310, 1e-5);

  EXPECT_NEAR (shots->points[103].descriptor[357], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[360], 0.0027993850, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[386], 0.045115642, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[387], 0.059068538, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[389], 0.0047547864, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[453], 0.0051176427, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[481], 0.0053625242, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[482], 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[511], 0.0057367259, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[512], 0.048357654, 1e-5);
*/

  // SHOT1344
  SHOTColorEstimationOMP<PointXYZRGBA, Normal, SHOT1344> shot1344 (true, true);
  shot1344.setInputNormals (normals);

  EXPECT_EQ (shot1344.getInputNormals (), normals);

  shot1344.setRadiusSearch ( 20 * mr);

  PointCloud<SHOT1344>::Ptr shots1344 (new PointCloud<SHOT1344> ());

  shot1344.setInputCloud (cloudWithColors.makeShared ());
  shot1344.setIndices (indicesptr);
  shot1344.setSearchMethod (rgbaTree);

  // estimate
  shot1344.compute (*shots1344);
  EXPECT_EQ (shots1344->points.size (), indices.size ());

  EXPECT_NEAR (shots1344->points[103].descriptor[10], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[11], 0.0021887729, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[21], 0.062557608, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[42], 0.011778189, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[53], 0.0065085669, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[54], 0.012025614, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[55], 0.0044803056, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[64], 0.064429596, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[65], 0.046486385, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[86], 0.011518310, 1e-5);

  EXPECT_NEAR (shots1344->points[103].descriptor[357], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[360], 0.0027993850, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[386], 0.045115642, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[387], 0.059068538, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[389], 0.0047547864, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[453], 0.0051176427, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[481], 0.0053625242, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[482], 0.012025614, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[511], 0.0057367259, 1e-5);
  EXPECT_NEAR (shots1344->points[103].descriptor[512], 0.048357654, 1e-5);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  //testSHOTIndicesAndSearchSurface<SHOTEstimationOMP<PointXYZRGBA, Normal, SHOT>, PointXYZRGBA, Normal, SHOT> (cloudWithColors.makeShared (), normals, test_indices);
  //testSHOTLocalReferenceFrame<SHOTEstimationOMP<PointXYZRGBA, Normal, SHOT>, PointXYZRGBA, Normal, SHOT> (cloudWithColors.makeShared (), normals, test_indices);

  testSHOTIndicesAndSearchSurface<SHOTColorEstimationOMP<PointXYZRGBA, Normal, SHOT1344>, PointXYZRGBA, Normal, SHOT1344> (cloudWithColors.makeShared (), normals, test_indices);
  testSHOTLocalReferenceFrame<SHOTColorEstimationOMP<PointXYZRGBA, Normal, SHOT1344>, PointXYZRGBA, Normal, SHOT1344> (cloudWithColors.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL,3DSCEstimation)
{
  float meshRes = 0.002f;
  //size_t nBinsL = 4;
  //size_t nBinsK = 4;
  //size_t nBinsJ = 4;
  float radius = 20.0f * meshRes;
  float rmin = radius / 10.0f;
  float ptDensityRad = radius / 5.0f;

  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> ne;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  ne.setInputCloud (cloudptr);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (radius);
  // estimate
  ne.compute (*normals);

  ShapeContext3DEstimation<PointXYZ, Normal, ShapeContext1980> sc3d;
  sc3d.setInputCloud (cloudptr);
  sc3d.setInputNormals (normals);
  sc3d.setSearchMethod (tree);
  sc3d.setRadiusSearch (radius);
  //sc3d.setAzimuthBins (nBinsL);
  //sc3d.setElevationBins (nBinsK);
  //sc3d.setRadiusBins (nBinsJ);
  sc3d.setMinimalRadius (rmin);
  sc3d.setPointDensityRadius (ptDensityRad);
  // Compute the features
  PointCloud<ShapeContext1980>::Ptr sc3ds (new PointCloud<ShapeContext1980> ());
  sc3d.compute (*sc3ds);
  EXPECT_EQ (sc3ds->size (), cloud.size ());

  // 3DSC does not define a repeatable local RF, we set it to zero to signal it to the user
  //EXPECT_NEAR ((*sc3ds)[0].rf[0], 0.2902f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[1], 0.7334f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[2], -0.6146f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[3], 0.9486f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[4], -0.3051f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[5], 0.0838f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[6], -0.1261f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[7], -0.6074f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[8], -0.7843f, 1e-4f);

  EXPECT_NEAR ((*sc3ds)[0].rf[0], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[1], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[2], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[3], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[4], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[5], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[6], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[7], 0.0f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[0].rf[8], 0.0f, 1e-4f);

  //EXPECT_EQ ((*sc3ds)[0].descriptor.size (), 64);

  EXPECT_NEAR ((*sc3ds)[94].descriptor[88], 55.2712f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[94].descriptor[584], 71.1088f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[94].descriptor[1106], 79.5896f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[94].descriptor[1560], 0.f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[94].descriptor[1929], 36.0636f, 1e-4f);

  EXPECT_NEAR ((*sc3ds)[108].descriptor[67], 0.f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[108].descriptor[548], 126.141f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[108].descriptor[1091], 30.4704f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[108].descriptor[1421], 38.088f, 1e-4f);
  EXPECT_NEAR ((*sc3ds)[108].descriptor[1900], 43.7994f, 1e-4f);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i++)
    test_indices->push_back (static_cast<int> (i));

  testSHOTIndicesAndSearchSurface<ShapeContext3DEstimation<PointXYZ, Normal, ShapeContext1980>, PointXYZ, Normal, ShapeContext1980> (cloudptr, normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, USCEstimation)
{
  float meshRes = 0.002f;
  //size_t nBinsL = 4;
  //size_t nBinsK = 4;
  //size_t nBinsJ = 4;
  float radius = 20.0f * meshRes;
  float rmin = radius / 10.0f;
  float ptDensityRad = radius / 5.0f;

  // estimate
  UniqueShapeContext<PointXYZ, ShapeContext1980> uscd;
  uscd.setInputCloud (cloud.makeShared ());
  uscd.setSearchMethod (tree);
  uscd.setRadiusSearch (radius);
  //uscd.setAzimuthBins (nBinsL);
  //uscd.setElevationBins (nBinsK);
  //uscd.setRadiusBins (nBinsJ);
  uscd.setMinimalRadius (rmin);
  uscd.setPointDensityRadius (ptDensityRad);
  uscd.setLocalRadius (radius);
  // Compute the features
  PointCloud<ShapeContext1980>::Ptr uscds (new PointCloud<ShapeContext1980>);
  uscd.compute (*uscds);
  EXPECT_EQ (uscds->size (), cloud.size ());

  EXPECT_NEAR ((*uscds)[160].rf[0], -0.97767f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[1], 0.0353674f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[2], -0.20715f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[3], 0.0125394f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[4], 0.993798f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[5], 0.110493f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[6], 0.209773f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[7], 0.105428f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].rf[8], -0.972049f, 1e-4f);

  //EXPECT_EQ ((*uscds)[0].descriptor.size (), 64);

  EXPECT_NEAR ((*uscds)[160].descriptor[56], 53.0597f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].descriptor[734], 80.1063f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].descriptor[1222], 93.8412f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].descriptor[1605], 0.f, 1e-4f);
  EXPECT_NEAR ((*uscds)[160].descriptor[1887], 32.6679f, 1e-4f);

  EXPECT_NEAR ((*uscds)[168].descriptor[72], 65.3358f, 1e-4f);
  EXPECT_NEAR ((*uscds)[168].descriptor[430], 88.8147f, 1e-4f);
  EXPECT_NEAR ((*uscds)[168].descriptor[987], 0.f, 1e-4f);
  EXPECT_NEAR ((*uscds)[168].descriptor[1563], 128.273f, 1e-4f);
  EXPECT_NEAR ((*uscds)[168].descriptor[1915], 59.2098f, 1e-4f);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  testSHOTIndicesAndSearchSurface<UniqueShapeContext<PointXYZ, ShapeContext1980>, PointXYZ, Normal, ShapeContext1980> (cloud.makeShared (), normals, test_indices);
  testSHOTLocalReferenceFrame<UniqueShapeContext<PointXYZ, ShapeContext1980>, PointXYZ, Normal, ShapeContext1980> (cloud.makeShared (), normals, test_indices);
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

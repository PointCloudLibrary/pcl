/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: test_ii_normals.cpp 4084 2012-01-31 02:05:42Z rusu $
 *
 */

#include <gtest/gtest.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/io/pcd_io.h>
#include "boost.h"

#include <pcl/common/time.h>

using namespace pcl;
using namespace std;

/** \brief if set to value other than 0 -> fine grained output */
#define DEBUG_OUT 1
#define EXCESSIVE_TESTING 0

#define TEST_unorganized_dense_cloud_COMPLETE_KNN     1
#define TEST_unorganized_dense_cloud_VIEW_KNN         1
#define TEST_unorganized_sparse_cloud_COMPLETE_KNN    1
#define TEST_unorganized_sparse_cloud_VIEW_KNN        1
#define TEST_unorganized_grid_cloud_COMPLETE_RADIUS   1
#define TEST_unorganized_dense_cloud_COMPLETE_RADIUS  1
#define TEST_unorganized_dense_cloud_VIEW_RADIUS      1
#define TEST_unorganized_sparse_cloud_COMPLETE_RADIUS 1
#define TEST_unorganized_sparse_cloud_VIEW_RADIUS     1
#define TEST_ORGANIZED_SPARSE_COMPLETE_KNN            1
#define TEST_ORGANIZED_SPARSE_VIEW_KNN                1
#define TEST_ORGANIZED_SPARSE_COMPLETE_RADIUS         1
#define TEST_ORGANIZED_SPARSE_VIEW_RADIUS             1

#if EXCESSIVE_TESTING
/** \brief number of points used for creating unordered point clouds */
const unsigned int unorganized_point_count = 100000;

/** \brief number of search operations on ordered point clouds*/
const unsigned int query_count = 5000;
#else
/** \brief number of points used for creating unordered point clouds */
const unsigned int unorganized_point_count = 1200;

/** \brief number of search operations on ordered point clouds*/
const unsigned int query_count = 100;
#endif

/** \brief organized point cloud*/
PointCloud<PointXYZ>::Ptr organized_sparse_cloud (new PointCloud<PointXYZ>);

/** \brief unorganized point cloud*/
PointCloud<PointXYZ>::Ptr unorganized_dense_cloud (new PointCloud<PointXYZ>);

/** \brief unorganized point cloud*/
PointCloud<PointXYZ>::Ptr unorganized_sparse_cloud (new PointCloud<PointXYZ>);

/** \brief unorganized point cloud*/
PointCloud<PointXYZ>::Ptr unorganized_grid_cloud (new PointCloud<PointXYZ>);

/** \brief uniform distributed random number generator for unsigned it in range [0;10]*/
boost::variate_generator< boost::mt19937, boost::uniform_int<unsigned> > rand_uint(boost::mt19937 (), boost::uniform_int<unsigned> (0, 10));
/** \brief uniform distributed random number generator for floats in the range [0;1] */
boost::variate_generator< boost::mt19937, boost::uniform_real<float> > rand_float(boost::mt19937 (), boost::uniform_real<float> (0, 1));

/** \brief used by the *_VIEW_* tests to use only a subset of points from the point cloud*/
std::vector<int> unorganized_input_indices;

/** \brief used by the *_VIEW_* tests to use only a subset of points from the point cloud*/
std::vector<int> organized_input_indices;

/** \brief instance of brute force search method to be tested*/
pcl::search::BruteForce<pcl::PointXYZ> brute_force;

/** \brief instance of KDTree search method to be tested*/
pcl::search::KdTree<pcl::PointXYZ> KDTree;

/** \brief instance of Octree search method to be tested*/
pcl::search::Octree<pcl::PointXYZ> octree_search (0.1);

/** \brief instance of Organized search method to be tested*/
pcl::search::OrganizedNeighbor<pcl::PointXYZ> organized;

/** \brief list of search methods for unorganized search test*/
vector<search::Search<PointXYZ>* > unorganized_search_methods;

/** \brief list of search methods for organized search test*/
vector<search::Search<PointXYZ>* > organized_search_methods;

/** \brief lists of indices to be used as query points for various search methods and different cloud types*/
vector<int> unorganized_dense_cloud_query_indices;
vector<int> unorganized_sparse_cloud_query_indices;
vector<int> organized_sparse_query_indices;

/** \briet test whether the result of a search containes unique point ids or not
  * @param indices resulting indices from a search
  * @param name name of the search method that returned these distances
  * @return true if indices are unique, false otherwise
  */
bool testUniqueness (const vector<int>& indices, const string& name)
{
  bool uniqueness = true;
  for (unsigned idx1 = 1; idx1 < indices.size () && uniqueness; ++idx1)
  {
    // check whether resulting indices are unique
    for (unsigned idx2 = 0; idx2 < idx1; ++idx2)
    {
      if (indices [idx1] == indices [idx2])
      {
#if DEBUG_OUT
        std::cout << name << " search: index is twice at positions: " << idx1 << " (" << indices [idx1] << ") , " << idx2  << " (" << indices [idx2] << ")" << std::endl;
#endif
        // can only be set to false anyway -> no sync required
        uniqueness = false;
        break;
      }
    }
  }
  return uniqueness;
}


/** \brief tests whether the ordering of results is ascending on distances
  * \param distances resulting distances from a search
  * \param name name of the search method that returned these distances
  * \return true if distances in weak ascending order, false otherwise
  */
bool testOrder (const vector<float>& distances, const string& name)
{
  bool ordered = true;
  for (unsigned idx1 = 1; idx1 < distances.size (); ++idx1)
  {
    if (distances [idx1-1] > distances [idx1])
    {
#if DEBUG_OUT
      std::cout << name << " search: not properly sorted: " << idx1 - 1 << "(" << distances [idx1-1] << ") > " 
                                                            << idx1     << "(" << distances [idx1]   << ")"<< std::endl;
#endif
      ordered = false;
      break;
    }
  }
  
  return ordered;
}

/** \brief test whether the results are from the view (subset of the cloud) given by input_indices and also not Nan
 * @param indices_mask defines the subset of allowed points (view) in the result of the search
 * @param nan_mask defines a lookup that indicates whether a point at a given position is finite or not
 * @param indices result of a search to be tested
 * @param name name of search method that returned the result
 * @return true if result is valid, false otherwise
 */
template<typename PointT> bool
testResultValidity (const typename PointCloud<PointT>::ConstPtr point_cloud, const vector<bool>& indices_mask, const vector<bool>& nan_mask, const vector<int>& indices, const vector<int>& /*input_indices*/, const string& name)
{
  bool validness = true;
  for (vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
  {
    if (!indices_mask [*iIt])
    {
#if DEBUG_OUT
      cerr << name << ": result contains an invalid point: " << *iIt << " not in indices list.\n";
      
//      for (vector<int>::const_iterator iIt2 = input_indices.begin (); iIt2 != input_indices.end (); ++iIt2)
//        cout << *iIt2 << "  ";
//      cout << endl;
#endif
      validness = false;
      break;
    }
    else if (!nan_mask [*iIt])
    {
#if DEBUG_OUT
      cerr << name << ": result contains an invalid point: " << *iIt << " = NaN (" << point_cloud->points [*iIt].x << " , " 
                                                                                   << point_cloud->points [*iIt].y << " , " 
                                                                                   << point_cloud->points [*iIt].z << ")\n";
#endif
      validness = false;
      break;
    }
  }
  
  return validness;
}

/** \brief compares two sets of search results
  * \param indices1
  * \param distances1
  * \param name1
  * \param indices2
  * \param distances2
  * \param name2
  * \param eps threshold for comparing the distances
  * \return true if both sets are the same, false otherwise
  */
bool compareResults (const std::vector<int>& indices1, const::vector<float>& distances1, const std::string& name1,
                     const std::vector<int>& indices2, const::vector<float>& distances2, const std::string& name2, float eps)
{
  bool equal = true;
  if (indices1.size () != indices2.size ())
  {
#if DEBUG_OUT
    cerr << "size of results between " << name1 << " search and " << name2 << " search do not match " <<indices1.size () << " vs. " << indices2.size () << endl;
//    for (unsigned idx = 0; idx < std::min (indices1.size (), indices2.size ()); ++idx)
//    {
//      cout << idx <<".\t" << indices1[idx] << "\t(" << distances1[idx] << "),\t" << indices2[idx] << "\t(" << distances2[idx] << ")\n";
//    }    
//    for (unsigned idx = std::min (indices1.size (), indices2.size ()); idx < std::max (indices1.size (), indices2.size ()); ++idx)
//    {
//      if (idx >= indices1.size ())
//        cout << idx <<".\t     \t      ,\t" << indices2[idx] << "\t(" << distances2[idx] << ")\n";
//      else
//        cout << idx <<".\t" << indices1[idx] << "\t(" << distances1[idx] << ")\n";
//    }
#endif
    equal = false;
  }
  else
  {
    for (unsigned idx = 0; idx < indices1.size (); ++idx)
    {
      if (indices1[idx] != indices2[idx] && fabs (distances1[idx] - distances2[idx]) > eps)
      {
#if DEBUG_OUT
        cerr << "results between " << name1 << " search and " << name2 << " search do not match: " << idx << " nearest neighbor: "
                << indices1[idx] << " with distance: " << distances1[idx] << " vs. "
                << indices2[idx] << " with distance: " << distances2[idx] << endl;
#endif
        equal = false;
        break;
      }
    }
  }
  return equal;
}

/** \brief does KNN search and tests the results to be unique, valid and ordered. Additionally it test whether all test methods are returning the same results
  * \param cloud the input point cloud
  * \param search_methods vector of all search methods to be tested
  * \param query_indices indices of query points in the point cloud (not necessarily in input_indices)
  * \param input_indices indices defining a subset of the point cloud.
  */
template<typename PointT> void
testKNNSearch (typename PointCloud<PointT>::ConstPtr point_cloud, vector<search::Search<PointT>*> search_methods,
                const vector<int>& query_indices, const vector<int>& input_indices = vector<int> () )
{
  vector< vector<int> >indices (search_methods.size ());
  vector< vector<float> >distances (search_methods.size ());
  vector<bool> passed (search_methods.size (), true);
  
  vector<bool> indices_mask (point_cloud->size (), true);
  vector<bool> nan_mask (point_cloud->size (), true);
  
  if (input_indices.size () != 0)
  {
    indices_mask.assign (point_cloud->size (), false);
    for (vector<int>::const_iterator iIt = input_indices.begin (); iIt != input_indices.end (); ++iIt)
      indices_mask [*iIt] = true;
  }
  
  // remove also Nans
  #pragma omp parallel for
  for (int pIdx = 0; pIdx < int (point_cloud->size ()); ++pIdx)
  {
    if (!isFinite (point_cloud->points [pIdx]))
      nan_mask [pIdx] = false;
  }
  
  boost::shared_ptr<vector<int> > input_indices_;
  if (input_indices.size ())
    input_indices_.reset (new vector<int> (input_indices));
  
  #pragma omp parallel for
  for (int sIdx = 0; sIdx < int (search_methods.size ()); ++sIdx)
    search_methods [sIdx]->setInputCloud (point_cloud, input_indices_);

  // test knn values from 1, 8, 64, 512
  for (unsigned knn = 1; knn <= 512; knn <<= 3)
  {
    // find nn for each point in the cloud
    for (vector<int>::const_iterator qIt = query_indices.begin (); qIt != query_indices.end (); ++qIt)
    {
      #pragma omp parallel for
      for (int sIdx = 0; sIdx < int (search_methods.size ()); ++sIdx)
      {
        search_methods [sIdx]->nearestKSearch (point_cloud->points[*qIt], knn, indices [sIdx], distances [sIdx]);
        passed [sIdx] = passed [sIdx] && testUniqueness (indices [sIdx], search_methods [sIdx]->getName ());
        passed [sIdx] = passed [sIdx] && testOrder (distances [sIdx], search_methods [sIdx]->getName ());
        passed [sIdx] = passed [sIdx] && testResultValidity<PointT>(point_cloud, indices_mask, nan_mask, indices [sIdx], input_indices, search_methods [sIdx]->getName ());
      }
      
      // compare results to each other
      #pragma omp parallel for
      for (int sIdx = 1; sIdx < int (search_methods.size ()); ++sIdx)
      {
        passed [sIdx] = passed [sIdx] && compareResults (indices [0],    distances [0],    search_methods [0]->getName (),
                                                         indices [sIdx], distances [sIdx], search_methods [sIdx]->getName (), 1e-6f);
      }
    }
  }
  for (size_t sIdx = 0; sIdx < search_methods.size (); ++sIdx)
  {
    cout << search_methods [sIdx]->getName () << ": " << (passed[sIdx]?"passed":"failed") << endl;
    EXPECT_TRUE (passed [sIdx]);
  }
}

/** \brief does radius search and tests the results to be unique, valid and ordered. Additionally it test whether all test methods are returning the same results
  * \param cloud the input point cloud
  * \param search_methods vector of all search methods to be tested
  * \param query_indices indices of query points in the point cloud (not necessarily in input_indices)
  * \param input_indices indices defining a subset of the point cloud.
  */
template<typename PointT> void
testRadiusSearch (typename PointCloud<PointT>::ConstPtr point_cloud, vector<search::Search<PointT>*> search_methods, 
                   const vector<int>& query_indices, const vector<int>& input_indices = vector<int> ())
{
  vector< vector<int> >indices (search_methods.size ());
  vector< vector<float> >distances (search_methods.size ());
  vector <bool> passed (search_methods.size (), true);
  vector<bool> indices_mask (point_cloud->size (), true);
  vector<bool> nan_mask (point_cloud->size (), true);
  
  if (input_indices.size () != 0)
  {
    indices_mask.assign (point_cloud->size (), false);
    for (vector<int>::const_iterator iIt = input_indices.begin (); iIt != input_indices.end (); ++iIt)
      indices_mask [*iIt] = true;
  }
  
  // remove also Nans
  #pragma omp parallel for
  for (int pIdx = 0; pIdx < int (point_cloud->size ()); ++pIdx)
  {
    if (!isFinite (point_cloud->points [pIdx]))
      nan_mask [pIdx] = false;
  }
  
  boost::shared_ptr<vector<int> > input_indices_;
  if (input_indices.size ())
    input_indices_.reset (new vector<int> (input_indices));
  
  #pragma omp parallel for
  for (int sIdx = 0; sIdx < int (search_methods.size ()); ++sIdx)
    search_methods [sIdx]->setInputCloud (point_cloud, input_indices_);

  // test radii 0.01, 0.02, 0.04, 0.08
  for (float radius = 0.01f; radius < 0.1f; radius *= 2.0f)
  {
    //cout << radius << endl;
    // find nn for each point in the cloud
    for (vector<int>::const_iterator qIt = query_indices.begin (); qIt != query_indices.end (); ++qIt)
    {
      #pragma omp parallel for
      for (int sIdx = 0; sIdx < static_cast<int> (search_methods.size ()); ++sIdx)
      {
        search_methods [sIdx]->radiusSearch (point_cloud->points[*qIt], radius, indices [sIdx], distances [sIdx], 0);
        passed [sIdx] = passed [sIdx] && testUniqueness (indices [sIdx], search_methods [sIdx]->getName ());
        passed [sIdx] = passed [sIdx] && testOrder (distances [sIdx], search_methods [sIdx]->getName ());
        passed [sIdx] = passed [sIdx] && testResultValidity<PointT>(point_cloud, indices_mask, nan_mask, indices [sIdx], input_indices, search_methods [sIdx]->getName ());
      }
      
      // compare results to each other
      #pragma omp parallel for
      for (int sIdx = 1; sIdx < static_cast<int> (search_methods.size ()); ++sIdx)
      {
        passed [sIdx] = passed [sIdx] && compareResults (indices [0],    distances [0],    search_methods [0]->getName (),
                                                         indices [sIdx], distances [sIdx], search_methods [sIdx]->getName (), 1e-6f);
      }
    }
  }
  for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
  {
    cout << search_methods [sIdx]->getName () << ": " << (passed[sIdx]?"passed":"failed") << endl;
    EXPECT_TRUE (passed [sIdx]);
  }
}

#if TEST_unorganized_dense_cloud_COMPLETE_KNN
// Test search on unorganized point clouds
TEST (PCL, unorganized_dense_cloud_Complete_KNN)
{
  testKNNSearch (unorganized_dense_cloud, unorganized_search_methods, unorganized_dense_cloud_query_indices);
}
#endif

#if TEST_unorganized_dense_cloud_VIEW_KNN
// Test search on unorganized point clouds
TEST (PCL, unorganized_dense_cloud_View_KNN)
{
  testKNNSearch (unorganized_dense_cloud, unorganized_search_methods, unorganized_dense_cloud_query_indices, unorganized_input_indices);
}
#endif

#if TEST_unorganized_sparse_cloud_COMPLETE_KNN
// Test search on unorganized point clouds
TEST (PCL, unorganized_sparse_cloud_Complete_KNN)
{
  testKNNSearch (unorganized_sparse_cloud, unorganized_search_methods, unorganized_sparse_cloud_query_indices);
}
#endif

#if TEST_unorganized_sparse_cloud_VIEW_KNN
TEST (PCL, unorganized_sparse_cloud_View_KNN)
{   
  testKNNSearch (unorganized_sparse_cloud, unorganized_search_methods, unorganized_sparse_cloud_query_indices, unorganized_input_indices);
}
#endif

#if TEST_unorganized_dense_cloud_COMPLETE_RADIUS
// Test search on unorganized point clouds
TEST (PCL, unorganized_dense_cloud_Complete_Radius)
{
  testRadiusSearch (unorganized_dense_cloud, unorganized_search_methods, unorganized_dense_cloud_query_indices);
}
#endif

#if TEST_unorganized_grid_cloud_COMPLETE_RADIUS
// Test search on unorganized point clouds in a grid
TEST (PCL, unorganized_grid_cloud_Complete_Radius)
{
  vector<int> query_indices;
  query_indices.reserve (query_count);
  
  unsigned skip = static_cast<unsigned> (unorganized_grid_cloud->size ()) / query_count;
  for (unsigned idx = 0; idx < unorganized_grid_cloud->size () && query_indices.size () < query_count; ++idx)
     if ((rand () % skip) == 0 && isFinite (unorganized_grid_cloud->points [idx]))
       query_indices.push_back (idx);
  
  testRadiusSearch (unorganized_grid_cloud, unorganized_search_methods, query_indices);
}
#endif

#if TEST_unorganized_dense_cloud_VIEW_RADIUS
// Test search on unorganized point clouds
TEST (PCL, unorganized_dense_cloud_View_Radius)
{  
  testRadiusSearch (unorganized_dense_cloud, unorganized_search_methods, unorganized_dense_cloud_query_indices, unorganized_input_indices);
}
#endif

#if TEST_unorganized_sparse_cloud_COMPLETE_RADIUS
// Test search on unorganized point clouds
TEST (PCL, unorganized_sparse_cloud_Complete_Radius)
{
  testRadiusSearch (unorganized_sparse_cloud, unorganized_search_methods, unorganized_sparse_cloud_query_indices);
}
#endif

#if TEST_unorganized_sparse_cloud_VIEW_RADIUS
TEST (PCL, unorganized_sparse_cloud_View_Radius)
{   
  testRadiusSearch (unorganized_sparse_cloud, unorganized_search_methods, unorganized_sparse_cloud_query_indices, unorganized_input_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_COMPLETE_KNN
TEST (PCL, Organized_Sparse_Complete_KNN)
{
  testKNNSearch (organized_sparse_cloud, organized_search_methods, organized_sparse_query_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_VIEW_KNN
TEST (PCL, Organized_Sparse_View_KNN)
{
  testKNNSearch (organized_sparse_cloud, organized_search_methods, organized_sparse_query_indices, organized_input_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_COMPLETE_RADIUS
TEST (PCL, Organized_Sparse_Complete_Radius)
{
  testRadiusSearch (organized_sparse_cloud, organized_search_methods, organized_sparse_query_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_VIEW_RADIUS
TEST (PCL, Organized_Sparse_View_Radius)
{
  testRadiusSearch (organized_sparse_cloud, organized_search_methods, organized_sparse_query_indices, organized_input_indices);
}
#endif

/** \brief create subset of point in cloud to use as query points
  * \param[out] query_indices resulting query indices - not guaranteed to have size of query_count but guaranteed not to exceed that value
  * \param cloud input cloud required to check for nans and to get number of points
  * \param[in] query_count maximum number of query points
  */
void createQueryIndices (std::vector<int>& query_indices, PointCloud<PointXYZ>::ConstPtr point_cloud, unsigned query_count)
{
  query_indices.clear ();
  query_indices.reserve (query_count);
  
  unsigned skip = static_cast<unsigned> (point_cloud->size ()) / query_count;
  for (unsigned idx = 0; idx < point_cloud->size () && query_indices.size () < query_count; ++idx)
     if ((rand () % skip) == 0 && isFinite (point_cloud->points [idx]))
       query_indices.push_back (idx);
}

/** \brief create an approx 50% view (subset) of a cloud.
  * \param indices 
  * \param max_index highest accented index usually given by cloud->size () - 1
  */
void createIndices (std::vector<int>& indices, unsigned max_index)
{
  // ~10% of the input cloud
  for (unsigned idx = 0; idx <= max_index; ++idx)
    if (rand_uint () == 0)
      indices.push_back (idx);
   
  boost::variate_generator< boost::mt19937, boost::uniform_int<> > rand_indices(boost::mt19937 (), boost::uniform_int<> (0, static_cast<int> (indices.size ()) - 1));
  // shuffle indices -> not ascending index list
  for (unsigned idx = 0; idx < max_index; ++idx)
  {
    unsigned idx1 = rand_indices ();
    unsigned idx2 = rand_indices ();

    std::swap (indices[idx1], indices[idx2]);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "need path to table_scene_mug_stereo_textured.pcd file\n";
    return (-1);
  }

  pcl::io::loadPCDFile (argv [1], *organized_sparse_cloud);
  
  // create unorganized cloud
  unorganized_dense_cloud->resize (unorganized_point_count);
  unorganized_dense_cloud->height = 1;
  unorganized_dense_cloud->width = unorganized_point_count;
  unorganized_dense_cloud->is_dense = true;
  
  unorganized_sparse_cloud->resize (unorganized_point_count);
  unorganized_sparse_cloud->height = 1;
  unorganized_sparse_cloud->width = unorganized_point_count;
  unorganized_sparse_cloud->is_dense = false;

  PointXYZ point;
  for (unsigned pIdx = 0; pIdx < unorganized_point_count; ++pIdx)
  {
    point.x = rand_float ();
    point.y = rand_float ();
    point.z = rand_float ();

    unorganized_dense_cloud->points [pIdx] = point;
    
    if (rand_uint () == 0)
      unorganized_sparse_cloud->points [pIdx].x = unorganized_sparse_cloud->points [pIdx].y = unorganized_sparse_cloud->points [pIdx].z = std::numeric_limits<float>::quiet_NaN ();
    else
      unorganized_sparse_cloud->points [pIdx] = point;
  }
  
  unorganized_grid_cloud->reserve (1000);
  unorganized_grid_cloud->height = 1;
  unorganized_grid_cloud->width = 1000;
  unorganized_grid_cloud->is_dense = true;
  
  // values between 0 and 1
  for (unsigned xIdx = 0; xIdx < 10; ++xIdx)
  {
    for (unsigned yIdx = 0; yIdx < 10; ++yIdx)
    {
      for (unsigned zIdx = 0; zIdx < 10; ++zIdx)
      {
        point.x = 0.1f * static_cast<float>(xIdx);
        point.y = 0.1f * static_cast<float>(yIdx);
        point.z = 0.1f * static_cast<float>(zIdx);
        unorganized_grid_cloud->push_back (point);
      }
    }
  }
  
  createIndices (organized_input_indices, static_cast<unsigned> (organized_sparse_cloud->size () - 1));
  createIndices (unorganized_input_indices, unorganized_point_count - 1);
  
  brute_force.setSortedResults (true);
  KDTree.setSortedResults (true);
  octree_search.setSortedResults (true);
  organized.setSortedResults (true);
  
  unorganized_search_methods.push_back (&brute_force);
  unorganized_search_methods.push_back (&KDTree);
  unorganized_search_methods.push_back (&octree_search);
  
  organized_search_methods.push_back (&brute_force);
  organized_search_methods.push_back (&KDTree);
  organized_search_methods.push_back (&octree_search);
  organized_search_methods.push_back (&organized);
  
  createQueryIndices (unorganized_dense_cloud_query_indices, unorganized_dense_cloud, query_count);
  createQueryIndices (unorganized_sparse_cloud_query_indices, unorganized_sparse_cloud, query_count);
  createQueryIndices (organized_sparse_query_indices, organized_sparse_cloud, query_count);
  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

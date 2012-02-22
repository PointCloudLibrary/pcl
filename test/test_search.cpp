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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#include <boost/smart_ptr/shared_array.hpp>

#include "pcl/common/time.h"

using namespace pcl;
using namespace std;

/** \brief if set to value other than 0 -> fine grained output */
#define DEBUG_OUT 0

#define TEST_UNORGANIZED_DENSE_COMPLETE_KNN     1
#define TEST_UNORGANIZED_DENSE_VIEW_KNN         1
#define TEST_UNORGANIZED_SPARSE_COMPLETE_KNN    1
#define TEST_UNORGANIZED_SPARSE_VIEW_KNN        1
#define TEST_UNORGANIZED_DENSE_COMPLETE_RADIUS  1
#define TEST_UNORGANIZED_DENSE_VIEW_RADIUS      1
#define TEST_UNORGANIZED_SPARSE_COMPLETE_RADIUS 1
#define TEST_UNORGANIZED_SPARSE_VIEW_RADIUS     1
#define TEST_ORGANIZED_SPARSE_COMPLETE_KNN      1
#define TEST_ORGANIZED_SPARSE_VIEW_KNN          1
#define TEST_ORGANIZED_SPARSE_COMPLETE_RADIUS   1
#define TEST_ORGANIZED_SPARSE_VIEW_RADIUS       1

/** \brief number of points used for creating unordered point clouds */
const unsigned int point_count = 1200;

/** \brief number of search operations on ordered point clouds*/
const unsigned int ordered_query_count = 100;

/** \brief ordered point cloud*/
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

/** \briet test whether the result of a search containes unique point ids or not
  * @param indices resulting indices from a search
  * @param name name of the search method that returned these distances
  * @return true if indices are unique, false otherwise
  */
bool testUniqueness (const vector<int>& indices, const string& name)
{
  bool uniqueness = true;
  for (unsigned idx1 = 1; idx1 < indices.size (); ++idx1)
  {
    // check whether resulting indices are unique
    for (unsigned idx2 = 0; idx2 < idx1; ++idx2)
    {
      if (indices [idx1] == indices [idx2])
      {
#if DEBUG_OUT
        std::cout << name << " search: index is twice at positions: " << idx1 << " (" << indices [idx1] << ") , " << idx2  << " (" << indices [idx2] << ")" << std::endl;
#endif        
        uniqueness = false;
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
    }
  }
  
  return ordered;
}

/** \brief test whether the results are from the view (subset of the cloud) given by input_indices and also not Nan
 * @param indices_mask defines the subset of allowed points (view) in the reult of the search
 * @param nan_mask defines a lookup that indicates whether a point at a given position is finite or not
 * @param indices result of a search to be tested
 * @param name name of search method that returned the result
 * @return true if result is valid, false otherwise
 */
template<typename PointT> bool
testResultValidity (const vector<bool>& indices_mask, const vector<bool>& nan_mask, const vector<int>& indices, const string& name)
{
  bool validness = true;
  for (vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
  {
    if (!indices_mask [*iIt])
    {
#if DEBUG_OUT      
      cerr << name << ": result contains an invalid point: " << *iIt << " not in indices list.\n";
      
      for (vector<int>::const_iterator iIt2 = input_indices.begin (); iIt2 != input_indices.end (); ++iIt2)
        cout << *iIt2 << "  ";
      cout << endl;
#endif            
      validness = false;
    }
    else if (!nan_mask [*iIt])
    {
#if DEBUG_OUT      
      cerr << name << ": result contains an invalid point: " << *iIt << " = NaN (" << cloud->points [*iIt].x << " , " 
                                                                                   << cloud->points [*iIt].y << " , " 
                                                                                   << cloud->points [*iIt].z << ")\n";
#endif      
      validness = false;
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
testKNNSearch (typename PointCloud<PointT>::ConstPtr cloud, vector<search::Search<PointT>*> search_methods,
                const vector<int>& query_indices, const vector<int>& input_indices = vector<int> () )
{
  vector< vector<int> >indices (search_methods.size ());
  vector< vector<float> >distances (search_methods.size ());
  vector<bool> passed (search_methods.size (), true);
  
  vector<bool> indices_mask (cloud->size ());
  vector<bool> nan_mask (cloud->size ());
  
  if (input_indices.size () != 0)
  {
    indices_mask.assign (cloud->size (), false);
    for (vector<int>::const_iterator iIt = input_indices.begin (); iIt != input_indices.end (); ++iIt)
      indices_mask [*iIt] = true;
  }
  
  // remove also Nans
  for (unsigned pIdx = 0; pIdx < cloud->size (); ++pIdx)
  {
    if (!isFinite (cloud->points [pIdx]))
      nan_mask [pIdx] = false;
  }
  
  boost::shared_ptr<vector<int> > input_indices_;
  if (input_indices.size ())
    input_indices_.reset (new vector<int> (input_indices));
  
  for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
    search_methods [sIdx]->setInputCloud (cloud, input_indices_);

  // test knn values from 1, 8, 64, 512
  for (unsigned knn = 1; knn <= 512; knn <<= 3)
  {
    // find nn for each point in the cloud
    for (vector<int>::const_iterator qIt = query_indices.begin (); qIt != query_indices.end (); ++qIt)
    {
      for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
      {
        search_methods [sIdx]->nearestKSearch (cloud->points[*qIt], knn, indices [sIdx], distances [sIdx]);
        passed [sIdx] = testUniqueness (indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
        passed [sIdx] = testOrder (distances [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
        passed [sIdx] = testResultValidity<PointT>(indices_mask, nan_mask, indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
      }
      
      // compare results to each other
      for (unsigned sIdx1 = 1; sIdx1 < search_methods.size (); ++sIdx1)
      { 
        compareResults (indices [0], distances [0], search_methods [0]->getName (),
                        indices [sIdx1], distances [sIdx1], search_methods [sIdx1]->getName (), 1e-6 );
      }
    }
//    // find nn for points out of the cloud
//    PointT point;
//    for (unsigned pIdx = 0; pIdx < query_indices.size (); ++pIdx)
//    {
//      point.x = 2.0 * (float)rand () / (float) RAND_MAX;
//      point.y = 2.0 * (float)rand () / (float) RAND_MAX;
//      point.z = 2.0 * (float)rand () / (float) RAND_MAX;
//      
//      for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
//      {
//        search_methods [sIdx]->nearestKSearch (point, knn, indices [sIdx], distances [sIdx]);
//        passed [sIdx] = testUniqueness (indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
//        passed [sIdx] = testOrder (distances [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
//        passed [sIdx] = testResultValidity<PointT>(indices_mask, nan_mask, indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
//      }
//      
//      // compare results to each other
//      for (unsigned sIdx1 = 1; sIdx1 < search_methods.size (); ++sIdx1)
//      {
//        compareResults (indices [0], distances [0], search_methods [0]->getName (),
//                        indices [sIdx1], distances [sIdx1], search_methods [sIdx1]->getName (), 1e-6 );
//      }
//    }
  }
  for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
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
testRadiusSearch (typename PointCloud<PointT>::ConstPtr cloud, vector<search::Search<PointT>*> search_methods, 
                   const vector<int>& query_indices, const vector<int>& input_indices = vector<int> ())
{
  vector< vector<int> >indices (search_methods.size ());
  vector< vector<float> >distances (search_methods.size ());
  vector <bool> passed (search_methods.size (), true);
  vector<bool> indices_mask (cloud->size ());
  vector<bool> nan_mask (cloud->size ());
  
  if (input_indices.size () != 0)
  {
    indices_mask.assign (cloud->size (), false);
    for (vector<int>::const_iterator iIt = input_indices.begin (); iIt != input_indices.end (); ++iIt)
      indices_mask [*iIt] = true;
  }
  
  // remove also Nans
  for (unsigned pIdx = 0; pIdx < cloud->size (); ++pIdx)
  {
    if (!isFinite (cloud->points [pIdx]))
      nan_mask [pIdx] = false;
  }
  
  boost::shared_ptr<vector<int> > input_indices_;
  if (input_indices.size ())
    input_indices_.reset (new vector<int> (input_indices));
  
  for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
    search_methods [sIdx]->setInputCloud (cloud, input_indices_);

  // test radii 0.01, 0.02, 0.04, 0.08
  for (float radius = 0.01; radius < 0.1; radius *= 2.0)
  {
    //cout << radius << endl;
    // find nn for each point in the cloud
    for (vector<int>::const_iterator qIt = query_indices.begin (); qIt != query_indices.end (); ++qIt)
    {
      for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
      {
        search_methods [sIdx]->radiusSearch (cloud->points[*qIt], radius, indices [sIdx], distances [sIdx], 0);
        //cout << search_methods [sIdx]->getName () << " :: " << radius << " :: " << qIt - query_indices.begin () << " / " << query_indices.size () << " results: " << indices [sIdx].size () << endl;
        passed [sIdx] = testUniqueness (indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
        passed [sIdx] = testOrder (distances [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
        passed [sIdx] = testResultValidity<PointT>(indices_mask, nan_mask, indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
      }
      
      // compare results to each other
      for (unsigned sIdx1 = 1; sIdx1 < search_methods.size (); ++sIdx1)
      {
        compareResults (indices [0], distances [0], search_methods [0]->getName (),
                        indices [sIdx1], distances [sIdx1], search_methods [sIdx1]->getName (), 1e-6 );
      }
    }
    // find nn for points out of the cloud
//    PointT point;
//    for (unsigned pIdx = 0; pIdx < query_indices.size (); ++pIdx)
//    {
//      point.x = 2.0 * (float)rand () / (float) RAND_MAX;
//      point.y = 2.0 * (float)rand () / (float) RAND_MAX;
//      point.z = 2.0 * (float)rand () / (float) RAND_MAX;
//      
//      for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
//      {
//        search_methods [sIdx]->radiusSearch (point, radius, indices [sIdx], distances [sIdx], 0);
//        passed [sIdx] = testUniqueness (indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
//        passed [sIdx] = testOrder (distances [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
//        passed [sIdx] = testResultValidity<PointT>(indices_mask, nan_mask, indices [sIdx], search_methods [sIdx]->getName ()) || passed [sIdx];
//      }
//      
//      // compare results to each other
//      for (unsigned sIdx1 = 1; sIdx1 < search_methods.size (); ++sIdx1)
//      {
//        compareResults (indices [0], distances [0], search_methods [0]->getName (),
//                        indices [sIdx1], distances [sIdx1], search_methods [sIdx1]->getName (), 1e-6 );
//      }
//    }
  }
  for (unsigned sIdx = 0; sIdx < search_methods.size (); ++sIdx)
  {
    cout << search_methods [sIdx]->getName () << ": " << (passed[sIdx]?"passed":"failed") << endl;
    EXPECT_TRUE (passed [sIdx]);
  }
}

#if TEST_UNORGANIZED_DENSE_COMPLETE_KNN
// Test search on unorganized point clouds
TEST (PCL, Unorganized_Dense_Complete_KNN)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = true;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     point.x = (float)rand () / (float)RAND_MAX;
     point.y = (float)rand () / (float)RAND_MAX;
     point.z = (float)rand () / (float)RAND_MAX;
     
     unorganized->points [pIdx] = point;
   }
   //std::cout << "testing knn search with sorted results on unordered point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;

   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);
   
   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.01);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices (size);
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     query_indices [idx] = idx;
   
   testKNNSearch (unorganized, search_methods, query_indices);
}
#endif

#if TEST_UNORGANIZED_DENSE_VIEW_KNN
// Test search on unorganized point clouds
TEST (PCL, Unorganized_Dense_View_KNN)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = true;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     point.x = (float)rand () / (float)RAND_MAX;
     point.y = (float)rand () / (float)RAND_MAX;
     point.z = (float)rand () / (float)RAND_MAX;
     
     unorganized->points [pIdx] = point;
   }
   vector<int> input_indices;
   for (unsigned idx = 0; idx < size; ++idx)
     if ((rand () % 2) == 0)
       input_indices.push_back (idx);
   
   // shuffle indices -> not ascending index list
   for (unsigned idx = 0; idx < size; ++idx)
   {
     unsigned idx1 = rand () % input_indices.size ();
     unsigned idx2 = rand () % input_indices.size ();
     
     std::swap (input_indices[idx1], input_indices[idx2]);
   }
     
   //std::cout << "testing knn search with sorted results on views of unordered point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;

   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.01);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);

   vector<int> query_indices (size);
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     query_indices [idx] = idx;
   
   testKNNSearch (unorganized, search_methods, query_indices, input_indices);
}
#endif

#if TEST_UNORGANIZED_SPARSE_COMPLETE_KNN
// Test search on unorganized point clouds
TEST (PCL, Unorganized_Sparse_Complete_KNN)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = false;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     // 10% Nans
     if ((rand () % 10) == 0)
       point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
     else
     {
       point.x = (float)rand () / (float)RAND_MAX;
       point.y = (float)rand () / (float)RAND_MAX;
       point.z = (float)rand () / (float)RAND_MAX;
     }
     unorganized->points [pIdx] = point;
   }
   
   //std::cout << "testing knn search with sorted results on unordered point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);
   
   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.01);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices;
   query_indices.reserve (size);
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
   {
     if (isFinite (unorganized->points [idx]))
       query_indices.push_back (idx);
   }
   testKNNSearch (unorganized, search_methods, query_indices);
}
#endif

#if TEST_UNORGANIZED_SPARSE_VIEW_KNN
TEST (PCL, Unorganized_Sparse_View_KNN)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = false;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     // 10% Nans
     if ((rand () % 10) == 0)
       point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
     else
     {
       point.x = (float)rand () / (float)RAND_MAX;
       point.y = (float)rand () / (float)RAND_MAX;
       point.z = (float)rand () / (float)RAND_MAX;
     }
     unorganized->points [pIdx] = point;
   }
   
   // ~50% of the input cloud
   vector<int> indices;
   for (unsigned idx = 0; idx < size; ++idx)
     if (rand () % 2)
       indices.push_back (idx);
   
   // shuffle indices -> not ascending index list
   for (unsigned idx = 0; idx < size; ++idx)
   {
     unsigned idx1 = rand () % indices.size ();
     unsigned idx2 = rand () % indices.size ();
     
     std::swap (indices[idx1], indices[idx2]);
   }
     
   //std::cout << "testing knn search with sorted results on views of unordered and non-dense point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);
   
   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.1);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices;
   query_indices.reserve (size);
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     if (isFinite (unorganized->points [idx]))
      query_indices.push_back (idx);
   
   testKNNSearch (unorganized, search_methods, query_indices, indices);
}
#endif

#if TEST_UNORGANIZED_DENSE_COMPLETE_RADIUS
// Test search on unorganized point clouds
TEST (PCL, Unorganized_Dense_Complete_Radius)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = true;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     point.x = (float)rand () / (float)RAND_MAX;
     point.y = (float)rand () / (float)RAND_MAX;
     point.z = (float)rand () / (float)RAND_MAX;
     
     unorganized->points [pIdx] = point;
   }
   //std::cout << "testing knn search with sorted results on unordered point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.01);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices (size);
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     query_indices [idx] = idx;
   
   testRadiusSearch (unorganized, search_methods, query_indices);
}
#endif

#if TEST_UNORGANIZED_DENSE_VIEW_RADIUS
// Test search on unorganized point clouds
TEST (PCL, Unorganized_Dense_View_Radius)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = true;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     point.x = (float)rand () / (float)RAND_MAX;
     point.y = (float)rand () / (float)RAND_MAX;
     point.z = (float)rand () / (float)RAND_MAX;
     
     unorganized->points [pIdx] = point;
   }
   vector<int> indices;
   for (unsigned idx = 0; idx < size; ++idx)
     if ((rand () % 2) == 0)
       indices.push_back (idx);
   
   // shuffle indices -> not ascending index list
   for (unsigned idx = 0; idx < size; ++idx)
   {
     unsigned idx1 = rand () % indices.size ();
     unsigned idx2 = rand () % indices.size ();
     
     std::swap (indices[idx1], indices[idx2]);
   }
     
   //std::cout << "testing knn search with sorted results on views of unordered point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.01);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices (size);
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     query_indices [idx] = idx;
   
   testRadiusSearch (unorganized, search_methods, query_indices, indices);
}
#endif

#if TEST_UNORGANIZED_SPARSE_COMPLETE_RADIUS
// Test search on unorganized point clouds
TEST (PCL, Unorganized_Sparse_Complete_Radius)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = false;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     // 10% Nans
     if ((rand () % 10) == 0)
       point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
     else
     {
       point.x = (float)rand () / (float)RAND_MAX;
       point.y = (float)rand () / (float)RAND_MAX;
       point.z = (float)rand () / (float)RAND_MAX;
     }
     unorganized->points [pIdx] = point;
   }
   
   //std::cout << "testing knn search with sorted results on unordered point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;

   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);
   
   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.01);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices;
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     if (isFinite (unorganized->points [idx]))
      query_indices [idx] = idx;
   
   testRadiusSearch (unorganized, search_methods, query_indices);
}
#endif

#if TEST_UNORGANIZED_SPARSE_VIEW_RADIUS
TEST (PCL, Unorganized_Sparse_View_Radius)
{
   const unsigned int size = point_count;
  //Create unorganized point cloud and test some corner cases
   PointCloud<PointXYZ>::Ptr unorganized (new PointCloud<PointXYZ>);
   unorganized->resize (size);
   unorganized->height = 1;
   unorganized->width = size;
   unorganized->is_dense = false;
   
   PointXYZ point;
   for (unsigned pIdx = 0; pIdx < size; ++pIdx)
   {
     // 10% Nans
     if ((rand () % 10) == 0)
       point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
     else
     {
       point.x = (float)rand () / (float)RAND_MAX;
       point.y = (float)rand () / (float)RAND_MAX;
       point.z = (float)rand () / (float)RAND_MAX;
     }
     unorganized->points [pIdx] = point;
   }
   
   // ~50% of the input cloud
   vector<int> indices;
   for (unsigned idx = 0; idx < size; ++idx)
     if (rand () % 2)
       indices.push_back (idx);
   
   // shuffle indices -> not ascending index list
   for (unsigned idx = 0; idx < size; ++idx)
   {
     unsigned idx1 = rand () % indices.size ();
     unsigned idx2 = rand () % indices.size ();
     
     std::swap (indices[idx1], indices[idx2]);
   }
     
   //std::cout << "testing knn search with sorted results on views of unordered and non-dense point clouds...\n";
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.1);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   vector<int> query_indices;
   for (unsigned idx = 0; idx < query_indices.size (); ++idx)
     if (isFinite (unorganized->points [idx]))
      query_indices [idx] = idx;
   
   testRadiusSearch (unorganized, search_methods, query_indices, indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_COMPLETE_KNN
TEST (PCL, Organized_Sparse_Complete_KNN)
{
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.1);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   pcl::search::OrganizedNeighbor<pcl::PointXYZ> organized;
   organized.setSortedResults (true);
   search_methods.push_back (&organized);
   
   vector<int> query_indices;
   query_indices.reserve (ordered_query_count);
   
   unsigned skip = cloud->size () / ordered_query_count;
   for (unsigned idx = 0; idx < cloud->size () && query_indices.size () < ordered_query_count; ++idx)
     if ((rand () % skip) == 0 && isFinite (cloud->points [idx]))
        query_indices.push_back (idx);
   
   testKNNSearch (cloud, search_methods, query_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_VIEW_KNN
TEST (PCL, Organized_Sparse_View_KNN)
{
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.1);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   pcl::search::OrganizedNeighbor<pcl::PointXYZ> organized;
   organized.setSortedResults (true);
   search_methods.push_back (&organized);
   
   // ~50% of the input cloud
   vector<int> input_indices;
   for (unsigned idx = 0; idx < cloud->size (); ++idx)
     if (rand () % 2)
       input_indices.push_back (idx);
   
   // shuffle indices -> not ascending index list
   for (unsigned idx = 0; idx < cloud->size (); ++idx)
   {
     unsigned idx1 = rand () % input_indices.size ();
     unsigned idx2 = rand () % input_indices.size ();
     
     std::swap (input_indices[idx1], input_indices[idx2]);
   }
   
   vector<int> query_indices;
   query_indices.reserve (ordered_query_count);
   
   unsigned skip = cloud->size () / ordered_query_count;
   for (unsigned idx = 0; idx < cloud->size () && query_indices.size () < ordered_query_count; ++idx)
     if ((rand () % skip) == 0 && isFinite (cloud->points [idx]))
       query_indices.push_back (idx);
   
   testKNNSearch (cloud, search_methods, query_indices, input_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_COMPLETE_RADIUS
TEST (PCL, Organized_Sparse_Complete_Radius)
{
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
   pcl::search::Octree<pcl::PointXYZ> octree (0.1);
   octree.setSortedResults (true);
   search_methods.push_back (&octree);
   
   pcl::search::OrganizedNeighbor<pcl::PointXYZ> organized;
   organized.setSortedResults (true);
   search_methods.push_back (&organized);
   
   vector<int> query_indices;
   query_indices.reserve (ordered_query_count);
   
   unsigned skip = cloud->size () / ordered_query_count;
   for (unsigned idx = 0; idx < cloud->size () && query_indices.size () < ordered_query_count; ++idx)
     if ((rand () % skip) == 0 && isFinite (cloud->points [idx]))
       query_indices.push_back (idx);
   
   testRadiusSearch (cloud, search_methods, query_indices);
}
#endif

#if TEST_ORGANIZED_SPARSE_VIEW_RADIUS
TEST (PCL, Organized_Sparse_View_Radius)
{
   vector<search::Search<PointXYZ>* > search_methods;
   
   pcl::search::BruteForce<pcl::PointXYZ> brute_force;
   brute_force.setSortedResults (true);
   search_methods.push_back (&brute_force);

   pcl::search::KdTree<pcl::PointXYZ> KDTree;
   KDTree.setSortedResults (true);
   search_methods.push_back (&KDTree);
   
//   pcl::search::Octree<pcl::PointXYZ> octree (0.1);
//   octree.setSortedResults (true);
//   search_methods.push_back (&octree);
   
   pcl::search::OrganizedNeighbor<pcl::PointXYZ> organized;
   organized.setSortedResults (true);
   search_methods.push_back (&organized);
   
   // ~50% of the input cloud
   vector<int> input_indices;
   for (unsigned idx = 0; idx < cloud->size (); ++idx)
     if (rand () % 2)
       input_indices.push_back (idx);
   
   // shuffle indices -> not ascending index list
   for (unsigned idx = 0; idx < cloud->size (); ++idx)
   {
     unsigned idx1 = rand () % input_indices.size ();
     unsigned idx2 = rand () % input_indices.size ();
     
     std::swap (input_indices[idx1], input_indices[idx2]);
   }
   
   vector<int> query_indices;
   query_indices.reserve (ordered_query_count);
   
   unsigned skip = cloud->size () / ordered_query_count;
   for (unsigned idx = 0; idx < cloud->size () && query_indices.size () < ordered_query_count; ++idx)
     if ((rand () % skip) == 0 && isFinite (cloud->points [idx]))
       query_indices.push_back (idx);
   
   testRadiusSearch (cloud, search_methods, query_indices, input_indices);
}
#endif

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "need path to table_scene_mug_stereo_textured.pcd file\n";
    return (-1);
  }

  std::cout << "loading " << argv [1] << std::endl;
  pcl::io::loadPCDFile (argv [1], *cloud);
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

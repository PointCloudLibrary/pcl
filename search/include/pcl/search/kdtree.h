/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: kdtree_flann.h 36261 2011-02-26 01:34:42Z mariusm $
 *
 */

#ifndef PCL_SEARCH_KDTREE_FLANN_H_
#define PCL_SEARCH_KDTREE_FLANN_H_

#include <cstdio>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <boost/thread/mutex.hpp>
//#include <flann/flann.hpp>

//using namespace std;
namespace pcl
{
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b KdTreeWrapper is a generic type of 3D spatial locator using kD-tree structures. The class is making use of
 * the FLANN (Fast Library for Approximate Nearest Neighbor) project by Marius Muja and David Lowe.
 *
 * @note libFLANN is not thread safe, so we need mutices in places to make KdTreeWrapper thread safe.
 * \author Radu Bogdan Rusu
 * \ingroup kdtree
 */
  template <typename PointT>
  class KdTreeWrapper : public pcl::Search<PointT>
  {
    /*
        using KdTree<PointT>::input_;
        using KdTree<PointT>::indices_;
        using KdTree<PointT>::epsilon_;
        using KdTree<PointT>::sorted_;
        using KdTree<PointT>::point_representation_;
     */
    typedef typename Search<PointT>::PointCloud PointCloud;
    typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

    typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
    typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;


  public:
    // Boost shared pointers
    typedef boost::shared_ptr<KdTree<PointT> > Ptr;
    typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;
    Ptr _searchptr;

    // Boost shared pointers
    ////      typedef boost::shared_ptr<KdTree<PointT> > Ptr;
    //      typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;


    //      KdTree (bool sorted = true) : input_(), indices_(),
    //                                  epsilon_(0.0), min_pts_(1), sorted_(sorted)
    //  {
    //  point_representation_.reset (new DefaultPointRepresentation<PointT>);
    //     };

    KdTreeWrapper (bool sorted = true)
    {
        _searchptr.reset (new KdTreeFLANN<PointT>(sorted));

    }



    ~KdTreeWrapper ()
    {
//        _searchptr->~KdTree();
    }


    void setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices); 
    void setInputCloud (const PointCloudConstPtr& cloud);

    int  nearestKSearch (const PointT& point, int k,
                         std::vector<int>& k_indices, std::vector<float>& k_distances);

    inline int
    nearestKSearch (const PointCloud& cloud, int index, int k,
                    std::vector<int>& k_indices, std::vector<float>& k_distances)
    {
	return _searchptr->nearestKSearch(cloud,index,k,k_indices,k_distances);
    }

    /** \brief Search for k-nearest neighbors for the given query point (zero-copy).
     * \param index the index representing the query point in the dataset given by \a setInputCloud
     *        if indices were given in setInputCloud, index will be the position in the indices vector
     * \param k the number of neighbors to search for
     * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
     * \param k_distances the resultant squared distances to the neighboring points (must be resized to \a k
     * a priori!)
     * \return number of neighbors found
     */
    inline int
    nearestKSearch (int index, int k,
                    std::vector<int>& k_indices, std::vector<float>& k_distances) 
    {

      return (_searchptr->nearestKSearch(index,k,k_indices,k_distances));
    }

    int radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_distances, int max_nn = -1) const;

    /** \brief Search for all the nearest neighbors of the query point in a given radius.
     * \param cloud the point cloud data
     * \param index the index in \a cloud representing the query point
     * \param radius the radius of the sphere bounding all of p_q's neighbors
     * \param k_indices the resultant indices of the neighboring points
     * \param k_distances the resultant squared distances to the neighboring points
     * \param max_nn if given, bounds the maximum returned neighbors to this value
     * \return number of neighbors found in radius
     */
    inline int
    radiusSearch (const PointCloud& cloud, int index, double radius,
                  std::vector<int>& k_indices, std::vector<float>& k_distances,
                  int max_nn = -1) 
    {
	return _searchptr->radiusSearch(cloud,index,radius,k_indices,k_distances,max_nn);
    }

    /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
     * \param index the index representing the query point in the dataset given by \a setInputCloud
     *        if indices were given in setInputCloud, index will be the position in the indices vector
     * \param radius the radius of the sphere bounding all of p_q's neighbors
     * \param k_indices the resultant indices of the neighboring points
     * \param k_distances the resultant squared distances to the neighboring points
     * \param max_nn if given, bounds the maximum returned neighbors to this value
     * \return number of neighbors found in radius
     */
    inline int
    radiusSearch (int index, double radius, std::vector<int>& k_indices,
                  std::vector<float>& k_distances, int max_nn = -1) const
    {
	return _searchptr->radiusSearch(index,radius,k_indices,k_distances,max_nn);
    }

 /* Functions which are not implemented */
      inline void
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, int query_index_arg, int &result_index_arg,
                             float &sqr_distance_arg)

{

        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
}

      inline void
        approxNearestSearch (const PointT &p_q_arg, int &result_index_arg, float &sqr_distance_arg){

        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
};

      inline void
        approxNearestSearch (int query_index_arg, int &result_index_arg, float &sqr_distance_arg){

        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
};


inline int
approxRadiusSearch (const PointCloudConstPtr& cloud, int index, double radius,
                              std::vector<int>& k_indices, std::vector<float>& k_distances,
                              int max_nn)const
{
        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
}


inline int
approxNearestKSearch (const PointCloudConstPtr& cloud, int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances)
{
        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
}

inline void
            evaluateSearchMethods (const PointCloudConstPtr& cloud, const int search_type){

        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
}


inline int
    nearestKSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> >& point, std::vector <int>& k, std::vector<std::vector<int> >& k_indices,    std::vector<std::vector<float> >& k_sqr_distances){
        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
};


inline int
    radiusSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> >& point, std::vector <  double >& radiuses, std::vector<std::vector<int> >& k_indices,    std::vector<std::vector<float> >& k_distances, int max_nn) const
{

        std::cerr << "This function is not supported by KdTreeWrapper" << std::endl;
exit(0);
};




  };

}

#endif

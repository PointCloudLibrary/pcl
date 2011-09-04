/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
  namespace search
  {
    /** \brief @b KdTreeWrapper is a generic type of 3D spatial locator using kD-tree structures. The class is making use of
     * the FLANN (Fast Library for Approximate Nearest Neighbor) project by Marius Muja and David Lowe.
     *
     * @note libFLANN is not thread safe, so we need mutices in places to make KdTreeWrapper thread safe.
     * \author Radu Bogdan Rusu
     * \ingroup kdtree
     */
    template <typename PointT>
    class KdTree : public pcl::search::Search<PointT>
    {
      typedef typename Search<PointT>::PointCloud PointCloud;
      typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
      typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;


      public:
        // Boost shared pointers
        typedef boost::shared_ptr<pcl::KdTreeFLANN<PointT> > Ptr;
        typedef boost::shared_ptr<const pcl::KdTreeFLANN<PointT> > ConstPtr;
        Ptr tree_;

	/** \brief Empty constructor for KdTree. */
        KdTree (bool sorted = true)
        {
          tree_.reset (new pcl::KdTreeFLANN<PointT> (sorted));
        }

        /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
        virtual ~KdTree ()
        {
        }
	/** \brief Provide a pointer to the input dataset.
        * \param cloud the const boost shared pointer to a PointCloud message
        * \param indices the point indices subset that is to be used from \a cloud - if NULL the whole point cloud is used
        */
        void setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices); 


	/** \brief Provide a pointer to the input dataset.
        * \param cloud the const boost shared pointer to a PointCloud message
        */
        void setInputCloud (const PointCloudConstPtr& cloud);

	/** \brief Search for k-nearest neighbors for the given query point.
        * \param point the given query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */

        int  nearestKSearch (const PointT &point, int k,
                             std::vector<int> &k_indices, 
                             std::vector<float> &k_distances);


	/** \brief Search for k-nearest neighbors for the given query point.
        * \param cloud the point cloud data
        * \param index the index in \a cloud representing the query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
        inline int
        nearestKSearch (const PointCloud &cloud, 
                        int index, 
                        int k,
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_distances)
        {
          return (tree_->nearestKSearch (cloud, index, k, k_indices, k_distances));
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
        nearestKSearch (int index, 
                        int k,
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_distances) 
        {
          return (tree_->nearestKSearch (index, k, k_indices, k_distances));
        }
 	/** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param point the given query point
        * \param radius the radius of the sphere bounding all of p_q's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
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
        radiusSearch (const PointCloud& cloud, 
                      int index, 
                      double radius,
                      std::vector<int> &k_indices, 
                      std::vector<float> &k_distances,
                      int max_nn = -1) 
        {
          return (tree_->radiusSearch (cloud, index, radius, k_indices, k_distances, max_nn));
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
        radiusSearch (int index, 
                      double radius, 
                      std::vector<int> &k_indices,
                      std::vector<float> &k_distances, 
                      int max_nn = -1) const
        {
          return (tree_->radiusSearch (index, radius, k_indices, k_distances, max_nn));
        }

        /* Functions which are not implemented */

 	/** \brief Search for approx. nearest neighbor at the query point.
         * \param cloud_arg the const boost shared pointer to a PointCloud message
         * \param query_index_arg the index in \a cloud representing the query point
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         */
        inline void
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, 
                             int query_index_arg, 
                             int &result_index_arg,
                             float &sqr_distance_arg)

        {
          PCL_ERROR("[pcl::search::KdTree::approxNearestSearch] This function is not supported by KdTree\n");
        }

 	 /** \brief Search for approx. nearest neighbor at the query point.
         * @param p_q_arg the given query point
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         */
        inline void
        approxNearestSearch (const PointT &p_q_arg, 
                             int &result_index_arg, 
                             float &sqr_distance_arg)
        {
          PCL_ERROR("[pcl::search::KdTree::approxNearestSearch] This function is not supported by KdTree\n");
        }

 	/** \brief Search for approx. nearest neighbor at the query point.
         * \param query_index_arg index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector.
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         */
	inline void
        approxNearestSearch (int query_index_arg, 
                             int &result_index_arg, 
                             float &sqr_distance_arg)
        {
          PCL_ERROR("[pcl::search::KdTree::approxNearestSearch] This function is not supported by KdTree\n");
        }

 	/** \brief Approximate Search for all the nearest neighbors of the query point in a given radius.
        * \param cloud the const boost shared pointer to a PointCloud message
        * \param index the index in \a cloud representing the query point
        * \param radius the radius of the sphere bounding all of point's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
        inline int
        approxRadiusSearch (const PointCloudConstPtr &cloud, 
                            int index, 
                            double radius,
                            std::vector<int> &k_indices, 
                            std::vector<float> &k_distances,
                            int max_nn) const
        {
          PCL_ERROR("[pcl::search::KdTree::approxRadiusSearch] This function is not supported by KdTree\n");
          return (0);
        }


      /** \brief Approximate Search for k-nearest neighbors for the given query point.
        * \param cloud the const boost shared pointer to a PointCloud message
        * \param index the index in \a cloud representing the query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
  	* \return number of neighbors found
        */
        inline int
        approxNearestKSearch (const PointCloudConstPtr &cloud, 
                              int index, 
                              int k, 
                              std::vector<int> &k_indices, 
                              std::vector<float> &k_sqr_distances)
        {
          PCL_ERROR("[pcl::search::KdTree::approxNearestKSearch] This function is not supported by KdTree\n");
          return (0);
        }

        /** \brief Evaluate the Search Methods for the given cloud.
        * \param cloud the const boost shared pointer to a PointCloud message
        * \param search_type the search type NEAREST_K_SEARCH and NEAREST_RADIUS_SEARCH
        */
        inline void
        evaluateSearchMethods (const PointCloudConstPtr &cloud, 
                               const int search_type)
        {
          PCL_ERROR("[pcl::search::KdTree::evaluateSearchMethods] This function is not supported by KdTree\n");
        }


	/** \brief Search for k-nearest neighbors for the given query points.
        * \param point the given query points
        * \param k the numbers of the query point's neighbors to search for
        * \param k_indices the resultant indices of the neighboring points 
        * \param k_sqr_distances the resultant squared distances to the neighboring points 
        * \return number of neighbors found
        */
        inline int
        nearestKSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> > &point, 
                        std::vector<int> &k, 
                        std::vector<std::vector<int> > &k_indices,
                        std::vector<std::vector<float> > &k_sqr_distances)
        {
          PCL_ERROR("[pcl::search::KdTree::nearestKSearch] This function is not supported by KdTree\n");
          return (0);
        }


	/** \brief Approximate Search for all the nearest neighbors of the query points in the given radiuses.
        * \param point the given query points
        * \param radiuses the radiuses of the sphere bounding all of point's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radiuses
        */
        inline int
        radiusSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> > &point, 
                      std::vector<double> &radiuses, 
                      std::vector<std::vector<int> > &k_indices,
                      std::vector<std::vector<float> > &k_distances, 
                      int max_nn) const
        {
          PCL_ERROR("[pcl::search::KdTree::radiusSearch] This function is not supported by KdTree\n");
          return (0);
        }
    };
  }
}

#endif

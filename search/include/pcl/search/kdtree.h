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
#include <boost/thread/mutex.hpp>
#include <flann/flann.hpp>

//using namespace std;
namespace pcl
{
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b KdTree is a generic type of 3D spatial locator using kD-tree structures. The class is making use of
 * the FLANN (Fast Library for Approximate Nearest Neighbor) project by Marius Muja and David Lowe.
 *
 * @note libFLANN is not thread safe, so we need mutices in places to make KdTree thread safe.
 * \author Radu Bogdan Rusu
 * \ingroup kdtree
 */
  template <typename PointT>
  class KdTree : public pcl::Search<PointT>
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

    typedef flann::Index< flann::L2_Simple<float> > FLANNIndex;

    typedef pcl::PointRepresentation<PointT> PointRepresentation;
    //typedef boost::shared_ptr<PointRepresentation> PointRepresentationPtr;
    typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;

  public:
    // Boost shared pointers
    typedef boost::shared_ptr<KdTree<PointT> > Ptr;
    typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;


    // Boost shared pointers
    ////      typedef boost::shared_ptr<KdTree<PointT> > Ptr;
    //      typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;


    //      KdTree (bool sorted = true) : input_(), indices_(),
    //                                  epsilon_(0.0), min_pts_(1), sorted_(sorted)
    //  {
    //  point_representation_.reset (new DefaultPointRepresentation<PointT>);
    //     };

    KdTree (bool sorted = true) : input_(), indices_(),
        epsilon_(0.0), min_pts_(1), sorted_(sorted), flann_index_(NULL), cloud_(NULL)
    {
        point_representation_.reset (new DefaultPointRepresentation<PointT>);
        cleanup ();

    }



    ~KdTree ()
    {
        cleanup();
    }


    void setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices); 
    void setInputCloud (const PointCloudConstPtr& cloud);

    int  nearestKSearch (const PointT& point, int k,
                         std::vector<int>& k_indices, std::vector<float>& k_distances);

    inline int
    nearestKSearch (const PointCloud& cloud, int index, int k,
                    std::vector<int>& k_indices, std::vector<float>& k_distances)
    {
        if (index >= (int)cloud.points.size ()) return 0;
        return nearestKSearch (cloud.points[index], k, k_indices, k_distances);
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
        if (indices_ == NULL) {
            if (index >= (int)input_->points.size ()) return 0;
            return nearestKSearch (input_->points[index], k, k_indices, k_distances);
        }
        else {
            if (index >= (int)indices_->size()) return 0;
            return nearestKSearch (input_->points[(*indices_)[index]], k, k_indices, k_distances);
        }
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
        if (index >= (int)cloud.points.size ()) return 0;
        return radiusSearch(cloud.points[index], radius, k_indices, k_distances, max_nn);
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
        if (indices_ == NULL) {
            if (index >= (int)input_->points.size ()) return 0;
            return radiusSearch (input_->points[index], radius, k_indices, k_distances, max_nn);
        }
        else {
            if (index >= (int)indices_->size ()) return 0;
            return radiusSearch (input_->points[(*indices_)[index]], radius, k_indices, k_distances, max_nn);
        }
    }




  private:
    /** \brief Internal cleanup method. */
    void cleanup ();
    bool initParameters ();

    /** \brief Simple initialization method for internal data buffers. */
    void initData ();

    /** \brief Converts a ROS PointCloud message to the internal FLANN point array representation. Returns the number
     * of points.
     * \param ros_cloud the ROS PointCloud message
     */
    void convertCloudToArray (const PointCloud& ros_cloud);

    /** \brief Converts a ROS PointCloud message with a given set of indices to the internal FLANN point array
     * representation. Returns the number of points.
     * \note ATTENTION: This method breaks the 1-1 mapping between the indices returned using \a getNeighborsIndices
     * and the ones from the \a ros_cloud message ! When using this method, make sure to get the underlying point data
     * using the \a getPoint method
     * \param ros_cloud the ROS PointCloud message
     * \param indices the point cloud indices
     */
    void convertCloudToArray (const PointCloud& ros_cloud, const std::vector<int>& indices);

    inline IndicesConstPtr const
    getIndices ()
    {
        return indices_;
    }

    /** \brief Get a pointer to the input point cloud dataset. */
    inline PointCloudConstPtr
    getInputCloud ()
    {
        return input_;
    }


    inline void
    setEpsilon (double eps)
    {
        epsilon_ = eps;
    }

    /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
     * \param point_representation the const boost shared pointer to a PointRepresentation
     */
    inline void
    setPointRepresentation (const PointRepresentationConstPtr& point_representation)
    {
        point_representation_ = point_representation;
        setInputCloud (input_, indices_);  // Makes sense in derived classes to reinitialize the tree
    }

    /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
    inline PointRepresentationConstPtr const
    getPointRepresentation ()
    {
        return point_representation_;
    }


    /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
    inline double
    getEpsilon ()
    {
        return epsilon_;
    }

    /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. */
    inline void
    setMinPts (int min_pts)
    {
        min_pts_ = min_pts;
    }

    /** \brief Get the minimum allowed number of k nearest neighbors points that a viable result must contain. */
    inline float
    getMinPts ()
    {
        return min_pts_;
    }


private:
    virtual std::string getName () const { return "KdTree";  }

    boost::mutex m_lock_;

    /** \brief A FLANN index object. */
    FLANNIndex* flann_index_;

    /** \brief Internal pointer to data. */
    float* cloud_;

    /** \brief mapping between internal and external indices. */
    std::vector<int> index_mapping_;

    /** \brief whether the mapping bwwteen internal and external indices is identity */
    bool identity_mapping_;

    /** \brief Tree dimensionality (i.e. the number of dimensions per point). */
    int dim_;



protected:
    /** \brief The input point cloud dataset containing the points we need to use. */
    PointCloudConstPtr input_;

    /** \brief A pointer to the vector of point indices to use. */
    IndicesConstPtr indices_;

    /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
    double epsilon_;

    /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. */
    int min_pts_;

    /** \brief Return the radius search neighbours sorted **/
    bool sorted_;

    /** \brief For converting different point structures into k-dimensional vectors for nearest-neighbor search. */
    PointRepresentationConstPtr point_representation_;

    /** \brief Class getName method. */
    //      virtual std::string
    //    getName () const = 0;

  };

}

#endif

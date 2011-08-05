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

#ifndef PCL_KDTREE_KDTREE_FLANN_H_
#define PCL_KDTREE_KDTREE_FLANN_H_

#include <cstdio>
#include <pcl/kdtree/kdtree.h>
#include <boost/thread/mutex.hpp>

namespace flann
{
  template <typename Distance>
  class Index;

  template <class T>
  struct L2_Simple;
}

namespace pcl
{
  /** \brief @b KdTreeFLANN is a generic type of 3D spatial locator using kD-tree structures. The class is making use of
    * the FLANN (Fast Library for Approximate Nearest Neighbor) project by Marius Muja and David Lowe.
    *
    * \author Radu Bogdan Rusu, Marius Muja
    * \ingroup kdtree 
    */
  template <typename PointT>
  class KdTreeFLANN : public pcl::KdTree<PointT>
  {
    using KdTree<PointT>::input_;
    using KdTree<PointT>::indices_;
    using KdTree<PointT>::epsilon_;
    using KdTree<PointT>::sorted_;
    using KdTree<PointT>::point_representation_;

    typedef typename KdTree<PointT>::PointCloud PointCloud;
    typedef typename KdTree<PointT>::PointCloudConstPtr PointCloudConstPtr;

    typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
    typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

    typedef flann::Index <flann::L2_Simple<float> > FLANNIndex;

    public:
      // Boost shared pointers
      typedef boost::shared_ptr<KdTreeFLANN<PointT> > Ptr;
      typedef boost::shared_ptr<const KdTreeFLANN<PointT> > ConstPtr;

      /** \brief Default Constructor for KdTreeFLANN.
        * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise. 
        *
        * By setting sorted to false, the \ref radiusSearch operations will be faster.
        */
      KdTreeFLANN (bool sorted = true) : pcl::KdTree<PointT> (sorted), flann_index_(NULL), cloud_(NULL)
      {
        cleanup ();
      }

     /** \brief Copy constructor
       *
       * This copy constructor does shallow copy of the tree, the only reason
       * why it's needed is because boost::mutex is non-copyable, so the
       * default copy constructor would not work
       *
       * \param[in] tree the tree to copy
       */
      KdTreeFLANN (KdTreeFLANN& tree) : pcl::KdTree<PointT> (tree)
      {
        shallowCopy (tree);
      }

      inline Ptr makeShared () { return Ptr (new KdTreeFLANN<PointT> (*this)); } 

      KdTreeFLANN& operator= (const KdTreeFLANN& tree)
      {
        if (this != &tree) 
        {
          shallowCopy (tree);
        }
        return (*this);
      }


     /** \brief Perform a shallow copy of the tree.
       * \param[in] tree the tree to copy
       */
      inline void 
      shallowCopy (const KdTreeFLANN& tree)
      {
        flann_index_ = tree.flann_index_;
        cloud_ = tree.cloud_;
        index_mapping_ = tree.index_mapping_;
        dim_ = tree.dim_;
        sorted_ = tree.sorted_;
      }


      /** \brief Destructor for KdTreeFLANN. 
        * Deletes all allocated data arrays and destroys the kd-tree structures. 
        */
      virtual ~KdTreeFLANN ()
      {
        cleanup ();
      }

      /** \brief Provide a pointer to the input dataset.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
        */
      void 
      setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ());

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param[in] point the given query point
        * \param[in[ k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k 
        * a priori!)
        * \return number of neighbors found
        */
      int 
      nearestKSearch (const PointT &point, int k, 
                      std::vector<int> &k_indices, std::vector<float> &k_distances);

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param[in] cloud the point cloud data
        * \param[in] index the index in \a cloud representing the query point
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k 
        * a priori!)
        * \return number of neighbors found
        */
      inline int 
      nearestKSearch (const PointCloud &cloud, int index, int k, 
                      std::vector<int> &k_indices, std::vector<float> &k_distances)
      {
        if (index >= (int)cloud.points.size ())
          return (0);
        return (nearestKSearch (cloud.points[index], k, k_indices, k_distances));
      }

      /** \brief Search for k-nearest neighbors for the given query point (zero-copy).
        * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
        *        if indices were given in setInputCloud, index will be the position in the indices vector
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k 
        * a priori!)
        * \return number of neighbors found
        */
      inline int 
      nearestKSearch (int index, int k, 
                      std::vector<int> &k_indices, std::vector<float> &k_distances)
      {
        if (indices_ == NULL)
        {
          if (index >= (int)input_->points.size ())
            return (0);
          return (nearestKSearch (input_->points[index], k, k_indices, k_distances));
        }
        else
        {
          if (index >= (int)indices_->size())
            return (0);
          return (nearestKSearch (input_->points[(*indices_)[index]], k, k_indices, k_distances));
        }
      }

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param[in] point the given query point
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      int 
      radiusSearch (const PointT &point, double radius, std::vector<int> &k_indices,
                    std::vector<float> &k_distances, int max_nn = -1) const;

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param[in] cloud the point cloud data
        * \param[in] index the index in \a cloud representing the query point
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      inline int 
      radiusSearch (const PointCloud &cloud, int index, double radius, 
                    std::vector<int> &k_indices, std::vector<float> &k_distances, 
                    int max_nn = -1) const
      {
        if (index >= (int)cloud.points.size ())
          return 0;
        return (radiusSearch(cloud.points[index], radius, k_indices, k_distances, max_nn));
      }

      /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
        * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
        *        if indices were given in setInputCloud, index will be the position in the indices vector
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      inline int 
      radiusSearch (int index, double radius, std::vector<int> &k_indices,
                    std::vector<float> &k_distances, int max_nn = -1) const
      {
        if (indices_ == NULL)
        {
          if (index >= (int)input_->points.size ())
            return 0;
          return (radiusSearch (input_->points[index], radius, k_indices, k_distances, max_nn));
        }
        else
        {
          if (index >= (int)indices_->size ())
            return 0;
          return (radiusSearch (input_->points[(*indices_)[index]], radius, k_indices, k_distances, max_nn));
        }
      }

    private:
      /** \brief Internal cleanup method. */
      void cleanup ();

      /** \brief Simple initialization method for internal parameters. */
      bool initParameters ();

      /** \brief Simple initialization method for internal data buffers. */
      void initData ();

      /** \brief Converts a ROS PointCloud message to the internal FLANN point array representation. Returns the number
        * of points.
        * \param ros_cloud the ROS PointCloud message
        */
      void 
      convertCloudToArray (const PointCloud &ros_cloud);

      /** \brief Converts a PointCloud with a given set of indices to the internal FLANN point array
        * representation. Returns the number of points.
        * \param[in] cloud the PointCloud data
        * \param[in] indices the point cloud indices
       */
      void 
      convertCloudToArray (const PointCloud &cloud, const std::vector<int> &indices);

    private:
      /** \brief Class getName method. */
      virtual std::string 
      getName () const { return ("KdTreeFLANN"); }

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
  };
}

#endif

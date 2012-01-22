/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id: $
 */

#ifndef PCL_SEARCH_FLANN_SEARCH_H_
#define PCL_SEARCH_FLANN_SEARCH_H_

#include <pcl/search/search.h>
#include <pcl/common/time.h>
#include <pcl/point_representation.h>
//#include <flann/util/matrix.h>

namespace flann
{
  template<typename T> class NNIndex;
  template<typename T> class L2;
  template<typename T> class Matrix;
}

namespace pcl
{
  namespace search
  {

    /** \brief @b search::FlannSearch is a generic FLANN wrapper class for the new search interface.
      * It is able to wrap any FLANN index type, e.g. the kd tree as well as indices for high-dimensional
      * searches and intended as a more powerful and cleaner successor to KdTreeFlann.
      * THIS CODE IS NOT DONE YET! IT CONTAINS KNOWN UNHANDLED FAILURE CASES, AND IS CURRENTLY TO BE
      * CONSIDERED WORK-IN-PROGRESS!
      * \note this class depends on an un-released change in FLANN git, so it is not built by default.
      *
      * \author Andreas Muetzel
      * \ingroup search
      */
    template<typename PointT>
    class FlannSearch: public Search<PointT>
    {
      typedef typename Search<PointT>::PointCloud PointCloud;
      typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
      typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;
      typedef flann::NNIndex< flann::L2<float> > Index;
      typedef boost::shared_ptr<flann::NNIndex <flann::L2<float> > > IndexPtr;
      typedef boost::shared_ptr<flann::Matrix <float> > MatrixPtr;
      typedef boost::shared_ptr<const flann::Matrix <float> > MatrixConstPtr;

      typedef pcl::PointRepresentation<PointT> PointRepresentation;
      //typedef boost::shared_ptr<PointRepresentation> PointRepresentationPtr;
      typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;

      using Search<PointT>::input_;
      using Search<PointT>::indices_;

      public:
        typedef boost::shared_ptr<FlannSearch<PointT> > Ptr;
        typedef boost::shared_ptr<const FlannSearch<PointT> > ConstPtr;

        class FlannIndexCreator
        {
          public:
            virtual IndexPtr createIndex (MatrixConstPtr data)=0;
        };

        class KdTreeIndexCreator: public FlannIndexCreator
        {
          public:
            virtual IndexPtr createIndex (MatrixConstPtr data);
        };

        FlannSearch (FlannIndexCreator* creator = new KdTreeIndexCreator());

        /** \brief Destructor for KdTree. */
        virtual
        ~FlannSearch ();


        //void
        //setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ());

        /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
          * \param[in] eps precision (error bound) for nearest neighbors searches
          */
        inline void
        setEpsilon (double eps)
        {
          eps_ = eps;
        }

        /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
        inline double
        getEpsilon ()
        {
          return (eps_);
        }

        /** \brief Provide a pointer to the input dataset.
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          * \param[in] indices the point indices subset that is to be used from \a cloud
          */
        inline void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices);

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const;


        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          */
        virtual void
        nearestKSearch (const PointCloud& cloud, const std::vector<int>& indices, int k, 
                        std::vector< std::vector<int> >& k_indices, std::vector< std::vector<float> >& k_sqr_distances) const;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \return number of neighbors found in radius
          */
        int
        radiusSearch (const PointT& point, double radius, 
                      std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          */
        virtual void
        radiusSearch (const PointCloud& cloud, const std::vector<int>& indices, double radius, std::vector< std::vector<int> >& k_indices,
                std::vector< std::vector<float> >& k_sqr_distances, unsigned int max_nn=0) const;

        /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
          * \param[in] point_representation the const boost shared pointer to a PointRepresentation
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
          setInputCloud (input_, indices_);  // Makes sense in derived classes to reinitialize the tree
        }

        /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
        inline PointRepresentationConstPtr const
        getPointRepresentation ()
        {
          return (point_representation_);
        }

      protected:

        void convertInputToFlannMatrix();

        IndexPtr index_;
        FlannIndexCreator *creator_;
        MatrixPtr input_flann_;
        float eps_;
        bool input_copied_for_flann_;

        PointRepresentationConstPtr point_representation_;

        int dim_;

        std::vector<int> index_mapping_;
        bool identity_mapping_;

    };
  }
}

#define PCL_INSTANTIATE_FlannSearch(T) template class PCL_EXPORTS pcl::search::FlannSearch<T>;

#endif    // PCL_SEARCH_KDTREE_H_


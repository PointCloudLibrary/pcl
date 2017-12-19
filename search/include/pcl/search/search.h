/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 */

#ifndef PCL_SEARCH_SEARCH_H_
#define PCL_SEARCH_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/for_each_type.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/copy_point.h>

namespace pcl
{
  namespace search
  {
    namespace detail
    {
      //  Forward declaration of a helper class which helps specialize
      //  nearestKSearchT and radiusSearchT
      template<typename PointT, typename PointTDiff, typename Enabled = void>
      class SearchHelper;
    }

    /** \brief Generic search class. All search wrappers must inherit from this.
      *
      * Each search method must implement 2 different types of search:
      *   - \b nearestKSearch - search for K-nearest neighbors.
      *   - \b radiusSearch - search for all nearest neighbors in a sphere of a given radius
      *
      * The input to each search method can be given in 3 different ways:
      *   - as a query point
      *   - as a (cloud, index) pair
      *   - as an index
      *
      * For the latter option, it is assumed that the user specified the input
      * via a \ref setInputCloud () method first.
      *
      * \note In case of an error, all methods are supposed to return 0 as the number of neighbors found.
      *
      * \note libpcl_search deals with three-dimensional search problems. For higher
      * level dimensional search, please refer to the libpcl_kdtree module.
      *
      * \author Radu B. Rusu
      * \ingroup search
      */
    template<typename PointT>
    class Search
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<pcl::search::Search<PointT> > Ptr;
        typedef boost::shared_ptr<const pcl::search::Search<PointT> > ConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        /** Constructor. */
        Search (const std::string& name = "", bool sorted = false);

        /** Destructor. */
        virtual
        ~Search ()
        {
        }

        /** \brief Returns the search method name
          */
        virtual const std::string& 
        getName () const;

        /** \brief sets whether the results should be sorted (ascending in the distance) or not
          * \param[in] sorted should be true if the results should be sorted by the distance in ascending order.
          * Otherwise the results may be returned in any order.
          */
        virtual void 
        setSortedResults (bool sorted);

        /** \brief Gets whether the results should be sorted (ascending in the distance) or not
          * Otherwise the results may be returned in any order.
          */
        virtual bool 
        getSortedResults ();

        
        /** \brief Pass the input dataset that the search will be performed on.
          * \param[in] cloud a const pointer to the PointCloud data
          * \param[in] indices the point indices subset that is to be used from the cloud
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud, 
                       const IndicesConstPtr &indices = IndicesConstPtr ());

        /** \brief Get a pointer to the input point cloud dataset. */
        virtual PointCloudConstPtr
        getInputCloud () const
        {
          return (input_);
        }

        /** \brief Get a pointer to the vector of indices used. */
        virtual IndicesConstPtr
        getIndices () const
        {
          return (indices_);
        }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        virtual int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances) const = 0;

        /** \brief Search for k-nearest neighbors for the given query point.
          * This method accepts a different template parameter for the point type.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        template <typename PointTDiff> inline int
        nearestKSearchT (const PointTDiff &point, int k,
                         std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const
        {
          search::detail::SearchHelper<PointT, PointTDiff> search (*this);
          return search.nearestKSearchT (point, k, k_indices, k_sqr_distances);
        }

        /** \brief Search for k-nearest neighbors for the given query point.
          *
          * \attention This method does not do any bounds checking for the input index
          * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
          *
          * \param[in] cloud the point cloud data
          * \param[in] index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          *
          * \return number of neighbors found
          *
          * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
          */
        virtual int
        nearestKSearch (const PointCloud &cloud, int index, int k,
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_sqr_distances) const;

        /** \brief Search for k-nearest neighbors for the given query point (zero-copy).
          *
          * \attention This method does not do any bounds checking for the input index
          * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
          *
          * \param[in] index a \a valid index representing a \a valid query point in the dataset given
          * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in
          * the indices vector.
          *
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          *
          * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
          */
        virtual int
        nearestKSearch (int index, int k,
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_sqr_distances) const;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          */
        virtual void
        nearestKSearch (const PointCloud& cloud, const std::vector<int>& indices, 
                        int k, std::vector< std::vector<int> >& k_indices,
                        std::vector< std::vector<float> >& k_sqr_distances) const;

        /** \brief Search for the k-nearest neighbors for the given query point. Use this method if the query points are of a different type than the points in the data set (e.g. PointXYZRGBA instead of PointXYZ).
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \note This method copies the input point cloud of type PointTDiff to a temporary cloud of type PointT (if the types are different) and performs the batch
          * search on the new cloud. You should prefer the single-point search if you don't use a search algorithm that accelerates batch NN search.
          */
        template <typename PointTDiff> void
        nearestKSearchT (const pcl::PointCloud<PointTDiff> &cloud, const std::vector<int>& indices, int k, std::vector< std::vector<int> > &k_indices,
                         std::vector< std::vector<float> > &k_sqr_distances) const
        {
          const search::detail::SearchHelper<PointT, PointTDiff> search (*this);
          search.nearestKSearchT (cloud, indices, k, k_indices, k_sqr_distances);
        }

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
        virtual int
        radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_sqr_distances, unsigned int max_nn = 0) const = 0;

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
        template <typename PointTDiff> inline int
        radiusSearchT (const PointTDiff &point, double radius, std::vector<int> &k_indices,
                       std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
        {
          const search::detail::SearchHelper<PointT, PointTDiff> search (*this);
          return search.radiusSearchT (point, radius, k_indices, k_sqr_distances, max_nn);
        }

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          *
          * \attention This method does not do any bounds checking for the input index
          * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
          *
          * \param[in] cloud the point cloud data
          * \param[in] index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \return number of neighbors found in radius
          *
          * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
          */
        virtual int
        radiusSearch (const PointCloud &cloud, int index, double radius,
                      std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const;

        /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
          *
          * \attention This method does not do any bounds checking for the input index
          * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
          *
          * \param[in] index a \a valid index representing a \a valid query point in the dataset given
          * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in
          * the indices vector.
          *
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \return number of neighbors found in radius
          *
          * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
          */
        virtual int
        radiusSearch (int index, double radius, std::vector<int> &k_indices,
                      std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] indices the indices in \a cloud. If indices is empty, neighbors will be searched for all points.
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          */
        virtual void
        radiusSearch (const PointCloud& cloud,
                      const std::vector<int>& indices,
                      double radius,
                      std::vector< std::vector<int> >& k_indices,
                      std::vector< std::vector<float> > &k_sqr_distances,
                      unsigned int max_nn = 0) const;

        /** \brief Search for all the nearest neighbors of the query points in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \note This method copies the input point cloud of type PointTDiff to a temporary cloud of type PointT (if the point types are different) and performs the batch
          * search on the new cloud. You should prefer the single-point search if you don't use a search algorithm that accelerates batch NN search.
          */
        template <typename PointTDiff> void
        radiusSearchT (const pcl::PointCloud<PointTDiff> &cloud,
                       const std::vector<int>& indices,
                       double radius,
                       std::vector< std::vector<int> > &k_indices,
                       std::vector< std::vector<float> > &k_sqr_distances,
                       unsigned int max_nn = 0) const
        {
          const search::detail::SearchHelper<PointT, PointTDiff> search (*this);
          search.radiusSearchT (cloud, indices, radius, k_indices, k_sqr_distances, max_nn);
        }

      protected:
        void 
        sortResults (std::vector<int>& indices, std::vector<float>& distances) const;

        PointCloudConstPtr input_;
        IndicesConstPtr indices_;
        bool sorted_results_;
        std::string name_;
        
      private:
        struct Compare
        {
          Compare (const std::vector<float>& distances)
          : distances_ (distances)
          {
          }
          
          bool 
          operator () (int first, int second) const
          {
            return (distances_ [first] < distances_[second]);
          }

          const std::vector<float>& distances_;
        };
    }; // class Search

    namespace detail
    {
      /** \class SearchHelper
        * \brief A helper class whose specializations aids in preventing unnecessary
        * copies when using \a Search<PointT>::nearestKSearchT() and \a
        * Search<PointT>::radiusSearchT()
        *
        * This particular specialization is invoked when the the above search
        * methods are invoked with point types different than the ones used
        * in the underlying search structure.
        */
      template<typename PointT, typename PointTDiff>
      class SearchHelper<PointT, PointTDiff, typename boost::disable_if<boost::is_same<PointT, PointTDiff> >::type>
      {
        public:

          /** \brief Constructor
            * \param[in] search - A reference to a search object which implements
            * the nearestKSearchT and radiusSearchT methods.
            */
          SearchHelper (const Search<PointT>& search) : search (search) {}

          /** \brief Search for k-nearest neighbors for the given query point
            * of any type
            * \param[in] point - the given query point
            * \param[in] k - the number of neighbors to search for
            * \param[out] k_indices - the resultant indices of the neighboring
            * points (must be resized to \a k a priori!)
            * \param[out] k_sqr_distances - the resultant squared distances to
            * the neighboring points (must be resized to \a k a priori!)
            * \return number of neighbors found
            */
          int nearestKSearchT (const PointTDiff& point,
                               int k,
                               std::vector<int>& k_indices,
                               std::vector<float>& k_sqr_distances) const
          {
            PointT p;
            copyPoint (point, p);
            return search.nearestKSearch (p, k, k_indices, k_sqr_distances);
          }

          /** \brief Search for the k-nearest neighbors for all points in a
            * given point cloud. The point cloud can be of any point type which
            * includes XYZ coordinates (e.g. PointXYZRGBA instead of PointXYZ).
            * \param[in] cloud - the point cloud data
            * \param[in] indices - a vector of point cloud indices to query for
            * nearest neighbors
            * \param[in] k - the number of neighbors to search for
            * \param[out] k_indices - the resultant indices of the neighboring
            * points, k_indices[i] corresponds to the neighbors of the query
            * point i
            * \param[out] k_sqr_distances - the resultant squared distances to
            * the neighboring points, k_sqr_distances[i] corresponds to the
            * neighbors of the query point i
            */
          void nearestKSearchT (const pcl::PointCloud<PointTDiff>& cloud,
                                const std::vector<int>& indices,
                                int k,
                                std::vector<std::vector<int> >& k_indices,
                                std::vector<std::vector<float> >& k_sqr_distances) const
          {
            const pcl::PointCloud<PointT> pc = copyPointCloud (cloud, indices);
            search.nearestKSearch (pc, std::vector<int> (), k, k_indices, k_sqr_distances);
          }

          /** \brief Search for all the nearest neighbors of the query point in
            * a given radius.
            * \param[in] point - the given query point
            * \param[in] radius - the radius of the sphere bounding all of p_q's
            * neighbors
            * \param[out] k_indices - the resultant indices of the neighboring points
            * \param[out] k_sqr_distances - the resultant squared distances to
            * the neighboring points
            * \param[in] max_nn - if given, bounds the maximum returned neighbors
            * to this value. If \a max_nn is set to 0 or to a number higher than
            * the number of points in the input cloud, all neighbors in \a radius
            * will be returned.
            * \return number of neighbors found in radius
            */
          int radiusSearchT (const PointTDiff& point,
                             double radius,
                             std::vector<int>& k_indices,
                             std::vector<float>& k_sqr_distances,
                             unsigned int max_nn = 0) const
          {
            PointT p;
            copyPoint (point, p);
            return search.radiusSearch (p, radius, k_indices, k_sqr_distances, max_nn);
          }

          /** \brief Search for all the nearest neighbors for all points in a
            * given point cloud, in a given radius.
            * \param[in] cloud - the point cloud data
            * \param[in] indices - a vector of point cloud indices to query for
            * nearest neighbors
            * \param[in] radius - the radius of the sphere bounding all of p_q's
            * neighbors
            * \param[out] k_indices - the resultant indices of the neighboring
            * points, k_indices[i] corresponds to the neighbors of the query point i
            * \param[out] k_sqr_distances - the resultant squared distances to the
            * neighboring points, k_sqr_distances[i] corresponds to the neighbors
            * of the query point i
            * \param[in] max_nn - if given, bounds the maximum returned neighbors
            * to this value. If \a max_nn is set to 0 or to a number higher than
            * the number of points in the input cloud, all neighbors in \a radius
            * will be returned.
            */
          void radiusSearchT (const pcl::PointCloud<PointTDiff>& cloud,
                              const std::vector<int>& indices,
                              double radius,
                              std::vector<std::vector<int> >& k_indices,
                              std::vector<std::vector<float> >& k_sqr_distances,
                              unsigned int max_nn = 0) const
          {
            const pcl::PointCloud<PointT> pc = copyPointCloud (cloud, indices);
            radiusSearch (pc, std::vector<int> (), radius, k_indices, k_sqr_distances, max_nn);
          }

        protected:

          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;

          /** \brief Creates a point cloud of point type PointT from a given point
            * cloud of point type PointTDiff, copying every common existing fields
            * between the two.
            * \param[in] cloud - the original point cloud.
            * \param[in] indices - a vector of indices masking the points of interest
            * in \a cloud.
            * \return A copy of \a cloud with point type PointT.
            */
          static pcl::PointCloud<PointT>
          copyPointCloud (const pcl::PointCloud<PointTDiff>& cloud, const std::vector<int>& indices)
          {
            // Copy all the data fields from the input cloud to the output one
            pcl::PointCloud<PointT> pc;
            if (indices.empty ())
            {
              pc.resize (cloud.size ());
              for (size_t i = 0; i < cloud.size (); ++i)
                pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (cloud[i], pc[i]));
            }
            else
            {
              pc.resize (indices.size ());
              for (size_t i = 0; i < indices.size (); ++i)
                pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (cloud[indices[i]], pc[i]));
            }
            return pc;
          }

          /** \brief A reference to a search object, with its index already built
            * which implements nearestKSearchT and radiusSearchT
            */
          const Search<PointT>& search;
      };

      /** \class SearchHelper
        * \brief A helper class whose specializations aids in preventing unnecessary
        * copies when using \a Search<PointT>::nearestKSearchT() and \a
        * Search<PointT>::radiusSearchT()
        *
        * This particular specialization is invoked when the the above search
        * methods are invoked with the same point types as the underlying
        * search structure.
        */
      template<typename PointT, typename PointTDiff>
      class SearchHelper<PointT, PointTDiff, typename boost::enable_if<boost::is_same<PointT, PointTDiff> >::type>
      {
        public:

          /** \brief Constructor
            * \param[in] search - A reference to a search object which implements
            * the nearestKSearchT and radiusSearchT methods.
            */
          SearchHelper (const Search<PointT>& search) : search (search) {}

          /** \brief Search for k-nearest neighbors for the given query point
            * of any type
            * \param[in] point - the given query point
            * \param[in] k - the number of neighbors to search for
            * \param[out] k_indices - the resultant indices of the neighboring
            * points (must be resized to \a k a priori!)
            * \param[out] k_sqr_distances - the resultant squared distances to
            * the neighboring points (must be resized to \a k a priori!)
            * \return number of neighbors found
            */
          int nearestKSearchT (const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const
          {
            return search.nearestKSearch (point, k, k_indices, k_sqr_distances);
          }

          /** \brief Search for the k-nearest neighbors for all points in a
            * given point cloud. The point cloud can be of any point type which
            * includes XYZ coordinates (e.g. PointXYZRGBA instead of PointXYZ).
            * \param[in] cloud - the point cloud data
            * \param[in] indices - a vector of point cloud indices to query for
            * nearest neighbors
            * \param[in] k - the number of neighbors to search for
            * \param[out] k_indices - the resultant indices of the neighboring
            * points, k_indices[i] corresponds to the neighbors of the query
            * point i
            * \param[out] k_sqr_distances - the resultant squared distances to
            * the neighboring points, k_sqr_distances[i] corresponds to the
            * neighbors of the query point i
            */
          void nearestKSearchT (const pcl::PointCloud<PointTDiff>& cloud,
                                const std::vector<int>& indices,
                                int k,
                                std::vector<std::vector<int> >& k_indices,
                                std::vector<std::vector<float> >& k_sqr_distances) const
          {
            search.nearestKSearch (cloud, indices, k, k_indices, k_sqr_distances);
          }

          /** \brief Search for all the nearest neighbors of the query point in
            * a given radius.
            * \param[in] point - the given query point
            * \param[in] radius - the radius of the sphere bounding all of p_q's
            * neighbors
            * \param[out] k_indices - the resultant indices of the neighboring points
            * \param[out] k_sqr_distances - the resultant squared distances to
            * the neighboring points
            * \param[in] max_nn - if given, bounds the maximum returned neighbors
            * to this value. If \a max_nn is set to 0 or to a number higher than
            * the number of points in the input cloud, all neighbors in \a radius
            * will be returned.
            * \return number of neighbors found in radius
            */
          int radiusSearchT (const PointTDiff& point,
                             double radius,
                             std::vector<int>& k_indices,
                             std::vector<float>& k_sqr_distances,
                             unsigned int max_nn = 0) const
          {
            return search.radiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);
          }

          /** \brief Search for all the nearest neighbors for all points in a
            * given point cloud, in a given radius.
            * \param[in] cloud - the point cloud data
            * \param[in] indices - a vector of point cloud indices to query for
            * nearest neighbors
            * \param[in] radius - the radius of the sphere bounding all of p_q's
            * neighbors
            * \param[out] k_indices - the resultant indices of the neighboring
            * points, k_indices[i] corresponds to the neighbors of the query point i
            * \param[out] k_sqr_distances - the resultant squared distances to the
            * neighboring points, k_sqr_distances[i] corresponds to the neighbors
            * of the query point i
            * \param[in] max_nn - if given, bounds the maximum returned neighbors
            * to this value. If \a max_nn is set to 0 or to a number higher than
            * the number of points in the input cloud, all neighbors in \a radius
            * will be returned.
            */
          void radiusSearchT (const pcl::PointCloud<PointTDiff>& cloud,
                              const std::vector<int>& indices,
                              double radius,
                              std::vector<std::vector<int> >& k_indices,
                              std::vector<std::vector<float> >& k_sqr_distances,
                              unsigned int max_nn = 0) const
          {
            search.radiusSearch (cloud, indices, radius, k_indices, k_sqr_distances, max_nn);
          }

        protected:

          /** \brief A reference to a search object, with its index already built
            * which implements nearestKSearchT and radiusSearchT
            */
          const Search<PointT>& search;
      };
    } // namespace detail
  } // namespace search
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/search/impl/search.hpp>
#endif

#endif  //#ifndef _PCL_SEARCH_SEARCH_H_

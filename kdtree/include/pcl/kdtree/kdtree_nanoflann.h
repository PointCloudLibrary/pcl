//
// KdTreeNanoFLANN
// PeopleTracker
//
// Created by Francisco Facioni on 5/13/13.
// Copyright (c) 2013 Agile Route. All rights reserved.
//


#ifndef PCL_KDTREE_KDTREE_NANO_FLANN_H_
#define PCL_KDTREE_KDTREE_NANO_FLANN_H_

#include <pcl/kdtree/kdtree.h>

// Forward declarations
namespace nanoflann
{
  struct SearchParams;
  template <class T, class DataSource, typename _DistanceType> struct L2_Simple_Adaptor;
  template <typename Distance, class DatasetAdaptor, int DIM, typename IndexType> struct KDTreeSingleIndexAdaptor;
}

namespace pcl
{
    template <typename PointCloud>
    struct PointCloudAdaptor;

    /** \brief KdTreeNanoFLANN is a generic type of 3D spatial locator using kD-tree structures. The class is making use of
      * the Nano FLANN (Nano Fast Library for Approximate Nearest Neighbor, branch of FLANN) project by Jose Luis Blanco.
      *
      * \author Jose Luis Blanco
      * \ingroup kdtree
      */
    template <typename PointT>
    class KdTreeNanoFLANN : public KdTree<PointT>{
        public:
            using KdTree<PointT>::input_;
            using KdTree<PointT>::indices_;
            using KdTree<PointT>::epsilon_;
            using KdTree<PointT>::sorted_;
            using KdTree<PointT>::point_representation_;
            using KdTree<PointT>::nearestKSearch;
            using KdTree<PointT>::radiusSearch;
    
            typedef typename KdTree<PointT>::PointCloud PointCloud;
            typedef typename KdTree<PointT>::PointCloudPtr PointCloudPtr;
            typedef typename KdTree<PointT>::PointCloudConstPtr PointCloudConstPtr;

            typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
            typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

            typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor<PointCloud>, float>,
                PointCloudAdaptor<PointCloud>,
                3,
                int
            > FLANNIndex;
    
            // Boost shared pointers
            typedef boost::shared_ptr<KdTreeNanoFLANN<PointT> > Ptr;
            typedef boost::shared_ptr<const KdTreeNanoFLANN<PointT> > ConstPtr;
    
            /** \brief Default Constructor for KdTreeNanoFLANN.
              * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise.
              *
              * By setting sorted to false, the \ref radiusSearch operations will be faster.
              */
            KdTreeNanoFLANN (bool sorted = true);
    
            /** \brief Copy constructor
              * \param[in] tree the tree to copy into this
              */
            KdTreeNanoFLANN (const KdTreeNanoFLANN<PointT> &k);
    
            /** \brief Copy operator
              * \param[in] tree the tree to copy into this
              */
            inline KdTreeNanoFLANN<PointT>&
            operator = (const KdTreeNanoFLANN<PointT>& k)
            {
                KdTree<PointT>::operator=(k);
                flann_index_ = k.flann_index_;
                index_mapping_ = k.index_mapping_;
                identity_mapping_ = k.identity_mapping_;
                dim_ = k.dim_;
                total_nr_points_ = k.total_nr_points_;
                param_k_ = k.param_k_;
                param_radius_ = k.param_radius_;
                return (*this);
            }
    
            /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
              * \param[in] eps precision (error bound) for nearest neighbors searches
              */
            void
            setEpsilon (float eps);
    
            void
            setSortedResults (bool sorted);
    
            inline Ptr makeShared () { return Ptr (new KdTreeNanoFLANN<PointT> (*this)); }
    
            /** \brief Provide a pointer to the input dataset.
              * \param[in] cloud the const boost shared pointer to a PointCloud message
              * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
              */
            void
            setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ());
    
            /** \brief Search for k-nearest neighbors for the given query point.
              *
              * \attention This method does not do any bounds checking for the input index
              * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
              *
              * \param[in] point a given \a valid (i.e., finite) query point
              * \param[in] k the number of neighbors to search for
              * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
              * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
              * a priori!)
              * \return number of neighbors found
              *
              * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
              */
            int
            nearestKSearch (const PointT &point, int k,
            std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const;
    
            /** \brief Search for all the nearest neighbors of the query point in a given radius.
              *
              * \attention This method does not do any bounds checking for the input index
              * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
              *
              * \param[in] point a given \a valid (i.e., finite) query point
              * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
              * \param[out] k_indices the resultant indices of the neighboring points
              * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
              * \return number of neighbors found in radius
              *
              * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
              */
            int
            radiusSearch (const PointT &point, double radius, std::vector<int> &k_indices,
            std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;
    
        private:

            /** \brief Class getName method. */
            virtual std::string
            getName () const { return ("KdTreeNanoFLANN"); }
    
            /** \brief Tree dimensionality (i.e. the number of dimensions per point). */
            int dim_;
    
            /** \brief The total size of the data (either equal to the number of points in the input cloud or to the number of indices - if passed). */
            int total_nr_points_;

            /** \brief mapping between internal and external indices. */
            std::vector<int> index_mapping_;

            /** \brief whether the mapping bwwteen internal and external indices is identity */
            bool identity_mapping_;

            boost::shared_ptr< PointCloudAdaptor<PointCloud> > data_;

            /** \brief A FLANN index object. */
            boost::shared_ptr<FLANNIndex> flann_index_;

            /** \brief The KdTree search parameters for K-nearest neighbors. */
            boost::shared_ptr<nanoflann::SearchParams> param_k_;

            /** \brief The KdTree search parameters for radius search. */
            boost::shared_ptr<nanoflann::SearchParams> param_radius_;
    };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/kdtree/impl/kdtree_nanoflann.hpp>
#endif

#endif //PCL_KDTREE_KDTREE_NANO_FLANN_H_

#ifndef PCL_TRACKING_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/search.h>

#include <pcl/tracking/coherence.h>

namespace pcl
{
  namespace tracking
  {
    /** \brief @b NearestPairPointCloudCoherence computes coherence between two pointclouds using the
         nearest point pairs.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class NearestPairPointCloudCoherence: public PointCloudCoherence<PointInT>
    {
      public:
        using PointCloudCoherence<PointInT>::getClassName;
        using PointCloudCoherence<PointInT>::coherence_name_;
        using PointCloudCoherence<PointInT>::target_input_;
        
        typedef typename PointCloudCoherence<PointInT>::PointCoherencePtr PointCoherencePtr;
        typedef typename PointCloudCoherence<PointInT>::PointCloudInConstPtr PointCloudInConstPtr;
        typedef PointCloudCoherence<PointInT> BaseClass;
        
        typedef boost::shared_ptr<NearestPairPointCloudCoherence<PointInT> > Ptr;
        typedef boost::shared_ptr<const NearestPairPointCloudCoherence<PointInT> > ConstPtr;
        typedef boost::shared_ptr<pcl::search::Search<PointInT> > SearchPtr;
        typedef boost::shared_ptr<const pcl::search::Search<PointInT> > SearchConstPtr;
        
        /** \brief empty constructor */
        NearestPairPointCloudCoherence ()
          : new_target_ (false)
          , search_ ()
          , maximum_distance_ (std::numeric_limits<double>::max ())
        {
          coherence_name_ = "NearestPairPointCloudCoherence";
        }

        /** \brief Provide a pointer to a dataset to add additional information
         * to estimate the features for every point in the input dataset.  This
         * is optional, if this is not set, it will only use the data in the
         * input cloud to estimate the features.  This is useful when you only
         * need to compute the features for a downsampled cloud.  
         * \param search a pointer to a PointCloud message
         */
        inline void 
        setSearchMethod (const SearchPtr &search) { search_ = search; }
        
        /** \brief Get a pointer to the point cloud dataset. */
        inline SearchPtr 
        getSearchMethod () { return (search_); }

        /** \brief add a PointCoherence to the PointCloudCoherence.
          * \param[in] cloud coherence a pointer to PointCoherence.
          */
        virtual inline void
        setTargetCloud (const PointCloudInConstPtr &cloud)
        {
          new_target_ = true;
          PointCloudCoherence<PointInT>::setTargetCloud (cloud);
        }
        
        /** \brief set maximum distance to be taken into account.
          * \param[in] val maximum distance.
          */
        inline void setMaximumDistance (double val) { maximum_distance_ = val; }

      protected:
        using PointCloudCoherence<PointInT>::point_coherences_;

        /** \brief This method should get called before starting the actual computation. */
        virtual bool initCompute ();

        /** \brief A flag which is true if target_input_ is updated */
        bool new_target_;
        
        /** \brief A pointer to the spatial search object. */
        SearchPtr search_;

        /** \brief max of distance for points to be taken into account*/
        double maximum_distance_;
        
        /** \brief compute the nearest pairs and compute coherence using point_coherences_ */
        virtual void
        computeCoherence (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w_j);

    };
  }
}

// #include <pcl/tracking/impl/nearest_pair_point_cloud_coherence.hpp>
#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/nearest_pair_point_cloud_coherence.hpp>
#endif

#endif

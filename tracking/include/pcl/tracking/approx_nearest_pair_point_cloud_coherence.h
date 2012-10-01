#ifndef PCL_TRACKING_APPROX_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_APPROX_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/search.h>
#include <pcl/search/octree.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
namespace pcl
{
  namespace tracking
  {
    /** \brief @b ApproxNearestPairPointCloudCoherence computes coherence between two pointclouds using the
         approximate nearest point pairs.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class ApproxNearestPairPointCloudCoherence: public NearestPairPointCloudCoherence<PointInT>
    {
    public:
      typedef typename NearestPairPointCloudCoherence<PointInT>::PointCoherencePtr PointCoherencePtr;
      typedef typename NearestPairPointCloudCoherence<PointInT>::PointCloudInConstPtr PointCloudInConstPtr;
      //using NearestPairPointCloudCoherence<PointInT>::search_;
      using NearestPairPointCloudCoherence<PointInT>::maximum_distance_;
      using NearestPairPointCloudCoherence<PointInT>::target_input_;
      using NearestPairPointCloudCoherence<PointInT>::point_coherences_;
      using NearestPairPointCloudCoherence<PointInT>::coherence_name_;
      using NearestPairPointCloudCoherence<PointInT>::new_target_;
      using NearestPairPointCloudCoherence<PointInT>::getClassName;
      
      /** \brief empty constructor */
      ApproxNearestPairPointCloudCoherence () : 
        NearestPairPointCloudCoherence<PointInT> (), search_ ()
      {
        coherence_name_ = "ApproxNearestPairPointCloudCoherence";
      }
      
    protected:
      /** \brief This method should get called before starting the actual computation. */
      virtual bool initCompute ();
      
      /** \brief compute the nearest pairs and compute coherence using point_coherences_ */
      virtual void
      computeCoherence (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w_j);

      typename boost::shared_ptr<pcl::search::Octree<PointInT> > search_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/approx_nearest_pair_point_cloud_coherence.hpp>
#endif

#endif


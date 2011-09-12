#ifndef PCL_TRACKING_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include "pcl/tracking/coherence.h"

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
      using PointCloudCoherence<PointInT>::deinitCompute;
      using PointCloudCoherence<PointInT>::getClassName;
      using PointCloudCoherence<PointInT>::coherence_name_;
      using PointCloudCoherence<PointInT>::indices_;
      using PointCloudCoherence<PointInT>::input_;
      using PointCloudCoherence<PointInT>::target_input_;
      
      typedef PointCloudCoherence<PointInT> BaseClass;
      
      typedef boost::shared_ptr< NearestPairPointCloudCoherence<PointInT> > Ptr;
      typedef boost::shared_ptr< const NearestPairPointCloudCoherence<PointInT> > ConstPtr;
      typedef typename pcl::KdTree<PointInT> KdTree;
      typedef typename pcl::KdTree<PointInT>::Ptr KdTreePtr;
      
      /** \brief empty constructor */
      NearestPairPointCloudCoherence (): tree_ ()
      {
        coherence_name_ = "NearestPairPointCloudCoherence";
      }

      /** \brief Provide a pointer to a dataset to add additional information
       * to estimate the features for every point in the input dataset.  This
       * is optional, if this is not set, it will only use the data in the
       * input cloud to estimate the features.  This is useful when you only
       * need to compute the features for a downsampled cloud.  
       * \param cloud a pointer to a PointCloud message
       */
      inline void 
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }
      
      /** \brief Get a pointer to the point cloud dataset. */
      inline KdTreePtr 
      getSearchMethod () { return (tree_); }
      
    protected:
      using PointCloudCoherence<PointInT>::point_coherences_;

      /** \brief This method should get called before starting the actual computation. */
      virtual bool initCompute ();
      
      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;
      
      /** \brief compute the nearest pairs and compute coherence using point_coherences_ */
      virtual inline double
      computeCoherence ();
      
    };
  }
}

#include <pcl/tracking/impl/nearest_pair_point_cloud_coherence.hpp>

#endif

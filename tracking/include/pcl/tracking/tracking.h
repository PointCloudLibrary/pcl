#ifndef PCL_TRACKING_TRACKING_H_
#define PCL_TRACKING_TRACKING_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/organized_data.h"


namespace pcl
{
  namespace tracking
  {
    /* state definition */
    struct ParticleXYZRPY;

    
    /** \brief @b Tracker represents the base tracker class.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT, typename StateT>
    class Tracker: public PCLBase<PointInT>
    {
    protected:
      using PCLBase<PointInT>::deinitCompute;
      
    public:
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::input_;
      
      typedef PCLBase<PointInT> BaseClass;
      typedef boost::shared_ptr< Tracker<PointInT, StateT> > Ptr;
      typedef boost::shared_ptr< const Tracker<PointInT, StateT> > ConstPtr;
      
      typedef typename pcl::KdTree<PointInT> KdTree;
      typedef typename pcl::KdTree<PointInT>::Ptr KdTreePtr;
      
      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      
      typedef pcl::PointCloud<StateT> PointCloudState;
      typedef typename PointCloudState::Ptr PointCloudStatePtr;
      typedef typename PointCloudState::ConstPtr PointCloudStateConstPtr;
      
    public:
      /** \brief Empty constructor. */
      Tracker (): tree_ () {}
      
      /** \brief Base method for tracking for all points given in 
        * <setInputCloud (), setIndices ()> using the indices in setIndices () 
        */
      void 
      compute ();
      
    protected:
      /** \brief The tracker name. */
      std::string tracker_name_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string& 
      getClassName () const { return (tracker_name_); }

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

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
      
      /** \brief Get an instance of the result of tracking. */
      virtual StateT getResult () = 0;
      
    private:
      /** \brief Abstract tracking method. */
      virtual void
      computeTracking () = 0;
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#include <pcl/tracking/impl/tracking.hpp>

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::tracking::_ParticleXYZRPY,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYZRPY, pcl::tracking::_ParticleXYZRPY)


#endif

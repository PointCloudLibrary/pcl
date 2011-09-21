#ifndef PCL_TRACKING_TRACKER_H_
#define PCL_TRACKING_TRACKER_H_

#include "pcl/tracking/tracking.h"

namespace pcl
{
  namespace tracking
  {
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

      typedef boost::shared_ptr<pcl::search::Search<PointInT> > SearchPtr;
      typedef boost::shared_ptr<const pcl::search::Search<PointInT> > SearchConstPtr;
            
      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      
      typedef pcl::PointCloud<StateT> PointCloudState;
      typedef typename PointCloudState::Ptr PointCloudStatePtr;
      typedef typename PointCloudState::ConstPtr PointCloudStateConstPtr;
      
    public:
      /** \brief Empty constructor. */
      Tracker (): search_ () {}
      
      /** \brief Base method for tracking for all points given in 
        * <setInputCloud (), setIndices ()> using the indices in setIndices () 
        */
      void 
      compute ();
      
    protected:
      /** \brief The tracker name. */
      std::string tracker_name_;

      /** \brief A pointer to the spatial search object. */
      SearchPtr search_;

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
      setSearchMethod (const SearchPtr &search) { search_ = search; }

      /** \brief Get a pointer to the point cloud dataset. */
      inline SearchPtr 
      getSearchMethod () { return (search_); }
      
      /** \brief Get an instance of the result of tracking. */
      virtual StateT 
      getResult () const = 0;
      
    private:
      /** \brief Abstract tracking method. */
      virtual void
      computeTracking () = 0;
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#include "pcl/tracking/impl/tracker.hpp"

#endif

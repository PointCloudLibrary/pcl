#ifndef PCL_TRACKING_COHERENCE_H_
#define PCL_TRACKING_COHERENCE_H_

namespace pcl
{

  namespace tracking
  {

    /** \brief @b PointCoherence is a base class to compute coherence between the two points.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class PointCoherence
    {
    public:
      typedef boost::shared_ptr< PointCoherence<PointInT> > Ptr;
      typedef boost::shared_ptr< const PointCoherence<PointInT> > ConstPtr;
      
    public:
      /** \brief empty constructor */
      PointCoherence () {}
      
      /** \brief empty distructor */
      ~PointCoherence () {}

      /** \brief compute coherence from the source point to the target point.
        * \param source instance of source point.
        * \param target instance of target point.
        */
      inline double
      compute (PointInT &source, PointInT &target);

    protected:

      /** \brief The coherence name. */
      std::string coherence_name_;

      /** \brief abstract method to calculate coherence.
        * \param source instance of source point.
        * \param target instance of target point.
        */
      virtual double computeCoherence (PointInT &source, PointInT &target) = 0;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string& 
      getClassName () const { return (coherence_name_); }

    };

    /** \brief @b PointCloudCoherence is a base class to compute coherence between the two PointClouds.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class PointCloudCoherence: public PCLBase<PointInT>
    {
    protected:
      using PCLBase<PointInT>::deinitCompute;

    public:
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::input_;

      typedef PCLBase<PointInT> BaseClass;
      typedef boost::shared_ptr< PointCloudCoherence<PointInT> > Ptr;
      typedef boost::shared_ptr< const PointCloudCoherence<PointInT> > ConstPtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      
      typedef typename PointCoherence<PointInT>::Ptr PointCoherencePtr;
      
      /** \brief empty constructor */
      PointCloudCoherence () {}

      /** \brief compute coherence between two pointclouds. */
      inline double
      compute ();

      /** \brief get a list of pcl::tracking::PointCoherence.*/
      inline std::vector<PointCoherencePtr>
      getPointCoherences () { return point_coherences_; }

      /** \brief set a list of pcl::tracking::PointCoherence.
        * \param coherences a list of pcl::tracking::PointCoherence.
        */
      inline void
      setPointCoherences (std::vector<PointCoherencePtr> coherences) { point_coherences_ = coherences; }

      /** \brief add a PointCoherence to the PointCloudCoherence.
        * \param coherence a pointer to PointCoherence.
        */
      inline void
      addPointCoherence (PointCoherencePtr coherence) { point_coherences_.push_back (coherence); }

      /** \brief add a PointCoherence to the PointCloudCoherence.
        * \param coherence a pointer to PointCoherence.
        */
      virtual inline void
      setTargetCloud (const PointCloudInConstPtr &cloud)  { target_input_ = cloud; }
      
    protected:
      /** \brief Abstract method to compute coherence. */
      virtual double
      computeCoherence () = 0;

      inline double calcPointCoherence (PointInT &source, PointInT &target);
      
      /** \brief Get a string representation of the name of this class. */
      inline const std::string& 
      getClassName () const { return (coherence_name_); }
      
      /** \brief This method should get called before starting the actual computation. */
      virtual bool initCompute ();
      
      /** \brief The coherence name. */
      std::string coherence_name_;

      /** \brief a pointer to target point cloud*/
      PointCloudInConstPtr target_input_;

      /** \brief a list of pointers to PointCoherence.*/
      std::vector<PointCoherencePtr> point_coherences_;
    };
    
  }
}


#include "pcl/tracking/impl/coherence.hpp"


#endif

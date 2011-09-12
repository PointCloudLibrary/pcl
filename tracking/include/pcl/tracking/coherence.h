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
      
      /** \brief empty constructor */
      PointCloudCoherence () {}

      /** \brief empty distructor */
      ~PointCloudCoherence () {}

      
      /** \brief compute coherence between two pointclouds. */
      inline double
      compute ();
      
    protected:
      /** \brief Abstract method to compute coherence. */
      virtual inline double
      computeCoherence () = 0;
      
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
      std::vector<typename PointCoherence<PointInT>::Ptr> point_coherences_;
    };
    
  }
}


#include "pcl/tracking/impl/coherence.hpp"


#endif

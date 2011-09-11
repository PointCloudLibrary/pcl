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
    protected:
      
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
  }
}


#include "pcl/tracking/impl/coherence.hpp"


#endif

#pragma once

#include <pcl/memory.h>
#include <pcl/tracking/coherence.h>


namespace pcl
{
  namespace tracking
  {
    /** \brief @b DistanceCoherence computes coherence between two points from the distance
        between them. the coherence is calculated by 1 / (1 + weight * d^2 ).
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class DistanceCoherence: public PointCoherence<PointInT>
    {
    public:

      using Ptr = shared_ptr<DistanceCoherence<PointInT> >;
      using ConstPtr = shared_ptr<const DistanceCoherence<PointInT>>;

      /** \brief initialize the weight to 1.0. */
      DistanceCoherence ()
      : PointCoherence<PointInT> ()
      , weight_ (1.0)
      {}

      /** \brief set the weight of coherence.
        * \param weight the value of the wehgit.
        */
      inline void setWeight (double weight) { weight_ = weight; }

      /** \brief get the weight of coherence.*/
      inline double getWeight () { return weight_; }
      
    protected:

      /** \brief return the distance coherence between the two points.
        * \param source instance of source point.
        * \param target instance of target point.
        */
      double computeCoherence (PointInT &source, PointInT &target) override;

      /** \brief the weight of coherence.*/
      double weight_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/distance_coherence.hpp>
#endif

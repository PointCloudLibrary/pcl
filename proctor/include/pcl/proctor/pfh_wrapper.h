#ifndef PFH_WRAPPER_H
#define PFH_WRAPPER_H

#include "proctor/feature_wrapper.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/export.hpp>

namespace pcl
{
  namespace proctor
  {

    class PFHWrapper : public FeatureWrapper {
      public:

        PFHWrapper() : FeatureWrapper("pfh")
        {}

        void
        compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output);

      private:
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
          ar & boost::serialization::base_object<FeatureWrapper>( *this );
        }
    };

  }
}

BOOST_CLASS_EXPORT(pcl::proctor::PFHWrapper)

#endif

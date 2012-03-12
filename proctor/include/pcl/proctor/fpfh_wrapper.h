#ifndef FPFH_WRAPPER_H
#define FPFH_WRAPPER_H

#include "proctor/feature_wrapper.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/export.hpp>

namespace pcl
{
  namespace proctor
  {

    class FPFHWrapper : public FeatureWrapper {
      public:

        FPFHWrapper() : FeatureWrapper("fpfh")
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

//BOOST_CLASS_EXPORT(pcl::proctor::FPFHWrapper)

#endif

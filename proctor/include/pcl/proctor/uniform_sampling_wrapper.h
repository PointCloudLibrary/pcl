#ifndef UNIFORM_SAMPLING_WRAPPER_H
#define UNIFORM_SAMPLING_WRAPPER_H

#include "proctor/keypoint_wrapper.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

namespace pcl
{
  namespace proctor
  {

    class UniformSamplingWrapper : public KeypointWrapper {
      public:
        UniformSamplingWrapper() : KeypointWrapper("uniform sampling")
        {}

        void
        compute(PointCloudInPtr input, PointCloudOut &output);

      private:
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
          ar & boost::serialization::base_object<KeypointWrapper>( *this );
        }
    };

  }
}

BOOST_CLASS_EXPORT_GUID(pcl::proctor::UniformSamplingWrapper, "UniformSamplingWrapper")

#endif

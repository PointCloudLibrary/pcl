#ifndef FRAME_HOUGH_PROPOSER_H
#define FRAME_HOUGH_PROPOSER_H

#include "proctor/hough_proposer.h"

#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/features/principal_curvatures.h"

namespace pcl
{
  namespace proctor
  {

    class FrameHoughProposer : public HoughProposer {
      public:
        typedef boost::shared_ptr<FrameHoughProposer> Ptr;
        typedef boost::shared_ptr<const FrameHoughProposer> ConstPtr;

        //typedef boost::multi_array<double, 3> bin_t;

        FrameHoughProposer(Detector *detector = NULL) : HoughProposer(detector)
        {}

        virtual void
        houghVote(Entry &query, Entry &target, bin_t& bins);

        Eigen::Matrix3f
        getTransformationBetweenFrames(Eigen::Vector3f x_from, Eigen::Vector3f y_from, Eigen::Vector3f x_to, Eigen::Vector3f y_to);

        Eigen::Vector3f
        getVectorCurvatureMap(PrincipalCurvatures p);

        PointCloud<PrincipalCurvatures>::Ptr
        computeCurvatures(Entry &e);
      private:
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
          ar & boost::serialization::base_object<HoughProposer>( *this );
        }
    };

  }
}

//BOOST_CLASS_EXPORT(pcl::proctor::FrameHoughProposer)

#endif

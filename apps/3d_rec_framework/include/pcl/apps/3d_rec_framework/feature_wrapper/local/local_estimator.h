/*
 * estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_LOCAL_ESTIMATOR_H_
#define REC_FRAMEWORK_LOCAL_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/keypoints/uniform_sampling.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
      class LocalEstimator
      {
      protected:
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;
        typedef pcl::PointCloud<int> KeypointCloud;

        typename boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > normal_estimator_;
        typename boost::shared_ptr<UniformSampling<PointInT> > keypoint_extractor_;
        float support_radius_;

      public:
        virtual bool
        estimate (PointInTPtr & in, PointInTPtr & processed, KeypointCloud & keypoints, FeatureTPtr & signatures)=0;

        void
        setNormalEstimator (boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > & ne)
        {
          normal_estimator_ = ne;
        }

        /**
         * \brief Right now only uniformSampling keypoint extractor is allowed
         */
        void
        setKeypointExtractor (boost::shared_ptr<UniformSampling<PointInT> > & ke)
        {
          keypoint_extractor_ = ke;
        }

        void
        setSupportRadius (float r)
        {
          support_radius_ = r;
        }
      };
  }
}

#endif /* REC_FRAMEWORK_LOCAL_ESTIMATOR_H_ */

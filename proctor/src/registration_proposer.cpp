#include "proctor/registration_proposer.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

namespace pcl {

  namespace proctor {

    void
    RegistrationProposer::getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output) {
      std::vector<std::string>::iterator database_it;

      vector<Candidate> ballot;
      for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        std::string target_id = (*database_it);
        Entry target = (*database_)[target_id];

        Candidate* candidate = new Candidate;
        candidate->id = target_id;
        candidate->votes = -computeRegistration(query, target);
        ballot.push_back(*candidate);
      }

      selectBestCandidates(max_num, ballot, output);
    }

    double
    RegistrationProposer::computeRegistration(Entry &source, Entry &target)
    {
      typedef boost::function<void(const PointCloud<PointNormal> &cloud_src,
                                  const vector<int> &indices_src,
                                  const PointCloud<PointNormal> &cloud_tgt,
                                  const vector<int> &indices_tgt)> f;

      // Copy the source/target clouds with only the subset of points that
      // also features calculated for them.
      PointCloud<PointNormal>::Ptr source_subset(
          new PointCloud<PointNormal>(*(source.cloud), *(source.indices)));
      PointCloud<PointNormal>::Ptr target_subset(
          new PointCloud<PointNormal>(*(target.cloud), *(target.indices)));


      // TODO Filtering the source cloud makes it much faster when computing the
      // error metric, but may not be as good
      SampleConsensusInitialAlignment<PointNormal, PointNormal, Signature> ia_ransac;
      ia_ransac.setMinSampleDistance (0.05);
      ia_ransac.setMaxCorrespondenceDistance (0.5);
      ia_ransac.setMaximumIterations (1000);
      ia_ransac.setInputCloud (source_subset);
      ia_ransac.setSourceFeatures (source.features);
      //ia_ransac.setSourceIndices(source.indices);
      ia_ransac.setInputTarget (target_subset);
      ia_ransac.setTargetFeatures (target.features);
      //ia_ransac.setTargetIndices(target.indices);

      PointCloud<PointNormal>::Ptr aligned (new PointCloud<PointNormal>());
      ia_ransac.align(*aligned);

      IterativeClosestPoint<PointNormal, PointNormal> icp;
      PointCloud<PointNormal>::Ptr aligned2 (new PointCloud<PointNormal>());
      icp.setInputCloud(aligned);
      icp.setInputTarget(target.cloud);
      icp.setMaxCorrespondenceDistance(0.1);
      icp.setMaximumIterations(64);
      icp.align(*aligned2);
      return icp.getFitnessScore();
    }

  }

}

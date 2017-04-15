#include "modelQuery3D.h"
#include "utilityFuncs3D.h"

using namespace pcl::registration;

void CModelQuery3D::computeNormals()
{
    modelFeatures_.computeNormals();
    queryFeatures_.computeNormals();
}

void CModelQuery3D::computeKeyPoints()
{
    modelFeatures_.computeKeypoints();
    queryFeatures_.computeKeypoints();
}

void CModelQuery3D::computeDescriptors() 
{ 
    modelFeatures_.computeDescriptors();
    queryFeatures_.computeDescriptors();
}

void CModelQuery3D::computeAll()
{
    modelFeatures_.computeAll();
    queryFeatures_.computeAll();
    computeMatches();
    computePoseEstimation();
    computeHybridPoseEstimation();
    computeNoDupPoseEstimation();
}

void CModelQuery3D::computeMatches()
{
    unsigned	pole[] = {0,1,2};   // to experiment with cluster centers.
                                    // no significant difference in results

    float max_matching_distance = inputParams_.max_matching_distance;	// squared for the search!

    vCorrPtr_.resize(NUM_3D_DESCRIPTORS);
    vGoodCorr_.resize(NUM_3D_DESCRIPTORS);

    vCorrPtr_[0].reset(new pcl::Correspondences);
    vCorrPtr_[1].reset(new pcl::Correspondences);
    vCorrPtr_[2].reset(new pcl::Correspondences);

    pcl::StopWatch timer;

    CorrespondenceEstimation<FPFHSignature33, FPFHSignature33>	 est_fpfh; 
    est_fpfh.setInputSource(modelFeatures_.getComputedFPFHDesc());	
    est_fpfh.setInputTarget(queryFeatures_.getComputedFPFHDesc());
    est_fpfh.determineReciprocalCorrespondences(*vCorrPtr_[pole[0]],max_matching_distance);
    getSortedCorrespondences(*vCorrPtr_[pole[0]]);	// sort
    (pGlobalPerf_->pd[DESC_FPFH33]).processingTime = 
        modelFeatures_.getProcessingTime(DESC_FPFH33-DESC_FPFH33) + timer.getTimeSeconds();
    
    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("FPFH - Matching Time: %f\n",timer.getTimeSeconds());
    }

    timer.reset();

    CorrespondenceEstimation<SHOT352, SHOT352>		est_shot;
    est_shot.setInputSource(modelFeatures_.getComputedSHOTDesc());
    est_shot.setInputTarget(queryFeatures_.getComputedSHOTDesc());
    est_shot.determineReciprocalCorrespondences(*vCorrPtr_[pole[1]] ,max_matching_distance);
    getSortedCorrespondences(*vCorrPtr_[pole[1]]);	// sort
    (pGlobalPerf_->pd[DESC_SHOT352]).processingTime = 
        modelFeatures_.getProcessingTime(DESC_SHOT352-DESC_FPFH33) + timer.getTimeSeconds();

    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("SHOT352 - Matching Time: %f\n",timer.getTimeSeconds());
    }

    timer.reset();

    CorrespondenceEstimation<SHOT1344, SHOT1344>	est_cshot;
    est_cshot.setInputSource(modelFeatures_.getComputedCSHOTDesc());
    est_cshot.setInputTarget(queryFeatures_.getComputedCSHOTDesc());
    est_cshot.determineReciprocalCorrespondences(*vCorrPtr_[pole[2]],max_matching_distance);
    getSortedCorrespondences(*vCorrPtr_[pole[2]]);	// sort

    (pGlobalPerf_->pd[DESC_SHOT1344]).processingTime = 
        modelFeatures_.getProcessingTime(DESC_SHOT1344-DESC_FPFH33) + timer.getTimeSeconds();

    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("SHOT1344 - Matching Time: %f\n",timer.getTimeSeconds());
    }

    if ( inputParams_.debug_level > OUTPUT_DETAILS )
    {
        pcl::console::print_info ("CModelQuery3D::computeMatches done!\n");
        pcl::console::print_info ("fpfh size: %d, ", vCorrPtr_[0]->size());
        pcl::console::print_info ("shot size: %d, ", vCorrPtr_[1]->size());
        pcl::console::print_info ("cshot size: %d\n", vCorrPtr_[2]->size());
    }
}

void CModelQuery3D::getSortedCorrespondences(pcl::Correspondences & corr)
{
        if ( best_k_sorted_3D_ <= inputParams_.min_correspondence_count )
            return;

        pcl::Correspondences validCorr = corr;

        std::sort (validCorr.begin (), validCorr.end (),
                   pcl::registration::sortCorrespondencesByDistance ());

        corr = validCorr;
        corr.resize(best_k_sorted_3D_); 
}

void CModelQuery3D::computePoseEstimation()
{
      for ( unsigned i = 0; i <  NUM_3D_DESCRIPTORS; i++ )
      {
         pcl::StopWatch	timer;

         perfData  & pd = pGlobalPerf_->pd[i+DESC_FPFH33];

         // init perf struture
         pd.descID = i+DESC_FPFH33; 
         pd.actualRotation = actualRotation_;

         ransacRegParams	regParams(modelFeatures_.getKeypoints(), 
                                      queryFeatures_.getKeypoints(),
                                      vGoodCorr_[i], 
                                      vCorrPtr_[i], 
                                      pd,
                                      inputParams_);

         computeRANSACRegistrationModel(regParams);

         pd.processingTime = pd.processingTime + timer.getTimeSeconds();
      }
}

void CModelQuery3D::computeHybridPoseEstimation()
{
    if ( !inputParams_.best_k_sorted_3D )
    {	// best_k must be > 0
        return;
    }

    vHybridCorr_.reset(new pcl::Correspondences);

    unsigned idx = vCorrPtr_[0]->size();	
   
    // get the min size 
    for ( unsigned i = 0; i < NUM_3D_DESCRIPTORS; i++ )
    {
        if ( vCorrPtr_[i]->size() < idx )
        {
            idx = vCorrPtr_[i]->size();
        }
    }

    // concatenate, in this order...
    for ( unsigned j = 0; j < idx; j++ )
    {
        for ( unsigned i = 0; i <  NUM_3D_DESCRIPTORS; i++ ) 
        {									
            vHybridCorr_->push_back((*vCorrPtr_[i])[j]);
        }
    }

    perfData  & pd = pGlobalPerf_->pd[DESC_MD3dR];

    // init perf struture
    pd.descID = DESC_MD3dR; 
    pd.actualRotation = actualRotation_;
 
    ransacRegParams	regParams(modelFeatures_.getKeypoints(), 
                              queryFeatures_.getKeypoints(),
                              vHybridOutputCorr_, 
                              vHybridCorr_, 
                              pd,
                              inputParams_);

    computeRANSACRegistrationModel(regParams);
}

void CModelQuery3D::computeNoDupPoseEstimation()
{
    vNoDupCorr_.reset(new pcl::Correspondences);

    unsigned max_size1 = vCorrPtr_[0]->size();
    unsigned max_size2 = vCorrPtr_[1]->size();
    unsigned max_size3 = vCorrPtr_[2]->size();
    unsigned min_size  = max_size1;

    min_size = min_size < max_size2 ? min_size : max_size2;
    min_size = min_size < max_size3 ? min_size : max_size3;

/*
    * Get the intersection of correspondences across the 3D descriptors.
    * # commented out c-shot, as we get too few correspondences.
    * # interesting to experiment with which 2 work better.
    * # for now, we'll keep fpfh+shot.
    * # change the commented out lines to try other comb.
    *
    */
    for ( unsigned j1 = 0; j1 < min_size /* best_k_sorted_3D_ */ ; j1++ )  // fpfh
    {
        for ( unsigned j2 = 0; j2 < min_size /* best_k_sorted_3D_ */ ; j2++ ) //shot352
        {
            //for ( unsigned j3 = 0; j3 < max_size1 /* best_k_sorted_3D_ */ ; j3++ ) //shot1344
            //{
    
                int u1, v1, u2,v2, u3,v3;
            
                u1 = (*vCorrPtr_[0])[j1].index_query; v1 = (*vCorrPtr_[0])[j1].index_match;
                u2 = (*vCorrPtr_[1])[j2].index_query; v2 = (*vCorrPtr_[1])[j2].index_match;
                //u3 = (*vCorrPtr_[2])[j3].index_query; v3 = (*vCorrPtr_[2])[j3].index_match;

                if ( (u1 == u2) && /*  (u2 == u3) && */ (v1 == v2) /* && (v2 == v3)*/ )
                {
                    vNoDupCorr_->push_back((*vCorrPtr_[2])[j2]); // all the same, get one
                }
            //}
        }
    }

    // proceed only if we have sufficient # of correspondences
    if ( vNoDupCorr_->size() < MIN_CORRESPONDENCE_COUNT )
    {
         return;
    }

    perfData  & pd = pGlobalPerf_->pd[DESC_MD3dUR];

    // init perf struture
    pd.descID = DESC_MD3dUR; 
    pd.actualRotation = actualRotation_;
 
    ransacRegParams	regParams(modelFeatures_.getKeypoints(), 
                              queryFeatures_.getKeypoints(),
                              vNoDupOutputCorr_, 
                              vNoDupCorr_, 
                              pd,
                              inputParams_);

    computeRANSACRegistrationModel(regParams);
}

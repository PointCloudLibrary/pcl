#ifndef _CMODEL_QUERY_3D_H_
#define _CMODEL_QUERY_3D_H_

#include "processFeatures3D.h"
#include "common_structs.h"

using namespace pcl;

class CModelQuery3D
{
public:
    
    CModelQuery3D(inputArgs  & inputParams,
                  outputPerf * pGlobalPerf,
                  pcPtr	       pModelCloud,
                  pcPtr	       pQueryCloud )
    :  	inputParams_(inputParams)
    ,	pGlobalPerf_(pGlobalPerf)
    ,	modelFeatures_(inputParams, pModelCloud, pModelCloud )
    ,	queryFeatures_(inputParams, pQueryCloud, pQueryCloud )
    ,	best_k_sorted_3D_(inputParams.best_k_sorted_3D)
    ,	actualRotation_(inputParams.rot_deg)
        {
            // initialize perf data for all 3D descriptors
            perfData_.resize(NUM_3D_DESCRIPTORS+2); 

            for ( unsigned i = 0; i < perfData_.size(); i++ )
            {
                perfData_[i].descID = i+DESC_FPFH33; 
                perfData_[i].actualRotation = -1.0;
                perfData_[i].computedRotation = 0.0;
                perfData_[i].inlierCount = 0;
                perfData_[i].inlierRate  = 0.0;
                perfData_[i].rotEstError = 100.0;
            }
        }

        void computeNormals();
        void computeKeyPoints(); 
        void import2DKeyPoints();
        void computeDescriptors();
        void computeMatches();
        void getSortedCorrespondences(pcl::Correspondences & corr);
        void computePoseEstimation();
        void computeHybridPoseEstimation();
        void computeNoDupPoseEstimation();
        void computeAll();

        void computePixelDomainPoseEstimation();

        vector<perfData> & getPerfData()	{ return perfData_; }

private:

    inputArgs				inputParams_;

    keyPointType3D			keyPointType3D_;
    descriptorType3D			descriptorType3D_;

    CProcessFeatures3D			modelFeatures_;
    CProcessFeatures3D			queryFeatures_;

    vector<CorrespondencesPtr>	        vCorrPtr_;
    vector<Correspondences>		vGoodCorr_;
    unsigned				best_k_sorted_3D_;

    // multi-descriptor correspondences
    CorrespondencesPtr	 		vHybridCorr_;	
    Correspondences			vHybridOutputCorr_;


    CorrespondencesPtr	 		vNoDupCorr_;	
    Correspondences			vNoDupOutputCorr_;

    // perf analysis
    float				actualRotation_;
    vector<perfData>			perfData_;
    outputPerf		 *		pGlobalPerf_;

};
#endif

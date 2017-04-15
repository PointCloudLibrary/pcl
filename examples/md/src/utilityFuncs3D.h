#ifndef _UTILITY_FUNCS_3D_H_
#define _UTILITY_FUNCS_3D_H_

#include "common_structs.h"
#include "processFeatures3D.h"

/*
 * ransacRegParams: This sturcture packs the needed
 * parameters for to perform ransac based registration
 * between a pair of point clouds.
 */

typedef struct ransacRegParams
{
    pcPtr			pModel;
    pcPtr			pQuery;

    pcl::Correspondences    &	corr2DOut;
    pcl::CorrespondencesPtr &	corr2D;

    perfData	 	    &	pd;
    inputArgs		    &	inputParams;

    float			inlierThreshold;
    unsigned			maxIterations;

    // constructor
    ransacRegParams( pcPtr	pModel_,
                     pcPtr	pQuery_,
    pcl::Correspondences    &	corr2DOut_,
    pcl::CorrespondencesPtr &	corr2D_,
                 perfData   &	pd_,
                 inputArgs  &	inputParams_ ) 

 :  pModel(pModel_)
 ,  pQuery(pQuery_)
 ,  corr2DOut(corr2DOut_)
 ,  corr2D(corr2D_)
 ,  pd(pd_)
 ,  inputParams(inputParams_)
 {
    inlierThreshold = inputParams.ransac_inlier_threshold_3D;
    maxIterations = inputParams.ransac_max_iterations_3D;
 }
} ransacRegParams;

/*
 * Forward declarations
 */

pcPtr spinPointCloud(pcPtr & sourcePc, float deg);
pcPtr downSampleCloud(pcPtr & sourcePc, float leafSize);
void computeRANSACRegistrationModel(ransacRegParams & in);

/*
 *  Descriptor computation routine
 */

template<typename descriptorT, typename pcDescT, typename pcDescPtrT, typename pcDescType>
pcDescPtrT computeDescriptor(pcPtr points, SurfaceNormalsPtr normals, pcPtr keypoints, float feature_radius)
{ 
    pcDescPtrT	computedDescriptorPtr(new pcDescT);
    pcDescType	descriptorEstimation;

    descriptorEstimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));

    descriptorEstimation.setRadiusSearch (feature_radius); 
    descriptorEstimation.setSearchSurface (points);  
    descriptorEstimation.setInputNormals (normals);
    descriptorEstimation.setInputCloud (keypoints); 

    descriptorEstimation.compute (*computedDescriptorPtr);

    return computedDescriptorPtr;
}

#endif

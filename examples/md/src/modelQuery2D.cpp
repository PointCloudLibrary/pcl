#include "modelQuery2D.h"


void CModelQuery2D::computeKeyPoints() 
{
    modelFeatures_.computeKeypoints(); 
    queryFeatures_.computeKeypoints(); 
}

void CModelQuery2D::computeDescriptors()
{
    modelFeatures_.computeDescriptors();
    queryFeatures_.computeDescriptors();
}

void CModelQuery2D::computeMatches()
{
    switch(descriptorType_)
    {
        case DESC_SIFT:
        case DESC_SURF: 
                    cv::BFMatcher(cv::NORM_L2,true).match(modelFeatures_.getDescriptors(),
                                                          queryFeatures_.getDescriptors(),
                                                          matchingResults_);
                    break;

        case DESC_ORB:
                    cv::BFMatcher(cv::NORM_HAMMING2,true).match(modelFeatures_.getDescriptors(),
                                                               queryFeatures_.getDescriptors(),
                                                               matchingResults_);
                    break;

        case DESC_BRISK:
                    cv::BFMatcher(cv::NORM_HAMMING,true).match(modelFeatures_.getDescriptors(),
                                                               queryFeatures_.getDescriptors(),
                                                               matchingResults_);
                    break;
        default:
            throw(-1);
    }

    // compute the minimum distance in the matching set.
    for ( unsigned i = 0; i < matchingResults_.size(); i++ )
    {
        if ( matchingResults_[i].distance < minDistance_ )
            minDistance_ = matchingResults_[i].distance;
    }
    
    if ( inputParams_.debug_level > OUTPUT_DETAILS )
    {
        pcl::console::print_info("--------------------------------------------------------------\n");
        pcl::console::print_info("DescriptorID: %d, 2D Correspondence size: %d, MinDistance: %3.3f\n", 
                                  descriptorType_, matchingResults_.size(), minDistance_);
    }
}

void CModelQuery2D::computePoseEstimation()
{
    std::vector<cv::DMatch> & validCorr = matchingResults_;

    if ( best_k_sorted_2D_ > 0 )
    {
        std::sort (validCorr.begin (), validCorr.end (),
                   sort2DCorrespondencesByDistance ());

        validCorr.resize(best_k_sorted_2D_);
    }

    vector<cv::KeyPoint>	modelKeyPoints = modelFeatures_.getKeypoints();
    vector<cv::KeyPoint>	queryKeyPoints = queryFeatures_.getKeypoints();

    for ( unsigned i = 0; i < validCorr.size(); i++ )
    {
        modelPoints_.push_back(modelKeyPoints[ validCorr[i].queryIdx].pt);
        queryPoints_.push_back(queryKeyPoints[ validCorr[i].trainIdx].pt);
    }

    vector<unsigned char> inliersMask(modelPoints_.size());

    H_ = findHomography(modelPoints_ , queryPoints_,  CV_RANSAC, inputParams_.ransac_inlier_threshold_2D, inliersMask );
    
    vector<cv::DMatch> inliers;
    unsigned inliersCount = 0;

    for ( unsigned i = 0; i < inliersMask.size(); i++ )
    {
        if ( inliersMask[i] )
        {
            inliersCount++;
            inliers.push_back(validCorr[i]);
        }
    }

    // fill our performance data structure

    perfData  & pd = pGlobalPerf_->pd[descriptorType_];

    pd.descID = descriptorType_;
    pd.actualRotation = inputParams_.rot_deg;
    pd.inlierCount = inliersCount; 
    pd.inlierRate  = (float) inliersCount / validCorr.size(); 

    if ( inputParams_.live_sensor )	// assumes horizontal rotation
    {
        pd.computedRotation = pcl::rad2deg ((atan(H_.at<double>(6)/H_.at<double>(0))));
        // if live_sensor, perfData_.actualRotation is ignored.
    }
    else  // simulated rotation
    {
        pd.computedRotation = pcl::rad2deg ((atan(H_.at<double>(3)/H_.at<double>(0))));
    }

    pd.rotEstError = abs(abs(pd.computedRotation) - abs(pd.actualRotation));

    pd.averageDistance = -1.0;	//## not in use

    if ( inputParams_.debug_level > OUTPUT_TRANSFORMATIONS )
    {
        // print out our H matrix
        pcl::console::print_info ("----------------------------------------------------------------------------\n");
        pcl::console::print_info ("2D DescID: %d,  Computed Rotation: %8.3f, actual Rotation: %8.3f\n",
                                   pd.descID,pd.computedRotation, pd.actualRotation);
        pcl::console::print_info ("inlierCount: %d, inlierRate: %8.3f\n", pd.inlierCount, pd.inlierRate);
        pcl::console::print_info ("\n");
        pcl::console::print_info ("    | %8.3f %8.3f %8.3f | \n", H_.at<double>(0), H_.at<double>(1), H_.at<double>(2) );
        pcl::console::print_info ("R = | %8.3f %8.3f %8.3f | \n", H_.at<double>(3), H_.at<double>(4), H_.at<double>(5) );
        pcl::console::print_info ("    | %8.3f %8.3f %8.3f | \n", H_.at<double>(6), H_.at<double>(7), H_.at<double>(8) );
        pcl::console::print_info ("\n");
    }
}

void CModelQuery2D::visualizeCorrespondences()
{
    pModelImage_->visualizeKeyPoints(modelPoints_);
    pQueryImage_->visualizeKeyPoints(queryPoints_);
}

void CModelQuery2D::computeAll()
{
    pcl::StopWatch    timer;

    modelFeatures_.computeAll();
    queryFeatures_.computeAll();
    computeMatches();
    computePoseEstimation();
    
    perfData  & pd = pGlobalPerf_->pd[descriptorType_];

    pd.processingTime = timer.getTimeSeconds();

    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("2D - DescID: %d, Time: %f\n", descriptorType_, pd.processingTime );
    }
    
    visualizeCorrespondences();
}



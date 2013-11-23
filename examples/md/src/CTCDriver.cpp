#include "CTCDriver.h"
#include "utilityFuncs3D.h"
 
CTCDriver::CTCDriver(inputArgs & inputParams)
    :	inputParams_(inputParams)
    ,   pModelCloud_(inputParams.pModelCloud)
    ,	pQueryCloud_(inputParams.pQueryCloud)
    ,	modelImage_(inputParams.b_enable_graphics)
    ,	queryImage_(inputParams.b_enable_graphics)
    ,	leafSize_(inputParams.leaf_size)
    ,	minKeyPointCount2D_(MIN_KEYPOINT_COUNT)
    ,	minKeyPointCount3D_(MIN_KEYPOINT_COUNT)
    ,	rotDeg_(inputParams.rot_deg)
    ,	best_k_sorted_2D_(inputParams.best_k_sorted_2D)
    ,	best_k_sorted_3D_(inputParams.best_k_sorted_3D)
    ,	logPerfData_(inputParams.logFile1)
    ,	logResults_(inputParams.logFile2)
{

}

void CTCDriver::init2D()
{ 
    modelImage_.init2DImage(pModelCloud_, rotDeg_);	
    queryImage_.init2DImage(pQueryCloud_);	

}

void CTCDriver::init2DFeatureObject(keyPointType2D kpType, descriptorType dType)
{ 
    mqPtr2D modelQuery2D(new CModelQuery2D(inputParams_,
                                           &globalPerf_,
                                           pModelCloud_ ,&modelImage_, 
                                           pQueryCloud_, &queryImage_ ));

    if ( !modelQuery2D )
    {
        pcl::console::print_info ("init2DFeatureObject: Invalid modelQuery2D\n");
        throw(-1);
    }

    modelQuery2D->init();

    modelQuery2D->setKeyPointType(kpType);
    modelQuery2D->setDescriptorType(dType);

    modelQuery2D_.push_back(*modelQuery2D);
}

void CTCDriver::init3DFeatureObject()		
{
    pcl::StopWatch timer;

    pcPtr pModel = downSampleCloud ( pModelCloud_, leafSize_ );

    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("DownSampling Time: %f\n",timer.getTimeSeconds());
    }

    pcPtr pQuery = downSampleCloud ( pQueryCloud_, leafSize_ );
    pQuery = spinPointCloud  ( pQuery, rotDeg_ );

    pModelQuery3D_ = new CModelQuery3D(inputParams_,
                                       &globalPerf_,
                                       pModel,
                                       pQuery );
    if ( !pModelQuery3D_ )
    {
        pcl::console::print_info ("init3DFeatureObject: Invalid pModelQuery3D_\n");
        throw(-1);
    }
}

void CTCDriver::compute2DKeyPoints()
{
    unsigned max_size = modelQuery2D_.size();

    for ( unsigned i = 0; i < max_size; i++ )
    {
        modelQuery2D_[i].computeKeyPoints();
    }
}

void CTCDriver::compute2DDescriptors()
{
    unsigned max_size = modelQuery2D_.size();

    for ( unsigned i = 0; i < max_size; i++ )
    {
        modelQuery2D_[i].computeDescriptors();
    }
}

void CTCDriver::compute2DMatches()
{
    unsigned max_size = modelQuery2D_.size();

    for ( unsigned i = 0; i < max_size; i++ )
    {
        modelQuery2D_[i].computeMatches();
    }
}

void CTCDriver::computeAll()
{
    // if thread_count is 1, multi-threading is disabled
    if ( inputParams_.thread_count < 2 )
    {
        if ( inputParams_.debug_level > OUTPUT_TRANSFORMATIONS )
        {
            pcl::console::print_info ("Threading mode : top layer : single threaded\n");
        }

        computeAll2D();
        computeAll3D();
    }
    else
    {	// default is to use 2 threads at the top level
        if ( inputParams_.debug_level > OUTPUT_TRANSFORMATIONS )
        {
            pcl::console::print_info ("Threading mode : top layer : multi-threaded\n");
        }

        boost::thread_group	thrd_grp;
    
        thrd_grp.create_thread(boost::bind(&CTCDriver::computeAll2D,this));
        thrd_grp.create_thread(boost::bind(&CTCDriver::computeAll3D,this));

        thrd_grp.join_all();
    }
}

void CTCDriver::computeAll2D()
{	

    unsigned max_2D_size = modelQuery2D_.size();

    for ( unsigned i = 0; i < max_2D_size; i++ )
    {
        modelQuery2D_[i].computeAll();
    }

    // apply RANSAC to all 2D descriptors
    compute_MD2dR();

    if ( inputParams_.debug_level > OUTPUT_DETAILS )
    {
        pcl::console::print_info ("\n############################ end 2D ########################\n");
    }
}
    
void CTCDriver::computeAll3D()
{
    if ( !pModelQuery3D_ )
    {
        pcl::console::print_info ("CTCDriver::computeAll3D: Invalid pModelQuery3D_\n");
        return;
    }

    pModelQuery3D_->computeAll();

    if ( inputParams_.debug_level > OUTPUT_DETAILS )
    {
        pcl::console::print_info ("\n############################ end 3D ########################\n");
    }
}

void CTCDriver::initiateVoteProcessing()
{		
    CMultiDescriptorVoting	  mdVoter(inputParams_, &globalPerf_);
    mdVoter.computeVotes();
    mdVoter.logFinalResults();
    logPerfDataToFile(globalPerf_.pd);
    mdVoter.displayRuntimePerformance();
    mdVoter.displayVotingResults();
}

void CTCDriver::compute_MD2dR()
{
    unsigned best_k_sorted = inputParams_.best_k_sorted_2D;
    std::vector<cv::Point2f> modelPoints;
    std::vector<cv::Point2f> queryPoints;
    cv::Mat					 H;

    for ( unsigned i = 0; i < NUM_2D_DESCRIPTORS; i++ )
    {
        std::vector<cv::DMatch> & validCorr		 = modelQuery2D_[i].getCorrespondences();
        vector<cv::KeyPoint>	& modelKeyPoints = modelQuery2D_[i].getModelKeyPoints();
        vector<cv::KeyPoint>	& queryKeyPoints = modelQuery2D_[i].getQueryKeyPoints();

        for ( unsigned i = 0; i < validCorr.size(); i++ )
        {
            modelPoints.push_back(modelKeyPoints[validCorr[i].queryIdx].pt);
            queryPoints.push_back(queryKeyPoints[validCorr[i].trainIdx].pt);
        }
    }

    vector<unsigned char> inliersMask(modelPoints.size());

    H = findHomography(modelPoints , queryPoints, CV_RANSAC , inputParams_.ransac_inlier_threshold_2D, inliersMask );

    vector<cv::DMatch> inliers;
    unsigned inliersCount = 0;

    for ( unsigned i = 0; i < inliersMask.size(); i++ )
    {
        if ( inliersMask[i] )
        {
            inliersCount++;
        }
    }

    // init per structure
     perfData  & pd = globalPerf_.pd[DESC_MD2dR];

     pd.descID = DESC_MD2dR; 
     pd.actualRotation = inputParams_.rot_deg;
     pd.inlierCount = inliersCount; 
     pd.inlierRate  = (float) inliersCount / inliersMask.size();

    if ( inputParams_.live_sensor )	// assumes horizontal rotation
    {
        pd.computedRotation = pcl::rad2deg ((atan(H.at<double>(6)/H.at<double>(0))));
        // if live_sensor, perfData_.actualRotation is ignored.
    }
    else	// simulated rotation
    {
        pd.computedRotation = pcl::rad2deg ((atan(H.at<double>(3)/H.at<double>(0))));
    }

    pd.actualRotation = inputParams_.rot_deg;  
    pd.rotEstError	  = abs(abs(pd.computedRotation) - abs(pd.actualRotation));

    pd.averageDistance = -1.0;	//## not in use

    // print out the H matrix
    if ( inputParams_.debug_level > OUTPUT_TRANSFORMATIONS )
    {
        pcl::console::print_info("----------------------------------------------------------------------------\n");
        pcl::console::print_info ("2D DescID: %d,  Computed Rotation: %8.3f, actual Rotation: %8.3f\n",
                                   pd.descID,pd.computedRotation, pd.actualRotation);
        pcl::console::print_info ("inlierCount: %d, inlierRate: %8.3f\n", pd.inlierCount, pd.inlierRate);
        pcl::console::print_info("\n");
        pcl::console::print_info("    | %8.3f %8.3f %8.3f | \n", H.at<double>(0), H.at<double>(1), H.at<double>(2) );
        pcl::console::print_info("R = | %8.3f %8.3f %8.3f | \n", H.at<double>(3), H.at<double>(4), H.at<double>(5) );
        pcl::console::print_info("    | %8.3f %8.3f %8.3f | \n", H.at<double>(6), H.at<double>(7), H.at<double>(8) );
        pcl::console::print_info("\n");
    }
}

// for matlab graphics, not tested after re-write
void CTCDriver::logPerfDataToFile(vector<perfData> & pd)
{
    char buf[1024];

    // actualrot, descID, computedrot, err,... repeat for the 7+4 (original & derived) evaluations.
    sprintf(buf,"%8.3f",pd[0].actualRotation);
    (*logPerfData_) << buf;

    for ( unsigned i = 0; i < pd.size() ; i++ )
    {
        sprintf(buf,",%3d,%8.3f,%8.3f",pd[i].descID,pd[i].computedRotation,pd[i].rotEstError);
        (*logPerfData_) << buf;
    }

    sprintf(buf,"\n");
    (*logPerfData_) << buf;
}

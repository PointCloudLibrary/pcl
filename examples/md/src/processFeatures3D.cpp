#include "processFeatures3D.h"
#include "utilityFuncs3D.h"


CProcessFeatures3D::CProcessFeatures3D(inputArgs & inputParams,
                                       pcPtr	   pCloud, 
                                       pcPtr	   pSearchSurface )
    :  inputParams_(inputParams)
    ,  pCloud_(pCloud)
    ,  pSearchSurface_(pSearchSurface)
    ,  keyPointType_(MDO_KP3D_INVALID)
    ,  descriptorType_(MDO_D3D_INVALID)
    ,  normalsRadius_(inputParams_.normals_estimation_radius)
    ,  descriptorRadius_(inputParams_.descriptor_radius)
    ,  bValidNormals_(false)
    ,  bValidKeypoints_(false)
{
    normals_.reset();
    filteredKeypoints_.reset();
    unfilteredKeypoints_.reset();
    fpfhDesc_.reset();
    shotDesc_.reset();
    cshotDesc_.reset();
    vProcessingTime.resize(NUM_3D_DESCRIPTORS);
}

void CProcessFeatures3D::setKeyPointType(keyPointType3D kpType)
{
    keyPointType_ = kpType;
}

void CProcessFeatures3D::setDescriptorType(descriptorType3D dType)
{
    descriptorType_ = dType; 
}

void CProcessFeatures3D::computeNormals()
{
  pcl::StopWatch timer;

  // assert preconditions
  assert( pCloud_ );
  assert( pSearchSurface_ );

  pcl::NormalEstimation<PointT, NormalT> normal_estimation;
  
  // set normal estimation parameters
  normal_estimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  normal_estimation.setRadiusSearch (normalsRadius_);
  
  normal_estimation.setInputCloud (pCloud_);
  normal_estimation.setSearchSurface(pSearchSurface_);
  
  // compute the normals
  normals_.reset(new SurfaceNormals);
  normal_estimation.compute (*normals_);

  bValidNormals_ = true;

  // assert Postcondition
  assert (normals_);

  processingTimeNormals_ = timer.getTimeSeconds();

}


unsigned CProcessFeatures3D::computeKeypoints()
{
    pcl::StopWatch timer;

    // assert precondtion
    assert ( pCloud_ );

    // Don't recompute if keypoints are already found for this set.
    if ( bValidKeypoints_ )
    {
        pcl::console::print_info ("keyPoint size: %d\n", unfilteredKeypoints_->size ());
        return unfilteredKeypoints_->size (); 
    }

    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_est;
    sift_est.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
    sift_est.setScales (MIN_SCALE, NR_OCTAVES, NR_SCALES_PER_OCTAVE);
    sift_est.setMinimumContrast (MIN_CONTRAST);
    sift_est.setInputCloud (pCloud_);

    pcl::PointCloud<pcl::PointWithScale> keypoints_with_scale;
    sift_est.compute (keypoints_with_scale);

    unfilteredKeypoints_.reset(new pc);
    pcl::copyPointCloud (keypoints_with_scale, *unfilteredKeypoints_);

    if ( !unfilteredKeypoints_->size () )
    {
        pcl::console::print_info ("warning: CProcessFeatures3D::computeKeypoints: 0 keypoints\n");
    }

    if ( inputParams_.debug_level > OUTPUT_DETAILS )
    {
        pcl::console::print_info ("SIFT keypoints found: %d\n", unfilteredKeypoints_->size ());
    }

    bValidKeypoints_ = true;

    processingTimeKPs_ = timer.getTimeSeconds();
    
    return unfilteredKeypoints_->size();
}

void CProcessFeatures3D::removeDuplicateKeyPoints()
{
    // Assumptions:
    //  (1) redundant keypoints are consequtive in the list. 
    //  (2) 1st keypoint is retained. The following duplicates are removed.

    // assert preconditions
    assert(bValidKeypoints_);
    assert(unfilteredKeypoints_);

    filteredKeypoints_.reset(new pc);

    size_t kSize = unfilteredKeypoints_->size();
    for ( unsigned i = 0 ; i < kSize; i++ )
    {	 
        if ( i == 0 )
        {
            filteredKeypoints_->push_back(unfilteredKeypoints_->at(i));
        }
        else
        {
            if  ( (  ((unfilteredKeypoints_->at(i)).x) != ((filteredKeypoints_->back()).x) ) && 
                  (  ((unfilteredKeypoints_->at(i)).y) != ((filteredKeypoints_->back()).y) ) &&
                  (  ((unfilteredKeypoints_->at(i)).z) != ((filteredKeypoints_->back()).z) ) )
            {
                        filteredKeypoints_->push_back(unfilteredKeypoints_->at(i));
            }
        }
    }

    filteredKeypoints_->width = filteredKeypoints_->points.size();
    filteredKeypoints_->height = 1;
    filteredKeypointsRatio_ = (float)filteredKeypoints_->size() / unfilteredKeypoints_->size();

    if ( inputParams_.debug_level > OUTPUT_DETAILS )
    {
        pcl::console::print_info ("Removed duplicates, Original: %d, Size after filtering: %d, Ratio: %f\n",
                                    unfilteredKeypoints_->size(), filteredKeypoints_->size(), (float)filteredKeypointsRatio_ );
    }	
}

void CProcessFeatures3D::computeFPFHDescriptor()
{
    pcl::StopWatch	timer;

    fpfhDesc_ = computeDescriptor<fpfhDescriptorT,
                                  fpfhDescriptors,
                                  fpfhDescriptorsPtr,	
                                  pcl::FPFHEstimation<PointT, NormalT, fpfhDescriptorT> >
                                  (pCloud_, normals_, filteredKeypoints_,descriptorRadius_);

    vProcessingTime[0] = processingTimeNormalsKPs_ + timer.getTimeSeconds();
    
    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("Time Normals+Keypoints: %f, FPFH Time: %f\n", processingTimeNormalsKPs_, timer.getTimeSeconds() );
    }

}

void CProcessFeatures3D::computeSHOTDescriptor()
{
    pcl::StopWatch	timer;

    shotDesc_ = computeDescriptor<shotDescriptorT,
                                  shotDescriptors,
                                  shotDescriptorsPtr,	 
                                  pcl::SHOTEstimation<PointT, NormalT, shotDescriptorT> >
                                  (pCloud_, normals_, filteredKeypoints_,descriptorRadius_);

    vProcessingTime[1] = processingTimeNormalsKPs_ + timer.getTimeSeconds();

    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("Time Normals+Keypoints: %f, SHOT352 Time: %f\n", processingTimeNormalsKPs_, timer.getTimeSeconds() );
    }

}

void CProcessFeatures3D::computeCSHOTDescriptor()
{
    pcl::StopWatch	timer;

    cshotDesc_ = computeDescriptor<cshotDescriptorT,
                                   cshotDescriptors,
                                   cshotDescriptorsPtr,	 
                                   pcl::SHOTColorEstimation<PointT, NormalT, cshotDescriptorT> >
                                   (pCloud_, normals_, filteredKeypoints_,descriptorRadius_);
    
    vProcessingTime[2] = processingTimeNormalsKPs_ + timer.getTimeSeconds();

    if ( inputParams_.debug_level == OUTPUT_RUNTIME_PERF )
    {
        pcl::console::print_info ("Time Normals+Keypoints: %f, SHOT1344 Time: %f\n", processingTimeNormalsKPs_, timer.getTimeSeconds() );
    }

}

void CProcessFeatures3D::computeDescriptors()
{	
    // if thread_count is <=2, 2nd tier multi-threading is disabled
    if ( inputParams_.thread_count <= 2 )
    {
        if ( inputParams_.debug_level > OUTPUT_TRANSFORMATIONS )
        {
            pcl::console::print_info("Thread mode: 3D Descriptors: single threaded\n");
        }

        computeFPFHDescriptor();
        computeSHOTDescriptor();
        computeCSHOTDescriptor();
    }
    else
    {	// if more than 2, enable multi-threading
        if ( inputParams_.debug_level > OUTPUT_TRANSFORMATIONS )
        {
            pcl::console::print_info("Thread mode: 3D Descriptors: multi-threaded\n");
        }

        boost::thread_group	thrd_grp;
        thrd_grp.create_thread(boost::bind(&CProcessFeatures3D::computeFPFHDescriptor,this));
        thrd_grp.create_thread(boost::bind(&CProcessFeatures3D::computeSHOTDescriptor,this));
        thrd_grp.create_thread(boost::bind(&CProcessFeatures3D::computeCSHOTDescriptor,this));
        thrd_grp.join_all();
    }

    return;
}

void CProcessFeatures3D::computeAll()
{
    pcl::StopWatch	timer;

    computeNormals();
    computeKeypoints();
    removeDuplicateKeyPoints(); 

    processingTimeNormalsKPs_ = timer.getTimeSeconds();

    computeDescriptors();

    return;
}

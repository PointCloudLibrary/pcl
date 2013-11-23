#include "processFeatures2D.h"

CProcessFeatures2D::CProcessFeatures2D()
    :  fdd_(0)
    ,  keyPointType_(MDO_KP2D_INVALID)
    ,  descriptorType_((descriptorType)DESC_INVALID)
    ,  ratioValidKeyPoints_(0.0)
{
}

void CProcessFeatures2D::init(pcPtr pCloud, cv::Mat_<uchar> * pImgGS)
{
    pCloud_ = pCloud;
    pImgGS_ = pImgGS;
}

void CProcessFeatures2D::setKeyPointType(keyPointType2D kpType)
{
    keyPointType_ = kpType;

    switch(keyPointType_)
    {
        case MDO_KP2D_SIFT:
                    fdd_ = new cv::SIFT(); 
                    break;

        case MDO_KP2D_SURF: 
                    fdd_ = new cv::SURF(); 
                    break;

        case MDO_KP2D_ORB:
                    fdd_ = new cv::ORB(); 
                    break;

        case MDO_KP2D_BRISK:
                    fdd_ = new cv::BRISK();
                    break;
        default:
            throw(-1);
    }
}


cv::Mat	CProcessFeatures2D::getDescriptors()
{
    // assert preconditions
    assert( extractedDescriptors_.size );

    if ( 0 == extractedDescriptors_.size )
    {
        pcl::console::print_info ("CProcessFeatures2D::getDescriptors: Invalid Descriptors size, exiting!\n");
        throw(-1);
    }

    return extractedDescriptors_;
}

void CProcessFeatures2D::computeKeypoints()
{	
    // assert preconditions
    assert ( fdd_ );

    if ( fdd_ == 0 )
    {
        pcl::console::print_info ("CProcessFeatures2D::computeKeypoints: Invalid Preconditions\n");
        throw(-1);
    }

    // compute keypoints
    fdd_->detect(*pImgGS_,extractedKeyPoints_);
}

void CProcessFeatures2D::computeDescriptors()
{	
    // assert preconditions
    assert ( fdd_ );
    assert ( extractedKeyPoints_.size() );

    if ( ( fdd_ == 0 ) || ( extractedKeyPoints_.size() == 0 ) )
    {
        pcl::console::print_info ("CProcessFeatures2D::computeDescriptors: Invalid Preconditions\n");
        throw(-1);
    }

    fdd_->compute(*pImgGS_, extractedKeyPoints_, extractedDescriptors_);
}


void CProcessFeatures2D::computeAll()
{
    computeKeypoints();
    computeDescriptors();
}

#include "imageFromCloud.h"

CImageFromCloud::CImageFromCloud(bool bEnableGraphics)
    :   bEnableGraphics_(bEnableGraphics)
    ,	imgWidth_(IMAGE_2D_WIDTH)
    ,	imgHeight_(IMAGE_2D_HEIGHT)
    ,	invalidPointCount_(0)
    ,	totalPointCount_(0)
    ,	validPointCountRatio_(0.0)
    {
        pVisImg_	= new	cv::Mat_<cv::Vec3b>(imgHeight_,imgWidth_,cv::Vec3b(0,0,0));
        pImgBGR_	= new	cv::Mat_<cv::Vec3b>(imgHeight_,imgWidth_,cv::Vec3b(0,0,0));
        pImgGS_		= new	cv::Mat_<uchar>(imgHeight_,imgWidth_,uchar(0));
        pImgDepth_	= new	cv::Mat_<uchar>(imgHeight_,imgWidth_,uchar(0));

        if ( !pVisImg_ || !pImgBGR_ || !pImgGS_ || !pImgDepth_ )
        {
            pcl::console::print_info ("CImageFromCloud::CImageFromCloud: mem allocation failure, exiting!\n");
            throw(-1);
        }
    }


void CImageFromCloud::init2DImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, float deg)
{
    /*
     * Input:  a valid organized point cloud
     * Output: valid BGR, Gray scale, and depth images
     *		   # of valid points in the cloud (with finite dimensions).
     */

    if ( !pCloud )
    {
        pcl::console::print_info ("CImageFromCloud: Null inputCloud, exiting!\n");
        throw(-1);
    }
    else
    {
        pCloud_ = pCloud;
    }

    if ( ( pCloud_->width != imgWidth_ ) || ( pCloud_->height != imgHeight_ ) )
    {
        pcl::console::print_info ("CImageFromCloud: invalid cloud size: %d, %d, exiting\n", pCloud_->width, pCloud_->height);
        throw(-1);
    }

    if ( ! ( pImgBGR_ && pImgGS_ && pImgDepth_ ) )
    {
        pcl::console::print_info ("CImageFromCloud: invalid pImgBGR, pImgGS, pImgDepth, exiting\n");
        throw(-1);
    }
    
    cv::Mat_<cv::Vec3b>	& imgBGR	=	*pImgBGR_;	
    cv::Mat_<uchar>	& imgGS		=	*pImgGS_;
    cv::Mat_<uchar>	& imgDepth	=	*pImgDepth_;
    cv::Mat_<cv::Vec3b> *  pImgVis	=	new cv::Mat_<cv::Vec3b>(imgHeight_,imgWidth_,cv::Vec3b(0,0,0));
    cv::Mat_<cv::Vec3b> & imgRORVis	=       *pImgVis;
    
    /*
     * Initialize our image set: BGR, gray scale, and depth
     * convert the RGB-D into BGR and D images,
     * 
     */  

    for ( unsigned idx_Y = 0; idx_Y < imgHeight_ ; idx_Y++ )
    {
        for ( unsigned idx_X = 0; idx_X < imgWidth_ ; idx_X++ )
        {
            unsigned currentPointIdx = getPointIdxFromPixelCoordinates(idx_X,idx_Y);

            unsigned b = (pCloud_->points[currentPointIdx]).b;
            unsigned r = (pCloud_->points[currentPointIdx]).r;
            unsigned g = (pCloud_->points[currentPointIdx]).g;

            float xx = (pCloud_->points[currentPointIdx]).x;
            float yy = (pCloud_->points[currentPointIdx]).y;
            float zz = (pCloud_->points[currentPointIdx]).z;
    
            // assign color, map the depth to gray scale
            imgBGR[idx_Y][idx_X]	= cv::Vec3b(b,g,r);
            imgDepth[idx_Y][idx_X]	= (zz * 255)/(3.0); // finite, scaled up to 3 meters.	 
            imgRORVis[idx_Y][idx_X]	= cv::Vec3b(0,g,0);
            unsigned depthVal = 255 - imgDepth[idx_Y][idx_X];
            (*pVisImg_)[idx_Y][idx_X]	= cv::Vec3b(depthVal,depthVal,depthVal);
            
            // count/mark points with invalid dimensions
            if  ( !pcl_isfinite(xx) || !pcl_isfinite(yy) ||  !pcl_isfinite(zz) ) //  (0x7fc00000)
            {
                imgDepth[idx_Y][idx_X]		= 0xff;	//## NaN
                imgRORVis[idx_Y][idx_X]		= cv::Vec3b(0,  g, r );
                (*pVisImg_)[idx_Y][idx_X]	= cv::Vec3b(b,g/2,0);

                invalidPointCount_++;						
            }
        }
    }

    cvtColor(*pImgBGR_,*pImgGS_,CV_BGR2GRAY);
    if ( deg )
    {
        rotateImage(deg);
    }

    showRGBImage(imgBGR,"RGB Image");
    showIntensityImage(imgDepth,"DepthImage");
    showIntensityImage(*pImgGS_,"Original Image");
    showRGBImage(imgRORVis,"Visualization Image");
}

void CImageFromCloud::visualizeKeyPoints(std::vector<cv::Point2f> & keyPoints)
{
    if ( !bEnableGraphics_ )
        return;

    cv::Mat	outputImage = *pVisImg_;

    // highlight the keypoints
    for(unsigned i = 0; i < keyPoints.size(); i++)
    {
        cv::circle(outputImage, keyPoints[i], 7, CV_RGB(255, 255, 0), 1, CV_AA);
    }

    cv::imshow("KeyPoints", outputImage);
    cv::waitKey(0);
    
}


float CImageFromCloud::getInvalidCountRatio()
{
    totalPointCount_ = imgWidth_ * imgHeight_;
    validPointCountRatio_ = (float)(totalPointCount_ - invalidPointCount_) / totalPointCount_;

    return validPointCountRatio_;
}

void CImageFromCloud::rotateImage(float deg)
{
    cv::Point2f	pt(IMAGE_2D_WIDTH_CENTER,IMAGE_2D_HEIGHT_CENTER);
    double	scale	= 1.0;

    // get the transformation matrix 
    cv::Mat m = getRotationMatrix2D(pt,deg,scale);

    warpAffine(*pImgGS_, *pImgGS_, m, pImgGS_->size());		
    warpAffine(*pVisImg_, *pVisImg_, m, pVisImg_->size());	

    showIntensityImage(*pImgGS_,"RotatedImage");
}

void CImageFromCloud::showIntensityImage(cv::Mat_<uchar> img, string name)
{
    if ( !bEnableGraphics_ )
        return;

    cv::namedWindow(name.c_str());
    cv::imshow(name.c_str(), img);
    cv::waitKey(0);
}

void CImageFromCloud::showRGBImage(cv::Mat_<cv::Vec3b> img, string name)
{
    if ( !bEnableGraphics_ )
        return;

    cv::namedWindow(name.c_str());
    cv::imshow(name.c_str(), img);
    cv::waitKey(0);
}

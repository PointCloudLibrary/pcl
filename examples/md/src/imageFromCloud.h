#ifndef _CIMAGE_FROM_CLOUD_H_
#define _CIMAGE_FROM_CLOUD_H_

#include "opencv_includes.h"
#include "common_structs.h"


class CImageFromCloud
{
public:
    
    CImageFromCloud(bool bEnableGraphics);
    virtual ~CImageFromCloud() { delete pImgBGR_ ; delete pImgGS_ ; delete pImgDepth_ ; delete pVisImg_ ; }

    void	init2DImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, float deg = 0.0);
    void	rotateImage(float deg);

    float	getInvalidCountRatio();
    void	visualizeKeyPoints(std::vector<cv::Point2f> & keyPoints);

    void	showIntensityImage(cv::Mat_<uchar> img, string name);
    void	showRGBImage(cv::Mat_<cv::Vec3b> img, string name);

    cv::Mat_<uchar>	* getExtractedImage() { return pImgGS_; }
    inline unsigned getPointIdxFromPixelCoordinates(unsigned u, unsigned v) { return ( u + ( v * imgWidth_ ) ); }

private:

    pcPtr			pCloud_;
    typedef boost::shared_ptr< cv::Mat_<cv::Vec3b> > imgPtr;

    cv::Mat_<cv::Vec3b>	*	pImgBGR_;		
    cv::Mat_<uchar>	*	pImgGS_;	// gray scale for 2D keypoints/features
    cv::Mat_<uchar>	*	pImgDepth_;	// depth for visualization
    cv::Mat_<cv::Vec3b>	*	pVisImg_;       // hybrid image for visualization

    unsigned	                invalidPointCount_;
    unsigned	                totalPointCount_;
    float		        validPointCountRatio_;

    unsigned	                imgWidth_;
    unsigned	                imgHeight_;

    bool		        bEnableGraphics_;
};

#endif

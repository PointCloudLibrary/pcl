#ifndef  _PROCESS_FEATURES_2D_H_
#define  _PROCESS_FEATURES_2D_H_

#include "opencv_includes.h"
#include "common_structs.h"

class CProcessFeatures2D
{

public:
    CProcessFeatures2D(); 
    virtual ~CProcessFeatures2D () {}

    void    init(pcPtr pCloud, cv::Mat_<uchar> * pImgGS);
    void    setKeyPointType(keyPointType2D kpType);
    void    setDescriptorType(descriptorType dType) { descriptorType_ = dType; }

    void    computeKeypoints();
    void    computeDescriptors();
    void    computeAll();

    vector<cv::KeyPoint> & getKeypoints()  { return extractedKeyPoints_; }
    cv::Mat		   getDescriptors(); 

private:
    
    pcPtr			pCloud_;
    pcPtr			pFilteredCloud_;
    cv::Mat_<uchar>	*	pImgGS_;

    // structs for keypoints and descriptors

    keyPointType2D		keyPointType_;
    descriptorType		descriptorType_;

    vector<cv::KeyPoint>	extractedKeyPoints_;
    float			ratioValidKeyPoints_;

    cv::Mat			extractedDescriptors_;
    cv::Ptr<cv::Feature2D>	fdd_;
};

#endif

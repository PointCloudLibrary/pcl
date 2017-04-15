#ifndef _CMODEL_QUERY_2D_H_
#define _CMODEL_QUERY_2D_H_

#include "processFeatures2D.h"
#include "imageFromCloud.h"
#include "opencv_includes.h"

class CModelQuery2D 
{
public:
    
    CModelQuery2D(inputArgs     inputParams,
                  outputPerf *	pGlobalPerf,
                  pcPtr		pModelCloud, CImageFromCloud * pModelImage,
                  pcPtr		pQueryCloud, CImageFromCloud * pQueryImage ) 

    :	inputParams_(inputParams)
    ,	pGlobalPerf_(pGlobalPerf)
    ,	pModelImage_(pModelImage)
    ,	pQueryImage_(pQueryImage)
    ,	pModelCloud_(pModelCloud)
    ,   pQueryCloud_(pQueryCloud)
    ,   modelFeatures_() 
    ,	queryFeatures_() 
    {
        best_k_sorted_2D_ = inputParams.best_k_sorted_2D;
        perfData_.actualRotation = inputParams.rot_deg;
        minDistance_ = 100.0;
    }

    void init()
    {
        modelFeatures_.init(pModelCloud_, pModelImage_->getExtractedImage() );
        queryFeatures_.init(pQueryCloud_, pQueryImage_->getExtractedImage() );
    }

    void setKeyPointType ( keyPointType2D kpType) 
    {
        keyPointType2D_ = kpType;
        modelFeatures_.setKeyPointType(kpType); 
        queryFeatures_.setKeyPointType(kpType); 
    }
        
    void setDescriptorType ( descriptorType dType ) 
    {
        descriptorType_ = dType;
        modelFeatures_.setDescriptorType(dType); 
        queryFeatures_.setDescriptorType(dType); 
    }

    struct sort2DCorrespondencesByDistance : public std::binary_function<cv::DMatch, cv::DMatch, bool>
    {
        bool 
        operator()( cv::DMatch a, cv::DMatch b)
        {
        return (a.distance < b.distance);
        }
    };

    void computeKeyPoints();
    void computeDescriptors();
    void computeMatches();
    void computePoseEstimation();
    void visualizeCorrespondences();
    void computeAll();

    vector<cv::KeyPoint> & getModelKeyPoints() { return modelFeatures_.getKeypoints(); }
    vector<cv::KeyPoint> & getQueryKeyPoints() { return queryFeatures_.getKeypoints(); }
    std::vector<cv::DMatch> & getCorrespondences() { return matchingResults_; }

private:
    inputArgs			inputParams_;
    keyPointType2D		keyPointType2D_;
    descriptorType		descriptorType_;

    pcPtr			pModelCloud_;
    pcPtr			pQueryCloud_;
    CProcessFeatures2D		modelFeatures_;
    CProcessFeatures2D		queryFeatures_;

    std::vector<cv::DMatch>	matchingResults_;
    std::vector<cv::Point2f>    modelPoints_;
    std::vector<cv::Point2f>    queryPoints_;
    cv::Mat			H_;

    float			minDistance_;
    unsigned			best_k_sorted_2D_;

    CImageFromCloud    *        pModelImage_;	
    CImageFromCloud    *        pQueryImage_;	

    // perf analysis 
    perfData			perfData_;
    outputPerf		   *	pGlobalPerf_;
};

#endif

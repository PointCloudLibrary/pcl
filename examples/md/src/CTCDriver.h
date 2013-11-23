#ifndef _CTCDriver_H_
#define _CTCDriver_H_

#include "common_structs.h"
#include "imageFromCloud.h"
#include "modelQuery2D.h"
#include "modelQuery3D.h"
#include "multiDescriptorVoting.h"

class CTCDriver
{

public:

    typedef boost::shared_ptr<CModelQuery2D> mqPtr2D;
    typedef boost::shared_ptr<CModelQuery3D> mqPtr3D;

    CTCDriver(inputArgs & inputParams);
    virtual ~CTCDriver() { delete pModelQuery3D_; }

    void setKeyPoint2DType(keyPointType2D keyPointType);
    void setDescriptor2DType(descriptorType dType); 
    void setMinKeyPoint2DCount(unsigned minCount)				{ minKeyPointCount2D_ = minCount; }
    void setMinKeyPoint3DCount(unsigned minCount)				{ minKeyPointCount3D_ = minCount; }
    
    //
    // init model and query images
    //
    void init2D();
    void init2DFeatureObject(keyPointType2D kpType, descriptorType dType);
    void init3DFeatureObject();

    void compute2DKeyPoints();
    void compute2DDescriptors();
    void compute2DMatches();

    void computeAll();
    void computeAll2D();
    void computeAll3D();
    void logPerfDataToFile();
    void initiateVoteProcessing();
    void compute_MD2dR();

    void logPerfDataToFile(vector<perfData> & pd);

private:

    inputArgs		        &	inputParams_;
    pcPtr			&	pModelCloud_;
    pcPtr			&	pQueryCloud_;
    float				leafSize_;
    float				rotDeg_;
    unsigned			        best_k_sorted_2D_;
    unsigned		                best_k_sorted_3D_;

    CImageFromCloud			modelImage_;
    CImageFromCloud			queryImage_;

    unsigned				minKeyPointCount2D_;
    unsigned				minKeyPointCount3D_;

    string				mFileName_;
    string				qFileName_;

    vector<CModelQuery2D>	        modelQuery2D_;
    perfData				perfDataMD2dR_;
    perfData				perfDataMD2aR_;

    outputPerf				globalPerf_;

    CModelQuery3D	*		pModelQuery3D_;

    ofstream		*		logPerfData_;
    ofstream		*		logResults_;
};

#endif

#ifndef  _PROCESS_FEATURES_3D_H_
#define  _PROCESS_FEATURES_3D_H_

#include "common_structs.h"

class CProcessFeatures3D
{

public:
    CProcessFeatures3D( inputArgs & inputParams, pcPtr  pCloud, pcPtr  pSearchSurface );
    virtual ~CProcessFeatures3D() {}	

    void	setKeyPointType(keyPointType3D keyPointType);
    void	setDescriptorType(descriptorType3D descriptorType);

    void	computeNormals();
    unsigned	computeKeypoints();
    void	removeDuplicateKeyPoints();
    
    void	computeFPFHDescriptor();
    void	computeSHOTDescriptor();
    void	computeCSHOTDescriptor();

    void	computeDescriptors();
    double	getProcessingTime(unsigned i) { return vProcessingTime[i]; }

    void	importComputedNormals	(pcPtr pNormals, float normalsRadius);
    void	importComputedKeypoints	(pcPtr pUnfilteredKeypoints,
                                         pcPtr pFilteredKeypoints,
                                         keyPointType3D  kpType);
    // get computed & filtered keypoints
    pcPtr	getKeypoints()				{ return filteredKeypoints_; }

    // get computed descriptors
    fpfhDescriptorsPtr	getComputedFPFHDesc()	{ return fpfhDesc_; }
    shotDescriptorsPtr	getComputedSHOTDesc()	{ return shotDesc_; }
    cshotDescriptorsPtr	getComputedCSHOTDesc()	{ return cshotDesc_; }

    // shorthand to compute all of the above
    void	computeAll();

private:
    inputArgs		&	inputParams_;

    pcPtr			pCloud_;		// cloud to use for keypoint computation
    pcPtr			pSearchSurface_;	// original cloud before downsampling

    // keypoint related 
    bool			bValidNormals_;
    SurfaceNormalsPtr		normals_;
    float			normalsRadius_;

    bool			bValidKeypoints_;
    keyPointType3D		keyPointType_;
    pcPtr			filteredKeypoints_;	// redundant keypoints removed
    pcPtr			unfilteredKeypoints_;	// computed keypoints without filtering
    float			filteredKeypointsRatio_; 

    // descriptor related
    descriptorType3D		descriptorType_;
    float			descriptorRadius_;

    fpfhDescriptorsPtr		fpfhDesc_;
    shotDescriptorsPtr		shotDesc_;
    cshotDescriptorsPtr		cshotDesc_;

    vector<double>		vProcessingTime;
    double			processingTimeKPs_;
    double			processingTimeNormals_;
    double			processingTimeNormalsKPs_;
};

#endif

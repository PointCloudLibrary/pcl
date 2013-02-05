/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author: Koen Buys
 */

#include <pcl/gpu/people/face_detector.h>
#include <pcl/console/print.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/point_types_conversion.h>

#include <cuda_runtime_api.h>

#define NVBIN_HAAR_SIZERESERVED     16
#define NVBIN_HAAR_VERSION          0x1

pcl::gpu::people::FaceDetector::FaceDetector(int cols, int rows)
{
  PCL_DEBUG("[pcl::gpu::people::FaceDetector::FaceDetector] : (D) : Constructor called\n");

  cols_ = cols; rows_ = rows;

  cuda_dev_id_ = 0;
  cudaSafeCall ( cudaSetDevice (cuda_dev_id_));
  cudaSafeCall ( cudaGetDeviceProperties (&cuda_dev_prop_, cuda_dev_id_));
  PCL_DEBUG("[pcl::gpu::people::FaceDetector::FaceDetector] : (D) : Using GPU: %d ( %s ), arch= %d . %d\n",cuda_dev_id_, cuda_dev_prop_.name, cuda_dev_prop_.major, cuda_dev_prop_.minor);

}

NCVStatus
pcl::gpu::people::FaceDetector::loadFromXML(const std::string &filename,
                                            HaarClassifierCascadeDescriptor &haar,
                                            std::vector<HaarStage64> &haarStages,
                                            std::vector<HaarClassifierNode128> &haarClassifierNodes,
                                            std::vector<HaarFeature64> &haarFeatures)
{
  // TODO fix this part
  /*
    NCVStatus ncvStat;

    haar.NumStages = 0;
    haar.NumClassifierRootNodes = 0;
    haar.NumClassifierTotalNodes = 0;
    haar.NumFeatures = 0;
    haar.ClassifierSize.width = 0;
    haar.ClassifierSize.height = 0;
    haar.bNeedsTiltedII = false;
    haar.bHasStumpsOnly = false;

    FILE *fp;
    fopen_s(&fp, filename.c_str(), "r");
    ncvAssertReturn(fp != NULL, NCV_FILE_ERROR);

    //get file size
    fseek(fp, 0, SEEK_END);
    Ncv32u xmlSize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    //load file to vector
    std::vector<char> xmlFileCont;
    xmlFileCont.resize(xmlSize+1);
    memset(&xmlFileCont[0], 0, xmlSize+1);
    fread_s(&xmlFileCont[0], xmlSize, 1, xmlSize, fp);
    fclose(fp);

    haar.bHasStumpsOnly = true;
    haar.bNeedsTiltedII = false;
    Ncv32u curMaxTreeDepth;

    std::vector<HaarClassifierNode128> h_TmpClassifierNotRootNodes;
    haarStages.resize(0);
    haarClassifierNodes.resize(0);
    haarFeatures.resize(0);

    //XML loading and OpenCV XML classifier syntax verification
    try
    {
        rapidxml::xml_document<> doc;
        doc.parse<0>(&xmlFileCont[0]);

        //opencv_storage
        rapidxml::xml_node<> *parserGlobal = doc.first_node();
        ncvAssertReturn(!strcmp(parserGlobal->name(), "opencv_storage"), NCV_HAAR_XML_LOADING_EXCEPTION);

        //classifier type
        parserGlobal = parserGlobal->first_node();
        ncvAssertReturn(parserGlobal, NCV_HAAR_XML_LOADING_EXCEPTION);
        rapidxml::xml_attribute<> *attr = parserGlobal->first_attribute("type_id");
        ncvAssertReturn(!strcmp(attr->value(), "opencv-haar-classifier"), NCV_HAAR_XML_LOADING_EXCEPTION);

        //classifier size
        parserGlobal = parserGlobal->first_node("size");
        ncvAssertReturn(parserGlobal, NCV_HAAR_XML_LOADING_EXCEPTION);
        sscanf_s(parserGlobal->value(), "%d %d", &(haar.ClassifierSize.width), &(haar.ClassifierSize.height));

        //parse stages
        parserGlobal = parserGlobal->next_sibling("stages");
        ncvAssertReturn(parserGlobal, NCV_HAAR_XML_LOADING_EXCEPTION);
        parserGlobal = parserGlobal->first_node("_");
        ncvAssertReturn(parserGlobal, NCV_HAAR_XML_LOADING_EXCEPTION);

        while (parserGlobal)
        {
            HaarStage64 curStage;
            curStage.setStartClassifierRootNodeOffset(haarClassifierNodes.size());
            Ncv32u tmpNumClassifierRootNodes = 0;

            rapidxml::xml_node<> *parserStageThreshold = parserGlobal->first_node("stage_threshold");
            ncvAssertReturn(parserStageThreshold, NCV_HAAR_XML_LOADING_EXCEPTION);
            Ncv32f tmpStageThreshold;
            sscanf_s(parserStageThreshold->value(), "%f", &tmpStageThreshold);
            curStage.setStageThreshold(tmpStageThreshold);

            //parse trees
            rapidxml::xml_node<> *parserTree;
            parserTree = parserGlobal->first_node("trees");
            ncvAssertReturn(parserTree, NCV_HAAR_XML_LOADING_EXCEPTION);
            parserTree = parserTree->first_node("_");
            ncvAssertReturn(parserTree, NCV_HAAR_XML_LOADING_EXCEPTION);

            while (parserTree)
            {
                rapidxml::xml_node<> *parserNode;
                parserNode = parserTree->first_node("_");
                ncvAssertReturn(parserNode, NCV_HAAR_XML_LOADING_EXCEPTION);
                Ncv32u nodeId = 0;

                while (parserNode)
                {
                    HaarClassifierNode128 curNode;

                    rapidxml::xml_node<> *parserNodeThreshold = parserNode->first_node("threshold");
                    ncvAssertReturn(parserNodeThreshold, NCV_HAAR_XML_LOADING_EXCEPTION);
                    Ncv32f tmpThreshold;
                    sscanf_s(parserNodeThreshold->value(), "%f", &tmpThreshold);
                    curNode.setThreshold(tmpThreshold);

                    rapidxml::xml_node<> *parserNodeLeft = parserNode->first_node("left_val");
                    HaarClassifierNodeDescriptor32 nodeLeft;
                    if (parserNodeLeft)
                    {
                        Ncv32f leftVal;
                        sscanf_s(parserNodeLeft->value(), "%f", &leftVal);
                        ncvStat = nodeLeft.create(leftVal);
                        ncvAssertReturn(ncvStat == NCV_SUCCESS, ncvStat);
                    }
                    else
                    {
                        parserNodeLeft = parserNode->first_node("left_node");
                        ncvAssertReturn(parserNodeLeft, NCV_HAAR_XML_LOADING_EXCEPTION);
                        Ncv32u leftNodeOffset;
                        sscanf_s(parserNodeLeft->value(), "%d", &leftNodeOffset);
                        nodeLeft.create(h_TmpClassifierNotRootNodes.size() + leftNodeOffset - 1);
                        haar.bHasStumpsOnly = false;
                    }
                    curNode.setLeftNodeDesc(nodeLeft);

                    rapidxml::xml_node<> *parserNodeRight = parserNode->first_node("right_val");
                    HaarClassifierNodeDescriptor32 nodeRight;
                    if (parserNodeRight)
                    {
                        Ncv32f rightVal;
                        sscanf_s(parserNodeRight->value(), "%f", &rightVal);
                        ncvStat = nodeRight.create(rightVal);
                        ncvAssertReturn(ncvStat == NCV_SUCCESS, ncvStat);
                    }
                    else
                    {
                        parserNodeRight = parserNode->first_node("right_node");
                        ncvAssertReturn(parserNodeRight, NCV_HAAR_XML_LOADING_EXCEPTION);
                        Ncv32u rightNodeOffset;
                        sscanf_s(parserNodeRight->value(), "%d", &rightNodeOffset);
                        nodeRight.create(h_TmpClassifierNotRootNodes.size() + rightNodeOffset - 1);
                        haar.bHasStumpsOnly = false;
                    }
                    curNode.setRightNodeDesc(nodeRight);

                    rapidxml::xml_node<> *parserNodeFeatures = parserNode->first_node("feature");
                    ncvAssertReturn(parserNodeFeatures, NCV_HAAR_XML_LOADING_EXCEPTION);

                    rapidxml::xml_node<> *parserNodeFeaturesTilted = parserNodeFeatures->first_node("tilted");
                    ncvAssertReturn(parserNodeFeaturesTilted, NCV_HAAR_XML_LOADING_EXCEPTION);
                    Ncv32u tiltedVal;
                    sscanf_s(parserNodeFeaturesTilted->value(), "%d", &tiltedVal);
                    haar.bNeedsTiltedII = (tiltedVal != 0);

                    rapidxml::xml_node<> *parserNodeFeaturesRects = parserNodeFeatures->first_node("rects");
                    ncvAssertReturn(parserNodeFeaturesRects, NCV_HAAR_XML_LOADING_EXCEPTION);
                    parserNodeFeaturesRects = parserNodeFeaturesRects->first_node("_");
                    ncvAssertReturn(parserNodeFeaturesRects, NCV_HAAR_XML_LOADING_EXCEPTION);
                    Ncv32u featureId = 0;

                    while (parserNodeFeaturesRects)
                    {
                        Ncv32u rectX, rectY, rectWidth, rectHeight;
                        Ncv32f rectWeight;
                        sscanf_s(parserNodeFeaturesRects->value(), "%d %d %d %d %f", &rectX, &rectY, &rectWidth, &rectHeight, &rectWeight);
                        HaarFeature64 curFeature;
                        ncvStat = curFeature.setRect(rectX, rectY, rectWidth, rectHeight, haar.ClassifierSize.width, haar.ClassifierSize.height);
                        curFeature.setWeight(rectWeight);
                        ncvAssertReturn(NCV_SUCCESS == ncvStat, ncvStat);
                        haarFeatures.push_back(curFeature);

                        parserNodeFeaturesRects = parserNodeFeaturesRects->next_sibling("_");
                        featureId++;
                    }

                    HaarFeatureDescriptor32 tmpFeatureDesc;
                    ncvStat = tmpFeatureDesc.create(haar.bNeedsTiltedII, featureId, haarFeatures.size() - featureId);
                    ncvAssertReturn(NCV_SUCCESS == ncvStat, ncvStat);
                    curNode.setFeatureDesc(tmpFeatureDesc);

                    if (!nodeId)
                    {
                        //root node
                        haarClassifierNodes.push_back(curNode);
                        curMaxTreeDepth = 1;
                    }
                    else
                    {
                        //other node
                        h_TmpClassifierNotRootNodes.push_back(curNode);
                        curMaxTreeDepth++;
                    }

                    parserNode = parserNode->next_sibling("_");
                    nodeId++;
                }

                parserTree = parserTree->next_sibling("_");
                tmpNumClassifierRootNodes++;
            }

            curStage.setNumClassifierRootNodes(tmpNumClassifierRootNodes);
            haarStages.push_back(curStage);

            parserGlobal = parserGlobal->next_sibling("_");
        }
    }
    catch (...)
    {
        return NCV_HAAR_XML_LOADING_EXCEPTION;
    }

    //fill in cascade stats
    haar.NumStages = haarStages.size();
    haar.NumClassifierRootNodes = haarClassifierNodes.size();
    haar.NumClassifierTotalNodes = haar.NumClassifierRootNodes + h_TmpClassifierNotRootNodes.size();
    haar.NumFeatures = haarFeatures.size();

    //merge root and leaf nodes in one classifiers array
    Ncv32u offsetRoot = haarClassifierNodes.size();
    for (Ncv32u i=0; i<haarClassifierNodes.size(); i++)
    {
        HaarClassifierNodeDescriptor32 nodeLeft = haarClassifierNodes[i].getLeftNodeDesc();
        if (!nodeLeft.isLeaf())
        {
            Ncv32u newOffset = nodeLeft.getNextNodeOffset() + offsetRoot;
            nodeLeft.create(newOffset);
        }
        haarClassifierNodes[i].setLeftNodeDesc(nodeLeft);

        HaarClassifierNodeDescriptor32 nodeRight = haarClassifierNodes[i].getRightNodeDesc();
        if (!nodeRight.isLeaf())
        {
            Ncv32u newOffset = nodeRight.getNextNodeOffset() + offsetRoot;
            nodeRight.create(newOffset);
        }
        haarClassifierNodes[i].setRightNodeDesc(nodeRight);
    }
    for (Ncv32u i=0; i<h_TmpClassifierNotRootNodes.size(); i++)
    {
        HaarClassifierNodeDescriptor32 nodeLeft = h_TmpClassifierNotRootNodes[i].getLeftNodeDesc();
        if (!nodeLeft.isLeaf())
        {
            Ncv32u newOffset = nodeLeft.getNextNodeOffset() + offsetRoot;
            nodeLeft.create(newOffset);
        }
        h_TmpClassifierNotRootNodes[i].setLeftNodeDesc(nodeLeft);

        HaarClassifierNodeDescriptor32 nodeRight = h_TmpClassifierNotRootNodes[i].getRightNodeDesc();
        if (!nodeRight.isLeaf())
        {
            Ncv32u newOffset = nodeRight.getNextNodeOffset() + offsetRoot;
            nodeRight.create(newOffset);
        }
        h_TmpClassifierNotRootNodes[i].setRightNodeDesc(nodeRight);

        haarClassifierNodes.push_back(h_TmpClassifierNotRootNodes[i]);
    }
  */
  return NCV_SUCCESS;
}

/**
 * \brief This loads the Haar description file from a NVBIN file format
 */
NCVStatus
pcl::gpu::people::FaceDetector::loadFromNVBIN(const std::string &filename,
                               HaarClassifierCascadeDescriptor &haar,
                               std::vector<HaarStage64> &haarStages,
                               std::vector<HaarClassifierNode128> &haarClassifierNodes,
                               std::vector<HaarFeature64> &haarFeatures)
{
    size_t readCount;
    FILE *fp = fopen(filename.c_str(), "rb");
    ncvAssertReturn(fp != NULL, NCV_FILE_ERROR);
    Ncv32u fileVersion;
    readCount = fread(&fileVersion, sizeof(Ncv32u), 1, fp);
    PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "return NCV_FILE_ERROR", NCV_FILE_ERROR);
    PCL_ASSERT_ERROR_PRINT_RETURN(fileVersion == NVBIN_HAAR_VERSION, "return NCV_FILE_ERROR", NCV_FILE_ERROR);
    Ncv32u fsize;
    readCount = fread(&fsize, sizeof(Ncv32u), 1, fp);
    PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "return NCV_FILE_ERROR", NCV_FILE_ERROR);
    fseek(fp, 0, SEEK_END);
    Ncv32u fsizeActual = ftell(fp);
    PCL_ASSERT_ERROR_PRINT_RETURN(fsize == fsizeActual, "return NCV_FILE_ERROR", NCV_FILE_ERROR);

    std::vector<unsigned char> fdata;
    fdata.resize(fsize);
    Ncv32u dataOffset = 0;
    fseek(fp, 0, SEEK_SET);
    readCount = fread(&fdata[0], fsize, 1, fp);
    PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "return NCV_FILE_ERROR", NCV_FILE_ERROR);
    fclose(fp);

    //data
    dataOffset = NVBIN_HAAR_SIZERESERVED;
    haar.NumStages = *(Ncv32u *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(Ncv32u);
    haar.NumClassifierRootNodes = *(Ncv32u *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(Ncv32u);
    haar.NumClassifierTotalNodes = *(Ncv32u *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(Ncv32u);
    haar.NumFeatures = *(Ncv32u *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(Ncv32u);
    haar.ClassifierSize = *(NcvSize32u *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(NcvSize32u);
    haar.bNeedsTiltedII = *(NcvBool *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(NcvBool);
    haar.bHasStumpsOnly = *(NcvBool *)(&fdata[0]+dataOffset);
    dataOffset += sizeof(NcvBool);

    haarStages.resize(haar.NumStages);
    haarClassifierNodes.resize(haar.NumClassifierTotalNodes);
    haarFeatures.resize(haar.NumFeatures);

    Ncv32u szStages = haar.NumStages * sizeof(HaarStage64);
    Ncv32u szClassifiers = haar.NumClassifierTotalNodes * sizeof(HaarClassifierNode128);
    Ncv32u szFeatures = haar.NumFeatures * sizeof(HaarFeature64);

    memcpy(&haarStages[0], &fdata[0]+dataOffset, szStages);
    dataOffset += szStages;
    memcpy(&haarClassifierNodes[0], &fdata[0]+dataOffset, szClassifiers);
    dataOffset += szClassifiers;
    memcpy(&haarFeatures[0], &fdata[0]+dataOffset, szFeatures);
    dataOffset += szFeatures;

    return NCV_SUCCESS;
}

/*
 * \brief Depending on file format load the Haar description file
 */
NCVStatus
pcl::gpu::people::FaceDetector::ncvHaarLoadFromFile_host(const std::string &filename,
                                   HaarClassifierCascadeDescriptor &haar,
                                   NCVVector<HaarStage64> &h_HaarStages,
                                   NCVVector<HaarClassifierNode128> &h_HaarNodes,
                                   NCVVector<HaarFeature64> &h_HaarFeatures)
{
    PCL_ASSERT_ERROR_PRINT_RETURN(h_HaarStages.memType() == NCVMemoryTypeHostPinned &&
                                  h_HaarNodes.memType() == NCVMemoryTypeHostPinned &&
                                  h_HaarFeatures.memType() == NCVMemoryTypeHostPinned, "return NCV_MEM_RESIDENCE_ERROR", NCV_MEM_RESIDENCE_ERROR);

    NCVStatus ncvStat;

    std::string fext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(fext.begin(), fext.end(), fext.begin(), ::tolower);

    std::vector<HaarStage64> haarStages;
    std::vector<HaarClassifierNode128> haarNodes;
    std::vector<HaarFeature64> haarFeatures;

    if (fext == "nvbin")
    {
        ncvStat = loadFromNVBIN(filename, haar, haarStages, haarNodes, haarFeatures);
        ncvAssertReturnNcvStat(ncvStat);  // Todo replace this
    }
    else if (fext == "xml")
    {
        ncvStat = loadFromXML(filename, haar, haarStages, haarNodes, haarFeatures);
        ncvAssertReturnNcvStat(ncvStat);  // Todo replace this
    }
    else
    {
        return NCV_HAAR_XML_LOADING_EXCEPTION;
    }

    PCL_ASSERT_ERROR_PRINT_RETURN(h_HaarStages.length() >= haarStages.size(), "Return NCV_MEM_INSUFFICIENT_CAPACITY", NCV_MEM_INSUFFICIENT_CAPACITY);
    PCL_ASSERT_ERROR_PRINT_RETURN(h_HaarNodes.length() >= haarNodes.size(), "Return NCV_MEM_INSUFFICIENT_CAPACITY", NCV_MEM_INSUFFICIENT_CAPACITY);
    PCL_ASSERT_ERROR_PRINT_RETURN(h_HaarFeatures.length() >= haarFeatures.size(), "Return NCV_MEM_INSUFFICIENT_CAPACITY", NCV_MEM_INSUFFICIENT_CAPACITY);

    memcpy(h_HaarStages.ptr(), &haarStages[0], haarStages.size()*sizeof(HaarStage64));
    memcpy(h_HaarNodes.ptr(), &haarNodes[0], haarNodes.size()*sizeof(HaarClassifierNode128));
    memcpy(h_HaarFeatures.ptr(), &haarFeatures[0], haarFeatures.size()*sizeof(HaarFeature64));

    return NCV_SUCCESS;
}

NCVStatus
pcl::gpu::people::FaceDetector::ncvHaarGetClassifierSize(const std::string &filename, Ncv32u &numStages,
                                                         Ncv32u &numNodes, Ncv32u &numFeatures)
{
    size_t readCount;
    NCVStatus ncvStat;

    std::string fext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(fext.begin(), fext.end(), fext.begin(), ::tolower);

    if (fext == "nvbin")
    {
        FILE *fp = fopen(filename.c_str(), "rb");
        ncvAssertReturn(fp != NULL, NCV_FILE_ERROR);
        Ncv32u fileVersion;
        readCount = fread(&fileVersion, sizeof(Ncv32u), 1, fp);
        ncvAssertReturn(1 == readCount, NCV_FILE_ERROR);
        ncvAssertReturn(fileVersion == NVBIN_HAAR_VERSION, NCV_FILE_ERROR);
        fseek(fp, NVBIN_HAAR_SIZERESERVED, SEEK_SET);
        Ncv32u tmp;
        readCount = fread(&numStages,   sizeof(Ncv32u), 1, fp);
        ncvAssertReturn(1 == readCount, NCV_FILE_ERROR);
        readCount = fread(&tmp,         sizeof(Ncv32u), 1, fp);
        ncvAssertReturn(1 == readCount, NCV_FILE_ERROR);
        readCount = fread(&numNodes,    sizeof(Ncv32u), 1, fp);
        ncvAssertReturn(1 == readCount, NCV_FILE_ERROR);
        readCount = fread(&numFeatures, sizeof(Ncv32u), 1, fp);
        ncvAssertReturn(1 == readCount, NCV_FILE_ERROR);
        fclose(fp);
    }
    else if (fext == "xml")
    {
        HaarClassifierCascadeDescriptor haar;
        std::vector<HaarStage64> haarStages;
        std::vector<HaarClassifierNode128> haarNodes;
        std::vector<HaarFeature64> haarFeatures;

        ncvStat = loadFromXML(filename, haar, haarStages, haarNodes, haarFeatures);
        ncvAssertReturnNcvStat(ncvStat);

        numStages = haar.NumStages;
        numNodes = haar.NumClassifierTotalNodes;
        numFeatures = haar.NumFeatures;
    }
    else
    {
        return NCV_HAAR_XML_LOADING_EXCEPTION;
    }

    return NCV_SUCCESS;
}

NCVStatus
pcl::gpu::people::FaceDetector::NCVprocess(pcl::PointCloud<pcl::RGB> input,
                                           //Mat *srcdst,
                                           Ncv32u width,
                                           Ncv32u height,
                                           NcvBool bFilterRects,
                                           NcvBool bLargestFace,
                                           HaarClassifierCascadeDescriptor &haar,
                                           NCVVector<HaarStage64> &d_haarStages,
                                           NCVVector<HaarClassifierNode128> &d_haarNodes,
                                           NCVVector<HaarFeature64> &d_haarFeatures,
                                           NCVVector<HaarStage64> &h_haarStages,
                                           INCVMemAllocator &gpuAllocator,
                                           INCVMemAllocator &cpuAllocator,
                                           cudaDeviceProp &devProp)
{
  // TODO fix this part

  pcl::PointCloud<Intensity32u>  input_gray;
  PointCloudRGBtoI(input, input_gray);

  //PCL_ASSERT_ERROR_PRINT_RETURN(!((srcdst == NULL) ^ gpuAllocator.isCounting()),"retcode=" << (int)NCV_NULL_PTR, NCV_NULL_PTR);

  NCVStatus ncvStat;

  NCV_SET_SKIP_COND(gpuAllocator.isCounting());

  NCVMatrixAlloc<Ncv8u> d_src(gpuAllocator, width, height);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_src.isMemAllocated(), "retcode=NCV_ALLOCATOR_BAD_ALLOC", NCV_ALLOCATOR_BAD_ALLOC);
  NCVMatrixAlloc<Ncv8u> h_src(cpuAllocator, width, height);
  PCL_ASSERT_ERROR_PRINT_RETURN(h_src.isMemAllocated(), "retcode=NCV_ALLOCATOR_BAD_ALLOC", NCV_ALLOCATOR_BAD_ALLOC);
  NCVVectorAlloc<NcvRect32u> d_rects(gpuAllocator, 100);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_rects.isMemAllocated(), "retcode=NCV_ALLOCATOR_BAD_ALLOC", NCV_ALLOCATOR_BAD_ALLOC);

  NCV_SKIP_COND_BEGIN

  /*
  for (Ncv32u i=0; i<(Ncv32u)input.height; i++)
  {
    memcpy(h_src.ptr() + i * h_src.stride(), srcdst->ptr(i), input.width);
  }
  */
  for(int i=0; i<input.points.size(); i++)
  {
 //   memcpy(h_src.ptr(), input.points[i], sizeof());
  }

  ncvStat = h_src.copySolid(d_src, 0);
  ncvAssertReturnNcvStat(ncvStat);
  ncvAssertCUDAReturn(cudaStreamSynchronize(0), NCV_CUDA_ERROR);

  NCV_SKIP_COND_END

  NcvSize32u roi;
  roi.width = d_src.width();
  roi.height = d_src.height();

  Ncv32u numDetections;
  ncvStat = ncvDetectObjectsMultiScale_device(
        d_src, roi, d_rects, numDetections, haar, h_haarStages,
        d_haarStages, d_haarNodes, d_haarFeatures,
        haar.ClassifierSize,
        (bFilterRects || bLargestFace) ? 4 : 0,
        1.2f, 1,
        (bLargestFace ? NCVPipeObjDet_FindLargestObject : 0)
        | NCVPipeObjDet_VisualizeInPlace,
        gpuAllocator, cpuAllocator, devProp, 0);

  //ncvAssertReturnNcvStat(ncvStat);
  //ncvAssertCUDAReturn(cudaStreamSynchronize(0), NCV_CUDA_ERROR);

  NCV_SKIP_COND_BEGIN

  ncvStat = d_src.copySolid(h_src, 0);
  //ncvAssertReturnNcvStat(ncvStat);
  //ncvAssertCUDAReturn(cudaStreamSynchronize(0), NCV_CUDA_ERROR);

  /*
  for (Ncv32u i=0; i<(Ncv32u)srcdst->rows; i++)
  {
    memcpy(srcdst->ptr(i), h_src.ptr() + i * h_src.stride(), srcdst->cols);
  }
  */
  NCV_SKIP_COND_END

  return NCV_SUCCESS;
}

/*
 * \brief This does the GPU allocations and configurations
 */
int
pcl::gpu::people::FaceDetector::configure(std::string cascade_file_name)
{
  cascade_file_name_ = cascade_file_name;

  // TODO: COPY VARIABLES TO CLASS VARIABLES

  // Load the classifier from file (assuming its size is about 1 mb), using a simple allocator
  NCVMemNativeAllocator gpuCascadeAllocator(NCVMemoryTypeDevice, static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(gpuCascadeAllocator.isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating cascade GPU allocator", -1);

  NCVMemNativeAllocator cpuCascadeAllocator(NCVMemoryTypeHostPinned, static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(cpuCascadeAllocator.isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating cascade CPU allocator", -1);

  NCVStatus ncvStat;
  Ncv32u haarNumStages, haarNumNodes, haarNumFeatures;

  ncvStat = ncvHaarGetClassifierSize(cascade_file_name_, haarNumStages, haarNumNodes, haarNumFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncvStat == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error reading classifier size (check the file)", -1);

  NCVVectorAlloc<HaarStage64> h_haarStages(cpuCascadeAllocator, haarNumStages);
  PCL_ASSERT_ERROR_PRINT_RETURN(h_haarStages.isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade CPU allocator", -1);
  NCVVectorAlloc<HaarClassifierNode128> h_haarNodes(cpuCascadeAllocator, haarNumNodes);
  PCL_ASSERT_ERROR_PRINT_RETURN(h_haarNodes.isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade CPU allocator", -1);
  NCVVectorAlloc<HaarFeature64> h_haarFeatures(cpuCascadeAllocator, haarNumFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(h_haarFeatures.isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade CPU allocator", -1);

  HaarClassifierCascadeDescriptor haar;
  ncvStat = ncvHaarLoadFromFile_host(cascade_file_name_, haar, h_haarStages, h_haarNodes, h_haarFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncvStat == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error loading classifier", -1);

  NCVVectorAlloc<HaarStage64> d_haarStages(gpuCascadeAllocator, haarNumStages);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_haarStages.isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade GPU allocator", -1);
  NCVVectorAlloc<HaarClassifierNode128> d_haarNodes(gpuCascadeAllocator, haarNumNodes);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_haarNodes.isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade GPU allocator", -1);
  NCVVectorAlloc<HaarFeature64> d_haarFeatures(gpuCascadeAllocator, haarNumFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_haarFeatures.isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade GPU allocator", -1);

  ncvStat = h_haarStages.copySolid(d_haarStages, 0);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncvStat == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error copying cascade to GPU", -1);
  ncvStat = h_haarNodes.copySolid(d_haarNodes, 0);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncvStat == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error copying cascade to GPU", -1);
  ncvStat = h_haarFeatures.copySolid(d_haarFeatures, 0);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncvStat == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error copying cascade to GPU", -1);

  // Calculate memory requirements and create real allocators
  NCVMemStackAllocator gpuCounter(static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(gpuCounter.isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating GPU memory counter", -1);
  NCVMemStackAllocator cpuCounter(static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(cpuCounter.isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating CPU memory counter", -1);

  // TODO fix this fake call
  /*
  ncvStat = NCVprocess (NULL, cols_, rows_,
                        false, false, haar,
                        d_haarStages, d_haarNodes,
                        d_haarFeatures, h_haarStages,
                        gpuCounter, cpuCounter, cuda_dev_prop_);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncvStat == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in memory counting pass", -1);
  */

  NCVMemStackAllocator gpuAllocator(NCVMemoryTypeDevice, gpuCounter.maxSize(), static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(gpuAllocator.isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating GPU memory allocator", -1);
  NCVMemStackAllocator cpuAllocator(NCVMemoryTypeHostPinned, cpuCounter.maxSize(), static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(cpuAllocator.isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating CPU memory allocator", -1);

  PCL_DEBUG("[pcl::gpu::people::FaceDetector::FaceDetector] : (D) : Initialized for frame size [%dx%d]\n", cols_, rows_);

  return 1;
}

void
pcl::gpu::people::FaceDetector::setDeviceId( int id )
{
  cuda_dev_id_ = id;
  cudaSafeCall ( cudaSetDevice (cuda_dev_id_));
  cudaSafeCall ( cudaGetDeviceProperties (&cuda_dev_prop_, cuda_dev_id_));
}

void
pcl::gpu::people::FaceDetector::process()
{

}

void
pcl::gpu::people::FaceDetector::allocate_buffers(int rows, int cols)
{

}

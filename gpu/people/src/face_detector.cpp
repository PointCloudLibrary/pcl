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

#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cuda_runtime_api.h>

#define NVBIN_HAAR_SIZERESERVED     16
#define NVBIN_HAAR_VERSION          0x1

#define PCL_ASSERT_NCVSTAT(ncvOp) \
    do \
    { \
        NCVStatus _ncv_return_status = ncvOp; \
        PCL_ASSERT_ERROR_PRINT_RETURN(NCV_SUCCESS==_ncv_return_status, "ncv_return_status!=NCV_SUCCESS", _ncv_return_status); \
    } while (0)

#define PCL_ASSERT_CUDA_RETURN(cudacall, errCode) \
    do \
    { \
        cudaError_t res = cudacall; \
        ncvAssertPrintReturn(cudaSuccess==res, "cudaError_t!=cudaSuccess", errCode); \
    } while (0)

using boost::property_tree::ptree;

pcl::gpu::people::FaceDetector::FaceDetector(int cols, int rows)
{
  PCL_DEBUG("[pcl::gpu::people::FaceDetector::FaceDetector] : (D) : Constructor called\n");

  cols_ = cols; rows_ = rows;

  cuda_dev_id_ = 0;
  cudaSafeCall ( cudaSetDevice (cuda_dev_id_));
  cudaSafeCall ( cudaGetDeviceProperties (&cuda_dev_prop_, cuda_dev_id_));
  PCL_DEBUG("[pcl::gpu::people::FaceDetector::FaceDetector] : (D) : Using GPU: %d ( %s ), arch= %d . %d\n",cuda_dev_id_, cuda_dev_prop_.name, cuda_dev_prop_.major, cuda_dev_prop_.minor);

}

/**
 * \brief This loads the Haar description file from a XML file format
 */
NCVStatus
pcl::gpu::people::FaceDetector::loadFromXML2(const std::string                   &filename,
                                             HaarClassifierCascadeDescriptor     &haar,
                                             std::vector<HaarStage64>            &haar_stages,
                                             std::vector<HaarClassifierNode128>  &haarClassifierNodes,
                                             std::vector<HaarFeature64>          &haar_features)
{
  NCVStatus ncv_return_status;      // TODO remove this type

  boost::property_tree::ptree pt;

  haar.NumStages = 0;
  haar.NumClassifierRootNodes = 0;
  haar.NumClassifierTotalNodes = 0;
  haar.NumFeatures = 0;
  haar.ClassifierSize.width = 0;
  haar.ClassifierSize.height = 0;

  try   // Fetch error loading the file
  {
    read_xml(filename,pt);
  }
  catch(boost::exception const&  exb)
  {
    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : Unable to read filename with boost exception\n");
    return NCV_HAAR_XML_LOADING_EXCEPTION;
  }
  catch (std::exception const&  ex)
  {
    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : Unable to read filename with exception %s\n", ex.what());
    return NCV_HAAR_XML_LOADING_EXCEPTION;
  }
  catch (...)
  {
    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : Unable to read filename\n");
    return NCV_HAAR_XML_LOADING_EXCEPTION;
  }

  haar.bHasStumpsOnly = true;
  haar.bNeedsTiltedII = false;

  Ncv32u cur_max_tree_depth;

  std::vector<HaarClassifierNode128> host_temp_classifier_not_root_nodes;
  haar_stages.resize(0);
  haarClassifierNodes.resize(0);
  haar_features.resize(0);

  try   // Fetch all parsing errors
  {
    int level1 = 0;     // For debug output only
    /// LEVEL1 (opencv_storage)
    BOOST_FOREACH(const ptree::value_type &top_node, pt)
    {
      if(!strcmp(top_node.first.c_str(), "opencv_storage"))      // Else NCV_HAAR_XML_LOADING_EXCEPTION
      {
        PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 1, XMLnode %d, first : %s\n", level1, top_node.first.c_str());

        int level2 = 0;
        boost::property_tree::ptree pt2 = top_node.second;

        /// LEVEL2 (haarcascade)
        BOOST_FOREACH(const ptree::value_type &w, pt2)
        {
          if(!strcmp(w.second.get("<xmlattr>.type_id","").c_str(), "opencv-haar-classifier"))                      // Else NCV_HAAR_XML_LOADING_EXCEPTION
          {
            PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 2, XMLnode %d, first : %s\n", level2, w.first.c_str());

            // Parse the size field, giving the classifier patch size
            std::string size_string  = w.second.get<std::string>("size");                                         // data field contains both width and height so we put it in string first
            std::cout << "loadFromXML2: level 2 : size string: " << size_string << std::endl;
            std::istringstream l( size_string );
            l >> std::skipws >> haar.ClassifierSize.width >> haar.ClassifierSize.height;
            if ( !l || haar.ClassifierSize.width <= 0 || haar.ClassifierSize.height <= 0 )
            {
              PCL_WARN("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 2 : size node format error\n");   //  format error: line doesn't start with an int.
              return (NCV_HAAR_XML_LOADING_EXCEPTION);
            }
            else
              PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level2 : size int1 %d, int2 %d\n", haar.ClassifierSize.width, haar.ClassifierSize.height);

            int level3 = 0;
            /// LEVEL3 (Stages)
            BOOST_FOREACH(const ptree::value_type &stage, w.second.get_child("stages"))
            {
              PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 3, XMLnode %d, first : %s\n", level3, stage.first.c_str());

              HaarStage64 current_stage;
              current_stage.setStartClassifierRootNodeOffset(haarClassifierNodes.size());
              Ncv32u tmp_num_classifier_root_nodes = 0;

              float stage_threshold = stage.second.get<float>("stage_threshold");
              int parent = stage.second.get<int>("parent");
              int next = stage.second.get<int>("next");

              current_stage.setStageThreshold(stage_threshold);

              PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 3 stage_threshold %f, parent %d, next %d\n", stage_threshold, parent, next);

              int level4 = 0;
              /// LEVEL4 (Trees)
              BOOST_FOREACH(const ptree::value_type &tree, stage.second.get_child("trees"))
              {
                PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 4, XMLnode %d, first : %s\n", level4, tree.first.c_str());
                Ncv32u node_identifier = 0;

                int level5 = 0;
                ptree root = tree.second;

                /// LEVEL5 (Root_node)
                BOOST_FOREACH(const ptree::value_type &root_node, root)
                {
                  PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 5, node %d, first : %s\n", level5, root_node.first.c_str());

                  if(!strcmp(root_node.first.c_str(), "_"))
                  {
                    HaarClassifierNode128 current_node;

                    float node_threshold = root_node.second.get<float>("threshold");
                    float left_val = root_node.second.get<float>("left_val");         // TODO Test if left node is available! see Nvidia code for solution
                    bool left_val_available = true;                                   // TODO set correctly
                    float right_val = root_node.second.get<float>("right_val");
                    bool right_val_available = true;                                  // TODO set correctly
                    bool tilted = root_node.second.get<bool>("feature.tilted");

                    current_node.setThreshold(node_threshold);

                    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 5 node_threshold %f, left_val %f, right_val %f, tilted %d\n", node_threshold, left_val, right_val, tilted);

                    HaarClassifierNodeDescriptor32 node_left;
                    ncv_return_status = node_left.create(left_val);                              // TODO check ncv_return_status return value line below and return
                    current_node.setLeftNodeDesc(node_left);

                    HaarClassifierNodeDescriptor32 node_right;
                    ncv_return_status = node_right.create(right_val);
                    current_node.setRightNodeDesc(node_right);

                    haar.bNeedsTiltedII = (tilted != 0);
                    Ncv32u feature_identifier = 0;

                    /// LEVEL6 (Rects)
                    BOOST_FOREACH(const ptree::value_type &rect, root_node.second.get_child("feature.rects"))
                    {
                      PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 6, first : %s\n", rect.first.c_str());

                      std::string r = rect.second.data();

                      std::istringstream re( r );
                      int rectangle_u = 0, rectangle_v = 0, rectangle_width = 0, rectangle_height =0;
                      float rectWeight = 0;
                      re >> std::skipws >> rectangle_u >> rectangle_v >> rectangle_width >> rectangle_height >> rectWeight;

                      if ( !re )
                      {
                        //  format error: line doesn't start with an int.
                        PCL_WARN("[pcl::gpu::people::FaceDetector::loadFromXML2] : (W) : level 6 : rect format error\n");
                      }
                      else
                      {
                        PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : level 6 : rectangle_u %d, rectangle_v %d, rectW %d, rectH %d, rectWeight %f\n", rectangle_u, rectangle_v, rectangle_width, rectangle_height, rectWeight);
                        HaarFeature64 current_feature;
                        ncv_return_status = current_feature.setRect(rectangle_u, rectangle_v, rectangle_width, rectangle_height, haar.ClassifierSize.width, haar.ClassifierSize.height);
                        current_feature.setWeight(rectWeight);
                        PCL_ASSERT_NCVSTAT(ncv_return_status);
                        haar_features.push_back(current_feature);
                        feature_identifier++;
                      }
                    }

                    HaarFeatureDescriptor32 temp_feature_descriptor;
                    ncv_return_status = temp_feature_descriptor.create(haar.bNeedsTiltedII, left_val_available, right_val_available, feature_identifier, haar_features.size() - feature_identifier);

                    //ncv_return_status = temp_feature_descriptor.create(haar.bNeedsTiltedII, feature_identifier, haar_features.size() - feature_identifier);
                    ncvAssertReturn(NCV_SUCCESS == ncv_return_status, ncv_return_status);
                    current_node.setFeatureDesc(temp_feature_descriptor);

                    if (!node_identifier)
                    {
                        //root node
                        haarClassifierNodes.push_back(current_node);
                        cur_max_tree_depth = 1;
                    }
                    else
                    {
                        //other node
                        host_temp_classifier_not_root_nodes.push_back(current_node);
                        // TODO replace with PCL_DEBUG in the future
                        PCL_INFO("[pcl::gpu::people::FaceDetector::loadFromXML2] : (I) : Found non root node number %d", host_temp_classifier_not_root_nodes.size());
                        cur_max_tree_depth++;
                    }
                    node_identifier++;
                  }
                  else
                    PCL_WARN("[pcl::gpu::people::FaceDetector::loadFromXML2] : (W) : Found fifth level node that is atypical : %s\n", root_node.first.c_str());
                  level5++;
                }
                tmp_num_classifier_root_nodes++;
                level4++;
              }

              current_stage.setNumClassifierRootNodes(tmp_num_classifier_root_nodes);
              haar_stages.push_back(current_stage);

              // TODO make this DEBUG later on
              PCL_INFO("[pcl::gpu::people::FaceDetector::loadFromXML2] : (I) : level 3 stage %d loaded with %d Root Nodes, %f Threshold, %d Root Node Offset", haar_stages.size(), tmp_num_classifier_root_nodes, current_stage.getStartClassifierRootNodeOffset());

              level3++;
            }
          }
          else   // Probably we stumbled upon a comment at this level or somebody adjusted the XML file
            PCL_WARN("loadFromXML2: Found second level node that is atypical : %s\n", w.first.c_str());
        }
        level2++;
      }
      else
      {
        PCL_WARN("[pcl::gpu::people::FaceDetector::loadFromXML2] : (W) : Found first level node that is atypical : %s\n", top_node.first.c_str());
        return (NCV_HAAR_XML_LOADING_EXCEPTION);
      }
      level1++;
    }
  }
  catch(boost::exception const&  exb)
  {
    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : Unable to process content with boost exception\n");
    return (NCV_HAAR_XML_LOADING_EXCEPTION);
  }
  catch (std::exception const&  ex)
  {
    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : Unable to process content with std exception %s\n", ex.what());
    return (NCV_HAAR_XML_LOADING_EXCEPTION);
  }
  catch (...)
  {
    PCL_DEBUG("[pcl::gpu::people::FaceDetector::loadFromXML2] : (D) : Unable to process content\n");
    return (NCV_HAAR_XML_LOADING_EXCEPTION);
  }

  //fill in cascade stats
  haar.NumStages = haar_stages.size();
  haar.NumClassifierRootNodes = haarClassifierNodes.size();
  haar.NumClassifierTotalNodes = haar.NumClassifierRootNodes + host_temp_classifier_not_root_nodes.size();
  haar.NumFeatures = haar_features.size();

  //merge root and leaf nodes in one classifiers array, leaf nodes are sorted behind the root nodes
  Ncv32u offset_root = haarClassifierNodes.size();

  for (Ncv32u i=0; i<haarClassifierNodes.size(); i++)
  {
      HaarClassifierNodeDescriptor32 node_left = haarClassifierNodes[i].getLeftNodeDesc();
      if (!node_left.isLeaf())
      {
          Ncv32u new_offset = node_left.getNextNodeOffset() + offset_root;
          node_left.create(new_offset);
      }
      haarClassifierNodes[i].setLeftNodeDesc(node_left);

      HaarClassifierNodeDescriptor32 node_right = haarClassifierNodes[i].getRightNodeDesc();
      if (!node_right.isLeaf())
      {
          Ncv32u new_offset = node_right.getNextNodeOffset() + offset_root;
          node_right.create(new_offset);
      }
      haarClassifierNodes[i].setRightNodeDesc(node_right);
  }

  for (Ncv32u i=0; i<host_temp_classifier_not_root_nodes.size(); i++)
  {
      HaarClassifierNodeDescriptor32 node_left = host_temp_classifier_not_root_nodes[i].getLeftNodeDesc();
      if (!node_left.isLeaf())
      {
          Ncv32u new_offset = node_left.getNextNodeOffset() + offset_root;
          node_left.create(new_offset);
      }
      host_temp_classifier_not_root_nodes[i].setLeftNodeDesc(node_left);

      HaarClassifierNodeDescriptor32 node_right = host_temp_classifier_not_root_nodes[i].getRightNodeDesc();
      if (!node_right.isLeaf())
      {
          Ncv32u new_offset = node_right.getNextNodeOffset() + offset_root;
          node_right.create(new_offset);
      }
      host_temp_classifier_not_root_nodes[i].setRightNodeDesc(node_right);

      haarClassifierNodes.push_back(host_temp_classifier_not_root_nodes[i]);
  }
  return (NCV_SUCCESS);
}

/**
 * \brief This loads the Haar description file from a NVBIN file format
 */
NCVStatus
pcl::gpu::people::FaceDetector::loadFromNVBIN(const std::string &filename,
                               HaarClassifierCascadeDescriptor &haar,
                               std::vector<HaarStage64> &haar_stages,
                               std::vector<HaarClassifierNode128> &haarClassifierNodes,
                               std::vector<HaarFeature64> &haar_features)
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

    haar_stages.resize(haar.NumStages);
    haarClassifierNodes.resize(haar.NumClassifierTotalNodes);
    haar_features.resize(haar.NumFeatures);

    Ncv32u szStages = haar.NumStages * sizeof(HaarStage64);
    Ncv32u szClassifiers = haar.NumClassifierTotalNodes * sizeof(HaarClassifierNode128);
    Ncv32u szFeatures = haar.NumFeatures * sizeof(HaarFeature64);

    memcpy(&haar_stages[0], &fdata[0]+dataOffset, szStages);
    dataOffset += szStages;
    memcpy(&haarClassifierNodes[0], &fdata[0]+dataOffset, szClassifiers);
    dataOffset += szClassifiers;
    memcpy(&haar_features[0], &fdata[0]+dataOffset, szFeatures);
    dataOffset += szFeatures;

    return NCV_SUCCESS;
}

/*
 * \brief Depending on file format load the Haar description file
 */
NCVStatus
pcl::gpu::people::FaceDetector::ncvHaarLoadFromFile_host(const std::string &filename,
                                   HaarClassifierCascadeDescriptor &haar,
                                   NCVVector<HaarStage64> &h_haar_stages,
                                   NCVVector<HaarClassifierNode128> &h_haar_nodes,
                                   NCVVector<HaarFeature64> &h_haar_features)
{
    PCL_ASSERT_ERROR_PRINT_RETURN(h_haar_stages.memType() == NCVMemoryTypeHostPinned &&
                                  h_haar_nodes.memType() == NCVMemoryTypeHostPinned &&
                                  h_haar_features.memType() == NCVMemoryTypeHostPinned, "return NCV_MEM_RESIDENCE_ERROR", NCV_MEM_RESIDENCE_ERROR);

    NCVStatus ncv_return_status;

    std::string fext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(fext.begin(), fext.end(), fext.begin(), ::tolower);

    std::vector<HaarStage64> haar_stages;
    std::vector<HaarClassifierNode128> haar_nodes;
    std::vector<HaarFeature64> haar_features;

    if (fext == "nvbin")
    {
        ncv_return_status = loadFromNVBIN(filename, haar, haar_stages, haar_nodes, haar_features);
        PCL_ASSERT_NCVSTAT(ncv_return_status);
    }
    else if (fext == "xml")
    {
        ncv_return_status = loadFromXML2(filename, haar, haar_stages, haar_nodes, haar_features);
        PCL_ASSERT_NCVSTAT(ncv_return_status);
    }
    else
    {
        return NCV_HAAR_XML_LOADING_EXCEPTION;
    }

    PCL_ASSERT_ERROR_PRINT_RETURN(h_haar_stages.length() >= haar_stages.size(), "Return NCV_MEM_INSUFFICIENT_CAPACITY", NCV_MEM_INSUFFICIENT_CAPACITY);
    PCL_ASSERT_ERROR_PRINT_RETURN(h_haar_nodes.length() >= haar_nodes.size(), "Return NCV_MEM_INSUFFICIENT_CAPACITY", NCV_MEM_INSUFFICIENT_CAPACITY);
    PCL_ASSERT_ERROR_PRINT_RETURN(h_haar_features.length() >= haar_features.size(), "Return NCV_MEM_INSUFFICIENT_CAPACITY", NCV_MEM_INSUFFICIENT_CAPACITY);

    memcpy(h_haar_stages.ptr(), &haar_stages[0], haar_stages.size()*sizeof(HaarStage64));
    memcpy(h_haar_nodes.ptr(), &haar_nodes[0], haar_nodes.size()*sizeof(HaarClassifierNode128));
    memcpy(h_haar_features.ptr(), &haar_features[0], haar_features.size()*sizeof(HaarFeature64));

    return NCV_SUCCESS;
}

/*
 * \brief Scans the Haar description file for the sizes of the Stages, Nodes and Features
 */
NCVStatus
pcl::gpu::people::FaceDetector::ncvHaarGetClassifierSize(const std::string &filename,
                                                         Ncv32u &numStages,
                                                         Ncv32u &numNodes,
                                                         Ncv32u &numFeatures)
{
    size_t readCount;
    NCVStatus ncv_return_status;

    std::string fext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(fext.begin(), fext.end(), fext.begin(), ::tolower);

    if (fext == "nvbin")
    {
        FILE *fp = fopen(filename.c_str(), "rb");
        PCL_ASSERT_ERROR_PRINT_RETURN(fp != NULL, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        Ncv32u fileVersion;
        readCount = fread(&fileVersion, sizeof(Ncv32u), 1, fp);
        PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        PCL_ASSERT_ERROR_PRINT_RETURN(fileVersion == NVBIN_HAAR_VERSION, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        fseek(fp, NVBIN_HAAR_SIZERESERVED, SEEK_SET);
        Ncv32u tmp;
        readCount = fread(&numStages,   sizeof(Ncv32u), 1, fp);
        PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        readCount = fread(&tmp,         sizeof(Ncv32u), 1, fp);
        PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        readCount = fread(&numNodes,    sizeof(Ncv32u), 1, fp);
        PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        readCount = fread(&numFeatures, sizeof(Ncv32u), 1, fp);
        PCL_ASSERT_ERROR_PRINT_RETURN(1 == readCount, "Return NCV_FILE_ERROR", NCV_FILE_ERROR);
        fclose(fp);
    }
    else if (fext == "xml")
    {
        HaarClassifierCascadeDescriptor haar;
        std::vector<HaarStage64> haar_stages;
        std::vector<HaarClassifierNode128> haar_nodes;
        std::vector<HaarFeature64> haar_features;

        ncv_return_status = loadFromXML2(filename, haar, haar_stages, haar_nodes, haar_features);
        PCL_ASSERT_NCVSTAT(ncv_return_status);    // TODO convert this to PCL methodS

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

/*
 * \brief the Wrapper that calls the actual Nvidia code
 */
NCVStatus
pcl::gpu::people::FaceDetector::NCVprocess(pcl::PointCloud<pcl::RGB>&           cloud_in,
                                           pcl::PointCloud<pcl::Intensity32u>&  cloud_out,
                                           HaarClassifierCascadeDescriptor      &haar,
                                           NCVVector<HaarStage64>               &d_haar_stages,
                                           NCVVector<HaarClassifierNode128>     &d_haar_nodes,
                                           NCVVector<HaarFeature64>             &d_haar_features,
                                           NCVVector<HaarStage64>               &h_haar_stages,
                                           INCVMemAllocator                     &gpu_allocator,
                                           INCVMemAllocator                     &cpu_allocator,
                                           cudaDeviceProp                       &device_properties,
                                           Ncv32u                               width,
                                           Ncv32u                               height,
                                           NcvBool                              bFilterRects,
                                           NcvBool                              bLargestFace)
{
  pcl::PointCloud<Intensity32u>  input_gray;

  PointCloudRGBtoI(cloud_in, input_gray);

  cloud_out.width = input_gray.width;
  cloud_out.height = input_gray.height;
  cloud_out.points.resize (input_gray.points.size ());

  PCL_ASSERT_ERROR_PRINT_RETURN(!gpu_allocator.isCounting(),"retcode=NCV_NULL_PTR", NCV_NULL_PTR);

  NCVStatus ncv_return_status;

  NCV_SET_SKIP_COND(gpu_allocator.isCounting());

  NCVMatrixAlloc<Ncv8u> d_src(gpu_allocator, width, height);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_src.isMemAllocated(), "retcode=NCV_ALLOCATOR_BAD_ALLOC", NCV_ALLOCATOR_BAD_ALLOC);
  NCVMatrixAlloc<Ncv8u> h_src(cpu_allocator, width, height);
  PCL_ASSERT_ERROR_PRINT_RETURN(h_src.isMemAllocated(), "retcode=NCV_ALLOCATOR_BAD_ALLOC", NCV_ALLOCATOR_BAD_ALLOC);
  NCVVectorAlloc<NcvRect32u> d_rects(gpu_allocator, 100);
  PCL_ASSERT_ERROR_PRINT_RETURN(d_rects.isMemAllocated(), "retcode=NCV_ALLOCATOR_BAD_ALLOC", NCV_ALLOCATOR_BAD_ALLOC);

  NCV_SKIP_COND_BEGIN

  for(int i=0; i<input_gray.points.size(); i++)
  {
    memcpy(h_src.ptr(), &input_gray.points[i].intensity, sizeof(input_gray.points[i].intensity));
  }

  ncv_return_status = h_src.copySolid(d_src, 0);
  PCL_ASSERT_NCVSTAT(ncv_return_status);
  PCL_ASSERT_CUDA_RETURN(cudaStreamSynchronize(0), NCV_CUDA_ERROR);

  NCV_SKIP_COND_END

  NcvSize32u roi;
  roi.width = d_src.width();
  roi.height = d_src.height();

  Ncv32u number_of_detections;
  ncv_return_status = ncvDetectObjectsMultiScale_device ( d_src,
                                                roi,
                                                d_rects,
                                                number_of_detections,
                                                haar,
                                                h_haar_stages,
                                                d_haar_stages,
                                                d_haar_nodes,
                                                d_haar_features,
                                                haar.ClassifierSize,
                                                (bFilterRects || bLargestFace) ? 4 : 0,
                                                1.2f,
                                                1,
                                                (bLargestFace ? NCVPipeObjDet_FindLargestObject : 0) | NCVPipeObjDet_VisualizeInPlace,
                                                gpu_allocator,
                                                cpu_allocator,
                                                device_properties,
                                                0);

  PCL_ASSERT_NCVSTAT(ncv_return_status);
  PCL_ASSERT_CUDA_RETURN(cudaStreamSynchronize(0), NCV_CUDA_ERROR);

  NCV_SKIP_COND_BEGIN

  ncv_return_status = d_src.copySolid(h_src, 0);
  PCL_ASSERT_NCVSTAT(ncv_return_status);
  PCL_ASSERT_CUDA_RETURN(cudaStreamSynchronize(0), NCV_CUDA_ERROR);

  // Copy result back into output cloud
  for(int i=0; i<cloud_out.points.size(); i++)
  {
    memcpy(&cloud_out.points[i].intensity, h_src.ptr() /* + i * ??? */, sizeof(cloud_out.points[i].intensity));
  }

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

  // Load the classifier from file (assuming its size is about 1 mb), using a simple allocator
  gpu_allocator_ = new NCVMemNativeAllocator(NCVMemoryTypeDevice, static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(gpu_allocator_->isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating cascade GPU allocator", -1);

  cpu_allocator_ = new NCVMemNativeAllocator(NCVMemoryTypeHostPinned, static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(cpu_allocator_->isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating cascade CPU allocator", -1);

  NCVStatus ncv_return_status;
  Ncv32u haarNumStages, haarNumNodes, haarNumFeatures;

  ncv_return_status = ncvHaarGetClassifierSize(cascade_file_name_, haarNumStages, haarNumNodes, haarNumFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncv_return_status == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error reading classifier size (check the file)", -1);

  haar_stages_host_ = new NCVVectorAlloc<HaarStage64>(*cpu_allocator_, haarNumStages);
  PCL_ASSERT_ERROR_PRINT_RETURN(haar_stages_host_->isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade CPU allocator", -1);

  haar_nodes_host_ = new NCVVectorAlloc<HaarClassifierNode128>(*cpu_allocator_, haarNumNodes);
  PCL_ASSERT_ERROR_PRINT_RETURN(haar_nodes_host_->isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade CPU allocator", -1);

  haar_features_host_ = new NCVVectorAlloc<HaarFeature64>(*cpu_allocator_, haarNumFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(haar_features_host_->isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade CPU allocator", -1);

  ncv_return_status = ncvHaarLoadFromFile_host(cascade_file_name_, haar_clas_casc_descr_, *haar_stages_host_, *haar_nodes_host_, *haar_features_host_);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncv_return_status == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error loading classifier", -1);

  haar_stages_dev_ = new NCVVectorAlloc<HaarStage64>(*gpu_allocator_, haarNumStages);
  PCL_ASSERT_ERROR_PRINT_RETURN(haar_stages_dev_->isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade GPU allocator", -1);

  haar_nodes_dev_ = new NCVVectorAlloc<HaarClassifierNode128>(*gpu_allocator_, haarNumNodes);
  PCL_ASSERT_ERROR_PRINT_RETURN(haar_nodes_dev_->isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade GPU allocator", -1);

  haar_features_dev_ = new NCVVectorAlloc<HaarFeature64>(*gpu_allocator_, haarNumFeatures);
  PCL_ASSERT_ERROR_PRINT_RETURN(haar_features_dev_->isMemAllocated(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in cascade GPU allocator", -1);

  ncv_return_status = haar_stages_host_->copySolid(*haar_stages_dev_, 0);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncv_return_status == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error copying cascade to GPU", -1);
  ncv_return_status = haar_nodes_host_->copySolid(*haar_nodes_dev_, 0);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncv_return_status == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error copying cascade to GPU", -1);
  ncv_return_status = haar_features_host_->copySolid(*haar_features_dev_, 0);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncv_return_status == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error copying cascade to GPU", -1);

  // Calculate memory requirements and create real allocators
  gpu_counter_ = new NCVMemStackAllocator(static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(gpu_counter_->isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating GPU memory counter", -1);

  cpu_counter_ = new NCVMemStackAllocator(static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(cpu_counter_->isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating CPU memory counter", -1);

  // TODO fix this fake call
  /*
  ncv_return_status = NCVprocess (NULL, cols_, rows_,
                        false, false, haar,
                        d_haar_stages, d_haar_nodes,
                        d_haar_features, h_haar_stages,
                        gpuCounter, cpuCounter, cuda_dev_prop_);
  PCL_ASSERT_ERROR_PRINT_RETURN(ncv_return_status == NCV_SUCCESS, "[pcl::gpu::people::FaceDetector::FaceDetector] : Error in memory counting pass", -1);
  */

  gpu_stack_allocator_ = new NCVMemStackAllocator(NCVMemoryTypeDevice, gpu_counter_->maxSize(), static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(gpu_stack_allocator_->isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating GPU memory allocator", -1);

  cpu_stack_allocator_ = new NCVMemStackAllocator(NCVMemoryTypeHostPinned, cpu_counter_->maxSize(), static_cast<Ncv32u>(cuda_dev_prop_.textureAlignment));
  PCL_ASSERT_ERROR_PRINT_RETURN(cpu_stack_allocator_->isInitialized(), "[pcl::gpu::people::FaceDetector::FaceDetector] : Error creating CPU memory allocator", -1);

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
pcl::gpu::people::FaceDetector::process(pcl::PointCloud<pcl::RGB>& cloud_in,
                                        pcl::PointCloud<pcl::Intensity32u>& cloud_out)
{
  PCL_DEBUG("[pcl::gpu::people::FaceDetector::process] : (D) : called\n");
  cols_ = cloud_in.width; rows_ = cloud_in.height;

  // TODO do something with the NCVStatus return value
  NCVStatus status = NCVprocess(cloud_in,
                                cloud_out,
                                haar_clas_casc_descr_,
                                *haar_stages_dev_,
                                *haar_nodes_dev_,
                                *haar_features_dev_,
                                *haar_stages_host_,
                                *gpu_allocator_,
                                *cpu_allocator_,
                                cuda_dev_prop_,
                                cloud_in.width,
                                cloud_in.height,
                                filter_rects_,
                                largest_object_);

}


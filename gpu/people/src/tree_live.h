/**
 * @authors: Cedric Cagniart, Koen Buys
 **/
#ifndef PCL_GPU_PEOPLE_TREELIVE_H_
#define PCL_GPU_PEOPLE_TREELIVE_H_

#include <iostream>
#include <boost/noncopyable.hpp>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/containers/device_array.h>
#include <vector>

#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        //forward declared but never shown
        struct CUDATree
        {                                                       
            int treeHeight;
            int numNodes;

            DeviceArray<Node> nodes_device;
            DeviceArray<Label> leaves_device;                      

            CUDATree (int treeHeight_, const std::vector<Node>& nodes, const std::vector<Label>& leaves)                
            {
              treeHeight = treeHeight_;
              numNodes = (1 << treeHeight) - 1;
              assert( nodes.size()  == (size_t)numNodes );
              assert( leaves.size() == (size_t)(1 << treeHeight) );
              
              nodes_device.upload(nodes);
              leaves_device.upload(leaves);          
            }
            ~CUDATree() {}
        };
       

        // utility function to get a colored map out
        int colorLMap( const cv::Mat& lmap, cv::Mat& cmap );
        cv::Scalar getLColor( const uint8_t l);

        /** Processor using a unique tree */
        class TreeLiveProc : public boost::noncopyable
        {
          public :
          /** The stream points to ascii data that goes:
           * ##################
           * TreeHeight
           * du1 dv1 du2 dv2 thresh
           * du1 dv1 du2 dv2 thresh
           * ............
           * label
           * label
           * ##################
           *
           * there are 2^threeheight -1 nodes ( [du1 dv1 du2 dv2 thresh] lines )
           * there are 2^threeheigh   leaves ( [label] lines )
           */
            TreeLiveProc(std::istream& is);
            ~TreeLiveProc();

            void process(const cv::Mat& dmap, cv::Mat& lmap);

          private :
            boost::shared_ptr<CUDATree> m_tree;
            DeviceArray<uint16_t> m_dmap_device;
            DeviceArray<uint8_t> m_lmap_device;
        };

        /** Processor using multiple trees */
        class PCL_EXPORTS MultiTreeLiveProc : public boost::noncopyable 
        {
        public:
            typedef boost::shared_ptr<MultiTreeLiveProc> Ptr;

            // this is a bit crappy... the constructor will take the first tree, 
            // garanteeing that there is something in there. other trees need to be 
            // added with addTree();
            MultiTreeLiveProc(std::istream& is);
            ~MultiTreeLiveProc();

            void addTree(std::istream& is);
            
            // same as process, but runs the trick of declaring as background any
            // neighbor that is more than FGThresh away.
            void process(const DeviceArray2D<unsigned short>& dmap, DeviceArray2D<unsigned char>& lmap, 
                int FGThresh = std::numeric_limits<Attrib>::max());

          private:
            std::vector<CUDATree> m_trees;                        
            DeviceArray<uint8_t> m_multilmap_device;

        };
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

#endif

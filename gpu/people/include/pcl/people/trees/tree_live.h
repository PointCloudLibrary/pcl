/**
 * @authors: Cedric Cagniart, Koen Buys
 **/
#ifndef PCL_PEOPLE_TREES_TREELIVE_H_
#define PCL_PEOPLE_TREES_TREELIVE_H_

#include <iostream>
#include <boost/noncopyable.hpp>
#include <opencv2/opencv.hpp>

namespace pcl
{
  namespace people
  {
    namespace trees
    {
      //forward declared but never shown
      struct CUDATree;

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

          void process(const cv::Mat& dmap,
                       cv::Mat&       lmap );

        private :
          CUDATree* m_tree;
          void*     m_dmap_device;
          void*     m_lmap_device;
      };

      /** Processor using multiple trees */
      class MultiTreeLiveProc : public boost::noncopyable 
      {
        public :
          // this is a bit crappy... the constructor will take the first tree, 
          // garanteeing that there is something in there. other trees need to be 
          // added with addTree();
          MultiTreeLiveProc(std::istream& is);
          ~MultiTreeLiveProc();

          void addTree(std::istream& is);

          // dmap has to be 640*480 16bit grayscale
          // lmap will be 640*480 8bit grayscale
          void process(const cv::Mat& dmap,
                       cv::Mat&       lmap );

          // same as process, but runs the trick of declaring as background any
          // neighbor that is more than FGThresh away.
          void process(const cv::Mat& dmap,
                       cv::Mat&       lmap,
                       int            FGThresh);

        private :
          std::vector<CUDATree*> m_trees;
          void*                  m_dmap_device;
          void*                  m_multilmap_device;
          void*                  m_lmap_device;
      };
    } // end namespace trees
  } // end namespace people
} // end namespace pcl

#endif

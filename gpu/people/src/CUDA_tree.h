/**
 * @authors: Cedric Cagniart, Koen Buys
 **/
#ifndef PCL_PEOPLE_TREES_CUDATREE_H_
#define PCL_PEOPLE_TREES_CUDATREE_H_

#include <vector>
#include <pcl/people/trees/tree.h>
#include <boost/noncopyable.hpp>

namespace pcl
{
  namespace people
  {
    namespace trees
    {
      class CUDATree : public boost::noncopyable 
      {
        public :
          CUDATree (int                             treeHeight, 
                    const std::vector<pcl::people::trees::Node>&  nodes,
                    const std::vector<pcl::people::trees::Label>& leaves );
          ~CUDATree ();

          inline int         treeHeight ()    const { return m_treeHeight; }
          inline int         numNodes ()      const { return m_numNodes; }
          inline const void* nodes_device ()  const { return m_nodes_device; }
          inline const void* leaves_device () const { return m_leaves_device; }

        protected :
          int   m_treeHeight;
          int   m_numNodes;
          void* m_nodes_device;
          void* m_leaves_device;
      };
    } // end namespace trees
  } // end namespace people
} // end namespace pcl
#endif //PCL_PEOPLE_TREES_CUDATREE_H_

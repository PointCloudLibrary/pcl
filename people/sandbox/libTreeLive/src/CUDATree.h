#ifndef CUDATREE_H_DEFINED
#define CUDATREE_H_DEFINED

#include <vector>
#include "commonTrees/Tree.h"
#include <boost/noncopyable.hpp>

namespace TreeLive
{


class CUDATree : public boost::noncopyable 
{
	public :
	CUDATree(int                             treeHeight, 
	         const std::vector<Tree::Node>&  nodes,
	         const std::vector<Tree::Label>& leaves );
	~CUDATree();

	inline int         treeHeight()    const { return m_treeHeight; }
	inline int         numNodes()      const { return m_numNodes; }
	inline const void* nodes_device()  const { return m_nodes_device; }
	inline const void* leaves_device() const { return m_leaves_device; }

	protected :
	int   m_treeHeight;
	int   m_numNodes;
	void* m_nodes_device;
	void* m_leaves_device;
};



} // end namespace Tree



#endif

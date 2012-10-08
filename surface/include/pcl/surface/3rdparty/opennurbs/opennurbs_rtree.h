/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

#if !defined(OPENNURBS_RTREE_INC_)
#define OPENNURBS_RTREE_INC_

/*
The opennurbs rtree code is a modifed version of the
free and unrestricted R-tree implementation obtianed from 
http://www.superliminal.com/sources/sources.htm

The first lines on the website indicate the code is free and unrestricted:

  Free Source Code
  Here are a few useful bits of free source code. 
  You're completely free to use them for any purpose whatsoever.
  All I ask is that if you find one to be particularly valuable, 
  then consider sending feedback. Please send bugs and suggestions too. 
  Enjoy 

The readme.txt file included with the R-tree source says

  LICENSE:
    Entirely free for all uses. Enjoy!

The original authors are 

AUTHORS
 * 1983 Original algorithm and test code by Antonin Guttman and Michael Stonebraker, UC Berkely
 * 1994 ANCI C ported from original test code by Melinda Green - melinda@superliminal.com
 * 1995 Sphere volume fix for degeneracy problem submitted by Paul Brook
 * 2004 Templated C++ port by Greg Douglas

The opennurbs version adds some custom memory allocation and replaces
the leaf iterator.  The rest of the changes are cosmetic.

*/



// Minimum and maximum number of elements 
// in ON_RTreeNode::m_branch[].
// must have ON_RTree_MAX_NODE_COUNT > ON_RTree_MIN_NODE_COUNT
#define ON_RTree_MIN_NODE_COUNT 2
#define ON_RTree_MAX_NODE_COUNT 6

/*
In a test of a sphere mesh with mesh: 8385 vertices, 8192 polygons
and ON_RTree_MAX_NODE_COUNT = 3, 4, 5, and 6, the memory use was 
most efficient with ON_RTree_MAX_NODE_COUNT=6

Memory Usage MAX_NODE_COUNT = 3
  ON_RTree: 1212 KB (1241136) (352 wasted)
  ON_RTree: 7036 nodes, 5881 unused branches (321 KB) 0.835844 per node

Memory Usage MAX_NODE_COUNT = 4
  ON_RTree: 1152 KB (1179720) (5568 wasted)
  ON_RTree: 5051 nodes, 6962 unused branches (380 KB) 1.37834 per node

Memory Usage MAX_NODE_COUNT = 5
  ON_RTree: 1040 KB (1065504) (11808 wasted)
  ON_RTree: 3655 nodes, 6429 unused branches (351 KB) 1.75896 per node

Memory Usage MAX_NODE_COUNT = 6
  ON_RTree:  995 KB (1019592) (3440 wasted)
  ON_RTree: 2951 nodes, 6564 unused branches (358 KB) 2.22433 per node
*/

// This struct is used instead of ON_BoundingBox to avoid calling
// constructors.
struct ON_RTreeBBox
{
  double m_min[3];
  double m_max[3];
};

struct ON_RTreeSphere
{
  double m_point[3];
  double m_radius;
};

struct ON_RTreeCapsule
{
  double m_point[2][3];
  double m_radius;
  double m_domain[2];
};

struct ON_RTreeBranch
{
  ON_RTreeBBox m_rect;

  // If ON_RTreeNode.m_level > 0, then m_child points to a child node.
  // If ON_RTreeNode.m_level == 0, then m_id identifies the leaf element.
  union
  {
    struct ON_RTreeNode* m_child;
    ON__INT_PTR m_id;
  };
};

struct ON_RTreeLeaf
{
  ON_RTreeBBox m_rect;
  ON__INT_PTR m_id;
};

// The ON_RTreeNode is used at root, branch and leaf nodes.
// When m_level > 0, the node is a branch.
// When m_level = 0, the node is a leaf.
struct ON_RTreeNode
{
  inline bool IsInternalNode() const
    { return (m_level > 0); }  // internal nodes have m_level > 0
  inline bool IsLeaf() const
    { return (m_level == 0); } // branch nodes have m_level = 0

  // m_level must be a signed int to insure signed compares work correctly
  int m_level;  // =0 at leaf nodes, > 0 at branch nodes

  // The m_branch[] array contains m_count elements
  // 0 <= m_count <= ON_RTree_MAX_NODE_COUNT
  // m_count must be a signed int to insure signed compares work correctly
  int m_count; 
  ON_RTreeBranch m_branch[ON_RTree_MAX_NODE_COUNT];
};

struct ON_RTreeSearchResult
{
  int m_capacity;   // m_id[] array capacity (search terminates when m_count == m_capacity)
  int m_count;      // number of elements in m_id[]
  ON__INT_PTR* m_id; // m_id[] = array of search results.
};

class ON_CLASS ON_RTreeMemPool
{
public:
  ON_RTreeMemPool( ON_MEMORY_POOL* heap, size_t leaf_count );
  ~ON_RTreeMemPool();

  ON_RTreeNode* AllocNode();
  void FreeNode(ON_RTreeNode* node);

  struct ON_RTreeListNode* AllocListNode();
  void FreeListNode(struct ON_RTreeListNode* list_node);

  void DeallocateAll();

  /*
  Returns:
    Total number of bytes of heap memory allocated.
  */
  size_t SizeOf() const;

  /*
  Returns:
    Number of bytes of heap memory not currently in use.
  */
  size_t SizeOfUnusedBuffer() const;

private:
  void GrowBuffer();

  struct Blk
  {
    struct Blk* m_next;
  };

  // linked list of unused ON_RTreeNode 
  struct Blk* m_nodes;
  // linked list of unused ON_RTreeListNode
  struct Blk* m_list_nodes;

  // buffer for new allocations
  unsigned char* m_buffer;
  size_t m_buffer_capacity;

  struct Blk* m_blk_list;   // linked list used to free all allocated memory
  size_t m_sizeof_blk;      // total amount of memory in each block.

  ON_MEMORY_POOL* m_heap;
  size_t m_sizeof_heap; // total amount of heap memory in this rtree
};

////////////////////////////////////////////////////////////////
//
// ON_RTreeIterator
//
//   The ON_RTreeIterator class can be used to iterate each leaf 
//   in an ON_RTree.
//
class ON_CLASS ON_RTreeIterator
{
public:
  /*
  Description:
    Construct an empty iterator.  Call Initialize() to attach
    the iterator to an R-tree.
  Remark:
    Any calls to ON_RTree::Insert() or ON_RTree::Remove() that modify
    an R-tree being iterated invalidate the iterator.  The iterator
    must be re-initialized before being used again.

    There is no connection between the order elements are inserted
    in an R-tree and the order the elements are iterated by an
    iterator.    
  */
  ON_RTreeIterator();
  ON_RTreeIterator(const class ON_RTree& a_rtree);

  ~ON_RTreeIterator();

  /*
  Description:
    Initialize an iterator to iterate every leaf in the rtree.
  Parameters:
    a_rtree - [in]
      R-tree to iterate
  Example:
    See the comment for ON_RTreeIterator::First().
  Returns:
    True if a_rtree has at least one element.
  Remarks:
    Any calls to ON_RTree::Insert() or ON_RTree::Remove() that modify
    this node or its children will invalidate this iterator and it
    must be re-initialized.

    There is no connection between the order elements are inserted
    in an R-tree and the order the elements are iterated by an
    iterator.    
  */
  bool Initialize(const class ON_RTree& a_rtree);

  /*
  Description:
    Initialize an iterator to iterate every leaf on or below a_node.
  Parameters:
    a_node - [in]
      R-tree node to iterate
  Example:
    See the comment for ON_RTreeIterator::First().
  Returns:
    True if a_node has at least one element.
  Remarks:
    Any calls to ON_RTree::Insert() or ON_RTree::Remove() that modify
    this node or its children will invalidate this iterator and it
    must be re-initialized.

    There is no connection between the order elements are inserted
    in an R-tree and the order the elements are iterated by an
    iterator.    
  */
  bool Initialize(const struct ON_RTreeNode* a_node);

  /*
  Description:
    Get the value of the current leaf element. Calling Value()
    does not increment or decrement the iterator.
  Example:
    See the comment for ON_RTreeIterator::First().
  Return:
    Null pointer if there are no more leaves to iterate
    A pointer to the current R-tree leaf.  When there are no more leaves,
    the returned pointer is null.
  */
  const ON_RTreeBranch* Value() const;

  /*
  Description:
    Reset the iterator so the current leaf is the first leaf in
    the R-tree.  The Initialize() functions automatically do
    this, but First() can be called if an iterator needs to be
    used more than once or to make code easy to read and understand.
  Example:
    Iterate every leaf in an R-tree.

          ON_RTree rtree;
          ...
          ON_RtreeIterator rit(rtree);
          const ON_RTreeBranch* rtree_leaf;
          for ( rit.First(); 0 != (rtree_leaf = rit.Value()); rit.Next() )
          {
            // leaf id           = rtree_leaf->m_id
            // leaf bounding box = rtree->m_rect
          }

  Returns:
    True if a call to Value() will return a non-null pointer.
  See Also:
    ON_RTreeIterator::Last();
  */
  bool First();

  /*
  Description:
    Increment the iterator to the next leaf in the R-tree.
  Example:
    See the comment for ON_RTreeIterator::First()
  Returns:
    True if a call to Value() will return a non-null pointer.
    False if there is not a next leaf and all susequent calls to
    Value() will return null.
  See Also:
    ON_RTreeIterator::Prev();
  */
  bool Next();


  /*
  Description:
    Set the iterator so the current leaf is the last leaf in the R-tree.

  Example:
    Iterate an R-tree in reverse order.

          ON_RTree rtree;
          ...
          ON_RTreeIterator rit(rtree);
          const ON_RTreeBranch* rtree_leaf;
          for ( rit.Last(); 0 != (rtree_leaf = rit.Value()); rit.Prev() )
          {
            // leaf id           = rtree_leaf->m_id
            // leaf bounding box = rtree->m_rect
          }

  Returns:
    True if a call to Value() will return a non-null pointer.
  See Also:
    ON_RTreeIterator::First();
  */
  bool Last();

  /*
  Description:
    Decrement the iterator to the previous leaf in the R-tree.
  Example:
    See the comment for ON_RTreeIterator::Last()
  Returns:
    True if a call to Value() will return a non-null pointer.
    False if there is not a previous leaf and all susequent calls to
    Value() will return null.
  See Also:
    ON_RTreeIterator::Next();
  */
  bool Prev();

private:
  enum { MAX_STACK = 32 }; //  Max stack size. Allows almost n^32 where n is number of branches in node
  
  struct StackElement
  {
    const struct ON_RTreeNode* m_node;
    int m_branchIndex; // must be a signed int to insure signed compares work correctly
  };

  bool PushChildren(struct StackElement* sp, bool bFirstChild);

  StackElement  m_stack[MAX_STACK]; // stack
  StackElement* m_sp;               // stack pointer (null or points into m_stack[])
  const ON_RTreeNode* m_root;       // root of tree being iterated
};


class ON_CLASS ON_RTree
{
public:
  ON_RTree( ON_MEMORY_POOL* heap = 0, size_t leaf_count = 0 );
  ~ON_RTree();

  /*
  Description:
    Create an R-tree with an element for each face in the mesh.
    The element id is set to the index of the face.
  Parameters:
    mesh - [in]
  Returns:
    True if successful.
  */
  bool CreateMeshFaceTree( const class ON_Mesh* mesh );
  
  /*
  Description:
    Insert an element into the RTree.
  Parameters:
    a_min - [in]
    a_max - [in]
      3d bounding box of the element.  The values in a_min[3] and a_max[3]
      must satisfy
      a_min[0] <= a_max[0], 
      a_min[1] <= a_max[1], and
      a_min[1] <= a_max[1].
    a_dataId - [in]
      id of the element.  This can be either a pointer or an integer id.
  Returns:
    True if element was successfully inserted.
  Remarks:
    Calling Insert() or Remove() invalidates any ON_RTreeIterator
    used to iterate this rtree. 
  */
  bool Insert(const double a_min[3], const double a_max[3], void* a_element_id);
  bool Insert(const double a_min[3], const double a_max[3], int a_element_id);
  bool Insert2d(const double a_min[2], const double a_max[2], void* a_element_id);
  bool Insert2d(const double a_min[2], const double a_max[2], int a_element_id);
  
  /*
  Description:
    Remove an element from the RTree.
  Parameters:
    a_min - [in]
    a_max - [in]
      3d bounding box of the element.  The values in a_min[3] and a_max[3]
      must satisfy
      a_min[0] <= a_max[0], 
      a_min[1] <= a_max[1], and
      a_min[2] <= a_max[2].
    a_dataId - [in]
      id of the element.  This can be either a pointer or an integer id.
  Returns:
    True if element was successfully removed.
  Remarks:
    Calling Insert() or Remove() invalidates any ON_RTreeIterator
    used to iterate this rtree. 
  */
  bool Remove(const double a_min[3], const double a_max[3], void* a_elementId);
  bool Remove(const double a_min[3], const double a_max[3], int a_elementId);
  bool Remove2d(const double a_min[2], const double a_max[2], void* a_elementId);
  bool Remove2d(const double a_min[2], const double a_max[2], int a_elementId);
  
  /*
  Description:
    Remove all elements from the R-tree.
  */
  void RemoveAll();

  /*
  Description:
    Search the R-tree for all elements whose bounding boxes overlap
    a_rect.
  Parameters:
    a_rect - [in/out]
      The version of search that has ON_RTreeBBox* a_rect as the first
      argument, allows you to shrink the a_rect as the search progresses.
      This is useful for doing things like searching for closest points.
      If you want to shrink a_rect, you must use a_context to pass it
      to the resultCallback function and shrink it in the resultCallback
      function. In the callback, the modified rect must be contained
      in the previous rect.
    a_sphere - [in/out]
      The version of search that has ON_RTreeSphere* a_sphere as the first
      argument, allows you to shrink the a_sphere as the search progresses.
      This is useful for doing things like searching for closest points.
      If you want to shrink a_sphere, you must use a_context to pass it
      to the resultCallback function and shrink it in the resultCallback
      function. In the callback, the modified sphere must be contained
      in the previous sphere.
    a_capsule - [in/out]
      The version of search that has ON_RTreeSphere* a_capsule as the first
      argument, allows you to shrink the a_capsule as the search progresses.
      This is useful for doing things like searching for closest points.
      If you want to shrink a_capsule, you must use a_context to pass it
      to the resultCallback function and shrink it in the resultCallback
      function. In the callback, the modified capsule must be contained
      in the previous capsule.
    a_min - [in]
    a_max - [in]
      (a_min,a_max) is the bounding box of the search region.
    a_results - [out]
      The ids of elements that overlaps the search region.
    resultCallback - [in]
      A function to call when leaf nodes overlap.
    a_context - [in]
      pointer passed to the resultCallback() function.
  Returns:
    True if entire tree was searched.  It is possible no results were found.
  Remarks:
    If you are using a Search() that uses a resultCallback() function,
    then return true to keep searching and false to terminate the search.
  */
  bool Search( 
    ON_RTreeSphere* a_sphere,
    bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_id), 
    void* a_context
    ) const;

  bool Search( 
    ON_RTreeCapsule* a_capsule,
    bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_id), 
    void* a_context
    ) const;

  bool Search( 
    ON_RTreeBBox* a_rect,
    bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_id), 
    void* a_context
    ) const;

  /*
  Description:
    Search the R-tree for all elements whose bounding boxes overlap
    the set of points between to parallel planes.
  Parameters:
    a_plane_eqn - [in]
    a_min - [in]
    a_max - [in]
      The region between the parallel planes is the set point points
      where the value of the plane equation is >= a_min and <= a_max.
    resultCallback - [in]
      A function to call when leaf nodes overlap the region between
      the parallel planes.
    a_context - [in]
      pointer passed to the resultCallback() function.
  Returns:
    True if entire tree was searched.  It is possible no results were found.
  Remarks:
    If you are using a Search() that uses a resultCallback() function,
    then return true to keep searching and false to terminate the search.
  */
  bool Search(
    const double a_plane_eqn[4],
    double a_min,
    double a_max,
    bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_id), 
    void* a_context
    ) const;

  bool Search(const double a_min[3], const double a_max[3],
    bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_id), void* a_context 
    ) const;

	bool Search(const double a_min[3], const double a_max[3],
    ON_RTreeSearchResult& a_result 
    ) const;

	bool Search(const double a_min[3], const double a_max[3],
    ON_SimpleArray<ON_RTreeLeaf>& a_result 
    ) const;

  bool Search(const double a_min[3], const double a_max[3],
    ON_SimpleArray<void*>& a_result 
    ) const;

  bool Search(const double a_min[3], const double a_max[3],
    ON_SimpleArray<int>& a_result 
    ) const;

  bool Search2d(const double a_min[2], const double a_max[2],
    bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_id), void* a_context
    ) const;

	bool Search2d(const double a_min[2], const double a_max[2],
    ON_RTreeSearchResult& a_result
    ) const;

	bool Search2d(const double a_min[2], const double a_max[2],
    ON_SimpleArray<ON_RTreeLeaf>& a_result
    ) const;

  bool Search2d(const double a_min[2], const double a_max[2],
    ON_SimpleArray<void*>& a_result
    ) const;

  bool Search2d(const double a_min[2], const double a_max[2],
    ON_SimpleArray<int>& a_result
    ) const;

  /*
  Description:
    Search two R-trees for all pairs elements whose bounding boxes overlap.
  Parameters:
    a_rtreeA - [in]
    a_rtreeB - [in]
    tolerance - [in]
      If the distance between a pair of bounding boxes is <= tolerance, 
      then the pair is added to a_result[].
    a_result - [out]
      Pairs of ids of elements who bounding boxes overlap.
  Returns:
    True if entire tree was searched.  It is possible no results were found.
  */
  static bool Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
          ON_SimpleArray<ON_2dex>& a_result
          );

  /*
  Description:
    Search two R-trees for all pairs elements whose bounding boxes overlap.
  Parameters:
    a_rtreeA - [in]
    a_rtreeB - [in]
    tolerance - [in]
      If the distance between a pair of bounding boxes is <= tolerance, 
      then resultCallback() is called.
    resultCallback - [out]
      callback function
    a_context - [in] argument passed through to resultCallback().
  Returns:
    True if entire tree was searched.  It is possible no results were found.
  */
  static bool Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
          void ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB),
          void* a_context
          );

  /*
  Description:
    Search two R-trees for all pairs elements whose bounding boxes overlap.
  Parameters:
    a_rtreeA - [in]
    a_rtreeB - [in]
    tolerance - [in]
      If the distance between a pair of bounding boxes is <= tolerance, 
      then resultCallback() is called.
    resultCallback - [out]
      callback function
      Return true for the search to continue and false to terminate the search.
    a_context - [in] argument passed through to resultCallback().
  Returns:
    True if entire tree was searched.  It is possible no results were found.
  */
  static bool Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
          bool ON_MSC_CDECL resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB),
          void* a_context
          );
  /*
  Returns:
    Number of elements (leaves).
  Remark:
    No internal count is maintained, so this function traverses the 
    tree to count the leaves.  If efficiency is important, save the
    result.
  */
  int ElementCount();

  /*
  Returns:
    Pointer to the root node.
  */
  const ON_RTreeNode* Root() const;
  
  /*
  Returns:
    Bounding box of the entire R-tree;
  */
  ON_BoundingBox BoundingBox() const;

  /*
  Returns:
    Number of bytes of heap memory used by this R-tree.
  */
  size_t SizeOf() const;

private:
  void SplitNode(ON_RTreeNode*, ON_RTreeBranch*, ON_RTreeNode**);
  bool AddBranch(ON_RTreeBranch*, ON_RTreeNode*, ON_RTreeNode**);
  bool InsertRectRec(ON_RTreeBBox*, ON__INT_PTR, ON_RTreeNode*, ON_RTreeNode**, int);
  bool InsertRect(ON_RTreeBBox*, ON__INT_PTR, ON_RTreeNode**, int);
  void LoadNodes(ON_RTreeNode*, ON_RTreeNode*, struct ON_RTreePartitionVars*);
  bool RemoveRect(ON_RTreeBBox*, ON__INT_PTR, ON_RTreeNode**);
  bool RemoveRectRec(ON_RTreeBBox*, ON__INT_PTR, ON_RTreeNode*, struct ON_RTreeListNode**);
  void ReInsert(ON_RTreeNode*, struct ON_RTreeListNode**);
  void RemoveAllRec(ON_RTreeNode*);
  ON_RTreeNode* m_root;
  size_t m_reserved;
  ON_RTreeMemPool m_mem_pool;
};

#endif

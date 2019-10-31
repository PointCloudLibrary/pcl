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
#if !defined(OPENNURBS_FSP_INC_)
#define OPENNURBS_FSP_INC_

class ON_CLASS ON_FixedSizePool
{
public:
  ON_FixedSizePool();
  ~ON_FixedSizePool();
  
  /*
  Description:
    Create a fixed size memory pool.
  Parameters:
    sizeof_element - [in] 
      number of bytes in each element. This parameter must be greater than zero.
      In general, use sizeof(element type).  If you pass a "raw" number as 
      sizeof_element, then be certain that it is the right size to insure the 
      fields in your elements will be properly aligned.
    element_count_estimate - [in] (0 = good default)
      If you know how many elements you will need, pass that number here.
      It is better to slightly overestimate than to slightly underestimate.
      If you do not have a good estimate, then use zero.
    block_element_capacity - [in] (0 = good default)
      If block_element_capacity is zero, Create() will calculate a block
      size that is efficent for most applications.  If you are an expert
      user and want to specify the number of elements per block,
      then pass the number of elements per block here.  When 
      block_element_capacity > 0 and element_count_estimate > 0, the first 
      block will have a capacity of at least element_count_estimate; in this
      case do not ask for extraordinarly large amounts of contiguous heap.
  Remarks:
    You must call Create() on an unused ON_FixedSizePool or call Destroy()
    before calling create.
  Returns:
    True if successful and the pool can be used.
  */
  bool Create( 
    std::size_t sizeof_element,
    std::size_t element_count_estimate,
    std::size_t block_element_capacity
    );

  /*
  Returns:
    Size of the elements in this pool.
  */
  std::size_t SizeofElement() const;

  /*
  Returns:
    A pointer to sizeof_element bytes.  The memory is zeroed.
  */
  void* AllocateElement();
  
  /*
  Description:
    Return an element to the pool.
  Parameters:
    p - [in]
      A pointer returned by AllocateElement().
      It is critical that p be from this pool and that
      you return a pointer no more than one time.
  Remarks:
    If you find the following remarks confusing, but you really want to use
    ReturnElement(), then here are some simple guidelines.
      1) SizeofElement() must be >= 16
      2) SizeofElement() must be a multiple of 8.
      3) Do not use FirstElement() and NextElement() to iterate through
         the pool.

    If 1 to 3 don't work for you, then you need to understand the following
    information before using ReturnElement().

    ON_FixedMemoryPool uses the first sizeof(void*) bytes of the
    returned element for bookkeeping purposes.  Therefore, if you
    are going to use ReturnElement(), then SizeofElement() must be 
    at least sizeof(void*).  If you are using a platform that requires
    pointers to be aligned on sizeof(void*) boundaries, then
    SizeofElement() must be a multiple of sizeof(void*).
    If you are going to use ReturnElement() and then use FirstElement()
    and NextElement() to iterate through the list of elements, then you
    need to set a value in the returned element to indicate that it
    needs to be skipped during the iteration.  This value cannot be
    located in the fist sizeof(void*) bytes of the element.  If the 
    element is a class with a vtable, you cannot call a virtual 
    function on a returned element because the vtable pointer is 
    trashed when ReturnElement() modifies the fist sizeof(void*) bytes.
  */
  void ReturnElement(void* p);

  /*
  Description:
    Return all allocated elements to the pool. No heap is freed and
    the pool remains initialized and ready for AllocateElement()
    to be called.
  */
  void ReturnAll();

  /*
  Description:
    Destroy the pool and free all the heap. The pool cannot be used again
    until Create() is called.
  */
  void Destroy();

  /*
  Returns:
    Number of active elements. (Elements that have been returned are not active.)
  */
  std::size_t ActiveElementCount() const;

  /*
  Returns:
    Total number of elements = number of active elements + number of returned elements.
  */
  std::size_t TotalElementCount() const;

  /*
  Description:
    Get the first element when iterating through the list of elements.
  Parameters:
    element_index - [in]
      If you use the version of FirstElement() that has an 
      element_index parameter, then the iteration begins at
      that element.
  Example:
    The loop will iteratate through all the elements returned from
    AllocateElement(), including any that have be returned to the pool
    using ReturnElement().

          // iterate through all elements in the pool
          // This iteration will go through TotalElements() items.
          for ( void* p = FirstElement(); 0 != p; p = NextElement() )
          {
            // If you are not using ReturnElement(), then you may process
            // "p" immediately. If you have used ReturnElement(), then you
            // must check some value in p located after the first sizeof(void*)
            // bytes to see if p is active.
            if ( p is not active )
              continue;

            ... process p
          }

  Returns:
    The first element when iterating through the list of elements.
  Remarks:
    FirstElement() and NextElement() will return elements that have 
    been returned to the pool using ReturnElement().  If you use 
    ReturnElement(), then be sure to mark the element so it can be
    identified and skipped.

    Do not make any calls to FirstBlock() or NextBlock() when using
    FirstElement() and NextElement() to iteratate through elements.

    If you need iterate through a fixed size pool and another
    function may also be in the middle of iterating the pool
    as well, then use ON_FixedSizePoolIterator.  In particular,
    if you have multiple concurrent threads iterating the same 
    fixed size pool, then use ON_FixedSizePoolIterator.
  */
  void* FirstElement();
  void* FirstElement( std::size_t element_index );

  /*
  Description:
    Get the next element when iterating through the list of elements.
  Example:
    See the FirstElement() documentation.
  Returns:
    The next element when iterating through the list of elements.
  Remarks:
    FirstElement() and NextElement() will return elements that have 
    been returned to the pool using ReturnElement().  If you use 
    ReturnElement(), then be sure to mark the element so it can be
    identified and skipped.

    Do not make any calls to FirstBlock() or NextBlock() when using
    FirstElement() and NextElement() to iteratate through elements.

    If you need iterate through a fixed size pool and another
    function may also be in the middle of iterating the pool
    as well, then use ON_FixedSizePoolIterator.  In particular,
    if you have multiple concurrent threads iterating the same 
    fixed size pool, then use ON_FixedSizePoolIterator.
  */
  void* NextElement();

  /*
  Description:
    Get a pointer to the first element in the first block.
  Parameters:
    block_element_count - [out] (can be null)
      If not null, the number of elements allocated from the
      first block is returned in block_element_count.
      Note that if you have used ReturnElement(), some
      of these elemements may have been returned.
  Example:
    The loop will iteratate through all the blocks.

          // iterate through all blocks in the pool
          std::size_t block_element_count = 0;
          for ( void* p = FirstBlock(&block_element_count); 
                0 != p; 
                p = NextBlock(&block_element_count) 
              )
          {
            ElementType* e = (ElementType*)p;
            for ( std::size_t i = 0; 
                  i < block_element_count; 
                  i++, e = ((const char*)e) + SizeofElement() 
                )
            {
              ...
            }
          }

  Returns:
    The first block when iterating the list of blocks.
  Remarks:
    The heap for a fixed size memory pool is simply a linked
    list of blocks. FirstBlock() and NextBlock() can be used
    to iterate through the list of blocks.

    Do not make any calls to FirstElement() or NextElement() when using
    FirstBlock() and NextBlock() to iteratate through blocks.

    If you need iterate through a fixed size pool and another
    function may also be in the middle of iterating the pool
    as well, then use ON_FixedSizePoolIterator.  In particular,
    if you have multiple concurrent threads iterating the same 
    fixed size pool, then use ON_FixedSizePoolIterator.
  */
  void* FirstBlock( std::size_t* block_element_count );

  /*
  Description:
    Get the next block when iterating through the blocks.
  Parameters:
    block_element_count - [out] (can be null)
      If not null, the number of elements allocated from the
      block is returned in block_element_count.  Note that if
      you have used ReturnElement(), some of these elemements
      may have been returned.
  Example:
    See the FirstBlock() documentation.
  Returns:
    The next block when iterating through the blocks.
  Remarks:
    Do not make any calls to FirstElement() or NextElement() when using
    FirstBlock() and NextBlock() to iteratate through blocks.

    If you need iterate through a fixed size pool and another
    function may also be in the middle of iterating the pool
    as well, then use ON_FixedSizePoolIterator.  In particular,
    if you have multiple concurrent threads iterating the same 
    fixed size pool, then use ON_FixedSizePoolIterator.
  */
  void* NextBlock( std::size_t* block_element_count );

  /*
  Description:
    Get the i-th elment in the pool.
  Parameters:
    element_index - [in]
  Returns:
    A pointer to the i-th element.  The first element has index = 0
    and is the element returned by the first call to AllocateElement().
    The last element has index = ElementCount()-1.
    If i is out of range, null is returned.
  Remarks:
    It is faster to use FirstElement() and NextElement() to iterate
    through the entire list of elements.  This function is relatively
    efficient when there are a few large blocks in the pool
    or element_index is small compared to the number of elements
    in the first few blocks.

    If ReturnElement() is not used or AllocateElement() calls to
    are made after any use of ReturnElement(), then the i-th 
    element is the one returned by the (i+1)-th call to 
    AllocateElement().
  */
  void* Element(std::size_t element_index) const;

public:
  // Expert user functions below for situations where you
  // need to specify the heap used for this pool.

  /*
  Description:
    Expert user function to specify which heap is used.
  */
  void SetHeap( ON_MEMORY_POOL* heap );

  /*
  Description:
    Expert user function.
  Returns:
    Heap used by this pool.  A null pointer means the default
    heap is being used.
  */
  ON_MEMORY_POOL* Heap();

  /*
  Description:
    Expert user function to call when the heap used by this pool
    is no longer valid.  This call zeros all fields and does not
    call any heap functions.  After calling EmergencyDestroy(), 
    the destructor will not attempt to free any heap.
  */
  void EmergencyDestroy();

private:
  friend class ON_FixedSizePoolIterator;

  void* m_first_block;

  // ReturnElement() adds to the m_al_element stack.
  // AllocateElement() will use the stack before using m_al_element_array[]
  void* m_al_element_stack;

  // used by the iterators
  void* m_qwerty_it_block;
  void* m_qwerty_it_element;

  void* m_al_block; // current element allocation block.
  // m_al_element_array[] is in m_al_block and has length m_al_count.
  void* m_al_element_array;
  std::size_t m_al_count;
  std::size_t m_sizeof_element;
  std::size_t m_block_element_count;  // block element count
  std::size_t m_active_element_count; // number of active elements
  std::size_t m_total_element_count;  // total number of elements (active + returned)
  ON_MEMORY_POOL* m_heap;
  
private:
  // returns capacity of elements in existing block
  std::size_t BlockElementCapacity( const void* block ) const;

  // returns number of allocated of elements in existing block
  std::size_t BlockElementCount( const void* block ) const;
private:
  // prohibit copy construction and operator=.
  ON_FixedSizePool(const ON_FixedSizePool&);
  ON_FixedSizePool& operator=(const ON_FixedSizePool&);
};

class ON_CLASS ON_FixedSizePoolIterator
{
public:
  ON_FixedSizePoolIterator( const class ON_FixedSizePool& fsp );

  const class ON_FixedSizePool& m_fsp;

  /*
  Description:
    Get the first element when iterating through the list of elements.
  Parameters:
    element_index - [in]
      If you use the version of FirstElement() that has an 
      element_index parameter, then the iteration begins at
      that element.
  Example:
    The loop will iteratate through all the elements returned from
    AllocateElement(), including any that have be returned to the pool
    using ReturnElement().

          // iterate through all elements in the pool
          // This iteration will go through TotalElements() items.
          for ( void* p = FirstElement(); 0 != p; p = NextElement() )
          {
            // If you are not using ReturnElement(), then you may process
            // "p" immediately. If you have used ReturnElement(), then you
            // must check some value in p located after the first sizeof(void*)
            // bytes to see if p is active.
            if ( p is not active )
              continue;

            ... process p
          }

  Returns:
    The first element when iterating through the list of elements.
  Remarks:
    FirstElement() and NextElement() will return elements that have 
    been returned to the pool using ReturnElement().  If you use 
    ReturnElement(), then be sure to mark the element so it can be
    identified and skipped.

    Do not make any calls to FirstBlock() or NextBlock() when using
    FirstElement() and NextElement() to iteratate through elements.
  */
  void* FirstElement();
  void* FirstElement( std::size_t element_index );

  /*
  Description:
    Get the next element when iterating through the list of elements.
  Example:
    See the FirstElement() documentation.
  Returns:
    The next element when iterating through the list of elements.
  Remarks:
    FirstElement() and NextElement() will return elements that have 
    been returned to the pool using ReturnElement().  If you use 
    ReturnElement(), then be sure to mark the element so it can be
    identified and skipped.

    Do not make any calls to FirstBlock() or NextBlock() when using
    FirstElement() and NextElement() to iteratate through elements.
  */
  void* NextElement();

  /*
  Description:
    Get a pointer to the first element in the first block.
  Parameters:
    block_element_count - [out] (can be null)
      If not null, the number of elements allocated from the
      first block is returned in block_element_count.
      Note that if you have used ReturnElement(), some
      of these elemements may have been returned.
  Example:
    The loop will iteratate through all the blocks.

          // iterate through all blocks in the pool
          std::size_t block_element_count = 0;
          for ( void* p = FirstBlock(&block_element_count); 
                0 != p; 
                p = NextBlock(&block_element_count) 
              )
          {
            ElementType* e = (ElementType*)p;
            for ( std::size_t i = 0; 
                  i < block_element_count; 
                  i++, e = ((const char*)e) + SizeofElement() 
                )
            {
              ...
            }
          }

  Returns:
    The first block when iterating the list of blocks.
  Remarks:
    The heap for a fixed size memory pool is simply a linked
    list of blocks. FirstBlock() and NextBlock() can be used
    to iterate through the list of blocks.

    Do not make any calls to FirstElement() or NextElement() when using
    FirstBlock() and NextBlock() to iteratate through blocks.
  */
  void* FirstBlock( std::size_t* block_element_count );

  /*
  Description:
    Get the next block when iterating through the blocks.
  Parameters:
    block_element_count - [out] (can be null)
      If not null, the number of elements allocated from the
      block is returned in block_element_count.  Note that if
      you have used ReturnElement(), some of these elemements
      may have been returned.
  Example:
    See the FirstBlock() documentation.
  Returns:
    The next block when iterating through the blocks.
  Remarks:
    Do not make any calls to FirstElement() or NextElement() when using
    FirstBlock() and NextBlock() to iteratate through blocks.
  */
  void* NextBlock( std::size_t* block_element_count );

private:
  void* m_it_block;
  void* m_it_element;

  // no implementation (you can use a copy construtor)
  ON_FixedSizePoolIterator& operator=(const ON_FixedSizePoolIterator&);
};


template <class T> class ON_SimpleFixedSizePool : private ON_FixedSizePool
{
public:
  // construction ////////////////////////////////////////////////////////

  ON_SimpleFixedSizePool();
  ~ON_SimpleFixedSizePool();
  
  /*
  Description:
    Create a fixed size memory pool.
  Parameters:
    element_count_estimate - [in] (0 = good default)
      If you know how many elements you will need, pass that number here.
      It is better to slightly overestimate than to slightly underestimate.
      If you do not have a good estimate, then use zero.
    block_element_count - [in] (0 = good default)
      If block_element_count is zero, Create() will calculate a block
      size that is efficent for most applications.  If you are an expert
      user and want to specify the number of blocks, then pass the number
      of elements per block here.  When block_element_count > 0 and
      element_count_estimate > 0, the first block will be large enough
      element_count_estimate*sizeof(T) bytes; in this case do not
      ask for extraordinarly large amounts of contiguous heap.
  Remarks:
    You must call Create() on an unused ON_FixedSizePool or call Destroy()
    before calling create.
  Returns:
    True if successful and the pool can be used.
  */
  bool Create( 
    std::size_t element_count_estimate,
    std::size_t block_element_count
    );

  /*
  Returns:
    Size of the elements in this pool.
  */
  std::size_t SizeofElement() const;

  /*
  Returns:
    A pointer to sizeof_element bytes.  The memory is zeroed.
  */
  T* AllocateElement();
  
  /*
  Description:
    Return an element to the pool.
  Parameters:
    p - [in]
      A pointer returned by AllocateElement().
      It is critical that p be from this pool and that
      you return a pointer no more than one time.
  Remarks:
    If you find the following remarks confusing, but you really want to use
    ReturnElement(), then here are some simple guidelines.
      1) SizeofElement() must be >= 16
      2) SizeofElement() must be a multiple of 8.
      3) Do not use FirstElement() and NextElement() to iterate through
         the pool.

    If 1 to 3 don't work for you, then you need to understand the following
    information before using ReturnElement().

    ON_FixedMemoryPool uses the first sizeof(void*) bytes of the
    returned element for bookkeeping purposes.  Therefore, if you
    are going to use ReturnElement(), then SizeofElement() must be 
    at least sizeof(void*).  If you are using a platform that requires
    pointers to be aligned on sizeof(void*) boundaries, then
    SizeofElement() must be a multiple of sizeof(void*).
    If you are going to use ReturnElement() and then use FirstElement()
    and NextElement() to iterate through the list of elements, then you
    need to set a value in the returned element to indicate that it
    needs to be skipped during the iteration.  This value cannot be
    located in the fist sizeof(void*) bytes of the element.  If the 
    element is a class with a vtable, you cannot call a virtual 
    function on a returned element because the vtable pointer is 
    trashed when ReturnElement() modifies the fist sizeof(void*) bytes.
  */
  void ReturnElement(T* p);

  /*
  Description:
    Return all allocated elements to the pool. No heap is freed and
    the pool remains initialized and ready for AllocateElement()
    to be called.
  */
  void ReturnAll();

  /*
  Description:
    Destroy the pool and free all the heap. The pool cannot be used again
    until Create() is called.
  */
  void Destroy();

  /*
  Returns:
    Number of active elements. (Elements that have been returned are not active.)
  */
  std::size_t ActiveElementCount() const;

  /*
  Returns:
    Total number of elements = number of active elements + number of returned elements.
  */
  std::size_t TotalElementCount() const;

  /*
  Description:
    Get the next element when iterating through the active elements.
  Example:
    The loop will iteratate through all the elements returned from
    AllocateElement(), including any that have be returned to the pool
    using ReturnElement().

          // iterate through all elements in the pool
          for ( T* p = FirstElement(); 0 != p; p = NextElement() )
          {
            // If you are not using ReturnElement(), then you may process
            // "p" immediately. If you have used ReturnElement(), then you
            // must check some value in p located after the first sizeof(void*)
            // bytes to see if p is active.
            if ( p is not active )
              continue;

            ... process p
          }

  Returns:
    The next element when iterating through the active elements.
  Remarks:
    NextElement() will return elements that have been returned to
    the pool using ReturnElement().  If you use ReturnElement(),
    be sure to mark the element so it can be identified and skipped.
  */
  T* FirstElement();

  /*
  Description:
    Get the next element when iterating through the active elements.
  Example:
    See the FirstElement() documentation.
  Returns:
    The next element when iterating through the active elements.
  Remarks:
    NextElement() will return elements that have been returned to
    the pool using ReturnElement().  If you use ReturnElement(),
    be sure to mark the element so it can be identified and skipped.
  */
  T* NextElement();

  /*
  Description:
    Get a pointer to the first element in the first block.
  Example:
    The loop will iteratate through all the blocks.

          // iterate through all blocks in the pool
          std::size_t block_element_count = 0;
          for ( T* p = FirstBlock(&block_element_count); 
                0 != p; 
                p = NextBlock(&block_element_count) 
              )
          {
            // a[] is an array of length block_element_count
          }

  Returns:
    The next block when iterating the list of blocks.
  Remarks:
    Do not make any calls to FirstElement() or NextElement() when using
    FirstBlock() and NextBlock() to iteratate through blocks.
  */
  T* FirstBlock( std::size_t* block_element_count );

  /*
  Description:
    Get the next block when iterating through the blocks.
  Example:
    See the FirstBlock() documentation.
  Returns:
    The next block when iterating through the blocks.
  Remarks:
    Do not make any calls to FirstElement() or NextElement() when using
    FirstBlock() and NextBlock() to iteratate through blocks.
  */
  T* NextBlock( std::size_t* block_element_count );


  /*
  Description:
    Get the i-th elment in the pool.
  Parameters:
    element_index - [in]
  Returns:
    A pointer to the i-th element.  The first element has index = 0
    and is the element returned by the first call to AllocateElement().
    The last element has index = ElementCount()-1.
    If i is out of range, null is returned.
  Remarks:
    It is faster to use FirstElement() and NextElement() to iterate
    through the entire list of elements.  This function is relatively
    efficient when there are a few large blocks in the pool
    or element_index is small compared to the number of elements
    in the first few blocks.

    If ReturnElement() is not used or AllocateElement() calls to
    are made after any use of ReturnElement(), then the i-th 
    element is the one returned by the (i+1)-th call to 
    AllocateElement().
  */
  T* Element(std::size_t element_index) const;

public:
  // Expert user functions below for situations where you
  // need to specify the heap used for this pool.

  /*
  Description:
    Expert user function to specify which heap is used.
  */
  void SetHeap( ON_MEMORY_POOL* heap );

  /*
  Description:
    Expert user function.
  Returns:
    Heap used by this pool.  A null pointer means the default
    heap is being used.
  */
  ON_MEMORY_POOL* Heap();

  /*
  Description:
    Expert user function to call when the heap used by this pool
    is no longer valid.  This call zeros all fields and does not
    call any heap functions.  After calling EmergencyDestroy(), 
    the destructor will not attempt to free any heap.
  */
  void EmergencyDestroy();

private:
  // prohibit copy construction and operator=.
  ON_SimpleFixedSizePool(const ON_SimpleFixedSizePool<T>&);
  ON_SimpleFixedSizePool<T>& operator=(const ON_SimpleFixedSizePool<T>&);
};

// definitions of the template functions are in a different file
// so that Microsoft's developer studio's autocomplete utility
// will work on the template functions.
#include "opennurbs_fsp_defs.h"

#endif


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

#if !defined(ON_FSP_DEFS_INC_)
#define ON_FSP_DEFS_INC_

template <class T> 
ON_SimpleFixedSizePool<T>::ON_SimpleFixedSizePool()
: ON_FixedSizePool()
{}

template <class T>
ON_SimpleFixedSizePool<T>::~ON_SimpleFixedSizePool()
{ 
  ON_FixedSizePool::Destroy();
}

template <class T>
bool ON_SimpleFixedSizePool<T>::Create( 
  size_t element_count_estimate,
  size_t block_element_count
  )
{
  return ON_FixedSizePool::Create(sizeof(T),element_count_estimate,block_element_count);
}

template <class T>
size_t ON_SimpleFixedSizePool<T>::SizeofElement() const
{
  return ON_FixedSizePool::SizeofElement();
}

template <class T>
T* ON_SimpleFixedSizePool<T>::AllocateElement()
{
  return (T *)ON_FixedSizePool::AllocateElement();
}

template <class T>
void ON_SimpleFixedSizePool<T>::ReturnElement(T* p)
{
  ON_FixedSizePool::ReturnElement(p);
}

template <class T>
void ON_SimpleFixedSizePool<T>::ReturnAll()
{
  ON_FixedSizePool::ReturnAll();
}

template <class T>
void ON_SimpleFixedSizePool<T>::Destroy()
{
  ON_FixedSizePool::Destroy();
}

template <class T>
size_t ON_SimpleFixedSizePool<T>::ActiveElementCount() const
{
  return ON_FixedSizePool::ActiveElementCount();
}

template <class T>
size_t ON_SimpleFixedSizePool<T>::TotalElementCount() const
{
  return ON_FixedSizePool::TotalElementCount();
}

template <class T>
T* ON_SimpleFixedSizePool<T>::FirstElement()
{
  return (T *)ON_FixedSizePool::FirstElement();
}

template <class T>
T* ON_SimpleFixedSizePool<T>::NextElement()
{
  return (T *)ON_FixedSizePool::NextElement();
}

template <class T>
T* ON_SimpleFixedSizePool<T>::FirstBlock( size_t* block_element_count )
{
  return (T *)ON_FixedSizePool::FirstBlock(block_element_count);
}

template <class T>
T* ON_SimpleFixedSizePool<T>::NextBlock( size_t* block_element_count )
{
  return (T *)ON_FixedSizePool::NextBlock(block_element_count);
}

template <class T>
T* ON_SimpleFixedSizePool<T>::Element(size_t element_index) const
{
  return (T *)ON_FixedSizePool::Element(element_index);
}

template <class T>
void ON_SimpleFixedSizePool<T>::SetHeap( ON_MEMORY_POOL* heap )
{
  ON_FixedSizePool::SetHeap(heap);
}

template <class T>
ON_MEMORY_POOL* ON_SimpleFixedSizePool<T>::Heap()
{
  return ON_FixedSizePool::Heap();
}

template <class T>
void ON_SimpleFixedSizePool<T>::EmergencyDestroy()
{
  ON_FixedSizePool::EmergencyDestroy();
}

#endif

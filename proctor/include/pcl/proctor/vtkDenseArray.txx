/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkDenseArray.txx

-------------------------------------------------------------------------
  Copyright 2008 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
  the U.S. Government retains certain rights in this software.
-------------------------------------------------------------------------

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef __vtkDenseArray_txx
#define __vtkDenseArray_txx

///////////////////////////////////////////////////////////////////////////////
// vtkDenseArray::MemoryBlock

template<typename T>
vtkDenseArray<T>::MemoryBlock::~MemoryBlock()
{
}

///////////////////////////////////////////////////////////////////////////////
// vtkDenseArray::HeapMemoryBlock

template<typename T>
vtkDenseArray<T>::HeapMemoryBlock::HeapMemoryBlock(const vtkArrayExtents& extents) :
  Storage(new T[extents.GetSize()])
{
}

template<typename T>
vtkDenseArray<T>::HeapMemoryBlock::~HeapMemoryBlock()
{
  delete[] this->Storage;
}

template<typename T>
T* vtkDenseArray<T>::HeapMemoryBlock::GetAddress()
{
  return this->Storage;
}

///////////////////////////////////////////////////////////////////////////////
// vtkDenseArray::StaticMemoryBlock

template<typename T>
vtkDenseArray<T>::StaticMemoryBlock::StaticMemoryBlock(T* const storage) :
  Storage(storage)
{
}

template<typename T>
T* vtkDenseArray<T>::StaticMemoryBlock::GetAddress()
{
  return this->Storage;
}

///////////////////////////////////////////////////////////////////////////////
// vtkDenseArray

template<typename T>
vtkDenseArray<T>* vtkDenseArray<T>::New()
{
  vtkObject* ret = vtkObjectFactory::CreateInstance(typeid(ThisT).name());
  if(ret)
    {
    return static_cast<ThisT*>(ret);
    }
  return new ThisT();
}

template<typename T>
void vtkDenseArray<T>::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkDenseArray<T>::Superclass::PrintSelf(os, indent);
}

template<typename T>
bool vtkDenseArray<T>::IsDense()
{
  return true;
}

template<typename T>
const vtkArrayExtents& vtkDenseArray<T>::GetExtents()
{
  return this->Extents;
}

template<typename T>
typename vtkDenseArray<T>::SizeT vtkDenseArray<T>::GetNonNullSize()
{
  return this->Extents.GetSize();
}

template<typename T>
void vtkDenseArray<T>::GetCoordinatesN(const SizeT n, vtkArrayCoordinates& coordinates)
{
  coordinates.SetDimensions(this->GetDimensions());

  vtkIdType divisor = 1;
  for(DimensionT i = 0; i < this->GetDimensions(); ++i)
    {
    coordinates[i] = ((n / divisor) % this->Extents[i].GetSize()) + this->Extents[i].GetBegin();
    divisor *= this->Extents[i].GetSize();
    }
}

template<typename T>
vtkArray* vtkDenseArray<T>::DeepCopy()
{
  vtkDenseArray<T>* const copy = vtkDenseArray<T>::New();

  copy->SetName(this->GetName());
  copy->Resize(this->Extents);
  copy->DimensionLabels = this->DimensionLabels;
  std::copy(this->Begin, this->End, copy->Begin);

  return copy;
}

template<typename T>
const T& vtkDenseArray<T>::GetValue(CoordinateT i)
{
  if(1 != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    static T temp;
    return temp;
    }

  return this->Begin[this->MapCoordinates(i)];
}

template<typename T>
const T& vtkDenseArray<T>::GetValue(CoordinateT i, CoordinateT j)
{
  if(2 != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    static T temp;
    return temp;
    }

  return this->Begin[this->MapCoordinates(i, j)];
}

template<typename T>
const T& vtkDenseArray<T>::GetValue(CoordinateT i, CoordinateT j, CoordinateT k)
{
  if(3 != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    static T temp;
    return temp;
    }

  return this->Begin[this->MapCoordinates(i, j, k)];
}

template<typename T>
const T& vtkDenseArray<T>::GetValue(const vtkArrayCoordinates& coordinates)
{
  if(coordinates.GetDimensions() != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    static T temp;
    return temp;
    }

  return this->Begin[this->MapCoordinates(coordinates)];
}

template<typename T>
const T& vtkDenseArray<T>::GetValueN(const SizeT n)
{
  return this->Begin[n];
}

template<typename T>
void vtkDenseArray<T>::SetValue(CoordinateT i, const T& value)
{
  if(1 != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    return;
    }

  this->Begin[this->MapCoordinates(i)] = value;
}

template<typename T>
void vtkDenseArray<T>::SetValue(CoordinateT i, CoordinateT j, const T& value)
{
  if(2 != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    return;
    }

  this->Begin[this->MapCoordinates(i, j)] = value;
}

template<typename T>
void vtkDenseArray<T>::SetValue(CoordinateT i, CoordinateT j, CoordinateT k, const T& value)
{
  if(3 != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    return;
    }

  this->Begin[this->MapCoordinates(i, j, k)] = value;
}

template<typename T>
void vtkDenseArray<T>::SetValue(const vtkArrayCoordinates& coordinates, const T& value)
{
  if(coordinates.GetDimensions() != this->GetDimensions())
    {
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    return;
    }

  this->Begin[this->MapCoordinates(coordinates)] = value;
}

template<typename T>
void vtkDenseArray<T>::SetValueN(const SizeT n, const T& value)
{
  this->Begin[n] = value;
}

template<typename T>
void vtkDenseArray<T>::ExternalStorage(const vtkArrayExtents& extents, MemoryBlock* storage)
{
  this->Reconfigure(extents, storage);
}

template<typename T>
void vtkDenseArray<T>::Fill(const T& value)
{
  std::fill(this->Begin, this->End, value);
}

template<typename T>
T& vtkDenseArray<T>::operator[](const vtkArrayCoordinates& coordinates)
{
  if(coordinates.GetDimensions() != this->GetDimensions())
    {
    static T temp;
    vtkErrorMacro(<< "Index-array dimension mismatch.");
    return temp;
    }

  return this->Begin[this->MapCoordinates(coordinates)];
}

template<typename T>
const T* vtkDenseArray<T>::GetStorage() const
{
  return this->Begin;
}

template<typename T>
T* vtkDenseArray<T>::GetStorage()
{
  return this->Begin;
}

template<typename T>
vtkDenseArray<T>::vtkDenseArray() :
  Storage(0),
  Begin(0),
  End(0)
{
}

template<typename T>
vtkDenseArray<T>::~vtkDenseArray()
{
  delete this->Storage;

  this->Storage = 0;
  this->Begin = 0;
  this->End = 0;
}

template<typename T>
void vtkDenseArray<T>::InternalResize(const vtkArrayExtents& extents)
{
  this->Reconfigure(extents, new HeapMemoryBlock(extents));
}

template<typename T>
void vtkDenseArray<T>::InternalSetDimensionLabel(DimensionT i, const vtkStdString& label)
{
  this->DimensionLabels[i] = label;
}

template<typename T>
vtkStdString vtkDenseArray<T>::InternalGetDimensionLabel(DimensionT i)
{
  return this->DimensionLabels[i];
}

template<typename T>
vtkIdType vtkDenseArray<T>::MapCoordinates(CoordinateT i)
{
  return ((i + this->Offsets[0]) * this->Strides[0]);
}

template<typename T>
vtkIdType vtkDenseArray<T>::MapCoordinates(CoordinateT i, CoordinateT j)
{
  return ((i + this->Offsets[0]) * this->Strides[0]) + ((j + this->Offsets[1]) * this->Strides[1]);
}

template<typename T>
vtkIdType vtkDenseArray<T>::MapCoordinates(CoordinateT i, CoordinateT j, CoordinateT k)
{
  return ((i + this->Offsets[0]) * this->Strides[0]) + ((j + this->Offsets[1]) * this->Strides[1]) + ((k + this->Offsets[2]) * this->Strides[2]);
}

template<typename T>
vtkIdType vtkDenseArray<T>::MapCoordinates(const vtkArrayCoordinates& coordinates)
{
  vtkIdType index = 0;
  for(vtkIdType i = 0; i != static_cast<vtkIdType>(this->Strides.size()); ++i)
    index += ((coordinates[i] + this->Offsets[i]) * this->Strides[i]);

  return index;
}

template<typename T>
void vtkDenseArray<T>::Reconfigure(const vtkArrayExtents& extents, MemoryBlock* storage)
{
  this->Extents = extents;
  this->DimensionLabels.resize(extents.GetDimensions(), vtkStdString());

  delete this->Storage;
  this->Storage = storage;
  this->Begin = storage->GetAddress();
  this->End = this->Begin + extents.GetSize();

  this->Offsets.resize(extents.GetDimensions());
  for(DimensionT i = 0; i != extents.GetDimensions(); ++i)
    {
    this->Offsets[i] = -extents[i].GetBegin();
    }

  this->Strides.resize(extents.GetDimensions());
  for(DimensionT i = 0; i != extents.GetDimensions(); ++i)
    {
    if(i == 0)
      this->Strides[i] = 1;
    else
      this->Strides[i] = this->Strides[i-1] * extents[i-1].GetSize();
    }
}

#endif

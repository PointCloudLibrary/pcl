/*=========================================================================
  
  Program:   Visualization Toolkit
  Module:    vtkVertexBufferObject.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include <pcl/visualization/vtk/vtkVertexBufferObject.h>

#include "vtkCellArray.h"
#include "vtkDataArray.h"
#include "vtkObject.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLExtensionManager.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkPoints.h"
#include "vtkRenderWindow.h"
#include "vtkUnsignedCharArray.h"

#include "vtkgl.h" // Needed for gl data types exposed in API
#include "vtkOpenGL.h"

//#define VTK_PBO_DEBUG
//#define VTK_PBO_TIMING

#ifdef VTK_PBO_TIMING
#include "vtkTimerLog.h"
#endif

// Mapping from Usage values to OpenGL values.

GLenum OpenGLVertexBufferObjectUsage[9]=
{
  vtkgl::STREAM_DRAW,
  vtkgl::STREAM_READ,
  vtkgl::STREAM_COPY,
  vtkgl::STATIC_DRAW,
  vtkgl::STATIC_READ,
  vtkgl::STATIC_COPY,
  vtkgl::DYNAMIC_DRAW,
  vtkgl::DYNAMIC_READ,
  vtkgl::DYNAMIC_COPY
};

const char *VertexBufferObjectUsageAsString[9]=
{
  "StreamDraw",
  "StreamRead",
  "StreamCopy",
  "StaticDraw",
  "StaticRead",
  "StaticCopy",
  "DynamicDraw",
  "DynamicRead",
  "DynamicCopy"
};

#ifdef  VTK_PBO_DEBUG
#include <pthread.h> // for debugging with MPI, pthread_self()
#endif

vtkStandardNewMacro(vtkVertexBufferObject);
//----------------------------------------------------------------------------
vtkVertexBufferObject::vtkVertexBufferObject()
{
  this->Handle = 0;
  this->Context = 0;
  this->BufferTarget = 0;
  this->Size=0;
  this->Count=0;
//  this->Type=VTK_UNSIGNED_CHAR;
  this->Usage=StaticDraw;
}

//----------------------------------------------------------------------------
vtkVertexBufferObject::~vtkVertexBufferObject()
{
  this->SetContext(0);
}

//----------------------------------------------------------------------------
// Description:
// Returns if the context supports the required extensions.
bool vtkVertexBufferObject::IsSupported(vtkRenderWindow* win)
{
  vtkOpenGLRenderWindow* renWin = vtkOpenGLRenderWindow::SafeDownCast(win);
  if (renWin)
    {
    vtkOpenGLExtensionManager* mgr = renWin->GetExtensionManager();
    
    bool vbo=mgr->ExtensionSupported("GL_VERSION_1_5") ||
      mgr->ExtensionSupported("GL_ARB_vertex_buffer_object");
    
    return vbo;
    }
  return false;
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::LoadRequiredExtensions(
  vtkOpenGLExtensionManager* mgr)
{
  bool gl15 = mgr->ExtensionSupported("GL_VERSION_1_5")==1;

  bool vbo = gl15 || mgr->ExtensionSupported("GL_ARB_vertex_buffer_object");
  
  if(vbo)
    {
    if(gl15)
      {
      mgr->LoadExtension("GL_VERSION_1_5");
      }
    else
      {
      mgr->LoadCorePromotedExtension("GL_ARB_vertex_buffer_object");
      }
    }
  return vbo;
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::SetContext(vtkRenderWindow* renWin)
{
  if (this->Context == renWin)
    {
    return;
    }
  
  this->DestroyBuffer(); 
  
  vtkOpenGLRenderWindow* openGLRenWin =
    vtkOpenGLRenderWindow::SafeDownCast(renWin);
  this->Context = openGLRenWin;
  if (openGLRenWin)
    {
    if (!this->LoadRequiredExtensions(openGLRenWin->GetExtensionManager()))
      {
      this->Context = 0;
      vtkErrorMacro("Required OpenGL extensions not supported by the context.");
      }
    }
  
  this->Modified();
}

//----------------------------------------------------------------------------
vtkRenderWindow* vtkVertexBufferObject::GetContext()
{
  return this->Context;
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::Bind()
{
  if (!this->Context)
    {
    vtkErrorMacro("No context specified. Cannot Bind.");
    return;
    }
  
  this->CreateBuffer();
  
  vtkgl::BindBuffer(static_cast<GLenum>(this->BufferTarget), this->Handle);

  vtkGraphicErrorMacro(this->Context,"after BindBuffer");
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::UnBind()
{
  if (this->Context && this->Handle && this->BufferTarget)
    {
    vtkgl::BindBuffer(this->BufferTarget, 0);
    vtkGraphicErrorMacro(this->Context,"after BindBuffer");
    //this->BufferTarget = 0;
    }
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::CreateBuffer()
{
  this->Context->MakeCurrent();
  if (!this->Handle)
    {
    cout << "Creating Buffer..." << endl;
    GLuint ioBuf;
    vtkgl::GenBuffers(1, &ioBuf);
    vtkGraphicErrorMacro(this->Context, "after GenBuffers");
    this->Handle = ioBuf;
    }
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::DestroyBuffer()
{
  if (this->Context && this->Handle)
    {
    GLuint ioBuf = static_cast<GLuint>(this->Handle);
    vtkgl::DeleteBuffers(1, &ioBuf);
    }
  this->Handle = 0;
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::SetData(vtkPoints *points)
{
  std::cout << "Setting Points" << endl;
  this->Count = points->GetNumberOfPoints();
  this->Size = this->Count * 3 * sizeof(float);
  this->BufferTarget = vtkgl::ARRAY_BUFFER;

  return this->SetData(points->GetVoidPointer(0));
}

//----------------------------------------------------------------------------
//bool vtkVertexBufferObject::SetData(vtkCellArray *verts)
//{
//  // The correct way to do this is to use each cells vtkIdTypes, but this
//  // returns a tuple (1, index) which I don't have time to understand right
//  // now
//  //vtkIdType *oldarray = input->GetVerts()->GetPointer();
//  //int *newarray = new int[numPoints];
//  //std::copy(oldarray, oldarray + numPoints, newarray);
//  //for (size_t i=0; i < numPoints*2; i+=2)
//  //  std::cout << "really" << oldarray[i] << endl;
//
//  this->Size = points->GetNumberOfPoints() * sizeof(unsigned int);
//  this->BufferTarget = vtkgl::ELEMENT_ARRAY_BUFFER;
//
//  // For now, display every indice
//  unsigned int *indices = new unsigned int[numPoints];
//  for (size_t i=0; i < numPoints; i++){
//    indices[i] = i;
//  }
//
//  this->SetData((void *)indices);
//}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::SetData(std::vector<unsigned int> *indices)
{
  std::cout << "Setting Indices" << endl;
  this->Count = indices->size();
  this->Size = this->Count * sizeof(unsigned int);
  this->BufferTarget = vtkgl::ELEMENT_ARRAY_BUFFER;

  return this->SetData(reinterpret_cast<GLvoid *>(indices->data()));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::SetData(vtkDataArray *colors)
{
  std::cout << "Setting Colors" << endl;
  //this->Count = colors->GetNumberOfTuples();
  this->Count = colors->GetSize();
  std::cout << "Type: " << colors->GetDataTypeAsString() <<std::endl;
  this->Size = this->Count * 1 * sizeof(float);
  this->BufferTarget = vtkgl::ARRAY_BUFFER;
  std::cout << "Setting Data" << endl;

  return this->SetData(reinterpret_cast<GLvoid *>(colors->GetVoidPointer(0)));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::SetData(vtkUnsignedCharArray *colors)
{
  std::cout << "Setting Colors" << endl;
  std::cout << "Colors == NULL: " << (colors == NULL) << endl;
  this->Count = colors->GetSize();
  std::cout << "Type: " << colors->GetDataTypeAsString() <<std::endl;
  this->Size = this->Count * 1 * sizeof(float);
  this->BufferTarget = vtkgl::ARRAY_BUFFER;

  std::cout << "Setting Data" << endl;
  return this->SetData(reinterpret_cast<GLvoid *>(colors->GetPointer(0)));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::SetData(GLvoid* data)
{
  if (!this->Context)
     {
     vtkErrorMacro("No context specified. Cannot upload data.");
     return false;
     }

  this->CreateBuffer();

//  cout << "  Context: " << this->Context << endl;
//  cout << "  Handle: " << this->Handle << endl;
//  cout << "  Size: " << this->Size << endl;
//  cout << "  Count: " << this->Count << endl;
//  cout << "  Usage: " << VertexBufferObjectUsageAsString[this->Usage] << endl;

  GLenum usage = OpenGLVertexBufferObjectUsage[this->Usage];

  this->Bind();
    vtkgl::BufferData(this->BufferTarget, this->Size, data, usage);
  this->UnBind();

  cout << "Buffer Created" << endl;
  return true;
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::ReleaseMemory()
{
  if (this->Context && this->Handle)
    {
    this->Bind();
    vtkgl::BufferData(this->BufferTarget, 0, NULL, OpenGLVertexBufferObjectUsage[this->Usage]);
    this->Size = 0;
    this->Count = 0;
    }
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Context: " << this->Context << endl;
  os << indent << "Handle: " << this->Handle << endl;
  os << indent << "Size: " << this->Size << endl;
  os << indent << "Count: " << this->Count << endl;
  os << indent << "Usage:" << VertexBufferObjectUsageAsString[this->Usage] << endl;
}

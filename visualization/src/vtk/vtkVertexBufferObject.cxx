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

#define GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX 0x9048
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

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
  this->ArrayType = 0;
  this->Usage=StaticDraw;
  this->AttributeIndex = -1;
  this->AttributeSize = 0;
  this->AttributeType = GL_INVALID_ENUM;
  this->AttributeNormalized = GL_FALSE;
  this->AttributeStride = 0;
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
  
//  const char *glVendor(reinterpret_cast<const char *>(glGetString(GL_VENDOR)));

//  // Get available gpu memory
//  const char *glRenderer(reinterpret_cast<const char *>(glGetString(GL_RENDERER)));
//  const char *glVersion(reinterpret_cast<const char *>(glGetString(GL_VERSION)));
//
//  if (strncmp(glVendor, "NVIDIA", 6) == 0){
//    if (mgr->ExtensionSupported("GL_NVX_gpu_memory_info")){
//      GLint nTotalMemoryInKB = 0;
//        glGetIntegerv(GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX, &nTotalMemoryInKB);
//    }
//  }

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
  
  vtkOpenGLRenderWindow* openGLRenWin = vtkOpenGLRenderWindow::SafeDownCast(renWin);
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
//int vtkVertexBufferObject::GetAttributeIndex(){
//  return this->AttributeIndex;
//}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::SetUserDefinedAttribute(int index, bool normalized/*=false*/, int stride/*=0*/)
{
  this->AttributeIndex = index;
  SetAttributeNormalized(normalized);
  this->AttributeStride = stride;
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::ResetUserDefinedAttribute()
{
  this->AttributeIndex = -1;
}

void vtkVertexBufferObject::SetAttributeNormalized(bool normalized)
{
  if (normalized)
  {
    this->AttributeNormalized = GL_TRUE;
  }
  else
  {
    this->AttributeNormalized = GL_FALSE;
  }

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

  if(this->AttributeIndex >= 0){
    vtkgl::VertexAttribPointer(this->AttributeIndex,
                               this->AttributeSize,
                               this->AttributeType,
                               static_cast<GLboolean> (this->AttributeNormalized),
                               this->AttributeStride,
                               0);
    vtkgl::EnableVertexAttribArray(this->AttributeIndex);
  } else {
    glEnableClientState(this->ArrayType);
    switch(this->ArrayType){
      case GL_VERTEX_ARRAY:
        glVertexPointer(this->AttributeSize, this->AttributeType, this->AttributeStride, 0);
        break;

      case GL_INDEX_ARRAY:
        glIndexPointer(this->AttributeType, this->AttributeStride, 0);
        break;

      case GL_COLOR_ARRAY:
        glColorPointer(this->AttributeSize, this->AttributeType, this->AttributeStride, 0);
        break;

      case GL_NORMAL_ARRAY:
        glNormalPointer(this->AttributeType, this->AttributeStride, 0);
        break;

      default:
        vtkErrorMacro("Unsupported Array Type: " << this->ArrayType)
    }
  }
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::UnBind()
{
  if (this->Context && this->Handle && this->BufferTarget){
    vtkgl::BindBuffer(this->BufferTarget, 0);

    if(this->AttributeIndex >= 0){
      vtkgl::DisableVertexAttribArray(this->AttributeIndex);
    } else {
      glDisableClientState(this->ArrayType);
    }
  }
}

//----------------------------------------------------------------------------
void vtkVertexBufferObject::CreateBuffer()
{
  this->Context->MakeCurrent();
  if (!this->Handle)
    {
    GLuint ioBuf;
    vtkgl::GenBuffers(1, &ioBuf);
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
int vtkVertexBufferObject::GetDataTypeSize(int type)
{
  switch (type)
    {
    vtkTemplateMacro(
      return sizeof(static_cast<VTK_TT>(0))
      );

    case VTK_BIT:
      return 0;
      break;

    case VTK_STRING:
      return 0;
      break;

    case VTK_UNICODE_STRING:
      return 0;
      break;

    default:
      vtkGenericWarningMacro(<<"Unsupported data type!");
    }

  return 1;
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(vtkPoints *points)
{
  this->Count = static_cast<unsigned int> (points->GetNumberOfPoints());
  this->AttributeSize = points->GetData()->GetNumberOfComponents();
  this->AttributeType = GL_FLOAT;
  this->Size = this->Count * this->AttributeSize * GetDataTypeSize(points->GetDataType());
  this->BufferTarget = vtkgl::ARRAY_BUFFER;
  this->ArrayType = GL_VERTEX_ARRAY;

  return this->Upload(points->GetVoidPointer(0));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(vtkCellArray *verts)
{
  vtkIdType npts;
  vtkIdType *pts;
  std::vector<unsigned int> indices;

  verts->InitTraversal();
  while(verts->GetNextCell(npts, pts) != 0){
    for (size_t i=0; i < static_cast<size_t>(npts); i++)
        indices.push_back(static_cast<unsigned int> (pts[i]));
  }

  this->Count = static_cast<unsigned int> (indices.size());
  this->AttributeSize = 1;
  this->AttributeType = GL_INT;
  this->Size = static_cast<unsigned int> (this->Count * sizeof(unsigned int));
  this->BufferTarget = vtkgl::ELEMENT_ARRAY_BUFFER;
  this->ArrayType = GL_INDEX_ARRAY;

  return this->Upload(&indices[0], static_cast <unsigned int> (indices.size()));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(unsigned int *indices, unsigned int count)
{
  this->Count = count;
  this->AttributeSize = 1;
  this->AttributeType = GL_INT;
  this->Size = static_cast<unsigned int> (this->Count * sizeof(unsigned int));
  this->BufferTarget = vtkgl::ELEMENT_ARRAY_BUFFER;
  this->ArrayType = GL_INDEX_ARRAY;

  return this->Upload(reinterpret_cast<GLvoid *>(indices));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::UploadNormals(vtkDataArray *normals)
{
  return Upload(normals, GL_FLOAT, GL_NORMAL_ARRAY);
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::UploadColors(vtkDataArray *colors)
{
  return Upload(colors, GL_UNSIGNED_BYTE, GL_COLOR_ARRAY);
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(vtkDataArray *array)
{
//  cout << "vtkDataArray" << endl;
  if (!array)
    return false;

  this->Count = static_cast<unsigned int> (array->GetNumberOfTuples());
  this->AttributeSize = 3;
  this->AttributeType = GL_UNSIGNED_BYTE;
  if(array->GetNumberOfComponents() == 1){
//    cout << "in here" << endl;
    this->AttributeType = GL_UNSIGNED_INT;
  }
  this->Size = this->Count * array->GetNumberOfComponents() * GetDataTypeSize(array->GetDataType());
  this->BufferTarget = vtkgl::ARRAY_BUFFER;
  this->ArrayType = GL_COLOR_ARRAY;

//  cout << "Count: " << this->Count << endl;
//  cout << "Attribute Size: " << this->AttributeSize << endl;
//  cout << "Size: " << this->Size << endl;
//  cout << "Usage:" << VertexBufferObjectUsageAsString[this->Usage] << endl;

  return this->Upload(reinterpret_cast<GLvoid *>(array->GetVoidPointer(0)));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(vtkDataArray *array, int attributeType, int arrayType)
{
//  cout << "vtkDataArray - attributeType/Array" << endl;
  this->Count = static_cast<unsigned int> (array->GetNumberOfTuples());
  this->AttributeSize = array->GetNumberOfComponents();
  this->AttributeType = attributeType;
  this->Size = this->Count * this->AttributeSize * GetDataTypeSize(array->GetDataType());
  this->BufferTarget = vtkgl::ARRAY_BUFFER;
  this->ArrayType = arrayType;

  return this->Upload(reinterpret_cast<GLvoid *>(array->GetVoidPointer(0)));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(vtkUnsignedCharArray *colors)
{
//  cout << "vtkUnsignedCharArray" << endl;
  this->Count = static_cast<unsigned int> (colors->GetNumberOfTuples());
  this->AttributeSize = colors->GetNumberOfComponents();
  this->AttributeType = GL_UNSIGNED_BYTE;
  this->Size = this->Count * this->AttributeSize * GetDataTypeSize(colors->GetDataType());
  this->BufferTarget = vtkgl::ARRAY_BUFFER;
  this->ArrayType = GL_COLOR_ARRAY;

  return this->Upload(reinterpret_cast<GLvoid *>(colors->GetPointer(0)));
}

//----------------------------------------------------------------------------
bool vtkVertexBufferObject::Upload(GLvoid* data)
{
  if (!this->Context)
     {
     vtkErrorMacro("No context specified. Cannot upload data.");
     return false;
     }

  this->CreateBuffer();

  GLenum usage = OpenGLVertexBufferObjectUsage[this->Usage];

//  std::cout << "Pushing " << (this->Size / 1024 / 1024) << "MB to GPU" << std::endl;

  this->Bind();
    vtkgl::BufferData(this->BufferTarget, this->Size, data, usage);
  this->UnBind();

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

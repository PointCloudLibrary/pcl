/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVertexBufferObjectMapper.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include <iostream>

#include "pcl/visualization/vtk/vtkVertexBufferObject.h"
#include "pcl/visualization/vtk/vtkVertexBufferObjectMapper.h"

#include "vtkVersion.h"
#include "vtkCellData.h"
#include "vtkExecutive.h"
#include "vtkInformation.h"
#include "vtkMath.h"
#include "vtkgl.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGL.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkPainterDeviceAdapter.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkShaderProgram2.h"
#include "vtkShader2.h"
#include "vtkShader2Collection.h"
#include "vtkUniformVariables.h"

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVertexBufferObjectMapper);

//----------------------------------------------------------------------------
vtkVertexBufferObjectMapper::vtkVertexBufferObjectMapper()
{
  initialized = false;
//  shadersInitialized = false;

  program = NULL;
  vertexVbo = vtkVertexBufferObject::New();
  indiceVbo = vtkVertexBufferObject::New();
  colorVbo = vtkVertexBufferObject::New();
  normalVbo = vtkVertexBufferObject::New();
//  normalIndiceVbo = vtkVertexBufferObject::New();

}

//----------------------------------------------------------------------------
void vtkVertexBufferObjectMapper::Render(vtkRenderer *ren, vtkActor *act)
{
  ren->GetRenderWindow()->MakeCurrent();

  if (this->program)
      this->program->SetContext(vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()));

//  this->InvokeEvent(vtkCommand::StartEvent,NULL);
//  this->GetInputAlgorithm()->Update();
//  this->InvokeEvent(vtkCommand::EndEvent,NULL);

  ren->GetRenderWindow()->MakeCurrent();
  this->SetColorModeToMapScalars();
  this->MapScalars(act->GetProperty()->GetOpacity());

  if (!this->initialized)
  {
    createVBOs(ren->GetRenderWindow());
    initialized = true;
  }

  // If full transparency
  vtkProperty *prop = act->GetProperty();
  if (prop->GetOpacity() <= 0.0)
    return;

  // Set point size
  glPointSize(prop->GetPointSize());

  // Use program
  if (this->program)
    this->program->Use();

  // Bind vertices and indices
  vertexVbo->Bind();
  indiceVbo->Bind();

  // Bind colors if present
  if (this->Colors)
    colorVbo->Bind();

  // Bind normals if present
  if (this->GetInput()->GetCellData()->GetNormals())
    normalVbo->Bind();

  // Draw
  ren->GetRenderWindow()->GetPainterDeviceAdapter()->DrawElements(VTK_VERTEX, indiceVbo->GetCount(), VTK_UNSIGNED_INT, 0);
  //glDrawElements(GL_POINTS, indiceVbo->GetCount(), GL_UNSIGNED_INT, 0);

  // Unbind vertices and indices
  vertexVbo->UnBind();
  indiceVbo->UnBind();

  // Unbind colors if present
  if (this->Colors)
    colorVbo->UnBind();

  // Unbind normals if present
  if (this->GetInput()->GetCellData()->GetNormals())
    normalVbo->UnBind();

  if (this->program)
    this->program->Restore();

//  //Display Normals (WIP for use with a geometry shader)
//  vtkPolyData *input= this->GetInput();
//  vtkDataArray *normals = input->GetCellData()->GetNormals();
//  if (normals){
//    if (this->program)
//        this->program->Use();
//    vertexVbo->Bind();
//    normalVbo->Bind();
//    normalIndiceVbo->Bind();
//
//    ren->GetRenderWindow()->GetPainterDeviceAdapter()->DrawElements(VTK_VERTEX, normalIndiceVbo->GetCount(), VTK_UNSIGNED_INT, 0);
//
//    vertexVbo->UnBind();
//    normalVbo->UnBind();
//    normalIndiceVbo->UnBind();
//    if (this->program)
//        this->program->Restore();
//  }
}

//----------------------------------------------------------------------------
void vtkVertexBufferObjectMapper::SetInput(vtkPolyData *input)
{
//  std::cout << "SetInput" << endl;
  if(input)
  {
#if VTK_MAJOR_VERSION < 6
    this->SetInputConnection(0, input->GetProducerPort());
#else
    this->SetInputDataObject (0, input);
#endif
  }
  else
  {
    // Setting a NULL input removes the connection.
    this->SetInputConnection(0, 0);
  }
  initialized = false;

}

void vtkVertexBufferObjectMapper::SetInput(vtkDataSet *input)
{
//  std::cout << "SetInput" << endl;
  if(input)
  {
#if VTK_MAJOR_VERSION < 6
    this->SetInputConnection(0, input->GetProducerPort());
#else
    this->SetInputDataObject (0, input);
#endif
  }
  else
  {
    // Setting a NULL input removes the connection.
    this->SetInputConnection(0, 0);
  }
}

void vtkVertexBufferObjectMapper::createVBOs(vtkRenderWindow* win)
{

  vtkPolyData *input= this->GetInput();

  // Create vertex buffer
  vertexVbo->SetContext(win);
  vertexVbo->Upload(input->GetPoints());

  // Create index buffer
  indiceVbo->SetContext(win);
  indiceVbo->SetUsage(vtkVertexBufferObject::DynamicDraw);
  indiceVbo->Upload(input->GetVerts());

//  int cellFlag = 0;
//  vtkDataArray *scalars = vtkAbstractMapper::
//      GetScalars(this->GetInput(), this->ScalarMode, this->ArrayAccessMode,
//                 this->ArrayId, this->ArrayName, cellFlag);

//  cout << "Color Mode" << this->GetColorModeAsString() << endl;
//
//  cout << "ArrayId " << this->ArrayId << endl;
//  cout << "ArrayName " << this->ArrayName << endl;
//  cout << "ScalarVisibility " << this->ScalarVisibility << endl;
//  cout << "scalars " << scalars << endl;
//  cout << "GetLookupTable " << scalars->GetLookupTable() << endl;
//  cout << "LookupTable " << this->LookupTable  << endl;
//  cout << "UseLookupTableScalarRange " << this->UseLookupTableScalarRange << endl;
//  cout << "InterpolateScalarsBeforeMapping " << this->InterpolateScalarsBeforeMapping << endl;

//  vtkDataArray *scalars = input->GetPointData()->GetScalars();
//  cout << "Number of tuples:" << scalars->GetNumberOfTuples() << endl;
//  cout << "Number of components:" << scalars->GetNumberOfComponents() << endl;
//  float rgb[3];
//  scalars->GetTuple(0, rgb);
//  cout << "r: " << rgb[0] << "\tg: " << rgb[1] << "\tb: " << rgb[2] << endl;

//  if(input->GetPointData()->GetScalars()){
//    colorVbo->SetContext(win);
//
//    int rgb = scalars->GetTuple1(0);
//      uint8_t r = (rgb >> 16) & 0x0000ff;
//      uint8_t g = (rgb >> 8)  & 0x0000ff;
//      uint8_t b = (rgb)     & 0x0000ff;
//    cout << "r: " << r << "\tg: " << g<< "\tb: " << b << endl;
//
//    colorVbo->SetAttributeNormalized(true);
//    colorVbo->UploadColors(input->GetPointData()->GetScalars());
//    colorVbo->Upload(input->GetPointData()->GetScalars());
//  }

  // todo: This is VERY BAD!  We need to figure out where the scalar data is coming from, ie PointData, CellData etc.
  //       Then use this as our color data.  In addition, scalars need to be mapped correctly via LUTs.  Although,
  //       writing a glsl shader for these mappings would probably be faster.
  vtkDataArray *scalars = input->GetCellData()->GetScalars();
  if (scalars)
  {
    colorVbo->SetContext(win);
    colorVbo->Upload(input->GetCellData()->GetScalars());
  }

//  // Create color buffer
//  this->Colors = this->MapScalars(1.0);
//  if (this->Colors)
//  {
//
//    colorVbo->SetContext(win);
//    colorVbo->Upload(this->Colors);
//    colorVbo->SetAttributeNormalized(true);
//    colorVbo->UploadColors(this->Colors);
//
//    cout << "Number of tuples:" << this->Colors->GetNumberOfTuples() << endl;
//              //for (size_t i = 0; i < scalars->GetNumberOfTuples(); i++){
//                double rgb[3];
//                this->Colors->GetTuple(0, rgb);
//                cout << "r: " << rgb[0] << "\tg: " << rgb[1] << "\tb: " << rgb[2] << endl;
//
//    for (size_t i = 0; i < this->Colors->GetNumberOfTuples(); i++){
//      double rgb[3];
//      this->Colors->GetTuple(i, rgb);
//      cout << "r: " << rgb[0] << "\tg: " << rgb[1] << "\tb: " << rgb[2] << endl;
//    }
//      colorVbo->Upload(input->GetCellData()->GetScalars());
//      colorVbo->Upload(input->GetPointData()->GetScalars());
//  }

  vtkDataArray *normals = input->GetCellData()->GetNormals();
  if (normals)
  {
    normalVbo->SetContext(win);
    normalVbo->UploadNormals(normals);

//    //Create normal IBO
//    normalIndiceVbo->SetContext(win);
//    normalIndiceVbo->SetUsage(vtkVertexBufferObject::DynamicDraw);
//
//    std::vector<unsigned int> indices;
//    for (size_t i=0; i < input->GetPoints()->GetNumberOfPoints(); i+=100){
//      indices.push_back(i);
//    }
//
//    normalIndiceVbo->Upload(&indices[0], indices.size());
  }
}

//----------------------------------------------------------------------------
// Specify the input data or filter.
vtkPolyData *vtkVertexBufferObjectMapper::GetInput()
{
  return vtkPolyData::SafeDownCast(
    this->GetExecutive()->GetInputData(0, 0));
}

void vtkVertexBufferObjectMapper::Update()
{
  cout << "Update" << endl;
  this->Superclass::Update();
}

// Get the bounds for the input of this mapper as
// (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax).
double *vtkVertexBufferObjectMapper::GetBounds()
{
  // do we have an input
  if (!this->GetNumberOfInputConnections(0))
    vtkMath::UninitializeBounds(this->Bounds);
  else
    this->ComputeBounds();

  return this->Bounds;
}

void vtkVertexBufferObjectMapper::ComputeBounds()
{
  this->GetInput()->GetBounds(this->Bounds);
}

//void vtkVertexBufferObjectMapper::PrintSelf(ostream& os, vtkIndent indent)
//{
//  this->Superclass::PrintSelf(os,indent);
//
////  os << indent << "Piece : " << this->Piece << endl;
////  os << indent << "NumberOfPieces : " << this->NumberOfPieces << endl;
////  os << indent << "GhostLevel: " << this->GhostLevel << endl;
////  os << indent << "Number of sub pieces: " << this->NumberOfSubPieces
////     << endl;
//}

//----------------------------------------------------------------------------
int vtkVertexBufferObjectMapper::FillInputPortInformation(
  int vtkNotUsed( port ), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  return 1;
}

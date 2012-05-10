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

#include "vtkExecutive.h"
#include "vtkInformation.h"
#include "vtkMath.h"
#include "vtkgl.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGL.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkShaderProgram2.h"
#include "vtkShader2.h"
#include "vtkShader2Collection.h"
#include "vtkUniformVariables.h"


// Shaders
// ----------------------------------------------------------------------------

// Display BGR data as RGB
const vtkgl::GLchar* RGBVertexShader =
"#version 120\n"

"attribute vec3 color;"
"uniform float alpha;"

"void main(void){\n"
"  gl_Position = ftransform();"
"  gl_FrontColor = vec4(color[0]/255.0f, color[1]/255.0f, color[2]/255.0f, alpha);"
"}\0";

const vtkgl::GLchar* PassthroughFrag = {
"#version 120\n"

"void main(void){"
"   gl_FragColor = gl_Color;"
"}\0"
};

const vtkgl::GLchar* SimpleVertexShader =
"#version 120\n"
"void main(void)\n"
"{\n"
"  gl_FrontColor = gl_Color;\n"
"  gl_Position = ftransform();\n"
"}\n";

const vtkgl::GLchar* SimpleFragmentShader =
"#version 120\n"

"void main(void){"
"   gl_FragColor = gl_Color;"
"}\0";

//----------------------------------------------------------------------------
// Needed when we don't use the vtkStandardNewMacro.
//vtkInstantiatorNewMacro(vtkVertexBufferObjectMapper);
vtkStandardNewMacro(vtkVertexBufferObjectMapper);

//----------------------------------------------------------------------------
// return the correct type of PolyDataMapper
//vtkVertexBufferObjectMapper *vtkVertexBufferObjectMapper::New()
//{
////  // First try to create the object from the vtkObjectFactory
////  vtkObject* ret = vtkGraphicsFactory::CreateInstance("vtkVertexBufferObjectMapper");
////  return static_cast<vtkVertexBufferObjectMapper *>(ret);
//  std::cout << "Creating vtkVertexBufferObjectMapper" << endl;
//  return new vtkVertexBufferObjectMapper;
//
//}

//----------------------------------------------------------------------------
vtkVertexBufferObjectMapper::vtkVertexBufferObjectMapper()
{
  std::cout << "vtkVertexBufferObjectMapper()" << endl;
  initialized = false;
  shadersInitialized = false;

  vertexVbo = vtkVertexBufferObject::New();
  indiceVbo = vtkVertexBufferObject::New();
  colorVbo = vtkVertexBufferObject::New();

}

//void vtkVertexBufferObjectMapper::RenderPiece(vtkRenderer *ren, vtkActor *act)
//{
//  //vtkPolyData *input= this->GetInput();
//
//  // make sure our window is current
//  //ren->GetRenderWindow()->MakeCurrent();
//
//  std::cout << "RenderPiece vtkVertexBufferObjectMapper" << endl;
//}

//bool vtkVertexBufferObjectMapper::Init(vtkRenderWindow* win)
//{
//  vtkOpenGLRenderWindow* renWin = vtkOpenGLRenderWindow::SafeDownCast(win);
//    if (!renWin)
//      return false;
//
//    vtkOpenGLExtensionManager* mgr = renWin->GetExtensionManager();
//
//    bool vbo=mgr->ExtensionSupported("GL_VERSION_1_5") ||
//      mgr->ExtensionSupported("GL_ARB_vertex_buffer_object");
//
//    if(vbo){
//      mgr->LoadExtension("GL_VERSION_1_5");
//      mgr->LoadCorePromotedExtension("GL_ARB_vertex_buffer_object");
//      std::cout << "VBOs are good!" << endl;
//      createShaders(renWin);
//      return vbo;
//    }
//    std::cout << "VBOs are not so good :(" << endl;
//
//    return false;
//}

//----------------------------------------------------------------------------
void vtkVertexBufferObjectMapper::Render(vtkRenderer *ren, vtkActor *act)
{
  ren->GetRenderWindow()->MakeCurrent();

  if (!this->shadersInitialized)
  {
    createShaders(vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()));
    shadersInitialized = true;
  }


  if (!this->initialized)
  {
    std::cout << "Initializing" << endl;
    ren->GetRenderWindow()->MakeCurrent();
    //Init(ren->GetRenderWindow());

    if ( this->LookupTable == NULL )
    {
      this->CreateDefaultLookupTable();
    }

    this->MapScalars(act->GetProperty()->GetOpacity());

    createVBOs(ren->GetRenderWindow());

    initialized = true;
  }

  // get the property
  vtkProperty *prop = act->GetProperty();

  // If full transparency
  if (prop->GetOpacity() <= 0.0)
    return;

//  vtkPolyData *input = this->GetInput();
//  vtkPoints *points = input->GetPoints();

//  vtkCellArray *vertices = input->GetVerts();
//  vtkIdType *ptIds = vertices->GetPointer();
//  cout << "Count: " << vertices->GetNumberOfCells() << endl;
//  cout << "Count? " << *ptIds << endl;
//
//  if (colors->GetName() != NULL)
//    std::cout << "Colors Name: " << colors->GetName() << std::endl;

//  if (colors)
//  {
//  idx |= VTK_PDM_COLORS;
//  if (c->GetName())
//    { // In the future, I will look at the number of components.
//    // All paths will have to handle 3 componet colors.
//    idx |= VTK_PDM_OPAQUE_COLORS;
//    }
//  }


  //vtkCellArray *verts = input->GetVerts();
  //vtkIdType *pIds = verts->GetPointer();
//  unsigned int numPoints = points->GetNumberOfPoints();

  vtkUniformVariables *v=this->program->GetUniformVariables();
  int colorIndex = this->program->GetAttributeLocation("color");

  float alpha = 0.5;
  v->SetUniformf("alpha", 1, &alpha);

  this->program->Use();

  // Bind position and indices
  vertexVbo->Bind();
  indiceVbo->Bind();

  // Position
  vtkgl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  vtkgl::EnableVertexAttribArray(0);

  if (this->Colors)
  {
    colorVbo->Bind();
    vtkgl::VertexAttribPointer(colorIndex, 4, GL_UNSIGNED_BYTE, GL_FALSE, 0, 0);
    vtkgl::EnableVertexAttribArray(colorIndex);
  }

  glDrawElements(GL_POINTS, indiceVbo->GetCount(), GL_UNSIGNED_INT, 0);

  // Unbind VBOs
  vertexVbo->UnBind();
  indiceVbo->UnBind();

  if (this->Colors)
  {
    colorVbo->UnBind();
    vtkgl::DisableVertexAttribArray(colorIndex);
  }

  vtkgl::DisableVertexAttribArray(0);

  this->program->Restore();
}

//----------------------------------------------------------------------------
void vtkVertexBufferObjectMapper::SetInput(vtkPolyData *input)
{
  std::cout << "SetInput" << endl;
  if(input)
  {
    this->SetInputConnection(0, input->GetProducerPort());
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
  std::cout << "SetInput" << endl;
  if(input)
  {
    this->SetInputConnection(0, input->GetProducerPort());
  }
  else
  {
    // Setting a NULL input removes the connection.
    this->SetInputConnection(0, 0);
  }
}

//void vtkVertexBufferObjectMapper::setVertexShader()
//{
//  // The vertex shader
//  vtkShader2 *shader = vtkShader2::New();
//  shader->SetType(VTK_SHADER_TYPE_VERTEX);
//  shader->SetSourceCode(SimpleVertexShader);
//  shader->SetContext(this->program->GetContext());
//  this->program->GetShaders()->AddItem(shader);
//  shader->Delete();
//
//  std::cout << "Created Vertex Shader" << std::endl;
//}

void vtkVertexBufferObjectMapper::createShaders(vtkOpenGLRenderWindow* glContext){

  std::cout << "Creating Shaders" << std::endl;

  // Check if GLSL is supported
  if (!vtkShaderProgram2::IsSupported(glContext))
    {
    vtkErrorMacro("GLSL is not supported on this system.");
    //this->IsCompiled = false;
    return;
    }

  this->program = vtkSmartPointer<vtkShaderProgram2>::New();
  this->program->SetContext(glContext);

  // The vertex shader
  vtkShader2 *shader = vtkShader2::New();
  shader->SetType(VTK_SHADER_TYPE_VERTEX);
  shader->SetSourceCode(RGBVertexShader);
  shader->SetContext(this->program->GetContext());
  this->program->GetShaders()->AddItem(shader);
  shader->Delete();

  std::cout << "Created Vertex Shader" << std::endl;

  // The fragment shader
  shader = vtkShader2::New();
  shader->SetType(VTK_SHADER_TYPE_FRAGMENT);
  shader->SetSourceCode(SimpleFragmentShader);
  shader->SetContext(this->program->GetContext());
  this->program->GetShaders()->AddItem(shader);
  shader->Delete();

  std::cout << "Created Fragment Shader" << std::endl;

  // Build the shader programs
  this->program->Build();
  if(this->program->GetLastBuildStatus() != VTK_SHADER_PROGRAM2_LINK_SUCCEEDED)
    {
    vtkErrorMacro("Couldn't build the shader program. It could be an error in a shader, or a driver bug.");
    return;
    }

  std::cout << "Compiled Program" << std::endl;
}

void vtkVertexBufferObjectMapper::createVBOs(vtkRenderWindow* win){

  std::cout << "Creating VBOs..." << std::endl;
  vtkPolyData *input= this->GetInput();

  //vtkCellArray *verts = input->GetVerts();
  //vtkIdType *pIds = verts->GetPointer();

  // Create vertex buffer
  vertexVbo->SetContext(win);
  vertexVbo->SetData(input->GetPoints());

  // Create index buffer

  //indiceVbo.SetData(input->GetVerts());
  unsigned int numPoints = input->GetPoints()->GetNumberOfPoints();
  std::cout << "Num Points: " << numPoints << std::endl;
  std::vector<unsigned int> *indices = new std::vector<unsigned int>;
  indices->resize(numPoints);
  //unsigned int *indices = new unsigned int[numPoints];
  for (size_t i=0; i < numPoints; i++){
    (*indices)[i] = i;
  }

  indiceVbo->SetContext(win);
  indiceVbo->SetUsage(vtkVertexBufferObject::DynamicDraw);
  indiceVbo->SetData(indices);

  // are they cell or point scalars
  bool cellScalars = false;
  vtkUnsignedCharArray *colors = NULL;

  if ( this->Colors )
  {
//    colors = this->Colors;
    if ( (this->ScalarMode == VTK_SCALAR_MODE_USE_CELL_DATA ||
          this->ScalarMode == VTK_SCALAR_MODE_USE_CELL_FIELD_DATA ||
          this->ScalarMode == VTK_SCALAR_MODE_USE_FIELD_DATA ||
          !input->GetPointData()->GetScalars() )
         && this->ScalarMode != VTK_SCALAR_MODE_USE_POINT_FIELD_DATA)
    {
      cellScalars = true;
    }
  }

  std::cout << "Cell Scalars: " << cellScalars << std::endl;

//  vtkDataArray *scalars = input->GetPointData()->GetScalars();
//  vtkDataArray *scalars = input->GetCellData()->GetScalars();
//  cout << "Count: " << scalars->GetNumberOfTuples() << endl;
  if (this->Colors)
  {
    colorVbo->SetContext(win);
    colorVbo->SetData(this->Colors);
  }

  std:: cout << "\tDONE" << endl;
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

//void vtkVertexBufferObjectMapper::ShallowCopy(vtkAbstractMapper *mapper)
//{
////  vtkVertexBufferObjectMapper *m = vtkVertexBufferObjectMapper::SafeDownCast(mapper);
////  if ( m != NULL )
////    {
////    this->SetInputConnection(m->GetInputConnection(0, 0));
////    this->SetGhostLevel(m->GetGhostLevel());
////    this->SetNumberOfPieces(m->GetNumberOfPieces());
////    this->SetNumberOfSubPieces(m->GetNumberOfSubPieces());
////    }
//
//  // Now do superclass
//  this->vtkMapper::ShallowCopy(mapper);
//}

//void vtkVertexBufferObjectMapper::MapDataArrayToVertexAttribute(
//    const char* vtkNotUsed(vertexAttributeName),
//    const char* vtkNotUsed(dataArrayName),
//    int vtkNotUsed(fieldAssociation),
//    int vtkNotUsed(componentno)
//    )
//{
//  vtkErrorMacro("Not implemented at this level...");
//}
//
//void vtkVertexBufferObjectMapper::MapDataArrayToMultiTextureAttribute(
//    int vtkNotUsed(unit),
//    const char* vtkNotUsed(dataArrayName),
//    int vtkNotUsed(fieldAssociation),
//    int vtkNotUsed(componentno)
//    )
//{
//  vtkErrorMacro("Not implemented at this level...");
//}
//
//
//void vtkVertexBufferObjectMapper::RemoveVertexAttributeMapping(const char* vtkNotUsed(vertexAttributeName))
//{
//  vtkErrorMacro("Not implemented at this level...");
//}
//
//
//void vtkVertexBufferObjectMapper::RemoveAllVertexAttributeMappings()
//{
//  vtkErrorMacro("Not implemented at this level...");
//}


void vtkVertexBufferObjectMapper::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

//  os << indent << "Piece : " << this->Piece << endl;
//  os << indent << "NumberOfPieces : " << this->NumberOfPieces << endl;
//  os << indent << "GhostLevel: " << this->GhostLevel << endl;
//  os << indent << "Number of sub pieces: " << this->NumberOfSubPieces
//     << endl;
}

//----------------------------------------------------------------------------
int vtkVertexBufferObjectMapper::FillInputPortInformation(
  int vtkNotUsed( port ), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  return 1;
}

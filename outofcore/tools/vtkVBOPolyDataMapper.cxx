/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVBOPolyDataMapper.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
//#include "vtkVBOPolyDataMapper.h"

#include <iostream>

#include "vtkVBOPolyDataMapper.h"

#include <vtkExecutive.h>
//#include <vtkGraphicsFactory.h>
#include <vtkInformation.h>
#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkgl.h>
#include <vtkOpenGL.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLExtensionManager.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
//#include <vtkStreamingDemandDrivenPipeline.h>

#include "vtkShaderProgram2.h"
#include "vtkShader2.h"
#include "vtkShader2Collection.h"
#include "vtkUniformVariables.h"


// Shaders
// ----------------------------------------------------------------------------

// Display BGR data as RGB
const GLchar* RGBVertexShader =
"#version 120\n"

"attribute vec3 color;"
"uniform float alpha;"

"void main(void){\n"
"  gl_Position = ftransform();"
"  gl_FrontColor = vec4(color[0]/255.0f, color[1]/255.0f, color[2]/255.0f, alpha);"
"}\0";

const GLchar* PassthroughFrag = {
"#version 120\n"

"void main(void){"
"   gl_FragColor = gl_Color;"
"}\0"
};

const char* SimpleVertexShader =
"#version 120\n"
"void main(void)\n"
"{\n"
"  gl_FrontColor = gl_Color;\n"
"  gl_Position = ftransform();\n"
"}\n";

const char* SimpleFragmentShader =
"#version 120\n"

"void main(void){"
"   gl_FragColor = gl_Color;"
"}\0";

//----------------------------------------------------------------------------
// Needed when we don't use the vtkStandardNewMacro.
vtkInstantiatorNewMacro(vtkVBOPolyDataMapper);
//vtkStandardNewMacro(vtkVBOPolyDataMapper);

//----------------------------------------------------------------------------
// return the correct type of PolyDataMapper
vtkVBOPolyDataMapper *vtkVBOPolyDataMapper::New()
{
//  // First try to create the object from the vtkObjectFactory
//  vtkObject* ret = vtkGraphicsFactory::CreateInstance("vtkVBOPolyDataMapper");
//  return static_cast<vtkVBOPolyDataMapper *>(ret);
  std::cout << "Creating vtkVBOPolyDataMapper" << endl;
  return new vtkVBOPolyDataMapper;

}

//----------------------------------------------------------------------------
vtkVBOPolyDataMapper::vtkVBOPolyDataMapper()
{
  std::cout << "vtkVBOPolyDataMapper()" << endl;
  cloudVertexVBOId = 0;
  cloudColorVBOId = 0;
  cloudIndicesVBOId = 0;
  initialized = false;
//  this->Piece = 0;
//  this->NumberOfPieces = 1;
//  this->NumberOfSubPieces = 1;
//  this->GhostLevel = 0;
}

void vtkVBOPolyDataMapper::RenderPiece(vtkRenderer *ren, vtkActor *act)
{
  //vtkPolyData *input= this->GetInput();

  // make sure our window is current
  //ren->GetRenderWindow()->MakeCurrent();

  std::cout << "RenderPiece vtkVBOPolyDataMapper" << endl;

}

bool vtkVBOPolyDataMapper::Init(vtkRenderWindow* win)
{
  vtkOpenGLRenderWindow* renWin = vtkOpenGLRenderWindow::SafeDownCast(win);
    if (!renWin)
      return false;

    vtkOpenGLExtensionManager* mgr = renWin->GetExtensionManager();

    bool vbo=mgr->ExtensionSupported("GL_VERSION_1_5") ||
      mgr->ExtensionSupported("GL_ARB_vertex_buffer_object");

    if(vbo){
      mgr->LoadExtension("GL_VERSION_1_5");
      mgr->LoadCorePromotedExtension("GL_ARB_vertex_buffer_object");
      std::cout << "VBOs are good!" << endl;
      createShaders(renWin);
      return vbo;
    }
    std::cout << "VBOs are not so good :(" << endl;

    return false;
}

//----------------------------------------------------------------------------
void vtkVBOPolyDataMapper::Render(vtkRenderer *ren, vtkActor *act)
{
  ren->GetRenderWindow()->MakeCurrent();
  if (!this->initialized)
  {
    std::cout << "Initializing" << endl;
    ren->GetRenderWindow()->MakeCurrent();
    Init(ren->GetRenderWindow());

    if ( this->LookupTable == NULL )
    {
    this->CreateDefaultLookupTable();
    }

    this->MapScalars(act->GetProperty()->GetOpacity());

    createVBOs();

    initialized = true;
  }

  vtkPolyData *input= this->GetInput();
  vtkPoints *points = input->GetPoints();
  //vtkCellArray *verts = input->GetVerts();
  //vtkIdType *pIds = verts->GetPointer();
  unsigned int numPoints = points->GetNumberOfPoints();

  vtkUniformVariables *v=this->program->GetUniformVariables();
  int colorIndex = this->program->GetAttributeLocation("color");


  float alpha = 0.5;
  v->SetUniformf("alpha", 1, &alpha);

  this->program->Use();

  // Bind position and indices
  vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, cloudVertexVBOId);
  vtkgl::BindBuffer(vtkgl::ELEMENT_ARRAY_BUFFER, cloudIndicesVBOId);

  // Position
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

  if (this->Colors)
  {
    vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, cloudColorVBOId);
    glVertexAttribPointer(colorIndex, 4, GL_UNSIGNED_BYTE, GL_FALSE, 0, 0);
    //glVertexAttribPointer(colorIndex, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(colorIndex);
  }

  glDrawElements(GL_POINTS, numPoints, GL_UNSIGNED_INT, 0);

  // Unbind VBOs
  vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, 0);
  vtkgl::BindBuffer(vtkgl::ELEMENT_ARRAY_BUFFER, 0);

  glDisableVertexAttribArray(0);

  this->program->Restore();
}

//----------------------------------------------------------------------------
void vtkVBOPolyDataMapper::SetInput(vtkPolyData *input)
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

void vtkVBOPolyDataMapper::createShaders(vtkOpenGLRenderWindow* glContext){

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

  // The vertext shader
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
    //this->IsCompiled = false;
    return;
    }

  std::cout << "Compiled Program" << std::endl;
}

void vtkVBOPolyDataMapper::createVBOs(){

  std:: cout << "Creating VBOs" << endl;
  vtkPolyData *input= this->GetInput();
  vtkPoints *points = input->GetPoints();
//  vtkCellArray *verts = input->GetVerts();
//  vtkIdType *pIds = verts->GetPointer();



  unsigned int numPoints = points->GetNumberOfPoints();

  // Vertex and color vbo
  vtkgl::GenBuffers(1, &cloudVertexVBOId);
  vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, cloudVertexVBOId);
  vtkgl::BufferData(vtkgl::ARRAY_BUFFER,
                       numPoints * 3 * sizeof(float),
                       input->GetPoints()->GetVoidPointer(0),
                       vtkgl::STATIC_DRAW);

  // Unbind vertex and color
  vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, 0);

  std::cout << "Colors: " << this->Colors << std::endl;

  if (this->Colors)
  {
    //vtkUnsignedCharArray *color = this->Colors;
    //unsigned char *colors = this->Colors->GetPointer(0);

    // Colorcolor vbo
    vtkgl::GenBuffers(1, &cloudColorVBOId);
    vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, cloudColorVBOId);
    vtkgl::BufferData(vtkgl::ARRAY_BUFFER,
                         numPoints * 3 * sizeof(float),
                         this->Colors->GetPointer(0),
                         vtkgl::STATIC_DRAW);

    // Unbind vertex and color
    vtkgl::BindBuffer(vtkgl::ARRAY_BUFFER, 0);
  }




  // Indices vbo
  vtkgl::GenBuffers(1, &cloudIndicesVBOId);

  vtkgl::BindBuffer(vtkgl::ELEMENT_ARRAY_BUFFER, cloudIndicesVBOId);

  // The correct way to do this is to use each cells vtkIdTypes, but this
  // returns a tuple (1, index) which I don't have time to understand right
  // now
  //vtkIdType *oldarray = input->GetVerts()->GetPointer();
  //int *newarray = new int[numPoints];
  //std::copy(oldarray, oldarray + numPoints, newarray);
  //for (size_t i=0; i < numPoints*2; i+=2)
  //  std::cout << "really" << oldarray[i] << endl;

  // For now, display every indice
  unsigned int *indices = new unsigned int[numPoints];
  for (size_t i=0; i < numPoints; i++){
    indices[i] = i;
  }

  vtkgl::BufferData(vtkgl::ELEMENT_ARRAY_BUFFER,
                  numPoints * sizeof(unsigned int),
                  indices,
                  vtkgl::DYNAMIC_DRAW);

  // Unbind
  vtkgl::BindBuffer(vtkgl::ELEMENT_ARRAY_BUFFER, 0);
  std:: cout << "Created VBOs" << endl;
}

//----------------------------------------------------------------------------
// Specify the input data or filter.
vtkPolyData *vtkVBOPolyDataMapper::GetInput()
{
  return vtkPolyData::SafeDownCast(
    this->GetExecutive()->GetInputData(0, 0));
}

void vtkVBOPolyDataMapper::Update()
{
  this->Superclass::Update();
}

// Get the bounds for the input of this mapper as
// (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax).
double *vtkVBOPolyDataMapper::GetBounds()
{
  // do we have an input
  if ( ! this->GetNumberOfInputConnections(0))
    {
      vtkMath::UninitializeBounds(this->Bounds);
      return this->Bounds;
    }
  else
    {
//    if (!this->Static)
//      {
//      // For proper clipping, this would be this->Piece, this->NumberOfPieces .
//      // But that removes all benefites of streaming.
//      // Update everything as a hack for paraview streaming.
//      // This should not affect anything else, because no one uses this.
//      // It should also render just the same.
//      // Just remove this lie if we no longer need streaming in paraview :)
//      //this->GetInput()->SetUpdateExtent(0, 1, 0);
//      //this->GetInput()->Update();
//
//      this->Update();
//      }
    this->ComputeBounds();

    // if the bounds indicate NAN and subpieces are being used then
    // return NULL
//    if (!vtkMath::AreBoundsInitialized(this->Bounds)
//        && this->NumberOfSubPieces > 1)
//      {
//      return NULL;
//      }
    return this->Bounds;
    }
}

void vtkVBOPolyDataMapper::ComputeBounds()
{
  this->GetInput()->GetBounds(this->Bounds);
}

void vtkVBOPolyDataMapper::ShallowCopy(vtkAbstractMapper *mapper)
{
  cout << "ShallowCopy" << endl;
//  vtkVBOPolyDataMapper *m = vtkVBOPolyDataMapper::SafeDownCast(mapper);
//  if ( m != NULL )
//    {
//    this->SetInputConnection(m->GetInputConnection(0, 0));
//    this->SetGhostLevel(m->GetGhostLevel());
//    this->SetNumberOfPieces(m->GetNumberOfPieces());
//    this->SetNumberOfSubPieces(m->GetNumberOfSubPieces());
//    }

  // Now do superclass
  this->vtkMapper::ShallowCopy(mapper);
}

void vtkVBOPolyDataMapper::MapDataArrayToVertexAttribute(
    const char* vtkNotUsed(vertexAttributeName),
    const char* vtkNotUsed(dataArrayName),
    int vtkNotUsed(fieldAssociation),
    int vtkNotUsed(componentno)
    )
{
  vtkErrorMacro("Not implemented at this level...");
}

void vtkVBOPolyDataMapper::MapDataArrayToMultiTextureAttribute(
    int vtkNotUsed(unit),
    const char* vtkNotUsed(dataArrayName),
    int vtkNotUsed(fieldAssociation),
    int vtkNotUsed(componentno)
    )
{
  vtkErrorMacro("Not implemented at this level...");
}


void vtkVBOPolyDataMapper::RemoveVertexAttributeMapping(const char* vtkNotUsed(vertexAttributeName))
{
  vtkErrorMacro("Not implemented at this level...");
}


void vtkVBOPolyDataMapper::RemoveAllVertexAttributeMappings()
{
  vtkErrorMacro("Not implemented at this level...");
}


void vtkVBOPolyDataMapper::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

//  os << indent << "Piece : " << this->Piece << endl;
//  os << indent << "NumberOfPieces : " << this->NumberOfPieces << endl;
//  os << indent << "GhostLevel: " << this->GhostLevel << endl;
//  os << indent << "Number of sub pieces: " << this->NumberOfSubPieces
//     << endl;
}

//----------------------------------------------------------------------------
int vtkVBOPolyDataMapper::FillInputPortInformation(
  int vtkNotUsed( port ), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  return 1;
}

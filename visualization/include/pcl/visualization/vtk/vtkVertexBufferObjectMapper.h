/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVertexBufferObjectMapper.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkVertexBufferObjectMapper - map vtkPolyData to graphics primitives
// .SECTION Description
// vtkVertexBufferObjectMapper is a class that maps polygonal data (i.e., vtkPolyData)
// to graphics primitives. vtkVertexBufferObjectMapper serves as a superclass for
// device-specific poly data mappers, that actually do the mapping to the
// rendering/graphics hardware/software.

#pragma once

#include <pcl/pcl_exports.h>
#include <pcl/pcl_macros.h>

#include "vtkMapper.h"
#include "vtkSmartPointer.h"

class vtkOpenGLRenderWindow;
class vtkPolyData;
class vtkRenderer;
class vtkRenderWindow;
class vtkShader2;
class vtkShaderProgram2;
class vtkVertexBufferObject;

PCL_DEPRECATED(1, 13, "The OpenGL backend of VTK is deprecated. Please switch to the OpenGL2 backend.")
class PCL_EXPORTS vtkVertexBufferObjectMapper : public vtkMapper
{
public:
  static vtkVertexBufferObjectMapper *New();
  vtkTypeMacro(vtkVertexBufferObjectMapper, vtkMapper);
//  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Implemented by sub classes. Actual rendering is done here.
//  virtual void RenderPiece(vtkRenderer *ren, vtkActor *act);

  // Description:
  // This calls RenderPiece (in a for loop is streaming is necessary).
  void Render(vtkRenderer *ren, vtkActor *act) override;

  // Description:
  // Specify the input data to map.
  //void SetInputData(vtkPolyData *in);
  void SetInput(vtkPolyData *input);
  void SetInput(vtkDataSet *input);
  vtkPolyData *GetInput();
  
  void SetProgram(vtkSmartPointer<vtkShaderProgram2> program)
  {
    this->program = program;
  }

  // Description:
  // Update that sets the update piece first.
  void Update() override;

  // Description:
  // Return bounding box (array of six doubles) of data expressed as
  // (xmin,xmax, ymin,ymax, zmin,zmax).
  double *GetBounds() override;
  void GetBounds(double bounds[6]) override 
    {this->Superclass::GetBounds(bounds);};
  
  // Description:
  // Make a shallow copy of this mapper.
//  void ShallowCopy(vtkAbstractMapper *m);

  // Description:
  // Select a data array from the point/cell data
  // and map it to a generic vertex attribute. 
  // vertexAttributeName is the name of the vertex attribute.
  // dataArrayName is the name of the data array.
  // fieldAssociation indicates when the data array is a point data array or
  // cell data array (vtkDataObject::FIELD_ASSOCIATION_POINTS or
  // (vtkDataObject::FIELD_ASSOCIATION_CELLS).
  // componentno indicates which component from the data array must be passed as
  // the attribute. If -1, then all components are passed.
//  virtual void MapDataArrayToVertexAttribute(
//    const char* vertexAttributeName,
//    const char* dataArrayName, int fieldAssociation, int componentno=-1);
//
//  virtual void MapDataArrayToMultiTextureAttribute(
//    int unit,
//    const char* dataArrayName, int fieldAssociation, int componentno=-1);

  // Description:
  // Remove a vertex attribute mapping.
//  virtual void RemoveVertexAttributeMapping(const char* vertexAttributeName);
//
//  // Description:
//  // Remove all vertex attributes.
//  virtual void RemoveAllVertexAttributeMappings();

protected:  
  vtkVertexBufferObjectMapper();
  ~vtkVertexBufferObjectMapper() {};

  // Description:
  // Called in GetBounds(). When this method is called, the consider the input
  // to be updated depending on whether this->Static is set or not. This method
  // simply obtains the bounds from the data-object and returns it.
  virtual void ComputeBounds();

  vtkVertexBufferObject *vertexVbo;
  vtkVertexBufferObject *indiceVbo;
  vtkVertexBufferObject *colorVbo;
  vtkVertexBufferObject *normalVbo;
//  vtkVertexBufferObject *normalIndiceVbo;

  vtkSmartPointer<vtkShaderProgram2> program;

  int FillInputPortInformation(int, vtkInformation*) override;

  void createShaders(vtkOpenGLRenderWindow* win);
  void createVBOs(vtkRenderWindow* win);

  bool initialized;
  bool shadersInitialized;

private:
  vtkVertexBufferObjectMapper(const vtkVertexBufferObjectMapper&);  // Not implemented.
  void operator=(const vtkVertexBufferObjectMapper&);  // Not implemented.
};

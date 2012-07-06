/*========================================================================
  VES --- VTK OpenGL ES Rendering Toolkit

      http://www.kitware.com/ves

  Copyright 2011 Kitware, Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
 ========================================================================*/

#include "vesKiwiPCLDemo.h"
#include "vesRenderer.h"
#include "vesCamera.h"
#include "vesMapper.h"
#include "vesGeometryData.h"
#include "vesActor.h"
#include "vesShaderProgram.h"
#include "vesKiwiDataConversionTools.h"
#include "vesKiwiPolyDataRepresentation.h"

#include "vtkPCLConversions.h"
#include "vtkPCLSACSegmentationPlane.h"
#include "vtkPCLVoxelGrid.h"

#include <vtkPolyData.h>
#include <vtkTimerLog.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkLookupTable.h>

#include <vector>
#include <cassert>
#include <sstream>

//----------------------------------------------------------------------------
class vesKiwiPCLDemo::vesInternal
{
public:

  vesInternal()
  {
    this->LeafSize = 0.05;
    this->PlaneDistanceThreshold = 0.0;
  }

  ~vesInternal()
  {
  }

  double LeafSize;
  double PlaneDistanceThreshold;
  vtkSmartPointer<vtkPolyData> PolyData;
  vesKiwiPolyDataRepresentation::Ptr PolyDataRep;
  vesShaderProgram::Ptr GeometryShader;
};

//----------------------------------------------------------------------------
vesKiwiPCLDemo::vesKiwiPCLDemo()
{
  this->Internal = new vesInternal();
}

//----------------------------------------------------------------------------
vesKiwiPCLDemo::~vesKiwiPCLDemo()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------
vesKiwiPolyDataRepresentation::Ptr vesKiwiPCLDemo::cloudRepresentation()
{
  return this->Internal->PolyDataRep;
}

//----------------------------------------------------------------------------
void vesKiwiPCLDemo::setLeafSize(double value)
{
  if (this->Internal->LeafSize != value) {
    this->Internal->LeafSize = value;
    this->Internal->PolyDataRep->mapper()->setGeometryData(this->updateGeometryData());
  }
}

//----------------------------------------------------------------------------
void vesKiwiPCLDemo::setPlaneDistanceThreshold(double value)
{
  if (this->Internal->PlaneDistanceThreshold != value) {
    this->Internal->PlaneDistanceThreshold = value;
    this->Internal->PolyDataRep->mapper()->setGeometryData(this->updateGeometryData());
  }
}

//----------------------------------------------------------------------------
void vesKiwiPCLDemo::initialize(const std::string& filename, vesSharedPtr<vesShaderProgram> shader)
{
  this->Internal->PolyData = vtkPCLConversions::PolyDataFromPCDFile(filename);

  this->Internal->GeometryShader = shader;
  this->Internal->PolyDataRep = vesKiwiPolyDataRepresentation::Ptr(new vesKiwiPolyDataRepresentation);
  this->Internal->PolyDataRep->initializeWithShader(this->Internal->GeometryShader);
  this->Internal->PolyDataRep->mapper()->setGeometryData(this->updateGeometryData());
}

namespace {

void ConvertVertexArrays(vtkDataSet* dataSet, vesSharedPtr<vesGeometryData> geometryData, vtkScalarsToColors* scalarsToColors=NULL)
{
  vtkUnsignedCharArray* colors = vesKiwiDataConversionTools::FindRGBColorsArray(dataSet);
  vtkDataArray* scalars = vesKiwiDataConversionTools::FindScalarsArray(dataSet);
  vtkDataArray* tcoords = vesKiwiDataConversionTools::FindTextureCoordinatesArray(dataSet);
  if (colors)
    {
    vesKiwiDataConversionTools::SetVertexColors(colors, geometryData);
    }
  else if (scalars)
    {
    vtkSmartPointer<vtkScalarsToColors> colorMap = scalarsToColors;
    if (!colorMap)
      {
      colorMap = vesKiwiDataConversionTools::GetRedToBlueLookupTable(scalars->GetRange());
      }
    vesKiwiDataConversionTools::SetVertexColors(scalars, colorMap, geometryData);
    }
  else if (tcoords)
    {
    vesKiwiDataConversionTools::SetTextureCoordinates(tcoords, geometryData);
    }
}

}


//----------------------------------------------------------------------------
vesGeometryData::Ptr vesKiwiPCLDemo::updateGeometryData()
{
  if (!this->Internal->PolyData) {
    return vesGeometryData::Ptr(new vesGeometryData);
  }

  vtkSmartPointer<vtkPolyData> polyData = this->Internal->PolyData;

  if (this->Internal->LeafSize != 0.0) {
    vtkSmartPointer<vtkPCLVoxelGrid> voxelGrid = vtkSmartPointer<vtkPCLVoxelGrid>::New();
    voxelGrid->SetInputData(this->Internal->PolyData);
    voxelGrid->SetLeafSize(this->Internal->LeafSize, this->Internal->LeafSize, this->Internal->LeafSize);
    voxelGrid->Update();
    polyData = voxelGrid->GetOutput();
  }

  if (this->Internal->PlaneDistanceThreshold != 0.0) {
    vtkSmartPointer<vtkPCLSACSegmentationPlane> fitPlane = vtkSmartPointer<vtkPCLSACSegmentationPlane>::New();
    fitPlane->SetInputData(polyData);
    fitPlane->SetMaxIterations(200);
    fitPlane->SetDistanceThreshold(this->Internal->PlaneDistanceThreshold);
    fitPlane->Update();
    polyData = fitPlane->GetOutput();
    polyData->GetPointData()->RemoveArray("rgb_colors");
  }

  vesGeometryData::Ptr geometryData = vesKiwiDataConversionTools::ConvertPoints(polyData);

  ConvertVertexArrays(polyData, geometryData);

  return geometryData;

}

//----------------------------------------------------------------------------
bool vesKiwiPCLDemo::handleSingleTouchDown(int displayX, int displayY)
{
  vesNotUsed(displayX);
  vesNotUsed(displayY);
  return false;
}

//----------------------------------------------------------------------------
bool vesKiwiPCLDemo::handleSingleTouchUp()
{
  return false;
}

//----------------------------------------------------------------------------
bool vesKiwiPCLDemo::handleSingleTouchTap(int displayX, int displayY)
{
  vesNotUsed(displayX);
  vesNotUsed(displayY);

  return false;
}

//----------------------------------------------------------------------------
bool vesKiwiPCLDemo::handleSingleTouchPanGesture(double deltaX, double deltaY)
{
  vesNotUsed(deltaX);
  vesNotUsed(deltaY);
  return false;
}

//----------------------------------------------------------------------------
void vesKiwiPCLDemo::willRender(vesSharedPtr<vesRenderer> renderer)
{
  vesNotUsed(renderer);
}

//----------------------------------------------------------------------------
void vesKiwiPCLDemo::addSelfToRenderer(vesSharedPtr<vesRenderer> renderer)
{
  this->Superclass::addSelfToRenderer(renderer);
  this->Internal->PolyDataRep->addSelfToRenderer(renderer);
}

//----------------------------------------------------------------------------
void vesKiwiPCLDemo::removeSelfFromRenderer(vesSharedPtr<vesRenderer> renderer)
{
  this->Superclass::removeSelfFromRenderer(renderer);
  if (this->Internal->PolyDataRep) {
    this->Internal->PolyDataRep->removeSelfFromRenderer(renderer);
  }
}

//----------------------------------------------------------------------------
int vesKiwiPCLDemo::numberOfFacets()
{
  if (this->Internal->PolyDataRep) {
    return this->Internal->PolyDataRep->numberOfFacets();
  }
  return 0;
}

//----------------------------------------------------------------------------
int vesKiwiPCLDemo::numberOfVertices()
{
  if (this->Internal->PolyDataRep) {
    return this->Internal->PolyDataRep->numberOfVertices();
  }
  return 0;
}

//----------------------------------------------------------------------------
int vesKiwiPCLDemo::numberOfLines()
{
  if (this->Internal->PolyDataRep) {
    return this->Internal->PolyDataRep->numberOfLines();
  }
  return 0;
}

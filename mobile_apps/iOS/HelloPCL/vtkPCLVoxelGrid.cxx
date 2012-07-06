/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLVoxelGrid.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLVoxelGrid.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"
#include "vtkAlgorithmOutput.h"

#include <pcl/filters/voxel_grid.h>

//----------------------------------------------------------------------------
namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr ApplyVoxelGrid(
                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                      double leafSize[3])
{
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  voxelGrid.setInputCloud(cloud);
  voxelGrid.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
  voxelGrid.filter(*cloudFiltered);
  return cloudFiltered;
}

}

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLVoxelGrid);

//----------------------------------------------------------------------------
vtkPCLVoxelGrid::vtkPCLVoxelGrid()
{
  this->LeafSize[0] = 0.01;
  this->LeafSize[1] = 0.01;
  this->LeafSize[2] = 0.01;
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLVoxelGrid::~vtkPCLVoxelGrid()
{
}

//----------------------------------------------------------------------------
int vtkPCLVoxelGrid::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = vtkPCLConversions::PointCloudFromPolyData(input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered = ApplyVoxelGrid(cloud, this->LeafSize);

  output->ShallowCopy(vtkPCLConversions::PolyDataFromPointCloud(cloudFiltered));
  return 1;
}

//----------------------------------------------------------------------------
void vtkPCLVoxelGrid::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

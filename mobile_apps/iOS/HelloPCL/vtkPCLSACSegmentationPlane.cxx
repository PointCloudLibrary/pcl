/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationPlane.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLSACSegmentationPlane.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPlane.h"
#include "vtkNew.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//----------------------------------------------------------------------------
namespace {

void ComputeSACSegmentationLine(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  int maxIterations,
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *modelCoefficients);
}

void ComputeSACSegmentationPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  int maxIterations,
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
{

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *modelCoefficients);
}

void ComputeSACSegmentationPerpendicularPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  const Eigen::Vector3f& perpendicularAxis,
                                  double angleEpsilon,
                                  int maxIterations,
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
{

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setAxis(perpendicularAxis);
  seg.setEpsAngle(angleEpsilon);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *modelCoefficients);
}

}


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLSACSegmentationPlane);

//----------------------------------------------------------------------------
vtkPCLSACSegmentationPlane::vtkPCLSACSegmentationPlane()
{
  this->DistanceThreshold = 0.05;
  this->MaxIterations = 200;

  this->PlaneCoefficients[0] = 0.0;
  this->PlaneCoefficients[1] = 0.0;
  this->PlaneCoefficients[2] = 0.0;
  this->PlaneCoefficients[3] = 0.0;

  this->PerpendicularConstraintEnabled = false;
  this->AngleEpsilon = 0.2;
  this->PerpendicularAxis[0] = 1.0;
  this->PerpendicularAxis[1] = 0.0;
  this->PerpendicularAxis[2] = 0.0;

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLSACSegmentationPlane::~vtkPCLSACSegmentationPlane()
{
}

//----------------------------------------------------------------------------
int vtkPCLSACSegmentationPlane::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get input and output data objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // perform plane model fit
  pcl::PointIndices::Ptr inlierIndices;
  pcl::ModelCoefficients::Ptr modelCoefficients;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = vtkPCLConversions::PointCloudFromPolyData(input);

  if (this->PerpendicularConstraintEnabled)
    {
    ComputeSACSegmentationPerpendicularPlane(
                                cloud,
                                this->DistanceThreshold,
                                Eigen::Vector3f(this->PerpendicularAxis[0],
                                                this->PerpendicularAxis[1],
                                                this->PerpendicularAxis[2]),
                                this->AngleEpsilon,
                                this->MaxIterations,
                                modelCoefficients,
                                inlierIndices);
    }
  else
    {
    ComputeSACSegmentationPlane(cloud,
                                this->DistanceThreshold,
                                this->MaxIterations,
                                modelCoefficients,
                                inlierIndices);
    }

  // store plane coefficients
  for (size_t i = 0; i < modelCoefficients->values.size(); ++i)
    {
    this->PlaneCoefficients[i] = modelCoefficients->values[i];
    }

  // compute origin and normal
  Eigen::Vector4d coeffs(this->PlaneCoefficients); 
  if (coeffs[2] < 0)
    {
    coeffs *= -1.0;
    }
  Eigen::Vector3d normal(coeffs.data());

  vtkNew<vtkPlane> plane;
  plane->SetNormal(normal.data());
  plane->Push(-coeffs[3]/normal.norm());
  plane->GetOrigin(this->PlaneOrigin);
  plane->GetNormal(this->PlaneNormal);


  // pass thru input add labels
  vtkSmartPointer<vtkIntArray> labels = vtkPCLConversions::NewLabelsArray(inlierIndices, input->GetNumberOfPoints());
  labels->SetName("ransac_labels");
  output->ShallowCopy(input);
  output->GetPointData()->AddArray(labels);

  return 1;
}

//----------------------------------------------------------------------------
void vtkPCLSACSegmentationPlane::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

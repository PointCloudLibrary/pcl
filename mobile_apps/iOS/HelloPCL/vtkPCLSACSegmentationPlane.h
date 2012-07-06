/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationPlane.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLSACSegmentationPlane -
// .SECTION Description
//

#ifndef __vtkPCLSACSegmentationPlane_h
#define __vtkPCLSACSegmentationPlane_h

#include <vtkPolyDataAlgorithm.h>


class vtkPCLSACSegmentationPlane : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLSACSegmentationPlane, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkPCLSACSegmentationPlane *New();

  vtkSetMacro(DistanceThreshold, double);
  vtkGetMacro(DistanceThreshold, double);

  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkGetVector4Macro(PlaneCoefficients, double);
  vtkGetVector3Macro(PlaneOrigin, double);
  vtkGetVector3Macro(PlaneNormal, double);

  vtkSetMacro(PerpendicularConstraintEnabled, bool);
  vtkGetMacro(PerpendicularConstraintEnabled, bool);

  vtkSetMacro(AngleEpsilon, double);
  vtkGetMacro(AngleEpsilon, double);

  vtkGetVector3Macro(PerpendicularAxis, double);
  vtkSetVector3Macro(PerpendicularAxis, double);

protected:

  double DistanceThreshold;
  int MaxIterations;

  bool PerpendicularConstraintEnabled;
  double PerpendicularAxis[3];
  double AngleEpsilon;

  double PlaneCoefficients[4];
  double PlaneOrigin[3];
  double PlaneNormal[3];

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);


  vtkPCLSACSegmentationPlane();
  virtual ~vtkPCLSACSegmentationPlane();

private:
  vtkPCLSACSegmentationPlane(const vtkPCLSACSegmentationPlane&);  // Not implemented.
  void operator=(const vtkPCLSACSegmentationPlane&);  // Not implemented.
};

#endif



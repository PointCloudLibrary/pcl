/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

// .NAME vtkPTXWriter - write a Leica PTX file.
// .SECTION Description

#ifndef __vtkPTXWriter_h
#define __vtkPTXWriter_h

#include "vtkPolyDataAlgorithm.h"

class vtkPTXWriter : public vtkPolyDataAlgorithm
{
public:
  static vtkPTXWriter *New();
  vtkTypeMacro(vtkPTXWriter,vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Specify the name of the file to write out.
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

  vtkSetMacro(NumberOfThetaPoints, unsigned int);
  vtkGetMacro(NumberOfThetaPoints, unsigned int);

  vtkSetMacro(NumberOfPhiPoints, unsigned int);
  vtkGetMacro(NumberOfPhiPoints, unsigned int);

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

protected:
  vtkPTXWriter();
  ~vtkPTXWriter();

private:
  vtkPTXWriter(const vtkPTXWriter&);  // Not implemented.
  void operator=(const vtkPTXWriter&);  // Not implemented.

  char *FileName;
  unsigned int NumberOfThetaPoints;
  unsigned int NumberOfPhiPoints;

};

#endif

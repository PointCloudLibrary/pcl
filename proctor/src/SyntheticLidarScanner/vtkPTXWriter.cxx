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

#include "vtkPTXWriter.h"

#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"

#include <fstream>

vtkStandardNewMacro(vtkPTXWriter);

vtkPTXWriter::vtkPTXWriter()
{
  this->FileName = NULL;

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(0);

  this->NumberOfThetaPoints = 0;
  this->NumberOfPhiPoints = 0;
}

vtkPTXWriter::~vtkPTXWriter()
{
  if ( this->FileName )
    {
    delete [] this->FileName;
    }
}

int vtkPTXWriter::RequestData(vtkInformation *vtkNotUsed(request),
                               vtkInformationVector **inputVector,
                               vtkInformationVector *vtkNotUsed(outputVector))
{

  // Get the input
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));

  // Open the file for writing
  std::ofstream fout(this->FileName, ios::out);

  fout << this->NumberOfPhiPoints << std::endl
    << this->NumberOfThetaPoints << std::endl
    << "0 0 0" << std::endl
    << "1 0 0" << std::endl
    << "0 1 0" << std::endl
    << "0 0 1" << std::endl
    << "1 0 0 0" << std::endl
    << "0 1 0 0" << std::endl
    << "0 0 1 0" << std::endl
    << "0 0 0 1" << std::endl;

  vtkIntArray* valid = vtkIntArray::SafeDownCast(input->GetPointData()->GetArray("Valid"));

  /*
  // This demonstrates the correct order
  for(unsigned int thetaCounter = 0; thetaCounter < NumberOfThetaPoints; thetaCounter++)
    {
      for(unsigned int phiCounter = 0; phiCounter < NumberOfPhiPoints; phiCounter++)
        {
        // If the ray in this grid location had a valid scene intersection
        if(this->OutputGrid->GetValue(phiCounter, thetaCounter)->GetHit())
          {
          vtkLidarPoint* LP = OutputGrid->GetValue(phiCounter, thetaCounter);
          double* coord = LP->GetCoordinate();
          fout << coord[0] << " " << coord[1] << " " << coord[2] << " .5 0 0 0" << endl;
          }
        else
          {
          fout << "0 0 0 0 0 0 0" << endl;
          }

        }//end phi for loop
    }//end theta for loop
  */

    for(unsigned int i = 0; i < this->NumberOfThetaPoints * this->NumberOfPhiPoints; i++)
      {
      // If the ray in this grid location had a valid scene intersection
      if(valid->GetValue(i))
        {
        double p[3];
        input->GetPoint(i,p);
        fout << p[0] << " " << p[1] << " " << p[2] << " .5 0 0 0" << endl;
        }
      else
        {
        fout << "0 0 0 0 0 0 0" << endl;
        }
      }//end theta for loop

  fout.close();

  return 1;
}


void vtkPTXWriter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  if (this->FileName)
    {
    os << indent << "FileName: " << this->FileName << "\n";
    }
  else
    {
    os << indent << "FileName: (null)\n";
    }
}

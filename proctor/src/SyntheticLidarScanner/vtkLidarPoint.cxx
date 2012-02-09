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

#include "vtkLidarPoint.h"
#include "vtkRay.h"

#include "vtkSmartPointer.h"
#include "vtkObjectFactory.h" //for new() macro

vtkStandardNewMacro(vtkLidarPoint);

vtkLidarPoint::vtkLidarPoint()
{
	// Until specified, this LidarPoint is invalid
  this->Hit = false;
  this->Ray = NULL;
}

vtkLidarPoint::~vtkLidarPoint() 
{
  if(this->Ray)
    {
    this->Ray->Delete();
    }
}

vtkCxxSetObjectMacro(vtkLidarPoint, Ray, vtkRay);

void vtkLidarPoint::GetNormal(double n[3])
{
  for(unsigned int i = 0; i < 3; i++)
    {
    n[i] = this->Normal[i];
    }
}

double* vtkLidarPoint::GetNormal()
{
	// There is only a normal if this is a valid LidarPoint
	if(this->Hit)
    {
		return Normal;
    }
	else
    {
		return NULL;
    }
}

void vtkLidarPoint::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
	//if the LidarPoint is valid, print its information
	if(this->Hit)
	  {
		os << "Coordinate: " << Coordinate[0] << " " << Coordinate[1] << " " << Coordinate[2] << vtkstd::endl
			<< "Ray: " << *Ray << vtkstd::endl
			<< "Normal: " << Normal[0] << " " << Normal[1] << " " << Normal[2] << vtkstd::endl;
	  }
	else
	  {
		os << "Invalid!" << vtkstd::endl;
	  }

}


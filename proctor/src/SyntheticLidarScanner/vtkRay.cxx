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

#include "vtkRay.h"

#include "vtkObjectFactory.h" //for new() macro
#include "vtkMath.h"
#include "vtkTransform.h"
#include "vtkSmartPointer.h"

vtkStandardNewMacro(vtkRay);

void vtkRay::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
  // Print the rays origin and direction when << is called
  os << "Origin: " << this->Origin[0] << " " << this->Origin[1] << " " << this->Origin[2] << vtkstd::endl
     << "Direction: " << this->Direction[0] << " " << this->Direction[1] << " " << this->Direction[2] << vtkstd::endl;
	
}

double* vtkRay::GetPointAlong(const double dist)
{
	// Return a point 'dist' units along the ray in the "forward" direction
	double* newPoint = new double[3];
  newPoint[0] = this->Origin[0] + this->Direction[0] * dist;
  newPoint[1] = this->Origin[1] + this->Direction[1] * dist;
  newPoint[2] = this->Origin[2] + this->Direction[2] * dist;
	return newPoint;
}

void vtkRay::SetDirection(double* dir)
{
	// Set the rays direction to the unit length Dir
	vtkMath::Normalize(dir);
  this->Direction[0] = dir[0];
  this->Direction[1] = dir[1];
  this->Direction[2] = dir[2];
}

bool vtkRay::IsInfront(double* P)
{
	// Create a vector (OtherRay) from the rays origin to the query point P
  vtkSmartPointer<vtkRay> otherRay = vtkSmartPointer<vtkRay>::New();
	otherRay->SetOrigin(this->Origin);
	double dir[3] = {P[0] - this->Origin[0], P[1] - this->Origin[1], P[2] - this->Origin[2]};
	otherRay->SetDirection(dir);
		
	//if the dot product between the above computed direction and the rays direction is greater than
	//zero, the query point is "in front of" the ray
	double dotprod = vtkMath::Dot(this->Direction, otherRay->GetDirection());
	
	if(dotprod > 0.0)
    {
		return true;
    }
	else
    {
		return false;
    }
  
}

void vtkRay::ApplyTransform(vtkTransform* trans)
{
	//transform the rays origin and a point 1 unit along the ray (p)
	//store the direction from the transformed origin and p as the rays new direction
	
	double* p = this->GetPointAlong(1.0);
	trans->TransformPoint(p, p);
	trans->TransformPoint(this->Origin, this->Origin);
	
	this->Direction[0] = p[0] - this->Origin[0];
	this->Direction[1] = p[1] - this->Origin[1];
	this->Direction[2] = p[2] - this->Origin[2];

}

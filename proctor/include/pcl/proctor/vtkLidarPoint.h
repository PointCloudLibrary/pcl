#ifndef __vtkLidarPoint_h
#define __vtkLidarPoint_h

#include "vtkObject.h" //super class
class vtkRay;

/*
A LidarPoint is a container to hold an intersection coordinate,
the ray that was cast to obtain this intersection, a flag to indicate if the
point is a valid scene intersection, and the normal of the triangle
of the scene that was intersected by this ray
*/

class vtkLidarPoint : public vtkObject
{
public:
  static vtkLidarPoint *New();
  vtkTypeRevisionMacro(vtkLidarPoint,vtkObject);
  void PrintSelf(std::ostream &os, vtkIndent indent);

  ////////// Accessors ///////////
  vtkGetVector3Macro(Coordinate,double); //get the coordinate of the intersection
  vtkGetObjectMacro(Ray, vtkRay); //get the ray that was/will be cast into the scene
  double* GetNormal(); // get the normal of the ray/scene intersection
  void GetNormal(double n[3]); // get the normal of the ray/scene intersection
  vtkGetMacro(Hit, bool); //see if this is a valid point (if there was a valid scene intersection)

  /////////// Mutators ///////////
  vtkSetVector3Macro(Coordinate,double); //set the 3d coordinate of the intersection of this point's ray with the scene
  void SetRay(vtkRay* ray);// set the ray that was/will be cast into the scene
  vtkSetVector3Macro(Normal,double); //save the normal of the scene's triangle that the ray intersected
  vtkSetMacro(Hit,bool); //set whether or not the LidarPoint is valid
		
protected:

  vtkLidarPoint();
  ~vtkLidarPoint();
private:
  vtkLidarPoint(const vtkLidarPoint&); //not implemented
  void operator=(const vtkLidarPoint&); //not implemented

  double Coordinate[3];//(x,y,z) coords of point in scanner centered coordinates

  vtkRay* Ray; //the direction and location from which the point was scanned

  bool Hit; //hit or miss? (i.e. is the point valid?)
  double Normal[3]; //the normal of the triangle that was intersected

};

#endif
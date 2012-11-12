// C++
#include <iostream>
#include <string>

// PCL - visualziation
#include <pcl/visualization/common/common.h>

// PCL - outofcore
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/object.h>

// VTK
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkHull.h>
#include <vtkPlanes.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

// Operators
// -----------------------------------------------------------------------------
Camera::Camera (std::string name) :
    Object (name), display_ (false)
{
  camera_ = vtkSmartPointer<vtkCamera>::New ();

  camera_actor_ = vtkSmartPointer<vtkCameraActor>::New ();
  camera_actor_->SetCamera (camera_);
  camera_actor_->GetProperty ()->SetLighting (0);
  camera_actor_->GetProperty ()->SetLineStipplePattern (0101010101010101);

  for (int i = 0; i < 24; i++)
    frustum_[i] = 0;

  hull_actor_ = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkPolyDataMapper> hull_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();

  hull_actor_->SetMapper (hull_mapper);
  hull_actor_->GetProperty ()->SetLighting (0);
  hull_actor_->GetProperty ()->SetColor (1.0, 0.0, 0.0);
  hull_actor_->GetProperty ()->SetOpacity (0.25);
}

Camera::Camera (std::string name, vtkSmartPointer<vtkCamera> camera) :
    Object (name), display_ (false)
{
  camera_ = camera;

  camera_actor_ = vtkSmartPointer<vtkCameraActor>::New ();
  camera_actor_->SetCamera (camera_);
  camera_actor_->GetProperty ()->SetLighting (0);

  for (int i = 0; i < 24; i++)
    frustum_[i] = 0;

  hull_actor_ = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkPolyDataMapper> hull_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();

  hull_actor_->SetMapper (hull_mapper);
  hull_actor_->GetProperty ()->SetLighting (0);
  hull_actor_->GetProperty ()->SetColor (1.0, 0.0, 0.0);
  hull_actor_->GetProperty ()->SetOpacity (0.25);
}

//std::ostream & operator<<(std::ostream &os, const Camera& camera)
//{
//    return os << camera.getName();
//}

// Methods
// -----------------------------------------------------------------------------
void
//Camera::computeFrustum(double aspect)
Camera::computeFrustum ()
{
  // The planes array contains six plane equations of the form (Ax+By+Cz+D=0), the first four values are (A,B,C,D)
  // which repeats for each of the planes. The planes are given in the following order: -x,+x,-y,+y,-z,+z.
  //camera_->GetFrustumPlanes(aspect, frustum_);

  pcl::visualization::getViewFrustum (getViewProjectionMatrix (), frustum_);

  vtkSmartPointer<vtkHull> hull = vtkSmartPointer<vtkHull>::New ();
  vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New ();
  vtkSmartPointer<vtkPolyData> hullData = vtkSmartPointer<vtkPolyData>::New ();

  planes->SetFrustumPlanes (frustum_);
  hull->SetPlanes (planes);
  hull->GenerateHull (hullData, -200, 200, -200, 200, -200, 200);

  vtkSmartPointer<vtkPolyDataMapper> hull_mapper = static_cast<vtkPolyDataMapper*> (hull_actor_->GetMapper ());

#if VTK_MAJOR_VERSION <= 5
  hull_mapper->SetInput (hullData);
#else
  hull_mapper->SetInputData(hullData);
#endif

  hull_actor_->SetMapper (hull_mapper);
}

void
Camera::printFrustum ()
{
  for (int i = 0; i < 6; i++)
  {
    std::cout << frustum_[(i * 4)] << "x + " << frustum_[(i * 4) + 1] << "y + " << frustum_[(i * 4) + 2] << "z + "
        << frustum_[(i * 4) + 3] << std::endl;
  }
}

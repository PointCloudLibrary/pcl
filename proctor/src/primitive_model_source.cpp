#include "proctor/primitive_model_source.h"

#include <pcl/pcl_base.h>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>

#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkDiskSource.h>
#include <vtkSmartPointer.h>

namespace pcl
{

  namespace proctor
  {

    void
    PrimitiveModelSource::addDefaultModel(std::string name, vtkAlgorithmOutput *mesh)
    {
      Model model;
      model.id = name_ + "_" + name;
      model.mesh = mesh;
      model.cx = 0;
      model.cy = 0;
      model.cz = 0;
      model.scale = 1;

      models_[model.id] = model;
    }

    void
    PrimitiveModelSource::loadModels()
    {
      // TODO There is a memory leak here
      // Sphere
      vtkSphereSource *sphere_source = vtkSphereSource::New();
      sphere_source->SetRadius(0.5);
      sphere_source->SetThetaResolution(50); 
      sphere_source->SetPhiResolution(50);
      sphere_source->SetCenter(0,0,0);
      sphere_source->Update();

      addDefaultModel("sphere", sphere_source->GetOutputPort());

      // Cube
      vtkCubeSource *cube_source = vtkCubeSource::New();
      cube_source->SetXLength(0.5);
      cube_source->SetYLength(0.5);
      cube_source->SetZLength(0.5);
      cube_source->SetCenter(0,0,0);
      cube_source->Update();

      addDefaultModel("cube", cube_source->GetOutputPort());

      // Cylinder
      vtkCylinderSource *cylinder_source = vtkCylinderSource::New();
      cylinder_source->Update();

      addDefaultModel("cylinder", cylinder_source->GetOutputPort());

      // Cone
      vtkConeSource *cone_source = vtkConeSource::New();
      cone_source->Update();

      addDefaultModel("cone", cone_source->GetOutputPort());

      // TODO Some scans are empty, which causes errors
      //vtkDiskSource *disk_source = vtkDiskSource::New();
      //disk_source->Update();

      //addDefaultModel("disk", disk_source->GetOutputPort());
    }

  }
}

#include <pcl/apps/cloud_composer/point_selectors/manipulation_event.h>

pcl::cloud_composer::ManipulationEvent::~ManipulationEvent ()
{
  //TODO Delete manipulated actor here?
  
}

void
pcl::cloud_composer::ManipulationEvent::addManipulation (QString id, vtkSmartPointer<vtkMatrix4x4> start, vtkSmartPointer<vtkMatrix4x4> end)
{
  id_start_map_.insert (id, start);
  id_end_map_.insert (id, end);
  
}
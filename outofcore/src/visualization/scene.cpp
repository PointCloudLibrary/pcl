// PCL
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/viewport.h>

Scene* Scene::instance_ = NULL;

Scene::Scene ()
{

}

// Accessors - Cameras
// -----------------------------------------------------------------------------
void
Scene::addCamera (Camera *camera)
{
  cameras_.push_back (camera);
}

std::vector<Camera*>
Scene::getCameras ()
{
  return cameras_;
}

Camera*
Scene::getCamera (vtkCamera *camera)
{
  for (size_t i = 0; i < cameras_.size (); i++)
  {
    if (cameras_[i]->getCamera ().GetPointer () == camera)
    {
      return cameras_[i];
    }
  }

  return NULL;
}

Camera*
Scene::getCamera (std::string name)
{
  for (size_t i = 0; i < cameras_.size (); i++)
    if (cameras_[i]->getName () == name)
      return cameras_[i];

  return NULL;
}

// Accessors - Objects
// -----------------------------------------------------------------------------
void
Scene::addObject (Object *object)
{
  objects_.push_back (object);
}

Object*
Scene::getObjectByName (std::string name)
{
  for (size_t i = 0; i < objects_.size (); i++)
    if (objects_[i]->getName () == name)
      return objects_[i];

  return NULL;
}

std::vector<Object*>
Scene::getObjects ()
{
  return objects_;
}

//  void removeObject(Object *object)
//  {
//
//  }

// Accessors - Viewports
// -----------------------------------------------------------------------------

void
Scene::addViewport (Viewport *viewport)
{
  viewports_.push_back (viewport);
}

std::vector<Viewport*>
Scene::getViewports ()
{
  return viewports_;
}
